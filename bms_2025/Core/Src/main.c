/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_uart.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_spi.h"
#include "stm32f4xx_hal_tim.h"
#include "stm32f4xx_hal_rcc.h"
#include "stm32f4xx_hal_cortex.h"
#include "stm32f4xx_hal_pwr.h"

#include "BMSconfig.h"
#include "Fault.h"
#include "LTC6811.h"
#include "PackCalculations.h"
#include "SPI.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define NUM_USED_CELLS               6U
#define NUM_USED_THERMISTORS         2U
#define VOLTAGE_POLL_PERIOD_MS       200U
#define TEMP_POLL_PERIOD_MS          500U
#define HEARTBEAT_PERIOD_MS          250U

#define DEFAULT_CELL_OV_THRESHOLD    42000U
#define DEFAULT_CELL_UV_THRESHOLD    30000U
#define DEFAULT_BALANCE_THRESHOLD    41500U
#define DEFAULT_MAX_DISCHARGE_CELLS  1U
#define DEFAULT_CELL_IMBALANCE_LIMIT 150U

#define UART_TX_PERIOD_MS            200U
#define UART_BMS_PACKET_SOF1         0xAAU
#define UART_BMS_PACKET_SOF2         0x55U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
CellData bmsData[NUM_CELLS];
uint8_t BMS_STATUS[6];
bool discharge[NUM_BOARDS][12];

volatile bool bmsFault = false;

BMS_critical_info_t BMSCriticalInfo;
static BMSConfigStructTypedef BMSConfig;
UART_HandleTypeDef huart2;

// Global variables for interrupt handlers and SPI communication
SPI_HandleTypeDef* ltc_spi;
SPI_HandleTypeDef* adc_spi;
int16_t poll_cell_voltages = 0;
int16_t poll_cell_temps = 0;
int32_t fault_timer = 0;

static uint32_t last_voltage_poll_ms = 0U;
static uint32_t last_temp_poll_ms = 0U;
static uint32_t last_heartbeat_ms = 0U;
static uint32_t last_uart_tx_ms = 0U;

/* Simulation variables for testing BMS viewer */
static uint32_t simulation_tick = 0U;
static const float VOLTAGE_BASE = 3.7f;    /* Base cell voltage in volts */
static const float VOLTAGE_AMPLITUDE = 0.3f; /* Varies by +/-0.3V */
static const float TEMP_BASE = 25.0f;      /* Base temperature in C */
static const float TEMP_AMPLITUDE = 10.0f; /* Varies by +/-10C */

/* ADC configuration for INA shunt current measurement */
static const float ADC_REF_VOLTAGE = 3.3f;  /* ADC reference voltage (3.3V) */
static const float ADC_MAX_VALUE = 4095.0f; /* 12-bit ADC max value */
static const float SHUNT_RESISTOR = 0.000001f;  /* Shunt resistor value in ohms */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void clear_all_discharge_requests(bool discharge_matrix[NUM_BOARDS][12]);
static void stop_all_balancing(BMSConfigStructTypedef *cfg, bool discharge_matrix[NUM_BOARDS][12]);
static bool pack_is_safe_for_discharge(const uint8_t status[6]);
static uint16_t get_cell_imbalance_counts(const BMS_critical_info_t *bms);
static void poll_cell_voltages_once(void);
static void simulate_bms_data(void);
static void poll_cell_temps_once(void);
static void refresh_fault_state(void);
static void handle_balancing(void);
static void heartbeat_task(void);
static float read_adc_shunt_current(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/*
 * Clear the software-side discharge request matrix.
 *
 * Balancing decisions are built in this local array first, then copied into
 * the LTC6811 configuration when it is time to push an update to the board.
 */
static void clear_all_discharge_requests(bool discharge_matrix[NUM_BOARDS][12])
{
  uint8_t board;
  uint8_t cell;

  for (board = 0U; board < NUM_BOARDS; board++) {
      for (cell = 0U; cell < 12U; cell++) {
        discharge_matrix[board][cell] = false;
      }
  }
}

/*
 * Shut balancing off immediately and push that state out to the monitor IC.
 *
 * This is the safe fallback whenever the pack is faulted or otherwise not in a
 * condition where passive discharge should be active.
 */
static void stop_all_balancing(BMSConfigStructTypedef *cfg, bool discharge_matrix[NUM_BOARDS][12])
{
  clear_all_discharge_requests(discharge_matrix);
  (void)dischargeCellGroups(cfg, discharge_matrix);
}

/*
 * Interpret the current packed fault-byte format as a simple discharge-safe or
 * discharge-unsafe decision.
 *
 * For this project, any OV, UV, OT, or UT condition blocks balancing.
 */
static bool pack_is_safe_for_discharge(const uint8_t status[6])
{
  const bool ov = ((status[0] & 0x01U) != 0U);
  const bool uv = ((status[0] & 0x02U) != 0U);
  const bool ot = ((status[0] & 0x04U) != 0U);
  const bool ut = ((status[0] & 0x08U) != 0U);

  return !(ov || uv || ot || ut);
}

/*
 * Compute the present voltage spread between the highest and lowest cells.
 *
 * The check against an inverted min/max pair protects against startup cases
 * where the extrema have not been populated with valid readings yet.
 */
static uint16_t get_cell_imbalance_counts(const BMS_critical_info_t *bms)
{
  if (bms->curr_max_voltage < bms->curr_min_voltage) {
      return 0U;
  }

  return (uint16_t)(bms->curr_max_voltage - bms->curr_min_voltage);
}

/*
 * Poll the voltage channels on the secondary board and immediately refresh the
 * pack-level voltage summary derived from those readings.
 */
static void poll_cell_voltages_once(void)
{
  (void)poll_single_secondary_voltage_reading(0U, &BMSConfig, bmsData);
  setCriticalVoltages(&BMSCriticalInfo, bmsData);
}

/*
 * Poll the temperature channels on the secondary board and refresh the pack's
 * temperature extrema based on the returned values.
 */
static void poll_cell_temps_once(void)
{
  (void)poll_single_secondary_temp_reading(0U, &BMSConfig, bmsData);
  setCriticalTemps(&BMSCriticalInfo, bmsData);
}

/*
 * Re-run the fault evaluation and mirror that result onto the board fault pin.
 *
 * This keeps both the software fault state and the hardware fault output in
 * sync with the latest measurements.
 */
static void refresh_fault_state(void)
{
  // bmsFault = FAULT_check(&BMSCriticalInfo, &BMSConfig, BMS_STATUS); temporarily diable fault checking for testing without slave 

  if (bmsFault) {
      HAL_GPIO_WritePin(BMS_FLT_EN_GPIO_Port, BMS_FLT_EN_Pin, GPIO_PIN_SET);
  } else {
      HAL_GPIO_WritePin(BMS_FLT_EN_GPIO_Port, BMS_FLT_EN_Pin, GPIO_PIN_RESET);
  }
}

/*
 * Decide whether balancing should be running right now and, if it should, send
 * the updated discharge pattern to the LTC6811.
 *
 * Balancing is only allowed when the pack is healthy enough for discharge and
 * the voltage spread is large enough to justify doing anything at all.
 */
static void handle_balancing(void)
{
  const uint16_t imbalance = get_cell_imbalance_counts(&BMSCriticalInfo);

  if ((!pack_is_safe_for_discharge(BMS_STATUS)) || bmsFault ||
      (imbalance < DEFAULT_CELL_IMBALANCE_LIMIT)) {
      stop_all_balancing(&BMSConfig, discharge);
      return;
  }

  thresholdBalance(&BMSConfig, &BMSCriticalInfo, bmsData, discharge, DEFAULT_BALANCE_THRESHOLD, DEFAULT_MAX_DISCHARGE_CELLS);

  (void)dischargeCellGroups(&BMSConfig, discharge);
}

// /*
//  * Blink the debug LED as a visible heartbeat from the main loop.
//  *
//  * When this LED keeps toggling, it is a quick sign that the firmware is still
//  * cycling through its normal background tasks.
//  */
// static void heartbeat_task(void)
// {
//   HAL_GPIO_TogglePin(DEBUG_LED_GPIO_Port, DEBUG_LED_Pin);
// }

static uint16_t crc16_ccitt(const uint8_t *data, uint16_t length)
{
  uint16_t crc = 0xFFFFU;
  uint16_t i;

  for (i = 0U; i < length; i++) {
    uint8_t j;
    crc ^= ((uint16_t)data[i] << 8U);
    for (j = 0U; j < 8U; j++) {
      if ((crc & 0x8000U) != 0U) {
        crc = (uint16_t)((crc << 1U) ^ 0x1021U);
      } else {
        crc = (uint16_t)(crc << 1U);
      }
    }
  }

  return crc;
}

// static void build_display_temperatures_cC(int16_t temp_display_cC[NUM_USED_CELLS])
// {
//   uint8_t i;
//   for (i = 0U; i < NUM_USED_CELLS; i++) {
//     /* LTC6811 temperature is raw ADC count: 1 LSB = 0.75°C */
//     temp_display_cC[i] = (int16_t)((bmsData[i].temperature * 75) / 100);
//   }
// }

/**
  * @brief Read ADC1_IN2 (PA2) INA shunt resistor voltage and convert to current
  * @return Current in Amperes
  * 
  * Conversion: I = V / R, where V is across shunt, R is shunt resistance
  */
static float read_adc_shunt_current(void)
{
  uint32_t adc_raw;
  float shunt_voltage;
  float current;
  
  /* Start ADC conversion on channel 2 (PA2) */
  if (HAL_ADC_Start(&hadc1) != HAL_OK) {
    return 0.0f;
  }
  
  /* Wait for conversion to complete */
  if (HAL_ADC_PollForConversion(&hadc1, 100U) != HAL_OK) {
    return 0.0f;
  }
  
  /* Get the raw ADC value */
  adc_raw = HAL_ADC_GetValue(&hadc1);
  
  /* Stop ADC */
  HAL_ADC_Stop(&hadc1);
  
  /* Convert raw ADC value to voltage across shunt resistor */
  /* ADC_raw: 0-4095 maps to 0-3.3V */
  shunt_voltage = (float)adc_raw * ADC_REF_VOLTAGE / ADC_MAX_VALUE;
  
  /* Calculate current from shunt resistor: I = V / R */
  current = shunt_voltage / SHUNT_RESISTOR;
  
  return current;
}

/**
  * @brief Simulate varying BMS data for testing/verification
  * Generates sine-wave varying cell voltages and temperatures
  */
static void simulate_bms_data(void)
{
  uint8_t i;
  float sine_value;
  static const float PI = 3.14159265359f;
  
  /* Generate sine wave that cycles every ~10 seconds */
  float phase = (2.0f * PI * (float)(simulation_tick % 50)) / 50.0f;
  
  /* Simple sine approximation using symmetry */
  if (phase < PI) {
    sine_value = 2.0f * phase / PI - 1.0f;
  } else {
    sine_value = 3.0f - 2.0f * phase / PI;
  }
  
  /* Clip to [-1, 1] */
  if (sine_value > 1.0f) sine_value = 1.0f;
  if (sine_value < -1.0f) sine_value = -1.0f;
  
  /* Simulate 6 cell voltages varying around base voltage */
  for (i = 0U; i < NUM_USED_CELLS; i++) {
    float cell_voltage = VOLTAGE_BASE + (VOLTAGE_AMPLITUDE * sine_value);
    bmsData[i].voltage = (uint16_t)(cell_voltage * 10000.0f);  /* Store in 100uV units */
  }
  
  /* Simulate cell temperatures varying */
  for (i = 0U; i < NUM_USED_CELLS; i++) {
    float cell_temp = TEMP_BASE + (TEMP_AMPLITUDE * sine_value);
    bmsData[i].temperature = (int16_t)(cell_temp * 100.0f) / 100;  /* Store in C */
  }
  
  /* Simulate pack voltage (sum of cells) */
  BMSCriticalInfo.isoAdcPackVoltage = (VOLTAGE_BASE * 6.0f) + (VOLTAGE_AMPLITUDE * 6.0f * sine_value);
  
  /* Read actual current from ADC1_IN2 (PA2) INA shunt resistor */
  BMSCriticalInfo.packCurrent = read_adc_shunt_current();
  
  /* Set status to OK */
  (void)memset(BMS_STATUS, 0x00, sizeof(BMS_STATUS));
}

static void uart_send_bms_telemetry(void)
{
  typedef struct __attribute__((packed)) {
    uint8_t sof1;
    uint8_t sof2;
    uint8_t length;
    uint32_t timestamp_ms;
    uint16_t cell_voltage_mV[6];
    int16_t cell_temp_cC[6];
    uint16_t pack_voltage_mV;
    int16_t pack_current_deciA;
    uint8_t status[6];
    uint16_t crc;
  } BmsUartPacket_t;

  BmsUartPacket_t packet;
  uint8_t packet_data[43];
  uint8_t i;

  packet.sof1 = UART_BMS_PACKET_SOF1;
  packet.sof2 = UART_BMS_PACKET_SOF2;
  packet.length = 37U;
  packet.timestamp_ms = HAL_GetTick();

  for (i = 0U; i < NUM_USED_CELLS; i++) {
    /* LTC6811 voltage: 1 LSB = 100uV = 0.0001V, convert to mV */
    packet.cell_voltage_mV[i] = (uint16_t)((bmsData[i].voltage * 100U) / 1000U);
  }

  /* Temperature array converted in-place */
  for (i = 0U; i < NUM_USED_CELLS; i++) {
    packet.cell_temp_cC[i] = (int16_t)((bmsData[i].temperature * 75) / 100);
  }

  /* Pack voltage: convert from float volts to mV */
  packet.pack_voltage_mV = (uint16_t)(BMSCriticalInfo.isoAdcPackVoltage * 1000.0f);
  packet.pack_current_deciA = (int16_t)(BMSCriticalInfo.packCurrent * 10.0f);

  (void)memcpy(packet.status, BMS_STATUS, sizeof(packet.status));

  packet_data[0] = packet.sof1;
  packet_data[1] = packet.sof2;
  packet_data[2] = packet.length;
  packet_data[3] = (uint8_t)((packet.timestamp_ms >> 0U) & 0xFFU);
  packet_data[4] = (uint8_t)((packet.timestamp_ms >> 8U) & 0xFFU);
  packet_data[5] = (uint8_t)((packet.timestamp_ms >> 16U) & 0xFFU);
  packet_data[6] = (uint8_t)((packet.timestamp_ms >> 24U) & 0xFFU);

  for (i = 0U; i < NUM_USED_CELLS; i++) {
    packet_data[7U + (i * 2U)] = (uint8_t)((packet.cell_voltage_mV[i] >> 0U) & 0xFFU);
    packet_data[8U + (i * 2U)] = (uint8_t)((packet.cell_voltage_mV[i] >> 8U) & 0xFFU);
  }

  for (i = 0U; i < NUM_USED_CELLS; i++) {
    uint16_t temp_u16 = (uint16_t)(packet.cell_temp_cC[i] & 0xFFFF);
    packet_data[19U + (i * 2U)] = (uint8_t)((temp_u16 >> 0U) & 0xFFU);
    packet_data[20U + (i * 2U)] = (uint8_t)((temp_u16 >> 8U) & 0xFFU);
  }

  packet_data[31U] = (uint8_t)((packet.pack_voltage_mV >> 0U) & 0xFFU);
  packet_data[32U] = (uint8_t)((packet.pack_voltage_mV >> 8U) & 0xFFU);
  packet_data[33U] = (uint8_t)((packet.pack_current_deciA >> 0U) & 0x00FF);
  packet_data[34U] = (uint8_t)((packet.pack_current_deciA >> 8U) & 0x00FF);

  (void)memcpy(&packet_data[35U], packet.status, 6U);

  packet.crc = crc16_ccitt(packet_data, 41U);
  packet_data[41U] = (uint8_t)((packet.crc >> 0U) & 0xFFU);
  packet_data[42U] = (uint8_t)((packet.crc >> 8U) & 0xFFU);

  (void)HAL_UART_Transmit(&huart2, (uint8_t *)packet_data, 43U, 100U);
}

static void MX_USART2_UART_Init(void)
{
    huart2.Instance = USART2;
    huart2.Init.BaudRate = 115200;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;

    if (HAL_UART_Init(&huart2) != HAL_OK) {
        Error_Handler();
    }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_SPI3_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  MX_TIM7_Init();
  MX_TIM13_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  // Initialize ltc_spi to point to appropriate SPI handle (SPI1 is used for LTC6811)
  ltc_spi = &hspi1;
  
  // Initialize adc_spi to point to appropriate SPI handle (SPI3 is used for ADC)
  adc_spi = &hspi3;
  
  initPECTable();
  loadConfig(&BMSConfig);
  init_BMS_info(&BMSCriticalInfo);

  clear_all_discharge_requests(discharge);
  (void)memset(BMS_STATUS, 0, sizeof(BMS_STATUS));
  (void)memset(bmsData, 0, sizeof(bmsData));

  writeConfigAll(&BMSConfig);

  last_voltage_poll_ms = HAL_GetTick();
  last_temp_poll_ms = HAL_GetTick();
  last_heartbeat_ms = HAL_GetTick();

  MX_USART2_UART_Init();
  last_uart_tx_ms = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    uint32_t now_ms = HAL_GetTick();

    if ((now_ms - last_voltage_poll_ms) >= VOLTAGE_POLL_PERIOD_MS) {
      poll_cell_voltages_once();
      last_voltage_poll_ms = now_ms;
    }

    if ((now_ms - last_temp_poll_ms) >= TEMP_POLL_PERIOD_MS) {
      poll_cell_temps_once();
      last_temp_poll_ms = now_ms;
    }

    refresh_fault_state();
    handle_balancing();

    if ((now_ms - last_heartbeat_ms) >= HEARTBEAT_PERIOD_MS) {
      // heartbeat_task();
      last_heartbeat_ms = now_ms;
    }

    if ((now_ms - last_uart_tx_ms) >= UART_TX_PERIOD_MS) {
      // simulate_bms_data();  /* Simulate varying voltage/temperature for testing */
      uart_send_bms_telemetry();
      last_uart_tx_ms = now_ms;
    }

    simulation_tick++;
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
