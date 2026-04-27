/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
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
#define VOLTAGE_POLL_PERIOD_MS       200U
#define TEMP_POLL_PERIOD_MS          500U
#define HEARTBEAT_PERIOD_MS          250U
#define UART_TX_PERIOD_MS            200U
#define ENABLE_UART_SIMULATION       1U
#define DEFAULT_BALANCE_THRESHOLD    41500U
#define DEFAULT_MAX_DISCHARGE_CELLS  1U
#define DEFAULT_CELL_IMBALANCE_LIMIT 150U
#define UART_BMS_PACKET_SOF1         0xAAU
#define UART_BMS_PACKET_SOF2         0x55U
#define UART_BMS_PACKET_LEN          40U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
CellData bmsData[NUM_CELLS];
uint8_t BMS_STATUS[6];
bool discharge[NUM_BOARDS][12];

volatile bool bmsFault = false;

BMS_critical_info_t BMSCriticalInfo;
static BMSConfigStructTypedef BMSConfig;
SPI_HandleTypeDef *ltc_spi;

static uint32_t last_voltage_poll_ms = 0U;
static uint32_t last_temp_poll_ms = 0U;
static uint32_t last_heartbeat_ms = 0U;
static uint32_t last_uart_tx_ms = 0U;
static uint32_t simulation_tick = 0U;

static const float ADC_REF_VOLTAGE = 3.3f;
static const float ADC_MAX_VALUE = 4095.0f;
static const float SHUNT_RESISTOR = 0.000001f;
static const float SIM_BASE_CELL_V = 3.92f;
static const float SIM_SWING_CELL_V = 0.18f;
static const float SIM_BASE_TEMP_C = 28.0f;
static const float SIM_SWING_TEMP_C = 7.0f;
static const float SIM_BASE_CURRENT_A = 6.0f;
static const float SIM_SWING_CURRENT_A = 1.5f;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */
static void clear_all_discharge_requests(bool discharge_matrix[NUM_BOARDS][12]);
static void stop_all_balancing(BMSConfigStructTypedef *cfg, bool discharge_matrix[NUM_BOARDS][12]);
static bool pack_is_safe_for_discharge(const uint8_t status[6]);
static uint16_t get_cell_imbalance_counts(const BMS_critical_info_t *bms);
static void poll_cell_voltages_once(void);
static void poll_cell_temps_once(void);
static void simulate_bms_data(void);
static void refresh_fault_state(void);
static void handle_balancing(void);
static float read_adc_shunt_current(void);
static void uart_send_bms_telemetry(void);
static uint16_t crc16_ccitt(const uint8_t *data, uint16_t length);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
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

static void stop_all_balancing(BMSConfigStructTypedef *cfg, bool discharge_matrix[NUM_BOARDS][12])
{
  clear_all_discharge_requests(discharge_matrix);
  (void)dischargeCellGroups(cfg, discharge_matrix);
}

static bool pack_is_safe_for_discharge(const uint8_t status[6])
{
  const bool ov = ((status[0] & 0x01U) != 0U);
  const bool uv = ((status[0] & 0x02U) != 0U);
  const bool ot = ((status[0] & 0x04U) != 0U);
  const bool ut = ((status[0] & 0x08U) != 0U);

  return !(ov || uv || ot || ut);
}

static uint16_t get_cell_imbalance_counts(const BMS_critical_info_t *bms)
{
  if (bms->curr_max_voltage < bms->curr_min_voltage) {
    return 0U;
  }

  return (uint16_t)(bms->curr_max_voltage - bms->curr_min_voltage);
}

static void poll_cell_voltages_once(void)
{
  (void)poll_single_secondary_voltage_reading(0U, &BMSConfig, bmsData);
  setCriticalVoltages(&BMSCriticalInfo, bmsData);
}

static void poll_cell_temps_once(void)
{
  (void)poll_single_secondary_temp_reading(0U, &BMSConfig, bmsData);
  setCriticalTemps(&BMSCriticalInfo, bmsData);
}

static void simulate_bms_data(void)
{
  uint8_t i;
  float triangle_phase;
  float triangle_wave;
  float current_a;
  uint32_t total_pack_mV = 0U;

  triangle_phase = (float)(simulation_tick % 60U) / 60.0f;
  if (triangle_phase < 0.5f) {
    triangle_wave = (4.0f * triangle_phase) - 1.0f;
  } else {
    triangle_wave = 3.0f - (4.0f * triangle_phase);
  }

  current_a = SIM_BASE_CURRENT_A + (SIM_SWING_CURRENT_A * triangle_wave);
  if (current_a < 0.0f) {
    current_a = 0.0f;
  }

  for (i = 0U; i < NUM_USED_CELLS; i++) {
    float cell_offset_v = ((float)i - 2.5f) * 0.008f;
    float drift_v = (i == 2U) ? ((float)(simulation_tick % 80U) * 0.0008f) : 0.0f;
    float cell_voltage_v = SIM_BASE_CELL_V + (SIM_SWING_CELL_V * triangle_wave) + cell_offset_v + drift_v;
    float cell_temp_c = SIM_BASE_TEMP_C + (SIM_SWING_TEMP_C * triangle_wave) + ((i < 3U) ? 0.5f : 2.0f);
    uint16_t cell_voltage_counts;
    uint16_t cell_temp_mC;

    if (cell_voltage_v < 3.0f) {
      cell_voltage_v = 3.0f;
    }
    if (cell_voltage_v > 4.2f) {
      cell_voltage_v = 4.2f;
    }
    if (cell_temp_c < 10.0f) {
      cell_temp_c = 10.0f;
    }
    if (cell_temp_c > 62.0f) {
      cell_temp_c = 62.0f;
    }

    cell_voltage_counts = (uint16_t)(cell_voltage_v * 10000.0f);
    cell_temp_mC = (uint16_t)(cell_temp_c * 1000.0f);

    bmsData[i].voltage = cell_voltage_counts;
    bmsData[i].temperature = cell_temp_mC;
    total_pack_mV += (uint32_t)(cell_voltage_counts / 10U);
  }

  setCriticalVoltages(&BMSCriticalInfo, bmsData);
  setCriticalTemps(&BMSCriticalInfo, bmsData);
  BMSCriticalInfo.packCurrent = current_a;
  BMSCriticalInfo.cellMonitorPackVoltage = (uint16_t)total_pack_mV;

  (void)memset(BMS_STATUS, 0, sizeof(BMS_STATUS));
  if (BMSCriticalInfo.curr_max_voltage > BMSConfig.OV_threshold) {
    BMS_STATUS[0] |= 0x01U;
  }
  if (BMSCriticalInfo.curr_min_voltage < BMSConfig.UV_threshold) {
    BMS_STATUS[0] |= 0x02U;
  }
  if (BMSCriticalInfo.curr_max_temp > BMSConfig.OT_threshold) {
    BMS_STATUS[0] |= 0x04U;
  }
  if (BMSCriticalInfo.curr_min_temp < BMSConfig.UT_threshold) {
    BMS_STATUS[0] |= 0x08U;
  }
}

static void refresh_fault_state(void)
{
  bmsFault = FAULT_check(&BMSCriticalInfo, &BMSConfig, BMS_STATUS);

  if (bmsFault) {
    HAL_GPIO_WritePin(BMS_FLT_EN_GPIO_Port, BMS_FLT_EN_Pin, GPIO_PIN_SET);
  } else {
    HAL_GPIO_WritePin(BMS_FLT_EN_GPIO_Port, BMS_FLT_EN_Pin, GPIO_PIN_RESET);
  }
}

static void handle_balancing(void)
{
  const uint16_t imbalance = get_cell_imbalance_counts(&BMSCriticalInfo);

  if ((!pack_is_safe_for_discharge(BMS_STATUS)) || bmsFault ||
      (imbalance < DEFAULT_CELL_IMBALANCE_LIMIT)) {
    stop_all_balancing(&BMSConfig, discharge);
    return;
  }

  thresholdBalance(&BMSConfig, &BMSCriticalInfo, bmsData, discharge,
                   DEFAULT_BALANCE_THRESHOLD, DEFAULT_MAX_DISCHARGE_CELLS);
  (void)dischargeCellGroups(&BMSConfig, discharge);
}

static float read_adc_shunt_current(void)
{
  uint32_t adc_raw;
  float shunt_voltage;

  if (HAL_ADC_Start(&hadc1) != HAL_OK) {
    return 0.0f;
  }

  if (HAL_ADC_PollForConversion(&hadc1, 100U) != HAL_OK) {
    return 0.0f;
  }

  adc_raw = HAL_ADC_GetValue(&hadc1);
  HAL_ADC_Stop(&hadc1);

  shunt_voltage = (float)adc_raw * ADC_REF_VOLTAGE / ADC_MAX_VALUE;
  return shunt_voltage / SHUNT_RESISTOR;
}

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

static void uart_send_bms_telemetry(void)
{
  typedef struct __attribute__((packed)) {
    uint8_t sof1;
    uint8_t sof2;
    uint8_t length;
    uint32_t timestamp_ms;
    uint16_t cell_voltage_mV[NUM_USED_CELLS];
    int16_t cell_temp_cC[NUM_USED_CELLS];
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
  packet.length = UART_BMS_PACKET_LEN;
  packet.timestamp_ms = HAL_GetTick();

  for (i = 0U; i < NUM_USED_CELLS; i++) {
    packet.cell_voltage_mV[i] = (uint16_t)((bmsData[i].voltage * 100U) / 1000U);
    packet.cell_temp_cC[i] = (int16_t)(bmsData[i].temperature / 10U);
  }

  packet.pack_voltage_mV = BMSCriticalInfo.cellMonitorPackVoltage;
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
  packet_data[33U] = (uint8_t)((packet.pack_current_deciA >> 0U) & 0xFFU);
  packet_data[34U] = (uint8_t)((packet.pack_current_deciA >> 8U) & 0xFFU);
  (void)memcpy(&packet_data[35U], packet.status, 6U);

  packet.crc = crc16_ccitt(packet_data, 41U);
  packet_data[41U] = (uint8_t)((packet.crc >> 0U) & 0xFFU);
  packet_data[42U] = (uint8_t)((packet.crc >> 8U) & 0xFFU);

//  (void)HAL_UART_Transmit(&huart2, packet_data, sizeof(packet_data), 100U);

  char message[] = "Nucleo1 comm\r\n";
  (void)HAL_UART_Transmit(&huart2, (uint8_t*)message, strlen(message), 1000);
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
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  ltc_spi = &hspi1;
  initPECTable();
  loadConfig(&BMSConfig);
  init_BMS_info(&BMSCriticalInfo);

  clear_all_discharge_requests(discharge);
  (void)memset(BMS_STATUS, 0, sizeof(BMS_STATUS));
  (void)memset(bmsData, 0, sizeof(bmsData));

#if !ENABLE_UART_SIMULATION
  writeConfigAll(&BMSConfig);
#endif

  last_voltage_poll_ms = HAL_GetTick();
  last_temp_poll_ms = HAL_GetTick();
  last_heartbeat_ms = HAL_GetTick();
  last_uart_tx_ms = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    uint32_t now_ms = HAL_GetTick();

#if ENABLE_UART_SIMULATION
    simulate_bms_data();
#else
    if ((now_ms - last_voltage_poll_ms) >= VOLTAGE_POLL_PERIOD_MS) {
      poll_cell_voltages_once();
      last_voltage_poll_ms = now_ms;
    }

    if ((now_ms - last_temp_poll_ms) >= TEMP_POLL_PERIOD_MS) {
      poll_cell_temps_once();
      last_temp_poll_ms = now_ms;
    }

    BMSCriticalInfo.packCurrent = read_adc_shunt_current();
    refresh_fault_state();
    handle_balancing();
#endif

    if ((now_ms - last_heartbeat_ms) >= HEARTBEAT_PERIOD_MS) {
      last_heartbeat_ms = now_ms;
    }

    if ((now_ms - last_uart_tx_ms) >= UART_TX_PERIOD_MS) {
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV4;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_7CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
