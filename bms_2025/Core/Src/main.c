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
SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi3;

SPI_HandleTypeDef* ltc_spi = &hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim13;

UART_HandleTypeDef huart2;
static uint32_t last_uart_tx_ms = 0U;

/* USER CODE BEGIN PV */
CellData bmsData[NUM_CELLS];
uint8_t BMS_STATUS[6];
bool discharge[NUM_BOARDS][12];

volatile bool bmsFault = false;

BMS_critical_info_t BMSCriticalInfo;
static BMSConfigStructTypedef BMSConfig;

static uint32_t last_voltage_poll_ms = 0U;
static uint32_t last_temp_poll_ms = 0U;
static uint32_t last_heartbeat_ms = 0U;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_SPI3_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM7_Init(void);
static void MX_TIM13_Init(void);

/* USER CODE BEGIN PFP */
static void clear_all_discharge_requests(bool discharge_matrix[NUM_BOARDS][12]);
static void stop_all_balancing(BMSConfigStructTypedef *cfg, bool discharge_matrix[NUM_BOARDS][12]);
static bool pack_is_safe_for_discharge(const uint8_t status[6]);
static uint16_t get_cell_imbalance_counts(const BMS_critical_info_t *bms);
static void poll_cell_voltages_once(void);
static void poll_cell_temps_once(void);
static void refresh_fault_state(void);
static void handle_balancing(void);
static void heartbeat_task(void);

static void MX_USART2_UART_Init(void);
static uint16_t crc16_ccitt(const uint8_t *data, uint16_t length);
static void build_display_temperatures_cC(int16_t temp_display_cC[NUM_USED_CELLS]);
static void uart_send_bms_telemetry(void);

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
  bmsFault = FAULT_check(&BMSCriticalInfo, &BMSConfig, BMS_STATUS);

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

/*
 * Blink the debug LED as a visible heartbeat from the main loop.
 *
 * When this LED keeps toggling, it is a quick sign that the firmware is still
 * cycling through its normal background tasks.
 */
static void heartbeat_task(void)
{
  HAL_GPIO_TogglePin(DEBUG_LED_GPIO_Port, DEBUG_LED_Pin);
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

/*
 * Bring up the MCU, initialize the peripherals this firmware uses, load the
 * BMS configuration, and then stay in the background polling loop.
 *
 * The loop is intentionally straightforward: sample voltages, sample
 * temperatures, refresh faults, and let the balancing logic make decisions
 * based on the newest data.
 */
int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_SPI3_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM7_Init();
  MX_TIM13_Init();

  /* USER CODE BEGIN 2 */
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

  while (1)
  {
    const uint32_t now = HAL_GetTick();

    if ((now - last_voltage_poll_ms) >= VOLTAGE_POLL_PERIOD_MS) {
      last_voltage_poll_ms = now;
      poll_cell_voltages_once();
      refresh_fault_state();
    }

    if ((now - last_temp_poll_ms) >= TEMP_POLL_PERIOD_MS) {
      last_temp_poll_ms = now;
      poll_cell_temps_once();
      refresh_fault_state();
    }

    handle_balancing();

    if ((now - last_heartbeat_ms) >= HEARTBEAT_PERIOD_MS) {
      last_heartbeat_ms = now;
      heartbeat_task();
    }

    if ((now - last_uart_tx_ms) >= UART_TX_PERIOD_MS) {
      last_uart_tx_ms = now;
      uart_send_bms_telemetry();
    }
  }
}

/*
 * Set up a simple clock tree using the internal high-speed oscillator.
 *
 * This keeps the system clock configuration easy to reason about during bring
 * up and avoids introducing PLL-related complexity in the main application
 * file.
 */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/*
 * Initialize SPI1 as a master peripheral.
 *
 * These settings define the clock mode and transfer format expected by the
 * device connected to this bus.
 */
static void MX_SPI1_Init(void)
{
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
}

/*
 * Initialize SPI2 with the timing required by the device on that interface.
 *
 * The phase and prescaler differ from SPI1 because this bus is serving a
 * different peripheral.
 */
static void MX_SPI2_Init(void)
{
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
}

/*
 * Initialize SPI3 as another master bus for board peripherals.
 */
static void MX_SPI3_Init(void)
{
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
}

/*
 * Configure TIM1 as a base timer with an internal clock source.
 *
 * No special trigger routing is used here; the timer is simply prepared for
 * later scheduling or timing work elsewhere in the firmware.
 */
static void MX_TIM1_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 62499;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

/*
 * Configure TIM2 for input-capture operation on channel 2.
 *
 * This lets the firmware timestamp incoming edges if another subsystem needs a
 * measured external signal.
 */
static void MX_TIM2_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295U;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/*
 * Configure TIM3 for PWM generation on channel 1.
 *
 * The post-init call finishes the pin-side setup after the timer itself is
 * configured.
 */
static void MX_TIM3_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_TIM_MspPostInit(&htim3);
}

/*
 * Configure TIM7 as a simple periodic base timer.
 */
static void MX_TIM7_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 0;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 31999;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

/*
 * Configure TIM13 as an additional free-running time base.
 */
static void MX_TIM13_Init(void)
{
  htim13.Instance = TIM13;
  htim13.Init.Prescaler = 495;
  htim13.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim13.Init.Period = 64515;
  htim13.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim13.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim13) != HAL_OK)
  {
    Error_Handler();
  }
}

/*
 * Initialize the GPIO used directly by this firmware.
 *
 * Output levels are written before the pins are switched into output mode so
 * attached hardware does not see a brief unintended pulse during startup.
 */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  HAL_GPIO_WritePin(BMS_FLT_EN_GPIO_Port, BMS_FLT_EN_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(DEBUG_LED_GPIO_Port, DEBUG_LED_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(SPI_UCOMM_CS_GPIO_Port, SPI_UCOMM_CS_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(SPI_FERAM_CS_GPIO_Port, SPI_FERAM_CS_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(SPI_ADC_CS_GPIO_Port, SPI_ADC_CS_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(ADC_RST_GPIO_Port, ADC_RST_Pin, GPIO_PIN_SET);

  GPIO_InitStruct.Pin = BMS_FLT_EN_Pin | ADC_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = SHUTDOWN_ACTIVE_Pin | LV_PWR_MONITOR_Pin | ADC_DRDY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = PRECHARGE_COMPLETE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(PRECHARGE_COMPLETE_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = DEBUG_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(DEBUG_LED_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = SPI_UCOMM_CS_Pin | SPI_FERAM_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = SPI_ADC_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI_ADC_CS_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = CHARGE_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(CHARGE_EN_GPIO_Port, &GPIO_InitStruct);
}


#pragma pack(push, 1)
typedef struct
{
  uint8_t sof1;
  uint8_t sof2;
  uint8_t payload_len;
  uint32_t timestamp_ms;

  uint16_t cell_voltage_mV[NUM_USED_CELLS];
  int16_t cell_temp_cC[NUM_USED_CELLS];

  uint16_t pack_voltage_mV;
  uint16_t pack_current_deciA;  /* Pack current in units of 0.1A (10A stored as 100) */
  uint8_t status_bytes[6];

  uint16_t crc;
} BmsUartPacket_t;
#pragma pack(pop)

static uint16_t crc16_ccitt(const uint8_t *data, uint16_t length)
{
  uint16_t crc = 0xFFFFU;

  for (uint16_t i = 0U; i < length; i++) {
      crc ^= (uint16_t)((uint16_t)data[i] << 8);

      for (uint8_t bit = 0U; bit < 8U; bit++) {
          if ((crc & 0x8000U) != 0U) {
              crc = (uint16_t)((crc << 1) ^ 0x1021U);
          } else {
              crc <<= 1;
          }
      }
  }

  return crc;
}

static void build_display_temperatures_cC(int16_t temp_display_cC[NUM_USED_CELLS])
{
  int16_t t1_cC = 0;
  int16_t t2_cC = 0;

  /* Use representative cell from each thermistor group */
  t1_cC = (int16_t)(bmsData[0].temperature / 10U);
  t2_cC = (int16_t)(bmsData[3].temperature / 10U);

  temp_display_cC[0] = t1_cC;
  temp_display_cC[1] = t1_cC;
  temp_display_cC[2] = t1_cC;
  temp_display_cC[3] = t2_cC;
  temp_display_cC[4] = t2_cC;
  temp_display_cC[5] = t2_cC;
}

static void uart_send_bms_telemetry(void)
{
  BmsUartPacket_t packet;
  int16_t expandedTemps[NUM_USED_CELLS];
  uint16_t crc_input_len;

  memset(&packet, 0, sizeof(packet));

  packet.sof1 = UART_BMS_PACKET_SOF1;
  packet.sof2 = UART_BMS_PACKET_SOF2;
  packet.payload_len = (uint8_t)(sizeof(BmsUartPacket_t) - 3U); /* excludes sof1, sof2, payload_len */
  packet.timestamp_ms = HAL_GetTick();

  for (uint8_t i = 0U; i < NUM_USED_CELLS; i++) {
    /*
      * Cell voltage code to mV
      * 42000 -> 4200 mV
      */
    packet.cell_voltage_mV[i] = (uint16_t)(bmsData[i].voltage / 10U);
}

  build_display_temperatures_cC(expandedTemps);

  for (uint8_t i = 0U; i < NUM_USED_CELLS; i++) {
    packet.cell_temp_cC[i] = expandedTemps[i];
  }

  packet.pack_voltage_mV = BMSCriticalInfo.cellMonitorPackVoltage;
  packet.pack_current_deciA = (int16_t)(BMSCriticalInfo.packCurrent * 10.0f);

  for (uint8_t i = 0U; i < 6U; i++) {
    packet.status_bytes[i] = BMS_STATUS[i];
  }

  crc_input_len = (uint16_t)(sizeof(BmsUartPacket_t) - sizeof(packet.crc));
  packet.crc = crc16_ccitt((const uint8_t *)&packet, crc_input_len);

  (void)HAL_UART_Transmit(&huart2, (uint8_t *)&packet, sizeof(packet), 50U);
}


/*
 * Stop normal execution after an unrecoverable error.
 *
 * Once we get here, interrupts are disabled and the firmware stays in a tight
 * loop so the failure is obvious during debugging.
 */
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef USE_FULL_ASSERT
/*
 * HAL assertion hook.
 *
 * The file and line are currently unused, but the function is kept so full
 * assert builds still have a defined landing point.
 */
void assert_failed(uint8_t *file, uint32_t line)
{
  (void)file;
  (void)line;
}
#endif
