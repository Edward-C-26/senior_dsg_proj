
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"

#include "BMSconfig.h"
#include "Fault.h"
#include "LTC6811.h"
#include "PackCalculations.h"
#include "SPI.h"
#include "isoADC.h"

#include "can_1.h"
#include "can_2.h"
#include "stm32f4xx_hal_can.h"
#include "stm32f4xx_hal_gpio.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CHARGER_OUT_ID 0x405
#define CAN_RX_MAILBOX_SIZE 3

#define CHARGER_IN_ID 0x1806E5F4	// charger CAN ID is 0x1806E5F4
#define CHARGER_INFO_ID 0x700
#define CONSTANT_CAN_ENABLE 0

#define BMS_RX_MSG_ID 0x0300
#define BMS_RX_MSG_MASK 0x0F00


#define BALANCE_EN  0

// STM manual balancing
#define THRESHOLD_BALANCE_EN 0
#define DISCHARGE_THRESHOLD 65535 // well above cell max voltage LOOK AT ACTUAL FUNCTION CALL FOR THRESHOLD SETTING

// CAN Manual Balancing
#define MANUAL_BALANCING_ID 0x381 // TODO

// CAN transmit timeout (ms)
// Assuming a 0% bus load 500kbit/s CAN bus, 5ms would be enough time to send ~23 8 byte CAN messages
// If we hit the 5ms timeout without transmitting any pending messages, we're either at an extremely high bus load
// and losing arbitration or there's some other issue (bus disconnected, no termination, etc.)
#define CAN_TX_TIMEOUT 2


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim13;

/* USER CODE BEGIN PV */
SPI_HandleTypeDef *ltc_spi = &hspi1;

CAN_TxHeaderTypeDef tx_header;
volatile CAN_RxHeaderTypeDef rx_header;
uint8_t tx_data[8];
volatile uint8_t rx_data[8] = {0, 0, 0, 0, 0, 0, 0, 0};
uint32_t tx_mailbox;

CAN_TxHeaderTypeDef charger_tx_header;
uint8_t charger_tx_data[8];
uint8_t charge_rate = 2;
uint8_t global_error_count = 0;
uint8_t balance_counter = 0;
uint8_t charging_msg_timeout = 1;

int16_t poll_cell_temps = 0;
int16_t poll_cell_voltages = 0;

bool CHARGE_EN = 0;

CellData bmsData[NUM_CELLS];
static_assert(NUM_CELLS <= 144, "NUM_CELLS must not exceed array size");
uint8_t BMS_STATUS[6];
bool discharge[NUM_BOARDS][12];
bool full_discharge[NUM_BOARDS][12];

volatile manual_balancing_t manual_balancing_config;
volatile bool bmsFault;

SPI_HandleTypeDef* isoADC_SPI_ptr_g = &hspi2;
GPIO_TypeDef* isoADC_SPI_cs_port_ptr_g = SPI_ADC_CS_GPIO_Port;
uint16_t isoADC_SPI_cs_pin_g = GPIO_PIN_12;
TIM_HandleTypeDef* isoADC_PWM_ptr_g = &htim3;
uint32_t isoADC_PWM_ch_g = TIM_CHANNEL_1;
BMS_critical_info_t BMSCriticalInfo;
uint8_t isoADC_rdy_status = 0, isoADC_period_miss = 0, max_isoADC_period = 0;
uint32_t cell_volt_timing = 0, cell_temp_timing = 0, volt_start_time = 0, temp_start_time = 0;

volatile bool pollingFlag = false;
volatile uint8_t new_balance_msg = 0, can_tx_phase = 0;
volatile uint8_t balancing_data_array[8];


uint32_t prevTime = 0, timeBetween = 0;
int32_t fault_timer = 2000;
uint8_t error_cnt = 0, error_voltage_read_count = 0, error_temp_read_count = 0;

float live_pack_voltage = 0, live_pack_current = 0;
bool live_precharge = false;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_SPI3_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_CAN2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM7_Init(void);
static void MX_TIM13_Init(void);
/* USER CODE BEGIN PFP */
static bool transmit_can_message(CAN_HandleTypeDef* hcan, const CAN_TxHeaderTypeDef *tx_header, const uint8_t msg_data[]);
static void transmit_cell_data_msg(CellData const bmsData[NUM_CELLS]);
static void transmit_bms_status_msg(BMS_critical_info_t const *bms, uint8_t const bmsStatus[6]);
static void transmit_cell_vlt_msg(BMS_critical_info_t const *bms);
static void transmit_cell_temp_msg(BMS_critical_info_t const *bms);
// void PACKSTAT_message(BMSConfigStructTypedef cfg, BMS_critical_info_t bms, uint8_t bmsData[NUM_CELLS][6]); // old method 
// static void PACKSTAT_message(BMS_critical_info_t const *bms); // draft :D

// Manual Charging/Balancing Functions
void CHARGER_message();
void resetChargerVariables();
void process_balancing_msg();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
    BMSConfigStructTypedef BMSConfig;

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN1_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_SPI3_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_CAN2_Init();
  MX_TIM1_Init();
  MX_TIM7_Init();
  MX_TIM13_Init();
  /* USER CODE BEGIN 2 */
    initPECTable();
    loadConfig(&BMSConfig);
    init_BMS_info(&BMSCriticalInfo);
    resetChargerVariables();

    // wake up isoADC
    HAL_GPIO_WritePin(DEBUG_LED_GPIO_Port, DEBUG_LED_Pin, GPIO_PIN_RESET);
    isoADC_rdy_status = wakeup_isoADC(&gIsoADCConfig, &gIsoADCData);

    // turn on timer interrupt for isoADC 128Hz
    HAL_TIM_Base_Start_IT(&htim1);
    // Sending CAN messages
    HAL_TIM_Base_Start_IT(&htim7);
    // Sending cell polling message
    HAL_TIM_Base_Start_IT(&htim13);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1)
    {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

        /** FUNCTION CALL OVERVIEW
         * First: Call read6811 -> Voltages, Temps on 6811
         * Second: Check for Faults
         * Third: Call balancing algorithm
         * Fourth : Remaining CAN messages
         * Last: :3 
         */
    	if (poll_cell_voltages <= 0){
    		poll_cell_voltages = 1000;
    		volt_start_time = HAL_GetTick();

    		for (int i = 0; i < NUM_BOARDS; i++){
				poll_single_secondary_voltage_reading((uint8_t) i, &BMSConfig, bmsData);
			}
			cell_volt_timing = HAL_GetTick() - volt_start_time;

			setCriticalVoltages(&BMSCriticalInfo, bmsData);

			DISABLE_ALL_CAN_IRQS
			bmsFault = FAULT_check(&BMSCriticalInfo, BMS_STATUS);
			ENABLE_ALL_CAN_IRQS

			// check voltage faults
			bool overvoltage_fault	= ((BMS_STATUS[0] & 0x01) == 0x01);
			bool undervoltage_fault = ((BMS_STATUS[0] & 0x02) == 0x02);


			if((overvoltage_fault == true) || (undervoltage_fault == true)){
				error_voltage_read_count++;
			}

    	}


        // Read all Temps from LTC6811, store them in 144x6 array,
        // set the critical info struct, then send temp info over CAN
    	if (poll_cell_temps <= 0){
    		poll_cell_temps = 2500;
    		temp_start_time = HAL_GetTick();

    	for (int i = 0; i < NUM_BOARDS; i++){
    		poll_single_secondary_temp_reading((uint8_t) i, &BMSConfig, bmsData);
    	}
			cell_temp_timing = HAL_GetTick() - temp_start_time;
			setCriticalTemps(&BMSCriticalInfo, bmsData);

			DISABLE_ALL_CAN_IRQS
			bmsFault = FAULT_check(&BMSCriticalInfo, BMS_STATUS);
			ENABLE_ALL_CAN_IRQS

			// check temperature faults
			bool overtemp_fault		= ((BMS_STATUS[0] & 0x04) == 0x04);
			bool undertemp_fault	= ((BMS_STATUS[0] & 0x08) == 0x08);


			if((overtemp_fault == true) || (undertemp_fault == true)){
				error_temp_read_count++;
			}
		}


       // Check Precharge Complete Pin
       manual_balancing_config.precharge_cplt = !(HAL_GPIO_ReadPin(PRECHARGE_COMPLETE_GPIO_Port, PRECHARGE_COMPLETE_Pin));
       live_precharge = manual_balancing_config.precharge_cplt;
       DISABLE_ALL_CAN_IRQS

	   /* DO THIS WHEN TESTING BMS FAULTS*/
	   //*NOTE* : need to figure out which pin is the BMS fault pin
	   bmsFault = FAULT_check(&BMSCriticalInfo, BMS_STATUS);
       if (bmsFault == false) {
    	   global_error_count = 0;
    	   fault_timer = 2000;
    	   error_temp_read_count = 0;
    	   error_voltage_read_count = 0;
    	   HAL_GPIO_WritePin(BMS_FLT_EN_GPIO_Port, BMS_FLT_EN_Pin, GPIO_PIN_RESET);
       }
       else {
    	   global_error_count++;
    	   if ((error_temp_read_count >= 2) || (error_voltage_read_count >= 2)){
    		   HAL_GPIO_WritePin(BMS_FLT_EN_GPIO_Port, BMS_FLT_EN_Pin, GPIO_PIN_SET);
    		   error_temp_read_count = 2;
    		   error_voltage_read_count = 2;
    	   }
       }

        BMSCriticalInfo.isoAdcPackVoltage = (float)gIsoADCData.bus_voltage;
        BMSCriticalInfo.packCurrent = (float)gIsoADCData.shunt_current;
        ENABLE_ALL_CAN_IRQS
    	DISABLE_CAN1_RX_FIF0_IRQ;
        if(new_balance_msg == true){
        	process_balancing_msg();
        	new_balance_msg = 0;
        }
    	ENABLE_CAN1_RX_FIF0_IRQ;

    	/* Discharge cells if:
    	 * charge message is valid
    	 * AND discharge is enabled
    	 * AND precharge is complete (contactors are closed)
    	 * AND there is no BMS fault
    	 */
        if((manual_balancing_config.valid_charge_message == true) &&
			(manual_balancing_config.discharge_balance_en == true) &&
			(manual_balancing_config.precharge_cplt == true) &&
			(bmsFault == false)) {

            thresholdBalance(&BMSConfig, &BMSCriticalInfo, bmsData, discharge, manual_balancing_config.discharge_threshold_voltage, manual_balancing_config.num_cells_discharged_per_secondary);
          
            dischargeCellGroups(&BMSConfig, discharge);
            HAL_Delay(BMSConfig.dischargeTime);

            dischargeCellGroups(&BMSConfig, discharge);
            HAL_Delay(BMSConfig.dischargeTime);
        }
        else {
        	thresholdBalance(&BMSConfig, &BMSCriticalInfo, bmsData, discharge, 43000, 0);	// default to an "off" state, cells should never go this high
        }

        // stop messages from being sent if no message has been sent in 2 seconds
        if (charging_msg_timeout == 1) {
        	resetChargerVariables();
        }

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

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */
  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 2;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = ENABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

	CAN_FilterTypeDef  sFilterConfig;
	sFilterConfig.FilterBank = 0;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_16BIT;
	sFilterConfig.FilterIdHigh = BMS_RX_MSG_ID;
	sFilterConfig.FilterIdLow = 0xFFFF;
	sFilterConfig.FilterMaskIdHigh = BMS_RX_MSG_MASK;
	sFilterConfig.FilterMaskIdLow = 0x0000; // (any uint16_t) & 0x0000 != 0xFFFF
	sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	sFilterConfig.FilterActivation = ENABLE;
	sFilterConfig.SlaveStartFilterBank = 14;

	if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK)
	{
	  /* Filter configuration Error */
	  Error_Handler();
	}

  if (HAL_CAN_Start(&hcan1) != HAL_OK)
  {
	/* Start Error */
	Error_Handler();
  }

  // This ties the interupt to the rx_fifo0 msg bufer
  if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
   {
     Error_Handler();
   }
  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief CAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN2_Init(void)
{

  /* USER CODE BEGIN CAN2_Init 0 */

  /* USER CODE END CAN2_Init 0 */

  /* USER CODE BEGIN CAN2_Init 1 */

  /* USER CODE END CAN2_Init 1 */
  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 1;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = ENABLE;
  hcan2.Init.AutoWakeUp = DISABLE;
  hcan2.Init.AutoRetransmission = DISABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN2_Init 2 */

  /* USER CODE END CAN2_Init 2 */

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
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
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
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
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
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
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
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
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
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
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
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
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
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief TIM13 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM13_Init(void)
{

  /* USER CODE BEGIN TIM13_Init 0 */

  /* USER CODE END TIM13_Init 0 */

  /* USER CODE BEGIN TIM13_Init 1 */

  /* USER CODE END TIM13_Init 1 */
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
  /* USER CODE BEGIN TIM13_Init 2 */

  /* USER CODE END TIM13_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BMS_FLT_EN_GPIO_Port, BMS_FLT_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DEBUG_LED_GPIO_Port, DEBUG_LED_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, SPI_UCOMM_CS_Pin|SPI_FERAM_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI_ADC_CS_GPIO_Port, SPI_ADC_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ADC_RST_GPIO_Port, ADC_RST_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : BMS_FLT_EN_Pin ADC_RST_Pin */
  GPIO_InitStruct.Pin = BMS_FLT_EN_Pin|ADC_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : SHUTDOWN_ACTIVE_Pin LV_PWR_MONITOR_Pin ADC_DRDY_Pin */
  GPIO_InitStruct.Pin = SHUTDOWN_ACTIVE_Pin|LV_PWR_MONITOR_Pin|ADC_DRDY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PRECHARGE_COMPLETE_Pin */
  GPIO_InitStruct.Pin = PRECHARGE_COMPLETE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(PRECHARGE_COMPLETE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DEBUG_LED_Pin */
  GPIO_InitStruct.Pin = DEBUG_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(DEBUG_LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI_UCOMM_CS_Pin SPI_FERAM_CS_Pin */
  GPIO_InitStruct.Pin = SPI_UCOMM_CS_Pin|SPI_FERAM_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI_ADC_CS_Pin */
  GPIO_InitStruct.Pin = SPI_ADC_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI_ADC_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CHARGE_EN_Pin */
  GPIO_InitStruct.Pin = CHARGE_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(CHARGE_EN_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan){
	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, (CAN_RxHeaderTypeDef *) &rx_header, (uint8_t*) rx_data);

	switch(rx_header.StdId){
		case CAN_1_CPU_BMS_VIEWER_POLL_ID:
			pollingFlag = true;
			break;
		case MANUAL_BALANCING_ID:
			new_balance_msg = 1;
			for(int i = 0; i < rx_header.DLC; i++){
				balancing_data_array[i] = rx_data[i];
			}
			break;
		default:
			break;
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if(htim == &htim1){
		HAL_GPIO_WritePin(DEBUG_LED_GPIO_Port, DEBUG_LED_Pin, GPIO_PIN_SET);

		uint32_t new_t = HAL_GetTick();
		timeBetween = new_t - prevTime;
		if(timeBetween >= 11){
			isoADC_period_miss++;
			if(timeBetween > max_isoADC_period){
				max_isoADC_period = (uint8_t) timeBetween;
			}
		}

		prevTime = new_t;
		isoADC_rdy_status = read_isoADC_ADCs(&gIsoADCConfig, &gIsoADCData);
		error_cnt += (uint8_t) (gIsoADCData.ch0_drdy ? 0 : 1);
		convert_raw_to_actual(&gIsoADCConfig, &gIsoADCData);
		BMSCriticalInfo.isoAdcPackVoltage = (float)gIsoADCData.bus_voltage;
		BMSCriticalInfo.packCurrent = (float)gIsoADCData.shunt_current;
		live_pack_voltage = BMSCriticalInfo.isoAdcPackVoltage;
		live_pack_current = BMSCriticalInfo.packCurrent;
		transmit_bms_status_msg(&BMSCriticalInfo, BMS_STATUS);
		HAL_GPIO_WritePin(DEBUG_LED_GPIO_Port, DEBUG_LED_Pin, GPIO_PIN_RESET);

	}
}

void send_can_msg_from_irq(){
	can_tx_phase++;
	switch(can_tx_phase){
		case 1:
			transmit_cell_vlt_msg(&BMSCriticalInfo);
			break;
		case 2:
//			transmit_bms_status_msg(&BMSCriticalInfo, BMS_STATUS);
			break;
		case 3:
			transmit_cell_temp_msg(&BMSCriticalInfo);
			break;
		case 4:
			if ((manual_balancing_config.valid_charge_message == true) &&
				(manual_balancing_config.charge_en == true) &&
				(!bmsFault == true)) {
				CHARGER_message();
			}
			break;
		case 5:
			can_tx_phase = 0;
			break;
	}
}

void send_cell_vals_polling(){
	// Reread 391 to see if this is bad
	DISABLE_ALL_CAN_IRQS
	if (pollingFlag || CONSTANT_CAN_ENABLE){
		transmit_cell_data_msg(bmsData);
		pollingFlag = false;
	}
	charging_msg_timeout = 1;
	ENABLE_ALL_CAN_IRQS
}

static bool transmit_can_message(CAN_HandleTypeDef* hcan, const CAN_TxHeaderTypeDef *tx_header, const uint8_t msg_data[]) {
    uint32_t can_error_code = HAL_CAN_GetError(hcan);

    // If we're in bus off or error passive, skip the 5ms timeout (but try to transmit anyway)
    if (!(can_error_code & (HAL_CAN_ERROR_BOF | HAL_CAN_ERROR_EPV))) {
        // Bus OK - wait for up to 5ms for space to become available in the transmit mailboxes
        uint32_t start_tick = HAL_GetTick();
        while(HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0 && HAL_GetTick() - start_tick < CAN_TX_TIMEOUT) {}
    }

    return HAL_CAN_AddTxMessage(hcan, tx_header, msg_data, &tx_mailbox) == HAL_OK;
}

static void transmit_cell_data_msg(CellData const bmsData[NUM_CELLS]) {
    CAN_TxHeaderTypeDef header = {
        .StdId = CAN_1_BMS_CELL_DATA_ID,
        .DLC = CAN_1_BMS_CELL_DATA_LENGTH,
        .IDE = CAN_ID_STD,
        .RTR = CAN_RTR_DATA,
        .TransmitGlobalTime = DISABLE
    };

    uint8_t data[CAN_1_BMS_CELL_DATA_LENGTH];
    struct can_1_bms_cell_data cell_data_msg = {};

    for (uint8_t cell = 0; cell < NUM_CELLS; cell++) {
        cell_data_msg.idx_cell_data = cell;

        cell_data_msg.vlt_cell_data = (float)bmsData[cell].voltage / LTC6811_ADC_LSB_PER_V;
        cell_data_msg.temp_cell_data = (float)bmsData[cell].temperature / 1000.0F;
        // TODO: remove placeholder values after merging SOC estimation branch
        cell_data_msg.soc_cell_data = 0.0F;
        cell_data_msg.soh_cell_data = 0.0F;

        // TODO: make bmsData[cell].fault a bitfield
        cell_data_msg.cell_fault_disconnected = (bool)(bmsData[cell].fault & CELL_DISCONNECT_MASK);
        cell_data_msg.cell_fault_dc = (bool)(bmsData[cell].fault & CELL_DCFAULT_MASK);
        cell_data_msg.cell_fault_temp = (bool)(bmsData[cell].fault & CELL_TEMP_FAIL_MASK);
        cell_data_msg.cell_fault_pec = (bool)(bmsData[cell].fault & CELL_PEC_FAIL_MASK);

        can_1_bms_cell_data_pack(data, &cell_data_msg, sizeof(data));
        transmit_can_message(&hcan1, &header, data);
    }
}

static void transmit_bms_status_msg(BMS_critical_info_t const *bms, uint8_t const bmsStatus[6]) {
    // canCounter2++;
    CAN_TxHeaderTypeDef header = {
        .StdId = CAN_1_BMS_STATUS_ID,
        .DLC = CAN_1_BMS_STATUS_LENGTH,
        .IDE = CAN_ID_STD,
        .RTR = CAN_RTR_DATA,
        .TransmitGlobalTime = DISABLE
    };

    uint8_t data[CAN_1_BMS_STATUS_LENGTH];
    struct can_1_bms_status status_msg = {
        .soc_accum = 0.0F,
        .cur_accum = (float)bms->packCurrent,
        .vlt_accum = (float)bms->isoAdcPackVoltage,
		.vlt_accum_6811 = (float)bms->cellMonitorPackVoltage / 10.0F,

        // TODO: refactor tf out of this lol
        .bms_fault_ovp = bmsStatus[0] & 0x01,
        .bms_fault_uvp = bmsStatus[0] & 0x02,
        .bms_fault_otp = bmsStatus[0] & 0x04,
        .bms_fault_utp = bmsStatus[0] & 0x08,

        // TODO: do we really not read this anywhere else in code? also active high/low?
        .precharge_cplt = !(HAL_GPIO_ReadPin(PRECHARGE_COMPLETE_GPIO_Port, PRECHARGE_COMPLETE_Pin))
    };

    can_1_bms_status_pack(data, &status_msg, sizeof(data));
    transmit_can_message(&hcan1, &header, data);
}

static void transmit_cell_vlt_msg(BMS_critical_info_t const *bms) {
    CAN_TxHeaderTypeDef header = {
        .StdId = CAN_1_BMS_CELL_VLT_ID,
        .DLC = CAN_1_BMS_CELL_VLT_LENGTH,
        .IDE = CAN_ID_STD,
        .RTR = CAN_RTR_DATA,
        .TransmitGlobalTime = DISABLE
    };

    uint8_t data[CAN_1_BMS_CELL_VLT_LENGTH];
    struct can_1_bms_cell_vlt cell_vlt_msg = {
        .vlt_cell_max = (float)bms->curr_max_voltage / LTC6811_ADC_LSB_PER_V,
        .vlt_cell_min = (float)bms->curr_min_voltage / LTC6811_ADC_LSB_PER_V,
        // TODO: make cell indices consistent
        .idx_vlt_max = bms->max_volt_cell,
        .idx_vlt_min = bms->min_volt_cell
    };

    can_1_bms_cell_vlt_pack(data, &cell_vlt_msg, sizeof(data));
    transmit_can_message(&hcan1, &header, data);
}

static void transmit_cell_temp_msg(BMS_critical_info_t const *bms) {
    CAN_TxHeaderTypeDef header = {
        .StdId = CAN_1_BMS_CELL_TEMP_ID,
        .DLC = CAN_1_BMS_CELL_TEMP_LENGTH,
        .IDE = CAN_ID_STD,
        .RTR = CAN_RTR_DATA,
        .TransmitGlobalTime = DISABLE
    };

    uint8_t data[CAN_1_BMS_CELL_TEMP_LENGTH];
    struct can_1_bms_cell_temp cell_temp_msg = {
        // TODO
        .temp_cell_max = (float)bms->curr_max_temp / 1000.0F,
        .temp_cell_min = (float)bms->curr_min_temp / 1000.0F,
        // TODO: make cell indices consistent
        .idx_temp_max = bms->max_temp_cell,
        .idx_temp_min = bms->min_temp_cell
    };

    can_1_bms_cell_temp_pack(data, &cell_temp_msg, sizeof(data));
    transmit_can_message(&hcan1, &header, data);

    // TODO: wtf does this do?? 2024 accumulator fans?
    // Insert PWM code
    // if ((maxT < 17400) || bmsFault) {
    //     HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
    // } else {
    //     HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);
    //     // HAL_GPIO_TogglePin(DEBUG_GPIO_Port, DEBUG_Pin);	// which PIN is this? 
    // }
}

void CHARGER_message() {
	tx_header.StdId = 0;  // Set to 0 for extended IDs
	tx_header.ExtId = CHARGER_IN_ID; // Set your 29-bit ID
	tx_header.DLC = 8;				// set command length
	tx_header.IDE = CAN_ID_EXT;       // Set to extended ID

	uint8_t CHARGER_DATA[8];

	if ((manual_balancing_config.charge_en == true) &&
		(manual_balancing_config.valid_charge_message == true) &&
		(manual_balancing_config.precharge_cplt == true)) {
		CHARGER_DATA[0] = (uint8_t)((manual_balancing_config.charge_voltage >> 8) & 0xFF);
		CHARGER_DATA[1] = (uint8_t)(manual_balancing_config.charge_voltage & 0xFF);
		CHARGER_DATA[2] = (uint8_t)((manual_balancing_config.charge_current >> 8) & 0xFF);
		CHARGER_DATA[3] = (uint8_t)(manual_balancing_config.charge_current & 0xFF);
	}
	else {
		return;	// abort send
	}

    /* these data bytes are not used */
	CHARGER_DATA[4] = 0x00;
	CHARGER_DATA[5] = 0x00;
	CHARGER_DATA[6] = 0x00;
	CHARGER_DATA[7] = 0x00;

    transmit_can_message(&hcan1, &tx_header, CHARGER_DATA);

    // reset to normal
	tx_header.StdId = 0;  // Set to 0 for extended IDs
	tx_header.ExtId = 0; // Set your 29-bit ID
	tx_header.DLC = 8;				// set command length
	tx_header.IDE = CAN_ID_STD;       // Set to extended ID
}

// reset all variables to default
void resetChargerVariables() {
	manual_balancing_config.charge_en = false;		// disable charge
	manual_balancing_config.charge_voltage = 6000;	// 600V
	manual_balancing_config.charge_current = 0;		// 0A

	manual_balancing_config.discharge_balance_en = false;	// disable discharge balance
	manual_balancing_config.num_cells_discharged_per_secondary = 0;	// no cells to be discharged
	manual_balancing_config.discharge_threshold_voltage = 41600;	// 4.16V

	manual_balancing_config.valid_charge_message = false;	// assume invalid message
}

void process_balancing_msg(){
	manual_balancing_config.charge_en = (balancing_data_array[0] == 0xFF);
	manual_balancing_config.charge_voltage = ((balancing_data_array[1] << 8) | (balancing_data_array[2]));
	manual_balancing_config.charge_current = ((balancing_data_array[3] << 8) | (balancing_data_array[4]));

	manual_balancing_config.discharge_balance_en = (balancing_data_array[5] & 0xF0) == 0xF0;
	manual_balancing_config.num_cells_discharged_per_secondary = (balancing_data_array[5] & 0x0F);
	manual_balancing_config.discharge_threshold_voltage = ((balancing_data_array[6] << 8) | (balancing_data_array[7]));

	// check for validity

	// if both charge and balance are enabled, invalid message
	if ((manual_balancing_config.charge_en == true) && (manual_balancing_config.discharge_balance_en == true)) {
	  manual_balancing_config.valid_charge_message = false;
	  resetChargerVariables();
	}

	// if charge is enabled AND if charge voltage is out of bounds [500-600][V]
	// multiplied by 10 -> [5000 - 6000]
	if ((manual_balancing_config.charge_en == true) && ((manual_balancing_config.charge_voltage < 5000) || (manual_balancing_config.charge_voltage > 6000))) {
	  manual_balancing_config.valid_charge_message = false;
	  resetChargerVariables();
	}

	// if charge is enabled AND if charge current is out of bounds [0-10][A]
	// multiplied by 10 -> [0 - 100]
	if ((manual_balancing_config.charge_en == true) && (manual_balancing_config.charge_current > 100)) {
	  manual_balancing_config.valid_charge_message = false;
	  resetChargerVariables();
	}

	// if cell discharge count is out of bounds
	if ((manual_balancing_config.discharge_balance_en == true) && (manual_balancing_config.num_cells_discharged_per_secondary > 12)) {
	  manual_balancing_config.valid_charge_message = false;
	  resetChargerVariables();
	}

	// if discharge is enabled AND if discharge cell threshold is out of bounds [3.2-4.2][V]
	// multiplied by 10,000 -> [32000 - 42000]
	if ((manual_balancing_config.discharge_balance_en == true) && ((manual_balancing_config.discharge_threshold_voltage < 32000) || (manual_balancing_config.discharge_threshold_voltage > 42000))) {
	  manual_balancing_config.valid_charge_message = false;
	  resetChargerVariables();
	}

	// if all conditions passed, charge message is true
	manual_balancing_config.valid_charge_message = true;
	charging_msg_timeout = 0;

}

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
