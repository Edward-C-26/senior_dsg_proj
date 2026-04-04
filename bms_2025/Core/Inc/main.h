/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
#define DISABLE_ALL_IRQS     do { } while (0)
#define ENABLE_ALL_IRQS      do { } while (0)


/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
extern SPI_HandleTypeDef* ltc_spi;
extern int16_t poll_cell_temps;
extern int16_t poll_cell_voltages;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim7;
extern TIM_HandleTypeDef htim13;
extern int32_t fault_timer;

void send_cell_vals_polling();

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BMS_FLT_EN_Pin GPIO_PIN_13
#define BMS_FLT_EN_GPIO_Port GPIOC
#define LSE_IN_Pin GPIO_PIN_14
#define LSE_IN_GPIO_Port GPIOC
#define LSE_OUT_Pin GPIO_PIN_15
#define LSE_OUT_GPIO_Port GPIOC
#define HSE_IN_Pin GPIO_PIN_0
#define HSE_IN_GPIO_Port GPIOH
#define HSE_OUT_Pin GPIO_PIN_1
#define HSE_OUT_GPIO_Port GPIOH
#define SHUTDOWN_ACTIVE_Pin GPIO_PIN_3
#define SHUTDOWN_ACTIVE_GPIO_Port GPIOC
#define PRECHARGE_COMPLETE_Pin GPIO_PIN_1
#define PRECHARGE_COMPLETE_GPIO_Port GPIOA
#define DEBUG_LED_Pin GPIO_PIN_2
#define DEBUG_LED_GPIO_Port GPIOA
#define SPI_UCOMM_CS_Pin GPIO_PIN_4
#define SPI_UCOMM_CS_GPIO_Port GPIOA
#define SPI_UCOMM_SCK_Pin GPIO_PIN_5
#define SPI_UCOMM_SCK_GPIO_Port GPIOA
#define SPI_UCOMM_MISO_Pin GPIO_PIN_6
#define SPI_UCOMM_MISO_GPIO_Port GPIOA
#define SPI_UCOMM_MOSI_Pin GPIO_PIN_7
#define SPI_UCOMM_MOSI_GPIO_Port GPIOA
#define LV_PWR_MONITOR_Pin GPIO_PIN_4
#define LV_PWR_MONITOR_GPIO_Port GPIOC
#define SPI_ADC_CS_Pin GPIO_PIN_12
#define SPI_ADC_CS_GPIO_Port GPIOB
#define SPI_ADC_SCK_Pin GPIO_PIN_13
#define SPI_ADC_SCK_GPIO_Port GPIOB
#define SPI_ADC_MISO_Pin GPIO_PIN_14
#define SPI_ADC_MISO_GPIO_Port GPIOB
#define SPI_ADC_MOSI_Pin GPIO_PIN_15
#define SPI_ADC_MOSI_GPIO_Port GPIOB
#define ADC_CLKIN_Pin GPIO_PIN_6
#define ADC_CLKIN_GPIO_Port GPIOC
#define ADC_DRDY_Pin GPIO_PIN_7
#define ADC_DRDY_GPIO_Port GPIOC
#define ADC_RST_Pin GPIO_PIN_8
#define ADC_RST_GPIO_Port GPIOC
#define CHARGE_EN_Pin GPIO_PIN_8
#define CHARGE_EN_GPIO_Port GPIOA
#define SWDIO_TC_Pin GPIO_PIN_13
#define SWDIO_TC_GPIO_Port GPIOA
#define SWCLK_TC_Pin GPIO_PIN_14
#define SWCLK_TC_GPIO_Port GPIOA
#define SPI_FERAM_CS_Pin GPIO_PIN_15
#define SPI_FERAM_CS_GPIO_Port GPIOA
#define SPI_FERAM_SCK_Pin GPIO_PIN_10
#define SPI_FERAM_SCK_GPIO_Port GPIOC
#define SPI_FERAM_MISO_Pin GPIO_PIN_11
#define SPI_FERAM_MISO_GPIO_Port GPIOC
#define SPI_FERAM_MOSI_Pin GPIO_PIN_12
#define SPI_FERAM_MOSI_GPIO_Port GPIOC
#define IMD_DIAGNOSTIC_Pin GPIO_PIN_3
#define IMD_DIAGNOSTIC_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

// This struct holds the configuration variables required for manual balancing
// https://confluence.illinielectricmotorsports.com/display/SOF/Manual+Balancing+Implementation
typedef struct {
	bool charge_en;				// defaults to false
	uint16_t charge_voltage;	// defaults to 600V | multiplied by 10 | 600V -> 6000 in the variable
	uint16_t charge_current;	// defaults to 0A	| multiplied by 10 | 10A -> 100 in the variable

	bool discharge_balance_en;	// defaults to false
	uint8_t num_cells_discharged_per_secondary;	// defaults to 0, must be set between 1 to 12 when discharge_balance_en == true
	uint16_t discharge_threshold_voltage;	// defaults to 4.16V | multiplied by 10,000 | 4.16V -> 41600 in the variable

	bool precharge_cplt;		// defaults to false

	bool valid_charge_message;

} manual_balancing_t;

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
