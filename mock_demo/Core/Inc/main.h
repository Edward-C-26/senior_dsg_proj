/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
extern SPI_HandleTypeDef *ltc_spi;
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BMS_FLT_EN_Pin GPIO_PIN_13
#define BMS_FLT_EN_GPIO_Port GPIOC
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define EXT_USART_TX_Pin GPIO_PIN_9
#define EXT_USART_TX_GPIO_Port GPIOA
#define EXT_USART_RX_Pin GPIO_PIN_10
#define EXT_USART_RX_GPIO_Port GPIOA
#define SPI_UCOMM_CS_Pin GPIO_PIN_4
#define SPI_UCOMM_CS_GPIO_Port GPIOA
#define SPI_UCOMM_SCK_Pin GPIO_PIN_5
#define SPI_UCOMM_SCK_GPIO_Port GPIOA
#define SPI_UCOMM_MISO_Pin GPIO_PIN_6
#define SPI_UCOMM_MISO_GPIO_Port GPIOA
#define SPI_UCOMM_MOSI_Pin GPIO_PIN_7
#define SPI_UCOMM_MOSI_GPIO_Port GPIOA
#define CHARGE_CURRENT_ADC_Pin GPIO_PIN_1
#define CHARGE_CURRENT_ADC_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
