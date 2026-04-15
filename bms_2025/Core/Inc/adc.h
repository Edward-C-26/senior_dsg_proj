/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    adc.h
  * @brief   This file contains all the function prototypes for
  *          the adc.c file
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
#ifndef __ADC_H__
#define __ADC_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern ADC_HandleTypeDef hadc1;

/* USER CODE BEGIN Private defines */
/* ADC1_IN1 (PA1) Configuration for shunt current measurement */
#define ADC_SHUNT_CHANNEL       ADC_CHANNEL_1   /* PA1 */
#define ADC_SHUNT_PORT          GPIOA
#define ADC_SHUNT_PIN           GPIO_PIN_1
#define ADC_SHUNT_SAMPLING_TIME ADC_SAMPLETIME_15CYCLES

/* Shunt measurement constants */
#define SHUNT_RESISTOR_OHMS     0.01f   /* 10 mOhm shunt resistor */
#define ADC_REF_VOLTAGE_V       3.3f    /* 3.3V reference */
#define ADC_MAX_VALUE_BITS      4095U   /* 12-bit ADC: 2^12 - 1 */
/* USER CODE END Private defines */

void MX_ADC1_Init(void);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __ADC_H__ */

