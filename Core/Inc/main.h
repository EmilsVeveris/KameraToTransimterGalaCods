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
#include "stm32u5xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SD_DETECTED_Pin GPIO_PIN_1
#define SD_DETECTED_GPIO_Port GPIOC
#define SD_CS_Pin GPIO_PIN_2
#define SD_CS_GPIO_Port GPIOA
#define Mosfet_controll_Pin GPIO_PIN_4
#define Mosfet_controll_GPIO_Port GPIOC
#define H_BRIDGE_nSLEEP_Pin GPIO_PIN_10
#define H_BRIDGE_nSLEEP_GPIO_Port GPIOE
#define H_BRIDGE_IN1_Pin GPIO_PIN_11
#define H_BRIDGE_IN1_GPIO_Port GPIOE
#define H_BRIDGE_EN1_Pin GPIO_PIN_12
#define H_BRIDGE_EN1_GPIO_Port GPIOE
#define H_BRIDGE_IN2_Pin GPIO_PIN_13
#define H_BRIDGE_IN2_GPIO_Port GPIOE
#define H_BRIDGE_EN2_Pin GPIO_PIN_14
#define H_BRIDGE_EN2_GPIO_Port GPIOE
#define SPIRIT1_CSn_Pin GPIO_PIN_12
#define SPIRIT1_CSn_GPIO_Port GPIOB
#define SPIRIT1_GPIO3_Pin GPIO_PIN_8
#define SPIRIT1_GPIO3_GPIO_Port GPIOD
#define SPIRIT1_GPIO3_EXTI_IRQn EXTI8_IRQn
#define SPIRIT1_SDN_Pin GPIO_PIN_9
#define SPIRIT1_SDN_GPIO_Port GPIOD
#define DIODE1_Pin GPIO_PIN_10
#define DIODE1_GPIO_Port GPIOD
#define DIODE2_Pin GPIO_PIN_11
#define DIODE2_GPIO_Port GPIOD
#define DIODE3_Pin GPIO_PIN_12
#define DIODE3_GPIO_Port GPIOD
#define DIODE4_Pin GPIO_PIN_13
#define DIODE4_GPIO_Port GPIOD
#define DIODE5_Pin GPIO_PIN_14
#define DIODE5_GPIO_Port GPIOD
#define DIODE6_Pin GPIO_PIN_15
#define DIODE6_GPIO_Port GPIOD
#define uSD_DETECT_Pin GPIO_PIN_0
#define uSD_DETECT_GPIO_Port GPIOD
#define CAM_CS_Pin GPIO_PIN_7
#define CAM_CS_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
