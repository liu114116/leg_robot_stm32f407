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
#define M2_EN_Pin GPIO_PIN_2
#define M2_EN_GPIO_Port GPIOE
#define M2_NFLT_Pin GPIO_PIN_3
#define M2_NFLT_GPIO_Port GPIOE
#define M3_EN_Pin GPIO_PIN_4
#define M3_EN_GPIO_Port GPIOE
#define M3_NFLT_Pin GPIO_PIN_5
#define M3_NFLT_GPIO_Port GPIOE
#define KEY_Pin GPIO_PIN_13
#define KEY_GPIO_Port GPIOC
#define LED1_Pin GPIO_PIN_0
#define LED1_GPIO_Port GPIOC
#define LED2_Pin GPIO_PIN_1
#define LED2_GPIO_Port GPIOC
#define LED3_Pin GPIO_PIN_2
#define LED3_GPIO_Port GPIOC
#define M4_IN3_Pin GPIO_PIN_0
#define M4_IN3_GPIO_Port GPIOA
#define M4_IN2_Pin GPIO_PIN_1
#define M4_IN2_GPIO_Port GPIOA
#define M4_IN1_Pin GPIO_PIN_2
#define M4_IN1_GPIO_Port GPIOA
#define M2_IN3_Pin GPIO_PIN_6
#define M2_IN3_GPIO_Port GPIOA
#define M2_IN2_Pin GPIO_PIN_7
#define M2_IN2_GPIO_Port GPIOA
#define M4_AS5600_SDA_Pin GPIO_PIN_14
#define M4_AS5600_SDA_GPIO_Port GPIOE
#define M4_AS5600_SCl_Pin GPIO_PIN_15
#define M4_AS5600_SCl_GPIO_Port GPIOE
#define M1_IN1_Pin GPIO_PIN_10
#define M1_IN1_GPIO_Port GPIOB
#define M3_AS5600_SDA_Pin GPIO_PIN_14
#define M3_AS5600_SDA_GPIO_Port GPIOB
#define M3_AS5600_SCL_Pin GPIO_PIN_15
#define M3_AS5600_SCL_GPIO_Port GPIOB
#define M3_IN3_Pin GPIO_PIN_12
#define M3_IN3_GPIO_Port GPIOD
#define M3_IN2_Pin GPIO_PIN_13
#define M3_IN2_GPIO_Port GPIOD
#define M3_IN1_Pin GPIO_PIN_14
#define M3_IN1_GPIO_Port GPIOD
#define M2_IN1_Pin GPIO_PIN_8
#define M2_IN1_GPIO_Port GPIOC
#define M2_AS5600_SDA_Pin GPIO_PIN_9
#define M2_AS5600_SDA_GPIO_Port GPIOC
#define M2_AS5600_SCL_Pin GPIO_PIN_8
#define M2_AS5600_SCL_GPIO_Port GPIOA
#define HC05_TX_Pin GPIO_PIN_9
#define HC05_TX_GPIO_Port GPIOA
#define HC05_RX_Pin GPIO_PIN_10
#define HC05_RX_GPIO_Port GPIOA
#define M1_IN3_Pin GPIO_PIN_15
#define M1_IN3_GPIO_Port GPIOA
#define STS_TX_Pin GPIO_PIN_12
#define STS_TX_GPIO_Port GPIOC
#define STS_RX_Pin GPIO_PIN_2
#define STS_RX_GPIO_Port GPIOD
#define M1_IN2_Pin GPIO_PIN_3
#define M1_IN2_GPIO_Port GPIOB
#define M1_AS5600_SCL_Pin GPIO_PIN_6
#define M1_AS5600_SCL_GPIO_Port GPIOB
#define M1_AS5600_SDA_Pin GPIO_PIN_7
#define M1_AS5600_SDA_GPIO_Port GPIOB
#define M4_EN_Pin GPIO_PIN_8
#define M4_EN_GPIO_Port GPIOB
#define M4_NFLT_Pin GPIO_PIN_9
#define M4_NFLT_GPIO_Port GPIOB
#define M1_EN_Pin GPIO_PIN_0
#define M1_EN_GPIO_Port GPIOE
#define M1_NFLT_Pin GPIO_PIN_1
#define M1_NFLT_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */
#define constrain(value, min, max)   ((value) < (min) ? (min) : ((value) > (max) ? (max) : (value)))
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
