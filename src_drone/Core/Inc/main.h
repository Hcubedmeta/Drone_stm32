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
#include "stm32f1xx_hal.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define GREEN_LED_Pin GPIO_PIN_13
#define GREEN_LED_GPIO_Port GPIOC
#define RED_LED_Pin GPIO_PIN_14
#define RED_LED_GPIO_Port GPIOC
#define PI_TX_Pin GPIO_PIN_2
#define PI_TX_GPIO_Port GPIOA
#define PI_RX_Pin GPIO_PIN_3
#define PI_RX_GPIO_Port GPIOA
#define CH3_Pin GPIO_PIN_6
#define CH3_GPIO_Port GPIOA
#define CH4_Pin GPIO_PIN_7
#define CH4_GPIO_Port GPIOA
#define CH5_Pin GPIO_PIN_0
#define CH5_GPIO_Port GPIOB
#define CH6_Pin GPIO_PIN_1
#define CH6_GPIO_Port GPIOB
#define MOTOR_1_Pin GPIO_PIN_8
#define MOTOR_1_GPIO_Port GPIOA
#define MOTOR_2_Pin GPIO_PIN_9
#define MOTOR_2_GPIO_Port GPIOA
#define MOTOR_3_Pin GPIO_PIN_10
#define MOTOR_3_GPIO_Port GPIOA
#define MOTOR_4_Pin GPIO_PIN_11
#define MOTOR_4_GPIO_Port GPIOA
#define CH1_Pin GPIO_PIN_15
#define CH1_GPIO_Port GPIOA
#define CH2_Pin GPIO_PIN_3
#define CH2_GPIO_Port GPIOB
#define GPS_TX_Pin GPIO_PIN_6
#define GPS_TX_GPIO_Port GPIOB
#define GPS_RX_Pin GPIO_PIN_7
#define GPS_RX_GPIO_Port GPIOB
#define MPU_MS_SCL_Pin GPIO_PIN_8
#define MPU_MS_SCL_GPIO_Port GPIOB
#define MPU_MS_SDA_Pin GPIO_PIN_9
#define MPU_MS_SDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
