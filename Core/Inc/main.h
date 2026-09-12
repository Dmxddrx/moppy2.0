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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define M3_AIN1_Pin GPIO_PIN_4
#define M3_AIN1_GPIO_Port GPIOE
#define M3_AIN2_Pin GPIO_PIN_5
#define M3_AIN2_GPIO_Port GPIOE
#define M4_BIN1_Pin GPIO_PIN_6
#define M4_BIN1_GPIO_Port GPIOE
#define M2_BIN2_Pin GPIO_PIN_0
#define M2_BIN2_GPIO_Port GPIOC
#define M2_BIN1_Pin GPIO_PIN_1
#define M2_BIN1_GPIO_Port GPIOC
#define M1_PWM_Pin GPIO_PIN_6
#define M1_PWM_GPIO_Port GPIOA
#define M2_PWM_Pin GPIO_PIN_7
#define M2_PWM_GPIO_Port GPIOA
#define M1_AIN1_Pin GPIO_PIN_4
#define M1_AIN1_GPIO_Port GPIOC
#define M1_AIN2_Pin GPIO_PIN_5
#define M1_AIN2_GPIO_Port GPIOC
#define M5_PWM_Pin GPIO_PIN_0
#define M5_PWM_GPIO_Port GPIOB
#define M4_BIN2_Pin GPIO_PIN_7
#define M4_BIN2_GPIO_Port GPIOE
#define ENC1_A_Pin GPIO_PIN_9
#define ENC1_A_GPIO_Port GPIOE
#define ENC1_B_Pin GPIO_PIN_11
#define ENC1_B_GPIO_Port GPIOE
#define MD2_STBY_Pin GPIO_PIN_8
#define MD2_STBY_GPIO_Port GPIOD
#define MD3_STBY_Pin GPIO_PIN_9
#define MD3_STBY_GPIO_Port GPIOD
#define MSD_PWM_Pin GPIO_PIN_12
#define MSD_PWM_GPIO_Port GPIOD
#define M3_PWM_Pin GPIO_PIN_13
#define M3_PWM_GPIO_Port GPIOD
#define M4_PWM_Pin GPIO_PIN_14
#define M4_PWM_GPIO_Port GPIOD
#define ENC2_A_Pin GPIO_PIN_6
#define ENC2_A_GPIO_Port GPIOC
#define ENC2_B_Pin GPIO_PIN_7
#define ENC2_B_GPIO_Port GPIOC
#define MSD_RPM_Pin GPIO_PIN_0
#define MSD_RPM_GPIO_Port GPIOD
#define MSD_RPM_EXTI_IRQn EXTI0_IRQn
#define MD1_STBY_Pin GPIO_PIN_2
#define MD1_STBY_GPIO_Port GPIOD
#define M5_AIN1_Pin GPIO_PIN_5
#define M5_AIN1_GPIO_Port GPIOD
#define M5_AIN2_Pin GPIO_PIN_6
#define M5_AIN2_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
