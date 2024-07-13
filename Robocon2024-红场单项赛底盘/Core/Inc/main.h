/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "can.h"
#include "usart.h"
#include "gpio.h"
#include "math.h"
#include "stdlib.h"
#include "tim.h"

#include "RobotControl.h"
#include "PID.h"
#include "M3508.h"
#include "ODRIVE.h"
#include "GO-M8010-6.h"
#include "SBUS.h"
#include "ACTION.h"

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
#define MOTOR7_ZERO_Pin GPIO_PIN_7
#define MOTOR7_ZERO_GPIO_Port GPIOI
#define MOTOR6_ZERO_Pin GPIO_PIN_6
#define MOTOR6_ZERO_GPIO_Port GPIOI
#define MOTOR5_ZERO_Pin GPIO_PIN_6
#define MOTOR5_ZERO_GPIO_Port GPIOC
#define NOTE_POSITION_Pin GPIO_PIN_9
#define NOTE_POSITION_GPIO_Port GPIOE
#define NOTE_POSITION_EXTI_IRQn EXTI9_5_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
