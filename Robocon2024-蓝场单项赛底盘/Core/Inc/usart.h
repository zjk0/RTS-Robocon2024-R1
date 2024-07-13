/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.h
  * @brief   This file contains all the function prototypes for
  *          the usart.c file
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
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern UART_HandleTypeDef huart1;

extern UART_HandleTypeDef huart2;

extern UART_HandleTypeDef huart3;

extern UART_HandleTypeDef huart6;

/* USER CODE BEGIN Private defines */
#define CCD_Head  0x48
#define CCD_Head_ 0x44
#define CCD_Tail  0x4c
#define CCD_Tail_ 0x54

extern uint8_t Paw_Command[1];
extern uint8_t Disable_Command[1];
extern uint8_t Enable_Command[1];
extern uint8_t GPIO_Pull_Down_Command[1];
extern uint8_t Receive_Command[1];
extern uint8_t Ack[1];
extern uint8_t rotate_servo_command[1];
extern uint8_t get_ball[1];
extern uint8_t shoot_ball[1];
extern uint8_t ShootBall_Num[6];
extern uint8_t area_2[1];
// extern uint8_t get_ball_1_command[1];
// extern uint8_t get_ball_2_command[1];
// extern uint8_t get_ball_3_command[1];
// extern uint8_t get_ball_4_command[1];
// extern uint8_t get_ball_5_command[1];
// extern uint8_t get_ball_6_command[1];

extern uint8_t Disable_Flag;
extern uint8_t Enable_Flag;

/* USER CODE END Private defines */

void MX_USART1_UART_Init(void);
void MX_USART2_UART_Init(void);
void MX_USART3_UART_Init(void);
void MX_USART6_UART_Init(void);

/* USER CODE BEGIN Prototypes */
extern uint8_t serialplot_data[15];

void Send_SerialPlot_Data_Speed(int16_t CurSpeed, int16_t ExpSpeed);
void Send_SerialPlot_Data_Angle(float Temparg, float Exparg);
void Send_SerialPlot_Data_Two_Speed(int16_t CurSpeed1, int16_t ExpSpeed1, int16_t CurSpeed2, int16_t ExpSpeed2);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

