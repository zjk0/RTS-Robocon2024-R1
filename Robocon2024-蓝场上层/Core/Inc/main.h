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
#include "math.h"
#include "stdlib.h"

#include "PID.h"
#include "M3508.h"
#include "CAN.h"
#include "GO-M8010-6.h"
#include "SBUS.h"
#include "USART.h"
#include "RobotControl.h"
#include "ACTION.h"
#include "My_Plan.h"
#include "LASER.h"
#include "JY901.h"
  #include "GM6030.h"
#include "TSL1401.h"
#include <string.h>

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
// extern uint8_t SBUS_Data_Low[17];
// extern uint8_t SBUS_Data_High[17] ;
// extern uint16_t SBUS_Data[17];
// extern uint8_t SBUS_Transmit_Flag;
// extern uint8_t SBUS_H_L;
extern SPI_HandleTypeDef hspi4;




/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define jiguang_duoji_2_Y_Pin GPIO_PIN_7
#define jiguang_duoji_2_Y_GPIO_Port GPIOI
#define jiguang_duoji_1_X_Pin GPIO_PIN_6
#define jiguang_duoji_1_X_GPIO_Port GPIOI
#define guang_dian_kai_guan_Z_Pin GPIO_PIN_2
#define guang_dian_kai_guan_Z_GPIO_Port GPIOI
#define chongzhi_gei_xia_cengQ2_Pin GPIO_PIN_9
#define chongzhi_gei_xia_cengQ2_GPIO_Port GPIOI
#define YUN_TAI_TONG_XIN_1_I1_Pin GPIO_PIN_0
#define YUN_TAI_TONG_XIN_1_I1_GPIO_Port GPIOF
#define qu_qiu_duoji_A_Pin GPIO_PIN_0
#define qu_qiu_duoji_A_GPIO_Port GPIOI
#define YUN_TAI_TONG_XIN_2_I2_Pin GPIO_PIN_1
#define YUN_TAI_TONG_XIN_2_I2_GPIO_Port GPIOF
#define fa_she_qiu_duoji_B_Pin GPIO_PIN_12
#define fa_she_qiu_duoji_B_GPIO_Port GPIOH
#define qumiao_jiazi_duoji2_C_Pin GPIO_PIN_11
#define qumiao_jiazi_duoji2_C_GPIO_Port GPIOH
#define qumiao_jiazi_duoji1_D_Pin GPIO_PIN_10
#define qumiao_jiazi_duoji1_D_GPIO_Port GPIOH
#define qumiao_jiazi_gao_N2_Pin GPIO_PIN_0
#define qumiao_jiazi_gao_N2_GPIO_Port GPIOC
#define AD7606_CS_O2_Pin GPIO_PIN_1
#define AD7606_CS_O2_GPIO_Port GPIOC
#define qumiao_shengjiang_di_L1_Pin GPIO_PIN_2
#define qumiao_shengjiang_di_L1_GPIO_Port GPIOC
#define qumiao_shengjiang_gao_M1_Pin GPIO_PIN_3
#define qumiao_shengjiang_gao_M1_GPIO_Port GPIOC
#define CCD_AO_Pin GPIO_PIN_0
#define CCD_AO_GPIO_Port GPIOA
#define AD7606_RST_P2_Pin GPIO_PIN_4
#define AD7606_RST_P2_GPIO_Port GPIOA
#define quqiu_jiazi_N1_Pin GPIO_PIN_4
#define quqiu_jiazi_N1_GPIO_Port GPIOC
#define AD7606_BUSY_P1_Pin GPIO_PIN_5
#define AD7606_BUSY_P1_GPIO_Port GPIOA
#define AD7606_BUSY_P1_EXTI_IRQn EXTI9_5_IRQn
#define AD7606_CVA_O1_Pin GPIO_PIN_5
#define AD7606_CVA_O1_GPIO_Port GPIOC
#define qumiao_jiazi_di_M2_Pin GPIO_PIN_1
#define qumiao_jiazi_di_M2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

#define CH_TRACE printf

#define CHSYNC1         (0x5A)        /* CHAOHE message sync code 1 */
#define CHSYNC2         (0xA5)        /* CHAOHE message sync code 2 */
#define CH_HDR_SIZE     (0x06)        /* CHAOHE protocol header size */

#define U1(p) (*((uint8_t *)(p)))
#define I1(p) (*((int8_t  *)(p)))
#define I2(p) (*((int16_t  *)(p)))

#define  CH_DEBUG
#define MAXRAWLEN       (512)
extern ADC_HandleTypeDef hadc1;

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;

extern UART_HandleTypeDef huart7;
extern UART_HandleTypeDef huart8;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart6;
extern DMA_HandleTypeDef hdma_usart1_rx;

extern uint16_t ADV[128];
extern uint8_t CCD_Zhongzhi;
extern uint8_t CCD_Yuzhi;  
extern uint16_t error_count;
extern uint8_t flag_ad7606;
extern uint8_t bin;
extern int16_t ad7606_value[4];
extern uint8_t SBUS_done;
extern uint8_t decode_succ ;
extern uint8_t rx_buf[10];
typedef struct
{
    uint32_t    id;            /* user defined ID       */
    int8_t      temperature;
    float       acc[3];           /* acceleration          */
    float       gyr[3];           /* angular velocity      */  
    float       mag[3];           /* magnetic field        */
    float       eul[3];           /* attitude: eular angle */
    float       quat[4];          /* attitude: quaternion  */
    float       pressure;         /* air pressure          */
    uint32_t    timestamp;
}ch_imu_data_t;

typedef struct
{
    int nbyte;                          /* number of bytes in message buffer */ 
    int len;                            /* message length (bytes) */
    uint8_t buf[MAXRAWLEN];             /* message raw buffer */
   
    ch_imu_data_t imu;   /* imu data list, if (HI226/HI229/CH100/CH110, use imu[0]) */
}raw_t;
typedef union imu_data_{
float yaw[2];//第一个是yaw角，第二个是oumiga
uint8_t data[8];
}imu;
extern imu imu_data;
extern raw_t raw;


extern uint8_t staus_dipan;
extern uint8_t action_us[4] ;

extern uint8_t fa_qiu_done;

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
