/**
 * @file SBUS.c
 * @author 陶子辰 (2405973406@qq.com)
 * @brief SBUS接收
 * @version 1.0
 * @date 2023-07-12
 *
 * @copyright Copyright (c) 2023
 *
 */
/*
*********************************************************************************************************
*                                             INCLUDE FILES
*********************************************************************************************************
*/
#include "SBUS.h"

/*
*********************************************************************************************************
*                                               CONSTANTS
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*                                            	变量定义
*********************************************************************************************************
*/

/// @brief SBUS状态
SBUS_CH SBUS;
/// @brief SBUS接收缓冲区
uint8_t SBUS_RX_BUF[SBUS_BUFFER_SIZE];
uint8_t SBUS_ours_RX_BUF[12];
uint8_t SBUS_ours_RX_BUF1[12];

uint8_t SBUS_Receive[SBUS_Receive_length] = {0};
/// @brief SBUS接收值
uint8_t SBUS_Receive_ch = 0; // 接受值
uint8_t SBUS_ours_Receive_ch ; 
/// @brief SBUS接收标志
uint8_t SBUS_Receive_flag = 0; // 接收标志
/// @brief SBUS接收状态机
uint8_t SBUS_Receive_step = 0; // 状态机
uint8_t SBUS_ours_Receive_step = 0; // 状态机

/// @brief SBUS接收计数
uint8_t SBUS_Receive_count = 0; // 计数
uint8_t SBUS_ours_Receive_count = 0; // 计数

/// @brief SBUS发送数据
uint8_t SBUS_Data_send[69] = {0};
/// @brief SBUS发送数据
static union
{
  uint8_t data[32];
  uint16_t SBUS_Data[16];
} SUS_vaule;
/*
*********************************************************************************************************
*                                               函数实现
*********************************************************************************************************
*/
/**
 * @brief  SBUS接受与解码
 * @param  None
 * @retval None
 */

int8_t SBUS_Analyse_Switch(int16_t SBUS_CHANNEL_VALUE)
{
    if (SBUS_CHANNEL_VALUE < 353 || SBUS_CHANNEL_VALUE > 1695)
        return -1;
    else if (SBUS_CHANNEL_VALUE >= 353 && SBUS_CHANNEL_VALUE <= 600)
        return 0;
    else if (SBUS_CHANNEL_VALUE >= 601 && SBUS_CHANNEL_VALUE <= 1100)
        return 1;
    else if (SBUS_CHANNEL_VALUE >= 1101 && SBUS_CHANNEL_VALUE <= 1695)
        return 2;
    else
        return -1;
}

void SBUS_Analyze(void)
{
  switch (SBUS_Receive_step)
  {
  case 0:
    if (SBUS_Receive_ch == SBUS_FRAME_HEADER)
    {
      SBUS_Receive_step++;
    }
    else
    {
      SBUS_Receive_step = 0;
    }
    break;
  case 1:
    SBUS_RX_BUF[SBUS_Receive_count] = SBUS_Receive_ch;
    SBUS_Receive_count++;
    if (SBUS_Receive_count >= 23)
    {
      SBUS_Receive_count = 0;
      SBUS_Receive_step++;
    }

    break;
  case 2:
    if (SBUS_Receive_ch == SBUS_FRAME_FOOTER)
    {
      SBUS_done = 1;
      if ((SBUS_RX_BUF[22] & 0x4) && (SBUS_RX_BUF[22] & 0x8))
      {
        SBUS.ConnectState = 0;
      }
      else
      {
        SBUS.ConnectState = 100;
      }

      SBUS_Receive_step = 0;
    }
    else
    {
      SBUS_Receive_step = 0;
    }
    break;
  }
}

void SBUS_Send(void)
{
  SUS_vaule.SBUS_Data[0] = SBUS.CH1;
  SUS_vaule.SBUS_Data[1] = SBUS.CH2;
  SUS_vaule.SBUS_Data[2] = SBUS.CH3;
  SUS_vaule.SBUS_Data[3] = SBUS.CH4;
  SUS_vaule.SBUS_Data[4] = SBUS.CH5;
  SUS_vaule.SBUS_Data[5] = SBUS.CH6;
  SUS_vaule.SBUS_Data[6] = SBUS.CH7;
  SUS_vaule.SBUS_Data[7] = SBUS.CH8;
  SUS_vaule.SBUS_Data[8] = SBUS.CH9;
  SUS_vaule.SBUS_Data[9] = SBUS.CH10;
  SUS_vaule.SBUS_Data[10] = SBUS.CH11;
  SUS_vaule.SBUS_Data[11] = SBUS.CH12;
  SUS_vaule.SBUS_Data[12] = SBUS.CH13;
  SUS_vaule.SBUS_Data[13] = SBUS.CH14;
  SUS_vaule.SBUS_Data[14] = SBUS.CH15;
  SUS_vaule.SBUS_Data[15] = SBUS.CH16;

  SBUS_Data_send[0] = 'H';
  SBUS_Data_send[1] = 'D';
  for (int i = 2; i <= 33; ++i)
  {
    SBUS_Data_send[i] = SUS_vaule.data[i - 2];
  }


  SBUS_Data_send[34] = (uint8_t)ad7606_value[0];
  SBUS_Data_send[35] = (uint8_t)(ad7606_value[0] >> 8);
  SBUS_Data_send[36] = (uint8_t)ad7606_value[1];
  SBUS_Data_send[37] = (uint8_t)(ad7606_value[1] >> 8);
  SBUS_Data_send[38] = (uint8_t)ad7606_value[2];
  SBUS_Data_send[39] = (uint8_t)(ad7606_value[2] >> 8);  
  SBUS_Data_send[40] = (uint8_t)ad7606_value[3];
  SBUS_Data_send[41] = (uint8_t)(ad7606_value[3] >> 8);  
  for (int i = 42; i <= 49; ++i)
  {
    SBUS_Data_send[i] = imu_data.data[i - 42];
  }  
  SBUS_Data_send[50] = staus_dipan;
  SBUS_Data_send[51] = action_us[0];
  SBUS_Data_send[52] = action_us[1];
  SBUS_Data_send[53] = action_us[2];
  SBUS_Data_send[54] = action_us[3];
  SBUS_Data_send[55] = SBUS_ours_RX_BUF1[0];
  SBUS_Data_send[56] = SBUS_ours_RX_BUF1[1];
  SBUS_Data_send[57] = SBUS_ours_RX_BUF1[2];
  SBUS_Data_send[58] = SBUS_ours_RX_BUF1[3];
  SBUS_Data_send[59] = SBUS_ours_RX_BUF1[4];
  SBUS_Data_send[60] = SBUS_ours_RX_BUF1[5];
  SBUS_Data_send[61] = SBUS_ours_RX_BUF1[6];
  SBUS_Data_send[62] = SBUS_ours_RX_BUF1[7];
  SBUS_Data_send[63] = SBUS_ours_RX_BUF1[8];
  SBUS_Data_send[64] = SBUS_ours_RX_BUF1[9];
  SBUS_Data_send[65] = SBUS_ours_RX_BUF1[10];
  SBUS_Data_send[66] = SBUS_ours_RX_BUF1[11];


  SBUS_Data_send[67] = 'L';
  SBUS_Data_send[68] = 'T';
  HAL_UART_Transmit(&huart7, SBUS_Data_send, 69, 0x3ff);
  flag_ad7606 = 1;
}


void SBUS_ours_Analyze(void){
   switch (SBUS_ours_Receive_step)
  {
  case 0:
    if (SBUS_ours_Receive_ch == SBUS_ours_FRAME_HEADER1)
    {
      SBUS_ours_Receive_step++;
    }
    else
    {
      SBUS_ours_Receive_step = 0;
    }
    break;
      case 1:
    if (SBUS_ours_Receive_ch == SBUS_ours_FRAME_HEADER2)
    {
      SBUS_ours_Receive_step++;
    }
    else
    {
      SBUS_ours_Receive_step = 0;
    }
    break;
  case 2:
    SBUS_ours_RX_BUF[SBUS_ours_Receive_count] = SBUS_ours_Receive_ch;
    SBUS_ours_Receive_count++;
    if (SBUS_ours_Receive_count == 12)
    {
      SBUS_ours_Receive_count = 0;
      SBUS_ours_Receive_step++;
    }

    break;
     case 3:
    if (SBUS_ours_Receive_ch == SBUS_ours_FRAME_FOOTER1)
    {
      SBUS_ours_Receive_step++;
    }
    else
    {
      SBUS_ours_Receive_step = 0;
    }
    break;
  case 4:
    if (SBUS_ours_Receive_ch == SBUS_ours_FRAME_FOOTER2)
    {
      for(uint8_t i = 0;i<12;i++){
      SBUS_ours_RX_BUF1[i] = SBUS_ours_RX_BUF[i];
      }
      SBUS_ours_Receive_step = 0;
    }
    else
    {
      SBUS_ours_Receive_step = 0;
    }
    break;
  }
}