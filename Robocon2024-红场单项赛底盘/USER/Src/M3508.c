/**
 * @file M3508.c
 * @author 陶子辰 (2405973406@qq.com)
 * @brief M3508电机控制
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

#include "M3508.h"

/*
*********************************************************************************************************
*                                            	变量定义
*********************************************************************************************************
*/
/// M3508电机参数
M3508_MOTOR M3508[9];
uint16_t error_time = 0;
// 速度环PID
PIDType M3508SpeedPID1 = {7, 0.7, 0, 0, 0, 0, 0, 0, 0};
PIDType M3508SpeedPID2 = {7, 0.7, 0, 0, 0, 0, 0, 0, 0};
PIDType M3508SpeedPID3 = {7, 0.7, 0, 0, 0, 0, 0, 0, 0};
//PIDType M3508SpeedPID4 = {8.2, 0.92, 0, 0, 0, 0, 0, 0, 0}; //{6, 0.1, 1, 0, 0, 0, 0};
//PIDType M3508SpeedPID5 = {8.2, 0.92, 0, 0, 0, 0, 0, 0, 0};
//PIDType M3508SpeedPID6 = {8.2, 0.92, 0, 0, 0, 0, 0, 0, 0}; // {13, 0.72, 0, 0, 0, 0, 0};
PIDType M3508SpeedPID4 = {20, 0.1, 0, 0, 0, 0, 0, 0, 0}; //{6, 0.1, 1, 0, 0, 0, 0};
PIDType M3508SpeedPID5 = {20, 0.1, 0, 0, 0, 0, 0, 0, 0};
PIDType M3508SpeedPID6 = {20, 0.1, 0, 0, 0, 0, 0, 0, 0}; // {13, 0.72, 0, 0, 0, 0, 0};
PIDType M3508SpeedPID7 = {10, 0.5, 0, 0, 0, 0, 0, 0, 0};
PIDType M3508SpeedPID8 = {7, 0.7, 0, 0, 0, 0, 0, 0, 0};

// 位置环PID
PIDType M3508PositionPID1 = {0.10, 0.10, 0, 0, 0, 0, 0, 0};
PIDType M3508PositionPID2 = {0.10, 0.10, 0, 0, 0, 0, 0, 0};
PIDType M3508PositionPID3 = {0.10, 0.10, 0, 0, 0, 0, 0, 0};
PIDType M3508PositionPID4 = {1, 0.1, 0, 0, 0, 0, 0, 0};
PIDType M3508PositionPID5 = {0.98, 0.1, 0, 0, 0, 0, 0, 0};
PIDType M3508PositionPID6 = {0.88, 0.3, 0, 0, 0, 0, 0, 0};
PIDType M3508PositionPID7 = {0.9, 0.07, 0, 0, 0, 0, 0, 0};
PIDType M3508PositionPID8 = {0.10, 0.10, 0, 0, 0, 0, 0, 0};

/**
 * @brief 分析C620电调反馈的数据.
 *
 * @param[in] canmsg: CAN消息指针
 *            canmsg->StdId: CAN消息标准ID
 *            canmsg->Data: CAN消息数据
 *            canmsg->Data[0]:转子机械角度高8位
 *            canmsg->Data[1]:转子机械角度低8位
 *            canmsg->Data[2]:转子转速高8位
 *            canmsg->Data[3]:转子转速低8位
 *            canmsg->Data[4]:实际转矩电流高8位
 *            canmsg->Data[5]:实际转矩电流低8位
 *            canmsg->Data[6]:电机温度
 *            canmsg->Data[7]:NULL
 *            canmsg->DLC: CAN消息数据长度
 *            
 * @return none
 *
 * @note 该函数用于分析C620电调反馈的数据，包括转子机械角度、转速、电流、温度等信息。
 *       输入参数canmsg包括CAN消息标准ID、数据和数据长度。
 *       该函数不返回任何值。
 *       发送频率：1KHz
 *       转子机械角度值范围：0~8191（对应0~360°）
 *       转子转速值单位：RPM
 *       电机温度单位：℃
 *
 * @example
 *         CanRxMsg canmsg;
 *         Motor_Analyze(&canmsg);
 */
void M3508_Analyze(uint32_t M3508_StdId,uint8_t* MotorData)
{

  char Motor_Id;
  int16_t MotorVal[9][3];

  Motor_Id = M3508_StdId - 0x200;

  MotorVal[Motor_Id][0] = MotorData[0] << 8 | MotorData[1]; // 转子机械角度
  MotorVal[Motor_Id][1] = MotorData[2] << 8 | MotorData[3]; // 转子转速

  MotorVal[Motor_Id][2] = MotorData[4] << 8 | MotorData[5]; // 转矩电流

  M3508[Motor_Id].posdel = MotorVal[Motor_Id][0] - M3508[Motor_Id].CurPosition;

  if (M3508[Motor_Id].CurPosition == 0)
  {
    M3508[Motor_Id].posdel = 0;
  }

  if (M3508[Motor_Id].posdel >= 4095 && M3508[Motor_Id].CurPosition != 0)
  {
    M3508[Motor_Id].posdel = M3508[Motor_Id].posdel - 8192;
  }
  if (M3508[Motor_Id].posdel <= -4095 && M3508[Motor_Id].CurPosition != 0)
  {
    M3508[Motor_Id].posdel = M3508[Motor_Id].posdel + 8192;
  }

  M3508[Motor_Id].Temparg += M3508[Motor_Id].posdel;
  M3508[Motor_Id].CurPosition = MotorVal[Motor_Id][0];
  M3508[Motor_Id].CurSpeed = MotorVal[Motor_Id][1];
  M3508[Motor_Id].ActCurrent = MotorVal[Motor_Id][2];
}

/**
 * @brief 电机控制函数.
 *
 * @param[in] none
 *            
 * @return none
 *
 * @note 该函数用于实现M3508的速度闭环控制。
 *
 * @example
 *         M3508_Motor_Speed_Ctrl();
 */
void M3508_Motor_Speed_Ctrl(void)
{
    M3508[1].OutCurrent += PIDCal(&M3508SpeedPID1, M3508[1].ExpSpeed - M3508[1].CurSpeed);
    M3508[2].OutCurrent += PIDCal(&M3508SpeedPID2, M3508[2].ExpSpeed - M3508[2].CurSpeed);
    M3508[3].OutCurrent += PIDCal(&M3508SpeedPID3, M3508[3].ExpSpeed - M3508[3].CurSpeed);
    M3508[4].OutCurrent += PIDCal(&M3508SpeedPID4, M3508[4].ExpSpeed - M3508[4].CurSpeed);
    M3508[5].OutCurrent += PIDCal(&M3508SpeedPID5, M3508[5].ExpSpeed - M3508[5].CurSpeed);
    M3508[6].OutCurrent += PIDCal(&M3508SpeedPID6, M3508[6].ExpSpeed - M3508[6].CurSpeed);
    M3508[7].OutCurrent += PIDCal(&M3508SpeedPID7, M3508[7].ExpSpeed - M3508[7].CurSpeed);
    M3508[8].OutCurrent += PIDCal(&M3508SpeedPID8, M3508[8].ExpSpeed - M3508[8].CurSpeed);
}
/**
 * @brief 电机控制函数.
 *
 * @param[in] M3508_id 电机ID
 * @param[in] M3508PositionPIDx 电机位置PID
 *            
 * @return M3508ExpSpeed 电机期望速度
 *
 * @note 该函数用于实现单个M3508的位置闭环控制。
 *
 * @example
 *         M3508_Motor_Signal_Position_Ctrl();
 */
float M3508_Motor_Signal_Position_Ctrl(int M3508_id, PIDType *M3508PositionPIDx)
{
  float M3508ExpSpeed;
  if (abs(M3508[M3508_id].Exparg - M3508[M3508_id].Temparg) < 11000)
  {
    M3508ExpSpeed = PIDCal(M3508PositionPIDx, M3508[M3508_id].Exparg - M3508[M3508_id].Temparg);
  }
  else if (M3508[M3508_id].Exparg > M3508[M3508_id].Temparg)
  {
    M3508ExpSpeed = 1.05 * M3508[M3508_id].Expvel;
  }
  else
  {
    M3508ExpSpeed = -1.05 * M3508[M3508_id].Expvel;
  }
  return M3508ExpSpeed;
}
/**
 * @brief 电机控制函数.
 *
 * @param[in] none
 *            
 * @return none
 *
 * @note 该函数用于实现M3508的位置闭环控制。
 *
 * @example
 *         M3508_Motor_Position_Ctrl();
 */
void M3508_Motor_Position_Ctrl(void)
{
    M3508[1].ExpSpeed = (int16_t)M3508_Motor_Signal_Position_Ctrl(1, &M3508PositionPID1);
    M3508[2].ExpSpeed = (int16_t)M3508_Motor_Signal_Position_Ctrl(2, &M3508PositionPID2);
    M3508[3].ExpSpeed = (int16_t)M3508_Motor_Signal_Position_Ctrl(3, &M3508PositionPID3);
    M3508[4].ExpSpeed = (int16_t)M3508_Motor_Signal_Position_Ctrl(4, &M3508PositionPID4);
    M3508[5].ExpSpeed = (int16_t)M3508_Motor_Signal_Position_Ctrl(5, &M3508PositionPID5);
    M3508[6].ExpSpeed = (int16_t)M3508_Motor_Signal_Position_Ctrl(6, &M3508PositionPID6);
    M3508[7].ExpSpeed = (int16_t)M3508_Motor_Signal_Position_Ctrl(7, &M3508PositionPID7);
    M3508[8].ExpSpeed = (int16_t)M3508_Motor_Signal_Position_Ctrl(8, &M3508PositionPID8);
}

/**
 * @brief 向C620电调发送指令控制电流.
 * 
 * @param[in] none
 * 
 * @return none
 * 
 * @note 该函数用于向C620电调发送指令控制电流。
 *       can_id=0x200（电调ID：1~4）或0x1FF（电调ID：5~8）
 *       控制电流值范围 -16384~16384  对应转矩电流范围 -20~20A
 *       data[0]:控制电流高8位
 *       data[1]:控制电流低8位
 *       data[2]:控制电流高8位
 *       data[3]:控制电流低8位
 *       data[4]:控制电流高8位
 *       data[5]:控制电流低8位
 *       data[6]:控制电流高8位
 *       data[7]:控制电流低8位
 * 
 * @example
 *        M3508_Motor_Send_Current(0x200, 0, 0, 0, 0, 0, 0, 0, 0);
 */
void M3508_Motor_Send_Current(void)
{
    uint8_t i;
    uint32_t send_mail_box;
    CAN_TxHeaderTypeDef TxHeader;
    TxHeader.StdId = 0x200;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.DLC = 0x08;
    TxHeader.TransmitGlobalTime = DISABLE;
    uint8_t Txmsg[8] = {0};

    for (i = 1; i <= 8; i++)
    {
        if (M3508[i].OutCurrent > 16384)
        {
            M3508[i].OutCurrent = 16384;
        }
        else if (M3508[i].OutCurrent < -16384)
        {
            M3508[i].OutCurrent = -16384;
        }
    }
    Txmsg[0] = (uint8_t)((M3508[1].OutCurrent >> 8) & 0xFF);
    Txmsg[1] = (uint8_t)( M3508[1].OutCurrent & 0xFF);
    Txmsg[2] = (uint8_t)((M3508[2].OutCurrent >> 8) & 0xFF);
    Txmsg[3] = (uint8_t)( M3508[2].OutCurrent & 0xFF);
    Txmsg[4] = (uint8_t)((M3508[3].OutCurrent >> 8) & 0xFF);
    Txmsg[5] = (uint8_t)( M3508[3].OutCurrent & 0xFF);
    Txmsg[6] = (uint8_t)((M3508[4].OutCurrent >> 8) & 0xFF);
    Txmsg[7] = (uint8_t)( M3508[4].OutCurrent & 0xFF);
    while(HAL_CAN_AddTxMessage(&hcan1, &TxHeader, Txmsg, &send_mail_box)!=HAL_OK)
    {
       error_time++;// Error_Handler();
    }
}
/**
 * @brief 电机控制函数.
 *
 * @param[in] none
 *            
 * @return none
 *
 * @note 该函数用于实现M3508的电流闭环控制。
 *
 * @example
 *         M3508_Motor_Send_Current_h();
 */
void M3508_Motor_Send_Current_h(void)
{
    uint8_t i;
    uint32_t send_mail_box;
    CAN_TxHeaderTypeDef TxHeader;
    TxHeader.StdId = 0x1FF;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.DLC = 0x08;
    TxHeader.TransmitGlobalTime = DISABLE;
    uint8_t Txmsg[8] = {0};

    for (i = 1; i <= 8; i++)
    {
        if (M3508[i].OutCurrent > 16384)
        {
            M3508[i].OutCurrent = 16384;
        }
        else if (M3508[i].OutCurrent < -16384)
        {
            M3508[i].OutCurrent = -16384;
        }
    }
    Txmsg[0] = (uint8_t)((M3508[5].OutCurrent >> 8) & 0xFF);
    Txmsg[1] = (uint8_t)( M3508[5].OutCurrent & 0xFF);
    Txmsg[2] = (uint8_t)((M3508[6].OutCurrent >> 8) & 0xFF);
    Txmsg[3] = (uint8_t)( M3508[6].OutCurrent & 0xFF);
    Txmsg[4] = (uint8_t)((M3508[7].OutCurrent >> 8) & 0xFF);
    Txmsg[5] = (uint8_t)( M3508[7].OutCurrent & 0xFF);
    Txmsg[6] = (uint8_t)((M3508[8].OutCurrent >> 8) & 0xFF);
    Txmsg[7] = (uint8_t)( M3508[8].OutCurrent & 0xFF);
	while(HAL_CAN_AddTxMessage(&hcan1, &TxHeader, Txmsg, &send_mail_box)!= HAL_OK){
		error_time++;//Error_Handler();
	}
}