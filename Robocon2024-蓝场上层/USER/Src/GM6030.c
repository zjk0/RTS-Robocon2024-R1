/*
*********************************************************************************************************
*                                             INCLUDE FILES
*********************************************************************************************************
*/

#include "GM6030.h"

/*
*********************************************************************************************************
*                                            	变量定义
*********************************************************************************************************
*/
uint8_t jishu2 = 0;

GM6030_MOTOR GM6030[9];
//速度环PID
PIDType GM6030SpeedPID1 = {7, 0.7, 0, 0, 0, 0, 0}; 
PIDType GM6030SpeedPID2 = {7, 0.7, 0, 0, 0, 0, 0}; 
PIDType GM6030SpeedPID3 = {7, 0.7, 0, 0, 0, 0, 0};
PIDType GM6030SpeedPID4 = {7, 0.7, 0, 0, 0, 0, 0};
PIDType GM6030SpeedPID5 = {7, 12, 10.3, 0, 0, 0, 0}; 
PIDType GM6030SpeedPID6 = {13, 0.72, 0, 0, 0, 0, 0};
PIDType GM6030SpeedPID7 = {7, 0.7, 0, 0, 0, 0, 0};
PIDType GM6030SpeedPID8 = {7, 0.7, 0, 0, 0, 0, 0};
// PIDType Motor5PID =  {17.5, 0.85,   0,0,0,0,0};
// PIDType Motor6PID =  {17.5, 0.85,   0,x0,0,0,0};
// PIDType Motor7PID =  {17.5, 0.85,   0,0,0,0,0};
// PIDType Motor8PID =  {17.5, 0.85,   0,0,0,0,0};

// 位置环PID
PIDType GM6030PositionPID1 = {0.10, 0.10, 0, 0, 0, 0, 0};
PIDType GM6030PositionPID2 = {0.10, 0.10, 0, 0, 0, 0, 0};
PIDType GM6030PositionPID3 = {0.10, 0.10, 0, 0, 0, 0, 0};
PIDType GM6030PositionPID4 = {0.10, 0.10, 0, 0, 0, 0, 0};
PIDType GM6030PositionPID5 = {0.1, 0.05, 0, 0, 0, 0, 0};
PIDType GM6030PositionPID6 = {0.2, 0.10, 0, 0, 0, 0, 0};
PIDType GM6030PositionPID7 = {0.05, 0.04, 0.03, 0, 0, 0, 0};
PIDType GM6030PositionPID8 = {0.10, 0.10, 0, 0, 0, 0, 0};



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
void GM6030_Analyze(uint32_t GM6030_StdId,uint8_t* MotorData)
{

  char Motor_Id;
  int16_t MotorVal[9][3];

  Motor_Id = GM6030_StdId - 0x204;

  MotorVal[Motor_Id][0] = MotorData[0] << 8 | MotorData[1]; // 转子机械角度
  MotorVal[Motor_Id][1] = MotorData[2] << 8 | MotorData[3]; // 转子转速

  MotorVal[Motor_Id][2] = MotorData[4] << 8 | MotorData[5]; // 转矩电流

  GM6030[Motor_Id].posdel = MotorVal[Motor_Id][0] - GM6030[Motor_Id].CurPosition;

  if (GM6030[Motor_Id].CurPosition == 0)
  {
    GM6030[Motor_Id].posdel = 0;
  }

  if (GM6030[Motor_Id].posdel >= 4095 && GM6030[Motor_Id].CurPosition != 0)
  {
    GM6030[Motor_Id].posdel = GM6030[Motor_Id].posdel - 8192;
  }
  if (GM6030[Motor_Id].posdel <= -4095 && GM6030[Motor_Id].CurPosition != 0)
  {
    GM6030[Motor_Id].posdel = GM6030[Motor_Id].posdel + 8192;
  }

  GM6030[Motor_Id].Temparg += GM6030[Motor_Id].posdel;
  GM6030[Motor_Id].CurPosition = MotorVal[Motor_Id][0];
  GM6030[Motor_Id].CurSpeed = MotorVal[Motor_Id][1];
  GM6030[Motor_Id].ActCurrent = MotorVal[Motor_Id][2];
}

/**
 * @brief 电机控制函数.
 *
 * @param[in] none
 *            
 * @return none
 *
 * @note 该函数用于实现GM6030的速度闭环控制。
 *
 * @example
 *         GM6030_Motor_Speed_Ctrl();
 */
void GM6030_Motor_Speed_Ctrl(void)
{
    GM6030[1].OutCurrent += PIDCal(&GM6030SpeedPID1, GM6030[1].ExpSpeed - GM6030[1].CurSpeed);
    GM6030[2].OutCurrent += PIDCal(&GM6030SpeedPID2, GM6030[2].ExpSpeed - GM6030[2].CurSpeed);
    GM6030[3].OutCurrent += PIDCal(&GM6030SpeedPID3, GM6030[3].ExpSpeed - GM6030[3].CurSpeed);
    GM6030[4].OutCurrent += PIDCal(&GM6030SpeedPID4, GM6030[4].ExpSpeed - GM6030[4].CurSpeed);
    GM6030[5].OutCurrent += PIDCal(&GM6030SpeedPID5, GM6030[5].ExpSpeed - GM6030[5].CurSpeed);
    GM6030[6].OutCurrent += PIDCal(&GM6030SpeedPID6, GM6030[6].ExpSpeed - GM6030[6].CurSpeed);
    GM6030[7].OutCurrent += PIDCal(&GM6030SpeedPID7, GM6030[7].ExpSpeed - GM6030[7].CurSpeed);
    GM6030[8].OutCurrent += PIDCal(&GM6030SpeedPID8, GM6030[8].ExpSpeed - GM6030[8].CurSpeed);
}
/**
 * @brief 电机控制函数.
 *
 * @param[in] GM6030_id 电机ID
 * @param[in] GM6030PositionPIDx 电机位置PID
 *            
 * @return GM6030ExpSpeed 电机期望速度
 *
 * @note 该函数用于实现单个GM6030的位置闭环控制。
 *
 * @example
 *         GM6030_Motor_Signal_Position_Ctrl();
 */
float GM6030_Motor_Signal_Position_Ctrl(int GM6030_id, PIDType *GM6030PositionPIDx)
{
  float GM6030ExpSpeed;
  if (abs(GM6030[GM6030_id].Exparg - GM6030[GM6030_id].Temparg) < 11000)
  {
    if(PIDCal(GM6030PositionPIDx, GM6030[GM6030_id].Exparg - GM6030[GM6030_id].Temparg)<-5)
   { GM6030ExpSpeed = -5;
  }
  else if(PIDCal(GM6030PositionPIDx, GM6030[GM6030_id].Exparg - GM6030[GM6030_id].Temparg)>5)
   { GM6030ExpSpeed = 5;
  }
  else
  { GM6030ExpSpeed = PIDCal(GM6030PositionPIDx, GM6030[GM6030_id].Exparg - GM6030[GM6030_id].Temparg);
  }
    
  }
  else if (GM6030[GM6030_id].Exparg > GM6030[GM6030_id].Temparg)
  {
    GM6030ExpSpeed = 1.05 * GM6030[GM6030_id].Expvel;
  }
  else
  {
    GM6030ExpSpeed = -1.05 * GM6030[GM6030_id].Expvel;
  }
  return GM6030ExpSpeed;
}
/**
 * @brief 电机控制函数.
 *
 * @param[in] none
 *            
 * @return none
 *
 * @note 该函数用于实现GM6030的位置闭环控制。
 *
 * @example
 *         GM6030_Motor_Position_Ctrl();
 */
void GM6030_Motor_Position_Ctrl(void)
{
    // GM6030[1].ExpSpeed = (int16_t)GM6030_Motor_Signal_Position_Ctrl(1, &GM6030PositionPID1);
    // GM6030[2].ExpSpeed = (int16_t)GM6030_Motor_Signal_Position_Ctrl(2, &GM6030PositionPID2);
   // GM6030[3].ExpSpeed = (int16_t)GM6030_Motor_Signal_Position_Ctrl(3, &GM6030PositionPID3);
    //GM6030[4].ExpSpeed = (int16_t)GM6030_Motor_Signal_Position_Ctrl(4, &GM6030PositionPID4);
    GM6030[5].ExpSpeed = (int16_t)GM6030_Motor_Signal_Position_Ctrl(5, &GM6030PositionPID5);
     //GM6030[6].ExpSpeed = (int16_t)GM6030_Motor_Signal_Position_Ctrl(6, &GM6030PositionPID6);
    // GM6030[7].ExpSpeed = (int16_t)GM6030_Motor_Signal_Position_Ctrl(7, &GM6030PositionPID7);
   // GM6030[8].ExpSpeed = (int16_t)GM6030_Motor_Signal_Position_Ctrl(8, &GM6030PositionPID8);
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
 *        GM6030_Motor_Current_Ctrl(0x200, 0, 0, 0, 0, 0, 0, 0, 0);
 */
//void GM6030_Motor_Current_Ctrl(void)
//{
//    uint8_t i;
//    uint32_t send_mail_box;
//    CAN_TxHeaderTypeDef TxHeader;
//    TxHeader.StdId = 0x1FF;
//    TxHeader.IDE = CAN_ID_STD;
//    TxHeader.RTR = CAN_RTR_DATA;
//    TxHeader.DLC = 0x08;
//    TxHeader.TransmitGlobalTime = DISABLE;
//    uint8_t Txmsg[8] = {0};
//
//    for (i = 1; i <= 8; i++)
//    {
//        if (GM6030[i].OutCurrent > 16384)
//        {
//            GM6030[i].OutCurrent = 16384;
//        }
//        else if (GM6030[i].OutCurrent < -16384)
//        {
//            GM6030[i].OutCurrent = -16384;
//        }
//    }
//    Txmsg[0] = (uint8_t)((GM6030[1].OutCurrent >> 8) & 0xFF);
//    Txmsg[1] = (uint8_t)( GM6030[1].OutCurrent & 0xFF);
//    Txmsg[2] = (uint8_t)((GM6030[2].OutCurrent >> 8) & 0xFF);
//    Txmsg[3] = (uint8_t)( GM6030[2].OutCurrent & 0xFF);
//    Txmsg[4] = (uint8_t)((GM6030[3].OutCurrent >> 8) & 0xFF);
//    Txmsg[5] = (uint8_t)( GM6030[3].OutCurrent & 0xFF);
//    Txmsg[6] = (uint8_t)((GM6030[4].OutCurrent >> 8) & 0xFF);
//    Txmsg[7] = (uint8_t)( GM6030[4].OutCurrent & 0xFF);
//    while(HAL_CAN_AddTxMessage(&hcan1, &TxHeader, Txmsg, &send_mail_box)!=HAL_OK)
//    {
//        //Error_Handler();
//    }
//    HAL_Delay(1);
//
//}
/**
 * @brief 电机控制函数.
 *
 * @param[in] none
 *            
 * @return none
 *
 * @note 该函数用于实现GM6030的电流闭环控制。
 *
 * @example
 *         GM6030_Motor_Current_Ctrl_h();
 */
void GM6030_Motor_Current_Ctrl_h(void)
{
    uint8_t i;
    uint32_t send_mail_box;
    CAN_TxHeaderTypeDef TxHeader;
    TxHeader.StdId = 0x2FF;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.DLC = 0x08;
    TxHeader.TransmitGlobalTime = DISABLE;
    uint8_t Txmsg[8] = {0};

    for (i = 1; i <= 8; i++)
    {
        if (GM6030[i].OutCurrent > 16384)
        {
            GM6030[i].OutCurrent = 16384;
        }
        else if (GM6030[i].OutCurrent < -16384)
        {
            GM6030[i].OutCurrent = -16384;
        }
    }
    Txmsg[0] = (uint8_t)((GM6030[5].OutCurrent >> 8) & 0xFF);
    Txmsg[1] = (uint8_t)( GM6030[5].OutCurrent & 0xFF);
    Txmsg[2] = (uint8_t)((GM6030[6].OutCurrent >> 8) & 0xFF);
    Txmsg[3] = (uint8_t)( GM6030[6].OutCurrent & 0xFF);
    Txmsg[4] = (uint8_t)((GM6030[7].OutCurrent >> 8) & 0xFF);
    Txmsg[5] = (uint8_t)( GM6030[7].OutCurrent & 0xFF);
    Txmsg[6] = (uint8_t)((GM6030[8].OutCurrent >> 8) & 0xFF);
    Txmsg[7] = (uint8_t)( GM6030[8].OutCurrent & 0xFF);
    while(HAL_CAN_AddTxMessage(&hcan2, &TxHeader, Txmsg, &send_mail_box)!=HAL_OK)
    {
//        Error_Handler();
      error_count++;
    }

//    if(HAL_CAN_AddTxMessage(&hcan1, &TxHeader, Txmsg, &send_mail_box)!=HAL_OK)
//    {
//        Error_Handler();
//    }
 //   jishu2++;
}