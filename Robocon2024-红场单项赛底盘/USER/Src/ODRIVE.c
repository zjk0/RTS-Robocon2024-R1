/*
*********************************************************************************************************
*                                             INCLUDE FILES
*********************************************************************************************************
*/

#include "ODRIVE.h"

/*
*********************************************************************************************************
*                                            	变量定义
*********************************************************************************************************
*/

ODRIVE_MOTOR ODRIVE[5];

/**
 * @brief 向odrive电调发送指令控制速度.
 * 
 * @param[in] Motor_ID 在odrive上设置的电调ID
 * @param[in] ExpSpeed 电机期望转速 单位（转/s）
 * 
 * @return none
 * 
 * @note 该函数用于向odrive电调发送指令控制速度（速度直接模式）。
 *       can_id=电机电调ID左移5位+0x00d（速度输入）
 *       控制速度限幅 -6000rpm~6000rpm
 *       Speed_float[0]:控制输出速度
 *       Speed_float[1]:控制前馈扭矩
 *       data[0]:控制输出(float转uint8_t)速度第4位
 *       data[1]:控制输出(float转uint8_t)速度第3位
 *       data[2]:控制输出(float转uint8_t)速度第2位
 *       data[3]:控制输出(float转uint8_t)速度第1位
 *       data[4]:控制前馈(float转uint8_t)扭矩第4位
 *       data[5]:控制前馈(float转uint8_t)扭矩第3位
 *       data[6]:控制前馈(float转uint8_t)扭矩第2位
 *       data[7]:控制前馈(float转uint8_t)扭矩第1位
 * 
 * @example
 *        ODRIVE_Motor_Speed_Ctrl(2, ODRIVE[2].set_input_vel.Input_Vel, ODRIVE[2].set_input_vel.Input_Torque_FF);
 */
void ODRIVE_Motor_Speed_Ctrl(uint8_t Motor_ID, float ExpSpeed, float ForwardTorque)
{
    static union
    {
        uint8_t Speed[8];
        float Speed_float[2];
    }Trans_Speed;
    uint32_t send_mail_box;
    CAN_TxHeaderTypeDef TxHeader;
    TxHeader.StdId = SET_INPUT_VEL + (Motor_ID<<5);
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.DLC = 0x08;
    TxHeader.TransmitGlobalTime = DISABLE;
    uint8_t Txmsg[8] = {0};
    if(ExpSpeed > 100)
      ExpSpeed = 100.0;
    else if(ExpSpeed < -100)
      ExpSpeed = -100.0;
    Trans_Speed.Speed_float[0] = ExpSpeed;
    Trans_Speed.Speed_float[1] = ForwardTorque;
    Txmsg[0] = Trans_Speed.Speed[0];
    Txmsg[1] = Trans_Speed.Speed[1];
    Txmsg[2] = Trans_Speed.Speed[2];
    Txmsg[3] = Trans_Speed.Speed[3];
    Txmsg[4] = Trans_Speed.Speed[7];
    Txmsg[5] = Trans_Speed.Speed[6];
    Txmsg[6] = Trans_Speed.Speed[5];
    Txmsg[7] = Trans_Speed.Speed[4];
    if(HAL_CAN_AddTxMessage(&hcan1, &TxHeader, Txmsg, &send_mail_box)!=HAL_OK)
    {
        Error_Handler();
    }
}