/*
*********************************************************************************************************
*                                             INCLUDE FILES
*********************************************************************************************************
*/

#pragma once
#include "main.h"

/*
*********************************************************************************************************
*                                             EXPORTED_TYPES
*********************************************************************************************************
*/

/**
 * @brief ODRIVE扭矩设置结构体 
 * @param Input_Torque          期望扭矩，单位:Nm
 * 
 */
typedef struct
{
    float Input_Torque;
}Set_Input_Torque;

/**
 * @brief ODRIVE速度设置结构体 
 * @param Input_Vel             期望转速，单位:圈/s
 * @param Input_Torque_FF       前馈扭矩，单位:Nm
 * 
 */
typedef struct
{
    float Input_Vel;
    float Input_Torque_FF;
}Set_Input_Vel;

/**
 * @brief ODRIVE位置设置结构体 
 * @param Input_Pos             期望位置，单位:圈
 * @param Input_Vel_FF          前馈转速，单位:圈/s
 * @param Input_Torque_FF       前馈扭矩，单位:Nm
 * 
 */
typedef struct
{
    float Input_Pos;
    int Input_Vel_FF;
    int Input_Torque_FF;
}Set_Input_Pos;

/**
 * @brief ODRIVE限幅设置结构体 
 * @param Velocity_Limit        转速限幅，单位:圈/s
 * @param Current_Limit         电流限幅，单位:A
 * 
 */
typedef struct
{
    float Velocity_Limit;
    float Current_Limit;
}Set_Limits;

/**
 * @brief ODRIVE扭矩设置结构体 
 * @param set_input_pos         位置设置
 * @param set_input_vel         转速设置
 * @param set_input_torque      扭矩设置
 * @param set_limits            限幅设置
 * 
 */
typedef struct
{
    Set_Input_Torque set_input_torque;
    Set_Input_Vel    set_input_vel;
    Set_Input_Pos    set_input_pos;
    Set_Limits       set_limits;
}ODRIVE_MOTOR;


/*
*********************************************************************************************************
*                                               DEFINES
*********************************************************************************************************
*/

#define SET_INPUT_TORQUE 0X00e
#define SET_INPUT_VEL    0x00d
#define SET_INPUT_POS    0x00c
#define SET_LIMITS       0x00f


/*
*********************************************************************************************************
*                                              函数声明
*********************************************************************************************************
*/

void ODRIVE_Motor_Speed_Ctrl(uint8_t Motor_ID, float ExpSpeed, float ForwardTorque);


/*
*********************************************************************************************************
*                                              变量声明
*********************************************************************************************************
*/


extern ODRIVE_MOTOR ODRIVE[5];


