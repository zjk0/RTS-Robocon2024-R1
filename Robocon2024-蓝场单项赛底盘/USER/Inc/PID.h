/**
 * @file PID.h
 * @author 陶子辰 (2405973406@qq.com)
 * @brief 
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

#pragma once
#include "main.h"

/*
*********************************************************************************************************
*                                             EXPORTED_TYPES
*********************************************************************************************************
*/

/**
 * @brief PID参数结构体
 * 
 */
typedef struct
{
  float KP;
  float KI;
  float KD;
  float PreErr;
  float LastErr;
  float delErr;       // error tolerance
  float KIlimit;      // intergral limit
  float SumErr;
  float Sumlimit;
}PIDType;



/*
*********************************************************************************************************
*                                           FUNCTION PROTOTYPES
*********************************************************************************************************
*/

float PIDCal( PIDType *PIDptr, float ThisError );
void pid_reset( PIDType *PIDptr);
float PIDCal_pos ( PIDType *PIDptr, float ThisError );