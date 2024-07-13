/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
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

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

float pos_ball_x[10][10] = {0};
float pos_ball_y[10][10] = {0};
extern PIDType PID_Vx_move_distance;
extern PIDType PID_Vy_move_distance;
extern PIDType PID_w_move_distance;
extern PIDType M3508SpeedPID1;
extern PIDType M3508SpeedPID2;
extern PIDType M3508SpeedPID3;
extern PIDType M3508SpeedPID4; //{6, 0.1, 1, 0, 0, 0, 0};
extern PIDType M3508SpeedPID5;
extern PIDType M3508SpeedPID6; // {13, 0.72, 0, 0, 0, 0, 0};
// PIDType M3508SpeedPID4 = {8.2, 0.92, 0, 0, 0, 0, 0, 0, 0}; //{6, 0.1, 1, 0, 0, 0, 0};
// PIDType M3508SpeedPID5 = {8.2, 0.92, 0, 0, 0, 0, 0, 0, 0};
// PIDType M3508SpeedPID6 = {8.2, 0.92, 0, 0, 0, 0, 0, 0, 0}; // {13, 0.72, 0, 0, 0, 0, 0};
// PIDType M3508SpeedPID4 = {20, 0.1, 0, 0, 0, 0, 0, 0, 0}; //{6, 0.1, 1, 0, 0, 0, 0};
// PIDType M3508SpeedPID5 = {20, 0.1, 0, 0, 0, 0, 0, 0, 0};
// PIDType M3508SpeedPID6 = {20, 0.1, 0, 0, 0, 0, 0, 0, 0}; // {13, 0.72, 0, 0, 0, 0, 0};
extern PIDType M3508SpeedPID7;
extern PIDType M3508SpeedPID8;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
int Motor5_Flag = 0;
int Motor6_Flag = 0;
int Motor7_Flag = 0;
float x_dist = 0, y_dist = 0;
int move_flag = 0;
int move_flag_laser = 0;
uint8_t shou_bin_enable = 0;

uint8_t now_pos_x = 0;
uint8_t now_pos_y = 0;
int next_pos_x = 0;
int next_pos_y = 0;
float next_distance_x = 0;
float next_distance_y = 0;
float act_err = 0;
uint8_t pid_done_ball = 0;
// uint8_t flag_test_action = 0;

char staus_char = 0;
// ********************* PID correct struct *********************
PIDType PID_Vx = {2, 0.5, 0, 0, 0, 0, 0, 0, 0};
PIDType PID_Vy = {2, 0.5, 0, 0, 0, 0, 0, 0, 0};
PIDType PID_Vw = {15, 1, 0, 0, 0, 0, 0, 0, 0};
// ********************* PID correct struct *********************

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
    .name = "defaultTask",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};
/* Definitions for SBUS */
osThreadId_t SBUSHandle;
const osThreadAttr_t SBUS_attributes = {
    .name = "SBUS",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityBelowNormal6,
};
/* Definitions for ACTION */
osThreadId_t ACTIONHandle;
const osThreadAttr_t ACTION_attributes = {
    .name = "ACTION",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityBelowNormal7,
};
/* Definitions for Position */
osThreadId_t PositionHandle;
const osThreadAttr_t Position_attributes = {
    .name = "Position",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityBelowNormal5,
};
/* Definitions for Speed */
osThreadId_t SpeedHandle;
const osThreadAttr_t Speed_attributes = {
    .name = "Speed",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityLow,
};
/* Definitions for IOcontrol */
osThreadId_t IOcontrolHandle;
const osThreadAttr_t IOcontrol_attributes = {
    .name = "IOcontrol",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityLow,
};
/* Definitions for M3508_control */
osThreadId_t M3508_controlHandle;
const osThreadAttr_t M3508_control_attributes = {
    .name = "M3508_control",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityLow,
};
/* Definitions for SerialPlot_Send */
osThreadId_t SerialPlot_SendHandle;
const osThreadAttr_t SerialPlot_Send_attributes = {
    .name = "SerialPlot_Send",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityLow,
};
/* Definitions for Restart */
osThreadId_t RestartHandle;
const osThreadAttr_t Restart_attributes = {
    .name = "Restart",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityLow,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void SBUSTask(void *argument);
void ACTIONTask(void *argument);
void PositionTask(void *argument);
void SpeedTask(void *argument);
void IOTask(void *argument);
void M3508_Task(void *argument);
void SerialPlot_Task(void *argument);
void Restart_Task(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
 * @brief  FreeRTOS initialization
 * @param  None
 * @retval None
 */
void MX_FREERTOS_Init(void)
{
    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* USER CODE BEGIN RTOS_MUTEX */
    /* add mutexes, ... */
    /* USER CODE END RTOS_MUTEX */

    /* USER CODE BEGIN RTOS_SEMAPHORES */
    /* add semaphores, ... */
    /* USER CODE END RTOS_SEMAPHORES */

    /* USER CODE BEGIN RTOS_TIMERS */
    /* start timers, add new ones, ... */
    /* USER CODE END RTOS_TIMERS */

    /* USER CODE BEGIN RTOS_QUEUES */
    /* add queues, ... */
    /* USER CODE END RTOS_QUEUES */

    /* Create the thread(s) */
    /* creation of defaultTask */
    defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

    /* creation of SBUS */
    SBUSHandle = osThreadNew(SBUSTask, NULL, &SBUS_attributes);

    /* creation of ACTION */
    ACTIONHandle = osThreadNew(ACTIONTask, NULL, &ACTION_attributes);

    /* creation of Position */
    PositionHandle = osThreadNew(PositionTask, NULL, &Position_attributes);

    /* creation of Speed */
    SpeedHandle = osThreadNew(SpeedTask, NULL, &Speed_attributes);

    /* creation of IOcontrol */
    IOcontrolHandle = osThreadNew(IOTask, NULL, &IOcontrol_attributes);

    /* creation of M3508_control */
    M3508_controlHandle = osThreadNew(M3508_Task, NULL, &M3508_control_attributes);

    /* creation of SerialPlot_Send */
    SerialPlot_SendHandle = osThreadNew(SerialPlot_Task, NULL, &SerialPlot_Send_attributes);

    /* creation of Restart */
    RestartHandle = osThreadNew(Restart_Task, NULL, &Restart_attributes);

    /* USER CODE BEGIN RTOS_THREADS */
    /* add threads, ... */
    /* USER CODE END RTOS_THREADS */

    /* USER CODE BEGIN RTOS_EVENTS */
    /* add events, ... */
    /* USER CODE END RTOS_EVENTS */
}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
    /* USER CODE BEGIN StartDefaultTask */

    pos_ball_x[6][2] = 3.2016711233 - 0.01;
    pos_ball_y[6][2] = 0.48383681 + 0.04;

    pos_ball_x[6][1] = 3.2016711233 - 0.01;
    pos_ball_y[6][1] = 1.003 + 0.02;

    pos_ball_x[5][1] = 2.71114111 - 0.05;
    pos_ball_y[5][1] = 1.003 + 0.02;

    pos_ball_x[5][2] = 2.71114111 - 0.03;
    pos_ball_y[5][2] = 0.48383681 + 0.04;

    pos_ball_x[4][1] = 2.2161109444 - 0.035;
    pos_ball_y[4][1] = 1.003 + 0.02;

    pos_ball_x[4][2] = 2.2161109444 - 0.02;
    pos_ball_y[4][2] = 0.48383681 + 0.03;

    //    pos_ball_x[3][1] = 1.72358107;
    //    pos_ball_y[3][1] = 1.003;
    //
    //    pos_ball_x[3][2] = 1.72358107;
    //    pos_ball_y[3][2] = 0.48383681;
    //
    //    pos_ball_x[2][1] = 1.23705136;
    //    pos_ball_y[2][1] = 1.003;
    //
    //    pos_ball_x[2][2] = 1.23705136;
    //    pos_ball_y[2][2] = 0.48383681;
    //
    //    pos_ball_x[1][1] = 0.742521286;
    //    pos_ball_y[1][1] = 1.003;
    //
    //    pos_ball_x[1][2] = 0.742521286;
    //    pos_ball_y[1][2] = 0.48383681;

    pos_ball_x[3][1] = 1.72358107 - 0.06;
    pos_ball_y[3][1] = 0.869976759 + 0.55 + 0.285;

    pos_ball_x[3][2] = 1.72358107 - 0.06;
    pos_ball_y[3][2] = 0.869976759 + 0.20;

    pos_ball_x[2][1] = 1.23705136 - 0.07;
    pos_ball_y[2][1] = 0.899976759 + 0.5 + 0.20 + 0.1;

    pos_ball_x[2][2] = 1.23705136 - 0.02 - 0.016;
    pos_ball_y[2][2] = 0.899976759 + 0.08 + 0.1 + 0.1;

    pos_ball_x[1][1] = 0.742521286 - 0.045;
    pos_ball_y[1][1] = 1.4681708811 + 0.5 + 0.07;

    pos_ball_x[1][2] = 0.742521286 - 0.045;
    pos_ball_y[1][2] = 1.4681708811 + 0.05;
    // 1.4681708811
    // 9.38148319E-1
    // 8.69976759E-1
    /* Infinite loop */
    for (;;)
    {
        osDelay(1);
    }
    /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_SBUSTask */
/**
 * @brief Function implementing the SBUS thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_SBUSTask */
void SBUSTask(void *argument)
{
    /* USER CODE BEGIN SBUSTask */
    /* Infinite loop */
    for (;;)
    {
        /* if (SBUS.ConnectState > 0)
           SBUS.ConnectState = SBUS.ConnectState - 1;
         switch (SBUS_Analyse_Switch(SBUS.CH8))
         {
         case -1:
           break;
         case 0:
           Robot.Robot_Contorl_Mode = CONTROL_MANUAL;
           Robot.Robot_in_self.velocity_exp.Vx = ROBOT_SPEED_MAX * SBUS_Analyse_Percent(SBUS.CH11) * (SBUS_Analyse_Percent(SBUS.CH1) - 0.5) * 2;
           Robot.Robot_in_self.velocity_exp.Vy = ROBOT_SPEED_MAX * SBUS_Analyse_Percent(SBUS.CH11) * (SBUS_Analyse_Percent(SBUS.CH2) - 0.5) * 2;
           Robot.Robot_in_self.velocity_exp.Vw = ROBOT_SPEED_YAW_MAX * SBUS_Analyse_Percent(SBUS.CH11) * (SBUS_Analyse_Percent(SBUS.CH4) - 0.5) * 20;
           break;
         case 1:
           Robot.Robot_Contorl_Mode = CONTROL_WORLD;
           Robot.Robot_in_world.velocity_exp.Vx = ROBOT_SPEED_MAX * SBUS_Analyse_Percent(SBUS.CH11) * (SBUS_Analyse_Percent(SBUS.CH1) - 0.5) * 2;
           Robot.Robot_in_world.velocity_exp.Vy = ROBOT_SPEED_MAX * SBUS_Analyse_Percent(SBUS.CH11) * (SBUS_Analyse_Percent(SBUS.CH2) - 0.5) * 2;
           Robot.Robot_in_world.velocity_exp.Vw = ROBOT_SPEED_YAW_MAX * SBUS_Analyse_Percent(SBUS.CH11) * (SBUS_Analyse_Percent(SBUS.CH4) - 0.5) * 20;
           break;
         case 2:
           Robot.Robot_Contorl_Mode = CONTROL_AUTO;
           break;
         }*/
        if (laser_status == 5 || laser_status == 10 || laser_status == 15 || (laser_status == 18 && SBUS_Analyse_Switch(SBUS.CH13) == 0))
        {
            if (jishu_0_x >= 1 && jishu_0_y >= 1 /* && jishu_0_w >= 1*/)
            {
                Robot.Robot_in_world.velocity_exp.Vy = 0;
                Robot.Robot_in_world.velocity_exp.Vx = 0;
                Robot.Robot_in_world.velocity_exp.Vw = 0;
                if (laser_status <= 18)
                {
                    move_flag_laser = 0;

                    //                if (laser_status == 15)
                    //                {
                    //                    HAL_UART_Transmit(&huart6, rotate_servo_command, 1, 1000);
                    //                    while (lllflag != 1)
                    //                    {
                    //                        osDelay(1);
                    //                    }
                    //                    lllflag = 0;
                    //                }
                    if (Release_Seed_Flag == 1)
                    {
                        HAL_UART_Transmit(&huart6, Paw_Command, 1, 1000);
                        Release_Seed_Flag = 0;
                    }

                    if (laser_status == 2 || laser_status == 3 || laser_status == 7 || laser_status == 8 || laser_status == 12 || laser_status == 13)
                    {
                        osDelay(1200);
                    }

                    // ���ߴ���
                    //  staus_char = laser_status + 48;
                    //  HAL_UART_Transmit(&huart1, &staus_char, 1, 1000);
                    //  HAL_UART_Transmit(&huart1, &pos[1], 1, 1000);
                    //  HAL_UART_Transmit(&huart1, &pos[9], 1, 1000);

                    move_flag_laser = 1;
                    laser_status++;
                }
                else
                {
                    pid_done_ball = 1;
                }
                jishu_0_x = 0;
                jishu_0_y = 0;
                jishu_0_w = 0;
                // x_pre = Robot.Robot_in_self.position_now.x;
                // y_pre = Robot.Robot_in_self.position_now.y;
                yaw_pre = Robot.Robot_in_self.position_now.yaw;
                if (laser_status == 3 && laser_status == 8 && laser_status == 13)
                {
                    pid_reset(&PID_Vx_move_distance);
                    pid_reset(&PID_Vy_move_distance);
                    pid_reset(&PID_w_move_distance);
                }
            }
        }
        else
        {
            if (jishu_0_x >= 25 && jishu_0_y >= 25 && jishu_0_w >= 25)
            {
                Robot.Robot_in_world.velocity_exp.Vy = 0;
                Robot.Robot_in_world.velocity_exp.Vx = 0;
                Robot.Robot_in_world.velocity_exp.Vw = 0;
                if (laser_status <= 18)
                {
                    move_flag_laser = 0;

                    //                if (laser_status == 15)
                    //                {
                    //                    HAL_UART_Transmit(&huart6, rotate_servo_command, 1, 1000);
                    //                    while (lllflag != 1)
                    //                    {
                    //                        osDelay(1);
                    //                    }
                    //                    lllflag = 0;
                    //                }
                    if (Release_Seed_Flag == 1)
                    {
                        HAL_UART_Transmit(&huart6, Paw_Command, 1, 1000);
                        Release_Seed_Flag = 0;
                    }

                    if (laser_status == 2 || laser_status == 3 || laser_status == 7 || laser_status == 8 || laser_status == 12 || laser_status == 13)
                    {
                        osDelay(1200);
                    }

                    // ���ߴ���
                    //  staus_char = laser_status + 48;
                    //  HAL_UART_Transmit(&huart1, &staus_char, 1, 1000);
                    //  HAL_UART_Transmit(&huart1, &pos[1], 1, 1000);
                    //  HAL_UART_Transmit(&huart1, &pos[9], 1, 1000);

                    move_flag_laser = 1;
                    laser_status++;
                }
                else
                {
                    pid_done_ball = 1;
                }
                jishu_0_x = 0;
                jishu_0_y = 0;
                jishu_0_w = 0;
                // x_pre = Robot.Robot_in_self.position_now.x;
                // y_pre = Robot.Robot_in_self.position_now.y;
                yaw_pre = Robot.Robot_in_self.position_now.yaw;
                if (laser_status == 3 && laser_status == 8 && laser_status == 13)
                {
                    pid_reset(&PID_Vx_move_distance);
                    pid_reset(&PID_Vy_move_distance);
                    pid_reset(&PID_w_move_distance);
                }
            }
        }
        // }

        osDelay(10);
    }
    /* USER CODE END SBUSTask */
}

/* USER CODE BEGIN Header_ACTIONTask */
/**
 * @brief Function implementing the ACTION thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_ACTIONTask */
void ACTIONTask(void *argument)
{
    /* USER CODE BEGIN ACTIONTask */
    //    while((pow(Robot.Robot_in_self.position_now.y,2) + pow(Robot.Robot_in_self.position_now.x,2)) < 0.04){
    //        osDelay(2);
    //    }
    //   uint8_t i = 1;
    uint8_t StartStatusChange = 1;
    /* Infinite loop */
    for (;;)
    {
        // Bee();
        //		if(Bee_flag == 0){
        //		        if(Robot.Robot_in_self.position_now.x != 0 || Robot.Robot_in_self.position_now.y != 0){
        //    		    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
        //			    osDelay(100);
        //			    HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_3);
        //			    Bee_flag = 1;
        //  		    }
        //	    }
        if (SBUS_Analyse_Switch(SBUS.CH12) == 2)
        {
            move_flag_laser = 1;
            if (StartStatusChange)
            {
                if (SBUS_Analyse_Switch(SBUS.CH5) == 0)
                {
                    laser_status = 17;
                }
                else
                    laser_status = 1;
                Action_Signal_Send();
                StartStatusChange--;
            }
        }
        if (SBUS_Analyse_Switch(SBUS.CH12) != 2)
        {
            move_flag_laser = 0;
        }
        //        if (SBUS_Analyse_Switch(SBUS.CH13) == 0) {
        //            status++;
        //            move_flag = 1;
        //            while (SBUS_Analyse_Switch(SBUS.CH13) == 0)
        //            {
        //                osDelay(2);
        //            }
        //        }

        //  if (SBUS_Analyse_Switch(SBUS.CH12) == 0) {
        //     laser_status++;
        //     move_flag_laser = 1;
        // 	//HAL_UART_Transmit(&huart6, Paw_Command, 1, 1000);
        //     while (SBUS_Analyse_Switch(SBUS.CH12) == 0)
        //     {
        //         osDelay(2);
        //     }
        // }

        osDelay(5);
    }
    /* USER CODE END ACTIONTask */
}

/* USER CODE BEGIN Header_PositionTask */
/**
 * @brief Function implementing the Position thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_PositionTask */
void PositionTask(void *argument)
{
    /* USER CODE BEGIN PositionTask */
    /* Infinite loop */
    for (;;)
    {
        if (move_flag_laser)
        {
            if (laser_status == 1) // From start point to the place where robot firstly catch seeds
            {
                // Robot run forward
                Robot.Robot_in_world.velocity_exp.Vy = 1.5;
                Robot.Robot_in_world.velocity_exp.Vx = 0;
                osDelay(400);

                // Robot run left
                Robot.Robot_in_world.velocity_exp.Vy = 0;
                Robot.Robot_in_world.velocity_exp.Vx = -1.3;
                osDelay(1310);

                // Robot run back
                Robot.Robot_in_world.velocity_exp.Vy = -0.7;
                Robot.Robot_in_world.velocity_exp.Vx = 0;
                osDelay(486);
                Robot.Robot_in_world.velocity_exp.Vy = -0.5;
                Robot.Robot_in_world.velocity_exp.Vx = 0;
                osDelay(391);

                // Robot run to catch seed
                Robot.Robot_in_world.velocity_exp.Vy = -0.2;
                Robot.Robot_in_world.velocity_exp.Vx = -0.6;

                while (laser_status == 1)
                {
                    osDelay(1);
                }
                if (laser_status == 2)
                {
                    Robot.Robot_in_world.velocity_exp.Vx = 0;
                    Robot.Robot_in_world.velocity_exp.Vy = -0.2;
                    osDelay(900);
                    Robot.Robot_in_world.velocity_exp.Vy = 0;
                }
            }
            else if (laser_status == 2) // ��һ������
            {
                Move_Distance_Laser(3.0770452022 + 0.025 + 0.09768, 1.3863713744 + 0.01, 0);
                Release_Seed_Flag = 1;
            }
            else if (laser_status == 3) // �ŵڶ�����
            {
                Move_Distance_Laser(2.79765772 + 0.09768, 0.859318077 + 0.01, 0);
                Release_Seed_Flag = 1;
            }
            else if (laser_status == 4) // ���Ҳ�Ҫ��̫��
            {
                Robot.Robot_in_world.velocity_exp.Vy = 0.6;
                Robot.Robot_in_world.velocity_exp.Vx = -1;
                osDelay(700);
                Robot.Robot_in_world.velocity_exp.Vy = -0.6;
                Robot.Robot_in_world.velocity_exp.Vx = -1.4;
                osDelay(500);
                Robot.Robot_in_world.velocity_exp.Vy = -1;
                Robot.Robot_in_world.velocity_exp.Vx = 0;
                osDelay(900);
                laser_status++;
                Enable_flag = 1;
                Robot.Robot_in_world.velocity_exp.Vy = 0;
                Robot.Robot_in_world.velocity_exp.Vx = 0;
                Release_Seed_Flag = 0;
            }
            else if (laser_status == 5) // ��ȡ��ǰ��һ��
            {
                Move_Distance_Laser(2.03742671 + 0.09768, 3.4714949133, 0);
                if (Enable_flag == 1) // ��ֻ֤��һ��
                {
                    HAL_UART_Transmit(&huart6, Enable_Command, 1, 1000);
                    Enable_flag = 0;
                }
                Release_Seed_Flag = 0;
            }
            else if (laser_status == 6) // ȡ�ڶ�����
            {
                Release_Seed_Flag = 0;
                Robot.Robot_in_world.velocity_exp.Vy = -0.5;
                Robot.Robot_in_world.velocity_exp.Vx = 0;
                osDelay(800);
                // Robot run to catch seed
                Robot.Robot_in_world.velocity_exp.Vy = -0.2;
                Robot.Robot_in_world.velocity_exp.Vx = -0.6;
                while (laser_status == 6)
                {
                    osDelay(1);
                }
                if (laser_status == 7)
                {
                    Robot.Robot_in_world.velocity_exp.Vy = -0.2;
                    Robot.Robot_in_world.velocity_exp.Vx = 0;
                    osDelay(900);
                    Robot.Robot_in_world.velocity_exp.Vy = 0;
                }
            }
            else if (laser_status == 7) // ����������
            {
                Move_Distance_Laser(2.077 + 0.09768, 1.3864 + 0.01, 0);
                Release_Seed_Flag = 1;
            }
            else if (laser_status == 8)
            {
                Move_Distance_Laser(1.7976 + 0.09768, 0.8593 + 0.01, 0);
                Release_Seed_Flag = 1;
            }
            else if (laser_status == 9) // ���Ҳ�Ҫ��̫��
            {
                Robot.Robot_in_world.velocity_exp.Vy = 0.6;
                Robot.Robot_in_world.velocity_exp.Vx = -1;
                osDelay(700);
                Robot.Robot_in_world.velocity_exp.Vy = -0.6;
                Robot.Robot_in_world.velocity_exp.Vx = -1.4;
                osDelay(500);
                Robot.Robot_in_world.velocity_exp.Vy = -1;
                Robot.Robot_in_world.velocity_exp.Vx = 0;
                osDelay(900);
                laser_status++;
                Enable_flag = 1;
                Robot.Robot_in_world.velocity_exp.Vy = 0;
                Robot.Robot_in_world.velocity_exp.Vx = 0;
                Release_Seed_Flag = 0;
            }
            else if (laser_status == 10) // ��ȡ��ǰ��һ��
            {
                Move_Distance_Laser(1.04817211 + 0.09768, 3.5407648, 0);
                if (Enable_flag == 1) // ��ֻ֤��һ��
                {
                    HAL_UART_Transmit(&huart6, Enable_Command, 1, 1000);
                    Enable_flag = 0;
                }
                Release_Seed_Flag = 0;
            }

            else if (laser_status == 11) // ȡ�ڶ�����
            {
                Release_Seed_Flag = 0;
                Robot.Robot_in_world.velocity_exp.Vy = -0.5;
                Robot.Robot_in_world.velocity_exp.Vx = 0;
                osDelay(800);
                // Robot run to catch seed
                Robot.Robot_in_world.velocity_exp.Vy = -0.2;
                Robot.Robot_in_world.velocity_exp.Vx = -0.6;
                while (laser_status == 11)
                {
                    osDelay(1);
                }
                if (laser_status == 12)
                {
                    Robot.Robot_in_world.velocity_exp.Vy = -0.2;
                    Robot.Robot_in_world.velocity_exp.Vx = 0;
                    osDelay(900);
                    Robot.Robot_in_world.velocity_exp.Vy = 0;
                }
            }
            else if (laser_status == 12) // ���������????
            {
                Move_Distance_Laser(1.077 + 0.09768, 1.3864 + 0.015, 0);
                Release_Seed_Flag = 1;
            }
            else if (laser_status == 13)
            {
                Move_Distance_Laser(0.7976 + 0.09768, 0.8593 + 0.01, 0);
                Release_Seed_Flag = 1;
            }
            else if (laser_status == 14) // ���Ҳ�Ҫ��̫��
            {
                Release_Seed_Flag = 0;
                Robot.Robot_in_world.velocity_exp.Vy = 1;
                Robot.Robot_in_world.velocity_exp.Vx = 0;
                osDelay(360);
                laser_status++;
                Robot.Robot_in_world.velocity_exp.Vy = 0;
                Robot.Robot_in_world.velocity_exp.Vx = 0;
            }
            else if (laser_status == 15)
            {
                Move_Distance_Laser(3.89188122 + 0.09768, 0.59679532 - 0.02, 0);
            }
            else if (laser_status == 16)
            {
                HAL_UART_Transmit(&huart6, area_2, 1, 1000);
                Robot.Robot_in_world.velocity_exp.Vx = 0;
                Robot.Robot_in_world.velocity_exp.Vy = -2;
                osDelay(750);
                Robot.Robot_in_world.velocity_exp.Vx = 0;
                Robot.Robot_in_world.velocity_exp.Vy = 0;
                osDelay(80);

                Robot.Robot_in_world.velocity_exp.Vx = 1.5;
                Robot.Robot_in_world.velocity_exp.Vy = 0;
                osDelay(1190);
                Robot.Robot_in_world.velocity_exp.Vx = 0;
                Robot.Robot_in_world.velocity_exp.Vy = 0;
                osDelay(80);

                Robot.Robot_in_world.velocity_exp.Vx = 0;
                Robot.Robot_in_world.velocity_exp.Vy = 1.7;
                osDelay(2120);
                laser_status++;
                Robot.Robot_in_world.velocity_exp.Vy = 0;
                Robot.Robot_in_world.velocity_exp.Vx = 0;
            }
            else if (laser_status == 17)
            {
                while (SBUS_Analyse_Switch(SBUS.CH13) == 1)
                {
                    osDelay(1);
                }

                if (SBUS_Analyse_Switch(SBUS.CH13) == 0)
                {
                    Move_Certain_W_PID(2 * PI / 3);
                }
                else if (SBUS_Analyse_Switch(SBUS.CH13) == 2)
                {
                    Move_Certain_W_PID(PI / 6);
                }
            }
            else if (laser_status == 18)
            {
                // Action_Clear();
                if (SBUS_Analyse_Switch(SBUS.CH13) == 0)
                {
                    PID_Vy_Laser_area2(Robot.Robot_in_self.position_now.x, pos_ball_y[4][2], 2 * PI / 3);
                }
                else if (SBUS_Analyse_Switch(SBUS.CH13) == 2)
                {
                    while (laser_status == 18)
                    {
                        Move_Distance_Laser_3(1.657335, 1.493054 - 0.03, PI / 6);
                        osDelay(1);
                    }
                    laser_status = 18;

                    HAL_UART_Transmit(&huart6, get_ball, 1, 1000);
                    osDelay(2200);

                    Robot.Robot_in_world.velocity_exp.Vx = 1;
                    Robot.Robot_in_world.velocity_exp.Vy = 0;
                    osDelay(1000);
                    Robot.Robot_in_world.velocity_exp.Vx = 0;
                    Robot.Robot_in_world.velocity_exp.Vy = 0;
                    osDelay(50);
                    while (laser_status == 18)
                    {
                        Move_Certain_W_PID(2 * PI / 3);
                        osDelay(1);
                    }
                    laser_status = 18;
                    HAL_UART_Transmit(&huart6, &(ShootBall_Num[6 - 1]), 1, 1000);
                    while (laser_status == 18)
                    {
                        PID_Vy_Laser_area2(Robot.Robot_in_self.position_now.x, pos_ball_y[4][2], 2 * PI / 3);
                        osDelay(1);
                    }
                }
            }
            //            else if (laser_status == 19)
            //            { // ԭ��
            //                Move_Distance_Laser_2(pos_ball_x[4][2], pos_ball_y[4][2] - 0.03, 2 * PI / 3);
            //                now_pos_x = 4;
            //                now_pos_y = 2;
            //            }
        }
        osDelay(1);
    }
    /* USER CODE END PositionTask */
}

/* USER CODE BEGIN Header_SpeedTask */
/**
 * @brief Function implementing the Speed thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_SpeedTask */
void SpeedTask(void *argument)
{
    /* USER CODE BEGIN SpeedTask */
    /* Infinite loop */
    for (;;) // Clockwise: Vw > 0, yaw < 0
    {
        if ((!move_flag_laser) && laser_status <= 18)
        {
            if (SBUS.CH2 > 1074 && SBUS.CH2 <= 1385)
            {
                Robot.Robot_in_world.velocity_exp.Vy = -1.5;
            }
            else if (SBUS.CH2 > 1385 && SBUS.CH2 <= 1695)
            {
                Robot.Robot_in_world.velocity_exp.Vy = -2;
            }
            else if (SBUS.CH2 >= 663 && SBUS.CH2 < 974)
            {
                Robot.Robot_in_world.velocity_exp.Vy = 1.5;
            }
            else if (SBUS.CH2 >= 353 && SBUS.CH2 < 663)
            {
                Robot.Robot_in_world.velocity_exp.Vy = 2;
            }
            else if (SBUS.CH2 >= 974 && SBUS.CH2 <= 1074)
            {
                Robot.Robot_in_world.velocity_exp.Vy = 0;
            }

            if (SBUS.CH1 > 1074 && SBUS.CH1 <= 1385)
            {
                Robot.Robot_in_world.velocity_exp.Vx = 1;
            }
            else if (SBUS.CH1 > 1385 && SBUS.CH1 <= 1695)
            {
                Robot.Robot_in_world.velocity_exp.Vx = 1;
            }
            else if (SBUS.CH1 >= 663 && SBUS.CH1 < 974)
            {
                Robot.Robot_in_world.velocity_exp.Vx = -1;
            }
            else if (SBUS.CH1 >= 353 && SBUS.CH1 < 663)
            {
                Robot.Robot_in_world.velocity_exp.Vx = -1;
            }
            else if (SBUS.CH1 >= 974 && SBUS.CH1 <= 1074)
            {
                Robot.Robot_in_world.velocity_exp.Vx = 0;
            }

            if (SBUS.CH4 > 1074 && SBUS.CH4 <= 1385)
            {
                Robot.Robot_in_world.velocity_exp.Vw = 0.6;
            }
            else if (SBUS.CH4 > 1385 && SBUS.CH4 <= 1695)
            {
                Robot.Robot_in_world.velocity_exp.Vw = 1.0;
            }
            else if (SBUS.CH4 >= 663 && SBUS.CH4 < 974)
            {
                Robot.Robot_in_world.velocity_exp.Vw = -0.6;
            }
            else if (SBUS.CH4 >= 353 && SBUS.CH4 < 663)
            {
                Robot.Robot_in_world.velocity_exp.Vw = -1.0;
            }
            else if (SBUS.CH4 >= 974 && SBUS.CH4 <= 1074)
            {
                Robot.Robot_in_world.velocity_exp.Vw = 0;
            }
        }
        else if ((!move_flag_laser) && laser_status > 18)
        {
            if (SBUS.CH2 > 1074 && SBUS.CH2 <= 1385)
            {
                Robot.Robot_in_world.velocity_exp.Vy = -0.5;
            }
            else if (SBUS.CH2 > 1385 && SBUS.CH2 <= 1695)
            {
                Robot.Robot_in_world.velocity_exp.Vy = -0.5;
            }
            else if (SBUS.CH2 >= 663 && SBUS.CH2 < 974)
            {
                Robot.Robot_in_world.velocity_exp.Vy = 0.5;
            }
            else if (SBUS.CH2 >= 353 && SBUS.CH2 < 663)
            {
                Robot.Robot_in_world.velocity_exp.Vy = 0.5;
            }
            else if (SBUS.CH2 >= 974 && SBUS.CH2 <= 1074)
            {
                Robot.Robot_in_world.velocity_exp.Vy = 0;
            }

            if (SBUS_Analyse_Switch(SBUS.CH8) == 0)
            {
                if (SBUS.CH1 > 1074 && SBUS.CH1 <= 1385)
                {
                    Robot.Robot_in_world.velocity_exp.Vx = 0;
                }
                else if (SBUS.CH1 > 1385 && SBUS.CH1 <= 1695)
                {
                    Robot.Robot_in_world.velocity_exp.Vx = 0;
                }
                else if (SBUS.CH1 >= 663 && SBUS.CH1 < 974)
                {
                    Robot.Robot_in_world.velocity_exp.Vx = 0;
                }
                else if (SBUS.CH1 >= 353 && SBUS.CH1 < 663)
                {
                    Robot.Robot_in_world.velocity_exp.Vx = 0;
                }
                else if (SBUS.CH1 >= 974 && SBUS.CH1 <= 1074)
                {
                    Robot.Robot_in_world.velocity_exp.Vx = 0;
                }
            }
            else if (SBUS_Analyse_Switch(SBUS.CH8) == 2)
            {
                if (SBUS.CH1 > 1074 && SBUS.CH1 <= 1385)
                {
                    Robot.Robot_in_world.velocity_exp.Vx = 0;
                }
                else if (SBUS.CH1 > 1385 && SBUS.CH1 <= 1695)
                {
                    Robot.Robot_in_world.velocity_exp.Vx = 0;
                }
                else if (SBUS.CH1 >= 663 && SBUS.CH1 < 974)
                {
                    Robot.Robot_in_world.velocity_exp.Vx = 0;
                }
                else if (SBUS.CH1 >= 353 && SBUS.CH1 < 663)
                {
                    Robot.Robot_in_world.velocity_exp.Vx = 0;
                }
                else if (SBUS.CH1 >= 974 && SBUS.CH1 <= 1074)
                {
                    Robot.Robot_in_world.velocity_exp.Vx = 0;
                }
            }

            if (SBUS.CH4 > 1074 && SBUS.CH4 <= 1385)
            {
                Robot.Robot_in_world.velocity_exp.Vw = 0.6;
            }
            else if (SBUS.CH4 > 1385 && SBUS.CH4 <= 1695)
            {
                Robot.Robot_in_world.velocity_exp.Vw = 1.0;
            }
            else if (SBUS.CH4 >= 663 && SBUS.CH4 < 974)
            {
                Robot.Robot_in_world.velocity_exp.Vw = -0.6;
            }
            else if (SBUS.CH4 >= 353 && SBUS.CH4 < 663)
            {
                Robot.Robot_in_world.velocity_exp.Vw = -1.0;
            }
            else if (SBUS.CH4 >= 974 && SBUS.CH4 <= 1074)
            {
                Robot.Robot_in_world.velocity_exp.Vw = 0;
            }
        }

        osDelay(5);
    }
    /* USER CODE END SpeedTask */
}

/* USER CODE BEGIN Header_IOTask */

/**
 * @brief Function implementing the IOcontrol thread.
 * @param argument: Not use
 * @retval None
 */
/* USER CODE END Header_IOTask */
void IOTask(void *argument)
{
    /* USER CODE BEGIN IOTask */
    //	uint8_t usart_flag = 1;

    /* Infinite loop */
    for (;;)
    {
        if (laser_status > 18 && SBUS_Analyse_Switch(SBUS.CH12) == 2)
        {
            while (SBUS_Analyse_Switch(SBUS.CH14) == 0)
            {
                osDelay(1);
            }
            // Robot.Robot_in_world.velocity_exp.Vy = 0.7;
            // Robot.Robot_in_world.velocity_exp.Vx = 0;
            // osDelay(100);
            // Robot.Robot_in_world.velocity_exp.Vy = 0;
            // Robot.Robot_in_world.velocity_exp.Vx = 0;
            // osDelay(500);
            if (SBUS_Analyse_Switch(SBUS.CH12) == 2)
            {
                if (WorkChannel >= 0)
                {
                    next_pos_y = WorkChannel / 6 + 1;
                    if (WorkChannel <= 5)
                    {
                        next_pos_x = WorkChannel + 1;
                    }
                    else
                    {
                        next_pos_x = WorkChannel - 5;
                    }
                    HAL_UART_Transmit(&huart6, &(ShootBall_Num[next_pos_x - 1]), 1, 1000);
                    next_distance_x = pos_ball_x[next_pos_x][next_pos_y];
                    next_distance_y = pos_ball_y[next_pos_x][next_pos_y];

                    //----------------------------------------------------
                    //----------------------------------------------------

                    Robot.Robot_in_world.velocity_exp.Vy = 0.7;
                    Robot.Robot_in_world.velocity_exp.Vx = 0;
                    osDelay(100);
                    Robot.Robot_in_world.velocity_exp.Vy = 0;
                    Robot.Robot_in_world.velocity_exp.Vx = 0;
                    osDelay(500);

                    // if (next_distance_x > 3)
                    // {
                    //     Robot.Robot_in_world.velocity_exp.Vy = 0.7;
                    //     Robot.Robot_in_world.velocity_exp.Vx = 0;
                    //     osDelay(100);
                    //     Robot.Robot_in_world.velocity_exp.Vy = 0;
                    //     Robot.Robot_in_world.velocity_exp.Vx = 0;
                    //     osDelay(500);
                    // }
                    //----------------------------------------------------
                    //----------------------------------------------------

                    y_pre = Robot.Robot_in_self.position_now.y;
                    while (pid_done_ball == 0 && SBUS_Analyse_Switch(SBUS.CH12) == 2)
                    {
                        PID_Vx_Laser_area2(next_distance_x, y_pre, 2 * PI / 3);
                        osDelay(1);
                    }
                    pid_done_ball = 0;

                    Robot.Robot_in_world.velocity_exp.Vx = 0;
                    Robot.Robot_in_world.velocity_exp.Vy = 0;
                    osDelay(80);
                    if (next_pos_y == 1)
                    {
                        if (next_pos_x > 3)
                        {
                            x_pre = Robot.Robot_in_self.position_now.x;
                            while (pid_done_ball == 0 && SBUS_Analyse_Switch(SBUS.CH12) == 2)
                            {

                                PID_Vy_Laser_area2(x_pre, next_distance_y, 2 * PI / 3);
                                osDelay(1);
                            }
                            pid_done_ball = 0;

                            if (SBUS_Analyse_Switch(SBUS.CH12) == 2)
                            {
                                HAL_UART_Transmit(&huart6, get_ball, 1, 1000);
                                osDelay(2200);
                            }

                            x_pre = Robot.Robot_in_self.position_now.x;
                            while (pid_done_ball == 0 && SBUS_Analyse_Switch(SBUS.CH12) == 2)
                            {
                                PID_Vy_Laser_area2(x_pre, pos_ball_y[next_pos_x][2], 2 * PI / 3);
                                osDelay(1);
                            }
                            pid_done_ball = 0;

                            // HAL_UART_Transmit(&huart6, shoot_ball, 1, 1000);
                        }
                        else
                        {
                            if (next_pos_x == 1)
                            {
                                y_pre = Robot.Robot_in_self.position_now.y;
                                x_pre = Robot.Robot_in_self.position_now.x;

                                while (pid_done_ball == 0 && SBUS_Analyse_Switch(SBUS.CH12) == 2)
                                {
                                    // PID_Vx_Vy_action(0, pos_ball_y[next_pos_x][2] - pos_ball_y[next_pos_x][1], 2 * PI / 3);
                                    PID_Vy_Laser_area2(x_pre, next_distance_y, 2 * PI / 3);
                                    osDelay(1);
                                }
                                pid_done_ball = 0;

                                if (SBUS_Analyse_Switch(SBUS.CH12) == 2)
                                {
                                    HAL_UART_Transmit(&huart6, get_ball, 1, 1000);
                                    osDelay(2200);
                                }

                                x_pre = Robot.Robot_in_self.position_now.x;
                                y_pre = Robot.Robot_in_self.position_now.y;

                                while (pid_done_ball == 0 && SBUS_Analyse_Switch(SBUS.CH12) == 2)
                                {
                                    // PID_Vx_Vy_action(0, pos_ball_y[next_pos_x][1] - pos_ball_y[next_pos_x][2], 2 * PI / 3);
                                    PID_Vy_Laser_area2(x_pre, pos_ball_y[next_pos_x][2], 2 * PI / 3);
                                    osDelay(1);
                                }
                                pid_done_ball = 0;
                            }

                            // HAL_UART_Transmit(&huart6, shoot_ball, 1, 1000);
                        }
                    }
                    else if (next_pos_y == 2)
                    {
                        //	if(next_pos_x > 3){
                        if (next_pos_x > 3 || next_pos_x == 1)
                        {
                            x_pre = Robot.Robot_in_self.position_now.x;
                            while (pid_done_ball == 0 && SBUS_Analyse_Switch(SBUS.CH12) == 2)
                            {
                                PID_Vy_Laser_area2(x_pre, next_distance_y, 2 * PI / 3);
                                osDelay(1);
                            }
                            pid_done_ball = 0;
                            //	}

                            if (SBUS_Analyse_Switch(SBUS.CH12) == 2)
                            {
                                HAL_UART_Transmit(&huart6, get_ball, 1, 1000);
                                osDelay(1800);
                            }
                            // HAL_UART_Transmit(&huart6, shoot_ball, 1, 1000);
                        }
                    }
                    if (next_pos_x > 3 || next_pos_x == 1)
                    {
                        while (SBUS_Analyse_Switch(SBUS.CH14) == 0 && SBUS_Analyse_Switch(SBUS.CH12) == 2)
                        {
                            osDelay(1);
                        }
                        if (SBUS_Analyse_Switch(SBUS.CH12) == 2)
                        {
                            HAL_UART_Transmit(&huart6, shoot_ball, 1, 1000);
                        }
                    }
                    // Action_Clear();

                    WorkChannel = -1;
                }
                while (SBUS_Analyse_Switch(SBUS.CH14) == 2 && SBUS_Analyse_Switch(SBUS.CH12) == 2)
                {
                    osDelay(1);
                }
            }
        }
        osDelay(1);
    }
    /* USER CODE END IOTask */
}

/* USER CODE BEGIN Header_M3508_Task */
/**
 * @brief Function implementing the M3508_control thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_M3508_Task */
void M3508_Task(void *argument)
{
    /* USER CODE BEGIN M3508_Task */
    /* Infinite loop */
    for (;;)
    {
        Robot_Velocity_World_To_Self();
        Robot_Velocity_To_Wheel_Speed();
        Wheel_Speed_To_Motor();

        // M3508_Motor_Position_Ctrl();
        M3508_Motor_Speed_Ctrl();
        M3508_Motor_Send_Current();
        M3508_Motor_Send_Current_h();

        if (laser_status > 18 && SBUS_Analyse_Switch(SBUS.CH12) == 2)
        {
            M3508SpeedPID4.KP = 7;
            M3508SpeedPID5.KP = 7;
            M3508SpeedPID6.KP = 7;
            M3508SpeedPID4.KI = 0.7;
            M3508SpeedPID5.KI = 0.7;
            M3508SpeedPID6.KI = 0.7;
            // x-aixs
            PID_w_move_distance.KP = 2.5;
            PID_w_move_distance.KD = 0.3;
            if (2 * PI / 3 - Robot.Robot_in_self.position_now.yaw >= -0.02 && 2 * PI / 3 - Robot.Robot_in_self.position_now.yaw <= 0.02)
            {
                PID_w_move_distance.SumErr = 0;
                Robot.Robot_in_world.velocity_exp.Vw = 0;
            }
            else
            {
                Robot.Robot_in_world.velocity_exp.Vw = -PIDCal_pos(&PID_w_move_distance, 2 * PI / 3 - Robot.Robot_in_self.position_now.yaw);
            }
        }
        osDelay(1);
    }
    /* USER CODE END M3508_Task */
}

/* USER CODE BEGIN Header_SerialPlot_Task */
/**
 * @brief Function implementing the SerialPlot_Send thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_SerialPlot_Task */
void SerialPlot_Task(void *argument)
{
    /* USER CODE BEGIN SerialPlot_Task */
    /* Infinite loop */
    for (;;)
    {
        if (SBUS_Analyse_Switch(SBUS.CH11) == 0)
        {
            HAL_UART_Transmit(&huart6, get_ball, 1, 1000);
            while (SBUS_Analyse_Switch(SBUS.CH11) == 0)
            {
                osDelay(1);
            }
        }
        else if (SBUS_Analyse_Switch(SBUS.CH11) == 2)
        {
            HAL_UART_Transmit(&huart6, shoot_ball, 1, 1000);
            while (SBUS_Analyse_Switch(SBUS.CH11) == 2)
            {
                osDelay(1);
            }
        }

        // For R2 restarting
        if (SBUS_Analyse_Switch(SBUS.CH15) == 2 && SBUS_Analyse_Switch(SBUS.CH12) == 2)
        {
            y_pre = Robot.Robot_in_self.position_now.y;
            while (pid_done_ball == 0)
            {
                PID_Vx_Laser_area2(pos_ball_x[6][1], y_pre, 2 * PI / 3);
                osDelay(1);
            }
            pid_done_ball = 0;

            Robot.Robot_in_world.velocity_exp.Vx = 1.4;
            Robot.Robot_in_world.velocity_exp.Vy = 0;
            osDelay(900);

            Robot.Robot_in_world.velocity_exp.Vx = 0;
            Robot.Robot_in_world.velocity_exp.Vy = 0;
            osDelay(50);

            Robot.Robot_in_world.velocity_exp.Vx = 0;
            Robot.Robot_in_world.velocity_exp.Vy = -1;
            osDelay(1500);

            Robot.Robot_in_world.velocity_exp.Vx = 0;
            Robot.Robot_in_world.velocity_exp.Vy = 0;

            // x_pre = Robot.Robot_in_self.position_now.x;
            // while (pid_done_ball == 0)
            // {
            //     PID_Vy_Laser_area2(x_pre, 1.571, 2 * PI / 3);
            //     osDelay(1);
            // }
            // pid_done_ball = 0;
        }
        osDelay(1);
    }
    /* USER CODE END SerialPlot_Task */
}

/* USER CODE BEGIN Header_Restart_Task */
/**
 * @brief Function implementing the Restart thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Restart_Task */
void Restart_Task(void *argument)
{
    /* USER CODE BEGIN Restart_Task */
    /* Infinite loop */
    for (;;)
    {
        // Use new controller to save one-area when there are uncommon situations in one-area
        if (SBUS_Analyse_Switch(SBUS.CH6) == 0)
        {
            move_flag_laser = 0; // �Ȱ��Զ��ص�
            jishu_0_x = 0;
            jishu_0_y = 0;
            jishu_0_w = 0;
            if (laser_status == 2 || laser_status == 3 || laser_status == 7 || laser_status == 8 || laser_status == 12 || laser_status == 13)
            {
                osDelay(1400);
                HAL_UART_Transmit(&huart6, Paw_Command, 1, 1000);
            }
            laser_status++;
            move_flag_laser = 1;
            while (SBUS_Analyse_Switch(SBUS.CH6) == 0)
            {
                osDelay(1);
            }
        }
        // if (SBUS_Analyse_Switch(SBUS.CH10) == 2) {
        //     // The process of restarting
        //     Robot.Robot_in_world.velocity_exp.Vx = 0.5;
        //     Robot.Robot_in_world.velocity_exp.Vy = 1.5;
        //     osDelay(200);

        //     Robot.Robot_in_world.velocity_exp.Vx = 0;
        //     Robot.Robot_in_world.velocity_exp.Vy = 1.5;
        //     osDelay(2000);

        //     laser_status = 17;

        //     while (SBUS_Analyse_Switch(SBUS.CH10) == 2) {
        //         osDelay(1);
        //     }
        // }
        osDelay(1);
    }
    /* USER CODE END Restart_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
