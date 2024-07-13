/*
*********************************************************************************************************
*                                             INCLUDE FILES
*********************************************************************************************************
*/
#include "ACTION.h"

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
uint8_t ACTION_RX_BUF[ACTION_BUFFER_SIZE];
uint8_t Action_Flag = 0x00;
float temp_x = 0,temp_y = 0;
/// @brief Action接收缓冲区
static union
{
    uint8_t data[24];
    float data_float[6];
} ACTION_vaule;
/// @brief Action接收值
uint8_t Action_ch = 0; // 接受值
/// @brief Action接收标志
__attribute__((unused)) static uint8_t Action_flag = 0; // 接收标志
/// @brief Action接收状态机
static uint8_t Action_step = 0; // 状态机
/// @brief Action接收计数
static uint8_t Action_count = 0; // 计数
float action_temp = 0;
/*
*********************************************************************************************************
*                                               函数实现
*********************************************************************************************************
*/
/**
 * @brief  ACTION接收与解码
 * @param  None
 * @retval None
 */
 int i=0;
 uint8_t sh[100]={0};

void ACTION_Analyze(void)
{  
  sh[i]=Action_ch;
     if(i<50)   
     {
       i++;}
     else
       i=0;
    switch (Action_step)
    {  
    case 0:
        if (Action_ch == 0x0d)
        {
            Action_step++;
        }
        else 
        {
            Action_step = 0;
        }
        break;
    case 1:
        if (Action_ch == 0x0A)
        {
            Action_count = 0;
            Action_step++;
        }
        else if (Action_ch == 0x0D)
        {
           // Action_step = 1;
        }
        else
        {
            Action_step = 0;
        }
        break;
    case 2:
        ACTION_vaule.data[Action_count] = Action_ch;
        Action_count++;
        if (Action_count >= 24)
        {
            Action_count = 0;
            Action_step++;
        }
         break;
    case 3:
        if (Action_ch == 0x0A)
        {
            Action_step++;
        }
        else
        {
            Action_step = 0;
        }
        break;
    case 4:
        if (Action_ch == 0x0D)
        {
            //Robot.Robot_in_self.position_now.yaw = ACTION_vaule.data_float[0] / 180.0f * PI;
            temp_x = ACTION_vaule.data_float[3] / 1000;
            temp_y = ACTION_vaule.data_float[4] / 1000;
			Robot.Robot_in_self.velocity_now.Vw = ACTION_vaule.data_float[5] / 180.0f * PI;
			//action_temp =  ACTION_vaule.data_float[0] / 180.0f * PI;
			
            
//            Robot.Robot_in_self.position_now.y = -temp_y * cos(PI / 3) - temp_x * sin(PI / 3);
//            Robot.Robot_in_self.position_now.x = temp_y * sin(PI / 3) - temp_x * cos(PI / 3);
			Robot.Robot_in_self.position_now.x = temp_x;
			Robot.Robot_in_self.position_now.y = temp_y;
        }
        Action_step = 0;
        break;
    default:
        Action_step = 0;
        break;
    }
}

void Action_Clear(void) {
    uint8_t Action_Clear_Command[4];
    Action_Clear_Command[0] = 'A';
    Action_Clear_Command[1] = 'C';
    Action_Clear_Command[2] = 'T';
    Action_Clear_Command[3] = '0';
    HAL_UART_Transmit(&huart1, Action_Clear_Command, 4, 1000);
}

uint8_t signal_flag = 0;

void Action_Signal_Send(void) {
    if (signal_flag == 1) {
        return;
    }

    uint8_t signal[1] = {0x01};
    
    //if (Robot.Robot_in_self.position_now.x != 0 || Robot.Robot_in_self.position_now.y != 0) {
        HAL_UART_Transmit(&huart6, signal, 1, 1000);
        signal_flag = 1;
   // }
}



/*
void ACTION_Analyze(void)
{

    for (int i = 0; i < ACTION_BUFFER_SIZE - ACTION_FRAME_SIZE; i++)
    {
        if (ACTION_RX_BUF[i] == ACTION_FRAME_HEADER_ONE && ACTION_RX_BUF[i + 1] == ACTION_FRAME_HEADER_TWO && ACTION_RX_BUF[i + ACTION_FRAME_SIZE - 1] == ACTION_FRAME_FOOTER_TWO && ACTION_RX_BUF[i + ACTION_FRAME_SIZE - 2] == ACTION_FRAME_FOOTER_ONE)
        {
            for (int j = 0; j < ACTION_FRAME_SIZE - 4; j++)
            {
                ACTION_vaule.data[j] = ACTION_RX_BUF[i + 2 + j];
            }
            Robot.Robot_in_self.position_now.yaw = ACTION_vaule.data_float[0] / 180.0f * PI;
            Robot.Robot_in_self.position_now.x = ACTION_vaule.data_float[3] / 1000;
            Robot.Robot_in_self.position_now.y = ACTION_vaule.data_float[4] / 1000;
            Robot.Robot_in_self.velocity_now.Vw = ACTION_vaule.data_float[5] / 180.0f * PI;

            if (Robot.Robot_in_self.position_now.y != 0)
                Action_Flag = 0x01;
        }
    }
}

*/