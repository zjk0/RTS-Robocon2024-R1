/**
 * @file SBUS.c
 * @author 陶子�?1?7 (2405973406@qq.com)
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

/// @brief SBUS状�?�1?7
SBUS_CH SBUS;
/// @brief SBUS接收缓冲�?1?7
uint8_t SBUS_RX_BUF[SBUS_BUFFER_SIZE];
uint8_t SBUS_Receive[SBUS_Receive_length] = {0};
uint8_t chuankou = 0;
uint8_t pos[10] = {'a', 'b', 'c', 'd', 'e', 'f', 'g', 'h', 'i', '\n'};
uint8_t distan[10] = {0};

uint8_t lllflag = 0;

// sprintf(strff,"%.2f",ff);
float yaw_temp = 0;
static union
{
    uint8_t data[32];
    uint16_t SBUS_CH[16];
} SBUS_vaule;

static union
{
    uint8_t data_receive[8];
    int16_t data[4];
} Laser_value;

static union
{
    uint8_t data_receive[8];
    float data[2];
} IMU_data;

static union
{
    uint8_t data_receive[4];
    int16_t data[2];
} Action_data;

uint8_t NewControllerData[12] = {0};
uint8_t NewControllerData_pre[12] = {0};
int WorkChannel = -1;

uint8_t SBUS_ch = 0; // 接受�?1?7
/// @brief SBUS接收标志
uint8_t SBUS_flag = 0; // 接收标志
/// @brief SBUS接收状�?��?
uint8_t SBUS_step = 0; // 状�?��?
/// @brief SBUS接收计数
uint8_t SBUS_count = 0; // 计数

uint8_t Laser_index = 0;
uint8_t IMU_index = 0;
uint8_t Action_index = 0;
uint8_t NewController_index = 0;
/*
*********************************************************************************************************
*                                               函数实现
*********************************************************************************************************
*/

/**
 * @brief 分析拨杆位置
 * @param SBUS_CHANNEL_VALUE SBUS通道�?1?7
 * @return 拨杆位置,0为上,1为中,2为下�?1?7-1为无效�?�1?7
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
/**
 * @brief 分析百分比位�?1?7
 * @param SBUS_CHANNEL_VALUE SBUS通道�?1?7
 * @return 百分比位�?1?7,0~1�?1?7-1为无效�?�1?7
 */
float SBUS_Analyse_Percent(int16_t SBUS_CHANNEL_VALUE)
{
    if (SBUS_CHANNEL_VALUE < 353 || SBUS_CHANNEL_VALUE > 1695)
        return -1;
    else
        return (SBUS_CHANNEL_VALUE - 353) / 1342.0f;
}

/**
 * @name Laser_Data_Analyse
 *
 * @brief Convert received data to the data we need
 *
 * @param data: The data which is needed to be converted
 *
 * @return The data which has been converted
 */
float Laser_Data_Analyse(float data, float max_available_distance, float min_available_distance)
{
    float result;
    result = (data * 1.0 / 16384) * (max_available_distance - min_available_distance) + min_available_distance;
    return result;
}

/**
 * @brief  SBUS接受�?�?
 * @param  None
 * @retval None

*/
void SBUS_Receive_Analyze(void)
{

    switch (SBUS_step)
    {
    case 0:
        if (SBUS_ch == SBUS_Head)
        {
            SBUS_step++;
        }
        else
        {
            SBUS_step = 0;
        }
        break;
    case 1:
        if (SBUS_ch == SBUS_Head_)
        {
            SBUS_count = 0;
            Laser_index = 0;
            IMU_index = 0;
            Action_index = 0;
            NewController_index = 0;
            SBUS_step++;
        }
        else if (SBUS_ch == SBUS_Head)
        {
            SBUS_step = 1;
        }
        else
        {
            SBUS_step = 0;
        }
        break;
    case 2:
        SBUS_vaule.data[SBUS_count++] = SBUS_ch;
        if (SBUS_count >= 32)
        {
            SBUS_step++;
        }
        break;
    case 3:
        Laser_value.data_receive[Laser_index++] = SBUS_ch;
        if (Laser_index > 7)
        {
            SBUS_step++;
        }
        break;
    case 4:
        IMU_data.data_receive[IMU_index++] = SBUS_ch;
        if (IMU_index > 7)
        {
            SBUS_step++;
        }
        break;
    case 5:
        upper_command = SBUS_ch;
        SBUS_step++;
        break;
    case 6:
        Action_data.data_receive[Action_index++] = SBUS_ch;
        if (Action_index > 3)
        {
            SBUS_step++;
        }
        break;
    case 7:
        NewControllerData_pre[NewController_index] = NewControllerData[NewController_index];
        NewControllerData[NewController_index++] = SBUS_ch;
        if (NewController_index > 11)
        {
            SBUS_step++;
        }
        break;
    case 8:
        if (SBUS_ch == SBUS_Tail)
        {
            SBUS_step++;
        }
        else
        {
            SBUS_step = 0;
        }
        break;
    case 9:
        if (SBUS_ch == SBUS_Tail_)
        {
            SBUS.CH1 = SBUS_vaule.SBUS_CH[0];
            SBUS.CH2 = SBUS_vaule.SBUS_CH[1];
            SBUS.CH3 = SBUS_vaule.SBUS_CH[2];
            SBUS.CH4 = SBUS_vaule.SBUS_CH[3];
            SBUS.CH5 = SBUS_vaule.SBUS_CH[4];
            SBUS.CH6 = SBUS_vaule.SBUS_CH[5];
            SBUS.CH7 = SBUS_vaule.SBUS_CH[6];
            SBUS.CH8 = SBUS_vaule.SBUS_CH[7];
            SBUS.CH9 = SBUS_vaule.SBUS_CH[8];
            SBUS.CH10 = SBUS_vaule.SBUS_CH[9];
            SBUS.CH11 = SBUS_vaule.SBUS_CH[10];
            SBUS.CH12 = SBUS_vaule.SBUS_CH[11];
            SBUS.CH13 = SBUS_vaule.SBUS_CH[12];
            SBUS.CH14 = SBUS_vaule.SBUS_CH[13];
            SBUS.CH15 = SBUS_vaule.SBUS_CH[14];
            SBUS.CH16 = SBUS_vaule.SBUS_CH[15];

            //			if(Laser_value.data[0] > 0 && Laser_value.data[1] > 0){
            //			Laser_Data_1 = Laser_Data_Analyse(Laser_value.data[0], 6.451, 0.19573);
            //			Laser_Data_2 = Laser_Data_Analyse(Laser_value.data[1], 4.575, 0.303);
            //			}
            if (Laser_Data_Analyse(Laser_value.data[0], 8.51916, 0.29512) > 6)
            {
                // HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
            }
            else
            {
                Laser_Data_1 = Laser_Data_Analyse(Laser_value.data[0], 8.51916, 0.29512);
                // HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_3);
            }
            Laser_Data_1 = Laser_Data_Analyse(Laser_value.data[0], 8.51916, 0.29512);
            if (Laser_Data_Analyse(Laser_value.data[1], 6.982918, 0.20239) > 3)
            {
                // HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
				//Laser_Data_2 = 3;
				Laser_Data_2 = Laser_Data_Analyse(Laser_value.data[1], 8.78938, 0.10345);

            }
            else
            {
            	Laser_Data_2 = Laser_Data_Analyse(Laser_value.data[1], 8.78938, 0.10345);
                // HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_3);
            }
            Laser_Data_3 = Laser_Data_Analyse(Laser_value.data[2], 8.3724865, 0.179987);
            Laser_Data_4 = Laser_Data_Analyse(Laser_value.data[3], 7.5733634, 0.176523);
            if(Laser_Data_1 > 4)
              Laser_Data_1 = 4;
            if(Laser_Data_2 > 4)
              Laser_Data_2 = 4;            
            if(Laser_Data_3 > 4)
              Laser_Data_3 = 4;          
            if(Laser_Data_4 > 4)
              Laser_Data_4 = 4;
            IMU_yaw = IMU_data.data[0];
            IMU_Vw = IMU_data.data[1];
            yaw_temp = IMU_yaw;

            Action_x = Action_data.data[0] * 0.001;
            Action_y = Action_data.data[1] * 0.001;

            for (uint8_t i = 0; i < 12; i++) {
                if ((NewControllerData[i] == 1 && NewControllerData_pre[i] == 0) || (NewControllerData[i] == 0 && NewControllerData_pre[i] == 1)) {
                    WorkChannel = i;
                    break;
                }
            }

            //            chuankou++;
            //            if (chuankou == 10)
            //            {
            // sprintf(distan,"%.3f",Laser_Data_1);
            // HAL_UART_Transmit(&huart1,distan, 4,1000);
            // sprintf(distan,"%.3f",Laser_Data_2);
            // HAL_UART_Transmit(&huart1,distan, 4,1000);
            //                chuankou = 0;
            //            }

            if (yaw_temp < 0)
            {
                Robot.Robot_in_self.position_now.yaw = 2 * PI + yaw_temp;
            }
            else
            {
                Robot.Robot_in_self.position_now.yaw = yaw_temp;
            }
            if (upper_command == 1)
            {
                // HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
                if (laser_status == 1 || laser_status == 5 || laser_status == 9 || laser_status == 13 || laser_status == 17)
                {
                    // HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
                    laser_status++;
                    // HAL_UART_Transmit(&huart1, &laser_status, 1, 1000);
                    // HAL_UART_Transmit(&huart1, &pos[2], 1, 1000);
                    // HAL_UART_Transmit(&huart1, &pos[9], 1, 1000);
                }
                HAL_UART_Transmit(&huart6, Ack, 1, 1000);
                upper_command = 0;
            }
            //            else if (upper_command == 2)
            //            { // ��һ�����ӷ������󣬾ͻ��յ�2��������ʹ�ܹ�翪��
            //                HAL_UART_Transmit(&huart6, Enable_Command, 1, 1000);
            //                upper_command = 0;
            //            }
            // else if (upper_command == 3) {
            //     if (laser_status == 23 || laser_status == 24 || laser_status == 26 || laser_status == 27 || laser_status == 29 || laser_status == 30) {
            //         laser_status++;
            //     }
            //     HAL_UART_Transmit(&huart6, Ack, 1, 1000);
            //     upper_command = 0;
            // }
            else if (upper_command == 4)
            {
                HAL_UART_Transmit(&huart6, Ack, 1, 1000);
                lllflag = 1;
                upper_command = 0;
            }
        }
        SBUS_step = 0;
        break;
    default:
        SBUS_step = 0;
        break;
    }
}

/**
 * @brief  SBUS接受与解�?1?7
 * @param  None
 * @retval None
 */
void SBUS_Analyze(void)
{
    // uint8_t i=0;

    // if(SBUS_RX_BUF[i]==SBUS_FRAME_HEADER&&SBUS_RX_BUF[i+SBUS_FRAME_SIZE-1]==SBUS_FRAME_FOOTER)
    // {
    //     SBUS.CH1=((int16_t)SBUS_RX_BUF[i+1]>>0|((int16_t)SBUS_RX_BUF[i+2]<<8))&0x07FF;
    //     SBUS.CH2=((int16_t)SBUS_RX_BUF[i+2]>>3|((int16_t)SBUS_RX_BUF[i+3]<<5))&0x07FF;
    //     SBUS.CH3=((int16_t)SBUS_RX_BUF[i+3]>>6|((int16_t)SBUS_RX_BUF[i+4]<<2)|(int16_t)SBUS_RX_BUF[5]<<10)&0x07FF;
    //     SBUS.CH4=((int16_t)SBUS_RX_BUF[i+5]>>1|((int16_t)SBUS_RX_BUF[i+6]<<7))&0x07FF;
    //     SBUS.CH5=((int16_t)SBUS_RX_BUF[i+6]>>4|((int16_t)SBUS_RX_BUF[i+7]<<4))&0x07FF;
    //     SBUS.CH6=((int16_t)SBUS_RX_BUF[i+7]>>7|((int16_t)SBUS_RX_BUF[i+8]<<1)|(int16_t)SBUS_RX_BUF[9]<<9)&0x07FF;
    //     SBUS.CH7=((int16_t)SBUS_RX_BUF[i+9]>>2|((int16_t)SBUS_RX_BUF[i+10]<<6))&0x07FF;
    //     SBUS.CH8=((int16_t)SBUS_RX_BUF[i+10]>>5|((int16_t)SBUS_RX_BUF[i+11]<<3))&0x07FF;
    //     SBUS.CH9=((int16_t)SBUS_RX_BUF[i+12]>>0|((int16_t)SBUS_RX_BUF[i+13]<<8))&0x07FF;
    //     SBUS.CH10=((int16_t)SBUS_RX_BUF[i+13]>>3|((int16_t)SBUS_RX_BUF[i+14]<<5))&0x07FF;
    //     SBUS.CH11=((int16_t)SBUS_RX_BUF[i+14]>>6|((int16_t)SBUS_RX_BUF[i+15]<<2)|(int16_t)SBUS_RX_BUF[i+16]<<10)&0x07FF;
    //     SBUS.CH12=((int16_t)SBUS_RX_BUF[i+16]>>1|((int16_t)SBUS_RX_BUF[i+17]<<7))&0x07FF;
    //     SBUS.CH13=((int16_t)SBUS_RX_BUF[i+17]>>4|((int16_t)SBUS_RX_BUF[i+18]<<4))&0x07FF;
    //     SBUS.CH14=((int16_t)SBUS_RX_BUF[i+18]>>7|((int16_t)SBUS_RX_BUF[i+19]<<1)|(int16_t)SBUS_RX_BUF[i+20]<<9)&0x07FF;
    //     SBUS.CH15=((int16_t)SBUS_RX_BUF[i+20]>>2|((int16_t)SBUS_RX_BUF[i+21]<<6))&0x07FF;
    //     SBUS.CH16=((int16_t)SBUS_RX_BUF[i+21]>>5|((int16_t)SBUS_RX_BUF[i+22]<<3))&0x07FF;
    //     if((SBUS_RX_BUF[i+23]&0x4)&&(SBUS_RX_BUF[i+23]&0x8))
    //         SBUS.ConnectState=0;
    //     else
    //         SBUS.ConnectState=100;
    // }

    // SBUS.CH1=((int16_t)data[0]>>0|((int16_t)data[1]<<8))&0x07FF;
    // SBUS.CH2=((int16_t)data[1]>>3|((int16_t)data[2]<<5))&0x07FF;
    // SBUS.CH3=((int16_t)data[2]>>6|((int16_t)data[3]<<2)|(int16_t)data[4]<<10)&0x07FF;
    // SBUS.CH4=((int16_t)data[4]>>1|((int16_t)data[5]<<7))&0x07FF;
    // SBUS.CH5=((int16_t)data[5]>>4|((int16_t)data[6]<<4))&0x07FF;
    // SBUS.CH6=((int16_t)data[6]>>7|((int16_t)data[7]<<1)|(int16_t)data[8]<<9)&0x07FF;
    // SBUS.CH7=((int16_t)data[8]>>2|((int16_t)data[9]<<6))&0x07FF;
    // SBUS.CH8=((int16_t)data[9]>>5|((int16_t)data[10]<<3))&0x07FF;
    // SBUS.CH9=((int16_t)data[11]>>0|((int16_t)data[12]<<8))&0x07FF;
    // SBUS.CH10=((int16_t)data[12]>>3|((int16_t)data[13]<<5))&0x07FF;
    // SBUS.CH11=((int16_t)data[13]>>6|((int16_t)data[14]<<2)|(int16_t)data[15]<<10)&0x07FF;
    // SBUS.CH12=((int16_t)data[15]>>1|((int16_t)data[16]<<7))&0x07FF;
    // SBUS.CH13=((int16_t)data[16]>>4|((int16_t)data[17]<<4))&0x07FF;
    // SBUS.CH14=((int16_t)data[17]>>7|((int16_t)data[18]<<1)|(int16_t)data[19]<<9)&0x07FF;
    // SBUS.CH15=((int16_t)data[19]>>2|((int16_t)data[20]<<6))&0x07FF;
    // SBUS.CH16=((int16_t)data[20]>>5|((int16_t)data[21]<<3))&0x07FF;
}