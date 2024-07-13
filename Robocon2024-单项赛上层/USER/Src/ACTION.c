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
void ACTION_Analyze(void)
{
    static union
    {
        uint8_t data[24];
        float data_float[6];
    }ACTION_vaule;
    for(int i=0;i<ACTION_BUFFER_SIZE-ACTION_FRAME_SIZE;i++)
    {
        if(ACTION_RX_BUF[i]==ACTION_FRAME_HEADER_ONE&&ACTION_RX_BUF[i+1]==ACTION_FRAME_HEADER_TWO&&ACTION_RX_BUF[i+ACTION_FRAME_SIZE-1]==ACTION_FRAME_FOOTER_TWO&&ACTION_RX_BUF[i+ACTION_FRAME_SIZE-2]==ACTION_FRAME_FOOTER_ONE)
        {
            for(int j=0;j<ACTION_FRAME_SIZE-4;j++)
            {
                ACTION_vaule.data[j]=ACTION_RX_BUF[i+2+j];
            }
            Robot.Robot_in_self.position_now.yaw=ACTION_vaule.data_float[0]/180.0f*PI;
            Robot.Robot_in_self.position_now.x=ACTION_vaule.data_float[3]/1000;
            Robot.Robot_in_self.position_now.y=ACTION_vaule.data_float[4]/1000;
            Robot.Robot_in_self.velocity_now.Vw=ACTION_vaule.data_float[5]/180.0f*PI;
        }
    }
}
