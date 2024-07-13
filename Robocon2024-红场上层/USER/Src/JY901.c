#include "JY901.h"

JY901 JY901_Data;
// struct SAcc 		stcAcc;
struct SGyro 		stcGyro;
struct SAngle 	stcAngle;

//接收结构体初始化
void JY901_USART_Init(JY901 *Data)
{
		for(uint16_t i=0; i < JY901_RXBUFFER_LEN; i++)	Data->RxBuffer[i] = 0;
		Data->frame_head = 0x55;
		Data->Rx_flag = 0;
		Data->Rx_len = 0;
}

void JY901_Process()
{
		if(JY901_Data.Rx_len < JY901_RXBUFFER_LEN) return;   	//如果位数不对

		for(uint8_t i=0;i<2;i++)
		{
				if(JY901_Data.RxBuffer[i*11]!= JY901_Data.frame_head) return;	//如果帧头不对
				switch(JY901_Data.RxBuffer[i*11+1])
				{
						// case 0x51:	
						// 	memcpy(&stcAcc,&JY901_Data.RxBuffer[2 + i*11],8);
						// 	for(uint8_t j = 0; j < 3; j++) JY901_Data.acc.a[j] = (float)stcAcc.a[j]/32768*16;									//官方加速度解算
						// break;
						case 0x52:	
							memcpy(&stcGyro,&JY901_Data.RxBuffer[2 + i*11],8);
							for(uint8_t j = 0; j < 3; j++) JY901_Data.w.w[j] = (float)stcGyro.w[j]/32768*2000;								//官方角速度解算
						break;
						case 0x53:	
							memcpy(&stcAngle,&JY901_Data.RxBuffer[2 + i*11],8);
							for(uint8_t j = 0; j < 3; j++) JY901_Data.angle.angle[j] = (float)stcAngle.angle[j]/32768*180;		//官方角度解算
						break;
				}
				
		}

}
