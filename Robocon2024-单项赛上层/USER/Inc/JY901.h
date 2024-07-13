#include "main.h"

#ifndef __JY901_H
#define __JY901_H
#define JY901_RXBUFFER_LEN 22

struct SAcc
{
	short a[3];
	short T;
};
struct SGyro
{
	short w[3];
	short T;
};
struct SAngle
{
	short angle[3];
	short T;
};

typedef struct
{
	float angle[3];
} Angle;

// typedef struct
// {
// 	float a[3];
// } Acc;

typedef struct
{
	float w[3];
} Gyro;

typedef struct
{
	uint8_t Rx_flag;
	uint8_t Rx_len;
	uint8_t frame_head;				// 帧头
	uint8_t RxBuffer[JY901_RXBUFFER_LEN]; // 接收缓冲
	Angle angle;					// 角度
	// Acc acc;						// 加速度
	Gyro w;						// 角速度
} JY901;

extern JY901 JY901_Data;

void JY901_USART_Init(JY901 *Data);
void JY901_Process();

#endif