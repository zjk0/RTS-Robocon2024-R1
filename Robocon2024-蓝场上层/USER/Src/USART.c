/*
*********************************************************************************************************
*                                             INCLUDE FILES
*********************************************************************************************************
*/
#include "USART.h"
#include "main.h"
/*
*********************************************************************************************************
*                                               CONSTANTS
*********************************************************************************************************
*/
static uint16_t U2(uint8_t *p) {uint16_t u; memcpy(&u,p,2); return u;}
static uint32_t U4(uint8_t *p) {uint32_t u; memcpy(&u,p,4); return u;}
static float    R4(uint8_t *p) {float    r; memcpy(&r,p,4); return r;}


/*
*********************************************************************************************************
*                                               函数实现
*********************************************************************************************************
*/
/**
 * @brief  串口接收中断回调函数
 * @param  huart: 串口句柄
 * @retval None
 * @note   
 *         
 *         
 * 
 */

static int parse_data(raw_t *raw)
{
    uint8_t *p = &raw->buf[CH_HDR_SIZE];
    /*memset(raw->item_code, 0, sizeof(raw->item_code));
    raw->nitem_code = 0;*/
    raw->imu.id = U1(p+1);
    raw->imu.temperature = U1(p+3);
    raw->imu.pressure = R4(p+4);
    raw->imu.timestamp = U4(p+8);
    raw->imu.acc[0] = R4(p+12);
    raw->imu.acc[1] = R4(p+16);
    raw->imu.acc[2] = R4(p+20);
    raw->imu.gyr[0] = R4(p+24);
    raw->imu.gyr[1] = R4(p+28);
    raw->imu.gyr[2] = R4(p+32);
    raw->imu.mag[0] = R4(p+36);
    raw->imu.mag[1] = R4(p+40);
    raw->imu.mag[2] = R4(p+44);
    raw->imu.eul[0] = R4(p+48);
    raw->imu.eul[1] = R4(p+52);
    raw->imu.eul[2] = R4(p+56);
    raw->imu.quat[0] = R4(p+60);
    raw->imu.quat[1] = R4(p+64);
    raw->imu.quat[2] = R4(p+68);
    raw->imu.quat[3] = R4(p+72);
		
    return 1;
}

static void crc16_update(uint16_t *currect_crc, const uint8_t *src, uint32_t len)
{
    uint32_t crc = *currect_crc;
    uint32_t j;
    for (j=0; j < len; ++j)
    {
        uint32_t i;
        uint32_t byte = src[j];
        crc ^= byte << 8;
        for (i = 0; i < 8; ++i)
        {
            uint32_t temp = crc << 1;
            if (crc & 0x8000)
            {
                temp ^= 0x1021;
            }
            crc = temp;
        }
    } 
    *currect_crc = crc;
}

static int decode_ch(raw_t *raw)
{
    uint16_t crc = 0;   

    /* checksum */
    crc16_update(&crc, raw->buf, 4);
    crc16_update(&crc, raw->buf+6, raw->len);
    if (crc != U2(raw->buf+4))
    {
       // CH_TRACE("ch checksum error: frame:0x%X calcuate:0x%X, len:%d\n", U2(raw->buf+4), crc, raw->len);
        return -1;
    }
    
    return parse_data(raw);
}

static int sync_ch(uint8_t *buf, uint8_t data)
{
    buf[0] = buf[1];
    buf[1] = data;
    return buf[0] == CHSYNC1 && buf[1] == CHSYNC2;
}

int ch_serial_input(raw_t *raw, uint8_t data)
{
    /* synchronize frame */
    if (raw->nbyte == 0)
    {
        if (!sync_ch(raw->buf, data)) return 0;
        raw->nbyte = 2;
        return 0;
    }

    raw->buf[raw->nbyte++] = data;
    
    if (raw->nbyte == CH_HDR_SIZE)
    {
        if ((raw->len = U2(raw->buf+2)) > (MAXRAWLEN - CH_HDR_SIZE))
        {
            //CH_TRACE("ch length error: len=%d\n",raw->len);
            raw->nbyte = 0;
            return -1;
        }
    }
    
    if (raw->nbyte < (raw->len + CH_HDR_SIZE)) 
    {
        return 0;
    }
    
    raw->nbyte = 0;
    
    return decode_ch(raw);
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	//串口1SBUS解析
	
	//串口2激光解析
//	else if(huart->Instance == USART2)
//	{
//		if(Size==LASER_FRAME_SIZE)
//		LASER_Analyze(1);
//		HAL_UARTEx_ReceiveToIdle_IT(&huart2,LASER_RX_BUF[1],LASER_FRAME_SIZE);
//	}
	//串口3激光解析

        if(huart->Instance == UART8)
	{
		if(Size==LASER_FRAME_SIZE)
		LASER_Analyze(2);
		HAL_UARTEx_ReceiveToIdle_IT(&huart3,LASER_RX_BUF[2],LASER_FRAME_SIZE);
	}
	//串口8激光解析
//	else if(huart->Instance == UART8)
//	{
//		if(Size==LASER_FRAME_SIZE)
//		LASER_Analyze(3);
//		HAL_UARTEx_ReceiveToIdle_IT(&huart8,LASER_RX_BUF[3],LASER_FRAME_SIZE);
//	}
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
        if(huart->Instance == USART1)
	{
          SBUS_Analyze();  
          HAL_UART_Receive_IT(&huart1, &SBUS_Receive_ch, 1); // 接收到一帧数据后再次开启空闲中断
	}
        if(huart->Instance == UART7){
          HAL_UART_Receive_IT(&huart7,&bin,1);
        }
        if(huart->Instance == UART8)
	{
            uint8_t ch;
            ch=rx_buf[0];
            HAL_UART_Receive_IT(&huart8,&rx_buf[0],1);
            decode_succ = ch_serial_input(&raw, ch);
	}
  if(huart->Instance == USART3)
	{
          SBUS_ours_Analyze();  
          HAL_UART_Receive_IT(&huart3,&SBUS_ours_Receive_ch,1);
//          for(uint8_t i = 0;i<4;i++){
//          action_data[i] = action_us[i];
//          }
	}
        
        
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == AD7606_BUSY_P1_Pin)
    {
        HAL_GPIO_WritePin(AD7606_CS_O2_GPIO_Port,AD7606_CS_O2_Pin,GPIO_PIN_RESET);
for(uint8_t i = 0;i<15;i++){
__NOP();
}
        //for(int8_t p = 1;p >=0;p--){
        //HAL_SPI_Transmit(&hspi4,&empty,1,0xffff);
        HAL_SPI_Receive(&hspi4,(uint8_t*)ad7606_value,4,0xffff);
        //}
        HAL_GPIO_WritePin(AD7606_CS_O2_GPIO_Port,AD7606_CS_O2_Pin,GPIO_PIN_SET);
        
    }
}


//IMU函数
//void ch_dump_imu_data(raw_t *raw)
//{
//  CH_TRACE("%-16s%.3f %.3f %.3f\r\n",       "acc(G):",        raw->imu.acc[0], raw->imu.acc[1],  raw->imu.acc[2]);
//  CH_TRACE("%-16s%.3f %.3f %.3f\r\n",       "gyr(deg/s):",    raw->imu.gyr[0], raw->imu.gyr[1],  raw->imu.gyr[2]);
//  CH_TRACE("%-16s%.3f %.3f %.3f\r\n",       "mag(uT):",       raw->imu.mag[0], raw->imu.mag[1],  raw->imu.mag[2]);
//  CH_TRACE("%-16s%.3f %.3f %.3f\r\n",       "eul(deg):",      raw->imu.eul[0], raw->imu.eul[1],  raw->imu.eul[2]);
//  CH_TRACE("%-16s%.3f %.3f %.3f %.3f\r\n",  "quat:",          raw->imu.quat[0], raw->imu.quat[1],  raw->imu.quat[2], raw->imu.quat[3]);
//  CH_TRACE("%-16s%.3f\r\n",       "presure(pa):",  raw->imu.pressure);
//  CH_TRACE("%-16s%d\r\n",       "timestamp(ms):",  raw->imu.timestamp);
//}

