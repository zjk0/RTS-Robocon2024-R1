/*
*********************************************************************************************************
*                                             INCLUDE FILES
*********************************************************************************************************
*/
#include "CAN.h"

/*
*********************************************************************************************************
*                                               CONSTANTS
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*                                               函数实现
*********************************************************************************************************
*/
/**
 * @brief  CANFIFO0接收中断回调函数
 * @param  hcan: CAN句柄
 * @retval None
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
    HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rx_header, rx_data);
    switch (rx_header.StdId)
    {
    case 0x201:
    case 0x202:
    case 0x203:
    case 0x204:
    case 0x205:
    case 0x206:
    case 0x207:
    case 0x208:
        M3508_Analyze(rx_header.StdId, rx_data);
        break;
        //        case 0x209:
        //        case 0x20A:
        //        case 0x20B:
        //        case 0x20C:
        //            GM6030_Analyze(rx_header.StdId, rx_data);
        //            break;
    }
}
/*
*********************************************************************************************************
*                                               函数实现
*********************************************************************************************************
*/
/**
 * @brief  CANFIFO1接收中断回调函数
 * @param  hcan: CAN句柄
 * @retval None
 */
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
    HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO1, &rx_header, rx_data);
    switch (rx_header.StdId)
    {

    case 0x209:
    case 0x20A:
    case 0x20B:
    case 0x20C:
        GM6030_Analyze(rx_header.StdId, rx_data);
        break;
    }
}
