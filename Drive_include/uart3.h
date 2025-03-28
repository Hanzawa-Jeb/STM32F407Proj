
#ifndef __UART3_H
#define __UART3_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"

//X-SOFT 接口函数
void UART3_Init(uint32_t baud);  //UART 调试串口初始化
uint8_t UART3_GetData(uint8_t *pbuf);  //UART 获取接收的数据
void    UART3_SendPacket(uint8_t *pbuf, uint8_t len, uint8_t num);  //UART 发送数据（X-Protocol协议）

#endif 

/******************* (C) 版权 2022 XTARK **************************************/
