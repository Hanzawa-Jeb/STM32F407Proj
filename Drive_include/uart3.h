
#ifndef __UART3_H
#define __UART3_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"

//X-SOFT �ӿں���
void UART3_Init(uint32_t baud);  //UART ���Դ��ڳ�ʼ��
uint8_t UART3_GetData(uint8_t *pbuf);  //UART ��ȡ���յ�����
void    UART3_SendPacket(uint8_t *pbuf, uint8_t len, uint8_t num);  //UART �������ݣ�X-ProtocolЭ�飩

#endif 

/******************* (C) ��Ȩ 2022 XTARK **************************************/
