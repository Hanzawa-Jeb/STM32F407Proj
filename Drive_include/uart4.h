
#ifndef __UART4_H
#define __UART4_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"

#include "y_global.h"

void UART4_Init(uint32_t baud); // UART ���Դ��ڳ�ʼ��
void uart4_send_str(u8 *s);

#endif

/******************* (C) ��Ȩ 2022 XTARK **************************************/
