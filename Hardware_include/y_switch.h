
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __Y_SWITCH_H
#define __Y_SWITCH_H

/* Includes ------------------------------------------------------------------*/	 
#include "stm32f4xx.h"
#include "main.h"

uint8_t KEY_Scan(void);
//X-SOFT �ӿں���
void AX_SW_Init(void);       //���뿪�س�ʼ��
uint8_t AX_SW_GetS1(void);   //��ȡ����1״̬
uint8_t AX_SW_GetS2(void);   //��ȡ����2״̬
uint8_t AX_SW_GetS12(void);  //��ȡ������״̬

#endif 

/******************* (C) ��Ȩ 2022 XTARK **************************************/
