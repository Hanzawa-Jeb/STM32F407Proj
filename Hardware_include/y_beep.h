

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BEEP_H
#define __BEEP_H

/* Includes ------------------------------------------------------------------*/	 
#include "stm32f4xx.h"
#include "main.h"

// �ӿں���
void BEEP_Init(void);

// ���������������궨��
#define BEEP_Off()       GPIO_ResetBits(GPIOC, GPIO_Pin_13)       //����������
#define BEEP_On()		GPIO_SetBits(GPIOC, GPIO_Pin_13)     //����������
#define BEEP_Toggle()    GPIO_WriteBit(GPIOC, GPIO_Pin_13, (BitAction) (1 - GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_13)))    //������״̬��ת

void beep_on_times(int times, int delay);

#endif 

/******************* (C) ��Ȩ 2022 XTARK **************************************/

