/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __Y_MOTOR_H
#define __Y_MOTOR_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "main.h"

//X-SOFT �ӿں���

//���PID�ջ��ٶȿ��ƺ���
int16_t SPEED_PidCtlA(float spd_target, float spd_current);   //PID���ƺ��������A
int16_t SPEED_PidCtlB(float spd_target, float spd_current);    //PID���ƺ��������B
int16_t SPEED_PidCtlC(float spd_target, float spd_current);    //PID���ƺ��������C
int16_t SPEED_PidCtlD(float spd_target, float spd_current);    //PID���ƺ��������D

void MOTOR_A_SetSpeed(int16_t speed);   //���A����
void MOTOR_B_SetSpeed(int16_t speed);   //���B����

void MOTOR_C_SetSpeed(int16_t speed);   //���C����
void MOTOR_D_SetSpeed(int16_t speed);   //���D����

#endif

/******************* (C) ��Ȩ 2022 XTARK **************************************/
