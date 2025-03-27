#ifndef __MAIN_H
#define __MAIN_H
#include <stdio.h>
#include <math.h>

#include "stm32f4xx.h"

#include "stdio.h"

#include "Sys.h"
#include "GPIO_int.h"

#include "uart1.h"
#include "y_motor.h"

typedef struct
{
	double RT; // 电机的实时速度，单位m/s
	float TG;  // 设置的目标速度，单位m/s
	short PWM; // 计算出的PWM控制速度
} MOTOR_T;

extern MOTOR_T, Motor_A, Motor_B, Motor_C, Motor_D;

void Motor_Init(void);
void Motor_Control_Task(void);

#endif
