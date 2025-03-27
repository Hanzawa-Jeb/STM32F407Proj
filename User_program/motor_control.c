#include "motor_control.h"

// 在.c文件中定义全局电机变量
MOTOR_T Motor_A, Motor_B, Motor_C, Motor_D;

void Motor_Init(void)
{
    // 初始化电机
    MOTOR_AB_Init();
    Delay_ms(5);
    MOTOR_CD_Init();
    Delay_ms(5);

    // 初始化编码器
    ENCODER_A_Init();
    ENCODER_B_Init();
    ENCODER_C_Init();
    ENCODER_D_Init();

    // 初始化目标速度
    Motor_A.TG = 2000;
    Motor_B.TG = 2000;
    Motor_C.TG = 2000;
    Motor_D.TG = 2000;
}

void Motor_Control_Task(void)
{
    // 通过编码器获取车轮实时转速m/s
    while(1){
    Motor_A.RT = (float)((int16_t)ENCODER_A_GetCounter()*420);
    ENCODER_A_SetCounter(0);
    Motor_B.RT = (float)((int16_t)ENCODER_B_GetCounter()*420);
    ENCODER_B_SetCounter(0);
    Motor_C.RT = -(float)((int16_t)ENCODER_C_GetCounter()*420);
    ENCODER_C_SetCounter(0);
    Motor_D.RT = (float)((int16_t)ENCODER_D_GetCounter()*420);
    ENCODER_D_SetCounter(0);

    // 利用PID算法计算电机PWM值
    Motor_A.PWM = SPEED_PidCtlA(Motor_A.TG, Motor_A.RT);
    Motor_B.PWM = SPEED_PidCtlB(Motor_B.TG, Motor_B.RT);
    Motor_C.PWM = SPEED_PidCtlC(Motor_C.TG, Motor_C.RT);
    Motor_D.PWM = SPEED_PidCtlD(Motor_D.TG, Motor_D.RT);

    // 设置电机PWM值
    MOTOR_A_SetSpeed(Motor_A.PWM);
    MOTOR_B_SetSpeed(Motor_B.PWM);
    MOTOR_C_SetSpeed(Motor_C.PWM);
    MOTOR_D_SetSpeed(Motor_D.PWM);
    }
}


