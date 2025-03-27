#include "motor_control.h"

MOTOR_T Motor_A,Motor_B,Motor_C,Motor_D;

int main(void)
{
    MOTOR_T Motor_A,Motor_B,Motor_C,Motor_D;
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

	
	UART1_Init(115200);
	
	MOTOR_AB_Init();
	Delay_ms(5);
	MOTOR_CD_Init();
	Delay_ms(5);


	ENCODER_A_Init();
	ENCODER_B_Init();
	ENCODER_C_Init();
	ENCODER_D_Init();
	
	Motor_A.TG = 2000;
	Motor_B.TG = 2000;
	Motor_C.TG = 10000;
	Motor_D.TG = 10000;
	
	while (1)
	{
		// 通过编码器获取车轮实时转速m/s
		Motor_A.RT = (float)((int16_t)ENCODER_A_GetCounter()*420 );
		ENCODER_A_SetCounter(0);
		Motor_B.RT = (float)((int16_t)ENCODER_B_GetCounter()*420);
		ENCODER_B_SetCounter(0);
		Motor_C.RT = -(float)((int16_t)ENCODER_C_GetCounter()*420 );
		ENCODER_C_SetCounter(0);
		Motor_D.RT = (float)((int16_t)ENCODER_D_GetCounter()*420);
		ENCODER_D_SetCounter(0);
		printf("@%f  %f   %f  %f\r\n",Motor_A.RT,Motor_B.RT,Motor_C.RT,Motor_D.RT);
		//printf("@%f\n",Motor_A.RT);
		
		// 利用PID算法计算电机PWM值
		Motor_A.PWM = SPEED_PidCtlA(Motor_A.TG, Motor_A.RT); // L1
		Motor_B.PWM = SPEED_PidCtlB(Motor_B.TG, Motor_B.RT); // R1
		Motor_C.PWM = SPEED_PidCtlC(Motor_C.TG, Motor_C.RT); // L2
		Motor_D.PWM = SPEED_PidCtlD(Motor_D.TG, Motor_D.RT); // R2

		//printf("@%d  %d  %d  %d \r\n",Motor_A.PWM,Motor_B.PWM,Motor_C.PWM,Motor_D.PWM);
	
		// 设置电机PWM值
		MOTOR_A_SetSpeed(Motor_A.PWM);
		MOTOR_B_SetSpeed(Motor_B.PWM);
		MOTOR_C_SetSpeed(-Motor_C.PWM);
		MOTOR_D_SetSpeed(Motor_D.PWM);

		//Delay_ms(1);
	}
}
