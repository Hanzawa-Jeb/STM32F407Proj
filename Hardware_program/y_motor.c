#include "y_motor.h"

/**
 * @��  ��  ���PID���ƺ���
 * @��  ��  spd_target:�������ٶ�Ŀ��ֵ ,��Χ����250��
 *          spd_current: �������ٶȵ�ǰֵ
 * @����ֵ  ���PWM�ٶ�
 */
int16_t SPEED_PidCtlA(float spd_target, float spd_current)
{
	static int16_t motor_pwm_out;
	static float bias, bias_last;

	// ���ƫ��ֵ
	bias = spd_target - spd_current;

	// PID���������PWMֵ
	motor_pwm_out += motor_kp * bias + motor_kd * (bias - bias_last);

	// ��¼�ϴ�ƫ��
	bias_last = bias;

	// ����������
	if (motor_pwm_out > 4200)
		motor_pwm_out = 4200;
	if (motor_pwm_out < -4200)
		motor_pwm_out = -4200;

	// ����PWM����ֵ
	return motor_pwm_out;
}

/**
 * @��  ��  ���PID���ƺ���
 * @��  ��  spd_target:�������ٶ�Ŀ��ֵ
 *          spd_target: �������ٶȵ�ǰֵ
 * @����ֵ  ���PWM�ٶ�
 */
int16_t SPEED_PidCtlB(float spd_target, float spd_current)
{
	static int16_t motor_pwm_out;
	static float bias, bias_last;

	// ���ƫ��ֵ
	bias = spd_target - spd_current;

	// PID���������PWMֵ
	motor_pwm_out += motor_kp * bias + motor_kd * (bias - bias_last);

	// ��¼�ϴ�ƫ��
	bias_last = bias;

	// ����������
	if (motor_pwm_out > 4200)
		motor_pwm_out = 4200;
	if (motor_pwm_out < -4200)
		motor_pwm_out = -4200;
	
	//printf("@%d  ",motor_pwm_out);

	// ����PWM����ֵ
	return motor_pwm_out;
}

/**
 * @��  ��  ���PID���ƺ���
 * @��  ��  spd_target:�������ٶ�Ŀ��ֵ
 *          spd_target: �������ٶȵ�ǰֵ
 * @����ֵ  ���PWM�ٶ�
 */
int16_t SPEED_PidCtlC(float spd_target, float spd_current)
{
	static int16_t motor_pwm_out;
	static float bias, bias_last;

	// ���ƫ��ֵ
	bias = spd_target - spd_current;

	// PID���������PWMֵ
	motor_pwm_out += motor_kp * bias + motor_kd * (bias - bias_last);

	// ��¼�ϴ�ƫ��
	bias_last = bias;

	// ����������
	if (motor_pwm_out > 4200)
		motor_pwm_out = 4200;
	if (motor_pwm_out < -4200)
		motor_pwm_out = -4200;

	// ����PWM����ֵ
	return motor_pwm_out;
}

/**
 * @��  ��  ���PID���ƺ���
 * @��  ��  spd_target:�������ٶ�Ŀ��ֵ
 *          spd_target: �������ٶȵ�ǰֵ
 * @����ֵ  ���PWM�ٶ�
 */
int16_t SPEED_PidCtlD(float spd_target, float spd_current)
{
	static int16_t motor_pwm_out;
	static float bias, bias_last;

	// ���ƫ��ֵ
	bias = spd_target - spd_current;

	// PID���������PWMֵ
	motor_pwm_out += motor_kp * bias + motor_kd * (bias - bias_last);

	// ��¼�ϴ�ƫ��
	bias_last = bias;

	// ����������
	if (motor_pwm_out > 4200)
		motor_pwm_out = 4200;
	if (motor_pwm_out < -4200)
		motor_pwm_out = -4200;

	// ����PWM����ֵ
	return motor_pwm_out;
}

/**
 * @��  �� ���PWM�ٶȿ���
 * @��  �� speed ���ת����ֵ����Χ-4200~4200
 * @����ֵ ��
 */
void MOTOR_A_SetSpeed(int16_t speed)
{
	int16_t temp;

	temp = speed;

	if (temp > 4200)
		temp = 4200;
	if (temp < -4200)
		temp = -4200;

	if (temp > 0)
	{
		TIM_SetCompare1(TIM1, 4200);
		TIM_SetCompare2(TIM1, (4200 - temp));
	}
	else
	{
		TIM_SetCompare2(TIM1, 4200);
		TIM_SetCompare1(TIM1, (4200 + temp));
	}
}

/**
 * @��  �� ���PWM�ٶȿ���
 * @��  �� speed ���ת����ֵ����Χ-4200~4200
 * @����ֵ ��
 */
void MOTOR_B_SetSpeed(int16_t speed)
{
	int16_t temp;

	temp = speed;

	if (temp > 4200)
		temp = 4200;
	if (temp < -4200)
		temp = -4200;

	if (temp > 0)
	{
		TIM_SetCompare3(TIM1, 4200);
		TIM_SetCompare4(TIM1, (4200 - temp));
	}
	else
	{
		TIM_SetCompare4(TIM1, 4200);
		TIM_SetCompare3(TIM1, (4200 + temp));
	}
}

/**
 * @��  �� ���PWM�ٶȿ���
 * @��  �� speed ���ת����ֵ����Χ-4200~4200
 * @����ֵ ��
 */
void MOTOR_D_SetSpeed(int16_t speed)
{
	int16_t temp;

	temp = speed;

	if (temp > 4200)
		temp = 4200;
	if (temp < -4200)
		temp = -4200;

	if (temp > 0)
	{
		TIM_SetCompare1(TIM9, 4200);
		TIM_SetCompare2(TIM9, (4200 - temp));
	}
	else
	{
		TIM_SetCompare2(TIM9, 4200);
		TIM_SetCompare1(TIM9, (4200 + temp));
	}
}

/**
 * @��  �� ���PWM�ٶȿ���
 * @��  �� speed ���ת����ֵ����Χ-4200~4200
 * @����ֵ ��
 */
void MOTOR_C_SetSpeed(int16_t speed)
{
	int16_t temp;

	temp = speed;

	if (temp > 4200)
		temp = 4200;
	if (temp < -4200)
		temp = -4200;

	if (temp > 0)
	{
		TIM_SetCompare1(TIM12, 4200);
		TIM_SetCompare2(TIM12, (4200 - temp));
	}
	else
	{
		TIM_SetCompare2(TIM12, 4200);
		TIM_SetCompare1(TIM12, (4200 + temp));
	}
}
