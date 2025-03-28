#include "main.h"

// 根据机器人类型，选择编译机器人运动学处理函数
#define ROBOT_TYPE ROBOT_MEC

#if (ROBOT_TYPE == ROBOT_MEC)
/**
 * @简  述  机器人运动学处理-麦克纳姆轮
 * @参  数  无
 * @返回值  无
 */
void ROBOT_Kinematics(void)
{
	if ((Vel.TG_IX == 0) && (Vel.TG_IY == 0) && (Vel.TG_IW == 0)) /* 电机停止 */
	{
		MOTOR_A_SetSpeed(0);
		MOTOR_B_SetSpeed(0);
		MOTOR_C_SetSpeed(0);
		MOTOR_D_SetSpeed(0);
		return;
	}

	// 通过编码器获取车轮实时转速m/s
	Wheel_A.RT = (float)-((int16_t)ENCODER_A_GetCounter() * MEC_WHEEL_SCALE);
	ENCODER_A_SetCounter(0);
	Wheel_B.RT = (float)((int16_t)ENCODER_B_GetCounter() * MEC_WHEEL_SCALE);
	ENCODER_B_SetCounter(0);
	Wheel_C.RT = (float)-((int16_t)ENCODER_C_GetCounter() * MEC_WHEEL_SCALE);
	ENCODER_C_SetCounter(0);
	Wheel_D.RT = (float)((int16_t)ENCODER_D_GetCounter() * MEC_WHEEL_SCALE);
	ENCODER_D_SetCounter(0);

	// 调试输出轮子转速
	// printf("@%f  %f   %f  %f\r\n",Wheel_A.RT,Wheel_B.RT,Wheel_C.RT,Wheel_D.RT);

	// 运动学正解析，由机器人轮子速度计算机器人速度
	Vel.RT_IX = ((Wheel_A.RT + Wheel_B.RT + Wheel_C.RT + Wheel_D.RT) / 4) * 1000;
	Vel.RT_IY = ((-Wheel_A.RT + Wheel_B.RT + Wheel_C.RT - Wheel_D.RT) / 4) * 1000;
	Vel.RT_IW = ((-Wheel_A.RT + Wheel_B.RT - Wheel_C.RT + Wheel_D.RT) / 4 / (MEC_WHEEL_BASE / 2 + MEC_ACLE_BASE / 2)) * 1000;

	// 机器人目标速度限制
	if (Vel.TG_IX > R_VX_LIMIT)
		Vel.TG_IX = R_VX_LIMIT;
	if (Vel.TG_IX < (-R_VX_LIMIT))
		Vel.TG_IX = (-R_VX_LIMIT);
	if (Vel.TG_IY > R_VY_LIMIT)
		Vel.TG_IY = R_VY_LIMIT;
	if (Vel.TG_IY < (-R_VY_LIMIT))
		Vel.TG_IY = (-R_VY_LIMIT);
	if (Vel.TG_IW > R_VW_LIMIT)
		Vel.TG_IW = R_VW_LIMIT;
	if (Vel.TG_IW < (-R_VW_LIMIT))
		Vel.TG_IW = (-R_VW_LIMIT);

	// 目标速度转化为浮点类型
	Vel.TG_FX = -Vel.TG_IX / 1000.0;
	Vel.TG_FY = -Vel.TG_IY / 1000.0;
	Vel.TG_FW = -Vel.TG_IW / 1000.0;

	// 运动学逆解析，由机器人目标速度计算电机轮子速度（m/s）
	Wheel_A.TG = Vel.TG_FX - Vel.TG_FY - Vel.TG_FW * (MEC_WHEEL_BASE / 2 + MEC_ACLE_BASE / 2);
	Wheel_B.TG = Vel.TG_FX + Vel.TG_FY + Vel.TG_FW * (MEC_WHEEL_BASE / 2 + MEC_ACLE_BASE / 2);
	Wheel_C.TG = Vel.TG_FX + Vel.TG_FY - Vel.TG_FW * (MEC_WHEEL_BASE / 2 + MEC_ACLE_BASE / 2);
	Wheel_D.TG = Vel.TG_FX - Vel.TG_FY + Vel.TG_FW * (MEC_WHEEL_BASE / 2 + MEC_ACLE_BASE / 2);

	// printf("@%f  %f  %f  %f \r\n",Wheel_A.TG,Wheel_B.TG,Wheel_C.TG,Wheel_D.TG);

	// 利用PID算法计算电机PWM值
	Wheel_A.PWM = SPEED_PidCtlA(Wheel_A.TG, Wheel_A.RT); // L1
	Wheel_B.PWM = SPEED_PidCtlB(Wheel_B.TG, Wheel_B.RT); // R1
	Wheel_C.PWM = SPEED_PidCtlC(Wheel_C.TG, Wheel_C.RT); // L2
	Wheel_D.PWM = SPEED_PidCtlD(Wheel_D.TG, Wheel_D.RT); // R2

	// printf("@%d  %d  %d  %d \r\n",Wheel_A.PWM,Wheel_B.PWM,Wheel_C.PWM,Wheel_D.PWM);

	// 设置电机PWM值
	MOTOR_A_SetSpeed(-Wheel_A.PWM);
	MOTOR_B_SetSpeed(Wheel_B.PWM);
	MOTOR_C_SetSpeed(-Wheel_C.PWM);
	MOTOR_D_SetSpeed(Wheel_D.PWM);
}
#endif

#if (ROBOT_TYPE == ROBOT_FWD)
/**
 * @简  述  机器人运动学处理-四轮差速
 * @参  数  无
 * @返回值  无
 */
void ROBOT_Kinematics(void)
{

	// 通过编码器获取车轮实时转速m/s
	Wheel_A.RT = (float)-((int16_t)ENCODER_A_GetCounter() * FWD_WHEEL_SCALE);
	ENCODER_A_SetCounter(0);
	Wheel_B.RT = (float)((int16_t)ENCODER_B_GetCounter() * FWD_WHEEL_SCALE);
	ENCODER_B_SetCounter(0);
	Wheel_C.RT = (float)-((int16_t)ENCODER_C_GetCounter() * FWD_WHEEL_SCALE);
	ENCODER_C_SetCounter(0);
	Wheel_D.RT = (float)((int16_t)ENCODER_D_GetCounter() * FWD_WHEEL_SCALE);
	ENCODER_D_SetCounter(0);

	// 调试输出轮子转速
	// printf("@%f  %f   \r\n",Wheel_A.RT,Wheel_B.RT);

	// 运动学正解析，由机器人轮子速度计算机器人速度
	Vel.RT_IX = ((Wheel_A.RT + Wheel_B.RT + Wheel_C.RT + Wheel_D.RT) / 4) * 1000;
	Vel.RT_IY = 0;
	Vel.RT_IW = ((-Wheel_A.RT + Wheel_B.RT - Wheel_C.RT + Wheel_D.RT) / (2 * FWD_WB_SCALE * FWD_WHEEL_BASE)) * 1000;

	// 机器人目标速度限制
	if (Vel.TG_IX > R_VX_LIMIT)
		Vel.TG_IX = R_VX_LIMIT;
	if (Vel.TG_IX < (-R_VX_LIMIT))
		Vel.TG_IX = (-R_VX_LIMIT);
	if (Vel.TG_IY > R_VY_LIMIT)
		Vel.TG_IY = R_VY_LIMIT;
	if (Vel.TG_IY < (-R_VY_LIMIT))
		Vel.TG_IY = (-R_VY_LIMIT);
	if (Vel.TG_IW > R_VW_LIMIT)
		Vel.TG_IW = R_VW_LIMIT;
	if (Vel.TG_IW < (-R_VW_LIMIT))
		Vel.TG_IW = (-R_VW_LIMIT);

	// 目标速度转化为浮点类型
	Vel.TG_FX = Vel.TG_IX / 1000.0;
	Vel.TG_FY = 0;
	Vel.TG_FW = Vel.TG_IW / 1000.0;

	// 运动学逆解析，由机器人目标速度计算电机轮子速度（m/s）
	Wheel_A.TG = Vel.TG_FX - Vel.TG_FW * (FWD_WHEEL_BASE * FWD_WB_SCALE / 2);
	Wheel_B.TG = Vel.TG_FX + Vel.TG_FW * (FWD_WHEEL_BASE * FWD_WB_SCALE / 2);
	Wheel_C.TG = Vel.TG_FX - Vel.TG_FW * (FWD_WHEEL_BASE * FWD_WB_SCALE / 2);
	Wheel_D.TG = Vel.TG_FX + Vel.TG_FW * (FWD_WHEEL_BASE * FWD_WB_SCALE / 2);

	// 利用PID算法计算电机PWM值
	Wheel_A.PWM = SPEED_PidCtlA(Wheel_A.TG, Wheel_A.RT);
	Wheel_B.PWM = SPEED_PidCtlB(Wheel_B.TG, Wheel_B.RT);
	Wheel_C.PWM = SPEED_PidCtlC(Wheel_C.TG, Wheel_C.RT);
	Wheel_D.PWM = SPEED_PidCtlD(Wheel_D.TG, Wheel_D.RT);

	if (Vel.TG_IX == 0 && Vel.TG_IW == 0)
	{
		Wheel_A.PWM = 0;
		Wheel_B.PWM = 0;
		Wheel_C.PWM = 0;
		Wheel_D.PWM = 0;
	}

	// 设置电机PWM值
	MOTOR_A_SetSpeed(Wheel_A.PWM);
	MOTOR_B_SetSpeed(Wheel_B.PWM);
	MOTOR_C_SetSpeed(-Wheel_C.PWM);
	MOTOR_D_SetSpeed(-Wheel_D.PWM);

	// 输出调试信息
	// printf("@%f  %f  %f %f  \r\n",Wheel_A.RT,Wheel_B.RT, Wheel_C.RT ,Wheel_D.RT);
	// printf("A%d B%d C%d  \r\n ",Vel.I_X, Vel.I_Y, Vel.I_W );
}
#endif

#if (ROBOT_TYPE == ROBOT_AKM)
/**
 * @简  述  机器人运动学处理-阿克曼
 * @参  数  无
 * @返回值  无
 */
void ROBOT_Kinematics(void)
{

	// 舵机角度
	int16_t servo_angle;

	// 通过编码器获取车轮实时转速m/s
	Wheel_A.RT = (float)-((int16_t)ENCODER_A_GetCounter() * AKM_WHEEL_SCALE);
	ENCODER_A_SetCounter(0);
	Wheel_B.RT = (float)((int16_t)ENCODER_B_GetCounter() * AKM_WHEEL_SCALE);
	ENCODER_B_SetCounter(0);

	// 调试输出轮子转速
	// printf("@%f  %f   \r\n",Wheel_A.RT,Wheel_B.RT);

	// 运动学正解析，由机器人轮子速度计算机器人速度
	Vel.RT_IX = ((Wheel_A.RT + Wheel_B.RT) / 2) * 1000;
	Vel.RT_IY = 0;
	Vel.RT_IW = ((-Wheel_A.RT + Wheel_B.RT) / AKM_WHEEL_BASE) * 1000;

	// 机器人目标速度限制
	if (Vel.TG_IX > R_VX_LIMIT)
		Vel.TG_IX = R_VX_LIMIT;
	if (Vel.TG_IX < (-R_VX_LIMIT))
		Vel.TG_IX = (-R_VX_LIMIT);
	if (Vel.TG_IY > R_VY_LIMIT)
		Vel.TG_IY = R_VY_LIMIT;
	if (Vel.TG_IY < (-R_VY_LIMIT))
		Vel.TG_IY = (-R_VY_LIMIT);
	if (Vel.TG_IW > R_VW_LIMIT)
		Vel.TG_IW = R_VW_LIMIT;
	if (Vel.TG_IW < (-R_VW_LIMIT))
		Vel.TG_IW = (-R_VW_LIMIT);

	// 目标速度转化为浮点类型
	Vel.TG_FX = Vel.TG_IX / 1000.0;
	Vel.TG_FY = Vel.TG_IY / 1000.0;
	Vel.TG_FW = Vel.TG_IW / 1000.0;

	if (Vel.TG_IW != 0)
	{
		// 判断机器人前进速度是否为0
		if (Vel.TG_IX != 0)
		{
			// 计算转弯半径
			RobotStr.Radius = Vel.TG_FX / Vel.TG_FW;

			// 阿克曼机器人需要设置最小转弯半径
			// 如果目标速度要求的转弯半径小于最小转弯半径，
			// 会导致机器人运动摩擦力大大提高，严重影响控制效果

			// 转弯半径小于最小转弯
			if (RobotStr.Radius > 0 && RobotStr.Radius < AKM_TURN_R_MINI)
			{
				RobotStr.Radius = AKM_TURN_R_MINI;
			}

			else if (RobotStr.Radius < 0 && RobotStr.Radius > (-AKM_TURN_R_MINI))
			{
				RobotStr.Radius = -AKM_TURN_R_MINI;
			}

			// 计算机器人前轮转向角度,单位弧度
			RobotStr.Angle = atan(AKM_ACLE_BASE / (RobotStr.Radius));

			// 运动学逆解析，由机器人目标速度计算电机轮子速度（m/s）
			Wheel_A.TG = Vel.TG_FX * (RobotStr.Radius - 0.5 * AKM_WHEEL_BASE) / RobotStr.Radius;
			Wheel_B.TG = Vel.TG_FX * (RobotStr.Radius + 0.5 * AKM_WHEEL_BASE) / RobotStr.Radius;
		}
		else
		{
			Wheel_A.TG = 0;
			Wheel_B.TG = 0;
			RobotStr.Radius = 0;
			RobotStr.Angle = 0;
		}
	}
	else
	{
		Wheel_A.TG = Vel.TG_FX;
		Wheel_B.TG = Vel.TG_FX;
		RobotStr.Radius = 0;
		RobotStr.Angle = 0;
	}

	// 根据前轮角度计算右前轮角度
	if (RobotStr.Angle != 0)
	{
		RobotStr.RAngle = (atan(AKM_ACLE_BASE / ((AKM_ACLE_BASE / tan(RobotStr.Angle * 0.01745)) + 0.5 * AKM_WHEEL_BASE))) * (180.0 / PI);
	}
	else
	{
		RobotStr.RAngle = 0;
	}

	// 根据右前轮角度，计算舵机转向角度
	RobotStr.SAngle = -(0.0041 * RobotStr.RAngle * RobotStr.RAngle + 1.2053 * RobotStr.RAngle) * 180 / PI;

	// 根据舵机转向角度，计算舵机PWM控制量
	servo_angle = (RobotStr.SAngle * 10 + 900 + ax_servo_offset);

	// 利用PID算法计算电机PWM值
	Wheel_A.PWM = SPEED_PidCtlA(Wheel_A.TG, Wheel_A.RT);
	Wheel_B.PWM = SPEED_PidCtlB(Wheel_B.TG, Wheel_B.RT);

	// 设置电机PWM值
	MOTOR_A_SetSpeed(Wheel_A.PWM);
	MOTOR_B_SetSpeed(Wheel_B.PWM);

	// 设置舵机角度，S1,S2任意一个都可以
	SERVO_S1_SetAngle(servo_angle);
	SERVO_S2_SetAngle(servo_angle);
}
#endif

#if (ROBOT_TYPE == ROBOT_TWD)
/**
 * @简  述  机器人运动学处理-两轮差速
 * @参  数  无
 * @返回值  无
 */
void ROBOT_Kinematics(void)
{

	// 通过编码器获取车轮实时转速m/s
	Wheel_A.RT = (float)-((int16_t)ENCODER_A_GetCounter() * MEC_WHEEL_SCALE);
	ENCODER_A_SetCounter(0);
	Wheel_B.RT = (float)((int16_t)ENCODER_B_GetCounter() * MEC_WHEEL_SCALE);
	ENCODER_B_SetCounter(0);

	// 调试输出轮子转速
	// printf("@%f  %f   \r\n",Wheel_A.RT,Wheel_B.RT);

	// 运动学正解析，由机器人轮子速度计算机器人速度
	Vel.RT_IX = ((Wheel_A.RT + Wheel_B.RT) / 2) * 1000;
	Vel.RT_IY = 0;
	Vel.RT_IW = ((-Wheel_A.RT + Wheel_B.RT) / TWD_WHEEL_BASE) * 1000;

	// 机器人目标速度限制
	if (Vel.TG_IX > R_VX_LIMIT)
		Vel.TG_IX = R_VX_LIMIT;
	if (Vel.TG_IX < (-R_VX_LIMIT))
		Vel.TG_IX = (-R_VX_LIMIT);
	if (Vel.TG_IY > R_VY_LIMIT)
		Vel.TG_IY = R_VY_LIMIT;
	if (Vel.TG_IY < (-R_VY_LIMIT))
		Vel.TG_IY = (-R_VY_LIMIT);
	if (Vel.TG_IW > R_VW_LIMIT)
		Vel.TG_IW = R_VW_LIMIT;
	if (Vel.TG_IW < (-R_VW_LIMIT))
		Vel.TG_IW = (-R_VW_LIMIT);

	// 目标速度转化为浮点类型
	Vel.TG_FX = Vel.TG_IX / 1000.0;
	Vel.TG_FY = 0;
	Vel.TG_FW = Vel.TG_IW / 1000.0;

	// 运动学逆解析，由机器人目标速度计算电机轮子速度（m/s）
	Wheel_A.TG = Vel.TG_FX - Vel.TG_FW * (TWD_WHEEL_BASE / 2);
	Wheel_B.TG = Vel.TG_FX + Vel.TG_FW * (TWD_WHEEL_BASE / 2);

	// 利用PID算法计算电机PWM值
	Wheel_A.PWM = SPEED_PidCtlA(Wheel_A.TG, Wheel_A.RT);
	Wheel_B.PWM = SPEED_PidCtlB(Wheel_B.TG, Wheel_B.RT);

	// 设置电机PWM值
	MOTOR_A_SetSpeed(Wheel_A.PWM);
	MOTOR_B_SetSpeed(Wheel_B.PWM);

	// printf("A%f B%f  \r\n ",MOTOR_A.Wheel_RT, MOTOR_B.Wheel_RT  );
	// printf("A%d B%d C%d  \r\n ",Vel.I_X, Vel.I_Y, Vel.I_W );
}
#endif

/**
 * @简  述  机器人停止运动状态
 * @参  数  无
 * @返回值  无
 */
void ROBOT_Stop(void)
{

	// 设置电机PWM值
	MOTOR_A_SetSpeed(0);
	MOTOR_B_SetSpeed(0);
	MOTOR_C_SetSpeed(0);
	MOTOR_D_SetSpeed(0);
}

#define pi 3.1415926f

kinematics_t kinematics;

/*
	设置四个关节的长度
	单位1mm
*/

void setup_kinematics(float L0, float L1, float L2, float L3, kinematics_t *kinematics) {
	//放大10倍
	kinematics->L0 = L0*10;
	kinematics->L1 = L1*10;
	kinematics->L2 = L2*10;
	kinematics->L3 = L3*10;
}

/*
	x,y 为映射到平面的坐标
	z为距离地面的距离
	Alpha 为爪子和平面的夹角 -25~-65范围比较好
*/

int kinematics_analysis(float x, float y, float z, float Alpha, kinematics_t *kinematics) {
	float theta3, theta4, theta5, theta6;
	float l0, l1, l2, l3;
	float aaa, bbb, ccc, zf_flag;
	
	//放大10倍
	x = x*10;
	y = y*10;
	z = z*10;
	
	
	l0 = kinematics->L0;
	l1 = kinematics->L1;
	l2 = kinematics->L2;
	l3 = kinematics->L3;
	
	if(x == 0) {
		theta6 = 0.0;
	} else {
		theta6 = atan(x/y)*270.0f/pi;
	}
	
	y = sqrt(x*x + y*y);    
    y = y-l3 * cos(Alpha*pi/180.0f);  
    z = z-l0-l3*sin(Alpha*pi/180.0f); 
    if(z < -l0) {
        return 1;
	}
    if(sqrt(y*y + z*z) > (l1+l2)) {
        return 2;
	}
	
	ccc = acos(y / sqrt(y * y + z * z));
    bbb = (y*y+z*z+l1*l1-l2*l2)/(2*l1*sqrt(y*y+z*z));
    if(bbb > 1 || bbb < -1) {
        return 5;
	}
    if (z < 0) {
        zf_flag = -1;
	} else {
        zf_flag = 1;
	}
    theta5 = ccc * zf_flag + acos(bbb);
    theta5 = theta5 * 180.0f / pi;
    if(theta5 > 180.0f || theta5 < 0.0f) {
        return 6;
	}
	
    aaa = -(y*y+z*z-l1*l1-l2*l2)/(2*l1*l2);
    if (aaa > 1 || aaa < -1) {
        return 3;
	}
    theta4 = acos(aaa); 
    theta4 = 180.0f - theta4 * 180.0f / pi ;  
    if (theta4 > 135.0f || theta4 < -135.0f) {
        return 4;
	}

    theta3 = Alpha - theta5 + theta4;
    if(theta3 > 90.0f || theta3 < -90.0f) {
        return 7;
	}
	
	kinematics->servo_angle[0] = theta6;
	kinematics->servo_angle[1] = theta5-90;
	kinematics->servo_angle[2] = theta4;
	kinematics->servo_angle[3] = theta3;    
	
	kinematics->servo_pwm[0] = (int)(1500-2000.0f * kinematics->servo_angle[0] / 270.0f);
	kinematics->servo_pwm[1] = (int)(1500+2000.0f * kinematics->servo_angle[1] / 270.0f);
	kinematics->servo_pwm[2] = (int)(1500+2000.0f * kinematics->servo_angle[2] / 270.0f);
	kinematics->servo_pwm[3] = (int)(1500-2000.0f * kinematics->servo_angle[3] / 270.0f);

	return 0;
}
