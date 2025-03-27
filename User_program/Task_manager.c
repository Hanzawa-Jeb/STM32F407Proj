// ############################################################
//  FILE:  Task_manager.c
//  Created on: 2021年8月14日
//  Author: lee
//  summary: Task_manager
// ############################################################

#include "main.h"
#include "stdio.h"
#include "usbh_usr.h"
#include "motor_control.h"



#define Task_Num 6
#define MOTOR_COUNT 5

#define HFPeriod_COUNT 1	// 5ms
#define FaulPeriod_COUNT 10 // 10ms
#define Robot_COUNT 5		// 10ms
#define KEY_COUNT 80		// 10ms
#define LEDPeriod_COUNT 600 // 500ms
#define filter_N 12

extern __ALIGN_BEGIN USB_OTG_CORE_HANDLE USB_OTG_Core_dev __ALIGN_END;
extern __ALIGN_BEGIN USBH_HOST USB_Host __ALIGN_END;

TaskTime TasksPare[Task_Num];
float dir_anglecmd = 135.0;

void Timer_Task_Count(void) // 1ms的中断计数
{
	u16 Task_Count = 0;

	for (Task_Count = 0; Task_Count < Task_Num; Task_Count++) // TASK_NUM=5
	{
		if ((TasksPare[Task_Count].Task_Count < TasksPare[Task_Count].Task_Period) && (TasksPare[Task_Count].Task_Period > 0))
		{
			TasksPare[Task_Count].Task_Count++; // 计数 事件任务计数
		}
	}
}

void Execute_Task_List_RUN(void)
{
	uint16_t Task_Count = 0;

	for (Task_Count = 0; Task_Count < Task_Num; Task_Count++)
	{
		if ((TasksPare[Task_Count].Task_Count >= TasksPare[Task_Count].Task_Period) && (TasksPare[Task_Count].Task_Period > 0))
		{
			TasksPare[Task_Count].Task_Function(); // 运行计数的时间任务函数
			TasksPare[Task_Count].Task_Count = 0;
		}
	}
}

void HFPeriod_1msTask(void) // 备用
{
	static u16 cmd_err=0; //有时指令会出错，用来记录指令是否出错
	
	app_sensor_run();
	
	USBH_Process(&USB_OTG_Core_dev, &USB_Host);
	app_ps2();

	if (uart1_get_ok) /* 接收指令 */
	{
		//printf("\r\n  app_uart_run %d= %s \r\n",uart1_get_ok, uart_receive_buf);
		if (uart1_mode == 1)
		{
			// 命令模式
			parse_cmd(uart_receive_buf);
		}
		else if (uart1_mode == 2)
		{
			// 单个舵机调试
			parse_action(uart_receive_buf);
		}
		else if (uart1_mode == 3)
		{
			// 多路舵机调试
			parse_action(uart_receive_buf);
		}
		else if (uart1_mode == 4)
		{
			// 存储模式
			save_action(uart_receive_buf);
		}
		uart1_mode = 0;
		uart1_get_ok = 0;
		//printf("\r\n  app_uart_run %d= %s \r\n",uart1_get_ok, uart_receive_buf);
	}
	
	/* 指令出错处理 */
	if(uart1_mode!=0)
	{
		cmd_err++;
	}
	else
	{
		cmd_err=0;
	}
	
	if(cmd_err>=6000)
	{
		uart1_mode=0;
	}
}

void task_send_Rece(void) 
{
	loop_action();
}

void Robot_Control(void) // 机器人控制任务，含电量管理
{
	MPU_Get_Gyroscope(imu_gyro_data);
	MPU_Get_Accelerometer(imu_acc_data);
//	printf("mpu gy= %d   %d   %d     ac= %d    %d    %d \r\n", imu_gyro_data[0], imu_gyro_data[1], imu_gyro_data[2],
//		   imu_acc_data[0], imu_acc_data[1], imu_acc_data[2]);
	Robot_Task();
}

void KEY_RUN(void) // 外部按键处理任务
{
}

void Task_LED(void) // 最小系统500ms的LED的闪烁
{
	GPIO_ToggleBits(GPIOB, GPIO_Pin_6); //  500ms的LED的闪烁
}

void Task_Manage_List_Init(void)
{
	TasksPare[0].Task_Period = HFPeriod_COUNT; // PERIOD_COUNT=5    5ms
	TasksPare[0].Task_Count = 1;			   // Task_Count的初值不一样，避免500ms到时，所有任务都执行一遍。
	TasksPare[0].Task_Function = HFPeriod_1msTask;

	TasksPare[1].Task_Period = FaulPeriod_COUNT; // 10ms
	TasksPare[1].Task_Count = 8;
	TasksPare[1].Task_Function = task_send_Rece; //

	TasksPare[2].Task_Period = Robot_COUNT; // 20ms
	TasksPare[2].Task_Count = 15;
	TasksPare[2].Task_Function = Robot_Control; //

	TasksPare[3].Task_Period = KEY_COUNT; // 100ms
	TasksPare[3].Task_Count = 80;
	TasksPare[3].Task_Function = KEY_RUN; //

	TasksPare[4].Task_Period = LEDPeriod_COUNT; // 500ms
	TasksPare[4].Task_Count = 300;
	TasksPare[4].Task_Function = Task_LED; // 500ms的LED的闪烁

    // 添加电机控制任务
    TasksPare[5].Task_Period = MOTOR_COUNT;    // 5ms周期
    TasksPare[5].Task_Count = 2;               // 错开其他任务执行时间
    TasksPare[5].Task_Function = Motor_Control_Task;
}
// USER CODE END

void Motor_Control_Task(void)
{
    // 读取编码器数据
    Motor_A.RT = (float)((int16_t)ENCODER_A_GetCounter()*420);
    ENCODER_A_SetCounter(0);
    Motor_B.RT = (float)((int16_t)ENCODER_B_GetCounter()*420);
    ENCODER_B_SetCounter(0);
    Motor_C.RT = -(float)((int16_t)ENCODER_C_GetCounter()*420);
    ENCODER_C_SetCounter(0);
    Motor_D.RT = (float)((int16_t)ENCODER_D_GetCounter()*420);
    ENCODER_D_SetCounter(0);

    // PID控制计算
    Motor_A.PWM = SPEED_PidCtlA(Motor_A.TG, Motor_A.RT);
    Motor_B.PWM = SPEED_PidCtlB(Motor_B.TG, Motor_B.RT);
    Motor_C.PWM = SPEED_PidCtlC(Motor_C.TG, Motor_C.RT);
    Motor_D.PWM = SPEED_PidCtlD(Motor_D.TG, Motor_D.RT);

    // 更新电机PWM
    MOTOR_A_SetSpeed(Motor_A.PWM);
    MOTOR_B_SetSpeed(Motor_B.PWM);
    MOTOR_C_SetSpeed(-Motor_C.PWM);
    MOTOR_D_SetSpeed(Motor_D.PWM);
}
