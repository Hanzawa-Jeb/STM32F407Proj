// ############################################################
//  FILE:  Task_manager.c
//  Created on: 2021��8��14��
//  Author: lee
//  summary: Task_manager
// ############################################################

#include "main.h"
#include "stdio.h"
#include "usbh_usr.h"
#include "motor_control.h"  // �������



#define Task_Num 6
#define MOTOR_COUNT 500

#define HFPeriod_COUNT 1	// 5ms
#define FaulPeriod_COUNT 10 // 10ms
#define Robot_COUNT 5		// 10ms
#define KEY_COUNT 80		// 10ms
#define LEDPeriod_COUNT 6 // 500ms
#define filter_N 12

extern __ALIGN_BEGIN USB_OTG_CORE_HANDLE USB_OTG_Core_dev __ALIGN_END;
extern __ALIGN_BEGIN USBH_HOST USB_Host __ALIGN_END;

TaskTime TasksPare[Task_Num];
float dir_anglecmd = 135.0;

void Timer_Task_Count(void) // 1ms���жϼ���
{
	u16 Task_Count = 0;

	for (Task_Count = 0; Task_Count < Task_Num; Task_Count++) // TASK_NUM=5
	{
		if ((TasksPare[Task_Count].Task_Count < TasksPare[Task_Count].Task_Period) && (TasksPare[Task_Count].Task_Period > 0))
		{
			TasksPare[Task_Count].Task_Count++; // ���� �¼��������
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
			TasksPare[Task_Count].Task_Function(); // ���м�����ʱ��������
			TasksPare[Task_Count].Task_Count = 0;
		}
	}
}

void HFPeriod_1msTask(void) // ����
{
	static u16 cmd_err=0; //��ʱָ������������¼ָ���Ƿ����
	
	app_sensor_run();
	
	USBH_Process(&USB_OTG_Core_dev, &USB_Host);
	app_ps2();

	if (uart1_get_ok) /* ����ָ�� */
	{
		//printf("\r\n  app_uart_run %d= %s \r\n",uart1_get_ok, uart_receive_buf);
		if (uart1_mode == 1)
		{
			// ����ģʽ
			parse_cmd(uart_receive_buf);
		}
		else if (uart1_mode == 2)
		{
			// �����������
			parse_action(uart_receive_buf);
		}
		else if (uart1_mode == 3)
		{
			// ��·�������
			parse_action(uart_receive_buf);
		}
		else if (uart1_mode == 4)
		{
			// �洢ģʽ
			save_action(uart_receive_buf);
		}
		uart1_mode = 0;
		uart1_get_ok = 0;
		//printf("\r\n  app_uart_run %d= %s \r\n",uart1_get_ok, uart_receive_buf);
	}
	
	/* ָ������� */
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

void Robot_Control(void) // �����˿������񣬺���������
{
	MPU_Get_Gyroscope(imu_gyro_data);
	MPU_Get_Accelerometer(imu_acc_data);
//	printf("mpu gy= %d   %d   %d     ac= %d    %d    %d \r\n", imu_gyro_data[0], imu_gyro_data[1], imu_gyro_data[2],
//		   imu_acc_data[0], imu_acc_data[1], imu_acc_data[2]);
	Robot_Task();
}

void KEY_RUN(void) // �ⲿ������������
{
}

void Task_LED(void) // ��Сϵͳ500ms��LED����˸
{
	GPIO_ToggleBits(GPIOB, GPIO_Pin_6); //  500ms��LED����˸
}

void Task_Manage_List_Init(void)
{
	TasksPare[0].Task_Period = HFPeriod_COUNT; // PERIOD_COUNT=5    5ms
	TasksPare[0].Task_Count = 1;			   // Task_Count�ĳ�ֵ��һ��������500ms��ʱ����������ִ��һ�顣
	TasksPare[0].Task_Function = HFPeriod_1msTask;

	TasksPare[1].Task_Period = FaulPeriod_COUNT; // 10ms
	TasksPare[1].Task_Count = 8;
	TasksPare[1].Task_Function = task_send_Rece; //

	TasksPare[2].Task_Period = Robot_COUNT; // 20ms
	TasksPare[2].Task_Count = 1;
	TasksPare[2].Task_Function = Robot_Control; //

	TasksPare[3].Task_Period = KEY_COUNT; // 100ms
	TasksPare[3].Task_Count = 8;
	TasksPare[3].Task_Function = KEY_RUN; //

	TasksPare[4].Task_Period = LEDPeriod_COUNT; // 500ms
	TasksPare[4].Task_Count = 30;
	TasksPare[4].Task_Function = Task_LED; // 500ms��LED����˸

    // ��ӵ����������
    TasksPare[5].Task_Period = MOTOR_COUNT;    // 5ms����
    TasksPare[5].Task_Count = 20;               // ����������ִ��ʱ��
    TasksPare[5].Task_Function = Motor_Control_Task;
}
// USER CODE END

