#include "main.h"

#include "usb_bsp.h"
#include "usbh_core.h"
#include "usbh_usr.h"
#include "usbh_hid_core.h"


__ALIGN_BEGIN USB_OTG_CORE_HANDLE USB_OTG_Core_dev __ALIGN_END;
__ALIGN_BEGIN USBH_HOST USB_Host __ALIGN_END;
extern HID_Machine_TypeDef HID_Machine;

int main(void)
{
	MOTOR_T Motor_A, Motor_B, Motor_C, Motor_D;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

	Motor_Init();
	SERVO_Init();

	//	KEY_Init();
	MOTOR_AB_Init();
	Delay_ms(5);
	MOTOR_CD_Init();
	Delay_ms(5);

	VIN_Init(); /* ADC电压检测初始化 */
	Delay_ms(5);

	BEEP_Init();

	// 调试串口初始化
	UART1_Init(115200);
	Delay_ms(5);
	// TTL串口初始化
	UART2_Init(115200);
	Delay_ms(5);
	// USB通信端口初始化
	USART3_Init(115200);
	Delay_ms(5);
	UART4_Init(115200);
	Delay_ms(5);
	//UART6_Init(115200);
	//Delay_ms(5);


	setup_w25q64();


    
	// 6050姿态初始化
	MPU_Init();
	Delay_ms(5);

	TIM14_Int_Init(5000 - 1, 84 - 1); // 定时器时钟84M，分频系数8400，所以84M/84=1Mhz的计数频率，计数5000次为500ms

	others_init();

	app_sensor_init(); /* 传感器应用 */
	
	// kinematics 100mm 105mm 88mm 155mm
    setup_kinematics(100, 105, 88, 155, &kinematics);

	OLED_Init();
	OLED_TEST();

	USBH_Init(&USB_OTG_Core_dev,
			  USB_OTG_FS_CORE_ID,
			  &USB_Host, &HID_cb, &USR_Callbacks);

	// 开机提示信息
	beep_on_times(3, 100);

	Task_Manage_List_Init();
	while (1)
	{	
		Execute_Task_List_RUN();
	}
}
