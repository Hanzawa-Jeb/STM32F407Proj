#include "main.h"

#include "usb_bsp.h"
#include "usbh_core.h"
#include "usbh_usr.h"
#include "usbh_hid_core.h"

typedef struct
{
	double RT; // ???????,??m/s
	float TG;  // ???????,??m/s
	short PWM; // ????PWM????
} MOTOR_T;

__ALIGN_BEGIN USB_OTG_CORE_HANDLE USB_OTG_Core_dev __ALIGN_END;
__ALIGN_BEGIN USBH_HOST USB_Host __ALIGN_END;
extern HID_Machine_TypeDef HID_Machine;

int main(void)
{
	MOTOR_T Motor_A, Motor_B, Motor_C, Motor_D;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

	Delay_ms(5);
	SysTickConfig(); // 1ms��ʱ��.
	Delay_ms(5);
	GPIO_LED_int(); // LED��ʼ��
	Delay_ms(5);
	SERVO_Init();

	//	KEY_Init();
	MOTOR_AB_Init();
	Delay_ms(5);
	MOTOR_CD_Init();
	Delay_ms(5);

	VIN_Init(); /* ADC��ѹ����ʼ�� */
	Delay_ms(5);

	BEEP_Init();

	// ���Դ��ڳ�ʼ��
	UART1_Init(115200);
	Delay_ms(5);
	// TTL���ڳ�ʼ��
	UART2_Init(115200);
	Delay_ms(5);
	// USBͨ�Ŷ˿ڳ�ʼ��
	USART3_Init(115200);
	Delay_ms(5);
	UART4_Init(115200);
	Delay_ms(5);
	//UART6_Init(115200);
	//Delay_ms(5);

	ENCODER_A_Init();
	ENCODER_B_Init();
	ENCODER_C_Init();
	ENCODER_D_Init();
	setup_w25q64();

    Motor_A.TG = 2000;
    Motor_B.TG = 2000;
    Motor_C.TG = 2000;
    Motor_D.TG = 2000;
    
	// 6050��̬��ʼ��
	MPU_Init();
	Delay_ms(5);

	TIM14_Int_Init(5000 - 1, 84 - 1); // ��ʱ��ʱ��84M����Ƶϵ��8400������84M/84=1Mhz�ļ���Ƶ�ʣ�����5000��Ϊ500ms

	others_init();

	app_sensor_init(); /* ������Ӧ�� */
	
	// kinematics 100mm 105mm 88mm 155mm
    setup_kinematics(100, 105, 88, 155, &kinematics);

	OLED_Init();
	OLED_TEST();

	USBH_Init(&USB_OTG_Core_dev,
			  USB_OTG_FS_CORE_ID,
			  &USB_Host, &HID_cb, &USR_Callbacks);

	// ������ʾ��Ϣ
	beep_on_times(3, 100);

	Task_Manage_List_Init();
	while (1)
	{	
		Execute_Task_List_RUN();
	}
}
