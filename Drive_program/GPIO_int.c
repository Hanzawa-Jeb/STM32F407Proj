#include "main.h"

void MOTOR_AB_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure; 

    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure; 
	
	//ʱ��ʹ��
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);		
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);	

	//���ù�������
	GPIO_PinAFConfig(GPIOE,GPIO_PinSource9,GPIO_AF_TIM1); 
	GPIO_PinAFConfig(GPIOE,GPIO_PinSource11,GPIO_AF_TIM1); 
	GPIO_PinAFConfig(GPIOE,GPIO_PinSource13,GPIO_AF_TIM1); 
	GPIO_PinAFConfig(GPIOE,GPIO_PinSource14,GPIO_AF_TIM1); 
	
	//����IO��Ϊ���ù���-��ʱ��ͨ��
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_9 | GPIO_Pin_11 | GPIO_Pin_13 | GPIO_Pin_14;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//�ٶ�100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //����
    GPIO_Init(GPIOE, &GPIO_InitStructure);

	//��ʱ������
	TIM_TimeBaseStructure.TIM_Period=4200-1;   //�Զ���װ��ֵ	
	TIM_TimeBaseStructure.TIM_Prescaler=1;     //��ʱ����Ƶ
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //���ϼ���ģʽ
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

    //PWM1 Mode configuration: Channel1 
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0;	    //ռ�ձȳ�ʼ��
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	
    TIM_OC1Init(TIM1, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);

    //PWM1 Mode configuration: Channel2
    TIM_OC2Init(TIM1, &TIM_OCInitStructure);
    TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);

    //PWM1 Mode configuration: Channel3
    TIM_OC3Init(TIM1, &TIM_OCInitStructure);
    TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);

    //PWM1 Mode configuration: Channel4
    TIM_OC4Init(TIM1, &TIM_OCInitStructure);
    TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);

    TIM_ARRPreloadConfig(TIM1, ENABLE);

    //ʹ�ܶ�ʱ��
    TIM_Cmd(TIM1, ENABLE);   
	
	//ʹ��MOEλ
	TIM_CtrlPWMOutputs(TIM1,ENABLE);

}

void MOTOR_CD_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;

	// ʱ��ʹ��
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9, ENABLE);

	// ���ù�������
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource5, GPIO_AF_TIM9);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource6, GPIO_AF_TIM9);

	// ����IO��Ϊ���ù���-��ʱ��ͨ��
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;	   // ���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; // �ٶ�100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	   // ���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;	   // ����
	GPIO_Init(GPIOE, &GPIO_InitStructure);

	// ��ʱ������
	TIM_TimeBaseStructure.TIM_Period = 4200 - 1;				// �Զ���װ��ֵ
	TIM_TimeBaseStructure.TIM_Prescaler = 1;					// ��ʱ����Ƶ
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; // ���ϼ���ģʽ
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInit(TIM9, &TIM_TimeBaseStructure);

	// PWM1 Mode configuration: Channel1
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0; // ռ�ձȳ�ʼ��
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

	TIM_OC1Init(TIM9, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM9, TIM_OCPreload_Enable);

	// PWM1 Mode configuration: Channel2
	TIM_OC2Init(TIM9, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM9, TIM_OCPreload_Enable);

	TIM_ARRPreloadConfig(TIM9, ENABLE);

	// ʹ�ܶ�ʱ��
	TIM_Cmd(TIM9, ENABLE);

	// ʱ��ʹ��
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM12, ENABLE);

	// ���ù�������
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource14, GPIO_AF_TIM12); /*����*/
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource15, GPIO_AF_TIM12); /*����*/

	// ����IO��Ϊ���ù���-��ʱ��ͨ��
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;	   // ���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; // �ٶ�100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	   // ���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;	   // ����
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	// ��ʱ������
	TIM_TimeBaseStructure.TIM_Period = 4200 - 1;				// �Զ���װ��ֵ
	TIM_TimeBaseStructure.TIM_Prescaler = 0;					// ��ʱ����Ƶ
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; // ���ϼ���ģʽ
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInit(TIM12, &TIM_TimeBaseStructure);

	// PWM1 Mode configuration: Channel1
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0; // ռ�ձȳ�ʼ��
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

	TIM_OC1Init(TIM12, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM12, TIM_OCPreload_Enable);

	// PWM1 Mode configuration: Channel2
	TIM_OC2Init(TIM12, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM12, TIM_OCPreload_Enable);

	TIM_ARRPreloadConfig(TIM12, ENABLE);
	// ʹ�ܶ�ʱ��
	TIM_Cmd(TIM12, ENABLE);
}

void ENCODER_A_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_ICInitTypeDef TIM_ICInitStructure;

	// ʱ��ʹ��
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

	// ���ù�������
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource15, GPIO_AF_TIM2);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource3, GPIO_AF_TIM2);

	// ����IO��Ϊ���ù���-��ʱ��ͨ��
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;	   // ���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; // �ٶ�100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	   // ���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;	   // ����
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	// ����IO��Ϊ���ù���-��ʱ��ͨ��
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;	   // ���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; // �ٶ�100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	   // ���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;	   // ����
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	// Timer configuration in Encoder mode
	TIM_DeInit(TIM2);
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);

	TIM_TimeBaseStructure.TIM_Prescaler = 0x0; // No prescaling
	TIM_TimeBaseStructure.TIM_Period = 65535;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

	TIM_EncoderInterfaceConfig(TIM2, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
	TIM_ICStructInit(&TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_ICFilter = 6;
	TIM_ICInit(TIM2, &TIM_ICInitStructure);

	// Reset counter
	TIM2->CNT = 0;

	TIM_Cmd(TIM2, ENABLE);
}

/**
 * @��  ��  ��������ȡ��������ֵ
 * @��  ��  ��
 * @����ֵ  ��������ǰֵ
 */
uint16_t ENCODER_A_GetCounter(void)
{
	return (short)TIM2->CNT;
}

/**
 * @��  ��  ���������ü�������ֵ
 * @��  ��  count  ��������ֵ
 * @����ֵ  ��
 */
void ENCODER_A_SetCounter(uint16_t count)
{
	TIM2->CNT = count;
}

void ENCODER_B_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_ICInitTypeDef TIM_ICInitStructure;

	// ʱ��ʹ��
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

	// ���ù�������
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource4, GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource5, GPIO_AF_TIM3);

	// ����IO��Ϊ���ù���-��ʱ��ͨ��
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;	   // ���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; // �ٶ�100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	   // ���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;	   // ����
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	// Timer configuration in Encoder mode
	TIM_DeInit(TIM3);
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);

	TIM_TimeBaseStructure.TIM_Prescaler = 0x0; // No prescaling
	TIM_TimeBaseStructure.TIM_Period = 65535;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

	TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
	TIM_ICStructInit(&TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_ICFilter = 6;
	TIM_ICInit(TIM3, &TIM_ICInitStructure);
	// Reset counter
	TIM3->CNT = 0;

	TIM_Cmd(TIM3, ENABLE);
}

/**
 * @��  ��  ��������ȡ��������ֵ
 * @��  ��  ��
 * @����ֵ  ��������ǰֵ
 */
uint16_t ENCODER_B_GetCounter(void)
{
	return (short)TIM3->CNT;
}

/**
 * @��  ��  ���������ü�������ֵ
 * @��  ��  count  ��������ֵ
 * @����ֵ  ��
 */
void ENCODER_B_SetCounter(uint16_t count)
{
	TIM3->CNT = count;
}



/**
 * @��  ��  ��������ʼ��
 * @��  ��  ��
 * @����ֵ  ��
 */
void ENCODER_C_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_ICInitTypeDef TIM_ICInitStructure;

	// ʱ��ʹ��
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

	// ���ù�������
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource12, GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource13, GPIO_AF_TIM4);

	// ����IO��Ϊ���ù���-��ʱ��ͨ��
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;	   // ���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; // �ٶ�100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	   // ���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;	   // ����
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	// Timer configuration in Encoder mode
	TIM_DeInit(TIM4);
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);

	TIM_TimeBaseStructure.TIM_Prescaler = 0x0; // No prescaling
	TIM_TimeBaseStructure.TIM_Period = 65535;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

	TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
	TIM_ICStructInit(&TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_ICFilter = 6;
	TIM_ICInit(TIM4, &TIM_ICInitStructure);

	// Reset counter
	TIM4->CNT = 0;

	TIM_Cmd(TIM4, ENABLE);
}

/**
 * @��  ��  ��������ȡ��������ֵ
 * @��  ��  ��
 * @����ֵ  ��������ǰֵ
 */
uint16_t ENCODER_C_GetCounter(void)
{
	return (short)TIM4->CNT;
}

/**
 * @��  ��  ���������ü�������ֵ
 * @��  ��  count  ��������ֵ
 * @����ֵ  ��
 */
void ENCODER_C_SetCounter(uint16_t count)
{
	TIM4->CNT = count;
}
/**
 * @��  ��  ��������ʼ��
 * @��  ��  ��
 * @����ֵ  ��
 */
void ENCODER_D_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_ICInitTypeDef TIM_ICInitStructure;

	// ʱ��ʹ��
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);

	// ���ù�������
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_TIM5);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_TIM5);

	// ����IO��Ϊ���ù���-��ʱ��ͨ��
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;	   // ���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; // �ٶ�100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	   // ���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;	   // ����
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	// Timer configuration in Encoder mode
	TIM_DeInit(TIM5);
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);

	TIM_TimeBaseStructure.TIM_Prescaler = 0x0; // No prescaling
	TIM_TimeBaseStructure.TIM_Period = 65535;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);

	TIM_EncoderInterfaceConfig(TIM5, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
	TIM_ICStructInit(&TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_ICFilter = 6;
	TIM_ICInit(TIM5, &TIM_ICInitStructure);

	// Reset counter
	TIM5->CNT = 0;

	TIM_Cmd(TIM5, ENABLE);

}

/**
 * @��  ��  ��������ȡ��������ֵ
 * @��  ��  ��
 * @����ֵ  ��������ǰֵ
 */
uint16_t ENCODER_D_GetCounter(void)
{
	return (short)TIM5->CNT;
}

/**
 * @��  ��  ���������ü�������ֵ
 * @��  ��  count  ��������ֵ
 * @����ֵ  ��
 */
void ENCODER_D_SetCounter(uint16_t count)
{
	TIM5->CNT = count;
}
/**************************************************************************
*  �������ܣ������ת����
*
*  ��ڲ�������
*
*  �� �� ֵ����
**************************************************************************/
uint32_t TIM_IsEnabledIT_UPDATE(TIM_TypeDef *TIMx)
{
  return (READ_BIT(TIMx->DIER, TIM_DIER_UIE) == (TIM_DIER_UIE));
}

uint32_t TIM_IsActiveFlag_UPDATE(TIM_TypeDef *TIMx)
{
  return (READ_BIT(TIMx->SR, TIM_SR_UIF) == (TIM_SR_UIF));
}

uint32_t TIM_GetDirection(TIM_TypeDef *TIMx)
{
  return (uint32_t)(READ_BIT(TIMx->CR1, TIM_CR1_DIR));
}

























// ������ʼ��
void KEY_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	// GPIO����
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE); // ����GPIOʱ��

	// ����ģʽѡ��S1 GPIO  ����ģʽ
	GPIO_SetBits(GPIOE, GPIO_Pin_4);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; // ����
	GPIO_Init(GPIOD, &GPIO_InitStructure);
}

// 4·����ѭ����ʼ��
/* X1=E0 X2=E2 X3=E3 X4=E4 */
void TRACK_IR4_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(TRTACK_IR4_X1_CLK | TRTACK_IR4_X2_CLK | TRTACK_IR4_X3_CLK | TRTACK_IR4_X4_CLK, ENABLE); /* ʹ�� ��� �˿�ʱ�� */

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;       
    GPIO_InitStructure.GPIO_Pin = TRTACK_IR4_X1_PIN;         
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;      /*����*/
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
    GPIO_Init(TRTACK_IR4_X1_PORT, &GPIO_InitStructure); /*��ʼ��IO*/

    GPIO_InitStructure.GPIO_Pin = TRTACK_IR4_X2_PIN;
    GPIO_Init(TRTACK_IR4_X2_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = TRTACK_IR4_X3_PIN;
    GPIO_Init(TRTACK_IR4_X3_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = TRTACK_IR4_X4_PIN;
    GPIO_Init(TRTACK_IR4_X4_PORT, &GPIO_InitStructure);
}

/***********************************************
    ultrasonic_sensor_init()
    ���ܽ��ܣ���ʼ��������������
    ������������
    ����ֵ��	��
 ***********************************************/
void ultrasonic_sensor_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM13, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

    // ��ʼ��������IO�� Trig PA3  Echo PA2
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; 
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
   // TIM_DeInit(TIM8);
    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_Prescaler = 168-1;  //168��Ƶ��Ƶ��Ϊ1M  
    TIM_TimeBaseStructure.TIM_Period = 30000;    //
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;   
    TIM_TimeBaseInit(TIM13, &TIM_TimeBaseStructure);
}

/**
 * @��������: ��ȡ����
 * @return {*}���ؾ��� cm
 * ����=�ߵ�ƽʱ��*���٣�340M/S�� /2
 */
float sensor_sr_ultrasonic_read(void)
{
    uint16_t csb_t;
    TRIG_SET(1);
    Delay_us(50);
    TRIG_SET(0);
	
	TIM_Cmd(TIM13, ENABLE);   // ʹ��TIMx����
    while (ECHO_GET() == 0) /* �ȴ����տڸߵ�ƽ��� */
        ;
    TIM_SetCounter(TIM13, 0); // �������
    
    while (ECHO_GET())
        ;
    
    csb_t = TIM13->CNT;
	TIM_Cmd(TIM13, DISABLE); // ʹ��TIMx����
    // 340m/s = 0.034cm/us
    if (csb_t < 25000)
    {
        return (float)csb_t * 0.034f;
    }
    return -1;
}

// LED ��ʼ��
void GPIO_LED_int(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;	   // ��ͨ���ģʽ
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	   // �������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; // 100MHz
	GPIO_Init(GPIOB, &GPIO_InitStructure);			   // ��ʼ��GPIO
	GPIO_SetBits(GPIOB, GPIO_Pin_6);				   // Ĭ��Ϩ��
}

void VIN_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	ADC_InitTypeDef ADC_InitStructure;

	// ʹ��ADC2��GPIOBʱ��
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2, ENABLE);

	// ����PB1Ϊģ������(ADC Channel10)
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC2, ENABLE);
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC2, DISABLE);

	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4;
	ADC_CommonInit(&ADC_CommonInitStructure);

	// ����ADC2
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfConversion = 1;
	ADC_Init(ADC2, &ADC_InitStructure);

	// ʹ��ADC2
	ADC_Cmd(ADC2, ENABLE);

	// ADC2 ����ͨ������
	ADC_RegularChannelConfig(ADC2, ADC_Channel_10, 1, ADC_SampleTime_480Cycles);
}

/**
 * @��  ��  VIN ��������ѹ
 * @��  ��  ��
 * @����ֵ  ��ѹֵ������100��������7.2V���Ϊ720��
 */
uint16_t VIN_GetVol_X100(void)
{
	uint16_t Input_Vol, temp;

	ADC_SoftwareStartConv(ADC2); // ʹ��ָ����ADC2��, ���ת����������

	while (!ADC_GetFlagStatus(ADC2, ADC_FLAG_EOC))
		; // �ȴ�ת������

	temp = ADC_GetConversionValue(ADC2);

	Input_Vol = (uint16_t)((3.3 * 11.0 * ADC_REVISE * temp) / 4095); // ��ѹ����1/11

	return Input_Vol;
}
