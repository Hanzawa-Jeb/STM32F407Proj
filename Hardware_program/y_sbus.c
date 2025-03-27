

#include "y_sbus.h"
#include <stdio.h>

#include "robot.h"

static uint8_t  uart_sbus_rx_con=0;       //���ռ�����
static uint8_t  uart_sbus_rx_buf[40];     //���ջ��壬��������С�ڵ���32Byte
static uint16_t sbus_buf[10];             //ͨ������

/**
  * @��  ��  SBUS���ڳ�ʼ��
  * @��  ��  ��
  * @����ֵ	 ��
  */
void SBUS_Init(void)
{

	GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef  NVIC_InitStructure;

	/* ����USART���� */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOD,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5,ENABLE);
	
	//USART��Ӧ���Ÿ���ӳ��
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource2, GPIO_AF_UART5); 

	//USART �˿�����	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; 
	GPIO_Init(GPIOD,&GPIO_InitStructure); 

	//USART��������
	USART_InitStructure.USART_BaudRate = 100000;    //������
	USART_InitStructure.USART_WordLength = USART_WordLength_9b;
	USART_InitStructure.USART_StopBits = USART_StopBits_2;
	USART_InitStructure.USART_Parity = USART_Parity_Even;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx;
	USART_Init(UART5, &USART_InitStructure);
	
    //USART3 NVIC ����
  NVIC_InitStructure.NVIC_IRQChannel = UART5_IRQn;//����1�ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;//��ռ���ȼ�
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =1;		//�����ȼ�
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���	
	
	//�������ڽ����ж�
	USART_ITConfig(UART5, USART_IT_RXNE, ENABLE);//��������ж�
	USART_ITConfig(UART5, USART_IT_IDLE, ENABLE);
	
		//USARTʹ��
	USART_Cmd(UART5, ENABLE); 
	
}

/**
  * @��  ��  UART �����жϷ�����
  * @��  ��  �� 
  * @����ֵ  ��
  */
void UART5_IRQHandler(void)
{
	uint8_t Res;
	
	//�����ж�
	if(USART_GetITStatus(UART5, USART_IT_RXNE) != RESET)  
	{
		Res =USART_ReceiveData(UART5);	
		
		uart_sbus_rx_buf[uart_sbus_rx_con++] = Res;
		
		USART_ITConfig(UART5, USART_IT_RXNE, ENABLE);
	}
	
    //���߿����ն�
	else if(USART_GetITStatus(UART5, USART_IT_IDLE) != RESET)
	{
		//������ɣ��ָ���ʼ״̬
		uart_sbus_rx_con = 0;
		
		//ת������Ӧͨ��
		sbus_buf[1] = ((int16_t)uart_sbus_rx_buf[ 1] >> 0 | ((int16_t)uart_sbus_rx_buf[ 2] << 8 )) & 0x07FF;
		sbus_buf[2] = ((int16_t)uart_sbus_rx_buf[ 2] >> 3 | ((int16_t)uart_sbus_rx_buf[ 3] << 5 )) & 0x07FF;
		sbus_buf[3] = ((int16_t)uart_sbus_rx_buf[ 3] >> 6 | ((int16_t)uart_sbus_rx_buf[ 4] << 2 )  | (int16_t)uart_sbus_rx_buf[ 5] << 10 ) & 0x07FF;
		sbus_buf[4] = ((int16_t)uart_sbus_rx_buf[ 5] >> 1 | ((int16_t)uart_sbus_rx_buf[ 6] << 7 )) & 0x07FF;
		sbus_buf[5] = ((int16_t)uart_sbus_rx_buf[ 6] >> 4 | ((int16_t)uart_sbus_rx_buf[ 7] << 4 )) & 0x07FF;
		sbus_buf[6] = ((int16_t)uart_sbus_rx_buf[ 7] >> 7 | ((int16_t)uart_sbus_rx_buf[ 8] << 1 )  | (int16_t)uart_sbus_rx_buf[9] <<  9 ) & 0x07FF;
		sbus_buf[7] = ((int16_t)uart_sbus_rx_buf[9] >> 2 | ((int16_t)uart_sbus_rx_buf[10] << 6 )) & 0x07FF;
		sbus_buf[8] = ((int16_t)uart_sbus_rx_buf[10] >> 5 | ((int16_t)uart_sbus_rx_buf[11] << 3 )) & 0x07FF;
		
		//�����߼���ͨ��5���˵��·���Ч
		if(sbus_buf[5] > 1500)
		{
			//�˶�����
			Vel.TG_IX = (int16_t)(  1.0 * (sbus_buf[2] - 992));
			Vel.TG_IY = (int16_t)( -0.8 * (sbus_buf[1] - 992));
			
			//����ǰ�����������
			#if (ROBOT_TYPE == ROBOT_AKM)
				ax_akm_angle = (int16_t)( -0.6 * (sbus_buf[4] - 992));
			#else
				Vel.TG_IW = (int16_t)( -5.0 * (sbus_buf[4] - 992));
			#endif
		}
		
		USART_ITConfig(UART5, USART_IT_IDLE, ENABLE);
	}
}

/******************* (C) ��Ȩ 2022 XTARK **************************************/













