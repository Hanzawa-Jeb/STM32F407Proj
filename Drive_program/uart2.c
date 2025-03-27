#include "main.h"

/**
 * @��  ��  UART   ���ڳ�ʼ��
 * @��  ��  baud�� ����������
 * @����ֵ	 ��
 */

// �û��ӿڣ�Ҳ��ֱ�ӽ���ݮ�ɵ�GPIO
void UART2_Init(uint32_t baud)
{

	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	/* ����USART���� */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

	// USART��Ӧ���Ÿ���ӳ��
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource5, GPIO_AF_USART2);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource6, GPIO_AF_USART2);

	// USART �˿�����
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	// USART��������
	USART_InitStructure.USART_BaudRate = baud; // ������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART2, &USART_InitStructure);

	// USARTʹ��
	USART_Cmd(USART2, ENABLE);

	// �������ڽ����ж�
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE); // ��������ж�

	// USART2 NVIC ����
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;		  // ����1�ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; // ��ռ���ȼ�
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;		  // �����ȼ�
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			  // IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);							  // ����ָ���Ĳ�����ʼ��VIC�Ĵ���
}

/***********************************************
	���ܽ��ܣ�	����2�����ֽ�
	����������	dat ���͵��ֽ�
	����ֵ��		��
 ***********************************************/
void uart2_send_byte(u8 dat)
{
	USART_SendData(USART2, dat);
	while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET)
		;
}

/***********************************************
	���ܽ��ܣ�	����2�����ַ���
	����������	*s ���͵��ַ���
	����ֵ��		��
 ***********************************************/
void uart2_send_str(u8 *s)
{
	while (*s)
	{
		uart2_send_byte(*s++);
	}
}

/**
 * @��  ��  UART �����жϷ�����
 * @��  ��  ��
 * @����ֵ  ��
 */
void USART2_IRQHandler(void)
{
	u8 sbuf_bak;
	static u16 buf_index = 0;

	if (USART_GetFlagStatus(USART2, USART_IT_RXNE) == SET)
	{
		USART_ClearITPendingBit(USART2, USART_IT_RXNE);
		sbuf_bak = USART_ReceiveData(USART2);
		//USART_SendData(USART1, sbuf_bak);
		if (uart1_get_ok)
			return;
		if (sbuf_bak == '<')
		{
			uart1_mode = 4;
			buf_index = 0;
		}
		else if (uart1_mode == 0)
		{
			if (sbuf_bak == '$')
			{
				uart1_mode = 1;
			}
			else if (sbuf_bak == '#')
			{
				uart1_mode = 2;
			}
			else if (sbuf_bak == '{')
			{
				uart1_mode = 3;
			}
			else if (sbuf_bak == '<')
			{
				uart1_mode = 4;
			}
			buf_index = 0;
		}

		uart_receive_buf[buf_index++] = sbuf_bak;

		if ((uart1_mode == 4) && (sbuf_bak == '>'))
		{
			uart_receive_buf[buf_index] = '\0';
			uart1_get_ok = 1;
		}
		else if ((uart1_mode == 1) && (sbuf_bak == '!'))
		{
			uart_receive_buf[buf_index] = '\0';
			uart1_get_ok = 1;
		}
		else if ((uart1_mode == 2) && (sbuf_bak == '!'))
		{
			uart_receive_buf[buf_index] = '\0';
			uart1_get_ok = 1;
		}
		else if ((uart1_mode == 3) && (sbuf_bak == '}'))
		{
			uart_receive_buf[buf_index] = '\0';
			uart1_get_ok = 1;
		}

		if (buf_index >= 1024)
			buf_index = 0;
	}
}
