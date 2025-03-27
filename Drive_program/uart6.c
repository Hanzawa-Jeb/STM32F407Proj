#include "uart6.h"

/**
 * @��  ��  UART   ���ڳ�ʼ��
 * @��  ��  baud�� ����������
 * @����ֵ	 ��
 */

// �û��ӿڣ�Ҳ��ֱ�ӽ���ݮ�ɵ�GPIO
void UART6_Init(uint32_t baud)
{

	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	/* ����USART���� */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);

	// USART��Ӧ���Ÿ���ӳ��
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_USART6);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_USART6);

	// USART �˿�����
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	// USART��������
	USART_InitStructure.USART_BaudRate = baud; // ������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART6, &USART_InitStructure);

	// USARTʹ��
	USART_Cmd(USART6, ENABLE);

	// �������ڽ����ж�
	USART_ITConfig(USART6, USART_IT_RXNE, ENABLE); // ��������ж�

	// USART6 NVIC ����
	NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn;		  // ����1�ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; // ��ռ���ȼ�
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;		  // �����ȼ�
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			  // IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);							  // ����ָ���Ĳ�����ʼ��VIC�Ĵ���
}

/***********************************************
	���ܽ��ܣ�	����1�����ֽ�
	����������	dat ���͵��ֽ�
	����ֵ��		��
 ***********************************************/
void uart6_send_byte(u8 dat)
{
	USART_SendData(USART6, dat);
	while (USART_GetFlagStatus(USART6, USART_FLAG_TXE) == RESET)
		;
}

/***********************************************
	���ܽ��ܣ�	����1�����ַ���
	����������	*s ���͵��ַ���
	����ֵ��		��
 ***********************************************/
void uart6_send_str(u8 *s)
{
	while (*s)
	{
		uart6_send_byte(*s++);
	}
}

/**
 * @��  ��  UART �����жϷ�����
 * @��  ��  ��
 * @����ֵ  ��
 */
void USART6_IRQHandler(void)
{
	u8 sbuf_bak;
	static u16 buf_index = 0;
	if (USART_GetFlagStatus(USART6, USART_IT_RXNE) == SET)
	{
		USART_ClearITPendingBit(USART6, USART_IT_RXNE);
		sbuf_bak = USART_ReceiveData(USART6);
		// USART_SendData(USART1, sbuf_bak);
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
