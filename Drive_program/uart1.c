
#include "main.h"

/**
 * @��  ��  UART ���ڳ�ʼ��
 * @��  ��  baud�� ����������
 * @����ֵ	 ��
 */
// ����1���Զ����ص�·
void UART1_Init(uint32_t baud)
{

	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	/* ���� USART1���� */
	// ��GPIO��USART������ʱ��
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

	// USART1��Ӧ���Ÿ���ӳ��
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);

	// USART1 �˿�����
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	// USART1��������
	USART_InitStructure.USART_BaudRate = baud; // ������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART1, &USART_InitStructure);

	// ʹ�� USART�� �������
	USART_Cmd(USART1, ENABLE);

	// ��ܵ�һ���ַ����������BUG
	//	USART_ClearFlag(USART1, USART_FLAG_TC);

	// �������ڽ����ж�
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE); // ��������ж�

	// Usart1 NVIC ����
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;		  // ����1�ж�ͨ��
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
void uart1_send_byte(u8 dat)
{
	USART_SendData(USART1, dat);
	while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET)
		;
}

/***********************************************
	���ܽ��ܣ�	����1�����ַ���
	����������	*s ���͵��ַ���
	����ֵ��		��
 ***********************************************/
void uart1_send_str(u8 *s)
{
	while (*s)
	{
		uart1_send_byte(*s++);
	}
}

/* QSA */
/**
 * @��  ��  DBUART �����жϷ�����
 * @��  ��  ��
 * @����ֵ  ��
 */
void USART1_IRQHandler(void)
{
   u8 sbuf_bak;
   static u16 buf_index = 0;
   if (USART_GetFlagStatus(USART1, USART_IT_RXNE) == SET)
   {
       USART_ClearITPendingBit(USART1, USART_IT_RXNE);
       sbuf_bak = USART_ReceiveData(USART1);
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
		
	    if(uart1_get_ok==1)
	    {
			uart_receive_num = 1;
        }
		
       if (buf_index >= 1024)
           buf_index = 0;
   }
}


/**************************���ڴ�ӡ��غ����ض���********************************/
/**
 * @��  ��  �ض���putc������USART1��
 */
int fputc(int ch, FILE *f)
{
	/* Place your implementation of fputc here */
	/* e.g. write a character to the USART */
	USART_SendData(USART1, (uint8_t)ch);

	/* Loop until the end of transmission */
	while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
	{
	}

	return ch;
}

/**
 * @��  ��  �ض���getc������USART1��
 */
int fgetc(FILE *f)
{
	/* �ȴ�����1�������� */
	while (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET)
	{
	}

	return (int)USART_ReceiveData(USART1);
}

/******************* (C) ��Ȩ 2022 XTARK **************************************/
