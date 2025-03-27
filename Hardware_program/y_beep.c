#include "y_beep.h"

/**
  * @��  ��  ��ʼ��
  * @��  ��  ��
  * @����ֵ  ��
  */
void BEEP_Init(void) 
{
	GPIO_InitTypeDef GPIO_InitStructure;

	//GPIO���� 
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE); 
	
	//����GPIO  �������ģʽ 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	//�رշ�����
	GPIO_ResetBits(GPIOC,  GPIO_Pin_13);	

}	

/**
 * @��������: ����������ʱ�䣬��λms
 * @param {uint16_t} times
 * @return {*}
 */
void beep_on_times(int times, int delay)
{
    int i;
    for (i = 0; i < times; i++)
    {
        BEEP_On();
        Delay_ms(delay);
        BEEP_Off();
        Delay_ms(delay);
    }
}

/******************* (C) ��Ȩ 2022 XTARK **************************************/
