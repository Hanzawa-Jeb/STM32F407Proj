
//############################################################
// FILE:  TIM1PWM.c
// Created on: 2019��7��5��
// Author: lee
// summary: TIM1_PWM_Init
//############################################################

#include "main.h"

void TIM1_PWM_Init(void)
{	
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseInitStructure;
	TIM_OCInitTypeDef    TIM_OCInitStructure;	
  TIM_BDTRInitTypeDef    TIM1_BDTRInitStructure;
	NVIC_InitTypeDef    NVIC_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);//����TIM1ʱ��
 
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1; // ����Ƶ
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_CenterAligned1;  // �������ĶԳ�
	TIM_TimeBaseInitStructure.TIM_Prescaler = 0;   //��Ƶϵ�� Timer clock = sysclock /(TIM_Prescaler+1) = 168M
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = REP_RATE;
	TIM_TimeBaseInitStructure.TIM_Period = PWM_PERIOD;    //��������
	TIM_TimeBaseInit(TIM1,&TIM_TimeBaseInitStructure);
		
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; // pwmģʽ2
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;  //ʹ��CHx
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;//ʹ��CHxN
	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCNPolarity_High; //����Ч
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;// TIM_OCNPolarity_High 
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCNIdleState_Reset;//����ʱ��λ
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;

	TIM_OC1Init(TIM1,&TIM_OCInitStructure);

	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OC2Init(TIM1,&TIM_OCInitStructure);

	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OC3Init(TIM1,&TIM_OCInitStructure);

  TIM1_BDTRInitStructure.TIM_OSSRState = TIM_OSSRState_Enable;
  TIM1_BDTRInitStructure.TIM_OSSIState = TIM_OSSIState_Enable;
  TIM1_BDTRInitStructure.TIM_LOCKLevel = TIM_LOCKLevel_1; 

  TIM1_BDTRInitStructure.TIM_DeadTime = DEADTIME;  //����ʱ��
  TIM1_BDTRInitStructure.TIM_Break =TIM_Break_Enable;  // ��������ͣ��������PWM  TIM_Break_Disable TIM_Break_Enable
  TIM1_BDTRInitStructure.TIM_BreakPolarity = TIM_BreakPolarity_High;// 
  TIM1_BDTRInitStructure.TIM_AutomaticOutput = TIM_AutomaticOutput_Disable;  

  TIM_BDTRConfig(TIM1, &TIM1_BDTRInitStructure);

	TIM_ClearITPendingBit(TIM1, TIM_IT_Break);  //���жϱ�־λ
  TIM_ITConfig(TIM1,TIM_IT_Break ,ENABLE); //ʹ���ж� 
	TIM_ClearFlag(TIM1, TIM_FLAG_Update);  
	TIM_ClearITPendingBit(TIM1, TIM_IT_Update);  //���жϱ�־λ
  TIM_ITConfig(TIM1,TIM_IT_Update,ENABLE); //���ж� 
 
 	TIM_Cmd(TIM1,DISABLE);//������ʼ
	TIM_CtrlPWMOutputs(TIM1,DISABLE);	
 
/* Configure one bit for preemption priority */
	//ѡ�����ȼ�����
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
	//����TIM1��ɲ���ж�ʹ��
	NVIC_InitStructure.NVIC_IRQChannel = TIM1_BRK_TIM9_IRQn; // TIM1_BRK_TIM9_IRQHandler
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;//ָ����ռ���ȼ�0
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;//ָ����ռ���ȼ�0
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
 
	//����TIM1�ĸ����ж�ʹ��
	NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_TIM10_IRQn;  // TIM1_UP_TIM10_IRQHandler
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;//ָ����ռ���ȼ�1
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;//ָ����Ӧ���ȼ�0
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

void Start_PWM(void)  // ��������
{
   //ʹ��PWM���ͨ��OC1/OC1N/OC2/OC2N/OC3/OC3N
    TIM1->CCER|=0x5555;	
		TIM_CtrlPWMOutputs(TIM1,ENABLE);
}
 
void Stop_PWM(void)  // ֹͣ����
{
   //��ʹ��PWM���ͨ��OC1/OC1N/OC2/OC2N/OC3/OC3N
	 TIM1->CCR1=0;
   TIM1->CCR2=0;
   TIM1->CCR3=0;
   TIM1->CCER&=0xAAAA;	
}

