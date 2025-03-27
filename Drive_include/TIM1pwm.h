#ifndef _TIM1pwm_H
#define _TIM1pwm_H

#include "Sys.h"   

#define CKTIM	((u32)168000000uL) 	/* Silicon running at 72MHz Resolution: 1Hz */
#define PWM_PRSC ((u8)0)                          
#define PWM_PERIOD ((u16) (CKTIM / (u32)(2*PWM_FREQ *(PWM_PRSC+1)))) //168000000/(2*8000*(1))=10500    100%
/*	������������Ƶ�� */
#define PWM_FREQ ((u16)8000) // 8K 8000
#define PWM_HalfPerMax   ((u16)5250)
#define REP_RATE (1)  
 // (N.b): Internal current loop is performed every (REP_RATE + 1)/(2*PWM_FREQ) seconds.  �ڲ�������·ÿ��REP_RATE+1��/��2*PWM_FREQ����ִ��һ�Ρ�               
 // REP_RATE has to be an odd number in case of three-shunt ������·������£��ظ��ʱ���������
 // current reading; this limitation doesn't apply to ICS
#define DEADTIME  (u16)((unsigned long long)CKTIM/2*(unsigned long long)DEADTIME_NS/1000000000uL)
#define DEADTIME_NS	((u16)2500)  //in nsec; range is [0...3500]  2500=2.1us

void TIM1_PWM_Init(void);
void Stop_PWM(void);
void Start_PWM(void);

void TIM8_PWM_Init(void);
void Start_PWM2(void);
void Stop_PWM2(void);




//#include "Sys.h"   
////////////////////////////////////////////////////////////////////////////////////	 
////STM32F4����ģ��-�⺯���汾							  
//////////////////////////////////////////////////////////////////////////////////// 	

//#define CKTIM	     ((u32)168000000uL) 	/* Silicon running at 168MHz Resolution: 1Hz */
//#define PWM_PRSC   ((u8)0)                          
//#define PWM_PERIOD ((u16) (CKTIM /(u32)(2*PWM_FREQ *(PWM_PRSC+1))))    //168000000/(2*15000*(1))=5600  
///*Power devices switching frequency������������Ƶ�� */

//#define PWM_FREQ         ((u16)15000) //15K in Hz(N.b.:pattern type is center aligned),�����������ʱ12.5kHZ
//#define PWM_HalfPerMax   ((u16)2800)
//#define REP_RATE         (1)  
// // (N.b):Internal current loop is performed every (REP_RATE + 1)/(2*PWM_FREQ) seconds�ڲ�������·ÿ��REP_RATE+1��/��2*PWM_FREQ����ִ��һ�Ρ�              
// // REP_RATE has to be an odd number in case of three-shunt ������·������£��ظ��ʱ���������
// // current reading; this limitation doesn't apply to ICS  ��ǰ�����������Ʋ�������IC
//#define DEADTIME     (u16)((unsigned long long)CKTIM/2*(unsigned long long)DEADTIME_NS/1000000000uL)
//#define DEADTIME_NS	 ((u16)2500)         //in nsec; range is [0...3500]  2500=2.1us

//void TIM1_PWM_Init(void);
//void Stop_PWM(void);
//void Start_PWM(void);
//void TIM8_PWM_Init(void);
//void Start_PWM2(void);
//void Stop_PWM2(void);



#endif
