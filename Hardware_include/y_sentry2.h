#ifndef _SENTRY2_H_
#define _SENTRY2_H_

#include "main.h"


#define SENTRY2_IS_ENANLE_Blob 0X02
#define SENTRY2_IS_ENANLE_Line 0X08

extern uint16_t sentry2_enable_flag;/* �㷨ʹ�ܱ�� */
extern uint8_t sentry2_read_succeed;/* �Ƿ������ݶ�ȡ�ɹ� */
extern uint8_t sentry2_clear_succeed_flag;
extern uint8_t sentry2_enable_succeed_flag;

/* ���ݽ��ջ��ң���������־λ������ȷ�ʣ��жϵ�ǰ���㷨�����Ƿ��� */
extern uint8_t sentry2_line_enable;
extern uint8_t sentry2_blob_enable;

/* ��ȡ��5������ */
extern int sentry2_data1;
extern int sentry2_data2;
extern int sentry2_data3;
extern int sentry2_data4;
extern int sentry2_data5;

extern uint8_t sentry2_buf[32];

void sentry2_clear_all(void);

void sentry2_start_line_arithmetic(void);
void sentry2_start_blob_arithmetic(void);
void sentry2_read(void);

void sentry2_read_r(void);
void sentry2_read_g(void);
void sentry2_read_b(void);

#endif



