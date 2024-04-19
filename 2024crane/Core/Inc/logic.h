#ifndef __LOGIC_H__
#define __LOGIC_H__


#include "main.h"

/*�������ͨ�ú�*/
#define V_START         0                   /* ���ٶ� */

/*�������������*/
//�����˶��������϶�
#define L_V_END_f                 300                 /* �����˶�ĩ�ٶ� */
#define L_ACCELTIME_f             0.01f                 /* �����˶����ٹ���ʱ��*/
#define L_DECEELTIME_f            0.01f                 /* �����˶����ٹ���ʱ��*/
//�����˶�����������
#define L_V_END_s                 100                 /* �����˶�ĩ�ٶ� */
#define L_ACCELTIME_s             0.01f                 /* �����˶����ٹ���ʱ�� */
#define L_DECEELTIME_s            0.01f                 /* �����˶����ٹ���ʱ�� */


/*ƽ�Ƶ��������*/
//�����˶��������϶�
#define T_V_END_f                 300                 /* �����˶�ĩ�ٶ� */
#define T_ACCELTIME_f             0.5f                 /* �����˶����ٹ���ʱ��*/
#define T_DECEELTIME_f            0.5f                 /* �����˶����ٹ���ʱ��*/
//�����˶�����������
#define T_V_END_s                 100                 /* �����˶�ĩ�ٶ� */
#define T_ACCELTIME_s             0.03                 /* �����˶����ٹ���ʱ�� */
#define T_DECEELTIME_s            0.03                 /* �����˶����ٹ���ʱ�� */

/*����ǶȲ�����*/
#define ZHANGKAI        60.0
#define BIHE            87.0


/*****************��������******************************/
/*��������*/
void zhengji(void);
/*��ʼ������*/
void canshu_init(void);
void bushu_init(void);
/*��������*/
void Crane_Init_juece(void);
void Crane_Init_yundong(void);
void sec1(void);
void section2_zhunbei(void);
void section3_zhunbei(void);
void sec2o3 (void);
/*��������*/
int sec1_zhuaqv1(void);
int sec1_fangwu1(void);
int sec2o3_zhuaqv1(void);
int sec2o3_zhuaqv2(void);
int sec2o3_zhuaqv(void);
int sec2o3_fangwu(void);
int wai_py(void);
int da_py(void);
int nei_py(void);

#endif /* __LOGIC_H__ */
