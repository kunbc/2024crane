#ifndef __LOGIC_H__
#define __LOGIC_H__


#include "main.h"

/*�������ͨ�ú�*/
#define V_START         0                   /* ���ٶ� */

/*�������������*/
//�����˶��������϶�
#define fast_57END                100                 /* �����˶�ĩ�ٶ� */
#define fast_57ACTIME             0.05f                 /* �����˶����ٹ���ʱ��*/
#define fast_57DETIME             0.05f                 /* �����˶����ٹ���ʱ��*/
//�����˶�����������
#define slow_57END                20                 /* �����˶�ĩ�ٶ� */
#define slow_57ACTIME             0.01f                 /* �����˶����ٹ���ʱ�� */
#define slow_57DETIME             0.01f                 /* �����˶����ٹ���ʱ�� */


/*ƽ�Ƶ��������*/
//�����˶��������϶�
#define fast_42END                300                 /* �����˶�ĩ�ٶ� */
#define fast_42ACTIME             0.5f                 /* �����˶����ٹ���ʱ��*/
#define fast_42DETIME             0.5f                 /* �����˶����ٹ���ʱ��*/
//�����˶�����������
#define slow_42END                100                 /* �����˶�ĩ�ٶ� */
#define slow_42ACTIME             0.1                 /* �����˶����ٹ���ʱ�� */
#define slow_42DETIME             0.1                 /* �����˶����ٹ���ʱ�� */


/*����ǶȲ�����*/
#define Z_sec1          10.0
#define ZHANGKAI        60.0
#define BIHE            87.0


/*****************��������******************************/
/*��������*/
void zhengji(void);
/*��ʼ������*/
void canshu_init(void);
void bushu_init(void);
/*��������*/
void Crane_Init(void);
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
