#ifndef __STEPPER_MOTOR_H
#define __STEPPER_MOTOR_H

#include "main.h"

/******************************************************************************************/

#define T1_FREQ                 (168000000/168)  		//��ʱ����Ƶf1                       /* Ƶ��ftֵ */
#define FSPR                    200              		//���������200����תһȦ                       /* ���������Ȧ���� */
#define MICRO_STEP              16                		//ϸ��ֵ
#define SPR                     (FSPR * MICRO_STEP) //�������ϸ��ֵһȦ��������                    /* ��Ȧ����Ҫ�������� */

#define ROUNDPS_2_STEPPS(rpm)   ((rpm) * SPR / 60)                      /* ���ݵ��ת�٣�r/min�������������٣�step/s�� */
#define MIDDLEVELOCITY(vo,vt)   ( ( (vo) + (vt) ) / 2 )                 /* S�ͼӼ��ټ��ٶε��е��ٶ�  */
#define INCACCEL(vo,v,t)        ( ( 2 * ((v) - (vo)) ) / pow((t),2) )   /* �Ӽ��ٶ�:���ٶ�������   V - V0 = 1/2 * J * t^2 */
#define INCACCELSTEP(j,t)       ( ( (j) * pow( (t) , 3 ) ) / 6.0f )     /* �Ӽ��ٶε�λ����(����)  S = 1/6 * J * t^3 */
#define ACCEL_TIME(t)           ( (t) / 2 )                             /* �Ӽ��ٶκͼ����ٶε�ʱ������ȵ� */
#define SPEED_MIN               (T1_FREQ / (65535.0f))                  /* ���Ƶ��/�ٶ� */

#ifndef TRUE
#define TRUE                    1
#endif
#ifndef FALSE
#define FALSE                   0
#endif

typedef struct {
    int32_t vo;             /*  ���ٶ� ��λ step/s */
    int32_t vt;             /*  ĩ�ٶ� ��λ step/s */
    int32_t accel_step;     /*  ���ٶεĲ�����λ step */
    int32_t decel_step;     /*  ���ٶεĲ�����λ step */
    float   *accel_tab;     /*  �ٶȱ�� ��λ step/s �������������Ƶ�� */
    float   *decel_tab;     /*  �ٶȱ�� ��λ step/s �������������Ƶ�� */
    float   *ptr;           /*  �ٶ�ָ�� */
    int32_t dec_point;      /*  ���ٵ� */
    int32_t step;
    int32_t step_pos;
} speed_calc_t;

typedef enum
{
    STATE_ACCEL = 1,        /* �������״̬ */
    STATE_AVESPEED = 2,     /* �������״̬ */
    STATE_DECEL = 3,        /* �������״̬ */
    STATE_STOP = 0,         /* ���ֹͣ״̬ */
    STATE_IDLE = 4,         /* �������״̬ */
} motor_state_typedef;

enum DIR
{
 CCW = 0,                   /*��ʱ��*/ 
 CW =1                        /*˳ʱ��*/
};

enum EN
{
 EN_ON = 0,                 /* ʧ���ѻ����� */
 EN_OFF                     /* ʹ���ѻ����� ʹ�ܺ���ֹͣ��ת */
};

/******************************************************************************************/

/* ���������Ŷ���*/

#define STEPPER_MOTOR_1       1         /* ��������ӿ���� */
#define STEPPER_MOTOR_2       2
#define STEPPER_MOTOR_3       3
#define STEPPER_MOTOR_4       4

/*----------------------- �������ſ��� -----------------------------------*/
/* ��������ʹ�õ��ǹ������ⷨ������Ӳ���Ե�ƽ����ȡ�����������Ե� x = 1 ��Ч��x = 0ʱ��Ч*/  
#define ST1_DIR(x)    do{ x ? \
                              HAL_GPIO_WritePin(GPIOG, GPIO_PIN_11, GPIO_PIN_SET) : \
                              HAL_GPIO_WritePin(GPIOG, GPIO_PIN_11, GPIO_PIN_RESET); \
                          }while(0)  

#define ST2_DIR(x)    do{ x ? \
                              HAL_GPIO_WritePin(GPIOG, GPIO_PIN_12, GPIO_PIN_SET) : \
                              HAL_GPIO_WritePin(GPIOG, GPIO_PIN_12, GPIO_PIN_RESET); \
                          }while(0)  

#define ST3_DIR(x)    do{ x ? \
                              HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13, GPIO_PIN_SET) : \
                              HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13, GPIO_PIN_RESET); \
                          }while(0)  

#define ST4_DIR(x)    do{ x ? \
                              HAL_GPIO_WritePin(GPIOG, GPIO_PIN_14, GPIO_PIN_SET) : \
                              HAL_GPIO_WritePin(GPIOG, GPIO_PIN_14, GPIO_PIN_RESET); \
                          }while(0)  

/******************************************************************************************/
/* �ⲿ�ӿں���*/
void stepper_start(uint8_t motor_num);                       /* ����������� */
void stepper_stop(uint8_t motor_num);                       /* �رղ������ */     
void stepmotor_move_rel(int32_t vo, int32_t vt, float AcTime,float DeTime,int32_t step,uint8_t id);  /* S�ͼӼ����˶����ƺ��� */
uint8_t calc_speed(int32_t vo, int32_t vt, float time, uint8_t id);    /* �����ٶȱ� */                   
#endif
