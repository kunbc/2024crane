#ifndef __STEPPER_MOTOR_H
#define __STEPPER_MOTOR_H

#include "main.h"

/******************************************************************************************/

#define T1_FREQ                 (168000000/168)  		//定时器分频f1                       /* 频率ft值 */
#define FSPR                    200              		//电机理论上200脉冲转一圈                       /* 步进电机单圈步数 */
#define MICRO_STEP              16                		//细分值
#define SPR                     (FSPR * MICRO_STEP) //电机算上细分值一圈的脉冲数                    /* 单圈所需要的脉冲数 */

#define ROUNDPS_2_STEPPS(rpm)   ((rpm) * SPR / 60)                      /* 根据电机转速（r/min），计算电机步速（step/s） */
#define MIDDLEVELOCITY(vo,vt)   ( ( (vo) + (vt) ) / 2 )                 /* S型加减速加速段的中点速度  */
#define INCACCEL(vo,v,t)        ( ( 2 * ((v) - (vo)) ) / pow((t),2) )   /* 加加速度:加速度增加量   V - V0 = 1/2 * J * t^2 */
#define INCACCELSTEP(j,t)       ( ( (j) * pow( (t) , 3 ) ) / 6.0f )     /* 加加速段的位移量(步数)  S = 1/6 * J * t^3 */
#define ACCEL_TIME(t)           ( (t) / 2 )                             /* 加加速段和减加速段的时间是相等的 */
#define SPEED_MIN               (T1_FREQ / (65535.0f))                  /* 最低频率/速度 */

#ifndef TRUE
#define TRUE                    1
#endif
#ifndef FALSE
#define FALSE                   0
#endif

typedef struct {
    int32_t vo;             /*  初速度 单位 step/s */
    int32_t vt;             /*  末速度 单位 step/s */
    int32_t accel_step;     /*  加速段的步数单位 step */
    int32_t decel_step;     /*  加速段的步数单位 step */
    float   *accel_tab;     /*  速度表格 单位 step/s 步进电机的脉冲频率 */
    float   *decel_tab;     /*  速度表格 单位 step/s 步进电机的脉冲频率 */
    float   *ptr;           /*  速度指针 */
    int32_t dec_point;      /*  减速点 */
    int32_t step;
    int32_t step_pos;
} speed_calc_t;

typedef enum
{
    STATE_ACCEL = 1,        /* 电机加速状态 */
    STATE_AVESPEED = 2,     /* 电机匀速状态 */
    STATE_DECEL = 3,        /* 电机减速状态 */
    STATE_STOP = 0,         /* 电机停止状态 */
    STATE_IDLE = 4,         /* 电机空闲状态 */
} motor_state_typedef;

enum DIR
{
 CCW = 0,                   /*逆时针*/ 
 CW =1                        /*顺时针*/
};

enum EN
{
 EN_ON = 0,                 /* 失能脱机引脚 */
 EN_OFF                     /* 使能脱机引脚 使能后电机停止旋转 */
};

/******************************************************************************************/

/* 步进电机序号定义*/

#define STEPPER_MOTOR_1       1         /* 步进电机接口序号 */
#define STEPPER_MOTOR_2       2
#define STEPPER_MOTOR_3       3
#define STEPPER_MOTOR_4       4

/*----------------------- 方向引脚控制 -----------------------------------*/
/* 由于我们使用的是共阳极解法，并且硬件对电平做了取反操作，所以当 x = 1 有效，x = 0时无效*/  
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
/* 外部接口函数*/
void stepper_start(uint8_t motor_num);                       /* 开启步进电机 */
void stepper_stop(uint8_t motor_num);                       /* 关闭步进电机 */     
void stepmotor_move_rel(int32_t vo, int32_t vt, float AcTime,float DeTime,int32_t step,uint8_t id);  /* S型加减速运动控制函数 */
uint8_t calc_speed(int32_t vo, int32_t vt, float time, uint8_t id);    /* 计算速度表 */                   
#endif
