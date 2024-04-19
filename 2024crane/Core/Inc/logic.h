#ifndef __LOGIC_H__
#define __LOGIC_H__


#include "main.h"

/*步进电机通用宏*/
#define V_START         0                   /* 初速度 */

/*起升电机参数宏*/
//快速运动，步数较多
#define L_V_END_f                 300                 /* 快速运动末速度 */
#define L_ACCELTIME_f             0.01f                 /* 快速运动加速过程时间*/
#define L_DECEELTIME_f            0.01f                 /* 快速运动减速过程时间*/
//慢速运动，步数较少
#define L_V_END_s                 100                 /* 慢速运动末速度 */
#define L_ACCELTIME_s             0.01f                 /* 慢速运动加速过程时间 */
#define L_DECEELTIME_s            0.01f                 /* 慢速运动减速过程时间 */


/*平移电机参数宏*/
//快速运动，步数较多
#define T_V_END_f                 300                 /* 快速运动末速度 */
#define T_ACCELTIME_f             0.5f                 /* 快速运动加速过程时间*/
#define T_DECEELTIME_f            0.5f                 /* 快速运动减速过程时间*/
//慢速运动，步数较少
#define T_V_END_s                 100                 /* 慢速运动末速度 */
#define T_ACCELTIME_s             0.03                 /* 慢速运动加速过程时间 */
#define T_DECEELTIME_s            0.03                 /* 慢速运动减速过程时间 */

/*舵机角度参数宏*/
#define ZHANGKAI        60.0
#define BIHE            87.0


/*****************函数声明******************************/
/*整机函数*/
void zhengji(void);
/*初始化函数*/
void canshu_init(void);
void bushu_init(void);
/*工作函数*/
void Crane_Init_juece(void);
void Crane_Init_yundong(void);
void sec1(void);
void section2_zhunbei(void);
void section3_zhunbei(void);
void sec2o3 (void);
/*动作函数*/
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
