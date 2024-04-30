#ifndef __BSP_CAN
#define __BSP_CAN

#include "stm32f4xx_hal.h"
#include "mytype.h"
#include "can.h"
// #include "pid.h"

#define ABS(x)		((x>0)? x: -x)
/*CAN发送或是接收的ID*/
typedef enum
{
	CAN_2006Moto_ALL_ID = 0x200,
	CAN_2006Moto1_ID = 0x201,
	CAN_2006Moto2_ID = 0x202,
	CAN_2006Moto3_ID = 0x203,
	CAN_2006Moto4_ID = 0x204,	
}CAN_Message_ID;


/*接收到的云台电机的参数结构体*/
typedef struct{
	int16_t	 	speed_rpm;
  float  	  real_current;
  int16_t  	given_current;
  uint8_t  	hall;
	uint16_t 	angle;				//abs angle range:[0,8191]
	uint16_t 	last_angle;	//abs angle range:[0,8191]
	uint16_t	offset_angle;  //偏斜角
	int32_t		round_cnt;     //旋转圈数
	int32_t		total_angle;   //总转角
	u32			  msg_cnt;
}moto_measure_t;

typedef struct
{
	float err;//当前偏差
	float err_last;//上次偏差
	float err_sum;//误差累计值
	float Kp,Ki,Kd;//比例，积分，微分系数
    float out;
} tPid;


typedef struct
{
    uint16_t ecd;// 角度
    int16_t speed_rpm;//转速
    int16_t given_current;//电流
    int16_t last_ecd;//
} motor_measure_t;
typedef struct 
{
  const motor_measure_t *data;
  float speed;
  float tar_speed;
  int16_t given_current;
  /* data */
}classic_motor;

typedef struct
{
  classic_motor motor[4];
  tPid pid[4];
  /* data */
}motor_control;

/* Extern  ------------------------------------------------------------------*/
extern moto_measure_t  moto_chassis[];


void can_filter_init(CAN_HandleTypeDef* _hcan);
void get_moto_measure(moto_measure_t *ptr, CAN_HandleTypeDef* hcan);
void can_receive_onetime(CAN_HandleTypeDef* _hcan, int time);
void set_moto_current(s16 iq1, s16 iq2, s16 iq3, s16 iq4);

const motor_measure_t *get_chassis_motor_measure_point(uint8_t i);
#endif
