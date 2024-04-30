#ifndef _PID_H
#define _PID_H

#include "stdint.h"
#include "bsp_can.h"
//typedef enum
//{

//	PID_Position,
//	PID_Speed
//	
//}PID_ID;

#define SPEED 16.0f

typedef struct _PID_Position_TypeDef
{
//	PID_ID 		id;
	
	float 		target;							//鐩敓鏂ゆ嫹鍊�
	float 		lastNoneZeroTarget;
	float	 		kp;
	float 		ki;
	float	 		kd;
	
	float   	measure;					//閿熸枻鎷烽敓鏂ゆ嫹鍊�
	float   	err;							//閿熸枻鎷烽敓锟�
	float   	last_err;      		//閿熻緝杈炬嫹閿熸枻鎷烽敓锟�
	
	float 		pout;
	float 		iout;
	float 		dout;
	
	float 		output;						//閿熸枻鎷烽敓鏂ゆ嫹閿熸枻鎷烽敓锟�
	float 		last_output;			//閿熻緝杈炬嫹閿熸枻鎷烽敓锟�

	float 		MaxOutput;				//閿熸枻鎷烽敓鏂ゆ嫹钖归敓锟�
	float 		IntegralLimit;		//閿熸枻鎷烽敓鏂ゆ嫹閿熺潾鍑ゆ嫹
	float 		DeadBand;			  //閿熸枻鎷烽敓鏂ゆ嫹閿熸枻鎷烽敓鏂ゆ嫹閿熸枻鎷峰€奸敓鏂ゆ嫹閿熸枻鎷烽敓鏂ゆ嫹鍋忛敓鏂ゆ嫹灏忛敓鏂ゆ嫹閿熸枻鎷蜂负閿熸枻鎷峰亸閿熸枻鎷�
	float 		ControlPeriod;		//閿熸枻鎷烽敓鏂ゆ嫹閿熸枻鎷烽敓鏂ゆ嫹
	float  		Max_Err;					//閿熸枻鎷烽敓鏂ゆ嫹閿熸枻鎷�
	
	uint32_t 	thistime;
	uint32_t 	lasttime;
	uint8_t 	dtime;	
	
	void (*f_param_init)(struct _PID_Position_TypeDef *pid,  //PID閿熸枻鎷烽敓鏂ゆ嫹閿熸枻鎷峰閿熸枻鎷�
//				   PID_ID id,
				   uint16_t maxOutput,
				   uint16_t integralLimit,
				   float deadband,
				   uint16_t controlPeriod,
					 int16_t max_err,     
					 int16_t  target,
				   float kp,
				   float ki,
				   float kd);
				   
	void (*f_pid_reset)(struct _PID_Position_TypeDef *pid, float kp,float ki, float kd);
//	float (*f_cal_pid)(struct _PID_TypeDef *pid, float measure);
	float (*f_Position_cal_pid)(struct _PID_Position_TypeDef *pid, float measure,float target);
}PID_Position_TypeDef;


typedef struct _PID_Speed_TypeDef
{
//	PID_ID 		id;
	
	float 		target;							//鐩敓鏂ゆ嫹鍊�
	float 		lastNoneZeroTarget;
	float	 		kp;
	float 		ki;
	float	 		kd;
	
	float   	measure;					//閿熸枻鎷烽敓鏂ゆ嫹鍊�
	float   	err;							//閿熸枻鎷烽敓锟�
	float   	last_err;      		//閿熻緝杈炬嫹閿熸枻鎷烽敓锟�
	
	float 		pout;
	float 		iout;
	float 		dout;
	
	float 		output;						//閿熸枻鎷烽敓鏂ゆ嫹閿熸枻鎷烽敓锟�
	float 		last_output;			//閿熻緝杈炬嫹閿熸枻鎷烽敓锟�

	float 		MaxOutput;				//閿熸枻鎷烽敓鏂ゆ嫹钖归敓锟�
	float 		IntegralLimit;		//閿熸枻鎷烽敓鏂ゆ嫹閿熺潾鍑ゆ嫹
	float 		DeadBand;			  //閿熸枻鎷烽敓鏂ゆ嫹閿熸枻鎷烽敓鏂ゆ嫹閿熸枻鎷峰€奸敓鏂ゆ嫹閿熸枻鎷烽敓鏂ゆ嫹鍋忛敓鏂ゆ嫹灏忛敓鏂ゆ嫹閿熸枻鎷蜂负閿熸枻鎷峰亸閿熸枻鎷�
	float 		ControlPeriod;		//閿熸枻鎷烽敓鏂ゆ嫹閿熸枻鎷烽敓鏂ゆ嫹
	float  		Max_Err;					//閿熸枻鎷烽敓鏂ゆ嫹閿熸枻鎷�
	
	uint32_t 	thistime;
	uint32_t 	lasttime;
	uint8_t 	dtime;	
	
	void (*f_param_init)(struct _PID_Speed_TypeDef *pid,  //PID閿熸枻鎷烽敓鏂ゆ嫹閿熸枻鎷峰閿熸枻鎷�
//				   PID_ID id,
				   uint16_t maxOutput,
				   uint16_t integralLimit,
				   float deadband,
				   uint16_t controlPeriod,
					 int16_t max_err,     
					 int16_t  target,
				   float kp,
				   float ki,
				   float kd);
				   
	void (*f_pid_reset)(struct _PID_Speed_TypeDef *pid, float kp,float ki, float kd);
	float (*f_cal_pid)(struct _PID_Speed_TypeDef* Speed_pid,struct _PID_Position_TypeDef* Position_pid, float Position_measure, float Speed_measure,float target);
	float (*f_speed_cal_pid)(struct _PID_Speed_TypeDef *pid, float measure,float target);
}PID_Speed_TypeDef;






extern PID_Speed_TypeDef motor_Speed_pid[4];
extern PID_Position_TypeDef motor_Position_pid[4];

extern void pid_init(motor_control* classic_move);
extern void PID_Speed_reset(tPid * pid, float kp, float ki, float kd);
extern void PI_realize(tPid * pid,float actual_val,float target_val);
extern void PID_realize(tPid * pid,float actual_val,float target_val);
extern void PID_stop_realize(tPid * pid,float actual_val,float target_val);
#endif
