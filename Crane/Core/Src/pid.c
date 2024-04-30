#include "pid.h"
#include "stm32f4xx_hal.h"

extern float PID_Position_output,PID_Speed_output;



// static void PID_Position_param_init(
// 	PID_Position_TypeDef 	*pid, 
// //	PID_ID   			id,
// 	uint16_t 			maxout,
// 	uint16_t 			intergral_limit,
// 	float 				deadband,
// 	uint16_t 			period,
// 	int16_t 			max_err,
// 	int16_t  			Target,

// 	float 				KP, 
// 	float 				KI, 
// 	float 				KD){
		
// //		pid->id = id;
// 		pid->MaxOutput = maxout;
// 		pid->IntegralLimit = intergral_limit;
// 		pid->DeadBand = deadband;
// 		pid->ControlPeriod = period;
// 		pid->Max_Err = max_err;
// 		pid->target = Target;
		
// 		pid->kp = KP;
// 		pid->ki = KI;
// 		pid->kd = KD;
		
// 		pid->output = 0;
		
// }

// static void PID_Speed_param_init(
// 	PID_Speed_TypeDef 	*pid, 
// //	PID_ID   			id,
// 	uint16_t 			maxout,
// 	uint16_t 			intergral_limit,
// 	float 				deadband,
// 	uint16_t 			period,
// 	int16_t 			max_err,
// 	int16_t  			Target,

// 	float 				KP, 
// 	float 				KI, 
// 	float 				KD){
		
// //		pid->id = id;
// 		pid->MaxOutput = maxout;
// 		pid->IntegralLimit = intergral_limit;
// 		pid->DeadBand = deadband;
// 		pid->ControlPeriod = period;
// 		pid->Max_Err = max_err;
// 		pid->target = Target;
		
// 		pid->kp = KP;
// 		pid->ki = KI;
// 		pid->kd = KD;
		
// 		pid->output = 0;
		
// }

void PID_Speed_reset(tPid * pid, float kp, float ki, float kd)
{
	pid->Kp = kp;
	pid->Ki = ki;
	pid->Kd = kd;
	pid->out=0.0f;
	pid->err=0.0f;
	pid->err_last=0.0f;
	pid->err_sum=0.0f;
}


void pid_init(motor_control* classic_move)
{
	uint8_t i;
	for (i = 0; i < 4; i++)
	{
		PID_Speed_reset(&(classic_move->pid[i]),300.0,3.0,1.0);
		/* code */
	}
	
}

void PI_realize(tPid * pid,float actual_val,float target_val)
{
	// pid->actual_val = actual_val;
	pid->err = target_val - actual_val;
	
	pid->out += pid->Kp*(pid->err-pid->err_last) + pid->Ki*pid->err;
	if(pid->out>=5000)
	{
		pid->out=5000;
	}
	if(pid->out<=-5000)
	{
		pid->out=-5000;
	}
	pid->err_last=pid->err;
	// return pid->out;
}

void PID_realize(tPid * pid,float actual_val,float target_val)
{
	// pid->actual_val = actual_val;
	pid->err = target_val - actual_val;
	pid->err_sum += pid->err;
	if(pid->err_sum>=3000)
	{
		pid->err_sum=3000;
	}
	if(pid->err_sum<=-3000)
	{
		pid->err_sum=-3000;
	}
	pid->out = pid->Kp*pid->err + pid->Ki*pid->err_sum + pid->Kd*(pid->err - pid->err_last);
	if(pid->out>=5000)
	{
		pid->out=5000;
	}
	if(pid->out<=-5000)
	{
		pid->out=-5000;
	}
	
	pid->err_last = pid->err;
	
	// return pid->actual_val;
}

void PID_stop_realize(tPid * pid,float actual_val,float target_val)
{
	// pid->actual_val = actual_val;
	pid->err = target_val - actual_val;
	pid->err_sum += pid->err;
	if(pid->err_sum>=15)
	{
		pid->err_sum=15;
	}
	if(pid->err_sum<=-15)
	{
		pid->err_sum=-15;
	}
	pid->out = pid->Kp*pid->err + pid->Ki*pid->err_sum + pid->Kd*(pid->err - pid->err_last);
	if(pid->out>=SPEED*1.5f)
	{
		pid->out=SPEED*1.5f;
	}
	if(pid->out<=-SPEED*1.5f)
	{
		pid->out=-SPEED*1.5f;
	}
	pid->err_last = pid->err;
	
	// return pid->actual_val;
}



