#include "servo.h"
#include "tim.h"
#include <math.h>

/*
			     PWM   
Servo1   	 PA6	 
Servo2   	 PA7	 


*/

double star_angle[2]={0};//起始角度
double stop_angle=0;//终止角度
uint16_t set_num=0;//调整次数
uint16_t pulse=0,stop_pulse=0;
uint8_t steering=0;//舵机运行标志位，舵机运行过程中为1，停止状态为0
uint8_t steer_tim=8;//数值越大，电机转的越慢
int id=0;//舵机编号

/**
 * @brief       舵机角度初始化
 * @param       Start_Angle : 起始角度
 * @retval      无
 */
void steer_init(double Start_Angle) 
{
	star_angle[0]=Start_Angle;//设置舵机起始角度。可以修改
	star_angle[1]=Start_Angle;//设置舵机起始角度。可以修改
	
	pulse=star_angle[0]*100/9+500;
	__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,pulse);
	__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,pulse);
}

/**
 * @brief       舵机角度初始化
 * @param       target_angle : 目标角度
 * @param       Servo_num : 舵机编号
 * @retval      无
 */
void steer(double target_angle,int Servo_num)
{
	while(steering);
	id = Servo_num;//给舵机赋值 1、2、3-全部运行
	if(id==1||id==2) 
	{
			set_num = ceil(fabs(target_angle - star_angle[id-1])*steer_tim);//ceil向上取整，fabs去绝对值，每个角度做8次
	}
	else if(id==3) 
	{
			set_num = ceil(fabs(target_angle - star_angle[0])*steer_tim);//ceil向上取整，fabs去绝对值，每个角度做8次
	}
	
	stop_angle = target_angle;
	stop_pulse = stop_angle*100/9+500;
	
	HAL_TIM_Base_Start_IT(&htim10);//启动定时器10中断
	steering = 1;

}

/*定时周期性中断函数*/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim -> Instance == TIM10)//定时器10控制PID调整时间
	{
			static uint16_t num = 0;
			static double angle;
			if(set_num>=num)
			{
					num++;
				if(id==1||id==2)
				{
						angle = star_angle[id-1]+3*(stop_angle-star_angle[id-1])*num*num/set_num/set_num
						-2*(stop_angle-star_angle[id-1])*num*num*num/set_num/set_num/set_num;
						pulse = angle*100/9+500;//加减速计算函数
				}
				else if(id==3)
				{
						angle = star_angle[0]+3*(stop_angle-star_angle[0])*num*num/set_num/set_num
						-2*(stop_angle-star_angle[0])*num*num*num/set_num/set_num/set_num;
						pulse = angle*100/9+500;//加减速计算函数
				}
				switch(id)
					{
							case SERVO_1:
									__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,pulse);//转到中间角度
							break;
							case SERVO_2:
									__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,pulse);//转到中间角度
							break;
							case SERVO_ALL:
									__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,pulse);//转到中间角度
									__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,pulse);//转到中间角度
							break;
					}
				}
			else 
			{
					switch(id)
					{
							case SERVO_1:
									__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,stop_angle);//转到停止角度
									star_angle[0] = stop_angle;
							break;
							case SERVO_2:
									__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,stop_angle);//转到停止角度
									star_angle[1] = stop_angle;
							break;
							case SERVO_ALL:
									__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,stop_angle);//转到停止角度
									__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,stop_angle);//转到停止角度
									star_angle[0] = stop_angle;
									star_angle[1] = stop_angle;
							break;
					}
					HAL_TIM_Base_Stop_IT(&htim10);//关闭定时器10中断
					steering = 0;
					num = 0;
					id=0;
			}
	}

}

