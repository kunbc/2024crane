#include "servo.h"
#include "tim.h"
#include <math.h>

/*
			     PWM   
Servo1   	 PA6	 
Servo2   	 PA7	 

*/

double star_angle=0;//��ʼ�Ƕ�
double stop_angle=0;//��ֹ�Ƕ�
uint16_t set_num=0;//��������
uint16_t pulse=0,stop_pulse=0;
uint8_t steering=0;//������б�־λ��������й�����Ϊ1��ֹͣ״̬Ϊ0
uint8_t steer_tim=8;//��ֵԽ�󣬵��ת��Խ��
int id=0;//������

/**
 * @brief       ����Ƕȳ�ʼ��
 * @param       Start_Angle : ��ʼ�Ƕ�
 * @retval      ��
 */
void steer_init(double Start_Angle) 
{
	star_angle=Start_Angle;//���ö����ʼ�Ƕȡ������޸�
	pulse=star_angle*100/9+500;
	__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,pulse);
	__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,pulse);
}

/**
 * @brief       ����Ƕȳ�ʼ��
 * @param       target_angle : Ŀ��Ƕ�
 * @param       Servo_num : ������
 * @retval      ��
 */
void steer(double target_angle,int Servo_num)
{
	while(steering);
	id = Servo_num;//�������ֵ 1��2��3-ȫ������
	set_num = ceil(fabs(target_angle - star_angle)*steer_tim);//ceil����ȡ����fabsȥ����ֵ��ÿ���Ƕ���8��
	stop_angle = target_angle;
	stop_pulse = stop_angle*100/9+500;
	HAL_TIM_Base_Start_IT(&htim10);//������ʱ��10�ж�
	steering = 1;

}

/*��ʱ�������жϺ���*/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim -> Instance == TIM10)//��ʱ��10����PID����ʱ��
	{
			static uint16_t num = 0;
			static double angle;
			if(set_num>=num)
			{
					num++;
					angle = star_angle+3*(stop_angle-star_angle)*num*num/set_num/set_num
					-2*(stop_angle-star_angle)*num*num*num/set_num/set_num/set_num;
					pulse = angle*100/9+500;//�Ӽ��ټ��㺯��
					switch(id)
					{
							case SERVO_1:
									__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,pulse);//ת���м�Ƕ�
							break;
							case SERVO_2:
									__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,pulse);//ת���м�Ƕ�
							break;
							case SERVO_ALL:
									__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,pulse);//ת���м�Ƕ�
									__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,pulse);//ת���м�Ƕ�
							break;
					}
				}
			else 
			{
					__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,stop_angle);//ת��ֹͣ�Ƕ�
					__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,stop_angle);//ת��ֹͣ�Ƕ�
					HAL_TIM_Base_Stop_IT(&htim10);//�رն�ʱ��10�ж�
					star_angle = stop_angle;
					steering = 0;
					num = 0;
					id=0;
			}
	}

}

