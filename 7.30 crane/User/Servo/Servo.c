#include "Servo.h"

/*TIM3 PWM�����ʼ�� CH1*/
void TIM3_GPIO_Config(void) 
{
  GPIO_InitTypeDef GPIO_InitStructure;

  // ����Ƚ�ͨ��1 GPIO ��ʼ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
}


///*
// * ע�⣺TIM_TimeBaseInitTypeDef�ṹ��������5����Ա��TIM6��TIM7�ļĴ�������ֻ��
// * TIM_Prescaler��TIM_Period������ʹ��TIM6��TIM7��ʱ��ֻ���ʼ����������Ա���ɣ�
// * ����������Ա��ͨ�ö�ʱ���͸߼���ʱ������.
// *-----------------------------------------------------------------------------
// *typedef struct
// *{ TIM_Prescaler            ����
// *	TIM_CounterMode			     TIMx,x[6,7]û�У���������
// *  TIM_Period               ����
// *  TIM_ClockDivision        TIMx,x[6,7]û�У���������
// *  TIM_RepetitionCounter    TIMx,x[1,8,15,16,17]����
// *}TIM_TimeBaseInitTypeDef; 
// *-----------------------------------------------------------------------------
// */

/* ----------------   PWM�ź� ���ں�ռ�ձȵļ���--------------- */
//PWMƵ�ʣ�	Freq = CK_PSC / (PSC + 1) / (ARR + 1)
//PWMռ�ձȣ�	Duty = CCR / (ARR + 1)
//PWM�ֱ��ʣ�	Reso = 1 / (ARR + 1)


void TIM3_Mode_Config(void)
{
  // ������ʱ��ʱ��,���ڲ�ʱ��CK_INT=72M
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);

/*--------------------ʱ���ṹ���ʼ��-------------------------*/
	// �������ڣ���������Ϊ100K
	
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	// �Զ���װ�ؼĴ�����ֵ���ۼ�TIM_Period+1��Ƶ�ʺ����һ�����»����ж�
	TIM_TimeBaseStructure.TIM_Period=20000-1;					//ARR
	// ����CNT��������ʱ�� = Fck_int/(psc+1)
	TIM_TimeBaseStructure.TIM_Prescaler= 72-1;		//PSC
	// ʱ�ӷ�Ƶ���� ����������ʱ��ʱ��Ҫ�õ�
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;		
	// ����������ģʽ������Ϊ���ϼ���
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;		
	// �ظ���������ֵ��û�õ����ù�
	TIM_TimeBaseStructure.TIM_RepetitionCounter=0;	
	// ��ʼ����ʱ��
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

	/*--------------------����ȽϽṹ���ʼ��-------------------*/	
	// ռ�ձ�����
	uint16_t CCR1_Val = 1537;
//	uint16_t CCR2_Val = 4;
//	uint16_t CCR3_Val = 0;
//	uint16_t CCR4_Val = 2;
	
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	// ����ΪPWMģʽ1
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	// ���ʹ��
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	// ���ͨ����ƽ��������	
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	
	// ����Ƚ�ͨ�� 1
	TIM_OCInitStructure.TIM_Pulse = CCR1_Val;					//CCR
	TIM_OC1Init(TIM3, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
	
//	// ����Ƚ�ͨ�� 2
//	TIM_OCInitStructure.TIM_Pulse = CCR2_Val;
//	TIM_OC2Init(GENERAL_TIM, &TIM_OCInitStructure);
//	TIM_OC2PreloadConfig(GENERAL_TIM, TIM_OCPreload_Enable);
	
//	// ����Ƚ�ͨ�� 3
//	TIM_OCInitStructure.TIM_Pulse = CCR3_Val;					//CCR
//	TIM_OC3Init(GENERAL_TIM, &TIM_OCInitStructure);
//	TIM_OC3PreloadConfig(GENERAL_TIM, TIM_OCPreload_Enable);
	
//	// ����Ƚ�ͨ�� 4
//	TIM_OCInitStructure.TIM_Pulse = CCR4_Val;
//	TIM_OC4Init(GENERAL_TIM, &TIM_OCInitStructure);
//	TIM_OC4PreloadConfig(GENERAL_TIM, TIM_OCPreload_Enable);
	
	// ʹ�ܼ�����
	TIM_Cmd(TIM3, ENABLE);
}
/*TIM3��ʱ����ʼ������*/
void TIM3_Init(void)
{
	TIM3_GPIO_Config();
	TIM3_Mode_Config();	
}
/*PWM���ռ�ձ�*/
void PWM_SetCompare(uint16_t Compare)
{
	TIM_SetCompare1(TIM3,Compare);
}
/*********************************************END OF FILE**********************/

/*�����ʼ������*/
void Servo_Init(void)
{
	TIM3_Init();
}

/*�Ƕȼ���
0			500
270		2500
*/

void Servo_SetAngle(float Angle)
{
		PWM_SetCompare(Angle / 360 * 2000 + 500);
}
