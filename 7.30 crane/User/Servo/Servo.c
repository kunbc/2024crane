#include "Servo.h"

/*TIM3 PWM输出初始化 CH1*/
void TIM3_GPIO_Config(void) 
{
  GPIO_InitTypeDef GPIO_InitStructure;

  // 输出比较通道1 GPIO 初始化
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
}


///*
// * 注意：TIM_TimeBaseInitTypeDef结构体里面有5个成员，TIM6和TIM7的寄存器里面只有
// * TIM_Prescaler和TIM_Period，所以使用TIM6和TIM7的时候只需初始化这两个成员即可，
// * 另外三个成员是通用定时器和高级定时器才有.
// *-----------------------------------------------------------------------------
// *typedef struct
// *{ TIM_Prescaler            都有
// *	TIM_CounterMode			     TIMx,x[6,7]没有，其他都有
// *  TIM_Period               都有
// *  TIM_ClockDivision        TIMx,x[6,7]没有，其他都有
// *  TIM_RepetitionCounter    TIMx,x[1,8,15,16,17]才有
// *}TIM_TimeBaseInitTypeDef; 
// *-----------------------------------------------------------------------------
// */

/* ----------------   PWM信号 周期和占空比的计算--------------- */
//PWM频率：	Freq = CK_PSC / (PSC + 1) / (ARR + 1)
//PWM占空比：	Duty = CCR / (ARR + 1)
//PWM分辨率：	Reso = 1 / (ARR + 1)


void TIM3_Mode_Config(void)
{
  // 开启定时器时钟,即内部时钟CK_INT=72M
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);

/*--------------------时基结构体初始化-------------------------*/
	// 配置周期，这里配置为100K
	
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	// 自动重装载寄存器的值，累计TIM_Period+1个频率后产生一个更新或者中断
	TIM_TimeBaseStructure.TIM_Period=20000-1;					//ARR
	// 驱动CNT计数器的时钟 = Fck_int/(psc+1)
	TIM_TimeBaseStructure.TIM_Prescaler= 72-1;		//PSC
	// 时钟分频因子 ，配置死区时间时需要用到
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;		
	// 计数器计数模式，设置为向上计数
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;		
	// 重复计数器的值，没用到不用管
	TIM_TimeBaseStructure.TIM_RepetitionCounter=0;	
	// 初始化定时器
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

	/*--------------------输出比较结构体初始化-------------------*/	
	// 占空比配置
	uint16_t CCR1_Val = 1537;
//	uint16_t CCR2_Val = 4;
//	uint16_t CCR3_Val = 0;
//	uint16_t CCR4_Val = 2;
	
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	// 配置为PWM模式1
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	// 输出使能
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	// 输出通道电平极性配置	
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	
	// 输出比较通道 1
	TIM_OCInitStructure.TIM_Pulse = CCR1_Val;					//CCR
	TIM_OC1Init(TIM3, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
	
//	// 输出比较通道 2
//	TIM_OCInitStructure.TIM_Pulse = CCR2_Val;
//	TIM_OC2Init(GENERAL_TIM, &TIM_OCInitStructure);
//	TIM_OC2PreloadConfig(GENERAL_TIM, TIM_OCPreload_Enable);
	
//	// 输出比较通道 3
//	TIM_OCInitStructure.TIM_Pulse = CCR3_Val;					//CCR
//	TIM_OC3Init(GENERAL_TIM, &TIM_OCInitStructure);
//	TIM_OC3PreloadConfig(GENERAL_TIM, TIM_OCPreload_Enable);
	
//	// 输出比较通道 4
//	TIM_OCInitStructure.TIM_Pulse = CCR4_Val;
//	TIM_OC4Init(GENERAL_TIM, &TIM_OCInitStructure);
//	TIM_OC4PreloadConfig(GENERAL_TIM, TIM_OCPreload_Enable);
	
	// 使能计数器
	TIM_Cmd(TIM3, ENABLE);
}
/*TIM3定时器初始化函数*/
void TIM3_Init(void)
{
	TIM3_GPIO_Config();
	TIM3_Mode_Config();	
}
/*PWM输出占空比*/
void PWM_SetCompare(uint16_t Compare)
{
	TIM_SetCompare1(TIM3,Compare);
}
/*********************************************END OF FILE**********************/

/*舵机初始化函数*/
void Servo_Init(void)
{
	TIM3_Init();
}

/*角度计算
0			500
270		2500
*/

void Servo_SetAngle(float Angle)
{
		PWM_SetCompare(Angle / 360 * 2000 + 500);
}
