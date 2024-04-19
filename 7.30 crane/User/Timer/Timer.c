#include "timer.h"
#include "bsp_usart.h"

extern uint8_t stop1;
extern uint8_t stop2;
extern uint8_t dir1,dir2;
extern int count1;
///*********** Group A	TIM1-TIM2主从***********/
//// 定时器1主模式
//void TIM1_GPIO_Config(uint16_t TIM1_Prescaler, uint16_t TIM1_Period, uint16_t CCR_A1)
//{
//	GPIO_InitTypeDef	GPIO_InitStructure;
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
//	// TIM1通道1\4
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;		// TIM1_CH1 - PA8， CH4 - PA11
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;			// 复用推挽输出
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_Init(GPIOA, &GPIO_InitStructure);
//	
//	TIM_TimeBaseInitTypeDef		TIM_TimeBaseStructure;
//	TIM_TimeBaseStructure.TIM_Period = TIM1_Period - 1;
//	TIM_TimeBaseStructure.TIM_Prescaler = TIM1_Prescaler - 1;
//	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
//	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
//	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;			// 重复计数，一定要 = 0; 高级定时器TIM1,TIM8，这句必须有。
//	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
//	
//	TIM_OCInitTypeDef		TIM_OCInitStructure;
//	// 设置工作模式
//	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;							// PWM1
//	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;	// 比较输出使能
//	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;			// 输出极性

//	// PWM通道，TIM1 - 通道1设置函数，50/100，脉冲信号
//	TIM_OCInitStructure.TIM_Pulse = CCR_A1;								// 设置待装入捕获寄存器的脉冲值
//	TIM_OC1Init( TIM1, &TIM_OCInitStructure);							// CH1预装载使能，修改
//	TIM_SelectMasterSlaveMode( TIM1, TIM_MasterSlaveMode_Enable);		// 主从模式下，作为主定时器使能
//	TIM_SelectOutputTrigger( TIM1, TIM_TRGOSource_Update);		// TIM - EGR寄存器， 定义UG位，产生更新事件
//	TIM_OC1PreloadConfig( TIM1, TIM_OCPreload_Enable);

////	// PWM通道，TIM1 - 通道4设置函数，50/100，脉冲信号
////	TIM_OCInitStructure.TIM_Pulse = CCR_A4;							// 初始化 TIM1-OC4
////	TIM_OC4Init( TIM1, &TIM_OCInitStructure);						// CH4预装载使能，修改
////	TIM_SelectMasterSlaveMode( TIM1, TIM_MasterSlaveMode_Enable);		// 主从模式下，作为主定时器使能
////	TIM_SelectOutputTrigger( TIM1, TIM_TRGOSource_Update);		// TIM - EGR寄存器， 定义UG位，产生更新事件
////	TIM_OC4PreloadConfig( TIM1, TIM_OCPreload_Enable);
//	
//	TIM_ARRPreloadConfig(TIM1, ENABLE);				// 预装载使能	
//}

////定时器2从模式
//void TIM2_GPIO_Config(u32 PulseNum_A)
//{
//	TIM_TimeBaseInitTypeDef	TIM_TimeBaseStructure;
//	NVIC_InitTypeDef	NVIC_InitStructure;
//	
//	RCC_APB1PeriphClockCmd( RCC_APB1Periph_TIM2, ENABLE);
//	
//	TIM_TimeBaseStructure.TIM_Period = PulseNum_A;
//	TIM_TimeBaseStructure.TIM_Prescaler = 0;
//	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
//	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;			// 向上计数
//	TIM_TimeBaseInit( TIM2, &TIM_TimeBaseStructure);
//	
//	TIM_SelectInputTrigger( TIM2, TIM_TS_ITR0);			// TIM1-主，TIM2-从，表中对应 ITR0
//	TIM_SelectSlaveMode( TIM2, TIM_SlaveMode_External1);		//主从模式下作为从定时器使能
//	TIM_ITConfig( TIM2, TIM_IT_Update, DISABLE);
//	
//	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3);
//	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init( &NVIC_InitStructure);
//}


///*******A组（TIM1-TIM2）主从定时器PWM输出 *******/
//// Cycle_A = Preiod_A; PulseNum_A 输出脉冲个数;DIR_A1 DIR_A2 高/低电平-方向信号
//void PWM_Output_A(u16 Cycle_A, u32 PulseNum_A,uint16_t CCR_A1)			// TIM1-主，TIM2-从，输出PWM
//{
//	TIM2_GPIO_Config(PulseNum_A);
//	TIM_Cmd( TIM2, ENABLE);
//	TIM_ClearITPendingBit( TIM2, TIM_IT_Update);
//	TIM_ITConfig( TIM2, TIM_IT_Update, ENABLE);
//	TIM1_GPIO_Config(72, Cycle_A, CCR_A1);	// 72M / 72 = 1MHz;	
//	TIM_Cmd( TIM1, ENABLE);
//	TIM_CtrlPWMOutputs( TIM1, ENABLE);		// 高级定时器必须有，使能其输出
//}
///*******中断函数*******/
//// Group A	TIM1-TIM2主从定时器
//void TIM2_IRQHandler(void)
//{
//	if (TIM_GetITStatus( TIM2, TIM_IT_Update) != RESET)
//	{
//		TIM_ClearITPendingBit( TIM2, TIM_IT_Update);			// 清除中断标志位
//		TIM_CtrlPWMOutputs( TIM1, DISABLE);		// 主输出使能关闭，高级定时器必须有
//		TIM_Cmd( TIM1, DISABLE);			// 关闭定时器1
//		TIM_Cmd( TIM2, DISABLE);			// 关闭定时器2
//		TIM_ITConfig( TIM2, TIM_IT_Update, DISABLE);
//		count1++;
//		printf("count1=%d\r\n",count1);
//		if(count1==1000)
//		{
//				stop1=1;
//		    printf("电机1方向1结束\r\n");
//		}
//		else if(dir1==2)
//		{
//			stop1=2;
//			printf("电机1方向2结束\r\n");
//		}
//	}
//}


///*********** Group B ***********/
//// 定时器5主模式
//void TIM5_GPIO_Config(uint16_t TIM5_Prescaler, uint16_t TIM5_Period, uint16_t CCR_B1)
//{
//	GPIO_InitTypeDef	GPIO_InitStructure;
//	// TIM5通道1\3
//	RCC_APB1PeriphClockCmd( RCC_APB1Periph_TIM5, ENABLE);
//	RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA, ENABLE);
//	
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;			// TIM5: CH1 - PA0, CH3 - PA2,
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;			// 复用推挽输出
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_Init(GPIOA, &GPIO_InitStructure);	

//	TIM_TimeBaseInitTypeDef		TIM_TimeBaseStructure;
//	// 时钟频率设置
//	TIM_TimeBaseStructure.TIM_Period = TIM5_Period - 1;
//	TIM_TimeBaseStructure.TIM_Prescaler = TIM5_Prescaler - 1;
//	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
//	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
//	TIM_TimeBaseInit( TIM5, &TIM_TimeBaseStructure);
//	
//	TIM_OCInitTypeDef		TIM_OCInitStructure;
//	// 设置工作模式
//	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;			// 设置工作模式是PWM，且为PWM1工作模式，TIMx_CNT<TIMx_CCR1时为高电平
//	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;		// 也就是使能PWM输出到端口					
//	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;			// 输出极性

//	// PWM通道，TIM5 - 通道1设置函数，50/100，脉冲信号
//	TIM_OCInitStructure.TIM_Pulse = CCR_B1;					// 设置待装入捕获寄存器的脉冲值
//	TIM_OC1Init( TIM5, &TIM_OCInitStructure);						// 初始化 TIM5-OC1
//	TIM_SelectMasterSlaveMode( TIM5, TIM_MasterSlaveMode_Enable);		// 定时器主从模式使能
//	TIM_SelectOutputTrigger( TIM5, TIM_TRGOSource_Update);						// 选择触发方式：使用更新事件作为触发输出
//	TIM_OC1PreloadConfig( TIM5, TIM_OCPreload_Enable);		// CH1预装载使能，修改	
//	
////	// PWM通道，TIM5 - 通道3设置函数，50/100，脉冲信号
////	TIM_OCInitStructure.TIM_Pulse = CCR_B3;					// 设置待装入捕获寄存器的脉冲值
////	TIM_OC3Init( TIM5, &TIM_OCInitStructure);						// 初始化 TIM5-OC3
////	TIM_SelectMasterSlaveMode( TIM5, TIM_MasterSlaveMode_Enable);		// 定时器主从模式使能
////	TIM_SelectOutputTrigger( TIM5, TIM_TRGOSource_Update);						// 选择触发方式：使用更新事件作为触发输出
////	TIM_OC3PreloadConfig( TIM5, TIM_OCPreload_Enable);		// CH3预装载使能，修改				

//	TIM_ARRPreloadConfig( TIM5, ENABLE);					// 使能ARR预装载寄存器
//}

////定时器8从模式
//void TIM8_GPIO_Config(u32 PulseNum_B)
//{
//	TIM_TimeBaseInitTypeDef	TIM_TimeBaseStructure;
//	NVIC_InitTypeDef	NVIC_InitStructure;
//	
//	RCC_APB2PeriphClockCmd( RCC_APB2Periph_TIM8, ENABLE);
//	
//	TIM_TimeBaseStructure.TIM_Period = PulseNum_B;
//	TIM_TimeBaseStructure.TIM_Prescaler = 0;
//	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
//	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
//	TIM_TimeBaseStructure.TIM_RepetitionCounter=0;
//	TIM_TimeBaseInit( TIM8, &TIM_TimeBaseStructure);
//	
//	TIM_SelectInputTrigger( TIM8, TIM_TS_ITR3);			// TIM5-主，TIM8-从 ITR3
//	TIM_SelectSlaveMode( TIM8, TIM_SlaveMode_External1);
//	TIM_ITConfig( TIM8, TIM_IT_Update, DISABLE);
//	
//	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3);
//	NVIC_InitStructure.NVIC_IRQChannel = TIM8_UP_IRQn;
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init( &NVIC_InitStructure);
//}


///*******B组（TIM5-TIM8）主从定时器PWM输出 *******/
//// Cycle_B = Preiod_B; PulseNum_B 输出脉冲个数; DIR_B1 DIR_B3 高/低电平-方向信号
//void PWM_Output_B(u16 Cycle_B, u32 PulseNum_B,uint16_t CCR_B1)
//{
//	TIM8_GPIO_Config(PulseNum_B);
//	TIM_Cmd( TIM8, ENABLE);
//	TIM_ClearITPendingBit( TIM8, TIM_IT_Update);
//	TIM_ITConfig( TIM8, TIM_IT_Update, ENABLE);
//	TIM5_GPIO_Config(72, Cycle_B,CCR_B1);
//	TIM_Cmd( TIM5, ENABLE);
//}


//// Group B	TIM5-TIM8主从定时器
//void TIM8_UP_IRQHandler(void)
//{
//	if (TIM_GetITStatus( TIM8, TIM_IT_Update) != RESET)
//	{
//		TIM_ClearITPendingBit( TIM8, TIM_IT_Update);			// 清除中断标志位
//		TIM_Cmd( TIM5, DISABLE);			// 关闭定时器5
//		TIM_Cmd( TIM8, DISABLE);			// 关闭定时器8
//		TIM_ITConfig( TIM8, TIM_IT_Update, DISABLE);
//		if(dir2==1)
//		{
//		stop2=1;
//		}
//		else if(dir2==2)
//		{
//			stop2=2;
//		}
//	}
//}


/////*********** Group C ***********/
////// 定时器3主模式
////void TIM3_GPIO_Config(uint16_t TIM3_Prescaler, uint16_t TIM3_Period, uint16_t CCR_C1, uint16_t CCR_C2,uint16_t CCR_C3,uint16_t CCR_C4)
////{
////	GPIO_InitTypeDef	GPIO_InitStructure;
////	// TIM3通道1\2
////	RCC_APB1PeriphClockCmd( RCC_APB1Periph_TIM3, ENABLE);
////	RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA, ENABLE);
////	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;	// TIM3_CH1 PA6, CH2 - PA7
////	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;			// 复用推挽输出
////	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
////	GPIO_Init(GPIOA, &GPIO_InitStructure);

////	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);	
////	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;			// TIM3_CH1 PA6, CH2 - PA7
////	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;			// 复用推挽输出
////	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
////	GPIO_Init(GPIOB, &GPIO_InitStructure);

////	TIM_TimeBaseInitTypeDef		TIM_TimeBaseStructure;
////	// 时钟频率设置
////	TIM_TimeBaseStructure.TIM_Period = TIM3_Period - 1;
////	TIM_TimeBaseStructure.TIM_Prescaler = TIM3_Prescaler - 1;
////	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
////	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
////	TIM_TimeBaseInit( TIM3, &TIM_TimeBaseStructure);
////	
////	TIM_OCInitTypeDef		TIM_OCInitStructure;
////	// 设置工作模式
////	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;			// 设置工作模式是PWM，且为PWM1工作模式，TIMx_CNT<TIMx_CCR1时为高电平
////	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;		// 也就是使能PWM输出到端口					
////	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;			// 输出极性

////	// PWM通道，TIM3 - 通道1设置函数，50/100
////	TIM_OCInitStructure.TIM_Pulse = CCR_C1 ;					// 设置待装入捕获寄存器的脉冲值
////	TIM_OC1Init( TIM3, &TIM_OCInitStructure);						// 初始化 TIM3-OC1
////	TIM_SelectMasterSlaveMode( TIM3, TIM_MasterSlaveMode_Enable);		// 定时器主从模式使能
////	TIM_SelectOutputTrigger( TIM3, TIM_TRGOSource_Update);						// 选择触发方式：使用更新事件作为触发输出
////	TIM_OC1PreloadConfig( TIM3, TIM_OCPreload_Enable);		// CH1预装载使能，修改
////	
////	// PWM通道，TIM3 - 通道2设置函数，100/100 or 0/100
////	TIM_OCInitStructure.TIM_Pulse = CCR_C2;									// 初始化 TIM3-OC2
////	TIM_OC2Init( TIM3, &TIM_OCInitStructure);						// CH2预装载使能，修改
////	TIM_SelectMasterSlaveMode( TIM3, TIM_MasterSlaveMode_Enable);		// 定时器主从模式使能
////	TIM_SelectOutputTrigger( TIM3, TIM_TRGOSource_Update);						// 选择触发方式：使用更新事件作为触发输出
////	TIM_OC2PreloadConfig( TIM3, TIM_OCPreload_Enable);
////	// PWM通道，TIM3 - 通道3设置函数，50/100
////	TIM_OCInitStructure.TIM_Pulse = CCR_C3 ;					// 设置待装入捕获寄存器的脉冲值
////	TIM_OC3Init( TIM3, &TIM_OCInitStructure);						// 初始化 TIM3-OC1
////	TIM_SelectMasterSlaveMode( TIM3, TIM_MasterSlaveMode_Enable);		// 定时器主从模式使能
////	TIM_SelectOutputTrigger( TIM3, TIM_TRGOSource_Update);						// 选择触发方式：使用更新事件作为触发输出
////	TIM_OC3PreloadConfig( TIM3, TIM_OCPreload_Enable);		// CH1预装载使能，修改				
////	// PWM通道，TIM3 - 通道4设置函数，50/100
////	TIM_OCInitStructure.TIM_Pulse = CCR_C4 ;					// 设置待装入捕获寄存器的脉冲值
////	TIM_OC4Init( TIM3, &TIM_OCInitStructure);						// 初始化 TIM3-OC1
////	TIM_SelectMasterSlaveMode( TIM3, TIM_MasterSlaveMode_Enable);		// 定时器主从模式使能
////	TIM_SelectOutputTrigger( TIM3, TIM_TRGOSource_Update);						// 选择触发方式：使用更新事件作为触发输出
////	TIM_OC4PreloadConfig( TIM3, TIM_OCPreload_Enable);		// CH1预装载使能，修改				

////	TIM_ARRPreloadConfig( TIM3, ENABLE);					// 使能ARR预装载寄存器
////}

////// 定时器4从模式
////void TIM4_GPIO_Config(u32 PulseNum_C)
////{
////	TIM_TimeBaseInitTypeDef	TIM_TimeBaseStructure;
////	NVIC_InitTypeDef	NVIC_InitStructure;
////	
////	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
////	
////	TIM_TimeBaseStructure.TIM_Period = PulseNum_C;
////	TIM_TimeBaseStructure.TIM_Prescaler = 0;
////	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
////	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
////	TIM_TimeBaseInit( TIM4, &TIM_TimeBaseStructure);
////	
////	TIM_SelectInputTrigger( TIM4, TIM_TS_ITR2);				// TIM3-主，TIM4-从 ITR2
////	TIM_SelectSlaveMode( TIM4,TIM_SlaveMode_External1 );		// 等同 TIM4->SMCR |= 0x07
////	TIM_ITConfig( TIM4, TIM_IT_Update, DISABLE);
////	
////	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3);
////	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
////	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
////	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
////	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
////	NVIC_Init(&NVIC_InitStructure);
////}

////// Cycle_C = Preiod_C; PulseNum_C 输出脉冲个数; DIR_C 高/低电平-方向信号
////void PWM_Output_C(u16 Cycle_C, u32 PulseNum_C, uint16_t CCR_C1, uint16_t CCR_C2,uint16_t CCR_C3,uint16_t CCR_C4)			// TIM3-主，TIM4-从
////{
////	TIM4_GPIO_Config(PulseNum_C);
////	TIM_Cmd( TIM4, ENABLE);
////	TIM_ClearITPendingBit( TIM4, TIM_IT_Update);
////	TIM_ITConfig( TIM4, TIM_IT_Update, ENABLE);
////	TIM3_GPIO_Config(72, Cycle_C, CCR_C1,CCR_C2,CCR_C3,CCR_C4);
////	TIM_Cmd( TIM3, ENABLE);
////}


////// Group C
////void TIM4_IRQHandler(void)
////{
////	if (TIM_GetITStatus( TIM4, TIM_IT_Update) != RESET)
////	{
////		TIM_ClearITPendingBit( TIM4, TIM_IT_Update);			// 清除中断标志位
////		TIM_Cmd( TIM3, DISABLE);			// 关闭定时器3
////		TIM_Cmd( TIM4, DISABLE);			// 关闭定时器4
////		TIM_ITConfig( TIM4, TIM_IT_Update, DISABLE);
////	}
////}



///******方向引脚定义初始化*******/
///**
//PD1-步进电机1方向
//PD3-步进电机1方向
//PD5-步进电机1方向
//PD7-步进电机1方向
//**/
//void direction_GPIO_Config(void)
//{		
//		/*定义一个GPIO_InitTypeDef类型的结构体*/
//		GPIO_InitTypeDef GPIO_InitStructure;

//		/*开启LED相关的GPIO外设时钟*/
//		RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOD, ENABLE);
//		/*选择要控制的GPIO引脚*/
//		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1|GPIO_Pin_3;	

//		/*设置引脚模式为通用推挽输出*/
//		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;   

//		/*设置引脚速率为50MHz */   
//		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 

//		/*调用库函数，初始化GPIO*/
//		GPIO_Init(GPIOD, &GPIO_InitStructure);
//		/*引脚全部拉高*/
//		GPIO_SetBits(GPIOD, GPIO_Pin_1|GPIO_Pin_3);
//}

