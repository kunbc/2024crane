#include "timer.h"
#include "bsp_usart.h"

extern uint8_t stop1;
extern uint8_t stop2;
extern uint8_t dir1,dir2;
extern int count1;
///*********** Group A	TIM1-TIM2����***********/
//// ��ʱ��1��ģʽ
//void TIM1_GPIO_Config(uint16_t TIM1_Prescaler, uint16_t TIM1_Period, uint16_t CCR_A1)
//{
//	GPIO_InitTypeDef	GPIO_InitStructure;
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
//	// TIM1ͨ��1\4
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;		// TIM1_CH1 - PA8�� CH4 - PA11
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;			// �����������
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_Init(GPIOA, &GPIO_InitStructure);
//	
//	TIM_TimeBaseInitTypeDef		TIM_TimeBaseStructure;
//	TIM_TimeBaseStructure.TIM_Period = TIM1_Period - 1;
//	TIM_TimeBaseStructure.TIM_Prescaler = TIM1_Prescaler - 1;
//	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
//	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
//	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;			// �ظ�������һ��Ҫ = 0; �߼���ʱ��TIM1,TIM8���������С�
//	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
//	
//	TIM_OCInitTypeDef		TIM_OCInitStructure;
//	// ���ù���ģʽ
//	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;							// PWM1
//	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;	// �Ƚ����ʹ��
//	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;			// �������

//	// PWMͨ����TIM1 - ͨ��1���ú�����50/100�������ź�
//	TIM_OCInitStructure.TIM_Pulse = CCR_A1;								// ���ô�װ�벶��Ĵ���������ֵ
//	TIM_OC1Init( TIM1, &TIM_OCInitStructure);							// CH1Ԥװ��ʹ�ܣ��޸�
//	TIM_SelectMasterSlaveMode( TIM1, TIM_MasterSlaveMode_Enable);		// ����ģʽ�£���Ϊ����ʱ��ʹ��
//	TIM_SelectOutputTrigger( TIM1, TIM_TRGOSource_Update);		// TIM - EGR�Ĵ����� ����UGλ�����������¼�
//	TIM_OC1PreloadConfig( TIM1, TIM_OCPreload_Enable);

////	// PWMͨ����TIM1 - ͨ��4���ú�����50/100�������ź�
////	TIM_OCInitStructure.TIM_Pulse = CCR_A4;							// ��ʼ�� TIM1-OC4
////	TIM_OC4Init( TIM1, &TIM_OCInitStructure);						// CH4Ԥװ��ʹ�ܣ��޸�
////	TIM_SelectMasterSlaveMode( TIM1, TIM_MasterSlaveMode_Enable);		// ����ģʽ�£���Ϊ����ʱ��ʹ��
////	TIM_SelectOutputTrigger( TIM1, TIM_TRGOSource_Update);		// TIM - EGR�Ĵ����� ����UGλ�����������¼�
////	TIM_OC4PreloadConfig( TIM1, TIM_OCPreload_Enable);
//	
//	TIM_ARRPreloadConfig(TIM1, ENABLE);				// Ԥװ��ʹ��	
//}

////��ʱ��2��ģʽ
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
//	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;			// ���ϼ���
//	TIM_TimeBaseInit( TIM2, &TIM_TimeBaseStructure);
//	
//	TIM_SelectInputTrigger( TIM2, TIM_TS_ITR0);			// TIM1-����TIM2-�ӣ����ж�Ӧ ITR0
//	TIM_SelectSlaveMode( TIM2, TIM_SlaveMode_External1);		//����ģʽ����Ϊ�Ӷ�ʱ��ʹ��
//	TIM_ITConfig( TIM2, TIM_IT_Update, DISABLE);
//	
//	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3);
//	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init( &NVIC_InitStructure);
//}


///*******A�飨TIM1-TIM2�����Ӷ�ʱ��PWM��� *******/
//// Cycle_A = Preiod_A; PulseNum_A ����������;DIR_A1 DIR_A2 ��/�͵�ƽ-�����ź�
//void PWM_Output_A(u16 Cycle_A, u32 PulseNum_A,uint16_t CCR_A1)			// TIM1-����TIM2-�ӣ����PWM
//{
//	TIM2_GPIO_Config(PulseNum_A);
//	TIM_Cmd( TIM2, ENABLE);
//	TIM_ClearITPendingBit( TIM2, TIM_IT_Update);
//	TIM_ITConfig( TIM2, TIM_IT_Update, ENABLE);
//	TIM1_GPIO_Config(72, Cycle_A, CCR_A1);	// 72M / 72 = 1MHz;	
//	TIM_Cmd( TIM1, ENABLE);
//	TIM_CtrlPWMOutputs( TIM1, ENABLE);		// �߼���ʱ�������У�ʹ�������
//}
///*******�жϺ���*******/
//// Group A	TIM1-TIM2���Ӷ�ʱ��
//void TIM2_IRQHandler(void)
//{
//	if (TIM_GetITStatus( TIM2, TIM_IT_Update) != RESET)
//	{
//		TIM_ClearITPendingBit( TIM2, TIM_IT_Update);			// ����жϱ�־λ
//		TIM_CtrlPWMOutputs( TIM1, DISABLE);		// �����ʹ�ܹرգ��߼���ʱ��������
//		TIM_Cmd( TIM1, DISABLE);			// �رն�ʱ��1
//		TIM_Cmd( TIM2, DISABLE);			// �رն�ʱ��2
//		TIM_ITConfig( TIM2, TIM_IT_Update, DISABLE);
//		count1++;
//		printf("count1=%d\r\n",count1);
//		if(count1==1000)
//		{
//				stop1=1;
//		    printf("���1����1����\r\n");
//		}
//		else if(dir1==2)
//		{
//			stop1=2;
//			printf("���1����2����\r\n");
//		}
//	}
//}


///*********** Group B ***********/
//// ��ʱ��5��ģʽ
//void TIM5_GPIO_Config(uint16_t TIM5_Prescaler, uint16_t TIM5_Period, uint16_t CCR_B1)
//{
//	GPIO_InitTypeDef	GPIO_InitStructure;
//	// TIM5ͨ��1\3
//	RCC_APB1PeriphClockCmd( RCC_APB1Periph_TIM5, ENABLE);
//	RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA, ENABLE);
//	
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;			// TIM5: CH1 - PA0, CH3 - PA2,
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;			// �����������
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_Init(GPIOA, &GPIO_InitStructure);	

//	TIM_TimeBaseInitTypeDef		TIM_TimeBaseStructure;
//	// ʱ��Ƶ������
//	TIM_TimeBaseStructure.TIM_Period = TIM5_Period - 1;
//	TIM_TimeBaseStructure.TIM_Prescaler = TIM5_Prescaler - 1;
//	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
//	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
//	TIM_TimeBaseInit( TIM5, &TIM_TimeBaseStructure);
//	
//	TIM_OCInitTypeDef		TIM_OCInitStructure;
//	// ���ù���ģʽ
//	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;			// ���ù���ģʽ��PWM����ΪPWM1����ģʽ��TIMx_CNT<TIMx_CCR1ʱΪ�ߵ�ƽ
//	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;		// Ҳ����ʹ��PWM������˿�					
//	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;			// �������

//	// PWMͨ����TIM5 - ͨ��1���ú�����50/100�������ź�
//	TIM_OCInitStructure.TIM_Pulse = CCR_B1;					// ���ô�װ�벶��Ĵ���������ֵ
//	TIM_OC1Init( TIM5, &TIM_OCInitStructure);						// ��ʼ�� TIM5-OC1
//	TIM_SelectMasterSlaveMode( TIM5, TIM_MasterSlaveMode_Enable);		// ��ʱ������ģʽʹ��
//	TIM_SelectOutputTrigger( TIM5, TIM_TRGOSource_Update);						// ѡ�񴥷���ʽ��ʹ�ø����¼���Ϊ�������
//	TIM_OC1PreloadConfig( TIM5, TIM_OCPreload_Enable);		// CH1Ԥװ��ʹ�ܣ��޸�	
//	
////	// PWMͨ����TIM5 - ͨ��3���ú�����50/100�������ź�
////	TIM_OCInitStructure.TIM_Pulse = CCR_B3;					// ���ô�װ�벶��Ĵ���������ֵ
////	TIM_OC3Init( TIM5, &TIM_OCInitStructure);						// ��ʼ�� TIM5-OC3
////	TIM_SelectMasterSlaveMode( TIM5, TIM_MasterSlaveMode_Enable);		// ��ʱ������ģʽʹ��
////	TIM_SelectOutputTrigger( TIM5, TIM_TRGOSource_Update);						// ѡ�񴥷���ʽ��ʹ�ø����¼���Ϊ�������
////	TIM_OC3PreloadConfig( TIM5, TIM_OCPreload_Enable);		// CH3Ԥװ��ʹ�ܣ��޸�				

//	TIM_ARRPreloadConfig( TIM5, ENABLE);					// ʹ��ARRԤװ�ؼĴ���
//}

////��ʱ��8��ģʽ
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
//	TIM_SelectInputTrigger( TIM8, TIM_TS_ITR3);			// TIM5-����TIM8-�� ITR3
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


///*******B�飨TIM5-TIM8�����Ӷ�ʱ��PWM��� *******/
//// Cycle_B = Preiod_B; PulseNum_B ����������; DIR_B1 DIR_B3 ��/�͵�ƽ-�����ź�
//void PWM_Output_B(u16 Cycle_B, u32 PulseNum_B,uint16_t CCR_B1)
//{
//	TIM8_GPIO_Config(PulseNum_B);
//	TIM_Cmd( TIM8, ENABLE);
//	TIM_ClearITPendingBit( TIM8, TIM_IT_Update);
//	TIM_ITConfig( TIM8, TIM_IT_Update, ENABLE);
//	TIM5_GPIO_Config(72, Cycle_B,CCR_B1);
//	TIM_Cmd( TIM5, ENABLE);
//}


//// Group B	TIM5-TIM8���Ӷ�ʱ��
//void TIM8_UP_IRQHandler(void)
//{
//	if (TIM_GetITStatus( TIM8, TIM_IT_Update) != RESET)
//	{
//		TIM_ClearITPendingBit( TIM8, TIM_IT_Update);			// ����жϱ�־λ
//		TIM_Cmd( TIM5, DISABLE);			// �رն�ʱ��5
//		TIM_Cmd( TIM8, DISABLE);			// �رն�ʱ��8
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
////// ��ʱ��3��ģʽ
////void TIM3_GPIO_Config(uint16_t TIM3_Prescaler, uint16_t TIM3_Period, uint16_t CCR_C1, uint16_t CCR_C2,uint16_t CCR_C3,uint16_t CCR_C4)
////{
////	GPIO_InitTypeDef	GPIO_InitStructure;
////	// TIM3ͨ��1\2
////	RCC_APB1PeriphClockCmd( RCC_APB1Periph_TIM3, ENABLE);
////	RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA, ENABLE);
////	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;	// TIM3_CH1 PA6, CH2 - PA7
////	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;			// �����������
////	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
////	GPIO_Init(GPIOA, &GPIO_InitStructure);

////	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);	
////	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;			// TIM3_CH1 PA6, CH2 - PA7
////	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;			// �����������
////	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
////	GPIO_Init(GPIOB, &GPIO_InitStructure);

////	TIM_TimeBaseInitTypeDef		TIM_TimeBaseStructure;
////	// ʱ��Ƶ������
////	TIM_TimeBaseStructure.TIM_Period = TIM3_Period - 1;
////	TIM_TimeBaseStructure.TIM_Prescaler = TIM3_Prescaler - 1;
////	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
////	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
////	TIM_TimeBaseInit( TIM3, &TIM_TimeBaseStructure);
////	
////	TIM_OCInitTypeDef		TIM_OCInitStructure;
////	// ���ù���ģʽ
////	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;			// ���ù���ģʽ��PWM����ΪPWM1����ģʽ��TIMx_CNT<TIMx_CCR1ʱΪ�ߵ�ƽ
////	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;		// Ҳ����ʹ��PWM������˿�					
////	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;			// �������

////	// PWMͨ����TIM3 - ͨ��1���ú�����50/100
////	TIM_OCInitStructure.TIM_Pulse = CCR_C1 ;					// ���ô�װ�벶��Ĵ���������ֵ
////	TIM_OC1Init( TIM3, &TIM_OCInitStructure);						// ��ʼ�� TIM3-OC1
////	TIM_SelectMasterSlaveMode( TIM3, TIM_MasterSlaveMode_Enable);		// ��ʱ������ģʽʹ��
////	TIM_SelectOutputTrigger( TIM3, TIM_TRGOSource_Update);						// ѡ�񴥷���ʽ��ʹ�ø����¼���Ϊ�������
////	TIM_OC1PreloadConfig( TIM3, TIM_OCPreload_Enable);		// CH1Ԥװ��ʹ�ܣ��޸�
////	
////	// PWMͨ����TIM3 - ͨ��2���ú�����100/100 or 0/100
////	TIM_OCInitStructure.TIM_Pulse = CCR_C2;									// ��ʼ�� TIM3-OC2
////	TIM_OC2Init( TIM3, &TIM_OCInitStructure);						// CH2Ԥװ��ʹ�ܣ��޸�
////	TIM_SelectMasterSlaveMode( TIM3, TIM_MasterSlaveMode_Enable);		// ��ʱ������ģʽʹ��
////	TIM_SelectOutputTrigger( TIM3, TIM_TRGOSource_Update);						// ѡ�񴥷���ʽ��ʹ�ø����¼���Ϊ�������
////	TIM_OC2PreloadConfig( TIM3, TIM_OCPreload_Enable);
////	// PWMͨ����TIM3 - ͨ��3���ú�����50/100
////	TIM_OCInitStructure.TIM_Pulse = CCR_C3 ;					// ���ô�װ�벶��Ĵ���������ֵ
////	TIM_OC3Init( TIM3, &TIM_OCInitStructure);						// ��ʼ�� TIM3-OC1
////	TIM_SelectMasterSlaveMode( TIM3, TIM_MasterSlaveMode_Enable);		// ��ʱ������ģʽʹ��
////	TIM_SelectOutputTrigger( TIM3, TIM_TRGOSource_Update);						// ѡ�񴥷���ʽ��ʹ�ø����¼���Ϊ�������
////	TIM_OC3PreloadConfig( TIM3, TIM_OCPreload_Enable);		// CH1Ԥװ��ʹ�ܣ��޸�				
////	// PWMͨ����TIM3 - ͨ��4���ú�����50/100
////	TIM_OCInitStructure.TIM_Pulse = CCR_C4 ;					// ���ô�װ�벶��Ĵ���������ֵ
////	TIM_OC4Init( TIM3, &TIM_OCInitStructure);						// ��ʼ�� TIM3-OC1
////	TIM_SelectMasterSlaveMode( TIM3, TIM_MasterSlaveMode_Enable);		// ��ʱ������ģʽʹ��
////	TIM_SelectOutputTrigger( TIM3, TIM_TRGOSource_Update);						// ѡ�񴥷���ʽ��ʹ�ø����¼���Ϊ�������
////	TIM_OC4PreloadConfig( TIM3, TIM_OCPreload_Enable);		// CH1Ԥװ��ʹ�ܣ��޸�				

////	TIM_ARRPreloadConfig( TIM3, ENABLE);					// ʹ��ARRԤװ�ؼĴ���
////}

////// ��ʱ��4��ģʽ
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
////	TIM_SelectInputTrigger( TIM4, TIM_TS_ITR2);				// TIM3-����TIM4-�� ITR2
////	TIM_SelectSlaveMode( TIM4,TIM_SlaveMode_External1 );		// ��ͬ TIM4->SMCR |= 0x07
////	TIM_ITConfig( TIM4, TIM_IT_Update, DISABLE);
////	
////	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3);
////	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
////	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
////	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
////	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
////	NVIC_Init(&NVIC_InitStructure);
////}

////// Cycle_C = Preiod_C; PulseNum_C ����������; DIR_C ��/�͵�ƽ-�����ź�
////void PWM_Output_C(u16 Cycle_C, u32 PulseNum_C, uint16_t CCR_C1, uint16_t CCR_C2,uint16_t CCR_C3,uint16_t CCR_C4)			// TIM3-����TIM4-��
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
////		TIM_ClearITPendingBit( TIM4, TIM_IT_Update);			// ����жϱ�־λ
////		TIM_Cmd( TIM3, DISABLE);			// �رն�ʱ��3
////		TIM_Cmd( TIM4, DISABLE);			// �رն�ʱ��4
////		TIM_ITConfig( TIM4, TIM_IT_Update, DISABLE);
////	}
////}



///******�������Ŷ����ʼ��*******/
///**
//PD1-�������1����
//PD3-�������1����
//PD5-�������1����
//PD7-�������1����
//**/
//void direction_GPIO_Config(void)
//{		
//		/*����һ��GPIO_InitTypeDef���͵Ľṹ��*/
//		GPIO_InitTypeDef GPIO_InitStructure;

//		/*����LED��ص�GPIO����ʱ��*/
//		RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOD, ENABLE);
//		/*ѡ��Ҫ���Ƶ�GPIO����*/
//		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1|GPIO_Pin_3;	

//		/*��������ģʽΪͨ���������*/
//		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;   

//		/*������������Ϊ50MHz */   
//		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 

//		/*���ÿ⺯������ʼ��GPIO*/
//		GPIO_Init(GPIOD, &GPIO_InitStructure);
//		/*����ȫ������*/
//		GPIO_SetBits(GPIOD, GPIO_Pin_1|GPIO_Pin_3);
//}

