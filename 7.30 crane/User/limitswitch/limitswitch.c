#include "stm32f10x.h"                  // Device header
#include "limitswitch.h"
#include "bsp_usart.h"

//extern int	 switch1;
//extern int	 switch2;

//void limitswitch_Init(void)
//{
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
//	
//	GPIO_InitTypeDef GPIO_InitStructure;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13|GPIO_Pin_14;
////	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_Init(GPIOB, &GPIO_InitStructure);
//	
//	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource13);
//	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource14);

//	/* 置低气泵1电平	*/
//	GPIO_ResetBits(GPIOB, GPIO_Pin_13);
//	/* 置低气泵2电平	*/
//	GPIO_ResetBits(GPIOB, GPIO_Pin_14);

//	EXTI_InitTypeDef EXTI_InitStructure;
//	EXTI_InitStructure.EXTI_Line = EXTI_Line13|EXTI_Line14;
//	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
//	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;//中断事件
//	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;//上升沿触发
//	EXTI_Init(&EXTI_InitStructure);
//	
//	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
//	
//	NVIC_InitTypeDef NVIC_InitStructure;
//	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
//	NVIC_Init(&NVIC_InitStructure);
//}

//void EXTI15_10_IRQHandler(void)
//{
//	if (EXTI_GetITStatus(EXTI_Line13) == SET)
//	{
//			/*如果出现数据乱跳的现象，可再次判断引脚电平，以避免抖动*/
//			if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_13) == 1)
//			{
//				if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_13) == 1)
//				{
//					switch1=1;
////					printf("switch1:on\r\n");
//				}
//			}
//			EXTI_ClearITPendingBit(EXTI_Line13);
////			EXTI->IMR &= ~(EXTI_Line13);
//	}
//	
//	if (EXTI_GetITStatus(EXTI_Line14) == SET)
//	{
//		/*如果出现数据乱跳的现象，可再次判断引脚电平，以避免抖动*/
//			if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_14) == 1)
//			{
//							if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_14) == 1)
//							{		switch2=1;
////									printf("switch2:on\r\n");
//							}
//			}
//			EXTI_ClearITPendingBit(EXTI_Line14);
////			EXTI->IMR &= ~(EXTI_Line14);
//	}
//}
///********注：库函数没有现成的函数打开和关闭外部中断********/
///*
//在某些时候我们希望暂时的关闭某条外部中断，以免造成误触发，此时可以使用下面的操作：
//关闭：EXTI->IMR &= ~(EXTI_Linex);
//开启：EXTI->IMR |= EXTI_Linex;
//其中EXTI_Linex为指定的中断线。*/

