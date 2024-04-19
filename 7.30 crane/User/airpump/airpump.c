#include "airpump.h"
 /**
  * @brief  初始化控制继电器的IO
  * @param  无
  * @retval 无
  */
void EleRelay_GPIO_Config(void)
{		
		/*定义一个GPIO_InitTypeDef类型的结构体*/
		GPIO_InitTypeDef GPIO_InitStructure;

		/*开启LED相关的GPIO外设时钟*/
		RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOB, ENABLE);
		/*选择要控制的GPIO引脚*/
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2;	

		/*设置引脚模式为通用推挽输出*/
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;   

		/*设置引脚速率为50MHz */   
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 

		/*调用库函数，初始化GPIO*/
		GPIO_Init(GPIOB, &GPIO_InitStructure);	

			/*开启LED相关的GPIO外设时钟*/
		RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOC, ENABLE);
		/*选择要控制的GPIO引脚*/
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_5;	
		/*调用库函数，初始化GPIO*/
		GPIO_Init(GPIOC, &GPIO_InitStructure);	

		/* 置低气泵1电平	*/
		GPIO_ResetBits(GPIOC, GPIO_Pin_5);
		/* 置低气泵2电平	*/
		GPIO_ResetBits(GPIOB, GPIO_Pin_1);
		/* 置低气泵3电平	*/
		GPIO_ResetBits(GPIOB, GPIO_Pin_2);	 
		/* 置低气泵4电平	*/
		GPIO_ResetBits(GPIOC, GPIO_Pin_4);	 
}


