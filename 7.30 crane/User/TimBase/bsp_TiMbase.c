// 基本定时器TIMx,x[6,7]定时初始化函数

#include "bsp_TiMbase.h" 


/*
 * 注意：TIM_TimeBaseInitTypeDef结构体里面有5个成员，TIM6和TIM7的寄存器里面只有
 * TIM_Prescaler和TIM_Period，所以使用TIM6和TIM7的时候只需初始化这两个成员即可，
 * 另外三个成员是通用定时器和高级定时器才有.
 *-----------------------------------------------------------------------------
 *typedef struct
 *{ TIM_Prescaler            都有
 *	TIM_CounterMode			     TIMx,x[6,7]没有，其他都有
 *  TIM_Period               都有
 *  TIM_ClockDivision        TIMx,x[6,7]没有，其他都有
 *  TIM_RepetitionCounter    TIMx,x[1,8,15,16,17]才有
 *}TIM_TimeBaseInitTypeDef; 
 *-----------------------------------------------------------------------------
 */

void TIM6_Init(void)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
		
		// 开启定时器时钟,即内部时钟CK_INT=72M
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);
	
		// 自动重装载寄存器的值，累计TIM_Period+1个频率后产生一个更新或者中断
    TIM_TimeBaseStructure.TIM_Period = 50000-1;	

	  // 时钟预分频数为
    TIM_TimeBaseStructure.TIM_Prescaler= 72-1;
	
	  // 初始化定时器
    TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStructure);
}


//微秒级延时
void TIM6_Delayus(u16 xus)
{
	TIM_Cmd(TIM6,ENABLE); //启动定时器
	while(TIM6->CNT < xus);
	TIM6->CNT = 0;
	TIM_Cmd(TIM6,DISABLE); //关闭定时器
}
//毫秒级延时
void TIM6_Delayms(u16 xms)
{
	int i;
	for(i=0;i<xms;i++)
	{
		TIM6_Delayus(1000);
	}
}
/*********************************************END OF FILE**********************/
