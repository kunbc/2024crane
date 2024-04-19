#ifndef __TIMER_H
#define __TIMER_H

#include	"stm32f10x.h"

/* Group A: TIM1 - 主定时器，TIM2 - 从定时器
	 TIM1: CH1 - PA8, CH4 - PA11   RCC_APB2Periph_TIM1
   TIM2: RCC_APB1Periph_TIM2
	 
	 Group B: TIM5 - 主定时器，TIM8 - 从定时器
	 TIM5: CH1 - PA0, CH3 - PA2   RCC_APB1Periph_TIM5
   TIM8: RCC_APB2Periph_TIM8
	 
	 Group C: TIM3 - 主定时器，TIM4 - 从定时器
	 TIM3: CH1 - PA6, CH2 - PA7    RCC_APB1Periph_TIM3
   TIM4: RCC_APB1Periph_TIM4	
*/


/*****方向引脚初始化函数*****/
void direction_GPIO_Config(void);

/*****定时器初始化函数*****/
//// Group A
//void TIM1_GPIO_Config(uint16_t TIM1_Prescaler, uint16_t TIM1_Period, uint16_t CCR_A1);
//void TIM2_GPIO_Config(u32 PulseNum_A);
//void PWM_Output_A(u16 Cycle_A, u32 PulseNum_A,uint16_t CCR_A1);
//void TIM2_IRQHandler(void);

//// Group B
//void TIM5_GPIO_Config(uint16_t TIM5_Prescaler, uint16_t TIM5_Period, uint16_t CCR_B1);
//void TIM8_GPIO_Config(u32 PulseNum_B);
//void PWM_Output_B(u16 Cycle_B, u32 PulseNum_B,uint16_t CCR_B1);
//void TIM8_UP_IRQHandler(void);

//// Group C
//void TIM3_GPIO_Config(uint16_t TIM3_Prescaler, uint16_t TIM3_Period, uint16_t CCR_C1, uint16_t CCR_C2,uint16_t CCR_C3,uint16_t CCR_C4);
//void TIM4_GPIO_Config(u32 PulseNum_C);
//void PWM_Output_C(u16 Cycle_C, u32 PulseNum_C, uint16_t CCR_C1, uint16_t CCR_C2,uint16_t CCR_C3,uint16_t CCR_C4);
//void TIM4_IRQHandler(void);

#endif	/* __TIMER_H */

