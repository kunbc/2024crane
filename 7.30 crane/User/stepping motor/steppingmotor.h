#ifndef __STEPPINGMOTOR_H
#define __STEPPINGMOTOR_H

#include "stm32f10x.h"
#include "timer.h"

/*电机控制函数定义*/
void TIM5_Init(void);
void TIM5_GPIO_Config(void);
void TIM5_Mode_Config(void);

void TIM2_Init(void);
void TIM2_GPIO_Config(void);
void TIM2_Mode_Config(void);

void direction_GPIO_Config(void);





//void steppingmotor3(u16 Cycle_B, u32 PulseNum_B,uint16_t DIR);
//void steppingmotor4(u16 Cycle_B, u32 PulseNum_B,uint16_t DIR);




#endif	/* __STEPPINGMOTOR_H */
