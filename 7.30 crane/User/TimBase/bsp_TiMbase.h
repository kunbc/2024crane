#ifndef __BSP_TIMEBASE_H
#define __BSP_TIMEBASE_H


#include "stm32f10x.h"

/**************************º¯ÊýÉùÃ÷********************************/

void TIM6_Init(void);
void TIM6_Delayus(u16 xus);
void TIM6_Delayms(u16 xms);

#endif	/* __BSP_TIMEBASE_H */


