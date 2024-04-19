#ifndef _AIRPUMP_H_
#define _AIRPUMP_H_

#include "stm32f10x.h"
				
/** the macro definition to trigger the airpump（or valve） on or off 
  * 1 - off
  *0 - on
  */
#define ON  1
#define OFF 0


/* 使用标准的固件库控制IO*/
#define airpump1(a)	if (a)	\
					GPIO_SetBits(GPIOC,GPIO_Pin_5);\
					else		\
					GPIO_ResetBits(GPIOC,GPIO_Pin_5)

#define airpump2(a)		if (a)	\
					GPIO_SetBits(GPIOB,GPIO_Pin_1);\
					else		\
					GPIO_ResetBits(GPIOB,GPIO_Pin_1)
					
#define airpump3(a)		if (a)	\
					GPIO_SetBits(GPIOB,GPIO_Pin_2);\
					else		\
					GPIO_ResetBits(GPIOB,GPIO_Pin_2)
#define airpump4(a)		if (a)	\
					GPIO_SetBits(GPIOC,GPIO_Pin_4);\
					else		\
					GPIO_ResetBits(GPIOC,GPIO_Pin_4)

/*****************函数声明**************************/
void EleRelay_GPIO_Config(void);




#endif

