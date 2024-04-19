#ifndef __LOGIC_H
#define __LOGIC_H

#include	"stm32f10x.h"

void F4_Receive_Data(uint8_t *com_data);
void Crane_Init (void);
void transimt(void);
void bj_int(void);
void load_int(void);
void djyundong(void);
void bjload(void);
void bj_dis_change(void);
void judge_change(void);
void bjbushu(void);
void load_bjbushu(void);






#endif /*__LOGIC_H */

