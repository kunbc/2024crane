#ifndef __SERVO_H__
#define __SERVO_H__


#include "main.h"

void steer1_init(double Start_Angle);
void steer2_init(double Start_Angle);

void steer(double target_angle,int Servo_num);

#define SERVO_1         1         /* ¶æ»ú½Ó¿ÚÐòºÅ */
#define SERVO_2         2
#define SERVO_ALL       3








#endif /* __SERVO_H__ */
