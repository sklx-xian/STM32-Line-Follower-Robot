#ifndef __MOTOR_H
#define __MOTOR_H

#include "main.h"
//1.Initialize pwm
void Motor_Init(void);
//2.Set the speed of the left and right wheels
void Motor_SetSpeed(int left_speed,int right_speed);
#endif
