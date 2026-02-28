#ifndef __SERVO_H
#define __SERVO_H

#include "main.h"
//1.initialize PWM
void Servo_Init(void);
//2.set angle
void Servo_SetAngle(uint8_t angle);
//3.Encapsulation of quick action
void Servo_LookForward(void);
void Servo_LookLeft(void);
void Servo_LookRight(void);
#endif
