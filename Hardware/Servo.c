#include "Servo.h"
#include "tim.h"
//1.initialize servo,open PWM
void Servo_Init(void)
{
HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
	
Servo_LookForward();
}
//2.define the angle of servo
void Servo_SetAngle(uint8_t angle)
{
if(angle>180)
{
angle=180;
}
uint16_t compare_value = 500 + ((uint32_t)angle * 2000) / 180;

__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,compare_value);

}	
void Servo_LookForward(void)
{
    Servo_SetAngle(90);
}
void Servo_LookLeft(void)
{
    Servo_SetAngle(160); 
}
void Servo_LookRight(void)
{
    Servo_SetAngle(20);  
}
