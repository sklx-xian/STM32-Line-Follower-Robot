#include "Motor.h"
#include "tim.h"
#include <stdlib.h>

void Motor_Init(void)
{
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
}
void Motor_SetSpeed(int left_speed,int right_speed)
{
//1.limited amplitude protection
		if (left_speed > 1000)  left_speed = 1000;
		if (left_speed < -1000) left_speed = -1000;
		if (right_speed > 1000)  right_speed = 1000;
		if (right_speed < -1000) right_speed = -1000;
//2.left
if (left_speed >= 0) 
    {
        HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,GPIO_PIN_RESET);
    }	
else
{
        HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,GPIO_PIN_SET);		 

}
__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,abs(left_speed));
if (right_speed >= 0) 
    {
      
        HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET);
    }
    else 
    {
        
        HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET);
    }
__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, abs(right_speed));
}
