/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "OLED.h"
#include "Servo.h"
#include "Sensor.h"
#include  "Motor.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
int left_speed;
int right_spped;
uint8_t sharp_turn_mode=0;
int active_num=0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
int Calculate_Error(uint8_t sensor_val);
void Motor_Init(void);
void Motor_SetSpeed(int left_seepd,int right_speed);
void Servo_Init(void);
void Obstacle_Avoide(void);
//float HCSR04_GetDistance(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int Calculate_Error(uint8_t sensor_val)
{
    sensor_val &= 0x7E;	
    if(sensor_val==0) return 9999;

    long pos=0;
    int active=0;	

    if(sensor_val & 0x40) {pos += 1000; active++;} 
    if(sensor_val & 0x20) {pos += 2000; active++;}
    if(sensor_val & 0x10) {pos += 3000; active++;}
    if(sensor_val & 0x08) {pos += 4000; active++;}
    if(sensor_val & 0x04) {pos += 5000; active++;}
    if(sensor_val & 0x02) {pos += 6000; active++;}
    int position = pos / active;
	active_num=active;
    return position - 3500;	
}
//=============================================================
// 1. PID parameters 
float Kp = 0.3f;   
float Ki = 0.0f;    
float Kd = 0.8f;    

// 2. procedure variable 
int last_error = 0;

// 3. Base Speed
int BaseSpeed = 350;

void PID_Track_Control(int current_error)  
{
   
    static int error_history[5] = {0,0,0,0,0};//此处的static关键字改变了数组error_history的生命周期，并且使其只被初始化一次
    static uint16_t lost_line_timer = 0;
        
    if(current_error != 9999) 
    {
        lost_line_timer = 0;
    }
   
//1.1 trigger a right-angle turn
if(current_error >= 2000 || (active_num>= 3 && current_error >= 1000)) 
    { 
        sharp_turn_mode = 1; 
        return; //right
    }
    if(current_error <= -2000 || (active_num>= 3 && current_error <= -1000)) 
    { 
        sharp_turn_mode = 2; 
        return; //left
    }	
	
//1.2 
	if(sharp_turn_mode == 1)
    {
        Motor_SetSpeed(350, -350);//dead zone lock
       
        if(current_error >= -500 && current_error <= 500 && current_error != 9999)
        {
            sharp_turn_mode = 0;//dead zone unlock
            for(int i=0; i<5; i++) error_history[i] = 0; 
        }           
        return;
    }
    else if(sharp_turn_mode == 2)
    {
        Motor_SetSpeed(-350, 350);
        if (current_error >= -500 && current_error <= 500 && current_error != 9999)
        {
            sharp_turn_mode = 0;
            for(int i=0; i<5; i++) error_history[i] = 0; 
        }   
        return;
    }
//2.software watchdog
    if(current_error == 9999) 
    {
        lost_line_timer++; 

        if(lost_line_timer < 15) 
        {
            
            if(last_error <= -1500) Motor_SetSpeed(100,300); 
            else if(last_error >= 1500) Motor_SetSpeed(300,100);
            else Motor_SetSpeed(0, 0);
        }
        else 
        {
            if(last_error <= -1500) Motor_SetSpeed(-350, 350);
            else if(last_error >= 1500) Motor_SetSpeed(350, -350);
            else Motor_SetSpeed(0, 0);
        }

        for(int i=0; i<5; i++) error_history[i] = 0;
        return;         
    }
	
// 3.reduce speed when making a sharp turn
    int current_base_speed = BaseSpeed; 
    if(current_error >= 2500 || current_error <= -2500) 
	{
        current_base_speed = 50; 
    }       
    else if(current_error <= -1500 || current_error >= 1500) 
	{ 
        current_base_speed = 220;
    }
//4.
    for(int i = 4; i > 0; i--) 
	{
        error_history[i] = error_history[i-1];
    }
    error_history[0] = current_error;

    int window_sum = 0;
    for(int i = 0; i < 5; i++) 
	{
        window_sum += error_history[i];//sliding‑window integration
    }

    float P = Kp * current_error;
    float I = Ki * window_sum;            
    float slope = current_error - last_error;
    float D = Kd * slope;

    float PID_output = P + I + D;

    int left_motor_speed  = current_base_speed + (int)PID_output;
    int right_motor_speed = current_base_speed - (int)PID_output;

    if (left_motor_speed > 1000)  left_motor_speed = 1000;
    if (left_motor_speed < -1000) left_motor_speed = -1000;
    if (right_motor_speed > 1000) right_motor_speed = 1000;
    if (right_motor_speed < -1000) right_motor_speed = -1000;

    Motor_SetSpeed(left_motor_speed, right_motor_speed);

    last_error = current_error;
}


//HC-SRO4+avoidance

float HCSR04_GetDistance(void)
{
__HAL_TIM_SET_COUNTER(&htim2,0);
	
__HAL_TIM_CLEAR_FLAG(&htim2,TIM_FLAG_CC1);	
__HAL_TIM_CLEAR_FLAG(&htim2,TIM_FLAG_CC2);
	
HAL_TIM_IC_Start(&htim2,TIM_CHANNEL_1);
HAL_TIM_IC_Start(&htim2,TIM_CHANNEL_2);	
	
HAL_GPIO_WritePin(GPIOA,GPIO_PIN_10,GPIO_PIN_SET);
for(volatile uint32_t i=0; i<1500; i++);
HAL_GPIO_WritePin(GPIOA,GPIO_PIN_10,GPIO_PIN_RESET);
	
uint8_t if_success=0;
uint32_t expiretime=HAL_GetTick()+50;
	
while(expiretime>HAL_GetTick())	
{
uint8_t cc1flag=__HAL_TIM_GET_FLAG(&htim2,TIM_FLAG_CC1);
uint8_t cc2flag=__HAL_TIM_GET_FLAG(&htim2,TIM_FLAG_CC2);

if(cc1flag&&cc2flag)
{
if_success=1;
break;	
}
}	
HAL_TIM_IC_Stop(&htim2,TIM_CHANNEL_1);
HAL_TIM_IC_Stop(&htim2,TIM_CHANNEL_2);

if(if_success)
{
uint16_t ccr1=__HAL_TIM_GET_COMPARE(&htim2,TIM_CHANNEL_1);
uint16_t ccr2=__HAL_TIM_GET_COMPARE(&htim2,TIM_CHANNEL_2);
float plusWidth=(uint16_t)(ccr2-ccr1)*1e-6f;
float distance=340.0f*plusWidth/2.0f;	
return distance;
}	

return 999.0f;


}
void Obstacle_Avoide(void)
{
    
    Motor_SetSpeed(0,0);
    HAL_Delay(200);

 
    Motor_SetSpeed(-800, -800);
    HAL_Delay(400);

  
    Motor_SetSpeed(0,0);
    
    
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM3_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
Motor_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
 
  while (1)
  {
	  
	

      float distance = HCSR04_GetDistance();
      
      // 2. 
      if(distance > 0.01f && distance < 0.2f)
      {
          Obstacle_Avoide();
      }
	  
     else
      {
          uint8_t Sensor_value_raw = Sensor_Read();
          int error = Calculate_Error(Sensor_value_raw);
          PID_Track_Control(error); 
      }
      HAL_Delay(60);
	
	  
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
