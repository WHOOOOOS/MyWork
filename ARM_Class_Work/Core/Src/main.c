/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "adc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "fsmc.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <math.h>
#include "lcd.h"
#include "oled.h"
#include "Photo.h"
#include "mode3.h"

#define CHANNEL_X TIM_CHANNEL_1
#define CHANNEL_Y TIM_CHANNEL_2
//ϵͳ����ģʽ��������
uint8_t Mod = 0;
uint8_t RxData3 = 0;
uint8_t Speed = 0;
//��K230����ͨ�Ų�������
uint8_t Receive_x[2];
uint8_t Receive_y[2];
uint8_t Rxbuf = 0;//��ʼ��������
uint8_t state = 0;//��ʼ��״̬��
//���������������
uint16_t X=400;
uint16_t Y=240;
// ����ͼ�����ĵ�����
int center_x = 400;
int center_y = 240;
/*------------------------------------------------------------------PID����������----------------------------------------------------------------------------*/
//	//PID ��Ѳ����浵��(2024.9.20)  �����ٶȾ��ұ仯ʱ��һ�������������ٶ��±Ƚ���
//		float Kp_X = 0.32, Ki_X = 0.0001, Kd_X = 0.0066;
//   	float Kp_Y = 0.3, Ki_Y = 0.0001, Kd_Y = 0.0066;
//    float dt = 0.001;	
//		float Am = 0.05;
//PID ��Ѳ����浵��(2024.9.13)  û��ô�����������ڸ���Ŀ���ٶȽ�������������ȣ����ᳬ��
      float Kp_X = 0.35, Ki_X = 0.001, Kd_X = 0.00027;
      float Kp_Y = 0.35, Ki_Y = 0.001, Kd_Y = 0.00027;
      float dt = 0.001;  
	    float Am = 0.03;
//	//PID����������
//    float Kp_X = 0.32, Ki_X = 0.0001, Kd_X = 0.0066;
//    float Kp_Y = 0.3, Ki_Y = 0.0001, Kd_Y = 0.0066;
//    float dt = 0.001;
//		float Am = 0.05; //�Ŵ�ϵ����0.05	
	
	//����PID�����²���
    float prev_error_X = 0;
    float prev_error_Y= 0;
/*------------------------------------------------------------------PID����������----------------------------------------------------------------------------*/
//������PWM�������
float pwm_X = 1500;
float pwm_Y = 1500;
//����ģʽ2����
uint32_t ADC_Value_X=0;
uint32_t ADC_Value_Y=0;
uint8_t Value_Z = 0;

uint16_t Mod2_X = 1500;
uint16_t Mod2_Y = 1500;
//����ģʽ3����
uint16_t Dir_PWM_X = 1500;
uint16_t Dir_PWM_Y = 1500;
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE BEGIN PTD */ 
//����ʽPID������
float Incremental_PID(float target, float current, float Kp, float Ki, float Kd, float *prev_error) {
    float error = target - current;
		if(fabs(error)<= 10)
			return 0.0;
    //�����������
    float delta_error = error - *prev_error;

    // �����������
    float delta_output = Kp * error  + Ki * error * dt + Kd * (delta_error ) / dt;

    // �������
    *prev_error = error;

    // �����������
    return delta_output;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART2)
    {
			if(Rxbuf == 0xAA && state == 0)
			{
				state = 1;
			}
			else if(Rxbuf == 0xAA && state == 1)
			{
				state = 2;
			}
	    else if(state == 2)
			{
				Receive_x[0] = Rxbuf;
				state = 3;
			}
			else if(state == 3)
			{
				Receive_x[1] = Rxbuf;
				state = 4;
			}
			else if(state == 4)
			{
				Receive_y[0] = Rxbuf;
				state = 5;
			}
			else if(state == 5)
			{
				Receive_y[1] = Rxbuf;
				state = 6;
			}
			else if(Rxbuf == 0x67 &&state == 6)
			{
				X = Receive_x[0]*256 + Receive_x[1];
				Y = Receive_y[0]*256 + Receive_y[1];
				//HAL_UART_Transmit_IT(&huart3,Receive_x,sizeof(Receive_x));
				//HAL_UART_Transmit_IT(&huart3,Receive_y,sizeof(Receive_y));
				state = 0;
			}			
			else state = 0;	    
		  HAL_UART_Receive_IT(&huart2,&Rxbuf,sizeof(Rxbuf));
       
    }
	if(huart->Instance == USART3)
	{
	 if(RxData3 == 'a')
	 {
	  Mod = 0;
	  LCD_ShowNum(600, 110, 0, 1, 24);
	 }
	 else if(RxData3 == 'b')
	 {
	  Mod = 1;
	  LCD_ShowNum(600, 110, 1, 1, 24);
	 }
	 else if(RxData3 == 'c')
	 {
	  Mod = 2;
	  LCD_ShowNum(600, 110, 2, 1, 24); 		 
	 }
	 HAL_UART_Receive_IT(&huart3,&RxData3,1);//ÿ�ν����Ժ�Ҫ�ٴο��������ж�
	}
	
}

//����ADC1��ADC2��ת������
uint32_t X_get_adc(){
    //����ADC1
  HAL_ADC_Start(&hadc2);
    //�ȴ�ADCת����ɣ���ʱΪ100ms
    HAL_ADC_PollForConversion(&hadc2,1000);
    //�ж�ADC�Ƿ�ת���ɹ�
    if(HAL_IS_BIT_SET(HAL_ADC_GetState(&hadc2),HAL_ADC_STATE_REG_EOC)){
         //��ȡֵ
       return HAL_ADC_GetValue(&hadc2);
    }
    return 0;
}
uint32_t Y_get_adc(){
    //����ADC1
  HAL_ADC_Start(&hadc3);
    //�ȴ�ADCת����ɣ���ʱΪ100ms
    HAL_ADC_PollForConversion(&hadc3,1000);
    //�ж�ADC�Ƿ�ת���ɹ�
    if(HAL_IS_BIT_SET(HAL_ADC_GetState(&hadc3),HAL_ADC_STATE_REG_EOC)){
         //��ȡֵ
       return HAL_ADC_GetValue(&hadc3);
    }
    return 0;
}

//����ADת��ֵ�����PWM�������ת������
uint16_t Map_XY_To_PWM(uint16_t  X_Y)
{
	uint16_t PWM_Output;
	PWM_Output = (((X_Y/200)*100)+500);//�Ƚ�ADCת�����ȶ��ĺ���λ���ԣ�����ֵ��ӳ��
	return PWM_Output;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == GPIO_PIN_1)  //��������
		if(++Speed == 3)
		{
			Speed = 0;
		}
		if(Speed == 2)
		{
			Kp_X = 0.35, Ki_X = 0.0001, Kd_X = 0.0066;
			Kp_Y = 0.32, Ki_Y = 0.0001, Kd_Y = 0.0066;
			dt = 0.001;	
			Am = 0.05;
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_SET);
			LCD_ShowNum(625, 135, 2, 1, 24);
		}
		else if(Speed == 1)
		{
			Kp_X = 0.32, Ki_X = 0.0001, Kd_X = 0.0066;
			Kp_Y = 0.3, Ki_Y = 0.0001, Kd_Y = 0.0066;
			dt = 0.001;	
			Am = 0.05; 
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,GPIO_PIN_SET);
			LCD_ShowNum(625, 135, 1, 1, 24);
		}
		else if(Speed == 0)
		{
			Kp_X = 0.35, Ki_X = 0.001, Kd_X = 0.00027;
			Kp_Y = 0.35, Ki_Y = 0.001, Kd_Y = 0.00027;
			dt = 0.001;	
			Am = 0.03; 
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_RESET);
			LCD_ShowNum(625, 135, 0, 1, 24);
		}

	__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_0);  //����жϱ�־
}

void Mod1_run(void)
{
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_RESET);
			
		//�ֱ����X��Y�����ϵ��������
		float delta_pwm_X = Incremental_PID(center_x, X, Kp_X, Ki_X, Kd_X, &prev_error_X);
		float delta_pwm_Y = Incremental_PID(center_y, Y, Kp_Y, Ki_Y, Kd_Y, &prev_error_Y);

		//���¶����ǰ���ֵ
		pwm_X += (delta_pwm_X * Am); 
		pwm_Y += (delta_pwm_Y * Am);
		
		//���ƶ��ת���Ƕȷ�Χ
		if (pwm_X < 500) pwm_X = 500;
		if (pwm_X > 2500) pwm_X = 2500;
		if (pwm_Y < 900) pwm_Y = 900;
		if (pwm_Y > 2100) pwm_Y = 2100;

		//���¶��PWM���
		__HAL_TIM_SET_COMPARE(&htim3, CHANNEL_X, pwm_X);
		__HAL_TIM_SET_COMPARE(&htim3, CHANNEL_Y, pwm_Y);
	//2024.9.28 Recored by ���żΣ������ʱ����Ҫ�У���PIDϵ����ʱ���Ǵ��������ʱ���ģ�ȥ����������⣡����
		HAL_Delay(1);
	//2024.9.28 Recored by ���żΣ������ʱ����Ҫ�У���PIDϵ����ʱ���Ǵ��������ʱ���ģ�ȥ����������⣡����

}
void Mod2_run(void)
{
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_RESET);
		ADC_Value_X = X_get_adc();
		ADC_Value_Y = Y_get_adc();
		Mod2_X = Map_XY_To_PWM(ADC_Value_X);
		Mod2_Y = Map_XY_To_PWM(ADC_Value_Y);
		//���¶��PWM���
		__HAL_TIM_SET_COMPARE(&htim3, CHANNEL_X, Mod2_X);
		__HAL_TIM_SET_COMPARE(&htim3, CHANNEL_Y, Mod2_Y);
		OLED_ShowNum(1,10,Mod2_X,4);
		OLED_ShowNum(11,10,Mod2_Y,4);
		//HAL_Delay(100);
}
void Mod3_run(void)
{
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_SET);
		ADC_Value_X = X_get_adc();
		ADC_Value_Y = Y_get_adc();
		if((Dir_PWM_X+Dir_PWM(ADC_Value_X)<2400)&&(Dir_PWM_X+Dir_PWM(ADC_Value_X)>600)) Dir_PWM_X += Dir_PWM(ADC_Value_X);
		if((Dir_PWM_Y+Dir_PWM(ADC_Value_Y)<2400)&&(Dir_PWM_Y+Dir_PWM(ADC_Value_Y)>600)) Dir_PWM_Y += Dir_PWM(ADC_Value_Y);
		//���¶��PWM���
		__HAL_TIM_SET_COMPARE(&htim3, CHANNEL_X, Dir_PWM_X);
		__HAL_TIM_SET_COMPARE(&htim3, CHANNEL_Y, Dir_PWM_Y);
		OLED_ShowNum(1,10,Dir_PWM_X,4);
		OLED_ShowNum(11,10,Dir_PWM_Y,4);
}
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  MX_USART3_UART_Init();
  MX_FSMC_Init();
  MX_ADC2_Init();
  MX_ADC3_Init();
  /* USER CODE BEGIN 2 */
	  /* USER CODE BEGIN 2 */
	/*�Զ����ʼ��BEGIN*/
	LCD_Init();
  LCD_Display_Dir(1);
	OLED_Init();
    //OLED_Init();
	/*�Զ����ʼ��END*/
	/*�򿪴��ڽ����ж�BEGIN*/
	HAL_UART_Receive_IT(&huart2,&Rxbuf,sizeof(Rxbuf));   //��������
	HAL_UART_Receive_IT(&huart3,&RxData3,1);
	/*�򿪴��ڽ����ж�END*/
	/*��ʼ��PWM���BEGIN*/
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);
	/*��ʼ��PWM���END*/
	//��ʼ����������ĽǶ�Ϊ135��
	__HAL_TIM_SET_COMPARE(&htim3,CHANNEL_X,1500);//	
	__HAL_TIM_SET_COMPARE(&htim3,CHANNEL_Y,1500);//
	
	LCD_ShowString(550, 50, 200, 24, 24, (uint8_t *)"HUST");
	LCD_ShowString(550, 20, 200, 24, 24, (uint8_t *)"FZJ");
	LCD_ShowString(550, 80, 200, 24, 24, (uint8_t *)"2205030404");
	LCD_ShowString(550, 110, 200, 24, 24, (uint8_t *)"Mod:");
	LCD_ShowString(550, 135, 200, 24, 24, (uint8_t *)"Speed:");
	LCD_ShowNum(600, 110, 0, 1, 24);
	OLED_ShowString(1,1,"Value_X:");
	OLED_ShowString(11,1,"Value_Y:");
	
	LCD_ShowPicture(0,0,480,480,(uint16_t*)gImage_Eason);
	
	//����ģʽ��ʼ��
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6,GPIO_PIN_SET);                               
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
/*----------------------------------------------------------------ѭ����ʼ-------------------------------------------------------------------------------------*/
  while (1)
  { 
	
	if(Mod == 0)
  {
		Mod1_run();
	}
	else if(Mod == 1)
	{
		Mod2_run();
	}
	else if(Mod == 2)
	{
		Mod3_run();
	}
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
