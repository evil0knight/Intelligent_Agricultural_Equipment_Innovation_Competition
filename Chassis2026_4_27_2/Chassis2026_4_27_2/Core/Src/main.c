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
#include "can.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "my_uart.h"
#include "motor.h"
#include "mike.h"
#include "SR04.h"
#include "oled.h"
#include "Task_move.h"
#include "pid.h"
#include "key.h"
#include "encoder.h"
#include <stdio.h>
#include "string.h"
#include "encoder.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* 测试代码  */


extern float speed_goal;
extern float u_distance;
extern uint8_t corridor_cnt;
extern uint8_t M_speed_state,Move_flag;
uint8_t m1,m2;
/* 测试代码  */




uint8_t display_buf[20];


extern int speed;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

//extern float LastDis;////////
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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM5_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_TIM6_Init();
  MX_UART4_Init();
  MX_UART5_Init();
  MX_TIM8_Init();
  MX_TIM9_Init();
  MX_CAN1_Init();
  MX_TIM12_Init();
  MX_USART1_UART_Init();
  MX_TIM10_Init();
  MX_TIM11_Init();
  MX_TIM13_Init();
  MX_TIM14_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
//	OLED_Init();
//  OLED_Clear();

	AUTO_uartDma_init();
	Move_init();
	Encoder_Clear();	
	/* 编码器定时调用 */
	HAL_TIM_Base_Start_IT(&htim6);
	
	/* 开启定时器(超声波) */
	HAL_TIM_IC_Init (&htim2);                		
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);   //开启TIM2的捕获通道1，并开启捕获
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);   
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_3);   
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_4);  
  __HAL_TIM_ENABLE_IT(&htim2, TIM_IT_UPDATE);   //使能更新中断
	/* 底盘 */
	HAL_TIM_PWM_Start(&htim10,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim11,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim8 ,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim8 ,TIM_CHANNEL_4);
	/* 作业机构 */
	HAL_TIM_PWM_Start(&htim12 ,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim12 ,TIM_CHANNEL_2);
	
	/* 底盘编码器 */
	HAL_TIM_Encoder_Start(&htim1,TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim4,TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim5,TIM_CHANNEL_ALL);
	
	Encoder_Filter_Init(&EncoAspeed_Filter);
	Encoder_Filter_Init(&EncoBspeed_Filter);
	Encoder_Filter_Init(&EncoCspeed_Filter);
	Encoder_Filter_Init(&EncoDspeed_Filter);
	
	Ultrasonic_Filter_Init(&Ultrasonic_L_Filter);
	Ultrasonic_Filter_Init(&Ultrasonic_B_Filter);
	Ultrasonic_Filter_Init(&Ultrasonic_F_Filter);
	Ultrasonic_Filter_Init(&Ultrasonic_R_Filter);
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {  

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		Working_motor_set(5500,5500);
//		printf("%.2f,%.2f\n",ultra.L,u_distance);
//		Motor_current_set(1000,1000,1000,1000);
//		printf("%.2f,%.2f,%.2f,%.2f\n",E_motor.speedA,E_motor.speedB,E_motor.speedC,E_motor.speedD);
//		printf("%.2f,%.2f\n",u_distance,ultra.B);
//		printf("%.2f,%.2f\n",start_yaw,yaw);
//		pes_scan(F_u);
//		m1 =HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_6);
//		m2 =PES_F;

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
