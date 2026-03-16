/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define KEY1_Pin GPIO_PIN_2
#define KEY1_GPIO_Port GPIOE
#define KEY1_EXTI_IRQn EXTI2_IRQn
#define KEY2_Pin GPIO_PIN_3
#define KEY2_GPIO_Port GPIOE
#define KEY2_EXTI_IRQn EXTI3_IRQn
#define KEY3_Pin GPIO_PIN_4
#define KEY3_GPIO_Port GPIOE
#define KEY3_EXTI_IRQn EXTI4_IRQn
#define KEY4_Pin GPIO_PIN_5
#define KEY4_GPIO_Port GPIOE
#define KEY4_EXTI_IRQn EXTI9_5_IRQn
#define Free_IO1_Pin GPIO_PIN_0
#define Free_IO1_GPIO_Port GPIOC
#define Free_IO2_Pin GPIO_PIN_1
#define Free_IO2_GPIO_Port GPIOC
#define Free_IO3_Pin GPIO_PIN_2
#define Free_IO3_GPIO_Port GPIOC
#define Free_IO4_Pin GPIO_PIN_3
#define Free_IO4_GPIO_Port GPIOC
#define M1_ENA_Pin GPIO_PIN_8
#define M1_ENA_GPIO_Port GPIOE
#define M1_PUL_Pin GPIO_PIN_9
#define M1_PUL_GPIO_Port GPIOE
#define M2_ENA_Pin GPIO_PIN_10
#define M2_ENA_GPIO_Port GPIOE
#define M2_PUL_Pin GPIO_PIN_11
#define M2_PUL_GPIO_Port GPIOE
#define M3_ENA_Pin GPIO_PIN_12
#define M3_ENA_GPIO_Port GPIOE
#define M3_PUL_Pin GPIO_PIN_13
#define M3_PUL_GPIO_Port GPIOE
#define OLED_DC_Pin GPIO_PIN_12
#define OLED_DC_GPIO_Port GPIOB
#define OLED_SCK_Pin GPIO_PIN_13
#define OLED_SCK_GPIO_Port GPIOB
#define OLED_CS_Pin GPIO_PIN_14
#define OLED_CS_GPIO_Port GPIOB
#define OLED_MOSI_Pin GPIO_PIN_15
#define OLED_MOSI_GPIO_Port GPIOB
#define M3_DIR_Pin GPIO_PIN_10
#define M3_DIR_GPIO_Port GPIOD
#define M2_DIR_Pin GPIO_PIN_11
#define M2_DIR_GPIO_Port GPIOD
#define M1_DIR_Pin GPIO_PIN_12
#define M1_DIR_GPIO_Port GPIOD
#define LED1_Pin GPIO_PIN_3
#define LED1_GPIO_Port GPIOB
#define LED2_Pin GPIO_PIN_4
#define LED2_GPIO_Port GPIOB
#define LED3_Pin GPIO_PIN_5
#define LED3_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

typedef struct
{

	int32_t angle;
	
} motor;

typedef struct
{
	uint8_t state;
	uint8_t status;
	
	float servo_180angle[4];
	uint16_t servo_180pulse[4];
		
	float servo_360angle[2];
	uint16_t servo_360pulse[2];
	
	uint8_t up_date_rx[3];
	uint8_t up_date_tx[3];
	
	uint8_t key_value[4];
	uint8_t last_key_value[4];
	
	
} gimbal_move_t;



/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
