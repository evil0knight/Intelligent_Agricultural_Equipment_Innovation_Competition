/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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
extern uint8_t  TIM5CH1_CAPTURE_STA;  // 输入捕获状态T5C1
extern uint16_t TIM5CH1_CAPTURE_VAL;  // 输入捕获值
extern uint8_t  TIM5CH2_CAPTURE_STA;  // 输入捕获状态T5C2
extern uint16_t TIM5CH2_CAPTURE_VAL;  // 输入捕获值
extern uint8_t  TIM5CH3_CAPTURE_STA;  // 输入捕获状态T5C3
extern uint16_t TIM5CH3_CAPTURE_VAL;  // 输入捕获值
extern uint8_t  TIM5CH4_CAPTURE_STA;  // 输入捕获状态T5C4
extern uint16_t TIM5CH4_CAPTURE_VAL;  // 输入捕获值
extern uint8_t  TIM3CH3_CAPTURE_STA;  // 输入捕获状态T3C3
extern uint16_t TIM3CH3_CAPTURE_VAL;  // 输入捕获值
extern uint8_t  TIM3CH4_CAPTURE_STA;  // 输入捕获状态T3C4
extern uint16_t TIM3CH4_CAPTURE_VAL;  // 输入捕获值
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
enum{Ting=0,Qian=1,Qianh=2,Zuo=3,You=4};
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
#define KEY5_Pin GPIO_PIN_6
#define KEY5_GPIO_Port GPIOE
#define KEY5_EXTI_IRQn EXTI9_5_IRQn
#define KEY6_Pin GPIO_PIN_13
#define KEY6_GPIO_Port GPIOC
#define KEY6_EXTI_IRQn EXTI15_10_IRQn
#define PES7_Pin GPIO_PIN_2
#define PES7_GPIO_Port GPIOC
#define PES8_Pin GPIO_PIN_3
#define PES8_GPIO_Port GPIOC
#define T5C1_ECHO1_Pin GPIO_PIN_0
#define T5C1_ECHO1_GPIO_Port GPIOA
#define T5C2_ECHO2_Pin GPIO_PIN_1
#define T5C2_ECHO2_GPIO_Port GPIOA
#define T5C3_ECHO3_Pin GPIO_PIN_2
#define T5C3_ECHO3_GPIO_Port GPIOA
#define T5C4_ECHO4_Pin GPIO_PIN_3
#define T5C4_ECHO4_GPIO_Port GPIOA
#define PES1_Pin GPIO_PIN_4
#define PES1_GPIO_Port GPIOA
#define PES2_Pin GPIO_PIN_5
#define PES2_GPIO_Port GPIOA
#define T3C3_ECHO5_Pin GPIO_PIN_0
#define T3C3_ECHO5_GPIO_Port GPIOB
#define T3C4_ECHO6_Pin GPIO_PIN_1
#define T3C4_ECHO6_GPIO_Port GPIOB
#define TRIG1_Pin GPIO_PIN_10
#define TRIG1_GPIO_Port GPIOE
#define TRIG2_Pin GPIO_PIN_11
#define TRIG2_GPIO_Port GPIOE
#define TRIG3_Pin GPIO_PIN_12
#define TRIG3_GPIO_Port GPIOE
#define TRIG4_Pin GPIO_PIN_13
#define TRIG4_GPIO_Port GPIOE
#define TRIG5_Pin GPIO_PIN_14
#define TRIG5_GPIO_Port GPIOE
#define TRIG6_Pin GPIO_PIN_15
#define TRIG6_GPIO_Port GPIOE
#define OLED_DC_Pin GPIO_PIN_12
#define OLED_DC_GPIO_Port GPIOB
#define OLED_SLK_Pin GPIO_PIN_13
#define OLED_SLK_GPIO_Port GPIOB
#define OLED_CS_Pin GPIO_PIN_14
#define OLED_CS_GPIO_Port GPIOB
#define OLED_MOSI_Pin GPIO_PIN_15
#define OLED_MOSI_GPIO_Port GPIOB
#define DIRA_Pin GPIO_PIN_6
#define DIRA_GPIO_Port GPIOC
#define DIRB_Pin GPIO_PIN_7
#define DIRB_GPIO_Port GPIOC
#define DIRC_Pin GPIO_PIN_8
#define DIRC_GPIO_Port GPIOC
#define DIRD_Pin GPIO_PIN_9
#define DIRD_GPIO_Port GPIOC
#define T1C1_PUL1_Pin GPIO_PIN_8
#define T1C1_PUL1_GPIO_Port GPIOA
#define T1C2_PUL2_Pin GPIO_PIN_9
#define T1C2_PUL2_GPIO_Port GPIOA
#define T1C3_PUL3_Pin GPIO_PIN_10
#define T1C3_PUL3_GPIO_Port GPIOA
#define T1C4_PUL4_Pin GPIO_PIN_11
#define T1C4_PUL4_GPIO_Port GPIOA
#define PES3_Pin GPIO_PIN_12
#define PES3_GPIO_Port GPIOA
#define PES4_Pin GPIO_PIN_15
#define PES4_GPIO_Port GPIOA
#define LED1_Pin GPIO_PIN_3
#define LED1_GPIO_Port GPIOB
#define LED2_Pin GPIO_PIN_4
#define LED2_GPIO_Port GPIOB
#define LED3_Pin GPIO_PIN_5
#define LED3_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
