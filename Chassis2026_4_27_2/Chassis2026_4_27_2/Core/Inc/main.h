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
#include "usart.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
extern uint8_t  TIM5CH1_CAPTURE_STA;  // ���벶��״̬T5C1
extern uint16_t TIM5CH1_CAPTURE_VAL;  // ���벶��ֵ
extern uint8_t  TIM5CH2_CAPTURE_STA;  // ���벶��״̬T5C2
extern uint16_t TIM5CH2_CAPTURE_VAL;  // ���벶��ֵ
extern uint8_t  TIM5CH3_CAPTURE_STA;  // ���벶��״̬T5C3
extern uint16_t TIM5CH3_CAPTURE_VAL;  // ���벶��ֵ
extern uint8_t  TIM5CH4_CAPTURE_STA;  // ���벶��״̬T5C4
extern uint16_t TIM5CH4_CAPTURE_VAL;  // ���벶��ֵ
extern uint8_t  TIM3CH3_CAPTURE_STA;  // ���벶��״̬T3C3
extern uint16_t TIM3CH3_CAPTURE_VAL;  // ���벶��ֵ
extern uint8_t  TIM3CH4_CAPTURE_STA;  // ���벶��״̬T3C4
extern uint16_t TIM3CH4_CAPTURE_VAL;  // ���벶��ֵ
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
#define key1_Pin GPIO_PIN_2
#define key1_GPIO_Port GPIOE
#define key1_EXTI_IRQn EXTI2_IRQn
#define key2_Pin GPIO_PIN_3
#define key2_GPIO_Port GPIOE
#define key2_EXTI_IRQn EXTI3_IRQn
#define key3_Pin GPIO_PIN_4
#define key3_GPIO_Port GPIOE
#define key3_EXTI_IRQn EXTI4_IRQn
#define SERVO1_Pin GPIO_PIN_5
#define SERVO1_GPIO_Port GPIOE
#define SERVO2_Pin GPIO_PIN_6
#define SERVO2_GPIO_Port GPIOE
#define LED3_Pin GPIO_PIN_15
#define LED3_GPIO_Port GPIOC
#define AIN2_Pin GPIO_PIN_0
#define AIN2_GPIO_Port GPIOC
#define AIN1_Pin GPIO_PIN_1
#define AIN1_GPIO_Port GPIOC
#define PES4_Pin GPIO_PIN_2
#define PES4_GPIO_Port GPIOC
#define PES5_Pin GPIO_PIN_3
#define PES5_GPIO_Port GPIOC
#define M4_ENCODER_A_Pin GPIO_PIN_0
#define M4_ENCODER_A_GPIO_Port GPIOA
#define M4_ENCODER_B_Pin GPIO_PIN_1
#define M4_ENCODER_B_GPIO_Port GPIOA
#define ECHO1_Pin GPIO_PIN_2
#define ECHO1_GPIO_Port GPIOA
#define ECHO2_Pin GPIO_PIN_3
#define ECHO2_GPIO_Port GPIOA
#define LED1_Pin GPIO_PIN_4
#define LED1_GPIO_Port GPIOA
#define ECHO3_Pin GPIO_PIN_5
#define ECHO3_GPIO_Port GPIOA
#define BIN1_Pin GPIO_PIN_4
#define BIN1_GPIO_Port GPIOC
#define BIN2_Pin GPIO_PIN_5
#define BIN2_GPIO_Port GPIOC
#define CIN1_Pin GPIO_PIN_2
#define CIN1_GPIO_Port GPIOB
#define CIN2_Pin GPIO_PIN_7
#define CIN2_GPIO_Port GPIOE
#define LED2_Pin GPIO_PIN_8
#define LED2_GPIO_Port GPIOE
#define work2B_Pin GPIO_PIN_10
#define work2B_GPIO_Port GPIOE
#define M1_ENCODER_B_Pin GPIO_PIN_11
#define M1_ENCODER_B_GPIO_Port GPIOE
#define work2A_Pin GPIO_PIN_12
#define work2A_GPIO_Port GPIOE
#define work1B_Pin GPIO_PIN_14
#define work1B_GPIO_Port GPIOE
#define work1A_Pin GPIO_PIN_15
#define work1A_GPIO_Port GPIOE
#define work1_Pin GPIO_PIN_14
#define work1_GPIO_Port GPIOB
#define work2_Pin GPIO_PIN_15
#define work2_GPIO_Port GPIOB
#define TR1_Pin GPIO_PIN_10
#define TR1_GPIO_Port GPIOD
#define TR2_Pin GPIO_PIN_11
#define TR2_GPIO_Port GPIOD
#define M3_ENCODER_A_Pin GPIO_PIN_12
#define M3_ENCODER_A_GPIO_Port GPIOD
#define M3_ENCODER_B_Pin GPIO_PIN_13
#define M3_ENCODER_B_GPIO_Port GPIOD
#define TR3_Pin GPIO_PIN_14
#define TR3_GPIO_Port GPIOD
#define TR4_Pin GPIO_PIN_15
#define TR4_GPIO_Port GPIOD
#define working_Pin GPIO_PIN_6
#define working_GPIO_Port GPIOC
#define PWMC_Pin GPIO_PIN_8
#define PWMC_GPIO_Port GPIOC
#define PWMD_Pin GPIO_PIN_9
#define PWMD_GPIO_Port GPIOC
#define M1_ENCODER_A_Pin GPIO_PIN_8
#define M1_ENCODER_A_GPIO_Port GPIOA
#define PES3_Pin GPIO_PIN_12
#define PES3_GPIO_Port GPIOA
#define PES4A15_Pin GPIO_PIN_15
#define PES4A15_GPIO_Port GPIOA
#define key4_Pin GPIO_PIN_7
#define key4_GPIO_Port GPIOD
#define key4_EXTI_IRQn EXTI9_5_IRQn
#define ECHO4_Pin GPIO_PIN_3
#define ECHO4_GPIO_Port GPIOB
#define M2_ENCODER_A_Pin GPIO_PIN_4
#define M2_ENCODER_A_GPIO_Port GPIOB
#define M2_ENCODER_B_Pin GPIO_PIN_5
#define M2_ENCODER_B_GPIO_Port GPIOB
#define PWMA_Pin GPIO_PIN_8
#define PWMA_GPIO_Port GPIOB
#define PWMB_Pin GPIO_PIN_9
#define PWMB_GPIO_Port GPIOB
#define DIN1_Pin GPIO_PIN_0
#define DIN1_GPIO_Port GPIOE
#define DIN2_Pin GPIO_PIN_1
#define DIN2_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
