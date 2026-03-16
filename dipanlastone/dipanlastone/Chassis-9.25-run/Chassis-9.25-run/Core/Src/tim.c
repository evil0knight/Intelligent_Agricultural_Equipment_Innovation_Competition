/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    tim.c
  * @brief   This file provides code for the configuration
  *          of the TIM instances.
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
#include "tim.h"

/* USER CODE BEGIN 0 */

#include "2ms_Task.h"
#include "5ms_Task.h"
#include "10ms_Task.h"
#include "20ms_Task.h"
#include "50ms_Task.h"
#include "500ms_Task.h"

uint8_t  TIM5CH1_CAPTURE_STA;  // 输入捕获状态T5C1
uint16_t TIM5CH1_CAPTURE_VAL;  // 输入捕获值
uint8_t  TIM5CH2_CAPTURE_STA;  // 输入捕获状态T5C2
uint16_t TIM5CH2_CAPTURE_VAL;  // 输入捕获值
uint8_t  TIM5CH3_CAPTURE_STA;  // 输入捕获状态T5C3
uint16_t TIM5CH3_CAPTURE_VAL;  // 输入捕获值
uint8_t  TIM5CH4_CAPTURE_STA;  // 输入捕获状态T5C4
uint16_t TIM5CH4_CAPTURE_VAL;  // 输入捕获值

uint8_t  TIM3CH3_CAPTURE_STA;  // 输入捕获状态T3C3
uint16_t TIM3CH3_CAPTURE_VAL;  // 输入捕获值
uint8_t  TIM3CH4_CAPTURE_STA;  // 输入捕获状态T3C4
uint16_t TIM3CH4_CAPTURE_VAL;  // 输入捕获值

uint16_t  Num1=0,Num2=0,Num3=0,Num4=0,Num5=0,Num6=0;//定时器中断计数值
/* USER CODE END 0 */

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim10;
TIM_HandleTypeDef htim11;
TIM_HandleTypeDef htim13;
TIM_HandleTypeDef htim14;

/* TIM1 init function */
void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 168-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
  sConfigOC.Pulse = 500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}
/* TIM2 init function */
void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 168-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}
/* TIM3 init function */
void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 168-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 6;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}
/* TIM4 init function */
void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}
/* TIM5 init function */
void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 168-1;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 65535;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 6;
  if (HAL_TIM_IC_ConfigChannel(&htim5, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim5, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim5, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim5, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}
/* TIM6 init function */
void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 168-1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 1000-1;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}
/* TIM10 init function */
void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 168-1;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 65535;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
  sConfigOC.Pulse = 50;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim10, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */
  HAL_TIM_MspPostInit(&htim10);

}
/* TIM11 init function */
void MX_TIM11_Init(void)
{

  /* USER CODE BEGIN TIM11_Init 0 */

  /* USER CODE END TIM11_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM11_Init 1 */

  /* USER CODE END TIM11_Init 1 */
  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 168-1;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 65535;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim11, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM11_Init 2 */

  /* USER CODE END TIM11_Init 2 */
  HAL_TIM_MspPostInit(&htim11);

}
/* TIM13 init function */
void MX_TIM13_Init(void)
{

  /* USER CODE BEGIN TIM13_Init 0 */

  /* USER CODE END TIM13_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM13_Init 1 */

  /* USER CODE END TIM13_Init 1 */
  htim13.Instance = TIM13;
  htim13.Init.Prescaler = 168-1;
  htim13.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim13.Init.Period = 65535;
  htim13.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim13.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim13) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim13) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim13, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM13_Init 2 */

  /* USER CODE END TIM13_Init 2 */
  HAL_TIM_MspPostInit(&htim13);

}
/* TIM14 init function */
void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 168-1;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 65535;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim14, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */
  HAL_TIM_MspPostInit(&htim14);

}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* tim_baseHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(tim_baseHandle->Instance==TIM1)
  {
  /* USER CODE BEGIN TIM1_MspInit 0 */

  /* USER CODE END TIM1_MspInit 0 */
    /* TIM1 clock enable */
    __HAL_RCC_TIM1_CLK_ENABLE();

    /* TIM1 interrupt Init */
    HAL_NVIC_SetPriority(TIM1_BRK_TIM9_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM1_BRK_TIM9_IRQn);
    HAL_NVIC_SetPriority(TIM1_UP_TIM10_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
    HAL_NVIC_SetPriority(TIM1_TRG_COM_TIM11_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM1_TRG_COM_TIM11_IRQn);
    HAL_NVIC_SetPriority(TIM1_CC_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM1_CC_IRQn);
  /* USER CODE BEGIN TIM1_MspInit 1 */

  /* USER CODE END TIM1_MspInit 1 */
  }
  else if(tim_baseHandle->Instance==TIM2)
  {
  /* USER CODE BEGIN TIM2_MspInit 0 */

  /* USER CODE END TIM2_MspInit 0 */
    /* TIM2 clock enable */
    __HAL_RCC_TIM2_CLK_ENABLE();

    /* TIM2 interrupt Init */
    HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);
  /* USER CODE BEGIN TIM2_MspInit 1 */

  /* USER CODE END TIM2_MspInit 1 */
  }
  else if(tim_baseHandle->Instance==TIM3)
  {
  /* USER CODE BEGIN TIM3_MspInit 0 */

  /* USER CODE END TIM3_MspInit 0 */
    /* TIM3 clock enable */
    __HAL_RCC_TIM3_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**TIM3 GPIO Configuration
    PB0     ------> TIM3_CH3
    PB1     ------> TIM3_CH4
    */
    GPIO_InitStruct.Pin = T3C3_ECHO5_Pin|T3C4_ECHO6_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* TIM3 interrupt Init */
    HAL_NVIC_SetPriority(TIM3_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM3_IRQn);
  /* USER CODE BEGIN TIM3_MspInit 1 */

  /* USER CODE END TIM3_MspInit 1 */
  }
  else if(tim_baseHandle->Instance==TIM4)
  {
  /* USER CODE BEGIN TIM4_MspInit 0 */

  /* USER CODE END TIM4_MspInit 0 */
    /* TIM4 clock enable */
    __HAL_RCC_TIM4_CLK_ENABLE();
  /* USER CODE BEGIN TIM4_MspInit 1 */

  /* USER CODE END TIM4_MspInit 1 */
  }
  else if(tim_baseHandle->Instance==TIM5)
  {
  /* USER CODE BEGIN TIM5_MspInit 0 */

  /* USER CODE END TIM5_MspInit 0 */
    /* TIM5 clock enable */
    __HAL_RCC_TIM5_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**TIM5 GPIO Configuration
    PA0-WKUP     ------> TIM5_CH1
    PA1     ------> TIM5_CH2
    PA2     ------> TIM5_CH3
    PA3     ------> TIM5_CH4
    */
    GPIO_InitStruct.Pin = T5C1_ECHO1_Pin|T5C2_ECHO2_Pin|T5C3_ECHO3_Pin|T5C4_ECHO4_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM5;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* TIM5 interrupt Init */
    HAL_NVIC_SetPriority(TIM5_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM5_IRQn);
  /* USER CODE BEGIN TIM5_MspInit 1 */

  /* USER CODE END TIM5_MspInit 1 */
  }
  else if(tim_baseHandle->Instance==TIM6)
  {
  /* USER CODE BEGIN TIM6_MspInit 0 */

  /* USER CODE END TIM6_MspInit 0 */
    /* TIM6 clock enable */
    __HAL_RCC_TIM6_CLK_ENABLE();

    /* TIM6 interrupt Init */
    HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
  /* USER CODE BEGIN TIM6_MspInit 1 */

  /* USER CODE END TIM6_MspInit 1 */
  }
  else if(tim_baseHandle->Instance==TIM10)
  {
  /* USER CODE BEGIN TIM10_MspInit 0 */

  /* USER CODE END TIM10_MspInit 0 */
    /* TIM10 clock enable */
    __HAL_RCC_TIM10_CLK_ENABLE();

    /* TIM10 interrupt Init */
    HAL_NVIC_SetPriority(TIM1_UP_TIM10_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
  /* USER CODE BEGIN TIM10_MspInit 1 */

  /* USER CODE END TIM10_MspInit 1 */
  }
  else if(tim_baseHandle->Instance==TIM11)
  {
  /* USER CODE BEGIN TIM11_MspInit 0 */

  /* USER CODE END TIM11_MspInit 0 */
    /* TIM11 clock enable */
    __HAL_RCC_TIM11_CLK_ENABLE();

    /* TIM11 interrupt Init */
    HAL_NVIC_SetPriority(TIM1_TRG_COM_TIM11_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM1_TRG_COM_TIM11_IRQn);
  /* USER CODE BEGIN TIM11_MspInit 1 */

  /* USER CODE END TIM11_MspInit 1 */
  }
  else if(tim_baseHandle->Instance==TIM13)
  {
  /* USER CODE BEGIN TIM13_MspInit 0 */

  /* USER CODE END TIM13_MspInit 0 */
    /* TIM13 clock enable */
    __HAL_RCC_TIM13_CLK_ENABLE();

    /* TIM13 interrupt Init */
    HAL_NVIC_SetPriority(TIM8_UP_TIM13_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM8_UP_TIM13_IRQn);
  /* USER CODE BEGIN TIM13_MspInit 1 */

  /* USER CODE END TIM13_MspInit 1 */
  }
  else if(tim_baseHandle->Instance==TIM14)
  {
  /* USER CODE BEGIN TIM14_MspInit 0 */

  /* USER CODE END TIM14_MspInit 0 */
    /* TIM14 clock enable */
    __HAL_RCC_TIM14_CLK_ENABLE();

    /* TIM14 interrupt Init */
    HAL_NVIC_SetPriority(TIM8_TRG_COM_TIM14_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM8_TRG_COM_TIM14_IRQn);
  /* USER CODE BEGIN TIM14_MspInit 1 */

  /* USER CODE END TIM14_MspInit 1 */
  }
}
void HAL_TIM_MspPostInit(TIM_HandleTypeDef* timHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(timHandle->Instance==TIM1)
  {
  /* USER CODE BEGIN TIM1_MspPostInit 0 */

  /* USER CODE END TIM1_MspPostInit 0 */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**TIM1 GPIO Configuration
    PA8     ------> TIM1_CH1
    PA9     ------> TIM1_CH2
    PA10     ------> TIM1_CH3
    PA11     ------> TIM1_CH4
    */
    GPIO_InitStruct.Pin = T1C1_PUL1_Pin|T1C2_PUL2_Pin|T1C3_PUL3_Pin|T1C4_PUL4_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN TIM1_MspPostInit 1 */

  /* USER CODE END TIM1_MspPostInit 1 */
  }
  else if(timHandle->Instance==TIM10)
  {
  /* USER CODE BEGIN TIM10_MspPostInit 0 */

  /* USER CODE END TIM10_MspPostInit 0 */

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**TIM10 GPIO Configuration
    PB8     ------> TIM10_CH1
    */
    GPIO_InitStruct.Pin = GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF3_TIM10;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN TIM10_MspPostInit 1 */

  /* USER CODE END TIM10_MspPostInit 1 */
  }
  else if(timHandle->Instance==TIM11)
  {
  /* USER CODE BEGIN TIM11_MspPostInit 0 */

  /* USER CODE END TIM11_MspPostInit 0 */

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**TIM11 GPIO Configuration
    PB9     ------> TIM11_CH1
    */
    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF3_TIM11;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN TIM11_MspPostInit 1 */

  /* USER CODE END TIM11_MspPostInit 1 */
  }
  else if(timHandle->Instance==TIM13)
  {
  /* USER CODE BEGIN TIM13_MspPostInit 0 */

  /* USER CODE END TIM13_MspPostInit 0 */

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**TIM13 GPIO Configuration
    PA6     ------> TIM13_CH1
    */
    GPIO_InitStruct.Pin = GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF9_TIM13;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN TIM13_MspPostInit 1 */

  /* USER CODE END TIM13_MspPostInit 1 */
  }
  else if(timHandle->Instance==TIM14)
  {
  /* USER CODE BEGIN TIM14_MspPostInit 0 */

  /* USER CODE END TIM14_MspPostInit 0 */

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**TIM14 GPIO Configuration
    PA7     ------> TIM14_CH1
    */
    GPIO_InitStruct.Pin = GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF9_TIM14;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN TIM14_MspPostInit 1 */

  /* USER CODE END TIM14_MspPostInit 1 */
  }

}

void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* tim_baseHandle)
{

  if(tim_baseHandle->Instance==TIM1)
  {
  /* USER CODE BEGIN TIM1_MspDeInit 0 */

  /* USER CODE END TIM1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM1_CLK_DISABLE();

    /* TIM1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(TIM1_BRK_TIM9_IRQn);
  /* USER CODE BEGIN TIM1:TIM1_UP_TIM10_IRQn disable */
    /**
    * Uncomment the line below to disable the "TIM1_UP_TIM10_IRQn" interrupt
    * Be aware, disabling shared interrupt may affect other IPs
    */
    /* HAL_NVIC_DisableIRQ(TIM1_UP_TIM10_IRQn); */
  /* USER CODE END TIM1:TIM1_UP_TIM10_IRQn disable */

  /* USER CODE BEGIN TIM1:TIM1_TRG_COM_TIM11_IRQn disable */
    /**
    * Uncomment the line below to disable the "TIM1_TRG_COM_TIM11_IRQn" interrupt
    * Be aware, disabling shared interrupt may affect other IPs
    */
    /* HAL_NVIC_DisableIRQ(TIM1_TRG_COM_TIM11_IRQn); */
  /* USER CODE END TIM1:TIM1_TRG_COM_TIM11_IRQn disable */

    HAL_NVIC_DisableIRQ(TIM1_CC_IRQn);
  /* USER CODE BEGIN TIM1_MspDeInit 1 */

  /* USER CODE END TIM1_MspDeInit 1 */
  }
  else if(tim_baseHandle->Instance==TIM2)
  {
  /* USER CODE BEGIN TIM2_MspDeInit 0 */

  /* USER CODE END TIM2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM2_CLK_DISABLE();

    /* TIM2 interrupt Deinit */
    HAL_NVIC_DisableIRQ(TIM2_IRQn);
  /* USER CODE BEGIN TIM2_MspDeInit 1 */

  /* USER CODE END TIM2_MspDeInit 1 */
  }
  else if(tim_baseHandle->Instance==TIM3)
  {
  /* USER CODE BEGIN TIM3_MspDeInit 0 */

  /* USER CODE END TIM3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM3_CLK_DISABLE();

    /**TIM3 GPIO Configuration
    PB0     ------> TIM3_CH3
    PB1     ------> TIM3_CH4
    */
    HAL_GPIO_DeInit(GPIOB, T3C3_ECHO5_Pin|T3C4_ECHO6_Pin);

    /* TIM3 interrupt Deinit */
    HAL_NVIC_DisableIRQ(TIM3_IRQn);
  /* USER CODE BEGIN TIM3_MspDeInit 1 */

  /* USER CODE END TIM3_MspDeInit 1 */
  }
  else if(tim_baseHandle->Instance==TIM4)
  {
  /* USER CODE BEGIN TIM4_MspDeInit 0 */

  /* USER CODE END TIM4_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM4_CLK_DISABLE();
  /* USER CODE BEGIN TIM4_MspDeInit 1 */

  /* USER CODE END TIM4_MspDeInit 1 */
  }
  else if(tim_baseHandle->Instance==TIM5)
  {
  /* USER CODE BEGIN TIM5_MspDeInit 0 */

  /* USER CODE END TIM5_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM5_CLK_DISABLE();

    /**TIM5 GPIO Configuration
    PA0-WKUP     ------> TIM5_CH1
    PA1     ------> TIM5_CH2
    PA2     ------> TIM5_CH3
    PA3     ------> TIM5_CH4
    */
    HAL_GPIO_DeInit(GPIOA, T5C1_ECHO1_Pin|T5C2_ECHO2_Pin|T5C3_ECHO3_Pin|T5C4_ECHO4_Pin);

    /* TIM5 interrupt Deinit */
    HAL_NVIC_DisableIRQ(TIM5_IRQn);
  /* USER CODE BEGIN TIM5_MspDeInit 1 */

  /* USER CODE END TIM5_MspDeInit 1 */
  }
  else if(tim_baseHandle->Instance==TIM6)
  {
  /* USER CODE BEGIN TIM6_MspDeInit 0 */

  /* USER CODE END TIM6_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM6_CLK_DISABLE();

    /* TIM6 interrupt Deinit */
    HAL_NVIC_DisableIRQ(TIM6_DAC_IRQn);
  /* USER CODE BEGIN TIM6_MspDeInit 1 */

  /* USER CODE END TIM6_MspDeInit 1 */
  }
  else if(tim_baseHandle->Instance==TIM10)
  {
  /* USER CODE BEGIN TIM10_MspDeInit 0 */

  /* USER CODE END TIM10_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM10_CLK_DISABLE();

    /* TIM10 interrupt Deinit */
  /* USER CODE BEGIN TIM10:TIM1_UP_TIM10_IRQn disable */
    /**
    * Uncomment the line below to disable the "TIM1_UP_TIM10_IRQn" interrupt
    * Be aware, disabling shared interrupt may affect other IPs
    */
    /* HAL_NVIC_DisableIRQ(TIM1_UP_TIM10_IRQn); */
  /* USER CODE END TIM10:TIM1_UP_TIM10_IRQn disable */

  /* USER CODE BEGIN TIM10_MspDeInit 1 */

  /* USER CODE END TIM10_MspDeInit 1 */
  }
  else if(tim_baseHandle->Instance==TIM11)
  {
  /* USER CODE BEGIN TIM11_MspDeInit 0 */

  /* USER CODE END TIM11_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM11_CLK_DISABLE();

    /* TIM11 interrupt Deinit */
  /* USER CODE BEGIN TIM11:TIM1_TRG_COM_TIM11_IRQn disable */
    /**
    * Uncomment the line below to disable the "TIM1_TRG_COM_TIM11_IRQn" interrupt
    * Be aware, disabling shared interrupt may affect other IPs
    */
    /* HAL_NVIC_DisableIRQ(TIM1_TRG_COM_TIM11_IRQn); */
  /* USER CODE END TIM11:TIM1_TRG_COM_TIM11_IRQn disable */

  /* USER CODE BEGIN TIM11_MspDeInit 1 */

  /* USER CODE END TIM11_MspDeInit 1 */
  }
  else if(tim_baseHandle->Instance==TIM13)
  {
  /* USER CODE BEGIN TIM13_MspDeInit 0 */

  /* USER CODE END TIM13_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM13_CLK_DISABLE();

    /* TIM13 interrupt Deinit */
    HAL_NVIC_DisableIRQ(TIM8_UP_TIM13_IRQn);
  /* USER CODE BEGIN TIM13_MspDeInit 1 */

  /* USER CODE END TIM13_MspDeInit 1 */
  }
  else if(tim_baseHandle->Instance==TIM14)
  {
  /* USER CODE BEGIN TIM14_MspDeInit 0 */

  /* USER CODE END TIM14_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM14_CLK_DISABLE();

    /* TIM14 interrupt Deinit */
    HAL_NVIC_DisableIRQ(TIM8_TRG_COM_TIM14_IRQn);
  /* USER CODE BEGIN TIM14_MspDeInit 1 */

  /* USER CODE END TIM14_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */


//[7]:0,没有成功的捕获;1,成功捕获到一次.
//[6]:0,还没捕获到低电平;1,已经捕获到低电平了.
//[5:0]:捕获低电平后溢出的次数

//溢出回调函数和捕获回调函数
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance  == htim2 .Instance )
	{
		Num1++;
		Num2++;
		Num3++;
		Num4++;
		Num5++;
		if(Num1 ==2)
		{		 
			Task2ms();
			Num1=0;
		}
		
		if(Num2 ==5)
		{		 
			Task5ms();
			Num2=0;
		}
		
		if(Num3 ==10)
		{		 
			Task10ms();
			Num3=0;
		}
		
		if(Num4 ==20)
		{		 
			Task20ms();
			Num4=0;
		}
		
		if(Num5 ==50)
		{		 
			Task50ms();
			Num5=0;
		}
		
		if(Num6 ==500)
		{		 
			Task500ms();
			Num6=0;
		}
		
	}
	
	
	
	
	
	
	if(htim->Instance  == htim5 .Instance )
	{
		if ((TIM5CH1_CAPTURE_STA & 0x80) == 0) // 还未捕获成功
		{
			if (TIM5CH1_CAPTURE_STA & 0x40) // 捕获到一个下降沿
			{
				if ((TIM5CH1_CAPTURE_STA & 0x3F) == 0x3F) // 高电平的时间太长
				{
					TIM5CH1_CAPTURE_STA |= 0X80; // 标记为成功捕获一次
					TIM5CH1_CAPTURE_VAL = 0XFFFF;
				}
				else
					TIM5CH1_CAPTURE_STA++; // 否则标记溢出数加1
			}
		}
		
		
		if ((TIM5CH2_CAPTURE_STA & 0x80) == 0) // 还未捕获成功
		{
			if (TIM5CH2_CAPTURE_STA & 0x40) // 捕获到一个下降沿
			{
				if ((TIM5CH2_CAPTURE_STA & 0x3F) == 0x3F) // 高电平的时间太长
				{
					TIM5CH2_CAPTURE_STA |= 0X80; // 标记为成功捕获一次
					TIM5CH2_CAPTURE_VAL = 0XFFFF;
				}
				else
					TIM5CH2_CAPTURE_STA++; // 否则标记溢出数加1
			}
		}
		
		if ((TIM5CH3_CAPTURE_STA & 0x80) == 0) // 还未捕获成功
		{
			if (TIM5CH3_CAPTURE_STA & 0x40) // 捕获到一个下降沿
			{
				if ((TIM5CH3_CAPTURE_STA & 0x3F) == 0x3F) // 高电平的时间太长
				{
					TIM5CH3_CAPTURE_STA |= 0X80; // 标记为成功捕获一次
					TIM5CH3_CAPTURE_VAL = 0XFFFF;
				}
				else
					TIM5CH3_CAPTURE_STA++; // 否则标记溢出数加1
			}
		}
		
		if ((TIM5CH4_CAPTURE_STA & 0x80) == 0) // 还未捕获成功
		{
			if (TIM5CH4_CAPTURE_STA & 0x40) // 捕获到一个下降沿
			{
				if ((TIM5CH4_CAPTURE_STA & 0x3F) == 0x3F) // 高电平的时间太长
				{
					TIM5CH4_CAPTURE_STA |= 0X80; // 标记为成功捕获一次
					TIM5CH4_CAPTURE_VAL = 0XFFFF;
				}
				else
					TIM5CH4_CAPTURE_STA++; // 否则标记溢出数加1
			}
		}
		
	}	
	
	
//	if(htim->Instance  == htim3 .Instance )
//	{	
//		if ((TIM3CH3_CAPTURE_STA & 0x80) == 0) // 还未捕获成功
//		{
//			if (TIM3CH3_CAPTURE_STA & 0x40) // 捕获到一个下降沿
//			{
//				if ((TIM3CH3_CAPTURE_STA & 0x3F) == 0x3F) // 高电平的时间太长
//				{
//					TIM3CH3_CAPTURE_STA |= 0X80; // 标记为成功捕获一次
//					TIM3CH3_CAPTURE_VAL = 0XFFFF;
//				}
//				else
//					TIM3CH3_CAPTURE_STA++; // 否则标记溢出数加1
//			}
//		}
//		
//		if ((TIM3CH4_CAPTURE_STA & 0x80) == 0) // 还未捕获成功
//		{
//			if (TIM3CH4_CAPTURE_STA & 0x40) // 捕获到一个下降沿
//			{
//				if ((TIM3CH4_CAPTURE_STA & 0x3F) == 0x3F) // 高电平的时间太长
//				{
//					TIM3CH4_CAPTURE_STA |= 0X80; // 标记为成功捕获一次
//					TIM3CH4_CAPTURE_VAL = 0XFFFF;
//				}
//				else
//					TIM3CH4_CAPTURE_STA++; // 否则标记溢出数加1
//			}
//		}
//	}
	
}



// 捕获中断发生时执行 上升沿复位开始计时，下降沿获取捕获值计算
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	
	/***************************************************T5C1_ECHO1*******************************************************/
  if ((TIM5CH1_CAPTURE_STA & 0X80) == 0) //还未捕获成功  [7]:0,没有成功的捕获;1,成功捕获到一次.
  {
    if (TIM5CH1_CAPTURE_STA & 0X40) // 成功率捕获到1个下降沿  [6]:0,还没捕获到低电平;1,已经捕获到低电平了.
    {
      // usart_printf("get down\r\n");
      TIM5CH1_CAPTURE_STA |= 0X80;                                            // 标记成功，捕获到1次高电平完成
      TIM5CH1_CAPTURE_VAL = HAL_TIM_ReadCapturedValue(&htim5, TIM_CHANNEL_1); // 捕获当前设置捕获值
      TIM_RESET_CAPTUREPOLARITY(&htim5, TIM_CHANNEL_1);                       // 清除原来设置
      TIM_SET_CAPTUREPOLARITY(&htim5, TIM_CHANNEL_1, TIM_ICPOLARITY_RISING);  // 捕获到下降沿之后，将捕获到复位为上升沿
    }
    else // 捕获到一个上升沿
    {
      // usart_printf("get up\r\n");
      TIM5CH1_CAPTURE_STA = 0;
      TIM5CH1_CAPTURE_VAL = 0;
      TIM5CH1_CAPTURE_STA |= 0X40; //将STA置为0x40 当下一次触发中断时，会进入上面的if语句
      __HAL_TIM_DISABLE(&htim5);
      __HAL_TIM_SET_COUNTER(&htim5, 0);
      TIM_RESET_CAPTUREPOLARITY(&htim5, TIM_CHANNEL_1);
      TIM_SET_CAPTUREPOLARITY(&htim5, TIM_CHANNEL_1, TIM_ICPOLARITY_FALLING);
      __HAL_TIM_ENABLE(&htim5);
    }
  }
	/***************************************************T5C2_ECHO2*******************************************************/	
	if ((TIM5CH2_CAPTURE_STA & 0X80) == 0) //还未捕获成功  [7]:0,没有成功的捕获;1,成功捕获到一次.
  {
    if (TIM5CH2_CAPTURE_STA & 0X40) // 成功率捕获到1个下降沿  [6]:0,还没捕获到低电平;1,已经捕获到低电平了.
    {
      // usart_printf("get down\r\n");
      TIM5CH2_CAPTURE_STA |= 0X80;                                            // 标记成功，捕获到1次高电平完成
      TIM5CH2_CAPTURE_VAL = HAL_TIM_ReadCapturedValue(&htim5, TIM_CHANNEL_2); // 捕获当前设置捕获值
      TIM_RESET_CAPTUREPOLARITY(&htim5, TIM_CHANNEL_2);                       // 清除原来设置
      TIM_SET_CAPTUREPOLARITY(&htim5, TIM_CHANNEL_2, TIM_ICPOLARITY_RISING);  // 捕获到下降沿之后，将捕获到复位为上升沿
    }
    else // 捕获到一个上升沿
    {
      // usart_printf("get up\r\n");
      TIM5CH2_CAPTURE_STA = 0;
      TIM5CH2_CAPTURE_VAL = 0;
      TIM5CH2_CAPTURE_STA |= 0X40; //将STA置为0x40 当下一次触发中断时，会进入上面的if语句
      __HAL_TIM_DISABLE(&htim5);
      __HAL_TIM_SET_COUNTER(&htim5, 0);
      TIM_RESET_CAPTUREPOLARITY(&htim5, TIM_CHANNEL_2);
      TIM_SET_CAPTUREPOLARITY(&htim5, TIM_CHANNEL_2, TIM_ICPOLARITY_FALLING);
      __HAL_TIM_ENABLE(&htim5);
    }
  }
	/***************************************************T5C3_ECHO3*******************************************************/	
		if ((TIM5CH3_CAPTURE_STA & 0X80) == 0) //还未捕获成功  [7]:0,没有成功的捕获;1,成功捕获到一次.
  {
    if (TIM5CH3_CAPTURE_STA & 0X40) // 成功率捕获到1个下降沿  [6]:0,还没捕获到低电平;1,已经捕获到低电平了.
    {
      // usart_printf("get down\r\n");
      TIM5CH3_CAPTURE_STA |= 0X80;                                            // 标记成功，捕获到1次高电平完成
      TIM5CH3_CAPTURE_VAL = HAL_TIM_ReadCapturedValue(&htim5, TIM_CHANNEL_3); // 捕获当前设置捕获值
      TIM_RESET_CAPTUREPOLARITY(&htim5, TIM_CHANNEL_3);                       // 清除原来设置
      TIM_SET_CAPTUREPOLARITY(&htim5, TIM_CHANNEL_3, TIM_ICPOLARITY_RISING);  // 捕获到下降沿之后，将捕获到复位为上升沿
    }
    else // 捕获到一个上升沿
    {
      // usart_printf("get up\r\n");
      TIM5CH3_CAPTURE_STA = 0;
      TIM5CH3_CAPTURE_VAL = 0;
      TIM5CH3_CAPTURE_STA |= 0X40; //将STA置为0x40 当下一次触发中断时，会进入上面的if语句
      __HAL_TIM_DISABLE(&htim5);
      __HAL_TIM_SET_COUNTER(&htim5, 0);
      TIM_RESET_CAPTUREPOLARITY(&htim5, TIM_CHANNEL_3);
      TIM_SET_CAPTUREPOLARITY(&htim5, TIM_CHANNEL_3, TIM_ICPOLARITY_FALLING);
      __HAL_TIM_ENABLE(&htim5);
    }
  }
	
	
	/***************************************************T5C4_ECHO4*******************************************************/	
		if ((TIM5CH4_CAPTURE_STA & 0X80) == 0) //还未捕获成功  [7]:0,没有成功的捕获;1,成功捕获到一次.
  {
    if (TIM5CH4_CAPTURE_STA & 0X40) // 成功率捕获到1个下降沿  [6]:0,还没捕获到低电平;1,已经捕获到低电平了.
    {
      // usart_printf("get down\r\n");
      TIM5CH4_CAPTURE_STA |= 0X80;                                            // 标记成功，捕获到1次高电平完成
      TIM5CH4_CAPTURE_VAL = HAL_TIM_ReadCapturedValue(&htim5, TIM_CHANNEL_4); // 捕获当前设置捕获值
      TIM_RESET_CAPTUREPOLARITY(&htim5, TIM_CHANNEL_4);                       // 清除原来设置
      TIM_SET_CAPTUREPOLARITY(&htim5, TIM_CHANNEL_4, TIM_ICPOLARITY_RISING);  // 捕获到下降沿之后，将捕获到复位为上升沿
    }
    else // 捕获到一个上升沿
    {
      // usart_printf("get up\r\n");
      TIM5CH4_CAPTURE_STA = 0;
      TIM5CH4_CAPTURE_VAL = 0;
      TIM5CH4_CAPTURE_STA |= 0X40; //将STA置为0x40 当下一次触发中断时，会进入上面的if语句
      __HAL_TIM_DISABLE(&htim5);
      __HAL_TIM_SET_COUNTER(&htim5, 0);
      TIM_RESET_CAPTUREPOLARITY(&htim5, TIM_CHANNEL_4);
      TIM_SET_CAPTUREPOLARITY(&htim5, TIM_CHANNEL_4, TIM_ICPOLARITY_FALLING);
      __HAL_TIM_ENABLE(&htim5);
    }
  }
	
	/***************************************************T3C3_ECHO5*******************************************************/		
	if ((TIM3CH3_CAPTURE_STA & 0X80) == 0) //还未捕获成功  [7]:0,没有成功的捕获;1,成功捕获到一次.
  {
    if (TIM3CH3_CAPTURE_STA & 0X40) // 成功率捕获到1个下降沿  [6]:0,还没捕获到低电平;1,已经捕获到低电平了.
    {
      // usart_printf("get down\r\n");
      TIM3CH3_CAPTURE_STA |= 0X80;                                            // 标记成功，捕获到1次高电平完成
      TIM3CH3_CAPTURE_VAL = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_3); // 捕获当前设置捕获值
      TIM_RESET_CAPTUREPOLARITY(&htim3, TIM_CHANNEL_3);                       // 清除原来设置
      TIM_SET_CAPTUREPOLARITY(&htim3, TIM_CHANNEL_3, TIM_ICPOLARITY_RISING);  // 捕获到下降沿之后，将捕获到复位为上升沿
    }
    else // 捕获到一个上升沿
    {
      // usart_printf("get up\r\n");
      TIM3CH3_CAPTURE_STA = 0;
      TIM3CH3_CAPTURE_VAL = 0;
      TIM3CH3_CAPTURE_STA |= 0X40; //将STA置为0x40 当下一次触发中断时，会进入上面的if语句
      __HAL_TIM_DISABLE(&htim3);
      __HAL_TIM_SET_COUNTER(&htim3, 0);
      TIM_RESET_CAPTUREPOLARITY(&htim3, TIM_CHANNEL_3);
      TIM_SET_CAPTUREPOLARITY(&htim3, TIM_CHANNEL_3, TIM_ICPOLARITY_FALLING);
      __HAL_TIM_ENABLE(&htim3);
    }
  }
	
	/***************************************************T3C4_ECHO6*******************************************************/		
	if ((TIM3CH4_CAPTURE_STA & 0X80) == 0) //还未捕获成功  [7]:0,没有成功的捕获;1,成功捕获到一次.
  {
    if (TIM3CH4_CAPTURE_STA & 0X40) // 成功率捕获到1个下降沿  [6]:0,还没捕获到低电平;1,已经捕获到低电平了.
    {
      // usart_printf("get down\r\n");
      TIM3CH4_CAPTURE_STA |= 0X80;                                            // 标记成功，捕获到1次高电平完成
      TIM3CH4_CAPTURE_VAL = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_4); // 捕获当前设置捕获值
      TIM_RESET_CAPTUREPOLARITY(&htim3, TIM_CHANNEL_4);                       // 清除原来设置
      TIM_SET_CAPTUREPOLARITY(&htim3, TIM_CHANNEL_4, TIM_ICPOLARITY_RISING);  // 捕获到下降沿之后，将捕获到复位为上升沿
    }
    else // 捕获到一个上升沿
    {
      // usart_printf("get up\r\n");
      TIM3CH4_CAPTURE_STA = 0;
      TIM3CH4_CAPTURE_VAL = 0;
      TIM3CH4_CAPTURE_STA |= 0X40; //将STA置为0x40 当下一次触发中断时，会进入上面的if语句
      __HAL_TIM_DISABLE(&htim3);
      __HAL_TIM_SET_COUNTER(&htim3, 0);
      TIM_RESET_CAPTUREPOLARITY(&htim3, TIM_CHANNEL_4);
      TIM_SET_CAPTUREPOLARITY(&htim3, TIM_CHANNEL_4, TIM_ICPOLARITY_FALLING);
      __HAL_TIM_ENABLE(&htim3);
    }
  }
	
	
}


/* USER CODE END 1 */
