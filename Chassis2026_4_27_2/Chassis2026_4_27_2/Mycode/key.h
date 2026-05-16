/**
 ****************************************************************************************************
 * @file        key.h
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2020-04-19
 * @brief       按键输入 驱动代码
 * @license     Copyright (c) 2020-2032, 广州市星翼电子科技有限公司
 ****************************************************************************************************
 * @attention
 *
 * 实验平台:正点原子 MiniSTM32 V4开发板
 * 在线视频:www.yuanzige.com
 * 技术论坛:www.openedv.com
 * 公司网址:www.alientek.com
 * 购买地址:openedv.taobao.com
 *
 * 修改说明
 * V1.0 20200419
 * 第一次发布
 *
 ****************************************************************************************************
 */

#ifndef __KEY_H
#define __KEY_H

#include "stm32f4xx_hal.h"

/******************************************************************************************/
/* 引脚 定义 */

#define KEY1_INT_GPIO_PORT              	GPIOE
#define KEY1_INT_GPIO_PIN               	GPIO_PIN_2

#define KEY2_INT_GPIO_PORT              	GPIOE
#define KEY2_INT_GPIO_PIN               	GPIO_PIN_3

#define KEY3_INT_GPIO_PORT              	GPIOE
#define KEY3_INT_GPIO_PIN               	GPIO_PIN_4

#define KEY4_INT_GPIO_PORT           			GPIOD
#define KEY4_INT_GPIO_PIN            			GPIO_PIN_7
/******************************************************************************************/

#define KEY1       	HAL_GPIO_ReadPin(KEY1_INT_GPIO_PORT, KEY1_INT_GPIO_PIN)    
#define KEY2       	HAL_GPIO_ReadPin(KEY2_INT_GPIO_PORT, KEY2_INT_GPIO_PIN)    
#define KEY3      	HAL_GPIO_ReadPin(KEY3_INT_GPIO_PORT, KEY3_INT_GPIO_PIN)     
#define KEY4       	HAL_GPIO_ReadPin(KEY4_INT_GPIO_PORT, KEY4_INT_GPIO_PIN)


#endif


















