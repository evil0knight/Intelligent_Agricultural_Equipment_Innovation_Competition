#ifndef __SR04_H
#define __SR04_H

#include "stm32f4xx_hal.h"

#define F_cap 0
#define B_cap 1
#define L_cap 2
#define R_cap 3



#define F_trig_h     	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_14, GPIO_PIN_SET)    
#define F_trig_l     	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_14, GPIO_PIN_RESET)   

#define B_trig_h     	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_15, GPIO_PIN_SET)    
#define B_trig_l     	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_15, GPIO_PIN_RESET) 

#define L_trig_h     	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_10, GPIO_PIN_SET)    
#define L_trig_l     	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_10, GPIO_PIN_RESET)   

#define R_trig_h     	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_11, GPIO_PIN_SET)    
#define R_trig_l     	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_11, GPIO_PIN_RESET)   


#define TRIG_F					{F_trig_l;Delay_ULTRA(40);F_trig_h;}
#define TRIG_B					{B_trig_l;Delay_ULTRA(40);B_trig_h;}
#define TRIG_L					{L_trig_l;Delay_ULTRA(40);L_trig_h;}
#define TRIG_R					{R_trig_l;Delay_ULTRA(40);R_trig_h;}

typedef struct
{
	unsigned char 	Complete;
	unsigned char   Count;
	float   				Cache;	        //计算中间缓存值
	float 		 			Last_Data;	    //上一次捕获值
	float  					Data_His[15];		//数据历史值
}UltrasonicFilter_Struct;

typedef struct
{
	uint8_t cap_state;    /* 输入捕获状态 */
	uint32_t cap_count;    /* 输入捕获值 */
	
	uint16_t start_count;
	uint16_t end_count;
	
}ul_cap_struct;

void U_F_cap_Receive(uint8_t mark);
void U_B_cap_Receive(uint8_t mark);
void U_L_cap_Receive(uint8_t mark);
void U_R_cap_Receive(uint8_t mark);



void Delay_ULTRA(unsigned char n);

#endif

