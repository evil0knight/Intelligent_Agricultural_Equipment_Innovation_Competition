#ifndef __TASK_MOVE_H__
#define __TASK_MOVE_H__

#include "stm32f4xx_hal.h"

#define start 			0
/* Â¢ */
#define Fieldridge1 1
#define Fieldridge2 2
#define Fieldridge3 3
#define Fieldridge4 4
#define Fieldridge5 5
#define Fieldridge6 6
/* ¹ýµÀ */
#define corridor1   11
#define corridor2   12
#define corridor3   13
#define corridor4   14
#define corridor5   15
#define corridor6   16
/* ³¬Éù²¨ */
#define F_u   0
#define B_u   1
#define L_u   2
#define R_u   3
/* yaw */
#define X_y   0
#define Y_y   1

/* speed */
#define X   0
#define Y   1

#define PES_F HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_12)
#define PES_B HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_3)

typedef struct 
{
	/* Ç°ºó×óÓÒ */
	float F;
	float B;
	float L;
	float R;
}Ultra_distance;

extern float start_yaw,yaw;				//³õÊ¼ YAW
extern Ultra_distance ultra,ultra_F;			//³¬Éù¾àÀë*4

void Move_init(void);
void Move(void);
float my_abs(float x);
int speed_change(int k,int s_speed,int M_speed);
float yaw_pid_control(void);
float ultra_pid_control(uint8_t i,float goal,float fd);

#endif

