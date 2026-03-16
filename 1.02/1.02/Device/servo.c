#include "servo.h"

extern gimbal_move_t gimbal;

void gimbal_init(gimbal_move_t* gimbal)
{
	gimbal->up_date_tx[0] = 0xFE;
	gimbal->up_date_tx[1] = 0x00;
	gimbal->up_date_tx[2] = 0xFF;
	gimbal->status = 0;
	gimbal->state = 0;
	
}



void Set_180Angle(gimbal_move_t* gimbal)
{
	for(int i = 0; i < 4; i ++)
	{
		if(gimbal->servo_180angle[i] < Min_180Angle) gimbal->servo_180angle[i] = Min_180Angle;
		if(gimbal->servo_180angle[i] > Max_180Angle) gimbal->servo_180angle[i] = Max_180Angle;
		gimbal->servo_180pulse[i] = (uint16_t)(gimbal->servo_180angle[i] * 2000.0f / 180.0f + 500); // 500~2500 ¦Ìs
	}
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, gimbal->servo_180pulse[0]);
}

void Set_360Angle(gimbal_move_t* gimbal)             //2,4
{
	for(int i = 0; i < 2; i ++)
	{
		if(gimbal->servo_360angle[i] < Min_360Angle) gimbal->servo_360angle[i] = Min_360Angle;
		if(gimbal->servo_360angle[i] > Max_360Angle) gimbal->servo_360angle[i] = Max_360Angle;
		
		gimbal->servo_360pulse[i] = (uint16_t)(gimbal->servo_360angle[i] * 2000.0f / 360.0f + 500); // 500~2500 ¦Ìs
	}
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, gimbal->servo_360pulse[0]);    // 90-240
}


