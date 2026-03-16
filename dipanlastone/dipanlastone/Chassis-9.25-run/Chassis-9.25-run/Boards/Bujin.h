#ifndef __Bujin_H__
#define __Bujin_H__

#include "main.h"


void Stop(void);
int Accel(int now_speed,int tar_speed);
void Forward_R(int err1,int err2);
void Forward_L(int err1,int err2);
void Forward_Mid_Q(int err1,int err2,int err3);
void Forward_Mid_H(int err1,int err2,int err3);
void Forward_Stable(int err1,int err2);
void Modify_ZH(int err1,int err2,int err3);//ÍÓÂÝÒÇ£¬×ó²à³¬Éù²¨£¬ºó²à³¬Éù²¨

void Heng(int err1,int err2);
void Heng1(int err1,int err2);
void Heng2(int err1,int err2);
void TurnLeft(int err);
void TurnRight(int err);
	
int Setpulse (int speed);
void MotorLQ_Ctrl(int Speed);
void MotorLH_Ctrl(int Speed);
void MotorRQ_Ctrl(int Speed);
void MotorRH_Ctrl(int Speed);
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim);

#endif
