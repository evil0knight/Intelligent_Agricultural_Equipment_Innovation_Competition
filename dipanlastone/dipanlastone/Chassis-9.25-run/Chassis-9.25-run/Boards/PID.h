#ifndef __PID_H
#define __PID_H

#include "main.h"

typedef struct
{
	float P;
	float pout;
	float I;
	float iout;
	float D;
	float dout;
	float integral;		   //积分项
	float err;       	   //偏差值
	float last_err;  	   //上次偏差值
	float Expected;  	   //期望值
	float PMAX;			   //P输出限幅值
	float IMAX;			   //I输出限幅值
	float DMAX;			   //D输出限幅值
	float PIDOUT;    	   //输出值
} PIDStru;

typedef struct//期望值结构体
{
	float Angle;
	int   Distance;
	
} Expect;



extern Expect  Expectation;//期望结构体
extern PIDStru PID_Stright;//走直线
extern PIDStru PID_Across_yaw;//陀螺仪
extern PIDStru PID_Sonic;//编码器
extern PIDStru PID_Sonic_stable;//超声波稳定距离
extern PIDStru PID_Sonic_stable2;//超声波稳定距离222
extern PIDStru PID_Sonic_mid;
extern PIDStru PID_Angle_Control;
extern PIDStru PID_Forward_Heng_Control;
extern PIDStru PID_Deceleration;
void PID_Init(PIDStru *PID,float P,float I,float D);//PID初始化，设定参数值
float	Limit_Pwm_float(float pwm,float pwm_min,float pwm_max);//输出限幅



int Control_Deceleration(float Expected, float distance);     //激光测距



int PID_Control(PIDStru *PID,float Expected,float observe_value);
int Control_Stright(float Expected,float angle);
int PID_Control_yaw(PIDStru *PID,float Expected,float observe_value);
int Control_Across_yaw(float Expected,float angle);
int PID_Control_sonic(PIDStru *PID,float Expected,float observe_value);
int Control_sonic(float Expected,float Dis);

int PID_Turn_Angle(PIDStru *PID,float Expected,float observe_value);
int Turn_Angle_Control(float Expected,float angle);

int PID_Heng_Control(PIDStru *PID,float Expected,float observe_value);
int Heng_Control(float Expected,float angle);

int PID_Control_sonic_stable(PIDStru *PID,float Expected,float observe_value);//超声波
int Control_sonic_stable(float Expected,float Dis);
int PID_Control_sonic_stable2(PIDStru *PID,float Expected,float observe_value);//超声波
int Control_sonic_stable2(float Expected,float Dis);


int PID_Control_sonic_mid(PIDStru *PID,float Expected,float observe_value);//超声波
int Control_sonic_mid(float Expected,float Dis);

int PID_T_decel(PIDStru *PID,float Expected,float observe_value);
int T_decel(int Target,int Present);
#endif

