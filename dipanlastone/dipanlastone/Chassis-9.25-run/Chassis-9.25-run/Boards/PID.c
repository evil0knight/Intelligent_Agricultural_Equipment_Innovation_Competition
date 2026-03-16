
#include "pid.h"

Expect  Expectation;//期望结构体PIDStru PID_velocity;//进场结构体

PIDStru PID_Stright;//走直线
PIDStru PID_Across_yaw;//陀螺仪
PIDStru PID_Sonic;//超声波
PIDStru PID_Sonic_stable;//超声波稳定距离
PIDStru PID_Sonic_stable2;//超声波稳定距离222
PIDStru PID_Sonic_mid;//垄间行驶超声波检测
PIDStru PID_Angle_Control;
PIDStru PID_Forward_Heng_Control;
PIDStru PID_Deceleration;

float speedA, speedB, speedC, speedD;

/******************************************************************************************************************
*                              void PID_Init(PIDStru *PID,float P,float I,float D)
*
*Author      : Jianfeng Zhang 
*Description : PID初始化函数
*Arguments   : PIDStru *PID            所要初始化的PID结构体的指针
               float P          	     比例系数
							 float I     			       积分系数
							 float D				         微分系数
*Returns     : none                  
*Notes       : Lab416 for 2019国赛
*******************************************************************************************************************
*/	

void PID_Init(PIDStru *PID,float P,float I,float D)
{
	PID->P 			= P;
	PID->I 			= I;
	PID->D 			= D;
	PID->integral = 0.0f;
	PID->pout 		= 0.0f;
	PID->iout 		= 0.0f;
	PID->dout 		= 0.0f;
	PID->PMAX   	= 10000.0f;
	PID->IMAX   	= 1000.0f;
	PID->DMAX   	= 10000.0f;
	PID->Expected = 0.0f;
	PID->err		  = 0.0f;
	PID->last_err	= 0.0f;
}

float	Limit_Pwm_float(float pwm,float pwm_min,float pwm_max)
{
	if(pwm > pwm_max)			 
    pwm =  pwm_max;
	if(pwm < pwm_min)	
    pwm =  pwm_min;
	return pwm;   
}



int PID_Reduce(PIDStru *PID, float Expected, float observe_value)
	{
    PID->Expected = Expected;
	PID->err      = observe_value - PID->Expected;
	PID->pout     = PID->P * PID->err;
	PID->integral = 0.8f*PID->integral;                    //一阶低通滤波
	PID->integral+= PID->err;
	PID->integral = Limit_Pwm_float(PID->integral,-80,80);//积分项限幅(防止退饱和慢)
	PID->iout     = PID->I * PID->integral;
	PID->iout     = Limit_Pwm_float(PID->iout,-300,300);   //限幅(防止退饱和慢)
	PID->dout	    = PID->D * (PID->err - PID->last_err);
	PID->last_err = PID->err;
	PID->PIDOUT   = PID->pout + PID->iout + PID->dout;
	PID->PIDOUT   = Limit_Pwm_float(PID->PIDOUT,-500,500);
	return (int)PID->PIDOUT;
}
// 激光测距减速PID
int Control_Deceleration(float Expected, float distance)
	{
    return PID_Reduce(&PID_Deceleration, Expected, distance);
}

 






int PID_Control(PIDStru *PID,float Expected,float observe_value)
{
	PID->Expected = Expected;
	PID->err      = observe_value - PID->Expected;
	PID->pout     = PID->P * PID->err;
	PID->integral = 0.8f*PID->integral;                    //一阶低通滤波
	PID->integral+= PID->err;
	PID->integral = Limit_Pwm_float(PID->integral,-80,80);//积分项限幅(防止退饱和慢)
	PID->iout     = PID->I * PID->integral;
	PID->iout     = Limit_Pwm_float(PID->iout,-300,300);   //限幅(防止退饱和慢)
	PID->dout	    = PID->D * (PID->err - PID->last_err);
	PID->last_err = PID->err;
	PID->PIDOUT   = PID->pout + PID->iout + PID->dout;
	PID->PIDOUT   = Limit_Pwm_float(PID->PIDOUT,-500,500);
	return (int)PID->PIDOUT;
}

int Control_Stright(float Expected,float angle)
{
return 	PID_Control(&PID_Stright,Expected,angle);
}

/****陀螺仪速度环*///////
int PID_Control_yaw(PIDStru *PID,float Expected,float observe_value)
{
	PID->Expected = Expected;
	PID->err      = observe_value - PID->Expected;
	PID->pout     = PID->P * PID->err;
	PID->integral = 0.8f*PID->integral;//一阶低通滤波
	PID->integral+= PID->err;
	PID->integral = Limit_Pwm_float(PID->integral,-800,800);//积分项限幅(防止退饱和慢)
	PID->iout     = PID->I * PID->integral;
	PID->iout     = Limit_Pwm_float(PID->iout,-800,800);   //限幅(防止退饱和慢)
	PID->dout	    = PID->D * (PID->err - PID->last_err);
	PID->last_err = PID->err;
	PID->PIDOUT   = PID->pout + PID->iout + PID->dout;
  PID->PIDOUT   = Limit_Pwm_float(PID->PIDOUT,-1000,1000);
	return (int)PID->PIDOUT;
}

int Control_Across_yaw(float Expected,float angle)
{
return 	PID_Control_yaw(&PID_Across_yaw,Expected,angle);
}


int PID_Control_sonic(PIDStru *PID,float Expected,float observe_value)//编码器速度环
{
	PID->Expected = Expected;
	PID->err      = -observe_value+PID->Expected;
	PID->pout     = PID->P * PID->err;
	PID->integral = 0.8f*PID->integral;//一阶低通滤波
	PID->integral+= PID->err;
	PID->integral = Limit_Pwm_float(PID->integral,-100,100);//积分项限幅(防止退饱和慢)
	PID->iout     = PID->I * PID->integral;
	PID->iout     = Limit_Pwm_float(PID->iout,-100,100);   //限幅(防止退饱和慢)
	PID->dout	    = PID->D * (PID->err - PID->last_err);
//	PID->dout	    = PID->D * PID->err ;
	PID->last_err = PID->err;
	PID->PIDOUT   = PID->pout + PID->iout + PID->dout;
  PID->PIDOUT   = Limit_Pwm_float(PID->PIDOUT,-80,80);//总输出限幅
	return (int)PID->PIDOUT;
}

int Control_sonic(float Expected,float Dis)
{
return 	PID_Control_sonic(&PID_Sonic,Expected,Dis);
}


int PID_Control_sonic_stable(PIDStru *PID,float Expected,float observe_value)//超声波
{
	PID->Expected = Expected;
	PID->err      = -observe_value+PID->Expected;
	PID->pout     = PID->P * PID->err;
	PID->integral = 0.8f*PID->integral;//一阶低通滤波
	PID->integral+= PID->err;
	PID->integral = Limit_Pwm_float(PID->integral,-1000,1000);//积分项限幅(防止退饱和慢)
	PID->iout     = PID->I * PID->integral;
	PID->iout     = Limit_Pwm_float(PID->iout,-1000,1000);   //限幅(防止退饱和慢)
	PID->dout	    = PID->D * (PID->err - PID->last_err);
//	PID->dout	    = PID->D * PID->err ;
	PID->last_err = PID->err;
	PID->PIDOUT   = PID->pout + PID->iout + PID->dout;
  PID->PIDOUT   = Limit_Pwm_float(PID->PIDOUT,-100,100);//总输出限幅
	return (int)PID->PIDOUT;
}

int Control_sonic_stable(float Expected,float Dis)
{
return 	PID_Control_sonic_stable(&PID_Sonic_stable,Expected,Dis);
}

int PID_Control_sonic_stable2(PIDStru *PID,float Expected,float observe_value)//超声波
{
	PID->Expected = Expected;
	PID->err      = -observe_value+PID->Expected;
	PID->pout     = PID->P * PID->err;
	PID->integral = 0.8f*PID->integral;//一阶低通滤波
	PID->integral+= PID->err;
	PID->integral = Limit_Pwm_float(PID->integral,-1000,1000);//积分项限幅(防止退饱和慢)
	PID->iout     = PID->I * PID->integral;
	PID->iout     = Limit_Pwm_float(PID->iout,-1000,1000);   //限幅(防止退饱和慢)
	PID->dout	    = PID->D * (PID->err - PID->last_err);
//	PID->dout	    = PID->D * PID->err ;
	PID->last_err = PID->err;
	PID->PIDOUT   = PID->pout + PID->iout + PID->dout;
  PID->PIDOUT   = Limit_Pwm_float(PID->PIDOUT,-80,80);//总输出限幅
	return (int)PID->PIDOUT;
}

int Control_sonic_stable2(float Expected,float Dis)
{
return 	PID_Control_sonic_stable2(&PID_Sonic_stable,Expected,Dis);
}



int PID_Control_sonic_mid(PIDStru *PID,float Expected,float observe_value)//超声波
{
	PID->Expected = Expected;
	PID->err      = -observe_value+PID->Expected;
	PID->pout     = PID->P * PID->err;
	PID->integral = 0.8f*PID->integral;//一阶低通滤波
	PID->integral+= PID->err;
	PID->integral = Limit_Pwm_float(PID->integral,-1000,1000);//积分项限幅(防止退饱和慢)
	PID->iout     = PID->I * PID->integral;
	PID->iout     = Limit_Pwm_float(PID->iout,-1000,1000);   //限幅(防止退饱和慢)
	PID->dout	    = PID->D * (PID->err - PID->last_err);
//	PID->dout	    = PID->D * PID->err ;
	PID->last_err = PID->err;
	PID->PIDOUT   = PID->pout + PID->iout + PID->dout;
  PID->PIDOUT   = Limit_Pwm_float(PID->PIDOUT,-80,80);//总输出限幅
	return (int)PID->PIDOUT;
}

int Control_sonic_mid(float Expected,float Dis)
{
return 	PID_Control_sonic_mid(&PID_Sonic_mid,Expected,Dis);
}




int PID_Turn_Angle(PIDStru *PID,float Expected,float observe_value)
{
	PID->Expected = Expected;
	PID->err      = observe_value - PID->Expected;
	PID->pout     = PID->P * PID->err;
	PID->integral = 0.8f*PID->integral;                    //一阶低通滤波
	PID->integral+= PID->err;
	PID->integral = Limit_Pwm_float(PID->integral,-300,300);//积分项限幅(防止退饱和慢)
	PID->iout     = PID->I * PID->integral;
	PID->iout     = Limit_Pwm_float(PID->iout,-300,300);   //限幅(防止退饱和慢)
	PID->dout	    = PID->D * (PID->err - PID->last_err);
	PID->last_err = PID->err;
	PID->PIDOUT   = PID->pout + PID->iout + PID->dout;
	PID->PIDOUT   = Limit_Pwm_float(PID->PIDOUT,-3500,3500);
	return (int)PID->PIDOUT;
}

int Turn_Angle_Control(float Expected,float angle)
{
return 	PID_Turn_Angle(&PID_Angle_Control,Expected,angle);
}


int PID_Heng_Control(PIDStru *PID,float Expected,float observe_value)
{
	PID->Expected = Expected;
	PID->err      = observe_value - PID->Expected;
	PID->pout     = PID->P * PID->err;
	PID->integral = 0.8f*PID->integral;                    //一阶低通滤波
	PID->integral+= PID->err;
	PID->integral = Limit_Pwm_float(PID->integral,-300,300);//积分项限幅(防止退饱和慢)
	PID->iout     = PID->I * PID->integral;
	PID->iout     = Limit_Pwm_float(PID->iout,-300,300);   //限幅(防止退饱和慢)
	PID->dout	    = PID->D * (PID->err - PID->last_err);
	PID->last_err = PID->err;
	PID->PIDOUT   = PID->pout + PID->iout + PID->dout;
	PID->PIDOUT   = Limit_Pwm_float(PID->PIDOUT,-500,500);
	return (int)PID->PIDOUT;
}

int Heng_Control(float  Expected,float angle)
{
return 	PID_Heng_Control(&PID_Forward_Heng_Control,Expected,angle);
}
