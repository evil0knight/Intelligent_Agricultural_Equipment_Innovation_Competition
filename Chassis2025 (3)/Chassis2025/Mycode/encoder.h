#ifndef __ENCODER_H__
#define __ENCODER_H__

#include "stm32f4xx_hal.h"

//编码器滤波结构体
typedef struct
{
	unsigned char 	Complete;
	unsigned char   Count;
	float   				Cache;	        //计算中间缓存值
	float 		 			Last_Data;	    //上一次捕获值
	float  					Data_His[10];		//数据历史值
}EncoderFilter_Struct;

typedef struct
{
	float	speedA;
	float	speedB;
	float	speedC;
	float	speedD;
	
}EncoderStruct;

typedef struct
{
	float	vx;
	float	vy;
	float	yaw;
	
}CarStruct;

extern EncoderStruct E_motor,E_motor_F;
extern CarStruct Car_speed,Car_s;//转速  x,y方向累计路程S

void Get_encoder(void);
void Encoder_calc(void);
void Encoder_Clear(void);
void Encoder_Filter(EncoderFilter_Struct *filterStruct,float newDatas,unsigned char FilterNum,float *Out);


#endif

