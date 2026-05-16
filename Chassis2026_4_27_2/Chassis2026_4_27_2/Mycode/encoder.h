#ifndef __ENCODER_H__
#define __ENCODER_H__

#include "stm32f4xx_hal.h"

typedef struct
{
    unsigned char Count;     // 数据计数
    unsigned char Complete;  // 窗口填满标志
    float Last_Data;         // 上一次输出值
    float Last_Valid;        // 上一次有效有效值
    float Data_His[32];      // 历史数据缓冲区（最大32点，足够用）
} EncoderFilter_Struct;

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
extern EncoderFilter_Struct EncoAspeed_Filter,EncoBspeed_Filter,EncoCspeed_Filter,EncoDspeed_Filter;

void Get_encoder(void);
void Encoder_calc(void);
void Encoder_Clear(void);
void Encoder_Filter_Init(EncoderFilter_Struct *filter);
void Encoder_Filter(EncoderFilter_Struct *filterStruct, float newDatas, unsigned char FilterNum, unsigned char RemoveNum,float MaxAmplitude,float MaxSlope,float *Out);

#endif

