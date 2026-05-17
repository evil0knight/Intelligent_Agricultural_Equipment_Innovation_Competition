#ifndef __ENCODER_H__
#define __ENCODER_H__

#include "stm32f4xx_hal.h"

/* ------------------------------------------------------------
 *  滤波器状态结构 —— 每个轮子一个实例
 *  采用环形缓冲实现滑动窗口，避免每次大量搬移数据
 * ------------------------------------------------------------ */
typedef struct
{
    unsigned char Count;     // 环形缓冲当前写入位置 (0~FilterNum-1)
    unsigned char Complete;  // 缓冲是否已首次填满（0=启动阶段，1=正常滤波）
    float Last_Data;         // 上一次的原始/输出值（保留作调试参考）
    float Last_Valid;        // 上一次"合法"输出（限幅后），斜率限幅以它为基准
    float Data_His[32];      // 历史数据缓冲（最大支持 32 点窗口）
} EncoderFilter_Struct;

/* 四个麦克纳姆轮的转速 (rpm)，约定车前进为正方向 */
typedef struct
{
	float	speedA;
	float	speedB;
	float	speedC;
	float	speedD;

}EncoderStruct;

/* 车体运动状态：vx/vy 平移速度，yaw 旋转速度（或航向） */
typedef struct
{
	float	vx;
	float	vy;
	float	yaw;

}CarStruct;

extern EncoderStruct E_motor,E_motor_F;	// E_motor=滤波后供控制环用；E_motor_F=原始值供调试
extern CarStruct Car_speed,Car_s;		// Car_speed=当前车速；Car_s=累计位移
extern EncoderFilter_Struct EncoAspeed_Filter,EncoBspeed_Filter,EncoCspeed_Filter,EncoDspeed_Filter;

void Get_encoder(void);				// 顶层：10ms 调用，更新转速 + 车体速度
void Encoder_calc(void);			// 读编码器 + 算转速 + 滤波 + 里程计
void Encoder_Clear(void);			// 里程计清零
void Encoder_Filter_Init(EncoderFilter_Struct *filter);	// 滤波器初始化（上电调一次）
/* 主滤波函数：环形去极值均值 + 幅值限幅 + 斜率限幅 */
void Encoder_Filter(EncoderFilter_Struct *filterStruct, float newDatas, unsigned char FilterNum, unsigned char RemoveNum,float MaxAmplitude,float MaxSlope,float *Out);

#endif

