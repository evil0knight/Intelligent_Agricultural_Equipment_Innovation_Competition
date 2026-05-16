#ifndef PID_H
#define PID_H

#include "stm32f4xx_hal.h"
#include "main.h"
enum PID_MODE
{
    PID_POSITION = 0,
    PID_DELTA
};

typedef struct
{
    uint8_t mode;
    //PID 三参数
    float Kp;
    float Ki;
    float Kd;

    float max_out;  //最大输出
    float max_iout; //最大积分输出
		
		//float	STEP;         		   /* 步长 */
		float	Anti_saturation;     /* 抗积分饱和 */
	
	
    float set;
    float fdb;
		float fdb_last;
    float out;
    float Pout;
    float Iout;
    float Dout;
    float Dbuf[3];  //微分项 0最新 1上一次 2上上次
    float error[3]; //误差项 0最新 1上一次 2上上次

} PidTypeDef;

extern PidTypeDef pid_motor_speed[4];
extern PidTypeDef pid_yaw,pid_ultra[4];
extern PidTypeDef pid_loc;
	
;
extern void PID_Init(PidTypeDef *pid, uint8_t mode,float P,float I,float D, float max_out, float max_iout,float Anti_saturation);

extern float PID_Calc(PidTypeDef *pid);
extern float U_PID_Calc(PidTypeDef *pid);
extern void PID_clear(PidTypeDef *pid);
extern void Control(void);
#endif
