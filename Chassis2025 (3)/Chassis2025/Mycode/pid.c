#include "pid.h"
#include "encoder.h"
#include "motor.h"

#define PID_POSITION 0x01
#define PID_DELTA 0x02

PidTypeDef pid_motor_speed[4];
PidTypeDef pid_yaw,pid_ultra[4];


float Encoder_Right;



// 限制输入值在指定范围内
#define LimitMax(input, max)   \
    {                          \
        if (input > max)       \
        {                      \
            input = max;       \
        }                      \
        else if (input < -max) \
        {                      \
            input = -max;      \
        }                      \
    }

/**
 * @brief 初始化 PID 控制器
 * 
 * @param mode 控制模式，可以是 PID_POSITION（位置型）或 PID_DELTA（增量型）
 * @param PID PID 参数数组，包括比例系数 Kp、积分系数 Ki 和微分系数 Kd
 * @param max_out 输出限幅值
 * @param max_iout 积分输出限幅值
 * @param Anti_saturation 抗积分饱和
 */
void PID_Init(PidTypeDef *pid, uint8_t mode,float P,float I,float D, float max_out, float max_iout,float Anti_saturation)
{
    pid->mode = mode;
    pid->Kp = P;
    pid->Ki = I;
    pid->Kd = D;
    pid->max_out = max_out;
    pid->max_iout = max_iout;
		pid->Anti_saturation = Anti_saturation;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->error[0] = pid->error[1] = pid->error[2] = pid->Pout = pid->Iout = pid->Dout = pid->out = 0.0f;
}

/**
 * @brief 计算 PID 控制器输出
 * 
 * @param pid PID 控制器结构体指针
 * @return float PID 控制器输出值
 */
float PID_Calc(PidTypeDef *pid)
{
		static float a=0.7;
    // 更新误差值
    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    //pid->error[0] = pid->set - pid->fdb;
    pid->error[0]=(1-a)*(pid->set - pid->fdb)+a*pid->error[1];
	
    if (pid->mode == PID_POSITION)    // 位置型 PID 控制
    {
        // 计算比例项、积分项和微分项
        pid->Pout = pid->Kp * pid->error[0];
			
			  if((pid->error[0]<pid->Anti_saturation)&&(pid->error[0]>-pid->Anti_saturation))
					pid->Iout += pid->Ki * pid->error[0];
				
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
				
        pid->Dout = pid->Kd * pid->Dbuf[0];

        // 限制积分项并计算输出值
        LimitMax(pid->Iout, pid->max_iout);
        pid->out = pid->Pout + pid->Iout + pid->Dout;
        LimitMax(pid->out, pid->max_out);
    }
		
		
		
    else if (pid->mode == PID_DELTA)    // 增量型 PID 控制
    {
        // 计算比例项、积分项和微分项
        pid->Pout = pid->Kp * (pid->error[0] - pid->error[1]);
        pid->Iout = pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]);
        pid->Dout = pid->Kd * pid->Dbuf[0];

        // 计算输出值
        pid->out += pid->Pout + pid->Iout + pid->Dout;
        LimitMax(pid->out, pid->max_out);
    }
		
		
		
    return pid->out;
}

/**
 * @brief 清除 PID 控制器状态
 * 
 * @param pid PID 控制器结构体指针
 */
void PID_clear(PidTypeDef *pid)
{
    // 将误差、微分缓存和输出值清零
    pid->error[0] = pid->error[1] = pid->error[2] = 0.0f;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->out = pid->Pout = pid->Iout = pid->Dout = 0.0f;
    pid->fdb = pid->set = 0.0f;
}


//uint8_t mark=0;
//float pid_speed_rx[3]={160,0,0};
//float pid_angle_rx[3];
//float angle;
//void Control(void)
//{
//	//Encoder_Right=(float)Read_Speed(&htim2,&angle)*6000/44/14;
//	
//	pid_angle.fdb=angle;
//	pid_angle.set=95;
//	PID_Calc(&pid_angle);
//	pid_speed.fdb=Encoder_Right;
//	pid_speed.set=pid_angle.out;
//	PID_Calc(&pid_speed);
//	//Load(pid_speed.out);

//}	



