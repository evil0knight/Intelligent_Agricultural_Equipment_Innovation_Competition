#include "pid.h"
#include "encoder.h"
#include "motor.h"
#include "Task_move.h"
#include "math.h"

#define PID_POSITION 0x01
#define PID_DELTA 0x02

PidTypeDef pid_motor_speed[4];
PidTypeDef pid_yaw,pid_ultra[4];
PidTypeDef pid_loc;


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
		static float a=0.3;
    // 更新误差值
    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->error[0]=(1-a)*(pid->set - pid->fdb)+a*pid->error[1];
	
    if (pid->mode == PID_POSITION)    // 位置型 PID 控制
    {
        // 计算比例项、积分项和微分项
        pid->Pout = pid->Kp * pid->error[0];
			
				// ====================== I项：积分分离 + 抗积分饱和 ======================
			  if(my_abs(pid->error[0])<pid->Anti_saturation)// 积分分离
				{
					pid->Iout += pid->Ki * pid->error[0];
				}

        LimitMax(pid->Iout, pid->max_iout);// 抗积分饱和，以免积分项过度累积
				
				
				
        // ====================== D项：微分先行 + 滤波（防噪声、防冲击）======================
				float dif = -(pid->fdb - pid->fdb_last);
				pid->fdb_last = pid->fdb;
				
				pid->Dbuf[1] = pid->Dbuf[0];
				pid->Dbuf[0] = 0.3f * pid->Dbuf[1] + 0.7f * dif;

				pid->Dout = pid->Kd * pid->Dbuf[0];

				// ====================== OUT ======================
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


/* 超声波 非线性·软饱和 */

float U_PID_Calc(PidTypeDef *pid)
{

 		static float a=0.3;
    // 更新误差值
    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->error[0]=(1-a)*(pid->set - pid->fdb)+a*pid->error[1];
		

    if (pid->mode == PID_POSITION)    // 位置型 PID 控制
    {
        // ====================== 分段P（平滑渐变，无突变）======================
        float err_abs = my_abs(pid->error[0]);
        float kp = pid->Kp;

        if(err_abs < 3.5f)
            kp *= err_abs / 9.0f;   // 平滑衰减，软着陆，防过冲
				
        pid->Pout = kp * pid->error[0];				
			
				
				
				
				// ====================== I项：积分分离 + 抗积分饱和 ======================
			  if(my_abs(pid->error[0])<pid->Anti_saturation)// 积分分离
				{
					pid->Iout += pid->Ki * pid->error[0];
				}
        LimitMax(pid->Iout, pid->max_iout);// 抗积分饱和，以免积分项过度累积			



				
        // ====================== D项：微分先行 + 滤波（防噪声、防冲击）======================
				float dif = -(pid->fdb - pid->fdb_last);
				pid->fdb_last = pid->fdb;
				
				pid->Dbuf[1] = pid->Dbuf[0];
				pid->Dbuf[0] = 0.3f * pid->Dbuf[1] + 0.7f * dif;

				pid->Dout = pid->Kd * pid->Dbuf[0];

				// ====================== OUT ======================
        pid->out = pid->Pout + pid->Iout + pid->Dout;
				
        LimitMax(pid->out, pid->max_out);
				
				if(my_abs(pid->out) < 5) pid->out = 0;
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






