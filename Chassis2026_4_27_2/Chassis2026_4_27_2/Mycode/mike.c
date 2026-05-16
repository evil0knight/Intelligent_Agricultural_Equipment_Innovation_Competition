#include "mike.h"
#include "motor.h"
#include "pid.h"
#include "encoder.h"
#include "math.h"

// 轮径系数（单位：cm / pulse）
#define COEF_LF  0.04047f
#define COEF_RF  0.04018f
#define COEF_LR  0.04020f
#define COEF_RR  0.04034f
// 有效轮距（左右轮横向中心距，单位 cm）
#define TRACK_WIDTH_CM  41.0f
// 轴距
#define WHEEL_BASE_CM   20.0f  
// 半轮距与半轴距（用于旋转分解）
#define HALF_W  (TRACK_WIDTH_CM / 2.0f)
#define HALF_L  (WHEEL_BASE_CM / 2.0f)
#define RADIUS_SUM  (HALF_W + HALF_L)
/*==================== 里程计状态 ====================*/


Odom_t g_odom = {0.0f, 0.0f, 0.0f};

/*======================================================================================*/

extern void speed_pid_control(float goal_A,float goal_B,float goal_C,float goal_D);

void mike_wheel(int x_out,int y_out,float yaw_out)
{
		int speed_A=0,speed_B=0,speed_C=0,speed_D=0;


		speed_A=x_out-y_out-yaw_out*(wheel_L1+wheel_L2);
		speed_B=x_out+y_out+yaw_out*(wheel_L1+wheel_L2);
		speed_C=x_out+y_out-yaw_out*(wheel_L1+wheel_L2);
		speed_D=x_out-y_out+yaw_out*(wheel_L1+wheel_L2);

		speed_pid_control(speed_A,speed_B,speed_C,speed_D);

}

/* 输入为角速度 */
void mike_speed2xy(float* vx,float* vy,float* v_yaw,float speedA,float speedB,float speedC,float speedD)
{
	(*vx)	 	= ( speedA+speedD + speedB+speedC )/4.0f;
	(*vy)		= (-speedA+speedB + speedC-speedD )/4.0f;
	(*v_yaw)= (-speedA+speedB - speedC+speedD )/(wheel_L1+wheel_L2)/4.0f;
}

void odom_update(int32_t dp_lf, int32_t dp_rf, int32_t dp_lr, int32_t dp_rr)
{
	  // 将脉冲增量转换为每个轮子的移动距离 (cm)
    float ds_lf = dp_lf * COEF_LF;
    float ds_rf = dp_rf * COEF_RF;
    float ds_lr = dp_lr * COEF_LR;
    float ds_rr = dp_rr * COEF_RR;
	
		float delta_x = ( ds_lf + ds_rf + ds_lr + ds_rr) * 0.25f;
    float delta_y = (-ds_lf + ds_rf + ds_lr - ds_rr) * 0.25f;
    float delta_theta = (-ds_lf + ds_rf - ds_lr + ds_rr) / (4.0f * RADIUS_SUM);
	
    float half_theta = g_odom.theta + delta_theta * 0.5f;
    float cos_half = cosf(half_theta);
    float sin_half = sinf(half_theta);	
	
		g_odom.x += delta_x * cos_half - delta_y * sin_half;
    g_odom.y += delta_x * sin_half + delta_y * cos_half;
    g_odom.theta += delta_theta;
}



void speed_pid_control(float goal_A,float goal_B,float goal_C,float goal_D)
{

		pid_motor_speed[0].set=goal_A;
		pid_motor_speed[1].set=goal_B;
		pid_motor_speed[2].set=goal_C;
		pid_motor_speed[3].set=goal_D;
	
	
		pid_motor_speed[0].fdb=E_motor.speedA;
		pid_motor_speed[1].fdb=E_motor.speedB;
		pid_motor_speed[2].fdb=E_motor.speedC;
		pid_motor_speed[3].fdb=E_motor.speedD;
	
		PID_Calc(&pid_motor_speed[0]);
		PID_Calc(&pid_motor_speed[1]);
		PID_Calc(&pid_motor_speed[2]);
		PID_Calc(&pid_motor_speed[3]);
		
		Motor_current_set(pid_motor_speed[0].out,pid_motor_speed[1].out,pid_motor_speed[2].out,pid_motor_speed[3].out);

}
