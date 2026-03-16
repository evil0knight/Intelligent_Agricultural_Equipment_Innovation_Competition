#include "mike.h"
#include "motor.h"
#include "pid.h"
#include "encoder.h"

extern void speed_pid_control(float goal_A,float goal_B,float goal_C,float goal_D);

void mike_wheel(int x_out,int y_out,float yaw_out)
{
		int speed_A=0,speed_B=0,speed_C=0,speed_D=0;


		speed_A=x_out-y_out-yaw_out*(wheel_L1+wheel_L2);
		speed_B=x_out+y_out+yaw_out*(wheel_L1+wheel_L2);
		speed_C=x_out+y_out-yaw_out*(wheel_L1+wheel_L2);
		speed_D=x_out-y_out+yaw_out*(wheel_L1+wheel_L2);

		speed_pid_control(speed_A,speed_B,speed_C,speed_D);
	
//	speed_1=(1/wheel_R)*(vy+vx-yaw*(wheel_L1+wheel_L2));
//	speed_2=(1/wheel_R)*(vy-vx+yaw*(wheel_L1+wheel_L2));
//	speed_3=(1/wheel_R)*(vy-vx-yaw*(wheel_L1+wheel_L2));
//	speed_4=(1/wheel_R)*(vy+vx+yaw*(wheel_L1+wheel_L2));


//	Motor_current_set(speed_2,speed_1,speed_4,speed_3);


}

/* 输入为角速度 */
void mike_speed2xy(float* vx,float* vy,float* v_yaw,float speedA,float speedB,float speedC,float speedD)
{
	(*vx)	 	= ( speedA+speedD + speedB+speedC )/4.0f;
	(*vy)		= (-speedA+speedB + speedC-speedD )/4.0f;
	(*v_yaw)= (-speedA+speedB - speedC+speedD )/(wheel_L1+wheel_L2)/4.0f;
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
