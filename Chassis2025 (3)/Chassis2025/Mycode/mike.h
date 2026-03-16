#ifndef __MIKE_H
#define __MIKE_H

#include "motor.h"
#define wheel_R 1
#define wheel_L1 6.7
#define wheel_L2 11.4

void mike_wheel(int y_out,int x_out,float yaw_out);
void mike_speed2xy(float* vx,float* vy,float* v_yaw,float speedA,float speedB,float speedC,float speedD);
#endif

