#ifndef __MIKE_H
#define __MIKE_H

#include "motor.h"
#define wheel_R 1
#define wheel_L1 6.7
#define wheel_L2 11.4

typedef struct {
    float x;        // 纵向位移 (cm)
    float y;        // 横向位移 (cm，规定向左为 +)
    float theta;    // 航向角 (rad)
} Odom_t;

extern Odom_t g_odom;

void mike_wheel(int y_out,int x_out,float yaw_out);
void mike_speed2xy(float* vx,float* vy,float* v_yaw,float speedA,float speedB,float speedC,float speedD);
void odom_update(int32_t dp_lf, int32_t dp_rf, int32_t dp_lr, int32_t dp_rr);
#endif

