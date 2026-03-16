#ifndef __SERVO_H
#define __SERVO_H

#include "main.h"
#include "tim.h"

#define Max_180Angle 180
#define Min_180Angle 0

#define Max_360Angle 240
#define Min_360Angle 90


void gimbal_init(gimbal_move_t* gimbal);

void Set_180Angle(gimbal_move_t* gimbal);

void Set_360Angle(gimbal_move_t* gimbal);       
#endif
