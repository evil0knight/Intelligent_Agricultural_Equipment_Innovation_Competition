#ifndef _MOTOR_H
#define _MOTOR_H

#include "stm32f4xx_hal.h"
void Motor_current_set(int motoA,int speedB,int speedC, int speedD);
void Working_motor_set(int motor1,int motor2);
#endif
