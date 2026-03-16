#ifndef __TASK_H
#define __TASK_H

#include "main.h"
#include "usart.h"
#include "StepMotor.h"
#include "servo.h"

void Key_Check(gimbal_move_t* gimbal);

void Start_Bujin_Servo();

#endif
