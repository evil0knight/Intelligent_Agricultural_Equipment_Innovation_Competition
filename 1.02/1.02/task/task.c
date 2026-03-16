#include "task.h"

extern gimbal_move_t gimbal;

void Key_Check(gimbal_move_t* gimbal)
{
	if(gimbal->last_key_value[0] == 0 && gimbal->key_value[0] == 1)
	{
		gimbal->up_date_tx[1] = 0x01;  //底盘启动
		HAL_UART_Transmit_IT(&huart5, gimbal->up_date_tx, sizeof(gimbal->up_date_tx));
//		HAL_Delay(20);
		gimbal->key_value[0] = 0;
		gimbal->last_key_value[0] = gimbal->key_value[0];
	}
}

void Start_Bujin_Servo()
{
//	STEPMOTOR_MOVE(-1000,200,200,15);           //开始移动X轴
//	HAL_Delay(2000);
//	CROSMOTOR_MOVE(-3200,200,200,15);        //伸出
//	HAL_Delay(2000);
	gimbal.servo_360angle[0] = 180;
	Set_360Angle(&gimbal);
//	HAL_Delay(200);
//	LIFTMOTOR_MOVE(-3200*10,400,400,400);   //下降
//	gimbal.status = 2;
}












