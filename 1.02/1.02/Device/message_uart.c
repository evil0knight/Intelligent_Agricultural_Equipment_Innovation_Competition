#include "message_uart.h"

extern gimbal_move_t gimbal;


//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
//	if(huart == &huart5)
//	{
//		if(gimbal.up_date_rx[0] == 0xFE && gimbal.up_date_rx[2] == 0xFF)
//		{
//			if(gimbal.up_date_rx[1] == 0x01)   //≤‚ ‘
//			{
//				gimbal.state = 1;
//			}
//		}
//		HAL_UART_Receive_IT(&huart5, gimbal.up_date_rx, sizeof(gimbal.up_date_rx));
//	}
//}

