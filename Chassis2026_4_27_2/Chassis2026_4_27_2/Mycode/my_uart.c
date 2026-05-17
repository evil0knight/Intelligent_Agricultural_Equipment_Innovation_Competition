#include "my_uart.h"
#include "usart.h"
#include "string.h"
#include "Task_move.h"
#include "JY60.h"

extern DMA_HandleTypeDef hdma_uart4_rx;
extern DMA_HandleTypeDef hdma_uart4_tx;
extern DMA_HandleTypeDef hdma_uart5_rx;
extern DMA_HandleTypeDef hdma_uart5_tx;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_usart3_tx;
extern DMA_HandleTypeDef hdma_usart6_rx;
extern DMA_HandleTypeDef hdma_usart6_tx;



uint8_t u_f_rx_buffer[BUFLENGTH],board_rx_buffer[BUFLENGTH];									
uint8_t work_off_mark;

uint8_t rx_buf[4][2];
uint8_t yaw_rx_buf[33],yaw_rx_buffer[BUFLENGTH];
int16_t yaw_temp;

void  AUTO_uartDma_init(void)
{

		__HAL_UART_ENABLE_IT(&huart4, UART_IT_IDLE);											//使能串口空闲中断（两次消息的间隙会触发）
		HAL_UART_Receive_DMA(&huart4, u_f_rx_buffer,BUFLENGTH);

	 /* YAW */	
	  __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);											//使能串口空闲中断（两次消息的间隙会触发）
		HAL_UART_Receive_DMA(&huart1, yaw_rx_buffer,BUFLENGTH);
}




/* YAW */
uint8_t YAW_data_length;
void YAW_Receive_IDLE(void)
{
	
	HAL_UART_DMAStop(&huart1);																					//关闭DMA
	YAW_data_length  = BUFLENGTH - __HAL_DMA_GET_COUNTER(&hdma_usart1_rx);  //128-剩余空间 得到已经接收的数量
	if( YAW_data_length == 22 )
	{
			memcpy(&yaw_rx_buf[0],yaw_rx_buffer,22);	
			JY_writedata(yaw_rx_buf);
	}
	
  memset(yaw_rx_buffer,0,BUFLENGTH);																	//清除缓存数组
	HAL_UART_Receive_DMA(&huart1,yaw_rx_buffer,BUFLENGTH);
}
/* 板间通信 */


