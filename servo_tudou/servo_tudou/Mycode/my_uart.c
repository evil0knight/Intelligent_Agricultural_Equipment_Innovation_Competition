#include "my_uart.h"
#include "usart.h"
#include "string.h"


extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_usart3_tx;

uint8_t rxCmd[3][BUFLENGTH] = {0};
								

void  AUTO_uartDma_init(void)
{

	  __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);											//使能串口空闲中断（两次消息的间隙会触发）
		HAL_UART_Receive_DMA(&huart1, rxCmd[0],BUFLENGTH);

		__HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);											//使能串口空闲中断（两次消息的间隙会触发）
		HAL_UART_Receive_DMA(&huart2, rxCmd[1],BUFLENGTH);

	  __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);											//使能串口空闲中断（两次消息的间隙会触发）
		HAL_UART_Receive_DMA(&huart3, rxCmd[2],BUFLENGTH);

}


void Receive_IDLE(void)
{
	uint8_t data_length;
	HAL_UART_DMAStop(&huart1);																					//关闭DMA
  data_length  = BUFLENGTH - __HAL_DMA_GET_COUNTER(&hdma_usart1_rx);  //128-剩余空间 得到已经接收的数量
//	if( data_length == 4 )
//	{
//		memcpy(&rx_buf[0],&u_f_rx_buffer[1],2);											

//	}
	
  memset(rxCmd[0],0,BUFLENGTH);																	//清除缓存数组
	HAL_UART_Receive_DMA(&huart1,rxCmd[0],BUFLENGTH);
}

/* 板间通信 */

//void U_F_Receive_IDLE(void)
//{
//	uint8_t data_length;
//	HAL_UART_DMAStop(&huart4);																					//关闭DMA
//  data_length  = BUFLENGTH - __HAL_DMA_GET_COUNTER(&hdma_uart4_rx);  //128-剩余空间 得到已经接收的数量
//	if( data_length == 4 )
//	{
//		memcpy(&rx_buf[0],&u_f_rx_buffer[1],2);											
//		ultra.F =(rx_buf[0][0] << 8) | rx_buf[0][1];// ？大端数据？
//	}
//	
//  memset(u_f_rx_buffer,0,BUFLENGTH);																	//清除缓存数组
//	HAL_UART_Receive_DMA(&huart4,u_f_rx_buffer,BUFLENGTH);
//}
