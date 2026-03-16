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




uint8_t u_f_rx_buffer[BUFLENGTH],u_b_rx_buffer[BUFLENGTH],u_l_rx_buffer[BUFLENGTH],u_r_rx_buffer[BUFLENGTH];									


uint8_t rx_buf[4][2];
uint8_t yaw_rx_buf[33],yaw_rx_buffer[BUFLENGTH];
int16_t yaw_temp;

void  AUTO_uartDma_init(void)
{
	 /* 前 */
		__HAL_UART_ENABLE_IT(&huart4, UART_IT_IDLE);											//使能串口空闲中断（两次消息的间隙会触发）
		HAL_UART_Receive_DMA(&huart4, u_f_rx_buffer,BUFLENGTH);
	 /* 后 */		
		__HAL_UART_ENABLE_IT(&huart5, UART_IT_IDLE);											//使能串口空闲中断（两次消息的间隙会触发）
		HAL_UART_Receive_DMA(&huart5, u_b_rx_buffer,BUFLENGTH);
	 /* 左 */	
		__HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);											//使能串口空闲中断（两次消息的间隙会触发）
		HAL_UART_Receive_DMA(&huart2, u_l_rx_buffer,BUFLENGTH);
	 /* 右 */	
	  __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);											//使能串口空闲中断（两次消息的间隙会触发）
		HAL_UART_Receive_DMA(&huart3, u_r_rx_buffer,BUFLENGTH);
	 /* YAW */	
	  __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);											//使能串口空闲中断（两次消息的间隙会触发）
		HAL_UART_Receive_DMA(&huart1, yaw_rx_buffer,BUFLENGTH);
}

/* 超声波接收 */
void U_F_Receive_IDLE(void)
{
	uint8_t data_length;
	HAL_UART_DMAStop(&huart4);																					//关闭DMA
  data_length  = BUFLENGTH - __HAL_DMA_GET_COUNTER(&hdma_uart4_rx);  //128-剩余空间 得到已经接收的数量
	if( data_length == 4 )
	{
		memcpy(&rx_buf[0],&u_f_rx_buffer[1],2);											
		ultra.F =(rx_buf[0][0] << 8) | rx_buf[0][1];// ？大端数据？
	}
	
  memset(u_f_rx_buffer,0,BUFLENGTH);																	//清除缓存数组
	HAL_UART_Receive_DMA(&huart4,u_f_rx_buffer,BUFLENGTH);
}

void U_B_Receive_IDLE(void)
{
	uint8_t data_length;
	HAL_UART_DMAStop(&huart5);																					//关闭DMA
  data_length  = BUFLENGTH - __HAL_DMA_GET_COUNTER(&hdma_uart5_rx);  //128-剩余空间 得到已经接收的数量
	if( data_length == 4 )
	{
//		memcpy(&rx_buf[1],&u_b_rx_buffer[1],2);											
//		ultra.B =(rx_buf[1][0] << 8) | rx_buf[1][1];
	}
	
  memset(u_b_rx_buffer,0,BUFLENGTH);																	//清除缓存数组
	HAL_UART_Receive_DMA(&huart5,u_b_rx_buffer,BUFLENGTH);
}
uint8_t data_length;
void U_L_Receive_IDLE(void)
{
	
	HAL_UART_DMAStop(&huart2);																					//关闭DMA
  data_length  = BUFLENGTH - __HAL_DMA_GET_COUNTER(&hdma_usart2_rx);  //128-剩余空间 得到已经接收的数量
	if( data_length == 4 )
	{
//		memcpy(&rx_buf[2],&u_l_rx_buffer[1],2);											
//		ultra.L =(rx_buf[2][0] << 8) | rx_buf[2][1];
	}
	
  memset(u_l_rx_buffer,0,BUFLENGTH);																	//清除缓存数组
	HAL_UART_Receive_DMA(&huart2,u_l_rx_buffer,BUFLENGTH);
}

void U_R_Receive_IDLE(void)
{
	uint8_t data_length;
	HAL_UART_DMAStop(&huart3);																					//关闭DMA
  data_length  = BUFLENGTH - __HAL_DMA_GET_COUNTER(&hdma_usart3_rx);  //128-剩余空间 得到已经接收的数量
	if( data_length == 4 )
	{
//		memcpy(&rx_buf[3],&u_r_rx_buffer[1],2);											
//		ultra.R =(rx_buf[3][0] << 8) | rx_buf[3][1];
	}
	
  memset(u_r_rx_buffer,0,BUFLENGTH);																	//清除缓存数组
	HAL_UART_Receive_DMA(&huart3,u_r_rx_buffer,BUFLENGTH);
}
/* YAW */

void YAW_Receive_IDLE(void)
{
	uint8_t YAW_data_length;
	HAL_UART_DMAStop(&huart1);																					//关闭DMA
  YAW_data_length  = BUFLENGTH - __HAL_DMA_GET_COUNTER(&hdma_usart1_rx);  //128-剩余空间 得到已经接收的数量
	if( YAW_data_length == 33 )
	{
			memcpy(&yaw_rx_buf[0],yaw_rx_buffer,33);	
			JY_writedata(yaw_rx_buf);
	}
	
  memset(yaw_rx_buffer,0,BUFLENGTH);																	//清除缓存数组
	HAL_UART_Receive_DMA(&huart1,yaw_rx_buffer,BUFLENGTH);
}
/* 板间通信 */


