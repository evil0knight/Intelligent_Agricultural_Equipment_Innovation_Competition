#ifndef __MY_UART_H
#define __MY_UART_H
#include "main.h"
#include "string.h"



#define BUFLENGTH  		255//最大接收的数据
#define DATELENGTH		6//有效数据


void  AUTO_uartDma_init(void);

void Receive_IDLE(void);

#endif

