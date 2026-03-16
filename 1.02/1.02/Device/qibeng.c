#include "message_uart.h"

void Open_qibeng()
{
	HAL_GPIO_WritePin(GPIOC, Free_IO3_Pin, GPIO_PIN_SET);
}

void Close_qibeng()
{
	HAL_GPIO_WritePin(GPIOC, Free_IO3_Pin, GPIO_PIN_RESET);
}

void Open_LED()
{
	HAL_GPIO_WritePin(GPIOC, Free_IO4_Pin, GPIO_PIN_SET);
}

void Close_LED()
{
	HAL_GPIO_WritePin(GPIOC, Free_IO4_Pin, GPIO_PIN_RESET);
}

void Open_Test()
{
	HAL_GPIO_WritePin(GPIOC, Free_IO1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, Free_IO2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, Free_IO3_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, Free_IO4_Pin, GPIO_PIN_SET);
}

void Close_Test()
{
	HAL_GPIO_WritePin(GPIOC, Free_IO1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, Free_IO2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, Free_IO3_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, Free_IO4_Pin, GPIO_PIN_RESET);
}
