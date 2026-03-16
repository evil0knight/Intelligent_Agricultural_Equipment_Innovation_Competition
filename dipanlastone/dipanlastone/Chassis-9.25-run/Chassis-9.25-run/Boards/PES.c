#include "PES.h"

extern uint8_t  PES[9];

void PES_Scan(void)
{
	PES[1]=HAL_GPIO_ReadPin (PES1_GPIO_Port,PES1_Pin );
	PES[2]=HAL_GPIO_ReadPin (PES2_GPIO_Port,PES2_Pin );
	PES[3]=HAL_GPIO_ReadPin (PES3_GPIO_Port,PES3_Pin );
	PES[4]=HAL_GPIO_ReadPin (PES4_GPIO_Port,PES4_Pin );
	PES[7]=HAL_GPIO_ReadPin (GPIOC, GPIO_PIN_2 );
	PES[8]=HAL_GPIO_ReadPin (GPIOC, GPIO_PIN_3 );
	
}

