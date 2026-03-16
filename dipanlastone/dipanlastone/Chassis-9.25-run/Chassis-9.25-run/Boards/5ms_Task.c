#include "5ms_Task.h"
#include "PID.h"
#include "Bujin.h"

extern uint8_t Motor_Mode;
extern uint16_t tim;
extern int a;

void Task5ms(void)
{
	tim++;
}
