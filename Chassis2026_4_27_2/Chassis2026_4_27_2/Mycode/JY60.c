#include "JY60.h"
#include "string.h"
#include "Task_move.h"

Angle_buf jy_Angle;

void JY_writedata(uint8_t* buf)
{
	memcpy(jy_Angle.buf,&buf[11],11);
		
	if(jy_Angle.buf[1] ==0x53)
	{
		yaw =(float)jy_Angle.angle.Yaw/32768.0f*180.0f;
	}
	
}


