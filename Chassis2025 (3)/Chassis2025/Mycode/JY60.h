#ifndef __JY60_H__
#define __JY60_H__

#include "stm32f4xx_hal.h"


typedef struct
{
	uint8_t frame1; 	  	// 0x55
	uint8_t frame2;			// 0x53(½Ç¶È)
	int16_t Roll;
	int16_t Pitch;
	int16_t Yaw;
	uint16_t T; 
	uint8_t  Sum;
	
} Angle;



typedef union
{
	Angle angle;
	uint8_t buf[11];
}Angle_buf;


void JY_writedata(uint8_t* buf);

#endif
