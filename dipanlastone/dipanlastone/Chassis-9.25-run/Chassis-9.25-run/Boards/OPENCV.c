#include "OPENCV.h"
#include "main.h"

extern uint8_t OPENCV_buffer[2];
extern uint8_t Color_L,Color_R;

void Color_Get(void)
{
	
	Color_L =OPENCV_buffer[0]/10;
	Color_R =OPENCV_buffer[0]%10;
	
}

