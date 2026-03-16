#include "WHT101.h"
#include "tim.h"

extern uint8_t YAW_buffer[30];
extern uint8_t angle_1,angle_2;
extern short angle_z;
extern float yaw_init,yaw_source,yaw;

void Yaw_Get(void)
{
	
	angle_1=YAW_buffer[18];
	angle_2=YAW_buffer[17];
	angle_z=(angle_1<<8)|angle_2;
	yaw_source=angle_z/32768.0f*180.0f;
	
}

void Yaw_Cul(void)//将获取陀螺仪数据0~360°计算转换成-180~180
{
	if(yaw_init>=0)
	{
		if(yaw_source>=-180&&yaw_source<=-180+yaw_init)
		{
		yaw=-(yaw_source+360-yaw_init);
		}
		else
		{
		yaw=-(yaw_source-yaw_init);
		}
	}
	else
	{
		if(yaw_source<=180&&yaw_source>=180+yaw_init)
		{
		yaw=-(yaw_source-360-yaw_init);
		}
		else
		{
		yaw=-(yaw_source-yaw_init);
		}
	}	
}



