#include "50ms_Task.h"

#include "main.h"
#include "oled.h"
#include "20ms_Task.h"
#include "chaoshengbo.h"

//#include "PES.h"
extern float yaw;
extern uint8_t start_flag;
extern float Distance[7];


extern uint8_t  PES[9];
void Task50ms(void)
{
	
	
  CHAOSHENGBO_Get();
	CHAOSHENGBO_Get1();
  OLED_ShowString(0,0,(uint8_t*)"Flag:",16,1);
	OLED_ShowNum(48,0,start_flag,2,16,1);
	OLED_ShowString(0,17,(uint8_t*)"YAW:",16,1);
	OLED_ShowNum(33,17,yaw*100,4,16,1);
	OLED_ShowNum(0,34,1000*Distance[6],4,16,1);
	OLED_ShowNum(48,34,PES[8],1,16,1);
	OLED_ShowNum(0,51,1000*Distance[3],4,16,1);
	OLED_Refresh();
	
}

