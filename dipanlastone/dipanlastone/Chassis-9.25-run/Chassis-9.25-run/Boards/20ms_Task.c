#include "20ms_Task.h"
#include "main.h"
#include "SR04.h"
//#include "chaoshengbo.h"
extern float Distance[7];
uint8_t i=5;

float LastDis[7];

void Task20ms(void)
{
//	CHAOSHENGBO_Get();

//	if((LastDis[6]=getSR04Distance_6()<10.0f))
//		{
//	  Distance[6]=LastDis[6];
//		}
	if(i<6) i++;
	else i=5;
	
	switch(i)
	{
		case 1:	HCSR_04_1();
						if((Distance[1]=getSR04Distance_1())>10.0f)
						{
							Distance[1]=7.0f;
						}
						break;
		case 2:	HCSR_04_2();
						if((LastDis[2]=getSR04Distance_2())<10.0f)
						{
							Distance[2]=LastDis[2];
							//Distance[2]=3.5f;
							//Distance[2]=0.9*LastDis[2];////////
						}
						break;
		case 3:	HCSR_04_3();
						if((Distance[3]=getSR04Distance_3())>10.0f)
						{
							Distance[3]=3.5f;
						}
						break;
		case 4:	HCSR_04_4();
						if((LastDis[4]=getSR04Distance_4())<10.0f)
						{
							Distance[4]=LastDis[4];
						}
						break;
		case 5:	HCSR_04_5();
						if((Distance[5]=getSR04Distance_5())>10.0f)
						{
							Distance[5]=3.5f;
						}
						break;
		case 6:	HCSR_04_6();
						if((LastDis[6]=getSR04Distance_6())<10.0f)
						{
							Distance[6]=LastDis[6];
						}
						break;
	}
}
