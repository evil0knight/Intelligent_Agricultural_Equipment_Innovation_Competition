#include "encoder.h"
#include "mike.h"
#include "tim.h"


EncoderStruct E_motor,E_motor_F;
CarStruct Car_speed,Car_s;//转速  x,y方向累计路程S

EncoderFilter_Struct EncoAspeed_Filter,EncoBspeed_Filter,EncoCspeed_Filter,EncoDspeed_Filter;

float angle;

/* 定时器调用 10ms */
void Get_encoder(void)
{
		Encoder_calc();
		mike_speed2xy(&Car_speed.vx,&Car_speed.vy,&Car_speed.yaw,E_motor.speedA,E_motor.speedB,E_motor.speedC,E_motor.speedD);
		
		Car_s.vx +=Car_speed.vx*6.28f*9.6f/60.0f*0.01f;
		Car_s.vy +=Car_speed.vy*6.28f*9.6f/60.0f*0.01f;
}

short temp[4]={0,0,0,0};
void Encoder_calc(void)
{
		
    temp[0] =(short)__HAL_TIM_GetCounter(&htim1);
		temp[1] =(short)__HAL_TIM_GetCounter(&htim3);
		temp[2] =(short)__HAL_TIM_GetCounter(&htim4);
		temp[3] =(short)__HAL_TIM_GetCounter(&htim5);
		/* 转速 526rpm */
		E_motor_F.speedA = (float)temp[0]*6000.0f/11/4/14;
		E_motor_F.speedB =-(float)temp[1]*6000.0f/11/4/14;
		E_motor_F.speedC =-(float)temp[2]*6000.0f/11/4/14;
		E_motor_F.speedD =-(float)temp[3]*6000.0f/11/4/14;
	
		Encoder_Filter(&EncoAspeed_Filter,E_motor_F.speedA,5,&E_motor.speedA);
		Encoder_Filter(&EncoBspeed_Filter,E_motor_F.speedB,5,&E_motor.speedB);
		Encoder_Filter(&EncoCspeed_Filter,E_motor_F.speedC,5,&E_motor.speedC);
		Encoder_Filter(&EncoDspeed_Filter,E_motor_F.speedD,5,&E_motor.speedD);
		
		//angle +=(float)temp[0]*360.0f/11/4/14;

    __HAL_TIM_SetCounter(&htim1,0);
		__HAL_TIM_SetCounter(&htim3,0);
		__HAL_TIM_SetCounter(&htim4,0);
		__HAL_TIM_SetCounter(&htim5,0);
    
}

void Encoder_Clear(void)
{
		Car_s.vx=0;
		Car_s.vy=0;
}


/******************************************************************************************************************
*             void Encoder_Filter(Filter_Struct *filterStruct,float newDatas,unsigned char FilterNum,float *Out)
*
*Description : 增强型均值滤波  去掉一个最大值和最小值
*Returns     : 当前的数据
*Notes       : 输入的数据在程序中完成缓存
*              
							*filterStruct		选择滤波的结构体
							newDatas				新数据
							FilterNum				滤波样本总数
							*Out						输出值
*******************************************************************************************************************
*/
void Encoder_Filter(EncoderFilter_Struct *filterStruct,float newDatas,unsigned char FilterNum,float *Out)
{
	unsigned int i = 0;
	float FilterSum  = 0;
	static float max,min;
	
	filterStruct->Data_His[filterStruct->Count++] = newDatas;
	
	if( filterStruct->Complete )                //已经采集完最初的AdcFilterNum个数据
	{
			max = min = filterStruct->Data_His[0];    //初始化
		//找到最大最小值
			for(i =1;i < FilterNum; i ++)
			{
					if(filterStruct->Data_His[i] > max)
					{
						max = filterStruct->Data_His[i];
					}
					
					if(filterStruct->Data_His[i] < min)
					{
						min = filterStruct->Data_His[i];
					}
			}
			
			for(i = 0; i < FilterNum; i++)
			{
					FilterSum += filterStruct->Data_His[i];
			}
			FilterSum -= max + min;
			filterStruct->Cache = FilterSum / (FilterNum-2);
	}
	
	if(filterStruct->Complete  == 1)
	{
			if(filterStruct->Cache < 5000)
			{
				*Out = filterStruct->Cache;
				filterStruct->Last_Data 	 = filterStruct->Cache;
			}
			else
				*Out = filterStruct->Last_Data;
	}
	if(filterStruct->Count == FilterNum)
	{
			filterStruct->Complete   = 1;
			filterStruct->Count = 0;
	}
	
	
	
	
}



