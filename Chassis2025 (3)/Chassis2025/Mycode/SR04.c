#include "SR04.h"
#include "Task_move.h"
#include "tim.h"

extern void Ultrasonic_Filter(UltrasonicFilter_Struct *filterStruct,float newDatas,unsigned char FilterNum,float *Out);
UltrasonicFilter_Struct 	Ultrasonic_L_Filter = {0,0,0,0,{0}};
UltrasonicFilter_Struct 	Ultrasonic_R_Filter = {0,0,0,0,{0}};
UltrasonicFilter_Struct 	Ultrasonic_F_Filter = {0,0,0,0,{0}};
UltrasonicFilter_Struct 	Ultrasonic_B_Filter = {0,0,0,0,{0}};


ul_cap_struct ultra_cap[4];

float Limit(float distance)
{
	if(distance>255)		 	distance=255;
	else if(distance<0)		distance=0;
	
	return distance;
}

void U_F_cap_Receive(uint8_t mark)
{

		if ((ultra_cap[F_cap].cap_state & 0X80) == 0)                /* 还未成功捕获 */
		{
				if(mark==0)
				{
			
						if (ultra_cap[F_cap].cap_state & 0X40)                   /* 捕获下降沿 64 */
						{
								ultra_cap[F_cap].cap_state |= 0X80;                  /* 标记成功捕获到一次高电平脉宽 128 */
							//counter = __HAL_TIM_GET_COUNTER(&htim1);
								ultra_cap[F_cap].end_count = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_1);  /* 获取当前的捕获值 */
								ultra_cap[F_cap].cap_count = ( ultra_cap[F_cap].cap_state & 0X3F )*65536 +ultra_cap[F_cap].end_count - ultra_cap[F_cap].start_count;
								
								ultra_F.F = Limit((float)ultra_cap[F_cap].cap_count / 57.5f);
								Ultrasonic_Filter(&Ultrasonic_F_Filter,ultra_F.F,7,&ultra.F);
								ultra_cap[F_cap].cap_state = 0;                    /* 清空 */
							
								TIM_RESET_CAPTUREPOLARITY(&htim2, TIM_CHANNEL_1);                      /* 一定要先清除原来的设置 */
								TIM_SET_CAPTUREPOLARITY(&htim2, TIM_CHANNEL_1, TIM_ICPOLARITY_RISING); /* 配置TIM5通道1上升沿捕获 */
						}
						else /* 捕获上升沿 */
						{								
								ultra_cap[F_cap].cap_state |= 0X40;                          /* 标记捕获到了上升沿 */
								
								ultra_cap[F_cap].start_count = HAL_TIM_ReadCapturedValue(&htim2,TIM_CHANNEL_1);
							
								TIM_RESET_CAPTUREPOLARITY(&htim2, TIM_CHANNEL_1);   										/* 一定要先清除原来的设置！！ */
								TIM_SET_CAPTUREPOLARITY(&htim2, TIM_CHANNEL_1, TIM_ICPOLARITY_FALLING); /* 定时器5通道1设置为下降沿捕获 */
						}
				}
				else if(mark==1)
				{
						if (ultra_cap[F_cap].cap_state & 0X40)
						{
								if((ultra_cap[F_cap].cap_state & 0x02) == 0x02)
								{
									ultra_cap[F_cap].cap_state = 0;
									TIM_SET_CAPTUREPOLARITY(&htim2, TIM_CHANNEL_1, TIM_ICPOLARITY_RISING);
									ultra_cap[F_cap].cap_count = 0xFFFFFFFF;
								}
								else 
								{
										ultra_cap[F_cap].cap_state++;
									
								}
						}
				}
		}
}


void U_B_cap_Receive(uint8_t mark)
{
		if ((ultra_cap[B_cap].cap_state & 0X80) == 0)                /* 还未成功捕获 */
		{
				if(mark==0)
				{
			
						if (ultra_cap[B_cap].cap_state & 0X40)                   /* 捕获下降沿 64 */
						{
								ultra_cap[B_cap].cap_state |= 0X80;                  /* 标记成功捕获到一次高电平脉宽 128 */
							//counter = __HAL_TIM_GET_COUNTER(&htim1);
								ultra_cap[B_cap].end_count = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_2);  /* 获取当前的捕获值 */
								ultra_cap[B_cap].cap_count = ( ultra_cap[B_cap].cap_state & 0X3F )*65536 +ultra_cap[B_cap].end_count - ultra_cap[B_cap].start_count;
								
								ultra_F.B = Limit((float)ultra_cap[B_cap].cap_count / 57.5f);
								Ultrasonic_Filter(&Ultrasonic_B_Filter,ultra_F.B,7,&ultra.B);
								ultra_cap[B_cap].cap_state = 0;                    /* 清空 */
								
							
								TIM_RESET_CAPTUREPOLARITY(&htim2, TIM_CHANNEL_2);                      /* 一定要先清除原来的设置 */
								TIM_SET_CAPTUREPOLARITY(&htim2, TIM_CHANNEL_2, TIM_ICPOLARITY_RISING); /* 配置TIM5通道1上升沿捕获 */
						}
						else /* 捕获上升沿 */
						{	
								ultra_cap[B_cap].cap_state |= 0X40;                          /* 标记捕获到了上升沿 */
								
								ultra_cap[B_cap].start_count = HAL_TIM_ReadCapturedValue(&htim2,TIM_CHANNEL_2);
							
								TIM_RESET_CAPTUREPOLARITY(&htim2, TIM_CHANNEL_2);   										/* 一定要先清除原来的设置！！ */
								TIM_SET_CAPTUREPOLARITY(&htim2, TIM_CHANNEL_2, TIM_ICPOLARITY_FALLING); /* 定时器5通道1设置为下降沿捕获 */
						}
				}
				else if(mark==1)
				{
					
						if (ultra_cap[B_cap].cap_state & 0X40)
						{
								if((ultra_cap[B_cap].cap_state & 0x02) == 0x02)
								{
									ultra_cap[B_cap].cap_state = 0;
									TIM_SET_CAPTUREPOLARITY(&htim2, TIM_CHANNEL_2, TIM_ICPOLARITY_RISING);
									ultra_cap[B_cap].cap_count = 0xFFFFFFFF;
								}
								else 
								{
										ultra_cap[B_cap].cap_state++;
									
								}
						}

				}
		}
}


void U_L_cap_Receive(uint8_t mark)
{
			if ((ultra_cap[L_cap].cap_state & 0X80) == 0)                /* 还未成功捕获 */
		{
				if(mark==0)
				{
			
						if (ultra_cap[L_cap].cap_state & 0X40)                   /* 捕获下降沿 64 */
						{
								ultra_cap[L_cap].cap_state |= 0X80;                  /* 标记成功捕获到一次高电平脉宽 128 */
							//counter = __HAL_TIM_GET_COUNTER(&htim1);
								ultra_cap[L_cap].end_count = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_3);  /* 获取当前的捕获值 */
								ultra_cap[L_cap].cap_count = ( ultra_cap[L_cap].cap_state & 0X3F )*65536 +ultra_cap[L_cap].end_count - ultra_cap[L_cap].start_count;
								
								ultra_F.L = Limit((float)ultra_cap[L_cap].cap_count / 57.5f);
								Ultrasonic_Filter(&Ultrasonic_L_Filter,ultra_F.L,7,&ultra.L);
								ultra_cap[L_cap].cap_state = 0;                    /* 清空 */
							
								TIM_RESET_CAPTUREPOLARITY(&htim2, TIM_CHANNEL_3);                      /* 一定要先清除原来的设置 */
								TIM_SET_CAPTUREPOLARITY(&htim2, TIM_CHANNEL_3, TIM_ICPOLARITY_RISING); /* 配置TIM5通道1上升沿捕获 */
						}
						else /* 捕获上升沿 */
						{								
								ultra_cap[L_cap].cap_state |= 0X40;                          /* 标记捕获到了上升沿 */
								
								ultra_cap[L_cap].start_count = HAL_TIM_ReadCapturedValue(&htim2,TIM_CHANNEL_3);
							
								TIM_RESET_CAPTUREPOLARITY(&htim2, TIM_CHANNEL_3);   										/* 一定要先清除原来的设置！！ */
								TIM_SET_CAPTUREPOLARITY(&htim2, TIM_CHANNEL_3, TIM_ICPOLARITY_FALLING); /* 定时器5通道1设置为下降沿捕获 */
						}
				}
				else if(mark==1)
				{
						if (ultra_cap[L_cap].cap_state & 0X40)
						{
								if((ultra_cap[L_cap].cap_state & 0x02) == 0x02)
								{
									ultra_cap[L_cap].cap_state = 0;
									TIM_SET_CAPTUREPOLARITY(&htim2, TIM_CHANNEL_3, TIM_ICPOLARITY_RISING);
									ultra_cap[L_cap].cap_count = 0xFFFFFFFF;
								}
								else 
								{
										ultra_cap[L_cap].cap_state++;
									
								}
						}
				}
		}
}


void U_R_cap_Receive(uint8_t mark)
{
			if ((ultra_cap[R_cap].cap_state & 0X80) == 0)                /* 还未成功捕获 */
		{
				if(mark==0)
				{
			
						if (ultra_cap[R_cap].cap_state & 0X40)                   /* 捕获下降沿 64 */
						{
								ultra_cap[R_cap].cap_state |= 0X80;                  /* 标记成功捕获到一次高电平脉宽 128 */
							//counter = __HAL_TIM_GET_COUNTER(&htim1);
								ultra_cap[R_cap].end_count = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_4);  /* 获取当前的捕获值 */
								ultra_cap[R_cap].cap_count = ( ultra_cap[R_cap].cap_state & 0X3F )*65536 +ultra_cap[R_cap].end_count - ultra_cap[R_cap].start_count;
								
								ultra_F.R = Limit((float)ultra_cap[R_cap].cap_count / 57.5f);
								Ultrasonic_Filter(&Ultrasonic_R_Filter,ultra_F.R,7,&ultra.R);
								ultra_cap[R_cap].cap_state = 0;                    /* 清空 */

								TIM_RESET_CAPTUREPOLARITY(&htim2, TIM_CHANNEL_4);                      /* 一定要先清除原来的设置 */
								TIM_SET_CAPTUREPOLARITY(&htim2, TIM_CHANNEL_4, TIM_ICPOLARITY_RISING); /* 配置TIM5通道1上升沿捕获 */
						}
						else /* 捕获上升沿 */
						{								
								ultra_cap[R_cap].cap_state |= 0X40;                          /* 标记捕获到了上升沿 */
								
								ultra_cap[R_cap].start_count = HAL_TIM_ReadCapturedValue(&htim2,TIM_CHANNEL_4);
							
								TIM_RESET_CAPTUREPOLARITY(&htim2, TIM_CHANNEL_4);   										/* 一定要先清除原来的设置！！ */
								TIM_SET_CAPTUREPOLARITY(&htim2, TIM_CHANNEL_4, TIM_ICPOLARITY_FALLING); /* 定时器5通道1设置为下降沿捕获 */
						}
				}
				else if(mark==1)
				{
						if (ultra_cap[R_cap].cap_state & 0X40)
						{
								if((ultra_cap[R_cap].cap_state & 0x02) == 0x02)
								{
									ultra_cap[R_cap].cap_state = 0;
									TIM_SET_CAPTUREPOLARITY(&htim2, TIM_CHANNEL_4, TIM_ICPOLARITY_RISING);
									ultra_cap[R_cap].cap_count = 0xFFFFFFFF;
								}
								else 
								{
										ultra_cap[R_cap].cap_state++;
									
								}
						}
				}
		}
}

void Delay_ULTRA(unsigned char n)
{
	unsigned int i,j;
	for(j = 0;j < n;j++)
	{
		for(i=0;i<40;i++);
	}
}


void Ultrasonic_Filter(UltrasonicFilter_Struct *filterStruct,float newDatas,unsigned char FilterNum,float *Out)
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
		filterStruct->Cache = FilterSum/(FilterNum-2);
	}
	
	if(filterStruct->Complete==1)
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
		filterStruct->Complete=1;
		filterStruct->Count = 0;
	}
}
