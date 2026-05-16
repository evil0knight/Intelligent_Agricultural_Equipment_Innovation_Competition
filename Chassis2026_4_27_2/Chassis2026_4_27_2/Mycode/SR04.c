#include "SR04.h"
#include "Task_move.h"
#include "tim.h"
#include "string.h"



extern void Ultrasonic_Filter(UltrasonicFilter_Struct *filterStruct, float newDatas, unsigned char FilterNum, unsigned char RemoveNum,float MaxAmplitude,float MaxSlope,float *Out);
UltrasonicFilter_Struct 	Ultrasonic_L_Filter,Ultrasonic_R_Filter,Ultrasonic_F_Filter,Ultrasonic_B_Filter;


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
			\
								Ultrasonic_Filter(&Ultrasonic_F_Filter,ultra_F.F,7,1,255,30,&ultra.F);
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
				//======定时器溢出触发======
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
							
								Ultrasonic_Filter(&Ultrasonic_B_Filter,ultra_F.B,7,1,255,30,&ultra.B);
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
							
								Ultrasonic_Filter(&Ultrasonic_L_Filter,ultra_F.L,7,1,255,30,&ultra.L);
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
							
								Ultrasonic_Filter(&Ultrasonic_R_Filter,ultra_F.R,7,1,255,30,&ultra.R);
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



void Ultrasonic_Filter_Init(UltrasonicFilter_Struct *filter)
{
    if(filter == NULL) return;  // 防空指针
    
    filter->Count = 0;
    filter->Complete = 0;
    filter->Last_Data = 0.0f;
    filter->Last_Valid = 0.0f;
    // 清空历史缓冲区
    memset(filter->Data_His, 0, sizeof(filter->Data_His));
}

/**
  * 功能：增强型滑动窗口去极值平均滤波 + 幅值限幅 + 斜率限幅
  * filterStruct：滤波结构体
  * newDatas：新数据
  * FilterNum：滤波窗口大小
  * RemoveNum：各剔除最大/最小值数量
  * MaxAmplitude：最大允许幅值
  * MaxSlope：最大允许变化率
  * Out：滤波输出
  */
void Ultrasonic_Filter(UltrasonicFilter_Struct *filterStruct, float newDatas, unsigned char FilterNum, unsigned char RemoveNum,float MaxAmplitude,float MaxSlope,float *Out)
{
    unsigned int i = 0, j = 0;
    float sum=0;
    float sorted_buf[32]={0};  // 与Data_His同大，安全

    // 安全保护：空指针直接返回
    if(filterStruct == NULL || Out == NULL) return;
    
		// 安全限制
    if(FilterNum > 32)    FilterNum = 32;
    if(FilterNum < 3)     FilterNum = 3;
    if(RemoveNum >= FilterNum/2) RemoveNum = 1;
		
		// 新数据放入
    filterStruct->Data_His[filterStruct->Count++] = newDatas;
    if(filterStruct->Count >= FilterNum)
    {
        filterStruct->Count = 0;
        filterStruct->Complete = 1;  // 标记窗口已满
    }
	
		// 未满前直接输出原值，不参与滤波
    if(!filterStruct->Complete)
    {
        *Out = newDatas;
        filterStruct->Last_Data = newDatas;
        filterStruct->Last_Valid = newDatas;
        return;
    }
		
    // ===================== 环形拷贝并排序 =====================
    unsigned char index = filterStruct->Count;
    for(i = 0; i < FilterNum; i++)
    {
        sorted_buf[i] = filterStruct->Data_His[index];
        index++;
        if(index >= FilterNum) index = 0;
    }

    // 冒泡排序
    for(i = 0; i < FilterNum-1; i++)
    {
        for(j = 0; j < FilterNum-1-i; j++)
        {
            if(sorted_buf[j] > sorted_buf[j+1])
            {
                float temp = sorted_buf[j];
                sorted_buf[j] = sorted_buf[j+1];
                sorted_buf[j+1] = temp;
            }
        }
    }

    // ===================== 去极值平均 =====================
    sum = 0;
    unsigned char valid_cnt = FilterNum - 2 * RemoveNum;
    for(i = RemoveNum; i < FilterNum - RemoveNum; i++)
    {
        sum += sorted_buf[i];
    }
    float avg = sum / valid_cnt;
		

    // ==================== 4. 多级限幅（修复核心逻辑） ====================
    float output = avg;

    // 4.1 幅值限幅
    if(my_abs(avg) > MaxAmplitude)
    {
        output = filterStruct->Last_Valid;
    }
    // 4.2 斜率限幅（变化率限制）
    else if(my_abs(avg - filterStruct->Last_Valid) > MaxSlope)
    {
        if(avg > filterStruct->Last_Valid)
            output = filterStruct->Last_Valid + MaxSlope;
        else
            output = filterStruct->Last_Valid - MaxSlope;
    }

    // ? 关键修复：限幅后必须更新 Last_Valid！
    filterStruct->Last_Valid = output;
    filterStruct->Last_Data = output;
    *Out = output;
}
