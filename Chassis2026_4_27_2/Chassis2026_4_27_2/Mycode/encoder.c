#include "encoder.h"
#include "mike.h"
#include "tim.h"
#include "Task_move.h"
#include "string.h"



EncoderStruct E_motor,E_motor_F;
CarStruct Car_speed,Car_s;//转速  x,y方向累计路程S

EncoderFilter_Struct EncoAspeed_Filter,EncoBspeed_Filter,EncoCspeed_Filter,EncoDspeed_Filter;

float angle;

/* 定时器调用 10ms */
void Get_encoder(void)
{
		Encoder_calc();
		mike_speed2xy(&Car_speed.vx,&Car_speed.vy,&Car_speed.yaw,E_motor.speedA,E_motor.speedB,E_motor.speedC,E_motor.speedD);
		
		//Car_s.vx +=Car_speed.vx*6.28f*9.6f/60.0f*0.01f;
		//Car_s.vy +=Car_speed.vy*6.28f*9.6f/60.0f*0.01f;
}

short temp[3]={0,};
int32_t temp3;

short last_tim_cnt[3]={0};
int32_t last_tim_cnt3;

short current_tim_cnt[3]={0};
int32_t current_tim_cnt3;

void Encoder_calc(void)
{
		
    temp[0] =(short)__HAL_TIM_GetCounter(&htim1);
		temp[1] =(short)__HAL_TIM_GetCounter(&htim3);
		temp[2] =(short)__HAL_TIM_GetCounter(&htim4);
		temp3 	=(int32_t)__HAL_TIM_GetCounter(&htim5);
	
		for(uint8_t i=0; i<3; i++)
		{
        current_tim_cnt[i] = temp[i] - last_tim_cnt[i];
        last_tim_cnt[i] = temp[i];
    }
		current_tim_cnt3=temp3-last_tim_cnt3;
		last_tim_cnt3=temp3;
		
    if (current_tim_cnt3 >  2147483647)  current_tim_cnt3 -= 4294967296;
    if (current_tim_cnt3 < -2147483648) current_tim_cnt3 += 4294967296;
	

		
		/* 转速 526rpm */
		E_motor_F.speedA = (float)current_tim_cnt[0]*6000.0f/11/4/14;
		E_motor_F.speedB =-(float)current_tim_cnt[1]*6000.0f/11/4/14;
		E_motor_F.speedC =-(float)current_tim_cnt[2]*6000.0f/11/4/14;
		E_motor_F.speedD =-(float)current_tim_cnt3 *6000.0f/11/4/14;
	
		

		odom_update(current_tim_cnt[0],-current_tim_cnt[1],-current_tim_cnt[2],-current_tim_cnt3);
		
		Encoder_Filter(&EncoAspeed_Filter,E_motor_F.speedA,10,2,500,30,&E_motor.speedA);
		Encoder_Filter(&EncoBspeed_Filter,E_motor_F.speedB,10,2,500,30,&E_motor.speedB);
		Encoder_Filter(&EncoCspeed_Filter,E_motor_F.speedC,10,2,500,30,&E_motor.speedC);
		Encoder_Filter(&EncoDspeed_Filter,E_motor_F.speedD,10,2,500,30,&E_motor.speedD);
		
		//angle +=(float)temp[0] *360.0f/11/4/14;


}

void Encoder_Clear(void)
{

		g_odom.x=0;
		g_odom.y=0;
		g_odom.theta=0;
		
}



void Encoder_Filter_Init(EncoderFilter_Struct *filter)
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
void Encoder_Filter(EncoderFilter_Struct *filterStruct, float newDatas, unsigned char FilterNum, unsigned char RemoveNum,float MaxAmplitude,float MaxSlope,float *Out)
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


