#include "Bujin.h"
#include "tim.h"

#define Toggle_Pulse 30  //宏定义步进电机速度测试
extern int32_t Speed1_Pulse,Speed2_Pulse,Speed3_Pulse,Speed4_Pulse;
extern int32_t SetSpeed;
uint8_t EN1,EN2,EN3,EN4;

void Stop(void)
{
	MotorLQ_Ctrl(0);
	MotorLH_Ctrl(0);
	MotorRQ_Ctrl(0);
	MotorRH_Ctrl(0);
}

int Accel(int now_speed,int tar_speed)//模拟加速
{
	for(int i=now_speed ;i<=tar_speed;i++)
	{
		MotorLQ_Ctrl(SetSpeed);
		MotorLH_Ctrl(SetSpeed);
		MotorRQ_Ctrl(SetSpeed);
		MotorRH_Ctrl(SetSpeed);
	}
	return tar_speed;
	
}


void Forward_R(int err1,int err2)
{
	MotorLQ_Ctrl(SetSpeed-err1-err2);
	MotorLH_Ctrl(SetSpeed-err1+err2);
	MotorRQ_Ctrl(SetSpeed+err1+err2);
	MotorRH_Ctrl(SetSpeed+err1-err2);
}

void Forward_L(int err1,int err2)
{
	MotorLQ_Ctrl(SetSpeed-err1+err2);
	MotorLH_Ctrl(SetSpeed-err1-err2);
	MotorRQ_Ctrl(SetSpeed+err1-err2);
	MotorRH_Ctrl(SetSpeed+err1+err2);
}

void Forward_Mid_Q(int err1,int err2,int err3)//陀螺仪，左侧传感器，右侧传感器
{
	MotorLQ_Ctrl(SetSpeed-err1+err2-err3);
	MotorLH_Ctrl(SetSpeed-err1-err2-err3);
	MotorRQ_Ctrl(SetSpeed+err1-err2+err3);
	MotorRH_Ctrl(SetSpeed+err1+err2+err3);
}

void Forward_Mid_H(int err1,int err2,int err3)//陀螺仪，左侧传感器，右侧传感器
{
	MotorLQ_Ctrl(-SetSpeed-err1+err2-err3);
	MotorLH_Ctrl(-SetSpeed-err1-err2-err3);
	MotorRQ_Ctrl(-SetSpeed+err1-err2+err3);
	MotorRH_Ctrl(-SetSpeed+err1+err2+err3);
}

void Forward_Stable(int err1,int err2)
{
	MotorLQ_Ctrl(SetSpeed-err1-err2);
	MotorLH_Ctrl(SetSpeed-err1-err2);
	MotorRQ_Ctrl(SetSpeed+err1-err2);
	MotorRH_Ctrl(SetSpeed+err1-err2);
}

void Modify_ZH(int err1,int err2,int err3)//陀螺仪，左侧超声波，后侧超声波
{
	MotorLQ_Ctrl(-err1+err2+err3);
	MotorLH_Ctrl(-err1-err2+err3);
	MotorRQ_Ctrl(err1 -err2+err3);
	MotorRH_Ctrl(err1 +err2+err3);
}

void Heng(int err1,int err2)//陀螺仪，前后侧超声波
{
	MotorLQ_Ctrl(+SetSpeed-err1-err2);
	MotorLH_Ctrl(-SetSpeed-err1-err2);
	MotorRQ_Ctrl(-SetSpeed+err1-err2);
	MotorRH_Ctrl(+SetSpeed+err1-err2);
}
void Heng1(int err1,int err2)//陀螺仪，前后侧超声波
{
	MotorLQ_Ctrl(+SetSpeed+240-err1-err2);
	MotorLH_Ctrl(-SetSpeed-err1-err2);
	MotorRQ_Ctrl(-SetSpeed+err1-err2);
	MotorRH_Ctrl(+SetSpeed+240+err1-err2);
}
void Heng2(int err1,int err2)//陀螺仪，前后侧超声波
{
	MotorLQ_Ctrl(+SetSpeed+300-err1-err2);
	MotorLH_Ctrl(-SetSpeed-err1-err2);
	MotorRQ_Ctrl(-SetSpeed+err1-err2);
	MotorRH_Ctrl(+SetSpeed+300+err1-err2);
}
void TurnLeft(int err)
{
	MotorLQ_Ctrl(-err);
	MotorLH_Ctrl(-err);
	MotorRQ_Ctrl(err);
	MotorRH_Ctrl(err);
}

void TurnRight(int err)
{
	MotorLQ_Ctrl(-err);
	MotorLH_Ctrl(-err);
	MotorRQ_Ctrl(err);
	MotorRH_Ctrl(err);
}




int Setpulse (int Pulse)//速度运算代码，将输入速度转化为步进电机脉冲数
{
	int PulseOUT;
	if(Pulse>0)
	{
		if(Pulse>400)
		{
			Pulse=400;
		}
		PulseOUT=410-Pulse;
	}
	else 
	{
		PulseOUT=0;
	}
	return PulseOUT;
}

void MotorLQ_Ctrl(int Speed)//单个电机控制函数，左前电机（速度值在0~4000之间）
{
	if(Speed>10)
	{
		EN1=1;
		HAL_GPIO_WritePin (DIRA_GPIO_Port ,DIRA_Pin ,GPIO_PIN_RESET );
		Speed1_Pulse=Setpulse(Speed/10);//将速度缩小10倍，转换为范围在（0~400之内）
	}
	else if(Speed<-10)
	{
		EN1=1;
		Speed1_Pulse=Setpulse(-Speed/10);
		HAL_GPIO_WritePin (DIRA_GPIO_Port ,DIRA_Pin ,GPIO_PIN_SET );
	}
	else 
	{
		EN1=0;
		Speed1_Pulse=0;
	}
}

void MotorLH_Ctrl(int Speed)//单个电机控制函数，左后电机
{
	if(Speed>10)
	{
		EN2=1;
		HAL_GPIO_WritePin (DIRB_GPIO_Port ,DIRB_Pin ,GPIO_PIN_RESET );
		Speed2_Pulse=Setpulse(Speed/10);
	}
	else if(Speed<-10)
	{
		EN2=1;
		Speed2_Pulse=Setpulse(-Speed/10);
		HAL_GPIO_WritePin (DIRB_GPIO_Port ,DIRB_Pin ,GPIO_PIN_SET );
	}
	else 
	{
		EN2=0;
		Speed2_Pulse=0;
	}
}

void MotorRQ_Ctrl(int Speed)//单个电机控制函数，右前电机
{
	if(Speed>10)
	{
		EN3=1;
		HAL_GPIO_WritePin (DIRC_GPIO_Port ,DIRC_Pin ,GPIO_PIN_SET );
		Speed3_Pulse=Setpulse(Speed/10);
	}
	else if(Speed<-10)
	{
		EN3=1;
		Speed3_Pulse=Setpulse(-Speed/10);
		HAL_GPIO_WritePin (DIRC_GPIO_Port ,DIRC_Pin ,GPIO_PIN_RESET );
	}
	else 
	{
		EN3=0;
		Speed3_Pulse=0;
	}
}
void MotorRH_Ctrl(int Speed)//单个电机控制函数，右后电机
{
	if(Speed>10)
	{
		EN4=1;
		HAL_GPIO_WritePin (DIRD_GPIO_Port ,DIRD_Pin ,GPIO_PIN_SET );
		Speed4_Pulse=Setpulse(Speed/10);
	}
	else if(Speed<-10)
	{
		EN4=1;
		Speed4_Pulse=Setpulse(-Speed/10);
		HAL_GPIO_WritePin (DIRD_GPIO_Port ,DIRD_Pin ,GPIO_PIN_RESET );
	}
	else 
	{
		EN4=0;
		Speed4_Pulse=0;
	}
}


//void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)//输出比较中断，当定时器计数值达到CCR，翻转电平从而驱动电机
//{

//	if(htim->Instance ==TIM10 )
//	{
//		if(htim->Channel ==HAL_TIM_ACTIVE_CHANNEL_1 )
//		{
//			__IO uint16_t cout1;
//			cout1 = __HAL_TIM_GetCounter (&htim10);
//			__HAL_TIM_SET_COMPARE (&htim10 ,TIM_CHANNEL_1 ,cout1+Toggle_Pulse );
//		}
//	}
//		
////	if(htim->Instance ==TIM11 )
////	{
////		if(htim->Channel ==HAL_TIM_ACTIVE_CHANNEL_1 )
////		{
////			__IO uint16_t cout2;
////			cout2 = __HAL_TIM_GetCounter (&htim11);
////			__HAL_TIM_SET_COMPARE (&htim11 ,TIM_CHANNEL_1 ,cout2+Toggle_Pulse );
////		}
////	}
////	
////		
////	if(htim->Instance ==TIM13 )
////	{
////		if(htim->Channel ==HAL_TIM_ACTIVE_CHANNEL_1 )
////		{
////			__IO uint16_t cout3;
////			cout3 = __HAL_TIM_GetCounter (&htim13);
////			__HAL_TIM_SET_COMPARE (&htim13 ,TIM_CHANNEL_1 ,cout3+Toggle_Pulse );
////		}
////	}
////	
////	if(htim->Instance ==TIM14 )
////	{
////		if(htim->Channel ==HAL_TIM_ACTIVE_CHANNEL_1 )
////		{
////			__IO uint16_t cout4;
////			cout4 = __HAL_TIM_GetCounter (&htim14);
////			__HAL_TIM_SET_COMPARE (&htim14 ,TIM_CHANNEL_1 ,cout4+Toggle_Pulse );
////		}
////	}
//}



void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)//输出比较中断，当定时器计数值达到CCR，翻转电平从而驱动电机
{
	if(htim->Instance ==TIM1 )
	{

		if(htim->Channel ==HAL_TIM_ACTIVE_CHANNEL_1 )
		{
			if(EN1==1)
			{
			__IO uint16_t cout1;
			cout1 = __HAL_TIM_GetCounter (&htim1);
			__HAL_TIM_SET_COMPARE (&htim1 ,TIM_CHANNEL_1 ,cout1+Speed1_Pulse );//cout值会略大于Pulse值，程序在运行中计数器任然增加，因此设定Pulse不能过低
//			Pulse_count++;
			}
		}
		if(htim->Channel ==HAL_TIM_ACTIVE_CHANNEL_2 )
		{
			if(EN2==1)
			{
			__IO uint16_t cout2;
			cout2 = __HAL_TIM_GetCounter (&htim1);
			__HAL_TIM_SET_COMPARE (&htim1 ,TIM_CHANNEL_2 ,cout2+Speed2_Pulse );
			}
		}
		if(htim->Channel ==HAL_TIM_ACTIVE_CHANNEL_3 )
		{
			if(EN3==1)
			{
			__IO uint16_t cout3;
			cout3 = __HAL_TIM_GetCounter (&htim1);
			__HAL_TIM_SET_COMPARE (&htim1 ,TIM_CHANNEL_3 ,cout3+Speed3_Pulse );
			}
		}
		if(htim->Channel ==HAL_TIM_ACTIVE_CHANNEL_4 )
		{
			if(EN4==1)
			{
			__IO uint16_t cout4;
			cout4 = __HAL_TIM_GetCounter (&htim1);
			__HAL_TIM_SET_COMPARE (&htim1 ,TIM_CHANNEL_4 ,cout4+Speed4_Pulse );
			}
		}
		
	}
}


