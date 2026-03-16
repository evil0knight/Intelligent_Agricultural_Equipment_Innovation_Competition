#include "Delay_us.h"
#include "tim.h"

/*
Systick功能实现us延时，参数SYSCLK为系统时钟  168M
*/
uint8_t   fac_us=0;							//us延时倍乘数

//void HAL_Delay_us_init(uint8_t SYSCLK)
//{
//     fac_us=SYSCLK; 
//}

void Delay_us_init(uint8_t SYSCLK)
{
     fac_us=SYSCLK; 	
//     fac_us=168; 
}

void Delay_us(uint32_t nus)
{
    uint32_t ticks;
    uint32_t told,tnow,tcnt=0;
    uint32_t reload=SysTick->LOAD;
    ticks=nus*fac_us; 
    told=SysTick->VAL; 
    while(1)
    {
        tnow=SysTick->VAL;
        if(tnow!=told)
        {
            if(tnow<told)tcnt+=told-tnow;
            else tcnt+=reload-tnow+told;
            told=tnow;
            if(tcnt>=ticks)break; 
        }
    }
}


void delay_us(uint32_t nus)
{		
	uint32_t temp;	    	 
	SysTick->LOAD=nus*fac_us; 				//时间加载	  		 
	SysTick->VAL=0x00;        				//清空计数器
	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk ; //开始倒数 	 
	do
	{
		temp=SysTick->CTRL;
	}while((temp&0x01)&&!(temp&(1<<16)));	//等待时间到达   
	SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk; //关闭计数器
	SysTick->VAL =0X00;       				//清空计数器 
}


///*
// 普通定时器实现us延时
//*/
//void user_delaynus_tim(uint32_t nus)
//{
// 
// uint16_t  differ = 0xffff-nus-5;
// //设置定时器2的技术初始值
//  __HAL_TIM_SetCounter(&htim2,differ);
//  //开启定时器
//  HAL_TIM_Base_Start(&htim2);
// 
//  while( differ<0xffff-5)
// {
//  differ = __HAL_TIM_GetCounter(&htim2);
// };
// //关闭定时器
//  HAL_TIM_Base_Stop(&htim2);
//}

