/**
 * @file HCSR04.c
 * @author Zhong Zepeng (1935595312@qq.com)
 * @brief
 * @version 0.1
 * @date 2022-11-25
 *
 * @copyright Copyright (c) 2022
 *
 */
#include "SR04.h"
#include "gpio.h"
#include "tim.h"
#include "Delay_us.h"
/**
 * @brief 激活超声波定时器
 * 
 */
void HCSR_04_1()
{
    uint32_t i;
    HAL_GPIO_WritePin(TRIG1_GPIO_Port, TRIG1_Pin, GPIO_PIN_SET);
    for (i = 0; i < 72 * 40; i++)
        __NOP();
    HAL_GPIO_WritePin(TRIG1_GPIO_Port, TRIG1_Pin, GPIO_PIN_RESET);
}


/**
 * @brief 计算超声波检测的距离
 * 
 * @return float 
 */
float getSR04Distance_1()
{
    float len = 0;
    uint32_t time = 0;
    if (TIM5CH1_CAPTURE_STA & 0X80) //输入捕获触发
    {
        time = TIM5CH1_CAPTURE_STA & 0X3f;   //获得溢出次数
        time *= 65536;                       //一次溢出为65536 得到溢出的时间
        time += TIM5CH1_CAPTURE_VAL;         //溢出的时间+现在定时器的值 得到总的时间
        len = time * 342.62 * 100 / 2000000; // 计算得到距离
        TIM5CH1_CAPTURE_STA = 0; //清除溢出
    }
    return len;
}



void HCSR_04_2()
{
    uint32_t i;
    HAL_GPIO_WritePin(TRIG2_GPIO_Port, TRIG2_Pin, GPIO_PIN_SET);
    for (i = 0; i < 72 * 40; i++)
        __NOP();
    HAL_GPIO_WritePin(TRIG2_GPIO_Port, TRIG2_Pin, GPIO_PIN_RESET);
}

float getSR04Distance_2()
{
    float len = 0;
    uint32_t time = 0;
    if (TIM5CH2_CAPTURE_STA & 0X80) //输入捕获触发
    {
        time = TIM5CH2_CAPTURE_STA & 0X3f;   //获得溢出次数
        time *= 65536;                       //一次溢出为65536 得到溢出的时间
        time += TIM5CH2_CAPTURE_VAL;         //溢出的时间+现在定时器的值 得到总的时间
        len = time * 342.62 * 100 / 2000000; // 计算得到距离
        TIM5CH2_CAPTURE_STA = 0; //清除溢出
    }
    return len;
}


void HCSR_04_3()
{
    uint32_t i;
    HAL_GPIO_WritePin(TRIG3_GPIO_Port, TRIG3_Pin, GPIO_PIN_SET);
    for (i = 0; i < 72 * 40; i++)
        __NOP();
    HAL_GPIO_WritePin(TRIG3_GPIO_Port, TRIG3_Pin, GPIO_PIN_RESET);
}

float getSR04Distance_3()
{
    float len = 0;
    uint32_t time = 0;
    if (TIM5CH3_CAPTURE_STA & 0X80) //输入捕获触发
    {
        time = TIM5CH3_CAPTURE_STA & 0X3f;   //获得溢出次数
        time *= 65536;                       //一次溢出为65536 得到溢出的时间
        time += TIM5CH3_CAPTURE_VAL;         //溢出的时间+现在定时器的值 得到总的时间
        len = time * 342.62 * 100 / 2000000; // 计算得到距离
        TIM5CH3_CAPTURE_STA = 0; //清除溢出
    }
    return len;
}


void HCSR_04_4()
{
    uint32_t i;
    HAL_GPIO_WritePin(TRIG4_GPIO_Port, TRIG4_Pin, GPIO_PIN_SET);
    for (i = 0; i < 72 * 40; i++)
        __NOP();
    HAL_GPIO_WritePin(TRIG4_GPIO_Port, TRIG4_Pin, GPIO_PIN_RESET);
}

float getSR04Distance_4()
{
    float len = 0;
    uint32_t time = 0;
    if (TIM5CH4_CAPTURE_STA & 0X80) //输入捕获触发
    {
        time = TIM5CH4_CAPTURE_STA & 0X3f;   //获得溢出次数
        time *= 65536;                       //一次溢出为65536 得到溢出的时间
        time += TIM5CH4_CAPTURE_VAL;         //溢出的时间+现在定时器的值 得到总的时间
        len = time * 342.62 * 100 / 2000000; // 计算得到距离
        TIM5CH4_CAPTURE_STA = 0; //清除溢出
    }
    return len;
}




void HCSR_04_5()
{
    uint32_t i;
    HAL_GPIO_WritePin(TRIG5_GPIO_Port, TRIG5_Pin, GPIO_PIN_SET);
    for (i = 0; i < 72 * 40; i++)
        __NOP();
    HAL_GPIO_WritePin(TRIG5_GPIO_Port, TRIG5_Pin, GPIO_PIN_RESET);
}


float getSR04Distance_5()
{
    float len = 0;
    uint32_t time = 0;
    if (TIM3CH3_CAPTURE_STA & 0X80) //输入捕获 触发
    {
        time = TIM3CH3_CAPTURE_STA & 0X3f;   //获得溢出次数
        time *= 65536;                       //一次溢出为65536 得到溢出的时间
        time += TIM3CH3_CAPTURE_VAL;         //溢出的时间+现在定时器的值 得到总的时间
        len = time * 342.62 * 100 / 2000000; // 计算得到距离
        TIM3CH3_CAPTURE_STA = 0; //清除溢出
    }
    return len;
}

void HCSR_04_6()
{
    uint32_t i;
    HAL_GPIO_WritePin(TRIG6_GPIO_Port, TRIG6_Pin, GPIO_PIN_SET);
    for (i = 0; i < 72 * 40; i++)
        __NOP();
    HAL_GPIO_WritePin(TRIG6_GPIO_Port, TRIG6_Pin, GPIO_PIN_RESET);
}


float getSR04Distance_6()
{
    float len = 0;
    uint32_t time = 0;
    if (TIM3CH4_CAPTURE_STA & 0X80) //输入捕获 触发
    {
        time = TIM3CH4_CAPTURE_STA & 0X3f;   //获得溢出次数
        time *= 65536;                       //一次溢出为65536 得到溢出的时间
        time += TIM3CH4_CAPTURE_VAL;         //溢出的时间+现在定时器的值 得到总的时间
        len = time * 342.62 * 100 / 2000000; // 计算得到距离
        TIM3CH4_CAPTURE_STA = 0; //清除溢出
    }
    return len;
}
