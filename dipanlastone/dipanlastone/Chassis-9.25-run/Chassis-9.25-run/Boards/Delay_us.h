#ifndef __Delay_us_H__
#define __Delay_us_H__

#include "main.h"

void Delay_us_init(uint8_t SYSCLK);
void Delay_us(uint32_t nus);
void delay_us(uint32_t nus);

#endif
