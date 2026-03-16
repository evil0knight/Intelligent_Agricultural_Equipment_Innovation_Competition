#include "chaoshengbo.h"
#include "tim.h"
#include <stdint.h> 
extern uint8_t distancex[4]; 
extern uint8_t distancey[4];
extern uint16_t chaoshengbo;
extern uint16_t chaoshengbo1;

void CHAOSHENGBO_Get()
{
  uint8_t Data_H = distancex[1];  
  uint8_t Data_L = distancex[2];  
  uint16_t distance_value = (Data_H << 8) | Data_L;  
  chaoshengbo = distance_value ; 
}
void CHAOSHENGBO_Get1()
{
  uint8_t Data_H = distancey[1];  
  uint8_t Data_L = distancey[2];  
  uint16_t distance_value = (Data_H << 8) | Data_L;  
  chaoshengbo1 = distance_value ; 
}
