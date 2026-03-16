#include "key.h"
#include "delay.h"
#include "pid.h"


extern float speed_goal;
extern uint8_t MOVE_STATE; // Æô¶¯
extern float start_yaw,yaw;


uint8_t key_mark;


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    delay_ms(20);      /* Ïû¶¶ */
    switch(GPIO_Pin)
    {
		
        case KEY1_INT_GPIO_PIN:
            if (KEY1 == 0)
            {
                MOVE_STATE=1;
							
            }
            break;
        case KEY2_INT_GPIO_PIN:
            if (KEY2 == 0)
            {
                key_mark=2;
								start_yaw=yaw;
            }
            break;
        case KEY3_INT_GPIO_PIN:
            if (KEY3 == 1)
            {
								key_mark=3;

            }
            break;
				case KEY4_INT_GPIO_PIN:
            if (KEY4 == 1)
            {
                key_mark=4;

            }
            break;
    }
}






















