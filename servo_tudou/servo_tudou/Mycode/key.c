#include "key.h"
#include "delay.h"



uint8_t key_mark;


/**
 * @brief       按键扫描函数
 * @note        该函数有响应优先级(同时按下多个按键): WK_UP > KEY1 > KEY0!!
 * @param       mode:0 / 1, 具体含义如下:
 *   @arg       0,  不支持连续按(当按键按下不放时, 只有第一次调用会返回键值,
 *                  必须松开以后, 再次按下才会返回其他键值)
 *   @arg       1,  支持连续按(当按键按下不放时, 每次调用该函数都会返回键值)
 * @retval      键值, 定义如下:
 *              KEY0_PRES, 1, KEY0按下
 *              KEY1_PRES, 2, KEY1按下
 *              WKUP_PRES, 3, WKUP按下
 */

uint8_t key_scan(uint8_t mode)
{
    static uint8_t key_up = 1;  /* 按键按松开标志 */
    
		uint8_t keyval = 0;
    if (mode) key_up = 1;       /* 支持连按 */

    if (key_up && (KEY1 == 0 || KEY2 == 0 || KEY3== 0 || KEY4 == 0 ))  /* 按键松开标志为1, 且有任意一个按键按下了 */
    {
       delay_ms(10);           /* 去抖动 */
       key_up = 0;

			if (KEY1 == 0)  keyval = 1;

			if (KEY2 == 0)  keyval = 2;

			if (KEY3 == 0)  keyval = 3;
	
			if (KEY4 == 0)  keyval = 4;
    }
    else if (KEY1 == 1 && KEY2 == 1 && KEY3 == 1&& KEY4 == 1) /* 没有任何按键按下, 标记按键松开 */
    {
        key_up = 1;
    }

    return keyval;              /* 返回键值 */
}

//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
//{

//    switch(GPIO_Pin)
//    {
//		
//        case KEY1_INT_GPIO_PIN:
//            if (KEY1 == 0)
//            {
//								key_mark=1;
//							
//            }
//            break;
//        case KEY2_INT_GPIO_PIN:
//            if (KEY2 == 0)
//            {
//                key_mark=2;

//            }
//            break;
//        case KEY3_INT_GPIO_PIN:
//            if (KEY3 == 1)
//            {
//								key_mark=3;

//            }
//            break;
//				case KEY4_INT_GPIO_PIN:
//            if (KEY4 == 1)
//            {
//                key_mark=4;

//            }
//            break;
//    }
//}






















