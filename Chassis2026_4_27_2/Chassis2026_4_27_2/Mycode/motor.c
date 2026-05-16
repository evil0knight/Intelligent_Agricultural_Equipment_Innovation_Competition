#include "motor.h"

#define PWM_MAX 6300
#define PWM_MIN -6300

extern TIM_HandleTypeDef htim8;
extern TIM_HandleTypeDef htim10;
extern TIM_HandleTypeDef htim11;
extern TIM_HandleTypeDef htim12;

int abs(int p)
{
    if(p>0)
        return p;
    else
        return -p;
}


void Motor_current_set(int motorA,int motorB,int motorC, int motorD)      //-7200~7200
{
		/* 限幅 */
		if(motorA>PWM_MAX)			motorA=PWM_MAX;
		else if(motorA<-PWM_MAX)motorA=-PWM_MAX;
		
		if(motorB>PWM_MAX)			motorB=PWM_MAX;
		else if(motorB<-PWM_MAX)motorB=-PWM_MAX;
		
		if(motorC>PWM_MAX)			motorC=PWM_MAX;
		else if(motorC<-PWM_MAX)motorC=-PWM_MAX;
		
		if(motorD>PWM_MAX)			motorD=PWM_MAX;
		else if(motorD<-PWM_MAX)motorD=-PWM_MAX;
	
	/* C0 :A2   C1 :A1 */
    if(motorA>=0)
    {
        HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_RESET);
    }
    else if(motorA<0)
    {
        HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_SET);
    }
//		else
//		{
//				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_SET);
//        HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_SET);
//		}
		/* C4 :B1  C5 :B2 */
		if(motorB>=0)
    {
        HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOC,GPIO_PIN_5,GPIO_PIN_RESET);
    }
    else if(motorB<0)
    {
        HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOC,GPIO_PIN_5,GPIO_PIN_SET);
    }
//		else
//		{
//				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_SET);
//        HAL_GPIO_WritePin(GPIOC,GPIO_PIN_5,GPIO_PIN_SET);
//		}
		/* B2 :C1  E7 :C2*/
		if(motorC>=0)
    {
        HAL_GPIO_WritePin(GPIOE,GPIO_PIN_7,GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB,GPIO_PIN_2,GPIO_PIN_RESET);
    }
    else if(motorC<0)
    {
        HAL_GPIO_WritePin(GPIOE,GPIO_PIN_7,GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB,GPIO_PIN_2,GPIO_PIN_SET);
    }
//		else
//		{
//				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_2,GPIO_PIN_SET);
//        HAL_GPIO_WritePin(GPIOE,GPIO_PIN_7,GPIO_PIN_SET);
//		}
		/* E0 :D1  E1:D2 */
		if(motorD>=0)
    {
        HAL_GPIO_WritePin(GPIOE,GPIO_PIN_1,GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOE,GPIO_PIN_0,GPIO_PIN_RESET);
    }
    else if(motorD<0)
    {
        HAL_GPIO_WritePin(GPIOE,GPIO_PIN_1,GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOE,GPIO_PIN_0,GPIO_PIN_SET);
    }
//		else
//		{
//				HAL_GPIO_WritePin(GPIOE,GPIO_PIN_0,GPIO_PIN_SET);
//        HAL_GPIO_WritePin(GPIOE,GPIO_PIN_1,GPIO_PIN_SET);
//		}
		
    __HAL_TIM_SetCompare(&htim10,TIM_CHANNEL_1,abs(motorA));
		__HAL_TIM_SetCompare(&htim11,TIM_CHANNEL_1,abs(motorB));
		__HAL_TIM_SetCompare(&htim8 ,TIM_CHANNEL_3,abs(motorC));
		__HAL_TIM_SetCompare(&htim8 ,TIM_CHANNEL_4,abs(motorD));
  
}

/* 作业机构 */

void Working_motor_set(int motor1,int motor2)      //-7200~7200
{
		/* 限幅 */
		if(motor1>PWM_MAX)			motor1=PWM_MAX;
		else if(motor1<-PWM_MAX)motor1=-PWM_MAX;
		
		if(motor2>PWM_MAX)			motor2=PWM_MAX;
		else if(motor2<-PWM_MAX)motor2=-PWM_MAX;
		

	
    if(motor1>=0)
    {
        HAL_GPIO_WritePin(GPIOE,GPIO_PIN_14,GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOE,GPIO_PIN_15,GPIO_PIN_RESET);
    }
    else if(motor1<0)
    {
        HAL_GPIO_WritePin(GPIOE,GPIO_PIN_14,GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOE,GPIO_PIN_15,GPIO_PIN_SET);
    }

		if(motor2>=0)
    {
        HAL_GPIO_WritePin(GPIOE,GPIO_PIN_10,GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOE,GPIO_PIN_12,GPIO_PIN_RESET);
    }
    else if(motor2<0)
    {
        HAL_GPIO_WritePin(GPIOE,GPIO_PIN_10,GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOE,GPIO_PIN_12,GPIO_PIN_SET);
    }

    __HAL_TIM_SetCompare(&htim12,TIM_CHANNEL_1,abs(motor1));
		__HAL_TIM_SetCompare(&htim12,TIM_CHANNEL_2,abs(motor2));

  
}



