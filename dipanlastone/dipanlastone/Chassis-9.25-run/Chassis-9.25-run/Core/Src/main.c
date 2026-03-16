/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
#include "Bujin.h"
#include "Delay_us.h"
#include "oled.h"
#include "PID.h"
#include "SR04.h"
#include "WHT101.h"
#include "PES.h"
#include "chaoshengbo.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
//#include "2ms_Task.h"
//#include "10ms_Task.h"
//#include "500ms_Task.h"
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
	
int a=0,b=0;
uint8_t YAW_buffer[30];  //串口1，陀螺仪数组
uint8_t distancex[4];   //串口4超声波数据
uint8_t distancey[4];  
uint8_t OPENCV_buffer[2];//串口3，摄像头颜色识别数组
float Distance[7];       //超声波获取距离数组
float Distance1=0.0,Distance2=0.0;



float expected_distance = 10.0f; // 期望的距离
float current_distance = 15.0f;  // 当前激光测距



uint16_t chaoshengbo;
uint16_t chaoshengbo1;
uint8_t PES[9];          //光电管检测数组
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
uint16_t tim=0;
uint8_t start_flag=0,accel_flag=0;
int32_t Speed1_Pulse=0,Speed2_Pulse=0,Speed3_Pulse=0,Speed4_Pulse=0;
uint8_t Color_L=0,Color_R=0;
int32_t SetSpeed=0,CurSpeed=0,LQ,LH,RQ,RH;
uint8_t Motor_Mode=0;
uint8_t angle_1,angle_2;
short angle_z;
uint8_t z[5]={0xFF,0xAA,0x76,0x00,0x00};//陀螺仪初始化归零代码
float yaw_init=0,yaw_source,yaw;//WHT101维特智能陀螺仪
float SetAngle=0.0;
int heng_flag;
int Turn_flag;//
int turn_angle;
float  x; 
uint8_t num3=3;
uint8_t num4=4;
int  flag1=0;
//extern float LastDis;////////
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
float decayFactor = 0.00007;
int initialSpeed = 3700;
int minSpeed = 0; 
int currentSpeed;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_SPI2_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM5_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_TIM10_Init();
  MX_TIM11_Init();
  MX_TIM13_Init();
  MX_TIM14_Init();
  MX_TIM6_Init();
  MX_UART4_Init();
  MX_UART5_Init();
  /* USER CODE BEGIN 2 */
	
	OLED_Init();
	OLED_Clear();
	HAL_Delay(5);
	OLED_ShowString(0,0,"stm32",16,1);
	OLED_Refresh();

  Delay_us_init(168);//微秒级定时器初始化
	
	HAL_TIM_Base_Start_IT (&htim2 );//tim2 开启定时器中断,作为主计数定时器，实现定时任务执行
	HAL_TIM_Base_Start_IT (&htim6 );//tim6 开启定时器中断，副定时器
	
	HAL_TIM_Base_Start (&htim3);                  //开启定时器(超声波)
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_3);   //开启TIM2的捕获通道1，并开启捕获
	HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_4);   //开启TIM2的捕获通道2，并开启捕获
  __HAL_TIM_ENABLE_IT(&htim3, TIM_IT_UPDATE);   //使能更新中断
	
	
	HAL_TIM_Base_Start (&htim5);                  //开启定时器(超声波)
  HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_1);   //开启TIM2的捕获通道1，并开启捕获
	HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_2);   //开启TIM2的捕获通道2，并开启捕获
	HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_3);   //开启TIM2的捕获通道1，并开启捕获
	HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_4);   //开启TIM2的捕获通道2，并开启捕获
  __HAL_TIM_ENABLE_IT(&htim5, TIM_IT_UPDATE);   //使能更新中断
	
	
	HAL_TIM_Base_Start  (&htim1);                 //开启定时器(步进电机)
	HAL_TIM_OC_Start_IT (&htim1 ,TIM_CHANNEL_1 ); //开启TIM1 输出比较通道1
	HAL_TIM_OC_Start_IT (&htim1 ,TIM_CHANNEL_2 );
	HAL_TIM_OC_Start_IT (&htim1 ,TIM_CHANNEL_3 );
	HAL_TIM_OC_Start_IT (&htim1 ,TIM_CHANNEL_4 );
	
	
	HAL_TIM_Base_Start(&htim10);                   //开启定时器(步进电机)
	HAL_TIM_OC_Start_IT (&htim10 ,TIM_CHANNEL_1 ); //开启TIM1 输出比较通道1
	HAL_TIM_Base_Start(&htim11);
	HAL_TIM_OC_Start_IT (&htim11 ,TIM_CHANNEL_1 );
	HAL_TIM_Base_Start(&htim13);
	HAL_TIM_OC_Start_IT (&htim13 ,TIM_CHANNEL_1 );
	HAL_TIM_Base_Start(&htim14);
	HAL_TIM_OC_Start_IT (&htim14 ,TIM_CHANNEL_1 );
	
	
	//陀螺仪PID初始化
	PID_Init(&PID_Across_yaw,4.0f,.2f,0.3f);//长直线垄陀螺仪速度环 // 60.0f,0.0f,8.3f //5.0f,0.1f,0.1f
	PID_Init(&PID_Forward_Heng_Control,15.0f,0.1f,0.1f);//横向陀螺仪
	
	PID_Init(&PID_Sonic,5.0f,0.3f,0.4f);       //长直线垄超声波12.0f,0.0f,0.3f
	PID_Init(&PID_Sonic_stable,200.0f,1.0f,0.4f);//停车后静止检测7.0,0.1f,0.2f
	
	PID_Init(&PID_Sonic_stable2,7.0f,0.1f,0.2f);//停车后静止检测222 7.0,0.1f,0.2f
	PID_Init(&PID_Sonic_mid,12.0f,0.1f,0.3f);//停车后静止检测7.0,0.1f,0.2f
	
	 PID_Init(&PID_Deceleration, 1.0f, 0.1f, 0.05f); // 激光减速
//	PID_Init(&PID_Sonic,10.5f,0.0f,0.2f);//长直线垄超声波
//	a = (a>0?a:-a);
//	if (a > 40)
//		PID_Init(&PID_Angle_Control,3*10.0f,4*0.1f,4*10.0f);// // 20.0f,0.0f,0.5f  
//	if (a <=40)
//		PID_Init(&PID_Angle_Control,0.0f,4*0.1f,4*10.0f);
	
	//串口1完成陀螺仪数据接收   
	__HAL_UART_ENABLE_IT (&huart1 ,UART_IT_IDLE );    //串口中断使能&huart1	
	HAL_UART_Receive_DMA (&huart1,YAW_buffer ,sizeof(YAW_buffer));  //cubemx开启dma串口通讯
	
	//串口3完成摄像头数据接收
	__HAL_UART_ENABLE_IT (&huart3 ,UART_IT_IDLE );    //串口中断使能&huart3	
	HAL_UART_Receive_DMA (&huart3,OPENCV_buffer ,sizeof(OPENCV_buffer));  //cubemx开启dma串口通讯
  //串口4完成摄像头数据接收
  __HAL_UART_ENABLE_IT (&huart4 ,UART_IT_IDLE );    //串口中断使能&huart4	
	HAL_UART_Receive_DMA (&huart4,distancex ,sizeof(distancex));  //cubemx开启dma串口通讯	
	 __HAL_UART_ENABLE_IT (&huart5 ,UART_IT_IDLE );    //串口中断使能&huart4	
	HAL_UART_Receive_DMA (&huart5,distancey ,sizeof(distancey));  //cubemx开启dma串口通讯	
	//步进电机转动方向初始化
	SetSpeed =0;
	SetAngle =0;
	start_flag=0;
	Stop();
	HAL_GPIO_WritePin (DIRA_GPIO_Port ,DIRA_Pin ,GPIO_PIN_SET );
	HAL_GPIO_WritePin (DIRB_GPIO_Port ,DIRB_Pin ,GPIO_PIN_SET );
	HAL_GPIO_WritePin (DIRC_GPIO_Port ,DIRC_Pin ,GPIO_PIN_SET );
	HAL_GPIO_WritePin (DIRD_GPIO_Port ,DIRD_Pin ,GPIO_PIN_RESET );
	
	HAL_UART_Transmit (&huart1 ,z,5,10);//陀螺仪清零函数
	HAL_Delay (200);
	HAL_UART_Transmit (&huart3 ,"0",1,10);//板间数据交流停止标志：0	
	HAL_Delay (200); 
	
//	Forward_L(0,0);
	
//	SetSpeed =90;
//	SetAngle =0;
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {  
//		MotorRQ_Ctrl(1000);

 if(start_flag==0)
{ 
	SetSpeed =0;
	SetAngle =0;
	Stop();
}
////////////////////////////////////////第一次直线行驶		
else if(start_flag ==1)
{
  if(accel_flag==1)
	{
	
for(int i=0;i<3800;i+=1)
			{
			delay_us(280);
			SetSpeed=i;
			Forward_L(0,0);
			}
accel_flag=2;		
SetSpeed=3700;
  }

Forward_L(Control_Across_yaw(SetAngle,yaw),0);
 if(PES[4]==0)//左侧光电管做入垄检测，当入垄后再进行超声波pid计算
	{
 Forward_L(Control_Across_yaw(SetAngle,yaw),Control_sonic(100,chaoshengbo)/2);
	}
		  if(PES[8]==0)
		{
			for(int i=3700;i>=2000;i-=0.5)
			{
				delay_us(310); 
				SetSpeed=i;
			  Forward_L(Control_Across_yaw(SetAngle,yaw),0);
			}
			
			for(int i=2000;i>=1000;i-=5)     //前侧的减速代码
			{
			delay_us(430);
			SetSpeed=i;
			Forward_L(Control_Across_yaw(SetAngle,yaw),0);//Control_Across_yaw(SetAngle,yaw)
			}
			
			for(int i=1000;i>=0;i-=10)
			{
			delay_us(420);
			SetSpeed=i;
			Forward_L(Control_Across_yaw(SetAngle,yaw),0);//Control_Across_yaw(SetAngle,yaw)
			}

			SetSpeed =0;	
//      SetAngle=90;			
			start_flag=99;
			tim=0;
		}
} 
/////////////////////转弯
else if (start_flag ==99)
{
	Forward_Stable(Control_Across_yaw(SetAngle,yaw),Control_sonic_stable(90,chaoshengbo1));
	if (fabs(chaoshengbo1-100)<4||tim>100)
	{
		start_flag = 2;
		tim=0; 
	}
}
else if (start_flag==2)
{
	SetAngle=90;
	a=SetAngle -yaw ;
	turn_angle = (a>0?a:-a);
	if (turn_angle > 15 && Turn_flag == 0)
	{
		Turn_flag=1;
	}
	else if (Turn_flag==1)
	{
		PID_Init(&PID_Angle_Control,40.0f,0.4f,40.0f);
		Turn_flag=2;
	}
	else if (a <= 15 && Turn_flag==2)
	{
		PID_Init(&PID_Angle_Control,0.0f,0.5f,40.0f);//0.0f,0.4f,40.0f
		Turn_flag=3;
	}
	TurnRight(Turn_Angle_Control(SetAngle,yaw));
	if (turn_angle<0.2||tim>100)
	{
		start_flag=4;
		tim=0;
	}
}
else if (start_flag==4)//&&(tim>500||tim<3000)
{
	//Forward_L(Control_Across_yaw(SetAngle,yaw),Control_sonic_stable(3.7,Distance[6]));
	Modify_ZH (Control_Across_yaw(SetAngle,yaw),0,0);
	if ((SetAngle -yaw<0.2)||tim>200)//
	{
		start_flag=6;
	}
//	start_flag=1;
}
///////////第一条垄道加速
else if (start_flag==6)//&&(tim>500||tim<3000)
{
	//Forward_L(Control_Across_yaw(SetAngle,yaw),Control_sonic_stable(3.7,Distance[6]));
	
	SetAngle=90;

	if(accel_flag==2)
	{
	
for(int i=0;i<3650;i+=1)
			{
			delay_us(200);
			SetSpeed=i;
			Forward_L(0,0);
			}
accel_flag=3;		
SetSpeed=3650;
tim =0;
 }
Forward_L(Control_Across_yaw(SetAngle,yaw),0);

 if(PES[7]==0)//右侧光电管做入垄检测，当入垄后再进行超声波pid计算
	{
 Forward_L(Control_Across_yaw(SetAngle,yaw),Control_sonic(90,chaoshengbo)/2);////

	}
			 if(PES[8]==0)
		{ 
			for(int i=3500;i>=2000;i-=2)  //前侧垄道阶梯减速
			{
				delay_us(270); 
			}
			for(int i=3500;i>=2000;i-=0.2)
			{
				delay_us(470); 
				SetSpeed=i;
			  Forward_L(Control_Across_yaw(SetAngle,yaw),0);
			}
			
			for(int i=2000;i>=1000;i-=5)
			{
			delay_us(480);
			SetSpeed=i;
			Forward_L(Control_Across_yaw(SetAngle,yaw),0);//Control_Across_yaw(SetAngle,yaw)
			}
			for(int i=1000;i>=0;i-=20)
			{
			delay_us(490);
			SetSpeed=i;
			Forward_L(Control_Across_yaw(SetAngle,yaw),0);//Control_Across_yaw(SetAngle,yaw)
			}
			SetSpeed =0;	
	    start_flag=7;
			tim=0;
		}

}
///////////////////////////////第一次横向移动到下一个笼道
else if(start_flag ==7)
{
	
//	 tim=0;	
	if(	accel_flag==3)
	{	for(int i=0;i<3000;i+=2)
			{
			delay_us(300);
			SetSpeed=i;
			Heng(Control_Across_yaw(SetAngle,yaw),0);
			}
	heng_flag=0;
	accel_flag=4;
	SetSpeed =3400;
	SetAngle =90;
	}
	Heng(Control_Across_yaw(SetAngle,yaw),0);
	 if(PES[3]==0)
       {		
         heng_flag=1;}
	  
		
	if(heng_flag==1&&PES[3]==1)
	{
	 heng_flag=2;
	 tim=0;
	}
	if(heng_flag==2&&tim>22)
	{
			
			for(int i=3500;i>=0;i-=1)
			{
			delay_us(80);
			SetSpeed=i;
			Heng(Control_Across_yaw(SetAngle,yaw),0);
			}
			start_flag =8;
	}
	
}
/////////////////////  第二条垄道作业

else if(start_flag ==8)
{

//	SetAngle =0;
	if(accel_flag==4)
	{
		HAL_UART_Transmit (&huart3 ,"1",1,10);//板间数据交流起始标志：1
		for(int i=0;i<3600;i+=1)//梯形加速
		{
			delay_us(200);
			SetSpeed=-i;
			Forward_L(Control_Across_yaw(SetAngle,yaw),0);
	  //Forward_L(Control_Across_yaw(SetAngle,yaw),Control_sonic(90,chaoshengbo)/2);
		}
		tim=0; 
	accel_flag=5;		
	SetSpeed =-3500;		
	}

  if(flag1==0)
	Forward_L(Control_Across_yaw(SetAngle,yaw),0);
	if(PES[4]==0||PES[7]==0)//左右侧光电管做入垄检测，当入垄后再进行超声波pid计算
	{flag1=1;
 Forward_L(Control_Across_yaw(SetAngle,yaw),Control_sonic(120,chaoshengbo)/2);
 SetSpeed =-3400;
	}
    if(PES[3]==0)  //后侧灯管
	{
			for(int i=3400;i>=0;i-=1)
			{
			delay_us(280);
			SetSpeed=-i;
			Forward_L(0,0);//Control_Across_yaw(SetAngle,yaw)
			}
			SetSpeed =0;	
			tim=0;
			start_flag=9;
			HAL_UART_Transmit (&huart3 ,"0",1,10);//板间数据交流停止标志：0	
	}

}
/////////////////////第二次横向移动到下一个笼道
else if(start_flag ==9)
{
	
//	 tim=0;	
	if(	accel_flag==5)
	{	for(int i=0;i<3000;i+=2)
			{
			delay_us(300);
			SetSpeed=i;
			Heng(Control_Across_yaw(SetAngle,yaw),Control_sonic(7.5,Distance[6]));
			}
	heng_flag=0;
	accel_flag=6;
	SetSpeed =3400;
	SetAngle =90;
	}
	Heng(Control_Across_yaw(SetAngle,yaw),Control_sonic(7.5,Distance[6]));
	  if(PES[8]==0)///////////////////
       {		
          heng_flag=2;
		   }
	if(heng_flag==2&&PES[8]==1)
	{
	 heng_flag=3;
	 tim=0;
	}
	if(heng_flag==3&&tim>22)
	{
			
			for(int i=3500;i>=0;i-=1)
			{
			delay_us(80);
			SetSpeed=i;
			Heng1(Control_Across_yaw(SetAngle,yaw),0);
			}
			start_flag =10;
	}
	
}
//////////////////////////第三条垄道加速
else if(start_flag ==10)
{
   
//	SetAngle =0;
	if(accel_flag==6)
	{ 
		for(int i=0;i<3700;i+=1)//梯形加速
		{
			delay_us(300);
			SetSpeed=i;
			Forward_Mid_Q(Control_Across_yaw(SetAngle,yaw),0,0);
		}
		tim=0;
		accel_flag=7;		
	SetSpeed =3700;		
	}

//	SetAngle =0;
	Forward_Mid_Q(Control_Across_yaw(SetAngle,yaw),0,0);
	if(PES[4]==0||PES[7]==0)//左右侧光电管做入垄检测，当入垄后再进行超声波pid计算
	{
	  Forward_Mid_Q(Control_Across_yaw(SetAngle,yaw),Control_sonic(120,chaoshengbo)/2,0);
	  SetSpeed =3700;                                 
	}
    if(PES[8]==0)  //后侧灯管
	{
		 for(int i=3500;i>=2000;i-=0.2)
			{
				delay_us(500); 
				SetSpeed=i;
			  Forward_L(Control_Across_yaw(SetAngle,yaw),0);
			}
			
			for(int i=2000;i>=1000;i-=1)
			{
			delay_us(600);
			SetSpeed=i;
			Forward_L(Control_Across_yaw(SetAngle,yaw),0);//Control_Across_yaw(SetAngle,yaw)
			}
			for(int i=1000;i>=0;i-=5)
			{
			delay_us(500);
			SetSpeed=i;
			Forward_L(Control_Across_yaw(SetAngle,yaw),0);//Control_Across_yaw(SetAngle,yaw)
			}
			SetSpeed =0;	
			tim=0;
			start_flag=11;
	}
}

/////////////////////////////////////第三次横向移动到下一个笼道


else if(start_flag ==11)
{
	
//	 tim=0;	
	if(	accel_flag==7)
	{	for(int i=0;i<3200;i+=2)
			{
			delay_us(300);
			SetSpeed=i;
			Heng(Control_Across_yaw(SetAngle,yaw),0);
			}
	heng_flag=0;
	accel_flag=8;
	SetSpeed =3400;
	SetAngle =90;
	}
	Heng(Control_Across_yaw(SetAngle,yaw),0);
	  if(PES[3]==0)
       {		
          heng_flag=2;
		}
	if(heng_flag==2&&PES[3]==1)
	{
	 heng_flag=3;
	 tim=0;
	}
	if(heng_flag==3&&tim>22)
	{
			
			for(int i=3500;i>=0;i-=1)
			{
			delay_us(80);
			SetSpeed=i;
			Heng(Control_Across_yaw(SetAngle,yaw),0);
			}
			start_flag =12;
	}
	
}
//////////////////////////////////////////第四条垄道作业

else if(start_flag ==12)
{

//	SetAngle =0;
	if(accel_flag==8)
	{
		HAL_UART_Transmit (&huart3 ,"1",1,10);//板间数据交流起始标志：1
		for(int i=0;i<3600;i+=1)//梯形加速
		{
			delay_us(200);
			SetSpeed=i;
			Forward_Mid_H(Control_Across_yaw(SetAngle,yaw),0,0);
		}
		tim=0;
		accel_flag=9;		
	SetSpeed =3500;		
	}

//	SetAngle =0;
	Forward_Mid_H(Control_Across_yaw(SetAngle,yaw),0,0);
	if(PES[4]==0||PES[7]==0)//左右侧光电管做入垄检测，当入垄后再进行超声波pid计算
	{
	  Forward_Mid_H(Control_Across_yaw(SetAngle,yaw),Control_sonic(120,chaoshengbo)/2,0);
	  SetSpeed =3500;
	}
    if(PES[3]==0)  //后侧灯管
	{
		
			for(int i=3500;i>=0;i-=1)
			{
			delay_us(250);
			SetSpeed=-i;
			Forward_L(0,0);//Control_Across_yaw(SetAngle,yaw)
			}
			SetSpeed =0;	
			tim=0;
			start_flag=13;
			HAL_UART_Transmit (&huart3 ,"0",1,10);//板间数据交流起始标志：0
	}

}
/////////////////////////////////////第四次横向移动到下一个笼道
else if(start_flag ==13)
{
	
//	 tim=0;	
	if(	accel_flag==9)
	{	for(int i=0;i<3000;i+=2)
			{
			delay_us(300);
			SetSpeed=i;
			Heng(Control_Across_yaw(SetAngle,yaw),0);
			}
	heng_flag=0;
	accel_flag=10;
	SetSpeed =3400;
	SetAngle =90;
	}
	Heng(Control_Across_yaw(SetAngle,yaw),0);
	  if(PES[8]==0)
       {		
          heng_flag=2;
		}
	if(heng_flag==2&&PES[8]==1)
	{
	 heng_flag=3;
	 tim=0;
	}
	if(heng_flag==3&&tim>22)
	{
			
			for(int i=3500;i>=0;i-=1)
			{
			delay_us(80);
			SetSpeed=i;
			Heng(Control_Across_yaw(SetAngle,yaw),0);
			}
			start_flag =14;
	}
	
}
/////////////////////////////////////第五条垄道加速
else if(start_flag ==14)
{
   
//	SetAngle =0;
	if(accel_flag==10)
	{ 
	HAL_UART_Transmit (&huart3 ,"0",1,10);//板间数据交流起始标志：1
		for(int i=0;i<3650;i+=1)//梯形加速   
		{
			delay_us(300);
			SetSpeed=i;
			Forward_Mid_Q(Control_Across_yaw(SetAngle,yaw),0,0);
		}
		tim=0; 
		accel_flag=11;		
	SetSpeed =3700;		
	}

//	SetAngle =0;
	Forward_Mid_Q(Control_Across_yaw(SetAngle,yaw),0,0);
	if(PES[4]==0||PES[7]==0)//左右侧光电管做入垄检测，当入垄后再进行超声波pid计算
	{ 
	  Forward_Mid_Q(Control_Across_yaw(SetAngle,yaw),Control_sonic(100,chaoshengbo)/2,0);
	  SetSpeed =3700;
	}
    if(PES[8]==0)  //后侧灯管
	{
		
//			for(int i=3650;i>=0;i-=1)
//			{
//			delay_us(470);
//			SetSpeed=i; 
//			Forward_L(0,0);//Control_Across_yaw(SetAngle,yaw)
//			}
	  	for(int i=3500;i>=2000;i-=0.2)
			{
				delay_us(700); 
				SetSpeed=i;
			  Forward_L(Control_Across_yaw(SetAngle,yaw),0);
			}
			
			for(int i=2000;i>=1000;i-=1)
			{
			delay_us(700);
			SetSpeed=i;
			Forward_L(Control_Across_yaw(SetAngle,yaw),0);//Control_Across_yaw(SetAngle,yaw)
			}
			for(int i=1000;i>=0;i-=5)
			{
			delay_us(700);
			SetSpeed=i;
			Forward_L(Control_Across_yaw(SetAngle,yaw),0);//Control_Across_yaw(SetAngle,yaw)
			}
			SetSpeed =0;	
			tim=0;
			start_flag=15;/////////
	}
}
//////////////////////////第五次横向移动到下一个笼道

else if(start_flag ==15)
{
	
//	 tim=0;	
	if(	accel_flag==11)
	{	for(int i=0;i<3000;i+=2)
			{
			delay_us(300);
			SetSpeed=i;
			Heng(Control_Across_yaw(SetAngle,yaw),0);
			}
	heng_flag=0;
	accel_flag=12;
	SetSpeed =3400;
	SetAngle =90;
	}
	Heng(Control_Across_yaw(SetAngle,yaw),0);
	  if(PES[3]==0)
       {		
          heng_flag=2;
		}
	if(heng_flag==2&&PES[3]==1)
	{
	 heng_flag=3;
	 tim=0;
	}
	if(heng_flag==3&&tim>22)
	{
			
			for(int i=3500;i>=0;i-=1)
			{
			delay_us(80);
			SetSpeed=i;
			Heng(Control_Across_yaw(SetAngle,yaw),0);
			}
			start_flag =16;
	}
	
}
//////////////////////////第六条垄道作业

else if(start_flag ==16)
{

//	SetAngle =0;
	if(accel_flag==12)
	{
		HAL_UART_Transmit (&huart3 ,"3",1,10);//板间数据交流起始标志：1
		for(int i=0;i<3600;i+=1)//梯形加速
		{
			delay_us(300);
			SetSpeed=i;
			Forward_Mid_H(Control_Across_yaw(SetAngle,yaw),0,0);
		}
		tim=0;
		accel_flag=13;		
	SetSpeed =3500;		
	}

//	SetAngle =0;
	Forward_Mid_H(Control_Across_yaw(SetAngle,yaw),0,0);
	if(PES[4]==0||PES[7]==0)//左右侧光电管做入垄检测，当入垄后再进行超声波pid计算
	{
	  Forward_Mid_H(Control_Across_yaw(SetAngle,yaw),Control_sonic(120,chaoshengbo)/2,0);
	  SetSpeed =3500;
	}
    if(PES[3]==0)  //后侧灯管
	{
		
			for(int i=3500;i>=0;i-=1)
			{
			delay_us(250);
			SetSpeed=-i;
			Forward_L(0,0);//Control_Across_yaw(SetAngle,yaw)
			}
			SetSpeed =0;	
			tim=0;
			start_flag=17;////////////////////////////////////////////////////
  HAL_UART_Transmit (&huart3 ,"0",1,10);//板间数据交流起始标志：0
	}

}
////////////////////////////离开赛道
else if(start_flag ==17)
{
	
//	 tim=0;	
//	SetAngle=90;
	if(	accel_flag==13)
	{	for(int i=0;i<3000;i+=2)
			{
			delay_us(300);
			SetSpeed=i;
			Heng(Control_Across_yaw(SetAngle,yaw),0);
			}
	accel_flag=0;
	SetSpeed =3400;
	SetAngle =90;
	} 
	Heng(Control_Across_yaw(SetAngle,yaw),0);
	 if(tim>160)
		{
			
			for(int i=3500;i>=0;i-=1)
			{
			delay_us(80);
			SetSpeed=i;
			Heng2(Control_Across_yaw(SetAngle,yaw),0);
			}
			start_flag =0;
		}
	
}

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{

	switch(GPIO_Pin ) 
	{
		case KEY1_Pin: 
					   break;
		case KEY2_Pin: start_flag =0;
					   break;
		case KEY3_Pin:start_flag =1;
					   accel_flag =1; break;
		case KEY4_Pin: 
			HAL_UART_Transmit (&huart3 ,"3",1,10);	
			break;
		case KEY5_Pin:
   HAL_UART_Transmit (&huart3 ,"1",1,10);
		break;
		case KEY6_Pin:
   HAL_UART_Transmit (&huart3 ,"0",1,10);			break;

	}
	
}


//void HAL_TIM_PeriodElapsedCallback (TIM_HandleTypeDef *htim)
//{		
//	Sonic_Overflow_Capture(TIM5CH1_CAPTURE_STA,TIM5CH1_CAPTURE_STA);
//	
//	if(htim->Instance  == htim2 .Instance )
//	{
//		Num1++;
//		Num2++;
//		
//		if(Num1 ==500)
//		{		 
//			LED_Toggle();
//			Num1=0;
//		}
//	}
//}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
