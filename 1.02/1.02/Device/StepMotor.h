#ifndef __STEPMOTOR_H
#define __STEPMOTOR_H

#include "gpio.h"
//#include "Delay_us.h"
#include "math.h"

#define X_Motor_OFF()          HAL_TIM_OC_Stop_IT(&htim1,TIM_CHANNEL_1)
#define Y_Motor_OFF()          HAL_TIM_OC_Stop_IT(&htim1,TIM_CHANNEL_2)
#define Z_Motor_OFF()          HAL_TIM_OC_Stop_IT(&htim1,TIM_CHANNEL_3)
 
/* 电机结构体宏定义 ------------------------------------------------------------------*/
typedef struct {
  __IO uint8_t  run_state ;  // 电机旋转状态
  __IO uint8_t  dir ;        // 电机旋转方向
  __IO int32_t  step_delay;  // 下个脉冲周期（时间间隔），启动时为加速度
  __IO uint32_t decel_start; // 启动减速位置
  __IO int32_t  decel_val;   // 减速阶段步数
  __IO int32_t  min_delay;   // 最小脉冲周期(最大速度，即匀速段速度)
  __IO int32_t  accel_count; // 加减速阶段计数值
}X_speedRampData;

typedef struct {
  __IO uint8_t  run_state ;  // 电机旋转状态
  __IO uint8_t  dir ;        // 电机旋转方向
  __IO int32_t  step_delay;  // 下个脉冲周期（时间间隔），启动时为加速度
  __IO uint32_t decel_start; // 启动减速位置
  __IO int32_t  decel_val;   // 减速阶段步数
  __IO int32_t  min_delay;   // 最小脉冲周期(最大速度，即匀速段速度)
  __IO int32_t  accel_count; // 加减速阶段计数值
}Y_speedRampData;

typedef struct {
  __IO uint8_t  run_state ;  // 电机旋转状态
  __IO uint8_t  dir ;        // 电机旋转方向
  __IO int32_t  step_delay;  // 下个脉冲周期（时间间隔），启动时为加速度
  __IO uint32_t decel_start; // 启动减速位置
  __IO int32_t  decel_val;   // 减速阶段步数
  __IO int32_t  min_delay;   // 最小脉冲周期(最大速度，即匀速段速度)
  __IO int32_t  accel_count; // 加减速阶段计数值
}Z_speedRampData;

extern X_speedRampData X_srd;
extern __IO int32_t  X_step_position;
extern __IO uint8_t  X_MotionStatus; 

extern Y_speedRampData Y_srd;
extern __IO int32_t  Y_step_position;
extern __IO uint8_t  Y_MotionStatus; 

extern Z_speedRampData Z_srd;
extern __IO int32_t  Z_step_position;
extern __IO uint8_t  Z_MotionStatus; 

#define STEPMOTOR_TIM_PRESCALER               38  // 步进电机驱动器细分设置为：   32  细分
// 定义定时器周期，输出比较模式周期设置为0xFFFF
#define STEPMOTOR_TIM_PERIOD                  0xFFFF
#define T1_FREQ                               (84000000/(STEPMOTOR_TIM_PRESCALER+1)) // 频率ft值
 
#define FALSE                                 0
#define TRUE                                  1
#define CW                                    0 // 顺时针
#define CCW                                   1 // 逆时针
 
#define STOP                                  0 // 加减速曲线状态：停止
#define ACCEL                                 1 // 加减速曲线状态：加速阶段
#define DECEL                                 2 // 加减速曲线状态：减速阶段
#define RUN                                   3 // 加减速曲线状态：匀速阶段
#define FSPR                                  200          //步进电机单圈步数
#define MICRO_STEP                            16           // 步进电机驱动器细分数
#define SPR                                   (FSPR*MICRO_STEP)   // 旋转一圈需要的脉冲数
 
// 数学常数
#define ALPHA                                 ((float)(2*3.14159/SPR))       // α= 2*pi/spr
#define A_T_x10                               ((float)(10*ALPHA*T1_FREQ))
#define T1_FREQ_148                           ((float)((T1_FREQ*0.676)/10)) // 0.69为误差修正值
#define A_SQ                                  ((float)(2*100000*ALPHA)) 
#define A_x200                                ((float)(200*ALPHA))
 
#define X_STEPMOTOR_DIR_FORWARD()                 HAL_GPIO_WritePin(M1_DIR_GPIO_Port,M1_DIR_Pin,GPIO_PIN_RESET)
#define X_STEPMOTOR_DIR_REVERSAL()                HAL_GPIO_WritePin(M1_DIR_GPIO_Port,M1_DIR_Pin,GPIO_PIN_SET)

#define Y_STEPMOTOR_DIR_FORWARD()                 HAL_GPIO_WritePin(M2_DIR_GPIO_Port,M2_DIR_Pin,GPIO_PIN_SET)
#define Y_STEPMOTOR_DIR_REVERSAL()                HAL_GPIO_WritePin(M2_DIR_GPIO_Port,M2_DIR_Pin,GPIO_PIN_RESET)

#define Z_STEPMOTOR_DIR_FORWARD()                 HAL_GPIO_WritePin(M2_DIR_GPIO_Port,M3_DIR_Pin,GPIO_PIN_SET)
#define Z_STEPMOTOR_DIR_REVERSAL()                HAL_GPIO_WritePin(M2_DIR_GPIO_Port,M3_DIR_Pin,GPIO_PIN_RESET)

#define X_STEPMOTOR_OUTPUT_ENABLE();			  HAL_GPIO_WritePin(M1_ENA_GPIO_Port,M1_ENA_Pin,GPIO_PIN_SET)
#define X_STEPMOTOR_OUTPUT_DISABLE();			  HAL_GPIO_WritePin(M1_ENA_GPIO_Port ,M1_ENA_Pin,GPIO_PIN_RESET)

#define Y_STEPMOTOR_OUTPUT_ENABLE();			  HAL_GPIO_WritePin(M2_ENA_GPIO_Port,M2_ENA_Pin,GPIO_PIN_SET)
#define Y_STEPMOTOR_OUTPUT_DISABLE();			  HAL_GPIO_WritePin(M2_ENA_GPIO_Port,M2_ENA_Pin,GPIO_PIN_RESET)

#define Z_STEPMOTOR_OUTPUT_ENABLE();			  HAL_GPIO_WritePin(M2_ENA_GPIO_Port,M3_ENA_Pin,GPIO_PIN_SET)
#define Z_STEPMOTOR_OUTPUT_DISABLE();			  HAL_GPIO_WritePin(M2_ENA_GPIO_Port,M3_ENA_Pin,GPIO_PIN_RESET)

//>0 顺时针 上升 收回

/*可以被外部使用的函数声明*/
void STEPMOTOR_MOVE(__IO int32_t step, __IO uint32_t accel, __IO uint32_t decel, __IO uint32_t speed);  //x
void LIFTMOTOR_MOVE(__IO int32_t step, __IO uint32_t accel, __IO uint32_t decel, __IO uint32_t speed);  //z
void CROSMOTOR_MOVE(__IO int32_t step, __IO uint32_t accel, __IO uint32_t decel, __IO uint32_t speed);  //y

#endif
