#include "STEPMOTOR.h"
#include "math.h"
//#include "All.h"

X_speedRampData X_srd             = {STOP,CW,0,0,0,0,0};         // X加减速曲线变量
__IO int32_t  X_step_position     = 0;           // X当前位置
__IO uint8_t  X_MotionStatus      = 0;           //X是否在运动？0：停止，1：运动

Y_speedRampData Y_srd             = {STOP,CW,0,0,0,0,0};         // Y加减速曲线变量
__IO int32_t  Y_step_position     = 0;           // Y当前位置
__IO uint8_t  Y_MotionStatus      = 0;           //Y是否在运动？0：停止，1：运动

Z_speedRampData Z_srd             = {STOP,CW,0,0,0,0,0};         // Z加减速曲线变量
__IO int32_t  Z_step_position     = 0;           // Z当前位置
__IO uint8_t  Z_MotionStatus      = 0;           //Z是否在运动？0：停止，1：运动

extern TIM_HandleTypeDef htim1;


/**
  * 函数功能: 相对位置运动：运动给定的步数
  * 输入参数: step：移动的步数 (正数为顺时针，负数为逆时针).  
              accel  加速度,实际值为accel*0.025*rad/sec^2
              decel  减速度,实际值为decel*0.025*rad/sec^2
              speed  最大速度,实际值为speed*0.05*rad/sec
  * 返 回 值: 无
  * 说    明: 以给定的步数移动步进电机，先加速到最大速度，然后在合适位置开始
  *           减速至停止，使得整个运动距离为指定的步数。如果加减速阶段很短并且
  *           速度很慢，那还没达到最大速度就要开始减速
  */
/***X电机梯形加减速***/
void STEPMOTOR_MOVE(__IO int32_t step, __IO uint32_t accel, __IO uint32_t decel, __IO uint32_t speed)
{  
  __IO uint16_t X_tim_count;
  // X达到最大速度时的步数
  __IO uint32_t X_max_s_lim;
  // X必须要开始减速的步数（如果加速没有达到最大速度）
  __IO uint32_t X_accel_lim;
 
  if(step < 0) // 步数为负数
  {
    X_srd.dir = CCW; // 逆时针方向旋转
    X_STEPMOTOR_DIR_REVERSAL();
    step =-step;   // 获取步数绝对值
  }
  else
  {
    X_srd.dir = CW; // 顺时针方向旋转
    X_STEPMOTOR_DIR_FORWARD();
  }

 if(step == 1)    // 步数为1
  {
    X_srd.accel_count = -1;   // 只移动一步
    X_srd.run_state = DECEL;  // 减速状态.
    X_srd.step_delay = 1000;	// 短延时	
  }
  else if(step != 0)  // 如果目标运动步数不为0
  {
    // 我们的驱动器用户手册有详细的计算及推导过程
 
    // 设置最大速度极限, 计算得到min_delay用于定时器的计数器的值。
    // min_delay = (alpha / tt)/ w
    X_srd.min_delay = (int32_t)(A_T_x10/speed);
 
    // 通过计算第一个(c0) 的步进延时来设定加速度，其中accel单位为0.1rad/sec^2
    // step_delay = 1/tt * sqrt(2*alpha/accel)
    // step_delay = ( tfreq*0.676/10 )*10 * sqrt( (2*alpha*100000) / (accel*10) )/100
    X_srd.step_delay = (int32_t)((T1_FREQ_148 * sqrt(A_SQ / accel))/10);
 
    // 计算多少步之后达到最大速度的限制
    // max_s_lim = speed^2 / (2*alpha*accel)
    X_max_s_lim = (uint32_t)(speed*speed/(A_x200*accel/10));
    // 如果达到最大速度小于0.5步，我们将四舍五入为0
    // 但实际我们必须移动至少一步才能达到想要的速度
    if(X_max_s_lim == 0){
      X_max_s_lim = 1;
    }
 
    // 计算多少步之后我们必须开始减速
    // n1 = (n1+n2)decel / (accel + decel)
    X_accel_lim = (uint32_t)(step*decel/(accel+decel));
    // 我们必须加速至少1步才能才能开始减速.
    if(X_accel_lim == 0){
      X_accel_lim = 1;
    }
 
    // 使用限制条件我们可以计算出减速阶段步数
    if(X_accel_lim <= X_max_s_lim){
      X_srd.decel_val = X_accel_lim - step;
    }
    else{
      X_srd.decel_val = -(X_max_s_lim*accel/decel);
    }
    // 当只剩下一步我们必须减速
    if(X_srd.decel_val == 0){
      X_srd.decel_val = -1;
    }
 
    // 计算开始减速时的步数
    X_srd.decel_start = step + X_srd.decel_val;
 
    // 如果最大速度很慢，我们就不需要进行加速运动
    if(X_srd.step_delay <= X_srd.min_delay){
      X_srd.step_delay = X_srd.min_delay;
      X_srd.run_state = RUN;
    }
    else{
      X_srd.run_state = ACCEL;
    }    
    // 复位加速度计数值
    X_srd.accel_count = 0;
  }
  X_MotionStatus = 1; // 电机为运动状态
  X_tim_count=__HAL_TIM_GET_COUNTER(&htim1);
  __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,X_tim_count+X_srd.step_delay); // 设置定时器比较值
	HAL_TIM_OC_Start_IT(&htim1,TIM_CHANNEL_1);
  TIM_CCxChannelCmd(TIM1,TIM_CHANNEL_1, TIM_CCx_ENABLE);// 使能定时器通道 
}

/***Y电机梯形加减速***/
void LIFTMOTOR_MOVE(__IO int32_t step, __IO uint32_t accel, __IO uint32_t decel, __IO uint32_t speed)
{  
  __IO uint16_t Y_tim_count;// X达到最大速度时的步数
  __IO uint32_t Y_max_s_lim;// X必须要开始减速的步数（如果加速没有达到最大速度）
  __IO uint32_t Y_accel_lim;
 
 
  if(step < 0) // 步数为负数
  {
    Y_srd.dir = CCW; // 逆时针方向旋转
		Y_STEPMOTOR_DIR_FORWARD();
    step =-step;   // 获取步数绝对值
  }
  else
  {
    Y_srd.dir = CW; // 顺时针方向旋转
		Y_STEPMOTOR_DIR_REVERSAL();
  }
  
  if(step == 1)    // 步数为1
  {
    Y_srd.accel_count = -1;   // 只移动一步
    Y_srd.run_state = DECEL;  // 减速状态.
    Y_srd.step_delay = 1000;	// 短延时	
  }
  else if(step != 0)  // 如果目标运动步数不为0
  {
    // 我们的驱动器用户手册有详细的计算及推导过程
 
    // 设置最大速度极限, 计算得到min_delay用于定时器的计数器的值。
    // min_delay = (alpha / tt)/ w
    Y_srd.min_delay = (int32_t)(A_T_x10/speed);
 
    // 通过计算第一个(c0) 的步进延时来设定加速度，其中accel单位为0.1rad/sec^2
    // step_delay = 1/tt * sqrt(2*alpha/accel)
    // step_delay = ( tfreq*0.676/10 )*10 * sqrt( (2*alpha*100000) / (accel*10) )/100
    Y_srd.step_delay = (int32_t)((T1_FREQ_148 * sqrt(A_SQ / accel))/10);
 
    // 计算多少步之后达到最大速度的限制
    // max_s_lim = speed^2 / (2*alpha*accel)
    Y_max_s_lim = (uint32_t)(speed*speed/(A_x200*accel/10));
    // 如果达到最大速度小于0.5步，我们将四舍五入为0
    // 但实际我们必须移动至少一步才能达到想要的速度
    if(Y_max_s_lim == 0){
      Y_max_s_lim = 1;
    }
 
    // 计算多少步之后我们必须开始减速
    // n1 = (n1+n2)decel / (accel + decel)
    Y_accel_lim = (uint32_t)(step*decel/(accel+decel));
    // 我们必须加速至少1步才能才能开始减速.
    if(Y_accel_lim == 0){
      Y_accel_lim = 1;
    }
 
    // 使用限制条件我们可以计算出减速阶段步数
    if(Y_accel_lim <= Y_max_s_lim){
      Y_srd.decel_val = Y_accel_lim - step;
    }
    else{
      Y_srd.decel_val = -(Y_max_s_lim*accel/decel);
    }
    // 当只剩下一步我们必须减速
    if(Y_srd.decel_val == 0){
      Y_srd.decel_val = -1;
    }
 
    // 计算开始减速时的步数
    Y_srd.decel_start = step + Y_srd.decel_val;
 
    // 如果最大速度很慢，我们就不需要进行加速运动
    if(Y_srd.step_delay <= Y_srd.min_delay){
      Y_srd.step_delay = Y_srd.min_delay;
      Y_srd.run_state = RUN;
    }
    else{
      Y_srd.run_state = ACCEL;
    }    
    // 复位加速度计数值
    Y_srd.accel_count = 0;
  }
  Y_MotionStatus = 1; // 电机为运动状态
  Y_tim_count=__HAL_TIM_GET_COUNTER(&htim1);
  __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,Y_tim_count+Y_srd.step_delay); // 设置定时器比较值
	HAL_TIM_OC_Start_IT(&htim1,TIM_CHANNEL_2);
  TIM_CCxChannelCmd(TIM1,TIM_CHANNEL_2, TIM_CCx_ENABLE);// 使能定时器通道 
  Y_STEPMOTOR_OUTPUT_ENABLE();
}


/***Z电机梯形加减速***/
void CROSMOTOR_MOVE(__IO int32_t step, __IO uint32_t accel, __IO uint32_t decel, __IO uint32_t speed)
{  
  __IO uint16_t Z_tim_count;// X达到最大速度时的步数
  __IO uint32_t Z_max_s_lim;// X必须要开始减速的步数（如果加速没有达到最大速度）
  __IO uint32_t Z_accel_lim;
 
 
  if(step < 0) // 步数为负数
  {
    Z_srd.dir = CCW; // 逆时针方向旋转
		Z_STEPMOTOR_DIR_FORWARD();
    step =-step;   // 获取步数绝对值
  }
  else
  {
    Z_srd.dir = CW; // 顺时针方向旋转
		Z_STEPMOTOR_DIR_REVERSAL();
  }
  
  if(step == 1)    // 步数为1
  {
    Z_srd.accel_count = -1;   // 只移动一步
    Z_srd.run_state = DECEL;  // 减速状态.
    Z_srd.step_delay = 1000;	// 短延时	
  }
  else if(step != 0)  // 如果目标运动步数不为0
  {
    // 我们的驱动器用户手册有详细的计算及推导过程
 
    // 设置最大速度极限, 计算得到min_delay用于定时器的计数器的值。
    // min_delay = (alpha / tt)/ w
    Z_srd.min_delay = (int32_t)(A_T_x10/speed);
 
    // 通过计算第一个(c0) 的步进延时来设定加速度，其中accel单位为0.1rad/sec^2
    // step_delay = 1/tt * sqrt(2*alpha/accel)
    // step_delay = ( tfreq*0.676/10 )*10 * sqrt( (2*alpha*100000) / (accel*10) )/100
    Z_srd.step_delay = (int32_t)((T1_FREQ_148 * sqrt(A_SQ / accel))/10);
 
    // 计算多少步之后达到最大速度的限制
    // max_s_lim = speed^2 / (2*alpha*accel)
    Z_max_s_lim = (uint32_t)(speed*speed/(A_x200*accel/10));
    // 如果达到最大速度小于0.5步，我们将四舍五入为0
    // 但实际我们必须移动至少一步才能达到想要的速度
    if(Z_max_s_lim == 0){
      Z_max_s_lim = 1;
    }
 
    // 计算多少步之后我们必须开始减速
    // n1 = (n1+n2)decel / (accel + decel)
    Z_accel_lim = (uint32_t)(step*decel/(accel+decel));
    // 我们必须加速至少1步才能才能开始减速.
    if(Z_accel_lim == 0){
      Z_accel_lim = 1;
    }
 
    // 使用限制条件我们可以计算出减速阶段步数
    if(Z_accel_lim <= Z_max_s_lim){
      Z_srd.decel_val = Z_accel_lim - step;
    }
    else{
      Z_srd.decel_val = -(Z_max_s_lim*accel/decel);
    }
    // 当只剩下一步我们必须减速
    if(Z_srd.decel_val == 0){
      Z_srd.decel_val = -1;
    }
 
    // 计算开始减速时的步数
    Z_srd.decel_start = step + Z_srd.decel_val;
 
    // 如果最大速度很慢，我们就不需要进行加速运动
    if(Z_srd.step_delay <= Z_srd.min_delay){
      Z_srd.step_delay = Z_srd.min_delay;
      Z_srd.run_state = RUN;
    }
    else{
      Z_srd.run_state = ACCEL;
    }    
    // 复位加速度计数值
    Z_srd.accel_count = 0;
  }
  Z_MotionStatus = 1; // 电机为运动状态
  Z_tim_count=__HAL_TIM_GET_COUNTER(&htim1);
  __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,Z_tim_count+Z_srd.step_delay); // 设置定时器比较值
	HAL_TIM_OC_Start_IT(&htim1,TIM_CHANNEL_3);
  TIM_CCxChannelCmd(TIM1,TIM_CHANNEL_3, TIM_CCx_ENABLE);// 使能定时器通道 
  Z_STEPMOTOR_OUTPUT_ENABLE();
}

/**
  * 函数功能: 定时器中断服务函数
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 实现加减速过程
  */
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)//定时器中断处理
{ 
	/*X电机*/
  __IO uint16_t X_tim_count=0;
  // X保存新（下）一个延时周期
  uint16_t X_new_step_delay=0;
  // X加速过程中最后一次延时（脉冲周期）.
  __IO static uint16_t X_last_accel_delay=0;
  // X总移动步数计数器
  __IO static uint32_t X_step_count = 0;
  // X记录new_step_delay中的余数，提高下一步计算的精度
  __IO static int32_t X_rest = 0;
  //X定时器使用翻转模式，需要进入两次中断才输出一个完整脉冲
  __IO static uint8_t X_i=0;
	
	/*Y电机*/
	__IO uint16_t Y_tim_count=0;
  // Y保存新（下）一个延时周期
  uint16_t Y_new_step_delay=0;
  // Y加速过程中最后一次延时（脉冲周期）.
  __IO static uint16_t Y_last_accel_delay=0;
  // Y总移动步数计数器
  __IO static uint32_t Y_step_count = 0;
  // X记录new_step_delay中的余数，提高下一步计算的精度
  __IO static int32_t Y_rest = 0;
  //Y定时器使用翻转模式，需要进入两次中断才输出一个完整脉冲
  __IO static uint8_t Y_i=0;

	/*Z电机*/
	__IO uint16_t Z_tim_count=0;
  // Z保存新（下）一个延时周期
  uint16_t Z_new_step_delay=0;
  // Z加速过程中最后一次延时（脉冲周期）.
  __IO static uint16_t Z_last_accel_delay=0;
  // Z总移动步数计数器
  __IO static uint32_t Z_step_count = 0;
  // X记录new_step_delay中的余数，提高下一步计算的精度
  __IO static int32_t Z_rest = 0;
  //Z定时器使用翻转模式，需要进入两次中断才输出一个完整脉冲
  __IO static uint8_t Z_i=0;
	
	if(htim->Instance==TIM1)
{
	/*X电机的定时器配置*/
  if(__HAL_TIM_GET_IT_SOURCE(&htim1, TIM_IT_CC1) !=RESET)
  {
    // 清楚定时器中断
    __HAL_TIM_CLEAR_IT(&htim1, TIM_IT_CC1);
    
    // 设置比较值
    X_tim_count=__HAL_TIM_GET_COUNTER(&htim1);
    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,X_tim_count+X_srd.step_delay);
 
    X_i++;     // 定时器中断次数计数值
    if(X_i==2) // 2次，说明已经输出一个完整脉冲
    {
      X_i=0;   // 清零定时器中断次数计数值
      switch(X_srd.run_state) // 加减速曲线阶段
      {
        case STOP:
          X_step_count = 0;  // 清零步数计数器
          X_rest = 0;        // 清零余值
          // 关闭通道
          TIM_CCxChannelCmd(TIM1, TIM_CHANNEL_1, TIM_CCx_DISABLE);        
          __HAL_TIM_CLEAR_FLAG(&htim1,TIM_FLAG_CC1);
          X_STEPMOTOR_OUTPUT_DISABLE();
          X_MotionStatus = 0;  //  电机为停止状态     
          break;
 
        case ACCEL:
          X_step_count++;      // 步数加1
          if(X_srd.dir==CW)
          {	  	
            X_step_position++; // 绝对位置加1
          }
          else
          {
            X_step_position--; // 绝对位置减1
          }
          X_srd.accel_count++; // 加速计数值加1
          X_new_step_delay = X_srd.step_delay - (((2 *X_srd.step_delay) + X_rest)/(4 * X_srd.accel_count + 1));//计算新(下)一步脉冲周期(时间间隔)
          X_rest = ((2 * X_srd.step_delay)+X_rest)%(4 * X_srd.accel_count + 1);// 计算余数，下次计算补上余数，减少误差
          if(X_step_count >= X_srd.decel_start)// 检查是够应该开始减速
          {
            X_srd.accel_count = X_srd.decel_val; // 加速计数值为减速阶段计数值的初始值
            X_srd.run_state = DECEL;           // 下个脉冲进入减速阶段
          }
          else if(X_new_step_delay <= X_srd.min_delay) // 检查是否到达期望的最大速度
          {
            X_last_accel_delay = X_new_step_delay; // 保存加速过程中最后一次延时（脉冲周期）
            X_new_step_delay = X_srd.min_delay;    // 使用min_delay（对应最大速度speed）
            X_rest = 0;                          // 清零余值
            X_srd.run_state = RUN;               // 设置为匀速运行状态
          }
          break;
 
        case RUN:
          X_step_count++;  // 步数加1
          if(X_srd.dir==CW)
          {	  	
            X_step_position++; // 绝对位置加1
          }
          else
          {
            X_step_position--; // 绝对位置减1
          }
          X_new_step_delay = X_srd.min_delay;     // 使用min_delay（对应最大速度speed）
          if(X_step_count >= X_srd.decel_start)   // 需要开始减速
          {
            X_srd.accel_count = X_srd.decel_val;  // 减速步数做为加速计数值
            X_new_step_delay = X_last_accel_delay;// 加阶段最后的延时做为减速阶段的起始延时(脉冲周期)
            X_srd.run_state = DECEL;            // 状态改变为减速
          }
          break;
 
        case DECEL:
          X_step_count++;  // 步数加1
          if(X_srd.dir==CW)
          {	  	
            X_step_position++; // 绝对位置加1
          }
          else
          {
            X_step_position--; // 绝对位置减1
          }
          X_srd.accel_count++;
          X_new_step_delay = X_srd.step_delay - (((2 * X_srd.step_delay) + X_rest)/(4 * X_srd.accel_count + 1)); //计算新(下)一步脉冲周期(时间间隔)
          X_rest = ((2 * X_srd.step_delay)+X_rest)%(4 * X_srd.accel_count + 1);// 计算余数，下次计算补上余数，减少误差
          
          //检查是否为最后一步
          if(X_srd.accel_count >= 0)
          {
            X_srd.run_state = STOP;
          }
          break;
      }      
      X_srd.step_delay = X_new_step_delay; // 为下个(新的)延时(脉冲周期)赋值
    }
  }
	
	
 if(__HAL_TIM_GET_IT_SOURCE(&htim1, TIM_IT_CC2) !=RESET)
  { 
    // 清楚定时器中断
    __HAL_TIM_CLEAR_IT(&htim1, TIM_IT_CC2);
    
    // 设置比较值
    Y_tim_count=__HAL_TIM_GET_COUNTER(&htim1);
    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,Y_tim_count+Y_srd.step_delay);
 
    Y_i++;     // 定时器中断次数计数值
    if(Y_i==2) // 2次，说明已经输出一个完整脉冲
    {
      Y_i=0;   // 清零定时器中断次数计数值
      switch(Y_srd.run_state) // 加减速曲线阶段
      {
        case STOP:
          Y_step_count = 0;  // 清零步数计数器
          Y_rest = 0;        // 清零余值
          // 关闭通道
          TIM_CCxChannelCmd(TIM1, TIM_CHANNEL_2, TIM_CCx_DISABLE);        
          __HAL_TIM_CLEAR_FLAG(&htim1,TIM_FLAG_CC2);
          Y_STEPMOTOR_OUTPUT_DISABLE();
          Y_MotionStatus = 0;  //  电机为停止状态     
          break;
 
        case ACCEL:
          Y_step_count++;      // 步数加1
          if(Y_srd.dir==CW)
          {	  	
            Y_step_position++; // 绝对位置加1
          }
          else
          {
            Y_step_position--; // 绝对位置减1
          }
          Y_srd.accel_count++; // 加速计数值加1
          Y_new_step_delay = Y_srd.step_delay - (((2 *Y_srd.step_delay) + Y_rest)/(4 * Y_srd.accel_count + 1));//计算新(下)一步脉冲周期(时间间隔)
          Y_rest = ((2 * Y_srd.step_delay)+Y_rest)%(4 * Y_srd.accel_count + 1);// 计算余数，下次计算补上余数，减少误差
          if(Y_step_count >= Y_srd.decel_start)// 检查是够应该开始减速
          {
            Y_srd.accel_count = Y_srd.decel_val; // 加速计数值为减速阶段计数值的初始值
            Y_srd.run_state = DECEL;           // 下个脉冲进入减速阶段
          }
          else if(Y_new_step_delay <= Y_srd.min_delay) // 检查是否到达期望的最大速度
          {
            Y_last_accel_delay = Y_new_step_delay; // 保存加速过程中最后一次延时（脉冲周期）
            Y_new_step_delay = Y_srd.min_delay;    // 使用min_delay（对应最大速度speed）
            Y_rest = 0;                          // 清零余值
            Y_srd.run_state = RUN;               // 设置为匀速运行状态
          }
          break;
 
        case RUN:
          Y_step_count++;  // 步数加1
          if(Y_srd.dir==CW)
          {	  	
            Y_step_position++; // 绝对位置加1
          }
          else
          {
            Y_step_position--; // 绝对位置减1
          }
          Y_new_step_delay = Y_srd.min_delay;     // 使用min_delay（对应最大速度speed）
          if(Y_step_count >= Y_srd.decel_start)   // 需要开始减速
          {
            Y_srd.accel_count = Y_srd.decel_val;  // 减速步数做为加速计数值
            Y_new_step_delay = Y_last_accel_delay;// 加阶段最后的延时做为减速阶段的起始延时(脉冲周期)
            Y_srd.run_state = DECEL;            // 状态改变为减速
          }
          break;
 
        case DECEL:
          Y_step_count++;  // 步数加1
          if(Y_srd.dir==CW)
          {	  	
            Y_step_position++; // 绝对位置加1
          }
          else
          {
            Y_step_position--; // 绝对位置减1
          }
          Y_srd.accel_count++;
          Y_new_step_delay = Y_srd.step_delay - (((2 * Y_srd.step_delay) + Y_rest)/(4 * Y_srd.accel_count + 1)); //计算新(下)一步脉冲周期(时间间隔)
          Y_rest = ((2 * Y_srd.step_delay)+Y_rest)%(4 * Y_srd.accel_count + 1);// 计算余数，下次计算补上余数，减少误差
          
          //检查是否为最后一步
          if(Y_srd.accel_count >= 0)
          {
            Y_srd.run_state = STOP;
          }
          break;
      }      
      Y_srd.step_delay = Y_new_step_delay; // 为下个(新的)延时(脉冲周期)赋值
    }
  }

	
 if(__HAL_TIM_GET_IT_SOURCE(&htim1, TIM_IT_CC3) !=RESET)
  { 
    // 清楚定时器中断
    __HAL_TIM_CLEAR_IT(&htim1, TIM_IT_CC3);
    
    // 设置比较值
    Z_tim_count=__HAL_TIM_GET_COUNTER(&htim1);
    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,Z_tim_count+Z_srd.step_delay);
 
    Z_i++;     // 定时器中断次数计数值
    if(Z_i==2) // 2次，说明已经输出一个完整脉冲
    {
      Z_i=0;   // 清零定时器中断次数计数值
      switch(Z_srd.run_state) // 加减速曲线阶段
      {
        case STOP:
          Z_step_count = 0;  // 清零步数计数器
          Z_rest = 0;        // 清零余值
          // 关闭通道
          TIM_CCxChannelCmd(TIM1, TIM_CHANNEL_3, TIM_CCx_DISABLE);        
          __HAL_TIM_CLEAR_FLAG(&htim1,TIM_FLAG_CC3);
          Z_STEPMOTOR_OUTPUT_DISABLE();
          Z_MotionStatus = 0;  //  电机为停止状态     
          break;
 
        case ACCEL:
          Z_step_count++;      // 步数加1
          if(Z_srd.dir==CW)
          {	  	
            Z_step_position++; // 绝对位置加1
          }
          else
          {
            Z_step_position--; // 绝对位置减1
          }
          Z_srd.accel_count++; // 加速计数值加1
          Z_new_step_delay = Z_srd.step_delay - (((2 *Z_srd.step_delay) + Z_rest)/(4 * Z_srd.accel_count + 1));//计算新(下)一步脉冲周期(时间间隔)
          Z_rest = ((2 * Z_srd.step_delay)+Z_rest)%(4 * Z_srd.accel_count + 1);// 计算余数，下次计算补上余数，减少误差
          if(Z_step_count >= Z_srd.decel_start)// 检查是够应该开始减速
          {
            Z_srd.accel_count = Z_srd.decel_val; // 加速计数值为减速阶段计数值的初始值
            Z_srd.run_state = DECEL;           // 下个脉冲进入减速阶段
          }
          else if(Z_new_step_delay <= Z_srd.min_delay) // 检查是否到达期望的最大速度
          {
            Z_last_accel_delay = Z_new_step_delay; // 保存加速过程中最后一次延时（脉冲周期）
            Z_new_step_delay = Z_srd.min_delay;    // 使用min_delay（对应最大速度speed）
            Z_rest = 0;                          // 清零余值
            Z_srd.run_state = RUN;               // 设置为匀速运行状态
          }
          break;
 
        case RUN:
          Z_step_count++;  // 步数加1
          if(Z_srd.dir==CW)
          {	  	
            Z_step_position++; // 绝对位置加1
          }
          else
          {
            Z_step_position--; // 绝对位置减1
          }
          Z_new_step_delay = Z_srd.min_delay;     // 使用min_delay（对应最大速度speed）
          if(Z_step_count >= Z_srd.decel_start)   // 需要开始减速
          {
            Z_srd.accel_count = Z_srd.decel_val;  // 减速步数做为加速计数值
            Z_new_step_delay = Z_last_accel_delay;// 加阶段最后的延时做为减速阶段的起始延时(脉冲周期)
            Z_srd.run_state = DECEL;            // 状态改变为减速
          }
          break;
 
        case DECEL:
          Z_step_count++;  // 步数加1
          if(Z_srd.dir==CW)
          {	  	
            Z_step_position++; // 绝对位置加1
          }
          else
          {
            Z_step_position--; // 绝对位置减1
          }
          Z_srd.accel_count++;
          Z_new_step_delay = Z_srd.step_delay - (((2 * Z_srd.step_delay) + Z_rest)/(4 * Z_srd.accel_count + 1)); //计算新(下)一步脉冲周期(时间间隔)
          Z_rest = ((2 * Z_srd.step_delay)+Z_rest)%(4 * Z_srd.accel_count + 1);// 计算余数，下次计算补上余数，减少误差
          
          //检查是否为最后一步
          if(Z_srd.accel_count >= 0)
          {
            Z_srd.run_state = STOP;
          }
          break;
      }      
      Z_srd.step_delay = Z_new_step_delay; // 为下个(新的)延时(脉冲周期)赋值
    }
  }

 }
}













