#include "encoder.h"
#include "mike.h"
#include "tim.h"
#include "Task_move.h"
#include "string.h"


/* ============================================================
 *  encoder.c —— 四个麦轮编码器读数 → 转速 → 滤波 → 里程计
 *  调用周期：10ms（由 Get_encoder() 在定时器/任务里调用）
 *  电机：减速比 1:14，编码器 11 线 × 4 倍频，名义转速 526rpm
 * ============================================================ */

/* 滤波后转速 E_motor，未滤波转速 E_motor_F（_F = Filter 前的原始值） */
EncoderStruct E_motor,E_motor_F;
CarStruct Car_speed,Car_s;	// Car_speed: vx/vy/yaw 当前速度； Car_s: x/y 累计路程

/* 四个轮子各一个独立滤波器，互不干扰 */
EncoderFilter_Struct EncoAspeed_Filter,EncoBspeed_Filter,EncoCspeed_Filter,EncoDspeed_Filter;

float angle;	// 调试用累计角度（当前未启用）

/* ------------------------------------------------------------
 *  Get_encoder() —— 顶层入口，10ms 被调用一次
 *  1) 读编码器并算出每个轮子的转速（rpm）
 *  2) 用麦轮逆解算把四轮转速 → 车体 vx / vy / yaw
 * ------------------------------------------------------------ */
void Get_encoder(void)
{
		Encoder_calc();		// 读取 + 滤波 + 里程计更新
		mike_speed2xy(&Car_speed.vx,&Car_speed.vy,&Car_speed.yaw,E_motor.speedA,E_motor.speedB,E_motor.speedC,E_motor.speedD);

		// 下面两行是把瞬时速度 × dt 累积成路程（积分），目前注释掉
		//Car_s.vx +=Car_speed.vx*6.28f*9.6f/60.0f*0.01f;
		//Car_s.vy +=Car_speed.vy*6.28f*9.6f/60.0f*0.01f;
}

/* 定时器原始计数寄存器值（CNT 寄存器），short = 16bit，TIM5 用 32bit */
short temp[3]={0,};
int32_t temp3;

/* 上一次的 CNT 值，用来做差分得到这 10ms 内增量 */
short last_tim_cnt[3]={0};
int32_t last_tim_cnt3;

/* 本次差分结果：10ms 内编码器的脉冲增量（带方向） */
short current_tim_cnt[3]={0};
int32_t current_tim_cnt3;

/* ------------------------------------------------------------
 *  Encoder_calc() —— 编码器核心计算
 *  关键点：利用 16bit 计数器自然回绕特性，相减得到带符号增量
 *         (short)(new - last) 自动处理 0xFFFF→0x0000 的回绕
 * ------------------------------------------------------------ */
void Encoder_calc(void)
{
		/* 1. 读取四个定时器的编码器计数 */
        temp[0] =(short)__HAL_TIM_GetCounter(&htim1);	// A 轮
		temp[1] =(short)__HAL_TIM_GetCounter(&htim3);	// B 轮
		temp[2] =(short)__HAL_TIM_GetCounter(&htim4);	// C 轮
		temp3 	=(int32_t)__HAL_TIM_GetCounter(&htim5);	// D 轮（32bit 计数器）

		/* 2. 16bit 三个轮子做差分：本次 - 上次 = 10ms 内增量 */
		for(uint8_t i=0; i<3; i++)
		{
        current_tim_cnt[i] = temp[i] - last_tim_cnt[i];	// short 自动回绕
        last_tim_cnt[i] = temp[i];
        }
		/* 3. 32bit 那一路也做差分 */
		current_tim_cnt3=temp3-last_tim_cnt3;
		last_tim_cnt3=temp3;

		/* 4. 32bit 计数器溢出修正：超过 int32 范围则补回 2^32 */
    if (current_tim_cnt3 >  2147483647)  current_tim_cnt3 -= 4294967296;
    if (current_tim_cnt3 < -2147483648) current_tim_cnt3 += 4294967296;


		/* 5. 脉冲数 → 转速(rpm) 换算
		 *    每圈脉冲数 = 11(线) × 4(倍频) × 14(减速比) = 616
		 *    10ms 一次采样 → 1s 内 100 次 → ×100 → ×60 才是 rpm
		 *    化简后：增量 × 6000 / (11*4*14) = 增量 × 6000/616
		 *    B/C/D 加负号是因为安装方向相反，统一为车前进时为正
		 */
		E_motor_F.speedA = (float)current_tim_cnt[0]*6000.0f/11/4/14;
		E_motor_F.speedB =-(float)current_tim_cnt[1]*6000.0f/11/4/14;
		E_motor_F.speedC =-(float)current_tim_cnt[2]*6000.0f/11/4/14;
		E_motor_F.speedD =-(float)current_tim_cnt3 *6000.0f/11/4/14;


		/* 6. 用脉冲增量更新里程计（积分得 x/y/theta） */
		odom_update(current_tim_cnt[0],-current_tim_cnt[1],-current_tim_cnt[2],-current_tim_cnt3);

		/* 7. 对四个轮子的转速分别做滤波，结果写入 E_motor.speedX 给上层用
		 *    参数：FilterNum=10  RemoveNum=2  MaxAmplitude=500  MaxSlope=30
		 *    含义：10 点窗口、去除最大2+最小2 取均值；
		 *          幅值超过 500rpm 视为异常；单步变化超过 30rpm 视为打滑/抖动
		 */
		Encoder_Filter(&EncoAspeed_Filter,E_motor_F.speedA,10,2,500,30,&E_motor.speedA);
		Encoder_Filter(&EncoBspeed_Filter,E_motor_F.speedB,10,2,500,30,&E_motor.speedB);
		Encoder_Filter(&EncoCspeed_Filter,E_motor_F.speedC,10,2,500,30,&E_motor.speedC);
		Encoder_Filter(&EncoDspeed_Filter,E_motor_F.speedD,10,2,500,30,&E_motor.speedD);

		//angle +=(float)temp[0] *360.0f/11/4/14;	// 累计角度（调试用）


}

/* 复位里程计：把车的全局位姿清零（用于上电对齐起点） */
void Encoder_Clear(void)
{

		g_odom.x=0;
		g_odom.y=0;
		g_odom.theta=0;

}


/* ------------------------------------------------------------
 *  Encoder_Filter_Init() —— 滤波器初始化
 *  上电时调用一次，把缓冲区和状态量清零
 * ------------------------------------------------------------ */
void Encoder_Filter_Init(EncoderFilter_Struct *filter)
{
    if(filter == NULL) return;  // 空指针保护

    filter->Count = 0;			// 环形缓冲写入位置
    filter->Complete = 0;		// 0=尚未填满一轮，1=已填满（开始正式滤波）
    filter->Last_Data = 0.0f;	// 上一次原始数据
    filter->Last_Valid = 0.0f;	// 上一次"合法"输出（限幅后）
    // 清空历史缓冲
    memset(filter->Data_His, 0, sizeof(filter->Data_His));
}

/**
  * 功能：增强型环形缓冲去极值平均滤波 + 幅值限幅 + 斜率限幅
  *
  * filterStruct  ：滤波器结构体（保存历史数据和状态）
  * newDatas      ：本次新输入数据（原始转速）
  * FilterNum     ：滑动窗口大小（推荐 5~32）
  * RemoveNum     ：去除最大/最小各几个（去极值数量）
  * MaxAmplitude  ：幅值上限（超过则丢弃，沿用上次结果）
  * MaxSlope      ：斜率上限（单步变化上限，防止突变）
  * Out           ：滤波结果输出指针
  *
  * 三段式处理：① 排序去极值取均值  ② 幅值限幅  ③ 斜率限幅
  */
void Encoder_Filter(EncoderFilter_Struct *filterStruct, float newDatas, unsigned char FilterNum, unsigned char RemoveNum,float MaxAmplitude,float MaxSlope,float *Out)
{
	unsigned int i = 0, j = 0;
    float sum=0;
    float sorted_buf[32]={0};  // 临时排序缓冲，与 Data_His 同大小

    /* 0. 入参合法性检查 */
    if(filterStruct == NULL || Out == NULL) return;

	/* 0.1 安全钳位：窗口大小不能越界，去极值数不能 ≥ 一半 */
    if(FilterNum > 32)    FilterNum = 32;
    if(FilterNum < 3)     FilterNum = 3;
    if(RemoveNum >= FilterNum/2) RemoveNum = 1;

	/* 1. 把新数据写入环形缓冲，Count 满则回卷并标记 Complete */
    filterStruct->Data_His[filterStruct->Count++] = newDatas;
    if(filterStruct->Count >= FilterNum)
    {
        filterStruct->Count = 0;
        filterStruct->Complete = 1;  // 缓冲首次填满，之后每次都做完整滤波
    }

		/* 2. 缓冲未满（启动阶段）：直接透传，不滤波 */
    if(!filterStruct->Complete)
    {
        *Out = newDatas;
        filterStruct->Last_Data = newDatas;
        filterStruct->Last_Valid = newDatas;
        return;
    }

    /* ===================== 3. 把环形缓冲拷贝到线性 sorted_buf =====================
     * Count 此时指向"最旧"的位置，从这里往后绕一圈即得按时间顺序的数据
     * （顺序对均值无影响，但对其它扩展算法有意义）
     */
    unsigned char index = filterStruct->Count;
    for(i = 0; i < FilterNum; i++)
    {
        sorted_buf[i] = filterStruct->Data_His[index];
        index++;
        if(index >= FilterNum) index = 0;
    }

    /* 4. 冒泡排序（窗口很小，O(n^2) 足够） */
    for(i = 0; i < FilterNum-1; i++)
    {
        for(j = 0; j < FilterNum-1-i; j++)
        {
            if(sorted_buf[j] > sorted_buf[j+1])
            {
                float temp = sorted_buf[j];
                sorted_buf[j] = sorted_buf[j+1];
                sorted_buf[j+1] = temp;
            }
        }
    }

    /* ===================== 5. 去极值平均 =====================
     * 砍掉最小 RemoveNum 个 + 最大 RemoveNum 个，剩余取均值
     * 这是中位滤波 + 均值滤波的折中：抗冲击噪声 + 保持平滑
     */
    sum = 0;
    unsigned char valid_cnt = FilterNum - 2 * RemoveNum;
    for(i = RemoveNum; i < FilterNum - RemoveNum; i++)
    {
        sum += sorted_buf[i];
    }
    float avg = sum / valid_cnt;


    /* ==================== 6. 多级限幅（防异常 + 防突变） ==================== */
    float output = avg;

    /* 6.1 幅值限幅：均值绝对值过大 → 视作异常，沿用上次合法输出 */
    if(my_abs(avg) > MaxAmplitude)
    {
        output = filterStruct->Last_Valid;
    }
    /* 6.2 斜率限幅：单步变化太大（打滑/卡顿）→ 按 MaxSlope 钳制 */
    else if(my_abs(avg - filterStruct->Last_Valid) > MaxSlope)
    {
        if(avg > filterStruct->Last_Valid)
            output = filterStruct->Last_Valid + MaxSlope;
        else
            output = filterStruct->Last_Valid - MaxSlope;
    }

    /* 7. 关键：限幅后的 output 才是"合法"输出，要回写 Last_Valid
     *    否则下次斜率比较会用未限幅的 avg 当基准，导致限幅失效 */
    filterStruct->Last_Valid = output;
    filterStruct->Last_Data = output;
    *Out = output;
}


