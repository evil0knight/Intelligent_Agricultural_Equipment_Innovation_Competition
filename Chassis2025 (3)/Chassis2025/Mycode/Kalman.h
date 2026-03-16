#ifndef __KALMAN_H__
#define __KALMAN_H__


#include "stm32f4xx_hal.h"

/* 卡尔曼滤波结构体 */
typedef struct 
{
    float x;  // 状态估计
    float P;  // 状态协方差
    float Q;  // 状态过程噪声方差
    float R; 	// 观测噪声方差
} KalmanFilter;

void KalmanFilter_init(KalmanFilter *kf, float Q, float R);
float KalmanFilter_update(KalmanFilter *kf, float z);

#endif
