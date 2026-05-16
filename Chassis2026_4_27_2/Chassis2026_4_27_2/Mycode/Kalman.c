#include "Kalman.h"

/* 初始化卡尔曼滤波器 */
void KalmanFilter_init(KalmanFilter *kf, float Q, float R) 
{
		/* 开始时可以将 ( Q ) 设置得较小，( R ) 设置得较大，
				然后逐步增加 ( Q )、减小 ( R )，观察滤波器的响应速度和稳定性。
	
				调整Q的大小可以影响滤波器对系统动态变化的敏感度；
				较大的Q表示更大的不确定性，系统会更快地适应变化。
				在高噪声环境中，可能需要增加  R 以降低对测量的信任度。
		 */
    kf->x = 0;
    kf->P = 1;
	
		/* Q 大，滤波器对模型的信任度降低，会更多地依赖观测数据 */
    kf->Q = Q;
		/* R 大，滤波器对测量数据的依赖减少 */
    kf->R = R;
}

/* 卡尔曼滤波更新步骤 */
float KalmanFilter_update(KalmanFilter *kf, float z) 
{
    // 预测步骤
    float x_pred = kf->x;
    float P_pred = kf->P + kf->Q;

    // 更新步骤
    float K = P_pred / (P_pred + kf->R);
    kf->x = x_pred + K * (z - x_pred);
    kf->P = (1 - K) * P_pred;

    return kf->x;
}

