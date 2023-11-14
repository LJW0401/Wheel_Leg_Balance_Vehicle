#ifndef EKF_H
#define EKF_H

typedef struct
{
    float LastP; // 上次估算协方差
    float out;   // 卡尔曼滤波器输出
    float Q;     // 过程噪声协方差
    float R;     // 观测噪声协方差
} EKF_s;           // Kalman Filter parameter

// typedef struct 
// {
//     float LastP;//上次估算协方差 初始化值为0.02
//     float Now_P;//当前估算协方差 初始化值为0
//     float out;//卡尔曼滤波器输出 初始化值为0
//     float Kg;//卡尔曼增益 初始化值为0
//     float Q;//过程噪声协方差 初始化值为0.001
//     float R;//观测噪声协方差 初始化值为0.543
// }KFP;//Kalman Filter parameter

#endif // EKF_H