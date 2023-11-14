#ifndef EKF_H
#define EKF_H

typedef struct
{
    float LastP; // 上次估算协方差
    float out;   // 卡尔曼滤波器输出
    float Q;     // 过程噪声协方差
    float R;     // 观测噪声协方差
} EKF_s;         // Kalman Filter parameter

void InitEKF(EKF_s *ekf, float Q, float R);

float ExtendedKalmanFilterCalc(EKF_s *ekf, float input1, float input2);

#endif // EKF_H