/**
  ****************************(C) COPYRIGHT 2023 POLARBEAR****************************
  * @file       EKF.c/h
  * @brief      扩展卡尔曼滤波器
  * @note
  * @history
  *  Version    Date         Author        Modification
  *  V1.0.0     2023-11-15   Penguin       1. 完成
  *
  @verbatim
  ==============================================================================
    如何使用：
    1.
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2023 POLARBEAR****************************
  */

#include "EKF.h"

/**
 * @brief      初始化卡尔曼滤波器
 * @param[in]  kf 卡尔曼结构体参数
 * @param[in]  Q 噪声协方差矩阵
 * @param[in]  R 测量噪声协方差矩阵
 * @return     none
 */
void InitKF(KF_s *kf, float Q, float R)
{
    kf->LastP = 0;
    kf->Now_P = 0;
    kf->out = 0;
    kf->Kg = 0;
    kf->Q = Q;
    kf->R = R;
}

/**
 *@brief       卡尔曼滤波器
 *@param[in]   KFP 卡尔曼结构体参数
 *@param[in]   input 需要滤波的参数的测量值（即传感器的采集值）
 *@return 滤波后的参数（最优值）
 */
float KalmanFilterCalc(KF_s *kfp, float input)
{
    // 预测协方差方程：k时刻系统估算协方差 = k-1时刻的系统协方差 + 过程噪声协方差
    kfp->Now_P = kfp->LastP + kfp->Q;
    // 卡尔曼增益方程：卡尔曼增益 = k时刻系统估算协方差 / （k时刻系统估算协方差 + 观测噪声协方差）
    kfp->Kg = kfp->Now_P / (kfp->Now_P + kfp->R);
    // 更新最优值方程：k时刻状态变量的最优值 = 状态变量的预测值 + 卡尔曼增益 * （测量值 - 状态变量的预测值）
    kfp->out = kfp->out + kfp->Kg * (input - kfp->out); // 因为这一次的预测值就是上一次的输出值
    // 更新协方差方程: 本次的系统协方差付给 kfp->LastP 威下一次运算准备。
    kfp->LastP = (1 - kfp->Kg) * kfp->Now_P;
    return kfp->out;
}

/**
 * @brief      初始化扩展卡尔曼滤波器
 * @param[in]  ekf *ekf 卡尔曼结构体参数
 * @param[in]  Q 噪声协方差矩阵
 * @param[in]  R 测量噪声协方差矩阵
 * @return     none
 */
void InitEKF(EKF_s *ekf, float Q, float R)
{
    ekf->LastP = 0;
    ekf->out = 0;
    ekf->Q = Q;
    ekf->R = R;
}


/**
 * @brief      扩展卡尔曼滤波器
 * @param[in]  ekf *ekf 卡尔曼结构体参数
 * @param[in]  float input1 需要滤波的参数1的测量值（即传感器的采集值）
 * @param[in]  float input2 需要滤波的参数2的测量值（即传感器的采集值）
 * @return 滤波后的参数（最优值）
 */
float ExtendedKalmanFilterCalc(EKF_s *ekf, float input1, float input2)
{
    // 预测状态方程：x(k) = f(x(k-1), u(k))
    float x_predict = ekf->out + input1 * cos(ekf->out) + input2 * sin(ekf->out);
    // 预测协方差方程：P(k) = F(k) * P(k-1) * F(k)^T + Q(k)
    float p_predict = ekf->LastP + ekf->Q;
    // 计算雅可比矩阵
    float jacobian[2][2] = {{1 - input1 * sin(ekf->out), cos(ekf->out)}, {input1 * cos(ekf->out), sin(ekf->out)}};
    // 计算卡尔曼增益
    float k_gain[2] = {p_predict * jacobian[0][0] / (p_predict * jacobian[0][0] * jacobian[0][0] + p_predict * jacobian[1][0] * jacobian[1][0] + ekf->R),
                       p_predict * jacobian[1][0] / (p_predict * jacobian[0][0] * jacobian[0][0] + p_predict * jacobian[1][0] * jacobian[1][0] + ekf->R)};
    // 更新状态方程：x(k) = x(k) + K(k) * [z(k) - h(x(k))]
    ekf->out = x_predict + k_gain[0] * (input1 - x_predict * cos(ekf->out)) + k_gain[1] * (input2 - x_predict * sin(ekf->out));
    // 更新协方差方程：P(k) = (I - K(k) * H(k)) * P(k)
    ekf->LastP = (1 - k_gain[0] * jacobian[0][0]) * p_predict;
    return ekf->out;
}
