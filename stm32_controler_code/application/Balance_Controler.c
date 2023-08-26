/**
  ****************************(C) COPYRIGHT 2023 POLARBEAR****************************
  * @file       Balance_Controler.c/h
  * @brief      北极熊的平衡车车底盘控制器
  * @note       

*********  *********  *              *      *********  ********   *********      *     *********
*       *  *       *  *             * *     *       *  *       *  *             * *    *       *
*       *  *       *  *            *   *    *       *  *       *  *            *   *   *       *
*       *  *       *  *           *     *   *       *  *       *  *           *     *  *       *
*********  *       *  *          *********  *********  ********   *********  ********* *********
*          *       *  *          *       *  * *        *       *  *          *       * * *
*          *       *  *          *       *  *   *      *       *  *          *       * *   *
*          *       *  *          *       *  *     *    *       *  *          *       * *     *
*          *********  *********  *       *  *       *  ********   *********  *       * *       *


(9*9)

  ****************************(C) COPYRIGHT 2023 POLARBEAR****************************
  */

#include "Balance_Controler.h"
#include <math.h>

/**
  * @brief          初始化一个电机对象
  * @attention      
  * @note           
  * @author         小企鹅
  * @param          motor 电机结构体地址
  * @param          offsetAngle 
  * @param          maxVoltage 
  * @param          torqueRatio 
  * @param          dir 
  * @param          calcRevVolt 反电动势计算函数
  */
void Motor_Init(Motor_s *motor, float offsetAngle, float maxVoltage, float torqueRatio, float dir, float (*calcRevVolt)(float speed))
{
	motor->speed = motor->angle = motor->voltage = 0;
	motor->offsetAngle = offsetAngle;
	motor->maxVoltage = maxVoltage;
	motor->torqueRatio = torqueRatio;
	motor->dir = dir;
	motor->calcRevVolt = calcRevVolt;
}




/**以下先作为一个储备吧，
  *本来想尝试跳过力矩计算的纯PID控制器的，
  *一半的时候发现计算结果不太支持，
  *就先搁置了，先回归到VMC上
  *还好有@遥想星空 的开源项目可以借鉴
  */

// /**
//   * @brief          初始化关节信息
//   * @attention      长度单位统一使用cm，角度单位统一使用radian
//   * @note           将关节结构体的参数初始化，值请参照自己的轮腿车车的相关参数，具体参数含义请参照小企鹅的Wheel_Leg_Balance_Vehicle项目中的五连杆几何学解算文档
//   * @author         小企鹅
//   * @param[in]      joint_info 关节的信息
//   */
// void InitJointPairInfo(Joint_t* joint_info,
//               float l_0, float l_1, float l_2, float l_3, float l_4,
//               float rod_angle, float rod_length)
// {
//   joint_info->length_0 = l_0;
//   joint_info->length_1 = l_1;
//   joint_info->length_2 = l_2;
//   joint_info->length_3 = l_3;
//   joint_info->length_4 = l_4;
//   joint_info->rod_angle = rod_angle;
//   joint_info->rod_length = rod_length;
//   CalculateJointMotorAnglePair(joint_info);
// }


// /**
//   * @brief          根据等效的直杆长度及摆动角度及关节长度的信息，计算出关节电机所需摆动的角度信息
//   * @attention      无
//   * @note           五连杆逆几何学解算，具体分析请参照小企鹅的Wheel_Leg_Balance_Vehicle项目中的五连杆几何学解算文档
//   * @author         小企鹅
//   * @param[in]      joint_info 关节的信息
//   */
// void CalculateJointMotorAnglePair(Joint_t* joint_info)
// {
//   float l = joint_info->rod_length;
//   float theta = joint_info->rod_angle; 
//   float l_0 = joint_info->length_0;
//   float l_1 = joint_info->length_1;
//   float l_2 = joint_info->length_2;
//   float l_3 = joint_info->length_3;
//   float l_4 = joint_info->length_4;
//   // float l_6_pow = pow(joint_info->rod_length,2) + pow(joint_info->length_0/2,2) - joint_info->rod_length*joint_info->length_0*sin(joint_info->rod_angle);
//   float l_6_pow = powf(l,2)+powf(l_0/2,2)-l*l_0*sinf(theta);
//   float theta_11 = acosf((powf(l,2)-powf(l_0/2,2)-l_6_pow)/(l_0*sqrtf(l_6_pow)));
//   float theta_12 = acosf((powf(l_3,2)-powf(l_1/2,2)-l_6_pow)/(2*l_1*sqrtf(l_6_pow)));
//   joint_info->front_angle = theta_11+theta_12;

//   float l_7_pow = powf(l,2)+powf(l_0/2,2)+l*l_0*sinf(theta);
//   float theta_21 = acosf((powf(l,2)-powf(l_0/2,2)-l_7_pow)/(l_0*sqrtf(l_7_pow)));
//   float theta_22 = acosf((powf(l_4,2)-powf(l_2/2,2)-l_7_pow)/(2*l_2*sqrtf(l_7_pow)));
//   joint_info->back_angle = theta_21+theta_22;

// }


// /**
//   * @brief          根据关节电机摆角信息及各连杆的长度计算得到五连杆等效直杆的长度和摆角信息
//   * @attention      无
//   * @note           五连杆正几何学解算，具体分析请参照小企鹅的Wheel_Leg_Balance_Vehicle项目中的五连杆几何学解算文档
//   * @author         小企鹅
//   * @param[in]      joint_info 关节的信息
//   */
// void CalculateEquivalentRodInfo(Joint_t* joint_info)
// {
//   float l_0 = joint_info->length_0;
//   float l_1 = joint_info->length_1;
//   float l_2 = joint_info->length_2;
//   float l_3 = joint_info->length_3;
//   float l_4 = joint_info->length_4;
//   float theta_1 = joint_info->front_angle;
//   float theta_2 = joint_info->back_angle;
  
//   float l_5_pow = pow(l_1-l_1*cosf(theta_1)-l_2*cosf(theta_2),2);
//   float theta_3 = acosf((powf(l_4,2)-l_5_pow-powf(l_3,2))/(2*l_3*sqrtf(l_5_pow)));
//   float theta_4 = atanf((l_2*sinf(theta_2)-l_1*sinf(theta_1))/(l_1-l_1*cosf(theta_1)-l_2*cosf(theta_2)));

//   joint_info->rod_length = 
//     sqrtf(
//       pow(l_0/2-l_1*cosf(theta_1)-l_3*cosf(theta_3+theta_4),2)+
//       pow(l_1*sinf(theta_1)+l_3*sinf(theta_3+theta_4),2)
//     );
//   joint_info->rod_angle =
//     atanf(
//       (l_0/2-l_1*cosf(theta_1)+l_3*cosf(theta_3+theta_4))/
//       (l_1*sinf(theta_1)+l_3*sinf(theta_3+theta_4))
//     );
// }


// /**
//   * @todo           目前先实现在平地上的起立、前后运动，后续再实现转弯等功能
//   * 
//   * @brief          根据底盘IMU数据动态调节两侧关节状态
//   * @attention      无
//   * @note           底盘平衡控制器
//   * @author         小企鹅
//   */
// void BalanceControler()
// {
// //把pitch偏转的量映射到直杆的偏转，产生对应力矩，使得车身保持平衡
// }


// /**
//   * @brief          为防止IMU安装位置导致的方向错位，需校准到正确的方向
//   * @attention      最终结果为当前量加上校准量
//   * @note           IMU位置校准
//   * @author         小企鹅
//   * @param[in]      yaw yaw轴校准量
//   * @param[in]      pitch pitch轴校准量
//   * @param[in]      roll roll轴校准量
//   */
// void ChassisIMUCalibration(float yaw, float pitch, float roll)
// {
//   chassis_imu.yaw += yaw;
//   chassis_imu.pitch += pitch;
//   chassis_imu.roll += roll;
// }