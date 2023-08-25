/**
  ****************************(C) COPYRIGHT 2023 POLARBEAR****************************
  * @file       Balance_Controler.c/h
  * @brief      北极熊的平衡车车底盘控制器
  * @note       
  * @todo       完成关节结构体初始化函数

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
  * @brief          根据等效的直杆长度及摆动角度及关节长度的信息，计算出关节电机所需摆动的角度信息
  * @attention      无
  * @note           五连杆逆几何学解算，具体分析请参照小企鹅的Wheel_Leg_Balance_Vehicle项目中的五连杆几何学解算文档
  * @author         小企鹅
  * @param[in]      joint_pair_info [Joint_Pair_t*]一对关节的信息
  */
void CalculateJointMotorAnglePair(Joint_Pair_t* joint_pair_info)
{
  float l = joint_pair_info->rod_length;
  float theta = joint_pair_info->rod_angle; 
  float l_0 = joint_pair_info->length_0;
  float l_1 = joint_pair_info->length_1;
  float l_2 = joint_pair_info->length_2;
  float l_3 = joint_pair_info->length_3;
  float l_4 = joint_pair_info->length_4;
  // float l_6_pow = pow(joint_pair_info->rod_length,2) + pow(joint_pair_info->length_0/2,2) - joint_pair_info->rod_length*joint_pair_info->length_0*sin(joint_pair_info->rod_angle);
  float l_6_pow = powf(l,2)+powf(l_0/2,2)-l*l_0*sinf(theta);
  float theta_11 = acosf((powf(l,2)-powf(l_0/2,2)-l_6_pow)/(l_0*sqrtf(l_6_pow)));
  float theta_12 = acosf((powf(l_3,2)-powf(l_1/2,2)-l_6_pow)/(2*l_1*sqrtf(l_6_pow)));
  joint_pair_info->front_angle = theta_11+theta_12;

  float l_7_pow = powf(l,2)+powf(l_0/2,2)+l*l_0*sinf(theta);
  float theta_21 = acosf((powf(l,2)-powf(l_0/2,2)-l_7_pow)/(l_0*sqrtf(l_7_pow)));
  float theta_22 = acosf((powf(l_4,2)-powf(l_2/2,2)-l_7_pow)/(2*l_2*sqrtf(l_7_pow)));
  joint_pair_info->back_angle = theta_21+theta_22;

}

/**
  * @brief          根据关节电机摆角信息及各连杆的长度计算得到五连杆等效直杆的长度和摆角信息
  * @attention      无
  * @note           五连杆正几何学解算，具体分析请参照小企鹅的Wheel_Leg_Balance_Vehicle项目中的五连杆几何学解算文档
  * @author         小企鹅
  * @param[in]      joint_pair_info [Joint_Pair_t*]关节电机摆动的角度信息
  */
void CalculateEquivalentRodInfo(Joint_Pair_t* joint_pair_info)
{
  float l_0 = joint_pair_info->length_0;
  float l_1 = joint_pair_info->length_1;
  float l_2 = joint_pair_info->length_2;
  float l_3 = joint_pair_info->length_3;
  float l_4 = joint_pair_info->length_4;
  float theta_1 = joint_pair_info->front_angle;
  float theta_2 = joint_pair_info->back_angle;
  
  float l_5_pow = pow(l_1-l_1*cosf(theta_1)-l_2*cosf(theta_2),2);
  float theta_3 = acosf((powf(l_4,2)-l_5_pow-powf(l_3,2))/(2*l_3*sqrtf(l_5_pow)));
  float theta_4 = atanf((l_2*sinf(theta_2)-l_1*sinf(theta_1))/(l_1-l_1*cosf(theta_1)-l_2*cosf(theta_2)));

  joint_pair_info->rod_length = 
    sqrtf(
      pow(l_0/2-l_1*cosf(theta_1)-l_3*cosf(theta_3+theta_4),2)+
      pow(l_1*sinf(theta_1)+l_3*sinf(theta_3+theta_4),2)
    );
  joint_pair_info->rod_angle =
    atanf(
      (l_0/2-l_1*cosf(theta_1)+l_3*cosf(theta_3+theta_4))/
      (l_1*sinf(theta_1)+l_3*sinf(theta_3+theta_4))
    );
}