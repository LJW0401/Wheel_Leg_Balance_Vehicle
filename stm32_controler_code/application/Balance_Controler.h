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

#include "struct_typedef.h"

//对于文件注释中关于关节模型的变量定义请参照小企鹅的Wheel_Leg_Balance_Vehicle项目中的五连杆几何学解算文档
/** @brief      一对关节的信息结构体
  * @note       对于文件注释中关于关节模型的变量定义请参照小企鹅的Wheel_Leg_Balance_Vehicle项目中的五连杆几何学解算文档
  */
typedef struct{
  float rod_length;//[单位:cm]等效连杆的长度，模型图中的l
  float rod_angle;//[单位:radian]等效连杆的偏转角，模型图中的\theta
  float length_0;//[单位:cm]两关节电机的电机轴间距，模型图中的l_0
  float length_1;//[单位:cm]与前方电机直接连接的连杆的长度，模型图中的l_1
  float length_2;//[单位:cm]与后方电机直接连接的连杆的长度，模型图中的l_2
  float length_3;//[单位:cm]与前方电机直间连接的连杆的长度，模型图中的l_3
  float length_4;//[单位:cm]与后方电机间接连接的连杆的长度，模型图中的l_4
  float front_angle;//[单位:radian]放置于前方的电机的摆动角度，模型图中的\theta_1
  float back_angle; //[单位:radian]放置于后方方的电机的摆动角度，模型图中的\theta_2
}Joint_Pair_t;

//2边对应的一对关节的信息，控制端只需要调用这里的数据即可
Joint_Pair_t left_joint_pair;//左侧关节电机的摆动信息
Joint_Pair_t right_joint_pair;//右侧关节电机的摆动信息

extern void CalculateJointMotorAnglePair(Joint_Pair_t* joint_motor_info_pair);
extern void CalculateEquivalentRodInfo(Joint_Pair_t* joint_pair_info);
