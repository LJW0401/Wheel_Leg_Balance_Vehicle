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
#ifndef BALANCE_CONTROLER_H
#define BALANCE_CONTROLER_H

#include "struct_typedef.h"
//导入轮腿模型
#include "./leg_model/JointPos.h"
#include "./leg_model/LegConv.h"
#include "./leg_model/LegPos.h"
#include "./leg_model/LegSpd.h"
#include "./leg_model/LQR_K.h"
#include "./leg_model/PID.h"
#include "./Drives/MI_motor_drive.h"


/* Useful constants.  */
#define MAXFLOAT	3.40282347e+38F
#define M_E		2.7182818284590452354
#define M_LOG2E		1.4426950408889634074
#define M_LOG10E	0.43429448190325182765
#define M_LN2		_M_LN2
#define M_LN10		2.30258509299404568402
#define M_PI		3.14159265358979323846
#define M_TWOPI         (M_PI * 2.0)
#define M_PI_2		1.57079632679489661923
#define M_PI_4		0.78539816339744830962
#define M_3PI_4		2.3561944901923448370E0
#define M_SQRTPI        1.77245385090551602792981
#define M_1_PI		0.31830988618379067154
#define M_2_PI		0.63661977236758134308
#define M_2_SQRTPI	1.12837916709551257390
#define M_SQRT2		1.41421356237309504880
#define M_SQRT1_2	0.70710678118654752440
#define M_LN2LO         1.9082149292705877000E-10
#define M_LN2HI         6.9314718036912381649E-1
#define M_SQRT3	1.73205080756887719000
#define M_IVLN10        0.43429448190325182765 /* 1 / log(10) */
#define M_LOG2_E        _M_LN2
#define M_INVLN2        1.4426950408889633870E0  /* 1 / log(2) */

#define REDUCTION_RATIO_2006 0.027777777777777776 /*2006减速比(1:36)*/

/** @brief      底盘IMU数据结构体
  * @note       
  */
typedef struct 
{
  float yaw, pitch, roll; // rad
  float yawSpd, pitchSpd, rollSpd; // rad/s
  float zAccel; // m/s^2
}Chassis_IMU_t;


/** @brief      电机结构体
  * @note       leftJoint[0]:左前关节电机, leftJoint[1]:左后关节电机, leftWheel:左车轮电机
  *             rightJoint[0]:右前关节电机, rightJoint[1]:右后关节电机, rightWheel:右车轮电机
  */
typedef struct 
{
  MI_Motor_s* MI_Motor;  // 小米电机对象
	float speed;           // rad/s
	float angle;           // rad 关节与机体水平正方向的夹角
  float offset_angle;    // rad 
  float initial_angle;   // rad 初始状态下电机反馈的角度
  float vertical_angle;  // rad 关节朝向机体竖直正方向的反馈角度
  float horizontal_angle;// rad 关节朝向机体水平正方向的反馈角度
  float target_angle;    // rad 关节与机体水平正方向的目标夹角
	float upper_limit_angle,lower_limit_angle;// rad
  float voltage, max_voltage; // V
	float torque, torque_ratio; // Nm, voltage = torque / torque_ratio
	float dir;				   // 1 or -1
	float (*calcRevVolt)(float speed); // 指向反电动势计算函数
}Motor_s;


/** @brief      关节长度结构体
  * @note       无
  */
typedef struct
{
  float l1, l2, l3, l4, l5; // m
}Joint_Length_t;
// l1=0.05;l2=0.105;l3=l2;l4=l1;l5=0.06; % @遥想星空 的腿部杆长

/** @brief      腿部姿态结构体
  * @note       无
  */
typedef struct 
{
  float angle, length;   // rad, m
  float dAngle, dLength; // rad/s, m/s
  float ddLength;       // m/s^2
}Leg_Pos_t;


/** @brief      腿部姿态目标量
  * @note       无
  */
typedef struct 
{
  float angle, length;   // rad, m
  float dAngle, dLength; // rad/s, m/s
  float ddLength;       // m/s^2
} Leg_Pos_Target_t;


/** @brief      状态变量结构体
  * @note       无
  */
typedef struct 
{
  float theta, dTheta;
  float x, dx;
  float phi, dPhi;
}State_Var_s; 


/** @brief      目标量结构体
  * @note       无
  */
typedef struct 
{
  float position;	 // m
  float speed_cmd;	 // m/s
  float speed;    // m/s
  float yaw_speed_cmd; // rad/s
  float yaw_angle;	 // rad
  float roll_angle; // rad
  float leg_length; // m
} Target_s;


/** @brief      触地检测数据结构体
  * @note       无
  */
typedef struct 
{
  float left_support_force, right_support_force;
  uint8_t is_touching_ground, is_cuchioning;
} GroundDetector;


/** @brief      站立过程状态枚举量
  * @note       无
  */
typedef enum{
  StandupState_None,
  StandupState_Prepare,
  StandupState_Standup,
}StandupState;


extern MI_Motor_s MI_Motor[5];
extern MI_Motor_s MI_Motor_None;

extern Chassis_IMU_t chassis_imu;
extern Motor_s left_joint[2], right_joint[2], left_wheel, right_wheel;
extern Leg_Pos_t left_leg_pos, right_leg_pos;
extern Leg_Pos_Target_t left_leg_pos_target, right_leg_pos_target;
extern State_Var_s state_var;
extern Target_s target;
extern GroundDetector ground_detector;
extern CascadePID leg_angle_PID, leg_length_PID;
extern CascadePID yaw_PID, roll_PID;

extern StandupState standup_state;


extern void MotorInitAll();
extern void MotorSetTorque(Motor_s *motor, float torque);
extern void MotorSetTargetAngle(Motor_s *motor, float target_angle);

extern void ChassisPostureUpdate();
extern void CtrlTargetUpdateTask();
extern void LegPosUpdateTask();

extern int16_t MotorTorqueToCurrentValue_2006(float torque);

extern void nop_delay_us(uint16_t us);

#endif


/**以下先作为一个储备吧，
  *本来想尝试跳过力矩计算的纯PID控制器的，
  *一半的时候发现计算结果不太支持，
  *就先搁置了，先回归到VMC上
  *还好有@遥想星空 的开源项目可以借鉴
  */

// //对于文件注释中关于关节模型的变量定义请参照小企鹅的Wheel_Leg_Balance_Vehicle项目中的五连杆几何学解算文档
// /** @brief      一对关节的信息结构体
//   * @note       对于文件注释中关于关节模型的变量定义请参照小企鹅的Wheel_Leg_Balance_Vehicle项目中的五连杆几何学解算文档
//   */
// typedef struct{
//   float rod_length;//[单位:cm]等效连杆的长度，模型图中的l
//   float rod_angle;//[单位:radian]等效连杆的偏转角，模型图中的\theta
//   float length_0;//[单位:cm]两关节电机的电机轴间距，模型图中的l_0
//   float length_1;//[单位:cm]与前方电机直接连接的连杆的长度，模型图中的l_1
//   float length_2;//[单位:cm]与后方电机直接连接的连杆的长度，模型图中的l_2
//   float length_3;//[单位:cm]与前方电机直间连接的连杆的长度，模型图中的l_3
//   float length_4;//[单位:cm]与后方电机间接连接的连杆的长度，模型图中的l_4
//   float front_angle;//[单位:radian]放置于前方的电机的摆动角度，模型图中的\theta_1
//   float back_angle; //[单位:radian]放置于后方方的电机的摆动角度，模型图中的\theta_2
// }Joint_t;


// /** @brief      底盘IMU数据结构体
//   * @note       获取惯性测量单元的数据，配合ChassisIMUCalibration函数使用，先读取原始数据，再通过ChassisIMUCalibration校准方向后获取底盘的姿态信息。
//   */
// typedef struct 
// {
//   float yaw;//[单位:radian]偏航角
//   float pitch;//[单位:radian]俯仰角
//   float roll;//[单位:radian]翻滚角
//   float ax;
//   float ay;
//   float az;
// }Chassis_IMU_t;


// //2边对应的一对关节的信息，控制端只需要调用这里的数据即可
// Joint_t left_joint_pair;//左侧关节电机的摆动信息
// Joint_t right_joint_pair;//右侧关节电机的摆动信息
// //底盘IMU数据
// Chassis_IMU_t chassis_imu;//底盘IMU数据


// extern void InitJointPairInfo(Joint_t* joint_info,
//               float l_0, float l_1, float l_2, float l_3, float l_4,
//               float rod_angle, float rod_length);
// extern void CalculateJointMotorAnglePair(Joint_t* joint_motor_info_pair);
// extern void CalculateEquivalentRodInfo(Joint_t* joint_pair_info);
// extern void ChassisIMUCalibration(float yaw, float pitch, float roll);
