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
//导入轮腿模型
#include "leg_model/leg_conv.h"
#include "leg_model/leg_pos.h"
#include "leg_model/leg_spd.h"
#include "leg_model/lqr_k.h"


/** @brief      底盘IMU数据结构体
  * @note       
  */
typedef struct 
{
  float yaw, pitch, roll; // rad
  float yawSpd, pitchSpd, rollSpd; // rad/s
  float zAccel; // m/s^2
}Chassis_IMU_t;
Chassis_IMU_t chassis_imu;

/** @brief      电机结构体
  * @note       leftJoint[0]:左前关节电机, leftJoint[1]:左后关节电机, leftWheel:左车轮电机
  *             rightJoint[0]:右前关节电机, rightJoint[1]:右后关节电机, rightWheel:右车轮电机
  */
typedef struct 
{
	float speed;			   // rad/s
	float angle, offsetAngle;  // rad
	float voltage, maxVoltage; // V
	float torque, torqueRatio; // Nm, voltage = torque / torqueRatio
	float dir;				   // 1 or -1
	float (*calcRevVolt)(float speed); // 指向反电动势计算函数
}Motor_s;
Motor_s leftJoint[2], rightJoint[2], leftWheel, rightWheel; //六个电机对象


/** @brief      关节长度结构体
  * @note       无
  */
typedef struct
{
  float l1, l2, l3, l4, l5; // m
}Joint_Length_t;
Joint_Length_t leftJointLength = {0.05f, 0.105f, 0.105f, 0.05f, 0.06f}; //关节长度
Joint_Length_t rightJointLength = {0.05f, 0.105f, 0.105f, 0.05f, 0.06f}; //关节长度
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
Leg_Pos_t leftLegPos, rightLegPos; //左右腿部姿态


/** @brief      状态变量结构体
  * @note       无
  */
typedef struct 
{
  float theta, dTheta;
  float x, dx;
  float phi, dPhi;
}State_Var_s; 
State_Var_s stateVar;


/** @brief      目标量结构体
  * @note       无
  */
typedef struct 
{
  float position;	 // m
  float speedCmd;	 // m/s
  float speed;    // m/s
  float yawSpeedCmd; // rad/s
  float yawAngle;	 // rad
  float rollAngle; // rad
  float legLength; // m
} Target_s;
Target_s target = {0, 0, 0, 0, 0, 0, 0.07f};


/** @brief      触地检测数据结构体
  * @note       无
  */
typedef struct 
{
  float leftSupportForce, rightSupportForce;
  uint8_t isTouchingGround, isCuchioning;
} GroundDetector;
GroundDetector groundDetector = {10, 10, 1, 0};


/** @brief      站立过程状态枚举量
  * @note       无
  */
enum StandupState {
  StandupState_None,
  StandupState_Prepare,
  StandupState_Standup,
} standupState = StandupState_None;






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
