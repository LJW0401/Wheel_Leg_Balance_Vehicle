/**
  ****************************(C) COPYRIGHT 2023 POLARBEAR****************************
  * @file       Balance_Controler.c/h
  * @brief      北极熊的平衡车车底盘控制器
  * @note
  * @history
  *  Version    Date         Author        Modification
  *  V1.0.0     2023-11-7    Penguin       1. 完成
  *  V1.0.1     2023-11-8    Penguin       1. 添加Pitch角PD控制
  *
  @verbatim
  ==============================================================================
    本控制器底盘控制器以小米电机作为关节电机和GM2006电机作为驱动轮电机，使用CAN通信，
    如果使用其他型号的电机的话请修改本控制器的信号发送部分和控制方式。

    如何使用本控制器操控平衡底盘：
    1. 通过调用BalanceControlerInit()初始化控制器。
    2. 通过调用BalanceControlerCalc()计算控制器的控制量。
    3. 通过ControlBalanceChassis()控制底盘电机。
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2023 POLARBEAR****************************
  */

#ifndef BALANCE_CONTROLER_H
#define BALANCE_CONTROLER_H

#include "struct_typedef.h"
#include "./Drives/MI_motor_drive.h"
// 导入轮腿模型
#include "./leg_model/JointPos.h"
#include "./leg_model/LegConv.h"
#include "./leg_model/LegPos.h"
#include "./leg_model/LegSpd.h"
#include "./leg_model/LQR_K.h"
#include "./leg_model/PID.h"

/* Useful constants.  */
#define MAXFLOAT 3.40282347e+38F
#define M_E 2.7182818284590452354
#define M_LOG2E 1.4426950408889634074
#define M_LOG10E 0.43429448190325182765
#define M_LN2 _M_LN2
#define M_LN10 2.30258509299404568402
#define M_PI 3.14159265358979323846 // PI
#define M_TWOPI (M_PI * 2.0)
#define M_PI_2 1.57079632679489661923   // PI/2
#define M_PI_4 0.78539816339744830962   // PI/4
#define M_3PI_4 2.3561944901923448370E0 // PI*3/4
#define M_SQRTPI 1.77245385090551602792981
#define M_1_PI 0.31830988618379067154 // 0.1 PI
#define M_2_PI 0.63661977236758134308 // 0.2 PI
#define M_2_SQRTPI 1.12837916709551257390
#define M_SQRT2 1.41421356237309504880
#define M_SQRT1_2 0.70710678118654752440
#define M_LN2LO 1.9082149292705877000E-10
#define M_LN2HI 6.9314718036912381649E-1
#define M_SQRT3 1.73205080756887719000
#define M_IVLN10 0.43429448190325182765 /* 1 / log(10) */
#define M_LOG2_E _M_LN2
#define M_INVLN2 1.4426950408889633870E0 /* 1 / log(2) */

#define REDUCTION_RATIO_2006 0.027777777777777776 // 2006减速比(1:36)
#define K_2006 0.18518518518518517                // 2006电机转速系数
#define START_TORQUE_2006 0.025                   // 2006电机起动力矩
#define WHEEL_RADIUS 0.0425                       // m，车轮半径
#define LEG_MASS 0.12368                          // kg，腿部质量

#define WHEEL_CAN hcan2

/**
 * @brief      底盘IMU数据结构体
 * @note
 */
typedef struct
{
    float yaw, pitch, roll;          // rad
    float yawSpd, pitchSpd, rollSpd; // rad/s
    float xAccel, yAccel, zAccel;    // m/s^2
} Chassis_IMU_t;

/**
 * @brief      电机结构体
 * @note       leftJoint[0]:左前关节电机, leftJoint[1]:左后关节电机, leftWheel:左车轮电机
 *             rightJoint[0]:右前关节电机, rightJoint[1]:右后关节电机, rightWheel:右车轮电机
 */
typedef struct
{
    MI_Motor_s *MI_Motor;                       // 小米电机对象
    float speed;                                // rad/s
    float angle;                                // rad 关节与机体水平正方向的夹角
    float offset_angle;                         // rad
    float initial_angle;                        // rad 初始状态下电机反馈的角度
    float vertical_angle;                       // rad 关节朝向机体竖直正方向的反馈角度
    float horizontal_angle;                     // rad 关节朝向机体水平正方向的反馈角度
    float target_angle;                         // rad 关节与机体水平正方向的目标夹角
    float upper_limit_angle, lower_limit_angle; // rad
    float voltage, max_voltage;                 // V
    float torque, torque_ratio;                 // Nm, voltage = torque / torque_ratio
    float dir;                                  // 1 or -1
    float rx_torque;                            // Nm 反馈力矩
} Motor_s;

/**
 * @brief      腿部姿态结构体
 * @note       无
 */
typedef struct
{
    float angle, length;   // rad, m
    float dAngle, dLength; // rad/s, m/s
    float ddLength;        // m/s^2
} Leg_Pos_t;

/**
 * @brief      状态变量结构体
 * @note       无
 */
typedef struct
{
    float theta, dTheta;
    float x, dx;
    float phi, dPhi;
    float leg_length, dLegLength;
} State_Var_s;

/**
 * @brief      目标量结构体
 * @note       无
 */
typedef struct
{
    float position;                  // m
    float speed_cmd;                 // m/s 期望达到的目标前进速度
    float speed;                     // m/s 实际控制的目标前进速度（加入积分项消除静差）
    float speed_integral;            // m/s 速度积分项
    float rotation_torque;           // N*m 旋转力矩
    float yaw;                       // rad 期望达到的目标航向角
    float pitch;                     // rad 期望达到的目标俯仰角
    float roll;                      // rad 期望达到的目标横滚角
    float leg_length;                // m  期望达到的目标腿长
    float left_length, right_length; // m  期望达到的目标腿长
    float leg_angle;                 // rad 期望达到的目标腿角
} Target_s;

/**
 * @brief      限制量结构体
 * @note       无
 */
typedef struct
{
    float leg_angle_max;       // 腿角最大摆幅（以PI/2为中心）
    float leg_length_min;      // 最短腿长
    float leg_length_max;      // 最长腿长
    float pitch_max;           // 最大pitch倾角
    float roll_max;            // 最大roll倾角
    float speed_cmd_max;       // 最大速度
    float speed_integral_max;  // 最大速度积分值
    float rotation_torque_max; // 最大旋转力矩
} Limit_Value_t;

/**
 * @brief      触地检测数据结构体
 * @note       无
 */
typedef struct
{
    float left_support_force, right_support_force;
    bool_t is_touching_ground, is_slipping;
    uint32_t last_touching_ground_time;
} Ground_Detector_s;

/**
 * @brief      比例系数结构体
 * @note       比例系数，用于手动优化控制效果
 */
typedef struct
{
    float kRatio[2][6];
    float LQR_Tp_ratio;
    float LQR_T_ratio;
    float length_ratio;
} Ratio_t;

/**
 * @brief      控制模式
 * @note       无
 */
typedef enum
{
    No_Control,
    Location_Control,
    Torque_Control,
} CyberGear_Control_State_e;

typedef enum
{
    OFF,      // 关闭
    STAND,    // 站立
    MOVING,   // 移动
    JUMPING,  // 跳跃
    FLOATING, // 悬空
    LEG_ONLY, // 仅控制关节电机
} Balance_Chassis_State_e;

// 外用变量
extern MI_Motor_s MI_Motor[5];
extern Motor_s left_joint[2], right_joint[2], left_wheel, right_wheel; // 六个电机对象

// 外用函数
void SetCyberGearMechPositionToZero();

const Chassis_IMU_t *GetChassisIMUPoint();
const Target_s *GetTargetPoint();
const Leg_Pos_t *GetLegPosPoint(uint8_t leg);
const State_Var_s *GetStateVarPoint();

void InitBalanceControler();
void DataUpdate(
    Chassis_IMU_t *p_chassis_IMU,
    float speed, float yaw_delta, float pitch, float roll, float length, float rotation_torque);
void ControlBalanceChassis(CyberGear_Control_State_e CyberGear_control_state);
void BalanceControlerCalc();

#endif // BALANCE_CONTROLER_H