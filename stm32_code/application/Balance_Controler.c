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

#include <math.h>
#include "Balance_Controler.h"
#include "./Drives/MI_motor_drive.h"
#include <string.h>

#include "main.h"

#define WHEEL_BASE 0.3f // 轮距(m)

#define MAX_LEG_LENGTH 0.25f // 最大腿长(m)
#define MIN_LEG_LENGTH 0.11f // 最小腿长(m)

/*CAN通信所需的变量*/
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

static CAN_TxHeaderTypeDef wheel_tx_message;
static uint8_t wheel_can_send_data[8];

/*电机变量*/
MI_Motor_s MI_Motor[5]; // 小米电机结构体
MI_Motor_s MI_Motor_None;
Motor_s left_joint[2], right_joint[2], left_wheel, right_wheel; // 六个电机对象

/*状态反馈*/
static Leg_Pos_t left_leg_pos, right_leg_pos; // 左右腿部反馈姿态
static Chassis_IMU_t chassis_imu;             // 底盘IMU数据
static State_Var_s state_var;
static Ground_Detector_s ground_detector = {10, 10, true, false, 0};

/*PID*/
static CascadePID yaw_PID; // 机身角度控制PID
static PID pitch_PID, roll_PID;

/*目标与限制*/
static Limit_Value_t limit_value;
static Target_s target;

/*比例系数，用于手动优化控制效果*/
Ratio_t ratio;

/*底盘状态*/
static Balance_Chassis_State_e chassis_state = OFF;

/**************************** 通用函数 ****************************/

/**
 * @brief       获取从系统运行开始经过的时间，默认情况下单位为ms；
 * @param[in]   motor 电机结构体地址
 * @return      millis 从系统运行开始经过的时间
 */
static uint32_t GetMillis()
{
    return HAL_GetTick();
}

/**
 * @brief 通过当前底盘姿态和目标roll角计算两腿长度期望差值
 * @param[in]  Ld0 (m)当前左右腿长度差值(L0l - L0r)
 * @param[in]  theta0 (rad)当前底盘roll角
 * @param[in]  theta1 (rad)目标roll角
 * @return 两腿长度期望差值(m)(L1l - L1r)
 */
inline float CalcLegLengthDiff(float Ld0, float theta0, float theta1)
{
    return WHEEL_BASE * tanf(theta1) -
           cosf(theta0) / cosf(theta1) * (WHEEL_BASE * tanf(theta0) - Ld0);
}

/**
 * @brief 双腿腿长协调控制，维持腿长目标在范围内，同时尽可能达到两腿目标差值
 * @param[in]  Ll_ref   (m)左腿长度指针
 * @param[in]  Lr_ref   (m)右腿长度指针
 * @param[in]  diff (m)腿长差值
 * @param[in]  add  (m)腿长差值补偿量
 */
void CoordinateLegLength(float *Ll_ref, float *Lr_ref, float diff, float add)
{
    *Ll_ref = *Ll_ref + diff * 0.5f + add;
    *Lr_ref = *Lr_ref - diff * 0.5f - add;

    // 先判断短腿范围，再判断长腿范围
    float *short_leg = *Ll_ref < *Lr_ref ? Ll_ref : Lr_ref;
    float *long_leg = *Ll_ref < *Lr_ref ? Lr_ref : Ll_ref;
    float move = 0;
    move = MIN_LEG_LENGTH - *short_leg;
    if (move > 0)
    {
        *short_leg += move;
        *long_leg += move;
    }
    if (*long_leg > MAX_LEG_LENGTH)
    {
        *long_leg = MAX_LEG_LENGTH;
    }
}
/**************************** 电机模块 ****************************/

/**
 * @brief          初始化一个电机对象
 * @attention
 * @note
 * @param[in]      motor 电机结构体地址
 * @param[in]      MI_Motor 小米电机结构体地址
 * @param[in]      offsetAngle
 * @param[in]      max_voltage
 * @param[in]      torque_ratio
 * @param[in]      dir
 * @return         none
 */
static void MotorInit(
    Motor_s *motor, MI_Motor_s *MI_Motor, CAN_HandleTypeDef *hcan, uint8_t motor_id,
    float initial_angle, float vertical_angle, float horizontal_angle, float upper_limit_angle, float lower_limit_angle,
    float max_voltage, float torque_ratio, float dir)
{
    motor->MI_Motor = MI_Motor;
    MI_motor_Init(MI_Motor, hcan, motor_id);
    motor->speed = motor->angle = motor->voltage = 0;
    motor->initial_angle = initial_angle;
    motor->vertical_angle = vertical_angle;
    motor->horizontal_angle = horizontal_angle;
    motor->upper_limit_angle = upper_limit_angle;
    motor->lower_limit_angle = lower_limit_angle;
    motor->max_voltage = max_voltage;
    motor->torque_ratio = torque_ratio;
    motor->dir = dir;
}

/**
 * @brief          设置电机扭矩
 * @param[in]      none
 * @return         none
 */
static void MotorSetTorque(Motor_s *motor, float torque)
{
    motor->torque = torque;
}

/**
 * @brief  设置电机目标角度
 * @param[in]      none
 * @return         none
 */
static void MotorSetTargetAngle(Motor_s *motor, float target_angle)
{
    motor->target_angle = target_angle;
}

/**
 * @brief          2006电机力矩到电流值的映射
 * @note
 * @param[in]      torque 力矩大小(N*m)
 * @param[in]      k 转矩常数(N*m/A)
 * @param[in]      start_torque 启动力矩大小(N*m)
 * @return         send_velue 电流值大小
 */
static int16_t MotorTorqueToCurrentValue_2006(float torque, float k, float start_torque)
{
    // float k = 0.18f; // N*m/A
    float _torque = torque;
    if (torque > 0)
        _torque = torque + start_torque;
    else if (torque < 0)
        _torque = torque - start_torque;

    float current; // A
    current = _torque / k;
    int16_t send_velue = (int16_t)(1000 * current);
    return send_velue;
}

/**************************** 状态设置 ****************************/

/**
 * @brief      底盘姿态更新
 * @param[in]  p_chassis_imu 计算好的底盘IMU数据指针
 * @return     none
 * @note       通过传入计算好的底盘姿态数据更新底盘姿态
 */
static void ChassisPostureUpdate(Chassis_IMU_t *p_chassis_IMU)
{
    chassis_imu = *p_chassis_IMU;
}

/**
 * @brief 更新平衡底盘状态
 * @param state 平衡底盘状态
 */
void BalanceChassisStateUpdate(Balance_Chassis_State_e state)
{
    chassis_state = state;
}

/**************************** 运动控制模块 ****************************/

/**
 * @brief          计算关节与正方向水平面的夹角
 * @param[in]      left_joint  左关节电机结构体
 * @param[in]      right_joint 右关节电机结构体
 * @return         none
 */
static void CalcJointAngle(Motor_s *left_joint, Motor_s *right_joint)
{
    left_joint[0].angle = left_joint[0].horizontal_angle - left_joint[0].MI_Motor->RxCAN_info.angle;
    left_joint[1].angle = left_joint[1].horizontal_angle - left_joint[1].MI_Motor->RxCAN_info.angle;
    right_joint[0].angle = right_joint[0].MI_Motor->RxCAN_info.angle - right_joint[0].horizontal_angle;
    right_joint[1].angle = right_joint[1].MI_Motor->RxCAN_info.angle - right_joint[1].horizontal_angle;
}

/**
 * @brief          目标值限幅
 * @param[in]      none
 * @return         none
 * @note           防止目标值超限导致的控制问题
 */
static void CtrlTargetLimit()
{
    // 限制前进速度
    if (target.speed_cmd > limit_value.speed_cmd_max)
        target.speed_cmd = limit_value.speed_cmd_max;
    else if (target.speed_cmd < -limit_value.speed_cmd_max)
        target.speed_cmd = -limit_value.speed_cmd_max;

    if (target.speed_integral > limit_value.speed_integral_max)
        target.speed_integral = limit_value.speed_integral_max;
    else if (target.speed_integral < -limit_value.speed_integral_max)
        target.speed_integral = -limit_value.speed_integral_max;

    // 限制yaw角
    if (target.yaw > M_PI)
        target.yaw = target.yaw - M_PI * 2;
    else if (target.yaw < -M_PI)
        target.yaw = target.yaw + M_PI * 2;

    // 限制pitch角
    if (target.pitch < -limit_value.pitch_max)
        target.pitch = -limit_value.pitch_max;
    else if (target.pitch > limit_value.pitch_max)
        target.pitch = limit_value.pitch_max;

    // 限制roll角
    if (target.roll < -limit_value.roll_max)
        target.roll = -limit_value.roll_max;
    else if (target.roll > limit_value.roll_max)
        target.roll = limit_value.roll_max;

    // 限制腿长
    if (target.leg_length < limit_value.leg_length_min)
        target.leg_length = limit_value.leg_length_min;
    else if (target.leg_length > limit_value.leg_length_max)
        target.leg_length = limit_value.leg_length_max;

    if (target.left_length < limit_value.leg_length_min)
        target.left_length = limit_value.leg_length_min;
    else if (target.left_length > limit_value.leg_length_max)
        target.left_length = limit_value.leg_length_max;

    if (target.right_length < limit_value.leg_length_min)
        target.right_length = limit_value.leg_length_min;
    else if (target.right_length > limit_value.leg_length_max)
        target.right_length = limit_value.leg_length_max;

    // 限制旋转力矩
    if (target.rotation_torque > limit_value.rotation_torque_max)
        target.rotation_torque = limit_value.rotation_torque_max;
    else if (target.rotation_torque < -limit_value.rotation_torque_max)
        target.rotation_torque = -limit_value.rotation_torque_max;

    // 限制腿部角度
    if (target.leg_angle > M_PI_2 + limit_value.leg_angle_max)
        target.leg_angle = M_PI_2 + limit_value.leg_angle_max;
    else if (target.leg_angle < M_PI_2 - limit_value.leg_angle_max)
        target.leg_angle = M_PI_2 - limit_value.leg_angle_max;
}

/**
 * @brief          更新目标值
 * @param[in]      speed           速度值
 * @param[in]      yaw_delta       yaw角度增量
 * @param[in]      pitch           pitch角度
 * @param[in]      roll            roll角度
 * @param[in]      length          腿长
 * @param[in]      rotation_torque 旋转力矩
 * @return         none
 */
static void CtrlTargetUpdate(float speed, float yaw_delta, float pitch, float roll, float length, float rotation_torque)
{
    // 设置前进速度
    target.speed_cmd = speed;

    // 设置yaw方位角
    target.yaw = target.yaw + yaw_delta;

    // 设置pitch角
    target.pitch = pitch;

    // 设置roll角
    target.roll = roll;

    // 设置目标长度
    target.leg_length = length;

    // 设置旋转力矩
    target.rotation_torque = rotation_torque;
}

/**
 * @brief          腿部姿态更新任务
 * @param[in]      none
 * @return         none
 * @note           根据关节电机数据计算腿部姿态
 */
static void LegPosUpdate()
{
    // 更新关节信息
    CalcJointAngle(left_joint, right_joint);
    left_joint[0].speed = left_joint[0].MI_Motor->RxCAN_info.speed;
    left_joint[1].speed = left_joint[1].MI_Motor->RxCAN_info.speed;
    right_joint[0].speed = right_joint[0].MI_Motor->RxCAN_info.speed;
    right_joint[1].speed = right_joint[1].MI_Motor->RxCAN_info.speed;

    const float lpf_ratio = 0.5f; // 低通滤波系数(新值的权重)
    float last_left_dLength = 0, last_right_dLength = 0;

    float legPos[2], legSpd[2];

    // 计算左腿位置
    LegPos(left_joint[1].angle, left_joint[0].angle, legPos);
    left_leg_pos.length = legPos[0];
    left_leg_pos.angle = legPos[1];

    // 计算左腿速度
    LegSpd(left_joint[1].speed, left_joint[0].speed, left_joint[1].angle, left_joint[0].angle, legSpd);
    left_leg_pos.dLength = legSpd[0];
    left_leg_pos.dAngle = legSpd[1];

    // 计算左腿腿长加速度
    left_leg_pos.ddLength = ((left_leg_pos.dLength - last_left_dLength) * 1000 / 4) * lpf_ratio + left_leg_pos.ddLength * (1 - lpf_ratio);
    last_left_dLength = left_leg_pos.dLength;

    // 计算右腿位置
    LegPos(right_joint[1].angle, right_joint[0].angle, legPos);
    right_leg_pos.length = legPos[0];
    right_leg_pos.angle = legPos[1];

    // 计算右腿速度
    LegSpd(right_joint[1].speed, right_joint[0].speed, right_joint[1].angle, right_joint[0].angle, legSpd);
    right_leg_pos.dLength = legSpd[0];
    right_leg_pos.dAngle = legSpd[1];

    // 计算右腿腿长加速度
    right_leg_pos.ddLength = ((right_leg_pos.dLength - last_right_dLength) * 1000 / 4) * lpf_ratio + right_leg_pos.ddLength * (1 - lpf_ratio);
    last_right_dLength = right_leg_pos.dLength;
}

/**************************** 初始化模块 ****************************/

/**
 * @brief          初始化各个PID参数
 * @param[in]      none
 * @return         none
 */
static void PIDInit()
{
    // yaw轴角度PID
    PID_Init(&yaw_PID.inner, 0.007, 0, 0.1, 0, 2);
    PID_Init(&yaw_PID.outer, 30, 0, 0, 0, 10);

    // pitch轴角度PID
    PID_Init(&pitch_PID, 0.0, 0, 0.0, 0, 0.0);

    // roll轴角度PID
    PID_Init(&roll_PID, 0.0, 0, 0.0, 0, 0.0);
}

/**
 * @brief          初始化所有电机对象
 * @param[in]      none
 * @return         none
 */
static void MotorInitAll()
{
    MotorInit(&left_joint[0], &MI_Motor[1], &MI_CAN_2, 1,
              -0.00019175051420461386f, // initial angle
              -0.349 - M_PI_2,          // vertical angle
              -0.349,                   // horizontal angle
              1.74,
              0.0,
              7, 0.0316f, -1);

    MotorInit(&left_joint[1], &MI_Motor[2], &MI_CAN_2, 2,
              -0.00019175051420461386f, // initial angle
              0.349 + M_PI_2,           // vertical angle
              0.349 + M_PI,             // horizontal angle
              1.74,
              0.0,
              7, 0.0317f, 1);

    MotorInit(&left_wheel, &MI_Motor_None, &MI_CAN_2, 0,
              0,
              0,
              0,
              0,
              0,
              4.0f, 0.0096f, 1);

    MotorInit(&right_joint[0], &MI_Motor[3], &MI_CAN_2, 3,
              -0.00019175051420461386f, // initial angle
              0.349 + M_PI_2,           // vertical angle
              0.349,                    // horizontal angle
              -0.0,
              -1.74,
              7, 0.0299f, -1);

    MotorInit(&right_joint[1], &MI_Motor[4], &MI_CAN_2, 4,
              -0.00019175051420461386f, // initial angle
              -0.349 - M_PI_2,          // vertical angle
              -0.349 - M_PI,            // horizontal angle
              -0.0,
              -1.74,
              7, 0.0321f, -1);

    MotorInit(&right_wheel, &MI_Motor_None, &MI_CAN_2, 0,
              0,
              0,
              0,
              0,
              0,
              4.0f, 0.0101f, 1);
}

/**************************** 电机控制模块 ****************************/

/**
 * @brief          发送驱动轮电机控制电流(0x207,0x208)
 * @param[in]      left_wheel  (0x207) 2006电机控制电流, 范围 [-10000,10000]
 * @param[in]      right_wheel (0x208) 2006电机控制电流, 范围 [-10000,10000]
 * @return         none
 */
static void CANCmdWheel(int16_t left_wheel, int16_t right_wheel)
{
    uint32_t send_mail_box;

    if (left_wheel > 10000)
        left_wheel = 10000;
    else if (left_wheel < -10000)
        left_wheel = -10000;

    if (right_wheel > 10000)
        right_wheel = 10000;
    else if (right_wheel < -10000)
        right_wheel = -10000;

    wheel_tx_message.StdId = 0x1FF;
    wheel_tx_message.IDE = CAN_ID_STD;
    wheel_tx_message.RTR = CAN_RTR_DATA;
    wheel_tx_message.DLC = 0x08;
    wheel_can_send_data[0] = ((int16_t)0 >> 8);
    wheel_can_send_data[1] = (int16_t)0;
    wheel_can_send_data[2] = ((int16_t)0 >> 8);
    wheel_can_send_data[3] = (int16_t)0;
    wheel_can_send_data[4] = (left_wheel >> 8);
    wheel_can_send_data[5] = left_wheel;
    wheel_can_send_data[6] = (right_wheel >> 8);
    wheel_can_send_data[7] = right_wheel;

    uint32_t free_TxMailbox = HAL_CAN_GetTxMailboxesFreeLevel(&WHEEL_CAN); // 检测是否有空闲邮箱
    while (free_TxMailbox < 3)
    { // 等待空闲邮箱数达到3
        free_TxMailbox = HAL_CAN_GetTxMailboxesFreeLevel(&WHEEL_CAN);
    }
    HAL_CAN_AddTxMessage(&WHEEL_CAN, &wheel_tx_message, wheel_can_send_data, &send_mail_box);
}

/**
 * @brief          发送关节位置控制信号
 * @param[in]      kp 响应速度(到达位置快慢)，一般取1-10
 * @param[in]      kd 电机阻尼，过小会震荡，过大电机会震动明显。一般取0.5左右
 * @return         none
 */
static void CANCmdJointLocation(float kp, float kd)
{
    float send_angle[4];

    send_angle[0] = left_joint[0].horizontal_angle - left_joint[0].target_angle;
    MI_motor_LocationControl(left_joint[0].MI_Motor, send_angle[0], kp, kd);
    for (int i = 0; i < 1; i++)
        MI_motor_ReadParam(left_joint[0].MI_Motor, 0X302d); // 添加后可以有效缓解小米电机反馈数据丢包问题（原理未知）

    send_angle[1] = left_joint[1].horizontal_angle - left_joint[1].target_angle;
    MI_motor_LocationControl(left_joint[1].MI_Motor, send_angle[1], kp, kd);
    for (int i = 0; i < 1; i++)
        MI_motor_ReadParam(left_joint[1].MI_Motor, 0X302d); // 添加后可以有效缓解小米电机反馈数据丢包问题（原理未知）

    send_angle[2] = right_joint[0].horizontal_angle + right_joint[0].target_angle;
    MI_motor_LocationControl(right_joint[0].MI_Motor, send_angle[2], kp, kd);
    for (int i = 0; i < 1; i++)
        MI_motor_ReadParam(right_joint[0].MI_Motor, 0X302d); // 添加后可以有效缓解小米电机反馈数据丢包问题（原理未知）

    send_angle[3] = right_joint[1].horizontal_angle + right_joint[1].target_angle;
    MI_motor_LocationControl(right_joint[1].MI_Motor, send_angle[3], kp, kd);
    for (int i = 0; i < 1; i++)
        MI_motor_ReadParam(right_joint[1].MI_Motor, 0X302d); // 添加后可以有效缓解小米电机反馈数据丢包问题（原理未知）
}

/**
 * @brief          发送关节力矩控制信号
 * @param[in]      limit_torque 限制力矩大小
 * @return         none
 */
static void CANCmdJointTorque(float limit_torque)
{
    /*安全保护措施*/
    /*进行关节力矩限制,输出力矩不得超过限制范围*/
    for (int i = 0; i < 2; i++)
    {
        if (left_joint[i].torque > limit_torque)
            left_joint[i].torque = limit_torque;
        else if (left_joint[i].torque < -limit_torque)
            left_joint[i].torque = -limit_torque;

        if (right_joint[i].torque > limit_torque)
            right_joint[i].torque = limit_torque;
        else if (right_joint[i].torque < -limit_torque)
            right_joint[i].torque = -limit_torque;
    }

    MI_motor_TorqueControl(left_joint[0].MI_Motor, left_joint[0].torque);
    for (int i = 0; i < 1; i++)
        MI_motor_ReadParam(left_joint[0].MI_Motor, 0X302d); // 添加后可以有效缓解小米电机反馈数据丢包问题（原理未知）

    MI_motor_TorqueControl(left_joint[1].MI_Motor, left_joint[1].torque);
    for (int i = 0; i < 1; i++)
        MI_motor_ReadParam(left_joint[1].MI_Motor, 0X302d); // 添加后可以有效缓解小米电机反馈数据丢包问题（原理未知）

    MI_motor_TorqueControl(right_joint[0].MI_Motor, right_joint[0].torque);
    for (int i = 0; i < 1; i++)
        MI_motor_ReadParam(right_joint[0].MI_Motor, 0X302d); // 添加后可以有效缓解小米电机反馈数据丢包问题（原理未知）

    MI_motor_TorqueControl(right_joint[1].MI_Motor, right_joint[1].torque);
    for (int i = 0; i < 1; i++)
        MI_motor_ReadParam(right_joint[1].MI_Motor, 0X302d); // 添加后可以有效缓解小米电机反馈数据丢包问题（原理未知）
}

/**
 * @brief          紧急关闭关节电机
 * @param[in]      none
 * @return         none
 */
static void CANCmdJointEmergencyStop()
{
    MI_motor_Stop(left_joint[0].MI_Motor);
    for (int i = 0; i < 1; i++)
        MI_motor_ReadParam(left_joint[0].MI_Motor, 0X302d); // 添加后可以有效缓解小米电机反馈数据丢包问题（原理未知）

    MI_motor_Stop(left_joint[1].MI_Motor);
    for (int i = 0; i < 1; i++)
        MI_motor_ReadParam(left_joint[1].MI_Motor, 0X302d); // 添加后可以有效缓解小米电机反馈数据丢包问题（原理未知）

    MI_motor_Stop(right_joint[0].MI_Motor);
    for (int i = 0; i < 1; i++)
        MI_motor_ReadParam(right_joint[0].MI_Motor, 0X302d); // 添加后可以有效缓解小米电机反馈数据丢包问题（原理未知）

    MI_motor_Stop(right_joint[1].MI_Motor);
    for (int i = 0; i < 1; i++)
        MI_motor_ReadParam(right_joint[1].MI_Motor, 0X302d); // 添加后可以有效缓解小米电机反馈数据丢包问题（原理未知）
}

/**
 * @brief          设置小米电机零点
 * @param[in]      none
 * @return         none
 */
void SetCyberGearMechPositionToZero()
{
    MI_motor_SetMechPositionToZero(left_joint[0].MI_Motor);
    for (int i = 0; i < 1; i++)
        MI_motor_ReadParam(left_joint[0].MI_Motor, 0X302d); // 添加后可以有效缓解小米电机反馈数据丢包问题（原理未知）

    MI_motor_SetMechPositionToZero(left_joint[1].MI_Motor);
    for (int i = 0; i < 1; i++)
        MI_motor_ReadParam(left_joint[1].MI_Motor, 0X302d); // 添加后可以有效缓解小米电机反馈数据丢包问题（原理未知）

    MI_motor_SetMechPositionToZero(right_joint[0].MI_Motor);
    for (int i = 0; i < 1; i++)
        MI_motor_ReadParam(right_joint[0].MI_Motor, 0X302d); // 添加后可以有效缓解小米电机反馈数据丢包问题（原理未知）

    MI_motor_SetMechPositionToZero(right_joint[1].MI_Motor);
    for (int i = 0; i < 1; i++)
        MI_motor_ReadParam(right_joint[1].MI_Motor, 0X302d); // 添加后可以有效缓解小米电机反馈数据丢包问题（原理未知）
}

/**************************** 反馈与计算 ****************************/

/**
 * @brief         计算状态变量
 * @param[out]    x 状态变量向量
 * @return        none
 */
static void StateVarCalc(float x[6])
{
    state_var.phi = chassis_imu.pitch;
    state_var.dPhi = chassis_imu.pitchSpd;
    state_var.x = (left_wheel.angle + right_wheel.angle) / 2 * WHEEL_RADIUS;
    state_var.dx = (left_wheel.speed + right_wheel.speed) / 2 * WHEEL_RADIUS;
    state_var.theta = (left_leg_pos.angle + right_leg_pos.angle) / 2 - M_PI_2 - chassis_imu.pitch;
    state_var.dTheta = (left_leg_pos.dAngle + right_leg_pos.dAngle) / 2 - chassis_imu.pitchSpd;
    state_var.leg_length = (left_leg_pos.length + right_leg_pos.length) / 2;
    state_var.dLegLength = (left_leg_pos.dLength + right_leg_pos.dLength) / 2;

    x[0] = state_var.theta;
    x[1] = state_var.dTheta;
    x[2] = state_var.x - target.position;
    x[3] = state_var.dx - target.speed;
    x[4] = state_var.phi;
    x[5] = state_var.dPhi;
}

/**
 * @brief         设置LQR的反馈矩阵K
 * @param[in]     leg_length 当前腿长
 * @param[out]    k 返回的反馈矩阵指针
 * @return        none
 */
static void SetLQR_K(float leg_length, float k[2][6])
{
    float kRes[12] = {0};
    LQR_K(leg_length, kRes);
    if (ground_detector.is_touching_ground) // 正常触地状态
    {
        for (int i = 0; i < 6; i++)
        {
            for (int j = 0; j < 2; j++)
                k[j][i] = kRes[i * 2 + j] * ratio.kRatio[j][i];
            // k[j][i] = kRes[i * 2 + j] * kRatio[j][i];
        }
    }
    else // 腿部离地状态，手动修改反馈矩阵，仅保持腿部竖直
    {
        memset(k, 0, sizeof(k));
        k[1][0] = kRes[1] * -2;
        k[1][1] = kRes[3] * -10;
    }

    // k[0][2] = 0.0; // 因为用不到距离反馈，所以置零
    // k[1][2] = 0.0; // 因为用不到距离反馈，所以置零
}

/**
 * @brief         矩阵相乘，计算LQR输出
 * @param[in]     k   LQR反馈矩阵K
 * @param[in]     x   状态变量向量
 * @param[out]    res 反馈数据T和Tp
 * @return        none
 * @note T为res[0],Tp为res[1]
 */
static void LQRFeedbackCalc(float k[2][6], float x[6], float res[2])
{
    res[0] = k[0][0] * x[0] + k[0][1] * x[1] + k[0][2] * x[2] + k[0][3] * x[3] + k[0][4] * x[4] + k[0][5] * x[5];
    res[1] = k[1][0] * x[0] + k[1][1] * x[1] + k[1][2] * x[2] + k[1][3] * x[3] + k[1][4] * x[4] + k[1][5] * x[5];
}

/**
 * @brief        离地检测
 * @param[in]    left_force  (N)左腿支持力
 * @param[in]    right_force (N)右腿支持力
 * @param[in]    leg_length  (m)当前腿长
 * @param[in]    interval    (ms)距上次检测到离地的最大时间间隔
 * @return       none
 */
static void GroundTouchingDetect(float left_force, float right_force, float leg_length, uint32_t interval)
{
    // 计算左右腿的地面支持力
    ground_detector.left_support_force = left_force + LEG_MASS * 9.8f - LEG_MASS * (left_leg_pos.ddLength - chassis_imu.zAccel);
    ground_detector.right_support_force = left_force + LEG_MASS * 9.8f - LEG_MASS * (right_leg_pos.ddLength - chassis_imu.zAccel);
    // 更新离地检测器数据
    bool_t is_touching_ground = ground_detector.left_support_force > 3 && ground_detector.right_support_force > 3; // 判断当前瞬间是否接地
    // if(!is_touching_ground && GetMillis() - ground_detector.last_touching_ground_time < interval) //若上次触地时间距离现在不超过interval ms，则认为当前瞬间接地，避免弹跳导致误判
    //     is_touching_ground = true;
    // if(!ground_detector.is_touching_ground && is_touching_ground) //判断转为接地状态，标记进入缓冲状态
    // {
    //     target.position = state_var.x;
    //     ground_detector.is_cuchioning = true;
    //     ground_detector.last_touching_ground_time = GetMillis();
    // }
    // if(ground_detector.is_cuchioning && leg_length < target.leg_length) //缓冲状态直到腿长压缩到目标腿长结束
    //     ground_detector.is_cuchioning = false;
    // ground_detector.is_touching_ground = is_touching_ground;
}

/**
 * @brief      计算关节电机输出位置
 * @param[in]  left_leg  左腿的期望姿态
 * @param[in]  right_leg 右腿的期望姿态
 * @return     none
 * @note 只使用姿态中的length和angle
 */
void JointPosCalc(Leg_Pos_t *left_leg, Leg_Pos_t *right_leg)
{
    float joint_pos[2];
    JointPos(left_leg->length, left_leg->angle, joint_pos); // 计算左关节摆角
    // 当解算出的角度正常时，设置目标角度
    if (!(isnan(joint_pos[0]) || isnan(joint_pos[1])))
    {
        MotorSetTargetAngle(&left_joint[1], joint_pos[0]);
        MotorSetTargetAngle(&left_joint[0], joint_pos[1]);
    }

    JointPos(right_leg->length, right_leg->angle, joint_pos); // 计算右关节摆角
    // 当解算出的角度正常时，设置目标角度
    if (!(isnan(joint_pos[0]) || isnan(joint_pos[1])))
    {
        MotorSetTargetAngle(&right_joint[1], joint_pos[0]);
        MotorSetTargetAngle(&right_joint[0], joint_pos[1]);
    }
}

/**************************** 获取控制器数据 ****************************/

/**
 * @brief      获取底盘IMU数据
 * @param[in]  none
 * @note        ！！！不可以修改数据内容！！！
 * @return     底盘IMU数据指针
 */
const Chassis_IMU_t *GetChassisIMUPoint()
{
    return &chassis_imu;
}

/**
 * @brief      获取目标值
 * @param[in]  none
 * @note        ！！！不可以修改数据内容！！！
 * @return     目标值指针
 */
const Target_s *GetTargetPoint()
{
    return &target;
}

/**
 * @brief       获取腿部姿态数据
 * @param[out]  leg  腿的编号(0-left,1-right)，默认返回左腿
 * @note        ！！！不可以修改数据内容！！！
 * @return      none
 */
const Leg_Pos_t *GetLegPosPoint(uint8_t leg)
{
    switch (leg)
    {
    case 0:
        return &left_leg_pos;
        break;
    case 1:
        return &right_leg_pos;
        break;
    default:
        return &left_leg_pos;
        break;
    }
}

/**
 * @brief       获取机器人状态数据
 * @param[in]   none
 * @return      机器人状态数据
 */
const State_Var_s *GetStateVarPoint()
{
    return &state_var;
}

/**
 * @brief 获取平衡底盘状态
 * @return 平衡底盘状态
 */
Balance_Chassis_State_e GetBalanceChassisState()
{
    return chassis_state;
}

/**************************** 抽象与封装 ****************************/

/**
 * @brief      平衡控制器的初始化
 * @param[in]  none
 * @return     none
 */
void InitBalanceControler()
{
    MotorInitAll();
    PIDInit();

    ground_detector.last_touching_ground_time = 0;
    ground_detector.is_touching_ground = true;
    ground_detector.is_slipping = false;

    // 设置比例系数
    ratio.kRatio[0][0] = ratio.kRatio[1][0] = 1.0f;
    ratio.kRatio[0][1] = ratio.kRatio[1][1] = 1.0f;
    ratio.kRatio[0][2] = ratio.kRatio[1][2] = 0.0f;
    ratio.kRatio[0][3] = ratio.kRatio[1][3] = 1.0f;
    ratio.kRatio[0][4] = ratio.kRatio[1][4] = 1.0f;
    ratio.kRatio[0][5] = ratio.kRatio[1][5] = 1.0f;
    ratio.LQR_T_ratio = 1.0;
    ratio.LQR_Tp_ratio = 1.0;
    ratio.length_ratio = 1.2;

    // 设定初始目标值
    target.roll = 0.0f;
    target.speed = 0.0f;
    target.position = (left_wheel.angle + right_wheel.angle) / 2 * WHEEL_RADIUS;
    target.yaw = 0.0f;
    target.leg_length = 0.185f;
    target.leg_angle = M_PI_2;

    // 设定各种限额
    limit_value.leg_angle_max = M_PI / 6;
    limit_value.leg_length_min = MIN_LEG_LENGTH;
    limit_value.leg_length_max = MAX_LEG_LENGTH;
    limit_value.pitch_max = M_PI / 6;
    limit_value.roll_max = M_PI / 8;
    limit_value.speed_cmd_max = 5.0f;
    limit_value.rotation_torque_max = 1.0f;
    limit_value.speed_integral_max = 0.5f;
}

/**
 * @brief      数据更新
 * @param[in]  none
 * @return     none
 */
void DataUpdate(
    Chassis_IMU_t *p_chassis_IMU,
    float speed, float yaw_delta, float pitch, float roll, float length, float rotation_torque)
{
    ChassisPostureUpdate(p_chassis_IMU);
    CtrlTargetUpdate(speed, yaw_delta, pitch, roll, length, rotation_torque);
    LegPosUpdate();
}

/**
 * @brief          发送平衡底盘控制信号
 * @param[in]      CyberGear_control_state 底盘控制模式
 * @param[in]      L_kp 位置控制的比例系数
 * @return         none
 */
void ControlBalanceChassis(CyberGear_Control_State_e CyberGear_control_state, float L_kp)
{
    switch (CyberGear_control_state)
    {
    case Location_Control: // 发送关节控制位置
    {
        CANCmdJointLocation(L_kp, 0.5);
        break;
    }
    case Torque_Control: // 发送关节控制力矩
    {
        CANCmdJointTorque(2.0);
        break;
    }
    default: // 紧急停止
    {
        CANCmdJointEmergencyStop();
        break;
    }
    }

    // 发送车轮控制力矩
    CANCmdWheel(
        MotorTorqueToCurrentValue_2006(left_wheel.torque, K_2006, START_TORQUE_2006),
        MotorTorqueToCurrentValue_2006(right_wheel.torque, K_2006, START_TORQUE_2006));
}

/**
 * @brief          平衡底盘计算
 * @param[in]      none
 * @return         none
 */
void BalanceControlerCalc()
{

    // GroundTouchingDetect();//离地检测

    // 根据底盘状态进行控制
    Leg_Pos_t left_leg_target;
    Leg_Pos_t right_leg_target;

    // 计算控制量
    switch (chassis_state)
    {
    case OFF: // 底盘关闭状态
        MotorSetTorque(&left_wheel, 0);
        MotorSetTorque(&right_wheel, 0);
        MotorSetTorque(&left_joint[0], 0);
        MotorSetTorque(&left_joint[1], 0);
        MotorSetTorque(&right_joint[0], 0);
        MotorSetTorque(&right_joint[1], 0);
        break;
    case STAND: // 原地站立状态
        target.speed_integral = target.speed_integral + (target.speed_cmd - state_var.dx) * SPEED_KI;
    case MOVING: // 移动状态
        ratio.LQR_T_ratio = 0.2;

        CtrlTargetLimit();
        target.speed = target.speed_cmd + target.speed_integral;

        // LQR计算部分
        float x[6]; // 状态变量
        StateVarCalc(x);
        float k[2][6]; // LQR反馈矩阵
        SetLQR_K(state_var.leg_length, k);
        float res[2];
        LQRFeedbackCalc(k, x, res);
        float LQR_out_T = res[0];  // 沿摆杆径向的力
        float LQR_out_Tp = res[1]; // 沿摆杆法向的力

        // 维持pitch角与设定值相同
        PID_SingleCalc(&pitch_PID, target.pitch, chassis_imu.pitch);
        target.leg_angle = M_PI_2 + pitch_PID.output;

        // 维持roll角与设定值相同
        PID_SingleCalc(&roll_PID, target.roll, chassis_imu.roll);
        target.left_length = target.leg_length + roll_PID.output;
        target.right_length = target.leg_length - roll_PID.output;

        float diff = CalcLegLengthDiff(left_leg_pos.length - right_leg_pos.length, chassis_imu.roll, target.roll);
        CoordinateLegLength(&target.left_length, &target.right_length,  diff, 0);

        // yaw角跟踪
        float angleFdb = target.yaw - chassis_imu.yaw; // 目标角度与底盘角度反馈之差
        if (angleFdb > M_PI)
            angleFdb = angleFdb - M_PI * 2;
        else if (angleFdb < -M_PI)
            angleFdb = angleFdb + M_PI * 2;
        PID_CascadeCalc(&yaw_PID, 0, angleFdb, chassis_imu.yawSpd);

        // 设定车轮电机输出扭矩，为LQR和旋转力矩的叠加
        float left_wheel_torque = -LQR_out_T * ratio.LQR_T_ratio - yaw_PID.output;
        float right_wheel_torque = LQR_out_T * ratio.LQR_T_ratio - yaw_PID.output;
        CtrlTargetLimit();

        // 设定关节输出
        left_leg_target.length = target.left_length;
        right_leg_target.length = target.right_length;
        left_leg_target.angle = target.leg_angle;
        right_leg_target.angle = target.leg_angle;
        JointPosCalc(&left_leg_target, &right_leg_target);

        MotorSetTorque(&left_wheel, left_wheel_torque);
        MotorSetTorque(&right_wheel, right_wheel_torque);
        break;
    case JUMPING: // 跳跃状态
        float explosive_torque = 10.0;
        MotorSetTorque(&left_joint[0], -explosive_torque);
        MotorSetTorque(&left_joint[1], explosive_torque);
        MotorSetTorque(&right_joint[0], explosive_torque);
        MotorSetTorque(&right_joint[1], -explosive_torque);
        MotorSetTorque(&left_wheel, 0);
        MotorSetTorque(&right_wheel, 0);
        break;
    case FLOATING: // 浮空状态
        // 关节位置设置

        left_leg_target.length = limit_value.leg_length_min;
        right_leg_target.length = limit_value.leg_length_min;

        left_leg_target.angle = M_PI_2;
        right_leg_target.angle = M_PI_2;
        JointPosCalc(&left_leg_target, &right_leg_target);

        MotorSetTorque(&left_wheel, 0);
        MotorSetTorque(&right_wheel, 0);
        break;
    default:
        MotorSetTorque(&left_wheel, 0);
        MotorSetTorque(&right_wheel, 0);
        MotorSetTorque(&left_joint[0], 0);
        MotorSetTorque(&left_joint[1], 0);
        MotorSetTorque(&right_joint[0], 0);
        MotorSetTorque(&right_joint[1], 0);
        break;
    }

    // 根据不同模式进行控制
    switch (chassis_state)
    {
    case STAND:  // 原地站立状态
    case MOVING: // 移动状态
        ControlBalanceChassis(Location_Control, L_KP_HARD);
        break;
    case FLOATING: // 浮空状态
        ControlBalanceChassis(Location_Control, L_KP_SOFT);
        break;
    case JUMPING: // 跳跃状态
        ControlBalanceChassis(Torque_Control, T_KP_NONE);
        break;
    case OFF: // 底盘关闭状态
    default:
        ControlBalanceChassis(Torque_Control, T_KP_NONE);
        break;
    }

    // 更新底盘状态
    if (chassis_state == JUMPING && (left_leg_pos.length > limit_value.leg_length_max - 0.01 ||
                                     right_leg_pos.length > limit_value.leg_length_max - 0.01))
    {
        chassis_state = MOVING;
    }
    // TODO:更新底盘状态
}