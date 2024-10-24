/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       chassis.c/h
  * @brief      chassis control task,
  *             底盘控制任务
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
  *  V1.1.0     Nov-11-2019     RM              1. add chassis power control
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H
#include "struct_typedef.h"
// #include "CAN_receive.h"
// #include "gimbal_task.h"
// #include "pid.h"
// #include "remote_control.h"
// #include "user_lib.h"

// //in the beginning of task ,wait a time
// //任务开始空闲一段时间
#define CHASSIS_TASK_INIT_TIME 357

// //底盘任务控制间隔 2ms
#define CHASSIS_CONTROL_TIME_MS 2
//底盘任务控制间隔 0.002s
#define CHASSIS_CONTROL_TIME 0.002f
//底盘任务控制频率，尚未使用这个宏
#define CHASSIS_CONTROL_FREQUENCE 500.0f



/**
  * @brief          底盘任务，间隔 CHASSIS_CONTROL_TIME_MS 2ms
  * @param[in]      pvParameters: 空
  * @retval         none
  */
extern void chassis_task(void const *pvParameters);

/**
 * @brief      机器人状态枚举量
 * @note       无
 */
typedef enum
{
    RobotState_OFF,          // 机器人关闭
    RobotState_MotorZeroing, // 电机零位校准
    RobotState_LegExtension, // 腿部伸展
    RobotState_Balance,
    RobotState_Stand,   // 站立
    RobotState_Moving,  // 移动
    RobotState_Jumping, // 跳跃
} Robot_State_e;

typedef struct
{
    float v_x;
    float v_y;
    float v_z;
    float v;
    float w_yaw;
    float w_pitch;
    float w_roll;
} Speed_t;
#endif
