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
#include "INS_task.h"
#include "./Drives/MI_motor_drive.h"

#include "main.h"
#include "remote_control.h"
#include "bsp_buzzer.h"

float motorOutRatio = 1.0f; //电机输出电压比例，对所有电机同时有效

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

MI_Motor_s MI_Motor[5];
MI_Motor_s MI_Motor_None;

Chassis_IMU_t chassis_imu;
Motor_s left_joint[2], right_joint[2], left_wheel, right_wheel; //六个电机对象
Leg_Pos_t left_leg_pos, right_leg_pos; //左右腿部姿态
Leg_Pos_Target_t left_leg_pos_target, right_leg_pos_target;
State_Var_s state_var;
Target_s target = {0, 0, 0, 0, 0, 0, 0.07f};
GroundDetector ground_detector = {10, 10, 1, 0};
CascadePID leg_angle_PID, leg_length_PID; //腿部角度和长度控制PID
CascadePID yaw_PID, roll_PID; //机身yaw和roll控制PID

StandupState standup_state = StandupState_None;

/************** 通用函数 **************/

/**
  * @brief          获取从系统运行开始经过的时间，默认情况下单位为ms；
  */
uint32_t GetMillis()
{
  return HAL_GetTick();
}

/************** 电机模块 **************/
/**
  * @brief          初始化一个电机对象
  * @attention      
  * @note           
  * @param          motor 电机结构体地址
  * @param          MI_Motor 小米电机结构体地址
  * @param          offsetAngle 
  * @param          max_voltage 
  * @param          torque_ratio 
  * @param          dir 
  */
void MotorInit(Motor_s *motor, MI_Motor_s* MI_Motor, CAN_HandleTypeDef *hcan, uint8_t motor_id, float initial_angle, float vertical_angle,float horizontal_angle ,float upper_limit_angle,float lower_limit_angle, float max_voltage, float torque_ratio, float dir)//, float (*calcRevVolt)(float speed))
{
  motor->MI_Motor = MI_Motor;
  MI_motor_Init(MI_Motor,hcan,motor_id);
  motor->speed = motor->angle = motor->voltage = 0;
  // motor->offsetAngle = offsetAngle;
  motor->initial_angle = initial_angle;
  motor->vertical_angle = vertical_angle;
  motor->horizontal_angle = horizontal_angle;
  motor->upper_limit_angle = upper_limit_angle;
  motor->lower_limit_angle = lower_limit_angle;
  motor->max_voltage = max_voltage;
  motor->torque_ratio = torque_ratio;
  motor->dir = dir;
  // motor->calcRevVolt = calcRevVolt;
}


/**
  * @brief          2006电机反电动势计算函数(输入速度，输出反电动势)
  * @attention      系数需要自行测量并拟合，目前还没测量
  * @note           测量并拟合出不同电压下对应的电机空载转速，调换自变量和因变量就是本函数。
  * @note           由于该测量方法忽略阻力对空载转速的影响，最终抵消反电动势时也会抵消大部分电机本身的阻力。
  * @param          speed 速度
  */
float Motor_CalcRevVolt2006(float speed)
{
  return 0.000004f * speed * speed * speed - 0.0003f * speed * speed + 0.0266f * speed;
}




/**
  * @brief          初始化所有电机对象
  * @attention      各个参数需要通过实际测量或拟合得到，目前还没拟合
  */
void MotorInitAll()
{
  HAL_Delay(10);
  MotorInit(&left_joint[0],&MI_Motor[1],&MI_CAN_1,1, 
             -0.00019175051420461386f, 
             -1.9656345844268799f,
             -1.9656345844268799f + M_PI_2,
             2.184230089187622,
             -0.14477163553237915,
             7, 0.0316f, -1);
  
  HAL_Delay(10);
  MotorInit(&left_joint[1],&MI_Motor[2],&MI_CAN_1,2,
             -0.00019175051420461386f, 
             1.6960333585739136f, 
             1.6960333585739136f + M_PI_2, 
             2.7893948554992676,
             0.5413116812705994,
             7, 0.0317f, 1);

  MotorInit(&left_wheel,&MI_Motor_None,&MI_CAN_1,0,
             0, 
             0, 
             0, 
             0,
             0,
             4.0f, 0.0096f, 1);
  
  HAL_Delay(10);
  MotorInit(&right_joint[0],&MI_Motor[3],&MI_CAN_1,3,
             -0.00019175051420461386f, 
             1.8908518552780151f, 
             1.8908518552780151f - M_PI_2, 
             -2.181162118911743,
             -4.348710060119629,
             7, 0.0299f, -1);

  HAL_Delay(10);
  MotorInit(&right_joint[1],&MI_Motor[4],&MI_CAN_1,4,
             -0.00019175051420461386f, 
             -1.6588337421417236f, 
             -1.6588337421417236f - M_PI_2, 
             -0.9044871926307678,
             -3.201658248901367,
             7, 0.0321f, -1);

  MotorInit(&right_wheel,&MI_Motor_None,&MI_CAN_1,0,
             0, 
             0, 
             0,
             0,
             0,
             4.0f, 0.0101f, 1);
}


/**
  * @brief          设置电机扭矩
  */
void MotorSetTorque(Motor_s *motor, float torque)
{
  motor->torque = torque;
}

/**
  * @brief          设置电机目标角度
  */
void MotorSetTargetAngle(Motor_s *motor, float target_angle)
{
  motor->target_angle = target_angle;
}


/**
  * @brief          2006电机力矩到电流值的映射
  * @note           
  * @param          torque 力矩大小
  * @return         send_velue 电流值大小
  */
int16_t MotorTorqueToCurrentValue_2006(float torque)
{
    float k = 0.18f;//N*m/A
    float current; //A
    current = torque/k;
    int16_t send_velue = (int16_t)(1000*current);
    return send_velue;
}


/******* 底盘姿态模块 *******/

/**
  * @brief          底盘姿态更新
  * @note           通过INS模块获取机体底盘的姿态数据
  */
void ChassisPostureUpdate()
{
  chassis_imu.yaw = get_INS_angle_point()[0];
  chassis_imu.pitch = get_INS_angle_point()[1];
  chassis_imu.roll = get_INS_angle_point()[2];

  chassis_imu.yawSpd = get_gyro_data_point()[0];
  chassis_imu.pitchSpd = get_gyro_data_point()[1];
  chassis_imu.rollSpd = get_gyro_data_point()[2];

  chassis_imu.zAccel = get_accel_data_point()[2];
}


/******* 运动控制模块 *******/
/**
 * @brief          计算关节与正方向水平面的夹角
 * @return         none
*/
float CalcJointAngle(Motor_s* left_joint, Motor_s* right_joint)
{
  left_joint[0].angle = left_joint[0].horizontal_angle - left_joint[0].MI_Motor->RxCAN_info.angle;
  left_joint[1].angle = left_joint[1].horizontal_angle - left_joint[1].MI_Motor->RxCAN_info.angle;
  right_joint[0].angle = right_joint[0].MI_Motor->RxCAN_info.angle - right_joint[0].horizontal_angle;
  right_joint[1].angle = right_joint[1].MI_Motor->RxCAN_info.angle - right_joint[1].horizontal_angle;
}
// {
//   joint_motor->angle = joint_motor->MI_Motor->RxCAN_info.angle - joint_motor->horizontal_angle;
// }

/**
  * @brief          目标量更新
  * @attention      从遥控器获取目标值
  * @note           根据目标量(target)计算实际控制算法的给定量
  */
void TargetUpdate()
{

};

/**
  * @brief          控制算法给定量更新任务
  * @attention      这里作为一个被调用的任务函数，而非FreeRTOS任务
  * @note           根据目标量(target)计算实际控制算法的给定量
  */
void CtrlTargetUpdateTask()
{
  //通过遥控器设定速度
  const RC_ctrl_t * rc_ctrl = get_remote_control_point();
  //设置前进速度
  target.speed_cmd = 0.25f + rc_ctrl->rc.ch[1]/660.0f*0.7;//其中第一个量为速度修正量，因为重心问题在初始状态下并不能稳定站在原地。
  target.speed = target.speed_cmd;
  //设置旋转速度
  target.yaw_speed_cmd = rc_ctrl->rc.ch[2]/660.0f;
  target.yaw_speed = target.yaw_speed_cmd;


  //=====控制给定量=====
  // float speed_slope_step = 0.003f;
  // //根据当前腿长计算速度斜坡步长(腿越短越稳定，加减速斜率越大)
  // float leg_length = (left_leg_pos.length + right_leg_pos.length) / 2;
  // speed_slope_step = -(leg_length - 0.07f) * 0.02f + 0.002f;

  // //计算速度斜坡，斜坡值更新到target.speed
  // if(fabs(target.speed_cmd - target.speed) < speed_slope_step)
  //   target.speed = target.speed_cmd;
  // else
  // {
  //   if(target.speed_cmd - target.speed > 0)
  //     target.speed += speed_slope_step;
  //   else
  //     target.speed -= speed_slope_step;
  // }

  // //计算位置目标，并限制在当前位置的±0.1m内
  // target.position += target.speed * 0.004f;
  // if(target.position - state_var.x > 0.1f)
  //   target.position = state_var.x + 0.1f; 
  // else if(target.position - state_var.x < -0.1f)
  //   target.position = state_var.x - 0.1f;

  // //限制速度目标在当前速度的±0.3m/s内
  // if(target.speed - state_var.dx > 0.3f)
  //   target.speed = state_var.dx + 0.3f;
  // else if(target.speed - state_var.dx < -0.3f)
  //   target.speed = state_var.dx - 0.3f;

  // //计算yaw方位角目标
  // target.yaw_angle += target.yaw_speed_cmd * 0.004f;
    
}


/**
  * @brief          腿部姿态更新任务
  * @note           根据关节电机数据计算腿部姿态
  */
void LegPosUpdateTask()
{
  //更新关节信息
  // CalcJointAngle(&left_joint[0]);
  // CalcJointAngle(&left_joint[1]);
  // CalcJointAngle(&right_joint[0]);
  // CalcJointAngle(&right_joint[1]);
  CalcJointAngle(left_joint, right_joint);
  left_joint[0].speed = left_joint[0].MI_Motor->RxCAN_info.speed;
  left_joint[1].speed = left_joint[1].MI_Motor->RxCAN_info.speed;
  right_joint[0].speed = right_joint[0].MI_Motor->RxCAN_info.speed;
  right_joint[1].speed = right_joint[1].MI_Motor->RxCAN_info.speed;

  const float lpf_ratio = 0.5f; //低通滤波系数(新值的权重)
  float last_left_dLength = 0, last_right_dLength = 0;

  float legPos[2], legSpd[2];


  //计算左腿位置
  LegPos(left_joint[1].angle, left_joint[0].angle, legPos);
  left_leg_pos.length = legPos[0];
  left_leg_pos.angle = legPos[1];

  //计算左腿速度
  LegSpd(left_joint[1].speed, left_joint[0].speed, left_joint[1].angle, left_joint[0].angle, legSpd);
  left_leg_pos.dLength = legSpd[0];
  left_leg_pos.dAngle = legSpd[1];

  //计算左腿腿长加速度
  left_leg_pos.ddLength = ((left_leg_pos.dLength - last_left_dLength) * 1000 / 4) * lpf_ratio + left_leg_pos.ddLength * (1 - lpf_ratio);
  last_left_dLength = left_leg_pos.dLength;

  //计算右腿位置
  LegPos(right_joint[1].angle, right_joint[0].angle, legPos);
  right_leg_pos.length = legPos[0];
  right_leg_pos.angle = legPos[1];

  //计算右腿速度
  LegSpd(right_joint[1].speed, right_joint[0].speed, right_joint[1].angle, right_joint[0].angle, legSpd);
  right_leg_pos.dLength = legSpd[0];
  right_leg_pos.dAngle = legSpd[1];

  //计算右腿腿长加速度
  right_leg_pos.ddLength = ((right_leg_pos.dLength - last_right_dLength) * 1000 / 4) * lpf_ratio + right_leg_pos.ddLength * (1 - lpf_ratio);
  last_right_dLength = right_leg_pos.dLength;

}


/**
  * @brief          站立准备任务
  * @attention      目前看来就是一个劈叉任务，没有实际价值
  * @note           将机器人从任意姿态调整到准备站立前的劈叉状态
  */
void CtrlStandupPrepareTask(void *arg)
{
  standup_state = StandupState_Prepare;

  //将左腿向后摆
  MotorSetTorque(&left_joint[0], 0.2f);
  MotorSetTorque(&left_joint[1], 0.2f);
  while(left_leg_pos.angle < M_3PI_4)
    vTaskDelay(5);
  MotorSetTorque(&left_joint[0], 0);
  MotorSetTorque(&left_joint[1], 0);
  vTaskDelay(1000);

  //将右腿向前摆
  MotorSetTorque(&right_joint[0], -0.2f);
  MotorSetTorque(&right_joint[1], -0.2f);
  while(right_leg_pos.angle > M_PI_4)
    vTaskDelay(5);
  MotorSetTorque(&right_joint[0], 0);
  MotorSetTorque(&right_joint[1], 0);
  vTaskDelay(1000);

  //完成准备动作，关闭电机结束任务
  MotorSetTorque(&left_joint[0], 0);
  MotorSetTorque(&left_joint[1], 0);
  MotorSetTorque(&left_wheel, 0);
  MotorSetTorque(&right_joint[0], 0);
  MotorSetTorque(&right_joint[1], 0);
  MotorSetTorque(&right_wheel, 0);
  
  standup_state = StandupState_Standup;

  vTaskDelete(NULL);
}


/**
  * @brief          PID部分初始化
  * @note           
  */
void PIDInit()
{
  //初始化各个PID参数
  // PID_Init();

  PID_Init(&yaw_PID.inner, 0.01, 0, 0, 0, 0.1);
  PID_Init(&yaw_PID.outer, 10, 0, 0, 0, 2);
  PID_Init(&roll_PID.inner, 1, 0, 5, 0, 5);
  PID_Init(&roll_PID.outer, 20, 0, 0, 0, 3);
  PID_SetErrLpfRatio(&roll_PID.inner, 0.1f);
  PID_Init(&leg_length_PID.inner, 10.0f, 1, 30.0f, 2.0f, 10.0f);
  PID_Init(&leg_length_PID.outer, 5.0f, 0, 0.0f, 0.0f, 0.5f);
  PID_SetErrLpfRatio(&leg_length_PID.inner, 0.5f);
  PID_Init(&leg_angle_PID.inner, 0.04, 0, 0, 0, 1);
  PID_Init(&leg_angle_PID.outer, 12, 0, 0, 0, 20);
  PID_SetErrLpfRatio(&leg_angle_PID.outer, 0.5f);
}


/**
  * @brief          延迟us
  * @param[in]      us:延时时间
  * @note           
  */
void nop_delay_us(uint16_t us)
{
  for(; us > 0; us--)
  {
    for(uint8_t i = 10; i > 0; i--)
    {
      __nop();
      __nop();
      __nop();
      __nop();
      __nop();
      __nop();
      __nop();
      __nop();
      __nop();
      __nop();
      __nop();
      __nop();
      __nop();
      __nop();
      __nop();
    }
  }
}
