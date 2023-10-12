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

float motorOutRatio = 1.0f; //电机输出电压比例，对所有电机同时有效

extern CAN_HandleTypeDef hcan1;

MI_Motor_s MI_Motor[5];
MI_Motor_s MI_Motor_None;

Chassis_IMU_t chassis_imu;
Motor_s left_joint[2], right_joint[2], left_wheel, right_wheel; //六个电机对象
// Joint_Length_t left_jointLength =  {0.05f, 0.105f, 0.105f, 0.05f, 0.06f}; //关节长度
// Joint_Length_t right_jointLength = {0.05f, 0.105f, 0.105f, 0.05f, 0.06f}; //关节长度
Leg_Pos_t left_leg_pos, right_leg_pos; //左右腿部姿态
State_Var_s state_var;
Target_s target = {0, 0, 0, 0, 0, 0, 0.07f};
GroundDetector ground_detector = {10, 10, 1, 0};
CascadePID leg_angle_PID, leg_length_PID; //腿部角度和长度控制PID
CascadePID yaw_PID, roll_PID; //机身yaw和roll控制PID

StandupState standup_state = StandupState_None;

/************** 通用函数 **************/

/**
  * @brief          获取从系统运行开始经过的时间，默认情况下单位为ms；
  * @author         小企鹅
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
  * @author         小企鹅
  * @param          motor 电机结构体地址
  * @param          MI_Motor 小米电机结构体地址
  * @param          offsetAngle 
  * @param          max_voltage 
  * @param          torque_ratio 
  * @param          dir 
  */
void MotorInit(Motor_s *motor, MI_Motor_s* MI_Motor,uint8_t motor_id, float initial_angle, float vertical_angle,float horizontal_angle ,float upper_limit_angle,float lower_limit_angle, float max_voltage, float torque_ratio, float dir)//, float (*calcRevVolt)(float speed))
{
  motor->MI_Motor = MI_Motor;
  MI_motor_Init(MI_Motor,&MI_CAN_1,motor_id);
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
  * @author         小企鹅
  * @param          speed 速度
  */
float Motor_CalcRevVolt2006(float speed)
{
  return 0.000004f * speed * speed * speed - 0.0003f * speed * speed + 0.0266f * speed;
}




/**
  * @brief          初始化所有电机对象
  * @attention      各个参数需要通过实际测量或拟合得到，目前还没拟合
  * @author         小企鹅
  */
void MotorInitAll()
{
  HAL_Delay(1000);
  // MotorInit(&left_joint[0],&MI_Motor[1],1, 
  //            -0.00019175051420461386f, 
  //            -1.9656345844268799f,
  //            -1.9656345844268799f + M_PI_2,
  //            2.184230089187622,
  //            -0.14477163553237915,
  //            7, 0.0316f, -1);
  
  // HAL_Delay(100);
  // MotorInit(&left_joint[1],&MI_Motor[2],2,
  //            -0.00019175051420461386f, 
  //            1.6960333585739136f, 
  //            1.6960333585739136f + M_PI_2, 
  //            2.7893948554992676,
  //            0.5413116812705994,
  //            7, 0.0317f, 1);

  MotorInit(&left_wheel,&MI_Motor_None,0,
             0, 
             0, 
             0, 
             0,
             0,
             4.0f, 0.0096f, 1);
  
  HAL_Delay(100);
  MotorInit(&right_joint[0],&MI_Motor[3],3,
             -0.00019175051420461386f, 
             1.8908518552780151f, 
             1.8908518552780151f - M_PI_2, 
             -2.181162118911743,
             -4.348710060119629,
             7, 0.0299f, -1);

  HAL_Delay(100);
  MotorInit(&right_joint[1],&MI_Motor[4],4,
             -0.00019175051420461386f, 
             -1.6588337421417236f, 
             -1.6588337421417236f - M_PI_2, 
             -0.9044871926307678,
             -3.201658248901367,
             7, 0.0321f, -1);

  MotorInit(&right_wheel,&MI_Motor_None,0,
             0, 
             0, 
             0,
             0,
             0,
             4.0f, 0.0101f, 1);
}


/**
  * @brief          设置电机扭矩
  * @author         小企鹅
  */
void MotorSetTorque(Motor_s *motor, float torque)
{
  motor->torque = torque;
}


// /**
//   * @brief          从CAN总线接收到的数据中解析出电机角度和速度
//   * @attention      后续会根据自己的设备条件调整函数内容
//   * @author         小企鹅
//   */
// void Motor_Update(Motor_s *motor, uint8_t *data)
// {
//   motor->angle = (*(int32_t *)&data[0] / 1000.0f - motor->offsetAngle) * motor->dir;
//   motor->speed = (*(int16_t *)&data[4] / 10 * 2 * M_PI / 60) * motor->dir;
// }


/**
  * @brief          由设置的目标扭矩和当前转速计算补偿反电动势后的驱动输出电压，并进行限幅
  * @note           补偿的意义: 电机转速越快反电动势越大，需要加大驱动电压来抵消反电动势，使电流(扭矩)不随转速发生变化
  * @author         小企鹅
  */
// void Motor_UpdateVoltage(Motor_s *motor)
// {
//   float voltage = motor->torque / motor->torque_ratio * motorOutRatio;
//   if (motor->speed >= 0)
//     voltage += motor->calcRevVolt(motor->speed);
//   else if (motor->speed < 0)
//     voltage -= motor->calcRevVolt(-motor->speed);
//   if (voltage > motor->max_voltage)
//     voltage = motor->max_voltage;
//   else if (voltage < -motor->max_voltage)
//     voltage = -motor->max_voltage;
//   motor->voltage = voltage * motor->dir;
// }


/**
  * @brief          2006电机力矩到电流值的映射
  * @note           
  * @param          torque 力矩大小
  * @return         current 电流大小
  * @author         小企鹅
  */
uint16_t MotorTorqueToCurrentValue_2006(float torque)
{
    //a: 0.586563749263927, b: -1.95946924227303, c: 0.466670000000000
    float current_value;
    current_value = 0.586563749263927f*torque*torque - 1.95946924227303f*torque*torque + 0.466670000000000f*torque;
    return (uint16_t)current_value;
}


/******* 底盘姿态模块 *******/

/**
  * @brief          底盘姿态更新
  * @note           通过INS模块获取机体底盘的姿态数据
  * @author         小企鹅
  */
void ChassisPostureUpdate()
{
  const fp32* INS_angle_point = get_INS_angle_point();
  const fp32* INS_gyro_data_point = get_gyro_data_point();
  const fp32* INS_accel_data_point = get_accel_data_point();
  chassis_imu.yaw = INS_angle_point[0];
  chassis_imu.pitch = INS_angle_point[1];
  chassis_imu.roll = INS_angle_point[2];
  chassis_imu.yawSpd = INS_gyro_data_point[0];
  chassis_imu.pitchSpd = INS_gyro_data_point[1];
  chassis_imu.rollSpd = INS_gyro_data_point[2];
  chassis_imu.zAccel = INS_accel_data_point[2];
}


/******* 运动控制模块 *******/
/**
 * @brief          计算关节与正方向水平面的夹角
 * @return         none
*/
float CalcJointAngle(Motor_s* left_joint, Motor_s* right_joint)
{
  left_joint[0].angle = left_joint[0].horizontal_angle - left_joint[0].MI_Motor->RxCAN_info.angle;
  // left_joint[0].angle = M_1_PI - left_joint[0].angle;
  left_joint[1].angle = left_joint[1].horizontal_angle - left_joint[1].MI_Motor->RxCAN_info.angle;
  // left_joint[1].angle = M_1_PI - left_joint[1].angle;
  right_joint[0].angle = right_joint[0].MI_Motor->RxCAN_info.angle - right_joint[0].horizontal_angle;
  // right_joint[0].angle = M_1_PI - right_joint[0].angle;
  right_joint[1].angle = right_joint[1].MI_Motor->RxCAN_info.angle - right_joint[1].horizontal_angle;
  // right_joint[1].angle = M_1_PI - right_joint[1].angle;
}
// {
//   joint_motor->angle = joint_motor->MI_Motor->RxCAN_info.angle - joint_motor->horizontal_angle;
// }

/**
  * @brief          目标量更新
  * @attention      从遥控器获取目标值
  * @note           根据目标量(target)计算实际控制算法的给定量
  * @author         小企鹅
  */
void TargetUpdate()
{

};

/**
  * @brief          控制算法给定量更新任务
  * @attention      这里作为一个被调用的任务函数，而非FreeRTOS任务
  * @note           根据目标量(target)计算实际控制算法的给定量
  * @author         小企鹅
  */
void CtrlTargetUpdateTask()
{
  // TickType_t xLastWakeTime = xTaskGetTickCount();

  float speed_slope_step = 0.003f;
  // while (1)
  // {
    //根据当前腿长计算速度斜坡步长(腿越短越稳定，加减速斜率越大)
    float leg_length = (left_leg_pos.length + right_leg_pos.length) / 2;
    speed_slope_step = -(leg_length - 0.07f) * 0.02f + 0.002f;

    //计算速度斜坡，斜坡值更新到target.speed
    if(fabs(target.speed_cmd - target.speed) < speed_slope_step)
      target.speed = target.speed_cmd;
    else
    {
      if(target.speed_cmd - target.speed > 0)
        target.speed += speed_slope_step;
      else
        target.speed -= speed_slope_step;
    }

    //计算位置目标，并限制在当前位置的±0.1m内
    target.position += target.speed * 0.004f;
    if(target.position - state_var.x > 0.1f)
      target.position = state_var.x + 0.1f; 
    else if(target.position - state_var.x < -0.1f)
      target.position = state_var.x - 0.1f;

    //限制速度目标在当前速度的±0.3m/s内
    if(target.speed - state_var.dx > 0.3f)
      target.speed = state_var.dx + 0.3f;
    else if(target.speed - state_var.dx < -0.3f)
      target.speed = state_var.dx - 0.3f;

    //计算yaw方位角目标
    target.yaw_angle += target.yaw_speed_cmd * 0.004f;
    
    // vTaskDelayUntil(&xLastWakeTime, 4); //每4ms更新一次
  // }
}


/**
  * @brief          腿部姿态更新任务
  * @note           根据关节电机数据计算腿部姿态
  * @author         小企鹅
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
  // TickType_t xLastWakeTime = xTaskGetTickCount();
  // while (1)
  // {
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

    // vTaskDelayUntil(&xLastWakeTime, 4); //每4ms更新一次
  // }
}


/**
  * @brief          站立准备任务
  * @attention      目前看来就是一个劈叉任务，没有实际价值
  * @note           将机器人从任意姿态调整到准备站立前的劈叉状态
  * @author         小企鹅
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
  * @author         小企鹅
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

// /**
//   * @todo           将函数转换为c函数
//   * 
//   * @brief          主控制任务
//   * @attention      这一块应该整合到chassis_task中     
//   * @author         小企鹅
//   */
// void Ctrl_Task(void *arg)
// {
//   const float wheelRadius = 0.026f; //m，车轮半径
//   const float legMass = 0.05f; //kg，腿部质量

//   TickType_t xLastWakeTime = xTaskGetTickCount();

//   //手动为反馈矩阵和输出叠加一个系数，用于手动优化控制效果
//   float kRatio[2][6] = {{1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f},
//                         {1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f}};
//   float lqrTpRatio = 1.0f, lqrTRatio = 1.0f;

//   //设定初始目标值
//   target.rollAngle = 0.0f;
//   target.leg_length = 0.07f;
//   target.speed = 0.0f;
//   target.position = (left_wheel.angle + right_wheel.angle) / 2 * wheelRadius;

//   // while (1)
//   // {
//     //计算状态变量
//     state_var.phi = chassis_imu.pitch;
//     state_var.dPhi = chassis_imu.pitchSpd;
//     state_var.x = (left_wheel.angle + right_wheel.angle) / 2 * wheelRadius;
//     state_var.dx = (left_wheel.speed + right_wheel.speed) / 2 * wheelRadius;
//     state_var.theta = (left_leg_pos.angle + right_leg_pos.angle) / 2 - M_PI_2 - chassis_imu.pitch;
//     state_var.dTheta = (left_leg_pos.dAngle + right_leg_pos.dAngle) / 2 - chassis_imu.pitchSpd;
//     float leg_length = (left_leg_pos.length + right_leg_pos.length) / 2;
//     float dLegLength = (left_leg_pos.dLength + right_leg_pos.dLength) / 2;

//     //如果正在站立准备状态，则不进行后续控制
//     if(standup_state == StandupState_Prepare)
//     {
//       vTaskDelayUntil(&xLastWakeTime, 4);
//       continue;
//     }

//     //计算LQR反馈矩阵
//     float kRes[12] = {0}, k[2][6] = {0};
//     lqr_k(leg_length, kRes);
//     if(ground_detector.isTouchingGround) //正常触地状态
//     {
//       for (int i = 0; i < 6; i++)
//       {
//         for (int j = 0; j < 2; j++)
//           k[j][i] = kRes[i * 2 + j] * kRatio[j][i];
//       }
//     }
//     else //腿部离地状态，手动修改反馈矩阵，仅保持腿部竖直
//     {
//       memset(k, 0, sizeof(k));
//       k[1][0] = kRes[1] * -2;
//       k[1][1] = kRes[3] * -10;
//     }

//     //准备状态变量
//     float x[6] = {state_var.theta, state_var.dTheta, state_var.x, state_var.dx, state_var.phi, state_var.dPhi};
//     //与给定量作差
//     x[2] -= target.position;
//     x[3] -= target.speed;

//     //矩阵相乘，计算LQR输出
//     float lqrOutT = k[0][0] * x[0] + k[0][1] * x[1] + k[0][2] * x[2] + k[0][3] * x[3] + k[0][4] * x[4] + k[0][5] * x[5];
//     float lqrOutTp = k[1][0] * x[0] + k[1][1] * x[1] + k[1][2] * x[2] + k[1][3] * x[3] + k[1][4] * x[4] + k[1][5] * x[5];

//     //计算yaw轴PID输出
//     PID_CascadeCalc(&yaw_PID, target.yaw_angle, chassis_imu.yaw, chassis_imu.yawSpd);
    
//     //设定车轮电机输出扭矩，为LQR和yaw轴PID输出的叠加
//     if(ground_detector.isTouchingGround) //正常接地状态
//     {
//       MotorSetTorque(&left_wheel, -lqrOutT * lqrTRatio - yaw_PID.output);
//       MotorSetTorque(&right_wheel, -lqrOutT * lqrTRatio + yaw_PID.output);
//     }
//     else //腿部离地状态，关闭车轮电机
//     {
//       MotorSetTorque(&left_wheel, 0);
//       MotorSetTorque(&right_wheel, 0);
//     }

//     //根据离地状态修改目标腿长，并计算腿长PID输出
//     PID_CascadeCalc(&leg_length_PID, (ground_detector.isTouchingGround && !ground_detector.isCuchioning) ? target.leg_length : 0.12f, leg_length, dLegLength);
//     //计算roll轴PID输出
//     PID_CascadeCalc(&roll_PID, target.rollAngle, chassis_imu.roll, chassis_imu.rollSpd);
//     //根据离地状态计算左右腿推力，若离地则不考虑roll轴PID输出和前馈量
//     float leftForce = leg_length_PID.output + ((ground_detector.isTouchingGround && !ground_detector.isCuchioning) ? 6-roll_PID.output : 0);
//     float rightForce = leg_length_PID.output + ((ground_detector.isTouchingGround && !ground_detector.isCuchioning) ? 6+roll_PID.output : 0);
//     if(left_leg_pos.length > 0.12f) //保护腿部不能伸太长
//       leftForce -= (left_leg_pos.length - 0.12f) * 100;
//     if(right_leg_pos.length > 0.12f)
//       rightForce -= (right_leg_pos.length - 0.12f) * 100;
    
//     //计算左右腿的地面支持力
//     ground_detector.leftSupportForce = leftForce + legMass * 9.8f - legMass * (left_leg_pos.ddLength - chassis_imu.zAccel);
//     ground_detector.rightSupportForce = rightForce + legMass * 9.8f - legMass * (right_leg_pos.ddLength - chassis_imu.zAccel);
//     //更新离地检测器数据
//     static uint32_t lastTouchTime = 0;
//     // bool_t isTouchingGround = ground_detector.leftSupportForce > 3 && ground_detector.rightSupportForce > 3; //判断当前瞬间是否接地
//     uint8_t isTouchingGround = ground_detector.leftSupportForce > 3 && ground_detector.rightSupportForce > 3; //判断当前瞬间是否接地
//     if(!isTouchingGround && millis() - lastTouchTime < 1000) //若上次触地时间距离现在不超过1s，则认为当前瞬间接地，避免弹跳导致误判
//       // isTouchingGround = true;
//       isTouchingGround = 1;
//     if(!ground_detector.isTouchingGround && isTouchingGround) //判断转为接地状态，标记进入缓冲状态
//     {
//       target.position = state_var.x;
//       // ground_detector.isCuchioning = true;
//       ground_detector.isCuchioning = 1;
//       lastTouchTime = millis();
//     }
//     if(ground_detector.isCuchioning && leg_length < target.leg_length) //缓冲状态直到腿长压缩到目标腿长结束
//       // ground_detector.isCuchioning = false;
//       ground_detector.isCuchioning = 0;
//     ground_detector.isTouchingGround = isTouchingGround;

//     //计算左右腿角度差PID输出
//     PID_CascadeCalc(&leg_angle_PID, 0, left_leg_pos.angle - right_leg_pos.angle, left_leg_pos.dAngle - right_leg_pos.dAngle);
    
//     //计算髋关节扭矩输出，为LQR输出和左右腿角度差PID输出的叠加
//     float leftTp = lqrOutTp * lqrTpRatio - leg_angle_PID.output * (left_leg_pos.length / 0.07f);
//     float rightTp = lqrOutTp * lqrTpRatio + leg_angle_PID.output * (right_leg_pos.length / 0.07f);
    
//     //使用VMC计算各关节电机输出扭矩
//     float left_jointTorque[2]={0};
//     leg_conv(leftForce, leftTp, left_joint[1].angle, left_joint[0].angle, left_jointTorque);
//     float right_jointTorque[2]={0};
//     leg_conv(rightForce, rightTp, right_joint[1].angle, right_joint[0].angle, right_jointTorque);
    
//     //保护腿部角度不超限
//     float leftTheta = left_leg_pos.angle - chassis_imu.pitch - M_PI_2;
//     float rightTheta = right_leg_pos.angle - chassis_imu.pitch - M_PI_2;
//     #define PROTECT_CONDITION (leftTheta < -M_PI_4 || leftTheta > M_PI_4 || \
//                    rightTheta < -M_PI_4 || rightTheta > M_PI_4 || \
//                    chassis_imu.pitch > M_PI_4 || chassis_imu.pitch < -M_PI_4) //腿部角度超限保护条件
//     if(PROTECT_CONDITION) //当前达到保护条件
//     {
//       if(standup_state == StandupState_None) //未处于起立过程中
//       {
//         //关闭所有电机
//         MotorSetTorque(&left_wheel, 0);
//         MotorSetTorque(&right_wheel, 0);
//         MotorSetTorque(&left_joint[0], 0);
//         MotorSetTorque(&left_joint[1], 0);
//         MotorSetTorque(&right_joint[0], 0);
//         MotorSetTorque(&right_joint[1], 0);
//         //阻塞等待腿部角度回到安全范围，再等待4s后恢复控制(若中途触发了起立则在起立准备完成后直接跳出)
//         while(PROTECT_CONDITION && standup_state == StandupState_None)
//         {
//           leftTheta = left_leg_pos.angle - chassis_imu.pitch - M_PI_2;
//           rightTheta = right_leg_pos.angle - chassis_imu.pitch - M_PI_2;
//           vTaskDelay(100);
//         }
//         if(standup_state == StandupState_None)
//           vTaskDelay(4000);
//         //退出保护后设定目标位置和yaw角度为当前值
//         target.position = (left_wheel.angle + right_wheel.angle) / 2 * wheelRadius;
//         target.yaw_angle = chassis_imu.yaw;
//         continue;
//       }
//       if(standup_state == StandupState_Standup && (leftTheta < -M_PI_4 || rightTheta > M_PI_4))
//         standup_state = StandupState_None;
//     }
//     else
//     {
//       if(standup_state == StandupState_Standup) //未达到保护条件且处于起立过程中，说明起立完成，退出起立过程
//         standup_state = StandupState_None;
//     }

//     //设定关节电机输出扭矩
//     MotorSetTorque(&left_joint[0], -left_jointTorque[0]);
//     MotorSetTorque(&left_joint[1], -left_jointTorque[1]);
//     MotorSetTorque(&right_joint[0], -right_jointTorque[0]);
//     MotorSetTorque(&right_joint[1], -right_jointTorque[1]);

//     // vTaskDelayUntil(&xLastWakeTime, 4); //4ms控制周期
//   // }
// }



// //初始化
// void setup()
// {
//     // pinMode(9, INPUT_PULLUP); //按钮引脚
//     // pinMode(10, OUTPUT); //LED引脚

//     // //上电后等待5s
//     // digitalWrite(10, HIGH);
//     // vTaskDelay(5000);
//     // digitalWrite(10, LOW);

//     //初始化所有模块
//     Serial_Init();
//     ADC_Init();
//     CAN_Init();
//     IMU_Init();
//     Motor_InitAll();
//     Ctrl_Init();
//     BT_Init();
// }







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