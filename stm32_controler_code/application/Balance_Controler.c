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

#define M_PI    3.14159265358979323846

float motorOutRatio = 1.0f; //电机输出电压比例，对所有电机同时有效

/************** 电机模块 **************/
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


/**
  * @brief          6020电机反电动势计算函数(输入速度，输出反电动势)
  * @attention      系数需要自行测量并拟合，目前还没测量
  * @note           测量并拟合出不同电压下对应的电机空载转速，调换自变量和因变量就是本函数。
  * @note           由于该测量方法忽略阻力对空载转速的影响，最终抵消反电动势时也会抵消大部分电机本身的阻力。
  * @author         小企鹅
  * @param          speed 速度
  */
float Motor_CalcRevVolt6020(float speed)
{
  return 0.00008f * speed * speed * speed - 0.0035f * speed * speed + 0.2322f * speed;
}


/**
  * @brief          3508电机反电动势计算函数(输入速度，输出反电动势)
  * @attention      系数需要自行测量并拟合，目前还没测量
  * @note           测量并拟合出不同电压下对应的电机空载转速，调换自变量和因变量就是本函数。
  * @note           由于该测量方法忽略阻力对空载转速的影响，最终抵消反电动势时也会抵消大部分电机本身的阻力。
  * @author         小企鹅
  * @param          speed 速度
  */
float Motor_CalcRevVolt3508(float speed)
{
  return 0.000004f * speed * speed * speed - 0.0003f * speed * speed + 0.0266f * speed;
}


/**
  * @brief          初始化所有电机对象
  * @attention      各个参数需要通过实际测量或拟合得到，目前还没拟合
  * @author         小企鹅
  */
void Motor_InitAll()
{
  Motor_Init(&leftJoint[0], 1.431, 7, 0.0316f, -1, Motor_CalcRevVolt6020);
  Motor_Init(&leftJoint[1], -7.76, 7, 0.0317f, 1, Motor_CalcRevVolt6020);
  Motor_Init(&leftWheel, 0, 4.0f, 0.0096f, 1, Motor_CalcRevVolt3508);
  Motor_Init(&rightJoint[0], 0.343, 7, 0.0299f, -1, Motor_CalcRevVolt6020);
  Motor_Init(&rightJoint[1], -2.446, 7, 0.0321f, -1, Motor_CalcRevVolt6020);
  Motor_Init(&rightWheel, 0, 4.0f, 0.0101f, 1, Motor_CalcRevVolt3508);
}


/**
  * @brief          设置电机扭矩
  * @author         小企鹅
  */
void Motor_SetTorque(Motor_s *motor, float torque)
{
  motor->torque = torque;
}


/**
  * @brief          从CAN总线接收到的数据中解析出电机角度和速度
  * @attention      后续会根据自己的设备条件调整函数内容
  * @author         小企鹅
  */
void Motor_Update(Motor_s *motor, uint8_t *data)
{
  motor->angle = (*(int32_t *)&data[0] / 1000.0f - motor->offsetAngle) * motor->dir;
  motor->speed = (*(int16_t *)&data[4] / 10 * 2 * M_PI / 60) * motor->dir;
}


/**
  * @brief          由设置的目标扭矩和当前转速计算补偿反电动势后的驱动输出电压，并进行限幅
  * @note           补偿的意义: 电机转速越快反电动势越大，需要加大驱动电压来抵消反电动势，使电流(扭矩)不随转速发生变化
  * @author         小企鹅
  */
void Motor_UpdateVoltage(Motor_s *motor)
{
  float voltage = motor->torque / motor->torqueRatio * motorOutRatio;
  if (motor->speed >= 0)
    voltage += motor->calcRevVolt(motor->speed);
  else if (motor->speed < 0)
    voltage -= motor->calcRevVolt(-motor->speed);
  if (voltage > motor->maxVoltage)
    voltage = motor->maxVoltage;
  else if (voltage < -motor->maxVoltage)
    voltage = -motor->maxVoltage;
  motor->voltage = voltage * motor->dir;
}


/******* 运动控制模块 *******/

/**
  * @brief          目标量更新任务
  * @note           这里作为一个被调用的任务函数，而非FreeRTOS任务
  * @note           根据目标量(target)计算实际控制算法的给定量
  * @author         小企鹅
  */
void Ctrl_TargetUpdateTask(void *arg)
{
  // TickType_t xLastWakeTime = xTaskGetTickCount();

  float speedSlopeStep = 0.003f;
  // while (1)
  // {
    //根据当前腿长计算速度斜坡步长(腿越短越稳定，加减速斜率越大)
    float legLength = (leftLegPos.length + rightLegPos.length) / 2;
    speedSlopeStep = -(legLength - 0.07f) * 0.02f + 0.002f;

    //计算速度斜坡，斜坡值更新到target.speed
    if(fabs(target.speedCmd - target.speed) < speedSlopeStep)
      target.speed = target.speedCmd;
    else
    {
      if(target.speedCmd - target.speed > 0)
        target.speed += speedSlopeStep;
      else
        target.speed -= speedSlopeStep;
    }

    //计算位置目标，并限制在当前位置的±0.1m内
    target.position += target.speed * 0.004f;
    if(target.position - stateVar.x > 0.1f)
      target.position = stateVar.x + 0.1f; 
    else if(target.position - stateVar.x < -0.1f)
      target.position = stateVar.x - 0.1f;

    //限制速度目标在当前速度的±0.3m/s内
    if(target.speed - stateVar.dx > 0.3f)
      target.speed = stateVar.dx + 0.3f;
    else if(target.speed - stateVar.dx < -0.3f)
      target.speed = stateVar.dx - 0.3f;

    //计算yaw方位角目标
    target.yawAngle += target.yawSpeedCmd * 0.004f;
    
    // vTaskDelayUntil(&xLastWakeTime, 4); //每4ms更新一次
  // }
}


/**
  * @brief          腿部姿态更新任务
  * @note           根据关节电机数据计算腿部姿态
  * @note           其中MATLAB生成的函数重新将l1-5作为变量加入后导出了新的函数
  * @author         小企鹅
  */
void LegPos_UpdateTask(void *arg)
{
  const float lpfRatio = 0.5f; //低通滤波系数(新值的权重)
  float lastLeftDLength = 0, lastRightDLength = 0;
  // TickType_t xLastWakeTime = xTaskGetTickCount();
  // while (1)
  // {
    float legPos[2], legSpd[2];

    //计算左腿位置
    leg_pos(leftJointLength.l1, leftJointLength.l2, leftJointLength.l3, leftJointLength.l4, leftJointLength.l5,
            leftJoint[1].angle, leftJoint[0].angle, legPos
            );
    leftLegPos.length = legPos[0];
    leftLegPos.angle = legPos[1];

    //计算左腿速度
    leg_spd(leftJoint[1].speed, leftJoint[0].speed, 
            leftJointLength.l1, leftJointLength.l2, leftJointLength.l3, leftJointLength.l4, leftJointLength.l5,
            leftJoint[1].angle, leftJoint[0].angle, legSpd);
    leftLegPos.dLength = legSpd[0];
    leftLegPos.dAngle = legSpd[1];

    //计算左腿腿长加速度
    leftLegPos.ddLength = ((leftLegPos.dLength - lastLeftDLength) * 1000 / 4) * lpfRatio + leftLegPos.ddLength * (1 - lpfRatio);
    lastLeftDLength = leftLegPos.dLength;

    //计算右腿位置
            
    leg_pos(rightJointLength.l1, rightJointLength.l2, rightJointLength.l3, rightJointLength.l4, rightJointLength.l5,
            rightJoint[1].angle, rightJoint[0].angle, legPos
            );
    rightLegPos.length = legPos[0];
    rightLegPos.angle = legPos[1];

    //计算右腿速度
    leg_spd(rightJoint[1].speed, rightJoint[0].speed, 
            rightJointLength.l1, rightJointLength.l2, rightJointLength.l3, rightJointLength.l4, rightJointLength.l5,
            rightJoint[1].angle, rightJoint[0].angle, legSpd);
    rightLegPos.dLength = legSpd[0];
    rightLegPos.dAngle = legSpd[1];

    //计算右腿腿长加速度
    rightLegPos.ddLength = ((rightLegPos.dLength - lastRightDLength) * 1000 / 4) * lpfRatio + rightLegPos.ddLength * (1 - lpfRatio);
    lastRightDLength = rightLegPos.dLength;

    // vTaskDelayUntil(&xLastWakeTime, 4); //每4ms更新一次
  // }
}


void =现在改到这里了;
/**
  * @todo           将函数转换为c函数，并将起立状态改为双腿竖直向下
  * 
  * @brief          站立准备任务
  * @note           将机器人从任意姿态调整到准备站立前的劈叉状态
  * @author         小企鹅
  */
void Ctrl_StandupPrepareTask(void *arg)
{
  standupState = StandupState_Prepare;

  //将左腿向后摆
  Motor_SetTorque(&leftJoint[0], 0.2f);
  Motor_SetTorque(&leftJoint[1], 0.2f);
  while(leftLegPos.angle < M_3PI_4)
    vTaskDelay(5);
  Motor_SetTorque(&leftJoint[0], 0);
  Motor_SetTorque(&leftJoint[1], 0);
  vTaskDelay(1000);

  //将右腿向前摆
  Motor_SetTorque(&rightJoint[0], -0.2f);
  Motor_SetTorque(&rightJoint[1], -0.2f);
  while(rightLegPos.angle > M_PI_4)
    vTaskDelay(5);
  Motor_SetTorque(&rightJoint[0], 0);
  Motor_SetTorque(&rightJoint[1], 0);
  vTaskDelay(1000);

  //完成准备动作，关闭电机结束任务
  Motor_SetTorque(&leftJoint[0], 0);
  Motor_SetTorque(&leftJoint[1], 0);
  Motor_SetTorque(&leftWheel, 0);
  Motor_SetTorque(&rightJoint[0], 0);
  Motor_SetTorque(&rightJoint[1], 0);
  Motor_SetTorque(&rightWheel, 0);
  
  standupState = StandupState_Standup;

  vTaskDelete(NULL);
}


/**
  * @todo           将函数转换为c函数
  * 
  * @brief          主控制任务
  * @note           
  * @author         小企鹅
  */
void Ctrl_Task(void *arg)
{
  const float wheelRadius = 0.026f; //m，车轮半径
  const float legMass = 0.05f; //kg，腿部质量

  TickType_t xLastWakeTime = xTaskGetTickCount();

  //手动为反馈矩阵和输出叠加一个系数，用于手动优化控制效果
  float kRatio[2][6] = {{1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f},
                        {1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f}};
  float lqrTpRatio = 1.0f, lqrTRatio = 1.0f;

  //设定初始目标值
  target.rollAngle = 0.0f;
  target.legLength = 0.07f;
  target.speed = 0.0f;
  target.position = (leftWheel.angle + rightWheel.angle) / 2 * wheelRadius;

  while (1)
  {
    //计算状态变量
    stateVar.phi = chassis_imu.pitch;
    stateVar.dPhi = chassis_imu.pitchSpd;
    stateVar.x = (leftWheel.angle + rightWheel.angle) / 2 * wheelRadius;
    stateVar.dx = (leftWheel.speed + rightWheel.speed) / 2 * wheelRadius;
    stateVar.theta = (leftLegPos.angle + rightLegPos.angle) / 2 - M_PI_2 - chassis_imu.pitch;
    stateVar.dTheta = (leftLegPos.dAngle + rightLegPos.dAngle) / 2 - chassis_imu.pitchSpd;
    float legLength = (leftLegPos.length + rightLegPos.length) / 2;
    float dLegLength = (leftLegPos.dLength + rightLegPos.dLength) / 2;

    //如果正在站立准备状态，则不进行后续控制
    if(standupState == StandupState_Prepare)
    {
      vTaskDelayUntil(&xLastWakeTime, 4);
      continue;
    }

    //计算LQR反馈矩阵
    float kRes[12] = {0}, k[2][6] = {0};
    lqr_k(legLength, kRes);
    if(groundDetector.isTouchingGround) //正常触地状态
    {
      for (int i = 0; i < 6; i++)
      {
        for (int j = 0; j < 2; j++)
          k[j][i] = kRes[i * 2 + j] * kRatio[j][i];
      }
    }
    else //腿部离地状态，手动修改反馈矩阵，仅保持腿部竖直
    {
      memset(k, 0, sizeof(k));
      k[1][0] = kRes[1] * -2;
      k[1][1] = kRes[3] * -10;
    }

    //准备状态变量
    float x[6] = {stateVar.theta, stateVar.dTheta, stateVar.x, stateVar.dx, stateVar.phi, stateVar.dPhi};
    //与给定量作差
    x[2] -= target.position;
    x[3] -= target.speed;

    //矩阵相乘，计算LQR输出
    float lqrOutT = k[0][0] * x[0] + k[0][1] * x[1] + k[0][2] * x[2] + k[0][3] * x[3] + k[0][4] * x[4] + k[0][5] * x[5];
    float lqrOutTp = k[1][0] * x[0] + k[1][1] * x[1] + k[1][2] * x[2] + k[1][3] * x[3] + k[1][4] * x[4] + k[1][5] * x[5];

    //计算yaw轴PID输出
    PID_CascadeCalc(&yawPID, target.yawAngle, chassis_imu.yaw, chassis_imu.yawSpd);
    
    //设定车轮电机输出扭矩，为LQR和yaw轴PID输出的叠加
    if(groundDetector.isTouchingGround) //正常接地状态
    {
      Motor_SetTorque(&leftWheel, -lqrOutT * lqrTRatio - yawPID.output);
      Motor_SetTorque(&rightWheel, -lqrOutT * lqrTRatio + yawPID.output);
    }
    else //腿部离地状态，关闭车轮电机
    {
      Motor_SetTorque(&leftWheel, 0);
      Motor_SetTorque(&rightWheel, 0);
    }

    //根据离地状态修改目标腿长，并计算腿长PID输出
    PID_CascadeCalc(&legLengthPID, (groundDetector.isTouchingGround && !groundDetector.isCuchioning) ? target.legLength : 0.12f, legLength, dLegLength);
    //计算roll轴PID输出
    PID_CascadeCalc(&rollPID, target.rollAngle, chassis_imu.roll, chassis_imu.rollSpd);
    //根据离地状态计算左右腿推力，若离地则不考虑roll轴PID输出和前馈量
    float leftForce = legLengthPID.output + ((groundDetector.isTouchingGround && !groundDetector.isCuchioning) ? 6-rollPID.output : 0);
    float rightForce = legLengthPID.output + ((groundDetector.isTouchingGround && !groundDetector.isCuchioning) ? 6+rollPID.output : 0);
    if(leftLegPos.length > 0.12f) //保护腿部不能伸太长
      leftForce -= (leftLegPos.length - 0.12f) * 100;
    if(rightLegPos.length > 0.12f)
      rightForce -= (rightLegPos.length - 0.12f) * 100;
    
    //计算左右腿的地面支持力
    groundDetector.leftSupportForce = leftForce + legMass * 9.8f - legMass * (leftLegPos.ddLength - chassis_imu.zAccel);
    groundDetector.rightSupportForce = rightForce + legMass * 9.8f - legMass * (rightLegPos.ddLength - chassis_imu.zAccel);
    //更新离地检测器数据
    static uint32_t lastTouchTime = 0;
    bool isTouchingGround = groundDetector.leftSupportForce > 3 && groundDetector.rightSupportForce > 3; //判断当前瞬间是否接地
    if(!isTouchingGround && millis() - lastTouchTime < 1000) //若上次触地时间距离现在不超过1s，则认为当前瞬间接地，避免弹跳导致误判
      isTouchingGround = true;
    if(!groundDetector.isTouchingGround && isTouchingGround) //判断转为接地状态，标记进入缓冲状态
    {
      target.position = stateVar.x;
      groundDetector.isCuchioning = true;
      lastTouchTime = millis();
    }
    if(groundDetector.isCuchioning && legLength < target.legLength) //缓冲状态直到腿长压缩到目标腿长结束
      groundDetector.isCuchioning = false;
    groundDetector.isTouchingGround = isTouchingGround;

    //计算左右腿角度差PID输出
    PID_CascadeCalc(&legAnglePID, 0, leftLegPos.angle - rightLegPos.angle, leftLegPos.dAngle - rightLegPos.dAngle);
    
    //计算髋关节扭矩输出，为LQR输出和左右腿角度差PID输出的叠加
    float leftTp = lqrOutTp * lqrTpRatio - legAnglePID.output * (leftLegPos.length / 0.07f);
    float rightTp = lqrOutTp * lqrTpRatio + legAnglePID.output * (rightLegPos.length / 0.07f);
    
    //使用VMC计算各关节电机输出扭矩
    float leftJointTorque[2]={0};
    leg_conv(leftForce, leftTp, leftJoint[1].angle, leftJoint[0].angle, leftJointTorque);
    float rightJointTorque[2]={0};
    leg_conv(rightForce, rightTp, rightJoint[1].angle, rightJoint[0].angle, rightJointTorque);
    
    //保护腿部角度不超限
    float leftTheta = leftLegPos.angle - chassis_imu.pitch - M_PI_2;
    float rightTheta = rightLegPos.angle - chassis_imu.pitch - M_PI_2;
    #define PROTECT_CONDITION (leftTheta < -M_PI_4 || leftTheta > M_PI_4 || \
                   rightTheta < -M_PI_4 || rightTheta > M_PI_4 || \
                   chassis_imu.pitch > M_PI_4 || chassis_imu.pitch < -M_PI_4) //腿部角度超限保护条件
    if(PROTECT_CONDITION) //当前达到保护条件
    {
      if(standupState == StandupState_None) //未处于起立过程中
      {
        //关闭所有电机
        Motor_SetTorque(&leftWheel, 0);
        Motor_SetTorque(&rightWheel, 0);
        Motor_SetTorque(&leftJoint[0], 0);
        Motor_SetTorque(&leftJoint[1], 0);
        Motor_SetTorque(&rightJoint[0], 0);
        Motor_SetTorque(&rightJoint[1], 0);
        //阻塞等待腿部角度回到安全范围，再等待4s后恢复控制(若中途触发了起立则在起立准备完成后直接跳出)
        while(PROTECT_CONDITION && standupState == StandupState_None)
        {
          leftTheta = leftLegPos.angle - chassis_imu.pitch - M_PI_2;
          rightTheta = rightLegPos.angle - chassis_imu.pitch - M_PI_2;
          vTaskDelay(100);
        }
        if(standupState == StandupState_None)
          vTaskDelay(4000);
        //退出保护后设定目标位置和yaw角度为当前值
        target.position = (leftWheel.angle + rightWheel.angle) / 2 * wheelRadius;
        target.yawAngle = chassis_imu.yaw;
        continue;
      }
      if(standupState == StandupState_Standup && (leftTheta < -M_PI_4 || rightTheta > M_PI_4))
        standupState = StandupState_None;
    }
    else
    {
      if(standupState == StandupState_Standup) //未达到保护条件且处于起立过程中，说明起立完成，退出起立过程
        standupState = StandupState_None;
    }

    //设定关节电机输出扭矩
    Motor_SetTorque(&leftJoint[0], -leftJointTorque[0]);
    Motor_SetTorque(&leftJoint[1], -leftJointTorque[1]);
    Motor_SetTorque(&rightJoint[0], -rightJointTorque[0]);
    Motor_SetTorque(&rightJoint[1], -rightJointTorque[1]);

    vTaskDelayUntil(&xLastWakeTime, 4); //4ms控制周期
  }
}

/**
  * @todo           将函数转换为c函数
  * 
  * @brief          控制模块初始化
  * @note           
  * @author         小企鹅
  */
void Ctrl_Init()
{
  //初始化各个PID参数
  PID_Init(&yawPID.inner, 0.01, 0, 0, 0, 0.1);
  PID_Init(&yawPID.outer, 10, 0, 0, 0, 2);
  PID_Init(&rollPID.inner, 1, 0, 5, 0, 5);
  PID_Init(&rollPID.outer, 20, 0, 0, 0, 3);
  PID_SetErrLpfRatio(&rollPID.inner, 0.1f);
  PID_Init(&legLengthPID.inner, 10.0f, 1, 30.0f, 2.0f, 10.0f);
  PID_Init(&legLengthPID.outer, 5.0f, 0, 0.0f, 0.0f, 0.5f);
  PID_SetErrLpfRatio(&legLengthPID.inner, 0.5f);
  PID_Init(&legAnglePID.inner, 0.04, 0, 0, 0, 1);
  PID_Init(&legAnglePID.outer, 12, 0, 0, 0, 20);
  PID_SetErrLpfRatio(&legAnglePID.outer, 0.5f);

  //触发各个控制任务
  xTaskCreate(Ctrl_TargetUpdateTask, "Ctrl_TargetUpdateTask", 4096, NULL, 3, NULL);
  xTaskCreate(LegPos_UpdateTask, "LegPos_UpdateTask", 4096, NULL, 2, NULL);
  vTaskDelay(2);
  xTaskCreate(Ctrl_Task, "Ctrl_Task", 4096, NULL, 1, NULL);
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