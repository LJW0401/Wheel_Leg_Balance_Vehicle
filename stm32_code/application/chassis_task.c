/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       chassis.c/h
  * @brief      chassis control task,
  *             底盘控制任务
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. add chassis power control
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
#include "chassis_task.h"
#include "Balance_Controler.h"
#include "CAN_communication.h"
#include "remote_control.h"
#include "usb_task.h"
// #include "chassis_behaviour.h"

// #include "cmsis_os.h"

// #include "arm_math.h"
// #include "pid.h"
// #include "remote_control.h"
// #include "CAN_receive.h"
// #include "detect_task.h"
// #include "INS_task.h"
// #include "chassis_power_control.h"

#define rc_deadband_limit(input, output, dealine)        \
    {                                                    \
        if ((input) > (dealine) || (input) < -(dealine)) \
        {                                                \
            (output) = (input);                          \
        }                                                \
        else                                             \
        {                                                \
            (output) = 0;                                \
        }                                                \
    }





//底盘运动数据
// chassis_move_t chassis_move;

/**
  * @brief          chassis task, osDelay CHASSIS_CONTROL_TIME_MS (2ms) 
  * @param[in]      pvParameters: null
  * @retval         none
  */
/**
  * @brief          底盘任务，间隔 CHASSIS_CONTROL_TIME_MS 2ms
  * @param[in]      pvParameters: 空
  * @retval         none
  */
void chassis_task(void const *pvParameters)
{
    //wait a time 
    //空闲一段时间
    vTaskDelay(CHASSIS_TASK_INIT_TIME);
    //chassis init
    //底盘初始化
    // chassis_init();

    HAL_Delay(1000);
    MotorInitAll();//初始化所有电机
    //HAL_Delay(2);
    MI_motor_Enable(&MI_Motor[1]);

    //HAL_Delay(2);
    MI_motor_Enable(&MI_Motor[2]);

    HAL_Delay(2);
    MI_motor_Enable(&MI_Motor[3]);

    //HAL_Delay(2);
    MI_motor_Enable(&MI_Motor[4]);

    //HAL_Delay(1000);

    //VMC解算时用到的PID控制器
    PID left_length_pid;
    PID_Init(&left_length_pid,50,0,10,100,100);
    PID left_angle_pid;
    PID_Init(&left_angle_pid,0.05,0.000,0.02,100,100);

    //2023.10.12左腿电机出问题了，先调右腿
    PID right_length_pid;
    PID_Init(&right_length_pid,50,0.01,10,100,100);
    PID right_angle_pid;
    PID_Init(&right_angle_pid,0.1,0.00001,0.08,100,100);
    //PID_Init(&right_angle_pid,0.01,0.00,0.02,100,100);


    static uint32_t lastTouchTime = 0;

    const float wheelRadius = 0.0425f; //m，车轮半径
    const float legMass = 0.12368f; //kg，腿部质量

    // TickType_t xLastWakeTime = xTaskGetTickCount();

    //手动为反馈矩阵和输出叠加一个系数，用于手动优化控制效果
    float kRatio[2][6] = {{1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f},
                          {1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f}};
    float lqrTpRatio = 1.0f, lqrTRatio = 1.0f;

    //设定初始目标值
    target.roll_angle = 0.0f;
    target.leg_length = 0.17f;
    target.speed = 0.0f;
    target.position = (left_wheel.angle + right_wheel.angle) / 2 * wheelRadius;

    //追加的控制项，先站起来再说
    target.speed_cmd = 0.0f;
    target.yaw_speed_cmd = 0.0f;
    target.yaw_angle = 0.0f;


    while (1)
    {
        //1.更新腿部姿态信息
        LegPosUpdateTask();
        //2.更新底盘姿态信息
        ChassisPostureUpdate();
        //3.更新目标信息
        CtrlTargetUpdateTask();
        //4.进入控制状态
        const RC_ctrl_t* rc_ctrl = get_remote_control_point();


        if(left_joint[0].MI_Motor->motor_mode_state==RESET_MODE)
            MI_motor_Enable(left_joint[0].MI_Motor);
        if(left_joint[1].MI_Motor->motor_mode_state==RESET_MODE)
            MI_motor_Enable(left_joint[1].MI_Motor);
        if(right_joint[0].MI_Motor->motor_mode_state==RESET_MODE)
            MI_motor_Enable(right_joint[0].MI_Motor);
        if(right_joint[1].MI_Motor->motor_mode_state==RESET_MODE)
            MI_motor_Enable(right_joint[1].MI_Motor);


        if(rc_ctrl->rc.s[1] == 0x01){//[OFF档]初始姿态标定
            // float torque = 0.5;
            float torque = 0.0;
            MotorSetTorque(&left_joint[0], torque);
            MotorSetTorque(&left_joint[1], -torque);
            MotorSetTorque(&right_joint[0], -torque);
            MotorSetTorque(&right_joint[1], torque);

            if (rc_ctrl->rc.s[0] == 0x02){
                MI_motor_SetMechPositionToZero(left_joint[0].MI_Motor);
                MI_motor_SetMechPositionToZero(left_joint[1].MI_Motor);
                MI_motor_SetMechPositionToZero(right_joint[0].MI_Motor);
                MI_motor_SetMechPositionToZero(right_joint[1].MI_Motor);
            }
            
        }else if(rc_ctrl->rc.s[1] == 0x03){//[CL档]腿部伸长
            //伸长固定长度
            // MI_motor_LocationControl(left_joint[0].MI_Motor , -1.1 ,5,0.5);
            // MI_motor_LocationControl(left_joint[1].MI_Motor , +1 ,5,0.5);
            // MI_motor_LocationControl(right_joint[0].MI_Motor, +1.1 ,5,0.5);
            // MI_motor_LocationControl(right_joint[1].MI_Motor, -1 ,5,0.5);

            //摇杆控制
            // float loc[4];
            // for (int i=0;i<5;i++){
            //     loc[i] = rc_ctrl->rc.ch[i]/300.0f;
            // }
            // MI_motor_LocationControl(left_joint[0].MI_Motor , loc[0] ,5,0.5);
            // MI_motor_LocationControl(left_joint[1].MI_Motor , loc[1] ,5,0.5);
            // MI_motor_LocationControl(right_joint[0].MI_Motor, loc[2] ,5,0.5);
            // MI_motor_LocationControl(right_joint[1].MI_Motor, loc[3] ,5,0.5);

            //VMC解算
            // float target_length = 0.17f;
            float target_length = 0.16f;
            target_length -= rc_ctrl->rc.ch[1]/16500.0f;
            float target_angle = 1.5f;
            target_angle -= rc_ctrl->rc.ch[0]/1320.0f;
            float feedback;

            //左腿
            //这样正常，不知道为什么
            feedback = target_length - left_leg_pos.length;
            PID_SingleCalc(&left_length_pid, 0, feedback);
            feedback = target_angle - left_leg_pos.angle;
            PID_SingleCalc(&left_angle_pid,  0, feedback);
            //这样会抽，不知道为什么
            // PID_SingleCalc(&left_length_pid, target_length, left_leg_pos.length);
            // PID_SingleCalc(&left_angle_pid,  target_angle, left_leg_pos.angle);

            //右腿
            //这样会抽，不知道为什么
            // feedback = target_length - right_leg_pos.length;
            // PID_SingleCalc(&right_length_pid, 0, feedback);
            // feedback = target_angle - right_leg_pos.angle;
            // PID_SingleCalc(&right_angle_pid,  0, feedback);
            //这样正常，不知道为什么
            PID_SingleCalc(&right_length_pid, target_length, right_leg_pos.length);
            PID_SingleCalc(&right_angle_pid,  target_angle, right_leg_pos.angle);



            float F_left_leg =  -left_length_pid.output;
            float Tp_left_leg = left_angle_pid.output;

            float F_right_leg =  -right_length_pid.output;
            float Tp_right_leg = right_angle_pid.output;

            float left_joint_torque[2];
            float right_joint_torque[2];
            LegConv(F_left_leg, Tp_left_leg, left_joint[1].angle, left_joint[0].angle, left_joint_torque);
            LegConv(F_right_leg, Tp_right_leg, right_joint[1].angle, right_joint[0].angle, right_joint_torque);

            for (int i=0;i<2;i++){
                float limit_torque = 0.7;
                if (left_joint_torque[i] > limit_torque)MotorSetTorque(&left_joint[i], limit_torque);
                else if (left_joint_torque[i] < -limit_torque)MotorSetTorque(&left_joint[i], -limit_torque);
                else MotorSetTorque(&left_joint[i], left_joint_torque[i]);

                if (right_joint_torque[i] > limit_torque)MotorSetTorque(&right_joint[i], limit_torque);
                else if (right_joint_torque[i] < -limit_torque)MotorSetTorque(&right_joint[i], -limit_torque);
                else MotorSetTorque(&right_joint[i], right_joint_torque[i]);
            }

            // MI_motor_TorqueControl(left_joint[0].MI_Motor,left_joint[0].torque);
            // MI_motor_TorqueControl(left_joint[1].MI_Motor,left_joint[1].torque);
            // MI_motor_TorqueControl(right_joint[0].MI_Motor,right_joint[0].torque);
            // MI_motor_TorqueControl(right_joint[1].MI_Motor,right_joint[1].torque);

        }else if(rc_ctrl->rc.s[1] == 0x02){//[HL档]平衡控制

            //计算状态变量
            state_var.phi = chassis_imu.pitch;
            state_var.dPhi = chassis_imu.pitchSpd;
            state_var.x = (left_wheel.angle + right_wheel.angle) / 2 * wheelRadius;
            state_var.dx = (left_wheel.speed + right_wheel.speed) / 2 * wheelRadius;
            state_var.theta = (left_leg_pos.angle + right_leg_pos.angle) / 2 - M_PI_2 - chassis_imu.pitch;
            state_var.dTheta = (left_leg_pos.dAngle + right_leg_pos.dAngle) / 2 - chassis_imu.pitchSpd;
            float leg_length = (left_leg_pos.length + right_leg_pos.length) / 2;
            float dLegLength = (left_leg_pos.dLength + right_leg_pos.dLength) / 2;

            //计算LQR反馈矩阵
            float kRes[12] = {0}, k[2][6] = {0};
            LQR_K(leg_length, kRes);

            //准备状态变量
            float x[6] = {state_var.theta, state_var.dTheta, state_var.x, state_var.dx, state_var.phi, state_var.dPhi};
            //与给定量作差
            x[2] -= target.position;
            x[3] -= target.speed;

            //矩阵相乘，计算LQR输出
            float lqrOutT = k[0][0] * x[0] + k[0][1] * x[1] + k[0][2] * x[2] + k[0][3] * x[3] + k[0][4] * x[4] + k[0][5] * x[5];
            float lqrOutTp = k[1][0] * x[0] + k[1][1] * x[1] + k[1][2] * x[2] + k[1][3] * x[3] + k[1][4] * x[4] + k[1][5] * x[5];

            //计算yaw轴PID输出
            PID_CascadeCalc(&yaw_PID, target.yaw_angle, chassis_imu.yaw, chassis_imu.yawSpd);

            //设定车轮电机输出扭矩，为LQR和yaw轴PID输出的叠加
            MotorSetTorque(&left_wheel, -lqrOutT * lqrTRatio - yaw_PID.output);
            MotorSetTorque(&right_wheel, -lqrOutT * lqrTRatio + yaw_PID.output);

            // CANCmdWheel(
            //     MotorTorqueToCurrentValue_2006(left_wheel.torque), 
            //     MotorTorqueToCurrentValue_2006(right_wheel.torque)
            //     );

        }else{//其他状态一律关闭电机
            MotorSetTorque(&left_joint[0], 0);
            MotorSetTorque(&left_joint[1], 0);
            MotorSetTorque(&right_joint[0], 0);
            MotorSetTorque(&right_joint[1], 0);

            MotorSetTorque(&left_wheel, 0);
            MotorSetTorque(&right_wheel, 0);
        }
        
        SendChassisCmd();//发送底盘控制指令
        
        //os delay
        //系统延时
        vTaskDelay(CHASSIS_CONTROL_TIME_MS);
    }
}
