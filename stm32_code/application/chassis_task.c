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

    MotorInitAll();//初始化所有电机

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


        if(rc_ctrl->rc.s[1] == 0x01){//[OFF0档]初始姿态标定（关闭电机）
            MotorSetTorque(&left_wheel, 0);
            MotorSetTorque(&right_wheel, 0);
            MotorSetTorque(&left_joint[0], 0);
            MotorSetTorque(&left_joint[1], 0);
            MotorSetTorque(&right_joint[0], 0);
            MotorSetTorque(&right_joint[1], 0);
        }else if(rc_ctrl->rc.s[1] == 0x03){//[CL档]手动控制平衡控制状态
            float left_wheel_torque = ((float)rc_ctrl->rc.ch[1])/660.0f;
            float right_wheel_torque = -((float)rc_ctrl->rc.ch[3])/660.0f;

            MotorSetTorque(&left_wheel, left_wheel_torque);
            MotorSetTorque(&right_wheel, right_wheel_torque);
            MotorSetTorque(&left_joint[0], 0.3);
            MotorSetTorque(&left_joint[1], -0.3);
            MotorSetTorque(&right_joint[0], 0.3);
            MotorSetTorque(&right_joint[1], -0.3);

        }else if(rc_ctrl->rc.s[1] == 0x02){//[HL档]正常控制状态
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
            if(ground_detector.is_touching_ground) //正常触地状态
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
            float x[6] = {state_var.theta, state_var.dTheta, state_var.x, state_var.dx, state_var.phi, state_var.dPhi};
            //与给定量作差
            x[2] -= target.position;
            x[3] -= target.speed;

            //矩阵相乘，计算LQR输出
            float lqr_out_T = k[0][0] * x[0] + k[0][1] * x[1] + k[0][2] * x[2] + k[0][3] * x[3] + k[0][4] * x[4] + k[0][5] * x[5];
            float lqr_out_Tp = k[1][0] * x[0] + k[1][1] * x[1] + k[1][2] * x[2] + k[1][3] * x[3] + k[1][4] * x[4] + k[1][5] * x[5];

            //计算yaw轴PID输出
            PID_CascadeCalc(&yaw_PID, target.yaw_angle, chassis_imu.yaw, chassis_imu.yawSpd);
            
            //设定车轮电机输出扭矩，为LQR和yaw轴PID输出的叠加
            if(ground_detector.is_touching_ground) //正常接地状态
            {
                MotorSetTorque(&left_wheel, -lqr_out_T * lqrTRatio - yaw_PID.output);
                MotorSetTorque(&right_wheel, -lqr_out_T * lqrTRatio + yaw_PID.output);
            }
            else //腿部离地状态，关闭车轮电机
            {
                MotorSetTorque(&left_wheel, 0);
                MotorSetTorque(&right_wheel, 0);
            }

            //根据离地状态修改目标腿长，并计算腿长PID输出
            PID_CascadeCalc(&leg_length_PID, (ground_detector.is_touching_ground && !ground_detector.is_cuchioning) ? target.leg_length : 0.12f, leg_length, dLegLength);
            //计算roll轴PID输出
            PID_CascadeCalc(&roll_PID, target.roll_angle, chassis_imu.roll, chassis_imu.rollSpd);
            //根据离地状态计算左右腿推力，若离地则不考虑roll轴PID输出和前馈量
            float leftForce = leg_length_PID.output + ((ground_detector.is_touching_ground && !ground_detector.is_cuchioning) ? 6-roll_PID.output : 0);
            float rightForce = leg_length_PID.output + ((ground_detector.is_touching_ground && !ground_detector.is_cuchioning) ? 6+roll_PID.output : 0);
            if(left_leg_pos.length > 0.12f) //保护腿部不能伸太长
            leftForce -= (left_leg_pos.length - 0.12f) * 100;
            if(right_leg_pos.length > 0.12f)
            rightForce -= (right_leg_pos.length - 0.12f) * 100;
            
            //计算左右腿的地面支持力
            ground_detector.left_support_force = leftForce + legMass * 9.8f - legMass * (left_leg_pos.ddLength - chassis_imu.zAccel);
            ground_detector.right_support_force = rightForce + legMass * 9.8f - legMass * (right_leg_pos.ddLength - chassis_imu.zAccel);
            //更新离地检测器数据
            uint8_t is_touching_ground = ground_detector.left_support_force > 3 && ground_detector.right_support_force > 3; //判断当前瞬间是否接地
            if(!is_touching_ground && (GetMillis() - lastTouchTime < 1000)) //若上次触地时间距离现在不超过1s，则认为当前瞬间接地，避免弹跳导致误判
                is_touching_ground = 1;
            if(!ground_detector.is_touching_ground && is_touching_ground) //判断转为接地状态，标记进入缓冲状态
            {
                target.position = state_var.x;
                ground_detector.is_cuchioning = 1;
                lastTouchTime = GetMillis();
            }
            if(ground_detector.is_cuchioning && leg_length < target.leg_length) //缓冲状态直到腿长压缩到目标腿长结束
                ground_detector.is_cuchioning = 0;
            ground_detector.is_touching_ground = is_touching_ground;

            //计算左右腿角度差PID输出
            PID_CascadeCalc(&leg_angle_PID, 0, left_leg_pos.angle - right_leg_pos.angle, left_leg_pos.dAngle - right_leg_pos.dAngle);
            
            //计算髋关节扭矩输出，为LQR输出和左右腿角度差PID输出的叠加
            float leftTp = lqr_out_Tp * lqrTpRatio - leg_angle_PID.output * (left_leg_pos.length / 0.07f);
            float rightTp = lqr_out_Tp * lqrTpRatio + leg_angle_PID.output * (right_leg_pos.length / 0.07f);
            
            //使用VMC计算各关节电机输出扭矩
            float left_joint_torque[2]={0};
            LegConv(leftForce, leftTp, left_joint[1].angle, left_joint[0].angle, left_joint_torque);
            float right_joint_torque[2]={0};
            LegConv(rightForce, rightTp, right_joint[1].angle, right_joint[0].angle, right_joint_torque);
/*添加角度限制，用等效直杆及2关节电机夹角极小值进行限制*/
            

            //设定关节电机输出扭矩
            // MotorSetTorque(&left_joint[0], -left_joint_torque[0]);
            // MotorSetTorque(&left_joint[1], -left_joint_torque[1]);
            // MotorSetTorque(&right_joint[0], -right_joint_torque[0]);
            // MotorSetTorque(&right_joint[1], -right_joint_torque[1]);

        }else{//其他状态一律关闭电机
            MotorSetTorque(&left_wheel, 0);
            MotorSetTorque(&right_wheel, 0);
            MotorSetTorque(&left_joint[0], 0);
            MotorSetTorque(&left_joint[1], 0);
            MotorSetTorque(&right_joint[0], 0);
            MotorSetTorque(&right_joint[1], 0);
        }
        
        SendChassisCmd();


        //os delay
        //系统延时
        vTaskDelay(CHASSIS_CONTROL_TIME_MS);
    }
}
