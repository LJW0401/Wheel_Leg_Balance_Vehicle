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
#include "INS_task.h"
#include "bsp_buzzer.h"

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

    // HAL_Delay(1000);

    InitBalanceControler();//初始化平衡控制器

    Chassis_IMU_t chassis_IMU;//底盘IMU数据

    while (1)
    {
        chassis_IMU.yaw = get_INS_angle_point()[0];
        chassis_IMU.pitch = get_INS_angle_point()[1];
        chassis_IMU.roll = get_INS_angle_point()[2];

        chassis_IMU.yawSpd = get_gyro_data_point()[2];
        chassis_IMU.pitchSpd = get_gyro_data_point()[1];
        chassis_IMU.rollSpd = get_gyro_data_point()[0];

        chassis_IMU.xAccel = get_accel_data_point()[0];
        chassis_IMU.yAccel = get_accel_data_point()[1];
        chassis_IMU.zAccel = get_accel_data_point()[2];
        
        const RC_ctrl_t* rc_ctrl = get_remote_control_point();
        DataUpdate(
                &chassis_IMU,
                rc_ctrl->rc.ch[1]/660.0f*0.4,
                -rc_ctrl->rc.ch[2]/660.0f*0.005,
                0,
                0,
                0.185 + rc_ctrl->rc.ch[3]/660.0f*0.05
                );//更新数据

        Robot_State_e robot_state;
        robot_state = GetRobotState();//获取机器人状态

        if(rc_ctrl->rc.s[0] == 0x01){//GPS档急停
            left_wheel.torque = 0;
            right_wheel.torque = 0;
            
            ControlBalanceChassis(No_Control);
        }else{
            switch (rc_ctrl->rc.s[1])
            {
                case 0x01://[OFF档]初始姿态标定
                {
                    //使能未使能的电机
                    int i;
                    for (i=1;i<5;i++)
                        if(MI_Motor[i].motor_mode_state==RESET_MODE) MI_motor_Enable(&MI_Motor[i]);
                    
                    float torque = 0.5;
                    (&left_joint[0])->torque = torque;
                    (&left_joint[1])->torque = -torque;
                    (&right_joint[0])->torque = -torque;
                    (&right_joint[1])->torque = torque;

                    left_wheel.torque = 0;
                    right_wheel.torque = 0;

                    if (rc_ctrl->rc.s[0] == 0x02){//设置0位
                        if (robot_state != RobotState_MotorZeroing) buzzer_on(50, 3000);
                        else buzzer_on(1000, 3000);
                        SetCyberGearMechPositionToZero();
                    }else{
                        buzzer_on(1000, 3000);
                    }

                    const Leg_Pos_t *left_leg_pos,*right_leg_pos;
                    left_leg_pos  = GetLegPosPoint(0);
                    right_leg_pos = GetLegPosPoint(1);

                    //判断是否完成零位设置
                    if (left_leg_pos->length<0.11 && right_leg_pos->length<0.11)
                    {
                        SetRobotState(RobotState_MotorZeroing);
                    }

                    ControlBalanceChassis(Torque_Control);
                    
                    break;
                }
                case 0x03://[CL档]腿部伸长
                {
                    if (robot_state < RobotState_MotorZeroing) break;
                    SetRobotState(RobotState_LegExtension);
                    buzzer_on(500, 3000);

                    float joint_pos[2];
                    float length = 0.18f + rc_ctrl->rc.ch[1]/16500.0f;
                    float angle = M_PI_2 + rc_ctrl->rc.ch[0]/1350.0f;


                    JointPos(length,angle,joint_pos);//计算关节摆角
                    (&left_joint[1])->target_angle = joint_pos[0];
                    (&left_joint[0])->target_angle = joint_pos[1];

                    JointPos(length,angle,joint_pos);//计算关节摆角
                    (&right_joint[1])->target_angle = joint_pos[0];
                    (&right_joint[0])->target_angle = joint_pos[1];

                    left_wheel.torque = 0;
                    right_wheel.torque = 0;

                    ControlBalanceChassis(Location_Control);

                    break;
                }
                case 0x02://[HL档]平衡控制
                {
                    if (robot_state < RobotState_LegExtension) break;
                    SetRobotState(RobotState_Balance);

                    BalanceControlerCalc();

                    ControlBalanceChassis(Location_Control);

                    buzzer_on(300, 3000);

                    break;

                    // float res[2];
                    // LQRFeedbackCalc(k,x,res);//矩阵相乘，计算LQR输出
                    // float lqr_out_T = res[0];
                    // float lqr_out_Tp = res[1];

                    // //计算yaw轴PID输出
                    // PID_CascadeCalc(&yaw_PID, target.yaw_angle, chassis_imu.yaw, chassis_imu.yawSpd);
                    
                    // if(ground_detector.is_touching_ground) //正常接地状态
                    // {
                    //     //设定车轮电机输出扭矩，为LQR和yaw轴PID输出的叠加
                    //     MotorSetTorque(&left_wheel ,  lqr_out_T * lqr_T_ratio - yaw_PID.output);
                    //     MotorSetTorque(&right_wheel, -lqr_out_T * lqr_T_ratio - yaw_PID.output);
                    // }
                    // else //腿部离地状态，关闭车轮电机
                    // {
                    //     MotorSetTorque(&left_wheel, 0);
                    //     MotorSetTorque(&right_wheel, 0);
                    // }

                    // //根据离地状态修改目标腿长，并计算腿长PID输出
                    // PID_CascadeCalc(&leg_length_PID, 
                    //         (ground_detector.is_touching_ground && !ground_detector.is_cuchioning) ? target.leg_length : MAX_LEG_LENGTH, 
                    //         leg_length, dLegLength
                    //     );
                    // //计算roll轴PID输出
                    // PID_CascadeCalc(&roll_PID, target.roll_angle, chassis_imu.roll, chassis_imu.rollSpd);
                    // //根据离地状态计算左右腿推力，若离地则不考虑roll轴PID输出和前馈量
                    // float leftForce = leg_length_PID.output  + ((ground_detector.is_touching_ground && !ground_detector.is_cuchioning) ? +roll_PID.output : 0);
                    // float rightForce = leg_length_PID.output + ((ground_detector.is_touching_ground && !ground_detector.is_cuchioning) ? -roll_PID.output : 0);
                    
                    // if(left_leg_pos.length > MAX_LEG_LENGTH) //保护腿部不能伸太长
                    //     leftForce -= (left_leg_pos.length - MAX_LEG_LENGTH) * 100;
                    // if(right_leg_pos.length > MAX_LEG_LENGTH)
                    //     rightForce -= (right_leg_pos.length - MAX_LEG_LENGTH) * 100;
                    
                    // //计算左右腿的地面支持力
                    // ground_detector.left_support_force = leftForce + legMass * 9.8f - legMass * (left_leg_pos.ddLength - chassis_imu.zAccel);
                    // ground_detector.right_support_force = rightForce + legMass * 9.8f - legMass * (right_leg_pos.ddLength - chassis_imu.zAccel);
                    // //更新离地检测器数据
                    // static uint32_t lastTouchTime = 0;
                    // boolean is_touching_ground = ground_detector.left_support_force > 3 && ground_detector.right_support_force > 3; //判断当前瞬间是否接地
                    // if(!is_touching_ground && GetMillis() - lastTouchTime < 1000) //若上次触地时间距离现在不超过1s，则认为当前瞬间接地，避免弹跳导致误判
                    //     is_touching_ground = true;
                    // if(!ground_detector.is_touching_ground && is_touching_ground) //判断转为接地状态，标记进入缓冲状态
                    // {
                    //     target.position = state_var.x;
                    //     ground_detector.is_cuchioning = true;
                    //     lastTouchTime = GetMillis();
                    // }
                    // if(ground_detector.is_cuchioning && leg_length < target.leg_length) //缓冲状态直到腿长压缩到目标腿长结束
                    //     ground_detector.is_cuchioning = false;
                    // ground_detector.is_touching_ground = is_touching_ground;

                    // //计算左右腿角度差PID输出
                    // PID_CascadeCalc(&leg_delta_angle_PID, 0, left_leg_pos.angle - right_leg_pos.angle, left_leg_pos.dAngle - right_leg_pos.dAngle);
                    
                    // //计算髋关节扭矩输出，为LQR输出和左右腿角度差PID输出的叠加
                    // float delta_angle_ratio = 1.0f;
                    // float leftTp = lqr_out_Tp * lqr_Tp_ratio  + leg_delta_angle_PID.output * delta_angle_ratio;
                    // float rightTp = lqr_out_Tp * lqr_Tp_ratio - leg_delta_angle_PID.output * delta_angle_ratio;
                    
                    // //使用VMC计算各关节电机输出扭矩
                    // float leftJointTorque[2]={0};
                    // LegConv(leftForce, leftTp, left_joint[1].angle, left_joint[0].angle, leftJointTorque);
                    // float rightJointTorque[2]={0};
                    // LegConv(rightForce, rightTp, right_joint[1].angle, right_joint[0].angle, rightJointTorque);
                    

                    // buzzer_on(333, 3000);

                    // //设定关节电机输出扭矩
                    // Motorque(&right_joint[1], -rightJointTorque[1]);

                    break;
                }
                default://其他状态一律关闭电机
                {
                    left_wheel.torque = 0;
                    right_wheel.torque = 0;
                    ControlBalanceChassis(No_Control);
                    break;
                }
            }
        }

        
        
        //os delay
        //系统延时
        vTaskDelay(CHASSIS_CONTROL_TIME_MS);
    }
}
