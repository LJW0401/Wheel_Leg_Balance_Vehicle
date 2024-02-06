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
#include "detect_task.h"
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

// 底盘运动数据
//  chassis_move_t chassis_move;

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
    // wait a time
    // 空闲一段时间
    vTaskDelay(CHASSIS_TASK_INIT_TIME);
    // chassis init

    // HAL_Delay(1000);

    InitBalanceControler(); // 初始化平衡控制器

    Chassis_IMU_t chassis_IMU; // 底盘IMU数据
    static Robot_State_e robot_state = RobotState_OFF;

    const RC_ctrl_t *rc_ctrl = get_remote_control_point();

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

        OutputData.data_3 = left_joint[0].MI_Motor->RxCAN_info.torque;
        OutputData.data_4 = left_joint[1].MI_Motor->RxCAN_info.torque;
        OutputData.data_5 = right_joint[0].MI_Motor->RxCAN_info.torque;
        OutputData.data_6 = right_joint[1].MI_Motor->RxCAN_info.torque;
        
        OutputData.data_7 = left_joint[0].MI_Motor->RxCAN_info.angle;
        OutputData.data_8 = left_joint[1].MI_Motor->RxCAN_info.angle;
        OutputData.data_9 = right_joint[0].MI_Motor->RxCAN_info.angle;
        OutputData.data_10 = right_joint[1].MI_Motor->RxCAN_info.angle;

        float speed_target = rc_ctrl->rc.ch[1] / 660.0f * 0.4;
        float yaw_delta_target = 0;
        float pitch_target = 0;
        float roll_target = rc_ctrl->rc.ch[2] / 660.0f * 0.5;
        float length_target = 0.12 + rc_ctrl->rc.ch[3] / 660.0f * 0.12;
        float rotation_torque_target = rc_ctrl->rc.ch[0] / 660.0f * 0.03;

        DataUpdate(
            &chassis_IMU,
            speed_target,
            yaw_delta_target,
            pitch_target,
            roll_target,
            length_target,
            rotation_torque_target); // 更新数据

        if (toe_is_error(DBUS_TOE) || (rc_ctrl->rc.s[0] == 0x01))
        { // 遥控器信号丢失 或 GPS裆 急停
            left_wheel.torque = 0;
            right_wheel.torque = 0;
            (&left_joint[0])->torque = 0;
            (&left_joint[1])->torque = 0;
            (&right_joint[0])->torque = 0;
            (&right_joint[1])->torque = 0;
            ControlBalanceChassis(Torque_Control);
        }
        else
        {
            switch (rc_ctrl->rc.s[1])
            {
            case 0x01: //[OFF档]初始姿态标定
            {
                // 使能未使能的电机
                int i;
                for (i = 1; i < 5; i++)
                    if (MI_Motor[i].motor_mode_state == RESET_MODE)
                        MI_motor_Enable(&MI_Motor[i]);

                float torque = 1.0;
                (&left_joint[0])->torque = torque;
                (&left_joint[1])->torque = -torque;
                (&right_joint[0])->torque = -torque;
                (&right_joint[1])->torque = torque;

                left_wheel.torque = 0;
                right_wheel.torque = 0;

                if (rc_ctrl->rc.s[0] == 0x02) // ATTI 2
                {
                    if (robot_state != RobotState_MotorZeroing) // 设置0位
                        buzzer_on(50, 3000);
                    else
                        buzzer_on(1000, 3000);
                    SetCyberGearMechPositionToZero();
                }
                else
                {
                    buzzer_on(1000, 3000);
                }

                const Leg_Pos_t *left_leg_pos, *right_leg_pos;
                left_leg_pos = GetLegPosPoint(0);
                right_leg_pos = GetLegPosPoint(1);

                // 判断是否完成零位设置
                if (left_leg_pos->length < 0.11 && right_leg_pos->length < 0.11)
                {
                    robot_state = RobotState_MotorZeroing;
                }

                ControlBalanceChassis(Torque_Control);

                break;
            }
            case 0x03: //[CL档]腿部伸长
            {
                if (robot_state < RobotState_MotorZeroing)
                    break;
                robot_state = RobotState_LegExtension;
                buzzer_on(500, 3000);

                float joint_pos[2];
                float length = (0.24 + 0.13) / 2 + rc_ctrl->rc.ch[1] / 660.0 * (0.24 - 0.13) / 2;
                float angle = M_PI_2 + rc_ctrl->rc.ch[0] / 660.0 * (M_PI / 4);

                JointPos(length, angle, joint_pos); // 计算关节摆角
                (&left_joint[1])->target_angle = joint_pos[0];
                (&left_joint[0])->target_angle = joint_pos[1];

                JointPos(length, angle, joint_pos); // 计算关节摆角
                (&right_joint[1])->target_angle = joint_pos[0];
                (&right_joint[0])->target_angle = joint_pos[1];

                left_wheel.torque = 0;
                right_wheel.torque = 0;

                ControlBalanceChassis(Location_Control);

                break;
            }
            case 0x02: //[HL档]平衡控制
            {
                if (robot_state < RobotState_LegExtension)
                    break;
                robot_state = RobotState_Balance;

                if (rc_ctrl->rc.ch[4] < -500)
                {
                    BalanceChassisStateUpdate(JUMPING);
                }
                else if (abs(speed_target) < 0.001)
                {
                    BalanceChassisStateUpdate(STAND);
                }
                else
                {
                    BalanceChassisStateUpdate(MOVING);
                }

                BalanceControlerCalc();

                buzzer_on(300, 3000);

                break;
            }
            default: // 其他状态一律关闭电机
            {
                left_wheel.torque = 0;
                right_wheel.torque = 0;
                ControlBalanceChassis(No_Control);
                break;
            }
            }
        }

        // os delay
        // 系统延时
        vTaskDelay(CHASSIS_CONTROL_TIME_MS);
    }
}
