/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       can_receive.c/h
  * @brief      there is CAN interrupt function  to receive motor data,
  *             and CAN send function to send motor current to control motor.
  *             这里是CAN中断接收函数，接收电机数据,CAN发送函数发送电机电流控制电机.
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. support hal lib
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include <math.h>

#include "CAN_communication.h"
#include "./Drives/MI_motor_drive.h"
#include "Balance_Controler.h"
#include "usb_task.h"
#include "cmsis_os.h"

#include "main.h"
#include "bsp_rng.h"

#include "remote_control.h"

// #include "detect_task.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
//motor data read
#define get_motor_measure(ptr, data)                                    \
    {                                                                   \
        (ptr)->last_ecd = (ptr)->ecd;                                   \
        (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);            \
        (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);      \
        (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);  \
        (ptr)->temperate = (data)[6];                                   \
    }
/*
motor data;
电机数据;
*/
static motor_measure_t motor_chassis[7];

// static CAN_TxHeaderTypeDef  wheel_tx_message;
// static uint8_t              wheel_can_send_data[8];
static CAN_TxHeaderTypeDef  wheel_tx_message;
static uint8_t              wheel_can_send_data[8];

/**
  * @brief          hal CAN fifo call back, receive motor data
  * @param[in]      hcan, the point to CAN handle
  * @retval         none
  */
/**
  * @brief          hal库CAN回调函数,接收电机数据
  * @param[in]      hcan:CAN句柄指针
  * @retval         none
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    RxCAN_info_s RxCAN_info;//用于存储小米电机反馈的数据
    uint8_t rx_data[8];

    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);//获取CAN数据


    if (rx_header.IDE == CAN_ID_STD) {//大疆2006电机解码
        switch (rx_header.StdId)
        {
            case CAN_3508_M1_ID:
            case CAN_3508_M2_ID:
            case CAN_3508_M3_ID:
            case CAN_3508_M4_ID:
            case CAN_YAW_MOTOR_ID:
            case CAN_PIT_MOTOR_ID:
            case CAN_LEFT_WHEEL_MOTOR_ID:
            case CAN_RIGHT_WHEEL_MOTOR_ID:
            {
                static uint8_t i = 0;
                //get motor id
                i = rx_header.StdId - CAN_3508_M1_ID;
                get_motor_measure(&motor_chassis[i], rx_data);
                // detect_hook(CHASSIS_MOTOR1_TOE + i);

                //这里还要改改，将2006电机数据获取后改为驱动轮电机的数据
                left_wheel.angle = motor_chassis[6].ecd/8191.0f*M_PI*2;
                left_wheel.speed = motor_chassis[6].speed_rpm*M_PI*2;
                right_wheel.angle = motor_chassis[7].ecd/8191.0f*M_PI*2;
                right_wheel.speed = motor_chassis[7].speed_rpm*M_PI*2;
                break;
            }

            default:
            {
                break;
            }
        }

    }else if (rx_header.IDE == CAN_ID_EXT) {//小米电机解码
        memcpy(&RxCAN_info,&rx_header.ExtId,29);//将扩展标识符的内容解码成对应内容

        uint16_t decode_temp_mi;//小米电机反馈数据解码缓冲
        if (RxCAN_info.communication_type == 2){//若为通信类型2的反馈帧就对应解码
          MI_motor_RxDecode(&RxCAN_info,rx_data);
        }

        if (RxCAN_info.motor_id == 1){
            MI_Motor_1.RxCAN_info = RxCAN_info;
        }else if (RxCAN_info.motor_id == 2){
            MI_Motor_2.RxCAN_info = RxCAN_info;
        }else if (RxCAN_info.motor_id == 3){
            MI_Motor_3.RxCAN_info = RxCAN_info;
        }else if (RxCAN_info.motor_id == 4){
            MI_Motor_4.RxCAN_info = RxCAN_info;
        }
        char_to_uint(OutputData.name_1,"M1_temp"); 
        OutputData.type_1 = 1;
        OutputData.data_1 = MI_Motor_1.RxCAN_info.angle;

        char_to_uint(OutputData.name_2,"M2_temp"); 
        OutputData.type_2 = 1;
        OutputData.data_2 = MI_Motor_2.RxCAN_info.angle;

        char_to_uint(OutputData.name_3,"M3_temp"); 
        OutputData.type_3 = 1;
        OutputData.data_3 = MI_Motor_3.RxCAN_info.angle;

        char_to_uint(OutputData.name_4,"M4_temp"); 
        OutputData.type_4 = 1;
        OutputData.data_4 = MI_Motor_4.RxCAN_info.angle;
    }

}

/**
  * @brief          发送驱动轮电机控制电流(0x205,0x206,0x207,0x208)
  * @param[in]      left_wheel: (0x207) 2006电机控制电流, 范围 [-10000,10000]
  * @param[in]      right_wheel: (0x208) 2006电机控制电流, 范围 [-10000,10000]
  * @retval         none
  */
void CANCmdWheel(int16_t left_wheel, int16_t right_wheel)
{
    uint32_t send_mail_box;
    wheel_tx_message.StdId = CAN_GIMBAL_ALL_ID;
    wheel_tx_message.IDE = CAN_ID_STD;
    wheel_tx_message.RTR = CAN_RTR_DATA;
    wheel_tx_message.DLC = 0x08;
    wheel_can_send_data[0] = 0;
    wheel_can_send_data[1] = 0;
    wheel_can_send_data[2] = 0;
    wheel_can_send_data[3] = 0;
    wheel_can_send_data[4] = (left_wheel >> 8);
    wheel_can_send_data[5] = left_wheel;
    wheel_can_send_data[6] = (right_wheel >> 8);
    wheel_can_send_data[7] = right_wheel;

    uint32_t free_TxMailbox = HAL_CAN_GetTxMailboxesFreeLevel(&WHEEL_CAN);//检测是否有空闲邮箱
    while (free_TxMailbox<3){//等待空闲邮箱数达到3
        free_TxMailbox = HAL_CAN_GetTxMailboxesFreeLevel(&WHEEL_CAN);
    }
    HAL_CAN_AddTxMessage(&WHEEL_CAN, &wheel_tx_message, wheel_can_send_data, &send_mail_box);
}

/**
  * @brief          发送控制信号
  * @retval         none
  */
void SendChassisCmd(void)
{
/*安全保护措施*/
    /*进行关节力矩限制,输出力矩不得超过限制范围*/
    float upper_limit_torque = 1.5;//N*m
    float lower_limit_torque = -1.5;//N*m

    if(left_joint[0].torque > upper_limit_torque){
        left_joint[0].torque = upper_limit_torque;
    }else if(left_joint[0].torque < lower_limit_torque){
        left_joint[0].torque = lower_limit_torque;
    }
    
    if(left_joint[1].torque > upper_limit_torque){
        left_joint[1].torque = upper_limit_torque;
    }else if(left_joint[1].torque < lower_limit_torque){
        left_joint[1].torque = lower_limit_torque;
    }

    if(right_joint[0].torque > upper_limit_torque){
        right_joint[0].torque = upper_limit_torque;
    }else if(right_joint[0].torque < lower_limit_torque){
        right_joint[0].torque = lower_limit_torque;
    }

    if(right_joint[1].torque > upper_limit_torque){
        right_joint[1].torque = upper_limit_torque;
    }else if(right_joint[1].torque < lower_limit_torque){
        right_joint[1].torque = lower_limit_torque;
    }
    /*对关节角度范围加以限制，不得超过各电机的角度范围*/
    if(left_leg_pos.angle > M_PI || left_leg_pos.angle < 0){
        left_joint[0].torque = 0;
        left_joint[1].torque = 0;
    }
    if(-right_leg_pos.angle > M_PI || -right_leg_pos.angle < 0){
        right_joint[0].torque = 0;
        right_joint[1].torque = 0;
    }
/*控制信号发送部分*/
    const RC_ctrl_t* rc_ctrl = get_remote_control_point();

    left_joint[0].torque = rc_ctrl->rc.ch[0]/660.0f;
    left_joint[1].torque = rc_ctrl->rc.ch[1]/660.0f;
    right_joint[0].torque = rc_ctrl->rc.ch[2]/660.0f;
    right_joint[1].torque = rc_ctrl->rc.ch[3]/660.0f;

    // left_joint[0].torque = 0;
    // left_joint[1].torque = 0;
    // right_joint[0].torque = 0;
    // right_joint[1].torque = 0;

    MI_motor_controlmode(left_joint[0].MI_Motor,left_joint[0].torque,0,0,0,0);
    MI_motor_controlmode(left_joint[1].MI_Motor,left_joint[1].torque,0,0,0,0);
    MI_motor_controlmode(right_joint[0].MI_Motor,right_joint[0].torque,0,0,0,0);
    MI_motor_controlmode(right_joint[1].MI_Motor,right_joint[1].torque,0,0,0,0);

    // CANCmdWheel(rc_ctrl->rc.ch[1],rc_ctrl->rc.ch[3]);
    // count_1++;
    // if (count_1>100){
    //     count_1 = 0;
    //     cmd = -cmd;
    // }
    int16_t ccc= 500;
    //CANCmdWheel(5000,5000);

    //发送关节控制力矩
    // MI_motor_controlmode(left_joint[0].MI_Motor,left_joint[0].torque,0,0,0,0);
    // for (i=0;i<10;i++){
    //     HAL_Delay(0);
    // }

    // MI_motor_controlmode(left_joint[1].MI_Motor,left_joint[1].torque,0,0,0,0);
    // for (i=0;i<10;i++){
    //     HAL_Delay(0);
    // }

    // MI_motor_controlmode(right_joint[0].MI_Motor,right_joint[0].torque,0,0,0,0);
    // for (i=0;i<10;i++){
    //     HAL_Delay(0);
    // }

    // MI_motor_controlmode(right_joint[1].MI_Motor,right_joint[1].torque,0,0,0,0);
    // for (i=0;i<100;i++){
    //     HAL_Delay(0);
    // }

    //发送车轮控制力矩
    // int16_t left_torque2current = (int16_t)(left_wheel.torque*6000);
    // int16_t right_torque2current = (int16_t)(right_wheel.torque*6000);
    // CANCmdWheel(
    //     left_torque2current,
    //     right_torque2current
    //             );


    // CANCmdWheel(
    //     rc_ctrl->rc.ch[1],
    //     rc_ctrl->rc.ch[3]
    //             );

    // CANCmdWheel(
    //     -500,
    //     500
    //             );
		
}


