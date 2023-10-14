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

//Function declaration

static void CANRxDecode(CAN_RxHeaderTypeDef rx_header,uint8_t rx_data[8]);


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
    uint8_t rx_data[8];

    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);//获取CAN数据
    CANRxDecode(rx_header,rx_data);//CAN数据解码
}
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &rx_header, rx_data);//获取CAN数据
    CANRxDecode(rx_header,rx_data);//CAN数据解码
}

/**
  * @brief          hal库CAN回调函数,处理CAN接收溢出
  * @param[in]      hcan:CAN句柄指针
  * @retval         none
  */
void HAL_CAN_RxFifo0FullCallback(CAN_HandleTypeDef *hcan)
{
    HAL_CAN_RxFifo0MsgPendingCallback(hcan);
}
void HAL_CAN_RxFifo1FullCallback(CAN_HandleTypeDef *hcan)
{
    HAL_CAN_RxFifo1MsgPendingCallback(hcan);
}


/**
  * @brief          反馈数据解码
  * @param[in]      left_wheel: (0x207) 2006电机控制电流, 范围 [-10000,10000]
  * @param[in]      right_wheel: (0x208) 2006电机控制电流, 范围 [-10000,10000]
  * @retval         none
  */
static void CANRxDecode(CAN_RxHeaderTypeDef rx_header,uint8_t rx_data[8])
{
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
        RxCAN_info_s RxCAN_info;//用于存储小米电机反馈的数据
        memcpy(&RxCAN_info,&rx_header.ExtId,4);//将扩展标识符的内容解码成对应内容

        uint16_t decode_temp_mi;//小米电机反馈数据解码缓冲
        if(RxCAN_info.communication_type == 0){//通信类型0的反馈帧解码
            RxCAN_info_type_0_s RxCAN_info_type_0;
            memcpy(&RxCAN_info_type_0,&rx_header.ExtId,4);//将扩展标识符的内容解码成通信类型0的对应内容
            memcpy(&RxCAN_info_type_0.MCU_id,rx_data,8);//获取MCU标识符
            OutputData.data_3 = RxCAN_info_type_0.motor_id;
        }else if(RxCAN_info.communication_type == 2){//通信类型2的反馈帧解码
            RxCAN_info_type_2_s RxCAN_info_type_2;
            memcpy(&RxCAN_info_type_2,&rx_header.ExtId,4);//将扩展标识符的内容解码成通信类型2的对应内容
            MI_motor_RxDecode(&RxCAN_info_type_2,rx_data);//通信类型2的数据解码

            MI_Motor[RxCAN_info_type_2.motor_id].RxCAN_info = RxCAN_info_type_2;
            MI_Motor[RxCAN_info_type_2.motor_id].motor_mode_state = RxCAN_info_type_2.mode_state;

        }else if(RxCAN_info.communication_type == 17){//通信类型17的反馈帧解码
            RxCAN_info_type_17_s RxCAN_info_type_17;
            memcpy(&RxCAN_info_type_17,&rx_header.ExtId,4);//将扩展标识符的内容解码成通信类型17的对应内容
            memcpy(&RxCAN_info_type_17.index,&rx_data[0],2);//获取查找的参数索引码
            memcpy(&RxCAN_info_type_17.param,&rx_data[4],4);//获取查找的参数信息
        }


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
    float upper_limit_torque = 1.0;//N*m
    float lower_limit_torque = -1.0;//N*m
    for (int i=0;i<2;i++){
        if(left_joint[i].torque > upper_limit_torque) left_joint[i].torque = upper_limit_torque;
        else if(left_joint[i].torque < lower_limit_torque) left_joint[i].torque = lower_limit_torque;

        if(right_joint[i].torque > upper_limit_torque) right_joint[i].torque = upper_limit_torque;
        else if(right_joint[i].torque < lower_limit_torque) right_joint[i].torque = lower_limit_torque;
    }
    
    //发送关节控制力矩
    MI_motor_TorqueControl(left_joint[0].MI_Motor,left_joint[0].torque);
    HAL_Delay(1);
    MI_motor_TorqueControl(left_joint[1].MI_Motor,left_joint[1].torque);

    HAL_Delay(1);

    MI_motor_TorqueControl(right_joint[0].MI_Motor,right_joint[0].torque);
    HAL_Delay(1);
    MI_motor_TorqueControl(right_joint[1].MI_Motor,right_joint[1].torque);

    //发送车轮控制力矩
    CANCmdWheel(0,0);
}


