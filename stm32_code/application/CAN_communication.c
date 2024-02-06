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
    HAL_CAN_RxFifo0MsgPendingCallback(hcan);
}
void HAL_CAN_RxFifo1FullCallback(CAN_HandleTypeDef *hcan)
{
    HAL_CAN_RxFifo1MsgPendingCallback(hcan);
    HAL_CAN_RxFifo1MsgPendingCallback(hcan);
}


/**
  * @brief          反馈数据解码
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
                left_wheel.speed = -motor_chassis[6].speed_rpm*M_PI/30*REDUCTION_RATIO_2006;//反馈的是转子速度，乘减速比得到轮子转速
                right_wheel.angle = motor_chassis[7].ecd/8191.0f*M_PI*2;
                right_wheel.speed = motor_chassis[7].speed_rpm*M_PI/30*REDUCTION_RATIO_2006;
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
            // OutputData.data_3 = RxCAN_info_type_0.motor_id;
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
            
            if (RxCAN_info_type_17.motor_id == 1) left_joint[0].rx_torque = RxCAN_info_type_17.param;
            if (RxCAN_info_type_17.motor_id == 2) left_joint[1].rx_torque = RxCAN_info_type_17.param;
            if (RxCAN_info_type_17.motor_id == 3) right_joint[0].rx_torque = RxCAN_info_type_17.param;
            if (RxCAN_info_type_17.motor_id == 4) right_joint[1].rx_torque = RxCAN_info_type_17.param;
        }


    }
}
