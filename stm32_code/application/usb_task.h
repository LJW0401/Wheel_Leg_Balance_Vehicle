/**
  ****************************(C) COPYRIGHT 2024 Polarbear*************************
  * @file       usb_task.c/h
  * @brief      usb outputs the IMU and gimbal data to the miniPC
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     2023-7-11       Penguin         1. done
  *  V1.0.1     Oct-31-2023     LihanChen       1. 完成核心框架的构建，使其与调试模式和 miniPC 模式兼容。
  *  V1.0.2     Nov-01-2023     LihanChen       1. 将 Append_CRC16_Check_Sum_SendPacketVision() 和 Append_CRC-16_Check_Stum_OutputPCData() 合并到 Append_CRC16_Check_Sum() 中
  *  v2.0.0     Feb-24-2024     LihanChen       1. 重构代码，将 usb_task 的发送和接收分离，并实现分包发送和接收视觉和导航数据
  *  v2.0.1     Mar-02-2024     LihanChen       1. 删除未使用的 USB_RECEIVE_STATE 定义，去除编译警告
  *  v2.1.0     Mar-04-2024     LihanChen       1. 添加裁判系统信息转发（保留接口，待填入真实数值）
  *  v2.1.1     Mar-24-2024     LihanChen       1. 简化Append_CRC16_Check_Sum()函数，移除usb_printf()函数
  *  v2.1.2     Mar-25-2024     Penguin         1. 优化了数据接收逻辑，添加了数据指针获取函数
  *                                             2. 通过CRC8_CRC16.c/h文件中的函数实现CRC8校验和CRC16校验
  *  v2.2.0     Apr-4-2024      Penguin         1. 添加了USB数据发送失败重发功能，保障了发送频率稳定
  *  v2.3.0     Apr-6-2024      Penguin         1. 添加了不同数据包的USB发送周期控制
  @verbatim
  =================================================================================
如果要添加一个新的接收数据包
    1.在usb_task.h中添加新的数据包结构体，如：
    typedef struct{
    ...; // 数据包内容
    }__attribute__((packed)) ReceivedPacketxxx_s;

    2.在usb_task.c中定义接收数据包的header，如：
    #define EXPECTED_INPUT_xxx_HEDER 0xxxx

    3.在usb_task.c中添加新的数据包接收函数，如：
    static void usb_receive_xxx(void)
    {
        uint8_t crc_ok = verify_CRC16_check_sum((uint8_t *)usb_rx_buf, sizeof(ReceivedPacketxxx_s));
        if (crc_ok)
        {
            memcpy(&ReceivedPacketxxx, usb_rx_buf, sizeof(ReceivedPacketxxx_s));
        }
    }

    4.在usb_receive()函数中添加新的header case和对应的接收函数

如果要添加一个新的发送数据包
    1.在usb_task.h中添加新的数据包结构体，如：
    typedef struct{
    ...; // 数据包内容
    }__attribute__((packed)) SendPacketxxx_s;

    2.在usb_task.c中定义发送数据包的header，如：
    #define SET_OUTPUT_xxx_HEDER 0xxxx

    3.在usb_task.h中的usb_send_duration_t添加新的数据包发送周期，如：
    typedef struct {
        ...; // 数据包发送周期
        uint8_t xxx;
    }usb_send_duration_t;


    4.在usb_task.c中添加新的数据包发送函数，如：
    static void usb_send_xxx(uint8_t t)
    {
        if (usb_send_duration.xxx < t)
        {
            usb_send_duration.xxx++;
            return;
        }
        usb_send_duration.xxx = 0;

        SendPacketxxx.header = SET_OUTPUT_xxx_HEDER; // 别放入 crc_ok
        ...; // 数据包内容
        append_CRC16_check_sum((uint8_t *)&SendPacketxxx, sizeof(SendPacketxxx));
        memcpy(usb_tx_buf, &SendPacketxxx, sizeof(SendPacketxxx_s));
        usb_send_data(sizeof(SendPacketxxx_s));
    }

    5.在usb_task()函数中添加新的header case和对应的发送函数
  =================================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear*************************
*/

#ifndef USB_TASK_H
#define USB_TASK_H

#include <stdint.h>
#include <stdbool.h>

#define PRESET_SELF_COLOR 1

typedef struct {
    uint8_t vision;
    uint8_t all_robot_hp;
    uint8_t game_status;
    uint8_t robot_status;
    uint8_t outputPC_data;
}usb_send_duration_t;//发送数据包的周期

typedef struct
{
    uint8_t header;
    bool tracking : 1;
    uint8_t id : 3;         // 0-outpost 6-guard 7-base
    uint8_t armors_num : 3; // 2-balance 3-outpost 4-normal
    uint8_t reserved : 1;
    float x;
    float y;
    float z;
    float yaw;
    float vx;
    float vy;
    float vz;
    float v_yaw;
    float r1;
    float r2;
    float dz;
    uint16_t checksum;
} __attribute__((packed)) ReceivedPacketVision_s;

typedef struct
{
    uint8_t header;
    bool stop_gimbal_scan;
    float chassis_spin_vel;
    uint16_t checksum;
} __attribute__((packed)) ReceivedPacketScanStatus_s;

typedef struct
{
    uint8_t header;
    uint8_t detect_color : 1; // 0-red 1-blue
    bool reset_tracker : 1;
    uint8_t fire : 1;
    uint8_t reserved : 5;
    float roll;
    float pitch;
    float yaw;
    float aim_x;
    float aim_y;
    float aim_z;
    uint16_t checksum;
} __attribute__((packed)) SendPacketVision_s;

typedef struct
{
    uint8_t header;
    float linear_x;
    float linear_y;
    float linear_z;
    float angular_x;
    float angular_y;
    float angular_z;
    uint16_t checksum;
} __attribute__((packed)) ReceivedPacketTwist_s;

typedef struct
{
    uint8_t header;
    uint16_t red_1_robot_hp;
    uint16_t red_2_robot_hp;
    uint16_t red_3_robot_hp;
    uint16_t red_4_robot_hp;
    uint16_t red_5_robot_hp;
    uint16_t red_7_robot_hp;
    uint16_t red_outpost_hp;
    uint16_t red_base_hp;
    uint16_t blue_1_robot_hp;
    uint16_t blue_2_robot_hp;
    uint16_t blue_3_robot_hp;
    uint16_t blue_4_robot_hp;
    uint16_t blue_5_robot_hp;
    uint16_t blue_7_robot_hp;
    uint16_t blue_outpost_hp;
    uint16_t blue_base_hp;
    uint16_t checksum;
} __attribute__((packed)) SendPacketAllRobotHP_s;

typedef struct
{
    uint8_t header;
    uint8_t game_progress;
    uint16_t stage_remain_time;
    uint16_t checksum;
} __attribute__((packed)) SendPacketGameStatus_s;

typedef struct
{
    uint8_t header;
    uint8_t robot_id;
    uint16_t current_hp;
    uint16_t shooter_heat;
    bool team_color; // 0-red 1-blue
    bool is_attacked;
    uint16_t checksum;
} __attribute__((packed)) SendPacketRobotStatus_s;

typedef struct
{
    uint8_t header;
    uint16_t ecd_set;
    uint16_t checksum;
} __attribute__((packed)) InputPCData_s;

typedef struct
{
    uint8_t header;
    uint16_t length;
    uint8_t name_1[10];
    uint8_t type_1;
    float data_1;
    uint8_t name_2[10];
    uint8_t type_2;
    float data_2;
    uint8_t name_3[10];
    uint8_t type_3;
    float data_3;
    uint8_t name_4[10];
    uint8_t type_4;
    float data_4;
    uint8_t name_5[10];
    uint8_t type_5;
    float data_5;
    uint8_t name_6[10];
    uint8_t type_6;
    float data_6;
    uint8_t name_7[10];
    uint8_t type_7;
    float data_7;
    uint8_t name_8[10];
    uint8_t type_8;
    float data_8;
    uint8_t name_9[10];
    uint8_t type_9;
    float data_9;
    uint8_t name_10[10];
    uint8_t type_10;
    float data_10;
    uint16_t checksum;
} __attribute__((packed)) OutputPCData_s;

extern OutputPCData_s OutputPCData;

extern void usb_task(void const *argument);

SendPacketVision_s *GetSendPacketVisionPoint(void);

const ReceivedPacketVision_s *GetReceivedPacketVisionPoint(void);

const ReceivedPacketTwist_s *GetReceivedPacketTwistPoint(void);

const ReceivedPacketScanStatus_s *GetReceivedPacketScanStatus(void);

#endif
