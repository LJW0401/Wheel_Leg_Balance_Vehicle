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

#include "usb_task.h"

#include <stdio.h>
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "usbd_conf.h"
#include "cmsis_os.h"
#include "bsp_buzzer.h"
#include "INS_task.h"
#include "referee.h"
#include "CRC8_CRC16.h"
#include "remote_control.h"
#include "detect_task.h"

// Constants
#define OUTPUT_VISION_MODE 0
#define OUTPUT_PC_MODE 1

#define EXPECTED_INPUT_SCANSTATUS_HEDER 0xA3
#define EXPECTED_INPUT_NAVIGATION_HEDER 0xA4
#define EXPECTED_INPUT_VISION_HEDER 0xA5
#define EXPECTED_INPUT_PC_HEDER 0xA6
#define SET_OUTPUT_PC_HEDER 0x6A
#define SET_OUTPUT_VISION_HEDER 0x5A
#define SET_OUTPUT_AllRobotHP_HEDER 0x5B
#define SET_OUTPUT_GameStatus_HEDER 0x5C
#define SET_OUTPUT_RobotStatus_HEDER 0x5D

#define CRC16_INIT 0xFFFF
#define USB_STATE_INVALID 0xFF
#define APP_RX_DATA_SIZE 2048
#define APP_TX_DATA_SIZE 2048
#define USB_RECEIVE_LEN 384

// Variable Declarations
static uint8_t usb_tx_buf[APP_TX_DATA_SIZE];
static uint8_t usb_rx_buf[APP_RX_DATA_SIZE];

static ReceivedPacketVision_s ReceivedPacketVision;
static ReceivedPacketTwist_s ReceivedPacketTwist;
static ReceivedPacketScanStatus_s ReceivedScanStatus = {
    .stop_gimbal_scan = 0,
    .chassis_spin_vel = 0};
static InputPCData_s InputPCData;

static SendPacketVision_s SendPacketVision;
static SendPacketAllRobotHP_s SendPacketAllRobotHP;
static SendPacketGameStatus_s SendPacketGameStatus;
static SendPacketRobotStatus_s SendPacketRobotStatus;
OutputPCData_s OutputPCData;

static uint8_t USB_SEND_MODE;
static const fp32 *gimbal_INT_gyro_angle_point;
static usb_send_duration_t usb_send_duration;

// Function Prototypes
static void usb_send_vision(uint8_t t);
static void usb_send_AllRobotHP(uint8_t t);
static void usb_send_GameStatus(uint8_t t);
static void usb_send_RobotStatus(uint8_t t);
static void usb_send_outputPC(uint8_t t);
static uint8_t usb_receive_navigation(void);
static uint8_t usb_receive_vision(void);
static uint8_t usb_receive_scan_status(void);
static uint8_t usb_receive_PC(void);
static void usb_receive(void);
static void usb_send_data(uint16_t len);
static void char_to_uint(uint8_t *word, const char *str);

/**
 * @brief      USB任务主函数，switch中为发送，接收部分在usb_receive()中处理
 * @param[in]  argument: 任务参数
 * @retval     None
 */
void usb_task(void const *argument)
{
    MX_USB_DEVICE_Init();
    memset(&usb_send_duration, 0, sizeof(usb_send_duration_t));
    USB_SEND_MODE = OUTPUT_VISION_MODE;

    while (1)
    {
        usb_receive();

        switch (USB_SEND_MODE)
        {
        case OUTPUT_VISION_MODE:
            usb_send_vision(6);

            usb_send_AllRobotHP(100);
            usb_send_GameStatus(100);
            usb_send_RobotStatus(100);
            break;

        case OUTPUT_PC_MODE:
            usb_send_outputPC(1);
            break;

        case USB_STATE_INVALID:
            break;
        }
        osDelay(1); // 运行周期1ms
    }
}

/**
 * @brief      USB接收主函数，处理接收到的原始数据，根据数据头判断数据类型并分包处理
 * @param      None
 * @retval     None
 */
static void usb_receive(void)
{
    uint32_t len = USB_RECEIVE_LEN;
    CDC_Receive_FS(usb_rx_buf, &len); // Read data into the buffer
    uint8_t receive_ok = 0;

    switch (usb_rx_buf[0])
    {
    case EXPECTED_INPUT_SCANSTATUS_HEDER:
        /* 云台是否扫描模式 */
        receive_ok = usb_receive_scan_status();
        break;

    case EXPECTED_INPUT_NAVIGATION_HEDER:
        /* 导航 */
        receive_ok = usb_receive_navigation();
        break;

    case EXPECTED_INPUT_VISION_HEDER:
        /* 自瞄 */
        USB_SEND_MODE = OUTPUT_VISION_MODE;
        receive_ok = usb_receive_vision();
        break;

    case EXPECTED_INPUT_PC_HEDER:
        /* LJW串口助手 https://gitee.com/SMBU-POLARBEAR/Serial_Port_Assistant */
        USB_SEND_MODE = OUTPUT_PC_MODE;
        receive_ok = usb_receive_PC();
        break;

    default:
        break;
    }

    if (receive_ok)
    {
        detect_hook(USB_TOE);
    }
}

/**
 * @brief      用USB发送数据
 * @param[in]  len 发送数据的长度
 */
static void usb_send_data(uint16_t len)
{
    uint8_t usb_send_state = USBD_FAIL;
    while (usb_send_state != USBD_OK)
    {
        usb_send_state = CDC_Transmit_FS(usb_tx_buf, len);
    }
}

/**
 * @brief      为视觉 部分发送数据
 * @param      t 数据发送周期
 * @retval     None
 */
static void usb_send_vision(uint8_t t)
{
    if (usb_send_duration.vision < t)
    {
        usb_send_duration.vision++;
        return;
    }
    usb_send_duration.vision = 0;

    SendPacketVision.header = SET_OUTPUT_VISION_HEDER;

    uint8_t self_color = get_team_color(); // 获取自身颜色
    if (self_color == 2)                   // 处理裁判系统队伍判断异常的情况
    {
        SendPacketVision.detect_color = !PRESET_SELF_COLOR;
    }
    else
    {
        SendPacketVision.detect_color = !self_color;
    }

    append_CRC16_check_sum((uint8_t *)&SendPacketVision, sizeof(SendPacketVision));
    memcpy(usb_tx_buf, &SendPacketVision, sizeof(SendPacketVision_s));
    usb_send_data(sizeof(SendPacketVision_s));
}

/**
 * @brief      转发裁判系统，为上位机发送敌我双方全体机器人血量信息
 * @param      t 数据发送周期
 * @retval     None
 */
static void usb_send_AllRobotHP(uint8_t t)
{
    if (usb_send_duration.all_robot_hp < t)
    {
        usb_send_duration.all_robot_hp++;
        return;
    }
    usb_send_duration.all_robot_hp = 0;

    SendPacketAllRobotHP.header = SET_OUTPUT_AllRobotHP_HEDER;

    SendPacketAllRobotHP.red_1_robot_hp = game_robot_HP.red_1_robot_HP;
    SendPacketAllRobotHP.red_2_robot_hp = game_robot_HP.red_2_robot_HP;
    SendPacketAllRobotHP.red_3_robot_hp = game_robot_HP.red_3_robot_HP;
    SendPacketAllRobotHP.red_4_robot_hp = game_robot_HP.red_4_robot_HP;
    SendPacketAllRobotHP.red_5_robot_hp = game_robot_HP.red_5_robot_HP;
    SendPacketAllRobotHP.red_7_robot_hp = game_robot_HP.red_7_robot_HP;
    SendPacketAllRobotHP.red_outpost_hp = game_robot_HP.red_outpost_HP;
    SendPacketAllRobotHP.red_base_hp = game_robot_HP.red_base_HP;
    SendPacketAllRobotHP.blue_1_robot_hp = game_robot_HP.blue_1_robot_HP;
    SendPacketAllRobotHP.blue_2_robot_hp = game_robot_HP.blue_2_robot_HP;
    SendPacketAllRobotHP.blue_3_robot_hp = game_robot_HP.blue_3_robot_HP;
    SendPacketAllRobotHP.blue_4_robot_hp = game_robot_HP.blue_4_robot_HP;
    SendPacketAllRobotHP.blue_5_robot_hp = game_robot_HP.blue_5_robot_HP;
    SendPacketAllRobotHP.blue_7_robot_hp = game_robot_HP.blue_7_robot_HP;
    SendPacketAllRobotHP.blue_outpost_hp = game_robot_HP.blue_outpost_HP;
    SendPacketAllRobotHP.blue_base_hp = game_robot_HP.blue_base_HP;

    append_CRC16_check_sum((uint8_t *)&SendPacketAllRobotHP, sizeof(SendPacketAllRobotHP_s));
    memcpy(usb_tx_buf, &SendPacketAllRobotHP, sizeof(SendPacketAllRobotHP_s));
    usb_send_data(sizeof(SendPacketAllRobotHP_s));
}

/**
 * @brief      转发裁判系统，为上位机发送比赛状态信息
 * @param      t 数据发送周期
 * @retval     None
 */
static void usb_send_GameStatus(uint8_t t)
{
    if (usb_send_duration.game_status < t)
    {
        usb_send_duration.game_status++;
        return;
    }
    usb_send_duration.game_status = 0;

    SendPacketGameStatus.header = SET_OUTPUT_GameStatus_HEDER;

    SendPacketGameStatus.game_progress = game_status.game_progress;
    SendPacketGameStatus.stage_remain_time = game_status.stage_remain_time;

    append_CRC16_check_sum((uint8_t *)&SendPacketGameStatus, sizeof(SendPacketGameStatus_s));
    memcpy(usb_tx_buf, &SendPacketGameStatus, sizeof(SendPacketGameStatus_s));
    usb_send_data(sizeof(SendPacketGameStatus_s));
}

/**
 * @brief      转发裁判系统，为上位机发送机器人状态信息
 * @param      t 数据发送周期
 * @retval     None
 */
static void usb_send_RobotStatus(uint8_t t)
{
    if (usb_send_duration.robot_status < t)
    {
        usb_send_duration.robot_status++;
        return;
    }
    usb_send_duration.robot_status = 0;

    SendPacketRobotStatus.header = SET_OUTPUT_RobotStatus_HEDER;

    SendPacketRobotStatus.robot_id = robot_status.robot_id;
    SendPacketRobotStatus.current_hp = robot_status.current_HP;
    SendPacketRobotStatus.shooter_heat = get_shoot_heat();
    uint8_t self_color = get_team_color(); // 获取自身颜色
    if (self_color == 2)                   // 处理裁判系统队伍判断异常的情况
    {
        SendPacketRobotStatus.team_color = PRESET_SELF_COLOR;
    }
    else
    {
        SendPacketRobotStatus.team_color = self_color;
    }
    SendPacketRobotStatus.is_attacked = get_is_attack();

    append_CRC16_check_sum((uint8_t *)&SendPacketRobotStatus, sizeof(SendPacketRobotStatus_s));
    memcpy(usb_tx_buf, &SendPacketRobotStatus, sizeof(SendPacketRobotStatus_s));
    usb_send_data(sizeof(SendPacketRobotStatus_s));
}

/**
 * @brief      发送数据到LJW的串口调试助手
 * @param      t 数据发送周期
 * @retval     None
 */
static void usb_send_outputPC(uint8_t t)
{
    if (usb_send_duration.outputPC_data < t)
    {
        usb_send_duration.outputPC_data++;
        return;
    }
    usb_send_duration.outputPC_data = 0;

    gimbal_INT_gyro_angle_point = get_INS_angle_point();
    const RC_ctrl_t *rc_ctrl = get_remote_control_point();

    OutputPCData.header = SET_OUTPUT_PC_HEDER;
    OutputPCData.length = sizeof(OutputPCData_s);

    char_to_uint(OutputPCData.name_1, "yaw_ecd");
    OutputPCData.type_1 = 1;

    char_to_uint(OutputPCData.name_2, "power");
    OutputPCData.type_2 = 1;

    char_to_uint(OutputPCData.name_3, "c3");
    OutputPCData.type_3 = 1;

    char_to_uint(OutputPCData.name_4, "c4");
    OutputPCData.type_4 = 1;

    char_to_uint(OutputPCData.name_5, "r1");
    OutputPCData.type_5 = 1;

    char_to_uint(OutputPCData.name_6, "r2");
    OutputPCData.type_6 = 1;

    char_to_uint(OutputPCData.name_7, "r3");
    OutputPCData.type_7 = 1;

    char_to_uint(OutputPCData.name_8, "r4");
    OutputPCData.type_8 = 1;

    append_CRC16_check_sum((uint8_t *)&OutputPCData, sizeof(OutputPCData_s));
    memcpy(usb_tx_buf, &OutputPCData, sizeof(OutputPCData_s));
    usb_send_data(sizeof(OutputPCData_s));
}

/**
 * @brief      接收云台是否进入扫描模式的数据
 * @param[in]  none
 * @retval     crc_ok
 */
static uint8_t usb_receive_scan_status(void)
{
    uint8_t crc_ok = verify_CRC16_check_sum((uint8_t *)usb_rx_buf, sizeof(ReceivedPacketScanStatus_s));
    if (crc_ok)
    {
        memcpy(&ReceivedScanStatus, usb_rx_buf, sizeof(ReceivedPacketScanStatus_s));
    }
    return crc_ok;
}

/**
 * @brief      接收导航数据
 * @param[in]  none
 * @retval     crc_ok
 */
static uint8_t usb_receive_navigation(void)
{
    uint8_t crc_ok = verify_CRC16_check_sum((uint8_t *)usb_rx_buf, sizeof(ReceivedPacketTwist_s));
    if (crc_ok)
    {
        memcpy(&ReceivedPacketTwist, usb_rx_buf, sizeof(ReceivedPacketTwist_s));
    }
    return crc_ok;
}

/**
 * @brief      接收视觉数据
 * @param[in]  none
 * @retval     crc_ok
 */
static uint8_t usb_receive_vision(void)
{
    uint8_t crc_ok = verify_CRC16_check_sum((uint8_t *)usb_rx_buf, sizeof(ReceivedPacketVision_s));
    if (crc_ok)
    {
        memcpy(&ReceivedPacketVision, usb_rx_buf, sizeof(ReceivedPacketVision_s));
        // buzzer_on(500, 30000);
    }
    return crc_ok;
}

/**
 * @brief      接收LJW的串口调试助手的数据
 * @param[in]  none
 * @retval     crc_ok
 */
static uint8_t usb_receive_PC(void)
{
    uint8_t crc_ok = verify_CRC16_check_sum((uint8_t *)usb_rx_buf, sizeof(InputPCData_s));
    if (crc_ok)
    {
        memcpy(&InputPCData, usb_rx_buf, sizeof(InputPCData_s));
        // buzzer_on(500, 30000);
    }
    return crc_ok;
}

/**
 * @brief      将字符串转换为uint8_t数组
 * @param[out] word: 转换后的数组
 * @param[in]  str: 原始字符串
 * @retval     None
 */
void char_to_uint(uint8_t *word, const char *str)
{
    int i = 0;
    while (str[i] != '\0' && i < 10)
    {
        word[i] = str[i];
        i++;
    }
}

/**
 * @brief          获取视觉接收数据
 * @param[in]      none
 * @return         视觉接收数据指针
 */
const ReceivedPacketVision_s *GetReceivedPacketVisionPoint(void)
{
    return &ReceivedPacketVision;
}

/**
 * @brief          获取视觉发送数据
 * @param[in]      none
 * @return         视觉发送数据指针
 * @note           用于修改需要发送的视觉数据
 */
SendPacketVision_s *GetSendPacketVisionPoint(void)
{
    return &SendPacketVision;
}

/**
 * @brief          获取导航接收数据
 * @param[in]      none
 * @return         导航接收数据指针
 */
const ReceivedPacketTwist_s *GetReceivedPacketTwistPoint(void)
{
    return &ReceivedPacketTwist;
}

/**
 * @brief          获取由上位机发送的云台是否进入扫描模式
 * @param[in]      none
 * @return         视觉接收数据指针
 */
const ReceivedPacketScanStatus_s *GetReceivedPacketScanStatus(void)
{
    return &ReceivedScanStatus;
}
