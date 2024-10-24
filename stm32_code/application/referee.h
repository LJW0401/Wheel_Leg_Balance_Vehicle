/**
  ****************************(C) COPYRIGHT 2024 Polarbear*************************
  * @file       referee.c/h
  * @brief      Updated the referee system Communication Protocol 1.6.1
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     2024/3/6         YZX             1.更新裁判系统通信协议至1.6.1（不包含半自动步兵和UI）
  @verbatim
  =================================================================================

  =================================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear*************************
*/
#ifndef REFEREE_H
#define REFEREE_H
#include <stdbool.h>
#include "main.h"
#include "protocol.h"

typedef enum
{
    RED_HERO        = 1,
    RED_ENGINEER    = 2,
    RED_STANDARD_1  = 3,
    RED_STANDARD_2  = 4,
    RED_STANDARD_3  = 5,
    RED_AERIAL      = 6,
    RED_SENTRY      = 7,
    BLUE_HERO       = 11,
    BLUE_ENGINEER   = 12,
    BLUE_STANDARD_1 = 13,
    BLUE_STANDARD_2 = 14,
    BLUE_STANDARD_3 = 15,
    BLUE_AERIAL     = 16,
    BLUE_SENTRY     = 17,
} robot_id_t;
typedef enum
{
    PROGRESS_UNSTART        = 0,
    PROGRESS_PREPARE        = 1,
    PROGRESS_SELFCHECK      = 2,
    PROGRESS_5sCOUNTDOWN    = 3,
    PROGRESS_BATTLE         = 4,
    PROGRESS_CALCULATING    = 5,
} game_progress_t;
typedef __packed struct //0x0001
{
    uint8_t game_type : 4; //比赛类型
    uint8_t game_progress : 4; //当前比赛阶段
    uint16_t stage_remain_time; //当前阶段剩余时间，单位：秒
    uint64_t SyncTimeStamp; //UNIX 时间，当机器人正确连接到裁判系统的 NTP 服务器后生效
} game_status_t;

typedef __packed struct //0x0002
{
    uint8_t winner; //0：平局 1：红方胜利 2：蓝方胜利
} game_result_t;
typedef __packed struct //0x0003
{
    uint16_t red_1_robot_HP;  //红 1 英雄机器人血量。若该机器人未上场或者被罚下，则血量为 0
    uint16_t red_2_robot_HP;  //红 2 工程机器人血量
    uint16_t red_3_robot_HP;  //红 3 步兵机器人血量
    uint16_t red_4_robot_HP;  //红 4 步兵机器人血量
    uint16_t red_5_robot_HP;  //红 5 步兵机器人血量
    uint16_t red_7_robot_HP;  //红 7 哨兵机器人血量
    uint16_t red_outpost_HP;  //红方前哨站血量
    uint16_t red_base_HP;     //红方基地血量
    uint16_t blue_1_robot_HP; //蓝 1 英雄机器人血量
    uint16_t blue_2_robot_HP; //蓝 2 工程机器人血量
    uint16_t blue_3_robot_HP; //蓝 3 步兵机器人血量
    uint16_t blue_4_robot_HP; //蓝 4 步兵机器人血量
    uint16_t blue_5_robot_HP; //蓝 5 步兵机器人血量
    uint16_t blue_7_robot_HP; //蓝 7 哨兵机器人血量
    uint16_t blue_outpost_HP; //蓝方前哨站血量
    uint16_t blue_base_HP;    //蓝方基地血量
} game_robot_HP_t;
typedef __packed struct //0x0101
{
    uint32_t event_data;
} event_data_t;

typedef __packed struct //0x0102
{
    uint8_t supply_projectile_id;
    uint8_t supply_robot_id;
    uint8_t supply_projectile_step;
    uint8_t supply_projectile_num;
} ext_supply_projectile_action_t;


typedef __packed struct //0x0103
{
    uint8_t reserved;
    uint8_t supply_robot_id;
    uint8_t supply_projectile_step;
    uint8_t supply_projectile_num;
} ext_supply_projectile_booking_t;

typedef __packed struct //0x0104
{
    uint8_t level;
    uint8_t offending_robot_id;
    uint8_t count;
} referee_warning_t;

typedef __packed struct //0x105
{
    uint8_t dart_remaining_time;
    uint16_t dart_info;
} dart_info_t;
typedef __packed struct //0x0201
{
	uint8_t robot_id;                             //本机器人 ID
    uint8_t robot_level;                          //机器人等级
    uint16_t current_HP;                          //机器人当前血量
    uint16_t maximum_HP;                          //机器人血量上限
    uint16_t shooter_barrel_cooling_value;        //机器人枪口热量每秒冷却值
    uint16_t shooter_barrel_heat_limit;           //机器人枪口热量上限
    uint16_t chassis_power_limit;                 //机器人底盘功率上限
    uint8_t power_management_gimbal_output : 1;   //电源管理模块的输出情况：gimbal 口输出：0 为无输出，1 为 24V 输出
    uint8_t power_management_chassis_output : 1;  //chassis 口输出：0 为无输出，1 为 24V 输出
    uint8_t power_management_shooter_output : 1;  //shooter 口输出：0 为无输出，1 为 24V 输出
} robot_status_t;

typedef __packed struct //0x0202
{
    uint16_t chassis_voltage;             //电源管理模块的 chassis 口输出电压（单位：mV）
    uint16_t chassis_current;             //电源管理模块的 chassis 口输出电流（单位：mA）
    float chassis_power;                  //底盘功率（单位：W）
    uint16_t buffer_energy;               //缓冲能量（单位：J）
    uint16_t shooter_17mm_1_barrel_heat;  //第 1 个 17mm 发射机构的枪口热量
    uint16_t shooter_17mm_2_barrel_heat;  //第 2 个 17mm 发射机构的枪口热量
    uint16_t shooter_42mm_barrel_heat;    //42mm 发射机构的枪口热量
} power_heat_data_t;

typedef __packed struct //0x0203
{
    float x;                          //本机器人位置 x 坐标，单位：m
    float y;                          //本机器人位置 y 坐标，单位：m
    float angle;                      //本机器人测速模块的朝向，单位：度。正北为 0 度
} robot_pos_t;

typedef __packed struct //0x0204
{
    uint8_t recovery_buff;           //机器人回血增益（百分比，值为 10 表示每秒恢复血量上限的 10%）
    uint8_t cooling_buff;            //机器人枪口冷却倍率（直接值，值为 5 表示 5 倍冷却）
    uint8_t defence_buff;            //机器人防御增益（百分比，值为 50 表示 50%防御增益）
    uint8_t vulnerability_buff;      //机器人负防御增益（百分比，值为 30 表示-30%防御增益）
    uint16_t attack_buff;            //机器人攻击增益（百分比，值为 50 表示 50%攻击增益）
} buff_t;

typedef __packed struct //0x0205
{
    uint8_t airforce_status;         //空中机器人状态（0 为正在冷却，1 为冷却完毕，2 为正在空中支援）
    uint8_t time_remain;             //此状态的剩余时间（单位为：秒，向下取整，即冷却时间剩余 1.9 秒时，此值为 1）若冷却时间为 0，但未呼叫空中支援，则该值为 0
} air_support_data_t;

typedef __packed struct //0x0206
{
    uint8_t armor_id : 4;             //当扣血原因为装甲模块被弹丸攻击、受撞击、离线或测速模块离线时，该 4 bit 组成的数值为装甲模块或测速模块的 ID 编号；当其他原因导致扣血时，该数值为 0
    uint8_t HP_deduction_reason : 4;  //血量变化类型
} hurt_data_t;

typedef __packed struct //0x0207
{
    uint8_t bullet_type;             //弹丸类型
    uint8_t shooter_number;          //发射机构 ID：
    uint8_t launching_frequency;     //弹丸射速（单位：Hz）
    float initial_speed;             //弹丸初速度（单位：m/s）
} shoot_data_t;
typedef __packed struct //0x0208
{
 uint16_t projectile_allowance_17mm; //17mm 弹丸允许发弹量
 uint16_t projectile_allowance_42mm; //42mm 弹丸允许发弹量
 uint16_t remaining_gold_coin;       //剩余金币数量
} projectile_allowance_t;
typedef __packed struct //0x0209
{
 uint32_t rfid_status;
} rfid_status_t;
typedef __packed struct
{
    uint8_t bullet_remaining_num;
} ext_bullet_remaining_t;
typedef __packed struct //0x020A
{
 uint8_t dart_launch_opening_status;
 uint8_t reserved;
 uint16_t target_change_time;
 uint16_t latest_launch_cmd_time;
} dart_client_cmd_t;
typedef __packed struct //0x020B
{
 float hero_x;                      //己方英雄机器人位置 x 轴坐标，单位：m
 float hero_y;                      //己方英雄机器人位置 y 轴坐标，单位：m
 float engineer_x;                  //己方工程机器人位置 x 轴坐标，单位：m
 float engineer_y;                  //己方工程机器人位置 y 轴坐标，单位：m
 float standard_3_x;                //己方 3 号步兵机器人位置 x 轴坐标，单位：m
 float standard_3_y;                //己方 3 号步兵机器人位置 y 轴坐标，单位：m
 float standard_4_x;                //己方 4 号步兵机器人位置 x 轴坐标，单位：m
 float standard_4_y;                //己方 4 号步兵机器人位置 x 轴坐标，单位：m
 float standard_5_x;                //己方 5 号步兵机器人位置 x 轴坐标，单位：m
 float standard_5_y;                //己方 5 号步兵机器人位置 y 轴坐标，单位：m
} ground_robot_position_t;
typedef __packed struct //0x020C
{
 uint8_t mark_hero_progress;
 uint8_t mark_engineer_progress;
 uint8_t mark_standard_3_progress;
 uint8_t mark_standard_4_progress;
 uint8_t mark_standard_5_progress;
 uint8_t mark_sentry_progress;
} radar_mark_data_t;
typedef __packed struct //0x020D
{
 uint32_t sentry_info;
} sentry_info_t;
typedef __packed struct //0x020E
{
 uint8_t radar_info;
} radar_info_t;
typedef __packed struct //0x0301
{
    uint16_t data_cmd_id;
    uint16_t sender_id;
    uint16_t receiver_id;
    uint8_t user_data[113];
} robot_interaction_data_t;

typedef __packed struct
{
    float data1;
    float data2;
    float data3;
    uint8_t data4;
} custom_data_t;


typedef __packed struct
{
    uint8_t data[64];
} ext_up_stream_data_t;

typedef __packed struct
{
    uint8_t data[32];
} ext_download_stream_data_t;

typedef __packed struct
{
uint16_t mouse_x;
uint16_t mouse_y;
uint16_t mouse_z;
uint8_t left_button_down;
uint8_t right_button_down;
uint16_t keyboard_value;
uint16_t reserved;
} ext_robot_command_t;

typedef  struct
{
  uint16_t current_hp;
  bool attacked;
  uint16_t daleytime;
} judge_attacked_t;

extern game_robot_HP_t game_robot_HP;
extern robot_status_t robot_status;
extern game_status_t game_status;
extern void init_referee_struct_data(void);
extern void referee_data_solve(uint8_t *frame);


extern uint16_t get_shoot_heat(void);
extern uint8_t get_robot_id(void);
extern bool get_is_attack(void);
extern uint8_t get_team_color(void);
extern void get_shoot_heat_limit_and_heat(uint16_t *heat_limit, uint16_t *heat);
extern void get_chassis_power_and_buffer(float *power, uint16_t *buffer);
#endif
