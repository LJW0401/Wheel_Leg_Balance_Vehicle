/**
  ****************************(C) COPYRIGHT 2023 Polarbear*************************
  * @file       usb_task.c/h
  * @brief      usb outputs the IMU and gimbal data to the miniPC
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     2023-7-11       Penguin         1. done
  *  V1.0.1     Oct-31-2023     LihanChen       1. Finish building the core framework to make it compatible with both debugging mode and MiniPC mode.
  *  V1.0.2     Nov-1-2023      LihanChen       1. Merge Append_CRC16_Check_Sum_SendData() and Append_CRC16_Check_Sum_OutputData() into Append_CRC16_Check_Sum()
  *
  @verbatim
  =================================================================================

  =================================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2023 Polarbear*************************
  */
#ifndef USB_TASK_H
#define USB_TASK_H
#include "struct_typedef.h"
#include "stdbool.h"

#define AUTO_AIM_STATE 0
#define OUTPUT_PC_STATE 1
#define USB_RECEIVE_LEN 384 // 8 +1+3+3+1 +11*32 +16 bit

typedef struct{
  uint8_t header;
  bool tracking : 1;
  uint8_t id : 3;          // 0-outpost 6-guard 7-base
  uint8_t armors_num : 3;  // 2-balance 3-outpost 4-normal
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
}__attribute__((packed)) ReceivedData_s;

typedef struct{
  uint8_t header;
  uint8_t detect_color : 1;  // 0-red 1-blue
  bool reset_tracker : 1;
  uint8_t reserved : 6;
  float roll;
  float pitch;
  float yaw;
  float aim_x;  // For ROS2 Visualization
  float aim_y;  // For ROS2 Visualization
  float aim_z;  // For ROS2 Visualization
  uint16_t checksum;
} __attribute__((packed)) SendData_s;



typedef struct{
  uint8_t header;
  uint16_t ecd_set;
  uint16_t checksum;
}__attribute__((packed)) InputData_s;

typedef struct{
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
} __attribute__((packed)) OutputData_s;

extern ReceivedData_s ReceivedData;
extern SendData_s SendData;
extern OutputData_s OutputData;

extern void usb_task(void const * argument);

#endif
