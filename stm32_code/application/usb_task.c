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
#include "usb_task.h"

#include "cmsis_os.h"

#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "usbd_conf.h"
#include <stdio.h>
#include <stdarg.h>
#include "string.h"
#include "bsp_buzzer.h"
#include "detect_task.h"
#include "voltage_task.h"
#include "gimbal_task.h"
#include "chassis_task.h"
#include "INS_task.h"
#include "referee.h"
#include <math.h>

#include "Balance_Controler.h"

// Function Declarations
void char_to_uint(uint8_t *word, const char *str);
static void usb_printf(const char *fmt, ...);
static void usb_send_AutoAim(void);
static void usb_send_outputPC(void);
static void usb_receive(void);

uint16_t Get_CRC16_Check_Sum(const uint8_t *pchMessage, uint32_t dwLength, uint16_t wCRC);
uint32_t Verify_CRC16_Check_Sum(const uint8_t *pchMessage, uint32_t dwLength);
void Append_CRC16_Check_Sum(void *pchMessage, uint32_t dwLength, int dataType);

// Constants Declaration
const error_t *error_list_usb_local;
static const fp32 *gimbal_INS_angle_point;

// External Variable Declarations
extern ext_game_robot_state_t robot_state;
extern ext_power_heat_data_t power_heat_data_t;
extern ext_game_robot_HP_t game_robot_HP_t;
extern uint8_t UserTxBufferFS;
extern uint8_t UserRxBufferFS;
extern PCD_HandleTypeDef hpcd_USB_OTG_FS;
extern CRC_HandleTypeDef hcrc;

// Variable Declarations
static uint8_t USB_STATE = AUTO_AIM_STATE;
ReceivedData_s ReceivedData;
ReceivedData_s AimingParameter;
SendData_s SendData;
InputData_s InputData;
OutputData_s OutputData;

#define APP_RX_DATA_SIZE 2048
#define APP_TX_DATA_SIZE 2048
static uint8_t usb_tx_buf[APP_TX_DATA_SIZE];
static uint8_t usb_rx_buf[APP_RX_DATA_SIZE];
uint32_t UserTxLengthFS = 0;
#define CRC16_INIT 0xFFFF

void usb_task(void const *argument)
{
    MX_USB_DEVICE_Init();

    buzzer_off();
    while (1)
    {
        usb_receive();

        if (InputData.header == 0xA6)
            /* Used for adapting Penguin's serial debugging software: https://gitee.com/SMBU-POLARBEAR/Serial_Port_Assistant */
            USB_STATE = OUTPUT_PC_STATE;
        else
            /* Used for sending data to the minipc */
            USB_STATE = AUTO_AIM_STATE;

        if (USB_STATE == AUTO_AIM_STATE)
        {
            SendData.header = 0x5A;

            uint8_t crc_ok = Verify_CRC16_Check_Sum((const uint8_t *)(&ReceivedData), sizeof(ReceivedData));
            if (crc_ok)
            {
                buzzer_on(300, 30000);
                /* Handle the data when CRC is correct */
                AimingParameter = ReceivedData;

                gimbal_INS_angle_point = get_INS_angle_point(); // Get Euler angles: 0:yaw, 1:pitch, 2:roll in radians

                SendData.detect_color = 0;  // TODO: Assigned by UART
                SendData.reset_tracker = 0; // TODO: Assigned by Auto_Aim
                SendData.reserved = 1;
                SendData.roll = gimbal_INS_angle_point[2];
                SendData.pitch = gimbal_INS_angle_point[1];
                SendData.yaw = gimbal_INS_angle_point[0];
                // SendData.aim_x assigned by gimbal_autoaim_control()
                // SendData.aim_y assigned by gimbal_autoaim_control()
                // SendData.aim_z assigned by gimbal_autoaim_control()
            }

            usb_send_AutoAim(); // Send data
        }
        else if (USB_STATE == OUTPUT_PC_STATE)
        {
            uint8_t crc_ok = Verify_CRC16_Check_Sum((const uint8_t *)(&InputData), sizeof(InputData));

            if (crc_ok)
            {
                gimbal_INS_angle_point = get_INS_angle_point(); // Get Euler angles: 0:yaw, 1:pitch, 2:roll in radians
                buzzer_on(500, 30000);

                const Leg_Pos_t *left_leg_pos = GetLegPosPoint(0);
                const Leg_Pos_t *right_leg_pos = GetLegPosPoint(1);
                const Chassis_IMU_t *chassis_imu = GetChassisIMUPoint();
                const Target_s *target = GetTargetPoint();
                const State_Var_s *state_var = GetStateVarPoint();

                OutputData.header = 0x6A;
                OutputData.length = sizeof(OutputData_s);

                char_to_uint(OutputData.name_1, "cur_val");
                OutputData.type_1 = 1;

                char_to_uint(OutputData.name_2, "Pitch");
                OutputData.type_2 = 1;
                OutputData.data_2 = chassis_imu->pitch;

                // char_to_uint(OutputData.name_1,"m1-angle");
                // OutputData.type_1 = 1;
                // OutputData.data_1 = left_joint[0].angle;

                // char_to_uint(OutputData.name_2,"m2-angle");
                // OutputData.type_2 = 1;
                // OutputData.data_2 = left_joint[1].angle;

                char_to_uint(OutputData.name_3,"L_F_T");
                OutputData.type_3 = 1;

                char_to_uint(OutputData.name_4,"L_B_T");
                OutputData.type_4 = 1;

                char_to_uint(OutputData.name_5,"R_F_T");
                OutputData.type_5 = 1;

                char_to_uint(OutputData.name_6,"R_B_T");
                OutputData.type_6 = 1;

                char_to_uint(OutputData.name_7,"l_wheel_s");
                OutputData.type_7 = 1;

                char_to_uint(OutputData.name_8,"r_wheel_s");
                OutputData.type_8 = 1;

                // char_to_uint(OutputData.name_9,"R_F_A");
                // OutputData.type_9 = 1;

                // char_to_uint(OutputData.name_10,"R_B_A");
                // OutputData.type_10 = 1;

                usb_send_outputPC();
            }
            else
            {
                buzzer_off();
            }
        }
        else
        {
            buzzer_on(1000, 30000);
        }
    }
}

void char_to_uint(uint8_t *word, const char *str)
{
    int i = 0;
    while (str[i] != '\0' && i < 10)
    {
        word[i] = str[i];
        i++;
    }
}

static void usb_send_AutoAim(void)
{
    Append_CRC16_Check_Sum(&SendData, sizeof(SendData_s), 0);
    memcpy(usb_tx_buf, &SendData, sizeof(SendData_s));
    CDC_Transmit_FS(usb_tx_buf, sizeof(SendData_s));
}

static void usb_send_outputPC(void)
{
    Append_CRC16_Check_Sum(&OutputData, sizeof(OutputData_s), 1);
    memcpy(usb_tx_buf, &OutputData, sizeof(OutputData_s));
    CDC_Transmit_FS(usb_tx_buf, sizeof(OutputData_s));
}

static void usb_receive(void)
{
    uint32_t len = USB_RECEIVE_LEN;
    CDC_Receive_FS(usb_rx_buf, &len); // Read data into the buffer
    if (usb_rx_buf[0] == 0xA5)
    {
        memcpy(&ReceivedData, usb_rx_buf, sizeof(ReceivedData_s));
    }
    else if (usb_rx_buf[0] == 0xA6)
    {
        memcpy(&InputData, usb_rx_buf, sizeof(InputData_s));
    }
    else
    {
        memcpy(&ReceivedData, usb_rx_buf, sizeof(ReceivedData_s));
    }
}

static void usb_printf(const char *fmt, ...)
{
    static va_list ap;
    uint16_t len = 0;
    va_start(ap, fmt);
    len = vsprintf((char *)usb_tx_buf, fmt, ap);
    va_end(ap);
    CDC_Transmit_FS(usb_tx_buf, len);
}

// CRC Table
const uint16_t W_CRC_TABLE[256] = {
    0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf, 0x8c48, 0x9dc1, 0xaf5a, 0xbed3,
    0xca6c, 0xdbe5, 0xe97e, 0xf8f7, 0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
    0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876, 0x2102, 0x308b, 0x0210, 0x1399,
    0x6726, 0x76af, 0x4434, 0x55bd, 0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
    0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c, 0xbdcb, 0xac42, 0x9ed9, 0x8f50,
    0xfbef, 0xea66, 0xd8fd, 0xc974, 0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
    0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3, 0x5285, 0x430c, 0x7197, 0x601e,
    0x14a1, 0x0528, 0x37b3, 0x263a, 0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
    0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9, 0xef4e, 0xfec7, 0xcc5c, 0xddd5,
    0xa96a, 0xb8e3, 0x8a78, 0x9bf1, 0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
    0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70, 0x8408, 0x9581, 0xa71a, 0xb693,
    0xc22c, 0xd3a5, 0xe13e, 0xf0b7, 0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
    0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036, 0x18c1, 0x0948, 0x3bd3, 0x2a5a,
    0x5ee5, 0x4f6c, 0x7df7, 0x6c7e, 0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
    0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd, 0xb58b, 0xa402, 0x9699, 0x8710,
    0xf3af, 0xe226, 0xd0bd, 0xc134, 0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
    0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3, 0x4a44, 0x5bcd, 0x6956, 0x78df,
    0x0c60, 0x1de9, 0x2f72, 0x3efb, 0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
    0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a, 0xe70e, 0xf687, 0xc41c, 0xd595,
    0xa12a, 0xb0a3, 0x8238, 0x93b1, 0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
    0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330, 0x7bc7, 0x6a4e, 0x58d5, 0x495c,
    0x3de3, 0x2c6a, 0x1ef1, 0x0f78};

/**
 * @brief CRC16 Caculation function
 * @param[in] pchMessage : Data to Verify,
 * @param[in] dwLength : Stream length = Data + checksum
 * @param[in] wCRC : CRC16 init value(default : 0xFFFF)
 * @return : CRC16 checksum
 */
uint16_t Get_CRC16_Check_Sum(const uint8_t *pchMessage, uint32_t dwLength, uint16_t wCRC)
{
    uint8_t ch_data;
    if (pchMessage == NULL)
        return 0xFFFF;
    while (dwLength--)
    {
        ch_data = *pchMessage++;
        (wCRC) =
            ((uint16_t)(wCRC) >> 8) ^ W_CRC_TABLE[((uint16_t)(wCRC) ^ (uint16_t)(ch_data)) & 0x00ff];
    }
    return wCRC;
}

/**
 * @brief CRC16 Verify function
 * @param[in] pchMessage : Data to Verify,
 * @param[in] dwLength : Stream length = Data + checksum
 * @return : True or False (CRC Verify Result)
 */
uint32_t Verify_CRC16_Check_Sum(const uint8_t *pchMessage, uint32_t dwLength)
{
    uint16_t w_expected = 0;
    if ((pchMessage == NULL) || (dwLength <= 2))
        return false;
    w_expected = Get_CRC16_Check_Sum(pchMessage, dwLength - 2, CRC16_INIT); // dwLength - 2 ->dwLength - 4
    return (
        (w_expected & 0xff) == pchMessage[dwLength - 2] &&
        ((w_expected >> 8) & 0xff) == pchMessage[dwLength - 1]);
}

/**
 * @brief CRC16 Append function for different data structures
 * @param[in] pchMessage : Data to append CRC
 * @param[in] dwLength : Stream length = Data length + checksum
 * @param[in] dataType : Type of data (0 for SendData_s, 1 for OutputData_s)
 * @return : True or False (CRC Verify Result)
 */
void Append_CRC16_Check_Sum(void *pchMessage, uint32_t dwLength, int dataType)
{
    uint16_t w_crc = 0;

    if (pchMessage == NULL || dwLength <= 2)
    {
        return;
    }

    if (dataType == 0)
    {
        SendData_s *send_data = (SendData_s *)pchMessage;
        w_crc = Get_CRC16_Check_Sum((uint8_t *)(send_data), dwLength - 2, CRC16_INIT);
        send_data->checksum = w_crc;
    }
    else if (dataType == 1)
    {
        OutputData_s *output_data = (OutputData_s *)pchMessage;
        w_crc = Get_CRC16_Check_Sum((uint8_t *)(output_data), dwLength - 2, CRC16_INIT);
        output_data->checksum = w_crc;
    }
}
