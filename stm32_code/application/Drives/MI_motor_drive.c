/**
 *
 * @File:        MI_motor_driver.c
 * @Author:      小企鹅
 *
 */
/* Includes -------------------------------------------------------------------*/
#include "MI_motor_drive.h"
#include "main.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

CAN_TxHeaderTypeDef CAN_TxHeader_MI;

uint8_t MI_MASTERID = 1; //master id 发送指令时EXTID的bit8:15,反馈的bit0:7
uint8_t MI_fdbid = 0;//反馈ID，获取电机ID和识别码用
uint8_t MI_MCU_identifier[8];

/*-------------------- 按照小米电机通信类型写的对应函数--------------------*/
/**
  * @brief  float转int，数据打包用
  * @param  x float数值
  * @param  x_min float数值的最小值
  * @param  x_max float数值的最大值
  * @param  bits  int的数据位数
  * @retval null
  */
uint32_t float_to_uint(float x, float x_min, float x_max, int bits) {
    float span = x_max - x_min;
    float offset = x_min;
    if(x > x_max) x=x_max;
    else if(x < x_min) x= x_min;
    return (uint32_t) ((x-offset)*((float)((1<<bits)-1))/span);
}
/**
  * @brief  小米电机CAN通信发送
  * @param  hmotor 电机结构体
  * @retval null
  */
void MI_Motor_CanTx(MI_Motor_t* hmotor) {
 
    CAN_TxHeader_MI.DLC = 8;
    CAN_TxHeader_MI.IDE = CAN_ID_EXT;
    CAN_TxHeader_MI.RTR = CAN_RTR_DATA;
    CAN_TxHeader_MI.ExtId = *((uint32_t*)&(hmotor->EXT_ID));
    uint32_t mailbox;
    /* Start the Transmission process */
    uint32_t free_TxMailbox = HAL_CAN_GetTxMailboxesFreeLevel(hmotor->phcan);//检测是否有空闲邮箱
    while (free_TxMailbox<3){//等待空闲邮箱数达到3
        free_TxMailbox = HAL_CAN_GetTxMailboxesFreeLevel(hmotor->phcan);
    }
    uint32_t ret = HAL_CAN_AddTxMessage(hmotor->phcan, &CAN_TxHeader_MI, hmotor->txdata, &mailbox);//将发送的数据添加到发送邮箱中
    // while (ret != HAL_OK) {
    //     /* Transmission request Error */
		// 	  HAL_Delay(0);
    //     ret = HAL_CAN_AddTxMessage(hmotor->phcan, &CAN_TxHeader_MI, hmotor->txdata, &mailbox);
    // }
}
/**
  * @brief  小米电机初始化
  * @param  hmotor 电机结构体
  * @param  phcan can总线句柄
  * @retval null
  */
void MI_motor_init(MI_Motor_t* hmotor,CAN_HandleTypeDef *phcan)
{
    hmotor->phcan = phcan;
}
/**
  * @brief  获取设备ID （通信类型0），需在电机使能前使用
  * @param  hmotor 电机结构体
  * @retval null
  */
void MI_motor_get_ID(MI_Motor_t* hmotor)
{
    hmotor->EXT_ID.mode = 0;
    hmotor->EXT_ID.data = 0;
    hmotor->EXT_ID.motor_id = 0;
    hmotor->EXT_ID.res = 0;
 
    for(uint8_t i=0; i<8; i++)
    {
        hmotor->txdata[i]=0;
    }
    MI_Motor_CanTx(hmotor);
}
/**
  * @brief  运控模式电机控制指令（通信类型1）
  * @param  hmotor 电机结构体
  * @param  torque 目标力矩
  * @param  MechPosition 不知道啥玩意
  * @param  speed 不知道啥玩意
  * @param  kp 不知道啥玩意
  * @param  kd 不知道啥玩意
  * @retval null
  */
void MI_motor_controlmode(MI_Motor_t* hmotor, float torque, float MechPosition , float speed , float kp , float kd)
{
    hmotor->EXT_ID.mode = 1;
    hmotor->EXT_ID.data = float_to_uint(torque,T_MIN,T_MAX,16);
    hmotor->EXT_ID.res = 0;
 
    hmotor->txdata[0]=float_to_uint(MechPosition,P_MIN,P_MAX,16)>>8;
    hmotor->txdata[1]=float_to_uint(MechPosition,P_MIN,P_MAX,16);
    hmotor->txdata[2]=float_to_uint(speed,V_MIN,V_MAX,16)>>8;
    hmotor->txdata[3]=float_to_uint(speed,V_MIN,V_MAX,16);
    hmotor->txdata[4]=float_to_uint(kp,KP_MIN,KP_MAX,16)>>8;
    hmotor->txdata[5]=float_to_uint(kp,KP_MIN,KP_MAX,16);
    hmotor->txdata[6]=float_to_uint(kd,KD_MIN,KD_MAX,16)>>8;
    hmotor->txdata[7]=float_to_uint(kd,KD_MIN,KD_MAX,16);
    MI_Motor_CanTx(hmotor);
}
/**
  * @brief  小米电机使能（通信类型 3）
  * @param  hmotor 电机结构体
  * @param  id 电机id
  * @retval null
  */
void MI_motor_enable(MI_Motor_t* hmotor,uint8_t id)
{
    hmotor->EXT_ID.mode = 3;
    hmotor->EXT_ID.motor_id = id;
    hmotor->EXT_ID.data = MI_MASTERID;
    hmotor->EXT_ID.res = 0;
    for(uint8_t i=0; i<8; i++)
    {
        hmotor->txdata[i]=0;
    }
    MI_Motor_CanTx(hmotor);
}
/**
  * @brief  电机停止运行帧（通信类型4）
  * @param  hmotor 电机结构体
  * @retval null
  */
void MI_motor_stop(MI_Motor_t* hmotor)
{
    hmotor->EXT_ID.mode = 4;
    hmotor->EXT_ID.data = MI_MASTERID;
    hmotor->EXT_ID.res = 0;
 
    for(uint8_t i=0; i<8; i++)
    {
        hmotor->txdata[i]=0;
    }
    MI_Motor_CanTx(hmotor);
}
/**
  * @brief  设置电机机械零位（通信类型6）会把当前电机位置设为机械零位（掉电丢失）
  * @param  hmotor 电机结构体
  * @retval null
  */
void MI_motor_setMechPosition2Zero(MI_Motor_t* hmotor)
{
    hmotor->EXT_ID.mode = 6;
    hmotor->EXT_ID.data = MI_MASTERID;
    hmotor->EXT_ID.res = 0;
    hmotor->txdata[0]=1;
 
    for(uint8_t i=1; i<8; i++)
    {
        hmotor->txdata[i]=0;
    }
    MI_Motor_CanTx(hmotor);
}
/**
  * @brief  设置电机CAN_ID（通信类型7）更改当前电机CAN_ID , 立即生效，需在电机使能前使用
  * @param  hmotor 电机结构体
  * @param  Now_ID 电机现在的ID
  * @param  Target_ID 想要改成的电机ID
  * @retval null
  */
void MI_motor_changeID(MI_Motor_t* hmotor,uint8_t Now_ID,uint8_t Target_ID)
{
    hmotor->EXT_ID.mode = 7;	
    hmotor->EXT_ID.motor_id = Now_ID;
    hmotor->EXT_ID.data = Target_ID << 8 | MI_MASTERID;
    hmotor->EXT_ID.res = 0;
 
    for(uint8_t i=0; i<8; i++)
    {
        hmotor->txdata[i]=0;
    }
    MI_Motor_CanTx(hmotor);
}
/**
  * @brief  单个参数读取（通信类型17）
  * @param  hmotor 电机结构体
  * @param  index 功能码
  * @retval null
  * @note   我用不着，所以没写反馈解码
  */
void MI_motor_Read_One_Para(MI_Motor_t* hmotor,uint16_t index)
{
    hmotor->EXT_ID.mode = 17;
    hmotor->EXT_ID.data = MI_MASTERID;
    hmotor->EXT_ID.res = 0;
    hmotor->txdata[0]=index;
    memcpy(&hmotor->txdata[0],&index,2);
    for(uint8_t i=2; i<8; i++)
    {
        hmotor->txdata[i]=0;
    }
    MI_Motor_CanTx(hmotor);
}
/**
  * @brief  单个参数写入（通信类型18） （掉电丢失）
  * @param  hmotor 电机结构体
  * @param  index 功能码
  * @param  data[4] 参数数据缓冲
  * @retval null
  * @note   我用不着，所以没写反馈解码
  */
void MI_motor_Write_One_Para(MI_Motor_t* hmotor, uint16_t index ,uint8_t data[4])
{
    hmotor->EXT_ID.mode = 18;
    hmotor->EXT_ID.data = MI_MASTERID;
    hmotor->EXT_ID.res = 0;
 
    memcpy(&hmotor->txdata[0],&index,2);
    memcpy(&hmotor->txdata[4],data, 4);
    MI_Motor_CanTx(hmotor);
}
/**
  * @brief          小米电机反馈帧解码（通信类型2）
  * @param[in]      Rx_can_info 接受到的电机数据结构体
  * @param[in]      rx_data[8] CAN线接收到的数据
  * @note           将接收到的CAN线数据解码到电机数据结构体中
  * @retval         none
  */
 void MI_motor_RxDecode(RxCAN_info_s* RxCAN_info,uint8_t rx_data[8]){
      uint16_t decode_temp_mi;//小米电机反馈数据解码缓冲
      decode_temp_mi = (rx_data[0] << 8 | rx_data[1]);
      RxCAN_info->angle = ((float)decode_temp_mi-32767.5)/32767.5*4*3.1415926f;;
  
      decode_temp_mi = (rx_data[2] << 8 | rx_data[3]);
      RxCAN_info->speed = ((float)decode_temp_mi-32767.5)/32767.5*30.0f;
  
      decode_temp_mi = (rx_data[4] << 8 | rx_data[5]);
      RxCAN_info->torque = ((float)decode_temp_mi-32767.5)/32767.5*12.0f;
  
      decode_temp_mi = (rx_data[6] << 8 | rx_data[7]);
      RxCAN_info->temperature = (float)decode_temp_mi/10.0f;
 }
// /**
//   * @brief          hal库CAN回调函数,接收电机数据
//   * @param[in]      hcan:CAN句柄指针
//   * @retval         none
//   */
// void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
// {
//     CAN_RxHeaderTypeDef rx_header;
//     RxCAN_info_s Rx_can_info;//用于存储小米电机反馈的数据
//     uint8_t rx_data[8];

//     HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

//     memcpy(&Rx_can_info,&rx_header.ExtId,29);//将扩展标识符的内容解码成对应内容

//     uint16_t decode_temp_mi;//小米电机反馈数据解码缓冲
//     if (Rx_can_info.communication_type == 2){//若为通信类型2的反馈帧就对应解码
//       MI_motor_RxDecode(&Rx_can_info,rx_data);
//     }
// }
