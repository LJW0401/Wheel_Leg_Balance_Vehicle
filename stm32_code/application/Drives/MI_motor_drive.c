/**
 *
 * @File:        MI_motor_driver.c
 * @Author:      北极熊
 *
 */
/* Includes -------------------------------------------------------------------*/
#include "MI_motor_drive.h"
#include "main.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

uint8_t MI_MASTERID = 1; //master id 发送指令时EXTID的bit8:15,反馈的bit0:7
uint8_t MI_fdbid = 0;//反馈ID，获取电机ID和识别码用
uint8_t MI_MCU_identifier[8];
MI_Motor_t MI_Motor;


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
CAN_TxHeaderTypeDef CAN_TxHeader_MI;
void MI_Motor_CanTx(MI_Motor_t* hmotor) {
 
    CAN_TxHeader_MI.DLC = 8;
    CAN_TxHeader_MI.IDE = CAN_ID_EXT;
    CAN_TxHeader_MI.RTR = CAN_RTR_DATA;
    CAN_TxHeader_MI.ExtId = *((uint32_t*)&(hmotor->EXT_ID));
	/*CAN_TxHeader_MI.ExtId = hmotor->EXT_ID.motor_id<<24 | hmotor->EXT_ID.data << 8 |          hmotor->EXT_ID.mode << 5;*/
    uint32_t mailbox;
    /* Start the Transmission process */
    uint32_t ret = HAL_CAN_AddTxMessage(hmotor->phcan, &CAN_TxHeader_MI, hmotor->txdata, &mailbox);
    // if (ret != HAL_OK) {
    //     /* Transmission request Error */
    //     while(1);
    // }
    while (ret != HAL_OK) {
        /* Transmission request Error */
			  HAL_Delay(1);
        ret = HAL_CAN_AddTxMessage(hmotor->phcan, &CAN_TxHeader_MI, hmotor->txdata, &mailbox);
    }
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
  * @brief  小米电机使能
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
  * @param  motor_id 电机id
  * @param  master_id 主机id
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
    hmotor->EXT_ID.mode = 0x12;
    hmotor->EXT_ID.data = MI_MASTERID;
    hmotor->EXT_ID.res = 0;
 
    memcpy(&hmotor->txdata[0],&index,2);
    memcpy(&hmotor->txdata[4],data, 4);
    MI_Motor_CanTx(hmotor);
}
/**
  * @brief  单电机解码
  * @param  hmotor 电机结构体
  * @param  state_byte状态字节，扩展ID的bit8:23
  * @param  rxdata 数据缓冲区
  * @retval null
  */
uint16_t decode_temp_mi = 0;
uint8_t nsvd = 0;
void MI_motor_decode(MI_Motor_t* hmotor,uint8_t state_byte,uint8_t rxdata[]) {
    nsvd = state_byte;
    if((state_byte&0xC0) == 0) {
        hmotor->motor_state = OK;
    } else {
        for(int i = 1; i < 7; i++) {
            if(state_byte&0x01) {
                hmotor->motor_state = i;
            }
            state_byte = state_byte>> 1;
        }
    }
    hmotor->motor_mode = state_byte;
 
    decode_temp_mi = (rxdata[0] << 8 | rxdata[1])^0x8000;
    hmotor->motor_fdb.angle_temp   = decode_temp_mi;
 
    decode_temp_mi = (rxdata[2] << 8 | rxdata[3])^0x8000;
    hmotor->motor_fdb.speed_temp   = decode_temp_mi;
 
    decode_temp_mi = (rxdata[4] << 8 | rxdata[5])^0x8000;
    hmotor->motor_fdb.torque_temp   = decode_temp_mi;
 
    decode_temp_mi = (rxdata[6] << 8 | rxdata[7]);
    hmotor->motor_fdb.temprature_temp  = decode_temp_mi;
 
    hmotor->motor_fdb.angle = (float)hmotor->motor_fdb.angle_temp/32768*4*3.1415926f;
    hmotor->motor_fdb.speed = (float)hmotor->motor_fdb.speed_temp/32768*30;
    hmotor->motor_fdb.torque = (float)hmotor->motor_fdb.torque_temp/32768*12.0f;
    hmotor->motor_fdb.temprature = (float)hmotor->motor_fdb.temprature_temp/10.0f;
 
    hmotor->motor_fdb.last_update_time = HAL_GetTick();
}
/**
  * @brief  小米电机解码
  * @param  rx_EXT_id 接收到的扩展ID
  * @param  rxdata 数据缓冲区
  * @retval null
  */
EXT_ID_t EXT_ID_tmp;//扩展ID数据结构体
void MIMotor_MotorDataDecode(uint32_t rx_EXT_id,uint8_t rxdata[])
{   EXT_ID_tmp = *((EXT_ID_t*)(&rx_EXT_id));
    if(EXT_ID_tmp.mode == 0&&EXT_ID_tmp.motor_id == 0xFE) {
        MI_fdbid = EXT_ID_tmp.data;
        memcpy(MI_MCU_identifier,rxdata, 8);
    }
    if(EXT_ID_tmp.mode == 2) {
        uint8_t id = EXT_ID_tmp.data&0xFF;
        if(id == MI_Motor.EXT_ID.motor_id) {
            MI_motor_decode(&MI_Motor,(uint8_t)(EXT_ID_tmp.data>>8),rxdata);
        }
    }
}

//小企鹅新增部分，以方便人们使用

CAN_receive_t CAN_receive;

/**
  * @brief          hal库CAN回调函数,接收电机数据
  * @param[in]      hcan:CAN句柄指针
  * @retval         none
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

    uint32_t ExtId = rx_header.ExtId;
    CAN_receive.motor_id = ExtId>>21;
    memcpy(CAN_receive.data,rx_data,8);
    // static uint8_t i = 0;
    // //get motor id
    // i = rx_header.StdId - CAN_3508_M1_ID;
    // get_motor_measure(&motor_chassis[i], rx_data);
    // detect_hook(CHASSIS_MOTOR1_TOE + i);


}



/**
  * @brief  所有小米电机初始化
  * @param  
  * @param  
  * @retval 
  */
void MI_motor_all_init()
{

}


/**
  * @brief  小米电机运控模式运行
  * @param  
  * @param  
  * @retval 
  */
void MI_motor_motion_control(double torque)
{
    MI_motor_init(&MI_Motor,&MI_CAN);
    MI_motor_enable(&MI_Motor,1);
    MI_motor_controlmode(&MI_Motor, 0.2, 6 , 5 , 0 , 0);
}


/**
  * @brief  小米电机速度模式运行
  * @param  
  * @param  
  * @retval 
  */
void MI_motor_speed_control()
{
    uint8_t data_temp[4];
    uint32_t data = float_to_uint(5,V_MIN,V_MAX,32);
    data = 0x00000000;
    data_temp[0] = data>>24;
    data_temp[1] = data>>16;
    data_temp[2] = data>>8;
    data_temp[3] = data;
    MI_motor_Write_One_Para(&MI_Motor, 0X700A ,data_temp);
    MI_motor_Read_One_Para(&MI_Motor, 0X7005);
    MI_motor_stop(&MI_Motor);
}



void init_speed_model()
{
    MI_motor_init(&MI_Motor,&MI_CAN);
    uint32_t run_mode = 0x00000002;
    uint8_t data_temp[4];
    data_temp[0] = run_mode>>24;
    data_temp[1] = run_mode>>16;
    data_temp[2] = run_mode>>8;
    data_temp[3] = run_mode;
    MI_motor_Write_One_Para(&MI_Motor, 0X7005 ,data_temp);
    MI_motor_enable(&MI_Motor,1);
}