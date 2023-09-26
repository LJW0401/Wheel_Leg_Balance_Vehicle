/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       oled_task.c/h
  * @brief      OLED show error value.oled屏幕显示错误码
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Nov-11-2019     RM              1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
#ifndef OLED_TASK_H
#define OLED_TASK_H
#include "struct_typedef.h"



/**
  * @brief          oled task
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
/**
  * @brief          oled任务
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
extern void oled_task(void const * argument);

#endif
