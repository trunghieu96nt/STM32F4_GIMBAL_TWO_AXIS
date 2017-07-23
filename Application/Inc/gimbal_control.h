/**
  ******************************************************************************
  * @file    gimbal_adis.h
  * @author  Vu Trung Hieu
  * @version V1.0
  * @date    2-July-2017
  * @brief   This file contains all the functions prototypes 
  *          for gimbal_control.c
  ******************************************************************************
  * @attention
  *
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GIMBAL_CONTROL_H
#define __GIMBAL_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
/* Define --------------------------------------------------------------------*/
/* Struct --------------------------------------------------------------------*/
/* Enum ----------------------------------------------------------------------*/

typedef enum {
  STATE_KEEP = -1,
  STATE_IDLE,
  STATE_HOME,
  STATE_MANUAL,
  STATE_POINTING,
  STATE_TRACKING,
  STATE_TEST_SINE,
} ENUM_AXIS_STATE_T;

/* Initialization and Configuration functions --------------------------------*/
/* Functions -----------------------------------------------------------------*/
void Gimbal_Control_Init(void);
void Gimbal_Control(void);
void Gimbal_Control_Change_Mode(ENUM_AXIS_STATE_T enumAZState, ENUM_AXIS_STATE_T enumELState);

#ifdef __cplusplus
}
#endif

#endif /*__GIMBAL_CONTROL_H */

/*********************************END OF FILE**********************************/
