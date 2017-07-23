/**
  ******************************************************************************
  * @file    gimbal_params.h
  * @author  Vu Trung Hieu
  * @version V1.0
  * @date    14-July-2017
  * @brief   This file contains all the functions prototypes for gimbal_params.c
  ******************************************************************************
  * @attention
  *
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GIMBAL_PARAMS_H
#define __GIMBAL_PARAMS_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "stdbool.h"
/* Define --------------------------------------------------------------------*/
#define NUM_PARAMS_MAX 20
/* Struct --------------------------------------------------------------------*/
typedef enum{
  CODE_VERSION = 0,
  PARAMS_PID_AZ_MANUAL_POS,
  PARAMS_PID_EL_MANUAL_POS,
} ENUM_PARAMS_T;

/* Union ---------------------------------------------------------------------*/
typedef union {
  float fValue;
  uint8_t byte[4];
} UNION_FLOAT_DATA_T;

/* Initialization and Configuration functions --------------------------------*/
/* Functions -----------------------------------------------------------------*/
void Gimbal_Params_Load_All(void);
void Gimbal_Params_Save_All(void);
bool Gimbal_Params_Save(ENUM_PARAMS_T enumParams, const uint8_t *pu8Data);
bool Gimbal_Params_Load(ENUM_PARAMS_T enumParams, uint8_t *pu8Data);

#ifdef __cplusplus
}
#endif

#endif /*__GIMBAL_PARAMS_H */

/*********************************END OF FILE**********************************/
