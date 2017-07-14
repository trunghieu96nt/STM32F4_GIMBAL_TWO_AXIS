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

/* Struct --------------------------------------------------------------------*/
typedef enum{
  CODE_VERSION = 0,
  KP_POS_AZ,
  KI_POS_AZ,
  KD_POS_AZ,
  KP_POS_EL,
  KI_POS_EL,
  KD_POS_EL,
  KP_VEL_Z,
  KI_VEL_Z,
  KD_VEL_Z,
  KP_VEL_Y,
  KI_VEL_Y,
  KD_VEL_Y,
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
bool Gimbal_Params_Save_Word(ENUM_PARAMS_T enumParams, uint8_t *pui8Data);
bool Gimbal_Params_Save_Float(ENUM_PARAMS_T enumParams, float fValue);
bool Gimbal_Params_Load_Float(ENUM_PARAMS_T enumParams, float *pfValue);
bool Gimbal_Params_Load_Word(ENUM_PARAMS_T enumParams, uint8_t *pui8Data);

#ifdef __cplusplus
}
#endif

#endif /*__GIMBAL_PARAMS_H */

/*********************************END OF FILE**********************************/
