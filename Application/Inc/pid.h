/**
  ******************************************************************************
  * @file    pid.h
  * @author  Vu Trung Hieu
  * @version V1.0
  * @date    12-July-2017
  * @brief   This file contains all the functions prototypes for pid.c
  ******************************************************************************
  * @attention
  *
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __PID_H
#define __PID_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stdbool.h"
#include "stdint.h"

/* Define --------------------------------------------------------------------*/
#define PID_METHOD_2
#define PID_DEFAULT_SYSTEM_TS             0.001f
#define PID_DEFAULT_D_PART_ALPHA          0.1f
#define PID_DEFAULT_MAX_RESPONSE          900
/* Struct --------------------------------------------------------------------*/
typedef struct
{
  //save to eeprom -> size: sizeof(float) * NumOfFloatVariable
  float Kp;
  float Ki;
  float Kd;
  
  //considerate of saving to eeprom
  uint8_t UseSetPointRamp; //easy to save
  float MaxSetPointStep;
  float DeadBand;
  float dPartAlpha; //for filtering
  float Ts;
  float MaxResponse;
  
  //no need to save to eeprom
  float SetPoint;
  float SetPointBuff;
  float e;
  float e_;
  float e__;
  float pPart;
  float iPart;
  float dPart;
  float dPartRaw;
  float Result;
} STRU_PID_T;
/* Initialization and Configuration functions --------------------------------*/
void PID_Init(STRU_PID_T *pidName);

/* Functions -----------------------------------------------------------------*/
float PID_Calc(STRU_PID_T *pidName, float fFeedback);
void PID_Reset(STRU_PID_T *pidName);

void PID_UseSetPointRamp_Set(STRU_PID_T *pidName, uint8_t ui8UseSetPointRamp);
uint8_t PID_UseSetPointRamp_Get(STRU_PID_T *pidName);

void PID_SetPoint_Set(STRU_PID_T *pidName, float fSetPoint);
float PID_SetPoint_Get(STRU_PID_T *pidName);

void PID_MaxSetPointStep_Set(STRU_PID_T *pidName, float fMaxSetPointStep);
float PID_MaxSetPointStep_Get(STRU_PID_T *pidName);

void PID_DeadBand_Set(STRU_PID_T *pidName, float fDeadBand);
float PID_DeadBand_Get(STRU_PID_T *pidName);

void PID_Kp_Set(STRU_PID_T *pidName, float fKp);
float PID_Kp_Get(STRU_PID_T *pidName);

void PID_Ki_Set(STRU_PID_T *pidName, float fKi);
float PID_Ki_Get(STRU_PID_T *pidName);

void PID_Kd_Set(STRU_PID_T *pidName, float fKd);
float PID_Kd_Get(STRU_PID_T *pidName);

void PID_dPartAlpha_Set(STRU_PID_T *pidName, float fdPartAlpha);
float PID_dPartAlpha_Get(STRU_PID_T *pidName);

void PID_Ts_Set(STRU_PID_T *pidName, float fTs);
float PID_Ts_Get(STRU_PID_T *pidName);

void PID_MaxResponse_Set(STRU_PID_T *pidName, float fMaxResponse);
float PID_MaxResponse_Get(STRU_PID_T *pidName);

#ifdef __cplusplus
}
#endif

#endif /*__PID_H */

/*********************************END OF FILE**********************************/
