/**
  ******************************************************************************
  * @file    pid.c
  * @author  Vu Trung Hieu
  * @version V1.0
  * @date    12-July-2017
  * @brief   This file provides functions to use pid controller
  *           + 
  *           + 
  * 
@verbatim  
 ===============================================================================
                      ##### How to use this driver #####
 ===============================================================================
   (#) 
   (#)          
   (#) 
       (--) 
       (--) 
       (++) 
       (++) 
@endverbatim        
  *
  ******************************************************************************
  * @attention
  * (#) Use STM32F4xx_StdPeriph_Driver
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "pid.h"
#include "math.h"

/* Public variables ----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Functions ---------------------------------------------------------*/
/**
  * @brief  Init PID
  * @note   Some params is set to default
  * @param  STRU_PID_T *pidName
  * @retval none
  */
void PID_Init(STRU_PID_T *pidName)
{
  pidName->UseSetPointRamp = 0;
  pidName->SetPoint        = 0.0f;
  pidName->SetPointBuff    = 0.0f;
  pidName->MaxSetPointStep = 0.0f;
  pidName->DeadBand        = 0.0f;
  pidName->e               = 0.0f;
  pidName->e_              = 0.0f;
  pidName->e__             = 0.0f;
  pidName->Kp              = 0.0f;
  pidName->Ki              = 0.0f;
  pidName->Kd              = 0.0f;
  pidName->pPart           = 0.0f;
  pidName->iPart           = 0.0f;
  pidName->dPart           = 0.0f;
  pidName->dPartRaw        = 0.0f;
  pidName->dPartAlpha      = PID_DEFAULT_D_PART_ALPHA;
  pidName->Ts              = PID_DEFAULT_SYSTEM_TS;
  pidName->Result          = 0.0f;
  pidName->MaxResponse     = PID_DEFAULT_MAX_RESPONSE;

}

/**
  * @brief  Calculate PID
  * @note   ...
  * @param  STRU_PID_T *pidName
  * @param  float fFeedback
  * @retval pidName->Result
  */
float PID_Calc(STRU_PID_T *pidName, float fFeedback)
{
#ifdef PID_METHOD_1
  float ke, ke_, ke__;
  
  if(pidName->UseSetPointRamp != 0) //true
  {
    if(pidName->SetPointBuff > (pidName->SetPoint + pidName->MaxSetPointStep))
      pidName->SetPoint += pidName->MaxSetPointStep;
    else if (pidName->SetPointBuff < (pidName->SetPoint - pidName->MaxSetPointStep))
      pidName->SetPoint -= pidName->MaxSetPointStep;
    else
      pidName->SetPoint = pidName->SetPointBuff;
  }
  
  pidName->e = pidName->SetPoint - fFeedback;
  if(fabsf(pidName->e) < pidName->DeadBand) pidName->e = 0;
  
  ke   = pidName->Kp + (pidName->Ki / 2) + pidName->Kd;
  ke_  = -pidName->Kp + (pidName->Ki / 2) - (2 * pidName->Kd);
  ke__ = pidName->Kd;
  
  pidName->Result += (ke * pidName->e) + (ke_ * pidName->e_) + (ke__ * pidName->e__);
  
  pidName->e__ = pidName->e_;
  pidName->e_ =pidName->e;
  
  if(pidName->Result > pidName->MaxResponse)
    pidName->Result = pidName->MaxResponse;
  else if(pidName->Result < -pidName->MaxResponse)
    pidName->Result = -pidName->MaxResponse;
  
  return pidName->Result;
#endif

#ifdef PID_METHOD_2
  if(pidName->UseSetPointRamp != 0) //true
  {
    if(pidName->SetPointBuff > (pidName->SetPoint + pidName->MaxSetPointStep))
      pidName->SetPoint += pidName->MaxSetPointStep;
    else if (pidName->SetPointBuff < (pidName->SetPoint - pidName->MaxSetPointStep))
      pidName->SetPoint -= pidName->MaxSetPointStep;
    else
      pidName->SetPoint = pidName->SetPointBuff;
  }
  
  pidName->e = pidName->SetPoint - fFeedback;
  if(fabsf(pidName->e) < pidName->DeadBand) pidName->e = 0;
  
  //pPart
  pidName->pPart = pidName->Kp * pidName->e;
  
  //iPart
  pidName->iPart += pidName->Ki * (pidName->e + pidName->e_) / 2;
  if (pidName->iPart > pidName->MaxResponse)
    pidName->iPart = pidName->MaxResponse;
  else if (pidName->iPart < -pidName->MaxResponse) 
    pidName->iPart = -pidName->MaxResponse;
  
  //dPart
  pidName->dPartRaw = pidName->Kd * (pidName->e - pidName->e_);
  pidName->dPart = pidName->dPart + pidName->dPartAlpha * (pidName->dPartRaw - pidName->dPart);
  
  if (pidName->dPart > (pidName->MaxResponse / 2))
    pidName->dPart = pidName->MaxResponse / 2;
  else if (pidName->dPart < (-pidName->MaxResponse / 2))
    (*pidName).dPart = -pidName->MaxResponse / 2;
  
  pidName->e_ =pidName->e;
  
  pidName->Result = pidName->pPart + pidName->iPart + pidName->dPart;
  return pidName->Result;
#endif
}

/**
  * @brief  Reset PID
  * @note   ...
  * @param  STRU_PID_T *pidName
  * @retval none
  */
void PID_Reset(STRU_PID_T *pidName)
{
  pidName->e        = 0.0f;
  pidName->e_       = 0.0f;
  pidName->e__      = 0.0f;
  pidName->pPart    = 0.0f;
  pidName->iPart    = 0.0f;
  pidName->dPart    = 0.0f;
  pidName->dPartRaw = 0.0f;
  pidName->Result   = 0.0f;
}

/**
  * @brief  Set UseSetPointRamp (Get UseSetPointRamp)
  * @note   ...
  * @param  STRU_PID_T *pidName
  * @param  uint8_t ui8UseSetPointRamp
  * @retval none (pidName->SetPoint)
  */
void PID_UseSetPointRamp_Set(STRU_PID_T *pidName, uint8_t ui8UseSetPointRamp)
{
  pidName->UseSetPointRamp = ui8UseSetPointRamp;
}

uint8_t PID_UseSetPointRamp_Get(STRU_PID_T *pidName)
{
  return pidName->UseSetPointRamp;
}

/**
  * @brief  Set setpoint (Get setpoint)
  * @note   ...
  * @param  STRU_PID_T *pidName
  * @param  float fSetPoint
  * @param  bool bUseRamp -> if true this function must call before when call PID_Calc
            this limit the accelerometer
  * @retval none (pidName->SetPoint)
  */
void PID_SetPoint_Set(STRU_PID_T *pidName, float fSetPoint)
{
  if(pidName->UseSetPointRamp != 0) //true
  {
    pidName->SetPointBuff = fSetPoint;
  }
  else
    pidName->SetPoint = fSetPoint;
}

float PID_SetPoint_Get(STRU_PID_T *pidName)
{
  return pidName->SetPoint;
}

/**
  * @brief  Set MaxSetPointStep (get)
  * @note   This is only work if bUseRamp == true when call PID_SetPoint_Set
  * @param  STRU_PID_T *pidName
  * @param  float fMaxSetPointStep
  * @retval none (pidName->MaxSetPointStep)
  */
void PID_MaxSetPointStep_Set(STRU_PID_T *pidName, float fMaxSetPointStep)
{
  pidName->MaxSetPointStep = fMaxSetPointStep;
}

float PID_MaxSetPointStep_Get(STRU_PID_T *pidName)
{
  return pidName->MaxSetPointStep;
}

/**
  * @brief  Set DeadBand (Get)
  * @note   Dead band of e
  * @param  STRU_PID_T *pidName
  * @param  float fDeadBand
  * @retval pidName->DeadBand
  */
void PID_DeadBand_Set(STRU_PID_T *pidName, float fDeadBand)
{
  pidName->DeadBand = fDeadBand;
}

float PID_DeadBand_Get(STRU_PID_T *pidName)
{
  return pidName->DeadBand;
}

/**
  * @brief  Set Kp Ki Kd (Get)
  * @note   ...
  * @param  STRU_PID_T *pidName
  * @param  float fKx (x = p i d)
  * @retval none (pidName->Kx = fKx (x = p i d))
  */
void PID_Kp_Set(STRU_PID_T *pidName, float fKp)
{
  pidName->Kp = fKp;
}

float PID_Kp_Get(STRU_PID_T *pidName)
{
  return pidName->Kp;
}

void PID_Ki_Set(STRU_PID_T *pidName, float fKi)
{
  pidName->Ki = fKi * pidName->Ts;
}

float PID_Ki_Get(STRU_PID_T *pidName)
{
  return pidName->Ki;
}

void PID_Kd_Set(STRU_PID_T *pidName, float fKd)
{
  pidName->Kd = fKd / pidName->Ts;
}

float PID_Kd_Get(STRU_PID_T *pidName)
{
  return pidName->Kd;
}

/**
  * @brief  Set d part alpha (get)
  * @note   This is low filter for dPart
  * @param  STRU_PID_T *pidName
  * @param  float fdPartAlpha
  * @retval none (pidName->dPartAlpha)
  */
void PID_dPartAlpha_Set(STRU_PID_T *pidName, float fdPartAlpha)
{
  pidName->dPartAlpha = fdPartAlpha;
}

float PID_dPartAlpha_Get(STRU_PID_T *pidName)
{
  return pidName->dPartAlpha;
}

/**
  * @brief  Set Ts (Get)
  * @note   ...
  * @param  STRU_PID_T *pidName
  * @param  float fTs
  * @retval none (pidName->Ts)
  */
void PID_Ts_Set(STRU_PID_T *pidName, float fTs)
{
  pidName->Ts = fTs;
}

float PID_Ts_Get(STRU_PID_T *pidName)
{
  return pidName->Ts;
}

/**
  * @brief  Set Max Response (Get)
  * @note   Limit the pidName->Result
  * @param  STRU_PID_T *pidName
  * @param  float fMaxResponse
  * @retval none (pidName->MaxResponse)
  */
void PID_MaxResponse_Set(STRU_PID_T *pidName, float fMaxResponse)
{
  pidName->MaxResponse = fMaxResponse;
}

float PID_MaxResponse_Get(STRU_PID_T *pidName)
{
  return pidName->MaxResponse;
}
/*********************************END OF FILE**********************************/
