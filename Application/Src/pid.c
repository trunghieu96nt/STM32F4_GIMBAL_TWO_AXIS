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
  * @brief  ...
  * @note   ...
  * @param  none
  * @retval none
  */
void PID_Init(STRU_PID_T *pidName)
{
  pidName->SetPoint        = 0.0f;
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
  pidName->dPartAlpha      = 0.0f;
  pidName->Ts              = 0.0f;
  pidName->Result          = 0.0f;
  pidName->MaxResponse     = 0.0f;

}

float PID_Calc(STRU_PID_T *pidName, float fFeedback)
{
#ifdef PID_METHOD_1
  float ke, ke_, ke__;
  
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
  * @brief  ...
  * @note   ...
  * @param  none
  * @retval none
  */
void PID_SetPoint_Set(STRU_PID_T *pidName, float fSetPoint, bool bUseRamp)
{
  if(bUseRamp == true)
  {
    if(fSetPoint > (pidName->SetPoint + pidName->MaxSetPointStep))
      pidName->SetPoint += pidName->MaxSetPointStep;
    else if (fSetPoint < (pidName->SetPoint - pidName->MaxSetPointStep))
      pidName->SetPoint -= pidName->MaxSetPointStep;
    else
      pidName->SetPoint = fSetPoint;
  }
  else
    pidName->SetPoint = fSetPoint;
}

float PID_SetPoint_Get(STRU_PID_T *pidName)
{
  return pidName->SetPoint;
}

/**
  * @brief  ...
  * @note   ...
  * @param  none
  * @retval none
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
  * @brief  ...
  * @note   ...
  * @param  none
  * @retval none
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
  * @brief  ...
  * @note   ...
  * @param  none
  * @retval none
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
  * @brief  ...
  * @note   ...
  * @param  none
  * @retval none
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
  * @brief  ...
  * @note   ...
  * @param  none
  * @retval none
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
  * @brief  ...
  * @note   ...
  * @param  none
  * @retval none
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
