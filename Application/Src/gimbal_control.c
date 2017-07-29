/**
  ******************************************************************************
  * @file    gimbal_control.c
  * @author  Vu Trung Hieu
  * @version V1.0
  * @date    7-July-2017
  * @brief   This file provides firmware functions to control system
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
#include <math.h>

#include "gimbal_control.h"
#include "stm32f4xx.h"
#include "pid.h"

#include "gimbal_gpio.h"
#include "gimbal_pwm.h"
#include "gimbal_enc.h"
#include "gimbal_adis.h"
#include "gimbal_utils.h"

/* Extern variables ----------------------------------------------------------*/
extern STRU_IMU_DATA_T struIMUData;

/* Public variables ----------------------------------------------------------*/
STRU_PID_T stru_PID_AZ_Manual_Pos;
STRU_PID_T stru_PID_EL_Manual_Pos;

STRU_PID_T stru_PID_AZ_Pointing_Outer;
STRU_PID_T stru_PID_AZ_Pointing_Inner;

/* Private define ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
float fAZEncAngle, fELEncAngle;
volatile ENUM_AXIS_STATE_T enumAZAxisState = STATE_HOME; //STATE_HOME STATE_SINE
volatile ENUM_AXIS_STATE_T enumELAxisState = STATE_IDLE; //STATE_IDLE
static volatile bool bAZAxisGoingHome = false;
static volatile bool bELAxisGoingHome = false;

/* Private function prototypes -----------------------------------------------*/
void Gimbal_Ctl_Init_Az(void);
void Gimbal_Ctl_Init_El(void);
void Gimbal_Ctl_Limit_El(void);

/* Functions ---------------------------------------------------------*/
/**
  * @brief  ...
  * @note   ...
  * @param  none
  * @retval none
  */
void Gimbal_Control_Init(void)
{
  /* Limit EL*/
  if(GPIO_ReadInputDataBit(EL_LIMIT_PULSE_PORT, EL_LIMIT_PULSE_PIN) == 1)
    Gimbal_Ctl_Limit_El();
  El_Limit_Falling_Register(Gimbal_Ctl_Limit_El);
  El_Limit_Rising_Register(Gimbal_Ctl_Limit_El);
  
  /* Manual mode */
  PID_Init(&stru_PID_AZ_Manual_Pos);
  PID_Kp_Set(&stru_PID_AZ_Manual_Pos, 40);
  PID_Ki_Set(&stru_PID_AZ_Manual_Pos, 0.3);
  PID_Kd_Set(&stru_PID_AZ_Manual_Pos, 0.01);
  PID_UseSetPointRamp_Set(&stru_PID_AZ_Manual_Pos, 1);
  PID_MaxSetPointStep_Set(&stru_PID_AZ_Manual_Pos, 0.01);
  
  PID_Init(&stru_PID_EL_Manual_Pos);
  PID_Kp_Set(&stru_PID_EL_Manual_Pos, 41);
  PID_Ki_Set(&stru_PID_EL_Manual_Pos, 0.5);
  PID_Kd_Set(&stru_PID_EL_Manual_Pos, 0.01);
  PID_UseSetPointRamp_Set(&stru_PID_EL_Manual_Pos, 1);
  PID_MaxSetPointStep_Set(&stru_PID_EL_Manual_Pos, 0.01);
  PID_MaxResponse_Set(&stru_PID_EL_Manual_Pos, 200);
  
  /* Pointing mode */
  PID_Init(&stru_PID_AZ_Pointing_Inner);
  PID_dPartAlpha_Set(&stru_PID_AZ_Pointing_Inner, 0.5);
  PID_DeadBand_Set(&stru_PID_AZ_Pointing_Inner, 100);
  //PID_MaxResponse_Set(&stru_PID_AZ_Pointing_Inner, 900);
  
  /* Tracking mode */
  
}

void Gimbal_Control(void)
{
  static uint32_t ui32SineIdx = 0;
  fAZEncAngle = Gimbal_ENC1_Get_Angle();
  switch(enumAZAxisState)
  {
    case STATE_IDLE:
    { 
      break;
    }
    case STATE_HOME:
      if(bAZAxisGoingHome == false)
      {
        bAZAxisGoingHome = true;
        Az_Home_Rising_Register(Gimbal_Ctl_Init_Az);
        Az_Home_Falling_Register(Gimbal_Ctl_Init_Az);
        
        if(GPIO_ReadInputDataBit(AZ_HOME_PULSE_PORT, AZ_HOME_PULSE_PIN) == 0)
          Gimbal_PWM_AZ_Set_Duty(-75);
        else
          Gimbal_PWM_AZ_Set_Duty(75);
      }
      break;
    case STATE_MANUAL:
      fAZEncAngle = Gimbal_ENC1_Get_Angle();
      PID_Calc(&stru_PID_AZ_Manual_Pos, fAZEncAngle);
      //PID_Calc(&stru_PID_AZ_Manual_Pos, Gimbal_ENC1_Get_Angle());
      
      //Output PWM 1 - AZ
      Gimbal_PWM_AZ_Set_Duty(stru_PID_AZ_Manual_Pos.Result + 0.5f);
      break;
    case STATE_POINTING:
      
      //stru_PID_AZ_Pointing_Inner.Result = -stru_PID_AZ_Pointing_Inner.Kp * struIMUData.gyro_z;
      PID_Calc(&stru_PID_AZ_Pointing_Inner, struIMUData.gyro_z);
      //Output PWM 1 - AZ
      Gimbal_PWM_AZ_Set_Duty(stru_PID_AZ_Pointing_Inner.Result + 0.5f);
      break;
    case STATE_SINE:
      Gimbal_PWM_AZ_Set_Duty(300*sin(2 * PI * ui32SineIdx/ 5000));
      if(++ui32SineIdx == 5000)  ui32SineIdx = 0;
      break;
    default:
      break;
  }
  
  switch(enumELAxisState)
  {
    case STATE_IDLE:
      fELEncAngle = Gimbal_ENC0_Get_Angle();
      break;
    case STATE_HOME:
      if(bELAxisGoingHome == false)
      {
        bELAxisGoingHome = true;
        El_Home_Rising_Register(Gimbal_Ctl_Init_El);
        El_Home_Falling_Register(Gimbal_Ctl_Init_El);
        
        if(GPIO_ReadInputDataBit(EL_HOME_PULSE_PORT, EL_HOME_PULSE_PIN) == 0)
          Gimbal_PWM_EL_Set_Duty(110);
        else
          Gimbal_PWM_EL_Set_Duty(-110);
      }
      break;
    case STATE_MANUAL:
      fELEncAngle = Gimbal_ENC0_Get_Angle();
      PID_Calc(&stru_PID_EL_Manual_Pos, fELEncAngle);
      //PID_Calc(&stru_PID_EL_Manual_Pos, Gimbal_ENC0_Get_Angle());
      
      //Output PWM 0 - EL
      Gimbal_PWM_EL_Set_Duty(stru_PID_EL_Manual_Pos.Result + 0.5f);
      break;
    default:
      break;
  }
}

void Gimbal_Control_Change_Mode(ENUM_AXIS_STATE_T enumAZState, ENUM_AXIS_STATE_T enumELState)
{
  if(enumAZState != STATE_KEEP)
  {
    switch(enumAZState)
    {
      case STATE_IDLE:
        Gimbal_PWM_AZ_Set_Duty(0);
        break;
      case STATE_HOME:
        Gimbal_PWM_AZ_Set_Duty(0);
        bAZAxisGoingHome = false;
        break;
      case STATE_MANUAL:
        PID_Reset(&stru_PID_AZ_Manual_Pos);
        // set SetPointBuff
        PID_SetPoint_Set(&stru_PID_AZ_Manual_Pos, Gimbal_ENC1_Get_Angle());
        PID_UseSetPointRamp_Set(&stru_PID_AZ_Manual_Pos, 0); 
        // set SetPoint
        PID_SetPoint_Set(&stru_PID_AZ_Manual_Pos, Gimbal_ENC1_Get_Angle());
        //Re-enable useRamp
        PID_UseSetPointRamp_Set(&stru_PID_AZ_Manual_Pos, 1);
        break;
      case STATE_POINTING:
        PID_Reset(&stru_PID_AZ_Pointing_Outer);
        PID_Reset(&stru_PID_AZ_Pointing_Inner);
        break;
      case STATE_TRACKING:
        break;
      default:
        break;
    }
    enumAZAxisState = enumAZState;
  }
  if(enumELState != STATE_KEEP)
  {
    switch(enumELState)
    {
      case STATE_IDLE:
        Gimbal_PWM_EL_Set_Duty(0);
        break;
      case STATE_HOME:
        Gimbal_PWM_EL_Set_Duty(0);
        bELAxisGoingHome = false;
        break;
      case STATE_MANUAL:
        PID_Reset(&stru_PID_EL_Manual_Pos);
        // set SetPointBuff
        PID_SetPoint_Set(&stru_PID_EL_Manual_Pos, Gimbal_ENC0_Get_Angle());
        PID_UseSetPointRamp_Set(&stru_PID_EL_Manual_Pos, 0); 
        // set SetPoint
        PID_SetPoint_Set(&stru_PID_EL_Manual_Pos, Gimbal_ENC0_Get_Angle());
        //Re-enable useRamp
        PID_UseSetPointRamp_Set(&stru_PID_EL_Manual_Pos, 1);
        break;
      case STATE_POINTING:
        break;
      case STATE_TRACKING:
        break;
      default:
        break;
    }
    enumELAxisState = enumELState;
  }
}

void Gimbal_Ctl_Init_Az(void)
{
  Gimbal_PWM_AZ_Set_Duty(0);
  Gimbal_ENC1_Reset();
  Az_Home_Rising_Unregister();
  Az_Home_Falling_Unregister();
  Gimbal_Control_Change_Mode(STATE_MANUAL, STATE_KEEP);
  bAZAxisGoingHome = false;
}

void Gimbal_Ctl_Init_El(void)
{
  Gimbal_PWM_EL_Set_Duty(0);
  Gimbal_ENC0_Reset();
  El_Home_Rising_Unregister();
  El_Home_Falling_Unregister();
  Gimbal_Control_Change_Mode(STATE_KEEP, STATE_MANUAL);
  bELAxisGoingHome = false;
}

void Gimbal_Ctl_Limit_El(void)
{
  Gimbal_Control_Change_Mode(STATE_IDLE, STATE_IDLE);
  while(true); //Loop forever
}

/*********************************END OF FILE**********************************/
