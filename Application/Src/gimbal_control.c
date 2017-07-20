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
#include "gimbal_control.h"
#include "stm32f4xx.h"
#include "pid.h"

#include "gimbal_gpio.h"
#include "gimbal_pwm.h"
#include "gimbal_enc.h"

/* Extern variables ----------------------------------------------------------*/

/* Public variables ----------------------------------------------------------*/
STRU_PID_T stru_PID_AZ_Manual_Pos_Outer;

STRU_PID_T stru_PID_EL_Manual_Pos_Outer;
STRU_PID_T stru_PID_EL_Manual_Vel_Inner;

/* Private define ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
float fAZEncAngle;
static volatile ENUM_AXIS_STATE_T enumAZAxisState = STATE_HOME;
static volatile bool bAZAxisGoingHome = false;

static volatile ENUM_AXIS_STATE_T enumELAxisState = STATE_HOME; //STATE_IDLE
static volatile bool bELAxisGoingHome = false;

/* Private function prototypes -----------------------------------------------*/
void Gimbal_Ctl_Init_Az(void);
void Gimbal_Ctl_Init_El(void);

/* Functions ---------------------------------------------------------*/
/**
  * @brief  ...
  * @note   ...
  * @param  none
  * @retval none
  */
void Gimbal_Control_Init(void)
{
  /* Must set Ts before set Kp Ki Kd */
  /* Manual mode */
  PID_Init(&stru_PID_AZ_Manual_Pos_Outer);
  PID_Kp_Set(&stru_PID_AZ_Manual_Pos_Outer, 40);
  PID_Ki_Set(&stru_PID_AZ_Manual_Pos_Outer, 0.3);
  PID_Kd_Set(&stru_PID_AZ_Manual_Pos_Outer, 0.01);
  PID_UseSetPointRamp_Set(&stru_PID_AZ_Manual_Pos_Outer, 1);
  PID_MaxSetPointStep_Set(&stru_PID_AZ_Manual_Pos_Outer, 0.01);
  
  PID_Init(&stru_PID_EL_Manual_Pos_Outer);
  PID_Init(&stru_PID_EL_Manual_Vel_Inner);
  /* Pointing mode */
  
  /* Tracking mode */
  
}

void Gimbal_Control(void)
{
  
  static float fELEncAngle_ = 0;
  float fELEncAngle, fELEncAngleRate;
  
  switch(enumAZAxisState)
  {
    case STATE_IDLE:
      break;
    case STATE_HOME:
      if(bAZAxisGoingHome == false)
      {
        bAZAxisGoingHome = true;
        Az_Home_Rising_Register(Gimbal_Ctl_Init_Az);
        Az_Home_Falling_Register(Gimbal_Ctl_Init_Az);
        
        if(GPIO_ReadInputDataBit(AZ_HOME_PULSE_PORT, AZ_HOME_PULSE_PIN) == 0)
          Gimbal_PWM1_Set_Duty(75);
        else
          Gimbal_PWM1_Set_Duty(-75);
      }
      break;
    case STATE_MANUAL:
      fAZEncAngle = Gimbal_ENC1_Get_Angle();
      PID_Calc(&stru_PID_AZ_Manual_Pos_Outer, fAZEncAngle);
      //PID_Calc(&stru_PID_AZ_Manual_Pos_Outer, Gimbal_ENC1_Get_Angle());
      
      //Output PWM 1 - AZ
      Gimbal_PWM1_Set_Duty(stru_PID_AZ_Manual_Pos_Outer.Result + 0.5f);
      break;
    default:
      break;
  }
  
  switch(enumELAxisState)
  {
    case STATE_IDLE:
      break;
    case STATE_HOME:
      if(bELAxisGoingHome == false)
      {
        bELAxisGoingHome = true;
        El_Home_Rising_Register(Gimbal_Ctl_Init_El);
        El_Home_Falling_Register(Gimbal_Ctl_Init_El);
        
        if(GPIO_ReadInputDataBit(EL_HOME_PULSE_PORT, EL_HOME_PULSE_PIN) == 0)
          Gimbal_PWM0_Set_Duty(75);
        else
          Gimbal_PWM0_Set_Duty(-75);
      }
      break;
    case STATE_MANUAL:
      //Outer PID -> feedback: angle, result: desired vel for inner PID
      fELEncAngle = Gimbal_ENC0_Get_Angle();
      PID_Calc(&stru_PID_EL_Manual_Pos_Outer, fELEncAngle);
      
      //Inner PID -> feedback: angle rate, result: PWM
      fELEncAngleRate = (fELEncAngle - fELEncAngle_) / stru_PID_EL_Manual_Vel_Inner.Ts;
      PID_SetPoint_Set(&stru_PID_EL_Manual_Vel_Inner, stru_PID_EL_Manual_Pos_Outer.Result);
      PID_Calc(&stru_PID_EL_Manual_Vel_Inner, fELEncAngleRate);
      
      //Output PWM 0 - EL
      Gimbal_PWM0_Set_Duty(stru_PID_EL_Manual_Vel_Inner.Result + 0.5f);
      
      //Update pre angle
      fELEncAngle_ = fELEncAngle;
      break;
    default:
      break;
  }
}

void Gimbal_Ctl_Init_Az(void)
{
  Gimbal_PWM1_Set_Duty(0);
  Gimbal_ENC1_Get_Pos();
  Gimbal_ENC1_Reset();
  Az_Home_Rising_Unregister();
  Az_Home_Falling_Unregister();
  enumAZAxisState = STATE_MANUAL;
  PID_SetPoint_Set(&stru_PID_AZ_Manual_Pos_Outer, Gimbal_ENC1_Get_Angle());
  bAZAxisGoingHome = false;
}

void Gimbal_Ctl_Init_El(void)
{
  Gimbal_PWM0_Set_Duty(0);
  Gimbal_ENC0_Get_Pos();
  Gimbal_ENC0_Reset();
  El_Home_Rising_Unregister();
  El_Home_Falling_Unregister();
  enumELAxisState = STATE_IDLE;
  //PID_SetPoint_Set(&stru_PID_EL_Manual_Pos_Outer, Gimbal_ENC0_Get_Angle());
  bELAxisGoingHome = false;
}

/*********************************END OF FILE**********************************/
