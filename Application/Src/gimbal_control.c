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

/* Public variables ----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
STRU_PID_T struPIDManual;
float encAngle;
extern float setpointPIDManual;

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
  //Must set Ts before set Kp Ki Kd
  PID_Init(&struPIDManual);
  PID_Ts_Set(&struPIDManual, 0.001);
  PID_Kp_Set(&struPIDManual, 10);
  PID_Ki_Set(&struPIDManual, 0);
  PID_Kd_Set(&struPIDManual, 0);
  PID_MaxSetPointStep_Set(&struPIDManual, 0.1);
  PID_MaxResponse_Set(&struPIDManual, 400);
}

void Gimbal_Control(void)
{
  //encAngle = Gimbal_ENC0_Get_Pos();
  //encAngle = (float)Gimbal_ENC0_Get_Pos() * 360 / (1000 * 50 * 4);
  //encAngle = Gimbal_ENC1_Get_Pos();
  encAngle = (float)Gimbal_ENC1_Get_Pos() * 360 / (1000 * 50 * 4);
  PID_SetPoint_Set(&struPIDManual, setpointPIDManual, true);
  Gimbal_PWM1_Set_Duty(PID_Calc(&struPIDManual, encAngle));
}
void Gimbal_Control_Home(void)
{
  Az_Home_Rising_Register(Gimbal_Ctl_Init_Az);
  Az_Home_Falling_Register(Gimbal_Ctl_Init_Az);
  El_Home_Rising_Register(Gimbal_Ctl_Init_El);
  El_Home_Falling_Register(Gimbal_Ctl_Init_El);
  if(GPIO_ReadInputDataBit(AZ_HOME_PULSE_PORT, AZ_HOME_PULSE_PIN) == 0)
  {
    Gimbal_PWM1_Set_Duty(75);
  }
  else
  {
    Gimbal_PWM1_Set_Duty(-75);
  }
  if(GPIO_ReadInputDataBit(EL_HOME_PULSE_PORT, EL_HOME_PULSE_PIN) == 0)
  {
    Gimbal_PWM0_Set_Duty(75);
  }
  else
  {
    Gimbal_PWM0_Set_Duty(-75);
  }
}
void Gimbal_Ctl_Init_Az(void)
{
  Gimbal_PWM1_Set_Duty(0);
  Gimbal_ENC1_Reset();
  Az_Home_Rising_Unregister();
  Az_Home_Falling_Unregister();
}

void Gimbal_Ctl_Init_El(void)
{
  Gimbal_PWM0_Set_Duty(0);
  Gimbal_ENC0_Reset();
  El_Home_Rising_Unregister();
  El_Home_Falling_Unregister();
}

/*********************************END OF FILE**********************************/
