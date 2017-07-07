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
#include "gimbal_gpio.h"
#include "gimbal_pwm.h"

/* Public variables ----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
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
void Gimbal_Control_Home(void)
{
  Az_Home_Rising_Register(Gimbal_Ctl_Init_Az);
  Az_Home_Falling_Register(Gimbal_Ctl_Init_Az);
  El_Home_Rising_Register(Gimbal_Ctl_Init_El);
  El_Home_Falling_Register(Gimbal_Ctl_Init_El);
  if(GPIO_ReadInputDataBit(AZ_HOME_PULSE_PORT, AZ_HOME_PULSE_PIN) == 0)
  {
    Gimbal_PWM1_Set_Duty(-100);
  }
  else
  {
    Gimbal_PWM1_Set_Duty(100);
  }
  if(GPIO_ReadInputDataBit(EL_HOME_PULSE_PORT, EL_HOME_PULSE_PIN) == 0)
  {
    Gimbal_PWM0_Set_Duty(100);
  }
  else
  {
    Gimbal_PWM0_Set_Duty(-100);
  }
}
void Gimbal_Ctl_Init_Az(void)
{
  Gimbal_PWM1_Set_Duty(0);
  Az_Home_Rising_Unregister();
  Az_Home_Falling_Unregister();
}

void Gimbal_Ctl_Init_El(void)
{
  Gimbal_PWM0_Set_Duty(0);
  El_Home_Rising_Unregister();
  El_Home_Falling_Unregister();
}

/*********************************END OF FILE**********************************/
