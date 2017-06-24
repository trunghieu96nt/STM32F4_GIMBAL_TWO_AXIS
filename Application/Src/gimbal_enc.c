/**
  ******************************************************************************
  * @file    gimbal_enc.c
  * @author  Vu Trung Hieu
  * @version V1.0
  * @date    24-June-2017
  * @brief   This file provides firmware functions to use ENC on gimbal board     
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
#include "gimbal_enc.h"
#include "stm32f4xx_gpio.h"

/* Public variables ----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
void Gimbal_ENC0_Init(void);

/* Functions ---------------------------------------------------------*/
/**
  * @brief  Initialize Gimbal ENC
  * @note   ...
  * @param  none
  * @retval none
  */
void Gimbal_ENC_Init(void)
{
  Gimbal_ENC0_Init();
}

void Gimbal_ENC0_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  /* TIM clock enable */
  if (IS_APB2_TIMER(ENC0_TIM_POS))
  {
    RCC_APB2PeriphClockCmd(ENC0_TIM_CLK_POS, ENABLE);
  }
  else
  {
    RCC_APB1PeriphClockCmd(ENC0_TIM_CLK_POS, ENABLE);    
  }
  RCC_AHB1PeriphClockCmd(ENC0_PERIPH_PORT_POS, ENABLE);
  /* TIM channel1,2 configuration */
  GPIO_InitStructure.GPIO_Pin   = ENC0_A_POS | ENC0_B_POS;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
  GPIO_Init(ENC0_PORT_POS, &GPIO_InitStructure);

  GPIO_PinAFConfig(ENC0_PORT_POS, ENC0_A_SOURCE_POS, ENC0_AF_POS);
  GPIO_PinAFConfig(ENC0_PORT_POS, ENC0_B_SOURCE_POS, ENC0_AF_POS);

  /* Initialise encoder interface */
  TIM_EncoderInterfaceConfig(ENC0_TIM_POS, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
  
  /* TIM enable counter */
  TIM_Cmd(ENC0_TIM_POS, ENABLE);  
  ENC0_TIM_POS->CNT = 0;
}

/*********************************END OF FILE**********************************/
