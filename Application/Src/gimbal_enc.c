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
#include "stm32f4xx.h"

/* Public variables ----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define IS_TIMER_32BIT(x) ((x == TIM2) || (x == TIM5))
#define IS_APB2_TIMER(x) ((x == TIM1) || (x == TIM8) || (x == TIM9) \
   || (x == TIM10) || (x == TIM11))

/* Private variables ---------------------------------------------------------*/
static volatile int32_t i32Enc0_Cur = 0;
static volatile int32_t i32Enc0_Dp = 0;
static volatile int32_t i32Enc0_P0 = 0,i32Enc0_P1 = 0;

static volatile int32_t i32Enc1_Cur = 0;
static volatile int32_t i32Enc1_Dp = 0;
static volatile int32_t i32Enc1_P0 = 0,i32Enc1_P1 = 0;

/* Private function prototypes -----------------------------------------------*/
void Gimbal_ENC0_Init(void);
void Gimbal_ENC1_Init(void);

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
  Gimbal_ENC1_Init();
  
  Gimbal_ENC0_Reset();
  Gimbal_ENC1_Reset();
}

/* Functions ---------------------------------------------------------*/
/**
  * @brief  Initialize Gimbal ENC0
  * @note   ...
  * @param  none
  * @retval none
  */
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

/* Functions ---------------------------------------------------------*/
/**
  * @brief  Get ENC0 Pos
  * @note   ...
  * @param  none
  * @retval int32_t i32Enc0_Cur
  */
int32_t Gimbal_ENC0_Get_Pos(void)
{
  i32Enc0_P0 = (int32_t)ENC0_TIM_POS->CNT;

  if (!IS_TIMER_32BIT(ENC0_TIM_POS))
  {
    i32Enc0_Dp = i32Enc0_P0 - i32Enc0_P1;
    if (i32Enc0_Dp > 32768)
    {
      i32Enc0_Dp -= 65536;
    }
    else if (i32Enc0_Dp < -32768)
    {
      i32Enc0_Dp += 65536;
    }
    i32Enc0_P1 = i32Enc0_P0;
    i32Enc0_Cur += i32Enc0_Dp;
  }
  else
  {
    i32Enc0_Cur = i32Enc0_P0;
  }
  return i32Enc0_Cur;
}

/* Functions ---------------------------------------------------------*/
/**
  * @brief  Get ENC0 Angle
  * @note   ...
  * @param  none
  * @retval float ENC0 Angle
  */
float Gimbal_ENC0_Get_Angle(void)
{
  return (float)Gimbal_ENC0_Get_Pos() * ENC0_ANGLE_SCALE;
}

/* Functions ---------------------------------------------------------*/
/**
  * @brief  Reset ENC0 Pos
  * @note   Reset i32Enc0_Cur
  * @param  none
  * @retval none
  */
void Gimbal_ENC0_Reset(void)
{
  Gimbal_ENC0_Get_Pos(); //for save pre variable
  i32Enc0_Cur = 0;
}

/* Functions ---------------------------------------------------------*/
/**
  * @brief  Initialize Gimbal ENC1
  * @note   ...
  * @param  none
  * @retval none
  */
void Gimbal_ENC1_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  
  /* TIM clock enable */
  if (IS_APB2_TIMER(ENC1_TIM_POS))
  {
    RCC_APB2PeriphClockCmd(ENC1_TIM_CLK_POS, ENABLE);    
  }
  else
  {
    RCC_APB1PeriphClockCmd(ENC1_TIM_CLK_POS, ENABLE);
  }
  RCC_AHB1PeriphClockCmd(ENC1_PERIPH_PORT_POS, ENABLE);
  /* TIM channel1,2 configuration */
  GPIO_InitStructure.GPIO_Pin   = ENC1_A_POS | ENC1_B_POS;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  GPIO_PinAFConfig(ENC1_PORT_POS, ENC1_A_SOURCE_POS, ENC1_AF_POS);
  GPIO_PinAFConfig(ENC1_PORT_POS, ENC1_B_SOURCE_POS, ENC1_AF_POS);

  /* Initialise encoder interface */
  TIM_EncoderInterfaceConfig(ENC1_TIM_POS, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
  
  TIM_Cmd(ENC1_TIM_POS, ENABLE);  
  ENC1_TIM_POS->CNT = 0;
}

/* Functions ---------------------------------------------------------*/
/**
  * @brief  Get ENC1 Pos
  * @note   ...
  * @param  none
  * @retval int32_t i32Enc1_Cur
  */
int32_t Gimbal_ENC1_Get_Pos(void)
{
  i32Enc1_P0 = (int32_t)ENC1_TIM_POS->CNT;

  if (!IS_TIMER_32BIT(ENC1_TIM_POS))
  {
    i32Enc1_Dp = i32Enc1_P0 - i32Enc1_P1;
    if (i32Enc1_Dp > 32768)
    {
      i32Enc1_Dp -= 65536;
    }
    else if (i32Enc1_Dp < -32768)
    {
      i32Enc1_Dp += 65536;
    }
    i32Enc1_P1 = i32Enc1_P0;
    i32Enc1_Cur += i32Enc1_Dp;
  }
  else
  {
    i32Enc1_Cur = i32Enc1_P0;
  }
  return i32Enc1_Cur;
}

/* Functions ---------------------------------------------------------*/
/**
  * @brief  Get ENC1 Angle
  * @note   ...
  * @param  none
  * @retval float ENC1 Angle
  */
float Gimbal_ENC1_Get_Angle(void)
{
  return (float)Gimbal_ENC1_Get_Pos() * ENC1_ANGLE_SCALE;
}

/* Functions ---------------------------------------------------------*/
/**
  * @brief  Reset ENC1 Pos
  * @note   Reset i32Enc1_Cur
  * @param  none
  * @retval none
  */
void Gimbal_ENC1_Reset(void)
{
  Gimbal_ENC1_Get_Pos(); //for save pre variable
  i32Enc1_Cur = 0;
}

/*********************************END OF FILE**********************************/
