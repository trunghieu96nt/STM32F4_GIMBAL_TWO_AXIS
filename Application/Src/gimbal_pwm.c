/**
  ******************************************************************************
  * @file    gimbal_pwm.c
  * @author  Vu Trung Hieu
  * @version V1.0
  * @date    25-June-2017
  * @brief   This file provides firmware functions to use PWM on gimbal board     
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
#include "gimbal_pwm.h"
#include "stm32f4xx.h"

/* Public variables ----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define IS_TIMER_32BIT(x) ((x == TIM2) || (x == TIM5))
#define IS_APB2_TIMER(x) ((x == TIM1) || (x == TIM8) || (x == TIM9) \
   || (x == TIM10) || (x == TIM11))

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
uint32_t Timer_Get_Clock(TIM_TypeDef* TIMx);
void Gimbal_PWM0_Init(void);
void Gimbal_PWM1_Init(void);

/* Functions ---------------------------------------------------------*/
/**
  * @brief  Initialize Gimbal PWM
  * @note   ...
  * @param  none
  * @retval none
  */
void Gimbal_PWM_Init(void)
{
  Gimbal_PWM0_Init();
  Gimbal_PWM1_Init();
  
  Gimbal_PWM0_Set_Duty(0);
  Gimbal_PWM1_Set_Duty(0);
}

/* Functions ---------------------------------------------------------*/
/**
  * @brief  Initialize Gimbal PWM0
  * @note   ...
  * @param  none
  * @retval none
  */
void Gimbal_PWM0_Init(void)
{
  GPIO_InitTypeDef          GPIO_InitStructure;
  TIM_TimeBaseInitTypeDef   TIM_TimeBaseStructure;
  TIM_OCInitTypeDef         TIM_OCInitStructure;

  /* TIM clock enable */
  if (IS_APB2_TIMER(PWM0_TIM))
  {
    RCC_APB2PeriphClockCmd(PWM0_TIM_CLK, ENABLE);
  }
  else
  {
    RCC_APB1PeriphClockCmd(PWM0_TIM_CLK, ENABLE);
  }
  
  /* GPIO clock enable */
  RCC_AHB1PeriphClockCmd(PWM0_PULSE_PORT_CLK, ENABLE);
  RCC_AHB1PeriphClockCmd(PWM0_DIR_PORT_CLK, ENABLE); 

  /* Pulse pin configuration */
  GPIO_InitStructure.GPIO_Pin = PWM0_PULSE;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN ;
  GPIO_Init(PWM0_PULSE_PORT, &GPIO_InitStructure); 
  GPIO_PinAFConfig(PWM0_PULSE_PORT, PWM0_PULSE_SOURCE, PWM0_PULSE_AF);
  
  /* DIR, EN pins configuration */
  GPIO_InitStructure.GPIO_Pin = PWM0_DIR;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(PWM0_DIR_PORT, &GPIO_InitStructure); 

#if PWM0_USE_EN_PIN
  RCC_AHB1PeriphClockCmd(PWM0_EN_PORT_CLK, ENABLE); 
  GPIO_InitStructure.GPIO_Pin = PWM0_EN;
  GPIO_Init(PWM0_EN_PORT, &GPIO_InitStructure);
#endif

  /* TIM CH1 configuration */
  TIM_TimeBaseStructure.TIM_Period = PWM0_PERIOD - 1;
  TIM_TimeBaseStructure.TIM_Prescaler = PWM0_PRESCALER - 1;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(PWM0_TIM, &TIM_TimeBaseStructure);

  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 0;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  TIM_OC1Init(PWM0_TIM, &TIM_OCInitStructure);
  TIM_OC1PreloadConfig(PWM0_TIM, TIM_OCPreload_Enable);

  TIM_Cmd(PWM0_TIM, ENABLE);
}

/* Functions ---------------------------------------------------------*/
/**
  * @brief  Set frequent PWM0
  * @note   ...
  * @param  uint32_t freq
  * @retval none
  */
void Gimbal_PWM0_Set_Freq(uint32_t freq)
{
  uint32_t period;
  period = Timer_Get_Clock(PWM0_TIM) / (freq * (PWM0_TIM->PSC + 1)) - 1;
  if ((!IS_TIMER_32BIT(PWM0_TIM)) && (period > 0xffff))
  {
    period = 0xffff;
  }
  PWM0_TIM->ARR = period;
}

/* Functions ---------------------------------------------------------*/
/**
  * @brief  Set Duty PWM0
  * @note   ...
  * @param  int16_t d: -1000 -> 1000
  * @retval none
  */
void Gimbal_PWM0_Set_Duty(int16_t d)
{
  if (d<-900)
    d = -900;
  else if (d>900)
    d = 900;
  // DIR = 1: OPEN (FORWARD); DIR = 0: CLOSE (BACKWARD)
  if (d == 0)
  {
#if PWM0_USE_EN_PIN
    PWM0_EN_PORT->BSRRH = PWM0_EN;// disable
#endif
  }
  else if (d > 0)
  {
#if PWM0_USE_EN_PIN
    PWM0_EN_PORT->BSRRL = PWM0_EN;// enable
#endif
    PWM0_DIR_PORT->BSRRL = PWM0_DIR; //1, forward
  }
  else //(d<0)
  {
#if PWM0_USE_EN_PIN    
    PWM0_EN_PORT->BSRRL = PWM0_EN; // enable
#endif
    PWM0_DIR_PORT->BSRRH = PWM0_DIR; //0,  backward
    d = -d;
  }
  d = ((PWM0_TIM->ARR + 1)* d + 500)/1000;
  PWM0_TIM->CCR1 = (uint32_t)d;
}

void Gimbal_PWM_EL_Set_Duty(int16_t d)
{
  Gimbal_PWM0_Set_Duty(d);
}

/* Functions ---------------------------------------------------------*/
/**
  * @brief  Initialize Gimbal PWM1
  * @note   ...
  * @param  none
  * @retval none
  */
void Gimbal_PWM1_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
  TIM_OCInitTypeDef TIM_OCInitStructure;

    /* TIM clock enable */
  if (IS_APB2_TIMER(PWM1_TIM))
  {
    RCC_APB2PeriphClockCmd(PWM1_TIM_CLK, ENABLE);
  }
  else
  {
    RCC_APB1PeriphClockCmd(PWM1_TIM_CLK, ENABLE);
  }
  
  /* GPIO clock enable */
  RCC_AHB1PeriphClockCmd(PWM1_PULSE_PORT_CLK, ENABLE);
  RCC_AHB1PeriphClockCmd(PWM1_DIR_PORT_CLK, ENABLE);
  
  /* Pulse pin configuration */
  GPIO_InitStructure.GPIO_Pin = PWM1_PULSE;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN ;
  GPIO_Init(PWM1_PULSE_PORT, &GPIO_InitStructure); 
  GPIO_PinAFConfig(PWM1_PULSE_PORT, PWM1_PULSE_SOURCE, PWM1_PULSE_AF);
  
  /* DIR, EN pins configuration */
  GPIO_InitStructure.GPIO_Pin = PWM1_DIR;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(PWM1_DIR_PORT, &GPIO_InitStructure); 
  
#if PWM1_USE_EN_PIN
  RCC_AHB1PeriphClockCmd(PWM1_EN_PORT_CLK, ENABLE);
  GPIO_InitStructure.GPIO_Pin = PWM1_EN;
  GPIO_Init(PWM1_EN_PORT, &GPIO_InitStructure);
#endif

  /* TIM CH1 configuration */
  TIM_TimeBaseStructure.TIM_Period = PWM1_PERIOD-1;
  TIM_TimeBaseStructure.TIM_Prescaler = PWM1_PRESCALER - 1;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(PWM1_TIM, &TIM_TimeBaseStructure);

  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = PWM1_DUTY;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  TIM_OC1Init(PWM1_TIM, &TIM_OCInitStructure);
  TIM_OC1PreloadConfig(PWM1_TIM, TIM_OCPreload_Enable);

  TIM_Cmd(PWM1_TIM, ENABLE);
    
}

/* Functions ---------------------------------------------------------*/
/**
  * @brief  Set frequent PWM1
  * @note   ...
  * @param  uint32_t freq
  * @retval none
  */
void Gimbal_PWM1_Set_Freq(uint32_t freq)
{
  uint32_t period;
  period = Timer_Get_Clock(PWM1_TIM) / (freq * (PWM1_TIM->PSC + 1)) - 1;
  if ((!IS_TIMER_32BIT(PWM1_TIM)) && (period > 0xffff))
  {
    period = 0xffff;
  }
  PWM1_TIM->ARR = period;
}

/* Functions ---------------------------------------------------------*/
/**
  * @brief  Set Duty PWM1 (AZ)
  * @note   ...
  * @param  int16_t d: -1000 -> 1000
  * @retval none
  */
void Gimbal_PWM1_Set_Duty(int16_t d)
{
  if (d<-900)
    d = -900;
  else if (d>900)
    d = 900;
  // DIR = 1: OPEN (FORWARD); DIR = 0: CLOSE (BACKWARD)
  if (d == 0)
  {
#if PWM1_USE_EN_PIN
    PWM1_EN_PORT->BSRRH = PWM1_EN; // disable
#endif
  }
  else if (d > 0)
  {
#if PWM1_USE_EN_PIN
    PWM1_EN_PORT->BSRRL = PWM1_EN; // enable
#endif
    PWM1_DIR_PORT->BSRRL = PWM1_DIR; //0,  backward
  }
  else // (d<0)
  {
#if PWM1_USE_EN_PIN
    PWM1_EN_PORT->BSRRL = PWM1_EN; // enable
#endif
    PWM1_DIR_PORT->BSRRH = PWM1_DIR; //1, forward
    d = -d;
  }
  d = ((PWM1_TIM->ARR + 1) * d + 500) / 1000;
  PWM1_TIM->CCR1 = (uint32_t)d;
}

void Gimbal_PWM_AZ_Set_Duty(int16_t d)
{
  Gimbal_PWM1_Set_Duty(d);
}

/* Functions ---------------------------------------------------------*/
/**
  * @brief  Get timer clock
  * @note   ...
  * @param  TIM_TypeDef* TIMx
  * @retval uint32_t clk
  */
uint32_t Timer_Get_Clock(TIM_TypeDef* TIMx)
{
  uint32_t clk;
  RCC_ClocksTypeDef RCC_Clocks;  
  /* Get system clocks */
  RCC_GetClocksFreq(&RCC_Clocks);
  if(IS_APB2_TIMER(TIMx))
  {
    if (RCC_Clocks.HCLK_Frequency == RCC_Clocks.PCLK2_Frequency)//prescaler 1
      clk = RCC_Clocks.PCLK2_Frequency;
    else
      clk = 2 * RCC_Clocks.PCLK2_Frequency;
  }
  else//APB1 timer
  {
    if (RCC_Clocks.HCLK_Frequency == RCC_Clocks.PCLK1_Frequency)//prescaler 1
      clk = RCC_Clocks.PCLK1_Frequency;
    else
      clk = 2 * RCC_Clocks.PCLK1_Frequency;
  }
  return clk;
}
/*********************************END OF FILE**********************************/
