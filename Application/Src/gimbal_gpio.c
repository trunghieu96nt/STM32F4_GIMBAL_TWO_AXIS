/**
  ******************************************************************************
  * @file    gimbal_gpio.c
  * @author  Vu Trung Hieu
  * @version V1.0
  * @date    24-June-2017
  * @brief   This file provides firmware functions to use GPIO on gimbal board     
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
#include "gimbal_gpio.h"
#include "stm32f4xx.h"

/* Public variables ----------------------------------------------------------*/
static PulseHandle_t El_Home_Rising_Handler = 0;
static PulseHandle_t El_Home_Falling_Handler = 0;
static PulseHandle_t Az_Home_Rising_Handler = 0;
static PulseHandle_t Az_Home_Falling_Handler = 0;
static PulseHandle_t El_Limit_Rising_Handler = 0;
static PulseHandle_t El_Limit_Falling_Handler = 0;

/* Private define ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
void Gimbal_Led_Init(void);
void Gimbal_SW_Init(void);
void Gimbal_DI_Init(void);
void Gimbal_Home_Pulse_Init(void);
void Gimbal_El_Limit_Init(void);

/* Functions -----------------------------------------------------------------*/
/**
  * @brief  Initialize Gimbal GPIO
  * @note   ...
  * @param  none
  * @retval none
  */
void Gimbal_GPIO_Init(void)
{
  Gimbal_Led_Init();
  Gimbal_Led_Reset(LED1_PIN);
  Gimbal_Led_Reset(LED2_PIN);
  Gimbal_Led_Reset(LED3_PIN);
  //Gimbal_SW_Init();
  //Gimbal_DI_Init();
  Gimbal_Home_Pulse_Init();
  Gimbal_El_Limit_Init();
}

/* Led Functions --------------------------------*/
/**
  * @brief  Initialize LED1, LED2, LED3
  * @note   define LED_PERIPH_GPIO, LEDx_PIN, LED_GPIO in gimbal_gpio.h
  * @param  none
  * @retval none
  */
void Gimbal_Led_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  /* GPIOD Peripheral clock enable */
  RCC_AHB1PeriphClockCmd(LED_PERIPH_GPIO, ENABLE);
  
  /* Configure LED1, LED2, LED3 in output pushpull mode */
  GPIO_InitStructure.GPIO_Pin   = LED1_PIN | LED2_PIN | LED3_PIN;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
  GPIO_Init(LED_GPIO, &GPIO_InitStructure);
}

/**
  * @brief  Set LED1, LED2, LED3 (led pin is high)
  * @note   define LED_GPIO in gimbal_gpio.h
  * @param  
  *     uint16_t LEDx_Pin: LED1_PIN, LED2_PIN, LED3_PIN
  * @retval none
  */
void Gimbal_Led_Set(uint16_t LEDx_Pin)
{
  GPIO_SetBits(LED_GPIO, LEDx_Pin);
}

/**
  * @brief  Set LED1, LED2, LED3 (led pin is low)
  * @note   define LED_GPIO in gimbal_gpio.h
  * @param  
  *     uint16_t LEDx_Pin: LED1_PIN, LED2_PIN, LED3_PIN
  * @retval none
  */
void Gimbal_Led_Reset(uint16_t LEDx_Pin)
{
  GPIO_ResetBits(LED_GPIO, LEDx_Pin);
}

/**
  * @brief  Toggle LED1, LED2, LED3
  * @note   define LED_GPIO in gimbal_gpio.h
  * @param  
  *     uint16_t LEDx_Pin: LED1_PIN, LED2_PIN, LED3_PIN
  * @retval none
  */
void Gimbal_Led_Toggle(uint16_t LEDx_Pin)
{
  GPIO_ToggleBits(LED_GPIO, LEDx_Pin);
}

/* SW & DI Functions ----------------------------*/
/**
  * @brief  Initialize SW1, SW2
  * @note   define SW_PERIPH_GPIO, SWx_PIN, SW_GPIO in gimbal_gpio.h
  * @param  none
  * @retval none
  */
void Gimbal_SW_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  /* GPIOD Peripheral clock enable */
  RCC_AHB1PeriphClockCmd(SW_PERIPH_GPIO, ENABLE);
  
  /* Configure LED1, LED2, LED3 in output pushpull mode */
  GPIO_InitStructure.GPIO_Pin   = SW1_PIN | SW2_PIN;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
  GPIO_Init(SW_GPIO, &GPIO_InitStructure);
}

/**
  * @brief  Initialize DI
  * @note   define DI_PERIPH_GPIO, SWx_PIN, SW_GPIO in gimbal_gpio.h
  * @param  none
  * @retval none
  */
void Gimbal_DI_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  /* GPIOD Peripheral clock enable */
  RCC_AHB1PeriphClockCmd(DI_PERIPH_GPIO, ENABLE);
  
  /* Configure LED1, LED2, LED3 in output pushpull mode */
  GPIO_InitStructure.GPIO_Pin   = DI1_PIN | DI2_PIN | DI3_PIN | DI4_PIN;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
  GPIO_Init(DI_GPIO, &GPIO_InitStructure);
}

/* Home Pulse Functions--------------------------*/
/**
  * @brief  Init home pulse
  * @note   ...
  * @param  none
  * @retval none
  */
void Gimbal_Home_Pulse_Init(void)
{
  EXTI_InitTypeDef   EXTI_InitStructure;
  GPIO_InitTypeDef   GPIO_InitStructure;
  NVIC_InitTypeDef   NVIC_InitStructure;
  /* Config EL Home Pulse*/
  /* Enable GPIO clock */
  RCC_AHB1PeriphClockCmd(EL_HOME_PULSE_PORT_CLK, ENABLE);
  /* Enable SYSCFG clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
  /* Configure pin as input */
  GPIO_InitStructure.GPIO_Pin   = EL_HOME_PULSE_PIN;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
  GPIO_Init(EL_HOME_PULSE_PORT, &GPIO_InitStructure);
  /* Connect EXTI Line2 to EL pin */
  SYSCFG_EXTILineConfig(EL_EXTI_PORT_SOURCE, EL_HOME_PULSE_PIN_SOURCE);
  /* Configure EXTI EL - rising & falling when home */
  EXTI_InitStructure.EXTI_Line    = EL_EXTI_LINE;
  EXTI_InitStructure.EXTI_Mode    = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
  /* Enable and set EXTI EL Interrupt to the lowest priority */
  NVIC_InitStructure.NVIC_IRQChannel = EL_EXTI_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority  = 2;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  /* Config AZ Home Pulse*/
  /* Enable GPIO clock */
  RCC_AHB1PeriphClockCmd(AZ_HOME_PULSE_PORT_CLK, ENABLE);
  /* Enable SYSCFG clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
  /* Configure pin as input */
  GPIO_InitStructure.GPIO_Pin   = AZ_HOME_PULSE_PIN;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
  GPIO_Init(AZ_HOME_PULSE_PORT, &GPIO_InitStructure);
  /* Connect EXTI Line1 to EL pin */
  SYSCFG_EXTILineConfig(AZ_EXTI_PORT_SOURCE, AZ_HOME_PULSE_PIN_SOURCE);
  /* Configure EXTI EL - rising & falling when home */
  EXTI_InitStructure.EXTI_Line    = AZ_EXTI_LINE;
  EXTI_InitStructure.EXTI_Mode    = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
  /* Enable and set EXTI EL Interrupt to the lowest priority */
  NVIC_InitStructure.NVIC_IRQChannel = AZ_EXTI_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);  
}

/**
  * @brief  Functions to register & unregister handle exti
  * @note   ...
  * @param  with register: PulseHandle_t f (handler function)
  * @retval none
  */
void El_Home_Rising_Register(PulseHandle_t f)
{
  El_Home_Rising_Handler = f;
}
void El_Home_Rising_Unregister(void)
{
  El_Home_Rising_Handler = 0;
}
void El_Home_Falling_Register(PulseHandle_t f)
{
  El_Home_Falling_Handler = f;
}
void El_Home_Falling_Unregister(void)
{
  El_Home_Falling_Handler = 0;
}
void Az_Home_Rising_Register(PulseHandle_t f)
{
  Az_Home_Rising_Handler = f;
}
void Az_Home_Rising_Unregister(void)
{
  Az_Home_Rising_Handler = 0;
}
void Az_Home_Falling_Register(PulseHandle_t f)
{
  Az_Home_Falling_Handler = f;
}
void Az_Home_Falling_Unregister(void)
{
  Az_Home_Falling_Handler = 0;
}

/**
  * @brief  Interrupt EL & AZ handler
  * @note   ...
  * @param  none
  * @retval none
  */
void EL_EXTI_IRQn_Handler(void)
{
  if(EXTI_GetITStatus(EL_EXTI_LINE) != RESET)
  {
    EXTI_ClearITPendingBit(EL_EXTI_LINE);
    if (GPIO_ReadInputDataBit(EL_HOME_PULSE_PORT, EL_HOME_PULSE_PIN))
    {
      if (El_Home_Rising_Handler != 0)
        El_Home_Rising_Handler();
    }
    else
    {
      if (El_Home_Falling_Handler != 0)
        El_Home_Falling_Handler();
    }
  }
}

void AZ_EXTI_IRQn_Handler(void)
{
  if(EXTI_GetITStatus(AZ_EXTI_LINE) != RESET)
  {
    EXTI_ClearITPendingBit(AZ_EXTI_LINE);
    if (GPIO_ReadInputDataBit(AZ_HOME_PULSE_PORT, AZ_HOME_PULSE_PIN))
    {
      if (Az_Home_Rising_Handler != 0)
        Az_Home_Rising_Handler();
    }
    else
    {
      if (Az_Home_Falling_Handler != 0)
        Az_Home_Falling_Handler();
    }
  }
}

/* Limit Functions--------------------------*/
/**
  * @brief  Init El Limit pulse
  * @note   ...
  * @param  none
  * @retval none
  */
void Gimbal_El_Limit_Init(void)
{
  EXTI_InitTypeDef   EXTI_InitStructure;
  GPIO_InitTypeDef   GPIO_InitStructure;
  NVIC_InitTypeDef   NVIC_InitStructure;
  /* Config EL Limit Pulse */
  /* Enable GPIO clock */
  RCC_AHB1PeriphClockCmd(EL_LIMIT_PULSE_PORT_CLK, ENABLE);
  /* Enable SYSCFG clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
  /* Configure pin as input floating */
  GPIO_InitStructure.GPIO_Pin   = EL_LIMIT_PULSE_PIN;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
  GPIO_Init(EL_LIMIT_PULSE_PORT, &GPIO_InitStructure);
  /* Connect EXTI Line2 to EL Limit pin */
  SYSCFG_EXTILineConfig(EL_LIMIT_EXTI_PORT_SOURCE, EL_LIMIT_PULSE_PIN_SOURCE);
  /* Configure EXTI EL Limit - rising & falling */
  EXTI_InitStructure.EXTI_Line    = EL_LIMIT_EXTI_LINE;
  EXTI_InitStructure.EXTI_Mode    = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
  /* Enable and set EXTI EL Limit Interrupt to the lowest priority */
  NVIC_InitStructure.NVIC_IRQChannel = EL_LIMIT_EXTI_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

/**
  * @brief  Functions to register & unregister handle exti
  * @note   ...
  * @param  with register: PulseHandle_t f (handler function)
  * @retval none
  */
void El_Limit_Rising_Register(PulseHandle_t f)
{
  El_Limit_Rising_Handler = f;
}
void El_Limit_Rising_Unregister(void)
{
  El_Limit_Rising_Handler = 0;
}
void El_Limit_Falling_Register(PulseHandle_t f)
{
  El_Limit_Falling_Handler = f;
}
void El_Limit_Falling_Unregister(void)
{
  El_Limit_Falling_Handler = 0;
}

/**
  * @brief  Interrupt EL Limit handler
  * @note   ...
  * @param  none
  * @retval none
  */
void EL_LIMIT_EXTI_IRQn_Handler(void)
{
  if(EXTI_GetITStatus(EL_LIMIT_EXTI_LINE) != RESET)
  {
    EXTI_ClearITPendingBit(EL_LIMIT_EXTI_LINE);
    if (GPIO_ReadInputDataBit(EL_LIMIT_PULSE_PORT, EL_LIMIT_PULSE_PIN))
    {
      if (El_Limit_Rising_Handler != 0)
        El_Limit_Rising_Handler();
    }
    else
    {
      if (El_Limit_Falling_Handler != 0)
        El_Limit_Falling_Handler();
    }
  }
}
/*********************************END OF FILE**********************************/
