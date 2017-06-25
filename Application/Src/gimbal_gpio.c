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
/* Private define ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
void Gimbal_Led_Init(void);
void Gimbal_SW_Init(void);
void Gimbal_DI_Init(void);

/* Functions ---------------------------------------------------------*/
/**
  * @brief  Initialize Gimbal GPIO
  * @note   ...
  * @param  none
  * @retval none
  */
void Gimbal_GPIO_Init(void)
{
  Gimbal_Led_Init();
  //Gimbal_SW_Init();
  //Gimbal_DI_Init;
}

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
  GPIO_InitStructure.GPIO_Pin = LED1_PIN | LED2_PIN | LED3_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
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
  GPIO_InitStructure.GPIO_Pin = SW1_PIN | SW2_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
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
  GPIO_InitStructure.GPIO_Pin = DI1_PIN | DI2_PIN | DI3_PIN | DI4_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(DI_GPIO, &GPIO_InitStructure);
}

/*********************************END OF FILE**********************************/
