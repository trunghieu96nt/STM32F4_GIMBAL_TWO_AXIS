/**
  ******************************************************************************
  * @file    gimbal_gpio.h
  * @author  Vu Trung Hieu
  * @version V1.0
  * @date    24-June-2017
  * @brief   This file contains all the functions prototypes for gimbal_gpio  
  ******************************************************************************
  * @attention
  *
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GIMBAL_GPIO_H
#define __GIMBAL_GPIO_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
/* typedef -------------------------------------------------------------------*/
typedef void (*PulseHandle_t)(void);

/* Define --------------------------------------------------------------------*/
/* DI Pin */
#define DI_PERIPH_GPIO                    RCC_AHB1Periph_GPIOD
#define DI_GPIO                           GPIOD
#define DI1_PIN                           GPIO_Pin_3
#define DI2_PIN                           GPIO_Pin_2
#define DI3_PIN                           GPIO_Pin_1
#define DI4_PIN                           GPIO_Pin_0

/* Home Pulse */
#define EL_HOME_PULSE_PORT                GPIOD
#define EL_HOME_PULSE_PORT_CLK            RCC_AHB1Periph_GPIOD
#define EL_HOME_PULSE_PIN                 GPIO_Pin_0
#define EL_HOME_PULSE_PIN_SOURCE          GPIO_PinSource0
#define EL_EXTI_LINE                      EXTI_Line0
#define EL_EXTI_PORT_SOURCE               EXTI_PortSourceGPIOD
#define EL_EXTI_IRQn                      EXTI0_IRQn
#define EL_EXTI_IRQn_Handler              EXTI0_IRQHandler

#define AZ_HOME_PULSE_PORT                GPIOD
#define AZ_HOME_PULSE_PORT_CLK            RCC_AHB1Periph_GPIOD
#define AZ_HOME_PULSE_PIN                 GPIO_Pin_1
#define AZ_HOME_PULSE_PIN_SOURCE          GPIO_PinSource1
#define AZ_EXTI_LINE                      EXTI_Line1
#define AZ_EXTI_PORT_SOURCE               EXTI_PortSourceGPIOD
#define AZ_EXTI_IRQn                      EXTI1_IRQn
#define AZ_EXTI_IRQn_Handler              EXTI1_IRQHandler

/* El  Limit */
#define EL_LIMIT_PULSE_PORT               GPIOD
#define EL_LIMIT_PULSE_PORT_CLK           RCC_AHB1Periph_GPIOD
#define EL_LIMIT_PULSE_PIN                GPIO_Pin_2
#define EL_LIMIT_PULSE_PIN_SOURCE         GPIO_PinSource2
#define EL_LIMIT_EXTI_LINE                EXTI_Line2
#define EL_LIMIT_EXTI_PORT_SOURCE         EXTI_PortSourceGPIOD
#define EL_LIMIT_EXTI_IRQn                EXTI2_IRQn
#define EL_LIMIT_EXTI_IRQn_Handler        EXTI2_IRQHandler

/* SW Pin */
#define SW_PERIPH_GPIO                    RCC_AHB1Periph_GPIOC
#define SW_GPIO                           GPIOC
#define SW1_PIN                           GPIO_Pin_6
#define SW2_PIN                           GPIO_Pin_8

/* LED Pin */
#define LED_PERIPH_GPIO                   RCC_AHB1Periph_GPIOA
#define LED_GPIO                          GPIOA
#define LED1_PIN                          GPIO_Pin_12
#define LED2_PIN                          GPIO_Pin_11
#define LED3_PIN                          GPIO_Pin_10

/* Initialization and Configuration functions --------------------------------*/
void Gimbal_GPIO_Init(void);

/* Functions -----------------------------------------------------------------*/
/* Led --------------------------------------------*/
void Gimbal_Led_Set(uint16_t LEDx_Pin);
void Gimbal_Led_Reset(uint16_t LEDx_Pin);
void Gimbal_Led_Toggle(uint16_t LEDx_Pin);

/* Register & Unregister interrupt handler---------*/
void El_Home_Rising_Register(PulseHandle_t f);
void El_Home_Rising_Unregister(void);
void El_Home_Falling_Register(PulseHandle_t f);
void El_Home_Falling_Unregister(void);
void Az_Home_Rising_Register(PulseHandle_t f);
void Az_Home_Rising_Unregister(void);
void Az_Home_Falling_Register(PulseHandle_t f);
void Az_Home_Falling_Unregister(void);
void El_Limit_Rising_Register(PulseHandle_t f);
void El_Limit_Rising_Unregister(void);
void El_Limit_Falling_Register(PulseHandle_t f);
void El_Limit_Falling_Unregister(void);

#ifdef __cplusplus
}
#endif

#endif /*__GIMBAL_GPIO_H */

/*********************************END OF FILE**********************************/
