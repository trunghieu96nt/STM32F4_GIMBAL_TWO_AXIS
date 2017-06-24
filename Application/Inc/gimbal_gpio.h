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

/* Define --------------------------------------------------------------------*/
#define DI_PERIPH_GPIO        RCC_AHB1Periph_GPIOD
#define DI_GPIO               GPIOD
#define DI1_PIN               GPIO_Pin_3
#define DI2_PIN               GPIO_Pin_2
#define DI3_PIN               GPIO_Pin_1
#define DI4_PIN               GPIO_Pin_0

#define SW_PERIPH_GPIO        RCC_AHB1Periph_GPIOC
#define SW_GPIO               GPIOC
#define SW1_PIN               GPIO_Pin_6
#define SW2_PIN               GPIO_Pin_8

#define LED_PERIPH_GPIO       RCC_AHB1Periph_GPIOA
#define LED_GPIO              GPIOA
#define LED1_PIN              GPIO_Pin_10
#define LED2_PIN              GPIO_Pin_6
#define LED3_PIN              GPIO_Pin_4

/* Initialization and Configuration functions --------------------------------*/
void Gimbal_GPIO_Init(void);

/* Functions -----------------------------------------------------------------*/
void Gimbal_Led_Set(uint16_t LEDx_Pin);
void Gimbal_Led_Reset(uint16_t LEDx_Pin);
void Gimbal_Led_Toggle(uint16_t LEDx_Pin);

#ifdef __cplusplus
}
#endif

#endif /*__GIMBAL_GPIO_H */

/*********************************END OF FILE**********************************/
