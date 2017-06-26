/**
  ******************************************************************************
  * @file    gimbal_enc.h
  * @author  Vu Trung Hieu
  * @version V1.0
  * @date    24-June-2017
  * @brief   This file contains all the functions prototypes for gimbal_enc
  ******************************************************************************
  * @attention
  *
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GIMBAL_ENC_H
#define __GIMBAL_ENC_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
   
   
/* Define --------------------------------------------------------------------*/
#define ENC0_TIM_POS                    TIM1 
#define ENC0_TIM_CLK_POS                RCC_APB2Periph_TIM1
#define ENC0_PERIPH_PORT_POS            RCC_AHB1Periph_GPIOA
#define ENC0_PORT_POS                   GPIOA
#define ENC0_A_POS                      GPIO_Pin_8
#define ENC0_A_SOURCE_POS               GPIO_PinSource8
#define ENC0_B_POS                      GPIO_Pin_9
#define ENC0_B_SOURCE_POS               GPIO_PinSource9
#define ENC0_AF_POS                     GPIO_AF_TIM1
   
#define ENC1_TIM_POS                    TIM3 
#define ENC1_TIM_CLK_POS                RCC_APB1Periph_TIM3
#define ENC1_PERIPH_PORT_POS            RCC_AHB1Periph_GPIOB
#define ENC1_PORT_POS                   GPIOB
#define ENC1_A_POS                      GPIO_Pin_4
#define ENC1_A_SOURCE_POS               GPIO_PinSource4
#define ENC1_B_POS                      GPIO_Pin_5
#define ENC1_B_SOURCE_POS               GPIO_PinSource5
#define ENC1_AF_POS                     GPIO_AF_TIM3

/* Initialization and Configuration functions --------------------------------*/
void Gimbal_ENC_Init(void);

/* Functions -----------------------------------------------------------------*/
int32_t Gimbal_ENC0_Get_Pos(void);
void Gimbal_ENC0_Reset(void);
  
int32_t Gimbal_ENC1_Get_Pos(void);
void Gimbal_ENC1_Reset(void);

#ifdef __cplusplus
}
#endif

#endif /*__GIMBAL_ENC_H */

/*********************************END OF FILE**********************************/
