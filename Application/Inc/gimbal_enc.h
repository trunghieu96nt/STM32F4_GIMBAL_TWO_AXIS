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

/* Define --------------------------------------------------------------------*/
#define IS_TIMER_32BIT(x) ((x == TIM2) || (x == TIM5))
#define IS_APB2_TIMER(x) ((x == TIM1) || (x == TIM8) || (x == TIM9) \
   || (x == TIM10) || (x == TIM11))
   
#define ENC0_TIM_POS                    TIM1 
#define ENC0_TIM_CLK_POS                RCC_APB2Periph_TIM1
#define ENC0_PERIPH_PORT_POS            RCC_AHB1Periph_GPIOA
#define ENC0_PORT_POS                   GPIOA
#define ENC0_A_POS                      GPIO_Pin_8
#define ENC0_A_SOURCE_POS               GPIO_PinSource8
#define ENC0_B_POS                      GPIO_Pin_9
#define ENC0_B_SOURCE_POS               GPIO_PinSource9
#define ENC0_AF_POS                     GPIO_AF_TIM1
#define ENC0_PULSE_TO_ANGLE             (float)(0.00018)//(float)360/2000000
#define ENC0_PULSE_A_PER_REV            500000//2000
#define ENC0_TIM_VEL_IRQn               TIM1_IRQn
#define ENC0_TIM_VEL_IRQHandler         TIM1_IRQHandler

#define ENC1_TIM_POS                    TIM1 
#define ENC1_TIM_CLK_POS                RCC_APB2Periph_TIM1
#define ENC1_PERIPH_PORT_POS            RCC_AHB1Periph_GPIOA
#define ENC1_PORT_POS                   GPIOA
#define ENC1_A_POS                      GPIO_Pin_8
#define ENC1_A_SOURCE_POS               GPIO_PinSource8
#define ENC1_B_POS                      GPIO_Pin_9
#define ENC1_B_SOURCE_POS               GPIO_PinSource9
#define ENC1_AF_POS                     GPIO_AF_TIM1
#define ENC1_PULSE_TO_ANGLE             (float)(0.00018)//(float)360/2000000
#define ENC1_PULSE_A_PER_REV            500000//2000
#define ENC1_TIM_VEL_IRQn               TIM1_IRQn
#define ENC1_TIM_VEL_IRQHandler         TIM1_IRQHandler

#define ENC1_TIM_POS                    TIM3 
#define ENC1_TIM_CLK_POS                RCC_APB1Periph_TIM3
#define ENC1_PERIPH_PORT_POS            RCC_AHB1Periph_GPIOB
#define ENC1_PORT_POS                   GPIOB
#define ENC1_A_POS                      GPIO_Pin_4
#define ENC1_A_SOURCE_POS               GPIO_PinSource4
#define ENC1_B_POS                      GPIO_Pin_5
#define ENC1_B_SOURCE_POS               GPIO_PinSource5
#define ENC1_AF_POS                     GPIO_AF_TIM3
#define ENC1_TIM_POS_IRQn               TIM3_IRQn
#define ENC1_TIM_POS_IRQHandler         TIM3_IRQHandler

#define ENC1_PULSE_TO_ANGLE    (float)(0.00018)//(float)360/2000000
#define ENC1_PULSE_A_PER_REV   500000

/* Initialization and Configuration functions --------------------------------*/
void Gimbal_ENC_Init(void);

/* Functions -----------------------------------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif /*__GIMBAL_ENC_H */

/*********************************END OF FILE**********************************/
