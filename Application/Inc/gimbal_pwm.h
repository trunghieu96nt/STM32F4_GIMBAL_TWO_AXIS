/**
  ******************************************************************************
  * @file    gimbal_pwm.h
  * @author  Vu Trung Hieu
  * @version V1.0
  * @date    25-June-2017
  * @brief   This file contains all the functions prototypes for gimbal_pwm
  ******************************************************************************
  * @attention
  *
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GIMBAL_PWM_H
#define __GIMBAL_PWM_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"

/* Define --------------------------------------------------------------------*/
#define PWM0_USE_EN_PIN           0
#define PWM1_USE_EN_PIN           0

#define PWM0_TIM                  TIM12 // Channel 1
#define PWM0_TIM_CLK              RCC_APB1Periph_TIM12
#define PWM0_PULSE_PORT_CLK       RCC_AHB1Periph_GPIOB
#define PWM0_PULSE_PORT           GPIOB
#define PWM0_PULSE                GPIO_Pin_14
#define PWM0_PULSE_SOURCE         GPIO_PinSource14
#define PWM0_PULSE_AF             GPIO_AF_TIM12
#define PWM0_DIR_PORT_CLK         RCC_AHB1Periph_GPIOB
#define PWM0_DIR_PORT             GPIOB
#define PWM0_DIR                  GPIO_Pin_15
   
#if PWM0_USE_EN_PIN
#define PWM0_EN_PORT_CLK          RCC_AHB1Periph_GPIOD
#define PWM0_EN_PORT              GPIOD
#define PWM0_EN                   GPIO_Pin_10
#endif

#define PWM0_PRESCALER            12 // 84M/12=7M
#define PWM0_PERIOD               700 // 7M/700=10k
#define PWM0_DUTY                 0

#define PWM1_TIM                  TIM4 // Channel 1
#define PWM1_TIM_CLK              RCC_APB1Periph_TIM4
#define PWM1_PULSE_PORT_CLK       RCC_AHB1Periph_GPIOB
#define PWM1_PULSE_PORT           GPIOB
#define PWM1_PULSE                GPIO_Pin_6
#define PWM1_PULSE_SOURCE         GPIO_PinSource6
#define PWM1_PULSE_AF             GPIO_AF_TIM4
#define PWM1_DIR_PORT_CLK         RCC_AHB1Periph_GPIOB
#define PWM1_DIR_PORT             GPIOB
#define PWM1_DIR                  GPIO_Pin_12

#if PWM1_USE_EN_PIN
#define PWM1_EN_PORT_CLK          RCC_AHB1Periph_GPIOC
#define PWM1_EN_PORT              GPIOC
#define PWM1_EN                   GPIO_Pin_7
#endif

#define PWM1_PRESCALER            12 // 84M/12=7M
#define PWM1_PERIOD               700 // 7M/700=10k
#define PWM1_DUTY                 0

/* Initialization and Configuration functions --------------------------------*/
void Gimbal_PWM_Init(void);

/* Functions -----------------------------------------------------------------*/
void Gimbal_PWM0_Set_Freq(uint32_t freq);
void Gimbal_PWM0_Set_Duty(int16_t d);
void Gimbal_PWM1_Set_Freq(uint32_t freq);
void Gimbal_PWM1_Set_Duty(int16_t d);

void Gimbal_PWM_EL_Set_Duty(int16_t d);
void Gimbal_PWM_AZ_Set_Duty(int16_t d);

#ifdef __cplusplus
}
#endif

#endif /*__GIMBAL_PWM_H */

/*********************************END OF FILE**********************************/
