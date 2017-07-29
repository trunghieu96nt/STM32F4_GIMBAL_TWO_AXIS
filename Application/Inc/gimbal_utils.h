/**
  ******************************************************************************
  * @file    gimbal_utils.h
  * @author  Vu Trung Hieu
  * @version V1.0
  * @date    8-July-2017
  * @brief   This file contains all the functions prototypes for gimbal_utils.c
  ******************************************************************************
  * @attention
  *
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GIMBAL_UTILS_H
#define __GIMBAL_UTILS_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "string.h"
#include "stdbool.h"
#include "stdint.h"

/* Define --------------------------------------------------------------------*/
#ifndef PI
#define PI (3.141592653589793)
#endif
/* Initialization and Configuration functions --------------------------------*/
/* Functions -----------------------------------------------------------------*/
void delay_us(uint32_t micros);
void *memrchr(const void *str, int c, size_t n);
void IntToStrN(int32_t number, uint8_t *str, uint32_t n);

#ifdef __cplusplus
}
#endif

#endif /*__GIMBAL_UTILS_H */

/*********************************END OF FILE**********************************/
