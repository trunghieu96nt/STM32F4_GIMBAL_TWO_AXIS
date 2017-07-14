/**
  ******************************************************************************
  * @file    gimbal_utils.c
  * @author  Vu Trung Hieu
  * @version V1.0
  * @date    8-July-2017
  * @brief   This file provides functions to use for many purpose
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
#include "gimbal_utils.h"
#include "stdlib.h"
#include "stdio.h"
#include "stm32f4xx.h"

/* Public variables ----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Functions ---------------------------------------------------------*/
/**
  * @brief  delay_us
  * @note   ...
  * @param  uint32_t micros
  * @retval none
  */
void delay_us(uint32_t micros)
{
  RCC_ClocksTypeDef RCC_Clocks;
  /* Get system clocks */
  RCC_GetClocksFreq(&RCC_Clocks);
  micros = micros * (RCC_Clocks.HCLK_Frequency / 4000000) - 10;
  /* 4 cycles for one loop */
  while (micros--);
}

/**
  * @brief  memrchr
  * @note   search from *str backward n
  * @param  pointer *str, desired int (or char) c, length n
  * @retval none
  */
void *memrchr(const void *str, int c, size_t n)
{
  if(n != 0)
  {
    const unsigned char *p = str;
    do {
      if (*p-- == (unsigned char) c)
      {
        return (void *) (p + 1);
      }
    } while (--n);
  }
  return NULL;
}

/**
  * @brief  IntToStrN
  * @note   convert int to str with n char, the first char is sign
            If n < lenOfStr(number), final len will be lenOfStr(number) + 1
            Example IntToStrN(10, str, 5)   -> str: 0010
                    IntToStrN(-10, str, 5)  -> str:-0010
                    IntToStrN(-0, str, 6)   -> str: 00000
                    IntToStrN(3000, str, 2) -> str: 3000
                    IntToStrN(-200, str, 3) -> str:-200
  * @param  int32_t number, uint8_t *str, uint32_t n
  * @retval none
  */
void IntToStrN(int32_t number, uint8_t *str, uint32_t n)
{
  uint8_t ui8Mask[10];
  if(number < 0)
  {
    str[0] = '-';
    number = -number;
  }
  else
  {
    str[0] = ' ';
  }
  sprintf((char *)ui8Mask, "%%0%dd", n - 1);
  sprintf((char *)(str + 1), (char *)ui8Mask, number);
}

/*********************************END OF FILE**********************************/
