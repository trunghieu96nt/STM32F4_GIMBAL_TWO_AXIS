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
//#include "stm32f4xx.h"

/* Public variables ----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Functions ---------------------------------------------------------*/
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


/*********************************END OF FILE**********************************/
