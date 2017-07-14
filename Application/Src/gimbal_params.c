/**
  ******************************************************************************
  * @file    gimbal_params.c
  * @author  Vu Trung Hieu
  * @version V1.0
  * @date    14-July-2017
  * @brief   This file provides functions to communicate with eeprom
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
#include "gimbal_params.h"
//#include "stm32f4xx.h"
#include "eeprom_i2c.h"

/* Public variables ----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Functions ---------------------------------------------------------*/
/**
  * @brief  ...
  * @note   ...
  * @param  none
  * @retval none
  */
uint8_t CODE_VESION_ARRAY[4] = {1, 0, 0, 1};

bool Gimbal_Params_Save_Word(ENUM_PARAMS_T enumParams, uint8_t *pui8Data)
{
  return EEP_WriteBytes(pui8Data, (uint32_t)(enumParams << 2), 4);
}

bool Gimbal_Params_Save_Float(ENUM_PARAMS_T enumParams, float fValue)
{
  UNION_FLOAT_DATA_T unionFloatData;
  unionFloatData.fValue = fValue;
  return Gimbal_Params_Save_Word(enumParams, unionFloatData.byte);
}

bool Gimbal_Params_Load_Float(ENUM_PARAMS_T enumParams, float *pfValue)
{
  UNION_FLOAT_DATA_T unionFloatData;
  if(EEP_ReadBytes(unionFloatData.byte, (uint32_t)(enumParams << 2), 4))
  {
    *pfValue = unionFloatData.fValue;
    return true;
  }
  else return false;
}

bool Gimbal_Params_Load_Word(ENUM_PARAMS_T enumParams, uint8_t *pui8Data)
{
  if(EEP_ReadBytes(pui8Data, (uint32_t)(enumParams << 2), 4))
    return true;
  else return false;
}

void Gimbal_Params_Save_All(void)
{
  
}

void Gimbal_Params_Load_All(void)
{
  
}

/*********************************END OF FILE**********************************/
