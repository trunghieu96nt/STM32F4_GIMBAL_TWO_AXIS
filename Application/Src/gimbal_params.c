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
#include "pid.h"

#include "gimbal_pc.h"
#include "stdio.h"
#include "string.h"

/* Extern variables ----------------------------------------------------------*/
extern STRU_PID_T stru_PID_AZ_Manual_Pos_Outer;

extern STRU_PID_T stru_PID_EL_Manual_Pos_Outer;
extern STRU_PID_T stru_PID_EL_Manual_Vel_Inner;
/* Public variables ----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static const uint8_t CODE_VESION_ARRAY[2] = {10, 1}; //Major.Minor
static const uint8_t au8ParamsIDLen[NUM_PARAMS_MAX] =
{
  /* Length. */     /* PARAMS_ID. */
  2,                //CODE_VERSION = 0,
  12,               //PARAMS_PID_AZ_MANUAL_POS_OUTER,
  12,               //PARAMS_PID_EL_MANUAL_POS_OUTER,
  12,               //PARAMS_PID_EL_MANUAL_VEL_INNER,
  
//  12,               //PARAMS_PID_AZ_POINTING_POS_OUTER,
//  12,               //PARAMS_PID_AZ_POINTING_VEL_INNER,
//  12,               //PARAMS_PID_EL_POINTING_POS_OUTER
//  12,               //PARAMS_PID_EL_POINTING_VEL_INNER
};
/* Private function prototypes -----------------------------------------------*/
uint32_t Gimbal_Params_Get_Pos(ENUM_PARAMS_T enumParams);

/* Functions ---------------------------------------------------------*/
/**
  * @brief  Get Pos of params in real eeprom
  * @note   ...
  * @param  ENUM_PARAMS_T enumParams
  * @retval ui32Pos
  */
uint32_t Gimbal_Params_Get_Pos(ENUM_PARAMS_T enumParams)
{
  uint32_t ui32Idx = 0, ui32Pos = 0;
  for(ui32Idx = 0; ui32Idx < enumParams; ui32Idx++)
  {
    ui32Pos += au8ParamsIDLen[ui32Idx];
  }
  return ui32Pos;
}

/**
  * @brief  save params
  * @note   ...
  * @param  ENUM_PARAMS_T enumParams
  * @param  const uint8_t *pu8Data: pointer of data need to save
  * @retval true if succeed and vice versa
  */
bool Gimbal_Params_Save(ENUM_PARAMS_T enumParams, const uint8_t *pu8Data)
{
  /* Test */
//  char cTest[20];
//  uint32_t ui32Idx = 0, ui32Test;
//  for(ui32Idx = 0; ui32Idx < au8ParamsIDLen[enumParams]; ui32Idx++)
//  {
//    ui32Test = Gimbal_Params_Get_Pos(enumParams);
//    printf(cTest,"%d\r\n", ui32Test);
//    Gimbal_Sender_Send((uint8_t *)cTest, strlen(cTest));
//  }
  return EEP_WriteBytes(pu8Data, Gimbal_Params_Get_Pos(enumParams), au8ParamsIDLen[enumParams]);
}

/**
  * @brief  load params
  * @note   ...
  * @param  ENUM_PARAMS_T enumParams
  * @param  uint8_t *pu8Data: pointer of data store data loaded from eeprom
  * @retval true if succeed and vice versa
  */
bool Gimbal_Params_Load(ENUM_PARAMS_T enumParams, uint8_t *pu8Data)
{
  return EEP_ReadBytes(pu8Data, Gimbal_Params_Get_Pos(enumParams), au8ParamsIDLen[enumParams]);
}

/**
  * @brief  save default params
  * @note   
  * @param  none
  * @retval none
  */
void Gimbal_Params_Save_Default(void)
{
  Gimbal_Params_Save(CODE_VERSION, CODE_VESION_ARRAY);
  Gimbal_Params_Save(PARAMS_PID_AZ_MANUAL_POS_OUTER, (uint8_t *)&stru_PID_AZ_Manual_Pos_Outer);
  Gimbal_Params_Save(PARAMS_PID_EL_MANUAL_POS_OUTER, (uint8_t *)&stru_PID_EL_Manual_Pos_Outer);
  Gimbal_Params_Save(PARAMS_PID_EL_MANUAL_VEL_INNER, (uint8_t *)&stru_PID_EL_Manual_Vel_Inner);
}

/**
  * @brief  load all params
  * @note   
  * @param  none
  * @retval none
  */
void Gimbal_Params_Load_All(void)
{
  uint8_t au8CodeVersion[2] = {0, 0};
  
  Gimbal_Params_Load(CODE_VERSION, au8CodeVersion);
  if((au8CodeVersion[0] != CODE_VESION_ARRAY[0]) || (au8CodeVersion[1] != CODE_VESION_ARRAY[1]))
  {
    Gimbal_Params_Save_Default();
  }
  
  Gimbal_Params_Load(PARAMS_PID_AZ_MANUAL_POS_OUTER, (uint8_t *)&stru_PID_AZ_Manual_Pos_Outer);
  Gimbal_Params_Load(PARAMS_PID_EL_MANUAL_POS_OUTER, (uint8_t *)&stru_PID_EL_Manual_Pos_Outer);
  Gimbal_Params_Load(PARAMS_PID_EL_MANUAL_VEL_INNER, (uint8_t *)&stru_PID_EL_Manual_Vel_Inner);
}

/*********************************END OF FILE**********************************/
