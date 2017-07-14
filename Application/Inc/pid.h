/**
  ******************************************************************************
  * @file    pid.h
  * @author  Vu Trung Hieu
  * @version V1.0
  * @date    12-July-2017
  * @brief   This file contains all the functions prototypes for pid.c
  ******************************************************************************
  * @attention
  *
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __PID_H
#define __PID_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
/* Define --------------------------------------------------------------------*/
/* Struct --------------------------------------------------------------------*/
typedef struct
{
  float Kp;
  float Ki;
  float Kd;
  float Ts;
  
  float SetPoint;
  
} STRU_PID_T;
/* Initialization and Configuration functions --------------------------------*/
/* Functions -----------------------------------------------------------------*/


#ifdef __cplusplus
}
#endif

#endif /*__PID_H */

/*********************************END OF FILE**********************************/
