/**
  ******************************************************************************
  * @file    eeprom_i2c.h
  * @author  Vu Trung Hieu
  * @version V1.0
  * @date    14-July-2017
  * @brief   This file contains all the functions prototypes for eeprom_i2c.c
  ******************************************************************************
  * @attention
  *
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __EEPROM_I2C_H
#define __EEPROM_I2C_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stdbool.h"
#include "stdint.h"

/* Define --------------------------------------------------------------------*/
#define EEP_I2C               I2C1
#define EEP_I2C_CLK           RCC_APB1Periph_I2C1
#define EEP_PORT              GPIOB
#define EEP_SCL               GPIO_Pin_8
#define EEP_SCL_SOURCE        GPIO_PinSource8
#define EEP_SDA               GPIO_Pin_9
#define EEP_SDA_SOURCE        GPIO_PinSource9
#define EEP_AF                GPIO_AF_I2C1
#define EEP_BAUDRATE          400000  // 400kHz
#define EEP_DATA_REG          (uint32_t)EEP_I2C + 0x10
#define I2C_TIMEOUT           100000

//Note: EEPROM AT24C04 address is 8 bits:|1010|A2|A1|Page|R/W|
#define EEP_ADD               0xA0
#define EEP_PAGE_0            0x00
#define EEP_PAGE_1            0x02
/* Initialization and Configuration functions --------------------------------*/
void EEP_Init(void);
/* Functions -----------------------------------------------------------------*/
bool EEP_WriteBytes(const uint8_t* pu8Buff, uint16_t ui16RegAdd, uint16_t ui16Len);
bool EEP_ReadBytes(uint8_t* pu8Buff, uint16_t ui16RegAdd, uint16_t ui16Len);

#ifdef __cplusplus
}
#endif

#endif /*__EEPROM_I2C_H */

/*********************************END OF FILE**********************************/
