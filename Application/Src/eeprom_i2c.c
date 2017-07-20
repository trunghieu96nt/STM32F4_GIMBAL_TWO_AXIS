/**
  ******************************************************************************
  * @file    eeprom_i2c.c
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
#include "eeprom_i2c.h"
#include "stm32f4xx.h"
#include "gimbal_utils.h"

/* Public variables ----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
bool I2C_ReadBytes(I2C_TypeDef* I2Cx, uint8_t *pu8Buff, uint8_t ui8SlaveAdd, uint8_t ui8RegAdd, uint8_t ui8Len);
bool I2C_WriteBytes(I2C_TypeDef * I2Cx, const uint8_t *pu8Buff, uint8_t ui8SlaveAdd, uint8_t ui8RegAdd, uint8_t ui8Len);

/* Functions ---------------------------------------------------------*/
/**
  * @brief  I2C_ReadBytes
  * @note   ...
  * @param  pu8Buff: pointer to array that will store bytes from slave
  * @param  ui8SlaveAdd: slave's address
  * @param  ui8RegAdd: start address of the register of the slave
  * @param  ui8Len: length of the array
  * @retval true if succeed and vice versa
  */
bool I2C_ReadBytes(I2C_TypeDef* I2Cx, uint8_t *pu8Buff, uint8_t ui8SlaveAdd, uint8_t ui8RegAdd, uint8_t ui8Len)
{
  uint32_t ui32Time;

  if (ui8Len == 0)   return false;

  /* While the bus is busy */
  ui32Time = I2C_TIMEOUT;
  while(I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY))
  {
    if((ui32Time--) == 0) return false;
  }

  /* Send START condition */
  I2C_GenerateSTART(I2Cx, ENABLE);

  /* Test on EV5 and clear it (cleared by reading SR1 then writing to DR) */
  ui32Time = I2C_TIMEOUT;
  while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT))
  {
    if((ui32Time--) == 0) return false;
  }

  /* Send Slave address for write */
  I2C_Send7bitAddress(I2Cx, ui8SlaveAdd, I2C_Direction_Transmitter);
      
  /* Test on EV6 and clear it */
  ui32Time = I2C_TIMEOUT;
  while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
  {
    if((ui32Time--) == 0) return false;
  }

  /* Send the Register address to read from: Only one byte address */
  I2C_SendData(I2Cx, ui8RegAdd);  

  /* Test on EV8 and clear it */
  ui32Time = I2C_TIMEOUT;
  while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) 
  {
    if((ui32Time--) == 0) return false;
  }

  /* Send START condition a second time to RESTART bus */
  I2C_GenerateSTART(I2Cx, ENABLE);

  /* Test on EV5 and clear it (cleared by reading SR1 then writing to DR) */
  ui32Time = I2C_TIMEOUT;
  while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT))
  {
    if((ui32Time--) == 0) return false;
  } 

  /* Send Slave address for read */
  I2C_Send7bitAddress(I2Cx, ui8SlaveAdd, I2C_Direction_Receiver);  

  /* SHOULD DISABLE INTERRUPT HERE, IF NOT IT MAY BE CORRUPT I2C TRANSACTION */
  __disable_irq();

  /* Test on EV6 and clear it */
  ui32Time = I2C_TIMEOUT;
  while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED))
  {
      if ((ui32Time--) == 0) return false;
  }

  while(ui8Len)
  {
    if(ui8Len == 1)
    {
        /* This configuration should be in the last second transfer byte */
        /* Disable Acknowledgement */
        I2C_AcknowledgeConfig(I2Cx, DISABLE);
        
        /* Send STOP Condition */
        I2C_GenerateSTOP(I2Cx, ENABLE);
    }

    /* Test on EV7 and clear it */
    ui32Time = I2C_TIMEOUT;
    while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED))
    {
      if ((ui32Time--) == 0) return false;
    }

    /* Read the byte received from the Sensor */
    /* Point to the next location where the byte read will be saved */
    *pu8Buff = I2C_ReceiveData(I2Cx);
    pu8Buff++;
    
    /* Decrease the read bytes counter */
    ui8Len--;
  }

  /* Wait to make sure that STOP control bit has been cleared */
  ui32Time = I2C_TIMEOUT;
  while(I2Cx->CR1 & I2C_CR1_STOP)
  {
      if((ui32Time--) == 0) return false;
  }

  /* Re-Enable Acknowledgement to be ready for another reception */
  I2C_AcknowledgeConfig(I2Cx, ENABLE);  
    
  /* ENABLE ALL INTERRUPT */
  __enable_irq();     
  return true; 
}

/**
  * @brief  I2C_WriteBytes
  * @note   ...
  * @param  pu8Buff: pointer to array that will store bytes from slave
  * @param  ui8SlaveAdd: slave's address
  * @param  ui8RegAdd: start address of the register of the slave
  * @param  ui8len: length of the array
  * @retval true if succeed and vice versa
  */
bool I2C_WriteBytes(I2C_TypeDef * I2Cx, const uint8_t *pu8Buff, uint8_t ui8SlaveAdd, uint8_t ui8RegAdd, uint8_t ui8Len)
{
  uint32_t ui32Time;
  I2C_AcknowledgeConfig(I2Cx, ENABLE); 
  /* While the bus is busy */
  ui32Time = I2C_TIMEOUT;
  while(I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY))
  {
    if((ui32Time--) == 0) return false;
  }

  /* Send START condition */
  I2C_GenerateSTART(I2Cx, ENABLE);

  /* Test on EV5 and clear it (cleared by reading SR1 then writing to DR) */
  ui32Time = I2C_TIMEOUT;
  while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT))
  {
    if((ui32Time--) == 0) return false;
  }

  /* Send Slave address for write */
  I2C_Send7bitAddress(I2Cx, ui8SlaveAdd, I2C_Direction_Transmitter);

  /* Test on EV6 and clear it */
  ui32Time = I2C_TIMEOUT;
  while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
  {
    if((ui32Time--) == 0) return false;
  }

  /* Send the Register address to read from: Only one byte address */
  I2C_SendData(I2Cx, ui8RegAdd);  

  /* Test on EV8 and clear it */
  ui32Time = I2C_TIMEOUT;
  while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
  {
      if (ui32Time-- == 0) return false;
  }
    
  /* SHOULD DISABLE INTERRUPT HERE, IF NOT IT MAY BE CORRUPT I2C TRANSACTION */
  __disable_irq();
  while(ui8Len)
  {
    /* Send the data & increase the pointer of write buffer */
    I2C_SendData(I2Cx, *pu8Buff); 
    pu8Buff++;
    ui8Len--;  
    /* Test on EV8_2 to ensure data is transmitted, can used EV_8 for faster transmission*/
    ui32Time = I2C_TIMEOUT;
    while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
    {
      if ((ui32Time--) == 0) return false;
    }
  }

  /* Send STOP Condition */
  I2C_GenerateSTOP(I2Cx, ENABLE);
  /* ENABLE ALL INTERRUPT */
  __enable_irq();
  return true;
}

/**
  * @brief  EEP_Init
  * @note   ...
  * @param  none
  * @retval none
  */
void EEP_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  I2C_InitTypeDef  I2C_InitStructure;

  /* GPIO configuration */
  GPIO_InitStructure.GPIO_Pin   = EEP_SCL | EEP_SDA;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  GPIO_PinAFConfig(GPIOB, EEP_SDA_SOURCE, EEP_AF);
  GPIO_PinAFConfig(GPIOB, EEP_SCL_SOURCE, EEP_AF);

  /* I2C configuration */
  RCC_APB1PeriphClockCmd(EEP_I2C_CLK, ENABLE);
  I2C_DeInit(EEP_I2C);
  I2C_InitStructure.I2C_ClockSpeed          = EEP_BAUDRATE;
  I2C_InitStructure.I2C_Mode                = I2C_Mode_I2C;
  I2C_InitStructure.I2C_DutyCycle           = I2C_DutyCycle_2;
  I2C_InitStructure.I2C_OwnAddress1         = 0;
  I2C_InitStructure.I2C_Ack                 = I2C_Ack_Enable;
  I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
  I2C_Init(EEP_I2C, &I2C_InitStructure);
  I2C_Cmd(EEP_I2C, ENABLE);
}

/**
  * @brief  EEP_ReadBytes
  * @note   ...
  * @param  pu8Buff: pointer to array that will store bytes from EEP
  * @param  ui16RegAdd: start regiter address from 0 to 511 
  *         (EEPROM 24C04: 512 bytes/address)
  *         And it is divided into 2 pages (256 bytes/page)
  * @param  ui16Len: length of data to read
  * @retval true if succeed and vice versa
  */
bool EEP_ReadBytes(uint8_t* pu8Buff, uint16_t ui16RegAdd, uint16_t ui16Len)
{
  uint16_t ui16RegAddBuff;
  uint8_t ui8EEPAddBuff;
  uint16_t ui16idx;
    
  for(ui16idx = 0; ui16idx < ui16Len; ui16idx++)
  {
    ui8EEPAddBuff = EEP_ADD;
    ui16RegAddBuff = ui16RegAdd + ui16idx;
    
    if(ui16RegAddBuff > 511) return false; // Out of range
    else if(ui16RegAddBuff > 255)
    {
      /* Page 1: insert bit page 1 in eeprom address */
      ui16RegAddBuff -= 256;
      ui8EEPAddBuff |= EEP_PAGE_1;
    }
    /*
    else if(ui16RegAddBuff < 255) Page 0: do nothing
    */
      
    /* Random read 1 byte */
    if(I2C_ReadBytes(EEP_I2C, pu8Buff, ui8EEPAddBuff, ui16RegAddBuff, 1) == false) 
    {
      return false;
    }
    pu8Buff++;
  }
  return true;
}

/**
  * @brief  EEP_WriteBytes
  * @note   ...
  * @param  pu8Buff: pointer to write array data
  * @param  ui16RegAdd: start regiter address from 0 to 511 
  *         (EEPROM 24C04: 512 bytes/address)
  *         And it is divided into 2 pages (256 bytes/page)
  * @param  ui16Len: length of data to write
  * @retval true if succeed and vice versa
  */
bool EEP_WriteBytes(const uint8_t* pu8Buff, uint16_t ui16RegAdd, uint16_t ui16Len)
{
  uint16_t ui16RegAddBuff;
  uint8_t ui8EEPAddBuff;
  uint16_t ui16idx;
    
  for(ui16idx = 0; ui16idx < ui16Len; ui16idx++)
  {
    ui8EEPAddBuff = EEP_ADD;
    ui16RegAddBuff = ui16RegAdd + ui16idx;

    if(ui16RegAddBuff > 511) return false; // Out of range
    else if(ui16RegAddBuff > 255)
    {
      /* Page 1: insert bit page 1 in eeprom address */
      ui16RegAddBuff -= 256;
      ui8EEPAddBuff |= EEP_PAGE_1;
    }
    /*
    else if(reg_add < 255) page 0: do nothing
    */

    // Random write 1 byte into eeprom
    if (I2C_WriteBytes(EEP_I2C, pu8Buff, ui8EEPAddBuff, ui16RegAddBuff, 1) == false)
    {
      return false;
    }
    pu8Buff++;
    delay_us(5000); //Important: Maximum write cycle time = 5ms
  }
  return true;
}
/*********************************END OF FILE**********************************/
