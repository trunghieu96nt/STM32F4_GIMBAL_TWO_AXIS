/**
  ******************************************************************************
  * @file    gimbal_adis.c
  * @author  Vu Trung Hieu
  * @version V1.0
  * @date    2-July-2017
  * @brief   This file provides firmware functions to use ADIS sensor
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
#include "gimbal_adis.h"
#include "stm32f4xx.h"

/* Public variables ----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint8_t ui8RXBuff[IMU_RXBUFF_SIZE]= {0};
/* Private function prototypes -----------------------------------------------*/
/* Functions ---------------------------------------------------------*/
/**
  * @brief  Initialize Gimbal Adis
  * @note   ...
  * @param  none
  * @retval none
  */
void Gimbal_ADIS_Init(void)
{
  USART_InitTypeDef     USART_InitStructure;
  GPIO_InitTypeDef      GPIO_InitStructure;
  DMA_InitTypeDef       DMA_InitStructure;
  NVIC_InitTypeDef      NVIC_InitStructure;
  
  RCC_AHB1PeriphClockCmd(IMU_PORT_CLK, ENABLE);  
  GPIO_InitStructure.GPIO_Pin   = IMU_TX | IMU_RX;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
  GPIO_Init(IMU_PORT, &GPIO_InitStructure);
  
  GPIO_PinAFConfig(IMU_PORT, IMU_TX_SOURCE, IMU_AF);
  GPIO_PinAFConfig(IMU_PORT, IMU_RX_SOURCE, IMU_AF);

  RCC_APB1PeriphClockCmd(IMU_USART_CLK, ENABLE);
  
  USART_InitStructure.USART_BaudRate            = IMU_BAUDRATE;
  USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits            = USART_StopBits_1;
  USART_InitStructure.USART_Parity              = USART_Parity_No;
  USART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_Init(IMU_USART, &USART_InitStructure);
  USART_Cmd(IMU_USART, ENABLE); 
  USART_ClearFlag(IMU_USART, USART_FLAG_TC);

  RCC_AHB1PeriphClockCmd(IMU_AHB_PERIPH_DMA, ENABLE);
  DMA_DeInit(IMU_RX_DMA_STREAM);  
  DMA_InitStructure.DMA_Channel            = IMU_RX_DMA_CHANNEL;
  DMA_InitStructure.DMA_DIR                = DMA_DIR_PeripheralToMemory;
  DMA_InitStructure.DMA_Memory0BaseAddr    = (uint32_t)&ui8RXBuff[0];
  DMA_InitStructure.DMA_PeripheralBaseAddr = IMU_DATA_REG;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_Priority           = DMA_Priority_VeryHigh;
  DMA_InitStructure.DMA_FIFOMode           = DMA_FIFOMode_Disable; //direct mode
  DMA_InitStructure.DMA_FIFOThreshold      = DMA_FIFOThreshold_Full;
  DMA_InitStructure.DMA_MemoryInc          = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_Mode               = DMA_Mode_Circular; // avoid buffer overload error
  DMA_InitStructure.DMA_BufferSize         = IMU_RXBUFF_SIZE;
  DMA_InitStructure.DMA_MemoryBurst        = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_PeripheralBurst    = DMA_PeripheralBurst_Single;
  DMA_Init(IMU_RX_DMA_STREAM, &DMA_InitStructure);
  /* Enable  request */
  USART_DMACmd(IMU_USART, USART_DMAReq_Rx, ENABLE);  
  /* Enable DMA RX Channel */
  DMA_Cmd(IMU_RX_DMA_STREAM, ENABLE);

#if USE_IMU_RX_DMA_IT
  DMA_ITConfig(IMU_RX_DMA_STREAM, DMA_IT_HT, ENABLE);
  NVIC_InitStructure.NVIC_IRQChannel = IMU_RX_STREAM_IRQ;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  DMA_ITConfig(IMU_RX_DMA_STREAM, DMA_IT_TC, ENABLE);
  NVIC_InitStructure.NVIC_IRQChannel = IMU_RX_STREAM_IRQ;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
#endif
}

#if USE_IMU_RX_DMA_IT
void IMU_RX_Interrupt(void)
{
  if(DMA_GetITStatus(IMU_RX_DMA_STREAM, IMU_RX_TC_IT_FLAG) == SET)
  {
    //DMA_Cmd(IMU_RX_DMA_STREAM, DISABLE);
    DMA_ClearITPendingBit(IMU_RX_DMA_STREAM, IMU_RX_TC_IT_FLAG);
  }
  else if(DMA_GetITStatus(IMU_RX_DMA_STREAM, IMU_RX_HT_IT_FLAG) == SET)
  {
    DMA_ClearITPendingBit(IMU_RX_DMA_STREAM, IMU_RX_HT_IT_FLAG);
  }
}
#else
void gimbal_adis_read(void)
{
  static uint8_t *pui8_RXBuff;
  
  
}
#endif

/*********************************END OF FILE**********************************/
