/**
  ******************************************************************************
  * @file    gimbal_pc.h
  * @author  Vu Trung Hieu
  * @version V1.0
  * @date    8-July-2017
  * @brief   This file contains all the functions prototypes for gimbal_pc.c
  ******************************************************************************
  * @attention
  *
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GIMBAL_PC_H
#define __GIMBAL_PC_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stdbool.h"
#include "stdint.h"

/* Define --------------------------------------------------------------------*/

// Gimbal receiver (receive from pc)
#define STM_RXBUFF_SIZE       1024
#define STM_FRAME_MAX_LEN     128
#define STM_START_FRAME       '{'
#define STM_END_FRAME         '}'

#define STM_USART             USART2
#define STM_USART_CLK         RCC_APB1Periph_USART2
#define STM_PORT              GPIOA
#define STM_PORT_CLK          RCC_AHB1Periph_GPIOA
#define STM_TX                GPIO_Pin_2
#define STM_TX_SOURCE         GPIO_PinSource2
#define STM_RX                GPIO_Pin_3
#define STM_RX_SOURCE         GPIO_PinSource3
#define STM_AF                GPIO_AF_USART2
#define STM_BAUDRATE          (uint32_t)115200

#define STM_AHB_PERIPH_DMA    RCC_AHB1Periph_DMA1
#define STM_DATA_REG          (uint32_t)STM_USART + 0x04
#define STM_TX_DMA_STREAM     DMA1_Stream6
#define STM_TX_DMA_CHANNEL    DMA_Channel_4
#define STM_TX_STREAM_IRQ     DMA1_Stream6_IRQn
#define STM_TX_DMA_FLAG       DMA_FLAG_TCIF6
#define STM_RX_DMA_STREAM     DMA1_Stream5
#define STM_RX_DMA_CHANNEL    DMA_Channel_4
#define STM_RX_STREAM_IRQ     DMA1_Stream5_IRQn
#define STM_RX_TC_IT_FLAG     DMA_IT_TCIF5
#define STM_RX_HT_IT_FLAG     DMA_IT_HTIF5
#define STM_RX_Interrupt      DMA1_Stream5_IRQHandler

// Gimbal sender (send to pc)
#define HMI_RXBUFF_SIZE       128

#define HMI_USART             USART3
#define HMI_USART_CLK         RCC_APB1Periph_USART3
#define HMI_PORT              GPIOB
#define HMI_PORT_CLK          RCC_AHB1Periph_GPIOB
#define HMI_TX                GPIO_Pin_10
#define HMI_TX_SOURCE         GPIO_PinSource10
#define HMI_RX                GPIO_Pin_11
#define HMI_RX_SOURCE         GPIO_PinSource11
#define HMI_AF                GPIO_AF_USART3
#define HMI_BAUDRATE          (uint32_t)115200 //921600 //115200

#define HMI_AHB_PERIPH_DMA    RCC_AHB1Periph_DMA1
#define HMI_DATA_REG          (uint32_t)HMI_USART + 0x04
#define HMI_TX_DMA_STREAM     DMA1_Stream3
#define HMI_TX_DMA_CHANNEL    DMA_Channel_4
#define HMI_TX_STREAM_IRQ     DMA1_Stream3_IRQn
#define HMI_TX_DMA_FLAG       DMA_FLAG_TCIF3
#define HMI_RX_DMA_STREAM     DMA1_Stream1
#define HMI_RX_DMA_CHANNEL    DMA_Channel_4
#define HMI_RX_STREAM_IRQ     DMA1_Stream1_IRQn
#define HMI_RX_TC_IT_FLAG     DMA_IT_TCIF1
#define HMI_RX_HT_IT_FLAG     DMA_IT_HTIF1
#define HMI_RX_Interrupt      DMA1_Stream1_IRQHandler

/* Initialization and Configuration functions --------------------------------*/
void Gimbal_PC_Init(void);

/* Functions -----------------------------------------------------------------*/
bool Gimbal_PC_Read(void);
bool Gimbal_PC_Read_Timeout(uint32_t ui32TimeOut_ms);
bool Gimbal_Sender_Send(uint8_t *pu8Message, uint32_t ui32MessageSize);

#ifdef __cplusplus
}
#endif

#endif /*__GIMBAL_PC_H */

/*********************************END OF FILE**********************************/
