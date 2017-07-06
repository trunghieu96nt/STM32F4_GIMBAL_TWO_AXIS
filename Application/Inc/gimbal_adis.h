/**
  ******************************************************************************
  * @file    gimbal_adis.h
  * @author  Vu Trung Hieu
  * @version V1.0
  * @date    2-July-2017
  * @brief   This file contains all the functions prototypes for gimbal_adis.c
  ******************************************************************************
  * @attention
  *
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GIMBAL_ADIS_H
#define __GIMBAL_ADIS_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
/* Define --------------------------------------------------------------------*/
#define USE_IMU_RX_DMA_IT     1

#if USE_IMU_RX_DMA_IT
#define IMU_TXBUFF_SIZE       256
#define IMU_RXBUFF_SIZE       256
#else
#define IMU_TXBUFF_SIZE       1024
#define IMU_RXBUFF_SIZE       1024
#endif


#define IMU_USART             UART4
#define IMU_USART_CLK         RCC_APB1Periph_UART4
#define IMU_PORT              GPIOA
#define IMU_PORT_CLK          RCC_AHB1Periph_GPIOA
#define IMU_TX                GPIO_Pin_0
#define IMU_TX_SOURCE         GPIO_PinSource0
#define IMU_RX                GPIO_Pin_1
#define IMU_RX_SOURCE         GPIO_PinSource1
#define IMU_AF                GPIO_AF_UART4
#define IMU_BAUDRATE          (uint32_t)9600 //921600

#define IMU_AHB_PERIPH_DMA    RCC_AHB1Periph_DMA1
#define IMU_DATA_REG          (uint32_t)IMU_USART + 0x04
#define IMU_TX_DMA_STREAM     DMA1_Stream4
#define IMU_TX_DMA_CHANNEL    DMA_Channel_4
#define IMU_TX_STREAM_IRQ     DMA1_Stream4_IRQn
#define IMU_TX_DMA_FLAG       DMA_FLAG_TCIF4
#define IMU_RX_DMA_STREAM     DMA1_Stream2
#define IMU_RX_DMA_CHANNEL    DMA_Channel_4
#define IMU_RX_STREAM_IRQ     DMA1_Stream2_IRQn
#define IMU_RX_TC_IT_FLAG     DMA_IT_TCIF2
#define IMU_RX_HT_IT_FLAG     DMA_IT_HTIF2
#define IMU_RX_Interrupt      DMA1_Stream2_IRQHandler
/* Initialization and Configuration functions --------------------------------*/
void Gimbal_ADIS_Init(void);
/* Functions -----------------------------------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif /*__GIMBAL_ADIS_H */

/*********************************END OF FILE**********************************/
