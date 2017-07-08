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
#include "gimbal_utils.h"
#include "string.h"
#include "stdlib.h"
#include "system_timetick.h"

/* Public variables ----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static uint8_t ui8IMURxBuff[IMU_RXBUFF_SIZE]= {0};
IMUData_t imuData = {false}; //initial isAvailable value.
/* Private function prototypes -----------------------------------------------*/
bool Gimbal_ADIS_Parse(uint8_t *pui8IMUFrame);

/* Functions ---------------------------------------------------------*/
/**
  * @brief  Initialize Gimbal Adis
  * @note   DMA UART RX, use IT by define USE_IMU_RX_DMA_IT
  * @param  none
  * @retval none
  */
void Gimbal_ADIS_Init(void)
{
  USART_InitTypeDef     USART_InitStructure;
  GPIO_InitTypeDef      GPIO_InitStructure;
  DMA_InitTypeDef       DMA_InitStructure;
#if USE_IMU_RX_DMA_IT
  NVIC_InitTypeDef      NVIC_InitStructure;
#endif
  
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
  DMA_InitStructure.DMA_Memory0BaseAddr    = (uint32_t)&ui8IMURxBuff[0];
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
  DMA_ITConfig(IMU_RX_DMA_STREAM, DMA_IT_TC, ENABLE);
  NVIC_InitStructure.NVIC_IRQChannel = IMU_RX_STREAM_IRQ;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
#endif
}

#if USE_IMU_RX_DMA_IT
/* Functions ---------------------------------------------------------*/
/**
  * @brief  IMU_RX_Interrupt Handler
  * @note   Handle with exact length
  * @param  none
  * @retval none
  */
void IMU_RX_Interrupt(void)
{
  uint8_t ui8rawIMUData[IMU_RXBUFF_SIZE];
  
  if(DMA_GetITStatus(IMU_RX_DMA_STREAM, IMU_RX_TC_IT_FLAG) == SET)
  {
    //DMA_Cmd(IMU_RX_DMA_STREAM, DISABLE);
    DMA_ClearITPendingBit(IMU_RX_DMA_STREAM, IMU_RX_TC_IT_FLAG);
    memcpy(ui8rawIMUData, ui8IMURxBuff, IMU_RXBUFF_SIZE);
    Gimbal_ADIS_Parse(ui8rawIMUData);
  }
}
#endif

/* Functions ---------------------------------------------------------*/
/**
  * @brief  Gimbal_ADIS_Parse
  * @note   parse raw IMU frame
  * @param  pointer *pui8IMUFrame
  * @retval true if parse correctly and vice versa
  */
bool Gimbal_ADIS_Parse(uint8_t *pui8IMUFrame)
{
  uint32_t ui32Idx = 0;
  uint8_t *pui8End = NULL, *pui8Start = pui8IMUFrame + 2;
  
  if(strlen((char *)pui8IMUFrame) != 87) return false;
  //get euler
  for(ui32Idx = 0; ui32Idx < 3; ui32Idx++)
  {
    pui8End = memchr(pui8Start, ' ', IMU_ELEMENT_MAX_LEN);
    if(pui8End == NULL) return false;
    *pui8End = 0;
    *(&imuData.euler_x + ui32Idx) = (double)atoi((char *)pui8Start - 1) * IMU_SCALE_EULER_UNIT;
    pui8Start = pui8End + 2;
  }
  //get gyro
  for(ui32Idx = 0; ui32Idx < 3; ui32Idx++)
  {
    pui8End = memchr(pui8Start, ' ', IMU_ELEMENT_MAX_LEN);
    if(pui8End == NULL) return false;
    *pui8End = 0;
    *(&imuData.gyro_x + ui32Idx) = (double)atoi((char *)pui8Start - 1) * IMU_SCALE_GYRO_UNIT;
    pui8Start = pui8End + 2;
  }
  //get mag
  for(ui32Idx = 0; ui32Idx < 3; ui32Idx++)
  {
    pui8End = memchr(pui8Start, ' ', IMU_ELEMENT_MAX_LEN);
    if(pui8End == NULL) return false;
    *pui8End = 0;
    *(&imuData.mag_x + ui32Idx) = (double)atoi((char *)pui8Start - 1) * IMU_SCALE_MAG_UNIT;
    pui8Start = pui8End + 2;
  }
  //get acc
  for(ui32Idx = 0; ui32Idx < 3; ui32Idx++)
  {
    pui8End = memchr(pui8Start, ' ', IMU_ELEMENT_MAX_LEN);
    if(pui8End == NULL) return false;
    *pui8End = 0;
    *(&imuData.acc_x + ui32Idx) = (double)atoi((char *)pui8Start - 1) * IMU_SCALE_ACC_UNIT;
    pui8Start = pui8End + 2;
  }
  //get gyro fog
  pui8End = memchr(pui8Start, ' ', IMU_ELEMENT_MAX_LEN);
  if(pui8End == NULL) return false;
  *pui8End = 0;
  imuData.gyro_fog = (double)atoi((char *)pui8Start - 1) * IMU_SCALE_FOG_UNIT;
  return true;
}

/* Functions ---------------------------------------------------------*/
/**
  * @brief  Gimbal_ADIS_Read
  * @note   get raw IMU frame and call Gimbal_ADIS_Parse()
  * @param  none
  * @retval true if get correctly and vice versa
  */
bool Gimbal_ADIS_Read(void)
{
  static uint8_t *pui8IMURxBuffCur = &ui8IMURxBuff[IMU_RXBUFF_SIZE - 1];
  static uint8_t *pui8IMURxBuffPre = &ui8IMURxBuff[IMU_RXBUFF_SIZE - 1];
  uint8_t *pui8EndChr = NULL, *pui8StartChr = NULL;
  uint8_t ui8IMUFrame[IMU_FRAME_MAX_LEN];
  
  if(IMU_RX_DMA_STREAM->NDTR == IMU_RXBUFF_SIZE)
    pui8IMURxBuffCur = &ui8IMURxBuff[IMU_RXBUFF_SIZE - 1];
  else
    pui8IMURxBuffCur = &ui8IMURxBuff[IMU_RXBUFF_SIZE - IMU_RX_DMA_STREAM->NDTR - 1];
  
  if(pui8IMURxBuffCur > pui8IMURxBuffPre)
  {
    //search IMU_END_FRAME backward from pui8IMURxBuffCur to pui8IMURxBuffPre
    pui8EndChr = memrchr(pui8IMURxBuffCur, IMU_END_FRAME,pui8IMURxBuffCur - pui8IMURxBuffPre + 1);
    if(pui8EndChr == NULL) return false;
    //search IMU_START_FRAME backward from pui8EndChr to pui8IMURxBuffPre
    pui8StartChr = memrchr(pui8EndChr, IMU_START_FRAME,pui8EndChr - pui8IMURxBuffPre + 1);
    if(pui8StartChr == NULL) return false;
    memcpy(ui8IMUFrame, pui8StartChr, pui8EndChr - pui8StartChr + 1);
    ui8IMUFrame[pui8EndChr - pui8StartChr + 1] = 0;
  }
  else if(pui8IMURxBuffCur < pui8IMURxBuffPre)
  {
    //search IMU_END_FRAME backward from pui8IMURxBuffCur to ui8IMURxBuff
    pui8EndChr = memrchr(pui8IMURxBuffCur, IMU_END_FRAME,pui8IMURxBuffCur - ui8IMURxBuff + 1);
    if(pui8EndChr != NULL)
    {
      //search IMU_START_FRAME backward from pui8EndChr to ui8IMURxBuff
      pui8StartChr = memrchr(pui8EndChr, IMU_START_FRAME,pui8EndChr - ui8IMURxBuff + 1);
      if(pui8StartChr != NULL)
      {
        memcpy(ui8IMUFrame, pui8StartChr, pui8EndChr - pui8StartChr + 1);
        ui8IMUFrame[pui8EndChr - pui8StartChr + 1] = 0;
      }
      else
      {
        //search IMU_START_FRAME backward from &ui8IMURxBuff[IMU_RXBUFF_SIZE - 1] to pui8IMURxBuffPre
        pui8StartChr = memrchr(&ui8IMURxBuff[IMU_RXBUFF_SIZE - 1], IMU_START_FRAME,&ui8IMURxBuff[IMU_RXBUFF_SIZE - 1] - pui8IMURxBuffPre + 1);
        if(pui8StartChr == NULL) return false;
        memcpy(ui8IMUFrame, pui8StartChr, &ui8IMURxBuff[IMU_RXBUFF_SIZE - 1] - pui8StartChr + 1);
        memcpy(&ui8IMUFrame[&ui8IMURxBuff[IMU_RXBUFF_SIZE - 1] - pui8StartChr + 1], ui8IMURxBuff, pui8EndChr - ui8IMURxBuff + 1);
        ui8IMUFrame[&ui8IMURxBuff[IMU_RXBUFF_SIZE - 1] - pui8StartChr + 1 + pui8EndChr - ui8IMURxBuff + 1] = 0;
      }
    }
    else
    {
      //search IMU_END_FRAME backward from &ui8IMURxBuff[IMU_RXBUFF_SIZE - 1] to pui8IMURxBuffPre
      pui8EndChr = memrchr(&ui8IMURxBuff[IMU_RXBUFF_SIZE - 1], IMU_END_FRAME,&ui8IMURxBuff[IMU_RXBUFF_SIZE - 1] - pui8IMURxBuffPre + 1);
      if(pui8EndChr == NULL) return false;
      //search IMU_START_FRAME backward from pui8EndChr to pui8IMURxBuffPre
      pui8StartChr = memrchr(pui8EndChr, IMU_START_FRAME,pui8EndChr - pui8IMURxBuffPre + 1);
      if(pui8StartChr == NULL) return false;
      memcpy(ui8IMUFrame, pui8StartChr, pui8EndChr - pui8StartChr + 1);
      ui8IMUFrame[pui8EndChr - pui8StartChr + 1] = 0;
    }
  }
  else //pui8IMURxBuffCur == pui8IMURxBuffPr
  {
    return false;
  }
  pui8IMURxBuffPre = pui8EndChr;
  if(Gimbal_ADIS_Parse(ui8IMUFrame) == false) return false;
  return true;
}

/* Functions ---------------------------------------------------------*/
/**
  * @brief  Gimbal_ADIS_Read_Timeout
  * @note   get raw IMU frame and call Gimbal_ADIS_Parse() with timeout
  * @param  desired timeout ui32TimeOut_ms
  * @retval true if timeout and vice versa (do not use return if you
            do not understand)
  */
bool Gimbal_ADIS_Read_IsTimeout(uint32_t ui32TimeOut_ms)
{
  static uint32_t ui32ReadDoneTime = 0;
  if(Gimbal_ADIS_Read() == false)
  {
    if(SysTick_IsTimeout(ui32ReadDoneTime, ui32TimeOut_ms) == true)
    {
        imuData.isAvailable = false;
        return true;
    }
  }
  else
  {
    ui32ReadDoneTime = SysTick_GetTick();
    imuData.isAvailable = true;
  }
  return false;
}


/*********************************END OF FILE**********************************/
