/**
  ******************************************************************************
  * @file    gimbal_pc.c
  * @author  Vu Trung Hieu
  * @version V1.0
  * @date    8-July-2017
  * @brief   This file provides firmware functions to comunicate with pc
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
#include "gimbal_pc.h"
#include "stm32f4xx.h"
#include "gimbal_utils.h"
#include "string.h"
#include "system_timetick.h"
#include "jsmn.h"

/* Public variables ----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
uint8_t ui8STMRxBuff[STM_RXBUFF_SIZE]= {0};

/* Private function prototypes -----------------------------------------------*/
static void Gimbal_Receiver_Init(void);
static void Gimbal_Sender_Init(void);
static void Gimbal_Receiver_Handler(uint8_t *pui8STMFrame);
static bool jsoneq(const char *json, jsmntok_t *tok, const char *s);

/* Functions ---------------------------------------------------------*/
/**
  * @brief  Initialize Gimbal PC
  * @note   Including Receiver & Sender Init
  * @param  none
  * @retval none
  */
void Gimbal_PC_Init(void)
{
  Gimbal_Receiver_Init();
  Gimbal_Sender_Init();
}

/**
  * @brief  Initialize Gimbal Receiver
  * @note   ...
  * @param  none
  * @retval none
  */
void Gimbal_Receiver_Init(void)
{
  USART_InitTypeDef     USART_InitStructure;
  GPIO_InitTypeDef      GPIO_InitStructure;
  DMA_InitTypeDef       DMA_InitStructure;
  
  RCC_AHB1PeriphClockCmd(STM_PORT_CLK, ENABLE);  
  GPIO_InitStructure.GPIO_Pin   = STM_TX | STM_RX;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
  GPIO_Init(STM_PORT, &GPIO_InitStructure);
  
  GPIO_PinAFConfig(STM_PORT, STM_TX_SOURCE, STM_AF);
  GPIO_PinAFConfig(STM_PORT, STM_RX_SOURCE, STM_AF);

  RCC_APB1PeriphClockCmd(STM_USART_CLK, ENABLE);
  
  USART_InitStructure.USART_BaudRate            = STM_BAUDRATE;
  USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits            = USART_StopBits_1;
  USART_InitStructure.USART_Parity              = USART_Parity_No;
  USART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_Init(STM_USART, &USART_InitStructure);
  USART_Cmd(STM_USART, ENABLE); 
  USART_ClearFlag(STM_USART, USART_FLAG_TC);

  RCC_AHB1PeriphClockCmd(STM_AHB_PERIPH_DMA, ENABLE);
  DMA_DeInit(STM_RX_DMA_STREAM);  
  DMA_InitStructure.DMA_Channel            = STM_RX_DMA_CHANNEL;
  DMA_InitStructure.DMA_DIR                = DMA_DIR_PeripheralToMemory;
  DMA_InitStructure.DMA_Memory0BaseAddr    = (uint32_t)&ui8STMRxBuff[0];
  DMA_InitStructure.DMA_PeripheralBaseAddr = STM_DATA_REG;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_Priority           = DMA_Priority_VeryHigh;
  DMA_InitStructure.DMA_FIFOMode           = DMA_FIFOMode_Disable; //direct mode
  DMA_InitStructure.DMA_FIFOThreshold      = DMA_FIFOThreshold_Full;
  DMA_InitStructure.DMA_MemoryInc          = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_Mode               = DMA_Mode_Circular; // avoid buffer overload error
  DMA_InitStructure.DMA_BufferSize         = STM_RXBUFF_SIZE;
  DMA_InitStructure.DMA_MemoryBurst        = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_PeripheralBurst    = DMA_PeripheralBurst_Single;
  DMA_Init(STM_RX_DMA_STREAM, &DMA_InitStructure);
  /* Enable  request */
  USART_DMACmd(STM_USART, USART_DMAReq_Rx, ENABLE);  
  /* Enable DMA RX Channel */
  DMA_Cmd(STM_RX_DMA_STREAM, ENABLE);
}

static bool jsoneq(const char *json, jsmntok_t *tok, const char *s)
{
  if (tok->type == JSMN_STRING && (int) strlen(s) == tok->end - tok->start &&
      strncmp(json + tok->start, s, tok->end - tok->start) == 0) {
    return true;
  }
  return false;
}

/* Functions ---------------------------------------------------------*/
/**
  * @brief  Gimbal_Receiver_Handler
  * @note   Handle by jsonm
  * @param  uint8_t *pui8STMFrame (pointer to frame need to handle)
  * @retval none
  */
static void Gimbal_Receiver_Handler(uint8_t *pui8STMFrame)
{
  int i;
  int r;
  jsmn_parser p;
  jsmntok_t t[128]; /* We expect no more than 128 tokens */

  jsmn_init(&p);
  r = jsmn_parse(&p, (char *)pui8STMFrame, strlen((char *)pui8STMFrame), t, sizeof(t)/sizeof(t[0]));
  if (r < 0) {
    //printf("Failed to parse JSON: %d\n", r);
    return;
  }
  /* Assume the top-level element is an object */
  if (r < 1 || t[0].type != JSMN_OBJECT) {
    //printf("Object expected\n");
    return;
  }
  
  if (jsoneq((char *)pui8STMFrame, &t[1], "test") == true)
  {
    if (jsoneq((char *)pui8STMFrame, &t[2], "ok") == true)
    {
      i++;
    }
  }
  
}

/* Functions ---------------------------------------------------------*/
/**
  * @brief  Gimbal_ADIS_Read
  * @note   get raw STM frame and call Gimbal_Receiver_Handler()
  * @param  none
  * @retval true if get correctly
  */
bool Gimbal_PC_Read(void)
{
  static uint8_t *pui8STMRxBuffCur = &ui8STMRxBuff[STM_RXBUFF_SIZE - 1];
  static uint8_t *pui8STMRxBuffPre = &ui8STMRxBuff[STM_RXBUFF_SIZE - 1];
  uint8_t *pui8EndChr = NULL, *pui8StartChr = NULL;
  uint8_t ui8STMFrame[STM_FRAME_MAX_LEN];
  uint32_t ui32TotalLen = 0;
  
  if(STM_RX_DMA_STREAM->NDTR == STM_RXBUFF_SIZE)
    pui8STMRxBuffCur = &ui8STMRxBuff[STM_RXBUFF_SIZE - 1];
  else
    pui8STMRxBuffCur = &ui8STMRxBuff[STM_RXBUFF_SIZE - STM_RX_DMA_STREAM->NDTR - 1];
  
  if(pui8STMRxBuffCur > pui8STMRxBuffPre)
  {
    //search STM_END_FRAME backward from pui8STMRxBuffCur to pui8STMRxBuffPre
    pui8EndChr = memrchr(pui8STMRxBuffCur, STM_END_FRAME,pui8STMRxBuffCur - pui8STMRxBuffPre + 1);
    if(pui8EndChr == NULL) return false;
    //search STM_START_FRAME backward from pui8EndChr to pui8STMRxBuffPre
    pui8StartChr = memrchr(pui8EndChr, STM_START_FRAME,pui8EndChr - pui8STMRxBuffPre + 1);
    if(pui8StartChr == NULL) return false;
    ui32TotalLen = pui8EndChr - pui8StartChr + 1;
    if(ui32TotalLen > (STM_FRAME_MAX_LEN - 1)) return false;
    memcpy(ui8STMFrame, pui8StartChr, ui32TotalLen);
    ui8STMFrame[ui32TotalLen] = 0;
  }
  else if(pui8STMRxBuffCur < pui8STMRxBuffPre)
  {
    //search STM_END_FRAME backward from pui8STMRxBuffCur to ui8STMRxBuff
    pui8EndChr = memrchr(pui8STMRxBuffCur, STM_END_FRAME,pui8STMRxBuffCur - ui8STMRxBuff + 1);
    if(pui8EndChr != NULL)
    {
      //search STM_START_FRAME backward from pui8EndChr to ui8STMRxBuff
      pui8StartChr = memrchr(pui8EndChr, STM_START_FRAME,pui8EndChr - ui8STMRxBuff + 1);
      if(pui8StartChr != NULL)
      {
        ui32TotalLen = pui8EndChr - pui8StartChr + 1;
        if(ui32TotalLen > (STM_FRAME_MAX_LEN - 1)) return false;
        memcpy(ui8STMFrame, pui8StartChr, ui32TotalLen);
        ui8STMFrame[ui32TotalLen] = 0;
      }
      else
      {
        //search STM_START_FRAME backward from &ui8STMRxBuff[IMU_RXBUFF_SIZE - 1] to pui8STMRxBuffPre
        pui8StartChr = memrchr(&ui8STMRxBuff[STM_RXBUFF_SIZE - 1], STM_START_FRAME,&ui8STMRxBuff[STM_RXBUFF_SIZE - 1] - pui8STMRxBuffPre + 1);
        if(pui8StartChr == NULL) return false;
        ui32TotalLen = &ui8STMRxBuff[STM_RXBUFF_SIZE - 1] - pui8StartChr + 1 + pui8EndChr - ui8STMRxBuff + 1;
        if(ui32TotalLen > (STM_FRAME_MAX_LEN - 1)) return false;
        memcpy(ui8STMFrame, pui8StartChr, &ui8STMRxBuff[STM_RXBUFF_SIZE - 1] - pui8StartChr + 1);
        memcpy(&ui8STMFrame[&ui8STMRxBuff[STM_RXBUFF_SIZE - 1] - pui8StartChr + 1], ui8STMRxBuff, pui8EndChr - ui8STMRxBuff + 1);
        ui8STMFrame[ui32TotalLen] = 0;
      }
    }
    else
    {
      //search STM_END_FRAME backward from &ui8STMRxBuff[IMU_RXBUFF_SIZE - 1] to pui8STMRxBuffPre
      pui8EndChr = memrchr(&ui8STMRxBuff[STM_RXBUFF_SIZE - 1], STM_END_FRAME,&ui8STMRxBuff[STM_RXBUFF_SIZE - 1] - pui8STMRxBuffPre + 1);
      if(pui8EndChr == NULL) return false;
      //search STM_START_FRAME backward from pui8EndChr to pui8STMRxBuffPre
      pui8StartChr = memrchr(pui8EndChr, STM_START_FRAME,pui8EndChr - pui8STMRxBuffPre + 1);
      if(pui8StartChr == NULL) return false;
      ui32TotalLen = pui8EndChr - pui8StartChr + 1;
      if(ui32TotalLen > (STM_FRAME_MAX_LEN - 1)) return false;
      memcpy(ui8STMFrame, pui8StartChr, pui8EndChr - pui8StartChr + 1);
      ui8STMFrame[pui8EndChr - pui8StartChr + 1] = 0;
    }
  }
  else //pui8STMRxBuffCur == pui8STMRxBuffPr
  {
    return false;
  }
  pui8STMRxBuffPre = pui8EndChr;
  Gimbal_Receiver_Handler(ui8STMFrame);
  return true;
}

/* Functions ---------------------------------------------------------*/
/**
  * @brief  Gimbal_ADIS_Read_Timeout
  * @note   get raw STM frame and call Gimbal_Receiver_Handler
            A message is correct if it is received in ui32TimeOut_ms
  * @param  uint32_t ui32TimeOut_ms
  * @retval true if get correctly and vice versa
  */
bool Gimbal_PC_Read_Timeout(uint32_t ui32TimeOut_ms)
{
  static uint8_t *pui8STMRxBuffCur = &ui8STMRxBuff[STM_RXBUFF_SIZE - 1];
  static uint8_t *pui8STMRxBuffPre = &ui8STMRxBuff[STM_RXBUFF_SIZE - 1];
  uint8_t *pui8EndChr = NULL, *pui8StartChr = NULL;
  uint8_t ui8STMFrame[STM_FRAME_MAX_LEN];
  uint32_t ui32TotalLen = 0;
  static bool bNewData = false;
  static uint32_t ui32NewDataTime = 0;
  
  if(STM_RX_DMA_STREAM->NDTR == STM_RXBUFF_SIZE)
    pui8STMRxBuffCur = &ui8STMRxBuff[STM_RXBUFF_SIZE - 1];
  else
    pui8STMRxBuffCur = &ui8STMRxBuff[STM_RXBUFF_SIZE - STM_RX_DMA_STREAM->NDTR - 1];
  
  if(pui8STMRxBuffCur != pui8STMRxBuffPre)
  {
    //new data in ui8STMRxBuff
    if(bNewData == false)
    {
      bNewData = true;
      ui32NewDataTime = SysTick_GetTick();
    }
    //A message should receive in ui32TimeOut_ms
    if(SysTick_IsTimeout(ui32NewDataTime, ui32TimeOut_ms) == true)
    {
      pui8STMRxBuffPre = pui8STMRxBuffCur;
      bNewData = false;
      return false;
    }
    if(pui8STMRxBuffCur > pui8STMRxBuffPre)
    {
      //search STM_END_FRAME backward from pui8STMRxBuffCur to pui8STMRxBuffPre
      pui8EndChr = memrchr(pui8STMRxBuffCur, STM_END_FRAME,pui8STMRxBuffCur - pui8STMRxBuffPre + 1);
      if(pui8EndChr == NULL) return false;
      //search STM_START_FRAME backward from pui8EndChr to pui8STMRxBuffPre
      pui8StartChr = memrchr(pui8EndChr, STM_START_FRAME,pui8EndChr - pui8STMRxBuffPre + 1);
      if(pui8StartChr == NULL) return false;
      ui32TotalLen = pui8EndChr - pui8StartChr + 1;
      if(ui32TotalLen > (STM_FRAME_MAX_LEN - 1)) return false;
      memcpy(ui8STMFrame, pui8StartChr, ui32TotalLen);
      ui8STMFrame[ui32TotalLen] = 0;
    }
    else //pui8STMRxBuffCur < pui8STMRxBuffPre
    {
      //search STM_END_FRAME backward from pui8STMRxBuffCur to ui8STMRxBuff
      pui8EndChr = memrchr(pui8STMRxBuffCur, STM_END_FRAME,pui8STMRxBuffCur - ui8STMRxBuff + 1);
      if(pui8EndChr != NULL)
      {
        //search STM_START_FRAME backward from pui8EndChr to ui8STMRxBuff
        pui8StartChr = memrchr(pui8EndChr, STM_START_FRAME,pui8EndChr - ui8STMRxBuff + 1);
        if(pui8StartChr != NULL)
        {
          ui32TotalLen = pui8EndChr - pui8StartChr + 1;
          if(ui32TotalLen > (STM_FRAME_MAX_LEN - 1)) return false;
          memcpy(ui8STMFrame, pui8StartChr, ui32TotalLen);
          ui8STMFrame[ui32TotalLen] = 0;
        }
        else
        {
          //search STM_START_FRAME backward from &ui8STMRxBuff[IMU_RXBUFF_SIZE - 1] to pui8STMRxBuffPre
          pui8StartChr = memrchr(&ui8STMRxBuff[STM_RXBUFF_SIZE - 1], STM_START_FRAME,&ui8STMRxBuff[STM_RXBUFF_SIZE - 1] - pui8STMRxBuffPre + 1);
          if(pui8StartChr == NULL) return false;
          ui32TotalLen = &ui8STMRxBuff[STM_RXBUFF_SIZE - 1] - pui8StartChr + 1 + pui8EndChr - ui8STMRxBuff + 1;
          if(ui32TotalLen > (STM_FRAME_MAX_LEN - 1)) return false;
          memcpy(ui8STMFrame, pui8StartChr, &ui8STMRxBuff[STM_RXBUFF_SIZE - 1] - pui8StartChr + 1);
          memcpy(&ui8STMFrame[&ui8STMRxBuff[STM_RXBUFF_SIZE - 1] - pui8StartChr + 1], ui8STMRxBuff, pui8EndChr - ui8STMRxBuff + 1);
          ui8STMFrame[ui32TotalLen] = 0;
        }
      }
      else
      {
        //search STM_END_FRAME backward from &ui8STMRxBuff[IMU_RXBUFF_SIZE - 1] to pui8STMRxBuffPre
        pui8EndChr = memrchr(&ui8STMRxBuff[STM_RXBUFF_SIZE - 1], STM_END_FRAME,&ui8STMRxBuff[STM_RXBUFF_SIZE - 1] - pui8STMRxBuffPre + 1);
        if(pui8EndChr == NULL) return false;
        //search STM_START_FRAME backward from pui8EndChr to pui8STMRxBuffPre
        pui8StartChr = memrchr(pui8EndChr, STM_START_FRAME,pui8EndChr - pui8STMRxBuffPre + 1);
        if(pui8StartChr == NULL) return false;
        ui32TotalLen = pui8EndChr - pui8StartChr + 1;
        if(ui32TotalLen > (STM_FRAME_MAX_LEN - 1)) return false;
        memcpy(ui8STMFrame, pui8StartChr, pui8EndChr - pui8StartChr + 1);
        ui8STMFrame[pui8EndChr - pui8StartChr + 1] = 0;
      }
    }
  }
  else //pui8STMRxBuffCur == pui8STMRxBuffPr
  {
    return false;
  }
  bNewData = false;
  pui8STMRxBuffPre = pui8EndChr;
  Gimbal_Receiver_Handler(ui8STMFrame);
  return true;
}

/**
  * @brief  Initialize Gimbal Sender
  * @note   ...
  * @param  none
  * @retval none
  */
static void Gimbal_Sender_Init(void)
{
  
}
/*********************************END OF FILE**********************************/
