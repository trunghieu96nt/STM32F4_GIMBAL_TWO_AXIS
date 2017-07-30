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

#include "stdlib.h"
#include "pid.h"
#include "gimbal_params.h"
#include "gimbal_control.h"

/* Public variables ----------------------------------------------------------*/
extern STRU_PID_T stru_PID_AZ_Manual_Pos;
extern STRU_PID_T stru_PID_EL_Manual_Pos;

extern STRU_PID_T stru_PID_EL_Pointing_Outer;
extern STRU_PID_T stru_PID_AZ_Pointing_Inner;
/* Private define ------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
uint8_t au8STMRxBuff[STM_RXBUFF_SIZE]= {0};
uint8_t au8HMITxBuff[HMI_RXBUFF_SIZE]= {0};

/* Private function prototypes -----------------------------------------------*/
void Gimbal_Receiver_Init(void);
void Gimbal_Sender_Init(void);
void Gimbal_Receiver_Handler(uint8_t *pui8STMFrame);
bool jsoneq(const char *json, jsmntok_t *tok, const char *s);

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

  /* DMA RX configuration */
  RCC_AHB1PeriphClockCmd(STM_AHB_PERIPH_DMA, ENABLE);
  DMA_DeInit(STM_RX_DMA_STREAM);  
  DMA_InitStructure.DMA_Channel            = STM_RX_DMA_CHANNEL;
  DMA_InitStructure.DMA_DIR                = DMA_DIR_PeripheralToMemory;
  DMA_InitStructure.DMA_Memory0BaseAddr    = (uint32_t)&au8STMRxBuff[0];
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
  
  /* DMA TX configuration */
  DMA_DeInit(STM_TX_DMA_STREAM);
  DMA_InitStructure.DMA_Channel            = STM_TX_DMA_CHANNEL;
  DMA_InitStructure.DMA_DIR                = DMA_DIR_MemoryToPeripheral;
  DMA_InitStructure.DMA_Memory0BaseAddr    = 0; //(uint32_t)&au8HMITxBuff[0]; //temporary
  DMA_InitStructure.DMA_PeripheralBaseAddr = STM_DATA_REG;   
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_Priority           = DMA_Priority_Low;
  DMA_InitStructure.DMA_FIFOMode           = DMA_FIFOMode_Disable;
  DMA_InitStructure.DMA_FIFOThreshold      = DMA_FIFOThreshold_Full;
  DMA_InitStructure.DMA_MemoryInc          = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_Mode               = DMA_Mode_Normal;
  DMA_InitStructure.DMA_BufferSize         = 0;
  DMA_InitStructure.DMA_MemoryBurst        = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_PeripheralBurst    = DMA_PeripheralBurst_Single;
  DMA_Init(STM_TX_DMA_STREAM, &DMA_InitStructure);
  
  // Enable DMA Stream Transfer Complete interrupt
  //DMA_ITConfig(STM_TX_DMA_STREAM, DMA_IT_TC, ENABLE);
  
  // Enable USART DMA TX request
  USART_DMACmd(STM_USART, USART_DMAReq_Tx, ENABLE);  
}

/**
  * @brief  Gimbal_Receiver_Send
  * @note   ...
  * @param  const uint8_t *pu8Message: pointer message to send
  * @param  uint32_t ui32MessageSize: number of char to send
  * @retval true if success and vice versa
  */
bool Gimbal_Receiver_Send(const uint8_t *pu8Message, uint32_t ui32MessageSize)
{
  if(ui32MessageSize > STM_TXBUFF_SIZE)
  {
    return false;
  }
  else
  {
    //check flag
    
    //clear flag
    DMA_ClearFlag(STM_TX_DMA_STREAM, STM_TX_DMA_FLAG);
    DMA_MemoryTargetConfig(STM_TX_DMA_STREAM, (uint32_t)pu8Message, DMA_Memory_0);
    DMA_SetCurrDataCounter(STM_TX_DMA_STREAM, ui32MessageSize);
    //STM_TX_DMA_STREAM->NDTR = BUFF_SIZE;
    DMA_Cmd(STM_TX_DMA_STREAM, ENABLE);
    return true;
  }
}

/**
  * @brief  jsoneq
  * @note   compare token
  * @param  const char *json: json string
  * @param  jsmntok_t *tok: parsed token
  * @param  const char *s: string to search
  * @retval none
  */
bool jsoneq(const char *json, jsmntok_t *tok, const char *s)
{
  if (tok->type == JSMN_STRING && (int) strlen(s) == tok->end - tok->start &&
      strncmp(json + tok->start, s, tok->end - tok->start) == 0) {
    return true;
  }
  return false;
}

/**
  * @brief  Gimbal_Receiver_Handler
  * @note   Handle by jsonm
  * @param  uint8_t *pui8STMFrame (pointer to frame need to handle)
  * @retval none
  */
void Gimbal_Receiver_Handler(uint8_t *pui8STMFrame)
{
  int r;
  jsmn_parser p;
  jsmntok_t t[128]; /* We expect no more than 128 tokens */
  uint8_t ui8HandleBuff[20];

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
  
  /* Handle data */
  if(jsoneq((char *)pui8STMFrame, &t[2], "PIDTunner") == true)
  {
    if(jsoneq((char *)pui8STMFrame, &t[4], "AZ_MANUAL_POS") == true)
    {
      if(jsoneq((char *)pui8STMFrame, &t[5], "Save") == true)
      {
        Gimbal_Params_Save(PARAMS_PID_AZ_MANUAL_POS, (uint8_t *)&stru_PID_AZ_Manual_Pos);
      }
      else
      {
        memcpy(ui8HandleBuff, pui8STMFrame + t[6].start, t[6].end - t[6].start);
        ui8HandleBuff[t[6].end - t[6].start] = 0;
        PID_Kp_Set(&stru_PID_AZ_Manual_Pos, (float)atoi((char *)ui8HandleBuff) * 0.001f);
        
        memcpy(ui8HandleBuff, pui8STMFrame + t[8].start, t[8].end - t[8].start);
        ui8HandleBuff[t[8].end - t[8].start] = 0;
        PID_Ki_Set(&stru_PID_AZ_Manual_Pos, (float)atoi((char *)ui8HandleBuff) * 0.001f);
        
        memcpy(ui8HandleBuff, pui8STMFrame + t[10].start, t[10].end - t[10].start);
        ui8HandleBuff[t[10].end - t[10].start] = 0;
        PID_Kd_Set(&stru_PID_AZ_Manual_Pos, (float)atoi((char *)ui8HandleBuff) * 0.001f);
      }
    }
    else if(jsoneq((char *)pui8STMFrame, &t[4], "EL_MANUAL_POS") == true)
    {
      if(jsoneq((char *)pui8STMFrame, &t[5], "Save") == true)
      {
        Gimbal_Params_Save(PARAMS_PID_EL_MANUAL_POS, (uint8_t *)&stru_PID_EL_Manual_Pos);
      }
      else
      {
        memcpy(ui8HandleBuff, pui8STMFrame + t[6].start, t[6].end - t[6].start);
        ui8HandleBuff[t[6].end - t[6].start] = 0;
        PID_Kp_Set(&stru_PID_EL_Manual_Pos, (float)atoi((char *)ui8HandleBuff) * 0.001f);
        
        memcpy(ui8HandleBuff, pui8STMFrame + t[8].start, t[8].end - t[8].start);
        ui8HandleBuff[t[8].end - t[8].start] = 0;
        PID_Ki_Set(&stru_PID_EL_Manual_Pos, (float)atoi((char *)ui8HandleBuff) * 0.001f);
        
        memcpy(ui8HandleBuff, pui8STMFrame + t[10].start, t[10].end - t[10].start);
        ui8HandleBuff[t[10].end - t[10].start] = 0;
        PID_Kd_Set(&stru_PID_EL_Manual_Pos, (float)atoi((char *)ui8HandleBuff) * 0.001f);
      }
    }
    else if(jsoneq((char *)pui8STMFrame, &t[4], "AZ_POINTING_INNER") == true)
    {
      if(jsoneq((char *)pui8STMFrame, &t[5], "Save") == true)
      {
        Gimbal_Params_Save(PARAMS_PID_EL_MANUAL_POS, (uint8_t *)&stru_PID_EL_Manual_Pos);
      }
      else
      {
        memcpy(ui8HandleBuff, pui8STMFrame + t[6].start, t[6].end - t[6].start);
        ui8HandleBuff[t[6].end - t[6].start] = 0;
        PID_Kp_Set(&stru_PID_AZ_Pointing_Inner, (float)atoi((char *)ui8HandleBuff) * 0.001f);
        
        memcpy(ui8HandleBuff, pui8STMFrame + t[8].start, t[8].end - t[8].start);
        ui8HandleBuff[t[8].end - t[8].start] = 0;
        PID_Ki_Set(&stru_PID_AZ_Pointing_Inner, (float)atoi((char *)ui8HandleBuff) * 0.001f);
        
        memcpy(ui8HandleBuff, pui8STMFrame + t[10].start, t[10].end - t[10].start);
        ui8HandleBuff[t[10].end - t[10].start] = 0;
        PID_Kd_Set(&stru_PID_AZ_Pointing_Inner, (float)atoi((char *)ui8HandleBuff) * 0.00001f);
      }
    }
  }
  else if(jsoneq((char *)pui8STMFrame, &t[2], "Control") == true)
  {
    if(jsoneq((char *)pui8STMFrame, &t[4], "Manual") == true)
    {
      if(jsoneq((char *)pui8STMFrame, &t[6], "AZ") == true)
      {
        memcpy(ui8HandleBuff, pui8STMFrame + t[8].start, t[8].end - t[8].start);
        ui8HandleBuff[t[8].end - t[8].start] = 0;
        PID_MaxSetPointStep_Set(&stru_PID_AZ_Manual_Pos, (float)atoi((char *)ui8HandleBuff) * 0.1f * 0.001f);
        
        memcpy(ui8HandleBuff, pui8STMFrame + t[10].start, t[10].end - t[10].start);
        ui8HandleBuff[t[10].end - t[10].start] = 0;
        PID_SetPoint_Set(&stru_PID_AZ_Manual_Pos, (float)atoi((char *)ui8HandleBuff) * 0.1f);
      }
      else if(jsoneq((char *)pui8STMFrame, &t[6], "EL") == true)
      {
        memcpy(ui8HandleBuff, pui8STMFrame + t[8].start, t[8].end - t[8].start);
        ui8HandleBuff[t[8].end - t[8].start] = 0;
        PID_MaxSetPointStep_Set(&stru_PID_EL_Manual_Pos, (float)atoi((char *)ui8HandleBuff) * 0.1f * 0.001f);
        
        memcpy(ui8HandleBuff, pui8STMFrame + t[10].start, t[10].end - t[10].start);
        ui8HandleBuff[t[10].end - t[10].start] = 0;
        PID_SetPoint_Set(&stru_PID_EL_Manual_Pos, (float)atoi((char *)ui8HandleBuff) * 0.1f);
      }
    }
    else if(jsoneq((char *)pui8STMFrame, &t[4], "Pointing") == true)
    {
      
    }
    else if(jsoneq((char *)pui8STMFrame, &t[4], "Tracking") == true)
    {
      
    }
  }
  else if(jsoneq((char *)pui8STMFrame, &t[2], "ChangeMode") == true)
  {
    if(jsoneq((char *)pui8STMFrame, &t[4], "Idle") == true)
    {
      if(jsoneq((char *)pui8STMFrame, &t[6], "AZ") == true)
        Gimbal_Control_Change_Mode(STATE_IDLE, STATE_KEEP);
      else if(jsoneq((char *)pui8STMFrame, &t[6], "EL") == true)
        Gimbal_Control_Change_Mode(STATE_KEEP, STATE_IDLE);
      else if(jsoneq((char *)pui8STMFrame, &t[6], "Both") == true)
        Gimbal_Control_Change_Mode(STATE_IDLE, STATE_IDLE);
    }
    else if(jsoneq((char *)pui8STMFrame, &t[4], "Home") == true)
    {
      if(jsoneq((char *)pui8STMFrame, &t[6], "AZ") == true)
        Gimbal_Control_Change_Mode(STATE_HOME, STATE_KEEP);
      else if(jsoneq((char *)pui8STMFrame, &t[6], "EL") == true)
        Gimbal_Control_Change_Mode(STATE_KEEP, STATE_HOME);
      else if(jsoneq((char *)pui8STMFrame, &t[6], "Both") == true)
        Gimbal_Control_Change_Mode(STATE_HOME, STATE_HOME);
    }
    else if(jsoneq((char *)pui8STMFrame, &t[4], "Manual") == true)
    {
      if(jsoneq((char *)pui8STMFrame, &t[6], "AZ") == true)
        Gimbal_Control_Change_Mode(STATE_MANUAL, STATE_KEEP);
      else if(jsoneq((char *)pui8STMFrame, &t[6], "EL") == true)
        Gimbal_Control_Change_Mode(STATE_KEEP, STATE_MANUAL);
      else if(jsoneq((char *)pui8STMFrame, &t[6], "Both") == true)
        Gimbal_Control_Change_Mode(STATE_MANUAL, STATE_MANUAL);
    }
    else if(jsoneq((char *)pui8STMFrame, &t[4], "Pointing") == true)
    {
      if(jsoneq((char *)pui8STMFrame, &t[6], "AZ") == true)
        Gimbal_Control_Change_Mode(STATE_POINTING, STATE_KEEP);
      else if(jsoneq((char *)pui8STMFrame, &t[6], "EL") == true)
        Gimbal_Control_Change_Mode(STATE_KEEP, STATE_POINTING);
      else if(jsoneq((char *)pui8STMFrame, &t[6], "Both") == true)
        Gimbal_Control_Change_Mode(STATE_POINTING, STATE_POINTING);
    }
    else if(jsoneq((char *)pui8STMFrame, &t[4], "Tracking") == true)
    {
      if(jsoneq((char *)pui8STMFrame, &t[6], "AZ") == true)
        Gimbal_Control_Change_Mode(STATE_TRACKING, STATE_KEEP);
      else if(jsoneq((char *)pui8STMFrame, &t[6], "EL") == true)
        Gimbal_Control_Change_Mode(STATE_KEEP, STATE_TRACKING);
      else if(jsoneq((char *)pui8STMFrame, &t[6], "Both") == true)
        Gimbal_Control_Change_Mode(STATE_TRACKING, STATE_TRACKING);
    }
    else if(jsoneq((char *)pui8STMFrame, &t[4], "Sine") == true)
    {
      if(jsoneq((char *)pui8STMFrame, &t[6], "AZ") == true)
        Gimbal_Control_Change_Mode(STATE_SINE, STATE_KEEP);
      else if(jsoneq((char *)pui8STMFrame, &t[6], "EL") == true)
        Gimbal_Control_Change_Mode(STATE_KEEP, STATE_SINE);
      else if(jsoneq((char *)pui8STMFrame, &t[6], "Both") == true)
        Gimbal_Control_Change_Mode(STATE_SINE, STATE_SINE);
    }
  }
  Gimbal_Receiver_Send((uint8_t *)"{\"Status\": \"Ok\"}", strlen("{\"Status\": \"Ok\"}"));
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
  static uint8_t *pu8STMRxBuffCur = &au8STMRxBuff[STM_RXBUFF_SIZE - 1];
  static uint8_t *pu8STMRxBuffPre = &au8STMRxBuff[STM_RXBUFF_SIZE - 1];
  uint8_t *pu8EndChr = NULL, *pu8StartChr = NULL;
  uint8_t au8STMFrame[STM_FRAME_MAX_LEN];
  uint32_t ui32TotalLen = 0;
  
  if(STM_RX_DMA_STREAM->NDTR == STM_RXBUFF_SIZE)
    pu8STMRxBuffCur = &au8STMRxBuff[STM_RXBUFF_SIZE - 1];
  else
    pu8STMRxBuffCur = &au8STMRxBuff[STM_RXBUFF_SIZE - STM_RX_DMA_STREAM->NDTR - 1];
  
  if(pu8STMRxBuffCur > pu8STMRxBuffPre)
  {
    //search STM_END_FRAME backward from pu8STMRxBuffCur to pu8STMRxBuffPre
    pu8EndChr = memrchr(pu8STMRxBuffCur, STM_END_FRAME,pu8STMRxBuffCur - pu8STMRxBuffPre + 1);
    if(pu8EndChr == NULL) return false;
    //search STM_START_FRAME backward from pu8EndChr to pu8STMRxBuffPre
    pu8StartChr = memrchr(pu8EndChr, STM_START_FRAME,pu8EndChr - pu8STMRxBuffPre + 1);
    if(pu8StartChr == NULL) return false;
    ui32TotalLen = pu8EndChr - pu8StartChr + 1;
    if(ui32TotalLen > (STM_FRAME_MAX_LEN - 1)) return false;
    memcpy(au8STMFrame, pu8StartChr, ui32TotalLen);
    au8STMFrame[ui32TotalLen] = 0;
  }
  else if(pu8STMRxBuffCur < pu8STMRxBuffPre)
  {
    //search STM_END_FRAME backward from pu8STMRxBuffCur to au8STMRxBuff
    pu8EndChr = memrchr(pu8STMRxBuffCur, STM_END_FRAME,pu8STMRxBuffCur - au8STMRxBuff + 1);
    if(pu8EndChr != NULL)
    {
      //search STM_START_FRAME backward from pu8EndChr to au8STMRxBuff
      pu8StartChr = memrchr(pu8EndChr, STM_START_FRAME,pu8EndChr - au8STMRxBuff + 1);
      if(pu8StartChr != NULL)
      {
        ui32TotalLen = pu8EndChr - pu8StartChr + 1;
        if(ui32TotalLen > (STM_FRAME_MAX_LEN - 1)) return false;
        memcpy(au8STMFrame, pu8StartChr, ui32TotalLen);
        au8STMFrame[ui32TotalLen] = 0;
      }
      else
      {
        //search STM_START_FRAME backward from &au8STMRxBuff[IMU_RXBUFF_SIZE - 1] to pu8STMRxBuffPre
        pu8StartChr = memrchr(&au8STMRxBuff[STM_RXBUFF_SIZE - 1], STM_START_FRAME,&au8STMRxBuff[STM_RXBUFF_SIZE - 1] - pu8STMRxBuffPre + 1);
        if(pu8StartChr == NULL) return false;
        ui32TotalLen = &au8STMRxBuff[STM_RXBUFF_SIZE - 1] - pu8StartChr + 1 + pu8EndChr - au8STMRxBuff + 1;
        if(ui32TotalLen > (STM_FRAME_MAX_LEN - 1)) return false;
        memcpy(au8STMFrame, pu8StartChr, &au8STMRxBuff[STM_RXBUFF_SIZE - 1] - pu8StartChr + 1);
        memcpy(&au8STMFrame[&au8STMRxBuff[STM_RXBUFF_SIZE - 1] - pu8StartChr + 1], au8STMRxBuff, pu8EndChr - au8STMRxBuff + 1);
        au8STMFrame[ui32TotalLen] = 0;
      }
    }
    else
    {
      //search STM_END_FRAME backward from &au8STMRxBuff[IMU_RXBUFF_SIZE - 1] to pu8STMRxBuffPre
      pu8EndChr = memrchr(&au8STMRxBuff[STM_RXBUFF_SIZE - 1], STM_END_FRAME,&au8STMRxBuff[STM_RXBUFF_SIZE - 1] - pu8STMRxBuffPre + 1);
      if(pu8EndChr == NULL) return false;
      //search STM_START_FRAME backward from pu8EndChr to pu8STMRxBuffPre
      pu8StartChr = memrchr(pu8EndChr, STM_START_FRAME,pu8EndChr - pu8STMRxBuffPre + 1);
      if(pu8StartChr == NULL) return false;
      ui32TotalLen = pu8EndChr - pu8StartChr + 1;
      if(ui32TotalLen > (STM_FRAME_MAX_LEN - 1)) return false;
      memcpy(au8STMFrame, pu8StartChr, pu8EndChr - pu8StartChr + 1);
      au8STMFrame[pu8EndChr - pu8StartChr + 1] = 0;
    }
  }
  else //pu8STMRxBuffCur == pui8STMRxBuffPr
  {
    return false;
  }
  pu8STMRxBuffPre = pu8EndChr;
  Gimbal_Receiver_Handler(au8STMFrame);
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
  static uint8_t *pu8STMRxBuffCur = &au8STMRxBuff[STM_RXBUFF_SIZE - 1];
  static uint8_t *pu8STMRxBuffPre = &au8STMRxBuff[STM_RXBUFF_SIZE - 1];
  uint8_t *pu8EndChr = NULL, *pu8StartChr = NULL;
  uint8_t au8STMFrame[STM_FRAME_MAX_LEN];
  uint32_t ui32TotalLen = 0;
  static bool bNewData = false;
  static uint32_t ui32NewDataTime = 0;
  
  if(STM_RX_DMA_STREAM->NDTR == STM_RXBUFF_SIZE)
    pu8STMRxBuffCur = &au8STMRxBuff[STM_RXBUFF_SIZE - 1];
  else
    pu8STMRxBuffCur = &au8STMRxBuff[STM_RXBUFF_SIZE - STM_RX_DMA_STREAM->NDTR - 1];
  
  if(pu8STMRxBuffCur != pu8STMRxBuffPre)
  {
    //new data in au8STMRxBuff
    if(bNewData == false)
    {
      bNewData = true;
      ui32NewDataTime = SysTick_GetTick();
    }
    //A message should receive in ui32TimeOut_ms
    if(SysTick_IsTimeout(ui32NewDataTime, ui32TimeOut_ms) == true)
    {
      pu8STMRxBuffPre = pu8STMRxBuffCur;
      bNewData = false;
      return false;
    }
    if(pu8STMRxBuffCur > pu8STMRxBuffPre)
    {
      //search STM_END_FRAME backward from pu8STMRxBuffCur to pu8STMRxBuffPre
      pu8EndChr = memrchr(pu8STMRxBuffCur, STM_END_FRAME,pu8STMRxBuffCur - pu8STMRxBuffPre + 1);
      if(pu8EndChr == NULL) return false;
      //search STM_START_FRAME backward from pu8EndChr to pu8STMRxBuffPre
      pu8StartChr = memrchr(pu8EndChr, STM_START_FRAME,pu8EndChr - pu8STMRxBuffPre + 1);
      if(pu8StartChr == NULL) return false;
      ui32TotalLen = pu8EndChr - pu8StartChr + 1;
      if(ui32TotalLen > (STM_FRAME_MAX_LEN - 1)) return false;
      memcpy(au8STMFrame, pu8StartChr, ui32TotalLen);
      au8STMFrame[ui32TotalLen] = 0;
    }
    else //pu8STMRxBuffCur < pu8STMRxBuffPre
    {
      //search STM_END_FRAME backward from pu8STMRxBuffCur to au8STMRxBuff
      pu8EndChr = memrchr(pu8STMRxBuffCur, STM_END_FRAME,pu8STMRxBuffCur - au8STMRxBuff + 1);
      if(pu8EndChr != NULL)
      {
        //search STM_START_FRAME backward from pu8EndChr to au8STMRxBuff
        pu8StartChr = memrchr(pu8EndChr, STM_START_FRAME,pu8EndChr - au8STMRxBuff + 1);
        if(pu8StartChr != NULL)
        {
          ui32TotalLen = pu8EndChr - pu8StartChr + 1;
          if(ui32TotalLen > (STM_FRAME_MAX_LEN - 1)) return false;
          memcpy(au8STMFrame, pu8StartChr, ui32TotalLen);
          au8STMFrame[ui32TotalLen] = 0;
        }
        else
        {
          //search STM_START_FRAME backward from &au8STMRxBuff[IMU_RXBUFF_SIZE - 1] to pu8STMRxBuffPre
          pu8StartChr = memrchr(&au8STMRxBuff[STM_RXBUFF_SIZE - 1], STM_START_FRAME,&au8STMRxBuff[STM_RXBUFF_SIZE - 1] - pu8STMRxBuffPre + 1);
          if(pu8StartChr == NULL) return false;
          ui32TotalLen = &au8STMRxBuff[STM_RXBUFF_SIZE - 1] - pu8StartChr + 1 + pu8EndChr - au8STMRxBuff + 1;
          if(ui32TotalLen > (STM_FRAME_MAX_LEN - 1)) return false;
          memcpy(au8STMFrame, pu8StartChr, &au8STMRxBuff[STM_RXBUFF_SIZE - 1] - pu8StartChr + 1);
          memcpy(&au8STMFrame[&au8STMRxBuff[STM_RXBUFF_SIZE - 1] - pu8StartChr + 1], au8STMRxBuff, pu8EndChr - au8STMRxBuff + 1);
          au8STMFrame[ui32TotalLen] = 0;
        }
      }
      else
      {
        //search STM_END_FRAME backward from &au8STMRxBuff[IMU_RXBUFF_SIZE - 1] to pu8STMRxBuffPre
        pu8EndChr = memrchr(&au8STMRxBuff[STM_RXBUFF_SIZE - 1], STM_END_FRAME,&au8STMRxBuff[STM_RXBUFF_SIZE - 1] - pu8STMRxBuffPre + 1);
        if(pu8EndChr == NULL) return false;
        //search STM_START_FRAME backward from pu8EndChr to pu8STMRxBuffPre
        pu8StartChr = memrchr(pu8EndChr, STM_START_FRAME,pu8EndChr - pu8STMRxBuffPre + 1);
        if(pu8StartChr == NULL) return false;
        ui32TotalLen = pu8EndChr - pu8StartChr + 1;
        if(ui32TotalLen > (STM_FRAME_MAX_LEN - 1)) return false;
        memcpy(au8STMFrame, pu8StartChr, pu8EndChr - pu8StartChr + 1);
        au8STMFrame[pu8EndChr - pu8StartChr + 1] = 0;
      }
    }
  }
  else //pu8STMRxBuffCur == pui8STMRxBuffPr
  {
    return false;
  }
  bNewData = false;
  pu8STMRxBuffPre = pu8EndChr;
  Gimbal_Receiver_Handler(au8STMFrame);
  return true;
}

/**
  * @brief  Initialize Gimbal Sender
  * @note   ...
  * @param  none
  * @retval none
  */
void Gimbal_Sender_Init(void)
{
  USART_InitTypeDef   USART_InitStructure;
  GPIO_InitTypeDef    GPIO_InitStructure;
  DMA_InitTypeDef     DMA_InitStructure;
  
  RCC_AHB1PeriphClockCmd(HMI_PORT_CLK, ENABLE);
  /* GPIO configuration */
  GPIO_InitStructure.GPIO_Pin   = HMI_TX;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
  GPIO_Init(HMI_PORT, &GPIO_InitStructure);
  GPIO_PinAFConfig(HMI_PORT, HMI_TX_SOURCE, HMI_AF);
  
  /* USART configuration */
  RCC_APB1PeriphClockCmd(HMI_USART_CLK, ENABLE);
  USART_InitStructure.USART_BaudRate            = HMI_BAUDRATE;
  USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits            = USART_StopBits_1;
  USART_InitStructure.USART_Parity              = USART_Parity_No;
  USART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
  USART_InitStructure.USART_HardwareFlowControl =  USART_HardwareFlowControl_None;
  USART_Init(HMI_USART, &USART_InitStructure);
  USART_Cmd(HMI_USART, ENABLE);
    
  USART_ClearFlag(HMI_USART, USART_FLAG_TC);
  //USART_ClearFlag(HMI_USART, USART_FLAG_RXNE);

  RCC_AHB1PeriphClockCmd(HMI_AHB_PERIPH_DMA, ENABLE);
  /* DMA TX configuration */
  DMA_DeInit(HMI_TX_DMA_STREAM);
  DMA_InitStructure.DMA_Channel            = HMI_TX_DMA_CHANNEL;
  DMA_InitStructure.DMA_DIR                = DMA_DIR_MemoryToPeripheral;
  DMA_InitStructure.DMA_Memory0BaseAddr    = 0; //(uint32_t)&au8HMITxBuff[0]; //temporary
  DMA_InitStructure.DMA_PeripheralBaseAddr = HMI_DATA_REG;   
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_Priority           = DMA_Priority_Low;
  DMA_InitStructure.DMA_FIFOMode           = DMA_FIFOMode_Disable;
  DMA_InitStructure.DMA_FIFOThreshold      = DMA_FIFOThreshold_Full;
  DMA_InitStructure.DMA_MemoryInc          = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_Mode               = DMA_Mode_Normal;
  DMA_InitStructure.DMA_BufferSize         = 0;
  DMA_InitStructure.DMA_MemoryBurst        = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_PeripheralBurst    = DMA_PeripheralBurst_Single;
  DMA_Init(HMI_TX_DMA_STREAM, &DMA_InitStructure);
  
  // Enable DMA Stream Transfer Complete interrupt
  //DMA_ITConfig(HMI_TX_DMA_STREAM, DMA_IT_TC, ENABLE);
  
  // Enable USART DMA TX request
  USART_DMACmd(HMI_USART, USART_DMAReq_Tx, ENABLE);  
  // Enable DMA TX Channel
  //DMA_Cmd(HMI_TX_DMA_STREAM, ENABLE);
}

/**
  * @brief  Gimbal_Sender_Send
  * @note   ...
  * @param  const uint8_t *pu8Message: pointer message to send
  * @param  uint32_t ui32MessageSize: number of char to send
  * @retval true if success and vice versa
  */
bool Gimbal_Sender_Send(const uint8_t *pu8Message, uint32_t ui32MessageSize)
{
  if(ui32MessageSize > HMI_TXBUFF_SIZE)
  {
    return false;
  }
  else
  {
    //check flag
    
    //clear flag
    DMA_ClearFlag(HMI_TX_DMA_STREAM, HMI_TX_DMA_FLAG);
    DMA_MemoryTargetConfig(HMI_TX_DMA_STREAM, (uint32_t)pu8Message, DMA_Memory_0);
    DMA_SetCurrDataCounter(HMI_TX_DMA_STREAM, ui32MessageSize);
    //HMI_TX_DMA_STREAM->NDTR = BUFF_SIZE;
    DMA_Cmd(HMI_TX_DMA_STREAM, ENABLE);
    return true;
  }
}
/*********************************END OF FILE**********************************/
