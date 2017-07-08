#include "include.h"

extern IMUData_t imuData;

void delay_us(uint32_t micros)
{
  RCC_ClocksTypeDef RCC_Clocks;
  /* Get system clocks */
  RCC_GetClocksFreq(&RCC_Clocks);
  micros = micros * (RCC_Clocks.HCLK_Frequency / 4000000) - 10;
  /* 4 cycles for one loop */
  while (micros--);
}

void Board_Init()
{
  /* Enable SysTick at 1ms interrupt */
  SysTick_Config(SystemCoreClock / F_CTRL);
  
  Gimbal_GPIO_Init();
  Gimbal_ENC_Init();
  Gimbal_PWM_Init();
  Gimbal_ADIS_Init();
  Gimbal_PC_Init();
  
  delay_us(1000000);
  //waiting for imuData is available
//  while(imuData.isAvailable == false)
//  {
//    Gimbal_ADIS_Read_IsTimeout(100);
//    if(sysTickCount > 250)
//    {
//      sysTickCount = 0;
//      Gimbal_Led_Toggle(LED2_PIN);
//    }
//  }
  Gimbal_Control_Home();
}

int main(void)
{
  Board_Init();
    
  while(true)
  {
    Gimbal_PC_Read_Timeout(100);
    Gimbal_ADIS_Read_IsTimeout(100);
    if(imuData.isAvailable == false)
    {
      if((sysTickCount % 500) < 250)
        Gimbal_Led_Set(LED2_PIN);
      else
        Gimbal_Led_Reset(LED2_PIN);
    }
    else
      Gimbal_Led_Reset(LED2_PIN);
    if(sysTickCount > 1000)
    {
      sysTickCount = 0;
      Gimbal_Led_Toggle(LED1_PIN);
    }
    if(tick_flag == true)
    {
      tick_flag = false;
    }
  }
}


