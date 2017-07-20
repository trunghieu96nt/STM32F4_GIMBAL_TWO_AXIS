#include "include.h"

extern STRU_IMU_DATA_T struIMUData;



void Board_Init()
{
  /* Enable SysTick at 1ms interrupt */
  SysTick_Config(SystemCoreClock / F_CTRL);
  
  Gimbal_GPIO_Init();
  Gimbal_ENC_Init();
  Gimbal_PWM_Init();
  Gimbal_ADIS_Init();
  Gimbal_PC_Init();
  Gimbal_Control_Init();
  
  EEP_Init();
  Gimbal_Params_Load_All();

  //waiting for struIMUData is available
  while(struIMUData.isAvailable == false)
  {
    Gimbal_ADIS_Read_IsTimeout(100);
    
    if(sysTickCount > 250)
    {
      sysTickCount = 0;
      Gimbal_Led_Toggle(LED2_PIN);
    }
  }
  delay_us(500000);
  Gimbal_Sender_Send((uint8_t *)"{\"Status\": \"Ok\"}", strlen("{\"Status\": \"Ok\"}"));
}

int main(void)
{
  Board_Init();
    
  while(true)
  {
    //Command from PC
    Gimbal_PC_Read_Timeout(100);
    
    //Read ADIS data
    Gimbal_ADIS_Read_IsTimeout(100);
    if(struIMUData.isAvailable == false)
    {
      if((sysTickCount % 500) < 250)
        Gimbal_Led_Set(LED2_PIN);
      else
        Gimbal_Led_Reset(LED2_PIN);
    }
    else
      Gimbal_Led_Reset(LED2_PIN);
    
    //LED Sys Status
    if(sysTickCount > 1000)
    {
      sysTickCount = 0;
      Gimbal_Led_Toggle(LED1_PIN);
    }
    
    //Controller
    if(tick_flag == true)
    {
      tick_flag = false;
      Gimbal_Control();
    }
  }
}


