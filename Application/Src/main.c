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
  
  delay_us(1000000);
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
  //Gimbal_PWM1_Set_Duty(-70);
  Gimbal_Control_Home();
}

int main(void)
{
  Board_Init();
    
  while(true)
  {
    Gimbal_PC_Read_Timeout(100);
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
    if(sysTickCount > 1000)
    {
      sysTickCount = 0;
      Gimbal_Led_Toggle(LED1_PIN);
    }
    if(tick_flag == true)
    {
      tick_flag = false;
      Gimbal_Control();
    }
  }
}


