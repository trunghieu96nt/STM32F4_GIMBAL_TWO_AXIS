#include "include.h"

int32_t i32test1, i32test2, i32test3, i32test4;

void delay_us(uint16_t period)
{
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);
  TIM6->PSC = 83; // clk = SystemCoreClock /2/(PSC+1) = 1MHz
  TIM6->ARR = period-1;
  TIM6->CNT = 0;
  TIM6->EGR = 1; // update registers

  TIM6->SR  = 0; // clear overflow flag
  TIM6->CR1 = 1; // enable Timer6

  while (!TIM6->SR);
    
  TIM6->CR1 = 0; // stop Timer6
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, DISABLE);
}

void delay_ms(uint16_t period)
{
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);
  TIM6->PSC = 8399; // clk = SystemCoreClock /2 /(PSC+1) = 10KHz
  TIM6->ARR = period-1;
  TIM6->CNT = 0;
  TIM6->EGR = 1; // update registers;

  TIM6->SR  = 0; // clear overflow flag
  TIM6->CR1 = 1; // enable Timer6

  while (!TIM6->SR);
    
  TIM6->CR1 = 0; // stop Timer6
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, DISABLE);
}

void el1(void) { i32test2++; };
void el2(void) { i32test2--; };
void az1(void)
{
  i32test3++;
  Gimbal_PWM1_Set_Duty(0);
};
void az2(void)
{
  i32test3--;
  Gimbal_PWM1_Set_Duty(0);
};
void el5(void) { i32test4++; };
void el6(void) { i32test4--; };

void Board_Init()
{
  /* Enable SysTick at 10ms interrupt */
  SysTick_Config(SystemCoreClock/100);
  
  Gimbal_GPIO_Init();
  Gimbal_ENC_Init();
  Gimbal_PWM_Init();
  Gimbal_ADIS_Init();
  
  El_Home_Falling_Register(el1);
  El_Home_Rising_Register(el2);
  Az_Home_Falling_Register(az1);
  Az_Home_Rising_Register(az2);
  El_Limit_Falling_Register(el5);
  El_Limit_Rising_Register(el6);
  
  Gimbal_PWM1_Set_Duty(400);
}

int main(void)
{
  Board_Init();
    
  while(true)
  {
    if(tick_count == 100)
    {
      tick_count = 0;
      Gimbal_Led_Toggle(LED1_PIN);
    }
    else
    {
      i32test1 = Gimbal_ENC1_Get_Pos();
    } // end if(tick_count == 100)
  } //end while(true)
}


