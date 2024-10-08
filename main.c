#include "main.h"
#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
/* Private typedef -----------------------------------------------------------*/
//EXTI_InitTypeDef   EXTI_InitStructure;

__IO uint32_t TimingDelay;

static void EXTILine0_Config(void);
static void store_delay_count_per_rotation(uint16_t delay_count);

void TIM4_IRQHandler(void);
void PWM120Degreedutyset(unsigned int pwm_hall_uvw,float pwm_duty);
//---------------------------------------------------------------------------------------
int timer_count=0,timer_count_set=100;
//---------------------------------------------------------------------------------------
void TIM_Config(void);
void PWM_Config(void);
void GPIO_PA5(void);
TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
TIM_OCInitTypeDef  TIM_OCInitStructure;

uint16_t CCR1_Val = 0;
uint16_t CCR2_Val = 0;
uint16_t CCR3_Val = 0;
uint16_t CCR4_Val = 0;
uint16_t PrescalerValue = 0;

int TIM_Period_set=2099;//1049
float duty=0.35;
int hall_circle[6]={3,1,5,4,6,2};
int hall_count=0;
int hall=0;
uint16_t delay_counts_per_rotation[100]={0};
uint16_t interruptCount=0;
uint8_t rotation_index = 0;

__IO uint32_t delay_count=0;
//---------------------------------------------------------------------------------------
int main(void)
{  
  RCC_ClocksTypeDef RCC_Clocks;
  /* SysTick end of count event each 1ms */
  RCC_GetClocksFreq(&RCC_Clocks);
  SysTick_Config(RCC_Clocks.HCLK_Frequency / 1000);
	

  
  /* Initialize LEDs mounted on EVAL board */
  STM_EVAL_LEDInit(LED3);
  STM_EVAL_LEDInit(LED4);
  
	 TIM_Config();
	 PWM_Config();
	 GPIO_PA5();
  /* Configure EXTI Line0 (connected to PA0 pin) in interrupt mode */
  EXTILine0_Config();
  //TIM_SetCompare1(TIM4,TIM_Period_set*0.1);
	GPIO_Write(GPIOC, 0x00);
  while (1)
  {
		//hall = GPIO_ReadInputData(GPIOC)&0x07;
		//PWM120Degreedutyset(hall,duty);
		Delay(1);
		
		if(interruptCount>23)
		{	store_delay_count_per_rotation(delay_count);
			delay_count=0;
			interruptCount=0;
			GPIO_ToggleBits(GPIOA, GPIO_Pin_5);
		}
		else
			delay_count++;
		
		
  }
}
void store_delay_count_per_rotation(uint16_t delay_count) {
    if (rotation_index < 100) {
        delay_counts_per_rotation[rotation_index] = delay_count;
        rotation_index++;
    }
}
void PWM120Degreedutyset(unsigned int pwm_hall_uvw,float pwm_duty)
{
    switch(pwm_hall_uvw)
    {
				case 3: //CA
				TIM_SetCompare1(TIM4,0);
				TIM_SetCompare2(TIM4,0);
				TIM_SetCompare3(TIM4,TIM_Period_set*duty);
				GPIO_WriteBit(GPIOC, GPIO_Pin_0, 0);
				GPIO_WriteBit(GPIOC, GPIO_Pin_1, 0xff);
				GPIO_WriteBit(GPIOC, GPIO_Pin_2, 0xff);
				break;	//0P 10 00 
				
				case 1:  //CB
				TIM_SetCompare1(TIM4,0);
				TIM_SetCompare2(TIM4,0);
				TIM_SetCompare3(TIM4,TIM_Period_set*duty);
				GPIO_WriteBit(GPIOC, GPIO_Pin_0, 0xff);
				GPIO_WriteBit(GPIOC, GPIO_Pin_1, 0);
				GPIO_WriteBit(GPIOC, GPIO_Pin_2, 0xff);
				break;	//0P 00 10
				
				case 5: //AB
				TIM_SetCompare1(TIM4,TIM_Period_set*duty);
				TIM_SetCompare2(TIM4,0);
				TIM_SetCompare3(TIM4,0);
				GPIO_WriteBit(GPIOC, GPIO_Pin_0, 0xff);
				GPIO_WriteBit(GPIOC, GPIO_Pin_1, 0);
				GPIO_WriteBit(GPIOC, GPIO_Pin_2, 0xff);
				break;	//00 0P 10
				
				case 4: //AC
				TIM_SetCompare1(TIM4,TIM_Period_set*duty);
				TIM_SetCompare2(TIM4,0);
				TIM_SetCompare3(TIM4,0);
				GPIO_WriteBit(GPIOC, GPIO_Pin_0, 0xff);
				GPIO_WriteBit(GPIOC, GPIO_Pin_1, 0xff);
				GPIO_WriteBit(GPIOC, GPIO_Pin_2, 0);
				break;	//10 0P 00
				
				case 6: //BC
				TIM_SetCompare1(TIM4,0);
				TIM_SetCompare2(TIM4,TIM_Period_set*duty);
				TIM_SetCompare3(TIM4,0);
				GPIO_WriteBit(GPIOC, GPIO_Pin_0, 0xff);
				GPIO_WriteBit(GPIOC, GPIO_Pin_1, 0xff);
				GPIO_WriteBit(GPIOC, GPIO_Pin_2, 0);
				break;	//10 00 0P
				
				case 2: //BA
				TIM_SetCompare1(TIM4,0);
				TIM_SetCompare2(TIM4,TIM_Period_set*duty);
				TIM_SetCompare3(TIM4,0);
				GPIO_WriteBit(GPIOC, GPIO_Pin_0, 0);
				GPIO_WriteBit(GPIOC, GPIO_Pin_1, 0xff);
				GPIO_WriteBit(GPIOC, GPIO_Pin_2, 0xff);
				break;	//00 10 0P
		}
}
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
//------------------------------------------------------------------------------------
void TIM_Config(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  /* TIM4 clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

  /* GPIOD clock enable */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
  
  /* GPIOD Configuration: TIM4 CH1 (PD12), TIM4 CH2 (PD13), TIM4 CH3 (PD14) and TIM4 CH4 (PD15) */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init(GPIOD, &GPIO_InitStructure); 

  /* Connect TIM4 pins to AF2 */  
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource12, GPIO_AF_TIM4);
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource13, GPIO_AF_TIM4); 
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource14, GPIO_AF_TIM4);
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource15, GPIO_AF_TIM4); 

	
}
void PWM_Config(void)
{
	  /* -----------------------------------------------------------------------
    TIM4 Configuration: generate 4 PWM signals with 4 different duty cycles.
    
    In this example TIM4 input clock (TIM4CLK) is set to 4 * APB1 clock (PCLK1), 
    since TIMPRE bit from RCC_DCKCFGR register is set.   
      TIM4CLK = 4 * PCLK1
      PCLK1 = HCLK / 4 
      => TIM4CLK = HCLK = SystemCoreClock
          
    To get TIM4 counter clock at 21 MHz, the prescaler is computed as follows:
       Prescaler = (TIM4CLK / TIM4 counter clock) - 1
       Prescaler = (SystemCoreClock /21 MHz) - 1
                                              
    To get TIM4 output clock at 30 KHz, the period (ARR)) is computed as follows:
       ARR = (TIM4 counter clock / TIM4 output clock) - 1
           = 699
                  
    TIM4 Channel1 duty cycle = (TIM4_CCR1/ TIM4_ARR)* 100 = 50%
    TIM4 Channel2 duty cycle = (TIM4_CCR2/ TIM4_ARR)* 100 = 37.5%
    TIM4 Channel3 duty cycle = (TIM4_CCR3/ TIM4_ARR)* 100 = 25%
    TIM4 Channel4 duty cycle = (TIM4_CCR4/ TIM4_ARR)* 100 = 12.5%

    Note: 
     SystemCoreClock variable holds HCLK frequency and is defined in system_stm32f4xx.c file.
     Each time the core clock (HCLK) changes, user had to call SystemCoreClockUpdate()
     function to update SystemCoreClock variable value. Otherwise, any configuration
     based on this variable will be incorrect.    
  ----------------------------------------------------------------------- */   
	NVIC_InitTypeDef   NVIC_InitStructure;
  RCC_TIMCLKPresConfig(RCC_TIMPrescActivated);

  /* Compute the prescaler value */
  PrescalerValue = (uint16_t) (SystemCoreClock / 21000000) - 1;

  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period = TIM_Period_set;
  TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

  /* PWM1 Mode configuration: Channel1 */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = CCR1_Val;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

  TIM_OC1Init(TIM4, &TIM_OCInitStructure);

  TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);

  /* PWM1 Mode configuration: Channel2 */
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = CCR2_Val;

  TIM_OC2Init(TIM4, &TIM_OCInitStructure);

  TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);

  /* PWM1 Mode configuration: Channel3 */
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = CCR3_Val;

  TIM_OC3Init(TIM4, &TIM_OCInitStructure);

  TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);

  /* PWM1 Mode configuration: Channel4 */
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = CCR4_Val;

  TIM_OC4Init(TIM4, &TIM_OCInitStructure);

  TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);

  TIM_ARRPreloadConfig(TIM4, ENABLE);
	
	TIM_ITConfig(TIM4 , TIM_IT_Update, ENABLE);
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_Init(&NVIC_InitStructure);
  /* TIM4 enable counter */
  TIM_Cmd(TIM4, ENABLE);
}

void GPIO_PA5(void)
{
		GPIO_InitTypeDef GPIO_InitStruct;
	
	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); 
		
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_5;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOA, &GPIO_InitStruct); 

}

void TIM4_IRQHandler(void)
{			
	if(TIM_GetITStatus(TIM4,TIM_IT_Update) != RESET) 
	{
		TIM_ClearITPendingBit(TIM4,TIM_FLAG_Update);
		if(timer_count>=timer_count_set)
		{
			hall_count++;
			//PWM120Degreedutyset(hall_circle[hall_count],duty);
			if(hall_count>=5)
			{
				hall_count=0;
			}
			timer_count=0;
		}
		
		//GPIO_ToggleBits(GPIOD , GPIO_Pin_1);
	}
		

}
//------------------------------------------------------------------------------------
void Hall_int(int data)
{
		interruptCount++;
		timer_count++;	
		hall = GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_0)*4;
		hall += GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_1)*2;
		hall += GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_2)*1;
  	PWM120Degreedutyset(hall,duty);
}
//------------------------------------------------------------------------------------
static void EXTILine0_Config(void)
{
  EXTI_InitTypeDef   EXTI_InitStructure;
  GPIO_InitTypeDef   GPIO_InitStructure;
  NVIC_InitTypeDef   NVIC_InitStructure;

  // Enable GPIOB clock 
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
  // Enable SYSCFG clock 
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	// Configure PB0123 pin as input floating
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  // Connect EXTI Line0123 to PB 0123 pin 
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource0);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource1);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource2);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource3);

  // Configure EXTI Line0123 
  EXTI_InitStructure.EXTI_Line = EXTI_Line0|EXTI_Line1|EXTI_Line2|EXTI_Line3;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;  
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);

  //Enable and set EXTI Line0123 Interrupt to the lowest priority 
  NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;
	NVIC_Init(&NVIC_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQn;
	NVIC_Init(&NVIC_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;
	NVIC_Init(&NVIC_InitStructure);
//--------------------------------------------------------------------------------
//PC 0123456
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
	GPIO_Write(GPIOC, 0xff);
	
//--------------------------------------------------------------------------------
//PD 0123
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
}
//------------------------------------------------------------------------------------

void Delay(__IO uint32_t nTime)
{
  TimingDelay = nTime;

  while(TimingDelay != 0);
}

//------------------------------------------------------------------------------------
void TimingDelay_Decrement(void)
{
  if (TimingDelay != 0x00)
  { 
    TimingDelay--;
  }
}
//------------------------------------------------------------------------------------
#ifdef  USE_FULL_ASSERT

void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif
//------------------------------------------------------------------------------------
