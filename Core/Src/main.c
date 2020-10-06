/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "comp.h"
#include "dac.h"
#include "dma.h"
#include "hrtim.h"
#include "iwdg.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */

  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

  /* System interrupt init*/
  /* SysTick_IRQn interrupt configuration */
  NVIC_SetPriority(SysTick_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),15, 0));

  /* Peripheral interrupt init*/

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_HRTIM1_Init();
  MX_USART1_UART_Init();
  MX_IWDG_Init();
  MX_TIM15_Init();
  MX_TIM16_Init();
  MX_TIM17_Init();
  MX_DAC1_Init();
  MX_DAC2_Init();
  MX_COMP2_Init();
  MX_COMP4_Init();
  MX_TIM1_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */

  /*******************************************************************/
   //Disable Systick
   /*******************************************************************/

 	  LL_SYSTICK_DisableIT();

   /*******************************************************************/
   //Enable Buzzer
   /*******************************************************************/

 	  LL_TIM_CC_EnableChannel(TIM15, LL_TIM_CHANNEL_CH1);
 	  LL_TIM_EnableCounter(TIM15);
 	  LL_TIM_OC_SetCompareCH1(TIM15,0);
 	  LL_TIM_EnableAllOutputs(TIM15);

   /*******************************************************************/
   //Enable IGLA
   /*******************************************************************/

 	  LL_TIM_CC_EnableChannel(TIM17, LL_TIM_CHANNEL_CH1);
 	  LL_TIM_EnableCounter(TIM17);
 	  LL_TIM_OC_SetCompareCH1(TIM17,4687); //50%
 	  LL_TIM_EnableAllOutputs(TIM17);

   /*******************************************************************/
   //Enable IGLA
   /*******************************************************************/

   LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH3);
   LL_TIM_EnableCounter(TIM1);
   LL_TIM_OC_SetCompareCH1(TIM1,0); //off
   LL_TIM_EnableAllOutputs(TIM1);

   /*******************************************************************/
   //Enable Cooler
   /*******************************************************************/

 	  LL_TIM_CC_EnableChannel(TIM16, LL_TIM_CHANNEL_CH1);
 	  LL_TIM_EnableCounter(TIM16);
 	  LL_TIM_OC_SetCompareCH1(TIM16,7200); //50%
 	  LL_TIM_EnableAllOutputs(TIM16);


   /*******************************************************************/
   //Enable DACs
   /*******************************************************************/

 	  LL_DMA_ConfigAddresses(DMA1,
 	                         LL_DMA_CHANNEL_4,
 	                         (uint32_t)&SlopeV,
 	                         LL_DAC_DMA_GetRegAddr(DAC1, LL_DAC_CHANNEL_2, LL_DAC_DMA_REG_DATA_12BITS_RIGHT_ALIGNED),
 	                         LL_DMA_DIRECTION_MEMORY_TO_PERIPH);

 	  LL_DMA_SetDataLength(DMA1,
 	                       LL_DMA_CHANNEL_4,
 						   SLOPE_RESOLUTION);

 	  LL_DMA_EnableChannel(DMA1,
 	                       LL_DMA_CHANNEL_4);

 	  LL_DAC_EnableDMAReq(DAC1, LL_DAC_CHANNEL_2);

 	  LL_DAC_Enable(DAC1, LL_DAC_CHANNEL_2);

 	  LL_DAC_EnableTrigger(DAC1, LL_DAC_CHANNEL_2);


   /*******************************************************************/
   //Enable COMP2 and COMP4
   /*******************************************************************/

 	  LL_COMP_Enable(COMP2);
 	  LL_COMP_Enable(COMP4);


   /*******************************************************************/
   //Unblock Driver
   /*******************************************************************/

   LL_GPIO_ResetOutputPin(drvONOFF_GPIO_Port, drvONOFF_Pin);


   /*******************************************************************/
   //Sky Init
   /*******************************************************************/

   ResetSlope();
   Start_Sky_IT();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
   while (1)
   {
	   asm("NOP");
	   LL_IWDG_ReloadCounter(IWDG);

 	  if (READ_BIT(HRTIM1->sCommonRegs.ISR, HRTIM_ISR_FLT1) == (HRTIM_ISR_FLT1)) {
 		  SET_BIT(HRTIM1->sCommonRegs.ICR, HRTIM_ICR_FLT1C);
 	  } else {
 		  if (FLT_Flag==true) {
 			  FLT_Flag=false;

 			  //RefV=Sinus_V[0];
 			  //RefI=Sinus_I[0];

 			  HRTIM1->sTimerxRegs[0].CMP4xR=4608;
 			  HRTIM1->sTimerxRegs[1].CMP4xR=4608;

 			  RefV=1500;
 			  RefI=1500;

 			  Duty.Index=0;

 			  PerformDACs();

 			  Start_Sky();
 		  }
 	  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
 	   }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_2);
  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_2)
  {
  }
  LL_RCC_HSE_Enable();

   /* Wait till HSE is ready */
  while(LL_RCC_HSE_IsReady() != 1)
  {

  }
  LL_RCC_LSI_Enable();

   /* Wait till LSI is ready */
  while(LL_RCC_LSI_IsReady() != 1)
  {

  }
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE_DIV_1, LL_RCC_PLL_MUL_6);
  LL_RCC_PLL_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {

  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {

  }
  LL_Init1msTick(72000000);
  LL_SetSystemCoreClock(72000000);
  LL_RCC_SetUSARTClockSource(LL_RCC_USART1_CLKSOURCE_SYSCLK);
  LL_RCC_SetTIMClockSource(LL_RCC_TIM1_CLKSOURCE_PLL);
  LL_RCC_SetHRTIMClockSource(RCC_CFGR3_HRTIM1SW_PLL);
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* HRTIM1_Master_IRQn interrupt configuration */
  NVIC_SetPriority(HRTIM1_Master_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),3, 0));
  NVIC_EnableIRQ(HRTIM1_Master_IRQn);
  /* HRTIM1_FLT_IRQn interrupt configuration */
  NVIC_SetPriority(HRTIM1_FLT_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),6, 0));
  NVIC_EnableIRQ(HRTIM1_FLT_IRQn);
  /* DMA1_Channel4_IRQn interrupt configuration */
  NVIC_SetPriority(DMA1_Channel4_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),4, 0));
  NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  NVIC_SetPriority(DMA1_Channel5_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),4, 0));
  NVIC_EnableIRQ(DMA1_Channel5_IRQn);
  /* USART1_IRQn interrupt configuration */
  NVIC_SetPriority(USART1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),14, 0));
  NVIC_EnableIRQ(USART1_IRQn);
  /* PVD_IRQn interrupt configuration */
  NVIC_SetPriority(PVD_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),13, 0));
  NVIC_EnableIRQ(PVD_IRQn);
  /* HRTIM1_TIMB_IRQn interrupt configuration */
  NVIC_SetPriority(HRTIM1_TIMB_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),2, 0));
  NVIC_EnableIRQ(HRTIM1_TIMB_IRQn);
  /* HRTIM1_TIMD_IRQn interrupt configuration */
  NVIC_SetPriority(HRTIM1_TIMD_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),2, 0));
  NVIC_EnableIRQ(HRTIM1_TIMD_IRQn);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
