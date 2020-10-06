/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f3xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32f3xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

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
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */

  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F3xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f3xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles PVD interrupt through EXTI line 16.
  */
void PVD_IRQHandler(void)
{
  /* USER CODE BEGIN PVD_IRQn 0 */

  /* USER CODE END PVD_IRQn 0 */

  /* USER CODE BEGIN PVD_IRQn 1 */

  /* USER CODE END PVD_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel4 global interrupt.
  */
void DMA1_Channel4_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel4_IRQn 0 */

  /* USER CODE END DMA1_Channel4_IRQn 0 */

  /* USER CODE BEGIN DMA1_Channel4_IRQn 1 */

  /* USER CODE END DMA1_Channel4_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel5 global interrupt.
  */
void DMA1_Channel5_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel5_IRQn 0 */

  /* USER CODE END DMA1_Channel5_IRQn 0 */

  /* USER CODE BEGIN DMA1_Channel5_IRQn 1 */

  /* USER CODE END DMA1_Channel5_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt / USART1 wake-up interrupt through EXT line 25.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */

  /* USER CODE END USART1_IRQn 0 */
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}

/**
  * @brief This function handles HRTIM master timer global interrupt.
  */
void HRTIM1_Master_IRQHandler(void)
{
  /* USER CODE BEGIN HRTIM1_Master_IRQn 0 */
	//if (READ_BIT(HRTIM1->sMasterRegs.MISR, HRTIM_MISR_MREP) == (HRTIM_MISR_MREP)) {
		SET_BIT(HRTIM1->sMasterRegs.MICR, HRTIM_MICR_MREP);

		// LED FLAG
		WRITE_REG(ledV_GPIO_Port->BSRR, ledV_Pin);

		if (Duty.Index50Hz>=MAX_50HZ_INDEX-1) {
			Duty.Index50Hz=0;
		} else {
			Duty.Index50Hz++;
		}

		HRTIM1->sMasterRegs.MREP=RepData[Duty.Index50Hz];

		if ((READ_BIT(HRTIM1->sCommonRegs.ISR, HRTIM_ISR_FLT1) == (HRTIM_ISR_FLT1))
				|| (FLT_Flag==true) || (Duty.Index50Hz==2) || (Duty.Index50Hz==5)) {
			TIM1->CCR1=0; TIM1->CCR3=0;

			// LED FLAG
			WRITE_REG(ledV_GPIO_Port->BRR, ledV_Pin);
			return;
		} else {
			if ((Duty.Index50Hz==0) || (Duty.Index50Hz==1)) {
				TIM1->CCR1=0; TIM1->CCR3=2879;

				// LED FLAG
				WRITE_REG(ledV_GPIO_Port->BRR, ledV_Pin);
				return;
			}
			if ((Duty.Index50Hz==3) || (Duty.Index50Hz==4)) {
				//TIM1->CCR3=0; TIM1->CCR1=2879;

				// LED FLAG
				WRITE_REG(ledV_GPIO_Port->BRR, ledV_Pin);
				return;
			}

			// LED FLAG
			WRITE_REG(ledV_GPIO_Port->BRR, ledV_Pin);
		}
	//}
  /* USER CODE END HRTIM1_Master_IRQn 0 */

  /* USER CODE BEGIN HRTIM1_Master_IRQn 1 */
  /* USER CODE END HRTIM1_Master_IRQn 1 */
}

/**
  * @brief This function handles HRTIM timer B global interrupt.
  */
void HRTIM1_TIMB_IRQHandler(void)
{
  /* USER CODE BEGIN HRTIM1_TIMB_IRQn 0 */
			SET_BIT(HRTIM1->sTimerxRegs[1].TIMxICR, HRTIM_MICR_MCMP4);

			// LED FLAG
			WRITE_REG(ledI_GPIO_Port->BSRR, ledI_Pin);

			// set pulls #2 by close #1
			//uint16_t temp_duty_1=HRTIM1->sTimerxRegs[3].CPT1xR;

			/*if ((temp_duty_1<4608) || (temp_duty_1>23040)) {
				temp_duty_1=23040;
			}*/

			//HRTIM1->sTimerxRegs[0].CMP4xR=temp_duty_1;

			if (Duty.Index>=MAX_INDEX) {
				Duty.Index=0;

				   if (Duty.Softstart) {
						Duty.MaxDuty=Duty.MaxDuty+softstart_step;

						if (Duty.MaxDuty>=23040) {
							Duty.MaxDuty=23040;
							Duty.Softstart=false;
						}
					}
				   HRTIM1->sTimerxRegs[0].CMP4xR=Duty.MaxDuty;
				   HRTIM1->sTimerxRegs[1].CMP4xR=Duty.MaxDuty;
			}


			//RefV=Sinus_V[Duty.Index];
			//RefI=Sinus_I[Duty.Index];

			RefV=1500;
			RefI=1500;

			PerformDACs();

			Duty.Index++;

			// LED FLAG
			WRITE_REG(ledI_GPIO_Port->BRR, ledI_Pin);
  /* USER CODE END HRTIM1_TIMB_IRQn 0 */

  /* USER CODE BEGIN HRTIM1_TIMB_IRQn 1 */

  /* USER CODE END HRTIM1_TIMB_IRQn 1 */
}

/**
  * @brief This function handles HRTIM timer D global interrupt.
  */
void HRTIM1_TIMD_IRQHandler(void)
{
  /* USER CODE BEGIN HRTIM1_TIMD_IRQn 0 */
	SET_BIT(HRTIM1->sTimerxRegs[3].TIMxICR, HRTIM_MICR_MREP);
  /* USER CODE END HRTIM1_TIMD_IRQn 0 */

  /* USER CODE BEGIN HRTIM1_TIMD_IRQn 1 */
	//uint16_t temp_duty_1=HRTIM1->sTimerxRegs[4].CPT1xR;
	//HRTIM1->sTimerxRegs[1].CMP4xR=temp_duty_1;

  /* USER CODE END HRTIM1_TIMD_IRQn 1 */
}

/**
  * @brief This function handles HRTIM fault global interrupt.
  */
void HRTIM1_FLT_IRQHandler(void)
{
  /* USER CODE BEGIN HRTIM1_FLT_IRQn 0 */
	SET_BIT(HRTIM1->sCommonRegs.ICR, HRTIM_ICR_FLT1C);
  /* USER CODE END HRTIM1_FLT_IRQn 0 */

  /* USER CODE BEGIN HRTIM1_FLT_IRQn 1 */
	Reset_Sky();
  /* USER CODE END HRTIM1_FLT_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
