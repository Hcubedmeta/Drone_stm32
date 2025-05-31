/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_it.h"
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
extern volatile int CH1,CH2,CH3,CH4,CH5,CH6;
volatile int first_CH1,first_CH2,first_CH3,first_CH4,first_CH5,first_CH6;
volatile int last_CH1,last_CH2,last_CH3,last_CH4,last_CH5,last_CH6;
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern UART_HandleTypeDef huart1;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M3 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
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
  * @brief This function handles Prefetch fault, memory access fault.
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
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles TIM1 break interrupt.
  */
void TIM1_BRK_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_BRK_IRQn 0 */

  /* USER CODE END TIM1_BRK_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_BRK_IRQn 1 */

  /* USER CODE END TIM1_BRK_IRQn 1 */
}

/**
  * @brief This function handles TIM1 update interrupt.
  */
void TIM1_UP_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_UP_IRQn 0 */

  /* USER CODE END TIM1_UP_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_UP_IRQn 1 */

  /* USER CODE END TIM1_UP_IRQn 1 */
}

/**
  * @brief This function handles TIM1 trigger and commutation interrupts.
  */
void TIM1_TRG_COM_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_TRG_COM_IRQn 0 */

  /* USER CODE END TIM1_TRG_COM_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_TRG_COM_IRQn 1 */

  /* USER CODE END TIM1_TRG_COM_IRQn 1 */
}

/**
  * @brief This function handles TIM1 capture compare interrupt.
  */
void TIM1_CC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_CC_IRQn 0 */

  /* USER CODE END TIM1_CC_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_CC_IRQn 1 */

  /* USER CODE END TIM1_CC_IRQn 1 */
}

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */
        
        if (GPIOA->IDR & (1 << 15)) // Ki?m tra m?c tín hi?u PPM
		{
				TIM2->SR &= ~TIM_SR_CC1IF;
				first_CH1 = TIM2->CCR1;  // Luu giá tr? th?i di?m b?t d?u
				TIM2->CCER |= TIM_CCER_CC1P;  // Ð?i c?nh xu?ng d? b?t th?i di?m k?t thúc
		}
		else
		{
				TIM2->SR &= ~TIM_SR_CC1IF;
				uint32_t temp_CH1 = (TIM2->CCR1 >= first_CH1) ? 
														(TIM2->CCR1 - first_CH1) : 
														(0xFFFF - first_CH1 + TIM2->CCR1);
				CH1 = temp_CH1;  // Luu d? r?ng xung
				TIM2->CCER &= ~TIM_CCER_CC1P;  // Ð?i l?i c?nh lên
		}

		if (GPIOB->IDR & (1 << 3)) 
		{
				TIM2->SR &= ~TIM_SR_CC2IF;
				first_CH2 = TIM2->CCR2;
				TIM2->CCER |= TIM_CCER_CC2P;
		}
		else
		{
				TIM2->SR &= ~TIM_SR_CC2IF;
				uint32_t temp_CH2 = (TIM2->CCR2 >= first_CH2) ? 
														(TIM2->CCR2 - first_CH2) : 
														(0xFFFF - first_CH2 + TIM2->CCR2);
				CH2 = temp_CH2;
				TIM2->CCER &= ~TIM_CCER_CC2P;
		}

  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}

/**
  * @brief This function handles TIM3 global interrupt.
  */
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */
        
        if (TIM3->SR & TIM_SR_CC1IF) {
        TIM3->SR &= ~TIM_SR_CC1IF;  // Xóa c? ng?t
        uint32_t capture = TIM3->CCR1;

        if (!(TIM3->CCER & TIM_CCER_CC1P)) {  // N?u dang ? c?nh lên
            first_CH3 = capture;
            TIM3->CCER |= TIM_CCER_CC1P;  // Ð?i sang c?nh xu?ng
        } else {  // N?u dang ? c?nh xu?ng
            CH3 = (capture >= first_CH3) ? (capture - first_CH3) : (0xFFFF - first_CH3 + capture);
            TIM3->CCER &= ~TIM_CCER_CC1P;  // Ð?i l?i v? c?nh lên
        }
    }

    // X? lý kênh 2 (CH4)
    if (TIM3->SR & TIM_SR_CC2IF) {
        TIM3->SR &= ~TIM_SR_CC2IF;  // Xóa c? ng?t
        uint32_t capture = TIM3->CCR2;

        if (!(TIM3->CCER & TIM_CCER_CC2P)) {
            first_CH4 = capture;
            TIM3->CCER |= TIM_CCER_CC2P;
        } else {
            CH4 = (capture >= first_CH4) ? (capture - first_CH4) : (0xFFFF - first_CH4 + capture);
            TIM3->CCER &= ~TIM_CCER_CC2P;
        }
    }

    // X? lý kênh 3 (CH3)
    if (TIM3->SR & TIM_SR_CC3IF) {
        TIM3->SR &= ~TIM_SR_CC3IF;
        uint32_t capture = TIM3->CCR3;

        if (!(TIM3->CCER & TIM_CCER_CC3P)) {
            first_CH5 = capture;
            TIM3->CCER |= TIM_CCER_CC3P;
        } else {
            CH5 = (capture >= first_CH5) ? (capture - first_CH5) : (0xFFFF - first_CH5 + capture);
            TIM3->CCER &= ~TIM_CCER_CC3P;
        }
    }

    // X? lý kênh 4 (CH2)
    if (TIM3->SR & TIM_SR_CC4IF) {
        TIM3->SR &= ~TIM_SR_CC4IF;
        uint32_t capture = TIM3->CCR4;

        if (!(TIM3->CCER & TIM_CCER_CC4P)) {
            first_CH6 = capture;
            TIM3->CCER |= TIM_CCER_CC4P;
        } else {
            CH6 = (capture >= first_CH6) ? (capture - first_CH6) : (0xFFFF - first_CH6 + capture);
            TIM3->CCER &= ~TIM_CCER_CC4P;
        }
    }
  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */

  /* USER CODE END TIM3_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */

  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
