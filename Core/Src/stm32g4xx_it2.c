/**
  ******************************************************************************
  * @file    STM32G474E-EVAL/Src/stm32g4xx_it2.c
  * @author  MCD Application Team
  * @version v2.0.0
  * @date    16-May-2019
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2018 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32g4xx_it.h"

/** @addtogroup DualBank
  * @{
  */
extern int const __ICFEDIT_stack_block_start__;

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
void (* const _vectab[NVIC_VT_COUNT])(void) =
{
  (void (*)(void))0x20013C00U,
  Reset_Handler,
  NMI_Handler2,
  HardFault_Handler2,
  MemManage_Handler2,
  BusFault_Handler2,
  UsageFault_Handler2,
  0,
  0,
  0,
  0,
  SVC_Handler2,
  DebugMon_Handler2,
  0,
  PendSV_Handler2,
  SysTick_Handler2,
  WWDG_IRQHandler2,
  PVD_PVM_IRQHandler2,
  TAMP_STAMP_IRQHandler2,
  RTC_WKUP_IRQHandler2,
  FLASH_IRQHandler2,
  RCC_IRQHandler2,
  EXTI0_IRQHandler2,
  EXTI1_IRQHandler2,
  EXTI2_IRQHandler2,
  EXTI3_IRQHandler2,
  EXTI4_IRQHandler2,
  DMA1_Channel1_IRQHandler2,
  DMA1_Channel2_IRQHandler2,
  DMA1_Channel3_IRQHandler2,
  DMA1_Channel4_IRQHandler2,
  DMA1_Channel5_IRQHandler2,
  DMA1_Channel6_IRQHandler2,
  DMA1_Channel7_IRQHandler2,
  ADC1_2_IRQHandler2,
  USB_HP_IRQHandler2,
  USB_LP_IRQHandler2,
  FDCAN1_IT0_IRQHandler2,
  FDCAN1_IT1_IRQHandler2,
  EXTI9_5_IRQHandler2,
  TIM1_BRK_TIM15_IRQHandler2,
  TIM1_UP_TIM16_IRQHandler2,
  TIM1_TRG_COM_TIM17_IRQHandler2,
  TIM1_CC_IRQHandler2,
  TIM2_IRQHandler2,
  TIM3_IRQHandler2,
  TIM4_IRQHandler2,
  I2C1_EV_IRQHandler2,
  I2C1_ER_IRQHandler2,
  I2C2_EV_IRQHandler2,
  I2C2_ER_IRQHandler2,
  SPI1_IRQHandler2,
  SPI2_IRQHandler2,
  USART1_IRQHandler2,
  USART2_IRQHandler2,
  USART3_IRQHandler2,
  EXTI15_10_IRQHandler2,
  RTC_Alarm_IRQHandler2,
  USBWakeUp_IRQHandler2,
  TIM8_BRK_IRQHandler2,
  TIM8_UP_IRQHandler2,
  TIM8_TRG_COM_IRQHandler2,
  TIM8_CC_IRQHandler2,
  ADC3_IRQHandler2,
  FMC_IRQHandler2,
  LPTIM1_IRQHandler2,
  TIM5_IRQHandler2,
  SPI3_IRQHandler2,
  UART4_IRQHandler2,
  UART5_IRQHandler2,
  TIM6_DAC_IRQHandler2,
  TIM7_DAC_IRQHandler2,
  DMA2_Channel1_IRQHandler2,
  DMA2_Channel2_IRQHandler2,
  DMA2_Channel3_IRQHandler2,
  DMA2_Channel4_IRQHandler2,
  DMA2_Channel5_IRQHandler2,
  ADC4_IRQHandler2,
  ADC5_IRQHandler2,
  USBPD_IRQHandler2,
  COMP1_2_3_IRQHandler2,
  COMP4_5_6_IRQHandler2,
  COMP7_IRQHandler2,
  HRTIM1_Master_IRQHandler2,
  HRTIM1_TIMA_IRQHandler2,
  HRTIM1_TIMB_IRQHandler2,
  HRTIM1_TIMC_IRQHandler2,
  HRTIM1_TIMD_IRQHandler2,
  HRTIM1_TIME_IRQHandler2,
  HRTIM1_FLT_IRQHandler2,
  HRTIM1_TIMF_IRQHandler2,
  CRS_IRQHandler2,
  SAI1_IRQHandler2,
  TIM20_BRK_IRQHandler2,
  TIM20_UP_IRQHandler2,
  TIM20_TRG_COM_IRQHandler2,
  TIM20_CC_IRQHandler2,
  FPU_IRQHandler2,
  I2C4_EV_IRQHandler2,
  I2C4_ER_IRQHandler2,
  SPI4_IRQHandler2,
  0,
  FDCAN2_IT0_IRQHandler2,
  FDCAN2_IT1_IRQHandler2,
  FDCAN3_IT0_IRQHandler2,
  FDCAN3_IT1_IRQHandler2,
  RNG_IRQHandler2,
  LPUART1_IRQHandler2,
  I2C3_EV_IRQHandler2,
  I2C3_ER_IRQHandler2,
  DMAMUX_OVR_IRQHandler2,
  QUADSPI_IRQHandler2,
  DMA1_Channel8_IRQHandler2,
  DMA2_Channel6_IRQHandler2,
  DMA2_Channel7_IRQHandler2,
  DMA2_Channel8_IRQHandler2,
  CORDIC_IRQHandler2,
  FMAC_IRQHandler2
};


/* UART handler declared in "main.c" file */
extern UART_HandleTypeDef UartHandle;
extern __IO uint32_t uwTick;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief   This function handles NMI exception.
  * There is a possibility that NMI will be called during the transition phase
  * as it cannot be really disabled. To prevent system crash, make sure NMI
  * handler is not modified in any patch code.
  * @param  None
  * @retval None
  */
#pragma location="ccmram2"
#pragma location="ccmram2_init"
__RAM_FUNCTION void NMI_Handler2(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
#pragma location="ccmram2"
#pragma location="ccmram2_init"
__RAM_FUNCTION void HardFault_Handler2(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
#pragma location="ccmram2"
#pragma location="ccmram2_init"
__RAM_FUNCTION void MemManage_Handler2(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
#pragma location="ccmram2"
#pragma location="ccmram2_init"
__RAM_FUNCTION void BusFault_Handler2(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
#pragma location="ccmram2"
#pragma location="ccmram2_init"
__RAM_FUNCTION void UsageFault_Handler2(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
#pragma location="ccmram2"
#pragma location="ccmram2_init"
__RAM_FUNCTION void SVC_Handler2(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
#pragma location="ccmram2"
#pragma location="ccmram2_init"
__RAM_FUNCTION void DebugMon_Handler2(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
#pragma location="ccmram2"
#pragma location="ccmram2_init"
__RAM_FUNCTION void PendSV_Handler2(void)
{
}

/**
  * @brief  This function handles SysTick Handler2.
  * @param  None
  * @retval None
  */
#pragma location="ccmram2"
#pragma location="ccmram2_init"
__RAM_FUNCTION void SysTick_Handler2(void)
{
  uwTick++;
}

#pragma location="ccmram2"
#pragma location="ccmram2_init"
__RAM_FUNCTION void WWDG_IRQHandler2(void)
{
}
#pragma location="ccmram2"
#pragma location="ccmram2_init"
__RAM_FUNCTION void PVD_PVM_IRQHandler2(void)
{
}
#pragma location="ccmram2"
#pragma location="ccmram2_init"
__RAM_FUNCTION void TAMP_STAMP_IRQHandler2(void)
{
}
#pragma location="ccmram2"
#pragma location="ccmram2_init"
__RAM_FUNCTION void RTC_WKUP_IRQHandler2(void)
{
}
#pragma location="ccmram2"
#pragma location="ccmram2_init"
__RAM_FUNCTION void FLASH_IRQHandler2(void)
{
}
#pragma location="ccmram2"
#pragma location="ccmram2_init"
__RAM_FUNCTION void RCC_IRQHandler2(void)
{
}
#pragma location="ccmram2"
#pragma location="ccmram2_init"
__RAM_FUNCTION void EXTI0_IRQHandler2(void)
{
}
#pragma location="ccmram2"
#pragma location="ccmram2_init"
__RAM_FUNCTION void EXTI1_IRQHandler2(void)
{
}
#pragma location="ccmram2"
#pragma location="ccmram2_init"
__RAM_FUNCTION void EXTI2_IRQHandler2(void)
{
}
#pragma location="ccmram2"
#pragma location="ccmram2_init"
__RAM_FUNCTION void EXTI3_IRQHandler2(void)
{
}
#pragma location="ccmram2"
#pragma location="ccmram2_init"
__RAM_FUNCTION void EXTI4_IRQHandler2(void)
{
}
#pragma location="ccmram2"
#pragma location="ccmram2_init"
__RAM_FUNCTION void DMA1_Channel1_IRQHandler2(void)
{
}
#pragma location="ccmram2"
#pragma location="ccmram2_init"
__RAM_FUNCTION void DMA1_Channel2_IRQHandler2(void)
{
}
#pragma location="ccmram2"
#pragma location="ccmram2_init"
__RAM_FUNCTION void DMA1_Channel3_IRQHandler2(void)
{
}
#pragma location="ccmram2"
#pragma location="ccmram2_init"
__RAM_FUNCTION void DMA1_Channel4_IRQHandler2(void)
{
}
#pragma location="ccmram2"
#pragma location="ccmram2_init"
__RAM_FUNCTION void DMA1_Channel5_IRQHandler2(void)
{
}
#pragma location="ccmram2"
#pragma location="ccmram2_init"
__RAM_FUNCTION void DMA1_Channel6_IRQHandler2(void)
{
}
#pragma location="ccmram2"
#pragma location="ccmram2_init"
__RAM_FUNCTION void DMA1_Channel7_IRQHandler2(void)
{
}
#pragma location="ccmram2"
#pragma location="ccmram2_init"
__RAM_FUNCTION void ADC1_2_IRQHandler2(void)
{
}
#pragma location="ccmram2"
#pragma location="ccmram2_init"
__RAM_FUNCTION void USB_HP_IRQHandler2(void)
{
}
#pragma location="ccmram2"
#pragma location="ccmram2_init"
__RAM_FUNCTION void USB_LP_IRQHandler2(void)
{
}
#pragma location="ccmram2"
#pragma location="ccmram2_init"
__RAM_FUNCTION void FDCAN1_IT0_IRQHandler2(void)
{
}
#pragma location="ccmram2"
#pragma location="ccmram2_init"
__RAM_FUNCTION void FDCAN1_IT1_IRQHandler2(void)
{
}
#pragma location="ccmram2"
#pragma location="ccmram2_init"
__RAM_FUNCTION void EXTI9_5_IRQHandler2(void)
{
}
#pragma location="ccmram2"
#pragma location="ccmram2_init"
__RAM_FUNCTION void TIM1_BRK_TIM15_IRQHandler2(void)
{
}
#pragma location="ccmram2"
#pragma location="ccmram2_init"
__RAM_FUNCTION void TIM1_UP_TIM16_IRQHandler2(void)
{
}
#pragma location="ccmram2"
#pragma location="ccmram2_init"
__RAM_FUNCTION void TIM1_TRG_COM_TIM17_IRQHandler2(void)
{
}
#pragma location="ccmram2"
#pragma location="ccmram2_init"
__RAM_FUNCTION void TIM1_CC_IRQHandler2(void)
{
}
#pragma location="ccmram2"
#pragma location="ccmram2_init"
__RAM_FUNCTION void TIM2_IRQHandler2(void)
{
}
#pragma location="ccmram2"
#pragma location="ccmram2_init"
__RAM_FUNCTION void TIM3_IRQHandler2(void)
{
}
#pragma location="ccmram2"
#pragma location="ccmram2_init"
__RAM_FUNCTION void TIM4_IRQHandler2(void)
{
}
#pragma location="ccmram2"
#pragma location="ccmram2_init"
__RAM_FUNCTION void I2C1_EV_IRQHandler2(void)
{
}
#pragma location="ccmram2"
#pragma location="ccmram2_init"
__RAM_FUNCTION void I2C1_ER_IRQHandler2(void)
{
}
#pragma location="ccmram2"
#pragma location="ccmram2_init"
__RAM_FUNCTION void I2C2_EV_IRQHandler2(void)
{
}
#pragma location="ccmram2"
#pragma location="ccmram2_init"
__RAM_FUNCTION void I2C2_ER_IRQHandler2(void)
{
}
#pragma location="ccmram2"
#pragma location="ccmram2_init"
__RAM_FUNCTION void SPI1_IRQHandler2(void)
{
}
#pragma location="ccmram2"
#pragma location="ccmram2_init"
__RAM_FUNCTION void SPI2_IRQHandler2(void)
{
}
#pragma location="ccmram2"
#pragma location="ccmram2_init"
__RAM_FUNCTION void USART1_IRQHandler2(void)
{
}
#pragma location="ccmram2"
#pragma location="ccmram2_init"
__RAM_FUNCTION void USART2_IRQHandler2(void)
{
}
#pragma location="ccmram2"
#pragma location="ccmram2_init"
__RAM_FUNCTION void USART3_IRQHandler2(void)
{
}
/**
* @brief This function handles EXTI line[15:10] interrupts.
*/
#pragma location="ccmram2"
#pragma location="ccmram2_init"
__RAM_FUNCTION void EXTI15_10_IRQHandler2(void)
{
  /* EXTI line interrupt detected */
  if(__HAL_GPIO_EXTI_GET_IT(USER_BUTTON_PIN) != RESET)
  {
    __HAL_GPIO_EXTI_CLEAR_IT(USER_BUTTON_PIN);
    DEMO_LED_GPIO->ODR ^= DEMO_LED_PIN;
  }
}
#pragma location="ccmram2"
#pragma location="ccmram2_init"
__RAM_FUNCTION void RTC_Alarm_IRQHandler2(void)
{
}
#pragma location="ccmram2"
#pragma location="ccmram2_init"
__RAM_FUNCTION void USBWakeUp_IRQHandler2(void)
{
}
#pragma location="ccmram2"
#pragma location="ccmram2_init"
__RAM_FUNCTION void TIM8_BRK_IRQHandler2(void)
{
}
#pragma location="ccmram2"
#pragma location="ccmram2_init"
__RAM_FUNCTION void TIM8_UP_IRQHandler2(void)
{
}
#pragma location="ccmram2"
#pragma location="ccmram2_init"
__RAM_FUNCTION void TIM8_TRG_COM_IRQHandler2(void)
{
}
#pragma location="ccmram2"
#pragma location="ccmram2_init"
__RAM_FUNCTION void TIM8_CC_IRQHandler2(void)
{
}
#pragma location="ccmram2"
#pragma location="ccmram2_init"
__RAM_FUNCTION void ADC3_IRQHandler2(void)
{
}
#pragma location="ccmram2"
#pragma location="ccmram2_init"
__RAM_FUNCTION void FMC_IRQHandler2(void)
{
}
#pragma location="ccmram2"
#pragma location="ccmram2_init"
__RAM_FUNCTION void LPTIM1_IRQHandler2(void)
{
}
#pragma location="ccmram2"
#pragma location="ccmram2_init"
__RAM_FUNCTION void TIM5_IRQHandler2(void)
{
}
#pragma location="ccmram2"
#pragma location="ccmram2_init"
__RAM_FUNCTION void SPI3_IRQHandler2(void)
{
}
#pragma location="ccmram2"
#pragma location="ccmram2_init"
__RAM_FUNCTION void UART4_IRQHandler2(void)
{
}
#pragma location="ccmram2"
#pragma location="ccmram2_init"
__RAM_FUNCTION void UART5_IRQHandler2(void)
{
}
#pragma location="ccmram2"
#pragma location="ccmram2_init"
__RAM_FUNCTION void TIM6_DAC_IRQHandler2(void)
{
}
#pragma location="ccmram2"
#pragma location="ccmram2_init"
__RAM_FUNCTION void TIM7_DAC_IRQHandler2(void)
{
}
#pragma location="ccmram2"
#pragma location="ccmram2_init"
__RAM_FUNCTION void DMA2_Channel1_IRQHandler2(void)
{
}
#pragma location="ccmram2"
#pragma location="ccmram2_init"
__RAM_FUNCTION void DMA2_Channel2_IRQHandler2(void)
{
}
#pragma location="ccmram2"
#pragma location="ccmram2_init"
__RAM_FUNCTION void DMA2_Channel3_IRQHandler2(void)
{
}
#pragma location="ccmram2"
#pragma location="ccmram2_init"
__RAM_FUNCTION void DMA2_Channel4_IRQHandler2(void)
{
}
#pragma location="ccmram2"
#pragma location="ccmram2_init"
__RAM_FUNCTION void DMA2_Channel5_IRQHandler2(void)
{
}
#pragma location="ccmram2"
#pragma location="ccmram2_init"
__RAM_FUNCTION void ADC4_IRQHandler2(void)
{
}
#pragma location="ccmram2"
#pragma location="ccmram2_init"
__RAM_FUNCTION void ADC5_IRQHandler2(void)
{
}
#pragma location="ccmram2"
#pragma location="ccmram2_init"
__RAM_FUNCTION void USBPD_IRQHandler2(void)
{
}
#pragma location="ccmram2"
#pragma location="ccmram2_init"
__RAM_FUNCTION void COMP1_2_3_IRQHandler2(void)
{
}
#pragma location="ccmram2"
#pragma location="ccmram2_init"
__RAM_FUNCTION void COMP4_5_6_IRQHandler2(void)
{
}
#pragma location="ccmram2"
#pragma location="ccmram2_init"
__RAM_FUNCTION void COMP7_IRQHandler2(void)
{
}
#pragma location="ccmram2"
#pragma location="ccmram2_init"
__RAM_FUNCTION void HRTIM1_Master_IRQHandler2(void)
{
}
#pragma location="ccmram2"
#pragma location="ccmram2_init"
__RAM_FUNCTION void HRTIM1_TIMA_IRQHandler2(void)
{
}
#pragma location="ccmram2"
#pragma location="ccmram2_init"
__RAM_FUNCTION void HRTIM1_TIMB_IRQHandler2(void)
{
}
#pragma location="ccmram2"
#pragma location="ccmram2_init"
__RAM_FUNCTION void HRTIM1_TIMC_IRQHandler2(void)
{
}
#pragma location="ccmram2"
#pragma location="ccmram2_init"
__RAM_FUNCTION void HRTIM1_TIMD_IRQHandler2(void)
{
}

#pragma location="ccmram2"
#pragma location="ccmram2_init"
__RAM_FUNCTION void HRTIM1_TIME_IRQHandler2(void)
{
  static volatile uint32_t Tempo = 0;

  LED1_GPIO_PORT->BSRR = (uint32_t)LED1_PIN;

  HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_E].TIMxICR = HRTIM_TIM_FLAG_REP;

  if ((HRTIM1->sCommonRegs.ISR)& HRTIM_FLAG_BMPER)
  {
    HRTIM1->sCommonRegs.ICR = HRTIM_FLAG_BMPER;
    Tempo++;
  }

  if (Tempo > 0)
  {
    Tempo = 0;
    if ((DAC4->STR2) < (16 << 16) + 180)
    {
      (DAC4->STR2) += 2;
    }
    else
    {
      DAC4->STR2 = (16 << 16) + 35;
    }
  }

  EnableSwitchOver = 0;
  LED1_GPIO_PORT->BRR = (uint32_t)LED1_PIN;

}


#pragma location="ccmram2"
#pragma location="ccmram2_init"
__RAM_FUNCTION void HRTIM1_FLT_IRQHandler2(void)
{
}
#pragma location="ccmram2"
#pragma location="ccmram2_init"
__RAM_FUNCTION void HRTIM1_TIMF_IRQHandler2(void)
{
}
#pragma location="ccmram2"
#pragma location="ccmram2_init"
__RAM_FUNCTION void CRS_IRQHandler2(void)
{
}
#pragma location="ccmram2"
#pragma location="ccmram2_init"
__RAM_FUNCTION void SAI1_IRQHandler2(void)
{
}
#pragma location="ccmram2"
#pragma location="ccmram2_init"
__RAM_FUNCTION void TIM20_BRK_IRQHandler2(void)
{
}
#pragma location="ccmram2"
#pragma location="ccmram2_init"
__RAM_FUNCTION void TIM20_UP_IRQHandler2(void)
{
}
#pragma location="ccmram2"
#pragma location="ccmram2_init"
__RAM_FUNCTION void TIM20_TRG_COM_IRQHandler2(void)
{
}
#pragma location="ccmram2"
#pragma location="ccmram2_init"
__RAM_FUNCTION void TIM20_CC_IRQHandler2(void)
{
}
#pragma location="ccmram2"
#pragma location="ccmram2_init"
__RAM_FUNCTION void FPU_IRQHandler2(void)
{
}
#pragma location="ccmram2"
#pragma location="ccmram2_init"
__RAM_FUNCTION void I2C4_EV_IRQHandler2(void)
{
}
#pragma location="ccmram2"
#pragma location="ccmram2_init"
__RAM_FUNCTION void I2C4_ER_IRQHandler2(void)
{
}
#pragma location="ccmram2"
#pragma location="ccmram2_init"
__RAM_FUNCTION void SPI4_IRQHandler2(void)
{
}
#pragma location="ccmram2"
#pragma location="ccmram2_init"
__RAM_FUNCTION void FDCAN2_IT0_IRQHandler2(void)
{
}
#pragma location="ccmram2"
#pragma location="ccmram2_init"
__RAM_FUNCTION void FDCAN2_IT1_IRQHandler2(void)
{
}
#pragma location="ccmram2"
#pragma location="ccmram2_init"
__RAM_FUNCTION void FDCAN3_IT0_IRQHandler2(void)
{
}
#pragma location="ccmram2"
#pragma location="ccmram2_init"
__RAM_FUNCTION void FDCAN3_IT1_IRQHandler2(void)
{
}
#pragma location="ccmram2"
#pragma location="ccmram2_init"
__RAM_FUNCTION void RNG_IRQHandler2(void)
{
}
#pragma location="ccmram2"
#pragma location="ccmram2_init"
__RAM_FUNCTION void LPUART1_IRQHandler2(void)
{
}
#pragma location="ccmram2"
#pragma location="ccmram2_init"
__RAM_FUNCTION void I2C3_EV_IRQHandler2(void)
{
}
#pragma location="ccmram2"
#pragma location="ccmram2_init"
__RAM_FUNCTION void I2C3_ER_IRQHandler2(void)
{
}
#pragma location="ccmram2"
#pragma location="ccmram2_init"
__RAM_FUNCTION void DMAMUX_OVR_IRQHandler2(void)
{
}
#pragma location="ccmram2"
#pragma location="ccmram2_init"
__RAM_FUNCTION void QUADSPI_IRQHandler2(void)
{
}
#pragma location="ccmram2"
#pragma location="ccmram2_init"
__RAM_FUNCTION void DMA1_Channel8_IRQHandler2(void)
{
}
#pragma location="ccmram2"
#pragma location="ccmram2_init"
__RAM_FUNCTION void DMA2_Channel6_IRQHandler2(void)
{
}
#pragma location="ccmram2"
#pragma location="ccmram2_init"
__RAM_FUNCTION void DMA2_Channel7_IRQHandler2(void)
{
}
#pragma location="ccmram2"
#pragma location="ccmram2_init"
__RAM_FUNCTION void DMA2_Channel8_IRQHandler2(void)
{
}
#pragma location="ccmram2"
#pragma location="ccmram2_init"
__RAM_FUNCTION void CORDIC_IRQHandler2(void)
{
}
#pragma location="ccmram2"
#pragma location="ccmram2_init"
__RAM_FUNCTION void FMAC_IRQHandler2(void)
{
}


/**
  * @}
  */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
