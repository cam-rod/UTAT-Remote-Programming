/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32g4xx_it.h
  * @brief   This file contains the headers of the interrupt handlers.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32G4xx_IT_H
#define __STM32G4xx_IT_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void NMI_Handler(void);
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
void SVC_Handler(void);
void DebugMon_Handler(void);
void PendSV_Handler(void);
void SysTick_Handler(void);
void USARTx_IRQHandler(void);
void EXTI15_10_IRQHandler(void);
void HRTIM1_TIME_IRQHandler(void);

void Reset_Handler(void);
void NMI_Handler2(void);
void HardFault_Handler2(void);
void MemManage_Handler2(void);
void BusFault_Handler2(void);
void UsageFault_Handler2(void);
void SVC_Handler2(void);
void DebugMon_Handler2(void);
void PendSV_Handler2(void);
void SysTick_Handler2(void);
void WWDG_IRQHandler2(void);
void PVD_PVM_IRQHandler2(void);
void TAMP_STAMP_IRQHandler2(void);
void RTC_WKUP_IRQHandler2(void);
void FLASH_IRQHandler2(void);
void RCC_IRQHandler2(void);
void EXTI0_IRQHandler2(void);
void EXTI1_IRQHandler2(void);
void EXTI2_IRQHandler2(void);
void EXTI3_IRQHandler2(void);
void EXTI4_IRQHandler2(void);
void DMA1_Channel1_IRQHandler2(void);
void DMA1_Channel2_IRQHandler2(void);
void DMA1_Channel3_IRQHandler2(void);
void DMA1_Channel4_IRQHandler2(void);
void DMA1_Channel5_IRQHandler2(void);
void DMA1_Channel6_IRQHandler2(void);
void DMA1_Channel7_IRQHandler2(void);
void ADC1_2_IRQHandler2(void);
void USB_HP_IRQHandler2(void);
void USB_LP_IRQHandler2(void);
void FDCAN1_IT0_IRQHandler2(void);
void FDCAN1_IT1_IRQHandler2(void);
void EXTI9_5_IRQHandler2(void);
void TIM1_BRK_TIM15_IRQHandler2(void);
void TIM1_UP_TIM16_IRQHandler2(void);
void TIM1_TRG_COM_TIM17_IRQHandler2(void);
void TIM1_CC_IRQHandler2(void);
void TIM2_IRQHandler2(void);
void TIM3_IRQHandler2(void);
void TIM4_IRQHandler2(void);
void I2C1_EV_IRQHandler2(void);
void I2C1_ER_IRQHandler2(void);
void I2C2_EV_IRQHandler2(void);
void I2C2_ER_IRQHandler2(void);
void SPI1_IRQHandler2(void);
void SPI2_IRQHandler2(void);
void USART1_IRQHandler2(void);
void USART2_IRQHandler2(void);
void USART3_IRQHandler2(void);
void EXTI15_10_IRQHandler2(void);
void RTC_Alarm_IRQHandler2(void);
void USBWakeUp_IRQHandler2(void);
void TIM8_BRK_IRQHandler2(void);
void TIM8_UP_IRQHandler2(void);
void TIM8_TRG_COM_IRQHandler2(void);
void TIM8_CC_IRQHandler2(void);
void ADC3_IRQHandler2(void);
void FMC_IRQHandler2(void);
void LPTIM1_IRQHandler2(void);
void TIM5_IRQHandler2(void);
void SPI3_IRQHandler2(void);
void UART4_IRQHandler2(void);
void UART5_IRQHandler2(void);
void TIM6_DAC_IRQHandler2(void);
void TIM7_DAC_IRQHandler2(void);
void DMA2_Channel1_IRQHandler2(void);
void DMA2_Channel2_IRQHandler2(void);
void DMA2_Channel3_IRQHandler2(void);
void DMA2_Channel4_IRQHandler2(void);
void DMA2_Channel5_IRQHandler2(void);
void ADC4_IRQHandler2(void);
void ADC5_IRQHandler2(void);
void USBPD_IRQHandler2(void);
void COMP1_2_3_IRQHandler2(void);
void COMP4_5_6_IRQHandler2(void);
void COMP7_IRQHandler2(void);
void HRTIM1_Master_IRQHandler2(void);
void HRTIM1_TIMA_IRQHandler2(void);
void HRTIM1_TIMB_IRQHandler2(void);
void HRTIM1_TIMC_IRQHandler2(void);
void HRTIM1_TIMD_IRQHandler2(void);
void HRTIM1_TIME_IRQHandler2(void);
void HRTIM1_FLT_IRQHandler2(void);
void HRTIM1_TIMF_IRQHandler2(void);
void CRS_IRQHandler2(void);
void SAI1_IRQHandler2(void);
void TIM20_BRK_IRQHandler2(void);
void TIM20_UP_IRQHandler2(void);
void TIM20_TRG_COM_IRQHandler2(void);
void TIM20_CC_IRQHandler2(void);
void FPU_IRQHandler2(void);
void I2C4_EV_IRQHandler2(void);
void I2C4_ER_IRQHandler2(void);
void SPI4_IRQHandler2(void);
void FDCAN2_IT0_IRQHandler2(void);
void FDCAN2_IT1_IRQHandler2(void);
void FDCAN3_IT0_IRQHandler2(void);
void FDCAN3_IT1_IRQHandler2(void);
void RNG_IRQHandler2(void);
void LPUART1_IRQHandler2(void);
void I2C3_EV_IRQHandler2(void);
void I2C3_ER_IRQHandler2(void);
void DMAMUX_OVR_IRQHandler2(void);
void QUADSPI_IRQHandler2(void);
void DMA1_Channel8_IRQHandler2(void);
void DMA2_Channel6_IRQHandler2(void);
void DMA2_Channel7_IRQHandler2(void);
void DMA2_Channel8_IRQHandler2(void);
void CORDIC_IRQHandler2(void);
void FMAC_IRQHandler2(void);
/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

#ifdef __cplusplus
}
#endif

#endif /* __STM32G4xx_IT_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
