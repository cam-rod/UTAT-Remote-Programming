/**
  ******************************************************************************
  * @file    STM32G4xx_DB/src/LED_Demo.c
  * @author  MCD Application Team
  * @version v2.0.0
  * @date    16-May-2019
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2019 STMicroelectronics</center></h2>
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

/** @addtogroup STM32G4xx_DB
  * @{
  */
/* Exported variables --------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

HRTIM_HandleTypeDef hhrtim1;

/* Private variables ---------------------------------------------------------*/
#define HRTIM_INPUT_CLOCK       ((uint64_t)150000000)   /* Value in Hz */

/* Formula below works down to 70.3kHz (with presc ratio = 1) */
#define _250KHz_PERIOD ((uint16_t)((HRTIM_INPUT_CLOCK * 32) / 250000))

static void MX_GPIO_Init(void);
static void MX_HRTIM1_Init(void);
static void DAC_Init(void);
static void COMP_Init(void);

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
void LED_Demo_Init(void)
{
  MX_GPIO_Init();
  DAC_Init();
  COMP_Init();
  MX_HRTIM1_Init();

}

/**
  * @brief  DAC4_CH2 init, sawtooth mode
  * @param  None
  * @retval None
  */
static void DAC_Init(void)
{

  __HAL_RCC_DAC4_CLK_ENABLE();

  /* Init DAC4_CH2 in sawtooth mode, trigger enabled */
  DAC4->CR = DAC_CR_WAVE2_1 + DAC_CR_WAVE2_0 + DAC_CR_TSEL2_3 + DAC_CR_TSEL2_2 + DAC_CR_TSEL2_0 + DAC_CR_TEN2;

  /* Select hrtim_dac_reset_trg5 and hrtim_dac_step_trg5 */
  DAC4->STMODR = DAC_STMODR_STRSTTRIGSEL2_3 + DAC_STMODR_STRSTTRIGSEL2_2 + DAC_STMODR_STRSTTRIGSEL2_0
                 + DAC_STMODR_STINCTRIGSEL2_3 + DAC_STMODR_STINCTRIGSEL2_2 + DAC_STMODR_STINCTRIGSEL2_0;
  /* High-speed mode (AHB clock > 80MHz) */
  DAC4->MCR = DAC_MCR_HFSEL_0 + DAC_MCR_MODE2_1 + DAC_MCR_MODE2_0;

  DAC4->STR2 = (16 << 16) + 35;

  DAC4->CR |= DAC_CR_EN2;

  __HAL_RCC_DAC2_CLK_ENABLE();

  /* DAC start-up value */
  DAC2->DHR12R1 = 64;

  /* High-speed mode (AHB clock > 80MHz) */
  DAC2->MCR = DAC_MCR_HFSEL_0 + DAC_MCR_MODE1_1 + DAC_MCR_MODE1_0;

  /* Enable DAC */
  DAC2->CR |= DAC_CR_EN1;
}

/**
  * @brief  COMP1 Init
  * @param  None
  * @retval None
  */
static void COMP_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  __HAL_RCC_SYSCFG_CLK_ENABLE();

  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_COMP6;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* Enable COMP6: scaler and bridge enabled, In+ on PB11, In- is DAC4_CH2 */
  /* Deglitcher ON with low hysteresis value */
  COMP6->CSR = COMP_CSR_INMSEL_2 + COMP_CSR_HYST_0 + COMP_CSR_DEGLITCHEN;
  COMP6->CSR |= COMP_CSR_EN;
}

/**
  * @brief  HRTIM1 init function
  * @param  None
  * @retval None
  */
static void MX_HRTIM1_Init(void)
{
  hhrtim1.Instance = HRTIM1;
  hhrtim1.Init.HRTIMInterruptResquests = HRTIM_IT_NONE;
  hhrtim1.Init.SyncOptions = HRTIM_SYNCOPTION_NONE;
  
  if (HAL_HRTIM_Init(&hhrtim1) != HAL_OK)
  {
    while(1)
    {}
  }

  if (HAL_HRTIM_DLLCalibrationStart(&hhrtim1, HRTIM_CALIBRATIONRATE_1) != HAL_OK)
  {
    while(1)
    {}
  }

  if (HAL_HRTIM_PollForDLLCalibration(&hhrtim1, 10) != HAL_OK)
  {
    while(1)
    {}
  }

  /* TIME counter operating in continuous mode */
  HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_E].TIMxCR = HRTIM_TIMCR_CONT;

  /* Set period to 100kHz and duty cycle (CMP1) to 50% */
  HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_E].PERxR = _250KHz_PERIOD;
  HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_E].CMP1xR = (9 * _250KHz_PERIOD) / 10;
  HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_E].CMP2xR = _250KHz_PERIOD / 60;   /* 15MSps */
  HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_E].CMP3xR = 0x0500;
  HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_E].CMP4xR = 0xC0;
  HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_E].EEFxR1 = HRTIM_EEFR1_EE3FLTR_0 +
      HRTIM_EEFR1_EE3FLTR_1;
  /* Blinking on CMP3 */

  /* TE1 output set on TIMD period and reset on TIMD CMP1 event*/
  HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_E].SETx1R = HRTIM_SET1R_CMP4;
  /* Turn-on delayed to avoid the comparator to trip for very low current thresholds */
  HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_E].RSTx1R = HRTIM_RST1R_CMP1 + HRTIM_RST1R_EXTVNT3;

  /* TE2 output set on TIMD period and reset on TIMD CMP2 event*/
  HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_E].SETx2R = HRTIM_SET2R_PER;
  HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_E].RSTx2R = HRTIM_RST2R_CMP1;

  HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_E].OUTxR = HRTIM_OUTR_IDLM1 + HRTIM_OUTR_IDLM2;

  /* Enable dual channel trigger mode */
  HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_E].TIMxCR2 = HRTIM_TIMCR2_DCDE;

  /* EEV3 rising edge sensitive, fast mode, COMP6 as source */
  HRTIM1->sCommonRegs.EECR1 = HRTIM_EECR1_EE3SNS_0 + HRTIM_EECR1_EE3FAST + HRTIM_EECR1_EE3SRC_0;


  /* Enable continuous mode burst, on TIME period */
  HRTIM1->sCommonRegs.BMCR = HRTIM_BMCR_BME + HRTIM_BMCR_BMOM + HRTIM_BMCR_BMCLK_0 + HRTIM_BMCR_BMCLK_2;
  HRTIM1->sCommonRegs.BMPER = 0xFF;
  HRTIM1->sCommonRegs.BMCMPR = 0xF0;

  /* Enable TIME repetition interrupt */
  HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_E].TIMxDIER = HRTIM_TIM_IT_REP;
  HAL_NVIC_SetPriority(HRTIM1_TIME_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(HRTIM1_TIME_IRQn);


  HRTIM1->sCommonRegs.OENR = HRTIM_OENR_TE1OEN + HRTIM_OENR_TE2OEN; /* Enable TE1 output */
  HRTIM1->sMasterRegs.MCR |= HRTIM_MCR_TECEN;    /* Start Timer E */

  HRTIM1->sCommonRegs.BMTRGR = HRTIM_BMTRGR_SW;

  HAL_HRTIM_MspPostInit(&hhrtim1);

}

/** @brief Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
  * @param  None
  * @retval None
*/
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  LED1_GPIO_CLK_ENABLE();
  
  /* Configure the GPIO_LED pin */
  GPIO_InitStruct.Pin   = LED1_PIN;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(LED1_GPIO_PORT, &GPIO_InitStruct);

  /* By default, turn off LED */
  HAL_GPIO_WritePin(LED1_GPIO_PORT, DEMO_LED_PIN, GPIO_PIN_SET);
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
