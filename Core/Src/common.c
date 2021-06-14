/**
  ******************************************************************************
  * @file    STM32G474E-EVAL/Src/common.c
  * @author  MCD Application Team
  * @version v2.0.0
  * @date    16-May-2019
  * @brief   This file provides all the common functions.
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

/** @addtogroup STM32G474E-EVAL
  * @{
  */

/* Includes ------------------------------------------------------------------*/
#include "common.h"
#include "main.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
void OWN_LED_Init(Led_TypeDef Led);
/* Private functions ---------------------------------------------------------*/

/* Public functions ----------------------------------------------------------*/
void OWN_LED_Init(Led_TypeDef Led)
{
  GPIO_TypeDef     *gpio_led;
  GPIO_InitTypeDef  GPIO_InitStruct;

  if ((Led == LED1) || (Led == LED3))
  {
    if (Led == LED1)
    {
      gpio_led = LED1_GPIO_PORT;
      /* Enable the GPIO_LED clock */
      LED1_GPIO_CLK_ENABLE();
    }
    else
    {
      gpio_led = LED3_GPIO_PORT;
      /* Enable the GPIO_LED clock */
      LED3_GPIO_CLK_ENABLE();
    }

    /* Configure the GPIO_LED pin */
    GPIO_InitStruct.Pin   = DEMO_LED_PIN;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

    HAL_GPIO_Init(gpio_led, &GPIO_InitStruct);

    /* By default, turn off LED */
    HAL_GPIO_WritePin(gpio_led, DEMO_LED_PIN, GPIO_PIN_SET);
  }
}
/**
  * @brief  Initialize the example: Configure UART and CRC.
  * @param  None
  * @retval None
  */
void COM_Init(void)
{
  /* UART configured as follow:
        - BaudRate = 115200 baud
        - Word Length = 8 Bits
        - One Stop Bit
        - No parity
        - Hardware flow control disabled (RTS and CTS signals)
        - Receive and transmit enabled
  */
  UartHandle.Instance = USARTx;
  UartHandle.Init.BaudRate = 115200U;
  UartHandle.Init.WordLength = UART_WORDLENGTH_8B;
  UartHandle.Init.StopBits = UART_STOPBITS_1;
  UartHandle.Init.Parity = UART_PARITY_NONE;
  UartHandle.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  UartHandle.Init.Mode = UART_MODE_TX_RX;
  UartHandle.Init.ClockPrescaler = UART_PRESCALER_DIV4;
  UartHandle.FifoMode = UART_FIFOMODE_DISABLE;
  if (HAL_UART_Init(&UartHandle) != HAL_OK)
  {
    /* Initialization Error */
    while (1)
    {}
  }

  /* CRC configured */
  CrcHandle.Instance = CRC;

  /* The CRC-16-CCIT polynomial is used */
  CrcHandle.Init.DefaultPolynomialUse    = DEFAULT_POLYNOMIAL_DISABLE;
  CrcHandle.Init.GeneratingPolynomial    = 0x1021U;
  CrcHandle.Init.CRCLength               = CRC_POLYLENGTH_16B;

  /* The zero init value is used */
  CrcHandle.Init.DefaultInitValueUse     = DEFAULT_INIT_VALUE_DISABLE;
  CrcHandle.Init.InitValue               = 0U;

  /* The input data are not inverted */
  CrcHandle.Init.InputDataInversionMode  = CRC_INPUTDATA_INVERSION_NONE;

  /* The output data are not inverted */
  CrcHandle.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;

  /* The input data are 32-bit long words */
  CrcHandle.InputDataFormat              = CRC_INPUTDATA_FORMAT_BYTES;

  if (HAL_CRC_Init(&CrcHandle) != HAL_OK)
  {
    /* Initialization Error */
    while (1)
    {}
  }

#ifdef USE_STM32G4XX_NUCLEO
  OWN_LED_Init(LED4);
#endif

#ifdef USE_STM32G4XX_EVAL
  OWN_LED_Init(LED3);
#endif

  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);
}

/**
  * @brief  Convert an Integer to a string
  * @param  p_str: The string output pointer
  * @param  intnum: The integer to be converted
  * @retval None
  */
void Int2Str(uint8_t *p_str, uint32_t intnum)
{
  uint32_t i, divider = 1000000000U, pos = 0U, status = 0U;

  for (i = 0U; i < 10U; i++)
  {
    p_str[pos++] = (uint8_t)((intnum / divider) + 48U);

    intnum = intnum % divider;
    divider /= 10U;
    if ((p_str[pos - 1U] == '0') & (status == 0U))
    {
      pos = 0U;
    }
    else
    {
      status++;
    }
  }
}

/**
  * @brief  Convert a string to an integer
  * @param  p_inputstr: The string to be converted
  * @param  p_intnum: The integer value
  * @retval 1: Correct
  *         0: Error
  */
uint32_t Str2Int(uint8_t *p_inputstr, uint32_t *p_intnum)
{
  uint32_t i = 0U, res = 0U;
  uint32_t val = 0U;

  if ((p_inputstr[0] == '0') && ((p_inputstr[1] == 'x') || (p_inputstr[1] == 'X')))
  {
    i = 2U;
    while ( ( i < 11U ) && ( p_inputstr[i] != '\0' ) )
    {
      if (ISVALIDHEX(p_inputstr[i]))
      {
        val = (val << 4U) + CONVERTHEX(p_inputstr[i]);
      }
      else
      {
        /* Return 0, Invalid input */
        res = 0U;
        break;
      }
      i++;
    }

    /* valid result */
    if (p_inputstr[i] == '\0')
    {
      *p_intnum = val;
      res = 1U;
    }
  }
  else /* max 10-digit decimal input */
  {
    while ( ( i < 11U ) && ( res != 1U ) )
    {
      if (p_inputstr[i] == '\0')
      {
        *p_intnum = val;
        /* return 1 */
        res = 1U;
      }
      else if (((p_inputstr[i] == 'k') || (p_inputstr[i] == 'K')) && (i > 0U))
      {
        val = val << 10U;
        *p_intnum = val;
        res = 1U;
      }
      else if (((p_inputstr[i] == 'm') || (p_inputstr[i] == 'M')) && (i > 0U))
      {
        val = val << 20U;
        *p_intnum = val;
        res = 1U;
      }
      else if (ISVALIDDEC(p_inputstr[i]))
      {
        val = val * 10U + CONVERTDEC(p_inputstr[i]);
      }
      else
      {
        /* return 0, Invalid input */
        res = 0U;
        break;
      }
      i++;
    }
  }

  return res;
}

/**
  * @brief  Print a string on the HyperTerminal
  * @param  p_string: The string to be printed
  * @retval None
  */
void Serial_PutString(uint8_t *p_string)
{
  uint16_t length = 0U;

  while (p_string[length] != '\0')
  {
    length++;
  }
  HAL_UART_Transmit(&UartHandle, p_string, length, TX_TIMEOUT);
}

/**
  * @brief  Transmit a byte to the HyperTerminal
  * @param  param The byte to be sent
  * @retval HAL_StatusTypeDef HAL_OK if OK
  */
HAL_StatusTypeDef Serial_PutByte( uint8_t param )
{
  /* May be timeouted... */
  if ( UartHandle.gState == HAL_UART_STATE_TIMEOUT )
  {
    UartHandle.gState = HAL_UART_STATE_READY;
  }
  return HAL_UART_Transmit(&UartHandle, &param, 1U, TX_TIMEOUT);
}
/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
