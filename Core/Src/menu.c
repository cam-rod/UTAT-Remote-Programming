/**
  ******************************************************************************
  * @file    STM32G474E-EVAL/src/menu.c
  * @author  MCD Application Team
  * @version v2.0.0
  * @date    16-May-2019
  * @brief   This file provides the software which contains the main menu routine.
  *          The main menu gives the options of:
  *             - downloading a new binary file,
  *             - uploading internal flash memory,
  *             - executing the binary file already loaded
  *             - configuring the write protection of the Flash sectors where the
  *               user loads his binary file.
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
#include "main.h"
#include "common.h"
#include "flash_if.h"
#include "menu.h"
#include "ymodem.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint32_t BankActive = 0U, BFSysMem = 0U;
FLASH_OBProgramInitTypeDef OBConfig;
uint8_t aFileName[FILE_NAME_LENGTH];

/* Private function prototypes -----------------------------------------------*/
static void SerialDownload(void);
static uint32_t CheckOtherBank( void );

/* Private functions ---------------------------------------------------------*/
/**
  * @brief  Download a file via serial port
  * @param  None
  * @retval None
  */
static void SerialDownload(void)
{
  uint8_t number[11] = {0};
  uint32_t size = 0U;
  COM_StatusTypeDef result;

  Serial_PutString((uint8_t *)"Waiting for the file to be sent ... (press 'a' to abort)\n\r");
  result = Ymodem_Receive( &size, BankActive );
  if (result == COM_OK)
  {
    /* Reporting */
    Serial_PutString((uint8_t *)
                     "\n\n\r Programming Completed Successfully!\n\r--------------------------------\r\n Name: ");
    Serial_PutString(aFileName);
    Int2Str(number, size);
    Serial_PutString((uint8_t *)"\n\r Size: ");
    Serial_PutString(number);
    Serial_PutString((uint8_t *)" Bytes\r\n");
    Serial_PutString((uint8_t *)"-------------------\n");
  }
  else if (result == COM_LIMIT)
  {
    Serial_PutString((uint8_t *)"\n\n\rThe image size is higher than the bank size!\n\r");
  }
  else if (result == COM_DATA)
  {
    Serial_PutString((uint8_t *)"\n\n\rVerification failed!\n\r");
  }
  else if (result == COM_ABORT)
  {
    Serial_PutString((uint8_t *)"\n\rAborted by user.\n\r");
  }
  else
  {
    Serial_PutString((uint8_t *)"\n\rFailed to receive the file!\n\r");
  }
}

static uint32_t CheckOtherBank( void )
{
  uint32_t result;

  result = FLASH_If_Check((BankActive == 1U) ? FLASH_START_BANK1 : FLASH_START_BANK2);
  if (result == FLASHIF_OK)
  {
    Serial_PutString((uint8_t *)"Other bank check passed.\r\n\n");
  }
  else
  {
    Serial_PutString((uint8_t *)"Failure!\r\n\n");
  }
  return result;
}

/**
  * @brief  Display the Main Menu on HyperTerminal
  * @param  None
  * @retval None
  */
void Main_Menu(void)
{
  uint8_t key = 0U;
  uint32_t  result, loop_exit = 1U;

  Serial_PutString((uint8_t *)"\r\n======================================================================");
  Serial_PutString((uint8_t *)"\r\n=              (C) COPYRIGHT 2018 STMicroelectronics                 =");
  Serial_PutString((uint8_t *)"\r\n=                                                                    =");
#ifdef ENCRYPT
  Serial_PutString((uint8_t *)"\r\n=   STM32G484 On-the-fly update for dual bank demo  (Version 2.0.0)  =");
#else
  Serial_PutString((uint8_t *)"\r\n=   STM32G474 On-the-fly update for dual bank demo  (Version 2.0.0)  =");
#endif /* ENCRYPT */
  Serial_PutString((uint8_t *)"\r\n=                                                                    =");
  Serial_PutString((uint8_t *)"\r\n=                                   By MCD Application Team          =");
  Serial_PutString((uint8_t *)"\r\n======================================================================");
  Serial_PutString((uint8_t *)"\r\n\r\n");

  /* Get the current configuration */
  HAL_FLASHEx_OBGetConfig( &OBConfig );

  FLASH_If_WriteProtectionClear();

  while (loop_exit)
  {
    /* Test from which bank the program runs */
    BankActive = READ_BIT(SYSCFG->MEMRMP, SYSCFG_MEMRMP_FB_MODE);

    Serial_PutString((uint8_t *)"\r\n======== Main Menu ========================\r\n\n");
    if (BankActive == 0U)
    {
      Serial_PutString((uint8_t *)"  Program running from Bank 1  \r\n");
    }
    else
    {
      Serial_PutString((uint8_t *)"  Program running from Bank 2  \r\n");
    }

    if ( OBConfig.USERConfig & OB_BFB2_ENABLE ) /* BANK2 active for boot */
    {
      Serial_PutString((uint8_t *)"  System ROM bank selection active  \r\n\n");
    }
    else
    {
      Serial_PutString((uint8_t *)"  System ROM bank selection deactivated \r\n\n");
    }

    Serial_PutString((uint8_t *)"===========================================\r\n");
    Serial_PutString((uint8_t *)"  Flash binary to the other bank ------- 1\r\n");
    Serial_PutString((uint8_t *)"  Erase the other bank ----------------- 2\r\n");
    Serial_PutString((uint8_t *)"  Rewrite the other bank --------------- 3\r\n");
    Serial_PutString((uint8_t *)"  Check the other bank integrity ------- 4\r\n");
    Serial_PutString((uint8_t *)"  Switch bank -------------------------- 5\r\n");
    Serial_PutString((uint8_t *)"  Toggle the system bank selection ----- 6\r\n");
    Serial_PutString((uint8_t *)"===========================================\r\n");

    /* Clean the input path */
    __HAL_UART_FLUSH_DRREGISTER(&UartHandle);
    __HAL_UART_CLEAR_IT(&UartHandle, UART_CLEAR_OREF);

    /* Receive key */
    HAL_UART_Receive(&UartHandle, &key, 1U, RX_TIMEOUT);

    switch (key)
    {
      case '1' :
        /* Download user application in the Flash */
        SerialDownload();
        break;
      case '2' :
        /* Choose the inactive bank and erase it */
        Serial_PutString((uint8_t *)"Erasing memory, wait a moment ...\r\n\n");
        result = FLASH_If_Erase( BankActive );
        if (result == FLASHIF_OK)
        {
          Serial_PutString((uint8_t *)"Success!\r\n\n");
        }
        else
        {
          Serial_PutString((uint8_t *)"Failure!\r\n\n");
        }
        break;
      case '3' :
        /* Choose the inactive bank and erase it */
        Serial_PutString((uint8_t *)"Rewriting memory, wait a moment ...\r\n\n");
        result = FLASH_If_Erase( BankActive );
        if (result == FLASHIF_OK)
        {
          if (BankActive == 0U )
          {
            Serial_PutString((uint8_t *)"Copying new content, wait a moment ...\r\n\n");
            result = FLASH_If_Write( FLASH_START_BANK2, (uint32_t *)FLASH_START_BANK1, 80000U);
          }
          else
          {
            Serial_PutString((uint8_t *)"Copying new content, wait a moment ...\r\n\n");
            result = FLASH_If_Write( FLASH_START_BANK1, (uint32_t *)FLASH_START_BANK2, 80000U);
          }

          if (result != FLASHIF_OK)
          {
            Serial_PutString((uint8_t *)"Failure!\r\n\n");
          }
          else
          {
            Serial_PutString((uint8_t *)"Success!\r\n\n");
          }
        }
        else
        {
          Serial_PutString((uint8_t *)"Failure!\r\n\n");
        }
        break;
      case '4' :
        CheckOtherBank();
        break;
      case '5' :
        if ( CheckOtherBank() == FLASHIF_OK )
        {
          loop_exit = 0U;
        }
        break;
      case '6' :
        Serial_PutString((uint8_t *)"Demo will reboot to apply changes to OB ...\r\n\n");
        if (FLASH_If_BankSwitch() != HAL_OK)
        {
          Serial_PutString((uint8_t *)"Failure!\r\n\n");
        }
        break;
      default:
        Serial_PutString((uint8_t *)"Invalid input ! ==> Enter number in range from 1 to 6\r");
        break;
    }
  }
}

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
