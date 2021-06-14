/**
  ******************************************************************************
  * @file    STM32G474E-EVAL/src/ymodem.c
  * @author  MCD Application Team
  * @version v2.0.0
  * @date    16-May-2019
  * @brief   This file provides all the software functions related to the ymodem
  *          protocol.
  ******************************************************************************
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software
  * distributed under the License is distributed on an "AS IS" BASIS,
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/** @addtogroup STM32G474E-EVAL
  * @{
  */

/** @addtogroup Ymodem
  * @brief Ymodem data reception.
  * @{
  */

/* Includes ------------------------------------------------------------------*/
#include "flash_if.h"
#include "common.h"
#include "ymodem.h"
#include "string.h"
#include "main.h"
#include "menu.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define CRC16_F       /* activate the CRC16 integrity */
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* @note ATTENTION - please keep this variable 32bit alligned */
#if defined ( __ICCARM__ ) /* IAR Compiler */
#pragma data_alignment=4
#endif /* IAR Compiler */
uint8_t aPacketData[PACKET_1K_SIZE + PACKET_DATA_INDEX + PACKET_TRAILER_SIZE];
#ifdef ENCRYPT
#if defined ( __ICCARM__ ) /* IAR Compiler */
__attribute__ ((aligned (4)))
#endif /* IAR Compiler */
uint8_t aDecryptData[PACKET_1K_SIZE];
#endif

/* Private function prototypes -----------------------------------------------*/
static HAL_StatusTypeDef ReceivePacket(uint8_t *p_data, uint32_t *p_length, uint32_t timeout);

/* Private functions ---------------------------------------------------------*/
/**
  * @brief  Receive a packet from sender
  * @param  p_data
  * @param  p_length
  *     0: end of transmission
  *     2: abort by sender
  *    >0: packet length
  * @param  timeout
  * @retval HAL_OK: normally return
  *         HAL_BUSY: abort by user
  */
static HAL_StatusTypeDef ReceivePacket(uint8_t *p_data, uint32_t *p_length, uint32_t timeout)
{
  uint32_t crc;
  uint32_t packet_size = 0U;
  HAL_StatusTypeDef status;
  uint8_t char1;

  *p_length = 0U;
  status = HAL_UART_Receive(&UartHandle, &char1, 1U, timeout);

  while (UartHandle.RxState == HAL_UART_STATE_BUSY_RX)
  {}

  if (status == HAL_OK)
  {
    switch (char1)
    {
      case SOH:
        packet_size = PACKET_SIZE;
        break;
      case STX:
        packet_size = PACKET_1K_SIZE;
        break;
      case EOT:
        break;
      case CA:
        if ((HAL_UART_Receive(&UartHandle, &char1, 1U, timeout) == HAL_OK) && (char1 == CA))
        {
          packet_size = 2U;
        }
        else
        {
          status = HAL_ERROR;
        }
        break;
      case ABORT1:
      case ABORT2:
        status = HAL_BUSY;
        break;
      default:
        status = HAL_ERROR;
        break;
    }
    *p_data = char1;

    if (packet_size >= PACKET_SIZE )
    {
      status = HAL_UART_Receive(&UartHandle, &p_data[PACKET_NUMBER_INDEX], (uint16_t)(packet_size + PACKET_OVERHEAD_SIZE),
                                timeout);

      /* Simple packet sanity check */
      if (status == HAL_OK )
      {
        if (p_data[PACKET_NUMBER_INDEX] != ((p_data[PACKET_CNUMBER_INDEX]) ^ NEGATIVE_BYTE))
        {
          packet_size = 0U;
          status = HAL_ERROR;
        }
        else
        {
          /* Check packet CRC */
          crc = (uint32_t)p_data[ packet_size + PACKET_DATA_INDEX ] << 8U;
          crc += p_data[ packet_size + PACKET_DATA_INDEX + 1U ];
          if (HAL_CRC_Calculate(&CrcHandle, (uint32_t *)&p_data[PACKET_DATA_INDEX], packet_size) != crc )
          {
            packet_size = 0U;
            status = HAL_ERROR;
          }
        }
      }
      else
      {
        packet_size = 0U;
      }
    }
  }
  *p_length = packet_size;
  return status;
}

/* Public functions ---------------------------------------------------------*/
/**
  * @brief  Receive a file using the ymodem protocol with CRC16.
  * @param  p_size The size of the file.
  * @param  bank The actual active bank
  * @retval COM_StatusTypeDef result of reception/programming
  */
COM_StatusTypeDef Ymodem_Receive ( uint32_t *p_size, uint32_t bank )
{
  uint32_t i, packet_length, session_done = 0U, file_done, errors = 0U, session_begin = 0U;
  uint32_t flashdestination, ramsource, filesize;
  uint8_t *file_ptr;
  uint8_t file_size[FILE_SIZE_LENGTH], tmp, packets_received;
  COM_StatusTypeDef result = COM_OK;

  /* Initialize flashdestination variable */
  if (bank == 1U )
  {
    flashdestination = FLASH_START_BANK1;
  }
  else
  {
    flashdestination = FLASH_START_BANK2;
  }

  while ((session_done == 0U) && (result == COM_OK))
  {
    packets_received = 0U;
    file_done = 0U;
    while ((file_done == 0U) && (result == COM_OK))
    {
      switch (ReceivePacket(aPacketData, &packet_length, DOWNLOAD_TIMEOUT))
      {
        case HAL_OK:
          errors = 0U;
          switch (packet_length)
          {
            case 2:
              /* Abort by sender */
              Serial_PutByte(ACK);
              result = COM_ABORT;
              break;
            case 0:
              /* End of transmission */
              Serial_PutByte(ACK);
              file_done = 1U;
              break;
            default:
              /* Normal packet */
              if (aPacketData[PACKET_NUMBER_INDEX] != packets_received)
              {
                Serial_PutByte(NAK);
              }
              else
              {
                if (packets_received == 0U)
                {
                  /* File name packet */
                  if (aPacketData[PACKET_DATA_INDEX] != 0U)
                  {
                    /* File name extraction */
                    i = 0U;
                    file_ptr = aPacketData + PACKET_DATA_INDEX;
                    while ( (*file_ptr != 0U) && (i < FILE_NAME_LENGTH))
                    {
                      aFileName[i++] = *file_ptr++;
                    }

                    /* File size extraction */
                    aFileName[i++] = '\0';
                    i = 0U;
                    file_ptr ++;
                    while ( (*file_ptr != ' ') && (i < FILE_SIZE_LENGTH))
                    {
                      file_size[i++] = *file_ptr++;
                    }
                    file_size[i++] = '\0';
                    Str2Int(file_size, &filesize);

                    /* Test the size of the image to be sent */
                    /* Image size is greater than Flash size */
                    *p_size = filesize;
                    if (*p_size > (FLASH_START_BANK2 - FLASH_START_BANK1))
                    {
                      /* End session */
                      tmp = CA;
                      HAL_UART_Transmit(&UartHandle, &tmp, 1U, NAK_TIMEOUT);
                      HAL_UART_Transmit(&UartHandle, &tmp, 1U, NAK_TIMEOUT);
                      result = COM_LIMIT;
                    }
                    else
                    {
                      /* erase destination area - always the other bank mapped on 0x08018000*/
                      FLASH_If_Erase(bank);
                      Serial_PutByte(ACK);
                      Serial_PutByte(CRC16);
                    }
                  }
                  /* File header packet is empty, end session */
                  else
                  {
                    Serial_PutByte(ACK);
                    file_done = 1U;
                    session_done = 1U;
                    break;
                  }
                }
                else /* Data packet */
                {
#ifdef ENCRYPT
                  if (HAL_CRYP_Decrypt( &DecHandle, (uint32_t *)&aPacketData[PACKET_DATA_INDEX],
                                        packet_length >> 2, (uint32_t *)&aDecryptData[0],  NAK_TIMEOUT) != HAL_OK)
                  {
                    /* End session */
                    Serial_PutByte(CA);
                    Serial_PutByte(CA);
                    result = COM_DATA;
                    break;
                  }
                  ramsource = (uint32_t) & aDecryptData;
#else
                  ramsource = (uint32_t) & aPacketData[PACKET_DATA_INDEX];
#endif
                  /* Write received data in Flash */
                  if (FLASH_If_Write(flashdestination, (uint32_t *) ramsource, packet_length / 4U) == FLASHIF_OK)
                  {
                    flashdestination += packet_length;
                    Serial_PutByte(ACK);
                  }
                  else /* An error occurred while writing to Flash memory */
                  {
                    /* End session */
                    Serial_PutByte(CA);
                    Serial_PutByte(CA);
                    result = COM_DATA;
                  }
                }
                packets_received ++;
                session_begin = 1U;
              }
              break;
          }
          break;
        case HAL_BUSY: /* Abort actually */
          Serial_PutByte(CA);
          Serial_PutByte(CA);
          result = COM_ABORT;
          break;
        default:
          if (session_begin > 0U)
          {
            errors ++;
          }
          if (errors > MAX_ERRORS)
          {
            /* Abort communication */
            Serial_PutByte(CA);
            Serial_PutByte(CA);
            result = COM_ABORT;
          }
          else
          {
            Serial_PutByte(CRC16); /* Ask for a packet */
          }
          break;
      }
    }
  }
  return result;
}

/**
  * @}
  */

/**
  * @}
  */

/*******************(C)COPYRIGHT STMicroelectronics *****END OF FILE****/
