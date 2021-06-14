/**
  @page STM32G474E-EVAL AN4767 STM32G4xx On-the-fly firmware upgrade readme file
  
  @verbatim
  ******************** (C) COPYRIGHT 2016 STMicroelectronics *******************
  * @file    STM32G474E-EVAL/readme.txt 
  * @author  MCD Application Team
  * @version v2.0.0
  * @date    16-May-2019
  * @brief   Description of the AN4767 "STM32G4xx On-the-fly firmware upgrade dual-bank demo".
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
  * limitations under the License
  *   
  ******************************************************************************
  @endverbatim

@par Example Description

This directory contains a set of sources files and pre-configured project that 
describes how to build an application to be loaded into Flash memory using the 
dual bank capability (through USART).

@par Directory contents

 - "STM32G474E-EVAL/inc": contains the firmware header files 
    - STM32G474E-EVAL/inc/main.h              The main include file of the project.
    - STM32G474E-EVAL/inc/common.h            This file provides all the headers of the common functions.
    - STM32G474E-EVAL/inc/flash_if.h          This file provides all the firmware 
                                            function headers of the flash_if.c file.
    - STM32G474E-EVAL/inc/menu.h              This file provides all the firmware
                                            function headers of the menu.c file.
    - STM32G474E-EVAL/inc/ymodem.h            This file provides all the firmware
                                            function headers of the ymodem.c file.
    - STM32G474E-EVAL/inc/stm32g4xx_hal_conf.h  Library Configuration file
    - STM32G474E-EVAL/inc/stm32g4xx_it.h      Header for stm32g4xx_it.c

  - "STM32G474E-EVAL/src": contains the firmware source files
    - STM32G474E-EVAL/src/main.c              Main program
    - STM32G474E-EVAL/src/stm32g4xx_it.c      Interrupt handlers in RAM for Bank1 code
    - STM32G474E-EVAL/src/stm32g4xx_it2.c     Interrupt handlers in RAM for Bank2 code
    - STM32G474E-EVAL/src/stm32g4xx_hal_msp.c Microcontroller specific packages
                                            initialization file.
    - STM32G474E-EVAL/src/flash_if.c          The file contains write, erase and disable
                                            write protection of the internal Flash
                                            memory.
    - STM32G474E-EVAL/src/menu.c              This file contains the menu to select
                                            downloading a new binary file, uploading
                                            internal Flash memory, executing the binary
                                            and disabling the write protection of
                                            write-protected pages
    - STM32G474E-EVAL/src/common.c            This file provides functions related to
                                            read/write from/to USART peripheral
    - STM32G474E-EVAL/src/ymodem.c            This file provides all the firmware functions
                                            related to the ymodem protocol.
    - STM32G474E-EVAL/src/LED_Demo.c          Demonstration involving the HR Timer to
											showcase the bank switch timing.
    - STM32G474E-EVAL/src/system_stm32g4xx.c  STM32G4xx system source file

@par Hardware and Software environment

  - This example runs on STM32G4xx Devices.
  - This example has been tested with STMicroelectronics STM32G474E-EVAL
    and with STM32G484RE device for encyption. The example can be easily tailored to any
    other supported device and development board.
  
  - STM32G474-EVAL Set-up
    - Connect a null-modem female/female RS232 cable between the boards DB9 connector 
      and PC serial port (or any suitable USB-to COM adapter).
	
  - STM32G484 variant
    - The STM32G484 in the Eval board must be replaced with the cryptography-enabled STM32G484
	  counterpart.
	- Download "crypto++" freeware cryptography library and demonstrator.
    - Use the crypto.bat in the Tools library to encrypt the binary for field upgrade.	´

  - Terminal configuration: 
    - Word Length = 8 Bits
    - One Stop Bit
    - No parity
    - BaudRate = 115200 baud
    - flow control: None 
    - Ymodem protocol is using CRC16 by default. To switch to checksum, comment #define CRC16_F
      in ymodem.c

@par How to use it? 

In order to make the program work, you must do the following:

  1. Open Terminal window using the settings listed in section "Hardware and Software environment" 
  2. Load and execute the example software. The main menu is then displayed on the Terminal window.
  3. Menu items should be self explanatory, there is no risk involved in experimenting with them.
  4. To go step-by-step through the proposed on-the-fly update sequence press:
     1, 4, 5, 6 (BFB2 ON), 3, 4, 5, 6 (BFB2 OFF).
	 Option 2 (erase) is included to simulate failure state.

In order to load the DB code, you have do the following:
      - Open the Project.eww workspace
      - Rebuild all files: Project->Rebuild all
      - Load project image: Project->Debug
      - Run program: Debug->Go(F5)
      
 * <h3><center>&copy; COPYRIGHT STMicroelectronics</center></h3>
 */
