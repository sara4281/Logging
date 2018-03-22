/**
  @page Wi-Fi Expansion Board for STM32 Nucleo Boards STA Mode Application
  
  @verbatim
  ******************** (C) COPYRIGHT 2015 STMicroelectronics *******************
  * @file    readme.txt  
  * @version V0.0.3
  * @date    10-03-2017
  * @brief   This application contains an example using which the serial comm
			 over USB on the Nucleo could be used to send AT commands to the 
			 Wi-Fi module directly. Basically it is a virtual communication 
			 simulation. 

  ******************************************************************************
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
  @endverbatim

@par Example Description 

In this application, the user can send AT commands directly to the module using the
serial terminal as a console. 

The program takes input via the serial terminal (any terminal program like tera-term on the PC). The
serial port settings of the terminal are as follows:
  - baud rate: 115200
  - data: 8bit
  - parity: None
  - Stop-bits: 1bit
  - HW Flow Control: None

The program by default does not give any visual feedback to the user when he/she is typing AT commands
on the terminal. For visual feedback, user may use the terminal settings for "local echo".
E.g. in tera-term, go to Setup->Terminal-> enable "Local Echo" radio button.

Here is a snapshot of the output of the program at the outset:

/********************************************************
 *                                                      *
 * X-CUBE-WIFI1 Expansion Software v3.0.0               *
 * Console Application                                  *
 * Send AT commands to SPWF module directly             *
 *                                                      *
 *******************************************************/

  
@par Hardware and Software environment

  - This example runs on STM32 Nucleo devices with WIFI1 (Wi-Fi) expansion board
    (X-NUCLEO-IDW01M1/X-NUCLEO-IDW04A1)
  - For X-NUCLEO-IDW01M1 the following boards are supported: STM32L053R8-Nucleo, STM32F401RE-Nucleo, STM32L476RG-Nucleo and STM32F103RB-Nucleo
  - For X-NUCLEO-IDW04A1 the following boards are supported: STM32L053R8-Nucleo, STM32F401RE-Nucleo, STM32L476RG-Nucleo and STM32F103RB-Nucleo
    

@par How to use it? 

In order to make the program work, you must do the following:
 - WARNING: before opening the project with any toolchain be sure your folder
   installation path is not too in-depth since the toolchain may report errors
   after building.
 - Open IAR toolchain (this firmware has been successfully tested with
   Embedded Workbench V7.40.5).
   Alternatively you can use the Keil uVision toolchain (this firmware
   has been successfully tested with V5.17.0.0).
   Alternatively you can use the System Workbench for STM32 (SW4STM32) toolchain (this firmware
   has been successfully tested with V1.0.0).
 - Rebuild all files and load your image into target memory.
 - Run the example.
 - Alternatively, you can download the pre-built binaries in "Binary" 
   folder included in the distributed package.

 * <h3><center>&copy; COPYRIGHT STMicroelectronics</center></h3>
 */
