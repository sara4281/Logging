/**
  @page Wi-Fi Expansion Board for STM32 Nucleo Boards FW UART Update Application

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

In this application the UART2 and UART1 are bridged so that the "Demonstrator GUI" application
can be run to flash the Wi-Fi FW over the UART to the Expansion module. All the user needs to do
is flash the bin files supplied with the X-CUBE-WIFI\Utilities\PC_Software\FW_Update_UART\ folder
to the relevant platform and then run the Demonstrator GUI application on the PC.

@par Hardware and Software environment

  - This example runs on STM32 Nucleo devices with WIFI1 (Wi-Fi) expansion board
    (X-NUCLEO-IDW01M1/X-NUCLEO-IDW04A1)
  - This example has been tested with STMicroelectronics STM32L053R8-Nucleo
    RevC, STM32F401RE-Nucleo RevC,  STM32L476RG-Nucleo RevC and STM32F103RB-Nucleo RevC boards and can be easily tailored to any 
    other supported device and development board.
    

@par How to use it? 

In order to make the program work, you must do the following:
 - WARNING: before opening the project with any toolchain be sure your folder
   installation path is not too in-depth since the toolchain may report errors
   after building.
 - Open IAR toolchain (this firmware has been successfully tested with
   Embedded Workbench V7.20.1).
   Alternatively you can use the Keil uVision toolchain (this firmware
   has been successfully tested with V5.10.0.2).
   Alternatively you can use the System Workbench for STM32 (SW4STM32) toolchain (this firmware
   has been successfully tested with V1.0.0).
 - Rebuild all files and load your image into target memory.
 - Run the example.
 - Alternatively, you can download the pre-built binaries in "Binary" 
   folder included in the distributed package.

 * <h3><center>&copy; COPYRIGHT STMicroelectronics</center></h3>
 */
