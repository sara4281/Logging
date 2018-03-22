/**
  @page Wi-Fi Expansion Board for STM32 Nucleo Boards Applications
  
  @verbatim
  ******************** (C) COPYRIGHT 2016 STMicroelectronics *******************
  * @file    readme.txt  
  * @version V0.0.4
  * @date    17-05-2017
  * @brief   This folder contains examples which shows how to use the 
			 client socket APIs available with the WIFI1 firmware. The APIs are used
			 to configure and use the Wi-Fi module in the following ways but not
			 limited to:
			 - Abstraction APIs to configure the module in STA, MiniAP mode
			 - Abstraction APIs to open/close, read/write sockets/socket servers
     		   in TCP/UDP mode
			 - Connecting to an AP in STA mode
			 - Restful APIs like HTTPGET and HTTPPOST

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

-----------
Readme.txt
-----------
This Readme.txt file describes some steps needed to overcome known bugs/issues.

---------
Contents
---------
In this folder there are several applications tested on both the STM32F401RE-Nucleo RevC and the STM32L476RG-Nucleo RevC:
	- Client_Socket
	- FW_Upgrade_UART
	- HTTP_Request	
	- Server_Socket
	- WiFi_VCOM
	
To run these applications, the Wi-Fi expansion board (X-NUCLEO-IDW01M1/X-NUCLEO-IDW04A1) plugged
on the STM32F401RE-Nucleo/STM32L476RG-Nucleo/STM32L053R8-Nucleo/STM32F103RB-Nucleo is needed.

Please read the respective readme.txt file within the application folders to
understand each application and it's usage.

Applications in folder Projects\Multi\Applications:
	-	Client_Socket
		Client scans the network and connects to desired AP if present. Client connects to server and sends and receives data.
	-	FW_Upgrade_UART
		Source code to flash Wi-Fi module firmware using the serial link of the Nucleo.
	-	Server_Socket
		Module is in mini-AP mode. PC application is client. Module opens a server socket and PC application connects to the server socket. PC application sends data and then closes the socket connection. Client receives callback for data reception and closed remote socket message.
	-	HTTP_Request
		Does a HTTP-GET request from httpbin.org/get URL. User gets the data through callback
	-	WiFi_VCOM
		AT command application directly to module through virtual serial com port

		
-----------
Setup of X-NUCLEO-IDW01M1
-----------
- to use SW Reset functionality (wifi_reset() function)
	- jumper on position JP3 (middle and top) is required
- the epoch time is set by default to : Mon, 25 Jan 2016 13:14:17 GMT
- to see DEBUG PRINTS on the serial terminal, the #define DEBUG_PRINT must be set to 1. File to change: X-CUBE-WIFI\Projects\Multi\Applications\<$PROJECT-NAME>\Inc\wifi_conf.h


-----------
Setup of X-NUCLEO-IDW04A1
-----------
- To use SPI interface:
	- JP5 must be on.
	- JP10…13 in cross mode
	- JP6…9 in position 2-3
	- JP3 in position 1-2
- To use UART interface:
	- JP5 must be off.
	- JP10…13 in close mode
	- JP6…9 in position 1-2
	- JP3 in position 1-2
- to see DEBUG PRINTS on the serial terminal, the #define DEBUG_PRINT must be set to 1. File to change: X-CUBE-WIFI\Projects\Multi\Applications\<$PROJECT-NAME>\Inc\wifi_conf.h

-----------
Supported Nucleo boards
-----------
- For X-NUCLEO-IDW01M1 the following boards are supported: STM32L053R8-Nucleo, STM32F401RE-Nucleo, STM32L476RG-Nucleo and STM32F103RB-Nucleo
- For X-NUCLEO-IDW04A1 the following boards are supported: STM32F401RE-Nucleo and STM32L476RG-Nucleo

-----------
Choosing between UART and SPI interface in the X-NUCLEOs Wi-Fi expansion board
-----------
Please edit the file X-CUBE-WIFI\Projects\Multi\Applications\<$PROJECT-NAME>\Inc\wifi_conf.h to select the communication interface (UART/SPI) between Nucleo and X-Nucleo and also configure which Wi-Fi module is used (i.e. SPWF01 or SPWF04)
Each supported Nucleo platform will have different configurations of supported interfaces and modules.
Please check which Nucleo platform (Macro) you are using and configure the interface accordingly.

Meaning of the following macros:
CONSOLE_UART_ENABLED:   Enable or disable the UART/SPI interface for either SPWF01 or SPWF04. 
                        Enabling this macro means the UART communication interface is used. 
                        Disabling will mean SPI will be used as communication  interface.
SPWF01:                 SPWF01SA module is used as underlying expansion board (X-NUCLEO-IDW01M1). Here only UART interface can be selected.
                        Enabling this macro will build for SPWF01SA. Must be disabled if SPWF04 is used.
SPWF04:                 SPWF04SA module is used as underlying expansion board (X-NUCLEO-IDW04A1). Here UART or SPI interface can be selected.
                        Enabling this macro will build for SPWF04SA. Must be disabled if SPWF01 is used.

-----------
Known-Limitations
-----------
- For X-NUCLEO-IDW04A1:
	- For the X-NUCLEO-IDW04A1, higher level application protocols are not supported like MQTT, Websocket, TFTP, etc. File APIs, ping API, server socket close for all clients and standby/wakeup APIs are not supported. The APIs seen for websocket and TFTP are under development and testing and not fully functional.
	- VCOM application at higher baud-rates (above 115200) may be problematic with missing characters during reception.
	- For SPI communication on X-NUCLEO-IDW04A1 please limit each receive data chunks to 740 bytes for sockets and 512 bytes for HTTP.
	- Turning off/on the radio (AT+S.WIFI=0) during the initialization phase might result in stalling, with no response from the 
	  module. Please reset the module/nucleo in this case.
	- Sometimes the FW might stop receiving SPI headers after prolonged data reception
- Network scan using UART interface on X-NUCLEO-IDW04A1 will yield a maximum of 10 results for each call
- It may occur that the module is not able to connect to certain Access Points and the module will repeatedly keep scanning in a loop for the network with which to connect. Or the module may connect and disconnect repeatedly. This behaviour is currently under investigation.
- Remote configuration is currently not supported
- A maximum of 50 events can be pending at any given time and any more events after this will be dropped (not queued)

-----------
Known-Bugs
-----------
- On X-NUCLEO-IDW04A1 data with NULL character in it is ignored and hence will not be received 
- Sometimes the FW might stop receiving SPI headers after prolonged data reception
- Turning off the radio (AT+S.WIFI=0) during the initialization phase might result in stalling, with no response from the module. Please reset the module/nucleo in this case.
- if remote server closes socket while data is pending, then at+s.sockc will return error and client socket will remain open (known issue)
- Hardware Issue: 
	There might be some connection issues with the ST-LINK while using any debugging tool with the Nucleo and expansion board (X-NUCLEO-IDW04M1).
	To achieve best results while debugging, it is highly recommended to remove the R21 resistor mounted on the X-NUCLEO-IDW01M1 if it is not removed already.
	Please see hardware schematic for more details.

* <h3><center>&copy; COPYRIGHT STMicroelectronics</center></h3>
 */
