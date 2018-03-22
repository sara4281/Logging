/**
  @page Utility Programs for X-CUBE-WIFI1 Applications
  
  @verbatim
  ******************** (C) COPYRIGHT 2015 STMicroelectronics *******************
  * @file    readme.txt  
  * @version V0.0.3
  * @date    31-05-2016
  * @brief   This folder contains utility programs that can be used by the user
			 while running some of the applications contained within the
			 X-CUBE-WIFI1 package:
			 - FW_Update_UART
			 - SocketTest3
			 - tcp socket server

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
This Readme.txt file describes the contents of this folder.

---------
Contents
---------
- FW_Update_UART: The binary provided in each platform folder needs to be flashed to the board to use the platform
			 as a flashing platform for the expansion module firmware. The demonstrator GUI
			 is used as the other program running on the PC for this tool. Details are in the
			 respective folder.

- SocketTest3: this program is used as a client. The usage of the
			 program and utility is detailed in the user manual. The program has 
			 scripts to run both on a Windows machine or a Ubuntu machine.
- tcp socket server: this program can be run on a windows machine
			 where it opens a socket at port number 32000 and acts as an echo server
			 socket. Any data that is written to this server socket is echoed or 
			 sent back by the server socket to client.

The programs given within this package are third party programs available on the internet
and is only supplied as a supplement to the accompanying example applications.

-----------
Known-Bugs
-----------
- None

* <h3><center>&copy; COPYRIGHT STMicroelectronics</center></h3>
 */
