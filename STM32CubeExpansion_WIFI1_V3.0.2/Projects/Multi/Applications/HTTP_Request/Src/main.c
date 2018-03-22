  /**
  ******************************************************************************
  * @file    main.c
  * @author  Central LAB
  * @version V2.1.0
  * @date    17-May-2016
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2015 STMicroelectronics</center></h2>
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
#include "stdio.h"
#include "string.h"
#include "wifi_module.h"
#include "wifi_globals.h"
#include "wifi_interface.h"
#include "stm32l4xx_hal_can.h"
#include "stm32l4xx_hal_can.c"

/** @defgroup WIFI_Examples
  * @{
  */
#define WINDOW_LEN	5
/** @defgroup WIFI_Example_HTTP_Request
  * @{
  */
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef    CanHandle;
/* Private function prototypes -----------------------------------------------*/
WiFi_Status_t wifi_get_AP_settings(void);
char print_msg_buff[512];
uint8_t user_buffer[513];
int databuf[20];	//data buffer to hold ECU CAN data
int filtered[20];  //filtered data
char indexptr = 0;			//run time index pointer
char flag = 0;	//collection complete flag;
__IO char http_char;
wifi_bool mqtt_publish_request = WIFI_TRUE;

uint8_t console_input[1], console_count=0;
char console_ssid[40];
char console_psk[20];
char console_host[20];
wifi_bool set_AP_config = WIFI_FALSE;

/* Private functions ---------------------------------------------------------*/
void  SystemClock_Config(void);
void  UART_Msg_Gpio_Init(void);
void  USART_PRINT_MSG_Configuration(UART_HandleTypeDef *UART_MsgHandle, uint32_t baud_rate);
void Error_CAN_Handler(void);
void CAN_Config(void);
//void moving_avg_wrapper(int* rawdataptr, int* filterdataptr, int count);
//int movingAvg(int* ptrArrNumbers, long* ptrSum, int pos, int len, int nextNum);
//WiFi_Status_t wifi_get_AP_settings(void);

/* Private Declarartion ------------------------------------------------------*/
__IO wifi_state_t wifi_state;
wifi_config config;
UART_HandleTypeDef UART_MsgHandle;

char * ssid = "DEV-WiFi-Pub";
char * seckey = "farwa12345";
WiFi_Priv_Mode mode = WPA_Personal;
char * hostname = "httpbin.org";
char * post_hostname = "broker.hivemq.com";
char * gcfg_key1 = "ip_ipaddr";
char * gcfg_key2 = "nv_model";
char wifi_ip_addr[20];
int ubKeyNumber = 0;
 /**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
  WiFi_Status_t status = WiFi_MODULE_SUCCESS;
  char * path = "/get";
  uint32_t  port_num = 8000;
  char * post_path = "/mqtt";

  __GPIOA_CLK_ENABLE();
  HAL_Init();

  /* Configure the system clock to 64 MHz */
  SystemClock_Config();
  SystemCoreClockUpdate();
  
  /* configure the timers  */
  Timer_Config( );
  BSP_LED_Init(LED2);

#ifdef USART_PRINT_MSG
  UART_Msg_Gpio_Init();
  USART_PRINT_MSG_Configuration(&UART_MsgHandle,115200);
  Set_UartMsgHandle(&UART_MsgHandle);
#endif  

/*##-1- Configure the CAN peripheral #######################################*/
CAN_Config();

/*##-2- Start the Reception process and enable reception interrupt #########*/
 if (HAL_CAN_Receive_IT(&CanHandle, CAN_FIFO0) != HAL_OK)
 {
   /* Reception Error */
   printf("\r\nError in Receiving CAN frames");
   Error_Handler();
 }
 //--------------------------------------------------------------------------------
	    /* Set the data to be tranmitted */
 //  CanHandle.pTxMsg->Data[0] = ubKeyNumber;
 //  CanHandle.pTxMsg->Data[1] = 0xFF;
 /*##-3- Start the Transmission process ###############################*/
	//   	   if (HAL_CAN_Transmit(&CanHandle, 10) != HAL_OK)
	//   	   {
	   		 /* Transmition Error */
	 //  		 printf("\r\nError in Transmitting CAN frames");
	 //  		 Error_Handler();
	 //  	   }
 //----------------------------------------------------------------------------------
/*------------------------------- USER BUTTON Config -------------------------------*/

  BSP_PB_Init(BUTTON_KEY, BUTTON_MODE_EXTI);	//PORTC.13


  status = wifi_get_AP_settings();
  if(status!=WiFi_MODULE_SUCCESS)
  {
    printf("\r\nError in AP Settings");
    return 0;
  }
  
  UART_Configuration(115200); 
  
  config.power=wifi_active;
  config.power_level=high;
  config.dhcp=on;//use DHCP IP address   
  wifi_state = wifi_state_idle;    
  
  printf("\r\n\nInitializing the wifi module...");
  
  /* Init the wi-fi module */  
  status = wifi_init(&config);
  if(status!=WiFi_MODULE_SUCCESS)
  {
    printf("Error in Config");
    return 0;
  }

  while (1)
  {
    switch (wifi_state) 
    {
    case wifi_state_reset:
        break;

    case wifi_state_ready:
        printf("\r\n >>connecting to AP...\r\n");
        wifi_connect(console_ssid, console_psk, mode);
        wifi_state = wifi_state_idle;
        break;

    case wifi_state_connected:
        printf("\r\n >>connected...\r\n");
        wifi_state = wifi_state_activity;
        break;

    case wifi_state_disconnected:
        wifi_state = wifi_state_reset;
        break;

    case wifi_state_activity:

        status = GET_Configuration_Value(gcfg_key1,(uint32_t *)wifi_ip_addr);
        printf("\r\n>>IP address is %s\r\n", wifi_ip_addr);
        
        memset(wifi_ip_addr, 0x00, 20);
        
        status = GET_Configuration_Value(gcfg_key2,(uint32_t *)wifi_ip_addr);
        printf("\r\n>>model no is %s\r\n", wifi_ip_addr);
        wifi_state =wifi_state_inter;
        break;

    case wifi_state_inter:        

    	printf("\r\nConnecting to MQTT Broker.. \r\n");

    	status = wifi_mqtt_connect((uint8_t *)post_hostname, (uint8_t *)post_path, port_num);

    	if(status == WiFi_MODULE_SUCCESS)
    	        {
    	          printf("\r\nMQTT Connect OK\r\n");

    	          status = wifi_mqtt_publish((uint8_t *)post_hostname, (uint8_t *)post_path, port_num);

    	          if(status == WiFi_MODULE_SUCCESS)
    	                {
    	                 printf("\r\nMQTT Publish OK\r\n");
						}
					else
						{
						  printf("\r\nMQTT Publish Error\r\n");
						}
    	        }

    	else
    		{
    		printf("\r\nMQTT Connect Error\r\n");
    		}

    	wifi_state = wifi_state_idle;
        break;

    case wifi_state_print_data:
        printf((char*)user_buffer);

        wifi_state = wifi_state_idle;

        break;
    case wifi_state_idle:        
        printf("."); 
        fflush(stdout);
        HAL_Delay(500);

        break;

    default:
        break;
    }
 /*   if(flag==1){
    	moving_avg_wrapper(databuf,filtered, 20);
    	flag = 0;
    }*/
  }
}
/*--------------------------- Moving Average filter-------------------------------------------*/
 int movingAvg(int* ptrArrNumbers, long* ptrSum, int pos, int len, int nextNum)
{
  //Subtract the oldest number from the prev sum, add the new number
  *ptrSum = *ptrSum - ptrArrNumbers[pos] + nextNum;
  //Assign the nextNum to the position in the array
  ptrArrNumbers[pos] = nextNum;
  //return the average
  return *ptrSum / len;
}
// raw data  and len for filtering
 void moving_avg_wrapper(int* rawdataptr, int* filterdataptr, int count)
{
  // a sample array of numbers. The represent "readings" from a sensor over time
 // int sample[] = {50, 10, 20, 18, 20, 100, 18, 10, 13, 500, 50, 40, 10};
  // the size of this array represents how many numbers will be used
  // to calculate the average
  int arrNumbers[WINDOW_LEN] = {0,0,0,0,0}; //moving average filter window

  char pos = 0;
  int newAvg = 0;
  long sum = 0;
  char len = WINDOW_LEN;

  for(char i = 0; i < count; i++){
    newAvg = movingAvg(arrNumbers, &sum, pos, len, *rawdataptr);
    printf("The new average is %d\n", newAvg);
    rawdataptr++;
    *filterdataptr = newAvg;
    filterdataptr++;
    pos++;
    if (pos >= len){
      pos = 0;
    }
  }
}
/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (HSI)
  *            SYSCLK(Hz)                     = 64000000
  *            HCLK(Hz)                       = 64000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 2
  *            APB2 Prescaler                 = 1
  *            PLLMUL                         = 16
  *            Flash Latency(WS)              = 2
  * @param  None
  * @retval None
  */

#ifdef USE_STM32F1xx_NUCLEO

void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef clkinitstruct = {0};
  RCC_OscInitTypeDef oscinitstruct = {0};
  
  /* Configure PLL ------------------------------------------------------*/
  /* PLL configuration: PLLCLK = (HSI / 2) * PLLMUL = (8 / 2) * 16 = 64 MHz */
  /* PREDIV1 configuration: PREDIV1CLK = PLLCLK / HSEPredivValue = 64 / 1 = 64 MHz */
  /* Enable HSI and activate PLL with HSi_DIV2 as source */
  oscinitstruct.OscillatorType  = RCC_OSCILLATORTYPE_HSE;
  oscinitstruct.HSEState        = RCC_HSE_ON;
  oscinitstruct.LSEState        = RCC_LSE_OFF;
  oscinitstruct.HSIState        = RCC_HSI_OFF;
  oscinitstruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  oscinitstruct.HSEPredivValue    = RCC_HSE_PREDIV_DIV1;
  oscinitstruct.PLL.PLLState    = RCC_PLL_ON;
  oscinitstruct.PLL.PLLSource   = RCC_PLLSOURCE_HSE;
  oscinitstruct.PLL.PLLMUL      = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&oscinitstruct)!= HAL_OK)
  {
    /* Initialization Error */
    while(1); 
  }

  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  clkinitstruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  clkinitstruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  clkinitstruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  clkinitstruct.APB2CLKDivider = RCC_HCLK_DIV1;
  clkinitstruct.APB1CLKDivider = RCC_HCLK_DIV2;  
  if (HAL_RCC_ClockConfig(&clkinitstruct, FLASH_LATENCY_2)!= HAL_OK)
  {
    /* Initialization Error */
    while(1); 
  }
}
#endif

#ifdef USE_STM32F4XX_NUCLEO

void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;

  /* Enable Power Control clock */
  __PWR_CLK_ENABLE();
  
  /* The voltage scaling allows optimizing the power consumption when the device is 
     clocked below the maximum system frequency, to update the voltage scaling value 
     regarding system frequency refer to product datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  
  /* Enable HSI Oscillator and activate PLL with HSI as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 0x10;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);
   
  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;  
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);
}
#endif

#ifdef USE_STM32L0XX_NUCLEO


/**
 * @brief  System Clock Configuration
 * @param  None
 * @retval None
 */
  void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;

  __PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 0x10;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_4;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_2;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);//RCC_CLOCKTYPE_SYSCLK;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;//RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1);

  __SYSCFG_CLK_ENABLE(); 
}
#endif

#ifdef USE_STM32L4XX_NUCLEO
/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow :
  *            System Clock source            = PLL (MSI)
  *            SYSCLK(Hz)                     = 80000000
  *            HCLK(Hz)                       = 80000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 1
  *            APB2 Prescaler                 = 1
  *            MSI Frequency(Hz)              = 4000000
  *            PLL_M                          = 1
  *            PLL_N                          = 40
  *            PLL_R                          = 2
  *            PLL_P                          = 7
  *            PLL_Q                          = 4
  *            Flash Latency(WS)              = 4
  * @param  None
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};

  /* MSI is enabled after System reset, activate PLL with MSI as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLP = 7;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    /* Initialization Error */
    while(1);
  }


  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;  
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    /* Initialization Error */
    while(1);
  }
}
#endif

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_CAN_Handler(void)
{
  while (1)
  {
  }
}


#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @brief  Configures the CAN.
  * @param  None
  * @retval None
  */
 void CAN_Config(void)
{
  CAN_FilterConfTypeDef  sFilterConfig;
  static CanTxMsgTypeDef        TxMessage;
  static CanRxMsgTypeDef        RxMessage;

  /*##-1- Configure the CAN peripheral #######################################*/
  CanHandle.Instance = CANx;
  CanHandle.pTxMsg = &TxMessage;
  CanHandle.pRxMsg = &RxMessage;

  CanHandle.Init.TTCM = DISABLE;
  CanHandle.Init.ABOM = DISABLE;
  CanHandle.Init.AWUM = DISABLE;
  CanHandle.Init.NART = ENABLE; // if ENABLE Each message is sent only once
  CanHandle.Init.RFLM = DISABLE;
  CanHandle.Init.TXFP = DISABLE;
  CanHandle.Init.Mode = CAN_MODE_NORMAL;
  CanHandle.Init.SJW = CAN_SJW_1TQ;
  CanHandle.Init.BS1 = CAN_BS1_11TQ;
  CanHandle.Init.BS2 = CAN_BS2_4TQ;
  CanHandle.Init.Prescaler = 20;

  if (HAL_CAN_Init(&CanHandle) != HAL_OK)
  {
    /* Initiliazation Error */
    Error_CAN_Handler();
  }

  /*##-2- Configure the CAN Filter ###########################################*/
  sFilterConfig.FilterNumber = 0;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterIdHigh = 0x0123<<5;
  sFilterConfig.FilterIdLow = 0x0000;
  sFilterConfig.FilterMaskIdHigh = 0xFFFF;
  sFilterConfig.FilterMaskIdLow = 0xFFFF;
  sFilterConfig.FilterFIFOAssignment = 0;
  sFilterConfig.FilterActivation = ENABLE;
  sFilterConfig.BankNumber = 14;

  if (HAL_CAN_ConfigFilter(&CanHandle, &sFilterConfig) != HAL_OK)
  {
    /* Filter configuration Error */
    Error_CAN_Handler();
  }

  /*##-3- Configure Transmission process #####################################*/
  CanHandle.pTxMsg->StdId = 0x225;
  CanHandle.pTxMsg->ExtId = 0x01;
  CanHandle.pTxMsg->RTR = CAN_RTR_DATA;
  CanHandle.pTxMsg->IDE = CAN_ID_STD;
  CanHandle.pTxMsg->DLC = 2;
}

/**
  * @brief  Transmission  complete callback in non blocking mode
  * @param  CanHandle: pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @retval None
  */
void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef *CanHandle)
{
  if ((CanHandle->pRxMsg->StdId == 0x321) && (CanHandle->pRxMsg->IDE == CAN_ID_STD) && (CanHandle->pRxMsg->DLC == 2))
  {
    //LED_Display(CanHandle->pRxMsg->Data[0]);
    ubKeyNumber = CanHandle->pRxMsg->Data[0];
    //HAL_GPIO_TogglePin(GPIOA, GPIO_PIN5);
  }

  /* Receive */
  if (HAL_CAN_Receive_IT(CanHandle, CAN_FIFO0) != HAL_OK)
  {
    /* Reception Error */
    Error_CAN_Handler();
  }
}


#ifdef USART_PRINT_MSG
void USART_PRINT_MSG_Configuration(UART_HandleTypeDef *UART_MsgHandle, uint32_t baud_rate)
{
  UART_MsgHandle->Instance             = WIFI_UART_MSG;
  UART_MsgHandle->Init.BaudRate        = baud_rate;
  UART_MsgHandle->Init.WordLength      = UART_WORDLENGTH_8B;
  UART_MsgHandle->Init.StopBits        = UART_STOPBITS_1;
  UART_MsgHandle->Init.Parity          = UART_PARITY_NONE ;
  UART_MsgHandle->Init.HwFlowCtl       = UART_HWCONTROL_NONE;// USART_HardwareFlowControl_RTS_CTS;
  UART_MsgHandle->Init.Mode            = UART_MODE_TX_RX;

  if(HAL_UART_DeInit(UART_MsgHandle) != HAL_OK)
  {
    Error_Handler();
  }  
  if(HAL_UART_Init(UART_MsgHandle) != HAL_OK)
  {
    Error_Handler();
  }
      
}

void UART_Msg_Gpio_Init()
{ 
  GPIO_InitTypeDef  GPIO_InitStruct;

  /*##-1- Enable peripherals and GPIO Clocks #################################*/
  /* Enable GPIO TX/RX clock */
  USARTx_PRINT_TX_GPIO_CLK_ENABLE();
  USARTx_PRINT_RX_GPIO_CLK_ENABLE();


  /* Enable USARTx clock */
  USARTx_PRINT_CLK_ENABLE(); 
    __SYSCFG_CLK_ENABLE();
  /*##-2- Configure peripheral GPIO ##########################################*/  
  /* UART TX GPIO pin configuration  */
  GPIO_InitStruct.Pin       = WiFi_USART_PRINT_TX_PIN;
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull      = GPIO_PULLUP;
  GPIO_InitStruct.Speed     = GPIO_SPEED_HIGH;
#if defined (USE_STM32L0XX_NUCLEO) || defined(USE_STM32F4XX_NUCLEO) || defined(USE_STM32L4XX_NUCLEO)
  GPIO_InitStruct.Alternate = PRINTMSG_USARTx_TX_AF;
#endif  
  HAL_GPIO_Init(WiFi_USART_PRINT_TX_GPIO_PORT, &GPIO_InitStruct);

  /* UART RX GPIO pin configuration  */
  GPIO_InitStruct.Pin = WiFi_USART_PRINT_RX_PIN;
  GPIO_InitStruct.Mode      = GPIO_MODE_INPUT;
#if defined (USE_STM32L0XX_NUCLEO) || defined(USE_STM32F4XX_NUCLEO) || defined(USE_STM32L4XX_NUCLEO)
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Alternate = PRINTMSG_USARTx_RX_AF;
#endif 
  
  HAL_GPIO_Init(WiFi_USART_PRINT_RX_GPIO_PORT, &GPIO_InitStruct);
  
#ifdef WIFI_USE_VCOM
  /*##-3- Configure the NVIC for UART ########################################*/
  /* NVIC for USART */
  HAL_NVIC_SetPriority(USARTx_PRINT_IRQn, 0, 1);
  HAL_NVIC_EnableIRQ(USARTx_PRINT_IRQn);
#endif
}
#endif  // end of USART_PRINT_MSG

/**
  * @brief  Query the User for SSID, password and encryption mode
  * @param  None
  * @retval WiFi_Status_t
  */
WiFi_Status_t wifi_get_AP_settings(void)
{
  WiFi_Status_t status = WiFi_MODULE_SUCCESS;
  printf("\r\n\n/********************************************************\n");
  printf("\r *                                                      *\n");
  printf("\r * X-CUBE-WIFI1 Expansion Software v3.0.2               *\n");
  printf("\r * X-NUCLEO-IDW0xx1 Wi-Fi Mini-AP Configuration.        *\n");
  printf("\r * HTTP-Request Example                                 *\n");
  printf("\r *                                                      *\n");
  printf("\r *******************************************************/\n");
  printf("\r\nDo you want to setup SSID?(y/n):");
  fflush(stdout);
  scanf("%s",console_input);
  //console_input[0] = 'n';
  printf("\r\n");

  //HAL_UART_Receive(UartMsgHandle, (uint8_t *)console_input, 1, 100000);
  if(console_input[0]=='y') 
        {
              set_AP_config = WIFI_TRUE;  
              printf("Enter the SSID:");
              fflush(stdout);

              console_count=0;
              console_count=scanf("%s",console_ssid);
              printf("\r\n");

                if(console_count==39) 
                    {
                        printf("Exceeded number of ssid characters permitted");
                        return WiFi_NOT_SUPPORTED;
                    }    
              
              //printf("entered =%s\r\n",console_ssid);
              printf("Enter the password:");
              fflush(stdout);
              console_count=0;
              
              console_count=scanf("%s",console_psk);
              printf("\r\n");
              //printf("entered =%s\r\n",console_psk);
                if(console_count==19) 
                    {
                        printf("Exceeded number of psk characters permitted");
                        return WiFi_NOT_SUPPORTED;
                    }    
              printf("Enter the encryption mode(0:Open, 1:WEP, 2:WPA2/WPA2-Personal):"); 
              fflush(stdout);
             scanf("%s",console_input);
             printf("\r\n");
              //printf("entered =%s\r\n",console_input);
              switch(console_input[0])
              {
                case '0':
                  mode = None;
                  break;
                case '1':
                  mode = WEP;
                  break;
                case '2':
                  mode = WPA_Personal;
                  break;
                default:
                  printf("\r\nWrong Entry. Priv Mode is not compatible\n");
                  return WiFi_NOT_SUPPORTED;              
              }
              
              memcpy(console_host, (const char*)hostname, strlen((char*)hostname));
              
        } else 
            {
                printf("\r\n\nModule will connect with default settings.");
                memcpy(console_ssid, (const char*)ssid, strlen((char*)ssid));
                memcpy(console_psk, (const char*)seckey, strlen((char*)seckey));
                memcpy(console_host, (const char*)hostname, strlen((char*)hostname));
            }
  
  printf("\r\n/*************************************************************\r\n");
  printf("\r\n * Configuration Complete                                     \r\n");
  printf("\r\n * Please make sure a Server is running at given hostname     \r\n");
  printf("\r\n *************************************************************\r\n");
  
  return status;
}


/******** Wi-Fi Indication User Callback *********/


void ind_wifi_on()
{
  printf("\r\n\nwifi started and ready...\r\n");
  wifi_state = wifi_state_ready;
}
  
void ind_wifi_connected()
{
  printf("\r\nwifi connected...\r\n");
  wifi_state = wifi_state_connected;
}

void ind_wifi_http_data_available(uint8_t * data_ptr, uint32_t message_size)
{
  //User is adviced to copy the data immediately to a user buffer memory as the data will be flushed after this callback
  printf("\r\nData Callback\r\n");
  memcpy(user_buffer, data_ptr, message_size);
  printf((char*)user_buffer);
  printf("\r\n");
}


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
