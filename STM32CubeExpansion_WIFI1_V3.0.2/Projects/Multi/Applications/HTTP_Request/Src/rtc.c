/**
  ******************************************************************************
  * @file    AN4759/Projects/STM32L476RG_Nucleo/RTC_SmoothCalib/Src/RTC.c
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    25-July-2016
  * @brief   This file provides code for the configuration
  *          of the RTC instances.
  ******************************************************************************
  * @attention
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
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
#include "rtc.h"
#include "stm32l4xx_hal_rtc.h"


RTC_HandleTypeDef hrtc;	//object for RTC
/*
HAL_StatusTypeDef HAL_RTC_Init(RTC_HandleTypeDef *hrtc);
HAL_StatusTypeDef HAL_RTC_SetTime(RTC_HandleTypeDef *hrtc, RTC_TimeTypeDef *sTime, uint32_t Format);
HAL_StatusTypeDef HAL_RTC_GetTime(RTC_HandleTypeDef *hrtc, RTC_TimeTypeDef *sTime, uint32_t Format);
HAL_StatusTypeDef HAL_RTC_SetDate(RTC_HandleTypeDef *hrtc, RTC_DateTypeDef *sDate, uint32_t Format);
HAL_StatusTypeDef HAL_RTC_GetDate(RTC_HandleTypeDef *hrtc, RTC_DateTypeDef *sDate, uint32_t Format);
HAL_StatusTypeDef RTC_EnterInitMode(RTC_HandleTypeDef* hrtc);
HAL_StatusTypeDef HAL_RTC_WaitForSynchro(RTC_HandleTypeDef* hrtc);

uint8_t RTC_Bcd2ToByte(uint8_t Value);
uint8_t RTC_ByteToBcd2(uint8_t Value);
*/
/* RTC init function */
void MX_RTC_Init(void)
{
  RTC_TimeTypeDef s_time;
  RTC_DateTypeDef s_date;

  /**Initialize RTC and set the Time and Date
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;		//RTC pps (pulse per second) not required
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_POS1;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != 0)
  {
    Error_Handler();
  }

/*-----------------------------------default time------------------------*/
  s_time.Hours = 0x1;
  s_time.Minutes = 0x1;
  s_time.Seconds = 0x1;
  s_time.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  s_time.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &s_time, RTC_FORMAT_BCD) != 0)
  {
    Error_Handler();
  }

  s_date.WeekDay = RTC_WEEKDAY_MONDAY;
  s_date.Month = RTC_MONTH_MARCH;
  s_date.Date = 0x01;	//Your exam date :)
  s_date.Year = 0x16;	//BCD

  if (HAL_RTC_SetDate(&hrtc, &s_date, RTC_FORMAT_BCD) != 0)
  {
    Error_Handler();
  }

  /**Enable Calibration
  */
 // if (HAL_RTCEx_SetCalibrationOutPut(&hrtc, RTC_CALIBOUTPUT_1HZ) != 0)
 // {
 //   Error_Handler();
 // }

}

void HAL_RTC_MspInit(RTC_HandleTypeDef* hrtc)
{

  GPIO_InitTypeDef gpio_init_struct;
  if (hrtc->Instance == RTC)
  {
    /* Peripheral clock enable */
    __HAL_RCC_RTC_ENABLE();

    /**RTC GPIO Configuration
    PB2     ------> RTC_OUT_CALIB 
    */
    gpio_init_struct.Pin = GPIO_PIN_2;
    gpio_init_struct.Mode = GPIO_MODE_AF_PP;
    gpio_init_struct.Pull = GPIO_NOPULL;
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_LOW;
    gpio_init_struct.Alternate = GPIO_AF0_RTC_50Hz;
    HAL_GPIO_Init(GPIOB, &gpio_init_struct);
  }
}

void HAL_RTC_MspDeInit(RTC_HandleTypeDef* hrtc)
{

  if (hrtc->Instance == RTC)
  {
    /* Peripheral clock disable */
    __HAL_RCC_RTC_DISABLE();

    /**RTC GPIO Configuration
    PB2     ------> RTC_OUT_CALIB 
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_2);

  }
}
//------------------------------------- 	RTC Support functions -----------------------------------
HAL_StatusTypeDef HAL_RTC_Init(RTC_HandleTypeDef *hrtc)
{
  /* Check the RTC peripheral state */
  if(hrtc == NULL)
  {
     return HAL_ERROR;
  }

  /* Check the parameters */
  assert_param(IS_RTC_ALL_INSTANCE(hrtc->Instance));
  assert_param(IS_RTC_HOUR_FORMAT(hrtc->Init.HourFormat));
  assert_param(IS_RTC_ASYNCH_PREDIV(hrtc->Init.AsynchPrediv));
  assert_param(IS_RTC_SYNCH_PREDIV(hrtc->Init.SynchPrediv));
  assert_param(IS_RTC_OUTPUT(hrtc->Init.OutPut));
  assert_param(IS_RTC_OUTPUT_REMAP(hrtc->Init.OutPutRemap));
  assert_param(IS_RTC_OUTPUT_POL(hrtc->Init.OutPutPolarity));
  assert_param(IS_RTC_OUTPUT_TYPE(hrtc->Init.OutPutType));

  if(hrtc->State == HAL_RTC_STATE_RESET)
  {
    /* Allocate lock resource and initialize it */
    hrtc->Lock = HAL_UNLOCKED;

    /* Initialize RTC MSP */
    HAL_RTC_MspInit(hrtc);
  }

  /* Set RTC state */
  hrtc->State = HAL_RTC_STATE_BUSY;

  /* Disable the write protection for RTC registers */
  __HAL_RTC_WRITEPROTECTION_DISABLE(hrtc);

  /* Set Initialization mode */
  if(RTC_EnterInitMode(hrtc) != HAL_OK)
  {
    /* Enable the write protection for RTC registers */
    __HAL_RTC_WRITEPROTECTION_ENABLE(hrtc);

    /* Set RTC state */
    hrtc->State = HAL_RTC_STATE_ERROR;

    return HAL_ERROR;
  }
  else
  {
    /* Clear RTC_CR FMT, OSEL and POL Bits */
    hrtc->Instance->CR &= ((uint32_t)~(RTC_CR_FMT | RTC_CR_OSEL | RTC_CR_POL));
    /* Set RTC_CR register */
    hrtc->Instance->CR |= (uint32_t)(hrtc->Init.HourFormat | hrtc->Init.OutPut | hrtc->Init.OutPutPolarity);

    /* Configure the RTC PRER */
    hrtc->Instance->PRER = (uint32_t)(hrtc->Init.SynchPrediv);
    hrtc->Instance->PRER |= (uint32_t)(hrtc->Init.AsynchPrediv << 16);

    /* Exit Initialization mode */
    hrtc->Instance->ISR &= ((uint32_t)~RTC_ISR_INIT);

    hrtc->Instance->OR &= (uint32_t)~(RTC_OR_ALARMOUTTYPE | RTC_OR_OUT_RMP);
    hrtc->Instance->OR |= (uint32_t)(hrtc->Init.OutPutType | hrtc->Init.OutPutRemap);

    /* Enable the write protection for RTC registers */
    __HAL_RTC_WRITEPROTECTION_ENABLE(hrtc);

    /* Set RTC state */
    hrtc->State = HAL_RTC_STATE_READY;

    return HAL_OK;
  }
}
//---------------------------------------------------SET TIME---------------------------
HAL_StatusTypeDef HAL_RTC_SetTime(RTC_HandleTypeDef *hrtc, RTC_TimeTypeDef *sTime, uint32_t Format)
{
  uint32_t tmpreg = 0;

 /* Check the parameters */
  assert_param(IS_RTC_FORMAT(Format));
  assert_param(IS_RTC_DAYLIGHT_SAVING(sTime->DayLightSaving));
  assert_param(IS_RTC_STORE_OPERATION(sTime->StoreOperation));

  /* Process Locked */
  __HAL_LOCK(hrtc);

  hrtc->State = HAL_RTC_STATE_BUSY;

  if(Format == RTC_FORMAT_BIN)
  {
    if((hrtc->Instance->CR & RTC_CR_FMT) != (uint32_t)RESET)
    {
      assert_param(IS_RTC_HOUR12(sTime->Hours));
      assert_param(IS_RTC_HOURFORMAT12(sTime->TimeFormat));
    }
    else
    {
      sTime->TimeFormat = 0x00;
      assert_param(IS_RTC_HOUR24(sTime->Hours));
    }
    assert_param(IS_RTC_MINUTES(sTime->Minutes));
    assert_param(IS_RTC_SECONDS(sTime->Seconds));

    tmpreg = (uint32_t)(((uint32_t)RTC_ByteToBcd2(sTime->Hours) << 16) | \
                        ((uint32_t)RTC_ByteToBcd2(sTime->Minutes) << 8) | \
                        ((uint32_t)RTC_ByteToBcd2(sTime->Seconds)) | \
                        (((uint32_t)sTime->TimeFormat) << 16));
  }
  else
  {
    if((hrtc->Instance->CR & RTC_CR_FMT) != (uint32_t)RESET)
    {
      tmpreg = RTC_Bcd2ToByte(sTime->Hours);
      assert_param(IS_RTC_HOUR12(tmpreg));
      assert_param(IS_RTC_HOURFORMAT12(sTime->TimeFormat));
    }
    else
    {
      sTime->TimeFormat = 0x00;
      assert_param(IS_RTC_HOUR24(RTC_Bcd2ToByte(sTime->Hours)));
    }
    assert_param(IS_RTC_MINUTES(RTC_Bcd2ToByte(sTime->Minutes)));
    assert_param(IS_RTC_SECONDS(RTC_Bcd2ToByte(sTime->Seconds)));
    tmpreg = (((uint32_t)(sTime->Hours) << 16) | \
              ((uint32_t)(sTime->Minutes) << 8) | \
              ((uint32_t)sTime->Seconds) | \
              ((uint32_t)(sTime->TimeFormat) << 16));
  }

  /* Disable the write protection for RTC registers */
  __HAL_RTC_WRITEPROTECTION_DISABLE(hrtc);

  /* Set Initialization mode */
  if(RTC_EnterInitMode(hrtc) != HAL_OK)
  {
    /* Enable the write protection for RTC registers */
    __HAL_RTC_WRITEPROTECTION_ENABLE(hrtc);

    /* Set RTC state */
    hrtc->State = HAL_RTC_STATE_ERROR;

    /* Process Unlocked */
    __HAL_UNLOCK(hrtc);

    return HAL_ERROR;
  }
  else
  {
    /* Set the RTC_TR register */
    hrtc->Instance->TR = (uint32_t)(tmpreg & RTC_TR_RESERVED_MASK);

    /* Clear the bits to be configured */
    hrtc->Instance->CR &= ((uint32_t)~RTC_CR_BCK);

    /* Configure the RTC_CR register */
    hrtc->Instance->CR |= (uint32_t)(sTime->DayLightSaving | sTime->StoreOperation);

    /* Exit Initialization mode */
    hrtc->Instance->ISR &= ((uint32_t)~RTC_ISR_INIT);

    /* If  CR_BYPSHAD bit = 0, wait for synchro else this check is not needed */
    if((hrtc->Instance->CR & RTC_CR_BYPSHAD) == RESET)
    {
      if(HAL_RTC_WaitForSynchro(hrtc) != HAL_OK)
      {
        /* Enable the write protection for RTC registers */
        __HAL_RTC_WRITEPROTECTION_ENABLE(hrtc);

        hrtc->State = HAL_RTC_STATE_ERROR;

        /* Process Unlocked */
        __HAL_UNLOCK(hrtc);

        return HAL_ERROR;
      }
    }

    /* Enable the write protection for RTC registers */
    __HAL_RTC_WRITEPROTECTION_ENABLE(hrtc);

   hrtc->State = HAL_RTC_STATE_READY;

   __HAL_UNLOCK(hrtc);

   return HAL_OK;
  }
}
HAL_StatusTypeDef HAL_RTC_GetTime(RTC_HandleTypeDef *hrtc, RTC_TimeTypeDef *sTime, uint32_t Format)
{
  uint32_t tmpreg = 0;

  /* Check the parameters */
  assert_param(IS_RTC_FORMAT(Format));

  /* Get subseconds structure field from the corresponding register*/
  sTime->SubSeconds = (uint32_t)(hrtc->Instance->SSR);

  /* Get SecondFraction structure field from the corresponding register field*/
  sTime->SecondFraction = (uint32_t)(hrtc->Instance->PRER & RTC_PRER_PREDIV_S);

  /* Get the TR register */
  tmpreg = (uint32_t)(hrtc->Instance->TR & RTC_TR_RESERVED_MASK);

  /* Fill the structure fields with the read parameters */
  sTime->Hours = (uint8_t)((tmpreg & (RTC_TR_HT | RTC_TR_HU)) >> 16);
  sTime->Minutes = (uint8_t)((tmpreg & (RTC_TR_MNT | RTC_TR_MNU)) >>8);
  sTime->Seconds = (uint8_t)(tmpreg & (RTC_TR_ST | RTC_TR_SU));
  sTime->TimeFormat = (uint8_t)((tmpreg & (RTC_TR_PM)) >> 16);

  /* Check the input parameters format */
  if(Format == RTC_FORMAT_BIN)
  {
    /* Convert the time structure parameters to Binary format */
    sTime->Hours = (uint8_t)RTC_Bcd2ToByte(sTime->Hours);
    sTime->Minutes = (uint8_t)RTC_Bcd2ToByte(sTime->Minutes);
    sTime->Seconds = (uint8_t)RTC_Bcd2ToByte(sTime->Seconds);
  }

  return HAL_OK;
}

/**
  * @brief  Set RTC current date.
  * @param  hrtc: RTC handle
  * @param  sDate: Pointer to date structure
  * @param  Format: specifies the format of the entered parameters.
  *          This parameter can be one of the following values:
  *            @arg RTC_FORMAT_BIN: Binary data format
  *            @arg RTC_FORMAT_BCD: BCD data format
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_RTC_SetDate(RTC_HandleTypeDef *hrtc, RTC_DateTypeDef *sDate, uint32_t Format)
{
  uint32_t datetmpreg = 0;

 /* Check the parameters */
  assert_param(IS_RTC_FORMAT(Format));

 /* Process Locked */
 __HAL_LOCK(hrtc);

  hrtc->State = HAL_RTC_STATE_BUSY;

  if((Format == RTC_FORMAT_BIN) && ((sDate->Month & 0x10) == 0x10))
  {
    sDate->Month = (uint8_t)((sDate->Month & (uint8_t)~(0x10)) + (uint8_t)0x0A);
  }

  assert_param(IS_RTC_WEEKDAY(sDate->WeekDay));

  if(Format == RTC_FORMAT_BIN)
  {
    assert_param(IS_RTC_YEAR(sDate->Year));
    assert_param(IS_RTC_MONTH(sDate->Month));
    assert_param(IS_RTC_DATE(sDate->Date));

   datetmpreg = (((uint32_t)RTC_ByteToBcd2(sDate->Year) << 16) | \
                 ((uint32_t)RTC_ByteToBcd2(sDate->Month) << 8) | \
                 ((uint32_t)RTC_ByteToBcd2(sDate->Date)) | \
                 ((uint32_t)sDate->WeekDay << 13));
  }
  else
  {
    assert_param(IS_RTC_YEAR(RTC_Bcd2ToByte(sDate->Year)));
    datetmpreg = RTC_Bcd2ToByte(sDate->Month);
    assert_param(IS_RTC_MONTH(datetmpreg));
    datetmpreg = RTC_Bcd2ToByte(sDate->Date);
    assert_param(IS_RTC_DATE(datetmpreg));

    datetmpreg = ((((uint32_t)sDate->Year) << 16) | \
                  (((uint32_t)sDate->Month) << 8) | \
                  ((uint32_t)sDate->Date) | \
                  (((uint32_t)sDate->WeekDay) << 13));
  }

  /* Disable the write protection for RTC registers */
  __HAL_RTC_WRITEPROTECTION_DISABLE(hrtc);

  /* Set Initialization mode */
  if(RTC_EnterInitMode(hrtc) != HAL_OK)
  {
    /* Enable the write protection for RTC registers */
    __HAL_RTC_WRITEPROTECTION_ENABLE(hrtc);

    /* Set RTC state*/
    hrtc->State = HAL_RTC_STATE_ERROR;

    /* Process Unlocked */
    __HAL_UNLOCK(hrtc);

    return HAL_ERROR;
  }
  else
  {
    /* Set the RTC_DR register */
    hrtc->Instance->DR = (uint32_t)(datetmpreg & RTC_DR_RESERVED_MASK);

    /* Exit Initialization mode */
    hrtc->Instance->ISR &= ((uint32_t)~RTC_ISR_INIT);

    /* If  CR_BYPSHAD bit = 0, wait for synchro else this check is not needed */
    if((hrtc->Instance->CR & RTC_CR_BYPSHAD) == RESET)
    {
      if(HAL_RTC_WaitForSynchro(hrtc) != HAL_OK)
      {
        /* Enable the write protection for RTC registers */
        __HAL_RTC_WRITEPROTECTION_ENABLE(hrtc);

        hrtc->State = HAL_RTC_STATE_ERROR;

        /* Process Unlocked */
        __HAL_UNLOCK(hrtc);

        return HAL_ERROR;
      }
    }

    /* Enable the write protection for RTC registers */
    __HAL_RTC_WRITEPROTECTION_ENABLE(hrtc);

    hrtc->State = HAL_RTC_STATE_READY ;

    /* Process Unlocked */
    __HAL_UNLOCK(hrtc);

    return HAL_OK;
  }
}

/**
  * @brief  Get RTC current date.
  * @param  hrtc: RTC handle
  * @param  sDate: Pointer to Date structure
  * @param  Format: Specifies the format of the entered parameters.
  *          This parameter can be one of the following values:
  *            @arg RTC_FORMAT_BIN:  Binary data format
  *            @arg RTC_FORMAT_BCD:  BCD data format
  * @note  You must call HAL_RTC_GetDate() after HAL_RTC_GetTime() to unlock the values
  *        in the higher-order calendar shadow registers to ensure consistency between the time and date values.
  *        Reading RTC current time locks the values in calendar shadow registers until Current date is read.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_RTC_GetDate(RTC_HandleTypeDef *hrtc, RTC_DateTypeDef *sDate, uint32_t Format)
{
  uint32_t datetmpreg = 0;

  /* Check the parameters */
  assert_param(IS_RTC_FORMAT(Format));

  /* Get the DR register */
  datetmpreg = (uint32_t)(hrtc->Instance->DR & RTC_DR_RESERVED_MASK);

  /* Fill the structure fields with the read parameters */
  sDate->Year = (uint8_t)((datetmpreg & (RTC_DR_YT | RTC_DR_YU)) >> 16);
  sDate->Month = (uint8_t)((datetmpreg & (RTC_DR_MT | RTC_DR_MU)) >> 8);
  sDate->Date = (uint8_t)(datetmpreg & (RTC_DR_DT | RTC_DR_DU));
  sDate->WeekDay = (uint8_t)((datetmpreg & (RTC_DR_WDU)) >> 13);

  /* Check the input parameters format */
  if(Format == RTC_FORMAT_BIN)
  {
    /* Convert the date structure parameters to Binary format */
    sDate->Year = (uint8_t)RTC_Bcd2ToByte(sDate->Year);
    sDate->Month = (uint8_t)RTC_Bcd2ToByte(sDate->Month);
    sDate->Date = (uint8_t)RTC_Bcd2ToByte(sDate->Date);
  }
  return HAL_OK;
}

HAL_StatusTypeDef RTC_EnterInitMode(RTC_HandleTypeDef* hrtc)
{
  uint32_t tickstart = 0;

  /* Check if the Initialization mode is set */
  if((hrtc->Instance->ISR & RTC_ISR_INITF) == (uint32_t)RESET)
  {
    /* Set the Initialization mode */
    hrtc->Instance->ISR = (uint32_t)RTC_INIT_MASK;

    tickstart = HAL_GetTick();
    /* Wait till RTC is in INIT state and if Time out is reached exit */
    while((hrtc->Instance->ISR & RTC_ISR_INITF) == (uint32_t)RESET)
    {
      if((HAL_GetTick()  - tickstart ) > RTC_TIMEOUT_VALUE)
      {
        return HAL_TIMEOUT;
      }
    }
  }

  return HAL_OK;
}


//------------------- support functions ------------------------------------
HAL_StatusTypeDef HAL_RTC_WaitForSynchro(RTC_HandleTypeDef* hrtc)
{
  uint32_t tickstart = 0;

  /* Clear RSF flag */
  hrtc->Instance->ISR &= (uint32_t)RTC_RSF_MASK;

  tickstart = HAL_GetTick();

  /* Wait the registers to be synchronised */
  while((hrtc->Instance->ISR & RTC_ISR_RSF) == (uint32_t)RESET)
  {
    if((HAL_GetTick() - tickstart ) > RTC_TIMEOUT_VALUE)
    {
      return HAL_TIMEOUT;
    }
  }

  return HAL_OK;
}
uint8_t RTC_Bcd2ToByte(uint8_t Value)
{
  uint32_t tmp = 0;
  tmp = ((uint8_t)(Value & (uint8_t)0xF0) >> (uint8_t)0x4) * 10;
  return (tmp + (Value & (uint8_t)0x0F));
}
uint8_t RTC_ByteToBcd2(uint8_t Value)
{
  uint32_t bcdhigh = 0;

  while(Value >= 10)
  {
    bcdhigh++;
    Value -= 10;
  }

  return  ((uint8_t)(bcdhigh << 4) | Value);
}

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
