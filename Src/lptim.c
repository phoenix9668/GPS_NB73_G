/**
  ******************************************************************************
  * File Name          : LPTIM.c
  * Description        : This file provides code for the configuration
  *                      of the LPTIM instances.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "lptim.h"

/* USER CODE BEGIN 0 */
#include "gpio.h"
__IO ITStatus LptimReady = SET;
__IO ITStatus PregnantReady = RESET;
__IO ITStatus PowerOffReady = RESET;
__IO uint8_t pregnantTimeBase = 0x00;//one step == 128s
__IO uint8_t wakeupTimeBase = 0x00;//one step == 128s
__IO uint8_t PowerOffTimeBase = 0x00;//one step == 128s
__IO uint8_t WAKEUPTIME = 0x38;//384s,must be up to 2
/* USER CODE END 0 */

/* LPTIM1 init function */
void MX_LPTIM1_Init(void)
{

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_LPTIM1);

  /* LPTIM1 interrupt Init */
  NVIC_SetPriority(LPTIM1_IRQn, 1);
  NVIC_EnableIRQ(LPTIM1_IRQn);

  LL_LPTIM_SetClockSource(LPTIM1, LL_LPTIM_CLK_SOURCE_INTERNAL);
  LL_LPTIM_SetPrescaler(LPTIM1, LL_LPTIM_PRESCALER_DIV128);
  LL_LPTIM_SetPolarity(LPTIM1, LL_LPTIM_OUTPUT_POLARITY_REGULAR);
  LL_LPTIM_SetUpdateMode(LPTIM1, LL_LPTIM_UPDATE_MODE_IMMEDIATE);
  LL_LPTIM_SetCounterMode(LPTIM1, LL_LPTIM_COUNTER_MODE_INTERNAL);
  LL_LPTIM_TrigSw(LPTIM1);

}

/* USER CODE BEGIN 1 */
void LPTIM1_Counter_Start_IT(void)
{
  /* Enable the Autoreload match Interrupt */
  LL_LPTIM_EnableIT_ARRM(LPTIM1);
  /* Enable the LPTIM1 counter */
  LL_LPTIM_Enable(LPTIM1);

  /* Set the Autoreload value */
  LL_LPTIM_SetAutoReload(LPTIM1, 0x7FFF);
  
  /* Start the LPTIM counter in continuous mode */
  LL_LPTIM_StartCounter(LPTIM1, LL_LPTIM_OPERATING_MODE_CONTINUOUS);
}

/**
  * @brief  LPTimer Autoreload match interrupt processing
  * @param  None
  * @retval None
  */
void LPTimerAutoreloadMatch_Callback(void)
{
	pregnantTimeBase++;
	wakeupTimeBase++;
	PowerOffTimeBase++;
	if(pregnantTimeBase == 0x0A)//1280s
	{
		PregnantReady = SET;
		pregnantTimeBase = 0x00;
	}
	if(PowerOffTimeBase == 0x02)//256s
	{
		PowerOffReady = SET;
	}
	if(wakeupTimeBase == WAKEUPTIME)//2hours
	{
		LptimReady = SET;
		wakeupTimeBase = 0x00;
		PowerOffTimeBase = 0x00;
	}
}

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
