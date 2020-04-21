/**
  ******************************************************************************
  * File Name          : USART.c
  * Description        : This file provides code for the configuration
  *                      of the USART instances.
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
#include "usart.h"

/* USER CODE BEGIN 0 */
#include "gpio.h"
extern uint8_t RxBuffer[RXBUFFERSIZE];
__IO uint8_t RxCounter = 0;
extern __IO uint8_t i;
/* USER CODE END 0 */

/* LPUART1 init function */

void MX_LPUART1_UART_Init(void)
{
  LL_LPUART_InitTypeDef LPUART_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_LPUART1);
  
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOB);
  /**LPUART1 GPIO Configuration  
  PB10   ------> LPUART1_TX
  PB11   ------> LPUART1_RX 
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_10;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_6;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_11;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_6;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* LPUART1 interrupt Init */
  NVIC_SetPriority(LPUART1_IRQn, 0);
  NVIC_EnableIRQ(LPUART1_IRQn);

  LPUART_InitStruct.BaudRate = 9600;
  LPUART_InitStruct.DataWidth = LL_LPUART_DATAWIDTH_8B;
  LPUART_InitStruct.StopBits = LL_LPUART_STOPBITS_1;
  LPUART_InitStruct.Parity = LL_LPUART_PARITY_NONE;
  LPUART_InitStruct.TransferDirection = LL_LPUART_DIRECTION_TX_RX;
  LPUART_InitStruct.HardwareFlowControl = LL_LPUART_HWCONTROL_NONE;
  LL_LPUART_Init(LPUART1, &LPUART_InitStruct);
  LL_LPUART_EnableOverrunDetect(LPUART1);
  LL_LPUART_EnableDMADeactOnRxErr(LPUART1);

}

/* USER CODE BEGIN 1 */

/**
  * @brief  This function Activate LPUART1
  * @param  None
  * @retval None
  */
void Activate_LPUART1(void)
{
  /* Enable LPUART1 */
	LL_LPUART_Enable(LPUART1);
	
	/* Polling USART initialisation */
  while((!(LL_LPUART_IsActiveFlag_TEACK(LPUART1))) || (!(LL_LPUART_IsActiveFlag_REACK(LPUART1))))
  {}
}

/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
int fputc(int ch,FILE *f)
{
	LPUART1->TDR = ch;
	while(!(LPUART1->ISR&USART_ISR_TXE))
	{;}
	return ch;
}

/**
  * @brief  Send Txt information message on USART Tx line (to PC Com port).
  * @param  None
  * @retval None
  */
void PrintInfo(uint8_t *String, uint32_t Size)
{
  uint32_t index = 0;
  uint8_t *pchar = String;
  
  /* Send characters one per one, until last char to be sent */
  for (index = 0; index < Size; index++)
  {
    /* Wait for TXE flag to be raised */
    while (!LL_LPUART_IsActiveFlag_TXE(LPUART1))
    {
    }

    /* Write character in Transmit Data register.
       TXE flag is cleared by writing data in TDR register */
    LL_LPUART_TransmitData8(LPUART1, *pchar++);
  }

  /* Wait for TC flag to be raised for last char */
  while (!LL_LPUART_IsActiveFlag_TC(LPUART1))
  {
  }
}



/**
  * @brief  Function called from USART IRQ Handler when RXNE flag is set
  *         Function is in charge of reading character received on USART RX line.
  * @param  None
  * @retval None
  */
void LPUART_CharReception_Callback(void)
{
  /* Read Received character. RXNE flag is cleared by reading of RDR register */
  RxBuffer[RxCounter++] = LL_LPUART_ReceiveData8(LPUART1);
  /* Echo received character on TX */
//  LL_LPUART_TransmitData8(LPUART1, RxBuffer[RxCounter-1]);
	/* Check if received value is corresponding to specific one : S or s */
  if ((RxBuffer[RxCounter-1] == 0xCB) || (RxBuffer[RxCounter-1] == 0x0A))
  {
    /* Clear RxCounter : Expected character has been received */
    RxCounter = 0x00;
//		for (i=0; i<RXBUFFERSIZE; i++) //clear array
//		{	RxBuffer[i] = 0; }
  }
}

/**
  * @brief  Function called in case of error detected in USART IT Handler
  * @param  None
  * @retval None
  */
void Error_Callback(void)
{
  __IO uint32_t isr_reg;

  /* Disable USARTx_IRQn */
  NVIC_DisableIRQ(LPUART1_IRQn);
  
  /* Error handling example :
    - Read USART ISR register to identify flag that leads to IT raising
    - Perform corresponding error handling treatment according to flag
  */
  isr_reg = LL_LPUART_ReadReg(LPUART1, ISR);
  if (isr_reg & LL_LPUART_ISR_NE)
  {
    /* case Noise Error flag is raised : ... */
			LL_LPUART_ClearFlag_NE(LPUART1);
			LL_LPUART_DeInit(LPUART1);
			MX_LPUART1_UART_Init();
			LL_LPUART_EnableIT_RXNE(LPUART1);
			LL_LPUART_EnableIT_ERROR(LPUART1);
			Activate_LPUART1();
			RxCounter = 0x00;
//			LEDRed_Blinking(200);
  }
	else if (isr_reg & LL_LPUART_ISR_FE)
  {
    /* case Noise Error flag is raised : ... */
			LL_LPUART_ClearFlag_FE(LPUART1);
			LL_LPUART_DeInit(LPUART1);
			MX_LPUART1_UART_Init();
			LL_LPUART_EnableIT_RXNE(LPUART1);
			LL_LPUART_EnableIT_ERROR(LPUART1);
			Activate_LPUART1();
			RxCounter = 0x00;
//			LEDRed_Blinking(500);
  }
	else if (isr_reg & LL_LPUART_ISR_ORE)
  {
    /* case Noise Error flag is raised : ... */
			LL_LPUART_ClearFlag_ORE(LPUART1);
			LL_LPUART_DeInit(LPUART1);
			MX_LPUART1_UART_Init();
			LL_LPUART_EnableIT_RXNE(LPUART1);
			LL_LPUART_EnableIT_ERROR(LPUART1);
			Activate_LPUART1();
			RxCounter = 0x00;
//			LEDRed_Blinking(1000);
  }
	else if (isr_reg & LL_LPUART_ISR_PE)
  {
    /* case Noise Error flag is raised : ... */
			LL_LPUART_ClearFlag_PE(LPUART1);
			LL_LPUART_DeInit(LPUART1);
			MX_LPUART1_UART_Init();
			LL_LPUART_EnableIT_RXNE(LPUART1);
			LL_LPUART_EnableIT_ERROR(LPUART1);
			Activate_LPUART1();
			RxCounter = 0x00;
  }
  else
  {
    /* Unexpected IT source : Set LED to Blinking mode to indicate error occurs */
			LL_LPUART_DeInit(LPUART1);
			MX_LPUART1_UART_Init();
			LL_LPUART_EnableIT_RXNE(LPUART1);
			LL_LPUART_EnableIT_ERROR(LPUART1);
			Activate_LPUART1();
			RxCounter = 0x00;
//			LEDRed_Blinking(2000);
  }
}

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
