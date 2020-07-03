/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l0xx_hal.h"
#include "stm32l0xx_ll_iwdg.h"
#include "stm32l0xx_ll_lptim.h"
#include "stm32l0xx_ll_lpuart.h"
#include "stm32l0xx_ll_rcc.h"
#include "stm32l0xx_hal.h"
#include "stm32l0xx_ll_spi.h"
#include "stm32l0xx.h"
#include "stm32l0xx_ll_system.h"
#include "stm32l0xx_ll_gpio.h"
#include "stm32l0xx_ll_exti.h"
#include "stm32l0xx_ll_bus.h"
#include "stm32l0xx_ll_cortex.h"
#include "stm32l0xx_ll_utils.h"
#include "stm32l0xx_ll_pwr.h"
#include "stm32l0xx_ll_dma.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
/* Size of Trasmission buffer */
#define TXBUFFERSIZE                      2*RXBUFFERSIZE
#define TxPREGNANTBUFFERSIZE              2*PREGNANTBUFFERSIZE
#define TxALLGNSSBUFFERSIZE              	2*ALLGNSSBUFFERSIZE
/* Size of Reception buffer */
#define RXBUFFERSIZE                      53
#define PREGNANTBUFFERSIZE                74
#define PREGNANTBUFFERFLAGLENGTH					35
#define GNSSBUFFERSIZE                		22
#define ALLGNSSBUFFERSIZE               	23
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define CS_Pin LL_GPIO_PIN_4
#define CS_GPIO_Port GPIOA
#define INT1_EXTI0_Pin LL_GPIO_PIN_0
#define INT1_EXTI0_GPIO_Port GPIOB
#define INT1_EXTI0_EXTI_IRQn EXTI0_1_IRQn
#define REL_EXTI2_Pin LL_GPIO_PIN_2
#define REL_EXTI2_GPIO_Port GPIOB
#define REL_EXTI2_EXTI_IRQn EXTI2_3_IRQn
#define GPS_CHARGE_Pin LL_GPIO_PIN_12
#define GPS_CHARGE_GPIO_Port GPIOB
#define U_Reload_Pin LL_GPIO_PIN_13
#define U_Reload_GPIO_Port GPIOB
#define U_RESET_Pin LL_GPIO_PIN_14
#define U_RESET_GPIO_Port GPIOB
#define LED_CHARGE_Pin LL_GPIO_PIN_15
#define LED_CHARGE_GPIO_Port GPIOA
#define LED_GREEN_Pin LL_GPIO_PIN_5
#define LED_GREEN_GPIO_Port GPIOB
#define LED_RED_Pin LL_GPIO_PIN_9
#define LED_RED_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
void Bufferchg(uint8_t* RxBuffer, uint8_t* TxBuffer, uint16_t RxBuffersize, uint16_t TxBuffersize);
void Convert_ASCII(uint8_t* RxBuffer, uint8_t* TxBuffer, uint16_t RxBuffersize);
void EEPROM_WRITE(uint16_t BiasAddress, uint8_t *Data, uint8_t len);
void EEPROM_READ(uint16_t BiasAddress, uint8_t *Buffer, uint8_t Len);
void LEDRed_Blinking(uint32_t Period);
void LEDGreen_Blinking(uint32_t Period);
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
