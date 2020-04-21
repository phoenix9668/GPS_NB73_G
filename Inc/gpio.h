/**
  ******************************************************************************
  * File Name          : gpio.h
  * Description        : This file contains all the functions prototypes for 
  *                      the gpio  
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __gpio_H
#define __gpio_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */

#define LED_CHARGE_ON()					LL_GPIO_SetOutputPin(LED_CHARGE_GPIO_Port, LED_CHARGE_Pin);
#define LED_CHARGE_OFF()				LL_GPIO_ResetOutputPin(LED_CHARGE_GPIO_Port, LED_CHARGE_Pin);
	 
#define GPS_CHARGE_ON()					LL_GPIO_SetOutputPin(GPIOB, GPS_CHARGE_Pin);
#define GPS_CHARGE_OFF()				LL_GPIO_ResetOutputPin(GPIOB, GPS_CHARGE_Pin);

#define U_Reload_ON()						LL_GPIO_SetOutputPin(GPIOB, U_Reload_Pin);
#define U_Reload_OFF()					LL_GPIO_ResetOutputPin(GPIOB, U_Reload_Pin);

#define U_RESET_ON()						LL_GPIO_SetOutputPin(GPIOB, U_RESET_Pin);
#define U_RESET_OFF()						LL_GPIO_ResetOutputPin(GPIOB, U_RESET_Pin);

#define LED_GREEN_ON()					LL_GPIO_SetOutputPin(GPIOB, LED_GREEN_Pin);
#define LED_GREEN_OFF()					LL_GPIO_ResetOutputPin(GPIOB, LED_GREEN_Pin);
#define LED_GREEN_TOG()					LL_GPIO_TogglePin(GPIOB, LED_GREEN_Pin);

#define LED_RED_ON()						LL_GPIO_SetOutputPin(GPIOB, LED_RED_Pin);
#define LED_RED_OFF()						LL_GPIO_ResetOutputPin(GPIOB, LED_RED_Pin);
#define LED_RED_TOG()						LL_GPIO_TogglePin(GPIOB, LED_RED_Pin);

#define CS_ON()									LL_GPIO_SetOutputPin(CS_GPIO_Port, CS_Pin);
#define CS_OFF()								LL_GPIO_ResetOutputPin(CS_GPIO_Port, CS_Pin);
  

/* USER CODE END Private defines */

void MX_GPIO_Init(void);

/* USER CODE BEGIN Prototypes */
void REL_EXTI_Init(void);
void REL_EXTI_DisInit(void);
void Int1_Callback(void);
void REL_Callback(void);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ pinoutConfig_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
