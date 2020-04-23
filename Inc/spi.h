/**
  ******************************************************************************
  * File Name          : SPI.h
  * Description        : This file provides code for the configuration
  *                      of the SPI instances.
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
#ifndef __spi_H
#define __spi_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include "gpio.h"

/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */

/* ------- Register names ------- */

#define XL362_DEVID_AD			  0x00
#define XL362_DEVID_MST			  0x01
#define XL362_PARTID			    0x02
#define XL362_REVID			  	  0x03
#define XL362_XDATA				    0x08
#define XL362_YDATA				    0x09
#define XL362_ZDATA				    0x0A
#define XL362_STATUS			    0x0B
#define XL362_FIFO_ENTRIES_L	0x0C
#define XL362_FIFO_ENTRIES_H	0x0D
#define XL362_XDATA_L			    0x0E
#define XL362_XDATA_H			    0x0F
#define XL362_YDATA_L			    0x10
#define XL362_YDATA_H			    0x11
#define XL362_ZDATA_L			    0x12
#define XL362_ZDATA_H			    0x13
#define XL362_TEMP_L			    0x14
#define XL362_TEMP_H			    0x15
#define XL362_SOFT_RESET		  0x1F
#define XL362_THRESH_ACT_L		0x20
#define XL362_THRESH_ACT_H		0x21
#define XL362_TIME_ACT			  0x22
#define XL362_THRESH_INACT_L	0x23
#define XL362_THRESH_INACT_H	0x24
#define XL362_TIME_INACT_L		0x25
#define XL362_TIME_INACT_H		0x26
#define XL362_ACT_INACT_CTL		0x27
#define XL362_FIFO_CONTROL		0x28
#define XL362_FIFO_SAMPLES		0x29
#define XL362_INTMAP1			    0x2A
#define XL362_INTMAP2			    0x2B
#define XL362_FILTER_CTL		  0x2C
#define XL362_POWER_CTL			  0x2D
#define XL362_SELF_TEST			  0x2E

/* USER CODE END Private defines */

void MX_SPI1_Init(void);

/* USER CODE BEGIN Prototypes */
void MX_SPI1_DeInit(void);
void Activate_SPI(void);
uint8_t SPI_ExchangeByte(uint8_t input);
void SpiFunction(uint8_t *OutputBuff,uint8_t *InputBuff, uint16_t OutNoOfBytes, uint16_t InNoOfBytes);
uint8_t ADXL362RegisterRead(uint8_t Address);
void ADXL362RegisterWrite(uint8_t Address, uint8_t SendValue);
void ADXL362BurstRead(uint16_t Address, uint16_t NumberofRegisters, uint8_t *RegisterData);
void ADXL362BurstWrite(uint16_t Address, uint16_t NumberofRegisters, uint8_t *RegisterData);
void ADXL362FifoRead(uint16_t NumberofRegisters, uint8_t *RegisterData);
void ADXL362_Init(void);
void ADXL362_ReInit(uint8_t thresh_act_h, uint8_t thresh_act_l, uint8_t thresh_inact_h, uint8_t thresh_inact_l, uint8_t time_inact_h, uint8_t time_inact_l, uint8_t filter_ctl);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ spi_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
