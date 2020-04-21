/**
  ******************************************************************************
  * File Name          : SPI.c
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

/* Includes ------------------------------------------------------------------*/
#include "spi.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* SPI1 init function */
void MX_SPI1_Init(void)
{
  LL_SPI_InitTypeDef SPI_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SPI1);
  
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
  /**SPI1 GPIO Configuration  
  PA5   ------> SPI1_SCK
  PA6   ------> SPI1_MISO
  PA7   ------> SPI1_MOSI 
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_5;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_0;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_6;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_0;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_7;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_0;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  SPI_InitStruct.TransferDirection = LL_SPI_FULL_DUPLEX;
  SPI_InitStruct.Mode = LL_SPI_MODE_MASTER;
  SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_8BIT;
  SPI_InitStruct.ClockPolarity = LL_SPI_POLARITY_LOW;
  SPI_InitStruct.ClockPhase = LL_SPI_PHASE_1EDGE;
  SPI_InitStruct.NSS = LL_SPI_NSS_SOFT;
  SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV2;
  SPI_InitStruct.BitOrder = LL_SPI_MSB_FIRST;
  SPI_InitStruct.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
  SPI_InitStruct.CRCPoly = 7;
  LL_SPI_Init(SPI1, &SPI_InitStruct);
  LL_SPI_SetStandard(SPI1, LL_SPI_PROTOCOL_MOTOROLA);

}

/* USER CODE BEGIN 1 */

/**
  * @brief  This function Activate SPI1
  * @param  None
  * @retval None
  */
void Activate_SPI(void)
{
  /* Enable SPI1 */
  LL_SPI_Enable(SPI1);
}

uint8_t SPI_ExchangeByte(uint8_t input)
{
	LL_SPI_TransmitData8(SPI1, input);

	while (!LL_SPI_IsActiveFlag_TXE(SPI1)){};
	while (!LL_SPI_IsActiveFlag_RXNE(SPI1)){};

	return (LL_SPI_ReceiveData8(SPI1));
}

// Function for sending and receiving data through SPI
void SpiFunction(uint8_t *OutputBuff,uint8_t *InputBuff, uint16_t OutNoOfBytes, uint16_t InNoOfBytes)
{
	uint16_t i;

	for(i=0;i<OutNoOfBytes;i++)
	{
    SPI_ExchangeByte(OutputBuff[i]);					// Send data
	}
   
	for(i=0;i<InNoOfBytes;i++)
	{
		InputBuff[i] = SPI_ExchangeByte(0xFF);		// Receive data
	}
}

/*******************************************************************
  @brief unsigned char ADXL362RegisterRead(unsigned char Address)
         Read a register value from ADXL362         
  @param
         unsigned char Address:       Register address
  @return   
         unsigned int  ReceiveValue:  read register value from ADXL362
*******************************************************************/
uint8_t ADXL362RegisterRead(uint8_t Address)
{   
    uint8_t SendTemp[2];
		uint8_t	ReceiveTemp[2];
		uint8_t ReceiveValue;
 

    SendTemp[0] = 0x0B;                 //0x0B: read register command
    SendTemp[1] = Address;              //address byte
		CS_OFF();
		SpiFunction(SendTemp, ReceiveTemp, 2, 1);
		ReceiveValue = ReceiveTemp[0];
		CS_ON();
    return(ReceiveValue);
}

/*******************************************************************
  @brief void ADXL362RegisterWrite(unsigned char Address, unsigned char SendValue)
         send SPI command to ADXL362
  @param
         unsigned char Address:       Register address
         unsigned char SendValue:     Value written to ADXL362 register
  @return   
         none
*******************************************************************/
void ADXL362RegisterWrite(uint8_t Address, uint8_t SendValue)
{    
    uint8_t SendTemp[3];
		uint8_t ReceiveTemp[3];
    
    SendTemp[0] = 0x0A;                 //0x0A: write register
    SendTemp[1] = Address;              //address byte
    SendTemp[2] = SendValue;
    
		CS_OFF();
		SpiFunction(SendTemp, ReceiveTemp, 3, 0);
		CS_ON();
}

/*******************************************************************
  @brief void ADXL362BurstRead(unsigned char Address, unsigned char NumberofRegisters, unsigned char *RegisterData)
         Multibyte read from ADXL362
  @param
         unsigned char Address:           Register address
         unsigned char NumberofRegisters: Register numbers to be read
         unsigned char *RegisterData:     Buffer save the read value
  @return   
         none  
*******************************************************************/
void ADXL362BurstRead(uint16_t Address, uint16_t NumberofRegisters, uint8_t *RegisterData)
{    
    unsigned char SendTemp[2];

    SendTemp[0] = 0x0B;            	//0x0B: read register
    SendTemp[1] = Address;         	//address byte
		CS_ON();
		SpiFunction(SendTemp, RegisterData, 2, NumberofRegisters);
		CS_OFF();
}

/*******************************************************************
  @brief void ADXL362BurstWrite(unsigned char Address, unsigned char NumberofRegisters, unsigned char *RegisterData)
         Multibyte write to ADXL362
  @param
         unsigned char Address:           Register address
         unsigned char NumberofRegisters: Register numbers to be written
         unsigned char *RegisterData:     Buffer save the written value
  @return   
         none 
*******************************************************************/
void ADXL362BurstWrite(uint16_t Address, uint16_t NumberofRegisters, uint8_t *RegisterData)
{ 
    uint8_t SendTemp[512];
    uint8_t ReceiveTemp[2];
    uint16_t RegisterIndex;
  
    SendTemp[0] = 0x0A;                 //0x0A: write register
    SendTemp[1] = Address;              //address byte
    for (RegisterIndex=0; RegisterIndex<NumberofRegisters; RegisterIndex++)
    {
        SendTemp[2+RegisterIndex] = *(RegisterData + RegisterIndex);
    }
		CS_ON();
		SpiFunction(SendTemp, ReceiveTemp, (2+NumberofRegisters), 0);
		CS_OFF();

}

/*******************************************************************
  @brief void ADXL362FifoRead(unsigned char NumberofRegisters, unsigned char *RegisterData)
         Multibyte read from ADXL362 FIFO
  @param
         unsigned char NumberofRegisters: Register numbers to be read
         unsigned char *RegisterData:     Buffer save the read value
  @return
         none
*******************************************************************/
void ADXL362FifoRead(uint16_t NumberofRegisters, uint8_t *RegisterData)
{
		uint8_t SendTemp[1];
		SendTemp[0] = 0x0D;            	//0x0D: read register

		CS_ON();
    SpiFunction(SendTemp, RegisterData, 1, NumberofRegisters);
		CS_OFF();
}


/*******************************************************************
  @brief void ADXL362_Init(void)
         initial and configure ADXL362
  @param
				 none
  @return
         none
*******************************************************************/
void ADXL362_Init(void)
{
		ADXL362RegisterWrite(XL362_SOFT_RESET,0x52);   						// software reset
		LL_mDelay(2000);
		ADXL362RegisterWrite(XL362_THRESH_ACT_L,0x5E);						//set active threshold equip 350mg
		ADXL362RegisterWrite(XL362_THRESH_ACT_H,0x01);
		ADXL362RegisterWrite(XL362_TIME_ACT,0x01);								//set inactive time equip 1/12.5s
		ADXL362RegisterWrite(XL362_THRESH_INACT_L,0x96);					//set inactive threshold equip 150mg
		ADXL362RegisterWrite(XL362_THRESH_INACT_H,0x00);
		ADXL362RegisterWrite(XL362_TIME_INACT_L,0x03);						//set inactive time equip 2/12.5s
		ADXL362RegisterWrite(XL362_TIME_INACT_H,0x00);
		ADXL362RegisterWrite(XL362_ACT_INACT_CTL,0x3F);						//configure loop mode,enable active and inactive
		ADXL362RegisterWrite(XL362_INTMAP1,0x40);									//configure awake map INT1
		ADXL362RegisterWrite(XL362_INTMAP2,0x00);									//configure awake map INT2
		ADXL362RegisterWrite(XL362_FIFO_CONTROL,0x00);						//select fifo stream mode//0x0E
		ADXL362RegisterWrite(XL362_FIFO_SAMPLES,0x00);						//select fifo sample number//0xFF
		ADXL362RegisterWrite(XL362_FILTER_CTL,0x50);             	//select 4g range,ODR:12.5Hz
    //any changes to the registers before the POWER_CTL register (Register 0x00 to Register 0x2C) should be made with the device in standby
    ADXL362RegisterWrite(XL362_POWER_CTL,0x02);              	//select measurement mode
}

/*******************************************************************
  @brief void ADXL362_ReInit(void)
         reinitial and configure ADXL362
  @param
				 none
  @return
         none
*******************************************************************/
void ADXL362_ReInit(uint8_t thresh_act_h, uint8_t thresh_act_l, uint8_t thresh_inact_h, uint8_t thresh_inact_l, uint8_t time_inact_h, uint8_t time_inact_l, uint8_t filter_ctl)
{
		ADXL362RegisterWrite(XL362_SOFT_RESET,0x52);   						// software reset
		LL_mDelay(2000);
		ADXL362RegisterWrite(XL362_THRESH_ACT_L,thresh_act_l);						//set active threshold equip 350mg
		ADXL362RegisterWrite(XL362_THRESH_ACT_H,thresh_act_h);
		ADXL362RegisterWrite(XL362_THRESH_INACT_L,thresh_inact_l);					//set inactive threshold equip 150mg
		ADXL362RegisterWrite(XL362_THRESH_INACT_H,thresh_inact_h);
		ADXL362RegisterWrite(XL362_TIME_INACT_L,time_inact_l);						//set inactive time equip 1/12.5s
		ADXL362RegisterWrite(XL362_TIME_INACT_H,time_inact_h);
		ADXL362RegisterWrite(XL362_ACT_INACT_CTL,0x3F);						//configure loop mode,enable active and inactive
		ADXL362RegisterWrite(XL362_INTMAP1,0x40);									//configure awake map INT1
		ADXL362RegisterWrite(XL362_INTMAP2,0x00);									//configure awake map INT2
		ADXL362RegisterWrite(XL362_FIFO_CONTROL,0x00);						//select fifo stream mode//0x0E
		ADXL362RegisterWrite(XL362_FIFO_SAMPLES,0x00);						//select fifo sample number//0xFF
		ADXL362RegisterWrite(XL362_FILTER_CTL,filter_ctl);             	//select 2g range,ODR:12.5Hz
    //any changes to the registers before the POWER_CTL register (Register 0x00 to Register 0x2C) should be made with the device in standby
    ADXL362RegisterWrite(XL362_POWER_CTL,0x02);              	//select measurement mode
}

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
