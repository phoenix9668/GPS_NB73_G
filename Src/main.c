/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "iwdg.h"
#include "lptim.h"
#include "usart.h"
#include "rtc.h"
#include "spi.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
uint8_t aTextInfoStart1[] = "NB-GPS module transfer program.\r\n";
uint8_t aTextInfoStart2[] = "using LPUART1,configuration:9600 8-N-1\r\n";
uint8_t alarmInfo[] = "03tamper alarm";
uint8_t ConnectedInfo[] = "04Connected";
uint8_t OKInfo[] = {0x30,0x36,0x00,0x01,0x00,0x4F,0x4B};//acknowledgeinfo = messageId(0x3036)+mid(0x0001)+errcode(0x00 is success,0x01 is fail)+acknowledge(0x4F4B is OK)
uint8_t ERInfo[] = {0x30,0x36,0x00,0x01,0x01,0x45,0x52};//acknowledgeinfo = messageId(0x3036)+mid(0x0001)+errcode(0x00 is success,0x01 is fail)+acknowledge(0x4552 is ER)
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
__IO uint8_t i;
uint8_t Pregnant_Buffer[PREGNANTBUFFERSIZE]={2,0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38,39,40,41,42,43,44,45,46,47,48,49,50,51,52,53,54,55,56,57,58,59,60,61,62,63,64,65,66,67,68,69,70,71,72};
uint8_t Pregnant_TxBuffer[TxPREGNANTBUFFERSIZE];
uint8_t RxBuffer[RXBUFFERSIZE];
uint8_t TxTempBuffer[TxPREGNANTBUFFERSIZE];
uint8_t TxBuffer[TXBUFFERSIZE];
extern __IO uint32_t step;
extern __IO ITStatus LptimReady;
extern __IO ITStatus PowerOffReady;
extern __IO ITStatus PregnantReady;
extern __IO uint8_t wakeupTimeBase;
extern __IO uint8_t WAKEUPTIME;
extern __IO uint8_t RxCounter;
extern __IO ITStatus AlarmReady;
__IO uint8_t Pregnant_Buffer_Flag = 0;
__IO FlagStatus SendState = RESET;
__IO FlagStatus AlarmState = RESET;
__IO FlagStatus ConnectedState = RESET;
extern __IO FlagStatus CommandState;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DEBUG
#define EEPROM_BASE_ADDR		0x08080000   /* Start @ of user eeprom area */
#define EEPROM_BYTE_SIZE		0xFF
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void SystemPower_Config(void);
static void Show_Message(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
  

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_LPUART1_UART_Init();
  MX_LPTIM1_Init();
  MX_RTC_Init();
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */
	REL_EXTI_DisInit();
	Activate_SPI();
	Activate_LPUART1();
	LPTIM1_Counter_Start_IT();
	ADXL362_Init();
	LED_CHARGE_ON();
	GPS_CHARGE_ON();
	LL_mDelay(12000);
	Show_Message();
	LL_mDelay(2000);
  /* Enable RXNE and Error interrupts */
  LL_LPUART_EnableIT_RXNE(LPUART1);
	LL_LPUART_EnableIT_ERROR(LPUART1);
	
//	EEPROM_READ(0,Pregnant_Buffer,PREGNANTBUFFERSIZE);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		/* Refresh IWDG down-counter to default value */
		LL_IWDG_ReloadCounter(IWDG);
		
		if(PregnantReady == SET)
		{
			if(Pregnant_Buffer_Flag == PREGNANTBUFFERFLAGLENGTH)
			{
				Pregnant_Buffer_Flag = 0;
			}
			else
			{
				Pregnant_Buffer_Flag++;
			}
			
			*(Pregnant_Buffer+Pregnant_Buffer_Flag*2+2) = (uint8_t)(0x000000FF & step>>8);
			*(Pregnant_Buffer+Pregnant_Buffer_Flag*2+3) = (uint8_t)(0x000000FF & step);
			Pregnant_Buffer[0] = 0x02;
			Pregnant_Buffer[1] = Pregnant_Buffer_Flag;
			
			EEPROM_WRITE(0,Pregnant_Buffer,PREGNANTBUFFERSIZE);
			step = 0;
			PregnantReady = RESET;
		}
		
		if(LptimReady == SET)
		{
			REL_EXTI_DisInit();
			LED_CHARGE_ON();
			GPS_CHARGE_ON();
			SendState = SET;
			LptimReady = RESET;
			RxCounter = 0x00;
		}
		
		if(AlarmReady == SET)
		{
			REL_EXTI_DisInit();
			LED_CHARGE_ON();
			GPS_CHARGE_ON();
			AlarmState = SET;
			AlarmReady = RESET;
			RxCounter = 0x00;
		}
		
		if(PowerOffReady == SET)
		{
			SendState = RESET;
			AlarmState = RESET;
			ConnectedState = RESET;
			PowerOffReady = RESET;
		
		if(RxBuffer[0] == 0x43 && RxBuffer[1] == 0x6F && RxBuffer[2] == 0x6E && RxBuffer[3] == 0x6E && RxBuffer[4] == 0x65 && RxBuffer[5] == 0x63 && RxBuffer[6] == 0x74 && RxBuffer[7] == 0x65 && RxBuffer[8] == 0x64)
		{
			if(SendState == SET)
			{
				if(ConnectedState == RESET)
				{
					LL_mDelay(2000);
					PrintInfo((uint8_t*)ConnectedInfo, sizeof(ConnectedInfo));
					LL_mDelay(100);
					Bufferchg((uint8_t*)Pregnant_Buffer, (uint8_t*)Pregnant_TxBuffer, PREGNANTBUFFERSIZE, TxPREGNANTBUFFERSIZE);
					PrintInfo((uint8_t*)Pregnant_TxBuffer, TxPREGNANTBUFFERSIZE);
					ConnectedState = SET;
				}
				for (i=0; i<RXBUFFERSIZE; i++) //clear array
				{	RxBuffer[i] = 0; }
				RxCounter = 0x00;
			}
		}
		
		if(RxBuffer[0] == 0x30 && RxBuffer[1] == 0x35)
		{
			if(CommandState == SET)
			{
				if(RxBuffer[4] == 0x41 && RxBuffer[5] == 0x41 && RxBuffer[6] == 0x43 && RxBuffer[7] == 0x43 && RxBuffer[8] == 0x30 && RxBuffer[9] == 0x31 && RxBuffer[10] >= 0x03)
				{
					WAKEUPTIME = RxBuffer[10];
					wakeupTimeBase = 0x00;
					OKInfo[2] = RxBuffer[2];
					OKInfo[3] = RxBuffer[3];
					PrintInfo((uint8_t*)OKInfo, sizeof(OKInfo));
				}
				else
				{
					ERInfo[2] = RxBuffer[2];
					ERInfo[3] = RxBuffer[3];
					PrintInfo((uint8_t*)ERInfo, sizeof(ERInfo));
				}
				CommandState = RESET;
			}
		}
		
		if(RxBuffer[0] == 0x01 && RxBuffer[1] == 0x46 && RxBuffer[2] == 0x00 && RxBuffer[3] == 0x00 && RxBuffer[4] == 0x00 && RxBuffer[5] == 0x16 && RxBuffer[6] == 0x2C)
		{
			LL_mDelay(50);
			Bufferchg((uint8_t*)RxBuffer, (uint8_t*)TxBuffer, RXBUFFERSIZE, TXBUFFERSIZE);
			PrintInfo((uint8_t*)TxBuffer, TXBUFFERSIZE);
			SendState = RESET;
			if(AlarmState == SET)
			{
				LL_mDelay(200);
				PrintInfo((uint8_t*)alarmInfo, sizeof(alarmInfo));
				AlarmState = RESET;
			}
			for (i=0; i<RXBUFFERSIZE; i++) //clear array
			{	RxBuffer[i] = 0; }
			RxCounter = 0x00;
			ConnectedState = RESET;
		}
		
		/* Refresh IWDG down-counter to default value */
		LL_IWDG_ReloadCounter(IWDG);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	
		if(PregnantReady == RESET && LptimReady == RESET && AlarmReady == RESET && SendState == RESET && AlarmState == RESET)
		{
			MX_LPUART1_UART_DeInit();
			MX_SPI1_DeInit();
			SystemPower_Config();
			/* Enter Stop Mode */
			HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
			
			SystemClock_Config();
			LED_CHARGE_OFF();
			GPS_CHARGE_OFF();
			REL_EXTI_Init();
			MX_SPI1_Init();
			MX_LPUART1_UART_Init();
			LL_LPUART_EnableIT_RXNE(LPUART1);
			LL_LPUART_EnableIT_ERROR(LPUART1);
			Activate_SPI();
			Activate_LPUART1();
			for (i=0; i<RXBUFFERSIZE; i++) //clear array
			{	RxBuffer[i] = 0; }
			RxCounter = 0x00;
//			LED_RED_TOG();
		}
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  /** Configure LSE Drive Capability 
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_LSE
                              |RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_5;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_LPUART1|RCC_PERIPHCLK_RTC
                              |RCC_PERIPHCLK_LPTIM1;
  PeriphClkInit.Lpuart1ClockSelection = RCC_LPUART1CLKSOURCE_LSE;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  PeriphClkInit.LptimClockSelection = RCC_LPTIM1CLKSOURCE_LSE;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/**
  * @brief  System Power Configuration
  *         The system Power is configured as follow :
  *            + Regulator in LP mode
  *            + VREFINT OFF, with fast wakeup enabled
  *            + HSI as SysClk after Wake Up
  *            + No IWDG
  *            + Automatic Wakeup using RTC clocked by LSI (after ~4s)
  * @param  None
  * @retval None
  */
static void SystemPower_Config(void)
{
	/* Disable Wakeup Counter */
	HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);
	/* ## Setting the Wake up time ############################################*/
	/*  RTC Wakeup Interrupt Generation:
			Wakeup Time Base = (RTC_WAKEUPCLOCK_RTCCLK_DIV /(LSE or LSI))
			Wakeup Time = Wakeup Time Base * WakeUpCounter 
			= (RTC_WAKEUPCLOCK_RTCCLK_DIV /(LSE or LSI)) * WakeUpCounter
				==> WakeUpCounter = Wakeup Time / Wakeup Time Base
    
			To configure the wake up timer to 10s the WakeUpCounter is set to 0x5000 for LSE:
			RTC_WAKEUPCLOCK_RTCCLK_DIV = RTCCLK_Div16 = 16 
			Wakeup Time Base = 16 /(32.768KHz) = 0.48828125 ms
			Wakeup Time = 10s = 0.48828125ms  * WakeUpCounter
				==> WakeUpCounter = 10s/0.48828125ms = 20480 = 0x5000 */
	HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 0x5000, RTC_WAKEUPCLOCK_RTCCLK_DIV16);
	
  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();

  /* Enable Ultra low power mode */
  HAL_PWREx_EnableUltraLowPower();

  /* Enable the fast wake up from Ultra low power mode */
  HAL_PWREx_EnableFastWakeUp();
}

static void Show_Message(void)
{
	uint8_t ReadValueTemp;
	#ifdef DEBUG
		PrintInfo(aTextInfoStart1, sizeof(aTextInfoStart1));
		PrintInfo(aTextInfoStart2, sizeof(aTextInfoStart2));
	#endif
		ReadValueTemp = ADXL362RegisterRead(XL362_DEVID_AD);     	//Analog Devices device ID, 0xAD
		if(ReadValueTemp == 0xAD)
		{
			LED_GREEN_ON();
			LL_mDelay(1000);
		}
	#ifdef DEBUG
		printf("Analog Devices device ID: %x\n",ReadValueTemp);	 	//send via UART
	#endif
		ReadValueTemp = ADXL362RegisterRead(XL362_DEVID_MST);    	//Analog Devices MEMS device ID, 0x1D
		LED_GREEN_OFF();
		LL_mDelay(1000);
	#ifdef DEBUG
		printf("Analog Devices MEMS device ID: %x\n",ReadValueTemp);	//send via UART
	#endif
		ReadValueTemp = ADXL362RegisterRead(XL362_PARTID);       	//part ID, 0xF2
		if(ReadValueTemp == 0xF2)
		{
			LED_GREEN_ON();
			LL_mDelay(1000);
		}
	#ifdef DEBUG
		printf("Part ID: %x\n",ReadValueTemp);										//send via UART
	#endif
		ReadValueTemp = ADXL362RegisterRead(XL362_REVID);       	//version ID, 0x03
		LED_GREEN_OFF();
	#ifdef DEBUG
		printf("Version ID: %x\n",ReadValueTemp);									//send via UART
	#endif
		for (i=0; i<RXBUFFERSIZE; i++) //clear array
		{	RxBuffer[i] = 0; }
		for (i=0; i<TXBUFFERSIZE; i++) //clear array
		{	TxBuffer[i] = 0; }
		PrintInfo((uint8_t*)ConnectedInfo, sizeof(ConnectedInfo));
}

void Bufferchg(uint8_t* RxBuffer, uint8_t* TxBuffer, uint16_t RxBuffersize, uint16_t TxBuffersize)
{
	for (i=0; i<RxBuffersize; i++) //clear array
	{
		*(TxTempBuffer+2*i) = (uint8_t)(0x0F & (*RxBuffer)>>4);
		*(TxTempBuffer+2*i+1) = (uint8_t)(0x0F & (*RxBuffer));
		RxBuffer++;
	}
	Convert_ASCII((uint8_t*)TxTempBuffer,(uint8_t*)TxBuffer,TxBuffersize);
}

void Convert_ASCII(uint8_t* RxBuffer, uint8_t* TxBuffer, uint16_t RxBuffersize)
{
	uint8_t temp;
	for (i=0; i<RxBuffersize; i++)
	{
		temp = *(RxBuffer+i);	
		switch(temp) {

		case 0x00:
			*(TxBuffer+i) = 0x30;
			break;

		case 0x01:
      *(TxBuffer+i) = 0x31;
      break;

		case 0x02:
      *(TxBuffer+i) = 0x32;
      break;

		case 0x03:
      *(TxBuffer+i) = 0x33;
      break;

		case 0x04:
      *(TxBuffer+i) = 0x34;
      break;
		
		case 0x05:
      *(TxBuffer+i) = 0x35;
      break;

		case 0x06:
      *(TxBuffer+i) = 0x36;
      break;

		case 0x07:
      *(TxBuffer+i) = 0x37;
      break;

		case 0x08:
      *(TxBuffer+i) = 0x38;
      break;

		case 0x09:
      *(TxBuffer+i) = 0x39;
      break;

		case 0x0A:
      *(TxBuffer+i) = 0x41;
      break;

		case 0x0B:
      *(TxBuffer+i) = 0x42;
      break;

		case 0x0C:
      *(TxBuffer+i) = 0x43;
      break;

		case 0x0D:
      *(TxBuffer+i) = 0x44;
      break;

		case 0x0E:
      *(TxBuffer+i) = 0x45;
      break;

		case 0x0F:
      *(TxBuffer+i) = 0x46;
      break;

		default :
      *(TxBuffer+i) = 0x45;
      break;}
	}
}

//Byte write
void EEPROM_WRITE(uint16_t BiasAddress, uint8_t *Data, uint8_t len)
{
	uint8_t i;
	HAL_StatusTypeDef status = HAL_OK;

	/* Unlocks the data memory and FLASH_PECR register access *************/ 
	if(HAL_FLASHEx_DATAEEPROM_Unlock() != HAL_OK)
	{
    Error_Handler();
	}
	/* Clear FLASH error pending bits */
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_WRPERR | FLASH_FLAG_SIZERR |
																											FLASH_FLAG_OPTVERR | FLASH_FLAG_RDERR | 
																												FLASH_FLAG_FWWERR | FLASH_FLAG_NOTZEROERR);
	for(i=0;i<len;i++)
	{
		status +=HAL_FLASHEx_DATAEEPROM_Program(FLASH_TYPEPROGRAMDATA_BYTE, EEPROM_BASE_ADDR+BiasAddress+i, *Data);
		Data++;
	}
  /* Locks the Data memory and FLASH_PECR register access. (recommended
     to protect the DATA_EEPROM against possible unwanted operation) *********/
  HAL_FLASHEx_DATAEEPROM_Lock(); 
}

//Byte read
void EEPROM_READ(uint16_t BiasAddress, uint8_t *Buffer, uint8_t Len)
{
	uint8_t *wAddr;
	wAddr=(uint8_t *)(EEPROM_BASE_ADDR+BiasAddress);
	while(Len--)
	{
		*Buffer++=*wAddr++;
	}
}

void LEDRed_Blinking(uint32_t Period)
{
  /* Toggle LED2 in an infinite loop */
  while (1)
  {
    LED_RED_TOG();
    LL_mDelay(Period);
  }
}

void LEDGreen_Blinking(uint32_t Period)
{
  /* Toggle LED2 in an infinite loop */
  while (1)
  {
    LED_GREEN_TOG();
    LL_mDelay(Period);
  }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
    /* Error if LED3 is slowly blinking (1 sec. period) */
    LEDRed_Blinking(1000);
  }  
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
