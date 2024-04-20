/**
  ******************************************************************************
  * @file    ExtMem_CodeExecution/ExtMem_Application/FreeRTOS/Src/main.c
  * @author  MCD Application Team
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2016 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include <stdlib.h>
#include <string.h>

SPI_HandleTypeDef SpiHandle;
USART_HandleTypeDef UsartHandle;

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
osThreadId USARTThreadHandle, SPIThreadHandle;
QueueHandle_t SPIInQueue, USARTInQueue;

volatile uint16_t SPITx;
volatile uint16_t SPIRx;
volatile uint8_t USARTTx;
volatile uint8_t USARTRx;
volatile uint8_t USARTRx2;
volatile size_t usarCounter = 0;

/* Private function prototypes -----------------------------------------------*/
static void USART_Thread(void const *argument);
static void SPI_Thread(void const *argument);
static void SystemClock_Config(void);
static void CPU_CACHE_Enable(void);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{

  /* Enable the CPU Cache */
  CPU_CACHE_Enable();

  /* STM32F7xx HAL library initialization:
       - Configure the Flash ART accelerator on ITCM interface
       - Configure the Systick to generate an interrupt each 1 msec
       - Set NVIC Group Priority to 4
       - Low Level Initialization
     */
  HAL_Init();  
  
  /* Configure the system clock to 216 Mhz */
  SystemClock_Config();
  
  /* Set the SPI parameters */
  SpiHandle.Instance               	= SPI2;
  SpiHandle.Init.BaudRatePrescaler 	= SPI_BAUDRATEPRESCALER_256;
  SpiHandle.Init.Direction         	= SPI_DIRECTION_2LINES;
  SpiHandle.Init.CLKPhase          	= SPI_PHASE_1EDGE;
  SpiHandle.Init.CLKPolarity       	= SPI_POLARITY_HIGH;
  SpiHandle.Init.DataSize          	= SPI_DATASIZE_16BIT;
  SpiHandle.Init.FirstBit          	= SPI_FIRSTBIT_MSB;
  SpiHandle.Init.TIMode            	= SPI_TIMODE_ENABLED; //! Slave can always send data
  SpiHandle.Init.CRCCalculation    	= SPI_CRCCALCULATION_DISABLE;
  SpiHandle.Init.NSS               	= SPI_NSS_SOFT;
  SpiHandle.Init.Mode 			   	= SPI_MODE_MASTER;

  UsartHandle.Instance				= USART2;
  UsartHandle.Init.BaudRate			= 9600;
  UsartHandle.Init.CLKLastBit		= USART_LASTBIT_DISABLE;
  UsartHandle.Init.CLKPhase 		= USART_PHASE_1EDGE;
  UsartHandle.Init.CLKPolarity 		= USART_POLARITY_HIGH;
  UsartHandle.Init.Mode 			= USART_MODE_TX_RX;
  UsartHandle.Init.Parity 			= USART_PARITY_NONE;
  UsartHandle.Init.StopBits 		= USART_STOPBITS_1;
  UsartHandle.Init.WordLength 		= USART_WORDLENGTH_8B;

  /* Thread 1 definition */
  osThreadDef(1, USART_Thread, osPriorityNormal, 0, configMINIMAL_STACK_SIZE);
  
  /* Thread 2 definition */
  osThreadDef(2, SPI_Thread, osPriorityNormal, 0, configMINIMAL_STACK_SIZE);
  
  /* Start thread 1 */
  USARTThreadHandle = osThreadCreate(osThread(1), NULL);
  
  /* Start thread 2 */
  SPIThreadHandle = osThreadCreate(osThread(2), NULL);
  
  SPIInQueue = xQueueCreate(32, sizeof(uint8_t)); //!Queue from SPI to USART
  USARTInQueue = xQueueCreate(32, sizeof(uint16_t)); //!Queue from USART to SPI

  if (!SPIInQueue || !USARTInQueue)
  {
	  exit(1);
  }

  if(HAL_SPI_Init(&SpiHandle) != HAL_OK)
  {
	  exit(1);
  }

  if(HAL_SPI_TransmitReceive_IT(&SpiHandle, (uint8_t*)&SPITx, (uint8_t*)&SPIRx, sizeof(uint16_t)) != HAL_OK)
  {
	  exit(1);
  }

  if(HAL_USART_TransmitReceive_IT(&UsartHandle, (uint8_t*)&USARTTx, (uint8_t*)&USARTRx, sizeof(uint8_t)) != HAL_OK)
  {
	  exit(1);
  }

  /* Start scheduler */
  osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */
  for(;;);
}

static void USART_Thread(void const *argument)
{
	(void) argument;
	uint16_t buff_in = 0;
	for(;;)
	{
		//! Receive rx data from queue
		if (xQueueReceive(USARTInQueue, &buff_in, portMAX_DELAY) == pdTRUE)
		{
			//! Send data to SPI
			HAL_SPI_Transmit(&SpiHandle, (uint8_t*)&buff_in, sizeof(uint16_t), portMAX_DELAY);
		}
	}
}

static void SPI_Thread(void const *argument)
{
	(void) argument;
	static size_t stringNotEnd = 0;
	uint8_t buff_in = 0;
	for(;;)
	{
		//! Receive rx data from queue
		if (xQueueReceive(SPIInQueue, &buff_in, portMAX_DELAY) == pdTRUE)
		{
			if (stringNotEnd)
			{
				//! Send data to USART
				HAL_USART_Transmit(&UsartHandle, &buff_in, sizeof(uint8_t), portMAX_DELAY);
			}
			//! If data is NULL, then string in end no need to transmit
			if (buff_in == 0) stringNotEnd = 1;
			else stringNotEnd = 0;
		}
	}
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
	//! Send rx data to thread queue
	//! If data is NULL, then string in end no need to transmit
	if (SPIRx != 0)
	{
		//!Hi
		xQueueSendFromISR(SPIInQueue, (uint8_t*)&SPIRx, 0);
		//!Lo
		xQueueSendFromISR(SPIInQueue, ((uint8_t*)&SPIRx)+1, 0);
		SPIRx = 0;
	}
}

void HAL_USART_RxCpltCallback(USART_HandleTypeDef *hspi)
{
	//! Send rx data to thread queue
	if ((usarCounter++) & 0x1)
	{
		uint16_t buff = (USARTRx2 << 8) | USARTRx;
		xQueueSendFromISR(USARTInQueue, &buff, 0);
		USARTRx2 = 0;
	}
	else
	{
		USARTRx2 = USARTRx;
	}
	USARTRx = 0;
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 216000000
  *            HCLK(Hz)                       = 216000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 4
  *            APB2 Prescaler                 = 2
  *            HSE Frequency(Hz)              = 25000000
  *            PLL_M                          = 25
  *            PLL_N                          = 432
  *            PLL_P                          = 2
  *            PLL_Q                          = 9
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale1 mode
  *            Flash Latency(WS)              = 7
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;

  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_OFF;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;

  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 432;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 8;

  HAL_RCC_OscConfig(&RCC_OscInitStruct);
  
  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;  
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7);

}


#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
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
  {}
}
#endif

/**
  * @brief  CPU L1-Cache enable.
  * @param  None
  * @retval None
  */
static void CPU_CACHE_Enable(void)
{
  /* Enable I-Cache */
  SCB_EnableICache();

  /* Enable D-Cache */
  SCB_EnableDCache();
}

