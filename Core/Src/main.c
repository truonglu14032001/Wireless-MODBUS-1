/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <stdint.h>
#include "lcd16x2.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MODBUS_GENERATOR 0xA001
#define RESPONSE_TIMEOUT 5000
#define RX_BUFFER_SIZE 256
#define SLAVE_ID_ADDRESS 0x08008000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint16_t MQ2;
uint16_t SensorValue;
int8_t X=120;
uint8_t RxBuffer[RX_BUFFER_SIZE];
uint16_t RxCount = 0;
uint8_t TxBuffer[RX_BUFFER_SIZE];
uint16_t TxCount = 0;
uint8_t SlaveID;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint16_t ModbusCalcCRC(uint8_t *Frame, uint16_t LenFrame)
{
	uint16_t CntByte;
	uint16_t j;
	uint8_t bitVal;
	uint16_t CRCcalculate = 0xFFFF;
	for(CntByte=0;CntByte<LenFrame;CntByte++)
	{
		CRCcalculate ^= Frame[CntByte];
		for(j=0;j<8;j++)
		{
			bitVal = CRCcalculate & 0x0001;
			CRCcalculate = CRCcalculate >> 1;
			if(bitVal == 1)
				CRCcalculate ^= MODBUS_GENERATOR;
		}
	}
	return CRCcalculate;
}

void ResetBuffers(void)
{
	RxCount = 0;
	TxCount = 0;
	memset(RxBuffer, 0, RX_BUFFER_SIZE);
	memset(TxBuffer, 0, RX_BUFFER_SIZE);
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	if (huart == &huart1)
	{
		RxCount = Size;
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, RxBuffer, RX_BUFFER_SIZE);
	}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	if(hadc->Instance == ADC1)
	{
		MQ2 = HAL_ADC_GetValue(&hadc1)*10000/4095;
		//SensorValue = MQ2/4095*10000;
	}
}

void WriteSlaveIDToMemory(uint16_t *slaveID)
{
	HAL_FLASH_Unlock(); // M? khóa Flash tru?c khi ghi
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, SLAVE_ID_ADDRESS, *slaveID);
	HAL_FLASH_Lock(); // Khóa Flash sau khi ghi
}
/*
bool isSlaveAddressValid(uint16_t Value) 
{
	if (Value >= 0x0001 && Value <= 0x00F7)
	{
		return true;
	}
	else
	{
		return false;
	}
}
*/
void Frame_erro(uint8_t Data_Erro)
{
	TxBuffer[0] = SlaveID;
	TxBuffer[1] = RxBuffer[1] + 0x80;
	TxBuffer[2] = Data_Erro;
	uint16_t Response_CRC = ModbusCalcCRC(TxBuffer, 3);
	TxBuffer[3] = 0x00 | Response_CRC;
	TxBuffer[4] = Response_CRC >> 8;
	HAL_UART_Transmit(&huart1, TxBuffer, 5, HAL_MAX_DELAY);
	ResetBuffers();
	HAL_UARTEx_ReceiveToIdle_IT(&huart1, RxBuffer, RX_BUFFER_SIZE);
}
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
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	lcd16x2_init_4bits(GPIOB, RS_Pin, E_Pin, GPIOB, D4_Pin, D5_Pin, D6_Pin, D7_Pin);
	HAL_UARTEx_ReceiveToIdle_IT(&huart1, RxBuffer, RX_BUFFER_SIZE);
	HAL_ADC_Start_IT(&hadc1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		HAL_ADC_Start_IT(&hadc1);
		HAL_Delay(200);
		lcd16x2_1stLine();
		lcd16x2_printf("MQ2: %d ppm", MQ2);
		if(MQ2 > 4000)
		{
			HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_11); 
		}
		else HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET); 
		if (SlaveID == 0)
		{
		// N?u không có giá tr? luu tr?, gán giá tr? m?c d?nh và luu tr? nó
				 SlaveID = 0x0006;
				 WriteSlaveIDToMemory(&SlaveID);
		}
		if (RxCount >= 1)
		{
			uint16_t RequestCRC;
			RequestCRC = (RxBuffer[RxCount - 1] << 8) | RxBuffer[RxCount-2];
			uint16_t CRCCalc = ModbusCalcCRC(RxBuffer, RxCount - 2);
			if (RequestCRC == CRCCalc)
			{
				if (RxBuffer[0] == SlaveID)
				{
					if (RxBuffer[1] == 0x03)
					{
						uint16_t Data_address;
						Data_address = (RxBuffer[2] << 8) | RxBuffer[3];
						if (Data_address == 0x0001)
						{
							uint16_t Data_value;
							Data_value = (RxBuffer[4] << 8) | RxBuffer[5];
							if (Data_value == 0x0001)
							{
								TxBuffer[0] = SlaveID;
								TxBuffer[1] = RxBuffer[1];
								TxCount = 2;
								TxBuffer[2] = TxCount;
								TxBuffer[3] = 0x00;
								TxBuffer[4] = SlaveID;
								uint16_t Response_CRC = ModbusCalcCRC(TxBuffer, 5);
								TxBuffer[5] = 0x00 | Response_CRC;
								TxBuffer[6] = Response_CRC >> 8 ;
								HAL_UART_Transmit(&huart1, TxBuffer, 7, HAL_MAX_DELAY);
								ResetBuffers();
								HAL_UARTEx_ReceiveToIdle_IT(&huart1, RxBuffer, RX_BUFFER_SIZE);
							}
							else
							{
								Frame_erro(0x03);
							}
						}
						else if (Data_address == 0x0011)
						{
							uint16_t Data_value;
							Data_value = (RxBuffer[4] << 8) | RxBuffer[5];
							if (Data_value == 0x0001)
							{
								//HAL_ADC_Start_IT(&hadc1);
								TxCount = 2;
								TxBuffer[0] = SlaveID;
								TxBuffer[1] = RxBuffer[1];
								TxBuffer[2] = TxCount;
								TxBuffer[3] = MQ2 >> 8;
								TxBuffer[4] = MQ2 & 0xFF;
								uint16_t Response_CRC = ModbusCalcCRC(TxBuffer, 5);
								TxBuffer[5] = 0x00 | Response_CRC;
								TxBuffer[6] = Response_CRC >> 8 ;
								HAL_UART_Transmit(&huart1, TxBuffer, 7, HAL_MAX_DELAY);
								ResetBuffers();
								HAL_UARTEx_ReceiveToIdle_IT(&huart1, RxBuffer, RX_BUFFER_SIZE);
							}
							else
							{
								Frame_erro(0x03);
							}
						}
						else if (Data_address == 0x0112)
						{
							uint16_t Data_value;
							Data_value = (RxBuffer[4] << 8) | RxBuffer[5];
							if (Data_value == 0x0001)
							{
								uint16_t Relay_state = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8);
								TxCount = 2;
								TxBuffer[0] = SlaveID;
								TxBuffer[1] = RxBuffer[1];
								TxBuffer[2] = TxCount;
								TxBuffer[3] = Relay_state >> 8;
								TxBuffer[4] = Relay_state & 0xFF;
								uint16_t Response_CRC = ModbusCalcCRC(TxBuffer, 5);
								TxBuffer[5] = 0x00 | Response_CRC;
								TxBuffer[6] = Response_CRC >> 8 ;
								HAL_UART_Transmit(&huart1, TxBuffer, 7, HAL_MAX_DELAY);
								ResetBuffers();
								HAL_UARTEx_ReceiveToIdle_IT(&huart1, RxBuffer, RX_BUFFER_SIZE);
							}
							else
							{
								Frame_erro(0x03);
							}
						}
						else
						{
							Frame_erro(0x02);
						}
					}
					else if (RxBuffer[1] == 0x06)
					{
						uint16_t Data_address;
						Data_address = (RxBuffer[2] << 8) | RxBuffer[3];
						if (Data_address == 0x0001)
						{
							uint16_t Data_value;
							Data_value = (RxBuffer[4] << 8) | RxBuffer[5];
							if (Data_value >= 0x0001 && Data_value <= 0x00F7)
							{
								for (uint8_t i = 0; i < RxCount; i++)
								{
									TxBuffer[i] = RxBuffer[i];
								}
								 // Luu tr? giá tr? m?i c?a SlaveID vào b? nh?
								HAL_UART_Transmit(&huart1, TxBuffer, 8, HAL_MAX_DELAY);
								WriteSlaveIDToMemory(&Data_value);
								ResetBuffers();
								HAL_UARTEx_ReceiveToIdle_IT(&huart1, RxBuffer, RX_BUFFER_SIZE);
							}
							else
							{
								Frame_erro(0x03);
							}
						}
						if (Data_address == 0x0112)
						{
							uint16_t Data_value;
							Data_value = (RxBuffer[4] << 8) | RxBuffer[5];
							if (Data_value == 0x0001)
							{
								HAL_GPIO_WritePin(RELAY_GPIO_Port, RELAY_Pin, GPIO_PIN_SET);
								for (uint8_t i = 0; i < RxCount; i++)
								{
									TxBuffer[i] = RxBuffer[i];
								}
								/*TxBuffer[0] = SlaveID;
								TxBuffer[1] = RxBuffer[1];
								TxBuffer[2] = TxCount;
								TxBuffer[3] = Data_value >> 8;
								TxBuffer[4] = Data_value & 0xFF;
								uint16_t Response_CRC = ModbusCalcCRC(TxBuffer, 5);
								TxBuffer[5] = 0x00 | Response_CRC;
								TxBuffer[6] = Response_CRC >> 8 ;*/
								HAL_UART_Transmit(&huart1, TxBuffer, RxCount, HAL_MAX_DELAY);
								ResetBuffers();
								HAL_UARTEx_ReceiveToIdle_IT(&huart1, RxBuffer, RX_BUFFER_SIZE);
							}
							else if (Data_value == 0x0000)
							{
								HAL_GPIO_WritePin(RELAY_GPIO_Port, RELAY_Pin, GPIO_PIN_RESET);
								for (uint8_t i = 0; i < RxCount; i++)
								{
									TxBuffer[i] = RxBuffer[i];
								}
								/*TxBuffer[0] = SlaveID;
								TxBuffer[1] = RxBuffer[1];
								TxBuffer[2] = TxCount;
								TxBuffer[3] = Data_value >> 8;
								TxBuffer[4] = Data_value & 0xFF;
								uint16_t Response_CRC = ModbusCalcCRC(TxBuffer, 5);
								TxBuffer[5] = 0x00 | Response_CRC;
								TxBuffer[6] = Response_CRC >> 8 ;*/
								HAL_UART_Transmit(&huart1, TxBuffer, RxCount, HAL_MAX_DELAY);
								ResetBuffers();
								HAL_UARTEx_ReceiveToIdle_IT(&huart1, RxBuffer, RX_BUFFER_SIZE);
							}
							else
							{
								Frame_erro(0x03);
							}
						}
						else
						{
							Frame_erro(0x02);
						}
					}
					else
					{
						Frame_erro(0x01);
					}
				}
				else
				{
					ResetBuffers();
					HAL_UARTEx_ReceiveToIdle_IT(&huart1, RxBuffer, RX_BUFFER_SIZE);
				}
			}
			else
			{
				ResetBuffers();
				HAL_UARTEx_ReceiveToIdle_IT(&huart1, RxBuffer, RX_BUFFER_SIZE);
			}
		}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, RS_Pin|E_Pin|D4_Pin|D5_Pin
                          |D6_Pin|D7_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, RELAY_Pin|GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pins : RS_Pin E_Pin D4_Pin D5_Pin
                           D6_Pin D7_Pin */
  GPIO_InitStruct.Pin = RS_Pin|E_Pin|D4_Pin|D5_Pin
                          |D6_Pin|D7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : RELAY_Pin */
  GPIO_InitStruct.Pin = RELAY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(RELAY_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA11 */
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
