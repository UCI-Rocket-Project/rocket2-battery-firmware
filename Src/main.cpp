/* USER CODE BEGIN Header */
////sdkfnjskdNFlkdsn vldfz bjdf k
/**
  ******************************************************************************
  * @file           : main.cpp
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "radio_sx127x_spi.h"

typedef enum {
	MOSFET1,    // ground/wall power
	MOSFET2     // battery power 
} mosfetPin;

ADC_HandleTypeDef hadc1;
SPI_HandleTypeDef hspi2;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI2_Init(void);

// creating an instance for the radio packet
RadioSx127xSpi radio(&hspi2, RADIO_nCS_GPIO_Port, RADIO_nCS_Pin, RADIO_nRST_GPIO_Port, 
                     RADIO_nRST_Pin, 0xDA, RadioSx127xSpi::RfPort::PA_BOOST, 433000000, 15, 
                     RadioSx127xSpi::RampTime::RT40US, RadioSx127xSpi::Bandwidth::BW250KHZ, 
                     RadioSx127xSpi::CodingRate::CR45, RadioSx127xSpi::SpreadingFactor::SF7, 
                     8, true, 500, 1023);

static void open_mosfet(mosfetPin selectM);
static void close_mosfet(mosfetPin selectM);

/*
* @brief closes circuit of parameter mosfet
* @param selectM payload pointer to payload buffer
*/
static void close_mosfet(mosfetPin selectM) { 
	GPIO_TypeDef* mosfet_port;
	uint16_t mosfet_pin;

	switch(selectM) {
		case MOSFET1:
			mosfet_port = MOSFET1_GPIO_Port;
			mosfet_pin = MOSFET1_Pin;
			HAL_GPIO_WritePin(mosfet_port, mosfet_pin, GPIO_PIN_SET);
			break;
		case MOSFET2:
			mosfet_port = MOSFET2_GPIO_Port;
			mosfet_pin = MOSFET2_Pin;
			HAL_GPIO_WritePin(mosfet_port, mosfet_pin, GPIO_PIN_SET);
			break;
		default:
			break;
	}
}

/*
* @brief opens circuit of parameter mosfet
* @param selectM payload pointer to payload buffer
*/
static void open_mosfet(mosfetPin selectM) { 
	GPIO_TypeDef* mosfet_port;
	uint16_t mosfet_pin;

	switch(selectM) {
		case MOSFET1:
			mosfet_port = MOSFET1_GPIO_Port;
			mosfet_pin = MOSFET1_Pin;
			HAL_GPIO_WritePin(mosfet_port, mosfet_pin, GPIO_PIN_RESET);
			break;
		case MOSFET2:
			mosfet_port = MOSFET2_GPIO_Port;
			mosfet_pin = MOSFET2_Pin;
			HAL_GPIO_WritePin(mosfet_port, mosfet_pin, GPIO_PIN_RESET);
			break;
		default:
			break;
	}
}


/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  // initialize radio and reset if it fails 
  if (!radio.Init())
    radio.Reset();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_SPI2_Init();

  uint8_t payload[255];                     // Buffer for received data
  int rssi;                                 // Variable to hold the received signal strength
  uint8_t payloadLength = sizeof(payload);  // Maximum payload length

  // Start recieving data
  radio.Receive(payload, payloadLength, &rssi);

  /* Infinite loop */
  while (1)
  {
    // Update radio state
    RadioSx127xSpi::State state = radio.Update();

    if (state == RadioSx127xSpi::State::RX_COMPLETE) {
        // Decode the packet
        
        // If the first bit is 1, then it CUT POWER
        if (payload[0] & 0x80) { // 0x80 = 10000000 in binary- checking MSB
          // open mosfets to cut power entirely 
          open_mosfet(MOSFET1);
          open_mosfet(MOSFET2);
        }

        // If the second bit is 1, then it is connected to wall/ground power
        else if (payload[0] & 0x40) { // 0x80 = 01000000 in binary- checking MSB
          // connect wall/ground power
          close_mosfet(MOSFET1);
          // disconnect battery power
          open_mosfet(MOSFET2);
        }

        // If the third bit is 1, then it is connected to battery power
        else if (payload[0] & 0x20) { // 0x80 = 00100000 in binary- checking MSB
          // connect battery power
          close_mosfet(MOSFET2);
          // disconnect ground/wall power
          open_mosfet(MOSFET1);
        }

        // Prepare for next packet
        radio.Receive(payload, payloadLength, &rssi);
    }

    /**************ERROR CHECKING******************/
    else if (state == RadioSx127xSpi::State::RX_TIMEOUT){
      // Restart receiving
      radio.Receive(payload, payloadLength, &rssi);
    }
    else if (state == RadioSx127xSpi::State::ERROR){
      // Reinitialize radio
      if (!radio.Init())
        radio.Reset();

      // restart receiving
      radio.Receive(payload, payloadLength, &rssi);
    }
  }
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
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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
  HAL_GPIO_WritePin(GPIOA, MOSFET1_Pin|MOSFET2_Pin|RADIO_DIO5_Pin|RADIO_DIO4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, RADIO_DIO3_Pin|RADIO_DIO2_Pin|RADIO_DIO1_Pin|RADIO_DIO0_Pin
                          |RADIO_nRST_Pin|RADIO_nCS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : MOSFET1_Pin MOSFET2_Pin RADIO_DIO5_Pin RADIO_DIO4_Pin */
  GPIO_InitStruct.Pin = MOSFET1_Pin|MOSFET2_Pin|RADIO_DIO5_Pin|RADIO_DIO4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : RADIO_DIO3_Pin RADIO_DIO2_Pin RADIO_DIO1_Pin RADIO_DIO0_Pin
                           RADIO_nRST_Pin RADIO_nCS_Pin */
  GPIO_InitStruct.Pin = RADIO_DIO3_Pin|RADIO_DIO2_Pin|RADIO_DIO1_Pin|RADIO_DIO0_Pin
                          |RADIO_nRST_Pin|RADIO_nCS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
