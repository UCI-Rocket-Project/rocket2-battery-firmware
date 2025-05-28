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
#include <stdio.h>
#include <cmath>
#include "radio_sx127x_spi.h"

using namespace std; 

typedef enum {
	MOSFET1,    // ground/wall power
	MOSFET2     // battery power 
} mosfetPin;

struct AdcData {
  uint32_t supplyBattery;     // V_Sense1
  uint32_t ecuBattery;        // V_Sense2 
}; 

struct BatteryData {
  uint32_t timestamp; 
  float supplyVoltage = std::nanf("");
  float ecuVoltage = std::nanf("");
};

struct CommandPacket {
  uint8_t id; 
  uint16_t sequence; 
};

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

/**
  * @brief  The application entry point.
  * @retval int
  */
 int main(void)
 {
   /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
   HAL_Init();
 
   /* Configure the system clock */
   SystemClock_Config();
 
   /* Initialize all configured peripherals */
   MX_GPIO_Init();
   MX_ADC1_Init();
   MX_SPI2_Init();

   // initialize radio and reset if it fails 
   if (!radio.Init())
    radio.Reset();

   CommandPacket commandPacket; 
   int rssi;                                 

   BatteryData batteryData; 

   uint16_t lastCommandSequence = 0; 
   uint8_t lastCommandId = 0x0F;
   int16_t lastCommandRssi = 0; 

   // Start recieving data
   radio.Receive((uint8_t *)&commandPacket, sizeof(CommandPacket), &rssi);
 
   /* Infinite loop */
   while (1)
   { 
    HAL_GPIO_WritePin(MOSFET2_GPIO_Port, MOSFET2_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(Status_LED_GPIO_Port, Status_LED_Pin, GPIO_PIN_SET);
    HAL_Delay(1000);

     char buffer[1024] = {0};

     uint32_t timestamp = HAL_GetTick(); // replace later with timers 
     batteryData.timestamp = timestamp;

     // Update radio state
     RadioSx127xSpi::State state = radio.Update();
 
     if (state == RadioSx127xSpi::State::RX_COMPLETE) {
        if (commandPacket.sequence <= lastCommandSequence){
          radio.Receive((uint8_t *)&commandPacket, sizeof(CommandPacket), &rssi);
          break; 
        }
        sprintf(buffer, 
                "Radio packet recieved: %02X\r\n"
                "Timestamp: %08X\r\n",
                commandPacket.id,
                (unsigned int)(batteryData.timestamp));

         // Process the packet
        switch (commandPacket.id){
          case 0x00: // cut power
            open_mosfet(MOSFET1);
            open_mosfet(MOSFET2);
            break; 
            
          case 0xF8: // connect to wall/ground power and disconnect from battery 
            // connect wall/ground power
            close_mosfet(MOSFET1);
            // disconnect battery power
            open_mosfet(MOSFET2);
            break; 

          case 0x1F: // connect to battery and disconnect from wall power
            // connect battery power
            close_mosfet(MOSFET2);
            // disconnect ground/wall power
            open_mosfet(MOSFET1);
            break; 

          default: 
            sprintf(buffer, "Unknown command: ");
            radio.Receive((uint8_t *)&commandPacket, sizeof(CommandPacket), &rssi);
            break; 
        } // end switch

        // checking battery data 
        AdcData rawData = {0};
        for (int i = 0; i < 16; i++) {
          HAL_ADC_Start(&hadc1); 

          HAL_ADC_PollForConversion(&hadc1, 10);

          uint32_t data1 = HAL_ADC_GetValue(&hadc1);
          *(((uint32_t *)&rawData) + i) += data1;
        }
        // convert to volts-- may need calibration
        batteryData.ecuVoltage = 0.062f * (float)rawData.ecuBattery;
        batteryData.supplyVoltage = 0.062 * (float)rawData.supplyBattery;
        
        sprintf(buffer, 
                "No power connect\r\n"
                "ECU Voltage: %04d  Supply Voltage: %04d"
                "--------------------\r\n",
                (int)(batteryData.ecuVoltage), (int)(batteryData.supplyVoltage));
        lastCommandId = commandPacket.id;
        lastCommandSequence = commandPacket.sequence; 
        lastCommandRssi = (int16_t)rssi;
      } // end if 

      /**************ERROR CHECKING******************/
      else if ((state == RadioSx127xSpi::State::RX_TIMEOUT) ||
               (state == RadioSx127xSpi::State::IDLE)){
        // Restart receiving
        radio.Receive((uint8_t *)&commandPacket, sizeof(CommandPacket), &rssi);
      } // end else if timeout

      else if (state == RadioSx127xSpi::State::ERROR){
        // Reinitialize radio
        if (!radio.Init())
          radio.Reset();

        // restart receiving
        radio.Receive((uint8_t *)&commandPacket, sizeof(CommandPacket), &rssi);
      } // end else if timeout     
   } // end while 
 }

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
