/* USER CODE BEGIN Header */
/*
 * Program to handle BlueTooth connection with a PS3 controller. Currently, sends data received to a USART connection on pins:
 *    SPI: PC2, PC3, PB10 - SCK, MISO, and MOSI, respectively.
 *    USART PC10 and PC11 - TX and RX, respectively.
*/
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
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

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void TAKE_CHAR(char character) {
 
	while(!(USART3->ISR & (1 << 7))) {
	}
		
	USART3->TDR = character;
}

void PRINT_STR(char string[]) {
	for(int i = 0; i < 100; i++) {
		if(string[i] != 0)
			TAKE_CHAR(string[i]);
		else
			break;
	}
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
	__HAL_RCC_SPI2_CLK_ENABLE(); // RCC enable SPI
	RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  /* USER CODE BEGIN 2 */
  // SPI Init configuration
	SPI2->CR1 &= ~(7 << 3); // Set Baudrate to 4 MHz
	SPI2->CR1 |= (1 << 2); // Set to Master mode.
	SPI2->CR1 |= (1 << 10); // Set to receive-only mode
															
	SPI2->CR2 |= (7 << 8); // Set to 8-bit transfers
	SPI2->CR2 |= (1 << 2); // Set SS bit to high, always in master mode
	SPI2->CR2 |= (1 << 12); // Set threshold for RXNE interrupt to 8-bits.
															
	SPI2->CR2 |= (1 << 4); // Enable SPI2 
															
	//Enable LEDs for toggling use
	GPIOC->MODER |= ((1 << 12) | (1 << 14) | (1 << 16) | (1 << 18));

	USART3->CR1 |= (1 << 5);
	
	NVIC_EnableIRQ(USART3_4_IRQn);
	NVIC_SetPriority(USART3_4_IRQn,1);
	
	// Sets the 21st and 23rd bits in the GPIOB_MODER register. 
	// Make MODER10 and MODER11 alternate function mode which is bit pattern 10
	GPIOC->MODER |= (1 << 21) | (1 << 23);
	
	GPIOC->MODER |= (1 << 5) | (1 << 7); // AF modes for PC2, PC3
	GPIOC->MODER |= (1 << 10); // GPIO mode for PC5
	GPIOC->BRR |= (1 << 5); // Reset PC5. SS pin for USB adapter. Since we only have 1 slave device, we always want this to be on and listen.
	
	GPIOB->MODER |= (1 << 21); // AF mode for PB10

	GPIOC->AFR[1] |= ((0x01 << GPIO_AFRH_AFSEL10_Pos) | (0x01 << GPIO_AFRH_AFSEL11_Pos)); // Set PC10 and PC11 to AF 1
	
	GPIOC->AFR[0] |= ((0x01 << GPIO_AFRL_AFSEL2_Pos) | (0x01 << GPIO_AFRL_AFSEL3_Pos)); // Set PC2 and PC3 to AF 1
	
	GPIOB->AFR[1] |= (0x05 << GPIO_AFRH_AFSEL10_Pos); // Set PB10 to AF5

  //Set baud rate to 115200 bit/sec
	uint32_t baud_rate = HAL_RCC_GetHCLKFreq() / 115200;
	USART3->BRR = baud_rate;
	
	//Enable Transmitter (bit 3) and Receiver (bit 2) of USART
	USART3->CR1 |= (1 << 3) | (1 << 2);
	
	//USART3 enable (bit 0)
	USART3->CR1 |= (1 << 0);
	
	char str[] = "BEGIN\r\n";
	
	PRINT_STR(str); // Send opening string to USART connection.
															
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		char input = SPI2->DR;
		TAKE_CHAR(input);
  }
  /* USER CODE END WHILE */
}

void USART3_4_IRQHandler(void) {
	
	while(!(USART3->ISR & (1 << 5))) {}
		
	char character = USART3->RDR;
}


/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

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
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
