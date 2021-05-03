/*
 * Performs I2C related code. Contains initialization code to establish communication, and read functions for any of the axes of the L3GD20 gyroscope, X, Y, or Z.
*/
#include "gyroscope.h"
#include "stm32f0xx_hal.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);


/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void perform_write(int slave_addr, int txdr_val);
short perform_read(int slave_addr);
/* USER CODE END PFP */

short threshold = 5000;
/* USER CODE BEGIN 0 */
short read_x()
{
	perform_write(0x6B, 0xA8);
	short x = perform_read(0x6B);
	
	return x;
	
}

short read_y()
{
	perform_write(0x6B, 0xAA);
	short y = perform_read(0x6B);

	return y;
}

short read_z()
{
	perform_write(0x6B, 0xAC);
	short z = perform_read(0x6B);
	
	return z;
}

/*
 *	Helper function for I2C operation. Sends the txdr_val to the slave_addr specified using I2C, 
			notifiying the user via LED if any issues occured.
*/ 
void perform_write(int slave_addr, int txdr_val)
{
	// Setup I2C transmission
	I2C2->CR2 |= slave_addr << 1; // Set slave address 
	I2C2->CR2 &= 0xFF00FFFF; // Clear NBYTES field
	I2C2->CR2 |= (1 << 16); // Set NBYTES to 1
	
	HAL_Delay(1);
	I2C2->CR2 &= ~(1 << 10); // Set mode to WRITE
	I2C2->CR2 |= (1 << 13); // Set START bit to true
	
	while (!(I2C2->ISR & (1 << 0))) // Wait for transmission reg to be ready.
	{
		if (I2C2->ISR & (1 << 4)) // If NACK detected, set LED
		{
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET); // blue LED
		}
	}
	
	I2C2->TXDR = txdr_val;
	
	while (!(I2C2->ISR & (1 << 6))) // TC status flag set, continue
	{
		if (I2C2->ISR & (1 << 4))
		{
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET); // red LED
			break;
		}
	}	
}

/*
 *	Helper function for I2C operation. Performs two read operations on the specified slave address using I2C, 
			notifiying the user via LED if any issues occured. This function expects two read operations, so an address
			on the L3GD20 gyro that will perform two reads, one for the low bits and one for the high bits.
*/ 
short perform_read(int slave_addr)
{
	I2C2->CR2 |= slave_addr << 1; // Set slave address
	I2C2->CR2 &= 0xFF00FFFF;	
	I2C2->CR2 |= 2 << 16; // Set NBYTES to 2
	I2C2->CR2 |= (1 << 10); // Set mode to READ
	
	HAL_Delay(1);
	I2C2->CR2 |= (1 << 13); // Set START bit to true
	
	while (!(I2C2->ISR & (1 << 2))) // RXNE status flag set, continue
	{
		if (I2C2->ISR & (1 << 4))
		{
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET); // orange LED
			break;
		}

	}
	
	short val1 = I2C2->RXDR;
	
		while (!(I2C2->ISR & (1 << 2))) // RXNE status flag set, continue
	{
		if (I2C2->ISR & (1 << 4))
		{
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET); // red LED
			break;
		}

	}
	
	short val2 = I2C2->RXDR << 8;

	while (!(I2C2->ISR & (1 << 6))) // TC status flag set, continue
	{
		if (I2C2->ISR & (1 << 4))
		{
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET); // red LED
			break;
		}				
	}
	short temp = val2 | val1; 
	return temp;
}
/* USER CODE END 0 */

/*
 *	Initilization function to establish I2C communication with L3GD20.
 *	If an error occurs, the user will be notified through LEDs.
 * 	This function must be called first, prior to any other gyroscope.c function, in order for each function to work properly.
*/
void i2c_init(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* USER CODE BEGIN 2 */
	__HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_I2C2_CLK_ENABLE(); // RCC enable I2C2

	GPIO_InitTypeDef initAF = {GPIO_PIN_11 | GPIO_PIN_13,
													  	GPIO_MODE_AF_OD,
															GPIO_SPEED_FREQ_LOW,
															GPIO_PULLUP};
	
	
	GPIO_InitTypeDef initPC = {GPIO_PIN_0,
													  	GPIO_MODE_OUTPUT_PP,
															GPIO_SPEED_FREQ_LOW,
															GPIO_NOPULL};
	
	GPIO_InitTypeDef initLED = {GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9,
															GPIO_MODE_OUTPUT_PP,
															GPIO_SPEED_FREQ_LOW,
															GPIO_NOPULL};


	HAL_GPIO_Init(GPIOB, &initAF); // initialize AF pins
	HAL_GPIO_Init(GPIOC, &initPC); // initialize PC0
	HAL_GPIO_Init(GPIOC, &initLED); // initialize LED pins
															
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);
														
	GPIOB->AFR[1] |= (1 << 12); // Set PB11 to AF1
	GPIOB->AFR[1] |= (5 << 20); // Set PB13 to AF5
																					
	I2C2->TIMINGR &= 0x0F0000; // Bit mask to clear TIMINGR
	I2C2->TIMINGR |= (0x13 << 0) | (0xF << 8) | (0x2 << 16) | (0x4 << 20); // Configure TIMINGR w/ fig. 5.4
	I2C2->TIMINGR |= (1 << 28);
				
															
	I2C2->CR1 |= (1 << 0); // Enable I2C2 	
	// Setup I2C transmission
	I2C2->CR2 |= 0x6B << 1; // Set slave address 
	I2C2->CR2 &= 0xFF00FFFF; // Clear NBYTES field
	I2C2->CR2 |= 2 << 16; // Set NBYTES to 2
	I2C2->CR2 &= ~(1 << 10); // Set mode to WRITE

	I2C2->CR2 |= (1 << 13); // Set START bit to true
	

	while (!(I2C2->ISR & (1 << 1)))
	{
		if (I2C2->ISR & (1 << 4))
		{
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET); // red LED
		}

	}
	
	I2C2->TXDR = 0x20; // Set register to gyro control reg 1

	while (!(I2C2->ISR & (1 << 1))) // wait until TXDR is ready to be written
	{
		if (I2C2->ISR & (1 << 4))
		{
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET); // red LED
		}

	}
	
	//I2C2->TXDR = 9; // Set control register to enable X, set to normal mode
	I2C2->TXDR = 12; // Set control register to enable Z, set to normal mode

	while (!(I2C2->ISR & (1 << 6))) // TC status flag set, continue
	{
		if (I2C2->ISR & (1 << 4))
		{
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET); // red LED
			break;
		}
	}
		
	I2C2->CR2 |= (1 << 14); // Set stop bit
															
  /* USER CODE END 2 */
}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks 
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

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
