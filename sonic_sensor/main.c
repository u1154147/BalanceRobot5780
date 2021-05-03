/* USER CODE BEGIN Header */
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


#include <stdio.h>


void SystemClock_Config(void);



const float speedSound = 0.0343;
float distance;

void usDelay(uint32_t uSec)
{
	if(uSec < 2) uSec = 2;
	TIM3->ARR = 10 * uSec - 1; 	/*sets the value in the auto-reload register*/
	TIM3->EGR = 1; 			/*Re-initialises the timer*/
	TIM3->PSC = 2-1;
	TIM3->SR &= ~1; 		//Resets the flag
	TIM3->CR1 |= 1; 		//Enables the counter
	while((TIM3->SR & 0x0001) != 1);
	TIM3->CR1 &= ~(1 << 1);
	
}

int main(void)
{
	uint32_t ticks = 0;

  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN; 

	
	//Enable LEDs for toggling use
	GPIOC->MODER |= ((1 << 12) | (1 << 14) | (1 << 16) | (1 << 18));
	
	//PB8 is trig output so set to output mode
	GPIOB->MODER |= (1 << 16);
	
	
	//Sets TIM3_PSC to value 7
	TIM3->PSC = 80-1;
	TIM3->CCER |= ((1 << 0) | (1 << 4));	
	
	
  while (1)
  {

		GPIOB->ODR &= ~(8 << 1); //Trigger pin hold low for a bit
		usDelay(3);

		
		GPIOB->ODR |= (8 << 1); //Trigger pin hold high for 10us to send sonie pulse
		usDelay(10);
		GPIOB->ODR &= ~(8 << 1); //Trigger pin hold low
		
		//Wait while the echo pin is still low
		while((GPIOB->ODR & (9 << 1)) == 0);
		
		//Try to time the echo response after sending out the initial sonic pulse
		ticks = 0;
		while((GPIOB->ODR & (9 << 1)) == 1) {
			ticks++;
			usDelay(2);
		}
		
		distance = (ticks + 0.0f)*2.8*(speedSound/2); //Calculate distance from cm/us
		
		//Test LEDs to see about how far an object is:
		if(distance > 300)
			GPIOC->ODR |= (1 << 6);
		else if(distance > 200)
			GPIOC->ODR |= (1 << 7);
		else if(distance > 100)
			GPIOC->ODR |= (1 << 8);	
		else if(distance > 2)
			GPIOC->ODR |= (1 << 9);	
		
		HAL_Delay(100); //Delay 100ms until next reading. This is above the recommended 60ms measurement cycle.
		
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