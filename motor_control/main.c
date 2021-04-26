#include <stdio.h>
#include <stdlib.h>
#include "stm32f0xx.h"
#include "motor.h"

/* -------------------------------------------------------------------------------------------------------------
 *  Global Variable Declarations
 *  -------------------------------------------------------------------------------------------------------------
 */
volatile uint32_t debouncer;

/* -------------------------------------------------------------------------------------------------------------
 *  Miscellaneous Core Functions
 *  -------------------------------------------------------------------------------------------------------------
 */

void LED_init(void) {
    // Initialize PC8 and PC9 for LED's
    RCC->AHBENR |= RCC_AHBENR_GPIOCEN;                                          // Enable peripheral clock to GPIOC
    GPIOC->MODER |= GPIO_MODER_MODER8_0 | GPIO_MODER_MODER9_0;                  // Set PC8 & PC9 to outputs
    GPIOC->OTYPER &= ~(GPIO_OTYPER_OT_8 | GPIO_OTYPER_OT_9);                    // Set to push-pull output type
    GPIOC->OSPEEDR &= ~((GPIO_OSPEEDR_OSPEEDR8_0 | GPIO_OSPEEDR_OSPEEDR8_1) |
                        (GPIO_OSPEEDR_OSPEEDR9_0 | GPIO_OSPEEDR_OSPEEDR9_1));   // Set to low speed
    GPIOC->PUPDR &= ~((GPIO_PUPDR_PUPDR8_0 | GPIO_PUPDR_PUPDR8_1) |
                      (GPIO_PUPDR_PUPDR9_0 | GPIO_PUPDR_PUPDR9_1));             // Set to no pull-up/down
    GPIOC->ODR &= ~(GPIO_ODR_8 | GPIO_ODR_9);                                   // Shut off LED's
}

void  button_init(void) {
    // Initialize PA0 for button input
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;                                          // Enable peripheral clock to GPIOA
    GPIOA->MODER &= ~(GPIO_MODER_MODER0_0 | GPIO_MODER_MODER0_1);               // Set PA0 to input
    GPIOC->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEEDR0_0 | GPIO_OSPEEDR_OSPEEDR0_1);     // Set to low speed
    GPIOC->PUPDR |= GPIO_PUPDR_PUPDR0_1;                                        // Set to pull-down
}

/* Called by SysTick Interrupt
 * Performs button debouncing, changes wave type on button rising edge
 * Updates frequency output from ADC value
 */
void EXTI0_1_IRQHandler(void) {
    // Remember that this function is called by the SysTick interrupt
    // You can't call any functions in here that use delay

    change_direction();
	
		EXTI->PR |= (1 << 0); // Clear status flag for EXTI0
}

/* -------------------------------------------------------------------------------------------------------------
 * Main Program Code
 *
 * Starts initialization of peripherals
 * Blinks green LED (PC9) in loop as heartbeat
 * -------------------------------------------------------------------------------------------------------------
 */
volatile uint32_t encoder_count = 0;

int main(int argc, char* argv[]) {
		
    debouncer = 0;                          // Initialize global variables
		HAL_Init();															// Initialize HAL
    LED_init();                             // Initialize LED's
    button_init();                          // Initialize button
	
		SYSCFG->EXTICR[0] &= ~(111 << 0); // Set EXTI0 to use PA0
	
		NVIC_EnableIRQ(5); // Enable EXTI0 interrupt requests.
		NVIC_SetPriority(5, 3); // Set priority of EXTI0 to 1 (highest). This will starve SysTick interrupt and main.

	
		//NVIC_SetPriority(-1, 2); // Set priority of SysTick to 2 
		NVIC_SetPriority(17, 1); // Set priority of SysTick to 2 
	
		EXTI->IMR |= (1 << 0); // Enable PA0 interrupt signal
		EXTI->RTSR |= (1 << 0); // posedge activation for EXTI0


    motor_init();                           // Initialize motor code

    while (1) {
        GPIOC->ODR ^= GPIO_ODR_9;           // Toggle green LED (heartbeat)
        encoder_count = TIM2->CNT;
        HAL_Delay(128);                      // Delay 1/8 second
    }
}

// ----------------------------------------------------------------------------
