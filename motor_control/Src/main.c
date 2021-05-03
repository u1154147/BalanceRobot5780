/*
	*	Main balancing loop that handles the logic of when to change direction and what the target RPM should be.
	* Dependencies:
	*				motor.c - for actual motor control and PID control
	*				gyroscope.c - for I2C with L3GD20, integrated gyroscope on the STM32F0 boards.
*/


#include <stdio.h>
#include <stdlib.h>
#include "stm32f0xx.h"
#include "motor.h"
#include "gyroscope.h"

/* -------------------------------------------------------------------------------------------------------------
 *  Global Variable Declarations
 *  -------------------------------------------------------------------------------------------------------------
 */
volatile uint32_t debouncer;

/* -------------------------------------------------------------------------------------------------------------
 *  Miscellaneous Core Functions
 *  -------------------------------------------------------------------------------------------------------------
 */
 
/*
 *	Initialize LEDs, mainly used for error feedback in I2C and to display main loop 'heartbeat'.
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

/*
 *	Initializes button for testing purposes.
*/
void  button_init(void) {
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;                                          // Enable peripheral clock to GPIOA
    GPIOA->MODER &= ~(GPIO_MODER_MODER0_0 | GPIO_MODER_MODER0_1);               // Set PA0 to input
    GPIOC->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEEDR0_0 | GPIO_OSPEEDR_OSPEEDR0_1);     // Set to low speed
    GPIOC->PUPDR |= GPIO_PUPDR_PUPDR0_1;                                        // Set to pull-down
}

/* 
 *  Testing function used to change the direction of the motors by pressing the user button (PA0)
 */
void EXTI0_1_IRQHandler(void) {
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
volatile short threshold_gyro = 1000;
short z;
short prev_z = 0;
short z_change = 0;
short change_direction_rpm = 0;
short slow;

int main(int argc, char* argv[]) {
		
    debouncer = 0;                          // Initialize global variables
		HAL_Init();															// Initialize HAL
    LED_init();                             // Initialize LED's
    button_init();                          // Initialize button
		i2c_init();															// Initialize i2c for gyroscope reading
	
		SYSCFG->EXTICR[0] &= ~(111 << 0); // Set EXTI0 to use PA0
	
		NVIC_EnableIRQ(5); // Enable EXTI0 interrupt requests.
		NVIC_SetPriority(5, 3); // Set priority of EXTI0 to 3. Set TIMER6 interrupt to higher than this to allow PID updater to take priority.

	
		EXTI->IMR |= (1 << 0); // Enable PA0 interrupt signal
		EXTI->RTSR |= (1 << 0); // posedge activation for EXTI0


    motor_init();                           // Initialize motor code
		
    while (1) {
        GPIOC->ODR ^= GPIO_ODR_9;           // Toggle green LED (heartbeat)
        encoder_count = TIM2->CNT;
				
				z = read_z();
				z_change = z - prev_z;

				short abs_z = z;
				if (abs_z < 0)
					abs_z *= -1;
				
				short abs_z_change = z_change;
				
				if (abs_z_change < 0)
					abs_z_change *= -1;
				
				slow = abs_z_change/100; // proportional slow to prevent correctional over-shooting.
				
				if (slow > 10) // Clamp proportional slow
					slow = 10;
				
				else if (slow < 2)
					slow = 2;
				
				prev_z = z;
				if (z > threshold_gyro && ccw == 0)
				{
					target_rpm += change_direction_rpm;
				}
				
				else if (z > threshold_gyro && ccw == 1)
				{
					change_direction();
					target_rpm = 5;
				}
				
				
				else if (z < -threshold_gyro && ccw == 0)
				{
					change_direction();
					target_rpm = 5;
				}
				
				else if (z < -threshold_gyro && ccw == 1)
				{
					target_rpm += change_direction_rpm;
				}
				
				else
					{
						target_rpm -= slow;
					}
				
				if (target_rpm < 0)
					target_rpm = 0;
				
				if (target_rpm > 100)
					target_rpm = 100;
				
				
				HAL_Delay(32); // Delay: 1/64th second
    }
}

// ----------------------------------------------------------------------------
