/* ####################################
Author: MD. Soyabbir Abu Hanif
 MSc in Embedded Systems
Tampere University, Finland
Email: hanifseceee535@gmail.com 
######################################
*/

/*
 * ##############################################
 * HOW TO USE THE INTERRUPT HANDLER IN MAIN CODE
 * ##############################################
 * 
 * 1. Use the configure_gpio_interrupt() function to configure the desired GPIO pin as an EXTI line.
 *    - Arguments:
 *        - pin: Pin number (e.g., 0 for PA0, 1 for PA1, etc.).
 *        - port: GPIO port :
										INT_PORT_A, INT_PORT_B, INT_PORT_C, INT_PORT_D, INT_PORT_E etc. 
				 - trigger_mode: Interrupt trigger condition:
										RISING, FALLING,BOTH
 *    - Example:
 *        configure_gpio_interrupt(0, INT_PORT_A, RISING);  // Configures PA0 (EXTI Line 0) for Rising Edge
 *
 * 2. Implement the corresponding handler for the EXTI line in your code (as described above).
 *    - Example:
 *        void EXTI0_IRQHandler(void) {
 *            if (EXTI_PR & (1 << 0)) {  // Check if EXTI Line 0 interrupt is pending
 *                EXTI_PR |= (1 << 0);   // Clear the pending bit
 *                // Your custom interrupt handling code here
 *            }
 *        }
 *
 * 3. The MCU will automatically invoke the handler when the interrupt condition occurs (e.g., button press).
 * 
 * 4. Example Main Code:
 *        #include "header.h"
 *        
 *        int main(void) {
 *            // Configure PA0 (EXTI Line 0) for Rising Edge interrupt
 *            configure_gpio_interrupt(0, INT_PORT_A, RISING);
 *            
 *            while (1) {
 *                // Main application code
 *            }
 *        }
 */


/*
 * ##############################################
 * HOW TO WRITE EXTI INTERRUPT HANDLER FUNCTIONS
 * ##############################################
 * 
 * 1. Identify the EXTI line you are configuring (e.g., EXTI0, EXTI1, etc.).
 * 2. Define the corresponding interrupt handler function. 
 *    - The handler name must follow the CMSIS convention for the specific EXTI line.
 *    - Example: EXTI0_IRQHandler() for EXTI Line 0.
 * 
 * 3. Inside the handler function:
 *    - Check if the interrupt is pending by verifying the corresponding bit in EXTI_PR.
 *    - Clear the pending bit in EXTI_PR to acknowledge the interrupt.
 *    - Write your custom application code to handle the interrupt.
 *
 * 4. Example:
 *    void EXTI0_IRQHandler(void) {
 *        if (EXTI_PR & (1 << 0)) {        // Check if EXTI Line 0 interrupt is pending
 *            EXTI_PR |= (1 << 0);         // Clear the pending bit
 *            // Your custom interrupt handling code here
 *        }
 *    }
 */

#include <stdint.h>
#include "header.h"

void configure_gpio_interrupt(uint8_t pin, uint8_t port, uint8_t trigger_type){
	
	volatile uint32_t *afio_exticr;
	uint8_t exti_line = pin;  // EXTI line corresponds directly to pin number (0–15);
	//Enabling the clock for alternative function AFIO
	RCCAPB2 = RCCAPB2 | (1<<0);
	if (exti_line < 4){
		afio_exticr = &AFIO_EXTR1;	
	}
	else if (exti_line < 8){
		afio_exticr = &AFIO_EXTR2;	
	}
	else if (exti_line < 12){
		afio_exticr = &AFIO_EXTR3;	
	}
	else {
		afio_exticr = &AFIO_EXTR4;
	}
	
	// Clear the existing port selection for the EXTI line in AFIO_EXTICR
  *afio_exticr &= ~(0xF << ((exti_line % 4) * 4));  // Clear the 4 bits for the line
  *afio_exticr |= (port << ((exti_line % 4) * 4));  // Set the new port value
	
	// Enable the interrupt mask for the corresponding EXTI line
   EXTI_IMR |= (1 << exti_line);
	
	// Configure the trigger type (Rising, Falling, or Both)
    if (trigger_type == 0) {  // Rising edge
        EXTI_RTSR |= (1 << exti_line);
        EXTI_FTSR &= ~(1 << exti_line);  // Ensure falling edge is disabled
    } else if (trigger_type == 1) {  // Falling edge
        EXTI_FTSR |= (1 << exti_line);
        EXTI_RTSR &= ~(1 << exti_line);  // Ensure rising edge is disabled
    } else if (trigger_type == 2) {  // Both edges
        EXTI_RTSR |= (1 << exti_line);
        EXTI_FTSR |= (1 << exti_line);
    }
		
		// Enable the NVIC interrupt for the EXTI line (lines 0–15 correspond to IRQ numbers 6–22)
    if (exti_line <= 4) {
        NVIC_ISER0 |= (1 << (exti_line + 6));
    }
		else if (exti_line > 4 && exti_line <=9){
			NVIC_ISER0 |= (1 << 23);
		}
		else if (exti_line > 9 && exti_line <=15){
			NVIC_ISER1 |= (1 << 8);
		}
}


