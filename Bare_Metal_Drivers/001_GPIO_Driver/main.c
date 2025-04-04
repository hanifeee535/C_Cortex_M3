#include <stdint.h>
#include "Drivers/stm32f103GPIODrive.h"

volatile uint8_t interrupt_occured = 0;
void EXTI15_10_IRQHandler(void) {
	if (EXTI->PR & (1 << 13))  // Checking if EXT line 13 interrupt flag is set or not
		{
			
			if (interrupt_occured == 0){ interrupt_occured = 1; }
			else { interrupt_occured = 0; }
		
			EXTI->PR |= (1 << 13); // Clearing the interrupt flag
	  }
}

int main (){
	Config_GPIO(portC, 0, output_50Mhz, gp_output);
	configure_gpio_interrupt (13, INT_PORT_C, RISING);
	
	while (1){
			
			if (interrupt_occured){
			Write_GPIO(portC, 0, HIGH);
			}
			else {
			toggle_gpio (portC, 0);
			
			for (int i=0; i<100000; i++) {}
			
			}
			
			
		}
	
}
