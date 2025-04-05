#include <stdint.h>
#include "../Drivers/stm32f103Driver.h"

 
uint16_t period = 500;  // 1kHz (initial period in microseconds)
uint8_t duty_percent = 10;  // Start with 50% duty cycle



void EXTI15_10_IRQHandler(void){
	if (EXTI->PR & (1 << 13)) {
		EXTI->PR |= (1 << 13); // Clear interrupt flag

		
		if (duty_percent <= 100) {
			duty_percent += 10;
		} else {
			duty_percent = 10;
		}

		// if (period <= 2000) {
		// 	period += 500;
		// } else {
		// 	period = 500;
		// }

		// Update PWM dynamically (assuming TIM2 CH1 on PA0)
		update_duty(portA,0, duty_percent);
		//update_period (portA, 0, period, duty_percent);
	
	}
}

int main () {
	configure_gpio_interrupt(13, INT_PORT_C, RISING);  // Button interrupt
	timer_PWM_Microsecond(portA, 0, period, duty_percent); // Start with 1kHz, 50%

	while (1) {
		// main loop does nothing, everything happens in interrupt
	}
}
