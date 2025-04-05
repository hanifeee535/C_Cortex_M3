/* ####################################
Author: MD. Soyabbir Abu Hanif
 MSc in Embedded Systems
Tampere University, Finland
Email: hanifseceee535@gmail.com 
######################################
*/

#include <stdint.h>
#include "../Drivers/stm32f103Driver.h"


/**********************************************************/
/*************************__GPIO__*************************/
/**********************************************************/

// Function to select GPIO port
GPIO_TypeDef* Select_GPIO(uint8_t port) {
    switch (port) {
        case 1: return GPIOA;
        case 2: return GPIOB;
        case 3: return GPIOC;
        case 4: return GPIOD;
        case 5: return GPIOE;
        default: return 0; // Return NULL for invalid port
    }
}






// Function to configure GPIO 
void Config_GPIO(uint8_t port, uint8_t pin, uint8_t mode, uint8_t config) {
    GPIO_TypeDef *GPIOx = Select_GPIO(port);
    if (!GPIOx) return; // Return if invalid port

    // Enable the corresponding GPIO clock
    RCC->APB2ENR |= (1 << (port + 1));

    volatile uint32_t *reg = (pin < 8) ? &GPIOx->CRL : &GPIOx->CRH;
    uint8_t pos = (pin % 8) * 4;

    *reg &= ~(0xF << pos); // Clear bits
    *reg |= ((mode) << pos) | ((config) << (pos + 2)); // Set mode and config
}

//function to write into gpio pin
void Write_GPIO (uint8_t port, uint8_t pin, uint8_t state){
	
		GPIO_TypeDef *GPIOx = Select_GPIO(port);
    
		volatile uint32_t *odr = &GPIOx ->ODR;
		
		//write the desired state to the desired pin
		state ? ((*odr |= 1 << pin)) : ((*odr &= ~(1 << pin)));

	
}

//function to read a single gpio pin
uint32_t Read_GPIO_Pin (uint8_t port, uint8_t pin){
	
		GPIO_TypeDef *GPIOx = Select_GPIO(port);
    
		volatile uint32_t *idr = &GPIOx ->IDR;
		uint32_t state;
		
		//write the desired state to the desired pin
	state = *idr & (1<<pin) >>pin;
	return state;
	
}

// Function to read the entire IDR register of a GPIO port
uint32_t Read_GPIO_Port (uint8_t port) {
		GPIO_TypeDef *GPIOx = Select_GPIO(port);
		volatile uint32_t *idr = &GPIOx ->IDR;
		return *idr;
		
}

//function to toggle gpio pin
void toggle_gpio (uint8_t port, uint8_t pin){
	GPIO_TypeDef *GPIOx = Select_GPIO(port);  
  // toggle the bit in the Output Data Register (ODR)
  GPIOx->ODR ^= (1 << pin);
}


/**********************************************************/
/*************************__GPIO_EXTI__*************************/
/**********************************************************/

void configure_gpio_interrupt(uint8_t pin, uint8_t port, uint8_t trigger_type){
	__disableinterrupt();
	volatile uint32_t *EXTI_X ;
	//Configuring the RCC register for AFIO
	RCC->APB2ENR |= (1<<0);
	if (pin <4){
		EXTI_X = &AFIO->EXTI_1;
	}
	else if (pin <8){
		EXTI_X = &AFIO->EXTI_2;
	}
	else if (pin <12){
		EXTI_X = &AFIO->EXTI_3;
	}
	else { EXTI_X = &AFIO->EXTI_4; }
	
	*EXTI_X &= ~(0xF << ((pin % 4) * 4));  // Clear the 4 bits for the line
	*EXTI_X  |= (port << ((pin % 4) * 4));  // Set the new port value
	
	//enable the interrupt mask for the corresponding EXTI line
	EXTI->IMR |= (1<<pin);
	
	//configuring the trigger type
	if (trigger_type == 0) { //rising edge
		EXTI->RTSR |= (1 << pin); 
		EXTI->FTSR &= ~(1 << pin);  // Ensure falling edge is disabled
	}

	else if (trigger_type == 1) { //falling edge
		EXTI->FTSR  |= (1 << pin); 
		EXTI->RTSR &= ~(1 << pin);  // Ensure rising edge is disabled
	}
	if (trigger_type == 2) { //both edge
		EXTI->RTSR |= (1 << pin); 
		EXTI->FTSR |= (1 << pin);  
	}
	
	
	//Enabling the NVIC interrupt for the exti line (lines 0–15 correspond to IRQ numbers 6–22)
	if (pin <= 4) {
        NVIC_ISER0 |= (1 << (pin + 6));
    }
		else if (pin > 4 && pin <=9){
			NVIC_ISER0 |= (1 << 23);
		}
		else if (pin > 9 && pin <=15){
			NVIC_ISER1 |= (1 << 8);
		}
	__enableinterrupt();

}




/*************************************************************************************************/
/*************************__Systic__*************************/
/*************************************************************************************************/

void systic_init(void) {
    // Disable the SysTick timer by clearing the CTRL register
    SysTick->CTRL = 0;

    // Set the reload value to the maximum possible (24-bit value)
    SysTick->LOAD = 0x00FFFFFF;

    // Clear the current value register (VAL), which resets the SysTick counter
    SysTick->VAL = 0;

    // Enable SysTick by setting the ENABLE bit (bit 0) and select the clock source
    // as the processor clock (AHB) by setting the CLKSOURCE bit (bit 2).
    // The binary value 101 corresponds to enabling the counter and selecting the clock source.
    SysTick->CTRL |= (101 << 0);
}


void Sys_delay_microsecond() {
    // Set the reload value to 72. Assuming the AHB clock is 72 MHz, this provides
    // a 1 microsecond delay since the SysTick counter decrements by 1 per clock cycle.
    SysTick->LOAD = 72;

    // Clear the current value register (VAL), which resets the counter to zero.
    SysTick->VAL = 0;

    // Wait for the COUNTFLAG (bit 16 of CTRL) to be set, indicating the counter has counted down to 0.
    while ((SysTick->CTRL & 0x00010000) == 0);
}

void Sys_Delay_Milisecond () {
	SysTick->LOAD = 72000;
	SysTick->VAL = 0;
	 while ((SysTick->CTRL & 0x00010000) == 0);
}

void Delay_Sys_US (uint16_t t){
	for (;t>0;t--){
		Sys_delay_microsecond ();
	}
}

void Delay_Sys_MS (uint16_t t) {
	for (;t>0;t--){
		Sys_Delay_Milisecond ();
	}
}

void Systic_interrupt (){
	__disableinterrupt();
	SysTick->CTRL = 0;
	SysTick->LOAD = 7200000-1; //for 0.1 second delay
	SysTick->VAL = 0;
	SysTick ->CTRL |= (111<<0);
	__enableinterrupt();
}





/*************************************************************************************************/
/*************************__Timers__*************************/
/*************************************************************************************************/

// Select timer instance based on timer number
Timer_TypeDef* select_timer(uint8_t timer_number) {
    switch (timer_number) {
        case 1: return TIM1; // Timer 1
        case 2: return TIM2; // Timer 2
        case 3: return TIM3; // Timer 3
        case 4: return TIM4; // Timer 4
        default: return 0;   // Invalid timer
    }
}

// Select appropriate timer for capture/PWM based on GPIO port and pin
uint8_t select_timer_for_capture_PWM (uint8_t port, uint8_t pin){
	if (port == 1){ // Port A
		if (pin == 8 || pin == 9 || pin == 10 || pin == 11){
			return 1; // Timer 1
		} else if (pin == 0 || pin == 1 || pin == 2 || pin == 3){
			return 2; // Timer 2
		} else if (pin == 6 || pin == 7 ){
			return 3; // Timer 3
		} else { return 0; } // Invalid pin
	} else if (port == 2) { // Port B
		if (pin == 0 || pin == 1){
			return 3; // Timer 3
		} else if (pin == 6 || pin == 7 || pin == 8 || pin == 9){
			return 4; // Timer 4
		} else { return 0; } // Invalid pin
	}
	return 0; // Invalid port
}

// Select timer channel based on port and pin
uint8_t select_timer_channel(uint8_t port, uint8_t pin){
	if (port == 1) {
		switch (pin) {
			case 0: return 1; case 1: return 2; case 2: return 3; case 3: return 4;
			case 6: return 1; case 7: return 2; case 8: return 1; case 9: return 2;
			case 10: return 3; case 11: return 4;
		}
		return 0;
	} else if (port == 2) {
		switch (pin) {
			case 0: return 3; case 1: return 4;
			case 6: return 1; case 7: return 2;
			case 8: return 3; case 9: return 4;
		}
		return 0;
	}
	return 0;
}

// Start timer to count microseconds
void start_timer_in_Microsecond(uint8_t timer_number, uint16_t delay){
	uint16_t prescaller = 72 - 1; // 72 MHz / 72 = 1 MHz -> 1us tick
	if (timer_number == 1) {
		RCC->APB2ENR |= 1 << 11; // Enable Timer 1 clock
	} else {
		RCC->APB1ENR |= 1 << (timer_number - 2); // Enable Timer 2-4 clock
	}
	Timer_TypeDef *timer = select_timer(timer_number);
	timer->CNT = 0;             // Reset counter
	timer->PSC = prescaller;    // Set prescaler
	timer->ARR = delay - 1;     // Auto-reload value
	timer->CR1 |= (1 << 0);     // Enable counter
}

//  Start timer to count milliseconds
void start_timer_in_Milisecond(uint8_t timer_number, uint16_t delay){
	uint16_t prescaller = 36000 - 1; // 72 MHz / 36000 = 2 kHz => 0.5 ms tick
	if (timer_number == 1) {
		RCC->APB2ENR |= 1 << 11;
	} else {
		RCC->APB1ENR |= 1 << (timer_number - 2);
	}
	Timer_TypeDef *timer = select_timer(timer_number);
	timer->CNT = 0;
	timer->PSC = prescaller;
	timer->ARR = (delay * 2) - 1; // Convert to ms (since tick = 0.5ms)
	timer->CR1 |= (1 << 0);
}

//Delay execution by given microseconds
void delay_microSecond(uint8_t timer, uint16_t delay){
	Timer_TypeDef *timerx = select_timer(timer);
	start_timer_in_Microsecond(timer, delay);
	while (!(timerx->SR & (1 << 0))); // Wait for update event
	timerx->SR &= ~(1 << 0); // Clear update flag
}

// Delay execution by given milliseconds
void delay_miliSecond(uint8_t timer, uint16_t delay){
	Timer_TypeDef *timerx = select_timer(timer);
	start_timer_in_Milisecond(timer, delay);
	while (!(timerx->SR & (1 << 0))); // Wait for update
	timerx->SR &= ~(1 << 0); // Clear update flag
}

// Start timer with interrupt in microseconds
void timer_irq_microSecond_start(uint8_t timer, uint16_t delay){
	Timer_TypeDef *timerx = select_timer(timer);
	start_timer_in_Microsecond(timer, delay);
	timerx->DIER |= (1 << 0); // Enable update interrupt
	__disableinterrupt();
	switch (timer) {
		case 1: NVIC_ISER0 |= (1 << 25); break;
		case 2: NVIC_ISER0 |= (1 << 28); break;
		case 3: NVIC_ISER0 |= (1 << 29); break;
		case 4: NVIC_ISER0 |= (1 << 30); break;
	}
	__enableinterrupt();
}

//  Start timer with interrupt in milliseconds
void timer_irq_milisecond_start(uint8_t timer, uint16_t delay){
	Timer_TypeDef *timerx = select_timer(timer);
	start_timer_in_Milisecond(timer, delay);
	timerx->DIER |= (1 << 0);
	__disableinterrupt();
	switch (timer) {
		case 1: NVIC_ISER0 |= (1 << 25); break;
		case 2: NVIC_ISER0 |= (1 << 28); break;
		case 3: NVIC_ISER0 |= (1 << 29); break;
		case 4: NVIC_ISER0 |= (1 << 30); break;
	}
	__enableinterrupt();
}

// Stop timer interrupt and disable NVIC
void stop_timer_irq(uint8_t timer){
	Timer_TypeDef *timerx = select_timer(timer);
	timerx->DIER &= ~(1 << 0); // Disable update interrupt
	__disableinterrupt();
	switch (timer) {
		case 1: NVIC_ISER0 &= ~(1 << 25); break;
		case 2: NVIC_ISER0 &= ~(1 << 28); break;
		case 3: NVIC_ISER0 &= ~(1 << 29); break;
		case 4: NVIC_ISER0 &= ~(1 << 30); break;
	}
	__enableinterrupt();
}

// Stop timer and disable its clock
void stop_timer(uint8_t timer){
	Timer_TypeDef *timerx = select_timer(timer);
	timerx->CR1 &= ~(1 << 0); // Disable counter
	if (timer == 1) {
		RCC->APB2ENR &= ~(1 << 11); // Disable Timer 1 clock
	} else {
		RCC->APB1ENR &= ~(1 << (timer - 2)); // Disable Timer 2-4
	}
	stop_timer_irq(timer); // Also stop IRQ if active
}

// Generate compare output signal in MHz
void timer_compare_MhZ(uint32_t port, uint8_t pin, uint16_t Load_value, uint16_t compare_value){
	uint8_t timer_number = select_timer_for_capture_PWM(port, pin);
	uint8_t channel = select_timer_channel(port, pin);
	Timer_TypeDef *timerx = select_timer(timer_number);

	// Enable clock for corresponding timer
	if (timer_number == 1)
		RCC->APB2ENR |= 1 << 11;
	else
		RCC->APB1ENR |= 1 << (timer_number - 2);

	Config_GPIO(port, pin, output_50Mhz, af_pp_output); // Setup pin as alternate output
	timerx->PSC = 72 - 1; // 1us tick

	// Configure channel output compare mode
	switch (channel) {
		case 1: timerx->CCMR1 |= 0x30; break;
		case 2: timerx->CCMR1 |= 0x3000; break;
		case 3: timerx->CCMR2 |= 0x30; break;
		case 4: timerx->CCMR2 |= 0x3000; break;
	}
	timerx->CCER |= (1 << ((channel - 1) * 4)); // Enable channel output
	timerx->BDTR |= (1 << 15); // Main output enable (for advanced timer)
	timerx->CR1 |= (1 << 0);   // Start timer

	timerx->ARR = Load_value; // Set period
	switch (channel) {
		case 1: timerx->CCR1 = compare_value; break;
		case 2: timerx->CCR2 = compare_value; break;
		case 3: timerx->CCR3 = compare_value; break;
		case 4: timerx->CCR4 = compare_value; break;
	}
}

//Generate compare output signal in kHz
void timer_compare_khZ(uint32_t port, uint8_t pin, uint16_t Load_value, uint16_t compare_value){
	uint8_t timer_number = select_timer_for_capture_PWM(port, pin);
	uint8_t channel = select_timer_channel(port, pin);
	Timer_TypeDef *timerx = select_timer(timer_number);

	if (timer_number == 1)
		RCC->APB2ENR |= 1 << 11;
	else
		RCC->APB1ENR |= 1 << (timer_number - 2);

	Config_GPIO(port, pin, output_50Mhz, af_pp_output);
	timerx->PSC = 36000 - 1; // 2kHz tick

	switch (channel) {
		case 1: timerx->CCMR1 |= 0x30; break;
		case 2: timerx->CCMR1 |= 0x3000; break;
		case 3: timerx->CCMR2 |= 0x30; break;
		case 4: timerx->CCMR2 |= 0x3000; break;
	}
	timerx->CCER |= (1 << ((channel - 1) * 4));
	timerx->BDTR |= (1 << 15);
	timerx->CR1 |= (1 << 0);

	timerx->ARR = (Load_value * 2) - 1;
	compare_value *= 2;
	switch (channel) {
		case 1: timerx->CCR1 = compare_value; break;
		case 2: timerx->CCR2 = compare_value; break;
		case 3: timerx->CCR3 = compare_value; break;
		case 4: timerx->CCR4 = compare_value; break;
	}
}

// Generate PWM output in microsecond time base
void timer_PWM_Microsecond(uint32_t port, uint8_t pin, uint16_t period, uint16_t duty_cycle_percentage){
	uint8_t timer_number = select_timer_for_capture_PWM(port, pin);
	uint8_t channel = select_timer_channel(port, pin);
	Timer_TypeDef *timerx = select_timer(timer_number);

	float duty_coeff = duty_cycle_percentage / 100.0f;
	uint16_t ccr = (uint16_t)(duty_coeff * period + 0.5f);

	if (timer_number == 1)
		RCC->APB2ENR |= 1 << 11;
	else
		RCC->APB1ENR |= 1 << (timer_number - 2);

	Config_GPIO(port, pin, output_50Mhz, af_pp_output);
	timerx->PSC = 72 - 1;

	switch (channel) {
		case 1: timerx->CCMR1 |= 0x60; break;
		case 2: timerx->CCMR1 |= 0x6000; break;
		case 3: timerx->CCMR2 |= 0x60; break;
		case 4: timerx->CCMR2 |= 0x6000; break;
	}
	timerx->CCER |= (1 << ((channel - 1) * 4));
	timerx->BDTR |= (1 << 15);
	timerx->CR1 |= (1 << 0);

	timerx->ARR = period - 1;
	switch (channel) {
		case 1: timerx->CCR1 = ccr; break;
		case 2: timerx->CCR2 = ccr; break;
		case 3: timerx->CCR3 = ccr; break;
		case 4: timerx->CCR4 = ccr; break;
	}
}

// Generate PWM output in millisecond time base
void timer_PWM_Milisecond(uint32_t port, uint8_t pin, uint16_t period, uint16_t duty_cycle_percentage){
	uint8_t timer_number = select_timer_for_capture_PWM(port, pin);
	uint8_t channel = select_timer_channel(port, pin);
	Timer_TypeDef *timerx = select_timer(timer_number);

	float duty_coeff = duty_cycle_percentage / 100.0f;
	uint16_t ccr = (uint16_t)(duty_coeff * period + 0.5f);

	if (timer_number == 1)
		RCC->APB2ENR |= 1 << 11;
	else
		RCC->APB1ENR |= 1 << (timer_number - 2);

	Config_GPIO(port, pin, output_50Mhz, af_pp_output);
	timerx->PSC = 36000 - 1;

	switch (channel) {
		case 1: timerx->CCMR1 |= 0x60; break;
		case 2: timerx->CCMR1 |= 0x6000; break;
		case 3: timerx->CCMR2 |= 0x60; break;
		case 4: timerx->CCMR2 |= 0x6000; break;
	}
	timerx->CCER |= (1 << ((channel - 1) * 4));
	timerx->BDTR |= (1 << 15);
	timerx->CR1 |= (1 << 0);

	timerx->ARR = (period * 2) - 1;
	ccr *= 2;
	switch (channel) {
		case 1: timerx->CCR1 = ccr; break;
		case 2: timerx->CCR2 = ccr; break;
		case 3: timerx->CCR3 = ccr; break;
		case 4: timerx->CCR4 = ccr; break;
	}
}

//Dynamic Duty Cycle Update Function 
void update_duty(uint32_t port, uint8_t pin, uint16_t duty_percent){
    uint8_t timer_number = select_timer_for_capture_PWM(port, pin);
    uint8_t channel = select_timer_channel(port, pin);
    Timer_TypeDef *timerx = select_timer(timer_number);

    uint16_t period = timerx->ARR + 1;
    uint16_t ccr = (period * duty_percent) / 100;

    switch (channel) {
        case 1: timerx->CCR1 = ccr; break;
        case 2: timerx->CCR2 = ccr; break;
        case 3: timerx->CCR3 = ccr; break;
        case 4: timerx->CCR4 = ccr; break;
    }
}

//Dynamic Period Update Function 
void update_period(uint32_t port, uint8_t pin, uint16_t new_period, uint16_t duty_percent){
    uint8_t timer_number = select_timer_for_capture_PWM(port, pin);
    uint8_t channel = select_timer_channel(port, pin);
    Timer_TypeDef *timerx = select_timer(timer_number);

    // Apply millisecond scaling (2x) as in timer_PWM_Milisecond
    uint16_t scaled_period = (new_period * 2) - 1;
    uint16_t scaled_ccr = (new_period * 2 * duty_percent) / 100;
		timerx->CR1 &= ~(1 << 0);
    timerx->ARR = scaled_period;
		timerx->CNT = 0;

    switch (channel) {
        case 1: timerx->CCR1 = scaled_ccr; break;
        case 2: timerx->CCR2 = scaled_ccr; break;
        case 3: timerx->CCR3 = scaled_ccr; break;
        case 4: timerx->CCR4 = scaled_ccr; break;
    }
		timerx->CR1 |= (1 << 0);
}



/*************************************************************************************************/
/*************************__USART__*************************/
/*************************************************************************************************/

/********************************
pis for USART: 
 USART3 -> PB10 (Tx) and PB11(Rx)
 USART2 -> PA2 (Tx) and PA3(Rx)
 USART1 -> PA9 (Tx) and PA10(Rx)
 
 IRQ handlers: 
 
 USART1_IRQHandler  
 USART2_IRQHandler  
 USART3_IRQHandler  

*/
USART_TypeDef* select_USART(uint8_t usart) {
    switch (usart) {
        case 1:
            return USART1;  // Return USART1 if usart == 1
        case 2:
            return USART2;  // Return USART2 if usart == 2
        case 3:
            return USART3;  // Return USART3 if usart == 3
        default:
            return 0;    // Return NULL if usart is invalid (not 1, 2, or 3)
    }
}

uint32_t USART_BRR(uint16_t usart, uint32_t baud_rate)
{
    // The BRR (Baud Rate Register) configures the USART baud rate.
    // It consists of two parts:
    //   - Mantissa (upper 12 bits): Integer part of USARTDIV.
    //   - Fraction (lower 4 bits): Fractional part of USARTDIV scaled by 16.

    // Initialize variables
    uint32_t peripheral_clock = 36000000UL;  // Default peripheral clock (36 MHz)
    uint32_t mantissa;                        // Integer part of USARTDIV (mantissa)
    uint32_t final_brr_value;                 // Final value for the BRR register
    double fractional_part = 36000000.00;     // Peripheral clock as a floating-point number
    double rounding_check = 1.00;             // Temporary value for checking rounding decision
    
    // Check if USART1 is selected, which operates at a different clock frequency
    if (usart == 1)
    {
        peripheral_clock = 72000000UL;       // Peripheral clock for USART1 (72 MHz)
        fractional_part = 72000000.00;       // Update the clock for calculations
    }
    
    // Calculate the integer part of USARTDIV (mantissa)
    mantissa = peripheral_clock / (baud_rate * 16);  // Integer division to get the mantissa

    // Calculate the fractional part of USARTDIV
    fractional_part = 16 * ((fractional_part / (baud_rate * 16)) - mantissa); // Get fractional part in base-16
    uint32_t fraction = fractional_part;  // Extract the integer part of the fractional value (truncated)
    
    // Determine if rounding is needed based on the remaining fractional part
    rounding_check = 100 * (fractional_part - fraction);  // Check if the fractional part exceeds 0.5
    if (rounding_check > 50)  // If fractional part is greater than 0.5, round up
    {
        fraction++;  // Round up the fractional part
        
        if (fraction == 16)  // If rounding causes the fractional part to overflow (i.e., 16)
        {
            fraction = 0;  // Reset fractional part to 0
            mantissa++;    // Increment the mantissa (carry over the overflow)
        }
    }
    
    // Combine the mantissa and fractional parts into the final BRR register value
    final_brr_value = (mantissa << 4);  // Shift mantissa (integer part) to the upper 12 bits
    final_brr_value += fraction;        // Add the fractional part (lower 4 bits)
    
    // Return the computed USART_BRR value
    return final_brr_value;
}


void init_USART ( uint8_t usart, uint32_t baud_rate ){
	
	USART_TypeDef *USARTX = select_USART (usart);
	uint32_t USART_BRR_cal = USART_BRR(usart, baud_rate);
	
	//Enabling the RCC clock for usart 
	//and conifuring the tx pin to alternative function and rx to input pull up/pull down	
	if (usart == 1) {
		RCC->APB2ENR |= (1<<14);
		Config_GPIO (portA, 9, output_50Mhz, af_pp_output);
		Config_GPIO (portA, 10, input , pp_in);
	}
	else if (usart == 2) {
		RCC->APB1ENR |= (1<<17);
		Config_GPIO (portA, 2, output_50Mhz, af_pp_output);
		Config_GPIO (portA, 3, input , pp_in);
	}
	else if (usart == 3) {
		RCC->APB1ENR |= (1<<18);
		Config_GPIO (portB, 10, output_50Mhz, af_pp_output);
		Config_GPIO (portB, 11, input , pp_in);
	}
	USARTX->BRR = USART_BRR_cal;
	USARTX->CR1 |=(1 << 13);  // Set bit 13 (UE - USART Enable)
	USARTX->CR1 |= (1 << 3);   // Set bit 3 (RE - Receiver Enable)
	USARTX->CR1 |= (1 << 2);   // Set bit 2 (TE - Transmitter Enable)
	
}

char USART_receive (uint8_t usart){
	USART_TypeDef *USARTX = select_USART (usart);
	char received_char; // Variable to store the received character
	
	// Wait for the RXNE (Receive Data Register Not Empty) flag to be set
    // RXNE is bit 5 of the Status Register (SR)
    while ((USARTX->SR & (1 << 5)) == 0) {
        // Busy wait until data is ready to be read
    }

    // Read the data from the Data Register (DR)
    received_char = USARTX->DR;

    // Return the received character
    return received_char;
	
}

void USART_transmit(uint8_t usart, char c) {
    USART_TypeDef *USARTX = select_USART(usart);  // Select the appropriate USART instance   
    // Wait for the TXE (Transmit Data Register Empty) flag to be set
    // TXE is bit 7 of the Status Register (SR)
    while ((USARTX->SR & (1 << 7)) == 0) {
        // Busy wait until the transmit buffer is empty
    }

    // Write the character to the Data Register (DR)
    USARTX->DR = c;
}

void init_usart_receive_interrupt(uint8_t usart, uint32_t baud_rate) {
    // Initialize USART (enable clock, set GPIO, configure baud rate, enable USART)
		// Select the appropriate USART instance
    USART_TypeDef *USARTX = select_USART(usart);
    init_USART(usart, baud_rate);   
	
	__disableinterrupt();
    // Enable RXNEIE (Receive Data Register Not Empty Interrupt Enable)
    USARTX->CR1 |= (1 << 5);

    // Enable the USART interrupt in the NVIC
    if (usart == 1) {
        NVIC_ISER1 |= (1 << 5);  // Enable interrupt for USART1
    } else if (usart == 2) {
        NVIC_ISER1 |= (1 << 6);  // Enable interrupt for USART2
    } else if (usart == 3) {
        NVIC_ISER1 |= (1 << 7);  // Enable interrupt for USART3
    }
	__enableinterrupt();
}

void init_usart_transmit_interrupt(uint8_t usart, uint32_t baud_rate) {
    // Initialize USART (enable clock, set GPIO, configure baud rate, enable USART)
    USART_TypeDef *USARTX = select_USART(usart);
    init_USART(usart, baud_rate);
    __disableinterrupt();
    // Enable TXEIE (Transmit Data Register Empty Interrupt Enable)
    USARTX->CR1 |= (1 << 7);

    // Enable the USART interrupt in the NVIC
    if (usart == 1) {
        NVIC_ISER1 |= (1 << 5);  // Enable interrupt for USART1
    } else if (usart == 2) {
        NVIC_ISER1 |= (1 << 6);  // Enable interrupt for USART2
    } else if (usart == 3) {
        NVIC_ISER1 |= (1 << 7);  // Enable interrupt for USART3
    }
		__enableinterrupt();
}

	



