
/* ####################################
Author: MD. Soyabbir Abu Hanif
 MSc in Embedded Systems
Tampere University, Finland
Email: hanifseceee535@gmail.com 
######################################
*/

/*
********************************************************** 
********************___Delay_Microsecond___***********************
**********************************************************
@Function name: 
	void delay_microSecond(uint8_t timer, uint16_t delay, uint32_t clock_freq)

@Purpose:
	Polling Delays program execution for a specified number of microseconds using a hardware timer.

@Parameters:

		Name			Type			Description
		timer		uint8_t				Timer number (e.g., 2, 3, 4, etc.)
		delay		uint16_t			Delay duration in microseconds
		clock_freq	uint32_t			System clock frequency in Hz

@Return Value: None

@Behavior:

	Configures the specified timer in polling mode.

	Calculates and loads appropriate prescaler and auto-reload values.

	Waits for the timer to elapse before continuing execution.

@Example:
	delay_microSecond(2, 50, 48000000); // Delays for 50 µs using Timer 2 at 48 MHz



********************************************************** 
********************___Delay_Milliseconds___***********************
**********************************************************
@Function name: 
	void delay_miliSecond(uint8_t timer, uint16_t delay, uint32_t clock_freq)

@Purpose:
	Polling Delays program execution for a specified number of microseconds using a hardware timer.

@Parameters:

		Name			Type			Description
		timer		uint8_t				Timer number (e.g., 2, 3, 4, etc.)
		delay		uint16_t			Delay duration in milliseconds
		clock_freq	uint32_t			System clock frequency in Hz

@Return Value: None

@Behavior:

	Configures the specified timer in polling mode.

	Calculates and loads appropriate prescaler and auto-reload values.

	Waits for the timer to elapse before continuing execution.

@Example:
	delay_miliSecond(2, 50, 48000000); // Delays for 50 ms using Timer 2 at 48 MHz




********************************************************** 
********************___Interrupts_In_Microseconds___***********************
**********************************************************
@Function name: 
	void timer_irq_microSecond_start(uint8_t timer, uint16_t delay, uint32_t clock_freq)
@Purpose:
	Starts a timer to trigger an interrupt after a specified number of microseconds.

@Parameters:

		Name			Type			Description
		timer			uint8_t			Timer number
		delay			uint16_t		Delay in microseconds before interrupt
		clock_freq		uint32_t		System clock frequency in Hz

@Return Value:None

@Behavior:

	Initializes the timer in up-counting mode.

	Configures the interrupt to fire when the delay expires.

	Enables the NVIC interrupt for the selected timer.

@Example:
	timer_irq_microSecond_start(3, 200, 48000000); // Interrupt in 200 µs using Timer 3


********************************************************** 
********************___Interrupts_In_Miliseconds___***********************
**********************************************************
@Function name: 
	void timer_irq_milisecond_start(uint8_t timer, uint16_t delay, uint32_t clock_freq)
@Purpose:
	Starts a timer to trigger an interrupt after a specified number of milliseconds.

@Parameters:
		Name			Type			Description
		timer			uint8_t			Timer number
		delay			uint16_t		Delay in milliseconds before interrupt
		clock_freq		uint32_t		System clock frequency in Hz

@Return Value:None

@Behavior:
	Sets up the timer to generate an update event after the delay.

	Configures interrupt handling through the NVIC.

	Timer continues running or can be stopped manually.

@Example:
	timer_irq_milisecond_start(4, 10, 48000000); // Interrupt in 10 ms using Timer 4




**************************************************************************** 
********************___List of timer interrupt handlers___***********************
**************************************************************************** 

		Timer			IRQ Handler Function

		TIM1			void TIM1_UP_IRQHandler(void)
		TIM2			void TIM2_IRQHandler(void)
		TIM3			void TIM3_IRQHandler(void)
		TIM4			void TIM4_IRQHandler(void)
@example: 
	 void TIM1_UP_IRQHandler(void)
 		{
     		if (TIM1->SR & (1 << 0)) {
         		TIM1->SR &= ~(1 << 0); // Clear the update interrupt flag

         		toggle_gpio (portC, 0);
     		}
 		}

********************************************************** 
********************___stop_Timer_Interrupt___***********************
**********************************************************

@Function name: 
	void stop_timer_irq(uint8_t timer)
@Purpose:
	Stops interrupt generation for the specified timer.

@Parameters:

		Name			Type			Description
		timer			uint8_t			Timer number

@Return Value:None

@Behavior:

	Disables timer update interrupt generation.

	Clears interrupt flags.

	Leaves timer running unless stopped separately.

@Example:
	stop_timer_irq(3); // Disables interrupt from Timer 3



********************************************************** 
********************___stop_Timer___***********************
**********************************************************
@Purpose:
	Completely stops the selected timer, disabling its clock and interrupt.

@Parameters:

		Name			Type			Description
		timer			uint8_t			Timer number

@Return Value:None

@Behavior:

	Stops the counter.

	Disables timer peripheral clock.

	Disables and clears all related interrupts.

@Example:
	stop_timer(2); // Fully stops Timer 2



******************************************************************************************************************** 
********************___Timer Channels & Connected GPIO Pin List___**************************************************
******************************************************************************************************************** 
		Timer			Channel			Pin 		Options	Notes

		TIM1			CH1				PA8			Advanced timer
						CH2				PA9	
						CH3				PA10	
						CH4				PA11	

		TIM2			CH1				PA0,(AF1)	Basic/general purpose
						CH2				PA1	
						CH3				PA2	
						CH4				PA3	

		TIM3			CH1				PA6, PB4	Used often in PWM
						CH2				PA7, PB5	
						CH3				PB0	
						CH4				PB1	

		TIM4			CH1				PB6	
						CH2				PB7	
						CH3				PB8	
						CH4				PB9


********************************************************** 
********************___Timer_Compare_MHz___***********************
**********************************************************
@Function name: 
	void timer_compare_MhZ(uint32_t port, uint8_t pin, uint16_t Load_value, uint16_t compare_value, uint32_t clock_freq)
@Purpose:
	Generates a high-frequency output compare signal in the MHz range on a specific GPIO pin.

@Parameters:

		Name				Type			Description
		port				uint32_t		GPIO port number. for example portA, portB, porC
		pin					uint8_t			GPIO pin number
		Load_value			uint16_t		Auto-reload value (defines signal frequency)
		compare_value		uint16_t		Compare match value (defines duty cycle timing)
		clock_freq			uint32_t		System clock frequency in Hz

@Return Value:None

@Behavior:

	Configures GPIO pin in alternate function mode.

	Sets timer to output compare mode with given frequency and compare value.

	Outputs a high-speed toggling signal.

@Example:
	timer_compare_MhZ(portA, 6, 24, 12, 48000000); // ~1 MHz square wave on PA6



********************************************************** 
********************___Timer_Compare_KHZ___***********************
**********************************************************
@Function name: 
	void timer_compare_khZ(uint32_t port, uint8_t pin, uint16_t Load_value, uint16_t compare_value, uint32_t clock_freq)
@Purpose:
	Generates an output compare signal in the kHz range on a specific GPIO pin.

@Parameters:

	Name			Type			Description
	port			uint32_t		GPIO port number. for example portA, portB, porC
	pin				uint8_t			GPIO pin number
	Load_value		uint16_t		Auto-reload value (defines signal frequency)
	compare_value	uint16_t		Compare match value (defines duty cycle timing)
	clock_freq		uint32_t		System clock frequency in Hz

@Return Value:None

@Behavior:

	Similar to timer_compare_MhZ, but suitable for lower-frequency kHz-range signals.

	Timer and GPIO configured for moderate-speed toggling output.

@Example:
	timer_compare_khZ(portB, 10, 4800, 2400, 48000000); // ~10 kHz on PB10



********************************************************** 
********************___Timer_PWM_MHZ___***********************
**********************************************************
@Function name: 
	void timer_PWM_Microsecond(uint32_t port, uint8_t pin, uint16_t period, uint16_t duty_cycle_percentage, uint32_t clock_freq)
@Purpose:
	Generates a PWM signal with microsecond resolution on a specified pin using hardware timer.

@Parameters:

		Name						Type			Description
		port						uint32_t		GPIO port number. for example portA, portB, porC
		pin							uint8_t			GPIO pin number
		period						uint16_t		PWM period in microseconds
		duty_cycle_percentage		uint16_t		Duty cycle in percentage (0–100)
		clock_freq					uint32_t		System clock frequency in Hz

@Return Value:None

@Behavior:

	Sets timer to run in PWM mode with precise µs resolution.

	Sets duty cycle as percentage of the total period.

	Outputs PWM waveform on selected GPIO pin.

@Example:
	timer_PWM_Microsecond(portA, 1, 100, 25, 48000000); // 100 µs period, 25% duty on PA1


********************************************************** 
********************___Timer_PWM_KHZ___***********************
**********************************************************
@Function name: 
	void timer_PWM_Milisecond(uint32_t port, uint8_t pin, uint16_t period, uint16_t duty_cycle_percentage, uint32_t clock_freq)
@Purpose:
	Generates a PWM signal with millisecond resolution on a specified pin using hardware timer.

@Parameters:

		Name						Type			Description
		port						uint32_t		GPIO port number. for example portA, portB, porC
		pin							uint8_t			GPIO pin number
		period						uint16_t		PWM period in milliseconds
		duty_cycle_percentage		uint16_t		Duty cycle in percentage (0–100)
		clock_freq					uint32_t		System clock frequency in Hz

@Return Value:None

@Behavior:

	Uses timer PWM mode with slower timing for human-visible signals.

	Suitable for servos, LEDs, motors, etc.

@Example:
	timer_PWM_Milisecond(portA, 8, 20, 50, 48000000); // 20 ms, 50% duty on PA8


********************************************************** 
********************___update_period___***********************
**********************************************************
@Function name: 
	void update_period(uint32_t port, uint8_t pin, uint16_t new_period, uint16_t duty_percent)
@Purpose:
	Dynamically updates the period of an active PWM signal and adjusts its duty cycle accordingly.

@Parameters:

		Name				Type				Description
		port				uint32_t			GPIO port number. for example portA, portB, porC
		pin					uint8_t				GPIO pin number
		new_period			uint16_t			New period in milliseconds
		duty_percent		uint16_t			New duty cycle percentage (0–100)

@Return Value:None

@Behavior:

	Recalculates ARR and CCRx values for new timing.

	Applies changes without stopping the timer.

@Example:
	update_period(portA, 8, 10, 75); // 10 ms period, 75% duty on PA8


********************************************************** 
********************___update_Duty_Cycle___***********************
**********************************************************

@Function name: 
	void update_duty(uint32_t port, uint8_t pin, uint16_t duty_percent)
@Purpose:
	Dynamically updates the duty cycle of an active PWM signal without changing its period.

@Parameters:

		Name				Type			Description
		port				uint32_t		GPIO port number. for example portA, portB, porC
		pin					uint8_t			GPIO pin number
		duty_percent		uint16_t		New duty cycle percentage (0–100)

@Return Value:None

@Behavior:

	Locates CCR register for associated timer/channel.

	Recomputes and updates the compare value accordingly.

@Example:
	update_duty(portA, 8, 90); // Sets duty cycle to 90% on PA8





**********************************************************/






#include <stdint.h>
#include "stm32f103Driver.h"

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
void start_timer_in_Microsecond(uint8_t timer_number, uint16_t delay,uint32_t clock_freq){
	uint16_t prescaller = (clock_freq/ 1000000) -1 ; // 72 MHz / 72 = 1 MHz -> 1us tick
	if (timer_number == 1) {
		RCC->APB2ENR |= 1 << 11; // Enable Timer 1 clock
	} else {
		RCC->APB1ENR |= 1 << (timer_number - 2); // Enable Timer 2-4 clock
	}
	Timer_TypeDef *timer = select_timer(timer_number);
	timer->CR1 &= ~(1 << 0);    // Disable the timer
	timer->CNT = 0;
	timer->PSC = prescaller;
	timer->ARR = (delay) - 1; 
	timer->EGR = 1;          // Force update event
	timer->SR = 0;    // Clear update interrupt flag (UIF)
	timer->CR1 |= (1 << 0);

}

//  Start timer to count milliseconds
void start_timer_in_Milisecond(uint8_t timer_number, uint16_t delay,uint32_t clock_freq){
	uint32_t prescaller = ((clock_freq/ 2000)) -1 ; // for example:  72 MHz / 2000 = 36000 =>2 kHz => .5 ms tick
	if (timer_number == 1) {
		RCC->APB2ENR |= 1 << 11;
	} else {
		RCC->APB1ENR |= 1 << (timer_number - 2);
	}
	Timer_TypeDef *timer = select_timer(timer_number);
	timer->CR1 &= ~(1 << 0);    // Disable the timer
	timer->CNT = 0;
	timer->PSC = prescaller;
	timer->ARR = (delay*2) - 1; 
	timer->EGR = 1;          // Force update event
	timer->SR = 0;    // Clear update interrupt flag (UIF)
	timer->CR1 |= (1 << 0);
	
}

//Delay execution by given microseconds
void delay_microSecond(uint8_t timer, uint16_t delay,uint32_t clock_freq){
	Timer_TypeDef *timerx = select_timer(timer);
	start_timer_in_Microsecond(timer, delay,clock_freq);
	while (!(timerx->SR & (1 << 0))); // Wait for update event
	timerx->SR &= ~(1 << 0); // Clear update flag
}

// Delay execution by given milliseconds
void delay_miliSecond(uint8_t timer, uint16_t delay,uint32_t clock_freq){
	Timer_TypeDef *timerx = select_timer(timer);
	start_timer_in_Milisecond(timer, delay,clock_freq);
	while (!(timerx->SR & (1 << 0))); // Wait for update
	timerx->SR &= ~(1 << 0); // Clear update flag
}

// Start timer with interrupt in microseconds
void timer_irq_microSecond_start(uint8_t timer, uint16_t delay,uint32_t clock_freq){
	Timer_TypeDef *timerx = select_timer(timer);
	start_timer_in_Microsecond(timer, delay,clock_freq);
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
void timer_irq_milisecond_start(uint8_t timer, uint16_t delay,uint32_t clock_freq){
	Timer_TypeDef *timerx = select_timer(timer);
	start_timer_in_Milisecond(timer, delay,clock_freq);
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
void timer_compare_MhZ(uint32_t port, uint8_t pin, uint16_t Load_value, uint16_t compare_value,uint32_t clock_freq){
	uint16_t prescaller = (clock_freq/ 1000000) -1 ;
	uint8_t timer_number = select_timer_for_capture_PWM(port, pin);
	uint8_t channel = select_timer_channel(port, pin);
	Timer_TypeDef *timerx = select_timer(timer_number);

	// Enable clock for corresponding timer
	if (timer_number == 1)
		RCC->APB2ENR |= 1 << 11;
	else
		RCC->APB1ENR |= 1 << (timer_number - 2);

	Config_GPIO(port, pin, output_50Mhz, af_pp_output); // Setup pin as alternate output
	timerx->PSC = prescaller; // 1us tick

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
void timer_compare_khZ(uint32_t port, uint8_t pin, uint16_t Load_value, uint16_t compare_value,uint32_t clock_freq){
	uint32_t prescaller = ((clock_freq/ 2000)) -1 ;
	uint8_t timer_number = select_timer_for_capture_PWM(port, pin);
	uint8_t channel = select_timer_channel(port, pin);
	Timer_TypeDef *timerx = select_timer(timer_number);

	if (timer_number == 1)
		RCC->APB2ENR |= 1 << 11;
	else
		RCC->APB1ENR |= 1 << (timer_number - 2);

	Config_GPIO(port, pin, output_50Mhz, af_pp_output);
	timerx->PSC = prescaller; // 2kHz tick

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
void timer_PWM_Microsecond(uint32_t port, uint8_t pin, uint16_t period, uint16_t duty_cycle_percentage,uint32_t clock_freq){
	uint16_t prescaller = (clock_freq/ 1000000) -1 ;
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
	timerx->PSC = prescaller;

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
void timer_PWM_Milisecond(uint32_t port, uint8_t pin, uint16_t period, uint16_t duty_cycle_percentage,uint32_t clock_freq){
	uint32_t prescaller = ((clock_freq/ 2000)) -1 ;
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
	timerx->PSC = prescaller;

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
