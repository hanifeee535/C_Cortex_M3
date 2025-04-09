/* ####################################
Author: MD. Soyabbir Abu Hanif
 MSc in Embedded Systems
Tampere University, Finland
Email: hanifseceee535@gmail.com 
######################################
*/


#include <stdint.h>    
#include "header.h"
//#include "stm32f10x.h"

// Function to select the timer based on input
Timer_TypeDef* select_timer(uint8_t timer_number) {
    switch (timer_number) {
        case 1: return TIM1;
        case 2: return TIM2;
        case 3: return TIM3;
        case 4: return TIM4;
        default: return 0; // Invalid timer number
    }
}

uint8_t select_timer_for_capture_PWM (uint8_t port, uint8_t pin){
	if (port ==1){ //port a
		if (pin == 8 || pin == 9 || pin == 10 || pin == 11){
			return 1; //timer 1
		}
		else if (pin == 0 || pin == 1 || pin == 2 || pin == 3){
			return 2; //timer 2
		}
		else if (pin == 6 || pin == 7 ){
			return 3; //timer 3
		}
		else {return 0;}
	}
	
	else if (port ==2) { //port B
		if (pin == 0 || pin == 1){
			return 3; // timer 3
		}
		else if (pin == 6 || pin == 7 || pin == 8 || pin == 9){
			return 4; // timer 4
		}
		else {return 0;}
	
	}
	
	else return 0; 
}

uint8_t  select_timer_channel( uint8_t port, uint8_t pin ){
	if (port == 1) { //port a
		switch (pin) {
		case 0 : return 1; break;
		case 1 : return 2; break;
		case 2 : return 3; break;
		case 3 : return 4; break;
		case 6 : return 1; break;
		case 7 : return 2; break;
		case 8 : return 1; break;
		case 9 : return 2; break;
		case 10 : return 3; break;
		case 11 : return 4; break;
		
	}
		return 0;
	}
 else if (port == 2) { //port b
	 switch (pin) {
		 case 0 : return 3; break; 
		 case 1 : return 4; break; 
		 case 6 : return 1; break; 
		 case 7 : return 2; break; 
		 case 8 : return 3; break; 
		 case 9 : return 4; break; 
	 }
	 return 0; 
	
 }
 else {return 0;}
}


void start_timer_in_Microsecond (uint8_t timer_number, uint16_t delay){
	 uint16_t prescaller = 72-1;
	 // Enable the appropriate timer clock
    if (timer_number == 1) {
        RCCAPB2 |= 1 << 11; // Enable APB2 bus for Timer 1
    } else {
        RCCAPB1 |= 1 << (timer_number - 2); // Enable APB1 bus for Timer 2, 3, or 4
    }
	 Timer_TypeDef *timer = select_timer(timer_number);
	 timer->CNT = 0;
	 timer->PSC = prescaller;
	 timer->ARR = delay-1;
	 timer->CR1 |= (1<<0);
	
}

void start_timer_in_Milisecond (uint8_t timer_number, uint16_t delay){
	 uint16_t prescaller = 36000-1;
	 // Enable the appropriate timer clock
    if (timer_number == 1) {
        RCCAPB2 |= 1 << 11; // Enable APB2 bus for Timer 1
    } else {
        RCCAPB1 |= 1 << (timer_number - 2); // Enable APB1 bus for Timer 2, 3, or 4
    }
	 Timer_TypeDef *timer = select_timer(timer_number);
	 timer->CNT = 0;
	 timer->PSC = prescaller;
	 timer->ARR = (delay*2)-1;
	 timer->CR1 |= (1<<0);
	
}

void delay_microSecond (uint8_t timer, uint16_t delay){
	Timer_TypeDef *timerx = select_timer(timer);
	start_timer_in_Microsecond(timer, delay);
	
  while (!(timerx->SR & (1 << 0))) {
                // Wait until the timer reaches the ARR value
            }
            timerx->SR &= ~(1 << 0); // Clear the UIF flag           
        
}

void delay_miliSecond(uint8_t timer, uint16_t delay) {
	Timer_TypeDef *timerx = select_timer(timer);
	start_timer_in_Milisecond(timer, delay);
	
  while (!(timerx->SR & (1 << 0))) {
                // Wait until the timer reaches the ARR value
            }
            timerx->SR &= ~(1 << 0); // Clear the UIF flag 

}

void timer_irq_microSecond_start (uint8_t timer, uint16_t delay){
	Timer_TypeDef *timerx = select_timer(timer);
	start_timer_in_Microsecond(timer, delay);
	timerx->DIER |= (1<<0);
	__disableirq();
	switch (timer) {
		case 1 : NVIC_ISER0 |= (1<<25); break;
		case 2 : NVIC_ISER0 |= (1<<28); break;
		case 3 : NVIC_ISER0 |= (1<<29); break;
		case 4 : NVIC_ISER0 |= (1<<30); break;
	
	}
	__enableirq();

}

void timer_irq_milisecond_start (uint8_t timer, uint16_t delay){
	Timer_TypeDef *timerx = select_timer(timer);
	 start_timer_in_Milisecond (timer, delay);
	timerx->DIER |= (1<<0);
	__disableirq();
	switch (timer) {
		case 1 : NVIC_ISER0 |= (1<<25); break;
		case 2 : NVIC_ISER0 |= (1<<28); break;
		case 3 : NVIC_ISER0 |= (1<<29); break;
		case 4 : NVIC_ISER0 |= (1<<30); break;
	
	}
	__enableirq();

}

void stop_timer_irq (uint8_t timer) {
	Timer_TypeDef *timerx = select_timer(timer);
	timerx->DIER &= ~(1 << 0);
	__disableirq();
	switch (timer) {
		case 1 : NVIC_ISER0 &= ~(1 <<25); break;
		case 2 : NVIC_ISER0 &= ~(1 <<28); break;
		case 3 : NVIC_ISER0 &= ~(1 <<29); break;
		case 4 : NVIC_ISER0 &= ~(1 <<30); break;
	
	}
	__enableirq();

}

void stop_timer (uint8_t timer) {
	Timer_TypeDef *timerx = select_timer(timer);
	timerx->CR1 &= ~(1<<0);
	// Disable the appropriate timer clock
    if (timer == 1) {
        RCCAPB2 &= ~(1 << 11); // Enable APB2 bus for Timer 1
    } else {
        RCCAPB1 &= ~(1 << (timer - 2)); // Enable APB1 bus for Timer 2, 3, or 4
    }
		stop_timer_irq (timer);
}

void timer_compare_MhZ(uint32_t port, uint8_t pin, uint16_t Load_value, uint16_t compare_value ){
	uint8_t timer_number = select_timer_for_capture_PWM(port, pin);
	uint8_t channel = select_timer_channel (port, pin);
	Timer_TypeDef *timerx = select_timer(timer_number);
	
	// Enable the appropriate timer clock
  if (timer_number == 1) {
        RCCAPB2 |= 1 << 11; // Enable APB2 bus for Timer 1
  } else {
        RCCAPB1 |= 1 << (timer_number - 2); // Enable APB1 bus for Timer 2, 3, or 4
  }
	
	config_gpio (port, pin, output_50Mhz, af_pp_output );
	timerx->PSC = 72-1;
	
	switch (channel) {
		case 1 : timerx->CCMR1  |= 0x30; break;
		case 2 : timerx ->CCMR1 |= 0x3000; break;
		case 3 : timerx ->CCMR2 |= 0x30; break;
		case 4 : timerx ->CCMR2 |= 0x3000 ; break;
	}
	timerx->CCER |= (1<<((channel-1)*4));
	timerx->BDTR |= (1<<15);
	timerx ->CR1 |= (1<<0);
	
	timerx->ARR = Load_value;
	switch (channel) {
		case 1 : timerx ->CCR1 = compare_value; break;
		case 2 : timerx ->CCR2 = compare_value; break;
		case 3 : timerx ->CCR3 = compare_value; break;
		case 4 : timerx ->CCR4 = compare_value; break;
	}	
}

void timer_compare_khZ(uint32_t port, uint8_t pin, uint16_t Load_value, uint16_t compare_value ){
	uint8_t timer_number = select_timer_for_capture_PWM(port, pin);
	uint8_t channel = select_timer_channel (port, pin);
	Timer_TypeDef *timerx = select_timer(timer_number);
	
	// Enable the appropriate timer clock
  if (timer_number == 1) {
        RCCAPB2 |= 1 << 11; // Enable APB2 bus for Timer 1
  } else {
        RCCAPB1 |= 1 << (timer_number - 2); // Enable APB1 bus for Timer 2, 3, or 4
  }
	
	config_gpio (port, pin, output_50Mhz, af_pp_output );
	timerx->PSC = 36000-1;
	
	switch (channel) {
		case 1 : timerx->CCMR1  |= 0x30; break;
		case 2 : timerx ->CCMR1 |= 0x3000; break;
		case 3 : timerx ->CCMR2 |= 0x30; break;
		case 4 : timerx ->CCMR2 |= 0x3000 ; break;
	}
	timerx->CCER |= (1<<((channel-1)*4));
	timerx->BDTR |= (1<<15);
	timerx ->CR1 |= (1<<0);
	
	timerx->ARR = (Load_value*2)-1;
	compare_value *=2;
	switch (channel) {
		case 1 : timerx ->CCR1 = compare_value; break;
		case 2 : timerx ->CCR2 = compare_value; break;
		case 3 : timerx ->CCR3 = compare_value; break;
		case 4 : timerx ->CCR4 = compare_value; break;
	}	
}
 
void timer_PWM_Microsecond(uint32_t port, uint8_t pin, uint16_t period, uint16_t duty_cycle_percentage ){
	uint8_t timer_number = select_timer_for_capture_PWM(port, pin);
	uint8_t channel = select_timer_channel (port, pin);
	Timer_TypeDef *timerx = select_timer(timer_number);
	float duty_coeff = duty_cycle_percentage / 100.0f;
	float tempccr = duty_coeff * period;
	 uint16_t ccr = (uint16_t)(tempccr + 0.5);
	// Enable the appropriate timer clock
  if (timer_number == 1) {
        RCCAPB2 |= 1 << 11; // Enable APB2 bus for Timer 1
  } else {
        RCCAPB1 |= 1 << (timer_number - 2); // Enable APB1 bus for Timer 2, 3, or 4
  }
	
	config_gpio (port, pin, output_50Mhz, af_pp_output );
	timerx->PSC = 72-1;
	
	switch (channel) {
		case 1 : timerx->CCMR1  |= 0x60; break;
		case 2 : timerx ->CCMR1 |= 0x6000; break;
		case 3 : timerx ->CCMR2 |= 0x60; break;
		case 4 : timerx ->CCMR2 |= 0x6000 ; break;
	}
	timerx->CCER |= (1<<((channel-1)*4));
	timerx->BDTR |= (1<<15);
	timerx ->CR1 |= (1<<0);
	
	timerx->ARR = period-1;
	switch (channel) {
		case 1 : timerx ->CCR1 = ccr; break;
		case 2 : timerx ->CCR2 = ccr; break;
		case 3 : timerx ->CCR3 = ccr; break;
		case 4 : timerx ->CCR4 = ccr; break;
	}	
}

void timer_PWM_Milisecond(uint32_t port, uint8_t pin, uint16_t period, uint16_t duty_cycle_percentage ){
	uint8_t timer_number = select_timer_for_capture_PWM(port, pin);
	uint8_t channel = select_timer_channel (port, pin);
	Timer_TypeDef *timerx = select_timer(timer_number);
	
	
	float duty_coeff = duty_cycle_percentage / 100.0f;
	float tempccr = duty_coeff * period;
	uint16_t ccr = (uint16_t)(tempccr + 0.5);
	
	
	
	// Enable the appropriate timer clock
  if (timer_number == 1) {
        RCCAPB2 |= 1 << 11; // Enable APB2 bus for Timer 1
  } else {
        RCCAPB1 |= 1 << (timer_number - 2); // Enable APB1 bus for Timer 2, 3, or 4
  }
	
	config_gpio (port, pin, output_50Mhz, af_pp_output );
	timerx->PSC = 36000-1;
	
	switch (channel) {
		case 1 : timerx->CCMR1  |= 0x60; break;
		case 2 : timerx ->CCMR1 |= 0x6000; break;
		case 3 : timerx ->CCMR2 |= 0x60; break;
		case 4 : timerx ->CCMR2 |= 0x6000 ; break;
	}
	timerx->CCER |= (1<<((channel-1)*4));
	timerx->BDTR |= (1<<15);
	timerx ->CR1 |= (1<<0);
	
	timerx->ARR = (period*2)-1;
	ccr *=2;
	switch (channel) {
		case 1 : timerx ->CCR1 = ccr; break;
		case 2 : timerx ->CCR2 = ccr; break;
		case 3 : timerx ->CCR3 = ccr; break;
		case 4 : timerx ->CCR4 = ccr; break;
	}	
}

