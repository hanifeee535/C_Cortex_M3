/* ####################################
Author: MD. Soyabbir Abu Hanif
 MSc in Embedded Systems
Tampere University, Finland
Email: hanifseceee535@gmail.com 
######################################
*/

#include <stdint.h>
#include "../Drivers/stm32f103SysticDrive.h"


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















