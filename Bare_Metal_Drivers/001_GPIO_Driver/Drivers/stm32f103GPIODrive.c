/* ####################################
Author: MD. Soyabbir Abu Hanif
 MSc in Embedded Systems
Tampere University, Finland
Email: hanifseceee535@gmail.com 
######################################
*/

#include <stdint.h>
#include "stm32f103GPIODrive.h"

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
	state? ((*odr |= 1<<pin)) : ((*odr &= ~1<<pin));
	
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
