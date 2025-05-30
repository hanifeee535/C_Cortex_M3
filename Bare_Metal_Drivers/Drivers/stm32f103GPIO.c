/* ####################################
Author: MD. Soyabbir Abu Hanif
 MSc in Embedded Systems
Tampere University, Finland
Email: hanifseceee535@gmail.com 
######################################
*/

/*
********************************************************** 
********************__Config_GPIO__***********************
**********************************************************

@Function name: void Config_GPIO(uint8_t port, uint8_t pin, uint8_t mode, uint8_t config)
@Purpose: To configure a specific GPIO pin with specific mode and configuration.
@Parameters:

    Port: GPIO Ports (1 = portA, 2 = portB, 3 = portC, 4 = portD, 5 = portE)

    Pin: GPIO pins (1-15)

    Modes: 

            Macro	        Value	    Description

            input	          0	        Input mode
            output_10Mhz	  1	        Output mode, max speed 10 MHz
            output_2Mhz	      2	        Output mode, max speed 2 MHz
            output_50Mhz	  3	        Output mode, max speed 50 MHz

    Configuration: if Mode is input: 

            Macro	        Value	    Description

            analog_in	      0	        Analog mode
            floating_in	      1	        Floating input (no pull-up/down)
            pp_in	          2	        Input with pull-up / pull-down

        Configuration: if Mode is input: 

            Macro	        Value	    Description

            gp_output	      0	        General purpose output push-pull
            od_output	      1	        General purpose output open-drain
            af_pp_output	  2	        Alternate function push-pull
            af_od_output	  3	        Alternate function open-drain
            
@Example: Config_GPIO(portA, 5, output_2Mhz, gp_output); // Configure PA5 as 2MHz push-pull output

**********************************************************
********************__Write-to_GPIO__***********************
**********************************************************
@Function name: void Write_GPIO(uint8_t port, uint8_t pin, uint8_t state)
@Purpose: To write a logical state (HIGH or LOW) to a specific GPIO output pin.
@Parameters:

    Port: GPIO Ports(1 = portA, 2 = portB, 3 = portC, 4 = portD, 5 = portE)

    Pin: GPIO pin number (0–15)

    State:
        HIGH (1) = Set pin high
        LOW (0) = Set pin low

@Example: Write_GPIO(portB, 3, HIGH); // Set PB3 to high
          Write_GPIO(portB, 3, LOW);  // Set PB3 to low


**********************************************************
********************__Read-From_GPIO_Pin__***********************
**********************************************************

@Function name: uint32_t Read_GPIO_Pin(uint8_t port, uint8_t pin)
@Purpose: To read the logical state of a single GPIO input pin.
@Parameters:

    Port: GPIO Ports(1 = portA, 2 = portB, 3 = portC, 4 = portD, 5 = portE)

    Pin: GPIO pin number (0–15)

@Returns:

        1 if the pin is HIGH

        0 if the pin is LOW
@Example: 
        uint32_t state = Read_GPIO_Pin(portC, 13); // Read PC13 state
            if (state) {
                // pin is high
            }


**********************************************************
********************__Read_Entire_GPIO_Port__***********************
**********************************************************
@Function name: uint32_t Read_GPIO_Port(uint8_t port)
@Purpose: To read the entire 16-bit input value of a GPIO port.
@Parameters:

    Port: GPIO Ports(1 = portA, 2 = portB, 3 = portC, 4 = portD, 5 = portE)

@Returns:

    16-bit integer representing the full input state of all 16 pins in the port.

@Example:
    uint32_t portState = Read_GPIO_Port(portA); // Read all pins of GPIOA



**********************************************************
********************__Toggling_GPIO_PIN__***********************
**********************************************************
@Function name: void toggle_gpio(uint8_t port, uint8_t pin)
@Purpose: Toggles the output state of a GPIO pin (HIGH becomes LOW, and vice versa).
@Parameters:

    Port: GPIO Ports(1 = portA, 2 = portB, 3 = portC, 4 = portD, 5 = portE)

    Pin: GPIO pin number (0–15)

@Example:
    toggle_gpio(portA, 5); // Toggle PA5


**********************************************************/






#include <stdint.h>
#include "stm32f103Driver.h"

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
	state = ((*idr & (1 << pin)) >> pin);
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