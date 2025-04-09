/* ####################################
Author: MD. Soyabbir Abu Hanif
 MSc in Embedded Systems
Tampere University, Finland
Email: hanifseceee535@gmail.com 
######################################
*/


 /* 
 *Note: 
 * - The actual hardware register offsets for GPIO (e.g., configuration, input, output) are 
 *   0x00, 0x04, 0x08, 0x0C, etc. However, this implementation uses logical offsets 
 *   (0, 1, 2, 3, ...) because pointer arithmetic on `uint32_t*` automatically accounts 
 *   for the 4-byte (32-bit) size of each register. For example:
 *   - `+0` corresponds to 0x00 (Configuration register low)
 *   - `+1` corresponds to 0x04 (Configuration register high)
 *   - `+2` corresponds to 0x08 (Input data register)
 *   - `+3` corresponds to 0x0C (Output data register)
 */


#include "header.h"
#include <stdint.h>

//configuration of GPIO ports:
void config_gpio ( uint32_t port , uint32_t pin, uint32_t mode, uint32_t config){
	volatile uint32_t *tempR; // Pointer to the configuration register
	uint32_t tempPin = pin; //temporary pin number
	
	uint32_t offset = 0x00; // Offset for configuration register (low)
	
// Adjust for higher pins (8-15), which use the high configuration register
	if (pin >7)
	{
		tempPin -=8; // Scale the pin number to the 0-7 range
		offset = 0x01;  // Offset for high configuration register
	}
	
	// Enable clock for the respective GPIO port and set the configuration register
	if(port ==1){  //A port
		RCCAPB2 = RCCAPB2 | (1<<2); //Enabling the APB2 clock enable Register for GPIOA
		tempR = (volatile uint32_t*) (&GPIOA + offset); // setting the offset = 0x00 for register 
		//low and offset is 0x01 for register high		
	}
	
	else if (port ==2){ //B port
		RCCAPB2 = RCCAPB2 | (1<<3); //Enabling the APB2 clock enable Register for GPIOA
		tempR = (volatile uint32_t*) (&GPIOB + offset); 
	}
	
	else if (port ==3){ //C port
		RCCAPB2 = RCCAPB2 | (1<<4); //Enabling the APB2 clock enable Register for GPIOA
		tempR = (volatile uint32_t*) (&GPIOC + offset); 
	}
	
	else if (port ==4){ //D port
		RCCAPB2 = RCCAPB2 | (1<<5); //Enabling the APB2 clock enable Register for GPIOA
		tempR = (volatile uint32_t*) (&GPIOD + offset); 
	}
	
	else if (port ==5){ //E port
		RCCAPB2 = RCCAPB2 | (1<<6); //Enabling the APB2 clock enable Register for GPIOA
		tempR = (volatile uint32_t*) (&GPIOE + offset); 
	}
	else { tempR = 0;}
	
	*tempR &= (unsigned) ~ (0xF<<(tempPin*4)); //reseting the gpio pin
	*tempR |= (config << (tempPin*4+2)); //setting the configuration 
	*tempR |= (mode << (tempPin*4)); //setting the mode 
}



//function to write into the gpio pin
void write_gpio (uint32_t port, uint32_t pin, uint32_t state){
	volatile uint32_t* ODR; // Pointer to the output data register (ODR
	volatile uint32_t offset = 0x03; // Offset for ODR
	
	// Determine the ODR address based on the port
	if (port == 1){ //A port
		ODR = (volatile uint32_t *) (&GPIOA + offset);
	}
	else if (port == 2){ //B port
		ODR = (volatile uint32_t *) (&GPIOB + offset);
	}
	else if (port == 3){ //C port
		ODR = (volatile uint32_t *) (&GPIOC + offset);
	}
	else if (port == 4){ //D port
		ODR = (volatile uint32_t *) (&GPIOD + offset);
	}
	else if (port == 5){ //E port
		ODR = (volatile uint32_t *) (&GPIOE + offset);
	}
	else {ODR = 0;}
	
	 // Write the desired state to the pin
	state? ((*ODR |= 1<< pin)) : (*ODR &= (unsigned) ~ (1<<pin));
		
}

//function to read from gpio pin
uint32_t read_gpio (uint32_t port, uint32_t pin){
	volatile uint32_t* IDR;  // Pointer to the input data register (IDR)
	volatile uint32_t offset = 0x02; // Offset for IDR
	uint32_t state;
	
	if (port == 1){ //A port
		IDR = (volatile uint32_t *) (&GPIOA + offset);
	}
	else if (port == 2){ //B port
		IDR = (volatile uint32_t *) (&GPIOB + offset);
	}
	else if (port == 3){ //C port
		IDR = (volatile uint32_t *) (&GPIOC + offset);
	}
	else if (port == 4){ //D port
		IDR = (volatile uint32_t *) (&GPIOD + offset);
	}
	else if (port == 5){ //E port
		IDR = (volatile uint32_t *) (&GPIOE + offset);
	}
	else {IDR = 0;}
	
	// Read the state of the pin
	state = (*IDR & (1<<pin)) >> pin;
	return state;
 
}


//Function to toggle the gpio pin
void toggle_gpio (uint32_t port, uint32_t pin){
	if (read_gpio(port, pin)){
		write_gpio (port, pin, 0);		
	}
	else { write_gpio(port, pin, 1); }

}
