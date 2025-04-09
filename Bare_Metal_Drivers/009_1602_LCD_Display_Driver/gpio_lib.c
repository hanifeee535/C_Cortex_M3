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
void config_gpio(uint32_t port, uint32_t pin, uint32_t mode, uint32_t config) {
    volatile uint32_t *tempR; // Pointer to the configuration register
    uint32_t tempPin = pin;   // Temporary pin number
    uint32_t offset = 0x00;   // Offset for configuration register (low)
    
    // Adjust for higher pins (8-15), which use the high configuration register
    if (pin > 7) {
        tempPin -= 8;  // Scale the pin number to the 0-7 range
        offset = 0x01;  // Offset for high configuration register
    }
    
    // Enable clock for the respective GPIO port and set the configuration register
    if (port == portA) {  // GPIOA
        RCCAPB2 |= (1 << 2); // Enabling the APB2 clock for GPIOA
        tempR = (volatile uint32_t *)(&(GPIOA->CRL) + offset); // Register for GPIOA
    }
    else if (port == portB) {  // GPIOB
        RCCAPB2 |= (1 << 3); // Enabling the APB2 clock for GPIOB
        tempR = (volatile uint32_t *)(&(GPIOB->CRL) + offset); // Register for GPIOB
    }
    else if (port == portC) {  // GPIOC
        RCCAPB2 |= (1 << 4); // Enabling the APB2 clock for GPIOC
        tempR = (volatile uint32_t *)(&(GPIOC->CRL) + offset); // Register for GPIOC
    }
    else if (port == portD) {  // GPIOD
        RCCAPB2 |= (1 << 5); // Enabling the APB2 clock for GPIOD
        tempR = (volatile uint32_t *)(&(GPIOD->CRL) + offset); // Register for GPIOD
    }
    else if (port == portE) {  // GPIOE
        RCCAPB2 |= (1 << 6); // Enabling the APB2 clock for GPIOE
        tempR = (volatile uint32_t *)(&(GPIOE->CRL) + offset); // Register for GPIOE
    } 
    else {
        tempR = 0;
    }

    if (tempR) {
        *tempR &= ~(0xF << (tempPin * 4));   // Reset the GPIO pin
        *tempR |= (config << (tempPin * 4 + 2)); // Set the configuration
        *tempR |= (mode << (tempPin * 4));   // Set the mode
    }
}


//function to write into the gpio pin
void write_gpio(uint32_t port, uint32_t pin, uint32_t state) {
    volatile uint32_t *ODR;  // Pointer to the output data register (ODR)
    
    // Determine the ODR address based on the port
    if (port == portA) {  // GPIOA
        ODR = &(GPIOA->ODR);
    }
    else if (port == portB) {  // GPIOB
        ODR = &(GPIOB->ODR);
    }
    else if (port == portC) {  // GPIOC
        ODR = &(GPIOC->ODR);
    }
    else if (port == portD) {  // GPIOD
        ODR = &(GPIOD->ODR);
    }
    else if (port == portE) {  // GPIOE
        ODR = &(GPIOE->ODR);
    }
    else {
        ODR = 0;
    }
    
    if (ODR) {
        // Write the desired state to the pin
        if (state) {
            *ODR |= (1 << pin);
        } else {
            *ODR &= ~(1 << pin);
        }
    }
}


//function to read from gpio pin
uint32_t read_gpio(uint32_t port, uint32_t pin) {
    volatile uint32_t *IDR;  // Pointer to the input data register (IDR)
    uint32_t state;
    
    // Determine the IDR address based on the port
    if (port == portA) {  // GPIOA
        IDR = &(GPIOA->IDR);
    }
    else if (port == portB) {  // GPIOB
        IDR = &(GPIOB->IDR);
    }
    else if (port == portC) {  // GPIOC
        IDR = &(GPIOC->IDR);
    }
    else if (port == portD) {  // GPIOD
        IDR = &(GPIOD->IDR);
    }
    else if (port == portE) {  // GPIOE
        IDR = &(GPIOE->IDR);
    }
    else {
        IDR = 0;
    }

    if (IDR) {
        // Read the state of the pin
        state = (*IDR & (1 << pin)) >> pin;
        return state;
    } else {
        return 0;
    }
}



//Function to toggle the gpio pin
void toggle_gpio(uint32_t port, uint32_t pin) {
    if (read_gpio(port, pin)) {
        write_gpio(port, pin, 0);        
    }
    else { 
        write_gpio(port, pin, 1); 
    }
}




