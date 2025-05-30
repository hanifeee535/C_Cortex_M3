/* ####################################
Author: MD. Soyabbir Abu Hanif
 MSc in Embedded Systems
Tampere University, Finland
Email: hanifseceee535@gmail.com 
######################################
*/

/*
********************************************************************************** 
********************__Configuring_GPIO_External_Interrupt__***********************
**********************************************************************************

@Function name:
    void configure_gpio_interrupt(uint8_t pin, uint8_t port, uint8_t trigger_type, uint8_t priority_level)

@Purpose:
    To configure an external interrupt (EXTI) on a specific GPIO pin with a specified trigger type and interrupt priority.

@Parameters:

    pin: GPIO pin number (0–15).

    port: GPIO port ID:(0 = INT_PORT_A, 1 = INT_PORT_B, 2 = INT_PORT_C, 3 = INT_PORT_D, 4 = INT_PORT_E)

    trigger_type: 

            Macro           Value	        Description
            RISING            0	            Rising edge
            FALLING           1	            Falling edge
            BOTH              2	            Both rising and falling

    priority_level: Interrupt priority level (0 is highest, 15 is the lowest priority)  


@Functionality:

    Enables AFIO and maps the EXTI line to the specified port.

    Configures EXTI line for the selected pin.

    Sets the appropriate edge trigger (rising, falling, or both).

    Enables the interrupt in the NVIC (IRQ number depends on pin).

    Sets NVIC priority.

@example 
    configure_gpio_interrupt(0, portA, 0, 0); // Configure PA0 to trigger on rising edge with highest priority

***************************************************************** 
********************__EXTI_Interrupt Handlers__***********************
*****************************************************************
    EXTI Line(s)		Interrupt Handler Function Name

    EXTI0		        void EXTI0_IRQHandler(void)
    EXTI1		        void EXTI1_IRQHandler(void)
    EXTI2		        void EXTI2_IRQHandler(void)
    EXTI3		        void EXTI3_IRQHandler(void)
    EXTI4		        void EXTI4_IRQHandler(void)
    EXTI[9:5]		    void EXTI9_5_IRQHandler(void)
    EXTI[15:10]		    void EXTI15_10_IRQHandler(void)

**********************************************************/






#include <stdint.h>
#include "stm32f103Driver.h"

void configure_gpio_interrupt(uint8_t pin, uint8_t port, uint8_t trigger_type,  uint8_t priority_level){
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
	uint32_t irq_number;
	if (pin <= 4) {
		irq_number = pin + 6;
        NVIC_ISER0 |= (1 << (pin + 6));
    }
		else if (pin > 4 && pin <=9){
			 irq_number = 23;
			NVIC_ISER0 |= (1 << 23);
		}
		else if (pin > 9 && pin <=15){
			irq_number = 40;
			NVIC_ISER1 |= (1 << 8);
		}
		else {
		        __enableinterrupt();
		        return; // Invalid pin
		    }

    //setting the interrupt priority
	NVIC_SetPriority(irq_number ,priority_level );

	    __enableinterrupt();

}

