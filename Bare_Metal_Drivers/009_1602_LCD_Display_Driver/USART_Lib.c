#include <stdint.h>
#include "header.h"

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
		RCCAPB2 |= (1<<14);
		config_gpio (portA, 9, output_50Mhz, af_pp_output);
		config_gpio (portA, 10, input , pp_in);
	}
	else if (usart == 2) {
		RCCAPB1 |= (1<<17);
		config_gpio (portA, 2, output_50Mhz, af_pp_output);
		config_gpio (portA, 3, input , pp_in);
	}
	else if (usart == 3) {
		RCCAPB1 |= (1<<18);
		config_gpio (portB, 10, output_50Mhz, af_pp_output);
		config_gpio (portB, 11, input , pp_in);
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
	
	__disableirq();
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
	__enableirq();
}

void init_usart_transmit_interrupt(uint8_t usart, uint32_t baud_rate) {
    // Initialize USART (enable clock, set GPIO, configure baud rate, enable USART)
    USART_TypeDef *USARTX = select_USART(usart);
    init_USART(usart, baud_rate);
    __disableirq();
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
		__enableirq();
}

