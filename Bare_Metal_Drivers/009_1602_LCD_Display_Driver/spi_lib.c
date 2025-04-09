#include<stdint.h>
#include "header.h"



void init_SPI(uint8_t spi, uint32_t  prescaler) {
   
		uint8_t prescaler_bits; // Variable to hold the BR[2:0] value
	
		RCCAPB2 |= (1<<0); 
		// Map the prescaler value to BR[2:0]
    switch (prescaler) {
        case 2:
            prescaler_bits = 0b000;
            break;
        case 4:
            prescaler_bits = 0b001;
            break;
        case 8:
            prescaler_bits = 0b010;
            break;
        case 16:
            prescaler_bits = 0b011;
            break;
        case 32:
            prescaler_bits = 0b100;
            break;
        case 64:
            prescaler_bits = 0b101;
            break;
        case 128:
            prescaler_bits = 0b110;
            break;
        case 256:
            prescaler_bits = 0b111;
            break;
        default:
            return; // Invalid prescaler value, exit the function
    }


    // Enable RCC clock and configure GPIOs based on SPI selection
    if (spi == 1) {
        RCCAPB2|= (1 << 12); // Enable SPI1 clock (Bit 12 of APB2ENR)
        config_gpio(portA, 5, output_50Mhz, af_pp_output); // PA5: SPI1_SCK
        config_gpio(portA, 6, input, pp_in);              // PA6: SPI1_MISO
        config_gpio(portA, 7, output_50Mhz, af_pp_output); // PA7: SPI1_MOSI
        config_gpio(portA, 4, output_50Mhz, gp_output); // PA4: Chip Select (CS)
			
			  SPI1->CR1 |= 0x4;
				SPI1->CR1 |= (prescaler_bits<<3);
				SPI1->CR2 |= 0x4;	
				SPI1->CR1 |= 0x40;
				write_gpio (portA, 4, HIGH);
    } 
    else if (spi == 2) {
        RCCAPB1 |= (1 << 14); // Enable SPI2 clock (Bit 14 of APB1ENR)
        config_gpio(portB, 13, output_50Mhz, af_pp_output); // PB13: SPI2_SCK
        config_gpio(portB, 14, input, pp_in);               // PB14: SPI2_MISO
        config_gpio(portB, 15, output_50Mhz, af_pp_output); // PB15: SPI2_MOSI
        config_gpio(portB, 12, output_50Mhz, gp_output); // PB12: Chip Select (CS)
			
			  SPI2->CR1 |= 0x4;
				SPI2->CR1 |= 0x31;
				SPI2->CR2 |= 0x4;	
				SPI2->CR1 |= 0x40;
				write_gpio (portB, 12, HIGH);
    } 
    else {
        return; // Invalid SPI number, exit the function
    }

    
}


void spi_transmit(uint8_t spi, char tx_char) {
   if (spi ==1){
		write_gpio (portA, 4, LOW);
		 SPI1->DR = tx_char;
		 while( SPI1->SR &= 0x80 ){}
		 write_gpio (portA, 4, HIGH);
	 }
	 else if (spi ==1){
		write_gpio (portB, 12, LOW);
		 SPI2->DR = tx_char;
		 while( SPI2->SR &= 0x80 ){}
		 write_gpio (portB, 12, HIGH);
	 }   
}

void spi_send_message (uint8_t spi, char message[]){
	int i = 0;
	if (spi ==1){
		write_gpio (portA, 4, LOW);
		while (message[i]){
			SPI1->DR = message[i];
			while( SPI1->SR &= 0x80 ){}
		  i++;
		}
		write_gpio (portA, 4, HIGH);
	}
	
	else if (spi ==2){
		write_gpio (portB, 12, LOW);
		while (message[i]){
			SPI2->DR = message[i];
			while( SPI2->SR &= 0x80 ){}
		  i++;
		}
		write_gpio (portB, 12, HIGH);
	}
}
