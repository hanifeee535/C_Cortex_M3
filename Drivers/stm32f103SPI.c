
/* ####################################
Author: MD. Soyabbir Abu Hanif
 MSc in Embedded Systems
Tampere University, Finland
Email: hanifseceee535@gmail.com 
######################################
*/



/*
************************************************************************************************
*************************__SPI_PINS__*************************
*************************************************************************************************



SPI1 (APB2 Bus)
            SCK (Serial Clock)           PA5

            MISO (Master In Slave Out)   PA6

            MOSI (Master Out Slave In)   PA7

            CS (Slave Select)           PA4

SPI2 (APB1 Bus)
            SCK                          PB13

            MISO                         PB14

            MOSI                         PB15

            CS                           PB12

SPI3 (APB1 Bus, if available)
            SCK                          PB3

            MISO                         PB4

            MOSI                         PB5

            CS                          PA15

IRQ Handlesrs:
            SPI1:           void SPI1_IRQHandler(void)

            SPI2:           void SPI2_IRQHandler(void)

            SPI3:           void SPI3_IRQHandler(void)



************************************************************************************************
*************************__SPI_init__*************************
*************************************************************************************************

@function_name: void init_SPI(uint8_t spi, uint32_t  prescaler)

@purpose:
    Initializes SPI1 or SPI2 peripheral with a user-defined clock prescaler. Also configures the associated GPIO pins for SPI communication and chip select (CS).

@parameters:

    spi:                 SPI peripheral to initialize (1 for SPI1, 2 for SPI2)
    prescaler:           SPI clock prescaler value (valid values: 2, 4, 8, 16, 32, 64, 128, 256)
    

@Return: none

@Behavior:
    Maps prescaler to SPI BR[2:0] bits.

    Enables the clock for the selected SPI peripheral.

    Configures SPI pins:

    SPI1: PA4 (CS), PA5 (SCK), PA6 (MISO), PA7 (MOSI)

    SPI2: PB12 (CS), PB13 (SCK), PB14 (MISO), PB15 (MOSI)

    Enables software slave management (SSM) and master mode.

    Activates the SPI peripheral by setting the SPE bit.

    Sets CS pin to HIGH (idle state).

@example:
    init_SPI(1, 16);  // Initializes SPI1 with a clock prescaler of 16



************************************************************************************************
*************************__SPI_transmit_simplex__*************************
*************************************************************************************************

@function_name: void spi_transmit(uint8_t spi, char tx_char)

@purpose:
    Transmits a single byte over SPI in simplex mode (no response read).

@parameters:

    spi:                 SPI peripheral to initialize (1 for SPI1, 2 for SPI2)
    tx_char:             Data byte to transmit
    

@Return: none

@Behavior:
    Pulls CS low before transmission.

    Sends the byte using the DR register.

    Waits for:

        TXE (transmit buffer empty)

        BSY (not busy)

    Pulls CS high to complete the transaction.

@example:
    spi_transmit(2, 'A');  // Sends character 'A' over SPI2


************************************************************************************************
*************************__SPI_transmit_message__*************************
*************************************************************************************************

@function_name: void spi_send_message(uint8_t spi, char message[])

@purpose:
    Transmits a null-terminated string (message) over SPI, byte by byte.

@parameters:

    spi:                 SPI peripheral to initialize (1 for SPI1, 2 for SPI2)
    message:             Pointer to a null-terminated character array
    

@Return: none

@Behavior:
    Pulls CS low.

    Sends each character sequentially.

    Waits after each byte for TXE and BSY flags.

    Pulls CS high at the end of the transmission.
@example:
    spi_send_message(1, "Hello SPI");  // Transmits the string via SPI1


************************************************************************************************
*************************__SPI_receive_simplex__*************************
*************************************************************************************************

@function_name: uint8_t spi_receive_simplex(uint8_t spi)

@purpose:
    Receives a single byte from an SPI device using simplex (read-only) communication.

@parameters:

    spi:                 SPI peripheral to initialize (1 for SPI1, 2 for SPI2)
       

@Return: rx_data: Received byte

@Behavior:
    Pulls CS low.

    Sends dummy byte 0xFF to generate clock and receive data.

    Waits for TXE and BSY.

    Reads received byte from DR.

    Pulls CS high.

@example:
    uint8_t data = spi_receive_simplex(2);  // Receives a byte from SPI2


************************************************************************************************
*************************__SPI_Transmit_Receive_Duplex__*************************
*************************************************************************************************

@function_name: uint8_t spi_transmit_receive_duplex(uint8_t spi, uint8_t tx_data)

@purpose:
    Performs full-duplex communication: sends a byte and simultaneously receives a byte.

@parameters:

    spi:                 SPI peripheral to initialize (1 for SPI1, 2 for SPI2)
    tx_data:             Byte to transmit
       

@Return: rx_data: Byte received during transmission

@Behavior:
    Behavior
    Pulls CS low.

    Sends the data byte.

    Waits for TXE and BSY.

    Reads the received byte from DR.

    Pulls CS high.
    
@example:
    uint8_t received = spi_transmit_receive_duplex(1, 0x9F);  // Sends 0x9F and receives response on SPI1



*/

#include <stdint.h>
#include "stm32f103Driver.h"



void init_SPI(uint8_t spi, uint32_t  prescaler) {
   
	uint8_t prescaler_bits; // Variable to hold the BR[2:0] value

	RCC->APB2ENR |= (1<<0); 
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
	RCC->APB2ENR|= (1 << 12); // Enable SPI1 clock (Bit 12 of APB2ENR)
	Config_GPIO(portA, 5, output_50Mhz, af_pp_output); // PA5: SPI1_SCK
	Config_GPIO(portA, 6, input, pp_in);              // PA6: SPI1_MISO
	Config_GPIO(portA, 7, output_50Mhz, af_pp_output); // PA7: SPI1_MOSI
	Config_GPIO(portA, 4, output_50Mhz, gp_output); // PA4: Chip Select (CS)
		
	SPI1->CR1 |= (1 << 2) | (1 << 8) | (1 << 9);  // 0x4 (MSTR) + 0x100 (SSM) + 0x200 (SSI)
	SPI1->CR1 |= (prescaler_bits<<3); // Prescaler
	SPI1->CR2 |= (1 << 2);                         // SSOE (0x4) if needed
    SPI1->CR1 |= (1 << 6);                         // SPE enable (0x40)
	Write_GPIO (portA, 4, HIGH);
} 
else if (spi == 2) {
	RCC->APB1ENR |= (1 << 14); // Enable SPI2 clock (Bit 14 of APB1ENR)
	Config_GPIO(portB, 13, output_50Mhz, af_pp_output); // PB13: SPI2_SCK
	Config_GPIO(portB, 14, input, pp_in);               // PB14: SPI2_MISO
	Config_GPIO(portB, 15, output_50Mhz, af_pp_output); // PB15: SPI2_MOSI
	Config_GPIO(portB, 12, output_50Mhz, gp_output); // PB12: Chip Select (CS)
		
	SPI2->CR1 |= (1 << 2) | (1 << 8) | (1 << 9);  // MSTR + SSM + SSI
	SPI2->CR1 |= (prescaler_bits << 3);
	SPI2->CR2 |= (1 << 2);
	SPI2->CR1 |= (1 << 6);
	Write_GPIO (portB, 12, HIGH);
} 
else {
	return; // Invalid SPI number, exit the function
}


}

//Simplex transmit
void spi_transmit(uint8_t spi, char tx_char) {
if (spi ==1){
	Write_GPIO (portA, 4, LOW);
	SPI1->DR = tx_char;
	while (!(SPI1->SR & (1 << 1))) {} ;  // Wait for TX buffer empty (TXE=1)
    while (SPI1->SR & (1 << 7)) {} ;     // Wait until not busy (BSY=0)    
	Write_GPIO (portA, 4, HIGH);
 }
 else if (spi ==2){
	Write_GPIO (portB, 12, LOW);
	 SPI2->DR = tx_char;
	 while (!(SPI2->SR & (1 << 1))) {} ;  // TXE check
	 while (SPI2->SR & (1 << 7)) {} ;     // BSY check
	 Write_GPIO (portB, 12, HIGH);
 }   
}

void spi_send_message (uint8_t spi, char message[]){
int i = 0;
if (spi ==1){
	Write_GPIO (portA, 4, LOW);
	while (message[i]) {
		SPI1->DR = message[i];
		while (!(SPI1->SR & (1 << 1))) {} ;  //  TXE check
		while (SPI1->SR & (1 << 7)) {} ;     //  BSY check
		i++;
	}
	Write_GPIO (portA, 4, HIGH);
}

else if (spi ==2){
	Write_GPIO (portB, 12, LOW);
	while (message[i]) {
		SPI2->DR = message[i];
		while (!(SPI2->SR & (1 << 1))) {} ;  //  TXE
		while (SPI2->SR & (1 << 7)) {} ;     //  BSY
		i++;
	}
	Write_GPIO (portB, 12, HIGH);
}
}


// Simplex Receive
uint8_t spi_receive_simplex(uint8_t spi) {
    uint8_t rx_data = 0;
    if (spi == 1) {
        Write_GPIO(portA, 4, LOW);      // CS Low
        SPI1->DR = 0xFF;                // Dummy byte to generate clock
        while (!(SPI1->SR & (1 << 1))){}; // Wait for TX buffer empty (TXE)
        while (SPI1->SR & (1 << 7)){} ;  // Wait until not busy (BSY)
        rx_data = SPI1->DR;             // Read received data
        Write_GPIO(portA, 4, HIGH);     // CS High
    } 
	
    else if (spi == 2) {
        Write_GPIO(portB, 12, LOW);
        SPI2->DR = 0xFF;
        while (!(SPI2->SR & (1 << 1))){}
        while (SPI2->SR & (1 << 7)){}
        rx_data = SPI2->DR;
        Write_GPIO(portB, 12, HIGH);
    }
    return rx_data;
}

//Full-Duplex Transmit & Receive
uint8_t spi_transmit_receive_duplex(uint8_t spi, uint8_t tx_data) {
    uint8_t rx_data = 0;
    if (spi == 1) {
        Write_GPIO(portA, 4, LOW);      // CS Low
        SPI1->DR = tx_data;             // Send command/address
		while (!(SPI1->SR & (1 << 1))){} // Wait for TXE
        while (SPI1->SR & (1 << 7)){}   // Wait for BSY
        rx_data = SPI1->DR;             // Read response
        Write_GPIO(portA, 4, HIGH);     // CS High
    } 
    else if (spi == 2) {
        Write_GPIO(portB, 12, LOW);
        SPI2->DR = tx_data;
        while (!(SPI2->SR & (1 << 1))){}
        while (SPI2->SR & (1 << 7)){}
        rx_data = SPI2->DR;
        Write_GPIO(portB, 12, HIGH);
    }
    return rx_data;
}

