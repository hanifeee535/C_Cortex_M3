#include <stdint.h>
#include "header.h"

/*
I2C2
PB10 -> SCL  // Clock line for I2C2
PB11 -> SDA  // Data line for I2C2

I2C1
PB6 -> SCL   // Clock line for I2C1
PB7 -> SDA   // Data line for I2C1
*/


I2C_TypeDef* select_I2C(uint8_t i2c) {
    switch (i2c) {
        case 1:
            return I2C1;  // Return I2C1 if i2c == 1
        case 2:
            return I2C2;  // Return I2C2 if i2c == 2
        default:
            return 0;     // Return NULL if i2c is invalid (not 1 or 2)
    }
}


void i2c_init ( uint8_t i2c, uint8_t speed ){
	// Enable the Alternate Function clock (needed for GPIO alternate functions)
	RCCAPB2 |= (1<<0);
	
	//Selecting the i2c
	I2C_TypeDef *I2CX =  select_I2C(i2c);
	
	// Enable the clocks for I2C1
	if (i2c == 1) { 
	RCCAPB1 |= (1<< 21);
	config_gpio (portB,6, output_50Mhz, af_od_output);
	config_gpio (portB,7, output_50Mhz, af_od_output);
	
	}
	else if (i2c == 2) { 
	RCCAPB1 |= (1<< 22) ;
	config_gpio (portB,6, output_50Mhz, af_od_output);
	config_gpio (portB,7, output_50Mhz, af_od_output);
	}
	
	//configuring the i2c
	/* When set, the I2C is under reset state. Before resetting this bit, make sure the I2C lines are 
   released and the bus is free. */
	I2CX->CR1 |= (1<<15); //enabling i2c reset
	I2CX->CR1 &= ~ (unsigned) (1<<15); 
	
	I2CX->CR2 = 0x8; // Set peripheral clock frequency to 36 MHz
	I2CX->CCR = speed; // Set speed (fast mode or standard mode) using CCR register
	I2CX -> TRISE = 0x9; // Configure maximum rise time
	I2CX -> CR1 |= (1<<0); // Enable the I2C2 peripheral
	
}

//i2c start bit set
void i2c_start (uint8_t i2c){
	if (i2c ==1){
		I2C1->CR1 |= (1<<8);  // Set the START bit
		while (!(I2C1->SR1 & 1)){}; // Wait for the SB flag to be set
	}
	else if (i2c ==2){
		I2C2->CR1 |= (1<<8);  // Set the START bit
		while (!(I2C2->SR1 & 1)){}; // Wait for the SB flag to be set
	}
}

//i2c adddress sending and read/write command
void i2c_send_address(uint8_t i2c, char address, uint8_t R_W ){
	//volatile uint32_t temp = 0;
	//Selecting the i2c
	I2C_TypeDef *I2CX =  select_I2C(i2c);
	I2CX->DR = (address | R_W); // Write slave address + R/W bit 
	while ((I2CX->SR1 & 2) ==0){}; // Wait for ADDR flag (Address Sent flag to be 1)
	while ((I2CX->SR1 & 2)) {
		/*Reading SR1 and SR2 is the hardware's way of ensuring that the CPU acknowledges the address phase is complete. */
		 I2CX->SR1;
		 I2CX->SR2; 
		if ((I2CX->SR1 & 2) == 0){ break;}
	}
	
}

//send i2c data
void i2c_send_data (uint8_t i2c, char data) {
	if (i2c==1) {
		while ((I2C1->SR1 & 0x80) == 0) {} // Wait until the I2C1 Data Register (DR) is empty (TXE = 1)
		I2C1->DR = data;
		while ((I2C1->SR1 & 0x80)==0) {}  // Wait until the data has been transferred (TXE = 1, meaning data is sent)
	}
	else if (i2c==2) {
		while ((I2C2->SR1 & 0x80) == 0) {} // Wait until the I2C1 Data Register (DR) is empty (TXE = 1)
		I2C2->DR = data;
		while ((I2C2->SR1 & 0x80)==0) {}  // Wait until the data has been transferred (TXE = 1, meaning data is sent)
	}
}

void i2c_stop (uint8_t i2c) {
	//volatile int tmp = 0;
	 if(i2c == 1)
    {
        I2C1->SR1;  // Read SR1 to clear any flags
         I2C1->SR2;  // Read SR2 to clear any flags

        I2C1->CR1 |= 0x200;  // Set the STOP bit to generate a STOP condition
    }
    else if(i2c == 2)
    {
        I2C2->SR1;  // Read SR1 to clear any flags
        I2C2->SR2;  // Read SR2 to clear any flags

        I2C2->CR1 |= 0x200;  // Set the STOP bit to generate a STOP condition
    }
	
}

//i2c write function: 
void i2c_write (uint8_t i2c, uint8_t address, char data []) {
	int i = 0;
	i2c_start (i2c);
	i2c_send_address (i2c, address, 0);
	while (data[i] != '\0'){
		i2c_send_data (i2c, data[1]);
		i++;
	}
	i2c_stop (i2c);

}
