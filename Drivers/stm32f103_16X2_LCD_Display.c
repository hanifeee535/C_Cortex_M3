/* ####################################
Author: MD. Soyabbir Abu Hanif
 MSc in Embedded Systems
Tampere University, Finland
Email: hanifseceee535@gmail.com 
######################################
*/

/*
hardware connection:
4-bit mode: Data is transferred in two 4-bit chunks (uses 4 data lines + control lines).

PC0 = RS
PC1 = RW
PC2 = E

PC3 = D4 
PC4 = D5
PC5 = D6
PC6 = D7

*/

#include <stdint.h>
#include "stm32f103Driver.h"
#include "../Src/config.h"


void LCD1602_Send_data(unsigned char data) {
    unsigned char highNibble, lowNibble;

    // Split the 8-bit data into high and low nibbles
    highNibble = (data >> 4) & 0x0F;  // Extract high 4 bits
    lowNibble = data & 0x0F;          // Extract low 4 bits

    // Send the High nibble (MSB)
    Write_GPIO(portC, 0, HIGH);  // RS=1 indicates data transfer
    Write_GPIO(portC, 1, LOW);   // 0 for write mode
    delay_miliSecond(3, 10,system_freq);
    

    Write_GPIO(portC, 2, HIGH);  // Enable pin high means ready to transfer
    delay_miliSecond(3, 10,system_freq);
    GPIOC->ODR &= ~(0xF << 3);   // Clears GPIOC pins 3, 4, 5, 6
    GPIOC->ODR |= (highNibble << 3); // Send the MSB data (high nibble)
    delay_miliSecond(3, 10,system_freq);
    Write_GPIO(portC, 2, LOW);   // Enable pin low means end of transfer
    delay_miliSecond(3, 10,system_freq);

    // Send the Low nibble (LSB)
    Write_GPIO(portC, 2, HIGH);  // Enable pin high means ready to transfer
    delay_miliSecond(3, 10,system_freq);
    GPIOC->ODR &= ~(0xF << 3);   // Clears GPIOC pins 3, 4, 5, 6
    GPIOC->ODR |= (lowNibble << 3); // Send the LSB data (low nibble)
    delay_miliSecond(3, 10,system_freq);
    Write_GPIO(portC, 2, LOW);   // Enable pin low means end of transfer
    delay_miliSecond(3, 10,system_freq);
}


void LCD1602_Send_command(unsigned char data) {
    unsigned char highNibble, lowNibble;

    // Split the 8-bit data into high and low nibbles
    highNibble = (data >> 4) & 0x0F;  // Extract high 4 bits
    lowNibble = data & 0x0F;          // Extract low 4 bits

    // Send the High nibble (MSB)
    Write_GPIO(portC, 0, LOW);  // RS=1 indicates data transfer
    Write_GPIO(portC, 1, LOW);   // 0 for write mode
    delay_miliSecond(3, 10,system_freq);

    Write_GPIO(portC, 2, HIGH);  // Enable pin high means ready to transfer
    delay_miliSecond(3, 10,system_freq);
    GPIOC->ODR &= ~(0xF << 3);   // Clears GPIOC pins 3, 4, 5, 6
    GPIOC->ODR |= (highNibble << 3); // Send the MSB data (high nibble)
    delay_miliSecond(3, 10,system_freq);
    Write_GPIO(portC, 2, LOW);   // Enable pin low means end of transfer
    delay_miliSecond(3, 10,system_freq);

    // Send the Low nibble (LSB)
    Write_GPIO(portC, 2, HIGH);  // Enable pin high means ready to transfer
    delay_miliSecond(3, 10,system_freq);
    GPIOC->ODR &= ~(0xF << 3);   // Clears GPIOC pins 3, 4, 5, 6
    GPIOC->ODR |= (lowNibble << 3); // Send the LSB data (low nibble)
    delay_miliSecond(3, 10,system_freq);
    Write_GPIO(portC, 2, LOW);   // Enable pin low means end of transfer
    delay_miliSecond(3, 10,system_freq);
}


void LCD1602_init (void){
	//configuring the corresponding gpio pins as output
	for (int i = 0; i<=6; i++){
		Config_GPIO (portC, i, output_50Mhz, gp_output);
	}	
	delay_miliSecond(3, 10,system_freq);
	
	 // Start initialization sequence for the LCD
   // Step 1: Set the interface to 8-bit mode initially
	LCD1602_Send_command(0x30); // 8-bit mode
	delay_miliSecond(3, 10,system_freq);
	LCD1602_Send_command(0x30); // Repeat to ensure 8-bit mode is stable
	delay_miliSecond(3, 10,system_freq);
	LCD1602_Send_command(0x30);// Repeat to ensure 8-bit mode is stable
	delay_miliSecond(3, 10,system_freq);
	
	// Step 2: Switch to 4-bit mode
	LCD1602_Send_command(0x20); // 4-bit mode
	delay_miliSecond(3, 10,system_freq);
	
	// Step 3: Configure the display: 2 lines, 5x8 font
  LCD1602_Send_command(0x28); // 4-bit mode, 2 lines, 5x8 font
  delay_miliSecond(3, 10,system_freq);
	
	// Step 4: Turn on display, disable cursor
  LCD1602_Send_command(0x0C); // Display ON, Cursor OFF, Blink OFF
  delay_miliSecond(3, 10,system_freq);
	
	// Step 5: Clear the display
  LCD1602_Send_command(0x01); // Clear display
  delay_miliSecond(3, 10,system_freq);
	
	// Step 6: Set cursor to home position
  LCD1602_Send_command(0x02); // Return home
  delay_miliSecond(3, 10,system_freq);
}

void lcd1602_send_string (char text[]){
	int i = 0;
	while (text[i])
	{
		LCD1602_Send_data(text[i]);
		i++;
		delay_miliSecond(3, 1,system_freq);
	}
}

void LCD1602_print (uint8_t line, uint16_t position, char message[]){
	short pos = 0;
	if (line == 1)
	{
		pos = 0;
	}
	else if (line == 2){
		pos = 0x40;
	}
	LCD1602_Send_command(0x80 + pos + position);
	delay_miliSecond(3, 1,system_freq);
	lcd1602_send_string(message);

}


// void LCD1602_scroll_from_right(char message[], uint8_t line, uint16_t delay_ms) {
//     int length = 0;
//     for (const char *p = message; *p != '\0'; ++p) {
//             length++;
//         }
//     char buffer[17] = {0};          // Temporary buffer for the display (16 chars + null)
//     int total_shifts = length + 16; // Total shifts (message + screen width)

//     for (int i = 0; i < total_shifts; i++) {
//         // Fill the buffer with spaces and text for the current position
//         for (int j = 0; j < 16; j++) {
//             if ((i + j) >= 16 && (i + j - 16) < length) {
//                 buffer[j] = message[i + j - 16];  // Visible portion of the message
//             } else {
//                 buffer[j] = ' ';  // Fill empty spaces
//             }
//         }

//         buffer[16] = '\0';         // Null-terminate the string
//         LCD1602_print(line, 0, buffer); // Print the buffer on the specified line
//         delay_miliSecond(3, delay_ms ,system_freq);
//     }
// }


// void LCD1602_scroll_from_left(char message[], uint8_t line, uint16_t delay_ms) {
//    int length = 0;
//     for (const char *p = message; *p != '\0'; ++p) {
//             length++;
//         }
//     char buffer[17] = {0};          // Buffer to hold the current text displayed (16 characters max + null terminator)

//     for (int shift = 16 + length; shift >= 0; shift--) { // Start from the end of the message
//         for (int j = 0; j < 16; j++) {  // Populate the buffer for the 16 characters on the screen
//             int char_index = shift - 16 + j; // Calculate the character position in the message

//             if (char_index >= 0 && char_index < length) {
//                 buffer[j] = message[char_index];  // Copy valid characters from the message
//             } else {
//                 buffer[j] = ' ';  // Fill with spaces when out of bounds
//             }
//         }

//         buffer[16] = '\0'; // Null-terminate the buffer
//         LCD1602_print(line, 0, buffer); // Print the buffer to the specified line of the LCD
//         delay_miliSecond(3, delay_ms ,system_freq);        // Delay for smooth scrolling
//     }
// }
