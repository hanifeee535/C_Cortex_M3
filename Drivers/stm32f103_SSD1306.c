#include <stdint.h>
#include "stm32f103Driver.h"
#include "font.h"
#include "lcd_image.h"

#define OLED_WIDTH 128
#define OLED_PAGES 8




//send one byte of command: 
void comand_1byte (uint8_t i2c,  char data ) {

	i2c_start (i2c);
	i2c_send_address (i2c, 0x78, 0);
	i2c_send_data (i2c, 0x00); //to send a command, we need to send a byte of 0x00 before the command byte. 
	i2c_send_data (i2c, data);	
	i2c_stop (i2c);
}

//send two byte of command 
void comand_2byte (uint8_t i2c,  char data[] ) {
	int i = 0;
	i2c_start (i2c);
	i2c_send_address (i2c, 0x78, 0);
	i2c_send_data (i2c, 0x00); //to send a command, we need to send a byte of 0x00 before the command byte. 
	for (i = 0; i < 2; i++){
		i2c_send_data (i2c, data[i]);	
	}
	
	i2c_stop (i2c);
}

//oled configuration function
void oled_configure ( uint8_t i2c, uint8_t screen_size ){
	i2c_init(i2c,i2c_FastMode);
	
	 // Set the Multiplex Ratio for the OLED display (0xA8: Set Multiplex Ratio, 0x3F: 64 multiplex mode)
    char multiplex_cmd[] = {0xA8, 0x3F}; // Multiplex ratio set to 64
    comand_2byte(i2c, multiplex_cmd);

    // Set the Display Offset (0xD3: Set Display Offset, 0x00: No offset)
    char offset_cmd[] = {0xD3, 0x00}; // Offset is 0
    comand_2byte(i2c, offset_cmd);

    // Set the Start Line (0x40: Set Start Line, starts at line 0)
    comand_1byte(i2c, 0x40); // Start at line 0

    // Set the Segment Re-map (0xA1: Reverse Segment Remap, flip column addresses)
    comand_1byte(i2c, 0xA1); // Reverses the segment mapping

    // Set the COM Output Scan Direction (0xC8: COM Scan Direction, flip rows)
    comand_1byte(i2c, 0xC8); // Reverses the COM output scan direction

    // Set COM Pins hardware configuration (0xDA: Set COM Pins, OLED screen size)
    char com_pins_cmd[] = {0xDA, screen_size}; // Configure COM pins according to the screen size
    comand_2byte(i2c, com_pins_cmd);

    // Set Contrast Control (0x81: Set Contrast, 0x7F: Maximum contrast)
    char contrast_cmd[] = {0x81, 0x7F}; // Set contrast to the maximum value (0x7F)
    comand_2byte(i2c, contrast_cmd);

    // Disable Entire Display On (0xA4: Display on/off, 0xA6: Normal display mode)
    comand_1byte(i2c, 0xA4); // Display mode: Normal (not Entire Display On)
    comand_1byte(i2c, 0xA6); // Set Normal display mode (default display mode)

    // Set the Display Clock Divide Ratio (0xD5: Set Display Clock, 0x80: Default clock divider)
    char display_clock_cmd[] = {0xD5, 0x80}; // Set the display clock divider to default value
    comand_2byte(i2c, display_clock_cmd);

    // Set Pre-charge period (0x8D: Set Pre-charge Period, 0x14: Default pre-charge period)
    char precharge_cmd[] = {0x8D, 0x14}; // Set the pre-charge period value
    comand_2byte(i2c, precharge_cmd);

    // Turn on the OLED display (0xAF: Turn on display)
    comand_1byte(i2c, 0xAF); // Turn on the OLED display

    // Set Addressing Mode (0x20: Set Addressing Mode, 0x10: Horizontal addressing mode)
    char addressing_mode_cmd[] = {0x20, 0x10}; // Set the addressing mode to horizontal mode
    comand_2byte(i2c, addressing_mode_cmd);
}

void oled_configure_64 (uint8_t i2c){
	oled_configure (i2c, 0x12);
}

void oled_configure_32 (uint8_t i2c){
	oled_configure (i2c, 0x22);
}

//send data to the oled: 
void oled_send_data (uint8_t i2c,  char data ) {

	i2c_start (i2c);
	i2c_send_address (i2c, 0x78, 0);
	i2c_send_data (i2c, 0x40); //to send a data, we need to send a byte of 0x40 before the actual data byte. 
	i2c_send_data (i2c, data);	
	i2c_stop (i2c);
}

//position function
void oled_pos(uint8_t i2c, uint8_t Ypos, uint8_t Xpos)
{
    // Set the lower 4 bits of Xpos
    comand_1byte(i2c, 0x00 + (Xpos & 0x0F));  // Lower nibble of X position (columns 0-15)
    
    // Set the upper 4 bits of Xpos
    comand_1byte(i2c, 0x10 + ((Xpos >> 4) & 0x0F)); // Upper nibble of X position (columns 16-127)
    
    // Set the page (Ypos)
    comand_1byte(i2c, 0xB0 + Ypos);  // Pages 0-7 for vertical positioning
}



// Function to clear the OLED screen by filling it with blank (off) pixels
void oled_blank(uint8_t i2c)
{
    int i, j;

    // Set the cursor position to the top-left corner of the OLED screen (Y = 0, X = 0)
    oled_pos(i2c, 0, 0);

    // Loop through the 8 pages (rows of pixels in vertical direction)
    // For a 128x64 OLED display, there are 8 pages (each page contains 8 rows)
    for(i = 0; i < 8; i++)
    {
        // Loop through all 128 columns in the current page
        // Each column in the page has 8 pixels vertically
        for(j = 0; j < 128; j++)
        {
            // Send 0x00 (binary: 00000000) to turn off all pixels in the current position
            // This fills each pixel in the column with an "off" state (empty)
            oled_send_data(i2c, 0x00);
        }
    }

    // Once the screen is cleared, set the cursor back to the top-left corner (Y = 0, X = 0)
    oled_pos(i2c, 0, 0);
}



void oled_print_5X8_font(uint8_t i2c, const char *str, uint8_t scale, 
                         uint8_t start_Y, uint16_t start_X, uint8_t inverted) {
    for(int i = 0; str[i]; i++) { 
        const uint8_t *char_data = ASCII_5X8[str[i] - 32]; // 5 columns per character

        // Vertical scaling
        for(uint8_t v_page = 0; v_page < scale; v_page++) {
            uint8_t current_Y = start_Y + v_page;
            uint16_t current_X = start_X + (i * 5 * scale); // 5 columns per char
            oled_pos(i2c, current_Y, current_X);

            // Process only 5 columns (original 5x8 font)
            for(uint8_t col = 0; col < 5; col++) { //  5 column
                uint8_t original = char_data[col];
                
                // Invert all pixels
                if(inverted) original = ~original;

                // Vertical scaling logic
                uint8_t scaled_col = 0;
                for(uint8_t bit = 0; bit < 8; bit++) { // 8 rows per column
                    if(original & (1 << bit)) {
                        // Expand each pixel vertically
                        for(uint8_t s = 0; s < scale; s++) {
                            uint16_t scaled_bit = (bit * scale + s);
                            if((scaled_bit / 8) == v_page) {
                                scaled_col |= (1 << (scaled_bit % 8));
                            }
                        }
                    }
                }

                // Horizontal scaling (repeat column 'scale' times)
                for(uint8_t h = 0; h < scale; h++) {
                    oled_send_data(i2c, scaled_col);
                }
            }
        }
    }
}


void oled_send_message_5X8_font(uint8_t i2c, const char *str, uint8_t scale, uint8_t start_Y, uint16_t start_X,uint8_t inverted ) {
    oled_print_5X8_font(i2c, str, scale, start_Y, start_X, inverted); 
 }


void oled_print_8X8_font(uint8_t i2c, const char *str, uint8_t scale, uint8_t start_Y, uint16_t start_X,uint8_t inverted ) {
    for (int i = 0; str[i]; i++) { 
        const uint8_t *char_data = ASCII_8X8[str[i] - 32];

        // Vertical scaling
        for (uint8_t v_page = 0; v_page < scale; v_page++) {
            uint8_t current_Y = start_Y + v_page;
            uint16_t current_X = start_X + (i * 8 * scale);
            oled_pos(i2c, current_Y, current_X);

            for (uint8_t col = 0; col < 8; col++) {
                uint8_t original = char_data[col];
                
                // Invert all pixels (text and background)
                if (inverted) original = ~original;

                // Vertical scaling logic
                uint8_t scaled_col = 0;
                for (uint8_t bit = 0; bit < 8; bit++) {
                    if (original & (1 << bit)) {
                        for (uint8_t s = 0; s < scale; s++) {
                            uint16_t scaled_bit = (bit * scale + s);
                            if ((scaled_bit / 8) == v_page) {
                                scaled_col |= (1 << (scaled_bit % 8));
                            }
                        }
                    }
                }

                // Send data
                for (uint8_t h = 0; h < scale; h++) {
                    oled_send_data(i2c, scaled_col);
                }
            }
        }
    }
}

void oled_send_message_8X8_font(uint8_t i2c, const char *str, uint8_t scale, uint8_t start_Y, uint16_t start_X,uint8_t inverted ) {
    oled_print_8X8_font(i2c, str, scale, start_Y, start_X, inverted); 
 }





 //clearing the oled buffer
 void clear_oled_buffer (unsigned char oled_buffer[][128]){
    for (int i = 0; i<8; i++){
        for (int j = 0; j<128; j++){
            oled_buffer [i][j] = 0;
        }
    }
 }


 void update_oled_buffer (lcd_image_typedef *img, uint8_t img_num, unsigned char oled_buffer[][128] ){

    int x_dir, y_dir, endx, endy, cnt;
    if ((img->width + img->x_position)>128){
        endx = 127;
    }

    else {
        endx = (img->width + img->x_position)-1;
    }

    if ((img->height + img->y_position)>8){
        endy = 7;
    }

    else {
        endy = (img->height + img->y_position)-1;
    }

    cnt = 0;
    for (y_dir = img->y_position; y_dir<=endy; y_dir++){
        for (x_dir = img->x_position; x_dir<= endx; x_dir++){
            cnt = (y_dir - img->y_position) * img->width + (x_dir - img->x_position);
            oled_buffer [y_dir][x_dir] = img->image[img_num][cnt];
        }
    }

 }

 void print_buffer (uint8_t i2c, unsigned char oled_buffer [][128]){

    oled_pos( i2c,0,0);
    for (int i = 0; i<8; i++){
        for (int j= 0; j<128; j++){
            oled_send_data ( i2c,  oled_buffer[i][j] );
        }
    }

 }



