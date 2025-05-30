#ifndef LCD_IMAGE_H_
#define LCD_IMAGE_H_

#include <stdint.h>

typedef struct {
    unsigned short height; 
    unsigned short width;
    const unsigned char *image[5]; 
    short x_position;
    short y_position;	
} lcd_image_typedef;

// Declare variables as extern
extern const unsigned short weew_stack_rows;
extern const unsigned short weew_stack_cols;
extern const unsigned char weew_stack[];

#endif /* LCD_IMAGE_H_ */