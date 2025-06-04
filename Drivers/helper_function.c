
#include <stdint.h>
#include "stm32f103Driver.h"

//converting integer number to character string
void int_to_str(int8_t num, char buffer[]) {
    uint8_t idx = 0;
    int is_negative = 0;

    // Handle zero
    if (num == 0) {
        buffer[0] = '0';
        buffer[1] = '\0';
        return;
    }

    // Handle negative numbers
    if (num < 0) {
        is_negative = 1;
        num = -num;
    }

    // Convert digits to characters
    while (num > 0) {
        buffer[idx++] = (num % 10) + '0';
        num /= 10;
    }

    // Add minus sign if needed
    if (is_negative) {
        buffer[idx++] = '-';
    }

    // Null-terminate
    buffer[idx] = '\0';

    // Reverse the buffer
    for (uint8_t i = 0; i < idx / 2; i++) {
        char temp = buffer[i];
        buffer[i] = buffer[idx - i - 1];
        buffer[idx - i - 1] = temp;
    }
}
