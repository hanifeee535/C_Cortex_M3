
#include <stdint.h>
#include "stm32f103Driver.h"


void SystemClock_Config(){
      //enable HSE (8 MHz external crystal)
      RCC->CR |= (1<<16);
      while (!(RCC->CR & (1 << 17)));

      // Set Flash latency for 72 MHz
      FLASH->ACR |= (0x2 << 0) ;//flash latency 2

      // Configure PLL: HSE (8MHz) ×9 = 72 MHz
      RCC->CFGR |=  (0x1 << 16) |(0x7 << 18); //HSE as PLL input| PLL multiplier ×9

    // Enable PLL and wait until ready
    RCC->CR |= (1 << 24);   // PLL enable
    while (!(RCC->CR & (1 << 25)  )); // PLL ready flag

    // Switch to PLL system clock
    RCC->CFGR |=  (0x2 << 0) ; // PLL as system clock
    while (((RCC->CFGR) & (0x2 << 2)) != (0x2 << 2));

    // APB1 Prescaler: HCLK divided by 2 (36 MHz for USART2/3)
    RCC->CFGR |= (0x4 << 8); // PPRE1 = 0b100 (HCLK/2)
    // APB2 Prescaler: HCLK not divided (72 MHz for USART1)
    RCC->CFGR |= (0x0 << 11); // PPRE2 = 0b000 (HCLK/1)



   }


//dwt cycle counter
void dwt_init(void) {
    // Enable trace and debug block DEMCR (bit 24 = TRCENA)
    DEMCR |= (1 << 24);

    // Reset the cycle counter
    DWT_CYCCNT = 0;

    // Enable the cycle counter (bit 0 of DWT_CTRL)
    DWT_CTRL |= (1 << 0);
}

