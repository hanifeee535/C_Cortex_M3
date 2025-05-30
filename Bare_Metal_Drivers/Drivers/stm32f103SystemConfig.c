/* ####################################
Author: MD. Soyabbir Abu Hanif
 MSc in Embedded Systems
Tampere University, Finland
Email: hanifseceee535@gmail.com 
######################################
*/

/*
******************************************************************** 
********************__NVIC_Priority_Setting__***********************
********************************************************************
@Function name:
     void NVIC_SetPriority(uint8_t irq_number, uint8_t priority_level)
@Purpose:
    Sets the interrupt priority for a specific IRQ (Interrupt Request Line) in the Nested Vectored Interrupt Controller (NVIC) for the STM32F103 (Cortex-M3 core).
@Parameters:
        Parameter	        Type	            Description
        irq_number	        uint8_t	            IRQ number of the peripheral interrupt (e.g., EXTI0 = 6, TIM2 = 28, etc.)
        priority_level	    uint8_t	4-bit       priority level (0–15), where 0 is highest and 15 is lowest

    Only the upper 4 bits of each 8-bit priority register are used on STM32F103.

    Values beyond 0–15 are masked automatically (priority_level &= 0x0F).

@Working:

    The NVIC stores interrupt priorities in the Interrupt Priority Registers (NVIC_IPRx).

    Each IPR register holds priorities for 4 IRQs.

    This function:

    Calculates the correct register and byte offset.

    Writes the 4-bit priority to the correct upper nibble (bits 7:4) of that byte.

@Example Usage:
    NVIC_SetPriority(6, 3); // Set EXTI0 (IRQ 6) to priority level 3





********************************************************************************
********************__Configuring system clock from HSE__***********************
*******************************************************************************

@Function name: int ConfigureSystemClock_HSE(uint32_t hse_freq, uint32_t target_sysclk)

@Purpose:
    Dynamically configures the system clock using the external high-speed oscillator (HSE) as the input source, 
    with an optional PLL multiplication to achieve the desired system clock frequency.

@Parameters:

    Name	        Type	                Description

    hse_freq	    uint32_t	            Frequency of the external crystal oscillator in Hz (e.g., 8000000 for 8 MHz)
    target_sysclk	uint32_t	            Desired system clock frequency in Hz (max 72 MHz)

@Return Value:

    Value	        Description

     0	            Success
    -1	            Invalid parameters or configuration

@Behavior:

        If the requested system clock equals the HSE frequency (no PLL needed), it switches directly to HSE.

        If PLL is required, it calculates and applies the appropriate multiplier.

        Configures Flash latency and bus prescalers accordingly.

        Ensures that the HSE and PLL are ready before switching the system clock source.

        Valid PLL multiplier range: 2 to 16.

@Example:
        ConfigureSystemClock_HSE(8000000, 72000000); // Configures system clock to 72 MHz using 8 MHz HSE



********************************************************************************
********************__Configuring system clock from HSI__***********************
*******************************************************************************

@Function name: int ConfigureSystemClock_HSI(uint32_t target_sysclk)

@Purpose:
    Dynamically configures the system clock using the internal high-speed oscillator (HSI)
     divided by 2 as the PLL input source to achieve the specified system clock frequency.

@Parameters:

    Name	        Type	                Description
    
    target_sysclk	uint32_t	            Desired system clock frequency in Hz (max 64 MHz)

@Return Value:

    Value	        Description

     0	            Success
    -1	            Invalid PLL multiplier or mismatch

@Behavior:

        Enables and uses HSI (8 MHz), which is internally divided by 2 (4 MHz) as the PLL input.

        Calculates and applies the required PLL multiplier.

        Configures Flash latency and bus prescalers for safe high-speed operation.

        Switches the system clock source to PLL once it is ready.

        Valid PLL multiplier range: 2 to 16.

        Ensures precise match between desired frequency and achievable PLL output.

@Example:
        ConfigureSystemClock_HSI(48000000); // Uses HSI/2 (4 MHz) × 12 = 48 MHz system clock



**********************************************************/

#include <stdint.h>
#include "stm32f103Driver.h"

void NVIC_SetPriority(uint8_t irq_number, uint8_t priority_level) {
    // Mask priority to 4 bits
    priority_level &= 0x0F;

    // Calculate IPR register index and position
    uint32_t ipr_index = irq_number / 4;
    uint8_t byte_pos = irq_number % 4;
    uint32_t shift = (byte_pos * 8) + 4;  // Upper nibble of each byte

    // Get pointer to the NVIC_IPR register
    volatile uint32_t *nvic_ipr = (volatile uint32_t *)(0xE000E400 + (ipr_index * 4));

    // Apply priority (only upper 4 bits used in each byte)
    uint32_t mask = 0x0F << shift;
    *nvic_ipr = (*nvic_ipr & ~mask) | ((priority_level & 0x0F) << shift);
}


int ConfigureSystemClock_HSE(uint32_t hse_freq, uint32_t target_sysclk) {
    // Check for invalid frequency or system clock request
    if (hse_freq == 0 || target_sysclk > 72000000) return -1;

    // Calculate the required PLL multiplier
    uint32_t pll_mul = target_sysclk / hse_freq;
		//uint32_t PLLMUL_VAL = pll_mul-2;
    // Make sure the multiplier is valid and results in an exact frequency
    if (pll_mul < 1 || pll_mul > 16 || (pll_mul * hse_freq != target_sysclk)) return -1;

    // Enable the HSE oscillator
    RCC->CR |= (1 << 16);
    // Wait until the HSE is stable and ready
     while (!(RCC->CR & (1 << 17)));

    if (pll_mul == 1) {
        // No PLL needed, use HSE directly
       
        RCC->CFGR &= ~(0x3 << 0);       // Clear SW bits
        RCC->CFGR |=  (0x1 << 0);       // Select HSE as system clock

        while (((RCC->CFGR >> 2) & 0x3) != 0x1); // Wait for HSE to be system clock
    } else {
        

        // Configure Flash latency
        FLASH->ACR &= ~(0x7 << 0);  // Clear LATENCY bits [2:0]
        if (target_sysclk > 48000000) {
            FLASH->ACR |= (0x2 << 0);  // 2 wait states
        } else if (target_sysclk > 24000000) {
            FLASH->ACR |= (0x1 << 0);  // 1 wait state
        } else {
            FLASH->ACR |= (0x0 << 0);  // 0 wait states
        }

        // Configure PLL
        RCC->CFGR &= ~((1 << 16) | (1 << 17) | (0xF << 18));
        RCC->CFGR |= (1 << 16);                             // PLLSRC = HSE
        RCC->CFGR &= ~(0xF << 18);                // Clear PLLMUL bits
				RCC->CFGR |= ((pll_mul - 2) << 18);       // Set desired PLLMUL

        RCC->CR |= (1 << 24);           // PLLON
        while (!(RCC->CR & (1 << 25))); // Wait for PLLRDY

        // Prescalers
        RCC->CFGR &= ~(0x7 << 8);       // Clear APB1 prescaler
        RCC->CFGR |=  (0x4 << 8);       // APB1 = HCLK/2
        RCC->CFGR &= ~(0x7 << 11);      // APB2 = HCLK

        // Switch to PLL
        RCC->CFGR &= ~(0x3 << 0);
        RCC->CFGR |=  (0x2 << 0);
        while (((RCC->CFGR >> 2) & 0x3) != 0x2); // Wait for PLL as sysclk
    }

    return 0;
}



int ConfigureSystemClock_HSI(uint32_t target_sysclk) {
    uint32_t hsi_freq = 8000000;               // HSI = 8 MHz
    uint32_t pll_input = hsi_freq / 2;         // HSI/2 is used for PLL input
    uint32_t pll_mul = target_sysclk / pll_input;

    // Validate PLL multiplier and exact frequency match
    if (pll_mul < 2 || pll_mul > 16 || (pll_mul * pll_input != target_sysclk)) {
        return -1;
    }

    // Enable HSI (bit 0)
    RCC->CR |= (1 << 0);
    // Wait for HSI ready (bit 1)
    while (!(RCC->CR & (1 << 1)));

    // Configure Flash latency
    FLASH->ACR &= ~(0x7 << 0);  // Clear LATENCY bits [2:0]
    if (target_sysclk > 48000000) {
        FLASH->ACR |= (0x2 << 0);  // 2 wait states
    } else if (target_sysclk > 24000000) {
        FLASH->ACR |= (0x1 << 0);  // 1 wait state
    } else {
        FLASH->ACR |= (0x0 << 0);  // 0 wait states
    }

    // Clear PLLSRC, PLLXTPRE, and PLLMUL bits
    RCC->CFGR &= ~((1 << 16) | (1 << 17) | (0xF << 18));
    // PLLSRC = 0 means HSI/2 is selected (so we don't need to set anything)
    RCC->CFGR |= ((pll_mul - 2) << 18);  // Set PLLMUL bits

    // Enable PLL (bit 24)
    RCC->CR |= (1 << 24);
    // Wait for PLL ready (bit 25)
    while (!(RCC->CR & (1 << 25)));

    // Prescalers
    RCC->CFGR &= ~(0x7 << 8);       // Clear APB1 prescaler
    RCC->CFGR |=  (0x4 << 8);       // APB1 = HCLK/2
    RCC->CFGR &= ~(0x7 << 11);      // APB2 = HCLK

    // Switch to PLL as system clock
    RCC->CFGR &= ~(0x3 << 0);       // Clear SW bits
    RCC->CFGR |=  (0x2 << 0);       // Set SW = 0b10 (PLL selected)
    while (((RCC->CFGR >> 2) & 0x3) != 0x2); // Wait for PLL to be used

    return 0;
}







/*





//dwt cycle counter
void dwt_init(void) {
  // Enable trace and debug block DEMCR (bit 24 = TRCENA)
  DEMCR |= (1 << 24);

  // Reset the cycle counter
  DWT_CYCCNT = 0;

  // Enable the cycle counter (bit 0 of DWT_CTRL)
  DWT_CTRL |= (1 << 0);
}
*/


// Set Priority Grouping 
void NVIC_SetPriorityGrouping(uint32_t priority_group) {
  uint32_t reg_value;

  reg_value  = SCB_AIRCR;
  reg_value &= ~(SCB_AIRCR_VECTKEY_MASK | SCB_AIRCR_PRIGROUP_MASK); // Clear fields
  reg_value |= (SCB_AIRCR_VECTKEY | priority_group);                // Set key and group
  SCB_AIRCR  = reg_value;
}



