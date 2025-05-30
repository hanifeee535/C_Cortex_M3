/* ####################################
Author: MD. Soyabbir Abu Hanif
 MSc in Embedded Systems
Tampere University, Finland
Email: hanifseceee535@gmail.com 
######################################
*/

/*
******************************************************************** 
********************__Systic_init__***********************
********************************************************************
@Function name: void systic_init(void)
@Purpose: 
    Initializes the SysTick timer for general-purpose delay or timing operations without interrupts.
@Parameters: none
@Return Value: none
@Usage:
    Used for delay-based functionality or manual polling.
    Systic must be initialized by calling this function before calling any systic delay function



******************************************************************** 
********************__Delay_Sys_MS__***********************
********************************************************************
@Function name: 
    void Delay_Sys_MS(uint16_t t, uint32_t clock_freq)

@Purpose:
    Creates a blocking delay of t milliseconds using SysTick.

@Parameters:

    Name	        Type	        Description

    t	            uint16_t	    Delay duration in milliseconds
    clock_freq	    uint32_t	    System clock frequency in Hz



******************************************************************** 
********************__Delay_Sys_US__***********************
********************************************************************
@Function name: 
    void Delay_Sys_US(uint16_t t, uint32_t clock_freq)
@Purpose:
    Creates a blocking delay of t microseconds using SysTick.

@Parameters:
        Name	        Type	        Description
        t	           uint16_t	        Delay duration in microseconds
        clock_freq	   uint32_t	        System clock frequency in Hz


example usage: 
    systic_init();
    Delay_Sys_US(500, 8000000);



******************************************************************** 
********************__Systic_Interrupt__***********************
********************************************************************

@Function name:
    void Systick_interrupt(uint32_t clock_freq, uint32_t interval_ms)

@Purpose:
    Configures SysTick to generate an interrupt at a regular interval in milliseconds.

@Parameters:

        Name	                 Type	            Description

        clock_freq	            uint32_t	        System clock frequency in Hz
        interval_ms	            uint32_t	        Desired interval between interrupts in ms


@Behavior:

    Calculates the number of ticks needed for the specified interval.

    Sets up the SysTick timer to use the processor clock with interrupt enabled.

    Caps tick count at 0xFFFFFF to avoid overflow.

    Enables the timer and re-enables global interrupts.

@Note:
    An interrupt handler (e.g., SysTick_Handler) must be defined elsewhere to respond to SysTick interrupts.

**********************************************************/



#include <stdint.h>
#include "stm32f103Driver.h"


void systic_init(void) {
    // Disable the SysTick timer by clearing the CTRL register
    SysTick->CTRL = 0;

    // Set the reload value to the maximum possible (24-bit value)
    SysTick->LOAD = 0x00FFFFFF;

    // Clear the current value register (VAL), which resets the SysTick counter
    SysTick->VAL = 0;

    // Enable SysTick by setting the ENABLE bit (bit 0) and select the clock source
    // as the processor clock (AHB) by setting the CLKSOURCE bit (bit 2).
    // The binary value 101 corresponds to enabling the counter and selecting the clock source.
    SysTick->CTRL |= (1 << 0) | (1 << 2); // ENABLE + CLKSOURCE (AHB)
}



void Sys_Delay_Milisecond (uint32_t clock_freq) {
    // Calculate ticks needed for 1 ms
    uint32_t ticks_per_ms = clock_freq / 1000;
	SysTick->LOAD = ticks_per_ms-1;
	SysTick->VAL = 0;
	 while ((SysTick->CTRL & 0x00010000) == 0);
}


void Delay_Sys_US (uint16_t t, uint32_t clock_freq){
    uint32_t ticks = (clock_freq / 1000000) * t;
    if (ticks > 0xFFFFFF) ticks = 0xFFFFFF;

    SysTick->LOAD = ticks - 1;
    SysTick->VAL = 0;
    //__asm volatile ("nop");

    while ((SysTick->CTRL & (1 << 16)) == 0);
}

void Delay_Sys_MS (uint16_t t, uint32_t clock_freq) {
	for (;t>0;t--){
		Sys_Delay_Milisecond (clock_freq);
	}
}


void Systick_interrupt(uint32_t clock_freq, uint32_t interval_ms) {
    // Calculate ticks needed for the desired interval
    // interval_ms is in milliseconds, so convert to seconds: interval_ms / 1000
    uint32_t ticks = (clock_freq / 1000) * interval_ms;

    // SysTick is 24-bit, check if value is too large
    if (ticks > 0xFFFFFF) ticks = 0xFFFFFF;

    __disableinterrupt();             // Disable global interrupts

    SysTick->CTRL = 0;           // Disable SysTick during setup
    SysTick->LOAD = ticks - 1;   // Set reload value
    SysTick->VAL = 0;            // Clear current value

    // Enable SysTick with:
    // - processor clock (CLKSOURCE = 1)
    // - interrupt enabled (TICKINT = 1)
    // - counter enabled (ENABLE = 1)
    SysTick->CTRL = (1 << 2) | (1 << 1) | (1 << 0);

    __enableinterrupt();              // Re-enable global interrupts
}