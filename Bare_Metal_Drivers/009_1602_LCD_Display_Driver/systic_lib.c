#include <stdio.h>
#include "header.h"



/*
void SysTick_Handler(void)
{
	Systic interrup handler
}
*/

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
    SysTick->CTRL |= (101 << 0);
}




void Sys_delay_microsecond() {
    // Set the reload value to 72. Assuming the AHB clock is 72 MHz, this provides
    // a 1 microsecond delay since the SysTick counter decrements by 1 per clock cycle.
    SysTick->LOAD = 72;

    // Clear the current value register (VAL), which resets the counter to zero.
    SysTick->VAL = 0;

    // Wait for the COUNTFLAG (bit 16 of CTRL) to be set, indicating the counter has counted down to 0.
    while ((SysTick->CTRL & 0x00010000) == 0);
}

void Sys_Delay_Milisecond () {
	SysTick->LOAD = 72000;
	SysTick->VAL = 0;
	 while ((SysTick->CTRL & 0x00010000) == 0);
}

void Delay_Sys_US (uint16_t t){
	for (;t>0;t--){
		Sys_delay_microsecond ();
	}
}

void Delay_Sys_MS (uint16_t t) {
	for (;t>0;t--){
		Sys_Delay_Milisecond ();
	}
}

void Systic_interrupt (){
	__disableirq();
	SysTick->CTRL = 0;
	SysTick->LOAD = 7200000-1; //for 0.1 second delay
	SysTick->VAL = 0;
	SysTick ->CTRL |= (111<<0);
	__enableirq();
}

