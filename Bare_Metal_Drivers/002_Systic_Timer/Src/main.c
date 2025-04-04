#include <stdint.h>
#include "../Drivers/stm32f103SysticDrive.h"

// int count = 0; 
// void SysTick_Handler(void){
// 	count++; 
// 	if (count == 20){
// 		toggle_gpio (portC, 0); 
// 		count = 0;
// 	}
	
// }

int main (){
	Config_GPIO(portC, 0, output_50Mhz, gp_output);
	systic_init();
	
	//Systic_interrupt();
	
	while (1){
		toggle_gpio(portC, 0);
		Delay_Sys_MS(100);
			
			
			
			
		}
	
}