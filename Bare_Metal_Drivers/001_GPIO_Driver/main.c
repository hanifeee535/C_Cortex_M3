#include <stdint.h>
#include "Drivers/stm32f103GPIODrive.h"

int main (){
	Config_GPIO(portC, 0, output_50Mhz, gp_output);
	
	
	while (1){
		
			Write_GPIO(portC, 0, HIGH);
			for (int i=0; i<10000; i++) {}
			Write_GPIO(portC, 0, LOW);
			for (int i=0; i<1000000; i++) {}
		}
	
}
