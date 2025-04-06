#include <stdint.h>
#include "../Drivers/stm32f103Driver.h"

 


int main ()
{
	//init_USART(2, 9600);
	init_SPI(2, 128);
	systic_init();
	
	while (1){
		spi_send_message(SPI_2, "HELLO SPI");
		Delay_Sys_MS(50);
		//spi_transmit(1, 'R');
		
		
		}
	


}