#include <stdint.h>
#include "../Drivers/stm32f103Driver.h"

 

char data [2] = {0x01, 0x02};
int main ()
{	
	i2c_init(1,i2c_FastMode);
	//init_USART(2, 9600);
	//init_SPI(2, 128);
	systic_init();
	
	while (1){
		//spi_send_message(SPI_2, "HELLO SPI");
		//Delay_Sys_MS(50);
		//spi_transmit(1, 'R');
		i2c_write (1, 0x78, data);
		Delay_Sys_MS (200);
		
		}
	


}