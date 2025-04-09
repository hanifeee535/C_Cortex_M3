#include <stdint.h>
#include "../Drivers/stm32f103Driver.h"

 
char adc_data[10];
//char data [2] = {0x01, 0x02};
int main ()
{	
	systic_init();
	//i2c_init(1,i2c_FastMode);
	init_USART(1, 9600);
	adc_init(1, portC, 0);
	//init_SPI(2, 128);
	
	
	while (1){
		
		if(adc_data_ready(1)){
			uint16_t adcvalue = adc_read(1);
			int_to_str(adcvalue, adc_data);
			USART_send_string(1, adc_data);
			USART_transmit(1, '\n');
		}
		//USART_transmit(1, 'r');
		
		}
}