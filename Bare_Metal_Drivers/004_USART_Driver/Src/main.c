#include <stdint.h>
#include "../Drivers/stm32f103Driver.h"

 

void USART3_IRQHandler  (void){
	char message = 'c';
	message = USART_receive (3);
	USART_transmit (3, message);
}


int main ()
{
	
	init_usart_receive_interrupt (3, 115200);
	
	
	while (1){
		
		}
	


}