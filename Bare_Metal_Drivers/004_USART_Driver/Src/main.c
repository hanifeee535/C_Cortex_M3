#include <stdint.h>
#include "../Drivers/stm32f103Driver.h"

 

void USART1_IRQHandler  (void){
	char message = 'c';
	message = USART_receive (1);
	USART_transmit (1, message);
}


int main ()
{
	
	init_usart_receive_interrupt (1, 115200);
	//init_USART(1, 9600);
	
	while (1){
		//USART_transmit(1, 'R');
		
		}
	


}