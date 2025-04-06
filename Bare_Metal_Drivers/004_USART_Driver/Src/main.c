#include <stdint.h>
#include "../Drivers/stm32f103Driver.h"

 

// void USART3_IRQHandler  (void){
// 	char message = 'c';
// 	message = USART_receive (2);
// 	USART_transmit (2, message);
// }


int main ()
{
	
	//init_usart_receive_interrupt (2, 115200);
	init_USART(2, 9600);
	
	while (1){
		USART_transmit(2, 'c');
		
		}
	


}