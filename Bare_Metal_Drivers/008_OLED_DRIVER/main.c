/* ####################################
Author: MD. Soyabbir Abu Hanif
 MSc in Embedded Systems
Tampere University, Finland
Email: hanifseceee535@gmail.com 
######################################
*/



#include <stdint.h>    
#include "header.h"

 

int main ()
{
	systic_init();
	oled_configure_64 (1);
	oled_blank(1);
	oled_send_message(1, 3,40, "HELLO WORLD");
	oled_send_message(1, 4,50, "WELCOME");
	
	
	
	while (1){
		
		
		
		
		}
	


}



