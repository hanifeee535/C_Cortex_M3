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
	LCD1602_init ();
	Delay_Sys_MS(10);
	//LCD1602_print (1, 2, "hello world");
	
	
	
	while (1){	
	//LCD1602_marquee_text("hello world from me", 1, 200);
	 LCD1602_scroll_from_right("Hello good people of Bangladesh", 2, 200);
		LCD1602_scroll_from_left("This is scrolling from left to right", 1, 200);
	}	


}



