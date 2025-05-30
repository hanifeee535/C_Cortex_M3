
/* ####################################
Author: MD. Soyabbir Abu Hanif
 MSc in Embedded Systems
Tampere University, Finland
Email: hanifseceee535@gmail.com 
######################################
*/


#include <stdint.h>
#include "stm32f103Driver.h"


void sleep_on_exit(void){
    SCB->SCR |= (1<<1);
}




  void enable_wfi(void) {
    __asm volatile ("wfi");
}


 void enable_wfe(void) {
    __asm volatile ("wfi");
}
