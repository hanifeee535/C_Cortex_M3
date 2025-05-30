/* ####################################
Author: MD. Soyabbir Abu Hanif
 MSc in Embedded Systems
Tampere University, Finland
Email: hanifseceee535@gmail.com 
######################################
*/




/*
************************************************************************************************
*************************__ADC_Data_Processing_Description__*************************
*************************************************************************************************

by default right aligned data
which means bit 15 to 12 is zero and bit 11 to 0 will held the actual data
adc has 12 bit resolution. so the value will be 0 to 4095
By default the adc data is 12 bit unsigned integer stored in the 
lower 12 bits of ADC_DR

voltage conversion:

Analog voltage = (ADC_value X Vref)/4095
gere vref is typically the supply voltage unless an external reference is used

So we can se that to convert the data into voltage, we have to do floating point 
calculation. As cortex m3 mcu does not have any dedicated floating point hardware
the mcu will be slower when it has to calculate the floating points.
So there may have several solution for processing the data. We can use 
them according to our needs:
1. floating point: float voltage = (adc_value * 3.3f) / 4095;  
2. Integer scalling to millivolts: uint16_t voltage_mV = (adc_value * 3300) / 4095;  
3. Fixed point: 
		// Fixed-point scaling: 3300/4095 ≈ 0.80586 ≈ 52812 in 16.16 fixed-point
		#define SCALE_FACTOR 52812 // (0.80586 * 65536) ≈ 52812
		uint16_t adc_value = ...;
		uint32_t voltage_mV = (adc_value * SCALE_FACTOR) >> 16; // Shift right 16 bits


**floating point calculation gives exact accuracy, it takes around 100 to 200 cycles to calculate data. Ram usage is low
** Integer scalling and division has also exact accurate value. It takes 50 to 100 clock cycle because of division. It's ram usage is low
** fixed point and bit shifting has data error of around 0.1%. But it takes 5 to 10 clock cycles. So it is faster. 
We can use different method according to our case. 




*/


#include <stdint.h>
#include "stm32f103Driver.h"
#include "../Src/config.h"

uint8_t get_adc_channel (uint8_t port, uint8_t pin ){
	uint8_t channel = -1; 
	if (port == portA){
		if(pin <= 7){  //PA0 to PA7
			channel = pin; //ADC_IN0 to ADC_IN7
		}
		else {return -1;}
	}

	else if (port == portB){
		if(pin <= 1){  //PB0 to PB1
			channel = 8+pin; //ADC_IN8 to ADC_IN9
		}
		else {return -1;}
	}

	else if (port == portC){
		if(pin <= 5){  //PC0 to PC5
			channel = 10+pin; //ADC_IN10 to ADC_IN15
		}
		else {return -1;}
	}

	return channel;


}

ADC_TypeDef* select_ADC (uint8_t adc){
	if(adc == 1){
		return ADC1;
	}
	else if (adc == 2){
		return ADC2;
	}
	else {
		return 0;
	}
}

void adc_init (uint8_t adc, uint8_t port, uint8_t pin){
	Config_GPIO(port, pin, input, analog_in ); //configuring pin for analog input
	ADC_TypeDef *adcx = select_ADC(adc);
	uint8_t channel = get_adc_channel(port, pin);
	if (adc == 1){
		RCC->APB2ENR |= (1<<9);
	}
	else if (adc == 2){
		RCC->APB2ENR |= (1<<10);
	}

	adcx->CR2 = 0x0; //reset the adc control register
	adcx->SQR3= channel;
	adcx->CR2 |= 1; //Adc on
	delay_miliSecond(3, 100 ,system_freq); //wait for sometime because Conversion starts when this bit holds a value of 1 and a 1 is written to it
	adcx->CR2 |= 1; //Adc on
	adcx->CR2 |= 2; //Adc continous mode
}


//Checking the ADC data ready flag
uint8_t adc_data_ready(uint8_t adc){
	uint8_t ready = 0; 
	if (adc == 1){
		if (ADC1->SR & 2){
			ready = 1;
		}
		else {ready = 0;}
	}

	else if (adc == 2){
		if (ADC2->SR & 2){
			ready = 1;
		}
		else {ready = 0;}
	}

	else {ready = 0;}

	return ready;
	
}

//reading ADC value 
uint16_t adc_read (uint8_t adc){
	uint16_t adc_data ;
	if (adc == 1){
		adc_data = ADC1->DR;
		
	}
	else if (adc == 2){
		adc_data = ADC2->DR;
	}
	else {adc_data= 0;}
	return adc_data;

}



