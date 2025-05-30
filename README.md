# Bare_Metal_Drivers_For_ARM_Cortex_M3_Processor
Drivers for STM32f103 microcontroller series. No IDE needed. Developed with arm-none-eabi toolchain, startup code, makefile, linkerscript.

## Folder structure: 
				|----> Drivers : All the header file and c files for the drivers.
				|
				|----> Src : main.c, startup.c, config.h
				|
				|----> build : Output files including final elf, object files etc
				|
				|----> Makefile 
				|
				|----> flash.gdb
				|
				|----> stm32f103ls.ld : Linker script
				

## List of API's: 
	### -- Clock configuration (HSE, HSI, PLL)
	### -- Systic Timer, Systic interrupt
	### -- GPIO
	### -- GPIO EXTI
	### -- Timers, Timer interrupt, PWM, Output compare
	### -- USART Transmit, receive, interrupt in simplex/duplex mode
	### -- I2C Read, write, interrupt
	### -- SPI Transmit, Receive, interrupt
	### -- ADC and ADC interrupt
	### -- 16X02 LCD display
	### -- SSD1306 OLED
	### -- RTC
	### -- Low-power
 ### NB: For detailed description of each API, please read the instruction above the driver files.  for example: Drivers/stm32f103GPIO.c file

## how to build and flash: 
	### -- navigate to the root folder
	### -- open terminal (git-bash preferred)
	### -- To clean previous build: make clean
	### -- to compile and build: make all
	### -- To load the openocd : make load
	### -- To flash, open another terminal and the command is: make flash


##Targeted_Hardware: Any STM32f103 series microcontroller


##Acknowledgements: 
	-- WeeW - Stack
	-- Fastbit Embedded Brain Academy
	-- Bare-Metal Embedded C Programming by Israel Gbati

