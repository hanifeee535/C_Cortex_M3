# Compiler and flags
CC = arm-none-eabi-gcc
CFLAGS = -c -mcpu=cortex-m3 -mthumb -std=gnu11 
LDFLAGS = -nostdlib -T stm32f103ls.ld
all: main.o gpiodrive.o stm32f103Startup.o blinking.elf


main.o:main.c 
	$(CC) $(CFLAGS) $^ -o $@
gpiodrive.o : Drivers/stm32f103GPIODrive.c
	$(CC) $(CFLAGS) $^ -o $@
startup.o : stm32f103Startup.c 
	$(CC) $(CFLAGS) $^ -o $@
blinking.elf: main.o gpiodrive.o startup.o 
	$(CC) $(LDFLAGS) -o $@ $^
clean:
	rm -rf *.o *.elf

load:
	openocd -f board/st_nucleo_f103rb.cfg