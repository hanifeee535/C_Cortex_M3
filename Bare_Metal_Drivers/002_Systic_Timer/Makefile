# Target executable name
TARGET = final
# Build output directory
BUILD_DIR = build


# Compiler and toolchain
PREFIX  = arm-none-eabi
CC      = $(PREFIX)-gcc
CFLAGS  = -c -mcpu=cortex-m3 -mthumb -std=gnu11 -Wall -Wextra -Werror -Os -fno-builtin-memcpy -fno-builtin-memset -fno-tree-loop-distribute-patterns          
LDFLAGS = -nostdlib -T stm32f103ls.ld -Wl,--gc-sections



all: $(BUILD_DIR)/main.o $(BUILD_DIR)/drive.o $(BUILD_DIR)/startup.o  $(BUILD_DIR)/final.elf


$(BUILD_DIR)/main.o:Src/main.c
	$(CC) $(CFLAGS) $^ -o $@
$(BUILD_DIR)/drive.o : Drivers/stm32f103SysticDrive.c
	$(CC) $(CFLAGS) $^ -o $@
$(BUILD_DIR)/startup.o : Src/stm32f103Startup.c 
	$(CC) $(CFLAGS) $^ -o $@
$(BUILD_DIR)/$(TARGET).elf: $(BUILD_DIR)/main.o $(BUILD_DIR)/drive.o $(BUILD_DIR)/startup.o 
	$(CC) $(LDFLAGS) -o $@ $^
clean:
	rm -rf build/*.o *.elf

load:
	openocd -f board/st_nucleo_f103rb.cfg

flash:
	arm-none-eabi-gdb -x flash.gdb
