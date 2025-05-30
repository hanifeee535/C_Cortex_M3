# Target executable name
TARGET     = final

# Build output directory
BUILD_DIR  = build

# Compiler and toolchain
PREFIX     = arm-none-eabi
CC         = $(PREFIX)-gcc

# Compiler and linker flags
CFLAGS     = -c -mcpu=cortex-m3 -mthumb -std=gnu11 -Wall -Wextra -Werror -Os \
             -fno-builtin-memcpy -fno-builtin-memset -fno-tree-loop-distribute-patterns \
             -mfloat-abi=soft

LDFLAGS    = -nostdlib -T stm32f103ls.ld -Wl,--gc-sections -mcpu=cortex-m3 -mthumb -mfloat-abi=soft
LDLIBS     = -lgcc

# Source files
SRC        = $(wildcard Src/*.c) $(wildcard Drivers/*.c)
OBJ        = $(patsubst %.c, $(BUILD_DIR)/%.o, $(subst Src/,Src_,$(subst Drivers/,Drivers_,$(SRC))))

# Default target
all: $(BUILD_DIR)/$(TARGET).elf

# Compile .c files to .o
$(BUILD_DIR)/Src_%.o: Src/%.c
	@mkdir -p $(BUILD_DIR)
	$(CC) $(CFLAGS) $< -o $@

$(BUILD_DIR)/Drivers_%.o: Drivers/%.c
	@mkdir -p $(BUILD_DIR)
	$(CC) $(CFLAGS) $< -o $@

# Link final ELF
$(BUILD_DIR)/$(TARGET).elf: $(OBJ)
	$(CC) $(LDFLAGS) -o $@ $^ $(LDLIBS)

# Clean build files
clean:
	rm -rf $(BUILD_DIR)/*.o $(BUILD_DIR)/$(TARGET).elf

# Load/flash (optional targets)
load:
	openocd -f board/st_nucleo_f103rb.cfg

flash:
	arm-none-eabi-gdb -x flash.gdb
