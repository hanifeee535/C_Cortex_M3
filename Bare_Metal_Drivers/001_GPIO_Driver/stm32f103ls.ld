/**
******************************************************************************
** STM32F103xx Linker Script
** - 128KB Flash (0x08000000 - 0x0801FFFF)
** - 20KB SRAM (0x20000000 - 0x20004FFF)
** - Stack: End of SRAM
** - Heap: Grows upwards from end of .bss
******************************************************************************
** MEMORY MAP:
** FLASH - Code, constants, vector table
** SRAM  - Data, BSS, Heap, Stack
******************************************************************************
** Created for STM32F103C8T6 (Medium-density)
** For other variants, the lenghth should be adjusted
******************************************************************************
*/

/* Entry Point - Required for boot process */
ENTRY(Reset_Handler)

/* Linker script for STM32F103 */
MEMORY
{
    FLASH (rx)  : ORIGIN = 0x08000000, LENGTH = 128K
    SRAM  (rwx) : ORIGIN = 0x20000000, LENGTH = 20K
}

/* Define stack top (end of SRAM) */
_estack = ORIGIN(SRAM) + LENGTH(SRAM);

SECTIONS
{
    /* Vector Table at start of Flash */
    .isr_vector :
    {
        . = ALIGN(4);
        KEEP(*(.isr_vector))
    } > FLASH

    /* Text (code) section */
    .text :
    {
        . = ALIGN(4);
        *(.text)       /* All code */
        *(.text*)      /* More code */
        *(.rodata)     /* Read-only data */
        *(.rodata*)    /* More read-only data */
        . = ALIGN(4);
        _etext = .;    /* End of code */
    } > FLASH

    /* Initialized data section */
    .data :
    {
        . = ALIGN(4);
        _sdata = .;      /* Start of .data in SRAM */
        *(.data)         /* Data section */
        *(.data*)        /* More data */
        . = ALIGN(4);
        _edata = .;      /* End of .data */
    } > SRAM AT> FLASH

    /* Load address of .data (in Flash) */
    _sidata = LOADADDR(.data);

    /* Uninitialized .bss section */
    .bss :
    {
        . = ALIGN(4);
        _sbss = .;       /* Start of .bss */
        *(.bss)          /* Uninitialized variables */
        *(.bss*)         /* More uninitialized variables */
        *(COMMON)        /* Common symbols */
        . = ALIGN(4);
        _ebss = .;       /* End of .bss */
    } > SRAM

    /* Stack region (at top of SRAM) */
    . = ALIGN(8);
    _end = .;
}
