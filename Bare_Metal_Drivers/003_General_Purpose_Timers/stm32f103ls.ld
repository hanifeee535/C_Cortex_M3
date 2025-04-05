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

MEMORY
{
  FLASH(rx):ORIGIN =0x08000000,LENGTH =128K
  SRAM(rwx):ORIGIN =0x20000000,LENGTH =20K
}

SECTIONS
{

.isr_vector :
  {
    KEEP(*(.isr_vector));
    . = ALIGN(4);
  }> FLASH

  .text :
  {
    *(.text)
    *(.text.*)
    *(.init)
    *(.fini)
    *(.rodata)
    *(.rodata.*)
    . = ALIGN(4);
    _etext = .;
  }> FLASH

  _la_data = LOADADDR(.data);

  .data :
  {
    _sdata = .;
    *(.data)
    *(.data.*)
    . = ALIGN(4);
    _edata = .;
  }> SRAM AT> FLASH

  .bss :
  {
    _sbss = .;
    __bss_start__ = _sbss;
    *(.bss)
    *(.bss.*)
    *(COMMON)
    . = ALIGN(4);
    _ebss = .;
    __bss_end__ = _ebss;
     . = ALIGN(4);
    end = .;
    __end__ = .;
  }> SRAM
}