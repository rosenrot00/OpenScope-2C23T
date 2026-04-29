ENTRY(Reset_Handler)

MEMORY
{
  FLASH (rx)  : ORIGIN = 0x08008000, LENGTH = 224K
  RAM   (rwx) : ORIGIN = 0x20000000, LENGTH = 0x36078
}

SECTIONS
{
  .isr_vector ORIGIN(FLASH) :
  {
    KEEP(*(.isr_vector))
  } > FLASH

  .text :
  {
    *(.text.Reset_Handler)
    *(.text*)
    *(.rodata*)
  } > FLASH

  /DISCARD/ :
  {
    *(.ARM.exidx*)
    *(.ARM.extab*)
  }

  .data :
  {
    __data_load = LOADADDR(.data);
    __data_start = .;
    *(.data*)
    __data_end = .;
  } > RAM AT > FLASH

  .bss (NOLOAD) :
  {
    __bss_start = .;
    *(.bss*)
    *(COMMON)
    __bss_end = .;
  } > RAM

  .noinit (NOLOAD) :
  {
    . = ALIGN(4);
    *(.noinit*)
    . = ALIGN(4);
  } > RAM

  . = ALIGN(8);
  __stack_top = ORIGIN(RAM) + LENGTH(RAM);
}
