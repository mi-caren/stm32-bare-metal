ENTRY(_reset);

MEMORY
{
    flash (rx)  : ORIGIN = 0x08000000, LENGTH = 256k
    sram1 (rwx) : ORIGIN = 0x20000000, LENGTH = 32k
}

_estack = ORIGIN(sram1) + LENGTH(sram1);

SECTIONS
{
    .vectors : {
        KEEP(*(.vectors))
    } > flash

    .text : {
        *(.text)
        *(.text*)
    } > flash

    .rodata : {
        *(.rodata*)
    } > flash

    .data : {
        _data_start = .;
        *(.data)
        *(.data*)
        _data_end = .;
    } > sram1 AT > flash

    _data_loadaddr = LOADADDR(.data);

    .bss : {
        _bss_start = .;
        *(.bss SORT(.bss) COMMON)
        _bss_end = .;
    } > sram1

    . = ALIGN(8);
    _end = .;
}
