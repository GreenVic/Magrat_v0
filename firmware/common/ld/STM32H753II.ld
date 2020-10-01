/* Label for the program's entry point */
ENTRY(reset_handler)

/* End of RAM/Start of stack. Use the SRAM4 bank. */
_estack = 0x38010000;

MEMORY
{
    FLASH    ( rx )    : ORIGIN = 0x08000000, LENGTH = 1M
    FLASH_B2 ( rx )    : ORIGIN = 0x08100000, LENGTH = 1M
    RAM_BKP  ( rxw )   : ORIGIN = 0x38800000, LENGTH = 4K
    SRAM4    ( rxw )   : ORIGIN = 0x38000000, LENGTH = 64K
    SRAM3    ( rxw )   : ORIGIN = 0x30040000, LENGTH = 32K
    SRAM2    ( rxw )   : ORIGIN = 0x30020000, LENGTH = 128K
    SRAM1    ( rxw )   : ORIGIN = 0x30000000, LENGTH = 128K
    RAM      ( rxw )   : ORIGIN = 0x24000000, LENGTH = 512K
    DTCMRAM  ( rxw )   : ORIGIN = 0x20000000, LENGTH = 128K
    ITCMRAM  ( rxw )   : ORIGIN = 0x00000000, LENGTH = 64K
}

INCLUDE "sections.ld"
