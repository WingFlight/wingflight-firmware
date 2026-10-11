/*
*****************************************************************************
**
**  File        : stm32_flash_h730_xip.ld
**
**  Abstract    : Linker script for STM32H730xB Device executing in place
**                from the OCTOSPI1 NOR flash
**                320K AXI-RAM mapped onto AXI bus on D1 domain
**                 16K SRAM1 mapped on D2 domain
**                 16K SRAM2 mapped on D2 domain
**                 16K SRAM4 mapped on D3 domain
**                 64K ITCM
**                128K DTCM
**
*****************************************************************************
*/

/* Entry Point */
ENTRY(Reset_Handler)

/*
0x00000000 to 0x0000FFFF   64K ITCM
0x20000000 to 0x2001FFFF  128K DTCM, main RAM
0x24000000 to 0x2404FFFF  320K AXI SRAM, D1 domain
0x30000000 to 0x30003FFF   16K SRAM1, D2 domain
0x30004000 to 0x30007FFF   16K SRAM2, D2 domain
0x38000000 to 0x38003FFF   16K SRAM4, D3 domain, unused
0x38800000 to 0x38800FFF    4K BACKUP SRAM, Backup domain, unused

0x08000000 to 0x0801FFFF  128K internal flash, holds the boot loader
0x90000000 to 0x901FFFFF    2M OCTOSPI1 NOR flash, memory mapped by the boot loader
*/

/*

The internal flash only holds the boot loader. It puts the OCTOSPI1 flash into memory mapped mode and jumps to
the vector table at the start of the image, from where the firmware executes in place. The boot loader leaves
the clocks feeding the OCTOSPI1 running, so the firmware must not reset or reconfigure them.

The configuration is not stored in the OCTOSPI1 flash, the firmware cannot write to it while executing from it.

*/

/* Specify the memory areas */
MEMORY
{
    ITCM_RAM (rwx)    : ORIGIN = 0x00000000, LENGTH = 64K
    DTCM_RAM (rwx)    : ORIGIN = 0x20000000, LENGTH = 128K
    RAM (rwx)         : ORIGIN = 0x24000000, LENGTH = 128K + 192K /* 128K AXI SRAM + 192K ITCM & AXI = 320K */

    D2_RAM (rwx)      : ORIGIN = 0x30000000, LENGTH = 32K /* SRAM1 16K + SRAM2 16K */
    D3_RAM (rwx)      : ORIGIN = 0x38000000, LENGTH = 16K /* SRAM4 16K */

    BACKUP_SRAM (rwx) : ORIGIN = 0x38800000, LENGTH = 4K

    MEMORY_B1 (rx)    : ORIGIN = 0x60000000, LENGTH = 0K

    OCTOSPI2 (rx)     : ORIGIN = 0x70000000, LENGTH = 256M
    OCTOSPI1 (rx)     : ORIGIN = 0x90000000, LENGTH = 256M

    /* The firmware image, laid out like the start of the internal flash on the other H7 targets */
    OCTOSPI1_VECTAB (rx)          : ORIGIN = ORIGIN(OCTOSPI1), LENGTH = 10K
    OCTOSPI1_CUSTOM_DEFAULTS (r)  : ORIGIN = ORIGIN(OCTOSPI1) + 10K, LENGTH = 54K
    OCTOSPI1_CODE (rx)            : ORIGIN = ORIGIN(OCTOSPI1) + 64K, LENGTH = 2M - 64K
}

REGION_ALIAS("STACKRAM", DTCM_RAM)
REGION_ALIAS("FASTRAM", DTCM_RAM)
REGION_ALIAS("MAIN", OCTOSPI1_CODE)

REGION_ALIAS("VECTAB", OCTOSPI1_VECTAB)

INCLUDE "stm32_h730_common.ld"

SECTIONS
{
  /* used during startup to initialized dmaram_data */
  _sdmaram_idata = LOADADDR(.dmaram_data);

  . = ALIGN(32);
  .dmaram_data :
  {
    PROVIDE(dmaram_start = .);
    _sdmaram = .;
    _dmaram_start__ = _sdmaram;
    _sdmaram_data = .;        /* create a global symbol at data start */
    *(.dmaram_data)           /* .data sections */
    *(.dmaram_data*)          /* .data* sections */
    . = ALIGN(32);
    _edmaram_data = .;        /* define a global symbol at data end */
  } >RAM AT >MAIN

  . = ALIGN(32);
  .dmaram_bss (NOLOAD) :
  {
    _sdmaram_bss = .;
    __dmaram_bss_start__ = _sdmaram_bss;
    *(.dmaram_bss)
    *(SORT_BY_ALIGNMENT(.dmaram_bss*))
    . = ALIGN(32);
    _edmaram_bss = .;
    __dmaram_bss_end__ = _edmaram_bss;
  } >RAM

  . = ALIGN(32);
  .DMA_RAM (NOLOAD) :
  {
    KEEP(*(.DMA_RAM))
    PROVIDE(dmaram_end = .);
    _edmaram = .;
    _dmaram_end__ = _edmaram;
  } >RAM

  .DMA_RW_D2 (NOLOAD) :
  {
    . = ALIGN(32);
    PROVIDE(dmarw_start = .);
    _sdmarw = .;
    _dmarw_start__ = _sdmarw;
    KEEP(*(.DMA_RW))
    PROVIDE(dmarw_end = .);
    _edmarw = .;
    _dmarw_end__ = _edmarw;
  } >D2_RAM

  .DMA_RW_AXI (NOLOAD) :
  {
    . = ALIGN(32);
    PROVIDE(dmarwaxi_start = .);
    _sdmarwaxi = .;
    _dmarwaxi_start__ = _sdmarwaxi;
    KEEP(*(.DMA_RW_AXI))
    PROVIDE(dmarwaxi_end = .);
    _edmarwaxi = .;
    _dmarwaxi_end__ = _edmarwaxi;
  } >RAM

  /* Storage for the address for the configuration section so we can grab it out of the hex file */
  .custom_defaults :
  {
    . = ALIGN(4);
    KEEP (*(.custom_defaults_start_address))
    . = ALIGN(4);
    KEEP (*(.custom_defaults_end_address))
    . = ALIGN(4);
    __custom_defaults_internal_start = .;
    *(.custom_defaults);
  } >OCTOSPI1_CUSTOM_DEFAULTS

  PROVIDE_HIDDEN (__custom_defaults_start = __custom_defaults_internal_start);
  PROVIDE_HIDDEN (__custom_defaults_end = ORIGIN(OCTOSPI1_CUSTOM_DEFAULTS) + LENGTH(OCTOSPI1_CUSTOM_DEFAULTS));
}

INCLUDE "stm32_h730_common_post.ld"
