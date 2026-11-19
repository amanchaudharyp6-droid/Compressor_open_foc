/*
******************************************************************************
**
**  File        : LinkerScript.ld
**
**  Author      : STM32CubeIDE
**
**  Abstract    : Linker script for STM32H7 series
**                2048Kbytes FLASH and 1056Kbytes RAM
**
**                Set heap size, stack size and stack location according
**                to application requirements.
**
**                Set memory bank area and size if external memory is used.
**
**  Target      : STMicroelectronics STM32
**
**  Distribution: The file is distributed as is, without any warranty
**                of any kind.
********************************************************************************
*/
/*Seperate section for Configs and LUT?*/
/* Specify the memory areas */
MEMORY
{
    Bootloader (rw)   : ORIGIN = 0x08000000, LENGTH = 100K
    App1       (rx)   : ORIGIN = 0x08040000, LENGTH = 256K
    App2       (rx)   : ORIGIN = 0x08080000, LENGTH = 256K
    CommonData (rwx)  : ORIGIN = 0x080C0000, LENGTH = 40K

    SharedRAM  (rwx)  : ORIGIN = 0x20000000, LENGTH = 10K
    DTCMRAM    (xrw)  : ORIGIN = 0x20002800, LENGTH = 128K - LENGTH(SharedRAM)
    RAM_D1     (xrw)  : ORIGIN = 0x24000000, LENGTH = 512K 
    RAM_D2     (xrw)  : ORIGIN = 0x30000000, LENGTH = 288K
    RAM_D3     (xrw)  : ORIGIN = 0x38000000, LENGTH = 64K
    ITCMRAM    (xrw)  : ORIGIN = 0X00000000, LENGTH = 64K
}

/* Define output sections */

/*Shared Memory includes all the flags shared between the bootloader and the apps*/
SECTIONS
{
    .shared_memory (NOLOAD) : {
        KEEP(*(.shared_memory))
    } >SharedRAM
}

/*Common Data section currently unused*/
SECTIONS
{
    .common_data (NOLOAD) : {
        KEEP(*(.common_data))
    } >CommonData
}

__SharedRAM_Start__  = ORIGIN(SharedRAM);
__SharedRAM_Size__   = LENGTH(SharedRAM);

__Bootloader_Start__ = ORIGIN(Bootloader);
__Bootloader_Size__  = LENGTH(Bootloader);

__App1_Start__ = ORIGIN(App1);
__App1_Size__  = LENGTH(App1);

__App2_Start__ = ORIGIN(App2);
__App2_Size__  = LENGTH(App2);

__CommonData_Start__ = ORIGIN(CommonData);


