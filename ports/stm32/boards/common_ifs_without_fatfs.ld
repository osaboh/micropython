/* Memory layout for internal flash storage configuration:

    FLASH_ISR   .isr_vector

    FLASH_TEXT  .text
    FLASH_TEXT  .data

    RAM         .data
    RAM         .bss
    RAM         .heap
    RAM         .stack
*/

ENTRY(Reset_Handler)

/* define output sections */
SECTIONS
{
    /* The startup code goes first into FLASH */
    .isr_vector :
    {
        . = ALIGN(4);
        KEEP(*(.isr_vector)) /* Startup code */

        /* This first flash block is 16K annd the isr vectors only take up
           about 400 bytes. So we pull in a couple of object files to pad it
           out. */

        . = ALIGN(4);

        /* NOTE: If you update the list of files contained in .isr_vector,
                 then be sure to also update smhal/Makefile where it forcibly
           builds each of these files with -Os */

        /* *\/ff.o(.text*) */
        /* *\/vfs_fat_*.o(.text*) */
        */py/formatfloat.o(.text*)
        */py/parsenum.o(.text*)
        */py/mpprint.o(.text*)

        . = ALIGN(4);
    } >FLASH_ISR

    /* The program code and other data goes into FLASH */
    .text :
    {
        . = ALIGN(4);
        *(.text*)          /* .text* sections (code) */
        *(.rodata*)        /* .rodata* sections (constants, strings, etc.) */
    /*  *(.glue_7)   */    /* glue arm to thumb code */
    /*  *(.glue_7t)  */    /* glue thumb to arm code */

        . = ALIGN(4);
        _etext = .;        /* define a global symbol at end of code */
    } >FLASH_TEXT

    /* used by the startup to initialize data */
    _sidata = LOADADDR(.data);

    /* This is the initialized data section
    The program executes knowing that the data is in the RAM
    but the loader puts the initial values in the FLASH (inidata).
    It is one task of the startup to copy the initial values from FLASH to RAM. */
    .data :
    {
        . = ALIGN(4);
        _sdata = .;        /* create a global symbol at data start; used by startup code in order to initialise the .data section in RAM */
        *(.data*)          /* .data* sections */

        . = ALIGN(4);
        _edata = .;        /* define a global symbol at data end; used by startup code in order to initialise the .data section in RAM */
    } >RAM AT> FLASH_TEXT

    /* Uninitialized data section */
    .bss :
    {
        . = ALIGN(4);
        _sbss = .;         /* define a global symbol at bss start; used by startup code */
        *(.bss*)
        *(COMMON)

        . = ALIGN(4);
        _ebss = .;         /* define a global symbol at bss end; used by startup code and GC */
    } >RAM

    /* this is to define the start of the heap, and make sure we have a minimum size */
    .heap :
    {
        . = ALIGN(4);
        . = . + _minimum_heap_size;
        . = ALIGN(4);
    } >RAM

    /* this just checks there is enough RAM for the stack */
    .stack :
    {
        . = ALIGN(4);
        . = . + _minimum_stack_size;
        . = ALIGN(4);
    } >RAM

    .ARM.attributes 0 : { *(.ARM.attributes) }
}
