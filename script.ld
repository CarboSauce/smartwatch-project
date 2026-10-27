/* Linker script for STM32L151VE, Rust + cortex-m-rt 0.7
 * Use INSTEAD of -Tlink.x (and without memory.x).
 * 512 KB Flash, 80 KB SRAM, 16 KB data EEPROM.
 */

MEMORY
{
  FLASH  (rx)  : ORIGIN = 0x08000000, LENGTH = 512K
  RAM    (xrw) : ORIGIN = 0x20000000, LENGTH = 80K
  EEPROM (rw)  : ORIGIN = 0x08080000, LENGTH = 16K
}

_stack_start = ORIGIN(RAM) + LENGTH(RAM);
_stack_size = 4K; /* Increase from default 2K */

ENTRY(Reset);
EXTERN(__RESET_VECTOR);
EXTERN(__EXCEPTIONS);
EXTERN(__INTERRUPTS);

/* Default exception handlers (can be overridden from Rust) */
PROVIDE(NonMaskableInt  = DefaultHandler);
PROVIDE(MemoryManagement = DefaultHandler);
PROVIDE(BusFault        = DefaultHandler);
PROVIDE(UsageFault      = DefaultHandler);
PROVIDE(SecureFault     = DefaultHandler);
PROVIDE(SVCall          = DefaultHandler);
PROVIDE(DebugMonitor    = DefaultHandler);
PROVIDE(PendSV          = DefaultHandler);
PROVIDE(SysTick         = DefaultHandler);

PROVIDE(DefaultHandler  = DefaultHandler_);
PROVIDE(HardFault       = HardFault_);
PROVIDE(__pre_init      = DefaultPreInit);

/* Device interrupt handlers, provided by the PAC (stm32l1 / stm32l1xx-hal "rt" feature) */
INCLUDE device.x

SECTIONS
{
  .vector_table ORIGIN(FLASH) :
  {
    __vector_table = .;
    LONG(_stack_start);
    KEEP(*(.vector_table.reset_vector));
    __reset_vector = .;
    KEEP(*(.vector_table.exceptions));
    __eexceptions = .;
    KEEP(*(.vector_table.interrupts));
  } > FLASH

  .text :
  {
    *(.Reset);
    *(.text .text.*);
    *(.HardFaultTrampoline);
    *(.HardFault.*);
    . = ALIGN(4);
  } > FLASH

  .rodata : ALIGN(4)
  {
    *(.rodata .rodata.*);
    . = ALIGN(4);
  } > FLASH

  .data : ALIGN(4)
  {
    . = ALIGN(4);
    __sdata = .;
    *(.data .data.*);
    . = ALIGN(4);
    __edata = .;
  } > RAM AT > FLASH
  __sidata = LOADADDR(.data);

  .bss (NOLOAD) : ALIGN(4)
  {
    . = ALIGN(4);
    __sbss = .;
    *(.bss .bss.*);
    *(COMMON);
    . = ALIGN(4);
    __ebss = .;
  } > RAM

  .uninit (NOLOAD) : ALIGN(4)
  {
    . = ALIGN(4);
    __suninit = .;
    *(.uninit .uninit.*);
    . = ALIGN(4);
    __euninit = .;
  } > RAM

  /* Data EEPROM: #[link_section = ".eeprom"] */
  .eeprom (NOLOAD) : ALIGN(4)
  {
    *(.eeprom .eeprom.*);
  } > EEPROM

  /* Heap starts here if you use an allocator (e.g. embedded-alloc) */
  . = ALIGN(4);
  __sheap = .;

  /DISCARD/ :
  {
    *(.ARM.exidx);
    *(.ARM.exidx.*);
    *(.ARM.extab.*);
  }
}

ASSERT(ORIGIN(FLASH) % 4 == 0, "FLASH origin must be 4-byte aligned");
ASSERT(__reset_vector == ADDR(.vector_table) + 0x8, "reset vector is not at offset 8");
ASSERT(__eexceptions == ADDR(.vector_table) + 0x40, "exception vectors have the wrong size");

