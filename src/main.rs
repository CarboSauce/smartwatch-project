#![no_std]
#![no_main]

use cortex_m_rt::entry;
use panic_halt as _;
use stm32l1xx_hal::{hal::digital::v2::OutputPin, prelude::*, stm32};

#[entry]
fn main() -> ! {
    // Enable the GPIOB clock (RCC_AHBENR bit 1) in case the HAL doesn't
    unsafe {
        let ahbenr = 0x4002_381C as *mut u32;
        core::ptr::write_volatile(ahbenr, core::ptr::read_volatile(ahbenr) | (1 << 1));
    }

    let dp = stm32::Peripherals::take().unwrap();
    let gpiob = dp.GPIOB.split();
    let mut led = gpiob.pb0.into_push_pull_output();

    loop {
        led.set_high().unwrap();
        cortex_m::asm::delay(300_000);
        led.set_low().unwrap();
        cortex_m::asm::delay(300_000);
        led.set_high().unwrap();
        cortex_m::asm::delay(600_000);
        led.set_low().unwrap();
        cortex_m::asm::delay(600_000);
    }
}
