#![no_std]
#![no_main]

// STEVAL-WESU1: STM32L151 + BlueNRG-MS over (bit-banged) SPI.
// Makes the board advertise as "WESU1" so a phone can see and connect to it.
//
// Pins (from the board schematic):
//   SCK  = PA5   MISO = PA6   MOSI = PA7
//   RST  = PA9   CS   = PC3   IRQ  = PC13
//
// Progress is written to the STAGE variable (read it with probe-rs):
//   1 start, 2 BlueNRG reset event received, 3 address set, 4 GATT init ok,
//   5 GAP init ok, 6 advertising, 7 phone connected, 8 disconnected (re-advertising)
//   0xE1..0xE5 = error at step 1..5

use core::ptr::{read_volatile, write_volatile};
use core::sync::atomic::{AtomicU32, Ordering};
use cortex_m_rt::entry;
use panic_halt as _;

pub static STAGE: AtomicU32 = AtomicU32::new(0);

// ---------- GPIO (raw registers) ----------
const RCC_AHBENR: usize = 0x4002_381C;
const GPIOA: usize = 0x4002_0000;
const GPIOC: usize = 0x4002_0800;
const MODER: usize = 0x00;
const PUPDR: usize = 0x0C;
const IDR: usize = 0x10;
const BSRR: usize = 0x18;

type Pin = (usize, u32);
const SCK: Pin = (GPIOA, 5);
const MISO: Pin = (GPIOA, 6);
const MOSI: Pin = (GPIOA, 7);
const RST: Pin = (GPIOA, 9);
const CS: Pin = (GPIOC, 3);
const IRQ: Pin = (GPIOC, 13);

fn reg_modify(addr: usize, f: impl FnOnce(u32) -> u32) {
    unsafe {
        let p = addr as *mut u32;
        write_volatile(p, f(read_volatile(p)));
    }
}

fn set_mode(p: Pin, mode: u32) {
    let s = p.1 * 2;
    reg_modify(p.0 + MODER, |v| (v & !(3 << s)) | (mode << s));
}

fn set_pull(p: Pin, pull: u32) {
    let s = p.1 * 2;
    reg_modify(p.0 + PUPDR, |v| (v & !(3 << s)) | (pull << s));
}

fn write_pin(p: Pin, high: bool) {
    let bit = if high { 1 << p.1 } else { 1 << (p.1 + 16) };
    unsafe { write_volatile((p.0 + BSRR) as *mut u32, bit) };
}

fn read_pin(p: Pin) -> bool {
    unsafe { (read_volatile((p.0 + IDR) as *const u32) >> p.1) & 1 != 0 }
}

// Default clock after reset is MSI ~2.1 MHz
fn delay_ms(ms: u32) {
    cortex_m::asm::delay(ms * 2_100);
}

fn gpio_init() {
    // Clocks for GPIOA (bit 0) and GPIOC (bit 2)
    reg_modify(RCC_AHBENR, |v| v | (1 << 0) | (1 << 2));
    let _ = unsafe { read_volatile(RCC_AHBENR as *const u32) };

    // Set idle levels first, then make the pins outputs
    write_pin(CS, true);
    write_pin(SCK, false);
    write_pin(MOSI, false);
    write_pin(RST, true);
    for p in [CS, SCK, MOSI, RST] {
        set_mode(p, 1);
    }
    // Inputs: MISO floating, IRQ with pull-down
    set_mode(MISO, 0);
    set_mode(IRQ, 0);
    set_pull(IRQ, 2);
}

// ---------- SPI (mode 0, MSB first, bit-banged) ----------
fn spi_xfer(out: u8) -> u8 {
    let mut inp = 0u8;
    for i in (0..8).rev() {
        write_pin(MOSI, (out >> i) & 1 != 0);
        cortex_m::asm::delay(10);
        write_pin(SCK, true);
        cortex_m::asm::delay(10);
        inp = (inp << 1) | (read_pin(MISO) as u8);
        write_pin(SCK, false);
        cortex_m::asm::delay(10);
    }
    inp
}

// 5-byte BlueNRG SPI header: op = 0x0A (write) or 0x0B (read)
fn spi_header(op: u8) -> [u8; 5] {
    let tx = [op, 0, 0, 0, 0];
    let mut h = [0u8; 5];
    for i in 0..5 {
        h[i] = spi_xfer(tx[i]);
    }
    h
}

fn bnrg_write(data: &[u8]) -> bool {
    for _ in 0..200 {
        write_pin(CS, false);
        let h = spi_header(0x0A);
        // h[0] == 0x02: SPI ready, h[1]: free bytes in the write buffer
        if h[0] == 0x02 && (h[1] as usize) >= data.len() {
            for &b in data {
                spi_xfer(b);
            }
            write_pin(CS, true);
            return true;
        }
        write_pin(CS, true);
        delay_ms(1);
    }
    false
}

fn bnrg_read(buf: &mut [u8]) -> usize {
    write_pin(CS, false);
    let h = spi_header(0x0B);
    let mut n = 0;
    if h[0] == 0x02 {
        let avail = ((h[4] as usize) << 8) | (h[3] as usize);
        for i in 0..avail {
            let b = spi_xfer(0xFF);
            if i < buf.len() {
                buf[i] = b;
                n = i + 1;
            }
        }
    }
    write_pin(CS, true);
    n
}

fn wait_irq(timeout_ms: u32) -> bool {
    for _ in 0..timeout_ms {
        if read_pin(IRQ) {
            return true;
        }
        delay_ms(1);
    }
    read_pin(IRQ)
}

// ---------- HCI over SPI ----------
fn send_cmd(opcode: u16, params: &[u8]) -> bool {
    let mut pkt = [0u8; 64];
    pkt[0] = 0x01; // HCI command packet
    pkt[1] = opcode as u8;
    pkt[2] = (opcode >> 8) as u8;
    pkt[3] = params.len() as u8;
    pkt[4..4 + params.len()].copy_from_slice(params);
    bnrg_write(&pkt[..4 + params.len()])
}

// Waits for the command-complete (or command-status) event; returns the status byte
fn wait_status(opcode: u16) -> Option<u8> {
    let mut buf = [0u8; 64];
    for _ in 0..50 {
        if !wait_irq(200) {
            return None;
        }
        let n = bnrg_read(&mut buf);
        if n >= 7 && buf[0] == 0x04 {
            // 04 0E len numpkts op_lo op_hi status ...
            if buf[1] == 0x0E && buf[4] == opcode as u8 && buf[5] == (opcode >> 8) as u8 {
                return Some(buf[6]);
            }
            // 04 0F 04 status numpkts op_lo op_hi
            if buf[1] == 0x0F && buf[5] == opcode as u8 && buf[6] == (opcode >> 8) as u8 {
                return Some(buf[3]);
            }
        }
    }
    None
}

fn command(opcode: u16, params: &[u8]) -> bool {
    send_cmd(opcode, params) && wait_status(opcode) == Some(0)
}

const OP_WRITE_CONFIG_DATA: u16 = 0xFC0C;
const OP_GATT_INIT: u16 = 0xFD01;
const OP_GAP_INIT: u16 = 0xFC8A;
const OP_GAP_SET_DISCOVERABLE: u16 = 0xFC83;

fn start_advertising() -> bool {
    // Complete local name: AD type 0x09 followed by "WESU1"
    let name = [0x09u8, b'W', b'E', b'S', b'U', b'1'];
    let mut p = [0u8; 32];
    let mut i = 0;
    let mut put = |bytes: &[u8]| {
        p[i..i + bytes.len()].copy_from_slice(bytes);
        i += bytes.len();
    };
    put(&[0x00]); // ADV_IND: connectable undirected
    put(&[0xA0, 0x00]); // interval min: 100 ms
    put(&[0xF0, 0x00]); // interval max: 150 ms
    put(&[0x00]); // own address: public
    put(&[0x00]); // filter policy: allow all
    put(&[name.len() as u8]);
    put(&name);
    put(&[0x00]); // no service UUID list
    put(&[0x00, 0x00]); // slave connection interval min (none)
    put(&[0x00, 0x00]); // slave connection interval max (none)
    command(OP_GAP_SET_DISCOVERABLE, &p[..i])
}

#[entry]
fn main() -> ! {
    STAGE.store(1, Ordering::Relaxed);
    gpio_init();

    // Hardware reset of the BlueNRG-MS
    write_pin(RST, false);
    delay_ms(10);
    write_pin(RST, true);

    // After reset the chip signals a "blue initialized" event with IRQ high
    let mut buf = [0u8; 64];
    if !wait_irq(1000) {
        STAGE.store(0xE1, Ordering::Relaxed);
        loop {}
    }
    bnrg_read(&mut buf);
    STAGE.store(2, Ordering::Relaxed);

    // Public Bluetooth address (LSB first)
    let addr = [0x12u8, 0x34, 0x00, 0xE1, 0x80, 0x02];
    let mut cfg = [0u8; 8];
    cfg[0] = 0x00; // offset: public address
    cfg[1] = 0x06; // length
    cfg[2..8].copy_from_slice(&addr);
    if !command(OP_WRITE_CONFIG_DATA, &cfg) {
        STAGE.store(0xE2, Ordering::Relaxed);
        loop {}
    }
    STAGE.store(3, Ordering::Relaxed);

    if !command(OP_GATT_INIT, &[]) {
        STAGE.store(0xE3, Ordering::Relaxed);
        loop {}
    }
    STAGE.store(4, Ordering::Relaxed);

    // role = peripheral (0x01), privacy off, device name characteristic length 7
    if !command(OP_GAP_INIT, &[0x01, 0x00, 0x07]) {
        STAGE.store(0xE4, Ordering::Relaxed);
        loop {}
    }
    STAGE.store(5, Ordering::Relaxed);

    if !start_advertising() {
        STAGE.store(0xE5, Ordering::Relaxed);
        loop {}
    }
    STAGE.store(6, Ordering::Relaxed);

    // Keep servicing events so the chip's IRQ line is released
    loop {
        if read_pin(IRQ) {
            let n = bnrg_read(&mut buf);
            if n >= 4 && buf[0] == 0x04 {
                match buf[1] {
                    // LE meta event, subevent 1: connection complete
                    0x3E if buf[3] == 0x01 => STAGE.store(7, Ordering::Relaxed),
                    // Disconnection complete: advertise again
                    0x05 => {
                        STAGE.store(8, Ordering::Relaxed);
                        if start_advertising() {
                            STAGE.store(6, Ordering::Relaxed);
                        }
                    }
                    _ => {}
                }
            }
        }
    }
}
