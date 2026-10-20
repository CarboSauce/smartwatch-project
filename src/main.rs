#![no_std]
#![no_main]

// STEVAL-WESU1: STM32L151 + BlueNRG-MS over SPI1 + LSM6DS3 over I2C1
use core::sync::atomic::{AtomicU32, Ordering};

use embassy_executor::Spawner;
use embassy_stm32::exti::{self, ExtiInput};
use embassy_stm32::gpio::{Level, Output, Pull, Speed};
use embassy_stm32::i2c::{self, I2c};
use embassy_stm32::mode::{Async, Blocking};
use embassy_stm32::spi::{self, mode::Master, Spi};
use embassy_stm32::time::Hertz;
use embassy_stm32::{bind_interrupts, interrupt};
use embassy_time::{with_timeout, Duration, Timer};
use panic_halt as _;

bind_interrupts!(struct Irqs {
    // PC13 -> EXTI13 -> shared EXTI15_10 interrupt
    EXTI15_10 => exti::InterruptHandler<interrupt::typelevel::EXTI15_10>;
});

pub static STAGE: AtomicU32 = AtomicU32::new(0);
pub static PAIR_STATUS: AtomicU32 = AtomicU32::new(0xFF);
pub static LAST_VENDOR_EVT: AtomicU32 = AtomicU32::new(0);

// ---- Sensor debug values (read them with probe-rs) ----
/// 7-bit I2C address that answered a WHO_AM_I read. 0 = nothing answered.
pub static I2C_ADDR_FOUND: AtomicU32 = AtomicU32::new(0);
/// WHO_AM_I value. LSM6DS3 should give 0x69 (LSM6DS3TR-C gives 0x6A). 0xFFFF = never read.
pub static I2C_WHOAMI: AtomicU32 = AtomicU32::new(0xFFFF);
/// Number of successful / failed accelerometer reads.
pub static ACCEL_READS_OK: AtomicU32 = AtomicU32::new(0);
pub static ACCEL_READS_ERR: AtomicU32 = AtomicU32::new(0);
/// Last raw accelerometer values (signed 16-bit, stored sign-extended).
pub static ACCEL_X: AtomicU32 = AtomicU32::new(0);
pub static ACCEL_Y: AtomicU32 = AtomicU32::new(0);
pub static ACCEL_Z: AtomicU32 = AtomicU32::new(0);
/// Number of times a notification/value update was sent to the BlueNRG.
pub static VALUE_UPDATES_SENT: AtomicU32 = AtomicU32::new(0);

const NAME: &[u8] = b"WESU1-Lab";

// HCI opcodes
const OP_WRITE_CONFIG_DATA: u16 = 0xFC0C;
const OP_GAP_SET_DISCOVERABLE: u16 = 0xFC83;
const OP_GAP_SET_IO_CAPABILITY: u16 = 0xFC85;
const OP_GAP_SET_AUTH_REQUIREMENT: u16 = 0xFC86;
const OP_GAP_INIT: u16 = 0xFC8A;
const OP_GAP_SLAVE_SECURITY_REQUEST: u16 = 0xFC8D;
const OP_GAP_CLEAR_SECURITY_DB: u16 = 0xFC94;
const OP_GAP_ALLOW_REBOND: u16 = 0xFC95;
const OP_GATT_INIT: u16 = 0xFD01;
const OP_GATT_ADD_SERVICE: u16 = 0xFD02;
const OP_GATT_ADD_CHAR: u16 = 0xFD04;
const OP_GATT_UPDATE_CHAR_VALUE: u16 = 0xFD06;

const FIXED_PIN: Option<u32> = None;
const MITM: u8 = if FIXED_PIN.is_some() { 1 } else { 0 };
const CLEAR_BONDS_ON_BOOT: bool = false; // Kept false so phones can reconnect

// ---------- BlueNRG-MS SPI transport ----------
struct Bnrg<'d> {
    spi: Spi<'d, Blocking, Master>,
    cs: Output<'d>,
    irq: ExtiInput<'d, Async>,
    resp: [u8; 12],
}

impl<'d> Bnrg<'d> {
    fn header(&mut self, op: u8) -> [u8; 5] {
        let mut h = [op, 0, 0, 0, 0];
        self.spi.blocking_transfer_in_place(&mut h).ok();
        h
    }

    async fn write(&mut self, data: &[u8]) -> bool {
        for _ in 0..200 {
            self.cs.set_low();
            let h = self.header(0x0A);
            if h[0] == 0x02 && (h[1] as usize) >= data.len() {
                self.spi.blocking_write(data).ok();
                self.cs.set_high();
                return true;
            }
            self.cs.set_high();
            Timer::after_millis(1).await;
        }
        false
    }

    fn read(&mut self, buf: &mut [u8]) -> usize {
        self.cs.set_low();
        let h = self.header(0x0B);
        let mut stored = 0;
        if h[0] == 0x02 {
            let avail = ((h[4] as usize) << 8) | (h[3] as usize);
            stored = avail.min(buf.len());
            buf[..stored].fill(0xFF);
            self.spi.blocking_transfer_in_place(&mut buf[..stored]).ok();
            for _ in stored..avail {
                let mut b = [0xFFu8];
                self.spi.blocking_transfer_in_place(&mut b).ok();
            }
        }
        self.cs.set_high();
        stored
    }

    async fn wait_event(&mut self, timeout_ms: u64) -> bool {
        with_timeout(Duration::from_millis(timeout_ms), self.irq.wait_for_high())
            .await
            .is_ok()
    }

    async fn send_cmd(&mut self, opcode: u16, params: &[u8]) -> bool {
        let mut pkt = [0u8; 64];
        pkt[0] = 0x01;
        pkt[1] = opcode as u8;
        pkt[2] = (opcode >> 8) as u8;
        pkt[3] = params.len() as u8;
        pkt[4..4 + params.len()].copy_from_slice(params);
        self.write(&pkt[..4 + params.len()]).await
    }

    async fn wait_status(&mut self, opcode: u16) -> Option<u8> {
        let mut buf = [0u8; 64];
        for _ in 0..50 {
            if !self.wait_event(200).await {
                return None;
            }
            let n = self.read(&mut buf);
            if n >= 7 && buf[0] == 0x04 {
                if buf[1] == 0x0E && buf[4] == opcode as u8 && buf[5] == (opcode >> 8) as u8 {
                    let m = (n - 7).min(self.resp.len());
                    self.resp[..m].copy_from_slice(&buf[7..7 + m]);
                    return Some(buf[6]);
                }
                if buf[1] == 0x0F && buf[5] == opcode as u8 && buf[6] == (opcode >> 8) as u8 {
                    return Some(buf[3]);
                }
            }
            if n == 0 {
                Timer::after_millis(1).await;
            }
        }
        None
    }

    async fn command(&mut self, opcode: u16, params: &[u8]) -> bool {
        self.send_cmd(opcode, params).await && self.wait_status(opcode).await == Some(0)
    }

    async fn set_device_name(&mut self) -> bool {
        let mut p = [0u8; 48];
        p[0..4].copy_from_slice(&self.resp[0..4]);
        p[4] = 0x00;
        p[5] = NAME.len() as u8;
        p[6..6 + NAME.len()].copy_from_slice(NAME);
        self.command(OP_GATT_UPDATE_CHAR_VALUE, &p[..6 + NAME.len()])
            .await
    }

    async fn setup_security(&mut self) -> bool {
        let (io_cap, use_pin, pin) = match FIXED_PIN {
            Some(pin) => (0x00u8, 1u8, pin),
            None => (0x03u8, 0u8, 0u32),
        };
        if !self.command(OP_GAP_SET_IO_CAPABILITY, &[io_cap]).await {
            return false;
        }
        let mut a = [0u8; 26];
        a[0] = MITM;
        a[1] = 0x00;
        a[18] = 7;
        a[19] = 16;
        a[20] = use_pin;
        a[21..25].copy_from_slice(&pin.to_le_bytes());
        a[25] = 0x01;
        self.command(OP_GAP_SET_AUTH_REQUIREMENT, &a).await
    }

    async fn start_advertising(&mut self) -> bool {
        let mut p = [0u8; 48];
        let mut i = 0;
        let mut put = |bytes: &[u8]| {
            p[i..i + bytes.len()].copy_from_slice(bytes);
            i += bytes.len();
        };
        put(&[0x00]);
        put(&[0xA0, 0x00]);
        put(&[0xF0, 0x00]);
        put(&[0x00]);
        put(&[0x00]);
        put(&[NAME.len() as u8 + 1]);
        put(&[0x09]);
        put(NAME);
        put(&[0x00]);
        put(&[0x00, 0x00]);
        put(&[0x00, 0x00]);
        self.command(OP_GAP_SET_DISCOVERABLE, &p[..i]).await
    }
}

#[embassy_executor::task]
async fn blink(mut led: Output<'static>) {
    loop {
        let half_period_ms = match STAGE.load(Ordering::Relaxed) {
            s if s >= 0xE0 => 50,
            7 | 9 | 10 => 100,
            _ => 500,
        };
        led.toggle();
        Timer::after_millis(half_period_ms).await;
    }
}

async fn halt() -> ! {
    loop {
        Timer::after_secs(1).await;
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    STAGE.store(1, Ordering::Relaxed);
    let p = embassy_stm32::init(Default::default());

    // SPI1 setup
    let mut spi_cfg = spi::Config::default();
    spi_cfg.frequency = Hertz(1_000_000);
    let spi = Spi::new_blocking(p.SPI1, p.PA5, p.PA7, p.PA6, spi_cfg);

    let mut rst = Output::new(p.PA9, Level::High, Speed::Low);
    let cs = Output::new(p.PC3, Level::High, Speed::VeryHigh);
    let irq = ExtiInput::new(p.PC13, p.EXTI13, Pull::Down, Irqs);

    // I2C1 setup. The pins are UNVERIFIED. Embassy's order is (peri, SCL, SDA, config).
    // If I2C_ADDR_FOUND stays 0, try (p.PB6 = SCL, p.PB7 = SDA) instead of PB8/PB9.
    let mut i2c_cfg = i2c::Config::default();
    i2c_cfg.frequency = Hertz(100_000);
    let mut i2c = I2c::new_blocking(p.I2C1, p.PB8, p.PB9, i2c_cfg);

    let led = Output::new(p.PB0, Level::Low, Speed::Low);
    spawner.spawn(blink(led)).unwrap();

    let mut bnrg = Bnrg {
        spi,
        cs,
        irq,
        resp: [0; 12],
    };

    // Find the LSM6DS3: address is 0x6A when SA0 is low, 0x6B when high.
    // Read WHO_AM_I (register 0x0F) to confirm it is really there.
    let mut sensor_addr = 0x6Au8;
    for addr in [0x6Au8, 0x6B] {
        let mut who = [0u8; 1];
        if i2c.blocking_write_read(addr, &[0x0F], &mut who).is_ok() {
            I2C_ADDR_FOUND.store(addr as u32, Ordering::Relaxed);
            I2C_WHOAMI.store(who[0] as u32, Ordering::Relaxed);
            sensor_addr = addr;
            break;
        }
    }
    // CTRL1_XL (0x10) = 0x40: accelerometer on, 104 Hz, +-2 g
    i2c.blocking_write(sensor_addr, &[0x10, 0x40]).ok();

    // Hardware reset of the BlueNRG-MS
    rst.set_low();
    Timer::after_millis(10).await;
    rst.set_high();

    let mut buf = [0u8; 64];
    if !bnrg.wait_event(1000).await {
        STAGE.store(0xE1, Ordering::Relaxed);
        halt().await;
    }
    bnrg.read(&mut buf);
    STAGE.store(2, Ordering::Relaxed);

    // Set custom MAC address
    let addr = [0x12u8, 0x34, 0x00, 0xE1, 0x80, 0x05];
    let mut cfg_data = [0u8; 8];
    cfg_data[0] = 0x00;
    cfg_data[1] = 0x06;
    cfg_data[2..8].copy_from_slice(&addr);
    if !bnrg.command(OP_WRITE_CONFIG_DATA, &cfg_data).await {
        STAGE.store(0xE2, Ordering::Relaxed);
        halt().await;
    }
    STAGE.store(3, Ordering::Relaxed);

    if !bnrg.command(OP_GATT_INIT, &[]).await {
        STAGE.store(0xE3, Ordering::Relaxed);
        halt().await;
    }
    STAGE.store(4, Ordering::Relaxed);

    if !bnrg
        .command(OP_GAP_INIT, &[0x01, 0x00, NAME.len() as u8])
        .await
    {
        STAGE.store(0xE4, Ordering::Relaxed);
        halt().await;
    }
    if !bnrg.set_device_name().await {
        STAGE.store(0xE4, Ordering::Relaxed);
        halt().await;
    }
    STAGE.store(5, Ordering::Relaxed);

    // --- Add Custom Accelerometer Service ---
    let accel_service_uuid = [
        0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF,
        0x00,
    ];
    let mut srv_req = [0u8; 19];
    srv_req[0] = 0x02; // 128-bit UUID
    srv_req[1..17].copy_from_slice(&accel_service_uuid);
    srv_req[17] = 0x01; // primary service
    srv_req[18] = 0x07; // max attribute records
    if !bnrg.command(OP_GATT_ADD_SERVICE, &srv_req).await {
        STAGE.store(0xE5, Ordering::Relaxed);
        halt().await;
    }
    let srv_handle = [bnrg.resp[0], bnrg.resp[1]];

    // --- Add Custom Accelerometer Characteristic ---
    let accel_char_uuid = [
        0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF,
        0x01,
    ];
    let mut char_req = [0u8; 25];
    char_req[0] = srv_handle[0];
    char_req[1] = srv_handle[1];
    char_req[2] = 0x02; // 128-bit UUID
    char_req[3..19].copy_from_slice(&accel_char_uuid);
    char_req[19] = 0x06; // value length: 6 bytes for X,Y,Z
    char_req[20] = 0x12; // properties: Read | Notify
    char_req[21] = 0x00; // security permissions: none
    char_req[22] = 0x00; // GATT event mask: none
    char_req[23] = 0x07; // min encryption key size
    char_req[24] = 0x00; // fixed-length value
    if !bnrg.command(OP_GATT_ADD_CHAR, &char_req).await {
        STAGE.store(0xE6, Ordering::Relaxed);
        halt().await;
    }
    let char_handle = [bnrg.resp[0], bnrg.resp[1]];

    if CLEAR_BONDS_ON_BOOT && !bnrg.command(OP_GAP_CLEAR_SECURITY_DB, &[]).await {
        STAGE.store(0xE6, Ordering::Relaxed);
        halt().await;
    }
    if !bnrg.setup_security().await {
        STAGE.store(0xE6, Ordering::Relaxed);
        halt().await;
    }

    if !bnrg.start_advertising().await {
        STAGE.store(0xE5, Ordering::Relaxed);
        halt().await;
    }
    STAGE.store(6, Ordering::Relaxed);

    let mut conn_handle = [0u8; 2];

    // Main Event Loop
    let mut conn_handle = [0u8; 2];
    let mut counter: i16 = 0; // Initialize our test counter

    // Main Event Loop
    loop {
        // Wait for a Bluetooth IRQ, but time out every 1000ms (1 second)
        match with_timeout(Duration::from_millis(1000), bnrg.irq.wait_for_high()).await {
            Ok(_) => {
                let n = bnrg.read(&mut buf);

                if n >= 5 && buf[0] == 0x04 && buf[1] == 0xFF {
                    LAST_VENDOR_EVT
                        .store((buf[3] as u32) | ((buf[4] as u32) << 8), Ordering::Relaxed);
                }

                if n >= 4 && buf[0] == 0x04 {
                    match buf[1] {
                        0x3E if n >= 7 && buf[3] == 0x01 => {
                            STAGE.store(7, Ordering::Relaxed);
                            if buf[4] == 0x00 {
                                conn_handle = [buf[5], buf[6]];
                                let req = [buf[5], buf[6], 0x01, MITM];
                                bnrg.send_cmd(OP_GAP_SLAVE_SECURITY_REQUEST, &req).await;
                            }
                        }
                        0xFF if n >= 5 && buf[3] == 0x05 && buf[4] == 0x04 => {
                            bnrg.send_cmd(OP_GAP_ALLOW_REBOND, &conn_handle).await;
                        }
                        0xFF if n >= 7 && buf[3] == 0x03 && buf[4] == 0x04 => {
                            let req = [conn_handle[0], conn_handle[1], 0x01];
                            bnrg.send_cmd(0xFC93, &req).await;
                        }
                        0xFF if n >= 8 && buf[3] == 0x01 && buf[4] == 0x04 => {
                            PAIR_STATUS.store(buf[7] as u32, Ordering::Relaxed);
                            STAGE.store(if buf[7] == 0x00 { 9 } else { 10 }, Ordering::Relaxed);
                        }
                        0x05 => {
                            STAGE.store(8, Ordering::Relaxed);
                            if bnrg.start_advertising().await {
                                STAGE.store(6, Ordering::Relaxed);
                            }
                        }
                        _ => {}
                    }
                }
            }
            Err(_) => {
                // Timeout fired (1 second passed). Send incrementing test data.
                let state = STAGE.load(Ordering::Relaxed);

                // Only send data if a phone is connected (7) or bonded (9)
                if state == 7 || state == 9 {
                    counter = counter.wrapping_add(1); // Increment counter safely

                    // Create dummy X, Y, Z values based on the counter
                    let x = counter;
                    let y = counter.wrapping_mul(2);
                    let z = counter.wrapping_mul(3);

                    let mut val_req = [0u8; 12];
                    val_req[0] = srv_handle[0];
                    val_req[1] = srv_handle[1];
                    val_req[2] = char_handle[0];
                    val_req[3] = char_handle[1];
                    val_req[4] = 0x00; // offset
                    val_req[5] = 0x06; // length (6 bytes)

                    val_req[6..8].copy_from_slice(&x.to_le_bytes());
                    val_req[8..10].copy_from_slice(&y.to_le_bytes());
                    val_req[10..12].copy_from_slice(&z.to_le_bytes());

                    bnrg.send_cmd(OP_GATT_UPDATE_CHAR_VALUE, &val_req).await;
                }
            }
        }
    }
}
