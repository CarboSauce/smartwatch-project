#![no_std]
#![no_main]

use core::sync::atomic::{AtomicU32, Ordering};

use embassy_executor::Spawner;
use embassy_stm32::exti::{self, ExtiInput};
use embassy_stm32::gpio::{Level, Output, Pull, Speed};
use embassy_stm32::mode::{Async, Blocking};
use embassy_stm32::spi::{self, mode::Master, Spi};
use embassy_stm32::time::Hertz;
use embassy_stm32::{bind_interrupts, interrupt};
use embassy_time::{with_timeout, Duration, Timer};
use panic_halt as _;
use rtt_target::{rprintln, rtt_init_print};

bind_interrupts!(struct Irqs {
    EXTI15_10 => exti::InterruptHandler<interrupt::typelevel::EXTI15_10>;
});

pub static STAGE: AtomicU32 = AtomicU32::new(0);
pub static PAIR_STATUS: AtomicU32 = AtomicU32::new(0xFF);
pub static LAST_VENDOR_EVT: AtomicU32 = AtomicU32::new(0);

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
const CLEAR_BONDS_ON_BOOT: bool = false;

// ---------- LSM6DS3 SPI Driver (Dedicated to SPI3) ----------
struct Lsm6ds3<'d> {
    spi: Spi<'d, Blocking, Master>,
    cs: Output<'d>,
}

impl<'d> Lsm6ds3<'d> {
    fn read_reg(&mut self, reg: u8) -> u8 {
        let mut buf = [reg | 0x80, 0x00];
        self.cs.set_low();
        self.spi.blocking_transfer_in_place(&mut buf).ok();
        self.cs.set_high();
        buf[1]
    }

    fn write_reg(&mut self, reg: u8, val: u8) {
        let mut buf = [reg & 0x7F, val];
        self.cs.set_low();
        self.spi.blocking_transfer_in_place(&mut buf).ok();
        self.cs.set_high();
    }

    // Burst read both Gyroscope (0x22-0x27) and Accelerometer (0x28-0x2D)
    fn read_motion(&mut self) -> (i16, i16, i16, i16, i16, i16) {
        // 1 byte register addr + 12 bytes of payload = 13 bytes
        let mut buf = [0u8; 13];
        buf[0] = 0x22 | 0x80; // Start at OUTX_L_G

        self.cs.set_low();
        self.spi.blocking_transfer_in_place(&mut buf).ok();
        self.cs.set_high();

        let gx = i16::from_le_bytes([buf[1], buf[2]]);
        let gy = i16::from_le_bytes([buf[3], buf[4]]);
        let gz = i16::from_le_bytes([buf[5], buf[6]]);
        let ax = i16::from_le_bytes([buf[7], buf[8]]);
        let ay = i16::from_le_bytes([buf[9], buf[10]]);
        let az = i16::from_le_bytes([buf[11], buf[12]]);

        (ax, ay, az, gx, gy, gz)
    }
}

// ---------- BlueNRG-MS SPI Transport (Dedicated to SPI1) ----------
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
    rtt_init_print!();
    rprintln!("--- STM32 WESU1 Initializing ---");

    STAGE.store(1, Ordering::Relaxed);
    let p = embassy_stm32::init(Default::default());

    let led = Output::new(p.PB0, Level::Low, Speed::Low);
    spawner.spawn(blink(led)).unwrap();

    // ==========================================
    // 1. BlueNRG SPI Setup (Dedicated to SPI1)
    // ==========================================
    let mut spi1_cfg = spi::Config::default();
    spi1_cfg.frequency = Hertz(1_000_000);
    // SCK=PA5, MOSI=PA7, MISO=PA6
    let spi1 = Spi::new_blocking(p.SPI1, p.PA5, p.PA7, p.PA6, spi1_cfg);

    let mut cs_bnrg = Output::new(p.PC3, Level::High, Speed::VeryHigh);
    cs_bnrg.set_high(); // Deselect immediately

    let mut rst = Output::new(p.PA9, Level::High, Speed::Low);
    let irq_bnrg = ExtiInput::new(p.PC13, p.EXTI13, Pull::Down, Irqs);

    let mut bnrg = Bnrg {
        spi: spi1,
        cs: cs_bnrg,
        irq: irq_bnrg,
        resp: [0; 12],
    };

    // ==========================================
    // 2. LSM6DS3 Accelerometer/Gyro (Dedicated to SPI3)
    // ==========================================
    let mut spi3_cfg = spi::Config::default();
    spi3_cfg.frequency = Hertz(1_000_000);
    // SCK=PB3, MOSI=PB5, MISO=PB4
    let spi3 = Spi::new_blocking(p.SPI3, p.PB3, p.PB5, p.PB4, spi3_cfg);

    let mut cs_accel = Output::new(p.PB6, Level::High, Speed::VeryHigh);
    cs_accel.set_high(); // Deselect immediately

    let mut accel = Lsm6ds3 {
        spi: spi3,
        cs: cs_accel,
    };

    // ------------------------------------------
    // Initialize Hardware
    // ------------------------------------------
    let whoami = accel.read_reg(0x0F);
    rprintln!("LSM6DS3 WHO_AM_I: 0x{:02X}", whoami);

    if whoami == 0x69 || whoami == 0x6A {
        rprintln!("LSM6DS3 successfully identified!");
    } else {
        rprintln!("WARNING: Unexpected WHO_AM_I.");
    }

    // CTRL1_XL (0x10) = 0x40 (Accel: 104 Hz ODR, +/-2g range)
    accel.write_reg(0x10, 0x40);
    // CTRL2_G (0x11) = 0x40 (Gyro: 104 Hz ODR, 250 dps)
    accel.write_reg(0x11, 0x40);

    // Hardware Reset BlueNRG
    rst.set_low();
    Timer::after_millis(10).await;
    rst.set_high();

    let mut buf = [0u8; 64];
    if !bnrg.wait_event(1000).await {
        rprintln!("ERROR: BlueNRG init timeout");
        STAGE.store(0xE1, Ordering::Relaxed);
        halt().await;
    }
    bnrg.read(&mut buf);

    // Give BlueNRG firmware time to settle post-reset
    Timer::after_millis(50).await;
    STAGE.store(2, Ordering::Relaxed);

    // Set custom MAC address (Non-fatal if rejected by stack)
    let addr = [0x12u8, 0x34, 0x00, 0xE1, 0x80, 0x05];
    let mut cfg_data = [0u8; 8];
    cfg_data[0] = 0x00;
    cfg_data[1] = 0x06;
    cfg_data[2..8].copy_from_slice(&addr);

    if !bnrg.command(OP_WRITE_CONFIG_DATA, &cfg_data).await {
        rprintln!("WARNING: Failed setting custom MAC, using default.");
    } else {
        rprintln!("Custom MAC set successfully.");
    }
    STAGE.store(3, Ordering::Relaxed);

    if !bnrg.command(OP_GATT_INIT, &[]).await {
        rprintln!("ERROR: GATT init failed");
        STAGE.store(0xE3, Ordering::Relaxed);
        halt().await;
    }
    STAGE.store(4, Ordering::Relaxed);

    if !bnrg
        .command(OP_GAP_INIT, &[0x01, 0x00, NAME.len() as u8])
        .await
    {
        rprintln!("ERROR: GAP init failed");
        STAGE.store(0xE4, Ordering::Relaxed);
        halt().await;
    }

    if !bnrg.set_device_name().await {
        rprintln!("ERROR: Failed setting device name");
        STAGE.store(0xE4, Ordering::Relaxed);
        halt().await;
    }
    STAGE.store(5, Ordering::Relaxed);

    // --- Add Custom IMU Service ---
    let accel_service_uuid = [
        0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF,
        0x00,
    ];
    let mut srv_req = [0u8; 19];
    srv_req[0] = 0x02; // 128-bit UUID
    srv_req[1..17].copy_from_slice(&accel_service_uuid);
    srv_req[17] = 0x01; // Primary service
    srv_req[18] = 0x07; // Max attribute records
    if !bnrg.command(OP_GATT_ADD_SERVICE, &srv_req).await {
        rprintln!("ERROR: GATT Add Service failed");
        STAGE.store(0xE5, Ordering::Relaxed);
        halt().await;
    }
    let srv_handle = [bnrg.resp[0], bnrg.resp[1]];

    // --- Add Custom IMU Characteristic (12 bytes) ---
    let accel_char_uuid = [
        0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF,
        0x01,
    ];
    let mut char_req = [0u8; 25];
    char_req[0] = srv_handle[0];
    char_req[1] = srv_handle[1];
    char_req[2] = 0x02; // 128-bit UUID
    char_req[3..19].copy_from_slice(&accel_char_uuid);
    char_req[19] = 0x0C; // Value length: 12 bytes (Accel X,Y,Z + Gyro X,Y,Z)
    char_req[20] = 0x12; // Properties: Read | Notify
    char_req[21] = 0x00; // Security permissions: none
    char_req[22] = 0x00; // GATT event mask: none
    char_req[23] = 0x07; // Min encryption key size
    char_req[24] = 0x00; // Fixed-length value
    if !bnrg.command(OP_GATT_ADD_CHAR, &char_req).await {
        rprintln!("ERROR: GATT Add Characteristic failed");
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
        rprintln!("ERROR: Advertising failed");
        STAGE.store(0xE5, Ordering::Relaxed);
        halt().await;
    }
    rprintln!("BLE Advertising started as 'WESU1-Lab'");
    STAGE.store(6, Ordering::Relaxed);

    let mut conn_handle = [0u8; 2];

    // --- Main Event Loop ---
    loop {
        match with_timeout(Duration::from_millis(100), bnrg.irq.wait_for_high()).await {
            Ok(_) => {
                let n = bnrg.read(&mut buf);

                if n >= 5 && buf[0] == 0x04 && buf[1] == 0xFF {
                    LAST_VENDOR_EVT
                        .store((buf[3] as u32) | ((buf[4] as u32) << 8), Ordering::Relaxed);
                }

                if n >= 4 && buf[0] == 0x04 {
                    match buf[1] {
                        0x3E if n >= 7 && buf[3] == 0x01 => {
                            rprintln!("BLE Status: Connected");
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
                            rprintln!("BLE Status: Disconnected");
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
                // Timeout fired (1 second passed). Read live IMU values.
                let (ax, ay, az, gx, gy, gz) = accel.read_motion();

                // Static print string bypassing the integer formatting bug on M3 architectures
                rprintln!("xyz: {} {} {} ; gxgygz: {} {} {}", ax, ay, az, gx, gy, gz);

                // Only send data if a phone is connected (7) or bonded (9)
                let state = STAGE.load(Ordering::Relaxed);
                if state == 7 || state == 9 {
                    // Packet structure: 6 byte header + 12 byte payload = 18 bytes
                    let mut val_req = [0u8; 18];
                    val_req[0] = srv_handle[0];
                    val_req[1] = srv_handle[1];
                    val_req[2] = char_handle[0];
                    val_req[3] = char_handle[1];
                    val_req[4] = 0x00; // offset
                    val_req[5] = 0x0C; // length (12 bytes)

                    // Pack Accelerometer Data
                    val_req[6..8].copy_from_slice(&ax.to_le_bytes());
                    val_req[8..10].copy_from_slice(&ay.to_le_bytes());
                    val_req[10..12].copy_from_slice(&az.to_le_bytes());

                    // Pack Gyroscope Data
                    val_req[12..14].copy_from_slice(&gx.to_le_bytes());
                    val_req[14..16].copy_from_slice(&gy.to_le_bytes());
                    val_req[16..18].copy_from_slice(&gz.to_le_bytes());

                    bnrg.send_cmd(OP_GATT_UPDATE_CHAR_VALUE, &val_req).await;
                }
            }
        }
    }
}
