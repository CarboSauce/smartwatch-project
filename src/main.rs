#![no_std]
#![no_main]

use core::sync::atomic::{AtomicBool, AtomicU32, AtomicU8, Ordering};

use embassy_executor::Spawner;
use embassy_stm32::exti::{self, ExtiInput};
use embassy_stm32::gpio::{Level, Output, Pull, Speed};
use embassy_stm32::mode::{Async, Blocking};
use embassy_stm32::spi::{self, mode::Master, Spi};
use embassy_stm32::time::Hertz;
use embassy_stm32::{bind_interrupts, interrupt};
use embassy_time::{with_timeout, Duration, Instant, Timer};
use micromath::F32Ext;
use panic_halt as _;
use rtt_target::{rprintln, rtt_init_print};

mod fall;

bind_interrupts!(struct Irqs {
    EXTI2 => exti::InterruptHandler<interrupt::typelevel::EXTI2>;
    EXTI15_10 => exti::InterruptHandler<interrupt::typelevel::EXTI15_10>;
});

pub static STAGE: AtomicU32 = AtomicU32::new(0);
pub static PAIR_STATUS: AtomicU32 = AtomicU32::new(0xFF);
pub static LAST_VENDOR_EVT: AtomicU32 = AtomicU32::new(0);
pub static FALL_DETECTED: AtomicBool = AtomicBool::new(false);

// BLE Alert Notification Queue: 0 = none, 1 = send FALL (0x01), 2 = send CANCELED (0x00)
pub static PENDING_ALERT_NOTIFY: AtomicU8 = AtomicU8::new(0);

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
const OP_L2CAP_CONN_PARAM_UPDATE_REQ: u16 = 0xFD81;
const OP_GATT_INIT: u16 = 0xFD01;
const OP_GATT_ADD_SERVICE: u16 = 0xFD02;
const OP_GATT_ADD_CHAR: u16 = 0xFD04;
const OP_GATT_UPDATE_CHAR_VALUE: u16 = 0xFD06;

const FIXED_PIN: Option<u32> = None;
const MITM: u8 = if FIXED_PIN.is_some() { 1 } else { 0 };
const CLEAR_BONDS_ON_BOOT: bool = false;

// =========================================================================
// 1. LSM6DS3 Accelerometer/Gyroscope Driver
// =========================================================================
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

    fn read_motion(&mut self) -> (i16, i16, i16, i16, i16, i16) {
        let mut buf = [0u8; 13];
        buf[0] = 0x22 | 0x80;

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

// =========================================================================
// 2. ML-Based Fall Detector (Real-Time Feature Extraction)
// =========================================================================
const WINDOW_SIZE: usize = 50; // 1 second at 50 Hz

#[derive(Copy, Clone, PartialEq)]
enum DetectorState {
    Monitoring,
    CollectingPostImpact {
        partial_features: fall::Features,
        count: usize,
    },
}

pub struct SmartwatchFallDetector {
    acc_buffer: [f32; WINDOW_SIZE],
    gyr_buffer: [f32; WINDOW_SIZE],
    post_acc_buffer: [f32; WINDOW_SIZE],
    buffer_idx: usize,
    buffer_filled: bool,
    state: DetectorState,
}

impl SmartwatchFallDetector {
    pub fn new() -> Self {
        Self {
            acc_buffer: [0.0; WINDOW_SIZE],
            gyr_buffer: [0.0; WINDOW_SIZE],
            post_acc_buffer: [0.0; WINDOW_SIZE],
            buffer_idx: 0,
            buffer_filled: false,
            state: DetectorState::Monitoring,
        }
    }

    fn calc_stats(buf: &[f32]) -> (f32, f32) {
        let len = buf.len() as f32;
        let mean = buf.iter().sum::<f32>() / len;
        let var = buf.iter().map(|&x| (x - mean) * (x - mean)).sum::<f32>() / len;
        (mean, var.sqrt())
    }

    pub fn process_sample(&mut self, ax: f32, ay: f32, az: f32, gx: f32, gy: f32, gz: f32) -> bool {
        let acc_mag = (ax * ax + ay * ay + az * az).sqrt();
        let gyr_mag = (gx * gx + gy * gy + gz * gz).sqrt();

        self.acc_buffer[self.buffer_idx] = acc_mag;
        self.gyr_buffer[self.buffer_idx] = gyr_mag;
        self.buffer_idx = (self.buffer_idx + 1) % WINDOW_SIZE;
        if self.buffer_idx == 0 {
            self.buffer_filled = true;
        }

        if !self.buffer_filled {
            return false;
        }

        match self.state {
            DetectorState::Monitoring => {
                let mut acc_max = 0.0_f32;
                let mut acc_min = f32::MAX;
                let mut gyr_max = 0.0_f32;

                for i in 0..WINDOW_SIZE {
                    let a = self.acc_buffer[i];
                    let g = self.gyr_buffer[i];

                    if a > acc_max {
                        acc_max = a;
                    }
                    if a < acc_min {
                        acc_min = a;
                    }
                    if g > gyr_max {
                        gyr_max = g;
                    }
                }

                if acc_max > 2.232382 {
                    let (acc_mean, acc_std) = Self::calc_stats(&self.acc_buffer);
                    let (gyr_mean, gyr_std) = Self::calc_stats(&self.gyr_buffer);

                    let partial_features = fall::Features {
                        acc_max,
                        acc_min,
                        acc_mean,
                        acc_std,
                        acc_range: acc_max - acc_min,
                        gyr_max,
                        gyr_mean,
                        gyr_std,
                        post_acc_mean: 0.0,
                        post_acc_std: 0.0,
                    };

                    self.state = DetectorState::CollectingPostImpact {
                        partial_features,
                        count: 0,
                    };
                }
            }

            DetectorState::CollectingPostImpact {
                mut partial_features,
                mut count,
            } => {
                self.post_acc_buffer[count] = acc_mag;
                count += 1;

                if count >= WINDOW_SIZE {
                    let (post_mean, post_std) = Self::calc_stats(&self.post_acc_buffer);
                    partial_features.post_acc_mean = post_mean;
                    partial_features.post_acc_std = post_std;

                    self.state = DetectorState::Monitoring;

                    if fall::is_fall(&partial_features) {
                        return true;
                    }
                } else {
                    self.state = DetectorState::CollectingPostImpact {
                        partial_features,
                        count,
                    };
                }
            }
        }

        false
    }
}

// =========================================================================
// 3. BlueNRG-MS SPI HCI Transport Driver
// =========================================================================
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
        for _ in 0..20 {
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

// =========================================================================
// 4. Tasks and Helper Functions
// =========================================================================
#[embassy_executor::task]
async fn cancel_button_task(mut btn: ExtiInput<'static, Async>) {
    loop {
        btn.wait_for_rising_edge().await;
        Timer::after_millis(50).await; // debounce

        if btn.is_high() {
            if FALL_DETECTED.load(Ordering::Relaxed) {
                rprintln!("*** FALL ALERT CANCELED BY USER ***");
                FALL_DETECTED.store(false, Ordering::Relaxed);
                PENDING_ALERT_NOTIFY.store(2, Ordering::Relaxed); // sends 0x00
            } else {
                rprintln!("*** BUTTON PRESSED ***");
                PENDING_ALERT_NOTIFY.store(3, Ordering::Relaxed); // sends 0x02
            }
        }

        btn.wait_for_falling_edge().await;
    }
}

#[embassy_executor::task]
async fn blink(mut led: Output<'static>) {
    let mut fall_start: Option<Instant> = None;

    loop {
        let fall_detected = FALL_DETECTED.load(Ordering::Relaxed);

        if fall_detected {
            let start = *fall_start.get_or_insert_with(Instant::now);

            // 10-second window for user to press PA2 to cancel before phone sends alert
            if start.elapsed() >= Duration::from_secs(10) {
                FALL_DETECTED.store(false, Ordering::Relaxed);
                fall_start = None;
                led.set_low();
                continue;
            }
        } else if fall_start.is_some() {
            // Fall alert was canceled via PA2 button before timeout
            fall_start = None;
            led.set_low();
        }

        let half_period_ms = if fall_detected {
            50
        } else {
            match STAGE.load(Ordering::Relaxed) {
                s if s >= 0xE0 => 50,
                7 | 9 | 10 => 100,
                _ => 500,
            }
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

// =========================================================================
// 5. Main Embassy Async Entry Point
// =========================================================================
#[embassy_executor::main]
async fn main(spawner: Spawner) {
    rtt_init_print!();
    rprintln!("--- STM32 WESU1 Initializing ---");

    STAGE.store(1, Ordering::Relaxed);
    let p = embassy_stm32::init(Default::default());

    let led = Output::new(p.PB0, Level::Low, Speed::Low);
    spawner.spawn(blink(led)).unwrap();

    // Button_PowerOn on PA2 (EXTI2, active-high)
    let cancel_btn = ExtiInput::new(p.PA2, p.EXTI2, Pull::Down, Irqs);
    spawner.spawn(cancel_button_task(cancel_btn)).unwrap();

    let mut spi1_cfg = spi::Config::default();
    spi1_cfg.frequency = Hertz(1_000_000);
    let spi1 = Spi::new_blocking(p.SPI1, p.PA5, p.PA7, p.PA6, spi1_cfg);

    let mut cs_bnrg = Output::new(p.PC3, Level::High, Speed::VeryHigh);
    cs_bnrg.set_high();

    let mut rst = Output::new(p.PA9, Level::High, Speed::Low);
    let irq_bnrg = ExtiInput::new(p.PC13, p.EXTI13, Pull::Down, Irqs);

    let mut bnrg = Bnrg {
        spi: spi1,
        cs: cs_bnrg,
        irq: irq_bnrg,
        resp: [0; 12],
    };

    let mut spi3_cfg = spi::Config::default();
    spi3_cfg.frequency = Hertz(1_000_000);
    let spi3 = Spi::new_blocking(p.SPI3, p.PB3, p.PB5, p.PB4, spi3_cfg);

    let mut cs_accel = Output::new(p.PB6, Level::High, Speed::VeryHigh);
    cs_accel.set_high();

    let mut accel = Lsm6ds3 {
        spi: spi3,
        cs: cs_accel,
    };

    let whoami = accel.read_reg(0x0F);
    rprintln!("LSM6DS3 WHO_AM_I: 0x{:02X}", whoami);

    // ODR = 104 Hz (0x40) on Accel & Gyro so 50 Hz sampling always reads fresh data
    // CTRL1_XL = 0x4C: 104 Hz, +/-8 g.  CTRL2_G = 0x4C: 104 Hz, +/-2000 dps.
    // (the fall models were trained on impacts up to ~6.5 g and >1000 dps)
    accel.write_reg(0x10, 0x4C);
    accel.write_reg(0x11, 0x4C);

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
    Timer::after_millis(50).await;
    STAGE.store(2, Ordering::Relaxed);

    let addr = [0x12u8, 0x34, 0x00, 0xE1, 0x80, 0x05];
    let mut cfg_data = [0u8; 8];
    cfg_data[0] = 0x00;
    cfg_data[1] = 0x06;
    cfg_data[2..8].copy_from_slice(&addr);

    if !bnrg.command(OP_WRITE_CONFIG_DATA, &cfg_data).await {
        rprintln!("WARNING: Failed setting custom MAC, using default.");
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

    // Primary GATT Service (max_attribute_records = 0x0A for 2 characteristics + CCCDs)
    let accel_service_uuid = [
        0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF,
        0x00,
    ];
    let mut srv_req = [0u8; 19];
    srv_req[0] = 0x02;
    srv_req[1..17].copy_from_slice(&accel_service_uuid);
    srv_req[17] = 0x01;
    srv_req[18] = 0x0A;
    if !bnrg.command(OP_GATT_ADD_SERVICE, &srv_req).await {
        STAGE.store(0xE5, Ordering::Relaxed);
        halt().await;
    }
    let srv_handle = [bnrg.resp[0], bnrg.resp[1]];

    // Characteristic 1: 50 Hz IMU Stream (...FF01, 14 bytes: 12B axes + 2B seq, Notify | Read)
    let accel_char_uuid = [
        0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF,
        0x01,
    ];
    let mut char_req = [0u8; 25];
    char_req[0] = srv_handle[0];
    char_req[1] = srv_handle[1];
    char_req[2] = 0x02;
    char_req[3..19].copy_from_slice(&accel_char_uuid);
    char_req[19] = 0x0E; // Max value len = 14 bytes (ax, ay, az, gx, gy, gz, seq_u16)
    char_req[20] = 0x12; // Properties: Notify (0x10) | Read (0x02)
    char_req[21] = 0x00;
    char_req[22] = 0x00;
    char_req[23] = 0x07;
    char_req[24] = 0x00;
    if !bnrg.command(OP_GATT_ADD_CHAR, &char_req).await {
        STAGE.store(0xE6, Ordering::Relaxed);
        halt().await;
    }
    let char_handle = [bnrg.resp[0], bnrg.resp[1]];

    // Characteristic 2: Fall Alert Status (...FF02, 1 byte, Notify | Read)
    // Payload: 0x01 = Fall Detected, 0x00 = Fall Canceled / Normal
    let alert_char_uuid = [
        0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF,
        0x02,
    ];
    let mut alert_req = [0u8; 25];
    alert_req[0] = srv_handle[0];
    alert_req[1] = srv_handle[1];
    alert_req[2] = 0x02;
    alert_req[3..19].copy_from_slice(&alert_char_uuid);
    alert_req[19] = 0x01; // Max value len = 1 byte
    alert_req[20] = 0x12; // Properties: Notify (0x10) | Read (0x02)
    alert_req[21] = 0x00;
    alert_req[22] = 0x00;
    alert_req[23] = 0x07;
    alert_req[24] = 0x00;
    if !bnrg.command(OP_GATT_ADD_CHAR, &alert_req).await {
        STAGE.store(0xE6, Ordering::Relaxed);
        halt().await;
    }
    let alert_char_handle = [bnrg.resp[0], bnrg.resp[1]];

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
    rprintln!("BLE Advertising started as 'WESU1-Lab'");
    STAGE.store(6, Ordering::Relaxed);

    let mut conn_handle = [0u8; 2];
    let mut fall_detector = SmartwatchFallDetector::new();
    let mut last_imu_sample = Instant::now();
    let mut last_rtt_log = Instant::now();
    let mut sample_seq: u16 = 0;

    loop {
        // Drain any pending BlueNRG HCI events (including Command Complete from 50Hz updates)
        while bnrg.irq.is_high() {
            let n = bnrg.read(&mut buf);
            if n == 0 {
                break;
            }

            if n >= 5 && buf[0] == 0x04 && buf[1] == 0xFF {
                LAST_VENDOR_EVT.store((buf[3] as u32) | ((buf[4] as u32) << 8), Ordering::Relaxed);
            }

            if n >= 4 && buf[0] == 0x04 {
                match buf[1] {
                    0x3E if n >= 7 && buf[3] == 0x01 => {
                        rprintln!("BLE Status: Connected");
                        STAGE.store(7, Ordering::Relaxed);
                        if buf[4] == 0x00 {
                            conn_handle = [buf[5], buf[6]];
                            let sec_req = [buf[5], buf[6], 0x01, MITM];
                            bnrg.send_cmd(OP_GAP_SLAVE_SECURITY_REQUEST, &sec_req).await;

                            // Request fast connection interval (7.5ms - 15ms) for 50Hz streaming
                            let conn_params = [
                                buf[5], buf[6], // conn_handle
                                0x06, 0x00, // interval_min = 6 * 1.25ms = 7.5ms
                                0x0C, 0x00, // interval_max = 12 * 1.25ms = 15ms
                                0x00, 0x00, // slave_latency = 0
                                0xC8, 0x00, // timeout_multiplier = 200 * 10ms = 2000ms
                            ];
                            bnrg.send_cmd(OP_L2CAP_CONN_PARAM_UPDATE_REQ, &conn_params)
                                .await;
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

        // Send immediate BLE notification when a fall is detected (0x01) or canceled (0x00)
        let pending_alert = PENDING_ALERT_NOTIFY.swap(0, Ordering::Relaxed);
        if pending_alert != 0 {
            let alert_byte = match pending_alert {
                1 => 0x01, // fall detected
                2 => 0x00, // fall canceled by button
                3 => 0x02, // button pressed, no fall
                _ => 0x00,
            };
            let mut alert_val_req = [0u8; 7];
            alert_val_req[0] = srv_handle[0];
            alert_val_req[1] = srv_handle[1];
            alert_val_req[2] = alert_char_handle[0];
            alert_val_req[3] = alert_char_handle[1];
            alert_val_req[4] = 0x00;
            alert_val_req[5] = 0x01; // 1-byte value
            alert_val_req[6] = alert_byte;

            bnrg.send_cmd(OP_GATT_UPDATE_CHAR_VALUE, &alert_val_req)
                .await;
            rprintln!("BLE Alert Char updated: 0x{:02X}", alert_byte);
        }

        // Periodic 50 Hz IMU Sampling, ML Processing, & BLE Streaming (Every 20ms)
        if last_imu_sample.elapsed() >= Duration::from_millis(20) {
            // Advance timestamp by exact 20ms to prevent drift
            last_imu_sample += Duration::from_millis(20);
            if last_imu_sample.elapsed() >= Duration::from_millis(20) {
                last_imu_sample = Instant::now();
            }

            let (ax, ay, az, gx, gy, gz) = accel.read_motion();

            // Convert to physical units (g and dps)
            // +/-8 g -> 0.244 mg/LSB, +/-2000 dps -> 70 mdps/LSB
            let ax_g = (ax as f32) * 0.000244;
            let ay_g = (ay as f32) * 0.000244;
            let az_g = (az as f32) * 0.000244;

            let gx_dps = (gx as f32) * 0.07;
            let gy_dps = (gy as f32) * 0.07;
            let gz_dps = (gz as f32) * 0.07;

            // Feed samples to on-device ML pipeline
            if fall_detector.process_sample(ax_g, ay_g, az_g, gx_dps, gy_dps, gz_dps) {
                rprintln!("*** ALERT: ML DECISION TREE CONFIRMED FALL ***");
                FALL_DETECTED.store(true, Ordering::Relaxed);
                PENDING_ALERT_NOTIFY.store(1, Ordering::Relaxed);
            }

            // Stream 50 Hz IMU packet to phone over BLE when connected
            let state = STAGE.load(Ordering::Relaxed);
            if state == 7 || state == 9 {
                let mut val_req = [0u8; 20];
                val_req[0] = srv_handle[0];
                val_req[1] = srv_handle[1];
                val_req[2] = char_handle[0];
                val_req[3] = char_handle[1];
                val_req[4] = 0x00;
                val_req[5] = 0x0E; // 14 bytes

                val_req[6..8].copy_from_slice(&ax.to_le_bytes());
                val_req[8..10].copy_from_slice(&ay.to_le_bytes());
                val_req[10..12].copy_from_slice(&az.to_le_bytes());

                val_req[12..14].copy_from_slice(&gx.to_le_bytes());
                val_req[14..16].copy_from_slice(&gy.to_le_bytes());
                val_req[16..18].copy_from_slice(&gz.to_le_bytes());

                val_req[18..20].copy_from_slice(&sample_seq.to_le_bytes());
                sample_seq = sample_seq.wrapping_add(1);

                bnrg.send_cmd(OP_GATT_UPDATE_CHAR_VALUE, &val_req).await;
            }

            // Keep RTT debug print at 1 Hz so RTT doesn't slow down the 50 Hz loop
            if last_rtt_log.elapsed() >= Duration::from_millis(1000) {
                last_rtt_log = Instant::now();
                rprintln!(
                    "seq: {} | xyz: {} {} {} ; gxgygz: {} {} {}",
                    sample_seq,
                    ax,
                    ay,
                    az,
                    gx,
                    gy,
                    gz
                );
            }
        }

        // Yield briefly (1ms) so background tasks (blink, PA2 button) run smoothly
        Timer::after_millis(1).await;
    }
}
