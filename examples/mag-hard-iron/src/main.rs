//! Soft-iron calibration for the LIS3MDL magnetometer on Sirin S2-R1,
//! using a hard-iron bias you already measured.
//!
//! How to use:
//!   1. Put your hard-iron bias (µT) in HARD_IRON_BIAS below.
//!   2. Flash this example and mount the board the way it will fly
//!      (in the ebay, with batteries and wiring attached) - nearby metal
//!      changes the result.
//!   3. During the countdown, move away from laptops, tools and metal benches.
//!   4. For COLLECT_SECONDS, slowly rotate the board through every orientation:
//!      tumble it end over end around all three axes so each axis points
//!      both up and down several times.
//!   5. Copy the printed SOFT_IRON matrix. Apply it as:
//!          corrected = SOFT_IRON * (raw_uT - HARD_IRON_BIAS)
//!
//! Method: after subtracting the hard-iron bias, readings lie on an ellipsoid
//! vᵀ M v = 1 centred at the origin. M (symmetric, 6 unknowns) is found by
//! linear least squares. The soft-iron matrix is A = F · sqrt(M), which maps
//! the ellipsoid back onto a sphere of radius F (the local field strength, µT).
//!
//! Every sample is also printed as "Magnetometer: [x, y, z]" (raw µT, before
//! any correction), so the output still works with ../mag-calibrate/store-mag.py.
//!
//! Without the debug cable:
//!   - Blue LED blinking: countdown, get ready. Solid: rotate the board.
//!     Off: collection finished.
//!   - Each successful fit is saved to flash. The next time this example boots
//!     (e.g. after reflashing with the probe attached) it prints the newest
//!     saved calibration first, before starting a new run.

#![no_std]
#![no_main]

use core::mem::MaybeUninit;
use core::ptr::addr_of_mut;

use defmt::{error, info, warn};
use embassy_executor::{task, Executor, Spawner};
use embassy_time::{Duration, Instant, Ticker, Timer};
use embedded_hal_1::spi::ErrorKind;
use nalgebra::{Matrix3, SMatrix, SVector, Vector3};
use {defmt_rtt as _, panic_probe as _};
use sirin::{spi::SpiDev, Sirin};
use w25qx::W25Q;

const HARD_IRON_BIAS: [f32; 3] = [-25.440907, 30.405136, -149.43695];

const COLLECT_SECONDS: u64 = 180;
const COUNTDOWN_SECONDS: u64 = 5;
const SAMPLE_PERIOD_MS: u64 = 100;
const MAX_SAMPLES: usize = 2400;
/// LIS3MDL sensitivity at ±4 gauss.
const LSB_PER_GAUSS: f32 = 6842.0;
/// 1 gauss = 100 µT.
const UT_PER_GAUSS: f32 = 100.0;
const MAX_SANE_UT: f32 = 300.0;

/// Bias-corrected samples. Static so ~14 KB doesn't sit on the task stack.
static mut SAMPLES: [[f32; 3]; MAX_SAMPLES] = [[0.0; 3]; MAX_SAMPLES];

/// Saved calibrations live in the last 4 KB sector of the 16 MB the flash
/// driver can address. That's inside the top 64 KB the flight log leaves free;
/// on ekf-implement the GPS nav DB also lives there but never grows this far.
/// One record per 256-byte page, appended in order, so the sector keeps the
/// last 16 runs. A record only counts if its checksum matches, so a run cut
/// off mid-write is ignored and the previous one is used.
const CAL_SECTOR_ADDR: u32 = 16 * 1024 * 1024 - 4096;
const CAL_PAGE_SIZE: u32 = 256;
const CAL_PAGES: u32 = 4096 / CAL_PAGE_SIZE;
const CAL_MAGIC: u32 = u32::from_le_bytes(*b"MCAL");
const CAL_WORDS: usize = 20;
const CAL_RECORD_LEN: usize = CAL_WORDS * 4;

const LED_BLINK_MS: u64 = 250;

/// One finished calibration, as saved to flash.
struct CalRecord {
    seq: u32,
    hard_iron_ut: [f32; 3],
    soft_iron: [[f32; 3]; 3],
    field_ut: f32,
    /// Spread of |B| as a % of its mean, with the old bias only and fully corrected
    spread_before_pct: f32,
    spread_after_pct: f32,
    samples: u32,
    rejected: u32,
}

impl CalRecord {
    /// Layout: magic, seq, hard iron (3), soft iron (9, row-major), F,
    /// spread before, spread after, samples, rejected, checksum.
    fn to_bytes(&self) -> [u8; CAL_RECORD_LEN] {
        let mut w = [0u32; CAL_WORDS];
        w[0] = CAL_MAGIC;
        w[1] = self.seq;
        for i in 0..3 {
            w[2 + i] = self.hard_iron_ut[i].to_bits();
        }
        for r in 0..3 {
            for c in 0..3 {
                w[5 + r * 3 + c] = self.soft_iron[r][c].to_bits();
            }
        }
        w[14] = self.field_ut.to_bits();
        w[15] = self.spread_before_pct.to_bits();
        w[16] = self.spread_after_pct.to_bits();
        w[17] = self.samples;
        w[18] = self.rejected;
        w[19] = fnv1a(&w[..19]);

        let mut b = [0u8; CAL_RECORD_LEN];
        for (i, word) in w.iter().enumerate() {
            b[i * 4..i * 4 + 4].copy_from_slice(&word.to_le_bytes());
        }
        b
    }

    fn from_bytes(b: &[u8; CAL_RECORD_LEN]) -> Option<Self> {
        let w: [u32; CAL_WORDS] = core::array::from_fn(|i| {
            u32::from_le_bytes([b[i * 4], b[i * 4 + 1], b[i * 4 + 2], b[i * 4 + 3]])
        });
        if w[0] != CAL_MAGIC || w[19] != fnv1a(&w[..19]) {
            return None;
        }
        Some(Self {
            seq: w[1],
            hard_iron_ut: core::array::from_fn(|i| f32::from_bits(w[2 + i])),
            soft_iron: core::array::from_fn(|r| core::array::from_fn(|c| f32::from_bits(w[5 + r * 3 + c]))),
            field_ut: f32::from_bits(w[14]),
            spread_before_pct: f32::from_bits(w[15]),
            spread_after_pct: f32::from_bits(w[16]),
            samples: w[17],
            rejected: w[18],
        })
    }

    fn print(&self) {
        info!(
            "Calibration #{}: {} samples ({} rejected), F = {} uT, |B| spread {}% before -> {}% after",
            self.seq, self.samples, self.rejected, self.field_ut, self.spread_before_pct, self.spread_after_pct,
        );
        info!("Paste into extended-kalman/src/lib.rs:");
        info!(
            "const MAG_HARD_IRON_UT: [f32; 3] = [{}, {}, {}];",
            self.hard_iron_ut[0], self.hard_iron_ut[1], self.hard_iron_ut[2],
        );
        info!("const MAG_SOFT_IRON: [[f32; 3]; 3] = [");
        for r in 0..3 {
            info!("    [{}, {}, {}],", self.soft_iron[r][0], self.soft_iron[r][1], self.soft_iron[r][2]);
        }
        info!("];");
    }
}

/// FNV-1a over the little-endian bytes of `words`.
fn fnv1a(words: &[u32]) -> u32 {
    let mut h: u32 = 0x811c_9dc5;
    for word in words {
        for byte in word.to_le_bytes() {
            h ^= byte as u32;
            h = h.wrapping_mul(0x0100_0193);
        }
    }
    h
}

struct CalScan {
    newest: Option<CalRecord>,
    /// First erased page, where the next record goes
    free_page: Option<u32>,
    valid: u32,
}

async fn scan_calibrations(flash: &mut W25Q<SpiDev>) -> Result<CalScan, ErrorKind> {
    let mut scan = CalScan { newest: None, free_page: None, valid: 0 };
    for page in 0..CAL_PAGES {
        let mut b = [0u8; CAL_RECORD_LEN];
        flash.read(CAL_SECTOR_ADDR + page * CAL_PAGE_SIZE, &mut b).await?;
        if b.iter().all(|&x| x == 0xFF) {
            scan.free_page.get_or_insert(page);
            continue;
        }
        if let Some(rec) = CalRecord::from_bytes(&b) {
            scan.valid += 1;
            if scan.newest.as_ref().map_or(true, |n| rec.seq > n.seq) {
                scan.newest = Some(rec);
            }
        }
    }
    Ok(scan)
}

/// Append `rec` after the newest saved record (erasing the sector once all 16
/// pages are used) and read it back. Returns the seq it was saved under.
async fn save_calibration(flash: &mut W25Q<SpiDev>, rec: &mut CalRecord) -> Result<Option<u32>, ErrorKind> {
    let scan = scan_calibrations(flash).await?;
    rec.seq = scan.newest.map_or(1, |n| n.seq.wrapping_add(1));

    let page = match scan.free_page {
        Some(p) => p,
        None => {
            flash.erase_sector(CAL_SECTOR_ADDR).await?;
            0
        }
    };
    let addr = CAL_SECTOR_ADDR + page * CAL_PAGE_SIZE;

    // A record fits in one page and starts page-aligned, so this is a single page program
    let bytes = rec.to_bytes();
    flash.write(addr, &bytes).await?;

    let mut back = [0u8; CAL_RECORD_LEN];
    flash.read(addr, &mut back).await?;
    Ok((back == bytes).then_some(rec.seq))
}

unsafe fn transmute_into_static<T>(item: &mut T) -> &'static mut T {
    core::mem::transmute(item)
}

#[cortex_m_rt::entry]
unsafe fn main() -> ! {
    let mut executor = Executor::new();
    let executor: &'static mut Executor = transmute_into_static(&mut executor);
    let mut sirin = MaybeUninit::<Sirin>::uninit();
    let sirin = transmute_into_static(&mut sirin);
    executor.run(|spawner| {
        spawner.must_spawn(setup_task(spawner, sirin))
    })
}

#[task()]
async fn setup_task(spawner: Spawner, sirin: &'static mut MaybeUninit<Sirin>) {
    let sirin = Sirin::init(sirin, spawner).await;
    // Only this task touches SAMPLES.
    let samples = unsafe { &mut *addr_of_mut!(SAMPLES) };
    main_task(sirin, samples).await;
}

/// Mean and standard deviation of |a · (s - offset)| over the samples.
fn magnitude_stats(samples: &[[f32; 3]], offset: &Vector3<f64>, a: &Matrix3<f64>) -> (f64, f64) {
    let n = samples.len() as f64;
    let mut sum = 0.0;
    let mut sum_sq = 0.0;
    for s in samples {
        let v = a * (Vector3::new(s[0] as f64, s[1] as f64, s[2] as f64) - offset);
        let m = v.norm();
        sum += m;
        sum_sq += m * m;
    }
    let mean = sum / n;
    let var = (sum_sq / n - mean * mean).max(0.0);
    (mean, libm::sqrt(var))
}

/// Fit a general ellipsoid (v - c)ᵀ M (v - c) = 1 to the samples.
/// Returns the soft-iron matrix A = F · sqrt(M), F (fitted field strength, µT),
/// and the centre c - the leftover hard-iron offset still in the samples.
fn fit_ellipsoid(samples: &[[f32; 3]]) -> Result<(Matrix3<f64>, f64, Vector3<f64>), &'static str> {
    // Shift the data so its mean is at the origin before fitting. The
    // vᵀQv + 2gᵀv = 1 form below is badly conditioned when the origin is far
    // outside the ellipsoid (a stale HARD_IRON_BIAS), and fails outright when
    // the origin is near its surface. With the origin at the data's centre it
    // works no matter what HARD_IRON_BIAS is. The shift is added back to c.
    let mut mean = Vector3::<f64>::zeros();
    for s in samples {
        mean += Vector3::new(s[0] as f64, s[1] as f64, s[2] as f64);
    }
    mean /= samples.len() as f64;

    // Solve d·p = 1 by least squares, where
    // d = [x², y², z², 2xy, 2xz, 2yz, 2x, 2y, 2z]
    // p = [Qxx, Qyy, Qzz, Qxy, Qxz, Qyz, gx, gy, gz]
    // i.e. vᵀ Q v + 2 gᵀ v = 1.
    let mut dtd = SMatrix::<f64, 9, 9>::zeros();
    let mut dt1 = SVector::<f64, 9>::zeros();
    for s in samples {
        let (x, y, z) = (s[0] as f64 - mean.x, s[1] as f64 - mean.y, s[2] as f64 - mean.z);
        let d = SVector::<f64, 9>::from_column_slice(&[
            x * x, y * y, z * z,
            2.0 * x * y, 2.0 * x * z, 2.0 * y * z,
            2.0 * x, 2.0 * y, 2.0 * z,
        ]);
        dtd += d * d.transpose();
        dt1 += d;
    }

    let p = dtd.lu().solve(&dt1).ok_or("least-squares system is singular (not enough rotation)")?;

    let q = Matrix3::new(
        p[0], p[3], p[4],
        p[3], p[1], p[5],
        p[4], p[5], p[2],
    );
    let g = Vector3::new(p[6], p[7], p[8]);

    // Complete the square: centre c = -Q⁻¹ g, and
    // (v - c)ᵀ Q (v - c) = 1 + cᵀ Q c, so M = Q / (1 + cᵀ Q c).
    let q_inv = q.try_inverse().ok_or("fit is degenerate (rotate through more orientations)")?;
    let c = -(q_inv * g);
    let k = 1.0 + (c.transpose() * q * c)[(0, 0)];
    if k <= 0.0 {
        return Err("fit is not an ellipsoid (rotate through more orientations)");
    }
    let m = q / k;

    // M = V Λ Vᵀ. Every eigenvalue must be positive for it to be an ellipsoid.
    let eig = m.symmetric_eigen();
    if eig.eigenvalues.iter().any(|&l| l <= 0.0) {
        return Err("fit is not an ellipsoid (rotate through more orientations, or check the hard-iron bias)");
    }

    // Ellipsoid semi-axes are 1/sqrt(λ). Field strength F = their geometric mean,
    // so the correction keeps readings in µT instead of normalising to 1.
    let l = eig.eigenvalues;
    let f = libm::cbrt(1.0 / libm::sqrt(l[0] * l[1] * l[2]));

    let sqrt_l = Matrix3::from_diagonal(&Vector3::new(libm::sqrt(l[0]), libm::sqrt(l[1]), libm::sqrt(l[2])));
    let v = eig.eigenvectors;
    let a = f * v * sqrt_l * v.transpose();

    Ok((a, f, c + mean))
}

async fn main_task(sirin: &'static mut Sirin, samples: &'static mut [[f32; 3]; MAX_SAMPLES]) {
    // Check the chip is actually answering. 0xFF means MISO is floating.
    match sirin.magnetometer.manufacturer_id().await {
        Ok(0x3D) => info!("LIS3MDL found (WHO_AM_I = 0x3D)"),
        Ok(id) => {
            error!("Unexpected magnetometer WHO_AM_I: {=u8:#x} (expected 0x3d). Stopping.", id);
            return;
        }
        Err(_) => {
            error!("SPI error reading magnetometer WHO_AM_I. Stopping.");
            return;
        }
    }

    // Report the last saved calibration before starting a new run
    match scan_calibrations(&mut sirin.flash.w25q).await {
        Ok(CalScan { newest: Some(rec), valid, .. }) => {
            info!("==== Last saved calibration ({} saved in flash) ====", valid);
            rec.print();
        }
        Ok(_) => info!("No saved calibration in flash yet"),
        Err(_) => error!("SPI error reading saved calibrations"),
    }

    if HARD_IRON_BIAS == [0.0, 0.0, 0.0] {
        warn!("HARD_IRON_BIAS is all zero - did you fill it in?");
    }

    // Blink blue during the countdown
    for i in (1..=COUNTDOWN_SECONDS).rev() {
        info!("Starting in {}... get away from metal", i);
        for _ in 0..(1000 / (2 * LED_BLINK_MS)) {
            sirin.led_blue.set_high();
            Timer::after_millis(LED_BLINK_MS).await;
            sirin.led_blue.set_low();
            Timer::after_millis(LED_BLINK_MS).await;
        }
    }
    // Solid blue while collecting: rotate now
    sirin.led_blue.set_high();
    info!("GO - rotate the board slowly through every orientation for {} s", COLLECT_SECONDS);

    let mut count: usize = 0;
    let mut rejected: u32 = 0;
    // Per-axis min/max of bias-corrected data, to check rotation coverage.
    let mut min = [f32::MAX; 3];
    let mut max = [f32::MIN; 3];

    let mut ticker = Ticker::every(Duration::from_millis(SAMPLE_PERIOD_MS));
    let start = Instant::now();
    let mut last_report = start;

    while Instant::now() - start < Duration::from_secs(COLLECT_SECONDS) && count < MAX_SAMPLES {
        ticker.next().await;

        let (x, y, z) = match sirin.magnetometer.magnetic().await {
            Ok(v) => v,
            Err(_) => {
                rejected += 1;
                continue;
            }
        };

        // All -1 means the chip didn't drive the bus this read.
        if (x, y, z) == (-1, -1, -1) {
            rejected += 1;
            continue;
        }

        let raw = [
            x as f32 / LSB_PER_GAUSS * UT_PER_GAUSS,
            y as f32 / LSB_PER_GAUSS * UT_PER_GAUSS,
            z as f32 / LSB_PER_GAUSS * UT_PER_GAUSS,
        ];

        if raw.iter().any(|v| v.abs() > MAX_SANE_UT) {
            rejected += 1;
            continue;
        }

        // Raw samples (store-mag.py parses this line).
        info!("Magnetometer: [{}, {}, {}]", raw[0], raw[1], raw[2]);

        let centred = [
            raw[0] - HARD_IRON_BIAS[0],
            raw[1] - HARD_IRON_BIAS[1],
            raw[2] - HARD_IRON_BIAS[2],
        ];
        for axis in 0..3 {
            min[axis] = min[axis].min(centred[axis]);
            max[axis] = max[axis].max(centred[axis]);
        }
        samples[count] = centred;
        count += 1;

        if Instant::now() - last_report >= Duration::from_secs(5) {
            last_report = Instant::now();
            info!("{} samples so far", count);
        }
    }

    sirin.led_blue.set_low();

    let samples = &samples[..count];
    info!("==== Collection done ====");
    info!("Samples used: {}, rejected: {}", count, rejected);

    if count < 100 {
        error!("Too few samples for a fit. Stopping.");
        return;
    }

    // Coverage: each axis should swing roughly ±F around zero. A lopsided
    // range means that axis wasn't pointed both ways, or the bias is off.
    for (axis, name) in ["x", "y", "z"].iter().enumerate() {
        info!("{}: min={} max={} (bias-corrected uT)", name, min[axis], max[axis]);
        let mid = (max[axis] + min[axis]) / 2.0;
        let half = (max[axis] - min[axis]) / 2.0;
        if mid.abs() > 0.2 * half {
            warn!("{} range isn't centred on 0 (midpoint {} uT) - hard-iron bias may be stale or that axis wasn't fully rotated", name, mid);
        }
    }

    let (a, f, offset) = match fit_ellipsoid(samples) {
        Ok(r) => r,
        Err(e) => {
            error!("Ellipsoid fit failed: {}", e);
            return;
        }
    };

    // Quality check: spread of |B| should shrink a lot after correction.
    // "Middle" shows how much comes from the hard-iron fix alone.
    let zero = Vector3::zeros();
    let identity = Matrix3::identity();
    let (mean_before, std_before) = magnitude_stats(samples, &zero, &identity);
    let (mean_mid, std_mid) = magnitude_stats(samples, &offset, &identity);
    let (mean_after, std_after) = magnitude_stats(samples, &offset, &a);
    info!("Fitted field strength F = {} uT", f);
    info!("|B| with your bias only:     mean={} std={} ({}%)", mean_before, std_before, 100.0 * std_before / mean_before);
    info!("|B| with refined bias:       mean={} std={} ({}%)", mean_mid, std_mid, 100.0 * std_mid / mean_mid);
    info!("|B| refined bias + soft-iron: mean={} std={} ({}%)", mean_after, std_after, 100.0 * std_after / mean_after);
    if std_after / mean_after > 0.05 {
        warn!("Corrected magnitude still varies by >5% - rerun with fuller rotation, away from metal.");
    }

    info!(
        "Your bias was off by: x={} y={} z={} uT",
        offset[0], offset[1], offset[2],
    );
    info!("Paste into your code (corrected = SOFT_IRON * (raw_uT - HARD_IRON_BIAS)):");
    info!(
        "const HARD_IRON_BIAS: [f32; 3] = [{}, {}, {}];",
        HARD_IRON_BIAS[0] + offset[0] as f32,
        HARD_IRON_BIAS[1] + offset[1] as f32,
        HARD_IRON_BIAS[2] + offset[2] as f32,
    );
    info!("const SOFT_IRON: [[f32; 3]; 3] = [");
    for r in 0..3 {
        info!("    [{}, {}, {}],", a[(r, 0)] as f32, a[(r, 1)] as f32, a[(r, 2)] as f32);
    }
    info!("];");

    let mut rec = CalRecord {
        seq: 0,
        hard_iron_ut: core::array::from_fn(|i| HARD_IRON_BIAS[i] + offset[i] as f32),
        soft_iron: core::array::from_fn(|r| core::array::from_fn(|c| a[(r, c)] as f32)),
        field_ut: f as f32,
        spread_before_pct: (100.0 * std_before / mean_before) as f32,
        spread_after_pct: (100.0 * std_after / mean_after) as f32,
        samples: count as u32,
        rejected,
    };
    match save_calibration(&mut sirin.flash.w25q, &mut rec).await {
        Ok(Some(seq)) => info!("Saved to flash as calibration #{}", seq),
        Ok(None) => error!("Calibration written to flash but read back wrong - not saved"),
        Err(_) => error!("SPI error saving calibration to flash"),
    }

    loop {
        ticker.next().await;
    }
}
