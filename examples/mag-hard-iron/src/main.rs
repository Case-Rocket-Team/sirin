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

#![no_std]
#![no_main]

use core::mem::MaybeUninit;
use core::ptr::addr_of_mut;

use defmt::{error, info, warn};
use embassy_executor::{task, Executor, Spawner};
use embassy_time::{Duration, Instant, Ticker};
use nalgebra::{Matrix3, SMatrix, SVector, Vector3};
use {defmt_rtt as _, panic_probe as _};
use sirin::Sirin;

const HARD_IRON_BIAS: [f32; 3] = [-74.704506, -169.0767, -71.9992];

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
    // Solve d·p = 1 by least squares, where
    // d = [x², y², z², 2xy, 2xz, 2yz, 2x, 2y, 2z]
    // p = [Qxx, Qyy, Qzz, Qxy, Qxz, Qyz, gx, gy, gz]
    // i.e. vᵀ Q v + 2 gᵀ v = 1.
    let mut dtd = SMatrix::<f64, 9, 9>::zeros();
    let mut dt1 = SVector::<f64, 9>::zeros();
    for s in samples {
        let (x, y, z) = (s[0] as f64, s[1] as f64, s[2] as f64);
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

    Ok((a, f, c))
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

    if HARD_IRON_BIAS == [0.0, 0.0, 0.0] {
        warn!("HARD_IRON_BIAS is all zero - did you fill it in?");
    }

    let mut ticker = Ticker::every(Duration::from_secs(1));
    for i in (1..=COUNTDOWN_SECONDS).rev() {
        info!("Starting in {}... get away from metal", i);
        ticker.next().await;
    }
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

    loop {
        ticker.next().await;
    }
}
