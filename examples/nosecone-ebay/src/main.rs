#![no_std]
#![no_main]
#![allow(unused_imports)]

use core::{f32::consts::PI, mem::{self, transmute_copy, MaybeUninit}, pin::Pin, sync::atomic::Ordering, u16};

use bmp3::{hal::{Bmp3RawData, ReadBmp3, RegErrReg, RegStatus}, Bmp3Readout};
use defmt::{debug, info, println, Debug2Format};
use embassy_executor::{task, Executor, Spawner};
use embassy_stm32::{bind_interrupts, dma::NoDma, gpio::{Level, Output, Speed}, peripherals::{self, DMA1_CH0, DMA1_CH1, PD8, PD9, USART3}, usart::{self, Config, Uart}};
use embassy_time::{Duration, Instant, Ticker, Timer, TICK_HZ};
use embedded_hal_1::spi::ErrorKind;
use postcard::take_from_bytes;
use rfm9::{ReadRfm9, Rfm9};
use w25qx::W25Q;
use {defmt_rtt as _, panic_probe as _};
use sirin::{Radio, Sirin, error::SirinError, flash::Flash, gps::{GPS_FIX, gps_task}, io::{FLASH_LOGGING_ENABLED, IN_CHANNEL, OUT_CHANNEL, broadcast, broadcast_log, flash_io_task, radio_io_task, send_packet, set_flash_logging_enabled, set_usb_broadcasting_enabled, try_receive_packet, usb_input_task, usb_output_task}, packet::{SirinDataState, GpsFixType, InPacket, IoChannel, IoPacket, Log, LogEntry, OutPacket, PacketError, Page, SirinData, SirinState}, song::{FromSong, SongSize}, spi::SpiDev, state::{Accel, AngularVel, ErrorState, Pos, Vel}, subsystems::measure_sirin, sync::Mutex, time::{duration_since_epoch, set_duration_since_epoch}, uunit::{Gs, Meters, MetersPerSecond2, MicroGs, WithUnits}};
use sirin::state::NominalState as SirinNominalState;
use sirin_filter::{Eskf, EskfConfig, InitialUncertainty, ImuNoise, FixedLagHistory, ImuSample, BarometerObservation, Vec3};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::{Channel, TrySendError}, pubsub::{PubSubBehavior, Publisher, Subscriber}};
use sirin_shared::{mode::SirinMode, physics::approx_pressure_altitude, time::AbsoluteTimeReference};
use sirin::song::SongDiscriminant;
use nalgebra as na;
use na::{Matrix3, Matrix6, Vector3, UnitQuaternion, Rotation3};

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

/// Initialize filter attitude from accelerometer + magnetometer
/// Falls back to accel-only if magnetometer is weak
fn initialize_from_sensors(
    accel_mps2: Vec3,
    mag_ut: Vec3,
) -> Result<na::UnitQuaternion<f32>, &'static str> {
    let accel_norm = accel_mps2.norm();
    if accel_norm < 1.0 {
        return Err("Acceleration magnitude too small");
    }
    let down = accel_mps2 / accel_norm;

    let mag_norm = mag_ut.norm();

    // Try to use magnetometer if strong enough
    if mag_norm >= 3000.0 {  // Relaxed threshold from 10000
        let mag_unit = mag_ut / mag_norm;
        let down_component = mag_unit.dot(&down);
        let mag_horizontal = mag_unit - down * down_component;
        let mag_horizontal_norm = mag_horizontal.norm();

        if mag_horizontal_norm > 0.05 {  // Relaxed from 0.1
            let north = mag_horizontal / mag_horizontal_norm;
            let up = -down;
            let east = north.cross(&up);

            let r_ned_to_body = na::Matrix3::from_columns(&[north, east, down]);
            let r_body_to_ned = r_ned_to_body.transpose();

            let rot_body_to_ned = na::Rotation3::from_matrix_unchecked(r_body_to_ned);
            let attitude = UnitQuaternion::from_rotation_matrix(&rot_body_to_ned);
            return Ok(attitude);
        }
    }

    // Fallback: accel-only initialization (level, heading north by default)
    // down vector defines pitch and roll; we assume heading is north (yaw = 0)
    let north = Vec3::new(1.0, 0.0, 0.0);  // North in NED frame
    let up = -down;
    let east = north.cross(&up);

    let r_ned_to_body = na::Matrix3::from_columns(&[north, east, down]);
    let r_body_to_ned = r_ned_to_body.transpose();

    let rot_body_to_ned = na::Rotation3::from_matrix_unchecked(r_body_to_ned);
    let attitude = UnitQuaternion::from_rotation_matrix(&rot_body_to_ned);

    info!("Using accel-only initialization (mag too weak: {} uT)", mag_norm as i32);
    Ok(attitude)
}

fn microgs_to_mps2(accel_microgs: Vec3) -> Vec3 {
    const GRAVITY: f32 = 9.80665;
    Vec3::new(
        accel_microgs.x * 1e-6 * GRAVITY,
        accel_microgs.y * 1e-6 * GRAVITY,
        accel_microgs.z * 1e-6 * GRAVITY,
    )
}

fn microdegps_to_radps(gyro_microdegps: Vec3) -> Vec3 {
    const DEG_TO_RAD: f32 = PI / 180.0;
    Vec3::new(
        gyro_microdegps.x * 1e-6 * DEG_TO_RAD,
        gyro_microdegps.y * 1e-6 * DEG_TO_RAD,
        gyro_microdegps.z * 1e-6 * DEG_TO_RAD,
    )
}

#[task()]
async fn setup_task(spawner: Spawner, sirin: &'static mut MaybeUninit<Sirin>) {
    debug!("Begin Sirin init");

    let sirin = Sirin::init(sirin, spawner).await;

    debug!("End Sirin init");

    match main_task(sirin).await {
        Ok(()) => {
            panic!("The main task ended! (It shouldn't do that)")
        },
        Err(e) => {
            panic!("The main task ran into an error: {:?}", e)
        }
    }
}

async fn main_task(sirin: &'static mut Sirin) -> Result<(), SirinError> {
    //println!("Time since epoch: {}", duration_since_epoch().unwrap());
    /*

    FOR IREC ROCKET - CHECK TO ENSURE THESE VALUES ARE CODED:
    DO NOT PUSH CODE WITH THESE VALUES SIGNIFICANTLY CHANGED
    accel_threshold = 10G * 10G
    altitude_threshold = 20m
    main_deployment_altitude = 457.2m (1500ft)
    flight_duration = 600s
    apogee_error = 1m
    timeout = 25s

     */

    let accel_threshold: Gs<f64> = (10.0 * 10.0).with_units(); //In Gs squared
    let altitude_threshold = 20.0; //In meters
    let main_deployment_altitude= 457.2; //In meters
    let flight_duration = 1000; //In seconds
    let apogee_error = 4.0; //In meters
    let timeout = 25; //In seconds

    let mut apo_deployed = false;
    let mut main_deployed = false;
    

    let mut state = SirinState::default();

    //For debugging purposes
    FLASH_LOGGING_ENABLED.store(false, Ordering::Relaxed);

    //Find the true barometric reading (bug causes large negative values upon initialization)
    let mut altitude_array: [f64; 100] = [0.0; 100];
    for i in 0..100 {
        let altitude = approx_pressure_altitude(sirin.baro.read().await?.pressure.convert());
        altitude_array[i] = altitude.value;
        Timer::after_millis(25).await;
    }
    
    altitude_array.sort_unstable_by(|a, b | a.partial_cmp(b).unwrap());
    let initial_altitude = altitude_array[50].with_units();
    info!("Initial altitude: {}", initial_altitude.value);

    //========================================================================
    // Initialize ESKF (Error-State Kalman Filter)
    //========================================================================
    let filter_config = EskfConfig {
        gravity_ned_mps2: Vec3::new(0.0, 0.0, 9.80665),
        min_imu_dt_s: 1.0e-6,
        max_imu_dt_s: 2.0,  // 2 seconds max (IMU available ~1s apart)
        max_measurement_age_us: 5_000_000,
        future_measurement_tolerance_us: 100_000,
        initial_uncertainty: InitialUncertainty {
            position_std_m: Vec3::new(100.0, 100.0, 100.0),
            velocity_std_mps: Vec3::repeat(20.0),
            attitude_std_rad: Vec3::repeat(0.1),  // Start more confident in attitude
            accel_bias_std_mps2: Vec3::repeat(0.5),
            gyro_bias_std_radps: Vec3::repeat(0.05),  // Higher initial gyro bias uncertainty
        },
        imu_noise: ImuNoise {
            accel_noise_density_mps2_sqrt_hz: Vec3::repeat(0.002158),
            gyro_noise_density_radps_sqrt_hz: Vec3::repeat(0.00006632),
            accel_bias_random_walk_mps3_sqrt_hz: Vec3::repeat(6.20e-6),
            gyro_bias_random_walk_radps2_sqrt_hz: Vec3::repeat(1.0e-6),  // Higher: allows faster bias learning
        },
        gps_position_variance_m2: Vec3::repeat(4.0),
        gps_velocity_variance_m2ps2: Vec3::repeat(0.25),
        gps_position_gate: 2.0,
        gps_velocity_gate: 2.0,
        barometer_gate: 1.5,     // Very tight - baro is very reliable
        magnetometer_gate: 2.0,  // Tight - mag should constrain yaw strongly
        magnetic_field_ned_ut: Vec3::new(19000.0, -2700.0, 48000.0),  // Cleveland, OH
    };

    let mut initial_attitude = UnitQuaternion::identity();
    let mut filter: Option<Eskf> = None;
    let mut history: Option<FixedLagHistory::<64>> = None;
    let mut current_time_us: u64 = 0;
    let mut filter_initialized = false;
    let mut last_imu_time_us: u64 = 0;
    let mut imu_sample_count: u32 = 0;  // Count actual samples sent

    // Bias calibration during first 5 seconds
    let mut gyro_bias_sum = Vec3::zeros();
    let mut accel_bias_sum = Vec3::zeros();
    let mut bias_sample_count: u32 = 0;
    let mut calibration_done = false;

    // Store measured biases for sensor correction
    let mut gyro_bias_radps = Vec3::zeros();
    let mut accel_bias_mps2 = Vec3::zeros();

    // Diagnostic counters
    let mut baro_count = 0;
    let mut mag_count = 0;
    let mut imu_attempted = 0;
    let mut imu_failed = 0;
    let mut last_diagnostic_time = 0u64;

    //Spawn background tasks
    sirin.spawner.spawn(radio_io_task(&sirin.config, &mut sirin.radio)).unwrap();
    sirin.spawner.spawn(usb_input_task(&mut sirin.usb.read_ep)).unwrap();
    sirin.spawner.spawn(usb_output_task(&mut sirin.usb.write_ep)).unwrap();
    sirin.spawner.spawn(gps_task(&mut sirin.gps_rx, &mut sirin.gps_tx)).unwrap();
    
    let mut flash = Mutex::new(&mut sirin.flash);
    sirin.spawner.spawn(flash_io_task(unsafe {
        transmute_into_static(&mut flash)
    })).unwrap();

    info!("Start main");

    //Update the loop every 50ms for faster IMU integration (20 Hz)
    let mut ticker = Ticker::every(Duration::from_millis(50));

    let mut launched_at = None;
    let mut dur: Option<Duration> = None;
    let mut max_altitude: Meters<f64> = 0.0.with_units();

    let mut desired_mode = None;

    let mut i = 0;

    loop {
        i += 1;
        //info!("Handle input packets");
        while let Ok(io_packet) = try_receive_packet() {
            info!("Received packet: {:?}", Debug2Format(&io_packet));
            match io_packet.packet {
                InPacket::DeployMain => {
                    sirin.parachute_main.set_high();
                }
                InPacket::DeployApo => {
                    sirin.parachute_apo.set_high();
                }
                InPacket::Null => {
                    continue;
                },
                InPacket::Ping => {}
                InPacket::SetTime(ref reference) => {
                    info!("SetTime packet received!");
                    if duration_since_epoch().is_some() {
                        continue;
                    }

                    // Subtract current uptime from time since boot
                    let ms_since_epoch = reference.ms_since_epoch - Instant::now().as_millis();

                    set_duration_since_epoch(Duration::from_millis(ms_since_epoch));
                    //set_flash_logging_enabled(true);
                    info!("Awaiting flash lock...");
                    let mut flash = flash.lock().await;
                    info!("Flash locked in main");
                    flash.set_absolute_time_reference(AbsoluteTimeReference { ms_since_epoch }).await?;
                    info!("Flash task complete");
                    //set_flash_logging_enabled(false);

                    // skip OK packet
                    continue;
                }
                InPacket::Reboot => {
                    Sirin::reboot();
                }
                InPacket::QueryConfig => {
                    send_packet(IoPacket::new(
                        io_packet.channel,
                        OutPacket::Config(sirin.config.clone())
                    ));
                }
                InPacket::SetConfig(ref config) => {
                    //info!("Updating the config to {:?}", Debug2Format(&config));

                    flash.lock().await.save_config(&config).await.unwrap();
                    Sirin::reboot();
                }
                InPacket::QueryMode => {
                    send_packet(io_packet.reply(OutPacket::Mode(state.mode)));
                }
                InPacket::SetMode(m) => {
                    desired_mode = Some(m);
                }
                InPacket::QueryFlights => {
                    //info!("QueryFlights packet received!");
                    let flash = flash.lock().await;

                    //info!("Querying flights...");

                    for (i, header) in flash.flight_headers.iter().enumerate() {
                        send_packet(io_packet.reply(OutPacket::FlightHeader(
                            Page::new(i as u16, header.header.clone())
                        )));
                    }
                }
                InPacket::ReadFlight(index) => {
                    let mut flash = flash.lock().await;
                    let Some(header) = flash.flight_headers.get(index as usize) else {
                        //TODO: Fix error
                        //send_packet(io_packet.reply(OutPacket::Error(PacketError::FlightNotFound(index))));
                        continue;
                    };

                    //info!("Reading flight with header: {:?}", Debug2Format(&header));

                    // borrow checker :(
                    let header = header.clone();

                    let mut iter = flash.read_logs(&header);
                    while let Some(log) = iter.next().await {
                        //info!("Sent log: {}", Debug2Format(&log));
                        send_packet(io_packet.reply(log?));
                    }

                    //info!("Done writing logs.")
                }
                InPacket::Tail(enabled) => {
                    set_usb_broadcasting_enabled(enabled);
                    continue;
                }
                InPacket::EraseFlash(..) => {
                    let mut flash = flash.lock().await;
                    info!("Starting chip erase...");
                    flash.w25q.chip_erase().await?;
                    info!("Waiting until flash is ready...");
                    flash.w25q.until_ready().await?;
                    info!("Finished chip erase.");
                    send_packet(io_packet.reply(OutPacket::Ok));

                    Timer::after_millis(500).await;

                    panic!("Reboot");
                }
            }

            send_packet(io_packet.reply(OutPacket::Ok));
        }

        ticker.next().await;

        //info!("Measure Sirin data");
        sirin.data = measure_sirin(
            &mut sirin.baro,
            &mut sirin.imu,
            &mut sirin.high_g_imu,
            &mut sirin.magnetometer
        ).await;

        // Initialize filter on first measurement
        if !filter_initialized {
            if let (Ok(accel), Ok(mag)) = (&sirin.data.imu.accel, &sirin.data.magnetometer.mag) {
                initial_attitude = initialize_from_sensors(
                    microgs_to_mps2(Vec3::new(
                        accel.x.value as f32,
                        accel.y.value as f32,
                        accel.z.value as f32,
                    )),
                    Vec3::new(
                        mag.x as f32,
                        mag.y as f32,
                        mag.z as f32,
                    ),
                ).unwrap_or_else(|e| {
                    info!("Attitude init error: {}, using identity", e);
                    UnitQuaternion::identity()
                });

                info!("Initial attitude: q=[{}, {}, {}, {}]",
                    initial_attitude.w, initial_attitude.i, initial_attitude.j, initial_attitude.k);

                // Create initial nominal state for ESKF with correct field names
                let initial_nominal_state = sirin_filter::NominalState {
                    pos: Vec3::zeros(),
                    vel: Vec3::zeros(),
                    rot_quaternion: initial_attitude.clone(),
                    accel_bias: Vec3::zeros(),
                    angular_vel_bias: Vec3::zeros(),
                    accel: Vec3::zeros(),
                };

                let initial_covariance = sirin_filter::CovarianceMatrixP::from_initial_uncertainty(
                    &filter_config.initial_uncertainty
                );

                filter = Some(Eskf::new(
                    initial_nominal_state,
                    initial_covariance,
                    filter_config.clone(),
                ));

                history = Some(FixedLagHistory::<64>::new(
                    filter.as_ref().unwrap().clone()
                ));

                filter_initialized = true;
                info!("ESKF initialized");

                // Initialize old state structure with attitude
                use sirin_shared::state::Quaternion;
                state.nominal.rot_quaternion = Quaternion::new(
                    initial_attitude.w,
                    initial_attitude.i,
                    initial_attitude.j,
                    initial_attitude.k,
                );
            }
        }

        //========================================================================
        // ESKF Propagation with IMU
        //========================================================================
        if filter_initialized {
            if let Some(ref mut f) = filter {
                // Update timestamp - ensure monotonic and reasonable
                if let Some(epoch) = duration_since_epoch() {
                    let epoch_us = epoch.as_millis() as u64 * 1000;
                    // Only update if it's newer than last time
                    if epoch_us > current_time_us {
                        current_time_us = epoch_us;
                    } else {
                        current_time_us = current_time_us.saturating_add(100_000);
                    }
                } else {
                    // No real time available - estimate with fixed step
                    current_time_us = current_time_us.saturating_add(100_000);
                }

                // Calibration phase: collect IMU biases for first ~5 seconds
                if !calibration_done {
                    if let (Ok(accel), Ok(gyro)) = (&sirin.data.imu.accel, &sirin.data.imu.angular_vel) {
                        // Accumulate raw sensor values
                        gyro_bias_sum.x += gyro.x.value as f32;
                        gyro_bias_sum.y += gyro.y.value as f32;
                        gyro_bias_sum.z += gyro.z.value as f32;

                        accel_bias_sum.x += accel.x.value as f32;
                        accel_bias_sum.y += accel.y.value as f32;
                        accel_bias_sum.z += accel.z.value as f32;
                        bias_sample_count += 1;

                        // Complete calibration after ~50 samples (~5 seconds)
                        if bias_sample_count >= 50 {
                            let gyro_bias_raw = gyro_bias_sum / (bias_sample_count as f32);
                            let accel_bias_raw = accel_bias_sum / (bias_sample_count as f32);

                            gyro_bias_radps = microdegps_to_radps(Vec3::new(
                                gyro_bias_raw.x,
                                gyro_bias_raw.y,
                                gyro_bias_raw.z,
                            ));

                            accel_bias_mps2 = microgs_to_mps2(Vec3::new(
                                accel_bias_raw.x,
                                accel_bias_raw.y,
                                accel_bias_raw.z,
                            )) - Vec3::new(0.0, 0.0, 9.80665);  // Remove gravity from Z

                            info!("Calibration complete!");
                            info!("Gyro bias (rad/s): x={} y={} z={}",
                                gyro_bias_radps.x, gyro_bias_radps.y, gyro_bias_radps.z);
                            info!("Accel bias (m/s2): x={} y={} z={}",
                                accel_bias_mps2.x, accel_bias_mps2.y, accel_bias_mps2.z);

                            calibration_done = true;
                        }
                    }
                    // Continue to next iteration during calibration
                } else {

                // Check IMU availability
                let accel_ok = sirin.data.imu.accel.is_ok();
                let gyro_ok = sirin.data.imu.angular_vel.is_ok();

                // Log diagnostics once per second
                if i % 10 == 0 {
                    info!("Quat: w={} x={} y={} z={}",
                        state.nominal.rot_quaternion.r as i32,
                        state.nominal.rot_quaternion.x as i32,
                        state.nominal.rot_quaternion.y as i32,
                        state.nominal.rot_quaternion.z as i32
                    );
                }

                // Process IMU if available
                if let (Ok(accel), Ok(gyro)) = (&sirin.data.imu.accel, &sirin.data.imu.angular_vel) {
                    imu_attempted += 1;

                    // Convert sensor readings
                    let accel_raw = microgs_to_mps2(Vec3::new(
                        accel.x.value as f32,
                        accel.y.value as f32,
                        accel.z.value as f32,
                    ));

                    // Only correct gyro bias (accel bias mixed with gravity)
                    let gyro_corrected = microdegps_to_radps(Vec3::new(
                        gyro.x.value as f32,
                        gyro.y.value as f32,
                        gyro.z.value as f32,
                    )) - gyro_bias_radps;

                    let imu_sample = ImuSample {
                        timestamp_us: current_time_us,
                        accel_mps2_b: accel_raw,
                        gyro_radps_b: gyro_corrected,
                        temperature_c: None,
                        accel_saturated: [false; 3],
                        gyro_saturated: [false; 3],
                        sequence: imu_sample_count,
                    };

                    match f.propagate_imu(imu_sample) {
                        Ok(()) => {
                            imu_sample_count = imu_sample_count.wrapping_add(1);

                            // IMU propagation succeeded - update state with estimates
                            let nav_solution = f.navigation_solution();

                            // Log quaternion once per second
                            if i % 10 == 0 {
                                info!("Quat: w={} x={} y={} z={}",
                                    nav_solution.attitude_nb_wxyz[0] as i32,
                                    nav_solution.attitude_nb_wxyz[1] as i32,
                                    nav_solution.attitude_nb_wxyz[2] as i32,
                                    nav_solution.attitude_nb_wxyz[3] as i32
                                );
                            }

                            // Update quaternion from filter attitude estimate
                            use sirin_shared::state::Quaternion;
                            state.nominal.rot_quaternion = Quaternion::new(
                                nav_solution.attitude_nb_wxyz[0],  // w
                                nav_solution.attitude_nb_wxyz[1],  // x (i)
                                nav_solution.attitude_nb_wxyz[2],  // y (j)
                                nav_solution.attitude_nb_wxyz[3],  // z (k)
                            );
                        },
                        Err(e) => {
                            imu_failed += 1;
                            if i % 10 == 0 {
                                info!("IMU propagation error (seq={}): {:?}", imu_sample_count, Debug2Format(&e));
                            }
                        }
                    }
                } else {
                    if i % 10 == 0 {
                        info!("IMU data not available");
                    }
                }
            }
        } else {
            if i % 10 == 0 {
                info!("Filter not initialized yet");
            }
        }

        //========================================================================
        // Barometer Altitude Fusion
        //========================================================================
        if filter_initialized {
            if let Some(ref mut f) = filter {
                if let Ok(pressure) = sirin.data.baro.pressure {
                    // Convert pressure to height above launch site
                    let measured_altitude = approx_pressure_altitude(pressure.convert());
                    let height_up_m = (measured_altitude - initial_altitude).value as f32;

                    let baro_obs = BarometerObservation {
                        timestamp_us: current_time_us,
                        height_up_m,
                        variance_m2: 0.04,  // ~0.2m std dev (tighter)
                    };

                    let _ = f.fuse_barometer(&baro_obs);
                    baro_count += 1;
                    if i % 10 == 0 {
                        info!("Baro: h={} m", height_up_m as i32);
                    }
                }

                //====================================================================
                // Magnetometer Heading Fusion
                //====================================================================
                if let Ok(mag) = &sirin.data.magnetometer.mag {
                    let mag_obs = sirin_filter::MagnetometerObservation {
                        timestamp_us: current_time_us,
                        field_ut_b: Vec3::new(
                            mag.x as f32,
                            mag.y as f32,
                            mag.z as f32,
                        ),
                        variance_ut2: Vec3::repeat(25.0),  // ~5 μT std dev per axis (tighter)
                    };

                    let _ = f.fuse_magnetometer(&mag_obs);
                    mag_count += 1;
                    if i % 10 == 0 {
                        info!("Mag: x={} y={} z={} uT", mag.x, mag.y, mag.z);
                    }
                }

                // Diagnostics: Print measurement frequency every ~10 seconds
                if current_time_us > last_diagnostic_time + 10_000_000 {
                    info!("Diagnostics: IMU attempted={} success={} failed={}, Baro={} Mag={}",
                        imu_attempted, imu_sample_count, imu_failed, baro_count, mag_count);
                    imu_attempted = 0;
                    imu_failed = 0;
                    baro_count = 0;
                    mag_count = 0;
                    last_diagnostic_time = current_time_us;
                }
                }  // End of else (after calibration)
            }
        }

        //Calculate DataState
        let datastate = SirinDataState{
            data: sirin.data.clone(),
            mode: state.mode.clone(),
            altitude: state.altitude.clone(),
            apogee: state.apogee.clone(),
            gps_fix: state.gps_fix.clone(),
            pos: state.nominal.pos.clone(),
            vel: state.nominal.vel.clone(),
            rot_quaternion: state.nominal.rot_quaternion.clone()
        };

        //info!("Calculate barometric altitude");
        if let Ok(pressure) = sirin.data.baro.pressure {
            let measured_altitude = approx_pressure_altitude(pressure.convert());
            state.altitude = measured_altitude - initial_altitude;
            if state.altitude.value > max_altitude.value {
                max_altitude = state.altitude;
            }
        }
        //info!("Altitude calculated");

        let accel_mag_squared = if let Ok(accel) = &sirin.data.imu.accel {
            let x_f64: MicroGs<f64> = (accel.x.value as f64).with_units();
            let x: Gs<f64> = x_f64.convert();
            let y_f64: MicroGs<f64> = (accel.y.value as f64).with_units();
            let y: Gs<f64> = y_f64.convert();
            let z_f64: MicroGs<f64> = (accel.z.value as f64).with_units();
            let z: Gs<f64> = z_f64.convert();
            Some(x * x + y * y + z * z)
        } else {
            None
        };

        match state.mode {
            SirinMode::Standby => {
                if state.altitude.value > altitude_threshold
                    || accel_mag_squared.is_some_and(|accel| accel.value > accel_threshold.value)
                    || desired_mode == Some(SirinMode::Flight)
                {
                    sirin.led.set_high();
                    state.mode = SirinMode::Flight;
                    info!("Entered flight mode...");
                    FLASH_LOGGING_ENABLED.store(true, Ordering::Relaxed);
                    launched_at = Some(Instant::now());
                }
            },
            SirinMode::Flight => {
                //Log a DataState packet every 100 milliseconds
                OUT_CHANNEL.publish_immediate(IoPacket::new(
                    IoChannel::Flash, OutPacket::LogEntry(
                        LogEntry::new(
                            sirin.data.time,
                            Log::DataState(datastate.clone())
                        )
                    )
                ));

                /*OUT_CHANNEL.publish_immediate(IoPacket::new(
                    IoChannel::Flash, OutPacket::LogEntry(
                        LogEntry::new(
                            sirin.data.time,
                            Log::State(state.clone())
                        )
                    )
                ));*/

                //Check apogee, deploy apo parachute
                if let None = state.apogee {
                    if max_altitude.value > state.altitude.value + apogee_error{
                        state.apogee = Some(max_altitude);
                        match dur{
                            Some(duration) => {
                                if duration > Duration::from_secs(timeout){
                                    if !apo_deployed{
                                        //Timer::after_millis(1000).await;
                                        Sirin::deploy_chute_apo(&mut sirin.parachute_apo);
                                        OUT_CHANNEL.publish_immediate(IoPacket::new(
                                        IoChannel::Flash, 
                                        OutPacket::DeployedApoAt(sirin.data.time.value)
                                        ));
                                        apo_deployed = true;
                                    }
                                }
                            },
                            None => {}
                        }
                    }
                }

                //Deploy main parachute
                if state.apogee.is_some() {
                    if state.altitude.value < main_deployment_altitude {
                        if !main_deployed {
                            Sirin::deploy_chute_main(&mut sirin.parachute_main);
                            OUT_CHANNEL.publish_immediate(IoPacket::new(
                    IoChannel::Flash, OutPacket::DeployedMainAt(sirin.data.time.value)
                            ));
                            main_deployed = true;
                        }
                    }
                }

                //Timeout after designated time
                if let Some(launched_at) = launched_at {
                    dur = Some(Instant::now() - launched_at);
                    if dur.unwrap() > Duration::from_secs(flight_duration) {
                        desired_mode = Some(SirinMode::Landed);
                        info!("Exiting flight mode...");
                    }
                }

                if desired_mode == Some(SirinMode::Landed) {
                    sirin.led.set_low();
                    state.mode = SirinMode::Landed;
                    FLASH_LOGGING_ENABLED.store(false, Ordering::Relaxed);
                }
            },
            SirinMode::Landed => {
                
            }
        }

        if let Some(mode) = desired_mode {
            state.mode = mode;
            desired_mode = None;
        }

        //info!("Broadcast");
        //TODO: This might be double logging to flash during flight
        //broadcast_log(sirin.data.time, Log::Data(sirin.data.clone()));
        //broadcast_log(sirin.data.time, Log::State(state.clone()));

        //TODO: Make State now exceed the MAX_OUTPACKET_SIZE
        OUT_CHANNEL.publish_immediate(IoPacket::new(
            IoChannel::Usb, OutPacket::LogEntry(LogEntry::new(
                sirin.data.time,
                Log::DataState(datastate.clone())
            ))
        ));

        /* 
        OUT_CHANNEL.publish_immediate(IoPacket::new(
            IoChannel::Usb, OutPacket::LogEntry(LogEntry::new(
                sirin.data.time,
                Log::Data(sirin.data.clone())
            ))
        ));
        */

        //info!("Try get GPS fix");
        if let Some(fix) = GPS_FIX.try_take() {
            //if fix.fix_type != GpsFixType::NoFix {
                state.gps_fix = fix;
            //}
        }

        //Alternates sending State and Data packets every half second
        //buffer overflow error, this is me trying to mitigate it since there is no time to 
        //info!("Transmit data");
        if i % 5 == 0  {
            OUT_CHANNEL.publish_immediate(IoPacket::new(
                IoChannel::ToLoRa, OutPacket::LogEntry(LogEntry::new(
                    sirin.data.time,
                    Log::DataState(datastate)
                ))
            ));
            //println!("State broadcasted!");
        }


        /* Uncomment if you ever want to broadcast raw data over LoRa for whatever reason
        if i % 5 == 0{
            OUT_CHANNEL.publish_immediate(IoPacket::new(
                IoChannel::ToLoRa, OutPacket::LogEntry(LogEntry::new(
                    sirin.data.time,
                    Log::Data(sirin.data.clone())
                ))
            ));
            //println!("Data broadcasted!");
        }
        */


        //info!("Final state altitude: {}", state.altitude.value);
        //info!("Done with GPS");
    }
}