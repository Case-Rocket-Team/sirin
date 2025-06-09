#![no_std]
#![no_main]
#![allow(unused_imports)]
#![allow(non_upper_case_globals)]

use core::{f32, f64::consts::PI, mem::{self, transmute_copy, MaybeUninit}, pin::Pin, u16};

use bmp3::{hal::{Bmp3RawData, ReadBmp3, RegErrReg, RegStatus}, Bmp3Readout};
use defmt::{debug, info, println, Debug2Format};
use embassy_executor::{task, Executor, Spawner};
use embassy_stm32::{bind_interrupts, dma::NoDma, gpio::{Level, Output, Speed}, pac::timer, peripherals::{self, DMA1_CH0, DMA1_CH1, PD8, PD9, USART3}, time, usart::{self, Config, Uart}};
use embassy_time::{Duration, Instant, Timer};
use embedded_hal_1::spi::ErrorKind;
use postcard::take_from_bytes;
use rfm9::{ReadRfm9, Rfm9};
use w25qx::W25Q;
use {defmt_rtt as _, panic_probe as _};
use sirin::{error::SirinError, flash::Flash, io::{broadcast, flash_io_task, radio_io_task, send_packet, set_usb_broadcasting_enabled, try_receive_packet, usb_input_task, usb_output_task, IN_CHANNEL}, packet::{InPacket, IoChannel, IoPacket, Log, LogEntry, OutPacket, PacketError, Page}, song::{FromSong, SongSize}, spi::SpiDev, state::{Accel, EcefPos, State, Vel}, subsystems::SirinData, sync::Mutex, time::{duration_since_epoch, set_duration_since_epoch}, uunit::WithUnits, Radio, Sirin};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::{Channel, TrySendError}, pubsub::{Publisher, Subscriber}};
use sirin_shared::{mode::SirinMode, time::AbsoluteTimeReference};
use sirin::song::SongDiscriminant;

unsafe fn transmute_into_static<T>(item: &mut T) -> &'static mut T {
    core::mem::transmute(item)
}

// constants
const MACH_CUTOFF: f64 = 1.0;
const PID_DT_MS: f64 = 1.0;
const MAX_PWM: f64 = 1.0;
const Kp: f64 = 1.0;
const Kd: f64 = 1.0;
const Ki: f64 = 1.0;

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

bind_interrupts!(struct Irqs {
    USART3 => usart::InterruptHandler<peripherals::USART3>;
});

async fn main_task(sirin: &'static mut Sirin) -> Result<(), SirinError> {
    let mut i: u32 = 0;

    //sirin.spawner.spawn(radio_io_task(&mut sirin.radio)).unwrap();
    sirin.spawner.spawn(usb_input_task(&mut sirin.usb.read_ep)).unwrap();
    sirin.spawner.spawn(usb_output_task(&mut sirin.usb.write_ep)).unwrap();

    let mut state: State;
    let mut mode = SirinMode::Standby;

    let mut pid_vars = PIDvars {
        launched: false,
        past_burnout: false,
        control_enabled: false,
        last_pid_time: 0,
        integral: 0.0,
        last_error: 0.0,
    };

    let mut flash = Mutex::new(&mut sirin.flash);
    sirin.spawner.spawn(flash_io_task(unsafe {
        transmute_into_static(&mut flash)
    })).unwrap();

    loop {
        while let Ok(io_packet) = try_receive_packet() {
            info!("Received packet: {:?}", Debug2Format(&io_packet));
            match io_packet.packet {
                InPacket::Null => {
                    continue;
                },
                InPacket::Ping => {}
                InPacket::SetTime(ref reference) => {
                    if duration_since_epoch().is_some() {
                        continue;
                    }

                    // Subtract current uptime from time since boot
                    let ms_since_epoch = reference.ms_since_epoch - Instant::now().as_millis();

                    set_duration_since_epoch(Duration::from_millis(ms_since_epoch));
                    flash.lock().await.set_absolute_time_reference(AbsoluteTimeReference { ms_since_epoch }).await?;

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
                    info!("Updating the config to {:?}", Debug2Format(&config));

                    flash.lock().await.save_config(&config).await.unwrap();
                    Sirin::reboot();
                }
                InPacket::QueryMode => {
                    send_packet(io_packet.reply(OutPacket::Mode(mode)));
                }
                InPacket::SetMode(m) => {
                    mode = m;
                }
                InPacket::QueryFlights => {
                    let flash = flash.lock().await;

                    info!("Querying flights...");

                    for (i, header) in flash.flight_headers.iter().enumerate() {
                        send_packet(io_packet.reply(OutPacket::FlightHeader(
                            Page::new(i as u16, header.header.clone())
                        )));
                    }
                }
                InPacket::ReadFlight(index) => {
                    let mut flash = flash.lock().await;
                    let Some(header) = flash.flight_headers.get(index as usize) else {
                        send_packet(io_packet.reply(OutPacket::Error(PacketError::FlightNotFound(index))));
                        continue;
                    };

                    info!("Reading flight with header: {:?}", Debug2Format(&header));

                    // borrow checker :(
                    let header = header.clone();

                    let mut iter = flash.read_logs(&header);
                    while let Some(log) = iter.next().await {
                        info!("Sent log: {}", Debug2Format(&log));
                        send_packet(io_packet.reply(log?));
                    }

                    info!("Done writing logs.")
                }
                InPacket::Tail(enabled) => {
                    set_usb_broadcasting_enabled(enabled);
                    continue;
                }
                InPacket::EraseFlash(..) => {
                    let mut flash = flash.lock().await;
                    info!("Starting chip erase...");
                    flash.w25q.chip_erase().await?;
                    flash.w25q.until_ready().await?;
                    info!("Finished chip erase.");
                    send_packet(io_packet.reply(OutPacket::Ok));

                    Timer::after_millis(500).await;
                    panic!("Reboot");
                }
            }

            send_packet(io_packet.reply(OutPacket::Ok));
        }

        Timer::after_millis(100).await;

        if mode == SirinMode::Flight {
            sirin.led.set_high();
        }

        sirin.data = SirinData::measure(
            &mut sirin.baro,
            &mut sirin.imu,
            &mut sirin.high_g_imu
        ).await;

        // TODO: Replace with call to C code
        unsafe {
            let mut _state: MaybeUninit<State> = MaybeUninit::uninit();
            run_kalman_filter(&mut _state, &sirin.data);
            state = _state.assume_init();
        }

        main_airbrakes(&state, &mut pid_vars).await;

        if i % 10 == 0 {
            // Do logging
            broadcast(OutPacket::LogEntry(LogEntry::new(
                Instant::now().as_millis() as u32,
                Log::State(state)
            )));
        }

        i = i.wrapping_add(1);

        sirin.led.set_low();
    }
}

unsafe fn run_kalman_filter(state: *mut MaybeUninit<State>, data: *const SirinData) {
    let accel = (*data).imu.accel.as_ref().unwrap();

    state.write(MaybeUninit::new(State {
        pos: EcefPos {
            x: 0.0.with_units(),
            y: 0.0.with_units(),
            z: 0.0.with_units(),
        },
        vel: Vel {
            x: 0.0.with_units(),
            y: 0.0.with_units(),
            z: 0.0.with_units(),
        },
        accel: Accel {
            x: (accel.x.value as f64).with_units(),
            y: (accel.y.value as f64).with_units(),
            z: (accel.z.value as f64).with_units(),
        },
        altitude: 0.0.with_units()
    }));
}


// TODO: airbreaks
/*#[task]
async fn kalman(
    mut event_sub: Subscriber<'static, CriticalSectionRawMutex, Event, 100, 4, 4>
) {
    loop {
        let event = event_sub.next_message_pure().await;

        match event {
            Event::Measurement(measurement) => {
                match measurement {
                    Measurement::Baro(bmp3_readout) => todo!(),
                    Measurement::ImuAccel(accel) => todo!(),
                    Measurement::ImuAngularVel(angular_vel) => todo!(),
                }
            },
        }

        // Example: call a C function from sirin-c Rust crate
        // Edit sirin-c crate and c project to add more functions
        sirin_c::cmsis_dsp_sin(f32::consts::PI / 2.0);
    }
}*/


// void main_loop() {
//     // Get current time
//     unsigned long now = millis();
//     // Get state estimates
//     float altitude = get_altitude();
//     float vertical_velocity = get_vertical_velocity();
//     float mach = get_mach();
//     // Detect launch (idk if Sirin has this built in, if so, use Sirin launch detection)
//     if (!launched && mach > 0.1) {
//       launched = true;
//     }
//     // Detect burnout and Mach lockout
//     if (launched && !past_burnout && mach < MACH_CUTOFF) {
//       past_burnout = true;
//       control_enabled = true;
//       last_pid_time = now; // reset PID loop timing
//     }
//     // Run PID loop
//     if (control_enabled && (now - last_pid_time >= PID_DT_MS)) {
//       last_pid_time = now;
//       // Compute target position from altitude and velocity
//       int target_position = compute_target_encoder(altitude, vertical_velocity);
//       // Read encoder position
//       int current_position = read_encoder_position();
//       // PID calcs
//       float error = (float)(target_position - current_position);
//       integral += error * (PID_DT_MS / 1000.0);
//       float derivative = (error - last_error) / (PID_DT_MS / 1000.0);
//       last_error = error;
//       // Compute motor command
//       float output = Kp * error + Ki * integral + Kd * derivative;
//       // Clamp output to limits
//       if (output > MAX_PWM) output = MAX_PWM;
//       if (output < -MAX_PWM) output = -MAX_PWM;
//       // Set motor
//       set_motor_pwm((int)output); // -255 to +255
//     }
//   }
//   // Maps altitude & velocity to encoder target 
//   int compute_target_encoder(float altitude, float vertical_velocity) {
//     float result = 100.0 - (altitude / 100.0) - (vertical_velocity * 0.5); //will be changed later based on RIPTIDE result
//     if (result < 0) result = 0;
//     if (result > 255) result = 255;
//     return (int)result;
//   }
#[derive(Debug, Clone)]
struct PIDvars {
    launched: bool,
    past_burnout: bool,
    control_enabled: bool,
    last_pid_time: u64,
    integral: f64,
    last_error: f64,
}

async fn main_airbrakes(current_state: *const State, pid_vars: *mut PIDvars) {
    
    let now = Instant::now().as_millis();
    let state: State;
    let mut vars: PIDvars;
    unsafe {
        state = (*current_state).clone();
        vars = (*pid_vars).clone();
    }
    
    let altitude = state.altitude;
    // make sure you are using the right x y or z here
    let vertical_velocity = state.vel.z;
    // todo
    let mach = 1.0;


    if !vars.launched && mach > 0.1 {
        vars.launched = true;
    }
    if vars.launched && !vars.past_burnout && mach < MACH_CUTOFF {
        vars.past_burnout = true;
        vars.control_enabled = true;
        vars.last_pid_time = now; // reset PID loop timing
    }
    if vars.control_enabled && (now - vars.last_pid_time >= PID_DT_MS as u64) {
        vars.last_pid_time = now;
        // Compute target position from altitude and velocity
        let target_position: i32 = compute_target_encoder(altitude.value, vertical_velocity.value).await.unwrap();
        // Read encoder position (TODO)
        let current_position: i32 = 1;
        // PID calcs
        let error: f64 = (target_position - current_position) as f64;
        vars.integral += error * (PID_DT_MS / 1000.0);
        let derivative: f64 = (error - vars.last_error) / (PID_DT_MS / 1000.0);
        vars.last_error = error;
        // Compute motor command
        let mut output: f64 = Kp * error + Ki * vars.integral + Kd * derivative;
        // Clamp output to limits
        if output > MAX_PWM {output = MAX_PWM;}
        if output < -MAX_PWM { output = -MAX_PWM;}
        // Set motor TODO
        // set_motor_pwm(output as i32); // -255 to +255
    }
}

async fn compute_target_encoder(altitude: f64, vertical_velocity: f64) -> Result<i32, SirinError> {
    let mut result: f64 = 100.0 - (altitude / 100.0) - (vertical_velocity * 0.5);
    if result < 0.0 {result = 0.0;}
    if result > 255.0 {result = 255.0;}
    Ok(result as i32)
}  