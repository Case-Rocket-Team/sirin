#![no_std]
#![no_main]
#![allow(unused_imports)]

use core::{f32::consts::PI, mem::{self, transmute_copy, MaybeUninit}, pin::Pin, u16};

use bmp3::{hal::{Bmp3RawData, ReadBmp3, RegErrReg, RegStatus}, Bmp3Readout};
use defmt::{debug, info, println, Debug2Format};
use embassy_executor::{task, Executor, Spawner};
use embassy_stm32::{bind_interrupts, dma::NoDma, gpio::{Level, Output, Speed}, peripherals::{self, DMA1_CH0, DMA1_CH1, PD8, PD9, USART3}, usart::{self, Config, Uart}};
use embassy_time::{Duration, Instant, Timer, TICK_HZ};
use embedded_hal_1::spi::ErrorKind;
use postcard::take_from_bytes;
use rfm9::{ReadRfm9, Rfm9};
use sirin_c::update_with_imu;
use w25qx::W25Q;
use {defmt_rtt as _, panic_probe as _};
use sirin::{error::SirinError, flash::Flash, gps::gps_task, io::{broadcast, broadcast_log, flash_io_task, radio_io_task, send_packet, set_usb_broadcasting_enabled, try_receive_packet, usb_input_task, usb_output_task, IN_CHANNEL}, packet::{InPacket, IoChannel, IoPacket, Log, LogEntry, OutPacket, PacketError, Page}, song::{FromSong, SongSize}, spi::SpiDev, state::{Accel, AngularVel, ErrorState, NominalState, Pos, Vel}, subsystems::SirinData, sync::Mutex, time::{duration_since_epoch, set_duration_since_epoch}, uunit::{Gs, MetersPerSecond2, WithUnits}, Radio, Sirin};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::{Channel, TrySendError}, pubsub::{Publisher, Subscriber}};
use sirin_shared::{mode::SirinMode, physics::approx_pressure_altitude, time::AbsoluteTimeReference};
use sirin::song::SongDiscriminant;

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
    let mut i: u32 = 0;
    let mut last_measurement_time: Option<Instant> = None;

    sirin.spawner.spawn(radio_io_task(&sirin.config, &mut sirin.radio)).unwrap();
    sirin.spawner.spawn(usb_input_task(&mut sirin.usb.read_ep)).unwrap();
    sirin.spawner.spawn(usb_output_task(&mut sirin.usb.write_ep)).unwrap();
    sirin.spawner.spawn(gps_task(&mut sirin.gps)).unwrap();

    let mut nominal = NominalState::default();
    let mut error = ErrorState::default();

    let mut mode = SirinMode::Standby;

    let initial_altitude = approx_pressure_altitude(sirin.baro.read().await?.pressure.convert());

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

        if let Ok(pressure) = sirin.data.baro.pressure {
            let measured_altitude = approx_pressure_altitude(pressure.convert());

            if i % 10 == 0 {
                broadcast_log(sirin.data.time, Log::BarometricAltitude(measured_altitude - initial_altitude));
            }
        }

        let curr = Instant::now();

        if let Some(prev) = last_measurement_time {
            let dt = (curr - prev).as_ticks() as f32 / TICK_HZ as f32;

            if let Ok(ref accel) = sirin.data.imu.accel {
                if let Ok(ref angular_vel) = sirin.data.imu.angular_vel {
                    // TODO: figure out better unit conversions

                    const GRAVITY: f32 = 10.0;
                    let accel_x = accel.x.value as f32 * GRAVITY / 1_000_000.0;
                    let accel_y = accel.y.value as f32 * GRAVITY / 1_000_000.0;
                    let accel_z = accel.z.value as f32 * GRAVITY / 1_000_000.0;
                    let accel = [accel_x, accel_y, accel_z];

                    const DEG_TO_RAD: f32 = PI / 180.0;
                    let x_pitch = angular_vel.x_pitch.value as f32 * DEG_TO_RAD / 1_000_000.0;
                    let y_roll = angular_vel.y_roll.value as f32 * DEG_TO_RAD / 1_000_000.0;
                    let z_yaw = angular_vel.z_yaw.value as f32 * DEG_TO_RAD / 1_000_000.0;
                    let ang_vel = [x_pitch, y_roll, z_yaw];
                    
                    unsafe {
                        update_with_imu(
                            &mut nominal,
                            &mut error,
                            dt.with_units(),
                            &accel[0] as *const f32,
                            &ang_vel[0] as *const f32
                        );
                    }
                }
            }
        }
        
        last_measurement_time = Some(curr);

        if i % 10 == 0 {
            // Do logging
            broadcast_log(
                sirin.data.time,
                Log::State(nominal.clone())
            );
        }

        i = i.wrapping_add(1);

        sirin.led.set_low();
    }
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