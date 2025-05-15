#![no_std]
#![no_main]
#![allow(unused_imports)]

use core::{f32, f64::consts::PI, mem::{self, transmute_copy, MaybeUninit}, pin::Pin, u16};

use bmp3::{hal::{Bmp3RawData, ReadBmp3, RegErrReg, RegStatus}, Bmp3Readout};
use defmt::{debug, info, println, Debug2Format};
use embassy_executor::{task, Executor, Spawner};
use embassy_stm32::{bind_interrupts, dma::NoDma, gpio::{Level, Output, Speed}, peripherals::{self, DMA1_CH0, DMA1_CH1, PD8, PD9, USART3}, usart::{self, Config, Uart}};
use embassy_time::{Duration, Instant, Timer};
use embedded_hal_1::spi::ErrorKind;
use postcard::take_from_bytes;
use rfm9::{ReadRfm9, Rfm9};
use w25qx::W25Q;
use {defmt_rtt as _, panic_probe as _};
use sirin::{error::SirinError, flash::Flash, io::{broadcast, flash_io_task, radio_io_task, send_packet, set_usb_broadcasting_enabled, try_receive_packet, usb_input_task, usb_output_task, IN_CHANNEL}, packet::{InPacket, IoChannel, IoPacket, Log, LogEntry, OutPacket, PacketError, Page}, song::{FromSong, SongSize}, spi::SpiDev, state::{Accel, EcefPos, State, Vel}, subsystems::SirinData, sync::Mutex, uunit::WithUnits, Radio, Sirin};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::{Channel, TrySendError}, pubsub::{Publisher, Subscriber}};
use sirin_shared::mode::SirinMode;
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