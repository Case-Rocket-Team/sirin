#![no_std]
#![no_main]
#[allow(unused_imports)]

use core::{f32, f64::consts::PI, mem::{self, MaybeUninit}};

use bmp3::{hal::{Bmp3RawData, ReadBmp3, RegErrReg, RegStatus}, Bmp3Readout};
use defmt::debug;
use embassy_executor::{task, Executor, Spawner};
use embassy_stm32::{bind_interrupts, gpio::{Level, Output, Speed}, peripherals::{self, DMA1_CH0, DMA1_CH1, PD8, PD9, USART3}, usart::{self, Config, Uart}};
use embassy_time::Timer;
use embedded_hal_1::spi::ErrorKind;
use rfm9x::ReadRfm9x;
use {defmt_rtt as _, panic_probe as _};
use sirin::{event::Event, flash_logger::FlashLogger, measurement::Measurement, Sirin};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, pubsub::{Publisher, Subscriber}};

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

    main_task(sirin).await
}

bind_interrupts!(struct Irqs {
    USART3 => usart::InterruptHandler<peripherals::USART3>;
});

async fn main_task(sirin: &'static mut Sirin) {
    let publisher = sirin.event_channel.immediate_publisher();
    let flash_sub = sirin.event_channel.subscriber().unwrap();

    let mut logger = FlashLogger::new(&mut sirin.flash);

    let logger_mut = unsafe {
        // Safety: this main task ought to live forever
        transmute_into_static(&mut logger)
    };

    sirin.spawner.must_spawn(flash_writer(logger_mut, flash_sub));

    loop {
        publisher.publish_immediate(Event::Measurement(Measurement::Baro(sirin.baro.read().await.unwrap())));
        publisher.publish_immediate(Event::Measurement(Measurement::ImuAccel(sirin.imu.accel().await.unwrap())));
        publisher.publish_immediate(Event::Measurement(Measurement::ImuAngularVel(sirin.imu.angular_vel().await.unwrap())));
    }
}

// TODO: airbreaks
#[task]
#[allow(unused_variables)]
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
}

#[task]
#[allow(unused_variables)]
async fn flash_writer(
    logger: &'static mut FlashLogger,
    mut flash_sub: Subscriber<'static, CriticalSectionRawMutex, Event, 100, 4, 4>
) {
    loop {
        // TODO: Report error on lag
        let event = flash_sub.next_message_pure().await;
        
    }
}