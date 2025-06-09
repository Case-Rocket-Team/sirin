#![no_std]
#![no_main]
#![allow(unused_imports)]

use core::mem::{self, MaybeUninit};
mod flight_computer;
use flight_computer::FlightComputer;

use cortex_m_rt::entry;
use cortex_m::Peripherals as CorePeripherals;
use stm32h7xx_hal::{
    pac,
    prelude::*,
    delay::Delay,
    gpio::{Output as Output2, PushPull},
};

use embedded_hal::digital::v2::OutputPin;
use embedded_hal::digital::v2::InputPin;

use bmp3::{hal::{Bmp3RawData, ReadBmp3, RegErrReg, RegStatus}, Bmp3Readout};
use defmt::*;
use embassy_executor::{task, Executor, Spawner};
use embassy_stm32::{bind_interrupts, dma::NoDma, gpio::{Level, Output, Speed}, peripherals::{self, DMA1_CH0, DMA1_CH1, PD8, PD9, USART3}, usart::{self, Config, Uart}};
use embassy_time::Timer;
use rfm9::ReadRfm9;
use {defmt_rtt as _, panic_probe as _};
use sirin::Sirin;

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

#[allow(unused_variables)]
async fn main_task(sirin: &'static mut Sirin) {
    println!("Hello world!");
}

fn main() -> ! {
    // let dp = pac::Peripherals::take().unwrap();
    // let cp = CorePeripherals::take().unwrap();

    // let rcc = dp.RCC.constrain();
    // let clocks = rcc.cfgr.sysclk(84.mhz()).freeze();
    // rcc.sys_ck(84.MHz());
    

    // let mut delay = Delay::new(cp.SYST, clocks);

    // let gpioa = dp.GPIOA.split(dp);
    // let screw_switch = gpioa.pa0.into_pull_down_input();
    // let mut led = gpioa.pa5.into_push_pull_output();

    // while screw_switch.is_low().unwrap() {

    // }

    // let mut fc = FlightComputer::new(dp, clocks);
    // fc.initialize_all(&mut delay);
    // fc.enter_standby();

    // for _ in 0..5 {
    //     led.set_high().unwrap();
    //     delay.delay_ms(200_u16);
    //     led.set_low().unwrap();
    //     delay.delay_ms(200_u16);
    // }

    loop {}
}