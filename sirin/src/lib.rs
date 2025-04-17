#![no_std]
#![allow(unused_imports)]
#![doc = include_str!("../README.md")]

use core::{marker::PhantomPinned, mem::MaybeUninit, pin::{pin, Pin}, ptr::addr_of_mut};
use bmp3::Bmp3;
use embassy_executor::{Executor, Spawner};
use embassy_futures::join::{join, join5, join_array};
use embassy_stm32::{ bind_interrupts, gpio::{Level, Output, Speed}, peripherals::USB_OTG_FS, spi as em_spi, time::mhz, Config, Peripherals };
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, pubsub::PubSubChannel};
use event::Event;
use gpio::GpioPins;
use rfm9::{ReadRfm9, Rfm9};
use snafu::{ensure, Snafu};
use subsystems::{BaroData, HighGImuData, ImuData, Measurement, SirinData, Subsystem, SubsystemError};
use usb::{usb_serial, UsbSerial};
use uunit::{Celsius, Pascals};
use w25qx::W25Q;
use lsm6dso_spi::Lsm6dso;
use h3lis::H3lis;
use spi::{Spi, SpiConfig, SpiConfigStruct, SpiDev, SpiInstance, WithSpiHandle};
use embassy_usb::class::cdc_acm::{CdcAcmClass, State as UsbState};
use embassy_usb::Builder as UsbBuilder;

pub use uunit;
pub mod spi;
pub mod delay;
pub mod gpio;
pub mod sync;
pub mod triplet;
pub mod flash_logger;
pub mod event;
pub mod state;
pub mod song;
pub mod subsystems;
pub mod usb;

#[derive(Debug, Clone)]
pub struct SirinHealth {
    pub flash: Result<(), SubsystemError>,
    pub radio: Result<(), SubsystemError>,
    pub baro: Result<(), SubsystemError>,
    pub imu: Result<(), SubsystemError>,
    pub high_g_imu: Result<(), SubsystemError>,
}

pub struct Sirin {
    pub spawner: Spawner,
    pub spi1: SpiInstance,
    pub spi2: SpiInstance,

    pub gpio: GpioPins,

    // Subsystems:
    pub flash: W25Q<SpiDev>,
    pub radio: Rfm9<SpiDev>,

    // Instrument subsytems
    pub baro: Bmp3<SpiDev>,
    pub imu: Lsm6dso<SpiDev>,
    pub high_g_imu: H3lis<SpiDev>,
    // pub gps: S1315F8,
    //pub driver: Driver<'static, peripherals::USB_OTG_FS>

    pub data: SirinData,
    pub health: SirinHealth,

    pub event_channel: PubSubChannel<CriticalSectionRawMutex, Event, 100, 4, 4>,

    pub usb: UsbSerial,

    _phantom_pinned: PhantomPinned
}

impl Sirin {
    /// Initializing Sirin is a PITA bc it is a self-referential struct
    #[inline]
    pub async fn init(
        sirin: &'static mut MaybeUninit<Sirin>,
        spawner: Spawner
    ) -> &'static mut Sirin {
        unsafe {
            macro_rules! ptr {
                (sirin . $field: ident) => {
                    addr_of_mut!((*sirin.as_mut_ptr()).$field)
                };
            }

            *ptr!(sirin.spawner) = spawner;
            let mut config = Config::default();
            {
                use embassy_stm32::rcc::*;
                config.rcc.hsi = Some(HSIPrescaler::DIV1);
                config.rcc.csi = true;
                config.rcc.pll1 = Some(Pll {
                    source: PllSource::HSI,
                    prediv: PllPreDiv::DIV4,
                    mul: PllMul::MUL50,
                    divp: Some(PllDiv::DIV2),
                    divq: Some(PllDiv::DIV8), // used by SPI3. 100Mhz.
                    divr: None,
                });
                config.rcc.sys = Sysclk::PLL1_P; // 400 Mhz
                config.rcc.ahb_pre = AHBPrescaler::DIV2; // 200 Mhz
                config.rcc.apb1_pre = APBPrescaler::DIV2; // 100 Mhz
                config.rcc.apb2_pre = APBPrescaler::DIV2; // 100 Mhz
                config.rcc.apb3_pre = APBPrescaler::DIV2; // 100 Mhz
                config.rcc.apb4_pre = APBPrescaler::DIV2; // 100 Mhz
                config.rcc.voltage_scale = VoltageScale::Scale1;
            }

            ptr!(sirin.event_channel).write(PubSubChannel::new());

            let p = embassy_stm32::init(config);
            let mut spi_config = em_spi::Config::default();
            spi_config.frequency = mhz(1);

            let spi1: *mut SpiInstance = ptr!(sirin.spi1);
            spi1.write(spi::SpiInstance::new(SpiConfigStruct {
                spi: p.SPI1,
                sck: p.PA5,
                miso: p.PA6,
                mosi: p.PA7,
                dma_tx: p.DMA1_CH3,
                dma_rx: p.DMA1_CH2,
                config: spi_config
            }));

            let spi2: *mut SpiInstance = ptr!(sirin.spi2);
            spi2.write(spi::SpiInstance::new(SpiConfigStruct {
                spi: p.SPI2,
                sck: p.PB13,
                miso: p.PB14,
                mosi: p.PB15,
                dma_tx: p.DMA1_CH5,
                dma_rx: p.DMA1_CH4,
                config: spi_config
            }));

            let gpio: *mut GpioPins = ptr!(sirin.gpio);
            gpio.write(GpioPins {
                p1: p.PA4,
                p2: p.PC4,
                p3: p.PC5,
                p4: p.PB0,
                p5: p.PB1,
                p6: p.PB2,
                p7: p.PE7,
                p8: p.PE8,
                p9: p.PE9,
                p10: p.PE10,
                p11: p.PD7,
                p12: p.PD6,
                p13: p.PD5,
                p14: p.PD4,
                p15: p.PD3,
                p16: p.PD1,
                p17: p.PD0,
                p18: p.PC12,
                p19: p.PC11,
                p20: p.PC10,
            });

            let baro_ptr: *mut Bmp3<SpiDev> = ptr!(sirin.baro);
            let baro_cs = Output::new(p.PA2, Level::High, Speed::High);
            let baro_future = Bmp3::new((*spi1).handle(baro_cs));

            let radio_ptr: *mut Rfm9<SpiDev> = ptr!(sirin.radio);
            let radio_cs = Output::new(p.PC8, Level::High, Speed::High);
            radio_ptr.write(Rfm9::new((*spi2).handle(radio_cs)));

            let flash_ptr: *mut W25Q<SpiDev> = ptr!(sirin.flash);
            let flash_cs = Output::new(p.PD2, Level::High, Speed::High);
            flash_ptr.write(W25Q::new((*spi2).handle(flash_cs)));

            let imu_ptr: *mut Lsm6dso<SpiDev> = ptr!(sirin.imu);
            let imu_cs = Output::new(p.PE11, Level::High, Speed::High);
            imu_ptr.write(Lsm6dso::new((*spi1).handle(imu_cs)));

            let highg_imu_ptr: *mut H3lis<SpiDev> = ptr!(sirin.high_g_imu);
            let highg_imu_cs = Output::new(p.PE13,Level::High, Speed::High);
            highg_imu_ptr.write(H3lis::new((*spi1).handle(highg_imu_cs)));

            ptr!(sirin.data).write(SirinData::unmeasured());

            ptr!(sirin.usb).write(usb_serial(
                &spawner,
                p.USB_OTG_FS,
                p.PA12,
                p.PA11
            ));
            
            // TODO: JOIN FUTURES, AWAIT
            baro_ptr.write(baro_future.await.unwrap());
            (*radio_ptr).init().await.unwrap();
            (*radio_ptr).use_high_power().await.unwrap();
            (*imu_ptr).setup().await.unwrap();
            (*highg_imu_ptr).setup().await.unwrap();

            {
                let (flash, radio, baro, imu, high_g_imu) = join5(
                    (*flash_ptr).selfcheck(),
                    (*radio_ptr).selfcheck(),
                    (*baro_ptr).selfcheck(),
                    (*imu_ptr).selfcheck(),
                    (*highg_imu_ptr).selfcheck()
                ).await;

                ptr!(sirin.health).write(SirinHealth {
                    flash,
                    radio,
                    baro,
                    imu,
                    high_g_imu,
                });
            }

            let sirin: &'static mut _ = sirin.assume_init_mut();
            sirin
        }
    }
}