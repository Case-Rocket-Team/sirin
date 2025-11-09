#![no_std]
#![allow(unused_imports)]
//#![doc = include_str!("../README.md")]

use core::{ffi::CStr, marker::PhantomPinned, mem::MaybeUninit, pin::{pin, Pin}, ptr::addr_of_mut};
use bmp3::Bmp3;
use defmt::{info, Display2Format};
use embassy_executor::{Executor, Spawner};
use embassy_futures::join::{join, join3, join5, join_array};
use embassy_stm32::{ Config, Peripherals, bind_interrupts, dma::NoDma, gpio::{Level, Output, Speed}, mode::Async, pac, peripherals::USB_OTG_FS, spi as em_spi, time::mhz, usart::{self, UartTx, BufferedUartTx, RingBufferedUartRx, Uart} };
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, pubsub::PubSubChannel};
use embassy_time::Timer;
use flash::Flash;
use gpio::GpioPins;
use rfm9::{ReadRfm9, Rfm9};
use sirin_macros::{FromSong, SongSize, ToSong};
use snafu::{ensure, Snafu};
use sirin_shared::{config::SirinConfig, packet::{Measurement, SirinData, SubsystemError}, song::{FromSong, FromSongError, SongSize, ToSong, ToSongError}};
use sync::Mutex;
use usb::{setup_usb, WriteEp, ReadEp, SirinUsb, UsbSerialClass};
use uunit::{Celsius, Pascals};
use w25qx::W25Q;
use lsm6dso_spi::Lsm6dso;
use lis3mdl::Lis3mdl;
use h3lis::H3lis;
use spi::{Spi, SpiConfig, SpiConfigStruct, SpiDev, SpiInstance, WithSpiHandle};
use embassy_usb::class::cdc_acm::{CdcAcmClass, State as UsbState};
use embassy_usb::Builder as UsbBuilder;
use ublox::{FixedBuffer, cfg_nav5::CfgNav5Builder, cfg_prt::{CfgPrtUartBuilder, DataBits, InProtoMask, OutProtoMask, Parity, StopBits, UartMode, UartPortId}, proto31::Proto31};
use ublox::{Parser,UbxPacket,proto31::*,GnssFixType,Position,Velocity};

pub use uunit;
pub mod spi;
pub mod delay;
pub mod gpio;
pub mod sync;
pub mod triplet;
pub mod flash;
pub mod subsystems;
pub mod usb;
pub mod io;
pub mod error;
pub mod time;
pub mod deque;
pub mod gps;

pub use sirin_shared::song;
pub use sirin_shared::state;
pub use sirin_shared::packet;

use crate::subsystems::Subsystem;

pub type Radio = Rfm9<SpiDev>;

static mut GPS_BUF: [u8; 512] = [0u8; 512];

#[derive(Debug, Clone)]
pub struct SirinHealth {
    pub flash: Result<(), SubsystemError>,
    pub radio: Result<(), SubsystemError>,
    pub baro: Result<(), SubsystemError>,
    pub imu: Result<(), SubsystemError>,
    pub high_g_imu: Result<(), SubsystemError>,
    pub magnetometer: Result<(), SubsystemError>
}

pub struct Sirin {
    pub spawner: Spawner,
    pub spi1: SpiInstance,
    pub spi2: SpiInstance,

    pub gpio: GpioPins,

    // Subsystems:
    pub flash: Flash,
    pub radio: Rfm9<SpiDev>,
    pub usb: SirinUsb,
    pub led: Output<'static>,
    pub parachute_main: Output<'static>,
    pub main_power: Output<'static>,
    pub parachute_apo: Output<'static>,
    pub apo_power: Output<'static>,


    // Instrument subsytems
    pub baro: Bmp3<SpiDev>,
    pub imu: Lsm6dso<SpiDev>,
    pub high_g_imu: H3lis<SpiDev>,
    pub magnetometer: Lis3mdl<SpiDev>,

    //pub gps: S1315F8,
    //pub gps: Uart<'static, Async>,
    pub gps_rx: RingBufferedUartRx<'static>,
    pub gps_tx: UartTx<'static, Async>,
    //pub driver: Driver<'static, peripherals::USB_OTG_FS>

    pub data: SirinData,
    pub health: SirinHealth,
    pub config: SirinConfig,
    
    _phantom_pinned: PhantomPinned
}

bind_interrupts!(struct Irqs {
    USART3 => usart::InterruptHandler<embassy_stm32::peripherals::USART3>;
});

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
                config.rcc.mux.usbsel = mux::Usbsel::HSI48;
            }

            //ptr!(sirin.event_channel).write(PubSubChannel::new());

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
                p14: p.PD4
            });



            let baro_ptr: *mut Bmp3<SpiDev> = ptr!(sirin.baro);
            let baro_cs = Output::new(p.PA2, Level::High, Speed::High);
            let baro_future = Bmp3::new((*spi1).handle(baro_cs));

            let radio_ptr: *mut Rfm9<SpiDev> = ptr!(sirin.radio);
            let radio_cs = Output::new(p.PC8, Level::High, Speed::High);
            radio_ptr.write(Rfm9::new((*spi2).handle(radio_cs)));

            let flash_ptr: *mut Flash = ptr!(sirin.flash);
            let flash_cs = Output::new(p.PD2, Level::High, Speed::High);
            let flash_dev = W25Q::new((*spi2).handle(flash_cs));
            flash_ptr.write(Flash::new(flash_dev));

            let config = ptr!(sirin.config);
            config.write((*flash_ptr).init().await.unwrap());
            info!("{}", Display2Format(&*config));

            let imu_ptr: *mut Lsm6dso<SpiDev> = ptr!(sirin.imu);
            let imu_cs = Output::new(p.PE11, Level::High, Speed::High);
            imu_ptr.write(Lsm6dso::new((*spi1).handle(imu_cs)));

            let highg_imu_ptr: *mut H3lis<SpiDev> = ptr!(sirin.high_g_imu);
            let highg_imu_cs = Output::new(p.PE13, Level::High, Speed::High);
            highg_imu_ptr.write(H3lis::new((*spi1).handle(highg_imu_cs)));

            let magnetometer_ptr: *mut Lis3mdl<SpiDev> = ptr!(sirin.magnetometer);
            let magnetometer_cs = Output::new(p.PA3, Level::High, Speed::High); 
            magnetometer_ptr.write(Lis3mdl::new((*spi1).handle(magnetometer_cs)));
            
            let mut gps_uart = Uart::new(
                p.USART3,
                p.PD9,
                p.PD8,
                Irqs,
                p.DMA1_CH6,
                p.DMA1_CH7,
                usart::Config::default()
            ).unwrap();

            //Send GPS setup packet(s)
            let port_config_packet = CfgPrtUartBuilder {
                portid: UartPortId::Uart2,
                reserved0: 0,
                tx_ready: 0,
                mode: UartMode::new(DataBits::Eight, Parity::None, StopBits::One),
                baud_rate: 9600,
                in_proto_mask: InProtoMask::all(),
                out_proto_mask: OutProtoMask::UBLOX,
                flags: 0,
                reserved5: 0,
            }.into_packet_bytes();

            let mut nav_mode_config = CfgNav5Builder::default();
            nav_mode_config.dyn_model = ublox::cfg_nav5::NavDynamicModel::Pedestrian;
            nav_mode_config.fix_mode = ublox::cfg_nav5::NavFixMode::Auto2D3D;
            gps_uart.write(&port_config_packet).await.unwrap();
            gps_uart.write(&nav_mode_config.into_packet_bytes()).await.unwrap();

            let (mut tx,rx) = gps_uart.split();

            ptr!(sirin.gps_rx).write(rx.into_ring_buffered(&mut GPS_BUF));

            //ptr!(sirin.gps_tx).write(tx.into());

            //ptr!(sirin.gps_tx).write(gps_uart.split().0);            

            ptr!(sirin.data).write(SirinData::unmeasured());

            ptr!(sirin.usb).write(setup_usb(
                &spawner,
                p.USB_OTG_FS,
                p.PA12,
                p.PA11
            ));

            ptr!(sirin.led).write(Output::new(p.PA1, Level::Low, Speed::High));

            ptr!(sirin.parachute_main).write(Output::new(p.PA8, Level::Low, Speed::High));
            ptr!(sirin.main_power).write(Output::new(p.PD3, Level::High, Speed::High));

            ptr!(sirin.parachute_apo).write(Output::new(p.PA10, Level::Low, Speed::High));

            ptr!(sirin.apo_power).write(Output::new(p.PD1, Level::High, Speed::High));

            // TODO: JOIN FUTURES, AWAIT
            baro_ptr.write(baro_future.await.unwrap());
            (*radio_ptr).init().await.unwrap();
            (*radio_ptr).use_high_power().await.unwrap();
            (*imu_ptr).setup().await.unwrap();
            (*highg_imu_ptr).setup().await.unwrap();
            (*magnetometer_ptr).setup().await.unwrap();

            {
                /*
                let [flash, radio, baro, imu, high_g_imu, magnetometer] = embassy_futures::join::join_array([
                    (*flash_ptr).w25q.selfcheck(),
                    (*radio_ptr).selfcheck(),
                    (*baro_ptr).selfcheck(),
                    (*imu_ptr).selfcheck(),
                    (*highg_imu_ptr).selfcheck(), 
                    (*magnetometer_ptr).selfcheck()
                ]).await;
             */
                let join1 = join3(
                    (*flash_ptr).w25q.selfcheck(),
                    (*radio_ptr).selfcheck(),
                    (*baro_ptr).selfcheck(),
                );

                let join2 = join3(
                    (*imu_ptr).selfcheck(),
                    (*highg_imu_ptr).selfcheck(),
                    (*magnetometer_ptr).selfcheck(),
                );

                let ((flash, radio, baro), (imu, high_g_imu, magnetometer)) = join(
                    join1,
                    join2,
                ).await;

                ptr!(sirin.health).write(SirinHealth {
                    flash,
                    radio,
                    baro,
                    imu,
                    high_g_imu,
                    magnetometer
                });

                let sirin: &'static mut _ = sirin.assume_init_mut();
                
                sirin
            }
        }
    }

    pub fn reboot() {
        cortex_m::peripheral::SCB::sys_reset();
    }

    pub fn deploy_chute_main(parachute_main: &mut Output<'static>){
        parachute_main.set_high();
    }
    
    pub fn deploy_chute_apo(parachute_apo: &mut Output<'static>){
        parachute_apo.set_high();
    }
}