#![no_std]
#![allow(unused_imports)]
#![doc = include_str!("../../README.md")]

use bmp3::Bmp3;
use core::{
    any::Any,
    cell::UnsafeCell,
    ffi::CStr,
    marker::PhantomPinned,
    mem::MaybeUninit,
    pin::{pin, Pin},
    ptr::addr_of_mut,
    task::RawWaker,
};
use defmt::{info, Display2Format};
use embassy_executor::{Executor, Spawner};
use embassy_futures::join::{join, join3, join4, join5, join_array};
use embassy_stm32::{
    bind_interrupts,
    dma::NoDma,
    gpio::{Level, Output, Speed},
    mode::Async,
    pac::{self, Interrupt::TIM16},
    peripherals::USB_OTG_FS,
    spi as em_spi,
    time::mhz,
    usart::{self, BufferedUartTx, RingBufferedUartRx, Uart, UartTx},
    Config, Peripherals,
};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, pubsub::PubSubChannel};
use embassy_time::Timer;
use embassy_usb::class::cdc_acm::{CdcAcmClass, State as UsbState};
use embassy_usb::Builder as UsbBuilder;
use flash::Flash;
use gpio::GpioPins;
use h3lis::H3lis;
pub use h3lis;
use lis3mdl::Lis3mdl;
pub use lis3mdl;
use lsm6dso_spi::Lsm6dso;
pub use lsm6dso_spi;
pub use rfm9;
use rfm9::{ReadRfm9, Rfm9};
use sirin_macros::{FromSong, SongSize, ToSong};
use sirin_shared::error::SirinError;
use sirin_shared::{
    config::SirinConfig,
    mode::SirinMode,
    packet::{Measurement, SirinData, SirinState},
    song::{FromSong, FromSongError, SongSize, ToSong, ToSongError},
};
use snafu::{ensure, Snafu};
use spi::{Spi, SpiConfig, SpiConfigStruct, SpiDev, SpiInstance, WithSpiHandle};
use sync::Mutex;
use ublox::{
    cfg_inf::{CfgInf, CfgInfBuilder, CfgInfMask},
    cfg_msg::CfgMsgSinglePortBuilder,
    cfg_nav5::CfgNav5Builder,
    cfg_prt::{
        CfgPrtUartBuilder, DataBits, InProtoMask, OutProtoMask, Parity, StopBits, UartMode,
        UartPortId,
    },
    cfg_rate::{CfgRate, CfgRateBuilder},
    mon_rf::MonRf,
    nav_dop::NavDop,
    nav_pvt::proto27_31::{NavPvt, NavPvtRef},
    nav_sat::NavSat,
    proto31::*,
    rxm_rawx::RxmRawx,
    FixedBuffer, UbxPacketMeta, UbxProtocol,
};
use ublox::{proto31::*, GnssFixType, Parser, Position, UbxPacket, Velocity};
use usb::{setup_usb, ReadEp, SirinUsb, UsbSerialClass, WriteEp};
use uunit::{Celsius, Pascals};
use w25qx::W25Q;

pub use sirin_shared::*;

pub mod delay;
pub mod deque;
pub mod flash;
pub mod gpio;
pub mod gps;
pub mod handle_requests;
pub mod io;
pub mod spi;
pub mod subsystems;
pub mod sync;
pub mod time;
pub mod triplet;
pub mod usb;

pub use handle_requests::*;

pub use sirin_shared::packet;
pub use sirin_shared::song;
pub use sirin_shared::state;
pub use uunit;

use crate::{io::Io, subsystems::Subsystem};

pub type Radio = Rfm9<SpiDev>;

#[repr(transparent)]
pub struct UnsafeSync<T>(pub T);

unsafe impl<T> Sync for UnsafeSync<T> {}

pub struct SirinStatics {
    pub spi1: SpiInstance,
    pub spi2: SpiInstance,
    pub gps_buf: [u8; 512],
    pub config: SirinConfig,
    pub sirin: Sirin,
    _phantom_pinned: PhantomPinned,
}

static STATICS: UnsafeSync<UnsafeCell<MaybeUninit<SirinStatics>>> =
    UnsafeSync(UnsafeCell::new(MaybeUninit::uninit()));

#[derive(Debug, Clone)]
pub struct SirinHealth {
    pub flash: Result<(), SirinError>,
    pub radio: Result<(), SirinError>,
    pub baro: Result<(), SirinError>,
    pub imu: Result<(), SirinError>,
    pub high_g_imu: Result<(), SirinError>,
    pub magnetometer: Result<(), SirinError>,
}

pub struct Sirin {
    pub spawner: Spawner,

    pub gpio: GpioPins,

    pub state: SirinState,

    pub io: &'static Io,
    pub led: Output<'static>,
    /*
    pub parachute_main: Output<'static>,
    pub main_power: Output<'static>,
    pub parachute_apo: Output<'static>,
    pub apo_power: Output<'static>,
    */
    // Instrument subsytems
    pub baro: Bmp3<SpiDev>,
    pub imu: Lsm6dso<SpiDev>,
    pub high_g_imu: H3lis<SpiDev>,
    pub magnetometer: Lis3mdl<SpiDev>,

    //UART GPS
    pub gps_rx: RingBufferedUartRx<'static>,
    pub gps_tx: UartTx<'static, Async>,

    pub mode: SirinMode,
    pub data: SirinData,
    pub health: SirinHealth,
    pub config: &'static SirinConfig,
}

bind_interrupts!(struct Irqs {
    USART3 => usart::InterruptHandler<embassy_stm32::peripherals::USART3>;
});

fn stm32_config() -> embassy_stm32::Config {
    use embassy_stm32::rcc::*;

    let mut c = embassy_stm32::Config::default();

    c.rcc.hsi = Some(HSIPrescaler::DIV1);
    c.rcc.csi = true;
    c.rcc.pll1 = Some(Pll {
        source: PllSource::HSI,
        prediv: PllPreDiv::DIV4,
        mul: PllMul::MUL50,
        divp: Some(PllDiv::DIV2),
        divq: Some(PllDiv::DIV8), // used by SPI3. 100Mhz.
        divr: None,
    });
    c.rcc.sys = Sysclk::PLL1_P; // 400 Mhz
    c.rcc.ahb_pre = AHBPrescaler::DIV2; // 200 Mhz
    c.rcc.apb1_pre = APBPrescaler::DIV2; // 100 Mhz
    c.rcc.apb2_pre = APBPrescaler::DIV2; // 100 Mhz
    c.rcc.apb3_pre = APBPrescaler::DIV2; // 100 Mhz
    c.rcc.apb4_pre = APBPrescaler::DIV2; // 100 Mhz
    c.rcc.voltage_scale = VoltageScale::Scale1;
    c.rcc.mux.usbsel = mux::Usbsel::HSI48;

    c
}

impl Sirin {
    #[inline]
    #[allow(unused)]
    #[allow(static_mut_refs)]
    pub async fn new(spawner: Spawner) -> &'static mut Sirin {
        let p = embassy_stm32::init(stm32_config());
        let mut spi_config = em_spi::Config::default();
        spi_config.frequency = mhz(1);

        let spi1: &mut SpiInstance;
        let spi2: &mut SpiInstance;
        let gps_buf: &mut [u8; 512];
        let config: &SirinConfig;

        let statics = unsafe { &mut *STATICS.0.get() };

        macro_rules! ptr {
            (statics . $field: ident) => {
                &raw mut (*statics.as_mut_ptr()).$field
            };
        }

        unsafe {
            let spi1_ptr: *mut SpiInstance = ptr!(statics.spi1);
            spi1_ptr.write(spi::SpiInstance::new(SpiConfigStruct {
                spi: p.SPI1,
                sck: p.PA5,
                miso: p.PA6,
                mosi: p.PA7,
                dma_tx: p.DMA1_CH3,
                dma_rx: p.DMA1_CH2,
                config: spi_config,
            }));
            spi1 = &mut *spi1_ptr;

            let spi2_ptr: *mut SpiInstance = ptr!(statics.spi2);
            spi2_ptr.write(spi::SpiInstance::new(SpiConfigStruct {
                spi: p.SPI2,
                sck: p.PB13,
                miso: p.PB14,
                mosi: p.PB15,
                dma_tx: p.DMA1_CH5,
                dma_rx: p.DMA1_CH4,
                config: spi_config,
            }));
            spi2 = &mut *spi2_ptr;

            let gps_buf_ptr: *mut [u8; 512] = ptr!(statics.gps_buf);
            gps_buf_ptr.write([0; 512]);
            gps_buf = &mut *gps_buf_ptr;
        }

        let gpio = GpioPins {
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
        };

        let state = SirinState::default();
        let mode = SirinMode::Standby;

        // IO
        let radio_cs = Output::new(p.PC8, Level::High, Speed::High);
        let mut radio = Rfm9::new((*spi2).handle(radio_cs));

        radio.init().await.unwrap();
        radio.use_high_power().await.unwrap();
        let radio_selfcheck = radio.selfcheck().await;

        let flash_cs = Output::new(p.PD2, Level::High, Speed::High);
        let mut flash_dev = W25Q::new((*spi2).handle(flash_cs));
        let flash_selfcheck = flash_dev.selfcheck().await;
        let mut flash = unsafe { Flash::new(flash_dev) };

        config = unsafe {
            let config = flash.init().await.unwrap();
            ptr!(statics.config).write(config);
            &*ptr!(statics.config)
        };

        let usb = unsafe { setup_usb(&spawner, p.USB_OTG_FS, p.PA12, p.PA11) };

        let io = unsafe { Io::new(config, spawner, radio, flash, usb.write_ep, usb.read_ep) };

        // Instruments
        let baro_cs = Output::new(p.PA2, Level::High, Speed::High);
        let baro_future = Bmp3::new((*spi1).handle(baro_cs));

        let imu_cs = Output::new(p.PE11, Level::High, Speed::High);
        let mut imu = Lsm6dso::new((*spi1).handle(imu_cs));

        let highg_imu_cs = Output::new(p.PE13, Level::High, Speed::High);
        let mut high_g_imu = H3lis::new((*spi1).handle(highg_imu_cs));

        let magnetometer_cs = Output::new(p.PA3, Level::High, Speed::High);
        let mut magnetometer = Lis3mdl::new((*spi1).handle(magnetometer_cs));

        let mut gps_config = usart::Config::default();
        gps_config.baudrate = 9600;
        gps_config.data_bits = usart::DataBits::DataBits8;
        gps_config.stop_bits = usart::StopBits::STOP1;
        gps_config.parity = usart::Parity::ParityNone;

        let mut gps_uart = Uart::new(
            p.USART3, p.PD9, p.PD8, Irqs, p.DMA1_CH6, p.DMA1_CH7, gps_config,
        )
        .unwrap();

        //Construct GPS config packets
        //UART config for GPS
        let port_config_packet = CfgPrtUartBuilder {
            portid: UartPortId::Uart1,
            reserved0: 0,
            tx_ready: 0,
            mode: UartMode::new(DataBits::Eight, Parity::None, StopBits::One),
            baud_rate: 9600,
            in_proto_mask: InProtoMask::UBLOX,
            out_proto_mask: OutProtoMask::UBLOX,
            flags: 0,
            reserved5: 0,
        };
        //Navigation Mode config
        let mut nav_mode_config = CfgNav5Builder::default();
        nav_mode_config.dyn_model =
            ublox::cfg_nav5::NavDynamicModel::AirborneWithLess4gAcceleration;
        nav_mode_config.fix_mode = ublox::cfg_nav5::NavFixMode::Auto2D3D;
        //GPS measurement and calculation rate
        let gps_update_config = CfgRateBuilder {
            measure_rate_ms: 100,
            nav_rate: 1,
            time_ref: ublox::cfg_rate::AlignmentToReferenceTime::Utc,
        };
        //Navigation message config (Position/Velocity/Time)
        let nav_msg_config = CfgMsgSinglePortBuilder {
            msg_class: NavPvt::CLASS,
            msg_id: NavPvt::ID,
            rate: 1,
        };
        //DOP message config (Dilution of precession)
        let nav_dop_config = CfgMsgSinglePortBuilder {
            msg_class: NavDop::CLASS,
            msg_id: NavDop::ID,
            rate: 1,
        };
        //RF message config ()
        let rf_msg_config = CfgMsgSinglePortBuilder {
            msg_class: MonRf::CLASS,
            msg_id: MonRf::ID,
            rate: 5,
        };
        //Satelite message config ()
        let satelite_msg_config = CfgMsgSinglePortBuilder {
            msg_class: NavSat::CLASS,
            msg_id: NavSat::ID,
            rate: 5,
        };

        //Send GPS config packets
        let gps_config_delay = 50u64;
        gps_uart
            .write(&port_config_packet.into_packet_bytes())
            .await
            .unwrap();
        Timer::after_millis(gps_config_delay).await;
        gps_uart
            .write(&nav_mode_config.into_packet_bytes())
            .await
            .unwrap();
        Timer::after_millis(gps_config_delay).await;
        gps_uart
            .write(&gps_update_config.into_packet_bytes())
            .await
            .unwrap();
        Timer::after_millis(gps_config_delay).await;
        gps_uart
            .write(&nav_msg_config.into_packet_bytes())
            .await
            .unwrap();
        Timer::after_millis(gps_config_delay).await;
        gps_uart
            .write(&nav_dop_config.into_packet_bytes())
            .await
            .unwrap();
        Timer::after_millis(gps_config_delay).await;
        gps_uart
            .write(&rf_msg_config.into_packet_bytes())
            .await
            .unwrap();
        Timer::after_millis(gps_config_delay).await;
        gps_uart
            .write(&satelite_msg_config.into_packet_bytes())
            .await
            .unwrap();
        Timer::after_millis(gps_config_delay).await;

        let (gps_tx, gps_rx) = gps_uart.split();
        let gps_rx = gps_rx.into_ring_buffered(gps_buf);

        let data = SirinData::unmeasured();

        let led = Output::new(p.PA1, Level::Low, Speed::High);

        // TODO: these need to be initialized somewhere. What are the pin numbers on the mosfets?
        // You shouldn't assume the pin functions, and you certainly shouldn't be so opinionated
        // to set this here!
        /*
        ptr!(sirin.parachute_main).write(Output::new(p.PA8, Level::Low, Speed::High));

        ptr!(sirin.main_power).write(Output::new(p.PD3, Level::High, Speed::High));

        ptr!(sirin.parachute_apo).write(Output::new(p.PA10, Level::Low, Speed::High));

        ptr!(sirin.apo_power).write(Output::new(p.PD1, Level::High, Speed::High)); */

        // TODO: JOIN FUTURES, AWAIT
        let mut baro = baro_future.await.unwrap();
        imu.setup().await.unwrap();
        high_g_imu.setup().await.unwrap();
        magnetometer.setup().await.unwrap();

        let health = {
            let (baro, imu, high_g_imu, magnetometer) = join4(
                baro.selfcheck(),
                imu.selfcheck(),
                high_g_imu.selfcheck(),
                magnetometer.selfcheck(),
            )
            .await;

            SirinHealth {
                flash: flash_selfcheck,
                radio: radio_selfcheck,
                baro,
                imu,
                high_g_imu,
                magnetometer,
            }
        };

        let sirin = Sirin {
            spawner,
            gpio,
            state,
            io,
            led,
            baro,
            imu,
            high_g_imu,
            magnetometer,
            gps_rx,
            gps_tx,
            mode,
            data,
            health,
            config,
        };

        unsafe {
            ptr!(statics.sirin).write(sirin);
            core::mem::transmute::<_, &'static mut Sirin>(&mut ptr!(statics.sirin))
        }
    }

    pub fn reboot() -> ! {
        cortex_m::peripheral::SCB::sys_reset();
    }

    pub fn deploy_chute_main(parachute_main: &mut Output<'static>) {
        parachute_main.set_high();
    }

    pub fn deploy_chute_apo(parachute_apo: &mut Output<'static>) {
        parachute_apo.set_high();
    }
}
