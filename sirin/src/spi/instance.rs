use embassy_stm32::{Peri, gpio::Output, mode::Async, spi::{ self as em_spi, Instance as EmSpiInstance, MisoPin, MosiPin, RxDma, SckPin, Spi as EmSpi, TxDma }};
use spi_handle::SpiHandle;

use crate::sync::Mutex;

use super::SpiDev;

pub type Spi = EmSpi<'static, Async>;

pub trait SpiConfig {
    type Spi: EmSpiInstance;
    type Sck: SckPin<Self::Spi>;
    type Miso: MisoPin<Self::Spi>;
    type Mosi: MosiPin<Self::Spi>;
    type TxDma: TxDma<Self::Spi>;
    type RxDma: RxDma<Self::Spi>;
}

/// TODO: better name
pub struct SpiConfigStruct<
        Spi: EmSpiInstance,
        Sck: SckPin<Spi>,
        Mosi: MosiPin<Spi>,
        Miso: MisoPin<Spi>,
        TTxDma: TxDma<Spi>,
        TRxDma: RxDma<Spi>,
    > {
    pub spi: Peri<'static, Spi>,
    pub sck: Peri<'static, Sck>,
    pub miso: Peri<'static, Miso>,
    pub mosi: Peri<'static, Mosi>,
    pub dma_tx: Peri<'static, TTxDma>,
    pub dma_rx: Peri<'static, TRxDma>,
    pub config: em_spi::Config
}

impl <
    Spi: EmSpiInstance,
    Sck: SckPin<Spi>,
    Miso: MisoPin<Spi>,
    Mosi: MosiPin<Spi>,
    TTxDma: TxDma<Spi>,
    TRxDma: RxDma<Spi>,
> SpiConfig for SpiConfigStruct<Spi, Sck, Mosi, Miso, TTxDma, TRxDma> {
    type Spi = Spi;
    type Sck = Sck;
    type Miso = Miso;
    type Mosi = Mosi;
    type TxDma = TTxDma;
    type RxDma = TRxDma;
}

//#[derive(SpiError)]
pub struct SpiInstance {
    spi_mutex: Mutex<Spi>
}

impl SpiInstance {
    /// SAFETY: the caller must insure the returned value is never moved.
    pub unsafe fn new<T: EmSpiInstance>(config: SpiConfigStruct<
        T,
        impl SckPin<T> + 'static,
        impl MosiPin<T> + 'static,
        impl MisoPin<T> + 'static,
        impl TxDma<T> + 'static,
        impl RxDma<T> + 'static
    >) -> Self {
        Self {
            spi_mutex: Mutex::new(Spi::new(
                config.spi,
                config.sck,
                config.mosi,
                config.miso,
                config.dma_tx,
                config.dma_rx,
                em_spi::Config::default()
            ))
        }
    }

    pub fn handle(&'static self, cs: Output<'static>) -> <Self as WithSpiHandle>::Handle<'static> {
        SpiDev::new(&self.spi_mutex, cs)
    }
}

pub trait WithSpiHandle {
    type Handle<'a>: SpiHandle;
}

impl WithSpiHandle for SpiInstance {
    type Handle<'a> = SpiDev;
}