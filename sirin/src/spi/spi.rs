#![allow(unused_imports)]
use core::{future::Future, marker::PhantomData};

use derive_new::new;
use embassy_stm32::gpio::Output;
use embedded_hal_async::spi::{self as hal_spi, ErrorType, SpiBus};
use sirin_macros::SpiError;
use spi_handle::{DerefSpiBus, SpiHandle, SpiHandleBus};

use crate::sync::{Mutex, MutexGuard};

use super::Spi;

#[derive(SpiError, new)]
pub struct SpiDev {
    spi: &'static Mutex<Spi>,
    cs: Output<'static>
}

impl SpiHandle for SpiDev {
    type Bus = DerefSpiBus<MutexGuard<'static, Spi>>;

    async fn select(&mut self) -> SpiHandleBus<'_, Self, u8> {
        let spi = self.spi.lock().await;
        self.cs.set_low();
        SpiHandleBus::new(self, DerefSpiBus(spi))
    }

    fn deselect(bus: &mut spi_handle::SpiHandleBusDestructor<Self, u8>) {
        bus.handle().cs.set_high();
    }
}