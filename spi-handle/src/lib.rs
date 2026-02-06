#![no_std]
#![doc = include_str!("../README.md")]
#![allow(mismatched_lifetime_syntaxes)]

use core::{future::Future, marker::PhantomData, ops::{Deref, DerefMut}};
use embedded_hal_async::spi::{Error, ErrorKind, ErrorType, SpiBus};

pub trait SpiHandle<W: 'static + Copy = u8> {
    /// The type of the actual SPI bus we're using.
    type Bus: SpiBus<W, Error = ErrorKind>;

    /// The future resolves once the SPI device has been selected on the bus. 
    /// This method is infallible. It should not panic or fail. If the 
    /// device cannot be selected, the future should wait intil it is able to select.
    fn select(&mut self) -> impl Future<Output = SpiHandleBus<Self, W>> + '_;

    /// Deselects the chip. You'll implement this here, but you shouldn't call
    /// this from your code -- the chip will automatically be deselected when
    /// [SpiHandleBus] is dropped. You can also call [SpiHandleBus::deselect],
    /// to drop manually.
    /// 
    /// You can access self through `bus.handle()`.
    fn deselect(bus: &mut SpiHandleBusDestructor<Self, W>);
}

pub struct SpiHandleBus<'h, H: 'h + SpiHandle<W> + ?Sized, W: 'static + Copy> {
    handle: &'h mut H,
    bus: H::Bus
}

impl <'h, H: SpiHandle<W> + ?Sized, W: 'static + Copy> SpiHandleBus<'h, H, W> {
    pub fn new(handle: &'h mut H, bus: H::Bus) -> Self {
        SpiHandleBus {
            handle,
            bus
        }
    }

    /// Deselects the chip (by dropping self)
    /// This is the same as just dropping it yourself, but the name
    /// makes your code more readable.
    pub fn deselect(self) {
        // drops it lol
    }
}

impl <'h, H: 'h + SpiHandle<W> + ?Sized, W: 'static + Copy> Drop for SpiHandleBus<'h, H, W> {
    fn drop(&mut self) {
        H::deselect(&mut SpiHandleBusDestructor {
            handle: self.handle,
            _phantom: PhantomData
        });
    }
}

pub struct SpiHandleBusDestructor<'h, H: 'h + SpiHandle<W> + ?Sized, W: 'static + Copy> {
    handle: &'h mut H,
    _phantom: PhantomData<W>
}

impl <H: SpiHandle<W> + ?Sized, W: 'static + Copy> SpiHandleBusDestructor<'_, H, W> {
    pub fn handle(&mut self) -> &mut H {
        &mut self.handle
    }
}

impl <H: SpiHandle<W> + ?Sized, W: 'static + Copy> Deref for SpiHandleBus<'_, H, W> {
    type Target = H::Bus;

    fn deref(&self) -> &Self::Target {
        &self.bus
    }
}

impl <H: SpiHandle<W> + ?Sized, W: 'static + Copy> DerefMut for SpiHandleBus<'_, H, W> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.bus
    }
}

impl <H: SpiHandle<W> + ?Sized, W: 'static + Copy> ErrorType for SpiHandleBus<'_, H, W> {
    type Error = ErrorKind;
}

impl <'h, H: SpiHandle<W>, W: 'static + Copy> SpiBus<W> for SpiHandleBus<'h, H, W> {
    async fn read(&mut self, words: &mut [W]) -> Result<(), Self::Error> {
        self.bus.read(words).await
    }

    async fn write(&mut self, words: &[W]) -> Result<(), Self::Error> {
        self.bus.write(words).await
    }

    async fn transfer(&mut self, read: &mut [W], write: &[W]) -> Result<(), Self::Error> {
        self.bus.transfer(read, write).await
    }

    async fn transfer_in_place(&mut self, words: &mut [W]) -> Result<(), Self::Error> {
        self.bus.transfer_in_place(words).await
    }

    async fn flush(&mut self) -> Result<(), Self::Error> {
        self.bus.flush().await
    }
}

pub struct SpiBusHandle<D: DerefMut>(pub D)
where D::Target: SpiBus;

impl <D: DerefMut> ErrorType for SpiBusHandle<D>
where D::Target: SpiBus {
    type Error = ErrorKind;
}

impl <D: DerefMut> Deref for SpiBusHandle<D>
where D::Target: SpiBus {
    type Target = D;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl <D: DerefMut> DerefMut for SpiBusHandle<D>
where D::Target: SpiBus {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

type W = u8;

impl <D: DerefMut> SpiBus for SpiBusHandle<D>
where D::Target: SpiBus {
    async fn read(&mut self, words: &mut [W]) -> Result<(), Self::Error> {
        (**self).read(words).await.map_err(|e| e.kind())
    }

    async fn write(&mut self, words: &[W]) -> Result<(), Self::Error> {
        (**self).write(words).await.map_err(|e| e.kind())
    }

    async fn transfer(&mut self, read: &mut [W], write: &[W]) -> Result<(), Self::Error> {
        (**self).transfer(read, write).await.map_err(|e| e.kind())
    }

    async fn transfer_in_place(&mut self, words: &mut [W]) -> Result<(), Self::Error> {
        (**self).transfer_in_place(words).await.map_err(|e| e.kind())
    }

    async fn flush(&mut self) -> Result<(), Self::Error> {
        (**self).flush().await.map_err(|e| e.kind())
    }
}