use embedded_io::Write;
use w25q::W25Q;

use crate::spi::SpiDev;

pub struct Flash {
    w25q: W25Q<SpiDev>,
    cursor: u32
}

// TODO: Create a buffer which we can write into synchronously, then set up a task to occasionally
// write it asynchronously.

impl Write for Flash {
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        todo!()
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        todo!()
    }
}