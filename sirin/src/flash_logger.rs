use defmt::println;
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, pipe::Pipe};
use embedded_hal::spi::ErrorKind;
use embedded_io::{Write, ErrorType};
use postcard::to_slice;
use w25q::{W25Q};
use crate::{event::Event, log_data::{LogData, SerializationError, SerializationSize, Serialize}, spi::SpiDev};

const SECTOR_SIZE: usize = 4096;
const PAGE_SIZE: usize = 256;

pub struct FlashLogger {
    page_n: usize,
    i: usize,
    i_last: usize,
    buffer: [u8; SECTOR_SIZE],
}

#[derive(Clone, Debug)]
pub enum FlashLoggerError {
    SpiError(ErrorKind),
    SerializationError(SerializationError)
}

impl From<ErrorKind> for FlashLoggerError {
    fn from(value: ErrorKind) -> Self {
        Self::SpiError(value)
    }
}

impl FlashLogger {
    pub fn new() -> Self {
        FlashLogger {
            page_n: 0,
            i: 0,
            i_last: 0,
            buffer: [0; SECTOR_SIZE],
        }
    }

    pub async fn log(&mut self, flash: &mut W25Q<SpiDev>, data: &LogData) -> Result<(), FlashLoggerError> {
        loop {
            match data.serialize(&mut self.buffer[self.i..]) {
                Ok(()) => {
                    self.i += data.serialization_size();
                    self.flush(flash).await?;
                    return Ok(())
                },
                Err(SerializationError::NotEnoughBytes) => {
                    self.buffer[
                        self.i..SECTOR_SIZE
                    ].fill(0);

                    println!("Out of bytes!");

                    self.i = SECTOR_SIZE;

                    self.flush(flash).await?;

                    self.i = 0;
                    self.i_last = 0;

                    continue;
                },
                #[allow(unreachable_patterns)]
                Err(e) => return Err(FlashLoggerError::SerializationError(e))
            }
        }
    }

    async fn flush(&mut self, flash: &mut W25Q<SpiDev>) -> Result<(), FlashLoggerError> {
        while self.i - self.i_last >= PAGE_SIZE {
            println!("Flushed: Wrote page to {}: {}", self.page_n * PAGE_SIZE, &self.buffer[self.i_last..(self.i_last + PAGE_SIZE)]);
            flash.page(
                (self.page_n * PAGE_SIZE) as u32,
                &self.buffer[self.i_last..(self.i_last + PAGE_SIZE)]
            ).await?;
            self.page_n += 1;
            self.i_last += PAGE_SIZE;
        }

        Ok(())
    }
}