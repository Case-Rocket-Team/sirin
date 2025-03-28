use defmt::println;
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, pipe::Pipe};
use embedded_hal::spi::ErrorKind;
use embedded_io::{Write, ErrorType};
use postcard::to_slice;
use w25q::{W25Q};
use crate::{event::Event, spi::SpiDev};

const SECTOR_SIZE: usize = 4096;
const PAGE_SIZE: usize = 256;

pub struct FlashLogger {
    w25q: &'static mut W25Q<SpiDev>,
    buffer_cursor: usize,
    flash_cursor: usize,
    sector: [u8; SECTOR_SIZE],
}

#[derive(Clone, Debug)]
pub enum FlashLoggerError {
    UnfilledPage,
    SpiError(ErrorKind),
    PostcardError(postcard::Error)
}

impl From<ErrorKind> for FlashLoggerError {
    fn from(value: ErrorKind) -> Self {
        Self::SpiError(value)
    }
}

impl From<postcard::Error> for FlashLoggerError {
    fn from(value: postcard::Error) -> Self {
        Self::PostcardError(value)
    }
}


impl FlashLogger {
    pub fn new(w25q: &'static mut W25Q<SpiDev>) -> Self {
        FlashLogger {
            w25q,
            buffer_cursor: 0, // TODO implement rolling buffer
            flash_cursor: 0,
            sector: [0; SECTOR_SIZE],
        }
    }

    pub async fn write_event(&mut self, event: &Event) -> Result<(), FlashLoggerError> {
        let res = to_slice(&event, &mut self.sector[self.buffer_cursor..]);

        let err = match res {
            Ok(slice) => {
                self.buffer_cursor += slice.len();
                if self.buffer_cursor > (self.flash_cursor / PAGE_SIZE + 1) * PAGE_SIZE {
                    self.write_page().await?;
                }

                return Ok(());
            },
            Err(e) => e
        };

        let postcard::Error::SerializeBufferFull = err else {
            return Err(err.into());
        };

        println!("Didn't fit. Erasing sector...");
        self.flash_cursor = (self.flash_cursor / SECTOR_SIZE + 1) * SECTOR_SIZE;
        self.w25q.sector_erase(self.flash_cursor as u32).await?;
        self.buffer_cursor = 0;

        let slice = to_slice(&event, &mut self.sector)?;
        self.buffer_cursor += slice.len();
        self.write_page().await?;

        Ok(())
    }

    async fn write_page(&mut self) -> Result<usize, FlashLoggerError> {
        let start = self.flash_cursor % SECTOR_SIZE;
        let end = (start + (self.buffer_cursor % PAGE_SIZE)).min((self.flash_cursor / PAGE_SIZE + 1) * PAGE_SIZE);

        let bytes_written = self.w25q.page(self.flash_cursor as u32, &self.sector[start..end]).await?;

        self.flash_cursor += bytes_written as usize;

        println!("Wrote page: {}", &self.sector[start..end]);

        Ok(bytes_written as usize)
    }
}