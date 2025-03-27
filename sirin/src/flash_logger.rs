use embassy_sync::{blocking_mutex::raw::NoopRawMutex, pipe::Pipe};
use embedded_io::{Write, ErrorType, ErrorKind};
use postcard::to_slice;
use w25q::{W25Q};
use crate::{event::Event, spi::SpiDev};

const SECTOR_SIZE: usize = 4096;

pub struct FlashLogger {
    w25q: &'static mut W25Q<SpiDev>,
    writer_cursor: u32, //Where in the buffer you are,
    sector: [u8; SECTOR_SIZE],
    last_page_written: usize,
}

impl FlashLogger {
    pub fn new(w25q: &'static mut W25Q<SpiDev>) -> Self {
        FlashLogger {
            w25q,
            writer_cursor: 0, // TODO implement rolling buffer
            sector: [0; SECTOR_SIZE],
            last_page_written: 0
        }
    }

    pub async fn write_event(&mut self, event: Event) {
        //to_slice(&event, self.sector)
    }
}

// TODO: Create a writer which we can write into synchronously, then set up a task to occasionally
// write it asynchronously.
impl ErrorType for FlashLogger {
    type Error = ErrorKind;
}

/*
//Writes to a buffer, returns the amount of bytes written
impl Write for FlashLogger {
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        let len = self.pipe.try_write(buf).map_err(|e| ErrorKind::OutOfMemory)?;
        Ok(len)
    }
    
    //I'm pretty sure this doesn't need to do anything because all data writing and buffering is handled in the buffer 
    fn flush(&mut self) -> Result<(), Self::Error>{
        Ok(())
    }
}*/