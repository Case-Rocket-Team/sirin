use embedded_sdmmc::{Controller, SdMmcSpi, Volume, Mode};

pub struct SdLogger;

impl SdLogger {
    pub fn init(&mut self) -> Result<(), ()> {
        // Real setup of SPI SD card interface
        Ok(())
    }

    pub fn log_data(&mut self, _data: &str) {
        // Simulate log
    }
}