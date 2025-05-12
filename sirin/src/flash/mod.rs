#![doc = include_str!("README.md")]

use cyclic::CyclicFlashSection;
use defmt::{println, trace, Display2Format};
use embedded_hal::spi::ErrorKind;
use sirin_shared::{packet::OutPacket, song::{FromSong, SongSize, ToSong}};
use w25qx::W25Q;

use crate::{error::SirinError, spi::SpiDev, SirinConfig};

mod cyclic;

const FLASH_SIZE: u32 = 16777216;

pub struct Flash {
    pub w25q: W25Q<SpiDev>,
    flight_data: CyclicFlashSection,
    is_second_config: bool
}

impl Flash {
    /// NB. Must call init
    pub(crate) fn new(w25q: W25Q<SpiDev>) -> Self {
        Self {
            w25q,
            flight_data: CyclicFlashSection::new(4096 * 2, FLASH_SIZE),
            is_second_config: true
        }
    }

    pub(crate) async fn debug(&mut self) -> Result<(), ErrorKind> {
        let mut buf = [0u8; 512];

        self.w25q.read_data(0, &mut buf).await?;
        println!("First sector: {:x}", buf[0..256]);

        self.w25q.read_data(4096, &mut buf).await?;
        println!("Second sector: {:x}", buf[0..256]);

        Ok(())
    }

    pub(crate) async fn init(&mut self) -> Result<SirinConfig, ErrorKind> {
        self.flight_data.init(&mut self.w25q).await?;

        self.debug().await?;

        // Read config
        let mut buf = [0u8; 512];
        self.is_second_config = true;

        self.w25q.read_data(4096, &mut buf).await?;

        if buf[0] != 0x77 {
            self.is_second_config = false;
            self.w25q.read_data(0, &mut buf).await?;
        }

        if buf[0] != 0x77 {
            trace!("Couldn't read config. Filling in with default.");
            let config = SirinConfig::default();
            self.save_config(&config).await?;

            return Ok(config)
        }

        // First byte will just be 0x77 to specify that it isn't erased
        let config = SirinConfig::from_song(&buf[1..]).unwrap_or_default();

        println!("{}", Display2Format(&config));

        Ok(config)
    }

    /// `data` must be an `OutPacket`!
    pub async fn log(&mut self, data: &[u8]) -> Result<(), ErrorKind> {
        self.flight_data.append(&mut self.w25q, data).await?;
        Ok(())
    }

    pub async fn save_config(&mut self, config: &SirinConfig) -> Result<(), ErrorKind> {
        let mut buf = [0u8; 512];
        buf[0] = 0x77;
        // TODO remove unwrap
        config.to_song(&mut buf[1..]).unwrap();

        if self.is_second_config {
            self.w25q.sector_erase(0).await?;
            self.w25q.write(0, &buf[0..config.song_size() + 1]).await?;
            self.w25q.sector_erase(4096).await?;
        } else {
            self.w25q.sector_erase(4096).await?;
            self.w25q.write(4096, &buf[0..config.song_size() + 1]).await?;
            self.w25q.sector_erase(0).await?;
        }

        Ok(())
    }
}