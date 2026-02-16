#![doc = include_str!("README.md")]

use core::{future::{poll_fn, Future, PollFn}, mem::MaybeUninit, ops::{Add, Neg, Rem, Sub}};

use cyclic::{AppendResult, CyclicFlashSection};
use defmt::{error, info, println, trace, Debug2Format, Display2Format};
use embassy_time::{Instant, Timer};
use sirin_shared::{packet::{FlightHeader, FlightHeaderStatus, LogPacket, MAX_OUT_PACKET_SIZE}, song::{FromSong, SongSize, ToSong, maybe_unwritten_max_bytes::MaybeUnwrittenMaxBytes}, time::AbsoluteTimeReference};
use sirin_shared::song::ConstSongSize;
use static_assertions::const_assert;
use w25qx::W25Q;

use crate::{deque::Deque, error::SirinError, spi::SpiDev, time::absolute_time_reference, SirinConfig};

mod cyclic;

const FLASH_SIZE: u32 = 16777216;
const SECTOR_SIZE: u32 = 4096;
const FLIGHT_HEADER_COUNT: usize = 256;

const_assert!(FLIGHT_HEADER_COUNT / 2 < SECTOR_SIZE as usize / FlightHeader::SONG_SIZE);

pub struct Flash {
    pub w25q: W25Q<SpiDev>,
    flight_data: CyclicFlashSection,
    pub flight_headers: Deque<FlashFlightHeader, FLIGHT_HEADER_COUNT>,
    is_second_config: bool,
}

#[derive(Clone, Debug)]
pub struct FlashFlightHeader {
    pub index: usize,
    pub header: FlightHeader,
}

impl Flash {
    /// NB. Must call `init()`` after calling `new()`
    #[allow(static_mut_refs)]
    pub(crate) unsafe fn new(w25q: W25Q<SpiDev>) -> Self {
        Self {
            w25q,
            flight_data: CyclicFlashSection::new(4 * SECTOR_SIZE, FLASH_SIZE),
            flight_headers: Deque::new(),
            is_second_config: true,
        }
    }

    pub(crate) async fn debug(&mut self) -> Result<(), SirinError> {
        let reg1 = self.w25q.read_status_reg1().await.unwrap();
        let reg2 = self.w25q.read_status_reg2().await.unwrap();
        let reg3 = self.w25q.read_status_reg3().await.unwrap();

        info!("W25Q32 Status Registers: 1[{:b}], 2[{:b}], 3[{:b}]", reg1, reg2, reg3);

        let mut buf = [0u8; 512];

        self.w25q.read(0, &mut buf).await?;
        println!("First sector: {:x}", buf[0..256]);

        self.w25q.read(4096, &mut buf).await?;
        println!("Second sector: {:x}", buf[0..256]);

        Ok(())
    }

    pub(crate) async fn init(&mut self) -> Result<SirinConfig, SirinError> {
        self.flight_data.init(&mut self.w25q).await?;

        self.debug().await?;

        // Read config
        let mut buf = [0u8; 512];
        self.is_second_config = true;

        self.w25q.read(4096, &mut buf).await?;

        if buf[0] != 0x77 {
            self.is_second_config = false;
            self.w25q.read(0, &mut buf).await?;
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

        match self.init_flight_headers().await {
            Ok(()) => {},
            Err(_) => {
                error!("Failed to init flight headers. Erasing.");
                self.flight_headers = Deque::new();
                self.erase_flight_headers().await?;
            }
        }

        self.new_flight().await?;

        Ok(config)
    }

    async fn init_flight_headers(&mut self) -> Result<(), SirinError> {
        for i in 0..FLIGHT_HEADER_COUNT {
            let mut buf = [0u8; FlightHeader::SONG_SIZE];
            self.w25q.read(flight_header_index_to_addr(i), &mut buf).await?;
            
            if buf[0] == FlightHeaderStatus::Valid as u8 {
                // the `set` method will take care of setting up the deque
                // for us
                self.flight_headers.set(i, FlashFlightHeader {
                    index: i,
                    header: FlightHeader::from_song(&buf)?
                }).map_err(|_| SirinError::CorruptedData)?;
            }
        }

        info!("Read these flight headers: {:#}", Debug2Format(&self.flight_headers));

        Ok(())
    }

    pub async fn erase_flight_headers(&mut self) -> Result<(), SirinError> {
        info!("Erasing flight headers");
        self.w25q.checked_erase_sector(SECTOR_SIZE * 2).await?;
        self.w25q.checked_erase_sector(SECTOR_SIZE * 3).await?;
        Ok(())
    }

    async fn invalidate_flight_header(&mut self, index: usize) -> Result<(), SirinError> {
        info!("Invalidating flight header {}", index);
        self.w25q.write(flight_header_index_to_addr(index), &[0]).await?;
        Ok(())
    }

    pub async fn new_flight(&mut self) -> Result<&FlightHeader, SirinError> {
        let header = FlashFlightHeader {
            index: self.flight_headers.back().map(|h| (h.index + 1) % FLIGHT_HEADER_COUNT).unwrap_or(0),
            header: FlightHeader::new(
                self.flight_data.cursor(),
                absolute_time_reference(),
                Instant::now().as_ticks()
            )
        };

        info!("New flight header: {:?}", Debug2Format(&header));

        let addr = flight_header_index_to_addr(header.index);

        if addr % SECTOR_SIZE == 0 {
            info!("Erasing flight header sector {}", addr);
            self.w25q.erase_sector(addr).await?;
            self.w25q.until_ready().await?;
            let mut buf = [0; 256];
            self.w25q.read(addr, &mut buf).await?;
            info!("Erased sector: [{}]", buf);
        }

        let mut buf = [0u8; FlightHeader::SONG_SIZE];
        assert!(header.header.song_size() == FlightHeader::SONG_SIZE);
        header.header.to_song(&mut buf).unwrap();
        self.w25q.checked_write(addr, &buf).await?;
        info!("Wrote flight header to {}", addr);

        if self.flight_headers.is_full() {
            self.flight_headers.pop_front();
        }

        self.flight_headers.push_back(header).unwrap();

        Ok(&self.flight_headers.back().unwrap().header)
    }

    pub fn active_flight_header(&self) -> &FlashFlightHeader {
        self.flight_headers.back().unwrap()
    }

    pub async fn set_absolute_time_reference(&mut self, reference: AbsoluteTimeReference) -> Result<(), SirinError> {
        let header = self.flight_headers.back_mut().unwrap();
        header.header.time_reference = MaybeUnwrittenMaxBytes(Some(reference.clone()));
        let mut buf = [0; FlightHeader::SONG_SIZE];
        header.header.to_song(&mut buf)?;

        self.w25q.write(
            flight_header_index_to_addr(header.index),
            &buf
        ).await?;

        Ok(())
    }

    pub async fn log(&mut self, packet: &LogPacket) -> Result<(), SirinError> {
        let mut data = [0u8; MAX_OUT_PACKET_SIZE];
        packet.to_song(&mut data)?;
        let result = self.flight_data.append(&mut self.w25q, &data[0..packet.song_size()]).await?;

        //info!("Wrote absolute address {}: {:x}", result.addr, &data[0..packet.song_size()]);

        // Check if we just overwrote an old log. If we did, invalidate it.
        if let Some(sector) = result.sector_erased {
            loop {
                if self.flight_headers.len() <= 1 {
                    break;
                }

                // use a code block to drop the borrow on the deque so we can pop later
                let (index, addr) = {
                    let Some(FlashFlightHeader { index, header }) = self.flight_headers.front() else {
                        break;
                    };

                    (*index, header.data_addr() + self.flight_data.region_start)
                };

                // check if it's inside the erased sector
                if sector <= addr && addr < sector + SECTOR_SIZE {
                    self.flight_headers.pop_front();
                    self.invalidate_flight_header(index).await?;
                } else {
                    break;
                }
            }
        }

        Ok(())
    }

    pub fn read_logs(&mut self, header: &FlashFlightHeader) -> FlashFlightPacketsIterator<'_> {
        let addr_of_next_header = self.flight_headers.get((header.index + 1) % self.flight_headers.len()).unwrap().header.data_addr();
        
        FlashFlightPacketsIterator {
            flash: self,
            rel_addr: header.header.data_addr(),
            rel_addr_of_next_header: addr_of_next_header,
            is_done: false
        }
    }

    pub async fn save_config(&mut self, config: &SirinConfig) -> Result<(), SirinError> {
        let mut buf = [0u8; 512];
        buf[0] = 0x77;
        // TODO remove unwrap
        config.to_song(&mut buf[1..]).unwrap();

        if self.is_second_config {
            self.w25q.checked_erase_sector(0).await?;
            self.w25q.checked_write(0, &buf[0..config.song_size() + 1]).await?;
            self.w25q.checked_erase_sector(4096).await?;
        } else {
            self.w25q.checked_erase_sector(4096).await?;
            self.w25q.checked_write(4096, &buf[0..config.song_size() + 1]).await?;
            self.w25q.checked_erase_sector(0).await?;
        }

        Ok(())
    }
}

pub struct FlashFlightPacketsIterator<'a> {
    flash: &'a mut Flash,

    // Not including offset!
    rel_addr: u32,

    // Not including offset!
    rel_addr_of_next_header: u32,

    is_done: bool
}

impl <'a> FlashFlightPacketsIterator<'a> {
    pub async fn next(&mut self) -> Option<Result<LogPacket, SirinError>> {
        let size = self.flash.flight_data.data_subregion_size;
        let offset = self.flash.flight_data.region_start;

        if self.is_done || self.rel_addr >= self.rel_addr_of_next_header + size {
            self.is_done = true;
            return None;
        }

        let mut buf = [0; MAX_OUT_PACKET_SIZE];

        loop {
            //info!("Ran iter loop");
            let addr = offset + (self.rel_addr % size);
            match self.flash.w25q.read(addr, &mut buf).await {
                Ok(..) => {},
                Err(e) => return Some(Err(e.into()))
            }

            if buf[0] == 0xFE {
                // Go to next sector
                info!("Going to next sector");
                self.rel_addr = (self.rel_addr / SECTOR_SIZE + 1) * SECTOR_SIZE;
                continue;
            } else if buf[0] == 0xFF {
                // We're done reading.
                self.is_done = true;
                return None
            } else if buf[0] == 0x00 {
                Timer::after_millis(150).await;
                info!("Buf: {:x}", &buf);
                panic!("Couldn't read flight data, addr: {}", addr)
            }

            let log = match LogPacket::from_song(&buf) {
                Ok(p) => p,
                Err(e) => {
                    self.is_done = true;
                    return Some(Err(e.into()))
                }
            };

            self.rel_addr += log.song_size() as u32;
            return Some(Ok(log))
        }
        
    }
}

fn flight_header_index_to_addr(i: usize) -> u32 {
    let i = i as u32;
    let headers_per_sector = FLIGHT_HEADER_COUNT as u32 / 2;

    2 * SECTOR_SIZE + i / headers_per_sector * SECTOR_SIZE + (i as u32 % headers_per_sector) * FlightHeader::SONG_SIZE as u32
}