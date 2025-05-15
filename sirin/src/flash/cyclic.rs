use core::{future::Future, ops::{Range, RangeBounds}};

use defmt::info;
use embassy_time::Timer;
use embedded_hal::spi::ErrorKind;
use w25qx::W25Q;

use crate::{spi::{SpiDev, SpiError}, SirinConfig};

const SECTOR_SIZE: u32 = 4096;
const PAGE_SIZE: u32 = 256;

pub(crate) struct CyclicFlashSection {
    pub(crate) region_start: u32,

    /// exclusive
    region_end: u32,

    data_sector_index: u32,
    data_byte_index: u32,

    pub(crate) data_subregion_size: u32,
    pub(crate) bitmap_subregion_size: u32,
}

#[derive(Debug, Clone)]
pub struct AppendResult {
    /// Address of the written data not including offset
    pub index: u32,

    /// Address of the written data including offset (e.g. absolute address)
    pub addr: u32,

    /// The sector that was erased, if any
    pub sector_erased: Option<u32>
}

impl CyclicFlashSection {
    /// Must be multiples of sector size
    pub fn new(region_start: u32, region_end: u32) -> Self {
        // Every 4096 * 8 data sectors corresponds to one pointer sector.
        let region_size = region_end - region_start;
        let data_size = (
            (((region_size as u64) * (SECTOR_SIZE * 8) as u64) / (SECTOR_SIZE * 8 + 1) as u64)
            / SECTOR_SIZE as u64 * SECTOR_SIZE as u64
        ) as u32;
        let bitmap_subregion_size = (data_size / SECTOR_SIZE).div_ceil(8).div_ceil(SECTOR_SIZE) * SECTOR_SIZE;

        Self {
            region_start,
            region_end,
            data_sector_index: 0,
            data_byte_index: 0,
            data_subregion_size: data_size,
            bitmap_subregion_size
        }
    }

    pub async fn init(&mut self, flash: &mut W25Q<SpiDev>) -> Result<(), ErrorKind> {
        let i = self.search_data_sector_index_from_bitmap(flash).await?;
        self.data_sector_index = i;

        info!("Starting at {}", self.cursor());
        Timer::after_millis(1000).await;

        Ok(())
    }

    /// Get the current cursor address (Does not include offsets!)
    pub fn cursor(&self) -> u32 {
        self.data_byte_index + SECTOR_SIZE * self.data_sector_index
    }

    pub fn sector_index_byte_addr(&self, index: u32) -> u32 {
        let offset = self.region_start + self.data_subregion_size;
        offset + index / 8
    }

    pub fn increment_sector_index(&mut self) {
        self.data_sector_index = (self.data_sector_index + 1) % (self.data_subregion_size / SECTOR_SIZE);
    }

    // assumption: you will not write a lower sector index unless its 0
    async fn write_sector_index(&self, flash: &mut W25Q<SpiDev>) -> Result<(), ErrorKind> {
        let offset = self.region_start + self.data_subregion_size;

        if self.data_sector_index + 1 == self.data_subregion_size / SECTOR_SIZE {
            for i in 0..(self.bitmap_subregion_size / SECTOR_SIZE) {
                flash.checked_erase_sector(offset + i * SECTOR_SIZE).await?;
            }
        } else {
            let prev = self.sector_index_byte_addr(self.data_sector_index);
            let curr = self.sector_index_byte_addr(self.data_sector_index + 1);

            if prev != curr {
                flash.write(prev, &[0]).await?;
            }

            flash.write(
                curr,
                &[(0xFF >> ((self.data_sector_index + 1) % 8))]
            ).await?;
        }

        Ok(())
    }

    /// binary search for the address of the sector index
    pub async fn search_data_sector_index_from_bitmap(&self, flash: &mut W25Q<SpiDev>) -> Result<u32, ErrorKind> {
        let mut upper = self.bitmap_subregion_size;
        let mut lower = 0;
        let offset = self.data_subregion_size + self.region_start;

        loop {
            let i = (upper + lower) / 2;

            let mut buf = [0u8];
            flash.read(offset + i, &mut buf).await?;

            if buf[0] == 0x00 {
                // too low
                lower = (i + 1).min(lower + 1);
            } else if buf[0] == 0xFF {
                // too high
                upper = i.max(upper - 1);
            } else {
                return Ok(i * 8 + buf[0].count_zeros());
            }

            if lower == upper {
                return Ok(upper * 8);
            }
        }
    }

    /// Append to the end of the cyclic buffer, deleting old sectors if necessary.
    /// Returns the address of what was written 
    pub async fn append(&mut self, flash: &mut W25Q<SpiDev>, data: &[u8]) -> Result<AppendResult, ErrorKind> {
        // do not write across two sectors
        if data.len() as u32 + self.data_byte_index > SECTOR_SIZE {
            if self.data_byte_index < SECTOR_SIZE {
                flash.checked_write(
                    self.region_start + self.data_sector_index * SECTOR_SIZE + self.data_byte_index,
                    &[0xFE]
                ).await?;
            }

            self.data_byte_index = 0;
            self.increment_sector_index();
        }

        let sector_erased;

        if self.data_byte_index == 0 {
            self.write_sector_index(flash).await?;
            let sector_addr = self.region_start + self.data_sector_index * SECTOR_SIZE;
            flash.checked_erase_sector(sector_addr).await?;
            sector_erased = Some(sector_addr);
        } else {
            sector_erased = None;
        }

        let index = self.data_sector_index * SECTOR_SIZE + self.data_byte_index;
        let addr = self.region_start + index;
        flash.checked_write(addr, data).await?;

        // due to the bounds check at the beginning this will surely be less the max
        self.data_byte_index += data.len() as u32;
        
        Ok(AppendResult {
            addr,
            index,
            sector_erased
        })
    }
}
