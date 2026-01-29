use embedded_hal_async::spi::ErrorKind;

use crate::yield_now;

/// Mock W25Q NOR flash chip for testing
pub struct MockW25Q<'a> {
    memory: &'a mut [u8],
    busy: bool,
    write_enabled: bool,
}

impl<'a> MockW25Q<'a> {
    /// Create a new mock flash chip with the given memory buffer
    /// 
    /// # Arguments
    /// * `memory` - A mutable slice representing the flash memory (should be initialized to 0xFF)
    pub fn new(memory: &'a mut [u8]) -> Self {
        Self {
            memory,
            busy: false,
            write_enabled: false,
        }
    }

    /// Helper to simulate operation delay
    async fn simulate_busy(&mut self, duration_yields: u32) {
        self.busy = true;
        for _ in 0..duration_yields {
            yield_now::yield_now().await;
        }
        self.busy = false;
    }

    pub async fn read_status_reg1(&mut self) -> Result<u8, ErrorKind> {
        let busy_bit = if self.busy { 1 } else { 0 };
        let wel_bit = if self.write_enabled { 2 } else { 0 };
        Ok(busy_bit | wel_bit)
    }

    pub async fn read_status_reg2(&mut self) -> Result<u8, ErrorKind> {
        Ok(0x00) // Mock implementation
    }

    pub async fn read_status_reg3(&mut self) -> Result<u8, ErrorKind> {
        Ok(0x00) // Mock implementation
    }

    pub async fn is_busy(&mut self) -> Result<bool, ErrorKind> {
        Ok(self.read_status_reg1().await? % 2 == 1)
    }

    /// Returns a future that completes when the chip is ready (not BUSY)
    pub async fn until_ready(&mut self) -> Result<(), ErrorKind> {
        while self.is_busy().await? {
            yield_now::yield_now().await;
        }
        Ok(())
    }

    pub async fn write_enable(&mut self) -> Result<(), ErrorKind> {
        self.write_enabled = true;
        Ok(())
    }

    pub async fn prepare_write(&mut self) -> Result<(), ErrorKind> {
        self.until_ready().await?;
        self.write_enable().await?;
        Ok(())
    }

    pub async fn page_program(&mut self, addr: u32, buf: &[u8]) -> Result<u32, ErrorKind> {
        self.prepare_write().await?;

        if !self.write_enabled {
            panic!("Write enable not set!");
        }

        let addr = addr as usize;
        
        // Check bounds
        if addr + buf.len() > self.memory.len() {
            panic!("Write would exceed memory bounds! addr={}, len={}, memory_size={}", 
                   addr, buf.len(), self.memory.len());
        }

        // Check that we're only writing within a single page (256 bytes)
        let page_start = addr / 256 * 256;
        let page_end = page_start + 256;
        if addr + buf.len() > page_end {
            panic!("Page program would cross page boundary! addr={}, len={}, page_end={}", 
                   addr, buf.len(), page_end);
        }

        // Verify all bytes being written to are erased (0xFF)
        for (i, &byte) in buf.iter().enumerate() {
            let mem_addr = addr + i;
            let current_value = self.memory[mem_addr];
            
            // In NOR flash, you can only change 1s to 0s (0xFF -> anything)
            // Check if we're trying to write to a non-erased location
            if current_value != 0xFF {
                // Check if the write would actually change the value
                let new_value = current_value & byte; // NOR flash ANDs bits
                if new_value != current_value {
                    panic!(
                        "Attempted to write to non-erased location!\n\
                         Address: 0x{:06X}\n\
                         Current value: 0x{:02X}\n\
                         Attempted write: 0x{:02X}\n\
                         You must erase before writing!",
                        mem_addr, current_value, byte
                    );
                }
            }
        }

        // Perform the write (simulate NOR flash AND operation)
        for (i, &byte) in buf.iter().enumerate() {
            let mem_addr = addr + i;
            self.memory[mem_addr] &= byte;
        }

        // Clear write enable latch
        self.write_enabled = false;

        // Simulate programming time
        self.simulate_busy(2).await;

        Ok(buf.len() as u32)
    }

    /// Write `buf` to the flash, possibly spanning multiple pages
    pub async fn write(&mut self, addr: u32, buf: &[u8]) -> Result<u32, ErrorKind> {
        let addr = addr as usize;
        let mut i = 0;

        while i < buf.len() {
            let page_end = ((addr + i) / 256 + 1) * 256;
            let section_size = (page_end - (addr + i)).min(buf.len() - i);
            
            self.page_program((addr + i) as u32, &buf[i..(i + section_size)]).await?;
            i += section_size;
        }

        Ok(buf.len() as u32)
    }

    /// For debugging, check to see if what you wrote is what's on the chip. This will panic if you
    /// accidentally make an invalid write.
    pub async fn checked_write(&mut self, addr: u32, buf: &[u8]) -> Result<u32, ErrorKind> {
        let addr_usize = addr as usize;
        
        // Check if bytes are erased
        if !self.memory[addr_usize..(addr_usize + buf.len())].iter().all(|b| *b == 0xFF)
        {
            panic!(
                "Bytes at 0x{:06X} are not erased!\nData: {:02X?}",
                addr,
                &self.memory[addr_usize..(addr_usize + buf.len())]
            );
        }

        self.write(addr, buf).await?;

        // Verify write
        if buf != &self.memory[addr_usize..(addr_usize + buf.len())] {
            panic!(
                "Bytes at 0x{:06X} are not the same as what was written!\n\
                 Data:    {:02X?}\n\
                 Written: {:02X?}",
                addr,
                &self.memory[addr_usize..(addr_usize + buf.len())],
                buf
            );
        }

        Ok(addr)
    }

    pub async fn erase_sector(&mut self, addr: u32) -> Result<(), ErrorKind> {
        self.prepare_write().await?;

        let addr = addr as usize;
        let sector_start = addr / 4096 * 4096;
        let sector_end = sector_start + 4096;

        if sector_end > self.memory.len() {
            panic!("Sector erase would exceed memory bounds!");
        }

        // Erase the sector (set all bytes to 0xFF)
        for byte in &mut self.memory[sector_start..sector_end] {
            *byte = 0xFF;
        }

        // Clear write enable latch
        self.write_enabled = false;

        // Simulate erase time (longer than program)
        self.simulate_busy(10).await;

        Ok(())
    }

    /// Will panic if the sector isn't actually erased.
    pub async fn checked_erase_sector(&mut self, addr: u32) -> Result<(), ErrorKind> {
        if addr % 4096 != 0 {
            panic!("Addr {} is not divisible by 4096!", addr);
        }

        self.erase_sector(addr).await?;

        let addr_usize = addr as usize;
        let sector_end = addr_usize + 4096;

        if !self.memory[addr_usize..sector_end].iter().all(|b| *b == 0xFF)
        {
            panic!("Sector 0x{:06X} was not erased!", addr);
        }

        Ok(())
    }

    pub async fn read(&mut self, addr: u32, words: &mut [u8]) -> Result<(), ErrorKind> {
        self.until_ready().await?;

        let addr = addr as usize;

        if addr + words.len() > self.memory.len() {
            panic!("Read would exceed memory bounds!");
        }

        words.copy_from_slice(&self.memory[addr..(addr + words.len())]);

        Ok(())
    }

    pub async fn chip_erase(&mut self) -> Result<(), ErrorKind> {
        self.prepare_write().await?;

        // Erase entire chip
        for byte in self.memory.iter_mut() {
            *byte = 0xFF;
        }

        // Clear write enable latch
        self.write_enabled = false;

        // Simulate chip erase time (very long)
        self.simulate_busy(100).await;

        Ok(())
    }

    pub async fn erase_64kb_block(&mut self, addr: u32) -> Result<(), ErrorKind> {
        self.prepare_write().await?;

        let addr = addr as usize;
        let block_start = addr / 65536 * 65536;
        let block_end = block_start + 65536;

        if block_end > self.memory.len() {
            panic!("64KB block erase would exceed memory bounds!");
        }

        // Erase the block
        for byte in &mut self.memory[block_start..block_end] {
            *byte = 0xFF;
        }

        // Clear write enable latch
        self.write_enabled = false;

        // Simulate erase time
        self.simulate_busy(30).await;

        Ok(())
    }

    pub async fn erase_32kb_block(&mut self, addr: u32) -> Result<(), ErrorKind> {
        self.prepare_write().await?;

        let addr = addr as usize;
        let block_start = addr / 32768 * 32768;
        let block_end = block_start + 32768;

        if block_end > self.memory.len() {
            panic!("32KB block erase would exceed memory bounds!");
        }

        // Erase the block
        for byte in &mut self.memory[block_start..block_end] {
            *byte = 0xFF;
        }

        // Clear write enable latch
        self.write_enabled = false;

        // Simulate erase time
        self.simulate_busy(20).await;

        Ok(())
    }

    pub async fn suspend(&mut self) -> Result<(), ErrorKind> {
        // Mock implementation - in real hardware this would suspend erase/program
        Ok(())
    }

    pub async fn resume(&mut self) -> Result<(), ErrorKind> {
        // Mock implementation - in real hardware this would resume erase/program
        Ok(())
    }

    pub async fn read_device_id(&mut self) -> Result<u8, ErrorKind> {
        // Return a mock device ID (0x17 is common for W25Q128)
        Ok(0x17)
    }
}