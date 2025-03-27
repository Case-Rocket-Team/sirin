use dev_csr::dev_csr;
use embedded_hal::spi::ErrorType;
use embedded_hal_async::spi::SpiBus;
use serde::{Deserialize, Serialize};
use spi_handle::SpiHandle;
use uunit::{Celsius, Pascals};

use crate::compensation::{Compensator, RawTrimData};

dev_csr! {
    dev Bmp3 {
        regs {
            /// Example: `0x50`.
            0x00 CHIP_ID r chip_id,
            0x02 ERR_REG r {
                0 fatal_err,
                1 cmd_err,
                2 conf_err
            },
            0x03 STATUS r {
                4 cmd_rdy,
                5 drdy_press,
                6 drdy_temp
            },
            0x04 DATA_0 r raw_pressure[0..7],
            0x05 DATA_1 r raw_pressure[8..15],
            0x06 DATA_2 r raw_pressure[16..23],
            0x07 DATA_3 r raw_temperature[0..7],
            0x08 DATA_4 r raw_temperature[8..15],
            0x09 DATA_5 r raw_temperature[16..23],
            0x0C SENSORTIME_0 r raw_sensor_time[0..7],
            0x0D SENSORTIME_1 r raw_sensor_time[8..15],
            0x0E SENSORTIME_2 r raw_sensor_time[16..23],
            0x10 EVENT r {
                0 por_detected
            },
            0x11 INT_STATUS r {
                0 fwm_int,
                1 ffull_int,
                2 drdy
            },
            0x12 FIFO_LENGTH_0 r fifo_length[0..7],
            0x13 FIFO_LENGTH_1 r {
                0 fifo_length[8]
            },
            0x14 FIFO_DATA r,
            0x15 FIFO_WTM_0 rw fifo_water_mark[0..7],
            0x16 FIFO_WTM_1 rw {
                0 fifo_water_mark[8]
            },
            0x17 FIFO_CONFIG_1 rw {
                0 fifo_mode,
                1 fifo_stop_on_full,
                2 fifo_time_en,
                3 fifo_press_en,
                4 fifo_temp_en
            },
            0x18 FIFO_CONFIG_2 rw {
                0..2 fifo_subsampling,
                3..4 data_select
            },
            0x19 INT_CT_RL rw {
                0 int_od,
                1 int_level,
                2 int_latch,
                3 fwtm_en,
                4 ffull_en,
                6 drdy_en
            },
            0x1A IF_CONF rw {
                0 spi3,
                1 i2c_wdt_en,
                2 i2c_wdt_sel
            },
            0x1B PWR_CTRL rw {
                0 press_en,
                1 temp_en,
                4..5 mode,
            },
            0x1C OSR rw {
                0..2 osr_p,
                3..5 osr_t
            },
            0x1D ODR rw {
                0..4 odr_sel
            },
            0x1F CONFIG rw {
                1..3 iir_filter
            },
            0x31 CALIB_DATA r,
            0x7E CMD rw cmd
        }
    }
}

pub struct Bmp3Io<S: SpiHandle> {
    spi: S
}



impl <S: SpiHandle> Bmp3Io<S> {
    pub async fn read_raw_trim_data(&mut self) -> Result<RawTrimData, <S::Bus as ErrorType>::Error> {
        let mut data = [0u8; 21];
        self.read_contiguous_regs(RegCalibData, &mut data).await?;

        macro_rules! bytes {
            ($b1:expr, $b2:expr) => {
                ((data[$b1] as u16) << 8) | (data[$b2] as u16)
            };
        }

        // deus hoc vult
        #[allow(unused_mut)]
        let mut data: RawTrimData = unsafe {
            core::mem::transmute(data)
        };

        /*let mut data = RawTrimData {
            par_t1: bytes!(1, 0),
            par_t2: bytes!(3, 2),
            par_t3: data[4] as i8,
            par_p1: bytes!(6, 5) as i16,
            par_p2: bytes!(8, 7) as i16,
            par_p3: data[9] as i8,
            par_p4: data[10] as i8,
            par_p5: bytes!(12, 11),
            par_p6: bytes!(14, 13),
            par_p7: data[15] as i8,
            par_p8: data[16] as i8,
            par_p9: bytes!(18, 17) as i16,
            par_p10: data[19] as i8,
            par_p11: data[20] as i8
        };*/

        #[cfg(target_endian = "big")]
        data.swap_bytes();

        Ok(data)
    }

    pub async fn read_raw_data(&mut self) -> Result<Bmp3RawData, <S::Bus as ErrorType>::Error> {
        let mut data = [0u8; 6];
        self.read_contiguous_regs(RegData0, &mut data).await?;
        let raw_pressure = (data[0] as u32) + ((data[1] as u32) << 8) + ((data[2] as u32) << 16);
        let raw_temperature = (data[3] as u32) + ((data[4] as u32) << 8) + ((data[5] as u32) << 16);

        Ok(Bmp3RawData {
            raw_pressure: raw_pressure as u64,
            raw_temperature: raw_temperature as i64,
        })
    }
}

impl <S: SpiHandle> ReadBmp3 for Bmp3Io<S> {
    type Error = <S::Bus as ErrorType>::Error;

    async fn read_contiguous_regs(
        &mut self,
        addr: impl ReadableAddr,
        out: &mut [u8]
    ) -> Result<(), Self::Error> {
        let mut bus = self.spi.select().await;

        // set rw bit
        let addr = addr.as_addr() | 0b1000_0000;
        let dummy = 0;
        
        bus.write(&[addr, dummy]).await?;
        bus.transfer_in_place(out).await?;
        Ok(())
    }
}

impl <S: SpiHandle> WriteBmp3 for Bmp3Io<S> {
    type Error = <S::Bus as ErrorType>::Error;

    async fn write_contiguous_regs(
        &mut self,
        addr: impl WritableAddr,
        values: &[u8]
    ) -> Result<(), Self::Error> {
        let mut bus = self.spi.select().await;

        bus.write(&[addr.as_addr()]).await?;
        bus.write(values).await?;

        Ok(())
    }
}

pub struct Bmp3RawData {
    pub raw_pressure: u64,
    pub raw_temperature: i64,
}

#[repr(C)]
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Bmp3Readout {
    pub pressure: Pascals<f64>,
    pub temperature: Celsius<f64>
}

pub struct Bmp3<S: SpiHandle> {
    io: Bmp3Io<S>,
    compensator: Compensator
}

impl <S: SpiHandle> Bmp3<S> {
    pub async fn new(spi: S) -> Result<Self, <S::Bus as ErrorType>::Error> {
        let mut io = Bmp3Io {
            spi
        };

        Ok(Self {
            compensator: Compensator::from_raw(&io.read_raw_trim_data().await?),
            io,
        })
    }

    pub fn compensator(&self) -> &Compensator {
        &self.compensator
    }

    pub async fn read(&mut self) -> Result<Bmp3Readout, <S::Bus as ErrorType>::Error> {
        self.io().write_reg(RegPwrCtrl, 0b0011_0011).await?;

        let Bmp3RawData { raw_pressure, raw_temperature } = self.io.read_raw_data().await?;
        let temperature = self.compensator.temperature(raw_temperature);
        let pressure = self.compensator.pressure(raw_pressure, temperature);
        Ok(Bmp3Readout {
            temperature,
            pressure
        })
    }

    pub fn io(&mut self) -> &mut Bmp3Io<S> {
        &mut self.io
    }
}

#[repr(u8)]
#[allow(non_camel_case_types)]
pub enum Command {
    /// reserved. No command.
    nop = 0x00,

    /// see extmode_en_last
    extmode_en_middle = 0x34,

    /// Clears all data in the FIFO, does not change FIFO_CONFIG
    /// registers
    fifo_flush = 0xB0,

    /// Triggers a reset, all user configuration settings are overwritten
    /// with their default state
    softreset = 0xB6,
}