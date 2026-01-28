#![no_std]
use dev_csr::dev_csr;
use embedded_hal::spi::ErrorType;
use embedded_hal_async::spi::SpiBus;
use spi_handle::SpiHandle;

dev_csr!{
    dev Lis3mdl {
        regs {
            /// Should be 32h
            0x0F WHO_AM_I r who_am_i,
            0x20 CTRL_REG1 rw {
                /// Default 0
                /// 0 disabled, 1 enabled
                0 self_test,
                /// Defualt 0
                /// 0 disabled, 1 enabled
                /// FAST_ODR enables data rates higher than 80 Hz
                1 fast_odr,
                /// Output data rate selection
                /// 000 -> 0.625 Hz
                /// 001 -> 1.25 Hz
                /// 010 -> 2.5 Hz
                /// 011 -> 5 Hz
                /// 100 -> 10 Hz
                /// 101 -> 20 Hz
                /// 110 -> 40 Hz
                /// 111 -> 80 Hz
                2..4 data_output_rate,
                /// 00 low-power mode
                /// 01 medium-performance mode
                /// 10 high performance mode
                /// 11 ultra-high performance mode
                5..6 operative_mode,
                /// Default 0
                /// 0 disabled, 1 enabled
                7 temperature_enable
            },
            0x21 CTRL_REG2 rw {
                /// bits 0,1,4,7 must be set to 0
                /// 0 default
                /// 1 reset operation
                2 soft_rs,
                /// Reboot memory content
                /// Defualt 0
                /// 0 normal mode, 1 reboot memory content
                3 reboot,
                /// 00 +- 4 guass (default)
                /// 01 +- 8 guass
                /// 10 +- 12 gauss
                /// 11 +- 16 gauss
                5..6 full_scale_config
            },
            0x22 CTRL_REG3 rw{
                /// bits 3,4,6,7 must be set to 0
                /// 11 power-down mode (default)
                /// 10 power-down mode
                /// 01 single-conversion mode (Has to be used with sampling frequency from 0.625 Hz to 80 Hz)
                /// 00 continuous_conversion mode
                0..1 mode_selection,
                /// SPI serial interface mode selection
                /// Defualt 0
                /// 0 4-wire interface, 1 3-wire interface
                2 spi_mode_selection,
                /// Default 0
                /// If this bit is ‘1’, DO[2:0] is set to 0.625 Hz and the system performs, for each channel, the minimum number of averages. 
                /// Once the bit is set to ‘0’, the magnetic data rate is configured by the DO bits in CTRL_REG1 (20h) register.
                5 low_power_mode_config
            },
            0x23 CTRL_REG4 rw{
                /// bits 0,4,5,6,7 must be set to 0
                /// Big/Little Endian data selection. 
                /// Default 0 
                /// 0: data LSb at lower address, 1: data MSb at lower address)
                1 big_little_endian_data_selction,
                /// 00 lower_power moddefault 
                /// 01 medium-performance mode
                /// 10 high_performance_mode
                /// 11 ultra_high_performance mode 
                2..3 z_axis_operating_mode
            },
            0x24 CTRL_REG5 rw{
                /// bits 0..5 must be set to 0
                /// Block data update for magnetic data. 
                /// Default value: 0 
                /// 0: continuous update, 1: output registers not updated until MSb and LSb have been read
                6 block_data_update,
                /// 0: continous update (default)
                /// 1: output registers not updated until MSb and LSb have been read
                7 fast_read
            },
            
            0x27 STATUS_REG r {
                /// Default value: 0
                /// (0: no new data ready; 1: new data available)
                0 x_data_available,
                1 y_data_available,
                2 z_data_available,
                /// (0: new set of data not available; 1: a new set is available)
                3 xyz_data_available,
                /// (0: no overrun has occurred; 1: a new data for the same axis has overwritten the previous data)
                4 x_overrun,
                5 y_overrun,
                6 z_overrun,
                /// (0: no overrun has occurred; 1: new data has overwritten the previous data before it was read)
                7 xyz_overrun
            }, 

            /// X-axis data output. The value of magnetic field is expressed as two’s complement.
            0x28 OUT_X_L r x_l[0..7],
            0x29 OUT_X_H r x_h[0..7],

            /// Y-axis data output. The value of magnetic field is expressed as two’s complement.
            0x2A OUT_Y_L r y_l [0..7],
            0x2B OUT_Y_H r y_h [0..7],

            /// Z-axis data output. The value of magnetic field is expressed as two’s complement.
            0x2C OUT_Z_L r z_l[0..7],
            0x2D OUT_Z_H r z_h[0..7],
            
            /// Temperature sensor data. The value of temperature is expressed as two’s complement.
            0x2E TEMP_OUT_L r temp_out_l[0..7], 
            0x2F TEMP_OUT_H r temp_out_h[0..7],
            
            0x30 INT_CFG rw {
                /// Default value: 0
                /// (0: disable interrupt request; 1: enable interrupt request on measured accel. value lower/higher than preset threshold)
                0 enable_interrupt_generation_x_low_event_int1,
                1 enable_interrupt_generation_x_high_event_int1,
                2 enable_interrupt_generation_y_low_event_int1,
                3 enable_interrupt_generation_y_high_event_int1,
                4 enable_interrupt_generation_z_low_event_int1,
                5 enable_interrupt_generation_z_high_event_int1,
                /// (0: OR combination of interrupt events; 1:  AND combination of interrupt events)
                7 and_or_combinayion_of_interrupt_events_int1,
                /// bits 3 must be 1 and 4 must be 0
                /// Interrupt enable on INT pin. 
                /// Default value: 0 (0: disabled; 1: enabled)
                0 interrupt_enable,
                /// Latch interrupt request. 
                /// Default value: 0 
                /// 0: interrupt request latched; 1: interrupt request not latched) Once latched, the INT pin remains in the same state until INT_SRC (31h) is read
                1 latch_interrupt_request,
                /// 0 low (default)
                /// 1 high 
                2 interrupt_active_configuration,
                /// 0: disable interrupt request; 1: enable interrupt request
                5 enable_interrupt_generation_z,
                /// 0: disable interrupt request; 1: enable interrupt request
                6 enable_interrupt_generation_y,
                /// 0: disable interrupt request; 1: enable interrupt request
                7 enable_interrupt_generation_x
            },
            0x31 INT_SRC r {
                /// This bit signals when an interrupt event occurs
                0 interupt_event,
                /// Internal measurement range overflow on magnetic value. Default value: 0
                1 internal_measurement_overflow,
                /// Value exceeds the threshold on the negative side
                /// Default 0
                2 z_neg_event,
                3 y_neg_event,
                4 x_neg_event,
                /// Value exceeds the threshold on the positive side
                /// Default 0
                5 z_pos_event,
                6 y_pos_event,
                7 x_pos_event
            },
            
            /// Default value
            /// The value is expressed in 16-bit unsigned. 
            0x32 INT_THS_L rw {
                0 ths0, 
                1 ths1,
                2 ths2,
                3 ths3,
                4 ths4,
                5 ths5,
                6 ths6,
                7 ths7
            },

            0x33 INT_THS_H rw {
                /// bit 7 must be set to 0
                0 ths8,
                1 ths9,
                2 ths10,
                3 ths11,
                4 ths12,
                5 ths13,
                6 ths14
            }
        }
    }
}

pub struct Lis3mdl<S: SpiHandle> {
    spi: S
}

impl <S: SpiHandle> Lis3mdl<S> {
    pub fn new(spi: S) -> Self {
        Self {
            spi
        }
    }

    pub async fn setup(
        &mut self
    ) -> Result<(),<S::Bus as ErrorType>::Error> {
        // self.write_reg(reg, value as u8).await?;
        // self.write_reg(CTRL_REG1, 0b1011_0000 as u8).await?;

        self.write_reg(RegCtrlReg1, 0b1_10_100_0_0 as u8).await?;
        self.write_reg(RegCtrlReg3, 0b00000000 as u8).await?;
        self.write_reg(RegCtrlReg4, 0b0000_10_00 as u8).await?;
        //self.write_reg().await?;
        Ok(())    
    }
    // 
    pub async fn magnetic(&mut self) -> Result<(i16,i16,i16), <S::Bus as ErrorType>::Error> {
        //MSB stored in the low register
        let mag_x = i16::from_ne_bytes([self.x_l().await? as u8,self.x_h().await? as u8]);
        let mag_y = - i16::from_ne_bytes([self.y_l().await? as u8,self.y_h().await? as u8]);
        let mag_z = i16::from_ne_bytes([self.z_l().await? as u8,self.z_h().await? as u8]);
        Ok((mag_x, mag_y, mag_z))
    }
    
    pub async fn temp(&mut self) -> Result<i16, <S::Bus as ErrorType>::Error> {
         Ok(
            i16::from_ne_bytes([self.temp_out_l().await? as u8, self.temp_out_h().await? as u8])
       )
    }

    pub async fn manufacturer_id(&mut self) -> Result<u8, <S::Bus as ErrorType>::Error> {
        Ok(self.who_am_i().await?)
    }

}

impl <S: SpiHandle> ReadLis3mdl for Lis3mdl<S>{
    type Error = <S::Bus as ErrorType>::Error;

    async fn read_contiguous_regs(
        &mut self,
        addr: impl ReadableAddr,
        out: &mut [u8]
    ) -> Result<(), Self::Error> {
        let mut bus = self.spi.select().await;
        // bit 0: READ bit. The value is 1. 
        // bit 1: MS bit. When 0, does not increment the address. When 1, increments the address in multiple reads. 
        // bit 2-7: address AD(5:0). This is the address field of the indexed register.
        // bit 8-15: data DO(7:0) (read mode). This is the data that is read from the device (MSB first). 
        // bit 16-... : data DO(...-8). Further data in multiple byte reads.

        // set rw bit
        
        // write = 1, read = 0
        
        // If broken try | 0b1100_0000;
        let addr: u8 = addr.as_addr() | 0b1100_0000;
        
        bus.write(&[addr]).await?;
        bus.transfer_in_place(out).await?;
        Ok(())
    }

}

impl <S: SpiHandle> WriteLis3mdl for Lis3mdl<S>{
    type Error = <S::Bus as ErrorType>::Error;

    async fn write_contiguous_regs(
        &mut self,
        addr: impl WritableAddr,
        values: &[u8]
    ) -> Result<(), Self::Error> {
        let mut bus = self.spi.select().await;
        // The SPI Write command is performed with 16 clock pulses. 
        // A multiple byte write command is performed by adding blocks of 8 clock pulses to the previous one. 
        // bit 0: WRITE bit. The value is 0. 
        // bit 1: MS bit. When 0, does not increment the address; when 1, increments the address in multiple writes. 
        // bit 2 -7: address AD(5:0). This is the address field of the indexed register. 
        // bit 8-15: data DI(7:0) (write mode). This is the data that is written inside the device (MSb first). 
        // bit 16-... : data DI(...-8). Further data in multiple byte writes.

        // If broken try & 0b0011_1111;
        let addr: u8 = addr.as_addr() & 0b0011_1111;

        bus.write(&[addr.as_addr()]).await?;
        bus.write(values).await?;

        Ok(())
    }

}