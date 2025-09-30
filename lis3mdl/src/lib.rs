#![no_std]
use dev_csr::dev_csr;
use embedded_hal::spi::ErrorType;
use embedded_hal_async::spi::SpiBus;
use spi_handle::SpiHandle;
use core::mem;

dev_csr!{
    dev H3lis {
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
                /// 001 ->
                /// 010 ->
                /// 011 ->
                /// 100 -> 
                /// 101 -> 20 Hz
                /// 110 -> 40 Hz
                /// 111 -> 80 Hz
                2..4 data_output_rate,
                ///
                5..6 operative_mode,
                ///
                7 temperature_enable
            },
            0x21 CTRL_REG2 rw {
                0..1 high_pass_filter_cutoff_freq,
                2 high_pass_filter_enabled_interrupt_1,
                3 high_pass_filter_enabled_interrupt_2,
                /// Filtered data selection. Default value: 0
                /// (0: internal filter bypassed; 1: data from internal filter sent to output register)
                4 filtered_data_selection,
                /// 0 = normal mode
                5..6 high_pass_filter_mode,
                /// Reboots memory content when true
                7 boot
            },
            0x22 CTRL_REG3 rw{
                /// Defualt value: 00
                /// 00: Interrupt 1 (2) source
                /// 01: Interrupt 1 source OR interrupt 2 source
                /// 10: Data ready
                /// 11: Boot running
                0..1 data_signal_on_int1_pad,
                /// (0: interrupt request not latched; 1: interrupt request latched)
                2 latch_interrupt_request_int1_src,
                /// Default value: 00
                3..4 data_signal_on_int2_pad_control_bits,
                /// (0: interrupt request not latched; 1: interrupt request latched)
                5 latch_interrupt_request_int2_src,
                /// (0: push-pull; 1: open-drain)
                6 push_pull_open_drain,
                /// (0: active high; 1: active low)
                7 interrupt_active
            },
            0x23 CTRL_REG4 rw{
                /// Default value: 0
                /// (0: 4-wire interface; 1: 3-wire interface)
                0 spi_serial_interface_mode_selection
            },
            0x24 CTRL_REG5 rw{
                /// Default value: 00
                /// (00: sleep-to-wake function is disabled; 11: Device is in low_power mode)
                0..1 turn_on_mode_selection
            },
            /// Reading at this address zeroes instantaneously the content of the internal high-pass filter. 
            /// If the high-pass filter is enabled, all three axes are instantaneously set to 0 g. 
            /// This allows the settling time of the high-pass filter to be overcome.
            0x25 HP_FILTER_RESET r hp_filter_reset,
            0x26 REFERENCE rw {
                /// Default value: 00000000
                /// Reference value for high pass filter
                0..7 reference
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
            0x29 OUT_X r x[0..7],
            0x2B OUT_Y r y [0..7],
            0x2D OUT_Z r z[0..7],
            0x30 INT1_CFG rw {
                /// Default value: 0
                /// (0: disable interrupt request; 1: enable interrupt request on measured accel. value lower/higher than preset threshold)
                0 enable_interrupt_generation_x_low_event_int1,
                1 enable_interrupt_generation_x_high_event_int1,
                2 enable_interrupt_generation_y_low_event_int1,
                3 enable_interrupt_generation_y_high_event_int1,
                4 enable_interrupt_generation_z_low_event_int1,
                5 enable_interrupt_generation_z_high_event_int1,
                /// (0: OR combination of interrupt events; 1:  AND combination of interrupt events)
                7 and_or_combinayion_of_interrupt_events_int1
            },
            0x31 INT1_SRC r {
                /// Default value: 0
                /// (0: no interrupt; 1: event has occurred)
                0 x_low_event_int1,
                1 x_high_event_int1,
                2 y_low_event_int1,
                3 y_high_event_int1,
                4 z_low_event_int1,
                5 z_high_event_int1,
                /// (0: no interrupt event has been generated; 1: one or more interrupt events have been generated)
                6 interrupt_active_int1
            },
            0x32 INT1_THS rw {
                /// Default value: 000 0000
                0..6 interrupt_1_threshold
            },
            0x33 INT1_DURATION rw {
                /// Default value: 000 0000
                /// These bits set the minimum duration of the interrupt event to be recognized.
                0..6 interrupt_1_duration
            },
            0x34 INT2_CFG rw {
                /// Default value: 0
                /// (0: disable interrupt request; 1: enable interrupt request on measured accel. value lower/higher than preset threshold)
                0 enable_interrupt_generation_x_low_event_int2,
                1 enable_interrupt_generation_x_high_event_int2,
                2 enable_interrupt_generation_y_low_event_int2,
                3 enable_interrupt_generation_y_high_event_int2,
                4 enable_interrupt_generation_z_low_event_int2,
                5 enable_interrupt_generation_z_high_event_int2,
                /// (0: OR combination of interrupt events; 1:  AND combination of interrupt events)
                7 and_or_combinayion_of_interrupt_events_int2
            },
            0x35 INT2_SRC rw {
                /// Default value: 0
                /// (0: no interrupt; 1: event has occurred)
                0 x_low_event_int2,
                1 x_high_event_int2,
                2 y_low_event_int2,
                3 y_high_event_int2,
                4 z_low_event_int2,
                5 z_high_event_int2,
                /// (0: no interrupt event has been generated; 1: one or more interrupt events have been generated)
                6 interrupt_active_int2
            },
            0x36 INT2_THS rw {
                /// Default value: 000 0000
                0..6 interrupt_2_threshold
            },
            0x37 INT2_DURATION rw {
                /// Default value: 000 0000
                /// These bits set the minimum duration of the interrupt event to be recognized.
                0..6 interrupt_2_duration
            }

        }
    }
}

pub struct H3lis<S: SpiHandle> {
    spi: S
}

impl <S: SpiHandle> H3lis<S> {
    pub fn new(spi: S) -> Self {
        Self {
            spi
        }
    }

    pub async fn setup(
        &mut self
    ) -> Result<(),<S::Bus as ErrorType>::Error> {
        // self.write_reg(reg, value as u8).await?;
        // enable x,y,z axis
        self.write_reg(RegCtrlReg1, 0b001_10_111 as u8).await?;
        //self.write_reg().await?;
        Ok(())    
    }

    pub async fn acceleration(&mut self) -> Result<(i32, i32, i32), <S::Bus as ErrorType>::Error> {
         Ok(unsafe {
            let accel_x: i8 = mem::transmute(self.x().await?);
            let accel_y: i8 = mem::transmute(self.y().await?);
            let accel_z: i8 = mem::transmute(self.z().await?);
            //xyz are corrected so that
            //x -> cable direction
            //yz follow from right hand rule, x as index finger
            ((accel_x as i32) * 780000, (accel_y as i32) * -780000, (accel_z as i32) * -780000)
       })
    }

    pub async fn manufacturer_id(&mut self) -> Result<u8, <S::Bus as ErrorType>::Error> {
        Ok(self.who_am_i().await?)
    }

}

impl <S: SpiHandle> ReadH3lis for H3lis<S>{
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
        let addr: u8 = addr.as_addr() | 0b1000_0000;
        
        bus.write(&[addr]).await?;
        bus.transfer_in_place(out).await?;
        Ok(())
    }
}

impl <S: SpiHandle> WriteH3lis for H3lis<S>{
    type Error = <S::Bus as ErrorType>::Error;

    async fn write_contiguous_regs(
        &mut self,
        addr: impl WritableAddr,
        values: &[u8]
    ) -> Result<(), Self::Error> {
        let mut bus = self.spi.select().await;

        // If broken try & 0b0011_1111;
        let addr: u8 = addr.as_addr() & 0b0111_1111;

        bus.write(&[addr.as_addr()]).await?;
        bus.write(values).await?;

        Ok(())
    }
}