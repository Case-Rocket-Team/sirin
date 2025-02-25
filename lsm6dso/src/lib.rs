#![no_std]

use core::{fmt::Debug, mem};

use dev_csr::dev_csr;
use embedded_hal::spi::{ ErrorKind as SpiError, ErrorType};
use embedded_hal_async::spi::SpiBus;
use spi_handle::SpiHandle;


dev_csr! {
    dev Lsm6dso{
        regs{
           ///enables access to some embedded functions registers
           0x01 FUNC_CFG_ACCESS rw 
           {
                ///embedded functions config register access, default 0
                7 emb_func, 
                ///sensor hub config reg access, default 0
                6 shub_reg, 
           },
           ///Pull-up control register for SDO, OCS-Aux, and SDO_Aux pins
           0x02 PIN_CTRL rw 
           {
                ///1 disables pull-up on OCS_Aux and SDO_Aux pins, 0 enables, default 0
                7 ois_pullup_L, 
                ///Enables pull-up on SDO pin, default 0
                6 sdo_pullup, 
           },
           ///FIFO watermark threshold, 1 LSB = TAG (1 byte) + 1 sensor (6 bytes) in FIFO, flag rises when #bytes in FIFO > threshold
           0x07 FIFO_CTRL1 rw fifo_wtm[1..7], 
           0x08 FIFO_CTRL2 rw
           {
                ///Limit FIFO depth to threshold level
                7 stop_fifo_on_wtm, 
                ///Enable compression
                6 fifo_compr, 
                ///Batch ODR nCHANGE sensor in FIFO
                4 fifo_batch_odrchg, 
                ///Configures the rate of written uncompressed data (default 0). 0: do not force uncompressed data writing, 1: uncompressed data every 8 batch, 2: every 16 batch, 3: every 32 batch
                1..2 fifo_uncompr_rate, 
                ///Enable FSM-triggered batching of channel 2 when available, 0 disabled, 1 enabled, default 0
                0 fifo_wtm[0], 
           },
           0x09 FIFO_CTRL3 rw
           {
                ///Select batch data rate for gyro data. 0000: gyro not batched, 0001: 1.875Hz, otherwise rate = 1.875*2^{input}, max input 1100
                4..7 fifo_gyro_bdr, 
                ///Select batch data rate for accelerometer. Same as gyro.
                0..3 fifo_accel_bdr 
           },
           0x0A FIFO_CTRL4 rw
           {
                ///Select decimation for timestamp batching. 00: timestamp not batched. 01: write rate = max(fifo_accel_bdr, fifo_gyro_bdr). 10: write rate = max(fifo_accel_bdr, fifo_gyro_bdr)/8.  11: write rate = max(fifo_accel_bdr, fifo_gyro_bdr)/32.
                6..7 fifo_ts_decim, 
                ///Select batch data rate for temperature. 00: not batched. 01: 1.875Hz. 10: 15Hz. 11: 60Hz.
                4..5 fifo_temp_bdr, 
                ///Mode selection.
                ///000: FIFO disabled.
                ///001: FIFO mode: stop collection when FIFO is full.
                ///010: continuousWTM-to-full mode: cont. mode with wtm until
                ///trigger is deasserted, then FIFO mode,
                ///011: continuous-to-FIFO mode: continuous until trigger is
                ///deasserted, then FIFO mode.
                ///100: bypass-to-continuous mode: FIFO disabled until trigger
                ///is deasserted, then continuous mode.
                ///101: reserved.
                ///110: continuous mode: if FIFO is full, new samples overwrite
                ///old ones.
                ///111: bypass-to-FIFO mode: FIFO disabled until trigger is
                ///deasserted, then FIFO mode.
                0..2 fifo_mode 
           },
           0x0B COUNTER_BDR_REG1 rw
           {
                ///Enables pulsed data-ready mode.
                ///0: data-ready latched mode (returns to 0 only after an interface reading) (default)
                ///1: data-ready pulsed mode (the data ready pulses are 75 �s long)
                7    pulsed_drdy,
                ///Select the trigger for the internal counter of batch events for the accel, gyro, and EIS gyro.
                ///00: accel batch event.
                ///01: gyro batch event.
                ///10-11: gyro EIS batch event.
                5..6 trig_ctr_bdr,
                ///Sets the threshold for the internal
                ///counter of batch events. When this counter reaches the
                ///threshold, the counter is reset and counter_bdr_reached is
                ///set to 1.
                0..2 batch_counter_thresh[0..2]
           },
           0x0C COUNTER_BDR_REG2 rw batch_counter_thresh[3..10],
           ///INT1 pin control register.
           ///Output is the OR combination of all selected here and in MD1_CFG.
           ///All bits default 0.
           0x0D INT1_CTRL rw 
           {
                ///Sends DEN_DRDY (DEN stamped on the sensor data flag) to the INT1 pin
                7 den_drdy_flag, 
                ///Enables COUNTER_BDR_IA interrupt on INT1 pin.
                6 int1_cnt_bdr, 
                ///Enables FIFO full flag interrupt on INT1 pin.
                5 int1_fifo_full, 
                ///Enables FIFO overrun interrupt on INT1 pin.
                4 int1_fifo_over, 
                ///Enables FIFO threshold interrupt on INT1 pin.
                3 int1_fifo_thresh, 
                ///Enables boot status on the INT1 pin
                2 int1_boot, 
                ///Enables gyro data-ready interrupt on INT1 pin.
                1 int1_gyro_rdy, 
                ///Enables accel data-ready interrupt on INT1 pin.
                0 int1_accel_rdy 
           },
           ///INT2 pin control register.
           ///Output is the OR combination of all selected here and in MD2_CFG.
           ///All defaults 0.
           0x0E INT2_CTRL rw 
           {
                ///Enables COUNTER_BDR_IA interrupt on INT2 pin.
                6 int2_cnt_bdr, 
                ///Enables FIFO full interrupt on INT2 pin.
                5 int2_fifo_full, 
                ///Enables FIFO overrun interrupt on INT2 pin.
                4 int2_fifo_over, 
                ///Enables FIFO threshold interrupt on INT2 pin.
                3 int2_fifo_thresh, 
                ///Enables temperature data-ready interrupt on INT2 pin.
                2 int2_temp_rdy, 
                ///Enables gyro data-ready interrupt on INT2 pin.
                1 int2_gyro_rdy, 
                ///Enables accel data-ready interrupt on INT2 pin.
                0 int2_accel_rdy 
           },
           ///whoami value. Read-only, fixed at 0x6C.
           0x0F WHO_AM_I r whoami, 
           0x10 CTRL1_XL rw
           {
                ///Accel ODR selection.
                ///When XL_HM_MODE=1 in CTRL6_C:
                ///0000: Power-down
                ///1011: 1.6Hz (low power only)
                ///0001: 12.5Hz (low power)
                ///0010: 26Hz (low power)
                ///0011: 52Hz (low power)
                ///0100: 104Hz (normal mode)
                ///0101: 208Hz (normal mode)
                ///0110: 416Hz (high performance)
                ///0111: 833Hz (high performance)
                ///1000: 1.66kHz (high performance)
                ///1001: 3.33kHz (high performance)
                ///1010: 6.66kHz (high performance)
                ///11xx: reserved
                ///Wnen XL_HM_MODE=0 in CTRL6_C:
                ///0000: Power-down
                ///1011: 12.5Hz (high performance)
                ///0001: 12.5Hz (high performance)
                ///0010: 26Hz (high performance)
                ///0011: 52Hz (high performance)
                ///0100: 104Hz (high performance)
                ///0101: 208Hz (high performance)
                ///0110: 416Hz (high performance)
                ///0111: 833Hz (high performance)
                ///1000: 1.66kHz (high performance)
                ///1001: 3.33kHz (high performance)
                ///1010: 6.66kHz (high performance)
                ///11xx: Reserved
                4..7 accel_odr_mode,
                ///Accelerometer full-scale selection
                ///When XL_FS_MODE=0 in CTRL8_XL
                ///00: ±4g (default)
                ///01: ±32
                ///10: ±8
                ///11: ±16
                2..3 accel_fs, 
                ///Accelerometer high-resolution selection
                ///0: output from first stage digital filtering selected (default)
                ///1: output from LPF2 second filtering stage selected
                1 accel_lpf2, 
           },
           0x11 CTRL2_G rw
           {
                ///Accel output data rate selection.
                ///When gyro_hm_mode=1:
                ///0000: power down (default)
                ///0001: 12.5Hz (low power)
                ///0010: 26Hz (low power)
                ///0011: 52Hz (low power)
                ///0100: 104Hz (normal)
                ///0101: 208Hz (normal)
                ///0110: 416Hz (high performance)
                ///0111: 833Hz (high performance)
                ///1000: 1.66kHz (high performance)
                ///1001: 3.33kHz (high performance)
                ///1010: 6.66kHz (high performance)
                ///others: reserved
                ///When gyro_hm_mode=0:
                ///0000: power down (high performance)
                ///0001: 12.5Hz (high performance)
                ///0010: 26Hz (high performance)
                ///0011: 52Hz (high performance)
                ///0100: 104Hz (high performance)
                ///0101: 208Hz (high performance)
                ///0110: 416Hz (high performance)
                ///0111: 833Hz (high performance)
                ///1000: 1.66kHz (high performance)
                ///1001: 3.33kHz (high performance)
                ///1010: 6.66kHz (high performance)
                ///others: reserved
                4..7 gyro_odr, 
                ///Gyroscope UI chain full-scale selection
                ///00: ±250 dp
                ///01: ±500 dp
                ///10: ±1000 dp
                ///11: ±2000 dp
                2..3 gyro_fs_select, 
                ///Selects gyro UI chain full-scale ±125 dp
                ///0: FS selected through gyro_fs_select
                ///1: FS set to ±125 dp
                1 gyro_125dps, 
           },
           0x12 CTRL3_C rw
           {
                ///Reboots memory content. Default 0, auto-cleared.
                7 reboot_mem, 
                ///Block data update. Default value: 0
                ///0: continuous update
                ///1: output registers are not updated until MSB and LSB have been read
                6 block_update, 
                ///Whether or not interrupts are active low, default 0
                5 interrupts_active_low, 
                ///Interrupt activation level. Default value: 0
                ///0: interrupt output pins active high
                ///1: interrupt output pins active low
                4 pp_od, 
                ///Push-pull/open-drain selection on the INT1 and INT2 pins. Default value: 0
                ///0: push-pull mode
                ///1: open-drain mode
                3 spi_mode_select, 
                ///Register address automatically incremented during a multiple byte access with a serial interface (I²C or SPI). Default value: 1
                ///0: disabled
                ///1: enabled
                2 inc_addrs, 
                ///Set 1 to reset software, default 0, auto-cleared.
                0 sw_reset, 
           },
           0x13 CTRL4 rw
           {
                ///Enables gyroscope sleep mode. Default value: 0
                6 gyro_sleep, 
                ///Enables all interrupt signals available on the INT1 pin. Default value: 0
                ///0: interrupt signals divided between the INT1 and INT2 pins
                ///1: all interrupt signals in logic or on the INT1 pin
                5 int2_on_int1, 
                ///Enables data available
                ///0: disabled
                ///1: mask DRDY on pin (both accelerometer and gyroscope) until filter settling ends (accelerometer and gyroscope independently masked)
                3 drdy_mask, 
                ///Disables I²C interface. Default value: 
                ///0: SPI, I²C, and MIPI I3CSM interfaces enabled(default)
                ///1: I²C interface disable
                2 disable_i2c, 
                ///Enables gyroscope digital LPF1 if the auxiliary SPI is disabled; the bandwidth can be selected through FTYPE[2:0] in CTRL6_C (15h).
                1 gyro_lpf1, 
           },
           0x14 CTRL5 rw
           {
                ///Enables accelerometer ultralow-power mode. Default value: 0
                7 accel_ulp, 
                ///Circular burst mode (wraparound) read from the output registers. Default value: 00
                ///00: no wraparound
                ///01: accelerometer only
                ///10: gyroscope only
                ///11: gyroscope + accelerometer
                5..6 read_wraparound, 
                ///Enables angular rate sensor self-test. Default value: 00
                ///00: self-test disabled
                ///01: Positive sign self-test
                ///10: Reserved
                ///11: Negative sign self-test
                2..3 gyro_selftest, 
                ///Enables linear acceleration sensor self-test. Default value: 00
                ///00: self-test disabled
                ///01: Positive sign self-test
                ///10: Negative sign self-test
                ///11: Reserved
                0..1 accel_selftest, 
           },
           0x15 CTRL6_C rw
           {
                ///Trigger mode selection for DEN
                ///100: Edge-sensitive trigger mode is selected.
                ///010: Level-sensitive trigger mode is selected.
                ///011: Level-sensitive latched mode is selected.
                ///110: Level-sensitive FIFO enable mode is selected.
                5..7 den_trigger_mode, 
                ///Disables high-performance operating mode for the accelerometer. Default value: 0
                4 accel_hp_dis, 
                ///Weight of the accelerometer user offset bits of registers X_OFS_USR (73h), Y_OFS_USR (74h), Z_OFS_USR (75h)
                ///0: 2^-10 g/LSB
                ///1: 2^-6 g/LSB
                3 accel_offset_weight,
                ///Gyroscope low-pass filter (LPF1) bandwidth selection.
                ///Table didn't fit, see datasheet
                0..2 gyro_lpf_bw, 

           },
           0x16 CTRL7_G rw
           {
                ///Disables high-performance operating mode for gyroscope. Default value: 0
                7 gyro_hp_dis, 
                ///Enables gyroscope digital high-pass filter. The filter is enabled only if the gyroscope is in HP mode. Default value: 0
                6 gyro_hpf_enable, 
                ///Gyroscope digital HP filter cutoff selection. Default: 00
                ///00: 16 mHz
                ///01: 65 mHz
                ///10: 260 mHz
                ///11: 1.04 Hz
                4..5 gyro_hpf_cutoff, 
                ///Selects how to enable and disable the OIS chain, after first configuration and enabling through SPI2.
                ///0: OIS chain is enabled/disabled with SPI2 interface
                ///1: OIS chain is enabled/disabled with primary interface
                2 ois_on_primary, 
                ///Enables accelerometer user offset correction block; it is valid for the low-pass path - see Figure 17. Accelerometer composite filter. Default value: 0
                ///0: accelerometer user offset correction block bypassed
                ///1: accelerometer user offset correction block enabled
                1 accel_offset_enable,
                ///Enables/disables the OIS chain from the primary interface when ois_on_primary is 1.
                0 ois_enable, 
           },
           0x17 CTRL8_XL rw
           {               
                ///Accelerometer LPF2 and HP filter configuration and cutoff setting.
                ///Low Pass, accel_lpf2=0: ODR/2
                ///Low Pass, accel_lpf2=1:
                ///000: ODR/4
                ///001: ODR/10
                ///010: ODR/20
                ///011: ODR/45
                ///100: ODR/100
                ///101: ODR/200
                ///110: ODR/400
                ///111: ODR/800
                ///High Pass:
                ///000: ODR/4
                ///001: ODR/10
                ///010: ODR/20
                ///011: ODR/45
                ///100: ODR/100
                ///101: ODR/200
                ///110: ODR/400
                ///111: ODR/800
                5..7 accel_hpf_cutoff,
                ///Enables accelerometer high-pass filter reference mode (valid for high-pass path - HP_SLOPE_XL_EN bit must be 1). Default value: 0
                4 accel_hp_refmode, 
                ///Enables accelerometer LPF2 and HPF fast-settling mode. The filter sets the second samples after writing this bit. Active only during device exit from power- down mode. Default value: 0
                3 accel_fastsettle, 
                ///Accelerometer slope filter / high-pass filter selection
                2 accel_slope_hp, 
                ///Accelerometer full-scale management between UI chain and OIS chain
                ///0: old full-scale mode. When XL UI is on, the full scale is the same between UI/OIS and is chosen by the UI CTRL registers; when XL UI is in PD, the OIS can choose the FS.
                ///1: new full-scale mode. Full scales are independent between the UI/OIS chain but both bound to ±8g
                1 accel_new_fs, 
                ///LPF2 on 6D function selection.
                ///0: ODR/2 low-pass filtered data sent to 6D interrupt function
                ///1: LPF2 output data sent to 6D interrupt function
                0 lpf2_on_6d, 
           },
           0x18 CTRL9_XL rw
           {
                ///DEN value stored in LSB of X-axis. Default value: 1
                7 den_x, 
                ///DEN value stored in LSB of Y-axis. Default value: 1
                6 den_y, 
                ///DEN value stored in LSB of Z-axis. Default value: 1
                5 den_z, 
                ///DEN stamping sensor selection. Default value: 0
                ///0: DEN pin info stamped in the gyroscope axis selected by bits [7:5]
                ///1: DEN pin info stamped in the accelerometer axis selected by bits [7:5]
                4 den_stamp, 
                ///Extends DEN functionality to accelerometer sensor. Default value: 0
                3 den_accel_enable, 
                ///DEN active level configuration. Default value: 0
                2 den_high, 
                ///Disables MIPI I3CSM communication protocol, default 0
                1 i3c_disable, 
           },
           0x19 CTRL10_C rw
           {
               ///Enables timestamp counter. Default value: 0
               5 timestamp_enable, 
                   
           },
           ///Source register for all interrupts
           0x1A ALL_INT_SRC r 
           {
                ///Alerts timestamp overflow within 6.4 ms
                7 timestamp_endct, 
                ///Detects change event in activity/inactivity status. Default value: 0
                5 sleep_change_ia, 
                ///Interrupt active for change in position of portrait, landscape, face-up, face-down. Default value: 0
                4 orientation_ia, 
                ///Double-tap event status. Default value: 0
                3 double_tap, 
                ///Single-tap event status. Default value: 0
                2 single_tap, 
                ///Wake-up event status. Default value: 0
                1 wake_ia, 
                ///Free-fall event status. Default value: 0
                0 freefall_ia, 
           },
           0x1B WAKE_UP_SRC r
           {
                ///Sleep status bit. Default value: 0
                4 sleep_state, 
                ///Wake-up event detection status on X-axis. Default value: 0
                2 wake_x, 
                ///Wake-up event detection status on Y-axis. Default value: 0
                1 wake_y, 
                ///Wake-up event detection status on Z-axis. Default value: 0
                0 wake_z, 
           },
           0x1C TAP_SRC r
           {
                ///Tap event detection status. Default: 0
                6 tap_ia, 
                ///Sign of acceleration detected by tap event. Default: 0
                3 tap_sign, 
                ///Tap event detection status on X-axis. Default value: 0
                2 tap_x, 
                ///Tap event detection status on Y-axis. Default value: 0
                1 tap_y, 
                ///Tap event detection status on Z-axis. Default value: 0
                0 tap_z, 
           },
           0x1D D6D_SRC r 
           {
                ///DEN data-ready signal. It is set high when the data output is related to the data coming from a DEN active condition.
                7 den_drdy, 
                ///Z-axis high event (over threshold). Default value: 0
                5 high_z, 
                ///Z-axis low event (under threshold). Default value: 0
                4 low_z, 
                ///Y-axis high event (over threshold). Default value: 0
                3 high_y, 
                ///Y-axis low event (under threshold). Default value: 0
                2 low_y, 
                ///X-axis high event (over threshold). Default value: 0
                1 high_x, 
                ///X-axis low event (under threshold). Default value: 0
                0 low_x, 
           },
           0x1E STATUS_REG r
           {
                ///temp data available
                2 temp_da, 
                ///gyro data available
                1 gyro_da, 
                ///accel data available
                0 accel_da 
           },
           ///Temp data output register
           0x20 OUT_TEMP_L r temp_data[0..7], 
           0x21 OUT_TEMP_H r temp_data[8..15],
           ///Gyro pitch axis angular rate
           0x22 OUTX_L_G r gyro_pitch_rate[0..7], 
           0x23 OUTX_H_G r gyro_pitch_rate[8..15],
           ///Gyro roll axis angular rate
           0x24 OUTY_L_G r gyro_roll_rate[0..7], 
           0x25 OUTY_H_G r gyro_roll_rate[8..15],
           ///Gyro yaw axis angular rate
           0x26 OUTZ_L_G r gyro_yaw_rate[0..7], 
           0x27 OUTZ_H_G r gyro_yaw_rate[8..15],
           ///Accel x output
           0x28 OUTX_L_A r accel_x[0..7], 
           0x29 OUTX_H_A r accel_x[8..15],
           ///Accel y output
           0x2A OUTY_L_A r accel_y[0..7], 
           0x2B OUTY_H_A r accel_y[8..15],
           ///Accel z output
           0x2C OUTZ_L_A r accel_z[0..7], 
           0x2D OUTZ_H_A r accel_z[8..15],
           ///x axis OIS/EIS
           0x2E UI_OUTX_L_G_OIS_EIS r x_ois_eis[0..7], 
           0x2F UI_OUTX_H_G_OIS_EIS r x_ois_eis[8..15],
           ///y axis OIS/EIS
           0x30 UI_OUTY_L_G_OIS_EIS r y_ois_eis[0..7], 
           0x31 UI_OUTY_H_G_OIS_EIS r y_ois_eis[8..15],
           ///z axis OIS/EIS
           0x32 UI_OUTZ_L_G_OIS_EIS r z_ois_eis[0..7], 
           0x33 UI_OUTZ_H_G_OIS_EIS r z_ois_eis[8..15],
           ///x axis OIS/DualC
           0x34 UI_OUTX_L_A_OIS_DualC r x_ois_dc[0..7], 
           0x35 UI_OUTX_H_A_OIS_DualC r x_ois_dc[8..15],
           ///y axis OIS/DualC
           0x36 UI_OUTY_L_A_OIS_DualC r y_ois_dc[0..7], 
           0x37 UI_OUTY_H_A_OIS_DualC r y_ois_dc[8..15],
           ///z axis OIS/DualC
           0x38 UI_OUTZ_L_A_OIS_DualC r z_ois_dc[0..7], 
           0x39 UI_OUTZ_H_A_OIS_DualC r z_ois_dc[8..15],
           ///Timestamp output. 1LSB = 21.75u
           0x40 TIMESTAMP0 r timestamp[0..7], 
           0x41 TIMESTAMP1 r timestamp[8..15],
           0x42 TIMESTAMP2 r timestamp[16..23],
           0x43 TIMESTAMP3 r timestamp[24..31],
           0x56 TAP_CFG0 rw
           {
                ///This bit allows immediately clearing the latched interrupts of an event detection
                ///upon the read of the corresponding status register. It must be set to 1 together
                ///with LIR. Default value: 0
                ///(0: latched interrupt signal cleared at the end of the ODR period;
                ///1: latched interrupt signal immediately cleared)
                6 int_clr_on_read, 
                ///Activity/inactivity interrupt mode configuration.
                ///If INT1_SLEEP_CHANGE or INT2_SLEEP_CHANGE bits are enabled, drives
                ///the sleep status or sleep change on the INT pins. Default value: 0
                ///(0: sleep change notification on INT pins; 1: sleep status reported on INT pins)
                5 sleep_status_on_int, 
                ///HPF or SLOPE filter selection on wake-up and Activity/Inactivity functions.
                ///Default value: 0 (0: SLOPE filter applied; 1: HPF applied)
                4 slope_fds,
                ///Enable X direction in tap recognition. Default value: 0
                3 tap_x_en, 
                ///Enable Y direction in tap recognition. Default value: 0
                2 tap_y_en, 
                ///Enable Z direction in tap recognition. Default value: 0
                1 tap_z_en, 
                ///Latched Interrupt. Default value: 0
                0 lir 
           },
           0x57 TAP_CFG1 rw
           {
                ///Selection of axis priority for TAP detection (see Table 119)
                ///TAP_PRIORITY_[2:0] | Max. priority | Mid. priority | Min. priority
                ///000 | X | Y | Z
                ///001 | Y | X | Z
                ///010 | X | Z | Y
                ///011 | Z | Y | X
                ///100 | X | Y | Z
                ///101 | Y | Z | X
                ///110 | Z | X | Y
                ///111 | Z | Y | X
                5..7 tap_priority, 
                ///X-axis tap recognition threshold. Default value: 0
                ///1 LSB = FS_XL / (2^5)
                0..4 tap_ths_x 
           },
           0x58 TAP_CFG2 rw
           {
                ///Enable basic interrupts (6D/4D, free-fall, wake-up, tap, inactivity). Default value: 0
                7 interrupts_enable, 
                ///Enable activity/inactivity (sleep) function. Default value: 00
                ///(00: stationary/motion-only interrupts generated, XL and gyro do not change;
                ///01: sets accelerometer ODR to 12.5 Hz (low-power mode), gyro does not change;
                ///10: sets accelerometer ODR to 12.5 Hz (low-power mode), gyro to sleep mode;
                ///11: sets accelerometer ODR to 12.5 Hz (low-power mode), gyro to power-down mode)
                5..6 inact_en,
                ///Y-axis tap recognition threshold. Default value: 0
                ///1 LSB = FS_XL / (2^5)
                0..4 tap_ths_y 
           },
           0x59 TAP_THS_6D rw
           {
                ///4D orientation detection enable. Z-axis position detection is disabled.
                7 d4d_en, 
                ///Threshold for 4D/6D function. Default value: 00
                ///SIXD_THS | Threshold value
                ///00 | 68 degrees
                ///01 | 47 degrees
                ///10 | Reserved
                ///11 | Reserved
                5..6 sixd_ths,
                ///Z-axis recognition threshold. Default value: 0
                0..4 tap_ths_z 
           },
           0x5A TAP_DUR rw
           {
                ///Duration of maximum time gap for double tap recognition. Default: 0000
                ///When double tap recognition is enabled, this register expresses the maximum time
                ///between two consecutive detected taps to determine a double tap event. The default
                ///value of these bits is 0000b which corresponds to 16*ODR_XL time. If the DUR[3:0]
                ///bits are set to a different value, 1LSB corresponds to 32*ODR_XL time.
                4..7 dur, 
                ///Expected quiet time after a tap detection. Default value: 00
                ///Quiet time is the time after the first detected tap in which there must not be any
                ///overthreshold event. The default value of these bits is 00b which corresponds to
                ///2*ODR_XL time. If the QUIET[1:0] bits are set to a different value, 1LSB corresponds
                ///to 4*ODR_XL time.
                2..3 quiet, 
                ///Maximum duration of overthreshold event. Default value: 00
                ///Maximum duration is the maximum time of an overthreshold signal detection to be
                ///recognized as a tap event. The default value of these bits is 00b which corresponds
                ///to 4*ODR_XL time. If the SHOCK[1:0] bits are set to a different value, 1LSB
                ///corresponds to 8*ODR_XL time.
                0..1 shock 
           },
           0x5B WAKE_UP_THS rw
           {                
                ///Single/double-tap event enable. Default: 0
                ///(0: only single-tap event enabled;
                ///1: both single and double-tap events enabled)
                7 single_double_tap, 

                ///Drives the low-pass filtered data with user offset correction (instead of high-pass filtered data) to the wakeup function.
                6 usr_off_on_wu, 
                ///Threshold for wakeup: 1 LSB weight depends on WAKE_THS_W in WAKE_UP_DUR (5Ch). Default value: 000000
                0..5 wk_ths 
           },
           0x5C WAKE_UP_DUR rw
           {
                ///Free fall duration event. Default: 0
                ///For the complete configuration of the free-fall duration, refer to FF_DUR[4:0] in
                ///FREE_FALL (5Dh) configuration.
                ///1 LSB = 1 ODR_time
                7 ff_dur5, 
                ///Wake up duration event. Default: 00
                ///1LSB = 1 ODR_time
                5..6 wake_dur, 
                ///Weight of 1 LSB of wakeup threshold. Default: 0
                ///(0: 1 LSB = FS_XL / (26);
                ///1: 1 LSB = FS_XL / (28) )
                4 wake_ths_w, 
                ///Duration to go in sleep mode. Default value: 0000 (this corresponds to 16 ODR)
                ///1 LSB = 512 ODR
                0..3 sleep_dur 
           },
           0x5D FREE_FALL rw
           {
                ///Free-fall duration event. Default: 0
                ///For the complete configuration of the free fall duration, refer to FF_DUR5 in
                ///WAKE_UP_DUR (5Ch) configuration
                3..7 ff_dur, 
                ///Free fall threshold setting. Default: 000
                ///FF_THS[2:0] | Threshold value
                ///000 | 312 mg
                ///001 | 438 mg
                ///010 | 500 mg
                ///011 | Reserved
                ///100 | Reserved
                ///101 | Reserved
                ///110 | Reserved
                ///111 | Reserved
                0..2 ff_ths 
           },
           0x5E MD1_CFG rw
               //Functions routing to INT1 pin register (R/W). Each bit in this
               //register enables a signal to be carried over the INT1 pin. The output
               //of the pin is the OR combination of the signals selected here and in
               //the INT1_CTRL (0Dh) register.
           {
                ///Routing of activity/inactivity recognition event on INT1. Default: 0
                7 int1_sleep_change, 
                ///Routing of single-tap recognition event on INT1. Default: 0
                6 int1_single_tap, 
                ///Routing of wakeup event on INT1. Default value: 0
                5 int1_wu, 
                ///Routing of free-fall event on INT1. Default value: 0
                4 int1_ff, 
                ///Routing of tap event on INT1. Default value: 0
                3 int1_double_tap, 
                ///Routing of 6D event on INT1. Default value: 0
                2 int1_6d, 
                ///Routing of embedded functions event on INT1. Default value: 0
                1 int1_emb_func, 
                ///Routing of sensor hub communication concluded event on INT1.
                0 int1_shub 
           },
           0x5F MD2_CFG rw
               //Functions routing to INT2 pin register (R/W). Each bit in this
               //register enables a signal to be carried over the INT2 pin. The output
               //of the pin is the OR combination of the signals selected here and in
               //the INT2_CTRL (0Eh) register.
           {
                ///Routing of activity/inactivity recognition event on INT2. Default: 0
                7 int2_sleep_change, 
                ///Single-tap recognition routing on INT2. Default: 0
                6 int2_single_tap, 
                ///Routing of wakeup event on INT2. Default value: 0
                5 int2_wu, 
                ///Routing of free-fall event on INT2. Default value: 0
                4 int2_ff, 
                ///Routing of tap event on INT2. Default value: 0
                3 int2_double_tap, 
                ///Routing of 6D event on INT2. Default value: 0
                2 int2_6d, 
                ///Routing of embedded functions event on INT2. Default value: 0
                1 int2_emb_func, 
                ///Enables routing on INT2 pin of the alert for timestamp overflow within 6.4 ms
                0 int2_timestamp 
           },
           0x62 I3C_BUS_AVB rw
           {
                ///These bits are used to select the bus available time when MIPI I3CSM IBI is used.
                ///Default value: 00
                ///(00: bus available time equal to 50 µsec (default);
                ///01: bus available time equal to 2 µsec;
                ///10: bus available time equal to 1 msec;
                ///11: bus available time equal to 25 msec
                3..4 i3c_bus_avb_sel,
                ///This bit allows disabling the INT1 pull-down.
                ///PD_DIS_INT1
                ///INT1
                ///(0: Pull-down on INT1 enabled (pull-down is effectively connected only when no interrupts are routed 
                ///to the INT1 pin or when the MIPI I3CSM dynamic address is assigned);
                ///1: Pull-down on INT1 disabled (pull-down not connected)
                0 pd_dis_int1
           },
           0x63 INTERNAL_FREQ_FINE rw
           {
                ///Difference in percentage of the effective ODR (and timestamp rate) with respect to the typical. Step: 0.15%. 
                ///8-bit format, two's complement.
                0..7 freq_fine
           },
           0x73 X_OFS_USR rw usr_offset_x,
           0x74 Y_OFS_USR rw usr_offset_y,
           0x75 Z_OFS_USR rw usr_offset_z,
           0x78 FIFO_DATA_OUT_TAG r
           {
                ///FIFO tag. Identifies sensor used for FIFO data.
                ///Value     Sensor
                ///0x01      Gyroscope NC
                ///0x02      Accelerometer NC
                ///0x03      Temperature
                ///0x04      Timestamp
                ///0x05      CFG_Change
                ///0x06      Accelerometer NC_T_2
                ///0x07      Accelerometer NC_T_1
                ///0x08      Accelerometer 2xC
                ///0x09      Accelerometer 3xC
                ///0x0A      Gyroscope NC_T_2
                ///0x0B      Gyroscope NC_T_1
                ///0x0C      Gyroscope 2xC
                ///0x0D      Gyroscope 3xC
                ///0x0E      Sensor hub slave 0
                ///0x0F      Sensor hub slave 1
                ///0x10      Sensor hub slave 2
                ///0x11      Sensor hub slave 3
                ///0x12      Step counter
                ///0x19      Sensor hub nack
                3..7 tag_sensor, 
                ///2-bit counter which identifies sensor time slot
                1..2 tag_counter, 
                ///Parity check of TAG content
                0 tag_parity 
           },
           ///x axis ouptut
           0x79 FIFO_DATA_OUT_X_L r, 
           ///x axis ouptut
           0x7A FIFO_DATA_OUT_X_H r, 
           ///y axis output
           0x7B FIFO_DATA_OUT_Y_L r, 
           ///y axis output
           0x7C FIFO_DATA_OUT_Y_H r, 
           ///z axis output
           0x7D FIFO_DATA_OUT_Z_L r, 
           ///z axis output
           0x7E FIFO_DATA_OUT_Z_H r 
        }
    }
}

pub struct Lsm6dso<S: SpiHandle> {
    spi: S
}

impl <S: SpiHandle> Lsm6dso<S> {
    pub fn new(spi: S) -> Self {
        Self {
            spi
        }
    }
    pub async fn setup(
        &mut self
    ) -> Result<(),<S::Bus as ErrorType>::Error> {
          //TODO: Use this function to perform initial setup of the IMU. Example: opening register access,
          // let accel_mode = self.read_reg(RegCtrl1Xl).await?;
          self.write_reg(RegCtrl1Xl, 0b1010_11_00 as u8).await?;
          // let gyro_mode = self.read_reg(RegCtrl2G).await?;
          self.write_reg(RegCtrl2G, 0b1010_11_00).await?;
          Ok(())    
    }

     pub async fn raw_accel(&mut self) -> Result<(i16, i16, i16), <S::Bus as ErrorType>::Error> {
          Ok(unsafe {
               let accel_x: i16 = mem::transmute(self.accel_x().await?);
               let accel_y: i16 = mem::transmute(self.accel_y().await?);
               let accel_z: i16 = mem::transmute(self.accel_z().await?);

               (accel_x, accel_y, accel_z)
          })
     }

     pub async fn raw_gyro(&mut self) -> Result<(i16, i16, i16), <S::Bus as ErrorType>::Error> {
          Ok(unsafe {
               let gyro_pitch: i16 = mem::transmute(self.gyro_pitch_rate().await?);
               let gyro_roll: i16 = mem::transmute(self.gyro_roll_rate().await?);
               let gyro_yaw: i16 = mem::transmute(self.gyro_yaw_rate().await?);

               (gyro_pitch, gyro_roll, gyro_yaw)
          })
     }

     pub async fn accel_sensitivity(&mut self) -> Result<i32, <S::Bus as ErrorType>::Error> {
          Ok(
          match self.accel_fs().await? {
               0 => 4,
               1 => 32,
               2 => 8,
               3 => 16,
               _ => unreachable!()
          })
     }
     /// 0 = 4g, 1 = 8g, 2 = 16g, 3 = 32g
     pub async fn set_accel_sensitivity(&mut self, new_fs: u8) -> Result<u8, <S::Bus as ErrorType>::Error> {
          let accel_mode = self.read_reg(RegCtrl1Xl).await?;
          let mask = 0b1111_00_11;

          let new_bits = match new_fs {
               0 => 0b0000_00_00,
               1 => 0b0000_10_00,
               2 => 0b0000_11_00,
               3 => 0b0000_01_00,
               _ => 0b0000_00_00
          };

          self.write_reg(RegCtrl1Xl, accel_mode & mask | new_bits as u8).await?;
          Ok(new_bits >> 2)
     }
     pub async fn gyro_sensitivity(&mut self) -> Result<i32, <S::Bus as ErrorType>::Error> {
          Ok(
          match self.gyro_fs_select().await? {
               0 => 250,
               1 => 500,
               2 => 1000,
               3 => 2000,
               _ => unreachable!()
          })
     }
     
     pub async fn test_fs(&mut self) -> Result<u8, <S::Bus as ErrorType>::Error> {
          Ok(self.accel_fs().await?)
     }

     /// returns a tuple with units of ug (10^-6)
     pub async fn accel(&mut self) -> Result<(i32, i32, i32), <S::Bus as ErrorType>::Error> {
         let (raw_x, raw_y, raw_z) = self.raw_accel().await?;
          //sensitivity mode TODO: read from chip
          let fs = self.accel_sensitivity().await?;
          let scalar: i32 = 122 * fs;//* fs/4;
          //xyz are corrected so that
          //x -> cable direction
          //yz follow from right hand rule, x as index finger
          let accel_x: i32 = -scalar * (raw_x as i32);
          let accel_y: i32 = scalar * (raw_y as i32);
          let accel_z: i32 = -scalar * (raw_z as i32);

          Ok((accel_x, accel_y, accel_z))
     }

     pub async fn gyro(&mut self) -> Result<(i32, i32, i32), <S::Bus as ErrorType>::Error> {
          let (raw_pitch, raw_roll, raw_yaw) = self.raw_gyro().await?;
          //sensitivity mode TODO: read from chip
          let fs = self.gyro_sensitivity().await?;
          let scalar: i32 = 4375 * fs/125;
          let gyro_pitch: i32 = scalar * (raw_pitch as i32);
          let gyro_roll: i32 = scalar * (raw_roll as i32);
          let gyro_yaw: i32 = scalar * (raw_yaw as i32);

          Ok((gyro_pitch, gyro_roll, gyro_yaw))
     }
    
}


impl <S: SpiHandle> ReadLsm6dso for Lsm6dso<S> {
    type Error = <S::Bus as ErrorType>::Error;

    async fn read_contiguous_regs(
        &mut self,
        addr: impl ReadableAddr,
        out: &mut [u8]
    ) -> Result<(), Self::Error> {
        let mut bus = self.spi.select().await;

        // set rw bit
        
        // write = 1, read = 0
        
        let addr: u8 = addr.as_addr() | 0b1000_0000;
        
        bus.write(&[addr]).await?;
        bus.transfer_in_place(out).await?;
        Ok(())
    }
}

impl <S: SpiHandle> WriteLsm6dso for Lsm6dso<S> {
    type Error = <S::Bus as ErrorType>::Error;

    async fn write_contiguous_regs(
        &mut self,
        addr: impl WritableAddr,
        values: &[u8]
    ) -> Result<(), Self::Error> {
        let mut bus = self.spi.select().await;

        let addr: u8 = addr.as_addr() & 0b0111_1111;

        bus.write(&[addr.as_addr()]).await?;
        bus.write(values).await?;

        Ok(())
    }

}
