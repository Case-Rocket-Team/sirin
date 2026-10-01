#![no_std]
#![allow(unused_imports)]
#![allow(unsafe_code)]

use core::{fmt::Debug, i16::MAX, mem, ops::Div};

use dev_csr::dev_csr;
use embedded_hal::spi::{ ErrorKind as SpiError, ErrorType};
use embedded_hal_async::spi::SpiBus;
use serde::{Deserialize, Serialize};
use spi_handle::SpiHandle;
use uunit::{MicroGs, Quantity, UnitMicrodegrees, UnitSeconds, WithUnits};

dev_csr! {
    dev Lsm6dsv32x{
        regs{
           ///enables access to embedded-functions / sensor-hub config registers, and misc resets. Default 0
           0x01 FUNC_CFG_ACCESS rw
           {
                ///enables access to the embedded functions configuration registers, default 0
                7 emb_func_reg_access,
                ///enables access to the sensor hub (I2C master) configuration registers, default 0
                6 shub_reg_access,
                ///enables FSM to autonomously write some CTRL registers. 0: disabled, 1: enabled. default 0
                3 fsm_wr_ctrl_en,
                ///global software reset of the device (POR-like), default 0
                2 sw_por,
                ///resets the SPI2 control registers from the primary interface. Write 1 then back to 0 (not auto-cleared). default 0
                1 spi2_reset,
                ///enables full control of OIS configuration from the primary interface, default 0
                0 ois_ctrl_from_ui,
           },
           ///SDO, OCS_Aux, SDO_Aux pins pull-up register. Not reset by CTRL3 sw_reset
           0x02 PIN_CTRL rw
           {
                ///disables pull-up on both OCS_Aux and SDO_Aux pins (mode 1/2). 0: pull-up enabled, 1: disconnected. default 0
                7 ois_pu_dis,
                ///enables pull-up on SDO pin. 0: disconnected, 1: enabled. default 0
                6 sdo_pu_en,
                ///action performed after "reset whole chip" I3C pattern. 0: config reset (sw reset + dynamic addr reset), 1: global reset (POR). default 1
                5 ibhr_por_en,
           },
           ///Interface configuration register. Not reset by CTRL3 sw_reset
           0x03 IF_CFG rw
           {
                ///enables pull-up on SDA pin, default 0
                7 sda_pu_en,
                ///enables master I2C (sensor hub) pull-up, default 0
                6 shub_pu_en,
                ///enables antispike filters on SCL/SDA always-on, default 0
                5 asf_ctrl,
                ///interrupt activation level. 0: active high, 1: active low. default 0
                4 h_lactive,
                ///push-pull/open-drain selection on INT1/INT2 pins. 0: push-pull, 1: open-drain. default 0
                3 pp_od,
                ///SPI serial interface mode. 0: 4-wire, 1: 3-wire. default 0
                2 sim,
                ///disables I2C and MIPI I3C interfaces. 0: enabled, 1: disabled. default 0
                0 i2c_i3c_disable,
           },
           ///ODR-triggered mode configuration register
           0x06 ODR_TRIG_CFG rw odr_trig_nodr,
           ///FIFO watermark threshold, 1 LSB = TAG (1 byte) + 1 sensor (6 bytes) in FIFO, flag rises when #bytes in FIFO >= threshold
           0x07 FIFO_CTRL1 rw wtm,
           0x08 FIFO_CTRL2 rw
           {
                ///limits FIFO depth to threshold level set in FIFO_CTRL1. default 0
                7 stop_on_wtm,
                ///enables/disables the FIFO compression algorithm at runtime (active only if FIFO_COMPR_EN in EMB_FUNC_EN_B is 1). default 0
                6 fifo_compr_rt_en,
                ///enables the ODR-CHANGE virtual sensor to be batched in FIFO. default 0
                4 odr_chg_en,
                ///forces uncompressed data writing at this rate.
                ///0: not forced (default)
                ///1: uncompressed data every 8 batches
                ///2: uncompressed data every 16 batches
                ///3: uncompressed data every 32 batches
                1..2 uncompr_rate,
                ///when dual-channel mode is enabled, enables FSM-triggered batching in FIFO of accelerometer channel 2. default 0
                0 xl_dualc_batch_from_fsm,
           },
           0x09 FIFO_CTRL3 rw
           {
                ///batch data rate for gyroscope.
                ///0000: gyroscope not batched (default)
                ///0001: 1.875 Hz
                ///0010: 7.5 Hz
                ///0011: 15 Hz
                ///0100: 30 Hz
                ///0101: 60 Hz
                ///0110: 120 Hz
                ///0111: 240 Hz
                ///1000: 480 Hz
                ///1001: 960 Hz
                ///1010: 1.92 kHz
                ///1011: 3.84 kHz
                ///1100: 7.68 kHz
                ///1101-1111: reserved
                4..7 bdr_gy,
                ///batch data rate for accelerometer.
                ///0000: accelerometer not batched (default)
                ///0001: 1.875 Hz
                ///0010: 7.5 Hz
                ///0011: 15 Hz
                ///0100: 30 Hz
                ///0101: 60 Hz
                ///0110: 120 Hz
                ///0111: 240 Hz
                ///1000: 480 Hz
                ///1001: 960 Hz
                ///1010: 1.92 kHz
                ///1011: 3.84 kHz
                ///1100: 7.68 kHz
                ///1101-1111: reserved
                0..3 bdr_xl,
           },
           0x0A FIFO_CTRL4 rw
           {
                ///decimation for timestamp batching. write rate = max(BDR_XL,BDR_GY).
                ///00: timestamp not batched
                ///01: decimation 1 (max rate)
                ///10: decimation 8 (max rate / 8)
                ///11: decimation 32 (max rate / 32)
                6..7 dec_ts_batch,
                ///batch data rate for temperature.
                ///00: temperature not batched
                ///01: 1.875 Hz
                ///10: 15 Hz
                ///11: 60 Hz
                4..5 odr_t_batch,
                ///enables FIFO batching of enhanced EIS gyroscope output values, default 0
                3 g_eis_fifo_en,
                ///FIFO mode selection.
                ///000: bypass mode (FIFO disabled, default)
                ///001: FIFO mode: stops collecting when FIFO is full
                ///010: continuousWTM-to-full mode: continuous with wtm size until trigger deasserted, then stores until full
                ///011: continuous-to-FIFO mode: continuous until trigger deasserted, then FIFO mode
                ///100: bypass-to-continuous mode: bypass until trigger deasserted, then continuous mode
                ///101: reserved
                ///110: continuous mode: if full, new sample overwrites the oldest
                ///111: bypass-to-FIFO mode: bypass until trigger deasserted, then FIFO mode
                0..2 fifo_mode,
           },
           0x0B COUNTER_BDR_REG1 rw
           {
                ///selects the trigger for the internal counter of batch events.
                ///00: accelerometer batch event
                ///01: gyroscope batch event
                ///10-11: gyroscope EIS batch event
                5..6 trig_counter_bdr,
                ///bits [9:8] of the batch-event counter threshold (see COUNTER_BDR_REG2 for bits [7:0]). When counter reaches threshold, it resets and COUNTER_BDR_IA is set
                0..1 batch_counter_thresh[8..9],
           },
           ///bits [7:0] of the batch-event counter threshold
           0x0C COUNTER_BDR_REG2 rw batch_counter_thresh[0..7],
           ///INT1 pin control register. Output is the OR of the bits here and in MD1_CFG. All bits default 0
           0x0D INT1_CTRL rw
           {
                ///enables COUNTER_BDR_IA interrupt on INT1 pin
                6 int1_cnt_bdr,
                ///enables FIFO full flag interrupt on INT1 pin (can also trigger an IBI over MIPI I3C)
                5 int1_fifo_full,
                ///enables FIFO overrun interrupt on INT1 pin (can also trigger an IBI over MIPI I3C)
                4 int1_fifo_ovr,
                ///enables FIFO threshold interrupt on INT1 pin (can also trigger an IBI over MIPI I3C)
                3 int1_fifo_th,
                ///enables gyroscope data-ready interrupt on INT1 pin (can also trigger an IBI over MIPI I3C)
                1 int1_drdy_g,
                ///enables accelerometer data-ready interrupt on INT1 pin (can also trigger an IBI over MIPI I3C)
                0 int1_drdy_xl,
           },
           ///INT2 pin control register. Output is the OR of the bits here and in MD2_CFG. All bits default 0
           0x0E INT2_CTRL rw
           {
                ///routes the embedded-functions end-of-operation signal to INT2
                7 int2_emb_func_endop,
                ///enables COUNTER_BDR_IA interrupt on INT2 pin
                6 int2_cnt_bdr,
                ///enables FIFO full flag interrupt on INT2 pin
                5 int2_fifo_full,
                ///enables FIFO overrun interrupt on INT2 pin
                4 int2_fifo_ovr,
                ///enables FIFO threshold interrupt on INT2 pin
                3 int2_fifo_th,
                ///enables gyroscope EIS data-ready interrupt on INT2 pin
                2 int2_drdy_g_eis,
                ///enables gyroscope data-ready interrupt on INT2 pin
                1 int2_drdy_g,
                ///enables accelerometer data-ready interrupt on INT2 pin
                0 int2_drdy_xl,
           },
           ///whoami value. Read-only, fixed at 0x70
           0x0F WHO_AM_I r whoami,
           0x10 CTRL1 rw
           {
                ///accelerometer operating mode.
                ///000: high-performance mode (default)
                ///001: high-accuracy ODR mode
                ///010: reserved
                ///011: ODR-triggered mode
                ///100: low-power mode 1 (2-sample mean)
                ///101: low-power mode 2 (4-sample mean)
                ///110: low-power mode 3 (8-sample mean)
                ///111: normal mode
                4..6 op_mode_xl,
                ///accelerometer ODR selection.
                ///0000: power-down
                ///0001: 1.875 Hz (low-power mode)
                ///0010: 7.5 Hz
                ///0011: 15 Hz
                ///0100: 30 Hz
                ///0101: 60 Hz
                ///0110: 120 Hz
                ///0111: 240 Hz
                ///1000: 480 Hz
                ///1001: 960 Hz
                ///1010: 1.92 kHz
                ///1011: 3.84 kHz (high-performance mode only)
                ///1100: 7.68 kHz (high-performance mode only)
                ///others: reserved
                0..3 odr_xl,
           },
           0x11 CTRL2 rw
           {
                ///gyroscope operating mode.
                ///000: high-performance mode (default)
                ///001: high-accuracy ODR mode
                ///010: reserved
                ///011: ODR-triggered mode
                ///100: sleep mode
                ///101: low-power mode
                ///110-111: reserved
                4..6 op_mode_g,
                ///gyroscope ODR selection.
                ///0000: power-down
                ///0010: 7.5 Hz
                ///0011: 15 Hz
                ///0100: 30 Hz
                ///0101: 60 Hz
                ///0110: 120 Hz
                ///0111: 240 Hz
                ///1000: 480 Hz
                ///1001: 960 Hz
                ///1010: 1.92 kHz
                ///1011: 3.84 kHz
                ///1100: 7.68 kHz
                ///others: reserved
                0..3 odr_g,
           },
           0x12 CTRL3 rw
           {
                ///reboots memory content, auto-cleared, default 0
                7 boot,
                ///block data update. 0: continuous update, 1: output regs not updated until LSB and MSB read. default 1
                6 bdu,
                ///register address auto-increment on multi-byte access. default 1
                2 if_inc,
                ///software reset, resets all control registers to default. auto-cleared. default 0
                0 sw_reset,
           },
           0x13 CTRL4 rw
           {
                ///routes embedded-functions interrupt signals to INT1 (OR'd with INT1's own enables; does not affect INT2). default 0
                4 int2_on_int1,
                ///masks data-ready signals (accel & gyro independently) until filter settling ends. default 0
                3 drdy_mask,
                ///enables temperature sensor data-ready interrupt on INT2 pin. default 0
                2 int2_drdy_temp,
                ///enables pulsed data-ready mode. 0: latched (cleared after high part of output read), 1: pulsed (~65us). default 0
                1 drdy_pulsed,
                ///polarity of the INT2 pin input trigger for DEN/embedded functions. 0: active low, 1: active high. default 0
                0 int2_in_lh,
           },
           0x14 CTRL5 rw
           {
                ///bus-available time selection for IBI.
                ///00: 2 us
                ///01: 50 us (default)
                ///10: 1 ms
                ///11: 25 ms
                1..2 bus_act_sel,
                ///enables INT pin usage when I3C is enabled, default 0
                0 int_en_i3c,
           },
           0x15 CTRL6 rw
           {
                ///gyroscope LPF1 bandwidth selection (see datasheet Table 64 for cutoff vs ODR); available when OIS and/or EIS are disabled
                4..6 lpf1_g_bw,
                ///gyroscope UI chain full-scale.
                ///0000: +-125 dps (default)
                ///0001: +-250 dps
                ///0010: +-500 dps
                ///0011: +-1000 dps
                ///0100: +-2000 dps
                ///1100: +-4000 dps (requires OIS gyro chain disabled)
                ///others: reserved
                0..3 fs_g,
           },
           0x16 CTRL7 rw
           {
                ///enables the analog hub / Qvar chain (connects AH/Qvar buffers to SDx/SCx pins). Accel+gyro must be power-down first. default 0
                7 ah_qvar_en,
                ///enables analog hub / Qvar data-ready interrupt on INT2 pin, default 0
                6 int2_drdy_ah_qvar,
                ///analog hub / Qvar buffer input impedance.
                ///00: 2.4 GOhm (default)
                ///01: 730 MOhm
                ///10: 300 MOhm
                ///11: 235 MOhm
                4..5 ah_qvar_c_zin,
                ///enables the gyroscope digital LPF1 filter (bandwidth via lpf1_g_bw in CTRL6 if OIS chain disabled)
                0 lpf1_g_en,
           },
           0x17 CTRL8 rw
           {
                ///accelerometer LPF2/HP filter cutoff setting (see datasheet Table 69 for full mapping vs ODR)
                5..7 hp_lpf2_xl_bw,
                ///enables dual-channel mode: max-full-scale data sent to output regs 0x34-0x39 via the UI chain. default 0
                3 xl_dualc_en,
                ///accelerometer full-scale selection.
                ///00: +-4 g
                ///01: +-8 g
                ///10: +-16 g
                ///11: +-32 g
                0..1 fs_xl,
           },
           0x18 CTRL9 rw
           {
                ///enables accelerometer high-pass filter reference mode (requires hp_slope_xl_en=1); first output must be discarded when enabled. default 0
                6 hp_ref_mode_xl,
                ///enables accelerometer LPF2/HPF fast-settling mode (first sample set after write; active only on exit from power-down). default 0
                5 xl_fastsettl_mode,
                ///accelerometer slope/high-pass filter path selection. 0: low-pass path, 1: high-pass path. default 0
                4 hp_slope_xl_en,
                ///accelerometer high-resolution selection. 0: first-stage digital filtering, 1: LPF2 second stage. default 0
                3 lpf2_xl_en,
                ///weight of accelerometer user-offset bits in X/Y/Z_OFS_USR. 0: 2^-10 g/LSB, 1: 2^-6 g/LSB. default 0
                1 usr_off_w,
                ///enables accelerometer user-offset correction block (valid on low-pass path). default 0
                0 usr_off_on_out,
           },
           0x19 CTRL10 rw
           {
                ///enables debug mode for the embedded functions
                6 emb_func_debug,
                ///gyroscope self-test selection.
                ///00: normal mode (default)
                ///01: positive sign self-test
                ///10: negative sign self-test
                ///11: reserved
                2..3 st_g,
                ///accelerometer self-test selection.
                ///00: normal mode (default)
                ///01: positive sign self-test
                ///10: negative sign self-test
                ///11: reserved
                0..1 st_xl,
           },
           ///acknowledge flag for FSM control of the config registers
           0x1A CTRL_STATUS r
           {
                ///0: all registers writable from standard interface, 1: some registers are under FSM control (read-only). Ack for FSM_WR_CTRL_EN toggling
                2 fsm_wr_ctrl_status,
           },
           ///number of unread sensor samples (TAG + 6 bytes) stored in FIFO, bits [7:0] (see FIFO_STATUS2 for bit 8)
           0x1B FIFO_STATUS1 r diff_fifo[0..7],
           0x1C FIFO_STATUS2 r
           {
                ///FIFO watermark status. 0: filling < WTM, 1: filling >= WTM. default 0
                7 fifo_wtm_ia,
                ///FIFO overrun status. 0: not completely filled, 1: completely filled. default 0
                6 fifo_ovr_ia,
                ///smart FIFO full status. 0: not full, 1: will be full at next ODR. default 0
                5 fifo_full_ia,
                ///batch-event counter reached its threshold; reset when COUNTER_BDR_REG1/2 are read. default 0
                4 counter_bdr_ia,
                ///latched FIFO overrun status; reset when this register is read. default 0
                3 fifo_ovr_latched,
                ///number of unread sensor samples stored in FIFO, bit 8 (see FIFO_STATUS1 for bits [7:0])
                0 diff_fifo[8..8],
           },
           0x1D ALL_INT_SRC r
           {
                ///embedded-functions interrupt status. default 0
                7 emb_func_ia,
                ///sensor hub (I2C master) interrupt status. default 0
                6 shub_ia,
                ///detects a change event in activity/inactivity status. default 0
                5 sleep_change_ia,
                ///interrupt active for a change in portrait/landscape/face-up/face-down position. default 0
                4 d6d_ia,
                ///single or double-tap event detection status (see SINGLE_DOUBLE_TAP in WAKE_UP_THS). default 0
                2 tap_ia,
                ///wake-up event status. default 0
                1 wu_ia,
                ///free-fall event status. default 0
                0 ff_ia,
           },
           0x1E STATUS_REG r
           {
                ///alerts timestamp overflow within 5.6ms
                7 timestamp_endcount,
                ///accelerometer OIS or gyroscope OIS new output data available. default 0
                5 ois_drdy,
                ///enhanced EIS gyroscope new data available. default 0
                4 gda_eis,
                ///analog hub or Qvar new data available. default 0
                3 ah_qvarda,
                ///temperature new data available. default 0
                2 tda,
                ///gyroscope new data available. default 0
                1 gda,
                ///accelerometer new data available. default 0
                0 xlda,
           },
           ///temperature output data (two's complement)
           0x20 OUT_TEMP_L r temp_data[0..7],
           0x21 OUT_TEMP_H r temp_data[8..15],
           ///gyro pitch axis (X) angular rate, UI chain, two's complement
           0x22 OUTX_L_G r gyro_pitch_rate[0..7],
           0x23 OUTX_H_G r gyro_pitch_rate[8..15],
           ///gyro roll axis (Y) angular rate, UI chain, two's complement
           0x24 OUTY_L_G r gyro_roll_rate[0..7],
           0x25 OUTY_H_G r gyro_roll_rate[8..15],
           ///gyro yaw axis (Z) angular rate, UI chain, two's complement
           0x26 OUTZ_L_G r gyro_yaw_rate[0..7],
           0x27 OUTZ_H_G r gyro_yaw_rate[8..15],
           ///accel X-axis output, UI chain, two's complement
           0x28 OUTX_L_A r accel_x[0..7],
           0x29 OUTX_H_A r accel_x[8..15],
           ///accel Y-axis output, UI chain, two's complement
           0x2A OUTY_L_A r accel_y[0..7],
           0x2B OUTY_H_A r accel_y[8..15],
           ///accel Z-axis output, UI chain, two's complement
           0x2C OUTZ_L_A r accel_z[0..7],
           0x2D OUTZ_H_A r accel_z[8..15],
           ///gyro X-axis (pitch), OIS or EIS channel, two's complement
           0x2E UI_OUTX_L_G_OIS_EIS r x_ois_eis[0..7],
           0x2F UI_OUTX_H_G_OIS_EIS r x_ois_eis[8..15],
           ///gyro Y-axis (roll), OIS or EIS channel, two's complement
           0x30 UI_OUTY_L_G_OIS_EIS r y_ois_eis[0..7],
           0x31 UI_OUTY_H_G_OIS_EIS r y_ois_eis[8..15],
           ///gyro Z-axis (yaw), OIS or EIS channel, two's complement
           0x32 UI_OUTZ_L_G_OIS_EIS r z_ois_eis[0..7],
           0x33 UI_OUTZ_H_G_OIS_EIS r z_ois_eis[8..15],
           ///accel X-axis, OIS channel or dual-channel mode, two's complement
           0x34 UI_OUTX_L_A_OIS_DualC r x_ois_dc[0..7],
           0x35 UI_OUTX_H_A_OIS_DualC r x_ois_dc[8..15],
           ///accel Y-axis, OIS channel or dual-channel mode, two's complement
           0x36 UI_OUTY_L_A_OIS_DualC r y_ois_dc[0..7],
           0x37 UI_OUTY_H_A_OIS_DualC r y_ois_dc[8..15],
           ///accel Z-axis, OIS channel or dual-channel mode, two's complement
           0x38 UI_OUTZ_L_A_OIS_DualC r z_ois_dc[0..7],
           0x39 UI_OUTZ_H_A_OIS_DualC r z_ois_dc[8..15],
           ///analog hub / Qvar output data, valid when ah_qvar_en=1 in CTRL7, two's complement
           0x3A AH_QVAR_OUT_L r ah_qvar_out[0..7],
           0x3B AH_QVAR_OUT_H r ah_qvar_out[8..15],
           ///timestamp output, 1 LSB = 21.75us (typical)
           0x40 TIMESTAMP0 r timestamp[0..7],
           0x41 TIMESTAMP1 r timestamp[8..15],
           0x42 TIMESTAMP2 r timestamp[16..23],
           0x43 TIMESTAMP3 r timestamp[24..31],
           0x44 UI_STATUS_REG_OIS r
           {
                ///high while the gyroscope output is in the settling phase
                2 gyro_settling,
                ///gyroscope OIS data available (reset on read of a data output high byte). default 0
                1 gda_ois,
                ///accelerometer OIS data available (reset on read of a data output high byte). default 0
                0 xlda_ois,
           },
           0x45 WAKE_UP_SRC r
           {
                ///detects a change event in activity/inactivity status. default 0
                6 sleep_change_ia,
                ///free-fall event detection status. default 0
                5 ff_ia,
                ///sleep status bit. 0: activity, 1: inactivity. default 0
                4 sleep_state,
                ///wake-up event detection status. default 0
                3 wu_ia,
                ///wake-up event detection status on X-axis. default 0
                2 x_wu,
                ///wake-up event detection status on Y-axis. default 0
                1 y_wu,
                ///wake-up event detection status on Z-axis. default 0
                0 z_wu,
           },
           0x46 TAP_SRC r
           {
                ///tap event detection status. default 0
                6 tap_ia,
                ///single-tap event status. default 0
                5 single_tap,
                ///double-tap event detection status. default 0
                4 double_tap,
                ///sign of acceleration detected by tap event. 0: positive, 1: negative. default 0
                3 tap_sign,
                ///tap event detection status on X-axis. default 0
                2 x_tap,
                ///tap event detection status on Y-axis. default 0
                1 y_tap,
                ///tap event detection status on Z-axis. default 0
                0 z_tap,
           },
           0x47 D6D_SRC r
           {
                ///interrupt active for a change in portrait/landscape/face-up/face-down position. default 0
                6 d6d_ia,
                ///Z-axis high event (over threshold). default 0
                5 zh,
                ///Z-axis low event (under threshold). default 0
                4 zl,
                ///Y-axis high event (over threshold). default 0
                3 yh,
                ///Y-axis low event (under threshold). default 0
                2 yl,
                ///X-axis high event (over threshold). default 0
                1 xh,
                ///X-axis low event (under threshold). default 0
                0 xl,
           },
           0x48 STATUS_MASTER_MAINPAGE r
           {
                ///set when the write-once operation on slave 0 is complete (see WRITE_ONCE in MASTER_CONFIG). default 0
                7 wr_once_done,
                ///set on NACK from sensor-hub slave 3 communication. default 0
                6 slave3_nack,
                ///set on NACK from sensor-hub slave 2 communication. default 0
                5 slave2_nack,
                ///set on NACK from sensor-hub slave 1 communication. default 0
                4 slave1_nack,
                ///set on NACK from sensor-hub slave 0 communication. default 0
                3 slave0_nack,
                ///sensor-hub communication status. 0: not concluded, 1: concluded. default 0
                0 sens_hub_endop,
           },
           0x49 EMB_FUNC_STATUS_MAINPAGE r
           {
                ///interrupt status bit for FSM long-counter timeout event
                7 is_fsm_lc,
                ///interrupt status bit for significant-motion detection
                5 is_sigmot,
                ///interrupt status bit for tilt detection
                4 is_tilt,
                ///interrupt status bit for step detection
                3 is_step_det,
           },
           0x4A FSM_STATUS_MAINPAGE r
           {
                ///interrupt status bit for FSM8 event
                7 is_fsm8,
                ///interrupt status bit for FSM7 event
                6 is_fsm7,
                ///interrupt status bit for FSM6 event
                5 is_fsm6,
                ///interrupt status bit for FSM5 event
                4 is_fsm5,
                ///interrupt status bit for FSM4 event
                3 is_fsm4,
                ///interrupt status bit for FSM3 event
                2 is_fsm3,
                ///interrupt status bit for FSM2 event
                1 is_fsm2,
                ///interrupt status bit for FSM1 event
                0 is_fsm1,
           },
           0x4B MLC_STATUS_MAINPAGE r
           {
                ///interrupt status bit for MLC4 event
                3 is_mlc4,
                ///interrupt status bit for MLC3 event
                2 is_mlc3,
                ///interrupt status bit for MLC2 event
                1 is_mlc2,
                ///interrupt status bit for MLC1 event
                0 is_mlc1,
           },
           ///difference in effective ODR (and timestamp rate) vs typical. Step 0.13%, 8-bit two's complement. Read-only
           0x4F INTERNAL_FREQ_FINE r freq_fine,
           0x50 FUNCTIONS_ENABLE rw
           {
                ///enables basic interrupts (6D/4D, free-fall, wake-up, tap, activity/inactivity). default 0
                7 interrupts_enable,
                ///enables the timestamp counter (readable in TIMESTAMP0-3). default 0
                6 timestamp_en,
                ///when set, reading ALL_INT_SRC does not reset the latched interrupt signals. default 0
                3 dis_rst_lir_all_int,
                ///enables activity/inactivity (sleep) function.
                ///00: stationary/motion-only interrupts, no config change (default)
                ///01: accel -> low-power mode 1 at ODR from XL_INACT_ODR in INACTIVITY_DUR, gyro unchanged
                ///10: same accel change, gyro -> sleep mode
                ///11: same accel change, gyro -> power-down mode
                0..1 inact_en,
           },
           0x51 DEN rw
           {
                ///enables DEN data level-sensitive trigger (see trigger-mode table with lvl2_en)
                6 lvl1_en,
                ///enables DEN level-sensitive latched mode (see trigger-mode table with lvl1_en)
                5 lvl2_en,
                ///extends DEN functionality to the accelerometer sensor. default 0
                4 den_xl_en,
                ///DEN value stored in LSB of X-axis. default 1
                3 den_x,
                ///DEN value stored in LSB of Y-axis. default 1
                2 den_y,
                ///DEN value stored in LSB of Z-axis. default 1
                1 den_z,
                ///DEN stamping sensor selection. 0: stamped on gyro axis selected by den_x/y/z, 1: stamped on accel axis. default 0
                0 den_xl_g,
           },
           0x54 INACTIVITY_DUR rw
           {
                ///drives sleep status or sleep change on the INT pin when enabled via INT1/2_SLEEP_CHANGE. 0: change notification, 1: status. default 0
                7 sleep_status_on_int,
                ///weight of 1 LSB of wake-up/inactivity threshold.
                ///000: 7.8125 mg/LSB (default)
                ///001: 15.625 mg/LSB
                ///010: 31.25 mg/LSB
                ///011: 62.5 mg/LSB
                ///100: 125 mg/LSB
                ///101-111: 250 mg/LSB
                4..6 wu_inact_ths_w,
                ///accelerometer ODR target during inactivity.
                ///00: 1.875 Hz
                ///01: 15 Hz (default)
                ///10: 30 Hz
                ///11: 60 Hz
                2..3 xl_inact_odr,
                ///duration of the stationary-to-motion transition.
                ///00: immediate, at first overthreshold event (default)
                ///01: after two consecutive overthreshold events
                ///10: after three consecutive overthreshold events
                ///11: after four consecutive overthreshold events
                0..1 inact_dur,
           },
           ///activity/inactivity threshold, resolution set by wu_inact_ths_w in INACTIVITY_DUR. default 0
           0x55 INACTIVITY_THS rw inact_ths[0..5],
           0x56 TAP_CFG0 rw
           {
                ///LPF2 filter on 6D function selection. 0: ODR/2 low-pass data to 6D, 1: LPF2 output to 6D. default 0
                6 low_pass_on_6d,
                ///masks execution trigger of basic interrupt functions while accelerometer data are settling. default 0
                5 hw_func_mask_xl_settl,
                ///HPF or slope filter selection for wake-up / activity-inactivity. 0: slope filter, 1: HPF. default 0
                4 slope_fds,
                ///enables X direction in tap recognition. default 0
                3 tap_x_en,
                ///enables Y direction in tap recognition. default 0
                2 tap_y_en,
                ///enables Z direction in tap recognition. default 0
                1 tap_z_en,
                ///latched interrupt. default 0
                0 lir,
           },
           0x57 TAP_CFG1 rw
           {
                ///axis priority selection for tap detection.
                ///000: X max, Y mid, Z min
                ///001: Y max, X mid, Z min
                ///010: X max, Z mid, Y min
                ///011: Z max, Y mid, X min
                ///100: X max, Y mid, Z min
                ///101: Y max, Z mid, X min
                ///110: Z max, X mid, Y min
                ///111: Z max, Y mid, X min
                5..7 tap_priority,
                ///X-axis tap recognition threshold, 1 LSB = FS_XL/2^5. default 0
                0..4 tap_ths_x,
           },
           ///Y-axis tap recognition threshold, 1 LSB = FS_XL/2^5. default 0
           0x58 TAP_CFG2 rw tap_ths_y[0..4],
           0x59 TAP_THS_6D rw
           {
                ///enables 4D orientation detection (Z-axis position detection disabled). default 0
                7 d4d_en,
                ///threshold for 4D/6D function.
                ///00: 80 degrees (default)
                ///01: 70 degrees
                ///10: 60 degrees
                ///11: 50 degrees
                5..6 sixd_ths,
                ///Z-axis tap recognition threshold, 1 LSB = FS_XL/2^5. default 0
                0..4 tap_ths_z,
           },
           0x5A TAP_DUR rw
           {
                ///max time gap for double-tap recognition. default 0000 = 16/ODR_XL; other values step by 32/ODR_XL
                4..7 dur,
                ///quiet time after a detected tap. default 00 = 2/ODR_XL; other values step by 4/ODR_XL
                2..3 quiet,
                ///max duration of an overthreshold event to be recognized as a tap. default 00 = 4/ODR_XL; other values step by 8/ODR_XL
                0..1 shock,
           },
           0x5B WAKE_UP_THS rw
           {
                ///enables single/double-tap event. 0: single only (default), 1: both single and double
                7 single_double_tap,
                ///drives low-pass filtered data with user-offset correction (instead of HPF data) to the wake-up / activity-inactivity functions. default 0
                6 usr_off_on_wu,
                ///wake-up threshold, resolution set by wu_inact_ths_w in INACTIVITY_DUR. default 0
                0..5 wk_ths,
           },
           0x5C WAKE_UP_DUR rw
           {
                ///free-fall duration bit 5 (see FF_DUR[4:0] in FREE_FALL for the rest). 1 LSB = 1/ODR_XL. default 0
                7 ff_dur5,
                ///wake-up duration event. 1 LSB = 1/ODR_XL. default 00
                5..6 wake_dur,
                ///duration to enter sleep mode. default 0000 = 16 ODR; 1 LSB = 512/ODR_XL
                0..3 sleep_dur,
           },
           0x5D FREE_FALL rw
           {
                ///free-fall duration bits [4:0] (see ff_dur5 in WAKE_UP_DUR for bit 5). default 00000
                3..7 ff_dur,
                ///free-fall threshold.
                ///000: 156 mg
                ///001: 219 mg
                ///010: 250 mg
                ///011: 312 mg
                ///100: 344 mg
                ///101: 406 mg
                ///110: 469 mg
                ///111: 500 mg
                0..2 ff_ths,
           },
           ///Functions routing to INT1 pin (R/W). Output is the OR of these bits and INT1_CTRL
           0x5E MD1_CFG rw
           {
                ///routes activity/inactivity recognition event to INT1 (mode depends on sleep_status_on_int in INACTIVITY_DUR). default 0
                7 int1_sleep_change,
                ///routes single-tap recognition event to INT1. default 0
                6 int1_single_tap,
                ///routes wake-up event to INT1. default 0
                5 int1_wu,
                ///routes free-fall event to INT1. default 0
                4 int1_ff,
                ///routes double-tap event to INT1. default 0
                3 int1_double_tap,
                ///routes 6D event to INT1. default 0
                2 int1_6d,
                ///routes embedded-functions event to INT1. default 0
                1 int1_emb_func,
                ///routes sensor-hub communication-concluded event to INT1. default 0
                0 int1_shub,
           },
           ///Functions routing to INT2 pin (R/W). Output is the OR of these bits and INT2_CTRL
           0x5F MD2_CFG rw
           {
                ///routes activity/inactivity recognition event to INT2 (mode depends on sleep_status_on_int in INACTIVITY_DUR). default 0
                7 int2_sleep_change,
                ///routes single-tap recognition event to INT2. default 0
                6 int2_single_tap,
                ///routes wake-up event to INT2. default 0
                5 int2_wu,
                ///routes free-fall event to INT2. default 0
                4 int2_ff,
                ///routes double-tap event to INT2. default 0
                3 int2_double_tap,
                ///routes 6D event to INT2. default 0
                2 int2_6d,
                ///routes embedded-functions event to INT2. default 0
                1 int2_emb_func,
                ///routes the timestamp-overflow alert (within 5.6ms) to INT2
                0 int2_timestamp,
           },
           ///selects the ODR set supported when high-accuracy ODR (HAODR) mode is enabled. default 00
           0x62 HAODR_CFG rw haodr_sel[0..1],
           0x63 EMB_FUNC_CFG rw
           {
                ///when dual-channel mode is enabled, enables batching of accelerometer channel 2 in FIFO from the interface side. default 0
                7 xl_dualc_batch_from_if,
                ///masks execution trigger of embedded functions while gyroscope data are settling. default 0
                5 emb_func_irq_mask_g_settl,
                ///masks execution trigger of embedded functions while accelerometer data are settling. default 0
                4 emb_func_irq_mask_xl_settl,
                ///disables execution of the embedded functions; forces re-init procedures when cleared. default 0
                3 emb_func_disable,
           },
           ///UI-side handshake control for the UI/SPI2 shared registers (0x65-0x6A)
           0x64 UI_HANDSHAKE_CTRL rw
           {
                ///primary-interface acknowledge bit for the handshake; set by the device when the shared registers are free for the primary interface
                1 ui_shared_ack,
                ///primary-interface master requests access to the shared registers; master must clear this when done
                0 ui_shared_req,
           },
           ///UI/SPI2 shared register 0 (R/W), a volatile handshake byte between the primary and secondary interface hosts
           0x65 UI_SPI2_SHARED_0 rw ui_spi2_shared_0,
           0x66 UI_SPI2_SHARED_1 rw ui_spi2_shared_1,
           0x67 UI_SPI2_SHARED_2 rw ui_spi2_shared_2,
           0x68 UI_SPI2_SHARED_3 rw ui_spi2_shared_3,
           0x69 UI_SPI2_SHARED_4 rw ui_spi2_shared_4,
           0x6A UI_SPI2_SHARED_5 rw ui_spi2_shared_5,
           0x6B CTRL_EIS rw
           {
                ///enables and selects the ODR of the gyroscope EIS channel.
                ///00: EIS channel off (default)
                ///01: 1.92 kHz
                ///10: 960 Hz
                ///11: reserved
                6..7 odr_g_eis,
                ///gyroscope EIS-channel digital LPF bandwidth selection
                4 lpf_g_eis_bw,
                ///routes gyroscope EIS output to the OIS/UI output registers (2Eh-33h); when 1, gyro OIS data can't be read from primary IF. default 0
                3 g_eis_on_g_ois_out_reg,
                ///gyroscope EIS-channel full-scale.
                ///000: +-125 dps (default)
                ///001: +-250 dps
                ///010: +-500 dps
                ///011: +-1000 dps
                ///100: +-2000 dps
                ///101-111: reserved
                0..2 fs_g_eis,
           },
           ///OIS interrupt configuration. Writable from primary IF only when ois_ctrl_from_ui=1 in FUNC_CFG_ACCESS; otherwise read-only mirror of SPI2_INT_OIS
           0x6F UI_INT_OIS rw
           {
                ///enables OIS-chain DRDY on INT2 from the UI interface; takes priority over other INT2 settings
                7 int2_drdy_ois,
                ///masks OIS data-ready (accel and gyro independently) until filter settling ends. default 0
                6 drdy_mask_ois,
                ///disables OIS-chain output clamp during self-test. 0: outputs = 0x8000 during self-test, 1: real self-test outputs. default 0
                4 st_ois_clampdis,
           },
           ///OIS configuration register 1. Writable from primary IF only when ois_ctrl_from_ui=1; otherwise mirrors SPI2_CTRL1_OIS
           0x70 UI_CTRL1_OIS rw
           {
                ///SPI2 3- or 4-wire interface. 0: 4-wire (default), 1: 3-wire
                5 sim_ois,
                ///enables accelerometer OIS chain. default 0
                2 ois_xl_en,
                ///enables gyroscope OIS chain. default 0
                1 ois_g_en,
                ///in primary-IF full-control mode, enables reading OIS data via the SPI2_OUTx_OIS registers. default 0
                0 spi2_read_en,
           },
           ///OIS configuration register 2. Writable from primary IF only when ois_ctrl_from_ui=1; otherwise mirrors SPI2_CTRL2_OIS
           0x71 UI_CTRL2_OIS rw
           {
                ///gyroscope OIS digital LPF1 filter bandwidth selection
                3..4 lpf1_g_ois_bw,
                ///gyroscope OIS full-scale.
                ///000: +-125 dps
                ///001: +-250 dps
                ///010: +-500 dps
                ///011: +-1000 dps
                ///100: +-2000 dps
                ///101-111: reserved
                0..2 fs_g_ois,
           },
           ///OIS configuration register 3. Writable from primary IF only when ois_ctrl_from_ui=1; otherwise mirrors SPI2_CTRL3_OIS
           0x72 UI_CTRL3_OIS rw
           {
                ///selects accelerometer OIS-channel bandwidth. default 0
                3..5 lpf_xl_ois_bw,
                ///selects accelerometer OIS-channel full-scale.
                ///00: +-4 g (default)
                ///01: +-8 g
                ///10: +-16 g
                ///11: +-32 g
                0..1 fs_xl_ois,
           },
           ///accelerometer X-axis user-offset correction (two's complement, weight per usr_off_w in CTRL9), applied per usr_off_on_out/usr_off_on_wu. Range [-127,127]
           0x73 X_OFS_USR rw usr_offset_x,
           ///accelerometer Y-axis user-offset correction, same weighting/application as X_OFS_USR
           0x74 Y_OFS_USR rw usr_offset_y,
           ///accelerometer Z-axis user-offset correction, same weighting/application as X_OFS_USR
           0x75 Z_OFS_USR rw usr_offset_z,
           0x78 FIFO_DATA_OUT_TAG r
           {
                ///FIFO tag. Identifies sensor used for FIFO data.
                ///0x00: FIFO empty
                ///0x01: Gyroscope NC
                ///0x02: Accelerometer NC
                ///0x03: Temperature
                ///0x04: Timestamp
                ///0x05: CFG_Change
                ///0x06: Accelerometer NC_T_2
                ///0x07: Accelerometer NC_T_1
                ///0x08: Accelerometer 2xC
                ///0x09: Accelerometer 3xC
                ///0x0A: Gyroscope NC_T_2
                ///0x0B: Gyroscope NC_T_1
                ///0x0C: Gyroscope 2xC
                ///0x0D: Gyroscope 3xC
                ///0x0E: Sensor hub slave 0
                ///0x0F: Sensor hub slave 1
                ///0x10: Sensor hub slave 2
                ///0x11: Sensor hub slave 3
                ///0x12: Step counter
                ///0x13: SFLP game rotation vector
                ///0x16: SFLP gyroscope bias
                ///0x17: SFLP gravity vector
                ///0x19: Sensor hub nack
                ///0x1A: MLC result
                ///0x1B: MLC filter
                ///0x1C: MLC feature
                ///0x1D: Accelerometer dualC
                ///0x1E: Enhanced EIS gyroscope
                3..7 tag_sensor,
                ///2-bit counter which identifies the sensor time slot
                1..2 tag_cnt,
           },
           ///x axis output
           0x79 FIFO_DATA_OUT_X_L r,
           ///x axis output
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

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Accel {
     pub x: MicroGs<i32>,
     pub y: MicroGs<i32>,
     pub z: MicroGs<i32>
}

type MicrodegreesPerSecond<T> = Quantity<T, <UnitMicrodegrees as Div<UnitSeconds>>::Output>;

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct AngularVel {
     pub x_pitch: MicrodegreesPerSecond<i64>,
     pub y_roll: MicrodegreesPerSecond<i64>,
     pub z_yaw: MicrodegreesPerSecond<i64>,
}

pub struct Lsm6dsv32x<S: SpiHandle> {
    spi: S
}

impl <S: SpiHandle> Lsm6dsv32x<S> {
    pub fn new(spi: S) -> Self {
        Self {
            spi
        }
    }
    pub async fn setup(
        &mut self
    ) -> Result<(),<S::Bus as ErrorType>::Error> {
          //Use this function to perform initial setup of the IMU.

          // 1010: 6.66kHz ODR
          // 01: +-32g FS
          // 0: output from first stage digital filtering selected
          self.write_reg(RegCtrl, 0b1010_11_0_0 as u8).await?;
          // 1010: 6.66kHz ODR
          // 00: +-250dps FS
          // 0: not +-125dps
          self.write_reg(RegCtrl2G, 0b1010_0001).await?;
          Ok(())    
    }

     #[allow(unused_unsafe)]
     pub async fn raw_accel(&mut self) -> Result<(i16, i16, i16), <S::Bus as ErrorType>::Error> {
          Ok(unsafe {
               let accel_x: i16 = self.accel_x().await?.cast_signed(); // switched transmute into cast_signed for warning removal. if this doesnt work revert 
               let accel_y: i16 = self.accel_y().await?.cast_signed();
               let accel_z: i16 = self.accel_z().await?.cast_signed();

               (accel_x, accel_y, accel_z)
          })
     }
     
     #[allow(unused_unsafe)]
     pub async fn raw_gyro(&mut self) -> Result<(i16, i16, i16), <S::Bus as ErrorType>::Error> {
          Ok(unsafe {
               let gyro_pitch: i16 = self.gyro_pitch_rate().await?.cast_signed(); // switched transmute into cast_signed for warning removal. if this doesnt work revert
               let gyro_roll: i16 = self.gyro_roll_rate().await?.cast_signed();
               let gyro_yaw: i16 = self.gyro_yaw_rate().await?.cast_signed();

               (gyro_pitch, gyro_roll, gyro_yaw)
          })
     }

     pub async fn accel_sensitivity(&mut self) -> Result<i32, <S::Bus as ErrorType>::Error> {
          let mask = 0b0000_11_00;
          let reg = self.read_reg(RegCtrl1Xl).await?; 
          let real_val = (reg & mask) >> 2;
          Ok(
          match real_val {
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
          let mask = 0b0000_1111;
          let reg = self.read_reg(CTRL6).await?; 
          let real_val = (reg & mask);
          Ok(
          match real_val {
               0 => 125,
               1 => 250,
               2 => 500,
               3 => 1000,
               4 => 2000,
               5 => 4000,
               _ => 125,
          })
     }
     /// 0 = 125dps, 1 = 250dps, 2 = 500dps, 3 = 1000dps, 4 = 2000dps, 5 = 4000dps
     pub async fn set_gyro_sensitivity(&mut self, dps: i32) -> Result<(), <S::Bus as ErrorType>::Error> {
          let val: u8 = match dps {
               125  => 0b0000,
               250  => 0b0001,
               500  => 0b0010,
               1000 => 0b0011,
               2000 => 0b0100,
               4000 => 0b0101,
               _ => return Ok(()), // Or return a custom driver error for invalid input
          };

          let mask = 0b1111_0000; // FS_G[3:0] field mask
          let reg = self.read_reg(RegCtrl6).await?;
          
          // Clear the existing FS_G bits, then set the new value
          let new_reg = (reg & !mask) | (val << 4);

          self.write_reg(RegCtrl6, new_reg).await
     }

     
     pub async fn test_fs(&mut self) -> Result<u8, <S::Bus as ErrorType>::Error> {
          Ok(self.accel_fs().await?)
     }

     /// returns a tuple with units of ug (10^-6)
     pub async fn accel(&mut self) -> Result<Accel, <S::Bus as ErrorType>::Error> {
          let (raw_x, raw_y, raw_z) = self.raw_accel().await?;
          //sensitivity mode
          let fs = self.accel_sensitivity().await?;
          let scalar: i32 = 122 * fs/4;
          //xyz are corrected so that
          //x -> cable direction
          //yz follow from right hand rule, x as index finger
          let accel_x: i32 = scalar * (raw_x as i32);
          let accel_y: i32 = scalar * (raw_y as i32);
          let accel_z: i32 = scalar * (raw_z as i32);

          Ok(Accel {
               x: accel_x.with_units(),
               y: accel_y.with_units(),
               z: accel_z.with_units()
          })
     }

     pub async fn angular_vel(&mut self) -> Result<AngularVel, <S::Bus as ErrorType>::Error> {
          let (raw_pitch, raw_roll, raw_yaw) = self.raw_gyro().await?;
          //sensitivity mode TODO: read from chip
          let fs = self.gyro_sensitivity().await?;
          let scalar: i64 = 4375 * (fs as i64)/125;
          let gyro_pitch: i64 = scalar * (raw_pitch as i64);
          let gyro_roll: i64 = scalar * (raw_roll as i64);
          let gyro_yaw: i64 = scalar * (raw_yaw as i64);

          Ok(AngularVel {
               x_pitch: gyro_pitch.with_units(),
               y_roll: gyro_roll.with_units(),
               z_yaw: gyro_yaw.with_units()
          })
     }


     pub async fn read_manufacturer_id(&mut self) -> Result<u8, <S::Bus as ErrorType>::Error> {
          Ok(self.whoami().await?)
     }
    
     // TODO
     pub async fn accel_autoscale(&mut self) -> Result<Accel, <S::Bus as ErrorType>::Error> {
          let (raw_x, raw_y, raw_z) = self.raw_accel().await?;

          let fs = self.accel_sensitivity().await?;
          let scalar: i32 = 122 * fs/4;//* fs/4;
          //xyz are corrected so that
          //x -> cable direction
          //yz follow from right hand rule, x as index finger
          let accel_x: i32 = scalar * (raw_x as i32);
          let accel_y: i32 = scalar * (raw_y as i32);
          let accel_z: i32 = scalar * (raw_z as i32);

          let max_accel = accel_x.abs().max(accel_y.abs()).max(accel_z.abs());
          let sensitivity = match max_accel{
               0..=3500000 => 0,
               3500001..=7000000 => 1,
               7000001..=14000000 => 2,
               _ => 3
          };
          self.set_accel_sensitivity(sensitivity).await?;
          
          Ok(Accel {
               x: accel_x.with_units(),
               y: accel_y.with_units(),
               z: accel_z.with_units()
          })
     }
}


impl <S: SpiHandle> ReadLsm6dsv32x for Lsm6dsv32x<S> {
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

impl <S: SpiHandle> WriteLsm6dsv32x  for Lsm6dsv32x<S> {
    type Error = <S::Bus as ErrorType>::Error;

    async fn write_contiguous_regs(
        &mut self,
        addr: impl WritableAddr,
        values: &[u8]
    ) -> Result<(), Self::Error> {
        let mut bus = self.spi.select().await;

        let addr: u8 = addr.as_addr() & 0b0111_1111;

        bus.write(&[addr]).await?;
        bus.write(values).await?;

        Ok(())
    }

}