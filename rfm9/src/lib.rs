#![no_std]

use core::{fmt::Debug, mem};

use defmt::{error, info};
use dev_csr::dev_csr;
use embassy_futures::yield_now;
//use embassy_futures::yield_now;
use embedded_hal::spi::{ ErrorKind as SpiError, ErrorType};
use embedded_hal_async::spi::SpiBus;
use spi_handle::SpiHandle;
use uunit::{Milliamps, WithUnits};

dev_csr! {
    dev Rfm9{
        regs{
            /// LoRa base-band FIFO data input/output. FIFO is cleared an not 
            /// accessible when device is in SLEEPmode
            0x00 FIFO rw fifo,
            0x01 OP_MODE rw {
                /// 0 is FSK Mode, 1 is LoRA mode
                /// Can only be modified in sleep mode
                /// Write opperation on other devices is ignored
                7 rw long_range_mode,
                /// This bit operates when device is in LoRa mode; 
                /// if set it allows access to FSK registers page 
                /// located in address space (0x0D:0x3F) while in LoRa mode
                /// 0: Access LoRa registers page 0x0D:0x3F
                /// 1: Access FSK registers page(in mode LoRa)0x0D:0x3F
                6 rw access_shared_reg,
                /// Access Low Frequency Mode registers 
                /// 0: High Frequency Mode (access to HF test registers) 
                /// 1: Low Frequency Mode(access to LF test registers)
                3 rw low_freq_mode_on,
                /// Device modes 
                /// 000 : SLEEP 
                /// 001 : STDBY 
                /// 010 : Frequency synthesis TX(FSTX) 
                /// 011 : Transmit(TX) 
                /// 100 : Frequency synthesis RX(FSRX) 
                /// 101 : Receive continuous(RXCONTINUOUS) 
                /// 110 : receive single(RXSINGLE) 
                /// 111 : Channel activity detection(CAD)
                0..2 rw mode
                //?rwt?
            },
            /// MSB or FR carrier frequency
            0x06 FR_MSB rw frf_msb[16..23],
            0x07 FR_MID rw frf_mid[8..15],
            /// LSB or RF carrier frequency
            0x08 FR_LSB rw frf_lsb[0..7],
            0x09 PA_CONFIG rw {
                /// Selects PA output pin
                /// 0: RFO pin. Output power is limited to +14dBm
                /// 1: PA_BOOST pin. Output power is limited to +20 dBm
                7 pa_select,
                /// Select max output power.
                /// PMax = 10.8 + 0.6*MaxPower [dBm]
                4..6 max_power,
                /// POut = PMax - (15-OutputPower) if PaSelect = 0 (RFO pin)
                /// POut = 17 - (15-OutputPower) if PaSelect = 1 (PA_BOOST pin)
                0..3 output_power
            },
            /// Rise/Fall time of ramp up/down in FSK
            /// 0000 -> 3.4ms 
            /// 0001 -> 2ms 
            /// 0010 -> 1ms 
            /// 0011 -> 500us 
            /// 0100 -> 250us 
            /// 0101 -> 125us 
            /// 0110 -> 100us 
            /// 0111 -> 62us 
            /// 1000 -> 50us 
            /// 1001 -> 40us 
            /// 1010 -> 31us 
            /// 1011 -> 25us 
            /// 1100 -> 20us 
            /// 1101 -> 15us 
            /// 1110 -> 12us 
            /// 1111 -> 10us
            0x0A PA_RAMP rw pa_ramp, /// IDK
            0x0B OCP rw{
                /// Trimming of OCP current: 
                /// Imax = 45 + 5 * OcpTrim[mA] if OcpTrim <= 15 (120mA)/
                /// Imax = -30 + 10 * OcpTrim[mA] if 15 < OcpTrim <= 27 (130 to 240mA) 
                /// Imax = 240mA for higher settings 
                /// Default Imax=100mA
                5 ocp_on,
                0..4 ocp_trim,
            },
            0x0C LNA rw {
                ///High Frequency (RFI_HF) LNA currentadjustment, 00 -> Default LNA current
                ///11 -> Boost on, 150% LNA current
                0..1 lna_boost_hf,
                ///Low Frequency (RFI_LF) LNA current adjustment, 00 -> Default LNA current
                ///Other -> Reserved
                3..4 lna_boost_lf,
                ///LNA gain setting: 
                ///000 -> not used
                ///001 -> G1 = maximum gain 010 G2
                ///011 -> G 3
                ///100 -> G4
                ///101 -> G5
                ///110 -> G6 = minimum gain 
                ///111  -> not used
                5..7 lna_gain
            },
            ///SPI interface address pointer in FIFO data buffer.
            0x0D FIFO_ADDR_PTR rw fifo_addr_ptr,
            ///write base address in FIFO data buffer for TX modulator
            0x0E FIFO_TX_BASE_ADDR rw fifo_tx_base_addr,
            ///read base address in FIFO data buffer for RX demodulator
            0x0F FIFO_RX_BASE_ADDR rw fifo_rx_base_addr,
            ///Start address (in data buffer) of last packet received
            0x10 FIFO_RX_CURRENT_ADDR r fifo_rx_current_addr,
            0x11 IRQ_FLAGS_MASK rw {
                ///Cad Detected Interrupt Mask: setting this bit masks the corresponding IRQ in RegIrqFlags
                0 cad_detected_mask,
                ///FHSS change channel interrupt mask: setting this bit masks the corresponding IRQ in RegIrqFlags
                1 fhss_change_channel_mask,
                ///CAD complete interrupt mask: setting this bit masks the corresponding IRQ in RegIrqFlags
                2 cad_done_mask,
                ///FIFO Payload transmission complete interrupt mask: setting this bit masks the corresponding IRQ in RegIrqFlags
                3 tx_done_mask,
                ///Valid header received in Rx mask: setting this bit masks the corresponding IRQ in RegIrqFlags
                4 valid_header_mask,
                ///Payload CRC error interrupt mask: setting this bit masks the corresponding IRQ in RegIrqFlags
                5 payload_crc_error_mask,
                ///Packet reception complete interrupt mask: setting this bit masks the corresponding IRQ in RegIrqFlags
                6 rx_done_mask,
                ///Timeout interrupt mask: setting this bit masks the corresponding IRQ in RegIrqFlags
                7 rx_timeout_mask
            },
            0x12 IRQ_FLAGS rw {
                /// Timeout interrupt: writing a 1 clears the IRQ
                7 rx_time_out,
                /// Packet reception complete interrupt: writing a 1 clears the IRQ
                6 rx_done,
                /// Payload CRC error interrupt: writing a 1 clears the IR
                5 payload_crc_err,
                /// Valid header received in Rx: writing a 1 clears theIRQ
                4 valid_header,
                /// FIFO Payload transmission complete interrupt: writing a 1 clears the IRQ
                3 tx_done,
                /// CAD complete: write to clear: writing a 1 clears the IRQ
                2 cad_done,
                /// FHSS change channel interrupt: writing a 1 clears the IR
                1 fhss_change_channel,
                /// Valid Lora signal detected during CAD operation: writing a 1clears the IR
                0 cad_detected,
            },
            /// Number of payload bytes of latest packet received
            0x13 RX_NB_BYTES r fifo_rx_nb_bytes,
            /// Number of valid headers received since last transition intoRx mode, MSB(15:8). Header and packet counters are reseted in Sleep mode
            0x14 RX_HEADER_CNT_VALUE_MSB r valid_header_cnt[8..15],
            /// Number of valid headers received since last transition intoRx mode, LSB(7:0). Header and packet counters are reseted in Sleep mode.
            0x15 RX_HEADER_CNT_VALUE_LSB r valid_header_cnt[0..7],
            /// Number of valid packets received since last transition into Rx mode, MSB(15:8). Header and packet counters are reseted in Sleep mode.
            0x16 RX_PACKET_CNT_VALUE_MSB rw valid_packet_cnt[8..15],
            /// Number of valid packets received since last transition intoRx mode, LSB(7:0). Header and packet counters are reseted in Sleep mode
            0x17 RX_PACKET_CNT_VALUE_LSB r valid_packet_cnt[0..7],
            0x18 MODEM_STAT r {
                ///Signal detected 
                0 signal_detected,
                /// 1 Signal synchronized
                1 signal_synchronized,
                /// 2 RX on-going
                2 rx_on_going,
                /// 3 Header info valid
                3 header_info_valid,
                /// 4 Modem clear
                4 modem_clear,
                /// Coding rate of last headerreceived
                5..7 rx_coding_rate,
            },
            /// Estimation of SNR on last packet received.In two’s compliment format multiplied by 4. SNR[dB] = PacketSnr[twos complement]/4
            0x19 PKT_SNR_VALUE r packet_snr,
            /// RSSI of the latest packet received (dBm): 
            ///RSSI[dBm] = -157 + Rssi (using HF output port, SNR >=0)
            ///or RSSI[dBm] = -164 + Rssi (using LF output port, SNR >= 0)
            /// (see section 5.5.5 for details)
            0x1A PKT_RSSI_VALUE r packet_rssi,
            /// Current RSSI value (dBm)
            /// RSSI[dBm] = -157 + Rssi (using HF outputport)
            /// or RSSI[dBm] = -164 + Rssi (using LF outputport)
            /// (see section 5.5.5 for details*/
            0x1B RSSI_VALUE r rssi,
            0x1C HOP_CHANNEL r {
                /// Current value of frequency hopping channel inuse.
                0..5 fhss_present_channel,
                /// CRC Information extracted from the received packetheader
                /// (Explicit header mode only)
                /// 0 -> Header indicates CRC off
                /// 1 -> Header indicates CRC on
                6 crc_on_payload,
                /// PLL failed to lock while attempting a TX/RX/CAD operation
                /// 1 -> PLL did not lock
                /// 0 -> PLL did lock
                7 pll_time_out,
            },
            0x1D MODEM_CONFIG_1 rw {
                /// 0 -> Explicit Header mode
                /// 1 -> Implicit Header mode
                0 implicit_header_mode_on,
                /// Error coding rate
                /// 001 -> 4/5
                /// 010 -> 4/6
                /// 011 -> 4/7
                /// 100 -> 4/8
                /// All other values -> reserved
                /// In implicit header mode should be set on receiver todetermineexpected coding rate. See 4.1.1.3
                1..3 coding_rate,
                /// Signal bandwidth:
                /// 0000 -> 7.8 kHz
                /// 0001 -> 10.4 kHz
                /// 0010 -> 15.6 kHz
                /// 0011 -> 20.8kHz
                /// 0100 -> 31.25 kHz
                /// 0101 -> 41.7 kHz
                /// 0110 -> 62.5 kHz
                /// 0111 -> 125 kHz
                /// 1000 -> 250 kHz
                /// 1001 -> 500 kHz
                /// other values -> reserved
                /// In the lower band (169MHz), signal bandwidths 8&9 are not supported)
                4..7 bw
            },
            0x1E MODEM_CONFIG_2 rw {
                /// RX Time-Out MSB
                0..1 symb_timeout[8..9],
                /// Enable CRC generation and check onpayload:
                /// 0 -> CRC disable
                /// 1 -> CRC enable
                /// If CRC is needed, RxPayloadCrcOn should beset: 
                /// - in Implicit header mode: on Tx and Rx side
                /// - in Explicit header mode: on the Tx side alone (recoveredfromtheheader in Rx side)
                2 rx_payload_crc_on,
                /// 0 -> normal mode, a single packet is sent
                /// 1 -> continuous mode, send multiple packets across the FIFO(used for spectral analysis)
                3 tx_continuous_mode,
                /// SF rate (expressed as a base-2logarithm)
                /// 6 -> 64 chips / symbol
                /// 7 -> 128 chips / symbol
                /// 8 -> 256 chips / symbol
                /// 9 -> 512 chips / symbol
                /// 10 -> 1024 chips / symbol
                /// 11 -> 2048 chips / symbol
                /// 12 -> 4096 chips / symbol
                /// other values reserved.
                4..7 spreading_factor
            },
            /// RX Time-Out LSB
            /// RX operation time-out value expressed as number of symbols:
            /// TimeOut = SymbTimeout * Ts
            0x1F SYMB_TIMEOUT_LSB rw symb_timeout[0..7],
            /// Preamble length MSB, = PreambleLength + 4.25Symbols
            /// See 4.1.1 for more details
            0x20 PREAMBLE_MSB rw preamble_length[8..15],
            /// Preamble Length LSB
            0x21 PREAMBLE_LSB rw preamble_length[0..7],
            /// Payload length in bytes. The register needs to be set in implicit header mode for the expected packet length. A 0 value is not permitted
            0x22 PAYLOAD_LENGTH rw payload_length,
            /// Maximum payload length; if header payload length exceeds value a header CRC error is generated. Allows filtering of packet with a bad size.
            0x23 MAX_PAYLOAD_LENGTH rw payload_max_length,
            /// Symbol periods between frequency hops. (0 = disabled). 1st hop always happen after the 1st header symbol
            0x24 HOP_PERIOD rw frew_hopping_period,
            /// Current value of RX databuffer pointer (address of last byte written by Lora receiver)
            0x25 FIFO_RX_BYTE_ADDR r fifo_rx_byte_addr_ptr,
            0x26 MODEM_CONFIG_3 rw {
                /// 0 -> LNA gain set by register LnaGain
                /// 1 -> LNA gain set by the internal AGC loo
                2 acg_auto_on,
                /// 0 -> Disabled
                /// 1 -> Enabled; mandated for when the symbol lengthexceeds16ms
                3 low_data_rate_optimize
            },
            /// Data rate offset value, used in conjunction with AFC
            0x27 PPM_CORRECTION rw ppm_correction,
            0x28 FEI_MSB r {
                /// Estimated frequency error from modem
                /// MSB of RF Frequency Error
                /// F_error = (FreqError * 2^24)/F_xtal * BW[kHz]/500
                0..3 freq_error[16..19]
            },
            /// Middle byte of RF FrequencyError
            0x29 FEI_MID r freq_error[8..15],
            /// LSB of RF Frequency Error
            0x2A FEI_LSB r freq_error[0..7],
            /// Wideband RSSI measurement used to locally generate a random number
            0x2C RSSI_WIDE_BAND r rssi_wide_band,
            /// See errata note
            0x2F IF_FREQ_2 rw if_freq_2,

            /// Version code of the chip. Bits 7-4 give the full revision number;
            /// bits 3-0 give the metal mask revision number.
            0x42 VERSION r version,

            0x4D PA_DAC rw pa_dac
        }
    }
}

pub enum ErrorCodingRate {
    FourFifths = 0b001,
    FourSixths = 0b010,
    FourSevenths = 0b011,
    FourEighths = 0b100
}


#[derive(Debug, Copy, Clone, Eq, PartialEq, Ord, PartialOrd, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum SignalBandwidth {
    Cr7_8 = 0b0000,
    Cr10_4 = 0b0001,
    Cr15_6 = 0b0010,
    Cr20_8 = 0b0011,
    Cr31_25 = 0b0100,
    Cr41_7 = 0b0101,
    Cr62_5 = 0b0110,
    Cr125 = 0b0111,
    Cr250 = 0b1000,
    Cr500 = 0b1001
}

#[derive(Debug, Copy, Clone, Eq, PartialEq, Ord, PartialOrd, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum Mode {
    Sleep = 0b000,
    Stdby = 0b001,
    Fstx =  0b010,
    Tx = 0b011,
    Fsrx =  0b100,
    RxContinuous = 0b101,
    RxSingle = 0b110,
    Cad = 0b111,
}

pub struct Rfm9<S: SpiHandle> {
    spi: S
}

#[derive(Debug, Copy, Clone, Eq, PartialEq, Ord, PartialOrd, Hash)]
pub enum Rfm9Error {
    Spi(SpiError),
    Crc,
    Timeout,
}

impl From<SpiError> for Rfm9Error {
    fn from(value: SpiError) -> Self {
        Rfm9Error::Spi(value)
    }
}

type Error = Rfm9Error;

impl <S: SpiHandle> Rfm9<S> {
    pub fn new(spi: S) -> Self {
        Self {
            spi
        }
    }

    pub async fn init(&mut self) -> Result<(), Error> {
        //let config1 = self.read_reg(RegModemConfig1).await?;
        self.set_mode(Mode::Sleep).await?;
        self.set_fifo_tx_base_addr(0).await?;
        self.set_fifo_rx_base_addr(0).await?;
        let lna = self.read_reg(RegLna).await?;
        self.write_reg(RegLna, lna | 0x03).await?;
        self.write_reg(RegModemConfig3, 0x04).await?;
        self.set_mode(Mode::Stdby).await?;
        Ok(())
    }

    pub async fn mode(&mut self) -> Result<Mode, Error> {
        let mode: Mode = unsafe {
            mem::transmute(self.read_reg(RegOpMode).await? & 0b0000_0111)
        };

        Ok(mode)
    }

    pub async fn set_mode(&mut self, mode: Mode) -> Result<(), Error> {
        self.write_reg(RegOpMode, 0b1000_0000 | mode as u8).await?;
        Ok(())
    }

    pub async fn read_version(&mut self) -> Result<u8, Error>{
        let version: u8 = self.version().await.unwrap();
        Ok(version)
    } 

    pub async fn calibrate(&mut self) -> Result<(), Error> {
        self.set_mode(Mode::Sleep).await?;
        
        // sets frequency to 434 MHz
        self.set_frf_msb(0x6c).await?;
        self.set_frf_mid(0x80).await?;
        self.set_frf_lsb(0x00).await?;

        Ok(())
    }

    pub async fn set_ocp(&mut self, current_limit: Milliamps<u8>) -> Result<(), Error> {
        let current_limit = current_limit.value;

        let trim = if current_limit < 45 {
            0
        } else if current_limit < 130 {
            (current_limit - 45)/5
        } else if current_limit < 240 {
            (current_limit + 30)/10
        } else {
            27
        };

        self.write_reg(RegOcp, 0b10_0000 + trim).await?;

        Ok(())
    }

    pub async fn use_high_power(&mut self) -> Result<(), Error> {
        self.set_pa_dac(0x87).await?;
        self.set_ocp(140u8.with_units()).await?;
        self.write_reg(RegPaConfig, 0b1_100_1111).await?;
        Ok(())
    }

    pub async fn use_explicit_headers(&mut self) -> Result<(), Error> {
        //let config1 = self.read_reg(RegModemConfig1).await?;
        self.write_reg(RegModemConfig1, 0x72).await?;
        Ok(())
    }

    pub async fn transmit(&mut self, data: &[u8]) -> Result<(), Error> {
        let len: u8 = data.len().try_into().unwrap();

        self.set_mode(Mode::Stdby).await?;
        self.use_explicit_headers().await?;

        self.write_reg(RegIrqFlags, 0).await?;
        // Write FIFO
        self.set_fifo_addr_ptr(0x00).await?;
        //self.set_fifo_tx_base_addr(0x00).await?;
        self.set_payload_length(0).await?;


        self.set_payload_length(len).await?;
        //self.write_contiguous_regs(RegFifo, &[0x80]).await?;
        //self.write_contiguous_regs(RegFifo, data).await?;
        for byte in data {
            self.write_reg(RegFifo, *byte).await?;
        }

        //self.set_payload_length(26).await?;
        self.write_reg(RegPayloadLength, len).await?;
        info!("Len: {}", len);

        self.set_mode(Mode::Tx).await?;

        /*while !self.tx_done().await? {
            yield_now().await;
        }*/

        Ok(())
    }

    pub async fn recieve(&mut self, data: &mut [u8]) -> Result<u8, Error> {
        self.write_reg(RegIrqFlagsMask, 0).await?;
        self.write_reg(RegIrqFlags, 0b1100_0000).await?;
        
        self.set_fifo_addr_ptr(0).await?;

        self.set_mode(Mode::RxSingle).await?;

        loop {
            let flags = self.read_reg(RegIrqFlags).await?;
            let rx_time_out = flags & 0b1000_0000 > 0;
            let rx_done = flags & 0b0100_0000 > 0;

            if rx_done {
                break;
            } else if rx_time_out {
                return Err(Rfm9Error::Timeout)
            }

            yield_now().await;
        }

        if self.payload_crc_err().await? {
            self.set_mode(Mode::Stdby).await?;
            return Err(Rfm9Error::Crc)
        }

        //let rx_cur_addr: u8 = self.fifo_rx_current_addr().await?;
        //self.set_fifo_addr_ptr(rx_cur_addr).await?;
        let len: u8 = self.fifo_rx_nb_bytes().await?;
        info!("{}", len);
        self.read_contiguous_regs(RegFifo, &mut data[..len as usize]).await?;
        self.set_mode(Mode::Stdby).await?;
        Ok(len)
    }
}


impl <S: SpiHandle> ReadRfm9 for Rfm9<S> {
    type Error = <S::Bus as ErrorType>::Error;

    async fn read_contiguous_regs(
        &mut self,
        addr: impl ReadableAddr,
        out: &mut [u8]
    ) -> Result<(), Self::Error> {
        let mut bus = self.spi.select().await;

        // set rw bit
        // write = 1, read = 0
        let addr: u8 = addr.as_addr() & 0b0111_1111;
        
        bus.write(&[addr]).await?;
        bus.transfer_in_place(out).await?;
        Ok(())
    }
}

impl <S: SpiHandle> WriteRfm9 for Rfm9<S> {
    type Error = <S::Bus as ErrorType>::Error;

    async fn write_contiguous_regs(
        &mut self,
        addr: impl WritableAddr,
        values: &[u8]
    ) -> Result<(), Self::Error> {
        let mut bus = self.spi.select().await;

        let addr: u8 = addr.as_addr() | 0b1000_0000;

        bus.write(&[addr.as_addr()]).await?;
        bus.write(values).await?;

        Ok(())
    }

}