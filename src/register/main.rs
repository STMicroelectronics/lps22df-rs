use super::super::{
    BusOperation, DelayNs, Error, Lps22df, RegisterOperation, SensorOperation, bisync,
    register::OnState,
};

use bitfield_struct::bitfield;
use derive_more::TryFrom;

use st_mem_bank_macro::register;

/// Register addresses for LPS22DF device.
#[repr(u8)]
#[derive(Clone, Copy, PartialEq)]
pub enum Reg {
    InterruptCfg = 0x0B,
    ThsPL = 0x0C,
    ThsPH = 0x0D,
    IfCtrl = 0x0E,
    WhoAmI = 0x0F,
    CtrlReg1 = 0x10,
    CtrlReg2 = 0x11,
    CtrlReg3 = 0x12,
    CtrlReg4 = 0x13,
    FifoCtrl = 0x14,
    FifoWtm = 0x15,
    RefPL = 0x16,
    RefPH = 0x17,
    I3cIfCtrl = 0x19,
    RpdsL = 0x1A,
    RpdsH = 0x1B,
    IntSource = 0x24,
    FifoStatus1 = 0x25,
    FifoStatus2 = 0x26,
    Status = 0x27,
    PressOutXl = 0x28,
    PressOutL = 0x29,
    PressOutH = 0x2A,
    TempOutL = 0x2B,
    TempOutH = 0x2C,
    FifoDataOutPressXl = 0x78,
    FifoDataOutPressL = 0x79,
    FifoDataOutPressH = 0x7A,
}

/// INTERRUPT_CFG (0x0B)
///
/// Interrupt mode for pressure acquisition configuration (R/W)
#[register(address = Reg::InterruptCfg, access_type = "Lps22df<B, T, OnState>")]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct InterruptCfg {
    /// Enable interrupt generation on pressure high event (1 bit)
    /// (0: disable interrupt request; 1: enable interrupt request on pressure value higher than preset threshold)
    #[bits(1)]
    pub phe: u8,
    /// Enable interrupt generation on pressure low event (1 bit)
    /// (0: disable interrupt request; 1: enable interrupt request on pressure value lower than preset threshold)
    #[bits(1)]
    pub ple: u8,
    /// Latch interrupt request to the INT_SOURCE register (1 bit)
    /// (0: interrupt request not latched; 1: interrupt request latched)
    #[bits(1)]
    pub lir: u8,
    #[bits(1, access = RO)]
    pub not_used_01: u8,
    /// Reset AUTOZERO function (1 bit)
    /// (0: normal mode; 1: reset AUTOZERO function)
    #[bits(1)]
    pub reset_az: u8,
    /// Enable AUTOZERO function (1 bit)
    /// (0: normal mode; 1: AUTOZERO enabled)
    #[bits(1)]
    pub autozero: u8,
    /// Reset AUTOREFP function (1 bit)
    /// (0: normal mode; 1: reset AUTOREFP function)
    #[bits(1)]
    pub reset_arp: u8,
    /// Enable AUTOREFP function (1 bit)
    /// (0: normal mode; 1: AUTOREFP enabled)
    #[bits(1)]
    pub autorefp: u8,
}

/// THS_P_L - THS_P_H (0x0C - 0x0D)
///
/// User-defined threshold value for pressure interrupt event  (R/W)
/// This register contains the 15-bit threshold value used for pressure interrupt generation.
/// The threshold value is expressed as: THS_P (15-bit unsigned) = Desired interrupt threshold (hPa) × 16.
#[register(address = Reg::ThsPL, access_type = "Lps22df<B, T, OnState>", override_type = u16)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u16, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u16, order = Lsb))]
pub struct ThsP {
    #[bits(15)]
    pub ths: u16,
    #[bits(1, access = RO)]
    pub not_used_01: u8,
}

/// IF_CTRL (0x0E)
///
/// Interface control register (R/W)
#[register(address = Reg::IfCtrl, access_type = "Lps22df<B, T, OnState>")]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct IfCtrl {
    #[bits(1, access = RO)]
    pub not_used_01: u8,
    /// Disable pull-up on the CS pin (1 bit)
    /// (0: CS pin with pull-up; 1: CS pin pull-up disconnected)
    #[bits(1)]
    pub cs_pu_dis: u8,
    /// Disable pull-down on the INT pin (1 bit)
    /// (0: INT pin with pull-down; 1: INT pin pull-down disconnected)
    #[bits(1)]
    pub int_pd_dis: u8,
    /// Enable pull-up on the SDO pin (1 bit)
    /// (0: SDO pin pull-up disconnected; 1: SDO pin with pull-up)
    #[bits(1)]
    pub sdo_pu_en: u8,
    /// Enable pull-up on the SDA pin (1 bit)
    /// (0: SDA pin pull-up disconnected; 1: SDA pin with pull-up)
    #[bits(1)]
    pub sda_pu_en: u8,
    /// SPI serial interface mode selection (1 bit)
    /// (0: 4-wire interface; 1: 3-wire interface)
    #[bits(1)]
    pub sim: u8,
    /// Disable I²C and I3C digital interfaces (1 bit)
    /// (0: enable I²C and I3C digital interfaces; 1: disable I²C and I3C digital interfaces)
    #[bits(1)]
    pub i2c_i3c_dis: u8,
    /// Enable INT pin with MIPI I3C (1 bit)
    /// (0: INT disabled with MIPI I3C; 1: INT enabled with MIPI I3C)
    #[bits(1)]
    pub int_en_i3c: u8,
}

/// WHO_AM_I (0x0F)
///
/// WHO_AM_I register (R), read-only, fixed value 0xB4.
#[register(address = Reg::WhoAmI, access_type = "Lps22df<B, T, OnState>")]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct WhoAmI {
    /// Device identification value.
    #[bits(8)]
    pub id: u8,
}

/// CTRL_REG1 (0x10)
///
/// Control register 1 (R/W)
#[register(address = Reg::CtrlReg1, access_type = "Lps22df<B, T, OnState>")]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct CtrlReg1 {
    /// Averaging selection for pressure and temperature (3 bits)
    /// Configures resolution and power consumption (see datasheet Table 18)
    #[bits(3)]
    pub avg: u8,
    /// Output data rate selection (4 bits)
    /// Controls frequency of pressure and temperature data updates (see datasheet Table 17)
    #[bits(4)]
    pub odr: u8,
    #[bits(1, access = RO)]
    pub not_used_01: u8,
}

/// CTRL_REG2 (0x11)
///
/// Control register 2 (R/W)
#[register(address = Reg::CtrlReg2, access_type = "Lps22df<B, T, OnState>")]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct CtrlReg2 {
    /// Enable one-shot mode (1 bit)
    /// (0: idle mode; 1: a new dataset is acquired)
    #[bits(1)]
    pub oneshot: u8,
    #[bits(1, access = RO)]
    pub not_used_01: u8,
    /// Software reset (1 bit)
    /// (0: normal mode; 1: software reset, self-clears when reset is completed)
    #[bits(1)]
    pub swreset: u8,
    /// Block data update (1 bit)
    /// (0: continuous update; 1: output registers not updated until MSB and LSB have been read)
    #[bits(1)]
    pub bdu: u8,
    /// Enable low-pass filter on pressure data (1 bit)
    /// (0: disable; 1: enable)
    #[bits(1)]
    pub en_lpfp: u8,
    /// Low-pass filter configuration (1 bit)
    /// (0: ODR/4; 1: ODR/9)
    #[bits(1)]
    pub lfpf_cfg: u8,
    #[bits(1, access = RO)]
    pub not_used_02: u8,
    /// Reboot memory content (1 bit)
    /// (0: normal mode; 1: reboot memory content, self-clears after boot)
    #[bits(1)]
    pub boot: u8,
}

/// CTRL_REG3 (0x12)
///
/// Control register 3 (R/W)
#[register(address = Reg::CtrlReg3, access_type = "Lps22df<B, T, OnState>")]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct CtrlReg3 {
    /// Register address automatically incremented during multiple byte access (1 bit)
    /// (0: disable; 1: enable)
    #[bits(1)]
    pub if_add_inc: u8,
    /// Push-pull/open-drain selection on interrupt pin (1 bit)
    /// (0: push-pull; 1: open-drain)
    #[bits(1)]
    pub pp_od: u8,
    #[bits(1, access = RO)]
    pub not_used_02: u8,
    /// Select interrupt active-high or active-low (1 bit)
    /// (0: active-high; 1: active-low)
    #[bits(1)]
    pub int_h_l: u8,
    #[bits(4, access = RO)]
    pub not_used_01: u8,
}

/// CTRL_REG4 (0x13)
///
/// Control register 4 (R/W)
#[register(address = Reg::CtrlReg4, access_type = "Lps22df<B, T, OnState>")]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct CtrlReg4 {
    /// Data-ready pulsed on INT pin (1 bit)
    /// (0: disable; 1: enable data-ready pulse on INT pin, pulse width ~5 μs)
    #[bits(1)]
    pub int_f_ovr: u8,
    /// Interrupt signal on INT pin (1 bit)
    /// (0: disable; 1: enable)
    #[bits(1)]
    pub int_f_wtm: u8,
    /// FIFO full flag on INT pin (1 bit)
    /// (0: FIFO empty; 1: FIFO full with 128 unread samples)
    #[bits(1)]
    pub int_f_full: u8,
    #[bits(1, access = RO)]
    pub not_used_02: u8,
    /// Interrupt enable on INT pin (1 bit)
    /// (0: disable; 1: enable)
    #[bits(1)]
    pub int_en: u8,
    /// Data-ready signal on INT pin (1 bit)
    /// (0: disable; 1: enable)
    #[bits(1)]
    pub drdy: u8,
    /// Data-ready pulse on INT pin (1 bit)
    /// (0: disable; 1: enable)
    #[bits(1)]
    pub drdy_pls: u8,
    #[bits(1, access = RO)]
    pub not_used_01: u8,
}

/// FIFO_CTRL (0x14)
///
/// FIFO control register (R/W)
#[register(address = Reg::FifoCtrl, access_type = "Lps22df<B, T, OnState>")]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct FifoCtrl {
    /// FIFO mode selection (2 bits)
    /// Selects FIFO operating mode (see datasheet Table 21)
    #[bits(2)]
    pub f_mode: u8,
    /// Enables triggered FIFO modes (1 bit)
    #[bits(1)]
    pub trig_modes: u8,
    /// Stop-on-FIFO watermark (1 bit)
    /// Enables FIFO watermark level use
    #[bits(1)]
    pub stop_on_wtm: u8,
    #[bits(4, access = RO)]
    pub not_used_01: u8,
}

/// FIFO_WTM (0x15)
///
/// FIFO threshold setting register (R/W)
#[register(address = Reg::FifoWtm, access_type = "Lps22df<B, T, OnState>")]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct FifoWtm {
    /// FIFO threshold watermark level (7 bits)
    #[bits(7)]
    pub wtm: u8,
    #[bits(1, access = RO)]
    pub not_used_01: u8,
}

/// REF_P_L - REF_P_H (0x16 - 0x17)
///
/// Reference pressure LSB data (R)
/// Contains reference pressure value used for AUTOZERO or AUTOREFP functions.
/// The value is expressed as two's complement and stored in REF_P_H and REF_P_L registers.
#[register(address = Reg::RefPL, access_type = "Lps22df<B, T, OnState>")]
#[cfg_attr(feature = "bit_order_msb", bitfield(u16, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u16, order = Lsb))]
pub struct RefP {
    #[bits(16)]
    pub refp: i16,
}

/// I3C_IF_CTRL_ADD (0x19)
///
/// Interface configuration register (R/W)
#[register(address = Reg::I3cIfCtrl, access_type = "Lps22df<B, T, OnState>")]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct I3cIfCtrl {
    /// I3C bus available time selection (2 bits)
    /// Selects bus available time when I3C IBI is used
    #[bits(2)]
    pub i3c_bus_avb_sel: u8,
    #[bits(3, access = RO)]
    pub not_used_02: u8,
    /// Enables antispike filters (1 bit)
    /// (0: antispike filters managed by protocol; 1: antispike filters always enabled)
    #[bits(1)]
    pub asf_on: u8,
    #[bits(2, access = RO)]
    pub not_used_01: u8,
}

/// RPDS_L - RPDS_H (0x1A - 0x1B)
///
/// Pressure offset (R/W)
#[register(address = Reg::RpdsL, access_type = "Lps22df<B, T, OnState>", override_type = u16)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u16, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u16, order = Lsb))]
pub struct Rpds {
    #[bits(16)]
    pub offset: i16,
}

/// INT_SOURCE (0x24)
///
/// Interrupt source register (read only)
#[register(address = Reg::IntSource, access_type = "Lps22df<B, T, OnState>")]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct IntSource {
    /// Differential pressure high event (1 bit)
    /// (0: no interrupt; 1: high differential pressure event occurred)
    #[bits(1)]
    pub ph: u8,
    /// Differential pressure low event (1 bit)
    /// (0: no interrupt; 1: low differential pressure event occurred)
    #[bits(1)]
    pub pl: u8,
    /// Interrupt active (1 bit)
    /// (0: no interrupt generated; 1: one or more interrupt events generated)
    #[bits(1)]
    pub ia: u8,
    #[bits(4, access = RO)]
    pub not_used_01: u8,
    /// Boot phase indication (1 bit)
    /// (0: boot phase not running; 1: boot phase running)
    #[bits(1)]
    pub boot_on: u8,
}

/// FIFO_STATUS1 (0x25)
///
/// FIFO status register (read only)
/// Contains the number of unread samples stored in the FIFO buffer.
/// Value 0x00 means FIFO empty; 0x80 means FIFO full with 128 unread samples.
#[register(address = Reg::FifoStatus1, access_type = "Lps22df<B, T, OnState>")]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct FifoStatus1 {
    #[bits(8)]
    pub fss: u8,
}

/// FIFO_STATUS2 (0x26)
///
/// FIFO status register (read only)
#[register(address = Reg::FifoStatus2, access_type = "Lps22df<B, T, OnState>")]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct FifoStatus2 {
    #[bits(5, access = RO)]
    pub not_used_01: u8,
    /// FIFO full interrupt active (1 bit)
    /// (0: FIFO not full; 1: FIFO full)
    #[bits(1)]
    pub fifo_full_ia: u8,
    /// FIFO overrun interrupt active (1 bit)
    /// (0: no overrun; 1: FIFO slot overwritten)
    #[bits(1)]
    pub fifo_ovr_ia: u8,
    /// FIFO watermark interrupt active (1 bit)
    /// (0: FIFO below watermark; 1: FIFO equal or above watermark)
    #[bits(1)]
    pub fifo_wtm_ia: u8,
}

/// STATUS (0x27)
///
/// Status register (read only)
#[register(address = Reg::Status, access_type = "Lps22df<B, T, OnState>")]
#[cfg_attr(feature = "bit_order_msb", bitfield(u8, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u8, order = Lsb))]
pub struct Status {
    /// Pressure data available (1 bit)
    /// (0: new data not available; 1: new pressure data generated)
    #[bits(1)]
    pub p_da: u8,
    /// Temperature data available (1 bit)
    /// (0: new data not available; 1: new temperature data generated)
    #[bits(1)]
    pub t_da: u8,
    #[bits(2, access = RO)]
    pub not_used_01: u8,
    /// Pressure data overrun (1 bit)
    /// (0: no overrun; 1: new data overwritten previous data)
    #[bits(1)]
    pub p_or: u8,
    /// Temperature data overrun (1 bit)
    /// (0: no overrun; 1: new data overwritten previous data)
    #[bits(1)]
    pub t_or: u8,
    #[bits(2, access = RO)]
    pub not_used_02: u8,
}

/// PRESS_OUT_XL - PRESS_OUT_L - PRESS_OUT_H (0x28 - 0x2A)
///
/// The pressure output value is a 24-bit data that contains the measured pressure. It is composed of
/// PRESS_OUT_H (2Ah), PRESS_OUT_L (29h) and PRESS_OUT_XL (28h). The value is expressed as two's
/// complement.
/// The output pressure register PRESS_OUT is provided as the difference between the measured pressure and the
/// content of the register RPDS (1Ah, 1Bh).
#[register(address = Reg::PressOutXl, access_type = "Lps22df<B, T, OnState>")]
#[cfg_attr(feature = "bit_order_msb", bitfield(u32, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u32, order = Lsb))]
pub struct PressOut {
    #[offset_before(8)]
    #[bits(32)]
    pub pressure: i32,
}

/// TEMP_OUT_L - TEMP_OUT_H (0x2B - 0x2C)
///
/// The temperature output value is 16-bit data that contains the measured temperature.
/// The value is expressed as two’s complement.
/// This register contains the temperature value and the resolution is: 1LSB = 0.01°C
#[register(address = Reg::TempOutL, access_type = "Lps22df<B, T, OnState>", override_type = u16)]
#[cfg_attr(feature = "bit_order_msb", bitfield(u16, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u16, order = Lsb))]
pub struct TempOut {
    #[bits(16)]
    pub temp_c: i16,
}

/// FIFO_DATA_OUT_PRESS_XL - FIFO_DATA_OUT_PRESS_L - FIFO_DATA_OUT_PRESS_H (0x78 - 0x7A)
///
/// The temperature output value is 16-bit data that contains the measured temperature.
/// The value is expressed as two’s complement.
/// This register contains the temperature value and the resolution is: 1LSB = 0.01°C
#[register(address = Reg::FifoDataOutPressXl, access_type = "Lps22df<B, T, OnState>")]
#[cfg_attr(feature = "bit_order_msb", bitfield(u32, order = Msb))]
#[cfg_attr(not(feature = "bit_order_msb"), bitfield(u32, order = Lsb))]
pub struct FifoDataOutPress {
    #[offset_before(8)]
    #[bits(32)]
    pub pressure: i32,
}

/// Interface selection options for the device communication interface.
#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Default, TryFrom)]
#[try_from(repr)]
pub enum Interface {
    /// Hardware selects interface (default)
    #[default]
    SelByHw = 0x00,
    /// SPI 4-wire interface
    Spi4w = 0x02,
    /// SPI 3-wire interface
    Spi3w = 0x03,
    /// Interrupt pin enabled on MIPI I3C interface
    IntPinOnI3c = 0x04,
}

/// Filter configuration for antispike filters on I3C bus.
#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Default, TryFrom)]
#[try_from(repr)]
pub enum Filter {
    /// Antispike filter managed automatically by protocol (default)
    #[default]
    FilterAuto = 0x00,
    /// Antispike filter always enabled
    FilterAlwaysOn = 0x01,
}

/// I3C In-Band Interrupt (IBI) bus available time selection.
#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Default, TryFrom)]
#[try_from(repr)]
pub enum I3cIbiTime {
    /// Bus available time 50 μs (default)
    #[default]
    Ibi50us = 0x0,
    /// Bus available time 2 μs
    Ibi2us = 0x1,
    /// Bus available time 1 ms
    Ibi1ms = 0x2,
    /// Bus available time 25 ms
    Ibi25ms = 0x3,
}

/// Device initialization states.
#[repr(u8)]
#[derive(Clone, Copy, PartialEq, TryFrom)]
#[try_from(repr)]
pub enum Init {
    /// Driver ready state
    DrvRdy = 0x00,
    /// Boot phase running
    Boot = 0x01,
    /// Reset state
    Reset = 0x02,
}

/// Output Data Rate (ODR) settings for pressure and temperature data.
#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Default, TryFrom)]
#[try_from(repr)]
pub enum Odr {
    /// One-shot mode (default)
    #[default]
    OneShot = 0x00,
    /// 1 Hz output data rate
    _1hz = 0x01,
    /// 4 Hz output data rate
    _4hz = 0x02,
    /// 10 Hz output data rate
    _10hz = 0x03,
    /// 25 Hz output data rate
    _25hz = 0x04,
    /// 50 Hz output data rate
    _50hz = 0x05,
    /// 75 Hz output data rate
    _75hz = 0x06,
    /// 100 Hz output data rate
    _100hz = 0x07,
    /// 200 Hz output data rate
    _200hz = 0x08,
}

/// Averaging settings for pressure and temperature measurements.
#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Default, TryFrom)]
#[try_from(repr)]
pub enum Avg {
    /// Average over 4 samples (default)
    #[default]
    _4 = 0,
    /// Average over 8 samples
    _8 = 1,
    /// Average over 16 samples
    _16 = 2,
    /// Average over 32 samples
    _32 = 3,
    /// Average over 64 samples
    _64 = 4,
    /// Average over 128 samples
    _128 = 5,
    /// Average over 256 samples
    _256 = 6,
    /// Average over 512 samples
    _512 = 7,
}

/// Low-pass filter configuration options.
#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Default, TryFrom)]
#[try_from(repr)]
pub enum LowPassFilter {
    /// Low-pass filter disabled (default)
    #[default]
    Disable = 0,
    /// Low-pass filter cutoff frequency at ODR/4
    OdrDiv4 = 1,
    /// Low-pass filter cutoff frequency at ODR/9
    OdrDiv9 = 3,
}

/// FIFO operating modes.
#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Default, TryFrom)]
#[try_from(repr)]
pub enum Operation {
    /// Bypass mode (FIFO disabled)
    #[default]
    Bypass = 0,
    /// FIFO mode (store data until full)
    Fifo = 1,
    /// Continuous (dynamic-stream) mode
    Stream = 2,
    /// Continuous (dynamic-stream) to FIFO mode
    StreamToFifo = 7,
    /// Bypass to continuous (dynamic-stream) mode
    BypassToStream = 6,
    /// Bypass to FIFO mode
    BypassToFifo = 5,
}

/// Reference application modes for pressure offset and interrupt generation.
#[repr(u8)]
#[derive(Clone, Copy, PartialEq, TryFrom)]
#[try_from(repr)]
pub enum ApplyRef {
    /// Apply reference to output and interrupt generation
    OutAndInterrupt = 0,
    /// Apply reference only to interrupt generation
    OnlyInterrupt = 1,
    /// Reset reference registers
    RstRefs = 2,
}

/// Represents the FIFO event types.
///
/// # Variants
///
/// - `FifoEvWtm`: FIFO watermark event.
/// - `FifoEvFull`: FIFO full event.
///
/// # Description
///
/// This enum is used to specify the FIFO event type, allowing for watermark or full event configurations.
#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Default, TryFrom)]
#[try_from(repr)]
pub enum FifoEvent {
    #[default]
    Wtm = 0x0,
    Full = 0x1,
}

#[derive(Default)]
pub struct BusMode {
    pub interface: Interface,
    pub filter: Filter,
    pub i3c_ibi_time: I3cIbiTime,
}

pub struct Stat {
    pub sw_reset: u8,
    pub boot: u8,
    pub drdy_pres: u8,
    pub drdy_temp: u8,
    pub ovr_pres: u8,
    pub ovr_temp: u8,
    pub end_meas: u8,
    pub ref_done: u8,
}

pub struct PinConf {
    pub int_push_pull: u8,
    pub int_pull_down: u8,
    pub sdo_pull_up: u8,
    pub sda_pull_up: u8,
    pub cs_pull_up: u8,
}

pub struct AllSources {
    pub drdy_pres: u8,
    pub drdy_temp: u8,
    pub over_pres: u8,
    pub under_pres: u8,
    pub thrsld_pres: u8,
    pub fifo_full: u8,
    pub fifo_ovr: u8,
    pub fifo_th: u8,
}

#[derive(Default)]
pub struct Md {
    pub odr: Odr,
    pub avg: Avg,
    pub lpf: LowPassFilter,
}

#[derive(Copy, Clone)]
pub struct FifoData {
    pub hpa: f32,
    pub raw: i32,
}

pub struct IntMode {
    pub int_latched: u8,
    pub active_low: u8,
    pub drdy_latched: u8,
}

#[derive(Default)]
pub struct PinIntRoute {
    pub drdy_pres: u8,
    pub fifo_th: u8,
    pub fifo_ovr: u8,
    pub fifo_full: u8,
}

pub struct IntThMd {
    pub threshold: u16,
    pub over_th: u8,
    pub under_th: u8,
}

pub struct RefMd {
    pub apply_ref: ApplyRef,
    pub get_ref: u8,
}

#[derive(Copy, Clone)]
pub struct Pressure {
    pub hpa: f32,
    pub raw: i32, // 32-bit signed-left aligned format
}
#[derive(Copy, Clone)]
pub struct Heat {
    pub deg_c: f32,
    pub raw: i16,
}
#[derive(Copy, Clone)]
pub struct Data {
    pub pressure: Pressure,
    pub heat: Heat,
}
