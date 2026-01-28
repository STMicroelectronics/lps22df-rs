use super::{
    BusOperation, DelayNs, I2c, RegisterOperation, SensorOperation, SevenBitAddress, SpiDevice,
    bisync, i2c, prelude::*, spi,
};

use core::fmt::Debug;
use core::marker::PhantomData;

/// Driver for LPS22DF sensor.
///
/// The struct takes a bus to write to the registers.
/// The bus is generalized over the BusOperation trait, allowing the use
/// of I2C or SPI protocols; this also allows the user to implement sharing
/// techniques to share the underlying bus.
#[bisync]
pub struct Lps22df<B, T, S>
where
    B: BusOperation,
    T: DelayNs,
    S: SensorState,
{
    pub bus: B,
    pub tim: T,
    _state: PhantomData<S>,
}

/// Driver errors.
#[bisync]
#[derive(Debug)]
pub enum Error<B> {
    Bus(B),          // Error at the bus level
    UnexpectedValue, // Unexpected value read from a register
}

#[bisync]
impl<B, T, S> Lps22df<B, T, S>
where
    B: BusOperation,
    T: DelayNs,
    S: SensorState,
{
    /// Constructor method based on general BusOperation implementation.
    pub fn new_bus(bus: B, tim: T) -> Self {
        Self {
            bus,
            tim,
            _state: PhantomData,
        }
    }
}

#[bisync]
impl<P, T> Lps22df<i2c::I2cBus<P>, T, OnState>
where
    P: I2c,
    T: DelayNs,
{
    /// Constructor method for using the I2C bus.
    pub fn new_i2c(i2c: P, address: I2CAddress, tim: T) -> Self {
        // Initialize the I2C bus with the COMPONENT address
        let bus = i2c::I2cBus::new(i2c, address as SevenBitAddress);
        Self {
            bus,
            tim,
            _state: PhantomData,
        }
    }
}

#[bisync]
impl<P, T> Lps22df<spi::SpiBus<P>, T, OnState>
where
    P: SpiDevice,
    T: DelayNs,
{
    /// Constructor method for using the SPI bus.
    pub fn new_spi(spi: P, tim: T) -> Self {
        // Initialize the SPI bus
        let bus = spi::SpiBus::new(spi);
        Self {
            bus,
            tim,
            _state: PhantomData,
        }
    }
}

#[bisync]
impl<B, T, S> SensorOperation for Lps22df<B, T, S>
where
    B: BusOperation,
    T: DelayNs,
    S: SensorState,
{
    type Error = Error<B::Error>;

    #[inline]
    async fn read_from_register(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Error<B::Error>> {
        self.bus
            .read_from_register(reg, buf)
            .await
            .map_err(Error::Bus)
    }

    #[inline]
    async fn write_to_register(&mut self, reg: u8, buf: &[u8]) -> Result<(), Error<B::Error>> {
        self.bus
            .write_to_register(reg, buf)
            .await
            .map_err(Error::Bus)
    }
}

#[bisync]
impl<B: BusOperation, T: DelayNs> Lps22df<B, T, OnState> {
    /// Get Device id: WHO_AM_I (0x0F)
    ///
    /// Device identification register (read only).
    /// Returns the fixed device ID value to verify communication.
    pub async fn id_get(&mut self) -> Result<u8, Error<B::Error>> {
        WhoAmI::read(self).await.map(|reg| reg.id())
    }
    /// Configures the bus operating mode.
    ///
    /// Configures the serial interface mode and enables/disables I2C, I3C, SPI modes and related filters.
    pub async fn bus_mode_set(&mut self, val: &BusMode) -> Result<(), Error<B::Error>> {
        let mut if_ctrl = IfCtrl::read(self).await?;
        if_ctrl.set_i2c_i3c_dis(((val.interface as u8) & 0x02) >> 1);
        if_ctrl.set_int_en_i3c(((val.interface as u8) & 0x04) >> 2);
        if_ctrl.set_sim((val.interface as u8) & 0x01);
        if_ctrl.write(self).await?;

        let mut i3c_if_ctrl = I3cIfCtrl::read(self).await?;
        i3c_if_ctrl.set_asf_on((val.filter as u8) & 0x01);
        i3c_if_ctrl.set_i3c_bus_avb_sel((val.i3c_ibi_time as u8) & 0x03);
        i3c_if_ctrl.write(self).await?;

        Ok(())
    }
    /// Retrive the bus operating mode.
    ///
    /// Reads interface control and I3C interface control registers to retrieve current bus operating mode,
    /// filter configuration, and I3C IBI timing.
    pub async fn bus_mode_get(&mut self) -> Result<BusMode, Error<B::Error>> {
        let if_ctrl = IfCtrl::read(self).await?;
        let i3c_if_ctrl = I3cIfCtrl::read(self).await?;

        let interface = (if_ctrl.int_en_i3c() << 2) | (if_ctrl.i2c_i3c_dis() << 1) | if_ctrl.sim();
        let interface = Interface::try_from(interface).unwrap_or(Interface::SelByHw);

        let filter = Filter::try_from(i3c_if_ctrl.asf_on()).unwrap_or_default();

        let i3c_ibi_time = i3c_if_ctrl.i3c_bus_avb_sel();
        let i3c_ibi_time = I3cIbiTime::try_from(i3c_ibi_time).unwrap_or_default();

        Ok(BusMode {
            interface,
            filter,
            i3c_ibi_time,
        })
    }

    /// Device initialization and reset control.
    ///
    /// This function manages the device initialization states including boot, reset, and driver ready modes.
    /// - `Init::Boot`: Triggers a reboot of internal memory and waits for boot completion indicated by INT_SOURCE register.
    /// - `Init::Reset`: Performs a software reset and waits for reset completion indicated by status register.
    /// - `Init::DrvRdy`: Enables block data update and automatic register address increment for multi-byte access.
    pub async fn init_set(&mut self, val: Init) -> Result<(), Error<B::Error>> {
        // subsequent reads are required for a correct reset
        let mut reg: [u8; 2] = [0, 0];
        self.read_from_register(Reg::CtrlReg2 as u8, &mut reg)
            .await?;
        let mut ctrl_reg2 = CtrlReg2::from_bits(reg[0]);
        let mut ctrl_reg3 = CtrlReg3::from_bits(reg[1]);
        let mut cnt: u8 = 0;

        match val {
            Init::Boot => {
                ctrl_reg2.set_boot(1);
                ctrl_reg2.write(self).await?;

                loop {
                    let int_src = IntSource::read(self).await?;

                    if int_src.boot_on() == 0 {
                        break;
                    }

                    self.tim.delay_ms(10).await;

                    if cnt >= 5 {
                        return Err(Error::UnexpectedValue);
                    }

                    cnt += 1;
                }
            }
            Init::Reset => {
                ctrl_reg2.set_swreset(1);
                self.write_to_register(Reg::CtrlReg2 as u8, &[ctrl_reg2.into()])
                    .await?;

                loop {
                    let status = self.status_get().await?;
                    if status.sw_reset == 0 {
                        break;
                    }

                    self.tim.delay_ms(10).await;

                    if cnt >= 5 {
                        return Err(Error::UnexpectedValue);
                    }

                    cnt += 1;
                }
            }
            Init::DrvRdy => {
                ctrl_reg2.set_bdu(1);
                ctrl_reg3.set_if_add_inc(1);
                // subsequent writes are required for a correct reset
                reg[0] = ctrl_reg2.into();
                reg[1] = ctrl_reg3.into();
                self.write_to_register(Reg::CtrlReg2 as u8, &reg).await?;
            }
        }

        Ok(())
    }
    /// Device status retrieval.
    ///
    /// Reads multiple status-related registers to provide comprehensive device status information:
    /// - Software reset completion status.
    /// - Boot phase status.
    /// - Pressure and temperature data ready flags.
    /// - Pressure and temperature data overrun flags.
    /// - One-shot measurement completion status.
    /// - AUTOZERO reference completion status.
    pub async fn status_get(&mut self) -> Result<Stat, Error<B::Error>> {
        let ctrl_reg2 = CtrlReg2::read(self).await?;
        let int_source = IntSource::read(self).await?;
        let status = Status::read(self).await?;
        let interrupt_cfg = InterruptCfg::read(self).await?;

        Ok(Stat {
            sw_reset: ctrl_reg2.swreset(),
            boot: int_source.boot_on(),
            drdy_pres: status.p_da(),
            drdy_temp: status.t_da(),
            ovr_pres: status.p_or(),
            ovr_temp: status.t_or(),
            end_meas: !ctrl_reg2.oneshot(),
            ref_done: !interrupt_cfg.autozero(),
        })
    }
    /// Electrical pin configuration setter.
    ///
    /// Configures the electrical characteristics of device pins including pull-up/down resistors and push-pull/open-drain modes.
    /// - Configures IF_CTRL register for pull-up/down on INT, SDA, SDO, and CS pins.
    /// - Configures CTRL_REG3 register for interrupt pin output type.
    pub async fn pin_conf_set(&mut self, val: &PinConf) -> Result<(), Error<B::Error>> {
        // Read IF_CTRL register
        let mut if_ctrl = IfCtrl::read(self).await?;

        // Configure IF_CTRL register fields
        if_ctrl.set_int_pd_dis(!val.int_pull_down);
        if_ctrl.set_sdo_pu_en(val.sdo_pull_up);
        if_ctrl.set_sda_pu_en(val.sda_pull_up);
        if_ctrl.set_cs_pu_dis(!val.cs_pull_up);

        // Write back to IF_CTRL register
        if_ctrl.write(self).await?;

        // Read CTRL_REG3 register
        let mut ctrl_reg3 = CtrlReg3::read(self).await?;

        // Configure CTRL_REG3 register fields
        ctrl_reg3.set_pp_od(!val.int_push_pull);

        // Write back to CTRL_REG3 register
        ctrl_reg3.write(self).await?;

        Ok(())
    }
    /// Get Electrical pin configuration.
    ///
    /// Reads IF_CTRL and CTRL_REG3 registers to retrieve current electrical pin configuration.
    pub async fn pin_conf_get(&mut self) -> Result<PinConf, Error<B::Error>> {
        let if_ctrl = IfCtrl::read(self).await?;
        let ctrl_reg3 = CtrlReg3::read(self).await?;

        let val = PinConf {
            sda_pull_up: if_ctrl.sda_pu_en(),
            cs_pull_up: !if_ctrl.cs_pu_dis(),
            int_pull_down: !if_ctrl.int_pd_dis(),
            sdo_pull_up: if_ctrl.sdo_pu_en(),
            int_push_pull: !ctrl_reg3.pp_od(),
        };

        Ok(val)
    }
    /// Retrieve status of all interrupt sources.
    ///
    /// Reads STATUS, INT_SOURCE, and FIFO_STATUS2 registers to provide a comprehensive snapshot of all interrupt flags,
    /// including data-ready, threshold, FIFO full, FIFO overrun, and watermark interrupts.
    pub async fn all_sources_get(&mut self) -> Result<AllSources, Error<B::Error>> {
        let status = Status::read(self).await?;
        let int_source = IntSource::read(self).await?;
        let fifo_status2 = FifoStatus2::read(self).await?;

        let val = AllSources {
            drdy_pres: status.p_da(),
            drdy_temp: status.t_da(),
            over_pres: int_source.ph(),
            under_pres: int_source.pl(),
            thrsld_pres: int_source.ia(),
            fifo_full: fifo_status2.fifo_full_ia(),
            fifo_ovr: fifo_status2.fifo_ovr_ia(),
            fifo_th: fifo_status2.fifo_wtm_ia(),
        };

        Ok(val)
    }
    /// Select conversion parameters.
    ///
    /// Sets output data rate, averaging, and low-pass filter configuration registers.
    pub async fn mode_set(&mut self, val: &Md) -> Result<(), Error<B::Error>> {
        let mut ctrl_reg1 = CtrlReg1::read(self).await?;
        let mut ctrl_reg2 = CtrlReg2::read(self).await?;

        ctrl_reg1.set_odr(val.odr as u8);
        ctrl_reg1.set_avg(val.avg as u8);
        ctrl_reg2.set_en_lpfp(val.lpf as u8 & 0x01);
        ctrl_reg2.set_lfpf_cfg((val.lpf as u8 & 0x02) >> 1);

        ctrl_reg1.write(self).await?;
        ctrl_reg2.write(self).await?;

        Ok(())
    }
    /// Get sensor conversion parameters.
    ///
    /// Reads output data rate, averaging, and low-pass filter configuration registers.
    pub async fn mode_get(&mut self) -> Result<Md, Error<B::Error>> {
        let ctrl_reg1 = CtrlReg1::read(self).await?;
        let ctrl_reg2 = CtrlReg2::read(self).await?;

        let odr = Odr::try_from(ctrl_reg1.odr()).unwrap_or_default();
        let avg = Avg::try_from(ctrl_reg1.avg()).unwrap_or_default();
        let lpf = (ctrl_reg2.lfpf_cfg() << 2) | ctrl_reg2.en_lpfp();
        let lpf = LowPassFilter::try_from(lpf).unwrap_or_default();

        Ok(Md { odr, avg, lpf })
    }
    /// Software trigger for One-Shot mode.
    ///
    /// Triggers a single pressure and temperature measurement if the device is in one-shot mode.
    pub async fn trigger_sw(&mut self, md: &Md) -> Result<(), Error<B::Error>> {
        if md.odr == Odr::OneShot {
            let mut ctrl_reg2 = CtrlReg2::read(self).await?;
            ctrl_reg2.set_oneshot(1);
            ctrl_reg2.write(self).await?;
        }
        Ok(())
    }
    /// Sensor data retrieval.
    ///
    /// Reads pressure and temperature output registers and converts raw data to physical units.
    pub async fn data_get(&mut self) -> Result<Data, Error<B::Error>> {
        // Pressure conversion
        let raw_pressure = PressOut::read(self).await?.pressure();
        let pressure_hpa = from_lsb_to_hpa(raw_pressure);

        // Temperature conversion
        let raw_heat: i16 = TempOut::read(self).await?.temp_c();
        let heat_deg_c = from_lsb_to_celsius(raw_heat);

        // Construct Data
        let data = Data {
            pressure: Pressure {
                hpa: pressure_hpa,
                raw: raw_pressure,
            },
            heat: Heat {
                deg_c: heat_deg_c,
                raw: raw_heat,
            },
        };

        Ok(data)
    }

    /// Pressure output raw value getter.
    ///
    /// Reads the 24-bit raw pressure output registers and returns the value left-aligned in a 32-bit integer.
    pub async fn pressure_raw_get(&mut self) -> Result<u32, Error<B::Error>> {
        Ok(PressOut::read(self).await?.into_bits())
    }
    /// Temperature output raw value getter.
    ///
    /// Reads the 16-bit raw temperature output registers.
    pub async fn temperature_raw_get(&mut self) -> Result<i16, Error<B::Error>> {
        TempOut::read(self).await.map(|reg| reg.temp_c())
    }
    /// Set FIFO operation mode and watermark level.
    ///
    /// Configures the FIFO_CTRL (0x14) register to select the FIFO operating mode, and
    /// enable triggered FIFO modes.
    pub async fn fifo_mode_set(&mut self, op: Operation) -> Result<(), Error<B::Error>> {
        let mut fifo_ctrl = FifoCtrl::read(self).await?;

        fifo_ctrl.set_f_mode((op as u8) & 0x03);
        fifo_ctrl.set_trig_modes(((op as u8) & 0x04) >> 2);

        fifo_ctrl.write(self).await
    }
    /// Get FIFO operation mode and watermark level.
    ///
    /// Reads FIFO_CTRL (0x14) and FIFO_WTM (0x15) registers to retrieve the current FIFO operating mode,
    /// triggered mode status, and watermark level.
    pub async fn fifo_mode_get(&mut self) -> Result<Operation, Error<B::Error>> {
        let fifo_ctrl = FifoCtrl::read(self).await?;

        let operation = (fifo_ctrl.trig_modes() << 2) | fifo_ctrl.f_mode();
        Ok(Operation::try_from(operation).unwrap_or_default())
    }

    /// Sets the FIFO watermark level.
    ///
    /// The FIFO watermark is a programmable threshold that determines when a FIFO threshold interrupt is generated.
    /// When the number of unread samples in the FIFO buffer reaches or exceeds this level, the device can trigger an interrupt
    /// (if enabled via the INT_F_WTM bit in CTRL_REG4).
    ///
    /// Registers used:
    /// - Writes to the `FIFO_WTM` (0x15) register, WTM[6:0] field.
    ///
    /// # Parameters
    /// - `val`: Watermark threshold value (0..=127). Each unit corresponds to one FIFO sample (1 sample = 7 bytes: 1 TAG + 6 DATA).
    ///
    /// # Returns
    /// - `Ok(())` on success.
    /// - `Err`: If the value is out of range or a bus error occurs.
    ///
    /// # Panics
    /// - Panics if `val >= 128` (assertion).
    ///
    /// # Example
    /// ```rust
    /// sensor.fifo_watermark_set(32)?;
    /// ```
    pub async fn fifo_watermark_set(&mut self, watermark: u8) -> Result<(), Error<B::Error>> {
        assert!(watermark < 128);

        let mut fifo_wtm = FifoWtm::read(self).await?;
        fifo_wtm.set_wtm(watermark);
        fifo_wtm.write(self).await
    }

    /// Retrieves the current FIFO watermark threshold value.
    ///
    /// This function reads the FIFO_WTM register and returns the current watermark threshold (WTM[6:0]).
    /// The watermark determines when the FIFO threshold interrupt is triggered (if enabled).
    ///
    /// Registers used:
    /// - Reads from the `FIFO_WTM` (0x15) register, FTH[6:0] field.
    ///
    /// # Returns
    /// - `Ok(u8)`: The current FIFO watermark threshold (0..=127).
    /// - `Err`: If a bus or register access error occurs.
    ///
    /// # Example
    /// ```rust
    /// let wtm = sensor.fifo_watermark_get()?;
    /// ```
    pub async fn fifo_watermark_get(&mut self) -> Result<u8, Error<B::Error>> {
        let fifo_wtm = FifoWtm::read(self).await?;
        Ok(fifo_wtm.wtm())
    }

    /// Enables or disables the FIFO stop-on-watermark feature.
    ///
    /// When enabled, the FIFO buffer stops collecting new data once the number of unread samples reaches the watermark threshold.
    /// This is useful for applications that require precise control over the number of samples collected in the FIFO.
    /// When disabled, the FIFO continues to collect data, potentially overwriting the oldest samples (depending on FIFO mode).
    ///
    /// # Parameters
    /// - `fth`: `FifoEvent` enum value indicating whether to enable or disable the stop-on-watermark feature.
    ///
    /// # Returns
    /// - `Ok(())` on success.
    /// - `Err`: If a bus or register access error occurs.
    ///
    /// # Example
    /// ```rust
    /// sensor.fifo_stop_on_wtm_set(FifoEvent::Enable)?;
    /// ```
    pub async fn fifo_stop_on_wtm_set(&mut self, fth: FifoEvent) -> Result<(), Error<B::Error>> {
        let mut fifo_ctrl = FifoCtrl::read(self).await?;
        fifo_ctrl.set_stop_on_wtm(fth as u8);
        fifo_ctrl.write(self).await
    }

    /// Reads the current status of the FIFO stop-on-watermark feature.
    ///
    /// This function checks whether the FIFO is configured to stop collecting data when the watermark threshold is reached.
    /// Returns the current setting as a `FifoEvent` enum.
    ///
    /// Registers used:
    /// - Reads from the `FIFO_CTRL` (0x14) register, STOP_ON_FTH bit (bit 3)
    /// # Returns
    /// - `Ok(FifoEvent)`: Current stop-on-watermark configuration.
    /// - `Err`: If a bus or register access error occurs.
    ///
    /// # Example
    /// ```rust
    /// let stop_on_wtm = sensor.fifo_stop_on_wtm_get()?;
    /// ```
    pub async fn fifo_stop_on_wtm_get(&mut self) -> Result<FifoEvent, Error<B::Error>> {
        let ctrl = FifoCtrl::read(self).await?;
        let evt = FifoEvent::try_from(ctrl.stop_on_wtm()).unwrap_or_default();
        Ok(evt)
    }

    /// Get the current number of unread samples stored in FIFO.
    ///
    /// Reads FIFO_STATUS1 (0x25) register which contains the FIFO stored data level (FSS[7:0]).
    pub async fn fifo_level_get(&mut self) -> Result<u8, Error<B::Error>> {
        let val: u8 = FifoStatus1::read(self).await?.fss();

        Ok(val)
    }
    /// Retrieve multiple pressure samples from FIFO buffer.
    ///
    /// Reads `samp` number of pressure samples from FIFO_DATA_OUT_PRESS_XL/H/L registers (0x78-0x7A).
    /// Each sample is 24-bit pressure data, converted to raw and hPa values.
    pub async fn fifo_data_get(&mut self, samp: u8) -> Result<[FifoData; 32], Error<B::Error>> {
        let mut data: [FifoData; 32] = [FifoData { raw: 0, hpa: 0.0 }; 32];
        for item in data.iter_mut().take(samp as usize) {
            let raw: i32 = FifoDataOutPress::read(self).await?.pressure();
            let hpa = from_lsb_to_hpa(raw);

            *item = FifoData { raw, hpa };
        }

        Ok(data)
    }
    /// Configure interrupt pins hardware signal behavior.
    ///
    /// Sets interrupt polarity, data-ready pulse mode, and interrupt latch mode by configuring CTRL_REG3 (0x12),
    /// CTRL_REG4 (0x13), and INTERRUPT_CFG (0x0B) registers.
    pub async fn interrupt_mode_set(&mut self, val: &IntMode) -> Result<(), Error<B::Error>> {
        let mut ctrl_reg3 = CtrlReg3::read(self).await?;
        let mut ctrl_reg4 = CtrlReg4::read(self).await?;
        ctrl_reg3.set_int_h_l(val.active_low);
        ctrl_reg4.set_drdy_pls(!val.drdy_latched);
        ctrl_reg3.write(self).await?;
        ctrl_reg4.write(self).await?;

        let mut interrupt_cfg = InterruptCfg::read(self).await?;
        interrupt_cfg.set_lir(val.int_latched);
        interrupt_cfg.write(self).await?;

        Ok(())
    }
    /// Retrieve interrupt pins hardware signal configuration.
    ///
    /// Reads CTRL_REG3 (0x12), CTRL_REG4 (0x13), and INTERRUPT_CFG (0x0B) registers to obtain
    /// interrupt polarity, data-ready pulse mode, and interrupt latch mode.
    pub async fn interrupt_mode_get(&mut self) -> Result<IntMode, Error<B::Error>> {
        let interrupt_cfg = InterruptCfg::read(self).await?;
        let ctrl_reg3 = CtrlReg3::read(self).await?;
        let ctrl_reg4 = CtrlReg4::read(self).await?;

        let active_low = ctrl_reg3.int_h_l();
        let drdy_latched = !ctrl_reg4.drdy_pls();
        let int_latched = interrupt_cfg.lir();

        Ok(IntMode {
            active_low,
            drdy_latched,
            int_latched,
        })
    }
    /// Configure routing of interrupt signals on INT1 pin.
    ///
    /// Sets which interrupt signals are routed to the INT1 pin by configuring CTRL_REG4 (0x13) register.
    pub async fn pin_int_route_set(&mut self, val: &PinIntRoute) -> Result<(), Error<B::Error>> {
        let mut ctrl_reg4 = CtrlReg4::read(self).await?;
        ctrl_reg4.set_drdy(val.drdy_pres);
        ctrl_reg4.set_int_f_wtm(val.fifo_th);
        ctrl_reg4.set_int_f_ovr(val.fifo_ovr);
        ctrl_reg4.set_int_f_full(val.fifo_full);
        ctrl_reg4.write(self).await?;

        Ok(())
    }
    /// Retrieve routing configuration of interrupt signals on INT1 pin.
    ///
    /// Reads CTRL_REG4 (0x13) register to determine which interrupt signals are routed to INT1 pin.
    pub async fn pin_int_route_get(&mut self) -> Result<PinIntRoute, Error<B::Error>> {
        let reg = CtrlReg4::read(self).await?;

        let val = PinIntRoute {
            drdy_pres: reg.drdy(),
            fifo_th: reg.int_f_wtm(),
            fifo_ovr: reg.int_f_ovr(),
            fifo_full: reg.int_f_full(),
        };

        Ok(val)
    }
    /// Configuration of Wake-up and Wake-up to Sleep.
    ///
    /// This function configures the interrupt generation based on pressure threshold crossing events.
    /// It sets the interrupt enable bits for pressure high and low threshold events, programs the threshold
    /// value, and enables the interrupt signal on the INT pin.
    pub async fn int_on_threshold_mode_set(
        &mut self,
        val: &IntThMd,
    ) -> Result<(), Error<B::Error>> {
        let mut interrupt_cfg = InterruptCfg::read(self).await?;
        let mut ths_p = ThsP::read(self).await?;

        interrupt_cfg.set_phe(val.over_th);
        interrupt_cfg.set_ple(val.under_th);
        ths_p.set_ths(val.threshold);

        interrupt_cfg.write(self).await?;
        ths_p.write(self).await?;

        let mut ctrl_reg4 = CtrlReg4::read(self).await?;
        ctrl_reg4.set_int_en(1);
        ctrl_reg4.write(self).await?;

        Ok(())
    }
    /// Retrieve configuration of Wake-up and Wake-up to Sleep.
    ///
    /// Reads the interrupt configuration and threshold registers to obtain the current settings for
    /// pressure threshold interrupt generation.
    pub async fn int_on_threshold_mode_get(&mut self) -> Result<IntThMd, Error<B::Error>> {
        let interrupt_cfg = InterruptCfg::read(self).await?;
        let ths_p = ThsP::read(self).await?;

        let over_th = interrupt_cfg.phe();
        let under_th = interrupt_cfg.ple();
        let threshold: u16 = ths_p.ths();
        Ok(IntThMd {
            over_th,
            under_th,
            threshold,
        })
    }
    /// Configure reference mode for pressure offset and interrupt generation.
    ///
    /// This function sets the AUTOZERO and AUTOREFP modes, and controls resetting of reference registers.
    /// AUTOZERO mode sets the current pressure as reference and outputs differential pressure.
    /// AUTOREFP mode uses the reference only for interrupt generation without changing output pressure.
    pub async fn reference_mode_set(&mut self, val: &RefMd) -> Result<(), Error<B::Error>> {
        let mut interrupt_cfg = InterruptCfg::read(self).await?;

        interrupt_cfg.set_autozero(val.get_ref);
        interrupt_cfg.set_autorefp((val.apply_ref as u8) & 0x01);
        interrupt_cfg.set_reset_az(((val.apply_ref as u8) & 0x02) >> 1);
        interrupt_cfg.set_reset_arp(((val.apply_ref as u8) & 0x02) >> 1);

        interrupt_cfg.write(self).await?;

        Ok(())
    }
    /// Retrieve reference mode configuration.
    ///
    /// Reads the INTERRUPT_CFG register to obtain the current AUTOZERO and AUTOREFP settings,
    /// as well as the reset state of reference registers.
    pub async fn reference_mode_get(&mut self) -> Result<RefMd, Error<B::Error>> {
        let interrupt_cfg = InterruptCfg::read(self).await?;
        let apply_ref = (interrupt_cfg.reset_az() << 1) | interrupt_cfg.autorefp();
        let apply_ref = ApplyRef::try_from(apply_ref).unwrap_or(ApplyRef::RstRefs);
        let get_ref = interrupt_cfg.autozero();

        Ok(RefMd { apply_ref, get_ref })
    }
    /// Set one-point calibration (OPC) offset value.
    ///
    /// Writes a 16-bit signed offset value to the pressure offset registers (RPDS_L and RPDS_H).
    /// This offset is added to compensated pressure data to calibrate the sensor after soldering.
    pub async fn opc_set(&mut self, val: i16) -> Result<(), Error<B::Error>> {
        Rpds::new().with_offset(val).write(self).await
    }
    /// Retrieve one-point calibration (OPC) offset value.
    ///
    /// Reads the 16-bit signed offset value from the pressure offset registers (RPDS_L and RPDS_H).
    pub async fn opc_get(&mut self) -> Result<i16, Error<B::Error>> {
        Rpds::read(self).await.map(|reg| reg.offset())
    }
}

#[bisync]
pub fn from_lsb_to_hpa(lsb: i32) -> f32 {
    (lsb as f32) / 1048576.0
}

#[bisync]
pub fn from_lsb_to_celsius(lsb: i16) -> f32 {
    (lsb as f32) / 100.0
}

/// I2C device addresses based on SDO/SA0 pin state.
#[repr(u8)]
#[derive(Clone, Copy, PartialEq)]
#[bisync]
pub enum I2CAddress {
    /// I2C address when SA0 pin is low (0)
    I2cAddL = 0x5C,
    /// I2C address when SA0 pin is high (1)
    I2cAddH = 0x5D,
}

#[bisync]
pub const ID: u8 = 0xB4;
