#![no_std]
#![no_main]
#![deny(unsafe_code)]

use core::fmt::Write;

use cortex_m_rt::entry;

use panic_halt as _;

use lps22df_rs::prelude::*;
use lps22df_rs::*;
use stm32f4xx_hal::{
    i2c::{DutyCycle, I2c, Mode},
    pac,
    prelude::*,
    serial::Config,
};

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();

    let rcc = dp.RCC.constrain();

    let clocks = rcc.cfgr.use_hse(8.MHz()).sysclk(48.MHz()).freeze();
    let tim1 = dp.TIM1.delay_us(&clocks);

    let mut delay = cp.SYST.delay(&clocks);

    let gpiob = dp.GPIOB.split();
    let gpioa = dp.GPIOA.split();

    let scl = gpiob.pb8;
    let sda = gpiob.pb9;

    let i2c: I2c<pac::I2C1> = I2c::new(
        dp.I2C1,
        (scl, sda),
        Mode::Fast {
            frequency: 400.kHz(),
            duty_cycle: DutyCycle::Ratio2to1,
        },
        &clocks,
    );

    let tx_pin = gpioa.pa2.into_alternate();

    let mut tx = dp
        .USART2
        .tx(
            tx_pin,
            Config::default()
                .baudrate(115200.bps())
                .wordlength_8()
                .parity_none(),
            &clocks,
        )
        .unwrap();

    delay.delay_ms(5);

    let mut sensor = Lps22df::new_i2c(i2c, I2CAddress::I2cAddH, tim1);
    // Check device ID
    let id = sensor.id_get().unwrap();
    if id != ID {
        writeln!(tx, "Device ID mismatch: {:#02x}", id).unwrap();
        loop {}
    }
    // Boot device
    sensor.init_set(Init::Boot).unwrap();

    // Reset device
    sensor.init_set(Init::Reset).unwrap();

    // Set BDU and IF_INC recommended for driver usage
    sensor.init_set(Init::DrvRdy).unwrap();

    // Select bus interface
    let bus_mode = BusMode {
        filter: Filter::FilterAuto,
        interface: Interface::SelByHw,
        i3c_ibi_time: I3cIbiTime::Ibi1ms,
    };
    sensor.bus_mode_set(&bus_mode).unwrap();

    // Set Output Data Rate
    let md = Md {
        odr: Odr::_4hz,
        avg: Avg::_16,
        lpf: LowPassFilter::OdrDiv4,
    };
    sensor.mode_set(&md).unwrap();

    // Configure interrupt pins
    let mut int_route = sensor.pin_int_route_get().unwrap();
    int_route.drdy_pres = 0;
    sensor.pin_int_route_set(&int_route).unwrap();

    // Read samples in polling mode (no interrupt)
    loop {
        let all_sources = sensor.all_sources_get().unwrap();

        if all_sources.drdy_pres == 1 || all_sources.drdy_temp == 1 {
            let data = sensor.data_get().unwrap();

            writeln!(
                tx,
                "pressure [hPa]: {:.2} temperature [degC]: {:.2}",
                data.pressure.hpa, data.heat.deg_c
            )
            .unwrap();
        }

        delay.delay_ms(1000);
    }
}
