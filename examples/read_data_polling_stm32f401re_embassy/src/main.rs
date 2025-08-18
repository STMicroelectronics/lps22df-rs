#![no_std]
#![no_main]

use core::fmt::Write;
use cortex_m::prelude::_embedded_hal_blocking_delay_DelayMs;
use embassy_executor::Spawner;
use embassy_stm32::bind_interrupts;
use embassy_stm32::dma::NoDma;
use embassy_stm32::i2c::{self, Config as I2cConfig, I2c};
use embassy_stm32::peripherals::{self, USART2};
use embassy_stm32::time::khz;
use embassy_stm32::usart::{
    BufferedInterruptHandler, Config as UsartConfig, DataBits, Parity, UartTx,
};
use embassy_time::Delay;
use heapless::String;
use {defmt_rtt as _, panic_probe as _};

use lps22df_rs::prelude::*;
use lps22df_rs::*;

#[defmt::panic_handler]
fn panic() -> ! {
    core::panic!("panic via `defmt::panic!`")
}

bind_interrupts!(struct Irqs {
    USART2 => BufferedInterruptHandler<USART2>;
    I2C1_EV => i2c::EventInterruptHandler<peripherals::I2C1>;
    I2C1_ER => i2c::ErrorInterruptHandler<peripherals::I2C1>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());

    let mut usart_config: UsartConfig = UsartConfig::default();
    usart_config.baudrate = 115200;
    usart_config.data_bits = DataBits::DataBits8;
    usart_config.parity = Parity::ParityNone;

    let mut tx: UartTx<_> = UartTx::new(p.USART2, p.PA2, NoDma, usart_config).unwrap();

    let i2c: I2c<_> = I2c::new(
        p.I2C1,
        p.PB8,
        p.PB9,
        Irqs,
        NoDma,
        NoDma,
        khz(100),
        I2cConfig::default(),
    );

    let mut delay = Delay;

    delay.delay_ms(5_u32);

    let mut msg: String<64> = String::new();

    let mut sensor = Lps22df::new_i2c(i2c, I2CAddress::I2cAddH, delay.clone());
    // Check device ID
    let id = sensor.id_get().unwrap();
    if id != ID {
        writeln!(&mut msg, "Device ID mismatch: {:#02x}", id).unwrap();

        let _ = tx.blocking_write(msg.as_bytes());
        msg.clear();
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
                &mut msg,
                "pressure [hPa]: {:.2} temperature [degC]: {:.2}",
                data.pressure.hpa, data.heat.deg_c
            )
            .unwrap();
            let _ = tx.blocking_write(msg.as_bytes());
            msg.clear();
        }

        delay.delay_ms(1000_u32);
    }
}
