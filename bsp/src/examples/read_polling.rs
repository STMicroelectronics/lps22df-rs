use defmt::info;
use maybe_async::maybe_async;
use crate::*;

#[maybe_async]
pub async fn run<B, D, L>(bus: B, mut tx: L, mut delay: D, irq: ()) -> !
where
    B: BusOperation,
    D: DelayNs + Clone,
    L: embedded_io::Write
{
    use lps22df::prelude::*;
    use lps22df::*;

    info!("Configuring the sensor");
    let mut sensor = Lps22df::new_bus(bus, delay.clone());

    // boot time
    delay.delay_ms(5).await;

    // Check device ID
    let id = sensor.id_get().await.unwrap();
    info!("Device ID: {:x}", id);
    if id != ID {
        info!("Unexpected device ID: {:x}", id);
        writeln!(tx, "Unexpected device ID: {:x}", id).unwrap();
        loop {}
    }

    // Boot device
    sensor.init_set(Init::Boot).await.unwrap();

    // Reset device
    sensor.init_set(Init::Reset).await.unwrap();

    // Set BDU and IF_INC recommended for driver usage
    sensor.init_set(Init::DrvRdy).await.unwrap();

    // Select bus interface
    let bus_mode = BusMode {
        filter: Filter::FilterAuto,
        interface: Interface::SelByHw,
        i3c_ibi_time: I3cIbiTime::Ibi1ms,
    };
    sensor.bus_mode_set(&bus_mode).await.unwrap();

    // Set Output Data Rate
    let md = Md {
        odr: Odr::_4hz,
        avg: Avg::_16,
        lpf: LowPassFilter::OdrDiv4,
    };
    sensor.mode_set(&md).await.unwrap();

    // Read samples in polling mode (no interrupt)
    loop {
        let all_sources = sensor.all_sources_get().await.unwrap();

        if all_sources.drdy_pres == 1 || all_sources.drdy_temp == 1 {
            let data = sensor.data_get().await.unwrap();

            writeln!(
                tx,
                "pressure [hPa]: {:.2} temperature [degC]: {:.2}",
                data.pressure.hpa, data.heat.deg_c
            )
            .unwrap();
        }

        delay.delay_ms(1000).await;
    }
}
