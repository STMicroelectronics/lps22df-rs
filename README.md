# lps22df-rs
[![Crates.io][crates-badge]][crates-url]
[![BSD 3-Clause licensed][bsd-badge]][bsd-url]

[crates-badge]: https://img.shields.io/crates/v/lps22df-rs
[crates-url]: https://crates.io/crates/lps22df-rs
[bsd-badge]: https://img.shields.io/crates/l/lps22df-rs
[bsd-url]: https://opensource.org/licenses/BSD-3-Clause

Provides a platform-agnostic, no_std-compatible driver for the ST LPS22DF sensor, supporting both I2C and SPI communication interfaces.

## Sensor Overview

The LPS22DF is an ultracompact, piezoresistive, absolute pressure sensor that
functions as a digital output barometer. The LPS22DF provides lower power
consumption, achieving lower pressure noise than its predecessor.

The device comprises a sensing element and an IC interface that communicates
over an I²C, MIPI I3CSM, or SPI interface from the sensing element to the
application and supports a wide Vdd IO range for the digital interfaces as well. The sensing
element, which detects absolute pressure, consists of a suspended membrane manufactured
using a dedicated process developed by ST.

The LPS22DF is available in a full-mold, holed LGA package (HLGA). It is
guaranteed to operate over a temperature range extending from -40 °C to +85 °C. 
The package is holed to allow external pressure to reach the sensing element.

For more info, please visit the device page at [https://www.st.com/en/mems-and-sensors/lps22df.html](https://www.st.com/en/mems-and-sensors/lps22df.html)

## Installation

Add the driver to your `Cargo.toml` dependencies:

```toml
[dependencies]
lps22df-rs = "2.0.0"
```

Or, add it directly from the terminal:

```sh
cargo add lps22df-rs
```

## Usage

By default, the create exposes the **asynchronous** API, and it could be included using:
```rust
use lps22df_rs::asynchronous as lps22df;
use lps22df::*;
use lps22df::prelude::*;
```

### Blocking API (optional feature)

To use the **blocking** API instead of the asynchronous one, disable default features and enable the `blocking` feature in your Cargo.toml
```toml
[dependencies]
lps22df-rs = { version = "2.0.0", default-features = false, features = ["blocking"] }
```
or from the terminal:
```sh
cargo add lps22df-rs --no-default-features --features blocking
```

Then import the blocking API:
```rust
use lps22df_rs::blocking as lps22df;
use lps22df::*;
use lps22df::prelude::*;
```

### Create an instance

Create an instance of the driver with the `new_<bus>` associated function, by passing an I2C (`embedded_hal::i2c::I2c`) instance and I2C address, or an SPI (`embedded_hal::spi::SpiDevice`) instance, along with a timing peripheral.

An example with I2C:

```rust
let mut sensor = Lps22df::new_i2c(i2c, I2CAddress::I2cAddH, delay);
```

### Check "Who Am I" Register

This step ensures correct communication with the sensor. It returns a unique ID to verify the sensor's identity.

```rust
let whoami = sensor.id_get().unwrap();
if whoami != ID {
    panic!("Invalid sensor ID");
}
```

### Configure

See details in specific examples; the following are common api calls:

```rust
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
```

## License

Distributed under the BSD-3 Clause license.

More Information: [http://www.st.com](http://st.com/MEMS).

**Copyright (C) 2025 STMicroelectronics**