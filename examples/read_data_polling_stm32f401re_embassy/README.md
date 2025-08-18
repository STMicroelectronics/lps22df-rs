# LPS22DF Pressure and Temperature Sensor Data Acquisition on STM32F401RE

This example demonstrates how to interface the **LPS22DF** pressure and temperature sensor with an **STM32F401RE** microcontroller board using I2C communication. The program reads pressure and temperature data from the sensor in polling mode and outputs the results over UART.

---

## Hardware Setup

- **Microcontroller Board:** STM32F401RE Nucleo-64
- **Sensor:** LPS22DF Pressure and Temperature Sensor
- **Communication Interface:** I2C1 at 100 kHz Standard Mode
- **UART:** USART2 for serial output at 115200 baud

### Default Pin Configuration

| Signal       | STM32F401RE Pin | Description                    |
|--------------|-----------------|--------------------------------|
| I2C1_SCL     | PB8             | I2C clock line (open-drain)   |
| I2C1_SDA     | PB9             | I2C data line (open-drain)    |
| USART2_TX    | PA2             | UART transmit for debug output|

The LPS22DF sensor is connected to the STM32F401RE via I2C1 on pins PB8 (SCL) and PB9 (SDA). UART output is routed through PA2 for serial communication.

---

## Code Description

### Initialization

- The program initializes microcontroller peripherals including clocks, GPIO pins, I2C, and UART.
- The I2C bus is configured for 100 kHz Standard Mode with open-drain pins PB8 and PB9.
- UART is configured on PA2 at 115200 baud for serial output.
- A delay abstraction is created for timing operations.

### Sensor Configuration

- The LPS22DF sensor is initialized over I2C with the high I2C address.
- The device ID is read and verified; if mismatched, an error message is sent over UART and the program halts.
- The sensor is booted and reset to ensure a clean start.
- Block Data Update (BDU) and interface increment (IF_INC) are enabled for safe driver usage.
- The bus interface is configured with automatic filtering and hardware-selected interface.
- Output data rate is set to 4 Hz with 16-sample averaging and a low-pass filter at ODR/4.
- Interrupt pins are configured, disabling data-ready pressure and temperature interrupts (polling mode).

### Data Acquisition Loop

- The program continuously polls the sensor for new pressure or temperature data availability.
- When new data is ready, pressure (in hPa) and temperature (in °C) are read.
- The values are printed over UART every second.

---

## Usage

1. Connect the LPS22DF sensor to the STM32F401RE Nucleo board via I2C1 (PB8/PB9).
2. Build and flash the firmware onto the STM32F401RE board.
3. Open a serial terminal at 115200 baud on the USART2 TX line.
4. Observe pressure and temperature readings printed every second.

---

## Notes

- This example uses polling mode to read sensor data without interrupts.
- UART output uses blocking writes.
- The environment is `#![no_std]` and `#![no_main]` for embedded Rust applications.
- Panic behavior is set to halt on panic using `defmt` and `panic_probe`.

---

## References

- [STM32F401RE Nucleo-64 Board](https://www.st.com/en/evaluation-tools/nucleo-f401re.html)
- [LPS22DF Datasheet](https://www.st.com/resource/en/datasheet/lps22df.pdf)
- [embassy-stm32 HAL](https://docs.rs/embassy-stm32)

---

*This README provides a detailed explanation of the embedded Rust program for pressure and temperature data acquisition on STM32F401RE using the LPS22DF sensor.*
