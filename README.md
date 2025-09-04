# STM32 Project
Interfacing DHT11 temperature sensor with STM32 microcontroller
<img width="996" height="562" alt="image" src="https://github.com/user-attachments/assets/7ea742ae-97d6-417e-9069-664034313735" />


# STM32 DHT11 Environmental Monitor

This project demonstrates how to interface a DHT11 temperature and humidity sensor with an STM32F401RE microcontroller. It uses a hardware timer for microsecond-level delays and an I2C LCD to display the sensor readings. This project is built using STM32CubeIDE and the HAL library.

## Features

- **Temperature & Humidity Reading:** Acquires digital data from the DHT11 sensor.
- **Microsecond Delays:** Uses a hardware timer (TIM10) to generate precise delays critical for DHT11 communication.
- **I2C LCD Display:** Presents real-time temperature and humidity data on a 16x2 LCD screen.
- **I2C Communication:** Interfaces with the LCD using the I2C protocol via a PCF8574 module.
- **GPIO Control:** Dynamically changes the data pin mode between output and input.

---

## Hardware and Connections

This project was developed on an STM32 Nucleo-F401RE board. The following table details the connections between the microcontroller, the DHT11 sensor, and the I2C LCD module.

### Wiring Diagram

| **Component** | **MCU Pin** | **Description** |
| :--- | :--- | :--- |
| **DHT11 Sensor** | | |
| VCC | 3.3V / 5V | Power supply for the sensor. |
| GND | GND | Ground connection. |
| Data | PA1 | The single-wire data pin for communication. |
| **I2C LCD Module** | | |
| VCC | 5V | Power supply for the LCD. |
| GND | GND | Ground connection. |
| SDA | PB9 (I2C1_SDA) | I2C Data Line. |
| SCL | PB6 (I2C1_SCL) | I2C Clock Line. |

_Note: A pull-up resistor may be required on the DHT11 data pin if it cannot pull the line high._

---

## STM32CubeIDE Configuration

### Clock Configuration

The microcontroller's clock is configured using an external 8 MHz crystal. The **HCLK** is set to **50 MHz**. The **APB2 Timer Clock** is also at 50 MHz, which is critical for the microsecond delays because the hardware timer we are using (TIM10) is on the APB2 bus on the Nucleo Board according to the datasheet. 

### Timer Configuration

- **Timer:** TIM10 (connected to APB2 bus)
- **Prescaler:** `50-1`
- **Counter Mode:** Up
- **Counter Period:** `0xffff-1`

This configuration ensures the timer's clock is `50MHz / 50 = 1 MHz`, allowing each timer tick to represent 1 microsecond (`1/1MHz = 1µs`).

### I2C Configuration

- **I2C Peripheral:** I2C1
- **Clock Speed:** 100 kHz (Standard Mode)
- **Pins:**
  - **SCL:** PB8
  - **SDA:** PB9
- **I2C LCD Address:** `0x4E` (The 8-bit address for the common `0x27` 7-bit address, derived by the operation `0x27 << 1`)

---

## Project Code

The core logic is implemented in `main.c`, which includes functions for DHT11 communication and LCD display.

### Key Functions

- `DHT11_Start()`: Initiates the communication sequence by pulling the data line low for 18ms.
- `DHT11_Check_Response()`: Verifies the sensor's response pulse to confirm it's ready.
- `DHT11_Read()`: Reads a single 8-bit data byte from the sensor by timing the duration of high pulses.
- `delay(us)`: A custom function that uses TIM10 to generate precise microsecond delays.
- `Display_Temp(float temperature)`: Formats and displays the temperature on the LCD.
- `Display_Rh(float humidity)`: Formats and displays the humidity on the LCD.

### LCD Driver Pinout

The `i2c_lcd.c` driver file must be configured to match the physical pinout of your specific I2C module. The following bitmasks are used in this project's code to correctly drive the LCD and enable the backlight.

| **Signal** | **I2C Module Pin** | **Bitmask** | **Function** |
| :--- | :--- | :--- | :--- |
| **RS (Register Select)** | P0 | `0x01` | `0` for command, `1` for data |
| **RW (Read/Write)** | P1 | `0x02` | `0` for write, `1` for read |
| **EN (Enable)** | P2 | `0x04` | `1` for strobe pulse |
| **Backlight** | P3 | `0x08` | `1` for backlight on |
| **Data (D4-D7)** | P4-P7 | `0x10-0x80` | |

By using these bitmasks, the driver correctly controls the LCD's display and backlight.

---

## Troubleshooting

- **No Display/Gibberish:** The most common issue is an incorrect I2C address or pinout mismatch in the driver code. Ensure the 8-bit address is correctly configured (`0x27` becomes `0x4E`). If you see gibberish, adjust the bitmasks in `i2c_lcd.c`.
- **No Backlight:** Confirm the LCD is powered with **5V**. The backlight bit (`0x08`) must be included in the data packets to turn it on.
- **Incorrect Readings:** This can be due to timing issues. Verify your timer configuration provides an accurate 1 µs tick. Adding a pull-up resistor to the DHT11 data line can also improve signal integrity.

***

## Acknowledgement

Adapted from the tutorial by Controllers Tech on a similar project [here](https://controllerstech.com/using-dht11-sensor-with-stm32/). All credit goes to them!

## 📞 Contact

For questions or suggestions, feel free to reach out via email:
📧 zhimingx27@gmail.com
