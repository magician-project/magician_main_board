# Magician Main Board for Sensing Robot

![8 Protype Assembled PCBs](https://github.com/magician-project/magician_main_board/blob/main/doc/banner.jpg?raw=true) 

This project implements a **multi-laser distance sensing and light control system** on an **Arduino Nano (ATmega328P)**. 

The software that drives this PCB (along with the Camera/ATI/Teensy) is the [Magician Grabber](https://github.com/magician-project/magician_grabber)

It integrates:

- **3x VL53L0X time-of-flight distance sensors** (over I²C, with unique addresses via XSHUT control) 
- **Up to 8 lights** controlled via direct GPIO or a **74HC595 shift register**
- **Extendable up to 64 lights** by daisy chaining more **74HC595 shift registers**
- **Ethernet W5100 support** (Telnet-style interface) 
- **Two analog input buttons** for manual interaction
- **Serial + Ethernet reporting** of sensor readings, light state, and button presses

The firmware cycles lights automatically, can react to distance sensor readings, and supports both **serial commands** and **Ethernet control**.

---

## Features

- 🔦 **3x VL53L0X** laser distance sensors on a shared I²C bus
- 💡 **6x-8x LED light outputs** (6x direct pin mode or 8x via 74HC595 shift register)
- 🌐 **Ethernet W5100 support** with telnet-style server on port `23`
- 🎛️ **Two analog buttons** (for annotations / input)
- ⚡ **Low-level SRAM monitoring** and reset recovery functions
- 🛠️ **Configurable light modes:**
  - Sequential cycling
  - Pulse mode
  - Sensor-based closest-light decision
  - Timed flashing

---



![Magician Main Board close-up](https://github.com/magician-project/magician_main_board/blob/main/doc/board.jpg?raw=true) 

## Setup

Important: Before connecting the board, [read the following guide](doc/connectors.md)!


## Hardware Components

- **Arduino Nano with USB-Micro port/ UNO (ATmega328P)**  
- **VL53L0X distance sensors** (x3)  
- **W5100 Ethernet module/shield**  
- **74HC595 shift register** (optional, for light control)  
- **6 LEDs or lights** (controlled by direct pins or shift register)  
- **2 push buttons** (wired to analog inputs `A6` and `A7`)  

### Pin Mapping

| Component                  | Pin(s) Used         | Notes |
|-----------------------------|---------------------|-------|
| I²C (VL53L0X)              | A4 (SDA), A5 (SCL) | Shared bus |
| VL53L0X XSHUT (x3)         | A0, A1, A2         | Controls sensor reset & addressing |
| Ethernet Reset             | D9                 | Shared with `I2C_EXTRA_PIN` |
| Direct Light Outputs       | D3, D4, D5, D6, D7, D8 | Default pin mode |
| 74HC595 Latch (STCP)       | D3                 | Used if shift register enabled |
| 74HC595 Clock (SHCP)       | D4                 | 〃 |
| 74HC595 Data (DS)          | D5                 | 〃 |
| 74HC595 OE (Enable)        | D2                 | Active low |
| 74HC595 SRCLR (Clear)      | A3                 | Active low |
| Buttons                    | A6, A7             | Analog threshold detection |

---

## Software Setup

1. **Clone this repository** and the Pololu VL53L0X Arduino library:
   ```bash
   git clone https://github.com/YOUR_GITHUB_USERNAME/arduino-nano-vl53l0x-ethernet
   cd arduino-nano-vl53l0x-ethernet
   git clone https://github.com/pololu/vl53l0x-arduino
   cp vl53l0x-arduino/VL53L0X.cpp ./
   cp vl53l0x-arduino/VL53L0X.h ./

    Fix Arduino Nano memory constraints (important!):
    The VL53L0X library is heavy and may cause stack overflow resets.
    To prevent this, reduce the serial buffer sizes:

    Edit:

~/.arduino15/packages/arduino/hardware/avr/1.8.6/cores/arduino/HardwareSerial.h

Change:

    #define SERIAL_TX_BUFFER_SIZE 16
    #define SERIAL_RX_BUFFER_SIZE 8

    With this, the sketch fits:

        Program storage: ~66% used

        Dynamic memory: ~80% used (401 bytes free)

    Install dependencies in Arduino IDE:

        Wire.h (I²C, built-in)

        SPI.h (Ethernet, built-in)

        Ethernet.h (W5100)

    Upload the sketch to your Arduino Nano.

Serial & Ethernet Commands


To connect to Ethernet (assuming you are on the correct subnet)
```
telnet 192.168.137.64
```


To connect using USB/Serial (assuming you are connected over USB)
```
sudo apt install picocom
picocom -b 115200 /dev/ttyUSB0  --imap lfcrlf 
Ctrl + A + Q to exit
```


When connected via USB serial or Ethernet Telnet (port 23), you can control and query the system by supplying the following command bytes.


Command	Description:

| Command                     | Dangerous           | Description |
|-----------------------------|---------------------|-------|
| v              | No    | Print firmware version |
| h              | Yes   | Enable 74HC595 mode + auto light cycling |
| i              | Yes   | Disable 74HC595 (direct pin mode) |
| o              | Maybe | Continuous lights (disable pulsing) |
| p              | Maybe | Switch to pulsing of 500μs / Subsequent calls increase pulse length by 500µs |
| r              | No    | Reset light cycling |
| a              | No    | Enable sensor-based light selection |
| t              | No    | Enable custom cycling mode (mode 3) |
| y              | No    | Flash lights for 10s, then turn off |
| z              | No    | Turn off all lights and reset Arduino |
| 0-6            | Yes   | Manually activate light with number #N |
| +              | Yes   | Step to next light |
| f              | No    | Disable serial/Ethernet reporting |
| x              | No    | (Ethernet only) Print local IP + hardware status | 
| b              | No    | Make Analog Buttons have binary output  1/0 | 
| n              | No    | Make Analog Buttons have non-binary output  0-1024 | 


Commands marked as potentially dangerous directly alter light states that can be dangerous when
using a connected Camera Board that over volts lights. Please see [the setup guide](doc/connectors.md) to understand why. 


Data Reporting

When enabled, the Arduino periodically outputs status lines in CSV format:

<time_ms>,<button1>,<button2>,<distance1>,<distance2>,<distance3>,<light1>,<light2>,...<light6>

Example:

1050,0,1,523,812,1500,1,0,0,0,0,0

    time_ms – system uptime in ms

    button1/button2 – 0 or 1

    distanceN – measured distance in mm (F = failed, H = too high, 0 = inactive)

    lightN – 1 if active, 0 otherwise

Memory & Stability Notes

    Free SRAM is checked at startup; if less than 231 bytes are available, instability is expected.

    Use the reduced VL53L0X library and shrunk serial buffers for stability.

    A resetFunc() software reset is available to recover from low memory conditions.

Versioning

    Version: 0.26

    Major/minor defined in VERSION_MAJOR, VERSION_MINOR.

License

MIT License.
This project uses Pololu’s VL53L0X Arduino Library

Possible Future Improvements

    Add MQTT over Ethernet for IoT integration

    Support ESP32 (more SRAM, WiFi support, no buffer hacks needed)

    Add dynamic light patterns and user-configurable modes
