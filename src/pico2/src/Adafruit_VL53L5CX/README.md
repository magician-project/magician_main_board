# Adafruit VL53L5CX Library
[![Arduino Library CI](https://github.com/adafruit/Adafruit_VL53L5/actions/workflows/githubci.yml/badge.svg)](https://github.com/adafruit/Adafruit_VL53L5/actions/workflows/githubci.yml)

Arduino library for the ST VL53L5CX 8x8 multizone Time-of-Flight sensor. Supports up to 4m range with 4x4 or 8x8 resolution.

**Features**
- Resolution control (4x4 or 8x8)
- Ranging frequency selection
- Sharpener control
- Target order selection
- Ranging mode selection
- Power mode control
- Detection thresholds
- Motion indicator
- Crosstalk (xtalk) calibration
- I2C address change
- LPn pin support

**Hardware Requirements**

> **This sensor will NOT work on AVR boards (Arduino Uno, Mega, etc.).**

The VL53L5CX stores its firmware in volatile RAM. Every time the sensor powers on, the host MCU must upload an ~84KB firmware blob over I2C before the sensor can range. This means:

- **You need an ARM-class MCU** with enough flash and RAM — ESP32, SAMD21/51, RP2040, nRF52840, STM32, etc.
- **AVR chips cannot compile this library.** The firmware blob alone exceeds AVR flash and array initializer limits.
- **First `begin()` call takes up to 10 seconds** while firmware uploads. This is normal — it only happens once after power-on.
- **The sensor retains firmware across soft resets** (as long as 3.3V stays up). Only a full power cycle requires re-upload.
- **I2C clock speed matters.** 400kHz is the default; 1MHz works on most boards and cuts upload time roughly in half.

**Dependencies**
- Adafruit BusIO

**Examples**
- `examples/01_simpletest` - Basic ranging readout
- `examples/02_ascii_art` - Print an 8x8 distance grid as ASCII art
- `examples/03_detection_threshold` - Configure detection thresholds and report triggers
- `examples/04_motion_detect` - Read motion indicator output
- `examples/05_webserial` - Stream data for the browser heatmap viewer
- `examples/06_set_address` - Change the I2C address
- `examples/07_tft_heatmap` - Display a heatmap on a TFT

**WebSerial**
The `webserial/` directory contains a browser-based heatmap visualization that pairs with the WebSerial example.

**License**
MIT

**Product Page**
https://www.adafruit.com/product/XXXXX
