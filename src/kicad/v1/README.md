# Magician Main Board (v1)

Original Magician Main Board PCB, designed around an **Arduino Nano (ATmega328P)** with 3x VL53L0X time-of-flight distance sensors and up to 64 lights driven through daisy-chained 74HC595 shift registers, per the [top-level README](../../../README.md).

This is the board described in the MAGICIAN D3.3 deliverable as replacing the earlier breadboard harness for the V2 camera sensor. It has been moved here from `src/kicad/` and superseded by [`v4/`](../v4/README.md) (MagicianCam4), which moves to a Raspberry Pi Pico 2 and a VL53L5CX multizone ToF sensor. Kept for reference and for any deployments still running this hardware revision.
