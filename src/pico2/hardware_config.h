#pragma once

// =============================================================================
//  MagicianCam4 — Rev. 2.2
//  Hardware Configuration, Pinout Mapping & Compile-Time Constants
// =============================================================================
//  Target Platform : Raspberry Pi Pico 2 (RP2350 - ARM Cortex-M33 @ 150 MHz)
//  Board Hardware  : MagicianCam4 Rev. 1.3
//
//  Subsystem Architecture:
//    - Real-Time Core (Core 0):
//        * Camera exposure trigger ISR (GP2) with direct SIO register gating.
//        * RP2350 Hardware Timer Alarms for exposure pulse safety guardrails.
//        * Fast-mode Plus I2C bus master (1 MHz) for MCP23018 expander and
//          three VL53L5CX Time-of-Flight ranging sensors.
//        * Push-button debounce state machine (PB0, PB1).
//    - Application & Transport Core (Core 1):
//        * Serial CDC (115200 baud) and W5500 SPI0 Ethernet transports.
//        * Multi-sensor ToF spatial analysis (center 4-zone matrix average).
//        * 2-D vector inverse-distance dynamic COB lighting recommendation.
//        * 12-field CSV real-time telemetry stream and command parser.
//        * Sliced non-blocking diagnostic reporter (1 line / tick, < 15 us).
// =============================================================================

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include "src/Ethernet/Ethernet.h"              // vendored: Arduino Ethernet 2.0.2
#include "src/Adafruit_VL53L5CX/Adafruit_VL53L5CX.h" // vendored: Adafruit_VL53L5CX 1.0.1 + FORTH patch
#include <atomic>
#include <cmath>
#include "pico/time.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/sync.h"
#include "hardware/timer.h"
#include "hardware/structs/sio.h"
#include "hardware/structs/timer.h"

// =============================================================================
//  Hardware Summary & Direct GPIO Pinout Mapping
// =============================================================================
//  I²C bus (GP4 SDA / GP5 SCL)
//    · 3× VL53L5CX  time-of-flight sensors (datasheet 8-bit addr 0x52; Wire 7-bit addr 0x29 = 0x52 >> 1)
//      Each sensor is LPn-gated via MCP23018 GPA5/4/3 and reassigned a
//      unique address (0x30, 0x31, 0x32) during init.
//    · MCP23018 I²C port expander (7-bit addr 0x20)
//
//  SPI0 bus — W5500 ethernet (USR-ES1 module)
//    MISO=GP16  MOSI=GP19  SCLK=GP18  CS=GP17  INT=GP22
//    Reset is driven by MCP23018 GPA2 (open-drain, active-low).
//
//  Direct GPIO Specification and Distinction:
//    GP2  — XCG-CP510 camera exposure signal (active-high input, ext. pull-down of 15K)
//    GP3  — MCP23018 hardware reset (active-low output, push-pull driven)
//    GP6  — Light Control 1 (active-high output, ext. pulldown of 15K)
//    GP7  — Light Control 2 (active-high output, ext. pulldown of 15K)
//    GP8  — Light Control 3 (active-high output, ext. pulldown of 15K)
//    GP9  — Light Control 4 (active-high output, ext. pulldown of 15K)
//    GP10 — Light Control 5 (active-high output, ext. pulldown of 15K)
//    GP11 — Light Control 6 (active-high output, ext. pulldown of 15K)
//    GP12 — VL53L5CX ToF#1 INT signal (active-low input, ext. pull-up of 4.7K on the sensor's micro-board)
//    GP13 — VL53L5CX ToF#2 INT signal (active-low input, ext. pull-up of 4.7K on the sensor's micro-board)
//    GP14 — VL53L5CX ToF#3 INT signal (active-low input, ext. pull-up of 4.7K on the sensor's micro-board)
//    GP20 — MCP23018 INTA (active-low input, push-pull driven)
//    GP21 — MCP23018 INTB (active-low input, push-pull driven)
//    GP22 — W5500INT (active-low input, ext. pull-up of 4.7K in the W5500 Lite board)
//
//  MCP23018 I/O Expander Port A Specification and Distinction:
//    GPA0 — Push Button 0 (active-low input, int. pull-up)
//    GPA1 — Push Button 1 (active-low input, int. pull-up)
//    GPA2 — W5500RST (active-low output, open-drain, ext. pull-up of 4.7K in the W5500 Lite board)
//    GPA3 — LPn sensor 2 (active-low output, open-drain, ext. pull-up of 4.7K in the VL53L5CX board)
//    GPA4 — LPn sensor 1 (active-low output, open-drain, ext. pull-up of 4.7K in the VL53L5CX board)
//    GPA5 — LPn sensor 0 (active-low output, open-drain, ext. pull-up of 4.7K in the VL53L5CX board)
//    GPA6 — 5 V  rail enable (active-low output, open-drain, ext pull-down of 680K)
//    GPA7 — 11.5 V rail enable (active-low output, open-drain, ext pull-down of 680K)
//
//  MCP23018 I/O Expander Port B — all pins unconnected (inputs, internal pull-ups enabled)
//
//  NOTE: MCP23018 GPIO outputs are open-drain. All output lines that must
//        reach a HIGH state require pull-up resistors (either internal or
//        external, as mapped out above. Internal pull-ups are managed
//        explicitly to prevent parasitic leaks.
//        INTA and INTB outputs are configurable either open-drain or push-pull.
//        Internal pull-ups can be enabled both for the inputs and the outputs.
// =============================================================================

#define LED_PIN                       25   // RP2350 on-board user LED

// =============================================================================
//  Feature Flags & Versioning
// =============================================================================
#define VERSION_STRING                "2.2"
#define USE_VL53L5CX                  1   // Enable 3x VL53L5CX Time-of-Flight subsystem
#define USE_W5500                     1   // Enable W5500 Lite Ethernet SPI0 subsystem
#define ENABLE_LASER_LIGHT_DECISION   1   // Enable automatic depth-based COB recommendations

// =============================================================================
//  Diagnostic Reporting Levels
// =============================================================================
#define VL53L5CX_REPORT_SILENT        0
#define VL53L5CX_REPORT_NORMAL        1
#define VL53L5CX_REPORT_VERBOSE       2
#define VL53L5CX_REPORT_LEVEL         VL53L5CX_REPORT_VERBOSE

#define ETHERNET_REPORT_SILENT        0
#define ETHERNET_REPORT_NORMAL        1
#define ETHERNET_REPORT_VERBOSE       2
#define ETHERNET_REPORT_LEVEL         ETHERNET_REPORT_VERBOSE

// -----------------------------------------------------------------------------
// Core0 -> Core1 Non-Blocking Output Formatting Helpers (Forward Declarations)
// -----------------------------------------------------------------------------
void core0QueueOutputText(const char *text);
void core0QueueOutputFlashText(const __FlashStringHelper *text);
void core0QueueOutputChar(char value);
void core0QueueOutputUnsigned(uint32_t value);
void core0QueueOutputSigned(int32_t value);
void core0QueueOutputHexByte(uint8_t value);
void core0QueueOutputNewline();

void core0QueueOutputValue(const char *value);
void core0QueueOutputValue(const __FlashStringHelper *value);
void core0QueueOutputValue(char value);
void core0QueueOutputValue(uint8_t value);
void core0QueueOutputValue(uint16_t value);
void core0QueueOutputValue(uint32_t value);
void core0QueueOutputValue(int value);

// -----------------------------------------------------------------------------
// Core0 VL53L5CX Logging Macros (Route non-blockingly to Core0->Core1 queue)
// -----------------------------------------------------------------------------
#define VL53L5CX_ERROR(msg)               core0QueueOutputFlashText(msg)
#define VL53L5CX_ERRORLN(msg)             do { core0QueueOutputFlashText(msg); core0QueueOutputNewline(); } while (0)
#define VL53L5CX_WARNING(msg)             core0QueueOutputFlashText(msg)
#define VL53L5CX_WARNINGLN(msg)           do { core0QueueOutputFlashText(msg); core0QueueOutputNewline(); } while (0)

#if VL53L5CX_REPORT_LEVEL >= VL53L5CX_REPORT_NORMAL
  #define VL53L5CX_NORMAL(msg)            core0QueueOutputFlashText(msg)
  #define VL53L5CX_NORMALLN(msg)          do { core0QueueOutputFlashText(msg); core0QueueOutputNewline(); } while (0)
  #define VL53L5CX_NORMAL_DECIMAL(val)    core0QueueOutputValue(val)
  #define VL53L5CX_NORMAL_UNSIGNED(val)   core0QueueOutputUnsigned(val)
  #define VL53L5CX_NORMAL_HEX_ADDRESS(val) do { core0QueueOutputFlashText(F("0x")); core0QueueOutputHexByte(static_cast<uint8_t>(val)); } while (0)
  #define VL53L5CX_NORMAL_NEWLINE()       core0QueueOutputNewline()
#else
  #define VL53L5CX_NORMAL(msg)            ((void)0)
  #define VL53L5CX_NORMALLN(msg)          ((void)0)
  #define VL53L5CX_NORMAL_DECIMAL(val)    ((void)0)
  #define VL53L5CX_NORMAL_UNSIGNED(val)   ((void)0)
  #define VL53L5CX_NORMAL_HEX_ADDRESS(val) ((void)0)
  #define VL53L5CX_NORMAL_NEWLINE()       ((void)0)
#endif

#if VL53L5CX_REPORT_LEVEL >= VL53L5CX_REPORT_VERBOSE
  #define VL53L5CX_VERBOSE(msg)           core0QueueOutputFlashText(msg)
  #define VL53L5CX_VERBOSELN(msg)         do { core0QueueOutputFlashText(msg); core0QueueOutputNewline(); } while (0)
  #define VL53L5CX_VERBOSE_DECIMAL(val)   core0QueueOutputValue(val)
  #define VL53L5CX_VERBOSE_UNSIGNED(val)  core0QueueOutputUnsigned(val)
  #define VL53L5CX_VERBOSE_HEX_BYTE(val)  core0QueueOutputHexByte(static_cast<uint8_t>(val))
  #define VL53L5CX_VERBOSE_HEX_ADDRESS(val) do { core0QueueOutputFlashText(F("0x")); core0QueueOutputHexByte(static_cast<uint8_t>(val)); } while (0)
  #define VL53L5CX_VERBOSE_NEWLINE()      core0QueueOutputNewline()
#else
  #define VL53L5CX_VERBOSE(msg)           ((void)0)
  #define VL53L5CX_VERBOSELN(msg)         ((void)0)
  #define VL53L5CX_VERBOSE_DECIMAL(val)   ((void)0)
  #define VL53L5CX_VERBOSE_UNSIGNED(val)  ((void)0)
  #define VL53L5CX_VERBOSE_HEX_BYTE(val)  ((void)0)
  #define VL53L5CX_VERBOSE_HEX_ADDRESS(val) ((void)0)
  #define VL53L5CX_VERBOSE_NEWLINE()      ((void)0)
#endif

// -----------------------------------------------------------------------------
// Core1 Ethernet Logging Macros (Serial transport directly owned by Core1)
// -----------------------------------------------------------------------------
#define ETH_ERROR(msg)                    Serial.print(msg)
#define ETH_ERRORLN(msg)                  Serial.println(msg)
#define ETH_WARNING(msg)                  Serial.print(msg)
#define ETH_WARNINGLN(msg)                Serial.println(msg)

#if ETHERNET_REPORT_LEVEL >= ETHERNET_REPORT_NORMAL
  #define ETH_NORMAL(msg)                 Serial.print(msg)
  #define ETH_NORMALLN(msg)               Serial.println(msg)
#else
  #define ETH_NORMAL(msg)                 ((void)0)
  #define ETH_NORMALLN(msg)               ((void)0)
#endif

#if ETHERNET_REPORT_LEVEL >= ETHERNET_REPORT_VERBOSE
  #define ETH_VERBOSE(msg)                Serial.print(msg)
  #define ETH_VERBOSELN(msg)              Serial.println(msg)
#else
  #define ETH_VERBOSE(msg)                ((void)0)
  #define ETH_VERBOSELN(msg)              ((void)0)
#endif

// =============================================================================
//  I2C Bus & MCP23018 Hardware Definitions
// =============================================================================
#define SDA_PIN                           4    // GP4: I2C Serial Data
#define SCL_PIN                           5    // GP5: I2C Serial Clock
#define MCP23018_ADDR                     0x20 // 7-bit I2C address (HW address --> A2=A1=A0=GND)
#define MCP23018_RESET_PIN                3    // GP3 : MCP23018 HW reset pin
#define MCP23018_INTA_PIN                 20   // GP28: Port A interrupt
#define MCP23018_INTB_PIN                 21   // GP21: Port B interrupt

// MCP23018 Registers (IOCON.BANK = 0 paired sequential memory map)
#define MCP23018_IODIRA                   0x00 // Port A Direction (1=Input, 0=Output)
#define MCP23018_IODIRB                   0x01 // Port B Direction
#define MCP23018_IPOLA                    0x02 // Port A Input Polarity
#define MCP23018_IPOLB                    0x03 // Port B Input Polarity
#define MCP23018_GPINTENA                 0x04 // Port A Interrupt-on-Change Enable
#define MCP23018_GPINTENB                 0x05 // Port B Interrupt-on-Change Enable
#define MCP23018_DEFVALA                  0x06 // Port A Default Value for Interrupts
#define MCP23018_DEFVALB                  0x07 // Port B Default Value
#define MCP23018_INTCONA                  0x08 // Port A Interrupt Control (0=vs prev, 1=vs DEFVAL)
#define MCP23018_INTCONB                  0x09 // Port B Interrupt Control
#define MCP23018_IOCON                    0x0A // Chip Configuration Register
#define MCP23018_GPPUA                    0x0C // Port A Internal Pull-up Enable
#define MCP23018_GPPUB                    0x0D // Port B Internal Pull-up Enable
#define MCP23018_INTFA                    0x0E // Port A Interrupt Flags (Read-only)
#define MCP23018_INTFB                    0x0F // Port B Interrupt Flags (Read-only)
#define MCP23018_INTCAPA                  0x10 // Port A Captured Value at Interrupt (Read-only)
#define MCP23018_INTCAPB                  0x11 // Port B Captured Value at Interrupt (Read-only)
#define MCP23018_GPIOA                    0x12 // Port A Pin State Register
#define MCP23018_GPIOB                    0x13 // Port B Pin State Register
#define MCP23018_OLATA                    0x14 // Port A Output Latch (Write to set outputs)
#define MCP23018_OLATB                    0x15 // Port B Output Latch

// MCP23018 Port A Pin Bit Positions
#define MCP_GPA_PB0                       0   // Bit 0: Push Button 0 (input, internal pull-up)
#define MCP_GPA_PB1                       1   // Bit 1: Push Button 1 (input, internal pull-up)
#define MCP_GPA_W5500RST                  2   // Bit 2: W5500 Hardware Reset (active-low, open-drain)
#define MCP_GPA_LPN2                      3   // Bit 3: VL53L5CX Sensor #2 LPn (active-low, open-drain)
#define MCP_GPA_LPN1                      4   // Bit 4: VL53L5CX Sensor #1 LPn (active-low, open-drain)
#define MCP_GPA_LPN0                      5   // Bit 5: VL53L5CX Sensor #0 LPn (active-low, open-drain)
#define MCP_GPA_RAIL5V                    6   // Bit 6: +5V Rail Enable (active-low, open-drain)
#define MCP_GPA_RAIL11V                   7   // Bit 7: +11.5V Rail Enable (active-low, open-drain)

#define PB_MASK                           0x03 // GPA0/GPA1 bitmask for input buttons
#define PB_TICK_PERSISTENCE               4    // Debounce persistence (4 ticks @ 20ms = ~80ms)

#define MCP23018_RST_SLEEP_MS             2    // Hold duration for MCP23018 hardware reset
#define MCP23018_RST_RESUME_MS            2    // Settling delay after releasing MCP23018 reset

// =============================================================================
//  VL53L5CX Time-of-Flight Sensor Definitions
// =============================================================================
#define NUMBER_OF_DISTANCE_SENSORS        3
#define LOX1_ADDRESS                      0x30 // Reassigned 7-bit runtime address for ToF 0
#define LOX2_ADDRESS                      0x31 // Reassigned 7-bit runtime address for ToF 1
#define LOX3_ADDRESS                      0x32 // Reassigned 7-bit runtime address for ToF 2

#define SENSOR0_INT_PIN                   12   // GP12: VL53L5CX 0 Data-Ready INT
#define SENSOR1_INT_PIN                   13   // GP13: VL53L5CX 1 Data-Ready INT
#define SENSOR2_INT_PIN                   14   // GP14: VL53L5CX 2 Data-Ready INT

#define VL53L5CX_LPN_SLEEP_MS             2    // Low-Power mode assertion hold time
#define VL53L5CX_LPN_RESUME_MS            2    // Low-Power mode release settling time
#define VL53L5CX_FIRST_FRAME_TIMEOUT_MS   5000 // Discard startup frame timeout (ms)
#define TOF_DATA_STALE_MS                 500  // Staleness threshold for ToF depth readings (ms)
#define TOF_MAX_DISTANCE_VALID_MM         6000u // Maximum valid ranging threshold (mm)

// =============================================================================
//  COB LED Illumination & Camera Exposure Trigger Definitions
// =============================================================================
#define NUMBER_OF_LIGHTS                  6
#define NO_LIGHT_ON                       16   // Sentinel value indicating no active COB
#define CAMERA_EXPOSURE_PIN               2    // GP2: Sony camera shutter exposure signal

// Hardware-Timed Guardrail Intervals (Strictly referenced to exposure rising edge t0):
#define LIGHT_FALL_ARM_DELAY_US           20    // Stage 1: Noise guard window before arming FALL (20 us)
#define LIGHT_TURNOFF_US                  1000  // Stage 2: Guardrail maximum ON timeout (1000 us from t0)
#define LIGHT_CYCLE_PERIOD_US             40000 // Stage 3: Minimum cycle period (40000 us from t0, <= 25 Hz)
#define LIGHT_ALL_PINS_MASK               0x0FC0u // SIO bitmask for COB pins GP6..GP11: (0x3F << 6)

// Host-adjustable light limits (Rev 2.2):
#define LIGHT_ON_CEILING_MIN_US           50    // 'E<us>' lower clamp (must stay > LIGHT_FALL_ARM_DELAY_US)
                                                // 'E<us>' upper clamp is LIGHT_TURNOFF_US
#define LIGHT_SCHEDULE_MAX_LENGTH         60    // 'S<digits>' maximum number of schedule entries
#define COMMAND_LINE_MAX_LENGTH           72    // Longest multi-character command line ('E','N','S','P')
// Power rails (Cam5 / v4 schematics):
//   +11.5VCC -> J16 "Cam. Pwr" jumper -> CAM_PWR -> camera connector J8 pin 4 (Sony XCG-CP510), panel I/O J13
//   +5VCC    -> Q19 P-FET -> Pico 2 VSYS whenever USB VBUS is absent (powers the Pico, and through its
//               3V3 regulator the MCP23018, ToF sensors and WIZ850io), panel I/O J13
// Switching +5VCC off without USB therefore cuts the board's own supply: refused unless VBUS is present.
#define USB_VBUS_SENSE_PIN                24    // Pico 2 GP24: high while USB VBUS is present
#define RAIL_CYCLE_DEFAULT_OFF_MS         500   // 'PR' with no number: the camera rail stays off this long
#define RAIL_CYCLE_MIN_OFF_MS             50    // 'PR<ms>' clamp
#define RAIL_CYCLE_MAX_OFF_MS             10000

// =============================================================================
//  W5500 Ethernet SPI Architecture & Network Configuration
// =============================================================================
#define W5500_CS_PIN                      17   // GP17: W5500 SPI0 Chip Select
#define W5500_INT_PIN                     22   // GP22: W5500 Interrupt
#define W5500_RST_SLEEP_MS                2    // W5500 reset pulse duration
#define W5500_RST_RESUME_MS               2    // W5500 post-reset boot delay

// =============================================================================
//  Global Timing, Scheduling & Communication Parameters
// =============================================================================
#define CPU_SLEEP_US                      1000  // Core 0 idle delay per loop iteration (1 ms)
#define LOOP1_SLEEP_MS                    10    // Core 1 idle delay per loop1 iteration (10 ms)
#define SERIAL_BAUD_RATE                  115200
#define USB_STARTUP_TIMEOUT_MS            5000u
#define USB_STARTUP_POLL_DELAY_MS         10
#define USB_POST_CONNECT_SETTLE_MS        200

#define MAX_ETHERNET_RX_BYTES_PER_LOOP1   4
#define MAX_SERIAL_RX_BYTES_PER_LOOP1     4
#define MAX_CORE0_OUTPUT_BYTES_PER_LOOP1  128u

#define RESET_COUNTDOWN_START_SEC         3
#define RESET_COUNTDOWN_PERIOD_MS         1000
#define RESET_PRE_WATCHDOG_DELAY_MS       300
#define RESET_WATCHDOG_TIMEOUT_MS         100
#define BOOT_INTERCORE_WAIT_DELAY_MS      1

#define BOOT_LED_START_COUNT              3
#define BOOT_LED_START_BLINK_MS           80
#define BOOT_LED_COMPLETE_COUNT           2
#define BOOT_LED_COMPLETE_BLINK_MS        300
