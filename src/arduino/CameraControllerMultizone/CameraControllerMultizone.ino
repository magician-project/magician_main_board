// =============================================================================
//  CameraControllerMultizone — Raspberry Pi Pico 2 (RP2350)
//
//  Hardware summary
//  ─────────────────────────────────────────────────────────────────────────
//  I²C bus (GP4 SDA / GP5 SCL)
//    · 3× VL53L5CX  time-of-flight sensors  (datasheet 8-bit addr 0x52; Wire 7-bit addr 0x29 = 0x52 >> 1)
//      Each sensor is XSHUT-gated via MCP23018 GPA3/4/5 and reassigned a
//      unique address (0x30, 0x31, 0x32) during init.
//    · MCP23018 I²C port expander            (7-bit addr 0x20)
//
//  SPI0 bus — W5500 ethernet (USR-ES1 module)
//    MISO=GP16  MOSI=GP19  SCLK=GP18  CS=GP17  INT=GP22
//    Reset is driven by MCP23018 GPA2 (open-drain, active-low).
//
//  Direct GPIO
//    GP2  — XCG-CP510 camera exposure signal (active-high input)
//    GP3  — MCP23018 hardware reset (active-low output)
//    GP6  — Light 1
//    GP7  — Light 2
//    GP8  — Light 3
//    GP9  — Light 4
//    GP10 — Light 5
//    GP11 — Light 6
//    GP20 — MCP23018 INTA (open-drain, active-low, input with pull-up)
//    GP21 — MCP23018 INTB (open-drain, active-low, input with pull-up)
//    GP22 — W5500 INT     (active-low, input with pull-up)
//
//  MCP23018 Port A
//    GPA0 — Push Button 0  (input,  internal pull-up, active-low)
//    GPA1 — Push Button 1  (input,  internal pull-up, active-low)
//    GPA2 — W5500 RST      (output, open-drain, active-low)
//    GPA3 — XSHUT sensor 0 (output, open-drain, active-low)
//    GPA4 — XSHUT sensor 1 (output, open-drain, active-low)
//    GPA5 — XSHUT sensor 2 (output, open-drain, active-low)
//    GPA6 — 5 V  rail enable  (output, open-drain, ext pull-up)
//    GPA7 — 11.5 V rail enable(output, open-drain, ext pull-up)
//
//  MCP23018 Port B — all pins unconnected (inputs, internal pull-ups enabled)
//
//  NOTE: MCP23018 GPIO outputs are open-drain.  All output lines that must
//        reach a HIGH state require external pull-up resistors.
// =============================================================================

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Ethernet.h>
#include <SparkFun_VL53L5CX_Library.h>
// Library: "SparkFun VL53L5CX Arduino Library" — install via Library Manager
#include "pico/time.h"      // add_alarm_in_us, cancel_alarm
#include "hardware/gpio.h"  // gpio_put, gpio_set_irq_enabled
#include "hardware/irq.h"   // irq_set_priority, PICO_HIGHEST_IRQ_PRIORITY

// =============================================================================
//  Feature flags
// =============================================================================
#define USE_LASER                  1   // Enable VL53L5CX time-of-flight sensors
#define USE_ETHERNET               1   // Enable W5500 ethernet
#define USE_MCP23018               1   // Enable MCP23018 I²C port expander
#define ENABLE_LASER_LIGHT_DECISION 1  // Enable depth-based automatic light selection
#define DEBUG_TOF               1   // Verbose ToF sensor init/scan output
#define DEBUG_ETHERNET             0   // Verbose ethernet init output

// =============================================================================
//  Version
// =============================================================================
#define VERSION_MAJOR 1
#define VERSION_MINOR 0

// =============================================================================
//  Debug macros
// =============================================================================
#if DEBUG_TOF
  #define DBG_TOF(...)   Serial.print(__VA_ARGS__)
  #define DBG_TOFLN(...) Serial.println(__VA_ARGS__)
#else
  #define DBG_TOF(...)
  #define DBG_TOFLN(...)
#endif

// =============================================================================
//  I²C bus
// =============================================================================
#define SDA_PIN 4   // GP4
#define SCL_PIN 5   // GP5

// =============================================================================
//  MCP23018 I²C port expander
// =============================================================================
#define MCP23018_ADDR      0x20  // 7-bit address; hardware pins A2=A1=A0=GND
#define MCP23018_RESET_PIN 3     // GP3  — active-low hardware reset
#define MCP23018_INTA_PIN  20    // GP20 — open-drain interrupt A (buttons)
#define MCP23018_INTB_PIN  21    // GP21 — open-drain interrupt B (unused)

// MCP23018 register addresses — IOCON.BANK=0 (sequential / paired layout, power-on default)
#define MCP23018_IODIRA   0x00  // Port A I/O direction  (1=input, 0=output)
#define MCP23018_IODIRB   0x01  // Port B I/O direction
#define MCP23018_IPOLA    0x02  // Port A input polarity
#define MCP23018_IPOLB    0x03  // Port B input polarity
#define MCP23018_GPINTENA 0x04  // Port A interrupt-on-change enable
#define MCP23018_GPINTENB 0x05  // Port B interrupt-on-change enable
#define MCP23018_DEFVALA  0x06  // Port A default compare value for interrupts
#define MCP23018_DEFVALB  0x07  // Port B default compare value
#define MCP23018_INTCONA  0x08  // Port A interrupt control (0=vs prev, 1=vs DEFVAL)
#define MCP23018_INTCONB  0x09  // Port B interrupt control
#define MCP23018_IOCON    0x0A  // Chip configuration register
#define MCP23018_GPPUA    0x0C  // Port A pull-up enable (input pins only)
#define MCP23018_GPPUB    0x0D  // Port B pull-up enable
#define MCP23018_INTFA    0x0E  // Port A interrupt flag  (read-only)
#define MCP23018_INTFB    0x0F  // Port B interrupt flag  (read-only)
#define MCP23018_INTCAPA  0x10  // Port A value captured at interrupt (read-only)
#define MCP23018_INTCAPB  0x11  // Port B value captured at interrupt (read-only)
#define MCP23018_GPIOA    0x12  // Port A GPIO register   (read = pin state)
#define MCP23018_GPIOB    0x13  // Port B GPIO register
#define MCP23018_OLATA    0x14  // Port A output latch    (write = set outputs)
#define MCP23018_OLATB    0x15  // Port B output latch

// Port A bit positions (used with mcp23018SetPortABit / read helpers)
#define MCP_GPA_PB0     0   // Push Button 0     — input,  pull-up (LOW=pressed)
#define MCP_GPA_PB1     1   // Push Button 1     — input,  pull-up (LOW=pressed)
#define MCP_GPA_ETHRST  2   // W5500 Reset       — output, open-drain, active-low
#define MCP_GPA_XSHUT0  3   // ToF XSHUT 0       — output, open-drain, active-low
#define MCP_GPA_XSHUT1  4   // ToF XSHUT 1       — output, open-drain, active-low
#define MCP_GPA_XSHUT2  5   // ToF XSHUT 2       — output, open-drain, active-low
#define MCP_GPA_RAIL5V  6   // 5 V  rail enable  — output, open-drain, ext pull-up
#define MCP_GPA_RAIL11V 7   // 11.5 V rail enable— output, open-drain, ext pull-up

// =============================================================================
//  VL53L5CX time-of-flight sensors
// =============================================================================
#define NUMBER_OF_DISTANCE_SENSORS 3

// Unique 7-bit I²C addresses assigned to each sensor after reset sequencing.
// The VL53L5CX datasheet quotes the default I²C address as 0x52 in 8-bit notation.
// The Arduino Wire library uses 7-bit addresses: 0x52 >> 1 = 0x29.  The value 0x29
// passed to tof[i].begin() is therefore correct for the Wire API.
// After sequencing, all three sensors are reassigned unique 7-bit addresses.
#define LOX1_ADDRESS 0x30
#define LOX2_ADDRESS 0x31
#define LOX3_ADDRESS 0x32
const uint8_t LOX_ADDRESS[NUMBER_OF_DISTANCE_SENSORS] = {LOX1_ADDRESS, LOX2_ADDRESS, LOX3_ADDRESS};

#define XSHUT_BOOT_DELAY_MS  80  // Time to allow a sensor to boot after XSHUT goes HIGH
#define XSHUT_RESET_DELAY_MS 10  // Time to hold all sensors in reset before sequencing

// =============================================================================
//  Light control
// =============================================================================
#define NUMBER_OF_LIGHTS 6
#define NO_LIGHT_ON      16  // Sentinel — no light currently active

// Direct GPIO pins for lights 1–6 (GP6–GP11)
const uint8_t directLightControl[NUMBER_OF_LIGHTS] = {6, 7, 8, 9, 10, 11};

// =============================================================================
//  Camera exposure trigger — XCG-CP510
// =============================================================================
// GP2 goes HIGH during the camera shutter open window (active-high).
// The sketch turns the currently selected light on at the rising edge and off
// at the falling edge so illumination is perfectly gated to exposure time.
#define CAMERA_EXPOSURE_PIN 2

// =============================================================================
//  Ethernet — W5500 (USR-ES1 module)
// =============================================================================
// SPI0: MISO=GP16, MOSI=GP19, SCLK=GP18, CS=GP17, INT=GP22
// Hardware reset is managed by MCP23018 GPA2 — no dedicated Pico reset pin.
#define ETHERNET_CS_PIN  17   // GP17
#define ETHERNET_INT_PIN 22   // GP22, active-low interrupt from W5500

// Network configuration — adjust to suit your deployment
byte     mac[]     = {0x02, 0xAB, 0xCD, 0x12, 0x34, 0x56};
IPAddress ip(192, 168, 137, 64);
byte     gateway[] = {192, 168, 1, 3};
byte    *dns       = gateway;
byte     subnet[]  = {255, 255, 255, 0};

EthernetServer server(23);   // Telnet-style command server on port 23
EthernetClient client;

// =============================================================================
//  Global state
// =============================================================================
#define CPU_SLEEP_US 1000   // Microseconds to sleep at the end of each loop tick

bool         serialOutputEnabled  = false;
unsigned int loopRateMsec         = 10;   // Main loop cadence (10 ms = 100 Hz)
unsigned int lightDurationMsec    = 100;  // How long each light stays on in auto modes

// Exposure-ISR timing parameters — written only by the command handler, read by ISRs.
// All times are measured from the rising edge of the camera exposure signal (t = 0).
volatile uint32_t LightDebouncingMicroseconds = 0;  // µs: delay before arming falling-edge ISR (set in setup)
volatile uint32_t lightTurnOffMicroseconds    = 0;  // µs: maximum light-on window from rising edge (set in setup)
volatile uint32_t LightKeepOffMicroseconds    = 0;  // µs: guard-rail before rising-edge ISR re-arms (set in setup)

// lightToActivate: 0 = no light, 1–6 = light to illuminate on next exposure rising edge.
// Written by activateLight() / deactivateLights() in the main context; read in exposureISR().
volatile uint8_t lightToActivate = 0;

unsigned char autoLights = 0;           // 0=manual 1=sequential 2=depth-guided 3=alt-seq 4=timed-seq
unsigned char lightOn    = NO_LIGHT_ON; // Logically selected light index, 0-indexed (NO_LIGHT_ON=none)

unsigned long lastUpdateTime = 0;
unsigned long lightStartTime = 0;

// =============================================================================
//  VL53L5CX sensor objects
// =============================================================================
SparkFun_VL53L5CX  tof[NUMBER_OF_DISTANCE_SENSORS];
VL53L5CX_ResultsData tofResults;  // Single results buffer, reused across all sensors

uint8_t  laser_working[NUMBER_OF_DISTANCE_SENSORS] = {0};   // 1 if sensor initialised OK
uint8_t  laser_status [NUMBER_OF_DISTANCE_SENSORS] = {0};   // 1 if new data available this tick
uint16_t laser_distance_millimeters[NUMBER_OF_DISTANCE_SENSORS] = {0};

// Full 8×8 (64-zone) distance frame for each sensor, refreshed every time
// read_triple_sensors() pulls a new frame.  Streamed verbatim by reportZones().
uint16_t zoneDistances[NUMBER_OF_DISTANCE_SENSORS][64] = {0};
// When set, every reportState() line is followed by three "x1/x2/x3" zone lines.
// Defaults ON: this firmware only runs on the multizone (VL53L5CX) hardware.
bool emitAllZones = true;

// =============================================================================
//  MCP23018 state
// =============================================================================
static bool    mcp23018_present    = false;
static uint8_t mcp23018_portA_latch = 0xFC;  // Software mirror of OLATA — avoids an extra I²C read
                                              // on every bit-set operation

// =============================================================================
//  Utilities
// =============================================================================

// reset_millis — zero the millisecond counter so timestamps in the serial
// output start from 0 whenever a new capture session begins.
// reset_millis stores a snapshot of millis() so that currentTime in each new
// capture session starts near zero (the command handler sets currentTime=0 on
// the same tick, making the timestamps consistent from the host's perspective).
static uint32_t millis_offset = 0;
void reset_millis() { millis_offset = millis(); }

// Software reset — jumps to address 0; works reliably on RP2040 (maps to ROM).
void (* resetFunc)(void) = 0;

// freeRAM — returns approximate free heap bytes (informational only).
int freeRAM()
{
#if defined(ARDUINO_ARCH_RP2040)
  return 512000;  // RP2350 has 520 kB SRAM; report a conservative constant
#else
  return -1;
#endif
}

void version()
{
  Serial.print(F("V:")); Serial.print(VERSION_MAJOR);
  Serial.print(F(".")); Serial.println(VERSION_MINOR);
#if USE_ETHERNET
  if (client) {
    client.print(F("V:")); client.print(VERSION_MAJOR);
    client.print(F(".")); client.println(VERSION_MINOR);
  }
#endif
}

void sram()
{
  Serial.print(F("SRAM(")); Serial.print(freeRAM()); Serial.println(F("b free)"));
}

// =============================================================================
//  I²C scan — checks only the known-present addresses (DEBUG_TOF only)
//
//  Expected addresses in this system:
//    0x20 — MCP23018 port expander (fixed, always present)
//    0x29 — VL53L5CX default boot address (7-bit of datasheet 8-bit 0x52)
//    0x30, 0x31, 0x32 — VL53L5CX after address reassignment
//  Scanning only these five addresses avoids probing the full 127-address space
//  and keeps initialisation time short at the 1 MHz (FM+) bus speed.
// =============================================================================
#if DEBUG_TOF
static void scanI2C()
{
  static const uint8_t known[] = {0x20, 0x29, 0x30, 0x31, 0x32};
  DBG_TOFLN(F("I2C scan:"));
  uint8_t count = 0;
  for (uint8_t k = 0; k < sizeof(known); k++)
  {
    uint8_t addr = known[k];
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0)
    {
      DBG_TOF(F("  found @ 0x"));
      if (addr < 16) DBG_TOF('0');
      DBG_TOFLN(addr, HEX);
      count++;
    }
  }
  if (count == 0) DBG_TOFLN(F("  (none)"));
}
#else
static void scanI2C() {}
#endif

// =============================================================================
//  Ethernet helpers — thin wrappers so the rest of the code stays readable
// =============================================================================
#if USE_ETHERNET
void ethPrint(const __FlashStringHelper *m) { if (client) client.print(m); }
void ethPrint(const char *m)                { if (client) client.print(m); }
void ethPrint(int v)                        { if (client) client.print(v); }
void ethPrint(unsigned int v)               { if (client) client.print(v); }
void ethPrint(unsigned long v)              { if (client) client.print(v); }
void ethPrintln(const __FlashStringHelper *m){ if (client) client.println(m); }
void ethPrintln(const char *m)              { if (client) client.println(m); }
void ethPrintln(int v)                      { if (client) client.println(v); }
void ethFlush()                             { if (client) client.flush(); }
#endif

// =============================================================================
//  MCP23018 — low-level I²C register access
// =============================================================================

// Write a single byte to an MCP23018 register.
// Returns true on ACK, false if the device is absent or the bus is busy.
static bool mcp23018WriteReg(uint8_t reg, uint8_t val)
{
  Wire.beginTransmission(MCP23018_ADDR);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

// Read a single byte from an MCP23018 register.
// Returns 0xFF if the device does not respond (safe default for input registers).
static uint8_t mcp23018ReadReg(uint8_t reg)
{
  Wire.beginTransmission(MCP23018_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return 0xFF;  // repeated-start
  Wire.requestFrom((uint8_t)MCP23018_ADDR, (uint8_t)1);
  return Wire.available() ? Wire.read() : 0xFF;
}

// =============================================================================
//  MCP23018 — Port A bit-level helpers
// =============================================================================

// Set or clear a single output bit on Port A.
// Uses mcp23018_portA_latch as a shadow register so we never need to read
// back OLATA over I²C (saves one transaction per operation).
static void mcp23018SetPortABit(uint8_t bit, bool state)
{
  if (state) mcp23018_portA_latch |=  (1u << bit);
  else       mcp23018_portA_latch &= ~(1u << bit);
  mcp23018WriteReg(MCP23018_OLATA, mcp23018_portA_latch);
}

// XSHUT control — idx 0/1/2 map to GPA3/4/5.
// Pass true to release the sensor (XSHUT HIGH → sensor running).
// Pass false to hold the sensor in reset (XSHUT LOW → sensor silent on bus).
// Open-drain: "HIGH" releases the pin to an external pull-up.
void mcp23018SetXSHUT(uint8_t idx, bool state)
{
  if (idx < NUMBER_OF_DISTANCE_SENSORS)
    mcp23018SetPortABit(MCP_GPA_XSHUT0 + idx, state);
}

// Ethernet W5500 hardware reset via GPA2 (open-drain, active-low).
// Pass false to assert reset, true to release (chip begins initialising).
void mcp23018SetEthReset(bool state) { mcp23018SetPortABit(MCP_GPA_ETHRST,  state); }

// Power rail enables (open-drain — external pull-ups required for HIGH state).
void mcp23018SetRail5V (bool state)  { mcp23018SetPortABit(MCP_GPA_RAIL5V,  state); }
void mcp23018SetRail11V(bool state)  { mcp23018SetPortABit(MCP_GPA_RAIL11V, state); }

// Read a button state.  idx=0 → PB0 (GPA0), idx=1 → PB1 (GPA1).
// Returns true when the button is pressed (pin pulled LOW through the switch;
// internal pull-up holds it HIGH when open).
bool mcp23018ReadButton(uint8_t idx)
{
  if (!mcp23018_present) return false;
  return !(mcp23018ReadReg(MCP23018_GPIOA) & (1u << (idx & 1)));
}

// =============================================================================
//  MCP23018 — initialisation
// =============================================================================
void initMCP23018()
{
  // ── 1. Hardware reset ───────────────────────────────────────────────────────
  // Pull the RESET pin LOW for ≥1 µs to force a full chip reset, then release.
  // This guarantees all registers are at their power-on defaults regardless of
  // any previous partial initialisation (e.g. after a Pico soft-reset).
  pinMode(MCP23018_RESET_PIN, OUTPUT);
  digitalWrite(MCP23018_RESET_PIN, LOW);
  delayMicroseconds(10);
  digitalWrite(MCP23018_RESET_PIN, HIGH);
  delay(5);  // Allow the oscillator and internal logic to stabilise

  // ── 2. Interrupt pin setup on Pico side ────────────────────────────────────
  // MCP23018 interrupt outputs are always open-drain regardless of IOCON.ODR.
  // We enable the Pico's internal pull-ups so the lines idle HIGH and the Pico
  // sees a clean falling edge when an interrupt fires.
  pinMode(MCP23018_INTA_PIN, INPUT_PULLUP);
  pinMode(MCP23018_INTB_PIN, INPUT_PULLUP);

  // ── 3. IOCON — chip-level configuration ────────────────────────────────────
  // Byte value 0x00 keeps all bits at reset defaults:
  //   BANK   = 0  registers are in sequential paired layout (address map above)
  //   MIRROR = 0  INTA and INTB are independent (not mirrored)
  //   SEQOP  = 0  address pointer auto-increments after each byte (sequential)
  //   DISSLW = 0  SDA slew rate control enabled
  //   HAEN   = 0  hardware address enable (no effect on MCP23018, I²C uses ADDR pins)
  //   ODR    = 0  INT pins are open-drain (MCP23018 ignores this; always open-drain)
  //   INTPOL = 0  INT pins are active-LOW
  if (!mcp23018WriteReg(MCP23018_IOCON, 0x00))
  {
    // If the first write fails the device is absent or the bus is not ready.
    Serial.println(F("MCP23018: not found"));
    mcp23018_present = false;
    return;
  }

  // ── 4. Port A direction ─────────────────────────────────────────────────────
  // IODIRA = 0b00000011
  //   Bits 0–1  (GPA0/GPA1) = 1 → inputs  (push buttons PB0, PB1)
  //   Bits 2–7  (GPA2–GPA7) = 0 → outputs (ETH RST, XSHUT ×3, rail enables)
  mcp23018WriteReg(MCP23018_IODIRA, 0x03);

  // Enable internal pull-ups on the two button inputs so they read HIGH at rest
  // and LOW when a button shorts the pin to GND.
  mcp23018WriteReg(MCP23018_GPPUA, 0x03);

  // ── 5. Port B direction ─────────────────────────────────────────────────────
  // All Port B pins are unconnected.  Set all as inputs and enable all pull-ups
  // to hold each pin at a defined HIGH level and prevent spurious interrupts.
  mcp23018WriteReg(MCP23018_IODIRB, 0xFF);
  mcp23018WriteReg(MCP23018_GPPUB,  0xFF);

  // ── 6. Interrupt-on-change — buttons only ──────────────────────────────────
  // GPINTENA = 0b00000011 → enable interrupt-on-change for GPA0 and GPA1 only.
  // INTCONA  = 0x00       → compare each pin against its previous value
  //                         (not against a fixed DEFVAL reference).
  // Port B interrupts are disabled entirely; floating unconnected pins with
  // pull-ups enabled will never toggle, but we leave INTB off for safety.
  mcp23018WriteReg(MCP23018_GPINTENA, 0x03);
  mcp23018WriteReg(MCP23018_GPINTENB, 0x00);
  mcp23018WriteReg(MCP23018_INTCONA,  0x00);
  mcp23018WriteReg(MCP23018_INTCONB,  0x00);

  // ── 7. Initial output state ─────────────────────────────────────────────────
  // Output bits start HIGH (latch = 0b11111100 = 0xFC):
  //   GPA2 (ETH RST)  HIGH → W5500 released from reset at start-up
  //   GPA3-5 (XSHUT)  HIGH → all ToF sensors released from reset at start-up
  //   GPA6-7 (rails)  HIGH → power rails enabled at start-up
  mcp23018_portA_latch = 0xFC;
  mcp23018WriteReg(MCP23018_OLATA, mcp23018_portA_latch);
  mcp23018WriteReg(MCP23018_OLATB, 0x00);  // Port B is all-input; latch value is irrelevant

  mcp23018_present = true;
  Serial.println(F("MCP23018: OK"));
  scanI2C();  // Show all I²C devices visible at this point (expect only 0x20)
}

// =============================================================================
//  VL53L5CX — address sequencing and initialisation
// =============================================================================
// The VL53L5CX boots at the default 8-bit address 0x52 (Wire 7-bit: 0x29 = 0x52 >> 1).
// When multiple sensors share one I²C bus we bring them up one at a time:
//   1. Assert XSHUT LOW on all sensors (put all into reset / bus-silent state).
//   2. Release XSHUT on sensor i (only sensor i is now active on the bus).
//   3. Communicate with sensor i at 0x29, reassign it a unique address.
//   4. Repeat for the next sensor.
// After all sensors have unique addresses we release all XSHUT lines; each
// sensor continues ranging independently at its assigned address.
void setI2CDistanceAddresses()
{
  DBG_TOFLN(F("ToF: starting VL53L5CX × 3 address sequencing"));

  // ── 1. Hold all sensors in reset ───────────────────────────────────────────
  mcp23018SetXSHUT(0, false);
  mcp23018SetXSHUT(1, false);
  mcp23018SetXSHUT(2, false);
  delay(XSHUT_RESET_DELAY_MS);

  // ── 2. I²C bus is already initialised in setup() — no re-init needed ──────────
  scanI2C();  // Should be empty — all sensors are in reset

  // ── 3. Bring up each sensor individually ───────────────────────────────────
  for (uint8_t i = 0; i < NUMBER_OF_DISTANCE_SENSORS; i++)
  {
    laser_working[i] = 0;

    // Release only sensor i; all others remain in reset so they cannot
    // respond to the default address 0x29 and cause an address collision.
    for (uint8_t j = 0; j < NUMBER_OF_DISTANCE_SENSORS; j++)
      mcp23018SetXSHUT(j, (j == i));

    delay(XSHUT_BOOT_DELAY_MS);  // Allow the newly released sensor to boot
    scanI2C();                   // Should show exactly one device at 0x29

    DBG_TOF(F("ToF[")); DBG_TOF(i); DBG_TOF(F("]: "));

    // Connect to the sensor at its boot-time 7-bit address (0x29 = datasheet 8-bit 0x52 >> 1)
    if (!tof[i].begin(0x29, Wire))
    {
      DBG_TOFLN(F("begin() failed — sensor absent or stuck"));
      continue;
    }

    // Reassign to a permanent unique address so the next sensor can use 0x29
    if (!tof[i].setAddress(LOX_ADDRESS[i]))
    {
      DBG_TOFLN(F("setAddress() failed"));
      continue;
    }

    DBG_TOF(F("addr → 0x")); DBG_TOFLN(LOX_ADDRESS[i], HEX);

#if DEBUG_TOF
    // Confirm the device acknowledges at its new address
    Wire.beginTransmission(LOX_ADDRESS[i]);
    Serial.print(F("  ACK check: "));
    Serial.println(Wire.endTransmission() == 0 ? F("OK") : F("FAIL"));
#endif

    // Configure ranging: 8×8 zone map, 10 Hz update rate
    tof[i].setResolution(64);       // 64 zones = 8×8 grid
    tof[i].setRangingFrequency(10); // Hz
    tof[i].startRanging();

    laser_working[i] = 1;
    DBG_TOFLN(F("  ready"));
  }

  // ── 4. Release all sensors ──────────────────────────────────────────────────
  // All sensors now have unique addresses and are actively ranging; releasing
  // all XSHUT lines is a no-op electrically but makes the intent explicit.
  mcp23018SetXSHUT(0, true);
  mcp23018SetXSHUT(1, true);
  mcp23018SetXSHUT(2, true);
}

// =============================================================================
//  VL53L5CX — data collection
// =============================================================================

// Extract a single representative distance from an 8×8 result frame.
// Averages the four central zones (indices 27, 28, 35, 36) to obtain a stable
// centre-of-field reading.  Returns 0 if no valid zone data is available.
static uint16_t centerDistanceMm(const VL53L5CX_ResultsData &r)
{
  const uint8_t idx[4] = {27, 28, 35, 36};
  uint32_t sum = 0;
  uint8_t  cnt = 0;
  for (uint8_t k = 0; k < 4; k++)
  {
    int16_t d = r.distance_mm[idx[k]];
    if (d > 0) { sum += (uint16_t)d; cnt++; }
  }
  return cnt ? (uint16_t)(sum / cnt) : 0;
}

// Poll all active ToF sensors and update the global distance arrays.
// Called once per main-loop tick at the configured loopRateMsec cadence.
void read_triple_sensors()
{
  for (uint8_t i = 0; i < NUMBER_OF_DISTANCE_SENSORS; i++)
  {
    if (!laser_working[i]) continue;
    laser_status[i] = 0;
    if (tof[i].isDataReady() && tof[i].getRangingData(&tofResults))
    {
      laser_status[i] = 1;
      laser_distance_millimeters[i] = centerDistanceMm(tofResults);

      // Keep the whole 8×8 frame so reportZones() can stream every zone.
      // Clamp invalid (<=0) zones to 0 so the column count stays fixed.
      for (uint8_t z = 0; z < 64; z++)
      {
        int16_t d = tofResults.distance_mm[z];
        zoneDistances[i][z] = (d > 0) ? (uint16_t)d : 0;
      }
    }
  }
}

// =============================================================================
//  Light control
// =============================================================================

// Forward-declare alarm IDs so deactivateLights() can cancel in-flight alarms.
static volatile alarm_id_t debouncingAlarmId = -1;  // debounce before enabling falling-edge ISR
static volatile alarm_id_t lightOffAlarmId   = -1;  // maximum light-on timeout
static volatile alarm_id_t guardRailAlarmId  = -1;  // guard-rail before re-arming rising-edge ISR
static volatile uint8_t    activeLightPin    = 0;   // GPIO pin currently driven HIGH (0 = none)
static volatile uint64_t   risingEdgeTimeUs  = 0;   // time_us_64() snapshot of the last rising edge
static volatile bool       waitingForFall    = false; // true after debounce, while falling edge is armed

// Schedule a light to be activated on the next exposure rising edge.
// lightIdx is 0-indexed (0–5 → lights 1–6).  Only updates the state variable;
// all GPIO control happens inside the exposure ISR and its timer callbacks.
void activateLight(uint8_t lightIdx)
{
  if (lightIdx < NUMBER_OF_LIGHTS)
  {
    lightOn         = lightIdx;
    lightToActivate = lightIdx + 1;  // ISR uses 1-indexed (0 = off)
  }
}

// Unconditionally cut all lights, cancel any running alarms, and re-arm the
// exposure ISR.  The only non-ISR path that drives the light GPIO pins.
void deactivateLights()
{
  if (debouncingAlarmId >= 0) { cancel_alarm(debouncingAlarmId); debouncingAlarmId = -1; }
  if (lightOffAlarmId   >= 0) { cancel_alarm(lightOffAlarmId);   lightOffAlarmId   = -1; }
  if (guardRailAlarmId  >= 0) { cancel_alarm(guardRailAlarmId);  guardRailAlarmId  = -1; }

  // Disable both edges before modifying shared state
  gpio_set_irq_enabled(CAMERA_EXPOSURE_PIN, GPIO_IRQ_EDGE_FALL, false);
  gpio_set_irq_enabled(CAMERA_EXPOSURE_PIN, GPIO_IRQ_EDGE_RISE, false);
  waitingForFall  = false;

  lightToActivate = 0;
  activeLightPin  = 0;
  lightOn         = NO_LIGHT_ON;

  for (uint8_t i = 0; i < NUMBER_OF_LIGHTS; i++)
    gpio_put(directLightControl[i], 0);

  gpio_set_irq_enabled(CAMERA_EXPOSURE_PIN, GPIO_IRQ_EDGE_RISE, true);
}

// =============================================================================
//  Automatic light selection based on ToF depth readings
// =============================================================================
#if ENABLE_LASER_LIGHT_DECISION
// Select the light whose hexagonal position best faces the closest surface.
// The three sensors are treated as vectors at 0°, 120° and 240° (every-other
// light position).  Each sensor's depth is converted to an inverse-distance
// weight and projected onto a 2-D unit circle.  The light whose angular
// position aligns most closely with the resulting normal vector is chosen.
uint8_t chooseClosestLight(const uint16_t *depths)
{
  float normalX = 0.0f, normalY = 0.0f;
  for (uint8_t i = 0; i < NUMBER_OF_DISTANCE_SENSORS; i++)
  {
    uint8_t lightIdx = i * 2;
    float   angle    = lightIdx * (float)M_PI / 3.0f;
    float   weight   = 1.0f / (float)(depths[i] + 1);
    normalX += cosf(angle) * weight;
    normalY += sinf(angle) * weight;
  }
  uint8_t best    = 0;
  float   bestDot = -1.0f;
  for (uint8_t i = 0; i < NUMBER_OF_LIGHTS; i++)
  {
    float angle  = i * (float)M_PI / 3.0f;
    float dotPrd = normalX * cosf(angle) + normalY * sinf(angle);
    if (dotPrd > bestDot) { bestDot = dotPrd; best = i; }
  }
  return best;
}
#endif

// Return the next light index given the current index and the active light mode.
//   mode 0/1 — simple round-robin sequence
//   mode 3   — alternating opposite pairs (0↔3, 1↔4, 2↔5)
uint8_t getNextLight(uint8_t current, uint8_t total, uint8_t mode)
{
  if (mode == 3)
  {
    const uint8_t alt[6] = {3, 4, 5, 1, 2, 0};
    if (current < 6) return alt[current];
  }
  return (current + 1) % total;
}

// =============================================================================
//  Serial / Ethernet output helpers
// =============================================================================
// The protocol sends one comma-separated line per light cycle:
//   timestamp, button1, button2, dist0, dist1, dist2, L0, L1, L2, L3, L4, L5
// Special distance tokens: F = sensor failed, H = out of range (>6000 mm), 0 = no new data

void comma()   { Serial.print(F(",")); if (client) client.print(F(",")); }
void zero()    { Serial.print(F("0")); if (client) client.print(F("0")); }
void one()     { Serial.print(F("1")); if (client) client.print(F("1")); }
void high_v()  { Serial.print(F("H")); if (client) client.print(F("H")); }

void number(int v)
{
  Serial.print(v);
#if USE_ETHERNET
  if (client) client.print(v);
#endif
}

void newline()
{
  Serial.print(F("\n"));
#if USE_ETHERNET
  if (client) client.print(F("\n"));
#endif
}

void flush()
{
  Serial.flush();
#if USE_ETHERNET
  if (client) client.flush();
#endif
}

void failed()
{
  Serial.print(F("F"));
#if USE_ETHERNET
  if (client) client.print(F("F"));
#endif
#if DEBUG_TOF
  Serial.println();
  Serial.println(F("[FAIL] sensor failure signalled"));
#endif
}

void reportState(unsigned long ts, unsigned int b1, unsigned int b2)
{
  Serial.print(ts);
#if USE_ETHERNET
  if (client) client.print(ts);
#endif
  comma();
  Serial.print(b1);
#if USE_ETHERNET
  if (client) client.print(b1);
#endif
  comma();
  Serial.print(b2);
#if USE_ETHERNET
  if (client) client.print(b2);
#endif
  comma();

  for (uint8_t i = 0; i < NUMBER_OF_DISTANCE_SENSORS; i++)
  {
    if      (!laser_working[i])              { failed(); }
    else if (laser_status[i] != 4)
    {
      int d = (int)laser_distance_millimeters[i];
      if (d > 6000) high_v(); else number(d);
    }
    else { zero(); }
    comma();
  }

  for (uint8_t i = 0; i < NUMBER_OF_LIGHTS; i++)
  {
    if (i != lightOn) zero(); else one();
    if (i != NUMBER_OF_LIGHTS - 1) comma();
  }
  newline();
  flush();
}

// Emit the full 8×8 ToF frames as three extra lines, one per sensor:
//   x1,<ts>,z0,z1,…,z63
//   x2,<ts>,z0,…,z63
//   x3,<ts>,z0,…,z63
// The leading "x" tag lets the host route these to a separate file while the
// compact reportState() line (which starts with a digit) is parsed as before.
// ts is the same device timestamp passed to reportState() for this frame, so
// the host can join the two streams row-for-row.
void reportZones(unsigned long ts)
{
  for (uint8_t s = 0; s < NUMBER_OF_DISTANCE_SENSORS; s++)
  {
    Serial.print(F("x")); Serial.print((int)(s + 1));
#if USE_ETHERNET
    if (client) { client.print(F("x")); client.print((int)(s + 1)); }
#endif
    comma();
    Serial.print(ts);
#if USE_ETHERNET
    if (client) client.print(ts);
#endif
    for (uint8_t z = 0; z < 64; z++)
    {
      comma();
      number((int)zoneDistances[s][z]);
    }
    newline();
  }
  flush();
}

// =============================================================================
//  Exposure ISR and timer callbacks
//
//  Cycle per camera frame (all times relative to the rising edge at t = 0):
//
//    1. exposureISR (RISE)   — t = 0 µs:
//         light ON, disable RISE, record risingEdgeTimeUs,
//         start debounce alarm (B) and max-on-time alarm (C).
//
//    2. debouncingCallback   — t = LightDebouncingMicroseconds (20 µs):
//         enable FALL ISR; the debounce window ensures the signal has settled.
//
//    3a. exposureISR (FALL)  — t = t_fall  (20 µs ≤ t_fall < lightTurnOffMicroseconds):
//         light OFF, disable FALL, cancel alarm C, start guard-rail alarm (D).
//
//    3b. lightOffCallback    — t = lightTurnOffMicroseconds (1000 µs), timeout path:
//         light OFF (if still on), disable FALL, start guard-rail alarm (D).
//
//    4. guardRailCallback    — t = LightKeepOffMicroseconds (15000 µs):
//         enable RISE ISR — cycle complete.
//
//  All alarm times use add_alarm_at() with absolute timestamps derived from
//  risingEdgeTimeUs, eliminating the systematic ISR-entry overhead that would
//  accumulate if relative add_alarm_in_us() calls were chained.
//
//  RP2350 (Cortex-M33 @ 150 MHz) timer resolution is 1 µs; ISR latency is
//  ~100 ns.  All microsecond-level deadlines in this design can be met with
//  sub-microsecond accuracy.
//
//  All functions are placed in SRAM (__not_in_flash_func) to eliminate XIP
//  flash-cache latency at interrupt time.
// =============================================================================

// Phase 4: guard-rail elapsed → re-enable the exposure rising-edge interrupt.
static int64_t __not_in_flash_func(guardRailCallback)(alarm_id_t, void *)
{
  guardRailAlarmId = -1;
  gpio_set_irq_enabled(CAMERA_EXPOSURE_PIN, GPIO_IRQ_EDGE_RISE, true);
  return 0;
}

// Phase 3b: maximum light-on time expired before the falling edge arrived.
// Extinguish the light, disable the (now obsolete) falling-edge interrupt,
// and start the guard-rail referenced to the original rising-edge timestamp.
static int64_t __not_in_flash_func(lightOffCallback)(alarm_id_t, void *)
{
  lightOffAlarmId = -1;
  gpio_set_irq_enabled(CAMERA_EXPOSURE_PIN, GPIO_IRQ_EDGE_FALL, false);
  waitingForFall = false;
  uint8_t pin = activeLightPin;
  if (pin) { gpio_put(pin, 0); activeLightPin = 0; }
  guardRailAlarmId = add_alarm_at(
      from_us_since_boot(risingEdgeTimeUs + LightKeepOffMicroseconds),
      guardRailCallback, nullptr, true);
  return 0;
}

// Phase 2: debounce window elapsed → arm falling-edge interrupt.
static int64_t __not_in_flash_func(debouncingCallback)(alarm_id_t, void *)
{
  debouncingAlarmId = -1;
  waitingForFall = true;
  gpio_set_irq_enabled(CAMERA_EXPOSURE_PIN, GPIO_IRQ_EDGE_FALL, true);
  return 0;
}

// Phase 1 / 3a: unified ISR for both rising and falling edges.
// waitingForFall distinguishes the two phases; RISE and FALL are never
// enabled simultaneously, so there is no ambiguity.
static void __not_in_flash_func(exposureISR)()
{
  if (!waitingForFall)
  {
    // ── Rising edge (Phase 1) ──────────────────────────────────────────────────
    uint8_t light = lightToActivate;
    if (light == 0) return;  // no light scheduled; leave ISR armed for next edge

    // Snapshot the rising-edge time before any other work so all subsequent
    // alarm_at() calls are corrected for the overhead of entering this ISR.
    risingEdgeTimeUs = time_us_64();

    gpio_set_irq_enabled(CAMERA_EXPOSURE_PIN, GPIO_IRQ_EDGE_RISE, false);

    uint8_t pin = directLightControl[light - 1];
    activeLightPin = pin;
    gpio_put(pin, 1);

    // Schedule debounce — re-arms ISR for falling edge after the signal settles
    debouncingAlarmId = add_alarm_at(
        from_us_since_boot(risingEdgeTimeUs + LightDebouncingMicroseconds),
        debouncingCallback, nullptr, true);

    // Schedule maximum light-on timeout (safety ceiling)
    lightOffAlarmId = add_alarm_at(
        from_us_since_boot(risingEdgeTimeUs + lightTurnOffMicroseconds),
        lightOffCallback, nullptr, true);
  }
  else
  {
    // ── Falling edge (Phase 3a) ───────────────────────────────────────────────
    gpio_set_irq_enabled(CAMERA_EXPOSURE_PIN, GPIO_IRQ_EDGE_FALL, false);
    waitingForFall = false;

    uint8_t pin = activeLightPin;
    if (pin) { gpio_put(pin, 0); activeLightPin = 0; }

    if (lightOffAlarmId >= 0) { cancel_alarm(lightOffAlarmId); lightOffAlarmId = -1; }

    // Guard-rail fires at the absolute time of (risingEdge + LightKeepOffMicroseconds)
    guardRailAlarmId = add_alarm_at(
        from_us_since_boot(risingEdgeTimeUs + LightKeepOffMicroseconds),
        guardRailCallback, nullptr, true);
  }
}

// =============================================================================
//  setup()
// =============================================================================
void setup()
{
  // ── Serial console ───────────────────────────────────────────────────────────
  Serial.begin(115200);
  // Wait up to 5 s for a USB serial connection (allows reading boot messages)
  unsigned long t0 = millis();
  while (!Serial && millis() - t0 < 5000) delay(10);
  delay(200);
  Serial.println(F("== CameraControllerMultizone boot =="));
  version();

  // ── Light output pins (GP6–GP11) ─────────────────────────────────────────────
  // Write LOW into the output latch BEFORE enabling the output driver so the pin
  // transitions directly from high-impedance to output-LOW without a HIGH glitch.
  for (uint8_t i = 0; i < NUMBER_OF_LIGHTS; i++)
  {
    digitalWrite(directLightControl[i], LOW);
    pinMode(directLightControl[i], OUTPUT);
  }

  // ── Camera exposure trigger input ─────────────────────────────────────────────
  // GP2 receives an active-high pulse from the XCG-CP510 for each shutter window.
  // The pin is configured as a plain input here; the rising-edge ISR is attached
  // at the very end of setup() after all other peripherals are ready.
  pinMode(CAMERA_EXPOSURE_PIN, INPUT);

  // ── Ethernet INT input ────────────────────────────────────────────────────────
  // The W5500 asserts GP22 LOW when it needs servicing.  The pull-up ensures the
  // line idles HIGH when no interrupt is pending.  Interrupt-driven RX is not yet
  // implemented; this pin is reserved for future use.
#if USE_ETHERNET
  pinMode(ETHERNET_INT_PIN, INPUT_PULLUP);
#endif

  // ── I²C bus + MCP23018 ───────────────────────────────────────────────────────
  // Wire must be configured before initMCP23018() because the function uses
  // Wire.beginTransmission() internally.  Wire is also used by the ToF sensors,
  // but we initialise it here (not inside setI2CDistanceAddresses) so the
  // MCP23018 — and specifically its XSHUT outputs — are ready before we attempt
  // the sensor address-sequencing procedure below.
  Wire.setSDA(SDA_PIN);
  Wire.setSCL(SCL_PIN);
  Wire.begin();
  // 1 MHz Fast-Mode Plus (FM+): the RP2350 I2C peripheral tops out at FM+ regardless
  // of device capability.  MCP23018 supports 3.4 MHz HS mode per its datasheet, but
  // HS mode requires a special master-code preamble that the RP2350 hardware does not
  // implement; FM+ at 1 MHz is therefore the highest speed achievable on this bus.
  // VL53L5CX also supports FM+.  Pull-ups should be ≤1 kΩ for reliable FM+ signalling.
  Wire.setClock(1000000);

#if USE_MCP23018
  // initMCP23018() performs a hardware reset, configures directions and pull-ups,
  // enables button interrupts, and drives all outputs to a safe initial state:
  //   GPA2 (ETH RST)  LOW  → W5500 held in reset until Ethernet.init()
  //   GPA3-5 (XSHUT)  LOW  → all ToF sensors held in reset
  //   GPA6-7 (rails)  LOW  → power rails off
  initMCP23018();
#endif

  // ── VL53L5CX time-of-flight sensors ──────────────────────────────────────────
  // setI2CDistanceAddresses() sequences each sensor's XSHUT line to bring sensors
  // up one at a time and assign unique I²C addresses (0x30, 0x31, 0x32).
  // At completion, all three sensors are ranging at 10 Hz.
#if USE_LASER
  DBG_TOFLN(F("ToF: initialising sensors..."));
  // With MCP23018: laserSwitch Pico pins are unused; sensor power is via XSHUT.
  // The Pico pins GP12/GP13/GP14 (legacy laserSwitch) are left as inputs (default)
  // to avoid driving against the open-drain MCP23018 outputs on the XSHUT net.
  setI2CDistanceAddresses();
  DBG_TOFLN(F("ToF: ready"));
#endif

  // ── Ethernet ─────────────────────────────────────────────────────────────────
  // Ethernet initialisation is intentionally last because:
  //   a) The W5500 RST is held LOW by MCP23018 GPA2 since initMCP23018() ran.
  //      We release it here after all other peripherals are stable.
  //   b) Ethernet.begin() is slow (~500 ms) and would delay the I²C setup if
  //      called earlier.
#if USE_ETHERNET
  #if DEBUG_ETHERNET
  Serial.println(F("ETH: initialising W5500..."));
  sram();
  #endif

  delay(500);  // Hold W5500 in reset a little longer while bus settles

  // Release W5500 from reset via MCP23018 GPA2.
  // The open-drain output stops pulling RST LOW; the external pull-up on the RST
  // line brings it HIGH and the W5500 begins its internal initialisation sequence.
  mcp23018SetEthReset(true);
  delay(500);  // Allow W5500 PLL and PHY to come up (~150 ms typical, 500 ms safe)

  // Select the correct SPI chip-select pin and assign a static IP address.
  // Ethernet.init() must be called before Ethernet.begin() on all non-standard
  // CS pins; it stores the pin number in the library for all subsequent SPI ops.
  Ethernet.init(ETHERNET_CS_PIN);
  Ethernet.begin(mac, ip);
  delay(100);

  #if DEBUG_ETHERNET
  Serial.print(F("ETH IP: ")); Serial.println(Ethernet.localIP());
  Serial.print(F("ETH HW: ")); Serial.println(Ethernet.hardwareStatus());
  if (Ethernet.hardwareStatus() == 0) Serial.println(F("ETH: hardware not detected!"));
  sram();
  #endif

  // Open the telnet-style command server on port 23.
  // Clients connect with: nc <ip> 23
  server.begin();
  Serial.print(F("ETH: server listening on ")); Serial.println(ip);
#endif

  // ── Exposure ISR ─────────────────────────────────────────────────────────────
  // Timing parameters must be set before the ISR is armed so the first callback
  // sees valid values.  All times are in µs from the rising edge.
  LightDebouncingMicroseconds = 20;     // µs: debounce before arming falling-edge ISR
  lightTurnOffMicroseconds    = 1000;   // µs: maximum light-on window from rising edge
  LightKeepOffMicroseconds    = 15000;  // µs: guard-rail before rising-edge ISR re-arms

  // Register the unified ISR for RISING.  The same callback is also invoked for
  // FALLING when debouncingCallback later enables GPIO_IRQ_EDGE_FALL via
  // gpio_set_irq_enabled(); the pico-sdk dispatches both event types to the
  // registered handler without requiring a separate attachInterrupt() call.
  attachInterrupt(digitalPinToInterrupt(CAMERA_EXPOSURE_PIN), exposureISR, RISING);

  // Raise IO_IRQ_BANK0 (all GPIO bank-0 interrupts) to the highest available
  // NVIC priority on the Cortex-M33 (RP2350).  Priority 0 cannot be preempted
  // by any other maskable interrupt on the same core, giving the exposure ISR
  // the minimum possible response latency.  delay() and delayMicroseconds() on
  // RP2350 use busy-wait loops that do not disable interrupts, so this ISR will
  // preempt them as well.
  irq_set_priority(IO_IRQ_BANK0, PICO_HIGHEST_IRQ_PRIORITY);

  Serial.println(F("== boot complete =="));
}

// =============================================================================
//  loop()
// =============================================================================
void loop()
{
  unsigned long currentTime = millis();

  // Light activation and deactivation are fully handled by exposureISR() and its
  // timer callbacks (lightOffCallback / guardRailCallback).  The main loop only
  // updates lightToActivate via activateLight() / deactivateLights().

  // ── Button read via MCP23018 ──────────────────────────────────────────────────
  // GPA0/GPA1 use internal pull-ups; a button press pulls the pin LOW.
  // We read the whole Port A GPIO register in a single I²C transaction and
  // extract the two button bits.  1 = pressed, 0 = not pressed.
  uint8_t mcpGpioA = mcp23018_present ? mcp23018ReadReg(MCP23018_GPIOA) : 0xFF;
  unsigned int buttonRaw1 = (mcpGpioA & (1u << MCP_GPA_PB0)) ? 0 : 1;
  unsigned int buttonRaw2 = (mcpGpioA & (1u << MCP_GPA_PB1)) ? 0 : 1;

  // ── Ethernet client management ───────────────────────────────────────────────
  char receivedChar = 0;
#if USE_ETHERNET
  // Drop stale connections (client disconnected without sending FIN)
  if (client && !client.connected())
  {
    client.stop();
    client = EthernetClient();
  }
  // Accept a new client if no one is connected
  if (!client)
  {
    EthernetClient nc = server.available();
    if (nc) { client = nc; client.flush(); Serial.println(F("ETH: client connected")); }
  }
  // Read one command byte from the Ethernet client if available
  if (client && client.available() > 0) receivedChar = client.read();
#endif

  // Read one command byte from the USB serial port if available.
  // If both serial and ethernet produce a byte in the same tick, serial wins
  // (it overwrites receivedChar) — acceptable since commands are rare.
  if (Serial.available() > 0) receivedChar = Serial.read();

  // ── Command handler ───────────────────────────────────────────────────────────
  // Single-character protocol over serial or telnet:
  //   v        — print firmware version
  //   h / i    — reset timer, enable output, start sequential light cycling
  //   0        — all lights off, stop auto cycle
  //   1–6      — activate specific light (manual mode)
  //   r        — sequential mode from light 0
  //   a        — depth-guided auto mode (uses ToF readings)
  //   t        — alternating-pair mode from light 0
  //   y        — timed flash mode (cycles for 10 s then stops)
  //   +        — advance one light step (manual stepping)
  //   o        — reset maximum light-on time to default (1000 µs)
  //   p        — extend maximum light-on time by 500 µs
  //   f        — disable serial output stream
  //   z        — all off + software reset
  //   x        — print Ethernet IP and hardware status
  if (receivedChar != 0)
  {
    switch (receivedChar)
    {
      case 'v': version(); break;

      case 'h':
      case 'i':
        reset_millis();
        serialOutputEnabled = true;
        currentTime         = 0;
        lightStartTime      = 0;
        autoLights          = 1;
        lightOn             = 0;
        lightToActivate     = 0;  // auto-cycle will set this on the first tick
        break;

      case '0': deactivateLights(); autoLights = 0;                    break;
      case '1': activateLight(0);   autoLights = 0;                    break;
      case '2': activateLight(1);   autoLights = 0;                    break;
      case '3': activateLight(2);   autoLights = 0;                    break;
      case '4': activateLight(3);   autoLights = 0;                    break;
      case '5': activateLight(4);   autoLights = 0;                    break;
      case '6': activateLight(5);   autoLights = 0;                    break;

      case 'r': autoLights = 1; lightOn = 0;                           break;
      case 'a': autoLights = 2;                                        break;
      case 't': autoLights = 3; lightOn = 0;                           break;
      case 'y': autoLights = 4; lightOn = 0;                           break;

      case 'o': lightTurnOffMicroseconds  = 1000;  break;  // reset to default max light-on time
      case 'p': lightTurnOffMicroseconds += 500;  break;  // extend max light-on time

      case 'm': emitAllZones = true;  break;  // stream full 8×8 zone frames (x1/x2/x3 lines)
      case 'c': emitAllZones = false; break;  // compact only — suppress zone frames

      case 'f': serialOutputEnabled = false;           break;
      case 'z': deactivateLights(); autoLights = 0; resetFunc(); break;

      case '+':
        autoLights = 0;
        lightOn    = getNextLight(lightOn, NUMBER_OF_LIGHTS, autoLights);
        activateLight(lightOn);
        lightStartTime = currentTime;
        reportState(currentTime, buttonRaw1, buttonRaw2);
        if (emitAllZones) reportZones(currentTime);
        break;

#if USE_ETHERNET
      case 'x':
        Serial.print(F("ETH IP: ")); Serial.println(Ethernet.localIP());
        Serial.print(F("ETH HW: ")); Serial.println(Ethernet.hardwareStatus());
        sram();
        break;
#endif
    }
  }

  // ── Timed main loop ───────────────────────────────────────────────────────────
  // Everything below runs at the loopRateMsec cadence (default 10 ms = 100 Hz).
  if (currentTime - lastUpdateTime >= loopRateMsec)
  {
    lastUpdateTime = currentTime;

    // Poll all ToF sensors for new distance data
#if USE_LASER
    read_triple_sensors();
#endif

    // Automatic light cycling and periodic state reporting
    if (autoLights != 0 && (currentTime - lightStartTime >= lightDurationMsec))
    {
      lightStartTime = currentTime;

      if (autoLights == 1 || autoLights >= 3)
      {
        lightOn = getNextLight(lightOn, NUMBER_OF_LIGHTS, autoLights);
        activateLight(lightOn);
        // Mode 4: timed flash — cycle for 10 s then stop automatically
        if (autoLights == 4 && currentTime > 10000)
        {
          autoLights = 0;
          deactivateLights();
        }
      }
#if ENABLE_LASER_LIGHT_DECISION
      else if (autoLights == 2)
      {
        lightOn = chooseClosestLight(laser_distance_millimeters);
        activateLight(lightOn);
      }
#endif

      if (serialOutputEnabled)
      {
        reportState(currentTime, buttonRaw1, buttonRaw2);
        if (emitAllZones) reportZones(currentTime);
      }
    }
  }

  // ── Yield / sleep ─────────────────────────────────────────────────────────────
  // Brief pause to avoid hammering the I²C bus and the W5500.
  // Strobe timing is now ISR-driven, so no compensation is needed here.
  // The exposure ISR can preempt this delay at any time.
  delayMicroseconds(CPU_SLEEP_US);
}
