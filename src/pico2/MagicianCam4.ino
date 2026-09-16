// =============================================================================
//  CameraControllerMultizone — Raspberry Pi Pico 2 (RP2350)
//  HW version: MagicianCam4 — Rev. 1.3
//  SW version: 1.65
//
//  Release 1.5 notes:
//    - Added boot-state aware VL53L5CX initialization.
//    - Supports cold default-address startup, warm assigned-address attach,
//      and LPn-isolated retained-address recovery.
//    - Reworked software reset command 'z' to use watchdog-based reboot.
//    - Added independent LED boot markers:
//        3 short blinks = setup entered
//        2 long blinks  = setup completed
//    - Reduced VL53L5CX debug output to operationally relevant diagnostics.
//
//  Release 1.6 notes:
//    - VL53L5CX range data acquired via ISR (ToF INT -> GP12/GP13/GP14) instead
//      of I2C polling, removing useless bus transactions every 20 ms.
//    - PB0/PB1 acquired via the MCP23018 INTA signal with a state-machine
//      debouncer, instead of polling GPIOA every tick.
//
//  Release 1.63 notes — merge of the 1.33 development thread into 1.6:
//    - Per-COB thermal budget (leaky bucket) enforced in the exposure ISR.
//    - Hard pulse-width / period limits with clampLightTiming() on every write.
//    - Light safety watchdog running unconditionally on every loop tick.
//    - Strobe accounting (strobeCounter / strobeLight) reported to the host.
//    - Host-uploaded strobe schedule and exposure-locked stepping mode.
//    - autoLights (who steps the light) separated from lightPattern (what order).
//    - Newline-terminated multi-character command protocol; both transports are
//      drained fully every tick instead of one byte each.
//    - Boots DARK: no COB is armed until the host completes its startup
//      handshake. See the BOOT DARK block in setup().
//  Nothing of the 1.5/1.6 ISR architecture was changed by this merge: the ToF
//  and push-button interrupt paths are untouched.
//
//  ##  SAFETY-CRITICAL NOTE — READ BEFORE MODIFYING LIGHT CONTROL  ##
//  The LED COBs on GP6–GP11 are driven ABOVE their rated voltage. They survive
//  only because they are strobed: every pulse is bounded in width, and the
//  average power per COB is bounded by how rarely that COB is selected.
//  Two independent limits must therefore hold at all times:
//
//    (a) PULSE WIDTH   — no single ON period may exceed LIGHT_HARD_MAX_ON_US.
//    (b) DUTY CYCLE    — the long-run ON fraction of EACH INDIVIDUAL COB must
//                        stay under 1/(2^LIGHT_DUTY_SHIFT).
//
//  (a) was already enforced via lightOffCallback. (b) was NOT: it held only
//  implicitly, because round-robin sequencing spread the strobes over six COBs,
//  giving each one a sixth of the load. Host-driven adaptive light selection can
//  legitimately ask for the SAME COB on every exposure — a 6x increase in per-COB
//  duty that would destroy an overvolted COB. (b) is therefore now enforced
//  explicitly and unconditionally, per COB, inside the exposure ISR (see
//  lightBudget*), where no host message, command, or selection policy can
//  bypass it.
//
//  When the requested COB is out of thermal budget the ISR does NOT skip the
//  frame — it SUBSTITUTES the COB with the most remaining headroom, so the frame
//  is still illuminated and the dataset has no holes. The COB that actually fired
//  is latched into strobeLight and reported to the host every strobe, so the host
//  always records ground truth rather than what it asked for. Under sustained
//  single-COB demand this degrades gracefully into a self-balancing round-robin
//  across the coolest COBs, which is exactly the thermally correct behaviour.
//
//  Release 1.64 notes:
//    - Added explicit SPI0 initialization for the W5500 Ethernet controller
//      before Ethernet.init():
//          SPI.setRX(16);
//          SPI.setTX(19);
//          SPI.setSCK(18);
//          SPI.begin();
//      This permanently fixes W5500 detection and bring-up reliability on
//      MAGICIAN Cam HW Rev. 1.3, including both cold and warm boot conditions.
//
//    - Added bounded command-drain policy for both Ethernet and Serial command
//      paths:
//
//          MAX_CLIENT_BYTES_PER_LOOP = 4
//          MAX_SERIAL_BYTES_PER_LOOP = 4
//
//      Previously, large TCP or Serial bursts could be drained entirely within
//      a single loop() iteration, causing excessive loop monopolization.
//      Diagnostic testing demonstrated worst-case loop gaps exceeding 1 second
//      when hundreds of '+' commands were processed consecutively.
//
//      The command-drain logic is now intentionally bounded so that no command
//      source can monopolize the main loop for an unbounded amount of time.
//      This improves timing predictability and overall real-time behaviour
//      without changing command protocol semantics.
//
//    - Validation results:
//
//          Ethernet management        < 1 ms
//          reportState()              ~4.3 ms worst case
//          ToF processing             ~55 ms worst case
//
//      After bounded drain was introduced, extreme >1 s loop gaps disappeared
//      and the maximum observed loop latency returned to being dominated by the
//      VL53L5CX ranging-data acquisition path.
//
//    - The motivation for this change is real-time scheduling hygiene and
//      bounded execution time, not CPU performance limitations.
//
//    - No changes were made to:
//          VL53L5CX ISR architecture
//          exposure ISR
//          light timing logic
//          MCP23018 button handling
//          application command protocol
//
//      Functional behaviour remains unchanged; only command-consumption
//      scheduling has been improved.
//
//  Release 1.65 notes:
//    - Explicit light-select commands '1'..'6' now emit a reportState() line,
//      exactly like '+'. Previously they were silent, and because they also set
//      autoLights = STEP_EXTERNAL they stopped the timed auto-report that 'i'
//      had started. A host stepping by digit (magician_grabber with --skip and
//      no --skipadvance, or its legacy polarization driver) therefore received
//      no distances, buttons or strobe ground truth after its first frame, and
//      its reported Arduino rate decayed as N/t.
//
//    - Cost per digit is now the same as per '+' (one report, ~4.3 ms worst
//      case). Like '+', the report is not gated by serialOutputEnabled.
//
//    - '0' (all lights off) remains silent.
//
//  Hardware summary
//  ─────────────────────────────────────────────────────────────────────────
//  I²C bus (GP4 SDA / GP5 SCL)
//    · 3× VL53L5CX  time-of-flight sensors  (datasheet 8-bit addr 0x52; Wire 7-bit addr 0x29 = 0x52 >> 1)
//      Each sensor is LPn-gated via MCP23018 GPA5/4/3 and reassigned a
//      unique address (0x30, 0x31, 0x32) during init.
//    · MCP23018 I²C port expander            (7-bit addr 0x20)
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
//    GP20 — MCP23018 INTA original board route, currently broken/high-Z on the Pico 2 board
//    GP21 — MCP23018 INTB (active-low input, push-pull driven)
//    GP22 — W5500INT (active-low input, ext. pull-up of 4.7K in the W5500 Lite board)
//    GP28 — MCP23018 INTA redirected via Y connection, active-low input, push-pull driven
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

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Ethernet.h>
#include <Adafruit_VL53L5CX.h>
#include "pico/time.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"

// =============================================================================
//  Feature flags
// =============================================================================
#define USE_VL53L5CX                1   // Enable VL53L5CX time-of-flight sensors
#define USE_W5500                   1   // Enable W5500 Lite Ethernet plug-in board
#define ENABLE_LASER_LIGHT_DECISION 1   // Enable depth-based automatic light selection

#define DEBUG_VL53L5CX              1   // Verbose VL53L5CX sensor init/scan output
#define DEBUG_W5500                 0   // Verbose Ethernet init output
// Note that Enabling/Disabling MCP23018 I²C port expander is not permitted,
// since this I/O Expander is a core component, always present in the system,
// and mandatory, even during debug & testing sessions

// =============================================================================
//  Version
// =============================================================================
#define VERSION_MAJOR 1
#define VERSION_MINOR 65

// =============================================================================
//  Debug macros
// =============================================================================
#if DEBUG_VL53L5CX
  #define DBG_VL53L5CX(...)   Serial.print(__VA_ARGS__)
  #define DBG_VL53L5CXLN(...) Serial.println(__VA_ARGS__)
#else
  #define DBG_VL53L5CX(...)
  #define DBG_VL53L5CXLN(...)
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
#define MCP23018_RESET_PIN 1     // GP3  — push-pull — active-low hardware reset --> 3 std, 1 dbg for the CRF board
#define MCP23018_INTA_PIN  28    // GP28 — MCP23018 INTA via Y connection; GP20 board path is broken/high-Z
#define MCP23018_INTB_PIN  21    // GP21 — push-pull — active-low interrupt B

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
#define MCP23018_GPPUA    0x0C  // Port A pull-up enable (both for input and for output pins)
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
#define MCP_GPA_PB0      0  // Push Button 0      — active-low input,  int. pull-up (LOW=pressed)
#define MCP_GPA_PB1      1  // Push Button 1      — active-low input,  int. pull-up (LOW=pressed)
#define MCP_GPA_W5500RST 2  // W5500 Reset        — active-low output, open-drain, ext. pull-up
#define MCP_GPA_LPN2     3  // VL53L5CX LPn 2     — active-low output, open-drain, ext. pull-up
#define MCP_GPA_LPN1     4  // VL53L5CX LPn 1     — active-low output, open-drain, ext. pull-up
#define MCP_GPA_LPN0     5  // VL53L5CX LPn 0     — active-low output, open-drain, ext. pull-up
#define MCP_GPA_RAIL5V   6  // 5 V  rail enable   — active-low output, open-drain, ext. pull-down
#define MCP_GPA_RAIL11V  7  // 11.5 V rail enable — active-low output, open-drain, ext. pull-down

// =============================================================================
//  MCP23018 Push Button interrupt/debounce configuration
// =============================================================================

#define PB_MASK             0x03  // GPA0/GPA1 only
#define PB_TICK_PERSISTENCE 4     // 4 ticks at loopRateMsec=20 ms -> about 80 ms

// =============================================================================
//  VL53L5CX time-of-flight sensors
// =============================================================================
#define NUMBER_OF_DISTANCE_SENSORS 3

// Mapping for the current system:
const uint8_t sensorLpnBit[3] = {
  MCP_GPA_LPN0,
  MCP_GPA_LPN1,
  MCP_GPA_LPN2
};

// Unique 7-bit I²C addresses assigned to each sensor after reset sequencing.
// The VL53L5CX datasheet quotes the default I²C address as 0x52 in 8-bit notation,
// corresponding to the std. 7-bit address: 0x52 >> 1 = 0x29.
// After sequencing, all three sensors are reassigned unique 7-bit addresses.
#define LOX1_ADDRESS 0x30
#define LOX2_ADDRESS 0x31
#define LOX3_ADDRESS 0x32
const uint8_t LOX_ADDRESS[NUMBER_OF_DISTANCE_SENSORS] = {LOX1_ADDRESS, LOX2_ADDRESS, LOX3_ADDRESS};

// Direct VL53L5CX INT pins to RP2350.
// These pins are used only for short data-ready interrupt pulses.
// Interrupts will be attached later, after ToF boot, startRanging()
// and safe-first-frame clearing.
#define SENSOR0_INT_PIN 12
#define SENSOR1_INT_PIN 13
#define SENSOR2_INT_PIN 14

const uint8_t sensorIntPins[NUMBER_OF_DISTANCE_SENSORS] = {
  SENSOR0_INT_PIN,
  SENSOR1_INT_PIN,
  SENSOR2_INT_PIN
};

// =============================================================================
//  VL53L5CX boot-path classification types
// =============================================================================

typedef enum {
  TOF_BUS_COLD_SILENT,
  TOF_BUS_WARM_ASSIGNED,
  TOF_BUS_MIXED_OR_AMBIGUOUS
} ToFBusPath;

typedef struct {
  uint8_t ackDefault;
  uint8_t ackAssigned[NUMBER_OF_DISTANCE_SENSORS];
} ToFBusSnapshot;

// MCP23018 HW reset cycle times, at the initialization stage
#define MCP23018_RST_SLEEP_MS 2    // Time to hold the MCP23018 IC in reset state
#define MCP23018_RST_RESUME_MS 2   // Time to allow the MCP23018 IC to resume after reset

// VL53L5CX Low Power mode cycle times, at the initialization stage
// Watch out. There is no way to reset the VL53L5CX sensors, other than cycling the power
// LPn signals actually detach from the I2C bus and put them into a "Low Power" state
#define VL53L5CX_LPN_SLEEP_MS 2    // Time to hold all sensors in LP mode before sequencing
#define VL53L5CX_LPN_RESUME_MS 2   // Time to allow a sensor to resume after the Low Power mode

// Safe-first-frame timeout.
// After startRanging(), the first available frame is explicitly read and discarded
// before ToF data-ready interrupts are attached. This prevents stale/startup data
// from entering the application path.
#define VL53L5CX_FIRST_FRAME_TIMEOUT_MS 5000

// W5500 HW reset cycle times, at the initialization stage
#define W5500_RST_SLEEP_MS 2       // Time to hold the W5500 Lite board in reset state
#define W5500_RST_RESUME_MS 2      // Time to allow the W5500 Lite board to resume after reset


// =============================================================================
//  Light control parameters
// =============================================================================
#define NUMBER_OF_LIGHTS 6
#define NO_LIGHT_ON      16  // Sentinel — no light currently active

// Direct GPIO pins for lights 1–6 (GP6–GP11)
const uint8_t directLightControl[NUMBER_OF_LIGHTS] = {6, 7, 8, 9, 10, 11};

// Exposure-ISR timing parameters — written only by the command handler, read by ISRs.
// All times are measured from the rising edge of the camera exposure signal (t = 0).
#define LIGHT_DEBOUNCING_US 20      // delay before arming falling-edge ISR.
#define LIGHT_TURNOFF_US    1000    // maximum light-on window from rising edge.
#define LIGHT_KEEPOFF_US    15000   // guard-rail before rising-edge ISR re-arms.

// ─────────────────────────────────────────────────────────────────────────────
//  HARD LIMITS — the COBs are overvolted; exceeding these destroys them.
//  These are compile-time constants on purpose. Nothing at runtime may raise
//  them: not a serial command, not an Ethernet message, not a light policy.
//  Every runtime write to the timing variables goes through clampLightTiming().
// ─────────────────────────────────────────────────────────────────────────────
#define LIGHT_HARD_MAX_ON_US     1000   // absolute ceiling on ANY single pulse (µs)
#define LIGHT_HARD_MIN_PERIOD_US 15000  // absolute floor on rise→rise spacing (µs)

// Per-COB sustained duty limit, as a right-shift: duty = 1/(2^LIGHT_DUTY_SHIFT).
// Shift 7 = 1/128 = 0.781%.
//
// Sizing rationale, at the default 650 µs exposure:
//
//   0.108%  round-robin, 10 fps
//   0.217%  round-robin + dual pulses, 10 fps  <-- FIELD-DEMONSTRATED SAFE.
//           An earlier late-'+' bug held the same COB across two consecutive
//           exposures, doubling its share of the cycle, and the COBs tolerated
//           that indefinitely. It is the only per-COB duty we have real evidence
//           for, so the limits below are quoted as multiples of it.
//   0.238%  round-robin, 22 fps
//   0.357%  adaptive worst case, 22 fps. The host's schedule builder caps any one
//           COB at 25% of strobes (6 base slots + 2 bonus dwell blocks = 24 slots,
//           worst COB takes 6), so this is the most an adaptive policy can ask for.
//   0.781%  THIS LIMIT — 2.2x the adaptive worst case, 3.6x demonstrated-safe.
//   1.430%  one COB pinned to 100% of strobes at 22 fps. Deliberately ABOVE the
//           limit: nothing in the host can request it, and if something ever did,
//           that is precisely the case this backstop exists to refuse.
//
// >> If the COB datasheet and your actual drive voltage imply a different <<
// >> sustained duty, THIS is the single number to change.                 <<
#define LIGHT_DUTY_SHIFT 7

// Burst allowance: how much ON-time a COB may bank while idle. Caps how many
// back-to-back max-length pulses a single COB can take before it is rate-limited
// down to the sustained duty above.
#define LIGHT_BUDGET_CAPACITY_US (8u * LIGHT_HARD_MAX_ON_US)

volatile uint32_t lightDebouncingMicroseconds = LIGHT_DEBOUNCING_US;
volatile uint32_t lightTurnOffMicroseconds = LIGHT_TURNOFF_US;
volatile uint32_t lightKeepOffMicroseconds = LIGHT_KEEPOFF_US;

// ── Per-COB thermal budget (leaky bucket, units = µs of permitted ON time) ────
// Refills at 1/(2^LIGHT_DUTY_SHIFT) of elapsed real time, saturating at
// LIGHT_BUDGET_CAPACITY_US. Charged the full worst-case pulse width at each
// rising edge (never refunded on early turn-off — deliberately conservative).
// Written only by the exposure ISR and by resetLightBudgets() before the ISR is armed.
static volatile uint32_t lightBudgetUs[NUMBER_OF_LIGHTS];
static volatile uint64_t lightBudgetStampUs[NUMBER_OF_LIGHTS];

// Diagnostics — reported to the host so a degrading thermal situation is visible
// in the recorded data rather than silent.
static volatile uint32_t lightSubstitutions = 0;  // requested COB was too hot, another fired
static volatile uint32_t lightStarvedFrames = 0;  // no COB had budget; frame went dark
static volatile uint32_t lightWatchdogTrips = 0;  // pin found stuck HIGH past the deadline

// =============================================================================
//  Camera exposure trigger — XCG-CP510
// =============================================================================
// GP2 goes HIGH during the camera shutter open window (active-high).
// The sketch turns the currently selected light on at the rising edge and off
// at the falling edge so illumination is perfectly gated to exposure time.
#define CAMERA_EXPOSURE_PIN 2

// =============================================================================
//  W5500 Lite plug-in board — Ethernet SPI Architecture
// =============================================================================
// SPI0: MISO=GP16, MOSI=GP19, SCLK=GP18, CS=GP17, INT=GP22
// Hardware reset is managed by MCP23018 GPA2
#define W5500_CS_PIN  17   // GP17
#define W5500_INT_PIN 22   // GP22, active-low interrupt from W5500

// Network configuration — adjusted to suit the MAGICIAN deployment
byte     mac[]     = {0x02, 0xAB, 0xCD, 0x12, 0x34, 0x56};
IPAddress ip(192, 168, 137, 64);
byte     gateway[] = {192, 168, 137, 1};
byte    *dns       = gateway;
byte     subnet[]  = {255, 255, 255, 0};

EthernetServer server(23);   // Telnet-style command server on port 23
EthernetClient client;

// =============================================================================
//  VL53L5CX sensor objects
// =============================================================================
Adafruit_VL53L5CX VL53L5CXSensor[NUMBER_OF_DISTANCE_SENSORS];
VL53L5CX_ResultsData VL53L5CXResults;  // Single results buffer, reused across all sensors
uint8_t  laser_working[NUMBER_OF_DISTANCE_SENSORS] = {0};   // 1 if sensor initialised OK
uint8_t  laser_status[NUMBER_OF_DISTANCE_SENSORS] = {0};   // 1 if recent valid data are available
uint16_t laser_distance_millimeters[NUMBER_OF_DISTANCE_SENSORS] = {0};
unsigned long laser_last_update_ms[NUMBER_OF_DISTANCE_SENSORS] = {0};
#define TOF_DATA_STALE_MS 500

// ISR-to-loop communication for VL53L5CX data-ready events.
// Each VL53L5CX INT pulse is captured by a minimal ISR.
// I2C data retrieval is performed later in the main loop.
volatile bool dataReadyFlags[NUMBER_OF_DISTANCE_SENSORS] = {false, false, false};


// =============================================================================
//  MCP23018 state
// =============================================================================
static bool    mcp23018_present    = false;
static uint8_t mcp23018_portA_olata = 0x3C;  // Software mirror of OLATA — avoids an extra I²C read
                                             // on every bit-set operation — 0b00111100
static uint8_t mcp23018_portA_gppua = 0x03;  // Software mirror of GPPUA — same as above
                                             // on every bit-set operation — 0b00000011

// MCP23018 PORTA interrupt flag.
// Set only by ISR, consumed only by the debounce state machine.
volatile bool mcp23018_PortAIntFlg = false;

static uint8_t prevPortA = 0xFF;
static uint8_t currPortA = 0xFF;

// Debounced application button states.
// false = released/open
// true  = pressed/closed to GND
bool debouncedPB0 = false;
bool debouncedPB1 = false;

// Debouncing state machine.
// 0 = Idle
// 1..PB_TICK_PERSISTENCE = active debounce stabilization states
int pbDebounceState = 0;


// =============================================================================
//  Global Object Registries
// =============================================================================
#define CPU_SLEEP_US 1000   // Microseconds to sleep at the end of each loop tick

// -----------------------------------------------------------------------------
// Command-drain limits
// -----------------------------------------------------------------------------
// Consume only a bounded number of command bytes per loop iteration.
// This prevents large Serial/TCP bursts from monopolising loop() for
// hundreds of milliseconds or more.
//
// Validated on Test Code, 2026-08-07:
//   - previous unbounded TCP drain could process 255 '+' commands in one loop,
//     producing >1 s loop gap;
//   - bounded drain at 4 bytes/loop limits command processing while preserving
//     normal Serial/TCP command behaviour.

#define MAX_CLIENT_BYTES_PER_LOOP 4
#define MAX_SERIAL_BYTES_PER_LOOP 4

bool         serialOutputEnabled  = false;
unsigned int loopRateMsec         = 20;   // 2026-06-02, formerly 10, unachievable - Main loop cadence (20 ms = 50 Hz)
unsigned int lightDurationMsec    = 100;  // How long each light stays on in auto modes
static volatile bool softwareResetRequested = false;

// lightToActivate: 0 = no light, 1–6 = light to illuminate on next exposure rising edge.
// Written by activateLight() / deactivateLights() in the main context; read in exposureISR().
volatile uint8_t lightToActivate = 0;

// ── Light stepping mode ──────────────────────────────────────────────────────
// autoLights says WHO advances the light; lightPattern says WHAT order it walks.
// Earlier revisions conflated the two: 'case +' set autoLights=0 and then passed
// that same variable to getNextLight() as the pattern, so host-stepped capture was
// always plain round-robin and the 't'/'a' modes were silently dead. They are
// separate now. The numeric values of STEP_* are unchanged from the old autoLights
// encoding, so an existing host that only sends h/r/a/y keeps working.
#define STEP_EXTERNAL 0  // advanced only by an explicit host '+' (legacy path)
#define STEP_TIMER    1  // advanced by the main loop every lightDurationMsec
#define STEP_DEPTH    2  // re-evaluated from ToF depth every lightDurationMsec
#define STEP_TIMED    4  // like STEP_TIMER but self-stops after 10 s
#define STEP_EXPOSURE 5  // advanced by the exposure ISR itself — one step per frame

#define PATTERN_ROUNDROBIN 0  // 0,1,2,3,4,5
#define PATTERN_OPPOSITE   3  // 0,3,1,4,2,5 — always jump to the opposing COB

unsigned char autoLights   = STEP_EXTERNAL;
unsigned char lightPattern = PATTERN_ROUNDROBIN;
unsigned char lightOn      = NO_LIGHT_ON; // Logically selected light index, 0-indexed (NO_LIGHT_ON=none)

// ── Host-uploaded strobe schedule (exposure-locked mode) ─────────────────────
// The host uploads an explicit sequence of COB indices; the exposure ISR walks it
// one entry per frame. A schedule is applied only at a cycle boundary, so an
// upload that lands late costs nothing — unlike a per-frame '+', where a late byte
// corrupted the frame↔light mapping. Written by the command handler, read by the
// ISR; scheduleLen is published last so the ISR never sees a partial schedule
// (single-core, and the ISR cannot preempt a byte store mid-way).
#define MAX_SCHEDULE_LEN 32
static volatile uint8_t  lightSchedule[MAX_SCHEDULE_LEN] = {0, 1, 2, 3, 4, 5};
static volatile uint8_t  lightScheduleLen    = NUMBER_OF_LIGHTS;
static volatile uint8_t  lightScheduleCursor = 0;

// ── Strobe accounting — the host's ground-truth join key ─────────────────────
// strobeCounter increments once per actual light pulse, inside the rising-edge
// ISR. strobeLight is the COB that genuinely fired (post thermal substitution),
// not the one that was requested. Reporting these lets the host map frame N to a
// COB by counting strobes instead of correlating two drifting clocks across a
// ~45 ms GigE pipeline delay.
static volatile uint32_t strobeCounter = 0;
static volatile uint8_t  strobeLight   = NO_LIGHT_ON;
static uint32_t          lastReportedStrobe = 0;

unsigned long lastUpdateTime = 0;
unsigned long lightStartTime = 0;
unsigned long flashStartTime = 0;  // millis() snapshot when timed-flash mode ('y') was armed


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
#if USE_W5500
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
//  I²C scan — VL53L5CX diagnostics only (DEBUG_VL53L5CX only)
//
//  Scans only the VL53L5CX-related addresses:
//
//    0x29             — default boot address
//    0x30, 0x31, 0x32 — runtime addresses after reassignment
//
//  The MCP23018 address (0x20) is intentionally excluded because the
//  port expander has a dedicated initialization and diagnostic sequence
//  (initMCP23018()). If the MCP23018 is not operational, the overall
//  system cannot function anyway.
//
//  Restricting the scan to VL53L5CX addresses makes the address-sequencing
//  diagnostics clearer and easier to interpret.
// =============================================================================

#if DEBUG_VL53L5CX
static void scanI2C()
{
  // VL53L5CX-related addresses only.
  // 0x29 = default boot address
  // 0x30..0x32 = assigned runtime addresses
  static const uint8_t known[] = {0x29, 0x30, 0x31, 0x32};

  DBG_VL53L5CXLN(F("I2C scan:"));

  uint8_t count = 0;

  for (uint8_t k = 0; k < sizeof(known) / sizeof(known[0]); k++)
  {
    uint8_t addr = known[k];

    Wire.beginTransmission(addr);

    if (Wire.endTransmission() == 0)
    {
      DBG_VL53L5CX(F("  found @ 0x"));

      if (addr < 0x10)
        DBG_VL53L5CX('0');

      DBG_VL53L5CXLN(addr, HEX);

      count++;
    }
  }

  if (count == 0)
    DBG_VL53L5CXLN(F("  (none)"));
}
#else
static void scanI2C() {}
#endif


// =============================================================================
//  Ethernet helpers — thin wrappers so the rest of the code stays readable
// =============================================================================
#if USE_W5500
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
// Returns 0xff if the device does not respond (safe default for input registers).
static uint8_t mcp23018ReadReg(uint8_t reg)
{
  Wire.beginTransmission(MCP23018_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return 0xff;  // repeated-start
  Wire.requestFrom((uint8_t)MCP23018_ADDR, (uint8_t)1);
  return Wire.available() ? Wire.read() : 0xff;
}


// =============================================================================
//  MCP23018 — Port A bit-level helpers
// =============================================================================

// Set or clear a single output bit on Port A.
// Uses mcp23018_portA_olata as a shadow register so we never need to read
// back OLATA over I²C (saves one transaction per operation).
static void mcp23018SetPortABit(uint8_t bit, bool state)
{
  if (state) mcp23018_portA_olata |=  (1u << bit);
  else       mcp23018_portA_olata &= ~(1u << bit);
  mcp23018WriteReg(MCP23018_OLATA, mcp23018_portA_olata);
}

// Enables or disables a single internal pull-up on Port A.
// Uses mcp23018_portA_gppua as a shadow register so we never need to read
// back GPPUA over I²C (saves one transaction per operation).
static void mcp23018EnablePortAPullup(uint8_t bit, bool state)
{
  if (state) mcp23018_portA_gppua |=  (1u << bit);
  else       mcp23018_portA_gppua &= ~(1u << bit);
  mcp23018WriteReg(MCP23018_GPPUA, mcp23018_portA_gppua);
}

// LPn control — idx 0/1/2 map to GPA5/4/3.
// Pass true to wake-up the sensor (LPn HIGH → sensor running).
// Pass false to hold the sensor in low power mode (LPn LOW → detach from the I2C bus).
// Open-drain: "HIGH" releases the pin to the pull-up on the VL53L5CX board.
void mcp23018SetLPN(uint8_t idx, bool state)
{
  if (idx < NUMBER_OF_DISTANCE_SENSORS)
    mcp23018SetPortABit(sensorLpnBit[idx], state);
}

// HW reset of the Ethernet W5500 Lite board via GPA2 (open-drain, active-low).
// Pass true to assert reset, false to release (chip begins initialising).
void mcp23018ResetW5500(bool state) { mcp23018SetPortABit(MCP_GPA_W5500RST, !state); }

// Power rail enables (open-drain, active-low).
// Dynamic handling of the MCP23018 internal pull-ups, that are enabled only
// when the requested state of the power rails is HIGH (power rail disabled).
void mcp23018EnableRail5V (bool state)
{
    // Control Function for the +5V Rail
    // state = true  -> Line to 0 (GND), Rail turns ON
    // state = false -> Line to 1 (2.88V), Rail turns OFF

    if (state)
    {
        // ENABLE RAIL: Force open-drain low-side driver ON to clamp line to GND
        // This has an immediate effect: in the transition Disabled --> Enabled,
        // the TPS54302 enabling line is immediately clamped to GND
        mcp23018SetPortABit(MCP_GPA_RAIL5V, !state);
        // Disable internal pull-up to eliminate parasitic leakage current
        mcp23018EnablePortAPullup(MCP_GPA_RAIL5V, !state);
    }
    else
    {
        // DISABLE RAIL: Release active GND clamp by setting latch to 1
        // This has NO immediate effect: in the transition Enabled --> Disabled,
        // the TPS54302 enabling line is still pulled-down by the external
        // 680K resistor, since the MCP23018 internal pull-up is disabled
        mcp23018SetPortABit(MCP_GPA_RAIL5V, !state);
        // Turn on internal ~100K pull-up to float the the TPS54302 enabling line
        // to ~2.8V via external divider network. This actually disables the rail
        mcp23018EnablePortAPullup(MCP_GPA_RAIL5V, !state);
    }
}

void mcp23018EnableRail11V (bool state)
{
    // Control Function for the +11.5V Rail
    // state = true  -> Line to 0 (GND), Rail turns ON
    // state = false -> Line to 1 (2.88V), Rail turns OFF

    if (state)
    {
        // ENABLE RAIL: Force open-drain low-side driver ON to clamp line to GND
        // This has an immediate effect: in the transition Disabled --> Enabled,
        // the TPS54302 enabling line is immediately clamped to GND
        mcp23018SetPortABit(MCP_GPA_RAIL11V, !state);
        // Disable internal pull-up to eliminate parasitic leakage current
        mcp23018EnablePortAPullup(MCP_GPA_RAIL11V, !state);
    }
    else
    {
        // DISABLE RAIL: Release active GND clamp by setting latch to 1
        // This has NO immediate effect: in the transition Enabled --> Disabled,
        // the TPS54302 enabling line is still pulled-down by the external
        // 680K resistor, since the MCP23018 internal pull-up is disabled
        mcp23018SetPortABit(MCP_GPA_RAIL11V, !state);
        // Turn on internal ~100K pull-up to float the the TPS54302 enabling line
        // to ~2.8V via external divider network. This actually disables the rail
        mcp23018EnablePortAPullup(MCP_GPA_RAIL11V, !state);
    }
}


// =============================================================================
//  MCP23018 Glitch-Free Initialization Sequence
// =============================================================================
void initMCP23018() {
  // Stage 1: Hardware Reset Cycle ─────────────────────────────────────────────
  // Pull the RESET pin LOW for ≥1 µs (actually 2 ms.) to force a full chip reset, then release.
  // This guarantees all registers are at their power-on defaults regardless of
  // any previous partial initialisation (e.g. after a Pico2 soft-reset).

  mcp23018_present = false;

  // Reset software mirrors so every boot/reboot starts from a known state.
  mcp23018_portA_olata = 0x3C;
  mcp23018_portA_gppua = 0x03;

  pinMode(MCP23018_RESET_PIN, OUTPUT);
  digitalWrite(MCP23018_RESET_PIN, LOW);
  delay(MCP23018_RST_SLEEP_MS); // Much longer than actually needed, however, no need to improve...
  digitalWrite(MCP23018_RESET_PIN, HIGH);
  delay(MCP23018_RST_RESUME_MS); // Brief hold (actually 2ms.) delay before starting I2C transactions

  // The Pico's internal pull-ups for MCP23018_INTA_PIN and MCP23018_INTB_PIN
  // should be kept disabled, since these lines are intended for a push-pull
  // active driving mode. The related configuration is implemented during the
  // Board Setup stage.
  // The Pico's internal pull-ups for MCP23018_INTA_PIN and MCP23018_INTB_PIN
  // should be kept disabled, since these lines are intended for push-pull
  // active driving mode. INTA is used for PB0/PB1 interrupt-on-change handling
  // and is routed to GP28 via the validated Y connection.

  // Stage 2: IOCON — Chip Level configuration ──────────────────────────────────
  // IOCON = 0x21, same as validated Golden Reference:
  //   BANK   = 0  registers are in sequential paired layout (address map above)
  //   MIRROR = 0  INTA and INTB are independent (not mirrored)
  //   SEQOP  = 1  address pointer disabled after each byte
  //   DISSLW = 0  SDA slew rate control enabled
  //   HAEN   = 0  hardware address enable
  //   ODR    = 0  INT pins are NOT open-drain:
  //               push-pull active output drivers on INTA/INTB pins
  //   INTPOL = 0  INT pins are active-low
  if (!mcp23018WriteReg(MCP23018_IOCON, 0x21))
  {
    // If the first write fails the device is absent or the bus is not ready
    Serial.println(F("MCP23018: not found"));
    mcp23018_present = false;
    return;
  }

  // Stage 3: Enable explicit internal Pull-Ups safely ──────────────────────────
  // Port A: Enable pullups ONLY for input keys PB0 and PB1 (Bit 0 and 1)
  // Value = 0x03 -> 0000 0011 Binary
  if (!mcp23018WriteReg(MCP23018_GPPUA, mcp23018_portA_gppua))
  {
    Serial.println(F("MCP23018: unable to set MCP23018_GPPUA register"));
    mcp23018_present = false;
    return;
  }
  
  // Port B: Enable all internal pull-ups for stabilizing unconnected lines,
  // holding each pin at a defined HIGH level and preventing spurious interrupts
  if (!mcp23018WriteReg(MCP23018_GPPUB, 0xFF))
  {
    Serial.println(F("MCP23018: unable to set MCP23018_GPPUB register"));
    mcp23018_present = false;
    return;
  }

  // Stage 4: Initial Output State Preloading (OLATA Latch Buffer Target) ───────
  // Value = 0x3C -> 0011 1100 Binary
  // Bits 6,7 (Rails) = 0 -> Prepared to start enabled at 0V ground clamps
  // Bits 2,3,4,5 (Resets/LPn) = 1
  //      --> Prepared to start releasing/high to wake peripherals; note that,
  //      --> even though redundant, one additional explicit cycling of the
  //      --> reset/LPn mode signals is implemented for the W5500 Lite and the
  //      --> VL53L5CX sensor boards in their dedicated initialization routines
  if (!mcp23018WriteReg(MCP23018_OLATA, mcp23018_portA_olata))
  {
    Serial.println(F("MCP23018: unable to set MCP23018_OLATA register"));
    mcp23018_present = false;
    return;
  }

  // Port B is all-input; latch value is irrelevant
  if (!mcp23018WriteReg(MCP23018_OLATB, 0x00))
  {
    Serial.println(F("MCP23018: unable to set MCP23018_OLATB register"));
    mcp23018_present = false;
    return;
  }

  // Stage 5: Commit Port Directions (IODIRA & IODIRB) ──────────────────────────
  // to switch from high-Z to active
  // IODIRA = 0b00000011
  //   Bits 0–1  (GPA0/GPA1) = 1 → inputs  (push buttons PB0, PB1)
  //   Bits 2–7  (GPA2–GPA7) = 0 → outputs (ETH RST, LPn ×3, rail enables ×2)
  if (!mcp23018WriteReg(MCP23018_IODIRA, 0x03))
  {
    Serial.println(F("MCP23018: unable to set MCP23018_IODIRA register"));
    mcp23018_present = false;
    return;
  }

  // Port B: All pins are unconnected. Set all as inputs
  if (!mcp23018WriteReg(MCP23018_IODIRB, 0xFF))
  {
    Serial.println(F("MCP23018: unable to set MCP23018_IODIRB register"));
    mcp23018_present = false;
    return;
  }

  // Stage 6: Interrupt configuration for PB0/PB1 on PORTA ──────────────────────
  // GPINTENA = 0b00000011 → enable interrupt-on-change for GPA0 and GPA1 only.
  //
  // INTCONA = 0x00:
  //   compare each enabled pin against its previous value.
  //   Both press and release generate interrupt events.
  //
  // DEFVALA is not used for GPA0/GPA1 when INTCONA bit0/bit1 = 0,
  // but it is cleared for determinism, as in the Golden Reference.
  //
  // Port B interrupts are disabled entirely.
  if (!mcp23018WriteReg(MCP23018_DEFVALA, 0x00))
  {
    Serial.println(F("MCP23018: unable to set MCP23018_DEFVALA register"));
    mcp23018_present = false;
    return;
  }

  if (!mcp23018WriteReg(MCP23018_GPINTENA, 0x03))
  {
    Serial.println(F("MCP23018: unable to set MCP23018_GPINTENA register"));
    mcp23018_present = false;
    return;
  }

  if (!mcp23018WriteReg(MCP23018_GPINTENB, 0x00))
  {
    Serial.println(F("MCP23018: unable to set MCP23018_GPINTENB register"));
    mcp23018_present = false;
    return;
  }

  if (!mcp23018WriteReg(MCP23018_INTCONA,  0x00))
  {
    Serial.println(F("MCP23018: unable to set MCP23018_INTCONA register"));
    mcp23018_present = false;
    return;
  }

  if (!mcp23018WriteReg(MCP23018_INTCONB,  0x00))
  {
    Serial.println(F("MCP23018: unable to set MCP23018_INTCONB register"));
    mcp23018_present = false;
    return;
  }

  mcp23018_present = true;
  Serial.println(F("MCP23018: OK"));
}

static uint8_t i2cAck(uint8_t addr)
{
  Wire.beginTransmission(addr);
  return Wire.endTransmission();
}

// VL53L5CX ISR-driven acquisition helpers.
// Implemented in small integration steps.
// Do not perform I2C or Serial operations inside the ISR functions.
void __not_in_flash_func(sensor0_ISR) ();
void __not_in_flash_func(sensor1_ISR) ();
void __not_in_flash_func(sensor2_ISR) ();

// =============================================================================
//  VL53L5CX data-ready ISRs
// =============================================================================
// The VL53L5CX GPIO1 / INT data-ready event is a short active-low pulse.
// Each ISR is kept minimal: no I2C, no Serial, no counters, no diagnostics.
// All GPIO ISRs are kept in SRAM when possible.
// PB and ToF ISRs are not timing-critical by themselves, since they only latch
// software flags. However, keeping them out of flash minimizes their execution
// latency and reduces possible interference with the exposure/LED timing chain,
// which is the only hard real-time interrupt path in this firmware.

void __not_in_flash_func(sensor0_ISR) ()
{
  dataReadyFlags[0] = true;
}

void __not_in_flash_func(sensor1_ISR) ()
{
  dataReadyFlags[1] = true;
}

void __not_in_flash_func(sensor2_ISR) ()
{
  dataReadyFlags[2] = true;
}

// =============================================================================
//  VL53L5CX — boot-state aware address sequencing and warm-start recovery
// =============================================================================
//
//  This function initializes the three VL53L5CX sensors connected to the shared
//  I2C bus through the MCP23018-controlled LPn lines.
//
//  The system must support two fundamentally different startup conditions:
//
//  -----------------------------------------------------------------------------
//  1. COLD START / FULL POWER CYCLE
//  -----------------------------------------------------------------------------
//
//  In a real power-on cycle, all VL53L5CX sensors start from their default I2C
//  address:
//
//      7-bit Wire address: 0x29
//      ST 8-bit notation : 0x52
//
//  Since all three sensors share the same I2C bus, they cannot be initialized
//  while they are all active at the default address. The normal cold-start
//  procedure is therefore:
//
//      a. initialize the MCP23018;
//      b. force all VL53L5CX LPn lines LOW;
//      c. verify that the VL53L5CX devices disappear from the bus;
//      d. release one LPn line at a time;
//      e. initialize the selected sensor at 0x29;
//      f. assign it its final unique address:
//
//             sensor 0 -> 0x30
//             sensor 1 -> 0x31
//             sensor 2 -> 0x32
//
//      g. configure resolution, ranging frequency and start ranging.
//
//  In this condition, LPn LOW has been observed to behave as expected:
//  the sensors become bus-silent and can then be sequenced one at a time.
//
//  -----------------------------------------------------------------------------
//  2. WARM START / MCU-ONLY RESET
//  -----------------------------------------------------------------------------
//
//  In a warm start, only the RP2350 / Raspberry Pi Pico 2 is reset, while the
//  VL53L5CX sensors remain powered.
//
//  In this condition the sensors may retain their previous runtime state,
//  including:
//
//      - assigned I2C addresses 0x30, 0x31, 0x32;
//      - active ranging state;
//      - internal firmware state from the previous run.
//
//  Experimental validation showed that, in this retained warm state, asserting
//  LPn LOW is not sufficient to guarantee that the sensors become bus-silent.
//  Even with the MCP23018 correctly driving the LPn lines LOW, the sensors may
//  continue to acknowledge at their already assigned addresses.
//
//  Therefore, in the warm path, LPn LOW must NOT be treated as a guaranteed
//  functional reset.
//
//  The correct strategy is:
//
//      a. detect that the sensors are already present at 0x30, 0x31, 0x32;
//      b. classify the condition as a warm retained-address path;
//      c. release the LPn lines;
//      d. reattach to each sensor at its assigned address;
//      e. call begin(assigned_address);
//      f. request stopRanging() to cleanly terminate any previous ranging state;
//      g. reconfigure resolution and ranging frequency;
//      h. restart ranging.
//
//  This avoids forcing a warm-retained system to behave like a cold-start system.
//
//  -----------------------------------------------------------------------------
//  3. MIXED OR AMBIGUOUS STATES
//  -----------------------------------------------------------------------------
//
//  Any bus state that does not match either of the two clean cases above is
//  treated as mixed or ambiguous.
//
//  Examples:
//
//      - 0x29 still present after all LPn lines were forced LOW;
//      - only some assigned addresses are present;
//      - both 0x29 and one assigned address acknowledge;
//      - one sensor responds at its assigned address while another appears at
//        the default address;
//      - a sensor acknowledges but begin() fails.
//
//  In those cases the firmware falls back to a per-sensor recovery attempt:
//  for each sensor, it checks both the default address and the expected assigned
//  address and chooses the safest available path:
//
//      - assigned address present -> warm attach;
//      - default address present  -> cold initialization;
//      - neither present          -> mark sensor unavailable.
//
//  If a sensor cannot be recovered, its LPn line is left LOW at the final stage
//  so that it does not disturb the shared I2C bus as much as possible.
//
//  -----------------------------------------------------------------------------
//  IMPORTANT DESIGN NOTES
//  -----------------------------------------------------------------------------
//
//  - The Reference Code must remain unchanged. This function belongs only to the
//    full application firmware.
//
//  - MCP23018 initialization is intentionally kept separate from this logic.
//    The MCP23018 must be operational before any LPn-based VL53L5CX strategy can
//    be attempted.
//
//  - LPn is used as an isolation/release control line, but not as an absolute
//    reset guarantee in warm-start conditions.
//
//  - The function explicitly classifies the bus state after all LPn lines have
//    been forced LOW:
//
//        no VL53L5CX address visible
//            -> cold sequencing path;
//
//        0x30, 0x31, 0x32 visible and 0x29 absent
//            -> warm retained-address attach path;
//
//        any other combination
//            -> mixed/ambiguous recovery path.
//
//  - At the end of the procedure, LPn lines are set according to the final
//    working status:
//
//        working sensor      -> LPn HIGH;
//        failed/unavailable  -> LPn LOW.
//
// =============================================================================

static void tofClearStatus()
{
  for (uint8_t i = 0; i < NUMBER_OF_DISTANCE_SENSORS; i++)
  {
    laser_working[i] = 0;
    laser_status[i] = 0;
    laser_distance_millimeters[i] = 0;
    laser_last_update_ms[i] = 0;
    dataReadyFlags[i] = false;
  }
}


static void tofForceAllLpnLow()
{
  DBG_VL53L5CXLN(F("ToF: forcing all LPn LOW"));

  for (uint8_t i = 0; i < NUMBER_OF_DISTANCE_SENSORS; i++)
  {
    mcp23018SetLPN(i, false);
  }

  delay(VL53L5CX_LPN_SLEEP_MS);
}

static void tofReleaseAllLpnHigh()
{
  DBG_VL53L5CXLN(F("ToF: releasing all LPn HIGH"));

  for (uint8_t i = 0; i < NUMBER_OF_DISTANCE_SENSORS; i++)
  {
    mcp23018SetLPN(i, true);
  }

  delay(VL53L5CX_LPN_RESUME_MS);
}

static ToFBusSnapshot tofReadBusSnapshot()
{
  ToFBusSnapshot s;

  s.ackDefault = i2cAck(0x29);

  for (uint8_t i = 0; i < NUMBER_OF_DISTANCE_SENSORS; i++)
  {
    s.ackAssigned[i] = i2cAck(LOX_ADDRESS[i]);
  }

  return s;
}

static void tofPrintBusSnapshot(const ToFBusSnapshot &s)
{
  DBG_VL53L5CX(F("ToF: ACK 0x29 = "));
  DBG_VL53L5CXLN(s.ackDefault);

  for (uint8_t i = 0; i < NUMBER_OF_DISTANCE_SENSORS; i++)
  {
    DBG_VL53L5CX(F("ToF: ACK assigned 0x"));
    DBG_VL53L5CX(LOX_ADDRESS[i], HEX);
    DBG_VL53L5CX(F(" = "));
    DBG_VL53L5CXLN(s.ackAssigned[i]);
  }
}

static ToFBusPath tofClassifyBusAfterAllLpnLow(const ToFBusSnapshot &s)
{
  bool defaultPresent = (s.ackDefault == 0);

  bool anyAssigned = false;
  bool allAssigned = true;

  for (uint8_t i = 0; i < NUMBER_OF_DISTANCE_SENSORS; i++)
  {
    bool assignedPresent = (s.ackAssigned[i] == 0);

    if (assignedPresent)
    {
      anyAssigned = true;
    }
    else
    {
      allAssigned = false;
    }
  }

  // Cold path:
  // After forcing all LPn LOW, no VL53L5CX address remains visible.
  if (!defaultPresent && !anyAssigned)
  {
    return TOF_BUS_COLD_SILENT;
  }

  // Warm retained path:
  // Sensors survived an MCU-only reset and are still available at 0x30/0x31/0x32.
  if (!defaultPresent && allAssigned)
  {
    return TOF_BUS_WARM_ASSIGNED;
  }

  // Any other combination is mixed or ambiguous.
  return TOF_BUS_MIXED_OR_AMBIGUOUS;
}

static bool tofWaitAndClearFirstFrame(uint8_t i, uint32_t timeoutMs)
{
  DBG_VL53L5CX(F("ToF["));
  DBG_VL53L5CX(i);
  DBG_VL53L5CXLN(F("]: waiting safe first frame before ISR attach"));

  uint32_t startMs = millis();

  while ((uint32_t)(millis() - startMs) < timeoutMs)
  {
    if (VL53L5CXSensor[i].isDataReady())
    {
      VL53L5CX_ResultsData temporaryBuffer;

      if (VL53L5CXSensor[i].getRangingData(&temporaryBuffer))
      {
        DBG_VL53L5CX(F("ToF["));
        DBG_VL53L5CX(i);
        DBG_VL53L5CXLN(F("]: safe first frame cleared"));
        return true;
      }
    }

    delay(10);
  }

  DBG_VL53L5CX(F("ToF["));
  DBG_VL53L5CX(i);
  DBG_VL53L5CXLN(F("]: safe first frame timeout"));

  return false;
}

static bool tofConfigureAndStart(uint8_t i)
{
  bool resolutionOk = VL53L5CXSensor[i].setResolution(64);
  if (!resolutionOk)
  {
    DBG_VL53L5CX(F("ToF["));
    DBG_VL53L5CX(i);
    DBG_VL53L5CXLN(F("]: setResolution(64) failed"));
    return false;
  }

  bool frequencyOk = VL53L5CXSensor[i].setRangingFrequency(10);
  if (!frequencyOk)
  {
    DBG_VL53L5CX(F("ToF["));
    DBG_VL53L5CX(i);
    DBG_VL53L5CXLN(F("]: setRangingFrequency(10) failed"));
    return false;
  }

  bool rangingOk = VL53L5CXSensor[i].startRanging();
  if (!rangingOk)
  {
    DBG_VL53L5CX(F("ToF["));
    DBG_VL53L5CX(i);
    DBG_VL53L5CXLN(F("]: startRanging() failed"));
    return false;
  }

  if (!tofWaitAndClearFirstFrame(i, VL53L5CX_FIRST_FRAME_TIMEOUT_MS))
  {
    DBG_VL53L5CX(F("ToF["));
    DBG_VL53L5CX(i);
    DBG_VL53L5CXLN(F("]: first frame validation failed"));
    return false;
  }

  DBG_VL53L5CX(F("ToF["));
  DBG_VL53L5CX(i);
  DBG_VL53L5CXLN(F("]: configured, ranging, and first frame validated"));

  return true;
}

static bool tofColdInitOneSensor(uint8_t i)
{
  DBG_VL53L5CX(F("ToF["));
  DBG_VL53L5CX(i);
  DBG_VL53L5CXLN(F("]: cold init from 0x29"));

  uint32_t beginStart = millis();
  bool beginOk = VL53L5CXSensor[i].begin(0x29, &Wire, 1000000);
  uint32_t beginDuration = millis() - beginStart;

  DBG_VL53L5CX(F("ToF["));
  DBG_VL53L5CX(i);
  DBG_VL53L5CX(F("]: begin(0x29) = "));
  DBG_VL53L5CX(beginOk ? F("true") : F("false"));
  DBG_VL53L5CX(F(", duration ms = "));
  DBG_VL53L5CXLN(beginDuration);

  if (!beginOk)
  {
    return false;
  }

  bool addressOk = VL53L5CXSensor[i].setAddress(LOX_ADDRESS[i]);

  DBG_VL53L5CX(F("ToF["));
  DBG_VL53L5CX(i);
  DBG_VL53L5CX(F("]: setAddress(0x"));
  DBG_VL53L5CX(LOX_ADDRESS[i], HEX);
  DBG_VL53L5CX(F(") = "));
  DBG_VL53L5CXLN(addressOk ? F("true") : F("false"));

  if (!addressOk)
  {
    return false;
  }

  delay(20);

#if DEBUG_VL53L5CX
  uint8_t ackAfterSet = i2cAck(LOX_ADDRESS[i]);

  DBG_VL53L5CX(F("ToF["));
  DBG_VL53L5CX(i);
  DBG_VL53L5CX(F("]: ACK after setAddress at 0x"));
  DBG_VL53L5CX(LOX_ADDRESS[i], HEX);
  DBG_VL53L5CX(F(" = "));
  DBG_VL53L5CXLN(ackAfterSet);
#endif

  return tofConfigureAndStart(i);
}

static bool tofWarmAttachOneSensor(uint8_t i)
{
  DBG_VL53L5CX(F("ToF["));
  DBG_VL53L5CX(i);
  DBG_VL53L5CX(F("]: warm attach at 0x"));
  DBG_VL53L5CXLN(LOX_ADDRESS[i], HEX);

  uint32_t beginStart = millis();
  bool beginOk = VL53L5CXSensor[i].begin(LOX_ADDRESS[i], &Wire, 1000000);
  uint32_t beginDuration = millis() - beginStart;

  DBG_VL53L5CX(F("ToF["));
  DBG_VL53L5CX(i);
  DBG_VL53L5CX(F("]: begin(assigned) = "));
  DBG_VL53L5CX(beginOk ? F("true") : F("false"));
  DBG_VL53L5CX(F(", duration ms = "));
  DBG_VL53L5CXLN(beginDuration);

  if (!beginOk)
  {
    return false;
  }

  DBG_VL53L5CX(F("ToF["));
  DBG_VL53L5CX(i);
  DBG_VL53L5CXLN(F("]: stopRanging() before reconfiguration"));

  VL53L5CXSensor[i].stopRanging();
  delay(20);

  return tofConfigureAndStart(i);
}

static void tofRunColdSequencing()
{
  DBG_VL53L5CXLN(F("ToF: selected LPn-isolated sequencing path"));

  for (uint8_t i = 0; i < NUMBER_OF_DISTANCE_SENSORS; i++)
  {
    laser_working[i] = 0;
    laser_status[i] = 0;
    laser_distance_millimeters[i] = 0;

    DBG_VL53L5CX(F("ToF["));
    DBG_VL53L5CX(i);
    DBG_VL53L5CXLN(F("]: preparing LPn-isolated state"));

    for (uint8_t j = 0; j < NUMBER_OF_DISTANCE_SENSORS; j++)
    {
      if (j < i)
      {
        mcp23018SetLPN(j, laser_working[j] != 0);
      }
      else if (j == i)
      {
        mcp23018SetLPN(j, true);
      }
      else
      {
        mcp23018SetLPN(j, false);
      }
    }

    delay(VL53L5CX_LPN_RESUME_MS);

    uint8_t ackDefault = i2cAck(0x29);
    uint8_t ackAssigned = i2cAck(LOX_ADDRESS[i]);

    DBG_VL53L5CX(F("ToF["));
    DBG_VL53L5CX(i);
    DBG_VL53L5CX(F("]: ACK 0x29 = "));
    DBG_VL53L5CXLN(ackDefault);

    DBG_VL53L5CX(F("ToF["));
    DBG_VL53L5CX(i);
    DBG_VL53L5CX(F("]: ACK assigned 0x"));
    DBG_VL53L5CX(LOX_ADDRESS[i], HEX);
    DBG_VL53L5CX(F(" = "));
    DBG_VL53L5CXLN(ackAssigned);

        bool ok = false;

    if ((ackDefault == 0) && (ackAssigned == 0))
    {
      DBG_VL53L5CX(F("ToF["));
      DBG_VL53L5CX(i);
      DBG_VL53L5CXLN(F("]: ambiguous state in LPn-isolated path, both default and assigned address ACK"));

      ok = false;
    }
    else if (ackAssigned == 0)
    {
      DBG_VL53L5CX(F("ToF["));
      DBG_VL53L5CX(i);
      DBG_VL53L5CX(F("]: LPn-isolated retained-address state at 0x"));
      DBG_VL53L5CXLN(LOX_ADDRESS[i], HEX);

      ok = tofWarmAttachOneSensor(i);
    }
    else if (ackDefault == 0)
    {
      DBG_VL53L5CX(F("ToF["));
      DBG_VL53L5CX(i);
      DBG_VL53L5CXLN(F("]: LPn-isolated default-address state at 0x29"));

      ok = tofColdInitOneSensor(i);
    }
    else
    {
      DBG_VL53L5CX(F("ToF["));
      DBG_VL53L5CX(i);
      DBG_VL53L5CXLN(F("]: no ACK at default or assigned address in LPn-isolated path"));

      ok = false;
    }

    if (!ok)
    {
      DBG_VL53L5CX(F("ToF["));
      DBG_VL53L5CX(i);
      DBG_VL53L5CXLN(F("]: LPn-isolated init/attach failed"));

      mcp23018SetLPN(i, false);
      delay(20);
      continue;
    }

    laser_working[i] = 1;

    DBG_VL53L5CX(F("ToF["));
    DBG_VL53L5CX(i);
    DBG_VL53L5CXLN(F("]: ready"));
  }
}

static void tofRunWarmAttach()
{
  DBG_VL53L5CXLN(F("ToF: selected WARM assigned-address attach path"));

  // In warm retained state, LPn LOW is not assumed to be a functional reset.
  // Release all sensors and reattach to their already assigned addresses.
  tofReleaseAllLpnHigh();

  for (uint8_t i = 0; i < NUMBER_OF_DISTANCE_SENSORS; i++)
  {
    laser_working[i] = 0;
    laser_status[i] = 0;
    laser_distance_millimeters[i] = 0;

    uint8_t ackAssigned = i2cAck(LOX_ADDRESS[i]);

    if (ackAssigned != 0)
    {
      DBG_VL53L5CX(F("ToF["));
      DBG_VL53L5CX(i);
      DBG_VL53L5CX(F("]: missing assigned-address ACK at 0x"));
      DBG_VL53L5CXLN(LOX_ADDRESS[i], HEX);

      mcp23018SetLPN(i, false);
      delay(20);
      continue;
    }

    if (!tofWarmAttachOneSensor(i))
    {
      DBG_VL53L5CX(F("ToF["));
      DBG_VL53L5CX(i);
      DBG_VL53L5CXLN(F("]: warm attach failed"));

      mcp23018SetLPN(i, false);
      delay(20);
      continue;
    }

    laser_working[i] = 1;

    DBG_VL53L5CX(F("ToF["));
    DBG_VL53L5CX(i);
    DBG_VL53L5CXLN(F("]: ready"));
  }
}

static void tofRunMixedRecovery()
{
  DBG_VL53L5CXLN(F("ToF: selected MIXED/AMBIGUOUS recovery path"));
  DBG_VL53L5CXLN(F("ToF: using per-sensor default/assigned detection"));

  for (uint8_t i = 0; i < NUMBER_OF_DISTANCE_SENSORS; i++)
  {
    laser_working[i] = 0;
    laser_status[i] = 0;
    laser_distance_millimeters[i] = 0;

    DBG_VL53L5CX(F("ToF["));
    DBG_VL53L5CX(i);
    DBG_VL53L5CXLN(F("]: preparing LPn state for mixed recovery"));

    for (uint8_t j = 0; j < NUMBER_OF_DISTANCE_SENSORS; j++)
    {
      if (j < i)
      {
        mcp23018SetLPN(j, laser_working[j] != 0);
      }
      else if (j == i)
      {
        mcp23018SetLPN(j, true);
      }
      else
      {
        mcp23018SetLPN(j, false);
      }
    }

    delay(VL53L5CX_LPN_RESUME_MS);

    uint8_t ackDefault = i2cAck(0x29);
    uint8_t ackAssigned = i2cAck(LOX_ADDRESS[i]);

    DBG_VL53L5CX(F("ToF["));
    DBG_VL53L5CX(i);
    DBG_VL53L5CX(F("]: ACK 0x29 = "));
    DBG_VL53L5CXLN(ackDefault);

    DBG_VL53L5CX(F("ToF["));
    DBG_VL53L5CX(i);
    DBG_VL53L5CX(F("]: ACK assigned 0x"));
    DBG_VL53L5CX(LOX_ADDRESS[i], HEX);
    DBG_VL53L5CX(F(" = "));
    DBG_VL53L5CXLN(ackAssigned);

    bool ok = false;

    if ((ackDefault == 0) && (ackAssigned == 0))
    {
      DBG_VL53L5CX(F("ToF["));
      DBG_VL53L5CX(i);
      DBG_VL53L5CXLN(F("]: ambiguous state, both default and assigned address ACK"));

      ok = false;
    }
    else if (ackAssigned == 0)
    {
      ok = tofWarmAttachOneSensor(i);
    }
    else if (ackDefault == 0)
    {
      ok = tofColdInitOneSensor(i);
    }
    else
    {
      DBG_VL53L5CX(F("ToF["));
      DBG_VL53L5CX(i);
      DBG_VL53L5CXLN(F("]: no ACK at default or assigned address"));

      ok = false;
    }

    if (!ok)
    {
      DBG_VL53L5CX(F("ToF["));
      DBG_VL53L5CX(i);
      DBG_VL53L5CXLN(F("]: mixed recovery failed"));

      mcp23018SetLPN(i, false);
      delay(20);
      continue;
    }

    laser_working[i] = 1;

    DBG_VL53L5CX(F("ToF["));
    DBG_VL53L5CX(i);
    DBG_VL53L5CXLN(F("]: ready"));
  }
}

static void tofApplyFinalLpnState()
{
  DBG_VL53L5CXLN(F("ToF: final LPn state"));

  for (uint8_t i = 0; i < NUMBER_OF_DISTANCE_SENSORS; i++)
  {
    if (laser_working[i])
    {
      mcp23018SetLPN(i, true);

      DBG_VL53L5CX(F("ToF["));
      DBG_VL53L5CX(i);
      DBG_VL53L5CXLN(F("]: LPn HIGH, working"));
    }
    else
    {
      mcp23018SetLPN(i, false);

      DBG_VL53L5CX(F("ToF["));
      DBG_VL53L5CX(i);
      DBG_VL53L5CXLN(F("]: LPn LOW, failed or unavailable"));
    }
  }

  delay(20);

#if DEBUG_VL53L5CX
  DBG_VL53L5CXLN(F("ToF: final address scan"));
  scanI2C();
#endif
}

static uint8_t tofCountWorkingSensors()
{
  uint8_t count = 0;

  for (uint8_t i = 0; i < NUMBER_OF_DISTANCE_SENSORS; i++)
  {
    if (laser_working[i])
    {
      count++;
    }
  }

  return count;
}


static void tofPrintInitSummary()
{
  uint8_t working = tofCountWorkingSensors();

  if (working == NUMBER_OF_DISTANCE_SENSORS)
  {
    DBG_VL53L5CX(F("ToF: ready, working sensors = "));
  }
  else if (working > 0)
  {
    DBG_VL53L5CX(F("ToF: degraded, working sensors = "));
  }
  else
  {
    DBG_VL53L5CX(F("ToF: failed, working sensors = "));
  }

  DBG_VL53L5CX(working);
  DBG_VL53L5CX(F("/"));
  DBG_VL53L5CXLN(NUMBER_OF_DISTANCE_SENSORS);
}

// ToF acquisition is interrupt-driven.
// VL53L5CX data-ready pulses set dataReadyFlags[] from ISR.
// Frames are retrieved in processTofInterruptEvents().
static void attachTofInterruptsForWorkingSensors()
{
  DBG_VL53L5CXLN(F("ToF: attaching hardware interrupts for working sensors only"));

  if (laser_working[0])
  {
    attachInterrupt(digitalPinToInterrupt(sensorIntPins[0]), sensor0_ISR, FALLING);
    DBG_VL53L5CXLN(F("ToFISR attached on GP12"));
  }
  else
  {
    DBG_VL53L5CXLN(F("ToFISR not attached on GP12, sensor not working"));
  }

  if (laser_working[1])
  {
    attachInterrupt(digitalPinToInterrupt(sensorIntPins[1]), sensor1_ISR, FALLING);
    DBG_VL53L5CXLN(F("ToFISR attached on GP13"));
  }
  else
  {
    DBG_VL53L5CXLN(F("ToFISR not attached on GP13, sensor not working"));
  }

  if (laser_working[2])
  {
    attachInterrupt(digitalPinToInterrupt(sensorIntPins[2]), sensor2_ISR, FALLING);
    DBG_VL53L5CXLN(F("ToFISR attached on GP14"));
  }
  else
  {
    DBG_VL53L5CXLN(F("ToFISR not attached on GP14, sensor not working"));
  }
}


void setI2CDistanceAddresses()
{
  DBG_VL53L5CXLN(F("ToF: starting VL53L5CX x3 address sequencing"));

  tofClearStatus();

  if (!mcp23018_present)
  {
    DBG_VL53L5CXLN(F("ToF: MCP23018 not present, cannot control LPn lines"));
    return;
  }

#if DEBUG_VL53L5CX
  DBG_VL53L5CXLN(F("ToF: pre-sequencing VL53L5CX address scan"));
  scanI2C();
#endif

  tofForceAllLpnLow();

  ToFBusSnapshot afterLow = tofReadBusSnapshot();

  DBG_VL53L5CXLN(F("ToF: bus snapshot after all LPn LOW"));
  tofPrintBusSnapshot(afterLow);

  ToFBusPath path = tofClassifyBusAfterAllLpnLow(afterLow);

  if (path == TOF_BUS_COLD_SILENT)
  {
    tofRunColdSequencing();
  }
  else if (path == TOF_BUS_WARM_ASSIGNED)
  {
    tofRunWarmAttach();
  }
  else
  {
    tofRunMixedRecovery();
  }

  tofApplyFinalLpnState();
}




// =============================================================================
//  VL53L5CX — data collection
// =============================================================================

// Validate one of the four central VL53L5CX zones.
// Current minimal policy:
//   - distance must be positive;
//   - target_status must be 5 or 255.
// Status 5 is "Range valid".
// Status 255 is accepted in this application because it has been experimentally
// stable and useful in the near-field setup.
// Status values such as 4 and 13 are rejected because they were associated
// with major outliers during validation.
static bool isValidCenterZone(const VL53L5CX_ResultsData &r, uint8_t idx)
{
  int16_t d = r.distance_mm[idx];
  uint8_t status = r.target_status[idx];

  if (d <= 0)
  {
    return false;
  }

  if ((status == 5) || (status == 255))
  {
    return true;
  }

  return false;
}

// Extract a single representative distance from an 8x8 result frame.
// Averages the four central zones: 27, 28, 35, 36.
// Returns 0 if no valid central-zone data are available.
static uint16_t centerDistanceMm(const VL53L5CX_ResultsData &r)
{
  const uint8_t idx[4] = {27, 28, 35, 36};
  uint32_t sum = 0;
  uint8_t cnt = 0;

  for (uint8_t k = 0; k < 4; k++)
  {
    uint8_t z = idx[k];

    if (isValidCenterZone(r, z))
    {
      sum += (uint16_t)r.distance_mm[z];
      cnt++;
    }
  }

  return cnt ? (uint16_t)(sum / cnt) : 0;
}

static void handleToFData(uint8_t i)
{
  if (i >= NUMBER_OF_DISTANCE_SENSORS)
  {
    return;
  }

  if (!laser_working[i])
  {
    return;
  }

  if (VL53L5CXSensor[i].getRangingData(&VL53L5CXResults))
  {
    uint16_t d = centerDistanceMm(VL53L5CXResults);

    if (d > 0)
    {
      unsigned long nowMs = millis();

      laser_status[i] = 1;
      laser_distance_millimeters[i] = d;
      laser_last_update_ms[i] = nowMs;
    }
  }
}

static void processTofInterruptEvents()
{
  for (uint8_t i = 0; i < NUMBER_OF_DISTANCE_SENSORS; i++)
  {
    if (!laser_working[i])
    {
      continue;
    }

    if (dataReadyFlags[i])
    {
      dataReadyFlags[i] = false;
      handleToFData(i);
    }
  }
}

// Legacy polling path. Not called in the code
// Kept in the source for temporary diagnostic fallback only.
// Normal ToF acquisition is now ISR-driven:
//   - VL53L5CX INT pulse sets dataReadyFlags[i];
//   - processTofInterruptEvents() consumes the flag in the fast loop;
//   - handleToFData(i) retrieves the frame over I2C.
void read_triple_sensors()
{
  unsigned long nowMs = millis();

  for (uint8_t i = 0; i < NUMBER_OF_DISTANCE_SENSORS; i++)
  {
    if (!laser_working[i])
    {
      continue;
    }

    if (VL53L5CXSensor[i].isDataReady() &&
        VL53L5CXSensor[i].getRangingData(&VL53L5CXResults))
    {
      uint16_t d = centerDistanceMm(VL53L5CXResults);

      if (d > 0)
      {
        laser_status[i] = 1;
        laser_distance_millimeters[i] = d;
        laser_last_update_ms[i] = nowMs;
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

// =============================================================================
//  Per-COB thermal budget — the hard backstop on duty cycle
//
//  A leaky bucket per COB, denominated in microseconds of permitted ON time.
//  It refills at a fixed fraction of real time and is charged the worst-case
//  pulse width up front. Because the charge happens at the rising edge and is
//  never refunded, the accounting can only ever over-estimate heat, never
//  under-estimate it — the safe direction to be wrong in.
//
//  All of this runs in ISR context, so it is deliberately branch-light and uses
//  a shift rather than a division for the refill.
// =============================================================================

// Clamp the runtime-adjustable timings to the hard limits. Every write to
// lightTurnOffMicroseconds anywhere in the firmware must go through here.
static void clampLightTiming()
{
  if (lightTurnOffMicroseconds > LIGHT_HARD_MAX_ON_US)
      lightTurnOffMicroseconds = LIGHT_HARD_MAX_ON_US;
  if (lightTurnOffMicroseconds < 1)
      lightTurnOffMicroseconds = 1;
  if (lightKeepOffMicroseconds < LIGHT_HARD_MIN_PERIOD_US)
      lightKeepOffMicroseconds = LIGHT_HARD_MIN_PERIOD_US;
  // The guard rail must always outlast the pulse, or a COB could be re-fired
  // before the previous pulse has been terminated.
  if (lightKeepOffMicroseconds <= lightTurnOffMicroseconds)
      lightKeepOffMicroseconds = lightTurnOffMicroseconds + LIGHT_HARD_MIN_PERIOD_US;
}

// Start every COB with a full bucket. Called once before the ISR is armed.
static void resetLightBudgets()
{
  uint64_t now = time_us_64();
  for (uint8_t i = 0; i < NUMBER_OF_LIGHTS; i++)
  {
    lightBudgetUs[i]      = LIGHT_BUDGET_CAPACITY_US;
    lightBudgetStampUs[i] = now;
  }
}

// Refill one COB's bucket up to `now` and return the credit available.
// Idempotent: advancing the stamp as we go means repeated calls don't double-credit.
static inline uint32_t __not_in_flash_func(lightBudgetRefill)(uint8_t idx, uint64_t now)
{
  uint64_t elapsed = now - lightBudgetStampUs[idx];
  lightBudgetStampUs[idx] = now;

  uint32_t credit = lightBudgetUs[idx] + (uint32_t)(elapsed >> LIGHT_DUTY_SHIFT);
  if (credit > LIGHT_BUDGET_CAPACITY_US) credit = LIGHT_BUDGET_CAPACITY_US;
  lightBudgetUs[idx] = credit;
  return credit;
}

// Pick which COB actually fires this exposure.
//   `wanted` is 0-indexed and assumed valid.
// Returns the 0-indexed COB to fire, or NO_LIGHT_ON if every COB is out of budget.
//
// Fast path (the overwhelmingly common one) refills and tests only the requested
// COB. Only when that COB is too hot do we price all six and substitute the one
// with the most headroom — which under sustained load naturally settles into a
// round-robin over the coolest COBs.
static inline uint8_t __not_in_flash_func(lightBudgetSelect)(uint8_t wanted, uint64_t now, uint32_t cost)
{
  if (lightBudgetRefill(wanted, now) >= cost)
  {
    lightBudgetUs[wanted] -= cost;
    return wanted;
  }

  uint8_t  best      = NO_LIGHT_ON;
  uint32_t bestCredit = 0;
  for (uint8_t i = 0; i < NUMBER_OF_LIGHTS; i++)
  {
    if (i == wanted) continue;                 // already refilled and rejected
    uint32_t credit = lightBudgetRefill(i, now);
    if (credit >= cost && credit > bestCredit) { bestCredit = credit; best = i; }
  }

  if (best == NO_LIGHT_ON) { lightStarvedFrames++; return NO_LIGHT_ON; }

  lightBudgetUs[best] -= cost;
  lightSubstitutions++;
  return best;
}

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
  // Order matters. Cancelling the alarms first leaves a window where the
  // (priority-0, preempting) exposure ISR can fire between the cancels and the
  // IRQ disable, schedule fresh alarms into the just-nulled IDs, and leak them
  // past the shutdown. Disabling both edges first makes the ISR unreachable for
  // the remainder of this function, so the cancels below cannot be undone.
  gpio_set_irq_enabled(CAMERA_EXPOSURE_PIN, GPIO_IRQ_EDGE_FALL, false);
  gpio_set_irq_enabled(CAMERA_EXPOSURE_PIN, GPIO_IRQ_EDGE_RISE, false);

  if (debouncingAlarmId >= 0) { cancel_alarm(debouncingAlarmId); debouncingAlarmId = -1; }
  if (lightOffAlarmId   >= 0) { cancel_alarm(lightOffAlarmId);   lightOffAlarmId   = -1; }
  if (guardRailAlarmId  >= 0) { cancel_alarm(guardRailAlarmId);  guardRailAlarmId  = -1; }

  waitingForFall  = false;

  lightToActivate = 0;
  activeLightPin  = 0;
  lightOn         = NO_LIGHT_ON;

  for (uint8_t i = 0; i < NUMBER_OF_LIGHTS; i++)
    gpio_put(directLightControl[i], 0);

  gpio_set_irq_enabled(CAMERA_EXPOSURE_PIN, GPIO_IRQ_EDGE_RISE, true);
}

// =============================================================================
//  Light safety watchdog — last line of defence for the overvolted COBs
//
//  The alarm chain is the primary pulse-width limiter, but it is software: a
//  cancelled-then-leaked alarm, a missed falling edge, or a hung ISR could in
//  principle leave a pin driven HIGH indefinitely, which destroys an overvolted
//  COB in short order. This runs every main-loop tick (~1 ms) and unconditionally
//  cuts any pin that has been HIGH past the hard deadline, independently of the
//  alarm chain and of all light state.
//
//  It must never be gated behind a feature flag or a mode check.
// =============================================================================
static void lightSafetyWatchdog()
{
  uint64_t now = time_us_64();

  // Deadline is generous relative to LIGHT_HARD_MAX_ON_US so this never races the
  // normal turn-off path; it exists to catch total failure, not to trim jitter.
  const uint64_t deadline = (uint64_t)LIGHT_HARD_MAX_ON_US * 4u;

  bool overdue = (activeLightPin != 0) && (now - risingEdgeTimeUs > deadline);

  // Also catch a pin that is HIGH while no pulse is supposed to be in flight at all.
  bool orphaned = false;
  if (activeLightPin == 0)
  {
    for (uint8_t i = 0; i < NUMBER_OF_LIGHTS; i++)
      if (gpio_get_out_level(directLightControl[i])) { orphaned = true; break; }
  }

  if (overdue || orphaned)
  {
    for (uint8_t i = 0; i < NUMBER_OF_LIGHTS; i++)
      gpio_put(directLightControl[i], 0);
    activeLightPin = 0;
    lightWatchdogTrips++;
  }
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
    if (depths[i] == 0) continue;  // no valid reading (failed / out-of-range / no data) — don't bias selection
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

// Return the next light index given the current index and the active pattern.
//   PATTERN_ROUNDROBIN — 0,1,2,3,4,5 — adjacent COBs, one step at a time
//   PATTERN_OPPOSITE   — 0,3,1,4,2,5 — always jump to the (roughly) opposing COB
//
// Note on PATTERN_OPPOSITE: the table is a single 6-cycle, not a set of swaps.
// It visits every COB exactly once per cycle while making each consecutive pair
// come from opposite sides of the ring, which is what makes per-pixel
// min(I_i, I_opposite) suppress specular lobes and what breaks the 180° azimuth
// ambiguity in AoLP. Earlier comments described it as "0↔3, 1↔4, 2↔5", which
// reads as an involution — it is not one; the table itself was already correct.
//
// `current` may legitimately be NO_LIGHT_ON (16) after deactivateLights(); both
// branches must cope. The old code fell through to (16+1)%6 = 5, which both
// skipped the pattern table and started the sequence at COB 6.
uint8_t getNextLight(uint8_t current, uint8_t total, uint8_t pattern)
{
  if (current >= total) return 0;  // undefined / just-deactivated → start of cycle

  if (pattern == PATTERN_OPPOSITE)
  {
    const uint8_t alt[6] = {3, 4, 5, 1, 2, 0};
    if (current < 6) return alt[current];
  }
  return (current + 1) % total;
}

// Replace the strobe schedule. `src` holds 0-indexed COB numbers.
// Applied atomically from the ISR's point of view: the length is published last,
// and the cursor is rewound so the new cycle starts at its first entry.
// Returns the number of entries accepted, or 0 if the schedule was rejected.
static uint8_t setLightSchedule(const uint8_t *src, uint8_t len)
{
  if (len == 0 || len > MAX_SCHEDULE_LEN) return 0;

  for (uint8_t i = 0; i < len; i++)
    if (src[i] >= NUMBER_OF_LIGHTS) return 0;   // reject the whole thing, don't half-apply

  for (uint8_t i = 0; i < len; i++)
    lightSchedule[i] = src[i];

  lightScheduleCursor = 0;
  lightScheduleLen    = len;   // published last

  lightOn         = lightSchedule[0];
  lightToActivate = lightSchedule[0] + 1;
  return len;
}

// Load a schedule generated from `pattern` covering every COB once.
static void setScheduleFromPattern(uint8_t pattern)
{
  uint8_t seq[NUMBER_OF_LIGHTS];
  uint8_t cur = 0;
  for (uint8_t i = 0; i < NUMBER_OF_LIGHTS; i++)
  {
    seq[i] = cur;
    cur = getNextLight(cur, NUMBER_OF_LIGHTS, pattern);
  }
  setLightSchedule(seq, NUMBER_OF_LIGHTS);
}

// =============================================================================
//  Serial / Ethernet output helpers
// =============================================================================
// The protocol sends one comma-separated line per light cycle:
//   timestamp, button1, button2, dist0, dist1, dist2, L0..L5,
//   strobeCounter, strobeLight, substitutions, starved, watchdog
// Special distance tokens: F = sensor failed, H = out of range (>6000 mm), 0 = no new data
//
// The five trailing fields are appended rather than inserted, so a host's existing
// 12-field sscanf() keeps matching unchanged.
//   strobeCounter  — monotonic pulse index; the host's frame↔COB join key
//   strobeLight    — the COB that ACTUALLY fired (0-5), or 16 if the pulse was
//                    suppressed. This is post-substitution ground truth, so it
//                    may differ from the L0..L5 flags, which show what was asked for.
//   substitutions  — running count of "requested COB too hot, fired a cooler one"
//   starved        — running count of pulses suppressed because no COB had budget
//   watchdog       — running count of safety-watchdog trips (should stay 0)

void comma()   { Serial.print(F(",")); if (client) client.print(F(",")); }
void zero()    { Serial.print(F("0")); if (client) client.print(F("0")); }
void one()     { Serial.print(F("1")); if (client) client.print(F("1")); }
void high_v()  { Serial.print(F("H")); if (client) client.print(F("H")); }

void number(int v)
{
  Serial.print(v);
#if USE_W5500
  if (client) client.print(v);
#endif
}

void newline()
{
  Serial.print(F("\n"));
#if USE_W5500
  if (client) client.print(F("\n"));
#endif
}

void flush()
{
  Serial.flush();
#if USE_W5500
  if (client) client.flush();
#endif
}

void failed()
{
  Serial.print(F("F"));
#if USE_W5500
  if (client) client.print(F("F"));
#endif
#if DEBUG_VL53L5CX
  Serial.println();
  Serial.println(F("[FAIL] sensor failure signalled"));
#endif
}

void reportState(unsigned long ts, unsigned int b1, unsigned int b2)
{
  Serial.print(ts);
#if USE_W5500
  if (client) client.print(ts);
#endif
  comma();
  Serial.print(b1);
#if USE_W5500
  if (client) client.print(b1);
#endif
  comma();
  Serial.print(b2);
#if USE_W5500
  if (client) client.print(b2);
#endif
  comma();

  for (uint8_t i = 0; i < NUMBER_OF_DISTANCE_SENSORS; i++)
  {
    if (!laser_working[i])
    {
      failed();
    }
    else if ((laser_status[i] == 1) &&
             ((ts - laser_last_update_ms[i]) <= TOF_DATA_STALE_MS))
    {
      int d = (int)laser_distance_millimeters[i];

      if (d > 6000)
      {
        high_v();
      }
      else
      {
        number(d);
      }
    }
    else
    {
      zero();
    }

    comma();
  }

  for (uint8_t i = 0; i < NUMBER_OF_LIGHTS; i++)
  {
    if (i != lightOn) zero(); else one();
    if (i != NUMBER_OF_LIGHTS - 1) comma();
  }

  // Ground-truth tail. Snapshot the volatiles once so the line is internally
  // consistent even if the ISR fires while we are printing.
  uint32_t sc   = strobeCounter;
  uint8_t  sl   = strobeLight;
  uint32_t subs = lightSubstitutions;
  uint32_t strv = lightStarvedFrames;
  uint32_t wdt  = lightWatchdogTrips;

  comma(); number((int)sc);
  comma(); number((int)sl);
  comma(); number((int)subs);
  comma(); number((int)strv);
  comma(); number((int)wdt);

  newline();
  flush();
}

// =============================================================================
//  Exposure ISR and timer callbacks
//
//  Cycle per camera frame -->
//  All times are relative to the rising edge of the
//  Sony XCG-CP510 Camera Exposure Active signal (t = 0):
//
//    1. exposureISR (RISE)   — t = 0 µs:
//         record risingEdgeTimeUs, disable RISE, charge the per-COB thermal
//         budget (substituting a cooler COB if the requested one is exhausted),
//         light ON, latch strobeLight/strobeCounter,
//         start debounce alarm (B) and max-on-time alarm (C).
//
//    2. debouncingCallback   — t = lightDebouncingMicroseconds (20 µs):
//         enable FALL ISR; the debounce window ensures the signal has settled.
//
//    3a. exposureISR (FALL)  — t = t_fall  (20 µs ≤ t_fall < lightTurnOffMicroseconds):
//         light OFF, disable FALL, cancel alarm C, start guard-rail alarm (D).
//
//    3b. lightOffCallback    — t = lightTurnOffMicroseconds (1000 µs), timeout path:
//         light OFF (if still on), disable FALL, start guard-rail alarm (D).
//
//    4. guardRailCallback    — t = lightKeepOffMicroseconds (15000 µs):
//         advance the strobe schedule (exposure-locked mode),
//         enable RISE ISR — cycle complete.
//
//  The guard rail is what bounds the *instantaneous* strobe rate; the per-COB
//  leaky buckets bound the *sustained* per-COB duty. Both are required: the guard
//  rail alone permits one COB to take every strobe, which is 6x the load that
//  round-robin sequencing implied and enough to destroy an overvolted COB.
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
//  flash-cache latency at interrupt time, and IO_IRQ_BANK0 is kept at highest priority.
// =============================================================================

// Phase 4: guard-rail elapsed → re-enable the exposure rising-edge interrupt.
//
// This is also where the strobe schedule advances in exposure-locked mode. Doing
// it here rather than at the rising edge means the cursor moves exactly once per
// completed pulse, in the deterministic timer domain, with no dependence on host
// message timing. The next COB is therefore already latched into lightToActivate
// well before the next exposure edge arrives.
static int64_t __not_in_flash_func(guardRailCallback)(alarm_id_t, void *)
{
  guardRailAlarmId = -1;

  if (autoLights == STEP_EXPOSURE)
  {
    uint8_t len = lightScheduleLen;
    if (len > 0)
    {
      uint8_t cursor = lightScheduleCursor + 1;
      if (cursor >= len) cursor = 0;
      lightScheduleCursor = cursor;

      uint8_t next = lightSchedule[cursor];
      if (next < NUMBER_OF_LIGHTS)
      {
        lightOn         = next;
        lightToActivate = next + 1;
      }
    }
  }

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
      from_us_since_boot(risingEdgeTimeUs + lightKeepOffMicroseconds),
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

    // ── Thermal gate — the COBs are overvolted, so this is not optional. ──
    // Charge the worst-case pulse width, not the expected one: the falling edge
    // may never arrive and the pulse would then run to lightTurnOffMicroseconds.
    // If the requested COB is out of budget we substitute the coolest one that
    // still has headroom rather than dropping the frame; if none does, we leave
    // the frame dark, which is the only remaining safe option.
    uint32_t cost   = lightTurnOffMicroseconds;
    uint8_t  actual = lightBudgetSelect(light - 1, risingEdgeTimeUs, cost);

    if (actual >= NUMBER_OF_LIGHTS)
    {
      // Every COB is too hot. Skip the pulse but still run the guard rail so the
      // rising edge is re-armed and the schedule keeps advancing in step.
      strobeLight = NO_LIGHT_ON;
      strobeCounter++;
      guardRailAlarmId = add_alarm_at(
          from_us_since_boot(risingEdgeTimeUs + lightKeepOffMicroseconds),
          guardRailCallback, nullptr, true);
      return;
    }

    uint8_t pin = directLightControl[actual];
    activeLightPin = pin;
    gpio_put(pin, 1);

    // Publish ground truth: the COB that actually fired, and a monotonic strobe
    // index. The host joins frames to COBs on this, not on wall-clock correlation.
    strobeLight = actual;
    strobeCounter++;

    // Schedule debounce — re-arms ISR for falling edge after the signal settles
    debouncingAlarmId = add_alarm_at(
        from_us_since_boot(risingEdgeTimeUs + lightDebouncingMicroseconds),
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

    // Guard-rail fires at the absolute time of (risingEdge + lightKeepOffMicroseconds)
    guardRailAlarmId = add_alarm_at(
        from_us_since_boot(risingEdgeTimeUs + lightKeepOffMicroseconds),
        guardRailCallback, nullptr, true);
  }
}


static void recoverI2CBus()
{
  // Conservative I2C bus recovery before Wire.begin().
  //
  // This routine is intended for MCU-only software reset conditions, where
  // external I2C devices may remain powered and active while the RP2350 I2C
  // peripheral has been reset.
  //
  // Board assumption:
  // SDA and SCL already have physical pull-up resistors on the board.
  // Therefore we intentionally use INPUT, not INPUT_PULLUP, when releasing
  // the lines. This avoids enabling unnecessary internal pull-ups and avoids
  // additional parasitic current paths.
  //
  // Open-drain emulation:
  // - LOW  = configure pin as OUTPUT and write LOW
  // - HIGH = release pin by configuring it as INPUT
  // - never actively drive SDA or SCL HIGH

  // Release both lines and let the external pull-ups bring them HIGH.
  pinMode(SDA_PIN, INPUT);
  pinMode(SCL_PIN, INPUT);

  delay(2);

  // Provide extra SCL clocks.
  // If a slave was interrupted during a read transaction, these clocks may
  // allow it to shift out the remaining bits and release SDA.
  for (uint8_t i = 0; i < 16; i++)
  {
    // Drive SCL LOW.
    digitalWrite(SCL_PIN, LOW);
    pinMode(SCL_PIN, OUTPUT);
    delayMicroseconds(5);

    // Release SCL HIGH through the external pull-up.
    pinMode(SCL_PIN, INPUT);
    delayMicroseconds(5);
  }

  // Generate a manual I2C STOP condition:
  // SDA goes from LOW to HIGH while SCL is HIGH.
  //
  // Step 1: ensure SDA is LOW.
  digitalWrite(SDA_PIN, LOW);
  pinMode(SDA_PIN, OUTPUT);
  delayMicroseconds(5);

  // Step 2: release SCL and allow it to go HIGH.
  pinMode(SCL_PIN, INPUT);
  delayMicroseconds(5);

  // Step 3: release SDA while SCL is HIGH.
  pinMode(SDA_PIN, INPUT);
  delayMicroseconds(5);

  // Leave both lines released.
  pinMode(SDA_PIN, INPUT);
  pinMode(SCL_PIN, INPUT);

  delay(1);
}

static void performSoftwareReset()
{
  Serial.println(F("DEBUG: performing watchdog software reset"));
  Serial.flush();

  // Stop application-level activity immediately.
  serialOutputEnabled = false;
  autoLights = STEP_EXTERNAL;

  // Stop lights and exposure-related armed callbacks/IRQs.
  // This does not depend on I2C and is a reasonable safety cleanup.
  deactivateLights();

  Serial.println(F("DEBUG: watchdog reset now"));
  Serial.flush();

  delay(300);

  // Use watchdog as the primary reset mechanism.
  // This is intentionally more deterministic than rp2040.reboot()
  // in active acquisition / USB CDC streaming conditions.
  rp2040.wdt_begin(100);

  while (1) {
  }
}

#define LED_PIN 25

static void bootBlinkStart()
{
  pinMode(LED_PIN, OUTPUT);

  for (uint8_t i = 0; i < 3; i++)
  {
    digitalWrite(LED_PIN, HIGH);
    delay(80);
    digitalWrite(LED_PIN, LOW);
    delay(80);
  }
}

static void bootBlinkComplete()
{
  for (uint8_t i = 0; i < 2; i++)
  {
    digitalWrite(LED_PIN, HIGH);
    delay(300);
    digitalWrite(LED_PIN, LOW);
    delay(300);
  }
}

// =============================================================================
//  MCP23018 Push Button ISR and debounce helpers
// =============================================================================

void __not_in_flash_func(mcp23018_PortA_ISR) ()
{
  mcp23018_PortAIntFlg = true;
}

static void updateBoardLedFromButtons()
{
  // After setup is complete:
  // LED OFF = both buttons released
  // LED ON  = at least one button pressed
  digitalWrite(LED_PIN, (debouncedPB0 || debouncedPB1) ? HIGH : LOW);
}

static void updateDebouncedButtonsFromPortA(uint8_t portA)
{
  bool newPB0 = ((portA & (1u << MCP_GPA_PB0)) == 0);
  bool newPB1 = ((portA & (1u << MCP_GPA_PB1)) == 0);

  if (newPB0 != debouncedPB0)
  {
    debouncedPB0 = newPB0;
  }

  if (newPB1 != debouncedPB1)
  {
    debouncedPB1 = newPB1;
  }

  updateBoardLedFromButtons();
}

static void initMcp23018ButtonStateAndInterrupt()
{
  // Initialize debounced state from actual GPIOA.
  // Then clear any pending startup interrupt by reading INTCAPA.
  uint8_t startupGPIOA = mcp23018ReadReg(MCP23018_GPIOA);
  mcp23018ReadReg(MCP23018_INTCAPA);

  prevPortA = startupGPIOA;
  currPortA = startupGPIOA;

  debouncedPB0 = ((startupGPIOA & (1u << MCP_GPA_PB0)) == 0);
  debouncedPB1 = ((startupGPIOA & (1u << MCP_GPA_PB1)) == 0);

  mcp23018_PortAIntFlg = false;
  pbDebounceState = 0;

  // IOCON = 0x21:
  // ODR = 0 -> interrupt output is not open-drain.
  // INTA actively drives both LOW and HIGH.
  // Therefore GP28 must be configured as plain INPUT, not INPUT_PULLUP.
  pinMode(MCP23018_INTA_PIN, INPUT);

  attachInterrupt(
    digitalPinToInterrupt(MCP23018_INTA_PIN),
    mcp23018_PortA_ISR,
    FALLING);
}

static void processMcp23018ButtonsDebounce()
{
  if (!mcp23018_present)
  {
    return;
  }

  if (pbDebounceState == 0)
  {
    if (mcp23018_PortAIntFlg)
    {
      // Single canonical point where ISR flag is consumed.
      //
      // The MCP23018 INTA line remains asserted LOW until INTCAPA is read.
      // Reading INTCAPA clears the MCP23018 interrupt condition and
      // re-arms INTA for the next FALLING edge.
      mcp23018_PortAIntFlg = false;

      prevPortA = mcp23018ReadReg(MCP23018_INTCAPA);

      updateDebouncedButtonsFromPortA(prevPortA);

      pbDebounceState = 1;
    }
  }
  else
  {
    if (mcp23018_PortAIntFlg)
    {
      // If a new interrupt occurred during debounce active states,
      // this is considered a new edge/bounce/new transition.
      //
      // The flag is deliberately NOT cleared here.
      // The state machine is forced back to State 0, where the flag will be
      // consumed and INTCAPA will be read in the single canonical point.
      pbDebounceState = 0;
    }
    else
    {
      currPortA = mcp23018ReadReg(MCP23018_GPIOA);

      if ((prevPortA & PB_MASK) == (currPortA & PB_MASK))
      {
        pbDebounceState++;
      }
      else
      {
        updateDebouncedButtonsFromPortA(currPortA);

        prevPortA = currPortA;
        pbDebounceState = 1;
      }

      if (pbDebounceState >= PB_TICK_PERSISTENCE)
      {
        pbDebounceState = 0;
      }
    }
  }
}

// =============================================================================
//  Command handling
//
//  Earlier revisions read exactly ONE byte per loop() tick, from whichever
//  transport happened to have one — and serial silently overwrote ethernet in the
//  same tick. The host sends two bytes per command ("+\n"), so each command cost
//  two ticks (~3-6 ms) to drain. A burst of commands — which happens whenever the
//  host's camera thread stalls and then delivers several frames back-to-back —
//  accumulated in the RX buffer and the light sequence fell permanently behind,
//  with no way to catch up. Both transports are now drained fully every tick
//  through the same line assembler.
//
//  Commands are newline-terminated, which is what allows multi-character commands
//  (S…, E…). A single character that is not a multi-character prefix still
//  dispatches immediately without waiting for a newline, so an interactive serial
//  monitor keeps working exactly as before.
// =============================================================================

static char cmdLine[MAX_SCHEDULE_LEN + 16];
static uint8_t cmdLen = 0;

// Returns true if `c` starts a multi-character command and must be buffered.
static inline bool isMultiCharCommand(char c) { return (c == 'S' || c == 'E'); }

static void reportSafetyStatus()
{
  Serial.print(F("LIMITS on<=")); Serial.print(LIGHT_HARD_MAX_ON_US);
  Serial.print(F("us period>=")); Serial.print(LIGHT_HARD_MIN_PERIOD_US);
  Serial.print(F("us duty=1/"));  Serial.print(1u << LIGHT_DUTY_SHIFT);
  Serial.print(F(" on="));        Serial.print(lightTurnOffMicroseconds);
  Serial.print(F(" keepoff="));   Serial.print(lightKeepOffMicroseconds);
  Serial.print(F(" subs="));      Serial.print(lightSubstitutions);
  Serial.print(F(" starved="));   Serial.print(lightStarvedFrames);
  Serial.print(F(" wdt="));       Serial.print(lightWatchdogTrips);
  Serial.print(F(" budget="));
  for (uint8_t i = 0; i < NUMBER_OF_LIGHTS; i++)
  { Serial.print(lightBudgetUs[i]); if (i < NUMBER_OF_LIGHTS - 1) Serial.print('/'); }
  Serial.println();
}

// Handle one complete command. `line` is NUL-terminated, `len` excludes the NUL.
static void handleCommand(const char *line, uint8_t len, unsigned long &currentTime,
                          unsigned int buttonRaw1, unsigned int buttonRaw2)
{
  if (len == 0) return;

  switch (line[0])
  {
    case 'v': version(); break;

    case 'h':
    case 'i':
      reset_millis();
      serialOutputEnabled = true;
      currentTime         = 0;
      lightStartTime      = 0;
      autoLights          = STEP_TIMER;
      lightOn             = 0;
      lightToActivate     = 0;  // auto-cycle will set this on the first tick
      break;

    case '0': deactivateLights(); autoLights = STEP_EXTERNAL;                break;

    // '1'..'6' name the next COB explicitly (the host's --skip path). They take over
    // stepping exactly like '+', so they must also report like '+' — otherwise the
    // host loses distances/buttons for the whole run.
    case '1': case '2': case '3': case '4': case '5': case '6':
      activateLight((uint8_t)(line[0] - '1'));
      autoLights = STEP_EXTERNAL;
      reportState(currentTime, buttonRaw1, buttonRaw2);
      break;

    // Pattern selection is now independent of who does the stepping, so 'r'/'t'
    // survive a subsequent '+' instead of being silently overwritten by it.
    case 'r': lightPattern = PATTERN_ROUNDROBIN; autoLights = STEP_TIMER; lightOn = 0; break;
    case 't': lightPattern = PATTERN_OPPOSITE;   autoLights = STEP_TIMER; lightOn = 0; break;
    case 'a': autoLights = STEP_DEPTH;                                     break;
    case 'y': autoLights = STEP_TIMED; lightOn = 0; flashStartTime = currentTime; break;

    // ── Exposure-locked mode — the controller owns sequencing ────────────────
    // The light advances once per exposure, in the guard-rail callback. The host
    // stops sending per-frame steps entirely; it only uploads schedules.
    case 'e':
      setScheduleFromPattern(lightPattern);
      autoLights          = STEP_EXPOSURE;
      serialOutputEnabled = true;
      break;

    // 'o' resets, 'p' extends the max light-on window. Both are clamped now:
    // the old 'p' incremented without bound, which could push the pulse width
    // past the COB's survivable limit and, beyond the guard rail, to 100% duty.
    case 'o': lightTurnOffMicroseconds  = LIGHT_TURNOFF_US; clampLightTiming(); break;
    case 'p': lightTurnOffMicroseconds += 500;              clampLightTiming(); break;

    // 'E<microseconds>' — set the max light-on window explicitly. The host derives
    // this from its configured camera exposure so the two cannot silently drift
    // apart. Always clamped to the hard ceiling regardless of what was asked for.
    case 'E':
    {
      long v = atol(line + 1);
      if (v > 0)
      {
        lightTurnOffMicroseconds = (uint32_t)v;
        clampLightTiming();
      }
      Serial.print(F("ON=")); Serial.println(lightTurnOffMicroseconds);
      break;
    }

    // 'S<digits>' — upload a strobe schedule, e.g. "S031425". Digits are 0-indexed
    // COBs. Applied at the next cycle boundary, so arrival timing does not matter.
    case 'S':
    {
      uint8_t seq[MAX_SCHEDULE_LEN];
      uint8_t n = 0;
      for (uint8_t i = 1; i < len && n < MAX_SCHEDULE_LEN; i++)
      {
        if (line[i] < '0' || line[i] > '9') continue;
        seq[n++] = (uint8_t)(line[i] - '0');
      }
      uint8_t accepted = setLightSchedule(seq, n);
      Serial.print(F("SCHED=")); Serial.println(accepted);
      break;
    }

    case 'Q': reportSafetyStatus(); break;

    case 'f': serialOutputEnabled = false;                     break;

    // Deferred to the main loop, which calls performSoftwareReset(): the reset
    // path stops the lights and then spins waiting for the watchdog, so it must
    // not run from inside the command drain.
    case 'z':
      Serial.println(F("DEBUG: software reset requested"));
      Serial.flush();

    #if USE_W5500
      if (client) {
        client.println(F("DEBUG: software reset requested"));
        client.flush();
      }
    #endif

      serialOutputEnabled = false;
      autoLights = STEP_EXTERNAL;
      softwareResetRequested = true;
      break;

    case '+':
      autoLights = STEP_EXTERNAL;
      lightOn    = getNextLight(lightOn, NUMBER_OF_LIGHTS, lightPattern);
      activateLight(lightOn);
      lightStartTime = currentTime;
      reportState(currentTime, buttonRaw1, buttonRaw2);
      break;

#if USE_W5500
    case 'x':
      Serial.print(F("ETH IP: ")); Serial.println(Ethernet.localIP());
      Serial.print(F("ETH HW: ")); Serial.println(Ethernet.hardwareStatus());
      sram();
      break;
#endif
  }
}

// Feed one received byte through the line assembler, dispatching complete commands.
static void feedCommandByte(char c, unsigned long &currentTime,
                            unsigned int buttonRaw1, unsigned int buttonRaw2)
{
  if (c == '\n' || c == '\r')
  {
    if (cmdLen > 0)
    {
      cmdLine[cmdLen] = 0;
      handleCommand(cmdLine, cmdLen, currentTime, buttonRaw1, buttonRaw2);
      cmdLen = 0;
    }
    return;
  }

  if (cmdLen < sizeof(cmdLine) - 1) cmdLine[cmdLen++] = c;

  // A lone character that cannot start a longer command dispatches straight away,
  // so a human on a serial monitor with no line ending still gets a response.
  if (cmdLen == 1 && !isMultiCharCommand(cmdLine[0]))
  {
    cmdLine[1] = 0;
    handleCommand(cmdLine, 1, currentTime, buttonRaw1, buttonRaw2);
    cmdLen = 0;
  }
}

// =============================================================================
//  setup()
// =============================================================================
void setup()
{
  bootBlinkStart();

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

  // ── VL53L5CX data-ready interrupt inputs ─────────────────────────────────────
  // GP12/GP13/GP14 receive short active-low data-ready pulses from the three
  // VL53L5CX sensors. Configure them as plain inputs now, but do not attach
  // interrupts yet. ToF interrupts are attached only after the full ToF boot,
  // startRanging(), and safe-first-frame clearing sequence.
#if USE_VL53L5CX
  pinMode(SENSOR0_INT_PIN, INPUT);
  pinMode(SENSOR1_INT_PIN, INPUT);
  pinMode(SENSOR2_INT_PIN, INPUT);
#endif

  // ── Ethernet INT input ────────────────────────────────────────────────────────
  // The W5500 asserts GP22 LOW when it needs servicing. One pull-up, already
  // present in the W5500 Lite Board, ensures the line idles HIGH when no
  // interrupt is pending.  Interrupt-driven RX is not yet implemented;
  // this pin is reserved for future use. We leave here the internal pullup
  // enabled in the case the W5500 Lite Board is physically removed.
#if USE_W5500
  pinMode(W5500_INT_PIN, INPUT_PULLUP);
#endif

  // ── I²C bus + MCP23018 ───────────────────────────────────────────────────────
  // Wire must be configured before initMCP23018() because the function uses
  // Wire.beginTransmission() internally.  Wire is also used by the VL53L5CX
  // sensors, but we initialise it here (not inside setI2CDistanceAddresses)
  // so the MCP23018 — and specifically its LPn outputs — are ready before we
  // attempt the sensor address-sequencing procedure hereafter.

  recoverI2CBus();

  Wire.setSDA(SDA_PIN);
  Wire.setSCL(SCL_PIN);
  Wire.begin();

  // 1 MHz Fast-Mode Plus (FM+): the RP2350 I2C peripheral tops out at FM+ regardless
  // of device capability.  MCP23018 supports 3.4 MHz HS mode per its datasheet, but
  // HS mode requires a special master-code preamble that the RP2350 hardware does not
  // implement; FM+ at 1 MHz is therefore the highest speed achievable on this bus.
  // VL53L5CX also supports FM+.  Pull-ups should be ≤1 kΩ for reliable FM+ signalling.
  // After a dedicated assessment on this regard, it turns out that the current I2C bus
  // load for the system, including the Pico 2, the MCP23018 and the three VL53L5CX
  // as I2C active nodes, is asymmetrical, due to the presence of Mosfet Level Shifters
  // in the VL53L5CX boards (which is actually a very favorable feature). We have, for
  // both the SDA and SCL lines, a dynamic impedance of ~1.2 kΩ, when the lines are in
  // their HIGH state, and ~670 Ω, when the lines are LOW.
  Wire.setClock(1000000);

  // initMCP23018() performs a hardware reset, configures directions and pull-ups,
  // and drives all outputs to a safe initial state:
  //   GPA2 (W5500RST)  HIGH  → one explicit and controlled reset cycle for the
  //         W5500 Lite Board is then required before Ethernet.init()
  //   GPA3-5 (LPn)  HIGH  → similarly, all VL53L5CX sensors are out from their
  //         Low Power mode, needing a LPn cycling and a dedicated setup stage,
  //         as carried out in the above setI2CDistanceAddresses() procedure.
  //   GPA6-7 (rails)  LOW  → power rails on. Both lines, driving the +5V and
  //         the +11.5V power rails, are softly tied to GND (rails enabled),
  //         at the powerup stage, by means of external 680K resistors,
  //         and then actively. The pull-ups management on both lines is dynamic.
  initMCP23018();

  // Configure MCP23018 PORTA pushbutton interrupt/debounce state.
  // This must be done after initMCP23018(), because initMCP23018()
  // resets and configures the MCP23018 registers.
  if (mcp23018_present)
  {
    initMcp23018ButtonStateAndInterrupt();
  }


  // ── VL53L5CX time-of-flight sensors ──────────────────────────────────────────
  // setI2CDistanceAddresses() sequences each sensor's LPn line to bring sensors
  // up one at a time and assign unique I²C addresses (0x30, 0x31, 0x32).
  // At completion, all three sensors are ranging at 10 Hz.
#if USE_VL53L5CX
  DBG_VL53L5CXLN(F("ToF: initialising sensors..."));
  setI2CDistanceAddresses();
  tofPrintInitSummary();

  // Attach ToF interrupts only after the complete ToF boot phase.
  // At this point each working sensor has completed begin/attach,
  // setResolution(64), setRangingFrequency(10), startRanging(),
  // and safe-first-frame clearing.
  attachTofInterruptsForWorkingSensors();
#endif


  // ── Ethernet ─────────────────────────────────────────────────────────────────
  // Ethernet initialisation is intentionally last, because Ethernet.begin()
  // execution time can be quite long, in some conditions. This time depends on
  // the network configuration, ranging from under a second to over a minute. With
  // Static IP the time is approx. 5 milliseconds; with DHCP and the server
  // immediately ready, the time is approx 2 to 5 seconds (Successful connection).
#if USE_W5500
  #if DEBUG_W5500
  Serial.println(F("ETH: initialising W5500..."));
  sram();
  #endif

  // W5500 HW reset cycling
  // Place the W5500 Lite board in reset via MCP23018 GPA2.
  // The open-drain output actively push RST LOW;
  mcp23018ResetW5500(true);
  delay(W5500_RST_SLEEP_MS);  // Hold W5500 in reset for 2 ms.

  // Release W5500 from reset via MCP23018 GPA2.
  // The open-drain output stops pulling RST LOW; the external pull-up on the RST
  // line brings it HIGH and the W5500 begins its internal initialisation sequence.
  mcp23018ResetW5500(false);
  delay(W5500_RST_RESUME_MS);  // Allow W5500 PLL and PHY to stabilyse (2 ms.)

  // Select the correct SPI chip-select pin and assign a static IP address.
  // Ethernet.init() must be called before Ethernet.begin() on all non-standard
  // CS pins; it stores the pin number in the library for all subsequent SPI ops.
  pinMode(W5500_CS_PIN, OUTPUT);
  digitalWrite(W5500_CS_PIN, HIGH);

  SPI.setRX(16);
  SPI.setTX(19);
  SPI.setSCK(18);

  SPI.begin();
  Ethernet.init(W5500_CS_PIN);

  // From this moment onwards, SPI transactions with the W5500 Lite board can start.
  Ethernet.begin(mac, ip, dns, gateway, subnet);

  // Ethernet diagnostics must distinguish between:
  //   - requested software IP configuration;
  //   - actual hardware detection;
  //   - actual IP state after Ethernet.begin();
  //   - server.begin() being issued.
  //
  // Do NOT print "server listening" just because the firmware variable 'ip'
  // contains the requested static address. That message is misleading if the
  // W5500 is not actually detected over SPI.
  int ethHwStatus   = Ethernet.hardwareStatus();
  int ethLinkStatus = Ethernet.linkStatus();

  Serial.print(F("ETH requested IP: "));
  Serial.println(ip);

  Serial.print(F("ETH HW status: "));
  Serial.println(ethHwStatus);

  Serial.print(F("ETH link status: "));
  Serial.println(ethLinkStatus);

  Serial.print(F("ETH local IP after begin: "));
  Serial.println(Ethernet.localIP());

  Serial.print(F("ETH subnet after begin: "));
  Serial.println(Ethernet.subnetMask());

  Serial.print(F("ETH gateway after begin: "));
  Serial.println(Ethernet.gatewayIP());

  if (ethHwStatus == EthernetNoHardware)
  {
    Serial.println(F("ETH ERROR: W5500 not detected, server NOT started"));
  }
  else
  {
    // Open the telnet-style command server on port 23.
    // Clients connect with: nc <ip> 23
    server.begin();

    Serial.print(F("ETH: server.begin() executed on actual IP "));
    Serial.println(Ethernet.localIP());
  }

  #if DEBUG_W5500
  sram();
  #endif
#endif

  // ── Exposure ISR ─────────────────────────────────────────────────────────────
  // Timing parameters must be set before the ISR is armed so the first callback
  // sees valid values. All times are in µs from the rising edge.

  lightDebouncingMicroseconds = LIGHT_DEBOUNCING_US;  // µs: debounce before arming falling-edge ISR
  lightTurnOffMicroseconds    = LIGHT_TURNOFF_US;     // µs: maximum light-on window from rising edge
  lightKeepOffMicroseconds    = LIGHT_KEEPOFF_US;     // µs: guard-rail before rising-edge ISR re-arms
  clampLightTiming();

  // Fill the per-COB thermal buckets before any pulse can be scheduled. This must
  // happen before attachInterrupt() below, or the first exposure would price its
  // COB against an all-zero budget and be needlessly substituted away.
  resetLightBudgets();

  // ── BOOT DARK ────────────────────────────────────────────────────────────────
  // Load the default schedule CONTENTS so that an 'e' arriving before any 'S'
  // has a coherent cycle to walk, then explicitly disarm: setLightSchedule()
  // latches lightOn/lightToActivate as a side effect, and nothing may be armed
  // at boot. With lightToActivate == 0 the exposure ISR returns immediately on
  // every rising edge, so the COBs stay off no matter what the camera is doing.
  //
  // The lights are released only by an explicit host command — which is what the
  // grabber's startup handshake sends (see multiModalGrabber.c / arduinoSensor.c):
  //
  //     v            probe version banner        -> stays dark
  //     i            start stream                -> timer-stepped cycling begins
  //     r / t / a    lighting mode               -> pattern + timer stepping
  //     E<us>        light-on ceiling            -> stays dark, clamps only
  //     e            hand sequencing to the ISR  -> exposure-locked cycling begins
  //     S<digits>    upload strobe schedule      -> arms the first entry
  //     +            legacy per-frame step       -> arms one step
  //     1..6         manual select               -> arms that COB
  //
  // A board that is powered up without a host therefore never strobes, however
  // long the camera runs.
  setScheduleFromPattern(lightPattern);
  lightOn         = NO_LIGHT_ON;
  lightToActivate = 0;

  // Register the unified ISR for RISING.  The same callback is also invoked for
  // FALLING when debouncingCallback later enables GPIO_IRQ_EDGE_FALL via
  // gpio_set_irq_enabled(); the pico-sdk dispatches both event types to the
  // registered handler without requiring a separate attachInterrupt() call.
  attachInterrupt(digitalPinToInterrupt(CAMERA_EXPOSURE_PIN), exposureISR, RISING);

  // Raise IO_IRQ_BANK0 (all GPIO bank-0 interrupts) to the highest available
  // NVIC priority on the Cortex-M33 (RP2350).  Priority 0 cannot be preempted
  // by any other maskable interrupt on the same core, giving the exposure ISR
  // the minimum possible response latency. delay() and delayMicroseconds() on
  // RP2350 use busy-wait loops that do not disable interrupts, so this ISR will
  // preempt them as well.
  irq_set_priority(IO_IRQ_BANK0, PICO_HIGHEST_IRQ_PRIORITY);

  Serial.println(F("== boot complete =="));
  bootBlinkComplete();

  // After boot-complete visual marker, hand over the board LED to PB state.
  updateBoardLedFromButtons();
}


// =============================================================================
//  loop()
// =============================================================================
void loop()
{
  unsigned long currentTime = millis();

  // Consume VL53L5CX data-ready events as soon as possible in the fast loop.
  // The ISR only sets dataReadyFlags[i]; all I2C data retrieval is performed here.
#if USE_VL53L5CX
  processTofInterruptEvents();
#endif

  // ── Light safety watchdog ────────────────────────────────────────────────────
  // Runs unconditionally, before anything can consume the tick. Never put this
  // behind a mode check or feature flag — it is what stops an overvolted COB from
  // being held on if the alarm chain ever fails.
  lightSafetyWatchdog();

  if (softwareResetRequested)
  {
    softwareResetRequested = false;
    performSoftwareReset();
  }

  // Light activation and deactivation are fully handled by exposureISR() and its
  // timer callbacks (lightOffCallback / guardRailCallback).  The main loop only
  // updates lightToActivate via activateLight() / deactivateLights().

  // ── Button state from MCP23018 interrupt/debounce logic ──────────────────────
  // GPA0/GPA1 are managed by an interrupt-driven debounce state machine.
  // No MCP23018 GPIOA polling is performed here.
  unsigned int buttonRaw1 = debouncedPB0 ? 1 : 0;
  unsigned int buttonRaw2 = debouncedPB1 ? 1 : 0;

  // ── Ethernet client management ───────────────────────────────────────────────
#if USE_W5500
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
#endif

  // ── Command intake ───────────────────────────────────────────────────────────
  // Drain both transports fully every tick. Taking a single byte from each and
  // letting serial overwrite ethernet in the same tick silently discarded the
  // ethernet byte; both are now fed through the same line assembler instead.
#if USE_W5500
  {
    uint32_t clientDrainBytes = 0;

    while (client &&
           client.available() > 0 &&
           clientDrainBytes < MAX_CLIENT_BYTES_PER_LOOP)
    {
      feedCommandByte((char)client.read(),
                      currentTime,
                      buttonRaw1,
                      buttonRaw2);

      clientDrainBytes++;
    }
  }
#endif

  uint32_t serialDrainBytes = 0;

  while (Serial.available() > 0 &&
         serialDrainBytes < MAX_SERIAL_BYTES_PER_LOOP)
  {
    feedCommandByte((char)Serial.read(),
                    currentTime,
                    buttonRaw1,
                    buttonRaw2);

    serialDrainBytes++;
  }

  // Command protocol (newline-terminated; single chars also dispatch immediately):
  //   v        — print firmware version
  //   h / i    — reset timer, enable output, start timer-driven light cycling
  //   0        — all lights off, stop auto cycle
  //   1–6      — activate specific light (manual mode)
  //   r        — round-robin pattern, timer-stepped
  //   t        — opposite-pair pattern, timer-stepped
  //   a        — depth-guided auto mode (uses ToF readings)
  //   y        — timed flash mode (cycles for 10 s then stops)
  //   e        — EXPOSURE-LOCKED: controller steps the schedule once per exposure
  //   S<digits>— upload strobe schedule, 0-indexed COBs, e.g. "S031425"
  //   E<us>    — set max light-on window in µs (hard-clamped)
  //   Q        — print safety limits, throttle counters and per-COB budgets
  //   +        — advance one light step (legacy host-stepped path)
  //   o        — reset maximum light-on time to default (1000 µs)
  //   p        — extend maximum light-on time by 500 µs (hard-clamped)
  //   f        — disable serial output stream
  //   z        — all off + watchdog software reset
  //   x        — print Ethernet IP and hardware status

  // ── Application tick ─────────────────────────────────────────────────────────
  // The 20 ms cadence (loopRateMsec) drives the MCP23018 push-button debounce
  // state machine. It must keep running in EVERY mode, including exposure-locked
  // mode, because the debouncer is what advances PB0/PB1 after an INTA event.
  bool applicationTick = (currentTime - lastUpdateTime >= loopRateMsec);

  if (applicationTick)
  {
    lastUpdateTime = currentTime;

    // Process MCP23018 PB0/PB1 interrupt-driven debounce.
    processMcp23018ButtonsDebounce();

    // Refresh local report variables after debounce processing.
    buttonRaw1 = debouncedPB0 ? 1 : 0;
    buttonRaw2 = debouncedPB1 ? 1 : 0;

    // ToF acquisition is ISR-driven and handled in the fast loop by
    // processTofInterruptEvents(). Do not poll VL53L5CX sensors here.
  }

  // ── Per-strobe reporting ─────────────────────────────────────────────────────
  // In exposure-locked mode the authoritative event is the strobe, not the timer,
  // so emit exactly one line per pulse. This is what gives the host a countable
  // frame↔COB mapping instead of a timestamp correlation across the GigE pipeline.
  if (autoLights == STEP_EXPOSURE)
  {
    uint32_t sc = strobeCounter;
    if (sc != lastReportedStrobe)
    {
      lastReportedStrobe = sc;

      if (serialOutputEnabled)
        reportState(currentTime, buttonRaw1, buttonRaw2);
    }
  }
  else if (applicationTick)
  {
    // Automatic light cycling and periodic state reporting
    if (autoLights != STEP_EXTERNAL && (currentTime - lightStartTime >= lightDurationMsec))
    {
      lightStartTime = currentTime;

      if (autoLights == STEP_TIMER || autoLights == STEP_TIMED)
      {
        lightOn = getNextLight(lightOn, NUMBER_OF_LIGHTS, lightPattern);
        activateLight(lightOn);
        // Timed flash — cycle for 10 s then stop automatically
        if (autoLights == STEP_TIMED && (currentTime - flashStartTime >= 10000))
        {
          autoLights = STEP_EXTERNAL;
          deactivateLights();
        }
      }
#if ENABLE_LASER_LIGHT_DECISION
      else if (autoLights == STEP_DEPTH)
      {
        lightOn = chooseClosestLight(laser_distance_millimeters);
        activateLight(lightOn);
      }
#endif

      if (serialOutputEnabled)
        reportState(currentTime, buttonRaw1, buttonRaw2);
    }
  }

  // ── Yield / sleep ─────────────────────────────────────────────────────────────
  // Brief pause to avoid hammering the I²C bus and the W5500.
  // Strobe timing is now ISR-driven, so no compensation is needed here.
  // The exposure ISR can preempt this delay at any time.
  delayMicroseconds(CPU_SLEEP_US);
}

// =============================================================================
//  End of CameraControllerMultizone Core Compilation Unit
// =============================================================================
