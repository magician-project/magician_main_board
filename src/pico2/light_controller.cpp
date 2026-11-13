// =============================================================================
//  MagicianCam4 — Rev. 2.1
//  COB LED Lighting Controller Implementation: Single-Cycle SIO & Hardware Timer Guardrails
// =============================================================================

#include "light_controller.h"

// GPIO pins assigned to COB LED channels 1..6
const uint8_t directLightControl[NUMBER_OF_LIGHTS] = {6, 7, 8, 9, 10, 11};

volatile uint8_t lightOn = NO_LIGHT_ON;                      // Selected COB (host/timer intent; in 'e' mode the next entry)
volatile LightCycleState lightCycleState = LIGHT_STATE_READY;// Exposure state machine phase
int lightHardwareAlarmNum = -1;                              // Claimed RP2350 hardware timer alarm ID
uint64_t risingEdgeTimeUs = 0;                               // Exposure rising edge timestamp t0 (us)

unsigned char autoLights = STEP_EXTERNAL;                    // Active stepping driver mode
unsigned char lightPattern = PATTERN_ROUNDROBIN;             // Active stepping pattern
unsigned int lightDurationMsec = 100;                        // Active COB dwell time (ms)
unsigned long loopRateMsec = 20;                             // Core 0 loop tick period (20 ms = 50 Hz)
unsigned long lastUpdateTime = 0;                            // Timestamp of last loop tick (ms)
unsigned long lightStartTime = 0;                            // Timestamp of last light advance (ms)
unsigned long flashStartTime = 0;                            // Timestamp when timed-flash mode was armed

// =============================================================================
//  Rev 2.2 real-time design (no interrupt-off sections added to 2.1)
// =============================================================================
//  * Armed-fire word: ONE 32-bit word says what the next rising edge fires. The rising-edge
//    fast path is 2.1's (load, test, SIO set) plus a single AND to extract the pin mask.
//      bits  0..3  COB index that fires (ARM_NO_LIGHT = none)
//      bit   4     FORCED : this COB was swapped in by the 'N' limit
//      bit   5     STARVED: 'N' limit reached and nothing else allowed (pin mask is 0)
//      bits  6..11 SIO pin mask (GP6..GP11), exactly LIGHT_ALL_PINS_MASK bits
//      bits 16..31 sequence, incremented on every ISR write
//  * Every 2.2 decision (consecutive limit, substitution, 'e' stepping) is taken AFTER the
//    20 us guard alarm is armed, inside the guard window: both GP2 edges are disabled there
//    and the only pending event is that alarm, which is already in the hardware. Bounded:
//    at most LIGHT_SCHEDULE_MAX_LENGTH compare steps, no division.
//  * The core-0 loop arms a light with a compare-and-swap on the armed-fire word
//    (LDREX/STREX, interrupts stay enabled). If the ISR ran meanwhile the sequence field
//    differs, the CAS fails and the loop recomputes from fresh state. The ISR never waits.
//  * Schedules are double-buffered: the loop fills the bank the ISR is not reading, then
//    publishes {bank, length, generation} with a single 32-bit store.
// =============================================================================
#define ARM_LIGHT_FIELD     0x0000000Fu
#define ARM_NO_LIGHT        0x0000000Fu
#define ARM_FORCED_BIT      0x00000010u
#define ARM_STARVED_BIT     0x00000020u
#define ARM_SEQUENCE_UNIT   0x00010000u
#define ARM_SEQUENCE_FIELD  0xFFFF0000u

#define SCHEDULE_BANK_FIELD       0x00000001u
#define SCHEDULE_LENGTH_SHIFT     1u
#define SCHEDULE_LENGTH_FIELD     0x7Fu
#define SCHEDULE_GENERATION_SHIFT 8u

static std::atomic<uint32_t> armedFire{ARM_NO_LIGHT};
static uint32_t lightPinMask[NUMBER_OF_LIGHTS];               // 1u << GPIO for each COB (SRAM)
static uint8_t lightScheduleBank[2][LIGHT_SCHEDULE_MAX_LENGTH] = {{0, 1, 2, 3, 4, 5}, {0, 1, 2, 3, 4, 5}};
static std::atomic<uint32_t> lightSchedulePublished{(uint32_t)NUMBER_OF_LIGHTS << SCHEDULE_LENGTH_SHIFT};

// Written only by the rising-edge ISR (and by deactivateLights() while the edges are disabled)
static volatile uint8_t lastFiredLight = NO_LIGHT_ON;        // COB that fired on the previous exposure
static volatile uint32_t consecutiveFireCount = 0;           // How many exposures in a row it has fired
static volatile uint32_t isrScheduleCursor = 0;              // Schedule index of the entry that fired last
static volatile uint32_t isrScheduleGeneration = 0;          // Schedule generation the ISR is walking
static volatile uint32_t pulseCeilingUs = LIGHT_TURNOFF_US;  // 'E' ceiling latched for the current pulse

/// @brief Decides what the next exposure fires, applying the 'N' consecutive-fire limit.
/// @param wanted COB the active mode asks for.
/// @param published Snapshot of lightSchedulePublished (substitutes come from that schedule).
/// @param cursor When non-null, the scan starts after *cursor and *cursor receives the substitute's index.
/// @return Armed-fire word without the sequence field.
static inline uint32_t __not_in_flash_func(chooseArmedFire)(uint8_t wanted, uint32_t published, volatile uint32_t *cursor)
{
  if (wanted >= NUMBER_OF_LIGHTS) return ARM_NO_LIGHT;

  uint32_t limit = lightMaxConsecutiveActivations.load(std::memory_order_relaxed);
  if ((limit == 0) || (wanted != lastFiredLight) || (consecutiveFireCount < limit))
  {
    return lightPinMask[wanted] | wanted;
  }

  const uint8_t *bank = lightScheduleBank[published & SCHEDULE_BANK_FIELD];
  uint32_t length = (published >> SCHEDULE_LENGTH_SHIFT) & SCHEDULE_LENGTH_FIELD;
  uint32_t index = (cursor != nullptr) ? *cursor : (length - 1u); // no cursor: scan from entry 0
  for (uint32_t step = 0; step < length; step++)
  {
    index++;
    if (index >= length) index = 0;
    uint8_t candidate = bank[index];
    if (candidate != wanted)
    {
      if (cursor != nullptr) *cursor = index;
      return lightPinMask[candidate] | candidate | ARM_FORCED_BIT;
    }
  }
  return ARM_STARVED_BIT | wanted;
}

/// @brief Core-0 loop side: arms `wanted` for the next exposure without disabling interrupts.
static void armFromLoop(uint8_t wanted)
{
  uint32_t current = armedFire.load(std::memory_order_relaxed);
  uint32_t desired;
  do
  {
    desired = chooseArmedFire(wanted, lightSchedulePublished.load(std::memory_order_relaxed), nullptr) |
              (current & ARM_SEQUENCE_FIELD);
  } while (!armedFire.compare_exchange_weak(current, desired, std::memory_order_relaxed, std::memory_order_relaxed));
}

/// @brief Core-0 loop side, 'e' mode: makes schedule entry 0 fire next after a new generation.
/// @note If the ISR has already fired since the generation was published, it has restarted the
///       walk itself (the light it fired counts as entry 0) and its arming is kept.
static void restartScheduleFromLoop()
{
  uint32_t published = lightSchedulePublished.load(std::memory_order_relaxed);
  uint32_t generation = published >> SCHEDULE_GENERATION_SHIFT;
  uint32_t current = armedFire.load(std::memory_order_relaxed);
  if (isrScheduleGeneration == generation) return;

  uint8_t first = lightScheduleBank[published & SCHEDULE_BANK_FIELD][0];
  uint32_t desired = chooseArmedFire(first, published, nullptr) | (current & ARM_SEQUENCE_FIELD);
  if (armedFire.compare_exchange_strong(current, desired, std::memory_order_relaxed, std::memory_order_relaxed))
  {
    lightOn = (uint8_t)(desired & ARM_LIGHT_FIELD);
  }
  // CAS failure: the ISR fired after the generation was published and re-armed per the new schedule.
}

/// @brief Configures direct GPIO pins, claims hardware alarm, and registers exposure ISR.
/// @note Sets NVIC Priority 0 (highest) on IO_IRQ_BANK0 and hardware timer interrupt.
void initLightController()
{
  for (uint8_t i = 0; i < NUMBER_OF_LIGHTS; i++)
  {
    digitalWrite(directLightControl[i], LOW);
    pinMode(directLightControl[i], OUTPUT);
    lightPinMask[i] = (1u << directLightControl[i]);
  }

  lightOn = NO_LIGHT_ON;
  armedFire.store(ARM_NO_LIGHT, std::memory_order_relaxed);
  lightCycleState = LIGHT_STATE_READY;

  lightHardwareAlarmNum = hardware_alarm_claim_unused(true);
  if (lightHardwareAlarmNum >= 0)
  {
    hardware_alarm_set_callback((uint)lightHardwareAlarmNum, lightHardwareTimerCallback);
#if defined(TIMER0_IRQ_0)
    irq_set_priority(TIMER0_IRQ_0 + (uint)lightHardwareAlarmNum, PICO_HIGHEST_IRQ_PRIORITY);
#elif defined(TIMER_IRQ_0)
    irq_set_priority(TIMER_IRQ_0 + (uint)lightHardwareAlarmNum, PICO_HIGHEST_IRQ_PRIORITY);
#endif
  }

  attachInterrupt(digitalPinToInterrupt(CAMERA_EXPOSURE_PIN), exposureISR, RISING);
  irq_set_priority(IO_IRQ_BANK0, PICO_HIGHEST_IRQ_PRIORITY);
}

/// @brief Schedules a specified COB LED to be gated on the next exposure rising edge.
/// @param lightIdx 0-indexed COB number (0..5). The 'N' limit may arm a substitute instead.
void activateLight(uint8_t lightIdx)
{
  if (lightIdx < NUMBER_OF_LIGHTS)
  {
    lightOn = lightIdx;
    armFromLoop(lightIdx);
  }
}

/// @brief Cuts all COB outputs in 1 cycle, cancels pending timer alarms, and re-arms RISE edge.
void deactivateLights()
{
  // 1. Disable both exposure interrupt edges on GP2
  gpio_set_irq_enabled(CAMERA_EXPOSURE_PIN, GPIO_IRQ_EDGE_FALL, false);
  gpio_set_irq_enabled(CAMERA_EXPOSURE_PIN, GPIO_IRQ_EDGE_RISE, false);

  // 2. Clear any pending latched edge flags
  gpio_acknowledge_irq(CAMERA_EXPOSURE_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL);

  // 3. Cancel any active hardware timer alarm
  if (lightHardwareAlarmNum >= 0)
  {
    hardware_alarm_cancel((uint)lightHardwareAlarmNum);
  }

  // 4. Force all 6 COBs LOW simultaneously in 1 CPU cycle
  sio_hw->gpio_clr = LIGHT_ALL_PINS_MASK;

  lightCycleState = LIGHT_STATE_KEEP_OFF;
  lightOn         = NO_LIGHT_ON;

  // Edges are disabled and the alarm is cancelled, so no ISR can run here: plain stores are safe.
  armedFire.store((armedFire.load(std::memory_order_relaxed) & ARM_SEQUENCE_FIELD) | ARM_NO_LIGHT,
                  std::memory_order_relaxed);
  lastFiredLight       = NO_LIGHT_ON; // Lights off ends any run
  consecutiveFireCount = 0;

  // 5. Re-arm RISE interrupt only after physical shutdown is complete
  lightCycleState = LIGHT_STATE_READY;
  gpio_set_irq_enabled(CAMERA_EXPOSURE_PIN, GPIO_IRQ_EDGE_RISE, true);
}

/// @brief Publishes a new light schedule (core-0 loop context, interrupts stay enabled).
/// @param schedule 0-indexed COBs; entries >= NUMBER_OF_LIGHTS are dropped.
/// @param length Number of entries (clamped to LIGHT_SCHEDULE_MAX_LENGTH). Empty input is ignored.
void setLightSchedule(const uint8_t *schedule, uint32_t length)
{
  if (schedule == nullptr) return;
  if (length > LIGHT_SCHEDULE_MAX_LENGTH) length = LIGHT_SCHEDULE_MAX_LENGTH;

  uint32_t published = lightSchedulePublished.load(std::memory_order_relaxed);
  uint32_t bank = (published & SCHEDULE_BANK_FIELD) ^ 1u; // the bank the ISR is NOT reading

  uint32_t count = 0;
  for (uint32_t i = 0; i < length; i++)
  {
    if (schedule[i] < NUMBER_OF_LIGHTS) lightScheduleBank[bank][count++] = schedule[i];
  }
  if (count == 0) return;

  uint32_t generation = (published >> SCHEDULE_GENERATION_SHIFT) + 1u;
  lightSchedulePublished.store((generation << SCHEDULE_GENERATION_SHIFT) |
                               (count << SCHEDULE_LENGTH_SHIFT) | bank,
                               std::memory_order_relaxed);

  if (autoLights == STEP_EXPOSURE) restartScheduleFromLoop();
}

/// @brief Hands light sequencing to the exposure ISR: one schedule step per exposure,
///        starting from schedule entry 0.
void enterExposureLockedMode()
{
  autoLights = STEP_EXPOSURE;

  // Same bank and length, new generation: tells the ISR to restart the walk
  uint32_t published = lightSchedulePublished.load(std::memory_order_relaxed);
  uint32_t generation = (published >> SCHEDULE_GENERATION_SHIFT) + 1u;
  lightSchedulePublished.store((generation << SCHEDULE_GENERATION_SHIFT) |
                               (published & ((1u << SCHEDULE_GENERATION_SHIFT) - 1u)),
                               std::memory_order_relaxed);
  restartScheduleFromLoop();
}

/// @brief Copies light-controller state for the diagnostic report (Core 1, no locking).
/// @note A value can be one exposure stale, and a schedule published at the same instant can
///       show mixed entries. Diagnostic use only; Core 0 is never blocked.
void copyLightControllerDiagnostics(LightControllerDiagnostics *out)
{
  if (out == nullptr) return;
  uint32_t published = lightSchedulePublished.load(std::memory_order_relaxed);
  uint32_t length = (published >> SCHEDULE_LENGTH_SHIFT) & SCHEDULE_LENGTH_FIELD;
  if (length > LIGHT_SCHEDULE_MAX_LENGTH) length = LIGHT_SCHEDULE_MAX_LENGTH;

  out->mode = autoLights;
  out->lastFiredLight = lastFiredLight;
  out->consecutiveFireCount = consecutiveFireCount;
  out->scheduleLength = length;
  out->scheduleCursor = isrScheduleCursor;
  out->armedWord = armedFire.load(std::memory_order_relaxed);
  for (uint32_t i = 0; i < length; i++) out->schedule[i] = lightScheduleBank[published & SCHEDULE_BANK_FIELD][i];
}

/// @brief Computes the next COB LED index based on current light and active pattern.
/// @param current Currently active COB index (0..5 or NO_LIGHT_ON).
/// @param total Total number of COBs (6).
/// @param pattern Active stepping pattern (PATTERN_ROUNDROBIN or PATTERN_OPPOSITE).
/// @return Next COB index (0..5).
uint8_t getNextLight(uint8_t current, uint8_t total, uint8_t pattern)
{
  if (current >= total) return 0;
  if (pattern == PATTERN_OPPOSITE)
  {
    const uint8_t alt[6] = {3, 4, 5, 1, 2, 0};
    if (current < 6) return alt[current];
  }
  return (current + 1) % total;
}

/// @brief Atomically acquires permission to close an active exposure pulse.
/// @note Resolves race between camera FALL edge and 1000 us timer timeout in SRAM.
static inline bool __not_in_flash_func(tryAcquirePulseClosure)()
{
  bool acquired = false;
  uint32_t interruptState = save_and_disable_interrupts();

  if (lightCycleState == LIGHT_STATE_ARMING_FALL || lightCycleState == LIGHT_STATE_WAIT_FALL)
  {
    lightCycleState = LIGHT_STATE_KEEP_OFF;
    acquired = true;
  }

  restore_interrupts(interruptState);
  return acquired;
}

/// @brief Physical pulse turn-off: clears all COBs in 1 cycle and arms 40 ms cooldown.
static inline void __not_in_flash_func(finishLightPulse)()
{
  // 1. Single-cycle global turn-off on all COB pins (fail-safe)
  sio_hw->gpio_clr = LIGHT_ALL_PINS_MASK;

  // 2. Disarm FALL interrupt and clear pending edge flags
  gpio_set_irq_enabled(CAMERA_EXPOSURE_PIN, GPIO_IRQ_EDGE_FALL, false);
  gpio_acknowledge_irq(CAMERA_EXPOSURE_PIN, GPIO_IRQ_EDGE_FALL);

  // 3. Schedule Step 3: 40 ms minimum cycle period (measured from t0 = risingEdgeTimeUs)
  if (lightHardwareAlarmNum >= 0)
  {
    hardware_alarm_set_target((uint)lightHardwareAlarmNum,
                              from_us_since_boot(risingEdgeTimeUs + LIGHT_CYCLE_PERIOD_US));
  }
}

/// @brief Dedicated hardware timer ISR for exposure guardrails (20 us guard, 1 ms timeout, 40 ms cooldown).
void __not_in_flash_func(lightHardwareTimerCallback)(uint alarm_num)
{
  (void)alarm_num;

  if (lightCycleState == LIGHT_STATE_ARMING_FALL)
  {
    // Step 1: 20 us noise guard window elapsed
    if (gpio_get(CAMERA_EXPOSURE_PIN) == 0)
    {
      // Shutter already LOW during guard window (< 20 us): close pulse immediately
      if (tryAcquirePulseClosure())
      {
        exposureShortPulseCount.fetch_add(1, std::memory_order_relaxed);
        finishLightPulse();
      }
      return;
    }

    // Exposure still HIGH: arm FALL interrupt and schedule Guardrail timeout (t0 + 1000 us)
    lightCycleState = LIGHT_STATE_WAIT_FALL;
    gpio_set_irq_enabled(CAMERA_EXPOSURE_PIN, GPIO_IRQ_EDGE_FALL, true);
    if (lightHardwareAlarmNum >= 0)
    {
      // Stage 2 timeout: 'E' ceiling latched by the rising-edge ISR for this pulse
      hardware_alarm_set_target((uint)lightHardwareAlarmNum,
                                from_us_since_boot(risingEdgeTimeUs + pulseCeilingUs));
    }
    return;
  }

  if (lightCycleState == LIGHT_STATE_WAIT_FALL)
  {
    // Step 2: Guardrail 1000 us timeout elapsed before FALL
    if (tryAcquirePulseClosure())
    {
      exposureGuardrailTimeoutCount.fetch_add(1, std::memory_order_relaxed);
      finishLightPulse();
    }
    return;
  }

  if (lightCycleState == LIGHT_STATE_KEEP_OFF)
  {
    // Step 3: 40 ms cycle period elapsed (t0 + 40000 us)
    exposureCycleCompleteCount.fetch_add(1, std::memory_order_relaxed);
    if (lightHardwareAlarmNum >= 0)
    {
      hardware_alarm_cancel((uint)lightHardwareAlarmNum);
    }
    gpio_acknowledge_irq(CAMERA_EXPOSURE_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL);
    lightCycleState = LIGHT_STATE_READY;
    gpio_set_irq_enabled(CAMERA_EXPOSURE_PIN, GPIO_IRQ_EDGE_RISE, true);
    return;
  }
}

/// @brief Rev 2.2 work done after the rising edge, inside the 20 us guard window.
/// @note Called by exposureISR() only after the COB is on, both GP2 edges are disabled and the
///       t0 + 20 us alarm is armed. Separate and noinline so that the ISR's entry (which runs
///       before turn-on) does not have to save the registers this work needs.
static void __attribute__((noinline)) __not_in_flash_func(exposureGuardWindowWork)(uint32_t armed, uint32_t pulseNumber)
{
  // -------------------------------------------------------------------------
  // Guard-window work (Rev 2.2). Both GP2 edges are disabled and the t0 + 20 us alarm is
  // already armed; the timer IRQ has the same priority, so it runs right after this
  // returns. Bounded: <= LIGHT_SCHEDULE_MAX_LENGTH compare steps, no division.
  // -------------------------------------------------------------------------
  pulseCeilingUs = lightOnCeilingUs.load(std::memory_order_relaxed); // clamped when 'E' was parsed

  uint8_t fired = (uint8_t)(armed & ARM_LIGHT_FIELD);
  if (armed & ARM_FORCED_BIT)
  {
    lightForcedAdvanceCount.fetch_add(1, std::memory_order_relaxed);
  }
  if (fired == lastFiredLight)
  {
    if (consecutiveFireCount != 0xFFFFFFFFu) consecutiveFireCount = consecutiveFireCount + 1u;
  }
  else
  {
    lastFiredLight = fired;
    consecutiveFireCount = 1;
  }

  // Counter and light in one word, so Core 1 can never pair a count with the wrong light
  lightLastStrobeWord.store((pulseNumber << STROBE_WORD_COUNTER_SHIFT) | fired, std::memory_order_relaxed);

  // Decide what the next exposure fires
  uint32_t published = lightSchedulePublished.load(std::memory_order_relaxed);
  uint32_t next;
  if (autoLights == STEP_EXPOSURE)
  {
    uint32_t length = (published >> SCHEDULE_LENGTH_SHIFT) & SCHEDULE_LENGTH_FIELD;
    uint32_t generation = published >> SCHEDULE_GENERATION_SHIFT;
    uint32_t cursor = isrScheduleCursor;
    if (generation != isrScheduleGeneration)
    {
      // New schedule (or 'e' restart): the COB that just fired counts as entry 0
      isrScheduleGeneration = generation;
      cursor = 0;
    }
    cursor++;
    if (cursor >= length) cursor = 0;

    volatile uint32_t chosenIndex = cursor;
    next = chooseArmedFire(lightScheduleBank[published & SCHEDULE_BANK_FIELD][cursor], published, &chosenIndex);
    isrScheduleCursor = chosenIndex;
    lightOn = (uint8_t)(next & ARM_LIGHT_FIELD);
  }
  else
  {
    // Keep the selected COB; with no selection, keep the one that fired (2.1 behaviour)
    uint8_t wanted = lightOn;
    if (wanted >= NUMBER_OF_LIGHTS) wanted = fired;
    next = chooseArmedFire(wanted, published, nullptr);
  }

  // Sequence bump makes any in-flight core-0 loop CAS fail and recompute
  armedFire.store(next | ((armed & ARM_SEQUENCE_FIELD) + ARM_SEQUENCE_UNIT), std::memory_order_relaxed);
}

/// @brief Unified exposure GPIO ISR on GP2 (NVIC Priority 0, resident in SRAM).
/// @note The rising-edge path up to the 20 us alarm is 2.1's, plus one AND on the armed word.
///       All Rev 2.2 work follows the alarm, inside the guard window (see the design note above).
void __not_in_flash_func(exposureISR)()
{
  if (lightCycleState == LIGHT_STATE_READY)
  {
    // -------------------------------------------------------------------------
    // Exposure rising edge (t0)
    // -------------------------------------------------------------------------
    uint32_t armed = armedFire.load(std::memory_order_relaxed);
    uint32_t mask = armed & LIGHT_ALL_PINS_MASK;
    if (mask == 0)
    {
      if (armed & ARM_STARVED_BIT)
      {
        // 'N' limit reached with no other COB allowed: this exposure stays dark
        lightStarvedCount.fetch_add(1, std::memory_order_relaxed);
      }
      return;
    }

    // Fast path: turn ON armed COB as the very first operation
    sio_hw->gpio_set = mask;

    // Record t0
    risingEdgeTimeUs = time_us_64();

    // Increment pulse counter while COB is already ON (0 ns jitter impact)
    uint32_t pulseNumber = exposurePulseCount.fetch_add(1, std::memory_order_relaxed) + 1u;

    // Immediately disable both edges on GP2 during the 20 us guard window
    gpio_set_irq_enabled(CAMERA_EXPOSURE_PIN, GPIO_IRQ_EDGE_RISE, false);
    gpio_set_irq_enabled(CAMERA_EXPOSURE_PIN, GPIO_IRQ_EDGE_FALL, false);
    gpio_acknowledge_irq(CAMERA_EXPOSURE_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL);

    // State becomes ARMING_FALL
    lightCycleState = LIGHT_STATE_ARMING_FALL;

    // Schedule Step 1: 20 us noise guard window (t0 + 20 us)
    if (lightHardwareAlarmNum >= 0)
    {
      hardware_alarm_set_target((uint)lightHardwareAlarmNum,
                                from_us_since_boot(risingEdgeTimeUs + LIGHT_FALL_ARM_DELAY_US));
    }

    // Rev 2.2 bookkeeping, kept out of this function so its entry stays as short as 2.1's
    exposureGuardWindowWork(armed, pulseNumber);
    return;
  }

  if (lightCycleState == LIGHT_STATE_WAIT_FALL)
  {
    // -------------------------------------------------------------------------
    // Exposure falling edge
    // -------------------------------------------------------------------------
    if (!tryAcquirePulseClosure())
    {
      return;
    }
    exposureNormalFallCount.fetch_add(1, std::memory_order_relaxed);
    finishLightPulse();
    return;
  }
}
