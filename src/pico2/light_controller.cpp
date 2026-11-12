// =============================================================================
//  MagicianCam4 — Rev. 2.1
//  COB LED Lighting Controller Implementation: Single-Cycle SIO & Hardware Timer Guardrails
// =============================================================================

#include "light_controller.h"

// GPIO pins assigned to COB LED channels 1..6
const uint8_t directLightControl[NUMBER_OF_LIGHTS] = {6, 7, 8, 9, 10, 11};

volatile uint8_t lightOn = NO_LIGHT_ON;                      // Currently selected COB index (0..5 or 16)
volatile uint32_t activeLightMask = 0;                       // Pre-calculated SIO bitmask (1u << GP)
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

/// @brief Configures direct GPIO pins, claims hardware alarm, and registers exposure ISR.
/// @note Sets NVIC Priority 0 (highest) on IO_IRQ_BANK0 and hardware timer interrupt.
void initLightController()
{
  for (uint8_t i = 0; i < NUMBER_OF_LIGHTS; i++)
  {
    digitalWrite(directLightControl[i], LOW);
    pinMode(directLightControl[i], OUTPUT);
  }

  lightOn = NO_LIGHT_ON;
  activeLightMask = 0;
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
/// @param lightIdx 0-indexed COB number (0..5). Pre-calculates SIO activeLightMask.
void activateLight(uint8_t lightIdx)
{
  if (lightIdx < NUMBER_OF_LIGHTS)
  {
    lightOn = lightIdx;
    activeLightMask = (1u << directLightControl[lightIdx]);
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
  activeLightMask = 0;
  lightOn         = NO_LIGHT_ON;

  // 5. Re-arm RISE interrupt only after physical shutdown is complete
  lightCycleState = LIGHT_STATE_READY;
  gpio_set_irq_enabled(CAMERA_EXPOSURE_PIN, GPIO_IRQ_EDGE_RISE, true);
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
      hardware_alarm_set_target((uint)lightHardwareAlarmNum,
                                from_us_since_boot(risingEdgeTimeUs + LIGHT_TURNOFF_US));
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

/// @brief Unified exposure GPIO ISR on GP2 (NVIC Priority 0, resident in SRAM).
/// @note Executes single-cycle SIO turn-on (~6.6 ns latency) as the very first instruction.
void __not_in_flash_func(exposureISR)()
{
  if (lightCycleState == LIGHT_STATE_READY)
  {
    // -------------------------------------------------------------------------
    // Exposure rising edge (t0)
    // -------------------------------------------------------------------------
    uint32_t mask = activeLightMask;
    if (mask == 0) return;

    // Fast path: turn ON selected COB as the very first operation (~6.6 ns latency)
    sio_hw->gpio_set = mask;

    // Record t0
    risingEdgeTimeUs = time_us_64();

    // Increment pulse counter while COB is already ON (0 ns jitter impact)
    exposurePulseCount.fetch_add(1, std::memory_order_relaxed);

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
