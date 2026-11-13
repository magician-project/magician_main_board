// =============================================================================
//  CameraControllerMultizone — Raspberry Pi Pico 2 (RP2350)
//  HW version: MagicianCam4 — Rev. 1.3
//  SW version: 2.2 (Modular Architecture)
//
//  Changes in 2.2 (relative to 2.1):
//    - Line commands 'E<us>' (light-on ceiling), 'N<n>' (max consecutive exposures per COB,
//      0 = unlimited, the default), 'S<digits>' (light schedule); unknown lines are discarded
//    - 'e': exposure-locked mode, one schedule step and one CSV row per exposure
//    - CSV extended to 17 fields (StrobeCounter, StrobeLight, ForcedAdvances, Starved, WatchdogTrips)
//    - 'P11,0|1' switches the 11.5 V camera rail, 'PR<ms>' power-cycles it; 'P5,0|1' switches the
//      5 V rail, but off is refused without USB (it is then the board's own supply). Lights are
//      turned off first. Previously the rails could not be controlled at all.
//    - 'p': one-line power query "P:11V=c/r,5V=c/r,VBUS=..,ADC3=..mV,CYCLE=..,MCP=..,REFUSED=.."
//      (c = commanded, r = MCP23018 enable-pin readback; there is no rail voltage sense)
//    - 'x' report: light limits & schedule, power rails sections
//    - No new interrupt-off sections; all new ISR work runs inside the 20 us guard window
//
//  Dual-Core Architecture:
//    - Core 0: Real-Time & Hardware Subsystems (Exposure ISR, COB LEDs, I2C, ToF)
//    - Core 1: Communications & Application Layer (Serial 115200, W5500 SPI0, Parser, CSV)
// =============================================================================

#include "hardware_config.h"
#include "intercore_comm.h"
#include "mcp23018_driver.h"
#include "tof_manager.h"
#include "light_controller.h"
#include "network_manager.h"
#include "application_layer.h"
#include "hardware/watchdog.h"

static volatile bool softwareResetRequested = false;

/// @brief Shuts down illumination, pauses, and triggers RP2350 hardware watchdog reboot.
static void performSoftwareReset()
{
  autoLights = STEP_EXTERNAL;
  deactivateLights();
  delay(RESET_PRE_WATCHDOG_DELAY_MS);
  watchdog_reboot(0, 0, RESET_WATCHDOG_TIMEOUT_MS);
  while (true) { tight_loop_contents(); }
}

/// @brief Generates initial startup LED blink pattern (3 short flashes @ 80 ms).
static void bootBlinkStart()
{
  pinMode(LED_PIN, OUTPUT);
  for (int i = 0; i < BOOT_LED_START_COUNT; i++)
  {
    digitalWrite(LED_PIN, HIGH);
    delay(BOOT_LED_START_BLINK_MS);
    digitalWrite(LED_PIN, LOW);
    delay(BOOT_LED_START_BLINK_MS);
  }
}

/// @brief Generates boot completion LED blink pattern (2 long flashes @ 300 ms).
static void bootBlinkComplete()
{
  for (int i = 0; i < BOOT_LED_COMPLETE_COUNT; i++)
  {
    digitalWrite(LED_PIN, HIGH);
    delay(BOOT_LED_COMPLETE_BLINK_MS);
    digitalWrite(LED_PIN, LOW);
    delay(BOOT_LED_COMPLETE_BLINK_MS);
  }
}

/// @brief Publishes coherent snapshot of Core 0 runtime status (buttons & active COB).
static void core0PublishRuntimeStatusSnapshot(uint32_t button1, uint32_t button2, uint32_t activeLight)
{
  uint32_t stableVersion = core0RuntimeStatusSnapshot.publicationVersion.load(std::memory_order_relaxed);
  if (stableVersion & 1u) stableVersion++;

  uint32_t writingVersion = stableVersion + 1u;
  uint32_t completeVersion = stableVersion + 2u;

  core0RuntimeStatusSnapshot.publicationVersion.store(writingVersion, std::memory_order_release);
  core0RuntimeStatusSnapshot.button1.store(button1, std::memory_order_relaxed);
  core0RuntimeStatusSnapshot.button2.store(button2, std::memory_order_relaxed);
  core0RuntimeStatusSnapshot.activeLight.store(activeLight, std::memory_order_relaxed);
  core0RuntimeStatusSnapshot.publicationVersion.store(completeVersion, std::memory_order_release);
}

/// @brief Non-blockingly accepts latest coherent ToF light recommendation from Core 1.
static void core0TryConsumeRecommendationSnapshot()
{
#if ENABLE_LASER_LIGHT_DECISION
  uint32_t versionBefore = core1RecommendationSnapshot.publicationVersion.load(std::memory_order_acquire);
  if ((versionBefore == 0) || (versionBefore & 1u))
  {
    core0RecommendationBusySkipCount.fetch_add(1, std::memory_order_relaxed);
    return;
  }

  uint32_t recommendedLight = core1RecommendationSnapshot.recommendedLight.load(std::memory_order_relaxed);
  uint32_t versionAfter = core1RecommendationSnapshot.publicationVersion.load(std::memory_order_acquire);

  if (versionBefore != versionAfter || (versionAfter & 1u))
  {
    core0RecommendationInconsistentSkipCount.fetch_add(1, std::memory_order_relaxed);
    return;
  }

  uint32_t lastAcceptedVersion = core0AcceptedRecommendationVersion.load(std::memory_order_relaxed);
  if (versionAfter == lastAcceptedVersion) return;

  core0AcceptedRecommendationLight.store(recommendedLight, std::memory_order_relaxed);
  core0AcceptedRecommendationVersion.store(versionAfter, std::memory_order_release);
  core0RecommendationAcceptCount.fetch_add(1, std::memory_order_relaxed);
#endif
}

/// @brief Dequeues and executes host commands dispatched by Core 1.
// =============================================================================
//  Power rails ('P5,x', 'P11,x', 'PR<ms>'): Core 0 owns the I2C bus to the MCP23018
// =============================================================================
static bool rail5VOn = true;          // Both rails are on from power-up (MCP23018 OLATA default 0x3C)
static bool rail11VOn = true;
static bool railCyclePending = false; // 'PR' in progress: rails off until railCycleOnAtMs
static uint32_t railCycleOnAtMs = 0;

/// @brief Publishes the rail state word read by the 'x' report.
static void core0PublishRailState()
{
  uint32_t state = (rail5VOn ? RAIL_STATE_5V_ON : 0u) | (rail11VOn ? RAIL_STATE_11V_ON : 0u) |
                   (railCyclePending ? RAIL_STATE_CYCLING : 0u) | (mcp23018_present ? RAIL_STATE_MCP_PRESENT : 0u);
  core0RailState.store(state, std::memory_order_release);
}

/// @brief Switches one rail. Lights are turned off and disarmed before any rail goes down.
/// @note The 5 V rail is the Pico's own supply (VSYS via Q19) whenever USB is absent, so turning
///       it off is refused in that case: it would brown out the board mid-command, and the rail
///       would come back by itself as the MCP23018 lost power, i.e. an uncontrolled reset.
/// @return False if the request was refused.
static bool core0SetRail(bool rail11V, bool on)
{
  if (!rail11V && !on && !gpio_get(USB_VBUS_SENSE_PIN))
  {
    core0RailRefusedCount.fetch_add(1, std::memory_order_relaxed);
    return false;
  }
  if (!on)
  {
    autoLights = STEP_EXTERNAL;
    deactivateLights();
  }
  if (rail11V) { mcp23018EnableRail11V(on); rail11VOn = on; }
  else         { mcp23018EnableRail5V(on);  rail5VOn  = on; }
  return true;
}

/// @brief Re-reads the rail enable pins (MCP23018 GPIOA) and the GP29/ADC3 pin voltage, and publishes them.
/// @note Core 0 owns the I2C bus; this is one register read (~50 us at 1 MHz) plus one ADC sample.
static uint32_t railReadbackLastMs = 0;
static void core0RefreshRailReadback()
{
  uint8_t gpioa = 0;
  bool readOk = mcp23018_present && mcp23018ReadRegChecked(MCP23018_GPIOA, &gpioa);
  uint32_t raw = (uint32_t)analogRead(VSYS_SENSE_PIN);            // 12-bit, 3.3 V reference
  uint32_t pinMv = (raw * 3300u) / 4095u;                         // raw pin voltage; see VSYS_SENSE_PIN
  core0RailReadback.store((uint32_t)gpioa | (readOk ? RAIL_READBACK_OK_BIT : 0u) |
                          (pinMv << RAIL_READBACK_VSYS_SHIFT), std::memory_order_relaxed);
  railReadbackLastMs = millis();
  core0RailReadbackMs.store(railReadbackLastMs, std::memory_order_relaxed);
  core0RailReadbackSeq.fetch_add(1, std::memory_order_release);
}

/// @brief Completes a pending 'PR' camera power-cycle once its off time has elapsed (non-blocking).
static void core0ProcessRailCycle()
{
  if (!railCyclePending) return;
  if ((int32_t)(millis() - railCycleOnAtMs) < 0) return;
  core0SetRail(true, true);
  railCyclePending = false;
  core0PublishRailState();
  core0RefreshRailReadback();
}

static void core0TryDequeueCommand()
{
  uint32_t tail = core1ToCore0CommandQueue.tail.load(std::memory_order_relaxed);
  uint32_t head = core1ToCore0CommandQueue.head.load(std::memory_order_acquire);
  if (tail == head) return;

  uint32_t descriptor = core1ToCore0CommandQueue.data[tail & CORE1_TO_CORE0_COMMAND_QUEUE_MASK];
  uint32_t command = unpackCoreCommandId(descriptor);
  uint32_t payload = unpackCoreCommandPayload(descriptor);

  core1ToCore0CommandQueue.tail.store(tail + 1u, std::memory_order_release);

  switch (command)
  {
    case CORE_COMMAND_SET_LIGHT_PATTERN:
      if ((payload == PATTERN_ROUNDROBIN) || (payload == PATTERN_OPPOSITE))
      {
        lightPattern = (uint8_t)payload;
        autoLights = STEP_TIMER;
        lightOn = NO_LIGHT_ON;
        core0LastAppliedPattern.store(payload, std::memory_order_relaxed);
        core0PatternCommandConsumeCount.fetch_add(1, std::memory_order_relaxed);
      }
      break;

    case CORE_COMMAND_SET_AUTO_MODE:
      if (payload == STEP_DEPTH)
      {
        autoLights = STEP_DEPTH;
        core0LastAppliedAutoMode.store(payload, std::memory_order_relaxed);
        core0AutoModeCommandConsumeCount.fetch_add(1, std::memory_order_relaxed);
      }
      break;

    case CORE_COMMAND_START_TIMER_MODE:
      lightStartTime = millis();
      autoLights = STEP_TIMER;
      lightOn = NO_LIGHT_ON;
      core0TimerModeCommandConsumeCount.fetch_add(1, std::memory_order_relaxed);
      break;

    case CORE_COMMAND_START_TIMED_MODE:
      autoLights = STEP_TIMED;
      lightOn = NO_LIGHT_ON;
      flashStartTime = millis();
      core0TimedModeCommandConsumeCount.fetch_add(1, std::memory_order_relaxed);
      break;

    case CORE_COMMAND_SELECT_LIGHT:
      if (payload < NUMBER_OF_LIGHTS)
      {
        autoLights = STEP_EXTERNAL; // before arming, so an 'e'-mode ISR cannot re-arm over it
        activateLight((uint8_t)payload);
        core0LastSelectedLight.store(payload, std::memory_order_relaxed);
        core0SelectLightCommandConsumeCount.fetch_add(1, std::memory_order_relaxed);
      }
      break;

    case CORE_COMMAND_DISABLE_LIGHTS:
      deactivateLights();
      autoLights = STEP_EXTERNAL;
      core0DisableLightsCommandConsumeCount.fetch_add(1, std::memory_order_relaxed);
      break;

    case CORE_COMMAND_ADVANCE_LIGHT:
      autoLights = STEP_EXTERNAL;
      activateLight(getNextLight(lightOn, NUMBER_OF_LIGHTS, lightPattern));
      lightStartTime = millis();
      core0AdvanceLightCommandConsumeCount.fetch_add(1, std::memory_order_release);
      break;

    case CORE_COMMAND_EXPOSURE_LOCKED:
      enterExposureLockedMode();
      core0ExposureLockedCommandConsumeCount.fetch_add(1, std::memory_order_relaxed);
      break;

    case CORE_COMMAND_SET_SCHEDULE:
    {
      // Seqlock read of Core 1's buffer; a torn copy is dropped (a newer command is queued)
      uint8_t schedule[LIGHT_SCHEDULE_MAX_LENGTH];
      uint32_t versionBefore = core1PendingScheduleVersion.load(std::memory_order_acquire);
      uint32_t length = core1PendingScheduleLength.load(std::memory_order_relaxed);
      if (length > LIGHT_SCHEDULE_MAX_LENGTH) length = LIGHT_SCHEDULE_MAX_LENGTH;
      for (uint32_t i = 0; i < length; i++) schedule[i] = core1PendingSchedule[i];
      std::atomic_thread_fence(std::memory_order_acquire);
      uint32_t versionAfter = core1PendingScheduleVersion.load(std::memory_order_relaxed);
      if (((versionBefore & 1u) == 0) && (versionBefore == versionAfter))
      {
        setLightSchedule(schedule, length);
        core0ScheduleCommandConsumeCount.fetch_add(1, std::memory_order_relaxed);
      }
      break;
    }

    case CORE_COMMAND_SET_RAIL:
      railCyclePending = false; // an explicit switch overrides a power-cycle in progress
      core0SetRail((payload & 1u) != 0, (payload & 2u) != 0);
      core0PublishRailState();
      core0RefreshRailReadback();
      core0RailCommandConsumeCount.fetch_add(1, std::memory_order_relaxed);
      break;

    case CORE_COMMAND_CYCLE_RAILS: // camera rail only; the 5 V rail may be the board's own supply
      core0SetRail(true, false);
      railCyclePending = true;
      railCycleOnAtMs = millis() + core1RailCycleOffMs.load(std::memory_order_acquire);
      core0PublishRailState();
      core0RefreshRailReadback();
      core0RailCommandConsumeCount.fetch_add(1, std::memory_order_relaxed);
      break;

    case CORE_COMMAND_QUERY_RAILS:
      core0RefreshRailReadback();
      break;

    case CORE_COMMAND_RESET_REQUEST:
      autoLights = STEP_EXTERNAL;
      softwareResetRequested = true;
      core0ResetCommandConsumeCount.fetch_add(1, std::memory_order_relaxed);
      break;

    default:
      break;
  }

  core0CommandQueueDequeueCount.fetch_add(1, std::memory_order_relaxed);
}

// =============================================================================
//  Core 0 Setup & Loop Orchestrators
// =============================================================================

/// @brief Core 0 entry point: initializes real-time peripherals, I2C, ToF, and COBs.
void setup()
{
  __asm volatile("mrs %0, msp" : "=r"(core0StackPointerAtSetupEntry));

  volatile const uint32_t *vtorRegister = (volatile const uint32_t *)0xE000ED08u;
  uintptr_t activeVectorTable = (uintptr_t)(*vtorRegister);
  core0InitialMainStackPointer = (uintptr_t)(*((const uint32_t *)activeVectorTable));

  core0SystemReady.store(0, std::memory_order_relaxed);
  w5500HardwareReady.store(0, std::memory_order_relaxed);
  core0OutputBytesQueued.store(0, std::memory_order_relaxed);
  core0OutputQueueOverflowCount.store(0, std::memory_order_relaxed);
  core0OutputQueueMaxOccupancy.store(0, std::memory_order_relaxed);

  bootBlinkStart();

  core0QueueOutputFlashText(F("== CameraControllerMultizone boot ==\r\n"));
  core0QueueOutputFlashText(F("V:"));
  core0QueueOutputFlashText(F(VERSION_STRING));
  core0QueueOutputNewline();

  for (uint8_t i = 0; i < NUMBER_OF_LIGHTS; i++)
  {
    digitalWrite(directLightControl[i], LOW);
    pinMode(directLightControl[i], OUTPUT);
  }

  pinMode(CAMERA_EXPOSURE_PIN, INPUT);

#if USE_VL53L5CX
  pinMode(SENSOR0_INT_PIN, INPUT);
  pinMode(SENSOR1_INT_PIN, INPUT);
  pinMode(SENSOR2_INT_PIN, INPUT);
#endif

#if USE_W5500
  pinMode(W5500_INT_PIN, INPUT_PULLUP);
#endif

  recoverI2CBus();

  Wire.setSDA(SDA_PIN);
  Wire.setSCL(SCL_PIN);
  Wire.begin();
  Wire.setClock(1000000);

  initMCP23018();

  if (mcp23018_present)
  {
    initMcp23018ButtonStateAndInterrupt();
  }

#if USE_VL53L5CX
  VL53L5CX_NORMALLN(F("VL53L5CX: initialising 3 sensors"));
  setI2CDistanceAddresses();
  tofPrintInitSummary();
  attachTofInterruptsForWorkingSensors();

  for (uint8_t sensor = 0; sensor < NUMBER_OF_DISTANCE_SENSORS; sensor++)
  {
    tofProducerSequence[sensor] = 0;
    tofPublishedFrameCount[sensor].store(0, std::memory_order_relaxed);
    tofProducerDropCount[sensor].store(0, std::memory_order_relaxed);

    for (uint8_t slot = 0; slot < 2; slot++)
    {
      tofSharedBuffers[sensor].slot[slot].timestampUs = 0;
      tofSharedBuffers[sensor].slot[slot].sequence = 0;
      tofSharedBuffers[sensor].slot[slot].state.store(TOF_SLOT_FREE, std::memory_order_relaxed);
    }
  }

  tofSecondarySlotUseCount.store(0, std::memory_order_relaxed);
  tofMaxOccupiedSlots.store(0, std::memory_order_relaxed);
  core0StepDepthRecommendationApplyCount.store(0, std::memory_order_relaxed);
  core0AcceptedRecommendationVersion.store(0, std::memory_order_relaxed);
  core0AcceptedRecommendationLight.store(NO_LIGHT_ON, std::memory_order_relaxed);
  core0RecommendationAcceptCount.store(0, std::memory_order_relaxed);
  core0RecommendationBusySkipCount.store(0, std::memory_order_relaxed);
  core0RecommendationInconsistentSkipCount.store(0, std::memory_order_relaxed);
#endif

#if USE_W5500
  mcp23018ResetW5500(true);
  delay(W5500_RST_SLEEP_MS);
  mcp23018ResetW5500(false);
  delay(W5500_RST_RESUME_MS);
  w5500HardwareReady.store(1, std::memory_order_release);
#else
  w5500HardwareReady.store(1, std::memory_order_release);
#endif

  initLightController();

  bootBlinkComplete();
  updateBoardLedFromButtons();

  core0QueueOutputFlashText(F("== boot complete ==\r\n"));

  core0CommandQueueDequeueCount.store(0, std::memory_order_relaxed);
  core0PatternCommandConsumeCount.store(0, std::memory_order_relaxed);
  core0LastAppliedPattern.store(PATTERN_ROUNDROBIN, std::memory_order_relaxed);
  core0AutoModeCommandConsumeCount.store(0, std::memory_order_relaxed);
  core0LastAppliedAutoMode.store(STEP_EXTERNAL, std::memory_order_relaxed);
  core0TimerModeCommandConsumeCount.store(0, std::memory_order_relaxed);
  core0SelectLightCommandConsumeCount.store(0, std::memory_order_relaxed);
  core0LastSelectedLight.store(NO_LIGHT_ON, std::memory_order_relaxed);
  core0DisableLightsCommandConsumeCount.store(0, std::memory_order_relaxed);
  core0AdvanceLightCommandConsumeCount.store(0, std::memory_order_relaxed);
  core0ResetCommandConsumeCount.store(0, std::memory_order_relaxed);
  core0TimedModeCommandConsumeCount.store(0, std::memory_order_relaxed);
  core0ExposureLockedCommandConsumeCount.store(0, std::memory_order_relaxed);
  core0ScheduleCommandConsumeCount.store(0, std::memory_order_relaxed);

  core0RuntimeStatusSnapshot.publicationVersion.store(0, std::memory_order_relaxed);
  core0RuntimeStatusSnapshot.button1.store(0, std::memory_order_relaxed);
  core0RuntimeStatusSnapshot.button2.store(0, std::memory_order_relaxed);
  core0RuntimeStatusSnapshot.activeLight.store(NO_LIGHT_ON, std::memory_order_relaxed);

  pinMode(USB_VBUS_SENSE_PIN, INPUT);
  analogReadResolution(12);
  core0PublishRailState();
  core0RefreshRailReadback();
  core0SystemReady.store(1, std::memory_order_release);

  while (core1TransportReady.load(std::memory_order_acquire) == 0)
  {
    delay(1);
  }

  core1SerialRuntimeReady.store(1, std::memory_order_release);
  __asm volatile("mrs %0, msp" : "=r"(core0StackPointerAtSetupCompletion));
}

/// @brief Core 0 real-time execution loop: handles ToF interrupts, debouncing, and stepping.
void loop()
{
  uint32_t currentTime = millis();

  if (core1ResetRequest.load(std::memory_order_acquire) != 0)
  {
    core1ResetRequest.store(0, std::memory_order_release);
    deactivateLights();
    autoLights = STEP_EXTERNAL;

    uint32_t head = core1ToCore0CommandQueue.head.load(std::memory_order_acquire);
    core1ToCore0CommandQueue.tail.store(head, std::memory_order_release);
    core0ResetHardwareSafe.store(1, std::memory_order_release);
  }

#if USE_VL53L5CX
  processTofInterruptEvents();
#endif

#if ENABLE_LASER_LIGHT_DECISION
  core0TryConsumeRecommendationSnapshot();
#endif

  core0TryDequeueCommand();
  core0ProcessRailCycle();
  if ((uint32_t)(millis() - railReadbackLastMs) >= RAIL_READBACK_PERIOD_MS) core0RefreshRailReadback();

  if (softwareResetRequested)
  {
    softwareResetRequested = false;
    performSoftwareReset();
  }

  unsigned int buttonRaw1 = debouncedPB0 ? 1 : 0;
  unsigned int buttonRaw2 = debouncedPB1 ? 1 : 0;

  bool applicationTick = (currentTime - lastUpdateTime >= loopRateMsec);

  if (applicationTick)
  {
    lastUpdateTime = currentTime;
    processMcp23018ButtonsDebounce();

    buttonRaw1 = debouncedPB0 ? 1 : 0;
    buttonRaw2 = debouncedPB1 ? 1 : 0;

    core0PublishRuntimeStatusSnapshot(buttonRaw1, buttonRaw2, lightOn);
  }

  if (applicationTick)
  {
    if (autoLights != STEP_EXTERNAL && (currentTime - lightStartTime >= lightDurationMsec))
    {
      lightStartTime = currentTime;

      if (autoLights == STEP_TIMER || autoLights == STEP_TIMED)
      {
        activateLight(getNextLight(lightOn, NUMBER_OF_LIGHTS, lightPattern));
        if (autoLights == STEP_TIMED && (currentTime - flashStartTime >= 10000))
        {
          autoLights = STEP_EXTERNAL;
          deactivateLights();
        }
      }
#if ENABLE_LASER_LIGHT_DECISION
      else if (autoLights == STEP_DEPTH)
      {
        uint32_t acceptedVersion = core0AcceptedRecommendationVersion.load(std::memory_order_acquire);
        uint32_t recommendedLight = core0AcceptedRecommendationLight.load(std::memory_order_relaxed);

        if ((acceptedVersion != 0) && (recommendedLight < NUMBER_OF_LIGHTS))
        {
          activateLight((uint8_t)recommendedLight);
          core0StepDepthRecommendationApplyCount.fetch_add(1, std::memory_order_relaxed);
        }
      }
#endif
    }
  }

  delayMicroseconds(CPU_SLEEP_US);
}

// =============================================================================
//  Core 1 Setup & Loop Orchestrators
// =============================================================================

/// @brief Core 1 entry point: initializes Serial CDC, W5500 SPI0 Ethernet, and local telemetry.
void setup1()
{
  __asm volatile("mrs %0, msp" : "=r"(core1StackPointerAtSetup1Entry));

  core1TransportReady.store(0, std::memory_order_relaxed);
  core1EthernetHardwareStatus.store(0, std::memory_order_relaxed);
  core1EthernetLinkStatus.store(0, std::memory_order_relaxed);
  core1EthernetLocalIpPacked.store(0, std::memory_order_relaxed);
  core1EthernetClientConnected.store(0, std::memory_order_relaxed);
  core1EthernetBytesReceived.store(0, std::memory_order_relaxed);
  core1EthernetBytesTransmitted.store(0, std::memory_order_relaxed);
  core1SerialBytesReceivedDirect.store(0, std::memory_order_relaxed);
  core1OutputBytesTransmitted.store(0, std::memory_order_relaxed);
  core1LoopIterations.store(0, std::memory_order_relaxed);
  core1FramesProcessed.store(0, std::memory_order_relaxed);
  core1RecommendationsPublished.store(0, std::memory_order_relaxed);
  core1LastActivityMs.store(0, std::memory_order_relaxed);
  core1LastFrameProcessingUs.store(0, std::memory_order_relaxed);
  core1MaxFrameProcessingUs.store(0, std::memory_order_relaxed);
  core1LastLoopDurationUs.store(0, std::memory_order_relaxed);
  core1MaxLoopDurationUs.store(0, std::memory_order_relaxed);
  core1Running.store(1, std::memory_order_release);

  core1ApplicationReportingEnabled = false;
  core1ApplicationStreamTarget     = CORE1_OUTPUT_SERIAL;
  core1AdvanceReportTarget         = CORE1_OUTPUT_SERIAL;
  core1ApplicationReportMode       = CORE1_REPORT_MODE_TIMER;
  core1LastApplicationReportMs     = 0;
  core1LastReportedLight           = NO_LIGHT_ON;
  core1AdvanceReportPending        = false;
  core1AdvanceConsumeCountObserved = 0;
  core1AdvanceSnapshotVersionBaseline = 0;
  core1CommandSequence             = 0;
  core1SessionResetPending         = false;

  core1ToCore0CommandQueue.head.store(0, std::memory_order_relaxed);
  core1ToCore0CommandQueue.tail.store(0, std::memory_order_relaxed);
  core1CommandQueueEnqueueCount.store(0, std::memory_order_relaxed);
  core1CommandQueueOverflowCount.store(0, std::memory_order_relaxed);
  core1CommandQueueMaxOccupancy.store(0, std::memory_order_relaxed);

  for (uint8_t sensor = 0; sensor < NUMBER_OF_DISTANCE_SENSORS; sensor++)
  {
    tof_has_valid_distance[sensor] = 0;
    tof_last_update_ms[sensor] = 0;
    tof_distance_mm[sensor] = 0;

    tofConsumedFrameCount[sensor].store(0, std::memory_order_relaxed);
    tofLastConsumedSequence[sensor].store(0, std::memory_order_relaxed);
    tofLastConsumedTimestampLowUs[sensor].store(0, std::memory_order_relaxed);
    core1CenterDistanceMm[sensor].store(0, std::memory_order_relaxed);
    core1CenterDistanceCalculationCount[sensor].store(0, std::memory_order_relaxed);

    core1RecommendationSnapshot.depthMm[sensor].store(0, std::memory_order_relaxed);
    core1RecommendationSnapshot.sourceSequence[sensor].store(0, std::memory_order_relaxed);
  }

  core1RecommendationSnapshot.recommendedLight.store(NO_LIGHT_ON, std::memory_order_relaxed);
  core1RecommendationSnapshot.publicationCount.store(0, std::memory_order_relaxed);
  core1RecommendationSnapshot.publicationVersion.store(0, std::memory_order_release);

  Serial.begin(SERIAL_BAUD_RATE);

  uint32_t serialWaitStartMs = millis();
  while (!Serial && ((uint32_t)(millis() - serialWaitStartMs) < USB_STARTUP_TIMEOUT_MS))
  {
    delay(USB_STARTUP_POLL_DELAY_MS);
  }

  delay(USB_POST_CONNECT_SETTLE_MS);
  core1DrainCore0BootstrapOutput();

  while (w5500HardwareReady.load(std::memory_order_acquire) == 0)
  {
    core1DrainCore0OutputQueue();
    delay(BOOT_INTERCORE_WAIT_DELAY_MS);
  }

  core1DrainCore0BootstrapOutput();
  core1InitializeTransport();

  while (core0SystemReady.load(std::memory_order_acquire) == 0)
  {
    core1DrainCore0OutputQueue();
    delay(BOOT_INTERCORE_WAIT_DELAY_MS);
  }

  core1DrainCore0BootstrapOutput();
  __asm volatile("mrs %0, msp" : "=r"(core1StackPointerAtSetup1Completion));
}

/// @brief Core 1 application loop: manages transport I/O, ToF consumer processing & CSV stream.
void loop1()
{
  uint32_t loopStartUs = (uint32_t)time_us_64();

  if (core1SessionResetPending)
  {
    core1SessionResetPending = false;
    reset_millis();
    core1LastApplicationReportMs = applicationMillis();
  }

  core1ProcessResetStateMachine();

  core1LoopIterations.fetch_add(1, std::memory_order_relaxed);

  core1DrainCore0OutputQueue();
  core1ProcessDiagnosticStreaming();
  core1ProcessRailQueryReply();
  core1ConsumeRuntimeStatusSnapshot();

#if USE_W5500
  if (client && !client.connected())
  {
    client.stop();
    client = EthernetClient();
    core1EthernetClientConnected.store(0, std::memory_order_release);
    ETH_NORMALLN(F("ETH: client disconnected"));
  }

  if (!client)
  {
    EthernetClient newClient = server.available();
    if (newClient)
    {
      client = newClient;
      client.flush();
      core1EthernetClientConnected.store(1, std::memory_order_release);
      ETH_NORMALLN(F("ETH: client connected"));
    }
  }

  uint32_t clientDrainBytes = 0;
  while (client && client.available() > 0 && clientDrainBytes < MAX_ETHERNET_RX_BYTES_PER_LOOP1)
  {
    uint8_t receivedEthernetByte = (uint8_t)client.read();
    core1EthernetBytesReceived.fetch_add(1, std::memory_order_relaxed);
    core1ClassifyCommandByte(receivedEthernetByte, COMMAND_SOURCE_ETHERNET);
    clientDrainBytes++;
  }
#endif

  if (core1SerialRuntimeReady.load(std::memory_order_acquire) != 0)
  {
    uint32_t serialDrainBytes = 0;
    while ((Serial.available() > 0) && (serialDrainBytes < MAX_SERIAL_RX_BYTES_PER_LOOP1))
    {
      uint8_t receivedSerialByte = (uint8_t)Serial.read();
      core1SerialBytesReceivedDirect.fetch_add(1, std::memory_order_relaxed);
      core1ClassifyCommandByte(receivedSerialByte, COMMAND_SOURCE_SERIAL);
      serialDrainBytes++;
    }
  }

  // =========================================================================
  //  --- APPLICATION LAYER: Spatial ToF Processing & Distance Analysis ---
  // =========================================================================
  core1ProcessToFConsumerLoop();
  core1ProcessApplicationReporting();

  static uint32_t lastHeartbeatMs = 0;
  uint32_t nowMs = millis();
  if ((uint32_t)(nowMs - lastHeartbeatMs) >= 1000)
  {
    lastHeartbeatMs = nowMs;
    uint32_t heartbeat = core1Heartbeat.fetch_add(1, std::memory_order_relaxed) + 1;
    core1UptimeSeconds.store(heartbeat, std::memory_order_release);
  }

  uint32_t loopDurationUs = (uint32_t)time_us_64() - loopStartUs;
  core1LastLoopDurationUs.store(loopDurationUs, std::memory_order_relaxed);

  uint32_t previousLoopMax = core1MaxLoopDurationUs.load(std::memory_order_relaxed);
  if (loopDurationUs > previousLoopMax)
  {
    core1MaxLoopDurationUs.store(loopDurationUs, std::memory_order_relaxed);
  }

  delay(LOOP1_SLEEP_MS);
}
