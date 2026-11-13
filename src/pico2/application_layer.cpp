// =============================================================================
//  MagicianCam4 — Rev. 2.1
//  Application Layer Implementation: 4-Zone Matrix Average & Vector Light Recommendation
// =============================================================================

#include "application_layer.h"
#include "light_controller.h"
#include "network_manager.h"

bool core1ApplicationReportingEnabled = false;
Core1OutputTarget core1ApplicationStreamTarget = CORE1_OUTPUT_SERIAL;
Core1OutputTarget core1AdvanceReportTarget     = CORE1_OUTPUT_SERIAL;
Core1ApplicationReportMode core1ApplicationReportMode = CORE1_REPORT_MODE_TIMER;
bool core1AdvanceReportPending = false;
uint32_t core1AdvanceConsumeCountObserved = 0;
uint32_t core1AdvanceSnapshotVersionBaseline = 0;
uint32_t core1LastApplicationReportMs = 0;
uint32_t core1LastReportedLight = NO_LIGHT_ON;
bool core1SessionResetPending = false;
Core1ResetState core1ResetState = CORE1_RESET_IDLE;
static uint32_t core1ResetCountdownStage = 0;
static uint32_t core1ResetNextCountdownMs = 0;

uint16_t tof_distance_mm[NUMBER_OF_DISTANCE_SENSORS] = {0, 0, 0};
uint8_t tof_has_valid_distance[NUMBER_OF_DISTANCE_SENSORS] = {0, 0, 0};
unsigned long tof_last_update_ms[NUMBER_OF_DISTANCE_SENSORS] = {0, 0, 0};

static uint16_t core1RecommendationDepthMm[NUMBER_OF_DISTANCE_SENSORS] = {0, 0, 0};
static uint32_t core1RecommendationLastValidMs[NUMBER_OF_DISTANCE_SENSORS] = {0, 0, 0};
static uint32_t core1RecommendationSourceSequence[NUMBER_OF_DISTANCE_SENSORS] = {0, 0, 0};
static bool core1RecommendationHasValidDepth[NUMBER_OF_DISTANCE_SENSORS] = {false, false, false};

static uint32_t millis_offset = 0; // Monotonic offset moved only by reset_millis() ('h')

static uint32_t core1ObservedButton1 = 0;
static uint32_t core1ObservedButton2 = 0;
static uint32_t core1ObservedLight   = NO_LIGHT_ON;
static Core1ApplicationReportState core1ApplicationReportState;

/// @brief Returns elapsed session time in milliseconds relative to session reset ('h').
uint32_t applicationMillis()
{
  return (uint32_t)(millis() - millis_offset);
}

/// @brief Resets the application time origin to the current monotonic hardware millis().
void reset_millis()
{
  millis_offset = millis();
}

/// @brief Computes representative distance from the 4 central zones (27, 28, 35, 36).
/// @param r Reference to 8x8 VL53L5CX results data frame.
/// @return Mean central distance in mm, or 0 if no valid reading exists (status 5 or 255).
uint16_t centerDistanceMm(const VL53L5CX_ResultsData &r)
{
  const uint8_t idx[4] = {27, 28, 35, 36};
  uint32_t sum = 0;
  uint8_t cnt = 0;

  for (uint8_t k = 0; k < 4; k++)
  {
    uint8_t z = idx[k];
    int16_t d = r.distance_mm[z];
    uint8_t status = r.target_status[z];

    // Status 5 is standard valid ranging; status 255 is validated near-field return
    if ((d > 0) && ((status == 5) || (status == 255)))
    {
      sum += (uint16_t)d;
      cnt++;
    }
  }

  return cnt ? (uint16_t)(sum / cnt) : 0;
}

/// @brief Selects the COB LED that best faces the closest surface via 2-D vector projection.
/// @param depths Pointer to array of 3 ToF central depth measurements.
/// @return Index of the best matching COB LED (0..5).
/// @note Runs hardware single-precision FPU operations on Core 1 in < 2 us.
uint8_t chooseClosestLight(const uint16_t *depths)
{
  float normalX = 0.0f;
  float normalY = 0.0f;

  for (uint8_t i = 0; i < NUMBER_OF_DISTANCE_SENSORS; i++)
  {
    if (depths[i] == 0) continue; // Skip invalid or out-of-range sensors

    // The 3 sensors are positioned angularly at 0°, 120°, 240° (COBs 0, 2, 4)
    uint8_t lightIdx = i * 2;
    float   angle    = lightIdx * (float)M_PI / 3.0f;
    float   weight   = 1.0f / (float)(depths[i] + 1);

    normalX += cosf(angle) * weight;
    normalY += sinf(angle) * weight;
  }

  uint8_t best    = 0;
  float   bestDot = -1.0f;

  // Evaluate dot-product across all 6 COB orientations (0°, 60°, 120°, 180°, 240°, 300°)
  for (uint8_t i = 0; i < NUMBER_OF_LIGHTS; i++)
  {
    float angle  = i * (float)M_PI / 3.0f;
    float dotPrd = normalX * cosf(angle) + normalY * sinf(angle);
    if (dotPrd > bestDot)
    {
      bestDot = dotPrd;
      best = i;
    }
  }

  return best;
}

/// @brief Publishes a coherent versioned recommendation snapshot for Core 0.
/// @param currentTime Current session timestamp in ms.
/// @return True on publication.
bool core1PublishRecommendationSnapshot(uint32_t currentTime)
{
  uint32_t stableVersion = core1RecommendationSnapshot.publicationVersion.load(std::memory_order_relaxed);
  if (stableVersion & 1u) stableVersion++;

  uint32_t writingVersion = stableVersion + 1u;
  uint32_t completeVersion = stableVersion + 2u;

  core1RecommendationSnapshot.publicationVersion.store(writingVersion, std::memory_order_release);

  uint16_t depths[NUMBER_OF_DISTANCE_SENSORS];

  for (uint8_t s = 0; s < NUMBER_OF_DISTANCE_SENSORS; s++)
  {
    if (core1RecommendationHasValidDepth[s] &&
        ((uint32_t)(currentTime - core1RecommendationLastValidMs[s]) <= TOF_DATA_STALE_MS))
    {
      depths[s] = core1RecommendationDepthMm[s];
      core1RecommendationSnapshot.depthMm[s].store(core1RecommendationDepthMm[s], std::memory_order_relaxed);
      core1RecommendationSnapshot.sourceSequence[s].store(core1RecommendationSourceSequence[s], std::memory_order_relaxed);
    }
    else
    {
      depths[s] = 0;
      core1RecommendationSnapshot.depthMm[s].store(0, std::memory_order_relaxed);
      core1RecommendationSnapshot.sourceSequence[s].store(0, std::memory_order_relaxed);
    }
  }

  uint8_t recommended = chooseClosestLight(depths);
  core1RecommendationSnapshot.recommendedLight.store(recommended, std::memory_order_relaxed);
  core1RecommendationSnapshot.publicationCount.fetch_add(1u, std::memory_order_relaxed);
  core1RecommendationSnapshot.publicationVersion.store(completeVersion, std::memory_order_release);

  core1RecommendationsPublished.fetch_add(1u, std::memory_order_relaxed);
  return true;
}

/// @brief Refreshes the local application report structure with latest coherent state.
void core1RefreshApplicationReportState(uint32_t currentTime)
{
  core1ApplicationReportState.timestampMs = currentTime;
  core1ApplicationReportState.button1 = core1ObservedButton1;
  core1ApplicationReportState.button2 = core1ObservedButton2;
  core1ApplicationReportState.activeLight = core1ObservedLight;

  for (uint8_t sensor = 0; sensor < NUMBER_OF_DISTANCE_SENSORS; sensor++)
  {
    uint32_t reportState = REPORT_SENSOR_NO_DATA;
    uint32_t reportDistance = 0;

    if (!tofSensorInitialized[sensor])
    {
      reportState = REPORT_SENSOR_FAILED;
    }
    else if ((tof_has_valid_distance[sensor] == 1) &&
             ((uint32_t)(currentTime - tof_last_update_ms[sensor]) <= TOF_DATA_STALE_MS) &&
             (tof_distance_mm[sensor] > 0))
    {
      reportState = REPORT_SENSOR_VALID;
      reportDistance = tof_distance_mm[sensor];
    }

    core1ApplicationReportState.sensorState[sensor] = reportState;
    core1ApplicationReportState.distanceMm[sensor] = reportDistance;
  }
}

/// @brief Formats and directly transmits the 17-field CSV:
///        timestamp,b1,b2,d0,d1,d2,L0..L5,StrobeCounter,StrobeLight,ForcedAdvances,Starved,WatchdogTrips
/// @note StrobeLight is 0-indexed, -1 before the first strobe. In strobe mode ('e') L0..L5 also show the
///       COB that fired; in the other modes they show the selected COB. WatchdogTrips is always 0 (no
///       host-timeout watchdog yet); the column is kept so the row matches the grabber's layout.
void core1PrintApplicationReport(Core1OutputTarget target)
{
  char csvLine[160];

  uint32_t strobeWord = lightLastStrobeWord.load(std::memory_order_relaxed);
  uint32_t strobeCounter = strobeWord >> STROBE_WORD_COUNTER_SHIFT;
  uint32_t strobeLight = strobeWord & STROBE_WORD_LIGHT_FIELD;
  uint32_t forcedAdvances = lightForcedAdvanceCount.load(std::memory_order_relaxed);
  uint32_t starved = lightStarvedCount.load(std::memory_order_relaxed);

  uint32_t timestamp = core1ApplicationReportState.timestampMs;
  uint32_t button1 = core1ApplicationReportState.button1;
  uint32_t button2 = core1ApplicationReportState.button2;
  uint32_t activeLight = core1ApplicationReportState.activeLight;
  if (core1ApplicationReportMode == CORE1_REPORT_MODE_STROBE) activeLight = strobeLight;

  int offset = snprintf(csvLine, sizeof(csvLine), "%lu,%lu,%lu,", 
                        (unsigned long)timestamp, (unsigned long)button1, (unsigned long)button2);
  if (offset <= 0 || (size_t)offset >= sizeof(csvLine))
  {
    return;
  }

  for (uint8_t sensor = 0; sensor < NUMBER_OF_DISTANCE_SENSORS; sensor++)
  {
    uint32_t sensorState = core1ApplicationReportState.sensorState[sensor];
    uint32_t distance = core1ApplicationReportState.distanceMm[sensor];
    int written = 0;

    if (sensorState == REPORT_SENSOR_FAILED)
    {
      written = snprintf(csvLine + offset, sizeof(csvLine) - (size_t)offset, "F,");
    }
    else if (sensorState == REPORT_SENSOR_VALID)
    {
      if (distance > TOF_MAX_DISTANCE_VALID_MM)
      {
        written = snprintf(csvLine + offset, sizeof(csvLine) - (size_t)offset, "H,");
      }
      else
      {
        written = snprintf(csvLine + offset, sizeof(csvLine) - (size_t)offset, "%lu,", (unsigned long)distance);
      }
    }
    else
    {
      written = snprintf(csvLine + offset, sizeof(csvLine) - (size_t)offset, "0,");
    }

    if (written <= 0 || (size_t)(offset + written) >= sizeof(csvLine))
    {
      return;
    }
    offset += written;
  }

  for (uint8_t light = 0; light < NUMBER_OF_LIGHTS; light++)
  {
    int written = snprintf(csvLine + offset, sizeof(csvLine) - (size_t)offset, "%c,",
                           (light == activeLight) ? '1' : '0');

    if (written <= 0 || (size_t)(offset + written) >= sizeof(csvLine))
    {
      return;
    }
    offset += written;
  }

  int written = snprintf(csvLine + offset, sizeof(csvLine) - (size_t)offset, "%lu,%ld,%lu,%lu,0\r\n",
                         (unsigned long)strobeCounter,
                         (strobeLight < NUMBER_OF_LIGHTS) ? (long)strobeLight : -1L,
                         (unsigned long)forcedAdvances, (unsigned long)starved);
  if (written <= 0 || (size_t)(offset + written) >= sizeof(csvLine))
  {
    return;
  }
  offset += written;

  core1SendDirectText(target, csvLine, (uint16_t)offset);
}

/// @brief Consumes one coherent snapshot of Core 0 runtime status (buttons and active light).
void core1ConsumeRuntimeStatusSnapshot()
{
  uint32_t versionBefore = core0RuntimeStatusSnapshot.publicationVersion.load(std::memory_order_acquire);
  if ((versionBefore == 0) || (versionBefore & 1u)) return;

  uint32_t button1 = core0RuntimeStatusSnapshot.button1.load(std::memory_order_relaxed);
  uint32_t button2 = core0RuntimeStatusSnapshot.button2.load(std::memory_order_relaxed);
  uint32_t activeLight = core0RuntimeStatusSnapshot.activeLight.load(std::memory_order_relaxed);
  uint32_t versionAfter = core0RuntimeStatusSnapshot.publicationVersion.load(std::memory_order_acquire);

  if (versionBefore != versionAfter) return;

  core1ObservedButton1 = button1;
  core1ObservedButton2 = button2;
  core1ObservedLight   = activeLight;
}

/// @brief Consumes ready ToF frames from double buffers and calculates spatial depth averages.
void core1ProcessToFConsumerLoop()
{
  for (uint8_t sensor = 0; sensor < NUMBER_OF_DISTANCE_SENSORS; sensor++)
  {
    for (uint8_t slot = 0; slot < 2; slot++)
    {
      ToFFrameSlot &candidate = tofSharedBuffers[sensor].slot[slot];
      uint32_t expectedState = TOF_SLOT_READY;

      if (!candidate.state.compare_exchange_strong(expectedState, TOF_SLOT_PROCESSING,
                                                  std::memory_order_acquire, std::memory_order_relaxed))
      {
        continue;
      }

      uint32_t frameStartUs = static_cast<uint32_t>(time_us_64());
      uint32_t sequence = candidate.sequence;
      uint64_t timestampUs = candidate.timestampUs;

      core1FramesProcessed.fetch_add(1, std::memory_order_relaxed);
      core1LastActivityMs.store(applicationMillis(), std::memory_order_relaxed);

      uint16_t core1Distance = centerDistanceMm(candidate.frame);
      core1CenterDistanceMm[sensor].store(core1Distance, std::memory_order_relaxed);
      core1CenterDistanceCalculationCount[sensor].fetch_add(1, std::memory_order_relaxed);

      uint32_t core1CurrentTime = applicationMillis();

      if (core1Distance > 0)
      {
        core1RecommendationDepthMm[sensor] = core1Distance;
        core1RecommendationLastValidMs[sensor] = core1CurrentTime;
        core1RecommendationSourceSequence[sensor] = sequence;
        core1RecommendationHasValidDepth[sensor] = true;

        tof_has_valid_distance[sensor] = 1;
        tof_distance_mm[sensor] = core1Distance;
        tof_last_update_ms[sensor] = core1CurrentTime;
      }

#if ENABLE_LASER_LIGHT_DECISION
      core1PublishRecommendationSnapshot(core1CurrentTime);
#endif

      tofLastConsumedSequence[sensor].store(sequence, std::memory_order_relaxed);
      tofLastConsumedTimestampLowUs[sensor].store(static_cast<uint32_t>(timestampUs), std::memory_order_relaxed);
      tofConsumedFrameCount[sensor].fetch_add(1, std::memory_order_relaxed);

      uint32_t frameProcessingUs = static_cast<uint32_t>(time_us_64()) - frameStartUs;
      core1LastFrameProcessingUs.store(frameProcessingUs, std::memory_order_relaxed);

      uint32_t previousMax = core1MaxFrameProcessingUs.load(std::memory_order_relaxed);
      if (frameProcessingUs > previousMax)
      {
        core1MaxFrameProcessingUs.store(frameProcessingUs, std::memory_order_relaxed);
      }

      candidate.state.store(TOF_SLOT_FREE, std::memory_order_release);
    }
  }
}

/// @brief Evaluates reporting triggers (continuous CSV stream or synchronized '+' report).
void core1ProcessApplicationReporting()
{
  uint32_t core1ReportTime = applicationMillis();
  core1RefreshApplicationReportState(core1ReportTime);

  static uint32_t core1LastReportedStrobeEvents = 0;
  uint32_t strobeEvents = exposurePulseCount.load(std::memory_order_acquire) +
                          lightStarvedCount.load(std::memory_order_relaxed);
  if ((core1ApplicationReportMode != CORE1_REPORT_MODE_STROBE) || !core1ApplicationReportingEnabled)
  {
    core1LastReportedStrobeEvents = strobeEvents; // Only strobes after entering 'e' are reported
  }

  if (core1AdvanceReportPending)
  {
    uint32_t advanceConsumeCount = core0AdvanceLightCommandConsumeCount.load(std::memory_order_acquire);

    if (advanceConsumeCount != core1AdvanceConsumeCountObserved)
    {
      if (core1AdvanceSnapshotVersionBaseline == 0)
      {
        core1AdvanceSnapshotVersionBaseline = core0RuntimeStatusSnapshot.publicationVersion.load(std::memory_order_acquire);
      }
      else
      {
        uint32_t currentSnapshotVersion = core0RuntimeStatusSnapshot.publicationVersion.load(std::memory_order_acquire);

        if ((currentSnapshotVersion != core1AdvanceSnapshotVersionBaseline) &&
            ((currentSnapshotVersion & 1u) == 0))
        {
          core1ConsumeRuntimeStatusSnapshot();
          core1ReportTime = applicationMillis();
          core1RefreshApplicationReportState(core1ReportTime);
          core1PrintApplicationReport(core1AdvanceReportTarget);
          core1LastReportedLight = core1ApplicationReportState.activeLight;
          core1LastApplicationReportMs = core1ReportTime;
          core1AdvanceReportPending = false;
          core1AdvanceConsumeCountObserved = advanceConsumeCount;
          core1AdvanceSnapshotVersionBaseline = 0;
        }
      }
    }
  }
  else if (core1ApplicationReportingEnabled)
  {
    if (core1ApplicationReportMode == CORE1_REPORT_MODE_STROBE)
    {
      // One row per exposure: every strobe fired, or left dark by the 'N' limit
      if (strobeEvents != core1LastReportedStrobeEvents)
      {
        core1LastReportedStrobeEvents = strobeEvents;
        core1PrintApplicationReport(core1ApplicationStreamTarget);
        core1LastReportedLight = core1ApplicationReportState.activeLight;
        core1LastApplicationReportMs = core1ReportTime;
      }
    }
    else if (core1ApplicationReportMode == CORE1_REPORT_MODE_DEPTH)
    {
      if ((uint32_t)(core1ReportTime - core1LastApplicationReportMs) >= lightDurationMsec)
      {
        core1LastApplicationReportMs = core1ReportTime;
        core1PrintApplicationReport(core1ApplicationStreamTarget);
        core1LastReportedLight = core1ApplicationReportState.activeLight;
      }
    }
    else
    {
      uint32_t currentReportedLight = core1ApplicationReportState.activeLight;
      if ((currentReportedLight < NUMBER_OF_LIGHTS) &&
          (currentReportedLight != core1LastReportedLight))
      {
        core1PrintApplicationReport(core1ApplicationStreamTarget);
        core1LastReportedLight = currentReportedLight;
        core1LastApplicationReportMs = core1ReportTime;
      }
    }
  }
}

/// @brief Executes 3-second coordinated countdown before triggering watchdog reboot ('z').
void core1ProcessResetStateMachine()
{
  uint32_t currentApplicationMs = applicationMillis();

  if (core1ResetState == CORE1_RESET_REQUESTED)
  {
    if (core0ResetHardwareSafe.load(std::memory_order_acquire) != 0)
    {
      core0ResetHardwareSafe.store(0, std::memory_order_release);
      core1ResetState = CORE1_RESET_HARDWARE_SAFE;
      core1ResetCountdownStage = RESET_COUNTDOWN_START_SEC;
      core1ResetNextCountdownMs = currentApplicationMs;
      core1BroadcastResetText(F("Reboot in 3 seconds...\r\n"));
    }
  }

  if (core1ResetState == CORE1_RESET_HARDWARE_SAFE)
  {
    if ((uint32_t)(currentApplicationMs - core1ResetNextCountdownMs) >= RESET_COUNTDOWN_PERIOD_MS)
    {
      core1ResetNextCountdownMs = currentApplicationMs;

      if (core1ResetCountdownStage > 1)
      {
        core1ResetCountdownStage--;
        if (core1ResetCountdownStage == 2)
        {
          core1BroadcastResetText(F("Reboot in 2 seconds...\r\n"));
        }
        else
        {
          core1BroadcastResetText(F("Reboot in 1 second...\r\n"));
        }
      }
      else
      {
        core1BroadcastResetText(F("Rebooting now...\r\n"));
        core1ResetState = CORE1_RESET_COMMIT;
      }
    }
  }

  if (core1ResetState == CORE1_RESET_COMMIT)
  {
#if USE_W5500
    if (client)
    {
      client.stop();
      client = EthernetClient();
      core1EthernetClientConnected.store(0, std::memory_order_release);
    }
#endif

    core1TryEnqueueCommand(CORE_COMMAND_RESET_REQUEST, 0, COMMAND_SOURCE_NONE, 0);
    core1ResetState = CORE1_RESET_IDLE;
  }
}
