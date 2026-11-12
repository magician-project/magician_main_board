#pragma once

// =============================================================================
//  MagicianCam4 — Rev. 2.1
//  Application Layer: Spatial ToF Analysis, Dynamic Lighting & CSV Reporting Header
// =============================================================================
//  Subsystem Architecture:
//    - Spatial Analysis : 4-zone central matrix average on 8x8 VL53L5CX data
//    - Vector Lighting  : 2-D inverse-distance angular normal vector projection
//    - Telemetry Stream : 12-field CSV stream (timestamp, buttons, ToF, COBs)
//    - Concurrency Domain: Core 1 exclusive execution
// =============================================================================

#include "hardware_config.h"
#include "intercore_comm.h"
#include "tof_manager.h"

/// @brief Per-sensor validation state within composed application reports.
enum ApplicationReportSensorState : uint32_t
{
  REPORT_SENSOR_FAILED  = 0, ///< Sensor uninitialized or hardware failure ('F')
  REPORT_SENSOR_NO_DATA = 1, ///< No valid or recent reading available ('0')
  REPORT_SENSOR_VALID   = 2  ///< Valid distance measurement in mm (or 'H' if > 6000 mm)
};

/// @brief Automatic CSV reporting cadence mode on Core 1.
enum Core1ApplicationReportMode : uint8_t
{
  CORE1_REPORT_MODE_TIMER = 0, ///< Report emitted on active COB change
  CORE1_REPORT_MODE_DEPTH = 1  ///< Report emitted periodically every lightDurationMsec
};

/// @brief Coordinated 3-stage reboot state machine.
enum Core1ResetState : uint8_t
{
  CORE1_RESET_IDLE          = 0, ///< Normal operational state
  CORE1_RESET_REQUESTED     = 1, ///< Host requested reboot; waiting for Core 0 hardware safety
  CORE1_RESET_HARDWARE_SAFE = 2, ///< Core 0 safely shut down; 3-second countdown active
  CORE1_RESET_COMMIT        = 3  ///< Countdown expired; committing watchdog reboot
};

/// @brief Local snapshot structure representing one complete application report line.
struct Core1ApplicationReportState
{
  uint32_t timestampMs = 0;              ///< Application session elapsed time (ms)
  uint32_t button1 = 0;                  ///< PB0 debounced state (0=released, 1=pressed)
  uint32_t button2 = 0;                  ///< PB1 debounced state (0=released, 1=pressed)
  uint32_t activeLight = NO_LIGHT_ON;    ///< Currently active COB index (0..5 or 16)

  uint32_t sensorState[NUMBER_OF_DISTANCE_SENSORS] = {
    REPORT_SENSOR_NO_DATA, REPORT_SENSOR_NO_DATA, REPORT_SENSOR_NO_DATA
  };

  uint32_t distanceMm[NUMBER_OF_DISTANCE_SENSORS] = { 0, 0, 0 };
};

extern bool core1ApplicationReportingEnabled;
extern Core1OutputTarget core1ApplicationStreamTarget;
extern Core1OutputTarget core1AdvanceReportTarget;
extern Core1ApplicationReportMode core1ApplicationReportMode;
extern bool core1AdvanceReportPending;
extern uint32_t core1AdvanceConsumeCountObserved;
extern uint32_t core1AdvanceSnapshotVersionBaseline;
extern uint32_t core1LastApplicationReportMs;
extern uint32_t core1LastReportedLight;
extern bool core1SessionResetPending;
extern Core1ResetState core1ResetState;

extern uint16_t tof_distance_mm[NUMBER_OF_DISTANCE_SENSORS];
extern uint8_t tof_has_valid_distance[NUMBER_OF_DISTANCE_SENSORS];
extern unsigned long tof_last_update_ms[NUMBER_OF_DISTANCE_SENSORS];

uint32_t applicationMillis();
void reset_millis();

uint16_t centerDistanceMm(const VL53L5CX_ResultsData &r);
uint8_t chooseClosestLight(const uint16_t *depths);

bool core1PublishRecommendationSnapshot(uint32_t currentTime);
void core1RefreshApplicationReportState(uint32_t currentTime);
void core1PrintApplicationReport(Core1OutputTarget target);
void core1ConsumeRuntimeStatusSnapshot();

void core1ProcessToFConsumerLoop();
void core1ProcessApplicationReporting();
void core1ProcessResetStateMachine();
