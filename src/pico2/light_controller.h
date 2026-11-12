#pragma once

// =============================================================================
//  MagicianCam4 — Rev. 2.1
//  COB LED Lighting Controller & Hardware-Timed Camera Exposure Subsystem Header
// =============================================================================
//  Subsystem Architecture:
//    - COB LED Channels: 6 direct active-high GPIO lines (GP6..GP11)
//    - Gating Trigger   : Sony XCG-CP510 shutter open signal on GP2
//    - Real-Time Model  : Hard real-time NVIC Priority 0 (SRAM-resident ISRs)
//    - SIO Direct Drive : Single-cycle direct register writes (`sio_hw->gpio_set/clr`)
//    - Safety Guardrails: 3-stage RP2350 hardware timer alarm referenced to t0:
//        * Stage 1: 20 us noise guard window before arming FALL interrupt.
//        * Stage 2: 1000 us maximum ON-time timeout guardrail.
//        * Stage 3: 40000 us minimum cycle period (<= 25 Hz maximum rate).
//    - Real-Time Metrics: Zero-jitter atomic counters for exposure diagnostics.
// =============================================================================

#include "hardware_config.h"
#include "intercore_comm.h"

/// @brief Exposure-synchronous COB state machine lifecycle phases.
enum LightCycleState : uint32_t
{
  LIGHT_STATE_READY       = 0, ///< Armed for shutter exposure rising edge (t0)
  LIGHT_STATE_ARMING_FALL = 1, ///< COB ON; 20 us noise guard window active
  LIGHT_STATE_WAIT_FALL   = 2, ///< COB ON; armed for shutter falling edge
  LIGHT_STATE_KEEP_OFF    = 3  ///< COB OFF; 40 ms cycle cooldown active
};

#define STEP_EXTERNAL 0 ///< Advance light only upon explicit host '+' command
#define STEP_TIMER    1 ///< Advance light automatically every lightDurationMsec
#define STEP_DEPTH    2 ///< Re-evaluate light from ToF depths every lightDurationMsec
#define STEP_TIMED    4 ///< Advance light on timer with 10s auto-stop

#define PATTERN_ROUNDROBIN 0 ///< Step sequentially through adjacent COBs: 0,1,2,3,4,5
#define PATTERN_OPPOSITE   3 ///< Step across opposing COBs: 0,3,1,4,2,5

extern const uint8_t directLightControl[NUMBER_OF_LIGHTS];
extern volatile uint8_t lightOn;
extern volatile uint32_t activeLightMask;
extern volatile LightCycleState lightCycleState;
extern int lightHardwareAlarmNum;
extern uint64_t risingEdgeTimeUs;

extern unsigned char autoLights;
extern unsigned char lightPattern;
extern unsigned int lightDurationMsec;
extern unsigned long loopRateMsec;
extern unsigned long lastUpdateTime;
extern unsigned long lightStartTime;
extern unsigned long flashStartTime;

void initLightController();
void activateLight(uint8_t lightIdx);
void deactivateLights();
uint8_t getNextLight(uint8_t current, uint8_t total, uint8_t pattern);

void __not_in_flash_func(exposureISR)();
void __not_in_flash_func(lightHardwareTimerCallback)(uint alarm_num);
