#pragma once

// =============================================================================
//  MagicianCam4 — Rev. 2.1
//  VL53L5CX Time-of-Flight Ranging & Multi-Sensor Manager Header
// =============================================================================
//  Subsystem Architecture:
//    - Sensors : 3x STMicroelectronics VL53L5CX 8x8 Multizone ToF Ranging Sensors
//    - Interface: Fast-mode Plus I2C (1 MHz) with hardware LPn line isolation
//    - Addresses: Default 0x29 -> Reassigned to 0x30, 0x31, 0x32
//    - Interrupt: GP12, GP13, GP14 Data-Ready active-low pulses
//    - Double-Buffer: Lock-free SPSC buffer transferring 8x8 frames to Core 1
// =============================================================================

#include "hardware_config.h"
#include "intercore_comm.h"
#include "mcp23018_driver.h"

extern Adafruit_VL53L5CX VL53L5CXSensor[NUMBER_OF_DISTANCE_SENSORS];
extern uint8_t tofSensorInitialized[NUMBER_OF_DISTANCE_SENSORS];
extern volatile bool dataReadyFlags[NUMBER_OF_DISTANCE_SENSORS];

/// @brief Classification of I2C bus state upon startup.
typedef enum {
  TOF_BUS_COLD_SILENT,        ///< Full power-on cycle: all sensors silent after LPn LOW
  TOF_BUS_WARM_ASSIGNED,      ///< MCU soft-reset: sensors retained assigned addresses 0x30..0x32
  TOF_BUS_MIXED_OR_AMBIGUOUS  ///< Mixed state requiring individual per-sensor recovery
} ToFBusPath;

/// @brief I2C acknowledgment snapshot across default and assigned ToF addresses.
typedef struct {
  uint8_t ackDefault;
  uint8_t ackAssigned[NUMBER_OF_DISTANCE_SENSORS];
} ToFBusSnapshot;

void scanI2C();
void setI2CDistanceAddresses();
void tofPrintInitSummary();
void attachTofInterruptsForWorkingSensors();
void processTofInterruptEvents();

void __not_in_flash_func(sensor0_ISR)();
void __not_in_flash_func(sensor1_ISR)();
void __not_in_flash_func(sensor2_ISR)();
