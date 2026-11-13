// =============================================================================
//  MagicianCam4 — Rev. 2.2
//  VL53L5CX Driver Implementation: Boot-State Aware Addressing & Interrupt Acquisition
// =============================================================================

#include "tof_manager.h"

Adafruit_VL53L5CX VL53L5CXSensor[NUMBER_OF_DISTANCE_SENSORS];
VL53L5CX_ResultsData VL53L5CXResults;

uint8_t tofSensorInitialized[NUMBER_OF_DISTANCE_SENSORS] = {0};
volatile bool dataReadyFlags[NUMBER_OF_DISTANCE_SENSORS] = {false, false, false};

static const uint8_t LOX_ADDRESS[NUMBER_OF_DISTANCE_SENSORS] = {LOX1_ADDRESS, LOX2_ADDRESS, LOX3_ADDRESS};
static const uint8_t sensorIntPins[NUMBER_OF_DISTANCE_SENSORS] = {SENSOR0_INT_PIN, SENSOR1_INT_PIN, SENSOR2_INT_PIN};

// Minimal SRAM Data-Ready ISRs (latch boolean flags without I2C or Serial operations)
void __not_in_flash_func(sensor0_ISR)() { dataReadyFlags[0] = true; }
void __not_in_flash_func(sensor1_ISR)() { dataReadyFlags[1] = true; }
void __not_in_flash_func(sensor2_ISR)() { dataReadyFlags[2] = true; }

/// @brief Tests I2C communication with a target 7-bit address.
/// @return 0 on ACK, non-zero on NACK or bus error.
static uint8_t i2cAck(uint8_t addr)
{
  Wire.beginTransmission(addr);
  return Wire.endTransmission();
}

#if VL53L5CX_REPORT_LEVEL >= VL53L5CX_REPORT_VERBOSE
/// @brief Performs diagnostic address scan on default (0x29) and assigned (0x30..0x32) ToF addresses.
void scanI2C()
{
  static const uint8_t known[] = {0x29, 0x30, 0x31, 0x32};
  VL53L5CX_VERBOSELN(F("VL53L5CX: address scan"));
  uint8_t count = 0;
  for (uint8_t k = 0; k < sizeof(known) / sizeof(known[0]); k++)
  {
    uint8_t addr = known[k];
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0)
    {
      VL53L5CX_VERBOSE(F("  ACK @ "));
      VL53L5CX_VERBOSE_HEX_ADDRESS(addr);
      VL53L5CX_VERBOSE_NEWLINE();
      count++;
    }
  }
  if (count == 0) { VL53L5CX_VERBOSELN(F("  no VL53L5CX ACK")); }
}
#else
void scanI2C() {}
#endif

/// @brief Clears initialization flags for all Time-of-Flight sensors.
static void tofClearStatus()
{
  for (uint8_t i = 0; i < NUMBER_OF_DISTANCE_SENSORS; i++)
  {
    tofSensorInitialized[i] = 0;
  }
}

/// @brief Forces all VL53L5CX LPn lines LOW via MCP23018 to isolate sensors on I2C bus.
static void tofForceAllLpnLow()
{
  VL53L5CX_VERBOSELN(F("VL53L5CX: forcing all LPn LOW"));
  mcp23018SetLPN(0, false);
  mcp23018SetLPN(1, false);
  mcp23018SetLPN(2, false);
  delay(VL53L5CX_LPN_SLEEP_MS);
}

/// @brief Releases all VL53L5CX LPn lines HIGH via MCP23018.
static void tofReleaseAllLpnHigh()
{
  VL53L5CX_VERBOSELN(F("VL53L5CX: releasing all LPn HIGH"));
  mcp23018SetLPN(0, true);
  mcp23018SetLPN(1, true);
  mcp23018SetLPN(2, true);
  delay(VL53L5CX_LPN_RESUME_MS);
}

/// @brief Reads I2C acknowledgments across default (0x29) and assigned (0x30..0x32) addresses.
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

/// @brief Prints diagnostic details of an I2C bus snapshot.
static void tofPrintBusSnapshot(const ToFBusSnapshot &snapshot)
{
  VL53L5CX_VERBOSE(F("VL53L5CX: 0x29 "));
  if (snapshot.ackDefault == 0)
  {
    VL53L5CX_VERBOSELN(F("ACK"));
  }
  else
  {
    VL53L5CX_VERBOSE(F("no ACK, code "));
    VL53L5CX_VERBOSE_UNSIGNED(snapshot.ackDefault);
    VL53L5CX_VERBOSE_NEWLINE();
  }

  for (uint8_t i = 0; i < NUMBER_OF_DISTANCE_SENSORS; i++)
  {
    VL53L5CX_VERBOSE(F("VL53L5CX: "));
    VL53L5CX_VERBOSE_HEX_ADDRESS(LOX_ADDRESS[i]);

    if (snapshot.ackAssigned[i] == 0)
    {
      VL53L5CX_VERBOSELN(F(" ACK"));
    }
    else
    {
      VL53L5CX_VERBOSE(F(" no ACK, code "));
      VL53L5CX_VERBOSE_UNSIGNED(snapshot.ackAssigned[i]);
      VL53L5CX_VERBOSE_NEWLINE();
    }
  }
}

/// @brief Classifies bus state following LPn LOW assertion (Cold, Warm, or Mixed).
static ToFBusPath tofClassifyBusAfterAllLpnLow(const ToFBusSnapshot &s)
{
  bool defaultPresent = (s.ackDefault == 0);
  bool anyAssigned = false;
  bool allAssigned = true;

  for (uint8_t i = 0; i < NUMBER_OF_DISTANCE_SENSORS; i++)
  {
    bool assignedPresent = (s.ackAssigned[i] == 0);
    if (assignedPresent) { anyAssigned = true; }
    else                 { allAssigned = false; }
  }

  if (!defaultPresent && !anyAssigned)
  {
    return TOF_BUS_COLD_SILENT;
  }
  if (!defaultPresent && allAssigned)
  {
    return TOF_BUS_WARM_ASSIGNED;
  }
  return TOF_BUS_MIXED_OR_AMBIGUOUS;
}

/// @brief Waits for and clears the startup frame after startRanging() to avoid stale data.
static bool tofWaitAndClearFirstFrame(uint8_t i, uint32_t timeoutMs)
{
  VL53L5CX_VERBOSE(F("VL53L5CX["));
  VL53L5CX_VERBOSE_DECIMAL(i);
  VL53L5CX_VERBOSELN(F("]: waiting for safe first frame"));

  uint32_t startMs = millis();
  while ((uint32_t)(millis() - startMs) < timeoutMs)
  {
    if (VL53L5CXSensor[i].isDataReady())
    {
      if (VL53L5CXSensor[i].getRangingData(&VL53L5CXResults))
      {
        VL53L5CX_VERBOSE(F("VL53L5CX["));
        VL53L5CX_VERBOSE_DECIMAL(i);
        VL53L5CX_VERBOSELN(F("]: safe first frame cleared"));
        return true;
      }
    }
    delay(5);
  }

  VL53L5CX_ERROR(F("VL53L5CX["));
  VL53L5CX_NORMAL_DECIMAL(i);
  VL53L5CX_ERRORLN(F("] ERROR: safe first frame timeout"));
  return false;
}

/// @brief Executes cold initialization for a single sensor starting from default 0x29.
static bool tofColdInitOneSensor(uint8_t i)
{
  VL53L5CX_VERBOSE(F("VL53L5CX["));
  VL53L5CX_VERBOSE_DECIMAL(i);
  VL53L5CX_VERBOSELN(F("]: cold init from 0x29"));

  if (i2cAck(0x29) != 0)
  {
    VL53L5CX_ERROR(F("VL53L5CX["));
    VL53L5CX_NORMAL_DECIMAL(i);
    VL53L5CX_ERRORLN(F("] ERROR: no ACK at 0x29"));
    return false;
  }

  uint32_t beginStartMs = millis();
  if (!VL53L5CXSensor[i].begin(0x29, &Wire, 1000000))
  {
    VL53L5CX_ERROR(F("VL53L5CX["));
    VL53L5CX_NORMAL_DECIMAL(i);
    VL53L5CX_ERRORLN(F("] ERROR: begin(0x29) failed"));
    return false;
  }
  uint32_t beginElapsedMs = millis() - beginStartMs;

  VL53L5CX_VERBOSE(F("VL53L5CX["));
  VL53L5CX_VERBOSE_DECIMAL(i);
  VL53L5CX_VERBOSE(F("]: begin(0x29) OK, "));
  VL53L5CX_VERBOSE_UNSIGNED(beginElapsedMs);
  VL53L5CX_VERBOSELN(F(" ms"));

  if (!VL53L5CXSensor[i].setAddress(LOX_ADDRESS[i]))
  {
    VL53L5CX_ERROR(F("VL53L5CX["));
    VL53L5CX_NORMAL_DECIMAL(i);
    VL53L5CX_ERRORLN(F("] ERROR: setAddress failed"));
    return false;
  }

  if (i2cAck(LOX_ADDRESS[i]) != 0)
  {
    VL53L5CX_ERROR(F("VL53L5CX["));
    VL53L5CX_NORMAL_DECIMAL(i);
    VL53L5CX_ERRORLN(F("] ERROR: no ACK after address change"));
    return false;
  }

  if (!VL53L5CXSensor[i].setResolution(VL53L5CX_RESOLUTION_8X8)) return false;
  if (!VL53L5CXSensor[i].setRangingFrequency(10)) return false;
  if (!VL53L5CXSensor[i].startRanging()) return false;

  return tofWaitAndClearFirstFrame(i, VL53L5CX_FIRST_FRAME_TIMEOUT_MS);
}

/// @brief Reattaches to a sensor that retained its assigned runtime address (Warm start).
static bool tofWarmAttachOneSensor(uint8_t i)
{
  VL53L5CX_VERBOSE(F("VL53L5CX["));
  VL53L5CX_VERBOSE_DECIMAL(i);
  VL53L5CX_VERBOSE(F("]: warm attach at "));
  VL53L5CX_VERBOSE_HEX_ADDRESS(LOX_ADDRESS[i]);
  VL53L5CX_VERBOSE_NEWLINE();

  uint32_t beginStartMs = millis();
  if (!VL53L5CXSensor[i].begin(LOX_ADDRESS[i], &Wire, 1000000))
  {
    VL53L5CX_ERROR(F("VL53L5CX["));
    VL53L5CX_NORMAL_DECIMAL(i);
    VL53L5CX_ERRORLN(F("] ERROR: warm begin() failed"));
    return false;
  }
  uint32_t beginElapsedMs = millis() - beginStartMs;

  VL53L5CX_VERBOSE(F("VL53L5CX["));
  VL53L5CX_VERBOSE_DECIMAL(i);
  VL53L5CX_VERBOSE(F("]: begin("));
  VL53L5CX_VERBOSE_HEX_ADDRESS(LOX_ADDRESS[i]);
  VL53L5CX_VERBOSE(F(") OK, "));
  VL53L5CX_VERBOSE_UNSIGNED(beginElapsedMs);
  VL53L5CX_VERBOSELN(F(" ms"));

  VL53L5CXSensor[i].stopRanging();
  delay(10);

  if (!VL53L5CXSensor[i].setResolution(VL53L5CX_RESOLUTION_8X8)) return false;
  if (!VL53L5CXSensor[i].setRangingFrequency(10)) return false;
  if (!VL53L5CXSensor[i].startRanging()) return false;

  return tofWaitAndClearFirstFrame(i, VL53L5CX_FIRST_FRAME_TIMEOUT_MS);
}

/// @brief Sequentially initializes sensors 0..2 one at a time via LPn line release (Cold path).
static void tofRunColdSequencing()
{
  VL53L5CX_NORMALLN(F("VL53L5CX: COLD/LPn-isolated sequencing path"));

  for (uint8_t i = 0; i < NUMBER_OF_DISTANCE_SENSORS; i++)
  {
    tofSensorInitialized[i] = 0;

    VL53L5CX_VERBOSE(F("VL53L5CX["));
    VL53L5CX_VERBOSE_DECIMAL(i);
    VL53L5CX_VERBOSELN(F("]: releasing LPn"));

    mcp23018SetLPN(i, true);
    delay(VL53L5CX_LPN_RESUME_MS);

    if (!tofColdInitOneSensor(i))
    {
      VL53L5CX_ERROR(F("VL53L5CX["));
      VL53L5CX_NORMAL_DECIMAL(i);
      VL53L5CX_ERRORLN(F("] ERROR: unavailable"));
      mcp23018SetLPN(i, false);
      delay(20);
      continue;
    }

    tofSensorInitialized[i] = 1;

    VL53L5CX_NORMAL(F("VL53L5CX["));
    VL53L5CX_NORMAL_DECIMAL(i);
    VL53L5CX_NORMAL(F("]: ready at "));
    VL53L5CX_NORMAL_HEX_ADDRESS(LOX_ADDRESS[i]);
    VL53L5CX_NORMAL_NEWLINE();
  }
}

/// @brief Reattaches all sensors directly at their assigned runtime addresses (Warm path).
static void tofRunWarmAttach()
{
  VL53L5CX_NORMALLN(F("VL53L5CX: WARM assigned-address attach path"));
  tofReleaseAllLpnHigh();

  for (uint8_t i = 0; i < NUMBER_OF_DISTANCE_SENSORS; i++)
  {
    tofSensorInitialized[i] = 0;
    uint8_t ackAssigned = i2cAck(LOX_ADDRESS[i]);

    if (ackAssigned != 0)
    {
      VL53L5CX_ERROR(F("VL53L5CX["));
      VL53L5CX_NORMAL_DECIMAL(i);
      VL53L5CX_ERROR(F("] ERROR: no ACK at assigned address "));
      core0QueueOutputHexByte(LOX_ADDRESS[i]);
      core0QueueOutputNewline();
      VL53L5CX_ERROR(F("VL53L5CX["));
      VL53L5CX_NORMAL_DECIMAL(i);
      VL53L5CX_ERRORLN(F("] ERROR: unavailable"));
      mcp23018SetLPN(i, false);
      delay(20);
      continue;
    }

    if (!tofWarmAttachOneSensor(i))
    {
      VL53L5CX_ERROR(F("VL53L5CX["));
      VL53L5CX_NORMAL_DECIMAL(i);
      VL53L5CX_ERRORLN(F("] ERROR: unavailable"));
      mcp23018SetLPN(i, false);
      delay(20);
      continue;
    }

    tofSensorInitialized[i] = 1;

    VL53L5CX_NORMAL(F("VL53L5CX["));
    VL53L5CX_NORMAL_DECIMAL(i);
    VL53L5CX_NORMAL(F("]: ready at "));
    VL53L5CX_NORMAL_HEX_ADDRESS(LOX_ADDRESS[i]);
    VL53L5CX_NORMAL_NEWLINE();
  }
}

/// @brief Fallback per-sensor recovery evaluating both default and assigned addresses.
static void tofRunMixedRecovery()
{
  VL53L5CX_NORMALLN(F("VL53L5CX: MIXED/AMBIGUOUS recovery path"));
  VL53L5CX_VERBOSELN(F("VL53L5CX: using per-sensor default/assigned detection"));

  for (uint8_t i = 0; i < NUMBER_OF_DISTANCE_SENSORS; i++)
  {
    tofSensorInitialized[i] = 0;

    for (uint8_t j = 0; j < NUMBER_OF_DISTANCE_SENSORS; j++)
    {
      if (j < i)       { mcp23018SetLPN(j, tofSensorInitialized[j] != 0); }
      else if (j == i) { mcp23018SetLPN(j, true); }
      else             { mcp23018SetLPN(j, false); }
    }

    delay(VL53L5CX_LPN_RESUME_MS);

    uint8_t ackDefault = i2cAck(0x29);
    uint8_t ackAssigned = i2cAck(LOX_ADDRESS[i]);

    bool ok = false;
    if (ackAssigned == 0 && ackDefault == 0)
    {
      VL53L5CX_ERROR(F("VL53L5CX["));
      VL53L5CX_NORMAL_DECIMAL(i);
      VL53L5CX_ERRORLN(F("] ERROR: default and assigned addresses both ACK"));
    }
    else if (ackAssigned == 0)
    {
      ok = tofWarmAttachOneSensor(i);
    }
    else if (ackDefault == 0)
    {
      ok = tofColdInitOneSensor(i);
    }

    if (!ok)
    {
      VL53L5CX_ERROR(F("VL53L5CX["));
      VL53L5CX_NORMAL_DECIMAL(i);
      VL53L5CX_ERRORLN(F("] ERROR: unavailable"));
      mcp23018SetLPN(i, false);
      delay(20);
      continue;
    }

    tofSensorInitialized[i] = 1;

    VL53L5CX_NORMAL(F("VL53L5CX["));
    VL53L5CX_NORMAL_DECIMAL(i);
    VL53L5CX_NORMAL(F("]: ready at "));
    VL53L5CX_NORMAL_HEX_ADDRESS(LOX_ADDRESS[i]);
    VL53L5CX_NORMAL_NEWLINE();
  }
}

/// @brief Establishes final LPn lines state and executes post-init diagnostic bus scan.
static void tofApplyFinalLpnState()
{
  VL53L5CX_VERBOSELN(F("VL53L5CX: final LPn state"));

  for (uint8_t i = 0; i < NUMBER_OF_DISTANCE_SENSORS; i++)
  {
    if (tofSensorInitialized[i])
    {
      mcp23018SetLPN(i, true);
    }
    else
    {
      mcp23018SetLPN(i, false);
      VL53L5CX_VERBOSE(F("VL53L5CX["));
      VL53L5CX_VERBOSE_DECIMAL(i);
      VL53L5CX_VERBOSELN(F("]: LPn LOW, unavailable sensor isolated"));
    }
  }

  delay(20);

#if VL53L5CX_REPORT_LEVEL >= VL53L5CX_REPORT_VERBOSE
  VL53L5CX_VERBOSELN(F("VL53L5CX: final address scan"));
  scanI2C();
#endif
}

/// @brief Counts the total number of successfully initialized ToF sensors.
static uint8_t tofCountWorkingSensors()
{
  uint8_t count = 0;
  for (uint8_t i = 0; i < NUMBER_OF_DISTANCE_SENSORS; i++)
  {
    if (tofSensorInitialized[i]) count++;
  }
  return count;
}

/// @brief Outputs boot summary of initialized Time-of-Flight sensors.
void tofPrintInitSummary()
{
  uint8_t working = tofCountWorkingSensors();

  if (working == NUMBER_OF_DISTANCE_SENSORS)
  {
    VL53L5CX_NORMAL(F("VL53L5CX: ready, working sensors = "));
    VL53L5CX_NORMAL_UNSIGNED(working);
    VL53L5CX_NORMAL(F("/"));
    VL53L5CX_NORMAL_UNSIGNED(NUMBER_OF_DISTANCE_SENSORS);
    VL53L5CX_NORMAL_NEWLINE();
    return;
  }

  if (working > 0)
  {
    VL53L5CX_WARNING(F("VL53L5CX WARNING: degraded, working sensors = "));
    core0QueueOutputUnsigned(working);
    VL53L5CX_WARNING(F("/"));
    core0QueueOutputUnsigned(NUMBER_OF_DISTANCE_SENSORS);
    core0QueueOutputNewline();
    return;
  }

  VL53L5CX_ERROR(F("VL53L5CX ERROR: no working sensors, 0/"));
  core0QueueOutputUnsigned(NUMBER_OF_DISTANCE_SENSORS);
  core0QueueOutputNewline();
}

/// @brief Attaches hardware falling-edge interrupts on GP12..GP14 for operational sensors.
void attachTofInterruptsForWorkingSensors()
{
  uint8_t count = 0;
  for (uint8_t i = 0; i < NUMBER_OF_DISTANCE_SENSORS; i++)
  {
    if (tofSensorInitialized[i])
    {
      pinMode(sensorIntPins[i], INPUT);
      switch (i)
      {
        case 0: attachInterrupt(digitalPinToInterrupt(sensorIntPins[0]), sensor0_ISR, FALLING); break;
        case 1: attachInterrupt(digitalPinToInterrupt(sensorIntPins[1]), sensor1_ISR, FALLING); break;
        case 2: attachInterrupt(digitalPinToInterrupt(sensorIntPins[2]), sensor2_ISR, FALLING); break;
      }
      count++;
    }
  }

  if (count == NUMBER_OF_DISTANCE_SENSORS)
  {
    VL53L5CX_NORMALLN(F("VL53L5CX: interrupts attached = 3/3"));
  }
  else if (count > 0)
  {
    VL53L5CX_WARNING(F("VL53L5CX WARNING: interrupts attached = "));
    VL53L5CX_NORMAL_DECIMAL(count);
    VL53L5CX_WARNING(F("/"));
    VL53L5CX_NORMAL_DECIMAL(NUMBER_OF_DISTANCE_SENSORS);
    VL53L5CX_WARNINGLN(F(""));
  }
  else
  {
    VL53L5CX_ERRORLN(F("VL53L5CX ERROR: interrupts attached = 0/3"));
  }
}

/// @brief Orchestrates boot-state aware addressing and configuration of 3x VL53L5CX sensors.
void setI2CDistanceAddresses()
{
  tofClearStatus();
  tofForceAllLpnLow();

  ToFBusSnapshot snapshot = tofReadBusSnapshot();
  tofPrintBusSnapshot(snapshot);

  ToFBusPath path = tofClassifyBusAfterAllLpnLow(snapshot);

  switch (path)
  {
    case TOF_BUS_COLD_SILENT:
      tofRunColdSequencing();
      break;

    case TOF_BUS_WARM_ASSIGNED:
      tofRunWarmAttach();
      break;

    case TOF_BUS_MIXED_OR_AMBIGUOUS:
    default:
      tofRunMixedRecovery();
      break;
  }

  tofApplyFinalLpnState();
}

/// @brief Atomically copies an acquired 8x8 ToF frame into a free double-buffer slot.
/// @param sensor Sensor index (0, 1, or 2).
/// @param sourceFrame Raw 8x8 VL53L5CX results data.
/// @return True on publication, false if dropped (both slots occupied).
static bool publishToFFrameToSharedBuffer(uint8_t sensor, const VL53L5CX_ResultsData &sourceFrame)
{
  if (sensor >= NUMBER_OF_DISTANCE_SENSORS) return false;

  ToFSensorDoubleBuffer &doubleBuffer = tofSharedBuffers[sensor];
  ToFFrameSlot *selectedSlot = nullptr;

  for (int slotIndex = 0; slotIndex < 2; slotIndex++)
  {
    ToFFrameSlot &candidate = doubleBuffer.slot[slotIndex];
    uint32_t expectedState = TOF_SLOT_FREE;
    if (candidate.state.compare_exchange_strong(
            expectedState, TOF_SLOT_FILLING,
            std::memory_order_acquire, std::memory_order_relaxed))
    {
      selectedSlot = &candidate;
      if (slotIndex == 1)
      {
        tofSecondarySlotUseCount.fetch_add(1, std::memory_order_relaxed);
      }
      break;
    }
  }

  if (selectedSlot == nullptr)
  {
    tofProducerDropCount[sensor].fetch_add(1, std::memory_order_relaxed);
    return false;
  }

  selectedSlot->frame = sourceFrame;
  selectedSlot->timestampUs = time_us_64();
  selectedSlot->sequence = ++tofProducerSequence[sensor];
  selectedSlot->state.store(TOF_SLOT_READY, std::memory_order_release);

  tofPublishedFrameCount[sensor].fetch_add(1, std::memory_order_relaxed);

  uint32_t occupiedSlots = 0;
  for (uint8_t i = 0; i < 2; i++)
  {
    uint32_t state = tofSharedBuffers[sensor].slot[i].state.load(std::memory_order_acquire);
    if (state != TOF_SLOT_FREE) occupiedSlots++;
  }

  uint32_t previousMax = tofMaxOccupiedSlots.load(std::memory_order_relaxed);
  if (occupiedSlots > previousMax)
  {
    tofMaxOccupiedSlots.store(occupiedSlots, std::memory_order_relaxed);
  }

  return true;
}

/// @brief Retrieves ranging results over I2C and publishes to lock-free double buffer.
static void handleToFData(uint8_t i)
{
  if (i >= NUMBER_OF_DISTANCE_SENSORS) return;
  if (!tofSensorInitialized[i]) return;

  if (VL53L5CXSensor[i].getRangingData(&VL53L5CXResults))
  {
    publishToFFrameToSharedBuffer(i, VL53L5CXResults);
  }
}

/// @brief Checks latched Data-Ready interrupt flags and retrieves ready frames.
/// @note Called by Core 0 within loop().
void processTofInterruptEvents()
{
  for (uint8_t i = 0; i < NUMBER_OF_DISTANCE_SENSORS; i++)
  {
    if (!tofSensorInitialized[i]) continue;

    if (dataReadyFlags[i])
    {
      dataReadyFlags[i] = false;
      handleToFData(i);
    }
  }
}
