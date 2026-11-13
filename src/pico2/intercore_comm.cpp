// =============================================================================
//  MagicianCam4 — Rev. 2.2
//  Inter-Core Communication Definitions & Storage
// =============================================================================

#include "intercore_comm.h"

// -----------------------------------------------------------------------------
// Core0 -> Core1 Output Byte Queue (4096 bytes)
// -----------------------------------------------------------------------------
Core0ToCore1OutputQueue core0ToCore1OutputQueue;
std::atomic<uint32_t> core0OutputBytesQueued{0};
std::atomic<uint32_t> core0OutputQueueOverflowCount{0};
std::atomic<uint32_t> core0OutputQueueMaxOccupancy{0};
std::atomic<uint32_t> core1OutputBytesTransmitted{0};

// -----------------------------------------------------------------------------
// Core1 -> Core0 Command FIFO (256 descriptors)
// -----------------------------------------------------------------------------
Core1ToCore0CommandQueue core1ToCore0CommandQueue;
std::atomic<uint32_t> core1CommandQueueEnqueueCount{0};
std::atomic<uint32_t> core1CommandQueueOverflowCount{0};
std::atomic<uint32_t> core1CommandQueueMaxOccupancy{0};

// -----------------------------------------------------------------------------
// ToF Shared Double Buffers
// -----------------------------------------------------------------------------
ToFSensorDoubleBuffer tofSharedBuffers[NUMBER_OF_DISTANCE_SENSORS];
uint32_t tofProducerSequence[NUMBER_OF_DISTANCE_SENSORS] = {0, 0, 0};
std::atomic<uint32_t> tofPublishedFrameCount[NUMBER_OF_DISTANCE_SENSORS];
std::atomic<uint32_t> tofProducerDropCount[NUMBER_OF_DISTANCE_SENSORS];
std::atomic<uint32_t> tofConsumedFrameCount[NUMBER_OF_DISTANCE_SENSORS];
std::atomic<uint32_t> tofLastConsumedSequence[NUMBER_OF_DISTANCE_SENSORS];
std::atomic<uint32_t> tofLastConsumedTimestampLowUs[NUMBER_OF_DISTANCE_SENSORS];
std::atomic<uint32_t> tofSecondarySlotUseCount{0};
std::atomic<uint32_t> tofMaxOccupiedSlots{0};

// -----------------------------------------------------------------------------
// Exposure & COB Timing Telemetry Atomics
// -----------------------------------------------------------------------------
std::atomic<uint32_t> exposurePulseCount{0};
std::atomic<uint32_t> exposureNormalFallCount{0};
std::atomic<uint32_t> exposureShortPulseCount{0};
std::atomic<uint32_t> exposureGuardrailTimeoutCount{0};
std::atomic<uint32_t> exposureCycleCompleteCount{0};

// -----------------------------------------------------------------------------
// Host-Adjustable Light Limits & Strobe Ground Truth (Rev 2.2)
// -----------------------------------------------------------------------------
std::atomic<uint32_t> lightOnCeilingUs{LIGHT_TURNOFF_US};
std::atomic<uint32_t> lightMaxConsecutiveActivations{0};
std::atomic<uint32_t> lightLastStrobeWord{STROBE_WORD_LIGHT_FIELD};
std::atomic<uint32_t> lightForcedAdvanceCount{0};
std::atomic<uint32_t> lightStarvedCount{0};
uint8_t core1PendingSchedule[LIGHT_SCHEDULE_MAX_LENGTH];
std::atomic<uint32_t> core1PendingScheduleLength{0};
std::atomic<uint32_t> core1PendingScheduleVersion{0};
std::atomic<uint32_t> core0RailState{RAIL_STATE_5V_ON | RAIL_STATE_11V_ON};
std::atomic<uint32_t> core0RailCommandConsumeCount{0};
std::atomic<uint32_t> core0RailRefusedCount{0};
std::atomic<uint32_t> core1RailCycleOffMs{RAIL_CYCLE_DEFAULT_OFF_MS};

// -----------------------------------------------------------------------------
// Seqlock Atomic Snapshots
// -----------------------------------------------------------------------------
Core1ToCore0RecommendationSnapshot core1RecommendationSnapshot;
Core0RuntimeStatusSnapshot core0RuntimeStatusSnapshot;

// -----------------------------------------------------------------------------
// Multicore Synchronization and Statistics Atomics
// -----------------------------------------------------------------------------
std::atomic<uint32_t> core0SystemReady{0};
std::atomic<uint32_t> core1Running{0};
std::atomic<uint32_t> core1Heartbeat{0};
std::atomic<uint32_t> core1UptimeSeconds{0};
std::atomic<uint32_t> w5500HardwareReady{0};
std::atomic<uint32_t> core1TransportReady{0};
std::atomic<uint32_t> core1SerialRuntimeReady{0};

std::atomic<uint32_t> core0CommandQueueDequeueCount{0};
std::atomic<uint32_t> core0PatternCommandConsumeCount{0};
std::atomic<uint32_t> core0LastAppliedPattern{0};
std::atomic<uint32_t> core0AutoModeCommandConsumeCount{0};
std::atomic<uint32_t> core0LastAppliedAutoMode{0};
std::atomic<uint32_t> core0TimerModeCommandConsumeCount{0};
std::atomic<uint32_t> core0SelectLightCommandConsumeCount{0};
std::atomic<uint32_t> core0LastSelectedLight{NO_LIGHT_ON};
std::atomic<uint32_t> core0DisableLightsCommandConsumeCount{0};
std::atomic<uint32_t> core0AdvanceLightCommandConsumeCount{0};
std::atomic<uint32_t> core0ResetCommandConsumeCount{0};
std::atomic<uint32_t> core0TimedModeCommandConsumeCount{0};
std::atomic<uint32_t> core0ExposureLockedCommandConsumeCount{0};
std::atomic<uint32_t> core0ScheduleCommandConsumeCount{0};

std::atomic<uint32_t> core0AcceptedRecommendationVersion{0};
std::atomic<uint32_t> core0AcceptedRecommendationLight{NO_LIGHT_ON};
std::atomic<uint32_t> core0RecommendationAcceptCount{0};
std::atomic<uint32_t> core0RecommendationBusySkipCount{0};
std::atomic<uint32_t> core0RecommendationInconsistentSkipCount{0};
std::atomic<uint32_t> core0StepDepthRecommendationApplyCount{0};

std::atomic<uint32_t> core1EthernetHardwareStatus{0};
std::atomic<uint32_t> core1EthernetLinkStatus{0};
std::atomic<uint32_t> core1EthernetLocalIpPacked{0};
std::atomic<uint32_t> core1EthernetClientConnected{0};
std::atomic<uint32_t> core1EthernetBytesReceived{0};
std::atomic<uint32_t> core1EthernetBytesTransmitted{0};
std::atomic<uint32_t> core1SerialBytesReceivedDirect{0};

std::atomic<uint16_t> core1CenterDistanceMm[NUMBER_OF_DISTANCE_SENSORS];
std::atomic<uint32_t> core1CenterDistanceCalculationCount[NUMBER_OF_DISTANCE_SENSORS];

std::atomic<uint32_t> core1LoopIterations{0};
std::atomic<uint32_t> core1LastActivityMs{0};
std::atomic<uint32_t> core1FramesProcessed{0};
std::atomic<uint32_t> core1RecommendationsPublished{0};
std::atomic<uint32_t> core1LastFrameProcessingUs{0};
std::atomic<uint32_t> core1MaxFrameProcessingUs{0};
std::atomic<uint32_t> core1LastLoopDurationUs{0};
std::atomic<uint32_t> core1MaxLoopDurationUs{0};

std::atomic<uint32_t> core1ResetRequest{0};
std::atomic<uint32_t> core0ResetHardwareSafe{0};

/// @brief Atomically enqueues a single output byte into the Core0->Core1 SPSC queue.
/// @param byteValue Character or byte to enqueue.
/// @return True on success, false if queue is full (drops byte without blocking).
bool core0TryQueueOutputByte(uint8_t byteValue)
{
  uint32_t head = core0ToCore1OutputQueue.head.load(std::memory_order_relaxed);
  uint32_t tail = core0ToCore1OutputQueue.tail.load(std::memory_order_acquire);
  uint32_t occupancy = head - tail;

  if (occupancy >= CORE0_TO_CORE1_OUTPUT_QUEUE_CAPACITY)
  {
    core0OutputQueueOverflowCount.fetch_add(1, std::memory_order_relaxed);
    return false;
  }

  core0ToCore1OutputQueue.data[head & CORE0_TO_CORE1_OUTPUT_QUEUE_MASK] = byteValue;
  core0ToCore1OutputQueue.head.store(head + 1u, std::memory_order_release);
  core0OutputBytesQueued.fetch_add(1u, std::memory_order_relaxed);

  uint32_t newOccupancy = occupancy + 1u;
  uint32_t previousMaximum = core0OutputQueueMaxOccupancy.load(std::memory_order_relaxed);
  if (newOccupancy > previousMaximum)
  {
    core0OutputQueueMaxOccupancy.store(newOccupancy, std::memory_order_relaxed);
  }

  return true;
}

void core0QueueOutputText(const char *text)
{
  if (!text) return;
  while (*text) { core0TryQueueOutputByte(static_cast<uint8_t>(*text++)); }
}

void core0QueueOutputFlashText(const __FlashStringHelper *text)
{
  if (!text) return;
  PGM_P p = reinterpret_cast<PGM_P>(text);
  while (true)
  {
    uint8_t c = pgm_read_byte(p++);
    if (c == 0) break;
    core0TryQueueOutputByte(c);
  }
}

void core0QueueOutputChar(char value)
{
  core0TryQueueOutputByte(static_cast<uint8_t>(value));
}

void core0QueueOutputUnsigned(uint32_t value)
{
  char buffer[12];
  snprintf(buffer, sizeof(buffer), "%lu", static_cast<unsigned long>(value));
  core0QueueOutputText(buffer);
}

void core0QueueOutputSigned(int32_t value)
{
  char buffer[12];
  snprintf(buffer, sizeof(buffer), "%ld", static_cast<long>(value));
  core0QueueOutputText(buffer);
}

void core0QueueOutputHexByte(uint8_t value)
{
  static const char hexChars[] = "0123456789ABCDEF";
  core0TryQueueOutputByte(static_cast<uint8_t>(hexChars[(value >> 4) & 0x0F]));
  core0TryQueueOutputByte(static_cast<uint8_t>(hexChars[value & 0x0F]));
}

void core0QueueOutputNewline()
{
  core0TryQueueOutputByte('\r');
  core0TryQueueOutputByte('\n');
}

void core0QueueOutputValue(const char *value)               { core0QueueOutputText(value); }
void core0QueueOutputValue(const __FlashStringHelper *value) { core0QueueOutputFlashText(value); }
void core0QueueOutputValue(char value)                       { core0QueueOutputChar(value); }
void core0QueueOutputValue(uint8_t value)                    { core0QueueOutputUnsigned(static_cast<uint32_t>(value)); }
void core0QueueOutputValue(uint16_t value)                   { core0QueueOutputUnsigned(static_cast<uint32_t>(value)); }
void core0QueueOutputValue(uint32_t value)                   { core0QueueOutputUnsigned(value); }
void core0QueueOutputValue(int value)                        { core0QueueOutputSigned(static_cast<int32_t>(value)); }
