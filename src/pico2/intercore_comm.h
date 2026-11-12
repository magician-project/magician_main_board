#pragma once

// =============================================================================
//  MagicianCam4 — Rev. 2.1
//  Inter-Core Communication: Lock-Free SPSC Queues, Shared Buffers & Atomics
// =============================================================================
//  This module defines the lock-free inter-core communication channels between
//  Core 0 (real-time control domain) and Core 1 (communication/application domain).
//
//  Concurrency Principles:
//    - Strictly Lock-Free: No spinlocks, mutexes or blocking critical sections
//      are used across the inter-core boundary.
//    - Single-Producer Single-Consumer (SPSC): Ring buffers use power-of-two
//      monotonic head/tail uint32_t indices with acquire/release memory ordering.
//    - Seqlock Versioned Snapshots: Coherent multi-word state transfers use
//      monotonic version counters (even = complete/valid, odd = in-progress).
//    - Lock-Free Double Buffers: ToF sensor frames transition through atomic
//      states (FREE -> FILLING -> READY -> PROCESSING -> FREE).
//    - Binary Diagnostic Snapshot: Captures all ~60 runtime metrics in < 1 us,
//      streamed out by Core 1 in 1-line slices (< 15 us / tick) with zero RAM waste.
// =============================================================================

#include "hardware_config.h"

// =============================================================================
//  Command Descriptors & Bit-Packing Constants (Core 1 -> Core 0 FIFO)
// =============================================================================

/// @brief Command identifiers transmitted from Core 1 to Core 0 via SPSC queue.
enum Core1ToCore0CommandId : uint32_t
{
  CORE_COMMAND_NONE              = 0, ///< No operation / idle command
  CORE_COMMAND_SET_LIGHT_PATTERN = 1, ///< Set active stepping pattern (RoundRobin/Opposite)
  CORE_COMMAND_SET_AUTO_MODE     = 2, ///< Set automatic illumination mode (STEP_DEPTH, etc.)
  CORE_COMMAND_START_TIMER_MODE  = 3, ///< Start timer-driven continuous light cycling ('i')
  CORE_COMMAND_SELECT_LIGHT      = 4, ///< Select a specific COB LED index ('1'..'6')
  CORE_COMMAND_DISABLE_LIGHTS    = 5, ///< Turn off all COB LED illumination ('0')
  CORE_COMMAND_ADVANCE_LIGHT     = 6, ///< Advance to next COB in sequence ('+')
  CORE_COMMAND_RESET_REQUEST     = 7, ///< Request hardware software reset sequence ('z')
  CORE_COMMAND_START_TIMED_MODE  = 8, ///< Start timed flash mode (10s auto-stop) ('y')
  CORE_COMMAND_EXPOSURE_LOCKED   = 9, ///< Step through the light schedule once per exposure ('e')
  CORE_COMMAND_SET_SCHEDULE      = 10 ///< Apply core1PendingSchedule as the light schedule ('S<digits>')
};

/// @brief Source interface from which a command was originated.
enum CommandSource : uint32_t
{
  COMMAND_SOURCE_NONE     = 0, ///< Internal / software originated
  COMMAND_SOURCE_SERIAL   = 1, ///< Received via USB CDC Serial
  COMMAND_SOURCE_ETHERNET = 2  ///< Received via W5500 Telnet server
};

// -----------------------------------------------------------------------------
// Bitfield allocation for 32-bit packed command descriptor:
//   bits  0..3   : Command ID (4 bits, 0..15)
//   bits  4..7   : Payload / light index (4 bits, 0..15)
//   bits  8..9   : Source interface (2 bits, 0..3)
//   bits 10..31  : Monotonic sequence counter (22 bits, 0..4194303)
// -----------------------------------------------------------------------------
#define CORE_COMMAND_ID_MASK        0x0000000Fu
#define CORE_COMMAND_PAYLOAD_MASK   0x0000000Fu
#define CORE_COMMAND_SOURCE_MASK    0x00000003u
#define CORE_COMMAND_SEQUENCE_MASK  0x003FFFFFu

#define CORE_COMMAND_PAYLOAD_SHIFT   4u
#define CORE_COMMAND_SOURCE_SHIFT    8u
#define CORE_COMMAND_SEQUENCE_SHIFT 10u

/// @brief Packs command parameters into a single 32-bit atomic descriptor.
static inline uint32_t packCoreCommandDescriptor(uint32_t command, uint32_t payload, uint32_t source, uint32_t sequence)
{
  return (command & CORE_COMMAND_ID_MASK) |
         ((payload & CORE_COMMAND_PAYLOAD_MASK) << CORE_COMMAND_PAYLOAD_SHIFT) |
         ((source & CORE_COMMAND_SOURCE_MASK) << CORE_COMMAND_SOURCE_SHIFT) |
         ((sequence & CORE_COMMAND_SEQUENCE_MASK) << CORE_COMMAND_SEQUENCE_SHIFT);
}

/// @brief Extracts the 4-bit command ID from a packed descriptor.
static inline uint32_t unpackCoreCommandId(uint32_t descriptor)
{
  return descriptor & CORE_COMMAND_ID_MASK;
}

/// @brief Extracts the 4-bit payload from a packed descriptor.
static inline uint32_t unpackCoreCommandPayload(uint32_t descriptor)
{
  return (descriptor >> CORE_COMMAND_PAYLOAD_SHIFT) & CORE_COMMAND_PAYLOAD_MASK;
}

/// @brief Extracts the 2-bit source identifier from a packed descriptor.
static inline uint32_t unpackCoreCommandSource(uint32_t descriptor)
{
  return (descriptor >> CORE_COMMAND_SOURCE_SHIFT) & CORE_COMMAND_SOURCE_MASK;
}

/// @brief Extracts the 22-bit diagnostic sequence number from a packed descriptor.
static inline uint32_t unpackCoreCommandSequence(uint32_t descriptor)
{
  return (descriptor >> CORE_COMMAND_SEQUENCE_SHIFT) & CORE_COMMAND_SEQUENCE_MASK;
}

// =============================================================================
//  Core0 -> Core1 Output Byte Queue (Capacity: 4096 Bytes)
// =============================================================================
//  Producer : Core 0 (writes boot logs, hardware status, diagnostic strings)
//  Consumer : Core 1 (drains bytes and transmits them physically to Serial CDC)
// =============================================================================
#define CORE0_TO_CORE1_OUTPUT_QUEUE_CAPACITY 4096u
#define CORE0_TO_CORE1_OUTPUT_QUEUE_MASK     (CORE0_TO_CORE1_OUTPUT_QUEUE_CAPACITY - 1u)

struct Core0ToCore1OutputQueue
{
  uint8_t data[CORE0_TO_CORE1_OUTPUT_QUEUE_CAPACITY];
  std::atomic<uint32_t> head{0};
  std::atomic<uint32_t> tail{0};
};

extern Core0ToCore1OutputQueue core0ToCore1OutputQueue;
extern std::atomic<uint32_t> core0OutputBytesQueued;
extern std::atomic<uint32_t> core0OutputQueueOverflowCount;
extern std::atomic<uint32_t> core0OutputQueueMaxOccupancy;
extern std::atomic<uint32_t> core1OutputBytesTransmitted;

// =============================================================================
//  Core1 -> Core0 Command FIFO (Capacity: 256 Entries)
// =============================================================================
//  Producer : Core 1 (enqueues parsed commands from Serial / Ethernet)
//  Consumer : Core 0 (dequeues and executes commands within loop())
// =============================================================================
#define CORE1_TO_CORE0_COMMAND_QUEUE_CAPACITY 256u
#define CORE1_TO_CORE0_COMMAND_QUEUE_MASK     (CORE1_TO_CORE0_COMMAND_QUEUE_CAPACITY - 1u)

struct Core1ToCore0CommandQueue
{
  uint32_t data[CORE1_TO_CORE0_COMMAND_QUEUE_CAPACITY];
  std::atomic<uint32_t> head{0};
  std::atomic<uint32_t> tail{0};
};

extern Core1ToCore0CommandQueue core1ToCore0CommandQueue;
extern std::atomic<uint32_t> core1CommandQueueEnqueueCount;
extern std::atomic<uint32_t> core1CommandQueueOverflowCount;
extern std::atomic<uint32_t> core1CommandQueueMaxOccupancy;

// =============================================================================
//  Core1 Output Target Specification
// =============================================================================
enum Core1OutputTarget : uint8_t
{
  CORE1_OUTPUT_NONE     = 0, ///< Discard output
  CORE1_OUTPUT_SERIAL   = 1, ///< Route line exclusively to Serial CDC
  CORE1_OUTPUT_ETHERNET = 2, ///< Route line exclusively to Ethernet Telnet client
  CORE1_OUTPUT_BOTH     = 3  ///< Broadcast line to both Serial and Ethernet
};

// =============================================================================
//  ToF Shared Double-Buffers (Lock-Free Multi-Slot State Machine)
// =============================================================================
//  Producer : Core 0 (copies raw 8x8 VL53L5CX results into an available slot)
//  Consumer : Core 1 (computes central 4-zone depth averages and recommendations)
// =============================================================================
enum ToFSlotState : uint32_t
{
  TOF_SLOT_FREE       = 0, ///< Slot is empty and available for production
  TOF_SLOT_FILLING    = 1, ///< Producer (Core 0) is writing frame payload
  TOF_SLOT_READY      = 2, ///< Frame payload is complete and ready for consumption
  TOF_SLOT_PROCESSING = 3  ///< Consumer (Core 1) is analyzing frame payload
};

struct ToFFrameSlot
{
  VL53L5CX_ResultsData frame;
  uint64_t timestampUs;
  uint32_t sequence;
  std::atomic<uint32_t> state;
};

struct ToFSensorDoubleBuffer
{
  ToFFrameSlot slot[2];
};

extern ToFSensorDoubleBuffer tofSharedBuffers[NUMBER_OF_DISTANCE_SENSORS];

// Producer & Consumer Diagnostic Telemetry
extern uint32_t tofProducerSequence[NUMBER_OF_DISTANCE_SENSORS];
extern std::atomic<uint32_t> tofPublishedFrameCount[NUMBER_OF_DISTANCE_SENSORS];
extern std::atomic<uint32_t> tofProducerDropCount[NUMBER_OF_DISTANCE_SENSORS];
extern std::atomic<uint32_t> tofConsumedFrameCount[NUMBER_OF_DISTANCE_SENSORS];
extern std::atomic<uint32_t> tofLastConsumedSequence[NUMBER_OF_DISTANCE_SENSORS];
extern std::atomic<uint32_t> tofLastConsumedTimestampLowUs[NUMBER_OF_DISTANCE_SENSORS];
extern std::atomic<uint32_t> tofSecondarySlotUseCount;
extern std::atomic<uint32_t> tofMaxOccupiedSlots;

// =============================================================================
//  Exposure & COB Timing Telemetry Atomics
// =============================================================================
extern std::atomic<uint32_t> exposurePulseCount;
extern std::atomic<uint32_t> exposureNormalFallCount;
extern std::atomic<uint32_t> exposureShortPulseCount;
extern std::atomic<uint32_t> exposureGuardrailTimeoutCount;
extern std::atomic<uint32_t> exposureCycleCompleteCount;

// =============================================================================
//  Host-Adjustable Light Limits & Strobe Ground Truth (Rev 2.2)
// =============================================================================
//  Written by Core 1 when it parses 'E<us>' / 'N<n>', read by the Core 0 exposure ISRs.
extern std::atomic<uint32_t> lightOnCeilingUs;              ///< Max COB on-time per exposure (us), clamped
extern std::atomic<uint32_t> lightMaxConsecutiveActivations;///< Max exposures in a row for one COB (1..12)
//  Written by the Core 0 exposure ISR, read by Core 1 for the CSV report.
extern std::atomic<uint32_t> lightLastStrobeLight;          ///< COB that actually fired last (NO_LIGHT_ON = none yet)
extern std::atomic<uint32_t> lightForcedAdvanceCount;       ///< Exposures where the 'N' limit swapped in another COB
extern std::atomic<uint32_t> lightStarvedCount;             ///< Exposures left dark: 'N' limit hit, no other COB allowed
//  'S<digits>' hand-off: Core 1 fills the buffer, then enqueues CORE_COMMAND_SET_SCHEDULE.
extern uint8_t core1PendingSchedule[LIGHT_SCHEDULE_MAX_LENGTH];
extern std::atomic<uint32_t> core1PendingScheduleLength;

// =============================================================================
//  Versioned Atomic Snapshots (Seqlock Pattern)
// =============================================================================

/// @brief Coherent ToF recommendation published by Core 1 and read by Core 0.
struct Core1ToCore0RecommendationSnapshot
{
  std::atomic<uint32_t> publicationVersion{0}; ///< Even=stable snapshot, Odd=in-progress
  std::atomic<uint32_t> depthMm[NUMBER_OF_DISTANCE_SENSORS];
  std::atomic<uint32_t> sourceSequence[NUMBER_OF_DISTANCE_SENSORS];
  std::atomic<uint32_t> recommendedLight{NO_LIGHT_ON};
  std::atomic<uint32_t> publicationCount{0};
};

/// @brief Coherent runtime status published by Core 0 and consumed by Core 1.
struct Core0RuntimeStatusSnapshot
{
  std::atomic<uint32_t> publicationVersion{0}; ///< Even=stable snapshot, Odd=in-progress
  std::atomic<uint32_t> button1{0};            ///< Debounced PB0 state (1=pressed, 0=released)
  std::atomic<uint32_t> button2{0};            ///< Debounced PB1 state (1=pressed, 0=released)
  std::atomic<uint32_t> activeLight{NO_LIGHT_ON}; ///< Currently selected COB index (0..5 or 16)
};

extern Core1ToCore0RecommendationSnapshot core1RecommendationSnapshot;
extern Core0RuntimeStatusSnapshot core0RuntimeStatusSnapshot;

// =============================================================================
//  Multicore Bootstrap Synchronization & System Statistics Atomics
// =============================================================================
extern std::atomic<uint32_t> core0SystemReady;
extern std::atomic<uint32_t> core1Running;
extern std::atomic<uint32_t> core1Heartbeat;
extern std::atomic<uint32_t> core1UptimeSeconds;
extern std::atomic<uint32_t> w5500HardwareReady;
extern std::atomic<uint32_t> core1TransportReady;
extern std::atomic<uint32_t> core1SerialRuntimeReady;

extern std::atomic<uint32_t> core0CommandQueueDequeueCount;
extern std::atomic<uint32_t> core0PatternCommandConsumeCount;
extern std::atomic<uint32_t> core0LastAppliedPattern;
extern std::atomic<uint32_t> core0AutoModeCommandConsumeCount;
extern std::atomic<uint32_t> core0LastAppliedAutoMode;
extern std::atomic<uint32_t> core0TimerModeCommandConsumeCount;
extern std::atomic<uint32_t> core0SelectLightCommandConsumeCount;
extern std::atomic<uint32_t> core0LastSelectedLight;
extern std::atomic<uint32_t> core0DisableLightsCommandConsumeCount;
extern std::atomic<uint32_t> core0AdvanceLightCommandConsumeCount;
extern std::atomic<uint32_t> core0ResetCommandConsumeCount;
extern std::atomic<uint32_t> core0TimedModeCommandConsumeCount;

extern std::atomic<uint32_t> core0AcceptedRecommendationVersion;
extern std::atomic<uint32_t> core0AcceptedRecommendationLight;
extern std::atomic<uint32_t> core0RecommendationAcceptCount;
extern std::atomic<uint32_t> core0RecommendationBusySkipCount;
extern std::atomic<uint32_t> core0RecommendationInconsistentSkipCount;
extern std::atomic<uint32_t> core0StepDepthRecommendationApplyCount;

extern std::atomic<uint32_t> core1EthernetHardwareStatus;
extern std::atomic<uint32_t> core1EthernetLinkStatus;
extern std::atomic<uint32_t> core1EthernetLocalIpPacked;
extern std::atomic<uint32_t> core1EthernetClientConnected;
extern std::atomic<uint32_t> core1EthernetBytesReceived;
extern std::atomic<uint32_t> core1EthernetBytesTransmitted;
extern std::atomic<uint32_t> core1SerialBytesReceivedDirect;

extern std::atomic<uint16_t> core1CenterDistanceMm[NUMBER_OF_DISTANCE_SENSORS];
extern std::atomic<uint32_t> core1CenterDistanceCalculationCount[NUMBER_OF_DISTANCE_SENSORS];

extern std::atomic<uint32_t> core1LoopIterations;
extern std::atomic<uint32_t> core1LastActivityMs;
extern std::atomic<uint32_t> core1FramesProcessed;
extern std::atomic<uint32_t> core1RecommendationsPublished;
extern std::atomic<uint32_t> core1LastFrameProcessingUs;
extern std::atomic<uint32_t> core1MaxFrameProcessingUs;
extern std::atomic<uint32_t> core1LastLoopDurationUs;
extern std::atomic<uint32_t> core1MaxLoopDurationUs;

extern std::atomic<uint32_t> core1ResetRequest;
extern std::atomic<uint32_t> core0ResetHardwareSafe;

// Core0 Non-Blocking Queue Publication Helper
bool core0TryQueueOutputByte(uint8_t byteValue);
