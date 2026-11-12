// =============================================================================
//  MagicianCam4 — Rev. 2.1
//  Network Manager Implementation: Ethernet SPI0, Serial CDC, Parsing & Sliced Diagnostics
// =============================================================================

#include "network_manager.h"
#include "application_layer.h"
#include "light_controller.h"

#if USE_W5500
static byte     mac[]     = {0x02, 0xAB, 0xCD, 0x12, 0x34, 0x56};
static IPAddress ip(192, 168, 137, 64);
static byte     gateway[] = {192, 168, 137, 1};
static byte    *dns       = gateway;
static byte     subnet[]  = {255, 255, 255, 0};

EthernetServer server(23); // Telnet command server listening on port 23
EthernetClient client;     // Active connected Ethernet client
#endif

// Stack Pointer telemetry snapshots for qualitative diagnostic reporting
uintptr_t core0InitialMainStackPointer = 0;
uintptr_t core0StackPointerAtSetupEntry = 0;
uintptr_t core0StackPointerAtSetupCompletion = 0;
uintptr_t core1StackPointerAtSetup1Entry = 0;
uintptr_t core1StackPointerAtSetup1Completion = 0;
uintptr_t core1StackPointerAtDiagnosticPoint = 0;

uint32_t core1CommandSequence = 0; // Monotonic sequence counter for Core 1 commands

// =============================================================================
//  Binary Snapshot Structure (~300 bytes) & Sliced Formatter State
// =============================================================================

struct DiagnosticSnapshot
{
  // System & Network
  uint32_t packedIp;
  uint32_t ethHwStatus;
  uint32_t ethLinkStatus;
  uint32_t heapFree;
  uint32_t heapUsed;
  uint32_t heapTotal;
  uint32_t stackDispCore0SetupEntry;
  uint32_t stackDispCore0SetupComp;
  uint32_t stackDispCore1Setup1Comp;
  uint32_t stackDispCore1Diag;

  // Multicore Sync
  uint32_t core0Ready;
  uint32_t core1Running;
  uint32_t w5500Ready;
  uint32_t core1TransportReady;
  uint32_t core1SerialReady;
  uint32_t serialBytesRx;
  uint32_t appStreamTarget;
  uint32_t advanceReportTarget;
  uint32_t ethClientConnected;
  uint32_t ethBytesRx;
  uint32_t ethBytesTx;
  uint32_t core1Heartbeat;
  uint32_t core1Uptime;
  uint32_t core1LoopIter;
  uint32_t core1FramesProcessed;
  uint32_t core1RecPublished;
  uint32_t core1LastActivityMs;
  uint32_t core1LastFrameProcUs;
  uint32_t core1MaxFrameProcUs;
  uint32_t core1LastLoopDurationUs;
  uint32_t core1MaxLoopDurationUs;

  // Intercore SPSC
  uint32_t core0BytesQueued;
  uint32_t core0QueueOverflows;
  uint32_t core0QueueMaxOccupancy;
  uint32_t core1BytesTx;
  uint32_t cmdQueueOccupancy;
  uint32_t cmdQueueEnqueued;
  uint32_t cmdQueueOverflows;
  uint32_t cmdQueueMaxOccupancy;
  uint32_t cmdQueueDequeued;

  // Recommendation & Seqlock
  uint32_t recVersion;
  uint32_t recPublications;
  uint32_t recLight;
  uint32_t acceptedVersion;
  uint32_t acceptedLight;
  uint32_t acceptCount;
  uint32_t busySkips;
  uint32_t inconsistentSkips;
  uint32_t stepDepthApplications;

  // Core 0 Command Consumes
  uint32_t patternConsumeCount;
  uint32_t lastAppliedPattern;
  uint32_t autoModeConsumeCount;
  uint32_t lastAppliedAutoMode;
  uint32_t timerModeConsumeCount;
  uint32_t selectLightConsumeCount;
  uint32_t lastSelectedLight;
  uint32_t disableLightsConsumeCount;
  uint32_t advanceLightConsumeCount;
  uint32_t resetConsumeCount;
  uint32_t timedModeConsumeCount;

  // Exposure & COB Timing
  uint32_t expPulses;
  uint32_t expNormalFalls;
  uint32_t expShortPulses;
  uint32_t expGuardrailTimeouts;
  uint32_t expCyclesCompleted;

  // ToF Shared Buffers & Rates
  uint32_t tofSecondarySlotUses;
  uint32_t tofMaxOccupiedSlots;
  uint32_t tofPublished[NUMBER_OF_DISTANCE_SENSORS];
  uint32_t tofDropped[NUMBER_OF_DISTANCE_SENSORS];
  uint32_t tofConsumed[NUMBER_OF_DISTANCE_SENSORS];
  uint32_t tofLastSeq[NUMBER_OF_DISTANCE_SENSORS];
  uint32_t tofLastTsLowUs[NUMBER_OF_DISTANCE_SENSORS];
  uint32_t tofRecDepth[NUMBER_OF_DISTANCE_SENSORS];
  uint32_t tofRecSourceSeq[NUMBER_OF_DISTANCE_SENSORS];
  uint32_t tofCenterDist[NUMBER_OF_DISTANCE_SENSORS];
  uint32_t tofCalculations[NUMBER_OF_DISTANCE_SENSORS];
  uint32_t tofSlotState[NUMBER_OF_DISTANCE_SENSORS][2];
  uint32_t tofRateHzTenths[NUMBER_OF_DISTANCE_SENSORS]; // Frequency * 10
};

static DiagnosticSnapshot diagSnap;
static bool diagnosticStreamingActive = false;
static Core1OutputTarget diagnosticStreamingTarget = CORE1_OUTPUT_SERIAL;
static uint32_t diagnosticLineIndex = 0;
#define TOTAL_DIAGNOSTIC_LINES 88u

/// @brief Packs a 4-byte IP address into a single 32-bit atomic word.
static inline uint32_t packIpAddress(const IPAddress &address)
{
  return ((uint32_t)address[0] << 24) |
         ((uint32_t)address[1] << 16) |
         ((uint32_t)address[2] <<  8) |
         ((uint32_t)address[3]);
}

/// @brief Maps an incoming command source to its corresponding output routing target.
Core1OutputTarget core1OutputTargetFromSource(uint32_t source)
{
  if (source == COMMAND_SOURCE_ETHERNET) return CORE1_OUTPUT_ETHERNET;
  if (source == COMMAND_SOURCE_SERIAL)   return CORE1_OUTPUT_SERIAL;
  return CORE1_OUTPUT_NONE;
}

/// @brief Sends formatted text directly to Serial, Ethernet client, or both.
void core1SendDirectText(Core1OutputTarget target, const char *text, uint16_t length)
{
  if (!text || length == 0) return;

  if (((uint8_t)target & (uint8_t)CORE1_OUTPUT_SERIAL) != 0)
  {
    Serial.write((const uint8_t *)text, length);
  }

#if USE_W5500
  if ((((uint8_t)target & (uint8_t)CORE1_OUTPUT_ETHERNET) != 0) && client && client.connected())
  {
    size_t written = client.write((const uint8_t *)text, length);
    if (written > 0)
    {
      core1EthernetBytesTransmitted.fetch_add((uint32_t)written, std::memory_order_relaxed);
    }
  }
#endif
}

/// @brief Dequeues a single byte produced by Core 0 from the lock-free output queue.
static bool core1TryConsumeOutputByte(uint8_t &outputByte)
{
  uint32_t currentTail = core0ToCore1OutputQueue.tail.load(std::memory_order_relaxed);
  uint32_t currentHead = core0ToCore1OutputQueue.head.load(std::memory_order_acquire);

  if (currentTail == currentHead) return false;

  outputByte = core0ToCore1OutputQueue.data[currentTail & CORE0_TO_CORE1_OUTPUT_QUEUE_MASK];
  core0ToCore1OutputQueue.tail.store(currentTail + 1u, std::memory_order_release);
  return true;
}

/// @brief Transmits a bounded number of bytes from the Core0->Core1 queue to Serial CDC.
void core1DrainCore0OutputBytes(uint32_t maximumBytes)
{
  uint32_t bytesDrained = 0;
  uint8_t  outputByte = 0;

  while ((bytesDrained < maximumBytes) && core1TryConsumeOutputByte(outputByte))
  {
    Serial.write(outputByte);
    core1OutputBytesTransmitted.fetch_add(1u, std::memory_order_relaxed);
    bytesDrained++;
  }
}

/// @brief Drains up to MAX_CORE0_OUTPUT_BYTES_PER_LOOP1 (128 bytes) per loop1 iteration.
void core1DrainCore0OutputQueue()
{
  core1DrainCore0OutputBytes(MAX_CORE0_OUTPUT_BYTES_PER_LOOP1);
}

/// @brief Unconditionally flushes all pending Core 0 output bytes during bootstrap.
void core1DrainCore0BootstrapOutput()
{
  core1DrainCore0OutputBytes(UINT32_MAX);
}

/// @brief Configures SPI0 pins (GP16..GP19), initializes W5500 Ethernet, and starts Telnet server.
void core1InitializeTransport()
{
#if USE_W5500
  pinMode(W5500_CS_PIN, OUTPUT);
  digitalWrite(W5500_CS_PIN, HIGH);

  SPI.setRX(16);
  SPI.setTX(19);
  SPI.setSCK(18);
  SPI.begin();

  Ethernet.init(W5500_CS_PIN);
  Ethernet.begin(mac, ip, dns, gateway, subnet);

  int ethHwStatus = Ethernet.hardwareStatus();
  int ethLinkStatus = Ethernet.linkStatus();

  core1EthernetHardwareStatus.store((uint32_t)ethHwStatus, std::memory_order_relaxed);
  core1EthernetLinkStatus.store((uint32_t)ethLinkStatus, std::memory_order_relaxed);
  core1EthernetLocalIpPacked.store(packIpAddress(Ethernet.localIP()), std::memory_order_release);

  ETH_VERBOSE(F("ETH requested IP: "));
  ETH_VERBOSELN(ip);
  ETH_NORMAL(F("ETH HW status: "));
  ETH_NORMALLN(ethHwStatus);
  ETH_NORMAL(F("ETH link status: "));
  ETH_NORMALLN(ethLinkStatus);
  ETH_NORMAL(F("ETH local IP after begin: "));
  ETH_NORMALLN(Ethernet.localIP());
  ETH_VERBOSE(F("ETH subnet after begin: "));
  ETH_VERBOSELN(Ethernet.subnetMask());
  ETH_VERBOSE(F("ETH gateway after begin: "));
  ETH_VERBOSELN(Ethernet.gatewayIP());

  if (ethHwStatus == EthernetNoHardware)
  {
    ETH_ERRORLN(F("ETH ERROR: W5500 not detected, server NOT started"));
  }
  else
  {
    server.begin();
    ETH_NORMAL(F("ETH: server.begin() executed on actual IP "));
    ETH_NORMALLN(Ethernet.localIP());
  }
#endif

  core1TransportReady.store(1, std::memory_order_release);
}

/// @brief Broadcasts reset countdown text simultaneously to Serial and Ethernet.
void core1BroadcastResetText(const __FlashStringHelper *text)
{
  if (!text) return;
  Serial.print(text);
#if USE_W5500
  if (client && client.connected())
  {
    client.print(text);
  }
#endif
}

/// @brief Packs and enqueues a command descriptor into the Core1->Core0 SPSC queue.
bool core1TryEnqueueCommand(uint32_t command, uint32_t payload, uint32_t source, uint32_t sequence)
{
  uint32_t head = core1ToCore0CommandQueue.head.load(std::memory_order_relaxed);
  uint32_t tail = core1ToCore0CommandQueue.tail.load(std::memory_order_acquire);
  uint32_t occupancy = head - tail;

  if (occupancy >= CORE1_TO_CORE0_COMMAND_QUEUE_CAPACITY)
  {
    core1CommandQueueOverflowCount.fetch_add(1, std::memory_order_relaxed);
    return false;
  }

  uint32_t descriptor = packCoreCommandDescriptor(command, payload, source, sequence);
  core1ToCore0CommandQueue.data[head & CORE1_TO_CORE0_COMMAND_QUEUE_MASK] = descriptor;
  core1ToCore0CommandQueue.head.store(head + 1u, std::memory_order_release);
  core1CommandQueueEnqueueCount.fetch_add(1, std::memory_order_relaxed);

  uint32_t newOccupancy = occupancy + 1u;
  uint32_t previousMaximum = core1CommandQueueMaxOccupancy.load(std::memory_order_relaxed);
  if (newOccupancy > previousMaximum)
  {
    core1CommandQueueMaxOccupancy.store(newOccupancy, std::memory_order_relaxed);
  }

  return true;
}

/// @brief Captures a frozen binary snapshot (~300 bytes) of all runtime metrics in < 1 us.
void core1TriggerDiagnosticSnapshot(Core1OutputTarget target)
{
  __asm volatile("mrs %0, msp" : "=r"(core1StackPointerAtDiagnosticPoint));

  diagSnap.packedIp = core1EthernetLocalIpPacked.load(std::memory_order_acquire);
  diagSnap.ethHwStatus = core1EthernetHardwareStatus.load(std::memory_order_relaxed);
  diagSnap.ethLinkStatus = core1EthernetLinkStatus.load(std::memory_order_relaxed);

#if defined(ARDUINO_ARCH_RP2040)
  diagSnap.heapFree = (uint32_t)rp2040.getFreeHeap();
  diagSnap.heapUsed = (uint32_t)rp2040.getUsedHeap();
  diagSnap.heapTotal = (uint32_t)rp2040.getTotalHeap();
#else
  diagSnap.heapFree = 0;
  diagSnap.heapUsed = 0;
  diagSnap.heapTotal = 0;
#endif

  diagSnap.stackDispCore0SetupEntry = (core0InitialMainStackPointer >= core0StackPointerAtSetupEntry)
                                          ? (uint32_t)(core0InitialMainStackPointer - core0StackPointerAtSetupEntry) : 0;
  diagSnap.stackDispCore0SetupComp = (core0InitialMainStackPointer >= core0StackPointerAtSetupCompletion)
                                         ? (uint32_t)(core0InitialMainStackPointer - core0StackPointerAtSetupCompletion) : 0;
  diagSnap.stackDispCore1Setup1Comp = (core1StackPointerAtSetup1Entry >= core1StackPointerAtSetup1Completion)
                                          ? (uint32_t)(core1StackPointerAtSetup1Entry - core1StackPointerAtSetup1Completion) : 0;
  diagSnap.stackDispCore1Diag = (core1StackPointerAtSetup1Entry >= core1StackPointerAtDiagnosticPoint)
                                    ? (uint32_t)(core1StackPointerAtSetup1Entry - core1StackPointerAtDiagnosticPoint) : 0;

  diagSnap.core0Ready = core0SystemReady.load(std::memory_order_acquire);
  diagSnap.core1Running = core1Running.load(std::memory_order_acquire);
  diagSnap.w5500Ready = w5500HardwareReady.load(std::memory_order_acquire);
  diagSnap.core1TransportReady = core1TransportReady.load(std::memory_order_acquire);
  diagSnap.core1SerialReady = core1SerialRuntimeReady.load(std::memory_order_acquire);
  diagSnap.serialBytesRx = core1SerialBytesReceivedDirect.load(std::memory_order_relaxed);
  diagSnap.appStreamTarget = (uint32_t)core1ApplicationStreamTarget;
  diagSnap.advanceReportTarget = (uint32_t)core1AdvanceReportTarget;
  diagSnap.ethClientConnected = core1EthernetClientConnected.load(std::memory_order_acquire);
  diagSnap.ethBytesRx = core1EthernetBytesReceived.load(std::memory_order_relaxed);
  diagSnap.ethBytesTx = core1EthernetBytesTransmitted.load(std::memory_order_relaxed);
  diagSnap.core1Heartbeat = core1Heartbeat.load(std::memory_order_relaxed);
  diagSnap.core1Uptime = core1UptimeSeconds.load(std::memory_order_acquire);
  diagSnap.core1LoopIter = core1LoopIterations.load(std::memory_order_relaxed);
  diagSnap.core1FramesProcessed = core1FramesProcessed.load(std::memory_order_relaxed);
  diagSnap.core1RecPublished = core1RecommendationsPublished.load(std::memory_order_relaxed);
  diagSnap.core1LastActivityMs = core1LastActivityMs.load(std::memory_order_relaxed);
  diagSnap.core1LastFrameProcUs = core1LastFrameProcessingUs.load(std::memory_order_relaxed);
  diagSnap.core1MaxFrameProcUs = core1MaxFrameProcessingUs.load(std::memory_order_relaxed);
  diagSnap.core1LastLoopDurationUs = core1LastLoopDurationUs.load(std::memory_order_relaxed);
  diagSnap.core1MaxLoopDurationUs = core1MaxLoopDurationUs.load(std::memory_order_relaxed);

  diagSnap.core0BytesQueued = core0OutputBytesQueued.load(std::memory_order_relaxed);
  diagSnap.core0QueueOverflows = core0OutputQueueOverflowCount.load(std::memory_order_relaxed);
  diagSnap.core0QueueMaxOccupancy = core0OutputQueueMaxOccupancy.load(std::memory_order_relaxed);
  diagSnap.core1BytesTx = core1OutputBytesTransmitted.load(std::memory_order_relaxed);

  uint32_t qHead = core1ToCore0CommandQueue.head.load(std::memory_order_acquire);
  uint32_t qTail = core1ToCore0CommandQueue.tail.load(std::memory_order_acquire);
  diagSnap.cmdQueueOccupancy = qHead - qTail;
  diagSnap.cmdQueueEnqueued = core1CommandQueueEnqueueCount.load(std::memory_order_relaxed);
  diagSnap.cmdQueueOverflows = core1CommandQueueOverflowCount.load(std::memory_order_relaxed);
  diagSnap.cmdQueueMaxOccupancy = core1CommandQueueMaxOccupancy.load(std::memory_order_relaxed);
  diagSnap.cmdQueueDequeued = core0CommandQueueDequeueCount.load(std::memory_order_relaxed);

  diagSnap.recVersion = core1RecommendationSnapshot.publicationVersion.load(std::memory_order_acquire);
  diagSnap.recPublications = core1RecommendationSnapshot.publicationCount.load(std::memory_order_relaxed);
  diagSnap.recLight = core1RecommendationSnapshot.recommendedLight.load(std::memory_order_relaxed);
  diagSnap.acceptedVersion = core0AcceptedRecommendationVersion.load(std::memory_order_acquire);
  diagSnap.acceptedLight = core0AcceptedRecommendationLight.load(std::memory_order_relaxed);
  diagSnap.acceptCount = core0RecommendationAcceptCount.load(std::memory_order_relaxed);
  diagSnap.busySkips = core0RecommendationBusySkipCount.load(std::memory_order_relaxed);
  diagSnap.inconsistentSkips = core0RecommendationInconsistentSkipCount.load(std::memory_order_relaxed);
  diagSnap.stepDepthApplications = core0StepDepthRecommendationApplyCount.load(std::memory_order_relaxed);

  diagSnap.patternConsumeCount = core0PatternCommandConsumeCount.load(std::memory_order_relaxed);
  diagSnap.lastAppliedPattern = core0LastAppliedPattern.load(std::memory_order_relaxed);
  diagSnap.autoModeConsumeCount = core0AutoModeCommandConsumeCount.load(std::memory_order_relaxed);
  diagSnap.lastAppliedAutoMode = core0LastAppliedAutoMode.load(std::memory_order_relaxed);
  diagSnap.timerModeConsumeCount = core0TimerModeCommandConsumeCount.load(std::memory_order_relaxed);
  diagSnap.selectLightConsumeCount = core0SelectLightCommandConsumeCount.load(std::memory_order_relaxed);
  diagSnap.lastSelectedLight = core0LastSelectedLight.load(std::memory_order_relaxed);
  diagSnap.disableLightsConsumeCount = core0DisableLightsCommandConsumeCount.load(std::memory_order_relaxed);
  diagSnap.advanceLightConsumeCount = core0AdvanceLightCommandConsumeCount.load(std::memory_order_relaxed);
  diagSnap.resetConsumeCount = core0ResetCommandConsumeCount.load(std::memory_order_relaxed);
  diagSnap.timedModeConsumeCount = core0TimedModeCommandConsumeCount.load(std::memory_order_relaxed);

  diagSnap.expPulses = exposurePulseCount.load(std::memory_order_relaxed);
  diagSnap.expNormalFalls = exposureNormalFallCount.load(std::memory_order_relaxed);
  diagSnap.expShortPulses = exposureShortPulseCount.load(std::memory_order_relaxed);
  diagSnap.expGuardrailTimeouts = exposureGuardrailTimeoutCount.load(std::memory_order_relaxed);
  diagSnap.expCyclesCompleted = exposureCycleCompleteCount.load(std::memory_order_relaxed);

  diagSnap.tofSecondarySlotUses = tofSecondarySlotUseCount.load(std::memory_order_relaxed);
  diagSnap.tofMaxOccupiedSlots = tofMaxOccupiedSlots.load(std::memory_order_relaxed);

  for (uint8_t s = 0; s < NUMBER_OF_DISTANCE_SENSORS; s++)
  {
    diagSnap.tofPublished[s] = tofPublishedFrameCount[s].load(std::memory_order_relaxed);
    diagSnap.tofDropped[s] = tofProducerDropCount[s].load(std::memory_order_relaxed);
    diagSnap.tofConsumed[s] = tofConsumedFrameCount[s].load(std::memory_order_relaxed);
    diagSnap.tofLastSeq[s] = tofLastConsumedSequence[s].load(std::memory_order_relaxed);
    diagSnap.tofLastTsLowUs[s] = tofLastConsumedTimestampLowUs[s].load(std::memory_order_relaxed);
    diagSnap.tofRecDepth[s] = core1RecommendationSnapshot.depthMm[s].load(std::memory_order_relaxed);
    diagSnap.tofRecSourceSeq[s] = core1RecommendationSnapshot.sourceSequence[s].load(std::memory_order_relaxed);
    diagSnap.tofCenterDist[s] = core1CenterDistanceMm[s].load(std::memory_order_relaxed);
    diagSnap.tofCalculations[s] = core1CenterDistanceCalculationCount[s].load(std::memory_order_relaxed);
    diagSnap.tofSlotState[s][0] = tofSharedBuffers[s].slot[0].state.load(std::memory_order_acquire);
    diagSnap.tofSlotState[s][1] = tofSharedBuffers[s].slot[1].state.load(std::memory_order_acquire);

    if (diagSnap.core1Uptime > 0)
    {
      diagSnap.tofRateHzTenths[s] = (diagSnap.tofConsumed[s] * 10u) / diagSnap.core1Uptime;
    }
    else
    {
      diagSnap.tofRateHzTenths[s] = 0;
    }
  }

  diagnosticStreamingTarget = target;
  diagnosticLineIndex = 0;
  diagnosticStreamingActive = true;
}

/// @brief Sliced non-blocking formatter: emits exactly 1 line per loop1 tick (< 15 us).
void core1ProcessDiagnosticStreaming()
{
  if (!diagnosticStreamingActive) return;

  char buf[128];
  int len = 0;

  switch (diagnosticLineIndex)
  {
    // =========================================================================
    //  SECTION 1: SYSTEM, NETWORK & MEMORY
    // =========================================================================
    case 0: len = snprintf(buf, sizeof(buf), "\r\n=== SYSTEM, NETWORK & MEMORY ===\r\n"); break;
#if USE_W5500
    case 1:
      len = snprintf(buf, sizeof(buf), "ETH IP: %lu.%lu.%lu.%lu\r\n",
                     (unsigned long)((diagSnap.packedIp >> 24) & 0xFFu),
                     (unsigned long)((diagSnap.packedIp >> 16) & 0xFFu),
                     (unsigned long)((diagSnap.packedIp >> 8)  & 0xFFu),
                     (unsigned long)(diagSnap.packedIp & 0xFFu));
      break;
    case 2: len = snprintf(buf, sizeof(buf), "ETH HW: %lu\r\n", (unsigned long)diagSnap.ethHwStatus); break;
    case 3: len = snprintf(buf, sizeof(buf), "ETH link: %lu\r\n", (unsigned long)diagSnap.ethLinkStatus); break;
#else
    case 1: case 2: case 3:
      len = snprintf(buf, sizeof(buf), "ETH disabled\r\n"); break;
#endif
    case 4: len = snprintf(buf, sizeof(buf), "Heap free: %lu bytes\r\n", (unsigned long)diagSnap.heapFree); break;
    case 5: len = snprintf(buf, sizeof(buf), "Heap used: %lu bytes\r\n", (unsigned long)diagSnap.heapUsed); break;
    case 6: len = snprintf(buf, sizeof(buf), "Heap total: %lu bytes\r\n", (unsigned long)diagSnap.heapTotal); break;
    case 7: len = snprintf(buf, sizeof(buf), "Core0 stack displacement near setup entry: %lu\r\n", (unsigned long)diagSnap.stackDispCore0SetupEntry); break;
    case 8: len = snprintf(buf, sizeof(buf), "Core0 stack displacement at setup completion: %lu\r\n", (unsigned long)diagSnap.stackDispCore0SetupComp); break;
    case 9: len = snprintf(buf, sizeof(buf), "Core1 stack displacement at setup1 completion: %lu\r\n", (unsigned long)diagSnap.stackDispCore1Setup1Comp); break;
    case 10: len = snprintf(buf, sizeof(buf), "Core1 stack displacement at diagnostic point: %lu\r\n", (unsigned long)diagSnap.stackDispCore1Diag); break;
    case 11: len = snprintf(buf, sizeof(buf), "Core0 ready: %lu\r\n", (unsigned long)diagSnap.core0Ready); break;
    case 12: len = snprintf(buf, sizeof(buf), "Core1 running: %lu\r\n", (unsigned long)diagSnap.core1Running); break;
    case 13: len = snprintf(buf, sizeof(buf), "W5500 hardware ready: %lu\r\n", (unsigned long)diagSnap.w5500Ready); break;
    case 14: len = snprintf(buf, sizeof(buf), "Core1 transport ready: %lu\r\n", (unsigned long)diagSnap.core1TransportReady); break;
    case 15: len = snprintf(buf, sizeof(buf), "Core1 Serial runtime ready: %lu\r\n", (unsigned long)diagSnap.core1SerialReady); break;
    case 16: len = snprintf(buf, sizeof(buf), "Core1 Serial bytes received direct: %lu\r\n", (unsigned long)diagSnap.serialBytesRx); break;
    case 17: len = snprintf(buf, sizeof(buf), "Core1 application stream target: %lu\r\n", (unsigned long)diagSnap.appStreamTarget); break;
    case 18: len = snprintf(buf, sizeof(buf), "Core1 advance report target: %lu\r\n", (unsigned long)diagSnap.advanceReportTarget); break;
    case 19: len = snprintf(buf, sizeof(buf), "Core1 Ethernet client connected: %lu\r\n", (unsigned long)diagSnap.ethClientConnected); break;
    case 20: len = snprintf(buf, sizeof(buf), "Core1 Ethernet bytes received: %lu\r\n", (unsigned long)diagSnap.ethBytesRx); break;
    case 21: len = snprintf(buf, sizeof(buf), "Core1 Ethernet bytes transmitted: %lu\r\n", (unsigned long)diagSnap.ethBytesTx); break;
    case 22: len = snprintf(buf, sizeof(buf), "Core1 heartbeat: %lu\r\n", (unsigned long)diagSnap.core1Heartbeat); break;
    case 23: len = snprintf(buf, sizeof(buf), "Core1 uptime seconds: %lu\r\n", (unsigned long)diagSnap.core1Uptime); break;
    case 24: len = snprintf(buf, sizeof(buf), "Core1 loop iterations: %lu\r\n", (unsigned long)diagSnap.core1LoopIter); break;
    case 25: len = snprintf(buf, sizeof(buf), "Core1 last activity ms: %lu\r\n", (unsigned long)diagSnap.core1LastActivityMs); break;
    case 26: len = snprintf(buf, sizeof(buf), "Core1 last frame processing us: %lu\r\n", (unsigned long)diagSnap.core1LastFrameProcUs); break;
    case 27: len = snprintf(buf, sizeof(buf), "Core1 max frame processing us: %lu\r\n", (unsigned long)diagSnap.core1MaxFrameProcUs); break;
    case 28: len = snprintf(buf, sizeof(buf), "Core1 last loop duration us: %lu\r\n", (unsigned long)diagSnap.core1LastLoopDurationUs); break;
    case 29: len = snprintf(buf, sizeof(buf), "Core1 max loop duration us: %lu\r\n", (unsigned long)diagSnap.core1MaxLoopDurationUs); break;

    // =========================================================================
    //  SECTION 2: INTERCORE SPSC QUEUES
    // =========================================================================
    case 30: len = snprintf(buf, sizeof(buf), "\r\n=== INTERCORE SPSC QUEUES ===\r\n"); break;
    case 31: len = snprintf(buf, sizeof(buf), "Core0 output bytes queued: %lu\r\n", (unsigned long)diagSnap.core0BytesQueued); break;
    case 32: len = snprintf(buf, sizeof(buf), "Core0 output queue overflows: %lu\r\n", (unsigned long)diagSnap.core0QueueOverflows); break;
    case 33: len = snprintf(buf, sizeof(buf), "Core0 output queue max occupancy: %lu / %u\r\n", (unsigned long)diagSnap.core0QueueMaxOccupancy, CORE0_TO_CORE1_OUTPUT_QUEUE_CAPACITY); break;
    case 34: len = snprintf(buf, sizeof(buf), "Core1 output bytes transmitted: %lu\r\n", (unsigned long)diagSnap.core1BytesTx); break;
    case 35: len = snprintf(buf, sizeof(buf), "Core1->Core0 command queue occupancy: %lu / %u\r\n", (unsigned long)diagSnap.cmdQueueOccupancy, CORE1_TO_CORE0_COMMAND_QUEUE_CAPACITY); break;
    case 36: len = snprintf(buf, sizeof(buf), "Core1->Core0 command queue enqueued: %lu\r\n", (unsigned long)diagSnap.cmdQueueEnqueued); break;
    case 37: len = snprintf(buf, sizeof(buf), "Core1->Core0 command queue overflows: %lu\r\n", (unsigned long)diagSnap.cmdQueueOverflows); break;
    case 38: len = snprintf(buf, sizeof(buf), "Core1->Core0 command queue max occupancy: %lu\r\n", (unsigned long)diagSnap.cmdQueueMaxOccupancy); break;
    case 39: len = snprintf(buf, sizeof(buf), "Core0 command queue dequeued: %lu\r\n", (unsigned long)diagSnap.cmdQueueDequeued); break;

    // =========================================================================
    //  SECTION 3: EXPOSURE & COB HARDWARE TIMING
    // =========================================================================
    case 40: len = snprintf(buf, sizeof(buf), "\r\n=== EXPOSURE & COB HARDWARE TIMING ===\r\n"); break;
    case 41: len = snprintf(buf, sizeof(buf), "Exposure pulses triggered (t0): %lu\r\n", (unsigned long)diagSnap.expPulses); break;
    case 42: len = snprintf(buf, sizeof(buf), "Exposure normal camera falls: %lu\r\n", (unsigned long)diagSnap.expNormalFalls); break;
    case 43: len = snprintf(buf, sizeof(buf), "Exposure short pulses (<20us): %lu\r\n", (unsigned long)diagSnap.expShortPulses); break;
    case 44: len = snprintf(buf, sizeof(buf), "Exposure guardrail timeouts (1000us): %lu\r\n", (unsigned long)diagSnap.expGuardrailTimeouts); break;
    case 45: len = snprintf(buf, sizeof(buf), "Exposure cycles completed (40ms): %lu\r\n", (unsigned long)diagSnap.expCyclesCompleted); break;

    // =========================================================================
    //  SECTION 4: TOF MULTIZONE DOUBLE-BUFFERS & RATES
    // =========================================================================
    case 46: len = snprintf(buf, sizeof(buf), "\r\n=== TOF MULTIZONE DOUBLE-BUFFERS ===\r\n"); break;
    case 47: len = snprintf(buf, sizeof(buf), "ToF secondary slot uses: %lu\r\n", (unsigned long)diagSnap.tofSecondarySlotUses); break;
    case 48: len = snprintf(buf, sizeof(buf), "ToF max occupied slots: %lu / 2\r\n", (unsigned long)diagSnap.tofMaxOccupiedSlots); break;

    // Sensor 0
    case 49: len = snprintf(buf, sizeof(buf), "ToF sensor 0 rate: %lu.%lu Hz\r\n", (unsigned long)(diagSnap.tofRateHzTenths[0] / 10u), (unsigned long)(diagSnap.tofRateHzTenths[0] % 10u)); break;
    case 50: len = snprintf(buf, sizeof(buf), "ToF producer sensor 0 published: %lu, dropped: %lu\r\n", (unsigned long)diagSnap.tofPublished[0], (unsigned long)diagSnap.tofDropped[0]); break;
    case 51: len = snprintf(buf, sizeof(buf), "ToF consumer sensor 0 consumed: %lu, last seq: %lu\r\n", (unsigned long)diagSnap.tofConsumed[0], (unsigned long)diagSnap.tofLastSeq[0]); break;
    case 52: len = snprintf(buf, sizeof(buf), "ToF consumer sensor 0 center distance: %lu mm\r\n", (unsigned long)diagSnap.tofCenterDist[0]); break;
    case 53: len = snprintf(buf, sizeof(buf), "ToF sensor 0 slots state: [%lu, %lu]\r\n", (unsigned long)diagSnap.tofSlotState[0][0], (unsigned long)diagSnap.tofSlotState[0][1]); break;

    // Sensor 1
    case 54: len = snprintf(buf, sizeof(buf), "ToF sensor 1 rate: %lu.%lu Hz\r\n", (unsigned long)(diagSnap.tofRateHzTenths[1] / 10u), (unsigned long)(diagSnap.tofRateHzTenths[1] % 10u)); break;
    case 55: len = snprintf(buf, sizeof(buf), "ToF producer sensor 1 published: %lu, dropped: %lu\r\n", (unsigned long)diagSnap.tofPublished[1], (unsigned long)diagSnap.tofDropped[1]); break;
    case 56: len = snprintf(buf, sizeof(buf), "ToF consumer sensor 1 consumed: %lu, last seq: %lu\r\n", (unsigned long)diagSnap.tofConsumed[1], (unsigned long)diagSnap.tofLastSeq[1]); break;
    case 57: len = snprintf(buf, sizeof(buf), "ToF consumer sensor 1 center distance: %lu mm\r\n", (unsigned long)diagSnap.tofCenterDist[1]); break;
    case 58: len = snprintf(buf, sizeof(buf), "ToF sensor 1 slots state: [%lu, %lu]\r\n", (unsigned long)diagSnap.tofSlotState[1][0], (unsigned long)diagSnap.tofSlotState[1][1]); break;

    // Sensor 2
    case 59: len = snprintf(buf, sizeof(buf), "ToF sensor 2 rate: %lu.%lu Hz\r\n", (unsigned long)(diagSnap.tofRateHzTenths[2] / 10u), (unsigned long)(diagSnap.tofRateHzTenths[2] % 10u)); break;
    case 60: len = snprintf(buf, sizeof(buf), "ToF producer sensor 2 published: %lu, dropped: %lu\r\n", (unsigned long)diagSnap.tofPublished[2], (unsigned long)diagSnap.tofDropped[2]); break;
    case 61: len = snprintf(buf, sizeof(buf), "ToF consumer sensor 2 consumed: %lu, last seq: %lu\r\n", (unsigned long)diagSnap.tofConsumed[2], (unsigned long)diagSnap.tofLastSeq[2]); break;
    case 62: len = snprintf(buf, sizeof(buf), "ToF consumer sensor 2 center distance: %lu mm\r\n", (unsigned long)diagSnap.tofCenterDist[2]); break;
    case 63: len = snprintf(buf, sizeof(buf), "ToF sensor 2 slots state: [%lu, %lu]\r\n", (unsigned long)diagSnap.tofSlotState[2][0], (unsigned long)diagSnap.tofSlotState[2][1]); break;

    // =========================================================================
    //  SECTION 5: RECOMMENDATION & APPLICATION STATE
    // =========================================================================
    case 64: len = snprintf(buf, sizeof(buf), "\r\n=== RECOMMENDATION & SEQLOCK ===\r\n"); break;
    case 65: len = snprintf(buf, sizeof(buf), "Core1 recommendation version: %lu\r\n", (unsigned long)diagSnap.recVersion); break;
    case 66: len = snprintf(buf, sizeof(buf), "Core1 recommendation publications: %lu\r\n", (unsigned long)diagSnap.recPublications); break;
    case 67: len = snprintf(buf, sizeof(buf), "Core1 recommendation light: %lu\r\n", (unsigned long)diagSnap.recLight); break;
    case 68: len = snprintf(buf, sizeof(buf), "Core0 accepted recommendation version: %lu\r\n", (unsigned long)diagSnap.acceptedVersion); break;
    case 69: len = snprintf(buf, sizeof(buf), "Core0 accepted recommendation light: %lu\r\n", (unsigned long)diagSnap.acceptedLight); break;
    case 70: len = snprintf(buf, sizeof(buf), "Core0 recommendation accept count: %lu\r\n", (unsigned long)diagSnap.acceptCount); break;
    case 71: len = snprintf(buf, sizeof(buf), "Core0 recommendation busy skips: %lu\r\n", (unsigned long)diagSnap.busySkips); break;
    case 72: len = snprintf(buf, sizeof(buf), "Core0 recommendation inconsistent skips: %lu\r\n", (unsigned long)diagSnap.inconsistentSkips); break;
    case 73: len = snprintf(buf, sizeof(buf), "Core0 STEP_DEPTH applications: %lu\r\n", (unsigned long)diagSnap.stepDepthApplications); break;

    // =========================================================================
    //  SECTION 6: CORE 0 COMMAND HANDLER COUNTERS
    // =========================================================================
    case 74: len = snprintf(buf, sizeof(buf), "\r\n=== CORE 0 COMMAND HANDLER ===\r\n"); break;
    case 75: len = snprintf(buf, sizeof(buf), "Core0 pattern command consume count: %lu (pattern: %lu)\r\n", (unsigned long)diagSnap.patternConsumeCount, (unsigned long)diagSnap.lastAppliedPattern); break;
    case 76: len = snprintf(buf, sizeof(buf), "Core0 auto-mode command consume count: %lu (mode: %lu)\r\n", (unsigned long)diagSnap.autoModeConsumeCount, (unsigned long)diagSnap.lastAppliedAutoMode); break;
    case 77: len = snprintf(buf, sizeof(buf), "Core0 timer-mode command consume count: %lu\r\n", (unsigned long)diagSnap.timerModeConsumeCount); break;
    case 78: len = snprintf(buf, sizeof(buf), "Core0 select-light command consume count: %lu (last light: %lu)\r\n", (unsigned long)diagSnap.selectLightConsumeCount, (unsigned long)diagSnap.lastSelectedLight); break;
    case 79: len = snprintf(buf, sizeof(buf), "Core0 disable-lights command consume count: %lu\r\n", (unsigned long)diagSnap.disableLightsConsumeCount); break;
    case 80: len = snprintf(buf, sizeof(buf), "Core0 advance-light command consume count: %lu\r\n", (unsigned long)diagSnap.advanceLightConsumeCount); break;
    case 81: len = snprintf(buf, sizeof(buf), "Core0 reset command consume count: %lu\r\n", (unsigned long)diagSnap.resetConsumeCount); break;
    case 82: len = snprintf(buf, sizeof(buf), "Core0 timed-mode command consume count: %lu\r\n", (unsigned long)diagSnap.timedModeConsumeCount); break;
    case 83: len = snprintf(buf, sizeof(buf), "=== END OF DIAGNOSTIC REPORT ===\r\n\r\n"); break;

    default:
      diagnosticStreamingActive = false;
      return;
  }

  if (len > 0)
  {
    core1SendDirectText(diagnosticStreamingTarget, buf, (uint16_t)len);
  }

  diagnosticLineIndex++;
  if (diagnosticLineIndex > 83)
  {
    diagnosticStreamingActive = false;
  }
}

// -----------------------------------------------------------------------------
// Multi-character command lines (Rev 2.2): 'E<us>', 'N<n>', 'S<digits>', each ended by '\n'.
// One assembler per source so Serial and Ethernet bytes never interleave into one line.
// -----------------------------------------------------------------------------
struct CommandLineAssembler
{
  char buffer[COMMAND_LINE_MAX_LENGTH];
  uint8_t length;
  bool collecting; // Inside an 'E'/'N'/'S' line
  bool discarding; // Dropping an unknown or overlong line up to its '\n'
};
static CommandLineAssembler core1LineAssembler[4];

/// @brief Parses the decimal number following a command letter.
/// @return True if at least one digit was present; value saturates at 99999.
static bool parseCommandNumber(const char *text, uint32_t *value)
{
  uint32_t result = 0;
  bool anyDigit = false;
  while ((*text >= '0') && (*text <= '9'))
  {
    if (result < 99999u) result = (result * 10u) + (uint32_t)(*text - '0');
    anyDigit = true;
    text++;
  }
  *value = result;
  return anyDigit;
}

/// @brief Executes one complete multi-character command line.
static void core1ExecuteCommandLine(const char *line, uint32_t source)
{
  uint32_t value = 0;

  switch (line[0])
  {
    case 'E': // Light-on ceiling per exposure, clamped to [LIGHT_ON_CEILING_MIN_US, LIGHT_TURNOFF_US]
      if (!parseCommandNumber(line + 1, &value)) return;
      if (value < LIGHT_ON_CEILING_MIN_US) value = LIGHT_ON_CEILING_MIN_US;
      if (value > LIGHT_TURNOFF_US) value = LIGHT_TURNOFF_US;
      lightOnCeilingUs.store(value, std::memory_order_relaxed);
      return;

    case 'N': // Max consecutive exposures for one COB; missing/0/too large -> the hard ceiling
      if (!parseCommandNumber(line + 1, &value) || (value == 0) ||
          (value > MAX_CONSECUTIVE_ACTIVATIONS_OF_LIGHT))
      {
        value = MAX_CONSECUTIVE_ACTIVATIONS_OF_LIGHT;
      }
      lightMaxConsecutiveActivations.store(value, std::memory_order_relaxed);
      return;

    case 'S': // Light schedule: one 0-indexed digit per entry ('0'..'5'); anything else is skipped
    {
      uint32_t count = 0;
      for (const char *p = line + 1; (*p != 0) && (count < LIGHT_SCHEDULE_MAX_LENGTH); p++)
      {
        if ((*p >= '0') && (*p < (char)('0' + NUMBER_OF_LIGHTS)))
        {
          core1PendingSchedule[count++] = (uint8_t)(*p - '0');
        }
      }
      if (count == 0) return;
      core1PendingScheduleLength.store(count, std::memory_order_release);
      uint32_t sequence = ++core1CommandSequence;
      core1TryEnqueueCommand(CORE_COMMAND_SET_SCHEDULE, 0, source, sequence);
      return;
    }

    default:
      return;
  }
}

/// @brief Classifies an incoming ASCII command byte and routes actions or enqueues commands.
/// @param receivedByte ASCII character: single-byte commands i f r t a y h 0 + 1..6 z v x e, or part of an E/N/S line.
/// @param source Originating transport interface (COMMAND_SOURCE_SERIAL or COMMAND_SOURCE_ETHERNET).
void core1ClassifyCommandByte(uint8_t receivedByte, uint32_t source)
{
  CommandLineAssembler &line = core1LineAssembler[source & 3u];
  bool endOfLine = (receivedByte == '\n') || (receivedByte == '\r');

  if (line.discarding)
  {
    if (endOfLine) line.discarding = false;
    return;
  }

  if (line.collecting)
  {
    if (endOfLine)
    {
      line.buffer[line.length] = 0;
      line.collecting = false;
      if (core1ResetState == CORE1_RESET_IDLE) core1ExecuteCommandLine(line.buffer, source);
      return;
    }
    if (line.length < (COMMAND_LINE_MAX_LENGTH - 1))
    {
      line.buffer[line.length++] = (char)receivedByte;
    }
    else
    {
      line.collecting = false; // Overlong: drop the whole line, never execute a fragment
      line.discarding = true;
    }
    return;
  }

  if (endOfLine) return;
  if (core1ResetState != CORE1_RESET_IDLE) return;

  if ((receivedByte == 'E') || (receivedByte == 'N') || (receivedByte == 'S'))
  {
    line.buffer[0] = (char)receivedByte;
    line.length = 1;
    line.collecting = true;
    return;
  }

  uint32_t command = CORE_COMMAND_NONE;
  uint32_t payload = 0;

  switch (receivedByte)
  {
    case 'z': // Request software reboot countdown sequence
      core1ApplicationReportingEnabled = false;
      core1AdvanceReportPending = false;
      core1AdvanceSnapshotVersionBaseline = 0;
      core1ResetRequest.store(1, std::memory_order_release);
      core1ResetState = CORE1_RESET_REQUESTED;
      return;

    case 'x': // Trigger non-blocking diagnostic snapshot and sliced streaming
    {
      Core1OutputTarget target = core1OutputTargetFromSource(source);
      if (target != CORE1_OUTPUT_NONE)
      {
        core1TriggerDiagnosticSnapshot(target);
      }
      return;
    }

    case 'h': // Reset application session timestamp (reset_millis)
      core1SessionResetPending = true;
      return;

    case 'v': // Query firmware version directly
    {
      Core1OutputTarget target = core1OutputTargetFromSource(source);
      if (target != CORE1_OUTPUT_NONE)
      {
        char versionBuf[16];
        int vLen = snprintf(versionBuf, sizeof(versionBuf), "V:%s\r\n", VERSION_STRING);
        if (vLen > 0)
        {
          core1SendDirectText(target, versionBuf, (uint16_t)vLen);
        }
      }
      return;
    }

    case 'r': // Set Round-Robin light pattern (0,1,2,3,4,5)
      core1ApplicationReportMode = CORE1_REPORT_MODE_TIMER;
      command = CORE_COMMAND_SET_LIGHT_PATTERN;
      payload = PATTERN_ROUNDROBIN;
      break;

    case 't': // Set Opposite light pattern (0,3,1,4,2,5)
      core1ApplicationReportMode = CORE1_REPORT_MODE_TIMER;
      command = CORE_COMMAND_SET_LIGHT_PATTERN;
      payload = PATTERN_OPPOSITE;
      break;

    case 'a': // Set Auto-Depth ToF recommendation mode
      core1ApplicationReportMode = CORE1_REPORT_MODE_DEPTH;
      core1LastApplicationReportMs = applicationMillis();
      command = CORE_COMMAND_SET_AUTO_MODE;
      payload = STEP_DEPTH;
      break;

    case 'y': // Start 10-second timed flash mode
      core1ApplicationReportMode = CORE1_REPORT_MODE_TIMER;
      core1LastReportedLight = NO_LIGHT_ON;
      command = CORE_COMMAND_START_TIMED_MODE;
      break;

    case 'i': // Enable real-time CSV streaming and start timer cycling
      core1ApplicationReportingEnabled = true;
      core1ApplicationStreamTarget = core1OutputTargetFromSource(source);
      core1ApplicationReportMode = CORE1_REPORT_MODE_TIMER;
      core1LastApplicationReportMs = applicationMillis();
      core1LastReportedLight = NO_LIGHT_ON;
      command = CORE_COMMAND_START_TIMER_MODE;
      break;

    case 'f': // Disable real-time CSV streaming
      core1ApplicationReportingEnabled = false;
      core1AdvanceReportPending = false;
      core1AdvanceSnapshotVersionBaseline = 0;
      return;

    case '0': // Disable all COB illumination
      command = CORE_COMMAND_DISABLE_LIGHTS;
      break;

    case '+': // Advance to next COB and emit single synchronized CSV report
      core1AdvanceReportPending = true;
      core1AdvanceReportTarget = core1OutputTargetFromSource(source);
      core1AdvanceConsumeCountObserved = core0AdvanceLightCommandConsumeCount.load(std::memory_order_acquire);
      core1AdvanceSnapshotVersionBaseline = 0;
      command = CORE_COMMAND_ADVANCE_LIGHT;
      break;

    case '1': case '2': case '3': case '4': case '5': case '6': // Direct COB selection (1..6 -> index 0..5)
      command = CORE_COMMAND_SELECT_LIGHT;
      payload = (uint32_t)(receivedByte - '1');
      break;

    case 'e': // Exposure-locked: step the schedule once per exposure, one CSV row per strobe
      core1ApplicationReportMode = CORE1_REPORT_MODE_STROBE;
      command = CORE_COMMAND_EXPOSURE_LOCKED;
      break;

    default:
      // Unknown command letter: drop the rest of its line, so trailing digits of a
      // command this firmware does not know are never executed as light selections.
      if ((receivedByte > ' ') && (receivedByte < 0x7F)) line.discarding = true;
      return;
  }

  uint32_t sequence = ++core1CommandSequence;
  core1TryEnqueueCommand(command, payload, source, sequence);
}
