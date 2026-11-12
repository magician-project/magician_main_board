#pragma once

// =============================================================================
//  MagicianCam4 — Rev. 2.1
//  Network Manager, Serial / Ethernet Transports & Sliced Diagnostic Header
// =============================================================================
//  Subsystem Architecture:
//    - Serial Transport  : USB CDC Serial @ 115200 baud
//    - Ethernet Transport: W5500 SPI0 Telnet command server on port 23 (192.168.137.64)
//    - Sliced Streaming  : Non-blocking diagnostic reporter streaming 1 line / tick (< 15 us)
//    - Concurrency Domain: Core 1 exclusive ownership for all transport I/O
// =============================================================================

#include "hardware_config.h"
#include "intercore_comm.h"

extern EthernetServer server;
extern EthernetClient client;

extern uintptr_t core0InitialMainStackPointer;
extern uintptr_t core0StackPointerAtSetupEntry;
extern uintptr_t core0StackPointerAtSetupCompletion;
extern uintptr_t core1StackPointerAtSetup1Entry;
extern uintptr_t core1StackPointerAtSetup1Completion;
extern uintptr_t core1StackPointerAtDiagnosticPoint;
extern uint32_t core1CommandSequence;

void core1InitializeTransport();
void core1DrainCore0BootstrapOutput();
void core1DrainCore0OutputQueue();
void core1DrainCore0OutputBytes(uint32_t maximumBytes);

bool core1TryEnqueueCommand(uint32_t command, uint32_t payload, uint32_t source, uint32_t sequence);
void core1ClassifyCommandByte(uint8_t receivedByte, uint32_t source);
void core1TriggerDiagnosticSnapshot(Core1OutputTarget target);
void core1ProcessDiagnosticStreaming();
void core1BroadcastResetText(const __FlashStringHelper *text);
void core1SendDirectText(Core1OutputTarget target, const char *text, uint16_t length);
Core1OutputTarget core1OutputTargetFromSource(uint32_t source);
