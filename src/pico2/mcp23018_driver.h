#pragma once

// =============================================================================
//  MagicianCam4 — Rev. 2.1
//  MCP23018 I2C I/O Expander & Push-Button Subsystem Header
// =============================================================================
//  Hardware Architecture:
//    - Chip: Microchip MCP23018 16-Bit I/O Expander (Open-Drain Outputs)
//    - Bus : Fast-mode Plus I2C (1 MHz), Address 0x20
//    - Pins Managed (Port A):
//        * Bit 0 (GPA0): Push Button 0 (PB0) — active-low input with pull-up
//        * Bit 1 (GPA1): Push Button 1 (PB1) — active-low input with pull-up
//        * Bit 2 (GPA2): W5500 Hardware Reset — active-low open-drain output
//        * Bit 3 (GPA3): VL53L5CX Sensor #2 LPn — active-low open-drain output
//        * Bit 4 (GPA4): VL53L5CX Sensor #1 LPn — active-low open-drain output
//        * Bit 5 (GPA5): VL53L5CX Sensor #0 LPn — active-low open-drain output
//        * Bit 6 (GPA6): +5V Rail TPS54302 Enable — active-low open-drain output
//        * Bit 7 (GPA7): +11.5V Rail TPS54302 Enable — active-low open-drain output
//    - Concurrency Domain: Core 0 exclusive ownership for all I2C transactions.
// =============================================================================

#include "hardware_config.h"
#include "intercore_comm.h"

extern bool mcp23018_present;
extern volatile bool mcp23018_PortAIntFlg;
extern bool debouncedPB0;
extern bool debouncedPB1;
extern int pbDebounceState;

void recoverI2CBus();
bool mcp23018WriteReg(uint8_t reg, uint8_t val);
uint8_t mcp23018ReadReg(uint8_t reg);

void mcp23018SetPortABit(uint8_t bit, bool state);
void mcp23018EnablePortAPullup(uint8_t bit, bool state);
void mcp23018SetLPN(uint8_t idx, bool state);
void mcp23018ResetW5500(bool state);
void mcp23018EnableRail5V(bool state);
void mcp23018EnableRail11V(bool state);

void initMCP23018();
void __not_in_flash_func(mcp23018_PortA_ISR)();
void updateBoardLedFromButtons();
void initMcp23018ButtonStateAndInterrupt();
void processMcp23018ButtonsDebounce();
