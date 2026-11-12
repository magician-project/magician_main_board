// =============================================================================
//  MagicianCam4 — Rev. 2.1
//  MCP23018 I2C I/O Expander & Push-Button Driver Implementation
// =============================================================================

#include "mcp23018_driver.h"

bool mcp23018_present = false;

// Software shadow registers to eliminate redundant I2C reads during bit-level writes
static uint8_t mcp23018_portA_olata = 0x3C; // Default OLATA output latch mirror
static uint8_t mcp23018_portA_gppua = 0x03; // Default GPPUA pull-up register mirror

volatile bool mcp23018_PortAIntFlg = false; // Latch set exclusively by GPIO ISR
static uint8_t prevPortA = 0xFF;            // Previous debounced PORTA state
static uint8_t currPortA = 0xFF;            // Current sampled PORTA state

bool debouncedPB0 = false;                  // Authoritative debounced PB0 state (true=pressed)
bool debouncedPB1 = false;                  // Authoritative debounced PB1 state (true=pressed)
int pbDebounceState = 0;                    // Debounce state machine (0=idle, 1..4=stabilizing)

/// @brief Clears bus lockups by bit-banging 16 SCL clock cycles and a manual STOP.
/// @note Called exclusively during early setup() before attaching the I2C hardware driver.
void recoverI2CBus()
{
  // 1. Release both lines to let external 2.2k pull-ups float them HIGH
  pinMode(SDA_PIN, INPUT);
  pinMode(SCL_PIN, INPUT);
  delay(2);

  // 2. Generate 16 clock cycles on SCL to allow any slave stuck mid-byte to release SDA
  for (uint8_t i = 0; i < 16; i++)
  {
    digitalWrite(SCL_PIN, LOW);
    pinMode(SCL_PIN, OUTPUT);
    delayMicroseconds(5);

    pinMode(SCL_PIN, INPUT);
    delayMicroseconds(5);
  }

  // 3. Generate a manual I2C STOP condition: transition SDA LOW -> HIGH while SCL is HIGH
  digitalWrite(SDA_PIN, LOW);
  pinMode(SDA_PIN, OUTPUT);
  delayMicroseconds(5);

  pinMode(SCL_PIN, INPUT);
  delayMicroseconds(5);

  pinMode(SDA_PIN, INPUT);
  delayMicroseconds(5);

  pinMode(SDA_PIN, INPUT);
  pinMode(SCL_PIN, INPUT);
  delay(1);
}

/// @brief Writes a single 8-bit value to a specified MCP23018 register over I2C.
/// @param reg MCP23018 internal register address.
/// @param val 8-bit configuration value.
/// @return True on I2C ACK (0), false on NACK or bus error.
bool mcp23018WriteReg(uint8_t reg, uint8_t val)
{
  Wire.beginTransmission(MCP23018_ADDR);
  Wire.write(reg);
  Wire.write(val);
  return (Wire.endTransmission() == 0);
}

/// @brief Reads a single 8-bit value from a specified MCP23018 register over I2C.
/// @param reg MCP23018 internal register address.
/// @return 8-bit register contents, or 0xFF on bus communication error.
uint8_t mcp23018ReadReg(uint8_t reg)
{
  Wire.beginTransmission(MCP23018_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission() != 0) return 0xFF;
  Wire.requestFrom((uint8_t)MCP23018_ADDR, (uint8_t)1);
  if (Wire.available()) return Wire.read();
  return 0xFF;
}

/// @brief Sets or clears a single output bit on Port A using the shadow latch register.
/// @param bit Bit position (0..7).
/// @param state True to set bit HIGH (open-drain floating), false to pull LOW (active GND clamp).
void mcp23018SetPortABit(uint8_t bit, bool state)
{
  if (!mcp23018_present) return;
  if (state) mcp23018_portA_olata |= (1 << bit);
  else       mcp23018_portA_olata &= ~(1 << bit);
  mcp23018WriteReg(MCP23018_OLATA, mcp23018_portA_olata);
}

/// @brief Enables or disables the internal ~100k pull-up resistor on a Port A pin.
/// @param bit Bit position (0..7).
/// @param state True to enable internal pull-up, false to disable.
void mcp23018EnablePortAPullup(uint8_t bit, bool state)
{
  if (!mcp23018_present) return;
  if (state) mcp23018_portA_gppua |= (1 << bit);
  else       mcp23018_portA_gppua &= ~(1 << bit);
  mcp23018WriteReg(MCP23018_GPPUA, mcp23018_portA_gppua);
}

/// @brief Controls the hardware Low-Power (LPn) pin of a specified VL53L5CX sensor.
/// @param idx Sensor index (0, 1, or 2).
/// @param state True to wake up sensor (LPn HIGH), false to isolate on I2C bus (LPn LOW).
void mcp23018SetLPN(uint8_t idx, bool state)
{
  if (idx >= 3) return;
  static const uint8_t lpnMap[3] = {MCP_GPA_LPN0, MCP_GPA_LPN1, MCP_GPA_LPN2};
  mcp23018SetPortABit(lpnMap[idx], state);
}

/// @brief Controls the active-low hardware reset pin of the W5500 Ethernet Lite board.
/// @param state True to assert hardware reset (GPA2 LOW), false to release reset (GPA2 HIGH).
void mcp23018ResetW5500(bool state)
{
  mcp23018SetPortABit(MCP_GPA_W5500RST, !state);
}

/// @brief Controls the +5V power rail TPS54302 switching regulator enabling line.
/// @param state True to enable +5V rail (clamps enable line to GND), false to disable.
void mcp23018EnableRail5V(bool state)
{
  if (state)
  {
    mcp23018EnablePortAPullup(MCP_GPA_RAIL5V, false);
    mcp23018SetPortABit(MCP_GPA_RAIL5V, false);
  }
  else
  {
    mcp23018SetPortABit(MCP_GPA_RAIL5V, true);
    mcp23018EnablePortAPullup(MCP_GPA_RAIL5V, true);
  }
}

/// @brief Controls the +11.5V power rail TPS54302 switching regulator enabling line.
/// @param state True to enable +11.5V rail (clamps enable line to GND), false to disable.
void mcp23018EnableRail11V(bool state)
{
  if (state)
  {
    mcp23018EnablePortAPullup(MCP_GPA_RAIL11V, false);
    mcp23018SetPortABit(MCP_GPA_RAIL11V, false);
  }
  else
  {
    mcp23018SetPortABit(MCP_GPA_RAIL11V, true);
    mcp23018EnablePortAPullup(MCP_GPA_RAIL11V, true);
  }
}

/// @brief Executes the 7-stage glitch-free hardware initialization sequence for MCP23018.
/// @note Sets IOCON=0x21 (sequential paired layout, push-pull INTA), enables PB0/PB1 pull-ups.
void initMCP23018()
{
  mcp23018_present = false;
  mcp23018_portA_olata = 0x3C;
  mcp23018_portA_gppua = 0x03;

  // Stage 1: Hardware Reset Cycle (assert LOW for >= 2 ms, then release)
  pinMode(MCP23018_RESET_PIN, OUTPUT);
  digitalWrite(MCP23018_RESET_PIN, LOW);
  delay(MCP23018_RST_SLEEP_MS);
  digitalWrite(MCP23018_RESET_PIN, HIGH);
  delay(MCP23018_RST_RESUME_MS);

  // Stage 2: IOCON Configuration (BANK=0, SEQOP=1 address pointer disabled after each byte)
  if (!mcp23018WriteReg(MCP23018_IOCON, 0x21))
  {
    core0QueueOutputFlashText(F("MCP23018 ERROR: not found\r\n"));
    mcp23018_present = false;
    return;
  }

  // Stage 3: Enable explicit internal Pull-Ups for PB0 and PB1 (bits 0 and 1)
  if (!mcp23018WriteReg(MCP23018_GPPUA, mcp23018_portA_gppua))
  {
    core0QueueOutputFlashText(F("MCP23018 ERROR: unable to set GPPUA register\r\n"));
    mcp23018_present = false;
    return;
  }

  if (!mcp23018WriteReg(MCP23018_GPPUB, 0xFF))
  {
    core0QueueOutputFlashText(F("MCP23018 ERROR: unable to set GPPUB register\r\n"));
    mcp23018_present = false;
    return;
  }

  // Stage 4: Output Latch Setup (OLATA / OLATB) before switching direction to output
  if (!mcp23018WriteReg(MCP23018_OLATA, mcp23018_portA_olata))
  {
    core0QueueOutputFlashText(F("MCP23018 ERROR: unable to set OLATA register\r\n"));
    mcp23018_present = false;
    return;
  }

  if (!mcp23018WriteReg(MCP23018_OLATB, 0x00))
  {
    core0QueueOutputFlashText(F("MCP23018 ERROR: unable to set OLATB register\r\n"));
    mcp23018_present = false;
    return;
  }

  // Stage 5: Commit Port Directions (IODIRA: GPA0/GPA1 inputs, GPA2..GPA7 outputs)
  if (!mcp23018WriteReg(MCP23018_IODIRA, 0x03))
  {
    core0QueueOutputFlashText(F("MCP23018 ERROR: unable to set IODIRA register\r\n"));
    mcp23018_present = false;
    return;
  }

  if (!mcp23018WriteReg(MCP23018_IODIRB, 0xFF))
  {
    core0QueueOutputFlashText(F("MCP23018 ERROR: unable to set IODIRB register\r\n"));
    mcp23018_present = false;
    return;
  }

  // Stage 6: Interrupt Configuration for PB0/PB1 on Port A (Interrupt-on-change)
  if (!mcp23018WriteReg(MCP23018_DEFVALA, 0x00))
  {
    core0QueueOutputFlashText(F("MCP23018 ERROR: unable to set DEFVALA register\r\n"));
    mcp23018_present = false;
    return;
  }

  if (!mcp23018WriteReg(MCP23018_GPINTENA, 0x03))
  {
    core0QueueOutputFlashText(F("MCP23018 ERROR: unable to set GPINTENA register\r\n"));
    mcp23018_present = false;
    return;
  }

  if (!mcp23018WriteReg(MCP23018_GPINTENB, 0x00))
  {
    core0QueueOutputFlashText(F("MCP23018 ERROR: unable to set GPINTENB register\r\n"));
    mcp23018_present = false;
    return;
  }

  if (!mcp23018WriteReg(MCP23018_INTCONA, 0x00))
  {
    core0QueueOutputFlashText(F("MCP23018 ERROR: unable to set INTCONA register\r\n"));
    mcp23018_present = false;
    return;
  }

  if (!mcp23018WriteReg(MCP23018_INTCONB, 0x00))
  {
    core0QueueOutputFlashText(F("MCP23018 ERROR: unable to set INTCONB register\r\n"));
    mcp23018_present = false;
    return;
  }

  mcp23018_present = true;
  core0QueueOutputFlashText(F("MCP23018: OK\r\n"));
}

/// @brief Minimal ultra-fast GPIO ISR for MCP23018 INTA pin (GP28).
/// @note Latches mcp23018_PortAIntFlg in SRAM without performing I2C operations.
void __not_in_flash_func(mcp23018_PortA_ISR)()
{
  mcp23018_PortAIntFlg = true;
}

/// @brief Updates on-board RP2350 user LED to mirror debounced button states.
void updateBoardLedFromButtons()
{
  digitalWrite(LED_PIN, (debouncedPB0 || debouncedPB1) ? HIGH : LOW);
}

/// @brief Reads baseline Port A pin states, clears interrupt latches, and attaches INTA ISR.
void initMcp23018ButtonStateAndInterrupt()
{
  uint8_t startupGPIOA = mcp23018ReadReg(MCP23018_GPIOA);
  mcp23018ReadReg(MCP23018_INTCAPA);

  prevPortA = startupGPIOA;
  currPortA = startupGPIOA;

  debouncedPB0 = ((startupGPIOA & (1u << MCP_GPA_PB0)) == 0);
  debouncedPB1 = ((startupGPIOA & (1u << MCP_GPA_PB1)) == 0);

  mcp23018_PortAIntFlg = false;
  pbDebounceState = 0;

  pinMode(MCP23018_INTA_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(MCP23018_INTA_PIN), mcp23018_PortA_ISR, FALLING);
}

/// @brief 4-tick debounce state machine (~80 ms) for push buttons PB0 and PB1.
/// @note Called periodically by Core 0 within loop() every loopRateMsec (20 ms).
void processMcp23018ButtonsDebounce()
{
  if (!mcp23018_present) return;

  if (pbDebounceState == 0)
  {
    if (mcp23018_PortAIntFlg)
    {
      mcp23018_PortAIntFlg = false;
      prevPortA = mcp23018ReadReg(MCP23018_INTCAPA);

      debouncedPB0 = ((prevPortA & (1u << MCP_GPA_PB0)) == 0);
      debouncedPB1 = ((prevPortA & (1u << MCP_GPA_PB1)) == 0);
      updateBoardLedFromButtons();

      pbDebounceState = 1;
    }
  }
  else
  {
    if (mcp23018_PortAIntFlg)
    {
      pbDebounceState = 0;
    }
    else
    {
      currPortA = mcp23018ReadReg(MCP23018_GPIOA);

      if ((prevPortA & PB_MASK) == (currPortA & PB_MASK))
      {
        pbDebounceState++;
      }
      else
      {
        debouncedPB0 = ((currPortA & (1u << MCP_GPA_PB0)) == 0);
        debouncedPB1 = ((currPortA & (1u << MCP_GPA_PB1)) == 0);
        updateBoardLedFromButtons();

        prevPortA = currPortA;
        pbDebounceState = 1;
      }

      if (pbDebounceState >= PB_TICK_PERSISTENCE)
      {
        pbDebounceState = 0;
      }
    }
  }
}
