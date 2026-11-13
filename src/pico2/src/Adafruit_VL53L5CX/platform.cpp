/*******************************************************************************
 * Copyright (c) 2020, STMicroelectronics - All Rights Reserved
 *
 * This file is part of the VL53L5CX Ultra Lite Driver and is dual licensed,
 * either 'STMicroelectronics Proprietary license'
 * or 'BSD 3-clause "New" or "Revised" License' , at your option.
 *
 * Modified for Adafruit BusIO by Limor 'ladyada' Fried with assistance from
 * Claude Code
 *
 *******************************************************************************/

/*
 * RP2350 / Raspberry Pi Pico 2 note
 *
 * During debug on Raspberry Pi Pico 2 / RP2350 with the Earle Philhower core,
 * occasional I2C write sensitivity was observed during the early VL53L5CX boot
 * sequence, especially around register 0x000F.
 *
 * This platform layer keeps small retry loops on byte read/write operations.
 *
 * The actual confirmed compatibility fix for the 0x000F 0x43 -> 0x40 boot
 * transition is implemented in vl53l5cx_api.cpp, inside vl53l5cx_init().
 */

#include "platform.h"

#include <Arduino.h>

uint8_t RdByte(VL53L5CX_Platform *p_platform,
               uint16_t RegisterAddress,
               uint8_t *p_value) {
  uint8_t reg[2];

  reg[0] = (RegisterAddress >> 8) & 0xFF;
  reg[1] = RegisterAddress & 0xFF;

  bool ok = false;

  for (uint8_t attempts = 1; attempts <= 10; attempts++) {
    ok = p_platform->i2c_dev->write_then_read(reg, 2, p_value, 1);

    if (ok) {
      break;
    }

    delay(2);
  }

  return ok ? 0 : 1;
}

uint8_t WrByte(VL53L5CX_Platform *p_platform,
               uint16_t RegisterAddress,
               uint8_t value) {
  uint8_t buffer[3];

  buffer[0] = (RegisterAddress >> 8) & 0xFF;
  buffer[1] = RegisterAddress & 0xFF;
  buffer[2] = value;

  bool ok = false;

  for (uint8_t attempts = 1; attempts <= 10; attempts++) {
    ok = p_platform->i2c_dev->write(buffer, 3);

    if (ok) {
      break;
    }

    delay(2);
  }

  return ok ? 0 : 1;
}

uint8_t RdMulti(VL53L5CX_Platform *p_platform,
                uint16_t RegisterAddress,
                uint8_t *p_values,
                uint32_t size) {
  uint8_t reg[2];
  uint32_t bytesRemaining = size;
  uint32_t offset = 0;

  uint32_t maxRead = p_platform->i2c_dev->maxBufferSize();

  if (maxRead == 0) {
    return 1;
  }

  while (bytesRemaining > 0) {
    uint32_t toRead = bytesRemaining;

    if (toRead > maxRead) {
      toRead = maxRead;
    }

    uint16_t currentRegister = RegisterAddress + offset;

    reg[0] = (currentRegister >> 8) & 0xFF;
    reg[1] = currentRegister & 0xFF;

    bool ok = p_platform->i2c_dev->write_then_read(reg, 2,
                                                   p_values + offset,
                                                   toRead);

    if (!ok) {
      return 1;
    }

    offset += toRead;
    bytesRemaining -= toRead;
  }

  return 0;
}

uint8_t WrMulti(VL53L5CX_Platform *p_platform,
                uint16_t RegisterAddress,
                uint8_t *p_values,
                uint32_t size) {
  uint32_t bytesRemaining = size;
  uint32_t offset = 0;

  uint32_t busMax = p_platform->i2c_dev->maxBufferSize();

  if (busMax <= 2) {
    return 1;
  }

  uint32_t maxPayload = busMax - 2;

  /*
   * Use a fixed-size local buffer instead of a variable-length array.
   * 34 bytes covers the common 32-byte I2C buffer plus the 2-byte register
   * address. If BusIO reports a smaller buffer, maxPayload is reduced below.
   */
  uint8_t buffer[34];

  uint32_t bufferPayload = sizeof(buffer) - 2;

  if (maxPayload > bufferPayload) {
    maxPayload = bufferPayload;
  }

  if (maxPayload == 0) {
    return 1;
  }

  while (bytesRemaining > 0) {
    uint32_t toWrite = bytesRemaining;

    if (toWrite > maxPayload) {
      toWrite = maxPayload;
    }

    uint16_t currentRegister = RegisterAddress + offset;

    buffer[0] = (currentRegister >> 8) & 0xFF;
    buffer[1] = currentRegister & 0xFF;

    memcpy(buffer + 2, p_values + offset, toWrite);

    bool ok = p_platform->i2c_dev->write(buffer, toWrite + 2);

    if (!ok) {
      return 1;
    }

    offset += toWrite;
    bytesRemaining -= toWrite;
  }

  return 0;
}

void SwapBuffer(uint8_t *buffer, uint16_t size) {
  uint32_t i, tmp;

  for (i = 0; i < size; i = i + 4) {
    tmp = (buffer[i] << 24) |
          (buffer[i + 1] << 16) |
          (buffer[i + 2] << 8) |
          (buffer[i + 3]);

    memcpy(&(buffer[i]), &tmp, 4);
  }
}

uint8_t WaitMs(VL53L5CX_Platform *p_platform, uint32_t TimeMs) {
  (void)p_platform;
  delay(TimeMs);
  return 0;
}




