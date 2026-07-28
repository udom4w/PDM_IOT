// ============================================================================
// [v16.6.5-fifo] fifo_codec.cpp
//
// See fifo_codec.h for the public contract and full documentation (CRC
// variant, update algorithm, streaming semantics, caller responsibilities).
// This file currently implements only the streaming CRC-16 component
// (Task 2.1) -- SampleDecoder (Task 2.2) and FrameCodec (Task 2.3) are not
// present yet.
//
// No transport logic, no frame parsing, no state machine, no dynamic
// allocation -- this file is pure arithmetic over caller-supplied bytes.
// ============================================================================

#include "fifo_codec.h"

// ----------------------------------------------------------------------------
// CRC-16/MODBUS reflected polynomial (0xA001 = bit-reversal of the normal
// form 0x8005) and initial value. See fifo_codec.h for the full variant
// specification (RefIn=true, RefOut=true, XorOut=0x0000).
// ----------------------------------------------------------------------------
static const uint16_t CRC16_MODBUS_POLY = 0xA001;
static const uint16_t CRC16_MODBUS_INIT = 0xFFFF;

void Crc16_Init(uint16_t* state) {
  *state = CRC16_MODBUS_INIT;
}

void Crc16_Update(uint16_t* state, uint8_t b) {
  uint16_t crc = *state;
  crc = static_cast<uint16_t>(crc ^ static_cast<uint16_t>(b));
  for (int i = 0; i < 8; i++) {
    if (crc & 0x0001u) {
      crc = static_cast<uint16_t>((crc >> 1) ^ CRC16_MODBUS_POLY);
    } else {
      crc = static_cast<uint16_t>(crc >> 1);
    }
  }
  *state = crc;
}

uint16_t Crc16_Final(uint16_t state) {
  // XorOut = 0x0000 for CRC-16/MODBUS -- pure pass-through. `state` is a
  // by-value parameter: this function has no pointer to the caller's own
  // state and structurally cannot modify it.
  return state;
}
