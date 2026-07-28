// ============================================================================
// [v16.6.5-fifo] fifo_codec.cpp
//
// See fifo_codec.h for the public contract and full documentation (CRC
// variant/update algorithm/streaming semantics for StreamingCrc16; byte
// layout/big-endian decoding/precondition for SampleDecoder). This file
// implements the streaming CRC-16 component (Task 2.1) and SampleDecoder
// (Task 2.2) -- FrameCodec (Task 2.3) is not present yet.
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

// ----------------------------------------------------------------------------
// SampleDecoder_DecodeStride() -- see fifo_codec.h for the full contract
// (byte layout, big-endian decoding, signed int16 interpretation,
// precondition, composition with StreamingCrc16/FrameCodec).
//
// Malformed input is not reported here, deliberately: every 6-byte input
// decodes to some well-defined int16 triple -- there is no invalid bit
// pattern for a two's-complement signed value, so this function has no
// error to detect. Frame-level corruption (bad CRC, truncated stream,
// anchor desync, K-10's fill regression/overrun) is caught entirely
// upstream, by StreamingCrc16's mismatch and FrameCodec's anchor/type/
// progress logic -- per SDS D-8, `status` is the sole authority on
// validity, so samples decoded from a frame that later fails CRC are simply
// never exposed, regardless of what this function produced for them.
//
// Stateless by construction: no static/global variable, no member state,
// no data read or written beyond this call's own parameters. Each call is
// fully independent -- calling this function twice with the same 6 bytes
// always produces the same triple, and calling it for stride N has no
// effect on the result for stride N+1.
// ----------------------------------------------------------------------------
void SampleDecoder_DecodeStride(const uint8_t* sixBytes,
                                 int16_t* outX, int16_t* outY, int16_t* outZ) {
  uint16_t rawX = static_cast<uint16_t>((static_cast<uint16_t>(sixBytes[0]) << 8) |
                                          static_cast<uint16_t>(sixBytes[1]));
  uint16_t rawY = static_cast<uint16_t>((static_cast<uint16_t>(sixBytes[2]) << 8) |
                                          static_cast<uint16_t>(sixBytes[3]));
  uint16_t rawZ = static_cast<uint16_t>((static_cast<uint16_t>(sixBytes[4]) << 8) |
                                          static_cast<uint16_t>(sixBytes[5]));

  // Reinterpret each unsigned 16-bit bit pattern as a signed int16_t. This
  // narrowing conversion is implementation-defined (not undefined) by the
  // C++ standard for out-of-positive-range values; GCC -- the toolchain
  // this project targets -- documents this exact case as mapping to the
  // expected two's-complement value. See fifo_codec.h's doc comment.
  *outX = static_cast<int16_t>(rawX);
  *outY = static_cast<int16_t>(rawY);
  *outZ = static_cast<int16_t>(rawZ);
}
