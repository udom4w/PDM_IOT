#pragma once
// ============================================================================
// [v16.6.5-fifo] fifo_codec.h
//
// L2 -- Frame Codec (CM-100_FIFO_DRIVER_SDS_v1.0 SS8, SS16; Implementation
// Plan Task 2.1).
//
// This file currently contains only the streaming CRC-16 component
// (Task 2.1). SampleDecoder (Task 2.2) and FrameCodec (Task 2.3) extend this
// same file in later tasks -- not present yet, by design; no placeholder
// content for them exists here.
//
// Zero Arduino/ESP32 dependency by design, matching fifo_transport.h and
// test/log_replay_transport.h -- compiles standalone under a plain C++
// compiler with no Arduino/ESP32 include paths.
// ============================================================================

#include <stdint.h>

// ----------------------------------------------------------------------------
// StreamingCrc16 -- CRC-16/MODBUS, computed incrementally, one byte at a
// time. No buffering: this is the mechanism that lets Phase 3 eliminate the
// 6144B staging buffer entirely (SDS D-11, PRR C-4/C-5).
//
// CRC variant (SDS K-6, K-7 -- the standard Modbus RTU CRC, independently
// confirmed against 12 real WTVB05 hardware captures, not merely assumed
// from the Modbus RTU convention):
//   Width:    16
//   Poly:     0xA001 (reflected form of the normal 0x8005 -- using it
//             directly in a right-shift update loop implements RefIn=true,
//             RefOut=true without a separate byte-reversal step)
//   Init:     0xFFFF
//   RefIn:    true
//   RefOut:   true
//   XorOut:   0x0000 (no final XOR -- Crc16_Final() is therefore a pure
//             pass-through of the accumulated state)
//   Check:    0x4B37, for the standard self-test string "123456789" (ASCII)
//             -- a property of the named variant itself, independent of
//             CM-100/WTVB05, and the first thing an implementation should
//             be checked against before any hardware-derived fixture.
//
// Coverage (SDS K-7 -- the CALLER's responsibility, not this component's):
// header bytes + payload; excludes the CRC pair itself. StreamingCrc16 has
// no knowledge of frame shape or byte range -- it accumulates exactly
// whatever bytes the caller feeds it, in order. Enforcing the correct byte
// range is entirely the caller's (Task 2.3 FrameCodec's) responsibility.
//
// Update algorithm, per Crc16_Update() call:
//   1. XOR the incoming byte into the low 8 bits of the running state.
//   2. Repeat 8 times: if the low bit is set, shift the state right one bit
//      and XOR with 0xA001; otherwise just shift the state right one bit.
// This is the standard reflected/right-shifting CRC-16/MODBUS update step.
//
// Streaming semantics: Crc16_Update() may be called once per byte as bytes
// arrive across many FifoDriver_Service() ticks, or in a tight loop over an
// already-fully-received buffer -- both produce byte-for-byte identical
// results, because "whole buffer" is nothing more than N sequential
// Crc16_Update() calls; there is no separate bulk-computation code path to
// diverge from the streaming one.
//
// Caller responsibilities:
//   - Call Crc16_Init() before the first Crc16_Update() of each new frame
//     attempt. This component has no way to detect a skipped or stale
//     Init() -- a caller that omits it silently accumulates from whatever
//     *state already held (SDS-wide convention: trust the documented
//     precondition, matching fifo_transport_uart485.cpp and
//     log_replay_transport.cpp).
//   - Feed exactly the bytes SDS K-7 specifies (header + payload, excluding
//     the CRC pair itself) -- no more, no fewer, in wire order.
//   - Compare Crc16_Final()'s result against the two transmitted CRC bytes
//     correctly: SDS K-6 states the CRC is transmitted low-byte-first, so
//     the wire pair reconstructs as (CRC_lo | (CRC_hi << 8)), not the
//     reverse -- this component returns a plain uint16_t and takes no
//     position on wire byte order; getting that comparison backwards at the
//     call site would be invisible to this file's own tests.
// ----------------------------------------------------------------------------

// Crc16_Init() -- reset `*state` to the algorithm's initial value (0xFFFF).
//   Precondition:  `state` is a valid, non-null pointer.
//   Postcondition: *state == 0xFFFF.
void Crc16_Init(uint16_t* state);

// Crc16_Update() -- fold one byte into the running CRC.
//   Precondition:  `state` has been initialized by Crc16_Init() for this
//                  frame attempt (or already holds a value from a prior
//                  Crc16_Update() call within the same attempt).
//   Postcondition: *state holds the CRC accumulated so far, including `b`.
void Crc16_Update(uint16_t* state, uint8_t b);

// Crc16_Final() -- finalize an accumulated CRC and return it.
//   Pure function: takes `state` BY VALUE, not by pointer -- it cannot and
//   does not modify whatever state the caller is still holding. Because
//   XorOut is 0x0000 for this variant, this is exactly an identity
//   pass-through; the function exists as a named, explicit step (matching
//   the frozen Task 2.1 interface) so a call site reads as "I am done
//   accumulating, give me the result," not as a decision to keep updating.
uint16_t Crc16_Final(uint16_t state);
