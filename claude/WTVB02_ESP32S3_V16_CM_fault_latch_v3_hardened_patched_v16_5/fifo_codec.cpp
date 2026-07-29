// ============================================================================
// [v16.6.5-fifo] fifo_codec.cpp
//
// See fifo_codec.h for the public contract and full documentation (CRC
// variant/update algorithm/streaming semantics for StreamingCrc16; byte
// layout/big-endian decoding/precondition for SampleDecoder; ownership/
// state lifetime/caller responsibilities for FrameCodec). This file
// implements the streaming CRC-16 component (Task 2.1), SampleDecoder
// (Task 2.2), and FrameCodec's SCAN_ANCHOR, READ_TYPE, progress-frame
// parsing, full-dump payload/CRC-pair consumption, CRC accumulation over
// every covered byte, CRC verification, and full-dump sample decoding via
// SampleDecoder_DecodeStride() (Task 2.3, Phase 8).
//
// FrameCodec's timeout handling is NOT implemented here -- per Design
// Closure SS2, that is owned entirely by L3 (Task 3.4), never by this file.
// No dynamic allocation anywhere in this file -- StreamingCrc16 and
// SampleDecoder remain pure arithmetic over caller-supplied bytes.
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

// ----------------------------------------------------------------------------
// FrameCodecState_Reset() -- see fifo_codec.h for the full contract.
// ----------------------------------------------------------------------------
void FrameCodecState_Reset(FrameCodecState* s) {
  s->desyncBytesDiscarded = 0;
  s->partialBytesReceived = 0;
  for (int i = 0; i < 6; i++) {
    s->partialBuf[i] = 0;
  }
  s->anchorFound = false;
  s->anchorPartialMatch = false;
  s->typeByteRead = false;
  s->frameTypeByte = 0;
  Crc16_Init(&s->runningCrc);
}

// ----------------------------------------------------------------------------
// Anchor bytes (K-2: request/response frames begin `50 03`) and the bounded
// resync limit (SDS SS17.2 -- the PRR C-6 fix). Private to this file, like
// CRC16_MODBUS_POLY/INIT above; nothing outside FrameCodec needs to know
// this exact number, only the DESYNC_LIMIT outcome if it's reached.
// ----------------------------------------------------------------------------
static const uint8_t ANCHOR_BYTE_0 = 0x50;
static const uint8_t ANCHOR_BYTE_1 = 0x03;
static const uint32_t MAX_DESYNC_BYTES = 64;

// ----------------------------------------------------------------------------
// Type/discriminator byte values (K-5 -- this is a type discriminator, not
// a length, despite superficially resembling one). Private to this file,
// like the anchor bytes above.
// ----------------------------------------------------------------------------
static const uint8_t FRAME_TYPE_FULL_DUMP = 0x00;
static const uint8_t FRAME_TYPE_PROGRESS = 0x01;

// ----------------------------------------------------------------------------
// Length of the progress-shape tail (K-3: fill_hi, fill_lo, CRC_lo,
// CRC_hi -- the remainder of the 7-byte progress frame after the 2-byte
// anchor and 1-byte type already consumed by SCAN_ANCHOR/READ_TYPE).
// Private to this file, like the constants above.
// ----------------------------------------------------------------------------
static const uint32_t PROGRESS_TAIL_LEN = 4;

// ----------------------------------------------------------------------------
// Length of the full-dump payload (K-4: 6144 payload bytes) and the
// trailing CRC pair consumed after it (Phase 5.1 -- CRC_lo, CRC_hi, read
// but not verified). FULL_DUMP_TOTAL_LEN (6146) matches the "6146 B" exit
// condition already named for L3's S8 READ_DUMP state (SDS SS11.1),
// confirming this is the frame's true total remaining length after the
// 2-byte anchor and 1-byte type already consumed by SCAN_ANCHOR/READ_TYPE.
// Private to this file, like the constants above.
// ----------------------------------------------------------------------------
static const uint32_t FULL_DUMP_PAYLOAD_LEN = 6144;
static const uint32_t FULL_DUMP_CRC_LEN = 2;
static const uint32_t FULL_DUMP_TOTAL_LEN = FULL_DUMP_PAYLOAD_LEN + FULL_DUMP_CRC_LEN;

// ----------------------------------------------------------------------------
// Per-call byte budget (SDS SS7.2, A-3's contract: "consumes at most
// FIFO_SERVICE_MAX_BYTES = 512 bytes per call"). SCAN_ANCHOR and the
// progress tail never approach this on their own (bounded to 64 and 4
// respectively); full-dump payload accumulation is the first sub-phase
// whose natural size (6144) genuinely exceeds it, so it is the first one
// that needs this cap applied explicitly rather than getting one for free
// from a small fixed size.
// ----------------------------------------------------------------------------
static const uint32_t FIFO_SERVICE_MAX_BYTES = 512;

// ----------------------------------------------------------------------------
// ScanAnchor() -- consume bytes from `t` looking for `50 03`, one byte at a
// time, until the anchor is found, the transport has nothing more
// available, or MAX_DESYNC_BYTES worth of bytes have been discarded. See
// fifo_codec.h's FrameCodec_Step() doc comment for the full behavioral
// contract (overlap handling, never-consume-past-the-anchor, budget).
//
// Returns true if a terminal outcome was reached this call (DESYNC_LIMIT,
// written to *outOutcome); false otherwise (state->anchorFound may or may
// not have just become true -- the caller returns PENDING either way, per
// the doc comment's explanation of why there is no dedicated "anchor
// found" outcome value).
//
// CRC accumulation (Phase 6, K-7): the 2 anchor bytes are fed into
// state->runningCrc only at the instant the anchor is CONFIRMED (the byte
// immediately following a tentative 0x50 turns out to be 0x03) -- never
// for bytes discarded during resync. A byte is only known to be part of
// the real anchor in hindsight, once the next byte confirms it, so this
// cannot happen any earlier without risking accumulating garbage. The
// confirmed 0x50 is fed using the ANCHOR_BYTE_0 constant rather than a
// separately-stored byte value, since every candidate that ever reaches
// anchorPartialMatch is, by construction, that exact value -- there is
// nothing else it could have been.
// ----------------------------------------------------------------------------
static bool ScanAnchor(FifoTransport* t, FrameCodecState* state,
                        FifoFrameOutcome* outOutcome) {
  for (;;) {
    uint8_t b;
    size_t got = t->read(t->ctx, &b, 1);
    if (got == 0) {
      return false;  // nothing available right now -- caller returns PENDING
    }

    if (!state->anchorPartialMatch) {
      if (b == ANCHOR_BYTE_0) {
        // Tentative match start. NOT discarded -- remembered across calls
        // if nothing more arrives this call.
        state->anchorPartialMatch = true;
      } else {
        state->desyncBytesDiscarded++;
        if (state->desyncBytesDiscarded >= MAX_DESYNC_BYTES) {
          *outOutcome = FifoFrameOutcome::DESYNC_LIMIT;
          return true;
        }
      }
    } else {
      if (b == ANCHOR_BYTE_1) {
        // Anchor complete. Feed both confirmed anchor bytes into the
        // running CRC now that they're known to be real (Phase 6, K-7) --
        // never earlier, since a lone 0x50 might still turn out to be a
        // false start.
        Crc16_Update(&state->runningCrc, ANCHOR_BYTE_0);
        Crc16_Update(&state->runningCrc, b);  // b == ANCHOR_BYTE_1 here
        // Stop immediately -- do not consume past the anchor.
        state->anchorFound = true;
        state->anchorPartialMatch = false;
        return false;  // caller returns PENDING; no bytes read past here
      } else if (b == ANCHOR_BYTE_0) {
        // The PREVIOUS 0x50 was a false start -- discard exactly that one
        // byte. This new 0x50 is a fresh, live candidate: never discarded,
        // anchorPartialMatch stays true for it.
        state->desyncBytesDiscarded++;
        if (state->desyncBytesDiscarded >= MAX_DESYNC_BYTES) {
          *outOutcome = FifoFrameOutcome::DESYNC_LIMIT;
          return true;
        }
      } else {
        // The previous 0x50 was a false start, and this byte doesn't
        // start a new candidate either -- both are discarded, one at a
        // time, each checked against the bound independently.
        state->desyncBytesDiscarded++;
        if (state->desyncBytesDiscarded >= MAX_DESYNC_BYTES) {
          state->anchorPartialMatch = false;
          *outOutcome = FifoFrameOutcome::DESYNC_LIMIT;
          return true;
        }
        state->desyncBytesDiscarded++;
        state->anchorPartialMatch = false;
        if (state->desyncBytesDiscarded >= MAX_DESYNC_BYTES) {
          *outOutcome = FifoFrameOutcome::DESYNC_LIMIT;
          return true;
        }
      }
    }
  }
}

// ----------------------------------------------------------------------------
// ReadType() -- read exactly one byte and validate it as the K-5 type
// discriminator. Called only once anchorFound is true and typeByteRead is
// still false (i.e. never in the same call SCAN_ANCHOR completed on -- see
// FrameCodec_Step()). See fifo_codec.h's FrameCodec_Step() doc comment for
// the full behavioral contract.
//
// Reads AT MOST one byte, never loops -- unlike ScanAnchor(), a single
// byte is all this state ever needs, so there is no partial-match
// bookkeeping to persist across calls beyond typeByteRead/frameTypeByte
// themselves.
//
// CRC accumulation (Phase 6, K-7): the type byte is fed into
// state->runningCrc once validated as 0x00 or 0x01. A byte that fails
// validation is never fed in -- that attempt returns BAD_TYPE_BYTE and is
// abandoned regardless, so it would not matter either way, but the code
// only reaches the update after validation succeeds.
// ----------------------------------------------------------------------------
static FifoFrameOutcome ReadType(FifoTransport* t, FrameCodecState* state) {
  uint8_t b;
  size_t got = t->read(t->ctx, &b, 1);
  if (got == 0) {
    return FifoFrameOutcome::PENDING;  // nothing available yet
  }

  if (b == FRAME_TYPE_FULL_DUMP || b == FRAME_TYPE_PROGRESS) {
    Crc16_Update(&state->runningCrc, b);
    state->frameTypeByte = b;
    state->typeByteRead = true;
    // Transitions to the next state only -- progress/dump parsing is not
    // implemented yet, so there is nothing further to do this call.
    return FifoFrameOutcome::PENDING;
  }

  return FifoFrameOutcome::BAD_TYPE_BYTE;
}

// ----------------------------------------------------------------------------
// ReadProgress() -- consume the 4-byte progress-shape tail (K-3: fill_hi,
// fill_lo, CRC_lo, CRC_hi) and populate *outProgressFill once complete.
// Called only once anchorFound && typeByteRead && frameTypeByte ==
// FRAME_TYPE_PROGRESS. See fifo_codec.h's FrameCodec_Step() doc comment
// for the full behavioral contract.
//
// Reads as many bytes as are currently available, up to PROGRESS_TAIL_LEN
// total since entering this sub-phase -- never a 5th byte. state->partialBuf
// and state->partialBytesReceived (shared with full-dump payload
// accumulation, mutually exclusive with it -- see their doc comments in
// fifo_codec.h) carry a partial tail across calls exactly as
// state->anchorPartialMatch does for SCAN_ANCHOR.
//
// CRC accumulation (Phase 6, K-7, UNCHANGED in Phase 7): only fill_hi and
// fill_lo (indices 0 and 1 -- the first 2 of the 4-byte tail) are fed into
// state->runningCrc. The CRC bytes themselves (partialBuf[2], partialBuf[3],
// indices 2 and 3) are still read into the buffer -- as they already were
// in Phase 6, so the frame boundary is correct and nothing is left unread
// -- and are still explicitly excluded from Crc16_Update().
//
// CRC verification (Phase 7, K-6): once all 4 bytes are in hand, the
// transmitted CRC is reconstructed from partialBuf[2]/partialBuf[3] as
// (CRC_lo | (CRC_hi << 8)) -- SDS K-6's documented low-byte-first wire
// order -- and compared against Crc16_Final(state->runningCrc). On a
// mismatch, this function returns CRC_MISMATCH and *outProgressFill is
// left untouched (matching the existing "written only on a PROGRESS
// outcome" contract, fifo_codec.h); only on a match is *outProgressFill
// populated and PROGRESS returned. Every byte of the frame has already
// been consumed from the transport by the time this comparison runs,
// regardless of which way it comes out.
// ----------------------------------------------------------------------------
static FifoFrameOutcome ReadProgress(FifoTransport* t, FrameCodecState* state,
                                      uint16_t* outProgressFill) {
  while (state->partialBytesReceived < PROGRESS_TAIL_LEN) {
    uint8_t b;
    size_t got = t->read(t->ctx, &b, 1);
    if (got == 0) {
      return FifoFrameOutcome::PENDING;  // nothing available yet
    }
    if (state->partialBytesReceived < 2) {
      // fill_hi (index 0) / fill_lo (index 1): CRC-covered payload (K-7).
      // Indices 2/3 (CRC_lo/CRC_hi) are deliberately excluded below.
      Crc16_Update(&state->runningCrc, b);
    }
    state->partialBuf[state->partialBytesReceived] = b;
    state->partialBytesReceived++;
  }

  // All 4 bytes present. partialBuf[2..3] = CRC_lo/CRC_hi, transmitted
  // low-byte-first (K-6) -- reconstruct as (CRC_lo | (CRC_hi << 8)).
  uint16_t transmittedCrc = static_cast<uint16_t>(
      static_cast<uint16_t>(state->partialBuf[2]) |
      (static_cast<uint16_t>(state->partialBuf[3]) << 8));
  uint16_t computedCrc = Crc16_Final(state->runningCrc);
  if (computedCrc != transmittedCrc) {
    return FifoFrameOutcome::CRC_MISMATCH;
  }

  // partialBuf[0..1] = fill_hi/fill_lo (K-3), big-endian, matching
  // SampleDecoder's own byte-order convention.
  uint16_t fill = static_cast<uint16_t>(
      (static_cast<uint16_t>(state->partialBuf[0]) << 8) |
      static_cast<uint16_t>(state->partialBuf[1]));
  *outProgressFill = fill;
  return FifoFrameOutcome::PROGRESS;
}

// ----------------------------------------------------------------------------
// ReadDump() -- consume the full-dump frame's payload AND trailing CRC pair
// (K-4: 6144 payload bytes + CRC_lo + CRC_hi = 6146 bytes total), so that
// FULL_DUMP correctly means "the entire frame has been consumed," not just
// "payload complete" (Task 2.3 Phase 5.1 -- corrects Phase 5's narrower
// completion semantics). Called only once anchorFound && typeByteRead &&
// frameTypeByte == FRAME_TYPE_FULL_DUMP. See fifo_codec.h's
// FrameCodec_Step() doc comment for the full behavioral contract.
//
// Two sub-states, both driven by state->partialBytesReceived, never in the
// same call as each other (matching every other phase boundary in this
// file):
//
//   1. Payload (state->partialBytesReceived: 0..FULL_DUMP_PAYLOAD_LEN):
//      accumulates 6 bytes at a time into state->partialBuf via
//      partialBytesReceived % 6, reusing the same buffer for every stride
//      -- no second "which stride" counter is introduced. Bounded per call
//      by FIFO_SERVICE_MAX_BYTES, since 6144 genuinely exceeds the
//      per-call service budget (unlike every other sub-phase in this
//      file), unchanged from Phase 6/7. The instant the payload completes,
//      this function returns PENDING for that call WITHOUT attempting the
//      trailing CRC pair in the same call. As of THIS PHASE, the instant
//      each 6-byte stride completes (partialBytesReceived becomes a
//      multiple of 6), SampleDecoder_DecodeStride() is called exactly
//      once for it, writing directly through xOut[idx]/yOut[idx]/zOut[idx]
//      (idx = partialBytesReceived/6 - 1) -- see below.
//   2. Trailing CRC pair (state->partialBytesReceived:
//      FULL_DUMP_PAYLOAD_LEN..FULL_DUMP_TOTAL_LEN, only once sub-state 1
//      has already completed on a prior call): reads exactly
//      FULL_DUMP_CRC_LEN (2) more bytes. Neither byte is accumulated into
//      state->runningCrc (K-7 excludes the CRC pair itself). Naturally
//      bounded to at most 2 iterations, needing no separate budget check
//      (same reasoning as READ_TYPE's single byte and the 4-byte progress
//      tail).
//
// CRC accumulation (Phase 6, K-7, UNCHANGED in Phase 7): every one of the
// 6144 payload bytes is fed into state->runningCrc as it's read, inside
// sub-state 1's loop only -- sub-state 2's loop still never touches
// state->runningCrc, which is what correctly excludes the trailing CRC
// pair from the running CRC.
//
// CRC verification (Phase 7, K-6): sub-state 2 now STORES its two bytes
// into partialBuf[0]/partialBuf[1] (index = partialBytesReceived -
// FULL_DUMP_PAYLOAD_LEN, i.e. 0 then 1) instead of discarding them --
// reusing the same buffer sub-state 1 used for the last payload stride,
// which is safe because nothing reads that stride's raw bytes once
// sub-state 1 has already returned to the caller (sample decoding reads
// each stride during sub-state 1 itself, not after -- see below).
// Once both CRC bytes are in hand, the transmitted CRC is reconstructed
// as (CRC_lo | (CRC_hi << 8)) -- SDS K-6's low-byte-first wire order --
// and compared against Crc16_Final(state->runningCrc). On a mismatch,
// this function returns CRC_MISMATCH; only on a match does it return
// FULL_DUMP. Every byte of the frame has already been consumed from the
// transport by the time this comparison runs, regardless of which way it
// comes out.
//
// Sample decoding (Phase 8, K-8): SampleDecoder_DecodeStride() is called
// exactly once per completed 6-byte stride, immediately, from inside
// sub-state 1's loop -- never batched, never deferred. FrameCodec buffers
// no decoded values itself and introduces no additional payload storage:
// the same state->partialBuf[6] used since Phase 5 is passed directly to
// SampleDecoder_DecodeStride() as its `sixBytes` argument, and results are
// written straight through the caller-supplied xOut/yOut/zOut base
// pointers at the current stride index -- there is nothing else to
// buffer. A completed stride sitting in partialBuf is overwritten by the
// next one (or, for the very last stride, by the trailing CRC pair) only
// AFTER it has already been decoded and its results written out, so
// nothing is lost.
// ----------------------------------------------------------------------------
static FifoFrameOutcome ReadDump(FifoTransport* t, FrameCodecState* state,
                                  int16_t* xOut, int16_t* yOut, int16_t* zOut) {
  if (state->partialBytesReceived < FULL_DUMP_PAYLOAD_LEN) {
    uint32_t consumedThisCall = 0;
    while (state->partialBytesReceived < FULL_DUMP_PAYLOAD_LEN &&
           consumedThisCall < FIFO_SERVICE_MAX_BYTES) {
      uint8_t b;
      size_t got = t->read(t->ctx, &b, 1);
      if (got == 0) {
        return FifoFrameOutcome::PENDING;  // nothing available right now
      }
      Crc16_Update(&state->runningCrc, b);  // payload byte: CRC-covered (K-7)
      state->partialBuf[state->partialBytesReceived % 6] = b;
      state->partialBytesReceived++;
      consumedThisCall++;
      if (state->partialBytesReceived % 6 == 0) {
        // A full stride just completed -- decode it now, exactly once,
        // directly from partialBuf, before it can be overwritten by the
        // next stride's bytes.
        uint32_t strideIndex = state->partialBytesReceived / 6 - 1;
        SampleDecoder_DecodeStride(state->partialBuf,
                                    &xOut[strideIndex],
                                    &yOut[strideIndex],
                                    &zOut[strideIndex]);
      }
    }
    // Either hit the per-call budget mid-payload, or the payload just
    // completed this call -- either way, the trailing CRC pair is deferred
    // to the next call.
    return FifoFrameOutcome::PENDING;
  }

  // Trailing CRC pair: state->partialBytesReceived is already >=
  // FULL_DUMP_PAYLOAD_LEN on entry here, i.e. only once payload
  // accumulation has already completed on a prior call.
  while (state->partialBytesReceived < FULL_DUMP_TOTAL_LEN) {
    uint8_t b;
    size_t got = t->read(t->ctx, &b, 1);
    if (got == 0) {
      return FifoFrameOutcome::PENDING;  // nothing available right now
    }
    // Store (not accumulate) -- index 0 then 1 within the CRC pair.
    state->partialBuf[state->partialBytesReceived - FULL_DUMP_PAYLOAD_LEN] = b;
    state->partialBytesReceived++;
  }

  // Both CRC bytes present at partialBuf[0..1], transmitted low-byte-first
  // (K-6) -- reconstruct as (CRC_lo | (CRC_hi << 8)).
  uint16_t transmittedCrc = static_cast<uint16_t>(
      static_cast<uint16_t>(state->partialBuf[0]) |
      (static_cast<uint16_t>(state->partialBuf[1]) << 8));
  uint16_t computedCrc = Crc16_Final(state->runningCrc);
  if (computedCrc != transmittedCrc) {
    return FifoFrameOutcome::CRC_MISMATCH;
  }
  return FifoFrameOutcome::FULL_DUMP;
}

// ----------------------------------------------------------------------------
// FrameCodec_Step() -- PHASE 8: SCAN_ANCHOR + READ_TYPE + progress-frame
// parsing + full-dump payload/CRC-pair consumption + CRC accumulation +
// CRC verification, with full-dump sample decoding now wired in:
// ReadDump() calls SampleDecoder_DecodeStride() once per completed stride
// and writes through `xOut`/`yOut`/`zOut`, which this function now passes
// through instead of discarding. Progress-frame parsing is unchanged and
// never touches `xOut`/`yOut`/`zOut` (no sample payload exists in that
// shape). SCAN_ANCHOR, READ_TYPE, CRC accumulation, CRC verification, and
// every byte-consumption boundary are unchanged from Phase 7. See
// fifo_codec.h for the full contract, ownership model, and caller
// responsibilities.
// ----------------------------------------------------------------------------
FifoFrameOutcome FrameCodec_Step(FifoTransport* t, FrameCodecState* state,
                                  uint16_t* outProgressFill,
                                  int16_t* xOut, int16_t* yOut, int16_t* zOut) {
  if (!state->anchorFound) {
    FifoFrameOutcome outcome;
    if (ScanAnchor(t, state, &outcome)) {
      return outcome;  // DESYNC_LIMIT
    }
    // Either still scanning, or the anchor was just found this call --
    // either way, READ_TYPE is deferred to the next call. Phase 2's
    // behavior is preserved exactly: this function never attempts to read
    // the type byte in the same call the anchor completed in.
    return FifoFrameOutcome::PENDING;
  }

  if (!state->typeByteRead) {
    return ReadType(t, state);
  }

  if (state->frameTypeByte == FRAME_TYPE_PROGRESS) {
    return ReadProgress(t, state, outProgressFill);
  }

  // state->frameTypeByte == FRAME_TYPE_FULL_DUMP (the only other value
  // READ_TYPE ever stores).
  return ReadDump(t, state, xOut, yOut, zOut);
}
