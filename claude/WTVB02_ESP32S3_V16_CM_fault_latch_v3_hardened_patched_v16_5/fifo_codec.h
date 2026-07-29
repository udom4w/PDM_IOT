#pragma once
// ============================================================================
// [v16.6.5-fifo] fifo_codec.h
//
// L2 -- Frame Codec (CM-100_FIFO_DRIVER_SDS_v1.0 SS8, SS16; Implementation
// Plan Tasks 2.1, 2.2, 2.3).
//
// This file contains the streaming CRC-16 component (Task 2.1),
// SampleDecoder (Task 2.2), and FrameCodec (Task 2.3, Phase 8: anchor
// scan, type parsing, progress-frame parsing, full-dump payload/CRC-pair
// consumption, CRC accumulation over every covered byte, CRC
// verification, and full-dump sample decoding via
// SampleDecoder_DecodeStride()). Timeout handling is NOT implemented here
// -- per Design Closure SS2, that is owned entirely by L3. See
// fifo_codec.cpp for the exact phase boundary.
//
// Zero Arduino/ESP32 dependency by design, matching fifo_transport.h and
// test/log_replay_transport.h -- compiles standalone under a plain C++
// compiler with no Arduino/ESP32 include paths. FrameCodec is the one
// exception to "stdint.h only": it references FifoTransport (L1), so this
// file also includes fifo_transport.h -- still zero Arduino/ESP32
// dependency, since fifo_transport.h itself has none.
// ============================================================================

#include <stdint.h>
#include "fifo_transport.h"

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

// ----------------------------------------------------------------------------
// [v16.6.6-fifo] SampleDecoder -- decode one already-assembled 6-byte
// RAWFIFO payload stride into one raw int16 tri-axial sample triple
// (SDS K-8, D-9; Implementation Plan Task 2.2).
//
// Byte layout (SDS K-8: 1024 samples x 3 axes x int16, big-endian, stride
// 6B, order X,Y,Z -- verified against the fixture corpus, not re-derived
// from the datasheet alone):
//   sixBytes[0..1] -> X, big-endian signed int16
//   sixBytes[2..3] -> Y, big-endian signed int16
//   sixBytes[4..5] -> Z, big-endian signed int16
//
// Big-endian decoding: each axis's two bytes are combined as
// (highByte << 8) | lowByte, i.e. sixBytes[2*axis] is the MOST significant
// byte -- the reverse of the little-endian byte order native to this
// platform.
//
// Signed int16 interpretation: the combined 16-bit value is a two's-
// complement signed count, not an unsigned magnitude. The implementation
// assembles the bit pattern as an unsigned uint16_t first, then reinterprets
// it as int16_t via a narrowing cast. That narrowing conversion is
// implementation-defined (not undefined) by the C++ standard for values
// that don't fit int16_t's positive range, and GCC -- the toolchain this
// project targets (xtensa-esp32s3-elf-g++) -- explicitly documents this
// case as mapping to the expected two's-complement value, not an
// unspecified one. This is a deliberate reliance on that documented GCC
// guarantee, not an oversight.
//
// D-9: this function never applies K-9's g-scale factor. Samples are raw
// counts only -- scaling is a consumer decision, made where K-9's "evidence-
// backed, not confirmed" caveat (RFC-0007 SS4) remains visible.
//
// Pure, stateless: SampleDecoder holds no state of its own, unlike
// StreamingCrc16's Init/Update/Final split. It does not accumulate partial
// bytes across calls -- deciding "when do I have 6 complete bytes" and
// holding a partial-stride buffer while waiting is FrameCodec's (Task 2.3)
// responsibility, not this component's. Each call is fully independent of
// every other call.
//
// Precondition: `sixBytes` must point to exactly one correctly aligned,
// already-fully-received 6-byte RAWFIFO sample stride, in wire order. This
// function has no way to detect a misaligned or partial window -- every
// 6-byte input decodes to *some* well-defined int16 triple; there is no
// invalid bit pattern. Stride-alignment correctness is entirely the
// caller's responsibility (see fifo_codec.cpp's doc comment for why this
// component cannot and does not report malformed input).
//
// Composition:
//   - With StreamingCrc16 (Task 2.1): sibling, not caller/callee. FrameCodec
//     feeds each arriving byte to Crc16_Update() for the running CRC, and
//     separately calls SampleDecoder_DecodeStride() once a stride is
//     complete -- two independent uses of the same bytes, composed by
//     FrameCodec, never by either of these components directly.
//   - With FrameCodec (Task 2.3; sample decoding not yet wired in as of
//     Phase 6): FrameCodec owns the partial-stride buffer and will call
//     this function once per complete stride, passing write pointers that
//     ultimately trace back to
//     FifoArena_WriteHandleX/Y/Z() (advanced per sample index by L3).
//     SampleDecoder never sees an array, an index, or FifoArena -- only the
//     one triple of pointers for this call (SDS D-7/Freeze Review 1.1's
//     "L2 has zero dependency on any L3 type" discipline, applied here too).
// ----------------------------------------------------------------------------

// SampleDecoder_DecodeStride() -- decode one 6-byte stride into (x, y, z).
//   Precondition: `sixBytes` points to exactly 6 valid, in-order bytes.
//                 `outX`, `outY`, `outZ` are valid, non-null, caller-owned
//                 pointers.
//   Postcondition: *outX, *outY, *outZ hold the decoded raw int16 counts.
//                  No other state is read or written.
void SampleDecoder_DecodeStride(const uint8_t* sixBytes,
                                 int16_t* outX, int16_t* outY, int16_t* outZ);

// ----------------------------------------------------------------------------
// [v16.6.7-fifo] FrameCodec -- PHASE 8: SAMPLEDECODER INTEGRATION.
//
// The core L2 protocol component: recognizes the RAWFIFO `50 03` anchor
// with a bounded resync (PRR C-6), discriminates progress vs full-dump
// shape (K-5), streams payload through StreamingCrc16 and SampleDecoder,
// and verifies CRC before reporting a terminal outcome (SDS SS8, SS16;
// Implementation Plan Task 2.3; Task 2.3 Design Review + Design Closure).
//
// Phase 1 added the approved types, FrameCodecState_Reset(), and
// FrameCodec_Step()'s function structure (always PENDING). Phase 2 added
// SCAN_ANCHOR. Phase 3 added READ_TYPE. Phase 4 added progress-frame
// parsing. Phase 5 added full-dump payload accumulation; Phase 5.1
// corrected FULL_DUMP to mean "the entire 6149-byte frame consumed."
// Phase 6 fed every CRC-covered byte into state->runningCrc via
// Crc16_Update() as it was consumed, without ever checking the result.
// Phase 7 added the check: once a frame's transmitted CRC bytes have been
// consumed (both shapes), the accumulated CRC (via Crc16_Final()) is
// compared against the transmitted value, reconstructed low-byte-first
// per K-6. On a match, PROGRESS/FULL_DUMP is returned exactly as before.
// On a mismatch, CRC_MISMATCH is returned instead -- and *outProgressFill*
// is left untouched on that path, preserving its "written only on a
// PROGRESS outcome" contract. Every byte of the frame has already been
// consumed from the transport by the time this comparison runs, on either
// outcome -- verification never leaves a byte unread.
// THIS PHASE wires in sample decoding for full-dump frames only:
// ReadDump() calls SampleDecoder_DecodeStride() exactly once per completed
// 6-byte payload stride, immediately, from inside its payload-accumulation
// loop, writing results straight through the caller-supplied xOut/yOut/
// zOut arrays at that stride's index. FrameCodec buffers no decoded
// samples itself -- state->partialBuf[6] (already reused since Phase 5) is
// passed directly to SampleDecoder_DecodeStride() as its input, and is
// only overwritten by the next stride (or the trailing CRC pair) after
// that call has already returned. Progress frames never call
// SampleDecoder_DecodeStride() -- no sample payload exists in that shape.
// No CRC accumulation call, CRC verification, parser state transition, or
// byte-consumption boundary changed from Phase 7.
// Still not implemented: timeout handling -- per Design Closure SS2, that
// is owned entirely by L3 (Task 3.4), never by FrameCodec.
//
// Ownership: `FrameCodecState` is caller-owned (SDS SS11.4, Freeze Review
// 3.1) -- FrameCodec never allocates, owns no static/global copy of it, and
// holds no state of its own between calls beyond what's inside the struct
// the caller passes by pointer. `xOut`/`yOut`/`zOut` are caller-owned
// destination arrays FrameCodec only ever writes through, never owns.
//
// State lifetime: one `FrameCodecState` instance is scoped to exactly one
// frame attempt. `FrameCodecState_Reset()` must be called once at the start
// of each new attempt (SDS SS11.4) -- the same instance is then threaded,
// unmodified in identity, through every `FrameCodec_Step()` call for that
// attempt until a terminal (non-PENDING) outcome is returned. A fresh
// attempt (e.g. a retry after CRC_MISMATCH) requires a fresh Reset() --
// reusing a post-terminal state without resetting it is undefined from
// this component's perspective (trust-the-precondition convention, matching
// every other component in this file).
//
// Caller responsibilities:
//   - Call FrameCodecState_Reset() exactly once per new frame attempt,
//     before the first FrameCodec_Step() call for that attempt.
//   - Call FrameCodec_Step() repeatedly (intended: once per
//     FifoDriver_Service() tick) until it returns something other than
//     PENDING. PENDING means "no terminal outcome yet, call again" -- it is
//     not an error and requires no special handling beyond continuing.
//   - Timeout detection is NOT this component's job (Design Closure SS2):
//     if the caller decides too much wall-clock time has passed, it stops
//     calling FrameCodec_Step() for this attempt and finalizes the result
//     itself using ERR_NO_RESPONSE/ERR_INTER_BYTE_TIMEOUT (SDS SS16.1) --
//     FrameCodec_Step() never calls nowMs() and never produces
//     TIMEOUT_NO_RESPONSE/TIMEOUT_INTER_BYTE internally. Those two
//     FifoFrameOutcome values exist for the caller's own use, for taxonomy
//     consistency with FrameCodec's other outcomes -- not values this
//     function's body returns.
//
// Relationship with StreamingCrc16 (Task 2.1): sibling, not caller/callee
// from the outside -- but FrameCodec is StreamingCrc16's only caller.
// `state.runningCrc` (below) is StreamingCrc16's accumulator, reset via
// Crc16_Init() inside FrameCodecState_Reset() and folded via Crc16_Update()
// once per CRC-covered byte (K-7: header + payload, excluding the CRC
// pair) at every point a covered byte is consumed: ScanAnchor()'s
// confirmation branch, ReadType()'s valid-byte branch, ReadProgress()'s
// fill_hi/fill_lo bytes, and ReadDump()'s payload loop. The transmitted
// CRC bytes themselves are never fed to Crc16_Update() by either
// ReadProgress() or ReadDump() -- but as of Phase 7, both functions now
// call Crc16_Final() once their respective frame's CRC bytes are in hand,
// to compare against the transmitted value (SDS K-6).
//
// Relationship with SampleDecoder (Task 2.2): FrameCodec is SampleDecoder's
// only caller. As of Phase 8, SampleDecoder_DecodeStride() is called once
// per completed 6-byte payload stride during dump parsing only, writing
// into the caller-supplied xOut/yOut/zOut arrays at an index derived from
// `partialBytesReceived` -- never during progress parsing (no sample
// payload exists in that shape).
// ----------------------------------------------------------------------------

// FifoFrameOutcome -- the outcome of one FrameCodec_Step() call.
//   PENDING is returned when no terminal outcome was reached this call --
//   the normal, expected result while a multi-call frame attempt is still
//   in progress (Task 2.3 Design Closure SS1: required because a single
//   bounded, non-blocking call cannot guarantee finishing a 6146-byte dump,
//   and this non-void return type must hold some value every call).
//   TIMEOUT_NO_RESPONSE/TIMEOUT_INTER_BYTE are never produced by
//   FrameCodec_Step() itself -- see "Caller responsibilities" above.
enum class FifoFrameOutcome {
  PENDING,
  PROGRESS,
  FULL_DUMP,
  DESYNC_LIMIT,
  CRC_MISMATCH,
  BAD_TYPE_BYTE,
  TIMEOUT_NO_RESPONSE,
  TIMEOUT_INTER_BYTE
};

// FrameCodecState -- explicit, caller-owned, per-frame-attempt scan/parse
// state. No member here is hidden or static (SDS D-6, SS11.4).
struct FrameCodecState {
  // [Task 2.3, frozen] Count of bytes discarded while scanning for the
  // `50 03` anchor. Bounds the resync against MAX_DESYNC_BYTES (64,
  // SDS SS17.2) -- the fix for PRR C-6's unbounded anchor resync.
  uint32_t desyncBytesDiscarded;

  // [Task 2.3, frozen] Count of bytes consumed within the current
  // byte-accumulation sub-phase. NOT a cumulative count since the anchor --
  // READ_TYPE consumes its one byte without touching this field, since a
  // single byte needs no partial-accumulation bookkeeping (see
  // typeByteRead instead). Meaning depends on which sub-phase is active
  // (mutually exclusive -- a given frame attempt is either progress or
  // dump, never both):
  //   Progress tail (Phase 4): 0..4, indexes partialBuf[0..3] directly
  //   (fill_hi/fill_lo/CRC_lo/CRC_hi, K-3) and is never reduced modulo
  //   anything, since the tail is read exactly once per attempt.
  //   Full-dump payload (Phases 5/5.1): 0..6144 while accumulating the
  //   payload -- partialBuf is indexed via `partialBytesReceived % 6`, so
  //   the same 6-byte buffer is reused for every stride in turn rather
  //   than needing 1024 separate buffers. As of Phase 8, this is also what
  //   derives the current sample index for xOut/yOut/zOut
  //   (`partialBytesReceived / 6 - 1` at the instant a stride completes).
  //   6144..6146 while consuming the frame's trailing
  //   CRC_lo/CRC_hi -- partialBuf[0..1] is indexed via
  //   `partialBytesReceived - 6144` (see ReadDump()'s doc comment,
  //   fifo_codec.cpp): as of Phase 7 the two CRC bytes are STORED there
  //   (for CRC verification), reusing the last payload stride's slot,
  //   which is safe since nothing reads that stride's raw bytes once the
  //   payload sub-state has already ended.
  uint32_t partialBytesReceived;

  // [Task 2.3, frozen] In-progress byte unit -- the largest partial unit
  // this codec ever assembles mid-call (the 4-byte progress tail, using
  // indices 0..3, or a 6-byte payload stride, using indices 0..5 via
  // partialBytesReceived % 6 -- see partialBytesReceived above). Carries a
  // partially-received unit across calls when a call ends before it's
  // complete, and is reused/overwritten for every subsequent stride during
  // full-dump payload accumulation -- this codec never buffers more than
  // one stride at a time (SDS D-11: streaming, no staging buffer).
  uint8_t partialBuf[6];

  // [Design Closure SS1, approved addition] True once the `50 03` anchor
  // has been matched. Without this, partialBytesReceived == 0 is ambiguous
  // between "still scanning for the anchor" and "anchor just found, zero
  // type/payload bytes read yet" -- two genuinely different phases.
  bool anchorFound;

  // [Phase 2, necessary addition] True if the most recently consumed byte,
  // while anchorFound is still false, was an unconfirmed 0x50 that has not
  // yet been followed by a 0x03 -- i.e. scanning is currently mid-way
  // through a possible anchor match. This is what makes single-byte
  // overlap handling correct across calls: a call may end immediately
  // after consuming a lone 0x50 with no further bytes available yet, and
  // the NEXT call must still treat that 0x50 as a live candidate rather
  // than having silently discarded it. Meaningless once anchorFound is
  // true; SCAN_ANCHOR is the only phase that reads or writes it.
  bool anchorPartialMatch;

  // [Phase 3, necessary addition] True once the single type/discriminator
  // byte immediately following the anchor (K-5) has been read and
  // validated. Needed for the same reason anchorFound is: without it,
  // there is no way to tell "anchor found, type not read yet" apart from
  // "type already read" across calls, and READ_TYPE must not re-read or
  // re-validate a byte it already consumed.
  bool typeByteRead;

  // [Phase 3, necessary addition] The raw type/discriminator byte (K-5:
  // 0x00 = full dump, 0x01 = progress), valid only once typeByteRead is
  // true. Persisted here so later phases (progress/dump parsing, not yet
  // implemented) know which shape to parse without re-reading the byte --
  // READ_TYPE reads it exactly once, ever, per frame attempt.
  uint8_t frameTypeByte;

  // [Design Closure SS1, approved addition] The in-flight StreamingCrc16
  // accumulator for this frame attempt. Lives here, alongside the rest of
  // the per-attempt scan state, so it survives correctly across repeated
  // FrameCodec_Step() calls exactly like partialBytesReceived does --
  // rather than being invented as a second, separately-threaded piece of
  // caller state.
  uint16_t runningCrc;
};

// FrameCodecState_Reset() -- initialize `*s` for a new frame attempt.
//   Precondition:  `s` is a valid, non-null pointer.
//   Postcondition: all scan/parse fields are zeroed, anchorFound,
//                  anchorPartialMatch, and typeByteRead are false,
//                  frameTypeByte is zeroed (meaningless until
//                  typeByteRead), and runningCrc is initialized via
//                  Crc16_Init() (SDS K-6: 0xFFFF) -- *s is ready for the
//                  first FrameCodec_Step() call of a new attempt.
void FrameCodecState_Reset(FrameCodecState* s);

// FrameCodec_Step() -- advance one frame attempt by whatever bytes are
// currently available, bounded and non-blocking (PRR C-1).
//
//   PHASE 8: implements SCAN_ANCHOR (Phase 2), READ_TYPE (Phase 3),
//   progress-frame parsing (Phase 4), full-dump payload +
//   trailing-CRC-pair consumption (Phase 5.1), CRC accumulation (Phase 6),
//   and CRC verification (Phase 7) -- now with full-dump sample decoding
//   wired in. No CRC accumulation call, CRC verification, parser state
//   transition, or byte-consumption boundary changed from Phase 7. Timeout
//   handling is still not implemented here (Design Closure SS2: owned by
//   L3).
//
//   SCAN_ANCHOR and READ_TYPE: byte-for-byte unchanged since Phase 6 (and
//   Phases 2/3 before that) -- see ScanAnchor()'s and ReadType()'s own doc
//   comments (fifo_codec.cpp). As before, this function returns PENDING
//   immediately after each of these completes, never attempting the next
//   state in the same call.
//
//   Progress-frame parsing and full-dump consumption: byte-consumption
//   boundaries, CRC accumulation, and CRC verification are unchanged from
//   Phase 7 -- see ReadProgress()'s and ReadDump()'s own doc comments
//   (fifo_codec.cpp):
//     - ReadProgress(): once partialBuf[2..3] (CRC_lo/CRC_hi) are in hand,
//       the transmitted CRC is reconstructed low-byte-first (K-6) and
//       compared against Crc16_Final(state->runningCrc). Match -> PROGRESS,
//       *outProgressFill populated. Mismatch -> CRC_MISMATCH,
//       *outProgressFill left untouched. Never calls
//       SampleDecoder_DecodeStride() -- no sample payload exists in this
//       shape.
//     - ReadDump(): what's new in THIS phase is that, immediately upon each
//       completed 6-byte payload stride (inside the same payload loop that
//       feeds Crc16_Update(), unchanged from Phase 6), it now also calls
//       SampleDecoder_DecodeStride() exactly once for that stride, writing
//       results through xOut[idx]/yOut[idx]/zOut[idx] (idx derived from
//       partialBytesReceived). Once the trailing CRC pair sub-state has
//       consumed both bytes -- STORED into partialBuf[0..1] as of Phase 7
//       -- the reconstruct-and-compare happens exactly as before. Match ->
//       FULL_DUMP. Mismatch -> CRC_MISMATCH.
//   In both cases, every byte of the frame has already been consumed from
//   the transport by the time the CRC comparison runs -- CRC_MISMATCH is
//   returned only after the full frame boundary has been reached, never
//   mid-frame, so it never leaves bytes belonging to that frame unread.
//
//     IMPORTANT: PROGRESS and FULL_DUMP mean "the entire frame was
//     consumed AND its CRC verified" -- the gate Phase 7 built. Per SDS
//     D-8, `status` (set by the L3 caller, not by FrameCodec) remains the
//     sole authority on sample validity: FULL_DUMP samples are decoded
//     speculatively, stride-by-stride, DURING capture -- before the CRC
//     that covers them has been checked -- but they only become
//     observable to a consumer once the caller has already seen a
//     CRC-verified FULL_DUMP outcome for that same attempt and marks
//     status accordingly (Design Closure SS3). On CRC_MISMATCH, whatever
//     was written into xOut/yOut/zOut for that attempt must be treated by
//     the caller as not valid -- FrameCodec itself does not erase it.
//
//     Per-call byte budget: payload accumulation is bounded to at most
//     FIFO_SERVICE_MAX_BYTES (512, SDS SS7.2's A-3 contract) bytes read per
//     call -- unlike SCAN_ANCHOR (bounded by MAX_DESYNC_BYTES=64) and the
//     progress tail (bounded to 4), the full-dump payload's natural size
//     (6144) genuinely exceeds the per-call service budget, so this is the
//     first sub-phase that needs an explicit cap rather than getting one
//     for free from a small fixed size. Hitting the budget mid-payload
//     returns PENDING, exactly like running out of available bytes -- the
//     caller cannot tell the two cases apart, and does not need to. The
//     trailing-CRC-pair sub-state reads at most 2 bytes, needing no
//     separate budget constant, same reasoning as READ_TYPE and the
//     progress tail.
//
//   Precondition:  `state` has been initialized by FrameCodecState_Reset()
//                  for this attempt (or already holds progress from a
//                  prior FrameCodec_Step() call within the same attempt).
//                  `t`, `outProgressFill`, `xOut`, `yOut`, `zOut` are valid,
//                  non-null, caller-owned.
//   Postcondition: returns the outcome of this call. `*state` is updated to
//                  reflect any scanning/type-read/progress-tail/payload
//                  progress made, including state->runningCrc for every
//                  CRC-covered byte consumed this call. `*outProgressFill`
//                  is written only on a PROGRESS outcome (never on
//                  CRC_MISMATCH). `xOut`/`yOut`/`zOut` are written to only
//                  during full-dump payload parsing (never during progress
//                  parsing), once per completed 6-byte stride, at the
//                  index that stride completed at -- see ReadDump()'s doc
//                  comment (fifo_codec.cpp) for the exact indexing. This
//                  can happen on a call that returns PENDING (mid-payload)
//                  as well as one that returns FULL_DUMP or CRC_MISMATCH
//                  (last stride). Crc16_Update() and Crc16_Final() are both
//                  called; the accumulated value is checked against the
//                  transmitted CRC exactly once per completed frame, on
//                  either a PROGRESS/FULL_DUMP or a CRC_MISMATCH outcome.
FifoFrameOutcome FrameCodec_Step(FifoTransport* t, FrameCodecState* state,
                                  uint16_t* outProgressFill,
                                  int16_t* xOut, int16_t* yOut, int16_t* zOut);
