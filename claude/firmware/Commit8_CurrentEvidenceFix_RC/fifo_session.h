#pragma once
// ============================================================================
// [v16.6.8-fifo] fifo_session.h
//
// L3 -- Session Controller (Implementation Plan Task 3.1; orchestrates
// FrameCodec, Task 2.3, which is complete and frozen as of this task).
//
// This file owns the lifetime of exactly one FrameCodecState and drives it
// through repeated FrameCodec_Step() calls, translating each L2
// FifoFrameOutcome into an L3-level FifoSessionOutcome and an L3-level
// FifoSessionPhase, per the session policy documented on FifoSession_Step()
// below.
//
// Explicitly OUT OF SCOPE for this task (deferred to later L3 tasks):
//   - Timeout detection (Design Closure SS2: wall-clock ownership belongs to
//     a later task built on top of this one; this file never calls nowMs()
//     and has no wall-clock awareness at all).
//   - Request framing/transmission (this file only ever reads via
//     FifoTransport, matching FrameCodec -- it never calls t->write()).
//   - Sample-arena ownership, admission gates, retry/backoff policy, the
//     public multi-entry-point driver API -- all later Implementation Plan
//     tasks (3.2+). This file's `xOut`/`yOut`/`zOut` are passed straight
//     through to FrameCodec_Step(), exactly as FrameCodec itself defines --
//     this file introduces no sample storage of its own.
//
// Zero Arduino/ESP32 dependency by design, matching fifo_transport.h and
// fifo_codec.h -- compiles standalone under a plain C++ compiler. Depends
// only on fifo_codec.h (L2) and, transitively, fifo_transport.h (L1) --
// never modifies either.
// ============================================================================

#include "fifo_codec.h"

// ----------------------------------------------------------------------------
// FifoSessionPhase -- the session-level lifecycle state, coarser than
// FrameCodec's own internal SCAN_ANCHOR/READ_TYPE/payload sub-states (which
// remain entirely FrameCodec's concern -- this file only ever reads
// state.anchorFound to distinguish WAIT_FRAME from RECEIVING, and never
// reads or writes any other FrameCodecState field directly).
//
//   IDLE        No attempt has ever been started (immediately after
//               FifoSession_Init()), or -- not automatically reachable,
//               reserved for a future caller-driven "acknowledge and go
//               idle" step that this task does not implement.
//   WAIT_FRAME  An attempt is in progress (FrameCodecState_Reset() has been
//               called) but the `50 03` anchor has not yet been found
//               (state.anchorFound == false) -- still waiting for the
//               response to begin arriving, or for the session to be
//               re-armed after a PROGRESS or CRC_MISMATCH outcome (both
//               loop back here automatically, see FifoSession_Step()).
//   RECEIVING   The anchor has been found (state.anchorFound == true) --
//               the response has started arriving and its type/payload/CRC
//               are being consumed, regardless of which of FrameCodec's own
//               internal sub-states that currently is.
//   COMPLETE    The most recent attempt ended in FULL_DUMP: samples were
//               published. Terminal -- FifoSession_Step() is a no-op until
//               FifoSession_StartAttempt() is called again.
//   FAILED      The most recent attempt ended in BAD_TYPE_BYTE or
//               DESYNC_LIMIT (see FifoSessionFailReason for which).
//               Terminal -- FifoSession_Step() is a no-op until
//               FifoSession_StartAttempt() is called again.
// ----------------------------------------------------------------------------
enum class FifoSessionPhase {
  IDLE,
  WAIT_FRAME,
  RECEIVING,
  COMPLETE,
  FAILED,
};

// FifoSessionFailReason -- which terminal parser outcome put the session
// into FAILED. Valid only while phase == FAILED; meaningless otherwise (not
// cleared on a successful attempt -- trust-the-phase convention, matching
// frameTypeByte's own "valid only once typeByteRead" contract in FrameCodec).
enum class FifoSessionFailReason {
  NONE,
  BAD_FRAME_TYPE,  // parser outcome was BAD_TYPE_BYTE
  DESYNC,          // parser outcome was DESYNC_LIMIT
};

// FifoSessionOutcome -- the translated, L3-level result of one
// FifoSession_Step() call. See FifoSession_Step()'s doc comment for the
// exact FifoFrameOutcome -> FifoSessionOutcome mapping (the "outcome
// transition table").
enum class FifoSessionOutcome {
  NONE,             // no terminal parser event this call (still PENDING)
  PROGRESS_UPDATE,  // a PROGRESS frame was received and published
  DUMP_COMPLETE,    // a FULL_DUMP frame was received and published
  FRAME_REJECTED,   // CRC_MISMATCH -- this attempt discarded, already
                     // re-armed and waiting for the next frame
  SESSION_FAILED,   // BAD_TYPE_BYTE or DESYNC_LIMIT -- session now FAILED,
                     // see FifoSessionState::failReason for which
};

// ----------------------------------------------------------------------------
// FifoSessionState -- explicit, caller-owned session state. No member here
// is hidden or static, matching FrameCodecState's own convention (SDS D-6).
// ----------------------------------------------------------------------------
struct FifoSessionState {
  // [Task 3.1] The L2 parser state this session owns for its entire
  // lifetime. Never touched by anything outside this file -- see
  // FifoSession_Step()'s doc comment for exactly when it is read
  // (state.anchorFound, for WAIT_FRAME/RECEIVING classification only) vs.
  // reset (FrameCodecState_Reset(), at every attempt boundary).
  FrameCodecState frameState;

  // [Task 3.1] The current session-level lifecycle state. See
  // FifoSessionPhase's own doc comment.
  FifoSessionPhase phase;

  // [Task 3.1] Which terminal parser outcome caused the most recent FAILED
  // transition. Valid only while phase == FAILED.
  FifoSessionFailReason failReason;

  // [Task 3.1] The fill value from the most recently published PROGRESS
  // outcome (SDS K-3). Valid only after at least one PROGRESS_UPDATE has
  // been returned by FifoSession_Step(); this file never clears it on a
  // later FULL_DUMP or FAILED transition -- it simply holds the last
  // published value, exactly like frameTypeByte holds FrameCodec's last
  // validated type byte.
  uint16_t lastProgressFill;
};

// FifoSession_Init() -- initialize `*s` for a brand-new session, never yet
// attempted.
//   Precondition:  `s` is a valid, non-null pointer.
//   Postcondition: s->frameState is reset via FrameCodecState_Reset() (so it
//                  is always in a defined, empty state, even though phase
//                  starts at IDLE and FifoSession_Step() will not drive it
//                  yet). s->phase == IDLE. s->failReason == NONE.
//                  s->lastProgressFill == 0.
void FifoSession_Init(FifoSessionState* s);

// FifoSession_StartAttempt() -- arm the session to begin (or resume)
// receiving a frame.
//   Precondition:  `s` has been initialized by FifoSession_Init(). `s->phase`
//                  is IDLE, COMPLETE, or FAILED -- i.e. no attempt is
//                  currently in progress. Calling this while phase is
//                  WAIT_FRAME or RECEIVING is undefined from this
//                  component's perspective (trust-the-precondition
//                  convention, matching FrameCodecState_Reset()'s own
//                  contract) -- it would abandon an attempt already in
//                  progress without going through the documented outcome
//                  handling.
//   Postcondition: s->phase == WAIT_FRAME. s->frameState is NOT reset here --
//                  it is already clean, either from FifoSession_Init() (the
//                  very first attempt) or from the immediately-preceding
//                  terminal outcome's own reset (every FifoSession_Step()
//                  terminal branch resets s->frameState before returning,
//                  see below) -- so exactly one FrameCodecState_Reset() call
//                  ever backs a given attempt, never a redundant second one
//                  at start time.
void FifoSession_StartAttempt(FifoSessionState* s);

// FifoSession_Step() -- advance the current attempt by whatever bytes are
// currently available on `t`, bounded and non-blocking (inherited directly
// from FrameCodec_Step()'s own PRR C-1 contract -- this function adds no
// blocking of its own).
//
//   No-op precondition: if s->phase is IDLE, COMPLETE, or FAILED, this
//   function does not call FrameCodec_Step() at all and returns
//   FifoSessionOutcome::NONE immediately -- no attempt is in progress, and
//   calling FrameCodec_Step() on a FrameCodecState that has not been
//   (re-)armed via FifoSession_StartAttempt() would violate FrameCodec's own
//   precondition. The caller must call FifoSession_StartAttempt() first.
//
//   Otherwise (s->phase is WAIT_FRAME or RECEIVING), this function calls
//   FrameCodec_Step(t, &s->frameState, ...) exactly once and handles its
//   FifoFrameOutcome per the following table -- the session policy this
//   task specifies:
//
//     FifoFrameOutcome   | s->phase after      | FrameCodecState_Reset()? | FifoSessionOutcome returned
//     -------------------|----------------------|---------------------------|------------------------------
//     PENDING            | WAIT_FRAME or        | no                        | NONE
//                        | RECEIVING (see below) |                           |
//     PROGRESS           | WAIT_FRAME            | yes                       | PROGRESS_UPDATE
//     FULL_DUMP          | COMPLETE              | yes                       | DUMP_COMPLETE
//     CRC_MISMATCH       | WAIT_FRAME            | yes                       | FRAME_REJECTED
//     BAD_TYPE_BYTE      | FAILED                | yes                       | SESSION_FAILED
//     DESYNC_LIMIT       | FAILED                | yes                       | SESSION_FAILED
//
//   On PENDING, s->phase is derived from s->frameState.anchorFound alone
//   (RECEIVING if true, WAIT_FRAME if false) -- this is a read of a public,
//   documented FrameCodecState field for classification purposes only; this
//   function never writes to any FrameCodecState field except via
//   FrameCodecState_Reset() itself.
//
//   Every terminal (non-PENDING) outcome calls FrameCodecState_Reset(
//   &s->frameState) before returning, unconditionally -- including
//   FULL_DUMP, BAD_TYPE_BYTE, and DESYNC_LIMIT, even though those three
//   leave the session in a terminal phase (COMPLETE/FAILED) that will not
//   itself drive another FrameCodec_Step() call until
//   FifoSession_StartAttempt() is called again. Resetting immediately, at
//   the moment each attempt ends, rather than deferring to the next
//   FifoSession_StartAttempt() call, is what guarantees no parser state
//   from a finished attempt can ever leak into whatever attempt comes next
//   -- the two concerns (closing out a finished attempt vs. deciding when
//   to begin the next one) are kept independent.
//
//   PROGRESS and CRC_MISMATCH both loop back to WAIT_FRAME automatically,
//   without requiring FifoSession_StartAttempt() -- a corrupted or
//   in-progress response is expected, ordinary behavior on this bus (SDS
//   K-13), not a condition that should require external intervention to
//   recover from. BAD_TYPE_BYTE and DESYNC_LIMIT, by contrast, indicate a
//   more fundamental desync or protocol violation and deliberately stop at
//   FAILED for the caller to decide what happens next (this task defines no
//   retry/backoff policy -- that is explicitly out of scope, a later task).
//
//   Precondition:  `s` has been initialized by FifoSession_Init() and armed
//                  by at least one FifoSession_StartAttempt() call more
//                  recently than any terminal outcome. `t`, `xOut`, `yOut`,
//                  `zOut` satisfy FrameCodec_Step()'s own precondition
//                  (valid, non-null, caller-owned) whenever s->phase is
//                  WAIT_FRAME or RECEIVING -- this function passes them
//                  through unchanged.
//   Postcondition: returns the FifoSessionOutcome of this call (NONE if no
//                  attempt was in progress, or if FrameCodec_Step() itself
//                  returned PENDING). `s->phase`, and on a terminal outcome
//                  `s->failReason` or `s->lastProgressFill`, are updated per
//                  the table above. `xOut`/`yOut`/`zOut` are written to
//                  exactly as FrameCodec_Step() itself would write to them
//                  -- this function introduces no additional writes, reads,
//                  or buffering of its own.
FifoSessionOutcome FifoSession_Step(FifoSessionState* s, FifoTransport* t,
                                     int16_t* xOut, int16_t* yOut, int16_t* zOut);
