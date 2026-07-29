// ============================================================================
// [v16.6.8-fifo] fifo_session.cpp
//
// See fifo_session.h for the public contract, session policy table, and
// FifoSessionState field ownership. This file does not modify, include the
// implementation of, or otherwise depend on anything beyond fifo_codec.h's
// public FrameCodec/SampleDecoder/StreamingCrc16 contract (Task 2.3, frozen)
// -- FrameCodec_Step() and FrameCodecState_Reset() are the only two L2
// entry points this file ever calls.
//
// No dynamic allocation, no nowMs()/millis() call, no transport write --
// this file reads FifoTransport only via what it forwards straight through
// to FrameCodec_Step().
// ============================================================================

#include "fifo_session.h"

void FifoSession_Init(FifoSessionState* s) {
  FrameCodecState_Reset(&s->frameState);
  s->phase = FifoSessionPhase::IDLE;
  s->failReason = FifoSessionFailReason::NONE;
  s->lastProgressFill = 0;
}

void FifoSession_StartAttempt(FifoSessionState* s) {
  // s->frameState is already clean here -- either from FifoSession_Init()
  // (first attempt ever) or from the terminal-outcome reset at the end of
  // the previous FifoSession_Step() call that brought phase to
  // IDLE/COMPLETE/FAILED (see FifoSession_Step()'s doc comment). No second
  // FrameCodecState_Reset() call is needed or made here.
  s->phase = FifoSessionPhase::WAIT_FRAME;
}

FifoSessionOutcome FifoSession_Step(FifoSessionState* s, FifoTransport* t,
                                     int16_t* xOut, int16_t* yOut, int16_t* zOut) {
  if (s->phase != FifoSessionPhase::WAIT_FRAME &&
      s->phase != FifoSessionPhase::RECEIVING) {
    // No attempt in progress (IDLE/COMPLETE/FAILED) -- do not touch
    // s->frameState or the transport. Caller must call
    // FifoSession_StartAttempt() first.
    return FifoSessionOutcome::NONE;
  }

  uint16_t progressFill = 0;
  FifoFrameOutcome outcome =
      FrameCodec_Step(t, &s->frameState, &progressFill, xOut, yOut, zOut);

  switch (outcome) {
    case FifoFrameOutcome::PENDING:
      // Still mid-attempt. Classify WAIT_FRAME vs RECEIVING from the one
      // FrameCodecState field this session reads for that purpose alone
      // (see fifo_session.h) -- frameState itself is untouched here beyond
      // whatever FrameCodec_Step() already did to it.
      s->phase = s->frameState.anchorFound ? FifoSessionPhase::RECEIVING
                                            : FifoSessionPhase::WAIT_FRAME;
      return FifoSessionOutcome::NONE;

    case FifoFrameOutcome::PROGRESS:
      s->lastProgressFill = progressFill;
      FrameCodecState_Reset(&s->frameState);
      s->phase = FifoSessionPhase::WAIT_FRAME;
      return FifoSessionOutcome::PROGRESS_UPDATE;

    case FifoFrameOutcome::FULL_DUMP:
      FrameCodecState_Reset(&s->frameState);
      s->phase = FifoSessionPhase::COMPLETE;
      return FifoSessionOutcome::DUMP_COMPLETE;

    case FifoFrameOutcome::CRC_MISMATCH:
      FrameCodecState_Reset(&s->frameState);
      s->phase = FifoSessionPhase::WAIT_FRAME;
      return FifoSessionOutcome::FRAME_REJECTED;

    case FifoFrameOutcome::BAD_TYPE_BYTE:
      FrameCodecState_Reset(&s->frameState);
      s->phase = FifoSessionPhase::FAILED;
      s->failReason = FifoSessionFailReason::BAD_FRAME_TYPE;
      return FifoSessionOutcome::SESSION_FAILED;

    case FifoFrameOutcome::DESYNC_LIMIT:
      FrameCodecState_Reset(&s->frameState);
      s->phase = FifoSessionPhase::FAILED;
      s->failReason = FifoSessionFailReason::DESYNC;
      return FifoSessionOutcome::SESSION_FAILED;

    default:
      // TIMEOUT_NO_RESPONSE / TIMEOUT_INTER_BYTE: per fifo_codec.h,
      // FrameCodec_Step() never produces these itself -- they exist only
      // for a future timeout-owning caller's own use (Design Closure SS2),
      // which this task does not implement. Unreachable in practice; this
      // default exists only to keep the switch exhaustive without a
      // -Wswitch warning.
      return FifoSessionOutcome::NONE;
  }
}
