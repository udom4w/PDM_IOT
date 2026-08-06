// ============================================================================
// [v16.6.4-fifo] log_replay_transport.cpp
//
// Host-only. See log_replay_transport.h for the public contract and
// CM-100_FIFO_DRIVER_SDS_v1.0 SS6.1 for the FifoTransport interface this
// satisfies. No FIFO protocol, framing, CRC, retry, or state-machine logic
// exists in this file -- it replays bytes and reports transport-level
// conditions only, exactly like its production sibling
// fifo_transport_uart485.cpp (Task 1.2).
// ============================================================================

#include "log_replay_transport.h"

// ----------------------------------------------------------------------------
// Context: everything needed to replay one recorded byte stream. All pointer
// members are caller-owned (see log_replay_transport.h); this file neither
// allocates nor frees any of the storage they point to. Static storage, not
// heap -- there is exactly one active replay at a time (see Init()'s own
// doc comment).
// ----------------------------------------------------------------------------
struct LogReplayTransportCtx {
  const uint8_t*  recordedBytes;
  size_t          recordedLen;
  const uint32_t* byteTimestampsMs;
  const uint32_t* simulatedNowMs;   // external, test-owned; read-only here
  const bool*     simulatedOverflow; // external, test-owned; read-only here
  size_t          readCursor;       // index of the next byte not yet
                                     // delivered to read()
};

static LogReplayTransportCtx s_ctx = {nullptr, 0, nullptr, nullptr, nullptr, 0};

// ----------------------------------------------------------------------------
// Shared helper: how many bytes starting at readCursor have already
// "arrived" as of *simulatedNowMs -- i.e. byteTimestampsMs[i] <= now, for a
// contiguous run starting at readCursor. Recorded timestamps are
// non-decreasing (Init()'s documented precondition), so this is a single
// forward scan, never a search.
// ----------------------------------------------------------------------------
static size_t Available_Locked(LogReplayTransportCtx* c) {
  uint32_t now = *c->simulatedNowMs;
  size_t n = 0;
  while (c->readCursor + n < c->recordedLen &&
         c->byteTimestampsMs[c->readCursor + n] <= now) {
    n++;
  }
  return n;
}

// ----------------------------------------------------------------------------
// read() -- FifoTransport contract: non-blocking, copies up to maxLen
// already-received bytes, 0 is normal.
//   Ownership:  copies from the caller-supplied recordedBytes array; owns
//               none of it, only the copy destination `dst` (caller-owned).
//   Blocking:   non-blocking by construction -- this function never waits;
//               it only ever reports bytes already "arrived" as of the
//               current value of *simulatedNowMs, exactly once, then
//               returns. Time itself never moves as a side effect of
//               calling this function -- only the external test moves it.
//   Error:      none -- 0 returned means "nothing arrived yet as of the
//               current simulated time," the fixture's equivalent of the
//               production transport's "nothing buffered right now."
//   Assumption: `ctx` is always the address of s_ctx, populated by
//               LogReplayTransport_Init().
// ----------------------------------------------------------------------------
static size_t LogReplayTransport_Read(void* ctx, uint8_t* dst, size_t maxLen) {
  LogReplayTransportCtx* c = static_cast<LogReplayTransportCtx*>(ctx);
  size_t avail = Available_Locked(c);
  size_t n = (avail < maxLen) ? avail : maxLen;
  for (size_t i = 0; i < n; i++) {
    dst[i] = c->recordedBytes[c->readCursor + i];
  }
  c->readCursor += n;
  return n;
}

// ----------------------------------------------------------------------------
// write() -- FifoTransport contract: may block only for the physical
// duration of transmitting `len` bytes.
//   Ownership:  nothing to own -- there is no real transmission medium in a
//               replay. This function SIMULATES A SUCCESSFUL TRANSMISSION:
//               it reports all `len` bytes as written, matching the
//               contract's return value (0..len, "actual bytes written"),
//               without producing any observable output. It deliberately
//               does not inspect `src` -- a replay fixture verifies driver
//               behavior against *received* bytes (the recorded stream),
//               not against what the driver chose to transmit.
//   Blocking:   non-blocking -- there is no physical medium to wait on.
//   Error:      none -- always reports the full length as written; a
//               fixture that needs to test a short/failed write can be
//               extended to do so later (not required by Task 1.3's scope).
//   Assumption: same as read().
// ----------------------------------------------------------------------------
static size_t LogReplayTransport_Write(void* ctx, const uint8_t* src, size_t len) {
  (void)ctx;
  (void)src;
  return len;
}

// ----------------------------------------------------------------------------
// available() -- FifoTransport contract: non-blocking byte count.
//   Ownership:  read-only query, same source as read().
//   Blocking:   non-blocking.
//   Error:      none.
//   Assumption: same as read().
// ----------------------------------------------------------------------------
static size_t LogReplayTransport_Available(void* ctx) {
  LogReplayTransportCtx* c = static_cast<LogReplayTransportCtx*>(ctx);
  return Available_Locked(c);
}

// ----------------------------------------------------------------------------
// flushRx() -- FifoTransport contract: discard all buffered RX bytes,
// non-blocking.
//   Ownership:  advances the read cursor past every byte currently
//               "arrived" as of *simulatedNowMs -- those bytes are
//               discarded, never delivered via a subsequent read(). Bytes
//               that have not yet arrived (timestamp still in the
//               fixture's simulated future) are unaffected; flushing does
//               not move time forward.
//   Blocking:   non-blocking BY CONSTRUCTION -- a single forward cursor
//               advance, bounded by recordedLen.
//   Error:      none.
//   Assumption: same as read().
// ----------------------------------------------------------------------------
static void LogReplayTransport_FlushRx(void* ctx) {
  LogReplayTransportCtx* c = static_cast<LogReplayTransportCtx*>(ctx);
  c->readCursor += Available_Locked(c);
}

// ----------------------------------------------------------------------------
// nowMs() -- FifoTransport contract: monotonic milliseconds, non-blocking.
//   Ownership:  delegates to the external, test-owned *simulatedNowMs --
//               this file never writes through this pointer, only reads
//               it. Mirrors Uart485Transport_NowMs() delegating to the
//               external, transport-external millis() it doesn't own
//               either -- same shape, a test-controlled source instead of
//               a hardware one.
//   Blocking:   non-blocking.
//   Error:      none.
//   Assumption: *simulatedNowMs is non-decreasing across a single replay
//               scenario -- enforced by the test driving it, not by this
//               file.
// ----------------------------------------------------------------------------
static uint32_t LogReplayTransport_NowMs(void* ctx) {
  LogReplayTransportCtx* c = static_cast<LogReplayTransportCtx*>(ctx);
  return *c->simulatedNowMs;
}

// ----------------------------------------------------------------------------
// hadOverflow() -- FifoTransport contract: true if an RX overflow occurred
// since the last call, non-blocking.
//   Ownership:  delegates to the external, test-owned *simulatedOverflow --
//               this file never writes through this pointer, only reads
//               it, exactly mirroring nowMs()'s relationship to
//               *simulatedNowMs. SDS SS6.1 states hadOverflow() is the sole
//               source of ERR_RX_OVERFLOW; hardcoding this to always false
//               would make that one error code permanently unreachable by
//               any fixture-driven test, cutting against the "all of L2
//               and L3 host-testable" testability target (PRR C-14).
//   Blocking:   non-blocking -- a single pointer dereference.
//   Error:      this callback IS the error report.
//   Semantics:  deliberately NOT clear-on-read, unlike
//               Uart485Transport_HadOverflow(). That production callback
//               owns and clears its own flag; this one only ever reads
//               caller-owned state, so clearing it would mean this
//               transport writing to memory it doesn't own -- exactly what
//               the external-state pattern (see simulatedNowMs) is meant to
//               avoid. The test is responsible for setting *simulatedOverflow
//               to true immediately before the call it wants to observe,
//               and clearing it back to false itself afterward.
//   Assumption: same as read() -- `ctx` is always &s_ctx, populated by
//               LogReplayTransport_Init().
// ----------------------------------------------------------------------------
static bool LogReplayTransport_HadOverflow(void* ctx) {
  LogReplayTransportCtx* c = static_cast<LogReplayTransportCtx*>(ctx);
  return *c->simulatedOverflow;
}

// ----------------------------------------------------------------------------
void LogReplayTransport_Init(FifoTransport* out,
                              const uint8_t* recordedBytes,
                              size_t recordedLen,
                              const uint32_t* byteTimestampsMs,
                              const uint32_t* simulatedNowMs,
                              const bool* simulatedOverflow) {
  s_ctx.recordedBytes = recordedBytes;
  s_ctx.recordedLen = recordedLen;
  s_ctx.byteTimestampsMs = byteTimestampsMs;
  s_ctx.simulatedNowMs = simulatedNowMs;
  s_ctx.simulatedOverflow = simulatedOverflow;
  s_ctx.readCursor = 0;

  out->ctx = &s_ctx;
  out->read = LogReplayTransport_Read;
  out->write = LogReplayTransport_Write;
  out->available = LogReplayTransport_Available;
  out->flushRx = LogReplayTransport_FlushRx;
  out->nowMs = LogReplayTransport_NowMs;
  out->hadOverflow = LogReplayTransport_HadOverflow;
}
