// ============================================================================
// [v16.6.12-fifo] fifo_driver.cpp
//
// See fifo_driver.h for FifoState. This file implements two internal
// entry points, matching the frozen Implementation Plan's own two-task
// naming exactly:
//   - FifoDriver_ServiceCoreState(FifoTransport* t)     -- Task 3.4, S0-S9
//   - FifoDriver_ServiceRecoveryState(FifoTransport* t) -- Task 3.5, S10-S14
// Both are `static` ("Public interfaces: None yet" for both tasks). Each
// switches only on its own range of FifoState values and silently
// no-ops (default: break;) for every state outside that range -- a
// future FifoDriver_Service() (Task 3.6) is expected to call both, once
// per tick, unconditionally; exactly one of the two ever does real work
// on any given tick, based on whatever s_state currently holds.
//
// ============================================================================
// TASK 3.4 SCOPE (S0-S9, "Core protocol state machine") -- unchanged from
// that task's own delivery except for the two additions Task 3.5 was
// explicitly asked to make within this same range:
//   - T_REQUEST_RESPONSE_MS / T_INTER_BYTE_MS stall detection inside the
//     Session-Controller-composed S4/S5/S6/S8 span (HandleReceiving()'s
//     FifoSessionOutcome::NONE case).
//   - T_POLL_INTERVAL_MS now the REAL, Task-3.5-owned named constant --
//     Task 3.4A's kPollIntervalPlaceholderMs is deleted outright (not
//     renamed), exactly as that task's own comment said it must be.
// S4/S5/S6/S8 remain "a single composed action" from this file's
// perspective per ADR-0004/SDS v1.4 Delta SS4 -- this file's own FifoState
// value while Session Controller drives reception is still only ever
// S4_AWAIT_ANCHOR (mirroring WAIT_FRAME) or S8_READ_DUMP (representative
// of RECEIVING); S5/S6 remain declared-but-not-independently-tracked, as
// disclosed in Task 3.4's own delivery.
//
// ============================================================================
// TASK 3.5 SCOPE (S10-S14, "Recovery/lifecycle states and timeout/retry
// policy") -- new in this revision.
//
//  - S10 DRAIN: bus-settle to true silence (T_DRAIN_QUIET_MS), THEN the
//    SS17.1 "Attempt level -> RETRY_ATTEMPT" decision: if the just-ended
//    attempt failed and FIFO_MAX_RETRIES has not been exhausted, route
//    through S1 IDLE for exactly one tick (releasing the bus for "at
//    least one normal poll tick", SS17.3) before re-arming for a fresh
//    attempt on the SAME capture (captureId/tag/triggerSource unchanged,
//    retryCount incremented). Otherwise ("Session level -> FAIL" or
//    success), the circuit breaker's consecutive-failure counter (SS17.6)
//    is updated here -- this is the one point that definitively knows the
//    session's final outcome -- and the state advances to S11.
//  - S11 RESULT_READY: SS9.1's "no" (bus not held) state. Watches
//    FifoArena_IsOwned()/FifoArena_HeldSinceMs() (Task 3.3/3.3A, never
//    FrameCodecState or FifoSessionState) for the SS17.5 result-hold
//    watchdog. A held-too-long result is force-reclaimed via
//    FifoArena_Release() and returns DIRECTLY to S1 IDLE, per SS17.5's
//    literal text -- see this state's own doc comment for the tension
//    that creates against SS17.4's "cooldown after ANY session" framing,
//    and why the more specific SS17.5 text was followed.
//  - S12 COOLDOWN: T_COOLDOWN_MS post-session rest, THEN the deferred
//    circuit-breaker threshold check (SS17.6) -- S14 DISABLED if tripped,
//    S1 IDLE otherwise. Placed here, not at S10, so the cooldown period
//    genuinely applies "after any session, success or failure" (SS17.4)
//    even for the session that trips the breaker.
//  - S13 FAILED: unchanged from Task 3.4 -- unconditional, immediate
//    hand-off to S10 (SS11.1: "yes -> drain | -> S10").
//  - S14 DISABLED: no auto-exit implemented -- SS17.6: "Manual reset
//    ensures a human sees the event; auto-reset would let a persistent
//    hardware fault oscillate indefinitely." Exit is a future explicit
//    reset entry point (Task 3.6, not this file).
//
// Explicitly NOT implemented, per instruction: Public API, admission gate
// (SS19.2 gating a request before s_requestPending is ever set), MQTT,
// logging framework (SS17.5's "logs ERR_RESULT_NOT_RELEASED" is satisfied
// at the DATA level only -- s_result.error is set; actual Serial/MQTT
// output is a later task's job), Arduino setup()/loop(), transport
// implementation, telemetry publishing. Also explicitly not implemented,
// per the frozen Implementation Plan's own scope for THIS task: T_DUMP_TOTAL_MS
// and T_CAPTURE_TOTAL_MS (whole-phase/whole-session ceilings, distinct
// from the five per-tick timeouts this task was asked to implement) --
// flagged as remaining work, not fabricated.
//
// No dynamic allocation. No duplicated retry state (s_result.retryCount,
// already part of the frozen FifoCaptureResult shape, is the SOLE retry
// counter -- no parallel field was added). No duplicated timeout
// ownership (MAX_DESYNC_BYTES stays exclusively in fifo_codec.cpp, Task
// 2.3; T_POLL_INTERVAL_MS now has exactly one definition, here,
// superseding Task 3.4A's placeholder).
// ============================================================================

#include "fifo_driver.h"
#include "fifo_types.h"
#include "fifo_transport.h"
#include "fifo_codec.h"
#include "fifo_session.h"
#include "fifo_arena.h"
#include <stddef.h>
#include <stdio.h>
#include <stdarg.h>

// ----------------------------------------------------------------------------
// D-14 / G-1: the single named compile-time constant Task 3.4 owns, per
// the frozen Implementation Plan's own Task 3.3(old)/3.4(new) Risks field.
// TRUEPOLL is the provisional default (SS11.3). Unchanged from Task 3.4.
// ----------------------------------------------------------------------------
#define FIFO_PROTOCOL_MODE_TRUEPOLL   0
#define FIFO_PROTOCOL_MODE_PURELISTEN 1
#define FIFO_PROTOCOL_MODE FIFO_PROTOCOL_MODE_PURELISTEN

// ----------------------------------------------------------------------------
// [Task 3.5] SDS SS15.1 (D-17) timeout budgets and SS17.3/17.6 retry/
// breaker constants this task owns. Values are exactly the frozen SDS's
// own published numbers -- none invented. T_DUMP_TOTAL_MS and
// T_CAPTURE_TOTAL_MS are deliberately absent -- not part of this task's
// explicit scope (see file header, "Explicitly NOT implemented").
// MAX_DESYNC_BYTES is deliberately absent too -- it already lives in
// fifo_codec.cpp (Task 2.3, frozen); duplicating it here would be exactly
// the "duplicated timeout ownership" this task's requirements forbid.
// ----------------------------------------------------------------------------
static const uint32_t T_REQUEST_RESPONSE_MS = 1000;  // K-12: pre-anchor response budget
static const uint32_t T_INTER_BYTE_MS       = 500;   // K-13: mid-frame stall budget
static const uint32_t T_POLL_INTERVAL_MS    = 250;   // SS11.1 S7's sole exit condition
static const uint32_t T_DRAIN_QUIET_MS      = 300;   // SS11.1 S10's sole exit condition
static const uint32_t T_RESULT_HOLD_MAX_MS  = 30000; // SS17.5 watchdog
static const uint32_t T_COOLDOWN_MS         = 60000; // SS17.4
static const uint8_t  FIFO_MAX_RETRIES      = 2;     // SS17.3: 3 attempts total
static const uint8_t  FIFO_BREAKER_THRESHOLD = 5;    // SS17.6

// ----------------------------------------------------------------------------
// Internal driver state -- singleton, matching taskModbusRead()'s own
// singleton nature (D-6). None of this is declared in fifo_driver.h.
// ----------------------------------------------------------------------------
static FifoState         s_state = FifoState::S0_UNINIT;
static FifoSessionState  s_session;

// [Provisional intake hook, Task 3.4] A future public FifoDriver_Request()
// (not this task's job -- "admission gate"/"public API" both excluded) is
// expected to populate s_pendingRequest and set s_requestPending = true,
// having ALREADY performed SS19.2's admission gating itself.
static bool               s_requestPending = false;
static FifoCaptureRequest s_pendingRequest;

// [Task 3.5] Set by HandleS10Drain() when a retry is warranted, consumed
// by HandleS1Idle() on the very next tick. Deliberately a SEPARATE flag
// from s_requestPending -- conflating "resume the same capture" with "a
// brand new external request arrived" would be exactly the kind of
// duplicated/ambiguous state this task's requirements forbid.
static bool s_retryPending = false;

static FifoCaptureResult s_result;
static uint32_t           s_captureIdCounter = 0;
static uint16_t           s_attemptFillBaseline = 0;  // K-10 anchor, reset per attempt

// [Task 3.4] Timestamp of the most recent S7_POLL_WAIT entry, ms.
static uint32_t s_pollWaitEnteredAtMs = 0;

// [Task 3.5] Timestamp of the last observed transport activity
// (t->available() > 0) since the current attempt's request was sent (or
// since the last resync point -- see HandleReceiving()/HandleS3Request()
// for every place this is reset). Drives T_REQUEST_RESPONSE_MS (pre-
// anchor) / T_INTER_BYTE_MS (post-anchor) detection without reaching into
// FrameCodecState/FifoSessionState internals -- purely a transport-level
// (FifoTransport::available()) observation, respecting Session
// Controller's "never touched by anything outside this file" ownership
// of FrameCodecState.
static uint32_t s_lastByteActivityMs = 0;

// [Task 3.5] Timestamp of the most recent S10_DRAIN entry, and the
// "quiet since" watermark reset on every byte seen during drain.
static uint32_t s_drainQuietSinceMs = 0;

// [Task 3.5] True once FifoArena_IsOwned() has been observed true during
// the current S11_RESULT_READY residency -- distinguishes "acquired, now
// released" (normal completion) from "never yet acquired" (still
// waiting, no timeout bound specified by the frozen SDS for this case).
static bool s_wasAcquiredThisHold = false;

// [Task 3.5] Timestamp of the most recent S12_COOLDOWN entry.
static uint32_t s_cooldownEnteredAtMs = 0;

// [Task 3.5] SS17.6 circuit breaker: consecutive FAILED SESSIONS (not
// attempts -- a session may internally retry up to FIFO_MAX_RETRIES times
// and still count as one). Incremented in HandleS10Drain() at the one
// point a session's final outcome is known; reset to 0 on any successful
// session. Persists across sessions (unlike s_result, which resets per
// session) -- this is the sole counter for this purpose, no duplicate.
static uint8_t s_consecutiveFailedSessions = 0;

// [Task 3.6] Bound once by FifoDriver_Init(), used by FifoDriver_Service()
// to drive the two internal service functions -- neither of which the
// PUBLIC FifoDriver_Service() signature (SDS SS7.1 A-3) takes a parameter
// for, unlike their `static`-linkage internal counterparts.
static FifoTransport* s_transport = nullptr;

// [Task 3.6] The captureId FifoDriver_Request() has already assigned
// synchronously, consumed by HandleS1Idle() when it next runs (see that
// function's own comment). Meaningless once consumed; not a second
// source of truth for captureId -- s_captureIdCounter remains the sole
// counter, now incremented only by FifoDriver_Request().
static uint32_t s_pendingCaptureId = 0;

// ----------------------------------------------------------------------------
// [Task 4.4 -- TEMPORARY DIAGNOSTIC ONLY, retry-root-cause investigation,
// not a permanent production feature] Optional diagnostic hook + one extra
// piece of purely-observational bookkeeping (s_stateEnteredAtMs: when the
// CURRENT s_state was entered, used only to compute "elapsed time in
// previous state" for the log line -- read by nothing else in this file,
// influences no decision). See fifo_driver.h's FifoDriver_SetDiagLogger()
// doc comment for why this is a pluggable hook (defaulting to a no-op)
// rather than a direct Serial.* call: this file must keep building on a
// plain host compiler for test/test_fifo_driver.cpp. No dynamic
// allocation -- fixed local buffer + vsnprintf, matching this file's
// existing "no dynamic allocation" convention. Intended to be removed once
// this investigation concludes.
// ----------------------------------------------------------------------------
static void (*s_diagLog)(const char* msg) = nullptr;

// [Task 7.1 -- TEMPORARY DIAGNOSTIC ONLY] Default-initialized to nullptr
// function pointers (FifoUartDiagHooks has no constructor, so this relies on
// static storage's zero-initialization, matching s_diagLog's own default
// above) -- both hooks unset until FifoDriver_SetUartDiagHooks() is called.
static FifoUartDiagHooks s_uartDiagHooks = {};

// [Task 7.5 -- TEMPORARY DIAGNOSTIC ONLY, S8-entry latency-budget audit, not
// a permanent production feature] Per-REQUEST-CYCLE (reset every time
// HandleS3Request() fires -- i.e. once per S3_REQUEST, including the
// PROGRESS-frame re-request cycles inside S7_POLL_WAIT->S3_REQUEST, not
// just once per whole attempt) capture flags/timestamps for T0..T5. This
// granularity lets the LAST request cycle of an attempt -- the one that
// turns out to be the FULL_DUMP response and actually reaches T4/T5 -- be
// examined on its own, separately from earlier PROGRESS-frame cycles
// (which only ever reach T0..T3, since ReadDump() is never the active
// sub-function for a progress-shaped response).
struct FifoS8LatencyState {
  bool     t0Captured, t1Captured, t2Captured, t3Captured, t4Captured, t5Captured;
  uint32_t t0Us, t1Us, t2Us, t3Us, t4Us, t5Us;
};
static FifoS8LatencyState s_s8Lat = {};

// [Task 7.6 -- TEMPORARY DIAGNOSTIC ONLY, UART byte accounting, not a
// permanent production feature] Per-request-cycle running totals -- reset
// at the same point as s_s8Lat (every HandleS3Request() call, i.e. every
// S3_REQUEST including PROGRESS-frame re-request cycles), accumulated
// tick-by-tick, printed as one summary row when the cycle ends (the next
// HandleS3Request(), or the attempt's own terminal transition). NOT timing
// data -- pure byte counts, per this task's "do not add more timing
// instrumentation" scope.
struct FifoByteAccounting {
  uint32_t bytesEntered;    // derived: (availableAfter-availableBefore) + bytesConsumed, summed
  uint32_t bytesConsumed;   // == bytes removed from the ring buffer (read() is the only remover)
  uint32_t rxAvailAtStart;  // ring buffer occupancy observed at this cycle's first tick
  uint32_t rxAvailLast;     // most recent availableAfterStep observed this cycle ("remaining")
  bool     haveData;
};
static FifoByteAccounting s_byteAcct = {};


// [Task 7.4 -- TEMPORARY DIAGNOSTIC ONLY] microsecond-clock pull hook -- see
// fifo_driver.h's FifoDriver_SetMicrosProvider() doc comment.
static uint32_t (*s_microsProvider)() = nullptr;

static uint32_t NowMicrosForDiag() {
  return s_microsProvider ? s_microsProvider() : 0;
}
static uint32_t s_stateEnteredAtMs = 0;

static void DiagLog(const char* fmt, ...) {
  if (!s_diagLog) {
    return;  // default, zero-cost path -- matches every other caller of
             // this file with no logger registered (e.g. the host test build)
  }
  char buf[224];  // [Task 4.8] bumped from 176 -- purely a diagnostic-format
                   // capacity margin (vsnprintf still safely truncates if
                   // ever exceeded); no protocol/timeout/retry/CRC effect.
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  s_diagLog(buf);
}

// StateName() -- human-readable FifoState label for diagnostic lines only;
// never used for any control-flow decision.
static const char* StateName(FifoState s) {
  switch (s) {
    case FifoState::S0_UNINIT:        return "S0_UNINIT";
    case FifoState::S1_IDLE:          return "S1_IDLE";
    case FifoState::S2_ARMED:         return "S2_ARMED";
    case FifoState::S3_REQUEST:       return "S3_REQUEST";
    case FifoState::S4_AWAIT_ANCHOR:  return "S4_AWAIT_ANCHOR";
    case FifoState::S5_READ_TYPE:     return "S5_READ_TYPE";
    case FifoState::S6_READ_PROGRESS: return "S6_READ_PROGRESS";
    case FifoState::S7_POLL_WAIT:     return "S7_POLL_WAIT";
    case FifoState::S8_READ_DUMP:     return "S8_READ_DUMP";
    case FifoState::S9_VERIFY:        return "S9_VERIFY";
    case FifoState::S10_DRAIN:        return "S10_DRAIN";
    case FifoState::S11_RESULT_READY: return "S11_RESULT_READY";
    case FifoState::S12_COOLDOWN:     return "S12_COOLDOWN";
    case FifoState::S13_FAILED:       return "S13_FAILED";
    case FifoState::S14_DISABLED:     return "S14_DISABLED";
    default:                          return "?";
  }
}

// FifoErrorName() -- human-readable FifoError label for diagnostic lines
// only; never used for any control-flow decision.
static const char* FifoErrorName(FifoError e) {
  switch (e) {
    case FifoError::NONE:                    return "NONE";
    case FifoError::ERR_NO_RESPONSE:         return "ERR_NO_RESPONSE";
    case FifoError::ERR_INTER_BYTE_TIMEOUT:  return "ERR_INTER_BYTE_TIMEOUT";
    case FifoError::ERR_DESYNC_LIMIT:        return "ERR_DESYNC_LIMIT";
    case FifoError::ERR_RX_OVERFLOW:         return "ERR_RX_OVERFLOW";
    case FifoError::ERR_CRC_MISMATCH:        return "ERR_CRC_MISMATCH";
    case FifoError::ERR_BAD_TYPE_BYTE:       return "ERR_BAD_TYPE_BYTE";
    case FifoError::ERR_PROGRESS_REGRESSION: return "ERR_PROGRESS_REGRESSION";
    case FifoError::ERR_PROGRESS_OVERRUN:    return "ERR_PROGRESS_OVERRUN";
    case FifoError::ERR_BUSY:                return "ERR_BUSY";
    case FifoError::ERR_NOT_PERMITTED:       return "ERR_NOT_PERMITTED";
    case FifoError::ERR_RESULT_NOT_RELEASED: return "ERR_RESULT_NOT_RELEASED";
    case FifoError::ERR_ABORTED:             return "ERR_ABORTED";
    case FifoError::ERR_RETRY_EXHAUSTED:     return "ERR_RETRY_EXHAUSTED";
    case FifoError::ERR_CIRCUIT_OPEN:        return "ERR_CIRCUIT_OPEN";
    default:                                 return "?";
  }
}

// ----------------------------------------------------------------------------
// CurrentPhase() -- [responsibility #7, Task 3.4] the sole S0..S14 ->
// FifoPhase mapping this driver produces. Unchanged from Task 3.4 --
// already correctly covered S10-S14 in anticipation of this task.
// ----------------------------------------------------------------------------
static FifoPhase CurrentPhase() {
  switch (s_state) {
    case FifoState::S0_UNINIT:
    case FifoState::S1_IDLE:
      return FifoPhase::IDLE;
    case FifoState::S11_RESULT_READY:
      return FifoPhase::RESULT_READY;
    case FifoState::S12_COOLDOWN:
      return FifoPhase::COOLDOWN;
    case FifoState::S14_DISABLED:
      return FifoPhase::BREAKER_DISABLED;
    default:
      // S2_ARMED .. S10_DRAIN, and S13_FAILED (transient, en route to S10).
      return FifoPhase::ACTIVE;
  }
}

// ----------------------------------------------------------------------------
// [Task 3.6] IsCaptureInProgress() -- true for exactly the internal
// states an in-progress capture occupies (S2..S9). Used only by
// FifoDriver_Abort() to decide whether there is anything to abort.
// Written as an explicit case list, not a numeric range comparison over
// FifoState's ordinal values, so it stays correct even if a future edit
// reorders the enum.
// ----------------------------------------------------------------------------
static bool IsCaptureInProgress() {
  switch (s_state) {
    case FifoState::S2_ARMED:
    case FifoState::S3_REQUEST:
    case FifoState::S4_AWAIT_ANCHOR:
    case FifoState::S5_READ_TYPE:
    case FifoState::S6_READ_PROGRESS:
    case FifoState::S7_POLL_WAIT:
    case FifoState::S8_READ_DUMP:
    case FifoState::S9_VERIFY:
      return true;
    default:
      return false;
  }
}

// ----------------------------------------------------------------------------
// BuildAndSendRequest() -- emit the 8-byte RAWFIFO request frame (K-2).
// Unchanged from Task 3.4.
// ----------------------------------------------------------------------------
static void BuildAndSendRequest(FifoTransport* t) {
  uint8_t frame[8] = {0x50, 0x03, 0x00, 0x2C, 0x00, 0x01, 0, 0};
  uint16_t crc;
  Crc16_Init(&crc);
  for (int i = 0; i < 6; i++) {
    Crc16_Update(&crc, frame[i]);
  }
  uint16_t finalCrc = Crc16_Final(crc);
  frame[6] = static_cast<uint8_t>(finalCrc & 0xFF);         // CRC_lo (K-6: low-byte-first)
  frame[7] = static_cast<uint8_t>((finalCrc >> 8) & 0xFF);  // CRC_hi
  t->write(t->ctx, frame, sizeof(frame));
}

// ----------------------------------------------------------------------------
// Per-state handlers, S0-S9 (Task 3.4, with Task 3.5's timeout additions
// clearly marked).
// ----------------------------------------------------------------------------

static void HandleS1Idle() {
  if (s_retryPending) {
    // [Task 3.5] Resume the SAME capture -- captureId/tag/triggerSource/
    // pollCount/retryCount/progressFrameCount/crcErrorCount all already
    // correct on s_result from before this retry (see HandleS10Drain()).
    s_retryPending = false;
    s_attemptFillBaseline = 0;
    s_state = FifoState::S2_ARMED;
    return;
  }
  if (!s_requestPending) {
    return;  // "Ready" -- remain IDLE until an admitted request arrives
  }
  s_requestPending = false;

  // [Task 7.1 -- TEMPORARY DIAGNOSTIC ONLY] requirement 4: reset the UART
  // error counters HERE, and only here -- this is the branch reached
  // exclusively by a genuinely NEW FIFO session admission (the retry-resume
  // branch above already returned before reaching this line). Nullptr-safe.
  if (s_uartDiagHooks.resetStats) {
    s_uartDiagHooks.resetStats();
  }

  // [Task 3.6] captureId is now pre-assigned synchronously by
  // FifoDriver_Request() (so it can return a handle immediately) rather
  // than incremented here -- s_captureIdCounter itself is no longer
  // touched by this function; see FifoDriver_Request()'s own comment.
  s_result = FifoCaptureResult{};  // fresh, zero-initialized result
  s_result.captureId = s_pendingCaptureId;
  s_result.triggerSource = s_pendingRequest.triggerSource;
  for (size_t i = 0; i < FIFO_TAG_MAXLEN; i++) {
    s_result.tag[i] = s_pendingRequest.tag[i];
  }
  s_attemptFillBaseline = 0;

  s_state = FifoState::S2_ARMED;  // "Provenance latched, gates passed"
}

static void HandleS2Armed() {
  FifoSession_StartAttempt(&s_session);
  s_state = FifoState::S3_REQUEST;
}

// [Task 7.5 -- TEMPORARY DIAGNOSTIC ONLY] forward declaration -- defined
// further down, alongside the rest of this task's helpers; needed here
// because HandleS3Request() (T0's capture site) is defined earlier in the
// file than LogS8LatencyStage() itself.
static void LogS8LatencyStage(const char* stageName, uint32_t nowUs, uint32_t prevUs,
                               bool havePrev, long rxAvail);
// [Task 7.6 -- TEMPORARY DIAGNOSTIC ONLY] forward declaration -- see
// LogS8LatencyStage()'s own comment above for why (HandleS3Request() is
// defined before this task's own helpers).
static void LogByteAcctCycleSummary(const char* reason);

static void HandleS3Request(FifoTransport* t) {
  uint32_t now = t->nowMs(t->ctx);
  s_result.tRequestMs = now;
  s_lastByteActivityMs = now;  // [Task 3.5] starts this attempt's stall clock
  // [Task 7.6 -- TEMPORARY DIAGNOSTIC ONLY] print the JUST-ENDED cycle's
  // byte-accounting summary (if any ticks were observed in it) BEFORE
  // resetting for this new cycle -- this is the "PROGRESS_UPDATE looped
  // back to another S3_REQUEST" case; the terminal-outcome case (attempt
  // ends instead of re-requesting) is flushed separately, in
  // HandleS9Verify().
  LogByteAcctCycleSummary("next_request");
  s_byteAcct = FifoByteAccounting{};
  // [Task 7.5 -- TEMPORARY DIAGNOSTIC ONLY] T0 -- reset ALL T0..T5 capture
  // flags for this NEW request cycle (every S3_REQUEST -- including the
  // PROGRESS-frame re-request cycles via S7_POLL_WAIT->S3_REQUEST, not just
  // once per whole attempt -- see s_s8Lat's own struct-level doc comment
  // for why), then capture T0 itself.
  s_s8Lat = FifoS8LatencyState{};
  s_s8Lat.t0Captured = true;
  s_s8Lat.t0Us = NowMicrosForDiag();
  BuildAndSendRequest(t);
  s_result.pollCount++;
  // [Task 4.4 -- TEMPORARY DIAGNOSTIC ONLY] "command transmitted" -- the
  // frame is fixed (50 03 00 2C 00 01 + CRC), matching BuildAndSendRequest()
  // exactly; logged here, not inside that function, so this remains the
  // only new call site in the S3 path.
  DiagLog("[FIFO-DIAG] attempt=%u cmd_sent=50 03 00 2C 00 01 (RAWFIFO read req) pollCount=%u t=%lums",
          (unsigned)(s_result.retryCount + 1), (unsigned)s_result.pollCount, (unsigned long)now);
  LogS8LatencyStage("T0_RequestTransmitted", s_s8Lat.t0Us, 0, false, -1);
  s_state = FifoState::S4_AWAIT_ANCHOR;  // exit condition: "Write complete"
}

// ----------------------------------------------------------------------------
// [Task 4.7 -- TEMPORARY DIAGNOSTIC ONLY, "final 1%" investigation, not a
// permanent production feature] Verbose per-tick parser snapshot, gated to
// fire only once this attempt's fill has crossed 5500 (requirement 1: "log
// every received frame after fill > 5500"). Reads ONLY already-public
// FrameCodecState fields (fifo_codec.h's own frozen, caller-visible struct:
// anchorFound/typeByteRead/frameTypeByte/partialBytesReceived) via
// s_session.frameState, plus this file's own existing s_lastByteActivityMs/
// s_attemptFillBaseline -- fifo_codec.cpp/.h and fifo_session.cpp are NOT
// modified anywhere in this task; every value logged here already existed
// and was already readable, just not yet surfaced. kFullDumpTotalLenForDiag
// (6146 = 6144 payload + 2 CRC bytes, SDS K-4) is a plain literal here,
// MIRRORING -- never redefining or relocating -- fifo_codec.cpp's own
// private FULL_DUMP_TOTAL_LEN constant of the identical frozen value,
// purely for requirement 2's "expected remaining bytes" calculation;
// nothing about the protocol's byte counting changes. Intended to be
// removed once this investigation concludes.
// ----------------------------------------------------------------------------
static const uint32_t kFullDumpTotalLenForDiag = 6146;  // K-4: 6144 payload + 2 trailing CRC bytes

static const char* OutcomeName(FifoSessionOutcome o) {
  switch (o) {
    case FifoSessionOutcome::NONE:            return "NONE(pending)";
    case FifoSessionOutcome::PROGRESS_UPDATE:  return "PROGRESS_UPDATE";
    case FifoSessionOutcome::DUMP_COMPLETE:    return "DUMP_COMPLETE";
    case FifoSessionOutcome::FRAME_REJECTED:   return "FRAME_REJECTED(CRC_MISMATCH)";
    case FifoSessionOutcome::SESSION_FAILED:   return "SESSION_FAILED";
    default:                                   return "?";
  }
}

static void LogNearEndSnapshot(FifoSessionOutcome outcome, uint32_t now) {
  if (s_attemptFillBaseline <= 5500) {
    return;  // requirement 1's threshold gate -- not yet in the endgame zone
  }
  const FrameCodecState& fs = s_session.frameState;
  const char* parserState =
      !fs.anchorFound              ? "SCAN_ANCHOR" :
      !fs.typeByteRead             ? "READ_TYPE" :
      (fs.frameTypeByte == 0x00)   ? "READ_DUMP(payload/CRC)" :
      (fs.frameTypeByte == 0x01)   ? "READ_PROGRESS(tail)" : "?";
  // requirement 2: only meaningful once we know this frame IS a full dump
  // (type byte read as 0x00) -- otherwise not yet determinable, reported as -1.
  long expectedRemaining =
      (fs.anchorFound && fs.typeByteRead && fs.frameTypeByte == 0x00)
          ? (static_cast<long>(kFullDumpTotalLenForDiag) -
             static_cast<long>(fs.partialBytesReceived))
          : -1;
  DiagLog("[FIFO-DIAG][4.7] attempt=%u outcome=%s fillBaseline=%u parserState=%s "
          "anchorFound=%d typeByteRead=%d frameTypeByte=0x%02X "
          "partialBytesReceived=%lu expectedRemaining=%ld "
          "interByteElapsed=%lums sessionPhase=%d t=%lums",
          (unsigned)(s_result.retryCount + 1), OutcomeName(outcome),
          (unsigned)s_attemptFillBaseline, parserState,
          (int)fs.anchorFound, (int)fs.typeByteRead, (unsigned)fs.frameTypeByte,
          (unsigned long)fs.partialBytesReceived, expectedRemaining,
          (unsigned long)(now - s_lastByteActivityMs),
          (int)s_session.phase, (unsigned long)now);
}

// ----------------------------------------------------------------------------
// [Task 4.8 -- TEMPORARY DIAGNOSTIC ONLY, UART receive-drain verification,
// not a permanent production feature] Reports, for every service tick once
// partialBytesReceived has crossed 3000, exactly how many bytes were
// available on the transport before this tick's parse attempt and how many
// the parser actually consumed.
//
// Deliberately does NOT touch fifo_transport_uart485.cpp/.h or
// fifo_codec.cpp/.h -- this file (fifo_driver.cpp, L3) only ever calls the
// ALREADY-PUBLIC FifoTransport::available() (D-7's frozen L1 contract,
// unchanged) and reads the ALREADY-PUBLIC FrameCodecState::
// partialBytesReceived field (fifo_codec.h's own frozen, caller-visible
// struct, unchanged) -- both from before Task 4.8 existed. No new L1/L2
// entry point, no interception, no wrapping of the transport.
//
// Field derivation (fifo_codec.cpp's ReadDump(), read-only, unmodified,
// confirmed by direct inspection this task): its payload loop calls
// t->read(t->ctx, &b, 1) -- exactly ONE byte requested per individual call
// -- repeatedly, up to FIFO_SERVICE_MAX_BYTES (512, SDS SS7.2 A-3) total
// reads per FrameCodec_Step() invocation, stopping the instant a read()
// call returns 0 (nothing buffered). Every successful 1-byte read
// increments partialBytesReceived by exactly 1. Therefore:
//   - "bytes requested this tick"   = up to 512, one read(...,1) call at a
//                                      time (the codec's own fixed, frozen
//                                      per-call shape -- not separately
//                                      measurable from outside without
//                                      touching that frozen file, so stated
//                                      here as the known architectural cap,
//                                      not fabricated per-call data).
//   - "bytes actually read"/"read() return value(s), summed" this tick
//                                    = partialBytesReceivedAfter -
//                                      partialBytesReceivedBeforeStep
//                                      (exact, not inferred -- every
//                                      increment IS a successful read()
//                                      call's return value of 1).
//   - "driver RX FIFO occupancy"    = the same t->available(t->ctx) query
//                                      HardwareSerial::available() already
//                                      performs against the ESP-IDF UART
//                                      driver's own ring buffer -- logged
//                                      both before and after this tick's
//                                      drain to show occupancy trend.
// ----------------------------------------------------------------------------
static void LogUartDrainSnapshot(FifoTransport* t, FifoSessionOutcome outcome,
                                  uint32_t availableBeforeStep,
                                  uint32_t partialBytesReceivedBeforeStep,
                                  uint32_t now) {
  if (partialBytesReceivedBeforeStep <= 3000) {
    return;  // gate: only once partialBytesReceived has crossed 3000
  }
  uint32_t availableAfterStep = t->available(t->ctx);
  // On a terminal (non-NONE) outcome, FrameCodec_Step() has already called
  // FrameCodecState_Reset() internally (fifo_session.h's own documented
  // "every terminal outcome resets frameState" contract) -- partialBytesReceived
  // in s_session.frameState is therefore already 0 and NOT usable for a
  // delta against partialBytesReceivedBeforeStep. Report that explicitly
  // rather than computing a meaningless negative number.
  bool frameEndedThisTick = (outcome != FifoSessionOutcome::NONE);
  uint32_t partialBytesReceivedAfterStep = s_session.frameState.partialBytesReceived;
  long bytesConsumedThisTick = frameEndedThisTick
      ? -1  // not meaningful -- frameState was reset this tick, see above
      : (static_cast<long>(partialBytesReceivedAfterStep) -
         static_cast<long>(partialBytesReceivedBeforeStep));
  // partialBytesReceivedAfter is rendered as a plain number when meaningful,
  // or -1 (sentinel, matching bytesConsumedThisTick's own convention) when
  // the frame ended this tick and frameState was already reset -- avoids
  // Arduino's String class, which is unavailable in this file's host-test
  // build (test/test_fifo_driver.cpp links this file on a plain host
  // compiler; see FifoDriver_SetDiagLogger()'s own doc comment, Task 4.4).
  long partialBytesReceivedAfterForLog =
      frameEndedThisTick ? -1 : static_cast<long>(partialBytesReceivedAfterStep);
  // Note: "UART available()" and "driver RX FIFO occupancy" are the SAME
  // underlying measurement on this platform -- HardwareSerial::available()
  // (which FifoTransport::available() wraps unchanged) itself queries the
  // ESP-IDF UART driver's ring-buffer occupancy (uart_get_buffered_data_len())
  // -- so rxAvail below answers both requirement 1 and requirement 5 at once
  // rather than fabricating a second, redundant number.
  DiagLog("[FIFO-DIAG][4.8] attempt=%u outcome=%s rxAvail %lu->%lu "
          "reqPerReadCall=1(cap512/tick) consumedThisTick=%ld "
          "partial %lu->%ld%s t=%lums",
          (unsigned)(s_result.retryCount + 1), OutcomeName(outcome),
          (unsigned long)availableBeforeStep, (unsigned long)availableAfterStep,
          bytesConsumedThisTick,
          (unsigned long)partialBytesReceivedBeforeStep,
          partialBytesReceivedAfterForLog,
          frameEndedThisTick ? "(RESET)" : "",
          (unsigned long)now);
}

// [Task 7.6 -- TEMPORARY DIAGNOSTIC ONLY, UART byte accounting, not a
// permanent production feature] Computes exactly how many bytes THIS tick's
// FifoSession_Step() call consumed from the transport (== bytes removed
// from the ring buffer, since t->read() is the only removal path anywhere
// in this codebase, and it is called only from inside ScanAnchor()/
// ReadType()/ReadProgress()/ReadDump() -- fifo_codec.cpp's own structure,
// confirmed Task 6.0/7.4, calls exactly ONE of those four per invocation,
// never two). Uses ONLY already-public FrameCodecState fields plus the
// frozen, already-cited protocol-length constants this investigation has
// already disclosed (PROGRESS_TAIL_LEN=4, FULL_DUMP_TOTAL_LEN=6146,
// MAX_DESYNC_BYTES=64 -- fifo_codec.cpp:118,134,147, unchanged, cited not
// duplicated as a new source of truth).
//
// DISCLOSED PRECISION NOTES:
//   - ScanAnchor sub-phase: exact except for the DESYNC_LIMIT terminal case,
//     where the exact trigger byte count within the final increment is not
//     independently observable from outside the frozen ScanAnchor() loop;
//     the MAX_DESYNC_BYTES bound is used, with a possible +/-1-2 byte
//     margin, disclosed rather than presented as exact.
//   - ReadType/ReadProgress/ReadDump sub-phases: exact, via direct
//     before/after field deltas or the fixed protocol-length constants.
static uint32_t ComputeBytesConsumedThisTick(FifoSessionOutcome outcome,
                                              bool anchorFoundBefore, bool typeByteReadBefore,
                                              bool anchorPartialMatchBefore,
                                              uint8_t frameTypeByteBefore,
                                              uint32_t desyncBefore, uint32_t partialBefore) {
  if (outcome == FifoSessionOutcome::NONE) {
    const FrameCodecState& fs = s_session.frameState;  // valid: NOT reset when outcome==NONE
    if (!anchorFoundBefore) {
      // This tick's call was ScanAnchor().
      uint32_t discardDelta = fs.desyncBytesDiscarded - desyncBefore;
      if (fs.anchorFound) {
        return discardDelta + 2;  // anchor just confirmed -- both anchor bytes consumed,
                                   // never counted in desyncBytesDiscarded (fifo_codec.cpp:206-216)
      }
      if (fs.anchorPartialMatch && !anchorPartialMatchBefore) {
        return discardDelta + 1;  // a new tentative 0x50 candidate is now pending,
                                   // consumed but not yet counted as discarded
      }
      return discardDelta;
    }
    if (!typeByteReadBefore) {
      return fs.typeByteRead ? 1u : 0u;  // ReadType() consumes at most 1 byte
    }
    return fs.partialBytesReceived - partialBefore;  // ReadProgress() or ReadDump()
  }
  // Terminal outcome this tick -- frameState already reset by
  // fifo_session.cpp (FifoSession_Step()'s own documented contract) --
  // infer completion bytes from the BEFORE snapshot + the fixed protocol
  // length for whichever shape was active when this tick began.
  switch (outcome) {
    case FifoSessionOutcome::PROGRESS_UPDATE:
      return 4u - partialBefore;
    case FifoSessionOutcome::DUMP_COMPLETE:
      return 6146u - partialBefore;
    case FifoSessionOutcome::FRAME_REJECTED:  // CRC_MISMATCH, either shape
      return (frameTypeByteBefore == 0x01 ? 4u : 6146u) - partialBefore;
    case FifoSessionOutcome::SESSION_FAILED:
      if (anchorFoundBefore && !typeByteReadBefore) {
        return 1u;  // BAD_TYPE_BYTE -- the one invalid type byte
      }
      return 64u - desyncBefore;  // DESYNC_LIMIT -- see disclosed precision note above
    default:
      return 0u;
  }
}

// [Task 8.1 -- TEMPORARY DIAGNOSTIC ONLY, pre-ReadDump transition trace, not
// a permanent production feature] Mirrors fifo_codec.cpp's own private
// FIFO_SERVICE_MAX_BYTES (512, unchanged, not duplicated as a new source of
// truth) purely so this file's own reason-classification below can name the
// "iteration limit" case without reaching into that frozen file -- same
// pattern as kFullDumpTotalLenForDiag above.
static const uint32_t kFifoServiceMaxBytesForDiag = 512;

// [Task 8.1 -- TEMPORARY DIAGNOSTIC ONLY] Per-tick trace from the last
// PROGRESS frame through ScanAnchor()/ReadType() up to and beyond the tick
// ReadDump() is first entered. Gated on the same s_attemptFillBaseline > 5500
// threshold Task 4.7 already established (this investigation's own observed
// last-progress values, ~6063-6075/6144, are comfortably above it), so this
// fires for exactly the requested window without introducing a new gate
// concept. Read-only: reuses ComputeBytesConsumedThisTick() (already called
// once per tick elsewhere in this file for Task 7.6's own byte accounting;
// calling it a second time here is safe -- it is a pure function of already-
// captured BEFORE-snapshots plus the current, unmodified frameState) and the
// existing FifoUartDiagHooks pull-hook (nullptr-safe, already used by the
// existing UART DIAGNOSTICS block). Does not alter s_state, s_session, or
// any timeout/retry decision -- diagnostic output only.
static void LogPreDumpTrace(FifoSessionOutcome outcome, bool anchorFoundBeforeStep,
                             bool typeByteReadBeforeStep, bool anchorPartialMatchBeforeStep,
                             uint8_t frameTypeByteBeforeStep, uint32_t desyncBytesDiscardedBeforeStep,
                             uint32_t partialBytesReceivedBeforeStep,
                             uint32_t availableBeforeStep, uint32_t availableAfterStep,
                             uint32_t now) {
  if (s_attemptFillBaseline <= 5500) {
    return;  // gate: last-progress-onward only, matching Task 4.7's own threshold
  }
  uint32_t consumedThisTick = ComputeBytesConsumedThisTick(
      outcome, anchorFoundBeforeStep, typeByteReadBeforeStep,
      anchorPartialMatchBeforeStep, frameTypeByteBeforeStep,
      desyncBytesDiscardedBeforeStep, partialBytesReceivedBeforeStep);

  // Post-call frameState is only meaningful when outcome==NONE (a terminal
  // outcome this tick means Session Controller already reset frameState --
  // same disclosed limitation LogReadDumpTiming()/LogUartDrainSnapshot()
  // above already document for this exact situation).
  bool anchorFoundAfter  = (outcome == FifoSessionOutcome::NONE) ? s_session.frameState.anchorFound  : false;
  bool typeByteReadAfter = (outcome == FifoSessionOutcome::NONE) ? s_session.frameState.typeByteRead : false;

  const char* reason;
  if (outcome != FifoSessionOutcome::NONE) {
    reason = "FRAME_COMPLETE";
  } else if (!anchorFoundBeforeStep && anchorFoundAfter) {
    reason = "ANCHOR_FOUND_STOP";       // ScanAnchor()'s own "stop immediately, do not
                                          // consume past the anchor" return (fifo_codec.cpp:216)
  } else if (anchorFoundBeforeStep && !typeByteReadBeforeStep && typeByteReadAfter) {
    reason = "TYPE_BYTE_READ_STOP";      // ReadType()'s own "at most one byte, never loops"
                                          // return (fifo_codec.cpp:278)
  } else if (consumedThisTick >= kFifoServiceMaxBytesForDiag) {
    reason = "ITERATION_LIMIT";          // ReadDump()'s FIFO_SERVICE_MAX_BYTES cap
                                          // (fifo_codec.cpp:426/450)
  } else if (availableAfterStep == 0) {
    reason = "NO_BYTES";
  } else {
    reason = "PARTIAL";
  }

  uint32_t fifoOvf = 0, bufferFull = 0, breakCnt = 0, frameErr = 0, parityErr = 0;
  if (s_uartDiagHooks.getStats) {
    s_uartDiagHooks.getStats(&fifoOvf, &bufferFull, &breakCnt, &frameErr, &parityErr);
  }

  DiagLog("[FIFO-DIAG][8.1] attempt=%u t=%lums state=%s anchorFound=%d typeByteRead=%d "
          "frameType=0x%02X availBefore=%lu bytesRead=%lu availAfter=%lu "
          "FIFO_OVF=%lu BUFFER_FULL=%lu reason=%s",
          (unsigned)(s_result.retryCount + 1), (unsigned long)now, StateName(s_state),
          (int)anchorFoundBeforeStep, (int)typeByteReadBeforeStep,
          (unsigned)frameTypeByteBeforeStep,
          (unsigned long)availableBeforeStep, (unsigned long)consumedThisTick,
          (unsigned long)availableAfterStep,
          (unsigned long)fifoOvf, (unsigned long)bufferFull, reason);
}

// [Task 7.4 -- TEMPORARY DIAGNOSTIC ONLY, ReadDump timing audit, not a
// permanent production feature] Per-invocation timing/byte-accounting line,
// printed for EVERY tick this driver is in the ReadDump payload/CRC
// sub-phase (requirement: "For every ReadDump() invocation").
//
// DISCLOSED MEASUREMENT LIMITATION: this file (fifo_driver.cpp, L3) does
// not call ReadDump() directly and does not modify fifo_codec.cpp/
// fifo_session.cpp (both remain frozen, per this whole investigation
// series' established discipline) -- ReadDump() is a `static` function
// private to fifo_codec.cpp, reachable only through FrameCodec_Step(),
// itself reachable only through FifoSession_Step() (fifo_session.cpp:34-46).
// What IS measured and reported below is therefore the wall-clock duration
// of THIS file's own call to FifoSession_Step() -- the closest externally
// observable proxy for "FrameCodec_Step() duration" (one call layer
// outward), timestamped at microsecond resolution via the Task 7.4
// NowMicrosForDiag() pull-hook. The "ReadDump invocation" gate below
// (anchorFound && typeByteRead && frameTypeByte==0x00, evaluated on the
// POST-call frameState) is the same externally-visible-field combination
// Task 4.7's WHY_NOT_FULL_DUMP already established as proof the driver is
// in the payload/CRC sub-phase this tick -- a reconstruction from public
// state, not a literal function-boundary intercept.
// [Task 7.5 -- TEMPORARY DIAGNOSTIC ONLY] Prints one T-stage line, computing
// delta_ms from the immediately-preceding captured stage (0 for T0, which
// has none). `rxAvail` is passed as -1 (sentinel, printed as "n/a") for T0,
// which this task's own requirement list does not ask for an rxAvail
// reading at.
static void LogS8LatencyStage(const char* stageName, uint32_t nowUs, uint32_t prevUs,
                               bool havePrev, long rxAvail) {
  char rxAvailBuf[16];
  if (rxAvail < 0) {
    rxAvailBuf[0] = 'n'; rxAvailBuf[1] = '/'; rxAvailBuf[2] = 'a'; rxAvailBuf[3] = '\0';
  } else {
    snprintf(rxAvailBuf, sizeof(rxAvailBuf), "%ld", rxAvail);
  }
  DiagLog("[FIFO-DIAG][7.5] attempt=%u %s delta_ms=%lu rxAvail=%s tUs=%lu",
          (unsigned)(s_result.retryCount + 1), stageName,
          havePrev ? (unsigned long)((nowUs - prevUs) / 1000) : 0UL,
          rxAvailBuf, (unsigned long)nowUs);
}

// [Task 7.6 -- TEMPORARY DIAGNOSTIC ONLY] Prints one per-cycle byte-
// accounting summary row and resets s_byteAcct's ACCUMULATORS -- caller
// (HandleS3Request()/HandleS9Verify()) decides when a cycle has ended.
// "bytesRemoved" and "bytesConsumedByParser" are the SAME quantity, by
// construction (see ComputeBytesConsumedThisTick()'s own doc comment: the
// parser's sub-functions are the only callers of t->read() anywhere in this
// codebase) -- both printed, under their own requested labels, rather than
// silently collapsing them into one field.
static void LogByteAcctCycleSummary(const char* reason) {
  if (!s_byteAcct.haveData) {
    return;  // no ticks observed in this cycle yet -- nothing to report
  }
  DiagLog("[FIFO-DIAG][7.6] attempt=%u CYCLE_END(%s) bytesEntered=%lu "
          "bytesRemovedFromRingBuffer=%lu bytesConsumedByParser=%lu rxAvailRemaining=%lu",
          (unsigned)(s_result.retryCount + 1), reason,
          (unsigned long)s_byteAcct.bytesEntered,
          (unsigned long)s_byteAcct.bytesConsumed,
          (unsigned long)s_byteAcct.bytesConsumed,
          (unsigned long)s_byteAcct.rxAvailLast);
}

static void LogReadDumpTiming(FifoSessionOutcome outcome, uint32_t stepDurationUs,
                               uint32_t availableBeforeStep, uint32_t availableAfterStep,
                               uint32_t partialBytesReceivedBeforeStep, uint32_t nowMs,
                               uint32_t nowUs) {
  bool frameEndedThisTick = (outcome != FifoSessionOutcome::NONE);
  const FrameCodecState& fs = s_session.frameState;
  // Only meaningful (frameState not already RESET by a terminal outcome
  // this tick) when outcome==NONE -- see LogUartDrainSnapshot()'s own,
  // identical reasoning (Task 4.8) for why a terminal-outcome tick's
  // post-call frameState cannot be used for this classification.
  bool inReadDumpSubphase = !frameEndedThisTick &&
      fs.anchorFound && fs.typeByteRead && fs.frameTypeByte == 0x00;
  if (!inReadDumpSubphase) {
    return;  // not a ReadDump()-attributable tick this call
  }
  long bytesRead = static_cast<long>(fs.partialBytesReceived) -
                    static_cast<long>(partialBytesReceivedBeforeStep);
  DiagLog("[FIFO-DIAG][7.4] ReadDump attempt=%u t=%lums rxAvailBefore=%lu "
          "bytesRequested<=512(1/call) bytesRead=%ld timeSpentUs=%lu rxAvailAfter=%lu "
          "partialBytesReceived=%lu",
          (unsigned)(s_result.retryCount + 1), (unsigned long)nowMs,
          (unsigned long)availableBeforeStep, bytesRead, (unsigned long)stepDurationUs,
          (unsigned long)availableAfterStep, (unsigned long)fs.partialBytesReceived);

  // [Task 7.5] T4 -- first ReadDump()-attributable invocation this request
  // cycle, regardless of whether it actually drained any bytes yet.
  if (!s_s8Lat.t4Captured) {
    s_s8Lat.t4Captured = true;
    s_s8Lat.t4Us = nowUs;
    LogS8LatencyStage("T4_FirstReadDumpInvocation",
                       s_s8Lat.t4Us, s_s8Lat.t3Us, s_s8Lat.t3Captured,
                       static_cast<long>(availableBeforeStep));
  }
  // [Task 7.5] T5 -- first tick this request cycle where a ReadDump()-
  // attributable invocation actually drained >0 bytes (partialBytesReceived
  // increased) -- may be the SAME tick as T4, or a later one (Task 7.4's
  // own captured evidence showed both patterns occur).
  if (!s_s8Lat.t5Captured && bytesRead > 0) {
    s_s8Lat.t5Captured = true;
    s_s8Lat.t5Us = nowUs;
    LogS8LatencyStage("T5_FirstByteDrained",
                       s_s8Lat.t5Us, s_s8Lat.t4Us, s_s8Lat.t4Captured,
                       static_cast<long>(availableAfterStep));
  }
}

static void HandleReceivingImpl(FifoTransport* t) {
  uint32_t now = t->nowMs(t->ctx);
  // [Task 4.8 -- TEMPORARY DIAGNOSTIC ONLY] snapshot BEFORE this tick's
  // FifoSession_Step()/FrameCodec_Step() call drains anything, reusing this
  // single t->available(t->ctx) call for both the existing Task 3.5
  // stall-clock check below AND the Task 4.8 UART-drain trace -- avoids a
  // second, redundant transport call.
  uint32_t availableBeforeStep = t->available(t->ctx);
  if (availableBeforeStep > 0) {
    s_lastByteActivityMs = now;  // [Task 3.5] transport-level activity only
  }
  uint32_t partialBytesReceivedBeforeStep = s_session.frameState.partialBytesReceived;
  // [Task 7.6 -- TEMPORARY DIAGNOSTIC ONLY] BEFORE-snapshots of every
  // FrameCodecState field ComputeBytesConsumedThisTick() needs -- see that
  // function's own doc comment for exactly how each is used.
  bool     anchorFoundBeforeStep       = s_session.frameState.anchorFound;
  bool     typeByteReadBeforeStep      = s_session.frameState.typeByteRead;
  bool     anchorPartialMatchBeforeStep = s_session.frameState.anchorPartialMatch;
  uint8_t  frameTypeByteBeforeStep     = s_session.frameState.frameTypeByte;
  uint32_t desyncBytesDiscardedBeforeStep = s_session.frameState.desyncBytesDiscarded;
  if (!s_byteAcct.haveData) {
    s_byteAcct.haveData = true;
    s_byteAcct.rxAvailAtStart = availableBeforeStep;
  }

  // [Task 7.5 -- TEMPORARY DIAGNOSTIC ONLY] T1 -- first tick THIS REQUEST
  // CYCLE where any byte is observed sitting in the ring buffer (the
  // earliest externally-visible signal available without instrumenting the
  // frozen transport/codec layers directly -- disclosed as tick-cadence-
  // granularity, not the literal ISR-level arrival instant).
  if (!s_s8Lat.t1Captured && availableBeforeStep > 0) {
    s_s8Lat.t1Captured = true;
    s_s8Lat.t1Us = NowMicrosForDiag();
    LogS8LatencyStage("T1_FirstRxByteObserved", s_s8Lat.t1Us, s_s8Lat.t0Us,
                       s_s8Lat.t0Captured, static_cast<long>(availableBeforeStep));
  }

  int16_t* xOut = FifoArena_WriteHandleX();
  int16_t* yOut = FifoArena_WriteHandleY();
  int16_t* zOut = FifoArena_WriteHandleZ();

  // [Task 7.4 -- TEMPORARY DIAGNOSTIC ONLY] microsecond-resolution timing
  // around the FifoSession_Step() call -- see LogReadDumpTiming()'s own doc
  // comment above for exactly what this measures and why.
  uint32_t stepT0Us = NowMicrosForDiag();
  FifoSessionOutcome outcome = FifoSession_Step(&s_session, t, xOut, yOut, zOut);
  uint32_t stepT1Us = NowMicrosForDiag();
  uint32_t availableAfterStep = static_cast<uint32_t>(t->available(t->ctx));

  // [Task 7.6 -- TEMPORARY DIAGNOSTIC ONLY] accumulate this tick's byte
  // accounting into the current cycle's running totals.
  {
    uint32_t consumedThisTick = ComputeBytesConsumedThisTick(
        outcome, anchorFoundBeforeStep, typeByteReadBeforeStep,
        anchorPartialMatchBeforeStep, frameTypeByteBeforeStep,
        desyncBytesDiscardedBeforeStep, partialBytesReceivedBeforeStep);
    // Conservation: entered = (after - before) + consumed. Signed
    // intermediate to avoid uint32 underflow if after < before (net
    // decrease -- entirely possible/expected when consumption exceeds
    // arrival this tick).
    long enteredThisTick = (static_cast<long>(availableAfterStep) -
                             static_cast<long>(availableBeforeStep)) +
                            static_cast<long>(consumedThisTick);
    if (enteredThisTick > 0) {
      s_byteAcct.bytesEntered += static_cast<uint32_t>(enteredThisTick);
    }
    s_byteAcct.bytesConsumed += consumedThisTick;
    s_byteAcct.rxAvailLast = availableAfterStep;
  }

  // [Task 7.5 -- TEMPORARY DIAGNOSTIC ONLY] T2 -- first tick THIS REQUEST
  // CYCLE where the anchor is observed found (post-call frameState; only
  // meaningful when the frame did not also terminate this same tick).
  if (!s_s8Lat.t2Captured && outcome == FifoSessionOutcome::NONE &&
      s_session.frameState.anchorFound) {
    s_s8Lat.t2Captured = true;
    s_s8Lat.t2Us = stepT1Us;
    LogS8LatencyStage("T2_AnchorDetected", s_s8Lat.t2Us, s_s8Lat.t1Us,
                       s_s8Lat.t1Captured, static_cast<long>(availableAfterStep));
  }

  // [Task 4.7 -- TEMPORARY DIAGNOSTIC ONLY] requirement 1: every received
  // frame (i.e. every tick this function runs, since FrameCodec_Step() is
  // called unconditionally above) once fill has crossed 5500 this attempt.
  LogNearEndSnapshot(outcome, now);

  // [Task 4.8 -- TEMPORARY DIAGNOSTIC ONLY] UART receive-drain trace, gated
  // on partialBytesReceived > 3000 (requirement: "every service tick after
  // partialBytesReceived > 3000").
  LogUartDrainSnapshot(t, outcome, availableBeforeStep,
                        partialBytesReceivedBeforeStep, now);

  // [Task 7.4 -- TEMPORARY DIAGNOSTIC ONLY] per-invocation ReadDump timing,
  // ungated (ALL ticks in the payload/CRC sub-phase, not just late ones).
  LogReadDumpTiming(outcome, stepT1Us - stepT0Us, availableBeforeStep,
                     availableAfterStep, partialBytesReceivedBeforeStep, now, stepT1Us);

  // [Task 8.1 -- TEMPORARY DIAGNOSTIC ONLY] pre-ReadDump transition trace --
  // see LogPreDumpTrace()'s own doc comment for gating/scope.
  LogPreDumpTrace(outcome, anchorFoundBeforeStep, typeByteReadBeforeStep,
                   anchorPartialMatchBeforeStep, frameTypeByteBeforeStep,
                   desyncBytesDiscardedBeforeStep, partialBytesReceivedBeforeStep,
                   availableBeforeStep, availableAfterStep, now);

  switch (outcome) {
    case FifoSessionOutcome::NONE: {
      // [Task 3.5] Still pending -- apply the applicable stall budget
      // before reflecting Session Controller's phase at this driver's
      // coarser granularity.
      bool preAnchor = (s_session.phase == FifoSessionPhase::WAIT_FRAME);
      uint32_t budget = preAnchor ? T_REQUEST_RESPONSE_MS : T_INTER_BYTE_MS;
      if (now - s_lastByteActivityMs > budget) {
        s_result.error = preAnchor ? FifoError::ERR_NO_RESPONSE
                                    : FifoError::ERR_INTER_BYTE_TIMEOUT;
        // [Task 4.4 -- TEMPORARY DIAGNOSTIC ONLY] "timeout reason"
        DiagLog("[FIFO-DIAG] attempt=%u TIMEOUT reason=%s budget=%lums elapsed=%lums t=%lums",
                (unsigned)(s_result.retryCount + 1),
                preAnchor ? "NO_RESPONSE(pre-anchor)" : "INTER_BYTE(post-anchor)",
                (unsigned long)budget, (unsigned long)(now - s_lastByteActivityMs),
                (unsigned long)now);
        // [Task 7.1 -- TEMPORARY DIAGNOSTIC ONLY] requirement 3: print the
        // UART DIAGNOSTICS block specifically when ERR_INTER_BYTE_TIMEOUT is
        // declared (not ERR_NO_RESPONSE -- that's the preAnchor case, gated
        // out below). Pulls the 5 cumulative counters through the
        // FifoUartDiagHooks pull-hook (nullptr-safe -- no-op if never
        // registered); rxAvailLast/partialBytes/lastByteAgeMs come from
        // already-in-scope values/the already-public `t` parameter, no new
        // dependency needed for those three.
        if (!preAnchor && s_uartDiagHooks.getStats) {
          uint32_t fifoOvf = 0, bufferFull = 0, breakCnt = 0, frameErr = 0, parityErr = 0;
          s_uartDiagHooks.getStats(&fifoOvf, &bufferFull, &breakCnt, &frameErr, &parityErr);
          bool hadOverflowNow = t->hadOverflow(t->ctx);
          uint32_t rxAvailLast = static_cast<uint32_t>(t->available(t->ctx));
          DiagLog("-----------------------------------");
          DiagLog("UART DIAGNOSTICS");
          DiagLog("");
          DiagLog("FIFO_OVF      = %lu", (unsigned long)fifoOvf);
          DiagLog("BUFFER_FULL   = %lu", (unsigned long)bufferFull);
          DiagLog("BREAK         = %lu", (unsigned long)breakCnt);
          DiagLog("FRAME_ERR     = %lu", (unsigned long)frameErr);
          DiagLog("PARITY_ERR    = %lu", (unsigned long)parityErr);
          DiagLog("hadOverflow   = %d", (int)hadOverflowNow);
          DiagLog("rxAvailLast   = %lu", (unsigned long)rxAvailLast);
          DiagLog("partialBytes  = %lu",
                  (unsigned long)s_session.frameState.partialBytesReceived);
          DiagLog("lastByteAgeMs = %lu", (unsigned long)(now - s_lastByteActivityMs));
          DiagLog("-----------------------------------");
        }
        // [Task 4.7 -- TEMPORARY DIAGNOSTIC ONLY] requirements 6+7: explicit
        // "why FULL_DUMP was not emitted" verdict, only meaningful once
        // we've crossed into the endgame zone this attempt.
        if (s_attemptFillBaseline > 5500 && !preAnchor) {
          const FrameCodecState& fs = s_session.frameState;
          bool wasReadingDumpFrame =
              fs.anchorFound && fs.typeByteRead && fs.frameTypeByte == 0x00;
          long missingBytes = wasReadingDumpFrame
              ? (static_cast<long>(kFullDumpTotalLenForDiag) -
                 static_cast<long>(fs.partialBytesReceived))
              : -1;
          DiagLog("[FIFO-DIAG][4.7] attempt=%u WHY_NOT_FULL_DUMP: %s -- "
                  "partialBytesReceived=%lu/%lu missingBytes=%ld crcStatus=%s",
                  (unsigned)(s_result.retryCount + 1),
                  wasReadingDumpFrame
                      ? "inter-byte silence exceeded budget mid-dump-frame -- "
                        "the sensor stopped sending bytes before the frame "
                        "(payload+trailing CRC) was fully received"
                      : "inter-byte silence exceeded budget before the type "
                        "byte confirmed this as a FULL_DUMP frame -- unknown "
                        "which shape this response was going to be",
                  (unsigned long)fs.partialBytesReceived,
                  (unsigned long)kFullDumpTotalLenForDiag, missingBytes,
                  wasReadingDumpFrame
                      ? "NOT VERIFIED -- frame incomplete, CRC never reached/checked"
                      : "n/a");
        }
        s_state = FifoState::S9_VERIFY;
        return;
      }
      // [Task 7.5 -- TEMPORARY DIAGNOSTIC ONLY] T3 -- the exact tick
      // s_state is set to S8_READ_DUMP for THIS request cycle (the only
      // site in this file that ever assigns S8_READ_DUMP -- confirmed by
      // Task 6.0's own exhaustive search).
      if (!preAnchor && !s_s8Lat.t3Captured) {
        s_s8Lat.t3Captured = true;
        s_s8Lat.t3Us = stepT1Us;
        LogS8LatencyStage("T3_S8Entered", s_s8Lat.t3Us, s_s8Lat.t2Us,
                           s_s8Lat.t2Captured, static_cast<long>(availableAfterStep));
      }
      s_state = preAnchor ? FifoState::S4_AWAIT_ANCHOR : FifoState::S8_READ_DUMP;
      return;
    }

    case FifoSessionOutcome::PROGRESS_UPDATE: {
      uint16_t fill = s_session.lastProgressFill;
      if (fill > 6144) {
        s_result.error = FifoError::ERR_PROGRESS_OVERRUN;
        s_state = FifoState::S9_VERIFY;
        return;
      }
      if (fill < s_attemptFillBaseline) {
        s_result.error = FifoError::ERR_PROGRESS_REGRESSION;
        s_state = FifoState::S9_VERIFY;
        return;
      }
      s_attemptFillBaseline = fill;
      s_result.progressFrameCount++;
      s_result.lastProgressFill = fill;
      if (s_result.tFirstProgressMs == 0) {
        s_result.tFirstProgressMs = now;
      }
      s_result.tLastProgressMs = now;
      // [Task 4.4 -- TEMPORARY DIAGNOSTIC ONLY] "sensor response"
      DiagLog("[FIFO-DIAG] attempt=%u SENSOR_RESPONSE=PROGRESS fill=%u progressFrameCount=%u t=%lums",
              (unsigned)(s_result.retryCount + 1), (unsigned)fill,
              (unsigned)s_result.progressFrameCount, (unsigned long)now);

#if FIFO_PROTOCOL_MODE == FIFO_PROTOCOL_MODE_TRUEPOLL
      s_pollWaitEnteredAtMs = now;
      s_state = FifoState::S7_POLL_WAIT;
#else
      s_lastByteActivityMs = now;  // [Task 3.5] fresh wait, no new request sent
      s_state = FifoState::S4_AWAIT_ANCHOR;  // PURELISTEN: keep listening, no re-request
#endif
      return;
    }

    case FifoSessionOutcome::DUMP_COMPLETE:
      s_result.error = FifoError::NONE;
      s_result.sampleCount = 1024;
      // [Task 4.4 -- TEMPORARY DIAGNOSTIC ONLY] "sensor response" + "CRC result"
      DiagLog("[FIFO-DIAG] attempt=%u SENSOR_RESPONSE=FULL_DUMP crc=PASS sampleCount=1024 t=%lums",
              (unsigned)(s_result.retryCount + 1), (unsigned long)now);
      s_state = FifoState::S9_VERIFY;
      return;

    case FifoSessionOutcome::FRAME_REJECTED:
      // Session Controller already re-armed itself to WAIT_FRAME -- no
      // attempt-ending transition needed, only diagnostics. [Task 3.5]
      // Fresh stall budget starts now -- no new physical request was
      // sent, but we are waiting for a new anchor in the ongoing stream.
      s_result.crcErrorCount++;
      // [Task 4.4 -- TEMPORARY DIAGNOSTIC ONLY] "sensor response" + "CRC result"
      // -- this is a mid-attempt, non-terminal event (session loops back to
      // WAIT_FRAME on its own); attempt does not end here.
      DiagLog("[FIFO-DIAG] attempt=%u SENSOR_RESPONSE=CRC_MISMATCH crc=FAIL "
              "(non-terminal, re-arming) crcErrorCount=%u t=%lums",
              (unsigned)(s_result.retryCount + 1), (unsigned)s_result.crcErrorCount,
              (unsigned long)now);
      s_lastByteActivityMs = now;
      s_state = FifoState::S4_AWAIT_ANCHOR;
      return;

    case FifoSessionOutcome::SESSION_FAILED:
      s_result.error = (s_session.failReason == FifoSessionFailReason::BAD_FRAME_TYPE)
                            ? FifoError::ERR_BAD_TYPE_BYTE
                            : FifoError::ERR_DESYNC_LIMIT;
      // [Task 4.4 -- TEMPORARY DIAGNOSTIC ONLY] "sensor response" + "failure reason"
      DiagLog("[FIFO-DIAG] attempt=%u SENSOR_RESPONSE=SESSION_FAILED reason=%s "
              "desyncBytesDiscarded=%lu t=%lums",
              (unsigned)(s_result.retryCount + 1),
              (s_session.failReason == FifoSessionFailReason::BAD_FRAME_TYPE)
                  ? "BAD_FRAME_TYPE" : "DESYNC_LIMIT",
              (unsigned long)s_session.frameState.desyncBytesDiscarded,
              (unsigned long)now);
      s_state = FifoState::S9_VERIFY;
      return;
  }
}

// [Task 7.4 -- TEMPORARY DIAGNOSTIC ONLY] Thin timing wrapper around the
// UNMODIFIED HandleReceivingImpl() body above -- measures this whole
// function's wall-clock duration (the literal "HandleReceiving() duration"
// this task asks for, fully accurate since this file owns the entire
// function) via a single entry/exit pair, avoiding the need to duplicate a
// measurement at each of HandleReceivingImpl()'s several `return` sites.
static void HandleReceiving(FifoTransport* t) {
  uint32_t t0Us = NowMicrosForDiag();
  HandleReceivingImpl(t);
  uint32_t t1Us = NowMicrosForDiag();
  DiagLog("[FIFO-DIAG][7.4] HandleReceiving() durationUs=%lu",
          (unsigned long)(t1Us - t0Us));
}

static void HandleS7PollWait(FifoTransport* t) {
  // S7's ONLY exit condition (SDS SS11.1): "T_POLL_INTERVAL elapsed".
  // [Task 3.5] Now the real, owned T_POLL_INTERVAL_MS constant --
  // Task 3.4A's kPollIntervalPlaceholderMs no longer exists in this file.
  uint32_t elapsed = t->nowMs(t->ctx) - s_pollWaitEnteredAtMs;
  if (elapsed >= T_POLL_INTERVAL_MS) {
    // Re-request. The actual write happens in HandleS3Request() on the
    // next tick -- one bounded action per tick, reusing the single
    // "send a request frame" code path rather than duplicating it here.
    s_state = FifoState::S3_REQUEST;
  }
  // else: remain in S7, re-checked next tick.
}

static void HandleS9Verify(FifoTransport* t) {
  // FrameCodec/Session Controller already performed CRC verification
  // before ever returning DUMP_COMPLETE/SESSION_FAILED -- this state's
  // job, under the ADR-0004 layering, is the FifoSessionOutcome ->
  // FifoError translation already performed in HandleReceiving(), plus
  // finalizing the completion timestamp and choosing the terminal state.
  s_result.tCompleteMs = t->nowMs(t->ctx);
  // [Task 4.4 -- TEMPORARY DIAGNOSTIC ONLY] "elapsed time per state" for
  // this whole attempt (request -> verify), plus "failure reason" (NONE on
  // success).
  DiagLog("[FIFO-DIAG] attempt=%u ATTEMPT_END error=%s pollCount=%u progressFrameCount=%u "
          "crcErrorCount=%u attemptElapsed=%lums t=%lums",
          (unsigned)(s_result.retryCount + 1), FifoErrorName(s_result.error),
          (unsigned)s_result.pollCount, (unsigned)s_result.progressFrameCount,
          (unsigned)s_result.crcErrorCount,
          (unsigned long)(s_result.tCompleteMs - s_result.tRequestMs),
          (unsigned long)s_result.tCompleteMs);
  // [Task 7.6 -- TEMPORARY DIAGNOSTIC ONLY] flush the FINAL request cycle's
  // byte-accounting summary -- this attempt is ending here, so there is no
  // "next HandleS3Request()" to trigger it the way earlier PROGRESS-cycle
  // summaries are flushed.
  LogByteAcctCycleSummary(FifoErrorName(s_result.error));
  if (s_result.error == FifoError::NONE) {
    s_drainQuietSinceMs = s_result.tCompleteMs;  // [Task 3.5] starts S10's quiet clock
    s_state = FifoState::S10_DRAIN;
  } else {
    s_state = FifoState::S13_FAILED;
  }
}

// ----------------------------------------------------------------------------
// Per-state handlers, S10-S14 (Task 3.5).
// ----------------------------------------------------------------------------

static void HandleS10Drain(FifoTransport* t) {
  uint32_t now = t->nowMs(t->ctx);
  if (t->available(t->ctx) > 0) {
    // K-14: a FIFO transaction gone wrong cascades into 0xE2 on the next
    // unrelated register read -- drain, don't just ignore, and reset the
    // quiet timer on every byte seen (SS11.2: "exits only on TRUE
    // silence, not elapsed time"). flushRx() is FifoTransport's own
    // named mechanism for exactly this (fifo_transport.h: "Used for
    // bus-settle draining").
    t->flushRx(t->ctx);
    s_drainQuietSinceMs = now;
    return;
  }
  if (now - s_drainQuietSinceMs < T_DRAIN_QUIET_MS) {
    return;  // not yet quiet long enough
  }

  if (s_result.error != FifoError::NONE && s_result.retryCount < FIFO_MAX_RETRIES) {
    // SS17.1 "Attempt level -> RETRY_ATTEMPT: full drain + bus release +
    // fresh attempt". Route through S1 IDLE for exactly one tick (bus
    // not held there) before re-arming -- SS17.3's "release the bus, and
    // allow at least one normal poll tick" -- rather than going straight
    // back to S2, which would never actually release the bus at all.
    // [Task 4.4 -- TEMPORARY DIAGNOSTIC ONLY] "difference between attempts"
    // input -- logs the just-finished attempt's outcome and that a retry
    // will happen, BEFORE retryCount is incremented (so this line reports
    // the attempt that just ended, not the upcoming one).
    DiagLog("[FIFO-DIAG] attempt=%u RETRY_DECISION lastError=%s retryCount=%u/%u willRetry=1 t=%lums",
            (unsigned)(s_result.retryCount + 1), FifoErrorName(s_result.error),
            (unsigned)s_result.retryCount, (unsigned)FIFO_MAX_RETRIES, (unsigned long)now);
    s_result.retryCount++;
    s_result.error = FifoError::NONE;  // fresh attempt starts clean
    s_retryPending = true;
    s_state = FifoState::S1_IDLE;
    return;
  }

  // Either success, or SS17.1 "Session level -> FAIL: terminal, result
  // published with diagnostics" (retries exhausted). The session's final
  // outcome is now known -- update SS17.6's consecutive-failure counter
  // here, the one point that definitively knows it. The actual S14
  // transition (if tripped) is deferred to S12 COOLDOWN's own exit, so
  // SS17.4's "cooldown after ANY session, success or failure" still
  // applies even to the session that trips the breaker.
  if (s_result.error == FifoError::NONE) {
    s_consecutiveFailedSessions = 0;
  } else if (s_consecutiveFailedSessions < 255) {
    s_consecutiveFailedSessions++;  // saturate, never wrap
  }
  // [Task 4.4 -- TEMPORARY DIAGNOSTIC ONLY] Session-level terminal outcome
  // -- either success (this branch is reached with error==NONE only when
  // the LAST attempt in the sequence succeeded) or retries exhausted.
  DiagLog("[FIFO-DIAG] SESSION_END finalError=%s totalAttempts=%u consecutiveFailedSessions=%u t=%lums",
          FifoErrorName(s_result.error), (unsigned)(s_result.retryCount + 1),
          (unsigned)s_consecutiveFailedSessions, (unsigned long)now);
  s_wasAcquiredThisHold = false;
  s_state = FifoState::S11_RESULT_READY;
}

static void HandleS11ResultReady(FifoTransport* t) {
  // SS9.1: "no" (bus not held) here -- no FifoTransport interaction at
  // all, only FifoArena (Task 3.3/3.3A). This driver never calls
  // FifoArena_TryAcquire() itself -- that remains the future
  // FifoDriver_TryAcquireResult() public wrapper's job (SS9.1's "exactly
  // one call site", Task 3.6); this state only WATCHES ownership.
  if (FifoArena_IsOwned()) {
    // [Task 3.7 defect fix] s_wasAcquiredThisHold is now set synchronously
    // by FifoDriver_TryAcquireResult() itself, not observed here by
    // polling -- a consumer that acquires AND releases faster than one
    // Service() tick would otherwise never be seen "held" by this poll,
    // leaving s_wasAcquiredThisHold permanently false and the driver
    // stuck in S11 forever (the "never yet acquired" branch below has no
    // timeout bound, by design -- SS17.5 only bounds an ALREADY-observed
    // hold). Discovered while tracing Task 3.7's end-to-end test
    // tick-by-tick; see the delivered Integration Report.
    uint32_t heldMs = FifoArena_HeldSinceMs();
    if (heldMs > T_RESULT_HOLD_MAX_MS) {
      // SS17.5: "the driver logs ERR_RESULT_NOT_RELEASED, reclaims
      // ownership, and returns to IDLE." Reclaim is a plain
      // FifoArena_Release() call -- the arena tracks THAT it is held,
      // never WHO holds it, so releasing unconditionally is exactly its
      // documented "prevent release without ownership" no-op-if-already-
      // free contract, used here deliberately for its force-reclaim
      // side effect. "Logs" is satisfied at the data level only (this
      // file implements no logging framework, per instruction) --
      // s_result.error is set so a later task's telemetry/log call has
      // something to report.
      FifoArena_Release();
      s_result.error = FifoError::ERR_RESULT_NOT_RELEASED;
      s_wasAcquiredThisHold = false;
      // [SDS tension, resolved by following the more specific text]
      // SS17.4 frames cooldown as applying "after any session, success
      // or failure", which reads as if it should apply here too. SS17.5
      // is more specific and explicit for this exact scenario --
      // "returns to IDLE" -- and is followed literally rather than
      // inferring a cooldown detour SS17.5's own text does not mention.
      //
      // [Task 3.6] Still stamp s_cooldownEnteredAtMs here, even though
      // the state machine itself skips S12 -- SS19.2's admission gate
      // "Cooldown elapsed" check reuses this SAME timestamp (rather than
      // a second, duplicate one) to independently guard against a new
      // request being admitted immediately after a watchdog reclaim,
      // which this state-machine shortcut would otherwise allow.
      s_cooldownEnteredAtMs = t->nowMs(t->ctx);
      s_state = FifoState::S1_IDLE;
      return;
    }
    return;  // still legitimately held, within budget -- keep waiting
  }

  if (s_wasAcquiredThisHold) {
    // Was acquired, now released -- normal completion (SS9.1: "(A-7) ->
    // driver").
    s_wasAcquiredThisHold = false;
    s_cooldownEnteredAtMs = t->nowMs(t->ctx);  // stamped here, read by HandleS12Cooldown()
    s_state = FifoState::S12_COOLDOWN;
    return;
  }
  // Never yet acquired -- keep waiting. The frozen SDS names a
  // hold-TOO-LONG watchdog (SS17.5), not a never-acquired-at-all bound;
  // none is fabricated here.
}

static void HandleS12Cooldown(FifoTransport* t) {
  uint32_t elapsed = t->nowMs(t->ctx) - s_cooldownEnteredAtMs;
  if (elapsed < T_COOLDOWN_MS) {
    return;
  }
  if (s_consecutiveFailedSessions >= FIFO_BREAKER_THRESHOLD) {
    // SS17.6: trip the breaker. A single high-priority /event on trip is
    // telemetry publishing -- explicitly not this file's job.
    s_state = FifoState::S14_DISABLED;
    return;
  }
  s_state = FifoState::S1_IDLE;
}

static void HandleS13Failed(FifoTransport* t) {
  // "Terminal failure, result finalised | yes -> drain | -> S10" -- an
  // unconditional, immediate handoff, matching SS11.1 exactly. Unchanged
  // from Task 3.4, except it must also start S10's quiet clock, exactly
  // like HandleS9Verify()'s success branch does -- stamped with a real
  // t->nowMs() reading, not a 0 sentinel (nowMs() is uptime-based and
  // already large by the time this state is ever reached; using 0 would
  // make HandleS10Drain()'s very first elapsed-time check spuriously
  // huge and exit before any real quiet period was observed).
  s_drainQuietSinceMs = t->nowMs(t->ctx);
  s_state = FifoState::S10_DRAIN;
}

static void HandleS14Disabled() {
  // SS17.6: "All requests rejected with ERR_CIRCUIT_OPEN until reboot or
  // an explicit reset." Rejection at admission is Task 3.6's job (no
  // admission gate exists in this file). This handler's only
  // responsibility is to NOT auto-exit -- "Manual reset ensures a human
  // sees the event; auto-reset would let a persistent hardware fault
  // oscillate indefinitely." A future explicit reset entry point (Task
  // 3.6, not this file) is expected to directly set
  // s_state = FifoState::S1_IDLE and s_consecutiveFailedSessions = 0.
}

// ----------------------------------------------------------------------------
// FifoDriver_ServiceCoreState() -- Task 3.4's entry point, S0-S9 only.
// `default: break;` for every state outside this range.
// ----------------------------------------------------------------------------
static void FifoDriver_ServiceCoreState(FifoTransport* t) {
  switch (s_state) {
    case FifoState::S0_UNINIT:
      // No transition implemented here -- entry to S1 happens once a
      // future FifoDriver_Init() (Task 3.6) explicitly sets s_state.
      break;
    case FifoState::S1_IDLE:
      HandleS1Idle();
      break;
    case FifoState::S2_ARMED:
      HandleS2Armed();
      break;
    case FifoState::S3_REQUEST:
      HandleS3Request(t);
      break;
    case FifoState::S4_AWAIT_ANCHOR:
    case FifoState::S5_READ_TYPE:
    case FifoState::S6_READ_PROGRESS:
    case FifoState::S8_READ_DUMP:
      HandleReceiving(t);
      break;
    case FifoState::S7_POLL_WAIT:
      HandleS7PollWait(t);
      break;
    case FifoState::S9_VERIFY:
      HandleS9Verify(t);
      break;
    default:
      break;  // S10-S14: FifoDriver_ServiceRecoveryState()'s territory
  }
}

// ----------------------------------------------------------------------------
// FifoDriver_ServiceRecoveryState() -- Task 3.5's entry point, S10-S14
// only. `default: break;` for every state outside this range. Matches the
// frozen Implementation Plan's own internal-interface name for this task
// exactly: "static void FifoDriver_ServiceRecoveryState(FifoTransport* t);"
// ----------------------------------------------------------------------------
static void FifoDriver_ServiceRecoveryState(FifoTransport* t) {
  switch (s_state) {
    case FifoState::S10_DRAIN:
      HandleS10Drain(t);
      break;
    case FifoState::S11_RESULT_READY:
      HandleS11ResultReady(t);
      break;
    case FifoState::S12_COOLDOWN:
      HandleS12Cooldown(t);
      break;
    case FifoState::S13_FAILED:
      HandleS13Failed(t);
      break;
    case FifoState::S14_DISABLED:
      HandleS14Disabled();
      break;
    default:
      break;  // S0-S9: FifoDriver_ServiceCoreState()'s territory
  }
}

// ============================================================================
// [Task 3.6] PUBLIC API -- thin wrappers only. See fifo_driver.h for the
// full contract of each function. No protocol parsing, no retry
// decisions, no timeout decisions, and no transport logic live here --
// every one of those remains owned by the internal handlers above
// (Tasks 3.1/3.3/3.4/3.5), preserving ADR-0004's layering exactly.
// ============================================================================

void FifoDriver_Init(FifoTransport* transport) {
  s_transport = transport;

  FifoArena_Init();  // Task 3.3 -- wrapper call, not new logic

  s_state = FifoState::S1_IDLE;
  FifoSession_Init(&s_session);  // Task 3.1 -- wrapper call; already zeroes
                                  // frameState/phase/failReason/lastProgressFill

  s_requestPending = false;
  s_pendingRequest = FifoCaptureRequest{};
  s_retryPending = false;
  s_result = FifoCaptureResult{};
  s_captureIdCounter = 0;
  s_pendingCaptureId = 0;
  s_attemptFillBaseline = 0;
  s_pollWaitEnteredAtMs = 0;
  s_lastByteActivityMs = 0;
  s_drainQuietSinceMs = 0;
  s_wasAcquiredThisHold = false;
  s_cooldownEnteredAtMs = 0;
  s_consecutiveFailedSessions = 0;
  s_stateEnteredAtMs = 0;  // [Task 4.4 -- TEMPORARY DIAGNOSTIC ONLY]
}

void FifoDriver_Service() {
  if (!s_transport) {
    return;  // parameter validation: Init() was never called
  }
  // [Task 4.4 -- TEMPORARY DIAGNOSTIC ONLY] "state transitions" + "elapsed
  // time per state" -- generic, single call site: compares s_state before
  // and after this tick's two service calls (unmodified below) and, on a
  // change, logs the transition plus how long the PREVIOUS state was
  // occupied. Purely observational -- runs after the real state machine has
  // already decided everything; reads s_state, never writes it.
  FifoState before = s_state;
  FifoDriver_ServiceCoreState(s_transport);
  FifoDriver_ServiceRecoveryState(s_transport);
  if (s_state != before) {
    uint32_t now = s_transport->nowMs(s_transport->ctx);
    DiagLog("[FIFO-DIAG] STATE %s -> %s elapsedInPrev=%lums t=%lums",
            StateName(before), StateName(s_state),
            (unsigned long)(now - s_stateEnteredAtMs), (unsigned long)now);
    s_stateEnteredAtMs = now;
  }
}

FifoError FifoDriver_Request(const FifoCaptureRequest* req, uint32_t* outHandle) {
  if (!req) {
    return FifoError::ERR_NOT_PERMITTED;
  }

  // SS19.2 admission gates, in the frozen table's own order. The four
  // driver-internal gates map to their own SS16.1-taxonomy codes where
  // one exists (matching D-18's "one code per distinct failure", not the
  // table header's literal "else ERR_NOT_PERMITTED" read as uniform --
  // SS16.1/SS17.6 already assign ERR_CIRCUIT_OPEN and ERR_BUSY/
  // ERR_RESULT_NOT_RELEASED their own dedicated codes). The three
  // application-state gates have no dedicated code and use the generic
  // ERR_NOT_PERMITTED, exactly as SS19.2's table states.
  //
  // [Interpretive decision, disclosed] req->requirePermissive == false is
  // implemented as bypassing ONLY the three application-state gates
  // (motor/sensor/network), never the four driver-internal structural
  // ones -- those protect real invariants (no concurrent capture, no
  // defeating a tripped breaker, no admitting during an unexpired
  // cooldown, no racing an unreleased result) that cannot be safely
  // bypassed by a caller flag. The frozen text's "(unless
  // requirePermissive == false)" does not explicitly scope which gates
  // it exempts; this is the more conservative, invariant-preserving
  // reading, not the only grammatically possible one.

  // Gate: driver idle (S1 IDLE) -- also excludes the one-tick retry-
  // pending window (Task 3.5's s_retryPending), closing the exact race
  // flagged as remaining work in that task's own delivery.
  if (s_state != FifoState::S1_IDLE || s_retryPending) {
    return FifoError::ERR_BUSY;
  }

  // Gate: circuit closed (not S14) -- redundant with the S1-IDLE check
  // above given this implementation (S14 is never S1), kept as its own
  // explicit check to match the frozen table's own separate listing and
  // stay correct even if that relationship ever changes.
  if (s_state == FifoState::S14_DISABLED) {
    return FifoError::ERR_CIRCUIT_OPEN;
  }

  // Gate: cooldown elapsed (>= T_COOLDOWN_MS since the last session
  // concluded). NOT redundant with "driver idle" -- SS17.5's watchdog-
  // reclaim path reaches S1 directly, bypassing S12's own wait, so this
  // independently guards that specific shortcut using the same
  // s_cooldownEnteredAtMs timestamp S12 and the watchdog both already
  // stamp (no duplicate timeout ownership introduced).
  if (s_transport) {
    uint32_t sinceLastSession = s_transport->nowMs(s_transport->ctx) - s_cooldownEnteredAtMs;
    if (s_cooldownEnteredAtMs != 0 && sinceLastSession < T_COOLDOWN_MS) {
      return FifoError::ERR_BUSY;
    }
  }

  // Gate: no result held (SS9 handoff complete).
  if (FifoArena_IsOwned()) {
    return FifoError::ERR_RESULT_NOT_RELEASED;
  }

  // Gates: application state (bypassed when requirePermissive == false).
  if (req->requirePermissive) {
    if (!req->admissionContext.motorStable) {
      return FifoError::ERR_NOT_PERMITTED;
    }
    if (!req->admissionContext.sensorHealthy) {
      return FifoError::ERR_NOT_PERMITTED;
    }
    if (req->admissionContext.mqttReconnecting) {  // inverted, per SS19.2
      return FifoError::ERR_NOT_PERMITTED;
    }
  }

  // Admitted. captureId assigned synchronously here (not by
  // HandleS1Idle(), which runs on the next tick) so a handle can be
  // returned to the caller immediately.
  s_captureIdCounter++;
  s_pendingCaptureId = s_captureIdCounter;
  s_pendingRequest = *req;
  s_requestPending = true;
  if (outHandle) {
    *outHandle = s_pendingCaptureId;
  }
  return FifoError::NONE;
}

FifoPhase FifoDriver_GetPhase() {
  return CurrentPhase();
}

bool FifoDriver_TryAcquireResult(FifoCaptureResult* outResult) {
  if (!outResult) {
    return false;
  }
  if (s_state != FifoState::S11_RESULT_READY) {
    return false;  // "none ready" -- checked BEFORE touching the arena,
                    // since the arena alone cannot distinguish "driver
                    // hasn't finished this capture yet" from "already
                    // released and idle"
  }

  const int16_t* x;
  const int16_t* y;
  const int16_t* z;
  if (!FifoArena_TryAcquire(&x, &y, &z)) {
    return false;  // "already held"
  }
  // [Task 3.7 defect fix] Set synchronously, here, at the exact moment
  // acquisition succeeds -- not polled by HandleS11ResultReady() anymore.
  // See that function's own updated comment for the race this closes.
  s_wasAcquiredThisHold = true;

  // [Thread-safety -- see delivered review] This copies s_result, which
  // this driver's OWN code (Task 3.4/3.5 handlers) only ever writes from
  // FifoDriver_Service()'s call chain, fully settling every field BEFORE
  // s_state ever transitions to S11_RESULT_READY (program-order-safe
  // within that single call chain). SDS SS7.1 A-6 names "one narrow
  // mutex (mutexFifoResult) held only for the pointer-exchange in
  // TryAcquireResult/ReleaseResult" as this driver's one new
  // synchronization primitive -- NOT implemented here. If this
  // function's actual call site ever runs on a different task/core than
  // FifoDriver_Service() (SDS SS9.1 does not pin this down explicitly),
  // this copy has no memory-visibility guarantee without that mutex.
  // Flagged, not silently assumed safe.
  *outResult = s_result;
  outResult->x = x;
  outResult->y = y;
  outResult->z = z;
  return true;
}

void FifoDriver_ReleaseResult() {
  FifoArena_Release();
}

void FifoDriver_Abort(FifoError reason) {
  (void)reason;  // accepted for a future diagnostics task's use; not
                 // stored -- SS16.1's own taxonomy fixes Abort()'s result
                 // to ERR_ABORTED regardless of the caller-supplied reason
  if (!IsCaptureInProgress()) {
    return;  // nothing active to abort
  }
  s_result.error = FifoError::ERR_ABORTED;
  // Do not retry an explicit abort -- supply data the driver's OWN
  // existing retry decision (HandleS10Drain(), Task 3.5) already
  // respects, rather than this function deciding not to retry itself.
  s_result.retryCount = FIFO_MAX_RETRIES;
  // Route directly to S13 FAILED -- S9's own job (translating a
  // FifoSessionOutcome into a FifoError) does not apply here, since this
  // function supplies the error directly; S13's existing, unchanged
  // handler still performs the mandatory drain via S10 ("driver still
  // performs its bus drain", SDS SS7.1 A-8).
  s_state = FifoState::S13_FAILED;
}

void FifoDriver_GetStats(FifoDriverStats* outStats) {
  if (!outStats) {
    return;
  }
  // FifoDriverStats has no fields (deliberately deferred since Task 3.2:
  // "finalized at coding time", never resolved) -- nothing to copy.
  *outStats = FifoDriverStats{};
}

void FifoDriver_SetDiagLogger(void (*fn)(const char* msg)) {
  s_diagLog = fn;
}

const char* FifoDriver_GetInternalStateNameForDiag() {
  return StateName(s_state);
}

uint32_t FifoDriver_GetAttemptNumberForDiag() {
  return static_cast<uint32_t>(s_result.retryCount) + 1;
}

void FifoDriver_SetUartDiagHooks(FifoUartDiagHooks hooks) {
  s_uartDiagHooks = hooks;
}

void FifoDriver_SetMicrosProvider(uint32_t (*fn)()) {
  s_microsProvider = fn;
}

bool FifoDriver_ResetCircuitBreaker() {
  if (s_state != FifoState::S14_DISABLED) {
    return false;  // no-op, not an error, if the breaker was never tripped
  }
  s_state = FifoState::S1_IDLE;
  s_consecutiveFailedSessions = 0;
  return true;
}
