#pragma once
// ============================================================================
// [v16.6.11-fifo] fifo_driver.h
//
// L3 -- FifoDriver Core State Machine (Implementation Plan v1.4, Task 3.4;
// originally specified as Task 3.3 "Core protocol state machine (S0-S9)"
// under Implementation Plan v1.0/v1.3, renumbered by ADR-0004). Per the
// frozen Implementation Plan, THIS task's scope is states S0 through S9 --
// "request through verify" -- not the full S10..S14 recovery/lifecycle
// span, which the frozen plan explicitly assigns to a separate task
// ("Recovery/lifecycle states and timeout/retry policy", Task 3.4(old) ->
// 3.5(new)). See fifo_driver.cpp's own top-of-file comment for exactly
// what this means in practice, and this session's delivered Architecture
// Review for the full accounting of what is and is not implemented.
//
// FifoState (below) declares all 15 frozen states (S0..S14), matching SDS
// SS11.1's single unified state table -- the state SPACE is one frozen
// thing, even though the frozen Implementation Plan splits which task
// implements transition BEHAVIOR for which subset. Every state value is
// reachable from this file's code; not every state's own internal timing
// logic is implemented here.
//
// [Task 3.6] "Public interfaces: None yet" no longer applies -- this is
// the public FifoDriver_* wrapper section the frozen plan's own "Files
// affected: fifo_driver.h (adds the public section)" note anticipated.
// Everything below FifoState is new in this revision; FifoState itself
// and every piece of internal driver state remain exactly as Tasks
// 3.4/3.5 left them, `static`, internal-linkage-only inside
// fifo_driver.cpp.
// ============================================================================

#include "fifo_types.h"
#include "fifo_transport.h"

// FifoState -- the driver's internal S0..S14 protocol state machine (SDS
// SS11.1, SS20.1's "state : FifoState (S0..S14)"). Distinct from, and
// coarser-grained than, the external FifoPhase (fifo_types.h) a caller is
// allowed to see -- FifoPhase is derived FROM this internal state, never
// the reverse, and never leaked directly (this task's own responsibility
// #7: "publish state through FifoPhase only").
enum class FifoState {
  S0_UNINIT,
  S1_IDLE,
  S2_ARMED,
  S3_REQUEST,
  S4_AWAIT_ANCHOR,
  S5_READ_TYPE,
  S6_READ_PROGRESS,
  S7_POLL_WAIT,
  S8_READ_DUMP,
  S9_VERIFY,
  S10_DRAIN,
  S11_RESULT_READY,
  S12_COOLDOWN,
  S13_FAILED,
  S14_DISABLED,
};

// ============================================================================
// [Task 3.6] Public API -- SDS SS7.1's nine entry points (A-1..A-9).
// FifoDriver_OwnsBus() (A-5) was deliberately withheld under Task 3.6's own
// "implement only" scope (Task 4.2's integration concern, not this task's)
// and has since been added, inline, under Task 4.2 -- see its own doc
// comment below for why it lives here rather than in fifo_driver.cpp.
// Plus one addition beyond the frozen nine: FifoDriver_ResetCircuitBreaker().
// SS17.6 requires "manual reset"
// as S14 DISABLED's ONLY exit, but no entry point among the frozen nine
// provides it (Abort() is for an IN-PROGRESS capture, not a tripped
// breaker) -- this is a disclosed, necessary tenth entry point closing a
// real gap the frozen SDS's own prose (SS17.6) requires but never names a
// function for, not a scope-creep addition.
//
// FifoDriver_Init()'s parameter: the frozen SDS names this argument `cfg`
// (`const FifoDriverConfig* cfg`) but `FifoDriverConfig`'s shape was never
// frozen -- CM-100_FIFO_Architecture_Freeze_Resolution_v1.1.md explicitly
// lists "FifoDriverConfig's copy-vs-retain lifetime" among two
// Should-Improve items "deliberately left untouched... logged as
// intentionally deferred." Inventing a struct for an explicitly deferred
// design detail would be exactly the kind of unreviewed decision this
// whole driver's process has avoided elsewhere (see ADR-0004, and
// FifoDriverStats' own deliberately-empty precedent in fifo_types.h).
// This implementation instead takes the one thing Init()'s own frozen
// purpose unambiguously requires -- "bind transport" -- directly.
// ============================================================================

// FifoDriver_Init() -- bind the transport and zero all driver state.
//   Precondition:  `transport` is a valid, non-null, caller-owned
//                  FifoTransport that outlives the driver (matches every
//                  other L1 consumer's own precondition). Called once, at
//                  boot (SDS SS7.1 A-1) -- idempotent if called again
//                  (re-zeros everything), but not intended to be.
//   Postcondition: FifoDriver_GetPhase() == FifoPhase::IDLE.
void FifoDriver_Init(FifoTransport* transport);

// FifoDriver_Service() -- advance the driver by one bounded tick.
//   Precondition:  called only from `taskModbusRead`, exactly once per
//                  250 ms tick (SDS SS7.2 A-3), after FifoDriver_Init().
//   Postcondition: never blocks. Thin wrapper: calls
//                  FifoDriver_ServiceCoreState() then
//                  FifoDriver_ServiceRecoveryState() -- both already
//                  internally bounded, one action each (Tasks 3.4/3.5).
void FifoDriver_Service();

// FifoDriver_Request() -- submit a capture request; performs SS19.2's
// admission gating.
//   Precondition:  `req` is a valid, non-null, caller-owned
//                  FifoCaptureRequest. `outHandle` may be null (handle is
//                  best-effort informational output only).
//   Postcondition: returns FifoError::NONE and admits the request if
//                  every applicable SS19.2 gate passes (see
//                  fifo_driver.cpp for the exact gate order and which
//                  gates `req->requirePermissive == false` bypasses);
//                  otherwise returns the specific rejection code and
//                  admits nothing. On admission, `*outHandle` (if
//                  non-null) is set to the newly assigned captureId.
FifoError FifoDriver_Request(const FifoCaptureRequest* req, uint32_t* outHandle);

// FifoDriver_GetPhase() -- the driver's coarse, public lifecycle phase.
//   Precondition:  none. Callable from any task/core (SDS SS7.1 A-4).
//   Postcondition: returns the current FifoPhase, derived from internal
//                  FifoState (never the reverse).
FifoPhase FifoDriver_GetPhase();

// FifoDriver_OwnsBus() -- [Task 4.2, A-5 -- the entry point this section's
// own header note above named as deliberately not yet added] true whenever
// the driver is actively using the shared RS485 bus and no other caller may
// touch it.
//   Precondition:  none. Callable from any task/core, same as
//                  FifoDriver_GetPhase(), which this is a direct,
//                  zero-new-state wrapper around -- FifoPhase::ACTIVE
//                  already denotes exactly the internal range (S2_ARMED..
//                  S10_DRAIN, plus transient S13_FAILED en route to S10)
//                  that owns the bus (see fifo_driver.cpp's CurrentPhase(),
//                  unmodified). Defined inline here, not in fifo_driver.cpp,
//                  per this task's "do not modify fifo_driver.cpp" scope --
//                  no new driver state or behavior is introduced.
//   Postcondition: returns (FifoDriver_GetPhase() == FifoPhase::ACTIVE).
inline bool FifoDriver_OwnsBus() {
  return FifoDriver_GetPhase() == FifoPhase::ACTIVE;
}

// FifoDriver_TryAcquireResult() -- acquire exclusive, read-only access to
// the most recently finished capture's result.
//   Precondition:  `outResult` is a valid, non-null, caller-owned
//                  FifoCaptureResult. Intended call site: exactly one,
//                  per SDS SS9.1's "handleFifoCaptureCompletion()"
//                  (Freeze Review Finding 2.1) -- see fifo_driver.cpp's
//                  Thread-safety note for why this file cannot itself
//                  enforce that.
//   Postcondition: returns true and populates `*outResult` (including
//                  fresh x/y/z pointers from FifoArena_TryAcquire()) if a
//                  result is ready (FifoDriver_GetPhase() ==
//                  RESULT_READY) and not already held; returns false and
//                  leaves `*outResult` untouched otherwise.
bool FifoDriver_TryAcquireResult(FifoCaptureResult* outResult);

// FifoDriver_ReleaseResult() -- return a previously acquired result.
//   Precondition:  none enforced -- matches FifoArena_Release()'s own
//                  safe-no-op-if-not-held contract exactly (this is a
//                  direct, unconditional wrapper around it).
//   Postcondition: FifoArena_IsOwned() == false. Mandatory: the driver
//                  will not admit a new request (SS19.2's "no result
//                  held" gate) or complete its own S11 RESULT_READY
//                  wait until this is called (or the SS17.5 watchdog
//                  force-reclaims).
void FifoDriver_ReleaseResult();

// FifoDriver_Abort() -- request orderly termination of an in-progress
// capture.
//   Precondition:  none.
//   Postcondition: if a capture is in progress (internal state S2..S9),
//                  it is marked FifoError::ERR_ABORTED and routed through
//                  the SAME mandatory drain (S10) every other terminal
//                  outcome uses -- "driver still performs its bus drain"
//                  (SDS SS7.1 A-8) -- and is NOT retried (SS17.1's
//                  RETRY_ATTEMPT policy is deliberately bypassed for an
//                  explicit abort, by supplying data the driver's own
//                  existing retry decision already respects -- see
//                  fifo_driver.cpp). If no capture is in progress, this
//                  is a no-op. `reason` is accepted for a future
//                  diagnostics/telemetry task's use; this implementation
//                  does not store it anywhere beyond setting
//                  FifoError::ERR_ABORTED itself (SDS SS16.1's own fixed
//                  mapping: "ERR_ABORTED | Abort() called | terminal").
void FifoDriver_Abort(FifoError reason);

// FifoDriver_GetStats() -- lifetime counters snapshot.
//   Precondition:  `outStats` is a valid, non-null, caller-owned
//                  FifoDriverStats.
//   Postcondition: `*outStats` is populated. FifoDriverStats has no
//                  fields yet (deliberately deferred since Task 3.2 --
//                  "finalized at coding time", never resolved) -- this
//                  function currently writes an empty struct. Not a
//                  defect introduced here; a direct, correctly-inherited
//                  consequence of that earlier deferral. No counters are
//                  fabricated to fill it.
void FifoDriver_GetStats(FifoDriverStats* outStats);

// FifoDriver_SetDiagLogger() -- [Task 4.4, TEMPORARY, retry-root-cause
// investigation only -- not a permanent production feature] register an
// optional diagnostic sink for per-attempt state-transition/CRC/timeout
// tracing.
//   Precondition:  none. `fn` may be nullptr (the default, and the state
//                  after FifoDriver_Init()) to disable all diagnostic
//                  output.
//   Postcondition: every subsequent internally-generated diagnostic line is
//                  forwarded to `fn` as a single, already-formatted,
//                  null-terminated C string (no allocation on this file's
//                  side). Purely observational -- nothing in this file's
//                  retry/timeout/CRC/state-sequencing logic reads this
//                  pointer or is influenced by whether a logger is
//                  registered, and nullptr keeps this file's existing
//                  zero-Arduino-dependency host-test build
//                  (test/test_fifo_driver.cpp links this file directly, on
//                  a plain host compiler with no Serial available) exactly
//                  as it was before this task. Intended to be removed, and
//                  every call site it guards along with it, once this
//                  investigation concludes.
void FifoDriver_SetDiagLogger(void (*fn)(const char* msg));

// FifoDriver_GetInternalStateNameForDiag() -- [Task 4.5, TEMPORARY,
// EN-ownership-hypothesis verification only -- not a permanent production
// feature] human-readable name of the internal S0..S14 FifoState (the
// state fifo_types.h's own header explicitly keeps un-exported, "FifoDriver's
// own internal-only symbol" -- see fifo_driver.h's FifoState comment).
// Exposed here, read-only, purely so an external diagnostic log line (e.g.
// the .ino's rs485Enable()/rs485Disable() instrumentation) can report which
// internal state was active at the moment of a GPIO transition, without
// leaking FifoState itself into any public struct or caller decision.
//   Precondition:  none. Callable from any task/core, same as
//                  FifoDriver_GetPhase().
//   Postcondition: returns a static, non-null, null-terminated string naming
//                  the current internal FifoState (e.g. "S4_AWAIT_ANCHOR").
//                  Read-only -- has no effect on, and is not read by, any
//                  retry/timeout/CRC/state-sequencing decision.
const char* FifoDriver_GetInternalStateNameForDiag();

// FifoDriver_GetAttemptNumberForDiag() -- [Task 5.3, TEMPORARY, auto-restart
// runtime-verification investigation only -- not a permanent production
// feature] the current session's 1-based attempt number (1..1+FIFO_MAX_RETRIES),
// so an external diagnostic log line (e.g. the .ino's stuck-detection/
// restartSensorViaModbus() call site) can report which FIFO attempt, if any,
// was in progress at the moment of an unrelated RS485 event.
//   Precondition:  none.
//   Postcondition: returns (retryCount + 1) of the current/most recent
//                  FifoCaptureResult -- meaningless (but harmless) when no
//                  capture has ever run (reads 1, the zero-initialized
//                  retryCount + 1). Read-only -- has no effect on, and is
//                  not read by, any retry/timeout/CRC/state-sequencing
//                  decision.
uint32_t FifoDriver_GetAttemptNumberForDiag();

// [Task 7.1 -- UART receive-error instrumentation & validation, TEMPORARY,
// not a permanent production feature] Pull-based hooks so this file (L3,
// transport-agnostic, must stay buildable against test/log_replay_transport.h
// on a plain host compiler) can obtain UART-485-specific error-counter data
// and trigger a session-boundary reset WITHOUT ever #including
// fifo_transport_uart485.h itself -- mirrors FifoDriver_SetDiagLogger()'s
// own established pattern (Task 4.4). `getStats` and `resetStats` may each
// be nullptr (the default) to disable this feature entirely; nullptr keeps
// this file's zero-Arduino-dependency host-test build exactly as it was.
struct FifoUartDiagHooks {
  // getStats -- fills the 5 cumulative counters from whichever transport is
  // actually in use. Called only immediately before printing the
  // "UART DIAGNOSTICS" block (requirement 3) -- never on any hot path,
  // never influences any retry/timeout/CRC/state-sequencing decision.
  void (*getStats)(uint32_t* outFifoOvf, uint32_t* outBufferFull, uint32_t* outBreak,
                    uint32_t* outFrameErr, uint32_t* outParityErr);
  // resetStats -- zeroes the counters. Called by this file exactly once per
  // genuinely NEW FIFO session admission (HandleS1Idle()'s non-retry
  // branch), never on a retry-of-the-same-session resume (requirement 4).
  void (*resetStats)();
};
void FifoDriver_SetUartDiagHooks(FifoUartDiagHooks hooks);

// [Task 7.4 -- ReadDump timing audit, TEMPORARY, not a permanent production
// feature] Pull-based microsecond-clock hook, same nullptr-safe pattern as
// FifoDriver_SetDiagLogger()/FifoDriver_SetUartDiagHooks() -- this file's
// own FifoTransport::nowMs() is millisecond-resolution only (SDS SS6.1's
// own contract), too coarse to measure a single tick's FrameCodec_Step()/
// HandleReceiving() call duration (expected sub-millisecond to low-
// millisecond). Rather than widen the shared FifoTransport contract
// (fifo_transport.h, used by log_replay_transport.h's host-test double
// too) for one temporary investigation, this narrow, separate hook lets
// the .ino supply Arduino's micros() directly. nullptr (default) makes
// every timing call in this file return 0, harmlessly.
void FifoDriver_SetMicrosProvider(uint32_t (*fn)());

// FifoDriver_ResetCircuitBreaker() -- [beyond the frozen nine -- see this
// section's own header note] the only way S14 DISABLED ever exits (SDS
// SS17.6: "until reboot or an explicit reset... manual reset ensures a
// human sees the event").
//   Precondition:  none.
//   Postcondition: if the driver was in S14 DISABLED, returns to IDLE,
//                  the consecutive-failure counter is zeroed, and this
//                  function returns true. Otherwise a no-op, returns
//                  false -- calling this when the breaker has not
//                  tripped has no effect and is not an error.
bool FifoDriver_ResetCircuitBreaker();
