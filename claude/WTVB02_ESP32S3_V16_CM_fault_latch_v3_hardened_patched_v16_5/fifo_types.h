#pragma once
// ============================================================================
// [v16.6.9-fifo] fifo_types.h
//
// L3 -- Shared Types (Implementation Plan v1.4, Task 3.2; originally
// specified as Task 3.1 under Implementation Plan v1.0/v1.3, renumbered by
// ADR-0004 to make room for Session Controller's promotion to Task 3.1 --
// see CM-100_ADR-0004_Session_Controller_Promotion_v1.0.md and
// CM-100_FIFO_Implementation_Plan_v1.4_Delta.md SS2/SS3. Content is otherwise
// byte-for-byte unchanged from the frozen v1.0/v1.3 specification -- this
// renumbering did not add, remove, or reshape a single field or enumerator.
//
// Single source of truth for the types every other L3/L4 file needs,
// declared once, included early -- this is what avoids the .ino
// auto-prototype forward-declaration pattern PRR flagged as a code smell in
// the Validation Tool (this was this task's stated purpose since v1.0).
//
// Zero Arduino/ESP32/FreeRTOS dependency by design, matching fifo_transport.h,
// fifo_codec.h, and fifo_session.h -- compiles standalone under a plain C++
// compiler. This file sits BELOW every other FIFO driver file in the
// include graph: it never includes fifo_transport.h, fifo_codec.h, or
// fifo_session.h, and nothing in this file references FifoTransport,
// FrameCodecState, or FifoSessionState. Other files include this one, never
// the reverse.
//
// Explicitly OUT OF SCOPE for this file (per Implementation Plan v1.4, Task
// 3.2's "shared types" purpose -- these are named types other files depend
// on, not behavior):
//   - Transport logic (fifo_transport.h/fifo_transport_uart485.h, L1).
//   - Parser/frame-codec logic and its state (fifo_codec.h, L2).
//   - Session-controller logic and its state (fifo_session.h, L3 Task 3.1).
//   - The internal S0..S14 protocol state-machine enum (SDS SS11.1,
//     SS20.1's "state : FifoState (S0..S14)") -- that is FifoDriver's own
//     internal-only symbol, defined in fifo_driver.h (Task 3.4, "internal-
//     only symbols not yet exported" per the Implementation Plan), never a
//     shared type. FifoPhase below is the only driver-state enum this file
//     defines -- the coarse, PUBLIC 5-value phase (SDS SS7.1 A-4), not the
//     internal 15-value state machine.
//   - Retry/backoff/circuit-breaker policy and its named timeout constants
//     (T_REQUEST_RESPONSE_MS, T_POLL_INTERVAL_MS, MAX_DESYNC_BYTES,
//     FIFO_MAX_RETRIES, FIFO_BREAKER_THRESHOLD, etc.) -- all assigned to
//     Task 3.5 (recovery/lifecycle), defined in fifo_driver.cpp, never here.
//
// All types below are POD / trivially copyable: plain enums and structs
// with no constructor, destructor, virtual member, or owning pointer.
// static_asserts at the bottom of this file make that a compiler-enforced
// guarantee, not just a comment. The three raw pointer fields on
// FifoCaptureResult (x/y/z) are non-owning views into FifoArena's storage
// (Task 3.3) -- copying them copies the pointer value only, never the
// pointed-to samples, and is exactly as safe (and exactly as non-owning) as
// copying any other pointer field.
// ============================================================================

#include <stdint.h>
#include <stddef.h>
#include <type_traits>

// ----------------------------------------------------------------------------
// FifoPhase -- the driver's coarse, PUBLIC lifecycle phase (SDS SS7.1, A-4:
// FifoDriver_GetPhase()). Distinct from the internal S0..S14 state machine
// (SS11.1) -- FifoPhase is what a caller outside the driver is allowed to
// see; S0..S14 is FifoDriver's own internal sequencing, never exposed here.
// ----------------------------------------------------------------------------
enum class FifoPhase {
  IDLE,          // Ready; no capture in progress, no result held
  ACTIVE,        // A capture attempt is in progress; bus held (SS11.1 S2..S10)
  RESULT_READY,  // A finished result is available via FifoDriver_TryAcquireResult()
  COOLDOWN,      // Post-capture bus rest (SS17.4); no new request admitted yet
  BREAKER_DISABLED,  // Circuit breaker open (SS17.6); no request admitted until reset
                     // [renamed from DISABLED -- collides with the ESP32
                     // Arduino core's #define DISABLED 0x00 in
                     // esp32-hal-gpio.h, which corrupts this enumerator
                     // via preprocessor text substitution once <Arduino.h>
                     // is included ahead of this header in the sketch.
                     // Spelling only -- FifoPhase's shape, order, and
                     // values are otherwise unchanged.]
};

// ----------------------------------------------------------------------------
// FifoError -- the 14-code, domain-classified error taxonomy (SDS SS16.1,
// D-18). Replaces the Validation Tool's single overloaded FIFOFrameResult
// value (PRR C-7) with one code per distinct failure, grouped by the three
// domains SS16.1 defines: byte/frame-level (parser), protocol-level (K-10
// invariants), and lifecycle-level (driver/caller policy).
//
// [ADR-0004 SS10] Verified compatible with Session Controller's outcome
// taxonomy: FifoSessionOutcome::FRAME_REJECTED maps losslessly to
// ERR_CRC_MISMATCH; FifoSessionOutcome::SESSION_FAILED with
// FifoSessionFailReason::BAD_FRAME_TYPE maps to ERR_BAD_TYPE_BYTE, and with
// FifoSessionFailReason::DESYNC maps to ERR_DESYNC_LIMIT. All three target
// codes already exist below -- nothing was added or renamed to satisfy
// this. The actual translation is Task 3.4's responsibility, not this
// file's; this file only guarantees the target codes exist.
// ----------------------------------------------------------------------------
enum class FifoError {
  NONE,                     // No error -- capture succeeded, or none attempted yet

  // Byte/frame-level (parser) domain
  ERR_NO_RESPONSE,          // No response within the request/response budget
  ERR_INTER_BYTE_TIMEOUT,   // Response started but stalled mid-frame
  ERR_DESYNC_LIMIT,         // Anchor rescan exceeded MAX_DESYNC_BYTES (SS17.2)
  ERR_RX_OVERFLOW,          // Transport-reported RX overflow (FifoTransport::hadOverflow)
  ERR_CRC_MISMATCH,         // Frame fully received; CRC-16 did not verify
  ERR_BAD_TYPE_BYTE,        // Type/discriminator byte was neither 0x00 nor 0x01

  // Protocol-level (K-10 invariant) domain
  ERR_PROGRESS_REGRESSION,  // fill regressed against the session's lastProgressFill
  ERR_PROGRESS_OVERRUN,     // fill exceeded 6144 (K-10)

  // Lifecycle domain -- driver/caller policy (SS16.1)
  ERR_BUSY,                 // Capture already active; reject at admission
  ERR_NOT_PERMITTED,        // SS19.2 admission gate failed
  ERR_RESULT_NOT_RELEASED,  // Previous result still held; reject at admission
  ERR_ABORTED,              // FifoDriver_Abort() called
  ERR_RETRY_EXHAUSTED,      // Retry budget spent
  ERR_CIRCUIT_OPEN,         // Breaker tripped (SS17.6); reject at admission
};

// ----------------------------------------------------------------------------
// FifoTriggerSource -- what caused a capture to be requested (SS19.1).
// [Commit 7B] REMOTE_ON_DEMAND added: the inbound MQTT channel that made it
// dead code now exists (Commit 7A's subscribe/callback, Commit 7B's Trigger
// Broker producer wiring in mqttCommandCallback()) -- this enumerator has a
// caller that can set it, per this comment's own previously-stated condition
// for adding it (CN-8, G-2).
// ----------------------------------------------------------------------------
// [Commissioning Removal] COMMISSIONING removed -- its only producer (the
// Task 4.4 one-shot auto-trigger in taskModbusRead()) has been deleted and
// no other production caller ever set this source. Do not re-add without a
// real caller.
enum class FifoTriggerSource {
  FAULT_LATCH,
  OPERATOR_BUTTON,
  SCHEDULED,
  REMOTE_ON_DEMAND,
};

// ----------------------------------------------------------------------------
// Shared compile-time constant: the fixed size of the caller-supplied
// diagnostic tag carried on both FifoCaptureRequest and FifoCaptureResult.
// Named once here (both structs reference it) rather than repeating the
// literal `16` independently in two places -- the size itself is unchanged
// from the frozen v1.0/v1.3 specification (`char tag[16]`); this only gives
// the existing literal a name.
// ----------------------------------------------------------------------------
static const size_t FIFO_TAG_MAXLEN = 16;

// ----------------------------------------------------------------------------
// FifoAdmissionContext -- [v1.1, Freeze Review 3.2] caller-populated, plain
// admission-gate input. Request()'s admission check (SS19.2) reads only
// this struct -- never a `.ino` global directly -- which is what keeps
// fifo_driver.cpp host-testable independent of the 8,343-line `.ino`
// (Freeze Review Finding 3.2).
// ----------------------------------------------------------------------------
struct FifoAdmissionContext {
  bool motorStable;       // from g_systemState, read by the CALLER, never by fifo_driver.cpp
  bool sensorHealthy;     // from g_modbusConsecErrors == 0
  bool mqttReconnecting;  // from network task state
};

// ----------------------------------------------------------------------------
// FifoCaptureRequest -- the caller-populated argument to
// FifoDriver_Request() (SS7.1, A-2).
// ----------------------------------------------------------------------------
struct FifoCaptureRequest {
  FifoTriggerSource    triggerSource;
  char                 tag[FIFO_TAG_MAXLEN];
  bool                 requirePermissive;
  uint8_t              maxRetries;
  FifoAdmissionContext admissionContext;  // [v1.1] the ONLY admission input Request() reads

  // [Phase 3A] Sample-rate provenance, supplied BY THE CALLER and copied
  // verbatim onto FifoCaptureResult at admission (SDS D-3: "provenance is
  // captured at request time, not publish time"). The driver never derives,
  // validates or defaults these -- it does not know the sensor's register
  // map and must stay portable (this file compiles host-side under plain
  // g++ with no Arduino). They are NOT admission inputs: FifoDriver_Request()
  // does not read them for any gate, so a capture is never rejected for
  // missing provenance -- it is merely reported as having none.
  //
  // FAIL-CLOSED CONTRACT: srHz == 0 means "sample rate NOT established for
  // this capture". Any consumer MUST treat that as invalid provenance and
  // MUST NOT substitute a nominal rate. srIndexAtCapture uses
  // FIFO_SR_INDEX_UNKNOWN for the same purpose.
  uint16_t             srIndexAtCapture;  // raw REG_SAMPLE_RATE value, or UNKNOWN
  uint32_t             srHz;              // decoded Hz, or 0 == not established

  // [R-3] Machine-state provenance, same contract as the sample-rate pair
  // above: supplied BY THE CALLER, copied verbatim onto FifoCaptureResult at
  // the provenance latch, never derived/validated/defaulted by the driver,
  // and NOT admission inputs -- FifoDriver_Request() reads none of them for
  // any gate, so a capture is never rejected for missing machine provenance.
  //
  // Previously these three lived only on FifoCaptureResult with NO producer
  // anywhere in the firmware, so the /event fifo_capture payload published
  // their zero-initialised defaults forever (observed on hardware: 317/317
  // events with motor_state=0 rpm=0 temp_c=0 while /vibration concurrently
  // reported motor_state=2 rpm~1484 temp~51.1). Same defect class as the
  // v16.6.16 s_result.status fix, which missed these.
  //
  // Caller sources g_telemSnapshot (the v16.5.4 atomic telemetry snapshot),
  // so fifo_capture provenance is internally consistent with /vibration by
  // construction. motorStateAtCapture is whatever the production motor-state
  // source already produced (MOTOR_SRC_CURRENT) -- this is a pure copy and
  // changes no detection logic.
  float                tempCAtCapture;       // [degC]
  uint8_t              motorStateAtCapture;  // MotorRunState_t: 0=STOPPED,1=STARTING,2=RUNNING,3=STOPPING
  float                rpmAtCapture;         // [rpm]
};

// [Phase 3A] Sentinel for "SR index not established". 0x0000 is a REAL SR
// index (SR0 = 32 kHz per WTVB02-485 manual Sec 6.4.12), so zero cannot be
// overloaded as "unknown" the way srHz == 0 legitimately can.
#define FIFO_SR_INDEX_UNKNOWN 0xFFFFu

// ----------------------------------------------------------------------------
// FifoCaptureResult -- the validity-gated result object (SDS SS8.2, D-8).
// `error` (not a fabricated "success" bool) is the sole authority on
// whether `x`/`y`/`z` may be read -- valid only while held via
// FifoDriver_TryAcquireResult() AND error == FifoError::NONE.
//
// `x`/`y`/`z` are non-owning views into FifoArena's static storage
// (Task 3.3) -- this struct never allocates, frees, or owns sample memory;
// it only ever holds three pointers into memory FifoArena continues to own.
// ----------------------------------------------------------------------------
struct FifoCaptureResult {
  uint32_t          captureId;
  char              tag[FIFO_TAG_MAXLEN];
  FifoTriggerSource triggerSource;
  FifoPhase         status;   // driver phase at the moment this result was finalized
  FifoError         error;    // FifoError::NONE required before x/y/z may be read

  uint16_t          sampleCount;
  uint16_t          srIndexAtCapture;
  uint32_t          srHz;

  uint32_t          tRequestMs;
  uint32_t          tCompleteMs;

  float             tempCAtCapture;
  uint8_t           motorStateAtCapture;
  float             rpmAtCapture;

  const int16_t*    x;  // non-owning; valid only while held AND error == NONE
  const int16_t*    y;
  const int16_t*    z;

  uint16_t          pollCount;
  uint16_t          progressFrameCount;
  uint16_t          crcErrorCount;
  uint16_t          retryCount;
  uint16_t          lastProgressFill;

  uint32_t          tFirstByteMs;
  uint32_t          tFirstProgressMs;
  uint32_t          tLastProgressMs;
  uint32_t          tAnchorAfterLastProgressMs;

  uint32_t          desyncBytesDiscarded;
};

// ----------------------------------------------------------------------------
// FifoDriverStats -- lifetime counters snapshot (SS7.1, A-9:
// FifoDriver_GetStats()).
//
// [Implementation Plan v1.0/v1.3, Task 3.1/3.2's own Risks field]
// "FifoDriverStats { /* lifetime counters -- finalized at coding time */ }"
// -- the SDS explicitly defers this struct's field list rather than
// specifying it. That deferral is honored here, not resolved: inventing a
// counter list now, unreviewed, is exactly the kind of late change to "the
// file every other file depends on" that same Risks field warns carries
// the widest blast radius of any task in this phase. Deliberately left
// empty. A future task (or a short ADR, if the shape turns out to be
// non-obvious) must populate this before Task 3.6 (public API) can give
// FifoDriver_GetStats() a real body.
// ----------------------------------------------------------------------------
struct FifoDriverStats {
  // Intentionally empty -- see comment above. Not zero-sized in C++
  // (sizeof(FifoDriverStats) >= 1 is guaranteed by the standard), so this
  // compiles and is safely copyable as-is; it simply carries no data yet.
};

// ----------------------------------------------------------------------------
// Compile-time guarantee, not just a comment: every POD struct above must
// remain trivially copyable. If a future edit ever adds a constructor,
// destructor, virtual member, or owning-pointer/container field to any of
// these, this fails to compile instead of silently breaking the "no
// dynamic allocation, no hidden ownership" contract every other file in
// this driver depends on.
// ----------------------------------------------------------------------------
static_assert(std::is_trivially_copyable<FifoAdmissionContext>::value,
              "FifoAdmissionContext must remain trivially copyable");
static_assert(std::is_trivially_copyable<FifoCaptureRequest>::value,
              "FifoCaptureRequest must remain trivially copyable");
static_assert(std::is_trivially_copyable<FifoCaptureResult>::value,
              "FifoCaptureResult must remain trivially copyable");
static_assert(std::is_trivially_copyable<FifoDriverStats>::value,
              "FifoDriverStats must remain trivially copyable");
