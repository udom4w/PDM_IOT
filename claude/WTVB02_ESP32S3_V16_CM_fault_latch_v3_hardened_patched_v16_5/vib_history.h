#pragma once
// ============================================================================
// [M1B-1] vib_history.h -- timestamped velocity trend history ring
//
// THE single append point for the new FIFO-DSP trend pipeline. Nothing else in
// the firmware may write trend samples; M1B-2..M1B-5 (EMA, windows, slope, TTW)
// will all READ this ring and never maintain state of their own.
//
// Zero Arduino/ESP32/FreeRTOS dependency, matching vib_accel.h / vib_velocity.h
// so the ring's ordering, wrap, gap and rejection logic can be host-tested
// under plain g++ -- the same property that let the DSP be verified to 1e-5%.
//
// WHAT THIS MODULE DELIBERATELY DOES NOT DO (M1B-1 scope):
//   - no EMA          (M1B-2)
//   - no slope        (M1B-4)
//   - no TTW          (M1B-5)
//   - no windowing/aggregation (M1B-3)
// It only stores what happened, when, and whether it was real.
//
// CADENCE REALITY (measured over three 30-minute hardware runs, n=441..579):
//   mean 3.11 s, median 3.88 s, BIMODAL at 2.1 s (~45%) and 3.9 s (~55%),
//   observed max 47.9 s during a FIFO stall, worst case ~15 min under the
//   900 s recovery backoff. Nothing here may assume a fixed interval: every
//   dt is derived from stored timestamps, never from a nominal period.
// ============================================================================

#include <stdint.h>
#include <stddef.h>

// ----------------------------------------------------------------------------
// Ring capacity.
//
// Sized from the measured MINIMUM interval, not the mean, so the worst (densest)
// case still spans the longest planned analysis window:
//   256 samples x 2.101 s (fastest observed) = 538 s  > 300 s target window
//   256 samples x 3.11 s  (mean observed)    = 796 s  (~13 min)
// Power of two so the modulo folds to a mask.
// ----------------------------------------------------------------------------
#define VIB_HIST_CAPACITY 256u

// Why an appended sample was refused. Exposed for diagnostics/tests; the ring
// never silently drops anything.
enum VibHistReject {
  VIB_HIST_OK = 0,
  VIB_HIST_REJ_NOT_FINITE,     // NaN/Inf value
  VIB_HIST_REJ_NEGATIVE,       // velocity RMS cannot be < 0
  VIB_HIST_REJ_TIME_BACKWARD,  // timestamp not strictly after the previous
  VIB_HIST_REJ_DUPLICATE_ID,   // same captureId already stored
};

// Why a gap marker was recorded.
enum VibHistGapReason {
  VIB_GAP_NONE = 0,
  VIB_GAP_VELOCITY_INVALID,    // DSP produced no usable velocity for a capture
  VIB_GAP_STALE,               // carrier aged past the freshness deadline
  VIB_GAP_MOTOR_NOT_RUNNING,   // STOPPED/STARTING/STOPPING
};

// ----------------------------------------------------------------------------
// VibHistorySample
//
// `valid` is the sole authority for reading `velocity_rms_overall`, mirroring
// FifoCaptureResult.error and VibAccelRms.valid.
//
// CRITICAL -- GAP MARKERS ARE NOT ZERO:
// For a gap (valid=false, gap=true) `velocity_rms_overall` is set to NaN, NOT
// 0.0f. This is a deliberate safety property, not a stylistic choice: 0.0 is a
// plausible-looking vibration reading, so a consumer that forgot to check
// `valid` would silently bias every mean, EMA and slope toward zero -- the
// exact "missing data becomes a zero vibration sample" failure M1B forbids.
// NaN cannot be mistaken for a measurement: it propagates, and any downstream
// arithmetic that ignores `valid` produces a visibly broken result instead of a
// quietly wrong one. Loud failure over silent corruption.
// ----------------------------------------------------------------------------
struct VibHistorySample {
  float    velocity_rms_overall;  // [mm/s] valid ONLY when valid==true; NaN when gap
  uint32_t timestampMs;           // producer-side millis() at capture completion
  uint32_t captureId;             // 0 for gap markers (no capture produced it)
  bool     valid;                 // true => real measurement
  bool     gap;                   // true => explicit missing-data marker
  uint8_t  gapReason;             // VibHistGapReason; VIB_GAP_NONE when valid
};

struct VibHistoryStats {
  uint32_t appended;      // valid samples stored
  uint32_t gaps;          // gap markers stored
  uint32_t rejected;      // append attempts refused (see lastReject)
  uint8_t  lastReject;    // VibHistReject
  uint32_t overwritten;   // samples evicted by ring wrap
};

// ---- Lifecycle -------------------------------------------------------------
void VibHistory_Init(void);

// ---- The ONE append point --------------------------------------------------
// Appends a real measurement. Returns VIB_HIST_OK, or the reason it was
// refused. Rejections are counted, never silently ignored.
//
// Enforces, in order:
//   - value finite and >= 0
//   - timestamp strictly after the previous entry (wrap-safe signed compare,
//     so the ~49.7-day millis() rollover is not seen as time going backwards)
//   - captureId not already stored (guards a double-drain of the same capture)
uint8_t VibHistory_Append(float velocityMmS, uint32_t timestampMs, uint32_t captureId);

// Records an explicit missing-data marker. Never stores a value.
// Consecutive gaps with the same reason are coalesced: a 15-minute stall must
// not consume the whole ring with thousands of identical markers and evict the
// real history either side of it -- the gap's extent is recoverable from the
// surrounding timestamps.
uint8_t VibHistory_AppendGap(uint32_t timestampMs, uint8_t gapReason);

// ---- Read-only access ------------------------------------------------------
uint16_t                VibHistory_Count(void);       // entries currently stored
const VibHistorySample* VibHistory_At(uint16_t idxFromOldest);  // NULL if out of range
const VibHistorySample* VibHistory_Latest(void);      // NULL if empty
const VibHistorySample* VibHistory_LatestValid(void); // newest valid sample, skipping gaps
VibHistoryStats         VibHistory_GetStats(void);

// Elapsed ms between two stored entries, wrap-safe. Helper so no consumer
// re-implements the rollover-correct subtraction.
uint32_t VibHistory_ElapsedMs(uint32_t earlierMs, uint32_t laterMs);
