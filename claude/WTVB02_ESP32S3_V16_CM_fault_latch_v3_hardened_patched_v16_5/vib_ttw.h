#pragma once
// ============================================================================
// [M1B-5] vib_ttw.h -- Time-To-Warning from FIFO-DSP velocity + M1B-4 slope
//
// Zero Arduino/ESP32/FreeRTOS dependency, matching the rest of the M1B chain,
// so every branch of the validity contract is host-testable under plain g++.
//
// SOURCES: the M1B-1 ring (newest VALID sample) and an M1B-4 VibSlopeResult
// passed in by the caller. NOTHING ELSE. Specifically NOT g_trendBuf[].rms,
// NOT rms_slope, NOT WARNING_RMS, NOT TREND_SLOPE_UP/DOWN, and NOT the 4.0f
// cadence constant -- the legacy TTW at .ino:8378 uses all five and is left
// running untouched for compatibility until M1B-8.
//
// WHY THE 4.0f DISAPPEARS RATHER THAN CHANGING VALUE:
// legacy   ratePerHour = rmsSlope[mm/s per SAMPLE] * 4.0[samples/s] * 3600
// new      ratePerHour = slope  [mm/s per SECOND]                   * 3600
// The cadence factor existed only to convert per-sample into per-second. The
// M1B-4 slope is already per-second, so 3600 is a pure seconds->hours
// conversion and carries no assumption about capture rate. Re-tuning 4.0 to
// 0.32 would have been the wrong fix; removing it is the right one.
//
// CURRENT PRODUCTION STATE: numeric output is DISABLED BY CONSTRUCTION.
// Both VIB_TTW_MIN_SLOPE (below) and the caller's warning threshold
// (VIB_WARNING_MMS) are unset sentinels, so VibTtw_Compute() can only ever
// return VIB_TTW_THRESHOLDS_UNSET until BOTH are re-baselined. There is no
// fallback path, no default, and no guessed value.
// ============================================================================

#include <stdint.h>
#include "vib_history.h"
#include "vib_slope.h"

// ----------------------------------------------------------------------------
// [M1B-5 TBD] Minimum slope, in mm/s per second, below which no TTW is emitted.
//
// *** DELIBERATELY UNSET. NO VALUE HAS BEEN CHOSEN. ***
//
// This is the single most safety-relevant number in this module, and it cannot
// be guessed. Measured on the M1B-4 hardware run (26 steady-state summaries,
// idle bench):
//     slope sd            = 1.150e-05 mm/s/s
//     => noise-only rate  = 0.0414 mm/s per HOUR (1 sigma)
//     => 3 sigma          = 0.1242 mm/s per hour
// With a gap of a few mm/s, pure sensor noise would divide out to a TTW in the
// tens-to-hundreds of hours -- a confident-looking number describing nothing.
// The legacy guard (ratePerHour > 0.001) sits ~40x BELOW that noise floor and
// would not have rejected it.
//
// The floor must therefore be derived from slope noise measured on a LOADED
// machine, not from this idle-bench figure, and re-derived together with
// VIB_WARNING_MMS. Negative sentinel so a comparison can never accidentally
// pass if a future edit drops the configured-check.
// ----------------------------------------------------------------------------
// The #ifndef guard is deliberate: it lets the floor be supplied at build time
// (host tests, and eventually a calibrated production build) WITHOUT editing
// this header. Nothing in the firmware defines it, so the production value
// remains the UNSET sentinel and numeric TTW stays disabled by construction.
#define VIB_TTW_SLOPE_UNSET   (-1.0f)
#ifndef VIB_TTW_MIN_SLOPE
#define VIB_TTW_MIN_SLOPE     VIB_TTW_SLOPE_UNSET   // TBD -- pending re-baselining
#endif

// ----------------------------------------------------------------------------
// Prediction horizon. Beyond this a TTW is arithmetically valid but practically
// meaningless -- at 720 h (30 days) the linear extrapolation has long outlived
// any confidence the 300 s regression window can support. Reported as
// HORIZON_EXCEEDED rather than as a large number, so no dashboard renders
// "4,812 hours" as if it were a forecast.
//
// This is a DESIGN CHOICE with a stated rationale, not a calibration constant:
// unlike VIB_TTW_MIN_SLOPE it does not gate correctness, only presentation.
// ----------------------------------------------------------------------------
#define VIB_TTW_MAX_HOURS     720.0f

enum VibTtwStatus {
  VIB_TTW_VALID = 0,
  VIB_TTW_THRESHOLDS_UNSET,    // warning threshold and/or MIN_SLOPE not configured
  VIB_TTW_MOTOR_STOPPED,
  VIB_TTW_VELOCITY_UNAVAILABLE,// no valid ring sample, or the newest is stale
  VIB_TTW_INSUFFICIENT_DATA,   // slope not valid (<8 samples in the newest segment)
  VIB_TTW_SLOPE_NOT_RISING,    // slope negative, zero, or below MIN_SLOPE
  VIB_TTW_ALREADY_AT_THRESHOLD,// current velocity already >= the warning threshold
  VIB_TTW_HORIZON_EXCEEDED,    // computed TTW beyond VIB_TTW_MAX_HOURS
};

struct VibTtwResult {
  float    hours;          // valid ONLY when status == VIB_TTW_VALID; otherwise 0.0f
                           // meaning "not computed", NEVER "0 hours to warning"
  uint8_t  status;         // VibTtwStatus
  float    v_current_mms;  // the level used, for traceability (0 when unavailable)
  float    slope_used;     // the slope used, for traceability (0 when unavailable)
};

// ----------------------------------------------------------------------------
// VibTtw_Compute
//
//   ttw_hours = (warnThresholdMmS - v_current) / (slope_mms_per_s * 3600)
//
//   numerator   [mm/s]
//   denominator [mm/s per second] * [s/h] = [mm/s per hour]
//   quotient    -> HOURS
//
// v_current is the newest VALID ring sample (VibHistory_LatestValid()), NOT the
// EMA: TTW extrapolates from the actual present level, and pairing an
// EMA-smoothed level with a raw-fit slope would misstate the gap.
//
// Thresholds are passed in rather than #included so this file stays free of
// .ino globals and so tests can drive any configuration. `warnThresholdMmS <= 0`
// means "unset", matching VIB_THRESHOLD_UNSET's negative-sentinel convention.
//
// Returns true only when status == VIB_TTW_VALID. `out` must be non-NULL;
// `slope` may be NULL (treated as INSUFFICIENT_DATA).
// Pure: no module state, no allocation, no I/O, no blocking.
// ----------------------------------------------------------------------------
bool VibTtw_Compute(bool                  motorRunning,
                    const VibSlopeResult* slope,
                    float                 warnThresholdMmS,
                    uint32_t              nowMs,
                    uint32_t              maxAgeMs,
                    VibTtwResult*         out);

const char* VibTtw_StatusStr(uint8_t status);
