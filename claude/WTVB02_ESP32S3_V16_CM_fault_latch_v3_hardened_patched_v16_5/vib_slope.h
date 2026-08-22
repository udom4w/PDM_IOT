#pragma once
// ============================================================================
// [M1B-4] vib_slope.h -- timestamp-aware velocity slope (mm/s per second)
//
// Zero Arduino/ESP32/FreeRTOS dependency, matching vib_history.h / vib_ema.h /
// vib_window.h, so segment selection, gap splitting and the regression itself
// are host-testable under plain g++.
//
// READS THE M1B-1 RING ONLY. Never g_velCarrier, never sensorData.rms_overall,
// never the legacy g_trendBuf.
//
// WHY THIS REPLACES THE LEGACY SLOPE:
// The legacy regression accumulates `sumX += i` -- the SAMPLE INDEX -- so its
// output is "mm/s per sample". That is only meaningful if samples are evenly
// spaced. The FIFO-DSP source is measurably not: three 30-minute hardware runs
// put the interval at mean ~3.10 s but BIMODAL at 2.1 s (45%) and 3.9 s (55%),
// so consecutive index steps represent physically different durations. The same
// index-based slope also silently carries a hidden cadence constant into TTW
// (`rmsSlope * 4.0f * 3600.0f`). Regressing against elapsed seconds removes
// both problems: the result is mm/s per second regardless of spacing.
//
// SCOPE (M1B-4): slope only. No TTW (M1B-5), no TelemetrySlot change (M1B-6),
// no legacy removal (M1B-8). Nothing here writes to the ring.
// ============================================================================

#include <stdint.h>
#include "vib_history.h"

// ----------------------------------------------------------------------------
// SLOPE WINDOW -- stated explicitly, not inherited from any legacy constant.
//
// 300 s, matching the M1B-3 300 s statistics window so the two describe the
// same span. At the measured 3.096 s cadence that is ~97 samples: enough for a
// regression whose noise is dominated by the signal rather than by sample
// count. The 60 s window (~19 samples) was considered and rejected for slope --
// with a per-sample sd of ~0.009 mm/s, 19 points give a slope confidence
// interval wide enough to swamp the small drifts this metric exists to detect.
//
// This is a DESIGN CHOICE, not a tunable inherited from the 4 Hz era. The
// legacy TREND_WINDOW_SAMPLES (120 samples) is deliberately NOT reused: at
// 0.32 Hz those 120 samples would span ~6.2 minutes, not the 30 s it meant at
// 4 Hz.
// ----------------------------------------------------------------------------
#define VIB_SLOPE_WINDOW_MS   300000u

// Any interval longer than this splits the segment. Set to 3*tau (the same
// 90 s the M1B-2 EMA uses to decide a reseed) so "too long to connect" means
// one thing across the whole trend pipeline rather than two different numbers.
#define VIB_SLOPE_GAP_SPLIT_MS 90000u

// Below this, report INSUFFICIENT_DATA. A least-squares slope needs >= 2 points
// to exist at all; 8 is the product floor, chosen so a slope is never published
// from a handful of samples that happen to survive a gap. At 3.096 s cadence a
// full 300 s window holds ~97, so this only trips during start-up, a motor
// stop, or heavy gapping.
#define VIB_SLOPE_MIN_SAMPLES  8u

// ----------------------------------------------------------------------------
// [M1B-4 TBD] Slope direction thresholds, in mm/s PER SECOND.
//
// *** DELIBERATELY UNSET. NO VALUE HAS BEEN CHOSEN. ***
// The legacy TREND_SLOPE_UP/DOWN (+/-0.002) are in "mm/s per SAMPLE" and were
// derived at 4 Hz -- reusing them here would be a unit error, not merely a
// retune. Re-derivation in mm/s/s requires field data and is deferred.
//
// Negative sentinels, so a direction test can never accidentally pass if a
// future edit drops the configured-check. M1B-4 publishes NO direction field,
// so these are currently unused by design.
// ----------------------------------------------------------------------------
#define VIB_SLOPE_THRESHOLD_UNSET (-1.0f)
#define VIB_SLOPE_UP_MMS_PER_S    VIB_SLOPE_THRESHOLD_UNSET   // TBD
#define VIB_SLOPE_DOWN_MMS_PER_S  VIB_SLOPE_THRESHOLD_UNSET   // TBD

struct VibSlopeResult {
  float    slope_mms_per_s;  // valid ONLY when valid == true
  bool     valid;            // false => INSUFFICIENT_DATA; slope is 0.0f meaning
                             // "not computed", NEVER "flat trend"
  bool     reseeded;         // segment start was set by a GAP SPLIT rather than
                             // by the window edge -- i.e. an outage inside the
                             // window truncated the regression
  uint32_t timestamp_ms;     // newest sample in the regressed segment
  uint16_t sample_count;     // points actually regressed
  uint32_t span_ms;          // elapsed time actually covered by those points
};

// ----------------------------------------------------------------------------
// VibSlope_Compute -- least squares over the NEWEST contiguous valid segment.
//
// Selection, newest -> oldest:
//   1. skip gap markers entirely (never zero, never interpolated)
//   2. stop at the window edge (nowMs - VIB_SLOPE_WINDOW_MS)
//   3. stop at the first interval > VIB_SLOPE_GAP_SPLIT_MS between two
//      consecutive VALID samples -- the regression never spans an outage,
//      because a 47.9 s (or 15-minute) hole would otherwise dominate the fit
//      and manufacture a trend out of a sensor failure
//
// Regression uses relative seconds from the segment's FIRST included sample:
//     slope = cov(t, v) / var(t)
// Relative time keeps the magnitudes small -- absolute millis() values near
// 2^32 squared would lose precision even in double.
//
// Returns out->valid. `out` must be non-NULL. Pure: no state, no allocation,
// no I/O, no blocking.
// ----------------------------------------------------------------------------
bool VibSlope_Compute(uint32_t nowMs, VibSlopeResult* out);
