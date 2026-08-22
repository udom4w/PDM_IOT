// ============================================================================
// [M1B-4] vib_slope.cpp -- timestamp-aware velocity slope over the M1B-1 ring
// See vib_slope.h for the window choice, the gap-split rule and the TBD
// direction thresholds.
// ============================================================================

#include "vib_slope.h"
#include <math.h>
#include <string.h>

bool VibSlope_Compute(uint32_t nowMs, VibSlopeResult* out) {
  if (out == NULL) {
    return false;
  }

  // Fail closed and fully, so no caller can read a stale or partial result.
  memset(out, 0, sizeof(*out));
  out->valid = false;

  const uint16_t n = VibHistory_Count();
  if (n == 0u) {
    return false;
  }

  // ---- Pass 1: walk NEWEST -> OLDEST, selecting the segment ---------------
  // Collect indices rather than values so pass 2 can re-read without copying
  // the samples. Bounded by the ring capacity, so no allocation is needed.
  uint16_t idx[VIB_HIST_CAPACITY];
  uint16_t cnt = 0;
  uint32_t prevTs = 0;
  bool     havePrev = false;
  bool     splitByGap = false;

  for (int i = (int)n - 1; i >= 0; i--) {
    const VibHistorySample* s = VibHistory_At((uint16_t)i);
    if (s == NULL) {
      continue;
    }
    // Gap markers are skipped outright. Their NaN value is never read, and no
    // zero is substituted -- a gap contributes nothing to the regression.
    if (!s->valid || s->gap) {
      continue;
    }

    // Window edge. Wrap-safe: unsigned subtraction is correct across the
    // ~49.7-day millis() rollover.
    const uint32_t age = nowMs - s->timestampMs;
    if (age > VIB_SLOPE_WINDOW_MS) {
      break;   // everything older is outside the window too
    }

    if (havePrev) {
      // prevTs is the NEWER neighbour (we are walking backwards), so this is
      // the interval between two consecutive VALID samples.
      const uint32_t dt = prevTs - s->timestampMs;
      if (dt > VIB_SLOPE_GAP_SPLIT_MS) {
        // An outage sits between this sample and the newer one. Stop here:
        // the regression covers only the newest contiguous segment. Connecting
        // across the hole would let a sensor failure masquerade as a trend.
        splitByGap = true;
        break;
      }
    }

    idx[cnt++] = (uint16_t)i;
    prevTs   = s->timestampMs;
    havePrev = true;

    if (cnt >= VIB_HIST_CAPACITY) {
      break;
    }
  }

  if (cnt < VIB_SLOPE_MIN_SAMPLES) {
    out->sample_count = cnt;   // report what was found: "still filling" vs "nothing"
    out->valid        = false;
    return false;
  }

  // idx[] is newest-first; the oldest included sample is the last entry.
  const VibHistorySample* oldest = VibHistory_At(idx[cnt - 1]);
  const VibHistorySample* newest = VibHistory_At(idx[0]);
  if (oldest == NULL || newest == NULL) {
    return false;
  }
  const uint32_t t0 = oldest->timestampMs;

  // ---- Pass 2: least squares on RELATIVE seconds --------------------------
  // Relative time keeps values small; regressing on raw millis() near 2^32
  // would square to ~1.8e19 and lose precision even in double.
  double sumT = 0.0, sumV = 0.0;
  for (uint16_t k = 0; k < cnt; k++) {
    const VibHistorySample* s = VibHistory_At(idx[k]);
    const double t = (double)(uint32_t)(s->timestampMs - t0) / 1000.0;
    sumT += t;
    sumV += (double)s->velocity_rms_overall;
  }
  const double meanT = sumT / (double)cnt;
  const double meanV = sumV / (double)cnt;

  double cov = 0.0, varT = 0.0;
  for (uint16_t k = 0; k < cnt; k++) {
    const VibHistorySample* s = VibHistory_At(idx[k]);
    const double t  = (double)(uint32_t)(s->timestampMs - t0) / 1000.0;
    const double dt = t - meanT;
    cov  += dt * ((double)s->velocity_rms_overall - meanV);
    varT += dt * dt;
  }

  if (varT <= 0.0) {
    // All samples share one timestamp -- impossible through the ring's
    // strictly-increasing guard, but a slope would be undefined so refuse.
    out->sample_count = cnt;
    out->valid        = false;
    return false;
  }

  const double slope = cov / varT;          // mm/s per second
  if (!isfinite(slope)) {
    out->sample_count = cnt;
    out->valid        = false;
    return false;
  }

  out->slope_mms_per_s = (float)slope;
  out->valid           = true;
  out->reseeded        = splitByGap;
  out->timestamp_ms    = newest->timestampMs;
  out->sample_count    = cnt;
  out->span_ms         = newest->timestampMs - t0;
  return true;
}
