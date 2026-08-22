// ============================================================================
// [M1B-3] vib_window.cpp -- time-based trend windows over the M1B-1 ring
// See vib_window.h for the contract and the no-zero-fill / no-interpolation
// guarantees.
// ============================================================================

#include "vib_window.h"
#include <math.h>
#include <string.h>

bool VibWindow_Compute(uint32_t windowMs, uint32_t nowMs, VibWindowStats* out) {
  if (out == NULL) {
    return false;
  }

  // Fail closed: write the whole struct before any early return so a caller can
  // never read a stale or half-updated result. These zeros mean "not computed"
  // and are only ever exposed alongside status == INSUFFICIENT_DATA.
  memset(out, 0, sizeof(*out));
  out->status = VIB_WIN_INSUFFICIENT_DATA;

  const uint16_t n = VibHistory_Count();
  if (n == 0u) {
    return false;
  }

  // Two passes so the standard deviation is computed against the window's own
  // mean rather than via the sum/sumsq identity. Same reasoning as
  // vib_accel.cpp's AxisRms(): with a large DC offset and small AC content the
  // one-pass identity loses precision exactly where the interesting signal is.
  double   sum   = 0.0;
  uint16_t count = 0;
  float    vmin  = 0.0f;
  float    vmax  = 0.0f;
  uint32_t firstTs = 0;
  uint32_t lastTs  = 0;
  bool     haveAny = false;

  for (uint16_t i = 0; i < n; i++) {
    const VibHistorySample* s = VibHistory_At(i);
    if (s == NULL) {
      continue;
    }
    // GAP EXCLUSION -- the single most important line in this file. A gap is
    // skipped entirely: not counted, not summed, not replaced by zero. Its
    // stored value is NaN and is never dereferenced into the arithmetic below.
    if (!s->valid || s->gap) {
      continue;
    }

    // Wrap-safe age. Unsigned subtraction stays correct across the millis()
    // rollover; a sample newer than nowMs (clock skew between producer and
    // caller) yields a huge unsigned age and is excluded, which is the
    // fail-closed direction.
    const uint32_t age = nowMs - s->timestampMs;
    if (age > windowMs) {
      continue;
    }

    const float v = s->velocity_rms_overall;
    if (!isfinite(v)) {
      continue;   // defence in depth; the ring already rejects non-finite
    }

    if (!haveAny) {
      vmin = vmax = v;
      firstTs = lastTs = s->timestampMs;
      haveAny = true;
    } else {
      if (v < vmin) vmin = v;
      if (v > vmax) vmax = v;
      // Entries are walked oldest -> newest, so these hold without comparison,
      // but they are written explicitly rather than assumed.
      if ((int32_t)(s->timestampMs - firstTs) < 0) firstTs = s->timestampMs;
      if ((int32_t)(s->timestampMs - lastTs)  > 0) lastTs  = s->timestampMs;
    }
    sum += (double)v;
    count++;
  }

  if (count < VIB_WIN_MIN_SAMPLES) {
    // Report how many were actually found -- a consumer distinguishing
    // "0 samples, sensor down" from "3 samples, still filling" needs this.
    out->sample_count = count;
    out->status       = VIB_WIN_INSUFFICIENT_DATA;
    return false;
  }

  const double mean = sum / (double)count;

  double sumSq = 0.0;
  for (uint16_t i = 0; i < n; i++) {
    const VibHistorySample* s = VibHistory_At(i);
    if (s == NULL || !s->valid || s->gap) {
      continue;
    }
    const uint32_t age = nowMs - s->timestampMs;
    if (age > windowMs) {
      continue;
    }
    const float v = s->velocity_rms_overall;
    if (!isfinite(v)) {
      continue;
    }
    const double d = (double)v - mean;
    sumSq += d * d;
  }

  out->sample_count = count;
  out->mean_mms     = (float)mean;
  out->min_mms      = vmin;
  out->max_mms      = vmax;
  out->stddev_mms   = (float)sqrt(sumSq / (double)count);   // population
  out->first_ts_ms  = firstTs;
  out->last_ts_ms   = lastTs;
  out->coverage_s   = (uint32_t)((lastTs - firstTs) / 1000u);
  out->status       = VIB_WIN_VALID;
  return true;
}

const char* VibWindow_StatusStr(uint8_t status) {
  switch (status) {
    case VIB_WIN_VALID:             return "VALID";
    case VIB_WIN_INSUFFICIENT_DATA: return "INSUFFICIENT_DATA";
    default:                        return "UNKNOWN";
  }
}
