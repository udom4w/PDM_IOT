// ============================================================================
// [M1B-2] vib_ema.cpp -- timestamp-aware EMA over the M1B-1 history ring
// See vib_ema.h for the contract and the tau/reset rationale.
// ============================================================================

#include "vib_ema.h"
#include <math.h>
#include <string.h>

namespace {

VibEmaState s_st;

// Timestamp of the newest history entry already examined (valid OR gap), so a
// re-scan never reprocesses one. Separate from s_lastValidTsMs because gaps
// advance the scan position but must not advance the EMA's dt reference.
uint32_t s_lastSeenTsMs = 0;
bool     s_haveSeen     = false;

// Timestamp of the last VALID sample folded into the EMA. This -- not the scan
// position -- is what dt is measured from, which is why a gap between two
// samples widens dt instead of being treated as its own event.
uint32_t s_lastValidTsMs = 0;

bool isAfter(uint32_t earlier, uint32_t later) {
  return (int32_t)(later - earlier) > 0;   // wrap-safe across the ~49.7-day millis() rollover
}

}  // namespace

void VibEma_Init(void) {
  memset(&s_st, 0, sizeof(s_st));
  s_st.valid      = false;
  s_st.reseeded   = false;
  s_lastSeenTsMs  = 0;
  s_haveSeen      = false;
  s_lastValidTsMs = 0;
}

void VibEma_Reset(void) {
  s_st.ema_mms     = 0.0f;
  s_st.valid       = false;
  s_st.reseeded    = false;
  s_st.timestampMs = 0;
  // Deliberately keeps s_lastSeenTsMs: a reset must not cause the whole ring
  // to be replayed from the beginning.
}

float VibEma_AlphaFor(uint32_t dtMs) {
  if (dtMs == 0u) {
    return 0.0f;               // no time passed -> no decay
  }
  const float dtS = (float)dtMs / 1000.0f;
  float a = 1.0f - expf(-dtS / VIB_EMA_TAU_S);
  if (a < 0.0f) a = 0.0f;
  if (a > 1.0f) a = 1.0f;      // very large dt -> alpha saturates at 1 (full replace)
  return a;
}

void VibEma_Update(void) {
  const uint16_t n = VibHistory_Count();

  for (uint16_t i = 0; i < n; i++) {
    const VibHistorySample* s = VibHistory_At(i);
    if (s == NULL) {
      continue;
    }
    // Skip anything already consumed on a previous call.
    if (s_haveSeen && !isAfter(s_lastSeenTsMs, s->timestampMs)) {
      continue;
    }

    // Advance the scan position for EVERY entry, gap included -- otherwise a
    // gap would be re-examined forever.
    s_lastSeenTsMs = s->timestampMs;
    s_haveSeen     = true;
    s_st.consumed++;

    // Gaps are never EMA samples. Their value is NaN and is never read here.
    if (!s->valid || s->gap) {
      continue;
    }

    if (!s_st.valid) {
      // First sample, or the first after a reset: seed exactly.
      s_st.ema_mms     = s->velocity_rms_overall;
      s_st.valid       = true;
      s_st.reseeded    = true;
      s_st.timestampMs = s->timestampMs;
      s_st.reseeds++;
      s_lastValidTsMs  = s->timestampMs;
      continue;
    }

    const uint32_t dtMs = s->timestampMs - s_lastValidTsMs;   // wrap-correct

    if (dtMs > VIB_EMA_RESET_GAP_MS) {
      // Outage longer than 3*tau: the retained average describes a machine we
      // have not observed since. Replace it rather than decay toward the new
      // value from a stale starting point.
      s_st.ema_mms     = s->velocity_rms_overall;
      s_st.reseeded    = true;
      s_st.timestampMs = s->timestampMs;
      s_st.reseeds++;
      s_lastValidTsMs  = s->timestampMs;
      continue;
    }

    const float alpha = VibEma_AlphaFor(dtMs);
    s_st.ema_mms     += alpha * (s->velocity_rms_overall - s_st.ema_mms);
    s_st.timestampMs  = s->timestampMs;
    // Cleared only on a genuine decay step, so the flag stays observable across
    // several 1 Hz publishes after a reseed (samples arrive every ~2-4 s).
    s_st.reseeded     = false;
    s_st.updates++;
    s_lastValidTsMs   = s->timestampMs;
  }
}

VibEmaState VibEma_Get(uint32_t nowMs, uint32_t maxAgeMs) {
  // [R-2] Freshness is applied to the COPY, never to s_st. Clearing the stored
  // seeded flag would make the next valid sample seed instead of decay, silently
  // changing the EMA's own reseed contract (VIB_EMA_RESET_GAP_MS) -- so the stored
  // state is deliberately left alone here.
  VibEmaState out = s_st;
  if (out.valid && (uint32_t)(nowMs - out.timestampMs) > maxAgeMs) {
    out.valid = false;  // stale -> not usable. Value retained, NOT zeroed.
  }
  return out;
}
