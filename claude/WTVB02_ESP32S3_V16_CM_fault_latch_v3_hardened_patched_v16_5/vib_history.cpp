// ============================================================================
// [M1B-1] vib_history.cpp -- timestamped velocity trend history ring
// See vib_history.h for the contract and the NaN-not-zero gap rationale.
// ============================================================================

#include "vib_history.h"
#include <math.h>
#include <string.h>

namespace {

VibHistorySample s_ring[VIB_HIST_CAPACITY];
uint16_t         s_head  = 0;   // next write index
uint16_t         s_count = 0;   // entries stored (saturates at capacity)
bool             s_haveLast = false;
uint32_t         s_lastTsMs = 0;
VibHistoryStats  s_stats;

const uint16_t MASK = (uint16_t)(VIB_HIST_CAPACITY - 1u);

// Wrap-safe "is `later` strictly after `earlier`?".
// millis() rolls over every ~49.7 days. A plain `later > earlier` would read
// the rollover as time running backwards and reject every sample from then on.
// Casting the unsigned difference to signed keeps the comparison correct across
// the wrap for any real interval (< 24.8 days apart).
bool isAfter(uint32_t earlier, uint32_t later) {
  return (int32_t)(later - earlier) > 0;
}

void push(const VibHistorySample& s) {
  if (s_count == VIB_HIST_CAPACITY) {
    s_stats.overwritten++;
  }
  s_ring[s_head] = s;
  s_head = (uint16_t)((s_head + 1u) & MASK);
  if (s_count < VIB_HIST_CAPACITY) {
    s_count++;
  }
  s_lastTsMs = s.timestampMs;
  s_haveLast = true;
}

// Index of the newest stored entry, or -1 when empty.
int newestIndex() {
  if (s_count == 0u) return -1;
  return (int)((s_head + VIB_HIST_CAPACITY - 1u) & MASK);
}

}  // namespace

void VibHistory_Init(void) {
  memset(s_ring, 0, sizeof(s_ring));
  s_head     = 0;
  s_count    = 0;
  s_haveLast = false;
  s_lastTsMs = 0;
  memset(&s_stats, 0, sizeof(s_stats));
  s_stats.lastReject = VIB_HIST_OK;
}

uint8_t VibHistory_Append(float velocityMmS, uint32_t timestampMs, uint32_t captureId) {
  // Fail closed on anything that is not a physically meaningful reading. A NaN
  // or negative RMS means the producer is broken; storing it would corrupt
  // every window that later averages this ring.
  if (!isfinite(velocityMmS)) {
    s_stats.rejected++;
    s_stats.lastReject = VIB_HIST_REJ_NOT_FINITE;
    return VIB_HIST_REJ_NOT_FINITE;
  }
  if (velocityMmS < 0.0f) {
    s_stats.rejected++;
    s_stats.lastReject = VIB_HIST_REJ_NEGATIVE;
    return VIB_HIST_REJ_NEGATIVE;
  }
  if (s_haveLast && !isAfter(s_lastTsMs, timestampMs)) {
    // Equal or backwards timestamps would make every dt zero or negative and
    // silently destroy the slope in M1B-4. Refuse rather than store.
    s_stats.rejected++;
    s_stats.lastReject = VIB_HIST_REJ_TIME_BACKWARD;
    return VIB_HIST_REJ_TIME_BACKWARD;
  }

  // Guard against the same capture being drained twice. captureId 0 is not
  // treated as a real id (gap markers use it), so it is exempt.
  if (captureId != 0u) {
    const int newest = newestIndex();
    if (newest >= 0) {
      for (uint16_t i = 0; i < s_count; i++) {
        const uint16_t idx = (uint16_t)((s_head + VIB_HIST_CAPACITY - 1u - i) & MASK);
        if (s_ring[idx].valid && s_ring[idx].captureId == captureId) {
          s_stats.rejected++;
          s_stats.lastReject = VIB_HIST_REJ_DUPLICATE_ID;
          return VIB_HIST_REJ_DUPLICATE_ID;
        }
        // Only the recent tail can plausibly collide; stop early.
        if (i >= 8u) break;
      }
    }
  }

  VibHistorySample s;
  s.velocity_rms_overall = velocityMmS;
  s.timestampMs          = timestampMs;
  s.captureId            = captureId;
  s.valid                = true;
  s.gap                  = false;
  s.gapReason            = VIB_GAP_NONE;
  push(s);
  s_stats.appended++;
  return VIB_HIST_OK;
}

uint8_t VibHistory_AppendGap(uint32_t timestampMs, uint8_t gapReason) {
  if (s_haveLast && !isAfter(s_lastTsMs, timestampMs)) {
    s_stats.rejected++;
    s_stats.lastReject = VIB_HIST_REJ_TIME_BACKWARD;
    return VIB_HIST_REJ_TIME_BACKWARD;
  }

  // Coalesce a run of identical gaps. During a 15-minute FIFO stall the
  // producer would otherwise emit hundreds of markers and evict the real
  // history on both sides of the outage -- destroying exactly the context
  // needed to interpret it. One marker plus the neighbouring timestamps
  // already describes the gap's full extent.
  const int newest = newestIndex();
  if (newest >= 0 && s_ring[newest].gap && s_ring[newest].gapReason == gapReason) {
    s_ring[newest].timestampMs = timestampMs;  // extend the existing marker
    s_lastTsMs = timestampMs;
    return VIB_HIST_OK;
  }

  VibHistorySample s;
  // NaN, never 0.0f -- see the header's gap-marker rationale.
  s.velocity_rms_overall = NAN;
  s.timestampMs          = timestampMs;
  s.captureId            = 0u;
  s.valid                = false;
  s.gap                  = true;
  s.gapReason            = gapReason;
  push(s);
  s_stats.gaps++;
  return VIB_HIST_OK;
}

uint16_t VibHistory_Count(void) {
  return s_count;
}

const VibHistorySample* VibHistory_At(uint16_t idxFromOldest) {
  if (idxFromOldest >= s_count) {
    return NULL;
  }
  const uint16_t oldest = (uint16_t)((s_head + VIB_HIST_CAPACITY - s_count) & MASK);
  return &s_ring[(uint16_t)((oldest + idxFromOldest) & MASK)];
}

const VibHistorySample* VibHistory_Latest(void) {
  const int n = newestIndex();
  return (n < 0) ? NULL : &s_ring[n];
}

const VibHistorySample* VibHistory_LatestValid(void) {
  for (uint16_t i = 0; i < s_count; i++) {
    const uint16_t idx = (uint16_t)((s_head + VIB_HIST_CAPACITY - 1u - i) & MASK);
    if (s_ring[idx].valid) {
      return &s_ring[idx];
    }
  }
  return NULL;
}

VibHistoryStats VibHistory_GetStats(void) {
  return s_stats;
}

uint32_t VibHistory_ElapsedMs(uint32_t earlierMs, uint32_t laterMs) {
  return laterMs - earlierMs;   // unsigned subtraction is wrap-correct
}
