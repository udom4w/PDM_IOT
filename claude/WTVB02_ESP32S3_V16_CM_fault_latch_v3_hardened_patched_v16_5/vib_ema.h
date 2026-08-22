#pragma once
// ============================================================================
// [M1B-2] vib_ema.h -- timestamp-aware EMA over the M1B-1 velocity history ring
//
// Zero Arduino/ESP32/FreeRTOS dependency, matching vib_history.h /
// vib_velocity.h / vib_accel.h, so the alpha derivation, gap handling and
// reseed logic are host-testable under plain g++.
//
// SOURCE OF TRUTH: this module reads ONLY the M1B-1 ring (VibHistory_*). It
// never touches g_velCarrier, g_emaRms, sensorData.rms_overall, or any other
// producer. The ring is the single append point; this is a single consumer of
// it. That is what keeps "what the trend saw" and "what was recorded"
// provably identical.
//
// WHY TIMESTAMP-AWARE:
// The legacy EMA used a fixed alpha (0.20) applied on a fixed 1 Hz tick, which
// encodes tau = -1.0/ln(0.8) = 4.48 s. The FIFO-DSP source does not tick: over
// three 30-minute hardware runs the interval was measured BIMODAL at 2.101 s
// (~45%) and 3.879 s (~55%), mean 3.11 s, with stall gaps to 47.9 s and a
// worst case near 15 minutes. A fixed alpha against that input would give a
// time constant that silently changes with every sample. Deriving alpha from
// the real dt makes tau mean the same thing regardless of when samples land.
//
// SCOPE (M1B-2): EMA only. No slope (M1B-4), no windows (M1B-3), no TTW
// (M1B-5), no TelemetrySlot change (M1B-6).
// ============================================================================

#include <stdint.h>
#include "vib_history.h"

// ---- Locked design parameters (M1B-2 decisions) ----------------------------
// tau = 30 s. Chosen because each velocity sample is already an RMS over 1024
// points (0.512 s of signal), so the input is far less noisy than the legacy
// 250 ms register read and needs less additional smoothing than the raw
// sample count alone would suggest. ~10 samples to 63% at the measured mean.
#define VIB_EMA_TAU_S            30.0f

// dt > 3*tau => the old state describes a machine we have not observed for 90
// seconds. Continuing to decay it would present stale history as current, so
// the EMA is reset and reseeded from the first new valid sample instead.
#define VIB_EMA_RESET_GAP_MS     90000u   // 3 * tau

struct VibEmaState {
  float    ema_mms;       // [mm/s] valid ONLY when valid == true
  bool     valid;         // false => never seeded, or reset and awaiting a sample
  bool     reseeded;      // last update was a seed/reseed rather than a decay step
  uint32_t timestampMs;   // timestamp of the sample that produced ema_mms
  uint32_t updates;       // decay steps applied (excludes seeds)
  uint32_t reseeds;       // seeds + reseeds
  uint32_t consumed;      // history entries examined (valid + gap)
};

// ---- Lifecycle -------------------------------------------------------------
void VibEma_Init(void);
void VibEma_Reset(void);   // forces valid=false; next valid sample reseeds

// ----------------------------------------------------------------------------
// VibEma_Update -- consume every history entry newer than the last one seen.
//
// Walks the ring oldest->newest. For each entry:
//   - gap  : SKIPPED entirely. A gap is never an EMA sample, and its NaN value
//            is never read. Gaps are not themselves a reset trigger -- the
//            reset decision is made purely from dt between consecutive VALID
//            samples, so a short outage is absorbed and a long one reseeds,
//            without needing to interpret the gap marker at all.
//   - valid: if not yet seeded, or dt > VIB_EMA_RESET_GAP_MS -> seed/reseed
//            (ema = sample exactly). Otherwise
//                alpha = 1 - exp(-dt_seconds / tau)
//                ema  += alpha * (sample - ema)
//
// Idempotent: entries already consumed are identified by timestamp and skipped,
// so calling this more often than samples arrive is a no-op. Safe to call every
// analytics tick.
// ----------------------------------------------------------------------------
void VibEma_Update(void);

VibEmaState VibEma_Get(void);

// Exposed for tests and for anyone needing the same derivation.
float VibEma_AlphaFor(uint32_t dtMs);
