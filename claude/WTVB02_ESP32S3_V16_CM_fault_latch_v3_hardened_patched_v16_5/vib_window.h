#pragma once
// ============================================================================
// [M1B-3] vib_window.h -- TIME-based trend windows over the M1B-1 history ring
//
// Zero Arduino/ESP32/FreeRTOS dependency, matching vib_history.h / vib_ema.h /
// vib_velocity.h / vib_accel.h, so window boundaries, gap exclusion and the
// insufficient-data rule are host-testable under plain g++.
//
// READS THE RING ONLY. Never g_velCarrier, never sensorData.rms_overall, never
// the legacy g_trendBuf. The ring is the single append point (M1B-1) and this
// is a second read-only consumer of it alongside the EMA (M1B-2).
//
// WHY TIME-BASED, NOT SAMPLE-COUNT:
// The legacy windows were sample counts tuned for a 4 Hz feed --
// TREND_WINDOW_SAMPLES 120 meant "30 s @ 4 Hz". Against the measured FIFO-DSP
// cadence (mean 3.105 s, bimodal 2.1 s / 3.9 s, hardware-confirmed over 551
// captures) that same 120 samples would silently span ~6.2 minutes, a 12x
// stretch. A window defined in seconds means the same thing at any cadence,
// which is the whole point of this sub-phase.
//
// THE 1 s WINDOW IS DELIBERATELY NOT PROVIDED. At ~0.32 Hz a 1-second window
// contains zero samples by construction. Offering it would return
// INSUFFICIENT_DATA forever while implying the measurement is meaningful.
// Retired rather than migrated.
//
// SCOPE (M1B-3): descriptive statistics per window only. No slope (M1B-4),
// no TTW (M1B-5), no TelemetrySlot change (M1B-6). Nothing here writes.
// ============================================================================

#include <stdint.h>
#include "vib_history.h"

// The two Product windows, in milliseconds.
#define VIB_WIN_60S_MS   60000u
#define VIB_WIN_300S_MS  300000u

// ----------------------------------------------------------------------------
// Minimum valid samples before a window reports statistics.
//
// 5 is a floor, not a target: at the measured 3.105 s mean a 60 s window
// normally holds ~19 samples and a 300 s window ~97, so this only trips during
// start-up, a motor stop, or heavy gapping -- exactly the cases where reporting
// a confident mean would be misleading. Standard deviation additionally needs
// at least 2 points to mean anything, so any floor below 2 would be unsound.
// ----------------------------------------------------------------------------
#define VIB_WIN_MIN_SAMPLES 5u

enum VibWindowStatus {
  VIB_WIN_VALID = 0,
  VIB_WIN_INSUFFICIENT_DATA,   // fewer than VIB_WIN_MIN_SAMPLES valid samples in span
};

// ----------------------------------------------------------------------------
// VibWindowStats
//
// `status` is the sole authority. When INSUFFICIENT_DATA every statistic is
// written to 0.0f / 0 -- meaning "not computed", NEVER "measured zero". A
// consumer must branch on status, never on a magnitude, exactly as with
// VibAccelRms.valid and VibHistorySample.valid.
// ----------------------------------------------------------------------------
struct VibWindowStats {
  uint16_t sample_count;   // VALID samples inside the window (gaps never counted)
  float    mean_mms;
  float    min_mms;
  float    max_mms;
  float    stddev_mms;     // POPULATION stddev (divisor N), consistent with the RMS
                           // convention used throughout this firmware
  uint32_t first_ts_ms;    // timestamp of the oldest included sample
  uint32_t last_ts_ms;     // timestamp of the newest included sample
  uint32_t coverage_s;     // (last_ts - first_ts) / 1000 -- ACTUAL span covered,
                           // which for a gappy window is less than the nominal
                           // window length. Reported so a consumer can tell
                           // "60 s of data" from "3 samples spread over 60 s".
  uint8_t  status;         // VibWindowStatus
};

// ----------------------------------------------------------------------------
// VibWindow_Compute -- statistics over [nowMs - windowMs, nowMs].
//
// Walks the ring and includes a sample only when ALL hold:
//   - entry.valid  (a gap is skipped outright; its NaN is never read, and it is
//     never substituted with 0 -- there is no zero-fill path in this function)
//   - (nowMs - entry.timestampMs) <= windowMs, computed with wrap-safe unsigned
//     arithmetic so the ~49.7-day millis() rollover does not empty the window
//
// No interpolation of any kind: a FIFO gap leaves a real hole in time, and the
// hole is reported via coverage_s rather than filled in. Two samples 47.9 s
// apart stay two samples, not a synthesised series.
//
// Returns true when status == VIB_WIN_VALID. `out` must be non-NULL.
// Pure: no globals of its own, no allocation, no I/O, no blocking.
// ----------------------------------------------------------------------------
bool VibWindow_Compute(uint32_t windowMs, uint32_t nowMs, VibWindowStats* out);

const char* VibWindow_StatusStr(uint8_t status);
