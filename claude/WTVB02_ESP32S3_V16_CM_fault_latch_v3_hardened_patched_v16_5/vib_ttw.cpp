// ============================================================================
// [M1B-5] vib_ttw.cpp -- Time-To-Warning from FIFO-DSP velocity + M1B-4 slope
// See vib_ttw.h for the formula, the units derivation, and why
// VIB_TTW_MIN_SLOPE must stay UNSET until re-baselining.
// ============================================================================

#include "vib_ttw.h"
#include <math.h>
#include <string.h>

bool VibTtw_Compute(bool                  motorRunning,
                    const VibSlopeResult* slope,
                    float                 warnThresholdMmS,
                    uint32_t              nowMs,
                    uint32_t              maxAgeMs,
                    VibTtwResult*         out) {
  if (out == NULL) {
    return false;
  }

  // Fail closed and completely before any early return: hours = 0.0f always
  // means "not computed", and is only ever exposed together with a non-VALID
  // status that the caller must branch on.
  memset(out, 0, sizeof(*out));
  out->status = VIB_TTW_THRESHOLDS_UNSET;

  // ---- 1. Configuration ---------------------------------------------------
  // Checked FIRST and unconditionally, so the current production state reports
  // one unambiguous reason rather than whichever downstream gate happens to
  // trip first. Both operands are negative sentinels today, so this is the only
  // reachable outcome until BOTH are re-baselined. No fallback, no default.
  const bool thresholdConfigured = (warnThresholdMmS > 0.0f);
  const bool minSlopeConfigured  = (VIB_TTW_MIN_SLOPE > 0.0f);
  if (!thresholdConfigured || !minSlopeConfigured) {
    out->status = VIB_TTW_THRESHOLDS_UNSET;
    return false;
  }

  // ---- 2. Machine state ---------------------------------------------------
  // A stopped machine is not approaching anything; extrapolating its last
  // known level would predict a failure that cannot occur while it is off.
  if (!motorRunning) {
    out->status = VIB_TTW_MOTOR_STOPPED;
    return false;
  }

  // ---- 3. Current level ---------------------------------------------------
  const VibHistorySample* latest = VibHistory_LatestValid();
  if (latest == NULL) {
    out->status = VIB_TTW_VELOCITY_UNAVAILABLE;
    return false;
  }
  // Wrap-safe age (unsigned subtraction is correct across the millis() rollover).
  const uint32_t age = nowMs - latest->timestampMs;
  if (age > maxAgeMs) {
    out->status = VIB_TTW_VELOCITY_UNAVAILABLE;
    return false;
  }
  const float vNow = latest->velocity_rms_overall;
  if (!isfinite(vNow)) {
    out->status = VIB_TTW_VELOCITY_UNAVAILABLE;
    return false;
  }
  out->v_current_mms = vNow;

  // ---- 4. Slope availability ----------------------------------------------
  if (slope == NULL || !slope->valid) {
    out->status = VIB_TTW_INSUFFICIENT_DATA;
    return false;
  }
  const float s = slope->slope_mms_per_s;
  if (!isfinite(s)) {
    out->status = VIB_TTW_INSUFFICIENT_DATA;
    return false;
  }
  out->slope_used = s;

  // ---- 5. Slope significance ----------------------------------------------
  // Covers negative, zero and merely-too-small in one test. This is the gate
  // that prevents sensor noise being reported as a maintenance forecast; see
  // the measured noise figures in vib_ttw.h.
  if (s < VIB_TTW_MIN_SLOPE) {
    out->status = VIB_TTW_SLOPE_NOT_RISING;
    return false;
  }

  // ---- 6. Gap to threshold ------------------------------------------------
  if (vNow >= warnThresholdMmS) {
    // Already at or past warning: "time to warning" is not a meaningful
    // forward-looking quantity, and returning 0 would read as "warning now"
    // from a prediction engine rather than from the alarm path that owns it.
    out->status = VIB_TTW_ALREADY_AT_THRESHOLD;
    return false;
  }

  // ---- 7. The computation -------------------------------------------------
  //   numerator   [mm/s]
  //   denominator [mm/s per second] * [s/h]  ->  [mm/s per hour]
  //   quotient    ->  HOURS
  // s is strictly positive here (step 5 with a positive MIN_SLOPE), so the
  // division cannot produce a divide-by-zero or a negative horizon.
  const double gap         = (double)warnThresholdMmS - (double)vNow;
  const double ratePerHour = (double)s * 3600.0;
  const double hours       = gap / ratePerHour;

  if (!isfinite(hours) || hours < 0.0) {
    out->status = VIB_TTW_INSUFFICIENT_DATA;
    return false;
  }

  // ---- 8. Horizon ---------------------------------------------------------
  if (hours > (double)VIB_TTW_MAX_HOURS) {
    out->status = VIB_TTW_HORIZON_EXCEEDED;
    return false;   // hours deliberately left 0: no number is published
  }

  out->hours  = (float)hours;
  out->status = VIB_TTW_VALID;
  return true;
}

const char* VibTtw_StatusStr(uint8_t status) {
  switch (status) {
    case VIB_TTW_VALID:                return "VALID";
    case VIB_TTW_THRESHOLDS_UNSET:     return "THRESHOLDS_UNSET";
    case VIB_TTW_MOTOR_STOPPED:        return "MOTOR_STOPPED";
    case VIB_TTW_VELOCITY_UNAVAILABLE: return "VELOCITY_UNAVAILABLE";
    case VIB_TTW_INSUFFICIENT_DATA:    return "INSUFFICIENT_DATA";
    case VIB_TTW_SLOPE_NOT_RISING:     return "SLOPE_NOT_RISING";
    case VIB_TTW_ALREADY_AT_THRESHOLD: return "ALREADY_AT_THRESHOLD";
    case VIB_TTW_HORIZON_EXCEEDED:     return "HORIZON_EXCEEDED";
    default:                           return "UNKNOWN";
  }
}
