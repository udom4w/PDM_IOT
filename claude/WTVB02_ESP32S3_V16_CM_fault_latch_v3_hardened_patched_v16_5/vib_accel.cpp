// ============================================================================
// [Phase 3B] vib_accel.cpp -- FIFO RAW -> acceleration RMS (time domain only)
// See vib_accel.h for the contract, the scaling constants' rationale, and the
// explicit list of what is deliberately NOT implemented in this phase.
// ============================================================================

#include "vib_accel.h"
#include <math.h>

namespace {

// Single-axis worker. Two passes over the axis:
//   pass 1 -> mean of the converted signal
//   pass 2 -> mean of the squared deviation from that mean
// A one-pass sum/sumsq variance identity would be cheaper but is numerically
// worse when the DC offset dominates the AC content -- which is exactly the
// regime a mounted accelerometer sits in (gravity on one axis is ~2048 counts
// while the vibration of interest can be single-digit counts). Two passes at
// 1024 samples costs nothing here and keeps V3's <=0.5% target comfortable.
double AxisRms(const int16_t* s, uint16_t n) {
  const double toMs2 = VIB_ACCEL_G_MS2 / VIB_ACCEL_LSB_PER_G;

  // Pass 1: convert to m/s^2 and accumulate the mean.
  double sum = 0.0;
  for (uint16_t i = 0; i < n; i++) {
    sum += (double)s[i] * toMs2;
  }
  const double mean = sum / (double)n;

  // Pass 2: mean-removed sum of squares.
  double sumSq = 0.0;
  for (uint16_t i = 0; i < n; i++) {
    const double d = ((double)s[i] * toMs2) - mean;
    sumSq += d * d;
  }

  return sqrt(sumSq / (double)n);
}

// ---- [Phase 3G] Mean of one axis, in m/s^2 ---------------------------------
// Identical to AxisRms()'s pass 1. Duplicated rather than factored out of
// AxisRms on purpose: leaving AxisRms byte-for-byte untouched is what makes
// rms_x/y/z/overall provably unchanged by this phase.
double AxisMean(const int16_t* s, uint16_t n) {
  const double toMs2 = VIB_ACCEL_G_MS2 / VIB_ACCEL_LSB_PER_G;
  double sum = 0.0;
  for (uint16_t i = 0; i < n; i++) {
    sum += (double)s[i] * toMs2;
  }
  return sum / (double)n;
}

// ---- [Phase 3G] Largest instantaneous vector magnitude, SQUARED ------------
// Returns max over i of (dx^2 + dy^2 + dz^2), not its square root: sqrt is
// monotonic, so taking it once on the winner gives the identical answer for a
// thousandth of the cost of calling it inside the loop.
//
// The three means come first because the peak must be of the DC-removed
// signal. The raw samples carry gravity as a ~9.8 m/s^2 offset on whichever
// axis faces down, and a peak taken over that would be a measurement of
// mounting angle, not of vibration.
double VectorPeakSq(const int16_t* x, const int16_t* y, const int16_t* z,
                    uint16_t n) {
  const double toMs2 = VIB_ACCEL_G_MS2 / VIB_ACCEL_LSB_PER_G;
  const double mx = AxisMean(x, n);
  const double my = AxisMean(y, n);
  const double mz = AxisMean(z, n);

  double peakSq = 0.0;
  for (uint16_t i = 0; i < n; i++) {
    const double dx = ((double)x[i] * toMs2) - mx;
    const double dy = ((double)y[i] * toMs2) - my;
    const double dz = ((double)z[i] * toMs2) - mz;
    const double m2 = dx * dx + dy * dy + dz * dz;
    if (m2 > peakSq) {
      peakSq = m2;
    }
  }
  return peakSq;
}

}  // namespace

bool VibAccel_ComputeRms(const int16_t* x,
                         const int16_t* y,
                         const int16_t* z,
                         uint16_t       sampleCount,
                         uint32_t       srHz,
                         VibAccelRms*   out) {
  if (out == NULL) {
    return false;
  }

  // Fail closed, and write the whole struct before any early return so no
  // caller can ever observe a stale or partially-updated result.
  out->rms_x       = 0.0f;
  out->rms_y       = 0.0f;
  out->rms_z       = 0.0f;
  out->rms_overall = 0.0f;
  out->valid       = false;
  out->crest_factor       = 0.0f;
  out->crest_factor_valid = false;

  if (x == NULL || y == NULL || z == NULL) {
    return false;
  }
  if (sampleCount != VIB_ACCEL_REQUIRED_SAMPLES) {
    return false;
  }
  if (srHz == 0u) {
    return false;  // Phase 3A: 0 == sample rate never established
  }

  const double rx = AxisRms(x, sampleCount);
  const double ry = AxisRms(y, sampleCount);
  const double rz = AxisRms(z, sampleCount);

  // Triaxial vector magnitude (APPROVED: vector magnitude, not per-axis max).
  const double ro = sqrt(rx * rx + ry * ry + rz * rz);

  // A NaN/Inf here would mean corrupt input rather than a physical reading;
  // publishing it would poison every downstream aggregate. Reject instead.
  if (!isfinite(rx) || !isfinite(ry) || !isfinite(rz) || !isfinite(ro)) {
    return false;
  }

  out->rms_x       = (float)rx;
  out->rms_y       = (float)ry;
  out->rms_z       = (float)rz;
  out->rms_overall = (float)ro;
  out->valid       = true;

  // ---- [Phase 3G] crest factor ---------------------------------------------
  // Everything above this line is Phase 3B, unchanged. This block only READS
  // the same input arrays and the `ro` already computed above; nothing it does
  // can alter an RMS figure that was written before it ran.
  //
  // `ro > 0.0` is a division guard, not an amplitude threshold. A capture
  // reaching here with ro == 0.0 would mean 1024 identical samples on all three
  // axes, which is a dead sensor rather than a quiet machine. No minimum
  // amplitude gate is applied and none is needed: FIFO capture is admitted only
  // while the motor is RUNNING (the admission gate in fifo_driver.cpp), so the
  // near-zero-RMS case such a gate would guard against cannot reach this
  // function at all -- measured directly: 8 min 38 s at standstill produced
  // zero captures.
  if (ro > 0.0) {
    const double peakSq = VectorPeakSq(x, y, z, sampleCount);
    const double cf     = sqrt(peakSq) / ro;
    if (isfinite(cf)) {
      out->crest_factor       = (float)cf;
      out->crest_factor_valid = true;
    }
  }
  return true;
}
