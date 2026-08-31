// ============================================================================
// [Phase 3C] vib_velocity.cpp -- FIFO RAW -> velocity RMS (frequency domain)
// See vib_velocity.h for the method, the locked parameters, and the rationale
// for a portable FFT instead of esp-dsp.
// ============================================================================

#include "vib_velocity.h"
#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace {

const uint16_t N     = (uint16_t)VIB_VEL_FFT_N;
const uint16_t NHALF = (uint16_t)(VIB_VEL_FFT_N / 2u);

// ---- Module-static scratch -------------------------------------------------
// Deliberately static, never stack: 24 KB of locals would instantly overflow
// taskAnalytics' 6144 B stack. Single-caller contract (see header) is what
// makes shared scratch safe.
float s_re[VIB_VEL_FFT_N];
float s_im[VIB_VEL_FFT_N];

// Hann window and its NPG, built once on first use.
float  s_hann[VIB_VEL_FFT_N];
double s_npg = 0.0;

// Twiddle table: exp(-j*2*pi*m/N) for m = 0..N/2-1. Built once. Without this
// the butterfly loop would call sinf/cosf ~5120 times per axis (~15k per
// capture), which is the single largest avoidable cost in this pipeline.
float s_twRe[VIB_VEL_FFT_N / 2];
float s_twIm[VIB_VEL_FFT_N / 2];

bool s_tablesReady = false;

void BuildTables() {
  if (s_tablesReady) {
    return;
  }
  // SYMMETRIC Hann (denominator N-1), as specified. The periodic form (N)
  // is the usual choice for spectral estimation, but the spec locks the
  // symmetric definition and the NPG below is computed from whatever window
  // is actually built -- so the normalization stays self-consistent either
  // way and cannot silently mismatch the window.
  double sumSq = 0.0;
  for (uint16_t i = 0; i < N; i++) {
    const double w = 0.5 - 0.5 * cos(2.0 * M_PI * (double)i / (double)(N - 1));
    s_hann[i] = (float)w;
    sumSq += w * w;
  }
  s_npg = sumSq / (double)N;   // NPG = mean(w^2) -- power gain, NOT mean(w)

  for (uint16_t m = 0; m < NHALF; m++) {
    const double a = -2.0 * M_PI * (double)m / (double)N;
    s_twRe[m] = (float)cos(a);
    s_twIm[m] = (float)sin(a);
  }
  s_tablesReady = true;
}

// ---- In-place iterative radix-2 complex FFT (decimation in time) ------------
// Operates on s_re/s_im. N is a compile-time power of two (1024), so no
// runtime power-of-two check is needed.
void Fft() {
  // Bit-reversal permutation.
  for (uint16_t i = 1, j = 0; i < N; i++) {
    uint16_t bit = N >> 1;
    for (; j & bit; bit >>= 1) {
      j = (uint16_t)(j ^ bit);
    }
    j = (uint16_t)(j ^ bit);
    if (i < j) {
      const float tr = s_re[i]; s_re[i] = s_re[j]; s_re[j] = tr;
      const float ti = s_im[i]; s_im[i] = s_im[j]; s_im[j] = ti;
    }
  }

  // Butterflies.
  for (uint16_t len = 2; len <= N; len = (uint16_t)(len << 1)) {
    const uint16_t half = (uint16_t)(len >> 1);
    const uint16_t step = (uint16_t)(N / len);   // twiddle stride
    for (uint16_t i = 0; i < N; i = (uint16_t)(i + len)) {
      uint16_t tw = 0;
      for (uint16_t k = 0; k < half; k++) {
        const float wr = s_twRe[tw];
        const float wi = s_twIm[tw];
        tw = (uint16_t)(tw + step);

        const uint16_t a = (uint16_t)(i + k);
        const uint16_t b = (uint16_t)(a + half);

        const float xr = s_re[b] * wr - s_im[b] * wi;
        const float xi = s_re[b] * wi + s_im[b] * wr;

        s_re[b] = s_re[a] - xr;
        s_im[b] = s_im[a] - xi;
        s_re[a] = s_re[a] + xr;
        s_im[a] = s_im[a] + xi;
      }
    }
  }
}

// ---- [Phase 3D] velocity power of ONE bin, from the FFT already in s_re/s_im
// ----------------------------------------------------------------------------
// Reproduces, for a single k, exactly the quantity the RMS loop accumulates:
//     ms_a = |X[k]|^2 / denom, doubled on interior bins,
//     ms_v = ms_a / (2*pi*k*dF)^2
// It is a pure read of s_re/s_im -- it transforms nothing and mutates nothing,
// so calling it cannot perturb the RMS result that was computed before it.
//
// Caller must guarantee VIB_VEL_HP_BIN <= k <= NHALF; that bound is enforced
// at both call sites rather than silently clamped here, so an out-of-range k
// is a caller bug that shows up as one, not as a plausible-looking number.
double VelocityPowerAtBin(uint16_t k, double dF, double denom) {
  const double re = (double)s_re[k];
  const double im = (double)s_im[k];
  double ms_a = (re * re + im * im) / denom;
  if (k != 0u && k != NHALF) {
    ms_a *= 2.0;
  }
  const double f     = (double)k * dF;
  const double omega = 2.0 * M_PI * f;
  return ms_a / (omega * omega);
}

// ---- One axis: DC-removed acceleration -> velocity RMS (m/s) ----------------
// [Phase 3D] outDomHz / outDomValid are OUTPUT-ONLY additions. Everything that
// produces the return value is byte-for-byte the Phase 3C code.
double AxisVelocityRms(const int16_t* s, uint32_t srHz,
                       float* outDomHz, bool* outDomValid) {
  const double toMs2 = VIB_ACCEL_G_MS2 / VIB_ACCEL_LSB_PER_G;

  // Step 1 -- identical conversion and mean removal to Phase 3B. Same
  // constants, same order, same two-pass mean: the velocity path starts from
  // exactly the waveform the acceleration path measured, never a variant.
  double sum = 0.0;
  for (uint16_t i = 0; i < N; i++) {
    sum += (double)s[i] * toMs2;
  }
  const double mean = sum / (double)N;

  // Step 2 -- symmetric Hann, applied to the DC-removed signal. Imag = 0:
  // a real input transformed by a complex FFT.
  for (uint16_t i = 0; i < N; i++) {
    s_re[i] = (float)((((double)s[i] * toMs2) - mean) * (double)s_hann[i]);
    s_im[i] = 0.0f;
  }

  // Step 3
  Fft();

  // Steps 4-8. Accumulate in double: the summed terms span many orders of
  // magnitude once 1/(2*pi*f)^2 weights the low bins far above the high ones.
  const double dF    = (double)srHz / (double)N;          // step 8's dF
  const double denom = s_npg * (double)N * (double)N;     // NPG * N^2
  double msvSum = 0.0;

  for (uint16_t k = VIB_VEL_HP_BIN; k <= NHALF; k++) {    // step 7's HP floor
    const double re = (double)s_re[k];
    const double im = (double)s_im[k];
    double ms_a = (re * re + im * im) / denom;            // step 5

    // x2 for interior bins only -- fold in the negative-frequency twin.
    // k == NHALF is Nyquist and k == 0 is DC; neither has one. (k == 0 is
    // already excluded by the HP floor, but the condition is written in full
    // so the rule is legible rather than implied by an unrelated constant.)
    if (k != 0u && k != NHALF) {
      ms_a *= 2.0;
    }

    const double f = (double)k * dF;                      // step 6: k*srHz/N
    const double omega = 2.0 * M_PI * f;
    msvSum += ms_a / (omega * omega);                     // step 7
  }

  // ==========================================================================
  // [Phase 3D] SECOND SCAN -- everything above this line is Phase 3C, unchanged.
  // ==========================================================================
  // A second pass over the SAME s_re/s_im the loop above just read. No FFT is
  // performed here and msvSum is already final, so the velocity RMS returned
  // below cannot be affected by anything in this block.
  if (outDomHz != NULL)    { *outDomHz = 0.0f; }
  if (outDomValid != NULL) { *outDomValid = false; }

  uint16_t peakBin   = 0u;
  double   peakPower = -1.0;
  for (uint16_t k = VIB_VEL_HP_BIN; k <= NHALF; k++) {
    const double p = VelocityPowerAtBin(k, dF, denom);
    if (p > peakPower) {
      peakPower = p;
      peakBin   = k;
    }
  }

  // Both neighbours must exist for the interpolation, and a peak sitting on
  // either boundary is refused outright: at the HP floor it is indistinguishable
  // from residual DC that the floor exists to suppress, and at Nyquist it is an
  // aliasing indicator rather than a measurement.
  if (peakBin > VIB_VEL_HP_BIN && peakBin < NHALF && peakPower > 0.0) {
    const double pPrev = VelocityPowerAtBin((uint16_t)(peakBin - 1u), dF, denom);
    const double pPeak = peakPower;
    const double pNext = VelocityPowerAtBin((uint16_t)(peakBin + 1u), dF, denom);

    double delta = 0.0;   // 0 == report the raw bin centre
    if (pPrev > 0.0 && pPeak > 0.0 && pNext > 0.0 &&
        isfinite(pPrev) && isfinite(pPeak) && isfinite(pNext)) {
      const double y1  = log(pPrev);
      const double y2  = log(pPeak);
      const double y3  = log(pNext);
      const double den = y1 - 2.0 * y2 + y3;
      if (isfinite(den) && fabs(den) > 1e-12) {
        const double d = 0.5 * (y1 - y3) / den;
        // A true parabola vertex cannot lie further than half a bin from the
        // sample that won the argmax; anything outside that is numerical noise.
        if (isfinite(d) && fabs(d) <= 0.5) {
          delta = d;
        }
      }
    }

    const double fHz = ((double)peakBin + delta) * dF;
    if (isfinite(fHz) && fHz > 0.0) {
      if (outDomHz != NULL)    { *outDomHz = (float)fHz; }
      if (outDomValid != NULL) { *outDomValid = true; }
    }
  }

  return sqrt(msvSum);                                    // step 9 (m/s)
}

}  // namespace

bool VibVelocity_ComputeRms(const int16_t*  x,
                            const int16_t*  y,
                            const int16_t*  z,
                            uint16_t        sampleCount,
                            uint32_t        srHz,
                            VibVelocityRms* out) {
  if (out == NULL) {
    return false;
  }

  out->rms_x       = 0.0f;
  out->rms_y       = 0.0f;
  out->rms_z       = 0.0f;
  out->rms_overall = 0.0f;
  out->valid       = false;
  out->dominant_frequency_x_hz    = 0.0f;
  out->dominant_frequency_y_hz    = 0.0f;
  out->dominant_frequency_z_hz    = 0.0f;
  out->dominant_frequency_x_valid = false;
  out->dominant_frequency_y_valid = false;
  out->dominant_frequency_z_valid = false;

  if (x == NULL || y == NULL || z == NULL) {
    return false;
  }
  if (sampleCount != (uint16_t)VIB_VEL_FFT_N) {
    return false;
  }
  if (srHz == 0u) {
    return false;  // Phase 3A fail-closed: rate never established
  }

  BuildTables();

  // Step 9's mm/s conversion applied once per axis.
  // [Phase 3D] the two extra arguments are outputs; the returned RMS is
  // computed by the same statements as before.
  float dfx = 0.0f, dfy = 0.0f, dfz = 0.0f;
  bool  dvx = false, dvy = false, dvz = false;
  const double vx = AxisVelocityRms(x, srHz, &dfx, &dvx) * VIB_VEL_MS_TO_MMS;
  const double vy = AxisVelocityRms(y, srHz, &dfy, &dvy) * VIB_VEL_MS_TO_MMS;
  const double vz = AxisVelocityRms(z, srHz, &dfz, &dvz) * VIB_VEL_MS_TO_MMS;
  const double vo = sqrt(vx * vx + vy * vy + vz * vz);

  if (!isfinite(vx) || !isfinite(vy) || !isfinite(vz) || !isfinite(vo)) {
    return false;
  }

  out->rms_x       = (float)vx;
  out->rms_y       = (float)vy;
  out->rms_z       = (float)vz;
  out->rms_overall = (float)vo;
  out->valid       = true;

  // [Phase 3D] dominant frequency rides on the same validity as the RMS it was
  // derived from: if the waveform was not good enough to produce a velocity
  // figure, its spectrum peak is not reportable either.
  out->dominant_frequency_x_hz    = dfx;
  out->dominant_frequency_y_hz    = dfy;
  out->dominant_frequency_z_hz    = dfz;
  out->dominant_frequency_x_valid = dvx;
  out->dominant_frequency_y_valid = dvy;
  out->dominant_frequency_z_valid = dvz;
  return true;
}
