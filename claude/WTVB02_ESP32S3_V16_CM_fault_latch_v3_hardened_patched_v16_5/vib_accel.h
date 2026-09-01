#pragma once
// ============================================================================
// [Phase 3B] vib_accel.h -- FIFO RAW -> acceleration RMS (time domain only)
//
// Zero Arduino/ESP32/FreeRTOS dependency by design, matching fifo_types.h /
// fifo_codec.h / fifo_session.h: compiles standalone under a plain C++
// compiler. That is not stylistic -- Phase 3B validation V3 requires the
// exact production arithmetic to be run host-side and compared against an
// independent Python reference, which is only possible if this translation
// unit has no target-only dependency.
//
// SCOPE (Phase 3B, deliberately narrow):
//   IN  : signed int16 raw -> g -> m/s^2 -> per-axis mean removal ->
//         per-axis acceleration RMS -> triaxial vector magnitude.
//   OUT : velocity RMS, FFT, integration, windowing, frequency-domain work.
//         None of that exists here yet and must not be added without its own
//         phase -- see Phase 3 plan.
// ============================================================================

#include <stdint.h>

// ----------------------------------------------------------------------------
// Scaling constants -- APPROVED DESIGN DECISIONS, not tunables.
//
// VIB_ACCEL_LSB_PER_G: WTVB02 RAWFIFO fixed-point scale. A raw int16 sample
//   divided by this yields acceleration in g.
//
// VIB_ACCEL_G_MS2: 9.8 exactly -- NOT 9.80665. This deliberately matches the
//   existing Python reference (analyze_fifo_dewesoft.py) so that V3's
//   production-vs-reference comparison measures implementation error and not
//   a constant mismatch. Changing this to 9.80665 shifts every published
//   acceleration by +0.068% and silently invalidates the V3 baseline.
// ----------------------------------------------------------------------------
#define VIB_ACCEL_LSB_PER_G        2048.0
#define VIB_ACCEL_G_MS2            9.8

// The only sample count Phase 3B accepts. A capture reporting anything else
// is rejected as invalid rather than processed at a different length --
// spec: "requirements: sample_count == 1024 ... otherwise data_valid = false".
#define VIB_ACCEL_REQUIRED_SAMPLES 1024u

// ----------------------------------------------------------------------------
// VibAccelRms -- computed acceleration RMS, all in m/s^2.
//
// `valid` is the sole authority on whether the four RMS fields may be read,
// mirroring FifoCaptureResult's `error`-as-authority contract. When valid ==
// false every RMS field is written to 0.0f (never left indeterminate), but
// that 0.0f means "not computed", NOT "measured zero vibration" -- consumers
// must branch on `valid`, never on a magnitude test.
// ----------------------------------------------------------------------------
struct VibAccelRms {
  float rms_x;
  float rms_y;
  float rms_z;
  float rms_overall;   // sqrt(rms_x^2 + rms_y^2 + rms_z^2) -- triaxial vector magnitude
  bool  valid;

  // ---- [Phase 3G] crest factor, APPENDED -- every field above is unchanged
  // and unreordered, so existing readers are unaffected.
  //
  //   crest_factor = max_i sqrt(dx_i^2 + dy_i^2 + dz_i^2) / rms_overall
  //   d = a - mean(a), per axis, over THIS capture only
  //
  // Numerator and denominator are both DC-removed time-domain acceleration in
  // m/s^2 from the same capture, so the ratio is dimensionless. Taking the peak
  // of the VECTOR magnitude rather than the largest per-axis crest makes it
  // independent of how the sensor happens to be mounted, and keeps it defined
  // when one axis sits near the noise floor -- a per-axis max would divide a
  // small peak by a smaller RMS there and report a meaningless spike.
  //
  // The window is time-domain on purpose. The Hann window used by the velocity
  // path tapers the ends of the capture to zero, so a transient near an edge
  // would be attenuated toward nothing and the answer would depend on where in
  // the buffer the impact happened to land. A spectral peak is worse still: an
  // impulse spreads its energy across bins, so the tallest bin FALLS as the
  // signal becomes more impulsive -- the opposite of what a crest factor means.
  //
  // crest_factor_valid is the sole authority: when false the value is 0.0f
  // meaning "not computed", NEVER "a measured crest factor of zero".
  float crest_factor;
  bool  crest_factor_valid;
};

// ----------------------------------------------------------------------------
// VibAccel_ComputeRms -- the whole of Phase 3B's arithmetic.
//
// Pipeline, executed in exactly the order the Phase 3B spec states it:
//   1. signed int16 raw sample
//   2. / VIB_ACCEL_LSB_PER_G      -> g
//   3. * VIB_ACCEL_G_MS2          -> m/s^2
//   4. per-axis mean removal (DC/offset removal, computed per axis over this
//      capture only -- never carried across captures)
//   5. RMS = sqrt( mean( (a_i - mean_a)^2 ) )
//   6. overall = sqrt(x^2 + y^2 + z^2)
//
// Accumulation is in double, not float: 1024 squared terms accumulated in
// float32 lose enough mantissa to matter against V3's <=0.5% target, and the
// Python reference accumulates in float64. Results are narrowed to float only
// on output, matching the published payload's precision.
//
// VALIDITY GATE (all must hold, else valid=false and NOTHING is computed):
//   - x, y, z all non-NULL
//   - sampleCount == VIB_ACCEL_REQUIRED_SAMPLES
//   - srHz != 0   (Phase 3A fail-closed provenance contract: 0 means the
//     sample rate was never established, so the capture is unattributable)
//
// srHz is NOT used arithmetically here -- time-domain RMS is rate-independent.
// It is gated on solely to enforce the provenance contract, so that no
// published acceleration figure can originate from a capture of unknown rate.
//
// Returns out->valid for caller convenience. `out` must be non-NULL.
// Pure function: no globals, no allocation, no I/O, no blocking. Safe to call
// from any task on any core.
// ----------------------------------------------------------------------------
bool VibAccel_ComputeRms(const int16_t* x,
                         const int16_t* y,
                         const int16_t* z,
                         uint16_t       sampleCount,
                         uint32_t       srHz,
                         VibAccelRms*   out);
