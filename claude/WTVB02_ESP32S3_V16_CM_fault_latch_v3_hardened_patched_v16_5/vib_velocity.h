#pragma once
// ============================================================================
// [Phase 3C] vib_velocity.h -- FIFO RAW -> velocity RMS (frequency domain)
//
// Zero Arduino/ESP32/FreeRTOS dependency, exactly like vib_accel.h and for the
// same reason: Phase 3C validation requires the EXACT production
// implementation to be run host-side against an independent Python reference.
// That is also why the FFT below is a portable radix-2 written in this
// translation unit rather than esp-dsp's dsps_fft2r -- esp-dsp exists only on
// the target, so using it would mean host-validating a re-implementation
// instead of the shipped code, which is not what was asked for.
//
// METHOD (frequency-domain integration -- NO time-domain integration anywhere
// in this file; there is no cumulative sum, no trapezoid, no leaky integrator):
//   1. DC-removed acceleration in m/s^2 -- identical conversion to Phase 3B
//      (this file includes vib_accel.h and uses ITS constants; the two can
//      never drift apart because there is only one definition).
//   2. symmetric Hann window
//   3. 1024-point FFT
//   4. NPG = mean(w^2)                      <-- power normalization, NOT
//                                               coherent gain (mean(w))
//   5. ms_a[k] = |X[k]|^2 / (NPG * N^2), x2 on interior bins only
//      (DC and Nyquist are never doubled -- they have no negative-frequency
//       twin to fold in)
//   6. f[k] = k * srHz / N
//   7. ms_v[k] = ms_a[k] / (2*pi*f[k])^2, only for k >= VIB_VEL_HP_BIN
//   8. sum
//   9. sqrt(sum) * 1000 -> mm/s
//
// The high-pass floor is what makes step 7 safe: 1/(2*pi*f)^2 diverges as
// f->0, so the near-DC bins would amplify residual offset and window leakage
// without bound. Gating at bin 8 removes k=0 (division by zero) and the first
// seven bins along with it.
// ============================================================================

#include <stdint.h>
#include "vib_accel.h"   // VIB_ACCEL_LSB_PER_G, VIB_ACCEL_G_MS2, required N

// ----------------------------------------------------------------------------
// Locked parameters (Phase 3C spec -- not tunables).
//
// VIB_VEL_FFT_N: fixed 1024, equal to VIB_ACCEL_REQUIRED_SAMPLES. No zero
//   padding: the transform length IS the capture length, so no energy
//   normalization correction is needed or applied.
//
// VIB_VEL_HP_BIN: high-pass floor expressed in BINS, not Hz, deliberately.
//   The cutoff frequency is therefore derived per capture as
//   VIB_VEL_HP_BIN * srHz / N and automatically tracks the actual sample rate
//   instead of being hard-coded. At the currently provisioned SR4 (2000 Hz)
//   this evaluates to 8 * 1.953125 = 15.625 Hz, matching the spec's stated
//   figure -- but 15.625 is the CONSEQUENCE of the bin floor, never an input.
// ----------------------------------------------------------------------------
#define VIB_VEL_FFT_N     1024u
#define VIB_VEL_HP_BIN    8u
#define VIB_VEL_MS_TO_MMS 1000.0

// ----------------------------------------------------------------------------
// VibVelocityRms -- velocity RMS in mm/s. `valid` is the sole authority, same
// contract as VibAccelRms: when false, every field is 0.0f meaning "not
// computed", NOT "measured zero".
// ----------------------------------------------------------------------------
struct VibVelocityRms {
  float rms_x;
  float rms_y;
  float rms_z;
  float rms_overall;   // sqrt(x^2 + y^2 + z^2)
  bool  valid;
};

// ----------------------------------------------------------------------------
// VibVelocity_ComputeRms -- Phase 3C's whole arithmetic.
//
// VALIDITY GATE (identical to Phase 3B, checked independently here so the
// module never trusts a caller to have filtered its input):
//   - x, y, z non-NULL
//   - sampleCount == VIB_ACCEL_REQUIRED_SAMPLES (== VIB_VEL_FFT_N)
//   - srHz != 0  (Phase 3A fail-closed provenance)
//
// Unlike the acceleration path, srHz IS used arithmetically here -- it sets
// f[k], and therefore the 1/(2*pi*f)^2 weighting and the high-pass cutoff.
// A wrong srHz would scale every velocity figure, which is precisely why the
// srHz != 0 gate is not optional.
//
// NOT reentrant: uses module-static scratch (FFT buffers, window and twiddle
// tables) to keep 22 KB off the caller's stack. Phase 3C calls it from
// taskAnalytics (Core 1) only -- the single-caller contract that makes this
// safe. Never call it from two tasks concurrently.
//
// Contains no reference to any WTVB02 VRMS register (0x50/0x5C/0x68) -- the
// only input is the RAW FIFO waveform passed in.
// ----------------------------------------------------------------------------
bool VibVelocity_ComputeRms(const int16_t*  x,
                            const int16_t*  y,
                            const int16_t*  z,
                            uint16_t        sampleCount,
                            uint32_t        srHz,
                            VibVelocityRms* out);
