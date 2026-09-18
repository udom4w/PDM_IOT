# VRMS Calculation Specification — PROMLOGIX WTVB05 / ESP32-S3 Firmware

**Status:** Engineering specification, derived from source code audit
(Phases 1–3) and hardware-validated against real RAW FIFO captures
(Phases 5–6). Read-only document — describes existing behavior, changes
nothing.

Source SHA256 at time of writing:
`4af78715fee526cc17c8e81c52f388fee9b8e60534e12147a46eac1b97b5e659`
(`.ino`, unchanged by this or any prior validation task).

---

## 1. WTVB05 RAW FIFO

| Item | Value | Source |
|---|---|---|
| Modbus register | **0x002C** | Request built in `taskModbusRead()` broker block (`.ino`, ADR-0006 SCHEDULED-only admission), confirmed on the wire by the earlier RS485 sniffer forensic audit in this engagement |
| Function code | FC03, qty 1 | Same |
| Samples per axis | **1024**, fixed | `fifo_driver.cpp:1175`, `s_result.sampleCount = 1024;` (an assignment, not a measured count — the driver never produces any other value on a successful capture) |
| Sample format | 6-byte stride per sample: X (2 bytes) + Y (2 bytes) + Z (2 bytes), big-endian, signed int16 | `fifo_codec.cpp:75-91` `SampleDecoder_DecodeStride()` |
| X/Y/Z representation | `rawX = (b0<<8)\|b1`, then reinterpreted as signed `int16_t` (two's-complement narrowing, documented GCC-defined behavior) | `fifo_codec.cpp:76-91` |
| `sample_count` gate | Rejected unless exactly 1024 — `VibVelocity_ComputeRms()` (`vib_velocity.cpp:368-370`) and `VibAccel_ComputeRms()` (`vib_accel.cpp:107-109`) both hard-gate `sampleCount != 1024 → return false` | |
| Sample rate | **2000 Hz** (SR4), the currently-provisioned rate; confirmed live at boot (`[SR] Readback = SR4 (2 kHz)`) and in all 24 Phase 6 captures (`sr_hz=2000` in every one) | `.ino` sensor reconfig path; provenance carried per-capture as `g_sensorSrHzVerified` / `FifoCaptureResult.srHz`, never a hardcoded literal in the DSP math |
| Capture/window duration | 1024 / 2000 = **0.512 s** | Derived |

## 2. Raw acceleration conversion

```
a[i] = raw[i] * (VIB_ACCEL_G_MS2 / VIB_ACCEL_LSB_PER_G)
     = raw[i] * (9.8 / 2048.0)                              [m/s^2]
```

- `VIB_ACCEL_LSB_PER_G = 2048.0` — `vib_accel.h:34`. This is the WTVB05
  RAWFIFO fixed-point scale: a raw int16 count divided by this yields `g`.
- `VIB_ACCEL_G_MS2 = 9.8` — `vib_accel.h:35`, **deliberately not 9.80665**;
  the header comment states this is to match an existing Python reference
  (`analyze_fifo_dewesoft.py`) so the firmware's own V3 validation measured
  implementation error, not a constant mismatch.
- Applied identically in both the acceleration path (`vib_accel.cpp:21`,
  `AxisRms()`) and the velocity path (`vib_velocity.cpp:200`,
  `AxisVelocityRms()`, `const double toMs2 = VIB_ACCEL_G_MS2 / VIB_ACCEL_LSB_PER_G;`)
  — one constant pair, used by both, cannot drift apart.

## 3. DC removal

```
mean = (1/N) * sum(a[i])
a_ac[i] = a[i] - mean
```

- Computed **independently per axis**, freshly for **this capture only** —
  no accumulator carries across captures. `vib_velocity.cpp:205-209`:
  ```cpp
  double sum = 0.0;
  for (uint16_t i = 0; i < N; i++) {
    sum += (double)s[i] * toMs2;
  }
  const double mean = sum / (double)N;
  ```
  followed immediately by the window multiplication (§4), which folds the
  subtraction in: `s_re[i] = (float)((((double)s[i] * toMs2) - mean) * (double)s_hann[i]);` (`vib_velocity.cpp:214`).
- The acceleration path (`vib_accel.cpp:24-28`, `AxisRms()`) performs the
  identical two-pass mean/DC-removal, independently, for its own
  (unrelated) time-domain RMS output.

## 4. Hann Window

```
w[i] = 0.5 - 0.5 * cos(2*pi*i / (N-1))          for i = 0 .. N-1
```

- Exact source (`vib_velocity.cpp:50`):
  ```cpp
  const double w = 0.5 - 0.5 * cos(2.0 * M_PI * (double)i / (double)(N - 1));
  ```
- **Symmetric form, denominator `N-1`** — explicitly not the periodic/DFT-
  even form (`vib_velocity.cpp:43-47` comment: *"the spec locks the
  symmetric definition"*).
- **Why applied:** it is the analysis window for the FFT — it tapers the
  edges of the 1024-sample block to reduce spectral leakage from the
  block boundary. **It is a window function, not a calibration factor**:
  it does not scale the physical result by any device- or sensor-specific
  number, and its own power effect is explicitly compensated (§6) before
  any RMS value is produced.

## 5. FFT

| Item | Value | Source |
|---|---|---|
| FFT size | **1024** (= `VIB_VEL_FFT_N`, which is defined equal to `VIB_ACCEL_REQUIRED_SAMPLES`, so the two cannot diverge) | `vib_velocity.h:99` |
| Implementation | In-place iterative **radix-2** decimation-in-time, hand-written (not a library call) | `vib_velocity.cpp:67-105`, `Fft()` |
| Real FFT? | Input is real (`s_im[i] = 0.0f` before the transform, `vib_velocity.cpp:215`) fed through a full complex FFT — not a dedicated real-FFT algorithm, but mathematically equivalent since the imaginary input is zero |
| Frequency-bin calculation | `f[k] = k * dF`, `dF = srHz / N` | `vib_velocity.cpp:223,240` |
| Sample-rate dependency | `dF` (and therefore every `f[k]` and the entire velocity conversion in §8) is computed **from the per-capture `srHz`**, never a hardcoded literal — `vib_velocity.h:163-166` states explicitly: *"A wrong srHz would scale every velocity figure"* | |

## 6. Window power normalization (NPG)

```
NPG = mean(w[i]^2)             (power gain, NOT mean(w))
```

- Exact source (`vib_velocity.cpp:48-54`):
  ```cpp
  double sumSq = 0.0;
  for (uint16_t i = 0; i < N; i++) {
    const double w = 0.5 - 0.5 * cos(2.0 * M_PI * (double)i / (double)(N - 1));
    s_hann[i] = (float)w;
    sumSq += w * w;
  }
  s_npg = sumSq / (double)N;
  ```
- **How this compensates for the Hann window's power loss:** the Hann
  window attenuates the signal's total energy (its coefficients are ≤1 and
  taper to 0 at the edges). Dividing the raw FFT power by `NPG * N^2` (§7)
  restores the spectral power estimate to what it would have been without
  the window's attenuation — this is the standard "power gain"
  normalization for a windowed periodogram, computed here from **the
  actual window array that was built**, not a textbook constant, so it can
  never silently mismatch the window in use (`vib_velocity.cpp:46-47`
  comment: *"the normalization stays self-consistent... cannot silently
  mismatch the window"*).

## 7. Acceleration spectrum / power calculation

```
ms_a[k] = |X[k]|^2 / (NPG * N^2)              for k = VIB_VEL_HP_BIN(8) .. N/2
ms_a[k] *= 2                                   for k != 0 and k != N/2  (interior bins)
```

Exact source (`vib_velocity.cpp:227-238`):
```cpp
for (uint16_t k = VIB_VEL_HP_BIN; k <= NHALF; k++) {
  const double re = (double)s_re[k];
  const double im = (double)s_im[k];
  double ms_a = (re * re + im * im) / denom;            // denom = NPG*N*N
  if (k != 0u && k != NHALF) {
    ms_a *= 2.0;
  }
  ...
```

- **One-sided spectrum handling / interior-bin ×2 rule:** the real FFT of a
  real signal produces a conjugate-symmetric spectrum; the ×2 folds the
  negative-frequency half back into the positive-frequency bins being
  summed, so the loop (which only ever visits `k = 8..512`) still accounts
  for the full signal's power.
- **DC/Nyquist treatment:** `k=0` (DC) and `k=N/2=512` (Nyquist) are
  explicitly **excluded from the ×2 doubling** — neither has a distinct
  negative-frequency twin to fold in. In this pipeline `k=0` never reaches
  this code at all, because the summation range's own lower bound
  (`VIB_VEL_HP_BIN=8`, see §8) already starts above it; the `k != 0u`
  check is retained "so the rule is legible rather than implied by an
  unrelated constant" (`vib_velocity.cpp:233-235` comment).

## 8. Acceleration → velocity conversion

```
f[k]     = k * srHz / N                                    [Hz]
ms_v[k]  = ms_a[k] / (2*pi*f[k])^2                          [(m/s)^2]
```

Exact source (`vib_velocity.cpp:240-242`):
```cpp
const double f = (double)k * dF;                      // step 6: k*srHz/N
const double omega = 2.0 * M_PI * f;
msvSum += ms_a / (omega * omega);                     // step 7
```

- **Why divided by `(2*pi*f)^2`:** this is the standard frequency-domain
  relationship between an acceleration spectrum and the corresponding
  velocity spectrum — for a sinusoidal component, `velocity = acceleration
  / (jω)`, so **power** scales by `1/ω²`. `vib_velocity.h:76-79` states the
  physical reason the summation is bounded away from `f→0`: *"1/(2*pi*f)^2
  diverges as f->0, so the near-DC bins would amplify residual offset and
  window leakage without bound."*
- **Frequency floor / lower bin used:** `VIB_VEL_HP_BIN = 8` —
  `vib_velocity.h:100`. **Corresponding frequency: `8 * (2000/1024) =
  15.625 Hz`** (`vib_velocity.h:92-97` comment states this exact figure as
  the *consequence* of the bin floor at the currently-provisioned 2 kHz
  rate, not an input — the cutoff is expressed in bins so it automatically
  tracks whatever `srHz` a given capture actually carries).
- **Ignored bins:** `k = 0..7` (below the HP floor) and `k > N/2` (above
  Nyquist, not visited by the loop at all) never contribute to `msvSum`.

## 9. Velocity RMS per axis

```
msvSum    = sum_{k=8}^{512} ms_v[k]                          [(m/s)^2]
RMS_axis  = sqrt(msvSum)                                     [m/s]
RMS_axis_mms = RMS_axis * VIB_VEL_MS_TO_MMS = RMS_axis * 1000  [mm/s]
```

- Exact summation range: `k = VIB_VEL_HP_BIN(8)` through `NHALF(512)`
  inclusive — `vib_velocity.cpp:227`, `for (uint16_t k = VIB_VEL_HP_BIN; k <= NHALF; k++)`.
- Final `sqrt` and mm/s conversion: `vib_velocity.cpp:325` (`return sqrt(msvSum);` — m/s)
  then `vib_velocity.cpp:392-397` (`* VIB_VEL_MS_TO_MMS` applied once per axis
  after `AxisVelocityRms()` returns).
- **X, Y, Z are computed fully independently** — three separate calls to
  `AxisVelocityRms()` (`vib_velocity.cpp:392,394,396`), each with its own
  DC removal, own window, own FFT, own spectrum. No axis reads another
  axis's intermediate values at any stage.

## 10. Overall VRMS

```
RMS_overall = sqrt( RMS_x^2 + RMS_y^2 + RMS_z^2 )             [mm/s]
```

Exact source (`vib_velocity.cpp:398`):
```cpp
const double vo = sqrt(vx * vx + vy * vy + vz * vz);
```

**Explicitly confirmed: this is `sqrt(X²+Y²+Z²)` — the triaxial vector
magnitude — NOT `sqrt((X²+Y²+Z²)/3)`.** There is no division by 3, or by
any other count, anywhere in this formula or its callers. The struct's own
doc comment states the same thing independently: `vib_velocity.h:112`,
`float rms_overall;   // sqrt(x^2 + y^2 + z^2)`.

**Meaning:** `RMS_overall` is the RMS of the resultant (vector-sum)
velocity — physically, the magnitude of the combined 3-axis vibration
vector, not an average of the three axes. A machine vibrating equally on
all three axes reports an `Overall` of `√3 ×` any single axis's reading,
not the same as one axis — confirmed on real hardware data in Phase 6 (see
§14) and, before that, on synthetic equal-XYZ input (prior source-audit
task, `synthetic_tests.py`, test `5_equal_xyz_overall_is_sqrt3_times_axis`, 0.00% error).

## 11. Firmware data flow

```
RAW FIFO (Modbus reg 0x002C, 6144-byte payload)
   ↓  fifo_driver.cpp ReadDump()
SampleDecoder_DecodeStride()           [fifo_codec.cpp:75-91 -- big-endian, signed int16, per 6-byte stride]
   ↓
FifoCaptureResult{ x[1024], y[1024], z[1024], captureId, sampleCount, srHz, error, ... }
   ↓  (Core 0, taskModbusRead)
handleFifoCaptureCompletion()          [.ino:6530 -- acquires result, builds "fifo_capture" /event metadata,
   ↓                                     memcpy's x/y/z into g_accelSnap under mutexAccelSnap, zero-copy]
   ↓  cross-core handoff via queueAccelSnapshot (depth 1)
processPendingAccelSnapshot()          [.ino:10922 -- Core 1, taskAnalytics]
   ↓  memcpy g_accelSnap -> g_accelWork (mutex-protected, then lock-free)
VibVelocity_ComputeRms()               [vib_velocity.cpp:330 -- the whole calculation, §2-10 above]
   ↓  produces vel.rms_x/y/z/overall (+ dominant frequency, 1x/2x amplitude)
   ├─→ MQTT "accel_rms" event, published immediately, this capture's fresh values
   │      [.ino:10987, aDoc["velocity_rms_overall"] = vel.rms_overall;  -- topic:
   │       factory/{plant}/machine/{machine}/vibration/event]
   └─→ g_velCarrier                    [.ino:2308 declaration; .ino:11069 g_velCarrier.overall = ...;
          mutex-protected cross-core carrier, consumed later (up to 10 s stale,
          VIB_VELOCITY_MAX_AGE_MS_TBD) by displayVelocity()/dspDominantFreq()/
          dspCrestFactor() for the separately-cadenced "/vibration" MQTT publish
          and the OLED display]
```

## 12. What VRMS does NOT depend on

Verified directly from source, this session and prior audits in this
engagement:

| Candidate dependency | Depends? | Evidence |
|---|---|---|
| Legacy WTVB02 VRMS Modbus registers (0x50/0x5C/0x68) | **No** | `vib_velocity.h:173-177` states explicitly: *"Contains no reference to any WTVB02 VRMS register... the only input is the RAW FIFO waveform."* Those registers still exist in the firmware (`REG_VRMS_X` etc., `.ino:449-451`, tagged `[DEPRECATED]`) but feed only the stuck-axis watchdog, an entirely separate code path, never `VibVelocity_ComputeRms()`'s input |
| FREQ X/Y/Z Modbus registers (0x44-0x46) | **No** | These registers and their read transaction were **removed from the firmware entirely** (confirmed in this engagement's FREQ/CF cleanup task — `.ino:453` comment records the removal); even before removal they were read-only reference values, never an input to the DSP math |
| Crest Factor Modbus registers (0x47/0x53/0x5F) | **No** | Same removal; Crest Factor is separately computed by `VibAccel_ComputeRms()` from the *raw time-domain waveform itself* (`vib_accel.cpp:146-153`), not from any register, and is not an input to `VibVelocity_ComputeRms()` either |
| RPM | **No** (for RMS/overall) — **partially, for 1x/2x only** | `rpmAtCapture` is a parameter to `VibVelocity_ComputeRms()` but is used **only** to locate the 1x/2x harmonic band amplitude (§ Phase 3E in `vib_velocity.h:48-79`); `vib_velocity.cpp:385-387` shows a non-finite or non-positive RPM simply leaves the 1x/2x fields invalid — `msvSum`, `rms_x/y/z/overall` are computed identically regardless |
| CT / current | **No** | No reference to any current-sensor variable (`current_a`, CTR4A01, etc.) anywhere in `vib_accel.cpp`/`vib_velocity.cpp` — confirmed by direct file read, both files have zero Arduino/ESP32 dependency by design (`vib_accel.h:5-10`, `vib_velocity.h:5-11`) |
| MQTT aggregation | **No** | The value published is exactly what `VibVelocity_ComputeRms()` returned for that capture (`.ino:10987`) — no averaging, smoothing, or recomputation occurs between the DSP call and the `serializeJson()`/`enqueueMqttOutbound()` calls that follow it in the same function |
| Dashboard calculations | **NOT CONFIRMED FROM SOURCE** — the dashboard/backend is outside this firmware repository and was not accessible in this engagement (see the earlier RCA task's §I/§J); this document can only state what the firmware itself computes and publishes, not what happens to that number afterward |

## 13. Sampling vs. polling

Two entirely independent cadences exist in this firmware — they must not be
confused:

| | Cadence | Purpose |
|---|---|---|
| **Modbus polling** | `MODBUS_POLL_PERIOD_MS = 500` (`.ino:423`) = **500 ms / 2 Hz** | Normal register reads: VRMS X/Y/Z (stuck-axis watchdog input), TEMPERATURE, CTR4A01 current — the `T1a/T1b/T1c/T2a/T7` transaction sequence |
| **WTVB05 FIFO waveform sampling** | **2000 Hz** internal to the sensor | The rate at which the sensor itself samples acceleration into its FIFO buffer — this is `srHz`, carried per-capture, never confused with the Modbus poll rate |
| Samples per FIFO capture | **1024** | Fixed, §1 |
| Waveform window duration | **0.512 s** (1024/2000) | §1 |
| FIFO capture interval | `FIFO_PERIODIC_INTERVAL_MS = 2000` (`.ino:2211`) = **≈2 seconds** | How often a NEW 1024-sample FIFO capture is requested — independent of, and four times slower than, the 500 ms Modbus poll |

The Modbus poll (500 ms) and the FIFO capture request (2000 ms) are
scheduled independently; a FIFO capture in progress does temporarily
suspend normal Modbus polling for its own duration (bus-ownership handoff,
`FifoDriver_OwnsBus()`), but the two cadence *numbers* themselves are not
derived from one another.

## 14. Phase 6 numerical validation (summary)

From `validation/evidence/phase6_numerical_validation.md` and
`numerical_comparison.csv` (real hardware evidence, not synthetic):

- **24 valid real RAW FIFO captures** (captureId 2-24, 26; captureId 25
  excluded as an INVALID CAPTURE — mid-dump data loss, unrelated to the
  algorithm)
- **1024 samples per axis**, every capture, `sr_hz=2000` every capture
- **96/96 comparisons PASS** (24 captures × {X, Y, Z, Overall}) against an
  independently-implemented Python reference (`reference_vrms.py`, built
  from this same specification's formulas, different FFT algorithm,
  double precision throughout, never given firmware's own RMS numbers as
  input)
- **Tolerance:** ≤0.5%, fixed before the comparison was run
- **Maximum relative error observed: 2.66×10⁻⁴%** (Y-axis, captureId=24) —
  roughly 1,880× smaller than the tolerance
- **No discrepancy requiring investigation** — the small residual (~10⁻⁷
  mm/s absolute) is fully explained by the firmware's `double→float`
  narrowing before publish and by ordinary floating-point rounding
  differences between two independent FFT algorithms computing the same
  DFT, both anticipated in the source audit *before* the comparison ran

## 15. Calibration

**A. Mathematical normalization of the Hann window (NPG, §6):** this IS
present in the firmware and IS necessary — it is not optional and not a
tunable. It restores the spectral power estimate to what an unwindowed
signal's power would be, compensating for the window function's own
attenuation. It is derived from the actual window coefficients
(`s_npg = mean(w²)`), not a fitted or empirical number, and applies equally
regardless of sensor, machine, or mounting.

**B. Sensor/system calibration** (e.g., a multiplier converting the
computed mm/s figure toward some other reference measurement): **no such
factor exists anywhere in this pipeline.** Every constant used
(`VIB_ACCEL_LSB_PER_G=2048.0`, `VIB_ACCEL_G_MS2=9.8`, the Hann/NPG pair,
`VIB_VEL_MS_TO_MMS=1000.0`) is either a documented physical/unit-conversion
constant or a normalization intrinsic to the DSP method itself — none is
described in source, or was found in this engagement's several audits of
this code, as an empirically-fitted calibration coefficient.

**Does current evidence support applying a calibration multiplier? No.**
Phase 6 validated that the firmware's implementation numerically agrees
with an independently-derived reference to within 2.66×10⁻⁴% — i.e., the
firmware is already computing exactly what the stated mathematical
specification says it should, on real captured data. That is evidence the
**implementation** is correct; it says nothing about whether the resulting
mm/s number matches an external reference instrument (no such comparison
has been made — §16). Introducing a multiplier now, without such a
reference measurement, would not correct anything: it would only move an
already mathematically-verified number away from what the stated
specification defines. **No calibration factor is proposed or applied by
this document.**

## 16. Limitations

**Validated:**
- The calculation pipeline described in §2-10 matches the compiled,
  running firmware's actual behavior on real hardware, to within
  floating-point/algorithm-rounding precision (§14).
- This agreement was demonstrated across 24 independent real captures, all
  in a steady-state operating condition (≈0.52-0.62 mm/s overall).
- The `Overall = sqrt(X²+Y²+Z²)` relationship holds exactly, both in the
  firmware and the reference, on every one of those 24 captures.

**NOT validated:**
- **No naturally-occurring high-vibration/spike capture was obtained**
  during the Phase 5 capture session — the numerical agreement in §14 has
  not been tested at large amplitude or under a strongly harmonic (clear
  1x/2x) condition, only at the steady-state level that occurred naturally.
- **Firmware intermediate DSP stages are not externally exposed** — no
  build of this firmware (production or the validation variant used in
  Phase 5) prints the DC-removed signal, windowed signal, raw FFT output,
  or per-bin power spectrum. Validation rests on final-value agreement
  across many real samples, not a stage-by-stage internal trace.
- **Sensor/physical measurement accuracy is not assessed** — this document
  and the Phase 6 evidence it summarizes validate *implementation
  agreement* (does the firmware compute what the specification says),
  never *ground-truth accuracy* (does the specification's output match the
  machine's true physical vibration, or a calibrated external instrument).
- **Dashboard/backend behavior downstream of MQTT is out of scope** — see
  §12's last row.

## 17. Final engineering conclusion

The WTVB05/ESP32-S3 firmware computes velocity RMS entirely from the RAW
FIFO acceleration waveform via frequency-domain integration: DC-removed
acceleration → symmetric Hann window → 1024-point FFT → NPG-normalized,
one-sided power spectrum → division by `(2πf)²` to obtain a velocity power
spectrum, integrated over bins 8 through 512 (15.625 Hz to Nyquist) →
per-axis RMS in mm/s → triaxial vector-sum `Overall = sqrt(X²+Y²+Z²)`. It
depends on no legacy VRMS/FREQ/Crest-Factor Modbus register, and depends on
RPM only for the separately-reported, non-RMS-affecting 1x/2x harmonic
amplitudes. This implementation has been independently re-derived from
source into a separate reference implementation and shown to agree with
the compiled firmware's actual output, on 24 real hardware captures, to
within 2.66×10⁻⁴% — several orders of magnitude inside a pre-declared 0.5%
tolerance, with the residual fully explained by expected numerical
precision effects rather than any uncorrected formula error. This
constitutes validated evidence that **the firmware's VRMS calculation is
implemented as specified**, under the steady-state condition actually
observed; it does not constitute evidence about sensor accuracy,
calibration correctness, or behavior under high-amplitude or strongly
harmonic vibration, which remain unvalidated per §16.
