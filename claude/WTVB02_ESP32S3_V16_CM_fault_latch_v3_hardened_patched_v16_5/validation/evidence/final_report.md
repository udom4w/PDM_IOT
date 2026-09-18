# VRMS Calculation Validation — WTVB05 / ESP32-S3 — Final Report

**Read-only review. No source, firmware, configuration, build, flash, or
git state was modified.** Source SHA256 checked at the end of this task:
`4af78715fee526cc17c8e81c52f388fee9b8e60534e12147a46eac1b97b5e659` — this
reflects the two diagnostic-label-only edits applied and hardware-validated
in the immediately preceding task (`T5-CFZ`→`T2a-TEMP`, `250ms`→`500ms`
display text; confirmed still present via `grep` after this review, 5 and 1
occurrences respectively). **This review itself made zero edits to the
`.ino`** — verified by `git diff --stat` showing only the pre-existing
accumulated session diff (2008 insertions / 514 deletions, unchanged shape
from before this task started), and by every source interaction in this
task being a `Read`/`Grep` call only (no `Edit`/`Write` was ever issued
against the `.ino` or any `vib_*`/`fifo_*` file). The only new filesystem
change from this task is the new `validation/` directory itself (untracked,
listed in `git status --short` as `?? validation/`).

---

## 1. Executive Summary

The VRMS calculation's **mathematical formula, sampling parameters, and
axis-combination method were fully traced from source** (Phases 1–3) and
**independently re-implemented in Python from the reconstructed
specification** (Phase 4, not transcribed line-by-line from the C++). That
independent implementation **passes all 12 synthetic self-consistency
tests** (Phase 8) — including exact single-axis/overall equality, exact
√3-scaling for equal tri-axial input, DC/zero-signal rejection, correct
`Fs/N` resolution, correct bin-8=15.625 Hz, and an analytic single-tone RMS
match to 0.017% error.

**However, no real 1024-sample RAW FIFO capture with a paired firmware VRMS
result exists anywhere in this engagement's evidence** (Phase 5 — an
exhaustive search of every log and build artifact produced this session
found zero raw-sample dumps; `DEBUG_FIFO_DUMP`, the only firmware path that
would emit them, was never enabled in any build). **Consequently Phases
6/7/9 — numerical cross-check against real firmware output, intermediate
stage-by-stage comparison, and the spike-origin investigation — could not
be performed**, and per this task's own explicit instruction for this
scenario, this is reported as a source-audit-level result, not a numerical
validation.

**Verdict: BLOCKED — INSUFFICIENT RAW EVIDENCE** (for the numerical
firmware-vs-reference comparison specifically; the source audit and
mathematical self-consistency check are complete and passing — see §12/§13).

---

## 2. Source Trace
Full table in `source_trace.md` (Phase 1). Summary: RAW FIFO reg 0x002C →
`fifo_codec.cpp:75` decode (big-endian, signed int16) → arena → `.ino:6617`
consumer gate (1024/NONE/srHz!=0) → `vib_accel.cpp`/`vib_velocity.cpp` DSP →
`.ino:10970-10987` fresh MQTT publish (`event:"accel_rms"`) and
`.ino:9695-10616` aged (≤10s) snapshot publish (`/vibration`).

## 3. Sampling Validation
| Item | Value | Status |
|---|---|---|
| sample_count | 1024, hardcoded, gated | **PASS** (confirmed in source) |
| Fs | 2000 Hz (SR4), confirmed live in prior boot logs | **PASS** |
| Window duration | 0.512 s | **PASS** (derived, correct) |
| Frequency resolution | 1.953125 Hz | **PASS** (confirmed by formula + independent recompute) |
| Bin 8 | 15.625 Hz | **PASS** |
| Bin range used | k=8..512 inclusive | **PASS** (confirmed in source) |
| Partial capture rejected | Yes, double-gated | **PASS** |
| X/Y/Z alignment | Correct by construction (single decode call per stride) | **PASS** |
| Endianness | Big-endian, confirmed | **PASS** |
| Signed int16 | Correct, documented two's-complement reinterpret | **PASS** |
| Sample duplication/drop | Structurally excluded on the accepted path | **PASS** |

Full detail and exact citations: `source_trace.md` §Phase 2.

## 4. Mathematical Reconstruction
Full derivation with unit tracking at every step: `source_trace.md` §Phase 3.
Formula confirmed: `RMS_overall = sqrt(RMS_x² + RMS_y² + RMS_z²)` (vector
sum, not mean-of-3, not divided by 3).

## 5. Independent Reference Method
`reference_vrms.py` — implements the Phase 3 specification using
`numpy.fft.fft` (a different FFT algorithm from the firmware's hand-rolled
radix-2 DIT), vectorized numpy expressions (not the firmware's explicit
per-bin loop). No calibration factor is applied anywhere in this file.

## 6. Test Captures
**BLOCKED — none available.** See `phase5_raw_evidence_search.md` for the
full search log. No file, in any log produced across this entire
engagement, contains a raw 1024-sample X/Y/Z FIFO capture. Categories A
(normal vibration), B (clear 1x/2x), and C (RMS spike) from the task's
Phase 5 instructions could not be selected because **zero** qualifying
captures exist, not because the selection criteria were unmet among
available ones.

## 7. Numerical Comparison (firmware vs. reference, per real capture)
**Not performed — no input data exists to perform it with** (§6). No
absolute/relative error numbers can be honestly reported for a
firmware-vs-reference comparison, because no capture provides both a raw
1024-sample input and its firmware-computed output from the same instant.

**Tolerance proposed for the record** (so this can be applied the moment a
real capture becomes available, per the task's instruction to state
tolerance *before* seeing results): given the reference and firmware differ
in FFT algorithm and accumulate in double vs. the firmware's mixed
double/float32 narrowing, a **relative error ≤ 0.5%** on `RMS_overall`
(matching the project's own previously-stated V3 validation target, found
in `vib_accel.h`'s and `vib_velocity.h`'s comments referencing "V3's <=0.5%
target") is the reasoned proposal — not picked after the fact, since no
result exists yet to pick it around.

## 8. Intermediate Comparison
**Not performed** — depends on §6/§7's real capture, which does not exist.

## 9. Synthetic Tests (Phase 8)
All 12 tests **PASS** (full detail: `synthetic_test_results.txt`):

| Test | Result |
|---|---|
| 1. Pure sine, known frequency/amplitude | PASS — 0.017% error vs. analytic RMS |
| 1b. Dominant frequency matches known tone | PASS — exact bin match |
| 2. Zero signal | PASS — RMS exactly 0 |
| 3. Constant DC signal | PASS — velocity RMS exactly 0 (DC removal + HP floor) |
| 4. Single-axis vibration | PASS — overall exactly equals that axis |
| 5. Equal X/Y/Z | PASS — overall = √3 × axis RMS, 0.00e+00% error |
| 6. Known 1x+2x components | PASS — 0.299% error vs. analytic two-tone sum |
| Hann/NPG closed-form match | PASS — bit-identical to independent recompute |
| Frequency resolution = Fs/N | PASS — exact |
| Bin 8 = 15.625 Hz | PASS — exact |
| Velocity ∝ 1/ω (amplitude domain) | PASS — ratio 2.0011 vs. expected 2.0 |
| Accel-path single-axis = overall | PASS — exact |

**Disclosed correction:** the first draft of the "velocity scaling" test
asserted an incorrect expected ratio (4.0), conflating the power-domain
`(2πf)²` weighting with its amplitude-domain consequence. This was caught
when the test failed, independently re-derived by hand (`RMS_v ∝ A/ω`,
confirmed against Test 1's already-validated analytic formula), corrected
to the mathematically correct expectation (ratio ≈ 2.0), and re-run. This
is disclosed rather than silently fixed — see `source_trace.md`'s closing
note.

## 10. Spike Investigation (Phase 9)
**Not performed.** No RAW FIFO capture correlated with the ~6 mm/s dashboard
spike (from the prior RCA task) exists in evidence. Whether such a spike
originates in RAW→DSP→MQTT or downstream of MQTT (backend/dashboard)
**cannot be determined from this review** — this restates and does not
contradict the prior RCA's own §K/§L findings, which already declined to
conclude "real" or "fake" without timestamp-matched raw evidence.

## 11. Findings
- The formula, as read from source, is internally consistent and produces
  physically sane results on every synthetic test devised for it.
- `RMS_overall = sqrt(x²+y²+z²)` is confirmed the correct transcription
  (not mean-of-3, not divided by 3) — both by direct source read and by the
  √3-scaling synthetic test.
- No calibration/fudge factor exists anywhere in the traced path.
- The single genuine defect found during this review was in the review's
  **own test-authoring** (an incorrect hand-derived expectation), not in
  the firmware or the reference implementation — caught, corrected, and
  disclosed rather than silently patched.
- The structural absence of any raw-sample capture evidence is itself a
  finding: this firmware has apparently never been run with
  `DEBUG_FIFO_DUMP` enabled during this entire engagement, so no prior task
  could have performed a true numerical validation either — this review is
  not missing evidence that existed and was overlooked; it does not exist.

## 12. Confirmed
- Exact RMS formula, per-axis and overall (source-read, not inferred).
- Sample count (1024), sample rate (2000 Hz, SR4), window duration (0.512s),
  frequency resolution (1.953125 Hz), HP-floor bin (8 = 15.625 Hz), and the
  exact bin range integrated (8..512).
- Big-endian, signed-int16 decode correctness.
- Partial/malformed captures cannot reach the DSP math.
- The reconstructed specification is self-consistent under 12 independent
  synthetic tests, including two exact (zero-error) algebraic identities
  (single-axis=overall; equal-axes=√3×axis).

## 13. Not Confirmed
- That the firmware's actual compiled/running implementation numerically
  matches this reference on a real captured waveform (no raw capture
  evidence exists — Phase 5/6/7 blocked).
- Whether the previously-observed ~6 mm/s dashboard spike originates before
  or after the MQTT boundary (Phase 9 blocked).
- Whether the firmware's `float`-narrowed intermediate values (vs. this
  reference's float64 throughout) introduce any error beyond the ≤0.5%
  band proposed in §7 — untested without real data.

## 14. Remaining Risks
- A defect that exists **only** in the firmware's specific float32/double
  mixed-precision accumulation, or in its hand-rolled radix-2 FFT
  implementation specifically (as opposed to the DFT it's supposed to
  compute), would not be caught by this review, since the reference
  deliberately uses a different FFT algorithm and full double precision
  throughout.
- Without `DEBUG_FIFO_DUMP` evidence, any future RMS anomaly on this device
  will remain equally hard to root-cause at the RAW→DSP boundary until a
  real capture is obtained.

## 15. Recommendation
Enable `DEBUG_FIFO_DUMP` for one bounded, explicitly-authorized diagnostic
session (build + flash + one motor-running capture + immediate revert) to
obtain at least one real 1024-sample raw waveform with its paired firmware
`velocity_rms_x/y/z/overall` output, then re-run Phases 6/7/9 of this
validation against it using the already-built `reference_vrms.py`. This is
a recommendation only — no such change was made in this review.

---

## Final Verdict

# **BLOCKED — INSUFFICIENT RAW EVIDENCE**

(Source audit: **PASS**, all items in §3/§9/§12. Numerical firmware
validation: **not performed**, §6/§7/§10/§13 — this is the reason for the
BLOCKED verdict, not a failure of the algorithm review itself.)
