# RFC-0006: Adversarial Review — What Is Actually In The FIFO?

**Role of this document:** Independent challenge review. The author's job here
is to try to break the current hypothesis, not confirm it. Per instruction,
confidence is never raised without new experimental evidence — existing
evidence is re-examined for what it actually proves, which in several places
is less than previously implied.

**Confidence tiers used throughout:**
- ✅ **Confirmed** — directly measured/derived with no room for alternative
  interpretation (e.g. CRC match, byte counts).
- 🟡 **Evidence-backed** — data points toward this conclusion and alternatives
  are less likely, but not excluded.
- 🟠 **Weak evidence** — consistent with the data but so is at least one
  competing explanation with similar plausibility.
- ⚪ **Speculation** — plausible-sounding, no supporting data yet.

---

## 1. Re-examining every assumption behind the current hypothesis

**Current hypothesis:** *"The FIFO full dump contains 1024 raw acceleration
samples for X, Y, and Z."*

| Claimed evidence | What it actually proves | What it does NOT prove |
|---|---|---|
| 6144 bytes = 1024 × 3 × 2 | ✅ The payload's **shape** is consistent with 1024 tri-axial 16-bit samples | ⚪ Nothing about what those 16-bit values *represent*. Any signal — raw, filtered, integrated, or an internal DSP working buffer — could be packaged in exactly this shape. Byte-count arithmetic is necessary but nowhere near sufficient evidence for content identity. |
| CRC16 validated | ✅ The bytes were transmitted **without transport corruption** | ⚪ **Nothing** about semantic content. A perfectly CRC-valid frame full of filtered data, compressed data, or an internal FFT buffer would look identical at this level of scrutiny. CRC and content-identity are completely orthogonal claims — this is the single most important correction in this review. |
| Normal polling reads 107 registers; FIFO is a separate transaction | ✅ Protocol-level structural fact | ⚪ Says nothing about FIFO content. A separate transaction type is equally consistent with "raw ADC dump" or "here's my internal FFT scratch buffer, exposed for debugging." |
| DSP features (CF, Kurtosis, Energy) are exported via registers | ✅ Confirms a DSP pipeline exists somewhere in the chip | ⚪ Does **not** establish that the FIFO is the *input* to that pipeline as opposed to an *intermediate* or even a *copy taken after* some preprocessing stage. |
| Decoded FIFO samples show a ~0.98-1.04g magnitude at idle (X≈0.80-0.84g, Y≈-0.56 to -0.60g, Z≈-0.02 to 0.01g, computed directly from the `data_0.bin` FIFO block, not from the AX/AY/AZ registers) | 🟡 Consistent with a DC-coupled signal that includes the gravity vector | 🟠 Does not by itself rule out "lightly filtered" (anti-aliased, DC preserved) acceleration, nor "internal DSP buffer" if that buffer happens to also be DC-coupled. It does meaningfully argue against heavy high-pass/AC-coupled filtering and against velocity (see §2b). |

**Bottom line on current assumptions:** the byte-shape and CRC evidence is
airtight for *structure*, and contributes essentially nothing to *content
identity*. The gravity-magnitude check is the only piece of evidence in the
current record that bears on content at all, and it was drawn from 3 samples
out of 1024 (see §8, missing evidence).

---

## 2. Alternative explanations for a 6144-byte payload

### 2a. Filtered acceleration (anti-aliased or band-limited, not truly "raw")

- **Why it could fit:** Every real ADC pipeline applies *some* anti-aliasing
  filter before sampling — there is no such thing as perfectly unfiltered
  "raw" acceleration in a literal sense. The 1024×3×2 byte shape is identical
  either way.
- **Why it might not fit:** If the filtering is a vibration-band bandpass
  (deliberately removing DC/gravity for a "clean vibration" signal), the
  ~1g magnitude we measured at idle should not appear. We observed a
  magnitude close to 1g, arguing against *aggressive* DC-removing filtering
  specifically — but says nothing about mild anti-aliasing, which is
  functionally indistinguishable from "raw" for this project's purposes.
- **Likelihood:** 🟡 Some anti-aliasing filtering is near-certain (universal
  in ADC design) but irrelevant to the hypothesis as stated. Aggressive
  DC-removing/vibration-band filtering: 🟠 weak evidence against, not
  excluded.

### 2b. Integrated velocity waveform (FIFO already contains velocity, not acceleration)

- **Why it could fit:** The device demonstrably computes velocity elsewhere
  (VX/VY/VZ registers exist). The byte shape is identical for any
  time-series of this length.
- **Why it does NOT fit:** The documented scale factor for this register
  (raw/32768×16g) is an **acceleration full-scale conversion**. If the raw
  16-bit values were actually velocity counts, applying this acceleration
  scale factor would need to coincidentally produce a near-exactly-1g-shaped
  magnitude at idle by pure chance — velocity of a stationary sensor should
  be ~0 in any unit, not ~1. This is a real, if narrow, quantitative
  argument against this alternative, not just a plausibility judgment.
- **Likelihood:** 🟠 Weak evidence against (not fully "confirmed false" —
  see §8 on why this rests on too few samples), but this is currently the
  least-supported alternative on the list.

### 2c. Interleaved / shared internal DSP working buffer

- **Why it could fit:** The DSP needs *some* buffer to compute FFT/energy/
  statistics registers. 1024 is a canonical FFT-friendly power-of-two
  length. It is entirely plausible that the "FIFO" register is a debug
  window into the *exact same memory* the FFT engine reads from — which
  could be pre-windowing, post-windowing, mean-removed, or scaled
  differently than a naive "raw ADC" interpretation assumes.
- **Why it might not fit:** If this buffer were post-windowing (e.g. a Hann
  taper applied before FFT), values near the start/end of the 1024-sample
  block should taper toward zero. **We have not checked this** — our
  existing decode only inspected the first 5 of 1024 samples. This
  alternative is not ruled out; it is simply untested.
- **Likelihood:** 🟠 Weak evidence either way — genuinely open, and possibly
  the most under-examined alternative on this list given how directly
  testable it is (§8).

### 2d. Compressed or delta-encoded data

- **Why it could fit:** Nothing about a 6144-byte payload precludes it
  holding delta-encoded or otherwise compressed values rather than absolute
  16-bit samples.
- **Why it does NOT fit well:** The values we decoded, read naively as
  absolute signed 16-bit acceleration counts, land at a physically sensible
  ~1g magnitude. If these were actually delta-codes, this outcome would
  require a coincidence (an incorrect decode landing suspiciously close to
  a correct physical answer). Parsimony favors the simpler explanation, but
  this is an argument from simplicity, not proof.
- **Likelihood:** ⚪ Speculation-level unlikely, no positive evidence for it,
  moderate evidence against via the parsimony argument above.

### 2e. FFT input buffer specifically

- **Why it could fit:** 1024 is a strong, specific signal that this buffer's
  size was *chosen because* it feeds an FFT — this is not a neutral round
  number.
- **Important note:** this alternative is **not necessarily exclusive** with
  the primary hypothesis. It is entirely possible the raw acceleration
  buffer *is* what feeds the FFT directly (unwindowed), in which case "raw
  acceleration" and "FFT input" are the same object, not competing claims.
  The open question is whether the exposed register is pre- or
  post-windowing, and whether it is a live view or a static copy taken at
  read-time (see §7, protocol inconsistency candidate).
- **Likelihood:** 🟡 Plausible and compatible with, not necessarily opposed
  to, the primary hypothesis.

### 2f. Calibration / bias buffer

- **Why it could fit:** Nothing about the byte count rules this out on its
  own.
- **Why it does NOT fit:** Calibration data is typically a handful of
  coefficients, not 1024 independent timestamped values per axis. We also
  observed sample-to-sample AC variation within the block we decoded
  (e.g. X: 1642, 1729, 1601, 1669, 1706 raw counts across 5 consecutive
  samples) — live-looking variation, not the static repeated values a
  calibration table would be expected to show.
- **Likelihood:** ⚪ Very low — the only alternative on this list with fairly
  strong evidence against it, though still not from a fully rigorous test
  (§8).

### 2g. Mixed metadata (sync words, counters, or status interleaved with data)

- **Why it could fit:** Some vendor protocols interleave framing/sync
  markers into raw buffers.
- **Why it does NOT fit well:** No obviously out-of-range or repeating
  sentinel-like values appeared in the small portion inspected.
- **Why this is NOT a strong conclusion:** Only 5 of 1024 samples per axis
  have been examined. A sync word appearing once every N samples could
  easily be missed entirely by this sample size. **This alternative has not
  been meaningfully tested and should not be treated as ruled out.**
- **Likelihood:** 🟠 Weak evidence against, primarily due to under-testing
  rather than a genuine negative result.

---

## 3. Summary: has any alternative actually been falsified?

**No alternative on this list has been rigorously falsified.** The strongest
argument available (gravity magnitude ≈1g at idle) meaningfully weakens the
"integrated velocity" and "aggressively filtered/AC-coupled" alternatives,
and weakly weakens "calibration buffer." It says nothing useful about
"internal DSP working buffer," "compressed data," "FFT input buffer," or
"mixed metadata" — these remain live alternatives, not eliminated ones.

**Current confidence in the primary hypothesis should be: 🟡 Evidence-backed,
not higher.** The evidence collected so far is consistent with "raw
acceleration" but was never designed to distinguish it from several
competitors. This is the central finding of this review.

---

## 4. Experiments designed to actually distinguish between hypotheses

Each experiment below is designed so that **raw acceleration**, **filtered
acceleration**, **velocity waveform**, and **internal DSP buffer** predict
*different, checkable* outcomes — not just "the current hypothesis is
consistent with this data," which was the flaw in the existing evidence.

### Experiment 1: Static gravity + orientation change

**Procedure:** Capture the FIFO at rest in orientation A, then physically
rotate the sensor 90° (e.g. Z-axis now points where X used to) and capture
again at rest.

| | Raw acceleration (TRUE) | Filtered/AC-coupled accel (if that's what it is) | Velocity waveform (if that's what it is) | Internal DSP buffer (if mean-removed) |
|---|---|---|---|---|
| FIFO waveform | DC level shifts to match new orientation; each axis settles at a new constant matching gravity's new component | DC component absent or heavily attenuated in both orientations; no clear shift | No physically meaningful DC level in either orientation | DC component may be removed/normalized away, no clear shift |
| Expected FFT | Energy concentrated at 0 Hz (DC), no meaningful spectral content otherwise, level differs by orientation | Little to no DC energy in the spectrum, similar in both orientations | Not applicable, no orientation-dependent DC term expected | DC bin near zero regardless of orientation |
| Expected RMS | RMS changes because the constant offset itself changes the computed value if not mean-removed first — records both this raw effect and the "true" AC RMS if you mean-remove during analysis | RMS relatively unaffected by orientation | Same | Same |
| Expected Peak | Peak/amplitude shifts with the new DC level if uncorrected | Unaffected | Unaffected | Unaffected |

**What this test can prove:** if the DC level in the FIFO tracks orientation
change 1:1 with gravity's expected component, that is strong, specific
evidence for "the FIFO contains a DC-coupled signal" (rules out heavy
AC-coupling and rules out velocity, since velocity has no gravity term at
all). It does **not** by itself distinguish "raw" from "internal DSP buffer
that happens to also be DC-coupled."

### Experiment 2: Tap impulse (single, sharp mechanical tap)

**Procedure:** With continuous FIFO capture running, deliver one sharp,
brief tap to the sensor housing.

| | Raw acceleration (TRUE) | Filtered acceleration | Velocity waveform | Internal DSP buffer (windowed) |
|---|---|---|---|---|
| FIFO waveform | A single sharp spike, decaying quickly (high-frequency content, consistent with an impulse response) | A spike, but smoothed/widened compared to raw (filter's impulse response convolved in) | A step-like change that persists (velocity integrates the impulse rather than spiking and returning) | Spike shape distorted depending on where in the 1024-sample window it lands, possibly attenuated if it lands near the window edges (tapering) |
| Expected FFT | Broadband energy across many bins (impulse = wide frequency content) | Energy concentrated below the filter's cutoff, high-frequency content suppressed | Energy concentrated at lower frequencies than the raw impulse would show | Broadband, but possibly asymmetric/distorted if a window function was applied |
| Expected RMS | Brief, sharp increase then rapid return to baseline | Increase, but smaller peak / longer duration than raw | Sustained increase (velocity doesn't return to zero as quickly as acceleration does) | Similar to raw unless the tap coincides with a window edge |
| Expected Peak | High, brief | Lower than raw's peak (attenuated by filtering) | Different character entirely — the "peak" would represent velocity, not acceleration, and would not decay the same way | Depends on window position |

**What this test can prove:** comparing FIFO's raw waveform shape and
decay behavior against what the *register-reported* Peak/RMS/CF/Kurtosis
show for the same tap event is a powerful cross-check — if the FIFO's
impulse response looks like acceleration (sharp, fast-decaying, broadband)
while the *registers* behave consistently with that same tap, this links
the FIFO to the register-computed statistics in a much more direct way
than has been established so far.

### Experiment 3: Rotating motor at known, steady RPM (already have `pump01`)

**Procedure:** Capture FIFO while the motor runs at a precisely known speed
(e.g. 25 Hz / 1500 rpm, using the existing 3-speed test setup).

| | Raw acceleration (TRUE) | Filtered acceleration | Velocity waveform | Internal DSP buffer |
|---|---|---|---|---|
| FIFO waveform | Quasi-sinusoidal at ~25 Hz (plus harmonics), riding on a DC offset from gravity | Same fundamental frequency, but harmonics/high-frequency content suppressed relative to raw | Quasi-sinusoidal at the same frequency but 90° phase-shifted and scaled by 1/ω relative to the true acceleration signal (integration property) | Same as raw unless windowed/mean-removed |
| Expected FFT (computed independently from the FIFO samples) | Strong peak at ~25 Hz (and harmonics), should match the register's HZX/Y/Z closely | Strong peak at ~25 Hz, but reduced harmonic content above the filter cutoff | Peak still at ~25 Hz (integration doesn't change frequency), but the peak's *relative amplitude shape* across harmonics differs — velocity spectra roll off by 1/f relative to acceleration | Depends on windowing |
| Expected RMS (computed from raw FIFO samples, mean-removed) | Should numerically match the register's `RRAX`/`VRMSX`-type outputs reasonably closely if the FIFO genuinely feeds those calculations | Should NOT match `RRAX` (acceleration RMS) if what's in the FIFO is already filtered differently than what feeds that register | Would match `VRMSX` (velocity RMS) far better than `RRAX` — this is a direct, decisive test | Depends on whether the buffer is pre- or post-DSP |
| Expected Peak | Comparable in shape to `VX`-type Peak register if scaling assumptions hold | — | — | — |

**This is the single most decisive test available.** Computing an FFT and
RMS independently from the raw FIFO samples (in software, on the collected
data — no vendor documentation required) and comparing those computed
values against the **already-validated register outputs** (`HZX`, `RRAX`,
`VRMSX`, `AX_ENERGY`, etc.) is a direct, quantitative cross-check. If the
independently-computed FFT peak frequency matches `HZX` and the
independently-computed RMS is close to `RRAX` (not `VRMSX`), that is strong
evidence FIFO = acceleration domain, not velocity domain, and that FIFO
genuinely feeds the register pipeline (not a divergent buffer).

### Experiment 4: Frequency sweep (vary motor speed continuously)

**Procedure:** Run the motor through a slow, continuous speed ramp (e.g.
20→50 Hz) while capturing FIFO blocks at intervals.

| | Raw acceleration (TRUE) | Filtered acceleration | Velocity waveform | Internal DSP buffer |
|---|---|---|---|---|
| Independently-computed FFT peak vs. speed | Should track the true, known motor speed at every point in the ramp | Should also track true speed but with a frequency-dependent attenuation trend at higher speeds (roll-off) | Tracks speed correctly (frequency is integration-invariant) but amplitude trend vs. speed should show acceleration-like ω² growth if wrongly assumed to be acceleration, or flat/1/ω trend if correctly treated as velocity |
| Amplitude vs. speed trend | Roughly proportional to (speed)² for constant displacement amplitude (a basic vibration-physics relationship, established earlier in this project) | Same shape, attenuated at higher frequencies if a low-pass filter is present | Roughly flat / much weaker speed-dependence than acceleration, since velocity = ∫acceleration |

**What this can prove:** if FIFO-derived amplitude grows with speed the way
acceleration should (not the way velocity should — established earlier in
this project via the `A=2πfV`, `V=2πfD` relationships), this is a strong,
physics-based discriminator between the acceleration and velocity
hypotheses, independent of any vendor claim.

### Experiment 5: Sampling-rate (SR register) variation

**Procedure:** Change the `SR` register (e.g. from SR5/1kHz to SR3/4kHz)
and capture FIFO blocks at both settings, holding the physical vibration
source constant.

| | Raw acceleration (TRUE, sampled at the configured SR) | Internal DSP buffer (fixed internal rate, independent of SR) |
|---|---|---|
| Apparent time span of 1024 samples | Should change proportionally with SR (e.g. half as much real time covered per 1024-sample block at 2x the sample rate) | May stay the same regardless of the SR register if the buffer is drawn from an internal, fixed-rate pipeline stage unrelated to the user-configurable SR |
| Independently-computed FFT frequency resolution | Bin spacing = SR/1024, should shift predictably with SR | May not shift if the underlying buffer's actual rate is decoupled from the SR setting |

**What this can prove:** if the FIFO's effective time-domain sample rate
does **not** track the SR register as expected, that is a direct,
falsifying result against "raw ADC samples at the configured rate" and
would point toward an internal, SR-independent DSP buffer instead. This is
one of the cheapest, fastest experiments to run and should be prioritized.

---

## 5. Firmware modifications to run these experiments

The current `WTVB05_ValidationTool` reads individual registers (Peak, RMS,
Freq, Accel, CF, Kurtosis, Temp, config) but has **never read the FIFO
register (0x2C) at all** — this is a real gap, since none of the
experiments above are runnable without it.

**Proposed additions (design only — not yet implemented):**

1. **A dedicated FIFO capture command** (e.g. a serial command `FIFO`) that
   issues the 3-transaction FIFO read sequence already reverse-engineered in
   this project (progress-check frames, then the 6149-byte full dump),
   decodes all 1024×3 samples (not just the first 5), and writes them to a
   raw CSV (`sample_index, X_raw, Y_raw, Z_raw, X_g, Y_g, Z_g`) — this
   directly enables checking the *entire* buffer for windowing/tapering
   (addresses the untested edge-of-buffer concern in §2c/§2g), not just the
   first few samples as done so far.
2. **An on-device or post-processing FFT** computed directly from the
   captured FIFO buffer (e.g. via a Python script reading the CSV, using
   `numpy.fft`) — this is required for Experiments 3 and 4 (comparing
   FIFO-derived frequency/RMS against the register-reported values).
3. **Simultaneous capture**: trigger a FIFO read AND a normal 107-register
   poll as close together in time as possible (same loop iteration), so the
   FIFO-derived statistics and the register-reported statistics describe
   (approximately) the same physical instant — needed for Experiments 2-4's
   cross-checks to be meaningful.
4. **SR-tagged FIFO captures**: extend the FIFO CSV output to also log the
   current SR register value, supporting Experiment 5 without needing a
   separate tool.
5. **Orientation/event marker**: a simple serial command (e.g. `MARK tap` or
   `MARK orientation_A`) that inserts an annotation row into the FIFO CSV
   log at the moment of a physical action (tap, tilt, etc.) — without this,
   correlating a physical event to its exact position in a 1024-sample
   buffer is guesswork.

None of this has been implemented yet — it is a design proposal, consistent
with this project's practice of designing before coding (see RFC-0004 §5,
RFC-0005 §8).

---

## 6. Protocol inconsistencies that could falsify the current architecture

Candidates worth actively checking for (not yet checked):

1. **FIFO "progress" values exceeding the buffer size.** The progress-report
   frames observed earlier (`50 03 01 XX XX ...`) reported values like 1515,
   3027, 4557, 6069 — all less than 6144, consistent with "bytes filled so
   far." If a future capture ever shows a progress value *exceeding* 6144,
   that would directly falsify the simple "buffer fill counter" reading of
   those frames.
2. **Timing of FIFO readiness vs. configured SR.** If the buffer fills in a
   time inconsistent with 1024 samples at the configured sample rate (e.g.
   fills much faster or slower than 1024/SR seconds), that would suggest the
   FIFO is not being filled by the live ADC stream at the configured rate.
3. **FIFO values during a `MODE` change.** If switching `MODE` (Time-domain
   vs. Frequency-domain algorithm) changes the FIFO's apparent content
   character (e.g. suddenly looks windowed, or DC disappears), that would
   suggest the FIFO is downstream of the mode-dependent processing, not a
   raw, mode-independent ADC tap.
4. **Value clipping/saturation patterns.** True raw ADC data saturates at
   the full-scale limit (±32767 raw, ±16g) during a hard impact; a
   processed/filtered/compressed buffer might show different clipping
   behavior (e.g. soft-clipping, or no saturation because filtering
   attenuated the transient before it reached this buffer). Not yet
   checked.

---

## 7. Missing evidence (what would most change this assessment)

In priority order:

1. **The other 1019 of 1024 samples in the FIFO block have never been
   decoded or inspected** — every conclusion so far rests on 3-5 samples out
   of 1024. This is the single largest evidence gap in the entire project.
2. **No independent FFT/RMS has ever been computed from FIFO data and
   compared to the register-reported values.** This is the most decisive
   test available (Experiment 3) and has not been run.
3. **No test has varied SR and re-captured the FIFO** — Experiment 5 is
   cheap and has not been attempted.
4. **No orientation-change test has been run** — Experiment 1 is simple and
   directly tests the gravity-DC hypothesis more rigorously than the
   single-orientation idle capture used so far.
5. **No tap/impulse test with FIFO capture has been run** — only register-
   level (Peak/RMS/Freq) tap-like anomalies have been observed so far (the
   negative-Peak events), never cross-referenced against a FIFO capture at
   the same instant.
6. **The FIFO has never been captured across different `MODE` settings** —
   needed for the protocol-inconsistency check in §6.3.

---

## 8. Updated confidence table

| Claim | Prior stated confidence | This review's confidence | Basis for change |
|---|---|---|---|
| A 1024×3×2-byte FIFO structure exists and is read via a distinct Modbus transaction | ✅ Confirmed | ✅ **Confirmed** | Unchanged — CRC-validated, byte arithmetic exact |
| FIFO contains **raw acceleration** specifically (vs. filtered/velocity/internal-buffer/compressed/calibration/metadata) | Implied ✅/🟡 | 🟡 **Evidence-backed, downgraded from implied-confirmed** | Byte-shape and CRC do not bear on content identity (§1); only the idle-magnitude check (3-5 samples) supports this, and several alternatives remain untested |
| FIFO excludes heavy AC-coupling / high-pass filtering | 🟡 (implicit) | 🟡 **Evidence-backed** | Gravity-magnitude ≈0.98-1.04g argues against this specifically |
| FIFO is NOT integrated velocity | 🟡 (implicit) | 🟠 **Weak evidence** | Scale-factor argument is real but narrow; not cross-checked against an independent FFT/RMS comparison |
| FIFO is NOT a calibration/bias table | 🟡 (implicit) | 🟠 **Weak evidence** | Sample-to-sample variation argues against, but from a tiny sample |
| FIFO is NOT an internal DSP working buffer (windowed/mean-removed) | Not previously addressed | ⚪ **Speculation / untested** | No evidence collected either way; this is the least-examined alternative relative to how testable it is |
| FIFO is NOT interleaved with metadata/sync words | Not previously addressed | ⚪ **Speculation / untested** | Only 5 of 1024 samples inspected; a periodic sync word could easily be missed |
| FIFO sample rate tracks the configured SR register | Assumed | ⚪ **Speculation / untested** | Experiment 5 has never been run |
| FIFO feeds the same pipeline that produces CF/Kurtosis/Energy registers | Assumed | 🟠 **Weak evidence** | Plausible given DSP-features-exist, but no direct comparison has been made between FIFO-derived and register-reported statistics |

**Overall verdict: the primary hypothesis survives this review as the most
likely single explanation, but "most likely" is not the same as
"confirmed."** Confidence should sit at 🟡 Evidence-backed for the core
raw-acceleration claim, with several specific sub-claims still at 🟠/⚪
pending the experiments in §4. Per the instruction governing this review,
none of these ratings are raised further without new experimental evidence
— this document itself does not run any of the proposed experiments; it
only designs them.
