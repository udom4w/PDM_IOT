# RFC-0005: Multi-Speed Validation Summary (25 Hz / 35 Hz / 45 Hz)

**Status:** 🟡 Evidence-backed findings from 3 field sessions, single unit (`pump01`).
**Relationship to other documents:** This RFC synthesizes findings across three
independent test sessions to answer open questions raised in the Register
Dictionary (§8.5, §10.1) and RFC-0004 (§5). It does not replace either
document — it feeds validated/falsified conclusions back into them.

## Executive Summary

**Primary Finding:** A negative Frequency–RMS correlation was observed in
all nine axis/speed combinations tested (3 axes × 3 speeds) without
exception. This is the strongest statistical finding in this study — it did not
occur once, but reproduced across every speed and every axis tested.

**Scope of this RFC:** This document validates **repeatability** — that the
same anomalous patterns recur reliably across independent sessions and
operating speeds on this unit. **It does not validate absolute accuracy** —
whether the sensor's Frequency, RMS, or harmonic-lock readings are
*correct* relative to the pump's true mechanical state is a separate
question this RFC cannot answer (see §9, Threats to Validity). Repeatable
and correct are not the same claim.

Secondary findings, in order of confidence: (1) the confidence-score design
misses 50-59% of Physical Invariant Violations, confirmed across two
independent sessions; (2) violation rate does not scale monotonically with
speed (35 Hz > 25 Hz > 45 Hz); (3) Z-axis frequency-lock shifts from the
fundamental to the 2nd harmonic as speed increases — plausible as either a
real mechanical signature or a DSP artifact, and unresolved.

---

## 1. Test conditions summary

| | Test1 | Test2 | Test3 |
|---|---|---|---|
| Setpoint | 25 Hz (1500 rpm) | 35 Hz (2100 rpm) | 45 Hz (2700 rpm) |
| SR | 5 (1 kHz) | 5 (1 kHz) | 5 (1 kHz) |
| Mode | FreqDomain | FreqDomain | FreqDomain |
| Duration | 13.7 min | 27.6 min | ~15 min |
| Samples | 1,644 | 3,301 | 1,834 |
| SampleSeq gaps | 0 | 0 | 0 |
| Config drift (SR/Mode/StaticDetection/DRM) | none | none | none |
| Temp drift | 35.8→38.7°C | 36.8→38.9°C | 34.3→37.9°C |

All three tests ran on the same physical unit, same SR/Mode/config, back to
back — differences below are attributable to motor speed, not test setup.

---

## 2. Physical Invariant Violation — three-way comparison

| | 25 Hz | 35 Hz | 45 Hz |
|---|---|---|---|
| Violation rows (unique) | 2 | 32 | **0** |
| Violation rate | 0.12% | 0.97% | **0.00%** |
| Missed by confidence score | 1/2 (50%) | 19/32 (59.4%) | n/a |
| X+Y simultaneous violation | 0 | 26 (81%) | n/a |

**🔴 Key finding: violation rate is NOT monotonic with speed.** It peaks at
35 Hz and drops to zero at 45 Hz — this falsifies a simple "higher speed =
more DSP stress = more violations" hypothesis. Two candidate explanations,
both speculative:

1. **Resonance-specific**: 35 Hz (or a harmonic of it) may coincide with a
   structural or algorithmic resonance specific to this pump/mount, not a
   general speed-dependent trend.
2. **Non-stationary confound**: Test2 was also the longest session (27.6
   min vs ~14-15 min) — more exposure time alone could explain more events
   without any true rate difference. This has NOT been controlled for
   (violation rate was computed as % of samples, which normalizes for
   duration, but if violations cluster in bursts rather than being
   uniformly distributed over time, a longer test has more opportunity to
   catch a burst).

🟡 **Verdict: unresolved — flagged as an Ambiguous Case, not a conclusion.**
This is exactly the kind of premature generalization the project has
committed to avoiding (see Register Dictionary §8.5 framing precedent).

---

## 3. Confidence score behavior across speeds

| | 25 Hz | 35 Hz | 45 Hz |
|---|---|---|---|
| AnyAnomaly (score-flagged) rate | 6.4% | 5.3% | 7.6% |

Unlike the violation rate, the **score-flagged rate stays in a narrow 5-8%
band across all three speeds** — the confidence score's general sensitivity
appears stable regardless of operating speed. This is a separate signal from
the invariant-violation rate, and the two do not move together (e.g. 35 Hz
has the highest violation rate but not the highest score-flagged rate) —
another reason the two detection layers (score vs. invariant check) need to
stay independent per RFC-0004 §3.5, not merged into one mechanism.

---

## 4. Peak/RMS ratio vs. theory (≈1.414) across speeds

| Axis | 25 Hz median | 35 Hz median | 45 Hz median |
|---|---|---|---|
| X | 1.43 | 1.41 | 1.41 |
| Y | 1.40 | **0.64** | 1.41 |
| Z | 4.23 | 0.53 | 11.84 |

**X-axis stays close to theory (1.41-1.43) at all three speeds** — the most
consistent/reliable axis for this ratio. **Y-axis dropped sharply at 35 Hz**
(0.64, closer to the inverse ratio 0.707) but returned to ~1.41 at 45 Hz —
this is itself odd (not a monotonic drift) and unexplained. **Z-axis ratio is
wildly inconsistent across speeds** (4.23 → 0.53 → 11.84) — combined with the
frequency-lock finding below, Z's Peak/RMS ratio should not be treated as a
stable reference at all; its expected value appears to depend heavily on
which harmonic the axis is currently locked to.

---

## 5. Frequency ↔ RMS correlation — the one fully reproducible finding

| Axis | 25 Hz | 35 Hz | 45 Hz |
|---|---|---|---|
| X | -0.569 | -0.702 | -0.429 |
| Y | -0.494 | -0.458 | -0.331 |
| Z | -0.345 | -0.138 | -0.444 |

**✅ Negative correlation between Frequency and RMS holds in all 9
axis/speed combinations tested (3 axes × 3 speeds), with no exceptions.**
This is the strongest statistical finding in this RFC — promoted from
"evidence-backed" to a pattern the project can treat as reliably reproducible
on this unit under these conditions (still: one unit, one mounting, one set
of test days — not yet cross-unit validated).

### 5.1. Pearson vs. Spearman (X-axis) — the relationship is stronger than first reported

The table above uses Pearson's r, which is sensitive to extreme outliers
(e.g. the Peak/RMS ratio of -1638 observed in the 35 Hz session, §2 above).
Re-computing with Spearman's ρ (rank-based, robust to outliers) on the
X-axis gives a materially different picture:

| Test | Pearson r | Spearman ρ | Samples | Anomalies (`AnyAnomaly=1`) |
|---|---|---|---|---|
| 25 Hz | -0.569 | -0.536 | 1,644 | 106 |
| 35 Hz | -0.702 | -0.664 | 3,301 | 176 |
| 45 Hz | -0.429 | **-0.606** | 1,834 | 139 |

**Spearman ρ is far more consistent across the three speeds (-0.54 to
-0.66) than Pearson r (-0.43 to -0.70).** The 45 Hz session in particular
looked like the weakest correlation under Pearson (-0.429) but is actually
one of the strongest under Spearman (-0.606) — the Pearson value was
depressed by outlier sensitivity, not a genuinely weaker relationship.
**This means the negative Frequency-RMS relationship is more robust and
more uniform across operating speeds than the original Pearson-only table
suggested.** The 9-combination finding above (all axes, all speeds negative)
stands either way — this refinement strengthens confidence in it, it does
not change the conclusion.

**Working interpretation (unchanged from RFC-0004, now cross-validated):**
when the dominant-frequency estimate drifts away from the true value, RMS
tends to degrade at the same time — consistent with a shared or coupled
computation path, not two independent failures. **Alternative explanations
cannot yet be excluded** — the same correlation could also arise from DSP
implementation details, mechanical resonance effects on the raw signal,
filter/window selection inside the sensor, or FFT resolution limits at
certain frequencies. The correlation itself is now firmly established; its
causal explanation is not.

---

## 6. New finding: Z-axis harmonic lock shifts with speed

| Setpoint | Z Frequency mode | Multiplier | Occurrence |
|---|---|---|---|
| 25 Hz | 28 Hz | ≈1× | 42.6% |
| 35 Hz | 70 Hz | **exactly 2×** | 96.6% |
| 45 Hz | 90 Hz (+182 Hz secondary) | **exactly 2× (+~4×)** | 50.2% (+23.1%) |

**At 25 Hz, Z tracks the fundamental (1×). At 35 Hz and 45 Hz, Z locks onto
the 2× harmonic instead, with a growing higher-harmonic (~4×) presence at
45 Hz.** CF (3.46) and Kurtosis (3.24) during the 35 Hz 2×-lock period were
both in normal ranges — no statistical red flag accompanies the shift.

🟡 **Two competing explanations, unresolved:**
1. **Genuine mechanical signature** — 2× dominance is the classic textbook
   indicator of shaft misalignment, and it strengthening with speed (more
   harmonic content at 45 Hz) is physically plausible for a real
   misalignment condition that becomes more pronounced at higher rpm.
2. **DSP/algorithm bias** — the dominant-frequency detector may have a
   structural tendency to lock onto a higher harmonic as the fundamental
   frequency rises, independent of the true mechanical state, possibly
   related to SR5's 4-512 Hz measurable window and how frequency resolution
   or energy-binning behaves near its edges.

**This cannot be resolved from Modbus register logs alone.** Current Modbus
registers are insufficient to distinguish between these hypotheses — the
register map exposes only the final dominant-frequency result and
pre-binned energy totals, not the underlying FFT bin values or the
algorithm's peak-selection logic. It requires either (a) full Energy
spectrum (Point1-8) capture correlated with these timestamps, or (b) an
independent reference measurement (handheld vibration meter, accelerometer
with known-good FFT) on the same pump at the same speeds.

---

## 7. Consolidated findings table

| Finding | 25→35→45 Hz behavior | Status |
|---|---|---|
| Physical Invariant Violation rate | 0.12% → 0.97% → 0% (non-monotonic) | 🟡 Ambiguous Case |
| Confidence score miss rate on violations | 50% → 59.4% → n/a | 🔴 Confirmed problem, not speed-dependent conclusion possible yet |
| Score-flagged (AnyAnomaly) rate | 6.4% → 5.3% → 7.6% (stable band) | ✅ Confirmed stable across speed |
| X-axis Peak/RMS ratio | 1.43 → 1.41 → 1.41 (stable) | ✅ Confirmed reliable axis |
| Y-axis Peak/RMS ratio | 1.40 → 0.64 → 1.41 (dips at 35Hz) | 🟡 Unexplained, non-monotonic |
| Z-axis Peak/RMS ratio | 4.23 → 0.53 → 11.84 (unstable) | ⚪ Not usable as a stable reference |
| Freq↔RMS negative correlation | -0.57/-0.49/-0.35 → -0.70/-0.46/-0.14 → -0.43/-0.33/-0.44 | ✅ Confirmed reproducible, all 9 combinations |
| Z-axis harmonic lock | 1× → 2× → 2×(+4×) | 🟡 Ambiguous — real vs. DSP artifact |

---

## 8. Recommendations

**Reordered per review: ask the vendor before running more experiments that
repeat the same underlying question.** If Witmotion confirms a simple fact
about their algorithm (e.g. whether "Dominant Frequency" is literally
"the FFT bin with maximum energy"), it could resolve the harmonic-lock
question (§6) and possibly the correlation's causal explanation (§5) without
needing another speed sweep at all — asking costs nothing and may make
Recommendation 4 (below) partially or fully unnecessary.

1. **Contact Witmotion with a specific technical question, referencing this
   RFC and its three reproducible datasets, before running further
   experiments under the same conditions.** Concretely ask: (a) is
   "Dominant Frequency" defined as the maximum-energy FFT bin, or something
   else (e.g. a peak-picking algorithm with harmonic rejection)? (b) are
   Velocity RMS and Dominant Frequency computed from a shared intermediate
   result, or fully independent pipelines? A vendor answer to either could
   directly resolve §5's open causal question and/or §6's harmonic-lock
   ambiguity — with three repeatable, well-documented datasets to support
   the question, this has a real chance of getting a substantive answer.
2. **Capture Energy spectrum (Point1-8) alongside future speed sweeps**
   specifically to resolve the Z-axis harmonic-lock question (§6) if the
   vendor cannot answer directly — this is the next most actionable
   experiment, since it could either confirm a real, PdM-relevant
   misalignment signature or rule it out as a DSP artifact.
3. **Test additional speeds** (e.g. 15 Hz, 55 Hz, 65 Hz) to build a real
   speed-vs-violation-rate curve rather than inferring a shape from 3
   points. **These experiments answer a different question from
   Recommendations 1–2** (violation rate vs. speed, §2, rather than the
   harmonic-lock/causal-mechanism question in §5-§6) — this is not a
   duplicate or a fallback if the vendor doesn't respond; it stands on its
   own regardless of the vendor's answer.
4. **Prioritize implementing RFC-0004 §3.5 (Physical Invariant Violation
   check) independent of confidence score** — reconfirmed across two of
   three sessions (25 Hz, 35 Hz) that the score alone misses 50-59% of
   invariant violations. This recommendation is now supported by two
   independent sessions, not one, and does not depend on the vendor
   question above — it can proceed in parallel.
5. **Add "X and Y simultaneously violated" as a named pattern** in the
   invariant framework — seen in 81% of the 35 Hz violations, though absent
   at 25 Hz and n/a at 45 Hz (no violations to check). Track its recurrence
   in future sessions before treating it as a stable signature.
6. **Do not generalize the 35 Hz violation spike as "higher speed = worse"**
   — 45 Hz's zero-violation result directly contradicts that reading. Treat
   speed-vs-violation-rate as an open question, not a trend line.
7. **Do not treat Z-axis Peak/RMS ratio as diagnostic** until the harmonic-
   lock behavior is understood — its "expected" value clearly depends on
   which harmonic is currently dominant, not a fixed physical constant.
8. **Repeat this same 3-speed test on a second physical unit** if/when
   available — every finding in this RFC is currently confounded with
   "this one specific pump, this one mounting" and cannot yet be
   generalized to WTVB05/WTVB02 units in general. See §9 for the full list
   of unaddressed threats to validity.

---

## 9. Threats to Validity

Per review, this section names the confounds this RFC has NOT controlled
for, so its conclusions are read with appropriately calibrated confidence
rather than over-generalized.

| Threat | Present in this RFC? |
|---|---|
| Single sensor | Yes |
| Single motor | Yes |
| Single mounting | Yes |
| Single firmware version | Yes |
| Single sampling rate (SR5) | Yes |
| Cross-unit validation | No |
| External reference instrument | No |
| Manufacturer algorithm undocumented | **Yes** |

Every finding in §1-§7 should be read as **"true for this pump, this
sensor, this mount, this firmware, at SR5"** — not yet established as a
general property of WTVB05/WTVB02 sensors.

**The "manufacturer algorithm undocumented" row is the root cause behind
several other rows in this table** — without knowing how Dominant Frequency
or Velocity RMS are actually computed internally, none of "cross-unit
validation," "external reference instrument," or the harmonic-lock ambiguity
(§6) can be fully closed by more field testing alone, no matter how many
speeds or units are added. This is why §8 Recommendation 1 (asking
Witmotion directly) is ranked above further experiments — it is the one
recommendation capable of closing this specific threat, which the others
cannot.

---

## 10. Figures (Appendix)

Supporting visualizations generated from the same three datasets (X-axis
unless noted; Figure 1 pools X/Y/Z):

| Figure | Content |
|---|---|
| Figure 1 | RMS vs Dominant Frequency scatter, all axes (X/Y/Z) pooled, colored by test speed |
| Figure 2 | Dominant Frequency timeline (Sample vs Freq), X-axis, one panel per test |
| Figure 3 | Velocity RMS timeline (Sample vs RMS), X-axis, one panel per test |
| Figure 4 | Frequency + RMS dual-axis overlay, X-axis, one panel per test — visual check for simultaneous jumps |
| Figure 5 | Peak/RMS ratio histogram, X-axis, all three tests overlaid, theoretical 1.414 marked |
| Figure 6 | X-axis Energy timeline (Sample vs Energy), one panel per test |
| Figure 7 | Frequency vs Peak/RMS ratio scatter, X-axis, all three tests overlaid |

Figures 5 and 7 clip extreme outlier ratios (e.g. -1638 at 35 Hz, §2) for
visual readability — see §5.1 for why Pearson-based statistics on the same
data should be read alongside the outlier-robust Spearman figures instead of
in isolation.
