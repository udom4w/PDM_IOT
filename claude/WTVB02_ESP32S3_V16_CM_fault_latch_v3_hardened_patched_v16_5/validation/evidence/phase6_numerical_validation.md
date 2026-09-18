# PHASE 6 — Numerical VRMS Validation (Firmware vs. Independent Reference)

**Read-only analysis of existing evidence. No source, firmware, config,
build, flash, or git state modified.**

---

## 1. Objective

Determine whether the firmware's compiled, running VRMS implementation
produces numerically the same result as an independent Python reference
(`reference_vrms.py`, built from the reconstructed mathematical
specification — not transcribed from the C++), when both are given the
**same real RAW FIFO 1024-sample capture** from Pump01's WTVB05 sensor.

## 2. Input Evidence

- `validation/evidence/raw_captures/capture_<id>.json` — 24 files, each
  containing the paired `captureId`, `sr_hz`, `sample_count`, raw
  `raw_x/raw_y/raw_z` (1024 int16 samples each), and firmware-computed
  `firmware_rms_x/y/z/overall`, captured from real hardware in the prior
  "CONTROLLED VRMS RAW CAPTURE" session.
- `validation/evidence/raw_captures/captures_summary.csv` — index of the
  same 24 captures.
- captureId **25 excluded** — mid-dump data loss (CSV index gap 321→357)
  found and reported as INVALID in the prior task; not used here.
- captureIds used: **2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17,
  18, 19, 20, 21, 22, 23, 24, 26** (24 total).

## 3. Pairing Verification

Every one of the 24 captures was programmatically re-verified (script:
`phase6_run_comparison.py`) against all required conditions:
`sample_count==1024`, `sr_hz==2000`, `len(raw_x)==len(raw_y)==len(raw_z)==1024`,
and all four firmware RMS fields present. **24/24 passed pairing — 0
excluded at this stage** (captureId 25 was already excluded upstream, before
this script ever ran, per the prior task's own finding). Full per-capture
pairing record: `phase6_stats.json` → `pairing_report`.

## 4. Reference Method

`reference_vrms.py`'s `velocity_rms_triaxial()`, called with **only** the
three raw int16 arrays and `sr_hz` read from each capture's JSON file. See
§7 (Independence Check) for confirmation this never reads or is seeded by
the firmware's own RMS numbers.

## 5. Numerical Comparison — all 24 captures

Full table: `validation/evidence/numerical_comparison.csv`. Representative
rows (min/max error captures) below; every relative error in the complete
set is between **2.7×10⁻⁶% and 2.7×10⁻⁴%** — see §6 for the full range.

| captureId | Firmware_Overall | Reference_Overall | AbsError_Overall | RelError_Overall_% |
|---|---|---|---|---|
| 12 (max Overall error) | 0.613992 | 0.6139924773336047 | 4.77×10⁻⁷ | 8.82×10⁻⁵ |
| 14 (min Overall error) | 0.571179 | 0.571178997998558 | 2.00×10⁻⁹ | 3.50×10⁻⁷ |
| 3 (max X error) | 0.617928 | — | — | 1.24×10⁻⁴ (X-axis) |
| 24 (max Y error) | 0.566944 | — | — | 2.66×10⁻⁴ (Y-axis) |
| 15 (max Z error) | 0.586983 | — | — | 1.23×10⁻⁴ (Z-axis) |

## 6. Error Statistics (24 captures)

| Axis | Mean AbsErr (mm/s) | Max AbsErr (mm/s) | Mean RelErr (%) | Max RelErr (%) | RMS Err (mm/s) | Max-error captureId | Min-error captureId |
|---|---|---|---|---|---|---|---|
| X | 2.82×10⁻⁷ | 5.34×10⁻⁷ | 7.70×10⁻⁵ | 1.29×10⁻⁴ | 3.18×10⁻⁷ | 3 | 4 |
| Y | 2.44×10⁻⁷ | 4.34×10⁻⁷ | 1.35×10⁻⁴ | 2.66×10⁻⁴ | 2.74×10⁻⁷ | 24 | 14 |
| Z | 2.20×10⁻⁷ | 4.80×10⁻⁷ | 5.61×10⁻⁵ | 1.23×10⁻⁴ | 2.68×10⁻⁷ | 15 | 26 |
| Overall | 2.70×10⁻⁷ | 4.77×10⁻⁷ | 4.71×10⁻⁵ | 8.82×10⁻⁵ | 3.05×10⁻⁷ | 12 | 14 |

Full machine-readable statistics: `phase6_stats.json`.

## 7. Tolerance Assessment

**Tolerance: ≤ 0.5%, fixed in advance** (per the prior task's proposal,
before any result was seen — not adjusted here to force a pass).

| Axis | PASS (≤0.5%) | FAIL (>0.5%) | Undefined (Reference≈0) |
|---|---|---|---|
| X | **24** | 0 | 0 |
| Y | **24** | 0 | 0 |
| Z | **24** | 0 | 0 |
| Overall | **24** | 0 | 0 |

**All 96 individual axis/capture comparisons (24 captures × 4 quantities)
PASS.** The observed maximum relative error (2.66×10⁻⁴%, Y-axis,
captureId=24) is **~1,880× smaller** than the 0.5% tolerance — not a
borderline pass.

**No division-by-zero occurred**: no `Reference` value in this dataset was
within 1×10⁻⁹ of zero (all captures are real, non-trivial vibration
signals — the steady-state condition observed, ≈0.5-0.6 mm/s, never
approaches the zero-guard threshold).

**Explanation of the non-zero residual** (disclosed, not glossed over):
the observed ~10⁻⁷ mm/s absolute errors are consistent with two expected,
disclosed sources — neither is a calculation defect:
1. The firmware narrows its internally double-precision accumulation to
   `float` (32-bit) before printing (`vib_velocity.cpp:404-407`,
   `out->rms_x = (float)vx;` etc.), and this task's diagnostic print
   formats with `%.6f` (6 decimal digits) — this alone bounds the
   comparable precision to roughly 10⁻⁶-10⁻⁷ at these magnitudes.
2. The firmware's hand-rolled radix-2 FFT and the reference's
   `numpy.fft.fft` are two different algorithms computing the same DFT;
   floating-point summation order differs between them, producing
   ULP-level rounding differences.

Both explanations were anticipated in `source_trace.md` (Phase 3 task)
*before* this comparison was run — this is confirmatory, not
after-the-fact rationalization.

## 8. First Divergence Analysis

**Not applicable — no divergence was found.** Per instruction, this section
still records what stage-by-stage tracing *was* and *was not* possible:
- The firmware's serial output (both existing `DumpFifoCaptureCsv()` and
  this session's added `[FIFO-VRMS-DUMP]` line) exposes only **raw samples**
  and **final per-axis/overall RMS** — no intermediate stage (DC-removed
  signal, windowed signal, FFT magnitude, PSD, velocity PSD) is ever
  surfaced by the firmware itself, in production or in this validation
  variant. A literal stage-by-stage firmware-vs-reference trace (RAW decode
  → acceleration → DC removal → Hann → NPG → FFT → PSD → velocity PSD →
  integration → axis RMS → overall RMS) was therefore **never possible with
  the available instrumentation**, independent of whether a divergence
  existed.
- Because final-value agreement is already at the float32/FFT-rounding
  noise floor (§7), there is no discrepancy left to localize to a specific
  stage — had one existed above that floor, this limitation would have
  prevented pinpointing *which* stage caused it, and that would need to be
  reported as a further limitation. It does not arise here.

## 9. Overall RMS Verification

Checked independently, per capture, for **both** sides:
- **Firmware**: `sqrt(Firmware_X² + Firmware_Y² + Firmware_Z²) == Firmware_Overall`
  — **True for all 24 captures** (within 1×10⁻⁴, accounting for the
  firmware's own 6-decimal print rounding).
- **Reference**: `sqrt(Reference_X² + Reference_Y² + Reference_Z²) == Reference_Overall`
  — **True for all 24 captures** (within 1×10⁻⁹, full double precision).

This confirms the `Overall = sqrt(X²+Y²+Z²)` relationship (vector sum, not
mean-of-3) holds on **real hardware data**, not just the synthetic tests
from the prior source-audit task.

## 10. Limitations

- **Steady-state only**: all 24 valid captures fall in a narrow band
  (≈0.52-0.62 mm/s overall) — no naturally-occurring high-RMS/spike
  capture (Category C) or clearly 1x/2x-dominant capture (Category B)
  occurred during this session, and none was fabricated to fill the gap
  (per instruction). The agreement demonstrated here is **not yet tested
  under a large-amplitude or strongly harmonic condition**.
- **No firmware intermediate-stage visibility** (§8) — validation rests on
  final-value agreement across 24 independent real samples, not on a
  stage-by-stage internal trace.
- **Per Phase 6.10, explicitly**: this result validates **implementation
  agreement between firmware and an independent reference**, computed from
  the same raw waveform. It does **not** validate, and this report makes
  **no claim about**, WTVB05 sensor accuracy, any calibration factor, or
  whether the reported mm/s values reflect the true physical vibration
  amplitude of the machine. The steady 0.524-0.618 mm/s readings in this
  session are an observation of this session's operating condition only.

## 11. Conclusion

Across 24 real, hardware-captured, sample-count-verified RAW FIFO windows,
the firmware's compiled VRMS output and an independently-implemented
Python reference (different FFT algorithm, full double precision,
mathematics reconstructed from source rather than transcribed) agree to
within 2.66×10⁻⁴% — four orders of magnitude inside the pre-declared 0.5%
tolerance — on every axis and on the overall vector-sum combination. The
residual disagreement is fully and specifically explained by disclosed
float32-narrowing and independent-FFT-rounding effects, not by any
uncorrected formula difference. This is strong evidence the **implementation
is correct** for the steady-state condition actually captured; it is
explicitly not evidence about sensor accuracy, calibration, or behavior
under high-amplitude/spike/strongly-harmonic conditions, which this session
did not naturally produce.

---

## Final Verdict

# **VALIDATED WITH LIMITATIONS**

**Why not unqualified VALIDATED:** all 96 numerical comparisons pass with
enormous margin (§7) — the *calculation* is validated. The qualifier is for
**coverage**, not accuracy: only Category A (steady-state) data was
available (§10), and the firmware exposes no intermediate-stage
diagnostics, so §8's stage-by-stage requirement could not be structurally
exercised even though it was not needed this time.

**Why not NOT VALIDATED:** no unexplained numerical discrepancy exists —
every residual is accounted for by disclosed, expected precision effects.

**Why not BLOCKED:** sufficient real evidence (24 valid, paired,
1024-sample captures) existed to run the comparison.
