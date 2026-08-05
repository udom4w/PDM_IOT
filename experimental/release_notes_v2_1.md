# Release Notes — analyze_fifo_dewesoft.py V2.1 (Stability Release)

**Scope:** V2.1 fixes only the specific defects confirmed in `code_review_v2.md`. No new
analysis features, no new spectrum mode, no change to the Peak Spectrum formula (Hann window,
Coherent Gain correction, single-sided fold — all byte-for-byte unchanged). Everything below
was verified by direct execution against the module, not asserted.

---

## Fixed Bugs

### 1. Equal-weight averaging of a zero-padded final block (`code_review_v2.md` Possible Bugs #1, HIGH SEVERITY)

**Before:** `split_into_blocks()` zero-padded a signal's trailing partial block up to
`fft_size`, and both `compute_fft_spectrum()` and `compute_energy_spectrum()` then averaged
all blocks with `np.mean()` — giving that padded, mostly-empty block the **same weight** as
every full block.

**Confirmed impact (worst case, reproduced during the original review):** a signal exactly
`fft_size + 1` samples long (one full block + a 1-real-sample straggler) had its reported
amplitude cut **exactly in half**.

**Fix chosen:** `split_into_blocks()` now returns `(blocks, weights)`, where `weights[i]` is
the number of *real* (non-zero-padded) samples block `i` contains. Both spectrum functions
combine blocks with `np.average(spectra, axis=0, weights=weights)` instead of `np.mean()`.

**Re-verified after the fix** (same worst-case construction as above):

| | Dilution from including the straggler block |
|---|---:|
| Before fix | −50.00% |
| After fix | −0.0488% |

The residual −0.0488% is exactly what a 1-out-of-2049-samples weight predicts
(`1/2049 ≈ 0.0488%`) — the fix is exact, not approximate.

**Why this method, over the alternatives:**
- *Dropping a short final block entirely* was rejected — it silently discards real data the
  user supplied.
- *Requiring input length to be an exact multiple of `--fft-size`* was rejected — overly
  restrictive for a Dewesoft export, whose row count is set by recording duration, not by
  `--fft-size`.
- *Sample-count weighting* was chosen because it degrades smoothly (a nearly-full trailing
  block is weighted almost like a full one; a nearly-empty one is weighted almost like
  nothing) and required no change to the per-block FFT math itself.

**This bug affected Energy Spectrum identically** (same `split_into_blocks()` output, same
equal-weight averaging pattern) and is fixed the same way in `compute_energy_spectrum()`.

**Numeric consequence for the validation file** (`3Axis_acc_0009.csv`, `--fft-size 2048`,
which produces 9 full blocks + 1 partial block of 1994/2048 real samples — 97.36% full, i.e.
close to the well-behaved end of this bug's range, not the worst case):

| Axis | Field | Before (V2) | After (V2.1) | Change |
|---|---|---:|---:|---:|
| X | Dominant Amplitude | 0.376300 | 0.376440 | +0.037% |
| X | Noise Floor | 0.007200 | 0.007204 | +0.056% |
| X | SNR (dB) | 34.36 | 34.36 | ~0.00 |
| Y | Dominant Amplitude | 1.672197 | 1.672381 | +0.011% |
| Y | Noise Floor | 0.005680 | 0.005669 | −0.19% |
| Y | SNR (dB) | 49.38 | 49.40 | +0.02 |
| Z | Dominant Amplitude | 0.976023 | 0.976232 | +0.021% |
| Z | Noise Floor | 0.006727 | 0.006722 | −0.07% |
| Z | SNR (dB) | 43.23 | 43.24 | +0.01 |

These are the **only** fields affected on this file — see "Unchanged Behavior" below for
everything that is numerically identical before and after.

### 2. Missing input validation (`code_review_v2.md` Possible Bugs #3, #4, #5)

All five cases named in the review are now rejected up front with a specific, actionable
message instead of a cryptic traceback, a silent `NaN`, or plausible-looking-but-wrong output:

| Input | Before | After |
|---|---|---|
| `--fft-size 0` | Unhandled `ValueError: range() arg 3 must not be zero` | `error: --fft-size must be a positive integer (got 0)`, exit code 2 |
| `--fft-size -5` | Silently produced `NaN` everywhere, only a `RuntimeWarning` | `error: --fft-size must be a positive integer (got -5)`, exit code 2 |
| `--sr-hz 0` | Silently switched the frequency axis to unitless cycles/block while still printing "Sample rate: 0 Hz" | `error: --sr-hz must be a positive number (got 0.0)`, exit code 2 |
| `--sr-hz` negative | Same silent mislabeling as above | `error: --sr-hz must be a positive number (got -100.0)`, exit code 2 |
| `--band-hz` reversed (e.g. `200 10`) | Silently returned Band RMS = 0.0 | `error: --band-hz LO HI must satisfy 0 <= LO < HI (got LO=200, HI=10)`, exit code 2 |
| `--band-hz` with negative LO | Not previously flagged as invalid | `error: --band-hz LO HI must satisfy 0 <= LO < HI (got LO=-10, HI=100)`, exit code 2 |
| Empty/unreadable CSV file | Raw `pandas.errors.EmptyDataError` traceback | `Error: CSV file '<path>' is empty or unreadable: No columns to parse from file`, exit code 1 |
| Header-only CSV (0 data rows) | Would have propagated 0-length arrays into DC/RMS/FFT computation silently | `Error: axis X has only 0 sample(s) in '<path>'; at least 2 samples per axis are required.`, exit code 1 |
| CSV with exactly 1 data row | Would have propagated a 1-sample array into DC/RMS/FFT computation silently | `Error: axis X has only 1 sample(s) in '<path>'; at least 2 samples per axis are required.`, exit code 1 |

All nine were re-tested directly against the updated tool as part of this release (not just
reasoned about) — see the confirmation output captured during V2.1 development.

### 3. Dead code removed (`code_review_v2.md` Code Quality #2)

`plot_waveform()`'s `n = len(stats['ac']) + 0  # ac already excludes nothing in length` line
has been removed. It computed a value that was never referenced again in the function; no
plot output is affected (confirmed: `waveform.png` generation was re-run and is visually and
numerically identical). No other "Code Quality" items from the review (the duplicated
`rfftfreq`/`if sr_hz else` pattern, the redundant `list()` conversion, the print-alignment
cosmetic difference) were touched — those were explicitly out of scope for this release; see
"Known Limitations" below.

---

## Unchanged Behavior

Confirmed by direct comparison, not assumed:

- **All time-domain statistics are byte-for-byte identical before and after V2.1**, for every
  axis of the validation file: `Mean (DC)`, `AC RMS`, `Peak`, `Peak-to-Peak`, `Std Dev`, and
  `Crest Factor` did not change at all (`compute_time_domain_stats()` was not touched by
  either fix — it doesn't call `split_into_blocks()` and has no CLI-argument dependency).
- **`Dominant Frequency` did not change** for any axis on the validation file — the bug only
  perturbs the reported *amplitude* at a bin, not which bin is dominant.
- **Peak Spectrum's per-block formula is untouched**: `hann_window()`, `coherent_gain()`, the
  `mag = |X|/fft_size/CG` scaling, and the single-sided fold (`×2`, DC/Nyquist un-doubled) are
  all the exact same code as V2 — the ONLY change to `compute_fft_spectrum()` is which
  averaging function combines already-identically-computed per-block spectra
  (`np.average(..., weights=...)` instead of `np.mean(...)`).
- **A signal that fits in a single block (`len(signal) <= fft_size`), or whose length is an
  exact multiple of `fft_size` (all blocks full), is mathematically unaffected by the
  averaging fix** — verified directly: with all block weights equal (or only one block to
  begin with), a weighted mean and a plain mean are identical by definition. Re-confirmed by
  execution: a 4×`fft_size`-length synthetic signal produced `weights=[2048, 2048, 2048, 2048]`
  and unchanged output.
- **Energy Spectrum's Overall RMS / Band RMS remain Parseval-consistent to the same ~1-2%
  order of magnitude documented in `validation_report.md` §7** — the averaging fix changes
  which numbers are being averaged into the final result, not the underlying NPG-correction
  math that determines the size of that residual.
- **CLI interface is backward compatible** for all previously-valid inputs — every command
  that worked correctly in V2 (positive `--fft-size`/`--sr-hz`, well-formed `--band-hz`,
  readable CSVs with ≥2 samples per axis) behaves identically in V2.1; the new checks only
  reject inputs that were already nonsensical.

---

## Known Limitations

Carried forward from `code_review_v2.md`, still present, **not addressed by this release**
(explicitly out of scope per the V2.1 stability mandate):

1. **Hann window scalloping loss (documented, not fixed).** Peak Spectrum's Coherent-Gain
   correction reads a sinusoid's true peak exactly only when its frequency falls exactly on
   an FFT bin center; off-bin content can read up to **~15.1%** low (reproduced again during
   this release, matching the textbook Hann worst case of −1.42 dB). This is a fundamental
   property of any windowed FFT, not a defect, and fixing it would require changing Peak
   Spectrum's numerical output (a different window, finer zero-padding, or peak
   interpolation) — which this release's own mandate ("preserve all validated numerical
   behavior of Peak Spectrum") rules out. Now documented in the module docstring
   ("V2.1 STABILITY RELEASE" section) and `validation_report.md` §8, per task requirement.
2. **The "~2% typical" Energy Spectrum residual is still based on one validation file.** Not
   re-examined in this release; still an open item from the original review.
3. **The Dewesoft "averages FFT blocks by linear magnitude" assumption remains unverified**
   against Dewesoft's own (closed-source) implementation — unchanged from V2.
4. **No committed automated test suite exists.** Every check in this release (the −50% → 
   −0.0488% dilution fix, the single-block/exact-multiple invariance, all nine validation
   error paths, the before/after validation-file comparison) was run via throwaway scripts
   during development, the same gap `code_review_v2.md`'s Maintainability section already
   flagged. Still open.
5. **Explicitly not touched in this release** (still present, still harmless, listed so a
   future contributor doesn't assume otherwise): the duplicated `if sr_hz else ...` /
   `rfftfreq` pattern across `compute_fft_resolution_hz()`, `compute_fft_spectrum()`, and
   `compute_energy_spectrum()`; the redundant `list(band_range_hz)` conversion in
   `build_summary_dict()` (Python's `json` module already serializes tuples as arrays); and
   the print-alignment difference between Peak and Energy mode's "Dominant Amplitude" console
   line. None of these were on the confirmed-bug list this release targets, and none affect
   correctness.
6. **`compute_band_rms()`'s bin-selection is still a brick-wall discrete mask**
   (`freqs >= lo & freqs <= hi`), not a fractional/interpolated band edge — unchanged from V2,
   still only relevant for very narrow `--band-hz` ranges relative to the FFT resolution.
