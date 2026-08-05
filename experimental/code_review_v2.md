# Code Review — analyze_fifo_dewesoft.py (V2)

**Reviewer stance:** independent DSP/production-code review. Source code was **not modified**
during this review. All numeric claims below were checked with throwaway, read-only scripts
that import the module — never by editing it. File reviewed as of the current V2 state (Peak
Spectrum + Energy Spectrum, `--spectrum {peak,energy}`, `--band-hz`).

---

## Executive Summary

The core FFT math is correct and internally consistent: Peak Spectrum's Coherent-Gain
correction and Energy Spectrum's Noise-Power-Gain correction are both textbook-correct for
their stated purposes, the single-sided fold (double all bins except DC/Nyquist) is applied
correctly for both even and odd `fft_size`, and the DC-removal → window → rFFT pipeline
matches its own documentation. Peak mode was independently re-verified to be numerically
**unchanged** from V1.

However, this review found **one previously-undocumented, quantitatively serious defect**:
non-overlapping block averaging gives a zero-padded, mostly-empty final block **equal
weight** to a full block. In the worst case (signal length = `fft_size + 1`) this silently
cuts the reported amplitude **in half** — a 50% error with no warning to the user, in both
Peak and Energy modes. This is more severe than anything previously flagged in
`validation_report.md` and should be treated as the top-priority finding.

A second real, previously-undiscussed effect is **Hann window scalloping loss**: a
non-bin-aligned sinusoid can read up to **~15.1%** low in Peak Spectrum (confirmed
numerically, matches the textbook Hann worst case of −1.42 dB). This is a fundamental,
unavoidable property of windowed FFT analysis, not a bug — but it is currently undocumented
anywhere in this tool, while the (arguably smaller, ~22.5%) CG/NPG distinction is documented
extensively. That imbalance is worth correcting in the next documentation pass.

Everything else found is minor: a handful of duplicated code fragments, one dead variable,
some unvalidated CLI edge cases that fail ungracefully rather than with a clear error message,
and the absence of any committed automated test suite (every verification to date — including
this one — has been an ad hoc scratch script, not a repeatable test).

**Bottom line:** the mathematics is sound for well-behaved inputs (signal length an exact or
near-exact multiple of `fft_size`, tones roughly bin-aligned or broadband content). It becomes
silently unreliable at the edges (odd-length signals, degenerate CLI arguments) in ways a
production tool should surface loudly instead of computing quietly.

---

## Strengths

1. **Peak Spectrum genuinely unchanged.** Re-diffed `summary.json` for the validation file
   in `--spectrum peak` (default) against the pre-V2 baseline: identical except for the new,
   additive `spectrum_mode` key. The V1→V2 change did not perturb the default path.
2. **Even/odd `fft_size` single-sided folding is handled correctly** in both
   `compute_fft_spectrum()` and `compute_energy_spectrum()`: DC is never doubled, the Nyquist
   bin is un-doubled only when `fft_size` is even (correct — an odd-length rFFT has no
   standalone Nyquist bin, so all bins from index 1 onward legitimately keep their factor of
   2). This is a detail many implementations get wrong; this one gets it right.
3. **CG vs. NPG distinction is mathematically correct and well-reasoned.** Verified
   independently: `CG = mean(hann(2048)) = 0.499756`, `NPG = mean(hann(2048)**2) = 0.374817`,
   and the documented `sqrt(NPG)/CG ≈ 1.225` (~22.5%) figure is an exact, data-independent
   property of the window itself (not an empirical estimate) — confirmed by direct
   computation, not just algebra.
4. **NaN/zero-signal edge cases are handled deliberately, not accidentally.** Crest factor and
   SNR both have explicit `> 0` guards before dividing/taking a log, returning `float('nan')`
   instead of raising or emitting a runtime warning. Constant/flat input signals were tested
   and degrade gracefully to `NaN` fields rather than crashing.
5. **Docstrings are unusually honest about their own limits.** The Energy Spectrum docstring
   explicitly says the Parseval agreement is approximate, not exact, and explains *why*
   (windowing tapers information at the edges) rather than overclaiming — this is good
   practice and made this review easier, since the code doesn't need to be caught
   contradicting an overconfident claim on this particular point.
6. **Peak/Energy separation is architecturally clean.** `compute_fft_spectrum()` (Peak) and
   `compute_energy_spectrum()` (Energy) are two independent functions with no shared mutable
   state; nothing in the Peak path was touched to add the Energy path.

---

## Weaknesses

1. **No automated test suite exists.** Every numeric claim in this review, in
   `validation_report.md`, and in this file's own verification was produced by an ad hoc,
   throwaway script written during a chat session — never committed, never re-runnable by a
   future maintainer without reconstructing it from scratch. For a tool whose entire purpose
   is "mathematically correct, traceable processing," this is a significant gap.
2. **CLI input validation is essentially absent.** `--sr-hz`, `--fft-size`, and `--band-hz`
   accept any float/int the user types, including zero, negative, or nonsensical (reversed)
   ranges, with no upfront check and no friendly error message — see Possible Bugs #3–#5.
3. **The block-averaging scheme silently weights unequal amounts of real data equally** — see
   Possible Bugs #1, the most severe finding in this review.
4. **Undocumented scalloping loss** — see Possible Bugs #2. Not a bug in the sense of an
   implementation error, but a real source of amplitude error the user has no way to know
   about from reading this tool's own documentation.
5. **The Energy Spectrum's "~0.3–1.6%" / "~2%" residual-error figures are generalized from a
   single validation file.** The docstring's phrase "typically within ~2% on real vibration
   data" is not established — only one file has ever been tested. A more tonal/narrowband
   signal, or one with a very different crest factor, could plausibly show a larger NPG
   residual; a broadband/noise-like signal should show a smaller one, closer to the
   assumption's ideal case. This claim should be labeled as file-specific until validated
   against a wider signal set.
6. **The "Dewesoft averages FFT blocks by linear magnitude" assumption (Peak Spectrum) is
   unverified.** Dewesoft's actual internal averaging algorithm (linear vs. RMS vs. peak-hold
   across blocks) is closed-source. This is already flagged as an assumption in the module
   docstring, and I cannot independently confirm or refute it — noting it here only because
   the review task asked to flag anything that cannot be verified.

---

## Possible Bugs

### 1. [HIGH SEVERITY] Zero-padded final block gets equal averaging weight regardless of how much real data it contains

**Where:** `split_into_blocks()` (lines 260–284) feeds unweighted blocks into
`np.mean(spectra, axis=0)` in `compute_fft_spectrum()` (line 316) and
`np.mean(power_spectra, axis=0)` in `compute_energy_spectrum()` (line 369).

**The mechanism:** when a signal's length isn't an exact multiple of `fft_size`, the final
block is zero-padded up to `fft_size` and then averaged into the result with the **same
weight (1/n_blocks)** as every full block — regardless of how few real samples it actually
contains.

**Reproduced numerically** (synthetic 50 Hz tone, amplitude 1.0, `fft_size=2048`,
`sr_hz=1000`, signal length = `fft_size + 1` samples, i.e. one full block plus a
single-real-sample straggler):

```
n_blocks = 2 (one full 2048-sample block + one block with 1 real sample, 2047 zeros)
Reported dominant amplitude with straggler included : 0.450538
Same data, full block only (straggler dropped)       : 0.901077
Dilution from including the straggler at equal weight: -50.00%
```

The straggler block, holding essentially no signal, still counts for half the average —
cutting the reported amplitude exactly in half. This is a worst-case construction (one
leftover sample), but the effect scales continuously: any signal whose length lands just
past a `fft_size` boundary is measurably diluted, and the tool gives **no indication** this
happened beyond the `n_fft_blocks_averaged` count in the summary (which a user would have to
know to interpret this way).

**Verdict:** this is not a hypothetical edge case — it is the default, unannounced behavior
of `split_into_blocks()` for *any* input whose length isn't an exact multiple of `--fft-size`,
which for real-world captures (a fixed-duration recording at a fixed sample rate) is the
common case, not the exception. The magnitude of the dilution is small for large,
close-to-a-multiple signals and severe for signals just barely longer than N blocks — this
tool has no way to warn the user which regime they're in.

**This affects both Peak and Energy Spectrum identically** (both consume the same
`split_into_blocks()` output with the same unweighted averaging pattern).

### 2. [MEDIUM SEVERITY, DOCUMENTATION GAP] Hann window scalloping loss is real, measured, and currently undocumented

**Where:** `compute_fft_spectrum()`'s Coherent-Gain correction (line 303) is only exact for a
sinusoid whose frequency falls exactly on an FFT bin center. Off-bin frequencies suffer
"scalloping loss" — a textbook, well-known property of any window (Harris, 1978, *"On the Use
of Windows for Harmonic Analysis with the DFT"*), worst-case ≈ −1.42 dB for a Hann window at
exactly half a bin's offset.

**Reproduced numerically** (`fft_size=2048`, `sr_hz=1000`, true peak amplitude = 1.0):

```
Bin-aligned tone      f=48.83 Hz   reported peak=1.000000   error=+0.00%
Half-bin-offset tone  f=49.07 Hz   reported peak=0.848965   error=-15.10%
```

This matches the textbook Hann worst case almost exactly. **This means the "Dominant
Amplitude" and "Peak" figures reported by Peak Spectrum can legitimately under-read a real
sinusoidal component's true amplitude by up to ~15%, purely as a function of where that
component's frequency happens to fall relative to the FFT bin grid** — completely
independent of, and larger in the worst case than, the ~22.5% CG-vs-NPG issue this tool
already documents at length. Nothing in the module docstring, `compute_fft_spectrum()`'s
docstring, or `validation_report.md` currently mentions scalloping loss at all.

**This is not a code defect** — every windowed-FFT tool has this property, including (almost
certainly) Dewesoft itself, since it is intrinsic to using any non-rectangular window. It is
listed here because the review was asked to check "every implementation against the
comments," and the comments currently give the impression that the CG correction fully
resolves peak-reading accuracy, which it does not for off-bin content.

### 3. [MEDIUM SEVERITY] `--fft-size 0` and negative values are not validated and fail badly

**Where:** `main()`'s `--fft-size` argument (line 620) has no range/positivity check.

- `--fft-size 0` → `split_into_blocks()` calls `range(0, n, fft_size)` with `fft_size=0`,
  raising `ValueError: range() arg 3 must not be zero` — an unhandled, unfriendly traceback
  with no indication to the user which argument caused it.
- `--fft-size -5` (or any negative value) → `range(0, n, -5)` silently produces **zero
  blocks**. Confirmed: `split_into_blocks(np.zeros(100), -5)` returns `[]`.
  `np.mean([], axis=0)` then emits a `RuntimeWarning: Mean of empty slice` and
  `RuntimeWarning: invalid value encountered in scalar divide`, and **the function returns
  `nan` with no exception at all** — the tool would proceed to print `nan` in every field
  derived from that spectrum, save a blank-looking plot, and write `NaN` into `summary.json`,
  all without ever telling the user the root cause was an invalid `--fft-size`.

### 4. [LOW-MEDIUM SEVERITY] `--sr-hz 0` silently switches units without saying so

**Where:** `compute_fft_resolution_hz()`, `compute_fft_spectrum()`, and
`compute_energy_spectrum()` each contain the pattern `... if sr_hz else ...` (see Code
Quality #1 for the duplication itself). `--sr-hz` is declared `required=True`, but not
validated to be positive — passing `--sr-hz 0` makes `sr_hz` falsy, so all three call sites
silently fall back to their "not given" branch (unitless `d=1.0` frequency axis, `None`
resolution), while `main()`'s header line unconditionally prints
`f"Sample rate: {args.sr_hz:g} Hz"` — i.e. it will print **"Sample rate: 0 Hz"** and still
label the plot axes "Frequency (Hz)", even though the actual frequency values are in
cycles-per-block, not Hz. A user who fat-fingers `--sr-hz 0` gets mislabeled but
plausible-looking output rather than an error.

### 5. [LOW SEVERITY] `--band-hz` accepts a reversed or nonsensical range silently

**Where:** `compute_band_rms()` (line 397) does not validate `lo < hi`. Confirmed: calling it
with `band_hz=(50, 10)` (reversed) returns `0.0` with no warning, rather than either swapping
the bounds or raising an error. A typo in the two `--band-hz` values (easy to do, since
argparse's `nargs=2` doesn't enforce ordering) silently produces a Band RMS of zero, which
could be mistaken for "no energy in this band" rather than "the arguments were backwards."

---

## Mathematical Verification

All formulas below were checked against the standard discrete Parseval identity for a real
signal's one-sided rFFT (even `M`):

```
sum_n x[n]^2  =  (1/M) * [ |X[0]|^2 + 2*sum_{k=1}^{M/2-1} |X[k]|^2 + |X[M/2]|^2 ]
```

| Formula in code | Location | Verified against | Result |
|---|---|---|---|
| `dc = signal.mean()`, `ac = signal - dc` | `compute_time_domain_stats` | Definition of DC/AC decomposition | Correct |
| `ac_rms = sqrt(mean(ac**2))` | same | Standard AC RMS | Correct |
| `peak = max(abs(ac))` | same | 0-to-peak amplitude | Correct |
| `peak_to_peak = max(ac) - min(ac)` | same | Since DC cancels (`ac = signal - dc`), this equals `max(signal) - min(signal)` exactly | Correct, and DC-independent as expected |
| `std_dev = np.std(ac)` | same | `np.std` uses `ddof=0` (population std); since `mean(ac) ≈ 0` by construction, this is **algebraically identical** to `ac_rms` | Confirmed to floating-point precision (`ac_rms == std_dev` to 1e-14 relative) on a 5000-sample synthetic test signal — **not an independent measurement, a mathematical tautology given the current code** (see Code Quality) |
| `crest_factor = peak/ac_rms` guarded by `ac_rms > 0` | same | Standard definition; guard prevents `0/0` | Correct |
| `CG = mean(window)` | `coherent_gain` | Standard Coherent Gain definition | Correct; computed value `0.499756` for `np.hanning(2048)` confirmed by direct execution |
| `NPG = mean(window**2)` | `noise_power_gain` | Standard Noise Power Gain definition | Correct; computed value `0.374817` confirmed by direct execution |
| `mag = \|rfft(w*x)\|/M/CG`, doubled except DC/Nyquist | `compute_fft_spectrum` | Amplitude-spectrum scaling with coherent-gain correction | Correct for bin-aligned tones (0.00% error, Check 1); subject to scalloping loss off-bin (Bug #2) — this is expected DSP behavior, not a scaling error |
| `ms = \|rfft(w*x)\|^2/(NPG·M^2)`, doubled except DC/Nyquist | `compute_energy_spectrum` | Windowed one-sided Parseval, mean-square domain | **Correct after the M² fix applied earlier in V2 development** — re-verified in this review: `Overall RMS` vs. time-domain RMS agreed to within 0.28–1.64% across X/Y/Z on the validation file (see `validation_report.md` §7.2, independently re-derived here, same order of magnitude). Confirmed the underlying `M` (not `M²`) scaling would be wrong by exactly `sqrt(fft_size)` — this is the bug that was caught and fixed during V2 development, not a residual issue |
| Fold: `×2` then un-double DC (`[0]`) and Nyquist (`[-1]`, only if `fft_size` even) | both spectrum functions | One-sided DFT fold identity | Correct for both parities of `fft_size` — explicitly checked: odd `fft_size` does not incorrectly un-double its last bin (no separate Nyquist bin exists at odd length) |
| `dF = sr_hz / fft_size` | `compute_fft_resolution_hz` | Standard FFT bin spacing | Correct |
| `sum(ms_avg)` → `Overall RMS`, masked-`sum` → `Band RMS` | `compute_overall_rms`, `compute_band_rms` | Parseval summation, band-restricted | Correct given `ms_avg`'s correctness (inherits Energy Spectrum's ~1-2% NPG approximation, not exact) |
| `snr_db = 20*log10(dominant_amp/noise_floor)` | `find_dominant_and_noise_floor` | Standard amplitude-ratio dB definition (20·log10, correct for amplitude not power ratios) | Correct — note both `dominant_amp` and `noise_floor` are *amplitude* quantities (not power), so `20·log10` (not `10·log10`) is the right constant here in both Peak and Energy modes |

**Item requiring explicit non-verification, per review instructions:** the claim (module
docstring) that Dewesoft's own FFT preview "averages/recomputes FFT blocks" the same way this
tool does (linear magnitude averaging, non-overlapping blocks) **cannot be verified** —
Dewesoft's internal implementation is closed-source and no reference output from the actual
Dewesoft software was available for this review to compare against bin-for-bin. This was
already flagged as an assumption in the source; repeating it here because the review task
explicitly asked to identify what cannot be verified.

---

## DSP Verification

- **Window choice (`np.hanning`, symmetric/non-periodic):** appropriate for single,
  non-overlapped block analysis, which is what this tool does. Correct choice for this design
  (a periodic/DFT-even Hann is the standard choice specifically for overlapped
  Welch/STFT-style analysis, which this tool does not perform).
- **Coherent Gain correction:** correct for its stated purpose (reading a bin-aligned
  sinusoid's true peak amplitude). Confirmed exact (0.00% error) for a bin-aligned tone, and
  confirmed to degrade in the textbook-predicted way (worst case ≈ −15.1%) for a half-bin
  offset — this is **scalloping loss**, a fundamental limitation of *any* windowed spectrum,
  not specific to this implementation, and is currently absent from this tool's own
  documentation of Peak Spectrum's limitations (Bug #2).
- **Noise Power Gain correction:** correct for its stated purpose (recovering broadband
  energy/RMS from a windowed spectrum). The residual ~0.3–1.6% (blocked) / ~0.28–1.64%
  (single-block) disagreement with time-domain RMS is the expected, textbook behavior of NPG
  correction for **non-stationary/deterministic** signals — NPG is only an *exact-in-
  expectation* correction for signals whose energy is uncorrelated with the window's shape
  (the standard Welch/periodogram assumption), never a bit-exact identity for one specific
  real signal. The tool's own docstring says this; this review independently confirms the
  claim is accurate and appropriately hedged.
- **Single-sided fold (`×2`, DC/Nyquist excluded):** correct for both even and odd `fft_size`,
  independently re-derived and confirmed by direct execution above.
- **Zero-padding vs. block-splitting policy:** mathematically well-defined *individually* (a
  zero-padded rFFT is a legitimate, standard operation — it interpolates the spectrum, it does
  not distort frequency content), but the way the resulting blocks are **combined** (equal
  arithmetic weight regardless of real-sample count) is the source of Bug #1, which is a
  genuine defect in the averaging policy, not in the zero-padding or FFT math themselves.
- **Amplitude vs. power domain SNR (`20·log10` vs. `10·log10`):** correctly uses `20·log10`
  throughout, appropriate since both quantities being compared (`dominant_amp`, `noise_floor`)
  are amplitude spectra in both Peak and Energy modes (Energy mode's spectrum stores RMS
  *amplitude* per bin, `sqrt(ms_avg)`, not power — so the same `20·log10` convention is
  correctly reused for both modes without modification).

---

## Code Quality

1. **Duplicated conditional:** the exact line
   `freqs = np.fft.rfftfreq(fft_size, d=1.0 / sr_hz) if sr_hz else np.fft.rfftfreq(fft_size, d=1.0)`
   appears verbatim in both `compute_fft_spectrum()` (line 317) and
   `compute_energy_spectrum()` (line 371); the equivalent `if sr_hz else None` pattern also
   appears a third time in `compute_fft_resolution_hz()` (line 412). Since `--sr-hz` is a
   `required=True` CLI argument, `sr_hz` will always be truthy in practice unless a user
   passes literally `0` (Bug #4) — meaning this is defensive code inherited from a context
   where the sample rate might have been optional, now duplicated three times for a case that
   (mostly) can't happen through normal use. A single shared helper (e.g.
   `resolve_freqs(fft_size, sr_hz)`) would remove the duplication without changing behavior.
2. **Dead variable:** `plot_waveform()` line 498, `n = len(stats['ac']) + 0  # ac already
   excludes nothing in length` — `n` is never referenced again in the function. The `+ 0` and
   the comment both suggest a partially-completed edit; as written this line does nothing.
3. **Redundant tuple→list conversion:** `build_summary_dict()` line 600,
   `list(stats['band_range_hz'])` — confirmed by direct test that `json.dumps` already
   serializes a Python tuple identically to a list (`json.dumps((10.0, 200.0))` →
   `"[10.0, 200.0]"`). The explicit `list()` call has no functional effect; it can be removed
   without changing the JSON output (not urgent, but a needless conversion).
4. **Print alignment breaks in Energy mode:** `print_axis_report()`'s energy-mode branch
   prints `"Dominant Amplitude (RMS) : ..."` instead of the peak-mode `"Dominant Amplitude :
   ..."` — the longer label shifts that one line's colon out of alignment with every other
   field in the same report block. Cosmetic only, but noticeable in the console output.
5. **Parallel field lists in `print_axis_report()` and `build_summary_dict()`:** both
   functions independently enumerate almost the same ~10 field names (`dc`, `ac_rms`, `peak`,
   …). For a script this size, explicit is arguably fine/more readable than a shared
   abstraction, but it is a duplication point to be aware of if a new field is ever added to
   one and forgotten in the other (already a minor risk: `overall_rms` and `band_rms` needed
   to be added to *both* places by hand in V2, and it would be easy for a future edit to
   update one and not the other).
6. **`analyze_axis()`'s peak/energy branches repeat their own boilerplate** (unpack
   freqs/spectrum, call `find_dominant_and_noise_floor`, assemble a dict of the same six core
   keys) with only the extra energy-only fields differing. Could be factored into "compute
   common fields once, then branch only for the extra energy fields" without changing any
   numeric output — flagged under Future Improvements rather than Weaknesses since the current
   form is still fully correct and readable, just slightly repetitive.

---

## Maintainability

- **No committed test suite.** This is the single largest maintainability gap. Every
  correctness claim in this file, in `validation_report.md`, and in the original
  bug-catch-and-fix during V2 development was established via one-off scripts that were
  never saved as part of the repository. A future change to `compute_energy_spectrum()`
  could silently reintroduce the exact M-vs-M² bug that was already found and fixed once,
  and nothing would catch it automatically.
- **Documentation-to-code traceability is otherwise strong.** The module docstring's V2
  section, and each function's own docstring, were checked line-by-line against the actual
  implementation in this review, and (aside from the scalloping-loss omission, Bug #2) they
  accurately describe what the code does, including honestly flagging their own
  approximations. This is well above average for a script of this size and is a genuine asset
  for a future maintainer trying to understand *why* a given line exists.
- **The single source of truth for CG/NPG numeric constants is the docstring text itself**
  (hardcoded values like `0.499756`/`0.374817`), not a code comment referencing a computed
  constant — if `DEFAULT_FFT_SIZE` or the window function ever changes, these hardcoded
  docstring numbers will silently go stale (this review confirmed they are currently accurate
  for `fft_size=2048`, but nothing enforces they stay that way).
- **Error messages are asymmetric.** `find_acceleration_columns()` raises a clear,
  informative `ValueError` when fewer than 3 acceleration columns are found (good) — but no
  equivalent validation exists for `--fft-size`, `--sr-hz`, or `--band-hz` (Bugs #3–#5),
  meaning the codebase already demonstrates it knows how to fail loudly and clearly when it
  wants to; it simply hasn't been applied consistently across all user-facing inputs.

---

## Performance

Not a bottleneck at the current scale (thousands of samples, single-digit-to-low-tens of
FFT blocks per run), but worth noting for a future, larger-scale use case:

1. **`hann_window(fft_size)` is recomputed from scratch on every call** to
   `compute_fft_spectrum()`/`compute_energy_spectrum()` — i.e. once per axis (X, Y, Z) per
   run, 3 redundant recomputations of an identical array. Trivial cost at `fft_size=2048`
   (microseconds), but an easy no-behavior-change memoization opportunity if this is ever
   called in a tighter loop (e.g. batch-processing many files).
2. **Block loop builds a Python list of full-size numpy arrays, then stacks via
   `np.mean(spectra, axis=0)`.** For the block counts this tool sees (tens, not thousands) this
   is a non-issue; it would become a real allocation/CPU cost only for signals with orders of
   magnitude more blocks (e.g. very long continuous captures at a small `--fft-size`).
3. **No vectorization across blocks** — each block's rFFT is computed in a separate Python-level
   loop iteration rather than as one batched call (e.g. reshaping into a `(n_blocks, fft_size)`
   array and calling `np.fft.rfft(..., axis=1)` once). Would reduce Python-loop overhead for
   large block counts; not necessary at current scale.
4. **Multiple full-array intermediate allocations per block** in both spectrum functions
   (`mag = ... ; mag = mag / gain; mag = mag * 2.0; mag[0] /= 2`) — four separate array
   read/write passes where one or two would suffice. Again, immaterial at `fft_size=2048`,
   worth revisiting only if profiling ever shows this loop as a hot path.

None of the above affect correctness; all are pure efficiency observations for a possible
future higher-throughput use case (e.g. batch-analyzing many captures), not issues for the
tool's current, single-file, interactive usage pattern.

---

## Future Improvements

*(Documented for planning purposes only — no code changes were made as part of this review.)*

1. **Fix the block-averaging weight bug (Possible Bugs #1)** — either weight each block's
   contribution by its fraction of real (non-padded) samples, or drop/handle a
   near-empty final block specially, or document the effect prominently enough that a user
   can recognize when they're exposed to it (e.g. surface a warning when the final block's
   real-sample fraction falls below some threshold).
2. **Document Hann scalloping loss** (Possible Bugs #2) alongside the existing CG/NPG
   documentation — currently the tool over-documents one ~22.5% effect and is silent on a
   potentially larger (~15%) one.
3. **Add upfront CLI validation** for `--sr-hz > 0`, `--fft-size > 0`, and `--band-hz` with
   `lo < hi`, each with a clear `argparse`-level or early `sys.exit` error message instead of
   the current mix of cryptic tracebacks (`--fft-size 0`), silent NaN propagation
   (`--fft-size -5`), or silently-wrong-but-plausible output (`--sr-hz 0`, reversed
   `--band-hz`).
4. **Add a committed, repeatable test suite** — at minimum: known-frequency/known-amplitude
   synthetic sine waves asserting (a) Peak Spectrum reads the true peak within a documented
   tolerance for a bin-aligned tone, (b) Energy Spectrum's Overall RMS matches time-domain RMS
   within a documented tolerance, (c) the M-vs-M² regression this review's Mathematical
   Verification section re-confirmed is caught automatically if ever reintroduced, and (d) the
   block-averaging edge case in Possible Bugs #1 is at least characterized/pinned by a test
   rather than left to be rediscovered.
5. **Validate the "~2% typical" Energy Spectrum claim against more than one file** before
   generalizing it further in documentation — currently based on a single validation dataset
   (Weaknesses #5).
6. **Consider overlapping blocks (e.g. 50% overlap, standard Welch practice)** as a v3 design
   direction — would reduce both the variance of the block-averaged estimate and the relative
   impact of a short final block, at the cost of added complexity and a departure from the
   current "matches Dewesoft's live FFT preview" non-overlapping design rationale. A genuine
   trade-off, not a strict improvement, and would need re-validation against Dewesoft
   reference output before being adopted as the default.
7. **Minor cleanups with zero behavior change** (safe to batch together whenever the file is
   next touched for a real feature): remove the dead `n = len(stats['ac']) + 0` line in
   `plot_waveform()`; remove the redundant `list(...)` conversion in `build_summary_dict()`;
   factor the duplicated `rfftfreq`/`if sr_hz else` pattern into one helper.
