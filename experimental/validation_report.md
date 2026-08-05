# Validation Report — analyze_fifo_dewesoft.py

**Input:** `experimental/dewesoft_data/3Axis_acc_0009.csv` (20,426 samples/axis)
**Sample rate:** 1000 Hz (`--sr-hz 1000`)
**FFT size:** 2048 (default) → 10 non-overlapping blocks, last block zero-padded, spectra averaged
**Command:** `python experimental/analyze_fifo_dewesoft.py experimental/dewesoft_data/3Axis_acc_0009.csv --sr-hz 1000`
**Tool version validated:** as committed, pre-V2 (no code changes made during this pass)

---

## 1. Processing Pipeline

```
CSV (Time, AI1/AI2/AI3 in m/s^2)
  │
  ├─► find_acceleration_columns()   -- pick 3 "m/s2" columns, skip "/v (m/s)" velocity columns
  │
  ▼
raw signal (per axis, full length N = 20426)
  │
  ├─► compute_time_domain_stats()   -- Mean/AC RMS/Peak/P2P/StdDev/CrestFactor  (Section 2)
  │
  ▼
ac = signal - mean(signal)            [DC removal]
  │
  ▼
split_into_blocks(ac, fft_size=2048)  -- 10 blocks; block 10 zero-padded (20426 = 9*2048 + 1994)
  │
  ▼  (per block)
windowed = block * hann(2048)         [Hann window]
  │
  ▼
X = rfft(windowed)                    [real FFT]
  │
  ▼
mag = |X| / fft_size                  [length normalization]
mag = mag / CG                        [coherent-gain correction, CG = mean(hann window)]
mag = mag * 2; mag[0] /= 2; mag[-1] /= 2   [single-sided fold, DC & Nyquist not doubled]
  │
  ▼
average mag across all 10 blocks      [block-averaged Peak(Auto) spectrum]
  │
  ▼
find_dominant_and_noise_floor()       -- argmax (bin 1..end), median of non-DC bins, SNR dB
```

This matches the pipeline order requested: **CSV → DC removal → Hann window → window
correction → zero padding → rFFT → single-sided spectrum → peak scaling → dominant
frequency detection.** (Zero-padding is applied per-block, before windowing/rFFT, only
to the input's final short block or to the whole signal if `N ≤ fft_size` — see
`split_into_blocks()`.)

---

## 2. Mathematical Formulas (exact equation per reported value)

Let `x[n]` be the raw signal (length N), `x̄ = mean(x)`, `ac[n] = x[n] − x̄`.

| Value | Formula | Code location |
|---|---|---|
| Mean (DC) | `x̄ = (1/N) Σ x[n]` | `compute_time_domain_stats`: `signal.mean()` |
| AC RMS | `RMS = sqrt( (1/N) Σ ac[n]² )` | `np.sqrt(np.mean(ac ** 2))` |
| Peak | `Peak = max( |ac[n]| )` | `np.max(np.abs(ac))` |
| Peak-to-Peak | `P2P = max(ac[n]) − min(ac[n])` | `np.max(ac) - np.min(ac)` |
| Std Dev | `σ = sqrt( (1/N) Σ (ac[n] − mean(ac))² )` — note `mean(ac) = 0` by construction, so this reduces to the **same formula as AC RMS** | `np.std(ac)` |
| Crest Factor | `CF = Peak / RMS` | `peak / ac_rms` |
| Hann window | `w[n] = 0.5 − 0.5·cos(2πn/(M−1))`, `n = 0..M−1`, `M = fft_size` (numpy's *symmetric*, non-periodic definition) | `np.hanning(fft_size)` |
| Coherent gain | `CG = (1/M) Σ w[n]` | `np.mean(window)` |
| Windowed rFFT | `X[k] = Σ_{n=0}^{M−1} (ac_block[n]·w[n]) · e^{−j2πkn/M}` | `np.fft.rfft(windowed)` |
| Peak-amplitude spectrum (per block) | `A[k] = ( |X[k]| / M ) / CG`, then `A[k] *= 2` for `0 < k < M/2`, **not** doubled at `k=0` or `k=M/2` (Nyquist) | `compute_fft_spectrum` |
| Block-averaged spectrum | `Ā[k] = (1/B) Σ_{b=1}^{B} A_b[k]`, B = number of blocks | `np.mean(spectra, axis=0)` |
| FFT resolution | `dF = SR / fft_size` | `compute_fft_resolution_hz` |
| Dominant frequency/amplitude | `k* = argmax( Ā[k] )` for `k ≥ 1`; `f_dom = k*·dF`; `A_dom = Ā[k*]` | `find_dominant_and_noise_floor` |
| Noise floor | `NF = median( Ā[1], Ā[2], …, Ā[M/2] )` (DC excluded) | same |
| SNR (dB) | `SNR = 20·log10( A_dom / NF )` | same |

---

## 3. Numerical Results

| Metric | X | Y | Z |
|---|---:|---:|---:|
| Mean (DC), m/s² | 0.000334 | −0.002082 | −0.001538 |
| AC RMS, m/s² | 0.648540 | 1.431491 | 0.870738 |
| Peak, m/s² | 2.366071 | 3.765580 | 2.872768 |
| Peak-to-Peak, m/s² | 4.702429 | 7.298364 | 5.615249 |
| Std Dev, m/s² | 0.648540 | 1.431491 | 0.870738 |
| Crest Factor | 3.6483 | 2.6305 | 3.2992 |
| Dominant Frequency, Hz | 356.9336 | 49.8047 | 49.8047 |
| Dominant Amplitude, m/s² | 0.376300 | 1.672197 | 0.976023 |
| Noise Floor, m/s² | 0.007200 | 0.005680 | 0.006727 |
| SNR, dB | 34.36 | 49.38 | 43.23 |
| FFT blocks averaged | 10 | 10 | 10 |

`fft_resolution = 1000 / 2048 = 0.48828125 Hz` (all axes, reported once).

Cross-checked against `summary.json` written by the same run — all values match exactly
(this table is not independently re-derived, it is the tool's own reported output,
transcribed here per the requested format).

---

## 4. Validation Checks

### 4.1 Std Dev vs AC RMS identity
Confirmed numerically identical for all three axes (see table). This is expected, **not
a bug**: `std_dev = std(ac)` and `ac_rms = sqrt(mean(ac²))` are mathematically the same
quantity whenever `mean(ac) = 0`, which is guaranteed here since `ac` is constructed by
subtracting its own mean. Reporting both is redundant but not incorrect.

### 4.2 Parseval's theorem — time-domain RMS vs frequency-domain energy

Two independent checks were run (script: temporary, not committed — see §6):

**(A) Rectangular (unwindowed) Parseval, full signal, no blocking/zero-padding.**
Using the exact one-sided real-DFT identity for even N:

```
Σ ac[n]²  =  (1/N) · [ |X[0]|² + 2·Σ_{k=1}^{N/2−1} |X[k]|² + |X[N/2]|² ]
```

with `X = rfft(ac)` computed once over the *entire* signal (no window, no
padding — a direct sanity check of the FFT/energy bookkeeping itself, independent
of any of the tool's display-scaling choices).

| Axis | Time-domain RMS | Parseval-reconstructed RMS | % error |
|---|---:|---:|---:|
| X | 0.648540 | 0.648540 | 0.00e+00 % |
| Y | 1.431491 | 1.431491 | 1.55e-14 % |
| Z | 0.870738 | 0.870738 | 1.28e-14 % |

**Result: agrees to floating-point precision (~10⁻¹⁴ %).** This confirms the
underlying FFT/energy relationship (and `np.fft.rfft` itself) is implemented and
used correctly wherever no window or amplitude-display scaling is involved.

**(B) RMS reconstructed from the tool's *actual displayed* spectrum** (Hann-windowed,
coherent-gain peak-corrected, 10-block-averaged — i.e. exactly what `fft.png` and
`summary.json` show), converting each AC bin's peak amplitude to RMS via `/√2` and
summing:

| Axis | Time-domain RMS | Reconstructed RMS from displayed spectrum | % error |
|---|---:|---:|---:|
| X | 0.648540 | 0.766218 | **18.15 %** |
| Y | 1.431491 | 1.755575 | **22.64 %** |
| Z | 0.870738 | 1.059776 | **21.71 %** |

**Result: does NOT agree — off by ~18–23%.** This is *expected*, not a defect in the
FFT math, and has an exact theoretical explanation:

- Coherent Gain (used by the tool, for correct **sinusoidal peak** readout):
  `CG = mean(w) = 0.499756`
- Noise Power Gain (needed for correct **energy/RMS** readout):
  `NPG = mean(w²) = 0.374817`
- Predicted overstatement when using a peak/CG-corrected spectrum to reconstruct RMS:
  `√NPG / CG = 1.22504` → **+22.50%**

This matches the Y (22.64%) and Z (21.71%) measurements almost exactly — both are
single dominant-tone-plus-harmonics signals, closely fitting the "isolated sinusoid"
assumption the `/√2` reconstruction and the CG correction both implicitly make. X
deviates more (18.15% vs. the 22.50% prediction) because X's spectrum is broadband/
noise-like rather than a few clean tones (visible in `fft.png` — dense, leakage-heavy
harmonic content across 0–500 Hz), so the "peak amplitude of an isolated tone / √2 =
RMS of that tone" identity underlying reconstruction (B) is a poorer approximation
for that axis specifically.

**Conclusion:** the tool's amplitude corrections are internally consistent and
mathematically correct **for their stated purpose** (Dewesoft "Peak (Auto)" — reading
the true 0-to-peak amplitude of a sinusoidal component off a Hann-windowed spectrum).
They are, by design, **not** energy-preserving, so the displayed spectrum should never
be summed/Parsevaled against time-domain RMS directly — only the raw unwindowed FFT (as
in check A) is valid for that. This distinction is not currently documented in the tool
itself.

---

## 5. Assumptions (carried from the module docstring, re-verified here)

- "Peak (Auto)" is interpreted as 0-to-peak amplitude of an isolated sinusoidal
  component — confirmed consistent with the CG-based correction actually implemented.
- Noise floor = median of all non-DC bins — a broadband estimate; not verified against
  Dewesoft's own (undocumented/closed-source) noise-floor algorithm, only against the
  formula as coded.
- `np.hanning` (symmetric/non-periodic window) is used rather than a periodic Hann
  (`M+1`-point Hann truncated to M samples). For M=2048 the numerical difference
  between the two is negligible (CG differs from the ideal periodic-Hann value of
  0.5 by <0.05%), but this is worth naming explicitly since some FFT tools (Dewesoft
  included, per its own documentation) default to a periodic Hann for block-based FFT
  analysis specifically to keep window edges from double-weighting when blocks are
  conceptually tiled back-to-back.
- Dominant frequency/amplitude and noise floor are computed from the **same
  Hann-windowed, CG-corrected, block-averaged** spectrum used for the peak display —
  appropriate for tonal peak-picking, but the noise floor and SNR inherit the same
  "not energy-consistent" caveat as §4.2(B) if ever compared against a time-domain
  broadband RMS figure.

## 6. Known Limitations / Items to Consider Before V2

1. **No RMS/energy-consistent spectrum mode.** The tool only ever computes the
   peak/CG-corrected spectrum. If a future feature (e.g. ISO 20816 velocity RMS,
   per-band energy) needs a windowed spectrum whose bins sum back to time-domain RMS
   via Parseval, that requires a *separate* correction (`1/√NPG`, not `1/CG`) — do not
   reuse `compute_fft_spectrum()`'s output for that purpose as-is.
2. **Std Dev and AC RMS are always numerically identical** given the current
   `compute_time_domain_stats()` implementation (§4.1) — not wrong, but reporting both
   as if they were independent measurements could be misread as redundant validation
   when it is actually a mathematical tautology given DC has already been removed.
3. **Symmetric vs periodic Hann window** (§5) — should be confirmed against
   Dewesoft's actual window definition before claiming byte-for-byte parity; current
   implementation is a reasonable and common choice but unverified against Dewesoft's
   internals (which are closed-source).
4. **Noise-floor formula is a broadband median estimate**, not validated against
   Dewesoft's own algorithm (module docstring already flags this; repeated here as a
   pre-V2 item since SNR is derived directly from it).
5. **This validation pass made no code changes.** All figures above are the tool's
   own unmodified output; only an external, temporary, non-committed script was used
   to compute the independent Parseval cross-check in §4.2. No fixes have been applied
   yet — this report is diagnostic input for a V2 decision, per instruction.

---

## 7. V2 Addendum — Energy Spectrum (`--spectrum energy`)

**Added after this report was first written.** Peak Spectrum (`--spectrum peak`, default) is
**unchanged** — verified numerically identical to §3 above (peak-mode `summary.json` diffed
byte-for-byte against the pre-V2 run; only the new informational `spectrum_mode` key was added).

### 7.1 What's different

Both spectra share the identical DC-removal → Hann window → zero-pad/block-split → rFFT →
single-sided-fold pipeline (§1). They diverge only in the final correction/averaging step:

| | Peak Spectrum (`peak`, unchanged) | Energy Spectrum (`energy`, new) |
|---|---|---|
| Purpose | True 0-to-peak amplitude of an **isolated sinusoidal tone** (matches Dewesoft "Peak (Auto)") | RMS amplitude-per-bin spectrum that is **Parseval-consistent** with time-domain energy |
| Correction factor | **Coherent Gain**, `CG = mean(w)` | **Noise Power Gain**, `NPG = mean(w²)` |
| CG / NPG for this tool's 2048-pt `np.hanning` window | `CG ≈ 0.499756` | `NPG ≈ 0.374817` |
| Per-block combination | Averages **magnitude** across blocks: `mean(\|Xb\|)` | Averages **power** across blocks: `mean(\|Xb\|²)` |
| Formula (per block, one-sided, DC/Nyquist not doubled) | `A[k] = (\|X[k]\|/M)/CG · fold_k` | `ms[k] = (\|X[k]\|²/(NPG·M²)) · fold_k`, `rms[k]=√ms[k]` |
| Energy-preserving? | **No, by design** — reconstructing RMS from it overstates true RMS by `√NPG/CG ≈ 1.225` (~22.5%), confirmed in §4.2(B) | **Approximately** — Overall RMS agrees with time-domain RMS to within ~0.3–1.6% on this test file (§7.2) |
| Use when | Reading a specific tone's true peak amplitude; comparing against Dewesoft's own Peak/Peak-Auto display | Computing Overall RMS, Band RMS, or any ISO-20816-style banded energy/severity figure |

**Why NPG, not CG, for energy:** NPG is the standard normalization (Welch/periodogram
convention) that keeps a windowed signal's *average power* consistent with the unwindowed
signal's power. CG instead preserves a single sinusoid's *peak amplitude* — the two
correction factors answer different questions, and using the wrong one for a given purpose
is exactly the source of the ~22.5% Peak-mode RMS overstatement documented in §4.2(B).

### 7.2 Overall RMS vs. time-domain RMS (independent verification)

Checked with a standalone script (not part of the tool) that imports `analyze_fifo_dewesoft`
and cross-checks `compute_overall_rms()` against `np.sqrt(np.mean(ac**2))`:

**Default (`--fft-size 2048`, 10 blocks, last block zero-padded):**

| Axis | Time-domain RMS | Overall RMS (energy mode) | % error |
|---|---:|---:|---:|
| X | 0.648540 | 0.643736 | −0.74% |
| Y | 1.431491 | 1.438031 | +0.46% |
| Z | 0.870738 | 0.874860 | +0.47% |

**Single full block (`--fft-size` = signal length, no zero-padding, isolates the pure
windowing effect from block-averaging dilution):**

| Axis | Time-domain RMS | Overall RMS (energy mode) | % error |
|---|---:|---:|---:|
| X | 0.648540 | 0.654879 | +0.98% |
| Y | 1.431491 | 1.435558 | +0.28% |
| Z | 0.870738 | 0.885034 | +1.64% |

**Sanity control** — rectangular/no-window Parseval (§4.2(A)'s check, repeated here for
contrast): agrees to ~1e-14%, i.e. floating-point precision.

**Interpretation:** Energy Spectrum's Overall RMS is **Parseval-consistent** (within ~2%,
both directions — sign flips between the blocked and single-block runs, confirming this is
data-dependent NPG approximation noise, not a systematic bias like Peak mode's) but **not
bit-exact**. This is fundamental, not a bug: applying *any* window permanently tapers
information at the block edges, so NPG-based energy recovery is only exact in expectation
for a stationary/broadband signal whose energy is uncorrelated with the window shape — never
an exact identity for one specific finite real signal. Bit-exact Parseval equality is only
achievable with a rectangular (unwindowed) transform, which is what §4.2(A) actually computes
and why it lands at 1e-14% instead of ~1%. If a future ISO 20816 implementation needs
audit-grade, machine-precision energy conservation, it should use a rectangular window for
that specific calculation rather than the shared Hann pipeline documented here.

### 7.3 Assumptions / Known Limitations added in V2

- **NPG-based energy recovery is an approximation**, not an identity (§7.2) — document this
  wherever Overall RMS / Band RMS feeds a downstream severity/compliance calculation.
- **`--band-hz` is silently ignored outside `--spectrum energy`** (prints a one-line note) —
  intentional, since Band RMS is only meaningful for the energy-consistent path, not the
  Peak Spectrum.
- **Band RMS uses inclusive bin selection** (`freqs >= lo & freqs <= hi`) against the FFT's
  discrete bin frequencies, not a continuous-frequency integral — for coarse `fft_size`/dF,
  a band edge that falls between bins will include or exclude the nearest bin entirely
  rather than partially; not an issue for the ISO 20816 broad bands (typically tens–hundreds
  of Hz wide relative to this tool's dF), but worth knowing before using very narrow bands.
- **No code change was made to Peak Spectrum's numeric path** — `compute_fft_spectrum()`,
  `coherent_gain()`, `hann_window()`, `split_into_blocks()`, and `find_dominant_and_noise_floor()`
  are all shared, untouched functions; only `compute_energy_spectrum()`, `noise_power_gain()`,
  `compute_overall_rms()`, and `compute_band_rms()` are new.

---

## 8. V2.1 Addendum — Hann Window Scalloping Loss (documentation-only, no code change)

**Confirmed during the independent code review (`code_review_v2.md`, Possible Bugs #2)** and
reproduced again here as part of the V2.1 stability release, which adds this section per that
review's recommendation. This is **not a code defect and V2.1 does not attempt to fix it** —
it is a fundamental, unavoidable property of any windowed FFT, and is documented here (and in
the module docstring) purely so a user of this tool's Peak Spectrum knows it exists.

### 8.1 What it is

`compute_fft_spectrum()`'s Coherent-Gain correction recovers a sinusoid's true 0-to-peak
amplitude **exactly** only when that sinusoid's frequency falls exactly on an FFT bin center
(an exact multiple of `dF = SR/fft_size`). For a frequency that falls between two bins, the
Hann window's main lobe has already started to roll off by the time you reach the nearest bin
center, so the reported peak reads **low** — this is "scalloping loss," documented since
Harris's 1978 paper *"On the Use of Windows for Harmonic Analysis with the DFT"* as a standard,
window-dependent property (a Hann window's worst case, at exactly half a bin's offset, is a
well-known ≈ −1.42 dB).

### 8.2 Numeric confirmation

Synthetic single-tone test, `fft_size=2048`, `sr_hz=1000` (`dF ≈ 0.4883` Hz), true peak
amplitude fixed at 1.0:

| Tone frequency | Bin position | Reported peak | Error |
|---|---|---:|---:|
| 48.83 Hz (= bin 100 exactly) | bin-aligned | 1.000000 | +0.00% |
| 49.07 Hz (= bin 100.5) | half-bin offset | 0.848965 | **−15.10%** |

This matches the textbook Hann worst-case figure (−1.42 dB ≈ −15.1% in linear amplitude)
almost exactly.

### 8.3 Why this matters more than it might first appear

This ~15% worst-case error is **larger than the well-documented ~22.5% CG-vs-NPG distinction
already covered at length in §7**, yet — unlike that distinction — it had never been mentioned
anywhere in this tool's own documentation before this addendum. A user reading a Peak Spectrum
"Dominant Amplitude" figure has no way to know, from the tool's own output, whether that
number is exact (bin-aligned) or reading up to ~15% low (off-bin) for any given capture, since
real-world vibration frequencies have no reason to land exactly on this tool's FFT bin grid.

### 8.4 Why it only affects Peak Spectrum, not Energy Spectrum

Energy Spectrum's Overall RMS / Band RMS are **sums of power across many bins** (§7), so a
single bin's scalloping loss is compensated by the adjacent bins the same lobe spreads energy
into — the total power in the vicinity of a tone is conserved by the window even though any
one bin's reading is not. Peak Spectrum, by contrast, reports a single bin's value directly,
so it has no such averaging-out effect and is fully exposed to this loss.

### 8.5 Why V2.1 does not fix this

Any fix (switching windows, adding zero-padding to interpolate the spectrum more finely, or
adding a parabolic/quadratic peak-interpolation estimator across neighboring bins) would
change Peak Spectrum's numerical output for real-world signals — which V2.1's own stability
mandate explicitly rules out ("preserve all validated numerical behavior of Peak Spectrum").
This is deliberately left as a documented, known limitation rather than silently patched.

---

## Appendix — Validation Run Artifacts

Regenerated by the exact command at the top of this report, saved to
`experimental/dewesoft_data/_validation_out/` (not committed with this report; rerun
the command above to reproduce):
- `waveform.png`
- `fft.png`
- `summary.json`
