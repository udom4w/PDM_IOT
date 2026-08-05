# Release Notes — analyze_fifo_dewesoft.py, Visualization-Only Update (pre-V1-freeze)

**Scope: this release changes visualization only. There are no DSP changes, no mathematical
changes, and no numerical changes.** It exists solely to make `waveform.png` and `fft.png`
look visually closer to Dewesoft's own dark-themed plots before V1 is frozen. Every claim
below was verified by direct execution, not asserted.

---

## What changed

Both plotting functions (`plot_waveform()`, `plot_fft()`) — used identically by `--spectrum
peak` and `--spectrum energy` — now render with a Dewesoft-style dark theme instead of
matplotlib's default white-background style:

1. **Figure background:** black (`#000000`).
2. **Axes background:** very dark gray (`#101010`).
3. **Per-axis trace colors:** X = bright red (`#FF3030`), Y = bright blue (`#3399FF`),
   Z = bright green (`#33CC33`) — matching Dewesoft's own AI1(red)/AI2(blue)/AI3(green)
   convention (see the reference screenshot this tool was originally built to match).
4. **Grid:** thin gray lines (`#404040`, linewidth 0.5, alpha 0.6) instead of the previous
   uncolored `alpha=0.2` default-gray grid.
5. **Axis labels, tick labels, and titles:** white (`ax.tick_params`, `xaxis.label`,
   `yaxis.label`, and each `set_title`/`suptitle` call now explicitly set white).
6. **Dominant-frequency marker (fft.png only):** changed from `tab:red` to a bright gold
   (`#FFD400`) — **necessary**, not cosmetic drift: with the X-axis trace now also red, the
   old `tab:red` marker would have blended into the X subplot's own trace and become
   illegible. Gold is visually distinct from all three new trace colors on every subplot.
7. **Spines:** the pre-existing choice to hide the top/right spines is unchanged; the
   remaining bottom/left spines are recolored white so they remain visible against the new
   dark background (they were previously left at matplotlib's default black, which would
   have been invisible on a black figure).

A small **real bug was caught and fixed during this same change**: `ax.title` in matplotlib
refers only to the *center*-aligned title, a separate Text object from the *left*-aligned
title `plot_waveform()` actually uses (`loc='left'`). Setting `ax.title.set_color(...)`
therefore recolored an unused, empty title object while the real, visible title text stayed
at matplotlib's default (black) color — invisible against the new black background. Fixed by
passing `color='white'` directly to the `ax.set_title(..., loc='left', ...)` call instead,
and the misleading `ax.title.set_color(...)` line was removed rather than left in as
dead/misleading code.

## What did NOT change (verified, not assumed)

- **`summary.json` is byte-for-byte identical before and after this change.** Compared the
  full parsed JSON (all three axes, all fields) from a pre-change baseline against a fresh
  run after the visual changes: `mean_dc`, `ac_rms`, `peak`, `peak_to_peak`, `std_dev`,
  `crest_factor`, `dominant_frequency_hz`, `dominant_amplitude`, `noise_floor`, `snr_db`, and
  `n_fft_blocks_averaged` all match exactly, for every axis, in both Peak and Energy Spectrum
  modes.
- **No DSP function was touched.** `compute_time_domain_stats()`, `compute_fft_spectrum()`,
  `compute_energy_spectrum()`, `compute_overall_rms()`, `compute_band_rms()`,
  `find_dominant_and_noise_floor()`, `split_into_blocks()`, `hann_window()`,
  `coherent_gain()`, `noise_power_gain()` — none of these were edited. Only
  `plot_waveform()`, `plot_fft()`, and a small new `_apply_dark_theme()` helper (styling
  only, receives no computed data, reads nothing, returns nothing) were changed.
- **Axis limits, data ranges, and line values are unchanged** — only color/background/text
  styling was modified; no `set_xlim`/`set_ylim`/data-transformation code was touched or
  added.
- **PNG resolution is unchanged**: `figsize=(11, 8)`, `dpi=150` (both untouched) → 1650×1200
  pixels, confirmed identical before and after by inspecting the saved file dimensions.
- **CLI, `--out-dir` default, argument validation, and console report text** are all
  unaffected — this release touches only the two plotting functions.

## Verification performed

1. Ran `python experimental/analyze_fifo_dewesoft.py experimental/dewesoft_data/3Axis_acc_0009.csv --sr-hz 1000`
   before and after the change, saved `summary.json` from each run, and compared them as
   parsed JSON (not just text diff, to rule out whitespace-only false negatives) — identical.
2. Repeated the same comparison for `--spectrum energy --band-hz 10 200` — the additional
   `overall_rms`/`band_rms`/`band_range_hz` fields are unaffected by the visual change.
3. Visually inspected both `waveform.png` and `fft.png` in both modes: black figure
   background, dark-gray axes background, red/blue/green per-axis traces, white axis labels/
   tick labels/titles, thin gray grid, and (fft.png) a gold dominant-frequency marker legible
   against all three trace colors, all confirmed present and correctly colored after the
   title-color bug fix described above.

## Known limitations (unchanged from before this release)

This release does not address, and was not intended to address, any of the previously
documented items in `code_review_v2.md`, `validation_report.md`, or `release_notes_v2_1.md`
(the block-averaging weight fix, Hann scalloping loss, the unvalidated "~2% typical" Energy
Spectrum residual claim, the missing automated test suite, or any of the still-open minor
duplication/dead-code items explicitly deferred in those documents). Those remain exactly as
previously described.
