#!/usr/bin/env python3
"""
analyze_fifo_dewesoft.py -- common vibration analysis engine, dual input format.

Reproduces Dewesoft's FFT processing (Hann window, single-sided rFFT,
Peak-Auto amplitude scaling) as closely as possible, and analyzes 3-axis
acceleration from EITHER of two input formats through one shared analysis
engine: a Dewesoft CSV export (e.g.
experimental/dewesoft_data/3Axis_acc_0009.csv) or a WTVB05 FIFO capture CSV
(raw ADC counts, e.g. the "Index,X,Y,Z" Phase 7A format also read by
analyze_fifo_capture.py). See "V2 PROJECT PHASE -- MULTI-FORMAT INPUT
ARCHITECTURE" below and experimental/V2_ARCHITECTURE.md for the full
Input Layer / Common Analysis Engine / Output Layer design.

This tool remains intentionally independent from analyze_fifo_capture.py
(which has its own, separate CLI/scope: multi-capture bearing-frequency,
envelope, and spectrogram analysis) -- it does not import that module, and
reproduces only the one FIFO counts->m/s^2 conversion line it needs (see
read_fifo_csv()) rather than depending on it as a library.

Usage:
    python experimental/analyze_fifo_dewesoft.py experimental/dewesoft_data/3Axis_acc_0009.csv --sr-hz 1000
    python experimental/analyze_fifo_dewesoft.py capture.csv --sr-hz 1000 --fft-size 4096
    python experimental/analyze_fifo_dewesoft.py fifo_capture.csv --sr-hz 1000   # WTVB05 FIFO CSV, auto-detected

Output: waveform.png, fft.png, and summary.json are written to --out-dir,
which defaults to experimental/output/ (created automatically if it does
not exist) -- NOT the project root or the current working directory. Pass
--out-dir to write elsewhere.

--------------------------------------------------------------------------
DEWESOFT FFT CONVENTION (FFT preview panel, "Amplitude display: Peak (Auto)")
--------------------------------------------------------------------------
This tool follows Dewesoft's own documented FFT pipeline as closely as a
from-scratch numpy implementation allows:

  1. DC offset is removed from the time-domain signal before windowing.
  2. A Hann (Hanning) window is applied to reduce spectral leakage.
  3. The window's amplitude attenuation (coherent gain = mean of the window
     samples) is corrected for after the FFT, so windowed peak amplitudes
     read the same as they would from an unwindowed signal -- this is what
     Dewesoft's "Peak (Auto)" amplitude display means.
  4. A real FFT (np.fft.rfft) is used and only the non-negative-frequency
     half is kept (single-sided spectrum); all bins except DC and Nyquist
     are doubled to fold the discarded negative-frequency energy back in.
  5. If the input is longer than --fft-size, it is split into consecutive
     (non-overlapping) blocks of --fft-size samples and the resulting
     amplitude spectra are averaged bin-by-bin (matches Dewesoft's "Line
     resolution" FFT preview, which recomputes/averages FFT blocks across
     a long recorder capture rather than computing one huge FFT).
  6. If the input (or its final block) is shorter than --fft-size, it is
     zero-padded to --fft-size before windowing+FFT.

FFT resolution (dF = SR / fft_size) is reported once and reused verbatim
across the console report, fft.png, and summary.json.

ASSUMPTIONS (flag explicitly since Dewesoft's internal implementation is
closed-source):
  - "Peak (Auto)" is taken to mean 0-to-peak amplitude of a single spectral
    line for a sinusoidal component, i.e. the same convention documented in
    analyze_fifo_capture.py's FFT_CONVENTION constant, just with the added
    Hann coherent-gain correction that a rectangular (unwindowed) FFT does
    not need.
  - Noise floor is estimated as the median amplitude of all non-dominant
    bins (excluding DC); this is a reasonable broadband estimate but is not
    guaranteed to match Dewesoft's own (undocumented) noise-floor algorithm
    bin-for-bin.

--------------------------------------------------------------------------
V2: PEAK SPECTRUM vs ENERGY SPECTRUM  (--spectrum peak | --spectrum energy)
--------------------------------------------------------------------------
Both paths share the same DC-removal -> Hann window -> zero-pad/block-split
-> rFFT -> single-sided-fold steps described above; they differ ONLY in how
the windowed FFT magnitude is corrected and averaged afterward. Peak
Spectrum is the ORIGINAL V1 implementation and is UNCHANGED (bit-for-bit
identical numeric output) -- it remains the default.

  Peak Spectrum (--spectrum peak, DEFAULT, UNCHANGED from V1):
    - Purpose: read the true 0-to-peak amplitude of an ISOLATED SINUSOIDAL
      component off a windowed spectrum -- this is what Dewesoft's own
      "Peak (Auto)" display means, and is why this is the default.
    - Correction: divide by Coherent Gain, CG = mean(window).
      For this tool's 2048-point np.hanning window, CG ~= 0.499756.
    - Block combination: averages MAGNITUDE across blocks
      (mean(|X_b|) over b), matching how Dewesoft's live FFT preview
      settles as more blocks accumulate.
    - Does NOT preserve Parseval / signal energy by design -- reconstructing
      RMS from this spectrum (sum bin peaks/sqrt(2)) overstates true RMS
      by ~sqrt(NPG)/CG (~22.5% for a 2048-pt Hann window), confirmed
      empirically in validation_report.md Sec 4.2(B). This is an accepted,
      documented trade-off of the Peak convention, not a defect.
    - Use when: comparing against Dewesoft's Peak/Peak-Auto readout, or
      picking out a specific tonal peak's true amplitude.

  Energy Spectrum (--spectrum energy, NEW in V2):
    - Purpose: an RMS (not peak) amplitude-per-bin spectrum whose bins,
      summed in POWER, are Parseval-CONSISTENT with the time-domain AC RMS
      -- i.e. it is built to be energy-consistent, not peak-reading-accurate
      for a single tone. This is an approximate (typically within ~2% on
      real vibration data, see validation_report.md), not bit-exact,
      recovery of time-domain energy -- see compute_overall_rms() for why
      windowing makes exact recovery impossible in general.
    - Correction: divide by Noise Power Gain, NPG = mean(window**2).
      For the same 2048-point np.hanning window, NPG ~= 0.374817.
      NPG (not CG) is the factor that keeps a windowed signal's average
      POWER consistent with the unwindowed signal's power -- the standard
      normalization behind Welch/periodogram-style PSD estimation.
    - Block combination: averages POWER (mean-square) across blocks, i.e.
      mean(|X_b|^2) over b, NOT mean(|X_b|) -- summing power (not
      magnitude) across bins is what Parseval's theorem actually relates
      to time-domain mean-square, so power-domain averaging is required
      for Overall RMS / Band RMS to stay energy-consistent block-to-block.
    - Supports:
        Overall RMS = sqrt(sum of all bins' mean-square) -- see
          compute_overall_rms(); measured within ~0.3-1.6% of time-domain
          AC RMS on this tool's validation file (Parseval-consistent, not
          bit-exact -- see compute_overall_rms()'s docstring for why a
          Hann-windowed spectrum cannot recover time-domain energy exactly,
          unlike validation_report.md Sec 4.2(A)'s rectangular/no-window
          check, which is exact by construction).
        Band RMS = sqrt(sum of mean-square in [--band-hz LO HI]) -- see
          compute_band_rms(); this is the building block ISO 20816 (and
          any other banded velocity/acceleration RMS limit) needs, since
          those standards define pass/fail bands in Hz, not single tones.
    - Use when: computing Overall/Band RMS, energy-based severity
      thresholds (ISO 20816), or any calculation that needs the spectrum
      to sum back to a physically meaningful energy quantity.

  Numeric summary (2048-pt np.hanning window, this tool's default):
        Coherent Gain      CG  = mean(w)     ~= 0.499756
        Noise Power Gain   NPG = mean(w**2)  ~= 0.374817
        sqrt(NPG) / CG          ~= 1.225  (Peak Spectrum overstates RMS by
                                            ~22.5% if summed as if it were
                                            an Energy Spectrum -- exactly
                                            why the two paths are kept
                                            separate rather than merged
                                            into one "corrected" spectrum.)

This tool NEVER mixes the two: Peak Spectrum output is never fed into
Overall RMS/Band RMS, and Energy Spectrum's "Dominant Amplitude" is an RMS
value, not a peak value -- console/summary label it "(RMS)" explicitly in
that mode to avoid ambiguity with the Peak Spectrum's dominant amplitude.

--------------------------------------------------------------------------
V2.1 STABILITY RELEASE -- fixes confirmed in code_review_v2.md
--------------------------------------------------------------------------
No new analysis features. No change to Peak Spectrum's per-block formula
(Hann window, Coherent Gain correction, single-sided fold -- all
byte-for-byte unchanged). Two fixes were made, both to confirmed defects:

  1. FIXED -- equal-weight averaging of a zero-padded final block
     (code_review_v2.md Possible Bugs #1, HIGH SEVERITY). Previously,
     split_into_blocks()'s trailing partial block (zero-padded up to
     fft_size) was averaged with the SAME weight as every full block --
     confirmed to cut the reported amplitude in HALF in the worst case (a
     signal exactly fft_size+1 samples long). Method chosen: a
     REAL-SAMPLE-COUNT-WEIGHTED mean (np.average(..., weights=weights),
     weights = number of real, non-padded samples per block -- see
     split_into_blocks()) replaces the previous plain np.mean() in both
     compute_fft_spectrum() and compute_energy_spectrum(). A block with
     only 1 real sample out of 2048 now contributes ~0.05% of the average
     instead of 50%. This is numerically IDENTICAL to the pre-fix
     behavior whenever there is only one block, or every block is exactly
     full (signal length an exact multiple of fft_size) -- it only changes
     output for a signal with a genuine partial trailing block, which is
     exactly the confirmed-wrong case. Alternatives considered: dropping a
     short final block entirely (rejected -- silently discards real data
     the user supplied) and requiring exact-multiple input lengths
     (rejected -- overly restrictive for a Dewesoft export whose row count
     is set by recording duration, not by --fft-size). See
     release_notes_v2_1.md for the exact before/after numbers this
     produced on the validation file.

  2. FIXED -- no input validation on --fft-size / --sr-hz / --band-hz /
     CSV size (code_review_v2.md Possible Bugs #3, #4, #5). --fft-size<=0,
     --sr-hz<=0, an invalid --band-hz range (LO<0 or HI<=LO), an
     empty/unreadable CSV, and fewer than 2 samples per axis are now all
     rejected up front with a specific, clear error message instead of
     producing a cryptic traceback, a silent NaN, or plausible-looking but
     wrong (mislabeled-units) output. See main() for the exact checks.

DOCUMENTED (not a code defect -- a fundamental, previously-undiscussed-in-
this-tool's-own-docs property of ANY windowed FFT, confirmed numerically
in code_review_v2.md Possible Bugs #2):

  HANN WINDOW SCALLOPING LOSS -- Peak Spectrum's Coherent-Gain correction
  (coherent_gain(), used in compute_fft_spectrum()) reads a sinusoid's true
  peak amplitude EXACTLY only when that sinusoid's frequency falls exactly
  on an FFT bin center. For a frequency that falls between bins, the Hann
  window's main-lobe droop causes the reported peak to read LOW by up to
  ~1.42 dB (~15.1%) in the worst case (exactly half a bin off-center) --
  this is textbook behavior (Harris, 1978, "On the Use of Windows for
  Harmonic Analysis with the DFT"), confirmed by direct measurement in
  code_review_v2.md:
        bin-aligned tone      (f = exactly bin 100 * dF): reported peak
                              matches true peak to 0.00% error
        half-bin-offset tone  (f = bin 100.5 * dF):        reported peak
                              reads ~15.10% LOW
  This is independent of, and can be LARGER than, the ~22.5% CG-vs-NPG
  distinction documented at length above -- it applies ONLY to Peak
  Spectrum (Energy Spectrum's Overall/Band RMS sum power across many bins,
  so a single bin's scalloping loss washes out in that sum and does not
  bias the Parseval-consistency result the same way). No fix is possible
  without changing Peak Spectrum's numerical behavior (a different window,
  zero-padding to interpolate the spectrum more finely, or a parabolic
  peak-interpolation estimator would all change output values) -- V2.1
  intentionally leaves Peak Spectrum's math untouched and instead documents
  this limitation here and in validation_report.md.

NOT changed in V2.1 (explicitly out of scope for this stability release,
listed here so a future contributor doesn't assume otherwise): the
duplicated `if sr_hz else ...` / rfftfreq pattern (compute_fft_resolution_hz,
compute_fft_spectrum, compute_energy_spectrum), the redundant
list(band_range_hz) conversion in build_summary_dict(), and the
print-alignment cosmetic difference between Peak and Energy mode's
"Dominant Amplitude" line -- all still present, all still harmless, none
of them were on the confirmed-bug list this release targets.

--------------------------------------------------------------------------
V2.2 -- R1 ZERO-PADDING ENERGY NORMALIZATION FIX (Energy Spectrum only)
--------------------------------------------------------------------------
FIXED -- Energy Spectrum under-reported energy by sqrt(n_real/fft_size) for
every zero-padded block. This affected ONLY compute_energy_spectrum() (and
therefore Overall RMS / Band RMS). Peak Spectrum, time-domain statistics,
the FFT itself, windowing, coherent-gain correction, NPG correction,
single-sided folding, and the V2.1 block-weighting logic are ALL UNCHANGED.

  Symptom: the production FIFO configuration (captureId2.csv, N=1024
  samples, SR=1000 Hz, --fft-size 2048 -- i.e. exactly the configuration
  V2_ARCHITECTURE_current.md documents as "the current test") reported
  Overall RMS ~30% BELOW the time-domain AC RMS:
        X 0.330882 vs 0.475557    Y 0.291468 vs 0.407776
        Z 0.179902 vs 0.266146
  A 1024-sample synthetic tone with an exactly known analytic RMS confirmed
  the error is exactly -1/sqrt(2) = -29.3% at n_real = fft_size/2.

  Root cause: split_into_blocks() zero-pads a short block up to fft_size,
  but the normalization divided by NPG*fft_size**2 -- i.e. by the energy of
  the WHOLE window -- while only the first n_real window samples actually
  multiply real signal. The padded tail contributes window energy to the
  denominator and zero signal energy to the numerator, so the recovered
  mean-square was scaled by sum(w[:n_real]**2)/sum(w**2). For n_real =
  fft_size/2 a symmetric Hann window carries exactly half its energy in its
  first half, giving exactly 0.5 in power = -29.3% in RMS. The V2.1
  real-sample-count weighting does not help: it reweights blocks AGAINST
  EACH OTHER, and a single zero-padded block has nothing to be reweighted
  against, so no correction was applied at all.

  Fix: normalize by the window energy ACTUALLY APPLIED TO REAL SAMPLES,
        denominator = fft_size * sum(w[:n_real]**2)
  For a full block sum(w[:n_real]**2) == sum(w**2) == NPG*fft_size, so this
  reduces to the original NPG*fft_size**2 -- the code keeps that literal
  expression on the full-block path so exact-multiple inputs stay
  BIT-FOR-BIT identical for any fft_size, not merely numerically close.

  Why not "NPG * fft_size * n_real" (the other obvious candidate): it is an
  APPROXIMATION of sum(w[:n_real]**2) that is exact only at n_real=fft_size
  and n_real=fft_size/2. It happens to be exact for captureId2.csv
  (1024/2048 = exactly half), but for the Dewesoft validation file's
  1994-sample trailing block it is wrong by +1.345% in RMS, because a Hann
  window's last 54 samples carry almost no energy while that form assumes
  energy accrues linearly with n_real. sum(w[:n_real]**2) is the exact
  quantity and is correct at every padding ratio.

  New edge case handled: np.hanning(M)[0] is EXACTLY 0.0, so a block with
  n_real=1 has zero window energy. Such a block is now SKIPPED (it carries
  no recoverable information) rather than producing a 0/0 NaN that would
  poison the whole averaged spectrum. The pre-fix code never risked this
  because it always divided by a nonzero constant.

  GUARD: _warn_if_not_parseval_consistent() now checks, on every energy-mode
  run, that the spectrum's bins still sum back to the time-domain AC RMS
  within +-10%, and prints a specific warning to stderr if not. R1 was
  silent for two releases precisely because nothing asserted this invariant.
  See also experimental/test_energy_spectrum_regression.py.

  KNOWN REMAINING LIMITATION (not fixed here, by design): heavy zero-padding
  (fft_size >> N) is now unbiased but increasingly NOISY, because the real
  samples then sit in the near-zero rise of the Hann window and
  sum(w[:n_real]**2) becomes very small. fft_size <= N remains the correct
  configuration for any energy/RMS work; the guard above warns when this
  degrades past 10%.

--------------------------------------------------------------------------
V2 PROJECT PHASE -- MULTI-FORMAT INPUT ARCHITECTURE (post-V1-freeze)
--------------------------------------------------------------------------
NAMING NOTE: "V1" was frozen (as a git commit / feature baseline) with the
Peak Spectrum default, the Energy Spectrum addition, and the V2.1 stability
fixes all already included -- the "V2:" section further above refers to
those PRE-freeze, WITHIN-V1 feature additions (spectrum modes), not this
project phase. This section is the FIRST change made AFTER the V1 freeze,
and is a NEW, SEPARATE use of "V2" -- this time meaning "the next project
phase," i.e. an architectural refactor, not another spectrum-mode feature.
The two are unrelated despite the shared name; this note exists so a future
reader isn't confused by the collision.

GOAL: support a second input format (WTVB05 FIFO CSV, raw ADC counts)
WITHOUT writing a second FFT/RMS/statistics implementation. Every DSP
function in this file (compute_time_domain_stats, hann_window,
coherent_gain, noise_power_gain, split_into_blocks, compute_fft_spectrum,
compute_energy_spectrum, compute_overall_rms, compute_band_rms,
find_dominant_and_noise_floor, analyze_axis) is UNCHANGED by this phase --
verified by re-running the Dewesoft validation file and confirming
summary.json is byte-for-byte identical to the pre-refactor V2.1 baseline
(see V2_ARCHITECTURE.md).

ARCHITECTURE (see V2_ARCHITECTURE.md for the full diagram):

    Input Layer                    read_dewesoft_csv() / read_fifo_csv() --
                                    both return the SAME structure:
                                    {'sample_rate', 'unit', 'X', 'Y', 'Z'}
         |
         v
    Common Analysis Engine         compute_time_domain_stats(),
                                    compute_fft_spectrum() /
                                    compute_energy_spectrum(), analyze_axis()
                                    -- reads plain ndarrays only, never knows
                                    or asks which loader produced them
         |
         v
    Output Layer                   print_axis_report(), build_summary_dict(),
                                    plot_waveform(), plot_fft() -- unchanged

FORMAT DETECTION: detect_input_format() sniffs the first non-blank line of
the input CSV (a Dewesoft "m/s2" column header vs. a WTVB05 "Index,X,Y,Z" or
legacy "FIFOIndex,Tag,SR,..." header) and dispatches to the matching loader
-- the same CLI invocation (`--sr-hz` and nothing else new) works for
either input format, per requirement 5. This mirrors
analyze_fifo_capture.py's own first-line auto-detection IDIOM (not its
code, which is not imported -- see above).

FIFO UNIT CONVERSION (requirement 3): read_fifo_csv() converts raw ADC
counts to m/s^2 via the same formula/constants analyze_fifo_capture.py's
counts_to_g()/g_to_ms2() use (WTVB05 manual Sec 6.1.4.8/6.1.4.16):
    m/s^2 = (raw_counts / 2048) * 9.8
This happens entirely inside the Input Layer, BEFORE any array reaches the
Common Analysis Engine -- the engine only ever sees already-converted
m/s^2 values, identically to the Dewesoft path.

EXPLICITLY NOT IMPLEMENTED in this phase (deferred to later V2 phases, per
requirement 6): cross-format/cross-run comparison, ISO 20816 severity
evaluation, velocity integration, and envelope analysis. None of this
phase's changes prepare specific hooks for those beyond what Energy
Spectrum's Overall RMS / Band RMS already provided in V2.1 -- adding
forward-looking scaffolding for unimplemented features was deliberately
avoided.
"""

import os
import re
import sys
import json
import argparse
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

AXIS_LABELS = ['X', 'Y', 'Z']
DEFAULT_FFT_SIZE = 2048
DEFAULT_OUTPUT_DIR = 'experimental/output'

# [V2.2 R1] Parseval self-check tolerance, in the RMS domain, used only by
# _warn_if_not_parseval_consistent(). Set ~6x looser than the worst
# Hann/NPG approximation error documented in validation_report.md Sec 7.2
# (~1.6%) so it can only ever fire on a genuinely broken normalization --
# the R1 zero-padding defect was -29.3% and would have been caught here.
ENERGY_PARSEVAL_RMS_TOLERANCE = 0.10


# ==========================================================================
# INPUT LAYER -- format-specific loaders, one common output structure.
# See experimental/V2_ARCHITECTURE.md for the full Input Layer / Common
# Analysis Engine / Output Layer diagram. Every loader below returns:
#   {'sample_rate': <Hz, echoed from --sr-hz>, 'unit': 'm/s²',
#    'X': ndarray, 'Y': ndarray, 'Z': ndarray}
# Nothing past this layer (compute_time_domain_stats, compute_fft_spectrum,
# compute_energy_spectrum, analyze_axis, plotting, build_summary_dict) reads
# or branches on which loader produced its input -- they only ever see
# plain ndarrays, unchanged from V1/V2.1.
# ==========================================================================

def find_acceleration_columns(df):
    """Locate the three acceleration (m/s^2) columns in a Dewesoft export.

    Dewesoft column headers look like 'AI 1/AI 1 (m/s2)' (unit suffix may
    render as m/s2, m/s^2, or m/s²). Velocity channels ('.../v (m/s)',
    no square/superscript on the unit) are excluded on purpose -- this tool
    only analyzes acceleration. The first three matching columns are taken,
    in file order, as X/Y/Z (Dewesoft does not label axis identity itself;
    this is the same "trust column order" convention used for
    AI 1/AI 2/AI 3 in the source recording).
    """
    candidates = []
    for col in df.columns:
        lowered = col.lower().replace('^', '').replace('²', '2')
        if 'm/s2' in lowered and '/v' not in lowered:
            candidates.append(col)

    if len(candidates) < 3:
        raise ValueError(
            f"Expected at least 3 acceleration (m/s^2) columns, found "
            f"{len(candidates)}: {candidates}. Available columns: {list(df.columns)}"
        )

    return candidates[:3]


def read_dewesoft_csv(csv_path, sr_hz):
    """Input Loader: Dewesoft CSV -> the common {sample_rate, unit, X, Y, Z}
    structure (see INPUT LAYER banner above). Acceleration is already in
    m/s^2 in a Dewesoft export -- no unit conversion happens here, in file
    row order (no resampling/decimation). This is V1's read path, renamed
    from load_axes() and reshaped to the common structure -- the pandas
    read + find_acceleration_columns() logic is byte-for-byte the same as
    before this refactor.

    [V2.1] Raises ValueError (instead of letting a raw pandas exception
    escape) if the file is completely empty/unreadable as CSV -- one of the
    input-validation gaps confirmed in code_review_v2.md ("Error messages
    are asymmetric"). A header-only CSV (0 data rows) is NOT rejected here
    -- it produces zero-length axis arrays, which main() rejects with a
    clearer, sample-count-specific message (see "fewer than two samples"
    check in main()) since that message applies uniformly regardless of
    whether the shortfall is 0, 1, empty file, or header-only file.
    """
    try:
        df = pd.read_csv(csv_path)
    except pd.errors.EmptyDataError as e:
        raise ValueError(f"CSV file '{csv_path}' is empty or unreadable: {e}")

    accel_cols = find_acceleration_columns(df)
    axes = {
        label: df[col].to_numpy(dtype=float)
        for label, col in zip(AXIS_LABELS, accel_cols)
    }
    return {'sample_rate': sr_hz, 'unit': 'm/s²', **axes}


# --------------------------------------------------------------------------
# WTVB05 FIFO CSV loader -- raw ADC counts -> m/s^2
# --------------------------------------------------------------------------
# Same counts-per-g / gravity constants analyze_fifo_capture.py's
# counts_to_g()/g_to_ms2() use (WTVB05 manual Sec 6.1.4.8/6.1.4.16:
# g = raw_counts / 32768 * 16 = raw_counts / 2048; m/s^2 = g * 9.8).
# Reproduced here as a single, trivial, one-line unit conversion (input
# normalization, not a DSP calculation) rather than importing
# analyze_fifo_capture.py -- that tool has its own separate CLI/scope
# (multi-capture bearing/envelope/spectrogram analysis) and is not meant to
# be depended on as a library; this is the ONE arithmetic line requirement
# 3 asks the Input Layer to apply, not a second FFT/RMS/statistics engine.
FIFO_COUNTS_PER_G = 2048
FIFO_STANDARD_GRAVITY_MS2 = 9.8


def _fifo_counts_to_ms2(raw_counts):
    """Raw WTVB05 FIFO ADC counts -> m/s^2: (counts / 2048) * 9.8."""
    return np.asarray(raw_counts, dtype=float) / FIFO_COUNTS_PER_G * FIFO_STANDARD_GRAVITY_MS2


def _parse_fifo_phase7a(lines):
    """Phase 7A 4-column FIFO format: 'Index,X,Y,Z' header, raw ADC counts.
    Only the FIRST capture block is used if more than one 'Index,X,Y,Z'
    header appears in the file (this engine analyzes one contiguous X/Y/Z
    record per run -- see read_fifo_csv()).
    """
    header_re = re.compile(r'^Index,X,Y,Z\s*$')
    x, y, z = [], [], []
    in_block = False
    for raw_line in lines:
        line = raw_line.rstrip('\r\n')
        if header_re.match(line):
            if in_block:
                break
            in_block = True
            continue
        if in_block:
            parts = line.split(',')
            if len(parts) != 4:
                continue
            try:
                x.append(float(parts[1]))
                y.append(float(parts[2]))
                z.append(float(parts[3]))
            except ValueError:
                continue
    return np.array(x), np.array(y), np.array(z)


def _parse_fifo_legacy_first_block(lines):
    """Legacy 9-column FIFO format:
    'FIFOIndex,Tag,SR,X_raw,Y_raw,Z_raw,X_g,Y_g,Z_g' header, one or more
    'FIFO_CAPTURE_END'-delimited blocks. Only the FIRST block is used (same
    reason as _parse_fifo_phase7a()); the precomputed X_g/Y_g/Z_g columns
    are intentionally ignored -- raw X_raw/Y_raw/Z_raw counts are read and
    converted via _fifo_counts_to_ms2() so both FIFO formats go through the
    identical conversion path (requirement 3).
    """
    header_re = re.compile(r'^FIFOIndex,Tag,SR,X_raw,Y_raw,Z_raw,X_g,Y_g,Z_g\s*$')
    x, y, z = [], [], []
    in_block = False
    for raw_line in lines:
        line = raw_line.rstrip('\r\n')
        if header_re.match(line):
            if in_block:
                break
            in_block = True
            continue
        if line.strip() == 'FIFO_CAPTURE_END':
            if in_block:
                break
            continue
        if in_block:
            parts = line.split(',')
            if len(parts) != 9:
                continue
            try:
                x.append(float(parts[3]))
                y.append(float(parts[4]))
                z.append(float(parts[5]))
            except ValueError:
                continue
    return np.array(x), np.array(y), np.array(z)


def read_fifo_csv(csv_path, sr_hz):
    """Input Loader: WTVB05 FIFO CSV -> the common {sample_rate, unit, X, Y,
    Z} structure (see INPUT LAYER banner above). Supports the same two
    capture formats analyze_fifo_capture.py documents (legacy 9-column and
    Phase 7A 4-column, auto-detected from the first non-blank header line --
    see detect_input_format()). Converts raw ADC counts to m/s^2 via
    _fifo_counts_to_ms2() (requirement 3's counts/2048*9.8 formula) before
    returning -- everything downstream of this function sees m/s^2, exactly
    like read_dewesoft_csv(), and cannot tell the two loaders apart.
    """
    with open(csv_path, encoding='utf-8', errors='replace') as f:
        lines = f.readlines()

    first_line = ''
    for raw_line in lines:
        stripped = raw_line.strip()
        if stripped:
            first_line = stripped
            break

    if re.match(r'^Index,X,Y,Z\s*$', first_line):
        x_raw, y_raw, z_raw = _parse_fifo_phase7a(lines)
    elif re.match(r'^FIFOIndex,Tag,SR,X_raw,Y_raw,Z_raw,X_g,Y_g,Z_g\s*$', first_line):
        x_raw, y_raw, z_raw = _parse_fifo_legacy_first_block(lines)
    else:
        raise ValueError(
            f"Unrecognized FIFO CSV header in '{csv_path}': {first_line!r} "
            f"(expected 'Index,X,Y,Z' or 'FIFOIndex,Tag,SR,X_raw,Y_raw,Z_raw,X_g,Y_g,Z_g')"
        )

    if len(x_raw) == 0:
        raise ValueError(f"No FIFO samples found in '{csv_path}'.")

    return {
        'sample_rate': sr_hz,
        'unit': 'm/s²',
        'X': _fifo_counts_to_ms2(x_raw),
        'Y': _fifo_counts_to_ms2(y_raw),
        'Z': _fifo_counts_to_ms2(z_raw),
    }


def detect_input_format(csv_path):
    """Sniff csv_path's first non-blank line and return 'dewesoft' or
    'fifo'. Mirrors analyze_fifo_capture.py's own first-line auto-detection
    IDIOM (not its code) -- this tool's Input Layer needs to make the same
    kind of format decision, but between a different pair of formats
    (Dewesoft acceleration export vs. WTVB05 FIFO capture).
    """
    with open(csv_path, encoding='utf-8', errors='replace') as f:
        first_line = ''
        for raw_line in f:
            stripped = raw_line.strip()
            if stripped:
                first_line = stripped
                break

    lowered = first_line.lower().replace('^', '').replace('²', '2')
    if 'm/s2' in lowered:
        return 'dewesoft'
    if re.match(r'^Index,X,Y,Z\s*$', first_line) or \
       re.match(r'^FIFOIndex,Tag,SR,X_raw,Y_raw,Z_raw,X_g,Y_g,Z_g\s*$', first_line):
        return 'fifo'

    raise ValueError(
        f"Could not detect input format for '{csv_path}' from its header line "
        f"({first_line!r}); expected a Dewesoft acceleration CSV (a column containing "
        f"'m/s2') or a WTVB05 FIFO CSV ('Index,X,Y,Z' or "
        f"'FIFOIndex,Tag,SR,X_raw,Y_raw,Z_raw,X_g,Y_g,Z_g')."
    )


# ==========================================================================
# TIME-DOMAIN STATISTICS
# ==========================================================================

def compute_time_domain_stats(signal):
    """Mean(DC)/AC-RMS/Peak/Peak-to-Peak/StdDev/CrestFactor/Kurtosis/Skewness
    for one axis.

    DC = signal.mean(); everything else is computed on the mean-removed
    (AC) signal, per standard vibration-analysis convention (matches
    Dewesoft's own Peak/RMS math widgets, which report AC quantities on an
    AC-coupled or DC-removed channel).

    [v16.3ae time-domain milestone] Kurtosis and Skewness are computed on
    the SAME AC signal as RMS/Peak/StdDev/CrestFactor, using population
    (N-divisor, ddof=0) moments -- matching std_dev's existing np.std()
    convention -- so all time-domain statistics in this function stay
    internally consistent:
      kurtosis  = mean(ac**4) / std(ac)**4 - 3   (excess/Fisher kurtosis;
                                                    0 for a Gaussian)
      skewness  = mean(ac**3) / std(ac)**3       (population skewness;
                                                    0 for a symmetric signal)
    Frozen definitions -- see experimental/V2_ARCHITECTURE.md discussion;
    not to be changed without a corresponding regression re-run.
    """
    signal = np.asarray(signal, dtype=float)
    dc = signal.mean()
    ac = signal - dc

    ac_rms = np.sqrt(np.mean(ac ** 2))
    peak = np.max(np.abs(ac))
    peak_to_peak = np.max(ac) - np.min(ac)
    std_dev = np.std(ac)
    crest_factor = peak / ac_rms if ac_rms > 0 else float('nan')
    kurtosis = np.mean(ac ** 4) / std_dev ** 4 - 3.0 if std_dev > 0 else float('nan')
    skewness = np.mean(ac ** 3) / std_dev ** 3 if std_dev > 0 else float('nan')

    return {
        'dc': dc,
        'ac_rms': ac_rms,
        'peak': peak,
        'peak_to_peak': peak_to_peak,
        'std_dev': std_dev,
        'crest_factor': crest_factor,
        'kurtosis': kurtosis,
        'skewness': skewness,
        'ac': ac,
    }


# ==========================================================================
# FFT -- Hann window, coherent-gain correction, block-averaging
# ==========================================================================

def hann_window(n):
    """Hann (Hanning) window of length n, matching np.hanning's symmetric
    (non-periodic) definition -- the conventional choice for a
    single, non-overlapped analysis block (as opposed to the periodic/DFT
    variant used for overlapped STFT processing).
    """
    return np.hanning(n)


def coherent_gain(window):
    """Hann coherent gain = mean of the window samples.

    Applying a window attenuates signal amplitude (a Hann window's samples
    average ~0.5 of full scale); dividing by this factor after the FFT
    restores true amplitude, which is what lets windowed peak readings be
    compared directly against an unwindowed time-domain Peak figure -- this
    is the correction Dewesoft's own "Peak (Auto)" display applies.
    """
    return np.mean(window)


def noise_power_gain(window):
    """Hann noise power gain, NPG = mean of the SQUARED window samples.

    This is the factor (not Coherent Gain) that keeps a windowed signal's
    average POWER consistent with the unwindowed signal's power -- the
    standard normalization used by Welch/periodogram-style PSD estimation.
    Used only by the Energy Spectrum path (compute_energy_spectrum()); the
    Peak Spectrum path (compute_fft_spectrum()) uses coherent_gain()
    instead and is unaffected by this function's existence.
    """
    return np.mean(window ** 2)


def split_into_blocks(signal, fft_size):
    """Split a signal into consecutive, non-overlapping fft_size blocks.

    - If the signal is shorter than fft_size, returns a single zero-padded
      block.
    - If longer, splits into as many full fft_size blocks as fit; a
      shorter final remainder block is zero-padded and included (matches
      Dewesoft's FFT preview, which keeps recomputing FFT blocks across the
      whole capture rather than discarding the tail).

    Returns (blocks, weights): weights[i] is the number of REAL
    (non-zero-padded) samples block i actually contains. [V2.1 fix] A
    zero-padded trailing block must NOT count the same as a full block when
    blocks are later averaged together (code_review_v2.md Possible Bugs
    #1 -- confirmed a signal of length fft_size+1 got its reported
    amplitude cut in half, because a 1-real-sample straggler block was
    averaged at equal (50%) weight against a full block). Callers must use
    these weights (e.g. np.average(..., weights=weights)) instead of a
    plain np.mean() across blocks -- see compute_fft_spectrum() /
    compute_energy_spectrum().
    """
    n = len(signal)
    if n <= fft_size:
        padded = np.zeros(fft_size)
        padded[:n] = signal
        return [padded], [n]

    blocks = []
    weights = []
    for start in range(0, n, fft_size):
        block = signal[start:start + fft_size]
        real_len = len(block)
        if real_len < fft_size:
            padded = np.zeros(fft_size)
            padded[:real_len] = block
            block = padded
        blocks.append(block)
        weights.append(real_len)
    return blocks, weights


def compute_fft_spectrum(ac_signal, fft_size, sr_hz):
    """Hann-windowed, coherent-gain-corrected, single-sided PEAK amplitude
    spectrum, averaged across consecutive fft_size blocks (see module
    docstring DEWESOFT FFT CONVENTION for the full step-by-step rationale).

    [V2.1] Block combination is a REAL-SAMPLE-COUNT-WEIGHTED mean
    (np.average(..., weights=weights)), not a plain np.mean() -- see
    split_into_blocks()'s docstring and code_review_v2.md Possible Bugs #1.
    For a signal whose length is an exact multiple of fft_size (all blocks
    full) or that fits in a single block, every weight is equal/there is
    only one block, so this is numerically IDENTICAL to the old plain
    mean -- this only changes output for a signal with a genuine partial
    trailing block, which is exactly the case that was confirmed wrong.

    Returns (freqs, peak_amplitude_spectrum, n_blocks).
    """
    window = hann_window(fft_size)
    gain = coherent_gain(window)

    blocks, weights = split_into_blocks(ac_signal, fft_size)
    spectra = []
    for block in blocks:
        windowed = block * window
        spectrum = np.fft.rfft(windowed)
        mag = np.abs(spectrum) / fft_size  # normalize by FFT length
        mag = mag / gain                   # correct for Hann coherent gain

        # Single-sided: double all bins except DC and (if present) Nyquist,
        # to fold the discarded negative-frequency half back onto the
        # positive side -- see FFT_CONVENTION note in analyze_fifo_capture.py
        # for the same rationale applied to the unwindowed case.
        mag = mag * 2.0
        mag[0] /= 2.0
        if fft_size % 2 == 0:
            mag[-1] /= 2.0

        spectra.append(mag)

    avg_spectrum = np.average(spectra, axis=0, weights=weights)
    freqs = np.fft.rfftfreq(fft_size, d=1.0 / sr_hz) if sr_hz else np.fft.rfftfreq(fft_size, d=1.0)

    return freqs, avg_spectrum, len(blocks)


def _warn_if_not_parseval_consistent(ac_signal, ms_avg, fft_size):
    """[V2.2 R1 GUARD] Warn on stderr if the Energy Spectrum's bins no longer
    sum back to the time-domain AC mean-square.

    This is the invariant compute_energy_spectrum() exists to provide, and it
    is the exact invariant the R1 zero-padding defect violated -- silently,
    for every capture shorter than fft_size, by a factor of
    sqrt(n_real/fft_size) (~-29.3% for the production 1024-sample capture at
    fft_size=2048). A guard here means that class of defect can only ever
    return LOUDLY.

    Compared in the RMS domain (not power) so the printed percentage is
    directly comparable to the +-0.3-1.6% figures quoted in
    validation_report.md Sec 7.2. The tolerance is deliberately ~6x looser
    than the worst documented approximation error: this must detect a broken
    normalization, never second-guess the documented Hann/NPG behaviour.
    """
    ac = np.asarray(ac_signal, dtype=float)
    td_rms = float(np.sqrt(np.mean(ac ** 2)))
    if td_rms <= 0.0:
        return  # all-zero signal: 0 == 0, nothing to check

    spec_rms = float(np.sqrt(np.sum(ms_avg)))
    rel_err = abs(spec_rms - td_rms) / td_rms
    if rel_err > ENERGY_PARSEVAL_RMS_TOLERANCE:
        print(
            f"WARNING: Energy Spectrum is not Parseval-consistent "
            f"(time-domain AC RMS {td_rms:.6f} vs spectrum-summed RMS "
            f"{spec_rms:.6f}, {100.0 * (spec_rms / td_rms - 1.0):+.2f}%, "
            f"tolerance +-{100.0 * ENERGY_PARSEVAL_RMS_TOLERANCE:.0f}%). "
            f"N={len(ac)} fft_size={fft_size}. Overall/Band RMS from this "
            f"spectrum are NOT trustworthy. Heavy zero-padding "
            f"(fft_size >> N) is the most likely cause -- prefer "
            f"fft_size <= N.",
            file=sys.stderr,
        )


def compute_energy_spectrum(ac_signal, fft_size, sr_hz):
    """Hann-windowed, Noise-Power-Gain-corrected, single-sided RMS-amplitude
    spectrum that preserves signal energy (Parseval) -- see module
    docstring "V2: PEAK SPECTRUM vs ENERGY SPECTRUM" for the full
    rationale. Independent of, and does not alter, compute_fft_spectrum()
    (the unchanged Peak Spectrum path) -- both start from the same
    Hann-windowed rFFT but diverge at the correction/averaging step.

    Averages per-bin POWER (mean-square) across blocks, not magnitude --
    summing power (not magnitude) across bins is what Parseval's theorem
    relates to time-domain mean-square, so power-domain averaging is what
    keeps Overall RMS / Band RMS energy-consistent block-to-block (see
    compute_overall_rms() / compute_band_rms()).

    Returns (freqs, rms_spectrum, mean_square_per_bin, n_blocks):
      - rms_spectrum[k]        = sqrt(mean_square_per_bin[k]), an RMS
                                 (not peak) amplitude-per-bin spectrum.
      - mean_square_per_bin[k] = block-averaged mean-square contribution of
                                 bin k; sum(mean_square_per_bin) is the
                                 Parseval-consistent estimate of the AC
                                 signal's mean-square (== AC RMS**2).

    [V2.1] Block combination is a REAL-SAMPLE-COUNT-WEIGHTED mean, same fix
    and same rationale as compute_fft_spectrum() -- see that function's
    docstring and code_review_v2.md Possible Bugs #1. This bug affected
    Energy Spectrum identically to Peak Spectrum (both consumed
    split_into_blocks()'s output with equal per-block weight); fixed here
    the same way, for the same reason.
    """
    window = hann_window(fft_size)
    npg = noise_power_gain(window)
    window_sq = window ** 2

    blocks, weights = split_into_blocks(ac_signal, fft_size)
    power_spectra = []
    block_weights = []
    for block, n_real in zip(blocks, weights):
        # [V2.2 R1] Normalize by the window energy ACTUALLY APPLIED TO REAL
        # SAMPLES, not by the full-length window energy. For a full block
        # these are the same quantity and the expression below is the
        # ORIGINAL one, bit-for-bit; for a zero-padded block they differ,
        # and using the full-length one is what diluted the measured energy
        # by n_real/fft_size (see R1 note in the module docstring).
        if n_real >= fft_size:
            denom = npg * fft_size ** 2                       # UNCHANGED path
        else:
            denom = fft_size * float(np.sum(window_sq[:n_real]))

        # [V2.2 R1] np.hanning(M)[0] is EXACTLY 0.0, so a 1-real-sample
        # trailing block has zero window energy -- the window annihilates it
        # and it carries no recoverable information. Skip it rather than
        # dividing by zero (which the pre-fix code never risked, because it
        # always divided by the full-window constant).
        if denom <= 0.0:
            continue

        windowed = block * window
        spectrum = np.fft.rfft(windowed)

        # Windowed one-sided Parseval: mean(x**2) ~= (1/(M*Sw)) *
        # [ |X0|^2 + 2*sum(|Xk|^2, k=1..M/2-1) + |X_{M/2}|^2 ], where
        # Sw = sum(w[n]**2) over the samples that actually carry signal
        # (== NPG*M for a full block, hence the NPG*M^2 form above). Note
        # the M*Sw, not M: mean-square is a POWER quantity, so it scales
        # with the square of the amplitude normalization
        # compute_fft_spectrum() uses for its M-scaled AMPLITUDE spectrum.
        # ms[k] is bin k's own contribution to that sum -- same
        # fold-then-undo-DC/Nyquist pattern as compute_fft_spectrum(), just
        # in the power domain (|.|^2) and normalized by window energy
        # instead of CG.
        ms = (np.abs(spectrum) ** 2) / denom
        ms = ms * 2.0
        ms[0] /= 2.0
        if fft_size % 2 == 0:
            ms[-1] /= 2.0

        power_spectra.append(ms)
        block_weights.append(n_real)

    if not power_spectra:
        raise ValueError(
            f"No usable FFT block in a {len(ac_signal)}-sample signal at "
            f"fft_size={fft_size}: every block was annihilated by the window."
        )

    ms_avg = np.average(power_spectra, axis=0, weights=block_weights)
    rms_spectrum = np.sqrt(ms_avg)
    freqs = np.fft.rfftfreq(fft_size, d=1.0 / sr_hz) if sr_hz else np.fft.rfftfreq(fft_size, d=1.0)

    # [V2.2 R1 GUARD] Parseval self-check. The whole point of this spectrum
    # is that its bins sum (in power) back to the time-domain AC mean-square;
    # the R1 defect broke exactly that invariant and did so SILENTLY for two
    # releases. A gross mismatch now says so on stderr instead of returning a
    # plausible-looking but wrong number. The threshold is deliberately loose
    # (10% in RMS) so it can never fire on the documented ~0.3-1.6% Hann/NPG
    # approximation -- it is a defect detector, not an accuracy assertion.
    _warn_if_not_parseval_consistent(ac_signal, ms_avg, fft_size)

    return freqs, rms_spectrum, ms_avg, len(power_spectra)


def compute_overall_rms(ms_avg):
    """Overall RMS from an Energy Spectrum's block-averaged mean-square
    array (see compute_energy_spectrum()). Summing power across all bins
    and taking the square root is what Parseval's theorem relates to the
    signal's own time-domain AC RMS.

    NOTE (honest limitation, not the Peak Spectrum's ~20%+ systematic
    overstatement -- see module docstring): because this path applies a
    Hann window, the NPG correction is only an approximate energy-recovery
    factor (exact in expectation for a stationary/broadband signal whose
    energy is uncorrelated with the window's shape; not an exact identity
    for an arbitrary finite real signal, since windowing permanently
    tapers/discards information at the block edges). Measured against
    validation_report.md's test file this Overall RMS is typically within
    ~0.3-1.6% of the time-domain AC RMS -- Parseval-*consistent*, not
    bit-exact. Bit-exact equality (~1e-14%) is only achievable with a
    rectangular/no-window transform (see validation_report.md Sec 4.2(A)).
    """
    return float(np.sqrt(np.sum(ms_avg)))


def compute_band_rms(freqs, ms_avg, band_hz):
    """RMS energy within [band_hz[0], band_hz[1]] Hz (inclusive), same
    Parseval-consistent power-summation as compute_overall_rms() but
    restricted to the bins whose frequency falls in the given band -- the
    building block ISO 20816 (and any other banded RMS/velocity-limit
    calculation) needs, since those standards define pass/fail bands in
    Hz, not single tones.
    """
    lo, hi = band_hz
    mask = (freqs >= lo) & (freqs <= hi)
    return float(np.sqrt(np.sum(ms_avg[mask])))


def compute_fft_resolution_hz(sr_hz, fft_size):
    """FFT bin spacing dF = SR / fft_size (Hz per bin)."""
    return sr_hz / fft_size if sr_hz else None


def find_dominant_and_noise_floor(freqs, spectrum):
    """Dominant frequency/amplitude (largest bin, DC excluded) plus a
    broadband noise-floor estimate and the resulting SNR (dB).

    Noise floor = median amplitude of all non-DC bins (see module docstring
    ASSUMPTIONS -- this is a reasonable broadband estimate, not a
    guaranteed match to Dewesoft's own undocumented algorithm).
    """
    non_dc = spectrum[1:]
    dominant_idx = int(np.argmax(non_dc)) + 1
    dominant_freq = freqs[dominant_idx]
    dominant_amp = spectrum[dominant_idx]

    noise_floor = float(np.median(non_dc))
    if noise_floor > 0 and dominant_amp > 0:
        snr_db = 20.0 * np.log10(dominant_amp / noise_floor)
    else:
        snr_db = float('nan')

    return dominant_freq, dominant_amp, noise_floor, snr_db


# ==========================================================================
# PER-AXIS ANALYSIS (pure computation, no plotting)
# ==========================================================================

def analyze_axis(signal, sr_hz, fft_size, spectrum_mode='peak', band_hz=None):
    """Run the full time-domain + FFT pipeline for one axis and return a
    single dict with everything the plots/console-report/summary.json need.

    spectrum_mode='peak' (default) is the ORIGINAL V1 Peak Spectrum path
    (compute_fft_spectrum) -- UNCHANGED, byte-for-byte identical return
    shape/values to V1. spectrum_mode='energy' instead uses
    compute_energy_spectrum() and additionally reports Overall RMS (and
    Band RMS if band_hz is given) -- see module docstring "V2: PEAK
    SPECTRUM vs ENERGY SPECTRUM".
    """
    time_stats = compute_time_domain_stats(signal)

    if spectrum_mode == 'energy':
        freqs, spectrum, ms_avg, n_blocks = compute_energy_spectrum(time_stats['ac'], fft_size, sr_hz)
        dominant_freq, dominant_amp, noise_floor, snr_db = find_dominant_and_noise_floor(freqs, spectrum)
        result = {
            **time_stats,
            'freqs': freqs,
            'spectrum': spectrum,
            'n_fft_blocks': n_blocks,
            'dominant_freq_hz': dominant_freq,
            'dominant_amplitude': dominant_amp,
            'noise_floor': noise_floor,
            'snr_db': snr_db,
            'overall_rms': compute_overall_rms(ms_avg),
        }
        if band_hz is not None:
            result['band_rms'] = compute_band_rms(freqs, ms_avg, band_hz)
            result['band_range_hz'] = tuple(band_hz)
        return result

    # --- Peak Spectrum (default) -- UNCHANGED from the V1 implementation ---
    freqs, spectrum, n_blocks = compute_fft_spectrum(time_stats['ac'], fft_size, sr_hz)
    dominant_freq, dominant_amp, noise_floor, snr_db = find_dominant_and_noise_floor(freqs, spectrum)

    return {
        **time_stats,
        'freqs': freqs,
        'spectrum': spectrum,
        'n_fft_blocks': n_blocks,
        'dominant_freq_hz': dominant_freq,
        'dominant_amplitude': dominant_amp,
        'noise_floor': noise_floor,
        'snr_db': snr_db,
    }


# ==========================================================================
# PLOTTING -- Dewesoft-style dark theme (V1 visual-only release, see
# release_notes_v1_visual.md). Cosmetic constants/helper only: nothing here
# reads or writes any computed value (RMS/FFT/Peak/Energy/stats/JSON) --
# every function in this section only receives already-computed numbers to
# draw, exactly as before this change.
# ==========================================================================

# Bright, high-contrast-on-black per-axis trace colors, matching Dewesoft's
# own AI1(red)/AI2(blue)/AI3(green) convention (see the reference screenshot
# this tool was originally built to match).
AXIS_COLORS = {'X': '#FF3030', 'Y': '#3399FF', 'Z': '#33CC33'}

DARK_FIGURE_BG = '#000000'   # figure background: black
DARK_AXES_BG = '#101010'     # axes background: very dark gray
GRID_COLOR = '#404040'       # thin gray grid, similar to Dewesoft
TEXT_COLOR = 'white'         # axis labels / tick labels / titles
# Dominant-frequency marker/annotation color in fft.png: deliberately NOT
# reused from AXIS_COLORS -- on the X subplot the trace itself is now red,
# so an annotation in the old 'tab:red' would blend into its own trace and
# become illegible. A color outside the X/Y/Z set keeps it visible on every
# subplot regardless of that subplot's axis color.
ANNOTATION_COLOR = '#FFD400'


def _apply_dark_theme(fig, axs):
    """Apply the black-figure / dark-gray-axes / white-text Dewesoft-style
    theme to an already-built figure. Purely cosmetic (colors, grid style,
    text color) -- does not touch axis limits, data, or any plotted value.
    """
    fig.patch.set_facecolor(DARK_FIGURE_BG)
    for ax in axs:
        ax.set_facecolor(DARK_AXES_BG)
        ax.grid(True, color=GRID_COLOR, linewidth=0.5, alpha=0.6)
        ax.tick_params(colors=TEXT_COLOR, labelcolor=TEXT_COLOR)
        ax.xaxis.label.set_color(TEXT_COLOR)
        ax.yaxis.label.set_color(TEXT_COLOR)
        # NOTE: per-axes titles (loc='left' in plot_waveform()) are NOT
        # colored here -- ax.title refers only to the CENTER title, a
        # separate (unused, empty) Text artist from the left/right title
        # matplotlib creates for loc='left'/'right'. Each ax.set_title(...)
        # call passes color=TEXT_COLOR directly at the call site instead.
        # Only recolor spines -- visibility (top/right hidden, bottom/left
        # shown) is unchanged from before this release; a hidden spine's
        # color has no visual effect regardless.
        for spine in ax.spines.values():
            spine.set_color(TEXT_COLOR)


def plot_waveform(axes_data, sample_rate_hz, out_path):
    """waveform.png -- time-domain X/Y/Z with DC/AC-RMS/Peak in each title."""
    fig, axs = plt.subplots(3, 1, figsize=(11, 8), sharex=True)
    for ax, label in zip(axs, AXIS_LABELS):
        stats = axes_data[label]
        signal = stats['dc'] + stats['ac']
        t = (np.arange(len(signal)) / sample_rate_hz) if sample_rate_hz else np.arange(len(signal))

        ax.plot(t, signal, linewidth=0.5, color=AXIS_COLORS[label])
        ax.set_ylabel(f'{label} Accel (m/s^2)')
        ax.spines['top'].set_visible(False)
        ax.spines['right'].set_visible(False)
        ax.set_title(
            f"{label}: DC={stats['dc']:.4f}  AC RMS={stats['ac_rms']:.4f}  "
            f"Peak={stats['peak']:.4f} m/s^2",
            fontsize=9, loc='left', color=TEXT_COLOR,
        )
    axs[-1].set_xlabel('Time (s)' if sample_rate_hz else 'Sample index')
    fig.suptitle('Time Waveform — X / Y / Z (m/s^2)', color=TEXT_COLOR)
    _apply_dark_theme(fig, axs)
    plt.tight_layout()
    plt.savefig(out_path, dpi=150, facecolor=fig.get_facecolor())
    plt.close(fig)


def plot_fft(axes_data, out_path, spectrum_mode='peak'):
    """fft.png -- single-sided amplitude spectrum X/Y/Z, dominant frequency
    annotated on each subplot.

    spectrum_mode='peak' (default) reproduces the ORIGINAL, UNCHANGED Peak
    Spectrum plot (identical labels/title/values to V1). 'energy' instead
    plots the RMS-amplitude Energy Spectrum -- see module docstring.
    """
    amp_unit = 'RMS' if spectrum_mode == 'energy' else 'Peak'

    fig, axs = plt.subplots(3, 1, figsize=(11, 8))
    for ax, label in zip(axs, AXIS_LABELS):
        stats = axes_data[label]
        ax.plot(stats['freqs'], stats['spectrum'], linewidth=0.8, color=AXIS_COLORS[label])
        ax.axvline(stats['dominant_freq_hz'], color=ANNOTATION_COLOR, linestyle='--', linewidth=1.0)
        ax.annotate(
            f"{stats['dominant_freq_hz']:.2f} Hz\n{stats['dominant_amplitude']:.4f} m/s^2",
            xy=(stats['dominant_freq_hz'], stats['dominant_amplitude']),
            xytext=(10, 10), textcoords='offset points',
            fontsize=8, color=ANNOTATION_COLOR,
        )
        ax.set_ylabel(f'{label} {amp_unit} Amplitude (m/s^2)')
        ax.spines['top'].set_visible(False)
        ax.spines['right'].set_visible(False)
    axs[-1].set_xlabel('Frequency (Hz)')
    if spectrum_mode == 'energy':
        fig.suptitle('FFT — Energy Spectrum (RMS Amplitude), Hann window, single-sided, Parseval-consistent',
                     color=TEXT_COLOR)
    else:
        fig.suptitle('FFT — Peak (Auto) Amplitude, Hann window, single-sided', color=TEXT_COLOR)
    _apply_dark_theme(fig, axs)
    plt.tight_layout()
    plt.savefig(out_path, dpi=150, facecolor=fig.get_facecolor())
    plt.close(fig)


# ==========================================================================
# CONSOLE REPORT + summary.json
# ==========================================================================

def print_axis_report(label, stats, spectrum_mode='peak'):
    print(f"\nAxis {label}")
    print("-------")
    print(f"Mean (DC)          : {stats['dc']:.6f} m/s^2")
    print(f"AC RMS             : {stats['ac_rms']:.6f} m/s^2")
    print(f"Peak               : {stats['peak']:.6f} m/s^2")
    print(f"Peak-to-Peak       : {stats['peak_to_peak']:.6f} m/s^2")
    print(f"Std Dev            : {stats['std_dev']:.6f} m/s^2")
    print(f"Crest Factor       : {stats['crest_factor']:.4f}")
    print(f"Kurtosis (excess)  : {stats['kurtosis']:.4f}")
    print(f"Skewness           : {stats['skewness']:.4f}")
    print(f"Dominant Frequency : {stats['dominant_freq_hz']:.4f} Hz")
    if spectrum_mode == 'energy':
        print(f"Dominant Amplitude (RMS) : {stats['dominant_amplitude']:.6f} m/s^2")
    else:
        print(f"Dominant Amplitude : {stats['dominant_amplitude']:.6f} m/s^2")
    print(f"Noise Floor        : {stats['noise_floor']:.6f} m/s^2")
    print(f"SNR                : {stats['snr_db']:.2f} dB")
    if spectrum_mode == 'energy':
        print(f"Overall RMS (Energy Spectrum, Parseval-consistent) : {stats['overall_rms']:.6f} m/s^2")
        if 'band_rms' in stats:
            lo, hi = stats['band_range_hz']
            print(f"Band RMS [{lo:g}-{hi:g} Hz] : {stats['band_rms']:.6f} m/s^2")


def build_summary_dict(axes_data, sample_rate_hz, fft_size, fft_resolution_hz, spectrum_mode='peak'):
    def axis_summary(stats):
        base = {
            'mean_dc': stats['dc'],
            'ac_rms': stats['ac_rms'],
            'peak': stats['peak'],
            'peak_to_peak': stats['peak_to_peak'],
            'std_dev': stats['std_dev'],
            'crest_factor': stats['crest_factor'],
            'kurtosis': stats['kurtosis'],
            'skewness': stats['skewness'],
            'dominant_frequency_hz': stats['dominant_freq_hz'],
            'dominant_amplitude': stats['dominant_amplitude'],
            'noise_floor': stats['noise_floor'],
            'snr_db': stats['snr_db'],
            'n_fft_blocks_averaged': stats['n_fft_blocks'],
        }
        if spectrum_mode == 'energy':
            base['overall_rms'] = stats['overall_rms']
            if 'band_rms' in stats:
                base['band_rms'] = stats['band_rms']
                base['band_range_hz'] = list(stats['band_range_hz'])
        return base

    return {
        'sampling_rate': sample_rate_hz,
        'fft_size': fft_size,
        'fft_resolution': fft_resolution_hz,
        'spectrum_mode': spectrum_mode,
        'axes': {label: axis_summary(axes_data[label]) for label in AXIS_LABELS},
    }


# ==========================================================================
# MAIN
# ==========================================================================

def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('csv_path',
                     help='Input CSV -- auto-detected as either a Dewesoft acceleration export '
                          '(Time + 3 "m/s2" columns) or a WTVB05 FIFO capture '
                          '("Index,X,Y,Z" or legacy "FIFOIndex,Tag,SR,..." raw-count format). '
                          'See detect_input_format() / V2_ARCHITECTURE.md.')
    ap.add_argument('--sr-hz', type=float, required=True, help='Sample rate in Hz')
    ap.add_argument('--fft-size', type=int, default=DEFAULT_FFT_SIZE,
                     help=f'FFT block size (default {DEFAULT_FFT_SIZE}); shorter input is zero-padded, '
                          'longer input is split into consecutive blocks and averaged')
    ap.add_argument('--out-dir', default=DEFAULT_OUTPUT_DIR,
                     help=f'Output directory for waveform.png / fft.png / summary.json '
                          f'(default: {DEFAULT_OUTPUT_DIR}; created automatically if missing)')
    ap.add_argument('--spectrum', choices=['peak', 'energy'], default='peak',
                     help="Spectrum processing path (default: peak). 'peak' is the ORIGINAL, "
                          "UNCHANGED Dewesoft-matching Peak (Auto) amplitude spectrum. 'energy' is "
                          "a Parseval-consistent RMS amplitude spectrum supporting Overall RMS / "
                          "Band RMS -- the foundation for future ISO 20816 work. See module "
                          "docstring 'V2: PEAK SPECTRUM vs ENERGY SPECTRUM'.")
    ap.add_argument('--band-hz', type=float, nargs=2, metavar=('LO', 'HI'), default=None,
                     help='Frequency band [LO HI] Hz for Band RMS (energy mode only; ignored in peak mode)')
    args = ap.parse_args()

    # --- [V2.1] Argument validation -------------------------------------
    # Reject nonsensical inputs up front with a clear message instead of
    # letting them silently produce NaN/mislabeled output or crash deep in
    # the pipeline with an unrelated traceback (code_review_v2.md Possible
    # Bugs #3, #4, #5). ap.error() prints usage + the message to stderr and
    # exits with status 2, matching argparse's own convention for bad args.
    if args.fft_size <= 0:
        ap.error(f"--fft-size must be a positive integer (got {args.fft_size})")
    if args.sr_hz <= 0:
        ap.error(f"--sr-hz must be a positive number (got {args.sr_hz})")
    if args.band_hz is not None:
        lo, hi = args.band_hz
        if lo < 0 or hi <= lo:
            ap.error(f"--band-hz LO HI must satisfy 0 <= LO < HI (got LO={lo:g}, HI={hi:g})")

    if args.band_hz is not None and args.spectrum != 'energy':
        print("Note: --band-hz is ignored outside --spectrum energy mode.")

    os.makedirs(args.out_dir, exist_ok=True)

    # --- Input Layer: detect format, dispatch to the matching loader -----
    # Both loaders return the identical {sample_rate, unit, X, Y, Z}
    # structure (see INPUT LAYER banner above / V2_ARCHITECTURE.md) -- the
    # Common Analysis Engine below this point is unchanged from V1/V2.1 and
    # cannot tell which loader ran.
    try:
        input_format = detect_input_format(args.csv_path)
        if input_format == 'dewesoft':
            loaded = read_dewesoft_csv(args.csv_path, args.sr_hz)
        else:
            loaded = read_fifo_csv(args.csv_path, args.sr_hz)
    except ValueError as e:
        print(f"Error: {e}", file=sys.stderr)
        sys.exit(1)

    raw_axes = {label: loaded[label] for label in AXIS_LABELS}

    # fewer than two samples (includes a header-only/empty-data CSV, where
    # every axis array has length 0) -- DC/RMS/FFT are not meaningful below
    # this, and split_into_blocks()/compute_time_domain_stats() do not
    # themselves guard against it.
    for label in AXIS_LABELS:
        if len(raw_axes[label]) < 2:
            print(f"Error: axis {label} has only {len(raw_axes[label])} sample(s) in "
                  f"'{args.csv_path}'; at least 2 samples per axis are required.",
                  file=sys.stderr)
            sys.exit(1)

    fft_resolution_hz = compute_fft_resolution_hz(args.sr_hz, args.fft_size)

    print(f"Loaded {len(raw_axes['X'])} samples per axis from {args.csv_path} "
          f"(input format: {input_format})")
    print(f"Sample rate: {args.sr_hz:g} Hz   FFT size: {args.fft_size}   "
          f"FFT Resolution (dF=SR/FFT size): {fft_resolution_hz:.4f} Hz   "
          f"Spectrum mode: {args.spectrum}")

    band_hz = args.band_hz if args.spectrum == 'energy' else None
    axes_data = {
        label: analyze_axis(raw_axes[label], args.sr_hz, args.fft_size,
                             spectrum_mode=args.spectrum, band_hz=band_hz)
        for label in AXIS_LABELS
    }

    for label in AXIS_LABELS:
        print_axis_report(label, axes_data[label], spectrum_mode=args.spectrum)

    waveform_path = os.path.join(args.out_dir, 'waveform.png')
    fft_path = os.path.join(args.out_dir, 'fft.png')
    summary_path = os.path.join(args.out_dir, 'summary.json')

    plot_waveform(axes_data, args.sr_hz, waveform_path)
    plot_fft(axes_data, fft_path, spectrum_mode=args.spectrum)

    summary = build_summary_dict(axes_data, args.sr_hz, args.fft_size, fft_resolution_hz,
                                  spectrum_mode=args.spectrum)
    with open(summary_path, 'w', encoding='utf-8') as f:
        json.dump(summary, f, indent=2)

    print(f"\nSaved: {waveform_path}")
    print(f"Saved: {fft_path}")
    print(f"Saved: {summary_path}")


if __name__ == '__main__':
    main()
