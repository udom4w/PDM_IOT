#!/usr/bin/env python3
"""
analyze_fifo_capture.py — RFC-0006 experiment analysis tool

Reads a Serial Monitor log (or a plain CSV) containing one or more `FIFO`
command captures from WTVB05_ValidationTool v3.0, and for each capture:

Supports two input formats, auto-detected from the file's first non-blank
line:

  - legacy 9-column format:
      FIFOIndex,Tag,SR,X_raw,Y_raw,Z_raw,X_g,Y_g,Z_g
    (one or more blocks per file, each terminated by a 'FIFO_CAPTURE_END'
    sentinel line; may be embedded in a raw serial log with surrounding
    boot/debug noise)

  - Phase 7A 4-column format:
      Index,X,Y,Z
    (X/Y/Z are raw ADC counts -- no Tag/SR fields. A block runs from its
    header line to the next header line or end of file.)

  1. Plots the full 1024-sample time series per axis (X, Y, Z) in g.
  2. Computes an FFT independently from the raw samples and plots the
     peak-amplitude magnitude spectrum per axis, in g.
  3. Computes DC offset, RMS (mean-removed/AC) and Peak (AC) per axis from
     the raw samples, each reported in raw counts, g, and (RMS/Peak only)
     m/s^2.
  4. Prints a summary table so these FIFO-derived numbers can be compared
     directly against the *register-reported* RRAX/VRMSX/HZX-type values
     from the same test session (RFC-0006 Experiment 3 — the single most
     decisive test proposed in that review).
  5. Writes a machine-readable `<out-prefix>_summary.csv` with one row per
     capture (tag, format, RMS/Peak/dominant-frequency per axis, and the
     paths to the waveform/FFT PNGs generated in step 1-2), so downstream
     tooling doesn't have to scrape stdout (Phase 7B-1).
  6. Plots a Short-Time Fourier Transform (STFT) spectrogram per axis via
     scipy.signal.spectrogram() (explicit Hann window, 50% overlap, window
     length auto-sized to the capture length, one shared colorbar for the
     whole figure) and records its path/window/overlap in the summary CSV
     (Phase 7B-2). This is independent of, and does not alter, the whole-
     capture FFT in step 2.
  7. Plots the envelope of the mean-removed (g) waveform per axis via
     scipy.signal.hilbert() (envelope = abs(hilbert(signal)); no band-pass
     filter yet -- that's a later phase) and records its path in the
     summary CSV (Phase 7B-3).
  8. Computes a plain np.fft.rfft() of that same envelope signal (DC-
     removed, no window function yet) per axis, plots it, and records its
     dominant frequency and PNG path in the summary CSV (Phase 7B-4).
  9. Optionally, if --manufacturer and --bearing are both given, looks up
     the bearing's geometry in the external bearing_database/bearing_db.json
     database and prints/records it (Z, ball diameter, pitch diameter,
     contact angle) in the summary CSV (Phase 7C-1, revised to an external
     JSON database instead of a hard-coded dict).
  10. If a bearing was selected AND --shaft-rpm was given (and is > 0),
      computes the standard characteristic frequencies (shaft_hz, FTF,
      BPFO, BPFI, BSF) via compute_bearing_frequencies() and records them
      in the summary CSV. Pure calculation only -- no plot, no overlay, no
      peak search, no diagnosis. RPM is never guessed: without a valid
      --shaft-rpm these columns are left blank (Phase 7C-2).
  11. Overlays those same characteristic frequencies as dashed vertical
      lines (distinct color + Hz-labeled legend entry each) on the
      Envelope FFT plot from step 8, capped to below Nyquist (sr_hz/2).
      Plot-only: does not touch the envelope-FFT calculation, the FFT
      algorithm, or any CSV column. Draws nothing if --shaft-rpm wasn't
      supplied (Phase 7C-3).

Usage:
    python3 analyze_fifo_capture.py serial_log.txt
    python3 analyze_fifo_capture.py serial_log.txt --sr-hz 1000

If --sr-hz is not given, the script uses the SR value embedded in the CSV
(from the SR register's *index*, not Hz) only to report it — it does NOT
guess the sample rate in Hz, since that mapping (SR index -> Hz) lives in
the firmware's SAMPLE_RATE_HZ table, not in the capture itself. Pass
--sr-hz explicitly for correct FFT frequency-axis labeling.

This script deliberately does NOT reference any vendor claim about what the
FIFO contains — it only computes what the numbers say, per RFC-0006's
instruction to distinguish hypotheses experimentally.

--------------------------------------------------------------------------
ENGINEERING UNITS (WTVB05 Data Sheet & User Manual V260403, wit-motion.com)
--------------------------------------------------------------------------
Both capture formats carry raw FIFO accelerometer ADC counts. Per the
manual:

  Sec 6.1.4.8  AX~AZ (acceleration):
    "Acceleration X = AX[15:0]/32768*16g (g is the acceleration due to
     gravity, which can be taken as 9.8m/s2)"
  Sec 6.1.4.16 Original Acceleration FIFO (the RAWFIFO register this
  tool's captures come from):
    "The acceleration XYZ data is converted to g, as described in
     [AX~AZ] acceleration data calculation."

i.e. the manual states the FIFO raw samples use the *exact same* 16-bit
signed, +-16g-full-scale mapping as the AX~AZ registers. That is why this
tool now converts raw counts to g the same way for BOTH the legacy and
Phase 7A capture formats (see counts_to_g() below), instead of only
trusting a pre-computed g column that was only ever present in the legacy
format. This was cross-checked against a real legacy-format capture line
(serial_log_backup_20260719_202918_4.txt: X_raw=2015, X_g=0.98389; and
2015/2048 = 0.983887, matching to 5 decimal places), confirming the
firmware's own g column already uses this formula.

ASSUMPTIONS (flagged per the manual's own wording):
  - The manual says gravity "can be taken as 9.8m/s2" -- it does not commit
    to the standard 9.80665 m/s^2 constant. This tool uses the manual's own
    9.8 value (STANDARD_GRAVITY_MS2 below) so m/s^2 numbers match what
    you'd get evaluating the manual's formula by hand. If you need
    standard-gravity precision, change STANDARD_GRAVITY_MS2 and this note
    together.
  - The FFT peak-amplitude scaling (|rfft|/n*2) does not special-case the
    Nyquist bin (which, strictly, should not be doubled). For the 1024-
    sample FIFO captures this tool targets, that bin is far from the
    dominant peaks of interest; full-spectrum energy math should account
    for it separately.
  - Phase 7A's raw counts are assumed to come from the same RAWFIFO
    acceleration path as the legacy format's X_raw/Y_raw/Z_raw (both are
    captures of the same FIFO register per the tool's own docstring); this
    is what makes applying the identical manual formula to Phase 7A valid.
"""

import sys
import os
import re
import csv
import json
import math
import argparse
import numpy as np
from scipy.signal import spectrogram as scipy_spectrogram
from scipy.signal.windows import hann
from scipy.signal import hilbert
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

SUMMARY_CSV_FIELDS = [
    'tag', 'input_format', 'sample_count', 'sample_rate_hz',
    # rms_x/y/z and peak_x/y/z are AC (mean-removed) acceleration in g --
    # for the legacy format these numerically match the old pre-Phase-7A
    # behavior (see module docstring cross-check); for Phase 7A they are
    # now real g values instead of raw ADC counts (see dc_offset_*/
    # rms_ac_*/peak_ac_* columns below for the full counts/g/m-s^2
    # breakdown of the same numbers).
    'rms_x', 'rms_y', 'rms_z',
    'peak_x', 'peak_y', 'peak_z',
    'dominant_freq_x_hz', 'dominant_freq_y_hz', 'dominant_freq_z_hz',
    'waveform_png', 'fft_png', 'spectrogram_png',
    'spectrogram_window', 'spectrogram_overlap', 'envelope_png', 'envelope_method',
    'dominant_envelope_freq_x_hz', 'dominant_envelope_freq_y_hz', 'dominant_envelope_freq_z_hz',
    'envelope_fft_png',
    'bearing_model', 'rolling_elements', 'ball_diameter_mm', 'pitch_diameter_mm', 'contact_angle_deg',
    'shaft_hz', 'FTF', 'BPFO', 'BPFI', 'BSF',
    # Full DC/RMS/Peak breakdown across all three engineering units,
    # per-axis (added for engineering-unit-correctness pass; append-only
    # so older summary CSV readers that key off column name are
    # unaffected).
    'dc_offset_counts_x', 'dc_offset_counts_y', 'dc_offset_counts_z',
    'dc_offset_g_x', 'dc_offset_g_y', 'dc_offset_g_z',
    'rms_ac_counts_x', 'rms_ac_counts_y', 'rms_ac_counts_z',
    'rms_ac_ms2_x', 'rms_ac_ms2_y', 'rms_ac_ms2_z',
    'peak_ac_counts_x', 'peak_ac_counts_y', 'peak_ac_counts_z',
    'peak_ac_ms2_x', 'peak_ac_ms2_y', 'peak_ac_ms2_z',
    # Engineering-polish pass: FFT resolution (dF = SR/N). sample_rate_hz
    # and sample_count already exist above (unchanged) -- only this one
    # column is new; see compute_fft_resolution_hz().
    'fft_resolution_hz',
]

# External bearing geometry database, Phase 7C-1 (revised architecture --
# replaces the earlier hard-coded BEARING_DB dict). Structure:
# {"<manufacturer>": {"<model>": {rolling_elements, ball_diameter_mm,
# pitch_diameter_mm, contact_angle_deg}}}. No fault-frequency math is done
# with these values yet, that's a later phase.
BEARING_DB_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'bearing_database', 'bearing_db.json')

BEARING_DB_REQUIRED_KEYS = {'rolling_elements', 'ball_diameter_mm', 'pitch_diameter_mm', 'contact_angle_deg'}


# ==========================================================================
# ACCELERATION PROCESSING -- unit conversion (WTVB05 manual Sec 6.1.4.8 /
# 6.1.4.16, see module docstring for the exact quoted formulas)
#
# Kept structurally separate from any future VELOCITY / VRMS processing:
# adding a parallel "compute_velocity_axis_stats()" that integrates g to
# mm/s (Sec 6.1.4.9 VX~VZ) for comparison against register VRMSX/Y/Z (Sec
# 6.1.4.17-19), plus an Acceleration-vs-Velocity-vs-VRMS comparison step,
# should slot in below without touching this section. No velocity
# integration is implemented yet -- see the placeholder stub at the bottom
# of this section.
# ==========================================================================

ACCEL_FULLSCALE_G = 16.0        # WTVB05 accelerometer full-scale range (+-16g)
ACCEL_COUNTS_FULLSCALE = 32768  # signed 16-bit half-range (2**15), per AX[15:0]
COUNTS_PER_G = ACCEL_COUNTS_FULLSCALE / ACCEL_FULLSCALE_G  # 2048 counts/g

# ASSUMPTION: manual says gravity "can be taken as 9.8m/s2" (Sec 6.1.4.8),
# not the standard 9.80665 m/s^2 -- see module docstring note.
STANDARD_GRAVITY_MS2 = 9.8

# --- FFT convention (engineering-polish Sec 1/5) --------------------------
# Single source of truth for the whole-capture FFT AND the Envelope FFT
# below -- both use this identical convention, so it is documented once
# here and referenced from both plot titles instead of being restated
# (and risking drifting out of sync) in two places.
#   Type:    single-sided (np.fft.rfft on a real input -- only the
#            non-negative frequencies 0..Nyquist are computed/plotted;
#            there is no separate negative-frequency half to discard).
#   Window:  none (rectangular/boxcar) -- no window function is applied
#            before either FFT.
#   Scaling: peak (0-to-peak) amplitude = |rfft(x)| * 2 / N. The *2
#            accounts for the single-sided spectrum folding the (discarded)
#            negative-frequency half back onto the positive side; dividing
#            by N normalizes for capture length so amplitude doesn't grow
#            just because N grew.
#   DC:      removed from the input before the FFT (x = signal -
#            signal.mean()); the DC bin is also excluded from the
#            dominant-frequency search.
# WHY peak amplitude (not RMS-per-bin or raw/unnormalized coefficients):
# it reads directly in the same g/counts/m-s^2 units as the time-domain
# Peak(AC) figure reported alongside it (no extra unit conversion needed
# to sanity-check one against the other), and is the conventional
# amplitude-spectrum convention in vibration analysis. The time-domain
# RMS(AC) figure (a single broadband number) is a DIFFERENT quantity from
# a per-bin RMS spectrum -- this tool does not compute the latter.
# The STFT Spectrogram plot below uses a DIFFERENT convention (Hann
# window, short-time magnitude, no *2/N peak normalization) -- see the
# NOTE beside its colorbar; its color values are NOT directly comparable
# to this FFT's amplitude numbers.
FFT_CONVENTION = ("single-sided rFFT, no window (rectangular), "
                   "peak (0-to-peak) amplitude = |rfft|*2/N, DC removed pre-FFT")


def counts_to_g(raw_counts):
    """Raw FIFO/AX~AZ ADC counts -> g, per WTVB05 manual Sec 6.1.4.8/6.1.4.16:
    g = raw_counts / 32768 * 16.
    """
    return np.asarray(raw_counts, dtype=float) / ACCEL_COUNTS_FULLSCALE * ACCEL_FULLSCALE_G


def g_to_ms2(g_values):
    """g -> m/s^2 using the manual's own gravity constant (Sec 6.1.4.8 note)."""
    return np.asarray(g_values, dtype=float) * STANDARD_GRAVITY_MS2


def counts_to_ms2(raw_counts):
    """Raw FIFO/AX~AZ ADC counts -> m/s^2 (composes counts_to_g + g_to_ms2)."""
    return g_to_ms2(counts_to_g(raw_counts))


def extract_raw_axes(cap):
    """Centralized raw-count extraction: pull X/Y/Z FIFO ADC counts out of a
    parsed capture dict, as float arrays, for both capture formats (legacy
    and Phase 7A both carry x_raw/y_raw/z_raw -- see parse_captures()).
    Kept as a single reusable helper so acceleration processing has exactly
    one place that reads cap['x_raw']/'y_raw'/'z_raw'.
    """
    return {
        'X': np.array(cap['x_raw'], dtype=float),
        'Y': np.array(cap['y_raw'], dtype=float),
        'Z': np.array(cap['z_raw'], dtype=float),
    }


def compute_acceleration_axis_stats(raw_counts, sr_hz, n):
    """Pure computation (no plotting) of one axis's acceleration numbers.

    Takes raw FIFO ADC counts for one axis and returns counts/g/m-s^2
    versions of: DC offset, AC (mean-removed) signal, RMS(AC), Peak(AC),
    and the FFT magnitude spectrum -- a single-sided PEAK (0-to-peak)
    amplitude spectrum, i.e. |rfft(ac)| * 2 / n (see module docstring
    ASSUMPTIONS re: the Nyquist bin). Also returns the dominant AC
    frequency (bin with the largest peak amplitude, DC bin excluded).
    """
    counts = np.asarray(raw_counts, dtype=float)
    g = counts_to_g(counts)
    ms2 = g_to_ms2(g)

    dc_counts = counts.mean()
    dc_g = g.mean()
    dc_ms2 = ms2.mean()

    ac_counts = counts - dc_counts
    ac_g = g - dc_g
    ac_ms2 = ms2 - dc_ms2

    rms_counts = np.sqrt(np.mean(ac_counts ** 2))
    rms_g = np.sqrt(np.mean(ac_g ** 2))
    rms_ms2 = np.sqrt(np.mean(ac_ms2 ** 2))

    peak_counts = np.max(np.abs(ac_counts))
    peak_g = np.max(np.abs(ac_g))
    peak_ms2 = np.max(np.abs(ac_ms2))

    freqs = np.fft.rfftfreq(n, d=1.0 / sr_hz) if sr_hz else np.fft.rfftfreq(n, d=1.0)
    fft_mag_counts = np.abs(np.fft.rfft(ac_counts)) / n * 2
    fft_mag_g = np.abs(np.fft.rfft(ac_g)) / n * 2
    fft_mag_ms2 = np.abs(np.fft.rfft(ac_ms2)) / n * 2
    dominant_idx = np.argmax(fft_mag_g[1:]) + 1  # skip DC bin
    dominant_freq = freqs[dominant_idx]

    return {
        'counts': counts, 'g': g, 'ms2': ms2,
        'dc_counts': dc_counts, 'dc_g': dc_g, 'dc_ms2': dc_ms2,
        'ac_counts': ac_counts, 'ac_g': ac_g, 'ac_ms2': ac_ms2,
        'rms_counts': rms_counts, 'rms_g': rms_g, 'rms_ms2': rms_ms2,
        'peak_counts': peak_counts, 'peak_g': peak_g, 'peak_ms2': peak_ms2,
        'freqs': freqs,
        'fft_mag_counts': fft_mag_counts, 'fft_mag_g': fft_mag_g, 'fft_mag_ms2': fft_mag_ms2,
        'dominant_freq': dominant_freq,
    }


# ==========================================================================
# VELOCITY PROCESSING -- not implemented yet.
#
# Future home for integrating acceleration (g) to vibration velocity
# (mm/s, matching WTVB05 manual Sec 6.1.4.9 VX~VZ) so it can be compared
# against the register-reported VRMSX/VRMSY/VRMSZ (Sec 6.1.4.17-19). Left
# unimplemented per explicit instruction -- do NOT implement velocity
# integration yet. When it lands, it should read compute_acceleration_axis_
# stats()'s 'ac_g' (or 'ac_ms2') array and sample rate, and return a
# parallel counts/mm-s^2-analog dict shaped like the one above so a
# combined Acceleration/Velocity/VRMS comparison step can consume both
# uniformly.
# ==========================================================================


# ==========================================================================
# CAPTURE METADATA (engineering-polish Sec 2/3) -- Sampling Rate, Number of
# Samples, and FFT Resolution (dF = SR/N), computed once per capture and
# reused verbatim in the console report, the summary CSV, and every plot's
# footer annotation, so those three never disagree with each other about
# what SR/N/dF were for a given capture.
# ==========================================================================

def compute_fft_resolution_hz(sr_hz, n):
    """FFT bin spacing dF = SR/N (Hz per bin). None if sr_hz wasn't given --
    same "never guess the sample rate" rule the module docstring already
    applies to the frequency axis: without --sr-hz, dF is only meaningful
    in cycles/block (1/N), not Hz.
    """
    return sr_hz / n if sr_hz else None


def format_capture_metadata_line(tag, n, sr_hz, fft_resolution_hz):
    """One-line 'Capture / Samples / SR / FFT Resolution' summary -- reused
    for console output, the small per-plot footer annotation, and (SR/N
    already existed; fft_resolution_hz is the new column) the summary CSV,
    so the numbers are guaranteed to match across all three.
    """
    sr_part = f"{sr_hz:g} Hz" if sr_hz else "not given (pass --sr-hz)"
    df_part = f"{fft_resolution_hz:.4f} Hz" if fft_resolution_hz else "N/A (needs --sr-hz)"
    return f"Capture: {tag}   Samples: {n}   SR: {sr_part}   FFT Resolution (dF=SR/N): {df_part}"


def annotate_capture_metadata(fig, metadata_line):
    """Small, low-emphasis footer so every plot can stand alone without the
    console log or CSV alongside it (Sec 3). Kept to one line / small font
    / low contrast so it doesn't compete with the actual data ("do not
    overcrowd the plots").
    """
    fig.text(0.995, 0.004, metadata_line, ha='right', va='bottom', fontsize=7, color='0.4')


def load_bearing_database(path):
    """Read the bearing geometry JSON database once and validate its shape.

    Expected shape: {manufacturer: {model: {rolling_elements,
    ball_diameter_mm, pitch_diameter_mm, contact_angle_deg}}}. On any
    problem -- missing file, invalid JSON, or a shape that doesn't match --
    prints a clear, specific error and exits cleanly (sys.exit(1)) rather
    than letting a traceback surface.
    """
    try:
        with open(path, encoding='utf-8') as f:
            db = json.load(f)
    except FileNotFoundError:
        print(f"Bearing database not found: {path}")
        sys.exit(1)
    except json.JSONDecodeError as e:
        print(f"Bearing database is not valid JSON ({path}): {e}")
        sys.exit(1)

    if not isinstance(db, dict):
        print(f"Bearing database malformed ({path}): top level must be an "
              f"object mapping manufacturer -> {{model: geometry}}")
        sys.exit(1)
    for manufacturer, models in db.items():
        if not isinstance(models, dict):
            print(f"Bearing database malformed ({path}): manufacturer "
                  f"'{manufacturer}' must map to an object of bearing models")
            sys.exit(1)
        for model, geometry in models.items():
            if not isinstance(geometry, dict) or not BEARING_DB_REQUIRED_KEYS.issubset(geometry):
                print(f"Bearing database malformed ({path}): '{manufacturer}/{model}' "
                      f"is missing one of {sorted(BEARING_DB_REQUIRED_KEYS)}")
                sys.exit(1)

    return db


def compute_bearing_frequencies(shaft_rpm, rolling_elements, ball_diameter_mm,
                                 pitch_diameter_mm, contact_angle_deg):
    """Pure calculation of standard bearing characteristic frequencies, Phase 7C-2.

    Geometry (rolling_elements=Z, ball_diameter_mm=Bd, pitch_diameter_mm=Pd,
    contact_angle_deg) is passed in from the Phase 7C-1 JSON database --
    this function does not duplicate or hard-code any bearing geometry.

    Never guesses shaft RPM: if shaft_rpm is None or <= 0, every frequency
    is returned as None instead of being computed from an assumed value.

    Uses the standard rolling-element-bearing equations:
        shaft_hz = shaft_rpm / 60
        FTF  = (shaft_hz / 2) * (1 - (Bd/Pd) * cos(contact_angle))
        BPFO = (Z / 2) * shaft_hz * (1 - (Bd/Pd) * cos(contact_angle))
        BPFI = (Z / 2) * shaft_hz * (1 + (Bd/Pd) * cos(contact_angle))
        BSF  = (Pd / (2*Bd)) * shaft_hz * (1 - ((Bd/Pd) * cos(contact_angle)) ** 2)

    This function only computes and returns numbers -- no plotting, no
    overlay onto any existing plot, no peak search, no diagnosis.
    """
    if shaft_rpm is None or shaft_rpm <= 0:
        return {'shaft_hz': None, 'FTF': None, 'BPFO': None, 'BPFI': None, 'BSF': None}

    shaft_hz = shaft_rpm / 60.0
    Z = rolling_elements
    bd_over_pd = ball_diameter_mm / pitch_diameter_mm
    cos_angle = math.cos(math.radians(contact_angle_deg))

    ftf = (shaft_hz / 2.0) * (1 - bd_over_pd * cos_angle)
    bpfo = (Z / 2.0) * shaft_hz * (1 - bd_over_pd * cos_angle)
    bpfi = (Z / 2.0) * shaft_hz * (1 + bd_over_pd * cos_angle)
    bsf = (pitch_diameter_mm / (2.0 * ball_diameter_mm)) * shaft_hz * (1 - (bd_over_pd * cos_angle) ** 2)

    return {'shaft_hz': shaft_hz, 'FTF': ftf, 'BPFO': bpfo, 'BPFI': bpfi, 'BSF': bsf}


def _spectrogram_nperseg(n):
    """Pick an STFT window length for an n-sample capture.

    Targets ~8 windows across the block (n // 8), rounded down to the
    nearest power of two, clamped to [16, 256] (and to n itself for very
    short captures) -- a reasonable time/frequency tradeoff for the 1024-
    sample FIFO blocks this tool normally sees, without hard-coding 1024.
    """
    target = max(16, n // 8)
    nperseg = 1 << (target.bit_length() - 1)
    return max(2, min(nperseg, 256, n))


def parse_captures(path):
    """Parse one or more FIFO capture blocks out of a file.

    Auto-detects the CSV format from the file's first non-blank line and
    dispatches to the matching parser. Both parsers return the same shape:
    a list of dicts {format, tag, sr, index, x_raw, y_raw, z_raw, x_g, y_g, z_g}.
    'format' is 'legacy' (X/Y/Z_g are g-scaled) or 'phase7a' (X/Y/Z_raw are
    raw ADC counts, X/Y/Z_g unavailable).
    """
    with open(path, encoding='utf-8', errors='replace') as f:
        lines = f.readlines()

    first_line = ''
    for raw_line in lines:
        stripped = raw_line.strip()
        if stripped:
            first_line = stripped
            break

    if re.match(r'^Index,X,Y,Z\s*$', first_line):
        return _parse_captures_phase7a(lines)
    return _parse_captures_legacy(lines)


def _parse_captures_legacy(lines):
    """Legacy 9-column format: FIFOIndex,Tag,SR,X_raw,Y_raw,Z_raw,X_g,Y_g,Z_g

    One or more blocks per file, each terminated by a 'FIFO_CAPTURE_END'
    sentinel line; may be embedded in a raw serial log with surrounding
    boot/debug noise.
    """
    captures = []
    current = None
    header_re = re.compile(r'^FIFOIndex,Tag,SR,X_raw,Y_raw,Z_raw,X_g,Y_g,Z_g\s*$')

    for raw_line in lines:
        line = raw_line.rstrip('\r\n')
        if header_re.match(line):
            current = {'format': 'legacy', 'tag': None, 'sr': None, 'index': [],
                       'x_raw': [], 'y_raw': [], 'z_raw': [], 'x_g': [], 'y_g': [], 'z_g': []}
            continue
        if line.strip() == 'FIFO_CAPTURE_END':
            if current is not None and len(current['index']) > 0:
                captures.append(current)
            current = None
            continue
        if current is not None:
            parts = line.split(',')
            if len(parts) != 9:
                continue
            try:
                idx = int(parts[0])
                tag = parts[1]
                sr = int(parts[2])
                x_raw, y_raw, z_raw = int(parts[3]), int(parts[4]), int(parts[5])
                x_g, y_g, z_g = float(parts[6]), float(parts[7]), float(parts[8])
            except ValueError:
                continue
            current['tag'] = tag
            current['sr'] = sr
            current['index'].append(idx)
            current['x_raw'].append(x_raw); current['y_raw'].append(y_raw); current['z_raw'].append(z_raw)
            current['x_g'].append(x_g); current['y_g'].append(y_g); current['z_g'].append(z_g)

    return captures


def _parse_captures_phase7a(lines):
    """Phase 7A 4-column format: Index,X,Y,Z (X/Y/Z are raw ADC counts).

    Unlike the legacy format, this CSV carries no separate raw/g pair --
    X/Y/Z are the raw counts, full stop. This tool now converts those raw
    counts to g itself (see counts_to_g() / module docstring), so x_g/y_g/
    z_g are still left empty here (this format has no *pre-computed* g
    column from the firmware to parse) but are no longer required
    downstream -- analyze_one() derives g from x_raw/y_raw/z_raw for both
    formats. This format also carries no Tag/SR fields, so those are
    synthesized (tag = 'phase7a_N' per block, sr = None). A block runs
    from its header line to the next header line or end of file -- there is
    no END sentinel.
    """
    header_re = re.compile(r'^Index,X,Y,Z\s*$')
    captures = []
    current = None
    block_num = 0

    def finalize():
        if current is not None and len(current['index']) > 0:
            captures.append(current)

    for raw_line in lines:
        line = raw_line.rstrip('\r\n')
        if header_re.match(line):
            finalize()
            block_num += 1
            current = {'format': 'phase7a', 'tag': f'phase7a_{block_num}', 'sr': None, 'index': [],
                       'x_raw': [], 'y_raw': [], 'z_raw': [], 'x_g': [], 'y_g': [], 'z_g': []}
            continue
        if current is not None:
            parts = line.split(',')
            if len(parts) != 4:
                continue
            try:
                idx = int(parts[0])
                x_raw, y_raw, z_raw = float(parts[1]), float(parts[2]), float(parts[3])
            except ValueError:
                continue
            current['index'].append(idx)
            current['x_raw'].append(x_raw); current['y_raw'].append(y_raw); current['z_raw'].append(z_raw)

    finalize()
    return captures


def analyze_one(cap, sr_hz, out_prefix, bearing_model=None, bearing=None, bearing_freqs=None):
    n = len(cap['index'])
    print(f"\n=== Capture tag='{cap['tag']}' SR_index={cap['sr']} samples={n} ===")

    # --- Capture metadata (Sec 2/3): computed once, reused verbatim in the
    # console line below, every plot's footer annotation, and the summary
    # CSV row built at the end of this function.
    fft_resolution_hz = compute_fft_resolution_hz(sr_hz, n)
    metadata_line = format_capture_metadata_line(cap['tag'], n, sr_hz, fft_resolution_hz)
    print(f"  {metadata_line}")

    # --- ACCELERATION PROCESSING ------------------------------------
    # Both capture formats carry raw FIFO ADC counts (x_raw/y_raw/z_raw).
    # Per the manual (Sec 6.1.4.16 + 6.1.4.8, see module docstring), those
    # raw counts use the identical +-16g / 32768-count scaling regardless
    # of which capture format produced them, so both are converted to g
    # here the same way -- this replaces the old per-format unit branch
    # (Phase 7A used to be plotted/reported in unconverted raw counts).
    raw_axes = extract_raw_axes(cap)
    stats = {label: compute_acceleration_axis_stats(raw, sr_hz, n) for label, raw in raw_axes.items()}

    # --- Time-series plot, all 1024 points per axis, in g -----------
    fig, axs = plt.subplots(3, 1, figsize=(11, 8), sharex=True)
    for ax, label in zip(axs, ['X', 'Y', 'Z']):
        ax.plot(cap['index'], stats[label]['g'], linewidth=0.7)
        ax.set_ylabel(f'{label} Acceleration (g)')
        ax.grid(alpha=0.2)
        ax.spines['top'].set_visible(False)
        ax.spines['right'].set_visible(False)
    axs[-1].set_xlabel('Sample index (0-1023 within this FIFO block)')
    fig.suptitle(f"FIFO acceleration waveform — tag='{cap['tag']}' (all {n} samples; "
                 f"counts -> g per WTVB05 manual Sec 6.1.4.8/6.1.4.16)")
    plt.tight_layout()
    annotate_capture_metadata(fig, metadata_line)
    ts_path = f"{out_prefix}_{cap['tag']}_timeseries.png"
    plt.savefig(ts_path, dpi=150)
    plt.close()
    print(f"  saved: {ts_path}")

    # --- FFT + DC/RMS/Peak per axis ----------------------------------
    fig, axs = plt.subplots(3, 1, figsize=(11, 8))
    for ax, label in zip(axs, ['X', 'Y', 'Z']):
        s = stats[label]
        ax.plot(s['freqs'], s['fft_mag_g'], linewidth=0.8)
        ax.set_ylabel(f'{label} FFT Peak Amplitude (g)')
        ax.grid(alpha=0.2)
        ax.spines['top'].set_visible(False)
        ax.spines['right'].set_visible(False)
        xunit = 'Hz' if sr_hz else 'cycles/1024-sample-block'
        ax.set_title(f"{label}: DC={s['dc_g']:.4f}g  RMS(AC)={s['rms_g']:.4f}g  "
                      f"Peak(AC)={s['peak_g']:.4f}g  dominant~{s['dominant_freq']:.1f}{xunit}",
                      fontsize=9, loc='left')
    axs[-1].set_xlabel(f"Frequency ({'Hz' if sr_hz else 'cycles per 1024-sample block -- pass --sr-hz for real Hz'})")
    # Title states type/window/scaling/DC-handling explicitly (Sec 1) --
    # see FFT_CONVENTION above for the full rationale/comment.
    fig.suptitle(f"FIFO-derived FFT — tag='{cap['tag']}'\n{FFT_CONVENTION}", fontsize=10)
    plt.tight_layout()
    annotate_capture_metadata(fig, metadata_line)
    fft_path = f"{out_prefix}_{cap['tag']}_fft.png"
    plt.savefig(fft_path, dpi=150)
    plt.close()
    print(f"  saved: {fft_path}")

    # --- Spectrogram (STFT) per axis, Phase 7B-2 -- independent of the
    # whole-capture FFT above; does not read or modify its results.
    nperseg = _spectrogram_nperseg(n)
    noverlap = nperseg // 2
    fs = sr_hz if sr_hz else 1.0
    window = hann(nperseg, sym=False)

    spec_per_axis = {}
    vmin, vmax = None, None
    for label in ['X', 'Y', 'Z']:
        ac_g = stats[label]['ac_g']
        f_spec, t_spec, Sxx = scipy_spectrogram(ac_g, fs=fs, window=window, nperseg=nperseg,
                                                 noverlap=noverlap, detrend=False, mode='magnitude')
        spec_per_axis[label] = (f_spec, t_spec, Sxx)
        vmin = Sxx.min() if vmin is None else min(vmin, Sxx.min())
        vmax = Sxx.max() if vmax is None else max(vmax, Sxx.max())

    fig, axs = plt.subplots(3, 1, figsize=(11, 8), sharex=True)
    mesh = None
    for ax, label in zip(axs, ['X', 'Y', 'Z']):
        f_spec, t_spec, Sxx = spec_per_axis[label]
        mesh = ax.pcolormesh(t_spec, f_spec, Sxx, shading='auto', cmap='viridis', vmin=vmin, vmax=vmax)
        ax.set_ylabel(f'{label} Freq ({"Hz" if sr_hz else "cycles/sample"})')
    axs[-1].set_xlabel(f"Time ({'s' if sr_hz else 'samples -- pass --sr-hz for seconds'})")
    # NOTE (avoiding the ambiguous-"Magnitude" pitfall -- see module
    # docstring ENGINEERING UNITS section): scipy.signal.spectrogram(...,
    # mode='magnitude') returns the magnitude of each short-time (Hann-
    # windowed) DFT frame. This is NOT the same single-sided peak-amplitude
    # convention as the whole-capture FFT above (windowing attenuates
    # amplitude relative to a rectangular full-block FFT, so the two
    # numbers are not directly comparable) -- labeled distinctly as "STFT
    # Magnitude" here rather than reusing "Peak Amplitude".
    fig.suptitle(f"FIFO-derived Spectrogram — STFT Magnitude (g), Hann window={nperseg}, "
                 f"50% overlap, DC removed pre-STFT — tag='{cap['tag']}'")
    fig.colorbar(mesh, ax=axs, label='STFT Magnitude (g)')
    annotate_capture_metadata(fig, metadata_line)
    spectrogram_path = f"{out_prefix}_{cap['tag']}_spectrogram.png"
    plt.savefig(spectrogram_path, dpi=150)
    plt.close()
    print(f"  saved: {spectrogram_path}")

    # --- Envelope (Hilbert transform) per axis, Phase 7B-3 -- uses the
    # mean-removed (AC) g waveform directly, no band-pass filter yet
    # (that's a later phase). Independent of FFT/spectrogram; does not
    # read or modify their results.
    envelope_x = np.array(cap['index']) / sr_hz if sr_hz else cap['index']
    envelope_per_axis = {}
    fig, axs = plt.subplots(3, 1, figsize=(11, 8), sharex=True)
    for ax, label in zip(axs, ['X', 'Y', 'Z']):
        envelope = np.abs(hilbert(stats[label]['ac_g']))
        envelope_per_axis[label] = envelope
        ax.plot(envelope_x, envelope, linewidth=0.8)
        ax.set_ylabel(f'{label} Envelope (g)')
        ax.grid(alpha=0.2)
        ax.spines['top'].set_visible(False)
        ax.spines['right'].set_visible(False)
    axs[-1].set_xlabel('Time (s)' if sr_hz else 'Sample index (0-1023 within this FIFO block)')
    fig.suptitle(f"FIFO-derived Envelope (Hilbert, g) — tag='{cap['tag']}' (all {n} samples)")
    plt.tight_layout()
    annotate_capture_metadata(fig, metadata_line)
    envelope_path = f"{out_prefix}_{cap['tag']}_envelope.png"
    plt.savefig(envelope_path, dpi=150)
    plt.close()
    print(f"  saved: {envelope_path}")

    # --- FFT of the envelope per axis, Phase 7B-4 -- reuses the exact
    # envelope arrays computed above (abs(hilbert(mean-removed g
    # waveform))); does not recompute the envelope differently. The
    # envelope itself is a positive-only signal with a nonzero mean, so
    # its own DC is removed here (envelope_ac = envelope - envelope.mean())
    # before the FFT -- otherwise that DC bin would dominate and hide the
    # modulation frequencies we're looking for. Same single-sided PEAK
    # amplitude convention as the waveform FFT above; no window function
    # in this phase.
    fig, axs = plt.subplots(3, 1, figsize=(11, 8))
    envelope_fft_summary = {}

    # --- Bearing characteristic frequency overlay, Phase 7C-3 -- plot-only,
    # does not touch the envelope-FFT calculation above or below. Only
    # drawn when a bearing was selected AND --shaft-rpm produced real
    # frequencies (bearing_freqs values non-None) AND the axis is actually
    # in Hz (sr_hz given); RPM is never estimated, no peak search is done.
    # Each line is capped to below Nyquist (sr_hz/2) since a line at or
    # above it would fall outside (or alias on) this rfft axis.
    overlay_freqs = []
    if sr_hz and bearing_freqs:
        nyquist = sr_hz / 2.0
        freq_colors = {'FTF': 'tab:green', 'BPFO': 'tab:red', 'BPFI': 'tab:purple',
                        'BSF': 'tab:orange', 'shaft_hz': 'tab:gray'}
        for freq_name in ('FTF', 'BPFO', 'BPFI', 'BSF', 'shaft_hz'):
            freq_value = bearing_freqs.get(freq_name)
            if freq_value is not None and 0 < freq_value < nyquist:
                overlay_freqs.append((freq_name, freq_value, freq_colors[freq_name]))

    for ax, label in zip(axs, ['X', 'Y', 'Z']):
        envelope = envelope_per_axis[label]
        envelope_ac = envelope - envelope.mean()

        freqs = stats[label]['freqs']
        mag = np.abs(np.fft.rfft(envelope_ac)) / n * 2
        dominant_idx = np.argmax(mag[1:]) + 1  # skip DC bin
        dominant_envelope_freq = freqs[dominant_idx]
        envelope_fft_summary[label] = dominant_envelope_freq

        ax.plot(freqs, mag, linewidth=0.8)
        for freq_name, freq_value, color in overlay_freqs:
            ax.axvline(freq_value, color=color, linestyle='--', linewidth=1.2,
                       label=f'{freq_name} {freq_value:.1f} Hz')
        if overlay_freqs:
            ax.legend(fontsize=7, loc='upper right')
        ax.set_ylabel(f'{label} Envelope FFT Peak Amplitude (g)')
        ax.grid(alpha=0.2)
        ax.spines['top'].set_visible(False)
        ax.spines['right'].set_visible(False)
        xunit = 'Hz' if sr_hz else 'cycles/1024-sample-block'
        ax.set_title(f'{label}: dominant~{dominant_envelope_freq:.1f}{xunit}', fontsize=9, loc='left')
    axs[-1].set_xlabel(f"Frequency ({'Hz' if sr_hz else 'cycles per 1024-sample block -- pass --sr-hz for real Hz'})")
    # Same convention as the whole-capture FFT above -- see FFT_CONVENTION;
    # only the input signal differs (envelope, DC-removed after Hilbert,
    # instead of the raw acceleration waveform).
    fig.suptitle(f"FIFO-derived Envelope FFT — tag='{cap['tag']}'\n{FFT_CONVENTION}", fontsize=10)
    plt.tight_layout()
    annotate_capture_metadata(fig, metadata_line)
    envelope_fft_path = f"{out_prefix}_{cap['tag']}_envelope_fft.png"
    plt.savefig(envelope_fft_path, dpi=150)
    plt.close()
    print(f"  saved: {envelope_fft_path}")

    # --- Gravity-vector magnitude check (RFC-0006 §1, §4 Experiment 1) ---
    # Now meaningful for both formats: both are converted to g via the
    # same manual formula (previously this was skipped for Phase 7A
    # because no g-conversion existed for it yet -- see module docstring).
    dc_vec_g = np.array([stats['X']['dc_g'], stats['Y']['dc_g'], stats['Z']['dc_g']])
    mag_g = np.linalg.norm(dc_vec_g)
    print(f"  DC vector magnitude |X,Y,Z| = {mag_g:.4f} g  "
          f"({'consistent with gravity' if 0.9 <= mag_g <= 1.1 else 'DOES NOT look like gravity -- re-examine hypothesis'})")

    print("  --- Compare these numbers against the SAME-moment register readout ---")
    for label in ['X', 'Y', 'Z']:
        s = stats[label]
        print(f"  {label}: DC={s['dc_g']:.4f}g ({s['dc_counts']:.1f} counts)   "
              f"RMS(AC)={s['rms_g']:.4f}g ({s['rms_counts']:.1f} counts, {s['rms_ms2']:.4f} m/s^2)   "
              f"Peak(AC)={s['peak_g']:.4f}g ({s['peak_counts']:.1f} counts, {s['peak_ms2']:.4f} m/s^2)   "
              f"dominant_freq~{s['dominant_freq']:.1f}"
              f"{'Hz' if sr_hz else ' (cycles/block, pass --sr-hz)'}"
              f"   <-- compare to register RRAX/VRMSX-type & HZX for this axis")

    # --- Structured summary row (Phase 7B-1) -- reports the same numbers
    # already computed above; does not alter the analysis itself. Dominant
    # frequency is left blank when --sr-hz wasn't given, since without it
    # the value above is in cycles/block, not Hz (same "don't guess" rule
    # the module docstring already applies to the frequency axis).
    row = {
        'tag': cap['tag'],
        'input_format': cap.get('format', 'legacy'),
        'sample_count': n,
        'sample_rate_hz': sr_hz if sr_hz else '',
        'rms_x': stats['X']['rms_g'], 'rms_y': stats['Y']['rms_g'], 'rms_z': stats['Z']['rms_g'],
        'peak_x': stats['X']['peak_g'], 'peak_y': stats['Y']['peak_g'], 'peak_z': stats['Z']['peak_g'],
        'dominant_freq_x_hz': stats['X']['dominant_freq'] if sr_hz else '',
        'dominant_freq_y_hz': stats['Y']['dominant_freq'] if sr_hz else '',
        'dominant_freq_z_hz': stats['Z']['dominant_freq'] if sr_hz else '',
        'waveform_png': ts_path,
        'fft_png': fft_path,
        'spectrogram_png': spectrogram_path,
        'spectrogram_window': nperseg,
        'spectrogram_overlap': noverlap / nperseg,
        'envelope_png': envelope_path,
        'envelope_method': 'hilbert_raw',
        'dominant_envelope_freq_x_hz': envelope_fft_summary['X'] if sr_hz else '',
        'dominant_envelope_freq_y_hz': envelope_fft_summary['Y'] if sr_hz else '',
        'dominant_envelope_freq_z_hz': envelope_fft_summary['Z'] if sr_hz else '',
        'envelope_fft_png': envelope_fft_path,
        'bearing_model': bearing_model if bearing_model else '',
        'rolling_elements': bearing['rolling_elements'] if bearing else '',
        'ball_diameter_mm': bearing['ball_diameter_mm'] if bearing else '',
        'pitch_diameter_mm': bearing['pitch_diameter_mm'] if bearing else '',
        'contact_angle_deg': bearing['contact_angle_deg'] if bearing else '',
        'dc_offset_counts_x': stats['X']['dc_counts'], 'dc_offset_counts_y': stats['Y']['dc_counts'],
        'dc_offset_counts_z': stats['Z']['dc_counts'],
        'dc_offset_g_x': stats['X']['dc_g'], 'dc_offset_g_y': stats['Y']['dc_g'], 'dc_offset_g_z': stats['Z']['dc_g'],
        'rms_ac_counts_x': stats['X']['rms_counts'], 'rms_ac_counts_y': stats['Y']['rms_counts'],
        'rms_ac_counts_z': stats['Z']['rms_counts'],
        'rms_ac_ms2_x': stats['X']['rms_ms2'], 'rms_ac_ms2_y': stats['Y']['rms_ms2'], 'rms_ac_ms2_z': stats['Z']['rms_ms2'],
        'peak_ac_counts_x': stats['X']['peak_counts'], 'peak_ac_counts_y': stats['Y']['peak_counts'],
        'peak_ac_counts_z': stats['Z']['peak_counts'],
        'peak_ac_ms2_x': stats['X']['peak_ms2'], 'peak_ac_ms2_y': stats['Y']['peak_ms2'], 'peak_ac_ms2_z': stats['Z']['peak_ms2'],
        'fft_resolution_hz': fft_resolution_hz if fft_resolution_hz else '',
    }

    return stats, row


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('logfile', help='Serial monitor log (or plain CSV) containing FIFO capture(s)')
    ap.add_argument('--sr-hz', type=float, default=None,
                     help='True sample rate in Hz for correct FFT frequency-axis labeling '
                          '(look up SR index -> Hz from the firmware SAMPLE_RATE_HZ table)')
    ap.add_argument('--out-prefix', default='fifo_analysis', help='Output filename prefix for plots')
    ap.add_argument('--manufacturer', default=None,
                     help='Bearing manufacturer, e.g. SKF (Phase 7C-1; must be given together '
                          f'with --bearing, no fault-frequency calculation yet). '
                          f'Looked up in {BEARING_DB_PATH}')
    ap.add_argument('--bearing', default=None,
                     help='Bearing model, e.g. 6205 (Phase 7C-1; must be given together '
                          'with --manufacturer)')
    ap.add_argument('--shaft-rpm', type=float, default=None,
                     help='Shaft speed in RPM (Phase 7C-2). Together with a selected bearing, '
                          'used to compute shaft_hz/FTF/BPFO/BPFI/BSF for the summary CSV. '
                          'Never guessed -- omitted or <=0 leaves those columns blank.')
    args = ap.parse_args()

    if bool(args.manufacturer) != bool(args.bearing):
        print("Both --manufacturer and --bearing are required together "
              "(e.g. --manufacturer SKF --bearing 6205).")
        sys.exit(1)

    bearing = None
    bearing_model = None
    if args.manufacturer and args.bearing:
        # manufacturer -> bearing model -> JSON lookup -> bearing geometry
        bearing_db = load_bearing_database(BEARING_DB_PATH)
        manufacturer_db = bearing_db.get(args.manufacturer)
        if manufacturer_db is None:
            print(f"Unknown manufacturer '{args.manufacturer}'. "
                  f"Known manufacturers: {', '.join(sorted(bearing_db))}")
            sys.exit(1)
        bearing = manufacturer_db.get(args.bearing)
        if bearing is None:
            print(f"Unknown bearing model '{args.bearing}' for manufacturer '{args.manufacturer}'. "
                  f"Known models: {', '.join(sorted(manufacturer_db))}")
            sys.exit(1)
        bearing_model = f"{args.manufacturer}{args.bearing}"
        print("Bearing:")
        print(f"  Model: {args.manufacturer} {args.bearing}")
        print(f"  Z: {bearing['rolling_elements']}")
        print(f"  Bd: {bearing['ball_diameter_mm']} mm")
        print(f"  Pd: {bearing['pitch_diameter_mm']} mm")
        print(f"  Angle: {bearing['contact_angle_deg']} deg")

    # Bearing characteristic frequencies, Phase 7C-2 -- pure calculation
    # only, no plotting. Left as all-None (-> blank CSV columns) unless a
    # bearing was selected AND a valid --shaft-rpm was given; RPM is never
    # guessed.
    bearing_freqs = {'shaft_hz': None, 'FTF': None, 'BPFO': None, 'BPFI': None, 'BSF': None}
    if bearing is not None:
        bearing_freqs = compute_bearing_frequencies(
            shaft_rpm=args.shaft_rpm,
            rolling_elements=bearing['rolling_elements'],
            ball_diameter_mm=bearing['ball_diameter_mm'],
            pitch_diameter_mm=bearing['pitch_diameter_mm'],
            contact_angle_deg=bearing['contact_angle_deg'],
        )
        if bearing_freqs['shaft_hz'] is not None:
            print("Bearing Frequencies:")
            print(f"  shaft_hz: {bearing_freqs['shaft_hz']:.4f} Hz")
            print(f"  FTF:  {bearing_freqs['FTF']:.4f} Hz")
            print(f"  BPFO: {bearing_freqs['BPFO']:.4f} Hz")
            print(f"  BPFI: {bearing_freqs['BPFI']:.4f} Hz")
            print(f"  BSF:  {bearing_freqs['BSF']:.4f} Hz")

    captures = parse_captures(args.logfile)
    if not captures:
        print("No FIFO captures found in this file. Expected either a legacy header line "
              "'FIFOIndex,Tag,SR,X_raw,Y_raw,Z_raw,X_g,Y_g,Z_g' followed by data rows and a "
              "'FIFO_CAPTURE_END' sentinel line, or a Phase 7A header line 'Index,X,Y,Z' "
              "followed by data rows.")
        sys.exit(1)

    print(f"Found {len(captures)} FIFO capture(s) in {args.logfile}")
    summary_rows = []
    for cap in captures:
        _, row = analyze_one(cap, args.sr_hz, args.out_prefix, bearing_model=bearing_model, bearing=bearing,
                              bearing_freqs=bearing_freqs)
        for freq_key in ('shaft_hz', 'FTF', 'BPFO', 'BPFI', 'BSF'):
            row[freq_key] = bearing_freqs[freq_key] if bearing_freqs[freq_key] is not None else ''
        summary_rows.append(row)

    summary_csv_path = f"{args.out_prefix}_summary.csv"
    with open(summary_csv_path, 'w', newline='', encoding='utf-8') as f:
        writer = csv.DictWriter(f, fieldnames=SUMMARY_CSV_FIELDS)
        writer.writeheader()
        writer.writerows(summary_rows)
    print(f"\nSummary CSV saved: {summary_csv_path} ({len(summary_rows)} row(s))")

    print("\nDone. Remember (RFC-0006 discipline): a match here is EVIDENCE, "
          "not proof -- run this across multiple experiments (idle, tap, "
          "running motor, orientation change) before raising confidence.")


if __name__ == '__main__':
    main()
