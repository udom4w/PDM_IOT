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
    (X/Y/Z are raw ADC counts, NOT g -- no Tag/SR fields, no g-conversion
    available. A block runs from its header line to the next header line
    or end of file. Time waveform / FFT axes are labeled "Raw Counts" and
    the gravity-vector sanity check, which assumes g-scaled DC values, is
    skipped for this format.)

  1. Plots the full 1024-sample time series per axis (X, Y, Z) in g
     (legacy format) or raw ADC counts (Phase 7A format).
  2. Computes an FFT independently from the raw samples and plots the
     magnitude spectrum per axis.
  3. Computes RMS (mean-removed) and Peak per axis from the raw samples.
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
  7. Plots the envelope of the mean-removed waveform per axis via
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
    'rms_x', 'rms_y', 'rms_z',
    'peak_x', 'peak_y', 'peak_z',
    'dominant_freq_x_hz', 'dominant_freq_y_hz', 'dominant_freq_z_hz',
    'waveform_png', 'fft_png', 'spectrogram_png',
    'spectrogram_window', 'spectrogram_overlap', 'envelope_png', 'envelope_method',
    'dominant_envelope_freq_x_hz', 'dominant_envelope_freq_y_hz', 'dominant_envelope_freq_z_hz',
    'envelope_fft_png',
    'bearing_model', 'rolling_elements', 'ball_diameter_mm', 'pitch_diameter_mm', 'contact_angle_deg',
    'shaft_hz', 'FTF', 'BPFO', 'BPFI', 'BSF',
]

# External bearing geometry database, Phase 7C-1 (revised architecture --
# replaces the earlier hard-coded BEARING_DB dict). Structure:
# {"<manufacturer>": {"<model>": {rolling_elements, ball_diameter_mm,
# pitch_diameter_mm, contact_angle_deg}}}. No fault-frequency math is done
# with these values yet, that's a later phase.
BEARING_DB_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'bearing_database', 'bearing_db.json')

BEARING_DB_REQUIRED_KEYS = {'rolling_elements', 'ball_diameter_mm', 'pitch_diameter_mm', 'contact_angle_deg'}


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
    X/Y/Z are the raw counts, full stop. There is no g-conversion available,
    so x_g/y_g/z_g are left empty; downstream code must not treat this
    format as g-scaled. This format also carries no Tag/SR fields, so those
    are synthesized (tag = 'phase7a_N' per block, sr = None). A block runs
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


def analyze_one(cap, sr_hz, out_prefix, bearing_model=None, bearing=None):
    n = len(cap['index'])
    is_phase7a = cap.get('format') == 'phase7a'
    unit_label = 'Raw Counts' if is_phase7a else 'g'
    unit_suffix = ' counts' if is_phase7a else 'g'
    num_fmt = '.1f' if is_phase7a else '.4f'
    print(f"\n=== Capture tag='{cap['tag']}' SR_index={cap['sr']} samples={n} ===")

    if is_phase7a:
        axes = {'X': np.array(cap['x_raw']), 'Y': np.array(cap['y_raw']), 'Z': np.array(cap['z_raw'])}
    else:
        axes = {'X': np.array(cap['x_g']), 'Y': np.array(cap['y_g']), 'Z': np.array(cap['z_g'])}

    # --- Time-series plot, all 1024 points per axis ---
    fig, axs = plt.subplots(3, 1, figsize=(11, 8), sharex=True)
    for ax, (label, data) in zip(axs, axes.items()):
        ax.plot(cap['index'], data, linewidth=0.7)
        ax.set_ylabel(f'{label} ({unit_label})')
        ax.grid(alpha=0.2)
        ax.spines['top'].set_visible(False)
        ax.spines['right'].set_visible(False)
    axs[-1].set_xlabel('Sample index (0-1023 within this FIFO block)')
    fig.suptitle(f"FIFO raw waveform — tag='{cap['tag']}' (all {n} samples)")
    plt.tight_layout()
    ts_path = f"{out_prefix}_{cap['tag']}_timeseries.png"
    plt.savefig(ts_path, dpi=150)
    plt.close()
    print(f"  saved: {ts_path}")

    # --- FFT + RMS/Peak per axis ---
    fig, axs = plt.subplots(3, 1, figsize=(11, 8))
    summary = {}
    for ax, (label, data) in zip(axs, axes.items()):
        mean_val = data.mean()
        ac = data - mean_val   # mean-removed, for RMS/FFT (DC handled separately)
        rms = np.sqrt(np.mean(ac ** 2))
        peak = np.max(np.abs(ac))

        freqs = np.fft.rfftfreq(n, d=1.0 / sr_hz) if sr_hz else np.fft.rfftfreq(n, d=1.0)
        mag = np.abs(np.fft.rfft(ac)) / n * 2
        dominant_idx = np.argmax(mag[1:]) + 1  # skip DC bin
        dominant_freq = freqs[dominant_idx]

        summary[label] = dict(dc=mean_val, rms=rms, peak=peak, dominant_freq=dominant_freq)

        ax.plot(freqs, mag, linewidth=0.8)
        ax.set_ylabel(f'{label} magnitude ({unit_label})' if is_phase7a else f'{label} magnitude')
        ax.grid(alpha=0.2)
        ax.spines['top'].set_visible(False)
        ax.spines['right'].set_visible(False)
        xunit = 'Hz' if sr_hz else 'cycles/1024-sample-block'
        ax.set_title(f'{label}: DC={mean_val:{num_fmt}}{unit_suffix}  RMS(AC)={rms:{num_fmt}}{unit_suffix}  '
                      f'Peak(AC)={peak:{num_fmt}}{unit_suffix}  dominant~{dominant_freq:.1f}{xunit}',
                      fontsize=9, loc='left')
    axs[-1].set_xlabel(f"Frequency ({'Hz' if sr_hz else 'cycles per 1024-sample block -- pass --sr-hz for real Hz'})")
    fig.suptitle(f"FIFO-derived FFT — tag='{cap['tag']}'")
    plt.tight_layout()
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
    for label, data in axes.items():
        ac = data - data.mean()   # mean-removed so DC doesn't swamp the color scale
        f_spec, t_spec, Sxx = scipy_spectrogram(ac, fs=fs, window=window, nperseg=nperseg,
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
    fig.suptitle(f"FIFO-derived Spectrogram (STFT, Hann window={nperseg}, 50% overlap) — tag='{cap['tag']}'")
    fig.colorbar(mesh, ax=axs, label=f'Magnitude ({unit_label})')
    spectrogram_path = f"{out_prefix}_{cap['tag']}_spectrogram.png"
    plt.savefig(spectrogram_path, dpi=150)
    plt.close()
    print(f"  saved: {spectrogram_path}")

    # --- Envelope (Hilbert transform) per axis, Phase 7B-3 -- uses the
    # mean-removed waveform directly, no band-pass filter yet (that's a
    # later phase). Independent of FFT/spectrogram; does not read or
    # modify their results.
    envelope_x = np.array(cap['index']) / sr_hz if sr_hz else cap['index']
    envelope_per_axis = {}
    fig, axs = plt.subplots(3, 1, figsize=(11, 8), sharex=True)
    for ax, (label, data) in zip(axs, axes.items()):
        ac = data - data.mean()
        envelope = np.abs(hilbert(ac))
        envelope_per_axis[label] = envelope
        ax.plot(envelope_x, envelope, linewidth=0.8)
        ax.set_ylabel(f'{label} Envelope ({unit_label})')
        ax.grid(alpha=0.2)
        ax.spines['top'].set_visible(False)
        ax.spines['right'].set_visible(False)
    axs[-1].set_xlabel('Time (s)' if sr_hz else 'Sample index (0-1023 within this FIFO block)')
    fig.suptitle(f"FIFO-derived Envelope (Hilbert) — tag='{cap['tag']}' (all {n} samples)")
    plt.tight_layout()
    envelope_path = f"{out_prefix}_{cap['tag']}_envelope.png"
    plt.savefig(envelope_path, dpi=150)
    plt.close()
    print(f"  saved: {envelope_path}")

    # --- FFT of the envelope per axis, Phase 7B-4 -- reuses the exact
    # envelope arrays computed above (abs(hilbert(mean-removed waveform)));
    # does not recompute the envelope differently. The envelope itself is a
    # positive-only signal with a nonzero mean, so its own DC is removed
    # here (envelope_ac = envelope - envelope.mean()) before the FFT --
    # otherwise that DC bin would dominate and hide the modulation
    # frequencies we're looking for. Same frequency-axis/peak-picking logic
    # as the waveform FFT above; no window function in this phase.
    fig, axs = plt.subplots(3, 1, figsize=(11, 8))
    envelope_fft_summary = {}
    for ax, label in zip(axs, ['X', 'Y', 'Z']):
        envelope = envelope_per_axis[label]
        envelope_ac = envelope - envelope.mean()

        freqs = np.fft.rfftfreq(n, d=1.0 / sr_hz) if sr_hz else np.fft.rfftfreq(n, d=1.0)
        mag = np.abs(np.fft.rfft(envelope_ac)) / n * 2
        dominant_idx = np.argmax(mag[1:]) + 1  # skip DC bin
        dominant_envelope_freq = freqs[dominant_idx]
        envelope_fft_summary[label] = dominant_envelope_freq

        ax.plot(freqs, mag, linewidth=0.8)
        ax.set_ylabel(f'{label} Envelope FFT ({unit_label})' if is_phase7a else f'{label} Envelope FFT')
        ax.grid(alpha=0.2)
        ax.spines['top'].set_visible(False)
        ax.spines['right'].set_visible(False)
        xunit = 'Hz' if sr_hz else 'cycles/1024-sample-block'
        ax.set_title(f'{label}: dominant~{dominant_envelope_freq:.1f}{xunit}', fontsize=9, loc='left')
    axs[-1].set_xlabel(f"Frequency ({'Hz' if sr_hz else 'cycles per 1024-sample block -- pass --sr-hz for real Hz'})")
    fig.suptitle(f"FIFO-derived Envelope FFT — tag='{cap['tag']}'")
    plt.tight_layout()
    envelope_fft_path = f"{out_prefix}_{cap['tag']}_envelope_fft.png"
    plt.savefig(envelope_fft_path, dpi=150)
    plt.close()
    print(f"  saved: {envelope_fft_path}")

    # --- Gravity-vector magnitude check (RFC-0006 §1, §4 Experiment 1) ---
    # Only meaningful for g-scaled DC values -- raw ADC counts have no fixed
    # reference magnitude, so this is skipped for Phase 7A input.
    if is_phase7a:
        print("  (Phase 7A raw-count input -- skipping gravity-vector interpretation, "
              "which requires g-scaled DC values)")
    else:
        dc_vec = np.array([summary['X']['dc'], summary['Y']['dc'], summary['Z']['dc']])
        mag_g = np.linalg.norm(dc_vec)
        print(f"  DC vector magnitude |X,Y,Z| = {mag_g:.4f} g  "
              f"({'consistent with gravity' if 0.9 <= mag_g <= 1.1 else 'DOES NOT look like gravity -- re-examine hypothesis'})")

    print("  --- Compare these numbers against the SAME-moment register readout ---")
    for label in ['X', 'Y', 'Z']:
        s = summary[label]
        print(f"  {label}: FIFO-derived RMS={s['rms']:{num_fmt}}{unit_suffix}  Peak={s['peak']:{num_fmt}}{unit_suffix}  "
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
        'rms_x': summary['X']['rms'], 'rms_y': summary['Y']['rms'], 'rms_z': summary['Z']['rms'],
        'peak_x': summary['X']['peak'], 'peak_y': summary['Y']['peak'], 'peak_z': summary['Z']['peak'],
        'dominant_freq_x_hz': summary['X']['dominant_freq'] if sr_hz else '',
        'dominant_freq_y_hz': summary['Y']['dominant_freq'] if sr_hz else '',
        'dominant_freq_z_hz': summary['Z']['dominant_freq'] if sr_hz else '',
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
    }

    return summary, row


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
        _, row = analyze_one(cap, args.sr_hz, args.out_prefix, bearing_model=bearing_model, bearing=bearing)
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
