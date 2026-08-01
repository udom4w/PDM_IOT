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
import re
import csv
import argparse
import numpy as np
from scipy.signal import spectrogram as scipy_spectrogram
from scipy.signal.windows import hann
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

SUMMARY_CSV_FIELDS = [
    'tag', 'input_format', 'sample_count', 'sample_rate_hz',
    'rms_x', 'rms_y', 'rms_z',
    'peak_x', 'peak_y', 'peak_z',
    'dominant_freq_x_hz', 'dominant_freq_y_hz', 'dominant_freq_z_hz',
    'waveform_png', 'fft_png', 'spectrogram_png',
    'spectrogram_window', 'spectrogram_overlap',
]


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


def analyze_one(cap, sr_hz, out_prefix):
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
    }

    return summary, row


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('logfile', help='Serial monitor log (or plain CSV) containing FIFO capture(s)')
    ap.add_argument('--sr-hz', type=float, default=None,
                     help='True sample rate in Hz for correct FFT frequency-axis labeling '
                          '(look up SR index -> Hz from the firmware SAMPLE_RATE_HZ table)')
    ap.add_argument('--out-prefix', default='fifo_analysis', help='Output filename prefix for plots')
    args = ap.parse_args()

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
        _, row = analyze_one(cap, args.sr_hz, args.out_prefix)
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
