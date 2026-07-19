#!/usr/bin/env python3
"""
analyze_fifo_capture.py — RFC-0006 experiment analysis tool

Reads a Serial Monitor log (or a plain CSV) containing one or more `FIFO`
command captures from WTVB05_ValidationTool v3.0, and for each capture:

  1. Plots the full 1024-sample time series per axis (X, Y, Z) in g.
  2. Computes an FFT independently from the raw samples and plots the
     magnitude spectrum per axis.
  3. Computes RMS (mean-removed) and Peak per axis from the raw samples.
  4. Prints a summary table so these FIFO-derived numbers can be compared
     directly against the *register-reported* RRAX/VRMSX/HZX-type values
     from the same test session (RFC-0006 Experiment 3 — the single most
     decisive test proposed in that review).

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
import argparse
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def parse_captures(path):
    """Parse one or more FIFO CSV blocks out of a raw serial log file.
    Returns a list of dicts: {tag, sr, index, x_raw, y_raw, z_raw, x_g, y_g, z_g}
    """
    captures = []
    current = None
    header_re = re.compile(r'^FIFOIndex,Tag,SR,X_raw,Y_raw,Z_raw,X_g,Y_g,Z_g\s*$')

    with open(path, encoding='utf-8', errors='replace') as f:
        for line in f:
            line = line.rstrip('\r\n')
            if header_re.match(line):
                current = {'tag': None, 'sr': None, 'index': [], 'x_raw': [], 'y_raw': [], 'z_raw': [],
                           'x_g': [], 'y_g': [], 'z_g': []}
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


def analyze_one(cap, sr_hz, out_prefix):
    n = len(cap['index'])
    print(f"\n=== Capture tag='{cap['tag']}' SR_index={cap['sr']} samples={n} ===")

    axes = {'X': np.array(cap['x_g']), 'Y': np.array(cap['y_g']), 'Z': np.array(cap['z_g'])}

    # --- Time-series plot, all 1024 points per axis ---
    fig, axs = plt.subplots(3, 1, figsize=(11, 8), sharex=True)
    for ax, (label, data) in zip(axs, axes.items()):
        ax.plot(cap['index'], data, linewidth=0.7)
        ax.set_ylabel(f'{label} (g)')
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
        ax.set_ylabel(f'{label} magnitude')
        ax.grid(alpha=0.2)
        ax.spines['top'].set_visible(False)
        ax.spines['right'].set_visible(False)
        xunit = 'Hz' if sr_hz else 'cycles/1024-sample-block'
        ax.set_title(f'{label}: DC={mean_val:.4f}g  RMS(AC)={rms:.4f}g  Peak(AC)={peak:.4f}g  '
                      f'dominant~{dominant_freq:.1f}{xunit}', fontsize=9, loc='left')
    axs[-1].set_xlabel(f"Frequency ({'Hz' if sr_hz else 'cycles per 1024-sample block -- pass --sr-hz for real Hz'})")
    fig.suptitle(f"FIFO-derived FFT — tag='{cap['tag']}'")
    plt.tight_layout()
    fft_path = f"{out_prefix}_{cap['tag']}_fft.png"
    plt.savefig(fft_path, dpi=150)
    plt.close()
    print(f"  saved: {fft_path}")

    # --- Gravity-vector magnitude check (RFC-0006 §1, §4 Experiment 1) ---
    dc_vec = np.array([summary['X']['dc'], summary['Y']['dc'], summary['Z']['dc']])
    mag_g = np.linalg.norm(dc_vec)
    print(f"  DC vector magnitude |X,Y,Z| = {mag_g:.4f} g  "
          f"({'consistent with gravity' if 0.9 <= mag_g <= 1.1 else 'DOES NOT look like gravity -- re-examine hypothesis'})")

    print("  --- Compare these numbers against the SAME-moment register readout ---")
    for label in ['X', 'Y', 'Z']:
        s = summary[label]
        print(f"  {label}: FIFO-derived RMS={s['rms']:.4f}g  Peak={s['peak']:.4f}g  "
              f"dominant_freq~{s['dominant_freq']:.1f}"
              f"{'Hz' if sr_hz else ' (cycles/block, pass --sr-hz)'}"
              f"   <-- compare to register RRAX/VRMSX-type & HZX for this axis")

    return summary


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
        print("No FIFO captures found in this file. Expected a header line "
              "'FIFOIndex,Tag,SR,X_raw,Y_raw,Z_raw,X_g,Y_g,Z_g' followed by 1024 "
              "data rows and a 'FIFO_CAPTURE_END' sentinel line.")
        sys.exit(1)

    print(f"Found {len(captures)} FIFO capture(s) in {args.logfile}")
    for cap in captures:
        analyze_one(cap, args.sr_hz, args.out_prefix)

    print("\nDone. Remember (RFC-0006 discipline): a match here is EVIDENCE, "
          "not proof -- run this across multiple experiments (idle, tap, "
          "running motor, orientation change) before raising confidence.")


if __name__ == '__main__':
    main()
