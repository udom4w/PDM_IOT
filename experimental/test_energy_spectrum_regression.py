#!/usr/bin/env python3
"""Regression tests for analyze_fifo_dewesoft.py's Energy Spectrum path.

Primary purpose: lock down the V2.2 R1 fix (zero-padding energy
normalization) so that defect cannot silently return. R1 was a ~-29.3%
under-report of Overall RMS for every capture shorter than fft_size, and it
survived two releases because nothing asserted the one invariant the Energy
Spectrum exists to provide -- that its bins sum, in power, back to the
time-domain AC mean-square.

Secondary purpose: prove the fix did NOT disturb the frozen Phase-1
behaviour -- Peak Spectrum, time-domain statistics, and exact-multiple
(non-padded) Energy Spectrum results must all be unchanged.

Run:
    python experimental/test_energy_spectrum_regression.py

Exits 0 if every test passes, 1 otherwise. No pytest dependency (this
project's toolchain is arduino-cli + a plain python analysis script; adding
a test framework dependency for eight assertions is not warranted).
"""

import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import analyze_fifo_dewesoft as E  # noqa: E402

SR = 1000.0
HERE = os.path.dirname(os.path.abspath(__file__))
FIFO_CAPTURE = os.path.join(HERE, 'fifo_data', 'captureId2.csv')

# Tolerance for "Energy Spectrum Overall RMS vs time-domain AC RMS" on REAL
# data. validation_report.md Sec 7.2 documents the Hann/NPG energy recovery
# as approximate (+-0.3-1.6% measured), never bit-exact -- windowing
# permanently tapers block edges, so exact recovery is impossible in
# general. 5% gives headroom over that documented range while still being
# 6x tighter than the -29.3% R1 defect.
REAL_DATA_TOL = 0.05

# Tolerance for a pure bin-aligned synthetic tone, where the analytic answer
# is known exactly and the leakage is confined to +-2 bins.
SYNTHETIC_TOL = 0.005

_failures = []
_passes = 0


def check(name, condition, detail=''):
    global _passes
    if condition:
        _passes += 1
        print(f"  PASS  {name}")
        if detail:
            print(f"        {detail}")
    else:
        _failures.append(name)
        print(f"  FAIL  {name}")
        if detail:
            print(f"        {detail}")


def rel(a, b):
    return abs(a - b) / abs(b) if b else float('inf')


def energy_overall_rms(signal, fft_size):
    """Overall RMS via the exact path the CLI uses (DC removal -> energy
    spectrum -> Parseval sum), so these tests exercise production code."""
    stats = E.compute_time_domain_stats(signal)
    _, _, ms_avg, _ = E.compute_energy_spectrum(stats['ac'], fft_size, SR)
    return E.compute_overall_rms(ms_avg), stats['ac_rms']


def sine(amplitude, freq_hz, n_samples):
    n = np.arange(n_samples)
    return amplitude * np.sin(2.0 * np.pi * freq_hz * n / SR)


# ==========================================================================
print("\n[A] Real capture captureId2.csv, SR=1000, fft_size=2048 (N=1024,")
print("    zero-padded to half -- the exact R1 failure configuration)")
# ==========================================================================
if not os.path.exists(FIFO_CAPTURE):
    check('A: capture file present', False, f'missing: {FIFO_CAPTURE}')
else:
    loaded = E.read_fifo_csv(FIFO_CAPTURE, SR)
    for label in E.AXIS_LABELS:
        got, td = energy_overall_rms(loaded[label], 2048)
        check(f'A: axis {label} energy RMS ~= time-domain AC RMS',
              rel(got, td) < REAL_DATA_TOL,
              f'energy={got:.6f}  time-domain={td:.6f}  delta={100 * (got / td - 1):+.3f}%')

# ==========================================================================
print("\n[B] Exact-multiple, N_real == fft_size -- the fix must change NOTHING")
# ==========================================================================
# Bit-for-bit check against the pre-fix formula, recomputed inline here.
# For a full block the fix reduces to the original NPG*fft_size**2, so the
# two must agree to the LAST BIT, not merely to a tolerance.
rng = np.random.default_rng(20260809)
for fft_size in (1024, 2048):
    sig = rng.standard_normal(fft_size)
    ac = sig - sig.mean()
    _, _, ms_new, _ = E.compute_energy_spectrum(ac, fft_size, SR)

    window = E.hann_window(fft_size)
    npg = E.noise_power_gain(window)
    spectrum = np.fft.rfft(ac * window)
    ms_old = (np.abs(spectrum) ** 2) / (npg * fft_size ** 2)
    ms_old = ms_old * 2.0
    ms_old[0] /= 2.0
    if fft_size % 2 == 0:
        ms_old[-1] /= 2.0

    check(f'B: fft_size={fft_size}, N==fft_size -- bit-for-bit vs pre-fix formula',
          np.array_equal(ms_new, ms_old),
          f'max abs diff = {np.max(np.abs(ms_new - ms_old)):.3e} (must be exactly 0.0)')

# ==========================================================================
print("\n[C] Multiple full blocks, N_real = 2*fft_size and 4*fft_size")
# ==========================================================================
FFT = 2048
tone = 102 * (SR / FFT)  # bin-aligned -> no scalloping
expected = 1.0 / np.sqrt(2.0)
results = {}
for mult in (1, 2, 4):
    got, td = energy_overall_rms(sine(1.0, tone, mult * FFT), FFT)
    results[mult] = got
    check(f'C: {mult}x full block -- RMS matches analytic A/sqrt(2)',
          rel(got, expected) < SYNTHETIC_TOL,
          f'got={got:.8f}  expected={expected:.8f}  delta={100 * (got / expected - 1):+.5f}%')
spread = (max(results.values()) - min(results.values())) / min(results.values())
check('C: stability across 1x/2x/4x full blocks (spread < 0.1%)',
      spread < 0.001, f'spread = {100 * spread:.6f}%')

# ==========================================================================
print("\n[D] Zero-padded single block, N_real = fft_size/2 -- THE R1 CASE")
# ==========================================================================
# Pre-fix this returned exactly sqrt(1/2) = 0.7071x the true RMS (-29.3%).
half = sine(1.0, tone, FFT // 2)
got, td = energy_overall_rms(half, FFT)
check('D: N=fft_size/2 is NOT diluted by sqrt(1/2)',
      rel(got, expected) < SYNTHETIC_TOL,
      f'got={got:.8f}  expected={expected:.8f}  delta={100 * (got / expected - 1):+.5f}%')
check('D: and is demonstrably not the pre-fix value',
      rel(got, expected * np.sqrt(0.5)) > 0.20,
      f'pre-fix would have been {expected * np.sqrt(0.5):.8f}')

# Non-half padding ratios too -- these are where the "NPG*fft_size*n_real"
# alternative would have been wrong (it is exact only at n_real=M and M/2).
for n_real in (1536, 1994, 1024 + 7):
    got, td = energy_overall_rms(sine(1.0, tone, n_real), FFT)
    check(f'D: n_real={n_real} (padding ratio {n_real / FFT:.3f}) within 2% of analytic',
          rel(got, expected) < 0.02,
          f'got={got:.8f}  delta={100 * (got / expected - 1):+.4f}%')

# ==========================================================================
print("\n[E] Pure sine with known analytic RMS, several amplitudes/frequencies")
# ==========================================================================
for amp, k in ((1.0, 102), (9.8, 60), (0.25, 205), (2.5, 410)):
    f_hz = k * (SR / FFT)
    exp_rms = amp / np.sqrt(2.0)
    got, td = energy_overall_rms(sine(amp, f_hz, 4 * FFT), FFT)
    check(f'E: A={amp} f={f_hz:.4f}Hz -> RMS = A/sqrt(2)',
          rel(got, exp_rms) < SYNTHETIC_TOL,
          f'got={got:.8f}  expected={exp_rms:.8f}  delta={100 * (got / exp_rms - 1):+.5f}%')

# ==========================================================================
print("\n[F] Peak Spectrum must be untouched by this fix")
# ==========================================================================
# compute_fft_spectrum() was not edited; assert it still produces the
# textbook peak amplitude for a bin-aligned tone, in BOTH the full-block and
# the zero-padded case (the padded case is expected to still read low -- Peak
# Spectrum has never claimed energy consistency, and R1 deliberately did not
# change it).
sig_full = sine(1.0, tone, 4 * FFT)
_, peak_spec, _ = E.compute_fft_spectrum(sig_full - sig_full.mean(), FFT, SR)
check('F: Peak Spectrum reads true 0-to-peak amplitude for a bin-aligned tone',
      rel(float(np.max(peak_spec)), 1.0) < SYNTHETIC_TOL,
      f'peak={float(np.max(peak_spec)):.8f}  expected=1.0')

if os.path.exists(FIFO_CAPTURE):
    loaded = E.read_fifo_csv(FIFO_CAPTURE, SR)
    ac = E.compute_time_domain_stats(loaded['X'])['ac']
    _, ps, nb = E.compute_fft_spectrum(ac, 2048, SR)
    check('F: Peak Spectrum on real capture is finite and unchanged in shape',
          np.all(np.isfinite(ps)) and len(ps) == 2048 // 2 + 1 and nb == 1,
          f'bins={len(ps)}  blocks={nb}  max={float(np.max(ps)):.6f}')

# ==========================================================================
print("\n[G] Time-domain metrics must be untouched by this fix")
# ==========================================================================
# compute_time_domain_stats() was not edited. Verify every reported metric
# against an independent inline computation.
if os.path.exists(FIFO_CAPTURE):
    loaded = E.read_fifo_csv(FIFO_CAPTURE, SR)
    for label in E.AXIS_LABELS:
        x = np.asarray(loaded[label], dtype=float)
        s = E.compute_time_domain_stats(x)
        a = x - x.mean()
        sd = np.std(a)
        want = {
            'dc': x.mean(),
            'ac_rms': np.sqrt(np.mean(a ** 2)),
            'peak': np.max(np.abs(a)),
            'peak_to_peak': np.max(a) - np.min(a),
            'std_dev': sd,
            'crest_factor': np.max(np.abs(a)) / np.sqrt(np.mean(a ** 2)),
            'kurtosis': np.mean(a ** 4) / sd ** 4 - 3.0,
            'skewness': np.mean(a ** 3) / sd ** 3,
        }
        bad = [k for k, v in want.items() if not np.isclose(s[k], v, rtol=0, atol=0)]
        check(f'G: axis {label} -- all 8 time-domain metrics bit-exact',
              not bad,
              'DC={dc:.6f} ACRMS={ac_rms:.6f} Peak={peak:.6f} P2P={peak_to_peak:.6f} '
              'Std={std_dev:.6f} CF={crest_factor:.4f} Kurt={kurtosis:.4f} '
              'Skew={skewness:.4f}'.format(**want)
              + (f'  MISMATCH: {bad}' if bad else ''))

# ==========================================================================
print("\n[Guard] Defensive checks -- these are what stop R1 returning silently")
# ==========================================================================
# The guard must FIRE on a deliberately broken (heavily diluted) spectrum...
import io                                                  # noqa: E402
import contextlib                                          # noqa: E402

buf = io.StringIO()
with contextlib.redirect_stderr(buf):
    fake_ac = sine(1.0, tone, FFT)
    _, _, ms_ok, _ = E.compute_energy_spectrum(fake_ac, FFT, SR)
    E._warn_if_not_parseval_consistent(fake_ac, ms_ok * 0.5, FFT)  # simulate R1
check('Guard: fires on a 29% energy dilution',
      'not Parseval-consistent' in buf.getvalue(),
      buf.getvalue().strip()[:150] or '(no output)')

# ...and must stay SILENT on every legitimate configuration.
buf = io.StringIO()
with contextlib.redirect_stderr(buf):
    for fs, n in ((FFT, 4 * FFT), (FFT, FFT), (FFT, FFT // 2), (1024, 1024)):
        E.compute_energy_spectrum(sine(1.0, 102 * (SR / FFT), n), fs, SR)
    if os.path.exists(FIFO_CAPTURE):
        loaded = E.read_fifo_csv(FIFO_CAPTURE, SR)
        for label in E.AXIS_LABELS:
            E.compute_energy_spectrum(E.compute_time_domain_stats(loaded[label])['ac'], 2048, SR)
check('Guard: silent on all legitimate configurations (no false positives)',
      buf.getvalue().strip() == '',
      buf.getvalue().strip()[:300] or '(silent, as required)')

# No NaN/Inf may ever reach the caller, including the w[0]==0.0 edge case.
edge_ok = True
edge_detail = []
for n, fs in ((2, 2048), (3, 2048), (2049, 2048), (4097, 2048)):
    try:
        _, rms_s, ms_a, nb = E.compute_energy_spectrum(sine(1.0, tone, n), fs, SR)
        finite = np.all(np.isfinite(ms_a)) and np.all(np.isfinite(rms_s))
        edge_ok &= finite
        edge_detail.append(f'N={n}/fft={fs}: blocks={nb} finite={finite}')
    except Exception as exc:  # noqa: BLE001
        edge_ok = False
        edge_detail.append(f'N={n}/fft={fs}: RAISED {type(exc).__name__}: {exc}')
check('Guard: no NaN/Inf for tiny and 1-real-sample-trailing-block inputs',
      edge_ok, '; '.join(edge_detail))

# ==========================================================================
print("\n" + "=" * 70)
if _failures:
    print(f"FAILED: {len(_failures)} of {_passes + len(_failures)} checks")
    for name in _failures:
        print(f"  - {name}")
    sys.exit(1)
print(f"ALL {_passes} CHECKS PASSED")
sys.exit(0)
