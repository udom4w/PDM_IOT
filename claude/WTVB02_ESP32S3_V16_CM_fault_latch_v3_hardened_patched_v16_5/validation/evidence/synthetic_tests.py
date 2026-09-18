#!/usr/bin/env python3
# ==============================================================================
# PHASE 8 -- SANITY TESTS against the INDEPENDENT REFERENCE implementation
# (reference_vrms.py). These tests validate the RECONSTRUCTED MATHEMATICAL
# SPECIFICATION's internal self-consistency using known, hand-computable
# synthetic inputs. They do NOT compare against firmware output (no real
# raw FIFO capture was available -- see PHASE 5 / final report). A pass here
# means "the formula transcribed from source behaves the way the formula
# itself predicts it should"; it increases confidence that the source trace
# in source_trace.md was read correctly, but is NOT a firmware numerical
# validation on its own.
# ==============================================================================

import numpy as np
from reference_vrms import (
    axis_velocity_rms, velocity_rms_triaxial, accel_rms_triaxial,
    VIB_ACCEL_LSB_PER_G, VIB_ACCEL_G_MS2, VIB_VEL_FFT_N, VIB_VEL_HP_BIN,
)

SR_HZ = 2000.0
N = VIB_VEL_FFT_N
results = []


def check(name, cond, detail):
    status = "PASS" if cond else "FAIL"
    results.append((name, status, detail))
    print(f"[{status}] {name}: {detail}")


def make_sine_counts(freq_hz, amp_ms2, sr=SR_HZ, n=N):
    """Synthetic int16 RAW counts for a pure sinusoidal acceleration of
    amplitude amp_ms2 (m/s^2) at freq_hz, converted back through the SAME
    LSB_PER_G/G_MS2 scale the firmware uses on the way in, so the round trip
    is honest (this does not smuggle in a calibration factor -- it inverts
    the documented forward conversion to build a KNOWN input, exactly as a
    reference signal generator must)."""
    t = np.arange(n) / sr
    a = amp_ms2 * np.sin(2 * np.pi * freq_hz * t)
    counts = a * (VIB_ACCEL_LSB_PER_G / VIB_ACCEL_G_MS2)
    counts = np.round(counts).astype(np.int16)
    return counts


# ---- Test 1: pure sine at a known frequency, known amplitude ---------------
# Analytic velocity RMS for a[t] = A*sin(2*pi*f*t) is v[t] = -(A/omega)*cos(...),
# whose RMS is (A/omega)/sqrt(2). Converted to mm/s: *1000.
f0 = 100.0  # Hz, comfortably above the HP floor (15.625 Hz) and below Nyquist (1000 Hz)
A0 = 5.0    # m/s^2 amplitude
sine_counts = make_sine_counts(f0, A0)
res = axis_velocity_rms(sine_counts, SR_HZ)
omega0 = 2 * np.pi * f0
expected_rms_mms = (A0 / omega0) / np.sqrt(2) * 1000.0
rel_err = abs(res["rms_mms"] - expected_rms_mms) / expected_rms_mms * 100
check("1_pure_sine_known_freq",
      rel_err < 5.0,  # generous tolerance: Hann leakage + int16 quantization
      f"f={f0}Hz A={A0}m/s^2 -> expected~{expected_rms_mms:.4f}mm/s, "
      f"got={res['rms_mms']:.4f}mm/s, rel_err={rel_err:.3f}%, "
      f"dom_freq={res['dominant_frequency_hz']:.3f}Hz")
check("1b_dominant_frequency_matches_known_tone",
      abs(res["dominant_frequency_hz"] - f0) < 0.5,
      f"expected {f0}Hz, got {res['dominant_frequency_hz']:.4f}Hz "
      f"(bin resolution={SR_HZ/N:.6f}Hz)")

# ---- Test 2: zero signal ----------------------------------------------------
zero_counts = np.zeros(N, dtype=np.int16)
res_zero = axis_velocity_rms(zero_counts, SR_HZ)
check("2_zero_signal_gives_zero_rms",
      res_zero["rms_mms"] < 1e-9,
      f"all-zero input -> rms={res_zero['rms_mms']:.3e}mm/s (expected exactly 0)")

# ---- Test 3: constant DC signal (no AC content at all) ----------------------
# A pure DC offset (e.g. gravity on a mounted axis) must produce velocity RMS
# ~0 -- DC removal (step 2) plus the k=0 exclusion (HP floor, step 8's k>=8)
# must jointly suppress it, not partially leak it into the spectrum.
dc_counts = np.full(N, 2048, dtype=np.int16)  # ~1g offset, no AC
res_dc = axis_velocity_rms(dc_counts, SR_HZ)
check("3_constant_dc_gives_near_zero_velocity_rms",
      res_dc["rms_mms"] < 1e-6,
      f"pure DC (2048 counts, ~1g) -> velocity rms={res_dc['rms_mms']:.3e}mm/s "
      f"(expected ~0; DC removal + HP floor must suppress it)")

# ---- Test 4: single-axis vibration -> overall must equal that axis ---------
x_only = make_sine_counts(f0, A0)
y_zero = np.zeros(N, dtype=np.int16)
z_zero = np.zeros(N, dtype=np.int16)
tri = velocity_rms_triaxial(x_only, y_zero, z_zero, SR_HZ)
check("4_single_axis_overall_equals_that_axis",
      abs(tri["rms_overall_mms"] - tri["rms_x_mms"]) < 1e-9,
      f"X={tri['rms_x_mms']:.6f} Y={tri['rms_y_mms']:.6f} Z={tri['rms_z_mms']:.6f} "
      f"Overall={tri['rms_overall_mms']:.6f} (overall must == X exactly since Y=Z=0)")

# ---- Test 5: equal X/Y/Z -> overall == sqrt(3) * axis RMS -------------------
same = make_sine_counts(f0, A0)
tri_eq = velocity_rms_triaxial(same, same.copy(), same.copy(), SR_HZ)
expected_overall = tri_eq["rms_x_mms"] * np.sqrt(3.0)
rel_err5 = abs(tri_eq["rms_overall_mms"] - expected_overall) / expected_overall * 100
check("5_equal_xyz_overall_is_sqrt3_times_axis",
      rel_err5 < 1e-6,
      f"axis={tri_eq['rms_x_mms']:.6f}mm/s, sqrt(3)*axis={expected_overall:.6f}, "
      f"overall={tri_eq['rms_overall_mms']:.6f}, rel_err={rel_err5:.2e}%")

# ---- Test 6: known 1x + 2x components ---------------------------------------
# f1 = 25 Hz (e.g. 1500 RPM / 60), f2 = 50 Hz. RMS must reflect the combined
# power of both tones: RMS = sqrt(rms1^2 + rms2^2) for two orthogonal
# (uncorrelated-in-FFT-bin) sinusoids.
f1, f2 = 25.0, 50.0
A1, A2 = 4.0, 2.0
t = np.arange(N) / SR_HZ
a_combo = A1 * np.sin(2 * np.pi * f1 * t) + A2 * np.sin(2 * np.pi * f2 * t)
combo_counts = np.round(a_combo * (VIB_ACCEL_LSB_PER_G / VIB_ACCEL_G_MS2)).astype(np.int16)
res_combo = axis_velocity_rms(combo_counts, SR_HZ)
rms1_expected = (A1 / (2 * np.pi * f1)) / np.sqrt(2) * 1000.0
rms2_expected = (A2 / (2 * np.pi * f2)) / np.sqrt(2) * 1000.0
combo_expected = np.sqrt(rms1_expected ** 2 + rms2_expected ** 2)
rel_err6 = abs(res_combo["rms_mms"] - combo_expected) / combo_expected * 100
check("6_known_1x_2x_combo_matches_analytic_sum",
      rel_err6 < 5.0,
      f"f1={f1}Hz/A1={A1} + f2={f2}Hz/A2={A2} -> expected~{combo_expected:.4f}mm/s, "
      f"got={res_combo['rms_mms']:.4f}mm/s, rel_err={rel_err6:.3f}%")

# ---- Additional required checks (explicit, per task spec) ------------------

# Hann + NPG must not introduce a systematic bias: verify NPG matches the
# closed-form mean(w^2) computed independently here (not re-using the
# reference's own internal value blindly -- recomputed from scratch).
w = 0.5 - 0.5 * np.cos(2 * np.pi * np.arange(N) / (N - 1))
npg_independent = float(np.mean(w * w))
check("hann_npg_matches_independent_closed_form",
      abs(res["npg"] - npg_independent) < 1e-12,
      f"reference npg={res['npg']:.10f}, independently recomputed={npg_independent:.10f}")

# Frequency conversion must equal Fs/N exactly at every bin.
expected_df = SR_HZ / N
check("frequency_resolution_equals_fs_over_n",
      abs(res["freq_resolution_hz"] - expected_df) < 1e-12,
      f"Fs/N = {expected_df} Hz, got {res['freq_resolution_hz']} Hz "
      f"(spec: 2000/1024 = 1.953125 Hz)")
check("bin_8_equals_15p625_hz",
      abs(8 * expected_df - 15.625) < 1e-9,
      f"bin 8 * {expected_df} = {8*expected_df} Hz (spec: 15.625 Hz)")

# Velocity conversion must divide the POWER (mean-square) spectrum by
# (2*pi*f)^2 [ms_v = ms_a / omega^2, step 8]. Note this is a power-domain
# division: since ms_a ~ A^2, RMS_v = sqrt(ms_v) ~ A/omega is LINEAR in
# 1/omega, not 1/omega^2 -- doubling frequency must halve the velocity RMS
# (ratio 2.0), not quarter it. [Corrected during this review: the first
# draft of this test asserted ratio~4.0, confusing the power-domain
# (2*pi*f)^2 weighting with the amplitude-domain scaling it produces; the
# analytic single-tone closed form in Test 1 (rms = (A/omega)/sqrt(2)) is
# unambiguous and was independently re-derived by hand before writing this
# corrected assertion -- see source_trace.md's Phase 8 note.]
lowf, highf = 50.0, 100.0
lo_counts = make_sine_counts(lowf, A0)
hi_counts = make_sine_counts(highf, A0)
rms_lo = axis_velocity_rms(lo_counts, SR_HZ)["rms_mms"]
rms_hi = axis_velocity_rms(hi_counts, SR_HZ)["rms_mms"]
ratio = rms_lo / rms_hi
check("velocity_scales_as_inverse_of_2pi_f_in_amplitude_domain",
      abs(ratio - 2.0) < 0.05,
      f"same accel amplitude at {lowf}Hz vs {highf}Hz -> velocity ratio={ratio:.4f} "
      f"(expected ~2.0 = 100/50, i.e. RMS_v ~ A/omega; ms_v itself is divided "
      f"by omega^2 in the power domain, per step 8)")

# ---- Accel-path (time-domain) sanity, separate module but same principle ---
tri_accel = accel_rms_triaxial(x_only, y_zero, z_zero)
check("accel_single_axis_overall_equals_that_axis",
      abs(tri_accel["rms_overall_ms2"] - tri_accel["rms_x_ms2"]) < 1e-9,
      f"accel X={tri_accel['rms_x_ms2']:.6f} Overall={tri_accel['rms_overall_ms2']:.6f}")

print()
n_pass = sum(1 for _, s, _ in results if s == "PASS")
n_fail = sum(1 for _, s, _ in results if s == "FAIL")
print(f"SUMMARY: {n_pass} PASS, {n_fail} FAIL, {len(results)} total")

with open("synthetic_test_results.txt", "w", encoding="utf-8") as f:
    f.write("PHASE 8 SYNTHETIC SANITY TEST RESULTS\n")
    f.write("=" * 70 + "\n")
    for name, status, detail in results:
        f.write(f"[{status}] {name}: {detail}\n")
    f.write("\n")
    f.write(f"SUMMARY: {n_pass} PASS, {n_fail} FAIL, {len(results)} total\n")

print("\nResults written to synthetic_test_results.txt")
