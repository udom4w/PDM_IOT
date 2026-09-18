#!/usr/bin/env python3
# ==============================================================================
# INDEPENDENT REFERENCE IMPLEMENTATION -- WTVB05 RAW FIFO -> Velocity RMS
#
# Built from the MATHEMATICAL SPECIFICATION reconstructed from firmware source
# (vib_velocity.h/.cpp, vib_accel.h/.cpp, fifo_codec.cpp), NOT by transcribing
# the firmware's C++ line-by-line. Differences from the firmware on purpose:
#   - Uses numpy.fft.rfft (a library FFT) instead of the firmware's hand-rolled
#     iterative radix-2 DIT butterfly -- a different FFT algorithm computing
#     the same DFT, which is what "independent" is supposed to mean here.
#   - Written as vectorized numpy expressions, not the firmware's explicit
#     per-bin loop -- same math, different code shape.
#
# Source trace this file is built from (see source_trace.md for the full
# citation list):
#   - Byte decode / endianness / sign:      fifo_codec.cpp SampleDecoder_DecodeStride()
#   - g -> m/s^2 scaling:                    vib_accel.h VIB_ACCEL_LSB_PER_G / _G_MS2
#   - DC removal, Hann, NPG, FFT, one-sided
#     doubling, velocity PSD, HP floor, RMS
#     integration, mm/s conversion:          vib_velocity.cpp AxisVelocityRms()
#   - Overall RMS combination:               vib_velocity.cpp:398 (sqrt sum of squares)
#
# No calibration factor is applied anywhere in this file to make outputs
# match anything -- every constant here is the same named constant the
# firmware defines, used for the same purpose, nothing else.
# ==============================================================================

import numpy as np

# ---- Locked constants, taken verbatim from firmware headers (not tuned) ----
VIB_ACCEL_LSB_PER_G = 2048.0          # vib_accel.h
VIB_ACCEL_G_MS2     = 9.8             # vib_accel.h (deliberately not 9.80665)
VIB_VEL_FFT_N        = 1024           # vib_velocity.h
VIB_VEL_HP_BIN       = 8              # vib_velocity.h
VIB_VEL_MS_TO_MMS    = 1000.0         # vib_velocity.h


def decode_stride_be_i16(raw_bytes: bytes):
    """fifo_codec.cpp SampleDecoder_DecodeStride(): big-endian uint16 per
    axis, reinterpreted as signed int16 (two's complement). Stride = 6 bytes
    = X(0:2) Y(2:4) Z(4:6). Returns (x[], y[], z[]) as int16 numpy arrays."""
    n = len(raw_bytes) // 6
    x = np.zeros(n, dtype=np.int16)
    y = np.zeros(n, dtype=np.int16)
    z = np.zeros(n, dtype=np.int16)
    for i in range(n):
        o = i * 6
        rawX = (raw_bytes[o + 0] << 8) | raw_bytes[o + 1]
        rawY = (raw_bytes[o + 2] << 8) | raw_bytes[o + 3]
        rawZ = (raw_bytes[o + 4] << 8) | raw_bytes[o + 5]
        # unsigned 16-bit -> signed 16-bit, two's complement
        x[i] = rawX - 0x10000 if rawX >= 0x8000 else rawX
        y[i] = rawY - 0x10000 if rawY >= 0x8000 else rawY
        z[i] = rawZ - 0x10000 if rawZ >= 0x8000 else rawZ
    return x, y, z


def axis_accel_rms(raw_i16: np.ndarray):
    """vib_accel.cpp AxisRms(): raw -> g -> m/s^2, two-pass mean-removed RMS.
    Time domain only, rate-independent. Returns RMS in m/s^2 (float64)."""
    to_ms2 = VIB_ACCEL_G_MS2 / VIB_ACCEL_LSB_PER_G
    a = raw_i16.astype(np.float64) * to_ms2
    mean = a.mean()
    d = a - mean
    return float(np.sqrt(np.mean(d * d)))


def hann_symmetric(n: int):
    """vib_velocity.cpp BuildTables(): SYMMETRIC Hann, denominator (N-1),
    NOT the periodic/DFT-even form. w[i] = 0.5 - 0.5*cos(2*pi*i/(N-1))."""
    i = np.arange(n, dtype=np.float64)
    return 0.5 - 0.5 * np.cos(2.0 * np.pi * i / (n - 1))


def axis_velocity_rms(raw_i16: np.ndarray, sr_hz: float, n: int = VIB_VEL_FFT_N,
                       hp_bin: int = VIB_VEL_HP_BIN):
    """vib_velocity.cpp AxisVelocityRms(), Phase 3C arithmetic ONLY (steps
    1-9 of the header's METHOD comment). Returns a dict of every intermediate
    stage plus the final RMS in mm/s, for audit-level comparison.

    Method (frequency-domain integration, no time-domain integration):
      1. a[i] = raw[i] * (G_MS2/LSB_PER_G)                    -> m/s^2
      2. DC removal: a'[i] = a[i] - mean(a)
      3. w[i] = symmetric Hann(N-1); x[i] = a'[i]*w[i]
      4. X[k] = FFT(x)   (real input, N=1024)
      5. NPG = mean(w[i]^2)                (power gain, not mean(w))
      6. ms_a[k] = |X[k]|^2 / (NPG * N^2), x2 for interior bins (k != 0, N/2)
      7. f[k] = k * srHz / N
      8. ms_v[k] = ms_a[k] / (2*pi*f[k])^2,  summed for k = hp_bin .. N/2
      9. RMS = sqrt(sum(ms_v)) * 1000   -> mm/s
    """
    assert len(raw_i16) == n, f"sample count must be exactly {n}"

    to_ms2 = VIB_ACCEL_G_MS2 / VIB_ACCEL_LSB_PER_G
    a = raw_i16.astype(np.float64) * to_ms2                  # step 1
    mean = a.mean()
    a_ac = a - mean                                           # step 2 (DC removal)

    w = hann_symmetric(n)                                     # step 3 window
    x = a_ac * w

    X = np.fft.fft(x)                                         # step 4 (full complex FFT,
                                                                # independent algorithm from
                                                                # firmware's radix-2 DIT)
    nhalf = n // 2
    npg = float(np.mean(w * w))                                # step 5

    k = np.arange(hp_bin, nhalf + 1)
    Xk = X[hp_bin:nhalf + 1]
    ms_a = (np.abs(Xk) ** 2) / (npg * n * n)                  # step 6
    # one-sided doubling: interior bins only (k != 0 and k != N/2)
    interior = (k != 0) & (k != nhalf)
    ms_a = np.where(interior, ms_a * 2.0, ms_a)

    f = k * (sr_hz / n)                                        # step 7
    omega = 2.0 * np.pi * f
    ms_v = ms_a / (omega ** 2)                                 # step 8

    msv_sum = float(np.sum(ms_v))
    rms_ms = np.sqrt(msv_sum)                                  # step 9 (m/s)
    rms_mms = rms_ms * VIB_VEL_MS_TO_MMS

    # dominant frequency: argmax of ms_v over the same summed range, with
    # parabolic interpolation on log-power -- vib_velocity.cpp:254-295,
    # reconstructed from the spec, not copied statement-by-statement.
    dom_hz = None
    if len(ms_v) >= 3:
        peak_idx = int(np.argmax(ms_v))
        if 0 < peak_idx < len(ms_v) - 1 and ms_v[peak_idx] > 0:
            p_prev, p_peak, p_next = ms_v[peak_idx - 1], ms_v[peak_idx], ms_v[peak_idx + 1]
            delta = 0.0
            if p_prev > 0 and p_next > 0:
                y1, y2, y3 = np.log(p_prev), np.log(p_peak), np.log(p_next)
                den = y1 - 2 * y2 + y3
                if abs(den) > 1e-12:
                    d = 0.5 * (y1 - y3) / den
                    if abs(d) <= 0.5:
                        delta = d
            peak_bin = k[peak_idx]
            dom_hz = (peak_bin + delta) * (sr_hz / n)

    return {
        "mean_dc_ms2": mean,
        "npg": npg,
        "bin_range": (int(k[0]), int(k[-1])),
        "freq_resolution_hz": sr_hz / n,
        "msv_sum": msv_sum,
        "rms_ms": rms_ms,
        "rms_mms": rms_mms,
        "dominant_frequency_hz": dom_hz,
        # intermediate arrays retained for checkpoint-level comparison
        "_a_ac": a_ac, "_windowed": x, "_fft": X, "_ms_a": ms_a, "_ms_v": ms_v, "_bins": k,
    }


def velocity_rms_triaxial(raw_x: np.ndarray, raw_y: np.ndarray, raw_z: np.ndarray,
                           sr_hz: float):
    """vib_velocity.cpp VibVelocity_ComputeRms(): per-axis velocity RMS, then
    overall = sqrt(x^2+y^2+z^2) [vib_velocity.cpp:398]. No mean-of-3, no /3."""
    rx = axis_velocity_rms(raw_x, sr_hz)
    ry = axis_velocity_rms(raw_y, sr_hz)
    rz = axis_velocity_rms(raw_z, sr_hz)
    overall = float(np.sqrt(rx["rms_mms"] ** 2 + ry["rms_mms"] ** 2 + rz["rms_mms"] ** 2))
    return {
        "rms_x_mms": rx["rms_mms"],
        "rms_y_mms": ry["rms_mms"],
        "rms_z_mms": rz["rms_mms"],
        "rms_overall_mms": overall,
        "dominant_frequency_x_hz": rx["dominant_frequency_hz"],
        "dominant_frequency_y_hz": ry["dominant_frequency_hz"],
        "dominant_frequency_z_hz": rz["dominant_frequency_hz"],
        "_axes": {"x": rx, "y": ry, "z": rz},
    }


def accel_rms_triaxial(raw_x: np.ndarray, raw_y: np.ndarray, raw_z: np.ndarray):
    """vib_accel.cpp VibAccel_ComputeRms(): time-domain accel RMS, overall =
    sqrt(x^2+y^2+z^2). Independent of sample rate."""
    rx = axis_accel_rms(raw_x)
    ry = axis_accel_rms(raw_y)
    rz = axis_accel_rms(raw_z)
    overall = float(np.sqrt(rx ** 2 + ry ** 2 + rz ** 2))
    return {"rms_x_ms2": rx, "rms_y_ms2": ry, "rms_z_ms2": rz, "rms_overall_ms2": overall}


if __name__ == "__main__":
    print(__doc__)
    print("This module is a library; run synthetic_tests.py to exercise it.")
