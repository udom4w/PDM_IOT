#!/usr/bin/env python3
# ==============================================================================
# PHASE 6 -- Numerical VRMS validation: firmware output vs. independent
# reference (reference_vrms.py), run against the SAME real RAW FIFO 1024-
# sample captures for both sides. RAW samples are read once, from the
# evidence JSON files, and used ONLY as reference_vrms.py's input -- the
# firmware's own RMS numbers are read separately, from the same JSON, and
# are NEVER fed into the reference computation (Phase 6.7 independence).
# ==============================================================================

import json, glob, os, math
import numpy as np
from reference_vrms import velocity_rms_triaxial, VIB_VEL_FFT_N, VIB_VEL_HP_BIN

EVDIR = os.path.dirname(os.path.abspath(__file__))
CAPTURES_DIR = os.path.join(EVDIR, "raw_captures")
TOLERANCE_PCT = 0.5  # fixed in advance, per Phase 6.5 -- not adjusted after seeing results

EXCLUDED_CAPTURE_IDS = {25}  # INVALID -- mid-dump data loss, excluded per prior task's finding

capture_files = sorted(
    glob.glob(os.path.join(CAPTURES_DIR, "capture_*.json")),
    key=lambda p: int(os.path.basename(p).split("_")[1].split(".")[0])
)

rows = []
pairing_report = []
divergence_notes = []

for path in capture_files:
    with open(path, "r", encoding="utf-8") as f:
        cap = json.load(f)

    cid = cap["captureId"]

    # ---- PHASE 6.1: pairing verification ----
    n_x, n_y, n_z = len(cap["raw_x"]), len(cap["raw_y"]), len(cap["raw_z"])
    pairing_ok = (
        cap.get("valid", False) is True
        and cap["sample_count"] == 1024
        and n_x == 1024 and n_y == 1024 and n_z == 1024
        and cap["sr_hz"] == 2000
        and all(k in cap for k in ("firmware_rms_x", "firmware_rms_y", "firmware_rms_z", "firmware_rms_overall"))
    )
    pairing_report.append({
        "captureId": cid, "sample_count": cap["sample_count"], "sr_hz": cap["sr_hz"],
        "raw_x_count": n_x, "raw_y_count": n_y, "raw_z_count": n_z,
        "firmware_rms_x": cap["firmware_rms_x"], "firmware_rms_y": cap["firmware_rms_y"],
        "firmware_rms_z": cap["firmware_rms_z"], "firmware_rms_overall": cap["firmware_rms_overall"],
        "pairing_ok": pairing_ok,
    })
    if not pairing_ok:
        continue  # excluded from numerical comparison

    # ---- PHASE 6.8: sampling consistency ----
    assert cap["sample_count"] == VIB_VEL_FFT_N == 1024
    assert cap["sr_hz"] == 2000
    df = cap["sr_hz"] / VIB_VEL_FFT_N
    assert abs(df - 1.953125) < 1e-12
    assert abs(VIB_VEL_HP_BIN * df - 15.625) < 1e-12
    window_s = VIB_VEL_FFT_N / cap["sr_hz"]
    assert abs(window_s - 0.512) < 1e-12

    # ---- PHASE 6.2: independent reference, RAW-only input ----
    raw_x = np.array(cap["raw_x"], dtype=np.int16)
    raw_y = np.array(cap["raw_y"], dtype=np.int16)
    raw_z = np.array(cap["raw_z"], dtype=np.int16)
    ref = velocity_rms_triaxial(raw_x, raw_y, raw_z, sr_hz=cap["sr_hz"])

    fw_x, fw_y, fw_z, fw_o = (cap["firmware_rms_x"], cap["firmware_rms_y"],
                               cap["firmware_rms_z"], cap["firmware_rms_overall"])
    ref_x, ref_y, ref_z, ref_o = (ref["rms_x_mms"], ref["rms_y_mms"],
                                   ref["rms_z_mms"], ref["rms_overall_mms"])

    def abs_err(fw, r):
        return abs(fw - r)

    def rel_err_pct(fw, r):
        # Reference ~= 0 guard -- never divide by zero (Phase 6.3 requirement).
        if abs(r) < 1e-9:
            return None  # undefined; reported as such, never fabricated as 0% or skipped silently
        return abs(fw - r) / abs(r) * 100.0

    ax, ay, az, ao = abs_err(fw_x, ref_x), abs_err(fw_y, ref_y), abs_err(fw_z, ref_z), abs_err(fw_o, ref_o)
    rx, ry, rz, ro = rel_err_pct(fw_x, ref_x), rel_err_pct(fw_y, ref_y), rel_err_pct(fw_z, ref_z), rel_err_pct(fw_o, ref_o)

    # ---- PHASE 6.9: Overall RMS relationship, both sides, every capture ----
    fw_overall_check = math.sqrt(fw_x**2 + fw_y**2 + fw_z**2)
    ref_overall_check = math.sqrt(ref_x**2 + ref_y**2 + ref_z**2)
    fw_overall_consistent = abs(fw_overall_check - fw_o) < 1e-4  # firmware's own internal consistency
    ref_overall_consistent = abs(ref_overall_check - ref_o) < 1e-9  # reference's own internal consistency

    rows.append({
        "captureId": cid,
        "Firmware_X": fw_x, "Reference_X": ref_x, "AbsError_X": ax, "RelError_X_%": rx,
        "Firmware_Y": fw_y, "Reference_Y": ref_y, "AbsError_Y": ay, "RelError_Y_%": ry,
        "Firmware_Z": fw_z, "Reference_Z": ref_z, "AbsError_Z": az, "RelError_Z_%": rz,
        "Firmware_Overall": fw_o, "Reference_Overall": ref_o, "AbsError_Overall": ao, "RelError_Overall_%": ro,
        "fw_overall_selfconsistent": fw_overall_consistent,
        "ref_overall_selfconsistent": ref_overall_consistent,
    })

# ---- write CSV ----
csv_path = os.path.join(EVDIR, "numerical_comparison.csv")
with open(csv_path, "w", encoding="utf-8") as f:
    cols = ["captureId",
            "Firmware_X", "Reference_X", "AbsError_X", "RelError_X_%",
            "Firmware_Y", "Reference_Y", "AbsError_Y", "RelError_Y_%",
            "Firmware_Z", "Reference_Z", "AbsError_Z", "RelError_Z_%",
            "Firmware_Overall", "Reference_Overall", "AbsError_Overall", "RelError_Overall_%",
            "fw_overall_selfconsistent", "ref_overall_selfconsistent"]
    f.write(",".join(cols) + "\n")
    for row in rows:
        f.write(",".join(str(row[c]) for c in cols) + "\n")

# ---- PHASE 6.4: statistics ----
def stats_for(axis_key):
    abs_vals = [r[f"AbsError_{axis_key}"] for r in rows]
    rel_vals = [r[f"RelError_{axis_key}_%"] for r in rows if r[f"RelError_{axis_key}_%"] is not None]
    rms_err = math.sqrt(sum(v**2 for v in abs_vals) / len(abs_vals)) if abs_vals else None
    max_abs_row = max(rows, key=lambda r: r[f"AbsError_{axis_key}"])
    min_abs_row = min(rows, key=lambda r: r[f"AbsError_{axis_key}"])
    return {
        "mean_abs_error": sum(abs_vals) / len(abs_vals) if abs_vals else None,
        "max_abs_error": max(abs_vals) if abs_vals else None,
        "mean_rel_error_%": sum(rel_vals) / len(rel_vals) if rel_vals else None,
        "max_rel_error_%": max(rel_vals) if rel_vals else None,
        "rms_error": rms_err,
        "max_error_captureId": max_abs_row["captureId"],
        "min_error_captureId": min_abs_row["captureId"],
    }

stats = {axis: stats_for(axis) for axis in ("X", "Y", "Z", "Overall")}

pass_fail = {}
for axis in ("X", "Y", "Z", "Overall"):
    n_pass = sum(1 for r in rows if r[f"RelError_{axis}_%"] is not None and r[f"RelError_{axis}_%"] <= TOLERANCE_PCT)
    n_fail = sum(1 for r in rows if r[f"RelError_{axis}_%"] is not None and r[f"RelError_{axis}_%"] > TOLERANCE_PCT)
    n_undef = sum(1 for r in rows if r[f"RelError_{axis}_%"] is None)
    pass_fail[axis] = {"PASS": n_pass, "FAIL": n_fail, "UNDEFINED (ref~=0)": n_undef}

fw_overall_all_consistent = all(r["fw_overall_selfconsistent"] for r in rows)
ref_overall_all_consistent = all(r["ref_overall_selfconsistent"] for r in rows)

print(f"Captures evaluated: {len(rows)} / {len(pairing_report)} in evidence "
      f"({sum(1 for p in pairing_report if not p['pairing_ok'])} failed pairing)")
print(f"\nPer-axis PASS/FAIL @ {TOLERANCE_PCT}% tolerance:")
for axis in ("X", "Y", "Z", "Overall"):
    print(f"  {axis}: {pass_fail[axis]}")
print(f"\nStatistics:")
for axis in ("X", "Y", "Z", "Overall"):
    s = stats[axis]
    print(f"  {axis}: mean_abs={s['mean_abs_error']:.6f} max_abs={s['max_abs_error']:.6f} "
          f"(captureId={s['max_error_captureId']}) mean_rel%={s['mean_rel_error_%']:.4f} "
          f"max_rel%={s['max_rel_error_%']:.4f} rms_err={s['rms_error']:.6f}")
print(f"\nFirmware Overall self-consistent (sqrt(X^2+Y^2+Z^2)==Overall) for all captures: {fw_overall_all_consistent}")
print(f"Reference Overall self-consistent for all captures: {ref_overall_all_consistent}")

# dump machine-readable summary for the report writer
with open(os.path.join(EVDIR, "phase6_stats.json"), "w", encoding="utf-8") as f:
    json.dump({
        "tolerance_pct": TOLERANCE_PCT,
        "n_evaluated": len(rows),
        "n_pairing_total": len(pairing_report),
        "pairing_report": pairing_report,
        "stats": stats,
        "pass_fail": pass_fail,
        "fw_overall_all_consistent": fw_overall_all_consistent,
        "ref_overall_all_consistent": ref_overall_all_consistent,
        "excluded_capture_ids": sorted(EXCLUDED_CAPTURE_IDS),
    }, f, indent=2)

print(f"\nCSV written: {csv_path}")
print("Stats JSON written: phase6_stats.json")
