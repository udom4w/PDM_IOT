# CT_TURNS Commissioning Checklist — Bench Execution

*Derived from `P2_CT_TURNS_VERIFICATION_PLAN.md` (commit `d385390`). Scope reminder: this checklist verifies only the current-scaling chain (`CT_TURNS`, `CT_RATIO_PRIMARY_A`/`CT_RATIO_SECONDARY_A`) — it does not test motor-state thresholds, hysteresis, or state-machine behavior. Findings are evidence only; no firmware change is authorized by this checklist alone.*

**Date:** _______________ **Technician:** _______________ **Machine/Tag:** _______________

---

## 1. Pre-Test Preparation

- ☐ `P2_CT_TURNS_VERIFICATION_PLAN.md` reviewed before starting
- ☐ Reference instrument datasheet/calibration certificate located (or confirmed unavailable)
- ☐ CTR4A01 datasheet accuracy spec located (or confirmed unavailable)
- ☐ External CT datasheet accuracy spec located, if applicable (or confirmed unavailable / not applicable)
- ☐ Firmware build identifier recorded: Git commit hash _______________ (from boot Serial banner / `build_info.h`)
- ☐ Confirmed `DEBUG_CURRENT_PATH` is enabled in the build under test (required for `[CURRENT_DIAG]` Serial output)
- ☐ Load-point availability confirmed (real motor load or controlled test load) for near-zero / mid-range / near-rated points
- ☐ Electrical safety precautions in place (LOTO / PPE as required for accessing the CT clamp and conductor)
- ☐ COM port / Serial monitor tooling ready (COM_______ at 115200 baud)

## 2. Hardware Inspection

- ☐ Power OFF confirmed before opening enclosure
- ☐ Physical turns counted through CTR4A01 clamp aperture: **_______ turns**
- ☐ Photograph taken of the turns/wiring
- ☐ External CT present? ☐ Yes ☐ No
  - If Yes: Model _______________ Ratio _______________
- ☐ Current firmware `CT_TURNS` value (from source): _______________
- ☐ Current firmware `CT_RATIO_PRIMARY_A` / `CT_RATIO_SECONDARY_A` values: _______________ / _______________

## 3. Required Instruments

| Instrument | Model | Serial No. | Calibration Status | Cal. Due Date |
|---|---|---|---|---|
| Reference ammeter/clamp meter | | | ☐ In cal ☐ Out of cal ☐ Unknown | |

- ☐ Serial/USB cable to ESP32 (COM5 or as assigned)
- ☐ Stopwatch/clock for pairing reference and Serial readings
- ☐ Camera/phone for turns photo (§2)

## 4. Test Setup

- ☐ Hardware inspection (§2) complete
- ☐ Power ON, `[CURRENT_DIAG]` confirmed streaming on Serial
- ☐ Three load points identified/achievable:
  - Near-zero: target _______ A
  - Mid-range (representative point only — not a threshold test): target _______ A
  - Near-rated: target _______ A
- ☐ Synchronization method for pairing reference reading ↔ Serial capture confirmed: _______________

## 5. Data Collection Sheet

| Load Point | Time | Reference Reading (A) | `rawA` | `engineeringA` | Expected* | Notes |
|---|---|---|---|---|---|---|
| Near-zero | | | | | | |
| Mid-range | | | | | | |
| Near-rated | | | | | | |

*Expected = `rawA × (CT_RATIO_PRIMARY_A / CT_RATIO_SECONDARY_A) / (counted turns)`

## 6. PASS / FAIL Decision Table

| Criterion | Result |
|---|---|
| Counted turns == `CT_TURNS` | ☐ Match ☐ Mismatch |
| External CT status == `CT_RATIO_*` constants | ☐ Match ☐ Mismatch ☐ N/A |
| Tolerance method used | ☐ Manufacturer method ☐ RSS ☐ ±5% fallback |
| Point 1 (near-zero) within tolerance | ☐ Pass ☐ Fail |
| Point 2 (mid-range) within tolerance | ☐ Pass ☐ Fail |
| Point 3 (near-rated) within tolerance | ☐ Pass ☐ Fail |
| Repeatable on re-measurement | ☐ Yes ☐ No ☐ Not re-checked |
| **Overall determination** | **☐ PASS ☐ FAIL** |

## 7. Evidence Package to Archive

- ☐ Turns-count photograph
- ☐ External-CT photograph/spec sheet (if applicable)
- ☐ Reference instrument traceability record (§3, filled in)
- ☐ This completed Data Collection Sheet (§5)
- ☐ Saved Serial log of `[CURRENT_DIAG]` output covering the test window
- ☐ Tolerance-method documentation, including any missing accuracy inputs
- ☐ Firmware build identifier (§1)
- ☐ This completed checklist (§1-6, signed/dated)

## 8. Follow-Up Actions for PASS

- ☐ Archive evidence package (§7)
- ☐ Update `P2_PLANNING.md` Technical Debt table: mark **D1** resolved, referencing this evidence package
- ☐ No firmware change required — `CT_TURNS`/`CT_RATIO_*` confirmed correct as installed
- ☐ Close P2 Item 1

## 9. Follow-Up Actions for FAIL

- ☐ **Do not modify firmware at the bench.** Document the specific discrepancy only (which constant, expected vs. observed delta, at which load point(s))
- ☐ Archive evidence package (§7) as-is, including the failing data
- ☐ Submit finding for separate review and approval before any firmware change is made
- ☐ If a firmware change is later approved: re-run this entire checklist afterward (not just the changed value) to confirm the fix
