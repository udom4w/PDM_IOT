# P2 Item 1 — Verification Plan: CT_TURNS / CT Ratio Configuration

## Scope

This plan validates **only** the current-scaling signal chain: `CT_TURNS` and `CT_RATIO_PRIMARY_A`/`CT_RATIO_SECONDARY_A`, and the resulting `current_a` value produced by `compensateCurrent()`.

It explicitly does **not** validate `CURRENT_ON_THRESHOLD_A`/`CURRENT_OFF_THRESHOLD_A`, hysteresis behavior, START/STOP/STOPPING/RUNNING transition logic, or any part of `updateMotorStateMachine()`/`buildMotorStateEvidence()`. Those remain out of scope for P2 Item 1 and would need their own separate verification if ever revisited.

## Philosophy (ground rules)

- Verification is evidence collection only.
- Findings from this plan do not themselves authorize any firmware change.
- Any firmware modification (to `CT_TURNS`, `CT_RATIO_*`, or anything else) requires a separate, explicit review and approval **after** this verification is complete and its findings have been reviewed.

## 1. What exactly must be verified

- Physically-counted conductor turns through the CTR4A01 clamp vs. `CT_TURNS`.
- External-CT presence/ratio vs. `CT_RATIO_PRIMARY_A`/`CT_RATIO_SECONDARY_A`.
- Firmware's `current_a` vs. independent reference truth, across representative real load points.

## 2. Acceptance Criteria

- Physically-counted turns == `CT_TURNS` (exact integer match).
- No external CT present, **or** if present, its ratio matches the `CT_RATIO_*` constants.
- **Tolerance — determined by this hierarchy:**
  1. Use the manufacturer's documented uncertainty-combination method, if the reference instrument's, CTR4A01's, or CT's datasheet specifies one.
  2. Otherwise, RSS (root-sum-square) is the preferred default for combining independent random error sources: reference instrument accuracy, CTR4A01 accuracy, CT accuracy (if a real external CT exists — otherwise fold into CTR4A01's figure), and firmware signal-chain error (register resolution, EMA settling/lag at non-steady points, floating-point rounding).
  3. If the inputs needed for either method above are unavailable, use **±5%** as the documented fallback.
  - **The accuracy budget supports the acceptance criteria — it is not a prerequisite for closing P2 Item 1.** If specs are unavailable, verification may still be completed using documented measurement evidence and the ±5% fallback, with the limitation explicitly recorded.
- Agreement within the applicable tolerance at ≥3 load points: near-zero, a mid-range point (chosen for being representative of the real operating range — not a validation of `CURRENT_ON/OFF_THRESHOLD_A` themselves, per Scope), and near-rated load.
- Repeatable on re-measurement.

## 3. Required Measurements

- Physical turns count.
- External-CT presence/ratio.
- `rawA`/`engineeringA` (from `[CURRENT_DIAG]`) at each load point.
- Reference instrument reading at each load point.
- Reference instrument traceability: model, serial number (if available), calibration status, calibration date (if available).

## 4. Required Equipment

- A reference ammeter/clamp meter, with its traceability information (per §3) accessible.
- Physical access to the installed CTR4A01 clamp and monitored conductor.
- A way to observe ≥3 real or controlled load points.
- Serial/USB connection to the ESP32 (COM5) to read `[CURRENT_DIAG]` live.
- A clock/stopwatch sufficient to pair reference and Serial readings to the same moment.

## 5. Verification Procedure

0. Record reference instrument traceability (model, serial number, calibration status, calibration date).
1. Attempt to gather the accuracy inputs in §2; determine tolerance per the stated hierarchy. If unavailable, proceed with the ±5% fallback and record which inputs were missing — this does not block proceeding.
2. Power off. Visually inspect the CTR4A01 clamp installation; count and photograph the physical turns. Inspect for an external CT; record its ratio if present.
3. Power on, confirm `[CURRENT_DIAG]` is streaming normally.
4. At each of the 3 load points: simultaneously record the reference-meter reading and the firmware's `rawA=`/`engineeringA=`.
5. Compute, per point: `expected = rawA × (CT_RATIO_PRIMARY_A/CT_RATIO_SECONDARY_A) / (physically-counted turns)`. Compare against (a) firmware's actual `engineeringA` and (b) the reference reading.

   This calculation is an independent verification of the scaling equation.
   It is used only to validate the current scaling chain and must not replace
   or modify the firmware implementation.
6. Record findings only — do not act on them (per Philosophy).

## 6. Expected Outputs

- Turns count + photo; external-CT finding.
- Reference instrument traceability record.
- 3-point comparison table (`rawA`, `engineeringA`, reference reading).
- Tolerance method used (manufacturer method / RSS / ±5% fallback) and rationale, including any missing accuracy inputs.
- PASS/FAIL determination against §2.
- If FAIL: an evidence-backed recommendation only — no implementation performed as part of this verification.

## 7. Evidence Sufficient to Close P2 Item 1

All of:
- Turns count documented and compared to `CT_TURNS`.
- External-CT status documented and compared to the `CT_RATIO_*` constants.
- Reference instrument traceability recorded.
- ≥3 load-point comparisons within the applicable tolerance — a formal accuracy budget, RSS combination, **or** the ±5% fallback are each independently sufficient bases; none is required over another.
- Tolerance method used and any missing accuracy inputs explicitly recorded.
- Explicit statement that these findings alone do not authorize any firmware change — separate review and approval is required before any modification.
