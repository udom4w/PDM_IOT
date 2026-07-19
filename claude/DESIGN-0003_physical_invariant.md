> **Document Status**
>
> **Design Review:** 🟡 Draft — pending approval
>
> **Implementation:** ⛔ NOT STARTED — do not implement until this document is approved
>
> **Governing plan:** `Phase1_Implementation_Plan.md` §Phase 1.2 (authoritative roadmap)
>
> **Firmware baseline:** `WTVB02_ESP32S3_V16_CM_fault_latch_v3_hardened_patched_v16_5.ino`, HEAD `4f63f92`
>
> **Depends on:** `DESIGN-0004` (Peak Telemetry) — hard dependency, this check needs Peak values that don't exist yet. Soft relationship to `DESIGN-0001` (Trigger Manager) — see §4.5.
>
> **Source of Truth:** `Phase1_Implementation_Plan.md` is authoritative for scope.

# DESIGN-0003: PhysicalInvariant() — Diagnostic-Only Layer (Phase 1.2)

## 1. Source and Scope

Per `Phase1_Implementation_Plan.md` §"Phase 1.2 — `PhysicalInvariant()` as a diagnostic-only layer":

> New, separate function — does not modify `rms_overall`, `motor_state`, or any existing deglitch variable. Checks: `Peak < 0`, `Peak/RMS` ratio outside a wide sanity band, `NaN`, `Inf`. On violation: **log only** (Serial + a telemetry counter), do not suppress, do not trigger fault-latch, do not affect alarms.

In scope: the check function itself, its violation counter, and (once `DESIGN-0001` exists) an optional submission to the Trigger Manager. Out of scope: wiring into fault-latch/alarm decisions (Phase 1.4 only, explicitly gated on Phase 1.3 field data per the plan).

## 2. Facts From Current Source (v16.5) — [FACTS]

**2.1 No existing Peak/RMS ratio or NaN/Inf check exists for this purpose.** However, an existing, reusable primitive does exist: `isFloatSafe()`, already used at the VRMS validation site (line ~3622-3624: `!isFloatSafe(localData.vel_peak_x) || !isFloatSafe(localData.vel_peak_y) || !isFloatSafe(localData.vel_peak_z)`). `PhysicalInvariant()`'s NaN/Inf check should reuse this existing function rather than reimplementing NaN/Inf detection — smallest-change principle, and avoids two slightly-different definitions of "safe float" coexisting in the same file.

**2.2 Existing "log + counter, don't gate" precedent.** Two established idioms already do exactly this pattern in this file: `g_deglitchCount` (incremented on VRMS de-glitch, exposed in `/sensor` as `deglitch_count` and in the 30s Serial status report) and `g_telemBufOverflowCount` (same pattern, exposed in the Serial report table). `PhysicalInvariant()`'s violation counter should follow this exact precedent — new counter, new `/status`-or-`/sensor` JSON key, same Serial report table treatment. No new counter idiom needs inventing.

**2.3 Cross-check against RFC-0005's own field data.** RFC-0005 §4 (Multi-Speed Validation Summary) measured real Peak/RMS ratios on `pump01` hardware:

| Axis | 25 Hz | 35 Hz | 45 Hz |
|---|---|---|---|
| X | 1.43 | 1.41 | 1.41 |
| Y | 1.40 | 0.64 | 1.41 |
| Z | 4.23 | 0.53 | 11.84 |

X is stable near the theoretical 1.414. Y and especially Z vary widely across already-observed, real, non-fault conditions (Z ranges 0.53–11.84). **A "wide sanity band" that doesn't false-trigger on Z's already-observed 11.84 must be wide enough that it may miss genuine anomalies on the more stable X axis.** This is a real, evidence-backed tension the plan's phrase "wide sanity band" glosses over — RFC-0005 itself flags Z's ratio as "should not be treated as a stable reference at all" pending the harmonic-lock question (RFC-0005 §6, RFC-0007 §4) being resolved. See Open Question 1.

## 3. Design Goal

A pure diagnostic evaluator, run once per cycle on already-finalized Peak (from `DESIGN-0004`) and RMS values. Never mutates `rms_overall`, `motor_state`, or any existing deglitch variable — this is the plan's own stated invariant, restated here as binding.

## 4. Detailed Design

**4.1 Checks (verbatim from the governing plan).**
- `Peak < 0`
- `Peak/RMS` ratio outside a wide sanity band
- `NaN`
- `Inf`

**4.2 Execution context.** Peak (once `DESIGN-0004` lands) and RMS both originate in `taskModbusRead()` (Core 0), populated into the same `VibrationData_t` struct within the same 250ms cycle. Recommend evaluating `PhysicalInvariant()` inline in `taskModbusRead()` immediately after both values are populated for that cycle — this avoids a cross-core read of not-yet-finalized values and keeps the check co-located with its inputs. This mirrors this project's general staged-pipeline discipline (raw → validate → condition, all on Core 0 before anything crosses to Core 1) without importing any specific document from the Telemetry Snapshot track, which is out of scope per current direction.

**4.3 Motor-state gating (inherited requirement, not invented).** Per `DESIGN-0004` §2.4/§4.5, Peak and RMS are recommended to read `0` while not RUNNING. A Peak/RMS ratio computed on two zeros (or a transiently-zero RMS during STOPPED/STARTING) is meaningless and would either divide-by-zero or fire a spurious violation. Recommend: `PhysicalInvariant()` only evaluates the ratio check while `motor_state == MOTOR_RUNNING` — mirroring the existing kurtosis-validity gating pattern (`kurtosisValid`) already present in this file. The `NaN`/`Inf`/`Peak < 0` checks are cheap and structural (not physically meaningful only during RUNNING) and may run unconditionally; this is a design choice to confirm at approval, not a firm recommendation either way.

**4.4 Violation counter and log.** New counter (e.g. `g_physInvariantViolationCount`), exposed via `/status` alongside `deglitch_count` (§2.2 precedent), plus a Serial log line per violation using this file's existing `[WARN]`/`[SENSOR] !` prefix convention.

**4.5 Trigger Manager hookup — soft dependency on `DESIGN-0001`.** On violation, `PhysicalInvariant()` may submit a Normal-tier trigger event via `DESIGN-0001`'s interface, if `DESIGN-0001` has already landed. If not, `PhysicalInvariant()` still functions standalone (log + counter only) — this matches the governing plan's own explicit statement that evaluation "keeps evaluating and logging every cycle either way... Phase 1.2 is unaffected" by whether the FIFO-capture-consuming side exists yet. No hard dependency in either direction between `DESIGN-0001` and `DESIGN-0003`; only `DESIGN-0004` is a hard prerequisite.

## 5. Non-Goals

- No fault-latch, deglitch, or alarm wiring. Explicitly deferred to Phase 1.4, itself gated on ~30 days of Phase 1.3 field data per the governing plan — "not yet" here means not yet, not "soon regardless."
- No change to `rms_overall`, `motor_state`, or any existing deglitch/suppression variable.
- No new sanity-band constant is finalized by this document (see Open Question 1).

## 6. Failure Modes

If the check's own inputs are NaN/Inf — the exact condition being detected — the check itself must not crash or print garbage. Use the existing `isFloatSafe()` guard (§2.1) before any `Serial.printf` formatting of the offending value, consistent with how this project already guards float formatting elsewhere in the file.

## 7. Open Questions (require explicit approval before implementation)

1. **Exact sanity-band bounds.** The plan says "wide" without a number. Per §2.3, RFC-0005's own data shows real, non-fault Z-axis ratios up to 11.84 and as low as 0.53. Recommend deriving the actual band from a reanalysis of RFC-0005's dataset (cheap, already-collected data) before picking a number, rather than guessing — consistent with `DESIGN_PRINCIPLES.md`'s "unknown is preferable to guessed."
2. **Execution location.** Core 0 inline in `taskModbusRead()` is recommended (§4.2) but not yet approved.
3. **Unconditional vs. RUNNING-gated NaN/Inf checks** (§4.3) — need an explicit choice.
4. **Counter persistence.** Should `g_physInvariantViolationCount` be NVS-persisted (like `g_rebootCount`) or RAM-only (like `g_deglitchCount`)? Matters for Phase 1.3's 30-day field study, which likely wants the count to survive a reboot — flagged since the plan's Phase 1.3 depends on this counter being meaningful over a long field window, and a RAM-only counter resets on every reboot.

## 8. Test Plan (for when implementation is approved)

- Clean build, zero new warnings.
- Confirm `rms_overall`, `motor_state`, and all existing deglitch variables are provably unaffected by this change (diff review — this function must be additive-only).
- Force a synthetic `Peak < 0` / NaN / Inf input (test build only) and confirm: counter increments, Serial log fires, no fault-latch entry, no alarm change, no crash.
- Once the sanity-band bound (Open Question 1) is set, run against a recorded RFC-0005-style dataset offline (or replay on bench) to confirm the chosen band does not fire on the already-observed, non-fault Z-axis 11.84 ratio.

## 9. Summary for Sign-Off

| Item | Status |
|---|---|
| Scope confirmed against governing plan | ✅ |
| Reusable primitives identified (`isFloatSafe`, counter/log precedent) | ✅ |
| Sanity-band bound cross-checked against RFC-0005 field data | ✅ Flagged real tension — Open Question 1 |
| Motor-state gating requirement inherited from DESIGN-0004 | ✅ Flagged — Open Question 3 |
| Hard dependency on DESIGN-0004 confirmed | ✅ |
| Soft (non-blocking) relationship to DESIGN-0001 confirmed | ✅ |
| **Patch status** | **Draft — 4 open questions must be resolved/approved before implementation begins** |
