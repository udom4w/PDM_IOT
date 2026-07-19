> **Document Status**
>
> **Design Review:** 🟡 Draft — pending approval (unchanged; no evidence of approval found)
>
> **Implementation:** ⚠️ PRESENT IN CURRENT BASELINE (commit `4274e524ea3568e693c3fa1e5068b7ce118599fb`) — implemented ahead of, and independent of, this document's approval gate. See §10 Provenance Note for the full retrospective reconciliation. This status line previously read "NOT STARTED — do not implement until this document is approved," which no longer reflects the actual repository state and has been corrected here rather than left inaccurate.
>
> **Governing plan:** `Phase1_Implementation_Plan.md` §Phase 1.1 (authoritative roadmap, per project owner direction — supersedes `IMPLEMENTATION_PLAN_telemetry_snapshot.md` for this work)
>
> **Firmware baseline:** `WTVB02_ESP32S3_V16_CM_fault_latch_v3_hardened_patched_v16_5.ino`, HEAD `4f63f92` ("v16.5 dev baseline candidate")
>
> **Depends on:** None — this is the first, foundation phase. `DESIGN-0003` (PhysicalInvariant) has a hard dependency on this document's output (Peak values).
>
> **Source of Truth:** `Phase1_Implementation_Plan.md` is authoritative for scope. This document adds implementation-grounding facts pulled directly from the current `.ino` source; it does not introduce new scope beyond Phase 1.1.

# DESIGN-0004: Peak Telemetry (Phase 1.1)

## 1. Source and Scope

Per `Phase1_Implementation_Plan.md` §"Phase 1.1 — Add Peak telemetry (VX/VY/VZ), read-only, zero decision impact":

> Add `readHoldingRegisters` calls for VX/VY/VZ (0x3A-0x3C) alongside the existing VRMS reads. Publish as new telemetry fields only (e.g. `peak_x`, `peak_y`, `peak_z`). **Explicitly NOT wired into any fault-latch, deglitch, or alarm logic yet.**

In scope: one new Modbus read transaction, new struct fields, new additive JSON fields on `/sensor`. Out of scope: any decision-path change, any change to existing fields, Modbus transaction consolidation (explicitly deferred to v17 per the plan's §1 correction table).

## 2. Facts From Current Source (v16.5) — [FACTS]

Verified by direct grep/read of `WTVB02_ESP32S3_V16_CM_fault_latch_v3_hardened_patched_v16_5.ino`, not inferred:

**2.1 Registers 0x3A–0x3C are not currently read anywhere in this firmware.** A full-file search of every `#define REG_*` constant shows only:
```
REG_VRMS_X 0x50   REG_VRMS_Y 0x5C   REG_VRMS_Z 0x68
REG_TEMPERATURE 0x40   REG_FREQ_X 0x44 (3 consecutive)
REG_CFX 0x47   REG_CFY 0x53   REG_CFZ 0x5F
```
No `REG_VX`/`REG_PEAK`/`0x3A` constant exists. The plan's premise — that Peak registers are not currently polled — is confirmed true.

**2.2 Naming collision risk — the most important fact this review surfaced.** `VibrationData_t` already declares `vel_peak_x` / `vel_peak_y` / `vel_peak_z` (struct, lines ~1137–1141), with a field comment reading *"peak velocity X จาก register โดยตรง (0x3A / 100)"*. That comment describes an **earlier** version of this firmware. A file-level comment at line 128 documents the actual history:

> `// เปลี่ยนจาก VX/VY/VZ (0x3A Peak ÷100) → VRMSX/Y/Z (True RMS ÷1000)`

("changed from VX/VY/VZ (0x3A Peak ÷100) to VRMSX/Y/Z (True RMS ÷1000)"). Confirmed at the read site (line ~3561): `localData.vel_peak_x = abs(raw_x) / 1000.0f; // [mm/s] VRMS X`, where `raw_x` comes from `REG_VRMS_X` (0x50), not 0x3A.

**Net effect: today, `vel_peak_x`/`vel_peak_y`/`vel_peak_z` are live, populated struct fields that — despite their name — hold VRMS content, not true Peak content.** Per-axis publication of these fields was removed from `/sensor` JSON entirely in v16.3i ("`vel_peak_x/y/z` removed — ซ้ำซ้อนกับ vx/vy/vz", line ~5517) as redundant with `vx`/`vy`/`vz` (which are themselves VRMS-per-axis, published from `reportedVx/Vy/Vz`). So `vel_peak_x` is populated every cycle, published nowhere, and misleadingly named.

**This DESIGN must not introduce new fields named `peak_x`/`peak_y`/`peak_z` without addressing this collision** — a future reader (human or AI-assisted session) searching the codebase for "peak" will find both the legacy `vel_peak_x` (= VRMS, mislabeled) and the new true-Peak field, with no code-level signal distinguishing them beyond the name itself. See §7, Open Question 1.

**2.3 Existing 8-transaction poll structure.** `taskModbusRead()` (Core 0, `PRIORITY_MODBUS = 5`, 250ms/4Hz cadence) currently issues 8 sequential Modbus transactions per cycle (VRMS×3, Temp, Freq×1-call-of-3, CF/K×3), each separated by a 5ms `vTaskDelay`. Adding VX/VY/VZ as one more transaction is structurally a 9th transaction of the same shape — consistent with the plan's explicit deferral of transaction *consolidation* to v17. This DESIGN does not touch that consolidation question.

**2.4 Mandatory RUNNING-gate policy — a governing document `Phase1_Implementation_Plan.md` does not mention.** `PATCH_NOTES_v16.5.md`'s "Engineering Rule (IMPORTANT)" states, unconditionally:

> Any feature originating from the sensor that is intended for machine-condition analysis MUST follow the same export policy. If `motor_state != MOTOR_RUNNING` then exported value shall be zero. Examples include RMS, Peak, Crest Factor, Kurtosis, Frequency, **Future statistical features**. Do not export raw statistical values while STOPPED.

Peak is named explicitly. `Phase1_Implementation_Plan.md`'s Phase 1.1 description says nothing about this gate. Per `CLAUDE_RULES.md` §14 ("If DESIGN and source code disagree, report the conflict. Do not choose automatically"), this is flagged rather than silently resolved either way — see §4.5 and Open Question 4.

## 3. Design Goal

Read true Peak (registers 0x3A–0x3C) as new, additive, distinctly-named telemetry fields. Zero impact on any existing field, buffer, gate, or decision path. Zero change to `vel_peak_x/y/z`, `rms_x/y/z`, `vx/vy/vz`, or `peak` (`currentPeak`/`g_velPeakHold`).

## 4. Detailed Design

**4.1 Modbus read.** New transaction reading registers 0x3A–0x3C. Whether this is one `readHoldingRegisters(0x3A, 3)` call (mirroring the existing 3-consecutive-register `REG_FREQ_X` pattern) or three separate single-register reads (mirroring the VRMS pattern) depends on whether 0x3A/0x3B/0x3C are contiguous in the WTVB02 register map — this file already cites datasheet section numbers for other registers (e.g. §6.4.14–16 for VRMS/CF) but no such citation exists yet for 0x3A–0x3C. **Confirm against the datasheet before implementation** — do not assume contiguity from the register numbers alone. See Open Question 2.

**4.2 Scaling factor.** The historical comment (§2.2) states "÷100" for the old VX/VY/VZ reads. Per `CLAUDE.md`'s own rule ("Before Declaring Any Future Patch Complete — search every occurrence, verify all locations, do not assume"), this scaling factor should be re-confirmed against the datasheet directly, not solely trusted from a comment describing code that was later repointed to a different register entirely. See Open Question 3.

**4.3 New struct fields.** Three new `float` fields added to `VibrationData_t`, populated in `taskModbusRead()` in the same cycle as the existing VRMS reads. Exact names: pending Open Question 1 (must not collide with `vel_peak_x/y/z`).

**4.4 JSON publish.** Three new keys added to the existing `/sensor` payload builder (`StaticJsonDocument<960>`, `char buf[1000]`). Per the plan's own test plan, headroom must be confirmed using the existing truncation-warning self-check already present at that call site (`if (szSensor == 0 || szSensor >= sizeof(buf) - 1)`) — no new headroom-checking mechanism is needed.

**4.5 Motor-state gate (recommended, pending approval).** Per §2.4, apply the same pattern already used for `cf_x/y/z` (line ~5534: `(data->motor_state == 2) ? round(value * 100) / 100.0f : 0.0f`) to the three new Peak fields. This is a recommendation, not a silent addition — `Phase1_Implementation_Plan.md` is silent on it, and this document surfaces the conflict per project rule rather than resolving it unilaterally.

## 5. Non-Goals

- No fault-latch, deglitch, or alarm wiring (explicit in the governing plan).
- No change to `peak`, `vx/vy/vz`, `rms_x/y/z`, or `vel_peak_x/y/z` — existing fields, existing JSON keys, existing values, unchanged.
- No Modbus transaction consolidation (deferred to v17, out of scope per plan §1).
- No change to publish cadence, topic list, or any other payload builder (`/status`, `/vibration`, `/trend`).

## 6. Failure Mode (already fully specified by the governing plan — no open question here)

Per `Phase1_Implementation_Plan.md` §5:

```
If Peak register read fails
    │
    ▼
Disable Peak telemetry for this cycle
    │
    ▼
Continue VRMS/Temp/Frequency as normal
    │
    ▼
No reboot, no fault-latch entry triggered by this failure alone
```

This matches the existing pattern already used for the optional CF/Kurtosis transactions (line ~3366: "Optional — ถ้า fail ปล่อยค่าเดิม (0) ไม่กระทบ success หลัก") — precedent exists in-file for exactly this failure-isolation behavior; no new mechanism needs to be invented.

## 7. Open Questions (require explicit approval before implementation)

1. **Field/JSON-key naming.** What names for the three new fields avoid confusion with the existing, misleadingly-named `vel_peak_x/y/z`? (e.g. `peak_true_x`, `peak_raw_x` — final naming is the project owner's call, not this document's.) Renaming the legacy `vel_peak_x/y/z` fields themselves is explicitly **out of scope** for this DESIGN (smallest-change principle) but should be logged as separate housekeeping debt if the collision is judged confusing enough to warrant it later.
2. **Register contiguity.** Are 0x3A/0x3B/0x3C contiguous in the WTVB02 register map (supporting one 3-register read), or must they be read as three separate transactions? Needs datasheet confirmation, not assumption.
3. **Scaling factor.** Is ÷100 still correct for a fresh read of 0x3A–0x3C, independent of the historical comment? Needs datasheet re-confirmation.
4. **RUNNING-gate application.** Should §4.5's gate be applied? Recommended yes, per PATCH_NOTES_v16.5.md's unconditional rule — but flagged for explicit sign-off since the governing plan is silent.

## 8. Test Plan (for when implementation is approved)

Restated from the governing plan, plus additions from this review:
- Clean build, zero new warnings.
- MQTT capture on `/sensor` confirms the three new fields present, correctly typed, and every previously-existing field unchanged in name and value.
- No truncation warning across a normal RUNNING/STOPPED cycle.
- **New:** if the RUNNING-gate (Open Question 4) is approved, confirm the three new fields read exactly `0` while STOPPED/STARTING/STOPPING and normal values while RUNNING — mirroring the existing CF regression test in `PATCH_NOTES_v16.5.md`.
- **New (code-review check, not a runtime test):** confirm no code path anywhere reads the new fields under a name that could be confused with `vel_peak_x/y/z`.

## 9. Summary for Sign-Off

| Item | Status |
|---|---|
| Scope confirmed against governing plan | ✅ |
| Register-not-currently-read premise verified in source | ✅ Confirmed |
| Naming collision with legacy `vel_peak_x/y/z` identified | ✅ Flagged — Open Question 1 |
| RUNNING-gate requirement cross-checked against PATCH_NOTES_v16.5.md | ✅ Flagged — Open Question 4 |
| Register contiguity / scaling factor | ⚪ Needs datasheet confirmation — Open Questions 2–3 |
| Failure mode | ✅ Fully specified by governing plan, no open question |
| **Patch status** | **Draft — approval was never recorded, but implementation is already present in the current baseline (commit `4274e52`). See §10. Open Questions 1–4 below were not formally resolved via sign-off, though the actual implementation's choices are noted against each for traceability.** |

**Retrospective note against Open Questions 1–4** (observed from source, not a claim of approval): the actual implementation uses field names `peak_velocity_x/y/z` (avoids the `vel_peak_x/y/z` collision named in Open Question 1, though not using either example name suggested here); reads registers 0x3A–0x3C as one contiguous 3-register `readHoldingRegisters` call (Open Question 2, contiguity confirmed by this choice); uses a ÷100 scaling factor with an in-code citation of datasheet §6.4.6 (Open Question 3); and applies the `motor_state == 2` RUNNING-gate exactly as recommended in §4.5 (Open Question 4). These are factual observations of what was implemented, not evidence that the Open Questions were formally reviewed or signed off.

## 10. Provenance Note — Retrospective Reconciliation

Added during a documentation-reconciliation pass, not during original implementation. Records only established facts; invents no approval history.

- **Baseline commit:** `4274e524ea3568e693c3fa1e5068b7ce118599fb` ("feat(firmware): harden deglitch causal state propagation") is the first commit in which this repository's Git history contains the Peak Telemetry implementation described in §§1–6 above. The commit subject describes an unrelated, separately-closed diagnostic capability (a `VERIFY_TEST`-gated deglitch causal-proof mechanism); it does not mention Peak Telemetry, DESIGN-0004, or PD-0003. This document's implementation status is corrected here specifically because the commit message does not surface it.
- **Chronology:** the Peak Telemetry implementation was already present, uncommitted, in the working tree *before* the `VERIFY_TEST` deglitch investigation began. It was not introduced by that investigation and was not written during it.
- **Compiled/flashed baseline inclusion:** because the implementation is unconditional (not gated behind any preprocessor flag), it was part of every compile and every flash performed during the deglitch investigation and its subsequent production-restoration/sanity-check session, including the final production build that was flashed to the DUT and passed a bounded runtime sanity check.
- **Validation status:** no test, review, or capture during that session specifically targeted or validated Peak Telemetry's correctness. Its Modbus read transaction executed every poll cycle without producing a visible failure, and no crash, panic, or watchdog event was observed in that session's captures — but this is incidental presence during another feature's validation, not a dedicated validation of this feature. In particular, no captured `/sensor` MQTT payload was inspected to confirm the published `peak_velocity_x/y/z` values are numerically correct.
- **No technical defect established:** a provenance review conducted alongside this reconciliation found no evidence that the implementation is technically incorrect — only that it was implemented and committed without the formal approval this document's own gate requires. Technical correctness and approval status are tracked separately here; this note does not resolve either Open Questions 1–4 above or the Design Review status.
