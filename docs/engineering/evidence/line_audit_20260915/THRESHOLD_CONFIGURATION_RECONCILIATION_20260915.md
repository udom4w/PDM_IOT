# Vibration Alarm-Threshold Configuration — Reconciliation
Recorded: 2026-09-15

Read-only reconciliation only. No firmware, Node-RED, API, frontend, or configuration file was modified. Nothing was restarted, rebuilt, or flashed. No existing document was edited. Nothing committed.

**Evidence-strength labels used throughout, per instructions:**
- **DIRECTLY OBSERVED** — this session read the exact bytes/output itself, right now.
- **SOURCE VERIFIED** — confirmed via file content + matching SHA256/git provenance, not merely asserted.
- **INFERRED** — the most probable explanation given the evidence, not independently proven.
- **HISTORICAL/STALE** — a past statement that was true (or believed true) at an earlier point but does not describe the current state.

---

## 1. Current threshold source of truth

### 1a. Production Source (repo `.ino`) — SOURCE VERIFIED
File: `claude/WTVB02_ESP32S3_V16_CM_fault_latch_v3_hardened_patched_v16_5/WTVB02_ESP32S3_V16_CM_fault_latch_v3_hardened_patched_v16_5.ino`
```
#define VIB_WARNING_MMS       2.1f   // WARNING_ON
#define VIB_CRITICAL_MMS      4.5f   // CRITICAL_ON
#define VIB_WARNING_OFF_MMS   1.9f   // WARNING_OFF
#define VIB_CRITICAL_OFF_MMS  4.2f   // CRITICAL_OFF
#define VIB_ALARM_PERSIST_CAPTURES 2u
#define VIB_REFERENCE_HIGH_MMS 7.1f  // reference / high-severity, NOT a 4th state
```
`git blame` confirms these exact values (lines 777–803) were last set by commit `c55853a2` — **"fix(firmware): promote combined S21 current-freshness NTP production baseline"**, authored 2026-08-27 13:09:14 +07. No commit since has changed these numeric values (confirmed by `git log -S` pickaxe search across the file's full history — the only later hits touch surrounding comments or an OLED display call, never the `#define` values themselves).

`vibThresholdsConfigured()` (lines 4528–4532):
```c
static inline bool vibThresholdsConfigured() {
  return (VIB_WARNING_MMS  > 0.0f) &&
         (VIB_CRITICAL_MMS > 0.0f) &&
         (VIB_CRITICAL_MMS > VIB_WARNING_MMS);
}
```
Evaluated against the current values: `2.1 > 0` ✓, `4.5 > 0` ✓, `4.5 > 2.1` ✓ → **this function returns `true` today, and has since 2026-08-27.**

**A stale comment sits directly above this function** (lines 4522–4527, `git blame` commit `a1abd8b0`, 2026-08-22 — five days *before* the values were promoted to real numbers):
```
// [M1A] Are BOTH vibration thresholds explicitly configured and coherent?
// Until this returns true no vibration WARNING/CRITICAL decision may be made
// and no health score may be normalized. Both operands are currently the
// negative VIB_THRESHOLD_UNSET sentinel, so this returns false by construction
// -- it is not a runtime flag someone forgot to set, it is the documented
// state of the product pending re-baselining.
```
This comment was never updated when commit `c55853a2` replaced the sentinel with real values two days later. **It is factually stale and now contradicts the code immediately beneath it.** This is a documentation-only defect in the firmware source itself (a comment, not executable code) — flagged here, not fixed, per this task's read-only scope.

### 1b. Running Firmware — pump01 (NET-RECOVERY-V2) — SOURCE VERIFIED
Per `claude/docs/FIRMWARE_SOURCE_OF_TRUTH.md`, the firmware currently flashed on `pump01` is NET-RECOVERY-V2, built from worktree `PDM_IOT_worktree_autorecovery_v2`, base commit `249e087` (2026-09-12), flashed 2026-09-13 19:05:43 +07, documented Source SHA256 `f945cf9cca2cf7025e5036f6dbdd06eb6a874fe4d6aeee24feaa6e3aa3c17744`.

This session located that worktree still on disk and read (never modified) its `.ino`:
```
sha256sum PDM_IOT_worktree_autorecovery_v2/.../WTVB02_ESP32S3_V16_CM_fault_latch_v3_hardened_patched_v16_5.ino
→ f945cf9cca2cf7025e5036f6dbdd06eb6a874fe4d6aeee24feaa6e3aa3c17744   (EXACT MATCH to the documented Source SHA256)
```
Its threshold defines, read directly from this exact-hash-matched file:
```
#define VIB_WARNING_MMS       2.1f
#define VIB_CRITICAL_MMS      4.5f
#define VIB_WARNING_OFF_MMS   1.9f
#define VIB_CRITICAL_OFF_MMS  4.2f
#define VIB_ALARM_PERSIST_CAPTURES 2u
```
**Identical to the Production Source.** This is expected and explained: `git merge-base --is-ancestor c55853a2 249e087` confirms the threshold-promotion commit (2026-08-27) is an ancestor of the running firmware's base commit (2026-09-12), and the NET-RECOVERY-V2 patch on top is documented as a single-file, network-recovery-only change (472 insertions/1 deletion) — nothing about vibration thresholds.

**On the compiled binary itself:** this session did **not** disassemble or extract constants from the actual flashed `.bin` (out of scope — "do not flash or rebuild," and binary extraction was not attempted). The claim that the *running binary* contains these values is therefore **SOURCE VERIFIED via matching SHA256 of the exact source that was built**, not **DIRECTLY OBSERVED** in the binary itself. Since `#define` is a compile-time textual substitution with no `#ifdef`/build-flag override found anywhere in this file for these four constants, source-level verification is sound evidence for the binary's behavior, but the distinction is recorded here as instructed.

### 1c. Node-RED — SOURCE VERIFIED, does not independently gate on these thresholds
Health Logic and LINE Message Builder (`docs/engineering/evidence/line_audit_20260915/staging/*_patched.js`, `pre_patch_nodes.json`) take `alarm_level` verbatim from the MQTT payload firmware publishes; neither node recomputes NORMAL/WARNING/CRITICAL from a raw RMS number using its own thresholds. **One unrelated, pre-existing field was found and is worth flagging separately (§6c below):** Health Logic sets `p.thresholds = { warning: 4.5, critical: 7.1 }`, explicitly commented `// These are NOT Product-1 vibration alarm thresholds`. This field is not read anywhere in the LINE Message Builder (grep confirms zero references) and was present, byte-for-byte, before the LINE verdict-consistency patch (confirmed against `pre_patch_nodes.json`) — it is untouched legacy code, not part of the actual alarm decision.

### 1d. API — DIRECTLY OBSERVED, does not expose thresholds
Live query `GET https://dash.promlogix.com/api/machine/plant01/pump01` (this session, today) returns `vibration.alarm_level`, `alarm_level_live`, `velocity_rms_overall_mms`, etc. — no threshold field of any kind appears anywhere in the response. The API surfaces only the *result* of the firmware's threshold comparison, never the threshold values themselves.

### 1e. Dashboard — DIRECTLY OBSERVED (read-only, via SSH), independently hardcodes the same values
`/opt/iot-stack/frontend/app.js` (VPS, not in git, per this engagement's existing note that the Dashboard frontend lives only on the VPS) contains, for chart-shading/labeling purposes only:
```js
var VIB_WARNING_MMS = 2.1;
var VIB_CRITICAL_MMS = 4.5;
```
own comment: `// (VIB_WARNING_MMS/VIB_CRITICAL_MMS) purely to shade/label the chart's ... display-only, not a new threshold`. **This is a manually-synced mirror, not a live fetch** — the Dashboard does not query firmware or the API for the threshold values; someone hardcoded the same numbers here separately. Checked the file's own backup history: `app.js.bak_20260907_234100` (2026-09-07 23:41, the oldest available backup) **already contains the identical constants** — i.e., this mirror has existed on the Dashboard for at least 8 days before today.

---

## 2. Historical threshold state — timeline

| Date | Event | Evidence type |
|---|---|---|
| 2026-08-22 21:21 +07 | Commit `a1abd8b0` introduces `vibThresholdsConfigured()` **and** its now-stale comment, at a time when the sentinel actually was unset (per the comment's own words, credible for that date) | SOURCE VERIFIED (git blame) |
| 2026-08-27 13:09 +07 | Commit `c55853a2` ("promote combined S21 current-freshness NTP production baseline") sets `VIB_WARNING_MMS=2.1f`, `VIB_CRITICAL_MMS=4.5f`, OFF-thresholds, and persistence count to real, non-sentinel values. **Comment above `vibThresholdsConfigured()` not updated.** | SOURCE VERIFIED (git blame, `git log -S`) |
| 2026-08-27 (same day) | `claude/docs/evidence/PRODUCT1_E2E_VALIDATION_RECORD_Pump01_20260827.md` documents these exact values as the **canonical, tested (22/22 PASS)** configuration, explicitly citing `c55853a2` as "canonical promotion commit," source SHA256 `827137c2...` | SOURCE VERIFIED (direct quote, matches git blame commit) |
| ~2026-09-07 (on or before) | Dashboard `app.js` already carries the identical `VIB_WARNING_MMS=2.1` / `VIB_CRITICAL_MMS=4.5` mirror (oldest available backup, 2026-09-07 23:41) | DIRECTLY OBSERVED (SSH, file content) |
| 2026-09-12 13:56 +07 | Commit `249e087` (base for the currently-running firmware) — descendant of `c55853a2`, thresholds already included | SOURCE VERIFIED (`git merge-base --is-ancestor`) |
| 2026-09-13 19:05:43 +07 | NET-RECOVERY-V2 flashed to `pump01`, source SHA256-matched to the worktree file containing the same threshold values | SOURCE VERIFIED (SHA256 match) |
| 2026-09-15, earlier today | `LINE_NOTIFICATION_AUDIT.md` §12(a) states `vibThresholdsConfigured()` "returns `false` unconditionally today" and that the thresholds "are documented as unset sentinels pending re-baselining" | **This statement was already incorrect at the moment it was written** — the values had been real since 2026-08-27, 19 days earlier |
| 2026-09-15, later today | `LIVE_UAT_CRITICAL_EVENT_20260915.md` independently re-discovers the real, current values via direct source inspection and flags the contradiction with the earlier same-day audit | SOURCE VERIFIED |
| 2026-09-15, this document | Full reconciliation performed | this document |

**Conclusion: at no point covered by this engagement's evidence were these thresholds genuinely "unset" in the sense the 2026-09-15 audit meant it.** They were unset only before 2026-08-27, which is outside the window any document in this engagement discusses. The 2026-09-15 audit's claim was **incorrect for the entire period it was describing ("today")** — not stale-but-formerly-true, simply mistaken about present-tense fact.

---

## 3. Root-cause hypothesis for the incorrect audit statement — INFERRED

`LINE_NOTIFICATION_AUDIT.md` §12(a)'s exact phrase — *"`VIB_WARNING_MMS`/`VIB_CRITICAL_MMS` are documented as unset sentinels pending re-baselining"* — is a near-verbatim paraphrase of the **stale comment** text above `vibThresholdsConfigured()` ("Both operands are currently the negative VIB_THRESHOLD_UNSET sentinel... it is the documented state of the product pending re-baselining"), not of the executable `#define` lines immediately below it, which have read `2.1f`/`4.5f` since 2026-08-27.

**This session cannot directly inspect the reasoning process that produced the earlier audit** — this is stated as the most probable explanation (INFERRED), not a confirmed fact about what happened in that turn. It is offered because the wording match is very close and the comment is independently confirmed stale (§1a), which is sufficient to explain the error without assuming any other cause (e.g., it does not require assuming a different threshold mechanism was confused for this one — a check was made for that alternative explanation too: `claude/docs/evidence/PRODUCT1_E2E_VALIDATION_RECORD_Pump01_20260827.md` documents a **separate, genuinely-still-unset** threshold, `VIB_TTW_MIN_SLOPE` (Time-To-Warning predictive slope, in `vib_ttw.h/.cpp`, explicitly "NOT part of the current live alarm decision"). The 2026-09-15 audit's text names `vibThresholdsConfigured()`/`VIB_WARNING_MMS`/`VIB_CRITICAL_MMS` specifically, not TTW, so this alternative explanation is less well-supported than the stale-comment hypothesis, but is recorded here since it was checked).

---

## 4. Exact affected documents

| Document | Exact statement | What it actually referred to |
|---|---|---|
| `docs/engineering/evidence/line_audit_20260915/LINE_NOTIFICATION_AUDIT.md` §12(a) | "Real firmware cannot currently produce a live CRITICAL verdict at all... `vibThresholdsConfigured()` gate... returns `false` unconditionally today" | **(f) An incorrect prior observation.** Not a historical-state description, not a different branch — a present-tense factual claim about the same Production Source this reconciliation checked, and it was wrong at time of writing. |
| `docs/engineering/evidence/line_audit_20260915/LINE_NOTIFICATION_AUDIT.md` §12, state-transition table, row **C** | "per Section 12(a) this combination cannot currently occur from real firmware (thresholds unset)" | Same root cause as above — inherits §12(a)'s error. |
| `docs/engineering/evidence/line_audit_20260915/AUTOMATIC_RECOVERY_FORENSIC_TRACE.md` §4–5 | "Firmware currently cannot produce a live CRITICAL verdict from the vibration-RMS threshold path at all... established in an earlier audit this engagement" | **(f)** Inherits the same error by explicit citation of §12(a); does not independently re-derive it. |
| `claude/docs/engineering/PHASE1_AUTOMATIC_MODEM_RECOVERY_V2_FINAL_STATIC_GATE_20260913.md` (N-4) | "`0` used as the unset sentinel for two **timestamps**" | **(e) A different threshold/sentinel mechanism entirely** — this is about timestamp fields in the modem-recovery state machine, unrelated to vibration RMS thresholds. Not affected, not in scope, mentioned only because it matched the grep for "unset sentinel." |
| `claude/docs/evidence/PRODUCT1_E2E_VALIDATION_RECORD_Pump01_20260827.md` | Documents `VIB_WARNING_MMS=2.1`/`VIB_CRITICAL_MMS=4.5` as the live, tested, canonical values; separately documents `VIB_TTW_MIN_SLOPE` as genuinely unset (a **different** threshold mechanism, not part of the live alarm decision) | **This document is correct and is the authoritative confirming record** — no correction needed. |
| `docs/engineering/evidence/line_audit_20260915/LIVE_UAT_CRITICAL_EVENT_20260915.md` §2d | Already correctly identified the discrepancy and flagged it for this reconciliation | Correct as written; this document supersedes it with the full root-cause chain. |

**No document needs correction due to (b) Dashboard configuration, (c) a historical state that was true when written, or (d) a different branch/build** — the erroneous documents (`LINE_NOTIFICATION_AUDIT.md`, `AUTOMATIC_RECOVERY_FORENSIC_TRACE.md`) were describing the same Production Source, at the same time, that this reconciliation checked, and were simply incorrect (category **f**).

---

## 5. Does any existing engineering record need correction?

**Yes — two records contain a factual error that should eventually be corrected, but per this task's explicit instruction, nothing has been rewritten yet:**
1. `LINE_NOTIFICATION_AUDIT.md` §12(a) and the Case-C row of its state-transition table.
2. `AUTOMATIC_RECOVERY_FORENSIC_TRACE.md` §4–5 (inherits the error by citation).

Both are evidence-directory records of what was believed and concluded *at the time*, not living specifications — this reconciliation does not silently alter them. See §7 for the recommended correction approach.

**No correction is needed** for `PRODUCT1_E2E_VALIDATION_RECORD_Pump01_20260827.md` (already correct) or `PHASE1_AUTOMATIC_MODEM_RECOVERY_V2_FINAL_STATIC_GATE_20260913.md` (unrelated mechanism, correctly scoped to timestamps).

---

## 6. Impact on the LIVE CRITICAL/WARNING UAT interpretation

### 6a. Effect on `LIVE_UAT_WARNING_EVENT_20260915.md` (the earlier WARNING event)
No change to its conclusion. That document's classification (`OBSERVED PASS`) did not rely on whether thresholds were configured — it verified verdict consistency and wording, which hold regardless.

### 6b. Effect on `LIVE_UAT_CRITICAL_EVENT_20260915.md` (the CRITICAL event)
**Strengthens, does not weaken, its conclusion.** That document's own §9 already declined to assert the event was firmware-genuine with certainty, citing (among other things) uncertainty about whether real thresholds could produce it. This reconciliation now **confirms with SOURCE VERIFIED-strength evidence** (not merely "the source *currently* looks configured," but a full, dated provenance chain back to 2026-08-27, independently corroborated by a same-day validation record and by the Dashboard's own 2026-09-07 mirror) that a genuine firmware-sourced CRITICAL verdict has been structurally possible for at least 19 days before this event, and was already the tested, canonical configuration, not an emergent or accidental state. This makes the "genuine, firmware-sourced physical event" reading of that CRITICAL event **more likely**, though — as already stated in that document — still not proven with certainty absent a definitive click-log or payload capture.

### 6c. New, unrelated observation surfaced during this reconciliation
Node-RED's Health Logic sets a legacy `p.thresholds = { warning: 4.5, critical: 7.1 }` field on every payload, explicitly commented as **not** the real Product-1 alarm thresholds. Its "warning" value (4.5) numerically coincides with firmware's real `VIB_CRITICAL_MMS` (also 4.5) — a confusing coincidence, not a wiring bug, since this field is not read by the LINE Message Builder or by anything in the alarm-decision path (confirmed by grep and by its presence, unchanged, in `pre_patch_nodes.json` from before the LINE patch). No impact on any UAT conclusion. Flagged as a documentation clarity item only (§7).

---

## 7. Recommended documentation-only corrections (not performed — awaiting authorization)

1. **`LINE_NOTIFICATION_AUDIT.md` §12(a):** add a dated correction note (not a silent rewrite) stating that the "thresholds unset" claim was incorrect at time of writing, with a pointer to this reconciliation document and to `c55853a2`/`PRODUCT1_E2E_VALIDATION_RECORD_Pump01_20260827.md` as the evidence. The Case-C row's "cannot currently occur from real firmware" caveat should be struck or annotated as superseded, since a real firmware CRITICAL is in fact possible and was independently confirmed to have occurred (`LIVE_UAT_CRITICAL_EVENT_20260915.md`).
2. **`AUTOMATIC_RECOVERY_FORENSIC_TRACE.md` §4–5:** add a similar dated correction note, since its conclusion about the 11:35:20 event's alarm source rested partly on the same incorrect premise. (Note: this does not necessarily overturn that event's conclusion — the fault-latch/current-sensor hypothesis there may still be correct for independent reasons — but the "vibration-RMS-threshold path is currently impossible" reasoning it partly relied on should no longer be cited as true.)
3. **Firmware source comment (lines 4522–4527 of the `.ino`):** the comment above `vibThresholdsConfigured()` is stale and should eventually be updated to reflect that thresholds have been configured since `c55853a2` (2026-08-27) — this is a code-comment change, not a behavior change, but is a firmware-source edit and is explicitly **not** performed here per this task's read-only scope.
4. **Node-RED Health Logic's `p.thresholds` field (§6c):** consider renaming or re-commenting to avoid the numeric coincidence with `VIB_CRITICAL_MMS`, purely for future-reader clarity — no functional issue exists today.

None of these four items were applied. All are offered as recommendations for a future, explicitly-authorized documentation/comment-correction task.

---

## 8. Scope confirmation

- No firmware, Node-RED flow, API, frontend file, or configuration was modified.
- Nothing was restarted, rebuilt, or flashed.
- No existing document was edited.
- All firmware/frontend reads were read-only (`grep`, `Read`, `sha256sum`, SSH read commands only).
- The historical NET-RECOVERY-V2 worktree was read-only inspected, never built or modified.
- This document has not been committed, per instructions.
- No LINE credential or token value is reproduced anywhere in this document.

---
*Read-only reconciliation. No production code, configuration, or existing evidence document was modified in producing this file.*
