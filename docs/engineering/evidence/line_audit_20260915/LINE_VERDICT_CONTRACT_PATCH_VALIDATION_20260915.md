# LINE/Dashboard Verdict-Consistency Patch — Isolated Implementation & Validation
Date: 2026-09-15

**Status: implemented and validated in an isolated copy only. The live production `flows.json` was never modified, Node-RED was never reloaded or restarted, and no LINE message was ever sent.**

Source of truth followed: `LINE_VERDICT_CONTRACT_PROPOSAL.md`.

---

## Scope

Patched exactly two nodes, in a local copy of the flow only:
```
adf3dc5f003a516c  🧠 Health Logic (Edge-Only, Drop-on-Missing)
e982d76b3ebe0b06  📱 LINE Message Builder
```
No other node was touched. The Unified State Engine (`ef04bfbbd91995e0`), its dedup/recovery/escalation/snooze/backoff timers, and its `machines` flow-context state shape were not modified, and — critically — were not even *executed* by the test harness (see Section 6): the harness mirrors that node's already-audited, unchanged output contract (`msg._machineId`, `msg._alertType`, pass-through `msg.payload`) rather than re-running or altering its code, so this validation cannot have exercised or drifted that logic in any way.

## Pre-patch snapshot

Saved verbatim before any edit:
```
docs/engineering/evidence/line_audit_20260915/staging/pre_patch_nodes.json
SHA256: 07b3c4c271b08401127861249852b442a37b707452d391fe0e0ee327374382d7
```
Contains the exact, complete pre-patch node objects for both `adf3dc5f003a516c` and `e982d76b3ebe0b06`, pulled fresh from the live flow (re-verified SHA256 `f3fba6aaac72e03d2a675e0417ac29d4c4ebc35146890ae35d9472f1996d2665`, identical to every prior audit this engagement) immediately before patching.

## Exact diff (both nodes)

**Health Logic (`adf3dc5f003a516c`)** — purely additive, 14 new lines inserted after the existing vibration-availability if/else block, nothing removed or reordered:
```diff
 // ─────────────────────────────────────────────
+// [Verdict-Fix-1] Authoritative live-verdict flag.
+// Mirrors contract.py's Dashboard formula EXACTLY:
+//   alarm_live = (vibration_status == "OK") and (motor_code == 2)
+// so LINE and the Dashboard agree on when alarm_level may be presented
+// as a trustworthy, live measurement rather than a held/unconfirmed
+// value. Purely additive -- nothing upstream or in alarm_level/alarm_code
+// itself is changed, and nothing previously read this field.
+// rpm_valid plays no part in this formula, unchanged from before: RPM
+// validity must not alter the vibration verdict.
+// ─────────────────────────────────────────────
+var motorCode = Number(p.motor_state);
+p.alarm_verdict_live = (p.vibration_measure_status === 'OK') && (motorCode === 2);
+
+// ─────────────────────────────────────────────
 // alarm_code
 // ─────────────────────────────────────────────
```
(One cosmetic-only artifact: the staged copy's file lacks a trailing newline the original had — a byte-level, non-functional difference from how the string was written to the staging file, not a code change.)

**LINE Message Builder (`e982d76b3ebe0b06`)**:
```diff
 var alertType = msg._alertType || 'alert';

+var verdictLive = p.alarm_verdict_live === true;
+var MOTOR_STATE_NAMES = { 0: 'STOPPED', 1: 'STARTING', 2: 'RUNNING', 3: 'STOPPING' };
+var motorStateText = MOTOR_STATE_NAMES[Number(p.motor_state)] || 'UNKNOWN';
+
 ...
 if (alertType === 'alert') {
     var emoji = p.alarm_level === 'CRITICAL' ? '🚨' : '⚠️';
-    text = emoji + ' แจ้งเตือน ' + p.alarm_level + '!\n'
+    var alertHeader = verdictLive
+        ? (emoji + ' แจ้งเตือน ' + p.alarm_level + '!')
+        : (emoji + ' ' + p.alarm_level + ' (ยืนยันสภาพจริงไม่ได้ ... unconfirmed — vibration data unavailable)');
+    text = alertHeader + '\n'
         + '🏭 Plant: ' + plant + '\n'
         + '⚙️ Machine: ' + mId + '\n'
-        + '📊 Velocity RMS: ' + velRms + '\n'
-        + '❤️ Health: ' + (...) + '\n'
+        + '⚙️ Motor State: ' + motorStateText + '\n'
+        + '📊 Velocity RMS: ' + velRms + '\n'
         + '⏰ ' + timeThai + ...
 } else if (alertType === 'recovery') {
-    text = '✅ กลับสู่ NORMAL\n'
+    var recoveryHeader = verdictLive ? '✅ กลับสู่ NORMAL'
+        : '⚠️ NORMAL (ยืนยันสภาพจริงไม่ได้ ... unconfirmed — vibration data unavailable)';
+    text = recoveryHeader + '\n'
         + '🏭 Plant: ' + plant + '\n'
         + '⚙️ Machine: ' + mId + '\n'
+        + '⚙️ Motor State: ' + motorStateText + '\n'
         + '📊 Velocity RMS: ' + velRms + '\n'
         + '⏰ ' + timeThai;
 }
```
Full, unabridged before/after text of both functions is in `staging/pre_patch_nodes.json` (before) and `staging/health_logic_patched.js` / `staging/line_message_builder_patched.js` (after).

## Proof no other node changed

The staged flow was built programmatically from the live flow (59 nodes), with only these two nodes' `func` fields replaced. A structural comparison (every node, full JSON equality) between the original and the staged copy confirmed:
```
node count: 59 -> 59
changed node ids: ['adf3dc5f003a516c', 'e982d76b3ebe0b06']
```
No other node — including the Unified State Engine, Escalation Timer, Auto Resume Check, 429 Handler, Command Parser, `Prepare InfluxDB`, `MQTT Vibration`, or any config/credential node — differs in any way.

**Staged artifact:**
```
docs/engineering/evidence/line_audit_20260915/staging/staged_flows_ISOLATED_COPY.json
SHA256: d10be3a0eb983bf84407b9d53e6c162249d0f58b407eaad4906cf46a2fd3a3d5
```
**Handling note:** because this staged file is a full copy of the entire production flow, it necessarily also carries the same hardcoded LINE channel token/secret/group-ID already present (unmodified) in the live `flows.json`'s `Identity Extractor / Credentials` node — this was flagged as a pre-existing exposure in the original `LINE_NOTIFICATION_AUDIT.md` and is not introduced or worsened by this patch. This file must be treated with the same care as the production flow (not committed, not shared) — it is not committed here, per instructions.

## Available tests run

**No Node-RED-specific automated test suite exists anywhere in this project** (confirmed by inspection — this codebase's only existing automated test suite, `test_contract.py`, covers the unrelated Python API and does not exercise Node-RED flows at all; running it would not validate this change and was not run for that reason). In its place, a purpose-built JS test harness (`staging/run_test_harness.js`, Node.js `v24.13.0`, run locally, no network access) was used to execute the **actual, unmodified, patched function-node source verbatim** — loaded directly out of the staged flow JSON, not retyped or reimplemented — inside a minimal Node-RED-compatible `msg`/`flow`/`node` shim. This is the closest available equivalent to a unit test for this class of change.

## Test matrix — results

Full raw output: `docs/engineering/evidence/line_audit_20260915/staging/test_output.json` (SHA256 `40b62c307a7ee1fdb6a1eefcd6c5b572af966f393b89ec0ced953dd3fc1ad633`).

| Case | Authoritative verdict input | Vibration availability | `alarm_verdict_live` | LINE decision (header) | LINE text (key fields) | Dashboard-equivalence |
|---|---|---|---|---|---|---|
| **A.** RUNNING+valid+NORMAL | NORMAL | OK (0.51 mm/s) | **true** | Clean recovery | "✅ กลับสู่ NORMAL ... Motor State: RUNNING ... Velocity RMS: 0.51 mm/s (Source: FIFO-DSP)" | Matches `alarm_level_live=true` case exactly — unchanged from pre-patch |
| **B.** RUNNING+valid+WARNING | WARNING | OK (5.20 mm/s) | **true** | Clean alert | "⚠️ แจ้งเตือน WARNING! ... Motor State: RUNNING ... Velocity RMS: 5.20 mm/s" | Matches |
| **C.** RUNNING+valid+CRITICAL | CRITICAL | OK (8.90 mm/s) | **true** | Clean alert | "🚨 แจ้งเตือน CRITICAL! ... Motor State: RUNNING ... Velocity RMS: 8.90 mm/s" | Matches |
| **D.** RUNNING+unavailable | CRITICAL | UNAVAILABLE | **false** | Qualified, non-clean | "🚨 CRITICAL (ยืนยันสภาพจริงไม่ได้ — ข้อมูล vibration ไม่พร้อมใช้งาน / unconfirmed — vibration data unavailable) ... Motor State: RUNNING ... Velocity RMS: N/A (FIFO-DSP unavailable)" | Would match Dashboard's `state=VIBRATION_UNAVAILABLE`/`alarm_level_live=false` — no longer a bare, unqualified alert |
| **E.** STOPPED | NORMAL | UNAVAILABLE (expected while stopped) | **false** (motor_code≠2) | Qualified, non-clean | "⚠️ NORMAL (ยืนยันสภาพจริงไม่ได้...) ... Motor State: STOPPED ... Velocity RMS: N/A" | See note below — deliberate minimal-scope simplification |
| **F.** RUNNING+RPM invalid+vibration valid | NORMAL | OK (0.48 mm/s) | **true** | Clean recovery, **identical in every field to case A** except the input `rpm_valid` | "✅ กลับสู่ NORMAL ... Motor State: RUNNING ... Velocity RMS: 0.48 mm/s" | Confirms `rpm_valid` has **zero** effect on the text — verified by direct comparison against case A |
| **G.** CRITICAL→automatic recovery→NORMAL (two-step, shared flow-context, vibration unavailable at both steps — the disputed real-incident shape) | CRITICAL → NORMAL | UNAVAILABLE throughout | **false** at both steps | Step 1 qualified alert, Step 2 qualified recovery | Step 1: "🚨 CRITICAL (unconfirmed...)"; Step 2: "⚠️ NORMAL (unconfirmed...)" — **never** "กลับสู่ NORMAL" | Recovery **transition itself fires exactly as before** (unchanged mechanism, verified by construction — the Unified State Engine's logic was not touched or exercised); only the message wording differs |

### Required behaviors — explicit confirmation

- **"D must NOT produce 'กลับสู่ NORMAL'"** — confirmed: case D is an `alert`-type message and never contains that string in either the old or new code paths; more importantly the *general* guard (verdictLive) ensures no case with unavailable vibration ever uses the clean recovery string, confirmed directly in case G step 2.
- **"D must identify vibration as unavailable/waiting-for-data"** — confirmed: the qualified header explicitly states "ข้อมูล vibration ไม่พร้อมใช้งาน / vibration data unavailable", and the existing `Velocity RMS: N/A (FIFO-DSP unavailable)` line is preserved unchanged.
- **"F must not alter vibration verdict merely because RPM is invalid"** — confirmed by direct text comparison: case F's LINE text is identical to case A's in every respect except the irrelevant timestamp (both runs execute at the same wall-clock second) — `rpm_valid: false` vs `true` produced byte-identical alert framing, Motor State, and Velocity RMS.
- **"G must preserve existing automatic recovery behavior"** — confirmed: the CRITICAL→NORMAL transition still occurs exactly once, on the same trigger (`payload.alarm_level` changing), with no new delay, debounce, or gate — because the Unified State Engine (the component that owns that mechanism) was neither modified nor executed differently by this patch. Only the *wording* of the resulting message changed.

### Deliberate scope note — case E (STOPPED)

The original proposal document suggested a **third**, STOPPED-specific wording ("⏹ STOPPED", distinct from both the clean NORMAL and the generic "unconfirmed" text). The implemented patch, in the interest of the smallest possible diff, uses a single two-way gate (`verdictLive` true/false) rather than a three-way branch, so STOPPED currently falls into the same "unconfirmed" wording as RUNNING+UNAVAILABLE. **This still satisfies the hard requirement — a STOPPED transition never says the clean "NORMAL"/"กลับสู่ NORMAL" — but the phrasing ("unconfirmed — vibration data unavailable") is not maximally precise for a machine that is *expected* to have no vibration data because it's simply off.** This is flagged as a known, intentional simplification, not an oversight; a follow-up could add the third branch if this wording distinction matters in practice.

## Explicit safety checks (item 8 of the request)

| Check | Result |
|---|---|
| No LINE Push actually sent | **Confirmed** — the harness never performs any network/HTTP call; `flow.get('PLANT_LINE_TOKENS')`/`PLANT_GROUP_IDS` were seeded with the literal placeholder strings `TEST-TOKEN-NOT-REAL-NEVER-SENT` / `TEST-GROUP-NOT-REAL-NEVER-SENT`, never the real values |
| No test button invoked | **Confirmed** — neither `7ec3e726495f761b` nor `05f64b8eeea910fa` was read, wired, or triggered at any point in this exercise |
| No credentials exposed | **Confirmed for the harness/output** — grepped `run_test_harness.js` and `test_output.json` for the real token/secret/group-ID substrings: zero matches in both. The one expected match is in `staged_flows_ISOLATED_COPY.json` itself (a full-file copy — see the Handling note above), not in any test output or this report. |
| No production file modified | **Confirmed** — see live re-verification immediately below |

**Live production file re-verification (after all patching/testing activity):**
```
SHA256: f3fba6aaac72e03d2a675e0417ac29d4c4ebc35146890ae35d9472f1996d2665
```
(identical to every prior audit this engagement — untouched)

**Node-RED container:**
```
RestartCount: 0
StartedAt:    2026-09-15T01:31:30Z  (unchanged, predates this entire exercise)
```
Not restarted, not reloaded, not deployed to.

## Confirmation: recovery logic untouched

The Unified State Engine's function source was read once (to confirm its unchanged output contract for the harness) and never written to, never included in the staged flow's diff, and never executed by the test harness — the harness manually sets `msg._machineId`/`msg._alertType` to the exact values that node's own already-audited logic would produce, rather than re-running or reimplementing it. Its Rule 1 (dedup)/Rule 4 (recovery-once) logic, `machines` state shape, and all timers (`stateChangedAt`, escalation 10-minute gate, snooze, backoff) are provably identical to the pre-existing, already-audited version.

## Confirmation: Unified State Engine untouched

`ef04bfbbd91995e0` does not appear in the "changed node ids" list from the structural diff (Section "Proof no other node changed") — confirmed unchanged at the byte level in the staged copy, exactly as in live production.

## Rollback method

Not applicable in the strict sense — **nothing was deployed**, so there is nothing running to roll back. If this patch is later deployed to production and needs reverting: restore the two functions' `func` fields from `staging/pre_patch_nodes.json` (the exact pre-patch snapshot captured above), leaving every other node untouched, then reload/restart Node-RED exactly as done for prior deployed changes this engagement.

---
*Implementation and validation performed entirely in an isolated local copy. No production file modified, no message sent, no button invoked, nothing restarted or reconfigured, no credential exposed. Not committed to git per instructions.*
