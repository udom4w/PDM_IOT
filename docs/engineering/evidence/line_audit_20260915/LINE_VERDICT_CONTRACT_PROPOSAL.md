# LINE / Dashboard Verdict Consistency — Proposed Minimal Fix
Date: 2026-09-15 — Read-only planning document. Nothing in this document has been implemented.

Source of truth: live `flows.json` (unchanged since prior audits, SHA256 `f3fba6aaac72e03d2a675e0417ac29d4c4ebc35146890ae35d9472f1996d2665`), the current production `contract.py`, `LINE_NOTIFICATION_AUDIT.md`, and `AUTOMATIC_RECOVERY_FORENSIC_TRACE.md`.

---

## 1. Dashboard's authoritative verdict source

`contract.py`, `build_response()`:
```python
alarm_live = (vibration_status == "OK") and (motor_code == 2)
...
if not online:                       state = "DEVICE_OFFLINE"
elif motor_name is None:             state = "UNKNOWN"
elif motor_code == 2 and not vel_ok: state = "VIBRATION_UNAVAILABLE"
else:                                state = motor_name
```
The Dashboard never presents `alarm_level` as a trustworthy live verdict unless `vibration_status == "OK"` **and** the motor is confirmed `RUNNING` (`motor_code == 2`). Otherwise it reports a rollup state (`DEVICE_OFFLINE` / `UNKNOWN` / `VIBRATION_UNAVAILABLE` / the plain motor-state name) instead, and separately exposes `alarm_level_live: false` so a consumer can see the raw `alarm_level` is being *held*, not *evaluated*.

## 2. LINE's current verdict source

`ef04bfbbd91995e0` (Unified State Engine): `payload.alarm_level`, taken verbatim, with **no cross-check against vibration availability or motor state at all.**

## 3. Exact node where they diverge

**`ef04bfbbd91995e0` "🧠 Unified State Engine (Rule 1 & 4)"** is the single point of divergence — it is the LINE-side equivalent of `contract.py`'s `alarm_live` computation, but it performs no such computation. Its input, `adf3dc5f003a516c` "🧠 Health Logic," already computes everything needed (`vibration_measure_status`) but never combines it with `motor_state` into a live/held distinction, and never passes such a distinction downstream.

## 4. Can LINE currently produce these combinations?

| Combination | Possible today? | Evidence |
|---|---|---|
| NORMAL + `vibration_measure_status=UNAVAILABLE` | **Yes** | This is the confirmed 11:36:13 recovery message |
| WARNING + `vibration_measure_status=UNAVAILABLE` | **Yes** | Same code path — Health Logic never gates on `alarm_level`'s value, only on its presence/validity as one of the 3 enum strings |
| CRITICAL + `vibration_measure_status=UNAVAILABLE` | **Yes** | Same code path; also the most probable shape of the 11:35:20 alert itself, per the forensic trace |

All three are structurally identical in the code — nothing distinguishes them.

## 5. Correct precedence (proposed)

Ordered most‑authoritative/urgent first, modeled directly on the Dashboard's existing rollup so both systems agree:

```
1. CRITICAL   — only when alarm_live is true (vibration_status=="OK" AND motor RUNNING).
                A CRITICAL sourced from a non-vibration fault (e.g. a fault-latch/current
                condition) while vibration is unavailable is still a REAL event and must
                still alert — but must be labeled as unconfirmed-by-vibration, never as a
                clean vibration-backed CRITICAL.
2. WARNING    — same rule as CRITICAL.
3. VIBRATION UNAVAILABLE (while RUNNING) — must never be silently presented as NORMAL.
                This is its own distinct condition, not a variant of NORMAL.
4. STOPPED    — benign by definition; vibration unavailability while STOPPED is expected,
                not an alert condition, and is already handled correctly upstream (Health
                Logic passes it through without a warning-worthy status).
5. NORMAL     — only when alarm_live is true. This is the "all clear," and only this rung
                may ever use the reassuring "กลับสู่ NORMAL" framing.
```

## 6. Preserving automatic recovery — explicit constraint

**No change is proposed to the Unified State Engine's Rule 1 (dedup) or Rule 4 (recovery-sent-once) logic, its `machines` state storage, its escalation timer, or its snooze/backoff timers.** The forensic trace found the ~53-second CRITICAL→NORMAL gap to be a genuine, evidenced automatic recovery — there is no evidence any timing/debounce mechanism is wrong, so per your instruction none is touched. `m.state` continues to be set from raw `payload.alarm_level` exactly as today, so escalation-timer math (`stateChangedAt`, 10-minute escalation, etc.) is byte-for-byte unaffected. The fix proposed below is entirely about **what the LINE message says**, not **when or how often it is sent.**

## 7. Minimal change for verdict consistency

Add one new computed field in Health Logic (mirroring `contract.py`'s formula exactly) and consume it — plus `motor_state` — in the LINE Message Builder's text-composition step only. No new nodes, no rewiring, no change to the state engine.

```js
// In Health Logic (adf3dc5f003a516c), alongside the existing vibOk computation:
var motorCode = Number(p.motor_state);
p.alarm_verdict_live = (p.vibration_measure_status === 'OK') && (motorCode === 2);
```
This is the exact same boolean `contract.py` already computes for the Dashboard (`vibration_status=="OK" and motor_code==2`), now available on `msg.payload` for every downstream node — including the Unified State Engine (unchanged, ignores it) and the LINE Message Builder (which will use it).

## 8. Manual test buttons — isolation recommendation (not implemented)

Confirmed again in this pass: `7ec3e726495f761b` and `05f64b8eeea910fa` remain wired directly into `885a21b2d935bf2d` (the live ingestion path) and are **not** the cause of the 11:35→11:36 incident (per the forensic trace — no `LEGACY_FALLBACK` fingerprint anywhere in that window or in this container's history). They remain a separate, real defect: any future manual click still dispatches a genuine LINE push through real credentials.

**Recommended isolation (planning only):** rewire both inject nodes' output away from `885a21b2d935bf2d` and into a small dedicated "test preview" branch that runs the same Health Logic → Unified State Engine → LINE Message Builder chain but terminates at a **debug node** instead of the real `830a5573f64cf991` "LINE Push API" http-request node — so a developer can see exactly what a test payload *would* produce without it ever reaching a real recipient. Alternatively, gate `830a5573f64cf991` itself on a flow-context `TEST_MODE` flag the two inject nodes set on their `msg` (e.g. `msg._testOnly = true`) and have the 429-adjacent branch short-circuit before the real HTTP call when that flag is present. Either approach requires editing the flow and is out of scope for this read-only document.

## 9. LINE message field audit — exact source per field

| LINE field | Current source | Notes |
|---|---|---|
| "📊 Velocity RMS" | `p.velocity_data_valid === true && typeof p.velocity_rms_overall === 'number'` → the number, else `'N/A (FIFO-DSP unavailable)'` | Already correctly guarded (`[Notify-Fix-1]`) — not fabricated. Kept as-is. |
| "❤️ Health" | `p.health_score` (shown as `NN%`, or `'UNKNOWN'` if undefined/negative) | `health_score` is documented elsewhere in this codebase as returning the `HEALTH_SCORE_UNKNOWN(-1)` sentinel while vibration thresholds remain unconfigured — meaning this field is realistically **always** "UNKNOWN" in the current production configuration. It is not a real, currently-available metric. |
| vibration source/status | **Not separately surfaced at all today** — only implicit in whether Velocity RMS says a number or "N/A" | No explicit "source: fifo_dsp" or "status: OK/UNAVAILABLE" line exists in the LINE text currently |

## 10. Recommended wording semantics (as instructed)

- Replace the "❤️ Health: NN%" line with **"⚙️ Motor State: <RUNNING/STOPPED/STARTING/STOPPING>"**, sourced from `p.motor_state` (already present on every `/vibration` payload) mapped through the same `MOTOR_STATES` name table `contract.py` uses (`{0:"STOPPED",1:"STARTING",2:"RUNNING",3:"STOPPING"}`) — this is real, always-available data, unlike `health_score`.
- Never fabricate a health percentage. If a genuine health score becomes available again in the future (thresholds re-baselined), it can be added back as an *additional* line — not a replacement for Motor State.
- When `alarm_verdict_live` is `false`, the message text must not use the clean "กลับสู่ NORMAL" / plain "แจ้งเตือน" framing at all — see the alert/recovery template changes below.

---

## Proposed Contract

```
INPUT DATA                                    → AUTHORITATIVE VERDICT   → Dashboard presentation        → LINE presentation
───────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────
motor_state=STOPPED                           → (n/a, benign)           → state="STOPPED"               → no alert; if a
                                                                                                            transition message
                                                                                                            fires, label it
                                                                                                            "STOPPED", never
                                                                                                            "NORMAL"

motor_state=RUNNING, vibration OK,            → alarm_level (live)      → state=motor_name,             → clean alert/
alarm_level=NORMAL/WARNING/CRITICAL                                        alarm_level_live=true            recovery text,
                                                                                                            exactly as today

motor_state=RUNNING, vibration UNAVAILABLE    → VIBRATION_UNAVAILABLE   → state="VIBRATION_UNAVAILABLE", → distinct, labeled
(regardless of raw alarm_level)                 (overrides raw            alarm_level_live=false           text: alarm state
                                                 alarm_level as a                                           unconfirmed —
                                                 "clean" verdict)                                           vibration data
                                                                                                            unavailable

RPM invalid, vibration valid                  → unaffected (rpm_valid   → unaffected                    → unaffected —
                                                 plays no role here)                                       rpm_valid still
                                                                                                            never read by
                                                                                                            this pipeline
```

### State table

| Case | motor_state | vibration | alarm_level (raw) | alarm_verdict_live | LINE text framing |
|---|---|---|---|---|---|
| A. STOPPED | STOPPED | n/a (expected unavailable) | usually NORMAL/held | false (motor_code≠2) | No alert, or if state changes, "⏹ STOPPED" — never "NORMAL" |
| B. RUNNING + vibration NORMAL | RUNNING | OK | NORMAL | **true** | Unchanged: "✅ กลับสู่ NORMAL... Velocity RMS: X.XX mm/s" |
| C. RUNNING + vibration WARNING | RUNNING | OK | WARNING | **true** | Unchanged: "⚠️ แจ้งเตือน WARNING!... Velocity RMS: X.XX mm/s" |
| D. RUNNING + vibration CRITICAL | RUNNING | OK | CRITICAL | **true** | Unchanged: "🚨 แจ้งเตือน CRITICAL!... Velocity RMS: X.XX mm/s" |
| E. RUNNING + vibration UNAVAILABLE | RUNNING | UNAVAILABLE | any | **false** | New: "⚠️ [alarm_level] — ข้อมูล vibration ไม่พร้อมใช้งาน ยืนยันสภาพจริงไม่ได้ / Vibration data unavailable — condition unconfirmed. Velocity RMS: N/A (FIFO-DSP unavailable)" — never the clean "กลับสู่ NORMAL" wording |
| F. RUNNING + RPM invalid + vibration valid | RUNNING | OK | as reported | true (unaffected by rpm_valid) | Unchanged — `rpm_valid` plays no role in this pipeline today, confirmed by design |
| G. CRITICAL → automatic recovery NORMAL | RUNNING | **depends on evidence at that instant** | CRITICAL→NORMAL | **depends** — if vibration is confirmed OK at the recovery instant, case B applies (clean recovery); if not, case E applies (qualified "condition cleared" wording, no "NORMAL" claim) | Message content now correctly reflects which of B or E actually applied — timing/mechanism of the transition itself is unchanged either way |

---

## 1. Exact node IDs requiring modification

```
adf3dc5f003a516c  🧠 Health Logic (Edge-Only, Drop-on-Missing)   — add alarm_verdict_live computation
e982d76b3ebe0b06  📱 LINE Message Builder                        — consume alarm_verdict_live + motor_state,
                                                                     rewrite alert/recovery text branches,
                                                                     replace Health line with Motor State line
```
No other node requires modification. `ef04bfbbd91995e0` (Unified State Engine) needs **zero changes** — it already passes `msg.payload` through untouched, so the new field reaches the LINE Message Builder automatically.

## 2. Exact fields requiring modification

- **New field:** `payload.alarm_verdict_live` (boolean), set in Health Logic.
- **Read (already exists, newly consumed):** `payload.motor_state` (already published by firmware on every `/vibration` message, already reaches this flow, simply not read by the LINE Message Builder today).
- **Text template fields changed:** the `❤️ Health` line (removed, replaced) and the `alert`/`recovery` branches' opening framing (conditionally altered based on `alarm_verdict_live`).
- **Unchanged:** `payload.alarm_level` itself, `velRms` computation, `health_score` field (left alone in the payload — just no longer displayed), all `machines` state fields, all timer fields.

## 3. Minimal patch strategy

1. One added statement in Health Logic (2 lines) — purely additive, cannot affect any existing behavior since nothing currently reads `alarm_verdict_live`.
2. In the LINE Message Builder, wrap the existing `alert`/`recovery` text-building blocks with a check on `p.alarm_verdict_live`:
   - `true` → exactly today's text (zero change for the common, correct case — B/C/D above).
   - `false` → the new qualified wording (case E above), applied identically whether `alertType` is `'alert'` or `'recovery'`.
3. Replace the `❤️ Health: ...` line with `⚙️ Motor State: ...`, mapped from `p.motor_state` via the same 4-entry name table as `contract.py`.
4. No changes to `wires`, no new nodes, no changes to the Unified State Engine, Escalation Timer, Auto Resume Check, or 429 Handler.

## 4. What must NOT be changed

- The Unified State Engine's Rule 1 (dedup) / Rule 4 (recovery) logic, its `machines` state object shape, or its `stateChangedAt`/`lastEscalation`/`snoozeUntil`/`backoffUntil` semantics.
- The Escalation Timer's 10-minute threshold or its once-per-alarm-episode gate.
- The Auto Resume Check's snooze-expiry timing (its separate `LINE_TOKEN`/`LINE_GROUP_ID` bug identified in the prior audit is a distinct issue, out of scope here).
- The `[Notify-Fix-1]` Velocity RMS guard itself — it is already correct.
- `payload.alarm_level`, `payload.health_score`, or any Dashboard/API/firmware field or contract.
- The two test-inject nodes' wiring (addressed separately in Section 8 above, not implemented here).

## 5. Expected before/after LINE messages

**Before (today, case E — the actual 11:36:13 message shape):**
```
✅ กลับสู่ NORMAL
🏭 Plant: plant01
⚙️ Machine: pump01
📊 Velocity RMS: N/A (FIFO-DSP unavailable)
⏰ 15 ก.ย. 2569 11:36:13 น.
```

**After (proposed, same underlying data — case E):**
```
⚠️ NORMAL (ยืนยันสภาพจริงไม่ได้ / unconfirmed — vibration data unavailable)
🏭 Plant: plant01
⚙️ Machine: pump01
⚙️ Motor State: RUNNING
📊 Velocity RMS: N/A (FIFO-DSP unavailable)
⏰ 15 ก.ย. 2569 11:36:13 น.
```

**Before/after for case B (vibration genuinely OK) — unchanged:**
```
✅ กลับสู่ NORMAL
🏭 Plant: plant01
⚙️ Machine: pump01
⚙️ Motor State: RUNNING          ← only this line differs (was "❤️ Health: UNKNOWN")
📊 Velocity RMS: 0.27 mm/s (Source: FIFO-DSP)
⏰ ...
```

## 6. Test plan for production validation (to run once implemented — not run now)

1. Unit-style dry run inside the Node-RED editor: use the *isolated* test-preview branch (Section 8) — never the real inject nodes wired to production — with four synthetic payloads matching cases B, D, and E (vibration OK/NORMAL, vibration OK/CRITICAL, vibration UNAVAILABLE with any alarm_level) and inspect the debug-node output text for each, confirming: case B/D unchanged from today's wording; case E never contains the word "NORMAL" without the "unconfirmed" qualifier, and shows "⚙️ Motor State" not "❤️ Health".
2. Confirm via the live read-only Machine Detail API that `alarm_verdict_live` (Health-Logic-side) agrees with `alarm_level_live` (API-side) for the same live sample, by comparing a real `/vibration` capture against the API's response for the same machine/timestamp.
3. Re-run the existing `test_contract.py` suite (API-side) to confirm zero regression — this change touches only Node-RED, not the API, so this is a pure non-regression check.
4. Monitor Node-RED logs for one full day after deployment for any unexpected `LEGACY_FALLBACK`/`VIB_UNAVAILABLE` correlation with alert/recovery messages, confirming the new qualified wording appears exactly when expected and never for a genuinely-live verdict.

## 7. Rollback plan

Both changed nodes are plain `function` nodes — rollback is a straightforward revert of the two functions' `func` bodies to their current, audited content (already captured verbatim in `LINE_NOTIFICATION_AUDIT.md`), followed by the same deploy/activation mechanism already used for prior Node-RED changes this engagement (a container restart, since this session cannot deploy via the admin API without credentials). No data migration, no state-shape change, no InfluxDB/API impact — the `machines` flow-context object's shape is untouched, so a rollback mid-operation cannot corrupt in-flight escalation/snooze state.

---
*Read-only planning document. No file modified, no message sent, no button invoked, nothing restarted or reconfigured. Not committed to git per instructions.*
