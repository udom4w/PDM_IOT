# PROMLOGIX LINE Notification Path — Audit
Date: 2026-09-15
Source of truth: live `flows.json` fetched fresh from `iotprom` (SHA256 `f3fba6aaac72e03d2a675e0417ac29d4c4ebc35146890ae35d9472f1996d2665` — the same, unchanged flow already verified in prior engagements this session). Read-only throughout: no file modified, no reload, no restart, no message sent, no credential rotated.

---

## 1. The exact flow — node IDs and pipeline

```
a375c54a616839e2  mqtt in    "📡 MQTT Vibration"            topic: factory/+/machine/+/vibration
 → 885a21b2d935bf2d  function  "🏭 Identity Extractor"
 → lat001calc2026fix1 function "⏱️ Latency Calculator"
 → adf3dc5f003a516c  function  "🧠 Health Logic (Edge-Only, Drop-on-Missing)"
 → ef04bfbbd91995e0  function  "🧠 Unified State Engine (Rule 1 & 4)"
 → e982d76b3ebe0b06  function  "📱 LINE Message Builder"
 → 830a5573f64cf991  http request "LINE Push API"
 → f63614552d4d191e  function  "🛡️ 429 Handler (Rule 3 — 5 min backoff)"
```
Supporting nodes:
- `cb07cceea16e874d` inject "⏱️ Every 1 min" → `f9378e1435e2c70e` "⏱️ Escalation Timer (Rule 2 — 10 min)" → into the same LINE Message Builder (`e982d76b3ebe0b06`)
- same 1-min tick → `c7fcf5f3a2ec18c2` "🔔 Auto Resume Check" → `77ae1c467feddef7` "LINE Push (Resume)"
- `d6b7d7d8e4736527` http in "📲 LINE Webhook POST /line/callback" → `8ee1165d481fc157` Signature Validator → `a23b228424fe6503` "🎛️ Command Parser (ack/snooze/status/reset/help)" → `68f05d08ca6e2bd8` LINE Reply API
- `9dd7345593be4c35` inject "🔧 Init on Deploy" → `f96b3d962be3abba` "⚙️ Identity Extractor / Credentials" (one-time flow-context setup, disconnected from the runtime path — `wires: [[]]`)
- **`7ec3e726495f761b` inject "🔴 Test CRITICAL (PUMP01)"** and **`05f64b8eeea910fa` inject "✅ Test NORMAL / Recovery (PUMP01)"** — both wired directly into `885a21b2d935bf2d` (Identity Extractor), i.e. **directly into the live production pipeline, not an isolated test path.** This is central to the finding below.

---

## 2–3. Trigger conditions and timers

**Health Logic** (`adf3dc5f003a516c`) drops the message entirely (returns `null`, nothing downstream fires) if `machine_id` is missing or `alarm_level` is not one of `NORMAL`/`WARNING`/`CRITICAL`. It does **not** gate on vibration availability — a message with `velocity_data_valid=false` (or absent) is passed through, only flagged as `vibration_measure_status = 'UNAVAILABLE'`.

**Unified State Engine** (`ef04bfbbd91995e0`) — the sole authority on CRITICAL/WARNING/NORMAL transitions:
```js
var machineId = msg.payload.machine_id || 'UNKNOWN';   // NOT uppercased
var newLevel  = msg.payload.alarm_level || 'NORMAL';
...
if (m.state === newLevel && !m.latched) return null;    // Rule 1: no dup alerts for same state
...
m.state = newLevel; m.stateChangedAt = now;
msg._alertType = (newLevel === 'NORMAL') ? 'recovery' : 'alert';
```
**The state transition is driven entirely by `payload.alarm_level` as received.** Nothing in this node — or anywhere upstream of it — checks `vibration_measure_status`/`velocity_data_valid` before accepting a CRITICAL→NORMAL (or any) transition. `alarm_level` and vibration-availability are two independently-computed fields with no cross-check between them anywhere in this pipeline.

**Timers found:**
- Backoff (`m.backoffUntil`): 5 minutes, set by the 429 Handler on a LINE API 429 response.
- Snooze (`m.snoozeUntil`): operator-set via `snooze <id> <minutes>` command, capped at 480 min (8h).
- Escalation (`f9378e1435e2c70e`, runs every 1 min via `cb07cceea16e874d`): fires once per CRITICAL episode after 10 continuous minutes in CRITICAL, gated on not-acknowledged/not-snoozed/not-backed-off.
- Auto-resume (`c7fcf5f3a2ec18c2`, same 1-min tick): clears an expired snooze and sends a resume push.

None of these timers impose any *minimum* dwell time on a CRITICAL state before a NORMAL transition is accepted — Rule 1 only suppresses a repeated identical state, never a rapid state *change*. **There is no debounce/hold-down on the CRITICAL→NORMAL edge at all.** This is exactly why two messages ~53 seconds apart is structurally possible: if two MQTT-shaped payloads carrying `alarm_level: "CRITICAL"` then `alarm_level: "NORMAL"` reach the Unified State Engine less than a minute apart (from any source — see Section 12), both transitions fire immediately.

## 4–6. Fields that determine severity / Health / Velocity RMS

| Displayed as | Field used | Node |
|---|---|---|
| Alert severity (CRITICAL/WARNING/NORMAL, emoji, `alertType`) | `payload.alarm_level` (verbatim, as received) | Unified State Engine |
| "❤️ Health: NN%" | `payload.health_score` (shown as `UNKNOWN` if undefined or negative) | LINE Message Builder |
| "📊 Velocity RMS" | `payload.velocity_data_valid === true && typeof payload.velocity_rms_overall === 'number'` → `velocity_rms_overall.toFixed(2)+' mm/s (Source: FIFO-DSP)'`, else the literal string `'N/A (FIFO-DSP unavailable)'` | LINE Message Builder, tagged `[Notify-Fix-1]` — this exact guard already prevents fabricating a number, but does **not** prevent sending the message at all |

**These three are computed completely independently.** Nothing forces `alarm_level` to be consistent with `velocity_data_valid`/`velocity_rms_overall` — a payload can legally carry `alarm_level: "CRITICAL"` (or `"NORMAL"`) together with no vibration data whatsoever, and every node downstream will process it as a normal, valid state transition.

## 7. Behavior when `vibration_status` is unavailable

Confirmed from Health Logic's own source: `velocity_data_valid` false/absent is explicitly treated as the *normal* steady state for a stopped machine and is **deliberately passed through, never dropped** (the code comment states dropping here "would silently delete every stopped-machine sample... a regression versus the legacy gate"). This is a correct design choice for the Dashboard/InfluxDB path. Its side effect on the **LINE** path, which was not separately considered, is that an unavailable-vibration message can still carry any `alarm_level` value and will still drive a full alert/recovery LINE push exactly as if vibration data were present.

## 8. Does LINE use the same severity/verdict as the Dashboard?

**No — they diverge in an important way.** The Dashboard/API's `contract.py` computes its own `alarm_level_live` flag specifically to prevent this exact class of problem:
```python
alarm_live = (vibration_status == "OK") and (motor_code == 2)
```
The Dashboard explicitly refuses to present `alarm_level` as a *live verdict* unless vibration is `OK` and the motor is RUNNING — otherwise it's presented as "state the firmware is HOLDING," not a fresh evaluation (see the API's own docstring, audited previously in this engagement). **The LINE Message Builder has no equivalent guard.** It uses `payload.alarm_level` directly with no `alarm_live`-style check, so LINE can announce a state transition the Dashboard itself would refuse to call "live."

## 9. Does RPM validity affect LINE in any way?

**No.** `rpm_valid` appears exactly once in the entire flow — inside `📊 Prepare InfluxDB (Edge-Only)` (`cc6ae17d3288db72`), which is downstream of and unrelated to the LINE branch. It is never read by Health Logic, the Unified State Engine, or the LINE Message Builder. RPM validity has zero influence on any LINE notification today.

## 10. Stale/legacy fields still in use

- **`🔔 Auto Resume Check` (`c7fcf5f3a2ec18c2`) reads `flow.get('LINE_TOKEN')` and `flow.get('LINE_GROUP_ID')`** — singular, legacy flow-context keys. The only init code in this flow (`⚙️ Identity Extractor / Credentials`, `f96b3d962be3abba`) sets `PLANT_LINE_TOKENS` / `PLANT_GROUP_IDS` (the multi-tenant v2 map used everywhere else) and never sets `LINE_TOKEN`/`LINE_GROUP_ID` at all. **This means every snooze-expiry auto-resume push currently builds a request with `Authorization: Bearer undefined` and `to: undefined`** — this is very likely silently failing today. Not exercised or confirmed live in this audit (no message was sent, per instructions), but the code path is unambiguous.
- **Legacy VRMS field names (`rms`, `vx`, `vy`, `vz`, `peak`)** are still used by the two test-injector payloads (Section 12) even though Health Logic's `[Phase0-R3]` comment states these are "no longer read anywhere in this node" — confirming the injectors were never updated to the current Product-1 payload shape.

## 11. Deduplication / cooldown / snooze / acknowledge logic

- **Dedup (Rule 1):** identical consecutive `alarm_level` for the same machine is dropped (`m.state === newLevel && !m.latched`).
- **Cooldown:** only the 429-triggered 5-minute backoff; no cooldown independent of a LINE API error.
- **Snooze:** `snooze <id> <min>` (command reply), capped at 8h; auto-expires via the 1-min timer (subject to the stale-token bug above).
- **Acknowledge:** `ack <id>` sets `acknowledged=true` and suppresses further *escalation* only — it does not suppress a fresh alert if the state changes again.
- **A significant, independently-confirmed defect:** the Unified State Engine keys its `machines` map by **`payload.machine_id` exactly as received, unmodified** (real production traffic sends this lowercase, e.g. `"pump01"`, confirmed via the topic parser `factory/plant01/machine/pump01/vibration`). The Command Parser, however, **forces the id to uppercase** before every lookup: `var mId = ackMatch[1].toUpperCase();`. The LINE alert message itself even instructs the operator to type the lowercase id it was stored under (`'  ack ' + mId`, where `mId` is the unmodified, lowercase stored key). **Following the exact instructions printed in the alert (`ack pump01`) looks up `machines["PUMP01"]`, not the real `machines["pump01"]` entry — silently creating a phantom, unrelated machine record instead of acknowledging the real one.** This was traced through the code and is a structural mismatch, not a hypothesis; it was not triggered live (no LINE command was sent, per instructions).

## 12. Root cause: why a NORMAL/recovery message can show "Velocity RMS: N/A" ~53 seconds after a CRITICAL alert

> **⚠️ CORRECTION — 2026-09-15 (later the same day), per `THRESHOLD_CONFIGURATION_RECONCILIATION_20260915.md`:**
> Point (a) immediately below states that `vibThresholdsConfigured()` "returns `false` unconditionally today" and that `VIB_WARNING_MMS`/`VIB_CRITICAL_MMS` "are documented as unset sentinels pending re-baselining." **This statement was factually incorrect at the moment it was written.** The current, and long-standing, production thresholds are `VIB_WARNING_MMS = 2.1 mm/s` and `VIB_CRITICAL_MMS = 4.5 mm/s` (with OFF-thresholds 1.9 / 4.2 mm/s and 2-capture escalation persistence), set by commit `c55853a2` on 2026-08-27 — 19 days before this audit was written — and confirmed still in effect in both the Production Source and the running firmware on `pump01` as of 2026-09-15. The error most likely arose from a stale code comment above `vibThresholdsConfigured()` that was never updated when the real values were promoted (full root-cause analysis in the reconciliation document). **This correction does not, by itself, overturn point (b) below** (the two manual test-inject buttons are a real, independently-confirmed finding, unaffected by this correction) — it only removes the false premise that a genuine firmware CRITICAL was impossible. See `LIVE_UAT_CRITICAL_EVENT_20260915.md` for a real, later CRITICAL event now interpreted in light of this correction: **this correction strengthens the case that a real CRITICAL/WARNING event can be, and in that instance most plausibly was, genuinely firmware-sourced**, rather than ruling that possibility out by default. The original text below is preserved unedited for provenance; read it together with this note, not in place of it.

Two independent facts combine to make this the almost-certain explanation:

**(a) Real firmware cannot currently produce a live CRITICAL verdict at all.** The firmware's own `vibThresholdsConfigured()` gate (audited earlier in this engagement) returns `false` unconditionally today — `VIB_WARNING_MMS`/`VIB_CRITICAL_MMS` are documented as unset sentinels pending re-baselining. So a genuine, live-firmware-sourced `alarm_level: "CRITICAL"` should not be occurring in production right now at all. *(See correction note above — this point was factually incorrect when written.)*

**(b) The flow contains two manual test-inject buttons wired directly into the live pipeline, and their payloads are exactly what would produce the observed symptom:**
```
7ec3e726495f761b  "🔴 Test CRITICAL (PUMP01)"
  { "rms":13.07, "vx":13.07, ..., "alarm_level":"CRITICAL", "health_score":0, ... }
  -- no velocity_data_valid, no velocity_rms_overall key at all --

05f64b8eeea910fa  "✅ Test NORMAL / Recovery (PUMP01)"
  { "rms":0.5, ..., "alarm_level":"NORMAL", "health_score":100, ... }
  -- same: no velocity_data_valid, no velocity_rms_overall --
```
Both use only the legacy VRMS shape and both **omit `velocity_data_valid`/`velocity_rms_overall` entirely.** Health Logic's own `LEGACY_FALLBACK` branch fires for exactly this shape (`velFlagPresent === false`), forcing `vibration_measure_status = 'UNAVAILABLE'` regardless of the injected `rms` value. That UNAVAILABLE status is exactly what the LINE Message Builder turns into `"N/A (FIFO-DSP unavailable)"`. Meanwhile `alarm_level` is taken verbatim from the injected payload, driving the CRITICAL alert and then the NORMAL recovery.

Both inject nodes have `repeat: ""`, `once: false`, `crontab: ""` — **they are pure manual-click buttons, not scheduled.** A ~53-second gap between the two is consistent with someone in the Node-RED editor clicking "🔴 Test CRITICAL (PUMP01)" and then, roughly a minute later, clicking "✅ Test NORMAL / Recovery (PUMP01)" — most likely while testing or demonstrating the alert flow, unaware that these buttons dispatch a **real** LINE push through the same credentials and group used for genuine alerts (Section 13 below flags that these credentials are, separately, hardcoded in the flow).

**This is the most probable, evidence-backed explanation for the operator's screenshot** — not a firmware defect, not a Dashboard defect, but two test-only trigger buttons left permanently wired into the production LINE-notification path.

---

## 13. Additional finding — credential exposure (flagged, not remediated, per instructions)

`⚙️ Identity Extractor / Credentials` (`f96b3d962be3abba`) contains, in plaintext, in the flow's own JSON: a LINE channel access token for `plant01`, a LINE group ID, and a LINE channel secret. This is stored directly in `flows.json` (not in `flows_cred.json`, Node-RED's separate encrypted-credential store) — meaning it is visible to anyone who can read the flow file or export it via the (now `/nr/`-auth-gated, per this session's earlier remediation) admin API. **Values are not reproduced in this report.** No credential was rotated, viewed further, or modified — flagged only, per your explicit instruction not to touch LINE credentials.

---

## State-Transition Table

| Case | Input state | LINE decision | Message | Expected/actual behavior |
|---|---|---|---|---|
| **A** | RUNNING, vibration valid, `alarm_level=NORMAL` | Rule 1 dedup unless state changed; if changed, `alertType='recovery'` | "✅ กลับสู่ NORMAL... Velocity RMS: X.XX mm/s (Source: FIFO-DSP)" | **Correct** — velocity shown because `velocity_data_valid=true` |
| **B** | RUNNING, vibration valid, `alarm_level=WARNING` | `alertType='alert'`, ⚠️ emoji | "⚠️ แจ้งเตือน WARNING!... Velocity RMS: X.XX mm/s" | **Correct** |
| **C** | RUNNING, vibration valid, `alarm_level=CRITICAL` | `alertType='alert'`, 🚨 emoji | "🚨 แจ้งเตือน CRITICAL!... Velocity RMS: X.XX mm/s" | **Correct**, but per Section 12(a) this combination cannot currently occur from real firmware (thresholds unset) — **⚠️ CORRECTED 2026-09-15: this caveat was based on the incorrect §12(a) premise; real firmware thresholds (2.1 / 4.5 mm/s) have been configured since 2026-08-27, so this combination CAN and did occur from real firmware — see `THRESHOLD_CONFIGURATION_RECONCILIATION_20260915.md` and `LIVE_UAT_CRITICAL_EVENT_20260915.md`** |
| **D** | RUNNING, vibration **unavailable** (`velocity_data_valid=false`) | Health Logic passes through (`UNAVAILABLE`), Unified State Engine still keys off whatever `alarm_level` was received | If `alarm_level` says CRITICAL/WARNING/NORMAL, LINE sends that alert/recovery text with **"Velocity RMS: N/A (FIFO-DSP unavailable)"** | **This is the defect path.** No check exists anywhere to withhold or flag the alert when the vibration evidence backing `alarm_level` is itself unavailable |
| **E** | STOPPED | Firmware typically reports `alarm_level=NORMAL` while stopped (per `contract.py`'s `alarm_live` gate, motor_code≠2 ⇒ not a live verdict); vibration naturally unavailable | If a transition occurs, same as case D's recovery text: NORMAL + "N/A" | **Likely intentional** for a genuinely stopped machine — distinguishable from case D only by `motor_state`, which the LINE Message Builder never reads or reports at all |
| **F** | CRITICAL → recovery (real) | `alertType='recovery'` once, per Rule 4/1 | "✅ กลับสู่ NORMAL... Velocity RMS: <value or N/A>" | Correct in shape; correctness of *content* entirely depends on whether the underlying `alarm_level`/vibration pairing was genuine (see Section 12) |
| **G** | WARNING → recovery | Same as F, `alertType='recovery'` | Same template, no WARNING-specific recovery text | Same caveat as F |
| **H** | RPM invalid, vibration valid | No effect — `rpm_valid` is never read by this pipeline | Unaffected; message identical to case A/B/C for the same `alarm_level` | **Confirmed by design/absence** — not a defect, simply out of scope today (Section 9) |

---

## Identified Defects / Inconsistencies (priority order)

1. **[Root cause] Test-inject nodes wired into the live production pipeline**, using legacy/incomplete payload shapes that trigger the exact "CRITICAL→NORMAL with vibration N/A" symptom, dispatched through real LINE credentials to the real recipient group.
2. **No cross-check between `alarm_level` and vibration availability** anywhere in the LINE path — structurally allows any alert/recovery text to be sent regardless of whether real vibration evidence exists, unlike the Dashboard's own `alarm_live` guard.
3. **No minimum dwell/debounce on the CRITICAL→NORMAL transition** — any two qualifying payloads under a minute apart will both fire.
4. **`machine_id` case mismatch** between the Unified State Engine (stores as-received, e.g. lowercase `pump01`) and the Command Parser (forces uppercase on lookup) — `ack`/`snooze` commands, including the exact command text the bot itself prints, silently fail to match real state.
5. **Stale flow-context keys** (`LINE_TOKEN`/`LINE_GROUP_ID`) used by Auto Resume Check, never set anywhere — snooze-expiry resume pushes are very likely broken.
6. **Hardcoded LINE credentials in `flows.json`** (not the encrypted credentials store) — a secrets-hygiene issue, flagged not remediated.
7. LINE messages never surface `motor_state`, so a genuinely-STOPPED recovery (case E) is textually indistinguishable from a fabricated-test recovery (case D) or a real-but-unavailable-vibration recovery.

## Recommended Minimal Fix (not implemented — planning only, per instructions)

- Move (or clearly isolate/disable) the two test-inject nodes so they no longer share a wire with the live MQTT ingestion path — e.g., route them into a separate, clearly-labeled test-only subflow that never reaches the Unified State Engine, or gate them behind a flow-context "test mode" flag that the production path checks and refuses to act on.
- In the Unified State Engine (or Health Logic just before it), add a guard analogous to the Dashboard's `alarm_live`: only accept an `alarm_level` transition into `alert`/`recovery` when `vibration_measure_status === 'OK'` (or explicitly allow-list the STOPPED case using `motor_state`, once that field is available to this flow), otherwise tag the message so the LINE Message Builder can either suppress it or clearly label it "based on held/unavailable data" rather than presenting it as a normal alert/recovery.
- Fix the `LINE_TOKEN`/`LINE_GROUP_ID` vs. `PLANT_LINE_TOKENS`/`PLANT_GROUP_IDS` mismatch in Auto Resume Check.
- Normalize case handling for `machine_id` consistently (either always uppercase at the point of first extraction, or never uppercase anywhere) so `ack`/`snooze` commands reliably match stored state.
- Move the LINE channel token/secret out of `flows.json` and into Node-RED's encrypted credentials store or an environment-backed secret.

## Expected Message Behavior After Fix

- A recovery/alert message would only ever be sent when backed by a genuine, live vibration evaluation (or an explicit, clearly-labeled STOPPED-state notice) — never from a manually-clicked test payload reaching real recipients.
- "Velocity RMS: N/A" would only appear on a message that also explicitly states *why* (e.g., "machine stopped" vs. "sensor data unavailable while running"), never silently alongside an unqualified "กลับสู่ NORMAL."
- `ack`/`snooze` typed exactly as instructed by the bot's own message would reliably match the real machine state.
- Snooze-expiry auto-resume notifications would actually send, using the correct multi-tenant credentials.

---
*Read-only audit. No file modified, no message sent, no credential viewed beyond confirming its presence, no reload/restart performed. Not committed to git per instructions.*
