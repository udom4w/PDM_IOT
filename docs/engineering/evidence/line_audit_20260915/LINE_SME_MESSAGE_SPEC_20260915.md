# PROMLOGIX LINE Notification — Final SME-Facing Message Specification
Recorded: 2026-09-15

**Type: DESIGN SPECIFICATION ONLY.** No Node-RED flow, firmware, API, Dashboard/frontend, nginx, MQTT, or credential was read-write-touched, modified, restarted, or sent to. This document proposes exact wording for a future, separately-authorized, wording-only patch. Nothing here has been implemented.

**Source of truth used:** the currently-deployed LINE Message Builder logic (`docs/engineering/evidence/line_audit_20260915/staging/line_message_builder_patched.js` and `health_logic_patched.js`, matching what is live in production per `LINE_VERDICT_PRODUCTION_DEPLOYMENT_20260915.md`), plus every evidence document listed in the task, read but not modified.

---

## 1. SME communication principle

An SME (small/medium enterprise) operator is not a vibration analyst. Every message must let them answer three questions, **in this fixed order, without needing to interpret raw numbers or implementation terms**:

1. **เกิดอะไรขึ้น?** (What happened?) — one plain-language sentence naming the condition and, where relevant, *why* it is being reported this way (e.g., "sensor data is not available" vs. "a threshold was crossed").
2. **ตอนนี้เป็นอย่างไร?** (What is the situation right now?) — the current measured value (when one genuinely exists) and the machine's run status, side by side, so the operator can see the two facts the Dashboard also shows without having to open it.
3. **ควรทำอะไร?** (What should be done?) — exactly one concrete, proportionate instruction: nothing / monitor / inspect / inspect urgently. Never more than one recommended action, never a vague "please check the system."

**Design consequence:** the template is fixed across all five states so the operator learns the shape once and only reads the changing content. Only wording changes between states — never the *order* of the three questions, never the underlying severity classification (see §7).

---

## 2. Final standard template

```
[status title with emoji]

🏭 โรงงาน: {plant}
⚙️ เครื่อง: {machine_id}

🔎 เกิดอะไรขึ้น:
{one or two plain-language sentences}

📌 ตอนนี้:
ค่าการสั่นสะเทือน: {value or "ไม่มีข้อมูล"}
สถานะเครื่อง: {กำลังทำงาน / หยุดทำงาน / เริ่มทำงาน / กำลังหยุด}

🔧 ควรทำอะไร:
{one concrete instruction}

⏰ {Thai local timestamp}
{command block — alert-type messages only, omitted on recovery}
```

Notes on the template itself:
- **Motor-state line always present**, in Thai, using the requested vocabulary (กำลังทำงาน/หยุดทำงาน/etc.) instead of the current English `Motor State: RUNNING`. This directly satisfies the "avoid 'Health: RUNNING'"-class instruction while keeping the *field* (motor state) that the prior patch already added.
- **"ค่าการสั่นสะเทือน" replaces "Velocity RMS"** as the user-facing label. The engineering term and unit (`mm/s`) are kept next to the number for anyone who forwards the message to a technician, but the label itself is plain Thai.
- **No "(Source: FIFO-DSP)" or "(FIFO-DSP unavailable)" in user-facing text** — this is an implementation detail (which internal computation pipeline produced the number), not something an SME needs to evaluate the message. See §6.
- **The command block (`ack`/`snooze`) is preserved verbatim** where it exists today (alert-type messages) — this is operational, not explanatory, text, and changing it is out of scope for a wording-only patch.

---

## 3. Final wording, states A–E

Each entry gives the exact title/body/action text, which payload fields it reads (all already present on the message today — no new upstream field required, see §8), which fields are intentionally left out, and the semantic reasoning tying the wording back to the authoritative verdict fields (`p.alarm_level`, `p.alarm_verdict_live`, `p.motor_state`, `p.vibration_measure_status`).

### A. WARNING (`alarm_level=WARNING`, `alarm_verdict_live=true`)

```
⚠️ แจ้งเตือน: พบการสั่นสะเทือนสูงกว่าปกติ

🏭 โรงงาน: plant01
⚙️ เครื่อง: PUMP01

🔎 เกิดอะไรขึ้น:
ระบบตรวจพบค่าการสั่นสะเทือนของเครื่องนี้สูงกว่าระดับปกติ ขณะที่เครื่องกำลังทำงานอยู่

📌 ตอนนี้:
ค่าการสั่นสะเทือน: 2.25 mm/s
สถานะเครื่อง: กำลังทำงาน

🔧 ควรทำอะไร:
เฝ้าระวังอย่างใกล้ชิด แนะนำให้ตรวจสอบเครื่องในรอบตรวจถัดไป

⏰ 15 ก.ย. 2569 12:36 น.

📖 คำสั่ง:
  ack PUMP01
  snooze PUMP01 30
```
- **Fields shown:** plant, machine id, motor state (กำลังทำงาน), measured vibration value + unit, timestamp, ack/snooze commands.
- **Fields intentionally omitted:** `alarm_code` (0/1/2 integer — internal), `vibration_measure_source`/FIFO-DSP label, raw threshold numbers (2.1/2.1-off — see §8's threshold decision below).
- **Semantic reason:** `alarm_verdict_live=true` means Health Logic has already confirmed vibration data is genuinely valid *and* the motor is confirmed RUNNING — this is exactly the condition under which a WARNING claim is trustworthy. The wording states this as a fact ("ตรวจพบ... สูงกว่าระดับปกติ"), not a possibility, because the verdict is live.

### B. CRITICAL (`alarm_level=CRITICAL`, `alarm_verdict_live=true`)

```
🚨 แจ้งเตือน: พบการสั่นสะเทือนระดับวิกฤต

🏭 โรงงาน: plant01
⚙️ เครื่อง: PUMP01

🔎 เกิดอะไรขึ้น:
ระบบตรวจพบค่าการสั่นสะเทือนของเครื่องนี้สูงถึงระดับวิกฤต ซึ่งอาจเป็นสัญญาณของความเสียหายที่กำลังเกิดขึ้น

📌 ตอนนี้:
ค่าการสั่นสะเทือน: 4.68 mm/s
สถานะเครื่อง: กำลังทำงาน

🔧 ควรทำอะไร:
ตรวจสอบเครื่องโดยด่วน หากเป็นไปได้ให้พิจารณาหยุดเครื่องเพื่อตรวจสอบก่อนเดินเครื่องต่อ

⏰ 15 ก.ย. 2569 12:58 น.

📖 คำสั่ง:
  ack PUMP01
  snooze PUMP01 30
```
- **Fields shown:** same set as WARNING.
- **Fields intentionally omitted:** same as WARNING, plus the escalation timer's internal duration counter (that belongs only to the separate `escalation` message type, unchanged by this spec).
- **Semantic reason:** identical live-verdict gating logic as WARNING, escalated wording and action ("ตรวจสอบโดยด่วน... พิจารณาหยุดเครื่อง") proportionate to CRITICAL. The real observed range for this state (`LIVE_UAT_CRITICAL_EVENT_20260915.md`) was 4.68–6.93 mm/s; 4.68 mm/s is used as the example because it is the value closest in time to the machine's actual final CRITICAL state before the Dashboard-confirmed instant, but any real value in the observed range is a valid example of this template.

### C. NORMAL / automatic recovery (`alertType='recovery'`, `alarm_level=NORMAL`, `alarm_verdict_live=true`)

```
✅ กลับสู่สถานะปกติ

🏭 โรงงาน: plant01
⚙️ เครื่อง: PUMP01

🔎 เกิดอะไรขึ้น:
ค่าการสั่นสะเทือนของเครื่องนี้กลับสู่ระดับปกติแล้ว หลังจากที่เคยตรวจพบความผิดปกติก่อนหน้านี้

📌 ตอนนี้:
ค่าการสั่นสะเทือน: 0.34 mm/s
สถานะเครื่อง: กำลังทำงาน

🔧 ควรทำอะไร:
ไม่ต้องดำเนินการเพิ่มเติม ระบบจะเฝ้าระวังค่าการสั่นสะเทือนต่อเนื่อง

⏰ 15 ก.ย. 2569 05:50 น.
```
- **Fields shown:** plant, machine id, motor state, measured value, timestamp. **No command block** — recovery messages have never carried `ack`/`snooze` commands (nothing to acknowledge), preserved as-is.
- **Fields intentionally omitted:** same technical fields as A/B.
- **Semantic reason:** this is the state the original defect (see §before/after in this document, and the header of this file) was about. The header "✅ กลับสู่สถานะปกติ" is only ever produced when `alarm_verdict_live=true` — i.e., vibration is confirmed genuinely OK **and** the motor is confirmed RUNNING at the recovery instant. This is unchanged from the already-deployed `verdictLive` gate; this spec only rewords the surrounding text, never the gating condition itself.

### D. VIBRATION UNAVAILABLE (`alarm_verdict_live=false`, motor confirmed RUNNING — i.e. `vibration_measure_status !== 'OK'` while `motor_state===2`)

This state can arise from either an `alert`-type or a `recovery`-type payload today (whatever `alarm_level` the firmware sent), and the currently-deployed code already unifies both into one "unconfirmed" wording — this spec preserves that unification, only rewording it:

```
⚠️ ไม่สามารถยืนยันสภาพการสั่นสะเทือนได้ในขณะนี้

🏭 โรงงาน: plant01
⚙️ เครื่อง: PUMP01

🔎 เกิดอะไรขึ้น:
ระบบไม่สามารถอ่านค่าการสั่นสะเทือนจากเซนเซอร์ได้ในขณะนี้ ขณะที่เครื่องยังกำลังทำงานอยู่ จึงยังไม่สามารถยืนยันได้ว่าเครื่องอยู่ในสภาพปกติหรือไม่

📌 ตอนนี้:
ค่าการสั่นสะเทือน: ไม่มีข้อมูล
สถานะเครื่อง: กำลังทำงาน

🔧 ควรทำอะไร:
แนะนำให้ตรวจสอบการเชื่อมต่อของเซนเซอร์ หากสังเกตเห็นความผิดปกติของเครื่องด้วยวิธีอื่น (เสียง ความร้อน) ให้ตรวจสอบเครื่องเพิ่มเติม

⏰ {timestamp}
{command block, same as A/B — an operator may still want to ack/snooze a noisy unavailable-data condition}
```
- **Fields shown:** plant, machine id, motor state (กำลังทำงาน — this is what makes it distinct from E), explicit "ไม่มีข้อมูล" instead of any number, timestamp, commands.
- **Fields intentionally omitted:** `alarm_level` string itself is **never surfaced as a severity word** here — this is the one state where showing the firmware's raw `alarm_level` (which could technically be CRITICAL, WARNING, or NORMAL underneath) would be actively misleading, since none of those words are confirmed true. This is the direct SME-facing expression of the **hard requirement: "VIBRATION UNAVAILABLE must NEVER be described as NORMAL"** — and, by the same logic, never as WARNING or CRITICAL either. It is its own, fourth, category.
- **Semantic reason:** `alarm_verdict_live=false` while the motor is confirmed running means the *only* thing broken is data availability, not necessarily the machine. The wording is deliberately calm (not alarming, since nothing is confirmed wrong) but explicit that no clean bill of health can be issued.

### E. STOPPED (`motor_state=0`, vibration naturally unavailable because the motor is not running)

**Current deployed behavior:** because `alarm_verdict_live` is defined as `(vibration_measure_status==='OK') && (motor_code===2)`, a STOPPED machine (`motor_code≠2`) always yields `verdictLive=false` and today receives **the same wording as state D** — this is a known, previously-flagged simplification (`LINE_PATCH_FINAL_REVIEW_20260915.md`, "Outstanding, non-blocking observations"). This spec proposes distinguishing E from D, since "sensor cannot be read" (D) and "the machine is simply switched off" (E) mean very different things to an operator, but **this distinction is a proposed future refinement, not yet implemented** (see §8 for exact scope):

```
ℹ️ เครื่องหยุดทำงาน

🏭 โรงงาน: plant01
⚙️ เครื่อง: PUMP01

🔎 เกิดอะไรขึ้น:
เครื่องนี้อยู่ในสถานะหยุดทำงาน จึงไม่มีการวัดค่าการสั่นสะเทือนในขณะนี้ (เป็นสถานะปกติของเครื่องที่หยุดทำงาน ไม่ใช่ความผิดปกติของระบบ)

📌 ตอนนี้:
ค่าการสั่นสะเทือน: ไม่มีการวัด (เครื่องหยุดทำงาน)
สถานะเครื่อง: หยุดทำงาน

🔧 ควรทำอะไร:
ไม่ต้องดำเนินการ ระบบจะเริ่มตรวจสอบค่าการสั่นสะเทือนอีกครั้งเมื่อเครื่องเริ่มทำงาน

⏰ {timestamp}
```
- **Fields shown:** plant, machine id, motor state (หยุดทำงาน), an explicit "ไม่มีการวัด (เครื่องหยุดทำงาน)" line (deliberately different phrasing from D's "ไม่มีข้อมูล" so the two are never confusable at a glance), timestamp. **No command block** — there is nothing to acknowledge about a machine being off.
- **Fields intentionally omitted:** same technical fields as all other states.
- **Semantic reason:** directly implements **"STOPPED must not claim that vibration is normal when no valid vibration measurement exists"** — E never says "ปกติ" (normal) anywhere, and never implies a clean vibration reading; it states plainly that no measurement was taken and explains why in one clause, which also prevents an SME from mistaking this for a sensor fault (D) and calling a technician unnecessarily.

---

## 4. Real observed examples used above

| State | Example value used | Source |
|---|---|---|
| A (WARNING) | 2.25 mm/s | `LIVE_UAT_WARNING_EVENT_20260915.md` — directly-viewed LINE screenshot, real production WARNING event, 2026-09-15 12:36:49 Bangkok |
| B (CRITICAL) | 4.68 mm/s (observed range 4.68–6.93 mm/s) | `LIVE_UAT_CRITICAL_EVENT_20260915.md` — operator-reported LINE history, independently corroborated against live trend-API data |
| C (NORMAL recovery) | 0.34 mm/s | `LIVE_UAT_CRITICAL_EVENT_20260915.md` §2b — real, independently-queried historical trend baseline immediately before/after both observed events (0.334–0.345 mm/s range) |

D and E have no "value" example by construction — the whole point of both states is that no trustworthy vibration number exists to show.

---

## 5. State semantics (authoritative mapping — unchanged from current deployed logic)

| State | `alarm_level` (firmware, verbatim) | `alarm_verdict_live` | `motor_state` | Message category |
|---|---|---|---|---|
| A. WARNING | WARNING | true | RUNNING (2) | Confirmed WARNING |
| B. CRITICAL | CRITICAL | true | RUNNING (2) | Confirmed CRITICAL |
| C. NORMAL/recovery | NORMAL | true | RUNNING (2) | Confirmed NORMAL |
| D. VIBRATION UNAVAILABLE | CRITICAL / WARNING / NORMAL (any — irrelevant) | false | RUNNING (2) | Unconfirmed — never a severity word |
| E. STOPPED | any (irrelevant) | false | STOPPED (0) [also applies to STARTING(1)/STOPPING(3), see note] | Explicit "machine off," never "normal" |

**Note on STARTING/STOPPING:** the firmware's `motor_state` enum has four values (STOPPED/STARTING/RUNNING/STOPPING); only `RUNNING` (2) satisfies the live-verdict gate. This spec's state E text is written for STOPPED specifically; STARTING/STOPPING would use the same "no measurement, not an error" framing with the motor-state line adjusted to "เริ่มทำงาน"/"กำลังหยุด" respectively — same category, same reasoning, trivial text substitution, not spelled out as a separate lettered state since the semantic is identical to E.

---

## 6. Hidden / technical fields (present in the payload, never shown to the SME)

| Field | Why it stays hidden |
|---|---|
| `vibration_measure_source` / "FIFO-DSP" | Names an internal computation pipeline (which of several redundant paths produced the number). Meaningless to an SME and explicitly listed as a term to avoid. Available to an engineer reading the same payload in Node-RED debug view or InfluxDB if ever needed. |
| `alarm_code` (0/1/2 integer) | Pure machine-readable mirror of `alarm_level`; redundant with the plain-language title. |
| `rpm`, `rpm_valid` | See §F below — RPM validity must never influence or appear in the vibration message, by design, not merely by omission. |
| `VIB_WARNING_MMS` / `VIB_CRITICAL_MMS` (2.1 / 4.5 mm/s) and their OFF-thresholds (1.9 / 4.2) | **Decision: do NOT hard-code these into the user-facing message.** Reasoning: (1) An SME reader has no basis to judge whether 2.1 or 4.5 is a "big" or "small" number for this specific machine — the categorical word (WARNING/CRITICAL) plus the recommended action already carries 100% of the actionable meaning; showing raw thresholds adds numeric clutter without adding decision-relevant information. (2) It creates a **false-precision hazard right at the hysteresis boundary**: an operator seeing "current 4.4 mm/s, critical threshold 4.5" could reasonably (and wrongly) conclude "we're fine, still under the limit," when the firmware's actual hysteresis/persistence logic (§ threshold reconciliation) may already be tracking an escalation in progress from a slightly earlier reading, or may hold a already-CRITICAL state at 4.4 (below the ON edge but above the 4.2 OFF edge) — exposing the raw cutoff invites incorrect DIY reasoning about a state machine the SME was never meant to operate manually. (3) The measured **value** (which this spec does show) is sufficient for anyone forwarding the message to a vibration specialist to make the real comparison themselves, with full context, on the Dashboard, where the threshold lines are already drawn on the trend chart for exactly that audience. |
| `captureId`, escalation `pendCount`, dedup/backoff timestamps | Internal state-machine bookkeeping; irrelevant to "what happened / what now / what to do." |

---

## 7. Consistency rules with Dashboard (no second severity calculation)

- **This spec introduces zero new classification logic.** Every title in §3 is a direct text-rendering of the exact same two fields the Dashboard already reads and displays: `p.alarm_level` and `p.alarm_verdict_live` (itself `(vibration_measure_status==='OK') && (motor_code===2)`, unchanged, computed once in Health Logic — the same authoritative formula documented in `contract.py`'s `alarm_live` gate and mirrored in `LINE_VERDICT_CONTRACT_PROPOSAL.md`).
- **The LINE Message Builder must remain a pure text-formatting function of these two fields (plus `motor_state` and the measured value) — never re-derive severity from a raw RMS number itself.** This is already true today and this spec does not change it.
- If the Dashboard's own displayed status ever needs new wording, the two surfaces are expected to change together, from the same underlying `alarm_level`/`alarm_verdict_live` pairing — this document's wording is written so that translating the same pairing to the Dashboard's UI text would use materially the same three-question structure, keeping both surfaces in agreement by construction rather than by manual synchronization.

---

## 8. Minimal future wording-only patch scope

**Everything in this section is a proposal for a future, separately-authorized change. Nothing has been implemented.**

Only the string-building sections of `line_message_builder_patched.js` (the currently-deployed LINE Message Builder function-node source) would need to change:

1. **`alert` branch (lines ~76–98):** replace the two-line `alertHeader` + flat field-list `text` construction with the §2 template, branching on `p.alarm_level` (WARNING vs CRITICAL) for the title/body wording of §3.A/§3.B. `verdictLive`, `mId`, `plant`, `motorStateText`, `velRms`, `timeThai` are all already computed exactly as needed — no new variable required.
2. **`recovery` branch (lines ~109–124):** replace `recoveryHeader` + flat field list with §3.C's template when `verdictLive===true`.
3. **New conditional needed only for distinguishing D from E:** today, `!verdictLive` is a single branch. To render §3.D vs §3.E distinctly, add one `if (motorCode === 2)` check (D) vs. `else` (E) inside the existing `!verdictLive` branches of both `alert` and `recovery` — `motorCode` is already computed identically to Health Logic's own gate (`Number(p.motor_state)`), so this requires **no new upstream field, no change to Health Logic, no change to the Unified State Engine.**
4. **`escalation` branch (lines ~100–107):** out of scope for this spec — not one of the five/six lettered states, and not mentioned as needing SME-facing rewording in this task; left exactly as-is.
5. **Explicitly unchanged, and must remain unchanged, by any implementation of this spec:**
   - The `verdictLive` computation itself (Health Logic, `adf3dc5f003a516c`).
   - The Unified State Engine's recovery/dedup/escalation-timer logic (`ef04bfbbd91995e0`, `f9378e1435e2c70e`, `c7fcf5f3a2ec18c2`) — the *timing* of when a recovery/alert/escalation message fires is entirely untouched; this spec only changes what the resulting message *says*.
   - Snooze/backoff gating (lines ~39–50).
   - Token/groupId lookup and the `msg.headers`/`msg.payload` HTTP-request construction (lines ~127–138).
   - `ack`/`snooze` command semantics themselves (only their presentation position in the template moves, text unchanged).

**Estimated blast radius of the eventual patch: one function-node's string-building code only — the same two nodes already touched by the prior verdict-consistency patch, with no changes to any other node.**

---

## 9. Acceptance test cases for the eventual implementation

Modeled on the existing JS-harness pattern (`docs/engineering/evidence/line_audit_20260915/staging/run_test_harness.js`), which already exercises the real, unmodified function-node source via `vm.runInContext` — the same harness can be extended, not replaced, for these cases:

| # | Input | Required output (assertions) |
|---|---|---|
| 1 | `alarm_level=WARNING`, `motor_state=2`, `velocity_data_valid=true`, `velocity_rms_overall=2.25` | Title = `⚠️ แจ้งเตือน: พบการสั่นสะเทือนสูงกว่าปกติ`; body contains `2.25 mm/s`; motor-state line = `กำลังทำงาน`; action text present; ack/snooze commands present |
| 2 | `alarm_level=CRITICAL`, `motor_state=2`, `velocity_data_valid=true`, `velocity_rms_overall=4.68` | Title = `🚨 แจ้งเตือน: พบการสั่นสะเทือนระดับวิกฤต`; body contains `4.68 mm/s`; action text mentions ตรวจสอบโดยด่วน |
| 3 | recovery, `alarm_level=NORMAL`, `motor_state=2`, `velocity_data_valid=true`, `velocity_rms_overall=0.34` | Title = `✅ กลับสู่สถานะปกติ`; body contains `0.34 mm/s`; **no** ack/snooze command block |
| 4 | `motor_state=2`, `velocity_data_valid=false` (any `alarm_level`, alert or recovery) | Title is the D-wording (`⚠️ ไม่สามารถยืนยันสภาพการสั่นสะเทือนได้...`); body contains `ไม่มีข้อมูล`; **the literal string "ปกติ" (normal) must never appear anywhere in this message** — hard assertion, directly enforcing "VIBRATION UNAVAILABLE must NEVER be described as NORMAL" |
| 5 | `motor_state=0` (STOPPED), `velocity_data_valid=false`, any `alarm_level` | Title is the E-wording (`ℹ️ เครื่องหยุดทำงาน`); body contains `หยุดทำงาน`; **must not contain the D-wording's "ไม่สามารถยืนยันสภาพการสั่นสะเทือนได้"**; **must not contain "ปกติ" describing vibration** |
| 6 | `motor_state=2`, `velocity_data_valid=true`, `velocity_rms_overall=0.48`, `rpm_valid=false` | Output byte-identical (aside from the numeric value) to test #3's NORMAL template — **no RPM-related word or number appears anywhere in the message**, and the title/category is unaffected by `rpm_valid` — directly enforcing item F |
| 7 | `motor_state=2`, `velocity_data_valid=true`, `velocity_rms_overall=6.93`, `rpm_valid=false`, `alarm_level=CRITICAL` | Same CRITICAL template as test #2 — confirms RPM invalidity does not soften or alter a CRITICAL verdict either |
| 8 | Two-step sequence: CRITICAL (`velocity_data_valid=false`) → recovery NORMAL (`velocity_data_valid=false`), ~53s apart, same `machines` flow-context (mirrors the real forensic-trace scenario) | Step 1 renders as D-wording (not a confirmed CRITICAL claim); step 2 renders as D-wording (**not** `✅ กลับสู่สถานะปกติ`) — regression guard for the original defect this whole patch chain exists to prevent |
| 9 | Full-flow structural diff (as performed in every prior patch validation) | Node count unchanged; only the LINE Message Builder's `func` field differs from the pre-patch baseline; Health Logic, Unified State Engine, Escalation Timer, Auto Resume Check, 429 Handler, and the credentials node remain byte-identical |

---

## Before → after comparison: the original problematic pattern

**Before (the actual pre-patch production defect, documented in `LINE_NOTIFICATION_AUDIT.md` §12 and `AUTOMATIC_RECOVERY_FORENSIC_TRACE.md`):**
```
✅ กลับสู่ NORMAL
🏭 Plant: plant01
⚙️ Machine: PUMP01
📊 Velocity RMS: N/A (FIFO-DSP unavailable)
⏰ 15 ก.ย. 2569 11:36:13 น.
```
**Why this was dangerous for an SME:** the header is an unqualified, celebratory "back to normal" claim, while the very next line admits the system has no vibration data at all. An SME reader — who is not expected to notice that these two lines contradict each other — walks away believing the machine's vibration is confirmed fine, when in fact *nothing about vibration was confirmed at all*. This could suppress a real follow-up inspection precisely when one might still be warranted.

**After (currently deployed, per the already-shipped Verdict-Fix-1 patch):**
```
⚠️ NORMAL (ยืนยันสภาพจริงไม่ได้ — ข้อมูล vibration ไม่พร้อมใช้งาน / unconfirmed — vibration data unavailable)
🏭 Plant: plant01
⚙️ Machine: PUMP01
⚙️ Motor State: RUNNING
📊 Velocity RMS: N/A (FIFO-DSP unavailable)
⏰ 15 ก.ย. 2569 11:36:13 น.
```
This is already *correct* — it no longer claims a confirmed NORMAL — but it is bilingual, technical ("FIFO-DSP"), and structured as a flat field list rather than the "what happened / now / do" shape.

**After (this spec's proposed SME-facing wording, state D):**
```
⚠️ ไม่สามารถยืนยันสภาพการสั่นสะเทือนได้ในขณะนี้

🏭 โรงงาน: plant01
⚙️ เครื่อง: PUMP01

🔎 เกิดอะไรขึ้น:
ระบบไม่สามารถอ่านค่าการสั่นสะเทือนจากเซนเซอร์ได้ในขณะนี้ ขณะที่เครื่องยังกำลังทำงานอยู่ จึงยังไม่สามารถยืนยันได้ว่าเครื่องอยู่ในสภาพปกติหรือไม่

📌 ตอนนี้:
ค่าการสั่นสะเทือน: ไม่มีข้อมูล
สถานะเครื่อง: กำลังทำงาน

🔧 ควรทำอะไร:
แนะนำให้ตรวจสอบการเชื่อมต่อของเซนเซอร์ หากสังเกตเห็นความผิดปกติของเครื่องด้วยวิธีอื่น (เสียง ความร้อน) ให้ตรวจสอบเครื่องเพิ่มเติม

⏰ 15 ก.ย. 2569 11:36 น.
```
**Why this is safer for an SME:**
1. **No contradiction is possible** — the title itself states "cannot confirm," so there is nothing for the body to contradict.
2. **Plain Thai throughout**, no "FIFO-DSP," no bilingual technical qualifier the reader must parse to realize something is off.
3. **Gives an action** ("ตรวจสอบการเชื่อมต่อของเซนเซอร์...") — the deployed "after" version gives no instruction at all beyond the implicit warning tone; an SME reading it may not know whether to do anything.
4. **States why** in one clause ("ขณะที่เครื่องยังกำลังทำงานอยู่") rather than leaving the reader to infer the significance of "Motor State: RUNNING" next to "N/A."

---

## Scope confirmation

- This document is a specification only; no file other than this one was created or modified.
- No Node-RED flow, firmware, API, Dashboard/frontend, nginx configuration, MQTT configuration, or credential was read-write-touched, modified, or restarted.
- No LINE message was sent.
- No existing evidence document was edited.
- Not committed, per instructions.
- No LINE credential or token value is reproduced anywhere in this document.

---
*Design specification only. No production code, configuration, or existing evidence document was modified in producing this file.*
