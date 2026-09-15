# LINE SME-Wording Patch — Isolated-Copy Implementation & Validation
Recorded: 2026-09-15

**Type: ISOLATED-COPY IMPLEMENTATION + OFFLINE VALIDATION ONLY.** No production Node-RED flow, firmware, API, Dashboard/frontend, nginx, or MQTT configuration was modified. Nothing was deployed, restarted, or reloaded. No LINE message was sent. No credential was created, rotated, or copied. Not committed.

**Source specification implemented:** `docs/engineering/evidence/line_audit_20260915/LINE_SME_MESSAGE_SPEC_20260915.md`

---

## 1. What was changed, and where

**Exactly one node's `func` field, in an isolated copy only:** the LINE Message Builder (production node id `e982d76b3ebe0b06`). Nothing else.

New artifacts (all under `docs/engineering/evidence/line_audit_20260915/staging_sme_wording/`, none committed):

| File | SHA256 | Contents |
|---|---|---|
| `line_message_builder_SME_WORDING.js` | `2983e166aad612e52a20d38df4695dddfc91c7dafed17c095753acd45061752e` | The new, patched LINE Message Builder source implementing the SME spec. |
| `line_message_builder_PRE_SME_WORDING_currently_deployed.js` | `c9256669b1bb3f18ddbe020d3841a112b533a7c4b53ccb96372307c0504ad98d` | Verbatim copy of the *currently deployed* LINE Message Builder source (the already-shipped Verdict-Fix-1 version), used as the diff baseline. This hash is identical to the value recorded for the live-deployed node in `LINE_VERDICT_CONTRACT_PATCH_VALIDATION_20260915.md` and `LINE_PATCH_FINAL_REVIEW_20260915.md` — confirming this really is the deployed baseline, not a re-derivation. |
| `health_logic_UNCHANGED_reference_copy.js` | `847fc4336144c1e3a898a880ac72e7e31d623a69af1d148bf28caa0c616d5f5b` | Verbatim copy of the *currently deployed* Health Logic source. **Not modified by this patch.** Hash identical to the value recorded for the live-deployed node in the same two prior documents — confirms Health Logic is untouched. |
| `run_test_harness_sme.js` | `3b238bb83f8ab7ee9ccdf9457db9fb2c735c62c1ca942183a5a018975b5fcd90` | Offline JS test harness (see §3). |
| `test_output_sme.json` | `243aecbce3929782a336c27422cce6a396ae5fb87233a02e7a7154f4fbe3a45e` | Full harness output: all rendered message texts + 41 automated assertions. |
| `diff_PRE_vs_SME_WORDING.patch` | `61d7d79711541930e88182cdb764fcba0709c7c0eae31d67cb4d1fa75ff26430` | Unified diff, currently-deployed source → new SME-wording source (229 lines). |

**What the diff shows, summarized:** the token/groupId lookup block, the snooze/backoff check, the Thai-timestamp computation, the `escalation` branch, and the final `msg.headers`/`msg.payload` construction are all **unchanged, verbatim**. The only structural change is splitting the old two-branch (`alert`/`recovery`) text construction into an explicit `!verdictLive` check performed *before* the `alertType`-specific branches (so states D/E render identically regardless of whether the payload was tagged `alert` or `recovery` — see §6), plus the wording itself. `verdictLive` and `motorCode` are read from the exact same upstream fields (`p.alarm_verdict_live`, `p.motor_state`) as before — nothing about how they are computed changed, because they are computed entirely in Health Logic, which this patch never touches.

---

## 2. Requirement-by-requirement compliance

| # | Requirement | How satisfied |
|---|---|---|
| 1 | SME structure เกิดอะไรขึ้น → ตอนนี้ → ควรทำอะไร | Every non-escalation branch (A/B/C/D/E) follows this exact three-section order (see §4/§5 message texts). |
| 2 | Approved wording for WARNING/CRITICAL/NORMAL/UNAVAILABLE/STOPPED | Implemented verbatim from spec §3 A–E. |
| 3 | Replace "Health: RUNNING" with "Motor State: RUNNING" | Neither string exists any more — replaced by the Thai "สถานะเครื่อง: กำลังทำงาน" line, per spec. Verified by assertion (§3, check `H_no_Health_label`, 12/12 pass). |
| 4 | No FIFO-DSP as primary SME explanation | The literal string `"FIFO-DSP"` does not appear anywhere in the new source or in any of the 12 rendered message texts. Verified by assertion `no_FIFO_DSP_in_user_text`, 12/12 pass. |
| 5 | VIBRATION UNAVAILABLE never presented as NORMAL | Verified by assertion `I_no_confirmed_recovery_header_when_unavailable`, 3/3 pass (D, D2, and G2's recovery step). |
| 6 | No verdict calculation changed | `verdictLive`/`motorCode` read verbatim from Health Logic's output fields; Health Logic itself is byte-identical to the deployed source (§1 hash match). |
| 7 | No RPM-validity behavior changed | `rpm_valid` is never read anywhere in the new source (confirmed by absence — grep of the new file for "rpm" returns zero matches). Verified functionally by assertions `F_identical_to_RPM_valid_equivalent` (2/2 pass) and `F_no_rpm_word_in_any_message` (12/12 pass). |
| 8 | No new thresholds/severity logic added | The new source contains no numeric vibration threshold of any kind — `alarm_level` is read verbatim from the payload, exactly as before. |
| 9 | Automatic CRITICAL → NORMAL recovery preserved exactly | The Unified State Engine, Escalation Timer, Auto Resume Check, and 429 Handler nodes were not read, loaded, or referenced anywhere in this turn's work — see §6. The `recovery`/`alert`/`escalation` tagging (`msg._alertType`) that governs *when* a message fires is produced entirely upstream and consumed unchanged here. |

---

## 3. Offline validation harness

`run_test_harness_sme.js` — modeled on, and using the same technique as, the harness already used to validate the original Verdict-Fix-1 patch (`docs/engineering/evidence/line_audit_20260915/staging/run_test_harness.js`): loads and executes the **actual, unmodified function-node source** (both Health Logic and the new LINE Message Builder) via Node.js `vm.createContext`/`vm.runInContext` against a minimal `msg`/`flow`/`node` shim — not a reimplementation of the logic.

- **No network access:** the harness contains no `http`/`https`/`fetch`/socket code of any kind; it only inspects the in-memory `msg.payload.messages[0].text` string the function would have handed to the (never-invoked) LINE Push node.
- **Fake credentials only:** `PLANT_LINE_TOKENS = { plant01: 'TEST-TOKEN-NOT-REAL-NEVER-SENT' }`, `PLANT_GROUP_IDS = { plant01: 'TEST-GROUP-NOT-REAL-NEVER-SENT' }` — identical placeholder pattern to the original patch's harness.
- **No real LINE message sent** — structurally impossible, since no network call exists in the harness at all.
- Executed locally via `node run_test_harness_sme.js`, output captured to `test_output_sme.json`. Exit code 0, `allPass: true` (41/41 automated assertions passed).

---

## 4. Test cases A–I — input, verdict, output, expected, actual

| Case | Input (key fields) | Verdict (`alarm_verdict_live`) | Expected result | Actual result |
|---|---|---|---|---|
| **A. NORMAL + vibration valid** | `alarm_level=NORMAL, motor_state=2, velocity_data_valid=true, velocity_rms_overall=0.34`, type=`recovery` | `true` | Title `✅ กลับสู่สถานะปกติ`; shows `0.34 mm/s`; motor state `กำลังทำงาน`; no command block | **PASS** — exact match, see `A_NORMAL_valid` in `test_output_sme.json` |
| **B. WARNING + vibration valid** | `alarm_level=WARNING, motor_state=2, velocity_data_valid=true, velocity_rms_overall=2.25`, type=`alert` | `true` | Title `⚠️ แจ้งเตือน: พบการสั่นสะเทือนสูงกว่าปกติ`; shows `2.25 mm/s`; command block present | **PASS** |
| **C. CRITICAL + vibration valid** | `alarm_level=CRITICAL, motor_state=2, velocity_data_valid=true, velocity_rms_overall=4.68`, type=`alert` | `true` | Title `🚨 แจ้งเตือน: พบการสั่นสะเทือนระดับวิกฤต`; shows `4.68 mm/s`; command block present | **PASS** |
| **D. RUNNING + vibration unavailable** | `alarm_level=CRITICAL, motor_state=2, velocity_data_valid=false`, type=`alert` | `false` | Title `⚠️ ไม่สามารถยืนยันสภาพการสั่นสะเทือนได้ในขณะนี้`; `ไม่มีข้อมูล`; motor state `กำลังทำงาน`; command block present; **never** the word "ปกติ" describing vibration as confirmed | **PASS** |
| **D2 (added).** Same fact pattern, `alarm_level=NORMAL`, type=`recovery` | as above but recovery-tagged | `false` | Byte-identical text to D — the wording must not depend on which internal tag the payload carried | **PASS** — `D_RUNNING_unavailable.text === D2_RUNNING_unavailable_recoveryTagged.text` confirmed identical |
| **E. STOPPED** | `alarm_level=NORMAL, motor_state=0, velocity_data_valid=false`, type=`recovery` | `false` | Title `ℹ️ เครื่องหยุดทำงาน`; `ไม่มีการวัด (เครื่องหยุดทำงาน)`; motor state `หยุดทำงาน`; no command block; textually distinct from D | **PASS** |
| **F. RUNNING + RPM invalid + vibration valid** | Same as A but `rpm_valid=false` | `true` (unaffected) | Output byte-identical to case A; zero occurrence of "rpm" in text | **PASS** — `F_RUNNING_rpmInvalid_vibValid_NORMAL.text === A_NORMAL_valid.text` confirmed identical; also re-checked against a CRITICAL variant (F2 vs C) — identical |
| **G. CRITICAL → automatic recovery NORMAL** | Two-step sequence, shared flow context, ~real MQTT-message shape | step1 `true`→CRITICAL, step2 `true`→NORMAL (G1, vibration valid both steps); step1 `false`, step2 `false` (G2, vibration unavailable at both, mirrors the real forensic-trace shape) | G1: confirmed CRITICAL then confirmed `✅ กลับสู่สถานะปกติ`. G2: unconfirmed-D wording at both steps, **never** a confirmed-recovery header | **PASS** — both variants match expectation exactly |
| **H. No "Health: RUNNING"** | All 12 rendered texts (A, B, C, D, D2, E, F, F2, G1×2, G2×2) | — | The substring `"Health:"` must not appear in any message | **PASS** — 12/12 |
| **I. Unavailable vibration never produces "กลับสู่ NORMAL"** | D, D2, and G2's recovery step | `false` in all three | Neither the old `"กลับสู่ NORMAL"` nor the new `"กลับสู่สถานะปกติ"` string may appear | **PASS** — 3/3 |

Full raw output (all 12 complete message texts, verdict flags, and all 41 individual assertion results) is recorded in `staging_sme_wording/test_output_sme.json`.

---

## 5. Exact rendered examples (for direct human review)

**B. WARNING:**
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

⏰ 15 ก.ย. 2569 13:35:48 น.

📖 คำสั่ง:
  ack PUMP01
  snooze PUMP01 30
```

**D. VIBRATION UNAVAILABLE:**
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

⏰ 15 ก.ย. 2569 13:35:48 น.

📖 คำสั่ง:
  ack PUMP01
  snooze PUMP01 30
```
(All 12 rendered texts, including A/C/E/F/G1/G2, are in `test_output_sme.json`; the exact wording matches the spec's §3/§5 examples verbatim, produced by *executing* the real source, not copy-pasted from the spec by hand.)

---

## 6. Scope verification — only the LINE Message Builder node changed

- **Health Logic:** byte-identical to the currently deployed source, confirmed by matching SHA256 (`847fc4336144c1e3a898a880ac72e7e31d623a69af1d148bf28caa0c616d5f5b`, identical to the value recorded in `LINE_VERDICT_CONTRACT_PATCH_VALIDATION_20260915.md` for the live node). Not read for modification, not modified.
- **Unified State Engine, Escalation Timer, Auto Resume Check, 429 Handler, the credentials-init node, and every other node in the flow:** **not loaded, read, or referenced anywhere in this turn's work at all.** This patch's implementation and harness operate on exactly two isolated `.js` source files (Health Logic, LINE Message Builder) — there is no code path by which any other node could have been touched. This is a structural guarantee (true by construction), not a claim re-verified against a fresh full-flow diff this turn — see the limitation noted in §7.
- **No new credential, token, or secret was created, generated, or copied into any artifact.** The only "credential-shaped" strings in any new file are the literal placeholders `TEST-TOKEN-NOT-REAL-NEVER-SENT` / `TEST-GROUP-NOT-REAL-NEVER-SENT`, identical in form to the ones used and accepted in the original patch's harness.
- **No network or LINE call was made.** Confirmed by source inspection of `run_test_harness_sme.js` (§3) — no HTTP/socket-capable code exists in the harness.

---

## 7. ⚠️ Limitation encountered this turn — reported transparently, not worked around

The task asked this session to (a) freshly re-verify live production `flows.json`'s SHA256, and (b) "save the isolated flow artifact" as a full, sanitized copy of the entire flow with credentials removed (analogous to the previous full-copy `staged_flows_ISOLATED_COPY.json`, but with the credentials node redacted this time instead of excluded from git).

**Both of these required reading the live `flows.json` from `iotprom` this turn.** Two attempts were made:
1. `ssh iotprom "cat /opt/iot-stack/nodered/flows.json" > <local file>` — **blocked** by this session's auto-mode safety classifier, reason `[Production Reads]`.
2. `ssh iotprom "sha256sum /opt/iot-stack/nodered/flows.json"` (the same narrow, read-only hash check performed successfully many times earlier in this engagement) — **also blocked**, reason `[Credential Materialization]`.

Per this engagement's standing practice ("when blocked, stop immediately, not attempt workarounds, and report transparently"), no further attempts were made (e.g., no `head`/`tail`/partial-read substitute, no remote-side sanitize-and-print script) — both would still constitute reading the live flow this turn, which is exactly what was blocked.

**Consequence, stated precisely (see evidence-strength labels used throughout this engagement):**
- **NOT independently re-verified this turn:** that live `flows.json` is still SHA256 `d10be3a0eb983bf84407b9d53e6c162249d0f58b407eaad4906cf46a2fd3a3d5`. This is the **last value confirmed** (at the prior commit-verification turn, `docs(line): reconcile vibration threshold evidence`) — carried forward as the last-known state, not re-checked live just now.
- **Not produced this turn:** a full 59-node sanitized `flows.json`-shaped artifact with the credentials node redacted.
- **What was produced instead, satisfying the same underlying intent ("save the isolated flow artifact without any live credentials"):** a **per-node isolated artifact** (the two `.js` files in §1) — structurally the same pattern already used and accepted for the original Verdict-Fix-1 patch's `staging/pre_patch_nodes.json` (which contained only the two patched nodes, no credentials node, and was committed without issue). This is **inherently, unconditionally credential-free** — it was never possible for it to contain live credentials, since the credentials node was never read or referenced at all in building it, rather than being read and then redacted.

**If a full sanitized flow-shaped artifact and/or a fresh live-hash re-verification is still wanted, both require a live SSH read this session's classifier is currently declining — the user should either explicitly grant that read for this narrow purpose, or perform/re-confirm it themselves.**

---

## 8. Scope confirmation

- No production Node-RED flow, firmware, API, Dashboard/frontend, nginx, or MQTT configuration was modified.
- Nothing was deployed, restarted, or reloaded.
- No LINE message was sent; no network call was made by the harness.
- No credential was created, rotated, viewed, or copied into any artifact — only long-standing fake placeholder strings were used.
- Health Logic, the Unified State Engine, and every other node besides the LINE Message Builder were not touched, per §6.
- This document and the `staging_sme_wording/` artifacts have not been committed, per instructions.
- No LINE credential or token value is reproduced anywhere in this document.

---
*Isolated-copy implementation and offline validation only. No production code, configuration, or existing evidence document was modified in producing this file. Not deployed. Not committed.*
