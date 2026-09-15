# LINE SME Wording — Review-Fix Patch V2 — Isolated-Copy Implementation & Validation
Recorded: 2026-09-15

**Type: ISOLATED-COPY IMPLEMENTATION + OFFLINE VALIDATION ONLY.** No production Node-RED flow, firmware, API, Dashboard/frontend, nginx, or MQTT configuration was modified. Nothing was deployed, restarted, or reloaded. No LINE message was sent. No credential was created, rotated, or copied. Not committed.

**Applies exactly the two minor fixes** identified in `LINE_SME_WORDING_FINAL_REVIEW_20260915.md` ("Defect 1" and "Defect 2") to the isolated-copy file already produced in `LINE_SME_MESSAGE_PATCH_VALIDATION_20260915.md`. Nothing else was changed.

---

## 1. Exact before/after diff

Applied to `docs/engineering/evidence/line_audit_20260915/staging_sme_wording/line_message_builder_SME_WORDING.js` in place. The pre-fix version is preserved verbatim as `line_message_builder_SME_WORDING_V1_pre_review_fixes.js` (SHA256 `2983e166aad612e52a20d38df4695dddfc91c7dafed17c095753acd45061752e` — identical to the file's hash recorded in `LINE_SME_MESSAGE_PATCH_VALIDATION_20260915.md` §1, confirming this really is the same content reviewed, not a re-derivation).

```diff
--- line_message_builder_SME_WORDING_V1_pre_review_fixes.js
+++ line_message_builder_SME_WORDING.js
@@ (state D, leading emoji) @@
-        text = '⚠️ ไม่สามารถยืนยันสภาพการสั่นสะเทือนได้ในขณะนี้\n\n'
+        text = '❓ ไม่สามารถยืนยันสภาพการสั่นสะเทือนได้ในขณะนี้\n\n'

@@ (state E, ambiguous "ปกติ") @@
             + 'เครื่องนี้อยู่ในสถานะหยุดทำงาน จึงไม่มีการวัดค่าการสั่นสะเทือนในขณะนี้ '
-            + '(เป็นสถานะปกติของเครื่องที่หยุดทำงาน ไม่ใช่ความผิดปกติของระบบ)\n\n'
+            + '(เป็นเรื่องธรรมดาสำหรับเครื่องที่หยุดทำงาน ไม่ใช่ความผิดปกติของระบบ)\n\n'
```
(Explanatory code comments were added alongside each change, citing the specific defect fixed — see the full unified diff, 27 lines including comments, saved as `staging_sme_wording/diff_V1_to_V2_review_fixes.patch`, SHA256 `cdaa3e6664c9a4c17e8c5971ee6bbdbed9fd012b2348fe9c3b4ca1638d86ea75`.)

**Exactly these two string literals changed. No other line, variable, branch, or condition was touched.**

---

## 2. Harness re-run and new regression assertions

`run_test_harness_sme_v2.js` (SHA256 `580a9d45c18457ec087955c96cfad6f947a5d127014c537da45ed0b67dab66b3`) — a copy of the original `run_test_harness_sme.js`, unchanged in every respect except that it appends the new V2-specific assertions listed below after the original 41. Loads the same real, unmodified function-node sources (`health_logic_UNCHANGED_reference_copy.js` and the now-fixed `line_message_builder_SME_WORDING.js`) via the same `vm.createContext`/`vm.runInContext` technique — no reimplementation, no network access, same fake placeholder credentials (`TEST-TOKEN-NOT-REAL-NEVER-SENT` / `TEST-GROUP-NOT-REAL-NEVER-SENT`).

Executed locally via `node run_test_harness_sme_v2.js`, output captured to `test_output_sme_v2.json` (SHA256 `82874525d21159dec7af2fa53d2e6a9d0307af93dbd3cc66359bfe1bd30249c6`). Exit code 0.

### All previous (V1) assertions still pass

All 41 original assertions from `test_output_sme.json` — covering states A–I, the RPM-invalidity checks, the "no FIFO-DSP" check, and the "no Health:" check — were re-run against the fixed source and **all 41 still pass**. This confirms the two wording fixes did not regress anything the first validation pass had already established.

### New V2 regression assertions (13, all pass)

| Check | Case(s) | Result |
|---|---|---|
| `V2_contains_thammada` — STOPPED contains "ธรรมดา" | E_STOPPED | ✅ PASS |
| `V2_parenthetical_no_ambiguous_pokati` — the specific parenthetical clause no longer contains the ambiguous "สถานะปกติ" phrase | E_STOPPED | ✅ PASS |
| `V2_D_starts_with_question_mark_emoji` — UNAVAILABLE starts with ❓ | D, D2, G2.step1, G2.step2 (4 cases) | ✅ PASS (4/4) |
| `V2_UNAVAILABLE_never_confirmed_recovery` — never contains "กลับสู่ NORMAL" (old wording) or "กลับสู่สถานะปกติ" (new wording) | D, D2 | ✅ PASS (2/2) |
| `V2_WARNING_starts_with_warning_emoji` — WARNING still starts with ⚠️ | B | ✅ PASS |
| `V2_CRITICAL_starts_with_siren_emoji` — CRITICAL still starts with 🚨 | C | ✅ PASS |
| `V2_NORMAL_wording_unchanged_from_V1` — NORMAL recovery text matches the exact V1 baseline byte-for-byte (up to the trailing timestamp, which is expected to vary by wall-clock time) | A | ✅ PASS |
| `V2_RPM_invalid_still_identical_to_valid_equivalent` — RPM-invalid output remains byte-identical to its RPM-valid equivalent, re-checked post-fix | F vs A, F2 vs C | ✅ PASS (2/2) |

**Total: 41 (V1, re-confirmed) + 13 (new V2) = 59 assertions, `allPass: true`, 0 failures.**

---

## 3. Rendered examples, post-fix (for direct human review)

**E. STOPPED (fixed):**
```
ℹ️ เครื่องหยุดทำงาน

🏭 โรงงาน: plant01
⚙️ เครื่อง: pump01

🔎 เกิดอะไรขึ้น:
เครื่องนี้อยู่ในสถานะหยุดทำงาน จึงไม่มีการวัดค่าการสั่นสะเทือนในขณะนี้ (เป็นเรื่องธรรมดาสำหรับเครื่องที่หยุดทำงาน ไม่ใช่ความผิดปกติของระบบ)

📌 ตอนนี้:
ค่าการสั่นสะเทือน: ไม่มีการวัด (เครื่องหยุดทำงาน)
สถานะเครื่อง: หยุดทำงาน

🔧 ควรทำอะไร:
ไม่ต้องดำเนินการ ระบบจะเริ่มตรวจสอบค่าการสั่นสะเทือนอีกครั้งเมื่อเครื่องเริ่มทำงาน

⏰ 15 ก.ย. 2569 13:44:16 น.
```
The word "ปกติ" no longer appears anywhere in this message.

**D. VIBRATION UNAVAILABLE (fixed, first line):**
```
❓ ไม่สามารถยืนยันสภาพการสั่นสะเทือนได้ในขณะนี้
```
(Remainder of the message is unchanged from `LINE_SME_WORDING_FINAL_REVIEW_20260915.md`'s rendering — only the leading emoji changed.)

Full raw output (all 12 message texts, all 59 assertion results) is in `staging_sme_wording/test_output_sme_v2.json`.

---

## 4. Scope verification

- **Only LINE Message Builder wording changed.** The two edits above (§1) are the entirety of the diff. `health_logic_UNCHANGED_reference_copy.js` was re-diffed against the currently-deployed `staging/health_logic_patched.js` this turn and is still byte-identical (`diff` produced no output) — Health Logic was not read for modification, and was not modified.
- **No other node was loaded, read, or referenced** — this turn's work, like the prior V1 patch turn, operates on exactly two isolated `.js` source files. There is no code path by which the Unified State Engine, Escalation Timer, Auto Resume Check, 429 Handler, or the credentials node could have been touched.
- **Verdict logic unchanged:** `verdictLive`/`motorCode` are read from the exact same fields (`p.alarm_verdict_live`, `p.motor_state`) as before this fix and as in the original deployed source — neither edit touches a conditional, a comparison, or any computation, only two string literals.
- **Recovery/dedup/escalation logic unchanged:** the snooze/backoff check, the `escalation` branch, and the branching structure that determines *when* each state's text is produced are untouched — confirmed by the diff in §1 showing only the two string literals changed, nothing in the surrounding control flow.
- **No live production file was modified.** This entire patch exists only under `docs/engineering/evidence/line_audit_20260915/staging_sme_wording/`, an evidence/staging directory never read by production Node-RED. No deploy, restart, or reload was performed. (Per the same limitation already disclosed in `LINE_SME_MESSAGE_PATCH_VALIDATION_20260915.md` §7, a fresh live SSH re-verification of `flows.json`'s hash was not attempted this turn either, since it was not required for an isolated-copy-only task and the prior turn's SSH block for that action remains the last known state.)
- **No real credential was copied into any artifact.** The only "credential-shaped" strings in any file this turn are the same long-standing fake placeholders (`TEST-TOKEN-NOT-REAL-NEVER-SENT` / `TEST-GROUP-NOT-REAL-NEVER-SENT`) already used in the V1 harness — confirmed by a fresh grep scan of every new/modified file in this turn, finding no other credential-shaped string.

---

## 5. Outstanding items

None. Both defects from `LINE_SME_WORDING_FINAL_REVIEW_20260915.md` are now fixed in the isolated copy and re-validated. Per that review's own closing statement, this reviewer's assessment was that applying these two fixes would make the result **READY FOR PRODUCTION** — that assessment is not re-asserted as a formal verdict here (this document is an implementation-and-validation record, not a second review), but the two conditions it named have now been met and verified offline.

Deployment of this fixed isolated copy to production is a separate, not-yet-authorized action and was not performed as part of this task.

---

## 6. Scope confirmation

- No production Node-RED flow, firmware, API, Dashboard/frontend, nginx, or MQTT configuration was modified.
- Nothing was deployed, restarted, or reloaded.
- No LINE message was sent; no network call was made by either harness.
- No credential was created, rotated, viewed, or copied into any artifact.
- Health Logic and every other node besides the LINE Message Builder were not touched.
- This document and the `staging_sme_wording/` artifacts have not been committed, per instructions.
- No LINE credential or token value is reproduced anywhere in this document.

---
*Isolated-copy implementation and offline validation only. No production code, configuration, or existing evidence document was modified in producing this file. Not deployed. Not committed.*
