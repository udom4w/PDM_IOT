# LINE SME Wording — Final Pre-Production Review
Recorded: 2026-09-15

**Type: READ-ONLY REVIEW.** No file under this review was modified as part of the review itself (the two minimal fixes recommended in §3 are proposed only — not applied). No production Node-RED flow, firmware, API, frontend, or nginx was touched. Nothing deployed, restarted, or sent.

**Source reviewed:** the actual rendered output in `docs/engineering/evidence/line_audit_20260915/staging_sme_wording/test_output_sme.json`, produced by executing the real patched source (`line_message_builder_SME_WORDING.js`) — not hand-copied from the spec. Cross-checked against `LINE_SME_MESSAGE_SPEC_20260915.md` and `LINE_SME_MESSAGE_PATCH_VALIDATION_20260915.md`.

---

## Final rendered messages A–G (verbatim from `test_output_sme.json`)

### A. NORMAL / automatic recovery
```
✅ กลับสู่สถานะปกติ

🏭 โรงงาน: plant01
⚙️ เครื่อง: pump01

🔎 เกิดอะไรขึ้น:
ค่าการสั่นสะเทือนของเครื่องนี้กลับสู่ระดับปกติแล้ว หลังจากที่เคยตรวจพบความผิดปกติก่อนหน้านี้

📌 ตอนนี้:
ค่าการสั่นสะเทือน: 0.34 mm/s
สถานะเครื่อง: กำลังทำงาน

🔧 ควรทำอะไร:
ไม่ต้องดำเนินการเพิ่มเติม ระบบจะเฝ้าระวังค่าการสั่นสะเทือนต่อเนื่อง

⏰ 15 ก.ย. 2569 13:35:48 น.
```
345 characters, 16 lines.

### B. WARNING
```
⚠️ แจ้งเตือน: พบการสั่นสะเทือนสูงกว่าปกติ

🏭 โรงงาน: plant01
⚙️ เครื่อง: pump01

🔎 เกิดอะไรขึ้น:
ระบบตรวจพบค่าการสั่นสะเทือนของเครื่องนี้สูงกว่าระดับปกติ ขณะที่เครื่องกำลังทำงานอยู่

📌 ตอนนี้:
ค่าการสั่นสะเทือน: 2.25 mm/s
สถานะเครื่อง: กำลังทำงาน

🔧 ควรทำอะไร:
เฝ้าระวังอย่างใกล้ชิด แนะนำให้ตรวจสอบเครื่องในรอบตรวจถัดไป

⏰ 15 ก.ย. 2569 13:35:48 น.

📖 คำสั่ง:
  ack pump01
  snooze pump01 30
```
395 characters, 20 lines.

### C. CRITICAL
```
🚨 แจ้งเตือน: พบการสั่นสะเทือนระดับวิกฤต

🏭 โรงงาน: plant01
⚙️ เครื่อง: pump01

🔎 เกิดอะไรขึ้น:
ระบบตรวจพบค่าการสั่นสะเทือนของเครื่องนี้สูงถึงระดับวิกฤต ซึ่งอาจเป็นสัญญาณของความเสียหายที่กำลังเกิดขึ้น

📌 ตอนนี้:
ค่าการสั่นสะเทือน: 4.68 mm/s
สถานะเครื่อง: กำลังทำงาน

🔧 ควรทำอะไร:
ตรวจสอบเครื่องโดยด่วน หากเป็นไปได้ให้พิจารณาหยุดเครื่องเพื่อตรวจสอบก่อนเดินเครื่องต่อ

⏰ 15 ก.ย. 2569 13:35:48 น.

📖 คำสั่ง:
  ack pump01
  snooze pump01 30
```
441 characters, 20 lines.

### D. VIBRATION UNAVAILABLE
```
⚠️ ไม่สามารถยืนยันสภาพการสั่นสะเทือนได้ในขณะนี้

🏭 โรงงาน: plant01
⚙️ เครื่อง: pump01

🔎 เกิดอะไรขึ้น:
ระบบไม่สามารถอ่านค่าการสั่นสะเทือนจากเซนเซอร์ได้ในขณะนี้ ขณะที่เครื่องยังกำลังทำงานอยู่ จึงยังไม่สามารถยืนยันได้ว่าเครื่องอยู่ในสภาพปกติหรือไม่

📌 ตอนนี้:
ค่าการสั่นสะเทือน: ไม่มีข้อมูล
สถานะเครื่อง: กำลังทำงาน

🔧 ควรทำอะไร:
แนะนำให้ตรวจสอบการเชื่อมต่อของเซนเซอร์ หากสังเกตเห็นความผิดปกติของเครื่องด้วยวิธีอื่น (เสียง ความร้อน) ให้ตรวจสอบเครื่องเพิ่มเติม

⏰ 15 ก.ย. 2569 13:35:48 น.

📖 คำสั่ง:
  ack pump01
  snooze pump01 30
```
533 characters, 20 lines. (Identical regardless of whether the underlying payload was tagged `alert` or `recovery` — confirmed against `D2` in the test output.)

### E. STOPPED
```
ℹ️ เครื่องหยุดทำงาน

🏭 โรงงาน: plant01
⚙️ เครื่อง: pump01

🔎 เกิดอะไรขึ้น:
เครื่องนี้อยู่ในสถานะหยุดทำงาน จึงไม่มีการวัดค่าการสั่นสะเทือนในขณะนี้ (เป็นสถานะปกติของเครื่องที่หยุดทำงาน ไม่ใช่ความผิดปกติของระบบ)

📌 ตอนนี้:
ค่าการสั่นสะเทือน: ไม่มีการวัด (เครื่องหยุดทำงาน)
สถานะเครื่อง: หยุดทำงาน

🔧 ควรทำอะไร:
ไม่ต้องดำเนินการ ระบบจะเริ่มตรวจสอบค่าการสั่นสะเทือนอีกครั้งเมื่อเครื่องเริ่มทำงาน

⏰ 15 ก.ย. 2569 13:35:48 น.
```
422 characters, 16 lines.

### F. RUNNING + RPM invalid + vibration valid
Byte-identical to A (NORMAL case) and to C (CRITICAL variant, `F2`) — confirmed by the harness's own equality assertion, not just visual inspection. No RPM-related word or number appears anywhere. Not reproduced again here; see A/C above.

### G. CRITICAL → automatic recovery NORMAL
**G1 (vibration valid at both instants):** step 1 = message C's exact text (CRITICAL, 6.93 mm/s); step 2 = message A's exact text (NORMAL, 0.34 mm/s). A clean, fully-confirmed alert-then-recovery sequence.

**G2 (vibration unavailable at both instants — mirrors the real forensic-trace incident):** step 1 = message D's exact text; step 2 = **also message D's exact text**, not A. The automatic CRITICAL→NORMAL *transition* still fires (per the unchanged Unified State Engine), but its *wording* correctly never claims a confirmed recovery when vibration cannot be confirmed at that instant — this is the direct fix for the original defect this whole engagement traced back to.

---

## SME readability evaluation

Evaluated against the 8 questions, for each state. "✅" = satisfies the question; "⚠️" = minor issue found, detailed below the table.

| State | 1. เกิดอะไรขึ้น ชัดเจน? | 2. ตอนนี้ชัดเจน? | 3. ควรทำอะไร ชัดเจน? | 4. technical wording ที่ไม่จำเป็น? | 5. เข้าใจผิดว่าเครื่องปกติทั้งที่ data ไม่พร้อม? | 6. Motor State ถูกต้อง? | 7. ยาวเกินไปสำหรับ LINE? | 8. แยกแยะจากสถานะอื่นชัดเจน? |
|---|---|---|---|---|---|---|---|---|
| A (NORMAL) | ✅ | ✅ | ✅ | ✅ ไม่มี | ✅ ไม่มีปัญหา (data พร้อมจริง) | ✅ | ✅ สั้นที่สุด (16 บรรทัด) | ✅ |
| B (WARNING) | ✅ | ✅ | ✅ | ✅ ไม่มี | ✅ ไม่มีปัญหา | ✅ | ✅ ยอมรับได้ (ดูหมายเหตุด้านล่าง) | ✅ |
| C (CRITICAL) | ✅ | ✅ | ✅ | ✅ ไม่มี | ✅ ไม่มีปัญหา | ✅ | ✅ ยอมรับได้ | ✅ |
| D (UNAVAILABLE) | ✅ | ✅ | ✅ | ✅ ไม่มี ("เซนเซอร์" เข้าใจง่ายกว่า "FIFO-DSP" มาก) | ✅ ไม่ claim ว่าปกติ (ดูรายละเอียดข้อ 5 ด้านล่าง) | ✅ | ✅ ยอมรับได้ (ยาวสุด 533 ตัวอักษร แต่ยังอยู่ในเกณฑ์) | ⚠️ **พบประเด็นเล็กน้อย — ดู Defect 2** |
| E (STOPPED) | ✅ | ✅ | ✅ | ✅ ไม่มี | ⚠️ **พบประเด็นเล็กน้อย — ดู Defect 1** | ✅ | ✅ สั้น (16 บรรทัด) | ✅ |

---

## Defects found

Two minor, non-blocking wording issues found — **neither violates a hard requirement** (no message ever literally claims vibration is confirmed normal when it is not; no message ever produces "กลับสู่ NORMAL"/"กลับสู่สถานะปกติ" without a confirmed live verdict, per the harness's own I-check, re-confirmed by direct reading above). Both are precision/ambiguity-reduction fixes, not redesigns, and neither changes the 3-question structure, the verdict semantics, or any logic.

### Defect 1 (minor) — State E reuses the word "ปกติ" in a sentence near vibration context

**Location:** E's "เกิดอะไรขึ้น" section: *"...(เป็น**สถานะปกติ**ของเครื่องที่หยุดทำงาน ไม่ใช่ความผิดปกติของระบบ)"*

**Why this is worth fixing, precisely:** this parenthetical correctly means "this is an *ordinary/expected* condition for a stopped machine" — it is grammatically about the STOPPED status, not about vibration, and does not literally claim vibration is normal (the "ตอนนี้" section two lines later explicitly says "ไม่มีการวัด"). It does **not** violate requirement 5. However, the word "ปกติ" is also the exact word used everywhere else in this template to mean *"vibration is confirmed OK"* (A's "กลับสู่**สถานะปกติ**", B's "สูงกว่า**ปกติ**", D's "สภาพ**ปกติ**"). A reader skimming quickly — which is the realistic reading mode for a mobile notification — could see "ปกติ" inside a message about their machine and register it as reassuring/positive before parsing the full clause, even though careful reading resolves it correctly. Given the entire point of this wording effort is to make the vibration-confirmed-or-not distinction unambiguous, reusing the trigger word in an unrelated sense here is an avoidable risk with a free fix available.

**Exact minimal fix (wording only, one clause):**
```diff
- เครื่องนี้อยู่ในสถานะหยุดทำงาน จึงไม่มีการวัดค่าการสั่นสะเทือนในขณะนี้ (เป็นสถานะปกติของเครื่องที่หยุดทำงาน ไม่ใช่ความผิดปกติของระบบ)
+ เครื่องนี้อยู่ในสถานะหยุดทำงาน จึงไม่มีการวัดค่าการสั่นสะเทือนในขณะนี้ (เป็นเรื่องธรรมดาสำหรับเครื่องที่หยุดทำงาน ไม่ใช่ความผิดปกติของระบบ)
```
Only "สถานะปกติของ" → "เรื่องธรรมดาสำหรับ" changes (one word, "ปกติ"→"ธรรมดา"); meaning, tone, and sentence structure are otherwise identical. "ไม่ใช่ความผิดปกติของระบบ" is left as-is — "ผิดปกติ" (abnormal) is a standard, unambiguous, idiomatic word in this negated form and does not carry the same risk (it directly denies abnormality, which cannot be misread as "vibration is fine").

### Defect 2 (minor) — State D shares its emoji with State B (WARNING)

**Location:** D's title emoji: `⚠️ ไม่สามารถยืนยันสภาพการสั่นสะเทือนได้ในขณะนี้` — the same `⚠️` used by B's `⚠️ แจ้งเตือน: พบการสั่นสะเทือนสูงกว่าปกติ`.

**Why this is worth fixing:** the full title text of D and B is clearly distinct on open — this is not a violation of requirement 8 in the body of the message. The risk is narrower: LINE's mobile notification banner and chat-list preview often show only the leading emoji plus the first few words before truncating. Two states that mean fundamentally different things — B is a *confirmed* elevated-vibration finding requiring monitoring, D is an *unconfirmed*, no-severity-claim data-availability issue — currently start with the same glyph, reducing at-a-glance distinguishability specifically in that truncated-preview context, even though the full message is unambiguous.

**Exact minimal fix (one emoji, D only):**
```diff
- ⚠️ ไม่สามารถยืนยันสภาพการสั่นสะเทือนได้ในขณะนี้
+ ❓ ไม่สามารถยืนยันสภาพการสั่นสะเทือนได้ในขณะนี้
```
`❓` was chosen because it carries no severity connotation at all (unlike `⚠️`/`🚨`), matching D's actual meaning ("unconfirmed," not "elevated"), and is visually distinct from every other state's emoji (`✅`/`⚠️`/`🚨`/`ℹ️`). No other character in the message changes.

---

## What was explicitly checked and found correct (not redesigned, no defect)

- **Requirement 5 (never present UNAVAILABLE as NORMAL):** verified directly by reading D's and G2's actual text above — neither the title nor any body line claims a confirmed normal condition; "ตอนนี้" explicitly says "ไม่มีข้อมูล." Confirmed also by the harness's automated `I_no_confirmed_recovery_header_when_unavailable` checks (3/3 pass).
- **Requirement — F must not imply RPM failure:** confirmed by direct byte-for-byte equality between F/F2 and their RPM-valid equivalents (A/C) — the word "rpm" appears nowhere in any of the 12 rendered messages.
- **3-question structure:** present, in the same fixed order, in all five states — not altered by either recommended fix (both fixes are inside existing sections, not structural).
- **Verdict semantics / any logic:** not evaluated for change here and not touched — this review is wording-only, per instructions, and neither recommended fix touches `verdictLive`, `motorCode`, `alarm_level`, or any conditional branch.
- **Length (question 7):** the new template is real and measurably longer than the previously-deployed wording (WARNING: 201→395 characters, 10→20 lines) — an intentional trade-off already disclosed in the spec, not a newly-discovered defect. LINE's text-message limit is far above any of these lengths (thousands of characters), so there is no technical truncation risk; the only cost is more scrolling/reading time, judged acceptable given the clarity gained.

---

## Final recommendation

```
WORDING REVISION REQUIRED (minor)
```

**Basis:** two small, precisely-scoped wording fixes are recommended before production use — both are single-word/single-emoji changes inside the already-approved template, neither is a structural redesign, neither changes verdict semantics or any logic, and neither is required by a hard requirement violation (none was found — both existing hard hard requirements checked in this review, "never present UNAVAILABLE as NORMAL" and "RPM must not imply failure," **pass cleanly as currently rendered**). This is closer to "ready with two one-line touch-ups" than a substantive rework: everything else in states A–G — structure, action clarity, motor-state wording, the "Health"/"FIFO-DSP" removals, and the CRITICAL→NORMAL recovery preservation (including the G2 regression case) — was reviewed and found correct as-is.

Once the two diffs in §"Defects found" are applied to `staging_sme_wording/line_message_builder_SME_WORDING.js` and re-validated by re-running `run_test_harness_sme.js`, this reviewer's assessment is that the result would be **READY FOR PRODUCTION**. Applying them was not done as part of this review, per instructions.

---

## Scope confirmation

- Read-only review; no file was modified by this review itself.
- No production Node-RED flow, firmware, API, frontend, or nginx was touched.
- Nothing deployed, restarted, or reloaded. No LINE message sent.
- Not committed, per instructions.
- No LINE credential or token value is reproduced anywhere in this document.

---
*Read-only review. No production code, configuration, or existing evidence document was modified in producing this file.*
