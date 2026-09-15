# LINE SME Wording V3 — Operator-Reviewed Language Pass, Isolated-Copy Implementation & Validation
Recorded: 2026-09-15

**Type: ISOLATED-COPY IMPLEMENTATION + OFFLINE VALIDATION ONLY.** Wording-only change. No production Node-RED flow, firmware, API, Dashboard/frontend, nginx, MQTT, Health Logic, Unified State Engine, recovery/debounce/dedup/escalation/snooze/ack logic, RPM-validity behavior, or credentials was modified. Nothing deployed, restarted, or sent. Not committed.

**Trigger:** operator review of live production output (V2 wording, currently deployed per `LINE_SME_WORDING_PRODUCTION_DEPLOYMENT_20260915.md`) requesting a further language pass — lead with the plain-language finding rather than the sensor reading, use plant-technician vocabulary, and never speculate that "damage is occurring" from a single elevated reading.

---

## 1. Before / after wording, and the reason for each change

All five states below use the **exact target template text specified for this task**, implemented verbatim in the isolated copy — not independently redesigned.

### A. NORMAL / RECOVERY

| | Before (V2, currently deployed) | After (V3) |
|---|---|---|
| Title | `✅ กลับสู่สถานะปกติ` | `✅ การสั่นสะเทือนกลับสู่ระดับปกติ` |
| เกิดอะไรขึ้น | "ค่าการสั่นสะเทือนของเครื่องนี้กลับสู่ระดับปกติแล้ว หลังจากที่เคยตรวจพบความผิดปกติก่อนหน้านี้" | "ค่าการสั่นสะเทือนของเครื่องกลับเข้าสู่ระดับปกติ / หลังจากก่อนหน้านี้ตรวจพบค่าที่สูงขึ้น" |
| ควรทำอะไร | "ไม่ต้องดำเนินการเพิ่มเติม ระบบจะเฝ้าระวังค่าการสั่นสะเทือนต่อเนื่อง" | "ไม่ต้องดำเนินการเพิ่มเติม / ระบบจะเฝ้าระวังค่าการสั่นสะเทือนต่อเนื่อง" (same wording, now wrapped as two short lines) |

**Reason:** the title now names the subject ("การสั่นสะเทือน") explicitly rather than a bare status word, consistent with the "อย่าเน้นรายงานค่า sensor อย่างเดียว" principle — it reads as a finding about the machine's vibration condition, not a generic status flip. The "เกิดอะไรขึ้น" clause is split onto two short lines for faster mobile scanning, per the operator's stated goal of reading the message once and knowing the situation immediately.

### B. WARNING

| | Before (V2) | After (V3) |
|---|---|---|
| Title | `⚠️ แจ้งเตือน: พบการสั่นสะเทือนสูงกว่าปกติ` | `⚠️ พบการสั่นสะเทือนสูงกว่าระดับเฝ้าระวัง` |
| เกิดอะไรขึ้น | "ระบบตรวจพบค่าการสั่นสะเทือนของเครื่องนี้สูงกว่าระดับปกติ ขณะที่เครื่องกำลังทำงานอยู่" | "ระบบตรวจพบการสั่นสะเทือนสูงกว่าระดับเฝ้าระวัง" |
| ควรทำอะไร | "เฝ้าระวังอย่างใกล้ชิด แนะนำให้ตรวจสอบเครื่องในรอบตรวจถัดไป" | "ติดตามค่าการสั่นสะเทือนต่อเนื่อง / และตรวจสอบเครื่องในรอบตรวจถัดไป" |

**Reason:** dropped the redundant "แจ้งเตือน:" prefix (the ⚠️ emoji and the sentence itself already signal an alert — the prefix added length without adding information). Replaced "สูงกว่าปกติ" (comparative "higher than normal") with "สูงกว่าระดับเฝ้าระวัง" ("above the watch/monitoring level") — this names the actual concept (a monitoring threshold was crossed) instead of reusing the word "ปกติ", which elsewhere in the template means "vibration confirmed OK" and is best reserved for that one meaning. Dropped "ขณะที่เครื่องกำลังทำงานอยู่" from the "เกิดอะไรขึ้น" clause since the same fact is already shown one section later on the "สถานะเครื่อง: กำลังทำงาน" line — saying it twice added length without adding information, contrary to the "ไม่มีคำฟุ่มเฟือย" (no filler words) instruction.

### C. CRITICAL

| | Before (V2) | After (V3) |
|---|---|---|
| Title | `🚨 แจ้งเตือน: พบการสั่นสะเทือนระดับวิกฤต` | `🚨 พบการสั่นสะเทือนระดับวิกฤต` |
| เกิดอะไรขึ้น | "ระบบตรวจพบค่าการสั่นสะเทือนของเครื่องนี้สูงถึงระดับวิกฤต **ซึ่งอาจเป็นสัญญาณของความเสียหายที่กำลังเกิดขึ้น**" | "ระบบตรวจพบการสั่นสะเทือนถึงระดับวิกฤต" |
| ควรทำอะไร | "ตรวจสอบเครื่องโดยด่วน หากเป็นไปได้ให้พิจารณาหยุดเครื่องเพื่อตรวจสอบก่อนเดินเครื่องต่อ" | "ตรวจสอบเครื่องโดยด่วน / และพิจารณาหยุดเครื่องตามขั้นตอนความปลอดภัยก่อนเดินเครื่องต่อ" |

**Reason (the most substantive wording change in this pass):** the old wording's clause **"ซึ่งอาจเป็นสัญญาณของความเสียหายที่กำลังเกิดขึ้น"** ("which may be a sign of damage currently occurring") is a speculative conclusion the system has no basis to draw from a single vibration reading — this is exactly the pattern this task explicitly prohibited ("ห้ามสรุปว่า 'กำลังเกิดความเสียหาย' เพราะ vibration สูงเพียงอย่างเดียว"). It is removed entirely; the message now states only the observed fact (a critical-level reading was detected) and lets the action ("ตรวจสอบเครื่องโดยด่วน... พิจารณาหยุดเครื่อง") carry the appropriate urgency without an unsupported causal claim. The action clause was also reworded to reference "ขั้นตอนความปลอดภัย" (safety procedure) — plant-technician register, not a bare instruction.

### D. VIBRATION UNAVAILABLE

| | Before (V2) | After (V3) |
|---|---|---|
| Title | `❓ ไม่สามารถยืนยันสภาพการสั่นสะเทือนได้ในขณะนี้` | `❓ ยังไม่สามารถประเมินการสั่นสะเทือนได้` |
| เกิดอะไรขึ้น | "ระบบไม่สามารถอ่านค่าการสั่นสะเทือนจากเซนเซอร์ได้ในขณะนี้ ขณะที่เครื่องยังกำลังทำงานอยู่ จึงยังไม่สามารถยืนยันได้ว่าเครื่องอยู่ในสภาพปกติหรือไม่" | "ระบบยังไม่ได้รับข้อมูลการสั่นสะเทือนที่พร้อมใช้งาน" |
| ควรทำอะไร | "แนะนำให้ตรวจสอบการเชื่อมต่อของเซนเซอร์ หากสังเกตเห็นความผิดปกติของเครื่องด้วยวิธีอื่น (เสียง ความร้อน) ให้ตรวจสอบเครื่องเพิ่มเติม" | "ตรวจสอบระบบวัดและรอข้อมูลกลับมา / **ยังไม่ควรสรุปว่าเครื่องอยู่ในภาวะปกติ**" |

**Reason:** shortened a three-clause sentence into one plain statement of fact ("no data has arrived yet") — dropped "เซนเซอร์" as the sole named cause (the real cause could be several things upstream of "the sensor," and the SME does not need to diagnose which one). Critically, the "ควรทำอะไร" line **keeps an explicit, standalone guardrail sentence — "ยังไม่ควรสรุปว่าเครื่องอยู่ในภาวะปกติ" ("do not yet conclude the machine is in a normal condition")** — this is not merely implied by the absence of the word "ปกติ" elsewhere (as V2 relied on); it is stated directly as part of the instruction to the reader, which is an even more explicit implementation of the hard rule "ห้ามบอกว่า NORMAL ถ้าข้อมูล vibration ไม่พร้อม" than the previous wording achieved.

### E. STOPPED

| | Before (V2) | After (V3) |
|---|---|---|
| Title | `ℹ️ เครื่องหยุดทำงาน` | `⏹️ เครื่องหยุดทำงาน` |
| เกิดอะไรขึ้น | "เครื่องนี้อยู่ในสถานะหยุดทำงาน จึงไม่มีการวัดค่าการสั่นสะเทือนในขณะนี้ (เป็นเรื่องธรรมดาสำหรับเครื่องที่หยุดทำงาน ไม่ใช่ความผิดปกติของระบบ)" | "เครื่องไม่ได้กำลังทำงาน จึงไม่มีการวัดการสั่นสะเทือนในขณะนี้" |
| ตอนนี้ | `ค่าการสั่นสะเทือน: ไม่มีการวัด (เครื่องหยุดทำงาน)` + `สถานะเครื่อง: หยุดทำงาน` (2 lines) | `สถานะเครื่อง: หยุดทำงาน` (1 line — the vibration line is dropped) |
| ควรทำอะไร | "ไม่ต้องดำเนินการ ระบบจะเริ่มตรวจสอบค่าการสั่นสะเทือนอีกครั้งเมื่อเครื่องเริ่มทำงาน" | "เมื่อเครื่องกลับมาทำงาน ระบบจะเริ่มประเมินการสั่นสะเทือนอีกครั้ง" |

**Reason:** emoji changed from the generic info bubble `ℹ️` to `⏹️` (a stop glyph), a more immediately recognizable "machine is off" signal for a plant technician than a neutral info icon. The defensive parenthetical explaining that a stopped machine not being measured is "ordinary, not a system fault" is removed entirely — it was reassurance text that added length without adding anything a technician needs to act on; a stopped machine not being measured is self-evident. Removing it also has the side effect of eliminating the word "ปกติ" from this message **entirely** (stronger than V2's earlier "ธรรมดา" substitution, which still used a related concept in the same spot — V3 simply doesn't discuss "normal" at all here, since there is nothing vibration-related to characterize while the machine is off). The "ตอนนี้" section drops the vibration-value line altogether, since a stopped machine has nothing to report there and the earlier "ไม่มีการวัด (เครื่องหยุดทำงาน)" line was redundant with the motor-state line directly below it.

---

## 2. Real production examples (value ranges cited, not hard-coded)

Per the task's explicit instruction — **no threshold number (2.1 / 4.5 mm/s) is hard-coded into the message text**, and none was added. `velValueText` continues to read the real payload value verbatim (`p.velocity_rms_overall.toFixed(2) + ' mm/s'`), unchanged from V1/V2. The ranges below are illustrative test inputs reflecting real observed production values, not literals in the code:

| State | Illustrative range cited | Value used in this validation's test case |
|---|---|---|
| WARNING | ≈2.25–3.96 mm/s | 2.25 mm/s |
| CRITICAL | ≈4.68–6.93 mm/s | 4.68 mm/s |
| NORMAL | ≈0.34–0.89 mm/s | 0.34 mm/s |

A dedicated assertion (`V3_no_hardcoded_threshold_numbers`, run against all 12 rendered messages) confirms neither `2.1` nor `4.5` appears as a literal number anywhere in any rendered output.

---

## 3. Test methodology

Same technique used throughout this engagement: `run_test_harness_sme_v3.js` loads and executes the **actual, unmodified function-node source** (`health_logic_UNCHANGED_reference_copy.js` and the new `line_message_builder_SME_WORDING_V3.js`) via Node.js `vm.createContext`/`vm.runInContext` — not a reimplementation. No network access (no HTTP/socket code exists in the harness). Fake placeholder credentials only (`TEST-TOKEN-NOT-REAL-NEVER-SENT` / `TEST-GROUP-NOT-REAL-NEVER-SENT`). Executed locally via `node run_test_harness_sme_v3.js`, output captured to `test_output_sme_v3.json`.

**All required cases exercised:** A (NORMAL), B (WARNING), C (CRITICAL), D (vibration unavailable), E (STOPPED), F/F2 (RPM invalid + vibration valid, NORMAL and CRITICAL variants), plus D2 (unavailable arriving recovery-tagged) and G1/G2 (the two-step automatic-recovery sequence, vibration-valid and vibration-unavailable variants) carried forward from V1/V2 for continuity.

---

## 4. Test results

**86/86 assertions pass, `allPass: true`, exit code 0.**

This breaks down as:
- **41 assertions carried forward unchanged from V1** (H: no "Health:" label; I: unavailable never confirmed-recovery; RPM-identity checks; no-FIFO-DSP; D/E distinctness; E states no measurement; A/B/C show real numeric values) — **all still pass** against the new V3 wording, since none of these check specific literal wording that this pass intentionally changed.
- **13 assertions carried forward from V2** with three retired as noted below, the rest re-verified — still pass.
- **New/updated V3-specific assertions** (see full list in `run_test_harness_sme_v3.js`), covering every explicit regression requirement in this task:

| Requirement (as given) | Assertion(s) | Result |
|---|---|---|
| UNAVAILABLE ไม่สามารถแสดง NORMAL | `V3_UNAVAILABLE_never_confirmed_recovery` (checked against all three historical confirmed-recovery phrasings: V1/V2/V3) | ✅ PASS (2/2 cases) |
| STOPPED ไม่สามารถแสดง NORMAL | `V3_STOPPED_never_confirmed_recovery`, `V3_STOPPED_never_uses_pokati` | ✅ PASS |
| Motor State ยังถูกต้อง | `V3_motor_state_line_correct` (checked per-case against the expected Thai label for that case's motor_state code) | ✅ PASS (8/8 cases) |
| ไม่มี "Health" | `H_no_Health_label` (carried forward from V1) | ✅ PASS (all 12 messages) |
| WARNING/CRITICAL/NORMAL/UNAVAILABLE อ่านต่างกันชัดเจน | `V3_four_states_titles_distinct`, `V3_four_states_emojis_distinct` (all four titles distinct strings AND all four leading emojis distinct — stronger than V1/V2, which had D and B sharing ⚠️ until V2's fix) | ✅ PASS |
| RPM invalid ไม่เปลี่ยนข้อความของ vibration verdict | `V3_RPM_invalid_still_identical_to_valid_equivalent` (F vs A, F2 vs C, byte-for-byte) | ✅ PASS (2/2) |
| automatic recovery logic unchanged | `V3_automatic_recovery_sequence_intact` (G1 step1 still starts 🚨, G1 step2 still starts ✅; G2 step2 never shows any confirmed-recovery phrasing) | ✅ PASS (3/3) |
| ห้ามสรุปว่า "กำลังเกิดความเสียหาย" | `V3_CRITICAL_no_damage_claim` — the word "เสียหาย" must not appear anywhere in any CRITICAL message | ✅ PASS (2/2, including the RPM-invalid CRITICAL variant) |
| ห้าม hard-code threshold 2.1/4.5 | `V3_no_hardcoded_threshold_numbers` | ✅ PASS (all 12 messages) |

**Note on the three retired V2-specific literal-text checks:** V2's harness contained three assertions that checked for the *exact V2 wording tokens* (`V2_contains_thammada`, `V2_parenthetical_no_ambiguous_pokati`, `V2_NORMAL_wording_unchanged_from_V1`). Since this task intentionally rewords STOPPED and NORMAL again (with explicit, operator-approved target text), re-running those exact checks against V3 would produce **expected, not-a-regression failures** (V3's STOPPED text no longer contains "ธรรมดา" at all — it removed the whole clause, which is a *stronger* resolution of the original ambiguity concern, not a regression). These three checks were replaced with assertions against the **underlying invariant** each one protected (see `V3_STOPPED_never_uses_pokati`, `V3_STOPPED_never_confirmed_recovery`, and the general four-state distinctness checks above) — this is disclosed explicitly rather than silently dropped.

Full raw output (all 12 rendered message texts, all 86 assertion results) is in `staging_sme_wording/test_output_sme_v3.json`.

---

## 5. Confirmation — wording-only change

- **Exact diff:** `staging_sme_wording/diff_V2_to_V3.patch` (218 lines, mostly comment additions) shows every changed line is inside a string-literal template (title / เกิดอะไรขึ้น / ตอนนี้ / ควรทำอะไร content) for states A–E. Verified by direct reading: no change touches `verdictLive`, `motorCode`, `alertType` branching, `velValueText`'s computation, `motorStateText`'s mapping, the snooze/backoff check, the `escalation` branch, or the final `msg.headers`/`msg.payload` construction.
- **Health Logic:** re-diffed this turn against the currently-deployed source (`staging/health_logic_patched.js`) — **byte-identical** (`diff` produced no output). Not read for modification, not modified.
- **Unified State Engine, Escalation Timer, Auto Resume Check, 429 Handler, credentials node, test-inject buttons:** not loaded, read, or referenced anywhere in this turn's work — this patch, like V1 and V2 before it, operates on exactly two isolated `.js` source files.
- **RPM validity:** `rpm_valid`/`rpm` do not appear anywhere in the new source (confirmed by the same grep-for-absence technique used in every prior validation turn) and are re-confirmed functionally by the F/F2 byte-identity assertions.
- **Dashboard:** not read, not touched — this task is scoped entirely to the isolated LINE Message Builder copy under `docs/engineering/evidence/line_audit_20260915/staging_sme_wording/`.
- **No LINE message was sent; no network call was made** — confirmed by source inspection of the harness (no HTTP/socket-capable code).
- **No production deploy or restart was performed** as part of this task.

---

## 6. Final recommendation

```
READY FOR PRODUCTION
```

**Basis:** every wording change implements the exact operator-approved target text given for this task, verbatim — this is not a fresh redesign requiring independent design judgment, and every explicit hard rule (never NORMAL when vibration unavailable, never claim active damage from a single reading, no hard-coded thresholds, "สถานะเครื่อง" not "Health", plain-Thai technician register) is independently verified by a passing automated assertion, not merely asserted by inspection. All 86 offline assertions pass against the actual executed source. Health Logic and every other node remain byte-identical to the currently-deployed baseline, confirming this is a pure wording change with zero logic impact. Deployment itself is a separate, not-yet-authorized action and was explicitly out of scope for this task.

---

## 7. Scope confirmation

- No production Node-RED flow, firmware, API, Dashboard/frontend, nginx, or MQTT configuration was modified.
- Nothing was deployed, restarted, or reloaded. No LINE message was sent.
- No credential was created, rotated, viewed, or copied into any artifact — only the same long-standing fake placeholders were used.
- Health Logic, the Unified State Engine, recovery/dedup/escalation/snooze/ack logic, and RPM-validity behavior were not touched.
- This document and the `staging_sme_wording/` artifacts have not been committed, per instructions.
- No LINE credential or token value is reproduced anywhere in this document.

---
*Isolated-copy implementation and offline validation only. No production code, configuration, or existing evidence document was modified in producing this file. Not deployed. Not committed.*
