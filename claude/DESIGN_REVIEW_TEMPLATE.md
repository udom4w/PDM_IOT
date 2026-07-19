# Design Review Template

โครงสำหรับทุก task ที่แก้ไข logic ที่กระทบ state machine, alarm, telemetry,
หรือ analytics pipeline อ่านคู่กับ `DESIGN_PRINCIPLES.md` ก่อนเริ่ม

Copy โครงนี้ไปวางในแต่ละ task prompt แล้วเติมส่วน "บริบท" ตามปัญหาจริง
เมื่อได้ Decision Record แล้ว ให้บันทึกไฟล์ผลลัพธ์ไว้ที่ `design_reviews/YYYY-MM-DD-slug.md`

## Header (บังคับทุกไฟล์ใน design_reviews/)

ใส่ 5 บรรทัดนี้บนสุดของทุกไฟล์ result เพื่อให้เห็นสถานะโดยไม่ต้องเปิดอ่านทั้งไฟล์:

```
Status: Proposed | Approved | Implemented | Verified | Rejected
Related commit:
Related issue:
Reviewer:
Date:
```

---

## กฎ (ตาม CLAUDE.md — ห้ามข้าม)
Never assume the project builds. Clean build ก่อนวิเคราะห์เสมอ.
ห้ามแก้โค้ดจนกว่าจะได้ approval. Smallest possible change.

## หลักการวิเคราะห์
แยก "ข้อเท็จจริงจากโค้ด" ออกจาก "สมมติฐาน" ให้ชัดในทุกข้อด้านล่าง
ถ้าพิสูจน์ไม่ได้ ให้บอกว่าพิสูจน์ไม่ได้ แล้วอธิบายเฉพาะ behavior ที่เกิดขึ้นจริง

## ขั้นตอน (หยุดหลังข้อสุดท้าย รอ approval เสมอ)

1. **Clean build** — ยืนยันว่า HEAD ปัจจุบัน build ผ่าน

2. **[FACTS]** อธิบายลำดับเหตุการณ์/อาการที่เกิดขึ้นจริง (จาก log/repro) เป็นข้อ ๆ
   ห้ามตีความหรือสรุปสาเหตุในขั้นนี้ — แค่บันทึกสิ่งที่สังเกตได้

3. **[FACTS]** อ่าน implementation ที่เกี่ยวข้อง อธิบายว่าโค้ดทำงานอย่างไรจริง ๆ
   (ไม่ใช่ควรทำงานอย่างไร) พร้อมอ้าง path/บรรทัดที่เกี่ยวข้อง

4. **[FACTS]** สำหรับ field/ตัวแปรที่ชื่อไม่ชัดเจนหรือกำกวม ให้ไปอ่าน source
   ว่าคำนวณ/นิยามจากอะไรจริง — ห้ามเดาความหมายจากชื่อ

5. **[INTENT]** ตรวจสอบเจตนาของ implementation เดิม จาก source + commit history
   (ถ้ามี) ถ้าหาหลักฐานไม่ได้ ให้ระบุว่าสรุป intent ไม่ได้
   → ระบุให้ชัด: สิ่งที่จะทำต่อไปคือ **bug fix** หรือ **behavior change**
     (ดู Principle #3 — behavior change ต้องมีเหตุผลหนักแน่นกว่า)

6. **[INVARIANTS]** ระบุ system invariants ที่ patch นี้ต้องรักษาไว้
   ก่อนเสนอทางแก้ใด ๆ พร้อมอ้างอิง source code ถ้าเป็นไปได้
   ถ้า invariant ใดเป็นข้อเสนอ ไม่ใช่ข้อเท็จจริงจากโค้ด ให้ label ว่า
   **"Design Assumption"** แยกจาก invariant ที่ยืนยันจากโค้ดได้จริง

7. **[OPTIONS + TRADE-OFFS]** เสนอแนวทางแก้ 1–3 แบบ แต่ละแบบระบุครบ:
   - ข้อดี
   - ข้อเสีย
   - ความเสี่ยง **false negative** โดยเฉพาะ (ไม่ใช่แค่ false positive ที่กำลังแก้)
   - ค่า/สถานะที่จะป้อนเข้าระบบถัดไป (state machine/analytics/telemetry) จะเป็นอะไร
   - พิสูจน์ว่า design นี้รักษา invariant จากข้อ 6 ครบทุกข้อ —
     ถ้าแลก invariant ใด ต้องระบุชัดว่าแลกอันไหน เพื่ออะไร
   แล้วสรุปเป็นตารางตัดสินใจ:

   | Option | แก้ปัญหา | Invariants | ความเสี่ยง | Recommendation |
   |---|---|---|---|---|
   | A | ... | รักษาครบ / ละเมิด # | ... | Recommended / Not recommended / Optional |

8. **[DECISION RECORD]**
   - **Recommended option:** ___ — **เหตุผล:** ___
   - **Rejected options:** ___ — **เหตุผลที่ไม่เลือก:** ___
   - **Open questions:** สิ่งที่ยังพิสูจน์ไม่ได้และต้องตรวจเพิ่มก่อนลงมือแก้

**หยุดที่นี่ รอ approval ก่อนแก้โค้ด**

## ขอบเขต (เติมตาม task)
- อย่าแตะ [subsystem อื่นที่ไม่เกี่ยว] ในงานนี้ — แยกเป็น task ต่างหาก
- อย่าเปลี่ยน [algorithm/architecture อื่น] ในงานนี้ — แยกเป็น task ต่างหาก
- โฟกัสแค่: [ขอบเขตของ task นี้]

## Exit Criteria

Task นี้ถือว่าเสร็จ (พร้อมให้ approve) เมื่อ:

- [ ] Build ผ่าน
- [ ] Facts ครบ (ข้อ 2–4)
- [ ] Intent ระบุแล้ว หรือระบุชัดว่าพิสูจน์ไม่ได้ (ข้อ 5)
- [ ] Invariants ระบุครบ พร้อมแยก fact/assumption (ข้อ 6)
- [ ] Options เปรียบเทียบครบ ทั้ง false positive และ false negative (ข้อ 7)
- [ ] Decision Record ครบ (ข้อ 8)
- [ ] Awaiting approval — ยังไม่มีการแก้โค้ด

ใช้ checklist นี้เพื่อดูสถานะงานได้ทันทีโดยไม่ต้องอ่านทั้งเอกสาร
