# รายงานสรุป: การสืบสวนความน่าเชื่อถือของ FIFO Dump บนเซนเซอร์ WitMotion WTVB05/WTVB02-485

**`FIFO Investigation — Revision 1.0 — FROZEN`**
**Freeze date:** 2026-07-09
**Status:** APPROVED — DOCUMENT FROZEN
**Change policy:** ห้ามแก้ไขเนื้อหาของ Revision 1 (หัวข้อ 1-8) อีกหลังจากจุดนี้ ข้อมูลใหม่ใดๆ ที่เกี่ยวข้องให้บันทึกเป็น **Revision 2** หรือ **Investigation Track 3** แยกต่างหาก ไม่ใช่แก้ไฟล์นี้

> **โครงสร้างเอกสาร (3 ส่วน แยกขอบเขตกันชัดเจน):**
> - **Revision 1** (หัวข้อ 1-8 ด้านล่าง) = ผลการสืบสวนและหลักฐานที่ยืนยันแล้ว (FROZEN)
> - **Investigation Track 2** (ภาคผนวกท้ายเอกสาร) = การสืบสวน TRUEPOLL ที่ยังขัดแย้งกับโมเดล ยังไม่สรุป
> - **Production Decision Addendum** (ท้ายเอกสาร ถัดจาก Track 2) = การตัดสินใจเลือก SR6 เป็น production baseline สำหรับ Phase 1 — เป็น business decision ที่ต่อยอดจาก Revision 1 ไม่ใช่ผลสืบสวนใหม่
> รายงานฉบับนี้ครอบคลุมเฉพาะข้อค้นพบที่มาจาก **`STRESSTEST` (single-request, no retry)** เท่านั้น และถือเป็น**เอกสารอ้างอิงหลัก (baseline)** สำหรับโมเดล remainder-vs-failure-rate ที่ผ่านการทดสอบเชิง predictive แล้ว
>
> ผลการทดลอง **`TRUEPOLL`** (active re-polling) ที่เริ่มแสดงความขัดแย้งกับโมเดลนี้ **ถูกแยกออกเป็นสายการสืบสวนใหม่ต่างหาก** (ดู "Investigation Track 2" ท้ายเอกสาร) แทนที่จะพยายามขยาย/ปรับโมเดลนี้ให้ครอบคลุมข้อมูลที่ยังขัดแย้งกันอยู่ — เหตุผล: ข้อมูลที่ยังไม่ลงตัวเป็นสัญญาณว่าโมเดลปัจจุบันยังไม่สมบูรณ์ ไม่ใช่เหตุผลให้ต้องแก้ทฤษฎีที่พิสูจน์แล้วอย่างแน่นหนา (Section 3) ให้ซับซ้อนขึ้นเพื่อกลบข้อมูลที่ยังขัดแย้ง
>
> การปรับปรุงโมเดลใน Section 3-4 ของเอกสารนี้จะเกิดขึ้นเมื่อ Investigation Track 2 มีข้อมูลเพียงพอที่จะบอกได้ชัดเจนว่าเป็นปรากฏการณ์แยกต่างหาก หรือเป็นส่วนขยายของโมเดลเดิมจริงๆ — **ไม่ใช่ก่อนหน้านั้น**

**หน่วยทดสอบ:** WTVB05 / WTVB02-485 (ระบุตัวเองในซอฟต์แวร์ผู้ผลิตว่า "pump01")
**Device ID:** 57e5cc9e-cf4b
**Firmware version:** 10211.1.24 (อ่านจาก WitMotion PC Sensor Configuration panel — ไม่มี register อ่านค่านี้ผ่าน Modbus)
**การเชื่อมต่อ:** RS485, Modbus RTU, 9600 baud, RS485 EN=LOW (เปิดใช้งานตลอด)
**โหมดที่ทดสอบ:** MODE=0x02 (FreqDomain), STATICDETECTION=100 (1.00x, ค่า default)

---

## 1. สรุปสำหรับผู้บริหาร (Executive Summary)

การอ่าน raw acceleration FIFO buffer (1024 ตัวอย่าง/แกน, register `0x2C`) ของเซนเซอร์นี้ **ล้มเหลวเป็นระบบ** ในบาง Sampling Rate (SR) โดยไม่ใช่ความผิดพลาดแบบสุ่ม แต่เป็นพฤติกรรมที่**ทำนายได้ล่วงหน้าอย่างแม่นยำ**ด้วยสูตรทางคณิตศาสตร์เดียว:

> **ภายใต้ methodology ของ `STRESSTEST`, Failure rate มีความสัมพันธ์ (correlation) ที่ชัดเจนกับ `(1024 ÷ SR_Hz) mod 250ms` และโมเดลนี้ทำนายผลการทดลองที่ตามมาได้ถูกต้อง** — ข้อความนี้ไม่ใช่กฎสากล เป็นข้อสรุปเฉพาะภายใต้เงื่อนไขการทดสอบที่ระบุไว้ในเอกสารนี้เท่านั้น (ดู Investigation Track 2 สำหรับข้อมูลที่เริ่ม challenge โมเดลนี้ภายใต้ methodology อื่น)

ข้อค้นพบนี้ผ่านการทดสอบเชิงสถิติ (n=100 ต่อจุดข้อมูล, รวม 8 จาก 10 ระดับ SR) และผ่านการ**ทำนายผลล่วงหน้าถูกต้องติดต่อกัน 4 ครั้ง** (SR2, SR7, SR0 และอีกจุดก่อนหน้า) ก่อนจะรันการทดลองจริง ซึ่งเป็นมาตรฐานการพิสูจน์ที่แข็งแรงกว่าการหา correlation ธรรมดา

**อย่างไรก็ตาม** การทดลองรอบล่าสุด (เปลี่ยนวิธี polling) พบ**ผลลัพธ์ที่ขัดแย้งกับโมเดลเดิมบางส่วน** ซึ่งยังไม่ได้ข้อสรุปสุดท้าย — ดูหัวข้อ 6 (Open Questions) ก่อนนำรายงานนี้ไปใช้ตัดสินใจเชิงสถาปัตยกรรม

---

## 2. พื้นหลังและโปรโตคอล

### 2.1 กลไกที่เกี่ยวข้อง

จาก WTVB05 Product Data Sheet (V260403) หัวข้อ **6.1.4.16 Original Acceleration FIFO** (register `RAWFIFO`, address `0x2C`):

- คำสั่งอ่าน: `50 03 00 2C 00 01 [CRC]` (Modbus function 0x03, 1 register)
- ถ้ายังไม่ครบ 1024 samples → เซนเซอร์ตอบ `50 03 01 XX XX [CRC]` (XX XX = ขนาดข้อมูลปัจจุบัน, "progress frame")
- ถ้าครบแล้ว → เซนเซอร์ตอบ `50 03 00 [6144 bytes accel data] [CRC]` ("full dump")
- ขนาด payload เต็ม: 3 (header) + 6144 (data) + 2 (CRC) = **6149 bytes**

Datasheet **ไม่ได้ระบุ** ว่าเซนเซอร์จะ push ข้อมูลเองโดยไม่มี request ใหม่จาก host หรือไม่ — ข้อความอ่านตรงตัวบ่งชี้ว่า host ควร poll ซ้ำจนกว่าจะได้ full dump

### 2.2 Test Harness ที่พัฒนาขึ้นระหว่างการสืบสวน

| คำสั่ง | รูปแบบ | จุดประสงค์ |
|---|---|---|
| `FIFO` | Legacy retry-loop | เดิมของโครงการ |
| `HYBRID` | 1 request + passive listen + follow-up request เดียว | ทดสอบสมมติฐานแรก (autonomous push) |
| `PURELISTEN` | 1 request + listen เฉยๆ ไม่ retry เลย | แยกคำถาม "sensor จบเองไหม" ออกจาก "retry ช่วยไหม" |
| `STRESSTEST` | รัน PURELISTEN-style ซ้ำ N ครั้ง พร้อม log ทุก timestamp | เก็บสถิติจำนวนมากเพื่อ map state machine |
| `TRUEPOLL` | Active re-poll ซ้ำทุก ~200-465ms ตามสเปกตัวอักษร | ทดสอบว่าการ poll ตามสเปกช่วยแก้ปัญหาได้จริงหรือไม่ |

---

## 3. ข้อค้นพบหลักที่ยืนยันแล้ว (High Confidence)

### 3.1 เวลาส่งข้อมูลผ่าน UART คงที่ ไม่ขึ้นกับ SR

Full dump (6149 bytes ที่ 9600 baud, 8N1 = 10 bits/byte) ใช้เวลาส่งจริง:

```
6149 × 10 ÷ 9600 ≈ 6.40 วินาที
```

วัดจากการทดลองจริงหลายสิบครั้งข้าม SR ต่างๆ ได้ค่า **6.40–6.65 วินาที เกือบทุกครั้ง** ไม่ว่า SR จะเป็นเท่าใด — ยืนยันว่านี่คือข้อจำกัดทางฟิสิกส์ของ UART ล้วนๆ ไม่ใช่พฤติกรรมของเซนเซอร์

### 3.2 Progress reporting เป็น discrete tick ทุก ~250ms ไม่ใช่ percentage ต่อเนื่อง

เมื่อ capture time (`1024 ÷ SR_Hz`) มากกว่า 1 tick period จะเห็น progress frame หลายเฟรมห่างกัน **~250ms สม่ำเสมอ** (ยืนยันข้าม SR4, SR6, SR7 ที่มี capture time ต่างกันมาก 512ms–4000ms) — จำนวน tick ที่เห็นแปรผันตรงกับ `capture_time ÷ 250ms`

### 3.3 Gap ระหว่าง progress สุดท้ายกับจุดเริ่มส่ง full dump เป็นแบบ Bimodal (Discrete State)

จากการวัด 100 รอบต่อ SR ด้วย `STRESSTEST`:

- **กรณีสำเร็จ:** Gap1 (T99 → anchor ของ full dump) = **~6ms คงที่ทุกครั้ง ไม่มีข้อยกเว้น**
- **กรณีล้มเหลว:** ไม่มี byte ใดๆ เข้ามาเลยตลอด 10 วินาที (STALLED_DUMP_STATE)

ไม่มีค่ากลางระหว่างสองกรณีนี้เลยจากหลายร้อยตัวอย่าง — บ่งชี้ว่าเป็น **state machine แบบ 2 สถานะ (สำเร็จ/ค้าง)** ไม่ใช่ความล่าช้าแบบต่อเนื่อง

### 3.4 Failure Rate สัมพันธ์กับ `(1024 ÷ SR) mod 250ms` อย่างแม่นยำ

**ตารางผลการทดสอบ (STRESSTEST, n=100 ต่อจุด ยกเว้นที่ระบุ):**

| SR | Sampling Rate | Capture Time | Remainder (mod 250ms) | Fail Rate |
|---|---|---|---|---|
| 0 | 32 kHz | 32 ms | 32 ms | 0% |
| 1 | 16 kHz | 64 ms | 64 ms | 0% |
| 2 | 8 kHz | 128 ms | 128 ms | 0% |
| **3** | **4 kHz** | **256 ms** | **6 ms** | **100%** |
| **4** | **2 kHz** | **512 ms** | **12 ms** | **16%** (n=120 รวม) |
| 5 | 1 kHz | 1024 ms | 24 ms | 0% |
| 6 | 512 Hz | 2000 ms | 0 ms (ลงตัวพอดี) | 0% |
| 7 | 256 Hz | 4000 ms | 0 ms (ลงตัวพอดี) | 0%* |
| 8 | 128 Hz | 8000 ms | 0 ms | *ไม่ได้ทดสอบ (ทำนาย 0%)* |
| 9 | 64 Hz | 16000 ms | 0 ms | *ไม่ได้ทดสอบ (ทำนาย 0%)* |

\* SR7 มีข้อมูลบางส่วน (27/100 รอบที่สังเกตได้ ทุกรอบสำเร็จ) ไม่ใช่ n=100 เต็ม

**รูปแบบ dose-response:**
```
remainder =   0ms    6ms   12ms   24ms   32ms   64ms  128ms
fail rate =   0%    100%   16%     0%     0%     0%     0%
```

กราฟนี้**ไม่ใช่ monotonic เรียบง่าย** — จุด remainder=0 (ลงตัวพอดี) ปลอดภัยสนิท แต่พอขยับออกจาก 0 เพียงเล็กน้อย (6ms) กลับพุ่งไป 100% ทันที แล้วค่อยๆ ลดกลับมา 0% เมื่อ remainder โตขึ้น — บ่งชี้ **discontinuity เฉพาะจุดใกล้ remainder=0 แต่ไม่เท่ากับ 0 พอดี**

**การยืนยันเชิง predictive:** ผลลัพธ์ของ SR0, SR2, SR7 (remainder ใหญ่ทั้งหมด) **ถูกทำนายไว้ล่วงหน้าก่อนรันการทดลองจริง** ว่าจะ 0% fail แล้วผลตรงตามคาดทั้ง 3 ครั้ง — เป็นมาตรฐานการพิสูจน์ (falsification test) ที่แข็งแรงกว่า correlation เฉยๆ

---

## 4. สมมติฐานเชิงกลไก (Mechanistic Hypothesis — ไม่ได้พิสูจน์ 100%)

**ข้อควรระวัง:** หัวข้อนี้เป็นการตีความที่**สอดคล้องกับข้อมูล** ไม่ใช่ข้อเท็จจริงที่ยืนยันแล้ว เพราะทีมไม่มี source code ของเฟิร์มแวร์เซนเซอร์ มีเพียง black-box behavior เท่านั้น

สมมติฐานที่อธิบายข้อมูลได้ดีที่สุด: เซนเซอร์อาจมี 2 กระบวนการทำงานคู่ขนานที่แชร์ทรัพยากรเดียวกัน —

1. **FIFO buffer fill** (เสร็จที่ `capture_time = 1024/SR`)
2. **Internal housekeeping tick ~250ms** (อาจเกี่ยวกับ FFT/spectrum processing ในโหมด FreqDomain)

เมื่อ buffer fill เสร็จ**ใกล้เคียง**กับจังหวะ tick (แต่ไม่ตรงกันพอดี) อาจเกิด race condition ที่ทำให้สถานะ "พร้อมส่ง" ถูกรีเซ็ต/ทับ ก่อนที่ transmission จะเริ่มได้จริง — ยิ่ง margin ระหว่างสองเหตุการณ์แคบเท่าไหร่ ยิ่งเสี่ยงชนกันบ่อยเท่านั้น

ทางเลือกอื่นที่เป็นไปได้เท่ากัน (ยังไม่ตัดออก): DMA double-buffering timing window, watchdog/interrupt priority ที่ต่างกันตาม SR, หรือ counter-overflow bug ใกล้ boundary

---

## 5. หมายเหตุ: ผลการทดลองด้วยวิธี Polling อื่น

ระหว่างการสืบสวน มีการทดลองใช้วิธี active re-polling (ส่ง request ซ้ำตามสเปกตัวอักษร แทนที่จะส่งครั้งเดียวแล้วฟัง) ซึ่ง**เริ่มแสดงผลที่ขัดแย้งกับโมเดลในหัวข้อ 3.4** (ช่วยบาง SR แต่ทำร้ายบาง SR ในทิศทางตรงข้าม) — รายละเอียดทั้งหมดถูกแยกไปไว้ใน **"Investigation Track 2"** ท้ายเอกสารนี้แทน เพื่อไม่ให้ปนกับข้อสรุปที่ยืนยันแน่นหนาแล้วในหัวข้อ 3 ดูรายละเอียดในภาคผนวก

---

## 6. คำถามเปิด (Open Questions) — เฉพาะภายในขอบเขตของโมเดล STRESSTEST

1. **Threshold ที่แท้จริงของ remainder อยู่ตรงไหน?** ทดสอบได้แค่ค่าที่ SR จริงมีให้ (0, 6, 12, 24, 32, 64, 128ms) — ไม่มีทางรู้ว่า remainder=15-20ms จะให้ fail rate เท่าไหร่ เพราะไม่มี SR ใดตกตรงนั้นพอดี
2. **SR8, SR9 ยังไม่ได้ทดสอบจริง** (ทำนายไว้ที่ 0% ตามรูปแบบ remainder=0 แต่ยังไม่ยืนยัน)
3. **กลไกภายในที่แท้จริง** (หัวข้อ 4) ยังเป็นเพียงสมมติฐาน ไม่ใช่ข้อเท็จจริงที่ยืนยันจาก source code ของผู้ผลิต

*(หมายเหตุ: คำถามเรื่องผล TRUEPOLL ที่ขัดแย้งกัน ไม่ได้อยู่ในขอบเขตของโมเดลนี้อีกต่อไป — ดู Investigation Track 2 ท้ายเอกสาร)*

---

## 7. ข้อจำกัดของ Methodology

- ทดสอบบนหน่วยเซนเซอร์จริงเพียง **1 ตัว** ("pump01") — ไม่ได้ยืนยันว่าพฤติกรรมนี้เกิดกับทุกหน่วยของรุ่นเดียวกัน
- Sample size ไม่เท่ากันทุกจุด (n=100 สำหรับ STRESSTEST ส่วนใหญ่, แต่ TRUEPOLL และ SR7/8/9 ยังไม่ครบ)
- ทดสอบที่อุณหภูมิห้องเดียว (~35-41°C ตามที่วัดได้ระหว่างทดสอบต่อเนื่อง) ไม่ได้ควบคุมอุณหภูมิแยกต่างหาก
- ไม่มีการเข้าถึง firmware source code ของผู้ผลิต — ข้อสรุปเชิงกลไกทั้งหมดเป็นการอนุมานจาก black-box behavior เท่านั้น
- ทดสอบเฉพาะ MODE=FreqDomain, STATICDETECTION=100 — ยังไม่ได้ทดสอบ MODE อื่น (Time domain low/high frequency)

---

## 8. ข้อเสนอแนะ

### สำหรับผู้ใช้งานเซนเซอร์นี้ในปัจจุบัน (Workaround)
- **หลีกเลี่ยง SR3 (4kHz)** โดยเด็ดขาดหากต้องใช้ FIFO dump — fail rate 100% ภายใต้เงื่อนไขการทดสอบนี้
- **SR4 (2kHz): ไม่แนะนำสำหรับ Production Baseline ของ Phase 1** — อัตรา fail ~16% สูงเกินมาตรฐานที่ยอมรับได้สำหรับระบบอุตสาหกรรม แม้จะเพิ่ม retry logic ระดับ application ก็ตาม — **หมายเหตุ:** อย่าเพิ่ง implement active re-polling ถี่ (แบบที่ทดลองใน Investigation Track 2) จนกว่าสายสืบสวนนั้นจะได้ข้อสรุปที่ชัดเจน เพราะมีสัญญาณเบื้องต้นว่าอาจทำให้ SR4 แย่ลงแทนที่จะดีขึ้น
- SR ที่เหลือทั้งหมด (0,1,2,5,6,7 และคาดว่า 8,9) ปลอดภัยตามข้อมูลที่มี

### สำหรับการสื่อสารกับ WitMotion
ส่งรายงานนี้พร้อมข้อมูลดิบ (CSV logs) ของตารางในหัวข้อ 3.4 เป็นหลักฐานหลัก — เน้นว่าทีมมีข้อมูลเชิงปริมาณที่ทำนายผลล่วงหน้าถูกต้อง ไม่ใช่แค่รายงานอาการทั่วไป และควรถามผู้ผลิตโดยตรงเกี่ยวกับ internal timing/tick mechanism ของ FIFO subsystem เพื่อยืนยันหรือหักล้างสมมติฐานในหัวข้อ 4

### สำหรับการสืบสวนต่อ (ถ้ามีเวลา/ทรัพยากรเพิ่ม)
- รัน TRUEPOLL ให้ครบ n=100 ทั้ง SR3 และ SR4 เพื่อยืนยัน cross-over effect ในหัวข้อ 5
- ทดสอบ SR8, SR9 ให้ครบเพื่อปิดตาราง 10/10
- ทดลองที่อุณหภูมิควบคุมต่างกัน เพื่อตัดตัวแปรอุณหภูมิออก (ข้อมูลปัจจุบันไม่ได้ควบคุมตัวแปรนี้)

---

*รายงานนี้รวบรวมจากการทดลองต่อเนื่องหลายสิบครั้ง โดยมีจำนวนตัวอย่างรวมมากกว่า 1,000 ครั้งของการอ่าน FIFO ข้าม 8 ระดับ Sampling Rate และ 5 รูปแบบ protocol ที่แตกต่างกัน*

---

## ภาคผนวก: Investigation Track 2 — Active Re-polling (TRUEPOLL)

> **สถานะ: แยกออกจาก Revision 1 โดยเจตนา — ไม่ใช่ส่วนหนึ่งของโมเดลหลัก**
> เนื้อหาในภาคผนวกนี้คือข้อมูล**preliminary**ที่ยังไม่ผ่านการยืนยันเพียงพอ ไม่ควรใช้เป็นฐานการตัดสินใจเชิงสถาปัตยกรรมหรือส่งต่อเป็นข้อสรุปให้ WitMotion จนกว่าจะมีข้อมูลครบตามที่ระบุด้านล่าง

### สิ่งที่ทดลอง

เปลี่ยนจาก "ส่ง request เดียวแล้วฟัง" (`STRESSTEST`, ใช้สร้างโมเดลใน Revision 1) เป็น "ส่ง request ซ้ำทุก ~200-465ms ตามสเปกตัวอักษรของ datasheet" (`TRUEPOLL`)

### ข้อมูลที่มีอยู่ (ยังไม่ครบ — preliminary เท่านั้น)

| SR | STRESSTEST (Rev 1 baseline) | TRUEPOLL (Track 2, ข้อมูลบางส่วน) |
|---|---|---|
| SR3 (remainder=6ms) | 0% success (n=100) | ~33% success (n=12) |
| SR4 (remainder=12ms) | 84% success (n=120) | 0% success (n=3) |

ทิศทางของผลตรงข้ามกันระหว่างสอง SR (re-poll ช่วยที่ SR3 แต่ทำร้ายที่ SR4) — **ยังไม่มีคำอธิบายที่สอดคล้องกับทั้งโมเดลในหัวข้อ 3.4 และข้อมูลนี้พร้อมกัน**

สังเกตเพิ่มเติม: ค่า progress ที่ ~99.4% ในการทดลอง TRUEPOLL ซ้ำกันเป๊ะหลายสิบ poll ติดต่อกันก่อนบางครั้งจะกระโดดไป full dump ทันที — สอดคล้องกับโมเดล discrete-state ในหัวข้อ 3.3 แต่ไม่ได้อธิบายทิศทางตรงข้ามกันของ success rate

### เงื่อนไขที่จะพิจารณารวม Track 2 กลับเข้า Revision หลัก

Track 2 จะถูกพิจารณารวมเข้าเป็นส่วนหนึ่งของโมเดลหลัก (Revision 2) ก็ต่อเมื่อ:

1. เก็บข้อมูล `TRUEPOLL` ครบ **n=100 ต่อ SR** ทั้ง SR3 และ SR4 (ปัจจุบันมีแค่ n=12 และ n=3 ตามลำดับ) เพื่อยืนยันว่า cross-over effect ที่เห็นเป็นของจริง ไม่ใช่ variance ของ sample เล็ก
2. มีสมมติฐานเชิงกลไกที่อธิบายทิศทางตรงข้ามกันระหว่าง SR3/SR4 ได้อย่างสอดคล้อง (ไม่ใช่แค่ขยายโมเดล remainder เดิมให้ครอบคลุมโดยเพิ่มเงื่อนไขทีละจุด)
3. ทดสอบ SR อื่นเพิ่มเติมด้วย `TRUEPOLL` (อย่างน้อย SR ที่ remainder=0 อีก 1-2 จุด) เพื่อดูว่า pattern ตรงข้ามกันเกิดเฉพาะ SR3/SR4 หรือเป็นวงกว้างกว่านั้น

จนกว่าจะถึงเงื่อนไขข้างต้น ให้ถือว่า **Revision 1 (หัวข้อ 1-8) เป็นเอกสารที่สมบูรณ์และใช้อ้างอิงได้ในตัวเอง** โดยไม่ต้องรอผลจาก Track 2

---

## Production Decision Addendum

> **สถานะ: แยกจาก Revision 1 และ Investigation Track 2 โดยเจตนา**
> เนื้อหาในส่วนนี้เป็น**การตัดสินใจเชิงธุรกิจ/production** ที่ต่อยอดจากหลักฐานใน Revision 1 ไม่ใช่ผลการสืบสวนใหม่ และไม่กระทบขอบเขตหรือข้อสรุปของ Revision 1 หรือ Track 2 แต่อย่างใด

### บริบท

ระหว่างการเตรียมความพร้อมสู่ production พบหลักฐานเพิ่มเติมว่าข้อจำกัดของ SR1 ไม่ได้มีแค่เรื่อง FIFO — แม้ SR1 จะผ่าน `STRESSTEST` 100/100 (ตามหัวข้อ 3.4 ของ Revision 1) แต่มีข้อจำกัดทางฟิสิกส์ที่แยกต่างหากซึ่งสำคัญกว่าสำหรับ use case ของงานนี้

### ตารางเปรียบเทียบ SR1 vs SR6 (เฉพาะรายการที่มีหลักฐานรองรับพอจะรายงาน)

| รายการ | SR1 (16 kHz) | SR6 (512 Hz) | Confidence |
|---|---|---|---|
| FIFO Stability | 100/100 | 100/100 | High |
| Dominant 1X Detection @25Hz | ไม่พบอย่างสม่ำเสมอ | พบ ~24Hz อย่างสม่ำเสมอ | High |
| Peak Magnitude | เทียบตรงไม่ได้ (scaling ต่างระบบ) | เทียบตรงไม่ได้ (scaling ต่างระบบ) | High |
| Velocity RMS | ยังสรุปไม่ได้ | ยังสรุปไม่ได้ | Medium |
| Harmonic Separation (1X/2X) | ยังไม่มีข้อมูล | ยังไม่มีข้อมูล | Low |

*หมายเหตุ: ตารางนี้ไม่รวมตัวเลข Peak/RMS แบบเทียบตรงข้าม SR เพราะ scaling ภายในของแต่ละ SR แตกต่างกัน (ตาม Revision 1 หัวข้อ 3) การแสดงตัวเลขเทียบกันตรงๆ มีความเสี่ยงที่จะทำให้ผู้อ่านเข้าใจผิดว่าเปรียบเทียบได้โดยตรง*

### ข้อสังเกตที่นำไปสู่การตัดสินใจ

During the current validation campaign, SR6 consistently reported a dominant Z-axis frequency close to the motor running speed (~24–25 Hz), while SR1 did not. This behavior is consistent with the documented measurement range for SR1 and therefore SR6 was selected as the Phase 1 production baseline for low-speed rotating machinery.

### Known Limitation (Phase 1)

- **Validation performed on one sensor unit only.**
- SR4 (2 kHz) exhibits approximately 16% RAW FIFO acquisition failures under the tested conditions.
- SR1 (16 kHz) and SR6 (512 Hz) completed 100/100 stress-test runs successfully.
- The current production baseline targets low-speed rotating machinery (~25–50 Hz). SR6 was selected because it demonstrated:
  - stable FIFO acquisition
  - repeatable dominant-frequency estimation near shaft speed
- Evaluation of higher sampling-rate modes for bearing diagnostics remains outside the scope of Phase 1.
- Investigation of SR4 remains open and is tracked separately (see Investigation Track 2 / future tracks).

### Decision Record — PD-0001

**Selected Sampling Rate:** SR6 (512 Hz)

**Reason:**
1. 100/100 FIFO success
2. Correct dominant frequency near 25 Hz
3. Suitable frequency resolution
4. Consistent with Phase 1 target machines

**Alternatives considered:**
- **SR1** — Rejected because low-speed shaft frequency could not be reliably observed.
- **SR4** — Rejected due to reproducible FIFO instability (~16%).

**SR1 disposition:** Retained as a High-Speed Diagnostic Mode for specific tasks (e.g., bearing damage detection with high-frequency components) — not the default system mode.

### Decision Record — PD-0003

**Status:** Retrospective documentation only, added during a documentation-reconciliation pass. Records established facts from source and runtime observation; does **not** assert that PD-0003 received the same review process as PD-0001, and does not invent a rationale for the change beyond what is directly evidenced.

**Selected Sampling Rate:** SR5 (1 kHz) — supersedes PD-0001's SR6 (512 Hz) selection.

**Baseline commit:** first present in Git history at `4274e524ea3568e693c3fa1e5068b7ce118599fb`, bundled alongside an unrelated, separately-closed `VERIFY_TEST` deglitch diagnostic commit whose message does not mention this change. The implementation pre-existed that diagnostic work in the working tree and was part of every compile and flash performed during that session, including the final production build.

**Implementation facts (from source, `reconfigSensorAfterRestart()`):**
- Register `REG_SAMPLE_RATE` (`0x0029`) is written with `SENSOR_SR_1K` (`0x0005`) after an unlock sequence (`REG_UNLOCK` = `0xB588`).
- A 500ms settle delay follows the write, after which the same register is read back and decoded via an SR0–SR9 lookup table, logged to Serial only (no retry, no MQTT, no struct/analytics propagation).
- This write+readback+decode sequence was directly observed succeeding — `[SENSOR-CFG] SR5 (1 kHz) (0x29=0x0005)... + OK` followed by `[SR] Readback = SR5 (1 kHz)` — across multiple independent boots during the session that also produced the deglitch causal-proof evidence, including the final production-restoration sanity check.

**Formal prior approval status:** UNKNOWN / NOT ESTABLISHED. No design document analogous to PD-0001's review process was found for this change; only in-code comment tags (`[PD-0001]` → `[PD-0003]`) record the progression.

**Validation status — distinguish from PD-0001:** PD-0001's SR6 selection was backed by the FIFO stress-test campaign documented in this report (§3.4, §"ตารางเปรียบเทียบ SR1 vs SR6"). PD-0003's SR5 selection has **no equivalent dedicated test campaign documented here or found elsewhere in the repository.** What exists is the write's own built-in self-verification (readback+decode) succeeding consistently, which is real runtime evidence that the *write itself* works, but is not evidence of a deliberate SR1-vs-SR5-vs-SR6 comparison of the kind PD-0001 underwent.

**Alternatives considered:** not evidenced — no record found of why SR5 was chosen over retaining SR6, or over SR1.

---

### Calibration Traceability Note (companion to PD-0003)

Three related production-configuration constants changed in the same baseline commit (`4274e52`), for the same machine (`plant01`/`pump01`):

| Constant | Value in current baseline | Runtime observation | Approval provenance |
|---|---|---|---|
| `NAMEPLATE_RPM` | 1800 | Confirmed printed (`RATED=1800 RPM`) and consistent with live RPM readings (~1737–1738) across this session's captures | NOT ESTABLISHED |
| `BASELINE_RMS` | 2.8 mm/s | Confirmed printed (`Baseline: 2.8 mm/s`) in boot banners this session | NOT ESTABLISHED |
| `CRITICAL_RMS` | 11.2 mm/s | Confirmed printed (`Critical: 11.2 mm/s`) in boot banners this session | NOT ESTABLISHED |

No dedicated calibration-validation record (e.g. a vibration-standard reference measurement, or a documented site survey justifying these specific values) was found in the repository. This note records that the values are present, active, and observed operating in the current baseline — it does not supply or invent an engineering justification for the specific numbers.
