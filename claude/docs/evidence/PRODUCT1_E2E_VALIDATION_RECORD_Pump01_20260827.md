# Product-1 End-to-End Validation Record — Pump01

## 1. Document Control

| Field | Value |
|---|---|
| Status | **DRAFT — uncommitted, awaiting review** |
| Date | 2026-08-27 |
| Scope owner | Session-driven investigation (Claude Code + operator), Pump01 / Product-1 Condition Monitoring |
| Document type | **Evidence record — not a design proposal, not a fix, not a recommendation to change threshold/architecture** |
| Revision | v2 — evidence classification corrected to 4-tier scheme (see below), superseding v1's 3-tier scheme |

### Evidence Classification Legend (applies throughout this document)

| Tier | Meaning |
|---|---|
| **VERIFIED BY SOURCE** | ผู้ตรวจ (Claude) อ่าน source code จริงด้วยตัวเอง (firmware `.ino`, หรือ `contract.py` ที่ operator paste raw output จากการรันคำสั่งบน production host จริง) — พิสูจน์ว่า "โค้ดควรทำงานอย่างไร" |
| **VERIFIED BY LIVE RUNTIME EVIDENCE** | มี raw output จากระบบที่กำลังรันจริง (terminal output พร้อม prompt/คำสั่งที่มองเห็นได้, HTTP response จริง, หรือเทียบเท่า) ปรากฏอยู่ในบทสนทนานี้จริง — พิสูจน์ว่า "ระบบทำงานอย่างไรจริง ณ ขณะนั้น" |
| **OPERATOR-OBSERVED / REPORTED** | Operator บอกผลที่พบ (เช่น จาก grep/query ที่รันเอง) เป็นคำอธิบาย/สรุป แต่ไม่มี raw terminal output/screenshot ปรากฏในบทสนทนานี้ให้ตรวจสอบซ้ำได้อิสระ — เป็นหลักฐานจริงที่มีน้ำหนัก แต่ต่ำกว่า raw evidence |
| **NOT VERIFIED** | ไม่มีการทำ/ไม่มีหลักฐานใดๆ เลยในบทสนทนานี้ |

---

## 2. Test Objective

พิสูจน์ data lineage และ behavioral consistency ของ Product-1 Condition Monitoring สำหรับ Pump01 ตลอดสาย:
`Firmware → MQTT /vibration → InfluxDB → API (/api/machine/plant01/pump01) → Machine Detail UI`

## 3. Scope

Pump01 เท่านั้น. Firmware, MQTT payload contract, API mapping layer (`contract.py`), และ frontend field usage ที่เกี่ยวกับ vibration/alarm/TTW. ไม่รวม threshold change, ไม่รวม UI redesign, ไม่รวม feature ใหม่.

## 4. Safety / Read-Only Constraints (ปฏิบัติตามตลอด session)

- ไม่มีการแก้ไข `.ino`, ไม่ checkout, ไม่ build, ไม่ flash, ไม่ commit
- ไม่มีการ publish ข้อมูลปลอมเข้า MQTT
- ไม่มีการเปลี่ยน threshold ใดๆ
- ไม่มีการ restart production service ใดๆ
- ไม่มีการเชื่อมต่อ/authenticate ไปยัง production server โดยตรงจาก Claude — ทุกคำสั่งที่รันบน production รันโดย operator เอง แล้ว paste ผลลัพธ์กลับมา
- **ไม่มีการจำลอง WARNING/CRITICAL condition ด้วยข้อมูลปลอม** — Phase E (alarm behavioral test) ไม่ได้ถูกทำจริงในสภาวะเครื่องจริง จึงรายงานเป็น NOT VERIFIED ทั้งหมด ตามกติกาที่กำหนดไว้

## 5. System Under Test

| Component | Identity |
|---|---|
| Device | ESP32-S3, LilyGO T-Vending S3, MAC `3c:84:27:e9:98:7c` |
| Machine | `plant01`/`pump01` |
| Production host | `ubuntu-s-2vcpu-2gb-sgp1` (Docker: `iot-stack-mosquitto-1`, `iot-stack-influxdb-1`, `iot-stack-nodered-1`, `iot-stack-api-1`, `iot-stack-grafana-1`, `nginx`, `iot-stack-tunnel-1`) |
| MQTT broker | `iot.promlogix.com:8883` (mTLS) |

---

## 6. Firmware Baseline — **VERIFIED BY SOURCE**

Source: `claude/WTVB02_ESP32S3_V16_CM_fault_latch_v3_hardened_patched_v16_5/WTVB02_ESP32S3_V16_CM_fault_latch_v3_hardened_patched_v16_5.ino`

| Item | Value | Citation |
|---|---|---|
| Canonical vibration source | FIFO accel → `VibVelocity_ComputeRms()` → `g_velCarrier` → `velocity_rms_overall/x/y/z` | .ino:9855-9921 |
| Legacy vibration source | Modbus `REG_VRMS_X/Y/Z` (0x50/0x5C/0x68), `[DEPRECATED]` | .ino:409-411, 6364-6378 |
| `WARNING ON` | `VIB_WARNING_MMS = 2.1f` | .ino:776 |
| `WARNING OFF` | `VIB_WARNING_OFF_MMS = 1.9f` | .ino:784 |
| `CRITICAL ON` | `VIB_CRITICAL_MMS = 4.5f` | .ino:777 |
| `CRITICAL OFF` | `VIB_CRITICAL_OFF_MMS = 4.2f` | .ino:785 |
| Persistence | `VIB_ALARM_PERSIST_CAPTURES = 2u` | .ino:802 |
| Alarm decision input | `velocity_rms_overall` via `readVelocityForAlarm()` | .ino:4554-4586, 7166-7167 |
| State machine (hysteresis+persistence) | full trace, host-tested 22/22 PASS (separate session artifact, not part of this document) | .ino:7203-7256 |
| `alarm_level` string mapping | .ino:9068-9083 (gate/comment 9068-9081, ternary assignment 9082-9083) | |
| TTW gate | `VIB_TTW_MIN_SLOPE` unset sentinel, never overridden anywhere in firmware | `vib_ttw.h:58-61`, `vib_ttw.cpp:32-37` |

**Firmware provenance:** HEAD `c55853a2237b8d2b9e68c59829ef2bd2d9bc0922` (canonical promotion commit; current session HEAD `57f474f...` adds only docs on top, `.ino` unchanged — confirmed by empty `git diff c55853a..HEAD -- <.ino>` in an earlier turn), source SHA256 `827137c2bac8d1455e698f20f3c3f6e9c9684772342b82061d9096ade8e4f338`.

---

## 7. Data Contract

| Field | Firmware origin | API mapping (contract.py) |
|---|---|---|
| `velocity_rms_overall` → `velocity_rms_overall_mms` | .ino:9555 | `_num(vib,"velocity_rms_overall") if vel_ok else None` — contract.py:176 |
| `velocity_rms_x/y/z` → `..._x/y/z_mms` | .ino:9556-9558 | contract.py:177-179 |
| `alarm_level` | .ino:9385/9592 | `_str(vib,"alarm_level")` — pass-through, contract.py:182 |
| `alarm_level_live` | *(not a firmware field — API-derived)* | `(vibration_status=="OK") and (motor_code==2)` — contract.py:111 |
| `vibration_status` | .ino:9508 | `_str(vib,"vibration_status")` — contract.py:106 |
| `velocity_data_valid` | .ino:9553 | `_flag(vib,"velocity_data_valid")` → `vel_ok` — contract.py:104-105 |

---

## 8. Test Environment

Production live system only — no staging/test environment used. Claude มีสิทธิ์เข้าถึงเฉพาะ: (a) local git repo บนเครื่อง Windows (source code), (b) ผลลัพธ์ที่ operator รันเองบน production host แล้ว paste กลับมา Claude ไม่มี session เชื่อมต่อ MQTT/InfluxDB/API/UI เอง ณ จุดใดใน session นี้เลย

---

## 9. Test Procedure — สถานะการทำจริงแต่ละ Phase

| Phase | Planned | ทำจริงหรือไม่ |
|---|---|---|
| A — MQTT baseline (3-5 publish cycles, raw capture) | ✅ Planned | **สำเร็จครบ** — raw `mosquitto_sub` output จริง 6 messages พร้อม prompt/command/timestamp (VERIFIED BY LIVE RUNTIME EVIDENCE — ดู §11) |
| B — InfluxDB query vs MQTT | ✅ Planned | **ไม่ได้ทำ** — operator สรุปว่า bucket/measurement ชื่ออะไรและตัวอย่างค่า แต่ไม่มี raw `influx query` output ปรากฏใน session |
| C — API GET request | ✅ Planned | **สำเร็จครบ** — ทั้ง source code (`contract.py`, VERIFIED BY SOURCE) และ live HTTP response จริง (`GET /api/machine/plant01/pump01`, raw output พร้อม prompt+command+timestamp, VERIFIED BY LIVE RUNTIME EVIDENCE — ดู §13) |
| D — UI observation | ✅ Planned | **ไม่ได้ทำ** — ไม่มี screenshot หรือค่าที่ operator จดจากหน้าจอจริง paste เข้ามา |
| E — Alarm behavioral test (NORMAL/WARNING/CRITICAL) | ✅ Planned, conditional | **ไม่ได้ทำ** — ไม่มีการเปลี่ยนสภาวะเครื่องจริง, ห้าม inject fake data ตามกติกา → **NOT VERIFIED โดยสมบูรณ์** |

---

## 10. Raw Evidence

### 10a. Firmware source (โดย Claude, ตรงจาก repo)
เก็บใน commit `c55853a`/`57f474f` — ไม่ทำซ้ำในเอกสารนี้ (อ้างอิงที่ §6)

### 10b. API source — raw terminal output ที่ operator paste เข้ามาจริง

```
$ grep -n -C 8 "alarm_level_live" /opt/iot-stack/api/contract.py
175-        "vibration": {
176-            "velocity_rms_overall_mms": _num(vib, "velocity_rms_overall") if vel_ok else None,
177-            "velocity_rms_x_mms": _num(vib, "velocity_rms_x") if vel_ok else None,
178-            "velocity_rms_y_mms": _num(vib, "velocity_rms_y") if vel_ok else None,
179-            "velocity_rms_z_mms": _num(vib, "velocity_rms_z") if vel_ok else None,
180-            "velocity_data_valid": vel_ok,
181-            "vibration_status": vibration_status,
182-            "alarm_level": _str(vib, "alarm_level"),
183:            "alarm_level_live": alarm_live,
184-            "source": "fifo_dsp",
185-        },
186-        "trend": {
187-            "window_s": 60,
188-            "status": trend_status,
189-            "samples": _num(trend, "velocity_samples_60s"),
```

```
$ sed -n '100,116p' /opt/iot-stack/api/contract.py
    motor_code = _num(vib, "motor_state")
    motor_name = MOTOR_STATES.get(motor_code)

    vel_valid = _flag(vib, "velocity_data_valid")
    vel_ok = vel_valid is True
    vibration_status = _str(vib, "vibration_status")

    alarm_live = (vibration_status == "OK") and (motor_code == 2)

    trend_status = _str(trend, "velocity_status_60s")
    trend_ok = trend_status == "VALID"
    slope_ok = _flag(trend, "velocity_slope_valid") is True
```

**สถานะ: VERIFIED BY SOURCE** — command ทั้งสองนี้รันบน production host จริงโดย operator (เห็น prompt `iotprom@ubuntu-s-2vcpu-2gb-sgp1:/opt/iot-stack$` และคำสั่งจริง) อ่านไฟล์ `contract.py` **ที่ deploy อยู่จริงบน host** ไม่ใช่ copy จาก repo อื่น — จึงเป็นหลักฐานที่หนักแน่นกว่าการอ่าน source จาก git repo เฉยๆ อย่างไรก็ตาม นี่คือหลักฐานของ **โค้ด/logic** ไม่ใช่หลักฐานของ **ข้อมูล runtime จริง** (ไม่มี HTTP response value ให้เห็น) จึงยังไม่เข้าเกณฑ์ tier "VERIFIED BY LIVE RUNTIME EVIDENCE"

**นี่คือ raw evidence 2 ชุดแรกในเอกสารนี้ที่มี terminal prompt/คำสั่งจริงปรากฏให้ตรวจสอบซ้ำได้ (tier: VERIFIED BY SOURCE — เป็นหลักฐานของ code/logic ไม่ใช่ runtime data)** — ต่อมาในเอกสารนี้ MQTT (§11) และ API live response (§13) ก็ได้ raw terminal output จริงเช่นกันและถูกจัดเป็น **VERIFIED BY LIVE RUNTIME EVIDENCE** แล้ว ส่วน InfluxDB values และ frontend field usage ที่กล่าวถึงในบทสนทนายังคงเป็นคำอธิบายจาก operator เท่านั้น (**tier: OPERATOR-OBSERVED / REPORTED** — ดู §12, §14) ไม่ใช่ "ไม่มีหลักฐานเลย" แต่ต่างระดับความหนักแน่นจาก raw terminal output

---

## 11. MQTT Evidence — **VERIFIED BY LIVE RUNTIME EVIDENCE**

**Observation timestamp:** `2026-08-27T22:16:39+07:00` (จาก `date -Is` ก่อนเริ่ม subscribe) — **6 messages** capture ได้จริง (มากกว่า 3-5 ที่ตั้งเป้าไว้)

**Raw terminal output (paste เข้ามาโดย operator, prompt+command+date+payload ครบ ไม่มี grep/filter ระหว่าง capture):**
```
iotprom@ubuntu-s-2vcpu-2gb-sgp1:/opt/iot-stack$ cd /opt/iot-stack/mosquitto/certs

echo "===== PRODUCT-1 MQTT LIVE EVIDENCE ====="
date -Is

timeout 180 mosquitto_sub -h localhost -p 8883 \
  --cafile ca.crt \
  --cert pump01.crt \
  --key pump01.key \
  --insecure \
  -t 'factory/plant01/machine/pump01/vibration' \
  -v
===== PRODUCT-1 MQTT LIVE EVIDENCE =====
2026-08-27T22:16:39+07:00
factory/plant01/machine/pump01/vibration {"plant":"plant01","machine_id":"pump01","sensor_id":"vb01","stage":"vibration","vibration_source":"fifo_dsp","vibration_status":"OK","vibration_source_legacy":"vrms_register","velocity_data_valid":true,"velocity_rms_overall":0.286,"velocity_rms_x":0.184,"velocity_rms_y":0.173,"velocity_rms_z":0.134,"rms":0.24,"vx":0.24,"vy":0.22,"vz":0.13,"peak":0.38,"temp":50,"rpm":1485.4,"current_a":0.75,"current_valid":true,"current_age_s":0.3,"motor_state":2,"operating_hours_total":106.1322,"alarm_level":"NORMAL","bearing_alert":"NORMAL","timestamp":"2026-08-27T15:16:40Z","time_synced":true}
factory/plant01/machine/pump01/vibration {"plant":"plant01","machine_id":"pump01","sensor_id":"vb01","stage":"vibration","vibration_source":"fifo_dsp","vibration_status":"OK","vibration_source_legacy":"vrms_register","velocity_data_valid":true,"velocity_rms_overall":0.279,"velocity_rms_x":0.182,"velocity_rms_y":0.174,"velocity_rms_z":0.121,"rms":0.21,"vx":0.18,"vy":0.21,"vz":0.15,"peak":0.47,"temp":50,"rpm":1485.3,"current_a":0.74,"current_valid":true,"current_age_s":0.9,"motor_state":2,"operating_hours_total":106.1403,"alarm_level":"NORMAL","bearing_alert":"NORMAL","timestamp":"2026-08-27T15:17:09Z","time_synced":true}
factory/plant01/machine/pump01/vibration {"plant":"plant01","machine_id":"pump01","sensor_id":"vb01","stage":"vibration","vibration_source":"fifo_dsp","vibration_status":"OK","vibration_source_legacy":"vrms_register","velocity_data_valid":true,"velocity_rms_overall":0.251,"velocity_rms_x":0.158,"velocity_rms_y":0.153,"velocity_rms_z":0.121,"rms":0.26,"vx":0.26,"vy":0.22,"vz":0.12,"peak":0.5,"temp":50,"rpm":1485.8,"current_a":0.74,"current_valid":true,"current_age_s":1.8,"motor_state":2,"operating_hours_total":106.1489,"alarm_level":"NORMAL","bearing_alert":"EARLY_WARNING","timestamp":"2026-08-27T15:17:40Z","time_synced":true}
factory/plant01/machine/pump01/vibration {"plant":"plant01","machine_id":"pump01","sensor_id":"vb01","stage":"vibration","vibration_source":"fifo_dsp","vibration_status":"OK","vibration_source_legacy":"vrms_register","velocity_data_valid":true,"velocity_rms_overall":0.282,"velocity_rms_x":0.177,"velocity_rms_y":0.179,"velocity_rms_z":0.128,"rms":0.25,"vx":0.2,"vy":0.25,"vz":0.17,"peak":0.38,"temp":49.9,"rpm":1484.9,"current_a":0.75,"current_valid":true,"current_age_s":0.5,"motor_state":2,"operating_hours_total":106.1572,"alarm_level":"NORMAL","bearing_alert":"EARLY_WARNING","timestamp":"2026-08-27T15:18:10Z","time_synced":true}
factory/plant01/machine/pump01/vibration {"plant":"plant01","machine_id":"pump01","sensor_id":"vb01","stage":"vibration","vibration_source":"fifo_dsp","vibration_status":"OK","vibration_source_legacy":"vrms_register","velocity_data_valid":true,"velocity_rms_overall":0.28,"velocity_rms_x":0.18,"velocity_rms_y":0.165,"velocity_rms_z":0.136,"rms":0.24,"vx":0.24,"vy":0.2,"vz":0.17,"peak":0.48,"temp":50,"rpm":1486.2,"current_a":0.71,"current_valid":true,"current_age_s":1.8,"motor_state":2,"operating_hours_total":106.166,"alarm_level":"NORMAL","bearing_alert":"EARLY_WARNING","timestamp":"2026-08-27T15:18:42Z","time_synced":true}
factory/plant01/machine/pump01/vibration {"plant":"plant01","machine_id":"pump01","sensor_id":"vb01","stage":"vibration","vibration_source":"fifo_dsp","vibration_status":"OK","vibration_source_legacy":"vrms_register","velocity_data_valid":true,"velocity_rms_overall":0.282,"velocity_rms_x":0.179,"velocity_rms_y":0.165,"velocity_rms_z":0.141,"rms":0.27,"vx":0.27,"vy":0.19,"vz":0.18,"peak":0.38,"temp":49.9,"rpm":1485,"current_a":0.74,"current_valid":true,"current_age_s":0,"motor_state":2,"operating_hours_total":106.1743,"alarm_level":"NORMAL","bearing_alert":"NORMAL","timestamp":"2026-08-27T15:19:12Z","time_synced":true}
```

**Cross-check ที่ทำจริง:**

1. **Vector-magnitude formula, ทุก message (คำนวณจริงด้วย Python):**

   | Timestamp | `velocity_rms_overall` (reported) | `sqrt(x²+y²+z²)` (calculated) | Match |
   |---|---|---|---|
   | 15:16:40Z | 0.286 | 0.285904 | ✅ |
   | 15:17:09Z | 0.279 | 0.279358 | ✅ |
   | 15:17:40Z | 0.251 | 0.251026 | ✅ |
   | 15:18:10Z | 0.282 | 0.282408 | ✅ |
   | 15:18:42Z | 0.280 | 0.279501 | ✅ |
   | 15:19:12Z | 0.282 | 0.281331 | ✅ |

   **ตรงกันทั้ง 6 message** — ยืนยันสูตร vector-magnitude (.ino:730-732, VERIFIED BY SOURCE) ด้วย live data จริง ไม่ใช่แค่ 1 ตัวอย่างเดียว

2. **`timestamp` field cadence:** 15:16:40 → 15:17:09 → 15:17:40 → 15:18:10 → 15:18:42 → 15:19:12 = ห่างกัน 29/31/30/32/30 วินาที — สอดคล้องกับ adaptive-cadence "NORMAL: 30 seconds" ที่เคย verified จาก boot banner ก่อนหน้านี้ใน session (ไม่ได้อยู่ในเอกสารนี้ แต่เป็น cross-session consistency ที่สังเกตได้)

3. **`vibration_source_legacy: "vrms_register"`** — **VERIFIED BY SOURCE เพิ่มเติม**: ตรงกับ `doc["vibration_source_legacy"] = "vrms_register";` ที่ `.ino:9513` (อยู่ใน `doc` object เดียวกับ `velocity_rms_overall` ที่ .ino:9555 และ `alarm_level` ที่ .ino:9592 — ยืนยันว่าเป็น topic เดียวกันจริง)

4. **`bearing_alert` เป็นอิสระจาก `alarm_level`:** message #3-5 มี `bearing_alert="EARLY_WARNING"` ขณะที่ `alarm_level` ยังคง `"NORMAL"` ทุก message — **ตรงกับ comment ใน source `.ino` (`[v16.3l] ปิด bearing escalation — alarmCode ใช้ RMS state machine อย่างเดียว`) ที่เคยพบก่อนหน้านี้ใน session**: bearing_alert คำนวณและ publish จริง แต่ไม่ feed เข้า alarm decision — live data ยืนยันพฤติกรรมนี้ตรงกับที่ source อธิบายไว้เป๊ะ

5. **Legacy field (`rms`/`vx`/`vy`/`vz`/`peak`) ปรากฏจริงในทุก message** พร้อมกับ field ใหม่ (`velocity_rms_*`) ในก้อนเดียวกัน — **ตรงกับ VERIFIED BY SOURCE ทุกจุด** (.ino:9563-9567 publish ทั้งสองชุดพร้อมกันเสมอ) ค่า legacy (เช่น `rms=0.24`) กับค่าใหม่ (`velocity_rms_overall=0.286`) **ต่างกันจริง** ในทุก message — ยืนยันด้วย live data ว่าเป็นคนละ metric คนละที่มาจริง ไม่ใช่ alias ของกันและกัน

6. **Cross-check กับ API live evidence (§13, observed 2026-08-27T22:08:05+07:00 = 15:08:05Z, `velocity_rms_overall_mms=0.260`):** MQTT capture นี้อยู่ที่ 15:16:40Z–15:19:12Z (~8-11 นาทีหลัง API observation) — **ไม่ถือว่าตัวเลขต้องเท่ากันเป๊ะตามที่ระบุไว้** (คนละ observation time) ค่าที่ได้ (0.251-0.286) อยู่ในช่วงใกล้เคียงกับ API (0.260) สอดคล้องกับสภาวะเครื่องเดียวกัน (RUNNING, low vibration, NORMAL) — **schema และ semantic ตรงกัน**: ทั้งสองแหล่งใช้ field name `velocity_rms_overall`(`_mms`), `vibration_status="OK"`, `alarm_level="NORMAL"`, `motor_state`/`motor_state_code=2` รูปแบบเดียวกัน ไม่มีความขัดแย้งเชิง provenance

**สรุป plant/machine/sensor identity:** `plant="plant01"`, `machine_id="pump01"`, `sensor_id="vb01"` — ตรงกับที่ระบุใน §5 ทุกจุด

## 12. InfluxDB Evidence — **OPERATOR-OBSERVED / REPORTED**

Operator รายงาน bucket=`iot_vibration`, measurement=`vibration`, ตัวอย่างค่า `velocity_rms_overall` ต่อเนื่อง (0.278, 0.293, 0.254, 0.276) จาก query จริงที่รันเอง — เป็นหลักฐานจริงจาก operator เช่นกัน แต่ไม่มี raw `influx query` terminal output ปรากฏในบทสนทนานี้ให้ตรวจสอบซ้ำได้อิสระ — จัดเป็น **OPERATOR-OBSERVED / REPORTED**

## 13. API Evidence

- **VERIFIED BY SOURCE:** field-mapping logic ใน `contract.py` (§10b, raw terminal output, อ่านจาก production host จริง) — pass-through ของ `alarm_level`/`vibration_status`, การคำนวณ `alarm_live`, hardcode ของ `"source":"fifo_dsp"`

- **VERIFIED BY LIVE RUNTIME EVIDENCE — `GET /api/machine/plant01/pump01`, observed 2026-08-27T22:08:05+07:00:**

  Raw terminal output (prompt + command + timestamp + full JSON response, paste เข้ามาโดย operator):
  ```
  iotprom@ubuntu-s-2vcpu-2gb-sgp1:/opt/iot-stack$ echo "===== PRODUCT-1 API LIVE EVIDENCE ====="
  date -Is
  curl -s http://127.0.0.1:8088/api/machine/plant01/pump01 | python3 -m json.tool
  ```
  Output timestamp: `2026-08-27T22:08:05+07:00`

  ```json
  {
      "machine": {
          "plant": "plant01",
          "machine_id": "pump01",
          "sensor_id": "vb01"
      },
      "status": {
          "motor_state": "RUNNING",
          "motor_state_code": 2,
          "online": true,
          "last_seen": "2026-08-27T15:07:10Z",
          "vibration_available": true,
          "acquisition_fault": false,
          "state": "RUNNING"
      },
      "operating_condition": {
          "rpm": 1484.1,
          "temp_c": 49.9,
          "operating_hours_total": 105.9728,
          "current_a": 0.74,
          "current_valid": true
      },
      "vibration": {
          "velocity_rms_overall_mms": 0.26,
          "velocity_rms_x_mms": 0.167,
          "velocity_rms_y_mms": 0.159,
          "velocity_rms_z_mms": 0.12,
          "velocity_data_valid": true,
          "vibration_status": "OK",
          "alarm_level": "NORMAL",
          "alarm_level_live": true,
          "source": "fifo_dsp"
      },
      "trend": {
          "window_s": 60,
          "status": "VALID",
          "samples": 16,
          "mean_mms": 0.271,
          "min_mms": 0.257,
          "max_mms": 0.288,
          "stddev_mms": 0.01139,
          "slope_mms_per_s": -4.3e-05,
          "slope_valid": true,
          "slope_reseeded": false,
          "gap_count": 11,
          "trend_gap_s": 0
      },
      "device_health": {
          "analysis_ready": true,
          "freeze_reason": "READY",
          "current_read_errors": 0,
          "current_buf_count": 120,
          "current_evidence_valid": true
      },
      "data_quality": {
          "vibration": { "age_s": 56.0, "stale": false, "stale_after_s": 90 },
          "trend": { "age_s": 10.7, "stale": false, "stale_after_s": 180 },
          "device_health": { "age_s": 86.9, "stale": false, "stale_after_s": 270 },
          "time_synced": true,
          "sync_age_s": 491
      }
  }
  ```

  **Cross-check กับ VERIFIED BY SOURCE — ทีละ field ตามที่ระบุ:**

  | Field | Live value | Cross-check ผลลัพธ์ |
  |---|---|---|
  | `velocity_rms_overall_mms` | 0.26 | `sqrt(0.167²+0.159²+0.12²) = 0.259942...` → **ตรงเป๊ะ** กับสูตร vector-magnitude ที่ VERIFIED BY SOURCE ไว้ (.ino:730-732) — คำนวณจริงด้วย python ยืนยัน |
  | `velocity_rms_x/y/z_mms` | 0.167/0.159/0.12 | ตรงกับ field mapping `contract.py:177-179` (pass-through ตรง) |
  | `vibration_status` | "OK" | สอดคล้องกับ `velocity_data_valid=true` — ตรงกับ logic `.ino:9508` (OK ก็ต่อเมื่อ `g_vibUnavailable=false`) |
  | `alarm_level` | "NORMAL" | ค่า RMS 0.26 ≪ `VIB_WARNING_MMS=2.1` — สอดคล้องกับทิศทางที่ state machine ควรให้ (.ino:7203-7233) |
  | `alarm_level_live` | true | คำนวณตรงตามสูตร VERIFIED BY SOURCE (contract.py:111): `(vibration_status=="OK") and (motor_code==2)` = `(True) and (True)` = **True — ตรงเป๊ะ** |
  | `motor_state`/`motor_state_code` | "RUNNING"/2 | ตรงกับ enum `MOTOR_RUNNING=2` (.ino:1670) |
  | `online` | true | **ไม่มี source trace** — `contract.py` snippet ที่มีอยู่ (§10b) ไม่ครอบคลุมการคำนวณ `online`/`status` block นี้ ยังไม่ยืนยันจาก source |
  | `trend.status` | "VALID" | ตรงกับ field name pattern `velocity_status_60s`→`trend_status` (contract.py:109) — ยืนยัน field name, ไม่ยืนยัน threshold/logic ที่ตัดสิน VALID |
  | `data_quality.*` | stale=false ทุกส่วน | **ไม่มี source trace** — ไม่เคยเห็นโค้ดส่วน `data_quality`/`age_s`/`stale_after_s` ใน `contract.py` snippet ที่มีอยู่ |

  **ข้อสังเกตสำคัญที่ต้องบันทึกตรงๆ:** field `status.acquisition_fault` **ไม่มีอยู่ในโค้ด firmware เลย** — grep `acquisition_fault` ทั้งไฟล์ `.ino` ได้ผลลัพธ์ว่างเปล่า (0 matches, ตรวจซ้ำแล้วในรอบนี้) แปลว่า field นี้ **ไม่ใช่ pass-through จาก firmware** ต้องเป็นค่าที่ API-layer คำนวณ/derive เองจากข้อมูลอื่น (source ยังไม่ทราบ — ไม่มี `contract.py` snippet ที่ครอบคลุมส่วนนี้) — บันทึกไว้เป็นข้อสังเกต ไม่ใช่ข้อผิดพลาด เพราะไม่มีหลักฐานขัดแย้งกัน แค่ยังไม่มี source ยืนยัน

- **NOT VERIFIED:** legacy field (`rms`/`vx`/`vy`/`vz`/`peak`) ไม่ปรากฏใน response ที่ได้จริง (ยืนยันจาก raw JSON ด้านบน — ไม่มี key เหล่านี้เลยทั้งก้อน) แต่ยังไม่ใช่การสแกนทั้งไฟล์ `contract.py` เพื่อยืนยันว่าไม่มี code path ใดที่จะ map เข้ามาได้ในกรณีอื่น

## 14. UI Evidence — **OPERATOR-OBSERVED / REPORTED**

Operator รายงานว่า `frontend/app.js` (production, `/opt/iot-stack/frontend`) อ่าน `d.vibration.velocity_rms_overall_mms`/`x/y/z_mms` เป็น vibration metric หลัก, อ่าน `d.vibration.alarm_level`/`alarm_level_live` สำหรับสถานะ, ไม่พบ TTW reference จาก grep, และไม่พบการอ้าง `rms`/`vx`/`vy`/`vz`/`peak` เป็น vibration display — **นี่คือหลักฐานจริงจาก operator ที่ตรวจสอบมาแล้ว ไม่ใช่ "ไม่มีหลักฐานเลย"** แต่ไม่มี raw `grep`/`cat` terminal output ของ `app.js` หรือ screenshot ปรากฏในบทสนทนานี้ให้ตรวจสอบซ้ำได้อิสระ — จัดเป็น **OPERATOR-OBSERVED / REPORTED**

## 15. Alarm Behavioral Evidence — **NOT VERIFIED**

Phase E ไม่ได้ถูกทำจริง — ไม่มีการเปลี่ยนสภาวะเครื่องจริงเพื่อทดสอบ WARNING (≥2.1mm/s, 2 captures), CRITICAL (≥4.5mm/s, 2 captures), หรือ de-escalation ต่ำกว่า OFF thresholds เลยตลอด session นี้ ตามกติกาที่ห้าม fabricate — จึงไม่มีข้อมูลใดๆ ในหัวข้อนี้ที่ตรวจสอบได้จากสภาวะจริง

---

## 16. Data Lineage Diagram

```
FIFO accel (g_accelWork.x/y/z)
   ↓  VibVelocity_ComputeRms()                    [.ino:9856-9857]  VERIFIED BY SOURCE
g_velCarrier.overall/x/y/z                          [.ino:9912-9921] VERIFIED BY SOURCE
   ↓  readVelocityForAlarm()  ─────────────► Alarm state machine    [.ino:4554, 7203-7256] VERIFIED BY SOURCE
   ↓  publishTelemetry() doc["velocity_rms_*"]      [.ino:9555-9558] VERIFIED BY SOURCE
MQTT /vibration  ─────────────────────────────────────────────────  VERIFIED BY LIVE RUNTIME EVIDENCE
                                                                       (6 raw messages, vector-magnitude match
                                                                       6/6, 2026-08-27T22:16:39+07:00, §11)
   ↓
InfluxDB  ─────────────────────────────────────────────────────────  OPERATOR-OBSERVED / REPORTED (§12)
   ↓
contract.py: vel_ok/vibration_status/alarm_live/response mapping    [contract.py:104-184] VERIFIED BY SOURCE
GET /api/machine/plant01/pump01 (live response)  ──────────────────  VERIFIED BY LIVE RUNTIME EVIDENCE
                                                                       (raw JSON, 2026-08-27T22:08:05+07:00, §13)
   ↓
Machine Detail UI (frontend/app.js)  ───────────────────────────────  OPERATOR-OBSERVED / REPORTED (§14)
```

ทุก layer มีหลักฐานอย่างน้อยระดับ OPERATOR-OBSERVED และไม่มีจุดใดที่ operator รายงานขัดแย้งกับสิ่งที่ VERIFIED BY SOURCE พิสูจน์ไว้ — gap ที่เหลือคือระดับความหนักแน่นของหลักฐาน (raw vs reported) ไม่ใช่การขาดหลักฐานโดยสิ้นเชิง

---

## 17. Pass / Fail / Not Verified Matrix

| # | Item | Status |
|---|---|---|
| 1 | Identity (plant01/pump01) | OPERATOR-OBSERVED / REPORTED |
| 2 | Device online state | OPERATOR-OBSERVED / REPORTED |
| 3 | Motor state = RUNNING | OPERATOR-OBSERVED / REPORTED (value); VERIFIED BY SOURCE (mapping logic, contract.py:100) |
| 4 | Canonical FIFO-DSP vibration lineage (firmware→API→live response) | **VERIFIED BY SOURCE + VERIFIED BY LIVE RUNTIME EVIDENCE** — vector-magnitude cross-check exact match (§13) |
| 5 | Alarm level pass-through, not recomputed | **VERIFIED BY SOURCE** (contract.py:182) **+ VERIFIED BY LIVE RUNTIME EVIDENCE** (live value `NORMAL`, §13) |
| 6 | Alarm live validity guard | **VERIFIED BY SOURCE** (contract.py:111) **+ VERIFIED BY LIVE RUNTIME EVIDENCE** (live `alarm_level_live=true` matches formula output exactly, §13) |
| 7 | Trend | VERIFIED BY LIVE RUNTIME EVIDENCE for field presence/value (`status=VALID`, §13); field-naming only VERIFIED BY SOURCE; **VALID-determination logic itself NOT VERIFIED** (threshold/logic not seen) |
| 8 | Data freshness handling | VERIFIED BY SOURCE for the *firmware-side* mechanism (`velocity_data_valid` gate, .ino:9553); **VERIFIED BY LIVE RUNTIME EVIDENCE** that `velocity_data_valid=true`, `data_quality.vibration.stale=false`, `age_s=56.0` at 2026-08-27T22:08:05+07:00; `data_quality` computation logic itself NOT VERIFIED (no source seen) |
| 9 | Legacy field isolation (API/UI) | **VERIFIED BY LIVE RUNTIME EVIDENCE at both ends**: MQTT payload DOES publish legacy fields (`rms`/`vx`/`vy`/`vz`/`peak`) in every message (§11, 6/6), yet the API response (§13) contains zero legacy keys — confirms the API layer filters them, not that firmware omits them; OPERATOR-OBSERVED that frontend also doesn't read them (§14) |
| 10 | UI presentation | OPERATOR-OBSERVED / REPORTED (§14) |
| 11 | No fabricated values | Upheld by design this session (no fake MQTT injected) — **VERIFIED BY SOURCE** (source = this session's own read-only action record, not firmware/`contract.py`) — a compliance claim, not a data-correctness finding |
| 12 | No stale held-alarm shown as live | **VERIFIED BY SOURCE** (contract.py:108-111) **+ VERIFIED BY LIVE RUNTIME EVIDENCE** (observed instance is genuinely live: `vibration_status=OK`, not a held/stale state) |

**ไม่มีรายการใดในตารางนี้ที่เขียนว่า "PASS" เพราะยังไม่มีหลักฐานครบทุก layer ถึงระดับ VERIFIED BY LIVE RUNTIME EVIDENCE — ใช้ 4-tier scheme (VERIFIED BY SOURCE / VERIFIED BY LIVE RUNTIME EVIDENCE / OPERATOR-OBSERVED-REPORTED / NOT VERIFIED) ตลอดทั้งเอกสาร ตามกติกาข้อ "ห้ามเขียนคำว่า PASS ถ้าไม่มี evidence รองรับ" — หมายเหตุ: รายการส่วนใหญ่มีหลักฐานอย่างน้อยระดับ OPERATOR-OBSERVED ไม่ใช่ "ไม่มีหลักฐานเลย"**

---

## 18. Deviations / Known Limitations

- มีหลักฐานระดับ raw terminal output รวมทั้งหมด 4 ชุดในเอกสารนี้: 2 ชุดเป็น **VERIFIED BY SOURCE** (`contract.py` source code ที่อ่านจาก production host จริง, §10b) และอีก 2 ชุดเป็น **VERIFIED BY LIVE RUNTIME EVIDENCE** (MQTT §11 และ API live response §13)
- InfluxDB และ frontend/UI มีหลักฐานระดับ **OPERATOR-OBSERVED / REPORTED** — เป็นคำยืนยันจริงจาก operator ที่ตรวจสอบมาแล้ว แต่ไม่มี raw terminal output/screenshot ให้ตรวจสอบซ้ำได้อิสระในบทสนทนานี้ ไม่ควรตีความว่า "ไม่มีหลักฐาน" แต่ควรตีความว่า "มีหลักฐานชั้นรองลงมา" (MQTT ถูกอัปเกรดเป็น **VERIFIED BY LIVE RUNTIME EVIDENCE** แล้ว — ดู §11 — ไม่อยู่ในกลุ่มนี้อีกต่อไป)
- ไม่พบความขัดแย้งใดๆ ระหว่างสิ่งที่ operator รายงานกับสิ่งที่ VERIFIED BY SOURCE พิสูจน์ไว้จาก firmware/API source code
- Phase E (behavioral alarm test) ไม่ได้ทำเลย ตามกติกาความปลอดภัย — ยังคง NOT VERIFIED โดยสมบูรณ์ ไม่มีข้อยกเว้น

## 19. TTW Status — **VERIFIED BY SOURCE**

`VIB_TTW_MIN_SLOPE` คงค่า sentinel (-1.0f) เพราะไม่มีจุดใดใน firmware ที่ `#define` ทับค่า default (`vib_ttw.h:58-61`) ส่งผลให้ `VibTtw_Compute()` คืนค่า `VIB_TTW_THRESHOLDS_UNSET` เสมอ (`vib_ttw.cpp:32-37`) — เป็น **intended behavior** ตาม engineering rationale ที่บันทึกไว้ใน `vib_ttw.h:34-57` (ต้อง re-baseline บนเครื่องที่มีโหลดจริงก่อน ไม่ใช่ bug) TTW **ไม่ใช่ส่วนหนึ่งของ current live alarm decision** ของ Product-1

## 20. Legacy Field Handling — **VERIFIED BY SOURCE + VERIFIED BY LIVE RUNTIME EVIDENCE**

Firmware publish `rms`/`vx`/`vy`/`vz`/`peak` ในทุก MQTT payload จริง (VERIFIED BY SOURCE, .ino:9563-9567) มาจาก Modbus VRMS register, ทำเครื่องหมาย `[DEPRECATED]` ชัดเจน (.ino:409-411) — **ยืนยันแล้วด้วย live MQTT capture จริง (§11, 6/6 messages)**: ทุก message มี `rms`/`vx`/`vy`/`vz`/`peak` อยู่จริง ค่าต่างจาก `velocity_rms_*` ทุกครั้ง ยืนยันว่าเป็นคนละ metric จริง ไม่ใช่ alias

**จุดสำคัญที่ค้นพบจากการเทียบ MQTT (§11) กับ API (§13):** legacy field **มีอยู่ใน MQTT payload แต่ไม่ปรากฏใน API response เลย** — สรุปได้ว่า**การกรอง legacy field ออกเกิดขึ้นที่ API layer (`contract.py`)** ไม่ใช่ firmware ไม่ publish (firmware publish จริง, API เลือกไม่ pass-through) เป็นหลักฐานเชิง data-lineage ที่สมบูรณ์กว่าที่เคยบันทึกไว้ในเวอร์ชันก่อนหน้าของเอกสารนี้ (v2 ระบุแค่ "ไม่ปรากฏใน API response" โดยไม่ทราบว่า filter เกิดที่ layer ไหน)

Frontend ไม่อ่าน legacy field เลย ยังคงเป็น OPERATOR-OBSERVED / REPORTED (§14) — ไม่มีการเปลี่ยนแปลง

## 21. Final Product-1 Validation Status

**PARTIAL** — ดูสรุปท้ายเอกสาร

## 22. Exact Commands Used

**โดย Claude (บนเครื่อง local, ตรวจ firmware source):** `grep`/`sed`/`git show`/`git diff` มาตรฐานตลอด session (ไม่ทำซ้ำที่นี่ — เป็น read-only source inspection)

**โดย operator (บน production host, paste raw output กลับมา):**
```bash
grep -n -C 8 "alarm_level_live" /opt/iot-stack/api/contract.py
sed -n '100,116p' /opt/iot-stack/api/contract.py
echo "===== PRODUCT-1 API LIVE EVIDENCE ====="
date -Is
curl -s http://127.0.0.1:8088/api/machine/plant01/pump01 | python3 -m json.tool
echo "===== PRODUCT-1 MQTT LIVE EVIDENCE ====="
date -Is
timeout 180 mosquitto_sub -h localhost -p 8883 \
  --cafile ca.crt --cert pump01.crt --key pump01.key --insecure \
  -t 'factory/plant01/machine/pump01/vibration' -v
```

## 23. Timestamp of Each Observation

- Firmware source inspection: session date 2026-08-27 (ไม่มี "live" timestamp เพราะเป็นการอ่าน static source)
- API source (`contract.py`) raw output: paste เข้ามาระหว่าง session นี้, 2026-08-27 — ไม่มี timestamp ของระบบ production ติดมาด้วย (เป็นแค่เนื้อหาไฟล์ ไม่ใช่ live data)
- API live data (**VERIFIED BY LIVE RUNTIME EVIDENCE**): observation timestamp `2026-08-27T22:08:05+07:00` (จาก `date -Is` ในคำสั่งเดียวกับ `curl`), field `status.last_seen = 2026-08-27T15:07:10Z` ภายใน response เอง (UTC) — ทั้งสอง timestamp สอดคล้องกัน (22:08:05+07:00 = 15:08:05Z ≈ 15:07:10Z last_seen, ห่างกัน ~55s ซึ่ง ≈56s, within 1s เทียบกับ `data_quality.vibration.age_s=56.0` (คลาดเคลื่อน ~1 วินาที สอดคล้องกับ serialization/measurement latency ไม่ใช่ค่าที่ตรงกันเป๊ะ) — ดู §13
- MQTT live data (**VERIFIED BY LIVE RUNTIME EVIDENCE**): observation timestamp `2026-08-27T22:16:39+07:00`; 6 payload timestamps `2026-08-27T15:16:40Z` ถึง `15:19:12Z` (คาบ ~30s ต่อกัน) — ดู §11
- InfluxDB/UI: **ไม่มี timestamp เพราะไม่มี raw capture**

## 24. Git Commit / Firmware Provenance References

```
Production HEAD (canonical promotion):  c55853a2237b8d2b9e68c59829ef2bd2d9bc0922
Current session HEAD (docs-only on top): 57f474f0e8a048a5cab7c186f0db734d742d4802
Source SHA256:  827137c2bac8d1455e698f20f3c3f6e9c9684772342b82061d9096ade8e4f338
Binary SHA256:  62c812d948fc18f217dd2d97a13a452963b4a50cb7846d4800a5aec4cf05dfd0
```

## 25. Evidence Hashes

- Firmware `.ino` source SHA256: ดู §24 (ยืนยันแล้วหลายครั้งตลอด session)
- `contract.py`: **ไม่มี hash** — ไม่เคยได้รับไฟล์เต็มหรือ hash ของไฟล์นี้ มีแค่ 2 snippet ที่ paste มา
- `frontend/app.js`: **ไม่มี hash** — ไม่เคยเห็นไฟล์นี้เลย
- API live JSON response (§13, 2026-08-27T22:08:05+07:00): SHA256 ของเนื้อหา JSON ที่บันทึกไว้ในเอกสารนี้ (minified) = `b569abccb25970b68b74b357b2a1e0c5b91c201265411ce683e9a24bfe5715ae` — **หมายเหตุ:** นี่คือ hash ของสำเนาที่บันทึกในเอกสารนี้เพื่อตรวจจับการแก้ไขภายหลัง ไม่ใช่ hash ที่พิสูจน์ความถูกต้องของ terminal output ต้นฉบับ (ไม่มีการเก็บ byte-exact raw capture แยกไว้นอกเอกสารนี้)
- MQTT live capture (§11, 2026-08-27T22:16:39+07:00, 6 messages, `factory/plant01/machine/pump01/vibration`): SHA256 ของ **6 JSON payload objects เท่านั้น** — คำนวณโดย (1) ตัด topic-name prefix `factory/plant01/machine/pump01/vibration ` ออกจากแต่ละบรรทัดใน §11 ก่อน เหลือเฉพาะ JSON object ที่ขึ้นต้นด้วย `{` และจบด้วย `}`, (2) เรียงตามลำดับที่บันทึกใน §11, (3) join ด้วย `\n`, (4) ไม่รวม prompt/command/date line ใดๆ = `ff31cec7ab9e57b8d15b329853bc44d16db081f02f8057897e85605e7505bd20` — **หมายเหตุ:** เช่นเดียวกับ API hash ด้านบน นี่คือ hash ของสำเนาที่บันทึกในเอกสารนี้เพื่อตรวจจับการแก้ไขภายหลัง ไม่ใช่ hash ที่พิสูจน์ความถูกต้องของ terminal output ต้นฉบับ — **หมายเหตุเพิ่มเติม (reproducibility):** หากคำนวณจาก 6 บรรทัดเต็มใน §11 โดยไม่ตัด topic-name prefix ออกก่อน จะได้ hash คนละค่ากับที่บันทึกไว้นี้ ต้อง strip prefix ก่อนเสมอจึงจะ reproduce hash นี้ได้ตรงกัน

---

## สรุปท้ายเอกสาร

**OVERALL STATUS = PARTIAL**

เหตุผล: firmware, MQTT layer, และ API layer (ทั้ง source code และ live raw evidence จริง) verified ถึงระดับสูงสุดแล้ว — **VERIFIED BY SOURCE + VERIFIED BY LIVE RUNTIME EVIDENCE** พร้อม cross-check ทางคณิตศาสตร์และ timestamp ที่สอดคล้องกันเป๊ะ ทั้งใน §11 (MQTT, 6 raw messages, vector-magnitude ตรงทั้ง 6) และ §13 (API, 1 raw response) InfluxDB และ UI ยังอยู่ที่ระดับ **OPERATOR-OBSERVED / REPORTED** เท่านั้น — เป็นหลักฐานจริงที่มีน้ำหนักและไม่ขัดแย้งกับสิ่งที่ verified แต่ยังไม่มี raw capture ระดับเดียวกับที่ได้จาก MQTT และ API และ Phase E (behavioral alarm test — NORMAL/WARNING/CRITICAL ด้วยสภาวะเครื่องจริง) ยังคง **NOT VERIFIED โดยสมบูรณ์** เพราะไม่ได้ทำจริงตามกติกาความปลอดภัย — ด้วยเหตุนี้ overall status จึงยังเป็น PARTIAL ไม่ใช่ PASS แม้ว่า data lineage ทั้ง 5 hop (Firmware→MQTT→InfluxDB→API→UI) จะมี Firmware อยู่ในระดับ **VERIFIED BY SOURCE** (อ่าน source code — ไม่ใช่ raw runtime evidence) และมี MQTT กับ API รวม 2 ใน 5 hop ที่ verified ด้วย **VERIFIED BY LIVE RUNTIME EVIDENCE** (raw evidence จริง) แล้วก็ตาม

**สิ่งที่ยังต้องทดสอบจริงที่สุด 1 อย่าง (อัปเดตหลังได้ MQTT live evidence แล้ว):**
Phase E — behavioral alarm test คือช่องว่างเดียวที่เหลืออยู่ซึ่งไม่มีทางปิดได้ด้วยการอ่านข้อมูลเฉยๆ (ต้องเปลี่ยนสภาวะเครื่องจริงอย่างปลอดภัยเพื่อดันค่า vibration ให้ถึง `VIB_WARNING_MMS=2.1mm/s` อย่างน้อย 2 captures ติดกัน แล้วสังเกตว่า `alarm_level` เปลี่ยนเป็น `WARNING` จริงหรือไม่ผ่าน API เดียวกันนี้) — ถ้าไม่สามารถสร้างสภาวะจริงได้อย่างปลอดภัย ให้คง Phase E เป็น NOT VERIFIED ต่อไปตามกติกา ไม่ต้อง fabricate
