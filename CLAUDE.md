# CLAUDE.md — ESP32-S3 Vibration Condition-Monitoring Firmware

## Architecture
ไฟล์หลัก: WTVB02_ESP32S3_V16_CM_fault_latch_v3_hardened_patched_v16_3v.ino (~6,600 บรรทัด, Arduino .ino)
ฮาร์ดแวร์: LilyGO T-Vending S3 (ESP32-S3) + SIMCom A7670 4G + WTVB02 vibration sensor (Modbus RTU/RS485)
FreeRTOS 2 core:
- Core 0 (time-critical): Modbus task (อ่านเซนเซอร์ 4Hz) + State machine task (RPM/motor state)
- Core 1: Display(OLED) + Network(4G/mTLS MQTT) + Analytics + Button + Buzzer
Telemetry: MQTT mTLS 5 topic — /vibration /sensor /decision /trend /event

## Workflow Rules (บังคับทุกครั้ง — อ่านก่อนแตะโค้ด)
Never assume the project builds.
Always perform a clean build before diagnosing or modifying code.

ก่อนแก้โค้ดใด ๆ:
1. Clean build โปรเจกต์ปัจจุบัน
2. ยืนยันว่า build ผ่าน
3. อธิบาย root cause
4. เสนอ implementation plan
5. รอ approval ก่อนลงมือแก้

หลังแก้ทุกครั้ง:
1. Build ใหม่
2. รายงานไฟล์ทั้งหมดที่แก้
3. อธิบายว่าทำไมแต่ละการแก้จำเป็น
4. สร้าง git commit message สั้น กระชับ
5. หยุด อย่าเดินหน้าไปงานถัดไปเองอัตโนมัติ

## Scope Discipline
- ห้าม large-scale refactoring เว้นแต่ผมสั่งชัดเจน
- เลือกการแก้ที่เล็กที่สุดที่แก้ปัญหาปัจจุบันได้ (smallest possible change)
- แก้ทีละงาน commit ทีละ checkpoint

## Cross-core Precautions
- Core 0 เขียน: g_trendBuf, g_motorRunState, g_velPeakHold
- Core 1 เขียน: g_buf1s/10s/60s, EMA
- สื่อสาร Core0→Core1 ใช้ command enum (AnalyticsCommand_t) / atomic — อย่าใช้ shared boolean flag ที่มีหลาย writer
- float/enum single-word อ่าน-เขียนข้าม core ได้ (atomic บน Xtensa) แต่ struct ต้องผ่าน mutex/queue

## Coding Style & Conventions
- Comment tag version ทุกการแก้ เช่น // [v16.3ae] <เหตุผล>
- คง backward-compat ของ MQTT payload (อย่าลบ field เดิม — ถ้าเปลี่ยนความหมายให้เพิ่ม field ใหม่ + deprecate ของเก่า)
- .ino auto-prototype: custom enum/struct ที่เป็น return type หรือ parameter ต้อง typedef ก่อน function แรกของไฟล์

## Build
- arduino-cli, FQBN: esp32:esp32:esp32s3 (ปรับ PSRAM/flash ตามบอร์ด LilyGO)

## Feature History (tag ที่ทำไปแล้ว — อย่าทำซ้ำ)
- v16.3z: motor RUNNING warm-up debounce (rpm in-band 2.5s ก่อนเป็น RUNNING)
- v16.3y: VRMS single-sample de-glitch (gate ด้วย RUNNING) + deglitch_count
- v16.3aa/ab: analytics FREEZE เมื่อไม่ RUNNING, resume reinit (EMA reseed + slope suppress), trend_gap_s
- v16.3ab: analysisReady = derived state (predicate แยกโดเมน → analysisReason() enum + freeze reason log)
- v16.3ac: AnalyticsCommand_t enum แทน boolean flag
- v16.3ad: thermal cold-start clear (COLD_START_TEMP_DROP_C) + configurable trend persistence (NVS)