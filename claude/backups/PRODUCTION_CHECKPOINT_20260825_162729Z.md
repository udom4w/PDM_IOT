# PRODUCTION BASELINE CHECKPOINT

**Checkpoint timestamp:** 2026-08-25 16:27:29Z
**Machine:** Plant01 / Pump01
**Status:** LIVE, VERIFIED, RUNNING

---

## 1. Device identity

| Field | Value |
|---|---|
| Chip | ESP32-S3 (QFN56), revision v0.2 |
| MAC | `3c:84:27:e9:98:7c` |
| Local connection | COM5 (USB-Serial/JTAG) |
| Flash | 16MB, quad mode, 3.3V |
| PSRAM | 8MB embedded (AP_3v3) |
| Hardware | LilyGO T-Vending S3 + SIMCom A7670 4G + WTVB02 vibration sensor + CTR4A01 current sensor |

## 2. Firmware provenance

| Field | Value |
|---|---|
| Source file | `WTVB02_ESP32S3_V16_CM_fault_latch_v3_hardened_patched_v16_5.ino` |
| Source SHA256 | `224e159001a93c5e1c4cbc7322aab6e5635c807ed9763694080494c8bd5e73c8` |
| Application binary SHA256 (`.ino.bin`) | `74A2B97E3AF5E7504462C3F77AB26B68A83B330B0C84297E7296E18C793606C3` |
| Build directory | `backups\build_s21_plus_currentfresh_20260825_160746Z` |
| FQBN | `esp32:esp32:esp32s3:CDCOnBoot=cdc,CPUFreq=240,FlashMode=qio,FlashSize=16M,PSRAM=opi,PartitionScheme=app3M_fat9M_16MB,USBMode=hwcdc,UploadMode=default,UploadSpeed=921600` |
| Firmware version string | `16.5` |
| Git baseline | `b603c2f` (last commit) + 208 insertions / 37 deletions, uncommitted working-tree diff |
| This checkpoint's source backup | `backups\WTVB02_v16_5.ino_PRODUCTION_CHECKPOINT_20260825_162729Z` (hash-verified match to the above source SHA256) |

## 3. Changes included in this build (vs. commit `b603c2f`)

**A. Current Telemetry Freshness fix** (49 insertions / 19 deletions)
- Hoisted `CURRENT_EVIDENCE_MAX_AGE_MS` (5000ms) to file scope for reuse.
- `publishTelemetry()`: exported `current_valid` changed from a raw per-250ms-tick flag to genuine age-based freshness, derived from `g_lastCurrentSampleMs` and the 5s window.
- Added new additive field `current_age_s` to both `/sensor` and `/vibration` MQTT payloads.
- `current_a` behavior unchanged — always the last successfully measured value, never fabricated to 0.
- Root cause fixed: exported `current_valid` no longer flickers false on ticks where no new 500ms sample was taken.

**B. S21/S21b — PROMLOGIX Provisional V1 vibration thresholds** (159 insertions / 18 deletions)
- `VIB_WARNING_MMS = 2.1f`, `VIB_CRITICAL_MMS = 4.5f` (ISO-informed, not ISO-certified — provisional product values, replacing `VIB_THRESHOLD_UNSET` placeholders).
- Hysteresis de-escalation edges: `VIB_WARNING_OFF_MMS = 1.9f`, `VIB_CRITICAL_OFF_MMS = 4.2f`.
- Escalation persistence: `VIB_ALARM_PERSIST_CAPTURES = 2u` (requires 2 distinct FIFO captures agreeing before raising an alarm level).
- `readVelocityForAlarm()` extended with an additive `outCaptureId` parameter to support persistence counting by genuine measurement, not by 250ms poll tick.
- Boot-banner text rewritten to print the new thresholds explicitly (cosmetic only).
- This restores vibration alarm evaluation that was live on the device before an earlier isolated flash in this session accidentally reverted it to `THRESHOLDS_UNSET`.

No other changes are present in this build. Verified via exact hunk-count/line arithmetic (49+159=208, 19+18=37) and zero cross-references between the two change sets.

## 4. Post-flash live validation results

**MQTT** (real broker capture, `factory/plant01/machine/pump01/{sensor,vibration}`, mTLS 8883):
- Connected and publishing on the normal 30s (NORMAL) adaptive cadence.
- 5 consecutive publish pairs observed 16:22-16:24Z, all `current_valid: true`, `current_age_s` ranging 0.1-2.1s (well inside the 5s window), `current_a` smooth and non-zero (0.66-0.73A), `vibration_status: "OK"`, `alarm_level: "NORMAL"`.
- `/sensor` and `/vibration` payloads confirmed byte-identical on all current-freshness fields at every timestamp.

**API** (`https://dash.promlogix.com/api/machine/plant01/pump01`), re-checked at checkpoint time:
```
status.online: true
vibration: velocity_rms_overall_mms=0.285, vibration_status="OK", alarm_level="NORMAL", alarm_level_live=true
operating_condition: rpm=1485.3, temp_c=51.1, operating_hours_total=90.3891, current_a=0.73, current_valid=true
```

**Boot log:** clean startup, no boot loop, `Sensor Reads` counter incrementing with 0 errors, modem/GPRS/NTP/MQTT sequence completed normally.

## 5. Summary

| Check | Result |
|---|---|
| MQTT connected | ✅ |
| `current_a` valid, never fabricated | ✅ |
| `current_valid` stable while fresh | ✅ |
| `current_age_s` present | ✅ |
| `vibration_status` = OK / evaluated | ✅ |
| `alarm_level` functioning (live, not placeholder) | ✅ |

**This is the verified production baseline as of 2026-08-25 16:27:29Z.**
