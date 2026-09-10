# P1-FIFO-1 — RELEASE RECORD

**Change ID:** P1-FIFO-1
**Date:** 2026-09-10
**Status:** VERIFIED ON HARDWARE — awaiting commit
**Scope:** Phase-1 productization. Single isolated change.

---

## 1. Purpose

Cloud connectivity loss must never stop local vibration measurement.

Before this change, the FIFO admission gate treated MQTT uplink health as a
precondition for **local** acquisition. When MQTT disconnected, every scheduled
capture was rejected and the entire local measurement chain went blind:

```
MQTT down -> mqttReconnecting = true
          -> FifoDriver_Request() returns ERR_NOT_PERMITTED
          -> no FIFO capture is ever started
          -> g_velCarrier never updated
          -> age exceeds VIB_VELOCITY_MAX_AGE_MS_TBD (10 s)
          -> OLED Vrms shows "--", local alarm evaluation suppressed,
             buffered telemetry contains no velocity
```

Measured impact of the defect (Test 2, 2026-09-10, 2,415 s natural outage):
**1,207 rejections, 0 FIFO results, 53/53 status reports showing
`Velocity:-- (unavailable)`**, while Modbus polling remained healthy
(+9,425 reads, 0 errors) — proving the sensor chain was fine and only the
admission gate was at fault.

The behaviour was **pre-existing**, introduced in commit `0bfad96`
(2026-07-29), not by any recent candidate work.

## 2. Exact source change

**File:** `claude/WTVB02_ESP32S3_V16_CM_fault_latch_v3_hardened_patched_v16_5/fifo_driver.cpp`

**Executable change — 3 lines removed, 0 added:**

```c
-    if (req->admissionContext.mqttReconnecting) {  // inverted, per SS19.2
-      return FifoError::ERR_NOT_PERMITTED;
-    }
```

Replaced by a `[P1-FIFO-1]` rationale comment at the gate site, plus two
factual corrections to the SS19.2 comment block, which otherwise still
claimed *"three application-state gates"* and *"(motor/sensor/network)"*.

**Explicitly retained (verified in situ and on hardware):**
- `motorStable` gate
- `sensorHealthy` gate
- the `requirePermissive` wrapper
- all four driver-internal structural gates
  (circuit breaker, driver idle, cooldown, result-not-released)

**Explicitly NOT changed:** FIFO scheduler, FIFO cadence
(`FIFO_PERIODIC_INTERVAL_MS = 2000UL`, 0.5 Hz), OLED code, alarm code,
`g_velCarrier`, TelemBuf, MQTT reconnect logic, modem recovery logic,
GPIO9/GPIO4, Node-RED, InfluxDB, API, dashboard.

**Spec note:** this amends the frozen **SS19.2** admission table by removing
its third application-state (network) gate. The comment block records the
removal at both the summary and the gate site.

## 3. Artifact identity

| Artifact | SHA256 |
|---|---|
| Production `.ino` (unchanged) | `0926281D2A568C1C35129BD94AA06805764EEB7767C8A77EF5586086C9793D92` |
| `fifo_driver.cpp` (changed) | `AD8819B6C5CBF316063D48958E6C35F6FB067C4BDC1B4A5406FFBC8AC7574B84` |
| Released application binary | `6B54B0840204DD39F90FB19B5457DEA7A8A2327C4342687C40CD2BAC803258D0` |
| Test A serial evidence | `5E44F3A1F3CE28F05EE7EE7BB555A42F3AB837D06D3AC5C0A770454470A80E34` |

Binary size: **676,832 bytes** on disk · flash 676,676 B (21%) · RAM 92,752 B (28%)
Delta vs pre-change baseline: **−8 bytes flash, 0 bytes RAM**.
Compiler warnings: **111 — byte-identical set to the pre-change baseline, zero new.**

## 4. Build

**FQBN (CLAUDE.md validated production string, every option explicit):**

```
esp32:esp32:esp32s3:CDCOnBoot=cdc,CPUFreq=240,FlashMode=qio,FlashSize=16M,PSRAM=opi,PartitionScheme=app3M_fat9M_16MB,USBMode=hwcdc,UploadMode=default,UploadSpeed=921600
```

Clean build (`--clean`), build path outside the repository.
Build ID observed at boot: `16.5-1c67257-dirty-20260910-2255`.

**Provenance (re-verified at release time):**
- `P1-FIFO-1` present in the compiled source copy — **2 occurrences**
- `if (req->admissionContext.mqttReconnecting)` in compiled source — **0 occurrences**
- `candidate_net_recovery` references in the build — **0**
- Source SHAs identical before and after the build

## 5. Flash provenance

| Item | Value |
|---|---|
| Device | ESP32-S3 (QFN56) rev v0.2, **MAC `3c:84:27:e9:98:7c`** — matches documented unit |
| Port | COM5 (native USB-Serial/JTAG) |
| Method | **App-partition only at `0x10000`**; erase confined to `0x00010000–0x000b5fff` |
| Bootloader / partition table | **untouched** |
| Verification | `Hash of data verified.` · esptool exit 0 |
| Boot | single clean boot, no panic / WDT / brownout / backtrace |

## 6. Test A — procedure

Controlled MQTT outage. The MQTT broker was the **only** fault injected; no
cellular, modem, GPIO or ESP32 manipulation.

1. Flash the released binary; capture serial continuously.
2. Establish a healthy baseline (GPRS + MQTT connected, numeric velocity,
   FIFO producing results, 0 errors).
3. `docker stop iot-stack-mosquitto-1` on the production VPS — **T0 =
   2026-09-10 23:06:06 +07:00 (16:06:05Z)**.
4. Observe ≥ 60 s (actual: ~160 s).
5. `docker start iot-stack-mosquitto-1` — 23:08:44.
6. Verify MQTT reconnect, TelemBuf replay, continued numeric velocity.

Recorder: `capture_net_recovery_test2.ps1` (receive-only, raw bytes,
auto-reattach). Session: 1,200 s, `opens=1 portLost=0`, single boot banner.

## 7. Test A — result: **PASS (11/11)**

Device detected the disconnect ~15 s after T0.

| Phase | verdict=0 | verdict=9 (BUSY) | verdict=10 | FIFO results |
|---|---|---|---|---|
| During outage | **67** | 36 | **1** | **67, all `error=NONE`** |
| Whole session | 116 | 65 | 1 | 382, all `error=NONE`, **0 errors** |

**The decisive observation — impossible under the previous firmware:**

```
[FIFO-BROKER] FifoDriver_Request() ... verdict=0 handle=57
              motorStable=1 sensorHealthy=1 mqttReconnecting=1
```

The single `verdict=10` carried **`motorStable=0`** with `sensorHealthy=1` —
a legitimate physical gate firing while the motor was momentarily not RUNNING.
This positively confirms the retained gates still enforce.

| # | Acceptance criterion | Result |
|---|---|---|
| 1 | `mqttReconnecting` becomes true | PASS |
| 2 | FIFO not rejected due to `mqttReconnecting` | PASS — 0 network-caused rejections |
| 3 | Capture continues at ~2 s cadence | PASS |
| 4 | verdict=0 successful | PASS — 67 admitted, 67 results, 0 errors |
| 5 | `g_velCarrier` receives valid data | PASS |
| 6 | OLED Vrms stays numeric | PASS — 0.31/0.33/0.32/0.33/0.32/0.36 mm/s, never `--` |
| 7 | `/vibration` numeric | PASS |
| 8 | Local alarm not suppressed | PASS — no `[LATCH] Suppress ON … vibOk=0` |
| 9 | TelemBuf normal | PASS — 0→7 slots, overflow 0, **7/7 replayed** on reconnect |
| 10 | No WDT / panic / brownout / reboot | PASS — 1 boot banner, 0 `###PORT_LOST`, 0 faults |
| 11 | Modbus/RS485 healthy | PASS — 15,549 reads, 0 errors |

Buffered slots now carry **real velocity data**; under the previous firmware
they would have been buffered empty.

## 8. Known limitations

1. **Modem/cellular recovery remains UNRESOLVED and is explicitly NOT part of
   this release.** `modem.restart()` failing at ~10.199 s, the `testAT()`
   hypothesis, GPIO9/PCIE_RST semantics and the AX7670 V1.1 carrier mapping
   are all still open. See
   `docs/engineering/net_recovery_decision_record_20260910.md`.
2. **v16.7a network-recovery remains a SEPARATE, uncommitted candidate**
   (`claude/candidate_net_recovery_v16_7a/`, source SHA
   `BAFA02FC9017FA99472522DD8612437430437E05E479C30B58EDF9897845B86C`).
   It is not part of this release and its tree still contains the pre-change
   `fifo_driver.cpp`.
3. **Change 2 (controlled ESP32 restart fallback) was NOT implemented.**
4. Restored 0.5 Hz capture during long outages returns normal FIFO duty cycle
   and RS485 bus contention — i.e. normal operation restored, not new load.
5. Test A validated a **broker-level** outage. A full cellular outage exercises
   the same `mqttClient.connected()` condition, but was not separately re-tested
   after this change.
6. `build_info.h` carries a generated `GIT_COMMIT_HASH` refresh
   (`f51fea5-dirty` → `1c67257-dirty`) produced by the Arduino IDE toolchain,
   not by hand.

## 9. Rollback

| Artifact | Value |
|---|---|
| Pre-change production app image | `build_replay_timestamp_fix_verify_20260907/WTVB02_…_v16_5.ino.bin` |
| SHA256 | `53BAEDD677A88EAA3B610AE8EFE7C88DC3AB8F61DE949E5031AB6F8D339BB37E` |
| Size | 676,848 bytes |
| Method | App-only reflash at `0x10000` — bootloader and partition images are byte-identical, so neither needs rewriting |

Source rollback: revert the single `fifo_driver.cpp` change; the production
`.ino` was never modified.

## 10. Evidence

| Item | Path |
|---|---|
| Test A serial capture | `claude/candidate_net_recovery_v16_7a/serial_testA_p1fifo1.log` (290,344 B) |
| Prior defect evidence | `claude/candidate_net_recovery_v16_7a/serial_test2_natural_loss.log` |
| Investigation record | `claude/docs/engineering/net_recovery_decision_record_20260910.md` |

## 11. Release status

**VERIFIED ON HARDWARE — READY FOR COMMIT.**

Source SHAs match, binary provenance re-verified, Test A PASS on all 11
criteria, no unintended firmware changes, staging area empty.
Not yet committed at time of writing.
