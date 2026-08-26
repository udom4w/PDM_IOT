# NTP Retry Scheduler Fix (v16.5j) — Release Verification Report

**Branch:** `feature/rs485-115200-validation`
**Baseline commit:** `b603c2f` — `fix(firmware): preserve SR provenance across failed quick-reconfig`
**Status:** Verified for current baseline (with one documented limitation — see §11).

---

## 1. Patch title

NTP retry scheduling gate fix — `checkAndSyncTime()` (`[v16.5j]`).

---

## 2. Root cause

`checkAndSyncTime()` scheduled its next NTP attempt using `g_timeSync.lastSyncMillis`, a **success-only** timestamp. While the modem kept returning invalid/no time, `lastSyncMillis` never advanced, so the "has enough time passed" gate stayed permanently true. `checkAndSyncTime()` is invoked from `taskNetwork()`'s ~100ms tick, so every unsynced tick re-attempted the sync — a ~100ms busy-loop instead of the intended 30-second backoff.

---

## 3. Fix description

- Scheduling gate changed from `lastSyncMillis` to `lastCheckMillis`.
- `lastCheckMillis` now advances on **every** NTP attempt, success or failure (`WTVB02_ESP32S3_V16_CM_fault_latch_v3_hardened_patched_v16_5.ino:5558`).
- `lastSyncMillis` remains success-only (unchanged semantics, still used elsewhere for "time since last successful sync").
- Interval selection: `g_timeSync.synced ? NTP_CHECK_INTERVAL_MS : NTP_RETRY_INTERVAL_MS` (line 5547).
- `NTP_RETRY_INTERVAL_MS = 30000UL` — unsynced retry interval, 30s (line 840).
- `NTP_CHECK_INTERVAL_MS = 1800000UL` — synced re-check interval, 30 min (line 839).

No other logic changed. FIFO, MQTT, and Trend subsystems were not touched by this patch and were not modified during this verification session.

---

## 4. Files / source involved

`claude/WTVB02_ESP32S3_V16_CM_fault_latch_v3_hardened_patched_v16_5/WTVB02_ESP32S3_V16_CM_fault_latch_v3_hardened_patched_v16_5.ino`
— `checkAndSyncTime()` (~line 5544), `syncRTCFromModem()` (~line 5428), constants at lines 839–840.

Built and tested from the linked worktree `PDM_IOT_worktree_patch1_isolated` (detached HEAD at `b603c2f`, identical content to this branch's HEAD).

---

## 5. Source SHA256

```
cff2b61c7989aee675936f8d1294ce74a975bb8f37dbc667c1ff1ad5e3a17658
```

---

## 6. Binary SHA256

```
aff1da5b62fc5121b0eb9d0b3503f54035ce0148b9a3e51d84126c5b2b7e3d0a
```

---

## 7. Binary size

681,408 bytes

---

## 8. Hardware / COM port

- Device: ESP32-S3, LilyGO T-Vending S3, embedded PSRAM 8MB, MAC `3c:84:27:e9:98:7c`
- Port: `COM5` (USB VID_303A / PID_1001 — ESP32-S3 native USB-Serial/JTAG)
- FQBN: `esp32:esp32:esp32s3:CDCOnBoot=cdc,CPUFreq=240,FlashMode=qio,FlashSize=16M,PSRAM=opi,PartitionScheme=app3M_fat9M_16MB,USBMode=hwcdc,UploadMode=default,UploadSpeed=921600` (matches CLAUDE.md production spec)

---

## 9. Flash verification

- Pre-flash integrity check: source SHA256 and binary SHA256 both matched expected values exactly; binary size matched (681,408 B). **PASS**
- App-partition-only flash via `esptool.exe` at offset `0x10000` (bootloader/partition table untouched, per test scope).
- Post-write hash verification by `esptool`: **PASS** ("Hash of data verified").

---

## 10. Runtime verification results

Two live serial captures on COM5 @ 115200 baud, ≥90s each, from a fresh reset.

| Check | Result |
|---|---|
| Valid GSM time synchronization | **VERIFIED** on hardware (`[NTP] GSM time valid` → `Time synchronized`, +35.079s from boot) |
| Post-sync interval switches to 1800s | **VERIFIED** on hardware (`[NTP] Time sync check (interval=1800s, synced=yes)`, +38.528s) |
| ~100ms NTP busy-loop | **NOT OBSERVED** in either runtime capture |
| FIFO | 19 `[FIFO-RESULT]` captures, 0 errors, `error=NONE` throughout, no stall across the NTP sync event |
| MQTT | 37 outbound publishes by end of capture, 0 failures, `mqttReconnecting` correctly flips 1→0 at sync |
| Trend / TREND-P2 | Continuous, normal values (slope/drift/EMA), one `/trend` publish observed |
| Panic / watchdog / reboot | None observed during either capture |

---

## 11. Known verification limitation

> Unsynced 3-attempt ~30s cadence was not empirically exercised because the modem acquired valid network time before a second `synced=no` retry could occur. The 30s retry behavior is verified by source-level scheduler inspection but not by multi-attempt hardware timing measurement.

This is **not** classified as a failure. Investigated and ruled out as infeasible to force in this session without either a physical action outside this session's capability (SIM pull / antenna shielding — not attempted, no separate AT-command channel exists on this hardware, no passthrough command exists in the current firmware, and no source change was authorized to add one).

Code-level evidence supporting the 30s behavior (read-only inspection of the already-flashed source, no edits made):

```
Line 839: #define NTP_CHECK_INTERVAL_MS 1800000UL   // 30 minutes between re-checks once synced
Line 840: #define NTP_RETRY_INTERVAL_MS 30000UL     // 30 seconds retry while not yet synced
Line 5547: uint32_t interval = g_timeSync.synced ? NTP_CHECK_INTERVAL_MS : NTP_RETRY_INTERVAL_MS;
Line 5557: if (g_timeSync.lastCheckMillis == 0 || (now - g_timeSync.lastCheckMillis >= interval)) {
Line 5558:   g_timeSync.lastCheckMillis = now;   // advances on every attempt, success or failure
```

---

## 12. Regression checks

- FIFO capture pipeline: no regression (§10).
- MQTT outbound queue / publish path: no regression (§10).
- Trend / TREND-P2 analytics: no regression (§10), trend logic not modified.
- No source changes, rebuild, or reflash occurred beyond the single verified binary flashed for this test.

---

## 13. Final disposition

**NTP PATCH VERIFIED FOR CURRENT BASELINE**

Verified: retry scheduling gate (source-level), 30000ms unsynced retry constant, 1800000ms synced re-check constant, `lastCheckMillis` advancing on every attempt, valid GSM time sync on hardware, post-sync interval switch on hardware, absence of ~100ms busy-loop, FIFO/MQTT/Trend non-regression.

Explicit limitation carried forward: unsynced 3-attempt ~30s cadence not empirically measured on hardware in this session (see §11) — verified by source-level scheduler inspection only.
