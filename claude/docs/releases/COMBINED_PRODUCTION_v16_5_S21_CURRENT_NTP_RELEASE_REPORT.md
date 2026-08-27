# Combined Production Candidate — S21 + Current-Freshness + NTP Retry Fix

**Branch:** `feature/rs485-115200-validation`
**Baseline:** Aug-25 verified production checkpoint (`claude/backups/WTVB02_v16_5.ino_PRODUCTION_CHECKPOINT_20260825_162729Z`), plus the NTP `[v16.5j]` retry-scheduler fix verified 2026-08-26.
**Status:** Combined Production Candidate — Verified For Current Baseline (two explicit limitations below).

---

## 1. Production Candidate

| Field | Value |
|---|---|
| Combined source | `WTVB02_v16_5_COMBINED_S21_CF_NTP_20260826.ino` |
| Source SHA256 | `827137c2bac8d1455e698f20f3c3f6e9c9684772342b82061d9096ade8e4f338` |
| Binary SHA256 | `62c812d948fc18f217dd2d97a13a452963b4a50cb7846d4800a5aec4cf05dfd0` |
| Binary size | 683,984 bytes |

**Merge composition** (reconciled via 3-way `git merge-file`, base=`b603c2f`, ours=Aug-25 checkpoint, theirs=today's NTP source — see reconciliation record in this session):
- S21 vibration-alarm thresholds and current-telemetry-freshness fix, unchanged from the Aug-25 checkpoint.
- NTP `[v16.5j]` retry-scheduler fix, transplanted from today's verified NTP source.
- CTR4A01 5ms turnaround fix, unchanged (common to both parent lines and to committed git HEAD).
- `[v16.5g]` SR-provenance re-verification block: **deliberately excluded** — present in today's NTP source but not part of either requested fix; two merge conflicts at the `current_a`/`current_valid` publish sites (where the NTP source carried an unrelated, undisclosed `[v16.5i]` variant) were resolved in favor of the Aug-25 checkpoint's version, per explicit instruction to include no other feature discovered during reconciliation.

---

## 2. Build

**Production FQBN** (exactly as defined in CLAUDE.md):
```
esp32:esp32:esp32s3:CDCOnBoot=cdc,CPUFreq=240,FlashMode=qio,FlashSize=16M,PSRAM=opi,PartitionScheme=app3M_fat9M_16MB,USBMode=hwcdc,UploadMode=default,UploadSpeed=921600
```

**Result:** exit 0, 0 warnings, 0 errors.
```
Sketch uses 683828 bytes (21%) of program storage space. Maximum is 3145728 bytes.
Global variables use 110040 bytes (33%) of dynamic memory, leaving 217640 bytes for local variables.
```

Built in an isolated sketch/output directory from the combined source plus unmodified companion files (`fifo_*`, `vib_*`, `log.*`, `build_info.h`) copied verbatim from the current production sketch.

---

## 3. Flash Evidence

| Field | Value |
|---|---|
| Hardware | ESP32-S3 (QFN56, rev v0.2), PSRAM 8MB |
| Port | COM5 |
| MAC | `3c:84:27:e9:98:7c` |
| Method | APP-PARTITION-ONLY, offset `0x10000` — bootloader and partition table untouched, no chip erase |
| Result | **PASS** |
| esptool verification | `Hash of data verified.` |

---

## 4. Runtime Evidence

- Capture: 130.1 seconds, fresh reset, 115200 baud, single continuous boot (1 ROM banner, no re-entry).
- **Build ID:** `16.5-b603c2f-dirty-20260826-2352`

### S21 — PASS
Boot banner:
```
WARNING        : 2.1 mm/s RMS  (clears below 1.9)
CRITICAL       : 4.5 mm/s RMS  (clears below 4.2)
Escalation     : 2 consecutive FIFO captures
```
Confirms `VIB_WARNING_MMS=2.1`, `VIB_CRITICAL_MMS=4.5`, `VIB_WARNING_OFF_MMS=1.9`, `VIB_CRITICAL_OFF_MMS=4.2`, `VIB_ALARM_PERSIST_CAPTURES=2`. Runtime: 31 FIFO captures, all `error=NONE`; `RMS: 0.19 mm/s` at +122s (well under WARNING); zero `THRESHOLDS_UNSET`/disabled-alarm warnings across the capture.

### Current Freshness — PASS (mechanism-level only)
11 `[CURRENT_DIAG]` samples over 130s, continuously varying, non-zero, non-frozen (0.770A → 0.764 → 0.765 → 0.760 → ... → 0.396 → 0.570A).

**Limitation, recorded precisely:** the actual MQTT payload fields `current_age_s` and `current_valid` could not be directly inspected — no MQTT client tool was available in this environment. This is **mechanism-level verification only**, not direct wire-level/MQTT-payload confirmation.

### CTR4A01 — PASS
Same 11 successful current readings above; zero current-read timeout/failure messages anywhere in the capture. The 5ms turnaround behavior (pre-fix baseline: 66/66 failures under equivalent conditions, per the CTR4A01 investigation report) is preserved.

### NTP — PASS for sync/post-sync scheduling/no-busy-loop; cadence INCONCLUSIVE
```
+35.223s  [NTP] Time sync check (interval=30s, synced=no) → GSM time valid → Time synchronized
+39.156s  [NTP] Time sync check (interval=1800s, synced=yes) → re-synced, drift +0s
```
Only 2 NTP checks in 130s; no ~100ms retry storm.

**Limitation, recorded precisely:** *"Unsynced 3-attempt ~30s cadence was not empirically exercised because the modem acquired valid network time before a second synced=no retry could occur. The 30s retry behavior is verified by source-level scheduler inspection but not by multi-attempt hardware timing measurement."* Classified as: **INCONCLUSIVE / NOT EXERCISED.**

### FIFO — PASS
31 captures, all `error=NONE`, continuous through the NTP sync event and to the end of capture.

### MQTT — PASS
59 publishes, 0 failures, `MQTT: CONNECTED (mTLS)` throughout.

### Trend — PASS
9 `[TREND]`/`[TREND-P2]` events, sane values throughout. One drift alert (`alert=1`) at +125.9s — this is normal trend-analytics behavior (the engine correctly flagging a drift condition), **not an error**.

### Stability — PASS
130.1s runtime, exactly one intentional boot/reset, 0 panic, 0 Guru Meditation, 0 watchdog reset, 0 unexpected reboot.

---

## 5. Provenance Chain

```
Aug-25 verified production checkpoint
        ↓
combined source (3-way merge: checkpoint + NTP fix, SR-verify excluded)
        ↓
source SHA256   827137c2bac8d1455e698f20f3c3f6e9c9684772342b82061d9096ade8e4f338
        ↓
production FQBN/config (CLAUDE.md spec, project arduino-cli.yaml)
        ↓
build (exit 0, 0 warnings, 0 errors)
        ↓
binary SHA256   62c812d948fc18f217dd2d97a13a452963b4a50cb7846d4800a5aec4cf05dfd0  (683,984 B)
        ↓
COM5, ESP32-S3, MAC 3c:84:27:e9:98:7c
        ↓
APP-PARTITION-ONLY flash @ 0x10000 (bootloader/partition table untouched)
        ↓
esptool hash verification ("Hash of data verified.")
        ↓
130.1s runtime verification (this report)
```

**Artifact integrity — all prior artifacts remain unchanged:**
- Previous NTP-only binary SHA256 `aff1da5b62fc5121b0eb9d0b3503f54035ce0148b9a3e51d84126c5b2b7e3d0a` — unchanged.
- Aug-25 checkpoint binary SHA256 `74a2b97e3af5e7504462c3f77ab26b68a83b330b0c84297e7296e18c793606c3` — unchanged.
- Aug-25 checkpoint source SHA256 `224e159001a93c5e1c4cbc7322aab6e5635c807ed9763694080494c8bd5e73c8` — unchanged.
- Combined source SHA256 `827137c2bac8d1455e698f20f3c3f6e9c9684772342b82061d9096ade8e4f338` — unchanged since approval.
- No source modification occurred after the approved combined source was created.

---

## 6. Known Limitations (carried forward, not resolved by this release)

1. **NTP repeated-unsynced 30s cadence:** INCONCLUSIVE / NOT EXERCISED. Verified by source-level scheduler inspection (`lastCheckMillis` gate, `NTP_RETRY_INTERVAL_MS=30000UL`) only — not by multi-attempt hardware timing measurement, across three separate hardware runs to date.
2. **Current-freshness MQTT payload fields (`current_age_s`, `current_valid`):** not directly observed on the wire; mechanism-level verification PASS only, due to no MQTT client tool being available in this environment.

---

## 7. Final Disposition

**COMBINED PRODUCTION CANDIDATE — VERIFIED FOR CURRENT BASELINE**

This firmware is not claimed to be fully proven or 100% verified. It carries the two limitations above, both non-blocking but unresolved, in addition to whatever else remains open from prior release records (e.g., `ERR_RESULT_NOT_RELEASED` host-test coverage from Commit 7C).
