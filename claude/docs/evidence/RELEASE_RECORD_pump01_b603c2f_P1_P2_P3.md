# Production Baseline Release Record — pump01

**Status: PRODUCTION BASELINE = FROZEN**
Record generated: 2026-08-26 (read-only release control, no source modification)

---

## 1. Source identity

```
Isolated source tree: C:\Users\HP\Downloads\PDM_IOT_worktree_patch1_isolated
Sketch path:          claude\WTVB02_ESP32S3_V16_CM_fault_latch_v3_hardened_patched_v16_5\
                       WTVB02_ESP32S3_V16_CM_fault_latch_v3_hardened_patched_v16_5.ino
Baseline commit:       b603c2fbdf100901833af18ce1d106537aa01347
Baseline commit date:  2026-08-24 23:26:29 +0700
Baseline commit msg:   fix(firmware): preserve SR provenance across failed quick-reconfig
Composition:           b603c2f + Patch 1 + Patch 2 + Patch 3 (uncommitted in this worktree,
                        hash-verified below; main dirty working tree with unrelated WIP
                        was never touched and remains separate)
```

## 2. Source SHA256 (re-verified from disk at record time)
```
e2a9e4350b96d2ed98686b57ceb45a0a50d05b1522b24fe51a3a5cc35de700e1
```

## 3. Binary SHA256 currently flashed on pump01 (re-verified from disk at record time)
```
c81f0e4b5341d64b686d815ee7227add2d9b3dfa445b0749a43c7885a2b274fd
```

## 4. Build/deployment identity

| Field | Value |
|---|---|
| Binary size | 681,248 bytes |
| FQBN | `esp32:esp32:esp32s3:CDCOnBoot=cdc,CPUFreq=240,FlashMode=qio,FlashSize=16M,PSRAM=opi,PartitionScheme=app3M_fat9M_16MB,USBMode=hwcdc,UploadMode=default,UploadSpeed=921600` |
| Build fingerprint (`GIT_COMMIT_HASH`) | `b603c2f-dirty` (accurate — Patch 1/2/3 are uncommitted in this worktree relative to the b603c2f baseline) |
| Build performed (UTC, this session) | 2026-08-26T08:47:05Z – 08:52:35Z |
| Flashed (UTC) | 2026-08-26T08:56:22Z – 08:56:34Z |
| Target COM port | COM5 |
| Device MAC | 3c:84:27:e9:98:7c (pump01, confirmed identical across every flash this session) |

## 5–6. Composition and WIP-contamination verification

```
git diff b603c2f -- <sketch.ino>: 10 hunks, 124 insertions(+), 20 deletions(-), 1 file
  2 hunks -- Patch 1 (SR re-verification)
  1 hunk  -- Patch 2 (QUICK-RECONFIG EN-pin bracket)
  7 hunks -- Patch 3 (Current telemetry held-evidence fix)
```
Patch 1 and Patch 2 integrity proven cryptographically (not just by inspection): mechanically
removing each later patch's exact insertion from the current source reproduces the prior
recorded SHA256 byte-for-byte at every step (b603c2f+P1 → +P2 → +P3 chain, each verified).

**WIP contamination check** — occurrence counts of `v16.6i` / `v16.6j` / `S21` / `S21b` in the
final source: **2 / 1 / 0 / 0 — identical to the pure `b603c2f` baseline count**, confirmed
unchanged by any of the three patches. These 3 occurrences are pre-existing comment
cross-references already committed in `b603c2f` itself (lines referencing historical/future
work by tag name), not the separate uncommitted WIP diff hunks (`VIB_WARNING_MMS` threshold
block, `CURRENT_EVIDENCE_MAX_AGE_MS` hoist, `readVelocityForAlarm()` signature change,
`taskStateMachine` escalation-persistence logic, `publishTelemetry()` changes) identified in
the main dirty working tree, which were never merged into this isolated baseline.

**Confirmed: unrelated WIP has NOT entered the production baseline.**

---

## 7. Validation evidence summary

### Patch 1 — READ-ONLY SR re-verification
| Check | Result |
|---|---|
| Build/source integrity | VALIDATED — isolated, hash-proven, 2 hunks / 70 lines |
| Normal-operation stability | VALIDATED — 335+ consecutive error-free FIFO captures across 3 independent post-flash windows (Patch 1, Patch 2, Patch 3 flashes) |
| SR-VERIFY dormancy | VALIDATED — 0 misfires in every observation window (correct: SR provenance never lost during any observation) |
| Actual SR-loss recovery | **NOT TESTED** — no fault was intentionally induced |

### Patch 2 — QUICK-RECONFIG RS485 EN-pin fix
| Check | Result |
|---|---|
| Build/source integrity | VALIDATED — isolated, hash-proven, 1 hunk / 11 lines |
| Normal-operation stability | VALIDATED — 0 interference with normal polling across all windows |
| Quick-reconfig recovery | **NOT TESTED** — no Modbus error burst was intentionally induced |

### Patch 3 — Current telemetry held-evidence fix
| Check | Result |
|---|---|
| Build/source integrity | VALIDATED — isolated, hash-proven, 7 hunks / 124 lines |
| Production validation window | 10.5 minutes (2026-08-26T08:56:34Z – 09:07:36Z) |
| API samples collected | 40 (exceeds the 20 required) |
| `current_valid=false` occurrences | **0** |
| Numeric → "—" flickers | **0** |
| Current value range observed | 0.74 – 0.76 A (matches/consistent with known-good 0.73–0.75 A range) |
| FIFO capture health during window | 200/200 `error=NONE` (captureId 1→201) |
| Crash/watchdog/brownout signatures | 0 (across 9,351 Serial lines) |

---

## 8. Release status

```
PRODUCTION BASELINE = FROZEN
```

## 9. Statement

**No further firmware changes required at this time.**

---

*This record reflects the state as of the last flash performed in this session
(2026-08-26T08:56:34Z). The main dirty working tree at
`C:\Users\HP\Downloads\PDM_IOT\claude\...` still contains separate, unrelated,
uncommitted WIP ([v16.6i]/[v16.6j]/[S21]/[S21b]) — untouched, unmerged, and
outside the scope of this baseline.*
