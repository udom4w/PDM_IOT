# Phase 1 — Mechanism-B Mitigation — Validation Summary

**Status: validated on real hardware. Not yet committed as production baseline — see §10.**

---

## 1. Purpose

Mechanism B is the false-transition failure mode identified and proven during RCA:

```
WTVB02 (WTVB05) RS485 Modbus timeout
    -> shared RS485 bus occupied for ~2000 ms
    -> CT/T7 current polling delayed
    -> current evidence age exceeds NO_CURRENT_STOPPING_MS (1500 ms)
    -> motor_state FSM misreads the delay as current absence
    -> false RUNNING -> STARTING -> STOPPING (-> STOPPED) transition
    -> FIFO admission and downstream vibration gating unnecessarily affected
```

Phase 1's purpose is to prevent this specific false transition — **communication failure of a
vibration sensor (WTVB02) must never be interpreted as physical motor-current absence** — without
changing any existing FSM timing constant, current threshold, Modbus protocol behavior, transaction
ordering, or FIFO admission logic, and without weakening genuine current-driven (Mechanism A) or
genuine CT-failure (Case 2) detection.

---

## 2. Source Identity

| | |
|---|---|
| Production source | `claude/WTVB02_ESP32S3_V16_CM_fault_latch_v3_hardened_patched_v16_5/WTVB02_ESP32S3_V16_CM_fault_latch_v3_hardened_patched_v16_5.ino` |
| Source SHA256 (post-patch) | `188e193effba872efd27de450f485a741d3ff2eb12e77b37ed6eb536a4ba826d` |

---

## 3. Binary Identity

| | |
|---|---|
| Binary SHA256 | `5c5f5e15a8d51f6e99976ab1365408bcdc631320444768ab8a0235cbdd2e5dac` |

This is the **flashed and verified Phase 1 binary currently running on Pump01**. Compiled with the
exact production FQBN
(`esp32:esp32:esp32s3:CDCOnBoot=cdc,CPUFreq=240,FlashMode=qio,FlashSize=16M,PSRAM=opi,PartitionScheme=app3M_fat9M_16MB,USBMode=hwcdc,UploadMode=default,UploadSpeed=921600`)
and ESP32 core 3.3.11 (matching the original production baseline core version). All four flash
segments (bootloader, partitions, boot_app0, application) were esptool hash-verified at flash time.

---

## 4. Phase 1 Patch Summary

Six additive locations in the single modified `.ino`, all scoped to the `MOTOR_SRC_CURRENT` evidence
path:

- **`CT_GRACE_WINDOW_MS = 3000`** — empirical containment window, sized against measured Mechanism-B
  evidence (22 pre-patch episodes, 1994–2011 ms observed range), not a formal scheduler worst-case
  guarantee.
- **`g_ctWtvbGraceUntilMs`** — file-scope grace-deadline timestamp, written **only** from WTVB02-side
  code paths.
- **`g_ctLastFailedAttemptMs`** — file-scope timestamp of the most recent genuine T7/CTR4A01
  acquisition failure, written **only** from the CT-side code path. Never touched by WTVB02 code.
- **WTVB02 failure grace** — extends `g_ctWtvbGraceUntilMs` at the point a WTVB02 (T1a/T1b/T1c/T2a)
  transaction failure is detected, immediately before the CT/T7 poll gate.
- **WTVB02 recovery grace** — re-extends `g_ctWtvbGraceUntilMs` at the entry of the quick-reconfig
  sequence, covering the additional ~1962–1976 ms of bus time that sequence consumes on the
  following (recovery) iteration.
- **CT failed-attempt timestamp** — records `g_ctLastFailedAttemptMs` whenever T7 is genuinely
  attempted and fails, independent of any WTVB02 activity.
- **Widened `evidenceFrozen` veto logic** — in `buildMotorStateEvidence()`, the staleness-trip freeze
  now activates when a WTVB02 grace window is active **and** no more-recent genuine CT failure has
  been recorded **and** the existing `CURRENT_EVIDENCE_MAX_AGE_MS` (5000 ms) outer bound has not been
  exceeded. This last condition is the mechanism that correctly distinguishes "CT itself failed" from
  "WTVB02 blocked CT" — a genuine T7 failure can never be masked by WTVB02-only grace.

**Unchanged:** `CURRENT_EVIDENCE_MAX_AGE_MS`, `NO_CURRENT_STOPPING_MS`, `FORCE_CURRENT_STOPPED_MS`,
`RUNNING_WARMUP_MS`, `CURRENT_ON_THRESHOLD_A`/`CURRENT_OFF_THRESHOLD_A`, `MODBUS_POLL_PERIOD_MS`, the
Modbus response timeout, T1a/T1b/T1c/T2a/T7 transaction ordering, FIFO driver/admission logic, and the
`MOTOR_SRC_RPM`/`MOTOR_SRC_PROXIMITY` branches.

---

## 5. Runtime Validation — Mechanism B

Post-flash passive serial capture, 11 minutes, real hardware (Pump01), Phase 1 binary running:

- **9 genuine WTVB02-caused timeout episodes** observed (T1a/T1b/T1c/T2a Modbus timeouts,
  `durationMs≈2002ms` each, matching the pre-patch failure signature exactly, including one
  compound double-failure + full quick-reconfig sequence).
- **9/9 contained — 0 false Motor-State transitions.**
- **1 genuine T7/CTR4A01 (CT-itself) failure occurred and was correctly NOT suppressed** — it produced
  the normal, expected `signal-absent` staleness trip and self-recovering warm-up cycle, direct
  confirmation that the veto logic discriminates the two failure causes correctly rather than
  masking every staleness event indiscriminately.
- **0 reboot / panic / watchdog indications** across the full capture.

---

## 6. Criterion 5 — Genuine Current-Driven Transition

Controlled physical current-reduction test performed on the flashed Phase 1 binary, on real hardware,
with a clean contamination check (no WTVB02/T7 activity within the active transition window):

- Genuine physical current reduction (~0.69 A -> 0.14 A), gradual manual decline.
- `signalPresent`: **1 -> 0**, with fresh evidence (`ageMs=0`) at every step.
- **RUNNING -> STARTING -> STOPPING -> STOPPED**, via the unmodified `absentMs`-based FSM path
  (firmware's own log confirmed a genuine 60-second stop).
- Current recovery above the existing ON threshold.
- `signalPresent`: **0 -> 1**.
- **STARTING -> RUNNING** after the existing, unmodified 2.5 s warm-up.
- Contamination check: clean — nearest WTVB02 event was 28.8 s before onset; no WTVB02/T7 activity
  during the active decline/recovery transition.
- **0 reboot / panic / watchdog indications.**

This closes Criterion 5: a genuine current-driven transition still functions correctly, end-to-end,
on the patched firmware.

---

## 7. Evidence References

Raw serial captures (local-only — see §9):

| File | SHA256 | Lines |
|---|---|---|
| `validation/evidence/mechanismB_phase1_postflash_capture.log` | `de75317d3511412908c450233cb94805c8a1640c218bf8864ee7e1bcab000d7a` | 2839 |
| `validation/evidence/mechanismB_phase1_criterion5_capture.log` | `a6ca4ac0e7071abe2fd8b1f5eedf73be707e345eb5785cdb03d9850fd79fbe8c` | 3150 |

---

## 8. Git Scope

Phase 1 source diff, verified:

```
1 file changed, 84 insertions(+), 1 deletion(-)
```

Single file: `WTVB02_ESP32S3_V16_CM_fault_latch_v3_hardened_patched_v16_5.ino`. No other tracked
production file modified.

---

## 9. Out of Scope

The two raw `*.log` captures referenced in §7 remain **local-only** and are not part of this
checkpoint. `*.log` is already covered by the repository's existing `.gitignore` policy
(`.gitignore:29`), consistent with every prior raw serial capture in this repository's history (the
earlier VRMS validation checkpoint likewise tracked only derived `.md`/`.csv`/`.json`/`.py` evidence,
never raw `.log` captures). This document, together with its recorded SHA256/line-count references,
is the tracked, durable record of what those captures contain.

---

## 10. Verdict

**PHASE 1 — VALIDATED / READY FOR CHECKPOINT COMMIT**

This document records validation results only. **No commit has been made as of this writing.**
