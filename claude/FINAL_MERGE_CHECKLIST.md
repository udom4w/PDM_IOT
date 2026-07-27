# FINAL_MERGE_CHECKLIST.md — v16.5.4 Architectural Hardening

Status: Verified, uncommitted. Read-only regression audit performed; no source
changes made during verification.

---

## Build Result

- Toolchain: `arduino-cli` (bundled with Arduino IDE), FQBN: see `CLAUDE.md` §
  "Production Build Configuration" (do NOT use the bare `esp32:esp32:esp32s3`
  FQBN)
- Clean build (`--clean`, empty `--build-path` cache, no reused objects): **PASS, exit code 0, zero warnings**

## Flash / RAM Usage

| Build | Flash | RAM |
|---|---|---|
| Pre-v16.5.4 baseline (untouched) | 630,272 B (48%) | 65,472 B (19%) |
| v16.5.4 final (this patch) | 630,436 B (48%) | 65,496 B (19%) |
| Delta | **+164 B** | **+24 B** |

Max program storage: 1,310,720 B. Max dynamic memory: 327,680 B (262,184 B free
for locals after globals).

## Files Modified

- `claude/WTVB02_ESP32S3_V16_CM_fault_latch_v3_hardened_patched_v16_5/WTVB02_ESP32S3_V16_CM_fault_latch_v3_hardened_patched_v16_5.ino`
  (only file touched; 28 diff hunks against the pre-v16.5.4 working-tree state)

No other source file was modified. This document is a new file, not a source change.

---

## Architecture Summary

Three surgical improvements, no new features, no FSM/MQTT/threshold changes:

1. **RPM EMA invalidation after long idle** — `MAX_EMA_INTERVAL_US` (independent
   2.0 s literal, deliberately not derived from `FORCE_STOP_TIMEOUT_MS`). A
   pulse gap or no-pulse idle longer than this invalidates `g_rpmFiltered`
   (reset to 0); the next valid pulse reseeds the EMA directly from raw RPM
   rather than blending with the discarded stale value.

2. **Atomic Telemetry Snapshot** — `TelemetrySnapshot` / `g_telemSnapshot`,
   captured exactly once per Core-0 state-machine cycle by
   `captureTelemetrySnapshot()`, after `updateMotorStateMachine()` and
   alarm/health evaluation are complete, guarded by the existing
   `mutexVibData`. All six consumers (MQTT publish, telemetry ring buffer,
   fault-latch replay, OLED, Analytics ×3, 30 s status log) now read this one
   struct instead of assembling their own view from two independently-locked
   globals. `g_vibData` was removed as dead weight once every consumer
   converted (verified via exhaustive search — zero remaining readers).
   `health_score` computation was relocated (not altered) into a small
   `computeHealthScore()` helper, called once at capture time, so the capture
   function stays pure data movement.

3. **RPM evidence freshness** — `RPMEvidence { rpm, valid, ageMs }`, refreshed
   every `processRPM()` cycle from `timeSincePulseMs` (the same clock the FSM
   itself uses, not a second independent one). `buildMotorStateEvidence()`
   now requires `g_rpmEvidence.valid` before reporting `signalPresent=true`,
   so a stale EMA can never do so. `evidence.ageMs` fed to the FSM is
   unchanged (`timeSincePulseMs` directly).

A self-review pass (post-implementation, pre-merge) found and corrected six
issues before finalizing: an unnecessary algebraic coupling between
`MAX_EMA_INTERVAL_US` and `FORCE_STOP_TIMEOUT_MS`; a redundant second
staleness clock that could diverge from the FSM's own clock under
spike-rejected pulses; business-logic (health-score arithmetic) embedded in
the snapshot-capture layer; write-only duplicate fields in `TelemetrySnapshot`;
`g_vibData` becoming fully dead code after conversion; and one unnecessary
refactor plus one redundant mutex acquisition. All six were fixed and
reverified with a second clean build.

---

## Regression Summary

| # | Item | Status |
|---|---|---|
| 1 | Motor State Machine transitions | INTENTIONALLY CHANGED — `signalPresent` now freshness-gated; transition thresholds/timing/`updateMotorStateMachine()` itself untouched |
| 2 | MQTT topics | VERIFIED IDENTICAL |
| 3 | MQTT payload schema and field names | VERIFIED IDENTICAL |
| 4 | Publish timing and intervals | VERIFIED IDENTICAL |
| 5 | Alarm thresholds and hysteresis | VERIFIED IDENTICAL vs. my starting point — **NOT VERIFIED** vs. last-committed baseline: `WARNING_RMS_EXIT`/`WARNING_DWELL_MS` were already absent before v16.5.4 (pre-existing uncommitted `v16.5.3-rpmdiag1` state), not touched by this patch |
| 6 | Fault latch behavior | VERIFIED IDENTICAL (trigger logic in `checkAndLatchFault()`) / INTENTIONALLY CHANGED (replay status message now sources `health_score`/`alarm_level` from the snapshot — numerically identical values) |
| 7 | Analytics inputs | INTENTIONALLY CHANGED — source is now the snapshot, not `g_vibData`; values identical, gating logic untouched |
| 8 | Trend buffer behavior | VERIFIED IDENTICAL |
| 9 | OLED updates | INTENTIONALLY CHANGED — measurement data sourced from snapshot; page/alarm-state selection still reads `g_systemState` directly, unchanged |
| 10 | Current sensor processing | VERIFIED IDENTICAL |
| 11 | Modbus polling | VERIFIED IDENTICAL (one comment-text-only edit, no code change) |
| 12 | Task scheduling and FreeRTOS behavior | VERIFIED IDENTICAL (all 20 task/queue/mutex creation calls byte-identical, no priority/stack/core changes) / INTENTIONALLY CHANGED — mutex acquisition *frequency* reduced (net fewer takes per cycle than even the first v16.5.4 draft, at parity with or better than pre-v16.5.4) |

Full basis: a complete `diff` was generated between the pre-v16.5.4 working-tree
snapshot and the final file — all 28 hunks were individually reviewed and
mapped to one of the three approved improvements or the self-review cleanup;
none touch FSM thresholds, MQTT topic/schema strings, publish-interval values,
alarm thresholds, fault-latch trigger conditions, trend-buffer logic, current-
sensor processing, Modbus transaction code, or task/queue/mutex creation.

---

## Known Limitations

1. **Corner-case FSM behavior change (intended)**: continuous spike-rejected
   pulses (or any pulse stream that never yields an accepted EMA update) for
   > 2 s now invalidates the EMA and forces `signalPresent=false`, which can
   drive the motor state to STOPPING/STOPPED via the absence-timer path
   (`ABSENT_STOPPING_MS`/`ABSENT_STOPPED_MS`) even while pulses keep arriving.
   Previously a stale in-band EMA could hold `signalPresent=true` indefinitely
   in this scenario. This is the literal, intended fix for "stale RPM must
   never produce signalPresent=true" — flagging because it is a genuine
   behavior change in that specific corner case, not because it is wrong.
2. **Startup-from-idle RPM transient**: after invalidation, the EMA seeds
   directly from the first valid pulse instead of dragging the previous
   (stale) value forward. `g_rpmReported`/telemetry `rpm` will therefore read
   "real" sooner during STARTING than before. Warm-up debounce
   (`RUNNING_WARMUP_MS`) is unaffected.
3. **Mutex timeout fallback (pre-existing pattern, now on one additional
   struct)**: `captureTelemetrySnapshot()` uses the same best-effort,
   skip-on-timeout pattern already used throughout this file for
   `mutexVibData`/`mutexSystemState`. On a timeout, that cycle's capture is
   silently skipped and consumers keep the previous snapshot — identical
   failure mode to every other mutex-guarded read/write in this codebase.
4. **Hysteresis constants status unresolved**: `WARNING_RMS_EXIT`/
   `WARNING_DWELL_MS` are absent from the working tree (see item 5 above).
   This predates v16.5.4 and was not evaluated as part of this patch — worth
   confirming intentional before merge, independent of this review.
5. **Uncommitted `v16.5.3-rpmdiag1` diagnostic logging** ([MOTOR-DIAG],
   [MOTOR-TRANSITION], [SIGNAL], [PULSE] Serial prints) still sits underneath
   this patch in the working tree. Decide whether to keep, strip, or split
   into a separate commit before merging v16.5.4.

---

## Test Cases to Execute on Hardware Before Merge

1. **Normal run/stop cycle** — start motor, confirm RUNNING reached after
   `RUNNING_WARMUP_MS` (2.5 s in-band), stop motor, confirm STOPPING at 400 ms
   / STOPPED at 2 s no-pulse, exactly as before. Watch for `[RPM-EMA]
   Invalidated -- no pulse for ...` log at the ~2 s mark.
2. **Restart-from-idle EMA reseed** — after a stop long enough to invalidate
   (>2 s), restart the motor and confirm `[RPM-EMA] Reseeded from first valid
   pulse` fires once, and `rpm`/telemetry converge to the correct running
   value without a multi-second lag from stale-value dragging.
3. **MQTT payload diff** — capture one full `/sensor`, `/status`(`/decision`),
   `/vibration`, and `/trend` payload before and after this patch under
   identical bench conditions; confirm field-for-field identical keys and
   numerically identical values (excluding timestamps/counters that
   naturally differ).
4. **Publish-interval timing** — verify 30 s (NORMAL), 10 s (WARNING), 5 s
   (CRITICAL) cadence unchanged; verify MQTT-offline ring-buffer buffering
   and replay-burst behavior (75 ms between replayed messages) unchanged.
5. **Fault-latch trigger + replay** — force a fault-latch trip, power-cycle
   or reconnect MQTT to trigger the replay path, confirm the replayed
   `/status` payload's `health_score`/`alarm_level`/`fault_*` fields match
   what the pre-patch firmware would have produced for the same fault event.
6. **OLED display** — confirm all pages (machine, axis, warning, critical)
   render identical values and blink timing to pre-patch firmware; confirm
   MAINTENANCE mode entry/exit via button still reacts immediately.
7. **Analytics/trend continuity** — confirm `/trend` slope/EMA/aggregation
   values track the same across a stop/resume cycle (thermal resume vs.
   cold-start clear logic), and that `rpm`/`motor_state` seen by Analytics
   never desyncs from what `/vibration` reports for the same instant.
8. **Sensor-offline path** — disconnect the vibration sensor, confirm
   STATE_NORMAL forcing, OLED offline screen, and `/sensor` offline alert
   payload all behave identically to pre-patch; confirm MAINTENANCE mode is
   preserved (not overwritten to NORMAL) if active during a sensor dropout.
9. **Long-duration soak** — multi-hour run with intermittent stops, watching
   Serial log for `[RPM-EMA]` frequency (should be edge-triggered only, not
   spamming every cycle) and for any `mutex timeout` warnings indicating
   contention regressions.
10. **CTR4A01 current-sourced build** (`-DTEST_CURRENT_SOURCE`, if used in
    bench testing) — confirm `MOTOR_SRC_CURRENT` evidence path (untouched by
    this patch) still behaves identically, since `computeHealthScore()`'s
    gate is `motor_state==2` regardless of which source produced that state.
