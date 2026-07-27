# Motor State Architecture — P2 Planning

## 1. P1 Completion Summary

P1 covered the `MotorStateEvidence` / `buildMotorStateEvidence()` /
`updateMotorStateMachine()` subsystem end to end:

- Observed on hardware as continuous STARTING⇄STOPPING flapping every
  ~250-500ms. Root-caused and fixed: `ageMs` was compared against
  RPM-tuned globals (`NO_PULSE_STOPPING_MS`, `FORCE_STOP_TIMEOUT_MS`)
  regardless of active source. Fix added source-specific
  `ageStoppingMs`/`ageStoppedMs` to `MotorStateEvidence`, derived from
  `CURRENT_SAMPLE_INTERVAL_MS` for `MOTOR_SRC_CURRENT`. Verified on
  hardware: 5.5-minute bench capture, zero oscillation recurrence,
  stable RUNNING.
- Full state-transition design review against a genuine cold-boot log —
  confirmed no logic bug, only a cosmetic same-cycle state-skip artifact
  inherent to the classifier's memoryless design (e.g. RUNNING can jump
  directly to STOPPED, skipping STOPPING, when two thresholds are crossed
  within one evaluation cycle) — accepted, not patched.
- Robustness review: confirmed communication failure (RS485 timeout,
  Modbus retry exhaustion, sensor reboot) can be misreported as motor
  failure. Findings deferred to P2 (see Remaining Technical Debt and
  Proposed Work Items below).
- Architecture compliance review against 4 agreed rules — 2 pass, 2 had
  minor violations (RPM diagnostics leaking into `updateMotorStateMachine()`
  logging; RPM filtering living outside `buildMotorStateEvidence()`).
- Hygiene cleanup: updated `MotorRunState_t`/`MotorStateSource` comments to
  match the shared business-state architecture; removed RPM-specific fields
  from all 6 diagnostic print sites in `updateMotorStateMachine()`.

**Committed as `6160c05`**, on top of the prior motor-state commit chain
(`e690765` → `b2ebf7e` → `424657a` → `879af65` → `6160c05`). Clean compile
verified (exit 0, no new warnings, flash usage slightly *decreased*).

## 2. Current Repository Status

- `HEAD` = `6160c05`, working tree clean for the `.ino` file.
- `CT_TURNS` = `2` (unchanged) — this edit has been "pending independent
  verification" since before P1 started (see `PATCH_NOTES_v16.5.md`,
  v16.5.5 closure note); P1 did not touch or resolve it.
- `build_info.h` regenerated, correctly reports `6160c05-dirty`
  (uncommitted by request) — because the file itself is uncommitted, plus
  3 untracked files (2 PROMLOGIX docs one level up, 1 bench capture
  script). None of these are source code and none are P2-blocking.

## 3. Remaining Technical Debt

| Ref | Item | Source | Severity |
|---|---|---|---|
| D1 | `CT_TURNS` not yet commissioned/verified against installed CT clamp hardware | Pre-existing, predates P1 | High — affects every current reading |
| D2 | `MOTOR_NAMEPLATE_CURRENT_A` = 2.0A placeholder | Code comment, pre-existing | High — `CURRENT_ON/OFF_THRESHOLD_A` derive from it |
| D3 | Comms failure can be reported as motor STOPPED/STOPPING (no distinct "stale evidence" signal) | Robustness review | Medium — real, logged on hardware (`teraterm.log`), no protective-logic impact but misleading telemetry |
| D4 | No CTR4A01 consecutive-failure/offline tracking (WTVB02 has `g_sensorOffline`, CTR4A01 doesn't) | Robustness review | Medium |
| D5 | CURRENT's `s_currentLatched`/`s_currentFiltered` never invalidate on sustained staleness (RPM has `MAX_EMA_INTERVAL_US`, CURRENT doesn't) | Robustness review | Medium |
| D6 | RPM filtering lives in `processRPM()`, not `buildMotorStateEvidence()` (Rule 4, literal wording) | Compliance review | Low — has a legitimate reason (`g_rpmFiltered` is dual-purpose telemetry) |
| D7 | `buildMotorStateEvidence()` signature closed over exactly 2 sources' raw inputs | Compliance review | Low — only matters if a 3rd source is ever added |
| D8 | `case MOTOR_SRC_RPM: default:` silently absorbs unrecognized future enum values | Compliance review | Low — latent trap, no current impact |

## 4. Proposed P2 Work Items (prioritized)

1. **Verify/resolve `CT_TURNS`** (addresses D1) — bench/field measurement
   against the actual installed CT clamp turns count; not a code change, a
   commissioning task that unblocks trusting any `MOTOR_SRC_CURRENT`
   reading.
2. **Commission `MOTOR_NAMEPLATE_CURRENT_A`** (addresses D2) — replace the
   2.0A placeholder with the real motor's nameplate FLA once known;
   recalibrates `CURRENT_ON/OFF_THRESHOLD_A` automatically.
3. **Additive comms-health telemetry** (addresses D3, D4; no FSM changes,
   per the robustness review's own constraint):
   - Surface the STOPPING/STOPPED trigger axis (`age>threshold` vs.
     `absent>threshold`) to MQTT, not just Serial.
   - Add a CTR4A01 consecutive-failure counter mirroring the existing
     `g_sensorOffline`/`MODBUS_OFFLINE_THRESHOLD` pattern.
4. **CURRENT staleness invalidation** (addresses D5) — give
   `s_currentLatched` an explicit invalidation path after sustained
   non-success, mirroring `g_rpmEvidence.valid`'s `MAX_EMA_INTERVAL_US`
   pattern.
5. **Document the Rule 4 clarification** (addresses D6) — record in-repo
   that dual-purpose signal conditioning (telemetry + evidence) may live
   outside `buildMotorStateEvidence()`; no code change.
6. **3rd-source extensibility hardening** (addresses D7, D8) — only
   if/when a real 3rd source is planned: give `buildMotorStateEvidence()`
   a self-describing input instead of two positional parameters, and make
   the `MOTOR_SRC_RPM` fallthrough explicit rather than default-silent.

## 5. Recommended Implementation Order

### Dependency-ordered work (Items 1–4)

`1 → 2 → 3 → 4`

Items 1-2 are commissioning/measurement tasks, not firmware changes, and
block trusting the signal that everything else in this list operates on —
do them first. Items 3-4 build directly on the just-fixed staleness
mechanism and address the robustness review's most concrete finding
(comms failure misreported as motor failure) on a now-correctly-calibrated
signal.

### Deferred / Conditional work (Items 5–6)

These sit outside the dependency chain above — neither depends on items
1-4, nor is depended on by them.

- **Item 5** has no external trigger and no dependency; it is grouped
  here because it is documentation-only and independent, not because it
  is blocked. It can be done at any time, in parallel with any of items
  1-4, at near-zero cost.
- **Item 6** has a genuine trigger condition: it is not scheduled at all
  until a 3rd evidence source is actually planned. Attempting it earlier
  would be speculative work against no current requirement.

## 6. Out of Scope

Explicitly excluded from P2, to keep scope aligned with the frozen
architecture:

- Introducing a 5th `MotorRunState_t` value (e.g., `UNKNOWN`/
  `SENSOR_FAULT`) — any additional confidence signal must remain additive
  telemetry, not a new state (per the robustness and compliance reviews).
- Any change to `updateMotorStateMachine()`'s decision logic, branch
  structure, or existing thresholds — the architecture is frozen.
- Full implementation of `MOTOR_SRC_PROXIMITY` as an independent evidence
  source — remains a documented RPM-mirroring placeholder; no
  requirements exist yet to implement it for real.
- Firmware-wide hygiene review beyond the motor-state subsystem — P1's
  cleanup was explicitly scoped to `MotorStateEvidence`/
  `buildMotorStateEvidence()`/`updateMotorStateMachine()` only.
- MQTT schema/topic redesign — item 3's telemetry additions are new,
  additive fields on existing topics only; no existing topic, field
  meaning, or payload structure changes.
- Grafana / Node-RED / InfluxDB redesign — any downstream
  dashboard/pipeline work consuming the new telemetry fields is out of
  scope for P2; P2 only produces the fields, it does not consume them.
