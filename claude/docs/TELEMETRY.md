# Telemetry System — WTVB02_ESP32S3_V16_CM_fault_latch_v3_hardened_patched_v16_5

**Source file:** `WTVB02_ESP32S3_V16_CM_fault_latch_v3_hardened_patched_v16_5.ino`
**Verification basis:** every JSON field listed below is quoted directly from the `doc["..."] = ...` / `s["..."] = ...` / `t["..."] = ...` assignment in the source, with line numbers. No field was inferred from naming convention alone.

**Related documents:** [ARCHITECTURE.md](ARCHITECTURE.md) · [MOTOR_STATE.md](MOTOR_STATE.md) · [FIRMWARE_CONFIG_AUDIT_v16.5.md](FIRMWARE_CONFIG_AUDIT_v16.5.md)

## Table of Contents

- [Overview](#overview)
- [MQTT Topics](#mqtt-topics)
- [Publishing Frequency Summary](#publishing-frequency-summary)
- [Sensor Payload (`/sensor`)](#sensor-payload-sensor)
- [Decision Payload (`/decision`)](#decision-payload-decision)
- [Vibration Payload (`/vibration`)](#vibration-payload-vibration)
- [Trend Payload (`/trend`)](#trend-payload-trend)
- [Event Payload (`/vibration/event`)](#event-payload-vibrationevent)
- [Health Score](#health-score)
- [Alarm Pipeline](#alarm-pipeline)
- [Edge Analytics / Trend Generation](#edge-analytics--trend-generation)
- [Offline Resilience](#offline-resilience)

---

## Overview

All telemetry is published over MQTT with mTLS (client certificate auth, `MQTT_QOS = 1`, at-least-once delivery) to `iot.promlogix.com:8883`. Five distinct topic suffixes exist, built at boot from `PLANT_ID`/`MACHINE_ID` (lines 7112-7128), all owned in string form by global `char[]` buffers (`g_mqttTopic`, `g_mqttTopicSensor`, `g_mqttTopicDecision`, `g_mqttTopicTrend`, `g_mqttTopicEvent`).

Two distinct publish architectures coexist in this firmware (see [ARCHITECTURE.md § MQTT Pipeline](ARCHITECTURE.md#mqtt-pipeline)):
- `/sensor`, `/decision`, `/vibration`, `/event` are published **synchronously**, directly from `taskNetwork` (Core 1), which is the sole owner of the `mqttClient` object.
- `/trend` is **built by `taskAnalytics`** (Core 1) but only **published by `taskNetwork`**, handed off through the `queueMqttOutboundTrend` FreeRTOS queue via `enqueueMqttOutbound()` (line 2530).

[⬆ Back to top](#table-of-contents)

## MQTT Topics

Verified from the boot-time topic construction (`snprintf` calls, lines 7112-7128) and every `mqttClient.publish()` call site:

| Topic suffix | Full topic (given `PLANT_ID=plant01`, `MACHINE_ID=pump01`) | Topic variable | Purpose |
|---|---|---|---|
| `/vibration` | `factory/plant01/machine/pump01/vibration` | `g_mqttTopic` | Legacy/backward-compatible full payload — every field kept for existing Grafana dashboards. |
| `/sensor` | `factory/plant01/machine/pump01/sensor` | `g_mqttTopicSensor` | Raw acquisition stage: sensor readings + motor context + harmonic features. Also used for the sensor-offline alert and for offline-buffer replay. |
| `/decision` | `factory/plant01/machine/pump01/decision` | `g_mqttTopicDecision` | Alarm/health/trend-direction summary. **Note:** internal code comments label this stage `"/status"` (e.g. line 6260's header comment `PUBLISH 2 of 3 — /status`, and the JSON field `stage: "status"`) even though the actual MQTT topic string is `/decision` — this is a verified naming inconsistency between code comments/JSON content and the real topic, not a documentation error. |
| `/trend` | `factory/plant01/machine/pump01/trend` | `g_mqttTopicTrend` | Multi-resolution buffer/slope/EMA/spike analytics, 60s cadence. |
| `/vibration/event` | `factory/plant01/machine/pump01/vibration/event` | `g_mqttTopicEvent` | Narrow-purpose: **only** carries `maintenance_reset` audit events triggered by an operator button action (line 5016: `evDoc["event"] = "maintenance_reset"`). It does *not* carry general fault/alarm events — those are delivered on `/decision` instead (see [Alarm Pipeline](#alarm-pipeline)). |

[⬆ Back to top](#table-of-contents)

## Publishing Frequency Summary

| Topic | Trigger | Cadence | Source line |
|---|---|---|---|
| `/sensor`, `/decision`, `/vibration` (normal) | `publishTelemetry()` called from `taskNetwork` | Adaptive: **30s** (`STATE_NORMAL`) / **10s** (`STATE_WARNING`) / **5s** (`STATE_CRITICAL`) | 4877-4879, switch on `localState` |
| `/sensor` (offline variant) | Sensor read invalid (`!localVibData.valid`) | Every 30s while offline | 4946 (`lastOfflinePublish`) |
| `/decision` (fault-latch variant) | `g_fl.pending == true` | Every `taskNetwork` loop iteration (~100ms) until successfully published, then NVS-cleared | 5076 (`if (mqttConnSnap25 && snapPending)`) |
| `/trend` | `taskAnalytics`, 1Hz task tick | Every 60th tick = 60s, gated on MQTT connected + `g_buf1sCount >= 4` | 6842-6849 |
| `/vibration/event` | Operator maintenance-reset button press | On demand, drained from `queueMaintEvent` | 4993 |

All three "normal" payloads (`/sensor`, `/decision`, `/vibration`) share a single condition gate and are published back-to-back inside one `publishTelemetry()` call (line 6008) — they always go out together, never independently, under normal operation.

[⬆ Back to top](#table-of-contents)

## Sensor Payload (`/sensor`)

Built at lines 6159-6255 inside `publishTelemetry()`, `StaticJsonDocument<960>`. All fields as they appear in source order:

| Field | Source | Description |
|---|---|---|
| `plant` | `PLANT_ID` | Plant/site identity |
| `machine_id` | `MACHINE_ID` | Machine tag identity |
| `sensor_id` | `SENSOR_ID` | Sensor identity (`"vb01"`) |
| `stage` | literal `"sensor"` | Pipeline stage tag |
| `execution_location` | literal `"edge"` | Processing location tag |
| `sensor_status` | literal `"ONLINE"` | (This payload is only built when the sensor read is valid — the offline variant is a separate, simpler doc, see below) |
| `deglitch_count` | `g_deglitchCount` | Cumulative VRMS single-sample de-glitch events |
| `rms`, `vx`, `vy`, `vz` | `reportedRms`/`reportedVx`/`reportedVy`/`reportedVz`, rounded to 2 decimals | Estimated RMS velocity (overall + per-axis); **gated to 0 outside `MOTOR_RUNNING`** |
| `peak` | `currentPeak` (= `g_velPeakHold`, then reset) | True peak velocity hold [mm/s]; gated to 0 outside RUNNING |
| `peak_velocity_x/y/z` | `data->peak_velocity_x/y/z` | Signed peak velocity per axis (register 0x3A-0x3C); gated to 0 outside RUNNING |
| `temp` | `data->temperature`, 1 decimal | Sensor temperature |
| `rpm` | `data->rpm` | Current RPM reading (always live regardless of `g_motorStateSource` — see [MOTOR_STATE.md](MOTOR_STATE.md#processrpm)) |
| `freq_x/y/z` | `freqX`/`freqY`/`freqZ` | Dominant frequency per axis, 1 decimal |
| `freq_ratio_x/y/z` | `freqRatioX`/`freqRatioY`/`freqRatioZ` | Frequency / rotational-frequency ratio (harmonic order); 0 unless `motor_state==RUNNING && rpm >= RPM_FREQ_GATE` |
| `crest_factor` | `crestFactor` (= `cf_max`) | Acceleration crest factor, gated to RUNNING |
| `cf_x/y/z` | `data->cf_x/y/z` | Per-axis crest factor, gated to 0 outside RUNNING |
| `kurtosis_x/y/z/max` | `kx`/`ky`/`kz`/`kmax` | Kurtosis values, valid only when RUNNING (`kurtosisValid`) |
| `kurtosis_axis` | `kaxis` | Dominant kurtosis axis ("X"/"Y"/"Z"/"-") |
| `kurtosis_valid` | `kurtosisValid` | `true` only when `motor_state == MOTOR_RUNNING` |
| `dominant_vibration_axis` | `domVibAxis` | Axis with highest velocity RMS |
| `bearing_alert` | `bearingAlert` | `"WARMING_UP"`/`"CONFIRMED"`/`"EARLY_WARNING"`/`"NORMAL"` when RUNNING; `"STARTING"`/`"STOPPING"`/`"STOPPED"` otherwise (state name, not a bearing assessment, when not running) |
| `motor_state` | `data->motor_state` | 0=STOPPED, 1=STARTING, 2=RUNNING, 3=STOPPING — see [MOTOR_STATE.md](MOTOR_STATE.md) |
| `rotation_signal_ok` | `data->prox` | Proximity/rotation signal flag |
| `operating_hours_total` | `data->runtime_hour` | Cumulative runtime hours |
| `reset_reason` | `g_resetReasonStr` | Last reset cause ("POWER_ON"/"BROWNOUT"/"PANIC" etc.) |
| `reboot_count` | `g_rebootCount` | Cumulative reboot count (NVS-persisted) |
| `timestamp` | `tsBuf` | ISO8601 UTC, or `"not_available"` if RTC invalid |
| `time_synced` | `g_timeSync.synced` | Whether NTP sync has succeeded since boot |

**Offline variant** (lines 4947-4972, `StaticJsonDocument<296>`, published only when `!localVibData.valid`): a much smaller doc with `plant`, `machine_id`, `sensor_id`, `stage: "sensor"`, `execution_location: "edge"`, `sensor_status: "OFFLINE"`, `error_count` (`g_sensorErrors`), `uptime_s`, and `ts` if RTC valid.

[⬆ Back to top](#table-of-contents)

## Decision Payload (`/decision`)

**Two distinct JSON shapes are published to this one topic**, verified as separate code paths:

**Shape A — routine status** (lines 6265-6320, inside `publishTelemetry()`, `StaticJsonDocument<800>`, code-labeled `"/status"` internally):

| Field | Source | Description |
|---|---|---|
| `plant`, `machine_id`, `sensor_id` | identity constants | |
| `stage` | literal `"status"` | |
| `execution_location` | literal `"edge"` | |
| `alarm_code` | `alarmCode` (0/1/2) | 0=NORMAL, 1=WARNING, 2=CRITICAL; only evaluated when `motor_state==2` |
| `alarm_level` | `alarmLevel` | `"NORMAL"`/`"WARNING"`/`"CRITICAL"` string form |
| `health_score` | `healthScore` | See [Health Score](#health-score) |
| `bearing_alert`, `kurtosis_max`, `kurtosis_axis`, `kurtosis_valid`, `dominant_vibration_axis` | same computation as the `/sensor` payload | |
| `freq_alert` | `freqGateOpen && g_trendResult.freq_alert` | |
| `freq_drift_x/y/z` | `g_trendResult.freq_drift_x/y/z`, gated by `freqGateOpen` | |
| `trend_dir` | `trendDirStr` (`"UP"`/`"DOWN"`/`"STABLE"`) | From `g_trendResult.trend_dir` |
| `rms_slope` | `g_trendResult.rms_slope` | |
| `spike_count` | `g_trendResult.spike_count` | |
| `ttw_estimate_h` | `g_trendResult.ttw_hours` | Only included if `> 0.0` |
| `timestamp` | `tsBuf` | |
| `fault_latch_pending` | snapshotted `g_fl.pending` | |
| `fault_latch_count` | snapshotted `g_flCount` | |
| `telemetry_buffer_pending` | `g_telemBufCount` | Offline ring-buffer backlog depth |

**Shape B — fault-latch delivery** (lines 5120-5151, `StaticJsonDocument<1024>`, published only when a fault event is pending — see [Alarm Pipeline](#alarm-pipeline)):

| Field | Source | Description |
|---|---|---|
| `plant`, `machine_id`, `sensor_id`, `stage:"status"`, `execution_location:"edge"` | same as above | |
| `alarm_code`, `alarm_level`, `health_score` | recomputed from a fresh `g_vibData`/`g_systemState` snapshot at delivery time | |
| `timestamp` | current time at delivery | |
| `fault_latch_pending` | literal `true` | |
| `fault_event` | `faultEventStr(snapCode)` | `"BEARING_CONFIRMED"` / `"ALARM_CRITICAL"` / `"HEALTH_LOW"` / etc. |
| `fault_severity` | `faultSeverity(snapCode)` | Numeric severity ranking |
| `fault_ts` / `fault_ts_iso` / `fault_ts_unknown` | latched event timestamp, or explicit unknown-timestamp markers if the RTC/NTP timestamp wasn't valid at latch time | |
| `fault_rms` | latched RMS value at the moment of the fault | |
| `fault_kurtosis` | latched kurtosis value at the moment of the fault | |
| `fault_latch_count` | cumulative fault-latch counter | |

[⬆ Back to top](#table-of-contents)

## Vibration Payload (`/vibration`)

Built at lines 6326-6436 inside `publishTelemetry()`, `StaticJsonDocument<2048>` — the largest payload, kept for backward compatibility with existing Grafana dashboards ("ALL original field names kept verbatim," source comment line 6322). Superset of `/sensor` + `/decision` fields plus full trend detail:

| Field | Source | Description |
|---|---|---|
| `plant`, `machine_id`, `sensor_id` | identity | |
| `rms`, `vx`, `vy`, `vz`, `peak`, `temp`, `rpm` | same gated values as `/sensor` | |
| `freq_x/y/z`, `freq_ratio_x/y/z` | same as `/sensor` | |
| `motor_state` | `data->motor_state` | Current field name |
| `state` | `data->motor_state` | **`[DEPRECATED]`** alias of `motor_state`, kept per the source's own comment (lines 6346-6349) so existing Grafana dashboards don't break; scheduled for removal once dashboards migrate |
| `operating_hours_total`, `rotation_signal_ok` | same as `/sensor` | |
| `alarm_code`, `alarm_level`, `health_score` | same as `/decision` | |
| `crest_factor`, `cf_x/y/z` | same as `/sensor` | |
| `kurtosis_x/y/z/max`, `kurtosis_axis`, `kurtosis_valid`, `dominant_vibration_axis`, `bearing_alert` | same as `/sensor` | |
| `sensor_status` | literal `"ONLINE"` | |
| `deglitch_count` | `g_deglitchCount` | |
| `analysis_ready` | `isAnalysisReady()` | Derived: is the analytics pipeline currently unfrozen? |
| `freeze_reason` | `analysisReasonStr(analysisReason())` | If frozen, why (enum → string) |
| `rms_slope`, `temp_slope`, `current_slope` | `g_trendResult.*` | Linear-regression slopes |
| `current_buf_count` | `g_currentCount` | CTR4A01 trend buffer fill level (0..`CURRENT_BUF_SIZE`) |
| `current_read_errors` | `g_ctReadErrors` | Cumulative CTR4A01 Modbus failures since boot — the field explicitly designed (per source comment 6381-6383) to disambiguate "current_slope=0 because motor stopped" from "current_slope=0 because CTR4A01 reads are failing" |
| `trend_dir`, `spike_count` | same as `/decision` | |
| `freq_drift_x/y/z`, `freq_alert`, `freq_gate_open` | same freq-gating as `/decision`, plus the raw gate-open debug flag | |
| `ttw_hours` | `g_trendResult.ttw_hours` | Only if `> 0.0` |
| `trend_window_s` | `(g_trendResult.window_samples * 250) / 1000` | |
| `slope_1s/10s/60s` + `slope_ready_1s/10s/60s` | `g_trendResult.*` | Multi-resolution slope + readiness flags |
| `ema_dir`, `ema_rms` | `g_trendResult.*` | |
| `stddev_1min`, `max_rms_10min` | `g_trendResult.*` | |
| `agg_buf_1s/10s/60s` | `g_buf1sCount`/`g_buf10sCount`/`g_buf60sCount` | Aggregation buffer fill levels |
| `timestamp`, `time_synced` | as above | |
| `sync_age_s` | `(millis() - g_timeSync.lastSyncMillis) / 1000` | Only if `time_synced` |

[⬆ Back to top](#table-of-contents)

## Trend Payload (`/trend`)

Built at lines 6879-6961 inside `taskAnalytics()`, `StaticJsonDocument<640>`:

| Field | Source | Description |
|---|---|---|
| `plant`, `machine_id`, `sensor_id`, `stage:"trend"`, `execution_location:"edge"` | identity | |
| `buf_1s/10s/60s` | `g_buf1sCount`/`g_buf10sCount`/`g_buf60sCount` | Buffer fill levels |
| `ready_1s/10s/60s` | `g_trendResult.slope_ready_*` | Whether each window has enough samples |
| `slope_1s/10s/60s` | `aggLinRegSlope(...)`, computed inline under `mutexAggBufs` | Only included when the corresponding `ready_*` flag is true |
| `max_rms_10min` | computed from `g_buf10s` | |
| `max_rms_60min` | computed from `g_buf60s`, only if `n60m > 0` | |
| `stddev_1min` | averaged `stddev_rms` across `g_buf1s` | |
| `slope_var_1s/10s/60s` | `g_slopeVar_1s/10s/60s` | Patent-relevant OSG/FVRI inputs |
| `ema_rms`, `ema_dir`, `ema_delta` | `g_emaRms`/`g_emaDir`/`g_emaDelta` | |
| `spike_count` | `g_trendResult.spike_count` | |
| `trend_gap_s` | `g_lastResumeGapS` | Seconds of the most recent analytics-freeze gap, so a downstream consumer knows where the time-series isn't continuous |
| `freq_alert`, `freq_drift_x/y/z` | gated by a fresh `motor_state`/`rpm` snapshot taken under `mutexVibData` | |
| `slot_dur_ms` | `g_slotDur1sMs` | Current RPM-adaptive `buf1s` slot width (Patent Claim 2) |
| `slot_revs_target` | `SLOT_REVS_TARGET` | |
| `timestamp` | `tsA` | |

[⬆ Back to top](#table-of-contents)

## Event Payload (`/vibration/event`)

Built at lines 5011-5020, `StaticJsonDocument<256>`. Fields: `plant_id`, `machine_id`, `event` (literal `"maintenance_reset"`), `timestamp`, `state` (literal `"WARMUP"`). This is the **only** producer of `/vibration/event` traffic in the source — confirmed via `mqttClient.publish(g_mqttTopicEvent, ...)` occurring at exactly one call site (line 5022).

[⬆ Back to top](#table-of-contents)

## Health Score

Computed identically in three places (`publishTelemetry()` line 6019-6034, the fault-latch delivery path line 5101-5109) — same formula, only evaluated when `motor_state == 2` (`MOTOR_RUNNING`); otherwise fixed at `100`:

```cpp
normalized   = (rms_overall - BASELINE_RMS) / (CRITICAL_RMS - BASELINE_RMS) * 100.0f
healthScore  = clamp(0, 100, round(100.0f - normalized))
```

With defaults `BASELINE_RMS = 2.8`, `CRITICAL_RMS = 11.2` (both mm/s): a machine at baseline RMS scores 100; at critical RMS it scores 0; linear in between.

`alarm_code`/`alarm_level` are derived independently, from the RMS-threshold state machine (`STATE_NORMAL`/`STATE_WARNING`/`STATE_CRITICAL`), verified at lines 4454-4464:

```cpp
if (g_motorRunState != MOTOR_RUNNING)      newState = STATE_NORMAL;   // not evaluated while stopped
else if (g_sensorWarmupReads > 0)          newState = STATE_NORMAL;   // suppress post-reconnect spike
else if (rms < WARNING_RMS)                newState = STATE_NORMAL;
else if (rms < CRITICAL_RMS)               newState = STATE_WARNING;
else                                        newState = STATE_CRITICAL;
```

`WARNING_RMS = 4.5`, `CRITICAL_RMS = 11.2` mm/s.

[⬆ Back to top](#table-of-contents)

## Alarm Pipeline

`checkAndLatchFault()` (line 2565, called from `taskStateMachine` at line 4512) implements a **single-slot, severity-overwrite latch**: at most one fault event is "pending" at a time; a new event only overwrites the pending one if strictly higher severity.

Event codes and severity ranking (lines 248-258 in the config audit's [Telemetry section](FIRMWARE_CONFIG_AUDIT_v16.5.md#telemetry-50-items)):

| Event | Code | Severity | Trigger condition (lines 2578-2588) |
|---|---|---|---|
| `FL_EVT_BEARING` → `"BEARING_CONFIRMED"` | 3 | 4 (highest) | Bearing fault newly confirmed (edge-triggered on `bearingConfirmed && !g_flPrevBearing`) |
| `FL_EVT_CRITICAL` → `"ALARM_CRITICAL"` | 2 | 3 | Newly entered `STATE_CRITICAL` |
| `FL_EVT_HEALTH` → `"HEALTH_LOW"` | 4 | 2 | Health score newly dropped to/below `FL_HEALTH_LOW_THOLD` (30) |
| `FL_EVT_WARNING` → `"ALARM_WARNING"` | 1 | 1 | Newly entered `STATE_WARNING` from `STATE_NORMAL` |
| `FL_EVT_NONE` → `"UNKNOWN"` (default case) | 0 | 0 (lowest) | No event this cycle |

`faultSeverity()` (line 2177) and `faultEventStr()` (line 2187) are pure functions of the code, called only from inside `checkAndLatchFault()`'s locked section (per the source's own concurrency comment, lines 2157-2170).

On latch: the event is persisted to NVS (`fault_latch` namespace, `FL_NS`) with a snapshot-then-release-mutex pattern specifically to avoid an `ipc1` stack overflow that a prior version hit when calling `Preferences`/NVS flash writes while still holding `mutexFaultLatch` (documented in the `[v16.3n]` comment at line 2639-2643). Delivery happens on the next `taskNetwork` iteration where MQTT is connected (Shape B of the [`/decision` payload](#decision-payload-decision)); on successful publish, `clearFaultLatchNVS()` runs and the pending flag clears — if publish fails, the latch stays pending and retries the following iteration.

[⬆ Back to top](#table-of-contents)

## Edge Analytics / Trend Generation

`calcTrend()` (line 5798) computes linear-regression RMS slope over a raw circular buffer (`g_trendBuf`, `TREND_BUF_SIZE=240` samples @ 4Hz = 60s), spike detection (`SPIKE_RMS_FACTOR`), and frequency drift (`FREQ_DRIFT_THRESH`) — called once per `publishTelemetry()` invocation (line 6136).

Independently, `taskAnalytics()` (1Hz, Core 1) maintains three cascaded multi-resolution buffers built from `g_trendBuf` snapshots:

| Buffer | Slot width | Slots | Total history | Size constant |
|---|---|---|---|---|
| `g_buf1s` | RPM-adaptive, `[SLOT_DUR_MIN_MS, SLOT_DUR_MAX_MS]` (200-5000ms), 1000ms fixed when RPM < `MIN_RPM_VALID` | 60 | ~60s (variable) | `AGG_BUF_1S_SIZE` |
| `g_buf10s` | Fixed 10s (cascaded from `g_buf1s`) | 60 | ~10 min | `AGG_BUF_10S_SIZE` |
| `g_buf60s` | Fixed 60s (cascaded from `g_buf10s`) | 60 | ~60 min | `AGG_BUF_60S_SIZE` |

`computeSlotDurMs()` (line 6464, Patent Claim 2) targets `SLOT_REVS_TARGET` (20) full shaft revolutions per `buf1s` slot: `ms = (SLOT_REVS_TARGET * 60000) / rpm`, clamped to `[SLOT_DUR_MIN_MS, SLOT_DUR_MAX_MS]`. An EMA of RMS (`EMA_ALPHA = 0.20`, `EMA_DIR_THRESHOLD = 0.003`) tracks slow drift direction independent of the windowed slopes.

[⬆ Back to top](#table-of-contents)

## Offline Resilience

When MQTT is disconnected at a scheduled publish moment, `pushTelemBuf()` (line 2303) stores a `VibrationData_t` + `MachineState_t` snapshot into a 120-slot RAM ring buffer (`TELEM_BUF_SIZE = 120`, ≈60 min at the 30s normal cadence) rather than dropping it. Once MQTT reconnects, `replayTelemBuf()` (line 2369) pops and publishes one slot per `taskNetwork` loop iteration (~75ms spacing, capped at ~9s worst-case burst for a full 120-slot backlog — source comment line 4896). This ring buffer is guarded by `mutexTelemBuf` and is entirely RAM-resident — no NVS persistence across a power cycle (per the buffer's own header comment, line 1483: *"Phase 1 — RAM-only, no NVS persistence"*).

[⬆ Back to top](#table-of-contents)

---

*This document describes firmware behavior as of the source state at documentation time. No source code was modified to produce it.*
