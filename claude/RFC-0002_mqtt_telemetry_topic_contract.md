# RFC-0002: MQTT Telemetry Topic Contract

Status: Proposed
Author: (engineering team)
Date: 2026-07-06
Applies to: WTVB02_ESP32S3 firmware, Node-RED "Alarm Engine v3", InfluxDB ingestion
Related: RFC-0001 (Telemetry Buffer Observability), PATCH_NOTES_v16.5.md

---

## 1. Summary

This RFC defines the canonical vs. derived relationship between the
firmware's MQTT topics, and states it as an explicit, written contract.

This document exists because an investigation into "missing replay data
in InfluxDB" revealed that the firmware and the backend (Node-RED) had
each independently formed a different, undocumented assumption about
which topic was the system's primary telemetry stream. Both sides
implemented their assumption correctly and consistently — but the two
assumptions did not match each other. This RFC closes that gap by
writing the contract down.

This is not a bug report and does not assign fault to any single
implementation. Current backend implementation subscribes only to
`/vibration`, which is inconsistent with the telemetry contract defined
in this RFC. Once this RFC is agreed, that inconsistency becomes the
actionable follow-up — tracked separately, outside this document.

---

## 2. Evidence: What the Source Code Actually Does

Traced directly from `publishTelemetry()` in the firmware:

```
Sensor (WTVB02, Modbus)
        │
        ▼
  VibrationData_t   (raw, in-memory struct)
        │
        ▼
  publishTelemetry()
        │
        ├──► build /sensor payload directly from VibrationData_t
        │           │
        │           ▼
        │     publish(/sensor)  ──────────────┐
        │                                     │  buffered on MQTT
        │                                     │  outage, replayed
        │                                     │  on reconnect
        │                                     ▼
        │                              Telemetry Ring Buffer
        │                                     │
        ├──► calculate healthScore,           │ (replay)
        │            alarmCode,               ▼
        │            bearingAlert        Node-RED (should
        │            (derived values)     consume here)
        │           │
        │           ▼
        │     build /vibration payload
        │     (raw fields + derived fields)
        │           │
        │           ▼
        └────► publish(/vibration)  ──── no buffer, no replay
                                          (live-only stream)
```

Key facts, in the order they occur in code:

1. `/sensor` is built **first**, populated directly from `VibrationData_t`
   fields (`data->rms_x`, `data->rpm`, etc.) with no intermediate
   calculation.
2. Only *after* `/sensor` is built does the function compute
   `alarmCode`, `healthScore`, and `bearingAlert` — none of which exist
   in `VibrationData_t`. These are derived in `publishTelemetry()` itself
   from state-machine thresholds and stabilization gates.
3. `/vibration` is built **second**, reusing those already-computed
   derived values alongside a copy of the same raw fields `/sensor` has.
4. The telemetry ring buffer (`pushTelemBuf` / `replayTelemBuf`) reads
   and writes exclusively against `/sensor`'s topic
   (`g_mqttTopicSensor`). No code path anywhere in the firmware buffers
   or replays `/vibration`.

None of this is incidental. A buffer/replay mechanism was deliberately
built for one topic and not the other, and that topic is the one built
from unprocessed sensor data.

---

## 3. Architecture Decision

**Decision:** `/sensor` SHALL be the canonical telemetry stream.

Reason:
- Originates directly from `VibrationData_t`, with no derived/calculated
  fields.
- Replay-capable — backed by the 120-slot ring buffer described in
  RFC-0001.
- Carries `buffered_at`, preserving the original sample time even when
  delivery is delayed.
- Protected against data loss during temporary MQTT outages, within
  buffer capacity.

**Decision:** `/vibration` SHALL be treated as a derived stream.

Reason:
- Contains calculated fields (`alarm_code`, `alarm_level`,
  `health_score`, `bearing_alert`) that are reproducible from `/sensor`
  plus firmware-side (or downstream-side) logic — they are not
  independent data.
- Not replayed, and not required to be: anything it carries can be
  recomputed post-hoc from `/sensor`'s replayed history if ever needed.
- Intended for direct-to-dashboard consumption where a live, low-latency,
  pre-enriched view is more valuable than replay guarantees.

---

## 4. Topic Contract Table

| Topic | Purpose | Replay | Primary Consumer(s) |
|---|---|---|---|
| `/sensor` | Raw, canonical machine telemetry | Yes | Node-RED, historical/analytics reconstruction |
| `/vibration` | Derived machine state (alarm, health, bearing) | No | Live dashboard |
| `/status` | Device/connectivity status | No | Monitoring |
| `/trend` | Derived analytics (slope, EMA, drift) | No (small 6-slot outbound queue only — see RFC-0001 §4.3 for correction of a stale "dormant" comment on this queue) | Grafana |

---

## 5. Contract Rules

| Rule | Requirement |
|---|---|
| CR-001 | `/sensor` must never lose data during temporary MQTT outages, within buffer capacity (120 slots, capacity varies with publish interval — see RFC-0001). |
| CR-002 | `/sensor` is replay-capable. Replayed messages are marked `"replayed": true` and carry the original sample time as `"buffered_at"`. |
| CR-003 | `/vibration` is derived from `/sensor`. Any consumer of `/vibration` must be able to tolerate its fields being reproducible from `/sensor` plus derivation logic — `/vibration` itself is not an independent source of truth. |
| CR-004 | `/vibration` replay is not required. Gaps in `/vibration` during an outage are acceptable by design; a consumer needing historical continuity should reconstruct from `/sensor`'s replayed data instead. |
| CR-005 | Downstream analytics shall consume `/sensor` as the authoritative source for historical reconstruction (including any future trend/health recomputation — see RFC-0001 §5 for why `/trend` itself is intentionally near-real-time-only). |

---

## 6. Known Inconsistency (as of this RFC)

Current backend implementation (Node-RED, "Alarm Engine v3", the single
MQTT In node in the ingestion flow) subscribes only to
`factory/+/machine/+/vibration`, which is inconsistent with the
telemetry contract defined in this RFC. As a direct consequence:

- `/sensor` messages — including all replayed messages following an
  MQTT outage — are never received by Node-RED, and therefore never
  reach InfluxDB.
- InfluxDB currently contains only derived (`/vibration`) fields
  (`alarm_code`, `health_score`, `bearing_alert`, etc.), with no
  `replayed` or `buffered_at` fields ever recorded, and no historical
  backfill after an outage.

This was discovered, not caused, by this RFC. The corrective change
(what Node-RED should subscribe to, and how derived fields should be
handled if `/sensor` becomes its primary input) is intentionally left
as a follow-up implementation task, tracked separately — this document
defines the contract the fix must satisfy, not the fix itself.

---

## 7. Open Questions

1. If Node-RED moves to consuming `/sensor` as canonical, should
   `/vibration`'s derived fields (`alarm_code`, `health_score`,
   `bearing_alert`) be recomputed in Node-RED from `/sensor`, or should
   Node-RED continue consuming `/vibration` *in addition to* `/sensor`
   for those fields specifically (dual-subscribe)?
2. Should replayed `/sensor` messages, once ingested, trigger any
   backfill recomputation of derived fields for the outage window, or
   is it acceptable for historical `/sensor` data to exist in InfluxDB
   without corresponding `alarm_code`/`health_score` values for that
   period?
3. Does this contract need a versioning mechanism (e.g. a `schema_version`
   field) so future firmware changes to either topic's shape can be
   detected by consumers automatically, rather than discovered the way
   this one was?

---

## 8. Next Steps

- Circulate this RFC for review alongside RFC-0001.
- Once the contract is agreed, open a separate, scoped implementation
  task for Node-RED (and/or firmware, depending on answers to Section 7)
  to bring the backend into compliance with CR-001 through CR-005.
- No changes to v16.5 firmware or the Node-RED flow should be made on
  the basis of this investigation until this RFC is settled.
