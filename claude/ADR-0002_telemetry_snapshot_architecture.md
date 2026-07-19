# ADR-0002: Telemetry Snapshot Architecture

Status: Proposed
Author: (engineering team)
Date: 2026-07-06
Applies to: WTVB02_ESP32S3 firmware, MQTT contract, Node-RED, InfluxDB, Dashboard, Edge AI
Related: ADR-0001 (Firmware as the Single Source of Business Truth), RFC-0001, RFC-0002

Builds directly on ADR-0001's accepted principles:
firmware is the only computer of Business Decisions; replay preserves a
decision, it never recomputes it; downstream components never
recalculate. This ADR defines the single data structure — the
**Telemetry Snapshot** — that carries those decisions (and their
supporting context) through every layer of the system, so that "the
contract" is one concrete, versioned shape rather than an implicit
agreement spread across multiple topics and payload builders.

---

## 1. Fields of a Telemetry Snapshot

A Telemetry Snapshot is organized into six field groups. Grouping is by
**purpose and guarantee level**, not by which sensor produced the value —
this is what lets the same six-group shape serve every current and future
domain.

### Identity
Describes *what this snapshot is about* — never a measured or decided
value, always known at capture time from device configuration.

| Field | Meaning |
|---|---|
| `plant_id` | Site/plant identifier |
| `machine_id` | Machine identifier |
| `sensor_id` | Physical sensor identifier |
| `domain` | `"vibration"` \| `"power"` \| `"thermal"` \| `"energy"` \| future domains |
| `firmware_version` | Version of the firmware logic that produced this snapshot's Business Decision fields |

### Measurement
Raw/diagnostic sensor features. Domain-specific in exact field names, but
the group itself is domain-agnostic. For the vibration domain, drawn from
what the firmware already captures: `rms_overall`, `rms_x`, `rms_y`,
`rms_z`, `peak`, `temperature`, `rpm`, `freq_x`, `freq_y`, `freq_z`,
`freq_ratio_x/y/z`, `crest_factor`, `cf_x/y/z`, `kurtosis_x/y/z`,
`kurtosis_max`, `kurtosis_axis`, `dominant_vibration_axis`.

### Business Decision
Exactly the set ADR-0001 names, plus its explicitly-anticipated future
extension point:

| Field | Meaning |
|---|---|
| `motor_state` | Firmware's decided run state (not a raw reading — it is itself the output of a debounce/decision process) |
| `alarm_code` / `alarm_level` | Firmware's alarm classification |
| `health_score` | Firmware's computed health metric |
| `bearing_alert` | Firmware's bearing-condition classification |
| `edge_ai_decisions` | Reserved, open-ended object for future Edge AI outputs (e.g. anomaly class, RUL estimate) — extensible without breaking existing consumers, governed by `schema_version` |

### Device Status
Describes the *device's own* health/condition — not the monitored
asset's. Diagnostic to the business decision, but still operationally
relevant to fleet health.

| Field | Meaning |
|---|---|
| `sensor_status` | `ONLINE` / `OFFLINE` |
| `reset_reason` / `reboot_count` | Boot diagnostics |
| `deglitch_count` | Cumulative sensor-noise correction count |
| `rotation_signal_ok` | Physical rotation-sensing confirmation |
| `fault_latch_pending` / `fault_latch_count` | Latched fault state |
| `telemetry_buffer_pending` | Current replay-buffer backlog (device-level, not asset-level) |
| `time_synced` / `sync_age_s` | NTP sync status |

### Replay Metadata
Present only in the sense that it is populated meaningfully on a replayed
record; absent/null fields here are the signal that a snapshot was
delivered live.

| Field | Meaning |
|---|---|
| `captured_at` | Original capture timestamp (epoch) — the moment the snapshot was created |
| `sent_at` | Transmission timestamp — set at publish or replay time, distinct from `captured_at` |
| `replayed` | `true` if this snapshot is being delivered via the replay path, `false`/absent if live |
| `replay_sequence` | Ordinal position within a replay burst (aids downstream gap detection) |
| `overflow_preceded` | `true` if one or more older snapshots were dropped (buffer overflow) immediately before this one |

### Schema Version
| Field | Meaning |
|---|---|
| `schema_version` | Integer, incremented whenever the snapshot shape changes; lets a consumer detect fields it doesn't yet understand rather than silently misreading them |

---

## 2. Which fields are immutable?

**All fields, once a snapshot instance is created, are immutable.** A
Telemetry Snapshot is a finalized fact (ADR-0001 Principle 2), not a
mutable record that gets updated in place. This applies uniformly across
all six groups: Identity, Measurement, Business Decision, Device Status,
Replay Metadata, and Schema Version are all fixed at the moment the
snapshot is captured.

The one deliberate exception in *timing*, not in mutability, is `sent_at`
— it is legitimately assigned later than `captured_at` (at the moment of
transmission or replay), but once assigned to a given transmission event
it does not change either. No field is ever edited after being written.

## 3. Which fields must never change after the snapshot is created?

This is the strict subset where a violation would break the ADR-0001
contract itself, not just data hygiene:

- **All Business Decision fields** (`motor_state`, `alarm_code`,
  `alarm_level`, `health_score`, `bearing_alert`, `edge_ai_decisions`) —
  changing any of these after creation is, by definition, a recomputation,
  which ADR-0001 forbids outright.
- **`captured_at`** — must never be overwritten with the time of replay
  or retransmission. This is the specific failure mode identified in
  prior analysis of `publishTelemetry()`: reusing "now" instead of the
  original capture time silently corrupts historical accuracy.
- **`schema_version`** — a given snapshot instance keeps the schema
  version it was created under, even after the schema evolves; new
  snapshots get the new version, old ones are not retroactively
  relabeled.
- **Identity fields** — a snapshot's `plant_id`/`machine_id`/`sensor_id`/
  `domain` must never be reassigned after creation.

## 4. Which fields are optional?

- **All of Replay Metadata** except `captured_at` — `sent_at` beyond
  first assignment, `replayed`, `replay_sequence`, and
  `overflow_preceded` are only meaningful in the context of a delivery
  event and may be absent/default on some transports.
- **Domain-specific Measurement and Device Status fields** that don't
  apply to every domain — e.g. `rotation_signal_ok` and `deglitch_count`
  are vibration/rotating-machinery concepts and would not appear in a
  `power` or `energy` domain snapshot; each domain defines its own
  applicable subset of Measurement and Device Status fields.
- **`edge_ai_decisions`** — absent entirely until an Edge AI capability
  producing that specific decision type ships.
- **`firmware_version`** is treated as required in principle (needed for
  the traceability ADR-0001 implies) but may be technically optional on
  transports where it is redundant with an out-of-band device registry —
  flagged as an open question below rather than settled here.

## 5. Which fields are diagnostic only?

The entire **Measurement** group, and most of the **Device Status**
group. These inform understanding of *why* a decision was made or *how*
the device is doing, but are not themselves decisions. Per ADR-0001 R5,
gaps in these fields during an outage are acceptable by design — they are
explicitly not covered by the "must not be lost" guarantee.

## 6. Which fields are business critical?

The entire **Business Decision** group — `motor_state`, `alarm_code`,
`alarm_level`, `health_score`, `bearing_alert`, `edge_ai_decisions` — and,
adjacently, **`schema_version`**, since without it a consumer cannot
safely interpret an evolving Business Decision block. `schema_version` is
not itself a decision, but it is critical infrastructure *for* the
decision fields, and should be treated with the same "never absent, never
altered" rigor.

`motor_state` is called out specifically because it is easy to
mis-classify as a raw measurement — it is not. It is the output of a
stateful debounce/decision process on the firmware side, which is exactly
the category of thing ADR-0001 reserves to firmware alone.

## 7. Which fields must always survive replay?

- The full **Business Decision** group.
- Enough **Identity** to interpret the decision unambiguously:
  `plant_id`, `machine_id`, `sensor_id`, `domain`.
- **`captured_at`** and **`schema_version`**, without which the decision
  cannot be correctly placed in time or correctly parsed.

This is deliberately the smallest set that satisfies ADR-0001 R3/R4 — it
excludes Measurement and most Device Status fields by design, keeping the
replay-guaranteed record small (this is what makes the guarantee
affordable on a RAM-constrained device, per ADR-0001's Trade-offs
section).

Whether any single Device Status field (e.g. `fault_latch_pending` at
decision time) should also be pulled into the replay-guaranteed set is
left open rather than decided here — see Open Questions.

---

## 8. Proposed JSON Schema

The following illustrates the **shape** of a Telemetry Snapshot — a
non-normative example instance, not an implementation. Field types and
exact naming are proposals for review, not a final wire format.

```json
{
  "schema_version": 1,

  "identity": {
    "plant_id": "plant01",
    "machine_id": "pump01",
    "sensor_id": "vb01",
    "domain": "vibration",
    "firmware_version": "v16.5"
  },

  "measurement": {
    "rms_overall": 4.12,
    "rms_x": 2.10,
    "rms_y": 1.98,
    "rms_z": 3.05,
    "peak": 6.40,
    "temperature": 38.5,
    "rpm": 1480.0,
    "freq_x": 24.6,
    "freq_y": 24.5,
    "freq_z": 49.1,
    "freq_ratio_x": 1.0,
    "freq_ratio_y": 1.0,
    "freq_ratio_z": 2.0,
    "crest_factor": 3.2,
    "cf_x": 3.0,
    "cf_y": 3.1,
    "cf_z": 3.2,
    "kurtosis_x": 3.4,
    "kurtosis_y": 3.1,
    "kurtosis_z": 5.8,
    "kurtosis_max": 5.8,
    "kurtosis_axis": "Z",
    "dominant_vibration_axis": "Z"
  },

  "business_decision": {
    "motor_state": "RUNNING",
    "alarm_code": 1,
    "alarm_level": "WARNING",
    "health_score": 72,
    "bearing_alert": "EARLY_WARNING",
    "edge_ai_decisions": {}
  },

  "device_status": {
    "sensor_status": "ONLINE",
    "reset_reason": "POWER_ON",
    "reboot_count": 14,
    "deglitch_count": 3,
    "rotation_signal_ok": true,
    "fault_latch_pending": false,
    "fault_latch_count": 0,
    "telemetry_buffer_pending": 0,
    "time_synced": true,
    "sync_age_s": 42
  },

  "replay_metadata": {
    "captured_at": "2026-07-06T09:14:00Z",
    "sent_at": "2026-07-06T09:14:01Z",
    "replayed": false,
    "replay_sequence": null,
    "overflow_preceded": false
  }
}
```

A replayed snapshot differs only within `replay_metadata` (and,
naturally, in having an older `captured_at`) — every other group is
identical in shape to a live snapshot, which is what allows Node-RED,
InfluxDB, the Dashboard, and AI to consume both without special-casing
replay at the schema level.

---

## 9. How this snapshot supports future domains (power, thermal, energy)

Three of the six groups — **Device Status**, **Replay Metadata**, and
**Schema Version** — are already fully domain-agnostic; they require no
change to add a new domain. **Identity** requires only populating
`domain` with the new value. Only **Measurement** and **Business
Decision** need domain-specific field sets, and both groups are designed
to vary in *content* while keeping the same *role*:

- `power`: Measurement might carry `voltage`, `current`, `power_factor`;
  Business Decision might carry `overload_alert`, `power_quality_state`.
- `thermal`: Measurement might carry `surface_temp`, `ambient_temp`,
  `gradient`; Business Decision might carry `thermal_alarm`,
  `derate_state`.
- `energy`: Measurement might carry `kwh_interval`, `demand_peak`;
  Business Decision might carry `efficiency_alert`, `consumption_anomaly`.

Because the *contract* (six groups, replay guarantees applying only to
Identity + Business Decision + `captured_at` + `schema_version`) is fixed
regardless of domain, adding a domain is populating two groups with new
fields, not designing a new architecture. This directly operationalizes
ADR-0001 Architecture Principle 6 ("the pattern is domain-agnostic") as a
concrete, reusable schema template rather than a stated intention.

---

## 10. Why the Telemetry Snapshot is the contract across the whole pipeline

```
Firmware → MQTT → Node-RED → InfluxDB → Dashboard → AI
```

Every one of these components, under ADR-0001, is either the sole
producer of Business Decisions (firmware) or a pure consumer (everything
downstream). A Telemetry Snapshot is the artifact that makes that
division enforceable rather than aspirational:

- **Firmware → MQTT**: firmware emits exactly one snapshot per capture
  event; nothing upstream of MQTT ever re-opens or edits it.
- **MQTT → Node-RED**: Node-RED's job becomes "route and persist the
  snapshot," not "interpret and recompute" — because the snapshot already
  contains the finished Business Decision, there is nothing left for
  Node-RED to calculate. This is what turns ADR-0001's Rule 3 ("Node-RED
  never recalculates") from a policy into something structurally true of
  the data it receives.
- **Node-RED → InfluxDB**: the snapshot's group structure maps directly
  onto a historian record — raw Measurement and immutable Business
  Decision land in the same write, at the same timestamp, with no
  after-the-fact join between two independently-timed streams.
- **InfluxDB → Dashboard**: the Dashboard reads Business Decision fields
  directly for alarm/health display; it has no basis to compute its own
  version because the snapshot's Business Decision group is presented as
  already-finished, not as inputs.
- **→ AI**: AI consumes the same snapshot shape both as historical
  training input (Measurement + Business Decision together, correctly
  time-aligned) and, going forward, as the producer of new
  `edge_ai_decisions` entries — which re-enter the system as part of the
  *same* contract rather than a parallel, bespoke format.

Because every layer reads the identical shape, `schema_version` becomes
the single point where the whole pipeline agrees on "what a Business
Decision even looks like right now" — one seam to manage, instead of an
implicit, per-topic assumption at every hop (the exact failure mode RFC-
0002 traced back to firmware and Node-RED each guessing independently).

---

## Open Questions

1. Should `firmware_version` be mandatory in every snapshot, or is it
   acceptable to resolve it out-of-band from a device registry keyed by
   `sensor_id`? Affects payload size on every message, however small.
2. Should any single Device Status field (e.g. `fault_latch_pending`) be
   pulled into the replay-guaranteed subset, given it could materially
   change how a replayed Business Decision is interpreted? Left open
   per §7.
3. What is the concrete `schema_version` bump policy — does adding an
   optional field require a version bump, or only removing/renaming a
   field? Not decided here.
4. Should `edge_ai_decisions` be a single open object (as shown) or a
   list of typed, versioned decision entries, to support multiple
   concurrent AI models per domain? Left for a future ADR when the first
   Edge AI decision type is actually specified.
5. Domain-specific Measurement/Business Decision field names for power,
   thermal, and energy are illustrative only in this document and require
   their own review once those domains are actually scoped.

---

## Next Steps

- Circulate this ADR alongside ADR-0001 for review.
- Treat the six-group shape and the §7 replay-guaranteed subset as the
  binding contract for any future firmware, Node-RED, or InfluxDB schema
  work — no implementation should proceed until this ADR is accepted and
  the Open Questions above are answered or explicitly deferred with
  owners assigned.
