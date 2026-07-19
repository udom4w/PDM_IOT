# ADR-0003: Telemetry Snapshot Lifecycle

Status: Proposed
Author: (engineering team)
Date: 2026-07-06
Applies to: WTVB02_ESP32S3 firmware, MQTT contract, Node-RED, InfluxDB, Dashboard, Edge AI
Related: ADR-0001 (Firmware as the Single Source of Business Truth),
ADR-0002 (Telemetry Snapshot Architecture)

This ADR defines the lifecycle of a single Telemetry Snapshot instance,
end to end. ADR-0001 established *who* may decide; ADR-0002 established
*what shape* a decision travels in. This ADR establishes *when* a
snapshot is mutable, *when* it becomes an immutable fact, and *which
stage* is the authoritative source of each Business Decision it carries.

Scope note: this document describes stages, ownership, and data-mutability
boundaries only. It does not prescribe function names, data structures, or
code — those remain implementation decisions for a future, separate piece
of work.

---

## Design Note: Snapshot Core vs. Delivery Envelope

Before the stage-by-stage lifecycle, one structural clarification is
necessary, because it resolves an apparent tension in ADR-0002.

ADR-0002 states that a Telemetry Snapshot is immutable in its entirety,
yet also lists `sent_at` as a field legitimately assigned *after*
`captured_at`, at transmission time — which sounds like a mutation of an
already-created object. This ADR resolves that by splitting what
ADR-0002 called "Replay Metadata" into two conceptually distinct parts:

- **Snapshot Core** — Identity, Measurement, Business Decision, Device
  Status, Schema Version, and `captured_at`. This is the part that
  freezes once and never changes again, for the rest of the snapshot's
  existence.
- **Delivery Envelope** — `sent_at`, `replayed`, `replay_sequence`,
  `overflow_preceded`. This is attached fresh at *each* transmission
  event (a live publish is one delivery event; a replay is another) and
  never alters the Snapshot Core it wraps.

This distinction is what allows "the snapshot is immutable" and "`sent_at`
is set at publish/replay time" to both be true without contradiction: the
thing that's immutable is the Core; the Envelope is delivery-event
metadata that rides alongside it, not a field of it.

---

## Lifecycle Stages

### 1. Sensor Acquisition

| | |
|---|---|
| Inputs | Physical vibration signal at the WTVB02 sensor; Modbus RTU/RS485 register map |
| Outputs | Raw register values (RMS, peak, kurtosis, frequency, temperature, RPM pulse count) |
| Mutable data | Local read buffers; per-cycle retry/poll-timing counters |
| Immutable data | None — nothing is a finalized fact yet |
| Owner | Core 0 (acquisition task) |
| Failure handling | Read timeout/retry; stuck-signal detection per axis; sensor restart after a cooldown-gated threshold of consecutive failures |

### 2. Validation

| | |
|---|---|
| Inputs | Raw register values from Stage 1 |
| Outputs | A validity determination (valid / invalid) and, if invalid, a decision to hold the last-known-good value |
| Mutable data | Last-known-good shadow values; consecutive-error counters |
| Immutable data | None |
| Owner | Core 0 (acquisition task, inline) |
| Failure handling | Implausible values (out-of-range, non-numeric) are discarded, not propagated; validity failures increment an error counter used to trigger Stage 1's restart path |

### 3. Signal Conditioning

| | |
|---|---|
| Inputs | Validated raw values from Stage 2; current Motor State (see note below) |
| Outputs | Conditioned measurement values — de-glitched, and explicitly zeroed for any feature that is not meaningful while the motor is not running |
| Mutable data | De-glitch hold state (short-lived, cycle-to-cycle); the shared conditioned-measurement record itself, while being written |
| Immutable data | None yet |
| Owner | Core 0 (acquisition task writes conditioned values); Motor State itself is decided by a separate debounce process on Core 0, ahead of this stage |
| Failure handling | Non-running gating policy: any feature not valid outside a running state is reported as zero/neutral rather than propagating noise-floor garbage |

**Note on Motor State:** Motor State is one of the four values ADR-0001
names as a Business Decision, but in the current architecture it is
decided earlier and on a different core than the other three (Alarm
Code, Health Score, Bearing Alert). This lifecycle treats Business
Decision generation as one *logical* stage (Stage 5) while acknowledging,
per this project's fact-before-assumption principle, that today it is
physically split. This split is noted, not resolved, by this ADR.

### 4. Analytics

| | |
|---|---|
| Inputs | Conditioned measurement stream (Stage 3), accumulated over a rolling historical window |
| Outputs | Trend/slope, moving averages, frequency-drift, and spike-count signals — diagnostic, not decisions |
| Mutable data | Rolling window buffers at multiple resolutions; EMA state; suppression counters gating output validity after a resume-from-stopped event |
| Immutable data | None — analytics state is continuously live by design |
| Owner | Core 1 (analytics task), fixed cadence |
| Failure handling | Output is suppressed (not fabricated) for a bounded number of cycles after any time-discontinuity (e.g. resume from stopped), so a slope computed across a gap is never presented as valid |

### 5. Business Decision Generation — **Authoritative Stage**

| | |
|---|---|
| Inputs | Conditioned measurement (Stage 3); configured thresholds; live debounce/stabilization counters (e.g. bearing warm-up cycles) |
| Outputs | Alarm Code, Alarm Level, Health Score, Bearing Alert (and, per the note above, Motor State — decided earlier but logically part of this same authority) |
| Mutable data | The debounce/stabilization counters themselves advance as a side effect of this stage |
| Immutable data | None at the instant of computation — the output becomes immutable only once it exits this stage and is carried into Stage 6 |
| Owner | Firmware exclusively (ADR-0001) — Core 1 for Alarm/Health/Bearing, Core 0 for Motor State |
| Failure handling | Warm-up/stabilization gating avoids a false decision during a startup transient; a not-running state maps decisions to defined neutral values, never to "unknown" or fabricated values |

This is **the single authoritative point in the entire lifecycle** where
Business Decisions are computed. No stage before this one has enough
information to decide; no stage after this one is permitted to
recompute. Every downstream stage — including the firmware's own
snapshot assembly — only carries this stage's output forward.

### 6. Snapshot Creation

| | |
|---|---|
| Inputs | Stage 3 measurement, Stage 5 decisions, current Device Status readings, Identity constants, a freshly-read capture timestamp |
| Outputs | A fully populated, but not yet sealed, Telemetry Snapshot (Snapshot Core, per ADR-0002) |
| Mutable data | The snapshot instance itself, while its fields are being assigned |
| Immutable data | None — this is the last stage where the snapshot is still "under construction" |
| Owner | Core 1 (the publish-orchestrating stage) |
| Failure handling | If assembly cannot complete correctly (e.g. a size/serialization sanity check fails), the in-progress snapshot is discarded here — it must never proceed to Stage 7 partially formed |

### 7. Snapshot Freeze — **Immutability Boundary**

| | |
|---|---|
| Inputs | The fully-assembled snapshot from Stage 6 |
| Outputs | A sealed Snapshot Core — no field in Identity, Measurement, Business Decision, Device Status, Schema Version, or `captured_at` may be written by any later stage |
| Mutable data | None — this stage's entire purpose is to end mutability |
| Immutable data | The complete Snapshot Core, from this instant onward, for the rest of its existence |
| Owner | Same execution context as Stage 6 — the boundary is a discipline, not necessarily a separate function |
| Failure handling | None applies at this stage — a snapshot either reaches Freeze fully formed (via Stage 6 succeeding) or never exists at all; there is no partial-freeze condition |

**This is the exact point at which a Telemetry Snapshot becomes
immutable** — after full assembly (Stage 6), strictly before any
transmission attempt (Stage 8) or buffering (Stage 9). Freezing before
any network I/O is deliberate: it is what prevents a later stage from
ever substituting live state (e.g. the current time, instead of the
original capture time) into what should be a historical fact — the exact
failure mode identified earlier in this project's replay-feasibility
analysis.

### 8. MQTT Publish (Live Path)

| | |
|---|---|
| Inputs | Frozen Snapshot Core; current MQTT connection state |
| Outputs | An MQTT message wire payload (Snapshot Core + a newly-created Delivery Envelope with `replayed=false`); a success/failure result |
| Mutable data | The Delivery Envelope created for this specific transmission attempt only |
| Immutable data | The Snapshot Core, unchanged |
| Owner | Core 1 (network task) |
| Failure handling | On failure, the snapshot is not discarded — control passes to Stage 9 instead of Stage 11 |

### 9. Replay Buffer Write

| | |
|---|---|
| Inputs | Frozen Snapshot Core (or at minimum its replay-guaranteed subset, per ADR-0002 §7) |
| Outputs | One ring-buffer slot written; buffer head/count updated |
| Mutable data | Ring-buffer bookkeeping (head, count, overflow counter) |
| Immutable data | The copied snapshot content itself — copied, never altered |
| Owner | Core 1 (network task), mutex-guarded |
| Failure handling | Lock-acquisition timeout: this write is dropped and logged, not retried synchronously; buffer full: oldest slot is overwritten and an overflow counter increments — this is the one place data loss is structurally possible, and it is scoped exactly to "beyond configured capacity," matching the business requirement's stated exception |

### 10. Replay Transmission

| | |
|---|---|
| Inputs | Oldest buffered slot; current MQTT connection state (post-reconnect) |
| Outputs | An MQTT message carrying the original frozen Snapshot Core plus a new Delivery Envelope (`replayed=true`, original `captured_at` preserved unchanged, a fresh `sent_at`) |
| Mutable data | Ring-buffer bookkeeping (slot popped on success); the new Delivery Envelope |
| Immutable data | The Snapshot Core — bit-identical to what Stage 7 sealed, never recomputed |
| Owner | Core 1 (network task), rate-limited to avoid flooding the broker on reconnect |
| Failure handling | Publish failure: slot is retained, retried on the next opportunity; slots beyond a defined age limit are dropped as operationally stale, a distinct and explicit decision from a Stage-9 capacity overflow |

### 11. Historical Storage

| | |
|---|---|
| Inputs | An MQTT message (Snapshot Core + Delivery Envelope), live or replayed |
| Outputs | A persisted historical record |
| Mutable data | None |
| Immutable data | The entire snapshot, unconditionally, from this stage onward |
| Owner | Node-RED ingestion flow — a consumer, never a decision-maker (ADR-0001) |
| Failure handling | An unrecognized `schema_version` should be flagged/quarantined rather than guess-parsed; storage-write failures are a Node-RED-side delivery concern, out of scope for this lifecycle document |

### 12. Dashboard Consumption

| | |
|---|---|
| Inputs | Stored or live-subscribed snapshot(s) |
| Outputs | A rendered visual state (alarm indicator, health gauge, trend chart) |
| Mutable data | None belonging to the snapshot; dashboard-local rendering state only |
| Immutable data | Everything it reads |
| Owner | Dashboard layer — a pure consumer |
| Failure handling | A gap in diagnostic Measurement data during an outage is acceptable and should read as a diagnostic gap, not as a missing decision; a gap in Business Decision data would indicate an architecture violation and should be visually distinguishable from an ordinary diagnostic gap |

### 13. AI Consumption

| | |
|---|---|
| Inputs | Historical and/or live snapshots (read-only) |
| Outputs | Model insights/training artifacts; conditionally, new Edge AI Business Decisions for a *future* cycle |
| Mutable data | None regarding any existing snapshot |
| Immutable data | Every snapshot it reads, without exception |
| Owner | AI layer — a pure consumer of all existing snapshots; a producer only if it is specifically the on-device Edge AI component ADR-0001 anticipates, and even then only for the *next* cycle's Stage 5, never by amending a past snapshot |
| Failure handling | Model disagreement with a historical decision is a monitoring/model-quality signal, never grounds to alter the historical record |

**Important scoping caveat:** ADR-0001 reserves Business Decision
authority to firmware. An AI component only produces a genuine Business
Decision (eligible to appear in `edge_ai_decisions`) if it executes
on-device, as part of the firmware's own Stage 5. A backend/cloud AI
model consuming historical snapshots produces analytical insight, not a
Business Decision, under this architecture — conflating the two would
silently violate ADR-0001's single-authority principle. This distinction
is left as an explicit open question for whichever future ADR specifies
the first real Edge AI decision.

---

## Immutability Summary

- A snapshot is **mutable** from the moment Stage 5 finalizes its inputs
  through the end of Stage 6 (assembly).
- A snapshot becomes **immutable at Stage 7 (Freeze)** — before any
  transmission, live or replayed.
- From Stage 7 onward, only a separate Delivery Envelope (created fresh
  at Stage 8 or Stage 10) may carry event-specific metadata; the
  Snapshot Core itself is never touched again by any stage, including
  Historical Storage, Dashboard, or AI consumption.

## Authoritative Stage for Business Decisions

**Stage 5 (Business Decision Generation)** is the sole authoritative
point for Alarm Code, Health Score, Bearing Alert, and Motor State, in
the entire 13-stage lifecycle. Every later stage — including replay,
which by ADR-0001 must never recompute — only carries this stage's
output forward unchanged.

---

## Sequence Diagram

```
Sensor      Core0(Acq+SM)     Core1(Analytics+Publish)     MQTT Broker     Node-RED      InfluxDB     Dashboard/AI
  |               |                       |                     |             |             |              |
  |--raw signal-->|                       |                     |             |             |              |
  |               |--(1 Acquire)          |                     |             |             |              |
  |               |--(2 Validate)         |                     |             |             |              |
  |               |--(3 Condition)        |                     |             |             |              |
  |               |--(Motor State decided)|                     |             |             |              |
  |               |==conditioned data====>|                     |             |             |              |
  |               |                       |--(4 Analytics)      |             |             |              |
  |               |                       |--(5 Decision: alarm/health/bearing)|             |              |
  |               |                       |--(6 Assemble Snapshot)             |             |              |
  |               |                       |--(7 FREEZE - immutable from here)  |             |              |
  |               |                       |                     |             |             |              |
  |               |                       |--(8 Publish attempt)------------->|             |              |
  |               |                       |     MQTT UP:   success ---------->|--(11 Store)>|              |
  |               |                       |     MQTT DOWN: fail               |             |              |
  |               |                       |--(9 Buffer write, replay-guaranteed subset)      |              |
  |               |                       |         ... outage persists ...   |             |              |
  |               |                       |<--(MQTT reconnects)---------------|             |              |
  |               |                       |--(10 Replay: same Core, new envelope)----------->|--(11 Store)>|
  |               |                       |                     |             |             |--(12 Dashboard reads)
  |               |                       |                     |             |             |--(13 AI reads)
```

---

## State Machine (per Snapshot instance)

```
   ACQUIRING
      |
      v
   DECIDING            (Stage 5 — authoritative)
      |
      v
   ASSEMBLING          (Stage 6)
      |            \
      v             v
   FROZEN         DROPPED   <-- assembly failure; snapshot never
      |                         became a fact, no requirement violated
      |
      +----------------------------+
      |                            |
      v                            v
  DELIVERED_LIVE               BUFFERED         (Stage 9; MQTT down
   (Stage 8 success)               |              or Stage 8 failed)
      |                            |
      |                    +-------+-------+
      |                    v               v
      |               REPLAYED         DROPPED   <-- overflow beyond
      |              (Stage 10             configured capacity —
      |               success)              the one loss scenario the
      |                    |                business requirement
      |                    |                explicitly permits
      +--------+-----------+
               v
            STORED           (Stage 11)
               |
               v
           CONSUMED           (Stage 12 / 13 — non-exclusive,
                                repeatable, read-only)
```

Two distinct `DROPPED` paths exist and are not equivalent:

- **`ASSEMBLING → DROPPED`**: failure before Freeze. The snapshot never
  became an immutable fact, so no Business Telemetry was ever "lost" in
  the sense the business requirement cares about.
- **`BUFFERED → DROPPED`**: overflow beyond replay buffer capacity. This
  is real business-telemetry loss, and it is the *only* case the
  business requirement explicitly tolerates — loss must never occur any
  other way, and must never occur within capacity.

---

## Design Rationale

- **Freeze is placed before any I/O, not after.** If freeze happened
  after a publish attempt, a failed publish followed by a retry could
  reuse live state a second time and produce two different values for
  what should be one fact. Freezing first guarantees the value
  transmitted live and the value later buffered/replayed are
  byte-for-bit the same object, computed exactly once.
- **Business Decision generation is treated as one logical stage even
  though it is physically split today (Core 0 for Motor State, Core 1
  for the rest).** Naming it as a single authoritative stage in this ADR
  is a statement about *where authority lives conceptually*, which is
  what downstream consumers need to rely on — it does not require, and
  this ADR does not propose, consolidating the physical implementation.
- **The Delivery Envelope is separated from the Snapshot Core** so that
  "immutable snapshot" and "sent_at is set at transmission time" can both
  be true without contradiction. This also cleanly explains why a
  replayed snapshot and a live one carry an identical Core and differ
  only in their Envelope — exactly the property ADR-0002 relies on for
  Node-RED/Dashboard/AI to treat both uniformly.
- **Buffering only the replay-guaranteed subset (Stage 9), not the full
  diagnostic payload**, is what keeps the "no business telemetry lost"
  guarantee affordable — the same rationale ADR-0002 §7 already
  established, restated here as a lifecycle boundary rather than a field
  list.
- **Two different `DROPPED` states are named explicitly** rather than
  treated as one generic failure, because they carry very different
  weight against the business requirement: one is an ordinary,
  acceptable construction failure; the other is the exact scenario the
  requirement is written to bound ("within replay buffer capacity") and
  must remain observable and rare.
- **AI's role is scoped narrowly and explicitly** (Stage 13) because
  ADR-0001's single-authority principle is easy to erode by accident —
  an AI model that "just this once" adjusts a historical health score,
  even with good intentions, would silently reopen the exact problem
  ADR-0001 was written to close. Naming AI as strictly read-only with
  respect to existing snapshots, and drawing the edge-vs-cloud
  distinction explicitly, is meant to make that erosion harder to do by
  accident.

---

## Open Questions

1. Should the physical split of Business Decision generation (Motor
   State on Core 0, Alarm/Health/Bearing on Core 1) eventually be
   consolidated so Stage 5 is a single, literal execution point rather
   than a logical one? Left open — not required for this ADR's
   guarantees to hold, but relevant to future maintainability.
2. What is the correct age/TTL policy for Stage 10 (replay transmission)
   given the replay-guaranteed subset is smaller than today's raw
   buffer's payload? Sizing is not decided here (see ADR-0001 Open
   Question 1).
3. How should a Stage-9 buffer overflow (the one accepted loss path) be
   surfaced to Dashboard/operators, consistent with ADR-0001's
   still-open "diagnostic-gap disclosure" question?
4. What is the exact boundary condition for treating an on-device model
   as "Edge AI" (Stage 5 participant) versus "AI consumption" (Stage 13,
   read-only)? Left for the ADR that introduces the first concrete Edge
   AI decision type.

---

## Next Steps

- Circulate this ADR alongside ADR-0001 and ADR-0002 for review.
- Treat the Freeze boundary (Stage 7) and the Snapshot Core / Delivery
  Envelope split as binding constraints for any future design work on
  buffering, replay, or snapshot construction.
- No firmware or Node-RED changes should be made on the basis of this ADR
  until it is accepted and the Open Questions above are answered or
  explicitly deferred with owners assigned.
