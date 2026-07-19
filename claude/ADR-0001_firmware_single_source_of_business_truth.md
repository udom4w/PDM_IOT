# ADR-0001: Firmware as the Single Source of Business Truth

Status: Accepted
Author: (engineering team)
Date: 2026-07-06
Applies to: WTVB02_ESP32S3 firmware, Node-RED, InfluxDB, Dashboard, Edge AI
Related: RFC-0001 (Telemetry Buffer Observability), RFC-0002 (MQTT Telemetry Topic Contract)

Supersedes prior discussion: this ADR adopts a governing principle that is
independent of, and does not require resolving, RFC-0002's canonical-topic
question. RFC-0001/RFC-0002 remain valid with respect to their own scope
(raw `/sensor` buffering, topic contract) and should be read as compatible
background, not as prerequisites to this decision.

---

## Context

This product is an Industrial Edge Monitoring Device. It computes
condition-monitoring outcomes — Health Score, Alarm Code, Bearing Alert,
Motor State — on the ESP32-S3 edge device, and publishes them over MQTT to
a backend stack (Node-RED, InfluxDB, Dashboard, and future Edge AI
consumers) operating on an unreliable 4G link subject to temporary
outages.

Two prior investigations (RFC-0001, RFC-0002) surfaced the same underlying
class of risk from different angles: when more than one part of the
system can independently compute or reconstruct a value that is meant to
represent a business outcome, those parts can silently drift into
disagreement — and the resulting inconsistency is only discovered after
it has already caused a real gap (e.g. missing replay data, mismatched
topic assumptions between firmware and backend).

This ADR exists to close that class of risk at the architectural level,
rather than case by case: it fixes, permanently, *where* business
decisions are allowed to be made, and states what "no telemetry lost
during an outage" actually obligates the system to guarantee.

---

## Problem Statement

Two requirements have historically pulled the architecture in different
directions:

1. The backend (Node-RED, InfluxDB, Dashboard, AI) needs Health Score,
   Alarm Code, Bearing Alert, and Motor State to be available, correct,
   and continuous — including across temporary 4G/MQTT outages.
2. The firmware is a resource-constrained edge device; naively buffering
   every enriched field needed to reconstruct those outcomes is expensive
   in RAM, and any logic duplicated between firmware and backend is a
   long-term maintainability and correctness liability (two
   implementations of the same decision can disagree).

Left unresolved, the natural failure mode is exactly what RFC-0002
documented: firmware and backend each form their own undocumented
assumption about where a business outcome "really" comes from, and those
assumptions diverge without anyone deciding it should happen.

---

## Requirements

- **R1.** Business Decisions (Health Score, Alarm Code, Bearing Alert,
  Motor State, and future Edge AI decisions) shall have exactly one
  computing authority in the system.
- **R2.** No downstream component (Node-RED, InfluxDB, Dashboard, AI)
  shall independently recompute a Business Decision. They shall only
  consume and persist/display what the authority produced.
- **R3.** During a temporary 4G/MQTT outage, Business Telemetry
  (Business Decisions specifically — not all diagnostic data) shall not
  be lost, up to the capacity of a defined replay buffer.
- **R4.** Replay of a Business Decision shall reproduce the original
  decision exactly. Replay shall never trigger, or depend on,
  recomputation.
- **R5.** Diagnostic data (raw sensor features, trend/slope/EMA context,
  anything that informs but is not itself a decision) is explicitly out
  of scope for R3/R4 and may be handled under a separate, weaker
  guarantee.

---

## Decision

**Firmware is the single authoritative source of all Business Decisions.**

Business Decisions are defined as: Health Score, Alarm Code, Bearing
Alert, Motor State, and any future Edge AI decision (e.g. anomaly
classification, remaining-useful-life estimate).

Business Decisions are architecturally separated from diagnostic
telemetry into their own compact, replay-guaranteed record and topic
per domain, distinct from the existing rich/best-effort diagnostic
stream. Diagnostic data continues to be published live, with no replay
obligation — it was never the subject of the "must not be lost" business
requirement, only the decisions were.

Replay, where it applies to Business Decisions, is defined as
retrieval and re-transmission of an already-finalized decision — never
as re-execution of the decision logic against current or reconstructed
state.

---

## Architecture Principles

1. **One authority, no exceptions.** Every Business Decision has exactly
   one place in the system where it is computed: firmware. This applies
   uniformly to existing decisions and to every future Edge AI decision
   added to the product.
2. **Decisions are finalized facts, not derivable views.** Once firmware
   computes a Business Decision, it is treated as an immutable record of
   "what was decided, and when" — not as an intermediate result that any
   component (including firmware itself, later) recomputes differently
   from the same inputs.
3. **Diagnostic data and Business Decisions serve different purposes and
   carry different guarantees.** Diagnostics inform understanding and can
   tolerate gaps; decisions drive alarms, dashboards, and downstream
   automation, and cannot.
4. **Replay preserves, it does not reconstruct.** A replayed Business
   Decision must be bit-identical to the one originally computed live.
   Any replay path that re-derives a decision from raw or partial state
   is, by definition, not compliant with this ADR.
5. **The buffered unit is the decision, sized to be small.** Because only
   the decision itself (not full diagnostic context) must survive an
   outage, the replay-guaranteed record is deliberately minimal — this is
   what keeps R3 affordable on a RAM-constrained edge device.
6. **The pattern is domain-agnostic.** The same authority/replay/topic
   pattern applies uniformly to every current and future sensing domain
   (vibration today; power, thermal, energy as they are added) — no
   domain gets a bespoke exception.

---

## Consequences

- Node-RED, InfluxDB, Dashboard, and AI become **pure consumers** of
  Business Decisions. Any feature request that would require one of them
  to compute or adjust a Health Score, Alarm Code, Bearing Alert, or
  Motor State independently is, by this ADR, out of bounds — the correct
  path is to change firmware, not to add equivalent logic downstream.
- Firmware's responsibility grows: it must not only compute decisions but
  also guarantee their survival across outages, which requires a
  dedicated buffering and replay mechanism for decisions specifically
  (distinct from, and smaller than, any existing raw-telemetry buffer).
- Diagnostic telemetry is explicitly permitted to have gaps during an
  outage. This must be communicated clearly to anyone consuming
  diagnostic data, so a gap there is not mistaken for a violation of this
  ADR.
- Every future Edge AI decision inherits this contract automatically: it
  must be computed only in firmware, captured as a finalized fact at
  decision time, and made replay-safe the same way — there is no opt-out
  path for "just this one new AI feature."
- Historical Business Decisions are permanently tied to the firmware
  logic version that produced them. There is no mechanism, under this
  ADR, for retroactively correcting a historical decision if the
  computing logic is later found to be flawed or is intentionally
  improved.

---

## Trade-offs

Per this project's design principles, every trade-off is stated with
both sides — what is gained and what is given up — rather than only the
side being optimized for.

| Gained | Given up |
|---|---|
| Exactly one implementation of business logic to maintain and audit | No independent downstream cross-check exists to catch a firmware logic bug by disagreement — this is a real reduction in error-detection redundancy, accepted deliberately |
| Small, cheap, replay-guaranteed decision record | Full diagnostic context behind a decision (why it was made) is not guaranteed to survive an outage — only the verdict is |
| Deterministic replay (bit-identical to live) | Historical decisions cannot be recomputed under improved logic later; a formula fix does not retroactively repair past records |
| Firmware changes are the only way to evolve business logic, keeping the system conceptually simple | Firmware iteration is slower and higher-risk to deploy (fleet flash) than a backend deploy would be — evolving business logic is now bottlenecked on firmware release cycles |
| A repeatable, domain-agnostic pattern for adding new sensing domains | Each new domain still adds a nonzero, permanent RAM cost to the device; the pattern is cheap per domain but not free |

---

## Future Expansion

This ADR is written to extend without modification to:

- **Additional sensing domains** (power, thermal, energy, and others):
  each follows the same shape — firmware computes that domain's Business
  Decisions, a small per-domain decision record is buffered and
  replay-guaranteed, diagnostic data for that domain remains live-only.
- **Additional Edge AI decisions**: any new AI-derived outcome (e.g.
  anomaly classification, remaining-useful-life estimate) is, by
  definition under this ADR, a Business Decision — it must be computed
  only in firmware and follow the same capture/replay discipline as
  Health Score or Alarm Code, from the moment it is introduced.
- **Schema evolution**: as new Business Decision fields are added over
  the product's life, consumers need a way to detect a schema they don't
  yet recognize rather than silently misinterpret it. This ADR does not
  mandate a specific mechanism, but any future design work in this area
  must not compromise Principle 4 (replay preserves, never
  reconstructs).

---

## Open Questions

Per this project's design principles, an honest "not yet known" is
preferred over a confident guess. The following are intentionally left
open by this ADR:

1. **Capacity sizing.** How many Business Decision slots (what outage
   duration, in minutes/hours) is the replay buffer required to cover?
   This is a product/commercial requirement, not an engineering default,
   and has not been specified.
2. **Overflow behavior.** If an outage outlasts the replay buffer's
   capacity, what should happen to the decisions that would be
   overwritten — silently drop-oldest (as today's raw buffer does), or
   surface a distinct "decisions were lost" signal to the operator? This
   ADR requires no data loss *within* capacity but does not yet define
   the required behavior *beyond* capacity.
3. **Schema versioning mechanism.** Whether a `schema_version`-style field
   or another mechanism is used to let consumers detect newly added
   decision fields is not decided here.
4. **Diagnostic-gap disclosure.** How (or whether) the Dashboard/AI layer
   should visually or programmatically indicate "diagnostic data has a
   gap here, but the Business Decision for this period is authoritative
   and complete" is not yet defined, and matters for operator trust in
   the system during/after an outage.
5. **Historical correction policy.** Given that historical decisions
   cannot be recomputed under this ADR, is there a business need for a
   separate, explicitly-labeled "corrected/reprocessed" record type in
   the future — and if so, how would it avoid violating Principle 2
   (decisions as immutable finalized facts)? Left for a future ADR if the
   need arises.

---

## Next Steps

- Circulate this ADR for review alongside RFC-0001 and RFC-0002.
- Treat this ADR as the governing constraint for any future design work
  on the Business Decision buffering/replay mechanism, topic layout, or
  new Edge AI decision types.
- No firmware or Node-RED changes should be made on the basis of this ADR
  until it is formally accepted and the Open Questions above are either
  answered or explicitly deferred with owners assigned.
