# Implementation Plan: Telemetry Snapshot (ADR-0001 / ADR-0002 / ADR-0003)

Status: Planning
Author: (engineering team)
Date: 2026-07-06
Governing architecture (frozen): ADR-0001, ADR-0002, ADR-0003
Applies to: `WTVB02_ESP32S3_V16_CM_fault_latch_v3_hardened_patched_v16_5.ino`
(current working copy this session has been analyzing)

This document breaks the frozen architecture into independently
committable phases. It contains no code and prescribes no function
names, data layouts, or algorithms — only what must change, where, why,
and how each change is verified and reversed. Per this project's
workflow rules, no phase below is to be started without a separate,
explicit approval step and a clean build immediately before and after.

---

## Ground Rules Carried Into Every Phase

- **Additive only.** No phase removes, renames, or repurposes an
  existing MQTT field. This matches the project's existing backward-
  compatibility convention and is what keeps every phase independently
  revertible with a plain `git revert` of its single commit.
- **One phase = one commit.** No phase bundles unrelated changes.
- **Cross-core discipline is unchanged.** Any new shared state introduced
  by a phase must follow the existing convention (atomic single-word
  values or mutex/queue-protected structs — never a new shared boolean
  flag with multiple writers).
- **Clean build before and after every phase**, per this project's
  mandatory workflow — not restated per-phase below to avoid repetition,
  but it applies to all of them without exception.

---

## Phase 1 — Schema Version & Identity Fields (Additive)

**Objective:** Add `schema_version` and `domain` (fixed to `"vibration"`
for now) to the three existing MQTT payload builders, with no change to
any existing field. This lays the groundwork ADR-0002's Identity/Schema
Version groups require, without altering any decision or measurement
logic.

**Files to modify:** The single firmware `.ino` — specifically the three
existing JSON payload construction blocks inside the live-publish
function (`/sensor`, `/status`, `/vibration` builders).

**Risk:** Low. Purely additive fields; no control-flow change.

**Regression risk:** Low. Slight increase in JSON buffer usage — must
confirm existing `StaticJsonDocument`/`char buf[]` sizes still have
headroom (each builder already logs a truncation warning if not).

**Test plan:**
- Clean build, zero new warnings.
- MQTT capture (subscribe to all three topics) confirms two new fields
  present, correctly typed, and every previously-existing field
  unchanged in name and value.
- Confirm no truncation warning appears in Serial log across a normal
  RUNNING/STOPPED cycle.

**Rollback plan:** `git revert` the single commit — no downstream
consumer can yet depend on the new fields, so reverting is risk-free.

**Dependencies:** None.
**Difficulty:** Low.
**Independent validation:** Yes — fully self-contained, verifiable from
MQTT capture alone.

---

## Phase 2 — Business Decision Record: Struct + Capture-at-Decision-Time Buffer

**Objective:** Introduce a new, small, dedicated ring buffer holding only
the four Business Decision values (Motor State, Alarm Code, Health
Score, Bearing Alert) plus minimal Identity/timestamp context, populated
at the exact moment those values are finalized (ADR-0003 Stage 5→6/7
boundary) — independent of, and much smaller than, the existing
`/sensor` ring buffer. This phase adds capture only; no replay is wired
yet, so the buffer simply accumulates and overwrites-oldest, exactly
like the existing `/sensor` buffer's proven behavior, but produces no
externally visible change yet.

**Files to modify:** The single firmware `.ino` — a new struct
declaration and new static ring-buffer storage (placed near the existing
`TelemetrySlot_t`/`g_telemBuf` declarations for locality), and one new
call site inserted immediately after Alarm Code/Health Score/Bearing
Alert/Motor State are finalized inside the live-publish function —
strictly before any other live-state mutation in that function (peak
hold reset, bearing counter increment, trend suppression decrements),
per ADR-0003's ordering rationale.

**Risk:** Medium. This is the first phase touching the live-publish
function's internals rather than only its JSON output. The capture call
must read already-finalized local values only — it must not read any
further live global state, or it silently reintroduces the exact
contamination bug identified in this project's prior replay-feasibility
analysis.

**Regression risk:** Medium. Incorrect placement of the new call
(inserted after, rather than before, a live-state mutation) would
capture a decision that has already been perturbed for *this* cycle —
low probability of directly breaking existing behavior, but a
correctness risk specific to this phase's own new code, not to anything
pre-existing. RAM increase is small (an estimated ~20–24 bytes per slot)
and should be confirmed against remaining heap/stack headroom.

**Test plan:**
- Clean build, zero new warnings.
- Add a temporary Serial log line (or use existing Serial infrastructure)
  confirming the new buffer's fill count advances once per publish
  cycle, matching the existing decision values already visible in the
  live `/vibration`/`/status` output for that same cycle.
- Confirm existing `/sensor` buffer's fill/overflow counters are
  completely unaffected (prove isolation between the two buffers).
- Run a full RUNNING → STOPPED → RUNNING cycle and confirm captured
  Motor State transitions match the live-published sequence exactly.

**Rollback plan:** `git revert` the single commit. The new buffer and
call site are additive and unreferenced by any existing consumer, so
reverting removes them cleanly with no cascading effect.

**Dependencies:** Soft dependency on Phase 1 (reuses the same
Identity/Schema Version constants) — could be built independently with
duplicated literals if sequencing requires it, but is not recommended.
**Difficulty:** Medium.
**Independent validation:** Yes, via Serial log and buffer-counter
inspection — no MQTT-visible change yet, so validation is device-local.

---

## Phase 3 — Business Decision Replay Path

**Objective:** Implement replay for the Phase 2 buffer: on MQTT
reconnect, dequeue the oldest buffered decision and publish it verbatim,
with a fresh Delivery Envelope (`replayed=true`, original `captured_at`
preserved, new `sent_at`), to a new dedicated topic. This must be a new,
purpose-built function — **not** a call into the existing live-publish
function — because that function is not a pure function of its inputs
(per this project's prior replay-feasibility finding) and must not be
reused for replay.

**Files to modify:** The single firmware `.ino` — a new replay function
(structurally analogous to the existing `/sensor` replay function:
mutex-guarded peek, JSON build directly from the buffered struct, publish,
pop-on-success only), one new call site in the network task's existing
reconnect-handling loop (rate-limited the same way the existing
`/sensor` replay is), and one new topic string initialized alongside the
existing topic strings in setup.

**Risk:** Medium-High. This is the phase most exposed to the specific
failure class already documented in this project (a replay path that
accidentally reads live state instead of the frozen buffered value).
Correctness here is the crux of the entire ADR-0001 guarantee — the
replayed decision must be provably bit-identical to what was buffered,
with zero live-global reads in the replay code path.

**Regression risk:** Medium. Must coexist with the existing `/sensor`
replay in the same network task loop without changing its rate-limiting
or mutex behavior — a new replay call sharing the same loop iteration
budget could, if not scoped carefully, slow down `/sensor` replay
throughput during a large backlog. Test plan below addresses this
directly.

**Test plan:**
- Clean build, zero new warnings.
- Simulate an outage (disconnect broker) long enough to buffer several
  distinct decisions across at least one Motor State transition.
- Reconnect; capture the replay topic and confirm: values match what was
  buffered (not current live state at replay time), `captured_at` matches
  original capture time, `replayed=true`, oldest-first ordering, one
  slot per loop iteration (verify via timing between messages).
- During the same replay burst, confirm existing `/sensor` replay still
  completes and its own counters/behavior are unchanged from before this
  phase.
- Confirm no decision is ever replayed with a live-state value substituted
  for a buffered one — cross-check every field in the replayed payload
  against the corresponding original buffered slot's contents.

**Rollback plan:** `git revert` the single commit. No existing consumer
depends on the new topic yet, so removal is clean. The Phase 2 buffer
remains intact and harmless without a replay path (as it already was
before this phase).

**Dependencies:** Hard dependency on Phase 2 (buffer must exist and be
populated).
**Difficulty:** Medium-High — the highest-scrutiny phase in this plan.
**Independent validation:** Yes, via direct MQTT capture during a
controlled outage simulation.

---

## Phase 4 — Delivery Envelope Enrichment for the Existing `/sensor` Replay

**Objective:** Add `replay_sequence` and `overflow_preceded` to the
existing, already-proven `/sensor` replay payload, so both replay paths
(the long-standing `/sensor` one and the new Phase 3 decision one) expose
the same Delivery Envelope richness, per ADR-0002/ADR-0003's uniform
envelope shape.

**Files to modify:** The single firmware `.ino` — the existing
`/sensor` replay function only (its JSON payload block), and its
existing pop/overflow-counter bookkeeping (read-only reference, no
behavior change to the counter itself).

**Risk:** Low. Two new fields added to an already-working, well-tested
code path; no change to its control flow, mutex usage, or rate limiting.

**Regression risk:** Low. The only real risk is JSON buffer size
headroom, already checked for truncation in the existing code.

**Test plan:**
- Clean build, zero new warnings.
- Simulate an outage spanning more than the buffer's capacity (force an
  overflow), reconnect, and confirm the first replayed message correctly
  reports `overflow_preceded=true` while subsequent ones in the same
  burst report `false`.
- Confirm `replay_sequence` increments monotonically within a single
  replay burst and resets appropriately on the next.
- Confirm all previously-existing `/sensor` replay fields
  (`replayed`, `buffered_at`, raw values) are byte-identical to
  pre-phase behavior.

**Rollback plan:** `git revert` the single commit — two additive fields
removed, no other behavior affected.

**Dependencies:** Soft dependency on Phase 3 (for envelope-shape
consistency between the two replay paths) — technically could be
implemented before Phase 3 since it only touches existing `/sensor`
code, but sequencing it after keeps both envelopes designed together.
**Difficulty:** Low-Medium.
**Independent validation:** Yes, via MQTT capture during a forced
overflow scenario — independent of Phase 3's own topic.

---

## Phase 5 — Business Decision Buffer Overflow Observability

**Objective:** Add a dedicated overflow counter and a
"minutes-until-decision-loss" style metric for the Phase 2 buffer
specifically, exposed via the existing `/status` payload — distinct from
the existing `/sensor` buffer's overflow counter, because, per ADR-0003,
an overflow of the *decision* buffer is the one case that constitutes a
genuine violation of the business requirement, and must be independently
observable rather than folded into an existing, less-critical metric.

**Files to modify:** The single firmware `.ino` — the Phase 2 buffer's
push logic (increment a new counter on overwrite), and the `/status`
JSON builder (expose the new counter and derived ETA metric).

**Risk:** Low. Counter-increment-and-expose pattern, already proven for
the existing `/sensor` buffer's overflow counter.

**Regression risk:** Low.

**Test plan:**
- Clean build, zero new warnings.
- Force a decision-buffer overflow (outage longer than its capacity) and
  confirm the new counter increments exactly once per overwritten slot,
  independently of the existing `/sensor` overflow counter (which should
  only increment if `/sensor`'s own, differently-sized buffer also
  overflows — confirm the two counters can diverge, proving isolation).
- Confirm the ETA metric decreases plausibly as the buffer fills during
  a simulated outage.

**Rollback plan:** `git revert` the single commit.

**Dependencies:** Hard dependency on Phase 2 (buffer must exist).
**Difficulty:** Low.
**Independent validation:** Yes, via `/status` MQTT capture during a
forced overflow.

---

## Phase 6 — Documentation Correction (Housekeeping)

**Objective:** Correct the stale "dormant, no producer/consumer wired
yet" comments on the trend outbound queue, already identified during
this project's architecture discovery and flagged in RFC-0001 §4.3 —
the producer (`taskAnalytics`) and consumer (network task drain loop)
are in fact both wired and active; only the comments are wrong.

**Files to modify:** The single firmware `.ino` — comment text only, at
the queue's declaration and at its two call sites.

**Risk:** None — comment-only change, zero effect on compiled behavior.

**Regression risk:** None.

**Test plan:** Clean build (byte-identical compiled output expected);
visual review confirming the corrected comment accurately reflects the
traced call sites.

**Rollback plan:** `git revert` the single commit (cosmetic only,
essentially risk-free to revert or keep).

**Dependencies:** None — can be done at any point, independently of every
other phase.
**Difficulty:** Trivial.
**Independent validation:** Yes, by inspection alone.

---

## Phase 7 — Per-Domain Extension (Future, Not Scheduled)

**Objective (deferred):** Replicate the Phase 1–5 pattern for `/power`,
`/thermal`, `/energy` once those sensor domains have real hardware and
requirements defined. Not part of the current roadmap — listed here only
to record that the architecture (ADR-0002 §9) anticipates it, and that
it should not begin until Phases 1–5 have been running in the field
against the vibration domain long enough to validate the pattern.

**Dependencies:** Hard dependency on Phases 1–5 being deployed and
field-validated first.
**Difficulty:** Unestimated — depends entirely on per-domain sensor
integration work not yet scoped.

---

## Dependency Graph

```
Phase 6  (no dependencies — can run anytime)

Phase 1  (no dependencies)
   |
   v
Phase 2  (soft dep: Phase 1)
   |
   +----------------------+
   v                      v
Phase 3               Phase 5
(hard dep: Phase 2)   (hard dep: Phase 2)
   |
   v
Phase 4
(soft dep: Phase 3, for envelope-shape consistency)

Phase 7  (hard dep: Phases 1-5, field-validated) — deferred, not scheduled
```

Phases 3 and 5 are siblings — both depend only on Phase 2, not on each
other, and can be implemented and reviewed in either order, or by two
people in parallel, once Phase 2 has landed.

---

## Implementation Difficulty Summary

| Phase | Difficulty | Why |
|---|---|---|
| 6 | Trivial | Comment-only |
| 1 | Low | Additive JSON fields, no logic change |
| 4 | Low-Medium | Extends existing, proven code path |
| 5 | Low | Repeats an already-proven counter/metric pattern |
| 2 | Medium | First phase touching live-publish internals; ordering-sensitive |
| 3 | Medium-High | Must avoid the project's known replay-correctness failure class |
| 7 | Unestimated | Deferred; scope depends on future hardware/requirements |

---

## Which Phases Can Be Validated Independently

All of them. No phase requires a later phase to exist in order to be
tested — each phase's test plan above is self-contained given only its
stated dependencies already landed:

- Phase 6: inspection only.
- Phase 1: MQTT capture only.
- Phase 2: Serial/device-local buffer-counter inspection only (no new
  MQTT surface yet).
- Phase 3: MQTT capture during a controlled outage simulation.
- Phase 4: MQTT capture during a forced overflow.
- Phase 5: `/status` MQTT capture during a forced overflow.

---

## Implementation Roadmap

Recommended commit order:

1. **Phase 6** — trivial, zero-risk, clears existing technical debt first.
2. **Phase 1** — establishes Identity/Schema Version groundwork.
3. **Phase 2** — introduces the Business Decision buffer (capture only).
4. **Phase 3** and **Phase 5** — can proceed in either order, or in
   parallel, once Phase 2 is merged and validated.
5. **Phase 4** — envelope consistency pass, once Phase 3's envelope shape
   is settled.
6. Field-validate Phases 1–5 in production before scoping **Phase 7**.

Each step above corresponds to exactly one commit, and per this
project's workflow, each requires its own clean build, its own
before/after report of files changed and why, and its own explicit
approval before the next phase begins.

---

## Explicit Non-Goals of This Plan

- No phase in this plan modifies Node-RED, InfluxDB schema, or any
  Dashboard configuration — those are separate, downstream implementation
  efforts that consume the topics this plan produces, out of scope here.
- No phase renames or removes any existing MQTT field, in keeping with
  this project's backward-compatibility convention.
- This plan does not implement Phase 7 (multi-domain expansion) — it is
  recorded for roadmap completeness only.
