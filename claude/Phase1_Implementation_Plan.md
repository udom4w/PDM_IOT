# Phase 1 Implementation Plan — Condition Monitoring (pump01)

**Status:** 🟡 Plan, staged and conservative by design — reflects review correction
that the original code-review recommendations overstated confidence in two
places and one recommendation was correctly deferred entirely.

---

## 1. Corrections from the code review (per review feedback)

| Original claim | Corrected framing |
|---|---|
| "Threshold เดิมไม่พอ" (20% threshold is insufficient) | **"Evidence suggests the current threshold may be insufficient"** — RFC-0005/0006 show events slipping past 20%, but sampling, DSP behavior, and timing haven't been ruled out as contributing factors. Threshold alone is not proven as the sole cause. |
| "ต้องเปลี่ยนเป็น per-sample state machine" (single-hold must change) | **"Likely beneficial, not yet proven"** — 3-11 sample durations are real (RFC-0005/0006), but no A/B test has compared single-hold vs. a state machine on the same events. Treat as a hypothesis to validate, not a mandated change. |
| "Consolidate 8 Modbus transactions → 1" for Phase 1 | **Deferred to v17, out of Phase 1 scope entirely.** Production firmware's transaction pattern is stable and working; changing it risks UART timing, queue behavior, and retry logic for a benefit that isn't clearly needed yet. Revisit only if Phase 1 telemetry shows bus time is actually a bottleneck. |

---

## 2. Staged rollout (replaces the original "do 1+2 together" recommendation)

Per review: don't let diagnostics become decisions before there's data to
justify it. Each phase only unlocks the next once its own evidence is in.

### Phase 1.1 — Add Peak telemetry (VX/VY/VZ), read-only, zero decision impact
- Add `readHoldingRegisters` calls for VX/VY/VZ (0x3A-0x3C) alongside the
  existing VRMS reads.
- Publish as new telemetry fields only (e.g. `peak_x`, `peak_y`, `peak_z`).
- **Explicitly NOT wired into any fault-latch, deglitch, or alarm logic yet.**
- Goal: start accumulating real Peak data so Phase 1.2-1.4 have something to
  validate against, without touching anything that currently works.

### Phase 1.2 — `PhysicalInvariant()` as a diagnostic-only layer
- New, separate function — does not modify `rms_overall`, `motor_state`, or
  any existing deglitch variable.
- Checks: `Peak < 0`, `Peak/RMS` ratio outside a wide sanity band, `NaN`,
  `Inf`.
- On violation: **log only** (Serial + a telemetry counter), do not suppress,
  do not trigger fault-latch, do not affect alarms.
- This mirrors the "don't touch existing logic" principle the review
  specifically praised — Phase 1.2 is purely additive and observational.

### Phase 1.3 — Collect statistics, ~30 days
- Let Phase 1.1 + 1.2 run in the field untouched.
- Measure: how often does `PhysicalInvariant()` actually fire on real
  production data? Does it correlate with known real events (motor faults,
  known noise sources) or does it mostly fire during otherwise-uneventful
  periods (suggesting DSP-side artifacts, per RFC-0006/0007)?
- This is the step that turns "likely beneficial" into either "confirmed
  beneficial" or "not worth the complexity" — decided by data, not by more
  RFC analysis.

### Phase 1.4 — Only then, wire `PhysicalInvariant()` into decision logic
- If Phase 1.3 shows a meaningful, reliable signal, promote specific checks
  into the fault-latch / alarm path.
- If Phase 1.3 shows mostly noise, keep it diagnostic-only indefinitely —
  that itself is a valid, useful outcome, not a failure.

---

## 3. New addition (per review): Diagnostic Mode — auto-triggered FIFO capture

This is the piece that directly connects RFC-0007's FIFO characterization
work to product value, rather than leaving it as a reverse-engineering
side-quest. **Revised per review feedback below** — the original design had
a single trigger source and an immediate blocking call; both are corrected
here before any implementation starts.

### 3.1 Trigger Manager, not a single trigger

Per review: `PhysicalInvariant()` should not be the only thing allowed to
ask for a FIFO capture. FIFO capture is expensive (multi-second, different
Modbus transaction shape), so it deserves a proper arbitration point with
multiple legitimate callers, not a hardcoded single caller:

```
Trigger Manager
    │
    ├── PhysicalInvariant() violation
    ├── Communication error (repeated Modbus failure)
    ├── Sensor reset detected
    ├── Temperature alarm
    └── Manual request (operator/debug command)
            │
            ▼
      FIFO Request Queue
            │
            ▼
      FIFO Task (separate FreeRTOS task)
            │
            ▼
      Capture (1024 samples + metadata, §3.3)
            │
            ▼
      Upload for analysis
```

**Trigger priority tiers (per review):** not every trigger source deserves
equal standing when the queue is under pressure.

```
Critical (never dropped)
├── Manual request
├── Sensor reset detected
└── Communication error

Normal (droppable under load)
├── PhysicalInvariant() violation
└── Temperature alarm
```

If the queue is full, **Normal-tier triggers may be dropped; Critical-tier
triggers must not be** — this protects the ability to capture evidence for
the events most likely to matter (an operator explicitly asking, or the
sensor/link itself being in a bad state) even if lower-priority noise (e.g.
a `PhysicalInvariant()` misfire storm) is filling the queue.

### 3.2 Queue, not a direct blocking call

Per review: `Event → Queue → FIFO Task → Capture`, not
`Event → FIFO Capture` inline. With multiple trigger sources (3.1), two
triggers could otherwise fire close together and race on the same Modbus
bus. A queue in front of a single dedicated FIFO task serializes capture
requests naturally and gives a clean place to drop/coalesce duplicate
requests (e.g. don't queue a second capture if one for the same cause is
already pending).

**Capture policy while a capture is already in progress (per review):**
needs an explicit answer, not left implicit. Options considered: *ignore*
the new trigger, *merge* it into the in-progress capture's metadata,
*replace* the in-progress capture with the new one, or simply *queue* it
behind the current one. **Proposed for Phase 1: queue it** (bounded by the
priority tiers in §3.1 and the rate limiter in §3.5) — this is the simplest
option and defers the harder "replace vs. merge" tradeoff until field data
shows whether concurrent trigger collisions are common enough to justify
the added complexity.

### 3.3 Rich metadata per capture, not just the 1024 samples

Per review: **metadata will matter more than the waveform itself in 6
months.** Each saved FIFO capture should include, alongside the 1024
samples:

| Field | Why |
|---|---|
| Timestamp | When it happened |
| Machine / sensor ID | Which asset (matters once this scales past pump01) |
| Motor state | Running/stopped/starting at capture time |
| Temperature | Correlate thermal state with the event |
| VRMS, Frequency, Peak (if 1.1 shipped) | The register-level readings that were live at capture time |
| Alarm/trigger reason | Which of the 3.1 sources fired — this is what makes the capture interpretable later |
| Firmware version | Which logic version produced this capture |
| SR (sample rate register) | Needed to interpret the FIFO samples' time axis correctly (RFC-0006/0007 dependency) |
| Gain / scale setting, if applicable | Same reason as SR |
| Capture source | Distinguish auto-triggered vs. manual request |
| Firmware git commit | Exact code version that produced this capture — more precise than a version string alone |
| RFC version | Which RFC-000X's findings this capture logic was built against, at time of capture |
| Reason code | Machine-readable version of the trigger reason (§3.1), for automated later analysis without parsing free-text logs |

### 3.4 FIFO Task state machine (per review — the missing lifecycle)

§3.1-3.3 describe the architecture (who can trigger, how requests queue,
what gets saved) but not the FIFO Task's own lifecycle. Per review, this
needs an explicit state diagram before implementation, not left implicit:

```
IDLE
  │  (request dequeued)
  ▼
WAIT_REQUEST
  │  (send FIFO read command)
  ▼
CAPTURING
  │
  ├── CRC OK ──► SAVE ──► IDLE
  │
  └── Timeout ──► FAILED ──► RETRY? ──► IDLE
```

This makes explicit what was previously only implied: a capture can fail
(timeout, CRC mismatch — both already known to happen on real hardware per
RFC-0006/0007 field testing) and the task must return to `IDLE` cleanly
either way, never get stuck in `CAPTURING`. The `RETRY?` decision (how many
retries, backoff) is an open implementation parameter, not decided here.

### 3.5 Capture rate limiter (per review — protects against runaway triggers)

Per review: without a limiter, a failing sensor that fires
`PhysicalInvariant()` violations every poll cycle (entirely plausible — this
is close to what RFC-0006 already observed happening intermittently) would
queue continuous FIFO captures, starving the Trigger Manager's other
sources and adding sustained load the NFRs in §4 are meant to prevent.

- **Initial proposal: max 1 capture per 5 minutes (subject to field
  validation)** — 5 minutes is a reasonable starting point, not a number
  backed by field data yet.
- This is a cooldown on the Trigger Manager/Queue (§3.1-3.2), not a change
  to how any individual trigger source decides to fire — `PhysicalInvariant()`
  keeps evaluating and logging every cycle either way (Phase 1.2 is
  unaffected); the limiter only throttles how often those evaluations are
  allowed to *also* consume a FIFO capture.
- This limiter's window should be revisited once Phase 1.3 data shows real
  trigger frequency in the field.



---

## 4. Non-functional requirements (per review — new for this revision)

Per review: FIFO Capture must not degrade the things that already work.
The firmware is dual-core FreeRTOS already, so these are enforceable, not
aspirational:

- FIFO capture **must not block MQTT** publishing.
- FIFO capture **must not block the local display**.
- FIFO capture **must not block normal telemetry** (VRMS/Temp/Frequency
  polling continues on schedule regardless of an in-progress FIFO capture).
- FIFO capture **must not increase normal poll jitter beyond a defined
  budget** (exact number TBD during Phase 1.1 implementation — needs a
  baseline jitter measurement first, then a explicit ceiling, e.g. "+X ms
  p95" — not left undefined).

These constraints are why §3.2's separate FIFO task (rather than inline
capture in the polling loop) is a requirement, not an implementation
preference.

## 5. Rollback plan / failure modes (per review — new for this revision)

Per review: every new feature needs an explicit failure mode, not just a
happy path. Example for Phase 1.1 (Peak register):

```
If Peak register read fails
    │
    ▼
Disable Peak telemetry for this cycle
    │
    ▼
Continue VRMS/Temp/Frequency as normal
    │
    ▼
No reboot, no fault-latch entry triggered by this failure alone
```

The general principle: **a new Phase 1 feature failing should degrade
gracefully to "Phase 1 feature absent this cycle," never to "existing
working behavior stops."** This applies to all of 1.1-1.4 and to the
Trigger Manager/FIFO Task in §3 — each needs its own explicit version of
this diagram before it ships, not just Peak's.

## 6. What's explicitly out of scope for Phase 1

- Modbus transaction consolidation (→ v17, per correction above)
- Per-axis state machine replacing single-sample hold (→ revisit after
  Phase 1.3 data, not before)
- Wiring any invariant check into alarms/fault-latch (→ Phase 1.4 only, and
  only if justified by Phase 1.3 data)
- Continuous/always-on FIFO streaming (the auto-trigger design above is
  explicitly chosen instead, to keep normal-operation bandwidth low)

---

## 7. Why this staged approach is the right call

The review's core point generalizes well: **RFC-0004/0005/0006/0007 are
strong evidence that something is worth investigating — they are not yet
proof of exactly which fix is correct or necessary.** Treating "evidence of
a gap" as equivalent to "proof of the right fix" is the same category error
this whole project has worked hard to avoid at every other stage (Physical
Invariant Violation ≠ DSP glitch confirmed cause, CRC match ≠ autonomous
streaming proof, etc.). Phase 1 should hold itself to the same standard.

---

## 8. Design freeze and next steps (per review)

This document is declared a **Design Freeze Candidate for Phase 1** —
architecture-level changes stop here. No further structural changes
(trigger sources, queue design, state machine shape, priority tiers) should
be made without new field evidence prompting them, consistent with §7's
principle.

The next step is to break this into small, independently reviewable and
compilable implementation documents, so implementation work (including any
AI-assisted coding) proceeds as small patches rather than one large change:

- **DESIGN-0001: Trigger Manager** (§3.1 — priority tiers, drop policy)
- **DESIGN-0002: FIFO Task** (§3.2, §3.4, §3.5 — queue, state machine, rate limiter, capture policy)
- **DESIGN-0003: `PhysicalInvariant()`** (Phase 1.2 — diagnostic-only checks)
- **DESIGN-0004: Peak Telemetry** (Phase 1.1 — VX/VY/VZ read, zero decision impact)

Each should be small enough to compile, review, and regression-test on its
own before the next is started.
