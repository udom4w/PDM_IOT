# Phase 0 Readiness Review: Telemetry Snapshot Implementation

Status: Review Complete
Author: (engineering team)
Date: 2026-07-06
Reviewed against: ADR-0001, ADR-0002, ADR-0003 (frozen),
`IMPLEMENTATION_PLAN_telemetry_snapshot.md` (frozen),
`RELEASE_PLAN_v17.0.md` (frozen)
Scope: read-only verification of current codebase readiness for
Implementation Plan Phase 1. No code modified.

---

## Executive Finding

Before answering the seven questions individually, one fact dominates
this review and changes the shape of the recommendation:

**The firmware version this entire architecture was analyzed and
planned against — `WTVB02_ESP32S3_V16_CM_fault_latch_v3_hardened_
patched_v16_5.ino` (v16.5, the MQTT single-owner refactor) — has not
itself completed production validation.**

Its own regression checklist
(`../design_reviews/mqtt_single_owner_regression_checklist_v16_5.md`),
already written and present in the repository, states plainly:

- Design Review: ✅ CLOSED
- Implementation: ✅ COMPLETE
- **Regression / Soak: ⏳ NOT STARTED**
- **Production Readiness: ❌ NOT YET APPROVED**
- Acceptance Gate (§10, 11 rows: build verification, MQTT connectivity,
  functional regression, single-owner verification, queue behavior,
  fault injection, 24–48h soak, performance comparison, canary,
  rollback-verified) — **every row unchecked.**

This means "Architecture is frozen" and "Implementation Plan is frozen"
are true at the design level, but the foundation those plans assume —
a stable, production-proven v16.5 — is not yet established. This is the
central hidden blocker this review exists to surface.

---

## 1. Does the current codebase support the planned implementation without major restructuring?

**Structurally, yes.** The codebase already contains a proven,
working pattern that Phases 2–5 can mirror directly:

- A ring-buffer + mutex-guarded push/replay pattern already exists and
  works (`TelemetrySlot_t` / `g_telemBuf` / `pushTelemBuf()` /
  `replayTelemBuf()`) — Phase 2/3's new Business Decision buffer is
  structurally the same shape, just smaller and separately scoped.
- The three existing JSON payload builders (`/sensor`, `/status`,
  `/vibration`) are cleanly separated blocks inside one function, so
  Phase 1's additive-field change is localized and low-risk.
- An MQTT outbound queue pattern (`queueMqttOutboundTrend` /
  `enqueueMqttOutbound()`) already exists for routing a non-owner task's
  publish need through the single owner task — this is directly
  relevant precedent for how Phase 3's new decision-replay publish
  should be wired.

**One constraint the Implementation Plan did not explicitly state must
now be treated as binding:** the v16.5 refactor establishes a
**single-owner invariant** — only `taskNetwork()` (Network4G) may call
`mqttClient.*`/`gsmClient.*`. Phase 3 adds a new publish call site (the
decision-replay function). That call site **must** live inside
`taskNetwork()`, or be invoked exclusively from it, exactly like
`publishTelemetry()` and `replayTelemBuf()` already do — not a new
independent caller. This is not "major restructuring," but it is a firm
architectural constraint the Implementation Plan's Phase 3 description
did not name explicitly and should.

## 2. Hidden blockers

1. **v16.5's own production-readiness gate is open** (Executive
   Finding above) — the single biggest hidden blocker. Phase 1 as
   currently planned would begin building on a candidate, not a proven
   baseline.
2. **CI enforcement for the single-owner invariant does not exist yet**
   (checklist §1.4, §5.3 — "Item 10" is explicitly not implemented; the
   checklist itself recommends implementing it "before promoting past
   canary"). Without it, Phase 3 adding a new `mqttClient.publish()`
   call site relies entirely on manual review (`git grep`) to catch an
   accidental invariant violation.
3. **This system has a documented history of a real crash class**
   directly relevant to Phase 3: the checklist's §7 fault-injection
   test describes a priority-injection race between Network4G's TLS
   reconnect path and a `/trend`-flush publish, previously reproducible
   and landing in `mbedtls_ssl_*` frames, which the v16.5 refactor was
   specifically built to fix. Phase 3 introduces another concurrent
   publish path (decision replay) into the same task — the
   Implementation Plan's Phase 3 test plan does not currently call for
   re-running this specific fault-injection scenario against the new
   path, and it should.
4. **Baseline RAM/flash figures were never actually measured.** The
   checklist itself flags (line 65) that a fresh clean build of the true
   v16.4 baseline was never measured in its authoring session — the
   only numbers on record are incremental, informational build sizes
   from the refactor's own implementation session. ADR-0002's and the
   Implementation Plan's RAM estimates for the new Business Decision
   buffer are therefore being compared against an unconfirmed baseline.

## 3. Technical debt that should be resolved BEFORE Phase 1

1. **Close the v16.5 acceptance gate** (§10 of the existing regression
   checklist) — this is the single highest-priority item. It is not new
   technical debt created by this review; it is pre-existing,
   already-documented, already-scheduled work that simply has not been
   executed yet.
2. **Implement the Item 10 CI/pre-build enforcement script** for the
   single-owner invariant, ahead of Phase 3 specifically, since Phase 3
   adds exactly the kind of new call site this tooling exists to guard.
3. **Measure a real, fresh clean-build baseline** (flash + RAM,
   `adf315b`/v16.4 and the actual v16.5 candidate) rather than relying on
   the incremental, informational figures currently on record, so
   Phase 2's RAM-headroom claims have a trustworthy baseline to be
   measured against.
4. The stale "dormant queue" comment issue is **not** new debt requiring
   resolution before Phase 1 — it is already correctly scheduled as
   Phase 6 / Milestone M0 in the frozen Implementation Plan and Release
   Plan. No action needed here beyond what's already planned.
5. `KNOWN_ISSUE_serial_race.md` (interleaved boot-log lines across
   cores) is open but explicitly documented as affecting log readability
   only, not sensor/MQTT/state-machine/analytics correctness. It does
   not block Phase 1 and is noted here only for completeness — resolving
   it is not a prerequisite.

## 4. Verify the planned commit sequence is feasible

Partially — with one logistics gap the Implementation Plan did not
address: **which file is Phase 1's actual starting point?**

`git ls-files` confirms only the v16.4 `.ino` is tracked in this
repository; the v16.5 `.ino` this entire architecture was analyzed
against is currently **untracked** (working-tree only). The
Implementation Plan's phases assume a single, stable, committed
starting file to branch from. Before "Phase 1: commit 1" can be created,
the following must be resolved and is currently undocumented:

- Is v16.5 to be committed and merged as the new tracked baseline
  first (presumably gated on closing its own acceptance checklist), and
  only then does Phase 1 begin on top of it?
- Or is Phase 1 meant to begin from the currently-tracked v16.4, with
  v16.5's refactor and the new Telemetry Snapshot phases converging
  later?

Neither the Implementation Plan nor the Release Plan states which.
Once that is resolved, the six-phase, one-commit-per-phase sequence
itself remains feasible: each phase's target files/functions exist,
are identifiable, and are additive-only, consistent with clean,
independent revertibility.

## 5. Undocumented assumptions

1. That v16.5 is a stable, production-validated baseline. It is not,
   per its own not-yet-closed regression checklist.
2. That the git branch/commit state Phase 1 begins from is already
   settled — it is not (v16.4 tracked, v16.5 untracked, no stated
   reconciliation plan).
3. That the RAM/flash headroom implicitly assumed available for the new
   Business Decision buffer (ADR-0002 §7, Implementation Plan Phase 2)
   is based on measured, confirmed figures — the only figures on record
   are informational/incremental, not a confirmed clean-build baseline.
4. That "frozen architecture/implementation/release plan" implies a
   "frozen, production-ready firmware baseline" — these are different
   kinds of "frozen" (design-approved vs. production-validated), and the
   planning documents so far have not distinguished them explicitly.
5. That Phase 3's new publish call site will automatically inherit the
   safety properties of the existing single-owner refactor without its
   own dedicated fault-injection verification against the specific
   crash class that refactor was built to prevent.

## 6. Implementation risk per phase, reassessed against the current (not-yet-validated) codebase

| Phase | Implementation Plan's estimate | Reassessed risk | Why it changes |
|---|---|---|---|
| 6 | Trivial | Trivial (unchanged) | Comment-only, no interaction with v16.5's open items |
| 1 | Low | Low (unchanged) | Additive fields, minimal interaction with single-owner refactor |
| 2 | Medium | **Medium, trending High** | Adds new static state on top of a codebase whose own stack/heap soak validation (checklist §8) is not yet complete — this compounds rather than simply adds risk |
| 3 | Medium-High | **High** | New `mqttClient.publish()` call site, into a task governed by an invariant with no CI enforcement yet, in a system with documented history of a concurrency-related crash in exactly this area |
| 4 | Low-Medium | Low-Medium (unchanged) | Touches only the long-proven `/sensor` replay path, unrelated to the open items |
| 5 | Low | Low (unchanged) | Counter/status field, well-precedented, unrelated to the open items |

## 7. Recommendation: Go / No-Go

# **NO-GO — conditional, not architectural.**

Do not begin Implementation Plan Phase 1 yet. This is not a finding
against ADR-0001/0002/0003 or the Implementation/Release Plans — they
remain sound as designs. The block is foundational: the codebase this
work would be built on top of is itself an unvalidated release
candidate, by its own already-written, already-scheduled regression
checklist.

**Path to GO:**

1. Execute and close `mqtt_single_owner_regression_checklist_v16_5.md`
   §10 acceptance gate in full — including the 24–48h soak, the
   priority-injection fault-injection repro, and canary deployment —
   and promote v16.5 to a tracked, committed fleet baseline.
2. Resolve which committed file Phase 1 branches from (§4 above) and
   record it, so the commit sequence has an unambiguous starting point.
3. Measure and record real clean-build flash/RAM figures for that
   confirmed baseline, replacing the current informational-only numbers.
4. Re-run this Phase 0 review against that confirmed baseline before
   starting Phase 1 — specifically re-checking whether Phase 3's plan
   should be amended to explicitly include a re-run of the
   priority-injection fault-injection test against the new decision-
   replay publish path.

Once those four items close, this review's answer to Question 1 remains
"yes, no major restructuring needed," and Phase 1 can begin.
