# v16.5 Baseline Closure Plan

Status: Planning
Author: (engineering team)
Date: 2026-07-06
Source checklist (unmodified): `../design_reviews/mqtt_single_owner_regression_checklist_v16_5.md`
Candidate: `ed55cad` — Baseline: `adf315b` (v16.4)
Triggered by: `PHASE0_READINESS_REVIEW.md` (No-Go pending this closure)

## Scope Statement

This plan promotes v16.5 **Release Candidate → Development Baseline**
only — the bar needed for Implementation Plan Phase 1 to safely begin
building on top of it. It is **not** the full Fleet Baseline promotion.
Two items in the existing checklist remain explicitly deferred beyond
this plan, and must not be read as satisfied by it:

- **Canary field deployment** (checklist §10, row "1 field unit running
  `ed55cad`") — a field/fleet-rollout concern, out of scope for a
  bench-validated Development Baseline.
- **Item 10 CI enforcement script** for the single-owner invariant —
  the checklist itself only requires this "before promoting past
  canary," so it is not gating for Development Baseline, but it is
  flagged as recommended in parallel, since Phase 3 adds a new
  `mqttClient.publish()` call site that this tooling would guard.

Everything else in the existing checklist (§2–§9) is in scope and must
be executed on a bench unit.

---

## 1. Ordered Execution Steps

| # | Step | Checklist ref | Depends on |
|---|---|---|---|
| 1 | Environment prep: confirm `adf315b` and `ed55cad` both buildable from a clean `.arduino` cache state on the target FQBN | §2 | — |
| 2 | Build Verification: clean build both, record flash/RAM for both (not just candidate) | §2 | 1 |
| 3 | Single-Owner Static Audit: `git grep` sweep, confirm zero unexpected `mqttClient.*`/`gsmClient.*` call sites | §5.1 | 2 |
| 4 | MQTT Connectivity: cold boot, forced GPRS drop, forced broker disconnect, backoff timing, cache-consistency window | §3 | 2 |
| 5 | Functional Regression: byte-identical payload check on all 5 topics + TelemBuf replay + fault latch + maintenance event, vs. `adf315b` | §4 | 4 |
| 6 | Single-Owner Behavioral Audit: runtime trace confirms Analytics/Display/loop never call `mqttClient`/`gsmClient` directly | §5.2 | 3, 5 |
| 7 | Queue Behavior: overflow, depth-under-reconnect, extended-outage drain, non-blocking enqueue, rate-limited drain, log-accuracy checks | §6 | 5 |
| 8 | Fault Injection: priority-injection repro (≥100 cycles, baseline vs. candidate), broker-kill mid-publish, GPRS-drop mid-publish, oversized-payload rejection, watchdog cadence | §7 | 6, 7 |
| 9 | Long-Run Soak: 24–48h continuous, periodic simulated 4G drops + irregular motor cycling + normal `/trend` cadence | §8 | 8 |
| 10 | Performance Comparison: clean-build size delta, loop/publish timing, enqueue-to-publish latency, queue occupancy trend | §9 | 9 |
| 11 | Bench Rollback Verification: revert `ed55cad` → `adf315b` on the same bench unit, confirm clean restoration | (Acceptance Gate row) | 9 |
| 12 | Sign-off: compile evidence, complete the existing checklist (all §2–§9 rows), record reviewer/date/result | §10 (Development Baseline subset) | 1–11 |

Steps 2–3 and 4 can run in parallel with each other once Step 2 (build)
completes for both configurations; Steps 6–7 can also overlap. Step 9
(soak) is the long pole — start it as early as its dependencies (Steps
1–8) allow, and use its unattended runtime window to draft the Step 10
performance-comparison report from data already collected in Steps 2–8.

---

## 2. Required Evidence (per step)

| Step | Evidence |
|---|---|
| 1–2 | Two clean-build logs (baseline, candidate); recorded flash/RAM bytes for both |
| 3 | `git grep -n "mqttClient\.\|gsmClient\."` output, annotated with enclosing function per match |
| 4 | Timestamped log of each connectivity transition and cache-consistency observation |
| 5 | Per-topic payload diff report (all 5 topics) against baseline, for identical synthetic sensor input |
| 6 | Runtime trace/log excerpt showing zero direct `mqttClient`/`gsmClient` calls outside `taskNetwork()` |
| 7 | Overflow-counter values, queue-occupancy time series, drain-rate observations under forced backpressure and extended outage |
| 8 | Priority-injection repro result on both baseline (crash reproduced) and candidate (crash absent), plus logs for the other four fault-injection scenarios |
| 9 | Continuous soak log; `g_rebootCount`/`g_resetReasonStr` before/after; stack high-water-mark and heap-watermark trend for `taskNetwork`/`taskAnalytics` |
| 10 | Build-size delta table; loop/publish/enqueue timing measurements; max/avg queue occupancy |
| 11 | Bench rollback log: revert executed, boot confirmed, baseline behavior restored |
| 12 | Completed checklist (all boxes in §2–§9), reviewer name, date, PASS/FAIL/CONDITIONAL result |

---

## 3. Pass / Fail Criteria

| Step | Pass condition | Fail condition |
|---|---|---|
| 2 | Zero warnings/errors on both clean builds; sizes recorded for both | Any build failure or new warning vs. baseline |
| 3 | Every match resolves to `taskNetwork()`, `setupTLS()`, or `publishTelemetry()` (single caller = `taskNetwork`); zero matches elsewhere | Any `mqttClient`/`gsmClient` call outside the approved set |
| 4 | All transitions match baseline timing within normal variance; cache reflects true state within one `taskNetwork()` iteration | Any transition diverges from baseline behavior, or cache lags beyond one iteration |
| 5 | All 5 topics + TelemBuf + fault latch + maintenance event byte-identical to baseline for fixed input | Any field diverges unexpectedly (unexplained by this refactor's intended scope) |
| 6 | Zero direct calls outside `taskNetwork()` observed at runtime | Any direct call observed |
| 7 | Drop-newest (not drop-oldest) on overflow; queue depth stays ≤6; drains fully after extended outage; enqueue never blocks | Oldest-drop instead of newest-drop; unbounded queue growth; blocking enqueue observed |
| 8 | Crash reproduced on baseline, **not** reproduced on candidate (≥100 cycles); other four scenarios show clean failure handling, no panic | Crash still reproducible on candidate, or any panic/unhandled fault in the other scenarios |
| 9 | Zero unexpected `PANIC`/`TASK_WDT`/`INT_WDT` resets over 24–48h; stack/heap watermarks flat, no closer approach to exhaustion than baseline | Any unexpected reset, or a trending (non-flat) heap/stack watermark |
| 10 | Size delta small and explainable; timing deltas negligible (µs, not ms); queue metrics within configured bound (≤6) throughout | Unexplained size/timing regression, or queue exceeding configured depth |
| 11 | Bench unit boots and operates normally on `adf315b` immediately after revert | Revert fails, or post-revert behavior diverges from known-good v16.4 |
| 12 | All of steps 2–11 pass; zero open Sev1/Sev2 findings | Any step fails, or any open Sev1/Sev2 finding |

---

## 4. Estimated Duration

| Step | Effort | Elapsed |
|---|---|---|
| 1–2 | 0.5 day | 0.5 day |
| 3 | 0.25 day | 0.25 day |
| 4 | 0.5 day | 0.5 day |
| 5 | 1 day | 1 day |
| 6 | 0.5 day | 0.5 day |
| 7 | 1 day | 1 day |
| 8 | 1.5 days | 1.5 days |
| 9 | 0.5 day active (setup/monitor/teardown) | 2 days elapsed (unattended) |
| 10 | 0.5 day (can overlap with Step 9's elapsed window) | — |
| 11 | 0.5 day | 0.5 day |
| 12 | 0.5 day | 0.5 day |
| **Total** | **~7 person-days effort** | **~7–8 working days elapsed**, since Step 9's 24–48h soak runs mostly unattended and Step 10 can be drafted during it |

---

## 5. Required Artifacts (Consolidated Deliverables)

1. Two clean-build logs with recorded flash/RAM (baseline + candidate).
2. Single-owner static audit output (annotated `git grep` results).
3. MQTT connectivity transition log.
4. Functional regression payload-diff report (5 topics + buffer + latch + event).
5. Single-owner behavioral trace log.
6. Queue behavior test log (overflow, depth, drain, latency).
7. Fault-injection report (priority-injection repro + 4 other scenarios).
8. Soak test log + reboot/reset counters + stack/heap watermark trend.
9. Performance comparison report (size, timing, queue metrics).
10. Bench rollback verification log.
11. **Completed regression checklist** (`mqtt_single_owner_regression_checklist_v16_5.md`, all §2–§9 rows checked) — produced by executing this plan, not by this planning document itself.
12. Signed closure record (reviewer, date, result, explicit note of what remains deferred to Fleet Baseline).

---

## 6. Go / No-Go Criteria (Development Baseline Promotion)

**GO** requires all of:
- Steps 2–11 all pass per §3 above.
- Zero open Sev1/Sev2 findings; any Sev3 explicitly triaged and accepted
  in writing, not silently ignored.
- Priority-injection fault reproduced on baseline and absent on
  candidate — this is the single most load-bearing result, since it is
  the specific defect class the refactor exists to fix.
- Bench rollback to `adf315b` confirmed clean.

**NO-GO** if any of:
- Any Sev1/Sev2 finding remains open.
- The priority-injection fault is still reproducible on the candidate.
- Soak test shows any unexpected reset or a trending (non-flat)
  memory watermark.
- Single-owner invariant is violated anywhere (static or behavioral).

On NO-GO: remediate the specific failing area, re-run only the affected
step(s) and any steps depending on them (per the dependency column in
§1) — not the full sequence from Step 1, unless the fix touches build
configuration or an earlier step's assumptions.

---

## 7. Exit Criteria

This closure plan is complete when:

1. All 12 steps have run and produced their required evidence (§2/§5).
2. All Pass/Fail criteria (§3) are met with a recorded GO decision (§6).
3. `ed55cad` (or its equivalent final state) is **committed and tagged**
   in git as the Development Baseline — resolving the untracked-file gap
   noted in `PHASE0_READINESS_REVIEW.md` §4.
4. The existing regression checklist document is fully completed and
   archived alongside this closure record (checklist itself is filled
   in during execution — not modified as part of this planning task).
5. A written note accompanies the closure record stating explicitly
   that Canary deployment and the Item 10 CI enforcement script remain
   open, deferred to the separate, later Fleet Baseline promotion gate.
6. `PHASE0_READINESS_REVIEW.md`'s No-Go condition is re-evaluated against
   this now-closed baseline before Implementation Plan Phase 1 begins.
