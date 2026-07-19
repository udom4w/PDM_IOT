> **Document Status**
>
> **Design Review:** 🔴 Draft — pending approval, **contains one Critical open question that blocks approval** (§4)
>
> **Implementation:** ⛔ NOT STARTED — do not implement until this document is approved
>
> **Governing plan:** `Phase1_Implementation_Plan.md` §3.2 (queue), §3.3 (metadata), §3.4 (state machine), §3.5 (rate limiter), §4 (NFRs), §5 (failure/rollback) (authoritative roadmap)
>
> **Firmware baseline:** `WTVB02_ESP32S3_V16_CM_fault_latch_v3_hardened_patched_v16_5.ino`, HEAD `4f63f92`
>
> **Depends on:** `DESIGN-0001` (Trigger Manager) — hard dependency, this is its sole consumer.
>
> **Source of Truth:** `Phase1_Implementation_Plan.md` is authoritative for scope. This document identifies one architectural question the plan does not resolve (§4) and does not decide it unilaterally, per `CLAUDE_RULES.md` §2/§16 (architecture is frozen; stop and ask rather than invent).

# DESIGN-0002: FIFO Task

## 1. Source and Scope

Per `Phase1_Implementation_Plan.md` §3.2–§3.5: a dedicated FreeRTOS task that dequeues capture requests from `DESIGN-0001`'s Trigger Manager, executes a 1024-sample-per-axis FIFO capture (per RFC-0006/0007's reverse-engineered 3-transaction protocol: progress frames + 6149-byte full dump), attaches rich metadata (§3.3), and saves/uploads the result — all without blocking MQTT, display, or normal telemetry polling (§4 NFRs).

In scope: capture execution, its state machine, its rate limiting (jointly with `DESIGN-0001`, see `DESIGN-0001` §4.5), and its failure handling. Out of scope: trigger arbitration (`DESIGN-0001`), the checks that produce a `PhysicalInvariant()` trigger (`DESIGN-0003`).

## 2. Facts From Current Source (v16.5) — [FACTS]

**2.1 There is exactly one `ModbusMaster` instance, and exactly one task calls it.** `ModbusMaster modbus;` is declared once (line ~1054). A full-file search of every `modbus.readHoldingRegisters(...)`, `modbus.writeSingleRegister(...)`, and `modbus.begin(...)` call site shows **all of them occur inside `taskModbusRead()`** (Core 0, `PRIORITY_MODBUS = 5`, the highest-priority task in the system) or in helper functions called synchronously from it (`reconfigSensorAfterRestart()`, sensor unlock/restart helpers). **No mutex guards `modbus`** — because until now, nothing needed one; access has always been single-task by construction, not by an enforced rule.

**2.2 This is the same class of latent risk the v16.5 MQTT refactor was built to eliminate — on a different shared resource.** The MQTT single-owner design doc (`mqtt_single_owner_design_v16_5.md`) exists because `mqttClient`/`gsmClient` were reachable, unsynchronized, from four task contexts under a preemptive scheduler — and that gap produced a real, reproduced crash. `modbus` today has the *same* structural shape (single unsynchronized global client object) but is currently safe only because it happens to have a single caller. **`Phase1_Implementation_Plan.md`'s own diagram labels the FIFO Task "a separate FreeRTOS task"** issuing "a different Modbus transaction shape" — if implemented literally, this introduces a second, concurrent caller of `modbus.*`, recreating the exact precondition (shared unsynchronized client, multiple task contexts) that produced the MQTT crash, this time reachable from the sole Core-0 time-critical Priority-5 task's shared bus. The plan does not address this. This is the most significant gap this review identified in the entire Phase 1 plan.

## 3. Design Goal (proposed, pending §4)

Modbus/RS485 bus access remains single-owner: `taskModbusRead()` is the only code path that ever calls `modbus.*`. FIFO capture is executed by `taskModbusRead()` itself, interleaved with its existing 4Hz polling, when it dequeues a request from `DESIGN-0001`'s queue — not by an independent task issuing concurrent Modbus transactions.

## 4. Open Question #1 — Critical, Blocks Approval: Who Owns the Modbus Bus During Capture?

Two options, presented per `DESIGN_PRINCIPLES.md` §7 (every option states invariants preserved, false positive/negative risk, and trade-offs) — **neither is adopted by this document; both require explicit project-owner sign-off**, since this is a genuine architecture-level decision the governing plan left unresolved, and `CLAUDE_RULES.md` §16 requires stopping and asking rather than inventing a new architecture pattern silently.

| Option | Description | Preserves single-owner invariant? | Jitter/priority-inversion risk | Effort |
|---|---|---|---|---|
| **A — Capture inside `taskModbusRead()` (recommended)** | FIFO Task becomes a state machine *within* `taskModbusRead()`'s own loop, not a separate task. On dequeuing a request, `taskModbusRead()` temporarily suspends normal polling, runs the capture sequence itself (still the sole `modbus.*` caller), then resumes. | Yes — extends the existing, already-safe single-caller pattern; no new mutex needed. | Low — same task sequences both duties deterministically, so the NFR "capture must not increase normal poll jitter beyond a defined budget" (plan §4) is directly enforceable by construction, not by hoping a mutex is fair. | Moderate — `taskModbusRead()`'s control flow gains a second mode; needs its own internal timing budget so a multi-second capture burst doesn't starve the 250ms poll cadence indefinitely. |
| **B — True separate task + new mutex around `modbus`** | A genuinely independent `FIFOTask`, as the plan's diagram literally shows, guarded by a new mutex (e.g. `mutexModbusBus`) around every `modbus.*` call in both tasks. | Only if the mutex is added everywhere, including retrofitting all 8 existing `taskModbusRead()` transactions — a change to already-proven, working code, which `CLAUDE_RULES.md` §5 explicitly discourages ("Avoid touching unrelated code") unless required. | **Higher** — introduces a priority-inversion path: `taskModbusRead()` (Priority 5, time-critical) could block on a mutex held by a lower-priority `FIFOTask`, which is a strictly worse blast radius than the original MQTT bug (that race only ever reached Core 1's non-time-critical tasks; this would reach the system's sole Priority-5 time-critical task). | Higher — new synchronization primitive, new failure mode (mutex timeout handling), retrofits proven code. |

**Recommendation: Option A.** It extends this project's own established idiom (single owner + queue, already proven for MQTT) rather than introducing a new synchronization primitive around a resource that has never needed one, and it makes the plan's own NFR (§4, bounded poll jitter) directly provable rather than dependent on scheduler fairness under contention. **This is a recommendation, not a decision — approval is required before either option is implemented.**

## 5. Detailed Design (drafted assuming Option A, pending §4 approval)

**5.1 State machine.** Verbatim from the governing plan §3.4, restated as an interleaved mode inside `taskModbusRead()`'s own loop rather than a separate task's lifecycle:

```
IDLE
  │  (request dequeued from DESIGN-0001's queue)
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

Invariant: normal 4Hz polling must not be starved. Capture should only begin at a safe point between poll cycles, and the `CAPTURING` state must budget its own `vTaskDelay`/watchdog resets the same way the existing 8-transaction poll sequence already does (existing precedent: `esp_task_wdt_reset()` cadence inside `taskNetwork()`'s loop, verified as a pattern this project already relies on per the MQTT regression checklist §7).

**5.2 Rich metadata (verbatim from the governing plan §3.3, unchanged):** timestamp, machine/sensor ID, motor state, temperature, VRMS/Frequency/Peak (once `DESIGN-0004` ships), alarm/trigger reason, firmware version, SR register, gain/scale setting, capture source (auto vs. manual), firmware git commit, RFC version, reason code.

**5.3 Rate limiter.** Per the governing plan §3.5, this cooldown ("max 1 capture per 5 minutes, subject to field validation") is enforced by `DESIGN-0001`'s Trigger Manager/Queue layer (see `DESIGN-0001` §4.5), not duplicated here.

**5.4 Capture-in-progress policy.** Per the governing plan §3.2: queue a new request behind the current one, bounded by `DESIGN-0001`'s priority tiers and the rate limiter. No change from the plan's own text.

**5.5 Storage/upload path — unspecified by the governing plan.** The plan's diagram ends at "Upload for analysis" without defining where 1024×3×2 = 6144 bytes of raw samples plus metadata are held between capture and upload, or what the upload transport is. On a RAM-constrained ESP32-S3 that already budgets multiple static buffers (MQTT outbound queue, `/sensor` ring buffer, JSON serialization buffers), a 6144-byte capture buffer is a non-trivial addition that needs explicit sizing and placement (static `.bss` vs. transient allocation — per `CLAUDE_CONTEXT.md`'s Memory Philosophy, "avoid dynamic allocation inside periodic execution," so static is the expected answer, but this needs to be said explicitly, not assumed). See Open Question 2.

## 6. Non-Functional Requirements (verbatim from governing plan §4)

- Must not block MQTT publishing.
- Must not block the local display.
- Must not block normal telemetry (VRMS/Temp/Frequency polling continues on schedule regardless of an in-progress capture).
- Must not increase normal poll jitter beyond a defined budget — **the plan itself states this number is "TBD during Phase 1.1 implementation — needs a baseline jitter measurement first."** `DESIGN-0004` (Phase 1.1) has not yet produced that baseline. **This document cannot set a concrete jitter ceiling until `DESIGN-0004` ships and a baseline is measured** — flagged as a sequencing dependency, not a defect in this document. See §7 Recommended Order.

## 7. Failure Modes

Per the governing plan §5 pattern, applied to FIFO capture specifically: capture fails (timeout or CRC mismatch — both already observed on real hardware per RFC-0006/0007's field testing) → `FAILED` state → bounded retry (count/backoff still an open plan-level parameter, "RETRY? decision... open implementation parameter," verbatim from the plan) → `IDLE`. Never blocks or stalls normal polling. No reboot, no fault-latch entry triggered by a capture failure alone.

## 8. Non-Goals

- No continuous/always-on FIFO streaming (explicit non-goal in the governing plan — the auto-trigger design is deliberately chosen instead to keep normal-operation bandwidth low).
- No wiring into fault-latch/alarm/decision logic.
- No change to the existing 8-transaction normal-poll sequence's *shape* (transaction consolidation is explicitly deferred to v17).

## 9. Open Questions (require explicit approval before implementation)

1. **§4 — Modbus bus ownership during capture (Critical, blocks approval).** Option A vs. B.
2. **Capture storage/upload path and buffer sizing** (§5.5) — not specified by the governing plan; needs an explicit decision, analogous to how `ADR-0001` flagged replay-buffer capacity sizing as "a product/commercial requirement, not an engineering default."
3. **Jitter budget** (§6) — cannot be finalized until `DESIGN-0004`'s baseline measurement exists. Sequencing implication, not an open design gap per se.
4. **Retry count/backoff** (§7) — explicitly left open by the governing plan itself; still open here.

## 10. Recommended Sequencing Note

Because §6's jitter budget depends on a `DESIGN-0004` baseline, and `DESIGN-0001`'s most valuable trigger source (`DESIGN-0003`) has a hard dependency on `DESIGN-0004`'s Peak values, this document should not be implemented before `DESIGN-0004` (and ideally `DESIGN-0003`) have shipped and produced field data — consistent with the governing plan's own staged-rollout philosophy (§2, Phase 1.1 → 1.2 → 1.3 before anything is "wired in"), even though the plan lists the Trigger Manager/FIFO Task as a nominally parallel "New Addition" (§3) rather than gating it explicitly on 1.1–1.3. This is a recommendation for the overall implementation order, addressed at the roadmap level, not a change to this document's own scope.

## 11. Test Plan (for when implementation is approved)

- Once §4 is resolved: if Option A, confirm via static audit that `modbus.*` calls remain confined to `taskModbusRead()` and its existing helpers — zero new call sites elsewhere, mirroring the MQTT single-owner `git grep` audit pattern exactly.
- Baseline jitter measurement on normal 4Hz polling *before* this feature exists (needed regardless of option chosen, to have something to compare against).
- Forced FIFO capture during active MQTT publish/display update/normal telemetry cycle — confirm zero measurable disruption to those paths (NFR §6).
- Forced timeout and forced CRC-mismatch capture (bench-injectable, per RFC-0006/0007's own documented real-hardware failure modes) — confirm clean `FAILED → RETRY? → IDLE` transition, no stuck state, no watchdog trip.
- Confirm capture-in-progress + new request arriving → queued, not dropped, not merged (per plan §3.2's locked policy).

## 12. Summary for Sign-Off

| Item | Status |
|---|---|
| Scope confirmed against governing plan | ✅ |
| Modbus single-owner risk identified in current source | ✅ **Critical finding — blocks approval until §4 resolved** |
| Recommended resolution (Option A) stated with trade-offs | ✅ Recommendation only, not adopted |
| Storage/upload path sizing | ⚪ Open — Open Question 2 |
| Jitter budget | ⚪ Blocked on DESIGN-0004 baseline — Open Question 3 |
| Retry policy | ⚪ Open per governing plan itself — Open Question 4 |
| **Patch status** | **Draft — Critical Open Question 1 must be resolved by explicit project-owner decision before this document can be approved; three further open questions besides** |
