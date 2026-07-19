> **Document Status**
>
> **Design Review:** 🟡 Draft — pending approval
>
> **Implementation:** ⛔ NOT STARTED — do not implement until this document is approved
>
> **Governing plan:** `Phase1_Implementation_Plan.md` §3.1, §3.2 (dedup), §3.5 (rate limiter) (authoritative roadmap)
>
> **Firmware baseline:** `WTVB02_ESP32S3_V16_CM_fault_latch_v3_hardened_patched_v16_5.ino`, HEAD `4f63f92`
>
> **Depends on:** None to exist as code, but its value is incremental — it is only as useful as the trigger sources wired into it. `DESIGN-0002` (FIFO Task) is its sole consumer. `DESIGN-0003` (PhysicalInvariant) is one of five trigger sources, optional at first (§4.1).
>
> **Source of Truth:** `Phase1_Implementation_Plan.md` is authoritative for scope.

# DESIGN-0001: Trigger Manager

## 1. Source and Scope

Per `Phase1_Implementation_Plan.md` §3.1 ("Trigger Manager, not a single trigger"):

> `PhysicalInvariant()` should not be the only thing allowed to ask for a FIFO capture... FIFO capture is expensive, so it deserves a proper arbitration point with multiple legitimate callers.

Five trigger sources, two priority tiers:

```
Critical (never dropped)          Normal (droppable under load)
├── Manual request                ├── PhysicalInvariant() violation
├── Sensor reset detected         └── Temperature alarm
└── Communication error
```

In scope: the arbitration point itself — accepting submissions, applying priority tiers and dedup, and producing a bounded request queue. **Out of scope: executing a FIFO capture** (that is `DESIGN-0002`'s responsibility entirely — the Trigger Manager never talks to Modbus/the sensor).

## 2. Facts From Current Source (v16.5) — [FACTS]

**2.1 Trigger sources are split across both cores — the plan's diagram doesn't show this.** Tracing each of the five sources to where it would naturally originate in the current codebase:

| Source | Priority tier | Natural origin (current code) | Core |
|---|---|---|---|
| Communication error | Critical | `g_modbusConsecErrors` threshold logic, `taskModbusRead()` | 0 |
| Sensor reset detected | Critical | Existing stuck-detection/auto-recovery path (`needRestart`, `stuckAxis`, all-zero and per-axis stuck counters, `taskModbusRead()` ~line 3426+) | 0 |
| Manual request | Critical | Button handler (`taskButtonHandler`) or a new debug command path | 1 |
| PhysicalInvariant() violation | Normal | `DESIGN-0003`, recommended Core 0 (`taskModbusRead()`) | 0 (recommended) |
| Temperature alarm | Normal | Not yet located in source — needs confirmation (see Open Question 3) | Unconfirmed |

Two of three Critical-tier sources, plus the recommended PhysicalInvariant() location, are Core 0. The Trigger Manager is therefore inherently a **cross-core aggregation point** — a fact the plan's trigger diagram does not surface, since it draws all five sources as siblings without noting they originate on different cores.

**2.2 Governing cross-core rule.** `CLAUDE.md`: *"สื่อสาร Core0→Core1 ใช้ command enum (AnalyticsCommand_t) / atomic — อย่าใช้ shared boolean flag ที่มีหลาย writer"* (Core0→Core1 communication uses a command enum/atomic value — never a shared boolean flag with multiple writers). A FreeRTOS queue submitted via `xQueueSend()` from either core satisfies this rule directly — it is the same pattern already proven cross-task in this file for `queueMqttOutboundTrend`/`MqttOutboundMsg_t` (Analytics → Network4G, same-core) and `queueSensorData`/`queueButtonEvent` (cross-task generally). `xQueueSend()` is itself cross-core-safe on ESP32 FreeRTOS as long as it is not called from a true ISR context (none of the five trigger sources are ISR-based), so no new mutex is required — the queue itself is the synchronization primitive, consistent with this project's established idiom.

## 3. Design Goal

Exactly one Trigger Manager. Every FIFO-capture request, from any of the five sources, on either core, passes through it before reaching `DESIGN-0002`'s FIFO Task. No trigger source calls into FIFO capture logic directly — this mirrors the single-owner discipline already established for MQTT (`mqtt_single_owner_design_v16_5.md`) applied to a different shared resource.

## 4. Detailed Design

**4.1 Submission interface (illustrative, not binding).** A function such as `triggerManagerSubmit(TriggerReason_t reason, TriggerPriority_t tier, ...)`, callable from any task on either core. Exact signature, parameter set, and whether metadata (§DESIGN-0002 §5.2) is captured at submission time or at dequeue time are implementation details for the eventual code-level design, not fixed here.

**4.2 Priority tiers.** Verbatim from the governing plan (§1 diagram above). Critical-tier requests must not be dropped under queue pressure; Normal-tier requests may be dropped with a counter, mirroring the already-proven `g_trendEnqueueDropCount` pattern used for the MQTT outbound queue.

**4.3 Queue design.** Recommend an independent FreeRTOS queue (not shared with any existing queue), holding a small struct: `{reason_code, priority_tier, timestamp, source_core}` at minimum — structurally analogous to `MqttOutboundMsg_t`, which this codebase already uses successfully for a similar "producer submits, single consumer drains" shape. **Overflow policy is an open question the plan does not resolve for this specific queue** (§3.2 of the plan addresses capture-in-progress queueing, which is a different question — see `DESIGN-0002` §5.4 — not what happens when the Trigger Manager's own submission queue is full). See Open Question 1.

**4.4 Deduplication.** Per the plan §3.2: *"don't queue a second capture if one for the same cause is already pending."* This logic must live somewhere that can see all pending submissions — the Trigger Manager is the natural place (reject or coalesce at submission time), rather than `DESIGN-0002`'s FIFO Task (which would need to inspect the whole queue at dequeue time to detect a duplicate already in flight). Recommend: Trigger Manager owns dedup. See Open Question 2 for confirmation.

**4.5 Rate limiter placement.** Per the plan §3.5: *"This is a cooldown on the Trigger Manager/Queue... the limiter only throttles how often those evaluations are allowed to also consume a FIFO capture."* The plan's own wording assigns this responsibility to the Trigger Manager/Queue layer, not to `DESIGN-0002`. This document adopts that: the Trigger Manager enforces the "max 1 capture per 5 minutes (subject to field validation)" cooldown before a request is ever queued for the FIFO Task to consume.

## 5. Cross-Core Discipline

All data crossing from Core 0 sources (Communication error, Sensor reset, PhysicalInvariant) to the Trigger Manager's queue rides inside the queued struct, copied by value via `xQueueSend()` — no shared mutable state, no new mutex, consistent with `CLAUDE.md`'s explicit rule (§2.2). No boolean flag, single-word or otherwise, is proposed anywhere in this design.

## 6. Non-Goals

- Does not talk to Modbus, the sensor, or the FIFO register — that is `DESIGN-0002` exclusively.
- Does not implement `PhysicalInvariant()`'s checks — that is `DESIGN-0003`.
- Does not decide capture retry policy or capture state machine — that is `DESIGN-0002`.
- Does not wire into fault-latch or alarms.

## 7. Failure Modes

- Submission queue full (Normal-tier): drop, increment a counter, log — mirrors `g_trendEnqueueDropCount`.
- Submission queue full (Critical-tier): must not drop. Mechanism TBD — see Open Question 1.
- Trigger Manager not yet initialized (a submission arrives before `setup()` completes queue creation): must no-op safely, never dereference a null queue handle — mirrors the existing `queueMqttOutboundTrend != NULL` guard already used at the MQTT outbound drain site.

## 8. Open Questions (require explicit approval before implementation)

1. **Critical-tier never-drop guarantee.** What mechanism enforces this under queue pressure — reserved slots, a separate Critical-only queue, or a large-enough combined depth that the rate limiter (§4.5) structurally prevents contention? Not decided here; needs an explicit choice with a stated trade-off.
2. **Dedup ownership confirmation** (§4.4) — Trigger Manager vs. FIFO Task.
3. **Temperature alarm source location.** Not yet found in the current source during this review — needs to be located (or confirmed not-yet-implemented) before this trigger source can be wired.
4. **Call-site footprint.** This adds a `triggerManagerSubmit()`-style call to potentially 3–4 existing functions across two cores (`taskModbusRead`, possibly `taskButtonHandler`, `DESIGN-0003`'s check, and wherever temperature alarm logic lives). Per `CLAUDE_RULES.md` §4 ("Avoid touching unrelated code"), this footprint should be enumerated exactly, function by function, before implementation begins — not discovered mid-patch.

## 9. Test Plan (for when implementation is approved)

- **Static:** confirm every `triggerManagerSubmit()` call site is one of the enumerated five sources — mirrors the MQTT single-owner static audit pattern (`git grep`-based).
- **Behavioral:** flood Normal-tier submissions (e.g. force repeated PhysicalInvariant violations) and confirm a concurrently-submitted Critical-tier request (e.g. manual request) is never dropped.
- Confirm dedup: submitting two requests for the same active reason while one is already pending results in exactly one queued entry.
- Confirm the 5-minute rate limiter (once wired) blocks a second capture-worthy submission within the cooldown window, while still logging/counting the underlying evaluation per the plan's explicit "PhysicalInvariant() keeps evaluating and logging every cycle either way" requirement.

## 10. Summary for Sign-Off

| Item | Status |
|---|---|
| Scope confirmed against governing plan | ✅ |
| Cross-core structure of trigger sources identified (not shown in plan's own diagram) | ✅ Flagged |
| Cross-core discipline (queue-only, no shared flag) confirmed compliant with CLAUDE.md | ✅ |
| Critical-tier never-drop mechanism | ⚪ Open — Open Question 1 |
| Dedup ownership | 🟡 Recommended (Trigger Manager), needs confirmation — Open Question 2 |
| Temperature alarm source | ⚪ Not located in current source — Open Question 3 |
| Call-site footprint enumerated | ⚪ Partial — Open Question 4 |
| **Patch status** | **Draft — 4 open questions must be resolved/approved before implementation begins** |
