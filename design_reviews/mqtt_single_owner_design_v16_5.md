> **Document Status**
>
> **Design Review:** ✅ CLOSED (Approved)
>
> **Implementation:** ⏳ NOT STARTED
>
> **Regression / Soak:** ⏳ NOT STARTED
>
> **Production Baseline:** v16.4
>
> **Target Candidate:** v16.5
>
> **Production Readiness:** ❌ NOT YET APPROVED
>
> **Source of Truth:** This document is the approved design specification for the MQTT Single-Owner refactor. All implementation work must follow this document. Design sign-off and Production Readiness sign-off are separate gates.

# Design Document: mqttClient/gsmClient Single-Owner Refactor
**Firmware:** WTVB02_ESP32S3_V16_CM_fault_latch_v3_hardened_patched_v16_4
**Status:** Design review — NOT YET IMPLEMENTED
**Author:** Claude (drafted per engineering review request)
**Target:** v16.5 candidate (v16.4 remains fallback baseline)

---

## 1. Epistemic status (carried over from investigation)

| Claim | Confidence | Evidence |
|---|---|---|
| `mqttClient`/`gsmClient` accessed unsynchronized from 4 task contexts | Confirmed (static analysis) | Full audit table, 38 call sites, zero mutex coverage |
| Scheduler permits preemption mid-call (Analytics/Display priority 3 > Network4G priority 2) | Confirmed (priority table + FreeRTOS preemptive semantics) | `xTaskCreatePinnedToCore` priorities, all pinned Core 1 |
| Crash occurred inside mbedTLS record handling during session teardown | Confirmed | addr2line frames 1–3 (reliable) |
| **This specific crash was caused by this specific race** | **Not instrumented-proven** | Stale stack frames 5+ are consistent with, but do not prove, the exact interleaving |

**Working classification: root cause candidate, ~95–99% confidence.** This document proceeds on that basis — the fix is justified by the confirmed vulnerability + confirmed preemption opportunity, independent of 100%-proving this one incident. Section 8 (testing) includes a step aimed specifically at closing the remaining gap.

---

## 2. Design goal

> **Invariant to establish:** After this refactor, exactly one task (`Network4G`) ever calls `mqttClient.*` or `gsmClient.*`. Every other task interacts with MQTT state only through a queue (outbound data) or a cached, task-safe status flag (connection state). No other task may hold a reference to `mqttClient`/`gsmClient` or call any of their methods, directly or transitively.

This mirrors the existing `mutexTelemBuf` / ring-buffer pattern already used in this codebase for offline telemetry replay — the refactor extends an established idiom rather than introducing a new one.

---

## 2a. Architecture Invariant — PROJECT RULE (not just an implementation detail)

> **`mqttClient` owner = `Network4G` task, exclusively. No other task, present or future, may call:**
> `.connect()` · `.loop()` · `.publish()` · `.connected()` · `.lastError()` · `.returnCode()` · `.begin()` · `.setKeepAlive()`
> **or touch `gsmClient` directly** (`.setCACert()`, `.setCertificate()`, `.setPrivateKey()`, `.applyCredentials()`, `.resetTLS()`, or any other method).

This is deliberately promoted from "how we happened to fix this bug" to a **standing project rule**, because the failure mode isn't tied to today's specific call sites — it's tied to *any* future code that reaches for `mqttClient`/`gsmClient` from a task other than `Network4G`. The most likely reintroduction path: someone adds a new MQTT topic six months from now, copies an existing publish block as a template, and wires it into whichever task produces that data — silently recreating this exact race.

**Enforcement mechanism:** §7.10 CI check (updated below) makes this rule machine-checked, not just documented. A rule that only lives in a markdown file gets forgotten; a rule that fails the build does not.

---

## 3. Call-site disposition (from full audit — all 38 sites)

### 3.1 No change — stays inside Network4G (owner task)
All sites already inside `taskNetwork()` / `setupTLS()` / `publishTelemetry()` (called only from `taskNetwork`) require **no change**. This covers audit rows #1–26, #28–34 (init, connect/reconnect, `.loop()`, `.lastError()`, `.returnCode()`, and the /sensor, /status, /vibration, /event, /decision publishes).

Action: **none.** Confirm during implementation that `publishTelemetry()` is never called from any task other than Network4G (currently true — single call site at line 4187 inside `taskNetwork`).

### 3.2 Must change — direct call → replace with queue enqueue

| Audit # | Task | Function | Line | Current call | Replace with |
|---|---|---|---|---|---|
| 36 | Analytics | `taskAnalytics()` main loop | 6064 | `mqttClient.publish(g_mqttTopicTrend, buf, sz, ...)` | Serialize as today, then `enqueueMqttOutbound(TOPIC_TREND, buf, sz)` — non-blocking, returns immediately |

Only **one** site needs this treatment. This is the direct trigger path identified in the audit (Analytics publish colliding with Network4G's connect/loop/publish).

### 3.3 Must change — direct call → replace with cached status read

| Audit # | Task | Function | Line | Current call | Replace with |
|---|---|---|---|---|---|
| 27 | DisplayUpdate | `drawNetworkScreen()` | 4906 | `mqttClient.connected()` | `getMqttConnectedCached()` |
| 35 | Analytics | `taskAnalytics()` main loop | 5943 | `mqttClient.connected()` (gate before building /trend JSON) | `getMqttConnectedCached()` |
| 37 | loopTask | `loop()` | 6544 | `mqttClient.connected()` | `getMqttConnectedCached()` |
| 38 | loopTask | `loop()` | 6601 | `mqttClient.connected()` | `getMqttConnectedCached()` |

4 sites. All are **read-only status checks** (gating/display), never followed by `.publish()`/`.connect()` in the same context — safe to satisfy from a cache with no functional loss. Worst case with a stale cache: display shows "CONNECTED" for one extra ~100ms tick after an actual disconnect, or Analytics builds one /trend payload that then sits in the queue slightly longer before Network4G can send it. Neither has safety implications.

---

## 4. New components required

### 4.1 Outbound MQTT queue (owned/consumed by Network4G only)

- **Purpose:** decouple "data is ready to publish" (Analytics) from "the act of publishing" (Network4G).
- **Producer:** Analytics (`taskAnalytics`), one call site (3.2 above).
- **Consumer:** Network4G, in its existing main loop, at a point analogous to where it currently drains `g_telemBuf`.

**DECISION: Q1 — new, independent queue.** (Locked following design review; supersedes the earlier Q2 lean.)

**Rationale for overriding the original Q2 lean:** Telemetry (`g_telemBuf`) and Trend have materially different traffic profiles that make a shared queue a latent coupling risk:

| | Telemetry (`g_telemBuf`) | Trend (/trend) |
|---|---|---|
| Frequency | Higher, event-driven (offline backlog can spike) | Fixed, 60s cadence |
| Latency sensitivity | Matters — represents real-time machine state | Doesn't — trend data is inherently a rolling aggregate |
| Payload size | Smaller | Larger (~1024B) |
| Criticality | Higher | Lower |

Sharing one buffer creates two failure directions: a telemetry backlog (e.g., during extended 4G outage) could starve /trend slots, or a large /trend payload could occupy capacity a telemetry replay burst needs right when it needs it. **Isolation is worth the small amount of duplicated code.** This also keeps the blast radius of any future bug in one queue's logic from affecting the other data path.

**Locked specification:**
- Independent `xQueueCreate` (or equivalent) holding `{topic_id, payload[], len, qos}` structs — separate from `g_telemBuf`/`mutexTelemBuf` entirely, no shared state with telemetry replay.
- Queue depth: 4–8 slots (still recommended — /trend's fixed 60s cadence means sustained backlog is not expected; this is generous headroom, not a tight budget).
- Overflow policy: **drop-newest with a counter** — mirrors the existing `[WARN] /trend JSON truncated!` visibility style, never blocks Analytics, and never risks Analytics blocking on a full queue.
- Enqueue failures surfaced via a new counter (e.g., `g_trendEnqueueDropCount`), added to the existing 30s status report table alongside `TelemBuf Overflow` for parity/visibility.

### 4.2 Cached MQTT connection-state flag

- **Purpose:** let DisplayUpdate/Analytics/loopTask read "is MQTT connected" without touching `mqttClient` itself.
- **Writer:** Network4G only, updated immediately after each `mqttClient.connected()` check it already performs today (rows #9, #14, #15/16, #17, #19, #20, #22, #23, #25 — no new calls needed, just also write the cached flag at those existing points).
- **Reader:** DisplayUpdate, Analytics, loopTask (4 sites above).

**DECISION: fold into existing `mutexSystemState` snapshot.** (Locked following design review; supersedes the volatile-bool option.)

**Rationale:** the deciding factor is *consistency of the snapshot*, not raw atomicity. `g_systemState` already represents the machine's point-in-time picture — RUNNING/STOPPED, RPM, fault status — read by consumers (DisplayUpdate, Analytics, loopTask) as one coherent snapshot under `mutexSystemState`. MQTT connectivity is conceptually part of "what is true about the system right now" in the same sense. Adding a parallel `volatile bool` alongside it would mean two different consistency models for two pieces of state that are often reasoned about together (e.g., a status report line that shows both machine state and MQTT connectivity should reflect the same instant, not two independently-torn reads). Folding it into the existing mutex-guarded struct keeps one consistency model for all "current system status" reads.

**Locked specification:**
- Add a `bool mqttConnected` field to the existing `g_systemState` struct (or equivalent structure already guarded by `mutexSystemState`).
- **Writer:** Network4G only, updated at its existing `.connected()` call sites (rows #9, #14, #15/16, #17, #19, #20, #22, #23, #25) — no new `mqttClient` calls introduced, just an additional write into the already-mutex-guarded struct at points where Network4G already holds the answer.
- **Reader:** DisplayUpdate (line 4906), Analytics (line 5943), loopTask (lines 6544, 6601) — all via the same `xSemaphoreTake(mutexSystemState, ...)` pattern already used for `g_systemState.state` elsewhere in the file. No new mutex introduced.

**Verification step retained regardless of mechanism chosen:** confirm no other code path infers "connected" indirectly by calling `.connected()` a second time right before a `.publish()` (i.e., check that we're not just moving the race, only removing genuinely redundant/non-owning reads) — the 3.1 vs 3.3 classification must be re-verified line-by-line during implementation, not just from the table in this document.

---

## 5. Per-task impact summary

| Task | Priority | Change | Risk if change is wrong |
|---|---|---|---|
| **Network4G** | 2 | Gains: queue-drain step in main loop, cache-write after each `.connected()` check. No change to existing connect/publish/loop logic. | Low — additive only, existing logic paths untouched |
| **Analytics** | 3 | Loses direct `.connected()` + `.publish()`; gains cached-read + enqueue call | Low — enqueue is non-blocking; worst case is delayed/dropped /trend publish, not a crash |
| **DisplayUpdate** | 3 | Loses direct `.connected()`; gains cached-read | Negligible — display-only |
| **loopTask (Arduino main)** | 1 | Loses direct `.connected()` (2 sites, both in the 30s status report); gains cached-read | Negligible — logging/reporting only |
| **ModbusRead / StateMachine** (Core 0) | 5 / 4 | No change (already zero references) | N/A |

---

## 6. Explicit non-goals for this change

- [ ] Do **not** touch the 7 existing mutexes (`mutexVibData`, `mutexSystemState`, `mutexI2C`, `mutexModem`, `mutexAggBufs`, `mutexFaultLatch`, `mutexTelemBuf`) — out of scope, not implicated.
- [ ] Do **not** modify Modbus/StateMachine (Core 0) — confirmed zero mqttClient references.
- [ ] Do **not** attempt to make `arduino-mqtt` / `lwmqtt` / mbedTLS internally thread-safe — we are removing the concurrent access, not fixing the libraries.
- [ ] Do **not** change JSON document sizes/structure for /sensor, /status, /vibration, /trend — unrelated to this bug (already verified: no shared buffers across topics).

---

## 7. Migration checklist (implementation order, once design questions in §4 are answered)

- [ ] 1. Decide Q1 vs Q2 (queue implementation) — team decision
- [ ] 2. Decide cache mechanism (volatile bool vs mutexSystemState) — team decision
- [ ] 3. Implement queue + enqueue function (no callers wired yet) — isolated addition, testable standalone
- [ ] 4. Implement cache write points inside Network4G's existing `.connected()` call sites (9 write points, all inside functions already owned by Network4G)
- [ ] 5. Implement cache read function `getMqttConnectedCached()`
- [ ] 6. Wire Analytics: replace line 5943 read, replace line 6064 publish with enqueue
- [ ] 7. Wire DisplayUpdate: replace line 4906 read
- [ ] 8. Wire loopTask: replace lines 6544, 6601 reads
- [ ] 9. Implement Network4G-side queue drain (consumer loop, publishes exactly like existing /sensor/status logic)
- [ ] 10. **Regression check — enforces the Architecture Invariant (§2a) as a build gate, not a one-time grep.** Concretely:
  - Scans for **both** `mqttClient.` **and** `gsmClient.`** call sites (the earlier draft only mentioned `mqttClient.` — corrected here; `gsmClient` is equally in scope of the ownership rule and was already broken out as its own audit rows #3–6, #10)
  - Whitelist: only call sites inside `taskNetwork()`, `setupTLS()`, and `publishTelemetry()` (the last, only because its single caller is confirmed to be `taskNetwork`) are permitted
  - **Any match outside the whitelist fails the build**, not just prints a warning — this is what makes §2a an enforced project rule rather than a convention someone can silently drift away from
  - Recommended implementation: a small script (grep/ripgrep + whitelist diff) run as a pre-build or CI step; exact tooling is an implementation detail, but "fails the build on violation" is not optional
- [ ] 11. Full diff review against v16.4 baseline before any flashing

**Note — implementation sequencing (does not renumber or modify any item above):** Item 9 (Network4G queue drain) must be implemented and merged before Item 6 (Analytics enqueue wiring) is merged, regardless of their listed order. Until Item 9 exists, items enqueued by Item 6 would have no consumer to reach Network4G and would instead be subject to the queue's approved overflow policy (§4.1 — drop-newest with counter). Merging Item 9 first keeps the drain path in place — dormant, since no producer exists until Item 6 lands — before Item 6 introduces the producer. This note is an implementation sequencing clarification only; it does not modify the approved design, any architecture decision, or the scope of any checklist item.

---

## 8. Test plan

### 8.1 Static verification
- [ ] Grep-based regression check (per §7.10) — must show **zero** direct `mqttClient`/`gsmClient` references outside Network4G's owned functions. Recommend committing this as a simple shell script / pre-commit check so future contributors can't reintroduce the pattern when adding a new MQTT topic.

### 8.2 Functional regression (confirm nothing broke)
- [ ] All 4 topics (/sensor, /status, /vibration, /trend) still publish at expected cadence (30s / 60s as applicable), payload content byte-identical to pre-refactor for a fixed synthetic input
- [ ] `TelemBuf Pending/Overflow/Replayed` counters behave identically to before (if Q2 chosen, verify /trend doesn't starve /sensor's existing replay slots; if Q1 chosen, verify the new queue doesn't interact badly with TelemBuf under simultaneous backlog)
- [ ] Motor stop/start cycle (the scenario from the original log) — confirm RESUME/CLEAR logic, EMA reseed, slope suppression all unaffected (this refactor should not touch `processRPM()` / `calcTrend()` at all — confirm via diff)

### 8.2a Ownership verification (behavioral, not just build-time)

The §7.10 CI check confirms the invariant *statically* (no forbidden call sites in source). This subsection confirms it *behaviorally* — that the running system actually respects single ownership under real conditions, and that the new queue/cache hold up under the specific stress pattern they exist for:

- [ ] Confirm at runtime (e.g., via debug log or trace) that Analytics never invokes any `mqttClient`/`gsmClient` method — only enqueues and reads the cached state
- [ ] Same confirmation for DisplayUpdate and loopTask — read-only cache access, nothing else
- [ ] **Queue depth stability under reconnect**: force a disconnect and observe the outbound queue's occupancy over the reconnect window — it should grow bounded (per §4.1's depth of 4–8) and never silently exceed that, with drops correctly counted via `g_trendEnqueueDropCount` if it does
- [ ] **Extended outage drain test**: force MQTT disconnect for 5–10 minutes (well beyond a normal reconnect), then restore connectivity — confirm the queue drains fully once Network4G reconnects, and that /trend publishing resumes on its normal 60s cadence afterward with no backlog pile-up or stuck state

This is the step that validates the *architecture's* behavior under stress, as distinct from §8.3 (which validates that the *original race* no longer reproduces) and §7.10 (which validates the *source* never reintroduces direct access).

### 8.3 Concurrency-targeted test (closes the "was this actually exploited" gap from §1)
- [ ] **Priority-injection repro attempt**: in a test build only, insert a deliberate short delay (e.g. `vTaskDelay`) inside Network4G's TLS reconnect path immediately before `mqttClient.connect()` returns, then force a reconnect (e.g. kill MQTT broker connection) at the same moment Analytics's 60s /trend flush is due. Repeat N cycles (e.g. 100+) on **v16.4 unmodified** to attempt to reproduce the crash under controlled conditions — this is the step that would let us upgrade "root cause candidate ~95–99%" to "confirmed." Run the same repro on the **refactored build** to confirm the crash no longer occurs under identical forced conditions.
- [ ] If reproducible: capture a fresh crash log + backtrace from this forced repro and re-run `addr2line` — if it lands on the same `mbedtls_ssl_*` frames, that is the closest thing to a direct proof available without a hardware debugger/JTAG trace.

### 8.4 Soak test (production-readiness gate)
- [ ] 24–48h continuous run on refactored firmware, replicating field conditions: periodic 4G signal drops (forces MQTT reconnect cycles), motor stop/start events at irregular intervals, normal 60s /trend cadence — zero panics/reboots is the pass criterion
- [ ] Compare `g_rebootCount` / `g_resetReasonStr` (NVS-persisted) before/after soak — should show 0 unexpected (`PANIC`/`TASK_WDT`/`INT_WDT`) resets

### 8.5 Rollout gate
- [ ] Canary: 1 unit in the field running the refactored build for an agreed observation window before fleet-wide rollout
- [ ] Keep v16.4 (current, unmodified) tagged and flashable as immediate rollback target
- [ ] Only after canary + soak pass: promote to fleet baseline as v16.5

---

## 9. Summary for sign-off

| Item | Status |
|---|---|
| Vulnerability confirmed (unsynchronized shared MQTT/TLS client across priority-preemptable tasks) | ✅ |
| Exploitation of this vulnerability in the observed crash | ⚠️ Strongly inferred (~95–99%), not instrumented-proven — §8.3 is the path to closing this |
| Fix approach selected | Option B (single-owner + queue/cache) |
| Architecture Invariant | ✅ Promoted to Project Rule (§2a), enforced as a build-failing check (§7.10) |
| Outbound queue design (Q1 vs Q2) | ✅ **Locked: Q1, independent queue** — isolated from `g_telemBuf` to avoid coupling telemetry and trend traffic profiles |
| Connection-state cache mechanism | ✅ **Locked: folded into `mutexSystemState`** — one consistency model for all "current system status" reads |
| CI enforcement scope | ✅ **Locked: covers both `mqttClient.` and `gsmClient.`, build-failing on violation** |
| Patch status | **Design fully locked — ready for implementation (§7), no open questions remain** |

All open questions from the previous revision are now resolved per design review. This document is ready to hand to implementation exactly as written in §7; no further design decisions are blocking.

**Design review status: closed, no design blockers.** Three review rounds completed (initial checklist → Q1/Q2 + cache lock → ownership verification addition). This closure applies **only to the design** — it does not by itself confirm the implementation fixes the crash. That confirmation is earned separately, through §8 (Regression + Soak), each compared explicitly against the v16.4 baseline. **v16.5 should not be promoted to production baseline until §8 passes** — design sign-off and production-readiness sign-off are two different gates, not one.
