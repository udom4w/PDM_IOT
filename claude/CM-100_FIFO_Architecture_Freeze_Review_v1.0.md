> **Document Status**
>
> **Type:** Architecture Freeze Review — gate document. Reviewed as lead firmware architect against `CM-100_FIFO_DRIVER_SDS_v1.0` and `CM-100_FIFO_Implementation_Plan_v1.0`, before any production code is written.
> **Version:** 1.0 findings (below, unmodified — historical record), **plus Resolution Addendum v1.1** (appended after the original Recommendation) confirming disposition of all 7 Must-Fix items against `CM-100_FIFO_DRIVER_SDS_v1.1` and `CM-100_FIFO_Implementation_Plan_v1.1`.
> **Date:** 2026-07-28 (findings); 2026-07-28 (resolution addendum, same day)
> **Scope discipline:** findings below are architectural — module boundaries, ownership, interfaces, dependency direction. Coding-style, naming, and other implementation-time concerns are explicitly out of scope and excluded even where noticed.
> **Method:** cross-checked the SDS and Implementation Plan against each other and against production source facts already established earlier in this investigation (cited, not re-derived). No new hardware investigation was performed.
> **Status:** 🟢 **ARCHITECTURE FROZEN** — see Final Consistency Audit at the end of this document.

# CM-100 FIFO Driver — Architecture Freeze Review

---

## Summary

**7 findings classified Must Fix Before Coding. 2 findings classified Should Improve. 9 areas classified Accept As-Is.**

None of the Must-Fix items require rewriting the four-layer design (L1 Transport / L2 Codec / L3 Session / L4 Diagnostic Workflow), the tick-sliced timing model (D-1), or the bus-ownership decision (D-15) — all three hold up under stress-testing and are confirmed sound below. Every Must-Fix item is a **boundary leak, an undefined responsibility, or an internal contradiction between the SDS and the Implementation Plan** — the kind of thing that is a one-paragraph or one-signature fix today and a retrofit-after-tests-exist problem if caught during or after coding.

**Recommendation: do not freeze yet.** See §Recommendation at the end.

---

## 1. Module Boundaries

### Finding 1.1 — L2 Frame Codec has a compile-time dependency on L3's arena type
**Must Fix Before Coding**

The SDS states L2 is "pure logic, host-testable" (§5.2) with "zero Arduino dependency" (§20.1's class diagram labels L2 explicitly `PURE LOGIC`, `PURE FUNCTION`). But Implementation Plan Task 2.3 drafts `FrameCodec_Step()`'s signature with a `FifoArena* arenaIfDump` parameter — meaning `fifo_codec.cpp` would need to `#include "fifo_arena.h"`, an L3 header, to even compile. This directly contradicts Task 2.2's own `SampleDecoder_DecodeStride()` signature, which correctly takes plain `int16_t*` output pointers with no L3 dependency at all.

This is not cosmetic. If L2 compiles against L3's arena type, L2 is no longer independently testable or reusable the way the SDS claims (D-7, the fix for PRR **C-14**) — every L2 unit test would need `FifoArena` linkable too, and the intended unidirectional dependency (L1 ← L2 ← L3) inverts at exactly this one call.

**Minimal fix:** `FrameCodec_Step()` should take plain `int16_t* xOut, int16_t* yOut, int16_t* zOut` write targets (matching Task 2.2's already-correct pattern), not a `FifoArena*`. L3's `S8_ReadDump` handler passes `FifoArena_WriteHandleX/Y/Z()`'s return values as those plain pointers — L3 bridges the two layers; L2 never needs to know `FifoArena` exists. No redesign; a one-line signature correction before Task 2.2/2.3 are coded.

### Everything else in Module Boundaries
**Accept As-Is.** The four-layer split itself (L1/L2/L3/L4) is sound: each layer's stated non-responsibilities (SDS §4.2) are consistent with what the Implementation Plan's tasks actually do, once Finding 1.1 is corrected.

---

## 2. Public API Stability

### Finding 2.1 — The single-consumer result model does not define how Phase 5 and Phase 7 coexist
**Must Fix Before Coding**

`FifoDriver_TryAcquireResult()`/`ReleaseResult()` implement a strict single-acquirer handoff (SDS §7.2, D-8/D-10) — deliberately so, per §9.2's reasoning. But the Implementation Plan describes **two** independent consumers of the same result: Task 5.2 (`/event` publish) is "called from the point where a session transitions to `RESULT_READY` (consumer side, after `TryAcquireResult`)", and Task 7.1 (`WaveformStore`) is "called from... the `TryAcquireResult`/`ReleaseResult` call site" — worded as if each independently acquires.

Under the API's actual contract, the *second* of these to call `TryAcquireResult()` gets `false` (already held) or finds the arena already released and reads garbage/nothing — whichever it is, it's a real bug class, not a documentation nit, because it's a use-after-release / double-acquire hazard on the exact object D-8 was designed to make impossible.

**Minimal fix:** define, now, that there is exactly **one** result-consumption coordinator — a single function (e.g. `FifoDriver_OnResultReady()`, called once per completed session) that acquires once, invokes the telemetry publisher and (when Phase 7 exists) the waveform store synchronously in a fixed order using the *same* acquired pointers, then releases once. This does not require writing Phase 7 now — it requires committing to the shape so Phase 5 doesn't get built around an assumption that breaks when Phase 7 arrives.

### Finding 2.2 — `outHandle` from `Request()` has no defined use
**Should Improve**

`FifoDriver_Request()` returns a `uint32_t* outHandle`, but no API entry point accepts a handle to disambiguate against (`TryAcquireResult()` takes none). With the depth-1 queue guaranteeing at most one in-flight capture, the handle currently has no consumer. Not unsafe, just dead API surface that will prompt "what is this for?" from the first person to read the header. Either make it meaningful (have `TryAcquireResult` optionally verify it against the held result, catching a stale-handle bug class for free) or remove it. Cheap either way, cheapest before any call site exists (Phase 6 is the first caller).

---

## 3. Internal Interfaces

### Finding 3.1 — `FrameCodec`'s scan/frame state is explicitly unresolved
**Must Fix Before Coding**

Implementation Plan Task 2.3's own signature draft contains the placeholder `/* internal scan state */ ...` — a candid admission that this wasn't settled. It matters more than a TODO marker suggests: if the anchor-scan position and partial-frame byte count end up as `static`/hidden state inside `fifo_codec.cpp` (the path of least resistance during coding), that directly violates the "No hidden state" principle the Implementation Plan itself lists as preserved (§3, mapped to D-6) — for *new* code, in the one file whose entire reason for existing is to not repeat the Validation Tool's mistakes.

It also breaks testability: hidden state means test cases can't run independently in the same binary without an explicit reset function nobody has specified, and it silently assumes exactly one capture stream ever exists (true today, but true *by accident of hidden state*, not by contract).

**Minimal fix:** define an explicit `FrameCodecState` struct (scan position, partial-frame byte count, running CRC) owned by L3's session state and passed by pointer into every `FrameCodec_Step()` call. No behavior changes — this is purely "where does the state live," decided before Task 2.3 is coded rather than discovered mid-implementation.

### Finding 3.2 — Admission-gate data access left as an open coding-time decision
**Must Fix Before Coding**

Implementation Plan Task 6.2 states directly: "this is a coding-time decision to make explicitly" — whether admission checks read `g_systemState`/`g_modbusConsecErrors`/MQTT state directly from the `.ino`'s globals, or receive them as an injected parameter. Left open, the path of least resistance during coding is "just read the global, it's right there" — which would give `fifo_driver.cpp` a hard compile-time dependency on production `.ino` global names and types, destroying host-testability for this specific function and reversing the dependency direction the rest of the design carefully avoids (L3 should depend on L1/L2 and plain data, never reach up into the monolithic `.ino`).

**Minimal fix:** decide now, not at coding time: `FifoDriver_CheckAdmissionGates(const FifoAdmissionContext* ctx)`, where `FifoAdmissionContext` is a plain struct (`bool motorStable; bool sensorHealthy; bool mqttReconnecting;`) populated by the *caller* in the `.ino` (which already has natural access to those globals) and passed in. Same information, same call site, zero new coupling.

*(Cross-reference: this is the same class of gap as Finding 1.1 — a boundary that the plan left implicit and that defaults, under coding pressure, to the coupled option rather than the clean one. Listed separately because it's a distinct file/task, not because the underlying lesson differs.)*

---

## 4. Ownership of Modbus Resources

### Finding 4.1 — SDS and Implementation Plan directly contradict each other on RS485 direction-control ownership
**Must Fix Before Coding**

This is the most consequential finding in this review, because an uncaught version of it is a functional failure, not a style issue.

- SDS §6.2 states plainly: *"It insulates the driver from the RS485 direction-control question... The transport implementation owns that difference; the driver never sees it."* — i.e., `Uart485Transport` (Task 1.2) is supposed to call `rs485Enable()`/`rs485Disable()` around its own `write()`.
- Implementation Plan Task 1.2's own **Risks** field says the opposite: *"Must not call `rs485Enable()`/`rs485Disable()` or touch `RS485_EN_PIN` — that remains `taskModbusRead`'s responsibility per D-15; this file only moves bytes on an already-enabled bus."*

These cannot both be true. Left unresolved, coding proceeds against whichever document the implementer happens to read last, and the two realistic failure modes are not subtle: if *nobody* toggles EN before the S3 `REQUEST` write, the 8-byte request frame may never actually reach the transceiver in transmit mode (confirmed from source: `RS485_EN_PIN` drives DE/RE tied together — LOW = receive, and production's own polling code toggles it around every transaction, `:4334/:4425/:4571/:4584/:4680`, established earlier this investigation) — the driver would then spin through `AWAIT_ANCHOR` timeouts indefinitely, indistinguishable from a genuine bus fault. If EN is left asserted (transmit) instead, the response can never be received either.

**Minimal fix:** the SDS's framing is correct and should stand — `Uart485Transport::write()` (Task 1.2) wraps its one blocking write with the *existing* `rs485Enable()`/`rs485Disable()` global functions, exactly as production's own polling code already does at every other transaction site. This is reuse, not new logic. Correct Task 1.2's Risks field to match SDS §6.2 (the plan's own text is the one that's wrong here, not the SDS).

### Everything else in Ownership of Modbus Resources
**Accept As-Is.** D-15 itself (`taskModbusRead` remains sole `modbus.*` caller, no new mutex, DESIGN-0002 Option A) is correctly reasoned and I found no better alternative under stress-testing — Option B's priority-inversion risk (a Priority-5 task blocking on a mutex a lower-priority task holds) is real and Option A avoids it structurally, not by convention. The depth-1 request queue's admission model is race-free by construction (`xQueueSend`'s atomicity means there's no separate check-then-enqueue window across cores) — verified by inspection, no TOCTOU found.

---

## 5. Timing Architecture

### Finding 5.1 — §15.3's retry-timing justification cites a timer it doesn't actually depend on
**Must Fix Before Coding**

SDS §15.3 justifies the mandatory bus-release-between-retries rule by arguing `T_CAPTURE_TOTAL_MS=20000` must stay under `ABSENT_STOPPING_MS=15000`, and that per-attempt bus release is "what keeps every individual bus-occupancy window under `ABSENT_STOPPING_MS`, so the motor FSM never sees a 15 s sensor absence."

This contradicts the SDS's *own* §13.3 table, three sections earlier, in the same document: *"RPM continuity | Unaffected — RPM comes from the `PIN_RPM` pulse ISR, independent of Modbus | Motor state machine keeps running throughout."* Both cannot be right. Tracing the actual mechanism, established earlier in this investigation: `ABSENT_STOPPING_MS`'s own source comment reads `signalPresent false (but fresh) > 15s -> STOPPING`, and `signalPresent` is derived from RPM evidence at the pulse-ISR EMA update site (`buildMotorStateEvidence()` — established in the very first source review of this investigation). `ABSENT_STOPPING_MS` watches the RPM pulse ISR's freshness, not the WTVB05 Modbus bus's — and the RPM ISR keeps firing regardless of what `taskModbusRead` is doing with `SerialRS485`.

If that's correct, §15.3's stated safety argument for the 20 s/15 s coupling is describing a constraint that doesn't bind — the *practice* of releasing the bus between retries is probably still good (it lets core telemetry breathe, and per K-15/RFC-0007 may matter to the sensor's own state machine), but the **reason given** is wrong, and a future engineer who trusts the SDS's stated reasoning when adjusting these timeouts could build a change on a false premise.

**Minimal fix:** correct the SDS's stated justification for the retry-release rule before Task 3.4 hardcodes `T_CAPTURE_TOTAL_MS` and the retry-release logic around it. Either find the actual production timer (if any) that *is* coupled to WTVB05 Modbus-read absence specifically (distinct from `MODBUS_OFFLINE_THRESHOLD`, which is separately and correctly handled via CN-10/Task 4.3), or state plainly that the 20 s budget is justified by telemetry-availability and sensor-settling concerns alone, not by `ABSENT_STOPPING_MS`. This is a documentation/reasoning fix, not necessarily a numeric one — the existing constants may well be fine as-is once the justification is corrected.

### Everything else in Timing Architecture
**Accept As-Is.** The core tick-sliced arithmetic (D-1) holds up under a second pass: 960 B/s arrival, 240 B/tick at 250 ms cadence, ~26 ticks ≈ 6.4 s matches K-11's measured transfer time exactly; the 512 B/call cap (2.1× arrival rate) gives genuine catch-up margin without materially raising worst-case `Service()` duration. The single-tick ownership-transition behavior (bus released and normal polling resuming within the *same* tick a capture completes) is intentional per §11.2 and is the correct choice, not an oversight. `vTaskDelayUntil`'s drift-correction behavior under an occasional near-5ms `Service()` call is standard FreeRTOS behavior, not a new risk this design introduces.

---

## 6. Memory Ownership

### Finding 6.1 — see Finding 2.1
The multi-consumer ownership ambiguity (§2) is, at root, a memory-safety issue: two independent copies of a `FifoCaptureResult` holding the same `x/y/z` pointers, with no coordination on which one calls `ReleaseResult()`, is a textbook use-after-release setup. Not double-counted as a separate Must-Fix here — the fix in §2.1 (single coordinator) resolves this too.

### Finding 6.2 — `FifoDriverConfig` lifetime at `Init()` is unstated
**Should Improve**

`FifoDriver_Init(const FifoDriverConfig* cfg)` — does `Init()` copy `cfg`'s contents, or retain the pointer? If `setup()` passes a stack-allocated config (a natural, common pattern) and the driver retains only the pointer, that's a dangling-pointer bug the moment `setup()` returns — a classic, easy-to-introduce, mildly annoying-to-diagnose embedded defect. Cheap to close: state explicitly, before Task 3.5 is coded, that `Init()` copies `cfg` by value into driver-owned static storage.

### Everything else in Memory Ownership
**Accept As-Is.** The single-arena, static-only, zero-heap strategy (D-10, D-11, D-12) is sound; the streaming-decode elimination of the 6144 B staging buffer is correctly reasoned and doesn't depend on anything found faulty in this review, once Finding 1.1's signature is corrected.

---

## 7. Error Propagation

### Finding 7.1 — Two of the fourteen error codes have no defined detection mechanism
**Must Fix Before Coding**

Cross-checking the SDS's 14-code taxonomy (§16.1) against the Implementation Plan's actual interfaces:

- `ERR_RX_OVERFLOW` — the SDS lists this as a transport-domain error, but `FifoTransport` (Task 1.1)'s interface has **no channel to signal it**: `read()` returns only a byte count, with no overflow/error out-parameter anywhere in the four-function interface. As specified, this error code can never actually be raised — it's currently dead.
- `ERR_PROGRESS_REGRESSION` / `ERR_PROGRESS_OVERRUN` — these require comparing the *current* progress-frame fill value against the *previous* one (K-10's invariant: fill is monotonic and ≤6144). But Task 2.3's `FrameCodec_Step()` only outputs the current fill for that call, and per Finding 3.1, `FrameCodec` should carry no comparison history itself. Nothing in either document currently assigns "remember the last fill and compare" to a specific owner.

Five of the seven `FifoFrameOutcome` values from Task 2.3 map cleanly 1:1 onto the corresponding transport/protocol-domain `FifoError` codes — the taxonomy isn't broken, but these two specific codes are effectively orphaned from any concrete detection point.

**Minimal fix:** (a) either add an overflow signal to the `FifoTransport` interface now — before it has two implementations (`Uart485Transport`, `LogReplayTransport`) that would each need retrofitting — e.g. a `bool (*hadOverflow)(void* ctx)` accessor, or accept that overflow detection isn't available at this layer and remove `ERR_RX_OVERFLOW` from the taxonomy until it is; (b) explicitly assign fill-regression/overrun comparison to L3's `S6_ReadProgress` handler (Task 3.3), which already needs to track "last progress fill" for the retry/telemeter diagnostics (`lastProgressFill` is already a field on `FifoCaptureResult`) — the comparison belongs right next to state that already exists, just needs to be named as this task's explicit job.

### Everything else in Error Propagation
**Accept As-Is.** The three-domain classification itself (transport / protocol / lifecycle) and the one-policy-per-code discipline (D-18, fixing PRR **C-7**) is sound and a clear improvement over the Validation Tool's collapsed `CRC_ERROR`.

---

## 8. Dependency Direction

Covered by Findings 1.1 (L2→L3) and 3.2 (L3→`.ino` globals) above — both are dependency-direction violations at root, listed under Module Boundaries and Internal Interfaces respectively since that's where they're first encountered, cross-referenced here rather than triplicated.

**Everything else: Accept As-Is.** With those two corrected, the intended direction (L1 ← L2 ← L3 ← L4 ← `.ino`, never reversed) holds throughout the rest of the design — including the transport injection itself (D-7), which is the mechanism that makes the *correct* direction possible in the first place.

---

## 9. Testability

No new findings beyond what's already captured above — testability is the *consequence* dimension for Findings 1.1, 3.1, and 3.2 (each one, left uncorrected, would have silently narrowed what's actually host-testable versus what the SDS claims is host-testable). With those three corrected, the testability story holds: L1's injected transport plus `LogReplayTransport` genuinely does turn the 7 recorded hardware logs into fixtures (PRR **C-14**'s fix), and L2/L3 become independently host-runnable as designed.

One minor implementation-time note, not elevated to a finding since it's a coding detail rather than an architectural one: the debug-build reentrancy assertion on `Service()`'s caller-task-handle (SDS §14.3) will need to compile out or no-op cleanly on a host test build with no FreeRTOS — worth a one-line `#ifdef` note at coding time, not a design change.

---

## 10. Future Extensibility

**Accept As-Is**, across the board:

- The design is deliberately **not** generalized beyond the one known WTVB05 RAWFIFO protocol (fixed 1024-sample/6144-byte arena, no parametrized capture-type abstraction). This is the correct call, not a gap — it matches the project's own stated scope discipline (`CLAUDE.md`: "ห้าม large-scale refactoring... เลือกการแก้ที่เล็กที่สุด") and avoids designing for a hypothetical second sensor that doesn't exist. Revisit only if a second capture type is actually proposed.
- `FIFO_PROTOCOL_MODEL` (D-14) is a well-designed extensibility seam — isolating the one genuinely open empirical question (G-1) to a single named constant on a single transition edge is exactly the right amount of future-proofing, not more.
- `REMOTE_ON_DEMAND`'s deferral (G-2) is already correctly scoped in the Implementation Plan (§8, "Notes for the Reviewer") as "a small addition to Task 6.1's pattern... not a redesign" — I agree with that assessment on inspection; the trigger-source enum and admission-gate shape both accommodate a fifth source without structural change.
- Fixed-size fields (`tag[16]`, etc.) are a deliberate, reasonable embedded-systems choice consistent with the zero-heap philosophy (D-2), not an extensibility gap.

---

## Consolidated Findings

| # | Area | Finding | Classification | Resolved (v1.1) |
|---|---|---|---|---|
| 1.1 | Module Boundaries / Dependency Direction | `FrameCodec_Step()` depends on `FifoArena`'s type; should take plain pointers | **Must Fix** | ✅ SDS §20.1/§20.6; Impl Plan Task 2.3 |
| 2.1 | Public API / Memory Ownership | No defined single consumer of `FifoCaptureResult` across Phase 5 + Phase 7 | **Must Fix** | ✅ SDS §9.1/§18.3/§18.4/§19.3/§20.1/§20.2/§20.6, §7.1 A-6/A-7; Impl Plan Task 5.2/7.1/§2.3 |
| 2.2 | Public API | `outHandle` has no consumer | Should Improve | ⏸ Deferred — not blocking, per original classification |
| 3.1 | Internal Interfaces / Testability | `FrameCodec`'s scan state left as an unresolved hidden-state placeholder | **Must Fix** | ✅ SDS §11.4; Impl Plan Task 2.3/3.3 |
| 3.2 | Internal Interfaces / Dependency Direction | Admission-gate data access left open between direct-global and injected-context | **Must Fix** | ✅ SDS §8.1/§19.2; Impl Plan Task 3.1/3.5/6.2 |
| 4.1 | Ownership of Modbus Resources | SDS and Implementation Plan directly contradict on who toggles RS485 EN | **Must Fix** | ✅ Impl Plan Task 1.1/1.2 corrected to match SDS §6.2 (SDS was already right) |
| 5.1 | Timing Architecture | §15.3's retry-timing justification cites a timer (`ABSENT_STOPPING_MS`) that §13.3 itself says is Modbus-independent | **Must Fix** | ✅ SDS §3.1 CN-9/§13.3/§15.3/§17.3, plus §19.3 and diagram annotations; Impl Plan Task 3.4 |
| 6.2 | Memory Ownership | `FifoDriverConfig` lifetime at `Init()` unstated | Should Improve | ⏸ Deferred — not blocking, per original classification |
| 7.1 | Error Propagation | `ERR_RX_OVERFLOW`, `ERR_PROGRESS_REGRESSION`/`OVERRUN` have no defined detection owner | **Must Fix** | ✅ SDS §6.1/§11.1/§16.1; Impl Plan Task 1.1/1.2/3.1/3.3 |

**Accept As-Is (confirmed sound under stress-testing):** the four-layer module structure; D-15 bus-ownership decision; D-1 tick-sliced timing arithmetic; the depth-1 request-queue admission model; the zero-heap static-allocation strategy; the three-domain error taxonomy's overall shape; the circuit-breaker/cooldown/watchdog recovery layering; `FIFO_PROTOCOL_MODEL`'s extensibility seam; the deliberate non-generalization beyond WTVB05's specific protocol.

---

## Recommendation

**Do not freeze the architecture yet.** Seven Must-Fix items remain. None require rewriting any part of the design — every one of them is a signature correction, an explicit state/ownership assignment, or a documentation correction resolving a contradiction between the SDS and the Implementation Plan. Estimated total cost to close all seven: a focused pass over both documents, no new hardware investigation, no coding.

**Path to freeze:**
1. Apply the seven minimal fixes listed above (each stated concretely enough to apply directly — no further design work needed per item).
2. Re-read the corrected SDS §6.2/§13.1/§15.3 and Implementation Plan Tasks 1.1–1.2, 2.2–2.3, 3.1, 3.5, 5.2, 6.2, 7.1 for internal consistency (a re-read, not a re-review — the shape of the fixes is already specified here).
3. Once applied and consistent, **the architecture is ready to freeze and Phase 1 implementation (Tasks 0.1–1.3) can begin** — those specific tasks (RX buffer sizing, transport interface, transport implementations) are unaffected by six of the seven findings and only lightly touched by Finding 4.1 (Task 1.2's Risks field correction) and Finding 1.1 (doesn't affect Phase 1 at all, only Phase 2's Task 2.3 signature).

Phase 1 can reasonably start **before** Findings 2.1, 3.2, 5.1, 6.2, and 7.1's `ERR_PROGRESS_*` half are closed, since none of those touch `fifo_transport.h`, `fifo_transport_uart485`, or `LogReplayTransport`. They must be closed before Phase 2 (Finding 1.1, 7.1) and Phase 3 (Finding 3.1) begin, respectively. Finding 4.1 must be closed before Task 1.2 specifically, since it's a correction to that exact task.

---

## Resolution Addendum (v1.1)

Applied to `CM-100_FIFO_DRIVER_SDS_v1.1` and `CM-100_FIFO_Implementation_Plan_v1.1`. Per instruction: architectural decisions already approved are preserved; no feature was added; no part of the driver was redesigned. Every change below eliminates a contradiction, an undefined ownership, a missing interface, or an incomplete contract — nothing else.

### Finding 1.1 — L2/L3 boundary leak

1. **Contradiction:** the SDS's own class diagram (§20.1) and stated principle (§5.2, "L2 is pure logic, zero Arduino dependency") required `FrameCodec` to be independent of L3, but the Implementation Plan's draft signature for `FrameCodec_Step()` (Task 2.3) took a `FifoArena*` parameter — an L3 type — contradicting the SDS it was supposed to implement.
2. **Final decision:** `FrameCodec_Step()` takes plain `int16_t* xOut, int16_t* yOut, int16_t* zOut` write targets, matching `SampleDecoder_DecodeStride()`'s already-correct pattern. `fifo_codec.cpp` never `#include`s `fifo_arena.h`. L3's `S8_ReadDump` handler bridges the two layers by passing `FifoArena_WriteHandleX/Y/Z()`'s return values as these plain pointers.
3. **Documents updated:** SDS §20.1 (class diagram — `FrameCodec` box corrected, bridging arrow annotated), §20.6 (data-flow diagram annotated). Implementation Plan Task 2.3 (signature corrected).
4. **Verified no contradiction remains:** `FifoArena`'s own interface (Task 3.2, `FifoArena_WriteHandleX/Y/Z() → int16_t*`) already returned plain pointers — only `FrameCodec_Step()`'s draft signature was ever inconsistent with it. Confirmed by direct re-read: no other section of either document references `FrameCodec` taking an arena type.

### Finding 2.1 — undefined multi-consumer ownership

1. **Contradiction:** SDS §7.2/D-8 mandate a single acquirer for `FifoCaptureResult`, but the Implementation Plan described Task 5.2 (`/event` publish) and Task 7.1 (`WaveformStore`) as each independently calling `TryAcquireResult`/`ReleaseResult` — a second acquirer would be rejected or would race the first's release, a use-after-release hazard on the exact object D-8 exists to protect.
2. **Final decision:** exactly one function, `handleFifoCaptureCompletion()`, is the sole call site for `TryAcquireResult`/`ReleaseResult`. It acquires once, runs the `/event`-publish step then (when Phase 7 exists) the `WaveformStore` step synchronously in that fixed order using the same pointers, then releases once. No new public API — this is application-level discipline over the existing A-6/A-7 contract, naming who follows it.
3. **Documents updated:** SDS §7.1 (A-6/A-7 caller column), §9.1 (new explanatory paragraph), §18.3, §18.4, §19.3 (workflow diagram), §20.1 (Consumers box redrawn), §20.2 (sequence diagram reordered — acquire now precedes both steps), §20.6 (data-flow diagram redrawn, single coordinator box replaces two parallel consumer arrows). Implementation Plan Task 5.2 (establishes the function), Task 7.1 (adds a step to it, does not acquire independently), §2.3 (added to the production-quality-abstractions list).
4. **Verified no contradiction remains:** grepped both documents for every remaining `TryAcquireResult` reference (9 in the SDS, 12 in the Implementation Plan) — all are consistent with the single-coordinator model; none depict or imply a second acquirer.

### Finding 3.1 — hidden state left unresolved

1. **Contradiction:** the Implementation Plan's own draft signature for `FrameCodec_Step()` contained the literal placeholder `/* internal scan state */ ...`, an explicit admission that the state's ownership (hidden/static vs. explicit/passed) was undecided — in direct tension with the "No hidden state" principle (D-6) the same document lists as preserved (§3).
2. **Final decision:** an explicit `FrameCodecState` struct (anchor-scan bytes discarded, partial-frame bytes received, a small partial buffer), owned and allocated by the session (L3), reset at the start of each frame attempt, passed by pointer into every `FrameCodec_Step()` call. No `static` state inside `fifo_codec.cpp`. Kept distinct from cross-frame session state (`lastProgressFill`, needed for Finding 7.1's K-10 check), which is explicitly *not* part of `FrameCodecState`.
3. **Documents updated:** SDS new §11.4 (state ownership explained, the distinction from `lastProgressFill` stated explicitly), §11.1 (`S6` row cross-referenced), §20.1 (class diagram, `FifoDriver`'s session line). Implementation Plan Task 2.3 (struct defined, placeholder removed), Task 3.3 (internal interfaces updated to show the struct's lifecycle).
4. **Verified no contradiction remains:** searched both documents for `FrameCodecState` (3 SDS occurrences, 5 Implementation Plan occurrences) — all consistent; no remaining reference to hidden or static per-frame state anywhere.

### Finding 3.2 — admission-gate data access left as an open decision

1. **Contradiction:** not a contradiction between documents, but an explicitly *undefined* interface — Implementation Plan Task 6.2 stated outright "this is a coding-time decision to make explicitly," leaving open whether admission checks read `.ino` globals directly (breaking L3's dependency direction and host-testability) or receive injected data.
2. **Final decision:** `FifoCaptureRequest` (already-existing, SDS §8.1) gains one field, `admissionContext` (type `FifoAdmissionContext { motorStable, sensorHealthy, mqttReconnecting }`), populated by the caller — each of Task 6.1's trigger sites, which already have natural access to the relevant globals — before calling the *unchanged* `FifoDriver_Request()`. `fifo_driver.cpp` never references a `.ino` global. This is more minimal than the function-parameter shape originally sketched in this review's Finding 3.2 write-up (a separate `FifoDriver_CheckAdmissionGates(ctx)` public entry point) — the chosen resolution reuses the existing single-entry-point pattern (`Request()`) rather than adding a new one, and resolves the identical dependency-direction/testability problem.
3. **Documents updated:** SDS §8.1 (field added), §19.2 (gate table gains a Source column, explanatory paragraph added). Implementation Plan Task 3.1 (`FifoAdmissionContext` defined, field added to the request struct), Task 3.5 (Risks field notes the boundary), Task 6.2 (rewritten — "coding-time decision" framing removed, decided).
4. **Verified no contradiction remains:** confirmed `admissionContext` appears consistently (6 SDS occurrences, 6 Implementation Plan occurrences) and `Request()`'s public signature is unchanged everywhere it is listed (SDS §7.1, Implementation Plan Task 3.5).

### Finding 4.1 — RS485 EN-pin ownership contradiction

1. **Contradiction:** SDS §6.2 stated the transport implementation owns `rs485Enable()`/`rs485Disable()` toggling around its `write()`. Implementation Plan Task 1.2's Risks field stated the literal opposite — that the transport must *not* touch it, "that remains `taskModbusRead`'s responsibility." Left unresolved, this was not a documentation nit but a functional-failure risk: without EN toggling, the S3 `REQUEST` write either never reaches the transceiver in transmit mode, or the bus never returns to receive mode — `AWAIT_ANCHOR` would then time out indefinitely, indistinguishable from a genuine bus fault.
2. **Final decision:** the SDS was correct; the Implementation Plan was wrong and is corrected to match. `Uart485Transport::write()` wraps its one blocking call with the existing `rs485Enable()`/`rs485Disable()` global functions — reuse of code already proven at every other production transaction site, not new logic. `taskModbusRead()` (D-15) remains the sole *sequencer* (decides *when* a FIFO transaction may run); direction control *within* that window belongs to the transport.
3. **Documents updated:** SDS §6.2 — **no change** (already correct; confirmed by re-read, including the class diagram at §20.1 which already showed `Uart485Transport` with `+ rs485Enable/Disable`). Implementation Plan Task 1.2 (Risks field rewritten to match).
4. **Verified no contradiction remains:** searched the Implementation Plan for every remaining `rs485Enable`/`RS485_EN_PIN` reference — one occurrence, the corrected text, consistent with SDS §6.2 and with the class diagram.

### Finding 5.1 — false timing-safety justification

1. **Contradiction:** SDS §15.3 justified mandatory bus-release-between-retries by arguing `T_CAPTURE_TOTAL_MS` (20000 ms) must stay under `ABSENT_STOPPING_MS` (15000 ms) to avoid a false motor-stopped detection — directly contradicting the SDS's own §13.3, three sections earlier, which states RPM/motor-state continuity is "unaffected... independent of Modbus." Traced to source facts established earlier in this investigation: `ABSENT_STOPPING_MS` fires off `signalPresent`, which is derived from the `PIN_RPM` pulse-ISR EMA, not from anything the WTVB05 Modbus bus does — so §13.3 was right and §15.3 was wrong.
2. **Final decision:** §15.3's justification is corrected. The retry-release-between-attempts *rule* is retained unchanged (still required, still correctly specified), re-grounded in its two reasons that were always independently valid: telemetry availability during multi-attempt sessions, and allowing the sensor's own state to settle between requests (K-15/RFC-0007). No numeric constant changed — `T_CAPTURE_TOTAL_MS`, `FIFO_MAX_RETRIES`, and the retry structure never depended on the false premise for their *values*, only for one sentence of *justification*.
3. **Documents updated:** SDS §3.1 (CN-9 row), §13.3 (table rows cross-referenced, not altered — they were already correct), §15.3 (rewritten in full), §17.3 (justification corrected), §19.3 and the §20.2 sequence-diagram annotation (both updated to remove the incorrect timer reference). Implementation Plan Task 3.4 (Purpose and Risks fields corrected, cross-referenced to Task 4.3's `g_modbusConsecErrors` check as the one genuinely Modbus-coupled constraint in this area).
4. **Verified no contradiction remains:** every remaining `ABSENT_STOPPING_MS` reference in the SDS (4 occurrences after correction) states the RPM-derived, Modbus-independent understanding consistently. **Found and fixed one additional, previously unflagged instance during this verification pass:** SDS §22 stated "Implementation must not begin until both [gates] clear" — directly contradicting the corrected position that neither gate blocks implementation. Corrected in the same pass (see §22, `[v1.1]`).

### Finding 7.1 — two error codes with no detection owner

1. **Contradiction:** not a document-vs-document contradiction but an internal incompleteness — the SDS's 14-code error taxonomy (§16.1) included `ERR_RX_OVERFLOW`, but the `FifoTransport` interface (§6.1) had no channel to signal an overflow, so the code could never actually be raised as specified. `ERR_PROGRESS_REGRESSION`/`ERR_PROGRESS_OVERRUN` require comparing consecutive progress-frame `fill` values, but neither document assigned that comparison to any specific function — and per Finding 3.1's resolution, it explicitly could not live inside `FrameCodecState` (per-frame, reset each attempt).
2. **Final decision:** (a) `FifoTransport` gains a fifth member, `hadOverflow(ctx) → bool`, the sole source of `ERR_RX_OVERFLOW`. (b) `S6_ReadProgress` (L3) compares each incoming `fill` against the session's `lastProgressFill` (already tracked for diagnostics, SDS §8.2) — `fill < lastProgressFill` → `ERR_PROGRESS_REGRESSION`; `fill > 6144` → `ERR_PROGRESS_OVERRUN`; otherwise `lastProgressFill` updates and the state machine proceeds.
3. **Documents updated:** SDS §6.1 (interface table gains `hadOverflow` row), §11.1 (`S6` row), §16.1 (both error rows cross-referenced to their new detection points). Implementation Plan Task 1.1 (interface signature), Task 1.2 (internal interfaces list), Task 3.1 (no change needed — `FifoError` enum already listed all 14 codes; only their detection *owner* was undefined), Task 3.3 (S6 handler responsibility spelled out explicitly).
4. **Verified no contradiction remains:** `hadOverflow` appears consistently (6 SDS occurrences, 5 Implementation Plan occurrences); `lastProgressFill`'s dual role (terminal diagnostic *and* live comparison anchor) is stated identically in SDS §11.4 and Implementation Plan Task 3.3.

---

## Final Consistency Audit

Performed across `CM-100_FIFO_DRIVER_SDS_v1.1`, `CM-100_FIFO_Implementation_Plan_v1.1`, and this review, after all seven resolutions above were applied.

| Audit criterion | Result | Evidence |
|---|---|---|
| Zero Must-Fix findings remain | ✅ Pass | All 7 rows in the Consolidated Findings table above show a Resolved (v1.1) entry with document/section citations. The 2 Should-Improve items (2.2, 6.2) remain open by design — that classification was never a freeze blocker, and both are explicitly logged as intentionally deferred (Implementation Plan §8, this document's table) rather than silently dropped. |
| Zero contradictory ownership definitions remain | ✅ Pass | Checked the three ownership questions this review identified as contested: (1) RS485 EN-pin control — SDS §6.2 and Implementation Plan Task 1.2 now state the identical position, cross-verified against the SDS's own class diagram. (2) `FifoCaptureResult` consumption — exactly one function (`handleFifoCaptureCompletion`) named consistently across all 21 combined references in both documents, zero remaining depiction of a second independent acquirer. (3) Admission-gate data source — `fifo_driver.cpp` reads only `FifoCaptureRequest.admissionContext` everywhere the gate logic is described, zero remaining reference to a direct `.ino`-global read from driver code. |
| Zero undefined public interfaces remain | ✅ Pass | `FifoTransport` (5 members, including `hadOverflow`) fully specified in both documents. `FrameCodec_Step()`'s signature contains no unresolved placeholder (the `/* internal scan state */ ...` marker is gone, replaced by the explicit `FrameCodecState*` parameter). `FifoCaptureRequest`/`FifoAdmissionContext` fully specified. Every one of the 14 `FifoError` codes now has a stated detection owner (verified individually: 5 map 1:1 from `FrameCodec`'s outcome enum, `ERR_RX_OVERFLOW` maps to `hadOverflow()`, `ERR_PROGRESS_REGRESSION`/`OVERRUN` map to `S6`'s comparison, the remaining 6 lifecycle-domain codes were never in question). |
| No approved architectural decision reversed | ✅ Pass | D-1 (tick-sliced timing), D-15 (bus ownership, no new mutex), the four-layer module structure, the zero-heap strategy, the 14-code/3-domain error taxonomy's overall shape, `FIFO_PROTOCOL_MODEL`'s extensibility seam, and every numeric timing constant are all unchanged from v1.0. Confirmed by diff-equivalence check: every edit made in this pass either corrects a stated contradiction, names a previously-anonymous owner, or adds a struct field/interface member needed to make an *already-specified* error code or gate reachable — none introduces new driver behavior. |
| No new feature introduced | ✅ Pass | `handleFifoCaptureCompletion()`, `FrameCodecState`, `FifoAdmissionContext`, and `hadOverflow()` are all either (a) application-level discipline over an existing contract (the coordinator), (b) an explicit name for state that had to exist somewhere regardless (the codec state), or (c) the minimum interface completion required for an already-approved error code or gate to be implementable at all. None expands what the driver does. |

**All three declared conditions are met: zero Must-Fix findings remain, zero contradictory ownership definitions remain, zero undefined public interfaces remain.**

# ARCHITECTURE FROZEN

**Recommendation: begin Phase 1 implementation** (`CM-100_FIFO_Implementation_Plan_v1.1`, Tasks 0.1–1.3 — RX buffer sizing, transport interface, transport implementations), per that document's own Integration Sequence (§4) and Definition of Done (§7). The two Should-Improve items (`outHandle`'s undefined use, `FifoDriverConfig`'s copy-vs-retain lifetime) remain open, non-blocking, and are coding-time decisions for whoever implements Tasks 3.5 and Task 1.1/3.5 respectively — flagged in both documents so they are addressed deliberately rather than by accident. G-1 and G-2 remain open per design (D-14, SDS §22 `[v1.1]`) and do not block any phase except G-1's effect on interpreting Phase 8's soak results.

*End of CM-100_FIFO_Architecture_Freeze_Review_v1.0 (Resolution Addendum v1.1).*
