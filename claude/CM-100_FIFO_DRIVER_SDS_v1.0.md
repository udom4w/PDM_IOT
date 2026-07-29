> **Document Status**
>
> **Type:** Software Design Specification (SDS) — design only, no implementation
> **Version:** 1.3 (v1.1 amended per Architecture Freeze Review v1.0, 7 Must-Fix findings; v1.2 amended per post-implementation Architecture Update — RS485 EN-pin ownership moved from `Uart485Transport` to `taskModbusRead()`'s `OwnsBus()` boundary, superseding Finding 4.1's original resolution; v1.3 amended per Architecture Review Feedback on v1.2 — removed the backward Task 1.2→Task 4.2 implementation dependency, documented the transport's bus/EN precondition explicitly, and replaced the "D-15 extended" framing with a standalone Architecture Decision Note, `ADR-1`, leaving D-15's text unchanged. See Revision Notes below.)
> **Date:** 2026-07-28
> **Design Review:** 🟢 Frozen (v1.1); EN-ownership model updated in v1.2; **v1.3 resolves architecture-review feedback on v1.2** (dependency direction, contract documentation, D-15/ADR separation) — still not re-frozen, since Task 1.2's implementation must still be revised to match before Phase 1 resumes (Task 1.3 onward). See `CM-100_ADR-1_Bus_Ownership_Includes_EN_Ownership_v1.0.md`.
> **Implementation:** Phase 1 in progress under `CM-100_FIFO_Implementation_Plan_v1.3`. Task 0.1 complete (uncommitted). Task 1.1 complete (uncommitted). Task 1.2 implemented against the superseded v1.1 EN model — **requires revision** (remove EN-pin references from `fifo_transport_uart485.cpp`) to match v1.2/v1.3 before proceeding. **No firmware changes were made in v1.3 — document-only review.**
>
> **Supersedes:** nothing. **Complements:** `DESIGN-0001` (Trigger Manager), `DESIGN-0002` (FIFO Task), `DESIGN-0003` (Physical Invariant).
> **Relationship to `DESIGN-0002`:** DESIGN-0002 §4 poses one Critical open question (Modbus bus ownership) and does not decide it. This SDS **adopts Option A** and states why (§13). That adoption still requires project-owner sign-off per `CLAUDE_RULES.md` §16.
>
> **Source of validated knowledge:** Production Readiness Review of `WTVB05_ValidationTool_v3_11_TRUEPOLL` (2026-07-28), `RFC-0006`, `RFC-0007`, `WTVB05_FIFO_Investigation_Report.md`.
> **Explicitly NOT a source:** the Validation Tool's implementation. No file, function, or control-flow structure from that project is carried forward. See §2.2.
>
> **Firmware baseline:** `WTVB02_ESP32S3_V16_CM_fault_latch_v3_hardened_patched_v16_5.ino`, HEAD `fece844`, blob `aba9c1f`, branch `recover-experimental`.

# CM-100_FIFO_DRIVER_SDS_v1.0

## Revision Notes (v1.0 → v1.1)

Applied per `CM-100_FIFO_Architecture_Freeze_Review_v1.0.md`. No architectural decision reversed; no feature added. Each row eliminates a contradiction, an undefined ownership, a missing interface, or an incomplete contract.

| Finding | Section(s) corrected | Change |
|---|---|---|
| 1.1 | §20.1 | `FrameCodec` (L2) documented as taking plain `int16_t*` write pointers, never `FifoArena*` — L2 has zero dependency on any L3 type |
| 2.1 | §7.1, §9.1, §18.3, §18.4, §19.3, §20.1 | Introduced **one** named application-level coordinator, `handleFifoCaptureCompletion()`, as the sole consumer of `TryAcquireResult`/`ReleaseResult`; telemetry publish and (later) waveform storage are internal steps of that one function, not independent acquirers |
| 3.1 | §11.3 (new §11.4) | `FrameCodec`'s per-frame scan/parse state is explicit (`FrameCodecState`, owned by the session, passed by pointer) — no hidden/static state |
| 3.2 | §8.1, §19.2 | Admission-gate inputs (`motorStable`, `sensorHealthy`, `mqttReconnecting`) added as a plain `admissionContext` field on the existing `FifoCaptureRequest` — `Request()`'s signature is unchanged; the driver never reads `.ino` globals directly |
| 4.1 | §6.2, §13.2, §20.1 (superseded — see Architecture Update, v1.2) | Originally resolved as "transport owns EN, matching §6.2." Post-implementation review found production's `rs485Enable()`/`rs485Disable()` are paired and session-scoped (verified at lines 4334/4425, 4571/4584, 4680), not per-transmission — and re-read D-15 ("`taskModbusRead()` is sole owner of the RS485 bus") as including EN assertion, matching every other bus window in production. Final decision: EN ownership moved to the `taskModbusRead()`/`OwnsBus()` boundary (Task 4.2); `Uart485Transport` is a pure byte transport |
| 5.1 | §3.1 (CN-9), §15.3, §17.3 | Corrected: `ABSENT_STOPPING_MS`/`ABSENT_STOPPED_MS` are driven by the RPM pulse-ISR (`signalPresent`), independent of the WTVB05 Modbus bus (consistent with §13.3, which was already correct). Retry-release-between-attempts is retained, re-justified on telemetry-availability and sensor-settling grounds, not on a false absence-timer coupling. No numeric constant changed |
| 7.1 | §6.1, §11.1, §16.1 | Added `hadOverflow(ctx)` to the transport interface, giving `ERR_RX_OVERFLOW` a defined source. Assigned `ERR_PROGRESS_REGRESSION`/`ERR_PROGRESS_OVERRUN` detection explicitly to the `S6 READ_PROGRESS` handler, comparing against the already-tracked `lastProgressFill` |

## Revision Notes (v1.1 → v1.2) — Architecture Update

Applied post-implementation, during Task 1.2's implementation review. Supersedes Finding 4.1's v1.1 resolution only. No other v1.1 decision is affected — see the Architecture Change Report (companion document) §5 for the explicit cross-check. Trigger: re-verification of production source during coding surfaced that `rs485Enable()`/`rs485Disable()` are paired, session-scoped calls (not the Validation Tool's permanently-enabled model, and not the v1.1 assumption of per-`write()`-call symmetric toggling either) — both prior readings were incomplete, not just the Implementation Plan's.

| Section | Change |
|---|---|
| §6.2 item 3 | Rewritten: transport is insulated from the EN-pin question *entirely* (no member, no reference), not merely "insulated from which model applies." |
| §13.2 (new §13.2.1) | **New content**, not a correction: `taskModbusRead()`'s `OwnsBus()` boundary now explicitly brackets `rs485Enable()`/`rs485Disable()`, with the guarantee argument (state-machine reachability) spelled out, and a new requirement that `rs485Disable()` fire on every bus-release path, not just the happy path. |
| §20.1 | `Uart485Transport` box: removed `+ rs485Enable/Disable`. Incidental fix in the same pass: `hadOverflow()` was missing from the `FifoTransport` interface box (present in prose §6.1 since v1.1, never added to this diagram) — added. |

## Revision Notes (v1.2 → v1.3) — Architecture Review Feedback

Applied per Architecture Review Feedback on v1.2. All three items are document-only corrections to how the v1.2 decision is recorded and depended upon; **the v1.2 decision itself (EN ownership at the `taskModbusRead()`/`OwnsBus()` boundary) is unchanged.** No firmware changes.

| Item | Section(s) corrected | Change |
|---|---|---|
| 1. Backward dependency | Implementation Plan Task 1.2, Task 4.2 | Task 1.2's Risks/Dependencies previously stated it was only correct once Task 4.2 existed and had asserted EN — a backward dependency of an earlier task on a later one. Removed: Task 1.2 is independently correct as a transport layer against a documented precondition (item 2 below); Task 4.2 satisfies that precondition at integration time, which is integration, not a code dependency |
| 2. Precondition documentation | §6.1 | Added an explicit precondition to the `write()` contract row: the caller (not the transport) is responsible for already owning the RS485 bus and having already asserted EN before calling `write()`. This was previously argued only in prose (§13.2.1); it is now part of the interface contract table itself |
| 3. D-15 vs. ADR | §13.2.1, §24 | §13.2.1 no longer frames its content as "D-15 extended." D-15's text (§13.1) is unchanged. The EN-specific interpretation is now recorded as a standalone Architecture Decision Note, `ADR-1` (`CM-100_ADR-1_Bus_Ownership_Includes_EN_Ownership_v1.0.md`), referenced from §13.2.1 and §24 rather than folded into D-15 |

---

## 1. Purpose and Scope

### 1.1 Purpose

Define, from first principles, a production FIFO Waveform Driver for CM-100 that acquires 1024 tri-axial raw acceleration samples from the WTVB05 sensor over the existing RS485 bus, without degrading any existing CM-100 function.

### 1.2 In scope

Driver responsibilities, public API, internal state machine, lifecycle, transport abstraction, integration with `taskModbusRead()`, bus and buffer ownership, memory strategy, error model, thread safety, timeout and recovery strategy, telemetry interface, diagnostic workflow, and the four required diagram sets.

### 1.3 Out of scope

- Trigger arbitration policy → `DESIGN-0001`.
- The physics checks that would produce a trigger → `DESIGN-0003`.
- Any DSP, FFT, or analysis of captured samples. This driver **acquires bytes and hands them over**; it never interprets them. RFC-0006 §1 is explicit that CRC-validity says nothing about content identity, and this driver inherits that discipline.
- Cloud-side storage, dashboards, or the remote command channel (§22 G-2).

### 1.4 Design principle governing this document

> **The driver's only job is to move 6144 bytes from the sensor into memory, correctly or not at all, without ever making any other part of CM-100 late.**

Every decision below is traceable to that sentence.

---

## 2. Inputs

### 2.1 Validated engineering knowledge carried forward

Each item below is corroborated by at least two independent sources (datasheet + real capture, or RFC + hardware log). This is the *entire* inheritance from the Validation Tool.

| # | Fact | Value | Corroboration |
|---|---|---|---|
| K-1 | RAWFIFO register | `0x002C` | Datasheet §6.1.4.16; hardware captures |
| K-2 | Request frame | `50 03 00 2C 00 01 <CRC_lo> <CRC_hi>` | Datasheet; 168 logged request/response pairs |
| K-3 | Response shape A — progress | `50 03 01 <fill_hi> <fill_lo> <CRC_lo> <CRC_hi>` (7 B) | Datasheet; logs |
| K-4 | Response shape B — full dump | `50 03 00 <6144 B payload> <CRC_lo> <CRC_hi>` (6149 B) | Datasheet; 12 successful captures |
| K-5 | `Len` byte is a **type discriminator, not a length** | `0x01`=progress, `0x00`=dump | RFC-0006; direct observation |
| K-6 | CRC algorithm | CRC-16/Modbus, poly `0xA001`, init `0xFFFF`, transmitted low-byte-first | 12 CRC-verified captures |
| K-7 | CRC coverage | Header bytes + payload; excludes the CRC pair itself | Verified on both frame shapes |
| K-8 | Payload layout | 1024 samples × 3 axes × int16 **big-endian**, stride 6 B, order X,Y,Z | RFC-0007; byte arithmetic exact |
| K-9 | Nominal scale | `raw / 32768.0 × 16.0` → g | Datasheet §6.1.4.8 — 🟡 **evidence-backed, not confirmed** (RFC-0007 §4) |
| K-10 | Progress `fill` is monotonic and bounded by 6144 | observed 2277…4641; never exceeded 6144 | RFC-0006 §; logs |
| K-11 | **Transfer duration is baud-limited** | **6402 ms**, σ≈0.5 ms across 12 captures | `6146 B × 10 bits ÷ 9600 = 6.402 s` — arithmetic identity |
| K-12 | Request→first-byte latency | 256–260 ms | logs |
| K-13 | Max inter-byte gap during dump | ≤ 2 ms mid-dump; ≤ 239 ms across a poll boundary | logs |
| K-14 | Failure cascades onto the next unrelated Modbus read | `0xE2` (`ku8MBResponseTimedOut`) on the following register read unless the bus is drained | Validation Tool v3.1 root-cause note; reproduced |
| K-15 | Empirical success rate of the *tool's* strategy | 12/19 ≈ 63% overall; 1/6 ≈ 17% at SR6/512 Hz | 7 hardware logs |

**K-11 is the single most consequential fact in this document.** It is an arithmetic identity, not a measurement artifact: at 9600 baud the transfer *cannot* be faster. No driver design can shorten it. Every architectural decision in §13 exists to absorb 6.4 seconds of bus occupancy rather than to avoid it.

**K-9 carries an explicit caveat.** The scale factor is evidence-backed but unconfirmed. The driver therefore **stores raw int16 counts only** and never applies the scale (§8.3). Scaling is a consumer decision, made where the caveat is visible.

### 2.2 Explicitly discarded

Nothing from the Validation Tool's implementation is reused. Discarded in full, with the Production Readiness Review finding that condemns each:

| Discarded | PRR finding |
|---|---|
| `readOneFIFOFrame()`, `readFIFORaw()`, `readFIFOHybrid()`, `readFIFOPureListen()`, `fifoPassiveListen()`, `runFIFOStressOnce()`, `runFIFOTruePollOnce()` | C-8 — five competing protocol hypotheses; a driver must embody one |
| Unyielding `while(!available()) continue;` receive loops | C-1 — would starve Core 0 IDLE0 (`WDT_PANIC=y`) and the StateMachine task |
| `g_fifoX/Y/Z` as bare globals | C-2 — no validity contract; failed capture leaves stale, plausible, CRC-valid data |
| Direct `SerialRS485` access from the FIFO path | C-3 — unarbitrated second owner of a shared bus |
| Two 6144 B static `dataBuf` copies | C-4 — 6144 B pure duplication, confirmed in ELF |
| `malloc(6147)` + `memcpy` on the CRC path | C-5 — heap on hot path; OOM misreported as CRC error |
| Unbounded anchor resync | C-6 — can lock onto payload bytes |
| 4-value `FIFOFrameResult` enum | C-7 — three dissimilar failures collapsed into `CRC_ERROR` |
| `Serial.printf()` inside the receive path | C-10 — I/O interleaved with frame reception |
| `dumpFIFOToSerialCSV()` | 1024 `printf` ≈ 5 s of blocking Serial |
| Serial CLI, register polling, spectrum energy, anomaly scoring, CSV formatting | C-12 — CM-100 already implements all of it, with cross-core discipline this tool lacks |

---

## 3. Constraints

### 3.1 Hard constraints from the CM-100 baseline

Verified against HEAD `fece844`:

| # | Constraint | Value | Consequence for this design |
|---|---|---|---|
| CN-1 | `taskModbusRead` cadence | `vTaskDelayUntil`, `pdMS_TO_TICKS(250)` = 4 Hz | Driver must fit a **250 ms service budget** |
| CN-2 | `taskModbusRead` priority / core | `PRIORITY_MODBUS 5` (highest), Core 0 | Any blocking here starves `StateMachine` (prio 4, same core) |
| CN-3 | `STACK_SIZE_MODBUS` | **4096 B** | **No multi-KB stack buffers.** A 6144 B stack array is an immediate overflow |
| CN-4 | Core 0 IDLE watchdog | `CONFIG_ESP_TASK_WDT_CHECK_IDLE_TASK_CPU0=y`, `WDT_PANIC=y`, reconfigured to 30 s | A busy-wait on Core 0 → panic reboot |
| CN-5 | RS485 RX buffer | **default 256 B** — production never calls `setRxBufferSize()` | ≈267 ms headroom at 960 B/s. **Insufficient**; must be enlarged (§10.4) |
| CN-6 | Bus is multi-slave | `modbus.begin()` re-targeted between `MODBUS_SLAVE_ID` (0x50) and `CURRENT_SENSOR_ID` (0x01) | Bus ownership must account for the CTR4A01 current sensor too |
| CN-7 | Single `ModbusMaster modbus;` instance, **no mutex** | DESIGN-0002 §2.1 | Adding a second caller recreates the reproduced MQTT-crash precondition |
| CN-8 | MQTT is **publish-only** | no `subscribe()` anywhere | No remote command channel exists (§22 G-2) |
| CN-9 | RPM/motor-state absence timers | `ABSENT_STOPPING_MS 15000`, `ABSENT_STOPPED_MS 30000` | Driven by `PIN_RPM` pulse-ISR freshness (`signalPresent`) — **confirmed independent of the WTVB05 Modbus bus** (§13.3). Not a ceiling on capture duration; corrected in v1.1, see Revision Notes and §15.3 |
| CN-10 | Modbus offline detector | `MODBUS_OFFLINE_THRESHOLD 3` consecutive errors | Suspended polling must not increment `g_modbusConsecErrors` |
| CN-11 | Partition scheme | `app3M_fat9M_16MB` — **9 MB FAT already provisioned** | Local waveform storage is available at zero cost (§18.4) |
| CN-12 | `.ino` auto-prototype | custom types used as return/param types must be declared before the first function | Driver types must be declared in the header region |
| CN-13 | MQTT payload backward-compat | `CLAUDE.md` — never remove a field | Telemetry must be purely additive (§18) |

### 3.2 Non-goals

- **Not** continuous or streaming waveform acquisition. Per Architecture Master Plan §5.1, CM-100 is a condition-monitoring product; waveform capture is dormant diagnostic infrastructure.
- **Not** a customer-visible feature in CM-100.
- **Not** a general Modbus abstraction. It handles exactly one non-standard transaction.
- **Not** self-tuning. No adaptive timeouts, no learned thresholds. All budgets are fixed constants derived from §2.1 measurements.

---

## 4. Driver Responsibilities

### 4.1 The driver IS responsible for

| R# | Responsibility |
|---|---|
| R-1 | Accepting a capture request and admitting or rejecting it against explicit preconditions |
| R-2 | Emitting the RAWFIFO request frame per K-2 |
| R-3 | Recognising and classifying the two response shapes (K-3, K-4) |
| R-4 | Incrementally consuming received bytes within a bounded per-call budget, never blocking |
| R-5 | Streaming CRC verification (K-6, K-7) |
| R-6 | Decoding payload to raw int16 tri-axial samples (K-8) — **no scaling** |
| R-7 | Producing exactly one immutable result object binding data to validity |
| R-8 | Classifying every failure into a distinct, actionable error code |
| R-9 | Bounded recovery: resync, retry, drain, cool down, and finally give up |
| R-10 | Leaving the RS485 bus provably clean before returning it (K-14) |
| R-11 | Publishing capture metadata and diagnostics via the existing telemetry path |
| R-12 | Maintaining lifetime counters for field diagnosis |

### 4.2 The driver is NOT responsible for

| Not responsible | Owner |
|---|---|
| Deciding *when* to capture | Trigger Manager (`DESIGN-0001`) |
| Owning or configuring the UART | `taskModbusRead` |
| Owning the sample rate (`REG_SR`) | Caller — recorded, never modified (§5.3 D-3) |
| Interpreting sample content | Consumer |
| Applying the g scale factor (K-9) | Consumer |
| Transporting the waveform to the cloud | Network layer (§18.4) |
| Persisting the waveform | Storage layer (§18.4) |

**Why this split.** Every responsibility the driver refuses is one it cannot get wrong, and one that does not have to be re-verified when the driver changes. The Validation Tool failed precisely because it accepted all of these at once (PRR C-12).

---

## 5. Architecture Overview

### 5.1 The central design decision

> **D-1 — The driver is a passive, non-blocking, tick-driven state machine serviced by `taskModbusRead()`. It owns no task, no thread, and no bus.**

**Why.** K-11 fixes the transfer at 6402 ms. CN-1 fixes the service cadence at 250 ms. CN-2/CN-4 forbid blocking on Core 0. Those three facts admit exactly one shape: the driver must be *sliced across ~26 existing task ticks* rather than run to completion in one.

The arithmetic works out favourably, and not by coincidence:

```
  arrival rate         = 9600 baud / 10 bits per byte      =  960 B/s
  bytes per 250 ms tick= 960 × 0.25                        =  240 B
  full dump            = 6146 B ÷ 240 B/tick               ≈ 26 ticks
  total wall clock     = 26 × 250 ms                       ≈ 6.4 s   ← equals K-11
```

The drain paces itself against arrival automatically. Each `Service()` call consumes only what has already arrived, so per-call work is ~240 bytes of CRC-and-decode — tens of microseconds — inside a 250 ms budget. **The 6.4 s constraint and the 250 ms budget are simultaneously satisfied with a ~1000× margin on CPU.**

**Alternatives considered.**

| Alternative | Verdict | Reason |
|---|---|---|
| **A. Dedicated `FIFOTask` + mutex on `modbus`** (DESIGN-0002 Option B, the plan's literal diagram) | ❌ Rejected | Introduces priority inversion: `taskModbusRead` (prio 5, time-critical) could block on a mutex held by a lower-priority task. DESIGN-0002 §4 rates this blast radius *strictly worse* than the original MQTT bug. Also requires retrofitting a mutex onto all 8 proven `modbus.*` call sites — `CLAUDE_RULES.md` §5 discourages this |
| **B. Run to completion inside `taskModbusRead` (blocking 6.4 s)** | ❌ Rejected | Violates CN-2/CN-4. Starves StateMachine (prio 4) for 6.4 s and risks IDLE0 watchdog panic. This is PRR finding C-1 reproduced deliberately |
| **C. UART event-queue task blocking on `xQueueReceive`** | 🟡 Deferred | Genuinely elegant and removes the polling entirely, but still creates a second bus owner (CN-7) and adds a task + stack. Revisit if D-1 ever proves insufficient. Recorded as the preferred future evolution |
| **D. DMA / hardware ring capture** | ❌ Rejected | ESP32-S3 UHCI DMA on UART is available but adds a large, poorly-covered dependency for a dormant feature. CPU cost of D-1 is already negligible — DMA solves a problem we do not have |

**Tradeoffs accepted.** D-1 couples the driver's progress to `taskModbusRead`'s liveness — if that task stalls, capture stalls. That is acceptable: if the highest-priority Core-0 task has stalled, a waveform capture is not the system's problem. D-1 also means the driver cannot be tested by "call it and wait"; tests must pump `Service()` in a loop, which the transport abstraction (§6) makes trivial.

### 5.2 Layering

```
┌──────────────────────────────────────────────────────────────┐
│  L4  Diagnostic Workflow    (§19)                            │
│      trigger admission · publication · retention             │
├──────────────────────────────────────────────────────────────┤
│  L3  Capture Session        (§11, §12, §17)                  │
│      state machine · retry budget · cooldown · circuit breaker│
├──────────────────────────────────────────────────────────────┤
│  L2  Frame Codec            (§8, §16)                        │
│      shape discrimination · streaming CRC · streaming decode │
├──────────────────────────────────────────────────────────────┤
│  L1  Transport Abstraction  (§6)                             │
│      non-blocking byte source/sink · monotonic clock         │
└──────────────────────────────────────────────────────────────┘
             ▲ injected by taskModbusRead, never self-acquired
```

**Why four layers.** Each boundary is a testability seam. L2 and L3 are pure logic once L1 is abstracted — they can be exercised on a host with recorded byte streams (§23). The Validation Tool had no boundaries at all, which is why only 1 of its 38 functions was unit-testable (PRR C-14).

### 5.3 Standing design rules

| Rule | Statement | Rationale |
|---|---|---|
| D-2 | **Zero heap.** No `malloc`, `new`, `String`, or dynamic container anywhere in the driver | PRR C-5. CM-100's heap is contended by mTLS/MQTT; a 6 KB transient allocation is a fragmentation source |
| D-3 | **The driver never writes a sensor register.** It reads `REG_SR` once per capture for provenance and writes nothing | Sensor config is CM-100's, not the driver's. The Validation Tool rebooted and NVM-saved the sensor at every boot — unacceptable side effects and sensor flash wear |
| D-4 | **No I/O of any kind inside the receive path** | PRR C-10. Diagnostics are recorded into the result object and emitted after the frame boundary |
| D-5 | **Every timing constant is named, documented, and traceable to a §2.1 measurement** | PRR C-9: the tool silently overrode its own timeout parameter, invalidating experiments run with it |
| D-6 | **All state is reachable only through the driver's own API.** No global the rest of the firmware can read or write | PRR C-2 |

---

## 6. Transport Abstraction

### 6.1 Interface

> **D-7 — The driver receives an injected transport interface. It never references `SerialRS485`, `Serial`, or `modbus`.**

Conceptual shape (declaration, not implementation):

| Member | Contract |
|---|---|
| `ctx` | Opaque owner-supplied context pointer |
| `read(ctx, dst, maxLen) → actualLen` | **Non-blocking.** Copies up to `maxLen` already-received bytes. Returns 0 immediately if none. **Must never wait** |
| `write(ctx, src, len) → actualLen` | Blocking only for the duration of the 8-byte request (≈8.3 ms at 9600 baud, accepted — see tradeoff below). **Precondition `[v1.3]`:** the caller already owns the RS485 bus and has already asserted EN before this is called — see below |
| `available(ctx) → count` | Bytes currently buffered |
| `flushRx(ctx)` | Discard all buffered RX bytes |
| `nowMs() → uint32_t` | Monotonic milliseconds. Injected so tests control time |
| `hadOverflow(ctx) → bool` | **[v1.1, Freeze Review Finding 7.1]** True if the underlying transport detected a receive-buffer overflow since the last check (implementation-specific detection mechanism — e.g. a UART overflow flag where the platform exposes one; not specified at this level). This is the sole source of `ERR_RX_OVERFLOW` (§16.1) — without it that code had no way to ever be raised |

`read()` returning 0 is **not** an error — it is the normal case on most ticks. The driver distinguishes "nothing yet" from "nothing for too long" via the timeout layer (§15), never by spinning.

> **Transport precondition `[v1.3, Architecture Review Feedback]`.** Every `write(ctx, src, len)` call is made under two preconditions the transport itself never checks and never enforces: **(1)** the caller already holds RS485 bus ownership, and **(2)** EN has already been asserted for the duration of the call. This is a contract obligation on the caller, stated here as part of the interface itself — it is not something a specific task "provides" to `Uart485Transport`; it is something every caller of `write()`, present or future, must already have done. See ADR-1 (§13.2.1) for why bus ownership is read as including EN assertion, and Implementation Plan Task 1.2/Task 4.2 for which concrete task currently satisfies it.

### 6.2 Rationale

**Why an injected interface rather than direct UART calls.**

1. **It is the mechanism that enforces bus ownership.** The driver *cannot* touch the bus without being handed the means, so PRR C-3 becomes structurally impossible rather than a rule someone must remember.
2. **It makes L2/L3 host-testable.** A fixture transport replaying the 7 recorded hardware logs — including the 7 failures — turns the existing evidence corpus into a regression suite (§23).
3. **It insulates the driver from the RS485 EN-pin question entirely.** `[v1.2/v1.3, Architecture Update]` `FifoTransport` has no EN-related member, and `Uart485Transport` never references `RS485_EN_PIN`. EN assertion/deassertion is owned by `taskModbusRead()`'s bus-ownership boundary (§13.2, Task 4.2) — the same actor, and the same mechanism (`rs485Enable()`/`rs485Disable()`), that already brackets every other Modbus transaction burst in production (verified: lines 4334/4425, 4571/4584, 4680). A FIFO capture is, structurally, one more bus-ownership window; it is bracketed the same way, by the same owner. L1/L2/L3 never see this — they only ever observe that bytes can be sent and received, never why.

**Alternatives considered.**

| Alternative | Verdict | Reason |
|---|---|---|
| Direct `SerialRS485` calls | ❌ | Exactly PRR C-3. Untestable without hardware |
| C++ abstract base class + virtual dispatch | 🟡 | Clean, but `.ino` auto-prototype friction (CN-12) and vtable indirection on a hot byte loop. A plain function-pointer struct gives the same seam with no inheritance |
| Compile-time template policy | ❌ | Zero-cost, but unusable from an `.ino` translation unit and makes the fixture harness harder, not easier |
| Reuse `ModbusMaster` | ❌ | Structurally impossible: its response buffer is 64 registers / 128 B against a 6144 B payload, and K-5's `Len` byte violates the semantics it assumes. This is *why* the FIFO path must be separate — documented in RFC-0006 |

**Tradeoffs accepted.** One indirect call per `read()` — amortised over ~240 bytes per tick, immeasurable. The 8.3 ms blocking `write()` is accepted rather than made asynchronous: it is 3.3% of one 250 ms tick, occurs once per poll, and making it async would add a state for no benefit.

---

## 7. Public API

### 7.1 Surface

Nine entry points. Deliberately small — every additional entry point is another ordering constraint a caller can violate.

| # | Function | Caller | Blocking? | Purpose |
|---|---|---|---|---|
| A-1 | `FifoDriver_Init(cfg)` | `setup()`, Core 0 | brief | Bind transport, zero state, validate config. Once per boot |
| A-2 | `FifoDriver_Request(req, *outHandle)` | **any task, any core** | no | Submit a capture request. Returns admission verdict + handle |
| A-3 | `FifoDriver_Service()` | **`taskModbusRead` ONLY** | **never** | Advance the state machine by one bounded step |
| A-4 | `FifoDriver_GetPhase()` | any | no | Coarse phase for UI/telemetry: `IDLE`/`ACTIVE`/`RESULT_READY`/`COOLDOWN`/`DISABLED` |
| A-5 | `FifoDriver_OwnsBus()` | `taskModbusRead` | no | True while normal polling must stay suspended. **The bus-arbitration primitive** (§13) |
| A-6 | `FifoDriver_TryAcquireResult(*outResult)` | **exactly one call site**: `handleFifoCaptureCompletion()` (§9.1, `[v1.1]`) | no | Transfer result ownership to caller. Fails if none ready or already held |
| A-7 | `FifoDriver_ReleaseResult()` | the same call site | no | Return ownership. **Mandatory** — driver stays blocked until called |
| A-8 | `FifoDriver_Abort(reason)` | any | no | Request orderly termination; driver still performs its bus drain |
| A-9 | `FifoDriver_GetStats(*outStats)` | any | no | Lifetime counters snapshot |

### 7.2 Contracts

**A-3 `Service()` — the load-bearing contract.**

- **Precondition:** called only from `taskModbusRead`, exactly once per 250 ms tick.
- **Postcondition:** returns within a bounded budget. Design target **≤ 5 ms** worst case; typical **< 100 µs**.
- **Invariant:** contains no unbounded loop, no `delay()`, no `vTaskDelay()`, no wait on any synchronisation object.
- **Bound:** consumes at most `FIFO_SERVICE_MAX_BYTES = 512` bytes per call. This is 2.1× the 240 B/tick arrival rate, so it drains faster than arrival (allowing catch-up after a late tick) while capping worst-case execution.

This contract, mechanically enforced by review, is what satisfies **PRR C-1**.

**A-6/A-7 — the ownership contract.** `TryAcquireResult` succeeds at most once per capture. Until `ReleaseResult` is called the driver refuses new requests (`ERR_RESULT_NOT_RELEASED`). Deliberately strict: it makes "consumer forgot to release" a loud, immediate, diagnosable stall rather than a silent buffer race.

### 7.3 What is deliberately absent

| Absent | Why |
|---|---|
| `FifoDriver_CaptureBlocking()` | Would reintroduce PRR C-1. No convenience wrapper may exist |
| Direct accessors to sample arrays | Would reintroduce PRR C-2. Samples are reachable only via an acquired result |
| `FifoDriver_SetTimeout()` / any tuning setter | D-5. Field-tunable timeouts make failures irreproducible |
| `FifoDriver_SetSampleRate()` | D-3. The driver does not own sensor config |

---

## 8. Data Model

### 8.1 Request

| Field | Purpose |
|---|---|
| `triggerSource` | Enum: `FAULT_LATCH` / `OPERATOR_BUTTON` / `SCHEDULED` / `COMMISSIONING` — recorded in telemetry so every capture is attributable |
| `tag[16]` | Fixed-size correlation label. **Fixed array, not `String`** (D-2) |
| `requirePermissive` | If true, admission enforces the §19.2 gate set; false permits commissioning captures under relaxed conditions |
| `maxRetries` | Clamped to `FIFO_MAX_RETRIES` |
| `admissionContext` | **[v1.1, Freeze Review Finding 3.2]** Plain struct `{ bool motorStable; bool sensorHealthy; bool mqttReconnecting; }`, populated by the caller from globals it already has natural access to (`g_systemState`, `g_modbusConsecErrors`, MQTT state). `Request()`'s admission check (§19.2) reads only this field — the driver never references a `.ino` global directly, preserving L3's dependency direction and host-testability. Same pattern already used for provenance latching (§8.2's "why provenance is captured at request time"), extended to gate inputs — not a new pattern |

### 8.2 Result — the validity contract

> **D-8 — Sample data is reachable only through a result object whose `status` field is the sole authority on validity. A result with `status != OK` exposes no samples.**

| Field group | Fields |
|---|---|
| **Identity** | `captureId` (monotonic), `tag`, `triggerSource` |
| **Validity** | `status` (`OK` / `FAILED` / `ABORTED`), `error` (§16), `sampleCount` (0 unless `OK`) |
| **Provenance** | `srIndexAtCapture`, `srHz`, `tRequestMs`, `tCompleteMs`, `tempCAtCapture`, `motorStateAtCapture`, `rpmAtCapture`, `fwVersion`, `gitCommitHash` |
| **Payload** | `x`, `y`, `z` — const pointers into the driver's arena, **valid only while the result is held and `status == OK`** |
| **Diagnostics** | `pollCount`, `progressFrameCount`, `lastProgressFill`, `crcErrorCount`, `desyncBytesDiscarded`, `retryCount`, `tFirstByteMs`, `tFirstProgressMs`, `tLastProgressMs`, `tAnchorAfterLastProgressMs` |

**Why the diagnostics block.** It is the Validation Tool's timeline instrumentation model (Treq / T49 / T99 / TanchorNext / Tcomplete) — the one part of that project the PRR rated genuinely well designed — promoted from a debug print into a first-class telemetry schema. A field failure is then diagnosable from MQTT alone, with no serial console. This is the difference between a 63% success rate that is *mysterious* and one that is *characterised*.

**Why provenance is captured at request time, not publish time.** `srIndex`, temperature, motor state, and RPM can all change during a 6.4 s capture. Binding them at admission makes the result self-describing and reproducible.

### 8.3 Sample representation

> **D-9 — Samples are stored as raw big-endian-decoded `int16` counts. The driver never applies K-9's g scale factor.**

**Why.** K-9 is 🟡 evidence-backed, not ✅ confirmed (RFC-0007 §4 narrows the open question to *which DSP stage* the data represents). Applying an unconfirmed scale inside the driver would bake an unproven assumption into the acquisition layer and make it invisible downstream. Storing counts keeps the caveat where the consumer can see it, and halves memory versus `float` (6144 B vs 12288 B).

**Alternative considered:** store `float` g-values. Rejected — 2× memory, and it launders an unconfirmed assumption into apparent fact.

---

## 9. Buffer Ownership

### 9.1 Model

> **D-10 — One statically allocated arena, owned exclusively by the driver, transferred to exactly one consumer at a time under a strict handoff protocol.**

```
  ┌────────────┬────────────────────┬──────────────────┬──────────────┐
  │  Phase     │  Arena owner       │  Consumer access │  Mutability  │
  ├────────────┼────────────────────┼──────────────────┼──────────────┤
  │ IDLE       │  driver            │  none            │  —           │
  │ CAPTURING  │  driver            │  none            │  driver only │
  │ RESULT_RDY │  driver (frozen)   │  none until A-6  │  none        │
  │ HELD       │  consumer          │  read-only       │  none        │
  │ (A-7)      │  driver            │  none            │  —           │
  └────────────┴────────────────────┴──────────────────┴──────────────┘
```

**Zero copies.** The consumer reads the arena in place. For a 6144 B payload on a device where the Network task stack is already 24 KB, avoiding even one copy is worth the ownership discipline.

**[v1.1, Freeze Review Finding 2.1] "Consumer" names exactly one call site, `handleFifoCaptureCompletion()`.** Two application-level needs exist on the acquired result — publishing `/event` (§18.3) and, when Phase 7 is undertaken, persisting the waveform to FAT (§18.4). These are not two independent consumers each performing their own acquire/release; a second acquirer would either be rejected (result already held) or race a first acquirer's release. `handleFifoCaptureCompletion()` is the sole function that calls `TryAcquireResult`/`ReleaseResult`: it acquires once per completed session, invokes the telemetry-publish step and (when present) the waveform-store step synchronously, in that fixed order, using the same acquired pointers, then releases once. This adds no new public API — it is the application-level discipline the existing A-6/A-7 contract already required *someone* to follow; it names who.

### 9.2 Why the strict handoff

The Validation Tool's failure mode (PRR C-2) was that `g_fifoX/Y/Z` remained readable after a failed capture, still holding the *previous* successful capture — CRC-valid, plausible, and wrong. That is worse than a crash because it is invisible. D-10 makes it unrepresentable: there is no path to the samples that does not pass through a `status == OK` result.

**Alternatives considered.**

| Alternative | Verdict | Reason |
|---|---|---|
| Copy-on-acquire into consumer buffer | ❌ | +6144 B RAM and a 6 KB memcpy for no safety gain — ownership already guarantees exclusivity |
| Double-buffer (capture N+1 while N is held) | ❌ | +6144 B for a feature explicitly *dormant* under normal operation. Captures are minutes apart at minimum (§17.4 cooldown); concurrency has no use case |
| Reference counting | ❌ | Multiple simultaneous consumers are not a requirement. Refcounting adds a concurrency primitive and a leak mode to solve a problem CM-100 does not have |
| Keep globals, add a `valid` flag | ❌ | A flag anyone can read *and ignore* is not a contract. The Validation Tool effectively had this via its return value |

**Tradeoff accepted.** A consumer that fails to call `ReleaseResult` stalls all future captures. Chosen deliberately: a loud stall with a specific error code (`ERR_RESULT_NOT_RELEASED`) beats a silent race. A watchdog on hold duration (§17.5) converts even that into a logged, self-clearing event.

---

## 10. Memory Strategy

### 10.1 Budget

| Item | Size | Placement |
|---|---|---|
| Sample arena `x[1024] + y[1024] + z[1024]` (int16) | **6144 B** | `.bss`, static |
| Frame scratch (request 8 B, header/CRC 8 B) | 16 B | `.bss` |
| Result descriptor | ~112 B | `.bss` |
| Session state (SM, timers, counters) | ~96 B | `.bss` |
| Request queue (depth 1) | ~48 B | FreeRTOS static allocation |
| **Driver total** | **≈ 6.4 KB** | |
| RS485 RX buffer enlargement (§10.4) | +1792 B | driver-attributable |
| **Total attributable** | **≈ 8.2 KB** | |

**Against the Validation Tool's 18,944 B (PRR C-4, ELF-confirmed): a 66% reduction**, achieved by eliminating both staging buffers.

### 10.2 Eliminating the staging buffer

> **D-11 — Streaming decode. Bytes are CRC-accumulated and decoded into the sample arena as they arrive. No 6144 B staging buffer exists.**

**Why it works.** CRC-16/Modbus is byte-sequential — the running CRC can absorb each byte on arrival. K-8's layout is a fixed 6-byte stride, so the destination index of every byte is a pure function of its offset. Neither operation needs the whole frame present.

**The apparent objection, resolved.** If the CRC fails after decoding, the arena holds garbage. But D-8/D-10 make the arena unreachable unless `status == OK`, so garbage is never observable. **The ownership model is what makes the memory optimisation safe** — the two decisions are load-bearing for each other.

**Alternatives considered.**

| Alternative | Verdict | Reason |
|---|---|---|
| 6144 B staging + `memcpy` + bulk CRC | ❌ | The Validation Tool's design: +6144 B, +6 KB memcpy, zero benefit |
| `malloc` the staging buffer | ❌ | PRR C-5. Heap on hot path; OOM misreported |
| Stack-allocate staging | ❌ | CN-3: `STACK_SIZE_MODBUS` is 4096 B. Immediate overflow |
| Decode to `float` g-values | ❌ | 2× memory; violates D-9 |

### 10.3 Static-only allocation

> **D-12 — All driver memory is statically allocated at compile time. Heap use is zero, permanently.**

**Why.** CM-100's heap is contended by mTLS handshakes (RSA-2048), TinyGSM buffers, and a 2200 B JSON document. A 6 KB transient allocation every capture is a fragmentation source in a device expected to run for months. Static allocation also makes the cost visible in the linker map instead of appearing at runtime.

**Tradeoff accepted.** 6.4 KB of `.bss` is permanently resident even though the feature is dormant. Against 284,400 B free (build-verified) this is **2.2%** — the correct price for determinism. This is also precisely what "Internal Platform Infrastructure … dormant during normal operation" means: the cost is paid at link time, not at capture time.

### 10.4 RX buffer sizing — a hard prerequisite

> **D-13 — `SerialRS485.setRxBufferSize(2048)` must be called before `begin()` in `setup()`. This is a prerequisite of this design, not an optimisation.**

**Why.** CN-5: production currently uses the 256 B default.

```
  256 B ÷ 960 B/s  =  267 ms of headroom
  Service cadence  =  250 ms  (CN-1)
  Margin           =  17 ms  ≈ 6%  ← unacceptable; one late tick overflows
```

At 2048 B:

```
  2048 B ÷ 960 B/s = 2133 ms of headroom
  Margin over 250 ms cadence = 8.5×
```

An overflow here is silent byte loss → CRC failure → a retry that costs another 6.4 s. This single line converts a marginal design into a robust one.

**Note on scope.** This changes a line in `setup()` that exists today, which is why it is called out as a prerequisite rather than buried in the driver. It costs 1792 B and affects nothing else — normal Modbus responses are ≤ 37 B.

**Alternatives considered.** Increase `taskModbusRead` cadence to 100 ms during capture (would give 2.7× margin at 256 B) — rejected: it perturbs a proven, cadence-critical loop for a dormant feature, and `vTaskDelayUntil` phase would need re-establishing on exit. Enlarging a buffer is strictly less invasive.

---

## 11. Internal State Machine

### 11.1 States

| State | Meaning | Bus held? | Exit condition |
|---|---|---|---|
| `S0 UNINIT` | Pre-`Init` | no | `Init()` |
| `S1 IDLE` | Ready | no | Admitted request |
| `S2 ARMED` | Provenance latched, gates passed | **yes** | Next `Service()` |
| `S3 REQUEST` | Emit 8-byte request frame | yes | Write complete |
| `S4 AWAIT_ANCHOR` | Scan for `50 03` | yes | Anchor / desync limit / timeout |
| `S5 READ_TYPE` | Read the `Len` discriminator | yes | Byte received / timeout |
| `S6 READ_PROGRESS` | Consume 4 remaining progress bytes; validate `fill` against K-10 (§11.4) | yes | Complete / timeout / `ERR_PROGRESS_REGRESSION` / `ERR_PROGRESS_OVERRUN` |
| `S7 POLL_WAIT` | Inter-poll spacing | yes | `T_POLL_INTERVAL` elapsed |
| `S8 READ_DUMP` | Streaming consume + CRC + decode | yes | 6146 B / inter-byte timeout |
| `S9 VERIFY` | Compare CRC, finalise result | yes | Always (one tick) |
| `S10 DRAIN` | Bus-settle to true silence | yes | `T_DRAIN_QUIET` silence |
| `S11 RESULT_READY` | Awaiting `TryAcquireResult` | **no** | Acquire+Release / hold timeout |
| `S12 COOLDOWN` | Post-capture bus rest | no | `T_COOLDOWN` |
| `S13 FAILED` | Terminal failure, result finalised | yes → drain | → S10 |
| `S14 DISABLED` | Circuit breaker open | no | Manual reset / reboot |

### 11.2 Why this decomposition

Each state is **one bounded action per tick**. The decomposition is chosen so no state can require an unbounded wait — the property that mechanically guarantees A-3's contract and therefore PRR C-1.

Two states deserve specific justification:

**`S10 DRAIN` is mandatory and unconditional**, on both success and failure. K-14 documents that a FIFO transaction gone wrong cascades into `0xE2` on the *next unrelated* register read. Returning a dirty bus corrupts CM-100's core telemetry — far worse than losing the capture. The drain resets its quiet timer on every byte seen, so it exits only on *true* silence, not elapsed time.

**`S11 RESULT_READY` releases the bus but not the driver.** Bus ownership ends the moment the last byte is drained — normal 4 Hz polling resumes immediately, minimising telemetry loss. The driver stays occupied until the consumer releases, decoupling "bus is free" from "arena is free". These are genuinely different resources and conflating them would either hold the bus too long or free the arena too early.

### 11.3 Protocol-model parameterisation

> **D-14 — Whether `S6 READ_PROGRESS` returns to `S7 POLL_WAIT` (re-request — TRUEPOLL model) or to `S4 AWAIT_ANCHOR` (listen-only — PURELISTEN model) is a single named compile-time constant, `FIFO_PROTOCOL_MODEL`.**

**Why.** PRR blocker **BL-1** is unresolved: the Validation Tool built the deciding experiment but its results were never captured (zero `[TRUEPOLL-CSV]` lines in any of the 7 logs). Rather than guess, the design isolates the entire disagreement to **one transition edge**. When the experiment is run (§22 G-1), the answer is a one-line change and the rest of the design is unaffected.

This is the honest way to design under an open empirical question: make the unknown a parameter, name it, and document what evidence will close it. It is emphatically *not* a licence to ship both — exactly one value is compiled, and `TRUEPOLL` is the provisional default (it is the datasheet-literal reading, WTVB05 §6.1.4.16).

### 11.4 Frame-parsing state is explicit, not hidden

**[v1.1, Freeze Review Finding 3.1]** `FrameCodec_Step()` (the L2 function driving states `S4`–`S6`/`S8`) needs per-frame parsing state — anchor-scan bytes discarded so far, partial-frame bytes received so far. This state is an explicit `FrameCodecState` struct, defined by L2 (it's L2's own state shape) but **owned and allocated by the session** (L3) and passed by pointer into every call, reset at the start of each new frame attempt. No `static`/hidden state exists inside the codec. This preserves D-6 ("No hidden state") for the one file that most needs to honor it, and keeps L2 callable from independent, concurrent-in-principle test cases without a reset function nobody specified.

This state is distinct from, and must not be confused with, **cross-frame session state** — specifically `lastProgressFill`, which persists across multiple `S6` visits within one attempt (not reset per-frame) and is what `S6`'s K-10 validation (§11.1, §16.1) compares each new `fill` against. `lastProgressFill` already exists as a tracked/published diagnostic field (§8.2) — `S6` reuses it as the comparison anchor rather than introducing a second tracking variable.

---

## 12. Driver Lifecycle

### 12.1 Boot

1. `setup()` calls `SerialRS485.setRxBufferSize(2048)` **before** `begin()` (D-13).
2. `setup()` calls `FifoDriver_Init(cfg)` with the transport binding, after `modbus.begin()`.
3. Driver zeroes all state, validates config, enters `S1 IDLE`.
4. **No sensor I/O at init.** Presence is already established by CM-100's own boot probe; a redundant probe would be an unowned side effect (D-3).

### 12.2 Steady state

`taskModbusRead` calls `Service()` once per 250 ms tick, unconditionally, for the process lifetime. In `S1 IDLE` this is a state check costing tens of nanoseconds. Calling unconditionally — rather than only when busy — eliminates an entire class of "forgot to service" bug.

### 12.3 Capture

Admission → provenance latch → bus acquisition → protocol → verify → drain → release bus → handoff → cooldown. Detailed in §19 and §20.2.

### 12.4 Shutdown / reset

There is no orderly shutdown; CM-100 runs until reset. On reboot all state is re-zeroed. Any in-flight capture is lost by design — waveform captures are diagnostic, never safety-critical, and persisting partial state would add a failure mode with no benefit.

---

## 13. Bus Ownership and `taskModbusRead()` Integration

### 13.1 Decision

> **D-15 — `taskModbusRead()` remains the sole owner of the RS485 bus and the sole caller of `modbus.*`. The FIFO driver is a *mode* of that task, not a peer. No new mutex is introduced anywhere.**

This adopts **DESIGN-0002 §4 Option A**.

**Why.**

1. **It preserves the existing safety property by construction.** DESIGN-0002 §2.1 establishes that `modbus` is unsynchronised but safe *because it has exactly one caller*. Option A extends that invariant; Option B replaces it with a mutex that must be applied correctly at all 8 existing call sites plus every future one.
2. **It eliminates priority inversion.** Option B lets `taskModbusRead` (prio 5, Core 0, time-critical) block on a mutex held by a lower-priority task. DESIGN-0002 §4 assesses this blast radius as *strictly worse* than the original MQTT crash — that race reached only Core 1's non-critical tasks; this would reach the system's sole priority-5 task.
3. **It makes the jitter NFR enforceable by construction.** With one task sequencing both duties deterministically, "capture must not increase normal poll jitter beyond budget" is provable from the code, not dependent on mutex fairness.
4. **It touches no proven code.** `CLAUDE_RULES.md` §5 discourages modifying working code; Option B requires retrofitting all 8 existing transactions.
5. **CN-6 makes it more valuable than DESIGN-0002 anticipated.** The bus is already shared with the CTR4A01 current sensor via `modbus.begin()` re-targeting. A second bus owner would have to arbitrate *that* too. Option A gets it free.

**Tradeoff accepted.** `taskModbusRead`'s control flow gains a second mode, and the driver's progress is coupled to that task's liveness. Both are acceptable; the second is arguably a feature (§5.1).

**Status.** DESIGN-0002 §4 marks this decision *Critical — Blocks Approval* and requires explicit project-owner sign-off. This SDS states a recommendation with reasons; it does not constitute that sign-off. **Gate G-1, §22.**

### 13.2 Integration shape

`taskModbusRead()` gains one mode branch at the top of its existing 250 ms loop body:

```
  ┌─ every 250 ms tick (vTaskDelayUntil, unchanged) ────────────┐
  │                                                             │
  │   FifoDriver_Service();          ← always, unconditional    │
  │                                                             │
  │   if (FifoDriver_OwnsBus()) {                               │
  │       rs485Enable();            ← [v1.2/v1.3] EN bracketing,│
  │                                      see below               │
  │       // suspend normal polling                             │
  │       // do NOT touch modbus.*                              │
  │       // do NOT increment g_modbusConsecErrors   (CN-10)    │
  │       // publish capture-active state to snapshot           │
  │   } else {                                                  │
  │       ... existing WTVB02 + CTR4A01 polling, unchanged ...  │
  │       ...   (already brackets its own rs485Enable()/        │
  │       ...    rs485Disable() internally, unchanged)   ...    │
  │   }                                                         │
  │                                                             │
  │   if (bus ownership just released this tick) {              │
  │       rs485Disable();            ← [v1.2/v1.3] EN bracketing│
  │   }                                                         │
  │                                                             │
  └─────────────────────────────────────────────────────────────┘
```

**Why `Service()` runs before and outside the branch.** It must advance even when it does not own the bus (draining `S10`, timing `S12`, awaiting handoff `S11`). Placing it unconditionally at the top makes "the driver always gets exactly one step per tick" a structural property.

**Why `OwnsBus()` rather than a shared boolean.** `CLAUDE.md` forbids multi-writer shared booleans across cores. `OwnsBus()` is a single-reader accessor over driver-owned state, read only by the task that also drives it — no cross-core write path exists.

### 13.2.1 RS485 EN-pin ownership `[v1.3, Architecture Review Feedback — see ADR-1; D-15 itself is unchanged]`

> **D-15 is unchanged (§13.1).** Its text says only that `taskModbusRead()` remains the sole owner of the RS485 bus and the sole caller of `modbus.*`. Whether "owning the bus" is read as including EN-pin assertion/deassertion is a separate, implementation-level interpretation question, not part of D-15's own wording — recorded as its own decision, **`ADR-1` — "Bus ownership includes EN ownership"** (`CM-100_ADR-1_Bus_Ownership_Includes_EN_Ownership_v1.0.md`), rather than folded into D-15.
>
> **ADR-1's conclusion, applied here:** `taskModbusRead()`'s `OwnsBus()` boundary calls `rs485Enable()`/`rs485Disable()` around a FIFO capture exactly as it already calls them around every normal polling burst (verified production call sites: `:4334/:4425`, `:4571/:4584`, `:4680`) — a FIFO capture is one more bus-ownership window, bracketed the same way, by the same owner.

**Why this interpretation, not the transport.** Every existing bus-window user in production treats "acquire the bus" as including "assert EN" and "release the bus" as including "deassert EN" — the pairing is never split across two different actors anywhere in the current codebase. `taskModbusRead()` is already the actor that decides window boundaries (`OwnsBus()` transitions); making it also the actor that brackets EN keeps that pairing intact for the FIFO window instead of introducing the first exception to it. Full alternatives analysis is in ADR-1.

**How this guarantees EN is asserted before any FIFO transmission.** By construction of the state machine's own control flow, not by a runtime check in the transport. `Uart485Transport_Write()` is first reachable at state `S3 REQUEST`, which is only reachable after `S2 ARMED`, which is only reachable after `FifoDriver_OwnsBus()` has already become `true` (§11.1's state table) — and `rs485Enable()` is asserted at that same `OwnsBus()` boundary, unconditionally, every tick the branch is taken (mirroring normal polling's own unconditional-reassert-every-cycle pattern, so there is no dependency on catching one specific transition tick). `write()` can therefore simply transmit.

**`Uart485Transport` never touches this pin.** No `digitalWrite`, no `RS485_EN_PIN` reference, no duplicated constant, anywhere in `fifo_transport_uart485.{h,cpp}`. This removes the one disclosed liability the original Task 1.2 implementation carried (a locally-duplicated pin constant, forced by `rs485Enable()`/`rs485Disable()`'s `static inline` linkage preventing reuse across translation units) — Task 4.2's code lives inside the `.ino` itself and calls the real functions directly.

**New requirement on Task 4.2, not present in the original integration shape.** `rs485Disable()` must be called exactly once when bus ownership releases, **on every path** — normal completion (`S11`→cooldown), abort (`FifoDriver_Abort()`), and the circuit breaker tripping (`S14`) all release the bus and must all reach the disable call. Task 4.2 must not assume only the "happy path" releases ownership.

### 13.3 Consequences that must be handled

| Consequence | Handling | Constraint |
|---|---|---|
| Vibration telemetry gaps ~7 s | Set `capture_active` in the telemetry snapshot; Analytics treats the gap as *suspended-by-design*, reusing the existing v16.3aa FREEZE semantics | Data-quality concern, not a motor-FSM safety constraint — see CN-9 `[v1.1]` and RPM continuity below |
| Modbus offline false-positive | `g_modbusConsecErrors` **must not** be incremented while `OwnsBus()` | CN-10: 3 strikes would falsely mark the sensor offline. This *is* the genuinely Modbus-coupled timing constraint in this table |
| RPM continuity | Unaffected — RPM comes from the `PIN_RPM` pulse ISR, independent of Modbus | Motor state machine keeps running throughout; this is *why* `ABSENT_STOPPING_MS` does not constrain capture duration (§15.3 `[v1.1]`) |
| CTR4A01 current polling gap | Same ~7 s gap; current evidence marked stale via the existing P4-02 validity mechanism | Reuses shipped infrastructure |
| Trend/analytics continuity | ~7 s at a 1 s analytics cadence = ~7 samples. Existing `trend_gap_s` (v16.3aa) already models this | No new mechanism |
| Display | Show a capture indicator so a technician does not read the freeze as a fault | UX, not correctness |

**Why suspension rather than interleaving.** Once the sensor begins the dump it transmits 6146 bytes continuously (K-13: ≤2 ms inter-byte gaps). There is no window in which another Modbus transaction could be issued. Suspension is not a design choice — it is what the protocol permits. The choice is only whether to *acknowledge* it honestly downstream; this design does.

---

## 14. Thread Safety

### 14.1 Model

> **D-16 — Single-writer by construction. All mutable capture state is written only by `taskModbusRead` (Core 0). Cross-core interaction is confined to two primitives: one FreeRTOS queue and one mutex-guarded handoff.**

| Object | Writer | Readers | Mechanism |
|---|---|---|---|
| Session state, timers, counters | `taskModbusRead` only | — | None needed — single writer |
| Sample arena | `taskModbusRead` during `S8` | consumer while `HELD` | Ownership handoff (§9) — phases are disjoint in time |
| Request submission | any task, any core | `taskModbusRead` | `xQueueSend` / `xQueueReceive`, depth 1 |
| Result handoff | `taskModbusRead` | one consumer | `mutexFifoResult`, held for pointer exchange only |
| `phase` (A-4), `OwnsBus()` (A-5) | `taskModbusRead` | any | Single 32-bit word — atomic on Xtensa per `CLAUDE.md` |
| Lifetime stats | `taskModbusRead` | any | Individual 32-bit counters, atomic single-word reads |

### 14.2 Rationale

This follows `CLAUDE.md`'s existing cross-core rules exactly: *"Core0→Core1 use command enum / atomic — never a shared boolean with multiple writers; float/enum single-word are atomic on Xtensa; structs must go through mutex/queue."* Requests are structs → queue. Phase is a single word → atomic. Nothing here invents a new concurrency pattern, which is the point.

**Why a depth-1 request queue.** Captures are minutes apart (§17.4 cooldown); a deeper queue would let stale requests accumulate and fire long after their triggering condition passed. Depth 1 plus an explicit `ERR_BUSY` rejection makes back-pressure visible to the caller.

**Why the result mutex is held only for a pointer exchange.** Holding a mutex across consumer processing would let a Core-1 consumer block Core-0's `taskModbusRead` — reintroducing exactly the priority inversion D-15 was chosen to avoid. The mutex guards the *transfer*, never the *use*.

**Alternatives considered.**

| Alternative | Verdict | Reason |
|---|---|---|
| Mutex around all driver state | ❌ | Unnecessary with a single writer; creates the inversion path D-15 avoids |
| Lock-free ring for results | ❌ | Only one result can exist at a time (§9). A ring solves a problem that cannot occur |
| Critical sections around arena access | ❌ | Would block interrupts for 6144-byte spans. Ownership discipline achieves exclusivity with zero interrupt latency |

### 14.3 Reentrancy

`Service()` is **not** reentrant and must never be called from an ISR, from more than one task, or recursively. This is stated as a hard API precondition (§7.2) rather than defended with a guard, because a guard would silently mask a serious integration error. A debug-build assertion on caller task handle is specified for development.

---

## 15. Timeout Strategy

### 15.1 Budgets

> **D-17 — Every timeout is a named constant with a documented derivation from a §2.1 measurement. No timeout is a parameter, none is adaptive, and none is silently overridden.**

| Constant | Value | Derivation |
|---|---|---|
| `T_REQUEST_RESPONSE_MS` | **1000** | K-12: observed 256–260 ms → ~3.8× margin |
| `T_INTER_BYTE_MS` | **500** | K-13: ≤2 ms mid-dump → 250× margin; catches a truncated stream fast |
| `T_POLL_INTERVAL_MS` | **250** | Aligned to CN-1 so a poll is issued on a tick boundary — no extra state |
| `T_DUMP_TOTAL_MS` | **9000** | K-11: 6402 ms + 40% margin for a late tick |
| `T_CAPTURE_TOTAL_MS` | **20000** | Request + polls + dump + drain, with retry headroom. Value unchanged in v1.1; its justification is corrected in §15.3 (was incorrectly tied to `ABSENT_STOPPING_MS`) |
| `T_DRAIN_QUIET_MS` | **300** | Validation Tool's empirically effective bus-settle value; the one operational constant inherited |
| `T_COOLDOWN_MS` | **60000** | §17.4 |
| `T_RESULT_HOLD_MAX_MS` | **30000** | §17.5 watchdog on a delinquent consumer |
| `MAX_DESYNC_BYTES` | **64** | 2× the Validation Tool's 32, sized to skip one full progress frame plus slack |

### 15.2 Why separate budgets

The Validation Tool had one `silenceTimeoutMs` parameter — and then silently ignored it mid-dump, hardcoding 4000 (PRR C-9). Any experiment run with a large `stallMs` was therefore measuring something other than what the operator believed. Separate, named, non-overridable budgets make that class of error impossible: there is no parameter to override.

**Why "no adaptive timeouts".** Adaptive timing makes failures irreproducible and destroys the statistical picture BL-1/G-1 needs. A fixed budget that fails is data; an adaptive budget that fails is noise.

### 15.3 Interaction with the motor state machine — corrected in v1.1

**[Freeze Review Finding 5.1]** The original text of this section argued that `T_CAPTURE_TOTAL_MS = 20000` exceeding `ABSENT_STOPPING_MS = 15000` required per-attempt bus release to avoid the motor FSM seeing a false "sensor absence." That argument is **wrong** and is corrected here, not merely footnoted, because a future engineer could otherwise trust it when adjusting these constants.

`ABSENT_STOPPING_MS`'s own source comment reads `signalPresent false (but fresh) > 15s -> STOPPING`. `signalPresent` is derived from RPM pulse-ISR evidence at the EMA update site (`buildMotorStateEvidence()`), not from anything `taskModbusRead()` does with `SerialRS485`. The `PIN_RPM` interrupt keeps firing regardless of whether the driver currently owns the WTVB05 bus — this is exactly what §13.3's "RPM continuity: unaffected, independent of Modbus" already stated, three sections before §15.3 contradicted it. A FIFO capture of any length, single-attempt or across retries, does **not** risk triggering `ABSENT_STOPPING_MS`, because that timer is not watching the resource this driver suspends.

**What the retry-release-between-attempts rule is actually justified by** (both reasons already stated correctly elsewhere in this document, now the sole justification rather than a secondary one):

- **Telemetry availability.** Releasing the bus between attempts lets normal WTVB05/CTR4A01 polling resume for at least one cycle, reducing the wall-clock proportion of a multi-attempt session during which core vibration/current telemetry is stale — a data-quality concern, not a safety one.
- **Sensor-state settling.** Per K-15 and RFC-0007, whether the sensor's own internal state benefits from an idle period between requests is an open empirical question; releasing between attempts keeps that possibility available rather than foreclosing it with back-to-back re-requests.

**Consequence for the design.** No numeric constant changes — `T_CAPTURE_TOTAL_MS = 20000`, `FIFO_MAX_RETRIES = 2`, and the retry-release requirement (§17.3) are all retained exactly as specified, because none of them depended on the false premise for their *value*, only for one sentence of *justification*. `MODBUS_OFFLINE_THRESHOLD`/CN-10 (§13.3's table) remains the one genuinely Modbus-coupled timing constraint in this area, and is handled separately and correctly (not incrementing `g_modbusConsecErrors` while `OwnsBus()`) — unaffected by this correction.

---

## 16. Error Model

### 16.1 Taxonomy

> **D-18 — Errors are classified by *domain* and *recovery policy*, never merged. Each code names one distinct condition.**

**Transport domain** — the link misbehaved.

| Code | Condition | Policy |
|---|---|---|
| `ERR_NO_RESPONSE` | No byte within `T_REQUEST_RESPONSE_MS` | `RETRY_POLL` |
| `ERR_INTER_BYTE_TIMEOUT` | Stream stopped mid-frame | `RESYNC` |
| `ERR_DESYNC_LIMIT` | > `MAX_DESYNC_BYTES` discarded without anchor | `ABORT_ATTEMPT` |
| `ERR_RX_OVERFLOW` | Transport reports lost bytes, via `FifoTransport.hadOverflow()` (§6.1 `[v1.1]`) | `ABORT_ATTEMPT` |

**Protocol domain** — bytes arrived; the sensor's behaviour was invalid.

| Code | Condition | Policy |
|---|---|---|
| `ERR_CRC_MISMATCH` | Computed ≠ transmitted CRC | `RETRY_POLL` |
| `ERR_BAD_TYPE_BYTE` | `Len` ∉ {`0x00`,`0x01`} | `RESYNC` |
| `ERR_PROGRESS_REGRESSION` | `fill` decreased vs. `lastProgressFill`, checked in `S6` (§11.1, §11.4 `[v1.1]`) | `ABORT_ATTEMPT` — violates K-10; indicates sensor state reset |
| `ERR_PROGRESS_OVERRUN` | `fill` > 6144, checked in `S6` (§11.1, §11.4 `[v1.1]`) | `ABORT_ATTEMPT` — RFC-0006 flags this as a hypothesis-invalidating observation |

**Lifecycle domain** — the driver or its caller.

| Code | Condition | Policy |
|---|---|---|
| `ERR_BUSY` | Capture already active | reject at admission |
| `ERR_NOT_PERMITTED` | §19.2 gate failed | reject at admission |
| `ERR_RESULT_NOT_RELEASED` | Previous result still held | reject at admission |
| `ERR_ABORTED` | `Abort()` called | terminal |
| `ERR_RETRY_EXHAUSTED` | Budget spent | terminal |
| `ERR_CIRCUIT_OPEN` | Breaker tripped (§17.6) | reject at admission |

### 16.2 Rationale

The Validation Tool returned `CRC_ERROR` for a genuine CRC mismatch, a `malloc` failure, *and* an unrecognised `Len` byte (PRR C-7) — three conditions demanding retry, abort, and resync respectively. All callers applied one policy. In the field this makes bus corruption and framing desync indistinguishable in the log.

Two structural properties follow from D-18:

1. **`ERR_PROGRESS_REGRESSION` and `ERR_PROGRESS_OVERRUN` did not exist in the Validation Tool.** They encode K-10 as an active runtime invariant. RFC-0006 explicitly identifies a `fill` value exceeding 6144 as evidence that would invalidate the current protocol hypothesis — detecting it in production turns a research question into a field-reportable event.
2. **There is no `ERR_OUT_OF_MEMORY`.** D-12 makes it unrepresentable. Removing an error code by removing its cause is strictly better than handling it.

### 16.3 Every error is reported

A failed capture still produces a result object with full diagnostics (§8.2) and is still published (§18.3). **Failures are the more valuable telemetry** — a 63% success rate (K-15) means failures are the common case, and they are exactly what G-1 needs characterised.

---

## 17. Recovery Strategy

### 17.1 Layered recovery

```
  Byte level    →  RESYNC        bounded anchor rescan (MAX_DESYNC_BYTES)
  Frame level   →  RETRY_POLL    re-issue request within the same attempt
  Attempt level →  RETRY_ATTEMPT full drain + bus release + fresh attempt
  Session level →  FAIL          terminal, result published with diagnostics
  Feature level →  CIRCUIT_OPEN  breaker disables captures until reset
```

### 17.2 Bounded resync

Anchor rescan discards at most `MAX_DESYNC_BYTES` (64), then fails with `ERR_DESYNC_LIMIT`.

**Why bounded.** PRR C-6: the Validation Tool's shared frame reader scanned indefinitely. Since the payload is raw acceleration data, the pair `0x50 0x03` occurs by chance roughly once per 65,536 byte positions — non-negligible across 6144 bytes over repeated captures. An unbounded scanner mid-payload can anchor on data, read a garbage type byte, and (in the tool) return `CRC_ERROR`, which callers read as "keep listening" — perpetuating the desync. The bound converts an unbounded misbehaviour into a fast, named, recoverable failure.

**Why 64.** Large enough to skip one complete 7-byte progress frame plus slack; small enough that a false anchor is detected within ~67 ms at 960 B/s.

### 17.3 Retry with mandatory bus release

Between attempts the driver **must** drain to silence, release the bus, and allow at least one normal poll tick.

**Why.** Two reasons, per the corrected §15.3 (`[v1.1]`): it lets CM-100's core telemetry breathe rather than being starved by a retry storm; and it gives the sensor's own state machine an unambiguous idle period — which, per K-15 and RFC-0007, may itself be a factor in the observed failure rate. (§15.3's original text also cited an `ABSENT_STOPPING_MS` safety constraint; that citation was incorrect and has been removed — the timer is RPM-derived and independent of Modbus bus state.)

`FIFO_MAX_RETRIES = 2` (3 attempts total). At ~7.5 s per attempt plus release, a fully-failing session costs ~25 s of degraded telemetry. Beyond 3 attempts the marginal recovery probability does not justify further core-function degradation — and at SR6, where the measured rate is 17%, more retries would mean minutes of degradation for a dormant diagnostic feature.

### 17.4 Cooldown

`T_COOLDOWN_MS = 60000` after any session, success or failure.

**Why.** Enforces "dormant during normal operation" as a runtime property rather than a documentation claim. It bounds worst-case telemetry impact to ~25 s in any 85 s window (≈29%) under total failure, and to ~7 s in 67 s (≈10%) on success. It also prevents a stuck trigger source from turning a diagnostic aid into a denial of service against the product's actual function.

### 17.5 Result-hold watchdog

If a result is held longer than `T_RESULT_HOLD_MAX_MS` (30 s), the driver logs `ERR_RESULT_NOT_RELEASED`, reclaims ownership, and returns to `IDLE`.

**Why.** §9.2 accepts a strict handoff whose failure mode is a permanent stall. The watchdog converts that into a logged, self-clearing, diagnosable event. The consumer's pointer is invalidated on reclaim — a documented precondition, and the reason consumers must copy anything they need beyond their hold window.

### 17.6 Circuit breaker

`FIFO_BREAKER_THRESHOLD = 5` consecutive failed sessions → `S14 DISABLED`. All requests rejected with `ERR_CIRCUIT_OPEN` until reboot or an explicit reset. A single high-priority `/event` is published on trip.

**Why.** If the sensor's FIFO state machine has genuinely wedged (the hypothesis RFC-0007 leaves open and G-1 must settle), continued attempts cost real telemetry availability for zero diagnostic value. The breaker makes that degradation self-limiting and loudly visible instead of a slow, silent decline in data quality.

**Why 5, and why manual reset.** Five consecutive failures against a 63% baseline has probability ≈ 0.7% — unlikely to be chance, so it is strong evidence of a real fault. Manual reset ensures a human sees the event; auto-reset would let a persistent hardware fault oscillate indefinitely.

---

## 18. Telemetry Interface

### 18.1 Backward compatibility

> **D-19 — Telemetry is purely additive. No existing field on any of the 5 topics changes meaning, type, or presence.**

Mandated by `CLAUDE.md` and `RFC-0002`.

### 18.2 Capture-active indication (`/sensor`, `/vibration`)

One additive boolean, `capture_active`, in the telemetry snapshot and its published payloads.

**Why.** Without it, a 7 s freeze in vibration data is indistinguishable from a sensor fault — both to cloud analytics and to a technician. This field is what makes the gap *explainable* rather than *alarming*, and it reuses the semantics already established by the v16.3aa FREEZE / `trend_gap_s` work rather than inventing new ones.

### 18.3 Capture event (`/event`)

Every session — success **and** failure — publishes one additive event containing identity, outcome, provenance, and the full §8.2 diagnostics block. **[v1.1]** This publish is the first internal step of `handleFifoCaptureCompletion()` (§9.1) — it executes between that function's single `TryAcquireResult` and `ReleaseResult`, not as an independent acquirer.

**Why every session.** At a 63% success rate (K-15), failures are the common case and the more informative one. Publishing only successes would produce survivorship-biased field data and leave G-1 permanently unanswerable from deployed units. The event schema is deliberately the Validation Tool's timeline model promoted to production telemetry — the one artefact of that project the PRR rated genuinely well designed.

**Size.** Metadata + diagnostics ≈ 400–500 B JSON — comparable to existing `/event` payloads, well inside the 2200 B document budget.

### 18.4 Waveform payload — deliberately deferred

> **D-20 — The driver does not transmit the 6144-byte payload. It exposes it via the §9 ownership protocol. Egress is a separate decision, deferred, with the recommendation below.**

**Why deferred.** Egress mechanism is a product and cloud-contract decision, not a driver decision. Binding it into the driver would couple acquisition to transport and make the driver unusable for any other consumer.

**Options, assessed:**

| Option | Cost | Assessment |
|---|---|---|
| **A. Store to the existing 9 MB FAT partition; publish metadata + a reference** | ~6 KB/capture; ~1500 captures | ✅ **Recommended.** CN-11: the partition is *already provisioned* by the production scheme `app3M_fat9M_16MB`. Zero incremental cost, no 4G airtime, survives reboot, retrievable at service time |
| **B. Base64 over MQTT** | ~8.2 KB payload + TLS overhead | 🟡 Viable but expensive on metered 4G, and 8.2 KB exceeds the current JSON document budget — would need chunking |
| **C. Decimated / summary only** | ~500 B | ❌ Discards exactly the raw detail the capture exists to obtain |
| **D. Discard after in-device analysis** | 0 | ❌ CM-100 has no waveform analysis, and RFC-0007 §4 leaves DSP-stage identity open — in-device interpretation would be premature |

Option A is recommended because it is the only one that costs nothing new: the storage already exists in the shipped partition table, unused. **[v1.1]** When undertaken, storage becomes the *second* internal step of `handleFifoCaptureCompletion()` (§9.1), added after the `/event` publish step within the same acquire/release pair — not a second, independent acquirer of the result.

---

## 19. Diagnostic Workflow

### 19.1 Trigger sources

| Source | Origin | Status |
|---|---|---|
| `FAULT_LATCH` | Existing fault-latch transition | ✅ Available today |
| `OPERATOR_BUTTON` | Long-press on the existing ENTER button | ✅ Available today |
| `SCHEDULED` | Periodic baseline (e.g. weekly) | ✅ Available today |
| `COMMISSIONING` | Boot-time Config Mode (5 s ENTER window) | ✅ Available today |
| `REMOTE_ON_DEMAND` | Cloud command | ❌ **No inbound channel exists** — §22 G-2 |

### 19.2 Admission gates

All must pass, else `ERR_NOT_PERMITTED` (unless `requirePermissive == false`):

| Gate | Condition | Source | Why |
|---|---|---|---|
| Driver idle | `S1 IDLE` | driver-internal | One capture at a time (§9) |
| Circuit closed | not `S14` | driver-internal | §17.6 |
| Cooldown elapsed | ≥ `T_COOLDOWN_MS` | driver-internal | §17.4 |
| No result held | §9 handoff complete | driver-internal | §7.2 |
| Motor state stable | `RUNNING` steady, past `RUNNING_WARMUP_MS` | `req.admissionContext.motorStable` `[v1.1]` | A waveform captured mid-transition is not comparable to any baseline |
| Sensor healthy | `g_modbusConsecErrors == 0` | `req.admissionContext.sensorHealthy` `[v1.1]` | Do not attempt a 6 KB transfer on a bus already failing |
| Network state | not mid-MQTT-reconnect | `req.admissionContext.mqttReconnecting` (inverted) `[v1.1]` | Avoids compounding two multi-second activities |

**[v1.1, Freeze Review Finding 3.2]** The four driver-internal gates read the driver's own state directly, as they always did. The three application-state gates read only `req->admissionContext` (§8.1) — populated by the caller before calling `Request()`. `Request()`'s implementation never references `g_systemState`, `g_modbusConsecErrors`, or MQTT connection state directly; this keeps L3 dependent only on plain data, testable without any `.ino` global existing.

**Why gate on motor state.** The capture's diagnostic value depends entirely on comparability. A capture taken during spin-up describes a transient, not the machine's condition, and would pollute any baseline built from it. Rejecting is better than capturing something misleading — the same principle as the v16.3aa analytics FREEZE.

### 19.3 Workflow

```
  trigger → Request(A-2) → gates(§19.2) → admit
     → provenance latch (SR, temp, motor state, RPM)
     → acquire bus  ──────────────────── telemetry suspended, capture_active=true
     → protocol (§11) [retry ≤ 2, bus released between attempts §17.3]
     → verify → drain to silence(§11.2)
     → release bus  ──────────────────── telemetry resumes, capture_active=false
     → handleFifoCaptureCompletion() [v1.1, §9.1]:
          TryAcquireResult → publish /event §18.3 → store to FAT §18.4 → ReleaseResult
     → cooldown 60 s §17.4
```

---

## 20. Diagrams

### 20.1 Class / module diagram

```
┌───────────────────────────────────────────────────────────────────────┐
│                          taskModbusRead  (Core 0, prio 5)             │
│                                                                       │
│   owns: SerialRS485 · ModbusMaster modbus · RS485 direction pin       │
│   calls: FifoDriver_Service()   ← once per 250 ms tick, unconditional │
│   reads: FifoDriver_OwnsBus()   ← suspends its own polling when true  │
└───────────────┬───────────────────────────────────────────────────────┘
                │ injects (once, at Init)
                ▼
┌───────────────────────────────────────────────────────────────────────┐
│  «interface»  FifoTransport                                    (L1)   │
│  ─────────────────────────────────────────────────────────────────    │
│  + ctx : void*                                                        │
│  + read(dst,maxLen) → len      ◄── NON-BLOCKING, may return 0         │
│  + write(src,len) → len                                               │
│  + available() → count                                                │
│  + flushRx()                                                          │
│  + nowMs() → uint32                                                   │
│  + hadOverflow() → bool        ◄── [v1.1] incidental fix: was missing │
│                                     from this box, present in §6.1     │
│                                                                       │
│   ▲ implemented by                     ▲ implemented by               │
│   │                                    │                              │
│  Uart485Transport (production)        LogReplayTransport (host test)  │
│   └─ wraps SerialRS485                 └─ replays recorded byte       │
│      pure byte I/O only ── [v1.2/v1.3]    streams from serial_log*.txt│
│      NO RS485_EN_PIN reference --                                     │
│      EN owned by taskModbusRead's                                     │
│      OwnsBus() boundary, §13.2                                        │
└───────────────────────────────────────────────────────────────────────┘
                ▲
                │ uses (never owns)
┌───────────────┴───────────────────────────────────────────────────────┐
│  FifoDriver                                                    (L3)   │
│  ─────────────────────────────────────────────────────────────────    │
│  - transport   : FifoTransport                                        │
│  - state       : FifoState  (S0..S14)                                 │
│  - session     : FifoSession   (timers, retry budget, provenance,     │
│                   lastProgressFill, FrameCodecState ◄── §11.4 [v1.1]) │
│  - arena       : FifoArena     ◄── 6144 B static, §9                  │
│  - result      : FifoCaptureResult                                    │
│  - stats       : FifoDriverStats                                      │
│  - breaker     : FifoBreaker                                          │
│  ─────────────────────────────────────────────────────────────────    │
│  + Init(cfg)                     + TryAcquireResult(**out) : bool     │
│  + Request(req,*h) : Verdict     + ReleaseResult()                    │
│  + Service()          ◄─ bounded + Abort(reason)                      │
│  + GetPhase() : Phase            + GetStats(*out)                     │
│  + OwnsBus() : bool                                                   │
└───────────────┬───────────────────────────────────────────────────────┘
                │ composes
     ┌──────────┴──────────┬────────────────────┬──────────────────┐
     ▼                     ▼                    ▼                  ▼
┌──────────────┐  ┌─────────────────┐  ┌───────────────┐  ┌──────────────┐
│ FrameCodec   │  │ StreamingCrc16  │  │ SampleDecoder │  │ FifoArena    │
│      (L2)    │  │      (L2)       │  │      (L2)     │  │      (§9)    │
│              │  │                 │  │               │  │              │
│ shape        │  │ poly 0xA001     │  │ K-8: BE int16 │  │ x[1024]      │
│ discriminate │  │ init 0xFFFF     │  │ stride 6      │  │ y[1024]      │
│ (K-5)        │  │ byte-sequential │  │ X,Y,Z order   │  │ z[1024]      │
│ bounded      │  │ NO buffering    │  │ NO scaling    │  │ owner-gated  │
│ resync (§17) │  │ NO malloc       │  │ (D-9)         │  │ (D-10)       │
│              │  │                 │  │               │  │              │
│ takes plain  │  │                 │  │ writes to     │  │              │
│ int16_t*     │  │                 │  │ plain int16_t*│  │              │
│ x/y/z out    │  │                 │  │ targets, NOT  │  │              │
│ params ◄──── │  │                 │  │ FifoArena* ── │  │              │
│ [v1.1] no    │  │                 │  │ [v1.1] no L3  │  │              │
│ FifoArena dep│  │                 │  │ type dep      │  │              │
│              │  │                 │  │               │  │              │
│ state: caller│  │                 │  │               │  │              │
│ -owned Frame │  │                 │  │               │  │              │
│ CodecState,  │  │                 │  │               │  │              │
│ passed in ── │  │                 │  │               │  │              │
│ [v1.1] §11.4 │  │                 │  │               │  │              │
│              │  │                 │  │               │  │              │
│ PURE LOGIC   │  │ PURE FUNCTION   │  │ PURE LOGIC    │  │ static .bss  │
│ host-testable│  │ host-testable   │  │ host-testable │  │              │
└──────────────┘  └─────────────────┘  └───────────────┘  └──────────────┘
        ▲ L3 (FifoDriver) bridges L2 to FifoArena: passes
          FifoArena_WriteHandleX/Y/Z() as the plain pointers
          above. L2 never #includes fifo_arena.h.  [v1.1]

┌───────────────────────────────────────────────────────────────────────┐
│  Consumers                                                     (L4)   │
│   TriggerManager (DESIGN-0001) ──── Request() ──────────►             │
│                                                                       │
│   handleFifoCaptureCompletion()  ◄── the ONE call site for            │
│    ── TryAcquireResult() ───────────────────────────────►             │
│    ── publish /event (TelemetryPublisher step) ─────────  [v1.1]      │
│    ── store to FAT (WaveformStore step, when present) ──              │
│    ── ReleaseResult() ──────────────────────────────────►             │
│   (see §9.1: not two independent consumers — one coordinator,         │
│    two internal steps, single acquire/release pair)                   │
└───────────────────────────────────────────────────────────────────────┘
```

### 20.2 Sequence — successful capture

```
Trigger    FifoDriver   taskModbusRead   Transport    Sensor      Consumer
   │            │              │             │           │            │
   │─Request()─►│              │             │           │            │
   │            │ gates §19.2  │             │           │            │
   │            │ latch prov.  │             │           │            │
   │◄─ADMITTED──│              │             │           │            │
   │            │ S1→S2 ARMED  │             │           │            │
   │            │              │             │           │            │
   │            │◄──Service()──│ tick 1      │           │            │
   │            │ S2→S3        │             │           │            │
   │            │──write(8 B)─────────────► │──request─►│            │
   │            │ S3→S4        │             │           │            │
   │            │              │ OwnsBus()=true ─── normal polling OFF│
   │            │              │             │           │            │
   │            │◄──Service()──│ tick 2      │  (260 ms latency K-12) │
   │            │ read()→0     │             │           │            │
   │            │              │             │           │            │
   │            │◄──Service()──│ tick 3      │           │            │
   │            │──read()────────────────► │◄──50 03 01──│            │
   │            │ anchor·type·progress      │           │            │
   │            │ S4→S5→S6→S7  │  fill=2277 │           │            │
   │            │              │             │           │            │
   │            │  ... POLL_WAIT / re-poll per D-14 ...  │            │
   │            │              │             │           │            │
   │            │◄──Service()──│ tick k      │           │            │
   │            │──read()────────────────► │◄──50 03 00──│  DUMP      │
   │            │ S5→S8 READ_DUMP           │           │  begins    │
   │            │              │             │           │            │
   │            │◄──Service()──│ tick k+1    │           │            │
   │            │──read(≤512)──────────────►│◄─240 B────│            │
   │            │ crc.update() + decode→arena           │            │
   │            │              │             │           │            │
   │            │      ⋮  ~26 ticks ≈ 6402 ms (K-11)  ⋮ │            │
   │            │              │             │           │            │
   │            │◄──Service()──│ tick k+26   │           │            │
   │            │ 6146 B complete → S9 VERIFY           │            │
   │            │ crc.final() == transmitted ✓          │            │
   │            │ status=OK, sampleCount=1024           │            │
   │            │ S9→S10 DRAIN │             │           │            │
   │            │              │             │           │            │
   │            │◄──Service()──│ tick k+27   │           │            │
   │            │ 300 ms true silence → S11 RESULT_READY │            │
   │            │              │             │           │            │
   │            │              │ OwnsBus()=false ── normal polling ON │
   │            │              │             │           │            │
   │            │        handleFifoCaptureCompletion() [v1.1, §9.1] — sole call site
   │            │◄──────────────── TryAcquireResult() ────────────────│
   │            │───── result* (status=OK, x/y/z valid) ─────────────►│
   │            │              │             │           │ publish    │
   │            │              │             │           │ /event     │
   │            │              │             │           │ §18.3      │
   │            │              │             │           │ store→FAT  │
   │            │              │             │           │ §18.4      │
   │            │◄──────────────── ReleaseResult() ───────────────────│
   │            │ S11→S12 COOLDOWN (60 s)    │           │            │
```

### 20.3 Sequence — CRC failure, retry, success

```
FifoDriver   taskModbusRead   Transport    Sensor
     │              │             │           │
     │  ... S8 READ_DUMP completes, 6146 B ...│
     │ S8→S9 VERIFY │             │           │
     │ crc.final() ≠ transmitted  ✗           │
     │ error=ERR_CRC_MISMATCH → policy RETRY_POLL
     │ crcErrorCount++ ; retryCount=1 ≤ MAX(2)│
     │ S9→S10 DRAIN │             │           │
     │              │             │           │
     │◄─Service()───│             │           │
     │ drain to 300 ms silence (K-14 — mandatory)
     │              │             │           │
     │ §17.3: RELEASE BUS between attempts    │
     │              │ OwnsBus()=false ── one normal poll cycle runs ──
     │              │ (restores telemetry availability, §15.3 [v1.1])
     │              │             │           │
     │◄─Service()───│             │           │
     │ S10→S2 ARMED │ re-acquire bus          │
     │──write(8 B)────────────► │──request──►│
     │              │             │           │
     │  ... attempt 2: anchor → progress → dump ...
     │              │             │           │
     │ S9 VERIFY: crc ✓ → status=OK           │
     │ result.retryCount=1, crcErrorCount=1   │
     │ (both published — §18.3 §16.3)         │
     │ S9→S10→S11 RESULT_READY                │
```

### 20.4 Sequence — desync detection and bounded abort

```
FifoDriver   Transport    Sensor
     │           │           │
     │ S4 AWAIT_ANCHOR       │
     │──read()─►│◄─garbage──│   (stale bytes / partial prior frame)
     │ scan for 50 03 ; discard non-matching
     │ desyncBytesDiscarded = 1 … 64
     │           │           │
     │ > MAX_DESYNC_BYTES(64) reached
     │ error = ERR_DESYNC_LIMIT   ◄── distinct code, NOT "CRC error"
     │ policy  = ABORT_ATTEMPT                        (fixes PRR C-6, C-7)
     │           │           │
     │ S4→S10 DRAIN → flushRx() → 300 ms silence
     │ retry budget check → S2 or S13 FAILED
     │
     │  ── had this been unbounded (Validation Tool),
     │     the scan could anchor on payload bytes
     │     (0x50 0x03 occurs ~1 per 65 536 positions)
     │     and mis-frame a 6 KB read.
```

### 20.5 State transition diagram

```
                        ┌──────────────┐
                        │  S0 UNINIT   │
                        └──────┬───────┘
                               │ Init()
                               ▼
       ┌──────────────► ┌──────────────┐ ◄────────────────────┐
       │                │   S1 IDLE    │                      │
       │                └──────┬───────┘                      │
       │ T_COOLDOWN            │ Request() admitted §19.2      │
       │                       ▼                               │
┌──────┴───────┐        ┌──────────────┐                       │
│ S12 COOLDOWN │        │  S2 ARMED    │ ◄── retry §17.3 ──┐   │
└──────▲───────┘        └──────┬───────┘                   │   │
       │                       │ acquire bus               │   │
       │ ReleaseResult()       ▼                           │   │
┌──────┴────────┐       ┌──────────────┐                   │   │
│S11 RESULT_RDY │       │  S3 REQUEST  │                   │   │
└──────▲────────┘       └──────┬───────┘                   │   │
       │ bus released          │ write complete            │   │
       │                       ▼                           │   │
┌──────┴───────┐        ┌──────────────┐ ◄──── RESYNC ─────┤   │
│  S10 DRAIN   │        │S4 AWAIT_ANCH │                   │   │
└──────▲───────┘        └──────┬───────┘                   │   │
       │ silence               │ anchor 50 03              │   │
       │ 300 ms                │                           │   │
       │                       ▼   ERR_DESYNC_LIMIT ───────┼───┤
       │                ┌──────────────┐  ERR_NO_RESPONSE  │   │
       │                │ S5 READ_TYPE │                   │   │
       │                └──┬────────┬──┘                   │   │
       │       Len=0x01 ◄──┘        └──► Len=0x00          │   │
       │                │                │                 │   │
       │                ▼                ▼                 │   │
       │         ┌──────────────┐ ┌──────────────┐         │   │
       │         │S6 READ_PROG  │ │ S8 READ_DUMP │         │   │
       │         └──────┬───────┘ └──────┬───────┘         │   │
       │                │                │ 6146 B          │   │
       │                ▼                ▼                 │   │
       │         ┌──────────────┐ ┌──────────────┐         │   │
       │         │S7 POLL_WAIT  │ │  S9 VERIFY   │         │   │
       │         └──────┬───────┘ └──┬────────┬──┘         │   │
       │                │            │  crc ✓ │ crc ✗      │   │
       │  ┌─────────────┘            │        └────────────┤   │
       │  │ D-14:                    │   retry budget left │   │
       │  │  TRUEPOLL   → S3         │                     │   │
       │  │  PURELISTEN → S4         │ status=OK           │   │
       │  └──────────────────────────┤                     │   │
       │                             ▼                     │   │
       └─────────────────────────────┴─────────────────────┘   │
                                     │                          │
                       retry exhausted│                         │
                                     ▼                          │
                              ┌──────────────┐                  │
                              │  S13 FAILED  │──► S10 DRAIN ────┘
                              └──────┬───────┘
                                     │ 5 consecutive §17.6
                                     ▼
                              ┌──────────────┐
                              │ S14 DISABLED │ ── manual reset / reboot only
                              └──────────────┘

  INVARIANT: every transition is taken inside ONE bounded Service() call.
             No state waits. No state loops unbounded.        (satisfies PRR C-1)
```

### 20.6 Data flow

```
  SENSOR (WTVB05)
      │  RS485 · 9600 8N1 · 960 B/s
      ▼
  ┌─────────────────────┐
  │ UART RX ring 2048 B │  ◄── D-13 PREREQUISITE (default 256 B is insufficient)
  │  2133 ms headroom   │      8.5× margin over the 250 ms service cadence
  └──────────┬──────────┘
             │ transport.read(≤512 B)   ── non-blocking, ≤1 call per tick
             ▼
  ┌──────────────────────────────────────────────────────────┐
  │  FrameCodec  ── bounded resync · shape discrimination     │
  └──────┬────────────────────────────────────┬──────────────┘
         │ progress frames                    │ dump payload bytes
         ▼                                    │
  ┌────────────────┐                          │  each byte, exactly once:
  │ fill value     │                          ├──────────────┬───────────┐
  │ K-10 invariants│                          ▼              ▼           │
  │ checked (§16)  │                 ┌────────────────┐ ┌─────────────┐  │
  └───────┬────────┘                 │ StreamingCrc16 │ │SampleDecoder│  │
          │                          │  accumulate    │ │ BE int16    │  │
          │                          └───────┬────────┘ │ stride 6    │  │
          │                                  │          └──────┬──────┘  │
          │                                  │        (writes via plain  │
          │                                  │         int16_t* handles,│
          │                                  │         not a FifoArena* │
          │                                  │         param — L3 passes│
          │                                  │         the pointers in, │
          │                                  │         [v1.1] §11.4)    │
          │                                  │                 ▼         │
          │                                  │        ┌─────────────────┐│
          │                                  │        │  FifoArena      ││
          │                                  │        │  x[] y[] z[]    ││
          │                                  │        │  6144 B static  ││
          │                                  │        │  ── NO staging  ││
          │                                  │        │     buffer      ││
          │                                  │        │     (D-11)      ││
          │                                  │        └────────┬────────┘│
          ▼                                  ▼                 │         │
  ┌──────────────────────────────────────────────────┐         │         │
  │  S9 VERIFY   crc.final() vs transmitted (K-6/K-7)│         │         │
  └───────┬──────────────────────────────────┬───────┘         │         │
      ✓ OK│                          ✗ FAILED│                 │         │
          ▼                                  ▼                 │         │
  ┌──────────────────────────────────────────────────┐         │         │
  │  FifoCaptureResult                                │◄────────┘         │
  │   status · error · provenance · diagnostics       │  pointers valid   │
  │   ── SOLE gate to arena access (D-8, D-10) ──     │  ONLY if OK       │
  └───────────────────────────┬──────────────────────┘                   │
                               │ TryAcquireResult()                       │
                               │ (ownership transfer, 0 copies)  [v1.1]   │
                               ▼                                          │
              ┌───────────────────────────────────────────┐              │
              │  handleFifoCaptureCompletion()   §9.1      │              │
              │  the ONE call site — not two consumers     │              │
              │  ┌───────────────────────────────────────┐ │              │
              │  │ 1. MQTT /event §18.3 — ALWAYS,         │ │              │
              │  │    success AND failure, ~450 B additive│ │              │
              │  ├───────────────────────────────────────┤ │              │
              │  │ 2. WaveformStore → 9 MB FAT (CN-11),   │ │              │
              │  │    when Phase 7 is present, OK only    │ │              │
              │  └───────────────────────────────────────┘ │              │
              └───────────────────────┬─────────────────────┘              │
                                       │ ReleaseResult()                   │
                                       └───────────────────────────────────┘
                                                  arena returns to driver
```

---

## 21. Traceability — PRR Critical and High findings

| PRR finding | Severity | Satisfied by | How |
|---|---|---|---|
| **C-1** Unyielding busy-waits starve Core 0 | CRITICAL | D-1, §7.2, §11.2, §20.5 | Tick-sliced state machine; `Service()` contract forbids any wait; every transition bounded; ≤512 B/call cap |
| **C-2** No validity contract; stale data | CRITICAL | D-8, D-10, §9 | Samples reachable only via `status == OK` result under exclusive ownership handoff |
| **C-3** Unarbitrated bus access | CRITICAL | D-15, D-7, §13 | `taskModbusRead` stays sole owner; driver is a mode, not a peer; transport injected, never self-acquired; no new mutex |
| **C-4** Duplicate 6 KB static buffers | HIGH | D-11, §10 | Single 6144 B arena; staging buffer eliminated entirely. 18,944 B → 6.4 KB (−66%) |
| **C-5** `malloc` on CRC path; OOM→CRC error | HIGH | D-11, D-12, §16.2 | Streaming CRC; zero heap; `ERR_OUT_OF_MEMORY` unrepresentable |
| **C-6** Unbounded resync | HIGH | §17.2 | `MAX_DESYNC_BYTES = 64`; distinct `ERR_DESYNC_LIMIT` |
| **C-7** Error taxonomy collapse | HIGH | D-18, §16 | 14 distinct codes across 3 domains, each with one recovery policy |
| **C-8** Five competing implementations | HIGH | D-14, §22 G-1 | One implementation; the single open question isolated to one named constant on one edge |
| **C-9** Timeout parameter silently overridden | MEDIUM | D-5, D-17, §15 | Named non-overridable constants, each traced to a measurement |
| **C-10** I/O inside receive path | MEDIUM | D-4, §8.2 | Zero I/O in receive path; diagnostics recorded to result, emitted after |
| **C-11** Duplicated logic (5×/3×/2×) | MEDIUM | §5.2, §20.1 | One codec, one CRC, one decoder, one drain — enforced by layering |
| **C-12** No separation of concerns | MEDIUM | §4.2, §5.2, §6 | Four layers, explicit non-responsibilities, injected transport |
| **C-13** Blocking `delay()` | MEDIUM | §7.2 | No `delay()`/`vTaskDelay()` anywhere in the driver |
| **C-14** 1 of 38 functions testable | MEDIUM | D-7, §5.2, §23 | L1 abstraction makes L2/L3 pure; recorded logs become fixtures |
| **C-15** 63% success, no retry budget | MEDIUM | §17.3, §17.4, §17.6, §18.3 | Bounded retries, cooldown, circuit breaker, and every outcome published |
| **C-16** Dead code | LOW | §2.2, §7.3 | Nothing ported; API surface minimal by construction |
| **C-17** Naming / version drift | LOW | §8.2 | `fwVersion` + `gitCommitHash` in every result, from `build_info.h` |

**All three CRITICAL findings are satisfied structurally** — by making the defect unrepresentable — rather than by a rule someone must remember.

---

## 22. Open Gates

**[v1.1]** Neither gate blocks Phases 1–7 of implementation — D-14 (§11.3) exists specifically so that isn't necessary. G-1 must clear before Phase 8's soak validation is interpreted (Implementation Plan Task 8.1); G-2 has no implementation dependency at all, only a scope boundary (`REMOTE_ON_DEMAND` excluded). Both remain open and are carried forward, not resolved by this revision — see Implementation Plan §0.

### G-1 — Protocol model undetermined (PRR BL-1)

**Question.** Does the sensor complete the dump autonomously (PURELISTEN), or only in response to repeated polls (TRUEPOLL)?

**Status.** The Validation Tool built the deciding experiment (v3.11) but **no result exists in this repository** — zero `[TRUEPOLL-CSV]` / `[STRESS-CSV]` lines across all 7 logs. Every log predates v3.9 and exercises only the older `readFIFORaw` path.

**Evidence required.** `TRUEPOLL` and `STRESSTEST` runs on hardware, ≥ 20 runs each, at SR5 (1 kHz) and SR6 (512 Hz) — SR6 being where the measured rate collapses to 17%. Compare success rates.

**Impact if wrong.** One constant, `FIFO_PROTOCOL_MODEL`, on one state transition (D-14). Contained by design.

**Recommendation.** Run the experiment with the existing Validation Tool before writing driver code. That is the tool's remaining purpose; it should be used for it and then retired.

### G-2 — Trigger channel for "on-demand" capture

**Question.** The Architecture Master Plan §5.1 describes the FIFO driver as available for *on-demand* diagnostic invocation. CM-100 has **no inbound command channel** — MQTT is publish-only (CN-8, verified: no `subscribe()` anywhere in the production firmware).

**Consequence.** Of the five trigger sources in §19.1, four are local and available today; `REMOTE_ON_DEMAND` is not implementable without adding an MQTT subscribe path — a change to the network layer, the security posture (an inbound control surface on an mTLS link), and the cloud contract.

**This is not a driver decision.** The driver's trigger interface (A-2) is source-agnostic and needs no change either way. Raised here because it materially affects what "on-demand" can mean for CM-100 and should be settled explicitly rather than discovered during integration.

### Prerequisite (not a gate)

**P-1 — `setRxBufferSize(2048)`** must be added to `setup()` before `SerialRS485.begin()` (D-13). One line, +1792 B, no behavioural effect on normal Modbus traffic. Without it the design has 6% timing margin instead of 8.5×.

---

## 23. Test Strategy

| Level | Method | Enabled by |
|---|---|---|
| **Unit — pure logic** | Host-compiled tests of `StreamingCrc16`, `SampleDecoder`, `FrameCodec` against known vectors | L2 has no I/O (§5.2) |
| **Unit — state machine** | Drive `Service()` from a scripted `LogReplayTransport`; assert transitions, timeouts, error codes | D-7 transport injection |
| **Regression — real frames** | Replay the 7 recorded hardware logs, **including all 7 failures**, as fixtures | PRR C-14 recommendation |
| **Fault injection** | Synthetic streams: truncated dump, corrupted CRC, injected `0x50 0x03` in payload, `fill` regression, `fill` > 6144, stale-byte prefix | Fixture transport |
| **Timing** | Instrument `Service()` worst-case duration on target; assert ≤ 5 ms | On-target |
| **Integration** | Verify normal polling suspends/resumes; `g_modbusConsecErrors` not incremented; motor FSM unaffected across a full capture | On-target |
| **Soak** | ≥ 100 captures at each SR; characterise success rate against K-15 baseline | On-target |

**The fault-injection row is the one the Validation Tool could never do.** Its parser was inseparable from `SerialRS485.read()`, so a corrupted-frame test required a cooperating, misbehaving sensor. The transport seam turns that into a table of byte arrays.

---

## 24. Summary of Design Decisions

| ID | Decision | Primary rationale |
|---|---|---|
| D-1 | Passive tick-driven state machine serviced by `taskModbusRead` | K-11 (6.4 s) + CN-1 (250 ms) + CN-2/CN-4 (no blocking) admit only this shape |
| D-2 | Zero heap | PRR C-5; heap contended by mTLS/MQTT |
| D-3 | Driver never writes a sensor register | Sensor config is not the driver's to own |
| D-4 | No I/O in the receive path | PRR C-10 |
| D-5 | Named, traceable, non-overridable timing constants | PRR C-9 |
| D-6 | No externally reachable driver state | PRR C-2 |
| D-7 | Injected transport abstraction | Makes C-3 structurally impossible; makes C-14 solvable |
| D-8 | `status` is the sole authority on validity | PRR C-2 |
| D-9 | Raw int16 counts; no scaling | K-9 is unconfirmed (RFC-0007 §4) |
| D-10 | Single arena, exclusive ownership handoff, zero copies | PRR C-2 + memory economy |
| D-11 | Streaming CRC + streaming decode; no staging buffer | PRR C-4/C-5; −66% RAM |
| D-12 | All-static allocation | Determinism; fragmentation avoidance |
| D-13 | `setRxBufferSize(2048)` prerequisite | CN-5: 6% margin → 8.5× |
| D-14 | Protocol model as one named compile-time constant | G-1 unresolved; isolate the unknown |
| D-15 | `taskModbusRead` remains sole bus owner (DESIGN-0002 Option A) | Preserves proven single-caller invariant; avoids priority inversion |
| D-16 | Single-writer; queue + one narrow mutex | Follows `CLAUDE.md` cross-core rules exactly |
| D-17 | Separate named budgets per timeout class | PRR C-9 |
| D-18 | Errors classified by domain and policy | PRR C-7 |
| D-19 | Purely additive telemetry | `CLAUDE.md`, RFC-0002 |
| D-20 | Waveform egress deferred; FAT storage recommended | CN-11: 9 MB already provisioned, unused |

**Related Architecture Decision Notes `[v1.3]`.** ADRs record implementation-level interpretations of the decisions above; they are a separate log, not additional rows in this table, and none of them amends the text of D-1..D-20.

| ID | Note | Interprets | Primary rationale |
|---|---|---|---|
| ADR-1 | Bus ownership includes EN ownership (`CM-100_ADR-1_Bus_Ownership_Includes_EN_Ownership_v1.0.md`) | D-15 (unchanged) | Every existing production bus-window pairs `rs485Enable()`/`rs485Disable()` with the window itself; the FIFO capture window is treated the same way, by the same owner (§13.2.1) |

---

*End of CM-100_FIFO_DRIVER_SDS_v1.0 — design document only; no implementation code is specified or implied.*
