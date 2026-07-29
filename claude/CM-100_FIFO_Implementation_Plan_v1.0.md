> **Document Status**
>
> **Type:** Implementation Plan / Blueprint — planning document only. **No production code is written here.** Interface listings below are signatures/declarations for review purposes; function bodies are coding-time work, out of scope for this document.
> **Version:** 1.3 (v1.1 amended per Architecture Freeze Review v1.0, 7 Must-Fix findings; v1.2 amended per post-implementation Architecture Update — RS485 EN-pin ownership moved from Task 1.2 to Task 4.2; v1.3 amended per Architecture Review Feedback on v1.2 — Task 1.2's backward dependency on Task 4.2 removed, its transport precondition documented explicitly, and its rationale re-pointed at `ADR-1` instead of an extended reading of D-15. See Revision Notes below.)
> **Date:** 2026-07-28
> **Input (frozen evidence, per instruction — not re-investigated here):** Reverse Engineering findings, `WTVB05_ValidationTool_v3_11_TRUEPOLL` Production Readiness Review, `CM-100_FIFO_DRIVER_SDS_v1.3`, `WTVB05_Communication_Regression_Investigation_Plan_v1.0`, Experiment 0 Baseline Report, WTVB05 Intermittent Communication Reproducibility Study, `CM-100_FIFO_Architecture_Freeze_Review_v1.0`.
> **Approval basis:** this plan proceeds on the instruction that `CM-100_FIFO_DRIVER_SDS_v1.3` is approved, including its D-15 recommendation (Modbus bus ownership, DESIGN-0002 Option A, unchanged) and `ADR-1`'s interpretation that bus ownership includes EN-pin ownership (§13.2.1). That instruction is treated as the approval record for this plan's purposes.
> **Firmware baseline:** `WTVB02_ESP32S3_V16_CM_fault_latch_v3_hardened_patched_v16_5.ino`, `FW_VERSION "16.5"`, branch `recover-experimental`.
> **Implementation status:** **Architecture frozen at v1.3** — see `CM-100_FIFO_Architecture_Freeze_Report_v1.3.md`. **Phase 1 implementation may resume.** Task 0.1 complete, uncommitted. Task 1.1 complete, uncommitted. Task 1.2 revised to match the frozen v1.3 architecture (`digitalWrite`/`RS485_EN_PIN` ownership removed from `fifo_transport_uart485.cpp`); uncommitted, pending build verification before commit. Task 1.3 may proceed once Task 1.2 is verified.

# CM-100_FIFO_Implementation_Plan_v1.0

## Revision Notes (v1.0 → v1.1)

Applied per `CM-100_FIFO_Architecture_Freeze_Review_v1.0.md`. Every task list, dependency line, and commit count below is otherwise unchanged — no phase added, removed, or reordered; total commit count is unchanged at 23 code commits + 2 milestones.

| Finding | Task(s) corrected | Change |
|---|---|---|
| 1.1 | Task 2.3 | `FrameCodec_Step()` signature corrected to plain `int16_t*` write pointers — no `FifoArena*` parameter |
| 2.1 | Task 5.2, Task 7.1 | Task 5.2 now explicitly establishes `handleFifoCaptureCompletion()` as the sole `TryAcquireResult`/`ReleaseResult` call site; Task 7.1 adds a step to it rather than acquiring independently |
| 3.1 | Task 2.3, Task 3.3 | `FrameCodecState` made explicit (caller-owned, passed by pointer) — the `/* internal scan state */ ...` placeholder is resolved |
| 3.2 | Task 3.1, Task 3.5, Task 6.2 | `FifoAdmissionContext` added as a field on `FifoCaptureRequest`; Task 6.2's "coding-time decision" framing removed — decided |
| 4.1 | Task 1.1, Task 1.2 | **[Superseded — see v1.2 below]** Originally resolved as "transport owns EN, matching SDS §6.2, reused via `write()`." |
| 5.1 | Task 3.4 | Risks field corrected: no `ABSENT_STOPPING_MS` coupling exists; retry-release justified on telemetry/sensor-settling grounds only. No constant changed |
| 7.1 | Task 1.1, Task 1.2, Task 3.1, Task 3.3 | `hadOverflow` added to the transport interface; `ERR_PROGRESS_REGRESSION`/`OVERRUN` detection assigned explicitly to `S6`'s handler in Task 3.3 |

## Revision Notes (v1.1 → v1.2) — Architecture Update

Applied post-implementation, during Task 1.2's implementation review — not at the Freeze Review stage, because the contradiction this resolves was only discoverable by reading the actual production call sites for `rs485Enable()`/`rs485Disable()` (`:4334/4425`, `:4571/4584`, `:4680`), which prior review passes cited but did not fully trace. Supersedes Finding 4.1's v1.1 resolution only. Nothing else in v1.1 is affected — see the Architecture Change Report §5 for the explicit per-decision cross-check.

| Task | Change |
|---|---|
| Task 1.2 | Purpose, Internal interfaces, and Risks rewritten: `Uart485Transport` is now a pure byte transport. `digitalWrite(RS485_EN_PIN, ...)`, the locally-duplicated pin constant, and all EN-related reasoning are removed from this task's scope entirely. |
| Task 4.2 | **New requirement**, not a correction: the `OwnsBus()` branch now explicitly brackets `rs485Enable()`/`rs485Disable()`, with an added Risk item that the disable call must fire on every bus-release path (normal completion, `Abort()`, circuit breaker), not only the happy path. |

## Revision Notes (v1.2 → v1.3) — Architecture Review Feedback

Applied per Architecture Review Feedback on v1.2. Document-only; no firmware changes. The v1.2 decision (EN owned at the `taskModbusRead()`/`OwnsBus()` boundary) is unchanged — only how Task 1.2 and Task 4.2 are described relative to each other, and relative to D-15, is corrected.

| Item | Task(s) corrected | Change |
|---|---|---|
| 1. Backward dependency | Task 1.2, Task 4.2 | Task 1.2's Risks/Dependencies previously read as depending on Task 4.2 (a later task) for its own correctness. Removed: Task 1.2's Dependencies field reverts to Task 1.1 only; its Risks field now states the bus/EN precondition as a documented contract obligation on *any* caller (present tense, not tied to Task 4.2's existence), so Task 1.2 is complete and correct standing alone. Task 4.2's role is now described as *satisfying* that precondition at integration time — integration, not a code dependency |
| 2. Precondition documentation | Task 1.2 | Purpose and Risks fields now state explicitly, matching SDS §6.1: every `write()` call assumes the caller already owns the RS485 bus and has already asserted EN. This was previously implied only through the cross-reference to Task 4.2; it is now stated as the transport's own contract |
| 3. D-15 vs. ADR | Task 1.2, Task 4.2 | References to "the extended reading of D-15" replaced with references to `ADR-1` (`CM-100_ADR-1_Bus_Ownership_Includes_EN_Ownership_v1.0.md`). D-15's text is unchanged |

---

## 0. Carried-Forward Open Items (not re-investigated, tracked here)

Two items from the SDS remain open. Neither blocks the phases below from starting — the SDS structured the design specifically so they wouldn't (D-14) — but both must be resolved before the phases that depend on them, and are listed here so no reviewer discovers them by surprise later.

| ID | Item | Status | Blocks |
|---|---|---|---|
| **G-1** | Protocol model (TRUEPOLL vs PURELISTEN) still empirically undetermined. The Reproducibility Study proved *basic* Modbus communication (register `0x3A`) is currently reliable (0/16 boot failures, 0 spontaneous failures over 300 s) — it did **not** exercise the RAWFIFO transaction (register `0x2C`, the 6144-byte dump) at all, so the old 63%/17% baseline (`K-15`) is evidence about the *old, discarded* implementation, not the new one. | Open, isolated to `FIFO_PROTOCOL_MODEL` (one constant, D-14) | Phase 8 soak validation only. Phases 1–7 are protocol-model-agnostic by design and proceed with `TRUEPOLL` as the provisional default per the SDS. |
| **G-2** | No inbound MQTT channel exists (`CN-8`, confirmed: no `subscribe()` anywhere in production). `REMOTE_ON_DEMAND` cannot be implemented. | Open, out of scope for this plan (network/security/cloud-contract decision) | `REMOTE_ON_DEMAND` trigger source only. All other trigger sources (`FAULT_LATCH`, `OPERATOR_BUTTON`, `SCHEDULED`, `COMMISSIONING`) are unaffected and in scope. |

---

## 1. Repository Layout

All new code lives in **new files** in the existing sketch directory. **Exactly one line changes in the existing `.ino` in Phase 0; the rest of the existing 8,343 lines are touched only at the specific, minimal integration points in Phases 4–6.**

```
claude/WTVB02_ESP32S3_V16_CM_fault_latch_v3_hardened_patched_v16_5/
├── WTVB02_ESP32S3_V16_CM_fault_latch_v3_hardened_patched_v16_5.ino   existing — touched in Phases 0, 4, 5, 6 only
├── build_info.h                                                       existing — untouched
├── fifo_types.h                    [NEW — Phase 3.1]  shared enums/structs
├── fifo_transport.h                [NEW — Phase 1.1]  L1 interface (SDS §6)
├── fifo_transport_uart485.h        [NEW — Phase 1.2]  L1 production impl
├── fifo_transport_uart485.cpp      [NEW — Phase 1.2]
├── fifo_codec.h                    [NEW — Phase 2.3]  L2: CRC, decoder, frame codec (SDS §8, §16)
├── fifo_codec.cpp                  [NEW — Phase 2.1–2.3]
├── fifo_arena.h                    [NEW — Phase 3.2]  L3: sample arena + ownership handoff (SDS §9)
├── fifo_arena.cpp                  [NEW — Phase 3.2]
├── fifo_driver.h                   [NEW — Phase 3.5]  L3: public API (SDS §7)
├── fifo_driver.cpp                 [NEW — Phase 3.3–3.5]
├── fifo_waveform_store.h           [NEW — Phase 7, deferred/optional]
├── fifo_waveform_store.cpp         [NEW — Phase 7, deferred/optional]
└── test/                           [NEW — Phase 2.4, 3.6 — host-only, NOT compiled into firmware]
    ├── fixtures/                     converted from the 7 recorded hardware logs (PRR evidence)
    ├── log_replay_transport.h/.cpp   test-only FifoTransport implementation
    ├── test_fifo_codec.cpp
    └── test_fifo_driver.cpp
```

**Why a `test/` subdirectory is safe:** confirmed empirically this session — the production sketch directory already contains a subdirectory (`pre_flash_backup_20260717_161226/`, containing a large `.bin`) that `arduino-cli compile` does not sweep into the build. Arduino's sketch builder only auto-includes `.ino`/`.cpp`/`.h` files directly in the sketch root. `test/` will not be picked up by the firmware build. **First action of Phase 2.4 is to confirm this holds** (a clean `arduino-cli compile` after `test/` exists, verifying no test-framework code leaks into the firmware binary) before relying on it further.

**Why separate files instead of appending to the monolithic `.ino`:** PRR finding **C-12** — the Validation Tool's 2,794-line single translation unit had no module boundary, no test seam, and required awkward forward-declarations (`struct VelocityData;` etc., PRR-cited) to work around `.ino` auto-prototype generation. Separate `.h` files, `#include`d near the top of the main `.ino` (Phase 4.1), sidestep that auto-prototype problem entirely — the types are already fully declared via textual `#include` before Arduino's prototype pass ever runs over the `.ino` content. This is a structural win worth preserving, not just a style preference.

---

## 2. Reuse Inventory

### 2.1 Existing knowledge to reuse (lift the values/algorithms, not the code)

| Item | Value | Source |
|---|---|---|
| RAWFIFO register | `0x002C` | PRR K-1 |
| Request frame layout | `50 03 00 2C 00 01 <CRC_lo> <CRC_hi>` | PRR K-2 |
| Response shape discrimination | `Len=0x01` progress (7B) / `Len=0x00` dump (6149B) | PRR K-3–K-5 |
| CRC algorithm | CRC-16/Modbus, poly `0xA001`, init `0xFFFF`, low-byte-first on wire | PRR K-6, K-7 |
| Payload layout | 1024×3 samples, int16 big-endian, stride 6B, X/Y/Z order | PRR K-8 |
| Transfer duration | 6402 ms, fixed (baud-limited arithmetic identity) | PRR K-11 |
| Request→first-byte latency | 256–260 ms | PRR K-12 |
| Bus-settle drain concept | 300 ms true-silence drain after any FIFO transaction | PRR §C-19, `0xE2` cascade finding K-14 |
| `setRxBufferSize(2048)` | production must add this (currently default 256B) | SDS D-13, CN-5 |
| Timeline instrumentation schema | Treq/T49/T99/TanchorNext/Tcomplete | PRR §C-19 (rated the one well-designed part of the Validation Tool) — promoted to the `/event` schema (SDS §18.3) |
| 7 recorded hardware logs, incl. all 7 failures | regression test fixtures | PRR evidence corpus |

### 2.2 Validation Tool code that must NOT be copied

| Function/pattern | Why not | PRR finding |
|---|---|---|
| `readOneFIFOFrame()`, `readFIFORaw()`, `readFIFOHybrid()`, `readFIFOPureListen()`, `fifoPassiveListen()`, `runFIFOStressOnce/Loop()`, `runFIFOTruePollOnce/Loop()` | Five competing protocol hypotheses in one file; a driver embodies exactly one (SDS D-14 makes this one named constant, not five functions) | C-8 |
| Unyielding `while(!available()) continue;` receive loops | Would starve Core 0 IDLE0 (`WDT_PANIC=y`) and the `StateMachine` task | C-1 |
| `g_fifoX/Y/Z` bare globals | No validity contract — stale, plausible, CRC-valid data survives a failed capture | C-2 |
| Direct `SerialRS485` access from FIFO code | Unarbitrated second bus owner | C-3 |
| Two 6144B static `dataBuf` copies | Confirmed duplicate in ELF; eliminated by streaming decode (SDS D-11) | C-4 |
| `malloc(6147)` + `memcpy` on the CRC path | Heap on hot path; OOM misreported as CRC error | C-5 |
| Unbounded anchor resync | Can lock onto payload bytes (`50 03` occurs ~1/65536 by chance in 6144B) | C-6 |
| `FIFOFrameResult` 4-value enum (`CRC_ERROR` overloaded 3 ways) | Collapses distinct failures into one code | C-7 |
| `Serial.printf()` inside the receive path | I/O interleaved with frame reception | C-10 |
| `dumpFIFOToSerialCSV()` (1024 `printf` lines) | ~5s blocking Serial per capture; production publishes via MQTT/FAT instead | — |
| Serial CLI (`handleSerialCommands()`), register polling, spectrum energy, anomaly scoring | CM-100 already implements all of this, with cross-core discipline the tool lacked | C-12 |

### 2.3 Production-quality abstractions this plan introduces

These did not exist in either codebase and are new, designed specifically to satisfy the seven preserved principles (§6):

- `FifoTransport` — injected, non-blocking byte interface (SDS §6). Makes bus-ownership violation structurally impossible, not just discouraged.
- `FifoCaptureResult` — validity-gated result object (SDS §8.2, D-8). Makes stale-data exposure structurally impossible.
- `FifoArena` ownership handoff (SDS §9, D-10). Makes concurrent/dangling access structurally impossible.
- Domain-classified error enum (SDS §16, D-18). Makes "three failures, one error code" structurally impossible.
- **[v1.1]** `handleFifoCaptureCompletion()` — the single named coordinator (SDS §9.1, Freeze Review 2.1). Makes "two independent consumers race the same acquired result" structurally impossible.
- **[v1.1]** `FrameCodecState` — explicit, caller-owned per-frame parsing state (SDS §11.4, Freeze Review 3.1). Makes hidden state in the one file most likely to need it structurally impossible.
- **[v1.1]** `FifoAdmissionContext` — plain-data admission-gate input (SDS §8.1/§19.2, Freeze Review 3.2). Makes a direct `.ino`-global dependency inside `fifo_driver.cpp` structurally impossible.

### 2.4 Temporary diagnostic code that must never enter production

| Item | Rule |
|---|---|
| `test/` fixtures, `LogReplayTransport` | Host-only. Never referenced from any file that ships to the device. Verified by Phase 2.4's build check. |
| Any per-tick `Serial.print` inside `FifoDriver_Service()` | Forbidden permanently (D-4), not just during development. If added for local debugging, it must be removed before the commit closes — this is a review-blocking item, not a style note (see §6 logging policy). |
| Any `#define FIFO_DEBUG_*` compile flag | If introduced during coding for bring-up convenience, must default to **off**, must not be enabled in any commit that touches `main`, and must be listed in the commit message per `CLAUDE.md`'s versioning-tag convention. None is planned in this blueprint; flagged here only so one isn't added silently later. |

---

## 3. Preserving Production Design Principles

Each principle mapped to the specific structural mechanism enforcing it, per SDS decision ID. This table is the basis for the review checklist in §8.

| Principle | Mechanism | SDS ref |
|---|---|---|
| **Single responsibility** | Four layers (L1 transport / L2 codec / L3 session / L4 diagnostic workflow), each with an explicit non-responsibility list. The driver never scales samples, never writes sensor config, never picks *when* to capture. | §4.2, §5.2 |
| **Deterministic timing** | Every timeout is a named, non-overridable constant traced to a measurement (K-11–K-13). `Service()` is bounded to ≤512 B/call, target ≤5 ms worst case. No adaptive/learned timing anywhere. | D-5, D-17, §7.2 |
| **Non-blocking operation** | `Service()` contract forbids any wait, any `delay()`/`vTaskDelay()`, any unbounded loop. Enforced per-state (15 states, each one bounded action). This is what satisfies PRR **C-1**. | D-1, §11 |
| **No hidden state** | All driver state reachable only through the driver's own API (D-6). No global the rest of the firmware can read or write directly. Cross-core rules follow `CLAUDE.md` exactly: single-writer, queue for structs, atomic single-word for phase/flags. | D-6, D-16, §14 |
| **Production-safe logging** | New policy, defined in §6 below — exactly one line at session start, one at session end; zero per-tick logging; rich diagnostics go to the `/event` MQTT payload (structured, not `Serial` prose), never to `Serial` from inside `Service()`. | D-4, §18.3 |
| **Fault containment** | 3-domain, 14-code error taxonomy, each with one recovery policy (D-18). Circuit breaker (§17.6) makes a persistently-faulting sensor self-limiting rather than a slow telemetry drain. Bus is unconditionally drained before release, success or failure (K-14). | §16, §17 |
| **Testability** | L1 injected, so L2/L3 are pure logic against a `LogReplayTransport`. The 7 recorded hardware logs — including all 7 failures — become regression fixtures. This is what satisfies PRR **C-14** (1 of 38 functions testable → target: all of L2 and L3 host-testable). | D-7, §23 |

### 3.1 Logging Policy (new, filling a gap the SDS left open)

| Event | Destination | Content |
|---|---|---|
| Session admitted | `Serial`, once | `[FIFO] capture admitted: id=<n> source=<trigger> tag=<...>` |
| Session terminal (success or failure) | `Serial`, once | `[FIFO] capture <id> <OK|FAILED> error=<code> retries=<n> polls=<n> t=<ms>` |
| Full diagnostics (Treq/T49/T99/TanchorNext/Tcomplete, per-poll detail) | `/event` MQTT payload only | structured JSON, per SDS §8.2/§18.3 |
| Anything per-tick, per-byte, per-poll | **Never to `Serial`.** | — |

This is a direct, deliberate contrast with the Validation Tool, which printed a line for nearly every byte and every poll (PRR C-10). The rich data the Validation Tool's prints were good at capturing (PRR's one compliment, §C-19) is preserved — just relocated to the structured `/event` payload where it's queryable, instead of scrolling past on a console.

---

## 4. Integration Sequence

Dependency graph. Phases in the same row can proceed in parallel; arrows show hard dependencies.

```
Phase 0 (prerequisite, production)
   │
   ├──────────────┐
   ▼              ▼
Phase 1          Phase 2         (independent of each other — L1 and L2
(Transport)      (Codec)          have no compile-time dependency)
   │              │
   └──────┬───────┘
          ▼
      Phase 3
   (Session / L3 driver + public API)
          │
          ▼
      Phase 4
   (taskModbusRead integration — bus ownership)
          │
   ┌──────┴──────┐
   ▼             ▼
Phase 5         Phase 6
(Telemetry)    (Trigger wiring)
   │             │
   └──────┬──────┘
          ▼
      Phase 7  (optional/deferred — waveform egress, D-20)
          │
          ▼
      Phase 8  (soak validation, sign-off)
```

**Phase 0 is a hard prerequisite for everything downstream that runs on hardware** (though it can land and soak independently, before any FIFO code exists — recommended, so any regression from the buffer-size change alone is immediately attributable).

**Phases 1 and 2 have zero dependency on each other or on hardware.** Both can be developed and fully unit-tested on a host machine before Phase 0 ever needs to be flashed.

**Phase 4 is the highest-risk phase** — the first point touching the proven, working `taskModbusRead()` loop. It is deliberately the smallest possible diff (one branch, per SDS §13.2's pseudocode shape) and is not bundled with any other change.

---

## 5. Phase-by-Phase Task Breakdown

Each task = one commit. Suggested tag prefix: `v16.6.N-fifo` (following the existing `v16.5.1-p4.1` / `v16.5.2-p4.2` convention). Every code change must carry a `// [v16.6.N-fifo]` comment tag per `CLAUDE.md`.

---

### Phase 0 — Prerequisite

#### Task 0.1 — RX buffer sizing
**Commit 1** · `fix(rs485): size RX buffer for FIFO dump transfers (P-1)` · `v16.6.1-fifo`

| | |
|---|---|
| Purpose | Close SDS gap **P-1**: default 256B RX buffer gives only 267 ms headroom against the 250 ms `taskModbusRead` cadence (6% margin) — insufficient once a 6146B dump exists to receive. 2048B gives 8.5× margin. |
| Files affected | The existing `.ino`, one line, immediately before `SerialRS485.begin(...)` in `setup()`. |
| Public interfaces | None — a single library call, no new symbol. |
| Internal interfaces | None. |
| Risks | None identified. Normal Modbus responses are ≤37B; a larger buffer has no behavioral effect on existing polling. This is documented in `CLAUDE.md`'s own convention for board/runtime-config changes (compare to the `CDCOnBoot=cdc` FQBN fix, which was a config-only change with zero source modification). |
| Verification method | Clean build; flash; confirm existing telemetry (VRMS, PEAK, current) unaffected over a short soak, matching the pattern already used for `serial_capture_test7_soak_20260727.log`. |
| Rollback strategy | Delete one line. |
| Dependencies | None. Can land before any other FIFO work exists. |

---

### Phase 1 — L1 Transport Abstraction

#### Task 1.1 — Define the transport interface
**Commit 2** · `feat(fifo): add FifoTransport interface (L1)` · `v16.6.2-fifo`

| | |
|---|---|
| Purpose | Establish the seam that makes PRR **C-3** (unarbitrated bus access) structurally impossible and PRR **C-14** (untestable parser) structurally solvable, per SDS D-7. |
| Files affected | `fifo_transport.h` (new) |
| Public interfaces | ```c\nstruct FifoTransport {\n  void*    ctx;\n  size_t (*read)(void* ctx, uint8_t* dst, size_t maxLen);   // non-blocking, may return 0\n  size_t (*write)(void* ctx, const uint8_t* src, size_t len);\n  size_t (*available)(void* ctx);\n  void   (*flushRx)(void* ctx);\n  uint32_t (*nowMs)(void* ctx);\n  bool   (*hadOverflow)(void* ctx);   // [v1.1, Freeze Review 7.1] sole source of ERR_RX_OVERFLOW\n};\n``` |
| Internal interfaces | None — this file is the interface only, no implementation. |
| Risks | None — pure declaration, zero Arduino dependency, compiles standalone. |
| Verification method | Header compiles under both `arduino-cli` (as part of the sketch) and a plain host `g++ -c` (confirming zero Arduino dependency, required for Phase 2.4/3.6 host tests). |
| Rollback strategy | Delete file. Nothing else exists yet to depend on it. |
| Dependencies | None. |

#### Task 1.2 — Production transport implementation
**Commit 3** · `feat(fifo): add Uart485Transport (production L1 impl)` · `v16.6.3-fifo`

| | |
|---|---|
| Purpose | Concrete `FifoTransport` wrapping the existing `SerialRS485` object — the only file in the entire driver that touches Arduino's `HardwareSerial` API directly. A pure byte transport: moves bytes in and out of the UART only. It does not assert, deassert, or reference the RS485 EN pin in any form. **[v1.3]** This is a self-contained, documented contract, not a reference to another task: `write()` requires the caller to already own the RS485 bus and to have already asserted EN (SDS §6.1) — who satisfies that requirement is out of this task's scope. |
| Files affected | `fifo_transport_uart485.h`, `fifo_transport_uart485.cpp` (new) |
| Public interfaces | ```c\nvoid Uart485Transport_Init(FifoTransport* out, HardwareSerial* serial);\n``` — returns a populated `FifoTransport` bound to the given serial object. |
| Internal interfaces | Static callback functions matching the five `FifoTransport` function-pointer signatures (`read`, `write`, `available`, `flushRx`, `nowMs`, `hadOverflow` `[v1.1]`), each a thin wrapper over the corresponding `HardwareSerial`/`millis()` call. No callback references `RS485_EN_PIN`, `digitalWrite`, or any pin constant — `write()` is reduced to `serial->write(src, len)`, nothing else. |
| Risks | **[v1.3, Architecture Review Feedback — supersedes the v1.2 framing]** The v1.1 resolution had `write()` assert `RS485_EN_PIN` via a locally-duplicated constant (forced by `rs485Enable()`/`rs485Disable()`'s `static inline` linkage, which prevented calling the real functions from this separate translation unit). Per `ADR-1`, EN assertion is part of what "owning the bus" means in this codebase (every production bus-ownership window already pairs the two), and that ownership belongs to `taskModbusRead()`, not this file — D-15 itself is unchanged; `ADR-1` records the interpretation. **This task's own risk is fully stated by its documented precondition, not by which other task happens to exist:** `write()` assumes the bus is already owned and EN already asserted (SDS §6.1). This file has no way to verify that precondition, and — per D-7, the transport has no visibility into bus arbitration by design — is not expected to. Task 1.2 is therefore complete and correct in isolation against this contract, independent of Task 4.2's implementation status or sequencing. |
| Verification method | Host-side smoke test: bind to a mock/loopback if available; otherwise deferred to Phase 4's on-target test, since this is a thin wrapper with minimal independent logic. **[v1.3]** Phase 4's on-target integration test (Task 4.4) verifies that whichever task owns bus/EN assertion (currently Task 4.2) actually satisfies Task 1.2's documented precondition before calling `write()` — a FIFO capture's first transmitted byte must be checked to have actually reached the bus. This is an integration-time check on the *caller*, not a gap in Task 1.2 itself. |
| Rollback strategy | Delete both files; nothing yet references them outside tests. |
| Dependencies | Task 1.1. **[v1.3]** No dependency on Task 4.2 — removed. Task 1.2 does not require Task 4.2 to exist, be implemented, or be sequenced first; Task 4.2 later integrates against Task 1.2's already-complete, already-documented contract. |

#### Task 1.3 — Test-fixture transport
**Commit 4** · `test(fifo): add LogReplayTransport for fixture-driven tests` · `v16.6.4-fifo`

| | |
|---|---|
| Purpose | Enable host-side testing of L2/L3 against real captured byte streams, turning the 7 recorded hardware logs (including all 7 failures) into a regression suite — directly answering PRR **C-14**. |
| Files affected | `test/log_replay_transport.h`, `test/log_replay_transport.cpp` (new, host-only) |
| Public interfaces | ```c\nvoid LogReplayTransport_Init(FifoTransport* out, const uint8_t* recordedBytes, size_t len, const uint32_t* byteTimestampsMs);\n``` — replays a captured byte stream with its recorded timing, so timeout logic is exercised realistically, not just data correctness. |
| Internal interfaces | Internal cursor/position state, not exposed. |
| Risks | Must faithfully reproduce timing gaps (K-13: ≤2 ms mid-dump, up to 239 ms across poll boundaries) or timeout-path tests (Phase 2.4, 3.6) will be testing the wrong thing. Extracting per-byte timestamps from the existing hex-dump-style logs requires a one-time conversion script (Phase 2.4's first sub-task), not hand-transcription. |
| Verification method | Self-test: replay a known-good captured dump, confirm the decoded samples and timing match what the original tool's own printed CRC-verified output already established. |
| Rollback strategy | Delete both files. Test-only; zero production impact by construction (§2.4). |
| Dependencies | Task 1.1. |

---

### Phase 2 — L2 Frame Codec

#### Task 2.1 — Streaming CRC
**Commit 5** · `feat(fifo): add StreamingCrc16 (CRC-16/Modbus, byte-sequential)` · `v16.6.5-fifo`

| | |
|---|---|
| Purpose | CRC-16/Modbus, accumulated one byte at a time, no buffering — the mechanism that lets Phase 3 eliminate the 6144B staging buffer entirely (SDS D-11, PRR **C-4**/**C-5**). |
| Files affected | `fifo_codec.h` (new, this task adds the CRC portion), `fifo_codec.cpp` |
| Public interfaces | ```c\nvoid     Crc16_Init(uint16_t* state);\nvoid     Crc16_Update(uint16_t* state, uint8_t b);\nuint16_t Crc16_Final(uint16_t state);\n``` |
| Internal interfaces | None. |
| Risks | Must match, byte-for-byte, the algorithm already validated against 12 real captures (poly `0xA001`, init `0xFFFF`, low-byte-first on wire — PRR K-6/K-7). Any deviation is caught immediately by Task 2.4's fixture tests. |
| Verification method | Unit test against known vectors + at least one full recorded 6146B frame from the fixture corpus, confirming the computed CRC matches the transmitted CRC the original tool already verified. |
| Rollback strategy | Delete the CRC functions from `fifo_codec.{h,cpp}`; nothing downstream exists yet. |
| Dependencies | None (pure logic, no transport needed). |

#### Task 2.2 — Sample decoder
**Commit 6** · `feat(fifo): add SampleDecoder (BE int16, no scaling)` · `v16.6.6-fifo`

| | |
|---|---|
| Purpose | Decode payload bytes to raw `int16` tri-axial counts, incrementally, one 6-byte stride at a time — no staging buffer, no scaling applied (SDS D-9: the g-scale factor, K-9, is evidence-backed but **not confirmed**, so the driver stores counts and leaves scaling to the consumer). |
| Files affected | `fifo_codec.h`, `fifo_codec.cpp` |
| Public interfaces | ```c\nvoid SampleDecoder_DecodeStride(const uint8_t* sixBytes, int16_t* outX, int16_t* outY, int16_t* outZ);\n``` — pure function, one 6-byte group in, one sample triple out. |
| Internal interfaces | None. |
| Risks | Byte order (big-endian) and stride order (X,Y,Z) must match PRR K-8 exactly — verified against the fixture corpus, not re-derived. |
| Verification method | Unit test: feed one recorded frame's payload through incrementally (simulating arrival one byte at a time via the fixture transport), confirm the resulting 1024×3 samples exactly match the values the original tool's own `dumpFIFOToSerialCSV()` output already recorded for that capture. |
| Rollback strategy | Delete the decoder function. |
| Dependencies | None (pure logic). |

#### Task 2.3 — Frame codec (shape discrimination, bounded resync, error classification)
**Commit 7** · `feat(fifo): add FrameCodec — anchor scan, shape discrimination, error taxonomy` · `v16.6.7-fifo`

| | |
|---|---|
| Purpose | The core protocol-level logic: recognize the `50 03` anchor with a **bounded** resync (fixes PRR **C-6**), discriminate `Len=0x01` (progress) vs `Len=0x00` (dump) per K-5, and return one of the domain-classified error codes (SDS D-18, fixing PRR **C-7**) rather than one overloaded value. |
| Files affected | `fifo_codec.h`, `fifo_codec.cpp` |
| Public interfaces | ```c\nenum class FifoFrameOutcome { PROGRESS, FULL_DUMP, DESYNC_LIMIT, CRC_MISMATCH, BAD_TYPE_BYTE, TIMEOUT_NO_RESPONSE, TIMEOUT_INTER_BYTE };\n\nstruct FrameCodecState {   // [v1.1, Freeze Review 3.1] explicit, caller-owned, never hidden/static\n  uint32_t desyncBytesDiscarded;\n  uint32_t partialBytesReceived;\n  uint8_t  partialBuf[6];        // largest partial unit this codec ever assembles mid-call\n};\nvoid FrameCodecState_Reset(FrameCodecState* s);   // called by L3 at the start of each new frame attempt\n\nFifoFrameOutcome FrameCodec_Step(FifoTransport* t, FrameCodecState* state, uint16_t* outProgressFill, int16_t* xOut, int16_t* yOut, int16_t* zOut);\n``` **[v1.1]** Two corrections from the original draft: (a) the scan/parse state is the explicit `FrameCodecState` above, owned by the caller (L3's session struct, SDS §11.4) and passed by pointer — not hidden inside this file; (b) the dump-decode targets are plain `int16_t*` write pointers (matching Task 2.2's `SampleDecoder_DecodeStride` exactly), **not** `FifoArena*` — this file never `#include`s `fifo_arena.h`. L3's `S8_ReadDump` handler (Task 3.3) passes `FifoArena_WriteHandleX/Y/Z()`'s return values as `xOut/yOut/zOut`. |
| Internal interfaces | None held internally — all scan/parse state lives in the caller-supplied `FrameCodecState` (above). `MAX_DESYNC_BYTES = 64` (SDS §17.2) is checked against `state->desyncBytesDiscarded`. |
| Risks | **Must never block.** This is the function PRR C-1 exists because of — every internal loop must be bounded to "consume what's currently available, return." Reviewed explicitly against the SDS §11 state table (each state = one bounded action) before merge. |
| Verification method | Fixture tests covering: a clean progress frame, a clean dump (CRC match), a corrupted CRC (fixture: synthetically flip one payload bit), a truncated stream (fixture: recorded log cut short), a stray-byte-prefixed frame (recorded in the original logs — e.g. the `50 03 01 0B D3 F3` "unsolicited bytes" pattern already on record), an injected `50 03` inside payload data (synthetic — tests the `MAX_DESYNC_BYTES` bound specifically, since real logs don't happen to contain this case). |
| Rollback strategy | This is the highest-value, highest-risk L2 task; if it needs rework, the interface (Task 1.1) and lower-level pieces (2.1, 2.2) are unaffected — revert this commit alone. |
| Dependencies | Tasks 1.1, 2.1, 2.2. |

#### Task 2.4 — L2 fixture test suite
**Commit 8** · `test(fifo): fixture regression suite for FrameCodec (7 recorded logs)` · `v16.6.8-fifo`

| | |
|---|---|
| Purpose | Convert the 7 recorded hardware logs into host-runnable regression fixtures, closing PRR **C-14**. |
| Files affected | `test/fixtures/*` (converted logs + a one-time conversion script, not shipped), `test/test_fifo_codec.cpp` (new) |
| Public interfaces | None (test binary). |
| Internal interfaces | None. |
| Risks | The **first sub-task of this commit is to confirm the "safe subdirectory" assumption from §1** — run `arduino-cli compile` on the production sketch after `test/` exists and contains these files, confirm the build output is unchanged (same flash/RAM figures as the current clean baseline) and no test code appears in the binary. This must pass before any other Phase 2/3 test content is trusted. |
| Verification method | `test/test_fifo_codec.cpp` run via host `g++`/CTest (or equivalent), all fixtures pass; `arduino-cli compile` of the sketch confirmed unaffected (see Risks). |
| Rollback strategy | Delete `test/` contents; zero production impact by construction. |
| Dependencies | Tasks 1.3, 2.1, 2.2, 2.3. |

---

### Phase 3 — L3 Capture Session

#### Task 3.1 — Shared types
**Commit 9** · `feat(fifo): add shared types (FifoPhase, FifoError, request/result structs)` · `v16.6.9-fifo`

| | |
|---|---|
| Purpose | Single source of truth for the types every other file needs, declared once, included early — this is what avoids the `.ino` auto-prototype forward-declaration pattern PRR flagged as a code smell in the Validation Tool. |
| Files affected | `fifo_types.h` (new) |
| Public interfaces | ```c\nenum class FifoPhase { IDLE, ACTIVE, RESULT_READY, COOLDOWN, DISABLED };\n\nenum class FifoError {\n  NONE,\n  ERR_NO_RESPONSE, ERR_INTER_BYTE_TIMEOUT, ERR_DESYNC_LIMIT, ERR_RX_OVERFLOW,\n  ERR_CRC_MISMATCH, ERR_BAD_TYPE_BYTE, ERR_PROGRESS_REGRESSION, ERR_PROGRESS_OVERRUN,\n  ERR_BUSY, ERR_NOT_PERMITTED, ERR_RESULT_NOT_RELEASED, ERR_ABORTED,\n  ERR_RETRY_EXHAUSTED, ERR_CIRCUIT_OPEN\n};  // 14 codes, 3 domains — SDS §16.1\n\nenum class FifoTriggerSource { FAULT_LATCH, OPERATOR_BUTTON, SCHEDULED, COMMISSIONING };  // REMOTE_ON_DEMAND intentionally absent — G-2\n\nstruct FifoAdmissionContext {   // [v1.1, Freeze Review 3.2] caller-populated, plain data\n  bool motorStable;              // from g_systemState, read by the caller, never by fifo_driver.cpp\n  bool sensorHealthy;            // from g_modbusConsecErrors == 0\n  bool mqttReconnecting;         // from network task state\n};\n\nstruct FifoCaptureRequest {\n  FifoTriggerSource    triggerSource;\n  char                 tag[16];\n  bool                 requirePermissive;\n  uint8_t              maxRetries;\n  FifoAdmissionContext admissionContext;  // [v1.1] Request()'s admission check reads only this — never a global directly\n};\n\nstruct FifoCaptureResult {\n  uint32_t       captureId;\n  char           tag[16];\n  FifoTriggerSource triggerSource;\n  FifoPhase      status;      // OK path represented via a dedicated bool, see coding-time note\n  FifoError      error;\n  uint16_t       sampleCount;\n  uint16_t       srIndexAtCapture;\n  uint32_t       srHz;\n  uint32_t       tRequestMs, tCompleteMs;\n  float          tempCAtCapture;\n  uint8_t        motorStateAtCapture;\n  float          rpmAtCapture;\n  const int16_t* x; const int16_t* y; const int16_t* z;  // valid only while held AND error==NONE\n  uint16_t       pollCount, progressFrameCount, crcErrorCount, retryCount;\n  uint16_t       lastProgressFill;\n  uint32_t       tFirstByteMs, tFirstProgressMs, tLastProgressMs, tAnchorAfterLastProgressMs;\n  uint32_t       desyncBytesDiscarded;\n};\n\nstruct FifoDriverStats { /* lifetime counters — finalized at coding time */ };\n``` |
| Internal interfaces | None — pure declarations. |
| Risks | This is the file every other file depends on; a late change here ripples widely. Reviewed carefully before merge specifically because of that blast radius — this is the one task where "small commit" is weighed against "get the shared contract right the first time." |
| Verification method | Compiles standalone (host `g++`) and as part of the sketch. No behavior to test — pure declarations. |
| Rollback strategy | High cost if downstream files already depend on it — this is why it is reviewed most carefully of any single task in Phase 3, not why it's risky to revert per se. |
| Dependencies | None. |

#### Task 3.2 — Sample arena and ownership handoff
**Commit 10** · `feat(fifo): add FifoArena — single static buffer, exclusive ownership handoff` · `v16.6.10-fifo`

| | |
|---|---|
| Purpose | The single 6144B static arena and the strict ownership-transfer protocol that makes PRR **C-2** (stale data survives a failed capture) structurally impossible, per SDS D-10. |
| Files affected | `fifo_arena.h`, `fifo_arena.cpp` (new) |
| Public interfaces | ```c\nvoid           FifoArena_Reset();\nint16_t*       FifoArena_WriteHandleX();  // driver-internal write access, only during S8\nint16_t*       FifoArena_WriteHandleY();\nint16_t*       FifoArena_WriteHandleZ();\nbool           FifoArena_TryAcquire(const int16_t** outX, const int16_t** outY, const int16_t** outZ);\nvoid           FifoArena_Release();\nbool           FifoArena_IsHeld();\nuint32_t       FifoArena_HeldSinceMs();  // for the §17.5 watchdog\n``` |
| Internal interfaces | The static `int16_t x[1024], y[1024], z[1024]` storage itself — never exposed directly, only through the handles above. |
| Risks | This is the object PRR C-2 is about — every access path must be reviewed against the question "can this be reached when `error != NONE`?" The answer must always be no. |
| Verification method | Unit test: acquire-without-release is rejected on a second acquire attempt; write handles are inaccessible outside the driver (compile-time — these are not part of the public `fifo_driver.h` surface); a released-then-reacquired arena from a *new* capture never exposes the *previous* capture's data unless that new capture also wrote it. |
| Rollback strategy | Delete both files; nothing outside Phase 3 references this yet. |
| Dependencies | Task 3.1. |

#### Task 3.3 — Core protocol state machine (S0–S9)
**Commit 11** · `feat(fifo): implement FifoDriver core state machine (request through verify)` · `v16.6.11-fifo`

| | |
|---|---|
| Purpose | States `UNINIT → IDLE → ARMED → REQUEST → AWAIT_ANCHOR → READ_TYPE → READ_PROGRESS/READ_DUMP → POLL_WAIT → VERIFY`, per SDS §11. This is the heart of the tick-sliced design (D-1) that satisfies the 6.4 s-transfer-vs-250 ms-tick constraint. |
| Files affected | `fifo_driver.h` (internal-only symbols not yet exported), `fifo_driver.cpp` |
| Public interfaces | None yet — this task is internal; Task 3.5 exposes the public API. |
| Internal interfaces | ```c\nstatic void FifoDriver_ServiceCoreState(FifoTransport* t);  // one bounded action, called from the eventual FifoDriver_Service()\n``` plus one static handler per state (`S3_Request`, `S4_AwaitAnchor`, ... `S9_Verify`), each consuming `FrameCodec_Step()` (Task 2.3) and advancing internal state — no state may itself contain an unbounded loop. The session struct owns one `FrameCodecState` instance (Task 2.3, `[v1.1]`), reset at the start of each new frame attempt and passed by pointer into every `FrameCodec_Step()` call — never held inside `fifo_codec.cpp`. **`S6_ReadProgress` [v1.1, Freeze Review 7.1]** additionally compares the incoming `fill` against the session's `lastProgressFill` (already tracked for diagnostics, SDS §8.2): `fill < lastProgressFill` → `ERR_PROGRESS_REGRESSION`; `fill > 6144` → `ERR_PROGRESS_OVERRUN` (both per K-10, SDS §11.4/§16.1); otherwise `lastProgressFill` is updated and the state machine proceeds. This comparison is session-level (persists across multiple `S6` visits within one attempt), distinct from `FrameCodecState` (per-frame, reset each attempt) — the two must not be conflated. |
| Risks | Same class of risk as Task 2.3 (must never block) — reviewed against the same per-state-bounded-action checklist. The `FIFO_PROTOCOL_MODEL` constant (D-14, G-1) lives here, on the single `S6→S3 or S6→S4` edge — implemented as one `#if`/`if` on that one named constant, nothing else in this file branches on protocol model. |
| Verification method | Fixture-driven: `LogReplayTransport` feeding a recorded successful capture drives the state machine from `ARMED` to `VERIFY` with `error==NONE`; a recorded failed capture drives it to `VERIFY` with the matching error code. |
| Rollback strategy | Revert this commit; Tasks 3.1/3.2/2.x are unaffected (nothing yet calls into this file publicly). |
| Dependencies | Tasks 2.3, 3.1, 3.2. |

#### Task 3.4 — Recovery/lifecycle states and timeout/retry policy
**Commit 12** · `feat(fifo): implement recovery states (drain, cooldown, retry, circuit breaker)` · `v16.6.12-fifo`

| | |
|---|---|
| Purpose | States `DRAIN → RESULT_READY → COOLDOWN`, plus `FAILED → DISABLED`; the named timeout constants (SDS §15, D-17); bounded retry with mandatory bus release between attempts (§17.3, justified by telemetry-availability and sensor-settling, per the corrected §15.3 `[v1.1]`); the result-hold watchdog (§17.5); the circuit breaker (§17.6). |
| Files affected | `fifo_driver.cpp` |
| Public interfaces | None yet (Task 3.5). |
| Internal interfaces | ```c\nstatic void FifoDriver_ServiceRecoveryState(FifoTransport* t);\n``` plus the named constants: `T_REQUEST_RESPONSE_MS=1000`, `T_INTER_BYTE_MS=500`, `T_POLL_INTERVAL_MS=250`, `T_DUMP_TOTAL_MS=9000`, `T_CAPTURE_TOTAL_MS=20000`, `T_DRAIN_QUIET_MS=300`, `T_COOLDOWN_MS=60000`, `T_RESULT_HOLD_MAX_MS=30000`, `MAX_DESYNC_BYTES=64`, `FIFO_MAX_RETRIES=2`, `FIFO_BREAKER_THRESHOLD=5`. |
| Risks | **[v1.1, Freeze Review Finding 5.1 — corrected]** The original Risks text here cited a "`T_CAPTURE_TOTAL_MS=20000` vs `ABSENT_STOPPING_MS=15000`" coupling. That coupling does not exist: `ABSENT_STOPPING_MS` is driven by RPM pulse-ISR freshness (`signalPresent`), confirmed independent of the WTVB05 Modbus bus (SDS §13.3, §15.3 `[v1.1]`) — a FIFO capture cannot trigger it regardless of duration. The retry-release-between-attempts requirement is retained (it is still real and still required), but the review item for this commit is now: confirm the bus-release/re-acquire sequence occurs between attempts for telemetry-availability reasons, and confirm `g_modbusConsecErrors` (the one genuinely Modbus-coupled timer, CN-10) is not incremented at any point while `OwnsBus()` — that check belongs to Task 4.3, cross-referenced here since it's the actual safety-relevant constraint in this area. No numeric constant changes. |
| Verification method | Fixture test: simulate a CRC failure followed by a successful retry, confirm the bus-release/re-acquire sequence occurs between attempts (observable via the transport's `flushRx`/timing calls). Separate test: 5 consecutive simulated failures trip the breaker (`DISABLED`), and a 6th request is rejected with `ERR_CIRCUIT_OPEN` without touching the transport at all. |
| Rollback strategy | Revert this commit. Task 3.3's core protocol states are self-contained and unaffected (they hand off to `VERIFY`; what happens after is entirely this commit's addition). |
| Dependencies | Task 3.3. |

#### Task 3.5 — Public API
**Commit 13** · `feat(fifo): expose FifoDriver public API (Init/Request/Service/...)` · `v16.6.13-fifo`

| | |
|---|---|
| Purpose | The 9-entry-point public surface (SDS §7), the thinnest possible wrapper over Tasks 3.3/3.4's internal state machine. |
| Files affected | `fifo_driver.h` (adds the public section) |
| Public interfaces | ```c\nvoid          FifoDriver_Init(const FifoDriverConfig* cfg);\nFifoError     FifoDriver_Request(const FifoCaptureRequest* req, uint32_t* outHandle);\nvoid          FifoDriver_Service();          // taskModbusRead ONLY, once per 250ms tick, never blocks\nFifoPhase     FifoDriver_GetPhase();\nbool          FifoDriver_OwnsBus();          // the bus-arbitration primitive, SDS §13.2\nbool          FifoDriver_TryAcquireResult(FifoCaptureResult* outResult);\nvoid          FifoDriver_ReleaseResult();\nvoid          FifoDriver_Abort(FifoError reason);\nvoid          FifoDriver_GetStats(FifoDriverStats* outStats);\n``` |
| Internal interfaces | Depth-1 FreeRTOS queue for `Request()` submission (any task → `taskModbusRead`); one narrow mutex (`mutexFifoResult`) held only for the pointer-exchange in `TryAcquireResult`/`ReleaseResult`, never across consumer processing — per SDS D-16, this is the only new synchronization primitive in the entire driver. |
| Risks | `Service()`'s "never blocks, single-caller-only" contract (SDS §7.2, §14.3) is the single most safety-critical property in this file. Reviewed with an explicit checklist item: does any code path in `Service()` call anything that can wait (a semaphore take with nonzero timeout, `delay`, `vTaskDelay`, a blocking write longer than the accepted 8-byte request exception)? A debug-build assertion on caller-task-handle is included specifically to catch a future accidental second caller. **[v1.1, Freeze Review Finding 3.2]** `Request()`'s admission check reads only `req->admissionContext` (Task 3.1) — this file must never `#include` or reference any `.ino`-scope global (`g_systemState`, `g_modbusConsecErrors`, etc.) directly; that boundary is what keeps this file host-testable independent of the 8,343-line `.ino`. |
| Verification method | Fixture test exercising every public entry point's stated contract (admission gates rejecting correctly per §16.1's lifecycle-domain codes; `ReleaseResult` required before a new `Request` succeeds; `Abort` still performs the bus drain). |
| Rollback strategy | Revert this commit; internal state machine (3.3, 3.4) remains intact and testable via its own internal test hooks if any exist, or simply un-exercised until this is re-landed. |
| Dependencies | Tasks 3.1–3.4. |

#### Task 3.6 — L3 fixture + fault-injection test suite
**Commit 14** · `test(fifo): FifoDriver integration tests (state machine, retry, breaker, fault injection)` · `v16.6.14-fifo`

| | |
|---|---|
| Purpose | Close the loop on PRR **C-14** for the whole L3 driver, and specifically exercise the fault classes the Validation Tool could never test because its parser was inseparable from `SerialRS485.read()` (PRR-noted limitation). |
| Files affected | `test/test_fifo_driver.cpp` (new) |
| Public interfaces | None (test binary). |
| Internal interfaces | None. |
| Risks | Coverage gap risk: fault-injection cases not present in the 7 recorded logs (e.g., `fill` regression, `fill` exceeding 6144 — K-10's invariant, `ERR_PROGRESS_REGRESSION`/`ERR_PROGRESS_OVERRUN`) must be synthesized, not just replayed — explicitly listed as required cases, not left implicit. |
| Verification method | This task **is** the verification method for Phase 3 as a whole — see Phase 3 DoD, §7. |
| Rollback strategy | Delete `test/test_fifo_driver.cpp`; zero production impact by construction. |
| Dependencies | Tasks 1.3, 3.5. |

---

### Phase 4 — `taskModbusRead` Integration (Bus Ownership)

#### Task 4.1 — Wire `FifoDriver_Init()` into `setup()`
**Commit 15** · `feat(fifo): initialize FifoDriver at boot` · `v16.6.15-fifo`

| | |
|---|---|
| Purpose | Bind the production `Uart485Transport` (Task 1.2) to the driver at boot, after `modbus.begin()`, before any FreeRTOS task starts. |
| Files affected | The existing `.ino` — `#include "fifo_types.h"`, `#include "fifo_transport_uart485.h"`, `#include "fifo_driver.h"` near the top (this is what avoids the auto-prototype problem, §1); one `Uart485Transport_Init(...)` + `FifoDriver_Init(...)` call added to `setup()`, after `SerialRS485.begin()`/`modbus.begin()`. |
| Public interfaces | Consumes `FifoDriver_Init` (3.5), `Uart485Transport_Init` (1.2). Introduces none new. |
| Internal interfaces | None. |
| Risks | Must not perform any sensor I/O at init (SDS §12.1 — presence is already established by CM-100's own boot probe; a redundant probe here would be an unowned side effect, D-3). Reviewed explicitly against that one-line rule. |
| Verification method | Clean build; boot log confirms `FifoDriver_GetPhase() == IDLE` immediately after init (a diagnostic assertion, removed or gated before this commit closes per §2.4's rule on temporary debug code) and confirms zero additional Modbus traffic at boot versus the current baseline (`serial_capture_test7_soak_20260727.log` as the reference trace). |
| Rollback strategy | Delete the three `#include` lines and the two init calls. Zero effect on any other boot behavior. |
| Dependencies | Tasks 1.2, 3.5. |

#### Task 4.2 — Mode branch in `taskModbusRead()`
**Commit 16** · `feat(fifo): integrate FifoDriver_Service into taskModbusRead (bus ownership, DESIGN-0002 Option A)` · `v16.6.16-fifo`

| | |
|---|---|
| Purpose | The single highest-risk change in this plan: implement SDS §13.2's integration shape — `FifoDriver_Service()` called unconditionally at the top of the existing 250 ms tick, then a branch on `FifoDriver_OwnsBus()` that suspends normal WTVB02+CTR4A01 polling while true. This adopts DESIGN-0002 Option A (no new mutex, `taskModbusRead` remains sole `modbus.*` caller) per the approved SDS D-15 (unchanged). **[v1.2/v1.3]** This task also owns RS485 EN-pin bracketing (SDS §13.2.1, `ADR-1`) — `rs485Enable()` on entry to the `OwnsBus()` branch, `rs485Disable()` when bus ownership releases. This is this task *satisfying* the precondition documented on Task 1.2's `write()` contract (SDS §6.1) at integration time — not a dependency running in the other direction. |
| Files affected | The existing `.ino`, inside `taskModbusRead()`'s loop body only. |
| Public interfaces | Consumes `FifoDriver_Service()`, `FifoDriver_OwnsBus()`. |
| Internal interfaces | Calls the existing, unmodified `rs485Enable()`/`rs485Disable()` (`static inline`, same translation unit — no linkage issue here, unlike the constant Task 1.2 previously duplicated). |
| Risks | **This is the one commit in the entire plan that touches proven, working, already-shipped polling logic.** Kept to the smallest possible diff: one unconditional call added at the top of the loop, one `if`/`else` wrapping the *existing* polling code (not rewriting it) — the existing WTVB02/CTR4A01 polling body moves inside the `else` branch unchanged, line for line. Must also suspend CTR4A01 polling (`readCTR4A01Current()`), not just WTVB05 — both share the bus per CN-6, confirmed in source (`modbus.begin()` re-addressing). `rs485Disable()` must fire on **every** path that releases `OwnsBus()` — normal completion (`S11`→`S12` cooldown), `FifoDriver_Abort()`, and the circuit breaker tripping (`S14`) all release the bus and must all reach the disable call. A path that releases bus ownership without disabling EN would leave the transceiver enabled indefinitely, which is not itself unsafe (§13.2.1: normal polling unconditionally reasserts LOW at its own next cycle regardless of prior state) but is a correctness gap versus this task's own stated symmetric-bracketing goal and must not be introduced silently. |
| Verification method | On-target integration test (not fixture — this must run on real hardware): trigger a capture, confirm via serial log that normal polling visibly pauses and resumes; confirm `StateMachine` task (priority 4) shows no missed cadence via its own existing diagnostics; confirm no crash, no watchdog trip, across at least 10 triggered captures. This is the first test in the whole plan that requires hardware, and the first that requires the WTVB05 bus to be in the condition established by the Reproducibility Study (currently healthy, 0/16 failures). Add: confirm `rs485Disable()` fires after a capture triggered via `Abort()` mid-session and after a simulated circuit-breaker trip, not only after normal completion; confirm Task 1.2's precondition (bus owned + EN asserted before `write()`) is actually met, per Task 1.2's Verification method. |
| Rollback strategy | Revert this single commit. The `else` branch is the exact prior loop body, so reverting restores `taskModbusRead()` to byte-identical prior behavior. This is the reason the "move existing code into an `else`, don't rewrite it" approach was chosen over a larger refactor. |
| Dependencies | Task 4.1. **[v1.3]** Not depended upon by Task 1.2 — Task 1.2 is already complete against its own documented contract. This task satisfies that contract at integration; the dependency runs Task 4.2 → Task 1.2's contract, not the reverse. |

#### Task 4.3 — `g_modbusConsecErrors` non-increment while `OwnsBus()`
**Commit 17** · `fix(fifo): prevent false Modbus-offline detection during FIFO capture` · `v16.6.17-fifo`

| | |
|---|---|
| Purpose | CN-10 compliance: `MODBUS_OFFLINE_THRESHOLD=3` must not fire from a deliberate, driver-owned bus suspension. |
| Files affected | The existing `.ino`, at the `g_modbusConsecErrors` increment site(s) inside the `else` branch established in Task 4.2 (no change needed inside the `if (OwnsBus())` branch, since that branch never touches `g_modbusConsecErrors` by construction — this task verifies and documents that, and adds an explicit guard only if a review finds a path that could touch it). |
| Public interfaces | None. |
| Internal interfaces | None. |
| Risks | Low — this task may turn out to require zero code change if Task 4.2's branch structure already fully isolates the counter (likely, given the counter only lives in the polling code that moved into `else`). Kept as its own commit regardless, so the review record explicitly shows this specific SDS requirement (CN-10) was checked, not assumed. |
| Verification method | Code inspection confirming no path from `if (OwnsBus())` reaches the counter; on-target test confirming `g_modbusConsecErrors` reads 0 (or unchanged) across a triggered capture. |
| Rollback strategy | Trivial — either no code changed, or a small guard is removed. |
| Dependencies | Task 4.2. |

#### Task 4.4 — Full on-target integration verification
**Commit 18** (verification commit — test artifacts + capture script, not driver code) · `test(fifo): on-target integration verification for Phase 4` · `v16.6.18-fifo`

| | |
|---|---|
| Purpose | Systematic on-target confirmation that Phase 4 satisfies every SDS §13.3 consequence: telemetry gap handling, absence-timer safety, RPM continuity, CTR4A01 gap handling, trend/analytics continuity, display indication readiness. |
| Files affected | New capture script in the sketch directory, following the project's existing `capture_p4_02_verification_20260727.ps1` pattern — `capture_fifo_p1_verification.ps1`. No driver code changes. |
| Public interfaces | None. |
| Internal interfaces | None. |
| Risks | This is a verification task, not a code task — the risk is in *not* running it thoroughly enough, not in the artifact itself. |
| Verification method | This task **is** the verification method — see Phase 4 DoD, §7. |
| Rollback strategy | N/A — capture scripts only. |
| Dependencies | Tasks 4.1–4.3. |

---

### Phase 5 — Telemetry Integration (Additive Only)

#### Task 5.1 — `capture_active` field
**Commit 19** · `feat(telemetry): add capture_active field to /sensor and /vibration (additive)` · `v16.6.19-fifo`

| | |
|---|---|
| Purpose | Make the ~7 s telemetry gap during a capture explainable rather than alarming (SDS §18.2), reusing v16.3aa FREEZE/`trend_gap_s` semantics rather than inventing new ones. |
| Files affected | The existing `.ino`: `TelemetrySnapshot` struct (add one `bool capture_active` field, mirroring the pattern of `currentEvidenceValid` from the P4-02 commit `322ef0b` exactly — same additive-mirror technique, already proven safe in production), `captureTelemetrySnapshot()`, `publishTelemetry()`. |
| Public interfaces | New MQTT field `capture_active` on `/sensor` and `/vibration`. Additive only — `CLAUDE.md`'s backward-compat rule: never remove a field. |
| Internal interfaces | `snap.capture_active = (FifoDriver_GetPhase() == FifoPhase::ACTIVE);` — pure read, no computation, mirroring the P4-02 commit's own "pure copy, no computation" pattern exactly. |
| Risks | Low — this is a template-following change; the P4-02 commit (`322ef0b`) is the direct, already-shipped, already-soak-tested precedent for this exact pattern (file-scope mirror written at one site, read at capture-snapshot time). |
| Verification method | MQTT payload inspection during a triggered capture, confirming the field toggles correctly and that no existing field's value or type changed. |
| Rollback strategy | Delete the one field, one write site, one publish line. |
| Dependencies | Task 4.2 (needs `FifoDriver_GetPhase()` to be meaningful on real hardware). |

#### Task 5.2 — `/event` capture-result publication
**Commit 20** · `feat(telemetry): publish FIFO capture result to /event (every session, success and failure)` · `v16.6.20-fifo`

| | |
|---|---|
| Purpose | Every capture session — success **and** failure — publishes full diagnostics (SDS §8.2/§18.3). At the old tool's 63% baseline (K-15, now itself an open question per §0/G-1), failures are the common case and the more informative one; publishing only successes would produce survivorship-biased field data. **[v1.1, Freeze Review Finding 2.1]** This task establishes `handleFifoCaptureCompletion()` — **the sole call site** for `TryAcquireResult`/`ReleaseResult` (SDS §9.1) — with the `/event` publish as its first (for now, only) internal step. Task 7.1, if undertaken, adds a second internal step to this *same* function; it must not create its own acquire/release pair. |
| Files affected | The existing `.ino`: `handleFifoCaptureCompletion(FifoDriverStats* /*unused for now*/)` — called once per session transition to `RESULT_READY` — containing `TryAcquireResult()` → `publishFifoCaptureEvent(&result)` → `ReleaseResult()`, plus the `publishFifoCaptureEvent()` helper itself following the existing `/event` publish pattern. |
| Public interfaces | New `/event` message type, additive per `RFC-0002`'s topic contract. |
| Internal interfaces | `handleFifoCaptureCompletion()` and `publishFifoCaptureEvent()` — both internal to this file. `publishFifoCaptureEvent()` is a pure formatter, callable with any `FifoCaptureResult`; it never itself calls `TryAcquireResult`/`ReleaseResult` — only `handleFifoCaptureCompletion()` does. |
| Risks | Payload size: metadata + diagnostics estimated ~400–500B JSON (SDS §18.3), against the existing 2200B document budget (`MQTT_OUTBOUND_PAYLOAD_MAX`) — must be measured, not assumed, once the actual field set is finalized in coding. **[v1.1]** Also: `handleFifoCaptureCompletion()` must remain the *only* place in the codebase that calls `TryAcquireResult`/`ReleaseResult` — this is a standing constraint for every future task that needs the result (see Task 7.1), not just this one. |
| Verification method | Triggered capture (success and a synthetically-forced failure, e.g. via `Abort()`) both produce a well-formed `/event` payload; payload size measured and confirmed within budget. |
| Rollback strategy | Delete `handleFifoCaptureCompletion()`, `publishFifoCaptureEvent()`, and the one call site. |
| Dependencies | Task 4.2. |

---

### Phase 6 — Diagnostic Workflow / Trigger Wiring

#### Task 6.1 — Local trigger sources
**Commit 21** · `feat(fifo): wire FAULT_LATCH, OPERATOR_BUTTON, SCHEDULED, COMMISSIONING triggers` · `v16.6.21-fifo`

| | |
|---|---|
| Purpose | Connect the four in-scope trigger sources (SDS §19.1) to `FifoDriver_Request()`. `REMOTE_ON_DEMAND` is explicitly **not** implemented here — blocked on G-2 (§0), tracked, not silently dropped. |
| Files affected | The existing `.ino`: fault-latch transition handler, button long-press handler (existing `PIN_BUTTON_ENTER` logic), a new periodic-schedule check (Analytics or State Machine task cadence), Config Mode's existing 5-second boot window. |
| Public interfaces | None new — each of these calls the existing `FifoDriver_Request()` (Task 3.5). |
| Internal interfaces | Per-source small glue functions, e.g. `maybeTriggerFifoOnFaultLatch()`. |
| Risks | Each trigger site is a small, independent addition to existing, working handlers — reviewed individually to confirm none of them can block or alter the existing handler's own timing-critical behavior (e.g., the button handler's existing debounce logic must be unaffected). |
| Verification method | On-target: exercise each trigger source individually, confirm `FifoDriver_Request()` is called with the correct `FifoTriggerSource`, confirm no regression to the existing behavior each trigger site previously had (fault-latch logging, button debounce, Config Mode timing). |
| Rollback strategy | Each trigger site can be reverted independently if one causes an issue — not bundled into a single all-or-nothing commit conceptually, though landed as one commit here since each is a small, low-risk addition; split into per-source commits at coding time if review prefers. |
| Dependencies | Task 3.5, Task 4.2 (bus must be integrated before triggers can meaningfully fire). |

#### Task 6.2 — Admission gates
**Commit 22** · `feat(fifo): implement capture admission gates (motor state, sensor health, cooldown)` · `v16.6.22-fifo`

| | |
|---|---|
| Purpose | SDS §19.2's gate set — reject a capture rather than take one that would be diagnostically meaningless (mid-transition motor state) or risky (bus already unhealthy). **[v1.1, Freeze Review Finding 3.2 — decided, no longer a coding-time open question]** Application-state gate inputs are read by each of Task 6.1's trigger sites (which already have natural access to `g_systemState`/`g_modbusConsecErrors`/MQTT state) and passed in via `FifoCaptureRequest.admissionContext` (Task 3.1). `fifo_driver.cpp` never references a `.ino` global directly. |
| Files affected | The existing `.ino`, at each of Task 6.1's trigger sites: populate `req.admissionContext = { motorStable, sensorHealthy, mqttReconnecting };` immediately before calling `FifoDriver_Request(&req, ...)`. `fifo_driver.cpp`: an internal (non-public) helper reads `req->admissionContext`. |
| Public interfaces | None new — `FifoDriver_Request()`'s signature is unchanged. |
| Internal interfaces | `static bool FifoDriver_CheckAdmissionGates(const FifoAdmissionContext* ctx);` — internal to `fifo_driver.cpp`, called from `Request()`'s `S1→S2` admission check, checking `ctx->motorStable`, `ctx->sensorHealthy`, `!ctx->mqttReconnecting`, plus the driver's own internal gates (idle, breaker closed, cooldown elapsed, no result held) already covered by `Request()`'s existing contract. Takes plain data only — host-testable with a synthetic `FifoAdmissionContext`, no `.ino` global needs to exist for this function's tests to compile or run. |
| Risks | Each trigger site (Task 6.1) must construct `admissionContext` correctly — reviewed per-site alongside Task 6.1's own review, since a wrong value here (e.g. reading `g_modbusConsecErrors` before vs. after the current cycle's update) is a data-freshness bug, not an architecture bug. |
| Verification method | Fixture test (host-side, `[v1.1]`): `FifoDriver_CheckAdmissionGates()` against synthetic contexts covering each gate's pass/fail boundary — no hardware needed. On-target: attempt a capture during motor warm-up (expect `ERR_NOT_PERMITTED`), during stable running (expect admission), confirm gate checks add negligible latency to `Request()`. |
| Rollback strategy | Revert; `Request()` falls back to its Phase 3.5 baseline (lifecycle-only gates), no motor/sensor-state gating. |
| Dependencies | Task 6.1, Task 3.1 (`admissionContext` field). |

---

### Phase 7 — Waveform Egress (Deferred / Optional)

#### Task 7.1 — FAT storage
**Commit 23** (deferred — requires explicit go-ahead before starting, see below) · `feat(fifo): store captured waveform to FAT partition (WaveformStore, D-20 Option A)` · `v16.6.23-fifo`

| | |
|---|---|
| Purpose | Persist the captured waveform using the existing, already-provisioned 9MB FAT partition (`app3M_fat9M_16MB`, CN-11) — zero incremental cost, no 4G airtime, survives reboot. SDS's recommended option (D-20) over MQTT chunking or in-device discard. **[v1.1, Freeze Review Finding 2.1]** This task does **not** perform its own `TryAcquireResult`/`ReleaseResult` — it adds a second internal step to Task 5.2's existing `handleFifoCaptureCompletion()`, after the `/event` publish step, inside the *same* acquire/release pair. |
| Files affected | `fifo_waveform_store.h`, `fifo_waveform_store.cpp` (new); the existing `.ino`'s `handleFifoCaptureCompletion()` (Task 5.2) gains one call, `WaveformStore_Save(&result)`, between the existing publish step and `ReleaseResult()`. |
| Public interfaces | ```c\nbool WaveformStore_Save(const FifoCaptureResult* result);  // writes x/y/z + metadata to FAT, filename keyed by captureId\n``` |
| Internal interfaces | Internal file-naming/rotation logic (~1500 captures fit in 9MB at 6KB/capture, per SDS §18.4 — rotation/eviction policy is a coding-time decision this task must make explicit, not leave implicit). |
| Risks | This entire phase is explicitly a **product decision**, not purely technical (SDS §18.4 presents it as a recommendation, D-20 defers the final call). **This task should not start without an explicit go-ahead**, the same treatment the SDS gave D-15 before implementation. Flagged here so it isn't started by default momentum from Phase 6. **[v1.1]** Also: must not introduce a second `TryAcquireResult` call anywhere — reviewed specifically against that constraint, since it is the exact mistake Finding 2.1 identified in this task's original description. |
| Verification method | On-target: triggered capture followed by confirmed file presence/content on FAT, readable back and matching the arena's samples exactly; storage exhaustion behavior (>1500 captures) explicitly tested, not assumed. |
| Rollback strategy | Delete the two new files and the one added call inside `handleFifoCaptureCompletion()`; Task 5.2's `/event` publish is untouched, zero effect on Phases 1–6. |
| Dependencies | Tasks 3.5, 5.2 (`handleFifoCaptureCompletion()` must exist as the coordinator before this task adds a step to it). |

---

### Phase 8 — Field Validation and Sign-off

Not code commits — milestone/verification events, tagged but not "reviewed diffs" in the usual sense.

#### Task 8.1 — Soak validation (closes G-1 for real, on the new driver)
**Milestone** · tag `v16.6.0-fifo-soak`

| | |
|---|---|
| Purpose | Characterize the **new** driver's actual success rate, per SR, on real hardware — this is genuinely new verification of new code, not a re-opening of the closed WTVB05 investigation. The old 63%/17% baseline (K-15) describes the discarded implementation (PRR C-1 through C-11), most of whose failure modes (unbounded resync, malloc, busy-waits, collapsed error taxonomy) this driver eliminates by design — so the new number may differ substantially and must be measured, not assumed. |
| Files affected | None (validation only), possibly a soak-capture script following existing conventions. |
| Public interfaces | None. |
| Internal interfaces | None. |
| Risks | If the new rate is still poor at a given SR, `FIFO_PROTOCOL_MODEL` (D-14) is the one constant to flip and re-test — contained by design, not a redesign. |
| Verification method | ≥100 captures per SR of interest (SDS §23 target), full `/event` diagnostics collected, success rate + failure-class breakdown computed and compared to K-15 as a reference point (not a pass/fail bar in itself). |
| Rollback strategy | N/A — measurement only. |
| Dependencies | Phases 1–6 complete on hardware. |

#### Task 8.2 — Sign-off
**Milestone** · tag `v16.6.0-fifo`

| | |
|---|---|
| Purpose | Final gate before this becomes part of the production baseline: confirm every item in §8's master checklist, confirm Definition of Done for all prior phases, confirm G-2's status is documented (not resolved, tracked) in the release notes. |
| Verification method | §8 checklist, fully checked. |
| Dependencies | Task 8.1, plus Phase 7 if it was undertaken. |

---

## 6. Master Verification Checklist

Applies across the whole plan, checked at Phase 8 sign-off; individual items are also checked at their originating phase's DoD (§7).

- [ ] Zero `delay()`/`vTaskDelay()`/blocking-wait calls anywhere in `FifoDriver_Service()` or anything it calls (PRR C-1)
- [ ] Zero heap allocation anywhere in the driver (`malloc`/`new`/`String`) (PRR C-5)
- [ ] Samples reachable only through a `FifoCaptureResult` with `error == FifoError::NONE` (PRR C-2)
- [ ] `taskModbusRead()` remains the sole caller of `modbus.*`; no new mutex introduced (SDS D-15)
- [ ] `g_modbusConsecErrors` unaffected by a driver-owned bus suspension (CN-10)
- [ ] Every timeout is a named constant, none overridden at a different call site (PRR C-9)
- [ ] Zero `Serial` I/O inside the receive path; logging matches §3.1's policy exactly
- [ ] All 14 error codes distinctly reachable and distinctly tested (PRR C-7)
- [ ] `MAX_DESYNC_BYTES` bound enforced and tested with a synthetic in-payload anchor (PRR C-6)
- [ ] Every existing MQTT field on `/sensor`, `/vibration`, `/event` unchanged in type/meaning (`CLAUDE.md` backward-compat rule)
- [ ] Single retry-with-bus-release attempt never exceeds `ABSENT_STOPPING_MS` (SDS §15.3)
- [ ] Circuit breaker trips and rejects further requests without touching the transport (SDS §17.6)
- [ ] All L2 and L3 logic covered by host-runnable fixture tests, including all 7 recorded failure logs (PRR C-14)
- [ ] `arduino-cli compile` flash/RAM figures recorded before and after each hardware-touching phase, deltas explained
- [ ] G-1 and G-2 status explicitly stated in the Phase 8 sign-off notes (open/closed, not silently assumed either way)

---

## 7. Definition of Done — Per Phase

| Phase | Definition of Done |
|---|---|
| **0** | One-line change landed, built, flashed, soaked ≥10 min with zero regression to existing telemetry. |
| **1** | `FifoTransport` interface + production + test-fixture implementations compile standalone (host) and within the sketch; `LogReplayTransport` reproduces recorded timing to within the fixture corpus's own resolution. |
| **2** | All of §2.1–2.3's public interfaces implemented; `test_fifo_codec.cpp` passes against all 7 recorded logs (both successes and failures) plus the synthetic desync-bound case; `arduino-cli compile` confirmed unaffected by `test/`'s presence. |
| **3** | Full state machine (S0–S14) implemented; public API (§7 of the SDS) complete; `test_fifo_driver.cpp` passes including retry/breaker/watchdog and all listed fault-injection cases; zero `Serial` I/O in `Service()`'s call graph (verified by code inspection, not just test pass). |
| **4** | On-target: capture triggers correctly, normal polling visibly suspends/resumes, `StateMachine` task cadence unaffected, `g_modbusConsecErrors` unaffected, no watchdog trip across ≥10 captures, Task 4.2's diff confirmed minimal (existing polling code moved, not rewritten). |
| **5** | `capture_active` and `/event` fields present and correct on real MQTT traffic; every pre-existing field verified unchanged; `/event` payload size measured within budget. |
| **6** | All four in-scope trigger sources fire correctly on-target; admission gates reject and admit correctly per SDS §19.2's table; `REMOTE_ON_DEMAND`'s absence explicitly documented, not silently missing. |
| **7** (if undertaken) | Explicit go-ahead obtained before starting; captures persist and read back correctly; storage-exhaustion behavior tested. |
| **8** | §6 master checklist fully checked; soak data collected and compared to K-15; G-1/G-2 status recorded in sign-off notes; tag `v16.6.0-fifo` cut. |

---

## 8. Notes for the Reviewer

- This plan introduces **zero net lines of behavioral change** to the existing, proven `taskModbusRead()` polling logic — Task 4.2 relocates it unchanged into an `else` branch. The diff for that commit should read as "add a branch," not "rewrite the loop."
- Every commit that touches the existing `.ino` is small enough to review in isolation; every commit that adds new files is either pure-logic-and-testable-off-target (Phases 1–3) or a thin, reviewable glue layer (Phases 4–6).
- G-1 is deliberately not "resolved by assumption" anywhere in this plan — `FIFO_PROTOCOL_MODEL` stays a single named constant through Phase 8, and Task 8.1 exists specifically to gather the evidence the old Validation Tool never captured for this driver.
- G-2 is deliberately not implemented anywhere in this plan. If a future change adds an inbound MQTT channel for other reasons, `REMOTE_ON_DEMAND` becomes a small addition to Task 6.1's pattern — not a redesign.
- **[v1.1]** `handleFifoCaptureCompletion()` (Task 5.2) is the one function every future consumer of a capture result must extend, never duplicate. Any task that needs the result after Task 5.2 lands (Task 7.1 today; anything added later) adds one internal step to it — reviewers should treat a second, independent `TryAcquireResult` call anywhere in the codebase as a defect, not a style choice.
- **[v1.1]** Two Should-Improve items from the Freeze Review (`outHandle`'s undefined consumer-facing use; `FifoDriverConfig`'s copy-vs-retain lifetime at `Init()`) were intentionally **not** addressed in this revision — they do not block freezing and are left as coding-time decisions, per the Freeze Review's own classification. Not an oversight; recorded here so they aren't mistaken for one.

*End of CM-100_FIFO_Implementation_Plan_v1.0 — planning document only. No production code was written in producing this document.*
