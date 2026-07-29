> **Document Status**
>
> **Type:** Architecture Freeze Report Delta — companion to `CM-100_FIFO_Architecture_Freeze_Report_v1.3.md`. Extends the frozen baseline to v1.4 for the single, narrow scope introduced by `CM-100_ADR-0004_Session_Controller_Promotion_v1.0.md`. Same pattern as the v1.0→v1.1→v1.2→v1.3 progression already recorded inside the SDS's own Revision Notes — this delta is the freeze-record equivalent for that one step.
> **Architecture version frozen:** v1.4 (extends v1.3; no decision D-1..D-20 is reopened — see §3)
> **Freeze date:** 2026-07-29
> **Freeze basis:** `CM-100_ADR-0004_Session_Controller_Promotion_v1.0.md` (Accepted 2026-07-29) + `CM-100_FIFO_Implementation_Plan_v1.4_Delta.md` + `CM-100_FIFO_DRIVER_SDS_v1.4_Delta.md`, following this session's Task 3.1A Integration Review (which surfaced the Task 3.1 mismatch this ADR resolves) and this document's own confirmation pass (§4).

# CM-100 FIFO Waveform Driver — Architecture Freeze Report v1.4 Delta

## 1. Declaration

**The design baseline for the CM-100 FIFO Waveform Driver is extended to v1.4**, covering exactly one addition: Session Controller (`fifo_session.h`/`fifo_session.cpp`) is a frozen, standalone L3 component, occupying Task 3.1 of Phase 3, with the public surface and ownership boundaries specified in ADR-0004 and the SDS v1.4 Delta. **No decision frozen at v1.3 is reopened, altered, or contradicted by this extension** — see §3.

As with v1.2/v1.3 before it, this is a documentation-only freeze step: **no firmware source is modified by this delta.** `fifo_session.{h,cpp}` already exist in the working tree, uncommitted, matching exactly what is now frozen.

## 2. Documents Included in This Freeze

| Document | Version | Role |
|---|---|---|
| `CM-100_ADR-0004_Session_Controller_Promotion_v1.0.md` | 1.0, **Accepted** | Architecture Decision Record — promotes Session Controller to standalone L3 |
| `CM-100_FIFO_Implementation_Plan_v1.4_Delta.md` | 1.4 delta | Phase 3 task renumbering (3.1 Session Controller → 3.2 shared types → ... → 3.7 test suite) |
| `CM-100_FIFO_DRIVER_SDS_v1.4_Delta.md` | 1.4 delta | §5.2, §7.1, §11.1, §11.4, §20.1, §24 section updates |
| `CM-100_FIFO_DRIVER_SDS_v1.0.md` | 1.3 (base, unchanged by this delta — see the delta document for how v1.4 content layers on top) | Design authority, prior baseline |
| `CM-100_FIFO_Implementation_Plan_v1.0.md` | 1.3 (base, unchanged — see the delta document) | Task breakdown, prior baseline |
| `CM-100_ADR-1_Bus_Ownership_Includes_EN_Ownership_v1.0.md` | 1.0, Accepted | Unaffected by this delta — carried forward unchanged |
| `CM-100_FIFO_Architecture_Freeze_Report_v1.3.md` | 1.3 | Prior freeze record this delta extends |

Filenames follow the same convention already established: delta/companion documents get their own filename (matching `CM-100_FIFO_Architecture_Change_Report_v1.0.md`'s precedent); the SDS and Implementation Plan keep their `_v1.0`-suffixed filenames regardless of internal version.

## 3. Confirmation: D-1..D-20 and ADR-1 Are Unaffected

Re-checked explicitly, since this freeze step was produced in direct response to a review finding rather than through the normal task-execution sequence — same discipline `CM-100_FIFO_Architecture_Change_Report_v1.0.md` §5 applied to the EN-ownership change:

| Decision | Result |
|---|---|
| D-1 (tick-sliced, `taskModbusRead`-serviced state machine) | Unaffected — Session Controller adds no new task, no new blocking call, no change to the 250 ms tick budget |
| D-6 (no externally reachable driver state) | Unaffected — `FifoSessionState` is caller-owned, reachable only through `FifoDriver`'s composition of it, never a global |
| D-7 (injected transport) | Unaffected — Session Controller receives `FifoTransport*`, never references `SerialRS485`/`Serial`/`modbus` |
| D-14 (protocol-model single named constant) | Unaffected — remains entirely `FifoDriver`'s concern (SDS v1.4 Delta §4) |
| D-15 / ADR-1 (bus/EN ownership) | Unaffected — Session Controller has zero bus/EN awareness by design; ADR-1's decision and its text are untouched |
| D-16 (single-writer thread model) | Unaffected — Session Controller is called only from `FifoDriver`, itself called only from `taskModbusRead()`; no new writer, no new cross-core path |
| D-18 (14-code error taxonomy) | Not contradicted — extended with an explicit translation obligation (`FifoSessionOutcome`/`FifoSessionFailReason` → `FifoError`), formally assigned to Task 3.4, not left implicit (SDS v1.4 Delta §9) |
| §11.4 (frame-parsing state explicit, not hidden) | Reinforced, not weakened — ownership is now *more* specific (a named field of `FifoSessionState`) than the v1.3 text ("owned by the session (L3)") required |
| §7 (9-entry public API surface) | Unaffected — no entry added, removed, or changed; Session Controller's functions are explicitly documented as internal to the driver, not a 10th entry point |

**No decision required re-opening.** This is, in the same shape as the v1.2 Architecture Update and the v1.3 Architecture Review Feedback before it, a single isolated correction — here, to which document governs Task 3.1's identity — found and applied outside the normal review sequence, then confirmed not to disturb anything else.

## 4. Outstanding Implementation Work — Updated

Supersedes the equivalent table in `CM-100_FIFO_Architecture_Freeze_Report_v1.3.md` §5 for the rows shown; all other rows carry forward unchanged.

| Item | Status | Notes |
|---|---|---|
| Task 0.1 — RX buffer sizing | Complete, uncommitted | Unchanged from v1.3 |
| Task 1.1 — `FifoTransport` interface | Complete, uncommitted | Unchanged from v1.3 |
| Task 1.2 — `Uart485Transport` implementation | Revised to match v1.3 EN model; still uncommitted | Unchanged from v1.3 |
| Task 1.3 — `LogReplayTransport` test fixture | **Implemented, uncommitted** | Status updated this session — `test/log_replay_transport.{h,cpp}` exist in the working tree. (Freeze v1.3 recorded this as "Not started"; superseded by direct observation.) |
| Task 2.1–2.3 — L2 Frame Codec (`StreamingCrc16`, `SampleDecoder`, `FrameCodec`) | **Complete, committed** (frozen, per `fifo_codec.h`/`fifo_codec.cpp`'s own headers and this session's git log) | Status updated this session |
| **Task 3.1 — Session Controller** `[NEW, v1.4]` | **Implemented; cleared to commit.** Reviewed this session (Task 3.1A Integration Review): header dependencies minimal, no circular include, deterministic state transitions, explicit memory ownership, no defect found in the code itself. **Not yet integrated into `taskModbusRead()`** — that function does not exist in the `.ino` yet; Phase 4 has not started | Commit under the Task 3.1 label and `v16.6.8-fifo` tag — cleared now that this freeze is accepted |
| Task 3.2 (renumbered from 3.1) — Shared types | Not started | Unchanged in content from v1.3's Task 3.1, renumbered only |
| Task 3.3 (renumbered from 3.2) — Sample arena | Not started | Renumbered only |
| Task 3.4 (renumbered from 3.3) — Core protocol state machine | Not started | Scope grows: composes Session Controller for `S4`/`S5`/`S6`/`S8`; gains K-10 validation (SDS v1.4 Delta §5, §6) and `FifoError` translation obligations (SDS v1.4 Delta §9; ADR-0004 §9 Risk 2) |
| Task 3.5 (renumbered from 3.4) — Recovery/lifecycle | Not started | Renumbered only |
| Task 3.6 (renumbered from 3.5) — Public API | Not started | Renumbered only; surface itself unchanged |
| Task 3.7 (renumbered from 3.6) — Test suite | Not started | Scope grows: adds `test/test_fifo_session.cpp` (Implementation Plan v1.4 Delta §6) |
| Task 4.1 — Wire `FifoDriver_Init()` into `setup()` | Not started | Unaffected |
| Task 4.2 — Mode branch in `taskModbusRead()` (bus + EN ownership) | Not started | Unaffected — still the highest-risk task, per v1.3 |
| Task 4.3 — `g_modbusConsecErrors` non-increment guard | Not started | Unaffected |
| Task 4.4 — On-target integration verification | Not started | Unaffected |
| Phases 5–8 | Not started | Unaffected |
| G-1 / G-2 | Open, unchanged | Unaffected by this delta |

## 5. Baseline Statement

> **The CM-100 FIFO Waveform Driver design baseline is extended to v1.4.** All decisions frozen at v1.3 (D-1..D-20, ADR-1) remain settled and unaltered. Session Controller (`fifo_session.h`/`fifo_session.cpp`) is now the frozen, formally specified Task 3.1 of Phase 3 — a standalone L3 sub-component composed by `FifoDriver`, not an inline responsibility of it. Any future change to Session Controller's public API, ownership boundary, or outcome-translation contract requires the same discipline this delta itself followed: a new ADR or a new revision cycle, never a silent edit inside a later implementation task.

---

*End of CM-100_FIFO_Architecture_Freeze_Report v1.4 Delta — freeze record only; no new design content beyond what ADR-0004 and its companion deltas already specify. No firmware source modified. No build run. Nothing committed.*
