> **Document Status**
>
> **Type:** Implementation Plan Delta — companion to `CM-100_FIFO_Implementation_Plan_v1.0.md` (internal version 1.3, frozen). Records the Phase 3 task renumbering and new Task 3.1 specification introduced by `CM-100_ADR-0004_Session_Controller_Promotion_v1.0.md`. Same pattern as `CM-100_FIFO_Architecture_Change_Report_v1.0.md` — a delta document, not a rewrite of the base plan.
> **Base document, version amended:** `CM-100_FIFO_Implementation_Plan_v1.0.md`, internal version 1.3 → 1.4.
> **Trigger:** ADR-0004. See that document for motivation.
> **Scope:** Phase 3 task numbering and Task 3.1's content only. Phases 0, 1, 2, and 4–8 are unaffected — see §4.
> **Status:** documentation only. **No firmware source changed.** `fifo_session.h`/`fifo_session.cpp` already exist in the working tree, uncommitted, and are unaffected by this delta.

# CM-100 FIFO Implementation Plan — v1.4 Delta: Session Controller Promotion

---

## 1. Decision Summary

| | Before (v1.3) | After (v1.4) |
|---|---|---|
| **Task 3.1** | Shared types (`fifo_types.h`) | **Session Controller** (`fifo_session.h`, `fifo_session.cpp`) |
| **Shared types task** | 3.1 | 3.2 |
| **Sample arena task** | 3.2 | 3.3 |
| **Core protocol state machine task** | 3.3 | 3.4 (scope changes — see §3) |
| **Recovery/lifecycle task** | 3.4 | 3.5 |
| **Public API task** | 3.5 | 3.6 |
| **Test suite task** | 3.6 | 3.7 (scope grows — see §3) |
| **Phase 3 task count** | 6 | 7 |
| **Version tags** | `v16.6.9-fifo` .. `v16.6.14-fifo` (Tasks 3.1–3.6) | `v16.6.8-fifo` (Task 3.1, already used, unchanged) + `v16.6.9-fifo` .. `v16.6.14-fifo` (Tasks 3.2–3.7, unchanged) — no renumbering of tags, only of task labels |
| **Commit numbers** | Commits 9–14 (Tasks 3.1–3.6) | Commit 9 (Task 3.1, new) + Commits 10–15 (Tasks 3.2–3.7) — renumbered by one, unlike tags, to make room for the inserted Task 3.1 |

---

## 2. Phase 3 Renumbering Table

| Old task | New task | Old Commit # | New Commit # | Commit content | Files | Tag |
|---|---|---|---|---|---|---|
| — (new) | **3.1** | — | **9** | `feat(fifo): promote Session Controller to standalone L3 component (ADR-0004)` | `fifo_session.h`, `fifo_session.cpp` | `v16.6.8-fifo` *(already present in working tree, uncommitted)* |
| 3.1 | 3.2 | 9 | 10 | `feat(fifo): add shared types (FifoPhase, FifoError, request/result structs)` | `fifo_types.h` (new) | `v16.6.9-fifo` |
| 3.2 | 3.3 | 10 | 11 | `feat(fifo): add FifoArena — single static buffer, exclusive ownership handoff` | `fifo_arena.h`, `fifo_arena.cpp` (new) | `v16.6.10-fifo` |
| 3.3 | 3.4 | 11 | 12 | `feat(fifo): implement FifoDriver core state machine (request through verify)` | `fifo_driver.h`, `fifo_driver.cpp` | `v16.6.11-fifo` |
| 3.4 | 3.5 | 12 | 13 | `feat(fifo): implement recovery states (drain, cooldown, retry, circuit breaker)` | `fifo_driver.cpp` | `v16.6.12-fifo` |
| 3.5 | 3.6 | 13 | 14 | `feat(fifo): expose FifoDriver public API (Init/Request/Service/...)` | `fifo_driver.h` (public section) | `v16.6.13-fifo` |
| 3.6 | 3.7 | 14 | 15 | `test(fifo): FifoDriver integration tests (state machine, retry, breaker, fault injection)` | `test/test_fifo_driver.cpp` (new) | `v16.6.14-fifo` |

No tag is reused, skipped, or reassigned to a different task's content — the insertion lands in the one slot (`v16.6.8-fifo`) the frozen v1.3 plan never allocated. See ADR-0004 §1 for why that slot was already open. **Unlike tags, commit numbers do shift by one**, shown in the Old/New Commit # columns above: Task 3.1 (Session Controller, new) claims Commit 9; shared types (Task 3.2) moves from its v1.3 label of Commit 9 to Commit 10, and every task after it shifts accordingly. This resolves the duplicate "Commit 9" label that otherwise exists between this table and ADR-0004 §1's citation of the v1.3 plan's original numbering.

**Task 3.2's Dependencies field** (`fifo_types.h`, "Dependencies | None.") is unchanged by the renumbering — it still depends on nothing preceding it in Phase 3. **Task 3.3's Dependencies field** changes from "Task 3.1" to "Task 3.2" (same relationship, renumbered label only).

---

## 3. New Task 3.1 — Session Controller

**Commit 9** · `feat(fifo): promote Session Controller to standalone L3 component (ADR-0004)` · `v16.6.8-fifo`

| | |
|---|---|
| Purpose | Own the lifetime of exactly one `FrameCodecState` and drive it through repeated `FrameCodec_Step()` calls, translating each L2 `FifoFrameOutcome` into an L3-level `FifoSessionOutcome` and `FifoSessionPhase`. The one component between L2 (`FrameCodec`, frozen Task 2.3) and the rest of L3 (`FifoDriver`) — see ADR-0004 §3. |
| Files affected | `fifo_session.h`, `fifo_session.cpp` (already written, uncommitted — adopted as-is, not modified by this task) |
| Public interfaces | ```c\nenum class FifoSessionPhase { IDLE, WAIT_FRAME, RECEIVING, COMPLETE, FAILED };\nenum class FifoSessionFailReason { NONE, BAD_FRAME_TYPE, DESYNC };\nenum class FifoSessionOutcome { NONE, PROGRESS_UPDATE, DUMP_COMPLETE, FRAME_REJECTED, SESSION_FAILED };\n\nstruct FifoSessionState {\n  FrameCodecState frameState;\n  FifoSessionPhase phase;\n  FifoSessionFailReason failReason;\n  uint16_t lastProgressFill;\n};\n\nvoid FifoSession_Init(FifoSessionState* s);\nvoid FifoSession_StartAttempt(FifoSessionState* s);\nFifoSessionOutcome FifoSession_Step(FifoSessionState* s, FifoTransport* t,\n                                     int16_t* xOut, int16_t* yOut, int16_t* zOut);\n``` Module-public (extern linkage), consumed exclusively by `fifo_driver.cpp` (Task 3.4) — never called from the `.ino` directly, same distinction §7.3 of the SDS already draws for `FrameCodec_Step()`. |
| Internal interfaces | None — the `.cpp` implements the public surface directly, with no static helper functions (unlike `fifo_codec.cpp`'s `ScanAnchor()`/`ReadType()`/`ReadProgress()`/`ReadDump()` split, which stays entirely inside L2, untouched). |
| Explicitly out of scope | Timeout detection (owned by `FifoDriver`/L3 lifecycle, Design Closure §2 — this file never calls `nowMs()`). Request framing/transmission (this file only ever reads via `FifoTransport`, never calls `t->write()`). Sample-arena ownership, admission gates, retry/backoff policy, the public multi-entry-point driver API, K-10 `fill`-regression/overrun validation (see Risks). |
| Risks | K-10 validation and the `FifoSessionOutcome → FifoError` translation are **not** implemented here and must not be assumed present by Task 3.4 — see ADR-0004 §9, Risks 1–2. Depends on Task 2.3's `FrameCodec_Step()`/`FrameCodecState_Reset()` contract remaining frozen; re-verified compatible by ADR-0004 §8 and by this session's Task 3.1A Integration Review. |
| Verification method | Host `g++` compile standalone (zero Arduino dependency, matching L1/L2). Fixture test (new scope, added to Task 3.7 below): drive `FifoSession_Step()` against a `LogReplayTransport` fixture and assert the outcome table in `fifo_session.h`'s doc comment holds for each of the 6 `FifoFrameOutcome` values `FrameCodec_Step()` can actually produce. |
| Rollback strategy | Delete both files; nothing outside Phase 3 references this yet (confirmed — the `.ino` has zero references to any FIFO file as of this session's review). |
| Dependencies | Task 2.3 (frozen `FrameCodec`). None within Phase 3 — this is now the first Phase 3 task. |

---

## 4. Confirmation: No Other Phase Is Affected

| Phase | Re-checked | Result |
|---|---|---|
| Phase 0 (RX buffer sizing) | No relationship to L3 session behavior | Unaffected |
| Phase 1 (`FifoTransport`, `Uart485Transport`, `LogReplayTransport`) | Session Controller consumes `FifoTransport` exactly as documented in `fifo_transport.h`; no change to that interface | Unaffected |
| Phase 2 (`StreamingCrc16`, `SampleDecoder`, `FrameCodec`) | Frozen, Task 2.3. See ADR-0004 §8 — Session Controller's only two call sites use the existing frozen contract exactly | Unaffected |
| Phase 3, Tasks 3.2–3.7 (renumbered) | Task numbers shift; task *content* is otherwise unchanged except Task 3.4 (§5 below) and Task 3.7 (§6 below) | Renumbered, two scope additions |
| Phases 4–8 (integration, telemetry, trigger wiring, waveform egress, soak) | No dependency on Phase 3's internal task numbering, only on Phase 3's final public API (`FifoDriver`'s 9 entry points), which is unchanged | Unaffected |

---

## 5. Task 3.4 (renumbered from 3.3) — Scope Addition

**Purpose — addition:** *"Composes Session Controller (Task 3.1) for states `S4`/`S5`/`S6`/`S8` instead of driving `FrameCodec_Step()` and threading `FrameCodecState` directly. This task's per-state handlers for those four states collapse into: call `FifoSession_Step()` once per tick, branch on the returned `FifoSessionOutcome`."*

**Internal interfaces — addition:** *"Gains two new obligations not present in v1.3: (1) K-10 `fill`-regression/overrun validation, comparing `FifoSessionState.lastProgressFill` across successive `PROGRESS_UPDATE` outcomes within one capture attempt (ADR-0004 §9, Risk 1); (2) translating `FifoSessionOutcome`/`FifoSessionFailReason` into the frozen `FifoError` taxonomy (D-18) before populating `FifoCaptureResult.error` — no `FifoSessionFailReason` value may be dropped or collapsed into a code that loses information the taxonomy was designed to preserve (ADR-0004 §9, Risk 2)."*

**Dependencies — addition:** Task 3.1 (Session Controller), in addition to the existing Tasks 2.3, 3.2 (renumbered from 3.1), 3.3 (renumbered from 3.2).

No other field of this task changes. States `S0`–`S3`, `S7`, `S9`–`S14` and their bus/EN/admission/timeout responsibilities are unaffected — Session Controller has no relationship to any of them.

---

## 6. Task 3.7 (renumbered from 3.6) — Scope Addition

**Purpose — addition:** *"Also closes the loop on Task 3.1: a Session-Controller-only fixture test, independent of the full `FifoDriver` state machine, exercising `fifo_session.h`'s documented outcome table (`FifoFrameOutcome → phase / reset? / FifoSessionOutcome`) directly against `LogReplayTransport`."*

**Files affected — addition:** `test/test_fifo_session.cpp` (new).

No other field of this task changes.

---

## 7. Migration Required

1. **No firmware source file requires revision.** `fifo_session.h`/`fifo_session.cpp` are adopted as-written under the new Task 3.1 label.
2. **No commits exist yet for any Phase 3 task** (confirmed via git status this session — `fifo_session.{h,cpp}` are untracked). There is nothing to revert; the renumbering applies cleanly to work not yet begun for Tasks 3.2–3.7.
3. **Recommendation:** commit Task 3.1 (`fifo_session.{h,cpp}`, tag `v16.6.8-fifo`) only after ADR-0004 and all three companion deltas are accepted, per `CLAUDE.md`'s approval-before-commit workflow rule.
4. Any existing reference (commit message drafts, personal notes, tracking issues) to "Task 3.1 = shared types" must be updated to "Task 3.2 = shared types" before Phase 3 work resumes.

---

*End of CM-100_FIFO_Implementation_Plan v1.4 Delta. No firmware source modified. No build run. Nothing committed.*
