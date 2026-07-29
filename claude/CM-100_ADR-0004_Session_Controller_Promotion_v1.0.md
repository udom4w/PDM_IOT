> **Document Status**
>
> **Type:** Architecture Decision Record (ADR) — promotes a component from an internal, undocumented implementation detail to a formally specified, standalone Layer 3 module. Not itself a full design document; the mechanical consequences are carried into the three companion delta documents listed in §References.
> **ID:** ADR-0004
> **Title:** Promote Session Controller to a standalone Layer 3 component
> **Status:** Accepted (2026-07-29) — confirmed via the same lightweight confirmation pattern already used for `CM-100_FIFO_DRIVER_SDS` v1.2/v1.3 (single documentation pass, narrow scope, code already exists and was independently reviewed). Not build-verified — acceptance is an architecture/documentation decision, independent of build status. See `CM-100_FIFO_Architecture_Freeze_Report_v1.4_Delta.md` for the resulting freeze status.
> **Date:** 2026-07-29
> **Supersedes:** nothing. **Reassigns:** `CM-100_FIFO_Implementation_Plan_v1.0` (frozen v1.3) Task 3.1 — see §3.
> **Raised by:** Task 3.1A Integration Review (this session, prior turn) of `fifo_session.h`/`fifo_session.cpp`, which found the files' own header comment claiming "Implementation Plan Task 3.1" does not match the frozen plan's Task 3.1 (`fifo_types.h`, shared types) — and that the concept the files actually implement already exists in the frozen SDS §20.1 class diagram, but as a private, internal field of `FifoDriver`, not a standalone public module.
> **Affects:** `CM-100_FIFO_Implementation_Plan_v1.0.md` (Phase 3 task numbering), `CM-100_FIFO_DRIVER_SDS_v1.0.md` §5.2, §7, §11, §11.4, §20.1, §24, `CM-100_FIFO_Architecture_Freeze_Report_v1.3.md` §2, §5.
> **No firmware changes result from this ADR.** `fifo_session.h`/`fifo_session.cpp` already exist in the working tree, uncommitted, and are not modified by this ADR. This document decides what they are, not what they contain.

# ADR-0004 — Promote Session Controller to a standalone Layer 3 component

## 1. Motivation

The prior turn's Task 3.1A Integration Review checked `fifo_session.h`/`fifo_session.cpp` against the frozen `CM-100_FIFO_Implementation_Plan` v1.3 and `CM-100_FIFO_DRIVER_SDS` v1.3, and found a genuine mismatch rather than a code defect:

- The frozen Implementation Plan's Task 3.1 is `fifo_types.h` — shared enums/structs (`FifoPhase`, the 14-code `FifoError` taxonomy, `FifoTriggerSource`, `FifoAdmissionContext`, `FifoCaptureRequest`, `FifoCaptureResult`, `FifoDriverStats`). `fifo_session.{h,cpp}` implement none of that.
- The frozen SDS §20.1 class diagram already names a `session : FifoSession (timers, retry budget, provenance, lastProgressFill, FrameCodecState)` field — but as a **private member of `FifoDriver`**, not a standalone module with its own header and public API.
- `fifo_session.{h,cpp}` were written as a free-standing, testable L3 unit — zero Arduino dependency, matching L1/L2's own standard — driving `FrameCodec_Step()` in a loop and translating its terminal outcomes into a coarser session-level phase. This is real, useful, well-documented work. It just isn't the work the frozen plan assigned to "Task 3.1," and it isn't shaped the way the frozen SDS's `FifoSession` field was shaped either (no timers, no retry budget, no provenance).

Per the Freeze Report's own closing statement, a change to public API, ownership rule, or state-machine shape *"may not be made silently inside an implementation task."* This ADR is that formal step: it does not change what the code does, it decides what the code **is**, on the record, so the documentation — not the header comment inside one file — becomes the source of truth for Phase 3's shape going forward.

**Supporting evidence that this promotion is the natural reading, not a stretch:** `fifo_session.h`/`fifo_session.cpp` already carry the version tag `[v16.6.8-fifo]` — the tag immediately following Task 2.3's `[v16.6.7-fifo]` (`FrameCodec` Phase 8) and immediately preceding the frozen plan's already-assigned `v16.6.9-fifo` (old Task 3.1, shared types, historically Commit 9 under v1.3 — renumbered Commit 10 as Task 3.2 under the Implementation Plan v1.4 Delta §1/§2, since Task 3.1 itself now claims Commit 9). Nothing in the frozen plan occupied `v16.6.8-fifo`. Whoever wrote this file already slotted it into the one open position directly after L2 and directly before the old Task 3.1 — which is exactly where this ADR formally places it.

## 2. Previous architecture

Per `CM-100_FIFO_DRIVER_SDS_v1.0` (frozen v1.3) §20.1, `FifoDriver` (L3) was to directly compose:

```
FifoDriver
 ├─ transport : FifoTransport
 ├─ state     : FifoState (S0..S14)
 ├─ session   : FifoSession   (timers, retry budget, provenance, lastProgressFill, FrameCodecState)
 ├─ arena     : FifoArena
 ├─ result    : FifoCaptureResult
 ├─ stats     : FifoDriverStats
 └─ breaker   : FifoBreaker
```

`FifoSession` here was an internal data bag, not a component with behavior of its own. Per Implementation Plan Task 3.3 (core protocol state machine), the *behavior* of driving `FrameCodec_Step()` across states `S4 AWAIT_ANCHOR`/`S5 READ_TYPE`/`S6 READ_PROGRESS`/`S8 READ_DUMP` lived directly inside `fifo_driver.cpp`'s per-state handlers (`S4_AwaitAnchor`, ... `S9_Verify`), each one threading `FrameCodecState` by pointer into `FrameCodec_Step()` itself. There was no separate translation step, no separate outcome enum, and no standalone testable unit smaller than the whole `FifoDriver` state machine for this behavior.

## 3. New architecture

**Session Controller is promoted to a standalone, public L3 module** — `fifo_session.h`/`fifo_session.cpp`, exactly as already written — sitting between L2 (`FrameCodec`) and the rest of L3 (`FifoDriver`, `FifoArena`):

```
┌──────────────────────────────────────────────────────────────┐
│  L4  Diagnostic Workflow    (§19)                            │
├──────────────────────────────────────────────────────────────┤
│  L3  Capture Session         (§11, §12, §17)                 │
│      FifoDriver: retry budget · cooldown · circuit breaker · │
│      admission · arena · bus/EN ownership · result lifecycle │
│        composes ▼                                             │
│      SessionController (fifo_session.h/.cpp, NEW standalone) │
│      one FrameCodecState per attempt · outcome translation ·  │
│      WAIT_FRAME/RECEIVING/COMPLETE/FAILED phase classification│
├──────────────────────────────────────────────────────────────┤
│  L2  Frame Codec             (§8, §16) — Task 2.3, frozen,   │
│      UNCHANGED by this ADR                                    │
├──────────────────────────────────────────────────────────────┤
│  L1  Transport Abstraction   (§6)                             │
└──────────────────────────────────────────────────────────────┘
```

This is deliberately a **narrow promotion**, not a redesign of L3. Session Controller owns exactly what its header already documents as in-scope: driving one `FrameCodecState` through repeated `FrameCodec_Step()` calls and translating each terminal `FifoFrameOutcome` into an `FifoSessionOutcome` plus a coarser `FifoSessionPhase`. Everything the old inline `FifoSession` field carried that Session Controller does *not* implement — timers, retry budget, provenance, admission, arena, bus/EN — stays exactly where the frozen SDS already put it: inside `FifoDriver` itself (Task 3.4, renumbered from 3.3). See `CM-100_FIFO_DRIVER_SDS_v1.4_Delta.md` for the full section-by-section diagram and prose update.

## 4. Responsibility changes

| Responsibility | Before (v1.3) | After (v1.4) |
|---|---|---|
| Drive `FrameCodec_Step()` across `S4`/`S5`/`S6`/`S8` | Inline, inside `fifo_driver.cpp`'s per-state handlers | Session Controller's `FifoSession_Step()` — one call per `FifoDriver_Service()` tick while an attempt is active |
| Classify "waiting for anchor" vs. "mid-receive" | Not separately named; implicit in which `FifoState` value the driver was in | `FifoSessionPhase::WAIT_FRAME` / `RECEIVING`, derived from `FrameCodecState.anchorFound` — a Session Controller responsibility |
| Reset `FrameCodecState` at each attempt boundary | `fifo_driver.cpp`, ad hoc per state | `SessionController`, uniformly, on every terminal outcome (`FifoSession_Step()`'s documented outcome table) — never left to the caller to remember |
| Decide retry vs. give up on `CRC_MISMATCH` / `BAD_TYPE_BYTE` / `DESYNC_LIMIT` | `fifo_driver.cpp`, Task 3.3/3.4 | **Unchanged, stays with `FifoDriver` (Task 3.4/3.5).** Session Controller only auto-loops `PROGRESS`/`CRC_MISMATCH` back to `WAIT_FRAME` (a same-attempt re-arm, not a retry-budget decision) and stops at `FAILED` for `BAD_TYPE_BYTE`/`DESYNC_LIMIT`, leaving the retry-attempt policy (bus drain, release, fresh `StartAttempt()`) entirely to the caller, exactly as `fifo_session.h` already documents |
| K-10 `fill` regression/overrun validation (Freeze Review Finding 7.1, assigned to "`S6`'s handler") | Unassigned at the code level (plan only said "`S6`'s handler," which no longer exists as a literal state after this promotion) | **Not implemented by Session Controller today** — `FifoSession_Step()` records `lastProgressFill` but performs no comparison against the previous value. Formally reassigned as a Task 3.4 (renumbered) responsibility — see §9 Future tasks affected |
| Timeout detection | `FifoDriver`, Design Closure §2 | **Unchanged.** Session Controller is explicitly, permanently out of this business — it never calls `nowMs()` |

## 5. Ownership changes

| State | Before (v1.3, SDS §20.1) | After (v1.4) |
|---|---|---|
| `FrameCodecState` | "Owned and allocated by the session (L3)" — generic, ambiguous between "a bare struct member of `FifoDriver`" and "some sub-object" | Owned by `FifoSessionState`, a field of `SessionController`'s own state struct — never touched by `FifoDriver` directly except by reading `s->phase`/`s->failReason`/`s->lastProgressFill` and by calling the three `FifoSession_*` entry points |
| `lastProgressFill` | Field of the monolithic `FifoSession` bag | Field of `FifoSessionState`, populated by Session Controller on every `PROGRESS_UPDATE` outcome. **Still the correct anchor for `FifoDriver`'s future K-10 comparison** (§4) — the value exists, only the comparison logic does not yet |
| Timers, retry budget, provenance | Fields of the monolithic `FifoSession` bag | Remain `FifoDriver`'s own fields (Task 3.4/3.5, renumbered). Session Controller's state struct (`FifoSessionState`) has no timer, no retry counter, no provenance field, and none is added by this ADR |
| Sample destination (`xOut`/`yOut`/`zOut`) | Implicitly `FifoArena`'s write handles, threaded through `fifo_driver.cpp` | **Unchanged in effect.** `FifoDriver` still owns `FifoArena` and still supplies the write-handle pointers; Session Controller only forwards them through to `FrameCodec_Step()` unmodified, exactly as `fifo_session.h` already documents ("this file introduces no sample storage of its own") |

## 6. Public API changes

**`FifoDriver`'s 9-entry public surface (SDS §7.1, A-1..A-9) is unchanged by this ADR.** No new function is added to it, no signature changes. Nothing outside the FIFO driver — including the eventual `taskModbusRead()` integration — calls `FifoSession_*` directly; it remains reachable only from `fifo_driver.cpp`.

New, module-scoped public symbols introduced by this promotion (already present in the working tree, unmodified):

```c
enum class FifoSessionPhase { IDLE, WAIT_FRAME, RECEIVING, COMPLETE, FAILED };
enum class FifoSessionFailReason { NONE, BAD_FRAME_TYPE, DESYNC };
enum class FifoSessionOutcome { NONE, PROGRESS_UPDATE, DUMP_COMPLETE, FRAME_REJECTED, SESSION_FAILED };

struct FifoSessionState {
  FrameCodecState frameState;
  FifoSessionPhase phase;
  FifoSessionFailReason failReason;
  uint16_t lastProgressFill;
};

void              FifoSession_Init(FifoSessionState* s);
void              FifoSession_StartAttempt(FifoSessionState* s);
FifoSessionOutcome FifoSession_Step(FifoSessionState* s, FifoTransport* t,
                                     int16_t* xOut, int16_t* yOut, int16_t* zOut);
```

These are extern-linkage (any translation unit that includes `fifo_session.h` can call them), the same as every other L1/L2 entry point in this driver — but **"public" here means "public to the rest of the FIFO driver," not "public to the rest of the firmware."** This is the same distinction §7.3 already draws for `FrameCodec_Step()`/`FrameCodecState_Reset()`: extern linkage is a C++ mechanism, not a statement about who is expected to call it.

## 7. Migration impact

- **No firmware source file changes.** `fifo_session.h`/`fifo_session.cpp` are adopted as-written.
- **No version-tag renumbering.** `v16.6.8-fifo` was already the correct, unused slot (see §1's supporting evidence); every downstream tag (`v16.6.9-fifo` onward) keeps the value the frozen plan already assigned it — see the Implementation Plan delta for the full task-to-tag table.
- **Task numbering shifts by one** for every task from old-3.1 onward: shared types (old 3.1) → new 3.2, sample arena (old 3.2) → new 3.3, core protocol state machine (old 3.3) → new 3.4, recovery/lifecycle (old 3.4) → new 3.5, public API (old 3.5) → new 3.6, test suite (old 3.6) → new 3.7. Full detail in `CM-100_FIFO_Implementation_Plan_v1.4_Delta.md` §2.
- **Nothing to revert.** Per the prior git-status check, `fifo_session.h`/`fifo_session.cpp` are untracked — there is no commit history to rewrite, no prior release built against the old Task 3.1 numbering. This is the cheapest possible point to make this change, matching the same observation `CM-100_FIFO_Architecture_Change_Report_v1.0.md` made about Task 1.2 at the time.
- **Process note:** per `CLAUDE.md`'s workflow rules, `fifo_session.{h,cpp}` should not be committed until this ADR and its three companion deltas are accepted — the files currently exist ahead of the formal decision that now legitimizes them, which this ADR closes out, not retroactively excuses going forward.

## 8. Compatibility with Task 2.3

Full compatibility, by construction and by inspection. `fifo_session.cpp` calls exactly two L2 entry points, both used strictly per their frozen, documented contracts:

- `FrameCodec_Step(t, &s->frameState, &progressFill, xOut, yOut, zOut)` — called once per `FifoSession_Step()` invocation while an attempt is active, never more, matching `FrameCodec_Step()`'s own "advance by whatever bytes are available, bounded and non-blocking" contract.
- `FrameCodecState_Reset(&s->frameState)` — called exactly once per new attempt (inside `FifoSession_Init()` and at every terminal outcome inside `FifoSession_Step()`), matching `FrameCodecState_Reset()`'s "once per new frame attempt" precondition.

Session Controller reads exactly one additional `FrameCodecState` field directly — `state.anchorFound`, for `WAIT_FRAME`/`RECEIVING` classification — which `fifo_codec.h` already documents as a public, stable field for exactly this purpose (`FrameCodecState`'s own Design Closure §1 note). No other `FrameCodecState` field is read or written outside `FrameCodec_Step()`/`FrameCodecState_Reset()` themselves.

**No change to `fifo_codec.h` or `fifo_codec.cpp` is required or implied by this ADR.** Task 2.3 remains frozen and untouched.

## 9. Risks

| # | Risk | Disposition |
|---|---|---|
| 1 | K-10 `fill`-regression/overrun validation (Finding 7.1) has no owner in code today — `FifoSessionOutcome` has no `PROGRESS_REGRESSION`/`PROGRESS_OVERRUN` value, and `FifoSessionState.lastProgressFill` is recorded but never compared against | Explicitly assigned to Task 3.4 (renumbered core state machine): compare `lastProgressFill` after each `PROGRESS_UPDATE` outcome before treating a fresh capture as valid, reusing the field Session Controller already tracks rather than adding a second one. Not silently dropped — named here so it cannot be lost |
| 2 | `FifoSessionFailReason` (2 values: `BAD_FRAME_TYPE`, `DESYNC`) is narrower than the frozen 14-code, domain-classified `FifoError` taxonomy (D-18) — a naive 1:1 forwarding into `FifoCaptureResult.error` would under-specify the failure the way PRR C-7 originally criticized | Task 3.4 must perform an explicit `FifoSessionOutcome → FifoError` translation (e.g. `SESSION_FAILED` + `failReason==DESYNC` → `ERR_DESYNC_LIMIT`), not treat the two enums as interchangeable. Flagged so this translation step is designed deliberately, not discovered as a bug later |
| 3 | `fifo_session.{h,cpp}` currently exist uncommitted, written ahead of this formal decision | Closed by this ADR going forward; recommend committing only after all four documents in this set are accepted (§7) |
| 4 | No Session-Controller-specific test fixture exists yet; Task 3.7 (renumbered test suite, old 3.6) did not previously scope for it | Implementation Plan delta adds explicit fixture scope for Session Controller under Task 3.7 |

## 10. Future tasks affected

- **Task 3.2 (shared types, renumbered from 3.1):** must define `FifoError` such that Session Controller's outcome/fail-reason pair can be losslessly translated by Task 3.4 — no `FifoSessionOutcome`/`FifoSessionFailReason` value should map to more than one `FifoError` code, and no `FifoError` code relevant to L2/L3 framing should be unreachable from a Session Controller outcome.
- **Task 3.3 (sample arena, renumbered from 3.2):** unaffected structurally — `FifoArena`'s write handles are still supplied by `FifoDriver`, still passed straight through Session Controller to `FrameCodec_Step()` unchanged.
- **Task 3.4 (core protocol state machine, renumbered from 3.3):** the task most affected. Its per-state handlers for `S4`/`S5`/`S6`/`S8` are replaced by "call `FifoSession_Step()` once per tick, branch on `FifoSessionOutcome`." Gains two new obligations: the K-10 comparison (Risk 1) and the outcome→`FifoError` translation (Risk 2).
- **Task 3.5 (recovery/lifecycle, renumbered from 3.4):** retry-attempt policy now operates at `FifoSession_StartAttempt()` granularity for a fresh attempt, and relies on Session Controller's documented self-healing of `PROGRESS`/`CRC_MISMATCH` (auto-loop to `WAIT_FRAME`, no external retry needed) versus `BAD_TYPE_BYTE`/`DESYNC_LIMIT` (external retry-attempt policy required). No numeric timeout/retry constant changes.
- **Task 3.6 (public API, renumbered from 3.5):** no change — `FifoDriver`'s 9-entry surface is untouched (§6).
- **Task 3.7 (test suite, renumbered from 3.6):** gains explicit scope for a Session-Controller-only fixture, exercising the outcome table in `fifo_session.h`'s doc comment independent of the full `FifoDriver` state machine — cheap to add given Session Controller's own zero-Arduino-dependency host-testability.

## References

- `CM-100_FIFO_Implementation_Plan_v1.4_Delta.md` — full Phase 3 task renumbering and new Task 3.1 specification
- `CM-100_FIFO_DRIVER_SDS_v1.4_Delta.md` — §5.2, §7, §11, §11.4, §20.1, §24 section-by-section changes
- `CM-100_FIFO_Architecture_Freeze_Report_v1.4_Delta.md` — updated freeze scope and outstanding-work table
- `CM-100_FIFO_DRIVER_SDS_v1.0.md` §11.4, §20.1, §24 (frozen v1.3 baseline this ADR amends)
- `CM-100_FIFO_Implementation_Plan_v1.0.md` Phase 3 (frozen v1.3 baseline this ADR renumbers)
- `fifo_session.h`, `fifo_session.cpp` (working tree, uncommitted, unmodified by this ADR)
- Task 3.1A Integration Review (this session, prior turn) — the review that surfaced the original mismatch

---

*End of ADR-0004 — Architecture Decision Record only; no implementation code is specified, modified, or implied.*
