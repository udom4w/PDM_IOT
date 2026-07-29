> **Document Status**
>
> **Type:** Software Design Specification Delta — companion to `CM-100_FIFO_DRIVER_SDS_v1.0.md` (internal version 1.3, frozen). Records every section changed by `CM-100_ADR-0004_Session_Controller_Promotion_v1.0.md`. Same pattern as `CM-100_FIFO_Architecture_Change_Report_v1.0.md`'s §2 (per-section before/after).
> **Base document, version amended:** `CM-100_FIFO_DRIVER_SDS_v1.0.md`, internal version 1.3 → 1.4.
> **Trigger:** ADR-0004.
> **Scope:** §5.2, §7.1, §11.1, §11.4, §20.1, §24. No other section is touched — see §6 (confirmation).
> **Status:** documentation only. No firmware source changed.

# CM-100 FIFO Driver SDS — v1.4 Delta: Session Controller Promotion

---

## 1. Decision Summary

Session Controller — previously an undocumented, inline responsibility of `FifoDriver`'s per-state handlers for `S4`/`S5`/`S6`/`S8`, and a data-only field (`session : FifoSession`) in the §20.1 class diagram — is promoted to a standalone L3 sub-component (`fifo_session.h`/`fifo_session.cpp`) with its own public API, composed by `FifoDriver` rather than inlined into it. See ADR-0004 for full motivation and consequences; this document carries the change into the SDS's own sections.

---

## 2. §5.2 Layering

**Before:**

```
┌──────────────────────────────────────────────────────────────┐
│  L4  Diagnostic Workflow    (§19)                            │
├──────────────────────────────────────────────────────────────┤
│  L3  Capture Session        (§11, §12, §17)                  │
│      state machine · retry budget · cooldown · circuit breaker│
├──────────────────────────────────────────────────────────────┤
│  L2  Frame Codec            (§8, §16)                        │
├──────────────────────────────────────────────────────────────┤
│  L1  Transport Abstraction  (§6)                             │
└──────────────────────────────────────────────────────────────┘
```

**After:**

```
┌──────────────────────────────────────────────────────────────┐
│  L4  Diagnostic Workflow    (§19)                            │
├──────────────────────────────────────────────────────────────┤
│  L3  Capture Session        (§11, §12, §17)                  │
│      FifoDriver: retry budget · cooldown · circuit breaker ·  │
│      admission · arena · bus/EN ownership · result lifecycle  │
│        composes ▼                                              │
│      SessionController  [v1.4, ADR-0004] one FrameCodecState  │
│      per attempt · outcome translation (§11.4) · phase        │
│      classification (WAIT_FRAME/RECEIVING/COMPLETE/FAILED)    │
├──────────────────────────────────────────────────────────────┤
│  L2  Frame Codec            (§8, §16) — unchanged             │
├──────────────────────────────────────────────────────────────┤
│  L1  Transport Abstraction  (§6)                              │
└──────────────────────────────────────────────────────────────┘
```

**Reason:** L3 was previously drawn as one undifferentiated box. Session Controller is now a named, separately testable sub-layer within it — narrower than all of L3 (it owns none of retry/cooldown/breaker/admission/arena/bus), but broader than "just a data field," which is what the old diagram implied.

---

## 3. §7.1 Public API Surface

**Before:** Table of 9 entry points (`FifoDriver_Init` .. `FifoDriver_GetStats`), no mention of any sub-component API.

**After:** Table unchanged — all 9 entries identical, no addition, no removal, no signature change. New paragraph appended:

> **[v1.4, ADR-0004]** `FifoDriver` internally composes a Session Controller sub-component (`fifo_session.h`), exposing three module-scoped functions — `FifoSession_Init()`, `FifoSession_StartAttempt()`, `FifoSession_Step()` — consumed exclusively by `fifo_driver.cpp`. These have extern linkage (the same C++ mechanism `FrameCodec_Step()`/`FrameCodecState_Reset()` already use) but are not part of the driver's public API surface described in this section — no code outside the FIFO driver calls them, and none should. This does not change the "Nine entry points... deliberately small" framing above; it is an internal composition detail, not a tenth entry point.

**Reason:** Prevents a future reader from mistaking Session Controller's extern-linkage functions for additions to the deliberately minimal 9-entry surface §7.3 argues for.

---

## 4. §11.1 States / §11.2 Decomposition

**Before:** Per-state table (`S0`..`S14`) with no distinction between "this state's action is inline in `fifo_driver.cpp`" and "this state's action is delegated to a sub-component."

**After:** New note appended after the table:

> **[v1.4, ADR-0004]** States `S4 AWAIT_ANCHOR`, `S5 READ_TYPE`, `S6 READ_PROGRESS`, and `S8 READ_DUMP` are, from `FifoDriver`'s state machine's own perspective, a single composed action: call Session Controller's `FifoSession_Step()` once per tick, and branch on the returned `FifoSessionOutcome` (`NONE` → remain in the composed sub-state; `PROGRESS_UPDATE` → publish diagnostics and remain armed; `DUMP_COMPLETE` → advance to `S9 VERIFY`; `FRAME_REJECTED` → remain armed, already re-armed by Session Controller itself; `SESSION_FAILED` → advance to `S9 VERIFY` with `FifoSessionState.failReason` set). `FifoDriver` no longer threads `FrameCodecState` by pointer into `FrameCodec_Step()` itself for these four states — that thread now lives entirely inside `FifoSessionState`, owned by Session Controller (§11.4). This is a re-shaping of *who calls L2*, not a change to *what L2 does* — `FrameCodec_Step()`'s own contract, and Task 2.3's freeze, are untouched.
>
> `FifoDriver` retains direct, unmediated ownership of every other state (`S0`–`S3`, `S7`, `S9`–`S14`) and every cross-attempt policy: retry-attempt decisions on `SESSION_FAILED`, the `S6`↔`S7`/`S4` protocol-model edge (D-14), bus/EN bracketing, admission, arena, circuit breaker, and all timeout detection (Design Closure §2 — Session Controller never calls `nowMs()` and this note does not change that).

**Reason:** Makes the state-machine/sub-component boundary explicit rather than leaving a reader to infer it from the Implementation Plan alone.

---

## 5. §11.4 Frame-Parsing State Ownership

**Before:**

> *"This state is an explicit `FrameCodecState` struct, defined by L2 (it's L2's own state shape) but **owned and allocated by the session** (L3) and passed by pointer into every call, reset at the start of each new frame attempt."*

**After:**

> *"This state is an explicit `FrameCodecState` struct, defined by L2 (it's L2's own state shape) but **owned and allocated as a field of `FifoSessionState`, inside Session Controller [v1.4, ADR-0004]** — a specific sub-component of L3, not L3 generically — and passed by pointer into every `FrameCodec_Step()` call, reset at the start of each new frame attempt (`FifoSession_Init()`, and at every terminal outcome inside `FifoSession_Step()` — see `fifo_session.h`'s own outcome table for the exact reset boundary on each of the five terminal `FifoFrameOutcome` values `FrameCodec_Step()` can produce (`PROGRESS`, `FULL_DUMP`, `CRC_MISMATCH`, `BAD_TYPE_BYTE`, `DESYNC_LIMIT` — every producible value except `PENDING`), which collapse into the four non-`NONE` `FifoSessionOutcome` values, since `BAD_TYPE_BYTE` and `DESYNC_LIMIT` both map to `SESSION_FAILED`)."*

Second paragraph (`lastProgressFill` cross-frame semantics) — **before:** *"already exists as a tracked/published diagnostic field (§8.2) — `S6` reuses it as the comparison anchor..."* — **after**, sentence appended:

> *"[v1.4] `lastProgressFill` is populated by Session Controller on every `PROGRESS_UPDATE` outcome (`fifo_session.h`), but the K-10 regression/overrun **comparison** against it is not performed by Session Controller — that comparison remains `FifoDriver`'s (Task 3.4, renumbered) responsibility, using this same field as its anchor. This is a known, explicitly tracked gap as of v1.4, not an oversight — see ADR-0004 §9, Risk 1, and §6 below."*

**Reason:** The original text's "owned by the session (L3)" was accurate but generic enough to permit exactly the ambiguity ADR-0004 §1 identified. This makes the ownership concrete and traceable to the actual struct and file.

---

## 6. Known Gap Carried Forward: K-10 Validation

**[v1.4, new subsection, not present in v1.3]** Freeze Review Finding 7.1 (v1.0→v1.1) assigned `ERR_PROGRESS_REGRESSION`/`ERR_PROGRESS_OVERRUN` detection to *"`S6`'s handler."* Under the v1.4 layering, `S6`'s handler is Session Controller's `FifoSession_Step()`, and — verified by direct inspection of `fifo_session.cpp` as part of this promotion — **it does not perform this comparison.** `FifoSessionOutcome` has no `PROGRESS_REGRESSION`/`PROGRESS_OVERRUN` value; a `fill` that regresses or exceeds 6144 currently reaches `FifoDriver` indistinguishable from any other `PROGRESS_UPDATE`.

This is recorded here, not silently inherited, because Finding 7.1 exists specifically to make this class of protocol violation field-reportable (RFC-0006) rather than silently accepted. **Formally reassigned to Task 3.4** (renumbered core state machine, Implementation Plan v1.4 Delta §5): compare `FifoSessionState.lastProgressFill` across successive `PROGRESS_UPDATE` outcomes within one attempt, using the field Session Controller already tracks. No new tracking variable is introduced; the gap is a missing comparison, not a missing value.

---

## 7. §20.1 Class / Module Diagram

**Before:**

```
│  FifoDriver                                                    (L3)   │
│  - session     : FifoSession   (timers, retry budget, provenance,     │
│                   lastProgressFill, FrameCodecState ◄── §11.4 [v1.1]) │
```

**After:**

```
│  FifoDriver                                                    (L3)   │
│  - session     : FifoSessionState  [v1.4, ADR-0004]                   │
│                   (owned by SessionController — FrameCodecState,      │
│                   phase, failReason, lastProgressFill; see §11.4)     │
│  - retryBudget, provenance, timers : remain direct FifoDriver fields  │
│                   (unchanged from v1.3 — NOT part of FifoSessionState)│
│    ▲ composes                                                         │
│  SessionController  [v1.4, NEW BOX]                            (L3)   │
│  ─────────────────────────────────────────────────────────────────    │
│  + Init(FifoSessionState*)                                            │
│  + StartAttempt(FifoSessionState*)                                    │
│  + Step(FifoSessionState*, FifoTransport*, int16_t*×3) : Outcome      │
│    ▲ uses (never owns)                                                │
│  FrameCodec (L2) — unchanged, see below                               │
```

**Reason:** The v1.3 diagram drew `FifoSession` as a plain data member with no box of its own and no operations. The promoted component needs its own box, matching how `FrameCodec`/`StreamingCrc16`/`SampleDecoder`/`FifoArena` already each get one. Explicitly calls out that `retryBudget`/`provenance`/`timers` do **not** move into `FifoSessionState` — only the fields Session Controller's actual code carries do (`FrameCodecState`, `phase`, `failReason`, `lastProgressFill`), preventing a future reader from assuming the promotion silently expanded Session Controller's scope.

---

## 8. §24 Summary of Design Decisions

**Addition to the Related Architecture Decision Notes table:**

| ID | Note | Interprets | Primary rationale |
|---|---|---|---|
| ADR-0004 | Promote Session Controller to a standalone L3 component (`CM-100_ADR-0004_Session_Controller_Promotion_v1.0.md`) | §11.4, §20.1 (both amended, not contradicted — see §5/§7 above) | Formalizes a component that already existed in code (`fifo_session.{h,cpp}`, tag `v16.6.8-fifo`) ahead of a documented decision; makes the split between "mechanical outcome translation" (Session Controller) and "capture policy" (`FifoDriver`) explicit rather than implicit |

No row in the D-1..D-20 table itself changes. **This is an amendment to how §11.4/§20.1 describe an existing decision's shape, not a new standing design rule** — consistent with the pattern ADR-1 already established (interprets, does not redefine).

---

## 9. Confirmation: No Other Decision Is Affected

| Decision / Section | Re-checked | Result |
|---|---|---|
| D-1 (tick-sliced timing) | Session Controller adds no blocking call, no new state, no change to `FifoDriver_Service()`'s per-tick budget | Unaffected |
| D-2/D-12 (zero heap, static allocation) | `FifoSessionState` is a plain struct, caller-owned, no dynamic allocation | Unaffected |
| D-6 (no hidden state) | `FifoSessionState` is fully explicit and caller-owned, same discipline §11.4 already required of `FrameCodecState` | Reinforced, not weakened |
| D-7 (injected transport) | Session Controller receives `FifoTransport*` exactly as `FrameCodec_Step()` already did; no new transport dependency | Unaffected |
| D-8 (`status` sole authority on validity) | Session Controller's outcomes are pre-`status` signals consumed only by `FifoDriver`; sample validity gating is unchanged | Unaffected |
| D-14 (protocol-model edge, `FIFO_PROTOCOL_MODEL`) | Still owned entirely by `FifoDriver` (§4 above) — Session Controller has no opinion on TRUEPOLL vs. PURELISTEN | Unaffected |
| D-15 / ADR-1 (bus/EN ownership) | Session Controller never calls `t->write()` and has no bus/EN awareness of any kind (`fifo_session.h`'s own explicit out-of-scope list) | Unaffected |
| D-16 (single-writer) | Session Controller is called only from `FifoDriver`, itself called only from `taskModbusRead()` — no new writer, no new cross-core path | Unaffected |
| D-18 (14-code error taxonomy) | Not weakened by this promotion, but **requires an explicit translation step** now formally assigned to Task 3.4 (ADR-0004 §9 Risk 2 — note: this obligation is distinct from §6's K-10 gap and is not covered there) — flagged, not silently satisfied | Extended, translation obligation made explicit |
| §15 (timeout strategy) | Session Controller never calls `nowMs()`, matching Design Closure §2 exactly | Unaffected |
| §17 (recovery strategy) | Byte-level `RESYNC` and frame-level `RETRY_POLL` recovery are Session Controller's automatic `WAIT_FRAME` loop-back (`PROGRESS`/`CRC_MISMATCH`); attempt-level `RETRY_ATTEMPT` and session-level `FAIL` remain `FifoDriver`'s, unchanged | Unaffected in policy, now explicitly mapped to the new layering |

No decision required re-opening. This is a documentation-shape correction to §11.4/§20.1, not a new architectural tradeoff.

---

*End of CM-100_FIFO_DRIVER_SDS v1.4 Delta. `CM-100_FIFO_DRIVER_SDS_v1.4` (same filename convention as the base document — internal `Version:` field bumps, filename does not) is understood to be this document's "before" text with the changes above applied, whenever that mechanical merge is next performed. No firmware source modified.*
