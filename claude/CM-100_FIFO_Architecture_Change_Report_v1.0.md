> **Document Status**
>
> **Type:** Architecture Change Report — companion to `CM-100_FIFO_DRIVER_SDS_v1.2` and `CM-100_FIFO_Implementation_Plan_v1.2`.
> **Date:** 2026-07-28
> **Trigger:** Task 1.2 implementation review. Re-verification of production source against the code just written surfaced that `rs485Enable()`/`rs485Disable()` are paired, session-scoped calls, not the model either the v1.1 SDS text or the v1.1 Implementation Plan assumed.
> **Scope:** RS485 EN-pin ownership only. No other frozen decision is changed — see §5.
> **Status:** documentation updated. **Firmware source (`fifo_transport_uart485.{h,cpp}`) not yet modified** — Task 1.2's code still reflects the superseded model and requires revision before Task 1.3 proceeds.

# CM-100 FIFO Driver — Architecture Change Report: RS485 EN-Pin Ownership

---

## 1. Decision Summary

| | Before (v1.1) | After (v1.2) |
|---|---|---|
| **Owner** | `Uart485Transport` (Task 1.2) | `taskModbusRead()`'s `OwnsBus()` boundary (Task 4.2) |
| **Mechanism** | `digitalWrite(pin, LOW)` inside `write()`, using a locally-duplicated pin constant | Calls to the real, existing `rs485Enable()`/`rs485Disable()`, from inside the `.ino` |
| **Pattern** | Asserted before every `write()` call, never deasserted by this file | Asserted once when a FIFO bus-ownership window opens, deasserted once when it closes — symmetric with every other bus window in production |
| **`Uart485Transport`'s role** | Owns EN + moves bytes | Moves bytes only |

**Reason, in one sentence:** D-15 says `taskModbusRead()` is sole owner of the RS485 bus; verified production source shows "owning the bus" already includes EN assertion/deassertion for every existing caller, and the FIFO capture should be bracketed the same way rather than introducing the first exception to that pattern.

---

## 2. Every Affected SDS Section (`CM-100_FIFO_DRIVER_SDS`, v1.1 → v1.2)

### §6.2, Rationale item 3

**Before:**
> "It insulates the driver from the RS485 direction-control question. Production toggles `rs485Enable()`/`rs485Disable()` around bus windows; the Validation Tool held the pin permanently LOW. The transport implementation owns that difference; the driver never sees it."

**After:**
> "It insulates the driver from the RS485 EN-pin question entirely. `FifoTransport` has no EN-related member, and `Uart485Transport` never references `RS485_EN_PIN`. EN assertion/deassertion is owned by `taskModbusRead()`'s bus-ownership boundary (§13.2, Task 4.2) — the same actor, and the same mechanism, that already brackets every other Modbus transaction burst in production."

**Reason:** The "before" text was true in outline (production does toggle EN) but assigned ownership of that toggling to the wrong layer — "the transport implementation owns that difference" was read, in v1.1, as *Task 1.2 performs the toggling*. Re-verification showed toggling is session-scoped (one enable, many transactions, one disable), not per-call, and is already performed by the actor that decides window boundaries (`taskModbusRead()`), not by the byte-transport layer beneath it.

### §13.2, Integration shape (pseudocode) + new §13.2.1

**Before:** The `OwnsBus()` branch pseudocode had no mention of EN at all — `// suspend normal polling`, `// do NOT touch modbus.*`, `// do NOT increment g_modbusConsecErrors`, `// publish capture-active state`. No statement anywhere in §13 addressed who asserts EN for a FIFO transaction.

**After:** Pseudocode gains two lines — `rs485Enable();` on entering the branch, `rs485Disable();` when ownership releases. New subsection §13.2.1 states ADR-1's interpretation that bus ownership includes EN ownership, the reachability argument for why `write()` can assume EN is already asserted (state `S3` is unreachable before `OwnsBus()` is true, which is unreachable before `rs485Enable()` has been called), and a new requirement that `rs485Disable()` fire on every release path, not only normal completion.

**Reason:** This is **new content**, not a correction — the gap simply didn't exist as text before, because the question "who asserts EN for a FIFO transaction" had not yet been asked precisely enough to need an answer in v1.0/v1.1.

### §20.1, Class diagram

**Before:** `Uart485Transport` box listed `+ rs485Enable/Disable` as one of its members.

**After:** Box states "pure byte I/O only... NO `RS485_EN_PIN` reference -- EN owned by `taskModbusRead`'s `OwnsBus()` boundary, §13.2." *(Incidental fix made in the same pass, unrelated to EN: the `FifoTransport` interface box was missing `hadOverflow()`, present in prose since v1.1 but never added to this diagram — added.)*

**Reason:** Diagram was drawn under the v1.1 model; corrected to match.

### Revision Notes (new "v1.1 → v1.2" section)

Added, documenting this change's trigger, scope, and the three sections above — same pattern as the existing "v1.0 → v1.1" section, for audit-trail continuity.

### Document header

Version bumped 1.1 → 1.2. Design Review status changed from "🟢 Frozen" to "Frozen (v1.1); EN-ownership model updated in v1.2, not yet re-frozen." Implementation status line added, stating Task 1.2's code has not yet been updated to match.

---

## 3. Every Affected Implementation Plan Task (`CM-100_FIFO_Implementation_Plan`, v1.1 → v1.2)

### Task 1.2 — Production transport implementation

**Purpose — before:** *"Concrete `FifoTransport` wrapping the existing `SerialRS485` object — the only file in the entire driver that touches Arduino's `HardwareSerial` API directly."*
**Purpose — after:** Same sentence, plus: *"A pure byte transport: moves bytes in and out of the UART only. It does not assert, deassert, or reference the RS485 EN pin in any form — that ownership belongs entirely to `taskModbusRead()`'s `OwnsBus()` boundary (Task 4.2, SDS §13.2.1)."*

**Internal interfaces — before:** Described five callbacks as "thin wrapper[s] over the corresponding `HardwareSerial`/`millis()` call," with no statement either way about EN.
**Internal interfaces — after:** Explicit negative statement added: *"No callback references `RS485_EN_PIN`, `digitalWrite`, or any pin constant — `write()` is reduced to `serial->write(src, len)`, nothing else."*

**Risks — before:** ~200 words describing why `write()` must wrap itself in `rs485Enable()`/`rs485Disable()`, the `static inline` linkage problem, and the locally-duplicated pin constant as a disclosed liability.
**Risks — after:** Entire section rewritten. States the v1.1 resolution is superseded, states the reason (verified paired/session-scoped production calls), and reframes the remaining risk as a **cross-task dependency**: `Uart485Transport_Write()` is only correct if Task 4.2 has already asserted EN — this file cannot verify that precondition itself and does not attempt to.

**Verification method — addition:** Phase 4's on-target test must now specifically confirm the *first transmitted byte reaches the bus* — not just that `write()` returns a byte count — since a missing or mistimed EN assertion in Task 4.2 would produce exactly that failure mode (a byte count returned, nothing physically transmitted).

**Dependencies — addition:** Now implicitly depends on Task 4.2's correctness, flagged explicitly since Task 4.2 is sequenced later in the plan.

### Task 4.2 — Mode branch in `taskModbusRead()`

**Purpose — addition:** *"This task now also owns RS485 EN-pin bracketing (SDS §13.2.1) — `rs485Enable()` on entry to the `OwnsBus()` branch, `rs485Disable()` when bus ownership releases — moved here from Task 1.2, which no longer touches the pin at all."*

**Internal interfaces — before:** "None."
**Internal interfaces — after:** *"Calls the existing, unmodified `rs485Enable()`/`rs485Disable()` (`static inline`, same translation unit — no linkage issue here, unlike Task 1.2)."*

**Risks — addition:** New item: `rs485Disable()` must fire on every path that releases `OwnsBus()` — normal completion, `Abort()`, and circuit-breaker trip — not only the happy path. Explicitly notes this is a correctness-versus-stated-goal gap, not a safety hazard (normal polling's own unconditional reassert means a stuck-enabled pin is not itself dangerous), but must not be introduced silently.

**Verification method — addition:** On-target test must confirm `rs485Disable()` fires after an aborted capture and after a simulated circuit-breaker trip, not only after normal completion.

**Dependencies — addition:** Cross-reference note that Task 1.2 now depends on this task's correctness.

### Revision Notes / Header

Same pattern as the SDS: new "v1.1 → v1.2" section added; version bumped; header's "Implementation status" line states Task 1.2's code requires revision before Task 1.3.

---

## 4. Impact Analysis

| Area | Impact |
|---|---|
| **`fifo_transport_uart485.cpp` (already written)** | Requires code changes: remove `#define UART485_RS485_EN_PIN`, remove the `digitalWrite` call and its surrounding doc comment in `write()`, remove all EN-related reasoning from every callback's doc comments. **Not yet done** — explicitly out of scope for this turn per instruction. |
| **`fifo_transport_uart485.h` (already written)** | No change required — the header's public contract (`Uart485Transport_Init(FifoTransport*, HardwareSerial*)`) doesn't mention EN and doesn't need to. |
| **`fifo_transport.h` (Task 1.1, already committed... not actually committed, see below)** | No change — `FifoTransport`'s interface never had an EN-related member; this decision was always encapsulated below the interface line, in whichever concrete transport implemented it. |
| **Commit history / tags** | `v16.6.3-fifo`'s planned content changes (smaller diff: no pin constant, no digitalWrite). Task 4.2's planned commit (`v16.6.16-fifo`, not yet written) grows by two function calls plus the every-path-disable requirement. Total commit count in the Implementation Plan is unchanged — no phase added or removed. |
| **Build/size** | Expected to be net-neutral or slightly favorable: removes one `#define`, one `digitalWrite` call, and ~60 lines of now-obsolete doc comment from Task 1.2; adds two function calls (already-existing functions) to Task 4.2. No new files, no new translation units. |
| **Timing budget** | No change to the SDS's stated `write()` blocking budget (~8.3ms) — EN assertion moves to Task 4.2's tick-boundary code, which was already going to execute regardless of this decision. |
| **G-1 / G-2** | Unaffected — neither open gate has any relationship to EN-pin ownership. |

---

## 5. Confirmation: No Other Architecture Decision Is Affected

Checked explicitly against every other Freeze Review finding and every SDS decision ID touched by prior revisions, since this change was found and applied outside the normal review sequence:

| Decision / Finding | Re-checked | Result |
|---|---|---|
| **D-1** (tick-sliced timing) | `write()`'s budget (~8.3ms) is unchanged; EN assertion in Task 4.2 happens at the tick boundary, which already existed | Unaffected |
| **D-7** (injected transport interface) | *"The driver receives an injected transport interface. It never references `SerialRS485`, `Serial`, or `modbus`."* — true regardless of which file asserts EN, since neither `Uart485Transport` nor L2/L3 ever exposed EN to the driver in either model | Unaffected |
| **D-15** (sole bus owner, no new mutex) | **Extended, not contradicted** — this change is precisely an application of D-15's existing scope to a question the original text hadn't been forced to answer yet. No new mutex, no new task, no change to `taskModbusRead()` remaining the sole `modbus.*` caller | Extended, consistent |
| **D-16** (single-writer thread model) | EN-pin state is not part of any `FifoDriver` session state, request, or result object — it's a GPIO write local to `taskModbusRead()`'s own execution, same as it always was for normal polling. No new cross-task data path introduced | Unaffected |
| **Finding 1.1** (L2/L3-arena decoupling) | Unrelated subsystem (L2 codec / arena), not touched by this change | Unaffected |
| **Finding 2.1** (single result coordinator) | Unrelated subsystem (result ownership), not touched | Unaffected |
| **Finding 3.1** (`FrameCodecState` explicit) | Unrelated subsystem (L2 parsing state), not touched | Unaffected |
| **Finding 3.2** (`FifoAdmissionContext`) | Unrelated subsystem (admission gates), not touched | Unaffected |
| **Finding 5.1** (`ABSENT_STOPPING_MS` false coupling) | Independently re-checked: that finding is about a *timing* constraint on capture duration, unrelated to *which file* asserts a GPIO. No interaction between the two | Unaffected |
| **Finding 7.1** (`hadOverflow`, `ERR_PROGRESS_*`) | Unrelated subsystem (transport error signaling, L3 progress validation), not touched | Unaffected |
| **CN-6** (shared bus with CTR4A01) | Task 4.2's Risks field already required suspending CTR4A01 polling, unchanged by this update — EN bracketing wraps the same `OwnsBus()` window that was already suspending it | Unaffected |
| **CN-10** (`g_modbusConsecErrors` non-increment) | Independent GPIO write, no interaction with this counter | Unaffected |

No decision required re-opening. This was a single, isolated correction to one ownership question, consistent with the instruction's own framing.

---

## 6. Risks Introduced by This Change

1. **New cross-task dependency, disclosed in Task 1.2's Risks field.** `Uart485Transport_Write()`'s correctness now depends on code that doesn't exist yet (Task 4.2). Between now and Task 4.2's implementation, Task 1.2's transport is *not independently functional* on real hardware — it was previously self-contained (if imperfect). This is a sequencing risk, not a design flaw: Task 4.2 was always going to be needed for the driver to work end-to-end; this change makes an implicit dependency explicit rather than introducing a new one.
2. **`rs485Disable()`-on-every-path is a new correctness obligation on Task 4.2**, not present in the original integration shape. Flagged as a specific risk item and a specific verification-method addition in both documents, precisely so it isn't lost.
3. **No risk to already-shipped production behavior.** Task 4.2 is not yet implemented; the `.ino`'s existing `taskModbusRead()` is untouched by this document-only update.

## 7. Migration Required

1. **`fifo_transport_uart485.cpp`** (already written under the v1.1 model) needs revision: remove `#define UART485_RS485_EN_PIN`, remove the `digitalWrite()` call in `write()`, remove the EN-ownership reasoning from every doc comment that references it. **Not performed in this turn**, per instruction — pending explicit approval to resume code changes.
2. **`fifo_transport_uart485.h`** — no migration needed.
3. **No commits exist yet for Task 0.1, 1.1, or 1.2** (confirmed in the prior turn's git-status check) — there is nothing to *revert*, only code still sitting in the working tree to *revise* before it is ever committed. This is the cheapest possible time to make this change: before any of it has entered history.
4. **Task 4.2, when its turn comes**, must be implemented against this updated plan from the start — no separate migration step needed there, since it hasn't been written yet.

---

*End of CM-100_FIFO_Architecture_Change_Report_v1.0. `CM-100_FIFO_DRIVER_SDS_v1.2` and `CM-100_FIFO_Implementation_Plan_v1.2` (same filenames, versions bumped internally) reflect the "after" state described above. No firmware source modified. No build run. Nothing committed.*
