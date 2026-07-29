> **Document Status**
>
> **Type:** Architecture Decision Note (ADR) — records one implementation-level interpretation of an existing frozen decision. Not itself a design document and not a substitute for the SDS or Implementation Plan.
> **ID:** ADR-1
> **Title:** Bus ownership includes EN ownership
> **Status:** Accepted
> **Date:** 2026-07-28
> **Supersedes:** nothing. **Interprets:** `CM-100_FIFO_DRIVER_SDS_v1.0` D-15 (§13.1). **Does not amend, redefine, or replace D-15's text.**
> **Raised by:** Architecture Review Feedback on `CM-100_FIFO_DRIVER_SDS` v1.2 / `CM-100_FIFO_Implementation_Plan` v1.2, item 3.
> **Affects:** `CM-100_FIFO_DRIVER_SDS_v1.0` §6.1, §6.2 item 3, §13.2.1, §24; `CM-100_FIFO_Implementation_Plan_v1.0` Task 1.2, Task 4.2.
> **No firmware changes result from this note by itself** — it records an interpretation; the code consequence (Task 1.2's `.cpp` no longer touching `RS485_EN_PIN`) is tracked as pending implementation work in the Implementation Plan, not performed here.

# ADR-1 — Bus ownership includes EN ownership

## 1. Context

D-15 states: *"`taskModbusRead()` remains the sole owner of the RS485 bus and the sole caller of `modbus.*`. The FIFO driver is a mode of that task, not a peer. No new mutex is introduced anywhere."*

D-15's text says nothing about the RS485 EN pin. When Task 1.2 (`Uart485Transport`, the FIFO driver's production transport) was first implemented, it read D-15 as covering only the Modbus byte stream, and independently asserted `RS485_EN_PIN` inside its own `write()` — duplicating the pin constant, since the real `rs485Enable()`/`rs485Disable()` functions are `static inline` and have no external linkage outside the `.ino` translation unit.

During Task 1.2's implementation review, re-examination of the verified production source found three existing bus-ownership windows, each already pairing EN assertion with the transaction it brackets:

| Call site (approx. line) | Pattern |
|---|---|
| `:4334` / `:4425` | `rs485Enable()` ... Modbus transaction burst ... `rs485Disable()` |
| `:4571` / `:4584` | `rs485Enable()` ... Modbus transaction burst ... `rs485Disable()` |
| `:4680` | `rs485Enable()` (paired release elsewhere in the same function) |

In every case, the actor that decides *when* the bus is acquired and released is `taskModbusRead()` itself, and that same actor also asserts and deasserts EN. No existing code path splits "who decides the window" from "who toggles EN" across two different actors.

This raised a genuine question, addressed by the Architecture Update turn and revisited here: **does D-15's grant of sole bus ownership to `taskModbusRead()` implicitly include EN, or is EN a separate concern that some other actor — such as the transport — may legitimately own?**

The Architecture Update turn resolved this by rewriting §13.2.1 to say "D-15 extended... includes EN-pin assertion." Architecture Review Feedback on that turn correctly flagged this as **redefining a frozen decision** rather than **recording a new, separate interpretation of it** — the fix is not to change the conclusion, but to change where it lives. This note is that fix.

## 2. Decision

**Bus ownership, as granted to `taskModbusRead()` by D-15, is interpreted to include EN-pin assertion and deassertion.** `taskModbusRead()`'s `OwnsBus()` boundary (Implementation Plan Task 4.2) calls `rs485Enable()` on entering a FIFO-capture bus window and `rs485Disable()` on releasing it, using the same functions and the same bracketing pattern already used at every other bus-ownership window in production.

`Uart485Transport` (Task 1.2) does not assert, deassert, or reference `RS485_EN_PIN` in any form. Its `write()` contract instead documents an explicit precondition: the caller already owns the RS485 bus and has already asserted EN (SDS §6.1). `Uart485Transport` is a pure byte transport against that precondition — it does not, and structurally cannot, verify the precondition itself (per D-7, the transport has no visibility into bus arbitration by design).

**D-15's own text is not changed by this decision.** D-15 continues to say exactly what it said at Architecture Freeze: sole bus ownership, sole `modbus.*` caller, no new mutex. This note documents how that ownership is exercised for one specific resource — the EN pin — when the FIFO driver's bus window is added as a new case alongside the three that already existed.

## 3. Rationale

1. **Consistency with every existing bus window.** All three verified production call sites already pair EN with the transaction it brackets, with the same actor deciding both. Splitting that pairing for exactly one window (the new FIFO one) would introduce the first exception to an otherwise universal pattern, for no benefit.
2. **Guaranteed ordering by construction, not by convention.** `Uart485Transport_Write()` is only reachable once the FIFO driver's state machine has reached `S3 REQUEST`, which is only reachable after `S2 ARMED`, which is only reachable after `FifoDriver_OwnsBus()` has already become `true`. Because `rs485Enable()` is asserted unconditionally at that same `OwnsBus()` boundary, every tick the branch is taken, `write()` can simply transmit — there is no runtime race to protect against, and no runtime check the transport needs to perform.
3. **No new actor, no new synchronization primitive.** `taskModbusRead()` already is the sole bus owner per D-15. Assigning it EN as well introduces no second writer, no new mutex, and no cross-core coordination — it is strictly smaller in scope than either alternative below.
4. **Removes a disclosed implementation liability.** The original Task 1.2 implementation carried a locally-duplicated `RS485_EN_PIN` constant, forced by `rs485Enable()`/`rs485Disable()`'s internal linkage. Moving EN ownership to Task 4.2 (same translation unit as the real functions) removes the duplication entirely rather than working around it.

## 4. Alternatives considered

| Option | Description | Rejected because |
|---|---|---|
| A — Transport owns EN (original Task 1.2) | `Uart485Transport::write()` asserts/deasserts `RS485_EN_PIN` directly | Requires a duplicated pin constant (linkage limitation); splits the "acquire bus / assert EN" pairing across two actors for the first time in this codebase; the transport would be reaching outside the byte-transport role D-7 assigns it |
| B — Permanently-enabled model | Assert EN once at boot, never deassert (the Validation Tool's model) | Not what verified production does — production's three existing bus windows all pair enable/disable around themselves, and production shares the bus with the CTR4A01 current sensor (CN-6), which the Validation Tool never had to coordinate with |
| C — New dedicated EN-owner component | Introduce a separate object/task solely responsible for EN, referenced by both `taskModbusRead()` and the FIFO driver | Adds a new actor and a new coordination surface where D-15 already assigns sole bus ownership to one existing actor; unjustified complexity for a single GPIO |
| **D — Chosen: `taskModbusRead()` owns EN via its existing `OwnsBus()` boundary** | EN bracketed at the same boundary that already decides bus-window timing | Zero new actors, zero new primitives, matches all three existing production call sites, and keeps D-15 itself untouched |

## 5. Consequences

- `Uart485Transport` (Task 1.2) is a pure byte transport, independently complete against a documented precondition (SDS §6.1) — it does not depend on Task 4.2's implementation status, and Task 4.2 does not need to exist for Task 1.2 to be correct.
- Task 4.2 gains a documented obligation: `rs485Enable()` must fire before any FIFO-capture byte is transmitted, and `rs485Disable()` must fire on **every** bus-release path (normal completion, `Abort()`, circuit-breaker trip) — not only the happy path.
- Phase 4's on-target integration test (Task 4.4) must verify the precondition is actually satisfied at runtime — a FIFO capture's first transmitted byte must be confirmed to have reached the bus — since neither Task 1.2 nor this note can verify it by inspection alone.
- `fifo_transport_uart485.cpp` (Task 1.2's implementation) still contains the superseded model (a duplicated `RS485_EN_PIN` constant and a `digitalWrite()` call inside `write()`) and has not yet been revised to match this decision. That revision is tracked as pending work in the Implementation Plan; **no firmware change is made by this note.**

## 6. Relationship to D-15

| | D-15 (§13.1, unchanged) | ADR-1 (this note) |
|---|---|---|
| Scope | Sole ownership of the RS485 *bus* and sole caller of `modbus.*` | Whether that ownership is read as including the EN *pin* specifically |
| Status | Frozen at Architecture Freeze Review v1.0; not reopened | Accepted; may be revisited independently of D-15 if EN-pin behavior ever changes |
| Text | Unmodified since freeze | New document; does not edit D-15's wording anywhere |

## 7. References

- `CM-100_FIFO_DRIVER_SDS_v1.0.md` §6.1 (transport precondition), §6.2 item 3, §13.1 (D-15, unchanged), §13.2.1, §24 (Decision Log + Related ADRs)
- `CM-100_FIFO_Implementation_Plan_v1.0.md` Task 1.2, Task 4.2
- `CM-100_FIFO_Architecture_Change_Report_v1.0.md` (prior turn's change record, produced before this note existed as a separate artifact)
- Verified production `rs485Enable()`/`rs485Disable()` call sites: `WTVB02_ESP32S3_V16_CM_fault_latch_v3_hardened_patched_v16_5.ino` lines `:4334/:4425`, `:4571/:4584`, `:4680`

---

*End of ADR-1 — Architecture Decision Note only; no implementation code is specified or implied.*
