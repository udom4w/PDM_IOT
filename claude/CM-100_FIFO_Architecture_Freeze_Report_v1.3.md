> **Document Status**
>
> **Type:** Architecture Freeze Report — declares the design baseline frozen. Not a design document itself; records the outcome of freezing one.
> **Architecture version frozen:** v1.3
> **Freeze date:** 2026-07-28
> **Freeze basis:** CM-100_FIFO_DRIVER_SDS v1.3 + CM-100_FIFO_Implementation_Plan v1.3 + ADR-1, following the final architecture consistency audit (PASS) and the two cosmetic cleanups applied afterward (version-tag refresh, Architecture Change Report wording fix). No architecture, interface, or design content changed since that audit.

# CM-100 FIFO Waveform Driver — Architecture Freeze Report v1.3

## 1. Declaration

**The design baseline for the CM-100 FIFO Waveform Driver is now frozen at v1.3.**

This freeze covers the driver's public API, layering, state machine, transport contract (including the RS485 bus/EN-pin ownership model), buffer-ownership model, thread-safety model, error taxonomy, and telemetry contract, as specified in the documents listed in §2. No further architecture, interface, or design changes are authorized without a new revision cycle (a new SDS/Plan version and, where applicable, a new ADR).

## 2. Documents included in this freeze

| Document | Version | Role |
|---|---|---|
| `CM-100_FIFO_DRIVER_SDS_v1.0.md` | 1.3 | Software Design Specification — the design authority |
| `CM-100_FIFO_Implementation_Plan_v1.0.md` | 1.3 | Implementation Plan — task breakdown against the frozen design |
| `CM-100_ADR-1_Bus_Ownership_Includes_EN_Ownership_v1.0.md` | 1.0, Accepted | Architecture Decision Note — EN-pin ownership interpretation of D-15 |
| `CM-100_FIFO_Architecture_Freeze_Review_v1.0.md` | 1.0 | Prior freeze review — resolved the original 7 Must-Fix findings (v1.0→v1.1) |
| `CM-100_FIFO_Architecture_Change_Report_v1.0.md` | 1.0 | Companion record of the v1.1→v1.2 EN-ownership architecture update |
| `architecture_update_v1.3.patch` | — | Verified unified diff of the v1.2→v1.3 document changes (supporting evidence, not a design artifact) |

Filenames are unchanged from their original v1.0 names by convention; the version numbers above are the documents' internal `Version:` header fields, not their filenames.

## 3. Summary of architectural decisions (D-1..D-20)

Full rationale for each lives in SDS §24; summarized here for freeze-record purposes. No decision below was altered by v1.2 or v1.3 — only D-15's interpretation for one specific resource (the EN pin) was clarified, and that clarification is recorded separately as ADR-1, not as an edit to D-15 (§4).

| ID | Decision |
|---|---|
| D-1 | Passive, non-blocking, tick-driven state machine serviced by `taskModbusRead()`. Owns no task, thread, or bus. |
| D-2 | Zero heap anywhere in the driver. |
| D-3 | The driver never writes a sensor register. |
| D-4 | No I/O of any kind inside the receive path. |
| D-5 | Every timing constant is named, documented, and traceable to a measurement. |
| D-6 | All state reachable only through the driver's own API — no externally reachable global. |
| D-7 | Injected `FifoTransport` interface — the driver never references `SerialRS485`/`Serial`/`modbus` directly. |
| D-8 | `status` is the sole authority on sample-data validity. |
| D-9 | Samples stored as raw `int16` counts; no scaling applied inside the driver. |
| D-10 | One statically allocated arena, exclusive ownership handoff, zero copies. |
| D-11 | Streaming CRC + streaming decode; no 6144B staging buffer. |
| D-12 | All driver memory statically allocated; heap use is permanently zero. |
| D-13 | `SerialRS485.setRxBufferSize(2048)` is a hard prerequisite (Task 0.1). |
| D-14 | Protocol model (TRUEPOLL vs. PURELISTEN) is one named compile-time constant — isolates open question G-1. |
| D-15 | `taskModbusRead()` remains the sole owner of the RS485 bus and sole caller of `modbus.*`. The FIFO driver is a mode of that task, not a peer. No new mutex. **Text unchanged since Architecture Freeze Review v1.0.** |
| D-16 | Single-writer by construction; cross-core interaction confined to one queue and one narrow mutex. |
| D-17 | Every timeout is a named, non-adaptive, non-overridable constant. |
| D-18 | Errors classified by domain and policy — a 14-code taxonomy, not one overloaded value. |
| D-19 | Telemetry is purely additive — no MQTT payload field is ever removed. |
| D-20 | Waveform egress (Phase 7) is deferred; FAT storage recommended when undertaken. |

**Bus/EN-pin ownership, as frozen:** `taskModbusRead()`'s `OwnsBus()` boundary (Task 4.2) owns RS485 EN-pin assertion/deassertion, matching every other bus-ownership window already in production. `Uart485Transport` (Task 1.2) is a pure byte transport with a documented precondition (SDS §6.1) — it neither asserts, deasserts, nor references the EN pin. This is the ADR-1 interpretation of D-15, not a change to D-15 itself.

## 4. ADR list

| ID | Title | Status | Interprets | Summary |
|---|---|---|---|---|
| ADR-1 | Bus ownership includes EN ownership | Accepted | D-15 (unchanged) | `taskModbusRead()`'s bus ownership (D-15) is interpreted to include EN-pin assertion/deassertion, matching all three existing production bus-ownership windows. `Uart485Transport` is a pure byte transport against a documented precondition; EN bracketing belongs to Task 4.2. |

No other ADRs exist as of this freeze. Any future reinterpretation of a frozen decision must follow the same pattern — a new, separately numbered ADR, never an edit to the original decision's text.

## 5. Outstanding implementation work

Nothing below requires an architecture change to complete — all of it is execution against the now-frozen v1.3 baseline.

| Item | Status | Notes |
|---|---|---|
| Task 0.1 — RX buffer sizing | Complete, uncommitted | `SerialRS485.setRxBufferSize(2048)` inserted in `setup()`. |
| Task 1.1 — `FifoTransport` interface | Complete, uncommitted | `fifo_transport.h`, zero Arduino dependency. |
| Task 1.2 — `Uart485Transport` implementation | **Revised this session** to match v1.3 (EN-pin ownership removed from `fifo_transport_uart485.cpp`); still uncommitted | Next action: build verification, then commit, per the standard per-task workflow. |
| Task 1.3 — `LogReplayTransport` test fixture | Not started | Explicitly deferred pending approval throughout the review cycle. |
| Task 2.x — L2 Frame Codec (CRC, decoder, frame codec, fixtures) | Not started | No dependency on the EN-ownership question. |
| Task 3.x — L3 Capture Session (arena, state machine, public API) | Not started | No dependency on the EN-ownership question. |
| Task 4.1 — Wire `FifoDriver_Init()` into `setup()` | Not started | |
| Task 4.2 — Mode branch in `taskModbusRead()` (bus + EN ownership) | Not started | Highest-risk task in the plan; implements the frozen §13.2/§13.2.1/ADR-1 model. Must satisfy Task 1.2's documented precondition and fire `rs485Disable()` on every release path. |
| Task 4.3 — `g_modbusConsecErrors` non-increment guard | Not started | |
| Task 4.4 — On-target integration verification | Not started | Must confirm Task 1.2's precondition is actually met at runtime (first transmitted byte reaches the bus). |
| Phases 5–8 (telemetry, trigger wiring, optional waveform egress, soak validation) | Not started | |
| G-1 — Protocol model (TRUEPOLL vs. PURELISTEN) | Open, isolated to `FIFO_PROTOCOL_MODEL` (D-14) | Blocks only Phase 8 soak validation; Phases 1–7 are protocol-model-agnostic by design. |
| G-2 — No inbound MQTT channel (`REMOTE_ON_DEMAND`) | Open, out of scope for this plan | Blocks only the `REMOTE_ON_DEMAND` trigger source; all other trigger sources are unaffected. |
| Cosmetic — `fifo_transport_uart485.h` top-of-file comment | Not yet corrected | Still states EN-pin ownership "is contained here and nowhere else," which is now inaccurate after this session's Task 1.2 revision. Flagged, not fixed, since it was outside this session's stated file scope (`fifo_transport_uart485.cpp` only). |

## 6. Baseline statement

> **The CM-100 FIFO Waveform Driver design baseline is now frozen at v1.3.** All architectural decisions (D-1..D-20) and the one Architecture Decision Note (ADR-1) are settled. Implementation may proceed against this baseline. Any change to a public API, ownership rule, state-machine shape, or frozen decision text requires a new revision cycle — it may not be made silently inside an implementation task.

---

*End of CM-100_FIFO_Architecture_Freeze_Report_v1.3 — freeze record only; no new design content.*
