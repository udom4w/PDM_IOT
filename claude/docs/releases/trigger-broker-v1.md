# Release Note — `trigger-broker-v1`

**Tag:** `trigger-broker-v1` (annotated) → commit `80361bc`
**Repository:** `https://github.com/udom4w/PDM_IOT.git`
**Branch:** `recover-experimental`

> **This is an architecture milestone, not a production firmware release.** It marks the point at which the FIFO Trigger Broker pattern — the mechanism by which anything outside `taskModbusRead()` can request a FIFO waveform capture — is implemented, verified end-to-end on real hardware, and pushed to GitHub for the first time. It is not a sign-off to deploy this firmware to production units; see [Known remaining work](#6-known-remaining-work) below.

---

## 1. Overview

Prior to this milestone, the FIFO capture driver (`fifo_driver.cpp` et al.) existed as a frozen, hardware-qualified receive path (see ADR-0005) with exactly one caller: a temporary, one-shot commissioning trigger inline in `taskModbusRead()`. That caller had, at one point, been deleted during release-prep cleanup, leaving the entire qualified receive path unreachable by anything.

This milestone replaces that single ad hoc caller with a proper **Trigger Broker**: a depth-1 queue (`queueFifoTrigger`) that any task, on any core, can submit a capture request to, with `taskModbusRead()` remaining the sole task that ever actually calls into the driver. Two real production trigger sources — `FAULT_LATCH` and `OPERATOR_BUTTON` (ENTER double-click) — were built on top of this broker and verified on hardware, alongside the restored commissioning trigger and a new result consumer that publishes capture metadata to MQTT.

## 2. Scope of this milestone

In scope and complete:
- The Trigger Broker itself (queue, intent type, drain logic)
- Restoring and rewiring the commissioning one-shot trigger through the broker
- The result consumer (`handleFifoCaptureCompletion()`) and its MQTT `/event` metadata publish
- Two production trigger sources: `FAULT_LATCH` and `OPERATOR_BUTTON`
- Hardware verification of all of the above, including duplicate-trigger prevention, `ACTIVE`/`COOLDOWN` rejection behavior, and regression checks against pre-existing behavior (Alarm ACK, SELECT, UART/H4 receive path)
- A source-verified architecture-invariants document and reinforcing code comments

Explicitly out of scope (see §6):
- `SCHEDULED` and `CONFIG_MODE` trigger sources
- `MQTT`/Scheduler/HMI/AI-originated triggers
- Waveform egress (the driver exposes only metadata to MQTT; the 6144-byte sample payload itself is never transmitted, per the frozen driver's own deferred-scope decision)
- Any change to the frozen driver files themselves

## 3. Architecture completed

### Trigger Broker
A single depth-1 FreeRTOS queue, `queueFifoTrigger`, carrying `FifoTriggerIntent_t` (source, tag, `requirePermissive`). Non-blocking `xQueueSend(...,0)` on the producer side is itself the duplicate-trigger guard — if a trigger is already pending, a new one is dropped and logged, never queued twice. Drained by exactly one consumer, once per `taskModbusRead()` tick.

### Single `Request()` caller
`FifoDriver_Request()` has exactly one call site in the entire firmware: the broker's drain block inside `taskModbusRead()`. Verified by a repository-wide search (every `.ino`/`.cpp`/`.h`/`.c` file, not just the target sketch's directory) — no other file in the project even includes `fifo_driver.h`.

### Single Result Consumer
`handleFifoCaptureCompletion()` is the sole caller of `FifoDriver_TryAcquireResult()` and `FifoDriver_ReleaseResult()`, called immediately after `FifoDriver_Service()` on every tick. It builds a metadata-only JSON document (capture ID, trigger source, timing, sample count, error code — never the raw waveform) and publishes it to MQTT `/event`, then unconditionally releases the result on every code path.

### Producer enqueue-only rule
All three producers — the commissioning one-shot (`taskModbusRead()`), `FAULT_LATCH` (`checkAndLatchFault()`), and `OPERATOR_BUTTON` (`taskButtonHandler()`, ENTER double-click) — only ever call `xQueueSend(queueFifoTrigger, ...)`. None constructs a `FifoCaptureRequest` or calls the driver directly; none inspects driver state (`FifoDriver_GetPhase()`/`OwnsBus()`) to decide whether to enqueue.

### Driver-owned admission
Every admission decision — idle/busy, cooldown-elapsed, circuit-breaker state, result-held, application-state gates — is decided entirely inside `FifoDriver_Request()` in the frozen, unmodified `fifo_driver.cpp`. No broker or producer code duplicates, pre-empts, or second-guesses that decision; a rejection is logged and dropped, never retried.

### Hardware validation summary
Verified on-target (COM5, live serial capture) across multiple sessions:
- **Commissioning trigger:** fires automatically post-boot once motor/MQTT/sensor gates pass; capture completes in ~7.5–7.8 s consistently, matching the ADR-0005 qualification baseline.
- **`FAULT_LATCH`:** real vibration events (WARNING/HEALTH_LOW/CRITICAL) correctly enqueue, admit, capture, and publish; 16 of 19 observed events in one stress session were correctly rejected (`ERR_BUSY`, cooldown), zero retries.
- **`OPERATOR_BUTTON`:** ENTER double-click (400 ms release-to-press window) verified via an interactive, live-monitored session — single click, slow double-click, 2 s Alarm ACK, fast double-click, and cooldown-rejection scenarios all confirmed correct.
- **Regressions:** zero — Alarm ACK fires identically to pre-milestone behavior; zero `UART_FIFO_OVF`/`UART_BUFFER_FULL` during any fully-observed capture; zero `ERR_RESULT_NOT_RELEASED`; zero queue corruption; stable heap across repeated captures.
- **Open, non-blocking UX observation:** a rapid triple-click currently produces two independent enqueue/`Request()` attempts (pairing click 1+2 and click 2+3), not one — documented, not treated as a defect, deferred as a future gesture-policy decision.

## 4. Commits included

| Commit | Summary |
|---|---|
| `f038b6d` | `feat: add OPERATOR_BUTTON trigger through Trigger Broker` — the Trigger Broker, the restored/rewired commissioning trigger, the result consumer, the `FAULT_LATCH` trigger, and the `OPERATOR_BUTTON` trigger, all landing together as the first commit of this effort |
| `80361bc` | `docs: harden Trigger Broker architecture invariants` — `docs/FIFO_TRIGGER_BROKER_INVARIANTS.md` plus short `[ARCH-INVARIANT]` comment tags at the six enforcement points; documentation-only, binary confirmed byte-for-byte identical before and after |

## 5. Tag

```
trigger-broker-v1  (annotated)
  commit: 80361bc
  message: "Trigger Broker architecture validated and first GitHub baseline"
```

## 6. Known remaining work

- **`fifo_driver.cpp` documentation review.** An unstaged, comment-only diff exists on top of the last committed version of this file (a version-tag bump and a rationale note for the already-active `PURELISTEN` protocol mode). It changes no logic and produces no binary difference, but it cites ADR-0005 by name while ADR-0005 itself is not yet committed — needs sequencing before either is pushed.
- **ADR-0005.** Currently untracked (`claude/docs/adr/`), documenting the H4 receive-cadence fix this whole trigger effort depends on. Needs its own commit.
- **Experimental artifacts.** `experimental/WTVB05_ValidationTool_FIFO/*` has unrelated, pre-existing unstaged modifications (predating this milestone's work) that still need triage.
- **Future producers.** `SCHEDULED`, `CONFIG_MODE`, and any MQTT-, Scheduler-, HMI-, or AI-originated trigger sources are explicitly not yet implemented. Before adding one, re-run the verification searches in `docs/FIFO_TRIGGER_BROKER_INVARIANTS.md` and confirm the new producer follows the same enqueue-only pattern as the three existing ones.

---

*Generated as part of the architecture-milestone documentation pass. No firmware, driver, or source files were modified in producing this document.*
