# FIFO Trigger Broker — Architecture Invariants

**Scope:** the production-trigger integration added on top of the frozen FIFO driver (`fifo_driver.cpp`/`fifo_codec.*`/`fifo_session.*`/`fifo_arena.*`/`fifo_transport_uart485.*`, none of which this document — or the comments it points to — ever modifies).

**Verification basis:** every claim below is a direct source citation (file:line), confirmed by a project-wide search across every `.ino`/`.cpp`/`.h`/`.c` file in the repository — not just the target sketch's own directory — as of the review that produced this document (`recover-experimental`, commit `f038b6d`).

**Related:** [ARCHITECTURE.md](ARCHITECTURE.md) · `CM-100_FIFO_DRIVER_SDS_v1.0.md` (§7, §9, §14, §19) · `CM-100_FIFO_Implementation_Plan_v1.0.md`

---

## The six invariants

1. **Every producer must enqueue a `FifoTriggerIntent`.** No producer constructs a `FifoCaptureRequest` or touches the driver directly.
2. **Only the Trigger Broker may call `FifoDriver_Request()`.** Exactly one call site in the whole firmware.
3. **Only the Result Consumer may acquire/release results.** Exactly one call site each for `FifoDriver_TryAcquireResult()` and `FifoDriver_ReleaseResult()`.
4. **Producers must never inspect Driver state.** No producer reads `FifoDriver_GetPhase()`, `FifoDriver_OwnsBus()`, or any other driver accessor to decide whether to enqueue.
5. **Driver owns admission decisions.** Whether a request is admitted (`S1_IDLE`, cooldown elapsed, breaker closed, no result held, application-state gates) is decided exclusively inside `FifoDriver_Request()` (`fifo_driver.cpp`, unmodified) — never pre-empted by broker or producer logic.
6. **ACTIVE/COOLDOWN policy belongs only to the Driver.** No `.ino` code branches on "is a capture currently running" to suppress or alter a producer's or the broker's own behavior; a rejected admission (`ERR_BUSY`) is simply logged and dropped, never retried.

## Verified compliance, per invariant

### 1 — Producers enqueue, never construct a driver request directly

Three producers exist, all in `WTVB02_ESP32S3_V16_CM_fault_latch_v3_hardened_patched_v16_5.ino`, all ending in `xQueueSend(queueFifoTrigger, ...)`:

| Producer | Function | Enqueue site |
|---|---|---|
| Commissioning (one-shot) | `taskModbusRead()` | `.ino:4799` |
| `FAULT_LATCH` | `checkAndLatchFault()` | `.ino:3031` |
| `OPERATOR_BUTTON` (double-click) | `taskButtonHandler()` | `.ino:6577` |

None of these three functions contains a call to `FifoDriver_Request(` — confirmed by repository-wide search; the only occurrences of that literal call (as opposed to a comment or declaration) are the driver's own definition (`fifo_driver.cpp:1497`), its header declaration (`fifo_driver.h:116`), the host-only test suite (`test/test_fifo_driver.cpp`, never compiled into firmware), and the one broker call site below.

### 2 — Single `FifoDriver_Request()` call site

```
.ino:4714   FifoError fifoVerdict = FifoDriver_Request(&fifoReq, &fifoHandle);
```

Inside the drain block of `taskModbusRead()` (`.ino:4696-4717`), immediately before `FifoDriver_Service()`. This is the **only** production call site in the entire repository — verified by grepping every `.ino`/`.cpp`/`.h`/`.c` file under the repo root, not just this sketch's own directory. No other `.ino` sketch anywhere in the repository (including every legacy/backup/experimental variant) even `#include`s `fifo_driver.h`, so none of them has the means to bypass this.

### 3 — Single Result Consumer

```
.ino:4579   if (!FifoDriver_TryAcquireResult(&result)) { ... }
.ino:4628   FifoDriver_ReleaseResult();
```

Both calls live inside `handleFifoCaptureCompletion()` (`.ino:4557-4629`) and nowhere else in production code. The function is called from exactly one site, immediately after `FifoDriver_Service()` in `taskModbusRead()`.

### 4 — Producers never inspect driver state

None of the three producer functions calls `FifoDriver_GetPhase()`, `FifoDriver_OwnsBus()`, `FifoDriver_GetStats()`, or any other driver accessor. Each producer's only interaction with the FIFO subsystem is a single, unconditional `xQueueSend(...)` — the decision of whether that intent is ever acted on is made entirely downstream, by the broker's drain (invariant 2) and the driver's own gates (invariant 5).

### 5 — Driver owns admission

The drain block (`.ino:4696-4717`) populates `FifoCaptureRequest.admissionContext` from live `.ino` globals (`g_motorRunState`, `g_modbusConsecErrors`, `mqttClient.connected()`) immediately before calling `FifoDriver_Request()`, but the admission verdict itself — `S1_IDLE` check, circuit-breaker check, cooldown-elapsed check, result-held check, and the three application-state gates — is computed entirely inside `FifoDriver_Request()` (`fifo_driver.cpp:1497-1579`, unmodified by any producer/broker work). The `.ino` never duplicates or second-guesses this decision.

### 6 — ACTIVE/COOLDOWN policy stays in the Driver

No producer or broker code branches on capture-in-progress state. A rejected `FifoDriver_Request()` call (`verdict != FifoError::NONE`, most commonly `ERR_BUSY`) is logged via the existing `[FIFO-BROKER]`/`[LATCH]`/`[CORE 1] ENTER:` diagnostic lines and the corresponding `FifoTriggerIntent` is simply discarded — confirmed on hardware across dozens of `ERR_BUSY` instances (FAULT_LATCH and OPERATOR_BUTTON hardware verification sessions) with zero retries observed in any case.

## What would violate these invariants

- A new producer that calls `FifoDriver_Request()` directly instead of enqueuing.
- A second call site of `FifoDriver_TryAcquireResult()`/`FifoDriver_ReleaseResult()` (a second result consumer).
- Any producer reading `FifoDriver_GetPhase()`/`FifoDriver_OwnsBus()` to decide whether to enqueue, or retrying after an `ERR_BUSY`/`ERR_*` rejection.
- Widening `queueFifoTrigger`'s depth beyond 1, or adding a second queue for FIFO requests.
- Any `.ino`-side code that mirrors or overrides an admission gate already owned by `fifo_driver.cpp`.

Before adding a new trigger source (MQTT, Scheduler, HMI, AI, etc.), re-run the verification searches in this document's "Verified compliance" section and confirm the new producer follows the same enqueue-only pattern as the three existing ones.
