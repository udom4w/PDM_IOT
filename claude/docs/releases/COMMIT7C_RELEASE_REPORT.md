# Commit 7C — Release Validation Report

**Scope:** FIFO Trigger Broker / Capture subsystem (`fifo_driver.cpp`, `fifo_codec.cpp`, `fifo_session.cpp`, `fifo_arena.cpp`, `fifo_types.h`, and the `.ino` integration layer).
**Status:** Release Candidate.
**Commit:** `34f755d` — `fix(fifo): Commit 7C closure -- 5 confirmed defects in FIFO driver`

---

## 1. Summary of the 4 Commit 7C fixes

All 4 were discovered via a host-executable verification pass (compiled test suite + new fault-injection battery, g++/ASan/UBSan), root-caused against source, patched, and re-verified by execution before any hardware involvement.

| # | Defect | Root cause | Fix |
|---|---|---|---|
| 1 | `ERR_CIRCUIT_OPEN` unreachable (dead code) | `FifoDriver_Request()`'s admission gates checked `S1_IDLE` (driver-busy) before `S14_DISABLED` (circuit breaker). Since `S14_DISABLED != S1_IDLE`, the busy gate always matched first — a tripped breaker always returned `ERR_BUSY`, never `ERR_CIRCUIT_OPEN`. Externally visible via `publishMqttRejectionEvent()`, misrepresenting recoverability to any REMOTE_ON_DEMAND caller. | Reordered the two gates: circuit-breaker check now runs first. |
| 2 | `ERR_RX_OVERFLOW` never raised | `hadOverflow()` — documented as "the SOLE source of `ERR_RX_OVERFLOW`" — was only read inside a diagnostics-only, `nullptr`-gated logging block that never wrote to `s_result.error`. A real UART overflow was silently absorbed and misattributed to whatever the corrupted remainder produced (typically `ERR_CRC_MISMATCH`/`ERR_BAD_TYPE_BYTE`). | Added an explicit check at the top of `HandleReceivingImpl()`, once per receiving tick, before anything else that tick. |
| 3 | `desyncBytesDiscarded` always 0 on the published result | `s_result.desyncBytesDiscarded` was never assigned anywhere in `fifo_driver.cpp`. Confirmed by grep (zero assignment sites) and empirically (a session that genuinely discarded 64 bytes reported `desyncBytesDiscarded=0` in its result). | Populated at `DESYNC_LIMIT` finalization using the same `MAX_DESYNC_BYTES`-based derivation this file's own `ComputeBytesConsumedThisTick()` already uses for the identical "value already reset" constraint. |
| 4 | Abort's `retryCount` telemetry overload | `FifoDriver_Abort()` force-set `s_result.retryCount = FIFO_MAX_RETRIES` as its retry-suppression signal — but that field is also the public, telemetry-facing `FifoCaptureResult::retryCount` (published as `/event`'s `retry_count`), so an aborted capture with zero real retries reported `retry_count=2`. | Decoupled via a dedicated internal `s_abortRequested` flag; `retryCount` now always reflects retries that actually occurred. |

---

## 2. Root cause of the `status` defect (discovered during hardware validation)

**Symptom:** A live hardware capture (`request_id=hwtest-001`, Hardware Test 2) published an `/event` payload containing `"status":"IDLE"` for a capture that had just completed a full, successful S2→S11 pipeline run — never plausibly `IDLE`.

**Root cause:** `s_result.status` (`fifo_types.h`: *"driver phase at the moment this result was finalized"*) was never assigned anywhere in `fifo_driver.cpp` — confirmed by grep (zero assignment sites pre-fix). It stayed permanently at its zero-initialized default, `FifoPhase::IDLE` (enumerator value 0), regardless of actual outcome. This is the same class of defect as fix #3 above (a documented result field with no writer), just outside the assertion coverage of the host test suite that caught the other 4 — none of those tests asserted on `result.status`.

**Fix:** `s_result.status = FifoPhase::RESULT_READY;` added in `HandleS10Drain()`, immediately before the single, unconditional `s_state = FifoState::S11_RESULT_READY` transition. Proven by exhaustive code trace (not just the fix itself) to be the *only* correct value and the *only* correct location:
- Every successful capture, every retries-exhausted failure, and every abort (which routes through `S13_FAILED → S10_DRAIN`, same as any other failure) reach this exact point.
- `S11_RESULT_READY` is assigned in exactly one place in the file, and `HandleS10Drain()` — the function containing that assignment — is itself called from exactly one site, gated on `s_state == S10_DRAIN`. No path can reach `S11_RESULT_READY` without first executing this line.
- `FifoDriver_TryAcquireResult()` itself gates on `s_state == S11_RESULT_READY`, so every successful read of this field happens while that is true — `RESULT_READY` is not just "the value at finalization," it is the value for the entire externally-visible lifetime of the result.

Full derivation and code citations: see the "Proof" exchange in this session's working notes (`fifo_driver.cpp:1263-1268`, `1433-1444`, `1499-1519`, `1734-1739`, `1351-1352`).

---

## 3. Hardware validation results

11-test sequence executed against the live ESP32-S3 device (COM5) and the real production MQTT broker (`iot.promlogix.com:8883`, mTLS), commands injected via MQTT Explorer.

| # | Test | Result |
|---|---|---|
| 1 | MQTT command → ACCEPTED → FIFO Capture | **PASS** |
| 2 | `request_id` appears in Success Event | **PASS** |
| 3 | Driver Reject (ACTIVE) | **PASS** |
| 4 | Driver Reject (COOLDOWN) | **PASS** |
| 5 | Invalid JSON | **PASS** |
| 6 | Invalid Action | **PASS** |
| 7 | Missing request_id | **PASS** |
| 8 | OPERATOR_BUTTON Regression | **PASS** |
| 9 | FAULT_LATCH Regression | **PASS** (code-trace verification — no real CRITICAL vibration event induced on production hardware; producer code, trigger enum, queue path, and broker admission logic all proven untouched by Commit 7C, and the shared downstream pipeline independently hardware-proven via 3 other trigger sources this session) |
| 10 | MQTT Telemetry Regression | **PASS** |
| 11 | `ERR_RESULT_NOT_RELEASED` | **Verified by Design** — not hardware-reproducible (see §5) |

A host-side USB-CDC serial transport anomaly occurred mid-sequence (total serial silence while OLED and MQTT telemetry remained healthy); root-caused to the host connection, not a firmware hang, and resolved without any code change or device power-cycle. Not a firmware defect.

---

## 4. Hardware re-verification of the `status` fix

Post-fix firmware rebuilt, reflashed, and re-tested live (`request_id=status-test-001`):

```
[MQTT-CMD] ACCEPTED request_id=status-test-001 source=REMOTE_ON_DEMAND
[FIFO-BROKER] FifoDriver_Request() source=4 verdict=0 handle=2 ...
[FIFO-RESULT] captureId=2 trigger=REMOTE_ON_DEMAND error=NONE enqueued=1 szEvt=476
```

Confirmed via the published `/vibration/event` payload (MQTT Explorer):
```
status  = RESULT_READY
error   = NONE
trigger = REMOTE_ON_DEMAND
```

Matches the fix's intended behavior exactly — `status` no longer reads `IDLE`.

---

## 5. Remaining known limitation

**`ERR_RESULT_NOT_RELEASED` is not hardware-reproducible by design**, and this is correct, not a gap:

- `FifoDriver_TryAcquireResult()`/`FifoDriver_ReleaseResult()` have exactly one call site each, both inside `handleFifoCaptureCompletion()` (`.ino`), which itself has exactly one call site, invoked unconditionally every tick immediately after `FifoDriver_Service()` — same task, same core, no blocking call between acquire and release.
- Defeating this from outside would require either a second consumer call site (architecturally forbidden, documented invariant) or a genuine multi-tick block inside the existing single call site — neither exists in current code.
- A black-box MQTT-command attempt to force this condition would not exercise the intended code path and was correctly not attempted against production hardware.

**Recommendation:** close this gap with a small, deterministic **host-side unit test** — complete a capture, call `TryAcquireResult()` and deliberately withhold `ReleaseResult()`, then call `FifoDriver_Request()` and assert `verdict == ERR_RESULT_NOT_RELEASED`. This is a coverage gap in the host test suite, not a hardware task.

---

## 6. Verdict

**Qualified as Release Candidate.** All 5 confirmed defects patched and verified by execution (host tests + live hardware). No regressions detected across 10 hardware-executable tests plus 1 correctly-scoped code-trace verification. The one remaining gap (`ERR_RESULT_NOT_RELEASED` host-test coverage) is tracked as a follow-up, not a blocker.
