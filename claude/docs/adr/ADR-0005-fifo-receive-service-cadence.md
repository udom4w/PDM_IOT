# ADR-0005: FIFO Receive Service Cadence

**Status:** Accepted
**Date:** 2026-07-31
**Supersedes:** none
**Related:** ADR-0001 (bus/EN ownership), ADR-0004 (L2/L3 layering), DESIGN-0005 (invariant recorded in `taskModbusRead()`)

## Context

The WTVB05 RAWFIFO capture (`50 03 00 2C 00 01`) never completed successfully in
production firmware. Every session failed in one of two ways:

- `partialBytesReceived` stalled at exactly **5501 / 6146** bytes, terminating
  with `ERR_INTER_BYTE_TIMEOUT`; or
- the byte count reached the full 6144-byte payload but the frame failed CRC
  (`CRC_MISMATCH`, then `ERR_DESYNC_LIMIT`).

The `5501` stall reproduced identically across four independent boots, which
ruled out random jitter early.

### What was eliminated

An independent, standalone ESP32 RS485 bus sniffer captured the same sessions
from the wire. It confirmed the sensor transmitted a **complete, CRC-valid
6149-byte frame every time** (`Bytes Lost Because Buffer Full: 0`), with
progress frames normal and no mid-payload transmission gap. The sensor and the
bus were therefore excluded as causes; the loss was on the ESP32 receive side.

Four hypotheses were then falsified experimentally on hardware, each as an
isolated single-variable test against the reproducible baseline:

| Hypothesis | Change tested | Result |
|---|---|---|
| Protocol mode | `PURELISTEN` → `TRUEPOLL` (re-request per progress frame) | **Falsified.** Same failure; UART FIFO-overflow events roughly quadrupled |
| Receive-path logging | Disabled the unconditional per-tick `[XPORT-DIAG]` log | **Falsified.** Every metric byte-for-byte identical to baseline |
| NVS flash persistence | Suppressed the periodic `saveRuntimeHour()` commit | **Falsified.** Every metric identical; the correlation that motivated it was not causal |
| Parser / CRC / byte accounting | Exhaustive source review | No defect found; behavior identical across protocol modes |

### Root cause

A source-level audit of the receive path established the actual mechanism:

`FrameCodec_Step()` performs exactly **one bounded sub-action per call** —
`ScanAnchor()` stops the instant the `50 03` anchor is matched and never
consumes past it; `ReadType()` reads at most one byte and never loops; the
progress tail is 4 bytes. `FifoDriver_Service()` invokes it exactly **once per
`taskModbusRead()` iteration**, verified by call-graph trace (no loop exists at
any level between the task loop and `FrameCodec_Step()`, and all three
`vTaskDelayUntil()` exit paths delay before continuing).

At the 250 ms task cadence the driver therefore consumed **0–3 bytes per cycle**
during the anchor/type/progress phases, while the WTVB05 streams its dump
autonomously at 9600 baud = **~240 bytes per 250 ms** — a ~100:1
producer/consumer mismatch.

Captured `availBefore` telemetry shows the consequence exactly:

```
t=76198  availBefore=234   availAfter=233    (1 byte consumed)
t=76448  availBefore=472   availAfter=472    (0 bytes consumed)
...
t=78198  availBefore=2048  availAfter=2045   <- 2048-byte ring buffer SATURATED
```

Fill rate ~236 B/tick → the 2048-byte IDF RX ring buffer saturates in ~8.7
cycles (~2.2 s). Measured: 8 cycles, 2000 ms. `UART_BUFFER_FULL` then fires,
followed by `UART_FIFO_OVF` as the 128-byte UART **hardware** FIFO overflows —
the first point at which bytes are irrecoverably discarded by the peripheral,
before any software layer can read them. The ~645 missing bytes (6146 − 5501)
are those lost bytes; when the loss pattern instead leaves the count intact but
the content shifted, the frame fails CRC.

## Decision

Service the FIFO receive path at **10 ms while `FifoDriver_OwnsBus()` is true**,
and retain **250 ms for normal Modbus polling**.

Implemented as a single gated delay in `taskModbusRead()`:

```c
vTaskDelayUntil(&xLastWakeTime,
                FifoDriver_OwnsBus() ? xFrequencyFifo : xFrequency);
```

The elevated cadence is deliberately scoped to FIFO bus ownership rather than
applied globally. **DESIGN-0005 invariant — normal polling remains at 250 ms**,
for two independent, load-bearing reasons:

1. `STUCK_THRESHOLD` is a **read count**, not a duration; the stuck-axis
   detection it drives is calibrated to this cadence ("5 reads × 250 ms =
   1.25 s"). A faster poll rate would trip it on the sensor's own
   not-yet-updated registers, causing spurious `restartSensorViaModbus()`
   calls, which set `g_modbusConsecErrors != 0` and in turn block FIFO
   admission via the request gate's `sensorHealthy` term.
2. The normal-poll body cannot fit a shorter period: ~9 Modbus transactions at
   9600 baud plus 10× `vTaskDelay(5 ms)` is ~230 ms.

Normal polling is already skipped entirely while FIFO owns the bus, so the loop
body is genuinely short in the elevated-cadence window and neither reason is
violated.

## Consequences

### Positive

- FIFO capture succeeds reliably: **10/10 qualification runs passed**, all on
  the first attempt, 100 % CRC pass, 1024 samples decoded each.
- **Zero** `UART_FIFO_OVF` and **zero** `UART_BUFFER_FULL` events across all
  runs (baseline: 5→10 and 4→8 per session).
- RX ring-buffer occupancy peaks at **32–40 bytes of 2048** (1.9 %), versus
  full saturation at baseline.
- No regressions: 0 spurious sensor restarts, 0 Modbus failures, 0
  crash/watchdog/brownout indicators, with MQTT and NVS persistence both active.

### Negative / accepted trade-offs

- `taskModbusRead()` iterates 25× more frequently during a FIFO session (~7.5 s
  per capture). The loop body is short in this window because normal polling is
  gated off, and no starvation of the co-resident Core-0 `taskStateMachine`
  (priority 4, below this task's 5) was observed in qualification.
- The transition into the elevated cadence can produce a brief catch-up burst,
  because `vTaskDelayUntil()` is an absolute-deadline primitive and the previous
  deadline was set for a 250 ms period. This is bounded and self-correcting.

### Not addressed by this ADR

The underlying **one-bounded-action-per-call codec structure** is unchanged.
The cadence increase compensates for it rather than removing it. Raising the
per-call byte budget in `fifo_codec.cpp` would address the mismatch at its
source but requires modifying the frozen L2 codec, and was deliberately not
attempted here.

## Alternatives considered

- **`TRUEPOLL` protocol mode** — falsified on hardware (see table above). Bus
  captures also showed the sensor streams autonomously after a single request,
  so per-frame re-requests are not required by the protocol.
- **Increase the RX ring buffer beyond 2048 bytes** — would delay saturation but
  not prevent it: at 0–3 B/cycle consumption the buffer fills regardless of
  size, and no size below the 6149-byte frame can hold a full dump.
- **Change the task cadence globally to 10 ms** — rejected; violates DESIGN-0005
  (see Decision).
- **Modify `FrameCodec_Step()` to consume until the buffer drains** — rejected
  for this change as it modifies frozen L2 codec logic and the CRC-accumulation
  byte boundaries that depend on it. Remains the more direct long-term fix.

## Verification

- Source-level call-graph trace confirming exactly one `FrameCodec_Step()` call
  per task cycle.
- Independent RS485 bus sniffer capture correlating wire truth against driver
  telemetry in the same session.
- Four falsification experiments, each isolating a single variable.
- 10-run repeatability qualification with all production subsystems enabled.
- Post-cleanup build verified byte-identical in size to the qualified firmware
  before instrumentation removal.
