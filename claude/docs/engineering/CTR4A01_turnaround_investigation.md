# CTR4A01 Modbus Turnaround Investigation

**Firmware:** `WTVB02_ESP32S3_V16_CM_fault_latch_v3_hardened_patched_v16_5.ino` (v16.6b/c working tree)
**Hardware:** LilyGO T-Vending S3 (ESP32-S3, 16MB flash, 8MB Octal PSRAM) + WTVB02 vibration sensor + CTR4A01 1-channel RS485 current transformer, shared RS485 multi-drop bus
**Branch:** `feature/mqtt-single-owner`
**Investigation dates:** 2026-07-17 (single session, six phases)
**Status:** Root cause identified and validated by controlled experiment. Fix implemented, flashed, and measured. Not yet merged/committed to source control.

Legend used throughout this document:
- **FACT** — directly verifiable from source code, hardware identification (`esptool`), or library source. Not time-dependent.
- **OBSERVATION** — empirical data recorded from a live serial capture or build/flash run on this specific session's hardware.
- **INFERENCE** — a conclusion connecting facts and observations. Explicitly reasoned, not directly measured.

---

## 1. Original Symptom

**OBSERVATION.** On the production firmware, the CTR4A01 current-sensor read (Transaction 7, "T7") inside `taskModbusRead()` was suspected of failing on every cycle. Prior to instrumentation, no direct Serial evidence of T7 existed — the firmware had zero logging for this transaction's success/failure state (see §3, Phase 0/1).

**FACT.** The firmware's own source comment (present before this investigation began) anticipated this exact failure mode:

```cpp
// [v16.6b] Remote diagnostics for current_slope=0 ambiguity -- if current_buf_count
// stays 0 while current_read_errors keeps climbing, CTR4A01 Modbus reads are failing
// (check slave address/wiring/baud); if both stay 0, the 500ms cadence itself never fired.
```
(`WTVB02_ESP32S3_V16_CM_fault_latch_v3_hardened_patched_v16_5.ino:6195-6197`)

---

## 2. Initial Hypotheses

Considered, in the order investigated:

1. **CTR4A01 hardware/wiring/power fault** — ruled out (§7).
2. **Wrong slave ID, function code, register address, or baud rate** — ruled out (§7).
3. **Missing Modbus RTU inter-frame silent interval** between the last WTVB02 transaction (T6, slave `0x50`) and the first CTR4A01 transaction (T7, slave `0x01`) — the hypothesis ultimately supported.

**FACT.** T1–T6 each address slave `0x50` (WTVB02) via function `0x03` (Read Holding Registers) and are each followed by `vTaskDelay(pdMS_TO_TICKS(5))` (lines 3561, 3568, 3575, 3583, 3595, 3603, 3610, 3617). T7 addresses a *different* slave (`0x01`, CTR4A01) via a *different* function (`0x04`, Read Input Registers) and, prior to the fix, had **no delay** between the end of T6 and the start of T7.

**FACT.** Modbus RTU requires ≥3.5 character-times of bus silence between frames. At 9600 baud, 8N1: one character = 11 bits ⇒ 3.5 characters ≈ 4.01ms. The existing 5ms delays used elsewhere exceed this minimum; the pre-fix 0ms gap at the T6→T7 boundary did not.

---

## 3. Experiments Performed

| # | Experiment | Code change | Result |
|---|---|---|---|
| 1 | Read-only source trace of T1–T7 (transaction map, register addresses, function codes, return-code handling) | None | Established full transaction map; identified the T6→T7 delay gap and confirmed zero Serial logging existed for T7 |
| 2 | Read-only comparison against standalone `experimental/CTR4A01_SENSOR/CTR4A01_SENSOR.ino` | None | Standalone sketch never switches slave ID during its read loop and always has ≥500ms of bus idle time before each request; production firmware switches slave ID twice per cycle with 0ms settle time at the critical boundary |
| 3 | Added diagnostic-only Serial logging (`[CURRENT] raw=... mA` / `[CURRENT] FAIL rc=... elapsed=... ms`) inside `readCurrentSensor()` | Added `modbusRcName()` helper + 2 `Serial.printf` calls + local timing vars. No Modbus logic, timing, or task-scheduling changes (diff reviewed and approved before build) | Enabled direct observation of T7's return code and elapsed time for the first time |
| 4 | Baseline capture on physical hardware (flashed, 150s live serial capture) | None (diagnostic build only) | 66/66 CURRENT reads failed — see §4 |
| 5 | Controlled experiment: inserted exactly one `vTaskDelay(pdMS_TO_TICKS(5));` immediately before the sole call site of `readCTR4A01Current()` (line ~3664) | Single line + 2 comment lines added; nothing else touched (diff reviewed before build) | 292/292 CURRENT reads succeeded — see §5 |
| 6 | Read-only audit: full data-flow trace of `current_a`/`current_valid`/`current_buf_count`/`current_read_errors` | None | Confirmed current data is telemetry-only; does not influence `motor_state`, health score, alarm engine, fault latch, `/decision`, or `/trend` |
| 7 | Read-only regression/safety review of the 5ms fix | None | No negative regression identified to WTVB02 sampling, RPM processing, queue latency, watchdog margin, or any Core 1 task |

---

## 4. Baseline Measurements

Captured over a 150-second live serial session on the physical device (COM5, 115200 baud), diagnostic-logging build, **no delay fix applied**.

| Metric | Value |
|---|---|
| CURRENT reads attempted | **66** |
| CURRENT successes | **0** |
| CURRENT failures | **66/66** |
| Success rate | **0%** |
| Return code (100% of failures) | `ku8MBResponseTimedOut` (0xE2) |
| Timeout elapsed — min / avg / max | **2009 ms** / 2009 ms / 2009 ms (zero variance across all 66 samples) |
| Cadence between CURRENT attempts — min / avg / max | 2204 ms / **2217 ms** / 2230 ms |
| WTVB02 (`Sensor Reads`) errors over the same window | 0 |
| Motor state / RPM / health during capture | `state=2` (RUNNING), `rpm≈1780`, `health=100%` — normal |

**OBSERVATION.** All 66 failures returned exactly `ku8MBResponseTimedOut`, never CRC error, never invalid-slave-ID, never a Modbus exception code.

**FACT (library source).** `ModbusMaster.cpp`'s response-wait loop only reaches the full timeout (`ku16MBResponseTimeout = 2000`, `ModbusMaster.h:252`) when **zero response bytes ever arrive**; a wrong slave ID, wrong function code, or bad CRC is detected and returned within milliseconds of the first 5 bytes arriving (`ModbusMaster.cpp:754-775`). The observed 2009ms (2000ms constant + ~9ms overhead) is therefore consistent only with CTR4A01 never responding at all during these attempts — not with a malformed-but-received reply.

**FACT (independent hardware proof, user-supplied).** A standalone sketch (`experimental/CTR4A01_SENSOR/CTR4A01_SENSOR.ino`) using the identical slave ID (`0x01`), function code (`0x04`), register (`0x0000`), baud rate (9600), and wiring reads the same physical CTR4A01 device continuously with no timeout, reporting ~71–72 mA (~1.0A after CT scaling).

---

## 5. Controlled 5 ms Delay Experiment

**Change applied (exact, reviewed diff):**

```diff
     if (millis() - s_lastCurrentSampleMs >= CURRENT_SAMPLE_INTERVAL_MS) {
       s_lastCurrentSampleMs = millis();
+      // [PHASE2-EXPERIMENT] single controlled inter-frame delay before the only
+      // readCTR4A01Current() call site -- validates the T6->T7 turnaround hypothesis.
+      vTaskDelay(pdMS_TO_TICKS(5));
       localData.current_valid = readCTR4A01Current(localData.current_a);
     }
```

No other line in the file was touched for this experiment. Same board configuration used for both builds (ESP32-S3, `FlashSize=16M`, `PSRAM=opi`, `PartitionScheme=app3M_fat9M_16MB`, `CDCOnBoot=cdc`).

Captured over a 150-second live serial session immediately after flashing.

| Metric | Value |
|---|---|
| CURRENT reads attempted | **292** |
| CURRENT successes | **292** |
| CURRENT failures | **0/292** |
| Success rate | **100%** |
| Return codes observed | `ku8MBSuccess` × 292; all others × 0 |
| CURRENT raw values — min / avg / max | 76 mA / 76.61 mA / 78 mA |
| Cadence between CURRENT attempts — min / avg / max | 485 ms / **500.0 ms** / 514 ms |
| WTVB02 `Sensor Reads` cadence over same window | ~121/30s ≈ 248 ms/cycle (matches designed 250ms/4Hz) |
| Motor state / RPM / health during capture | `state=2` (RUNNING), `rpm≈1781`, `health=100%` — normal, unchanged from baseline |

---

## 6. Before / After Comparison

| Metric | Before (no delay) | After (+5ms delay) |
|---|---|---|
| CURRENT successes / attempts | **0/66** | **292/292** |
| Success rate | **0%** | **100%** |
| Return code | 100% `ku8MBResponseTimedOut` | 100% `ku8MBSuccess` |
| Timeout elapsed (avg) | 2009 ms | n/a (no failures) |
| Cadence (avg) | **2217 ms** | **500.0 ms** |
| `taskModbusRead` measured period | ~2217 ms (vs. 250ms designed) | ~248 ms (matches 250ms designed) |
| WTVB02 errors | 0 | 0 |
| Motor state correctness | Normal | Normal (unchanged) |

**OBSERVATION.** The two runs used the identical physical device, identical wiring, identical board flash configuration, differing only in the single line of §5.

---

## 7. Root Cause Supported by Evidence

**INFERENCE (root cause).** The missing Modbus RTU inter-frame silent interval between T6 (last WTVB02 transaction) and T7 (first CTR4A01 transaction) caused CTR4A01 to fail to recognize a validly-addressed request frame, resulting in complete silence from the sensor and a full library-timeout on every cycle. Restoring a delay consistent with the ≥3.5-character-time minimum (using the same 5ms idiom already applied at every other transaction boundary in this function) eliminated the failure entirely, reproducibly, with no measured side effects.

Evidence chain supporting this inference:
- **FACT:** T6→T7 was the only transaction boundary in the function with 0ms delay, versus 5ms everywhere else.
- **FACT:** T7 is the only transaction that both changes slave ID *and* function code.
- **FACT (hardware):** the standalone sketch, with the identical device/wiring/registers, only ever issues requests to slave `0x01` preceded by ≥500ms of bus idle time, and never fails.
- **OBSERVATION:** 66/66 failures, 100% `ku8MBResponseTimedOut`, zero variance in elapsed time (2009ms every time) — a deterministic, structural signature, not intermittent noise.
- **OBSERVATION:** after inserting exactly one 5ms delay at that boundary, 292/292 successes, 0 failures — full reversal.

**Elimination of alternative causes (FACT-based):**
- Hardware/wiring/power: eliminated by the standalone sketch's continuous success on the same physical device.
- Wrong slave ID/function/register/baud: eliminated — the standalone sketch uses identical values and succeeds; the failing return code (`ku8MBResponseTimedOut`) is structurally distinct from what a wrong-address/function/CRC error would produce (those resolve in milliseconds, not 2000ms).
- `modbus.begin()` side effects: eliminated by direct library-source inspection — `begin()` only reassigns `_u8MBSlave`/`_serial` and resets the transmit index; it does not touch UART hardware or buffers (`ModbusMaster.cpp:61-73`).
- Stale library state carrying over from T6 to T7: eliminated — `ModbusMasterTransaction()` resets all buffer indices at the end of every call, including T6's (`ModbusMaster.cpp:872-874`).

---

## 8. Remaining Uncertainties

Stated explicitly, per the "do not speculate beyond available evidence" constraint:

- **INFERENCE, not directly measured:** the exact physical bus-silence duration at the T6→T7 boundary was never measured with a logic analyzer or oscilloscope. The 0ms-vs-5ms code-timing argument and the ~4.01ms t3.5 calculation are derived from source code and the Modbus RTU specification, not from a direct electrical measurement.
- **Scope of validation:** only two ~150-second capture windows exist (one per state). No extended soak test (hours/days) has been run with the fix applied.
- **`current_read_errors` / `current_buf_count` MQTT fields** were never directly read via MQTT during this investigation (only inferred from the Serial-logged return codes, since these two fields are not printed to Serial by design).
- **Pre-existing, unrelated finding not remediated by this fix:** `ModbusMasterTransaction()`'s response-wait loop busy-spins with no yield point when no `idle()` callback is registered (none is, in this firmware). This was not created by the fix and is not fixed by it — noted for awareness, out of scope for this investigation.
- **Untested condition:** behavior when a WTVB02 fault/offline condition (`g_sensorOffline`) occurs simultaneously with a CTR4A01 read attempt was not specifically exercised in either capture.

---

## 9. Risk Assessment

Summary from the Phase 5 regression/safety review (full detail in that review):

| Area | Assessed impact |
|---|---|
| WTVB02 sampling (T1–T6) | None — the delay executes after T1–T6 complete, only on cadence-gated cycles |
| RPM processing / motor-state values | None — `processRPM()` computes purely from `millis()` deltas against tachometer pulses; correctness unaffected regardless of when it runs |
| Queue latency (`queueSensorData`, depth 5) | None — non-blocking producer, consumer faster than production rate in both scenarios |
| Watchdog margin (30s TWDT, both cores) | None — the pre-fix worst case (~2009ms busy-spin) was already far under the 30s threshold; the fix reduces Core-0 busy time, it does not add to it |
| MQTT / Analytics / Display (Core 1) | None — physically isolated by `xTaskCreatePinnedToCore`; a Core 0 delay cannot affect them |
| Stuck/offline-detection responsiveness (read-count-based thresholds) | Restored to intended ~1.25s/0.75s response times, which had been silently degraded to ~9x slower during the bug |

**Classification: B — Safe with monitoring.** Not classified "A — Safe for production outright" because validation to date is limited to two ~150-second capture windows and does not include an extended soak test or a simultaneous WTVB02-fault scenario.

---

## 10. Final Recommendation

Adopt the single `vTaskDelay(pdMS_TO_TICKS(5))` fix at the T6→T7 boundary (documented in §5), on the following basis:

- It is evidenced, not speculative: a controlled, single-variable experiment produced a 0% → 100% success-rate reversal on the same physical hardware.
- It is minimal and consistent with existing code style: identical to the delay idiom already used at every other transaction boundary in the same function.
- It has no measured negative impact on WTVB02 sampling, motor-state correctness, queue behavior, watchdog margin, or any other task, and restores previously-degraded fault-detection responsiveness as a side benefit.
- Recommended follow-up before treating this as fully closed: an extended soak test (multi-hour), and confirmation via MQTT that `current_read_errors` stops climbing and `current_buf_count` reaches steady state, to close the uncertainty noted in §8.

**This report reflects a read-only/experimental investigation. The 5ms fix exists only in the currently-flashed device and the local working tree; it has not been committed to source control as of this report.**
