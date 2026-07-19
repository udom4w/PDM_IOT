> **Document Status**
>
> **Design Review:** ✅ CLOSED (Approved) — see `mqtt_single_owner_design_v16_5.md`
>
> **Implementation:** ✅ COMPLETE — final wiring commit `ed55cad`
>
> **Regression / Soak:** ⏳ NOT STARTED
>
> **Baseline:** `adf315b` (v16.4, production fallback)
>
> **Candidate:** `ed55cad` (v16.5)
>
> **Production Readiness:** ❌ NOT YET APPROVED
>
> **Purpose:** This checklist operationalizes design v16.5 §8 (Test plan) into an executable, sign-off-able regression procedure. Design sign-off (already closed) and production-readiness sign-off (gated by this document) are separate gates — passing this checklist is what earns the latter.
>
> **⚠️ Baseline currency note (added during a documentation-reconciliation pass, not a regression run):** the repository's current `feature/mqtt-single-owner` HEAD is `4274e524ea3568e693c3fa1e5068b7ce118599fb`, three commits past this checklist's `ed55cad` candidate (`4f63f92`, `cc65593`, `4274e52`). Current HEAD is **not scope-identical to `ed55cad`** — it additionally contains a separately-closed `VERIFY_TEST` deglitch diagnostic capability (compiled out by default, verified zero-leakage into production builds) and, bundled into the same commit, unrelated peak-telemetry and sensor-configuration/calibration changes not tracked by this checklist. See `claude/DESIGN-0004_peak_telemetry.md` §10 and `claude/WTVB05_FIFO_Investigation_Report.md`'s PD-0003 entry for details. **This checklist's §1.1 table below still describes only the original `adf315b`→`ed55cad` scope and has not been re-run or re-scoped against current HEAD.** No checkbox in this document reflects any testing against `4274e52` — regression/soak status remains NOT STARTED regardless of which commit is used as the candidate.

# MQTT Single-Owner Refactor — Production Regression Checklist
**Firmware:** `WTVB02_ESP32S3_V16_CM_fault_latch_v3_hardened_patched_v16_4.ino`
**Design doc:** `design_reviews/mqtt_single_owner_design_v16_5.md`
**Sequencing note:** `design_reviews/... ` §7 clarification (Item 9 merged before Item 6)

---

## 1. Scope

### 1.1 Commits under test (baseline → candidate)

| Commit | Item | Description |
|---|---|---|
| `adf315b` | — | **Baseline** (v16.4) — pre-refactor, production fallback |
| `bfdbd1e` | 3 | Dormant outbound queue infrastructure |
| `d91d3b0` | 4 | Cached `mqttConnected` write points (Network4G) |
| `509d9e0` | 5 | Cached `mqttConnected` reader (`getMqttConnectedCached()`) |
| `a415598` | 7 | DisplayUpdate reads via cache |
| `5bfc970` | 8 | loopTask reads via cache |
| `4f77a16` | 9 | Network4G-side queue drain (dormant until Item 6) |
| `ed55cad` | 6 | **Candidate (v16.5)** — Analytics wired to cache read + enqueue (final wiring) |

### 1.2 In scope
- Every `mqttClient`/`gsmClient` call site touched by Items 3–9 (see design §3).
- The single-owner invariant (design §2a): `taskNetwork()` (Network4G) is the only task calling `mqttClient.*`/`gsmClient.*`.
- The new outbound queue (`queueMqttOutboundTrend`) and cache (`g_systemState.mqttConnected`).
- `/trend` publish path end-to-end (Analytics enqueue → Network4G drain → broker).

### 1.3 Out of scope (per design §6 — confirm untouched, do not re-test as if changed)
- [ ] `mutexVibData`, `mutexI2C`, `mutexModem`, `mutexAggBufs`, `mutexFaultLatch`, `mutexTelemBuf` — unmodified.
- [ ] `taskModbusRead` / `taskStateMachine` (Core 0) — zero `mqttClient` references before and after.
- [ ] `processRPM()`, `calcTrend()` — not touched by this refactor.
- [ ] JSON schema/field names for `/sensor`, `/status`, `/vibration`, `/trend` — unchanged.

### 1.4 Process gaps (non-firmware)
- [ ] **Section 7 Items 10 and 11 are NOT implemented.** These are process/tooling improvements — a CI enforcement script and a standalone diff-review step — not firmware defects; the firmware itself (Items 3–9, 6) is complete and functionally correct independent of these two items. Item 10 (build-failing CI/pre-build script enforcing the ownership invariant, design §7.10) does not exist yet — §5 below substitutes a manual `git grep` audit. Item 11 (full diff review vs. v16.4 baseline) has not been performed as a standalone step. Recommend implementing Item 10 as an automated gate before promoting past canary.

---

## 2. Build Verification

| Check | Baseline (`adf315b`) | Candidate (`ed55cad`) | Pass? |
|---|---|---|---|
| Clean build succeeds (arduino-cli, FQBN `esp32:esp32:esp32s3:CDCOnBoot=cdc,CPUFreq=240,FlashMode=qio,FlashSize=16M,PSRAM=opi,PartitionScheme=app3M_fat9M_16MB,USBMode=hwcdc,UploadMode=default,UploadSpeed=921600`) | ☐ | ☐ | ☐ |
| Zero compiler warnings introduced vs. baseline | ☐ | ☐ | ☐ |
| Flash usage recorded | ______ B | 636,202 B (20%)* | ☐ |
| RAM usage recorded | ______ B | 65,588 B (20%)* | ☐ |

\* Candidate figures are from the incremental implementation build (§9 has full per-commit detail). **A fresh clean build of `adf315b` has not been measured in this checklist's authoring session — fill in the Baseline column from an actual rebuild before signing off, do not assume.**

- [ ] Build from a clean `.arduino`/cache state (not incremental) at least once for both baseline and candidate.
- [ ] `compile_commands.json` / `includes.cache` regenerate without error.
- [ ] No new `.ino` auto-prototype ordering hazards (CLAUDE.md rule) — confirm any new function is not positioned earlier in the file than the pre-existing first function, or re-verify a clean compile after any further edits.

---

## 3. MQTT Connectivity

- [ ] Cold boot → modem init → GPRS connect → TLS load (`setupTLS()`) → `mqttClient.connect()` succeeds, matches baseline timing (± normal variance).
- [ ] Forced GPRS drop → automatic reconnect → MQTT reconnects once GPRS is back, matches baseline behavior.
- [ ] Forced broker-side disconnect (kill broker connection) → exponential backoff observed: 30s → 60s → 120s → 300s (cap), resets to 30s on successful reconnect — unchanged from baseline (Items 3–9 do not touch this logic).
- [ ] `gsmClient.resetTLS()` still called before every reconnect attempt (unchanged call site, line ~4109 area).
- [ ] `g_systemState.mqttConnected` (cache) reflects true connection state within one `taskNetwork()` iteration (~100ms) of every transition — verify via the 9 write points (design §4.2 rows #9,#14,#15/16,#17,#19,#20/#22,#23,#25).
- [ ] Display (`drawNetworkScreen`), Analytics gate, and `loop()` status report all show consistent MQTT state within the same ~100ms window during a connect/disconnect transition (no consumer stuck showing stale state beyond one tick).

---

## 4. Functional Regression

| Topic | Cadence | Payload byte-identical to v16.4 for fixed synthetic input? | Pass? |
|---|---|---|---|
| `/vibration` | on publish interval (30s NORMAL / 10s WARNING / 5s CRITICAL) | ☐ | ☐ |
| `/sensor` | same as `/vibration` | ☐ | ☐ |
| `/status` (`/decision`) | same as `/vibration` | ☐ | ☐ |
| `/trend` | 60s (Analytics, `analyticsPublishCnt >= 60`) | ☐ (payload construction unchanged — only the transport call changed) | ☐ |
| `/event` (maintenance audit) | on button-triggered maintenance reset | ☐ | ☐ |

- [ ] `TelemetrySlot_t` ring buffer replay (`replayTelemBuf()`) behaves identically — 1 slot/iteration, 75ms spacing, drop-on-24h-stale logic unaffected.
- [ ] `TelemBuf Pending/Overflow/Replayed` counters match baseline behavior under a forced offline burst.
- [ ] Fault latch (`g_fl`/`g_flCount`) replay-on-`/status` unaffected — snapshot-copy pattern, mutex timing unchanged.
- [ ] Maintenance-reset MQTT audit event (`queueMaintEvent` drain) unaffected — still gated on live `mqttClient.connected()` inside `taskNetwork()` (this call site was intentionally left direct, not swapped to cache — confirm it still reads live state, not cached).
- [ ] Motor stop/start cycle: RESUME/CLEAR logic, EMA reseed, slope suppression, `trend_gap_s` — all unaffected (confirm via diff that `processRPM()`/`calcTrend()` are untouched).
- [ ] OLED `PAGE_NETWORK` screen (`drawNetworkScreen`) MQTT status line renders correctly for both CONN/DISC states.
- [ ] 30s system status report (`loop()`) MQTT line and Cloud LED (GPIO15) blink pattern behave identically to baseline.

---

## 5. Single Owner Verification

### 5.1 Static (manual substitute for not-yet-built Item 10 CI gate)

Run and attach output:
```
git grep -n "mqttClient\.\|gsmClient\." -- '*.ino'
```
- [ ] Every match's enclosing function is one of: `taskNetwork()`, `setupTLS()`, `publishTelemetry()` (single caller confirmed = `taskNetwork`), or a comment/string literal.
- [ ] Zero matches inside `taskAnalytics()`, `drawNetworkScreen()`, `loop()`, `taskModbusRead()`, `taskStateMachine()`, `taskButtonHandler()`, `taskBuzzerControl()`.
- [ ] Last known-good result (this session, at `ed55cad`): confirmed zero direct `mqttClient.*` calls remain in `taskAnalytics()`; only `getMqttConnectedCached()` and `enqueueMqttOutbound()` are referenced there. **Re-run at execution time — do not rely solely on this note.**

### 5.2 Behavioral

- [ ] Runtime trace/log confirms Analytics never invokes any `mqttClient`/`gsmClient` method — only `getMqttConnectedCached()` (read) and `enqueueMqttOutbound()` (enqueue).
- [ ] Same confirmation for DisplayUpdate and loopTask — cache read only, nothing else.
- [ ] Confirm no code path infers "connected" indirectly by calling `.connected()` a second time immediately before a `.publish()` outside `taskNetwork()` (design §4.2 verification step — re-verify line-by-line, not just from this table).

### 5.3 Enforcement gap
- [ ] Item 10 CI script implemented and wired into build/pre-commit before this refactor is promoted past canary (see §1.4).

---

## 6. Queue Behavior

| Property | Locked spec (design §4.1) | Verify |
|---|---|---|
| Queue type | Independent `xQueueCreate`, separate from `g_telemBuf`/`mutexTelemBuf` | ☐ no shared state with telemetry replay |
| Message struct | `{topic_id, payload[1024], len, qos}` (`MqttOutboundMsg_t`) | ☐ matches implementation |
| Depth | 4–8 slots (implemented: 6, `QUEUE_SIZE_MQTT_OUTBOUND`) | ☐ |
| Overflow policy | Drop-newest with counter | ☐ `xQueueSend(..., 0)` non-blocking confirmed |
| Overflow counter | `g_trendEnqueueDropCount` | ☐ increments on forced overflow |
| Producer | `taskAnalytics()` only, one call site | ☐ |
| Consumer | `taskNetwork()` only, one call site, rate-limited to 1 msg/iteration | ☐ |

- [ ] **Overflow test:** force queue full (disconnect Network4G's drain by simulating backpressure or flooding faster than 1 msg/100ms) — confirm new enqueues are dropped (not the oldest), `g_trendEnqueueDropCount` increments, Analytics never blocks.
- [ ] **Queue depth stability under reconnect:** force MQTT disconnect, observe queue occupancy over the reconnect window — must stay bounded ≤6, drops counted correctly if exceeded.
- [ ] **Extended outage drain test:** force disconnect 5–10 minutes (beyond normal reconnect), restore connectivity — confirm queue drains fully once Network4G reconnects and `/trend` resumes normal 60s cadence with no backlog pile-up or stuck state.
- [ ] **Enqueue never blocks:** confirm `enqueueMqttOutbound()` returns immediately (non-blocking) even when queue is full — Analytics task timing must not stall.
- [ ] **Drain rate-limit holds:** confirm at most 1 `/trend` message is published per `taskNetwork()` iteration even if the queue has a backlog (matches the `g_telemBuf` replay analogy, design §4.1/§7 item 9).
- [ ] **Logging accuracy:** `[TREND] /trend %u B queued` / `FAILED to queue` in Analytics reflects enqueue outcome, not publish outcome; `[MQTT] Outbound queue published -> ... / FAILED ... (err=%d)` in Network4G reflects the actual broker publish outcome — confirm these are not conflated in logs during triage.

---

## 7. Fault Injection

- [ ] **Priority-injection repro (design §8.3):** insert a deliberate delay (e.g. `vTaskDelay`) in Network4G's TLS reconnect path immediately before `mqttClient.connect()` returns; force a reconnect at the same moment Analytics's 60s `/trend` flush is due. Repeat ≥100 cycles.
  - [ ] On **v16.4 unmodified** (`adf315b`): attempt to reproduce the original crash under controlled conditions.
  - [ ] On **v16.5** (`ed55cad`): confirm the crash no longer occurs under identical forced conditions.
  - [ ] If reproducible on baseline: capture crash log + `addr2line` backtrace, confirm frames land on the same `mbedtls_ssl_*` path documented in the original investigation.
- [ ] Forced broker kill mid-publish (during an active `mqttClient.publish()` call) — confirm clean failure handling, no panic, `publishFailures` increments appropriately.
- [ ] Forced GPRS drop mid-publish — confirm no panic, reconnect logic proceeds normally afterward.
- [ ] Malformed/oversized `/trend` payload (force `sz >= sizeof(buf) - 1`) — confirm existing truncation warning still fires and `enqueueMqttOutbound()`'s `len` bound check (`len >= MQTT_OUTBOUND_PAYLOAD_MAX`) rejects safely.
- [ ] Watchdog behavior unaffected — confirm `esp_task_wdt_reset()` cadence inside `taskNetwork()`'s main loop still covers the new queue-drain block (it executes between existing WDT resets, adds no blocking I/O).

---

## 8. Long-run (24h) Soak Test

Per design §8.4 — production-readiness gate, not optional.

- [ ] 24–48h continuous run on `ed55cad`, replicating field conditions:
  - [ ] Periodic 4G signal drops (forces MQTT reconnect cycles)
  - [ ] Motor stop/start events at irregular intervals
  - [ ] Normal 60s `/trend` cadence throughout
- [ ] **Pass criterion: zero panics/unexpected reboots.**
- [ ] Compare `g_rebootCount` / `g_resetReasonStr` (NVS-persisted) before vs. after soak — expect 0 unexpected `PANIC`/`TASK_WDT`/`INT_WDT` resets.
- [ ] Stack high-water-mark check (`uxTaskGetStackHighWaterMark`) for `taskNetwork` and `taskAnalytics` specifically — both gained local variables, mutex calls, and (Analytics) a queue-send call; confirm no closer approach to stack exhaustion vs. baseline.
- [ ] Heap watermark stable over 24h (no leak introduced by the new queue/struct allocations, which are static `.bss` — confirm no unexpected heap growth elsewhere).
- [ ] `g_trendEnqueueDropCount` remains 0 (or low and explained) over a normal-conditions 24h run — sustained non-zero drops would indicate the drain isn't keeping up and warrants investigation before fleet rollout.

---

## 9. Performance Comparison

### 9.1 Build size (incremental, this implementation session — informational; re-measure baseline vs. final directly before sign-off)

| Commit | Item | Flash (B) | RAM (B) |
|---|---|---|---|
| — | (pre-Item-3 / true v16.4 baseline) | *not measured this session — measure from `adf315b`* | *not measured* |
| `bfdbd1e` | 3 | 634,882 | 65,580 |
| `d91d3b0` | 4 | 635,338 | 65,580 |
| `509d9e0` | 5 | 635,338 | 65,580 |
| `a415598` | 7 | 635,370 | 65,580 |
| `5bfc970` | 8 | 635,386 | 65,580 |
| `4f77a16` | 9 (+ logging fix) | 635,582 | 65,580 |
| `ed55cad` | 6 (final) | 636,202 | 65,588 |

- [ ] Perform a fresh clean-build size comparison: `adf315b` vs. `ed55cad` directly, record delta, confirm it is small and explainable (expect low hundreds of bytes flash, ≤10 bytes RAM based on incremental data above).

### 9.2 Runtime timing

- [ ] `taskNetwork()` loop iteration time — added 9 `xSemaphoreTake`/`xSemaphoreGive` pairs (cache writes) + 1 queue-drain check per iteration; measure and confirm negligible impact (expect microseconds, not milliseconds).
- [ ] `taskAnalytics()` per-`/trend`-cycle time — added 1 cached read + 1 `enqueueMqttOutbound()` call (replacing 1 direct `.publish()` call); measure and confirm no regression (enqueue should be faster than the direct blocking publish it replaced).
- [ ] **New metric — enqueue-to-publish latency:** time from Analytics's `enqueueMqttOutbound()` call to Network4G's actual `mqttClient.publish()` call for the same message. This latency did not exist before this refactor (publish was synchronous) — measure it, confirm it stays bounded (worst case ≈ one `taskNetwork()` iteration period, ~100ms, under normal load) and document the expected range.
- [ ] mTLS handshake / reconnect timing unaffected (no code in that path was touched).

### 9.3 Queue metrics

- [ ] Maximum queue occupancy (`queueMqttOutboundTrend`) recorded — highest number of messages observed waiting to be drained at any point during testing.
- [ ] Average queue occupancy recorded — typical steady-state depth under normal 60s `/trend` cadence.
- [ ] Maximum enqueue latency recorded — worst-case time for `enqueueMqttOutbound()` to return (should remain near-instant/non-blocking per §6).
- [ ] Queue depth stayed within configured limit (`QUEUE_SIZE_MQTT_OUTBOUND` = 6) throughout all test scenarios — no growth beyond the configured bound observed.
- [ ] No unexpected queue growth during soak test (§8) — occupancy trend over the 24–48h run stays flat/bounded, not trending upward.

---

## 10. Acceptance Gate

Production baseline promotion (`ed55cad` → fleet v16.5) requires **all** of the following:

| Gate | Status |
|---|---|
| §2 Build Verification — clean build, sizes recorded, zero new warnings | ☐ |
| §3 MQTT Connectivity — all transitions verified | ☐ |
| §4 Functional Regression — all 5 topics + TelemBuf + fault latch + maintenance audit verified | ☐ |
| §5 Single Owner Verification — static + behavioral confirmed, Item 10 CI gap acknowledged | ☐ |
| §6 Queue Behavior — overflow, drain, extended-outage tests pass | ☐ |
| §7 Fault Injection — priority-injection repro attempted on both baseline and candidate | ☐ |
| §8 Soak Test — 24–48h, zero unexpected resets | ☐ |
| §9 Performance Comparison — deltas measured and explainable | ☐ |
| Canary: 1 field unit running `ed55cad` for an agreed observation window | ☐ |
| `adf315b` (v16.4) remains tagged and flashable as immediate rollback target | ☐ |
| Rollback from `ed55cad` to `adf315b` verified successfully | ☐ |

**Only after every row above is checked may `ed55cad` be promoted to fleet baseline as v16.5.** Design sign-off (already closed) and this production-readiness sign-off are separate gates — this document is the latter.

| Field | Value |
|---|---|
| Reviewer | ________________ |
| Date | ________________ |
| Result | ☐ PASS ☐ FAIL ☐ CONDITIONAL |
| Notes | ________________ |
