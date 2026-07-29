> **Document Status**
>
> **Type:** Engineering Investigation Plan — planning document only. No experiments have been executed. No firmware has been modified.
> **Version:** 1.0
> **Date:** 2026-07-28
> **Subject:** WTVB05 RS485 communication regression — sensor unresponsive (`0xE2`, zero bytes) on register `0x003A`, first observed this session, last known good 2026-07-27 23:51:11.
> **Input:** treated as fixed evidence — three prior investigations this session: (1) Production Readiness Review of `WTVB05_ValidationTool_v3_11_TRUEPOLL`, (2) forensic comparison of Log A (failing) vs Log B (working), (3) regression timeline analysis. No new repository investigation was performed to produce this document.
> **Constraint:** no firmware redesign; no firmware source modification unless an experiment below is explicitly marked as requiring one, and even then only as a last resort after all non-invasive experiments are exhausted.

# WTVB05 Communication Regression — Engineering Investigation Plan

---

## 1. Current Facts

Proven only. Each fact is cited to its source. No inference beyond what the citation directly supports.

### 1.1 Last Known Good — 2026-07-27, 23:51:11

| # | Fact | Source |
|---|---|---|
| F1 | A 700-second continuous capture completed with **zero** `0xE2` occurrences and **zero** offline events. | `serial_capture_test7_soak_20260727.log`, `###CAPTURE_STATUS=COMPLETE elapsedSeconds=700.163574` |
| F2 | The capture was taken running **production firmware**, slave `0x50`, on `SerialRS485` (UART2, pins 38/39, EN 42). | same log; production `.ino` constants |
| F3 | Telemetry values including `peak=` were live and varying (`3.85`, `3.84`, `3.89`, `3.95`, `3.96`, `7.55`, ...). `peak` is sourced from `REG_PEAK_X = 0x3A` — **the exact register that later fails**. | production `.ino:267,4405`; `FIRMWARE_CONFIG_AUDIT_v16.5.md:95` — `REG_PEAK_X | 0x3A | ... | 3830 | Yes | No | hardware | used` |
| F4 | This is the most recent proof, from any firmware, that register `0x3A` was reachable on this bus. | derived from F1–F3; no later successful read exists in the repository |

### 1.2 Confirmed activity inside the regression window (23:51:11 → live probe)

| # | Fact | Source |
|---|---|---|
| F5 | Commit `322ef0b` (23:55:31, tag `v16.5.2-p4.2`) modifies the production `.ino`. The diff touches only `g_currentEvidenceValid`, `TelemetrySnapshot.currentEvidenceValid`, and the MQTT field `current_evidence_valid` — a CTR4A01 telemetry mirror. **Zero lines touch `SerialRS485`, `modbus.begin()`, any `RS485_*_PIN`, `MODBUS_BAUDRATE`, or any WTVB05 register.** | `git show 322ef0b`, full diff reviewed |
| F6 | Commit `70809a4` (00:02:28) documents an FQBN change (`CDCOnBoot=cdc`) affecting USB-CDC serial visibility only. Explicitly scoped in its own text: *"boot ROM messages still appear, but every application `Serial.print()`/`Serial.println()` is lost, **while MQTT/application behavior is unaffected**."* No relationship to UART2/RS485. | `git show 70809a4`; `CLAUDE.md` (as committed) |
| F7 | The same commit (`322ef0b`) adds `P4_02_HARDWARE_VERIFICATION_PLAN.md` and `P4_02_FINAL_VERIFICATION_PACKAGE.md`, whose Tests 4, 5, and 7 explicitly instruct: *"Disconnect CTR4A01/RS485 for ~2 s"* / *"≥35 s"* / repeated cycles *"twice"* over a session of *"≥5 min."* | `P4_02_HARDWARE_VERIFICATION_PLAN.md:65-68`; `P4_02_FINAL_VERIFICATION_PACKAGE.md:56,83,125` |
| F8 | `"CTR4A01/RS485"` in F7 refers to the **same physical bus** that carries the WTVB05. Confirmed in source: single `HardwareSerial SerialRS485(2)` object (`:573`); `readCTR4A01Current()` re-addresses the same object between slave `0x01` and slave `0x50` (`:4244,4246`); its own comment states it *"shares the WTVB02 bus/rs485Enable() window, single acquisition source, no new UART/task/timer."* | production `.ino:573,4239-4246` |
| F9 | Both documents in F7 explicitly state the disconnect/reconnect procedure had **not been executed** at commit time: *"Hardware execution of this plan and its results are the next step, to be supplied before any commit or tag is considered"* / *"execution and results are the next step."* | `P4_02_HARDWARE_VERIFICATION_PLAN.md:159-160`; `P4_02_FINAL_VERIFICATION_PACKAGE.md:187-188` |
| F10 | **No file of any kind — log, capture, commit — exists anywhere in the repository with a timestamp between 00:03 and 09:00 on 2026-07-28.** | exhaustive filesystem search, prior session |
| F11 | The device currently responds with boot strings (`[INIT] Checking sensor connection...`, `[INIT] Sensor not responding – retrying in 3 s...`) that exist **only** in `WTVB05_ValidationTool_v3_11_TRUEPOLL.ino`, nowhere else in the repository. | repo-wide grep, prior session; `.ino:2682-2687` |
| F12 | No tool call in this session issued `arduino-cli` with `--upload`, `-u`, or a port flag. All compiles in this session targeted the scratchpad only. | this session's own command log |

### 1.3 First Known Bad — live probe, this session

| # | Fact | Source |
|---|---|---|
| F13 | `readHoldingRegisters(0x003A, 3)` failed with `0xE2` (`ku8MBResponseTimedOut`) on **8 consecutive attempts** over ~40 s. `0xE2` denotes zero bytes received, not a corrupted or exception response. | live probe capture, this session |
| F14 | A `TRUEPOLL 1` command sent during the failure was **never acknowledged** — the device never printed a `[TRUEPOLL]` line of any kind. | live probe capture, this session |
| F15 | `readVelocityComparison()` is called in an **unconditional, unbounded loop** (`while (!readVelocityComparison(probe)) { delay(3000); }`) with no timeout and no escape. `loop()` — and therefore `handleSerialCommands()` — is provably unreachable while this loop runs. | `.ino:2684-2687` |

### 1.4 Cross-referenced firmware/bus facts (established in the forensic comparison)

| # | Fact | Source |
|---|---|---|
| F16 | Every `0xE2` recorded in any *working* validation-tool log (10 occurrences, 7 log files) is the same operation: `writeSingleRegister(REG_SAVE_REBOOT, SAVE_CMD)` — a write the sensor is not expected to ACK. **No log anywhere shows a data-register READ returning `0xE2` other than F13.** | prior forensic comparison, full log grep |
| F17 | Pins, baud, and slave ID are byte-identical between production and the Validation Tool: `RS485_RX_PIN=38`, `RS485_TX_PIN=39`, `RS485_EN_PIN=42`, `9600 8N1`, slave `0x50`. Neither firmware defines a sensor power-enable GPIO. | source comparison, both `.ino` files |
| F18 | `0x003A` has a clean, unbroken historical record as a post-reboot aliveness probe: 4/4 successes, 0/4 failures, across all recorded logs (validation tool + an independent earlier project, `WTVB02_SensorReset_Test.ino:32-33`, using the same register for the same purpose). | prior startup-probe analysis |
| F19 | The Validation Tool's RS485 RX buffer is `2048` B (`:2673`, explicit call); production never calls `setRxBufferSize()` (default `256` B). This affects overflow tolerance for large transfers, not the ability to receive any bytes at all — irrelevant to a **zero-byte** symptom. | source comparison |

---

## 2. Unknowns

Every open technical question this investigation must answer. Grouped by what physical/logical layer they probe.

| # | Unknown | Why it matters |
|---|---|---|
| U1 | Was the P4-02 disconnect/reconnect procedure (F7) actually executed? | Determines whether F7-F9 is a live candidate or an unexecuted plan |
| U2 | Is the RS485 physical wiring (A/B/GND) currently continuous between the ESP32 and the WTVB05? | Directly tests the open-circuit hypothesis |
| U3 | Is the WTVB05 sensor currently powered? | Total silence is equally consistent with no power as with no signal path |
| U4 | Does the second slave on the same bus, CTR4A01 (`0x01`), currently respond? | The single most discriminating test available — separates "whole bus down" from "WTVB05 specifically down" |
| U5 | Does the binary currently flashed match the reviewed `.ino` source, and was it built with the validated FQBN? | A mismatched/corrupted image is a distinct failure class from a source-logic defect |
| U6 | Is GPIO42 (`RS485_EN_PIN`) currently held LOW (receive-enabled) on the running device? | If stuck HIGH, the transceiver is permanently transmit-only and would produce exactly the observed symptom regardless of sensor/wiring health |
| U7 | Has the WTVB05's Modbus slave address been altered from `0x50`? | No firmware in the repo writes an address register, but the full register map was not exhaustively checked against the datasheet for one |
| U8 | Is the WTVB05 sensor itself functional, independent of this ESP32 and this firmware? | Required to distinguish sensor hardware failure from anything upstream of it |
| U9 | Precisely when, and by what mechanism, did the running firmware change from production to the Validation Tool? | F11 proves the switch happened; no artifact establishes when or how |
| U10 | Is the physical ESP32-S3 board currently on the bench the same unit that ran the 23:51:11 soak test? | An unstated board swap would invalidate any "same hardware" assumption underlying every hypothesis below |

---

## 3. Hypothesis Matrix

Confidence is qualitative, weighted only by the evidence in §1 — no fabricated probabilities.

| ID | Hypothesis | Supporting evidence | Contradicting evidence | Confidence | Confirming/rejecting experiment |
|---|---|---|---|---|---|
| **H1** | RS485 bus physically open/disconnected (loose or unrestored connector), most plausibly from the F7 disconnect/reconnect procedure not being fully reversed. | F7 + F8 + F9: a documented plan to physically disconnect this exact bus, timestamped inside the only unexplained gap (F10); total silence (F13) is the textbook signature of an open circuit. | None directly — nothing confirms the test ran, but nothing confirms it was skipped either. | **Medium-High** — the only hypothesis with a documented, timed, mechanistically-matched trigger event. | Exp. 1 (§4) |
| **H2** | RS485 transceiver or direction-control (EN pin) fault on the ESP32 board — e.g., EN stuck HIGH, permanently transmit-enabled. | Structurally capable of producing total silence regardless of wiring/sensor health; EN-pin handling differs between firmwares (production toggles it, Validation Tool holds it permanently LOW), and a reflash (F11, confirmed) means the board was recently handled. | No log or commit evidence directly implicates the transceiver or EN pin. | **Low-Medium** | Exp. 3 (§4) |
| **H3** | WTVB05 sensor not currently powered. | Total silence is equally consistent with no power as with H1; F7's Test 1 mentions a "cold boot (power cycle)" of "the device," though that is the ESP32, not stated as the sensor's own supply. | No documented action specifically targets sensor power (only bus disconnect is documented in F7). | **Medium** | Exp. 1 (§4) |
| **H4** | Sensor Modbus address altered from `0x50`. | None identified — U7 remains genuinely open. | No firmware reviewed writes any address-change register; F17 confirms slave ID is source-identical between firmwares. | **Low** | Exp. 4 (§4) |
| **H5** | Deployed ESP32 binary defective, stale, or built with a wrong/mismatched FQBN — independent of RS485 physical layer. | F11 confirms a firmware switch occurred; `CLAUDE.md` (per F6) documents a known class of silent build-config failure on this exact hardware (default FQBN → `CDCOnBoot=default`), establishing that build-config-driven silent faults are a real risk category here. | F6 itself scopes that specific known bug to USB-CDC only, explicitly stating *"MQTT/application behavior is unaffected"* — it does not touch RS485. No direct evidence connects it to this symptom. | **Low-Medium** | Exp. 2 + U5 check (§4) |
| **H6** | WTVB05 sensor hardware failure (internal fault), unrelated to address or power. | F4 establishes the sensor was healthy as recently as 23:51:11 the same session-day; no evidenced fault-inducing event other than H1/H3, both of which are external to the sensor itself. | Nothing rules it out, but nothing positively implicates it over an external cause either — residual hypothesis. | **Low** | Exp. 4 (§4) |
| **H7** | ESP32 UART2 pin-level fault (GPIO 38/39), e.g. from handling during the reflash (F11). | A reflash requires physical USB handling of the board; pin-level damage during handling is a known general risk. | No direct evidence. | **Low** | Exp. 3 (§4) |

---

## 4. Prioritized Experiment Plan

Ranked by information gain — expected number of hypotheses resolved per experiment, using only existing, unmodified artifacts wherever possible. **Proceed to the next experiment only if the previous one is inconclusive.** In the best case, one experiment closes the investigation.

---

### Experiment 1 — Physical-layer inspection (visual + multimeter)

| | |
|---|---|
| **Resolves** | U1 (partially), U2, U3 — directly tests H1 and H3 |
| **Objective** | Determine whether the RS485 bus (A/B/GND) and the WTVB05's power rail are physically intact, with zero firmware or software involvement. |
| **Procedure** | 1. Visually inspect the RS485 connector(s) at the ESP32 (pins 38/39/42 header) and at the WTVB05, for a disconnected, reversed, or partially-seated connector — the direct physical signature a botched F7 restoration would leave. 2. With the bus unpowered, multimeter continuity check: ESP32 A↔sensor A, ESP32 B↔sensor B, GND↔GND. 3. With the system powered, multimeter voltage check on the sensor's power input pins against its rated supply. 4. Record findings with a photo if a fault is found. |
| **Required equipment** | Multimeter. No spare parts, no code, no ESP32 involvement. |
| **Expected observations & interpretation** | See table below. |

| Observation | Interpretation | Next step |
|---|---|---|
| Connector visibly disconnected/loose | **H1 confirmed.** Reconnect, then re-run the live probe (F13's exact test) to verify recovery. | If recovered → **stop, root cause = RS485 communication issue (physical).** If not recovered after reconnection → proceed to Exp. 2. |
| Continuity fails on A, B, or GND despite connector appearing seated | **H1 confirmed** (broken conductor, not a connector). | Same as above. |
| Sensor power rail absent or out of spec | **H3 confirmed.** | Restore power, re-run F13's test. If recovered → **stop, root cause = sensor hardware issue (power supply), not a communication fault.** If not recovered → proceed to Exp. 2. |
| Wiring continuous, power correct, connectors seated | **H1 and H3 both rejected.** | Proceed to Exp. 2. |

---

### Experiment 2 — Dual-slave differential test (redeploy existing production firmware)

| | |
|---|---|
| **Resolves** | U4, U9 (bounds it), and discriminates H2/H5/H7 (bus/board-wide) from H4/H6 (WTVB05-specific) |
| **Objective** | Determine whether the fault is bus/board-wide (affects every slave) or isolated to the WTVB05 specifically, using an already-validated binary — no code is written or modified. |
| **Procedure** | 1. Retrieve the production `.ino` from git HEAD (`git show HEAD:.../WTVB02_ESP32S3_V16_CM_fault_latch_v3_hardened_patched_v16_5.ino`) — this restores the exact content already proven at F1-F4; it is a **restore of unmodified tracked content, not an edit**. 2. Compile with the validated FQBN from `CLAUDE.md` (`CDCOnBoot=cdc,CPUFreq=240,...`) — the same configuration used for the soak test. 3. Flash to the board currently on the bench. 4. Observe boot: production reads `REG_UNLOCK` (write, `0x0069`) first, then polls WTVB05 (`0x50`) registers including `0x3A`, then separately addresses CTR4A01 (`0x01`) via `readCTR4A01Current()`. Both are soft-fail — the firmware reports each independently rather than halting. |
| **Required equipment** | Existing production source (git), arduino-cli/Arduino IDE, USB cable — all already used earlier this session. No spare hardware. |
| **Expected observations & interpretation** | See table below. |

| Observation | Interpretation | Next step |
|---|---|---|
| **Both** WTVB05 and CTR4A01 respond normally | Bus, transceiver, EN pin, and ESP32 UART are all proven functional. The fault is confined to the Validation Tool's own state on the device, or was transient and has since cleared. **H1, H2, H3, H6, H7 all rejected for the current state.** | Proceed to Exp. 5 (software-only confirmation) to determine whether it's a build/deployment issue (H5) or the fault has simply resolved. |
| WTVB05 silent, **CTR4A01 responds** | Bus, transceiver, EN pin, and ESP32 UART are proven functional (a working transaction just occurred on the same wire). Fault is isolated to the WTVB05 specifically. **H2, H7, and shared-bus H1 rejected.** Sensor-specific H1 (its own drop/stub, if wiring is not a single shared trunk), H3 (if power is per-sensor), H4, H6 remain open. | Proceed to Exp. 4. |
| **Both** silent | Fault is upstream of both slaves — shared bus, transceiver, EN pin, or ESP32 UART. **H4, H6 rejected** (a sensor-specific fault cannot silence a different device). | Proceed to Exp. 3. |
| Neither responds AND Exp. 1 already found a physical fault, unresolved | Confirms Exp. 1's finding under an independent firmware; does not add new information. | Resolve the Exp. 1 finding physically before repeating. |

---

### Experiment 3 — ESP32 board / transceiver isolation

*(Only reached if Exp. 2 shows both slaves silent with Exp. 1's physical checks clean.)*

| | |
|---|---|
| **Resolves** | H2, H7 |
| **Objective** | Determine whether the fault is on the ESP32 board (transceiver, EN pin, UART pins) rather than the bus or sensors. |
| **Procedure** | **3a (no spare board needed):** With production firmware running, multimeter/logic-probe check of GPIO42 during a transaction — confirm it reads LOW (receive-enabled) at rest and during the read window, matching `rs485Enable()`'s intent. If available, an oscilloscope/logic analyzer on TX (pin 39) confirms the request frame is actually being transmitted. **3b (if 3a is inconclusive and a spare board is available):** Substitute a known-good ESP32-S3 board, identical wiring, same production firmware. |
| **Required equipment** | Multimeter (mandatory); oscilloscope/logic analyzer (optional, strengthens 3a); spare ESP32-S3 board of the same type (optional, only for 3b). |
| **Expected observations & interpretation** | See table below. |

| Observation | Interpretation | Next step |
|---|---|---|
| GPIO42 stuck HIGH, or no TX activity observed | **H2 confirmed** (EN/transceiver fault). | **Stop — root cause = ESP32 firmware/hardware issue (RS485 direction control).** |
| GPIO42 correctly LOW, TX activity present, but still no response from either slave | H2/H7 on this board not indicated by this test alone; if a spare board (3b) then responds correctly to both slaves on the same wiring | **H7 confirmed** (original board's UART/transceiver hardware fault). **Stop — root cause = ESP32 firmware issue (board hardware).** |
| Spare board (3b) also silent on both slaves, same wiring | H2/H7 rejected for the ESP32 side entirely. | Return to physical layer — re-run Exp. 1 with more scrutiny (e.g., termination resistor, cable damage between the connector and the sensor housing not visible in the first pass), since two independent boards failing identically on the same wiring points back to the shared physical path. |

---

### Experiment 4 — Sensor isolation

*(Only reached if Exp. 2 shows WTVB05 silent while CTR4A01 responds.)*

| | |
|---|---|
| **Resolves** | H4, H6, U7, U8 |
| **Objective** | Determine whether the WTVB05 unit itself is at fault, independent of this ESP32, this firmware, and this specific wiring run. |
| **Procedure** | **4a (preferred, fully independent):** Connect the WTVB05 to a separate USB-RS485 adapter and the WitMotion PC software (already present in the project tree — `PDM_IOT/Download the WitMotion Software`), bypassing the ESP32 entirely. Attempt to detect the sensor; if the vendor tool has an address-scan function, scan for a response at any address, not only `0x50`. **4b (if a spare sensor is available):** Substitute a known-good WTVB05/WTVB02-485 unit on the identical ESP32 wiring already proven functional for CTR4A01 in Exp. 2. |
| **Required equipment** | USB-RS485 adapter + WitMotion PC software (4a, adapter is the only new equipment item); spare WTVB05 unit (4b, optional). |
| **Expected observations & interpretation** | See table below. |

| Observation | Interpretation | Next step |
|---|---|---|
| WitMotion tool detects the sensor at `0x50` | **H4 and H6 both rejected.** Sensor is fundamentally healthy and correctly addressed; the fault is specific to this ESP32-to-sensor wiring path or transaction, not the sensor itself. | Re-inspect the WTVB05-side connector/drop specifically (not the shared trunk, already cleared in Exp. 2) — likely a localized wiring fault at the sensor end. **Root cause = RS485 communication issue (localized wiring).** |
| WitMotion tool detects the sensor at a **different** address | **H4 confirmed.** | **Stop — root cause = sensor configuration issue.** Re-addressing the sensor (via the vendor tool, not firmware) restores communication. |
| WitMotion tool cannot detect the sensor at any address, with the adapter/software otherwise verified working | **H6 supported** — sensor unresponsive to an entirely independent master, ruling out anything ESP32/firmware-side. | If a spare sensor (4b) is available, confirm: spare responds normally on the identical wiring → **H6 confirmed. Stop — root cause = sensor hardware failure.** |
| No spare sensor available to run 4b | H6 cannot be fully confirmed vs. an unidentified fault in the sensor's power/ground specifically (already checked in Exp. 1, but re-verify at the sensor's own terminals, not upstream). | Report as **inconclusive between sensor hardware failure and an unresolved localized power/wiring fault at the sensor** — state this explicitly rather than guessing. |

---

### Experiment 5 — Software-only confirmation

*(Only reached if Exp. 2 shows both slaves responding normally under production firmware — i.e., the physical/board/bus layer is entirely clean.)*

| | |
|---|---|
| **Resolves** | H5, U5, U9 (bounds it) |
| **Objective** | Determine whether the Validation Tool's own failure is reproducible on hardware just proven healthy, isolating the fault to software/deployment rather than physical state. |
| **Procedure** | 1. Immediately after Exp. 2 confirms both slaves healthy under production firmware, without touching any wiring, flash the **unmodified** `WTVB05_ValidationTool_v3_11_TRUEPOLL.ino` (git HEAD content, no edits) back to the same board. 2. Observe whether `readVelocityComparison()`'s first transaction (`0x3A`) succeeds or reproduces F13. 3. If it fails, record the exact `GIT_COMMIT_HASH`/build fingerprint of the binary just flashed (note: this sketch has no `build_info.h`; record the arduino-cli output's compiled artifact path/hash and the exact FQBN used, since U5 asks whether the deployed image matches the reviewed source). |
| **Required equipment** | Existing Validation Tool source (git), arduino-cli/Arduino IDE, USB cable. No new hardware. |
| **Expected observations & interpretation** | See table below. |

| Observation | Interpretation | Next step |
|---|---|---|
| Validation Tool now succeeds | The original failure was **transient** (most consistent with H1/H3 having been present at probe time and self-resolved or resolved by an intervening action, e.g. someone reseating the connector between sessions) — not a software defect. | **Stop — root cause = RS485 communication issue (transient), already resolved; no residual defect to fix.** Document the transient nature explicitly; do not classify as a software regression without a reproducible failure. |
| Validation Tool **reproduces F13** on hardware just proven healthy under production firmware, same wiring, same session | Direct, controlled demonstration that identical physical conditions produce success under one firmware and failure under another. | **Stop — root cause = software regression, isolated to the Validation Tool build/deployment (H5).** Follow with a source-level comparison of the exact flags/FQBN used for this build against the validated configuration (U5) before concluding it is a source-logic defect versus a build-configuration defect — these are distinguished in §6. |

---

## 5. Decision Tree

Shortest path from the current state to a root-cause declaration. Each node is one experiment from §4; each edge is an observed outcome.

```
                         ┌───────────────────────────────┐
                         │   CURRENT STATE                │
                         │   0x3A silent, 0xE2 x8, F13     │
                         └───────────────┬────────────────┘
                                         │
                              ┌──────────▼───────────┐
                              │  Exp.1: Physical      │
                              │  inspection           │
                              │  (multimeter/visual)  │
                              └──┬────────┬────────┬──┘
              connector loose /  │        │        │  wiring + power
              continuity fails   │        │        │  both clean
                                 ▼        │        ▼
                    ┌────────────────┐   │   ┌──────────────────────┐
                    │ H1 CONFIRMED   │   │   │ Exp.2: Dual-slave     │
                    │ reconnect,     │   │   │ differential test     │
                    │ re-test F13    │   │   │ (production firmware) │
                    └───┬────────┬───┘   │   └──┬─────────┬───────┬──┘
                recovered│  not   │       │      │         │       │
                         │recovered       │ both │  WTVB05 │  both
                         ▼        └───────┼─responded silent, │silent
              ┌──────────────────┐        │      │  CTR4A01│       │
              │ STOP:            │        │      │  responds│      │
              │ RS485 comm issue │        │      ▼      ▼   ▼      ▼
              │ (physical)       │        │  ┌────────┐ ┌─────────┐┌────────┐
              └──────────────────┘        │  │Exp.5:  │ │Exp.4:   ││Exp.3:  │
                                          │  │Software│ │Sensor   ││Board / │
              power rail absent/          │  │-only   │ │isolation││trans-  │
              out of spec ─────────┐      │  │confirm │ │(WitMotion││ceiver  │
                                   ▼      │  └───┬──┬─┘ │ or spare││isolation│
                        ┌────────────────┐│      │  │   │ sensor) ││(EN pin,│
                        │ H3 CONFIRMED   ││   VT  │  VT  └──┬───┬──┘│spare   │
                        │ restore power, ││ succeeds fails  │   │   │board)  │
                        │ re-test F13    ││      │  │  found responds└──┬──┬─┘
                        └───┬────────┬───┘│      ▼  ▼  at other │       │  │
                    recovered│  not   │    │  ┌─────┐┌────────┐ addr    │  │
                             │recovered    │  │STOP:││STOP:   │ ▼      GPIO │both
                             ▼        │    │  │trans-││software│┌──────┐stuck│slaves
                  ┌──────────────────┐│    │  │ient, ││regres- ││STOP: │HIGH/│respond
                  │ STOP:            ││    │  │ RS485││ sion   ││sensor│no TX│on
                  │ sensor hardware  ││    │  │already││(H5)   ││config││    │spare
                  │ issue (power)    ││    │  │resolved│       ││issue ││    │board
                  └──────────────────┘│    │  └──────┘└────────┘│(H4)  ││    │
                                       │    │                    └──────┘│    ▼
                                       │    │  responds at 0x50,        ▼ ┌──────────┐
                                       │    │  no other address found  ┌────┐│wiring    │
                                       │    │  (WitMotion) ────────────┤STOP:││re-check, │
                                       │    │                          │ESP32││both boards│
                                       │    │  no response at any       │fw   ││fail      │
                                       │    │  address, spare sensor    │issue││ same wiring│
                                       │    │  responds normally ───────┤(board)└──────────┘
                                       │    │                          └────┘
                                       │    │  ┌──────────────────┐
                                       │    └─▶│ STOP:             │
                                       │        │ sensor hardware   │
                                       │        │ failure (H6)      │
                                       │        └──────────────────┘
                                       │
                        (Exp.1 all clean, proceed to Exp.2 — path above)
```

**Shortest possible path:** Exp. 1 alone, if the physical inspection finds and the technician reconnects a loose connector — one experiment, root cause = H1.
**Longest path:** Exp. 1 → 2 → 3 or 4 → (5) — at most four experiments, still bounded and terminating.

---

## 6. Investigation Stop Criteria

Exact evidence required to declare each category. Each row also states what is **not** sufficient, to prevent a premature conclusion from a single ambiguous observation.

| Category | Sufficient evidence to declare | NOT sufficient alone |
|---|---|---|
| **RS485 communication issue** | Exp. 1 finds a physical open circuit, reversed polarity, or disconnected connector directly (continuity failure or visual confirmation) — OR Exp. 4 shows the sensor responds correctly via an independent master (WitMotion) at the correct address, isolating the fault to the wiring segment between the ESP32 and the sensor. | The symptom (`0xE2`, F13) by itself — total silence has multiple candidate causes (§3) and does not by itself indicate wiring over power, transceiver, or configuration. |
| **Sensor configuration issue** | Exp. 4a: an independent master (WitMotion, or address-scan) detects the sensor responding at an address **other than** `0x50`, or only after a vendor-tool unlock/reset sequence not performed by either firmware. | The sensor failing to respond to the ESP32 alone — that is equally consistent with every other category. |
| **RS485 communication issue vs. Sensor hardware failure — disambiguation** | Both categories require Exp. 4; the deciding factor is Exp. 4b (substitution): a **spare** WTVB05 responding normally on the **same, unmodified wiring** proves the fault is the original sensor unit, not the wiring — declare **sensor hardware failure**. If instead reseating/repairing the sensor-end connector (found in a closer re-inspection after Exp. 4a's independent-master success) resolves it without swapping the sensor — declare **RS485 communication issue**. | Exp. 4a alone, without either a spare-sensor substitution or a confirmed physical repair — report as inconclusive between the two rather than guessing. |
| **ESP32 firmware issue** | Exp. 3 shows GPIO42 (EN) stuck HIGH or no TX activity on a transaction attempt, while wiring/sensor are otherwise confirmed intact (Exp. 1, Exp. 2's CTR4A01 result) — OR Exp. 5's failing binary is shown (by comparing the exact FQBN/build flags used) to differ from the validated production build configuration in `CLAUDE.md`, reproducing the known class of silent build-config fault (F6) applied to a different symptom. | A firmware switch having occurred (F11/F12) is not by itself evidence of a firmware *defect* — it is evidence of a *deployment event*, which must still be tested (Exp. 2, Exp. 5) before concluding the firmware itself is at fault. |
| **Software regression** (source-logic defect, as distinct from a build/deployment defect) | Exp. 5 reproduces the failure on hardware that Exp. 2 has just proven physically healthy (both slaves responding), in the same session, with no wiring touched between the two tests — a controlled, immediate A/B comparison. **This is the only category requiring a positive successful communication event (Exp. 2) immediately adjacent to the failure (Exp. 5) on identical physical conditions.** | Exp. 5 failing in isolation, without Exp. 2 having first established the physical layer as clean in the same session — a failure after an untested gap does not rule out a physical explanation for that specific attempt. |

**General rule across all five categories:** no category may be declared from Exp. 1 or Exp. 2 findings alone if a later experiment in the decision tree (§5) was reachable and not run. Each "STOP" leaf in §5 corresponds to exactly one row above; do not stop earlier than the leaf reached.

---

*End of WTVB05_Communication_Regression_Investigation_Plan_v1.0 — planning document only. No experiments executed, no firmware modified, no repository changes made in producing this document.*
