# Integration Analysis Report
**Base firmware:** `New folder\WTVB02_ESP32S3_V16_CM.ino`
**Patch:** `New folder\fault_latch_v3_code.ino` ("FAULT LATCH v3")
**Mode:** read-only analysis, no files modified.

---

## 1. Line counts

| File | Lines |
|---|---|
| `WTVB02_ESP32S3_V16_CM.ino` | 5,081 (5,082 incl. trailing blank) — note: `Get-Content \| Measure-Object -Line` undercounts this file at 4,408 due to PowerShell's default encoding mis-handling the embedded Thai comments; raw byte/text split confirms 5,081 |
| `fault_latch_v3_code.ino` | 441 |

The patch file is **not a standalone sketch** — it is an insertion/replacement spec (14 sections, several wrapped in `/* */` as "find this block, replace with this") intended to be hand-applied to the base file. It will not compile on its own (references base-file types/globals/functions it doesn't define) and is not meant to.

---

## 2. Integration points

All 6 anchor points the patch claims were independently verified against the base file and **match exactly**:

| Section | Anchor claimed | Verified at | Match |
|---|---|---|---|
| 1 (constants) | after `#define NVS_SAVE_INTERVAL_MS 30000UL` | line 147 | ✅ exact |
| 4 (`faultSeverity()`) | after closing brace of `saveRuntimeHour()` | line 1434 | ✅ exact |
| 10 (`setup()`) | after `loadRuntimeHour();` call | line 4779 | ✅ exact |
| 11 (`taskStateMachine()` tail) | 5-line block ending the function | lines 2614–2618, byte-for-byte identical to patch's "Find" block | ✅ exact |
| 12 (`taskNetwork()` replay) | before `// -- 100ms sleep --` comment, "~line 3035" | line 3035 | ✅ exact |
| 13 (`publishTelemetry()` /status) | "PUBLISH 2 of 3 — /status" block | lines 3994–4037, field-for-field identical to patch's "Find" block before the v3 additions | ✅ exact |
| Mandatory fix | `static uint8_t g_bearingStableCnt = 0;` "Line ~1399" | line 1399 | ✅ exact |

This is a clean, well-targeted patch — every insertion point was checked against current line numbers rather than assumed.

---

## 3. Symbol conflicts

Searched the base file for every new identifier the patch introduces (functions, globals, macros): `FaultLatch_t`, `g_fl`, `g_flCount`, `g_flPrevBearing`, `g_flPrevHealthLow`, `g_flPrevState`, `faultSeverity`, `faultEventStr`, `saveFaultLatchNVS`, `loadFaultLatchNVS`, `clearFaultLatchNVS`, `checkAndLatchFault`.

**No collisions found.** All are new names. Function insertion point (after `saveRuntimeHour()`, before `updateRuntimeHour()`) sits ahead of every call site (`taskStateMachine` ~2614+, `setup()` ~4779), and Arduino's IDE auto-prototyping makes the precise textual order non-load-bearing anyway.

NVS namespace `"fault_latch"` (`FL_NS`) does not collide with the existing `"motor_nvs"` (runtime-hour) or `"boot"` (reboot counter) namespaces — confirmed via search.

---

## 4. Enum conflicts

| Patch macro group | Base enum | Conflict? |
|---|---|---|
| `FL_EVT_NONE..FL_EVT_MAX_VALID` (0–4, `#define`) | `MachineState_t { STATE_NORMAL=0, STATE_WARNING, STATE_CRITICAL, STATE_MAINTENANCE, STATE_WARMUP }` | No — different namespace (macro vs. enum), no shared identifier text, values only coincidentally overlap (0–4) and are never compared against each other in the patch |
| `FL_SEV_NONE..FL_SEV_BEARING` (0–4, `#define`) | `MotorRunState_t { MOTOR_STOPPED=0..MOTOR_STOPPING=3 }` | No identifier overlap |

No enum identifiers are redefined. The patch deliberately uses `#define` constants rather than a real `enum`, so there's no risk of a `typedef`/tag clash with `MachineState_t` or `MotorRunState_t`. One soft note: mixing raw `#define` severities with the existing `enum`-based state machine is a style inconsistency, not a compile risk.

---

## 5. Struct conflicts

`FaultLatch_t` (new) does not collide with any existing struct/typedef (`VibrationData_t`, `SystemState_t`, `NetworkStatus_t`, `TimeSyncStatus_t`, etc.) — no name reuse, no field-name reuse (`code`, `pending`, `rms`, `kurtosis`, `ts` don't appear as fields in any base struct).

Fields the patch *reads* from existing structs were checked against the real definitions (lines ~1004–1046 `VibrationData_t`, ~1049–1056 `SystemState_t`):

- `data->rms_overall`, `data->kurtosis_max` — present ✅
- `sensorData.motor_state`, `sensorData.rms_overall`, `sensorData.kurtosis_max` — present ✅
- `g_systemState.state` (`MachineState_t`) — present ✅

All good — no struct conflicts, no missing-field risk.

---

## 6. Compile risks

**Low overall — the patch is internally consistent and was clearly written against this exact source.** Specific checks:

- **`Preferences`/`ArduinoJson` includes**: both already `#include`d in the base file (lines 45, 47) — no new includes needed.
- **`DateTime(uint32_t)` constructor** used in Section 12 (`DateTime flEv(g_fl.ts)`) — confirmed valid; base file already uses this exact constructor form at line 2060 (`dt = DateTime(unixUTC);`).
- **`mqttClient.publish(topic, payload, len, retain, qos)` 5-arg form** used in Sections 12/13 — matches the exact call signature used elsewhere in the base file (e.g. line 2982), so no signature mismatch.
- **`goto fl_discard;` in `loadFaultLatchNVS()`**: jumps forward past only plain-old-data locals (`uint8_t`/`uint32_t`/`float`) that are already initialized before any `goto` executes, and no new variable is declared between any `goto` and the `fl_discard:` label. This is legal C++ — no "jump bypasses initialization" error.
- **Mandatory fix dependency**: Section 11 (`checkAndLatchFault`) reads `g_bearingStableCnt` from `taskStateMachine` (Core 0), while `publishTelemetry()` (Core 1, called from `taskNetwork`) increments/reads it at line 3846, and a Core 1 button handler resets it at line 3175. This is a **genuine pre-existing cross-core shared variable** — the patch's required `volatile` fix at line 1399 is real and necessary, not speculative. Confirmed via full usage search.
- **`enum` comparisons using raw literals**: patch's Section 11 uses `sensorData.motor_state == 2` — consistent with the base file's own style (e.g. line 3844 uses the same literal `2` instead of `MOTOR_RUNNING`), so no new inconsistency.
- **JSON capacity claims (Section 14)**: arithmetic wasn't independently re-derived field-by-field, but buffer sizes (`StaticJsonDocument<768>`/`<1024>`, `char buf[768]`/`[1024]`) are generous relative to the field counts added; low risk of truncation.

**One non-blocking runtime (not compile) risk worth flagging:** `g_fl` (`FaultLatch_t`) is written by Core 0 (`checkAndLatchFault`, inside `taskStateMachine`) and read by Core 1 (`taskNetwork`'s replay block, `publishTelemetry`'s `/status` block) with **no mutex** — only `pending` is `volatile`; `ts`, `rms`, `kurtosis`, `code` are not. The patch mitigates the worst case by setting `pending = true` *last*, after the NVS write, so a half-updated struct shouldn't be read as valid — but a reader could still observe a brief window of inconsistent `ts`/`rms`/`kurtosis`/`code` while `pending` is already `true` mid-assignment on a future overwrite. Not a compiler error; worth a design call before merging.

---

## Summary

No symbol, enum, or struct conflicts. All 7 integration anchors verified byte-exact against the current base file. The only actionable items are the already-flagged mandatory `volatile` fix (real and justified) and the unguarded cross-core read of the rest of `FaultLatch_t` (a runtime race, not a build blocker).

Waiting for approval before applying any edits.
