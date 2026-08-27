# Phase E — Behavioral Alarm Test Plan (Pump01)

**Status: PLANNED ONLY — NO TEST HAS BEEN EXECUTED.**
This is a read-only planning document. No firmware change, build, flash, checkout,
commit, service restart, fake MQTT publish, threshold change, configuration change,
or software-simulated vibration has been performed to produce this document, and
none of those actions are authorized by this document.

---

## 0. Source-Code Audit (required before proposing any trigger method)

### 0.1 Is there any way to change `velocity_rms_overall` without modifying firmware?

Searched the full production `.ino` and its companion files
(`fifo_arena.*`, `fifo_codec.*`, `fifo_driver.*`, `fifo_session.*`,
`fifo_transport*.*`, `vib_ema.h`, `vib_ttw.*`) for any debug flag, NVS-configurable
override, simulation hook, or test-injection path that could alter the reported
velocity or thresholds without recompiling firmware:

- No `simulate`/`fake`/`inject`/`mock`/`override` hook exists anywhere in the
  production `.ino` that touches vibration, velocity, or alarm state. The only
  matches for that pattern are `DEBUG_MODEM_DIAG` (modem diagnostics only,
  `#define`d **off** at `.ino:177`, and even if enabled it does not touch
  vibration/velocity) and unrelated `override` keywords on C++ virtual method
  signatures (TinyGSM client wrapper, `.ino:1024-1254`).
- No NVS/Preferences key exists that reads or writes a velocity or threshold
  value (`grep` for `nvs`/`preferences.get/put*(...vel|vib|test|thresh...)`
  returned zero matches). `VIB_WARNING_MMS`, `VIB_WARNING_OFF_MMS`,
  `VIB_CRITICAL_MMS`, `VIB_CRITICAL_OFF_MMS`, `VIB_ALARM_PERSIST_CAPTURES`
  are compile-time `#define` constants (`.ino:776-802`) — changing them requires
  editing source and rebuilding, both explicitly prohibited for this task.
- The physical buttons (`PIN_BUTTON` / `PIN_BUTTON_ENTER`, `.ino:333-334`) only
  do page navigation, alarm-acknowledge, and an `OPERATOR_BUTTON` **FIFO trigger**
  (forces an extra real sensor capture on demand) or a maintenance-reset event.
  None of these inject a fabricated velocity value — an `OPERATOR_BUTTON` trigger
  still reads the real WTVB02 accelerometer over Modbus RTU/RS485 and runs it
  through the same DSP path as any other capture.
- `velocity_rms_overall` is produced exclusively by
  `VibVelocity_ComputeRms(g_accelWork.x/y/z, sampleCount, srHz, &vel)` inside
  `processPendingAccelSnapshot()` (`.ino:9855-9921`), fed by real FIFO accelerometer
  samples pulled from the WTVB02 sensor. There is no code path that assigns this
  value from a constant, a debug command, or an MQTT-received value.

**Conclusion (VERIFIED BY SOURCE):** there is no firmware-resident, config-resident,
or protocol-level way to change what Product-1 reports for `velocity_rms_overall`
except by physically changing the real vibration present at the WTVB02 sensor's
mounting point on the running pump. Any method that does not do that is either
(a) firmware/config tampering (prohibited by this task) or (b) not a real test of
production behavior at all.

### 0.2 What physically changes real vibration on a running pump?

Only two physical categories exist:
1. **Operating-condition changes within the pump's normal design envelope**
   (e.g., flow/throttle point, if the installation allows it) — not a fault,
   potentially reversible, but its effect on `velocity_rms_overall` is unknown
   and must not be assumed (see instruction #4 — no guessing that this reaches
   2.1 or 4.5 mm/s).
2. **Mechanical fault conditions** (imbalance, looseness, misalignment, bearing
   damage, overload beyond rating, cavitation, blocked cooling, etc.) — these are
   the dominant real-world causes of the vibration magnitudes WARNING/CRITICAL are
   calibrated for (2.1 / 4.5 mm/s is 7–16× the currently-observed healthy baseline
   of ~0.25–0.29 mm/s, per the MQTT evidence already on record). Deliberately
   inducing any of these is explicitly prohibited by this task without an
   approved engineering procedure, and no such procedure/approval exists in this
   conversation or repository.

No source-code finding changes this: the gap between "safe" and "reaches
threshold" is a mechanical/process fact about Pump01, not something the firmware
source can answer, and I am not going to guess it.

---

## A. Safe Test Method(s)

Ranked by how much of Phase E they can actually close, given §0's findings.

### A1. Passive/opportunistic observation — safe, executable now, no approval needed
Do not induce anything. Simply watch Pump01's real MQTT `/vibration` stream
continuously (as already done for the MQTT evidence on record) and log any
**naturally occurring** excursions — motor start-up transient, brief process
upsets, etc. — that happen to cross 2.1 or 4.5 mm/s on their own.
- Pros: zero risk, zero approval needed, genuinely real production data.
- Cons: not guaranteed to ever produce a WARNING/CRITICAL crossing on a healthy
  pump; cannot be scheduled; may never close Phase E within a useful timeframe.
- Use: run continuously as a background watch; if it fires, capture per §D below.
  If it never fires, it still fully validates the **STATE 1 — NORMAL** row and the
  "stays NORMAL under real healthy operation" negative case.

### A2. Within-rated-envelope operating-condition change — conditionally safe, requires plant/process sign-off
If Pump01's installation has a normal, designed operating range (e.g., a throttle
valve, a variable process setpoint) that plant/process engineering confirms is
**within the pump's rated envelope and normal operating procedure** (i.e., not an
induced fault, not exceeding nameplate rating), that change could be exercised as
a real-condition test.
- This requires **written plant/process engineering approval** before execution —
  none exists yet in this conversation.
- Whether it reaches 2.1 or 4.5 mm/s is **unknown and not assumed** — it may not,
  in which case Phase E stays partially unclosed even after doing it safely.
- Not proposed for execution here; listed only as the one category of real,
  non-fault physical change that could in principle be evaluated, pending
  approval this document does not have.

### A3. Bench / test rig or hardware vibration simulator — safe, fully controllable, but is NOT production validation
A separate WTVB02 unit (same sensor model/firmware build) mounted on a bench
shaker, unbalanced test rotor, or hardware vibration simulator can be driven
through the full 0 → 2.1 → 4.5 → 4.2 → 1.9 → 0 mm/s range under full operator
control, with no risk to Pump01.
- This is the only method that can safely and repeatably exercise **every** row
  in §C (including CRITICAL) without touching production equipment.
- **Hard constraint (per this task's own instruction G): results from a bench rig
  must never be recorded or reported as "Phase E PASS" or "production validation."**
  They validate firmware alarm logic in isolation from Pump01's real mechanical
  condition. They would go in a *separate* document, explicitly labeled
  "bench-validated firmware behavior — not a Pump01 production test," and Phase E
  in the Product-1 E2E record stays `NOT VERIFIED` until a real Pump01 production
  observation exists.

---

## B. Unsafe / Prohibited Methods (explicitly ruled out)

- Loosening, shimming, or otherwise deliberately altering the WTVB02 sensor's
  mounting/bracket to change measurement-path coupling — this both simulates a
  real fault condition (mounting looseness) without approval, and does not
  represent genuine machine health, so it would corrupt the evidence even if it
  "worked."
- Deliberately inducing rotor/shaft imbalance (e.g., temporary added mass),
  misalignment, bearing pre-load change, or any other intentional fault — explicitly
  prohibited without an approved engineering procedure (none exists here).
- Running the motor beyond its rated load/current to force higher vibration —
  explicitly prohibited (overload) without approval.
- Manipulating a valve/process point beyond the pump's normal design envelope
  (as opposed to A2's within-envelope case) to induce cavitation or surge.
- Blocking cooling, restricting flow, or any action whose primary purpose is to
  stress the machine into an abnormal condition.
- Physically disconnecting/relocating the sensor by hand to read a different
  vibration source (e.g., holding it against an unrelated vibrating object) —
  this produces a "real" reading but not a real Pump01 condition; it would
  falsify the evidence chain, not test it.
- Any software or protocol-level fake: publishing a crafted MQTT `/vibration`
  message, writing a fake Modbus register reply, editing firmware constants,
  or otherwise injecting a value the sensor did not actually measure — already
  prohibited by this task, and confirmed in §0.1 that no legitimate code path
  for this exists anyway.

**If none of A1/A2/A3 is exercised, Phase E's WARNING/CRITICAL transitions
(STATE 2–5) remain `NOT EXECUTABLE SAFELY in the current production environment`,
per this task's own instruction #5.** STATE 1 (NORMAL) can still be fully
evidenced today via A1, and is not blocked by this conclusion.

---

## C. Exact Observation Sequence (design only — for whichever platform, A2 or A3, is eventually approved/available)

Applies identically whether the eventual trigger is an approved A2 production
change or an A3 bench rig — only the label on the resulting evidence differs
(production vs. bench-only, per §A3's hard constraint).

| Step | Action | Wait-for | Real capture cadence driving this |
|---|---|---|---|
| 0 | Baseline: confirm `alarm_level=NORMAL`, `motor_state=2` (RUNNING), `velocity_data_valid=true` for ≥ 3 consecutive MQTT `/vibration` messages before touching anything | steady NORMAL | MQTT publish cadence in NORMAL = **30 s** (alarm-driven, `.ino:174`) |
| 1 | STATE 1 — NORMAL (already the baseline) | n/a | — |
| 2 | Begin real-condition change (A2/A3) toward WARNING | `velocity_rms_overall` crosses **≥ 2.1 mm/s** (`VIB_WARNING_MMS`, `.ino:7231`) | Underlying FIFO capture/decision cadence = **2 s** (`FIFO_PERIODIC_INTERVAL_MS`, `.ino:2173`) — this is what "captures" means for persistence, *not* the MQTT publish interval |
| 3 | STATE 2 — WARNING ENTRY | 2 distinct FIFO captures both `≥ 2.1` while still `< 4.5` (persistence, `.ino:7249`, `VIB_ALARM_PERSIST_CAPTURES=2`) | Escalation is **not immediate** — 1st qualifying capture only sets `s_vibPendCount=1` and holds NORMAL; state flips to WARNING only on the 2nd distinct qualifying capture (`.ino:7240-7250`) |
| 4 | Continue real-condition change toward CRITICAL | `velocity_rms_overall` crosses **≥ 4.5 mm/s** (`VIB_CRITICAL_MMS`, `.ino:7226`) | Once in WARNING, MQTT publish cadence changes to **10 s** (`.ino:174`) |
| 5 | STATE 3 — CRITICAL ENTRY | 2 distinct FIFO captures both `≥ 4.5` (same persistence rule, evaluated from the WARNING branch at `.ino:7226`) | MQTT publish cadence in CRITICAL = **5 s** |
| 6 | Begin reversing the real-condition change | `velocity_rms_overall` drops **below 4.2 mm/s** (`VIB_CRITICAL_OFF_MMS`, strict `<`, `.ino:7223`) | De-escalation is **immediate, no persistence** — `cand <= prevVib` applies on the very next capture (`.ino:7235-7239`) |
| 7 | STATE 4 — CRITICAL EXIT | Exactly one capture `< 4.2` → CRITICAL → WARNING on that same capture | — |
| 8 | Continue reversing | `velocity_rms_overall` drops **below 1.9 mm/s** (`VIB_WARNING_OFF_MMS`, strict `<`, `.ino:7222`/`7227`) | Immediate, no persistence |
| 9 | STATE 5 — WARNING EXIT | Exactly one capture `< 1.9` → WARNING → NORMAL on that same capture | MQTT publish cadence returns to 30 s once NORMAL is re-entered |

**Boundary values, exactly as the comparators read in source (do not
reinterpret):**
- `>= 2.1` and `>= 4.5` are **inclusive** on the ON edges (exactly-at-threshold
  counts as a qualifying capture) — `.ino:7226, 7230-7231`.
- `< 4.2` and `< 1.9` are **strict** on the OFF edges — a capture reading
  **exactly** 4.2 or exactly 1.9 does **not** de-escalate; the value must go
  strictly below (e.g., 4.19 mm/s, 1.89 mm/s) — `.ino:7222-7223, 7227`.

---

## D. Required Evidence Per Transition

For **every** state entry (STATE 1–5), the same evidence bundle applies (matches
the classification standard already used in
`claude/docs/evidence/PRODUCT1_E2E_VALIDATION_RECORD_Pump01_20260827.md`):

1. **MQTT `/vibration` raw** — full raw terminal capture (prompt + `mosquitto_sub`
   command + payload), not a values summary.
2. **Firmware serial log line**, if a console/USB capture is available in
   parallel: `[CORE 0] State: %d -> %d (vel: %.3f mm/s, legacyRMS: %.2f)`
   (`.ino:7288-7289`) — this is the ground-truth transition moment and FIFO
   capture cadence, independent of MQTT publish timing.
3. **Timestamp** — both the payload `timestamp` field (device RTC/NTP) and the
   operator's own `date -Is` at capture time.
4. `velocity_rms_overall`, `velocity_rms_x/y/z` — raw values at the transition
   and the 1–2 captures immediately before/after it (to show the persistence or
   immediate-clear behavior directly, not just the end state).
5. `vibration_status`, `vibration_data_valid`(sic: `velocity_data_valid`),
   `vibration_source`/`vibration_source_legacy`.
6. `alarm_level` (and, once available from the API, `alarm_level_live`).
7. `motor_state` (must remain `2`/RUNNING throughout — a motor stop mid-test
   forces `newState = STATE_NORMAL` directly, bypassing hysteresis entirely,
   `.ino:7179-7182` — this would invalidate that specific transition's evidence).
8. **API raw JSON** (`GET /api/machine/plant01/pump01`), captured within the
   same observation window as the MQTT message for that transition.
9. **Machine Detail UI** screenshot or literal displayed value, same window.
10. **InfluxDB raw query output** (not a values summary) covering the same
    timestamp window, to confirm the transition is durably stored, not just
    seen in-flight on MQTT.

**Acceptance requires all four layers (Firmware decision ↕ MQTT ↕ API ↕ UI) to
agree within the same observation window** for a transition to count as evidenced
— partial evidence (e.g., MQTT only) documents that layer alone, not the
cross-layer chain.

---

## E. PASS Criteria

A transition is **PASS** only if all of the following hold, each backed by raw
evidence per §D:

- The transition occurs at the correct boundary and direction per §C's exact
  comparators (no reinterpretation of `>=` vs `>`, `<` vs `<=`).
- Escalations (STATE 2, STATE 3) show the 2-distinct-capture persistence
  behavior — i.e., evidence exists of at least one qualifying-but-not-yet-escalated
  capture immediately before the transition capture, OR an explicit acknowledgment
  that this sub-behavior could not be captured (do not silently assume it held).
- De-escalations (STATE 4, STATE 5) show the transition on the very first
  capture below the OFF threshold, with no lag — any observed lag is a FAIL
  against source, not a tolerance to relax.
- `alarm_level` in MQTT, `alarm_level`/`alarm_level_live` in API, and the UI
  display are mutually consistent for that same window.
- `motor_state` stayed `2` (RUNNING) throughout the transition window.
- The whole sequence NORMAL→WARNING→CRITICAL→WARNING→NORMAL is demonstrated
  end-to-end, not just individual thresholds in isolation.

**Overall Phase E may only be marked PASS in the Product-1 E2E record once every
row in §C has raw evidence meeting the above — partial completion stays
`NOT VERIFIED` or, if explicitly noted, `PARTIAL`, never `PASS`.**

---

## F. Abort Criteria

Abort immediately, restore Pump01 to its normal operating condition, and record
the abort (not a fabricated result) if any of the following occur:

- `velocity_rms_overall` rises faster or higher than expected during a
  within-envelope change (A2) — do not wait for it to "settle."
- Any audible/visible abnormal machine behavior (unusual noise, smell, heat,
  leakage, current spike beyond rating) unrelated to the vibration reading itself.
- `current_a`/`current_valid` shows the motor drawing outside its rated range.
- `motor_state` drops out of RUNNING unexpectedly (forces an immediate
  uncontrolled NORMAL reset per `.ino:7179-7182`, discarding in-progress
  persistence state — the test run for that attempt is void, not evidence of
  anything).
- `velocity_data_valid`/`vibration_status` goes false/stale mid-test (per
  `.ino:7185-7202`, the alarm state simply holds — this is not a transition and
  must not be recorded as one).
- Any plant/process engineering stop-work instruction, or the operator's own
  judgment that continuing is unsafe, for any reason — abort takes priority over
  completing the observation matrix.
- CRITICAL is reached and the operator has any doubt about safely reversing the
  condition — do not wait for the "natural" reversal if a faster safe shutdown
  path exists.

An aborted run is documented as **ABORTED**, with whatever partial evidence was
captured up to the abort point kept and classified honestly (e.g., "STATE 1–2
evidenced, abort before STATE 3") — never rounded up to PASS.

---

## G. If Production Testing Is Not Safe: Alternative

Per §A3: a bench rig or hardware vibration simulator running the same WTVB02
sensor/firmware build is the recommended fallback to validate the firmware's
alarm state machine logic (hysteresis, persistence, boundary comparators) in
full, safely, on demand.

**Hard rule carried forward into any future document:** bench/simulator results
must be labeled exactly as that — e.g., "Bench-rig firmware alarm-logic
validation, YYYY-MM-DD, not a Pump01 production observation" — and must **never**
be written into `PRODUCT1_E2E_VALIDATION_RECORD_Pump01_20260827.md` §15 (Alarm
Behavioral Evidence) as if it were a Pump01 production result. Phase E in that
document stays `NOT VERIFIED` (or `PARTIAL` if A1/A2 production evidence
eventually arrives) regardless of how much bench-rig evidence exists, until a
genuine Pump01 production observation closes it.

---

## Summary

- **§0 finding:** no software/config backdoor exists to fake `velocity_rms_overall`
  or thresholds — confirmed by source audit, not assumed.
- **A1** (passive observation) is safe and executable today, but not guaranteed
  to produce WARNING/CRITICAL.
- **A2** (within-envelope operating change) is conditionally safe but requires
  plant/process engineering approval that does not currently exist, and its
  effect on vibration magnitude is explicitly not assumed.
- **A3** (bench rig) is the only method that can safely close the full C–F
  matrix on demand, but its results can never be called Pump01 production
  validation.
- Full observation sequence, evidence requirements, PASS criteria, and abort
  criteria are designed (§C–F) and ready to execute on whichever safe platform
  becomes available — **none of it has been executed.**

**PHASE E STATUS = PLANNED ONLY. No test has been run. Pump01 has not been
touched. No document has been marked PASS.**
