# P4-02 — Communication Health Telemetry: Architecture Review

**Status:** Design only. No firmware modified. Awaiting approval.

## 1. Objective

Expose CURRENT (CTR4A01) communication health to telemetry consumers
(Grafana/MQTT), so that a real communication loss is visible as a distinct
condition rather than being silently absorbed as "current not present" —
without changing firmware behavior, and without duplicating the evidence
lifecycle already implemented and hardware-verified in P4-01
(`v16.5.1-p4.1`).

## 2. Current Telemetry Pipeline (as reviewed)

```
Core 0                                          Core 1
──────                                          ──────
taskModbusRead()                                taskNetwork()
  readCTR4A01Current()                            publishTelemetry(snap)
  -> localData.current_a,                           reads TelemetrySnapshot
     localData.current_valid                        (local copy, no cross-
        (per-poll-cycle flag,                         core access during
         NOT evidence lifecycle)                       JSON build)
        |
        v
taskStateMachine()
  buildMotorStateEvidence()  <-- P4-01 lives here, entirely inside the
    MOTOR_SRC_CURRENT case:       MOTOR_SRC_CURRENT case block.
      s_currentFiltered
      s_currentLatched
      s_currentEvidenceValid  <-- function-local static. Computed every
                                  cycle. NOT visible outside this
                                  function -- no external reader exists
                                  today.
        |
        v
  updateMotorStateMachine(evidence)   <-- unmodified, reacts to
                                          ev.signalPresent/ageMs only
        |
        v
  captureTelemetrySnapshot(data, effectiveState)   <-- Core 0, THE ONE
    snap.rpm / motor_state / health_score /            capture point for
    memcpy(snap.vib, data, ...)                        TelemetrySnapshot.
        |                                              Called once per
        | mutexVibData                                 state-machine cycle,
        v                                              after the FSM update.
  g_telemSnapshot (protected by mutexVibData)
        |
        | (Core 0 -> Core 1 handoff, existing mechanism)
        v
publishTelemetry(snap)  [Core 1]
  PUB 1/3 /sensor    -- raw acquisition + motor context
  PUB 2/3 /status    -- alarm/health/fault-latch (-> g_mqttTopicDecision)
  PUB 3/3 /vibration -- backward-compat Grafana payload; ALREADY carries
                        current_read_errors, current_buf_count,
                        current_slope (the existing home for CTR4A01
                        diagnostics)
```

Key finding: **`s_currentEvidenceValid` has no path out of
`buildMotorStateEvidence()` today.** It is a function-local `static bool`,
confined to the `MOTOR_SRC_CURRENT` case block by design (P4-01 Design
Contract explicitly scoped it that way). Nothing else in the firmware reads
it. Exposing it to telemetry therefore requires *some* new plumbing — the
question this review answers is where that plumbing should live so it adds
the least surface area.

## 3. Can An Existing Field Be Reused? (Requirement 3/5 check)

Two existing fields look superficially related. Both were checked and
ruled out:

| Existing field | Where set | What it actually means | Why it cannot carry evidence validity |
|---|---|---|---|
| `VibrationData_t.current_valid` | `taskModbusRead()`, line ~4405/4411 | "Did a CTR4A01 poll happen *this specific 250ms task tick*?" — true only on the ~500ms cadence tick where a poll was attempted, false on every other tick. Feeds `g_currentBuf[]`/`calcTrend()` sample selection. | Per-tick polling flag, unrelated to the evidence lifecycle. It is `false` most cycles even when communication is perfectly healthy — publishing it as "comm health" would read as constant flapping and misrepresent a healthy sensor as failing most of the time. |
| `current_read_errors` (`g_ctReadErrors`) | `readCTR4A01Current()`, line ~4273 | Cumulative Modbus failure counter since boot. Already on `/vibration`. | Monotonic counter, never resets. Tells you failures have happened historically, not whether communication is healthy *right now*. Cannot distinguish "1 failure 2 hours ago, fine since" from "failing right now." |

Neither field expresses "is the evidence the FSM is currently using
trustworthy." A new field is required. This satisfies the requirement to
justify rather than assume.

## 4. Proposed Design

### 4.1 Principle

Reuse `s_currentEvidenceValid`'s *value* verbatim. Add exactly one
additional write of an already-computed boolean to a wider scope, at the
two existing assignment sites inside the `MOTOR_SRC_CURRENT` case (the
`ageMsNow > CURRENT_EVIDENCE_MAX_AGE_MS` branch and the `isFreshSample`
branch) — **no new timeout, no new counter, no new comparison is
introduced.**

### 4.2 Mechanism (mirrors the existing `g_sensorOffline` pattern exactly)

1. **New single-word global**, same shape as the existing
   `g_sensorOffline` (`static volatile bool`, WTVB02's own proven
   comm-health signal at line ~1760):

   ```
   static volatile bool g_currentEvidenceValid = false;
   ```

   Written only inside `buildMotorStateEvidence()`'s `MOTOR_SRC_CURRENT`
   case, at the same two lines that already assign
   `s_currentEvidenceValid` (2998 and 3011 in current numbering) — a
   trailing `g_currentEvidenceValid = s_currentEvidenceValid;` alongside
   each existing assignment. This is a **side-effect mirror of a value
   already computed**, not a new calculation. `buildMotorStateEvidence()`'s
   signature, return type, and control flow are unchanged; the function's
   behavior as observed by any existing caller is identical.

   Per CLAUDE.md's cross-core rule, a single-word `bool` is safe to read
   from Core 1 without a mutex (same class of variable as
   `g_sensorOffline`, already read across cores today).

2. **`TelemetrySnapshot`** gains one new field, populated at the existing,
   single capture point (`captureTelemetrySnapshot()`, Core 0, called once
   per state-machine cycle — already the mechanism that moves
   `motor_state`/`health_score` across cores):

   ```
   snap.current_evidence_valid = g_currentEvidenceValid;
   ```

   This piggybacks on the mutex (`mutexVibData`) already taken in that
   function — no new synchronization primitive.

3. **`publishTelemetry()`** (Core 1) adds one new field to the `/vibration`
   payload — the topic that already owns every other CTR4A01 diagnostic
   (`current_read_errors`, `current_buf_count`, `current_slope`):

   ```
   doc["current_evidence_valid"] = snap->current_evidence_valid;
   ```

### 4.3 What is explicitly NOT touched

- `buildMotorStateEvidence()` — signature, return value, and existing
  logic unchanged. Two lines added, each a mirror-write of a value the
  function already computes.
- `updateMotorStateMachine()` — untouched, not even read from.
- `MotorStateEvidence` — untouched. Validity is a *telemetry* concern, not
  an FSM input; the FSM continues to react only to `signalPresent`/`ageMs`
  exactly as verified in P4-01.
- `MotorRunState_t` — untouched.
- Every existing MQTT field on every topic — untouched; this is a pure
  addition of one new key to `/vibration`, following the same
  additive-only convention already used for `current_read_errors` and
  `current_buf_count` in v16.6a/b.

## 5. New Field Justification (Requirement 5)

| Field | `current_evidence_valid` |
|---|---|
| **Purpose** | Let telemetry consumers (Grafana/alerting) distinguish "motor genuinely stopped" from "CURRENT evidence is stale/invalid because communication was lost," closing the P4 objective gap without any new decision logic. |
| **Producer** | `buildMotorStateEvidence()`'s `MOTOR_SRC_CURRENT` case, mirroring its existing `s_currentEvidenceValid` computation (P4-01, hardware-verified) — Core 0. |
| **Consumer** | Grafana dashboard / downstream alerting via the `/vibration` MQTT topic; secondarily, Serial `[CURRENT_DECISION]` debug (optional, see §6 alternatives). |
| **Lifetime** | Per state-machine cycle — reflects current live evidence state, not a cumulative/latched value. No persistence (NVS) needed; resets to `false` on boot like `s_currentEvidenceValid` itself. |
| **Why no existing field covers it** | See §3 table — `current_valid` is a per-tick polling flag (flaps every ~500ms by design) and `current_read_errors` is a monotonic historical counter; neither expresses live evidence trustworthiness. |

## 6. Design Alternatives Considered

| # | Alternative | Rejected because |
|---|---|---|
| A | Add validity as a new field on `MotorStateEvidence` itself | Explicitly excluded by your preserve list; also conflates an FSM-input struct with a telemetry concern — P4-01's contract kept the FSM deliberately unaware of *why* evidence is what it is, only *what* it is (`signalPresent`/`ageMs`). |
| B | Derive comm health independently in the telemetry/network layer (e.g. a new age check against `g_lastCurrentSampleMs` in `taskNetwork`) | Violates requirement 3 directly — this would be a second, independent freshness calculation using a different threshold owner, able to drift out of sync with the FSM's own view of validity (the exact class of bug P4-01 was designed to prevent for the FSM itself). |
| C | Reuse `g_ctReadErrors` (error counter) as a proxy for health, e.g. "healthy if no error in the last N seconds" | Requires a new timestamp + comparison — another independent freshness calculation, same objection as B. Also semantically wrong: a counter delta needs a window, which is itself a new "freshness" concept. |
| D | Expose the raw `ageMs`/timestamp instead of (or in addition to) a boolean | More information, but: (a) not requested/needed by the stated objective ("expose health," not "expose staleness magnitude"), (b) invites downstream consumers to invent their *own* threshold against `CURRENT_EVIDENCE_MAX_AGE_MS`, duplicating the exact judgment call `s_currentEvidenceValid` already makes. Rejected in favor of the single boolean; can be added later as its own justified field if a real consumer need appears. |
| **E (recommended)** | Mirror `s_currentEvidenceValid` to a new single-word global, carried through the existing snapshot mechanism, published as one new `/vibration` field | Satisfies reuse (no new calculation), satisfies every "preserve" constraint, follows the codebase's own proven pattern (`g_sensorOffline`) and its own additive-MQTT-field convention. |

## 7. Risks

- **Cross-core read timing**: `g_currentEvidenceValid` is written on Core 0
  and captured into the snapshot on Core 0 in the same cycle
  (`captureTelemetrySnapshot()` runs immediately after
  `updateMotorStateMachine()`, which runs immediately after
  `buildMotorStateEvidence()`), so there is no cross-core race on the
  write side — identical timing guarantee already relied upon for
  `motor_state`/`rpm` in the same snapshot. Low risk.
- **Telemetry lag**: like every other `/vibration` field, this reflects the
  state at the last snapshot capture (one state-machine cycle old at
  most), not the instant of publish. Consistent with existing fields;
  not a new class of staleness.
- **Consumer misinterpretation**: a dashboard could conflate
  `current_evidence_valid=false` with `motor_state=STOPPED`, since both
  can be true simultaneously in a real stop. Mitigation is dashboard-side
  (e.g. only alert "comms lost" when `current_evidence_valid=false` AND
  `current_read_errors` is climbing), not a firmware concern — no
  additional firmware field is warranted to solve a dashboard
  interpretation problem (would reopen alternative D).
- **Backward compatibility**: purely additive JSON key; existing consumers
  ignoring unknown keys are unaffected (same pattern as
  `current_read_errors`/`current_buf_count` additions in v16.6a/b, which
  shipped without incident).
- **Scope creep**: this design deliberately stops at one boolean on one
  topic. CTR4A01 offline detection proper (mirroring
  `g_sensorOffline`/`MODBUS_OFFLINE_THRESHOLD` for the WTVB02 sensor) is
  P4-03's scope, not this one — no overlap, since P4-03 would operate on
  `g_ctReadErrors`/Modbus return codes, an entirely different signal from
  the EMA evidence-validity flag proposed here.

## 8. Recommended Approach

Alternative E, as detailed in §4: one new `static volatile bool
g_currentEvidenceValid`, written as a mirror of the existing P4-01
computation (2 lines, no new logic), carried through the existing
`TelemetrySnapshot` capture mechanism (1 new field, 1 new assignment in
`captureTelemetrySnapshot()`), published as exactly one new key,
`current_evidence_valid`, on the existing `/vibration` topic (1 new line
in `publishTelemetry()`). Total surface: 3 small additive touch points,
zero modification to any preserved function/struct/field, zero new
timeout or validity computation.

---

## 9. Design Refinement Review — Is `g_currentEvidenceValid` Necessary?

**Status:** Design review only, requested before implementation approval.
No firmware modified.

### 9.1 The question

§4's recommended design (Alternative E) introduces a *second*, separately
named variable (`g_currentEvidenceValid`) that mirrors the value already
held in `s_currentEvidenceValid`. Two designs are compared:

- **Option A** — keep `s_currentEvidenceValid` exactly as P4-01 left it
  (function-local static, confined to the `MOTOR_SRC_CURRENT` case). Add a
  *new*, separately-named global `g_currentEvidenceValid`, written as a
  mirror at the same two assignment sites. `captureTelemetrySnapshot()`
  reads the new global.
- **Option B** — no second variable. Promote `s_currentEvidenceValid`
  itself out of the function-local/case-block scope to file scope (still
  written only inside `buildMotorStateEvidence()`'s `MOTOR_SRC_CURRENT`
  case), so `captureTelemetrySnapshot()` reads that *same* storage
  directly. No mirror-write, no second name.

### 9.2 Comparison

| Axis | Option A (mirror global) | Option B (direct read, promoted scope) |
|---|---|---|
| **Ownership** | Two variables, one producer. `s_currentEvidenceValid` remains the "real" internal state; `g_currentEvidenceValid` is a second symbol claiming to mean the same thing. Nothing but code discipline keeps them in agreement. | One variable, one producer, one name. Single source of truth — the same principle P3-01 already established for `CURRENT_DIAG`/`CURRENT_DECISION` (eliminate duplicated fields, not create new ones). |
| **Coupling** | `captureTelemetrySnapshot()` couples to a well-named, intentionally-published global — it does not need to know `buildMotorStateEvidence()`'s internal structure at all. This is the same coupling shape as the existing `g_sensorOffline` → `anaSensorHealthy()` relationship (a deliberate "publish point"). | `captureTelemetrySnapshot()` couples to a symbol whose conceptual home is *inside* the `MOTOR_SRC_CURRENT` case of `buildMotorStateEvidence()`. Reading it "directly" only works today because exactly one source is active; if evidence-validity telemetry is ever needed for RPM/PROXIMITY too, `captureTelemetrySnapshot()` would need to know *which* per-source static to read for the currently-active `g_motorStateSource` — i.e. it would need to re-implement the source dispatch that `buildMotorStateEvidence()`'s switch already owns. That pushes source-awareness into a function that was deliberately kept source-agnostic (mirroring why `MotorStateEvidence` exists at all: to keep `updateMotorStateMachine()` source-agnostic). |
| **Lifetime** | Static-storage-duration global, same lifetime characteristics as any other file-scope static in this file. Identical in both options — not a differentiator. | Identical to Option A. |
| **Thread safety** | Producer (`buildMotorStateEvidence()`, called from `taskStateMachine`) and consumer (`captureTelemetrySnapshot()`, also called from `taskStateMachine`, immediately after) both run on **Core 0, same task, same cycle, sequentially**. No cross-core race exists at this hand-off point in *either* option — the real Core0→Core1 boundary is `g_telemSnapshot`, already protected by `mutexVibData` identically in both designs. Marking the variable `volatile` (as §4 proposed, mirroring `g_sensorOffline`) is not actually required for correctness here, since `g_sensorOffline` needs `volatile` because it *is* read from a different task (`anaSensorHealthy()`, consumed on Core 1 in `taskAnalytics`) — this new variable is not. This applies equally to A and B; not a differentiator, but worth correcting in the implementation either way (a plain `static bool` suffices; `volatile` was over-mirrored from a precedent that doesn't structurally apply here). |
| **Core0/Core1 interaction** | No new interaction — identical to Option B. The variable itself never crosses cores; only `g_telemSnapshot` does, unchanged in both options. | Identical to Option A. |
| **Maintainability** | One extra global name in a file that already has a large global namespace (a cost the P1 hygiene pass specifically pushed back against). Risk: the two variables silently drifting out of sync if a future edit updates one assignment site but not its mirror — a class of bug that is easy to introduce and easy to miss in review, since both compile fine independently. | No duplicate-assignment risk (only one place to update). Cost: obtaining this requires *moving* an existing declaration line out of `buildMotorStateEvidence()` — a structural edit to code that was the exact subject of P4-01's hardware fault-injection verification (`v16.5.1-p4.1`). The value/semantics do not change, but the function's own text does, which does not fit "preserve `buildMotorStateEvidence()`" as cleanly as a pure addition would, and reopens (even if only cosmetically) code that was deliberately signed off as verified and frozen. |
| **Future scalability** | Extending to a second source (e.g. RPM, which already has an analogous `RPMEvidence.valid`) is a direct copy of the same pattern: a new `g_rpmEvidenceValid` mirror, written at RPM's own evidence site, read the same uniform way. `captureTelemetrySnapshot()` needs no new knowledge to support additional sources. | Extending to a second source requires `captureTelemetrySnapshot()` (or its caller) to either (a) grow a switch on `g_motorStateSource` to pick the right promoted static, duplicating dispatch logic that already exists once in `buildMotorStateEvidence()`, or (b) promote a differently-named static per source and hope nothing collides — neither is as uniform as Option A's "just add another mirror global" extension path. |

### 9.3 Recommendation

**Option A**, with one correction: declare the new global as a plain
`static bool` (not `volatile`) — `volatile` was mirrored from
`g_sensorOffline` by analogy, but that analogy does not hold structurally
here, since (per §9.2 Thread Safety) the producer and the only consumer
both run on Core 0 in the same task, sequentially, within the same
`taskStateMachine` cycle. `volatile` is harmless but not justified by an
actual cross-core read, so it should not be added on the strength of a
precedent that does not apply.

**Why not Option B**, despite being the "purer" single-source-of-truth
choice on paper:

1. It requires structurally editing code inside
   `buildMotorStateEvidence()`'s `MOTOR_SRC_CURRENT` case — the exact code
   whose behavior was hardware fault-injection-verified in P4-01
   (`v16.5.1-p4.1`). Option A is a pure line-addition superset of that
   verified code; nothing already verified is touched, moved, or
   restructured.
2. It would make `captureTelemetrySnapshot()` — deliberately a
   source-agnostic, "pure data movement" function today — either grow
   source-dispatch knowledge or accept a scalability ceiling of exactly
   one source. That cuts against the same architectural principle that
   justified `MotorStateEvidence` existing at all (keeping
   `updateMotorStateMachine()` source-agnostic); Option B would reintroduce
   the same kind of source-coupling one layer up, in telemetry capture.
3. The duplication cost Option A pays (two variables that must be kept in
   sync) is small, mechanical, and low-risk in practice: both assignment
   sites are three lines apart in the same case block, changed together
   in one diff, and never touched again after this implementation. The
   duplicated-single-source-of-truth concern that motivated P3-01 was
   about *measurement data* (`rawA`/`engineeringA`) diverging across two
   independently-timed print sites — a materially different risk shape
   than two co-located assignments written in the same edit.

Net: Option A trades a small, contained, easily-reviewed duplication for
(a) leaving verified P4-01 code untouched and (b) preserving the
source-agnostic property of the capture layer — both outweigh the
single-source-of-truth purity Option B would offer.

---

Stopping here per instruction. Waiting for approval before any
implementation.
