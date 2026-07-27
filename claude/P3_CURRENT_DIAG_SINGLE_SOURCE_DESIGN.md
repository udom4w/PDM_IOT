# P3-01 — CURRENT_DIAG Single Source of Truth (Design)

Design phase only. No firmware modified.

## 1. Objective

- Eliminate ambiguity in `MOTOR_SRC_CURRENT` diagnostics: a technician or
  maintainer reading Serial output must never be able to pair a `rawA`
  and an `engineeringA` value that came from different sampling instants.
- Preserve existing motor-state behavior exactly: `updateMotorStateMachine()`,
  `buildMotorStateEvidence()`'s EMA/hysteresis/threshold/`ageMs` logic,
  timing, and all MQTT/JSON telemetry payloads are unaffected. This is a
  diagnostics-only redesign.

## 2. Current Architecture

**Measurement-layer diagnostics** — `readCTR4A01Current()`, inside the
`if (ok)` success branch, gated by its own static `s_lastMeasDiagMs`
(max 1 print/sec, only on a successful CT poll). Prints `rawA`,
`engineeringA`, `ctTurns`, `ctRatioPrimaryA`, `ctRatioSecondaryA`,
`scale` — all computed in this one function call from one `currentMa`
read, so `rawA`/`engineeringA` are atomic and same-sample by
construction.

**Decision-layer diagnostics** — `buildMotorStateEvidence()`'s
`MOTOR_SRC_CURRENT` branch, gated by its own separate static
`s_lastDiagMs` (also max 1 print/sec, but fires every `processRPM()`
cycle, ~250ms, regardless of whether that cycle's CT poll succeeded).
Prints `engineeringA`, `threshold`, `signalPresent`, `raw_current`
(mislabeled — actually `engineeringCurrentA`, not a true raw value),
`ema_current`, `signal_present`, `current_threshold_off`, `motor_state`,
`source`.

**Execution scope is asymmetric between the two blocks — this is
existing behavior, unchanged by this proposal:**
- The measurement-layer block executes whenever `DEBUG_CURRENT_PATH` is
  defined, **independently of `g_motorStateSource`** — the CTR4A01 is
  polled on its own cadence regardless of which source drives the FSM.
- The decision-layer block only exists at all when `g_motorStateSource
  == MOTOR_SRC_CURRENT`, since it is nested inside that specific
  `switch` case in `buildMotorStateEvidence()`.
- Consequence: in a build where `MOTOR_SRC_RPM` is active, the
  measurement layer still prints `[CURRENT_DIAG]` (the CT sensor is
  still polled), while the decision layer's block never prints at all.
  This proposal does not change this — it only changes what the
  decision-layer block contains on the occasions it *does* execute.

**Data ownership — two categories, kept distinct throughout this
document:**
- **Measurement-owned data**: values computed and held by the
  measurement layer — `rawCurrentA`/`engineeringCurrentA` (locals,
  ephemeral per call), `g_lastCurrentSampleMs` (global, written only
  here, on success).
- **Decision-derived values**: values computed *by* the decision layer,
  *from* measurement-owned inputs — e.g. `evidence.ageMs`, which the
  decision layer computes as `millis() - g_lastCurrentSampleMs`. This is
  not measurement-layer state being passed through; it is a value the
  decision layer derives from a measurement-layer timestamp. The
  decision layer also owns `s_currentFiltered`/`s_currentLatched`
  (function-local statics, EMA + hysteresis state) and the returned
  `MotorStateEvidence` fields, none of which have a measurement-layer
  origin at all.
- A third, distinct component — the **acquisition/queue layer**
  (`taskModbusRead()` populating `VibrationData_t.current_a`/
  `current_valid`, dequeued later by `taskStateMachine()`) — sits
  between the two. The decision layer's `currentA` parameter is this
  queued value, not a value it computed itself, and it may be one or
  more cycles stale if a poll was skipped or failed.

**Timing ownership:** the two print gates (`s_lastMeasDiagMs`,
`s_lastDiagMs`) are independent, unsynchronized static timers with no
shared reference point. Under steady-state current they usually agree;
under a changing load or a poll gap, they can diverge.

## 3. Problems

- **Duplicate `engineeringA`** — printed by both layers, but the
  decision-layer's copy can lag the measurement-layer's by one or more
  cycles, since it reflects a queued value, not a freshly-computed one.
- **Duplicate `CURRENT_DIAG` tag** — both blocks share the same header
  string, giving no label-level signal to distinguish which block a
  given `engineeringA=` line came from.
- **Asynchronous sampling** — the two gating timers are independent;
  nothing guarantees temporal alignment between a measurement-layer
  print and a decision-layer print that happen to appear close together
  in the Serial stream.
- **Commissioning ambiguity** — directly observed in this session: a
  recorded `(rawA=1.79, engineeringA=0.82)` pair failed the deterministic
  `engineeringA = rawA / CT_TURNS` check. Investigation traced this to
  exactly the mechanism above — the two values were plausibly read from
  different blocks/instants, not a firmware defect in the compensation
  math itself.

## 4. Proposed Architecture

**Measurement Layer — sole owner of (all measurement-owned data):**
- `rawA`
- `engineeringA`
- CT constants (`ctTurns`, `ctRatioPrimaryA`, `ctRatioSecondaryA`)
- `scale`
- `sample_id` *(optional — diagnostic-only; see Ownership Rules, §5, and
  Risks, §8, for its required scoping)*
- timestamp/age *(optional)* — e.g. `millis()` at print time, letting a
  reader compute elapsed time since this sample independently.

**Decision Layer — renamed to `CURRENT_DECISION`, owns only
decision-derived values and decision-owned state:**
- `signalPresent`
- thresholds (`CURRENT_ON_THRESHOLD_A` / `CURRENT_OFF_THRESHOLD_A`)
- `motor_state`
- EMA (`ema_current` = `s_currentFiltered`)
- evidence age (`ageMs` = `evidence.ageMs`, already computed at this
  call site, simply not currently printed here — a decision-derived
  value computed from the measurement-owned `g_lastCurrentSampleMs`
  timestamp, not measurement-layer state itself)

No field that mirrors measurement-owned data (`rawA`, `engineeringA`,
CT constants) appears in the decision layer under the new design.

**`current_valid` — explicitly out of scope for P3-01.** This flag is
not required to satisfy this design's Objective (§1): SSOT for
`rawA`/`engineeringA` is fully achieved without it, and `ageMs` already
gives the decision-layer block a staleness signal. See §8 for the
available options if a future design chooses to bring it in scope.

## 5. Ownership Rules

1. Measurement-owned data (a raw or engineering-unit *measurement* of
   current, or a constant describing the measurement chain) is printed
   by the measurement layer, and only there.
2. The decision layer may compute and print decision-derived values
   from measurement-owned inputs (e.g. `ageMs`, derived from
   `g_lastCurrentSampleMs`; or a future reference to `sample_id`, if
   adopted) — but must never re-print measurement-owned data itself
   (e.g. `rawA`, `engineeringA`).
3. Each diagnostic tag (`[CURRENT_DIAG]`, `[CURRENT_DECISION]`)
   corresponds to exactly one owning layer; a tag is never shared.
4. Any future source-specific decision state (a new filter, a new
   latch) is owned and printed by the decision layer exclusively.
5. This ownership split requires no change to `buildMotorStateEvidence()`'s
   signature or to `updateMotorStateMachine()` — it operates entirely
   inside the existing `#ifdef DEBUG_CURRENT_PATH` blocks, preserving the
   frozen architecture from the P2 compliance review.
6. Any new state introduced solely to support these diagnostics (e.g.
   `sample_id`) must itself be declared and updated only inside
   `#ifdef DEBUG_CURRENT_PATH` — it must not exist, in any form, in a
   build where that flag is undefined. Diagnostic-only state must never
   become production state.

## 6. Migration Plan

1. Rename the decision-layer's diagnostic header from `[CURRENT_DIAG]`
   to `[CURRENT_DECISION]`.
2. Remove the `engineeringA` and `raw_current` fields from the
   decision-layer printf block.
3. Add `ageMs` (`evidence.ageMs`, already computed, currently unprinted
   at this call site) to the decision-layer block.
4. *(Optional)* Introduce a `sample_id` counter, declared and
   incremented only inside `#ifdef DEBUG_CURRENT_PATH` in the
   measurement layer, incrementing once per successful measurement-layer
   print; print it in both blocks so the decision layer can reference
   "as of sample #N" without duplicating the amp value. This counter
   must not exist, and must have no effect of any kind, when
   `DEBUG_CURRENT_PATH` is undefined.
5. Compile-verify only, per this project's established pattern — no
   flash required unless a full functional re-verification on hardware
   is separately desired.
6. Note (not a step to execute now): `P2_CT_TURNS_COMMISSIONING_CHECKLIST.md`'s
   existing caveat ("read from the measurement-layer block, not the
   decision layer's `raw_current`") would become partially moot once
   `raw_current` no longer exists — but P2 documentation is frozen, so
   any update to that checklist is deferred to a future, explicitly
   approved pass, not part of this migration.

## 7. Backward Compatibility

- These are Serial-only diagnostics — never part of any MQTT/JSON
  payload (`doc[...]` construction elsewhere in the file is entirely
  separate from these `#ifdef DEBUG_CURRENT_PATH` blocks). `CLAUDE.md`'s
  MQTT-payload backward-compatibility rule does not apply here by its
  own scope (it governs payload fields, not debug logging).
- The measurement-layer block's tag and field set are unchanged — any
  tooling or habit built around `[CURRENT_DIAG]`'s `rawA`/`engineeringA`/
  CT-constant fields continues to work identically.
- The decision-layer block's tag changes, and it drops two fields
  (`engineeringA`, `raw_current`). Anything currently reading those two
  fields from the decision layer must switch to the measurement layer —
  which the P2 checklist already recommends, since those fields were the
  source of the commissioning ambiguity in the first place.

## 8. Risks

- Existing scripts or muscle memory tied to the decision layer's old tag
  or its `engineeringA`/`raw_current` fields will need to adjust. Low
  stakes: diagnostic-only, and the removed fields were actively
  misleading.
- If `sample_id` is adopted, it is new shared diagnostic-only state.
  Per Ownership Rule 6, it must be declared and incremented strictly
  inside `#ifdef DEBUG_CURRENT_PATH`, must remain write-only from the
  measurement layer and read-only (diagnostic-print use only) from the
  decision layer, and must never be consulted by
  `updateMotorStateMachine()`'s decision logic — otherwise it
  reintroduces the kind of source-specific coupling the P2 compliance
  review specifically closed.
- **`current_valid` — options, stated objectively (not adopted in this
  design; see §4):**
  - *Option A — widen `buildMotorStateEvidence()`'s signature* to accept
    `current_valid` directly. Makes it available immediately, but
    conflicts with Ownership Rule 5 and the P2 compliance review's
    constraint against widening this function's parameter surface.
  - *Option B — a new measurement-layer-owned global*, mirroring
    `current_valid`'s last-known state the same way `g_lastCurrentSampleMs`
    already mirrors sample timing. Makes the flag available to the
    decision layer with no signature change, consistent with Ownership
    Rule 5.
  - *Option C — leave it out of scope*, as this design does: `ageMs`
    already gives the decision-layer block a staleness signal, which
    covers the practical need without adding new state.
  - This design adopts **Option C** for P3-01. If a future phase wants
    `current_valid` specifically, Option B is the path consistent with
    this document's ownership rules; Option A is not recommended.
- `DEBUG_CURRENT_PATH` is a bench/commissioning-only flag; this entire
  redesign has no effect on production builds where it is undefined,
  which correctly scopes the risk and urgency of this change to bench
  work only.

## 9. Acceptance Criteria

- Exactly one `Serial.println("[CURRENT_DIAG]")` call site exists in
  source (verifiable via `grep -c`).
- The decision-layer block is tagged `[CURRENT_DECISION]` and contains
  no field duplicating measurement-owned data.
- `rawA`/`engineeringA`, wherever printed, are always from the same
  function call — trivially satisfied once there is only one print site
  for them.
- If `sample_id` is implemented, it (and its increment) exists only
  inside `#ifdef DEBUG_CURRENT_PATH`, with zero footprint or effect when
  that flag is undefined.
- Clean compile, exit 0, no new warnings.
- `updateMotorStateMachine()` and `buildMotorStateEvidence()`'s decision
  logic are unchanged outside the two `#ifdef DEBUG_CURRENT_PATH` blocks
  (verifiable by diff review).
- No MQTT/JSON payload field added, removed, or changed in meaning.
- (Recommended, not required) A bench smoke-test confirming both blocks
  print sensibly, and that the decision layer's `ageMs` correctly
  reflects staleness relative to the measurement layer's most recent
  sample.
