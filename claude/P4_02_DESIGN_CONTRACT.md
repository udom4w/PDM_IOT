# P4-02 — Communication Health Telemetry: Design Contract

**Status:** Design only. No firmware modified. Awaiting approval.

Supersedes nothing — this contract implements Alternative E / Option A as
approved in `P4_02_ARCHITECTURE_REVIEW.md` (§4, §9), with the `volatile`
correction from §9.3 applied (plain `static bool`, not `volatile`).

## 1. Objective

Expose CURRENT (CTR4A01) communication health to MQTT telemetry, so a real
communication loss is visible as a distinct condition, while preserving
every behavior introduced and hardware fault-injection-verified in P4-01
(`v16.5.1-p4.1`) without modification.

## 2. Source of Truth

The Source of Truth for CURRENT evidence validity is, and remains:

```
s_currentEvidenceValid
  — function-local static bool
  — declared and written only inside buildMotorStateEvidence()'s
    MOTOR_SRC_CURRENT case
```

This contract introduces **no second calculation** of validity. Nothing
in this document changes when, how, or why `s_currentEvidenceValid` is
set. Its two existing assignment sites (the `ageMsNow >
CURRENT_EVIDENCE_MAX_AGE_MS` branch, and the `isFreshSample` branch) are
the only places validity is ever decided, exactly as verified in P4-01.

## 3. The One Telemetry Mirror

Exactly one new symbol is introduced:

```
g_currentEvidenceValid
  — static bool (file scope)
  — a published TELEMETRY MIRROR of s_currentEvidenceValid's value
  — NOT a second source of truth
  — NOT read by any FSM, business-logic, or decision-making code
```

Its only purpose is to make an already-decided value visible outside
`buildMotorStateEvidence()`. It never originates a decision; it only ever
repeats one already made.

## 4. Ownership, Producer, Consumer, Lifetime

| Concept | Definition |
|---|---|
| **Ownership** | `buildMotorStateEvidence()` owns the *value* (via `s_currentEvidenceValid`) and is the sole writer of its mirror, `g_currentEvidenceValid`. Ownership is never transferred, shared, or duplicated — the mirror has exactly one producer, matching its one source. |
| **Producer** | `buildMotorStateEvidence()`'s `MOTOR_SRC_CURRENT` case, and only that case. Written at the same two lines that already assign `s_currentEvidenceValid` (Invariant I1, §7) — a trailing mirror-write alongside each existing assignment, not a new decision point. |
| **Consumer** | `captureTelemetrySnapshot()` (Core 0) — reads `g_currentEvidenceValid` once per state-machine cycle and copies it, unmodified, into `TelemetrySnapshot.currentEvidenceValid`. `publishTelemetry()` (Core 1) — reads `TelemetrySnapshot.currentEvidenceValid` from its local snapshot copy and copies it, unmodified, into the MQTT JSON field `current_evidence_valid`. No other consumer exists. |
| **Lifetime** | Static storage duration (program lifetime), initialized `false` at boot — matching `s_currentEvidenceValid`'s own initial value, so the mirror never reports a stale "true" before the evidence pipeline has run even once. No NVS persistence; this is live-state telemetry, not a logged/latched event. |

## 5. Synchronization and Thread Safety

- **Write side**: `buildMotorStateEvidence()` runs exclusively inside
  `taskStateMachine()`, Core 0. `g_currentEvidenceValid` is written only
  there.
- **First read side**: `captureTelemetrySnapshot()` also runs exclusively
  inside `taskStateMachine()`, Core 0, called immediately after
  `updateMotorStateMachine()` completes in the same state-machine cycle
  that produced the value. **Producer and this consumer are the same
  core, the same task, the same cycle, sequential** — there is no
  concurrent access to `g_currentEvidenceValid` itself, and therefore no
  race to guard against at this hand-off point.
- **Declaration**: plain `static bool` — no `volatile` qualifier.
  `volatile` is not required here (unlike `g_sensorOffline`, which is
  read from a genuinely different task, `taskAnalytics` on Core 1, via
  `anaSensorHealthy()`). Adding `volatile` on the strength of that
  precedent alone would be unjustified, per the P4-02 Architecture Review
  §9.2/§9.3 finding.
- **Second read side (real Core0→Core1 boundary)**: the actual
  cross-core hand-off is `TelemetrySnapshot` itself
  (`g_telemSnapshot`/`localSnap`), already synchronized by the existing
  `mutexVibData` inside `captureTelemetrySnapshot()`. This contract adds
  no new mutex, semaphore, or queue — `currentEvidenceValid` rides inside
  the same struct copy already protected for every other
  `TelemetrySnapshot` field.

## 6. Core0/Core1 Data Flow

```
CORE 0 (taskStateMachine)                       CORE 0 -> CORE 1 HANDOFF        CORE 1 (taskNetwork)
──────────────────────────                      ─────────────────────          ────────────────────

buildMotorStateEvidence()
  MOTOR_SRC_CURRENT case:
    s_currentEvidenceValid = ...   (existing,
                                    unmodified,
                                    the ONLY place
                                    this is decided)
    g_currentEvidenceValid =
        s_currentEvidenceValid    <- NEW: mirror-write only,
                                       no new condition, no new
                                       comparison, no new timer
        |
        v  (same task, same cycle, sequential -- no race)
captureTelemetrySnapshot(data, effectiveState)
    snap.currentEvidenceValid =
        g_currentEvidenceValid    <- NEW: pure copy, no
                                       computation
        |
        | xSemaphoreTake(mutexVibData)   <- EXISTING mechanism,
        | memcpy(&g_telemSnapshot, ...)     unchanged, already
        | xSemaphoreGive(mutexVibData)      used for every other
        |                                   TelemetrySnapshot field
        v
  g_telemSnapshot ────────────────────────────────────┐
                                                       │
                                                       v
                                         localSnap = g_telemSnapshot
                                         (existing copy-out pattern,
                                          e.g. taskNetwork's
                                          TelemetrySnapshot localSnap)
                                                       │
                                                       v
                                         publishTelemetry(&localSnap)
                                           doc["current_evidence_valid"]
                                             = snap->currentEvidenceValid  <- NEW:
                                                                              pure copy,
                                                                              no computation
                                           -> /vibration MQTT payload
                                              (existing topic, already
                                               carries current_read_errors,
                                               current_buf_count,
                                               current_slope)
```

Every arrow above is either an existing, unmodified mechanism
(state-machine cycle sequencing, `mutexVibData`, the snapshot copy-out
pattern, the `/vibration` publish call) or a plain, uncomputed copy. No
new arrow performs a calculation.

## 7. Invariants

- **I1 — Single decision point.** Validity is decided in exactly one
  place: `s_currentEvidenceValid`'s two existing assignment sites inside
  `buildMotorStateEvidence()`'s `MOTOR_SRC_CURRENT` case. Nothing added by
  this contract introduces a third assignment site, a new condition, or a
  new comparison against time.
- **I2 — One producer for the mirror.** `g_currentEvidenceValid` is
  written only inside `buildMotorStateEvidence()`. No other function,
  task, or ISR may assign to it under any circumstance, present or future.
- **I3 — Mirror is read-only downstream.** `captureTelemetrySnapshot()`,
  `TelemetrySnapshot.currentEvidenceValid`, and
  `doc["current_evidence_valid"]` are each a straight, unconditional copy
  of the value one step upstream. None of them may contain a conditional,
  a timeout, or a fallback default that alters the value in transit.
- **I4 — No feedback path.** No code downstream of
  `buildMotorStateEvidence()` (telemetry, MQTT, display, analytics) may
  write back to `g_currentEvidenceValid`, `s_currentEvidenceValid`, or any
  other evidence-lifecycle state. Telemetry is a one-way observer.
- **I5 — FSM isolation.** `updateMotorStateMachine()` never reads
  `g_currentEvidenceValid`, `TelemetrySnapshot.currentEvidenceValid`, or
  the MQTT field. The FSM's inputs remain exactly `MotorStateEvidence`
  (`signalPresent`/`ageMs`/etc.), unchanged since P4-01.
- **I6 — Additive-only schema.** `current_evidence_valid` is a new key on
  the existing `/vibration` payload. No existing key on any topic changes
  name, type, or meaning.
- **I7 — Boot-safe default.** `g_currentEvidenceValid` initializes to
  `false`, matching `s_currentEvidenceValid`'s own initial value, so a
  cold boot never publishes a false "valid" before the first evidence
  cycle runs.

## 8. Acceptance Criteria

- **AC1** — `g_currentEvidenceValid`'s value, sampled at any instant,
  equals `s_currentEvidenceValid`'s value as of the most recently
  completed `buildMotorStateEvidence()` call. No divergence window longer
  than one state-machine cycle.
- **AC2** — Grep-level audit: `g_currentEvidenceValid` is assigned in
  exactly two places in the source, both inside
  `buildMotorStateEvidence()`'s `MOTOR_SRC_CURRENT` case, both immediately
  adjacent to an existing `s_currentEvidenceValid` assignment.
- **AC3** — `captureTelemetrySnapshot()`'s new line is a bare assignment
  (`snap.currentEvidenceValid = g_currentEvidenceValid;`) with no
  surrounding conditional, no timeout comparison, no default-value
  fallback.
- **AC4** — `publishTelemetry()`'s new line is a bare assignment
  (`doc["current_evidence_valid"] = snap->currentEvidenceValid;`) with no
  surrounding conditional.
- **AC5** — Every existing field on `/sensor`, `/status`, and
  `/vibration` is byte-identical in name, type, and value to pre-P4-02
  behavior, for identical firmware state (regression check against
  `v16.5.1-p4.1`).
- **AC6** — `updateMotorStateMachine()`'s compiled behavior is provably
  unchanged (no diff inside that function).
- **AC7** — `MotorStateEvidence`'s definition is unchanged (no new
  field, no changed field).
- **AC8** — Hardware verification reproduces a P4-01-style fault
  injection (RS485/CTR4A01 disconnect) and confirms
  `current_evidence_valid` transitions `true → false` on the same cycle
  `s_currentEvidenceValid` (and therefore `ev.signalPresent`) does, and
  `false → true` on the same first-fresh-sample cycle as the existing
  reseed behavior — i.e., the mirror never lags or leads its source by
  more than the one sequential hand-off in §6.

## 9. Implementation Constraints (for the implementation phase, not this document)

1. Modify only:
   - `buildMotorStateEvidence()`'s `MOTOR_SRC_CURRENT` case — add the
     `g_currentEvidenceValid` declaration (file scope, near
     `s_currentEvidenceValid`'s existing declaration comment block or
     other CURRENT-related file-scope statics) and its two mirror-write
     lines only.
   - `captureTelemetrySnapshot()` — add one bare copy line.
   - `TelemetrySnapshot` struct — add exactly one new field,
     `currentEvidenceValid` (type `bool`).
   - `publishTelemetry()`'s existing `/vibration` block (PUBLISH 3 of 3)
     — add exactly one new bare-assignment line,
     `doc["current_evidence_valid"]`.
2. Do not modify `updateMotorStateMachine()`.
3. Do not modify `MotorStateEvidence`.
4. Do not modify `MotorRunState_t`.
5. Do not modify, rename, retype, or reinterpret any existing MQTT field
   on any topic.
6. Do not add `volatile` (§5).
7. Do not add any new timer, timestamp, counter, or comparison anywhere
   in this feature — every new line is a declaration or a bare copy.
8. Do not commit. Wait for code review before any hardware testing or
   commit, consistent with the P4-01 precedent.

## 10. Risks

- **Divergence risk (mitigated by I1/I2/AC2):** the two mirror-write
  sites could, in a future edit, be updated independently and drift out
  of sync. Mitigated by keeping them textually adjacent to
  `s_currentEvidenceValid`'s own assignments (same diff, same review,
  same three-line span) and by AC2's grep-level audit becoming a
  permanent, cheap regression check.
- **Telemetry lag:** `current_evidence_valid` reflects the value as of
  the last completed state-machine cycle, not the instant of MQTT
  publish — identical staleness characteristics to every other
  `TelemetrySnapshot`-derived field (`motor_state`, `rpm`,
  `health_score`); not a new class of risk.
- **Consumer misinterpretation (unchanged from Architecture Review §7):**
  a dashboard could conflate `current_evidence_valid=false` with
  `motor_state=STOPPED`; this is a dashboard-side interpretation concern,
  not a firmware defect, and out of scope for this contract.
- **Scope creep:** this contract does not implement CTR4A01
  Modbus-return-code offline detection (P4-03) or any change to the
  `/sensor` or `/status` payloads. Any temptation to "also expose ageMs"
  or "also add a WTVB02-style offline threshold" here is explicitly out
  of scope and would reintroduce Alternative D/C from the Architecture
  Review (both rejected).

## 11. Design Rationale

**Telemetry reports evidence. Telemetry never decides evidence.**

Every function this contract touches or adds a line to
(`buildMotorStateEvidence()`'s two mirror-writes,
`captureTelemetrySnapshot()`'s copy, `publishTelemetry()`'s copy) performs
no comparison against time, no threshold check, and no state transition.
The single decision — is CURRENT evidence currently valid — is made once,
in the one place it has always been made
(`s_currentEvidenceValid`, hardware fault-injection-verified in P4-01),
and is then carried downstream by unconditional copies only. This is the
same discipline already applied to `motor_state`, `rpm`, and
`health_score` in `TelemetrySnapshot` today: the state-machine layer
decides, the telemetry layer reports.

---

No firmware modification. No code generated. Stopping here per
instruction — waiting for approval before implementation.
