# P4-02 — Communication Health Telemetry: Hardware Verification Plan

**Status:** Planning only. No firmware modified. No code generated. Not yet
executed — waiting for hardware test results before any commit or tag.

## 1. Objective

Verify, on real hardware, that `current_evidence_valid` (MQTT,
`/vibration` topic) accurately mirrors `s_currentEvidenceValid`
(`buildMotorStateEvidence()`'s `MOTOR_SRC_CURRENT` case, the sole source of
truth) with correct timing and no independent behavior of its own — and
that P4-01's already-verified evidence-lifecycle behavior
(`v16.5.1-p4.1`) is unchanged.

## 2. Observability Constraint (read before executing)

This plan must work within "do not modify firmware, do not generate code"
— no new Serial print was added for `g_currentEvidenceValid` or
`TelemetrySnapshot.currentEvidenceValid` in the P4-02 implementation. The
three layers are observable as follows:

| Layer | How it is observed in this plan |
|---|---|
| `s_currentEvidenceValid` | **Indirectly inferred** from the existing `[CURRENT_DECISION]` Serial block (`ageMs`, `ema_current`, `signalPresent`) — same inference method already used and validated in the P4-01 fault-injection report: `ageMs > 5000` with `ema_current` reset to `0.000` marks an invalidation edge; a subsequent fresh sample whose `ema_current` exactly equals the raw sample value (not a blend) marks a reseed edge. |
| `g_currentEvidenceValid` | **Not independently observable at runtime** (no Serial print exists for it). Per Design Contract Invariants I1/I2 — it is written only in the same two statements as `s_currentEvidenceValid`, in the same source line pair — its value is identical to `s_currentEvidenceValid` **by construction**, already confirmed by static code inspection (AC2 in the Design Contract). This plan does not re-test that identity at runtime; it is not testable independently of the code itself. |
| `TelemetrySnapshot.currentEvidenceValid` | **Not independently observable at runtime** either (no Serial print). Confirmed by code inspection to be a bare copy of `g_currentEvidenceValid` (AC3). Same reasoning as above. |
| `current_evidence_valid` (MQTT) | **Directly observable** — the only externally visible representation of the mirrored value, at the `/vibration` topic's existing publish cadence. |

**Consequence for this plan:** the hardware test's job is to confirm the
**end-to-end pipeline** (Serial-inferred `s_currentEvidenceValid` proxy
↔ MQTT `current_evidence_valid`) agrees in value and timing, and that
P4-01's own FSM/evidence behavior is not regressed. It is not attempting
to re-derive what static inspection already proved (AC2/AC3/AC4).

**Timing constraint:** `/vibration` publishes on a state-dependent
interval — **30 s in NORMAL state**, 10 s in WARNING, 5 s in CRITICAL
(`publishInterval`, `taskNetwork`). `TelemetrySnapshot` itself is captured
far more often (~every state-machine cycle, ~250 ms), but only the value
present in `g_telemSnapshot` at the moment a publish fires reaches MQTT.
Therefore:
- A transient state that resolves in a few seconds may or may not be
  caught by an MQTT publish, depending on timing luck.
- To make MQTT verification **deterministic rather than probabilistic**,
  the "long interruption" test (§3, Test 5) must be held for **at least
  35 seconds** — longer than one full NORMAL-state publish interval —
  guaranteeing at least one `/vibration` publish lands while
  `current_evidence_valid` is `false`.
- The "short interruption" test (§3, Test 4) relies on Serial as the
  primary evidence; MQTT is used only to confirm the value never left
  `true` across the surrounding publishes (a negative check, for which
  30 s cadence is sufficient).

## 3. Test Matrix

Motor is running (current source active, `TEST_CURRENT_SOURCE`,
`MOTOR_SRC_CURRENT`) for all tests except Test 2, which specifically
requires a genuine stop. `CURRENT_EVIDENCE_MAX_AGE_MS = 5000 ms`
throughout.

| # | Test | Physical action | Duration guidance |
|---|---|---|---|
| 1 | Power-on | Cold boot the device (power cycle), CTR4A01 wiring intact throughout. | Capture from power-up through first ~40 s (covers first publish + one full cycle). |
| 2 | Motor STOPPED (genuine, comms healthy) | Let motor sit stopped/idle; CTR4A01 wiring intact, CT continues to read ~0 A on its normal ~500 ms cadence. | ≥60 s steady-state. |
| 3 | Motor RUNNING (comms healthy) | Motor running normally, CTR4A01 wiring intact. | ≥60 s steady-state. |
| 4 | Short communication interruption (< `CURRENT_EVIDENCE_MAX_AGE_MS`) | Motor running. Disconnect CTR4A01/RS485 for ~2 s. Restore. | Interruption ~2 s; observe ≥20 s before and after. |
| 5 | Long communication interruption (> `CURRENT_EVIDENCE_MAX_AGE_MS`) | Motor running. Disconnect CTR4A01/RS485 for **≥35 s** (see §2 timing constraint — long enough to guarantee an MQTT publish lands mid-outage). Restore. | Interruption ≥35 s; observe ≥20 s before and after. |
| 6 | Communication recovery | The restore step of Tests 4 and 5, examined specifically for reseed-vs-blend correctness (this is not a separate physical action — it is a focused re-read of the recovery edges already captured in Tests 4 and 5). | N/A — analysis pass over existing captures. |
| 7 | Multiple consecutive disconnect/reconnect cycles | Motor running. Repeat: disconnect ~2 s → restore → settle ~15–20 s → disconnect ~7 s → restore → settle ~15–20 s, **twice** (mirrors the P4-01 fault-injection sequence exactly, so results are directly comparable to `v16.5.1-p4.1`). | Full sequence ≥5 min to comfortably allow at least one MQTT publish window to fall inside a disconnect. |

## 4. Expected State/Transition Table

`valid` = evidence considered valid/fresh. `—` = not independently
observable this round (see §2); expected value shown is what code
inspection guarantees it must equal, for cross-reference only.

| # | Phase | `s_currentEvidenceValid` (Serial-inferred) | `g_currentEvidenceValid` | `TelemetrySnapshot.currentEvidenceValid` | MQTT `current_evidence_valid` |
|---|---|---|---|---|---|
| 1 | Immediately at boot, before first CT sample | `false` (static init) | `false` (— , guaranteed identical) | `false` (—, guaranteed identical) | not yet published (no MQTT until first successful vibration capture + connect) |
| 1 | After first successful CT sample | `true` | `true` (—) | `true` (—) | `true`, at the first publish that occurs after this point |
| 2 | Motor genuinely stopped, comms healthy | `true` (fresh ~0 A samples keep arriving every ~500 ms; `signalPresent=false` because latched off, but evidence itself stays fresh) | `true` (—) | `true` (—) | `true` — **this is the key distinguishing case**: `motor_state=STOPPED` with `current_evidence_valid=true` is the *expected, healthy* signature of a real stop, not a defect |
| 3 | Motor running, comms healthy | `true`, `ema_current` blending toward steady running current | `true` (—) | `true` (—) | `true` |
| 4 | During short interruption (peak `ageMs` stays < 5000) | `true` throughout — no invalidation edge in Serial (`ema_current` continues blending, never resets to `0.000`) | `true` throughout (—) | `true` throughout (—) | `true` at every publish surrounding the test (no transition expected/observable) |
| 5 | During long interruption, once `ageMs` first exceeds 5000 | `false` (Serial shows `ema_current=0.000`, `signalPresent=0`, `ageMs>5000`) | `false` (—) | `false` (—) | `false`, on the publish that lands during the ≥35 s hold |
| 5 | First fresh sample after restore | `true`, `ema_current` == raw sample value exactly (reseed, not blend) | `true` (—) | `true` (—) | `true`, on the next publish after recovery |
| 6 | Recovery edge (both Test 4 restore and Test 5 restore) | Test 4: no edge to find (was never invalid). Test 5: reseed edge present, `ema_current` matches raw sample exactly on first post-recovery sample, blend arithmetic resumes correctly on the next | same, by construction | same, by construction | Test 5: `false → true` transition visible across two consecutive publishes at most |
| 7 | Across repeated short/long cycles | Alternates `true`→(stays true, short)→`true`→`false`→`true`(long)→ repeat, matching each disconnect's actual peak `ageMs`, exactly reproducing the pattern already seen in the P4-01 fault-injection log (two sub-threshold events + one over-threshold event observed there) | mirrors, by construction | mirrors, by construction | mirrors the Serial-inferred pattern, subject to §2's cadence caveat for any sub-35s-total-exposure long event |

## 5. Verification Checklist

For each test, confirm all of the following against the collected evidence
(§6):

- [ ] **No false transition** — `current_evidence_valid` never reports
  `false` while Serial evidence shows `ageMs` staying below 5000 ms
  throughout (i.e., no spurious invalidation with comms healthy).
- [ ] **No delayed transition** — the MQTT value change (when a publish
  does land during/after a real transition) is consistent with the
  Serial-inferred edge having already occurred at or before that publish
  — never a stale value held past the transition by more than the
  explainable one-cycle capture lag (§2).
- [ ] **No missing transition** — Test 5's ≥35 s hold produces at least
  one MQTT publish showing `current_evidence_valid=false`; Test 5's
  restore produces a subsequent publish showing `true` again. Neither
  edge may be silently absent.
- [ ] **No oscillation** — Test 7's repeated cycles show clean, single
  transitions per disconnect/reconnect event, with no rapid
  true/false/true flapping within a single physical event (mirrors the
  "no oscillation" concern that originally motivated the P1
  STARTING⇄STOPPING investigation — same failure class, different
  layer).
- [ ] **No regression of P4-01 behavior** — for every test, independently
  confirm (from the same Serial capture) that `[MOTOR-TRANSITION]`,
  `ev.signalPresent`, EMA blend/reseed arithmetic, and FSM state
  transitions match the same patterns already confirmed correct in
  `P4_01_FAULT_INJECTION_VERIFICATION_REPORT.md` — this plan's new field
  must be a strict addition, not a change, to that already-verified
  behavior.

## 6. Evidence to Collect

- **Serial log**: one continuous capture per test (or one combined
  capture spanning Tests 3→7 sequentially, matching the P4-01
  fault-injection session's approach), including `[CURRENT_DECISION]`,
  `[CURRENT_DIAG]`, `[MOTOR-TRANSITION]`, `[MOTOR-DIAG]`, `[SIGNAL]`, and
  `[CURRENT] FAIL` lines. Use the existing capture script
  (`capture_v16_5_6_validation_20260727.ps1` pattern) with a duration
  covering each test's guidance in §3.
- **MQTT payload**: a subscriber (e.g. `mosquitto_sub` with the
  appropriate mTLS client certificate, or the existing Grafana/Node-RED
  consumer if already wired to `/vibration`) capturing every message on
  the `/vibration` topic for the same wall-clock window as the Serial
  capture, with receipt timestamps. This is external to the firmware and
  must be run in parallel by whoever performs the physical test — I
  cannot reach the MQTT broker myself.
- **Timeline**: reconstruct a single merged timeline per test, aligning
  Serial line timestamps with MQTT message receipt timestamps against a
  common wall clock, the same reconciliation method used in the P4-01
  fault-injection report (§4 of that report).
- **Screenshots**: optional — only if a Grafana/Node-RED dashboard
  already visualizes `/vibration` and showing `current_evidence_valid`
  there is convenient; not required if the raw MQTT payload capture is
  available.

## 7. Pass / Fail Criteria

| Test | PASS condition | FAIL condition |
|---|---|---|
| 1 — Power-on | Both `s_currentEvidenceValid` (Serial-inferred) and `current_evidence_valid` (MQTT) start at/imply `false` before the first successful CT sample, and both become `true` no later than the first publish after the first successful sample. | Either layer shows `true` before any successful CT sample, or MQTT never transitions to `true` despite Serial showing valid evidence for >1 publish interval. |
| 2 — Motor STOPPED, comms healthy | `motor_state=STOPPED` co-occurs with `current_evidence_valid=true` for the full observation window (comms healthy, evidence fresh, current genuinely near 0 A). | `current_evidence_valid` ever reports `false` while Serial shows fresh ~500 ms sampling with `ageMs` staying below 5000 ms. |
| 3 — Motor RUNNING, comms healthy | `current_evidence_valid=true` for the full window, consistent with `signalPresent=1`/`motor_state=RUNNING`. | Any unexplained `false` reading during healthy operation. |
| 4 — Short interruption | `current_evidence_valid` remains `true` across every publish surrounding the interruption; Serial shows peak `ageMs` < 5000 and continuous EMA blending (no reset to `0.000`). | Any MQTT or Serial-inferred sign of invalidation for an interruption whose measured peak `ageMs` stayed below 5000 ms. |
| 5 — Long interruption | At least one MQTT publish during the ≥35 s hold shows `current_evidence_valid=false`, coincident with Serial showing `ageMs > 5000`, `ema_current=0.000`; the next publish after restore shows `true` again, coincident with Serial showing a reseed (exact-match, non-blended) sample. | No `false` publish observed during the hold (missing transition), or `false` observed while Serial's peak `ageMs` never exceeded 5000 (false transition), or `true` fails to return within one publish cycle after restore (delayed transition). |
| 6 — Recovery | Test 4's restore shows no reseed edge (because no invalidation occurred) and Test 5's restore shows an exact-match reseed followed by normal blending on the next sample — both consistent with P4-01's already-verified reseed-vs-blend logic. | Test 5's restore shows a blend instead of a reseed on the first post-recovery sample (would indicate the mirror/telemetry change somehow affected P4-01's decision logic — must not happen, since P4-02 touches no decision code). |
| 7 — Repeated cycles | Every disconnect/reconnect event produces a clean, single, correctly-classified transition (per its own measured peak `ageMs`) with no flapping within an event, across all repetitions. | Any event shows oscillation, a transition inconsistent with its own measured `ageMs`, or divergence between repetitions of the same nominal interruption length. |

## 8. Stop Condition

Per instruction: **this document stops here.** No firmware modification,
no code generated, no commit, no tag. Hardware execution of this plan and
its results are the next step, to be supplied before any commit or tag is
considered.
