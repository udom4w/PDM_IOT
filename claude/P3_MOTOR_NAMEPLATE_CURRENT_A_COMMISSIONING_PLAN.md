# P3-02 — MOTOR_NAMEPLATE_CURRENT_A Commissioning Plan

Planning phase only. No firmware modified, no constants changed.

## 1. Objective

- Replace the placeholder `MOTOR_NAMEPLATE_CURRENT_A` (currently `2.0f`,
  explicitly commented "PLACEHOLDER, commission before use") with a
  commissioned value reflecting the actual installed motor's nameplate
  Full-Load Amps (FLA).
- Use the `[CURRENT_DIAG]` Single Source of Truth established in P3-01
  as the sole source of live current readings during this commissioning
  — `rawA`/`engineeringA` from `[CURRENT_DIAG]` only, never from
  `[CURRENT_DECISION]`, which no longer carries measurement-owned data.

## 2. Existing Implementation

- **Definition**: `constexpr float MOTOR_NAMEPLATE_CURRENT_A = 2.0f;`
  (source, `[A] nameplate FLA -- PLACEHOLDER, commission before use`).
- **Derivation**:
  - `CURRENT_ON_THRESHOLD_A = MOTOR_NAMEPLATE_CURRENT_A * 0.20f` (20% FLA)
  - `CURRENT_OFF_THRESHOLD_A = MOTOR_NAMEPLATE_CURRENT_A * 0.12f` (12% FLA)
  - Together these form a Schmitt-trigger hysteresis pair with an
    8-point-of-FLA gap, applied to `s_currentFiltered` (the EMA of
    engineering current) inside `buildMotorStateEvidence()`'s
    `MOTOR_SRC_CURRENT` branch.
- **Components affected by changing this value**:
  - `s_currentLatched` / `ev.signalPresent` — directly gates
    STARTING/RUNNING/STOPPING detection accuracy for `MOTOR_SRC_CURRENT`.
  - `[CURRENT_DECISION]`'s `threshold=`/`current_threshold_off=` fields
    (P3-01) — literal aliases of these two constants; their printed
    values change as a byproduct of the same constant changing, not a
    separate implementation surface.
  - **Not affected**: `CT_TURNS`, `CT_RATIO_PRIMARY_A`/`CT_RATIO_SECONDARY_A`,
    or anything in the current-scaling chain (P2 scope). This is an
    orthogonal axis — P2 verifies that `engineeringA` correctly reflects
    true current; P3-02 verifies what *level* of that (now-trusted)
    current should count as "motor running."
  - **Dependency**: this commissioning's electrical cross-check (§3)
    is only meaningful if the scaling chain is already known-correct.
    If P2 Item 1 (`CT_TURNS`) has not been verified/closed, `engineeringA`
    readings used here are not yet trustworthy for the cross-check step
    — though the nameplate-reading method itself (§3) does not depend
    on this at all, since it is a direct transcription, not a
    measurement.
- **Rated FLA vs. Service Factor Amps**: `MOTOR_NAMEPLATE_CURRENT_A` is
  intended to represent the motor's **rated Full-Load Amps (FLA)** — the
  nameplate current at 100% rated load — **not** Service Factor Amps
  (SFA), which represents an allowed overload ceiling (e.g. at a 1.15
  service factor), not the motor's normal operating condition. The
  ON/OFF thresholds derived from this constant exist to detect ordinary
  running vs. stopped, not overload capacity; using SFA would shift both
  thresholds upward relative to normal operating current, degrading
  detection accuracy at the exact condition this constant exists to
  identify. If a nameplate lists both figures, use the rated FLA
  specifically, and record which figure was used as the source.

## 3. Measurement Methodology

**Primary method — read the nameplate directly.** Unlike `CT_TURNS`
(which required a physical turns count plus electrical verification),
`MOTOR_NAMEPLATE_CURRENT_A` is, by definition, a value stamped on the
motor's nameplate. The primary source of truth is that stamped figure,
transcribed directly — not derived from a measurement.

**Nameplate selection — multi-rating nameplates.** Many nameplates list
more than one current figure: dual voltage (e.g. 230V/460V), dual
frequency (e.g. 50/60Hz), or separate Y (star) and Δ (delta) start
ratings. Select the figure matching:
- The voltage actually supplied to this installation.
- The frequency actually supplied (line frequency).
- The winding connection actually in use (Y or Δ), if applicable.

If the correct figure cannot be unambiguously identified from the
nameplate and the installation's known supply configuration, **do not
guess.** Record all listed values on the nameplate, plus the
installation's known supply voltage/frequency/connection, and stop for
engineering review before proceeding to commission any value.

**Secondary method — electrical cross-check**, used to sanity-check the
transcription (or as a fallback if the nameplate is illegible/missing):

- **Required equipment**: the same reference clamp meter used in P2
  (with traceability: model, serial number, calibration status, date),
  and Serial/USB access to `[CURRENT_DIAG]` (COM5).
- **CT configuration**: must be the same, already-verified `CT_TURNS`/
  `CT_RATIO_*` configuration from P2 — see the dependency noted in §2.
- **Test conditions**: the motor operating at genuine, real rated/full
  load — not a bench idle or partial-load condition. FLA describes
  full-load current specifically; a reading taken at partial or no load
  will understate it. If genuine full load is not available at
  commissioning time, this must be recorded as a limitation, not
  papered over.
- **Measurement settling**: before recording any reading, observe at
  least 5 consecutive `[CURRENT_DIAG]` samples (≈5 seconds at the
  block's 1Hz print cadence) and confirm `engineeringA` has converged —
  no continuing directional drift across those samples — before treating
  the value as representative of steady-state full load. `CURRENT_EMA_ALPHA
  = 0.25` means a step change in load takes several cycles to settle; a
  reading taken mid-transient will understate true full-load current.
- **Load points**: one primary point — steady-state operation at rated/
  full load, settled per above. Optionally, one additional partial-load
  point purely as a sanity/correlation check (not a formal acceptance
  criterion, since nameplate FLA is a single stamped value, not a
  scaling relationship across a range).
- **Reference instrument**: same as P2 — model, serial number,
  calibration status, calibration due date recorded.
- **Acceptance tolerance**: motor nameplate FLA figures are themselves
  rated/typical values, not lab-precision numbers, and real loading
  varies session-to-session. A wider tolerance than P2's ±5% is
  appropriate for the cross-check specifically. Following the same
  hierarchy as P2: use a manufacturer/motor-datasheet-documented
  tolerance if available; otherwise a **±10-15% engineering fallback**
  for this cross-check only. The nameplate transcription itself carries
  no separate tolerance — it is used as read.

## 4. Commissioning Procedure

**Electrical safety (prerequisite — complete before any other step):**
- **PPE**: appropriate personal protective equipment for live electrical
  work at the installation's voltage class (insulated gloves, safety
  glasses/face shield, arc-rated clothing as required by site electrical
  safety policy).
- **LOTO**: Lockout/Tagout procedures must be followed for any step
  requiring the circuit de-energized (e.g., initial CT/wiring
  inspection, or safely accessing an enclosed nameplate). LOTO does not
  apply to the live current measurement itself, which requires the
  circuit energized and the motor running under load — but must be
  strictly observed for any preceding or surrounding de-energized step.
- **Safe measurement practices**: use a clamp meter rated for the
  circuit's voltage/category; clamp around the conductor using the
  meter's rated procedure; never open the clamp jaw on, or otherwise
  directly contact, an energized conductor.
- **Bench safety prerequisites**: confirm a qualified/authorized person
  performs or directly supervises the live measurement; confirm the
  emergency shutoff/disconnect location is known before starting; follow
  site policy on second-person presence for live electrical work.
- This section applies specifically to the electrical cross-check (§3).
  The nameplate-reading (primary) method requires no live electrical
  work — though safely accessing an enclosed or hard-to-reach nameplate
  may itself require LOTO, depending on installation.

**Bench preparation:**
- Complete the electrical safety prerequisites above.
- Locate and photograph the motor's nameplate.
- Confirm P2 Item 1 (`CT_TURNS`) status — verified/closed, or otherwise
  known-trustworthy, before relying on any electrical cross-check.
- Confirm the P3-01 firmware (`[CURRENT_DIAG]` SSOT) is flashed.
- Record reference instrument traceability, if the electrical
  cross-check will be performed.

**Data collection:**
- Record the nameplate-stamped FLA value (photograph as evidence),
  applying the nameplate-selection guidance in §3 if multiple ratings
  are present.
- If performing the cross-check: at genuine rated/full load, and only
  after the measurement-settling requirement (§3) is satisfied,
  simultaneously record the reference-meter reading and
  `[CURRENT_DIAG]`'s `rawA`/`engineeringA`. Note the loading context
  (confirmed full load, or partial-load limitation) and how full-load
  condition was confirmed (see Verification step 4 below).

**Verification steps:**
1. Compare the nameplate-stamped value against the current
   `MOTOR_NAMEPLATE_CURRENT_A = 2.0f` — this documents how far the
   placeholder is from reality, for the record.
2. If cross-checked: compare measured `engineeringA` against the
   nameplate-stamped value, within the tolerance in §3.
3. **Threshold sanity check**: compute what `CURRENT_ON_THRESHOLD_A`
   (20%) and `CURRENT_OFF_THRESHOLD_A` (12%) *would* become under the
   commissioned FLA, and confirm they remain physically sensible against
   observed `[CURRENT_DIAG]` behavior — the ON level should sit clearly
   above any observed noise floor when stopped, the OFF level should
   sit clearly below observed running current, and the hysteresis gap
   between them should not collapse.
4. **Full-load confirmation**: positively confirm the load condition
   during measurement was genuinely representative of rated/full load —
   e.g. corroborated via observed RPM, power, or process condition
   consistent with the motor's rated operation, or explicit
   operator/engineering attestation. An assumed or asserted "full load"
   without such corroboration does not satisfy this step.

**Failure criteria:**
- Nameplate illegible/missing **and** no reliable cross-check available
  — cannot close; escalate rather than guess a value.
- Nameplate ratings ambiguous (multi-voltage/frequency/connection) and
  the correct figure cannot be identified — stop for engineering review
  per §3; do not guess.
- Cross-check measurement inconsistent with the nameplate figure beyond
  tolerance — do not average or split the difference; investigate the
  discrepancy (wrong nameplate transcription? unresolved `CT_TURNS`?
  wrong motor?) before proceeding.
- Reading not settled per §3, or full-load condition not confirmed per
  Verification step 4 — do not record the measurement as accepted
  evidence; repeat once conditions are met.
- Prospective thresholds fail the sanity check in step 3 — record and
  escalate; do not adjust the percentages (20%/12%) as part of this
  plan, that is a separate, out-of-scope design decision.

**Rollback procedure:** this phase produces evidence only — there is no
firmware state to roll back yet. If a future, separately-approved
implementation changes `MOTOR_NAMEPLATE_CURRENT_A` and a problem is
later observed in production, rollback is: revert the constant to its
last-known-good value via `git revert`, recompile, and reflash — the
same pattern already established for every change in this engagement.

## 5. Risks

- Nameplate illegible or inaccessible (sealed enclosure, corroded tag).
  Mitigation: fall back to the electrical cross-check as primary
  evidence, with the wider tolerance in §3 explicitly acknowledged.
- Nameplate ratings ambiguous (dual voltage/frequency, Y/Δ) and
  misread as a single value without checking the installation's actual
  supply configuration. Mitigation: §3's nameplate-selection guidance
  and the "stop for engineering review" failure criterion.
- Rated FLA confused with Service Factor Amps, producing thresholds
  shifted upward relative to normal operating current. Mitigation: §2's
  explicit definition of which figure is intended.
- Motor not available at genuine full load during the commissioning
  window, or a reading taken before the EMA has settled. Either
  understates true FLA; committing a threshold based on an understated
  value risks the ON threshold being commissioned too low, potentially
  causing false "signal present" at loads that shouldn't count as
  running. Mitigation: the settling requirement and full-load
  confirmation step (§3, §4) — must be recorded as an explicit
  limitation if not met, not silently accepted.
- Electrical hazard from measuring a live, loaded motor circuit.
  Mitigation: the electrical safety prerequisites in §4.
- The same RS485/Modbus staleness risk documented in the P2 robustness
  review applies here: before trusting any `[CURRENT_DIAG]` sample,
  confirm it is fresh (via `[CURRENT_DECISION]`'s `ageMs`), the same
  discipline already used in P2 Item 1.
- Dependency on P2 Item 1 (`CT_TURNS`): if unresolved, only the
  electrical cross-check is affected — the nameplate-reading method
  itself remains independently valid.

## 6. Acceptance Criteria

- Nameplate FLA legibly read and recorded (photograph as evidence), **or**
  a documented, evidence-backed reason it was unavailable plus the
  electrical-measurement fallback used instead.
- If multiple ratings were present on the nameplate: the correct figure
  was identified against the installation's actual supply configuration
  and recorded, per §3 — or, if ambiguous, the item was escalated rather
  than guessed.
- The recorded figure is confirmed to be rated FLA, not Service Factor
  Amps, per §2.
- If the electrical cross-check was performed: the reading was taken
  only after the measurement-settling requirement (§3) was satisfied,
  and measured current at genuine rated/full load agrees with the
  nameplate-stamped value within the applicable tolerance
  (manufacturer-documented, or the ±10-15% fallback).
- **Positive confirmation that the motor was operating under
  representative rated/full-load conditions at the time of measurement**
  is recorded (Verification step 4) — an assumed or asserted "full load"
  without such confirmation does not satisfy this criterion.
- Prospective `CURRENT_ON_THRESHOLD_A`/`CURRENT_OFF_THRESHOLD_A`, computed
  from the commissioned FLA, pass the threshold sanity check (§4, step 3).
- Reference instrument traceability recorded, if the cross-check was
  performed.
- All evidence recorded in a results document, mirroring the format
  established by `P2_CT_TURNS_COMMISSIONING_RESULTS.md`.

## 7. Deliverables

- This planning document.
- *(Future phase, not part of this deliverable)*: a commissioning
  results document, a photograph of the nameplate, and reference
  instrument traceability record, once commissioning is executed.
- **No firmware change is part of this or the commissioning phase.** A
  change to `MOTOR_NAMEPLATE_CURRENT_A` itself is a separate, later,
  explicitly-approved implementation task, reviewed against the evidence
  this plan produces — mirroring exactly how `CT_TURNS` commissioning
  was kept separate from its firmware implementation.
