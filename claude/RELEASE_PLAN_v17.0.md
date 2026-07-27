# Release Plan: Firmware v17.0 — Telemetry Snapshot

Status: Planning
Author: (engineering team)
Date: 2026-07-06
Governing architecture (frozen): ADR-0001, ADR-0002, ADR-0003
Governing implementation plan (frozen): `IMPLEMENTATION_PLAN_telemetry_snapshot.md`
Current shipped baseline: v16.5 (per `PATCH_NOTES_v16.5.md`)
Applies to: WTVB02_ESP32S3 firmware, LilyGO T-Vending S3 hardware

This document plans the release of v17.0 — the version that carries
Implementation Plan Phases 1–5 into production. It does not revisit
architecture or implementation-phase design; those are frozen inputs.
This plan governs milestones, gating, acceptance, rollback, and
production readiness only.

**In scope for v17.0:** Implementation Plan Phases 1, 2, 3, 4, 5, 6.
**Out of scope for v17.0:** Phase 7 (per-domain expansion to
power/thermal/energy) — explicitly deferred per the Implementation
Plan's own roadmap, pending field validation of this release first. Any
Node-RED, InfluxDB, or Dashboard change is also out of scope — this is a
firmware-only release; those layers consume what this release produces,
on their own release schedule.

---

## 1. Milestones

Each milestone maps to one or more Implementation Plan phases and ends
at a regression gate that must pass before the next milestone begins,
consistent with this project's workflow rule that no phase proceeds
without an explicit approval checkpoint.

### M0 — Housekeeping Baseline
- **Maps to:** Phase 6 (stale "dormant queue" comment correction)
- **Deliverables:** Corrected comments only; zero functional change.
- **Commit boundary:** One commit, comment-only diff.
- **Regression gate:** Clean build produces byte-identical compiled
  behavior to pre-M0 baseline (v16.5).
- **Acceptance criteria:** Diff review confirms no non-comment lines
  changed.
- **Exit criteria:** Gate passed; tag as `v17.0-m0`.

### M1 — Schema Foundation
- **Maps to:** Phase 1 (`schema_version`, `domain` additive fields)
- **Deliverables:** Two new fields present on `/sensor`, `/status`,
  `/vibration`; all pre-existing fields unchanged.
- **Commit boundary:** One commit.
- **Regression gate:** MQTT capture diff against v16.5 baseline shows
  only the two new fields added, nothing removed or renamed, on all
  three topics, across a full RUNNING/STOPPED/STARTING/STOPPING cycle.
- **Acceptance criteria:** Every existing downstream consumer (Node-RED,
  Dashboard, in their *current*, unmodified form) continues to parse the
  payload without error — new fields are additive and must be ignorable
  by old parsers.
- **Exit criteria:** Gate passed; tag as `v17.0-m1`.

### M2 — Business Decision Capture
- **Maps to:** Phase 2 (Business Decision struct + capture-at-decision-
  time buffer, no replay yet)
- **Deliverables:** New in-memory ring buffer populated once per publish
  cycle with the four Business Decision values; no new MQTT-visible
  behavior.
- **Commit boundary:** One commit.
- **Regression gate:**
  - Existing `/sensor` buffer's fill count, overflow counter, and replay
    behavior are provably unaffected (isolation test).
  - New buffer's captured values match the same cycle's live-published
    decision values, for every cycle across a full state-transition
    sweep (STOPPED → STARTING → RUNNING → STOPPING → STOPPED).
  - Heap/stack headroom after this change is measured and recorded
    (baseline for M3–M5 comparisons).
- **Acceptance criteria:** Captured decision values are bit-identical to
  the corresponding live-published values for 100% of a multi-hour
  bench-test run spanning at least 3 full motor state cycles.
- **Exit criteria:** Gate and acceptance criteria both passed; tag as
  `v17.0-m2`.

### M3 — Business Decision Replay
- **Maps to:** Phase 3 (dedicated replay path, new topic)
- **Deliverables:** Working replay of buffered Business Decisions on
  MQTT reconnect, via a purpose-built replay function (never the
  live-publish function).
- **Commit boundary:** One commit.
- **Regression gate:**
  - Existing `/sensor` replay behavior (ordering, rate limiting, mutex
    timing, overflow counter) is unchanged during a replay burst that
    also exercises the new decision-replay path concurrently.
  - Zero instances, across every field of every replayed message in a
    forced-outage test, of a live-state value substituted for a
    buffered one (this is the specific, previously-identified failure
    class this gate exists to catch).
- **Acceptance criteria:** A controlled outage spanning at least one
  full motor-state transition, followed by reconnect, replays every
  buffered decision in oldest-first order with `captured_at` matching
  original capture time and values matching what was buffered — verified
  field-by-field, not spot-checked.
- **Exit criteria:** Gate and acceptance criteria both passed; tag as
  `v17.0-m3`. **This is the highest-scrutiny milestone in the release —
  it does not proceed to M4 without a second independent reviewer
  confirming the zero-live-state-contamination result.**

### M4 — Envelope Consistency & Observability
- **Maps to:** Phase 4 (envelope enrichment for existing `/sensor`
  replay) and Phase 5 (decision-buffer overflow observability) — treated
  as one milestone since they are sibling phases with no dependency on
  each other.
- **Deliverables:** `replay_sequence`/`overflow_preceded` on `/sensor`
  replay; a dedicated overflow counter and ETA metric for the decision
  buffer, exposed on `/status`.
- **Commit boundary:** Two commits (Phase 4, Phase 5), independently
  revertible, order-agnostic.
- **Regression gate:** Forced overflow of each buffer independently
  confirms its own counter increments and the *other* buffer's counter
  does not — proving isolation between the two overflow signals.
- **Acceptance criteria:** `/status`'s decision-buffer ETA metric
  decreases plausibly and monotonically during a simulated sustained
  outage, and both replay paths' envelopes are structurally consistent
  with each other and with ADR-0002's schema.
- **Exit criteria:** Gate and acceptance criteria both passed; tag as
  `v17.0-m4`.

### M5 — Release Candidate Hardening
- **Maps to:** No new Implementation Plan phase — this milestone is
  release-specific soak/hardening work, not architecture or
  implementation work, and introduces no new commits beyond
  configuration/test artifacts if any are needed.
- **Deliverables:** A release candidate build (`v17.0-rc1`) subjected to
  extended soak testing.
- **Commit boundary:** None expected; if a defect is found, its fix is
  scoped as its own single, minimal commit and re-enters this milestone
  for re-soak — it does not retroactively modify M0–M4's commits.
- **Regression gate:**
  - Minimum 72-hour continuous bench run with no unexplained reset,
    watchdog trigger, or heap-fragmentation trend.
  - At least 3 injected outages of varying duration (including one
    exceeding decision-buffer capacity, to exercise the accepted-loss
    path deliberately) during the soak window.
  - Full `PATCH_NOTES_v16.5.md`-style regression suite re-run (RUNNING/
    STOPPED gating of RMS/Peak/Crest Factor/Kurtosis/Frequency) to
    confirm this release has not regressed any pre-existing, already-
    shipped behavior.
- **Acceptance criteria:** Zero Sev1/Sev2 defects open at the end of the
  soak window; any Sev3 (cosmetic/non-functional) defects are explicitly
  triaged and accepted or fixed before proceeding.
- **Exit criteria:** Soak window complete, regression suite green, sign-
  off recorded; tag as `v17.0-rc-final`.

### M6 — General Availability (v17.0)
- **Maps to:** Release of the validated `v17.0-rc-final` build as `v17.0`.
- **Deliverables:** Tagged, signed-off firmware release; updated
  `CLAUDE.md` Feature History entry; this document's checklist (§10)
  fully checked off.
- **Commit boundary:** A single tag/release commit (version string bump
  only); no functional change from `v17.0-rc-final`.
- **Regression gate:** Identical binary (or byte-for-byte equivalent
  build from identical source) to what passed M5 — a version-string-only
  diff.
- **Acceptance criteria:** All prior milestones' exit criteria remain
  satisfied at release time (no drift since M5).
- **Exit criteria:** Release criteria (§6) fully met; production
  readiness checklist (§10) fully checked; explicit human sign-off
  recorded per this project's workflow rule against automatic,
  un-approved progression.

---

## 2. Deliverables (Summary)

| Deliverable | Milestone |
|---|---|
| Corrected queue documentation | M0 |
| `schema_version` / `domain` fields on all existing topics | M1 |
| Business Decision capture buffer | M2 |
| Business Decision replay path + new topic | M3 |
| `/sensor` replay envelope enrichment | M4 |
| Decision-buffer overflow observability | M4 |
| Soak-tested release candidate | M5 |
| Tagged, signed-off v17.0 firmware | M6 |

---

## 3. Commit Boundaries

One commit per Implementation Plan phase, no exceptions, in this order:

```
commit 1: M0 / Phase 6 — comment correction
commit 2: M1 / Phase 1 — schema_version + domain fields
commit 3: M2 / Phase 2 — decision buffer (capture only)
commit 4: M3 / Phase 3 — decision replay path
commit 5: M4 / Phase 5 — decision buffer overflow observability
commit 6: M4 / Phase 4 — /sensor replay envelope enrichment
[commit 7+: only if M5 soak testing surfaces a defect — each fix is its
 own additional, minimal commit, not a rewrite of commits 1-6]
commit N: M6 — version tag/release commit (string bump only)
```

Phases 4 and 5 (both M4) are commutable — order between them does not
matter, and either could be swapped without affecting any other
commit's correctness.

---

## 4. Regression Gates

A consolidated view of the gate at the end of each milestone (detail in
§1); no milestone begins before the prior gate has passed and been
explicitly signed off:

| Gate | Checks | Blocking? |
|---|---|---|
| G0 (end of M0) | Byte-identical compiled behavior | Yes |
| G1 (end of M1) | Additive-only field diff vs. v16.5 baseline | Yes |
| G2 (end of M2) | Buffer isolation; capture correctness across state cycle | Yes |
| G3 (end of M3) | Zero live-state contamination in replay; `/sensor` replay unaffected | Yes — **hardest gate, second-reviewer required** |
| G4 (end of M4) | Overflow-counter isolation; envelope consistency | Yes |
| G5 (end of M5) | 72h soak clean; full legacy regression suite green | Yes |
| G6 (end of M6) | No drift from `v17.0-rc-final`; checklist complete | Yes |

No gate may be skipped or bypassed under time pressure — a slipped
milestone date is preferable to a shipped gate failure, given the
business requirement this release exists to satisfy.

---

## 5. Acceptance Criteria (Per-Deliverable)

Restated compactly from §1 for quick reference:

- **M1:** New fields present and additive; zero existing-field
  regression; old parsers unaffected.
- **M2:** Captured decisions bit-identical to live-published decisions,
  100% match across ≥3 full state cycles.
- **M3:** Replayed decisions bit-identical to originally-buffered
  values, field-by-field, across a real outage/reconnect cycle.
- **M4:** Both buffers' overflow signals provably independent; envelope
  fields structurally consistent across both replay paths.
- **M5:** 72h soak clean, legacy regression suite green, zero open
  Sev1/Sev2.

---

## 6. Release Criteria (GA Gate for v17.0 as a Whole)

All of the following must be true before M6 tags `v17.0`:

1. Every milestone M0–M5's exit criteria are met and have not drifted.
2. Zero open Sev1/Sev2 defects; any open Sev3 defects explicitly
   triaged and accepted in writing.
3. The full legacy regression suite (from `PATCH_NOTES_v16.5.md`) passes
   unchanged — this release introduces no regression to already-shipped
   RUNNING/STOPPED gating behavior.
4. RAM/heap headroom after all five phases is measured, recorded, and
   confirmed to leave adequate margin for the current hardware
   configuration (per board-specific PSRAM/flash tuning).
5. Rollback strategy (§7) has been executed at least once in a bench
   environment and confirmed to restore v16.5 behavior cleanly.
6. Documentation is current: ADR-0001/0002/0003, the Implementation
   Plan, and this Release Plan all accurately reflect the shipped state;
   `CLAUDE.md`'s Feature History is updated with the new v17.0 entries.
7. Explicit, recorded human sign-off — this release does not proceed to
   fleet distribution automatically upon soak completion.

---

## 7. Rollback Strategy

Rollback is planned at two independent levels:

### Source-level rollback (pre-release, during development)
Every milestone is one or more independently revertible commits (§3).
A defect discovered before GA is fixed by reverting the offending
milestone's commit(s) and re-entering that milestone — never by
patching forward across multiple unreviewed commits.

### Field-level rollback (post-release, on deployed devices)
- The previous firmware image (v16.5) is retained, unmodified, and kept
  flashable/OTA-deployable as the rollback target for the entire v17.0
  release window.
- Because every v17.0 change is additive (§ Ground Rules), a downstream
  consumer that has not yet adopted v17.0's new fields/topics is
  unaffected by a rollback to v16.5 — there is no forward-only migration
  that a rollback would strand.
- Rollback trigger conditions (any one is sufficient): a Sev1 defect
  discovered post-GA affecting Business Decision correctness; a
  confirmed RAM/stability regression in the field not caught in soak;
  evidence of replay-path live-state contamination (the specific,
  highest-priority failure class this release is designed to prevent).
- Rollback procedure is device-by-device or fleet-wide depending on
  defect severity and blast radius — a canary-first rollout (see §10)
  is what limits the blast radius requiring rollback in the first place.
- Rollback is a firmware-only action. Because no Node-RED/InfluxDB/
  Dashboard change ships with v17.0, there is no coordinated downstream
  rollback required — those layers are unaffected either way.

---

## 8. Risk Matrix

| Risk | Likelihood | Impact | Mitigation |
|---|---|---|---|
| Replay path reads live state instead of buffered value (known failure class from prior analysis) | Medium | High — directly violates the business requirement this release exists to satisfy | G3 gate with mandatory second-reviewer sign-off; field-by-field replay verification, not spot-checking |
| RAM/heap exhaustion from new buffer + fields | Low-Medium | High — device instability/crash | Headroom measured at M2, re-confirmed at M5 soak; buffer sized deliberately small per ADR-0002 §7 |
| New shared state introduces a cross-core race | Low | High | Ground Rules mandate atomic/mutex-protected access only, matching existing project convention; reviewed at each phase's gate |
| MQTT broker flooding during a large replay burst (both buffers replaying concurrently) | Low-Medium | Medium | M3 regression gate explicitly tests concurrent replay of both paths under the existing rate-limiting scheme |
| Decision-buffer capacity mis-sized (too small: frequent accepted loss; too large: wasted RAM) | Medium | Medium | Explicitly flagged as an open question in ADR-0001/ADR-0003; capacity is a business/product decision to be confirmed before M2 capacity is finalized, not assumed |
| Reboot during an extended outage still loses buffered decisions (RAM-only buffer, accepted architectural limitation) | Low | Medium | Documented, accepted limitation (ADR-0003) — out of scope to fix in v17.0; must be disclosed in release notes, not silently accepted |
| Schema version mismatch confuses a downstream consumer not yet updated | Medium (Node-RED not being updated in this release) | Low-Medium | All new fields additive; old consumers ignore unknown fields safely; `schema_version` itself is inert until a consumer chooses to check it |
| Field technicians/dashboard viewers confused by two parallel streams (live vs. decision) | Medium | Low | Documentation-only mitigation for this release (no Dashboard change in scope); flagged for the Dashboard team's own release planning |
| Soak test window insufficient to surface a slow leak or rare race | Low-Medium | High | 72h minimum soak with injected outages is a floor, not a ceiling — extend if any anomaly trend is observed, do not release on a fixed calendar date over an unresolved trend |

---

## 9. Exit Criteria for Every Milestone (Consolidated)

| Milestone | Exit Criteria |
|---|---|
| M0 | Byte-identical compiled behavior confirmed |
| M1 | Additive-only field diff confirmed across full state cycle |
| M2 | Buffer isolation + 100% capture-match across ≥3 state cycles |
| M3 | Zero live-state contamination confirmed by second reviewer |
| M4 | Overflow-counter isolation + envelope consistency confirmed |
| M5 | 72h soak clean, legacy suite green, zero open Sev1/Sev2 |
| M6 | All release criteria (§6) met + recorded human sign-off |

---

## 10. Production Readiness Checklist

**Build**
- [ ] Clean `arduino-cli` build, FQBN: see `CLAUDE.md` § "Production Build
      Configuration" (do NOT use the bare `esp32:esp32:esp32s3` FQBN), zero
      warnings/errors
- [ ] Production library versions pinned and recorded (ESP32-S3 core,
      TinyGSM, ModbusMaster, PubSubClient, ArduinoJson) — per this
      project's existing build-requirement convention
- [ ] PSRAM/flash configuration confirmed correct for the target LilyGO
      T-Vending S3 board variant

**Functional Validation**
- [ ] Full legacy regression suite (RUNNING/STOPPED gating of RMS, Peak,
      Crest Factor, Kurtosis, Frequency) passes unchanged
- [ ] M1–M4 acceptance criteria all independently re-verified on the
      final `v17.0-rc-final` build (not only on intermediate milestone
      builds)
- [ ] Outage/reconnect cycle tested at multiple durations, including one
      exceeding decision-buffer capacity (accepted-loss path exercised
      deliberately, not just avoided)

**Longevity**
- [ ] 72-hour minimum soak completed with no unexplained reset or
      watchdog trigger
- [ ] Heap/stack headroom trend flat (no slow leak) across the soak
      window

**Documentation**
- [ ] ADR-0001/0002/0003 reflect the as-shipped architecture with no
      unresolved contradictions
- [ ] Implementation Plan and this Release Plan reflect the as-shipped
      state
- [ ] `CLAUDE.md` Feature History updated with v17.0 entries
- [ ] Release notes explicitly disclose the accepted architectural
      limitations (RAM-only buffer, reboot-during-outage loss, decision-
      buffer capacity boundary)

**Rollback Readiness**
- [ ] v16.5 image retained and confirmed flashable/OTA-deployable
- [ ] Rollback executed at least once in bench environment, confirmed
      clean

**Rollout Readiness**
- [ ] Canary/staged rollout plan defined (not fleet-wide on day one)
- [ ] Monitoring in place to observe the new `/status` overflow/ETA
      fields during canary rollout, even though Dashboard changes are
      out of this release's scope — someone must be watching the raw
      MQTT signal
- [ ] Explicit, recorded sign-off obtained before proceeding past
      canary to fleet-wide distribution

**Sign-off**
- [ ] All items above checked
- [ ] Named approver recorded for the GA release, per this project's
      workflow rule requiring explicit approval before any release step
      proceeds automatically
