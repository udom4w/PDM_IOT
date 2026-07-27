# P4-02 — Final Hardware Verification Package

**Status:** Preparation only. No firmware modified. No documentation changed.
Build to use for all remaining tests: the **Production Build Configuration**
(`CLAUDE.md`), FQBN
`esp32:esp32:esp32s3:CDCOnBoot=cdc,CPUFreq=240,FlashMode=qio,FlashSize=16M,PSRAM=opi,PartitionScheme=app3M_fat9M_16MB,USBMode=hwcdc,UploadMode=default,UploadSpeed=921600`
— Serial **and** MQTT are both available for these tests, unlike the earlier
MQTT-only phase of this session.

## 1. Review of Existing P4-02 Hardware Verification Status

Per `P4_02_HARDWARE_VERIFICATION_PLAN.md`'s test matrix:

| Test | Status | Basis |
|---|---|---|
| 1 — Power-on | Evidence already captured, not yet formally logged as a P4-02 test-case record | The `CDCOnBoot=cdc` diagnostic experiment (this session) captured a complete, clean boot on the exact Production Build: Serial showed the full boot sequence from `[CONFIG] Press ENTER...` through task creation with no gaps, and MQTT reached `CONNECTED (mTLS)` and published normally. This satisfies Test 1's intent but was captured as a build-validation artifact, not recorded test-case-by-test-case the way Tests 2/3 were. Recommend formally accepting it as PASS rather than repeating the action, unless you want a dedicated re-run. |
| 2 — Motor STOPPED, comms healthy | **PASS** (recorded) | MQTT: `current_evidence_valid=true` stable, `motor_state=0`, `current_read_errors=0`. Serial: NOT OBSERVED at the time (root cause since found and fixed), did not affect the verdict. |
| 3 — Motor RUNNING, comms healthy | **PASS** (recorded) | MQTT: `current_evidence_valid=true` stable, `motor_state=2`, `current_read_errors=0`. Serial: NOT OBSERVED at the time, did not affect the verdict. |
| 4 — Short interruption | **Not closed** | An MQTT capture was submitted and analyzed, but the evidence showed a genuine motor stop (rpm/rotation_signal_ok→0), not a CTR4A01 comms interruption — `current_read_errors` never moved and `current_evidence_valid` never reacted at the transition. Does not close Test 4. |
| 5 — Long interruption | **Not executed** | No attempt yet. |
| 6 — Recovery | **Not executed** | Depends on Test 5 data (analysis pass, not a separate physical action). |
| 7 — Long-term stability | **Not executed** | No attempt yet. |

## 2. Test Cases Already PASS

- **Test 2** — Motor STOPPED, comms healthy.
- **Test 3** — Motor RUNNING, comms healthy.
- **Test 1** — evidence exists (see above); recommend formal acceptance, not a required re-run, but listed separately from 2/3 since it wasn't logged in the same test-case-record format.

## 3. Remaining Test Cases Required to Close P4-02

- **Test 4** — Short (sub-threshold) CTR4A01/RS485 interruption.
- **Test 5** — Long (over-threshold) CTR4A01/RS485 interruption.
- **Test 6** — Communication recovery (reseed-vs-blend analysis of Test 5's — and optionally Test 4's — reconnect edge; not a new physical action).
- **Test 7** — Long-term stability (repeated fault cycles + an extended stable-running soak).

## 4. Detailed Test Procedures

All four tests should be run in one continuous physical session where practical
(mirrors the P4-01 fault-injection session's approach), with one continuous
Serial capture spanning Tests 4→7, and your MQTT subscriber running in
parallel throughout. `CURRENT_EVIDENCE_MAX_AGE_MS = 5000 ms` for all threshold
references below.

---

### Test 4 — Short (sub-threshold) CTR4A01/RS485 interruption

**Objective:** confirm `current_evidence_valid` (and its Serial-visible source,
`s_currentEvidenceValid` via the `[CURRENT_DECISION]` proxy) stays `true`
throughout a communication gap that never exceeds 5000 ms — no false
invalidation on a brief comms blip (AC7).

**Test steps:**
1. Motor RUNNING, CTR4A01 wired, comms healthy. Baseline ~20 s.
2. Disconnect CTR4A01/RS485 wiring for **~2 seconds**.
3. Reconnect.
4. Settle ~30 s (covers at least one MQTT publish either side of the event).

**Expected MQTT fields (`/vibration`):**
- `current_evidence_valid`: stays `true` at every publish surrounding the test — no transition to `false` expected or required.
- `current_read_errors`: may increment by 0 or 1 (depends on whether the ~2s gap happened to overlap an in-flight poll attempt).
- `motor_state`: may show a transient RUNNING→STOPPING→RUNNING flicker — this is pre-existing FSM behavior (`ageMs>ageStoppingMs`), unrelated to P4-02's scope; not a fail condition by itself.

**Expected Serial logs:**
- `[CURRENT] FAIL rc=0xE2 (ku8MBResponseTimedOut)` — zero or one occurrence.
- `[CURRENT_DECISION]`: `ageMs` rises but stays **< 5000**; `ema_current` continues the normal EMA blend formula (`0.25*new + 0.75*old`) — never resets to `0.000`.
- No `[SIGNAL] 1->0` transition (latch should not drop, since the filtered value never resets).
- `[MOTOR-TRANSITION]` RUNNING→STOPPING→RUNNING is expected/benign (existing FSM timing, not this feature).

**PASS criteria:** peak observed `ageMs` stays below 5000 ms throughout; `ema_current` never resets to `0.000` (reseed branch never taken); `current_evidence_valid` never shows `false` at any MQTT publish during/after the test.

**FAIL criteria:** `ema_current` resets to `0.000` despite `ageMs` staying under 5000 (false invalidation — real defect); OR `current_evidence_valid` shows `false` in MQTT while Serial shows `ageMs<5000` throughout (mirror desync — violates the P4-02 Design Contract's I1/AC1 invariant).

---

### Test 5 — Long (over-threshold) CTR4A01/RS485 interruption

**Objective:** confirm `current_evidence_valid` transitions to `false` when `ageMs` exceeds 5000 ms, in lockstep with `s_currentEvidenceValid` (AC1).

**Test steps:**
1. Motor RUNNING, comms healthy. Baseline ~20 s.
2. Disconnect CTR4A01/RS485 for **at least 7–10 seconds** (comfortable margin over the 5000 ms threshold). Since Serial is now available, exact timing precision matters less than in the MQTT-only phase — Serial will show the invalidation moment directly regardless of MQTT's 30s publish cadence. (Optional: extend to ≥35s if you also want to guarantee an MQTT publish lands showing `false` directly, for extra corroboration.)
3. Reconnect.
4. Settle ~30 s.

**Expected Serial logs:**
- `[CURRENT] FAIL rc=0xE2` — one or more occurrences.
- `[CURRENT_DECISION]`: `ageMs` climbs past 5000; in that same print, `ema_current=0.000` and `signalPresent=0` appear together (this is the invalidation firing).
- `[SIGNAL] 1->0`.
- `[MOTOR-TRANSITION]` to STOPPED, `reason=ageMs>ageStoppedMs` (same cycle as the evidence invalidation — the "lockstep" guarantee from the P4-01 Design Contract).

**Expected MQTT fields (`/vibration`):**
- `current_evidence_valid`: `false` on any publish that lands during the outage (guaranteed if held ≥35s; best-effort otherwise).
- `motor_state`: likely `0` (STOPPED) during/after, depending on timing overlap with the FSM's own absence thresholds.

**PASS criteria:** Serial directly shows `ageMs>5000` coincident with `ema_current=0.000`/`signalPresent=0` in the same print block (this is the authoritative check); if an MQTT publish lands in the window, it shows `current_evidence_valid=false` at that time, with no lag beyond one snapshot cycle.

**FAIL criteria:** `ageMs` exceeds 5000 but `ema_current`/`signalPresent` do not reset (evidence-layer regression); OR MQTT ever shows `current_evidence_valid=true` while Serial simultaneously shows `ageMs>5000` with `ema_current=0.000` (mirror/pipeline bug).

---

### Test 6 — Communication recovery (reseed vs. blend)

**Objective:** confirm the first fresh sample after invalidity reseeds directly (not a blend with the stale/reset value), and that `current_evidence_valid` flips back to `true` at that same moment (AC3). **This is an analysis pass over Test 5's (and optionally Test 4's) reconnect edge — no new physical action.**

**Test steps:** none beyond Tests 4/5 — analyze the captured logs' reconnect moments.

**Expected Serial logs:** at the first `[CURRENT_DIAG]`/`[CURRENT_DECISION]` pair after Test 5's reconnect, `ema_current` on that first fresh sample equals the raw `engineeringA` value **exactly** (reseed, not blend). The second fresh sample onward follows the normal blend formula again — reproduce the arithmetic to 3 decimal places, the same method used in the P4-01 fault-injection report.

**Expected MQTT fields:** `current_evidence_valid` flips to `true` on the publish coincident with or immediately after the reseed edge.

**PASS criteria:** exact arithmetic match confirming reseed-not-blend on sample 1, normal blend resuming on sample 2; MQTT `current_evidence_valid` returns to `true` within one publish cycle of the Serial-confirmed recovery. (Test 4's reconnect, as a negative control, should show **no** reseed edge at all, since evidence never invalidated.)

**FAIL criteria:** first post-recovery sample shows a blended value instead of an exact match (reseed logic regression); or `current_evidence_valid` stays `false` for more than one publish cycle after Serial confirms evidence is valid again.

---

### Test 7 — Long-term stability

**Objective:** confirm no oscillation or classification drift across repeated fault cycles, and no regressions accumulate over an extended soak on the validated Production Build.

**Test steps:**
1. Motor RUNNING, comms healthy. Baseline ~20 s.
2. Repeat **twice**: disconnect ~2 s → reconnect → settle 15–20 s → disconnect ~7–10 s → reconnect → settle 15–20 s.
3. After the repeated-cycle portion, let the system run **undisturbed, comms healthy, for an extended soak** (recommend ≥10 minutes minimum; longer if practical).

**Expected Serial logs:** each disconnect event's `ageMs`/`ema_current` pattern classifies consistently with its own measured duration (sub-threshold events never reset; over-threshold events do — matching Tests 4/5's individual criteria) across all repetitions; zero crash signatures (`Guru Meditation`, `Backtrace`, `task_wdt`, `Brownout detector`) at any point; no unexpected reboot (cross-check `reboot_count` in the periodic status report — must stay constant).

**Expected MQTT fields:** `current_evidence_valid` mirrors the Serial-observed pattern at each publish; `current_read_errors` climbs monotonically, consistent with the actual count of real failures seen in Serial; no `motor_state` flapping beyond what each individual disconnect's own duration should produce.

**PASS criteria:** every cycle reproduces the correct classification with no oscillation; the soak window completes with zero crash signatures, zero unexpected reboots, and continuous, stable `current_evidence_valid=true` reporting while comms remain healthy throughout the soak.

**FAIL criteria:** any cycle's classification is inconsistent with its own measured `ageMs`; any rapid true/false oscillation within a single physical event; any crash signature or unexpected reboot during the soak window.

## 5. Hardware Test Checklist

**Pre-test:**
- [ ] Flashed with Production Build FQBN (`CDCOnBoot=cdc,...` — confirm exact string matches `CLAUDE.md`).
- [ ] Serial capture running (continuous, spanning all tests below).
- [ ] MQTT subscriber running in parallel, logging `/vibration` with timestamps.
- [ ] Motor running, CTR4A01 wired, comms healthy — baseline confirmed (`current_evidence_valid=true`, `motor_state=2`, `current_read_errors` noted).

**Test 4 — Short interruption:**
- [ ] Disconnected CTR4A01/RS485 ~2 s.
- [ ] Reconnected.
- [ ] Settled ≥30 s.
- [ ] Serial: peak `ageMs` < 5000 confirmed.
- [ ] Serial: `ema_current` never reset to 0.000.
- [ ] MQTT: `current_evidence_valid` stayed `true` throughout.
- [ ] Verdict recorded: PASS / FAIL / NOT OBSERVED.

**Test 5 — Long interruption:**
- [ ] Disconnected CTR4A01/RS485 ≥7–10 s (or ≥35 s if targeting direct MQTT capture).
- [ ] Reconnected.
- [ ] Settled ≥30 s.
- [ ] Serial: `ageMs>5000` coincident with `ema_current=0.000`/`signalPresent=0` confirmed.
- [ ] Serial: `[SIGNAL] 1->0` and `[MOTOR-TRANSITION]` to STOPPED confirmed.
- [ ] MQTT: `current_evidence_valid=false` observed (if a publish landed in-window).
- [ ] Verdict recorded: PASS / FAIL / NOT OBSERVED.

**Test 6 — Recovery (analysis only):**
- [ ] First post-recovery sample's `ema_current` matches raw `engineeringA` exactly (reseed).
- [ ] Second sample onward resumes normal blend arithmetic (verified to 3 decimals).
- [ ] MQTT `current_evidence_valid` returned to `true` within one publish cycle.
- [ ] Test 4's reconnect confirmed to show no reseed edge (negative control).
- [ ] Verdict recorded: PASS / FAIL.

**Test 7 — Long-term stability:**
- [ ] Cycle 1: ~2 s disconnect → reconnect → settle 15–20 s.
- [ ] Cycle 1: ~7–10 s disconnect → reconnect → settle 15–20 s.
- [ ] Cycle 2: ~2 s disconnect → reconnect → settle 15–20 s.
- [ ] Cycle 2: ~7–10 s disconnect → reconnect → settle 15–20 s.
- [ ] Extended soak ≥10 minutes, comms healthy, undisturbed.
- [ ] Zero crash signatures across the full session.
- [ ] `reboot_count` unchanged across the full session.
- [ ] Every cycle's classification consistent with its own measured `ageMs`.
- [ ] Verdict recorded: PASS / FAIL.

**Post-test:**
- [ ] Complete Serial log saved.
- [ ] Complete MQTT log saved.
- [ ] All verdicts (Tests 4–7) recorded with evidence excerpts.

---

No firmware modified. No documentation changed. This is the preparation
package only — execution and results are the next step.
