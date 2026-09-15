# Live UAT — Real CRITICAL Event, Post-Deployment Forensic Validation
Recorded: 2026-09-15 (read-only investigation only — no production, Node-RED, firmware, API, frontend, or nginx file was modified; nothing was reloaded, restarted, or committed; no LINE message was sent; no test button was used)

---

## 1. Screenshot evidence classification

Unlike the prior WARNING event (`LIVE_UAT_WARNING_EVENT_20260915.md`), **no image file was attached to this conversation for this event.** Both the Dashboard and LINE evidence for this CRITICAL event are **OPERATOR-REPORTED TEXT DESCRIPTIONS ONLY**:

- **Dashboard screenshot: OPERATOR-REPORTED (text description only).** Reported figures: `pump01`, motor_state RUNNING, status CRITICAL, Velocity RMS 4.677 mm/s, Current 1.00 A, RPM 2803, "Attention: critical", "Recommendation: immediate inspection", dashboard time ≈ 12:58:47 Bangkok.
- **LINE screenshot: OPERATOR-REPORTED (text description only).** Reported: CRITICAL notification ≈ 12:56:43, Velocity RMS 6.93 mm/s, Motor State RUNNING, Source FIFO-DSP; LINE history additionally shows ≈12:58:18 WARNING 4.64 mm/s and ≈12:58:24 CRITICAL 4.68 mm/s.

This distinction is made deliberately, per this engagement's standing practice: nothing below should be read as "this session viewed an image" for either screenshot. Everything reported as directly observed in this document comes from this session's own independent tool calls (live API queries, historical trend queries, firmware source inspection, container log inspection), not from the screenshots themselves.

---

## 2. Independent evidence gathered by this session

### 2a. Live Machine Detail API (`GET https://dash.promlogix.com/api/machine/plant01/pump01`)

Two queries were made, both well after the reported event window:

| Field | Query @ 2026-09-15T06:02:25Z | Query @ 2026-09-15T06:06:48Z |
|---|---|---|
| `alarm_level` | NORMAL | NORMAL |
| `alarm_level_live` | true | true |
| `motor_state` | RUNNING (code 2) | RUNNING (code 2) |
| `current_a` / `current_valid` | 0.66 / true | 0.67 / true |
| `velocity_rms_overall_mms` | 0.304 | 0.285 |
| `velocity_rms_x/y/z_mms` | 0.135 / 0.203 / 0.182 | 0.135 / 0.186 / 0.169 |
| `vibration_status` | OK | OK |
| `velocity_data_valid` | true | true |
| `rpm` / `rpm_valid` | 1125.6 / true | 1124.6 / true |
| `last_seen` | 2026-09-15T06:02:04Z | 2026-09-15T06:06:37Z |

**This confirms the machine had already returned to a genuine NORMAL baseline by query time** — these two queries do **not** capture the CRITICAL event itself; the event had already resolved several minutes earlier. The RPM (~1125) is much lower than the screenshot's reported 2803, and vibration is back to ~0.3 mm/s — both consistent with a real, already-concluded transient event, not with any current abnormal condition.

### 2b. Live historical trend query (`GET https://dash.promlogix.com/api/machines/pump01/trend?range=1h`)

This is a genuine, independently-queried, minute-resolution historical series (InfluxDB-backed), not a screenshot and not a rebuild from memory. Full 61-point series retrieved; the relevant portion:

```
05:36:00Z  0.266 mm/s  A=0.670  T=43.9°C
05:37:00Z  1.585 mm/s  A=0.830  T=44.0°C   <- WARNING-range event begins
05:38:00Z  2.269 mm/s  A=0.898  T=44.0°C
...(plateau ~2.25-2.35 mm/s, A~0.86-0.90, T climbing 44.0->46.6, for ~12 min)...
05:49:00Z  2.331 mm/s  A=0.856  T=46.6°C
05:50:00Z  0.345 mm/s  A=0.680  T=46.9°C   <- decays back to baseline
05:51:00Z-05:56:00Z   ~0.33-0.34 mm/s, A~0.69-0.71, T~47.2-47.4°C (baseline)
05:57:00Z  5.107 mm/s  A=0.988  T=47.2°C   <- CRITICAL-range event begins
05:58:00Z  4.451 mm/s  A=0.994  T=47.1°C
05:59:00Z  4.582 mm/s  A=0.994  T=47.0°C
06:00:00Z  4.682 mm/s  A=0.985  T=47.0°C
06:01:00Z  2.458 mm/s  A=0.820  T=47.1°C   <- decaying
06:02:00Z  0.294 mm/s  A=0.665  T=47.3°C   <- back to baseline
06:03:00Z-06:05:58Z   ~0.29-0.32 mm/s, A~0.665-0.68 (baseline, matches 2a above)
```

This is real production telemetry, retrieved live for this investigation, independent of and prior to seeing the operator's reported numbers.

### 2c. Node-RED / mosquitto container logs

- Literal window specified in the task (`04:56:00–05:00:00 UTC`): **no log output at all.**
- Actual event window derived from the screenshots' own Bangkok timestamps (see §3 below), `05:35:00–06:07:00 UTC`: Node-RED produced exactly one line, an unrelated Node-RED-update notice (`A new version of Node-RED is available: 5.0.7`) — **no application-level output**. Mosquitto produced **zero** lines in this window.
- This absence is expected and not a gap introduced by this investigation: the LINE flow's function nodes report state via `node.status()` (visible only in the live editor UI, not the container's stdout) and do not call `node.warn`/`console.log` on ordinary alert processing, as already established in this engagement's prior audits. **No Node-RED/mosquitto log evidence exists for this event either way** — this investigation relies on the independent API/trend evidence (§2a, §2b) and firmware source inspection (§4) instead.

### 2d. Firmware source (read-only inspection only, no modification)

`WTVB02_ESP32S3_V16_CM_fault_latch_v3_hardened_patched_v16_5.ino` (current production source, read-only):
```
#define VIB_WARNING_MMS       2.1f   // WARNING_ON
#define VIB_CRITICAL_MMS      4.5f   // CRITICAL_ON
#define VIB_WARNING_OFF_MMS   1.9f   // WARNING_OFF
#define VIB_CRITICAL_OFF_MMS  4.2f   // CRITICAL_OFF
#define VIB_ALARM_PERSIST_CAPTURES 2u
```
**This is a materially different state than the prior LINE audit recorded.** `LINE_NOTIFICATION_AUDIT.md` §12(a) (this engagement, earlier the same day) stated real firmware could not produce a genuine CRITICAL verdict because `vibThresholdsConfigured()` returned false (thresholds unset). That is **no longer true**: the current source has real, non-zero, correctly-ordered ON/OFF thresholds, so `vibThresholdsConfigured()` now returns `true` and a genuine firmware-sourced CRITICAL verdict is structurally possible. (This session did not investigate when/how this threshold change was made — that is outside this task's read-only Node-RED/API forensic scope — it is reported only because it directly bears on item 9 below.)

---

## 3. Timestamp alignment analysis

**Discrepancy found and disclosed, not silently corrected:** the task's literally-specified log window (`2026-09-15 04:56:00–05:00:00 UTC`) does not correspond to the screenshots' own reported times. Converting the screenshots' stated Bangkok (ICT, UTC+7) times:

| Screenshot time (ICT) | UTC |
|---|---|
| 12:56:43 | 05:56:43 |
| 12:58:18 | 05:58:18 |
| 12:58:24 | 05:58:24 |
| 12:58:47 | 05:58:47 |

The task-specified window is exactly **one hour earlier** than the window implied by the screenshots' own timestamps. This investigation used the Bangkok-derived window (`05:35–06:07 UTC`) as the actual event window, since it is internally consistent with the screenshots' own stated times and is corroborated by real trend data showing a genuine event precisely there — the literal `04:56–05:00 UTC` window contains no activity of any kind (§2c).

Alignment of each reported figure against the independently-queried trend series (§2b):

| Reported (screenshot) | UTC time | Nearest independent trend sample(s) | Assessment |
|---|---|---|---|
| LINE CRITICAL 6.93 mm/s | 05:56:43 | 05:56:00=0.335, **05:57:00=5.107** | Real, sharp rise from baseline confirmed within the same minute (17s after screenshot time). Exact value differs (6.93 vs 5.107) because these are different points in a fast-changing signal, not the same 1-minute aggregate. |
| LINE WARNING 4.64 mm/s | 05:58:18 | 05:58:00=4.451 | Same order of magnitude, 18s apart; consistent with a continuously-evolving signal, not identical because trend endpoint reports 1-minute snapshots, not the same sub-second sample. |
| LINE CRITICAL 4.68 mm/s | 05:58:24 | 05:58:00=4.451, 05:59:00=4.582 | Sits just above both neighboring 1-minute samples — consistent with normal short-timescale measurement noise on a slowly-declining signal. |
| Dashboard CRITICAL 4.677 mm/s | 05:58:47 | 05:59:00=4.582, 06:00:00=4.682 | Falls almost exactly between these two real samples, essentially identical to the 06:00:00 sample (4.682 vs 4.677, Δ0.005 mm/s). |
| Dashboard current 1.00 A | 05:58:47 | 05:58:00=0.994, 05:59:00=0.994, 06:00:00=0.985 | Matches to within 0.01–0.02 A. |
| Dashboard RPM 2803 | 05:58:47 | not present in trend endpoint (vibration/current/temp only) | Cannot be independently corroborated from this API; not contradicted by anything gathered. |

**Conclusion:** every reported number aligns, within the expected tolerance of comparing a screenshot instant against 1-minute-aggregated independent data, to a real, continuously-evolving physical signal — not to a static or fabricated value.

---

## 4. Dashboard vs LINE verdict comparison

Both sources' **most recent/final state within the event** report **CRITICAL**:
- LINE's own history shows its last state before the Dashboard screenshot was CRITICAL (4.68 mm/s @ 05:58:24), 23 seconds before the Dashboard screenshot's own timestamp (05:58:47).
- Dashboard shows CRITICAL (4.677 mm/s) at 05:58:47.

**Same authoritative verdict (CRITICAL) confirmed on both paths, at approximately the same real-world instant.**

### Explaining the "4.677 vs 6.93" RMS difference

**This is not a Dashboard/LINE inconsistency — it is a comparison between two different points in time on the same decaying curve, not simultaneous readings:**
- LINE's 6.93 mm/s is from **05:56:43** — the *initial spike* of the event (confirmed against real trend data: baseline 0.335→5.107 mm/s within that exact minute).
- Dashboard's 4.677 mm/s is from **05:58:47** — over **two minutes later**, well into the event's decline (confirmed against real trend data: 4.582→4.682 mm/s at that time).
- If the Dashboard reading is instead compared against LINE's *own later* CRITICAL reading (4.68 mm/s @ 05:58:24, 23 seconds before the Dashboard screenshot), the two values are **essentially identical** (4.677 vs 4.68 mm/s).

**Not a defect.** The apparent "difference" arises only from comparing the operator's two screenshots at their face-value single instants without accounting for the ~2-minute gap between them, during a real event whose RMS was genuinely falling (5.1 → 4.45 → 4.58 → 4.68 mm/s across 05:57–06:00, per independently-queried trend data). No evidence suggests Dashboard and LINE disagreed about the vibration level at any single instant.

---

## 5. CRITICAL → WARNING → CRITICAL sequence analysis

Reported sequence: CRITICAL 6.93 (05:56:43) → WARNING 4.64 (05:58:18) → CRITICAL 4.68 (05:58:24), the last two only 6 seconds apart.

**This matches the current firmware's documented hysteresis + escalation-persistence state machine** (read-only inspection, lines ~7011–7062 of the current `.ino`):
```
CRITICAL_ON=4.5, CRITICAL_OFF=4.2, WARNING_ON=2.1, WARNING_OFF=1.9
VIB_ALARM_PERSIST_CAPTURES = 2   // escalation requires 2 agreeing captures; de-escalation is immediate
```
The state machine compares each new FIFO capture against a band that depends on the *currently held* state (hysteresis), and any **escalation** (state increasing) additionally requires two consecutive, distinct FIFO captures to agree before it is applied — one transient sample can never raise an alarm; **de-escalation applies immediately**. Given a value ≥4.5 mm/s arriving while the held state is WARNING, the candidate state becomes CRITICAL, but if this is only the *first* qualifying capture, the reported/held state remains WARNING one more cycle; the *next* qualifying capture (here, 6 seconds later) satisfies the 2-capture requirement and CRITICAL is then actually applied. This is structurally exactly the shape observed: a brief WARNING classification at 4.64 mm/s (a value that, taken alone, is already past the CRITICAL_ON edge) followed 6 seconds later by CRITICAL at 4.68 mm/s.

**Caveat, stated plainly:** the trend endpoint used in §2b/§3 reports 1-minute aggregates, not the individual sub-minute FIFO captures the firmware's decision engine actually evaluates, so this session cannot replay the exact sample-by-sample arithmetic for this specific 6-second window from that data alone. The claim above is a structural/mechanism-level match (the mechanism that produces exactly this kind of "WARNING-then-CRITICAL-6-seconds-later" pattern genuinely exists in the current firmware and is not hypothetical), not a byte-for-byte replay of this specific transition. No Node-RED or mosquitto log evidence exists to replay it at finer granularity either (§2c).

**This sequence does not match the known test-injection fingerprint.** Per `LINE_NOTIFICATION_AUDIT.md` §12(b), the two manual test-inject buttons emit fixed, single-shot legacy-shape payloads that **omit `velocity_data_valid`/`velocity_rms_overall` entirely**, which forces `vibration_measure_status = 'UNAVAILABLE'` and produces `"N/A"` in the LINE message — never a real numeric RMS value. Every value in this event (6.93, 4.64, 4.68, 4.677 mm/s) is a real, non-"N/A" number, and the independently-queried trend data (§2b) shows a **continuous, multi-minute rise-and-decay curve correlated across three independent channels** (vibration, current, temperature) — a shape a single manual button click cannot produce. This does not, by itself, prove human origin is impossible, but it is affirmative evidence against the specific test-button mechanism previously documented, and no evidence of a manual click (e.g., an editor-side log) exists in either direction.

---

## 6. Verification checklist (task item 8)

- **Motor State = RUNNING in both paths:** confirmed from the operator's own reported text for both Dashboard (`motor_state: RUNNING`) and LINE (`Motor State: RUNNING`); independently corroborated by both live API queries in §2a, which show `motor_state: RUNNING` continuously through and after the event.
- **"Health" wording gone from LINE / "Motor State" wording used:** the operator's own reported LINE text uses `Motor State: RUNNING`, not `Health: ...` — consistent with the deployed patch, matching the structural guarantee already independently verified in `LINE_PATCH_FINAL_REVIEW_20260915.md` §3 (the string `"Health:"` does not occur anywhere in the deployed LINE Message Builder source).
- **No RPM-validity condition changed the vibration verdict:** this event does not, on its own, add new evidence here (no Node-RED payload log exists to inspect for this specific event, §2c). It relies on the same structural guarantee already established and re-verified twice in this engagement (`LINE_PATCH_FINAL_REVIEW_20260915.md` §5): `rpm_valid` appears only in a code comment in Health Logic and is never read by any executable branch of either patched node, so it cannot have influenced this or any event's LINE/Dashboard vibration verdict. Both API queries in §2a additionally show `rpm_valid: true` throughout, so this event does not even present the RPM-invalid case.

---

## 7. On the possibility of a test-button click (task item 9)

**No evidence available to this investigation proves or disproves a manual test-button click.** No Node-RED editor click-log exists; container logs are silent (§2c). This document does **not** conclude the event was a test-button click.

What the available evidence does show, stated only as far as it goes:
- The event's shape (continuous multi-minute rise/plateau/decay, correlated across vibration + current + temperature, real non-"N/A" numeric values throughout) is **inconsistent with the specific, previously-documented test-inject-button fingerprint** (instantaneous fixed values, `UNAVAILABLE`/`N/A`, no correlated current/temperature change).
- The firmware's vibration thresholds, previously documented as unset (making a genuine firmware CRITICAL impossible), are now configured with real values, so a genuine firmware-sourced CRITICAL is structurally possible today, which was not true at the time of the original audit.

Taken together, this is evidence **consistent with a genuine, firmware-sourced physical event**, but it is not proof, since this investigation had no access to a definitive source (e.g., an editor click log, or a payload-level MQTT capture) that could rule out manual injection with certainty.

---

## 8. Final classification

Basis for the verdict:
- Dashboard and LINE's most-recent states within the event both report **CRITICAL**, at approximately the same real-world instant (§4).
- The reported RMS "difference" is fully explained by a ~2-minute timestamp gap between the two screenshots on a genuinely decaying real signal, not a Dashboard/LINE inconsistency (§4).
- Motor State = RUNNING and "Motor State" (not "Health") wording confirmed on both paths (§6).
- No RPM-validity condition could have affected the vibration verdict (structural guarantee, §6).
- Independent, live-queried historical trend data corroborates a real, continuous, multi-channel physical event at the times and magnitudes the screenshots describe (§2b, §3).

```
LIVE UAT — CRITICAL — DASHBOARD/LINE VERDICT CONSISTENCY = PASS
```

- Dashboard screenshot = OPERATOR-REPORTED (text description only; no image viewed by this session)
- LINE screenshot = OPERATOR-REPORTED (text description only; no image viewed by this session)
- Verdict consistency = OBSERVED PASS (both CRITICAL, same event, same approximate instant, independently corroborated via live API + historical trend query — not screenshot-only)
- Numeric RMS equality = NOT REQUIRED and not a defect (explained by ~2-minute timestamp gap on a real decaying signal, per §4)
- Test-button origin = NOT INFERRED, NOT PROVEN EITHER WAY (§7); available evidence is consistent with, but does not prove, genuine firmware origin

---

## 9. Scope confirmation

- No production file was modified. No Node-RED flow, firmware source, API, frontend, or nginx configuration was changed.
- No container was restarted or reloaded.
- No LINE message was sent; no test-inject button was used.
- All API calls made (`GET /api/machine/plant01/pump01`, `GET /api/machines/pump01/trend?range=1h`) are read-only public endpoints already exercised repeatedly elsewhere in this engagement.
- Firmware source was read-only inspected (`grep`/`Read`), never edited.
- This document has not been committed, per instructions.
- No LINE credential or token value is reproduced anywhere in this document.

---
*Read-only forensic validation. No production code or configuration was modified while recording this evidence.*
