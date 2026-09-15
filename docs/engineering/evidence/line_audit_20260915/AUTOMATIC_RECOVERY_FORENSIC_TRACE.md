# Forensic Trace — CRITICAL → NORMAL Automatic Recovery, 2026-09-15
Event under investigation: CRITICAL at ~2026-09-15 11:35:20 Bangkok (04:35:20 UTC) → NORMAL at ~11:36:13 Bangkok (04:36:13 UTC).

Read-only throughout: no file modified, no message sent, no button clicked, nothing restarted or reconfigured.

> **Correction to the prior audit, made on the strength of evidence found in this trace, not on the strength of being told to correct it:** the prior `LINE_NOTIFICATION_AUDIT.md` speculated the manual test-inject buttons were the likely cause of a CRITICAL→NORMAL pair ~53 seconds apart. Direct log evidence gathered below **does not support that** for this specific event and in fact actively contradicts it (see Section 6/7). That earlier speculation is superseded by this document for this event.

---

## 1. Flow, trigger path, and state storage (unchanged from prior audit — restated for completeness)

```
a375c54a616839e2 mqtt in → 885a21b2d935bf2d Identity Extractor → lat001calc2026fix1 Latency Calculator
 → adf3dc5f003a516c Health Logic → ef04bfbbd91995e0 Unified State Engine → e982d76b3ebe0b06 LINE Message Builder
 → 830a5573f64cf991 LINE Push API → f63614552d4d191e 429 Handler
```
- **CRITICAL entry condition:** `Unified State Engine` accepts any `payload.alarm_level` value it receives and stores it as the machine's new `state` if it differs from the currently-stored state (`m.state !== newLevel || m.latched`). There is no independent CRITICAL-entry threshold check inside Node-RED — `alarm_level` is computed entirely upstream, in firmware, and taken at face value here.
- **Recovery condition:** identical mechanism — `newLevel === 'NORMAL'` sets `msg._alertType = 'recovery'`. No separate "recovery-specific" gate exists.
- **Recovery timer/debounce/hold:** none. Rule 1 (`m.state === newLevel && !m.latched → drop`) only suppresses a *repeated identical* state; it does not impose any minimum dwell time before accepting a *changed* state. A CRITICAL→NORMAL pair less than a minute apart is structurally unremarkable to this code — nothing here would reject or delay it.
- **Previous-state tracking:** `flow.get('machines')[machineId]`, an in-memory (RAM, not persisted) flow-context object, one entry per machine, holding `{state, latched, acknowledged, snoozeUntil, lastEscalation, backoffUntil, stateChangedAt, lastAlertSentAt}`.

## 2. What recovery is actually based on

**Confirmed from source: `payload.alarm_level` alone.** Not `vibration_status`, not `vibration_measure_status`, not `velocity_data_valid`, not an RMS threshold evaluated in Node-RED, not a count of consecutive samples, not elapsed time. The Unified State Engine reads exactly one field (`msg.payload.alarm_level`) to decide CRITICAL/WARNING/NORMAL. Everything else in the payload (vibration fields, health_score) is carried through unexamined for use later in the LINE *message text*, not in the *verdict decision*.

## 3. Reconstructed likely transition

Based on log evidence (Sections 6–7 below), the most defensible reconstruction is:

```
Real acquisition/sensor degradation (vibration unavailable + elevated current-read
errors), ongoing continuously for over an hour
  → firmware-side condition clears around 11:34:58–11:35:20 Bangkok
  → alarm_level (firmware-computed, from a source other than the vibration-RMS
    threshold path — see Section 4) transitions CRITICAL → NORMAL within this window
  → Unified State Engine accepts both transitions at face value (no debounce)
  → two LINE messages sent ~53s apart, the first tagged 'alert' (CRITICAL), the
    second tagged 'recovery' (NORMAL)
```

## 4–5. Why the recovery message could show "Velocity RMS: N/A (FIFO-DSP unavailable)"

> **⚠️ CORRECTION — 2026-09-15 (later the same day), per `THRESHOLD_CONFIGURATION_RECONCILIATION_20260915.md`:**
> The opening claim of this section — "Firmware currently cannot produce a live CRITICAL verdict from the vibration-RMS threshold path at all... `vibThresholdsConfigured()` returns `false` unconditionally" — **was factually incorrect**, both at the moment this document was written and at the time of the 11:35:20 event itself. The current, and long-standing, production thresholds are `VIB_WARNING_MMS = 2.1 mm/s` and `VIB_CRITICAL_MMS = 4.5 mm/s` (OFF-thresholds 1.9 / 4.2 mm/s, 2-capture escalation persistence), set by commit `c55853a2` on 2026-08-27 — before this event occurred, not after. This document inherited the error by citing "an earlier audit this engagement" (`LINE_NOTIFICATION_AUDIT.md` §12(a)), which has itself now been corrected. **This does not, by itself, prove the 11:35:20 CRITICAL was vibration-RMS-sourced rather than a fault-latch condition** — the fault-latch hypothesis below may still be correct for independent reasons (the `current_read_errors` spike is real, unaffected evidence) — but it *does* remove the false premise that a vibration-RMS-threshold origin was structurally impossible for this event. The original reasoning below is preserved unedited for provenance; its conclusion should not be cited as ruling out a genuine vibration-threshold cause. **This correction strengthens, rather than weakens, the interpretation that a real CRITICAL/WARNING event — including the later, independently-documented event in `LIVE_UAT_CRITICAL_EVENT_20260915.md` — can be genuinely firmware-sourced.**

**Firmware currently cannot produce a live CRITICAL verdict from the vibration-RMS threshold path at all** — this was established in an earlier audit this engagement: `vibThresholdsConfigured()` returns `false` unconditionally (`VIB_WARNING_MMS`/`VIB_CRITICAL_MMS` are documented unset sentinels). So whatever produced `alarm_level: "CRITICAL"` at 11:35:20 was **not** a vibration-RMS-threshold breach — it must have come from a different firmware alarm source (most plausibly a fault-latch condition, e.g. tied to the acquisition/current-sensor problem — see the `current_read_errors` spike in Section 7). This is exactly the same architectural gap flagged in the prior audit: **`alarm_level` and vibration-availability are computed by two independent subsystems in firmware, and nothing in the Node-RED LINE path cross-checks them.** A real, non-vibration-sourced alarm transition can occur at the exact same time vibration data is unavailable or only just recovering — the LINE Message Builder has no way to know these two facts came from different subsystems, and no reason to withhold the message. *(See correction note above — the premise that a vibration-RMS-threshold origin was impossible was factually incorrect.)*

Answering item 5 directly — "N/A" in this event most likely refers to **current missing data at the moment that specific message was built** (i.e., the literal, real-time `velocity_data_valid`/`velocity_rms_overall` state of the payload the LINE Message Builder processed), not a stale field, not a legacy fallback, and not a different-payload mismatch in the sense of a code bug — it is the correctly-computed, honest output of the `[Notify-Fix-1]` guard (`p.velocity_data_valid === true && typeof p.velocity_rms_overall === 'number'`), faithfully reporting that vibration genuinely was not available in the specific message that triggered that particular LINE push. The defect is not that "N/A" is wrong — it's that the *alarm verdict* was allowed to say "NORMAL" (implying "safe to relax") in the same message that admits the sensor data behind that verdict is unavailable, with nothing to reconcile or flag the contradiction.

**One evidentiary limit, stated plainly:** I did not obtain the literal byte-for-byte content of the two specific MQTT messages that drove the 11:35:20 and 11:36:13 transitions. Node-RED's Health Logic node only emits a console warning on the DROP/UNAVAILABLE paths — it does not log payload content on a successful pass, and no message was captured live during this exact window (this investigation started well after the event). Obtaining the literal historical payloads would require either a live MQTT capture at the time (not possible after the fact) or a direct, credentialed InfluxDB query of the raw `vibration` measurement around that timestamp — which was deliberately not done, consistent with this engagement's standing practice of not extracting or using database/MQTT credentials. The reconstruction above is therefore **strongly evidenced, not directly proven at the individual-message level.**

## 6–7. Actual log evidence for the 11:35→11:36 window, and the two test buttons

**Direct evidence found (all timestamps UTC; Bangkok = UTC+7):**

```
15 Sep 03:27:03 through 04:34:58 UTC (10:27:03–11:34:58 Bangkok):
  Continuous "[Health] ⚠ VIB_UNAVAILABLE — velocity_data_valid=false
  velocity_rms_overall=undefined | passing through | machine=pump01"
  warnings at a steady ~30-second cadence, unbroken, for over an hour —
  67 minutes of continuous vibration-unavailable condition on the real device.

  LAST occurrence: 15 Sep 04:34:58 UTC = 11:34:58 Bangkok
  — exactly 22 seconds before the reported CRITICAL at 11:35:20 Bangkok.

15 Sep 04:34:58 UTC onward, through the current time of this investigation
(2026-09-15T05:04:29Z, i.e. ~30 minutes later): ZERO further log lines of
any kind from Node-RED (checked with no filter, not just the VIB_UNAVAILABLE
pattern). The hour-long, every-30-second warning stream did not merely
pause — it stopped entirely and has not resumed.

Mosquitto broker log, 04:30:00–04:40:00 UTC: zero connect/disconnect events
for "pump01" — the device held one continuous MQTT session through the
entire transition. This was not a device reboot or reconnect.

Live API poll performed just now (2026-09-15T05:04:29Z, 30 minutes after
the event): motor_state=RUNNING, velocity_data_valid=true,
vibration_status=OK, alarm_level=NORMAL, current_valid=true, online=true,
last_seen=2026-09-15T05:04:29Z (fresh) — the device has remained healthy
and continuously reporting ever since.

Corroborating anomaly: device_health.current_read_errors was 18 at the time
of this check, sharply up from a baseline of 1 observed in earlier audits
this engagement — consistent with a real, if now-resolved, acquisition/
current-sensor issue coinciding with the vibration outage.
```

**The two manual test-inject buttons — audited separately, as requested:**

```
7ec3e726495f761b "🔴 Test CRITICAL (PUMP01)"   — repeat="", once=false, crontab=""
05f64b8eeea910fa "✅ Test NORMAL / Recovery (PUMP01)" — repeat="", once=false, crontab=""
```
Both remain pure manual-click inject nodes (confirmed again in this pass), wired directly into the production pipeline (`885a21b2d935bf2d`) — this remains an accurate, standing finding about the flow's design (Section 6.A of the answer below). **However, their payloads omit the `velocity_data_valid` key entirely**, which — per Health Logic's own source — fires a distinctly different, specifically-worded warning: `"[Health] ⚠ LEGACY_FALLBACK — velocity_data_valid absent..."`. This exact string was searched for across the entire retained Node-RED log history: **zero occurrences from the Health Logic node, ever, in this container's lifetime** (a different, unrelated `LEGACY_FALLBACK` message exists in a completely different function — `🔬 Maintenance Analytics v2`, about a legacy `p.peak` bearing-evidence fallback — from `31 Aug`, over two weeks before this incident and before this engagement began; it is unrelated to vibration availability or this event). **This is direct, conclusive evidence that the test-inject buttons were never fired during this incident, or at any other point in this container's recorded history.**

---

## Evidence Table

| Timestamp (UTC / Bangkok) | Input fields (observed/inferred) | Authoritative verdict | Recovery condition | LINE output | Dashboard output (contract.py logic) | Consistent? |
|---|---|---|---|---|---|---|
| 03:27–04:34:58 / 10:27–11:34:58 | `velocity_data_valid=false` (explicit, present), real MQTT traffic, `machine=pump01` | Not observable (no alarm-level log); Dashboard would show `vibration_available=false` | n/a — ongoing outage | Not observable directly; if `alarm_level` was CRITICAL during this window (plausible, given current_read_errors), an 'alert' push may have occurred | `alarm_level_live = (vibration_status=="OK") and (motor_code==2)` → **false** throughout — Dashboard would correctly withhold treating `alarm_level` as a live verdict | **No** — LINE has no equivalent guard to withhold/qualify the alert |
| ~04:35:20 / 11:35:20 | `alarm_level` transitions to CRITICAL (firmware-side, non-vibration-threshold source); vibration state at/near this exact moment not directly captured | Unified State Engine: `state='CRITICAL'`, `alertType='alert'` | n/a | 🚨 CRITICAL alert text (content not independently verified) | Same `alarm_live` gate — likely still false if vibration hadn't yet stabilized | Not verifiable at message level (see limit, Section 5) |
| ~04:36:13 / 11:36:13 | `alarm_level` transitions to NORMAL; vibration likely mid-recovery (last VIB_UNAVAILABLE 22s before the CRITICAL entry; full stabilization confirmed only ~30 min later via live poll) | Unified State Engine: `state='NORMAL'`, `alertType='recovery'` | none (no debounce) | "✅ กลับสู่ NORMAL... Velocity RMS: N/A (FIFO-DSP unavailable)" | `alarm_live` would only be true once `vibration_status=="OK"` — not necessarily true yet at this exact second | **No** — LINE announced recovery without the Dashboard's live-verdict qualification |
| 05:04:29 / 12:04:29 (this investigation) | `velocity_data_valid=true`, `vibration_status=OK`, `alarm_level=NORMAL`, fresh `last_seen` | Stable, healthy | n/a | (no new message expected — Rule 1 dedup) | `alarm_live=true` | Consistent — both systems now agree, post-recovery |

---

## Direct Answers

**A. Was 11:35→11:36 an automatic recovery?**
Yes — supported by direct evidence: a genuine, uninterrupted, real hour-long vibration-unavailable condition (logged every ~30s from 03:27 to 04:34:58 UTC) ends within 22 seconds of the reported CRITICAL timestamp, the device never disconnected/reconnected from MQTT during this window (ruling out a reboot or reconnect-triggered event), and the device has remained healthy and continuously reporting for the 30 minutes since (confirmed by a fresh live poll). No manual-injection fingerprint (the `LEGACY_FALLBACK` warning specific to the test payload shape) appears anywhere in this incident.

**B. What exact condition caused recovery?**
The Unified State Engine's only input is `payload.alarm_level`; it transitioned CRITICAL→NORMAL because that is the value firmware published. What caused firmware to change that value cannot be fully confirmed without raw historical payload access (not pursued, per this engagement's credential-avoidance practice), but the timing strongly correlates with the resolution of a real acquisition/current-sensor degradation (see the `current_read_errors` spike and the vibration-outage log stream).

**C. Was vibration actually valid at recovery?**
Not confirmed to be valid at the exact 11:36:13 instant — the log evidence only proves vibration was unavailable 22 seconds before the CRITICAL entry, and confirmed-valid roughly 30 minutes later. The specific state at the recovery message's own timestamp sits inside that gap and cannot be pinned down further with the evidence available.

**D. Why did LINE show N/A if it did?**
Because the `[Notify-Fix-1]` guard in the LINE Message Builder correctly and honestly reports "N/A (FIFO-DSP unavailable)" whenever `velocity_data_valid !== true` in the specific message it processed — this is working as designed for *that field*. The defect is architectural, not a bug in this specific line of code: nothing anywhere in the pipeline stops an alarm-level *transition* (especially a reassuring "recovered to NORMAL") from being announced in the same breath as an admission that the sensor data behind it is unavailable.

**E. Does LINE use the same verdict as Dashboard?**
No — confirmed again in this trace. The Dashboard's `contract.py` computes `alarm_level_live = (vibration_status=="OK") and (motor_code==2)` specifically to avoid presenting a held/stale/unavailable-backed `alarm_level` as a live verdict. The LINE path has no analogous check anywhere.

**F. Are the manual test buttons a separate defect?**
Yes, on both counts requested: (i) **they remain a real, standing defect** — wired directly into the production pipeline, capable of dispatching genuine LINE pushes through real credentials using stale/legacy payload shapes, independent of this incident; and (ii) **they are confirmed unrelated to the actual 11:35→11:36 event** — the specific log fingerprint their payload shape would produce (`LEGACY_FALLBACK` from Health Logic) never appears anywhere in this incident's timeframe or, in fact, ever in this container's retained log history.

---
*Read-only forensic trace. No file modified, no message sent, no button invoked, nothing restarted or reconfigured. Not committed to git per instructions.*
