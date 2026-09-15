# Live UAT — Real WARNING Event, Post-Deployment
Recorded: 2026-09-15

Evidence classification, stated precisely and distinguished by source:

- **LINE screenshot: DIRECTLY VIEWED by this session** — an actual image was attached to this conversation mid-turn and inspected directly. This is not a text description relayed by the operator; it is genuine visual evidence this session examined itself.
- **Dashboard screenshot: OPERATOR-REPORTED (text description only)** — no Dashboard image file was attached to this conversation. The Dashboard figures below (RMS 2.294 mm/s, Current 0.90 A, RPM 2,382, "Attention: WARNING", "Recommendation: increased monitoring") are recorded as the operator's textual report, not independently viewed by this session as an image. This distinction is made deliberately, per this engagement's standing practice of not claiming to have seen evidence that was not actually provided as an artifact.

---

## What was directly viewed (LINE screenshot)

A LINE chat screenshot ("promlogix (4)" group) showing, in order:

1. **11:54 AM** — a prior `status` command reply: `CRITICAL: 0`, `WARNING: 0`, `Snoozed: 0`, `Acked: 0` (all-clear, predates the event).
2. **12:36 PM** (message's own internal timestamp: `15 ก.ย. 2569 12:36:49 น.`) — a WARNING alert:
   ```
   ⚠️ แจ้งเตือน WARNING!
   🏭 Plant: plant01
   ⚙️ Machine: pump01
   ⚙️ Motor State: RUNNING
   📊 Velocity RMS: 2.25 mm/s (Source: FIFO-DSP)
   ⏰ 15 ก.ย. 2569 12:36:49 น.

   📖 Commands:
     ack pump01
     snooze pump01 30
   ```
3. **12:37 PM** — the user typed `status`.
4. **12:37 PM** — bot reply: `CRITICAL: 0`, `WARNING: 1 — pump01`, `Snoozed: 0`, `Acked: 0`.

## Independent corroboration performed by this session

Before and after this image arrived, the read-only Machine Detail API was polled directly:
```
2026-09-15T05:46:34Z  motor_state=RUNNING  alarm_level=WARNING  velocity_rms_overall_mms=2.329
2026-09-15T05:47:46Z  motor_state=RUNNING  alarm_level=WARNING  velocity_rms_overall_mms=2.378
```
Both confirm a real, ongoing, live WARNING condition on `pump01`, `motor_state=RUNNING`, consistent with the screenshot's `12:36:49 น.` (05:36:49 UTC) alert — these two checks are ~10 and ~11 minutes after the alert fired, respectively. The RMS trend across all four data points (2.25 → 2.294 [operator-reported Dashboard, time unspecified] → 2.329 → 2.378 mm/s) is monotonically increasing over this window, consistent with a real, continuously-evolving physical vibration reading rather than a static or fabricated number — this supports, rather than contradicts, the numbers all being genuine live samples taken moments apart, exactly as the operator's own note anticipated ("may simply reflect different sampling timestamps").

## Assessment

**The directly-viewed LINE screenshot confirms the deployed patch's behavior in real production, for the first time with genuine image evidence (not the isolated test harness, not a text relay):**
- Clean `⚠️ แจ้งเตือน WARNING!` header used (not the "unconfirmed" qualified wording) — correct, since vibration was genuinely valid for this event (`alarm_verdict_live` was true).
- **`⚙️ Motor State: RUNNING` line is present.** The old `❤️ Health: ...` line does **not** appear anywhere in the screenshot. This is the first live-production confirmation that the "Health" → "Motor State" wording change (item 3 of the original proposal) is genuinely deployed and functioning, not merely proven in the offline harness.
- A real velocity number is shown (`2.25 mm/s`), never a fabricated one.

**Numeric RMS equality between the Dashboard's reported 2.294 mm/s and the LINE screenshot's 2.25 mm/s: NOT REQUIRED, and not treated as a defect.** No raw, timestamp-aligned data was captured to align these two readings to the same instant; the live API polling above shows this exact value naturally drifting by roughly this much within single-digit minutes during a real, ongoing WARNING event. Declaring this a defect would require timestamp-aligned raw evidence that does not exist and was not sought.

## Classification

```
LIVE UAT — WARNING — DASHBOARD/LINE VERDICT CONSISTENCY = OBSERVED PASS
```
- Dashboard screenshot = OPERATOR-SUPPLIED (text description only; not an image this session viewed)
- LINE screenshot = DIRECTLY VIEWED by this session (an actual image, not merely operator-supplied text)
- Verdict consistency = OBSERVED (both sources report WARNING for pump01, RUNNING motor state, at approximately the same real-world time)
- Numeric RMS equality = NOT REQUIRED (no timestamp-aligned raw evidence exists or was sought to demand exact equality)

## Effect on the deployment record

This event satisfies **Case B (WARNING + vibration valid)** of the production deployment's UAT plan (`LINE_VERDICT_PRODUCTION_DEPLOYMENT_20260915.md`), previously recorded as "NOT OBSERVED (live)" because no real WARNING had occurred during that session's monitoring window. That report's Case B row is updated to reference this event and this evidence file, rather than restating it in full.

---
*No production code or configuration was modified while recording this evidence. No test LINE message was sent. No manual test button was used. This event was entirely real, naturally-occurring production activity, observed and corroborated after the fact. No LINE credential or token value is reproduced anywhere in this document.*
