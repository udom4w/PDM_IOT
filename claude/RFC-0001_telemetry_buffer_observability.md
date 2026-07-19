# RFC-0001: Telemetry Buffer Observability

Status: Proposed
Author: (engineering team)
Date: 2026-07-05
Applies to: WTVB02_ESP32S3 firmware, /sensor telemetry ring buffer
Related: PATCH_NOTES_v16.5.md (Crest Factor export policy — separate, unrelated change)

---

## 1. Context

The firmware already has a working telemetry ring buffer for the `/sensor`
topic:

- 120 slots, one snapshot per publish interval (30s/10s/5s depending on
  machine state)
- Overwrite-oldest behavior when full
- 24-hour TTL drop on replay
- Replay in oldest-to-newest order after MQTT reconnect

This design was reviewed and rated adequate for an ESP32 edge device.
Two gaps were identified during review:

1. Buffer overflow (data loss) is tracked as a passive counter only.
   No event, alarm, or dashboard signal is raised when it happens.
2. There is no visibility into how full the buffer currently is, or
   how much time remains before the device starts losing data if the
   4G connection stays down.

This RFC proposes closing those two observability gaps only.

---

## 2. Non-Goals

To keep scope tight and risk low, this RFC explicitly does **not**
propose:

- Expanding the `/trend` outbound queue (currently 6 slots, drop-newest)
  to match the `/sensor` ring buffer (120 slots, overwrite-oldest).
  `/trend` is intentionally a near-real-time analytics stream, not a
  historical record — see Section 5.
- Making the firmware responsible for historical trend reconstruction.
  If historical trend is needed later, it should be computed downstream
  (Node-RED / InfluxDB) from replayed `/sensor` data, which already
  carries `buffered_at` timestamps sufficient for that purpose.
- Persisting the ring buffer across reboot (still RAM-only).
- Any change to the Crest Factor export policy (v16.5) — unrelated.

---

## 3. Problem Statement

Today, if the 4G/MQTT link is down long enough to overflow the 120-slot
buffer:

- The firmware silently overwrites the oldest unsent readings.
- `g_telemBufOverflowCount` increments, but this is only visible in the
  30-second Serial status report on the device itself.
- No one downstream (Grafana, on-call engineer, field technician) is
  notified that data loss occurred, or when.
- A field technician standing next to the machine during an outage has
  no way to know "how much longer until we start losing data."

For a PdM system, silent data loss is a meaningful gap — overflow events
should be visible and actionable, not just counted.

---

## 4. Proposed Scope

### 4.1 Overflow → Event → Alarm

When `g_telemBufOverflowCount` increments (i.e. a slot is actually
overwritten, not just when the buffer becomes full):

- Publish a one-shot event to MQTT (topic TBD, likely `.../vibration/event`
  which already exists per the topic list in boot log) containing at
  minimum: `event=telem_buffer_overflow`, `overflow_count`, `buffered_since`
  (timestamp of the first slot that started backing up).
- Rate-limit this event (e.g. once per N overflow occurrences, or once
  per minute) to avoid flooding the broker once overflow starts happening
  repeatedly.
- Downstream: Grafana annotation + alert rule on this event.

### 4.2 Buffer Utilization Metric

Add to the existing `/status` payload (published every cycle regardless
of MQTT buffer state, since `/status` itself is not currently buffered):

- `telem_buf_pct`: `g_telemBufCount / TELEM_BUF_SIZE * 100`
- `telem_buf_eta_min`: estimated minutes remaining before overflow begins,
  computed from current fill level and current `publishInterval`
  (i.e. `(TELEM_BUF_SIZE - g_telemBufCount) * publishInterval / 60000`)

This gives a field technician a direct, human-readable answer to
"if 4G doesn't come back, how many minutes until we start losing data,"
directly on the OLED or in the MQTT payload.

### 4.3 Documentation Correction (housekeeping, not a feature)

While reviewing this area, a stale comment was found: the trend outbound
queue (`queueMqttOutboundTrend`) is commented as "dormant, no producer/
consumer wired yet," but tracing the code shows both the producer
(`taskAnalytics` via `enqueueMqttOutbound()`) and the consumer
(`taskNetwork` drain loop) are actually wired and active. This comment
should be corrected to avoid misleading future maintainers — same class
of issue as the CF gating bug (comment says one thing, code does
another). Recommend fixing as a documentation-only change, independent
of this RFC's functional scope.

---

## 5. Why /trend Stays As-Is

`/trend` fields such as `trend_gap_s`, `slope_ready_1s/10s/60s`, and the
EMA reseed marker reflect the firmware's own runtime state (e.g. warm-up
suppression after a resume from STOPPED). These cannot be perfectly
reconstructed downstream without replicating that internal state.

However, the core trend signal — slope and moving averages of RMS over
time — is fully reconstructable downstream from the replayed `/sensor`
series, since each replayed slot carries `buffered_at`. This means
historical trend, if ever needed, can be built in Node-RED/InfluxDB
without spending ESP32 RAM on a larger on-device queue.

Open question for review: is near-real-time-only `/trend` (with a gap
in the dashboard during long outages) acceptable for this deployment's
use case, or does it need to be revisited once real outage patterns from
the field are observed?

---

## 6. Open Questions (for review before implementation)

1. What MQTT topic and payload shape should the overflow event use —
   reuse `.../vibration/event`, or a new dedicated topic?
2. What rate limit is appropriate for the overflow event (per-overflow
   vs. time-windowed)?
3. Should `telem_buf_pct` / `telem_buf_eta_min` also appear on the OLED
   display for on-site technicians without a dashboard handy?
4. Is the stale "dormant" comment fix (Section 4.3) worth its own tiny
   commit, or bundled into whichever release implements this RFC?

---

## 7. Next Steps

- Circulate this RFC for review.
- Once scope is confirmed, convert into a design spec / patch notes
  document (following the same format as PATCH_NOTES_v16.5.md) for
  implementation, tracked as its own release — kept separate from
  v16.5 and from any future `/trend` design work.
