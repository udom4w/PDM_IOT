# PHASE-2 `/vibration/event` DATA-PATH DESIGN REVIEW

**Date:** 2026-09-03
**Type:** Read-only design audit — no code, no deployment, no commit
**Phase-1 status:** FROZEN — see `docs/releases/PHASE1_FREEZE_RECORD_2026-09-03.md`

**Frozen references (unchanged by this review):**

| | |
|---|---|
| Firmware commit | `d63e2339f701e37632ef17ac5606631a09a6cafd` |
| Firmware binary SHA256 | `3c635c5fcdaaae830bda8596b858a7c3540527fd1c28aac08d4c17b315325396` |
| Firmware source SHA256 | `d7484b696b2a24ee77dc3125aee8ae2da17ab636061283d00605ff2021427d2e` |

**Governing principle for everything below:**
*Edge event data is evidence, not a second alarm engine.*
`alarm_level` remains the only machine-condition verdict.

---

## 1. CURRENT DATA PATH

```
ESP32 firmware
  ├── /vibration        ──▶ Node-RED ──▶ InfluxDB ──▶ API ──▶ Dashboard   [LIVE]
  ├── /trend            ──▶ Node-RED ──▶ InfluxDB ──▶ API ──▶ Dashboard   [LIVE]
  ├── /device-health    ──▶ Node-RED ──▶ InfluxDB ──▶ API ──▶ Dashboard   [LIVE]
  ├── /sensor           ──▶ (no subscriber)                                [DARK]
  ├── /decision         ──▶ (no subscriber)                                [DARK]
  └── /vibration/event  ──▶ (no subscriber)                                [DARK]
```

### Verified consumers (2026-09-03, read-only)

| Question | Answer | Evidence |
|---|---|---|
| Any MQTT consumer of `/vibration/event`? | **No** | live `nodered/flows.json` declares exactly three `mqtt in` nodes |
| Node-RED consumers | `factory/+/machine/+/vibration`, `.../trend`, `.../device-health` | same file |
| Does Influx receive event data? | **No** | `schema.measurementFieldKeys` over 30 d shows no `acceleration_rms_*`, `velocity_1x_*`, `velocity_2x_*`, `crc_error_count`, `sample_rate_hz`, `sample_count`, `duration_ms` |
| Does the API receive event data? | **No** | `api/contract.py` reads only the `vibration`, `vibration_trend` and `device_health` measurements; `contract.py:129` records the fact in a comment |

Broker delivery is not the constraint — mosquitto accepts the publishes on
listener 8883 under mTLS (`require_certificate true`). **The path simply has no
subscriber.** Nothing downstream has ever seen a `/vibration/event` message.

### Cadence — the decisive architectural fact

| Stream | Interval | Messages/day/machine | Source |
|---|---|---|---|
| `/vibration` | **30 s** | 2,880 | empirically confirmed (Node-RED log timestamps 30 s apart; Influx write cadence) |
| `/vibration/event` | **2 s per capture, 2 events per capture** | **≈ 86,400** | `#define FIFO_PERIODIC_INTERVAL_MS 2000UL` (`.ino:2107`, ADR-0006 capture cadence) |

**`/vibration/event` carries roughly 30× the message rate of the canonical
topic, and ~60× the message count once both event types are counted.** With
`accel_rms` at up to 1,024 B and `fifo_capture` at up to 700 B, raw ingestion is
on the order of **70–75 MB/day/machine of JSON**, against a bucket
(`iot_vibration`) whose retention is 720 h / 30 days.

> ⚠️ **Caveat, stated plainly:** the 2 s figure is read from the firmware source
> constant, **not observed on the wire**. It could not be measured during this
> audit: subscribing would require a client certificate, and reusing the device's
> client id `pump01` would trigger a broker *session takeover* and knock the
> production device offline. **Measuring the real event rate and payload size
> with a distinct client id is the mandatory first step of any Phase-2 work** —
> every volume number in this document is a source-derived estimate until then.

---

## 2. EVENT INVENTORY

`/vibration/event` = `factory/{plant}/machine/{machine}/vibration/event`
(`g_mqttTopicEvent`, `.ino:2725`, `.ino:10272`).

The brief anticipated two producers. **There are four.**

### 2.1 `event: "fifo_capture"` — capture metadata

| | |
|---|---|
| Producer | `handleFifoCaptureCompletion()` — `.ino:5911`, enqueue at `.ino:5962` |
| Trigger | every completed FIFO capture, all trigger sources |
| Frequency | ~every 2 s (periodic `SCHEDULED`), plus event-driven captures |
| Buffer | `char evBuf[700]` |
| Routing | outbound queue → `MQTT_OUTBOUND_TOPIC_EVENT` |

Fields: `plant_id`, `machine_id`, `event`, `capture_id`, `tag`, `request_id`
*(REMOTE_ON_DEMAND only)*, `trigger`, `status`, `error`, `sample_count`,
`sr_index`, `sr_hz`, `t_request_ms`, `t_complete_ms`, `duration_ms`, `temp_c`,
`motor_state`, `rpm`, `poll_count`, `progress_frame_count`, `crc_error_count`,
`retry_count`, `last_progress_fill`, `desync_bytes_discarded`.

### 2.2 `event: "accel_rms"` — the measurement payload

| | |
|---|---|
| Producer | `processPendingAccelSnapshot()` — `.ino:9463`, enqueue at `.ino:9595` |
| Trigger | one per completed capture, queue-driven (`queueAccelSnapshot`) |
| Frequency | tracks capture cadence, ~every 2 s |
| Buffer | `char aBuf[1024]` |

Fields: `plant_id`, `machine_id`, `event`, `capture_id`,
`vibration_data_valid`, `acceleration_rms_x/y/z`, `acceleration_rms_overall`,
`velocity_data_valid`, `velocity_rms_x/y/z`, `velocity_rms_overall`,
`sample_rate_hz`, `sample_count`, `dominant_frequency_x/y/z_hz` *(omit-when-invalid)*,
`velocity_1x_x/y/z_mm_s`, `velocity_2x_x/y/z_mm_s` *(omit-when-invalid)*,
`vibration_source`.

**Validity contract is strict and already correct:** an axis the DSP could not
resolve has its key **absent**, never `0`/`null`/`NaN`. Any consumer must treat
absence as "not measured".

### 2.3 `event: "fifo_capture_rejected"` — trigger rejection

| | |
|---|---|
| Producer | `publishMqttRejectionEvent()` — `.ino:5833`, enqueue at `.ino:5852` |
| Trigger | **only** a rejected `REMOTE_ON_DEMAND` trigger |
| Frequency | exception-only; zero in normal operation |
| Buffer | `char rejBuf[300]` |

Fields: `plant_id`, `machine_id`, `event`, `request_id`, `trigger`, `error`.

### 2.4 `event: "maintenance_reset"` — operator audit

| | |
|---|---|
| Producer | `taskNetwork()` maintenance-queue drain — `.ino:7721` |
| Trigger | operator maintenance reset only |
| Frequency | rare, human-initiated |
| Routing | **direct** `mqttClient.publish(g_mqttTopicEvent, …)` — bypasses the outbound queue |

Fields: `plant_id`, `machine_id`, `event`, `timestamp`, `state`.

> Any subscriber must branch on the `event` key. It is **not** a homogeneous
> stream, and 2.4 does not pass through the same queue as 2.1–2.3.

### Shared capture provenance

`accel_rms` and `fifo_capture` for the same capture share `capture_id`, and the
`accel_rms` figures are derived from **the same canonical FIFO/DSP capture** that
produces the `/vibration` velocity numbers (`VibVelocity_ComputeRms` →
`g_velCarrier` → `velocity_rms_*`). There is **no second measurement engine** —
`/vibration` is a 30 s decimation of what `/vibration/event` emits every 2 s.

---

## 3. DUPLICATION ANALYSIS

Legend — **A** already on `/vibration` · **B** useful only at event level ·
**C** diagnostic-only · **D** genuinely new evidence

| Field | Class | Note |
|---|---|---|
| `velocity_rms_overall`, `_x`, `_y`, `_z` | **A** | identical values, 15× finer time resolution (2 s vs 30 s) |
| `dominant_frequency_x/y/z_hz` | **A** | published on `/vibration` as `freq_x/y/z` from the same spectrum |
| `velocity_data_valid` | **A** | same flag |
| `vibration_source` | **A** | constant `"fifo_dsp"` |
| `motor_state`, `rpm`, `temp_c` | **A** | already on `/vibration` (`temp_c` → `temp`) |
| `acceleration_rms_x/y/z/overall` | **D** | **acceleration domain exists nowhere downstream.** Only `crest_factor` (a ratio built from it) survives to `/vibration` |
| `velocity_1x_x/y/z_mm_s` | **D** | order-component amplitude — no equivalent anywhere |
| `velocity_2x_x/y/z_mm_s` | **D** | as above |
| `sample_rate_hz`, `sample_count` | **B** | per-capture provenance; meaningless without the capture |
| `capture_id` | **B** | the join key between 2.1 and 2.2 |
| `trigger`, `tag`, `request_id` | **B** | why this capture happened |
| `status`, `error` | **C→D** | capture verdict; see §4E — the one diagnostic field with real customer value |
| `crc_error_count`, `retry_count`, `desync_bytes_discarded` | **C** | RS485 link health |
| `duration_ms`, `t_request_ms`, `t_complete_ms` | **C** | capture timing |
| `poll_count`, `progress_frame_count`, `last_progress_fill`, `sr_index` | **C** | driver internals |
| `vibration_data_valid` | **B** | acceleration-domain validity, distinct from `velocity_data_valid` |

**Conclusion.** Only **two families are genuinely new**: acceleration RMS, and
the 1X/2X order components. Everything else is either already stored (A), only
meaningful inside a capture (B), or engineering diagnostics (C).

**Do not duplicate class-A fields into a new measurement.** Re-storing
`velocity_rms_*` at 2 s would multiply storage for data the product already has
at the resolution its 30 s alarm cadence can act on. The only defensible reason
to store class-A at event resolution is a specific, stated need for sub-30 s
transient capture — which no current requirement asks for.

---

## 4. EVIDENCE-VALUE MATRIX

### A. 1X / 2X velocity amplitude

| Question | Answer |
|---|---|
| Tells an SME | **Nothing safely, today.** An amplitude with no baseline is a number without meaning |
| Tells an engineer | Order-component energy at shaft speed and twice shaft speed, per axis — the raw material for classical diagnosis |
| Cannot tell | That a fault exists. A 1X amplitude is not unbalance; a 2X amplitude is not misalignment. Those inferences need a baseline, a machine class, mounting context and usually phase data the sensor does not produce |
| **Usable without RPM?** | **No — and this is decisive.** `vib_velocity.h:51` defines `f1 = rpmAtCapture / 60`. The bands are *placed* using the RPM latched at capture. The values are RPM-derived at the edge, so consuming them silently imports an RPM dependency Phase-1 deliberately refused. When RPM is low the band falls below the high-pass floor and the valid flag goes false |
| Needs historical baseline? | **Yes**, unavoidably |
| New API endpoint? | Yes |
| New Influx fields? | Yes (6 values + 6 validity flags) |

> The firmware is deliberately disciplined here: *"Phase 3E reports AMPLITUDES
> ONLY. There is deliberately no 2x/1x ratio, no threshold"* (`vib_velocity.h:70`).
> Phase 2 must not undo that restraint downstream.

### B. Dominant frequency X/Y/Z

| Question | Answer |
|---|---|
| Tells an SME | Nothing new — already visible on the Dashboard as `freq_x/y/z` |
| Tells an engineer | At 2 s resolution: whether the dominant peak *moves*, which 30 s sampling can alias |
| Cannot tell | That a frequency is abnormal. **A high frequency is not evidence of anything** without a per-machine reference |
| Usable without RPM? | Yes — it is a measured spectral peak |
| Needs baseline? | **Yes**, for any "changed" claim |
| New endpoint / fields? | Only if event-resolution history is genuinely required |

### C. Acceleration RMS

| Question | Answer |
|---|---|
| Tells an SME | Nothing directly; the unit is not one an SME acts on |
| Tells an engineer | High-frequency energy content that velocity RMS integrates away — the domain where early bearing and gear-mesh energy first appears |
| Cannot tell | **That a bearing is damaged.** Also: a high crest factor does **not** prove bearing damage. CF is a waveform-shape ratio, and it *falls* again as damage progresses and the signal becomes broadband |
| Usable without RPM? | Yes |
| Needs baseline? | **Yes** — no universal threshold exists for this machine class |
| New endpoint / fields? | Yes — 4 fields; genuinely absent downstream today |

### D. Velocity RMS axis breakdown

| Question | Answer |
|---|---|
| Tells an SME | Already delivered — Phase-1 ships the ranked axis chip under WARNING/CRITICAL |
| Tells an engineer | 2 s resolution instead of 30 s |
| Cannot tell | Which fault type. **Dominant axis does not identify a fault** |
| Usable without RPM? | Yes |
| Needs baseline? | No for display, yes for "changed" |
| New endpoint / fields? | **No — duplicate.** Class A |

### E. FIFO capture quality (`status`, `error`)

| Question | Answer |
|---|---|
| Tells an SME | **Why** the assessment is degraded, in place of today's generic *"คุณภาพการวัดลดลง"*. This is the one event field with immediate, actionable customer value |
| Tells an engineer | The exact driver verdict per capture |
| Cannot tell | Anything about machine condition. A capture failure is a *measurement chain* fact |
| Usable without RPM? | Yes |
| Needs baseline? | **No** — the firmware already emits a verdict, not a raw number |
| New endpoint / fields? | Small: an enum + a timestamp on `device_health` |

### F. Sample rate / sample count

| Question | Answer |
|---|---|
| Tells an SME | Nothing |
| Tells an engineer | Whether a capture is trustworthy — the known failure shape is `sample_count == 1024` with `sr_hz == 0` (`.ino:4846`, `.ino:5998`) |
| Cannot tell | Anything about the machine |
| Baseline / endpoint | No baseline; diagnostic surface only |

### G. Capture duration / retry / CRC / desync

| Question | Answer |
|---|---|
| Tells an SME | Nothing, and must never be shown as a machine fact |
| Tells an engineer | RS485 link quality, driver backoff behaviour, cable/EMI problems |
| Cannot tell | Machine condition. **Never convert these into a machine-condition alarm** |
| Baseline | Trend-useful, threshold-free |
| New fields | Yes, if a diagnostic view is built |

### H. `capture_id` / timestamp relationship

| Question | Answer |
|---|---|
| Tells an SME | Nothing directly |
| Tells an engineer | The join key. `capture_id` links `accel_rms` to `fifo_capture`; `t_request_ms`/`t_complete_ms` are **device-uptime milliseconds**, not wall clock |
| Cannot tell | Absolute time — correlation with Influx timestamps requires the broker/ingest receive time, or `time_synced` + `sync_age_s` from the existing path |
| Note | **Design constraint:** any event storage must carry `capture_id` as a tag-or-field and must not assume `t_*_ms` is comparable across reboots |

---

## 5. SME USE CASES

Judged by: *would a non-engineer act differently because of this?*

| # | Use case | Verdict |
|---|---|---|
| 1 | *"Why does it say measurement quality is degraded?"* | ✅ **Real.** Today the Dashboard says it, and cannot say why. `fifo_capture.error` answers it |
| 2 | *"Is the sensor cable or the machine at fault?"* | ✅ **Real.** CRC/retry/desync distinguish a link problem from a machine problem — expressed as measurement wording, never as an alarm |
| 3 | *"Is the vibration getting worse than it used to be?"* | ⚠️ **Real but blocked** — needs a per-machine baseline, which does not exist. Also achievable from the *existing* `/vibration` history without ingesting events at all |
| 4 | *"Which axis is worst?"* | ✅ Already shipped in Phase 1 |
| 5 | *"Do I have unbalance / misalignment / a bad bearing?"* | ❌ **Out of scope.** Requires proof this product does not have |
| 6 | *"What is my 1X amplitude?"* | ❌ Not an SME question. Engineer-facing only |

**Only use cases 1 and 2 are unblocked, and both are data-quality, not
machine-condition.** That single observation determines the recommendation.

---

## 6. PROPOSED PHASE-2 ARCHITECTURE

| Option | Shape | Assessment |
|---|---|---|
| **A** | event → backend only → diagnostic view | Lowest risk, zero customer value on its own. Good for engineers, does not move the product |
| **B** | event → backend → evidence API → Dashboard Details | Delivers use cases 1 & 2. Needs no baseline and no new measurement if scoped tightly |
| **C** | event → Influx → historical evidence/trend → Dashboard | The only route to use case 3, but pays the full ~60× volume cost *before* anyone knows which fields matter |

### Recommendation: **B, deliberately scoped to exception-only capture quality**

Not "B because it shows more". B **only** for §4E, and specifically:

```
firmware /vibration/event
   └── Node-RED subscriber, branch on event type
         ├── event=="fifo_capture" && error!=NONE  ──▶ Influx device_health (enrich)
         ├── event=="fifo_capture_rejected"        ──▶ Influx device_health (enrich)
         └── everything else                        ──▶ DROP (counted, not stored)
                                                          │
                                    API device_health block gains 2-3 fields
                                                          │
                                    Dashboard: existing Recommendation
                                    becomes SPECIFIC. No new Attention item.
```

**Why this is the minimum that creates real customer value:**

- It answers the only two unblocked SME questions.
- **Volume collapses to near zero.** A healthy machine emits no non-OK captures,
  so the write path is idle in normal operation — none of the 60× cost is paid.
- It needs **no baseline**, because the firmware already emits a verdict.
- It adds **no new measurement** — it enriches the existing `device_health` one.
- It changes **no Attention item**; only the wording of an existing
  Recommendation becomes specific.
- It cannot create a second alarm source: capture quality can only ever alter
  *measurement* wording, which Phase-1 already separates from machine condition.

**Option C is explicitly deferred, not rejected.** It is the correct eventual
home for baseline work — but building it now means paying full ingestion cost to
store fields (1X/2X, acceleration RMS) whose usefulness is unproven and whose
interpretation needs a baseline that must be accumulated *first*. If baseline
accumulation is approved as a separate engineering objective, do it as
Option A — decimated, diagnostic-only, no customer surface — and revisit.

---

## 7. ATTENTION-BUDGET RULES (PHASE 2 — HARD)

Carried forward from the Phase-1 freeze, unchanged:

| Rule | Value |
|---|---|
| Primary Attention | **exactly 1**, or none |
| Evidence items | **maximum 2**, subordinate |
| Evidence may change `alarm_level` | **never** |
| Evidence styling | no severity colour, no alert glyph, visually subordinate |
| NORMAL | stays visually quiet |

**How event evidence appears without creating spam:**

1. **Event data adds no new Attention line.** Capture quality already has a
   primary line at rank 4 of the ladder. Phase 2 makes that line's *Recommendation*
   specific — it does not add a line.
2. **Evidence chips remain gated on WARNING/CRITICAL + online.** Event-derived
   evidence inherits the same gate; it never appears while NORMAL.
3. **Persistence is mandatory.** At 2 s cadence there are ~15 captures per 30 s
   publish. A single non-OK capture must **never** put anything on screen. Require
   an N-of-M rule (e.g. ≥3 non-OK in the last 15) before any wording changes.
4. **Aggregate, never enumerate.** One line about the measurement chain — never
   one line per counter, and never a raw CRC/retry/desync number in the customer UI.
5. **Diagnostic fields stay off the Dashboard entirely.** Class C is for an
   engineering view, behind a separate route.

---

## 8. IMPLEMENTATION ROADMAP

### Ranked candidates

| Rank | Candidate | Rationale |
|---|---|---|
| **P0** | Measure the real `/vibration/event` rate and payload size with a distinct MQTT client id | Every volume decision below depends on it, and it is currently an estimate |
| **P0** | Exception-only `fifo_capture` ingestion → specific measurement-quality Recommendation | Only unblocked customer value; near-zero volume; no baseline needed |
| **P1** | Engineering diagnostic view for capture quality (CRC / retry / desync / duration / sr / sample_count) | Real engineering value, no customer surface, no alarm risk |
| **P2** | Decimated `accel_rms` ingestion for **baseline accumulation only** (acceleration RMS first) | Prerequisite for any future "changed vs. its own history" wording. Diagnostic-only until ≥30 d of baseline exists |
| **Later** | 1X / 2X as customer-visible evidence | Needs the P2 baseline **and** a resolution of the RPM dependency (§4A). Not before both |
| **Not ranked** | Fault-type naming (unbalance / misalignment / looseness / bearing) | No use case that this product can honestly serve. See §11 |

### By change class

| Change class | Items |
|---|---|
| **No firmware change needed** | Everything in this roadmap. The firmware already publishes all of it |
| **Backend-only** (Node-RED + Influx + API) | P0 measurement, P0 exception ingestion, P1 diagnostic view, P2 decimated baseline store |
| **Frontend** | One Recommendation wording branch for P0. Nothing else |
| **Requires firmware change** | Nothing in Phase 2. *(Only a change to the 2 s cadence or to the omit-when-invalid contract would, and neither is proposed)* |
| **Should NOT be implemented yet** | Full-rate event ingestion; any 1X/2X or CF threshold; any ratio-derived indicator; any fault-type label; any new Attention line |

---

## 9. RISKS / FALSE-POSITIVE RISKS

| # | Risk | Mitigation |
|---|---|---|
| 1 | **Volume shock.** ~60× message count, ~70–75 MB/day/machine raw, into a 30 d bucket. Multiplied by every machine added | Exception-only ingestion at P0; measure before deciding (P0); decimate at P2 |
| 2 | **Flapping from a single bad capture.** 15 captures per publish window; one transient CRC error is normal | N-of-M persistence rule, §7.3 |
| 3 | **RPM dependency smuggled in.** 1X/2X are RPM-derived at the edge; consuming them imports the dependency Phase-1 refused | Do not surface 1X/2X to customers (P-Later). Record the dependency wherever they are stored |
| 4 | **"High frequency = abnormal."** The most likely wrong inference | Frequency stays observation-only. No "unusual" claim without a per-machine baseline |
| 5 | **"High CF = bearing damage."** CF is a waveform-shape ratio and can *fall* as damage becomes broadband | No CF threshold, ever. CF stays display-only |
| 6 | **Dominant axis read as a fault type** | Ranked magnitudes only; never a letter alone; never mapped to a fault |
| 7 | **Diagnostic counters leaking into customer UI** as machine faults | Class C is engineering-only, behind a separate route; customer wording says *measurement*, never *machine* |
| 8 | **Second alarm engine by accident** — a "capture quality" rule that starts gating machine state | Capture quality may only alter measurement wording. It can never write, upgrade or clear `alarm_level` |
| 9 | **Duplicate storage drift** — class-A fields stored twice at two cadences, then disagreeing | Do not store class A. If ever required, store it as a separate measurement with explicit provenance, never merged into `vibration` |
| 10 | **Event stream is not homogeneous** — four event types, one of which bypasses the outbound queue | Subscriber must branch on `event` and default to DROP for unknown types |

---

## 10. RECOMMENDED FIRST PHASE-2 INCREMENT

**Increment 1 — "Say why the measurement is degraded."**

1. **Measure first.** Subscribe read-only with a distinct client id (never
   `pump01` — that causes a broker session takeover and knocks the device
   offline). Record actual event rate, payload sizes and the real distribution
   of `status`/`error` over ≥24 h. **Do not build anything until this is known.**
2. **Ingest exceptions only.** Node-RED branches on `event`; forwards
   `fifo_capture` with `error != NONE` and `fifo_capture_rejected`; drops the
   rest with a counter.
3. **Enrich `device_health`** with a small, bounded set: latest capture error
   enum, its timestamp, and a rolling non-OK count. No new measurement.
4. **API:** extend the existing `device_health` block. No new endpoint.
5. **Dashboard:** the existing degraded-measurement Recommendation becomes
   specific. **No new Attention line, no new evidence chip, budget unchanged.**

**Acceptance criteria for the increment:**

- `alarm_level` still has exactly one producer, and the frontend still never
  assigns it.
- Attention budget unchanged: 1 primary + ≤2 evidence.
- NORMAL still renders zero Attention lines.
- No customer-facing string contains a CRC, retry or desync number.
- No new threshold constant introduced anywhere.
- Steady-state ingest volume increase is measurably near zero on a healthy
  machine.

---

## 11. EXPLICIT NON-GOALS

Phase 2 will **not**:

1. Create a second machine-condition alarm source. `alarm_level` remains sole.
2. Let CF, frequency, 1X, 2X, acceleration RMS or dominant axis generate
   `WARNING` or `CRITICAL` — independently or in combination.
3. Name a fault type — unbalance, misalignment, looseness, bearing fault, gear
   fault, resonance, cavitation — unless separately proven and separately
   approved.
4. Introduce a universal threshold for crest factor (no 3.5 / 5 / 7), for any
   frequency, or for any order amplitude.
5. Add a new Attention line or a third evidence chip.
6. Ingest `/vibration/event` at full rate.
7. Duplicate class-A fields into new storage.
8. Show CRC / retry / desync / sample-rate / capture-duration figures to SME
   users.
9. Treat a measurement-chain problem as a machine problem.
10. Resurrect any removed legacy diagnosis field
    (`bearing_alert`, `kurtosis_*`, `fault_type`, `fault_confidence`,
    `maintenance_risk_score`, `predictive_status_code`, `cf_*`, `freq_ratio_*`,
    `freq_drift_*`, `vibration_source_legacy`).
11. Modify Phase-1 firmware, its binary, or the frozen baseline.

---

## APPENDIX — Source references

| Item | Location |
|---|---|
| Event topic construction | `.ino:2725`, `.ino:10272` |
| Outbound topic enum | `.ino:1936` |
| Capture cadence constant | `.ino:2107` — `FIFO_PERIODIC_INTERVAL_MS 2000UL` |
| `fifo_capture` producer | `.ino:5911` → enqueue `.ino:5962` |
| `fifo_capture_rejected` producer | `.ino:5833` → enqueue `.ino:5852` |
| `accel_rms` producer | `.ino:9463` → enqueue `.ino:9595` |
| `maintenance_reset` producer | `.ino:7721` → direct publish |
| 1X/2X band definition (RPM-derived) | `vib_velocity.h:51` — `f1 = rpmAtCapture / 60` |
| Firmware restraint note (amplitudes only) | `vib_velocity.h:70` |
| API's own record that `/event` is not ingested | `api/contract.py:129` |
| Node-RED subscriptions | `/opt/iot-stack/nodered/flows.json` — three `mqtt in` nodes |

---

*Read-only design review. No firmware modified, built or flashed; no API,
frontend, Node-RED or InfluxDB change; nothing deployed, restarted or committed.
Phase-1 baseline untouched.*
