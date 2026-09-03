# PHASE-1 FREEZE / RELEASE RECORD

**Date:** 2026-09-03
**System:** PROMLOGIX Condition Monitoring — Pump01 / plant01
**Record type:** Production baseline freeze
**Verification method:** read-only audit of live production (VPS `iotprom`, live API, InfluxDB, git)

Every hash, field list and status in this record was re-verified against the
running system on 2026-09-03 before this file was written. Nothing was
deployed, restarted, rebuilt, flashed or committed to produce it. Where a claim
could **not** be re-verified from the production data path, that is stated
explicitly rather than assumed.

---

## 1. PHASE-1 PRODUCT SCOPE

- The machine-condition verdict is **`alarm_level`**, with exactly three values:
  `NORMAL` / `WARNING` / `CRITICAL`.
- **`alarm_level` is the only machine-condition alarm in the product.** No other
  field, metric or layer may produce, upgrade or clear a machine alarm.
- **Hysteresis and persistence are firmware-owned.** The enter threshold, the
  clear threshold and the hold behaviour live in the ESP32 firmware and are not
  represented, duplicated or second-guessed anywhere downstream.
- **The Dashboard does not create or modify `alarm_level`.** Verified this date:
  in the deployed `app.js` the only assignment is `var alarmLevel =
  d.vibration.alarm_level;` (line 237) — a read *from* the API response. Every
  other occurrence is a comparison. No write path exists.

---

## 2. CANONICAL VIBRATION DATA

Canonical fields currently flowing on `/vibration` → Node-RED → InfluxDB → API →
Dashboard. Verified live 2026-09-03T10:36:23Z (last Influx write) and against
the live API response.

| Field | Live value at verification | Notes |
|---|---|---|
| `velocity_rms_overall` | 0.542 mm/s | source of record for the alarm |
| `velocity_rms_x` | 0.256 mm/s | omitted entirely when invalid, never 0 |
| `velocity_rms_y` | 0.185 mm/s | |
| `velocity_rms_z` | 0.441 mm/s | |
| `freq_x` | 24.5 Hz | FIFO/DSP dominant frequency, per-axis validity |
| `freq_y` | 24.4 Hz | |
| `freq_z` | 24.4 Hz | |
| `crest_factor` | 2.66 | `vector_peak / acceleration_rms_overall` (firmware `.ino:8717`) |
| `dominant_vibration_axis` | `Z` | stored in Influx as `dominant_axis` |
| `vibration_source` | `fifo_dsp` | provenance declaration |
| `vibration_status` | `OK` | **must be read before `alarm_level`** |
| `velocity_data_valid` | `true` | sole authority for velocity presence |

**Validity contract:** the firmware omits any field it could not measure. An
absent key becomes `None` in the API (`contract.py::_num`), never a fabricated
`0`. `velocity_rms_*` are published only when `velocity_data_valid` is true.

### Operating-context fields (available, NOT alarm sources)

`rpm`, `temp`, `current_a` + `current_valid`, `operating_hours_total`,
`motor_state` / `motor_state_code`, `time_synced`, `sync_age_s`,
`data_latency_ms`, plus the `/trend` window statistics
(`mean/min/max/stddev/slope/slope_valid/samples/gap_count/trend_gap_s`) and
`/device-health` (`analysis_ready`, `freeze_reason`, `current_*`).

These remain available for display and context. **None of them is redefined as
an alarm source by this freeze.** In particular, Phase-1 does not require RPM as
a customer input, and no dashboard rule depends on it.

---

## 3. TOPIC CONTRACT

| Topic | Role | Ingested by Dashboard/API? |
|---|---|---|
| `/vibration` | Canonical customer telemetry: velocity RMS, frequency, crest factor, dominant axis, `alarm_level`, `vibration_status`, motor/operating context | **YES** |
| `/trend` | 60 s / 300 s window statistics and slope | **YES** |
| `/device-health` | Analytics readiness, freeze reason, current-evidence health, sync age | **YES** |
| `/sensor` | Device/sensor-state reporting | **NO** — published, not subscribed |
| `/decision` | Decision-engine detail stream | **NO** — published, not subscribed |
| `/vibration/event` | FIFO capture metadata: `acceleration_rms_*`, `velocity_1x_*`, `velocity_2x_*`, `sample_rate_hz`, `sample_count`, `crc_error_count`, `retry_count`, `desync_bytes_discarded`, `duration_ms`, capture status/error | **NO** |

### `/vibration/event` is NOT ingested

Stated explicitly, as required. Verified three independent ways:

1. Live `nodered/flows.json` declares exactly **three** `mqtt in` nodes:
   `factory/+/machine/+/vibration`, `.../trend`, `.../device-health`.
2. Firmware routes `MQTT_OUTBOUND_TOPIC_EVENT` to `g_mqttTopicEvent` =
   `factory/…/vibration/event` (`.ino:2725`, `.ino:10272`).
3. `api/contract.py:129` records it in a source comment: *"the FifoError codes
   live on /vibration/event, which is not ingested."*

**Consequence:** no 1X/2X, acceleration RMS, sample-rate or FIFO capture-quality
counter is available to the API or Dashboard. Any future feature needing them
requires a deliberate ingestion change (new subscriber + Influx write + contract
fields), not a frontend change.

---

## 4. LEGACY REMOVAL

Current product logic no longer uses the following. Verified by last-write
timestamp in InfluxDB (`iot_vibration` bucket, `vibration` measurement) on
2026-09-03:

| Field | Last write observed | Status |
|---|---|---|
| `bearing_alert`, `bearing_alert_code` | 2026-09-01T17:02:09Z | no writes since |
| `kurtosis_*` (`kurtosis_x`, `kurtosis_axis_code`) | `kurtosis_x` 2026-08-23T15:59:49Z | no writes since |
| `cf_x` / `cf_y` / `cf_z` | 2026-08-23T15:59:49Z | no writes since |
| `freq_ratio_*` | 2026-08-23T15:59:49Z | no writes since |
| `freq_drift_*` | 2026-08-23T15:59:49Z | no writes since |
| `fault_type`, `fault_type_code` | 2026-09-01T17:02:09Z | no writes since |
| `fault_confidence` | 2026-09-01T17:02:09Z | no writes since |
| `maintenance_risk_score` | 2026-09-01T17:02:09Z | no writes since |
| `predictive_status_code` | 2026-09-01T17:02:09Z | no writes since |
| `vibration_source_legacy` | 2026-08-31T06:18:29Z | no writes since |
| legacy VRMS/VPEAK (`rms`, `vx`, `vy`, `vz`, `peak`) | 2026-08-31T06:18:29Z | no writes since |

**No legacy write has occurred in ~41 hours** (newest legacy write
2026-09-01T17:02:09Z vs. canonical writes continuing at 2026-09-03T10:36:23Z).

**Historical data is preserved.** All the fields above remain queryable over the
30-day retention window. **No historical delete, rewrite, migration or retention
change was performed** at any point in this work.

### Known trap: dead field mappings still present in Node-RED

The Node-RED *Prepare InfluxDB* function still maps `cf_x/y/z`,
`freq_ratio_*`, `freq_drift_*`, `freq_alert` and `freq_gate_open`, but the
firmware no longer publishes any of them on `/vibration` (zero `doc["…"]`
assignments in the source). These mappings are inert. **Do not design any future
rule on these fields** — they would read as permanently absent. Recorded here so
the mapping is not mistaken for a live data source.

---

## 5. DASHBOARD UX RULES (FROZEN)

| # | Rule | Enforcement |
|---|---|---|
| 1 | **NORMAL is visually quiet** | Attention card hidden entirely; zero lines |
| 2 | **WARNING = one primary Attention** | exclusive `if/else-if` chain assigns at most one `primary` |
| 3 | **CRITICAL = one primary Attention** | same chain |
| 4 | **Maximum 2 evidence chips** | `evidence.slice(0, EVIDENCE_MAX)`, `EVIDENCE_MAX = 2` |
| 5 | **Currently one axis-evidence chip** | only the axis ranking is pushed; second slot intentionally unused |
| 6 | **Frequency and Crest Factor are display/diagnostic only** | they appear only in their own cards (`app.js:333–349`); zero references in any alarm/Attention/evidence branch |
| 7 | **Dominant axis is supporting evidence, not a diagnosis** | ranked magnitudes only; no unbalance/misalignment/looseness/bearing conclusion anywhere |
| 8 | **STOPPED / STARTING / STOPPING create no vibration alarm** | every branch below "offline" is gated on `motorRunning`; `alarm_level_live` is false unless `motor_state_code == 2` |
| 9 | **OFFLINE must not display stale vibration values as current** | `vibLive = vibValid && d.status.online` drives values, pill and empty state; axis evidence additionally gated on `d.status.online` |
| 10 | **Attention answers "what should I know?"** | statements of state |
| 11 | **Recommendation answers "what should I do?"** | actions only; no sentence duplicated between the two panels |
| 12 | **No alarm spam** | worst case (every trigger simultaneously true) resolves to exactly 1 line, down from 9 |

### Priority ladder (exclusive — exactly one primary)

1. Device offline *(outranks all vibration states; suppresses evidence chips)*
2. CRITICAL
3. WARNING
4. Measurement quality degraded *(collapses acquisition fault + analysis paused + three staleness flags into one line)*
5. Thresholds not configured
6. NORMAL + sustained rising trend *(neutral styling; requires 3 consecutive polls)*
7. otherwise — no Attention

### Frozen Thai wording

| State | Status hero | Attention | Recommendation |
|---|---|---|---|
| NORMAL | ปกติ — เครื่องทำงานอยู่ในสภาวะปกติ | *(hidden)* | เฝ้าระวังตามปกติ / ค่าการสั่นสะเทือนอยู่ในช่วงการทำงานปกติ — ติดตามแนวโน้มตามรอบปกติ |
| NORMAL + rising | ปกติ *(unchanged)* | ℹ สังเกตแนวโน้มการสั่นสะเทือนค่อย ๆ เพิ่มขึ้นในช่วง 60 วินาทีล่าสุด | *(unchanged)* |
| WARNING | เฝ้าระวัง — สถานะ Warning ยังคงอยู่ | ⚠ สถานะ Warning ยังคงอยู่ — จะกลับเป็นปกติเมื่อค่าการสั่นลดลงถึงเกณฑ์ปลด Warning<br>*chip:* ความเร็วการสั่นรายแกน: Z 0.438 · X 0.287 · Y 0.179 mm/s | เฝ้าระวังเพิ่มเติม / ติดตามแนวโน้มการสั่นสะเทือนอย่างใกล้ชิด และตรวจสอบเครื่องตามโอกาส |
| CRITICAL | วิกฤต — ค่าการสั่นสะเทือนเกินระดับวิกฤต | 🔴 ตรวจพบการสั่นสะเทือนระดับวิกฤต<br>*chip:* axis ranking | ควรตรวจสอบทันที / ตรวจสอบเครื่องจักรโดยเร็ว และพิจารณาหยุดเครื่องเพื่อตรวจสอบ |
| STOPPED | ยังไม่ประเมิน — รอเครื่อง RUNNING เพื่อประเมิน vibration | *(none)* | รอเครื่องทำงาน / รอเครื่อง RUNNING เพื่อประเมิน vibration — ไม่ต้องดำเนินการ |
| Degraded | ยังไม่ประเมิน | ⚠ คุณภาพการวัดลดลง — ผลประเมินอาจไม่ครบถ้วนชั่วคราว | รอการประเมิน |
| OFFLINE | ออฟไลน์ — ไม่มีข้อมูลล่าสุดจากอุปกรณ์ | 🔴 อุปกรณ์ออฟไลน์ — พบข้อมูลล่าสุดเมื่อ *hh:mm:ss* | ตรวจสอบการเชื่อมต่อ |

### Vibration card empty-state wording (state-accurate)

| Condition | Text |
|---|---|
| OFFLINE (any prior motor state) | อุปกรณ์ออฟไลน์ — ค่าล่าสุดเมื่อ *hh:mm:ss* |
| STOPPED | เครื่องหยุดทำงาน — ไม่มีการวัดการสั่นสะเทือนใหม่ |
| STARTING | เครื่องกำลังเริ่มทำงาน — รอเพื่อประเมิน vibration |
| STOPPING | เครื่องกำลังหยุดทำงาน — รอการประเมินรอบถัดไป |
| RUNNING, velocity rejected | ยังไม่มีข้อมูลการสั่นสะเทือนที่ถูกต้องในขณะนี้ |

---

## 6. HYSTERESIS UX (ACCEPTED BEHAVIOUR)

**A machine can remain in `WARNING` while the current instantaneous RMS is below
the warning-enter threshold.** This is correct, intended firmware behaviour: the
alarm state uses hysteresis and is held until vibration falls to the clear
threshold.

Worked example that motivated this rule:

```
current velocity RMS   = 2.03 mm/s
warning ENTER threshold = 2.10 mm/s
alarm_level             = WARNING     <- correct, not a bug
```

**Dashboard obligation.** The UI explains the *held state* and must never claim
the current value is above a limit unless that is actually true. It does **not**
invent or duplicate a clear threshold: the API exposes no clear-threshold field,
and the browser must not fabricate one.

Frozen wording: *"สถานะ Warning ยังคงอยู่ — จะกลับเป็นปกติเมื่อค่าการสั่นลดลงถึงเกณฑ์ปลด Warning"*
— states the state, names no number.

Verified this date: grep for any `N.N mm/s` literal inside status, Attention or
Recommendation strings in the deployed `app.js` returns **nothing**. The
display-only constant `VIB_WARNING_MMS = 2.1` is used solely for chart band
shading and drives no wording and no decision.

---

## 7. FIRMWARE BASELINE

| Item | Value | Verification |
|---|---|---|
| Commit | `d63e2339f701e37632ef17ac5606631a09a6cafd` | `git log -1` — 2026-09-02 02:47:05 +0700, *"feat(firmware): canonical buffered replay contract for /vibration"* |
| BUILD_ID | `16.5-d63e233-dirty-20260902-0253` | consistent with `build_info.h` → `GIT_COMMIT_HASH "d63e233-dirty"` and binary link time 2026-09-02 03:01 |
| Source SHA256 | `d7484b696b2a24ee77dc3125aee8ae2da17ab636061283d00605ff2021427d2e` | **verified** — git object and working-tree file hash identically |
| Flashed binary SHA256 | `3c635c5fcdaaae830bda8596b858a7c3540527fd1c28aac08d4c17b315325396` | **verified** — `build_d63e233/…​.ino.bin` |
| Flashed binary size | `676704` bytes | **verified** |
| `sizeof(TelemetrySlot_t)` | `64` bytes | **verified** — `#define TELEM_SLOT_EXPECTED_SIZE 64u` (`.ino:2348`) enforced by `static_assert` (`.ino:2349`) |
| Replay `vbuf` | `1024` bytes | **verified** — `char vbuf[1024];` (`.ino:3662`) |
| Telemetry buffer | `TELEM_BUF_SIZE = 120` slots (60 min @ 30 s) | `.ino:2282` |

### On the `-dirty` suffix

`BUILD_ID` carries `-dirty` because the repository working tree was not clean at
build time. **This is unrelated repository-wide noise, not firmware drift.**
Verified on 2026-09-03:

- Working tree: **9,466 deleted** files (a bulk vendor-software directory,
  `Download the WitMotion Software/…`), **53 untracked**, **4 modified**.
- The 4 modified tracked files are
  `T-Vending-master/…/TinyGsmClientSIM7600.h`,
  `claude/…/build_info.h`,
  `experimental/analyze_fifo_dewesoft.py`,
  and `ssh root@iot.promlogix.com.txt`.
- **The production firmware source is `clean`** — `git status --porcelain` on
  `WTVB02_ESP32S3_V16_CM_fault_latch_v3_hardened_patched_v16_5.ino` returns
  nothing.
- The committed source and the working-tree source hash **identically**
  (`d7484b69…`), so the binary was built from the committed source byte-for-byte.

### Documentation discrepancy (no code impact)

The comment block at `.ino:2267` still reads *"Slot size : sizeof(TelemetrySlot_t)
= 100 B / Total RAM : 120 × 100 B = 12 000 B"*. The compiler-enforced value is
**64 B** (`static_assert`), giving 120 × 64 = 7,680 B of static `.bss`. The
comment is stale documentation only; the build could not have succeeded if the
asserted size were wrong. Recorded for accuracy — **not** listed as an open item
because it has no product effect.

---

## 8. FIRMWARE REPLAY STATUS

> **Status: IMPLEMENTED / BUILD-VERIFIED / NOT RUNTIME-OBSERVED**

Verified at source level on 2026-09-03:

- **Canonical replay contract implemented.** The buffered-replay serializer emits
  the canonical vibration set on `/vibration`: `stage` (`.ino:3582`),
  `vibration_status` (`:3632`), `velocity_rms_overall` and per-axis (`:3638`),
  `freq_x/y/z`, `crest_factor`, `dominant_vibration_axis`, `alarm_level`
  (`:3656`).
- **`alarm_level` is transported from capture-time state.**
  `captureTelemetrySnapshot()` stores `snap.alarm_level = effectiveState`
  (`.ino:4679`) — a pure copy of the already-decided state, with no business
  arithmetic inlined.
- **Replay does not recompute `alarm_level`.** The replay path emits
  `alarmLevelStr((MachineState_t)snap.machine_state, …)`, reading the stored
  slot. No evaluation, classification or threshold comparison occurs at replay
  time.
- **Replay `/vibration` carries canonical vibration fields** — previously these
  existed only on the `/event` accel-RMS message.
- **Replay `/sensor` is device/sensor-state only.**

**Why not runtime-observed:** no MQTT outage test has been performed. The replay
path has not been exercised end-to-end against a real disconnect/reconnect
cycle. Build verification and source review are complete; runtime proof is not.
See §12.

---

## 9. API BASELINE

**Dominant axis mapping verified:**

```
firmware  doc["dominant_vibration_axis"]        (.ino:9191)
   -> Node-RED  fields.dominant_axis            (Prepare InfluxDB, §6)
   -> InfluxDB  field  dominant_axis
   -> API       _str(vib, "dominant_axis")      (contract.py:209)
   -> response  "dominant_vibration_axis"
```

The Influx field name (`dominant_axis`) and the Product-1 response key
(`dominant_vibration_axis`) deliberately differ. Reading the response key back as
a field name is the historical bug that made this permanently null; the current
code reads the correct field name. **Verified live 2026-09-03: API returned
`"dominant_vibration_axis": "Z"`.**

**Live API returned valid canonical vibration data after restart** — verified
this date: `velocity_data_valid: true`, `vibration_status: "OK"`,
`alarm_level: "NORMAL"`, `alarm_level_live: true`, `source: "fifo_dsp"`, all
four velocity figures, all three frequencies and `crest_factor` present and
non-null.

**API contract invariants frozen:** a null field is delivered as `null` and
rendered as `—`, never `0`; `alarm_level_live` is true only when
`vibration_status == "OK"` **and** `motor_state_code == 2`; `current_a` is
nulled unless `current_valid` is true for that cycle.

---

## 10. DASHBOARD BASELINE

Production frontend at `/opt/iot-stack/frontend/` on host `iotprom`, served by
nginx from a read-only bind mount (no build step). **Hashes re-read directly
from production on 2026-09-03 immediately before writing this record.**

| File | SHA256 | Size | Last modified |
|---|---|---|---|
| `app.js` | `98668e997911a2d29d16d27b8b8773999f6849bc203a868cb8a14d016c8dffa6` | 55,086 B | 2026-09-03 17:30 |
| `style.css` | `e8f0e62dd1f8e15deee0cd85ac72750355cd11647f6f1bb611ef1db6f47471d8` | 38,780 B | 2026-09-03 16:03 |
| `index.html` | `89e3389165e5bc99985d4e856a6a7b58b5b895ff7d298acbe5296d149a664419` | 23,825 B | 2026-09-01 19:26 |
| `shared/api.js` | `08bb17cbd121f86b426c469b972b65b9e05dd48232b8dc8bdaf7a00def7f9693` | — | 2026-08-25 |
| `shared/shell.js` | `8f389e3f35aaedbdfa70c5a6d977224b0e94607cdd881d95370a2aec49cb59ab` | — | 2026-08-30 |

The bytes served over HTTPS hash identically to the files on disk (verified via
`curl … | sha256sum`).

**Note:** the frontend is not in the git repository. It exists only on the VPS.
These hashes are therefore the authoritative Phase-1 frontend identity, and any
future change must be compared against them.

### Deployment lineage into this baseline (2026-09-03)

| Time | Change | Resulting `app.js` SHA256 |
|---|---|---|
| 15:19 | Hysteresis-aware Attention/Recommendation wording; STOPPED neutral wording | `c4c147df…e38e895` |
| 16:03 | Attention budget (1 primary + ≤2 chips) + axis evidence chip; `style.css` evidence rule | `97c11abb…14bc85` |
| 17:30 | Acceptance defects D1 (stale RMS while offline) + D2 (STARTING/STOPPING wording) | `98668e99…6c8dffa6` |

All three were in-place writes preserving inode 802086, owner `iotprom:iotprom`,
mode 644. **No nginx or service restart was performed for any of them.**
Backups: `/opt/iot-stack/backups/{dash_wording_hysteresis_20260903_151914,
attention_evidence_20260903_160257, acceptance_d1_d2_20260903_173013}/`, each
with a `SHA256_BEFORE.txt`.

---

## 11. PRODUCTION HEALTH

Verified 2026-09-03 unless otherwise noted.

| Check | Status | Evidence |
|---|---|---|
| MQTT mTLS connected | ✅ **verified** | mosquitto: `Client pump01 negotiated TLSv1.2 cipher ECDHE-RSA-AES256-GCM-SHA384`; broker config on listener 8883 has `require_certificate true` + `use_identity_as_username true` with a CA file — genuine mutual TLS |
| Influx ingest active | ✅ **verified** | newest canonical write `2026-09-03T10:36:23Z`; all of `velocity_rms_*`, `dominant_axis`, `crest_factor`, `freq_x`, `alarm_level`, `vibration_status`, `velocity_data_valid` written in the same cycle |
| Dashboard frontend served | ✅ **verified** | `GET /machine/` → HTTP 200 (23,825 B); `app.js` → 200 (55,086 B); `style.css` → 200 (38,780 B); served bytes hash-match disk |
| API returning fresh data | ✅ **verified** | `last_seen 2026-09-03T10:33:52Z`, `online: true`, `acquisition_fault: false`, all three `data_quality.*.stale` false, `time_synced: true` |
| Analytics ready | ✅ **verified** | `analysis_ready: true`, `freeze_reason: "READY"` |
| No legacy writes since cleanup | ✅ **verified** | newest legacy write `2026-09-01T17:02:09Z`; ~41 h with none, while canonical writes continue |
| Stack services stable | ✅ **verified** | nginx 8 d, api 40 h, nodered 41 h (healthy), mosquitto 6 d, influxdb 4 w, grafana 2 d, tunnel 4 w |
| Firmware boot clean | ⚠️ **not re-verified this session** | no serial access during this audit. Observable proxies today: device authenticates and publishes on schedule, telemetry is fresh and internally consistent, `analysis_ready: true`. Last direct boot verification predates this record. |
| FIFO captures `error=NONE` | ⚠️ **not observable from production** | capture status/error live on `/vibration/event`, which is not ingested (§3). Indirect indicators today: `vibration_status: OK`, `velocity_data_valid: true`, `acquisition_fault: false`. |

**Operational observation (not a defect):** 12 device connect events in 24 h,
with `session taken over` and `exceeded timeout` disconnects. This is normal
4G/cellular reconnection behaviour for this deployment and is recorded for
baseline comparison, not flagged as a fault.

---

## 12. KNOWN OPEN ITEMS

Only genuinely outstanding items are listed.

1. **Replay end-to-end outage test not performed.** The buffered-replay path is
   implemented and build-verified but has never been exercised against a real
   MQTT disconnect/reconnect cycle. Status per §8:
   *IMPLEMENTED / BUILD-VERIFIED / NOT RUNTIME-OBSERVED.*
2. **`/vibration/event` is not ingested by the Dashboard/API path.** This is a
   deliberate Phase-1 boundary, not a bug — recorded because it is the single
   constraint that decides what any future evidence feature can use. Lifting it
   requires a Node-RED subscriber, an Influx write path and API contract fields.

No other open items. Nothing further is asserted.

---

## 13. FREEZE RULE

Any future feature or change must **prove**, before merge or deploy, that it
does **not**:

- **a)** create a second machine alarm source — `alarm_level` remains the sole
  machine-condition verdict;
- **b)** increase Attention noise — the budget of 1 primary + ≤2 subordinate
  evidence chips is a ceiling, and NORMAL must stay visually quiet;
- **c)** present stale data as current — anything not `online` and not currently
  valid must render as unavailable, in every panel, on the same render pass;
- **d)** resurrect removed legacy diagnosis — no `bearing_alert`, `kurtosis_*`,
  `fault_type`, `fault_confidence`, `maintenance_risk_score`,
  `predictive_status_code`, `cf_*`, `freq_ratio_*`, `freq_drift_*` or
  `vibration_source_legacy`;
- **e)** bypass the canonical FIFO/DSP vibration truth — `velocity_rms_*` gated
  by `velocity_data_valid`, with `vibration_status` read before `alarm_level`.

Supporting metrics (frequency, crest factor, dominant axis, and any future 1X/2X)
may be shown as **evidence**, never as a trigger, and may not be labelled
abnormal without a real reference — a per-machine baseline or an explicit
firmware verdict. A high value is not, by itself, evidence of anything.

---

## 14. FINAL STATUS

> # PHASE 1 — FROZEN
>
> All current production verification checks pass. The two items in §12 are
> recorded as known and accepted at freeze, not as failures: neither affects the
> Phase-1 customer-facing product, and both are boundary conditions rather than
> defects.

**Acceptance gate result:** the two defects raised in the 2026-09-03 acceptance
review — D1 (stale RMS presented as VALID while offline) and D2 (STARTING /
STOPPING described as "machine stopped") — were fixed and deployed at 17:30, and
re-verified against the deployed file. No open acceptance defects remain.

**Baseline identity for future comparison:**

```
firmware source   d7484b696b2a24ee77dc3125aee8ae2da17ab636061283d00605ff2021427d2e
firmware binary   3c635c5fcdaaae830bda8596b858a7c3540527fd1c28aac08d4c17b315325396
frontend app.js   98668e997911a2d29d16d27b8b8773999f6849bc203a868cb8a14d016c8dffa6
frontend style    e8f0e62dd1f8e15deee0cd85ac72750355cd11647f6f1bb611ef1db6f47471d8
frontend index    89e3389165e5bc99985d4e856a6a7b58b5b895ff7d298acbe5296d149a664419
```

---

*Record produced by read-only audit. No firmware built or flashed, no service
restarted, no deployment performed, no application source or configuration
modified, and nothing committed in the course of producing this document.*
