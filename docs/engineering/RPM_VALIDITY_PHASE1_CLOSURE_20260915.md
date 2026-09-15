# RPM Validity — Phase 1 Engineering Closure Record
Date: 2026-09-15
Machine under test: pump01 (plant01), production/demo unit

> **Editorial note on this record's accuracy:** this document reports only what was directly observed or directly verified during this engagement. Where a claim rests on the physical operator's report rather than independent tooling, it is labeled **[OPERATOR OBSERVED]** rather than presented as independently verified. The physical RPM-invalid hardware test described in Section 5 was executed on production pump01 on 2026-09-15; the invalid-state and recovery API transitions were independently captured and hashed by tooling (SSH + `curl` against the read-only Machine Detail API), not merely reported. Full raw evidence, including SHA256 of every captured response, is retained at `docs/engineering/evidence/rpm_validity_test_20260915/`.

## 1. Objective
Prevent numeric `RPM=0` from being interpreted as a valid stopped-machine measurement while `motor_state` is RUNNING.

## 2. Implemented contract
```
motor_state == STOPPED                                -> rpm_valid = true
motor_state != STOPPED, fresh RPM pulse evidence       -> rpm_valid = true
motor_state != STOPPED, no fresh RPM pulse evidence    -> rpm_valid = false
rpm_valid does not read or write motor_state (one-way, read-only dependency)
```

## 3. Production implementation (firmware)
- Candidate branch `candidate/rpm-valid-f6f7a48`, based on commit `f6f7a488e01cab1bef0ce0a347a59506ad2fff0a` (the Replay Timestamp Fix commit). The Replay-Contract change in that commit is unrelated and untouched by this feature.
- **The RPM-valid firmware change itself has not been committed to git.** It exists as verified, hash-matched working-tree changes in the candidate worktree and as the exact binary flashed to hardware — not yet merged into version history. This is a real gap in the record, not an oversight in this document.
- Binary flashed to pump01, app-only, at flash offset `0x10000`:
  ```
  SHA256: 5063f71e7ccdbb77e8e1dfa10eac86d118d8f2e69323cf57b5f9d4e7e3254e39
  ```
  Confirmed via an independent post-flash readback of the same 676,912-byte region — SHA256 matched exactly (not merely esptool's own internal check).
- `FW_VERSION`: `16.5` (source-level `#define`, single source of truth) — confirmed by inspecting the exact source compiled into the verified binary.
- `GIT_COMMIT_HASH`: `f6f7a48-dirty` (baked into `build_info.h` at compile time) — confirmed the same way. **Not independently confirmed from the device's live serial boot banner**: three separate capture attempts (two line-based, one raw-byte) all showed that specific printf burst silently dropped, most likely due to unsynchronized concurrent `Serial` writes from a FreeRTOS task at boot — a pre-existing characteristic of this banner code, unrelated to this feature, and not fixed or investigated further in this engagement.

## 4. Downstream implementation
- **Node-RED**: `Prepare InfluxDB (Edge-Only)` node (`cc6ae17d3288db72`) patched to convert `p.rpm_valid` to `fields.rpm_valid` (1/0), guarded on `!== undefined` so absent legacy data writes no field. Deployed via container restart (performed by the user after Claude Code's own harness blocked an automated restart attempt). Pre/post-deploy diffs confirmed only this one node changed, no wiring/config/credential-node changes, zero DROP/exception/Influx-write-error activity since.
- **InfluxDB `rpm_valid` field**: not directly queried (no Influx credentials were used, by design, throughout this engagement). Its presence and 1/0 typing is **inferred with high confidence** from (a) the reviewed Node-RED code, and (b) the API now correctly returning a live boolean `rpm_valid` value sourced from that same Influx row (see below) — this closes the loop that an earlier turn in this engagement had explicitly logged as "NOT CONFIRMED." Not upgraded to directly-proven, since no raw Influx row was read.
- **API (`/opt/iot-stack/api/contract.py`)**: patched to add `rpm_valid` to the response and to null `rpm` when `rpm_valid is False` (never inferred from `rpm==0`; absent `rpm_valid` leaves legacy `rpm` behavior untouched). Installed to production (SHA256 `0d5e11a64a55a0fee6edc4d1f85984c515c4c43de9b4af039ad4c15d7950fb05`), verified against a byte-exact diff, and the `iot-stack-api-1` container was subsequently restarted (by the user), activating it. Live confirmation: repeated polling of `GET /api/machine/plant01/pump01` shows `operating_condition.rpm_valid` transitioning correctly (see Section 5). **The `rpm=null` branch of this logic has now been exercised against real production data** during the physical test in Section 5 (`phase2_poll_20260915T031346Z.json`, `phase2_poll_20260915T031402Z.json`) — previously verified only via the local unit test suite (6 new cases in `test_contract.py`, 28/28 tests passing), now also confirmed live.
- **Dashboard "—" display for invalid RPM**: **[OPERATOR OBSERVED]** — during the physical test in Section 5, the operator reported the dashboard rendered RPM as "—" while the API was returning `rpm=null` and the machine remained RUNNING. The frontend source was never located, reviewed, or modified in this engagement, so this records the observed behavior of the existing, unmodified frontend when handed a null value — not a new feature built or independently verified by tooling in this session. No screenshot file was provided.

## 5. Production test evidence

Full raw evidence retained at `docs/engineering/evidence/rpm_validity_test_20260915/` (6 API-response JSON files + `test_timeline.txt`, every file SHA256-hashed).

### Phase 0 — Baseline: PASS — [API OBSERVED]
Two independent confirmations: a 15-poll stability check earlier in this engagement (~4m42s, 2026-09-15T02:25:17Z–02:29:59Z, 15/15 consistent), and the immediate pre-test capture on 2026-09-15T03:10:24Z (`phase0_pretest.json`, SHA256 `95204fc72e26f694a8857a49b58a7c61cde6b660cfbd5c5e2c0542a26c71b478`):
```
motor_state = RUNNING (2)          online = true
current_valid = true (current_a ~0.70–0.74 A)
rpm_valid = true                   rpm ≈ 1482.7–1484.3
vibration_status = OK, velocity_data_valid = true
trend.status = VALID, slope_reseeded = false
acquisition_fault = false
```

### Phase 1 — Physical disconnect — [OPERATOR OBSERVED]
Operator reported disconnecting only the verified GPIO17 RPM/proximity signal conductor at **T0 = 10:13 Bangkok (~2026-09-15T03:13:00Z, minute precision)**. Not independently verifiable by tooling (no physical-layer access); no wiring log exists beyond this report.

### Phase 2 — Invalid state: PASS — [API OBSERVED]
Independently captured by SSH+`curl` against the live read-only API, not merely reported:
```
2026-09-15T03:13:46Z (~T0+46s) — phase2_poll_20260915T031346Z.json
  SHA256: e669290e23bdeb86d3b85dc58cb7178c9c5964997dd9c14158693e8ed7a85928
  motor_state=RUNNING current_valid=true rpm=null rpm_valid=false
  vibration_status=OK velocity_data_valid=true acquisition_fault=false

2026-09-15T03:14:02Z (~T0+62s) — phase2_poll_20260915T031402Z.json
  SHA256: 32fe745ffdec849466c5be0813c097478c4b32a8f92f60da3a2101d0364df8a4
  motor_state=RUNNING current_valid=true rpm=null rpm_valid=false
  vibration_status=OK velocity_data_valid=true acquisition_fault=false
  trend.status=VALID trend.gap_count=17 (unchanged — vibration/trend
  pipeline unaffected by the RPM-only fault, confirming the two
  subsystems are independent as designed)
```
Invalid state persisted across both polls, spanning the required 60-second hold.

### Phase 3 — Dashboard: PASS — [OPERATOR OBSERVED]
[OPERATOR OBSERVED] Dashboard state reported by operator via text message immediately after Phase 2: "Phase 3 — PASS. Dashboard แสดง RPM — ขณะเครื่อง RUNNING" (dashboard displayed RPM as "—" while the machine was RUNNING). No screenshot file was supplied to this session; this is a text report only, not independently verified by tooling. See Section 4's Downstream Implementation note for the caveat that the frontend itself was never reviewed.

### Phase 4 — Reconnect / Recovery: PASS
**[OPERATOR OBSERVED]** Reconnection of the same GPIO17 signal conductor to the same terminal at **T1 = 10:17:05 Bangkok (2026-09-15T03:17:05Z, second precision)**.

**[API OBSERVED]** Three independent, consecutive confirmations (exceeding the required minimum of two):
```
2026-09-15T03:17:32Z (~T1+27s) — phase4_poll_20260915T031732Z.json
  SHA256: 471bb4e110117713366a02a6e17dcb5f0dcbcca241c285887ff4100af4fcc3f0
  motor_state=RUNNING current_valid=true rpm=1484 rpm_valid=true
  vibration_status=OK acquisition_fault=false

2026-09-15T03:17:44Z (~T1+39s) — phase4_poll_20260915T031744Z.json
  SHA256: 505ebf599940b286cb0273f52434e82caf6c181e9025e686d52974ef96b78ade
  motor_state=RUNNING current_valid=true rpm=1484 rpm_valid=true
  vibration_status=OK acquisition_fault=false

2026-09-15T03:17:55Z (~T1+50s) — phase4_poll_20260915T031755Z.json
  SHA256: 4255b3b048e2da2d57c586d37fa9fdab9c117d9d495cc3c926dc3fcef62e1b51
  motor_state=RUNNING current_valid=true rpm=1484 rpm_valid=true
  vibration_status=OK acquisition_fault=false trend.status=VALID
```
`rpm=1484` falls within the required ~1450–1500 recovery band.

## 6. Acceptance result

| Criterion | Result |
|---|---|
| `rpm_valid` contract (STOPPED→true, running+fresh→true, running+stale→false) implemented in firmware | PASS (source-verified, hash-verified on flashed binary; running+stale→false path confirmed live in Phase 2) |
| `rpm_valid` does not affect `motor_state` | PASS (diff-verified: no write path added to any FSM variable; `motor_state` observed RUNNING throughout Phases 0–4 regardless of `rpm_valid`) |
| Node-RED converts `rpm_valid` to `fields.rpm_valid` (1/0), leaves `rpm` mapping untouched | PASS (structural diff verified; end-to-end transition now also confirmed via the API round-trip in Phases 2 and 4) |
| InfluxDB stores `rpm_valid` as 1/0 | NOT OBSERVED directly (no Influx query performed); STRONGLY SUPPORTED by code + the live invalid/recovery transition observed at the API layer, which reads directly from this Influx field |
| API exposes `rpm_valid` | PASS (live-confirmed, transitions both false→true and true→false observed) |
| API returns `rpm=null` when `rpm_valid=false` | **PASS — [API OBSERVED]** live in production, Phase 2 (`phase2_poll_20260915T031346Z.json`, `phase2_poll_20260915T031402Z.json`) |
| Legacy absent `rpm_valid` preserves existing `rpm` behavior | NOT OBSERVED in production (this firmware always sends `rpm_valid` now); unit-test-verified only |
| Dashboard shows "—" for invalid RPM | **PASS — [OPERATOR OBSERVED]**, Phase 3; not independently verified by tooling, no screenshot provided |
| Physical RUNNING+no-pulse hardware test performed | **PASS**, Phases 1–4, 2026-09-15 T0=03:13Z–T1+50s |

## 7. Known observations / non-blockers
- `trend.gap_count` increased (2→5) during the observation period in this engagement, traced to an independent ~60–90s vibration-acquisition interruption around 2026-09-15T02:18–02:19 UTC (three `VIB_UNAVAILABLE` warnings in Node-RED logs), already resolved by the time of the baseline check. This occurred well after any restart performed in this engagement and is not attributed to the RPM-valid feature.
- `current_read_errors=1` was observed in `device_health` while `current_valid` remained `true` throughout. A single, non-recurring count; not attributed to the RPM-valid feature without further evidence, and none was gathered.
- Neither observation is treated as a blocker for the RPM-valid work itself, but neither should be read as "explained" by anything in this closure record — they are logged, not resolved.

## 8. Explicit exclusions
- Temperature validity: **DEFERRED**, out of scope for this record.
- No changes were made to motor-state source logic (`MOTOR_SRC_CURRENT`/`buildMotorStateEvidence()` untouched).
- No Replay-Contract changes (`replayTelemBuf()`, `tsBufCapture`/`tsBufReplayed` untouched by this feature).
- No NET-RECOVERY-V2 changes.

## 9. Final status
```
RPM VALIDITY PHASE 1 = PASS / CLOSED
  - Firmware, Node-RED, and API changes: implemented, deployed, and verified.
  - Live hardware invalid-state test (Phases 0-4, 2026-09-15): PASS.
    Invalid state (rpm=null, rpm_valid=false) and recovery (rpm_valid=true,
    numeric rpm) both independently captured via the read-only Machine
    Detail API and hashed. See Section 5 and
    docs/engineering/evidence/rpm_validity_test_20260915/.
  - Dashboard "—" display: observed [OPERATOR OBSERVED] during the test;
    frontend source itself was not reviewed or modified in this engagement.

Residual, non-blocking notes carried forward from the implementation record:
  - The RPM-valid firmware change is flashed and verified but not yet
    committed to version control.
  - InfluxDB's rpm_valid field was not directly queried (no Influx
    credentials used); its storage as 1/0 remains inferred, not
    independently proven by a raw Influx read.
  - GIT_COMMIT_HASH was not independently confirmed from the device's live
    serial boot banner (see Section 3).

Temperature Validity = DEFERRED
```
