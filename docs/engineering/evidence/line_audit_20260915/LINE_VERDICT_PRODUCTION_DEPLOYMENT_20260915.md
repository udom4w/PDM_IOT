# LINE/Dashboard Verdict-Consistency Patch — Production Deployment Record
Deployment timestamp (restart command issued): **2026-09-15T05:27:15Z**

**Status: DEPLOYED. Live production `flows.json` now runs the approved patch. No rollback was required.**

No LINE credential or token value is reproduced anywhere in this document.

---

## Phase 1 — Pre-deploy safety

```
Live flows.json SHA256 (checked immediately before any action): f3fba6aaac72e03d2a675e0417ac29d4c4ebc35146890ae35d9472f1996d2665
Expected:                                                        f3fba6aaac72e03d2a675e0417ac29d4c4ebc35146890ae35d9472f1996d2665
Match: YES — proceeded.
```

**Rollback backup created:**
```
/opt/iot-stack/backups/nodered/flows_PRE_VERDICT_FIX_20260915T052613Z.json
SHA256: f3fba6aaac72e03d2a675e0417ac29d4c4ebc35146890ae35d9472f1996d2665
```
(Identical to the pre-deploy live file, confirmed.)

**Final live-vs-staged comparison, immediately before install:**
```
live count: 59   staged count: 59
ids only in live: []   ids only in staged: []
changed node ids: ['adf3dc5f003a516c', 'e982d76b3ebe0b06']
```
Only the two approved nodes differed. Proceeded to Phase 2.

## Phase 2 — Deploy

```
Staged source SHA256:  d10be3a0eb983bf84407b9d53e6c162249d0f58b407eaad4906cf46a2fd3a3d5
Installed file SHA256: d10be3a0eb983bf84407b9d53e6c162249d0f58b407eaad4906cf46a2fd3a3d5  (exact match)
Owner/group/mode preserved: iotadmin:iotadmin, 644 (unchanged from before)
```
Installed via `sudo install -o iotadmin -g iotadmin -m 644` (preserves ownership without a separate chmod/chown step).

**Deployment action:** no Node-RED admin-API credentials are available to this session, so the only available activation mechanism is a container restart — explicitly pre-authorized by this task ("Prefer a controlled Node-RED restart only if required by the deployment mechanism"). Executed:
```
docker restart iot-stack-nodered-1     (issued 2026-09-15T05:27:15Z)
```
**Container identifiers:**
```
Container ID:  93d381a6fd6ecb985323a1a76fcc7734b39d9c5bdf5898b3e447869359e1a516  (unchanged — restart, not recreate)
StartedAt:     2026-09-15T05:27:19Z
RestartCount:  0  (manual `docker restart` does not increment this counter, per Docker semantics confirmed repeatedly earlier this engagement — only restart-policy-triggered restarts do)
```

## Phase 3 — Immediate verification

**Startup:** `docker ps` shows `Up ... (healthy)` within ~40 seconds of the restart. Startup log:
```
Stopping flows -> Stopped flows -> Welcome to Node-RED -> ... -> Starting flows -> Started flows
[mqtt-broker] Connected to broker: mqtts://iot.promlogix.com:8883
```
**Zero errors or exceptions** in the startup sequence.

**MQTT subscriptions active (all 3, confirmed post-restart):**
```
a375c54a616839e2   📡 MQTT Vibration        factory/+/machine/+/vibration
s16trendmqtt0001   📡 MQTT Trend [S16]      factory/+/machine/+/trend
s17dhmqtt000001    📡 MQTT Device Health [S17]  factory/+/machine/+/device-health
```

**Node count:** post-deploy live flow re-fetched and confirmed **59 nodes**, matching pre-deploy exactly.

**Diff vs. the pre-deploy rollback backup:** re-confirmed independently after the restart — `changed vs pre-deploy backup: ['adf3dc5f003a516c', 'e982d76b3ebe0b06']` — only the two intended function bodies differ; nothing else changed by the deploy/restart itself.

**Telemetry reaching the pipeline:** confirmed via a fresh live poll immediately after startup — `last_seen: 2026-09-15T05:28:12Z` (fresh, ~1 minute after restart), full vibration/trend/device-health data present and current.

**Dashboard/API health:**
```
/machine/?plant=plant01&machine=pump01        -> 200
/api/machine/plant01/pump01                    -> 200
/api/overview                                  -> 200
/api/machines/pump01/trend?range=5m (trend)    -> 200
```
All healthy, no regression.

## Phase 4 — LINE UAT (real production conditions only — no test buttons used)

**Important, honest constraint on this section:** per instructions, the manual test-inject buttons were not used. The real `pump01` machine was, and remained throughout the entire deployment and monitoring window, in a stable **RUNNING / vibration-valid / NORMAL** state — confirmed by a full 150-second bounded live-log observation immediately after deploy that produced **zero** log lines of any kind (no `VIB_UNAVAILABLE`, no alert/recovery activity), consistent with Rule 1 (dedup: no state change → no new LINE message). No real fault, stop, or vibration-unavailable condition occurred naturally during this session's monitoring window to exercise cases B, C, D, E, or G as an actual live LINE send.

Where a case could not be exercised by a real live trigger, this is reported as **NOT OBSERVED (live)** rather than fabricated — but is still backed by the **structural, code-level proof and the isolated JS-harness execution of this exact, now-deployed code**, completed and independently re-verified in the prior review (`LINE_PATCH_FINAL_REVIEW_20260915.md`). Because the file now running in production is byte-for-byte identical to the staged file that was exhaustively tested (confirmed by the matching SHA256 above), those results carry over with full confidence to this deployment — they are not being re-asserted from memory, they are the same code, now live.

| Case | Live status | Evidence |
|---|---|---|
| **A.** NORMAL + vibration valid | **OBSERVED (live)** | The real machine's continuous state throughout this session: `alarm_level=NORMAL`, `velocity_data_valid=true`, `alarm_level_live=true`, `motor_state=RUNNING`. No *new* LINE message fired (Rule 1 dedup — state hadn't changed), but this is the exact live input the deployed code is now processing every publish cycle. Isolated-harness proof (pre-deploy): clean `✅ กลับสู่ NORMAL` / `⚙️ Motor State: RUNNING` / real Velocity RMS number, no misleading Health field (removed entirely). |
| **B.** WARNING + vibration valid | **OBSERVED (live) — updated after this report was first written.** | A real WARNING occurred on `pump01` at 2026-09-15 12:36:49 Bangkok (05:36:49 UTC). See `LIVE_UAT_WARNING_EVENT_20260915.md` for full detail: directly confirmed via an actual LINE screenshot viewed by this session (not a text relay), plus two independent live-API polls minutes later. The clean `⚠️ แจ้งเตือน WARNING!` header was used, `⚙️ Motor State: RUNNING` was shown, no `❤️ Health` line appeared, and a real (non-fabricated) velocity value was displayed — the first genuine live-production confirmation of this patch's behavior. |
| **C.** CRITICAL + vibration valid | **NOT OBSERVED (live)** — structurally verified only | Same reasoning as B. |
| **D.** RUNNING + vibration unavailable | **NOT OBSERVED (live)** — structurally verified only | No natural occurrence during the monitoring window (the machine remained healthy throughout). Structural proof (unchanged since the final review, same deployed code): the string `'✅ กลับสู่ NORMAL'` occurs exactly once in the deployed `e982d76b3ebe0b06`, gated behind `verdictLive`, which is `false` whenever vibration is unavailable — the qualified "unconfirmed — vibration data unavailable" wording is produced instead, with no fake vibration number and Motor State always shown separately. |
| **E.** STOPPED | **NOT OBSERVED (live)** — structurally verified only | Motor remained RUNNING throughout. Structural proof: `verdictLive` requires `motor_code === 2`; a STOPPED payload cannot satisfy this, so the same non-"NORMAL" qualified wording applies (see the disclosed STOPPED-wording scope note from the final review — unchanged). |
| **F.** RUNNING + RPM invalid + vibration valid | **STRUCTURALLY CONFIRMED, live rpm_valid tracked but not seen false** | Live `rpm_valid` has been `true` throughout this session (consistent with every observation this engagement — no live RPM-invalid condition occurred). The deployed code's `alarm_verdict_live` formula and `velRms` computation do not reference `rpm_valid` anywhere (confirmed by source inspection of the now-live code, identical to the reviewed staged copy) — RPM validity structurally cannot affect the vibration verdict or Motor State text. |
| **G.** CRITICAL → automatic recovery → NORMAL | **NOT OBSERVED (live)** — mechanism structurally confirmed unchanged | No CRITICAL occurred during this session. The Unified State Engine (`ef04bfbbd91995e0`) — the component owning the recovery mechanism — is confirmed, by direct post-deploy diff, **byte-identical** to its pre-deploy state; no new delay/debounce was introduced anywhere in this deploy. |

## Phase 5 — Dashboard vs. LINE verdict cross-check

Only one real, live-observable state existed throughout this session (NORMAL/RUNNING/valid), so a live side-by-side comparison is limited to that one case:

| Field | Dashboard (`/api/machine/plant01/pump01`) | LINE (deployed code, same input) | Match? |
|---|---|---|---|
| Verdict | `alarm_level=NORMAL`, `alarm_level_live=true` | `verdictLive=true` → clean `✅ กลับสู่ NORMAL` | **Yes** |
| Vibration | `velocity_data_valid=true`, `velocity_rms_overall_mms=0.287` | `Velocity RMS: 0.29 mm/s (Source: FIFO-DSP)` (same underlying field) | **Yes** |
| Motor state | `motor_state=RUNNING` | `Motor State: RUNNING` (new field, added by this patch) | **Yes** |

The three specific proofs required by Phase 5 (`CRITICAL=CRITICAL`, `WARNING=WARNING`, `UNAVAILABLE never presented as NORMAL`) were **not exercisable live** for the same reason as Phase 4's B/C/D — no such real condition occurred. They remain proven at the code/structural level (both `contract.py`'s `alarm_live` formula and the deployed `alarm_verdict_live` formula are now identical: `(vibration_status=="OK") and (motor_code==2)`), and were exhaustively exercised in the pre-deploy isolated harness against this exact, now-live code.

## Phase 6 — Monitoring

```
Node-RED exceptions since deploy:            0
Flow startup errors:                         0
LINE send failures:                          0 (no LINE send was triggered at all during the monitoring window)
Unexpected repeated notifications:           none observed (none were sent)
```
**InfluxDB:** no regression — the Dashboard API (which reads directly from InfluxDB) returned fresh, valid data throughout (`data_quality.vibration.age_s` in single digits at every check).
**MQTT:** healthy — broker reconnect logged cleanly at startup, all 3 subscriptions active, live telemetry (`last_seen`) advancing continuously post-deploy.
**API:** no regression — all 4 tested Dashboard/API routes returned 200 throughout.

## Rollback status

**Not needed.** No regression was detected at any point in this deployment. The rollback artifact remains available if ever required:
```
/opt/iot-stack/backups/nodered/flows_PRE_VERDICT_FIX_20260915T052613Z.json
SHA256: f3fba6aaac72e03d2a675e0417ac29d4c4ebc35146890ae35d9472f1996d2665
```

---

## Summary

```
Pre-deploy flows SHA:    f3fba6aaac72e03d2a675e0417ac29d4c4ebc35146890ae35d9472f1996d2665
Rollback backup SHA:     f3fba6aaac72e03d2a675e0417ac29d4c4ebc35146890ae35d9472f1996d2665
Staged SHA (approved):   d10be3a0eb983bf84407b9d53e6c162249d0f58b407eaad4906cf46a2fd3a3d5
Post-deploy live SHA:    d10be3a0eb983bf84407b9d53e6c162249d0f58b407eaad4906cf46a2fd3a3d5
Changed node IDs:        adf3dc5f003a516c, e982d76b3ebe0b06 (only)
Deployment timestamp:    2026-09-15T05:27:15Z
Container:               93d381a6fd6ecb985323a1a76fcc7734b39d9c5bdf5898b3e447869359e1a516 (restarted, not recreated)
StartedAt / RestartCount: 2026-09-15T05:27:19Z / 0
Dashboard/API regression: NONE
InfluxDB/MQTT regression: NONE
Node-RED errors:          NONE
Rollback performed:       NO — not required
```

**Deployment outcome: SUCCESSFUL.** The approved patch is live and byte-verified. Full case-by-case LINE UAT (B, C, D, E, G) could not be exercised against a real, naturally-occurring production trigger during this session's monitoring window — the machine remained continuously healthy throughout — and this is reported honestly rather than substituted with the forbidden test buttons or fabricated evidence. Those cases rest on the exhaustive, code-identical isolated-harness validation already completed and independently re-reviewed before this deployment. Recommend continued passive monitoring of Node-RED logs for the next real CRITICAL/WARNING/vibration-unavailable/STOPPED event to capture a live confirmation opportunistically, without forcing one.

---
*Production deployment record. No LINE credential or token value reproduced. Not committed to git per instructions.*
