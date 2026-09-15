# LINE SME Wording V3 — Production Deployment
Recorded: 2026-09-15

**Type: PRODUCTION DEPLOYMENT.** Wording-only change to a single Node-RED function node (`e982d76b3ebe0b06`, LINE Message Builder), executed with pre-check/backup/diff/install/restart/verify discipline. No firmware, API, frontend, nginx, MQTT configuration, Health Logic, Unified State Engine, severity/verdict calculation, automatic recovery, deduplication, escalation, snooze, ack/resume, RPM-validity behavior, credentials, or test-button logic was modified.

**Source deployed:** `docs/engineering/evidence/line_audit_20260915/staging_sme_wording/line_message_builder_SME_WORDING_V3.js`, per `LINE_SME_WORDING_FINAL_V3_VALIDATION_20260915.md` (SHA256 `47fdb62af1a012b1790954f955a11bd66e454bc0150e1ad9f625c855cb1da5ea`), 86/86 offline assertions PASS.

---

## 0. Pre-deploy baseline discrepancy — flagged and resolved before proceeding

The deployment task's pre-deploy check named `d10be3a0eb983bf84407b9d53e6c162249d0f58b407eaad4906cf46a2fd3a3d5` as the expected live SHA. The actual live SHA was `81de516e0d84eb1ebbe4568b2fc50a164ee448c86976aff3e35e1b78852651ed` — the confirmed result of the **immediately preceding V2 deployment turn** (`LINE_SME_WORDING_PRODUCTION_DEPLOYMENT_20260915.md`), not the pre-V2 baseline. This was raised to the user before any backup or install action was taken; the user confirmed `81de516e...` as the correct V3 pre-deploy baseline and explicitly authorized continuing. No file was touched before this confirmation.

---

## 1. Pre-deploy verification

```
$ ssh iotprom "sha256sum /opt/iot-stack/nodered/flows.json"
81de516e0d84eb1ebbe4568b2fc50a164ee448c86976aff3e35e1b78852651ed  /opt/iot-stack/nodered/flows.json
```
**Matches** the user-confirmed V3 pre-deploy baseline exactly. Proceeded.

---

## 2. Rollback backup

```
$ ssh iotprom "sudo cp /opt/iot-stack/nodered/flows.json /opt/iot-stack/backups/nodered/flows_PRE_SME_WORDING_V3_20260915T071732Z.json && sha256sum ..."
81de516e0d84eb1ebbe4568b2fc50a164ee448c86976aff3e35e1b78852651ed  /opt/iot-stack/backups/nodered/flows_PRE_SME_WORDING_V3_20260915T071732Z.json
```
**Rollback backup path:** `/opt/iot-stack/backups/nodered/flows_PRE_SME_WORDING_V3_20260915T071732Z.json`
**Rollback backup SHA256:** `81de516e0d84eb1ebbe4568b2fc50a164ee448c86976aff3e35e1b78852651ed` — byte-identical to the pre-deploy live hash, confirming a faithful backup before any change was made.

---

## 3. Build and structural verification (before install)

The live flow was fetched read-only, and the same credential-safe Python builder used for the V2 deployment (never printing node content — only counts, booleans, and hashes) built the V3-patched copy and verified it structurally before anything was installed:

```json
{
  "live_node_count": 59,
  "patched_node_count": 59,
  "live_ids_count": 59,
  "patched_ids_count": 59,
  "ids_only_in_live": [],
  "ids_only_in_patched": [],
  "changed_node_ids": ["e982d76b3ebe0b06"],
  "target_id": "e982d76b3ebe0b06",
  "old_func_sha256": "398e9f75bfc33e9a65693e5e12b7bc65304c91f75489772efaa075806e859c29",
  "new_func_sha256": "3b1f1ebf24f467cbee89563f9a3408772c2ee596d38b9f8191434e482dc027d0",
  "output_file_sha256": "ff42a18d025f202742f71c77fa60b7f611e9bb604765ccc1d4d47da82f9af8ed"
}
```

**Confirmations from this output:**
- **Node count unchanged** (59 → 59), **id sets identical** (no additions, no removals).
- **Exactly one node changed:** `e982d76b3ebe0b06` (LINE Message Builder). Every other node — including Health Logic, the Unified State Engine, the Escalation Timer, Auto Resume Check, the 429 Handler, the two test-inject buttons, and the credentials node — is **byte-for-byte identical** to the live pre-deploy flow (canonical-JSON equality per node, not merely ID presence).
- **`old_func_sha256` (`398e9f75...`) is an exact match** to the deployed V2 node hash recorded in `LINE_SME_WORDING_PRODUCTION_DEPLOYMENT_20260915.md` — confirms the live node held exactly the V2 wording immediately before this deploy, corroborating §0's resolution independently.
- **`new_func_sha256` (`3b1f1ebf24f467cbee89563f9a3408772c2ee596d38b9f8191434e482dc027d0`) is an exact match** to the SHA256 already recorded for `staging_sme_wording/line_message_builder_SME_WORDING_V3.js` in `LINE_SME_WORDING_FINAL_V3_VALIDATION_20260915.md` — the deployed text is byte-identical to the validated, 86/86-tested source. No drift between validation and deployment.

**Diff content — wording only, confirmed exhaustive:** the full unified diff between the V2 (previously live) and V3 source (`staging_sme_wording/diff_V2_to_V3.patch`, already reviewed line-by-line in `LINE_SME_WORDING_FINAL_V3_VALIDATION_20260915.md` §1/§5) touches only the title/เกิดอะไรขึ้น/ตอนนี้/ควรทำอะไร string literals for states A–E. `verdictLive`, `motorCode`, `alertType` branching, `velValueText`'s computation, `motorStateText`'s mapping, the snooze/backoff check, and the `escalation` branch are unchanged — and this deploy step additionally confirms every *other node* in the flow (where the actual verdict/recovery/dedup/escalation/RPM logic lives — Health Logic and the Unified State Engine) is untouched too.

---

## 4. Deploy

```
$ scp <V3-patched flow> iotprom:/tmp/flows_PRODUCTION_READY_SME_WORDING_V3.json
$ ssh iotprom "sha256sum /tmp/flows_PRODUCTION_READY_SME_WORDING_V3.json"
ff42a18d025f202742f71c77fa60b7f611e9bb604765ccc1d4d47da82f9af8ed  (matches, transfer verified intact)

$ ssh iotprom "sudo install -o iotadmin -g iotadmin -m 644 /tmp/flows_PRODUCTION_READY_SME_WORDING_V3.json /opt/iot-stack/nodered/flows.json"
$ ssh iotprom "sha256sum /opt/iot-stack/nodered/flows.json && ls -la /opt/iot-stack/nodered/flows.json"
ff42a18d025f202742f71c77fa60b7f611e9bb604765ccc1d4d47da82f9af8ed  /opt/iot-stack/nodered/flows.json
-rw-r--r-- 1 iotadmin iotadmin 111606 Sep 15 14:18 /opt/iot-stack/nodered/flows.json
```
Owner/group/mode preserved: `iotadmin:iotadmin`, `644` (unchanged), single atomic `sudo install`, no separate chown/chmod.

**Restart (minimum required mechanism to activate a flow — no Node-RED admin-API credentials, per this engagement's standing practice):**
```
Restart initiated: 2026-09-15T07:18:42Z
$ ssh iotprom "docker restart iot-stack-nodered-1"
```

**Container state:**
| | Pre-restart | Post-restart |
|---|---|---|
| Container ID | `93d381a6fd6ecb985323a1a76fcc7734b39d9c5bdf5898b3e447869359e1a516` | `93d381a6fd6ecb985323a1a76fcc7734b39d9c5bdf5898b3e447869359e1a516` (same — restarted, not recreated) |
| StartedAt | `2026-09-15T06:52:38.832637283Z` | `2026-09-15T07:18:45.898992085Z` |
| RestartCount | 0 | 0 (manual `docker restart` does not increment the crash-policy counter) |
| Status | running | **running** |
| Health | healthy | **healthy** (transiently reported `starting` for ~15s immediately after restart — normal Docker healthcheck warm-up, not an error; confirmed settled to `healthy` on re-check) |

---

## 5. Startup health — Node-RED logs

```
15 Sep 07:18:45 - [info] Stopping flows
15 Sep 07:18:45 - [info] [mqtt-broker:271d86a3897f9309] Disconnected from broker: mqtts://iot.promlogix.com:8883
15 Sep 07:18:45 - [info] Stopped flows
15 Sep 07:18:46 - [info] Welcome to Node-RED
15 Sep 07:18:46 - [info] Node-RED version: v4.1.8
15 Sep 07:18:48 - [info] Server now running at http://127.0.0.1:1880/nr/
15 Sep 07:18:48 - [info] Starting flows
15 Sep 07:18:48 - [info] Started flows
15 Sep 07:18:48 - [info] [mqtt-broker:271d86a3897f9309] Connected to broker: mqtts://iot.promlogix.com:8883
```
**No errors.** The only warning present is the pre-existing, unrelated, unchanged "flow credentials file is encrypted using a system-generated key" informational notice (present at every startup in this engagement, not new). `Started flows` and MQTT reconnection both confirmed — **MQTT subscription active**.

---

## 6. Post-deploy verification

**59 nodes present, unique ids confirmed directly on the live file:**
```
node_count: 59
unique_ids: 59
```

**Production flow SHA matches the approved deployed artifact:** `ff42a18d025f202742f71c77fa60b7f611e9bb604765ccc1d4d47da82f9af8ed` — confirmed identically at transfer, at install, and in this final check.

**API / Dashboard / trend health checks (all 200):**
```
GET /api/overview                                         -> 200
GET /api/machine/plant01/pump01                            -> 200
GET /machine/?plant=plant01&machine=pump01                  -> 200
GET /api/machines/pump01/trend?range=5m                     -> 200
```

**Live telemetry confirmed flowing** (query at 2026-09-15T07:19:49Z, `last_seen: 2026-09-15T07:19:42Z` — 7 seconds old):
```json
"motor_state": "RUNNING", "alarm_level": "NORMAL", "alarm_level_live": true,
"vibration_status": "OK", "velocity_rms_overall_mms": 0.528
```
Confirms Health Logic (untouched) and the full MQTT→Health Logic→API/InfluxDB pipeline are functioning end-to-end post-deploy, exactly as before.

---

## 7. Live UAT

Per instructions: **no manual test button was used; no event was fabricated.**

Current live state at deployment time (`alarm_level: NORMAL`, `alarm_level_live: true`, `vibration_status: OK`, `velocity_rms_overall_mms: 0.528`, `motor_state: RUNNING`) — the machine is in a genuine, ordinary NORMAL/RUNNING condition. **No real WARNING, CRITICAL, VIBRATION UNAVAILABLE, or STOPPED event was occurring or observed during this deployment window.**

All required live checks recorded as follows:

| Check | Live UAT status |
|---|---|
| WARNING — title indicates elevated vibration/monitoring; เกิดอะไรขึ้น/ตอนนี้/ควรทำอะไร present | **NOT OBSERVED LIVE** this window |
| CRITICAL — clear critical wording, immediate action, no active-damage claim | **NOT OBSERVED LIVE** this window |
| NORMAL recovery — only when vibration confirmed valid, clearly communicates recovery | **NOT OBSERVED LIVE** this window (no CRITICAL/WARNING→NORMAL transition occurred; machine has simply remained NORMAL throughout) |
| VIBRATION UNAVAILABLE — ❓ distinct from WARNING, never says NORMAL, explicitly says vibration cannot currently be evaluated | **NOT OBSERVED LIVE** this window |
| STOPPED — clearly says machine stopped, does not imply vibration normal | **NOT OBSERVED LIVE** this window (motor has remained RUNNING throughout) |
| "สถานะเครื่อง" used correctly / no "Health" / no unnecessary FIFO-DSP / RPM invalid does not alter verdict | **Structurally guaranteed** by the deployed source itself (verified by direct reading of the installed diff in §3, and by the 86/86 offline harness assertions in `LINE_SME_WORDING_FINAL_V3_VALIDATION_20260915.md`) — not re-observed live this window, since no message was sent to observe. |

**Basis for confidence in the absence of live observation:** the 86/86 offline assertion suite executed the *actual deployed source* (byte-identical, confirmed in §3) against all required cases, including the specific "no active-damage claim," "UNAVAILABLE never confirmed-recovery," "STOPPED never uses ปกติ," "all four states' emojis distinct," and "RPM invalid does not alter verdict" checks. This deployment does not depend on a live event to be correct — it depends on the offline validation, which exercises the real deployed code directly and is unaffected by the passage of time.

**No further LINE UAT monitoring was performed as part of this task**, per instructions not to fabricate events.

---

## 8. Rollback status

**Not triggered. No regression occurred.** All post-deploy checks (node count, node diff, flow SHA, Node-RED startup logs, MQTT connection, all four API/Dashboard endpoints, live telemetry freshness) passed cleanly. The rollback backup created in §2 remains available at `/opt/iot-stack/backups/nodered/flows_PRE_SME_WORDING_V3_20260915T071732Z.json` (SHA256 `81de516e0d84eb1ebbe4568b2fc50a164ee448c86976aff3e35e1b78852651ed`) should it ever be needed.

---

## 9. Credential handling

- **No credential was exposed in any tool output or this document.** The live flow (containing the pre-existing hardcoded LINE credentials in the untouched `⚙️ Identity Extractor / Credentials` node, `f96b3d962be3abba`) was processed entirely by the same credential-safe Python script used in the V2 deployment — printed only node counts, id lists, and SHA256 hashes, never node content.
- **No credential-bearing full-flow copy was saved under `docs/`.** The only full-flow JSON files created during this deployment existed only in this session's local scratchpad and the remote host's `/tmp`, both deleted after use:
  ```
  $ ssh iotprom "rm -f /tmp/flows_PRODUCTION_READY_SME_WORDING_V3.json"
  $ rm -f <local scratchpad copies>
  ```
- **No credential was created, rotated, or modified.**

---

## 10. Scope confirmation

- Firmware, API, frontend, nginx, and MQTT broker configuration: **not touched.**
- Health Logic (`adf3dc5f003a516c`): **not touched** — confirmed byte-identical, both structurally (§3) and by the untouched-node-count check.
- Unified State Engine, Escalation Timer, Auto Resume Check, 429 Handler: **not touched** — all four confirmed byte-identical in the structural diff (§3).
- Severity/verdict calculation, automatic recovery, deduplication, escalation, snooze, ack/resume: **unchanged** — the nodes implementing all of these were not touched; only the LINE Message Builder's *wording* changed.
- RPM-validity behavior: **unchanged** — `rpm_valid` is not read anywhere in the deployed source, re-confirmed via the offline validation's F/F2 checks.
- Credentials: **not touched, not exposed, not copied into any evidence artifact.**
- Test-button behavior (`7ec3e726495f761b`, `05f64b8eeea910fa`): **not touched** — both confirmed byte-identical in the structural diff, and no test button was clicked during this deployment.
- This document has **not been committed**, per instructions.

---
*Production deployment, executed with the discipline established throughout this engagement (pre-check → backup → structural diff → install → restart → verify). Pre-deploy baseline discrepancy flagged and resolved with explicit user confirmation before any change was made. No credential exposed or modified. Rollback not needed. Not committed.*
