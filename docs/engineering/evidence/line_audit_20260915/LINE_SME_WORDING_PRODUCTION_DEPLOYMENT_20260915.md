# LINE SME Wording V2 — Production Deployment
Recorded: 2026-09-15

**Type: PRODUCTION DEPLOYMENT.** Wording-only change to a single Node-RED function node (`e982d76b3ebe0b06`, LINE Message Builder), executed with pre-check/backup/diff/install/restart/verify discipline. No firmware, API, frontend, nginx, MQTT configuration, Health Logic, Unified State Engine, recovery/debounce/dedup/escalation logic, RPM-validity behavior, credentials, or test-button wiring was modified.

**Source deployed:** `docs/engineering/evidence/line_audit_20260915/staging_sme_wording/line_message_builder_SME_WORDING.js`, per `LINE_SME_WORDING_PATCH_VALIDATION_V2_20260915.md` (SHA256 `1c23b3c6571e52abbe7a364559c607d55cd697c8e1ab5a0faa24bf7a66e829f6`), 59/59 offline assertions PASS.

---

## 1. Pre-deploy verification

```
2026-09-15T06:49:xx (approx, before backup)
$ ssh iotprom "sha256sum /opt/iot-stack/nodered/flows.json"
d10be3a0eb983bf84407b9d53e6c162249d0f58b407eaad4906cf46a2fd3a3d5  /opt/iot-stack/nodered/flows.json
```
**Matches** the expected baseline exactly (`d10be3a0eb983bf84407b9d53e6c162249d0f58b407eaad4906cf46a2fd3a3d5`) — production had not drifted since the last commit-verification turn. Proceeded per instructions ("if different, STOP" — not triggered).

---

## 2. Rollback backup

```
$ ssh iotprom "sudo cp /opt/iot-stack/nodered/flows.json /opt/iot-stack/backups/nodered/flows_PRE_SME_WORDING_V2_20260915T064947Z.json && sha256sum ..."
d10be3a0eb983bf84407b9d53e6c162249d0f58b407eaad4906cf46a2fd3a3d5  /opt/iot-stack/backups/nodered/flows_PRE_SME_WORDING_V2_20260915T064947Z.json
```
**Rollback backup path:** `/opt/iot-stack/backups/nodered/flows_PRE_SME_WORDING_V2_20260915T064947Z.json`
**Rollback backup SHA256:** `d10be3a0eb983bf84407b9d53e6c162249d0f58b407eaad4906cf46a2fd3a3d5` — identical to the pre-deploy live hash, confirming a faithful, uncorrupted backup before any change was made.

---

## 3. Build and structural verification (before install)

The live flow was fetched read-only, and a Python script (never printing node content — only counts, booleans, and hashes, to avoid exposing the credentials node's content in any tool output) built the patched copy and verified it structurally before anything was installed:

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
  "old_func_sha256": "35a847a2fb74329562becbf918fad7dd4859645b5a95fccd329a97835f2b5c23",
  "new_func_sha256": "398e9f75bfc33e9a65693e5e12b7bc65304c91f75489772efaa075806e859c29",
  "output_file_sha256": "81de516e0d84eb1ebbe4568b2fc50a164ee448c86976aff3e35e1b78852651ed"
}
```

**Confirmations from this output:**
- **Node count unchanged** (59 → 59), **id sets identical** (no additions, no removals).
- **Exactly one node changed:** `e982d76b3ebe0b06` (LINE Message Builder). Every other one of the 59 nodes — including Health Logic, the Unified State Engine, the Escalation Timer, Auto Resume Check, the 429 Handler, the two test-inject buttons, and the credentials node — is **byte-for-byte identical** to the live pre-deploy flow (verified by canonical-JSON equality per node, not merely by ID presence).
- **`new_func_sha256` (`398e9f75bfc33e9a65693e5e12b7bc65304c91f75489772efaa075806e859c29`) is an exact match** to the SHA256 already recorded for `staging_sme_wording/line_message_builder_SME_WORDING.js` in `LINE_SME_WORDING_PATCH_VALIDATION_V2_20260915.md` — the deployed text is byte-identical to the validated, 59/59-tested source. No drift between validation and deployment.
- **`old_func_sha256` (`35a847a2...`) was independently confirmed** (separately, via direct string comparison after normalizing a trailing newline) to be byte-identical to `staging_sme_wording/line_message_builder_PRE_SME_WORDING_currently_deployed.js` — the previously-deployed Verdict-Fix-1 wording, confirming the pre-deploy baseline was exactly what was expected, not something unexpectedly different.

**Diff content — the two approved wording changes only, confirmed present and exhaustive:**
```diff
- text = '⚠️ ไม่สามารถยืนยันสภาพการสั่นสะเทือนได้ในขณะนี้\n\n'
+ text = '❓ ไม่สามารถยืนยันสภาพการสั่นสะเทือนได้ในขณะนี้\n\n'

- + '(เป็นสถานะปกติของเครื่องที่หยุดทำงาน ไม่ใช่ความผิดปกติของระบบ)\n\n'
+ + '(เป็นเรื่องธรรมดาสำหรับเครื่องที่หยุดทำงาน ไม่ใช่ความผิดปกติของระบบ)\n\n'
```
No verdict/recovery/dedup/escalation code differs — the entire diff (already reviewed in full in `LINE_SME_WORDING_PATCH_VALIDATION_V2_20260915.md` §1) touches only these two string literals; `verdictLive`, `motorCode`, `alertType` branching, the snooze/backoff check, and the escalation branch are untouched, and this deploy step additionally confirms every *other node* in the flow (where the actual recovery/dedup/escalation logic lives — Health Logic and the Unified State Engine) is untouched too.

---

## 4. Deploy

```
$ scp <patched flow> iotprom:/tmp/flows_PRODUCTION_READY_SME_WORDING_V2.json
$ ssh iotprom "sha256sum /tmp/flows_PRODUCTION_READY_SME_WORDING_V2.json"
81de516e0d84eb1ebbe4568b2fc50a164ee448c86976aff3e35e1b78852651ed  (matches, transfer verified intact)

$ ssh iotprom "sudo install -o iotadmin -g iotadmin -m 644 /tmp/flows_PRODUCTION_READY_SME_WORDING_V2.json /opt/iot-stack/nodered/flows.json"
$ ssh iotprom "sha256sum /opt/iot-stack/nodered/flows.json && ls -la /opt/iot-stack/nodered/flows.json"
81de516e0d84eb1ebbe4568b2fc50a164ee448c86976aff3e35e1b78852651ed  /opt/iot-stack/nodered/flows.json
-rw-r--r-- 1 iotadmin iotadmin 113808 Sep 15 13:52 /opt/iot-stack/nodered/flows.json
```
Owner/group/mode preserved: `iotadmin:iotadmin`, `644` (unchanged from before), installed via `sudo install` (single atomic operation, no separate chown/chmod step).

**Restart (only mechanism available to activate a flow — no Node-RED admin-API credentials, per this engagement's standing practice):**
```
Restart initiated: 2026-09-15T06:52:35Z
$ ssh iotprom "docker restart iot-stack-nodered-1"
```

**Container state:**
| | Pre-restart | Post-restart |
|---|---|---|
| Container ID | `93d381a6fd6ecb985323a1a76fcc7734b39d9c5bdf5898b3e447869359e1a516` | `93d381a6fd6ecb985323a1a76fcc7734b39d9c5bdf5898b3e447869359e1a516` (same — restarted, not recreated) |
| StartedAt | `2026-09-15T05:27:19.123710083Z` | `2026-09-15T06:52:38.832637283Z` |
| RestartCount | 0 | 0 (manual `docker restart` does not increment the crash-policy counter) |
| Status | running | **running** |
| Health | — | **healthy** |

---

## 5. Startup health — Node-RED logs

```
15 Sep 06:52:38 - [info] Stopping flows
15 Sep 06:52:38 - [info] [mqtt-broker:271d86a3897f9309] Disconnected from broker: mqtts://iot.promlogix.com:8883
15 Sep 06:52:38 - [info] Stopped flows
15 Sep 06:52:39 - [info] Welcome to Node-RED
15 Sep 06:52:39 - [info] Node-RED version: v4.1.8
15 Sep 06:52:41 - [info] Server now running at http://127.0.0.1:1880/nr/
15 Sep 06:52:41 - [info] Starting flows
15 Sep 06:52:41 - [info] Started flows
15 Sep 06:52:41 - [info] [mqtt-broker:271d86a3897f9309] Connected to broker: mqtts://iot.promlogix.com:8883
```
**No errors, no warnings other than the pre-existing, unrelated, unchanged "flow credentials file is encrypted using a system-generated key" informational notice** (present at every startup in this engagement, not new). `Started flows` and MQTT reconnection both confirmed — **MQTT subscription active**.

---

## 6. Post-deploy verification

**All 59 nodes present, unique ids confirmed directly on the live file (independent re-check, not reused from §3):**
```
$ ssh iotprom "python3 -c \"...json.load(open('/opt/iot-stack/nodered/flows.json'))...\""
node_count: 59
unique_ids: 59
```

**Production flow SHA matches the approved deployed artifact:** `81de516e0d84eb1ebbe4568b2fc50a164ee448c86976aff3e35e1b78852651ed` — confirmed identically at transfer, at install, and again in this final check.

**API / Dashboard / trend health checks (all 200):**
```
2026-09-15T06:53:xxZ
GET /api/overview                                       -> 200
GET /api/machine/plant01/pump01                          -> 200
GET /machine/?plant=plant01&machine=pump01                -> 200
GET /api/machines/pump01/trend?range=5m                   -> 200
```

**Live telemetry confirmed flowing** (fresh sample, 2026-09-15T06:54:07Z query, `last_seen: 2026-09-15T06:54:05Z` — 2 seconds old):
```json
"status": { "motor_state": "RUNNING", "motor_state_code": 2, "online": true, ... },
"operating_condition": { "rpm": 1064, "rpm_valid": true, "current_a": 0.7, "current_valid": true },
```
Confirms Health Logic (untouched) and the full MQTT→Health Logic→API/InfluxDB pipeline are functioning end-to-end post-deploy, exactly as before.

---

## 7. Live UAT

Per instructions: **no manual test button was used; no event was fabricated.**

Current live state at the time of this deployment (`alarm_level: NORMAL`, `alarm_level_live: true`, `vibration_status: OK`, `velocity_rms_overall_mms: 0.273`) — the machine is in a genuine, ordinary NORMAL/RUNNING condition. **No real WARNING, CRITICAL, VIBRATION UNAVAILABLE, or STOPPED event was occurring or observed during this deployment window.**

Per the explicit instruction — *"If no real STOPPED or VIBRATION UNAVAILABLE event occurs during the observation window, do NOT fabricate one. Record it as NOT OBSERVED LIVE and rely on the 59/59 isolated validation"* — all five wording states are recorded as follows:

| State | Live UAT status |
|---|---|
| WARNING → ⚠️ | **NOT OBSERVED LIVE** this window |
| CRITICAL → 🚨 | **NOT OBSERVED LIVE** this window |
| NORMAL recovery → ✅ | **NOT OBSERVED LIVE** this window (no CRITICAL/WARNING→NORMAL transition occurred; the machine has simply remained NORMAL throughout) |
| VIBRATION UNAVAILABLE → ❓ | **NOT OBSERVED LIVE** this window |
| STOPPED → "เรื่องธรรมดา" wording | **NOT OBSERVED LIVE** this window (motor has remained RUNNING throughout) |
| No "Health" field / "Motor State" present / unavailable never says NORMAL | **Structurally guaranteed** by the deployed source itself (verified by direct reading of the installed diff in §3, and by the 59/59 offline harness assertions in `LINE_SME_WORDING_PATCH_VALIDATION_V2_20260915.md`) — not re-observed live this window, since no message was sent to observe.

**Basis for confidence in the absence of live observation:** the 59/59 offline assertion suite (`test_output_sme_v2.json`) executed the *actual deployed source* (byte-identical, confirmed in §3) against all required cases, including the specific "unavailable vibration never says NORMAL" and "Motor State present, no Health field" checks. This deployment does not depend on a live event to be correct — it depends on the offline validation, which is unchanged by the passage of time since it exercises the real code directly.

**No further LINE UAT monitoring was performed as part of this task.** If the user wants live confirmation of one or more of these states, that requires either waiting for a real event or explicitly authorizing a different verification method — neither was requested here beyond "wait for a real notification when available," and none occurred within this deployment turn.

---

## 8. Rollback status

**Not triggered. No regression occurred.** All post-deploy checks (node count, node diff, flow SHA, Node-RED startup logs, MQTT connection, all four API/Dashboard endpoints, live telemetry freshness) passed cleanly. The rollback backup created in §2 remains available at `/opt/iot-stack/backups/nodered/flows_PRE_SME_WORDING_V2_20260915T064947Z.json` (SHA256 `d10be3a0eb983bf84407b9d53e6c162249d0f58b407eaad4906cf46a2fd3a3d5`) should it ever be needed.

---

## 9. Credential handling

- **No credential was exposed in any tool output or this document.** The live flow (containing the pre-existing hardcoded LINE credentials in the untouched `⚙️ Identity Extractor / Credentials` node, `f96b3d962be3abba`) was processed entirely by a Python script that printed only node counts, id lists, and SHA256 hashes — never node content.
- **No credential-bearing full-flow copy was saved under `docs/`.** The only full-flow JSON files created during this deployment (`live_flows_RAW.json`, the patched `flows_PRODUCTION_READY_SME_WORDING_V2.json`) existed only in this session's local scratchpad directory and on the remote host's `/tmp`, both of which have been deleted after use:
  ```
  $ ssh iotprom "rm -f /tmp/flows_PRODUCTION_READY_SME_WORDING_V2.json"
  $ rm -f <local scratchpad copies>
  ```
- **No credential was created, rotated, or modified.**

---

## 10. Scope confirmation

- Firmware, API, frontend, nginx, and MQTT broker configuration: **not touched.**
- Health Logic (`adf3dc5f003a516c`): **not touched** — confirmed byte-identical, both structurally (§3) and by the untouched-node-count check.
- Unified State Engine, Escalation Timer, Auto Resume Check, 429 Handler: **not touched** — all four confirmed byte-identical in the structural diff (§3).
- Recovery/debounce/dedup/escalation behavior: **unchanged** — the nodes implementing it were not touched; only the LINE Message Builder's *wording* changed.
- RPM-validity behavior: **unchanged** — `rpm_valid` is not read anywhere in the deployed source (confirmed in prior validation turns), and this deploy did not touch that.
- Credentials: **not touched, not exposed, not copied into any evidence artifact.**
- Test-button behavior (`7ec3e726495f761b`, `05f64b8eeea910fa`): **not touched** — both confirmed byte-identical in the structural diff, and no test button was clicked during this deployment.
- This document has **not been committed**, per instructions.

---
*Production deployment, executed with the discipline established throughout this engagement (pre-check → backup → structural diff → install → restart → verify). No credential exposed or modified. Rollback not needed. Not committed.*
