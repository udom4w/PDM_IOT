# Node-RED Editor Public Exposure Remediation — Execution Record
Change timestamp: 2026-09-15T04:26:26Z (nginx reload signal) — verification completed by 2026-09-15T04:26:49Z

Scope executed: **only** the `location /nr/` block in the `dash.promlogix.com` server section of `/opt/iot-stack/nginx.conf`. No other location, file, container, credential, MQTT, InfluxDB, or Cloudflare setting was touched. Node-RED's own `flows.json`/application config was not opened or modified.

---

## Pre-check

**Pre-change config SHA256:** `a50f2559ea2738c392e429e669491382bead30aa6e8a3bfde45bee48c918f693`
(This is the post-InfluxDB-remediation state from the prior execution — confirmed identical to that report's recorded post-change hash before starting this change.)

**InfluxDB remediation still active:** confirmed live — `https://influx.promlogix.com/` → 401, before touching anything in this task.

**Location map confirmed (line numbers in the pre-change file):**
```
line 49:  location /machine/ {
line 83:  location /api/ {
line 93:  location /api/ops/ {
line 124: location /nr/ {
line 138: location /line/callback {
```

**Precedence verification:** `/nr/` and `/line/callback` are disjoint URL prefixes — the string `/line/callback` does not begin with `/nr/`, so no nginx longest-prefix-match ambiguity is even possible between them. Adding `auth_basic` to `location /nr/` cannot intercept requests to `/line/callback` under any circumstance; this was confirmed by inspection before editing, and reconfirmed empirically after the change (Section "Verification" below).

**Anonymous baseline (before):**
| Route | HTTP |
|---|---|
| `/nr/` | 200 |
| `/line/callback` (safe GET, no payload) | 404 (Node-RED's webhook `http in` node only registers POST; a GET reaching Node-RED and getting its own 404 is expected, non-destructive baseline behavior) |
| `/machine/` | 200 |
| `/api/machine/plant01/pump01` | 200 |

---

## Change applied

Exact diff:
```diff
     location /nr/ {
+      auth_basic           "PromLogix Ops";
+      auth_basic_user_file /etc/nginx/ops.htpasswd;
       proxy_pass http://iot-stack-nodered-1:1880/nr/;
       proxy_read_timeout 300;
       proxy_connect_timeout 300;
```
Reused the existing bcrypt `ops.htpasswd` — no new secret, no new mount, no container change.

**`nginx -t`:**
```
nginx: the configuration file /etc/nginx/nginx.conf syntax is ok
nginx: configuration file /etc/nginx/nginx.conf test is successful
```
PASS.

**Reload:**
```
docker exec nginx nginx -s reload
2026/09/15 04:26:26 [notice] 293#293: signal process started
```
Post-reload nginx container check: `StartedAt: 2026-09-14T08:14:57Z`, `RestartCount: 0` — unchanged from before, confirming a graceful reload, not a restart.

---

## Verification

| Check | Before | After |
|---|---|---|
| `/nr/` | 200 | **401**, with `WWW-Authenticate: Basic realm="PromLogix Ops"` confirmed present |
| `/line/callback` (safe GET) | 404 | **404 — identical**, still reaching Node-RED (response carries Node-RED's own `Content-Security-Policy`/`Access-Control-Allow-Origin` headers, not an nginx 401) — proves the new auth rule did not leak onto this route |
| `/machine/` | 200 | **200** |
| `/api/machine/plant01/pump01` | 200 | **200** |
| `/api/overview` | — | **200** |
| Trend endpoint (`/api/machines/pump01/trend?range=5m`) | — | **200** |
| `/shared/api.js` | — | **200** |
| `/api/ops/actions` GET | — | **404** (unchanged — no GET handler, not new) |
| `/api/ops/actions` POST | — | **401** (unchanged — auth still enforced) |
| InfluxDB `/`, `/health`, `/ping`, `/metrics` | — | **all 401** — prior remediation still fully active |
| nginx error log since reload | — | **0** error/crit/emerg lines |

No webhook payload was ever sent to `/line/callback`; the safe GET used produces no downstream Node-RED flow execution (confirmed by the unchanged 404, which is the route's normal "wrong method" response, not a processed event).

**Node-RED container:**
```
ID:           93d381a6fd6ecb985323a1a76fcc7734b39d9c5bdf5898b3e447869359e1a516  (unchanged)
StartedAt:    2026-09-15T01:31:30Z  (unchanged — predates this entire task)
RestartCount: 0
```
Not restarted, not reconfigured, not touched in any way by this change.

---

## Rollback

**Not needed** — no regression was detected on any checked route.

**Rollback path, if ever required:**
```
docs/engineering/evidence/public_exposure_audit_20260915/nginx.conf.PRE_NR_FIX_20260915T042500Z.bak
SHA256: a50f2559ea2738c392e429e669491382bead30aa6e8a3bfde45bee48c918f693
```
Procedure: copy this file back to `/opt/iot-stack/nginx.conf`, verify its SHA256 matches, `docker exec nginx nginx -t`, `docker exec nginx nginx -s reload`, then re-run the verification table above.

A redundant on-host safety copy made during application (`/opt/iot-stack/nginx.conf.runtime_before_nr_apply`) was diff/hash-verified identical to the same pre-change state and then removed, leaving the one canonical backup above as the rollback artifact.

---

## Post-change config

**Post-change config SHA256:** `dadc699eed6e2f8930e63519710a2abee7afe30c3cfc23dd2c88e8e7095cc07c`

## File-scope confirmation

On the server: only `/opt/iot-stack/nginx.conf` changed (verified by SHA256 before/after). No other file, container, or credential was touched.

Locally: only this new evidence report and its accompanying backup file (`nginx.conf.PRE_NR_FIX_20260915T042500Z.bak`) were added under `docs/engineering/evidence/public_exposure_audit_20260915/`; nothing else in the repository was modified.

---

## FINAL VERDICT

```
NODE-RED PUBLIC EXPOSURE REMEDIATION = PASS
```
- `/nr/` is now protected (401, correct `auth_basic` challenge). ✅
- `/line/callback` behavior is unchanged (identical 404, still reaching Node-RED, no auth added). ✅
- Dashboard, telemetry API (including the exact trend endpoint used), and `/shared/` all remain public and healthy (200). ✅
- `/api/ops/` authentication unchanged. ✅
- InfluxDB protection from the prior remediation remains fully active (401 on all four paths). ✅
- Node-RED container was never restarted or reconfigured. ✅
- No new nginx errors. ✅

No rollback was necessary.

---
*Not committed to git per instructions.*
