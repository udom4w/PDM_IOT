# Grafana /metrics Public Exposure Remediation — Execution Record
Change timestamp: 2026-09-15T04:36:24Z (nginx reload signal) — verification completed by 2026-09-15T04:36:40Z

Scope executed: **only** a new `location = /metrics` block added to the `dash.promlogix.com` server section of `/opt/iot-stack/nginx.conf`. No other location was modified, no container was restarted, no security headers or rate limiting were added, no firewall/Cloudflare/credential/MQTT/InfluxDB/Node-RED-flow change was made.

---

## Phase A — Pre-check

**Pre-change config SHA256:** `dadc699eed6e2f8930e63519710a2abee7afe30c3cfc23dd2c88e8e7095cc07c`
(This is the post-Node-RED-remediation state from the prior execution — confirmed identical to that report's recorded post-change hash before starting this change.)

**Currently active remediations confirmed live before touching anything:**
```
influx.promlogix.com/ -> 401
dash.promlogix.com/nr/ -> 401
```

**Grafana `/metrics` routing identified:** no dedicated `/metrics` location existed anywhere in the config — it fell through to the generic Grafana catch-all `location /` (line 111 in the pre-change file), which proxies to `iot-stack-grafana-1:3000` with no path-specific handling. Live testing confirmed the exact working path is `/metrics` (no trailing slash) — this matches the source audit's original finding.

**Anonymous baseline (before):**
| Route | HTTP | Notes |
|---|---|---|
| `dash.promlogix.com/metrics` | 200 | real Prometheus metrics returned, headers included Grafana's own `Cache-Control: no-store`, `X-Content-Type-Options: nosniff`, `X-Frame-Options: deny` (pre-existing Grafana defaults, not added by this task) |
| `dash.promlogix.com/` (Grafana root) | 302 → `/overview/` | nginx's own `location = /` rule, unrelated to this change |
| `/machine/` | 200 | |
| `/api/machine/plant01/pump01` | 200 | |
| `/api/overview` | 200 | |
| trend endpoint (`/api/machines/pump01/trend?range=5m`) | 200 | |

---

## Phase B — Change applied

Exact diff:
```diff
       proxy_pass        http://iot-stack-api-1:8088/api/ops/;
     }
+    location = /metrics {
+      auth_basic           "PromLogix Ops";
+      auth_basic_user_file /etc/nginx/ops.htpasswd;
+      proxy_pass http://iot-stack-grafana-1:3000;
+      proxy_set_header Host $host;
+      proxy_set_header X-Real-IP $remote_addr;
+      proxy_set_header X-Forwarded-For $proxy_add_x_forwarded_for;
+      proxy_set_header X-Forwarded-Proto $scheme;
+    }
     location / {
       proxy_pass http://iot-stack-grafana-1:3000;
       proxy_read_timeout 300;
```
An exact-match (`location =`) block was used deliberately so it intercepts only the literal `/metrics` path — nginx's exact-match locations always outrank prefix matches (like the catch-all `location /`) regardless of file order, so this is correct and unambiguous. Reused the existing bcrypt `ops.htpasswd` — no new secret, no new mount, no container change. No `add_header` or `limit_req` directive was added, per instructions.

**`nginx -t`:**
```
nginx: the configuration file /etc/nginx/nginx.conf syntax is ok
nginx: configuration file /etc/nginx/nginx.conf test is successful
```
PASS.

**Reload:**
```
docker exec nginx nginx -s reload
2026/09/15 04:36:24 [notice] 308#308: signal process started
```
Post-reload nginx container check: `StartedAt: 2026-09-14T08:14:57Z`, `RestartCount: 0` — unchanged, confirming a graceful reload, not a restart.

---

## Phase C — Verify

| Check | Before | After |
|---|---|---|
| `dash.promlogix.com/metrics` | 200 | **401**, `WWW-Authenticate: Basic realm="PromLogix Ops"` confirmed present |
| Grafana root `/` | 302 → `/overview/` | **302 → `/overview/` — unchanged** |
| Grafana dashboard path (`/d/<uid>`) | (not re-tested pre-change this session, but established behavior from prior audits) | **302 → `/login?redirectTo=...` — login still required, not weakened** |
| `/login` | (200, established) | **200 — unchanged** |
| `/machine/` | 200 | **200** |
| `/api/machine/plant01/pump01` | 200 | **200** |
| `/api/overview` | 200 | **200** |
| Trend endpoint | 200 | **200** |
| `/shared/api.js` | — | **200** |
| `influx.promlogix.com` `/`, `/health`, `/ping`, `/metrics` | 401 (prior remediation) | **all still 401** |
| `dash.promlogix.com/nr/` | 401 (prior remediation) | **still 401** |
| `/api/ops/actions` GET | — | **404** (unchanged, no GET handler) |
| `/api/ops/actions` POST | — | **401** (unchanged, auth still enforced) |
| nginx error log since reload | — | **0** error/crit/emerg lines |

**Container restart check (all 7 containers):**
```
nginx:                 RestartCount=0   StartedAt=2026-09-14T08:14:57Z  (unchanged)
iot-stack-api-1:        RestartCount=0   StartedAt=2026-09-15T02:04:32Z  (unchanged, predates this task)
iot-stack-nodered-1:    RestartCount=0   StartedAt=2026-09-15T01:31:30Z  (unchanged, predates this task)
iot-stack-tunnel-1:     RestartCount=0   StartedAt=2026-08-06T07:54:17Z  (unchanged)
iot-stack-mosquitto-1:  RestartCount=0   StartedAt=2026-09-10T16:08:46Z  (unchanged)
iot-stack-influxdb-1:   RestartCount=0   StartedAt=2026-08-06T07:54:17Z  (unchanged)
iot-stack-grafana-1:    RestartCount=15  StartedAt=2026-09-13T16:21:13Z  (unchanged — StartedAt predates this task by 2 days; RestartCount=15 is a pre-existing historical counter, not caused by this change, which never touches the Grafana container)
```
Every `StartedAt` predates this task's reload timestamp (04:36:24Z) — **no container was restarted by this change.**

---

## Rollback

**Not needed** — no regression was detected on any checked route.

**Rollback path, if ever required:**
```
docs/engineering/evidence/public_exposure_audit_20260915/nginx.conf.PRE_METRICS_FIX_20260915T043500Z.bak
SHA256: dadc699eed6e2f8930e63519710a2abee7afe30c3cfc23dd2c88e8e7095cc07c
```
Procedure: copy this file back to `/opt/iot-stack/nginx.conf`, verify SHA256 match, `docker exec nginx nginx -t`, `docker exec nginx nginx -s reload`, then re-run the verification table above.

A redundant on-host safety copy made during application (`/opt/iot-stack/nginx.conf.runtime_before_metrics_apply`) was hash-verified identical to this same pre-change state and then removed.

---

## Post-change config

**Post-change config SHA256:** `b43c21d9b9cf846d952dca7941a36c3f0357513473b46beb2a89767490b2e7e1`

## File-scope confirmation

On the server: only `/opt/iot-stack/nginx.conf` changed (verified by SHA256 before/after). No other file, container, or credential touched.

Locally: only this new evidence report and its accompanying backup file (`nginx.conf.PRE_METRICS_FIX_20260915T043500Z.bak`) were added under `docs/engineering/evidence/public_exposure_audit_20260915/`; nothing else in the repository was modified.

---

## FINAL VERDICT

```
GRAFANA /metrics REMEDIATION = PASS
```

## Final Acceptance (cumulative, all three remediations)
```
InfluxDB public exposure       = PASS
Node-RED /nr/ exposure         = PASS
Grafana /metrics exposure      = PASS
Customer Dashboard             = PASS
Public telemetry API           = PASS
No unexpected service changes  = PASS
```

No rollback was necessary at any stage across all three remediations.

---
*Not committed to git per instructions.*
