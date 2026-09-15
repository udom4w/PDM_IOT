# InfluxDB Public Exposure Remediation — Execution Record
Change timestamp: 2026-09-15T04:22:08Z (nginx reload signal) — verification completed by 2026-09-15T04:22:59Z

Scope executed: **only** the `influx.promlogix.com` server block in `/opt/iot-stack/nginx.conf`. No other file was touched; no other service, container, credential, firewall rule, or Cloudflare setting was changed.

---

## 1. Pre-change verification
```
Live config SHA256 (re-checked immediately before editing): d8fdb07b496a36e0c477da89c1a42fe4e43b9ef10cb7edb1aea46f4f395b148f
Expected (from INFLUX_PRECHANGE_VERIFICATION.md):            d8fdb07b496a36e0c477da89c1a42fe4e43b9ef10cb7edb1aea46f4f395b148f
Match: YES — proceeded.
```

## 2–3. Change applied

Exact diff (pre-change backup → new live config):
```diff
     ssl_certificate /etc/nginx/certs/influx.pem;
     ssl_certificate_key /etc/nginx/certs/influx.key;
     location / {
+      auth_basic           "PromLogix Ops";
+      auth_basic_user_file /etc/nginx/ops.htpasswd;
       proxy_pass http://iot-stack-influxdb-1:8086;
       proxy_read_timeout 300;
       proxy_connect_timeout 300;
```
Exactly the 2 lines identified in the pre-change verification. No other line in the file was touched. Reused the existing, already-mounted `/etc/nginx/ops.htpasswd` (bcrypt) — no new secret, no new Docker volume mount, no container recreation.

## 4. `nginx -t`
```
nginx: the configuration file /etc/nginx/nginx.conf syntax is ok
nginx: configuration file /etc/nginx/nginx.conf test is successful
```
**PASS.**

## 5. Graceful reload
```
docker exec nginx nginx -s reload
2026/09/15 04:22:08 [notice] 279#279: signal process started
```
Post-reload container check:
```
Status: running   StartedAt: 2026-09-14T08:14:57Z   RestartCount: 0
```
`StartedAt` unchanged from before the reload — confirms this was a **graceful config reload, not a container restart**.

## 6. Anonymous InfluxDB access — now blocked

| Test | Before | After |
|---|---|---|
| A. `https://influx.promlogix.com/` | 200 | **401** |
| B. `https://influx.promlogix.com/health` | 200 | **401** |
| C. `https://influx.promlogix.com/ping` | 204 | **401** |
| D. `https://influx.promlogix.com/metrics` | 200 | **401** |

Confirmed the 401 is a genuine `auth_basic` challenge, not a generic error:
```
WWW-Authenticate: Basic realm="PromLogix Ops"
```

## 7. Customer-facing services — confirmed still public and healthy

| Route | Result |
|---|---|
| E. `dash.promlogix.com/machine/?plant=plant01&machine=pump01` | **200** |
| F. `dash.promlogix.com/api/machine/plant01/pump01` | **200** |

## 8. Regression checks (exact routes from the pre-change verification)

| Route | Result |
|---|---|
| `/api/overview` | **200** |
| `/api/machines/pump01/trend?range=5m` (the actual trend endpoint the Dashboard's `shared/api.js` calls) | **200** |
| `/shared/api.js` | **200** |

No regression on any customer-facing route.

## 9. `/api/ops/` authentication — unchanged

| Method | Result | Matches pre-change baseline? |
|---|---|---|
| GET `/api/ops/actions` | 404 | Yes — identical to the pre-change test (this path simply has no GET handler; not a new condition) |
| POST `/api/ops/actions` | **401** | Yes — auth still enforced exactly as before |

## 10. nginx error log

Checked `docker logs nginx` from the reload timestamp onward: **zero** `error`/`crit`/`emerg` lines. Clean.

## 11. Record

```
Change timestamp (reload):     2026-09-15T04:22:08Z
Pre-change config SHA256:      d8fdb07b496a36e0c477da89c1a42fe4e43b9ef10cb7edb1aea46f4f395b148f
Post-change config SHA256:     a50f2559ea2738c392e429e669491382bead30aa6e8a3bfde45bee48c918f693
Diff:                          2 lines added (auth_basic, auth_basic_user_file), 0 removed, 0 elsewhere
nginx -t:                      PASS
nginx reload:                  PASS (graceful, container StartedAt unchanged, RestartCount 0)
Before/after HTTP results:     Sections 6-9 above
Rollback path:                 docs/engineering/evidence/public_exposure_audit_20260915/nginx.conf.PRE_INFLUX_FIX_20260915T041731Z.bak
                                (SHA256 d8fdb07b496a36e0c477da89c1a42fe4e43b9ef10cb7edb1aea46f4f395b148f)
```

A redundant on-host safety copy made during application (`/opt/iot-stack/nginx.conf.runtime_before_apply`) was diff-verified identical to the known pre-change hash and then removed, so the only persistent rollback artifact is the one already recorded in the evidence directory above — avoiding duplicate, potentially-conflicting backup copies on the host.

## 14. File-change confirmation

On the server: only `/opt/iot-stack/nginx.conf` changed (verified by SHA256 before/after; the pre-existing, unrelated `/opt/iot-stack/nginx.conf.bak_2026-08-26` predates this session and was not touched). No other file, container, credential, or config was modified.

Locally: only this new evidence report was created; no other file in the repository was modified in the course of this execution.

---

## FINAL VERDICT

```
INFLUX PUBLIC EXPOSURE REMEDIATION = PASS
```
- Anonymous InfluxDB access is blocked (401 on all four tested paths, correct `auth_basic` challenge). ✅
- Dashboard still loads publicly (200). ✅
- Public telemetry API still works, including the exact trend endpoint the Dashboard uses (200). ✅
- No regression detected anywhere tested, including `/api/ops/` auth behavior and the nginx error log. ✅

No rollback was necessary.

---
*Not committed to git per instructions.*
