# PROMLOGIX — Public Exposure Remediation Plan
Date: 2026-09-15
Source audit: `docs/engineering/evidence/public_exposure_audit_20260915/PUBLIC_EXPOSURE_SECURITY_AUDIT.md`
Status: **PLANNING ONLY — nothing in this document has been executed.** No nginx reload, no container restart, no firewall change, no credential rotation was performed while producing this plan.

Design constraint honored throughout: the customer-facing Dashboard and its required read-only telemetry API are **intentionally public** and are treated as already-correct. No change below narrows, gates, or otherwise touches `/machine/`, `/overview/`, `/plant/`, `/machines/`, `/shared/`, or the telemetry routes under `/api/` (`/api/overview`, `/api/machines*`, `/api/machine/{plant}/{machine}`, `/api/machines/{id}/trend`), unless a check below explicitly finds evidence requiring it — none do.

All proposed nginx changes reuse the exact `auth_basic` + bcrypt-htpasswd pattern already proven live and working on `/api/ops/`, rather than introducing a new mechanism. All proposed changes are config-only edits to `/opt/iot-stack/nginx.conf` followed by a graceful `nginx -s reload` — none require a container restart, a new file mount, or a new secret, because the existing `/etc/nginx/ops.htpasswd` (already bind-mounted into the nginx container) is reused everywhere auth is added below. This is the deliberate "safest minimal" choice: it closes every perimeter hole with zero new moving parts and zero downtime.

---

## 1. HIGH — `influx.promlogix.com` public reverse proxy

**Currently responsible nginx block:**
```
server {
  listen 443 ssl;
  server_name influx.promlogix.com;
  ssl_certificate /etc/nginx/certs/influx.pem;
  ssl_certificate_key /etc/nginx/certs/influx.key;
  location / {
    proxy_pass http://iot-stack-influxdb-1:8086;
    ...
  }
}
```

**Current public URL(s):** `https://influx.promlogix.com/` (and every path under it — UI, `/health`, `/ping`, `/metrics`, `/api/v2/*`).

**Exact proposed restriction:**
```diff
   location / {
+    auth_basic           "PromLogix Ops";
+    auth_basic_user_file /etc/nginx/ops.htpasswd;
     proxy_pass http://iot-stack-influxdb-1:8086;
     proxy_read_timeout 300;
     proxy_connect_timeout 300;
     proxy_send_timeout 300;
     proxy_set_header Host $host;
     proxy_set_header X-Real-IP $remote_addr;
     proxy_set_header X-Forwarded-For $proxy_add_x_forwarded_for;
     proxy_set_header X-Forwarded-Proto $scheme;
   }
```
This adds one HTTP Basic auth gate in front of the *entire* InfluxDB proxy — UI, health check, metrics, and the (already token-gated) data API all now require the ops credential before nginx will even forward the request.

**Effect on other surfaces:**
| Surface | Affected? |
|---|---|
| `/machine/` | No — different `server_name` block entirely |
| `/overview/` | No |
| Public telemetry API (`/api/*` on `dash.promlogix.com`) | No |
| Grafana dashboard | No |
| MQTT | No |

**Safest minimal production change:** exactly the 2-line diff above — reuses the existing, already-mounted, already-bcrypt `ops.htpasswd`, so no new secret file or new Docker volume mount is needed. A `nginx -s reload` is sufficient; no container recreation.

**Rollback method:** keep a timestamped copy of `nginx.conf` before editing (`cp nginx.conf nginx.conf.pre-influx-auth`); to roll back, restore the copy and `nginx -s reload` again. Fully reversible in seconds, no data at risk (this is a proxy-layer change only, InfluxDB itself is untouched).

**Verification commands:**
```
# Before (expected: 200 — the vulnerable state)
curl -sk -o /dev/null -w '%{http_code}\n' https://influx.promlogix.com/health

# After (expected: 401 without credentials)
curl -sk -o /dev/null -w '%{http_code}\n' https://influx.promlogix.com/health

# After (expected: 200 with the correct ops credential)
curl -sk -u '<ops-user>:<ops-pass>' -o /dev/null -w '%{http_code}\n' https://influx.promlogix.com/health
```

**Downtime expected:** none. `nginx -s reload` is graceful — in-flight connections complete on the old config, new connections use the new config. No dropped requests for the dashboard or any other domain sharing this nginx instance.

---

## 2. MEDIUM — public Node-RED editor (`/nr/`)

**Currently responsible nginx block:**
```
location /nr/ {
  proxy_pass http://iot-stack-nodered-1:1880/nr/;
  ...
}
```

**Current public URL(s):** `https://dash.promlogix.com/nr/` (editor shell; `adminAuth` inside Node-RED already protects `/nr/flows` and other API calls, confirmed 401 live).

**Exact proposed restriction:**
```diff
   location /nr/ {
+    auth_basic           "PromLogix Ops";
+    auth_basic_user_file /etc/nginx/ops.htpasswd;
     proxy_pass http://iot-stack-nodered-1:1880/nr/;
     proxy_read_timeout 300;
     proxy_connect_timeout 300;
     proxy_send_timeout 300;
     proxy_http_version 1.1;
     proxy_set_header Upgrade $http_upgrade;
     proxy_set_header Connection $connection_upgrade;
     proxy_set_header X-Forwarded-Prefix /nr;
     proxy_set_header Host $host;
     proxy_set_header X-Real-IP $remote_addr;
     proxy_set_header X-Forwarded-For $proxy_add_x_forwarded_for;
     proxy_set_header X-Forwarded-Proto $scheme;
   }
```
This adds a second, nginx-level auth gate in front of the editor shell itself, on top of (not instead of) Node-RED's own `adminAuth`. Legitimate staff now enter the ops credential once (browser caches it) before ever reaching Node-RED's own login page.

**Verified non-side-effect:** `location /line/callback` is a separate, more-specific `location` block (`proxy_pass http://iot-stack-nodered-1:1880/nr/api/line/callback`) matched independently of `/nr/` — nginx location matching is not affected by declaration order for this pairing, so gating `/nr/` does **not** add auth to the LINE webhook callback. This was explicitly checked against the current `nginx.conf` to avoid breaking inbound LINE message delivery.

**Effect on other surfaces:**
| Surface | Affected? |
|---|---|
| `/machine/` | No |
| `/overview/` | No |
| Public telemetry API | No |
| Grafana dashboard | No |
| MQTT | No |
| LINE webhook (`/line/callback`) | No — verified separately matched, not nested under `/nr/` |

**Safest minimal production change:** same 2-line `auth_basic` addition as Item 1, same reused credential file. An optional, stronger follow-up (not required to close the immediate gap) would be an IP-allowlist (`allow <ops-ip>; deny all;`) if the ops team's source IPs are stable — deferred here per the instruction not to propose broad restrictions unless strictly necessary; `auth_basic` alone already closes the anonymous-reachability finding.

**Rollback method:** same pattern — timestamped config backup, restore + reload.

**Verification commands:**
```
# Before (expected: 200 — editor shell loads anonymously)
curl -sk -o /dev/null -w '%{http_code}\n' https://dash.promlogix.com/nr/

# After (expected: 401 without credentials)
curl -sk -o /dev/null -w '%{http_code}\n' https://dash.promlogix.com/nr/

# Confirm LINE webhook still unaffected (expected: unchanged from its pre-change status, whatever that is)
curl -sk -o /dev/null -w '%{http_code}\n' https://dash.promlogix.com/line/callback
```

**Downtime expected:** none (reload only).

---

## 3. MEDIUM — public Grafana `/metrics`

**Currently responsible nginx block:** none dedicated — `/metrics` currently falls through to the generic Grafana catch-all:
```
location / {
  proxy_pass http://iot-stack-grafana-1:3000;
  ...
}
```

**Current public URL(s):** `https://dash.promlogix.com/metrics`.

**Exact proposed restriction:** add a new, more-specific `location` block matched *before* being shadowed by the catch-all (nginx's exact-match `location =` always outranks a prefix `location /` regardless of file position, so placement in the file is not safety-critical, but it is added near the other explicit locations for readability):
```diff
+  location = /metrics {
+    auth_basic           "PromLogix Ops";
+    auth_basic_user_file /etc/nginx/ops.htpasswd;
+    proxy_pass http://iot-stack-grafana-1:3000;
+    proxy_set_header Host $host;
+    proxy_set_header X-Real-IP $remote_addr;
+    proxy_set_header X-Forwarded-For $proxy_add_x_forwarded_for;
+    proxy_set_header X-Forwarded-Proto $scheme;
+  }
   location / {
     proxy_pass http://iot-stack-grafana-1:3000;
     ...
   }
```
(InfluxDB's own `/metrics` is already covered by Item 1's server-block-wide auth — no separate rule is needed there.)

**Effect on other surfaces:**
| Surface | Affected? |
|---|---|
| `/machine/` | No |
| `/overview/` | No |
| Public telemetry API | No |
| Grafana dashboard (`/login`, `/d/*`, everything except `/metrics`) | **No** — the exact-match `location = /metrics` only intercepts that one literal path; every other Grafana path continues to hit the unauthenticated-at-nginx catch-all exactly as today, still gated by Grafana's own login (already confirmed working: 302 → `/login` for any dashboard path) |
| MQTT | No |

**Safest minimal production change:** the 7-line block above, same reused credential file, no new mount.

**Rollback method:** same pattern — remove the added block, reload.

**Verification commands:**
```
# Before (expected: 200 — real Prometheus metrics returned anonymously)
curl -sk -o /dev/null -w '%{http_code}\n' https://dash.promlogix.com/metrics

# After (expected: 401 without credentials)
curl -sk -o /dev/null -w '%{http_code}\n' https://dash.promlogix.com/metrics

# Confirm Grafana dashboard login flow is unaffected (expected: unchanged, 302 to /login)
curl -sk -D - -o /dev/null https://dash.promlogix.com/d/any-uid 2>&1 | grep -i location
```

**Downtime expected:** none (reload only).

---

## 4. LOW/MEDIUM — missing HSTS / security headers

**Currently responsible:** the `dash.promlogix.com` and `influx.promlogix.com` `server{}` blocks have no `add_header` for `X-Content-Type-Options`, `X-Frame-Options`, or `Strict-Transport-Security` anywhere. Two locations (`/machine/`, `/shared/`) already set their own `add_header Cache-Control "no-cache";`.

**Current public URL(s):** every path on both domains lacks these headers, as shown by live capture in the audit.

**Exact proposed restriction (additive, no routing/auth change):**
```diff
 server {
   listen 443 ssl;
   server_name dash.promlogix.com;
   ssl_certificate ...
   ssl_certificate_key ...
+  add_header X-Content-Type-Options nosniff always;
+  add_header X-Frame-Options DENY always;
+  add_header Strict-Transport-Security "max-age=15552000; includeSubDomains" always;
   ...
```
**Important nginx-specific detail this plan must flag:** `add_header` does *not* inherit into a `location` block that already defines its own `add_header` — nginx treats "any `add_header` present at this level" as replacing the entire inherited set, not merging with it. Since `/machine/` and `/shared/` already have their own `add_header Cache-Control ...`, the three new headers must be **explicitly repeated inside those two location blocks** as well, or they will silently not appear on those two paths:
```diff
   location /machine/ {
     alias /usr/share/nginx/machine/;
     index index.html;
     try_files $uri $uri/ /machine/index.html;
     add_header Cache-Control "no-cache";
+    add_header X-Content-Type-Options nosniff always;
+    add_header X-Frame-Options DENY always;
+    add_header Strict-Transport-Security "max-age=15552000; includeSubDomains" always;
   }
   ...
   location /shared/ {
     alias /usr/share/nginx/machine/shared/;
     add_header Cache-Control "no-cache";
+    add_header X-Content-Type-Options nosniff always;
+    add_header X-Frame-Options DENY always;
+    add_header Strict-Transport-Security "max-age=15552000; includeSubDomains" always;
   }
```
Apply the same three lines to the `influx.promlogix.com` server block.

**Pre-check needed before applying `X-Frame-Options: DENY` (not resolvable from this audit alone):** confirm nothing currently embeds the dashboard or a Grafana panel in an `<iframe>` from a different origin. No such usage was found in the reviewed frontend/Grafana config, but the frontend's full source was not exhaustively reviewed in this engagement. If any embedding use case exists, use `X-Frame-Options: SAMEORIGIN` instead of `DENY`, or a `Content-Security-Policy: frame-ancestors` directive scoped to the specific allowed origin.

**Effect on other surfaces:** purely additive response headers — no routing, no auth, no behavior change to any request that isn't literally reading its own response headers. Safe for `/machine/`, `/overview/`, the telemetry API, and Grafana (Grafana already sends its own equivalent headers on Grafana-served paths; nginx's headers would apply on top for the same effect, no conflict). No effect on MQTT (unrelated protocol).

**Safest minimal production change:** the header additions above; can be deployed together with Items 1–3 in the same `nginx.conf` edit and single reload.

**Rollback method:** remove the added `add_header` lines, reload.

**Verification commands:**
```
# Before
curl -sk -D - -o /dev/null https://dash.promlogix.com/overview/ | grep -iE 'x-content-type|x-frame|strict-transport'
# (expected: no output)

# After
curl -sk -D - -o /dev/null https://dash.promlogix.com/overview/ | grep -iE 'x-content-type|x-frame|strict-transport'
# (expected: all three present)
```

**Downtime expected:** none (reload only).

---

## 5. LOW/MEDIUM — missing rate limiting

**Currently responsible:** no `limit_req_zone`/`limit_req` exists anywhere in `nginx.conf`; the Python API implements none either.

**Current public URL(s):** every route under `/api/` on `dash.promlogix.com`, and the entire `influx.promlogix.com` proxy, can be queried at unlimited frequency.

**Exact proposed restriction:**
```diff
 http {
   include mime.types;
   default_type application/octet-stream;
   map $http_upgrade $connection_upgrade { ... }
+  limit_req_zone $binary_remote_addr zone=telemetry_rl:10m rate=10r/s;
+  limit_req_status 429;
   ...
   location /api/ {
+    limit_req zone=telemetry_rl burst=20 nodelay;
     proxy_pass http://iot-stack-api-1:8088/api/;
     ...
   }
```
And, in the `influx.promlogix.com` server block:
```diff
   location / {
+    auth_basic ...              (Item 1, already applied)
+    limit_req zone=telemetry_rl burst=20 nodelay;
     proxy_pass http://iot-stack-influxdb-1:8086;
     ...
   }
```
Rate chosen (10 req/s per client IP, burst 20) is a conservative starting point sized to comfortably absorb a normal dashboard page load (which fires a handful of API calls at once) while throttling scripted scraping or a minor DoS attempt. `limit_req_status 429` makes throttled responses a standard `429 Too Many Requests` rather than nginx's default `503`, so a legitimate client can distinguish "rate limited" from "server down."

**Scope note:** the audit flagged this against "`/api/*`, static dashboard paths" together, but the practical risk is concentrated in the API/database proxy, not static HTML files. This plan scopes the minimal fix to `/api/` and the `influx.promlogix.com` proxy only, leaving the static `/machine/`, `/overview/`, `/plant/`, `/machines/`, `/shared/` locations unthrottled — consistent with the instruction not to touch the customer-facing Dashboard surface without specific evidence requiring it. Extending `limit_req` to the static locations later is a one-line addition per location if ever needed.

**Effect on other surfaces:**
| Surface | Affected? |
|---|---|
| `/machine/`, `/overview/`, `/plant/`, `/machines/`, `/shared/` | No — intentionally left unthrottled |
| Public telemetry API | Throttled above 10 req/s sustained per IP — normal dashboard usage is well under this |
| Grafana dashboard | No |
| MQTT | No — this is an nginx/HTTP-layer control, unrelated to the MQTT listener |

**Safest minimal production change:** the `limit_req_zone` (one line, `http{}` level) plus two `limit_req` lines in the two proxy locations named above.

**Rollback method:** remove the three added lines, reload. `limit_req_zone` at the `http{}` level with no matching `limit_req` anywhere is inert, so a partial rollback (removing only the per-location lines) is also safe if ever needed for quick triage.

**Verification commands:**
```
# Before: rapid-fire requests all succeed
for i in $(seq 1 30); do curl -sk -o /dev/null -w '%{http_code} ' https://dash.promlogix.com/api/overview; done; echo

# After: expect a mix of 200s (within burst) and 429s (once the burst is exceeded)
for i in $(seq 1 30); do curl -sk -o /dev/null -w '%{http_code} ' https://dash.promlogix.com/api/overview; done; echo
```
Run this verification burst sparingly and only from the operator's own machine — it is, by design, an artificial load test against a production endpoint.

**Downtime expected:** none (reload only). No legitimate dashboard user is expected to notice any behavior change at the chosen rate.

---

## 6. Credential exposure via `docker inspect` / `docker exec env`

This is not an nginx change — it is a Docker/secrets-management change to how two credentials are supplied to their containers:
- The Cloudflare Tunnel token, currently passed as a `tunnel run --token <value>` CLI argument (visible forever in `docker inspect iot-stack-tunnel-1`).
- The Grafana admin password, currently set via a `GF_SECURITY_ADMIN_PASSWORD` environment variable (visible via `docker exec iot-stack-grafana-1 env`).

**Exposure boundary, restated precisely:** neither value is reachable by an unauthenticated Internet user under any test performed in the audit. Both require pre-existing SSH/Docker access to the `iotprom` host, which is itself the actual security boundary here (gated by port 22 and whatever key-based auth is already in place — not reassessed in this plan, as SSH hardening was out of scope for the public-exposure audit).

**Rotate immediately, or schedule after perimeter remediation?**
**Recommendation: schedule after Items 1–5, not immediately** — with one explicit exception below. Reasoning: Items 1–3 are actively, anonymously exploitable by anyone on the Internet right now; this credential-visibility issue is not — it requires an attacker to already have host-level access, at which point far more than these two credentials would typically be at risk. Prioritizing the perimeter items first addresses the larger, more urgent exposure surface. Rotating these two credentials is real work (Grafana admin password rotation is trivial; the Cloudflare Tunnel token requires either a Cloudflare dashboard/API action to reissue and then a container recreation, which **does** cause a brief tunnel interruption) and is better scheduled deliberately than rushed.

**Exception — rotate immediately regardless of this schedule if:** there is any reason to believe SSH/host access has been shared beyond the current trusted operator set, or any suspicion of host compromise. No such evidence exists from this audit; this is a standing condition, not a finding.

**What "rotation" must include to actually fix the underlying issue** (not just reset the exposure clock): rotating the *value* alone leaves the new value equally visible via the same commands, because the *mechanism* (CLI arg / env var) is what's inspectable, not just the current value. A complete fix should also move both credentials off directly-inspectable mechanisms — e.g., a Docker secret, or an entrypoint script that reads the value from a file with restricted permissions at container start rather than receiving it via `-e`/CLI argument. This is a container-recreation-level change (not a config reload) and should be planned as its own scheduled maintenance window, separate from the zero-downtime nginx changes above.

**No value is reproduced anywhere in this document, the source audit, or any command shown above.**

---

## Summary — execution order recommendation

| Order | Item | Change type | Downtime | Blocking for Demo? |
|---|---|---|---|---|
| 1 | `influx.promlogix.com` auth | nginx config + reload | none | Yes — HIGH, actively exploitable |
| 2 | `/nr/` auth | nginx config + reload | none | Recommended before wider release |
| 3 | Grafana `/metrics` auth | nginx config + reload | none | Recommended before wider release |
| 4 | Security headers | nginx config + reload | none | Nice-to-have, low urgency |
| 5 | Rate limiting | nginx config + reload | none | Nice-to-have, low urgency |
| 6 | Credential rotation + mechanism change | Docker secret/env change, container recreation | brief tunnel interruption expected for the Cloudflare token step | Scheduled, not urgent (see exception above) |

Items 1–5 can all be applied in a single `nginx.conf` edit followed by one `nginx -s reload`, with zero expected downtime for any customer-facing surface. Item 6 is intentionally decoupled and scheduled separately.

---

*This document is a plan only. No file was modified, no service was reloaded or restarted, and no firewall or credential was changed while producing it. Not committed to git per instructions.*
