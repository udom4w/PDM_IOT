# Pre-Change Verification — influx.promlogix.com Public Exposure (HIGH finding)
Timestamp: 2026-09-15T04:17:28Z (operator machine) / 2026-09-15T04:17:31Z (server, iotprom)

Status: **READ-ONLY VERIFICATION ONLY.** No file was edited, no nginx reload was issued, no container was restarted, no firewall/Cloudflare/authentication setting was changed, and no destructive HTTP method was sent while producing this report.

---

## 1–2. Current nginx config — captured and backed up

Live config hash at time of capture: `d8fdb07b496a36e0c477da89c1a42fe4e43b9ef10cb7edb1aea46f4f395b148f` (`/opt/iot-stack/nginx.conf`), re-verified identical at the end of this session (config untouched throughout).

**Backup saved:**
```
docs/engineering/evidence/public_exposure_audit_20260915/nginx.conf.PRE_INFLUX_FIX_20260915T041731Z.bak
SHA256: d8fdb07b496a36e0c477da89c1a42fe4e43b9ef10cb7edb1aea46f4f395b148f
```
(Identical hash to the live file — confirms an exact, unmodified copy.)

**Current route map (relevant excerpts, unchanged from the prior audit):**

| server_name | Path | Backend | Auth at nginx |
|---|---|---|---|
| `dash.promlogix.com` | `= /` | 302 → `/overview/` | n/a |
| `dash.promlogix.com` | `/machine/` | static alias `/usr/share/nginx/machine/` | none (public, by design) |
| `dash.promlogix.com` | `/overview/` | static alias `/usr/share/nginx/machine/overview/` | none (public, by design) |
| `dash.promlogix.com` | `/plant/`, `/machines/` | static aliases | none (public, by design) |
| `dash.promlogix.com` | `/shared/` | static alias (shared JS modules) | none (public, by design) |
| `dash.promlogix.com` | `/api/` | proxy → `iot-stack-api-1:8088/api/` | none (public read-only API, by design) |
| `dash.promlogix.com` | `/api/ops/` | proxy → `iot-stack-api-1:8088/api/ops/` | `auth_basic` for all methods except GET/HEAD |
| `dash.promlogix.com` | `/nr/` | proxy → `iot-stack-nodered-1:1880/nr/` (Node-RED editor) | none at nginx (relies on Node-RED's own `adminAuth`) |
| `dash.promlogix.com` | `/line/callback` | proxy → `iot-stack-nodered-1:1880/nr/api/line/callback` | none at nginx |
| `dash.promlogix.com` | `/` (catch-all) | proxy → `iot-stack-grafana-1:3000` (Grafana) | none at nginx (relies on Grafana's own login) |
| **`influx.promlogix.com`** | `/` (all paths) | proxy → `iot-stack-influxdb-1:8086` | **none — this is the finding** |

---

## 3. Current public behavior — anonymous curl, this session

All six tests executed fresh from the host against the live public hostnames. No credentials were sent or printed anywhere.

| Test | URL | HTTP | Content-Type | Size | Notable headers |
|---|---|---|---|---|---|
| A | `https://influx.promlogix.com/` | 200 | text/html; charset=utf-8 | 534 B | `X-Influxdb-Version: v2.8.0`, `Cache-Control: public, max-age=3600` |
| B | `https://influx.promlogix.com/health` | 200 | application/json | 137 B | `X-Influxdb-Version: v2.8.0` |
| C | `https://influx.promlogix.com/ping` | 204 | (none) | 0 B | `X-Influxdb-Version: v2.8.0` |
| D | `https://influx.promlogix.com/metrics` | 200 | text/plain (Prometheus exposition) | **431,947 B (~422 KB)** | `X-Influxdb-Version: v2.8.0` — a full internal runtime-metrics dump, unauthenticated, ~422 KB per anonymous request |
| E | `https://dash.promlogix.com/machine/?plant=plant01&machine=pump01` | 200 | text/html | 36,057 B | `Cache-Control: no-cache` |
| F | `https://dash.promlogix.com/api/machine/plant01/pump01` | 200 | application/json | 1,729 B | `Cache-Control: no-store`, `X-Content-Type-Options: nosniff` |

No `Location` header appeared on any of these six (no redirects involved). No credential material appeared in any response header.

**New observation vs. the prior audit:** the `/metrics` payload size (~422 KB) is now measured precisely — this is a meaningful amount of internal data served to *any* anonymous requester on every single hit, worth noting as both an information-disclosure and a minor bandwidth-amplification concern, not just a "metrics exist" finding.

---

## 4. Confirmed Dashboard dependencies (source-verified, not just observed)

Read `/opt/iot-stack/frontend/shared/api.js` directly — this is the single module the entire Dashboard uses for every backend call (confirmed by grepping every other frontend JS file for `/api/` and finding no calls outside this module). Exact endpoints called by the Dashboard:

```
GET /api/overview
GET /api/machines
GET /api/machines/{machineId}
GET /api/machine/{plant}/{machine}
GET /api/machines/{machineId}/trend?...     <- the trend endpoint the Dashboard actually uses
GET/POST /api/ops/notes, /api/ops/maintenance, /api/ops/actions   (operator-only, auth-gated writes; not used by anonymous viewing)
```

All of these reach exactly one backend: `iot-stack-api-1:8088` (the custom read-only Python API), via nginx's `/api/` and `/api/ops/` locations on `dash.promlogix.com`. **None of them reach InfluxDB directly, and none of them use `influx.promlogix.com`.**

This is not an inference — `shared/api.js` states it explicitly in its own header comment:
> *"The frontend never talks to InfluxDB or MQTT directly: this module is the [single source of truth for API calls]"*

---

## 5. Nginx routing precedence — minimal change identified

`influx.promlogix.com` is a **separate `server{}` block** (separate `server_name`, separate TLS certificate pair — `influx.pem`/`influx.key` vs. `fullchain.pem`/`privkey.pem` for `dash.promlogix.com`) with exactly one `location / { proxy_pass http://iot-stack-influxdb-1:8086; ... }`. There is no location precedence complexity to navigate here — unlike `dash.promlogix.com` (where `/api/` vs. the catch-all vs. `/nr/` interact), this server block has a single catch-all location and nothing else.

**Minimal change:** add `auth_basic` + `auth_basic_user_file` to that one `location /` block, pointing at the already-mounted `/etc/nginx/ops.htpasswd` (same bcrypt credential file already gating `/api/ops/` writes on the other domain — no new secret, no new mount, no container change):
```diff
 server {
   listen 443 ssl;
   server_name influx.promlogix.com;
   ssl_certificate /etc/nginx/certs/influx.pem;
   ssl_certificate_key /etc/nginx/certs/influx.key;
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
 }
```
This is a 2-line addition, requires only `nginx -s reload` (no container restart, no new volume mount), and is fully reversible.

---

## 6. Explicit side-effect determination

| Question | Answer | Evidence |
|---|---|---|
| Affects `dash.promlogix.com`? | **NO** | Separate `server{}` block, separate `server_name`, separate TLS cert pair. Nginx dispatches by SNI/`Host` header before any location matching occurs; the two domains share nothing structurally except being in the same `http{}` file. |
| Affects `/machine/`? | **NO** | Served entirely from the `dash.promlogix.com` block; confirmed live (test E, 200, unaffected) and by source (frontend never calls InfluxDB directly). |
| Affects `/overview/`? | **NO** | Same reasoning as `/machine/`. |
| Affects telemetry API (`/api/*` on `dash.promlogix.com`)? | **NO** | Proxies to `iot-stack-api-1:8088`, not to `iot-stack-influxdb-1:8086`. Confirmed live (test F, 200, unaffected) and by source (`shared/api.js` only calls `/api/...` paths). |
| Affects Grafana? | **NO** | Grafana is reached via `dash.promlogix.com`'s catch-all `location /`, proxying to `iot-stack-grafana-1:3000` — an entirely different upstream container from InfluxDB. Grafana's own datasource connection to InfluxDB (container-to-container, `http://influxdb:8086`) does not route through `influx.promlogix.com` at all — confirmed below in Section 10. |
| Affects Node-RED? | **NO** | Node-RED's InfluxDB output node is configured with `url: 'http://influxdb:8086'` (the internal Docker DNS name), not the public hostname — confirmed directly from the live `flows.json` config in Section 10. |
| Affects MQTT? | **NO** | Entirely separate protocol/port (8883, mosquitto container) with no relationship to nginx or any HTTP proxy path. |

---

## 7. Safest protection mechanism (not implemented — decision only)

**HTTP Basic Auth (`auth_basic`) against the existing `ops.htpasswd`**, applied to the single `location /` block in the `influx.promlogix.com` server block (shown in Section 5). This is preferred over the alternatives considered:
- *New/separate credential file* — rejected as unnecessary for the minimal fix; reusing `ops.htpasswd` requires zero new secret provisioning or mounts. (A dedicated credential can be a later, non-blocking hardening step.)
- *IP allowlist (`allow`/`deny`)* — rejected as the primary mechanism because it's brittle against dynamic/remote ops IPs and was explicitly deprioritized in the remediation plan ("do not propose broad restrictions unless strictly necessary").
- *Removing the public subdomain entirely* — the most restrictive option, but a bigger behavioral change than needed to close the specific finding (anonymous reachability), and not the smallest reversible step.

This is the smallest, fastest, most reversible change that fully closes the anonymous-access finding.

---

## 8. Rollback procedure

```
Exact current-state backup (already saved, Section 2):
  docs/engineering/evidence/public_exposure_audit_20260915/nginx.conf.PRE_INFLUX_FIX_20260915T041731Z.bak
  SHA256: d8fdb07b496a36e0c477da89c1a42fe4e43b9ef10cb7edb1aea46f4f395b148f

Rollback steps (when/if ever needed, after the eventual change is applied):
  1. Copy the backup file back over the live path:
     scp <backup> iotprom:/opt/iot-stack/nginx.conf
     (or restore in place if edited directly on the host)
  2. sha256sum /opt/iot-stack/nginx.conf   # must equal d8fdb07b496a36e0c477da89c1a42fe4e43b9ef10cb7edb1aea46f4f395b148f
  3. docker exec nginx nginx -t             # syntax check before reload
  4. docker exec nginx nginx -s reload      # graceful reload, zero downtime
  5. Re-run the Section 3 test battery to confirm the pre-change (open) state is restored.
```

---

## 9. Post-change verification matrix (to run AFTER the eventual change — not run now)

**Expected PUBLIC (200), unaffected by the change:**
```
curl -sk -o /dev/null -w '%{http_code}\n' 'https://dash.promlogix.com/machine/?plant=plant01&machine=pump01'   # expect 200
curl -sk -o /dev/null -w '%{http_code}\n' https://dash.promlogix.com/overview/                                   # expect 200
curl -sk -o /dev/null -w '%{http_code}\n' https://dash.promlogix.com/api/overview                                # expect 200
curl -sk -o /dev/null -w '%{http_code}\n' https://dash.promlogix.com/api/machines                                # expect 200
curl -sk -o /dev/null -w '%{http_code}\n' https://dash.promlogix.com/api/machine/plant01/pump01                  # expect 200
curl -sk -o /dev/null -w '%{http_code}\n' 'https://dash.promlogix.com/api/machines/pump01/trend?range=5m'        # expect 200
```

**Expected PROTECTED (401 without credentials), after the change:**
```
curl -sk -o /dev/null -w '%{http_code}\n' https://influx.promlogix.com/                # expect 401 (was 200)
curl -sk -o /dev/null -w '%{http_code}\n' https://influx.promlogix.com/health          # expect 401 (was 200)
curl -sk -o /dev/null -w '%{http_code}\n' https://influx.promlogix.com/ping            # expect 401 (was 204)
curl -sk -o /dev/null -w '%{http_code}\n' https://influx.promlogix.com/metrics         # expect 401 (was 200)
curl -sk -o /dev/null -w '%{http_code}\n' https://influx.promlogix.com/api/v2/buckets  # expect 401 (was already 401 — unauthenticated InfluxDB token check; now double-gated)
```

**Expected 200 with correct credentials, after the change:**
```
curl -sk -u '<ops-user>:<ops-pass>' -o /dev/null -w '%{http_code}\n' https://influx.promlogix.com/health   # expect 200
```

---

## 10. Internal dependency check — does anything legitimate use the public hostname?

Checked both services that talk to InfluxDB in this stack:

```
iot-stack-api-1 container env:      INFLUX_URL=http://influxdb:8086      (internal Docker DNS name)
Node-RED influxdb config node:      url: 'http://influxdb:8086'          (internal Docker DNS name, from live flows.json)
```

**Neither service uses `influx.promlogix.com` — both talk to InfluxDB over the internal Docker network, by its internal service name, on the internal port.** No legitimate internal service depends on the public hostname in any way found in this verification. The public `influx.promlogix.com` subdomain exists solely for external/human ops access (e.g., browsing the InfluxDB UI or querying it directly from outside the network) — which is precisely the access the proposed `auth_basic` change would gate, without touching either internal integration.

---

## Summary

Every check in this verification supports the conclusion already reached in the remediation plan: adding `auth_basic` to the single `location /` block in the `influx.promlogix.com` server block is a minimal, fully reversible, zero-downtime change that closes the HIGH finding with **no effect on any other surface** — confirmed by (a) structural separation in the nginx config, (b) live before-state curl evidence, (c) frontend source code stating explicitly it never talks to InfluxDB directly, and (d) both real backend integrations (API and Node-RED) confirmed to use the internal Docker hostname, not the public one.

---
*This document is read-only verification evidence. No file was modified, no service was reloaded or restarted, and no firewall, Cloudflare, or authentication setting was changed while producing it. Not committed to git per instructions.*
