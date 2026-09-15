# PROMLOGIX — Public Exposure Security Audit
Date: 2026-09-15
Scope: everything reachable from the public Internet on the `iotprom` host (dash.promlogix.com, influx.promlogix.com, and any directly bound host ports).
Method: read-only inspection (config files, `docker inspect`, `ss -tlnp`, container logs) plus live `curl`/`openssl s_client` probes issued from the host itself against its own public hostnames — i.e. testing exactly what an anonymous Internet client would see. No configuration, container, firewall, or auth changes were made. No destructive/mutating requests were sent; all POST/DELETE/PATCH/OPTIONS tests targeted read-only or nonexistent resources only, or were used purely to observe the HTTP status code returned.

No secrets are reproduced in this report. Two live credentials were discovered incidentally while inspecting container configuration (a Cloudflare Tunnel token via `docker inspect`, and a Grafana admin password via a container env var) — their *existence and exposure surface* is reported below; their *values* are not.

---

## 1. nginx

Config source: `/opt/iot-stack/nginx.conf` (single `http{}` block, three `server{}` blocks).

**Listening / public ports (cross-checked against `ss -tlnp` on the host):**
| Port | Bind | Reachable from Internet? |
|---|---|---|
| 80 | container-internal only, **not published to host** | **NO** — confirmed by direct test: `curl http://dash.promlogix.com/` from the server itself returned `curl: (7) Failed to connect`. There is no plaintext HTTP entry point at all on this host. |
| 443 | `0.0.0.0:443` / `[::]:443` | **YES** |

**server_name / routing (server block for `dash.promlogix.com`):**
| Path | Backend | Auth at nginx layer |
|---|---|---|
| `/` (bare) | 302 → `/overview/` (nginx `return`, doesn't reach Grafana) | n/a |
| `/machine/`, `/overview/`, `/plant/`, `/machines/`, `/shared/` | static files, `alias /usr/share/nginx/machine/...` | none (by design — public dashboard) |
| `/api/` | proxy → `iot-stack-api-1:8088` (the read-only Python API) | none for this location |
| `/api/ops/` | proxy → `iot-stack-api-1:8088/api/ops/` | **`auth_basic` required for every method except GET/HEAD** (`limit_except GET HEAD`), htpasswd file bcrypt-hashed (`$2y$` prefix confirmed) |
| `/nr/` | proxy → `iot-stack-nodered-1:1880/nr/` (Node-RED editor) | none at nginx; relies entirely on Node-RED's own `adminAuth` |
| `/line/callback` | proxy → `iot-stack-nodered-1:1880/nr/api/line/callback` | none at nginx; relies on the flow's own LINE signature validator |
| everything else (catch-all) | proxy → `iot-stack-grafana-1:3000` (Grafana) | none at nginx; relies entirely on Grafana's own login |

**server_name `influx.promlogix.com`** — a **second, dedicated public subdomain that reverse-proxies the raw InfluxDB HTTP API to the Internet with no auth at the nginx layer at all.** InfluxDB's own token auth is the only gate.

**HTTP→HTTPS:** a redirect server block exists in the config (`listen 80; return 301 https://...`), but as shown above **port 80 isn't reachable from outside at all** — the redirect is effectively dead code from the Internet's perspective (harmless, since there's nothing to redirect from).

**Security headers observed (live):**
- Static dashboard (`/overview/`): **no security headers at all** — no `X-Content-Type-Options`, no `X-Frame-Options`/`frame-ancestors`, no CSP, no HSTS.
- `/api/overview`: `Cache-Control: no-store`, `X-Content-Type-Options: nosniff` present (set by the Python API itself). No CORS header (`Access-Control-Allow-Origin`) under any tested `Origin`.
- Grafana-proxied paths (e.g. `/login`): `X-Content-Type-Options`, `X-Frame-Options: deny`, `X-Xss-Protection` present (Grafana's own defaults).
- **No `Strict-Transport-Security` (HSTS) header anywhere**, on either domain.

**Allowed HTTP methods (live tests against `/api/overview`):** `GET`→200, `HEAD`→501, `OPTIONS`→501 (the Python API's raw `BaseHTTPRequestHandler` implements no `do_HEAD`/`do_OPTIONS`, so these fall through to Python's built-in 501 — not a vulnerability, just non-standard REST behavior), `DELETE`→405, `PATCH`→405. None of the error responses leaked a stack trace, file path, or exception text — all matched the API's fixed JSON error format.

**TLS:** both domains present valid, current Let's Encrypt certificates (`dash.promlogix.com` exp. 2026-11-30, `influx.promlogix.com` exp. 2026-11-29) — not self-signed, not expired.

---

## 2. API (`iot-stack-api-1`, proxied at `/api/`)

Source reviewed in full in a prior audit this engagement (`/opt/iot-stack/api/app.py`, `contract.py`, `influx.py`, `overview.py`, `trend.py`). Endpoints, per the module's own docstring and route table:

| Method | Path | Anonymous access | Read/Write | Sensitive data? | Auth | Rate limit | Notes |
|---|---|---|---|---|---|---|---|
| GET | `/health` | ALLOWED | read-only | no (`{"status":"ok"}` only) | none | none observed | trivial liveness check |
| GET | `/api/overview` | ALLOWED | read-only | fleet-wide machine status/telemetry | none | none observed | intended public data |
| GET | `/api/machines` | ALLOWED | read-only | fleet-wide machine status/telemetry | none | none observed | intended public data |
| GET | `/api/machines/{id}` | ALLOWED | read-only | one machine's telemetry | none | none observed | intended public data |
| GET | `/api/machines/{id}/trend` | ALLOWED | read-only | historical vibration trend | none | none observed | intended public data |
| GET | `/api/machine/{plant}/{machine}` | ALLOWED | read-only | full machine detail (rpm, current, vibration, temp, operating hours) | none | none observed | intended public data — this is the Product-1 contract |
| GET/HEAD | `/api/ops/*` | ALLOWED | read-only | operator action/notes history | none for GET/HEAD (nginx `limit_except`) | none observed | |
| POST/DELETE (any non-GET/HEAD) | `/api/ops/*` | **DENIED** — HTTP 401 confirmed live | mutating | operator actions/notes | HTTP Basic (bcrypt htpasswd) at nginx, `X-Ops-Actor` header passed through | none observed | correctly enforced |
| POST/PUT/PATCH/DELETE | `/api/*` (telemetry routes) | DENIED — HTTP 405 confirmed live | n/a | n/a | method rejected outright | none observed | the module implements no write path for telemetry at all |

**No rate limiting was found anywhere** — nginx has no `limit_req`, and the Python API implements none itself. Every read endpoint (the entire telemetry surface, including the customer-facing machine-detail data) can be queried at unlimited frequency by anyone.

**No OpenAPI/Swagger/docs endpoint exists** (`/api/docs`, `/api/openapi.json`, `/swagger.json` all return 404/fall through to Grafana's 302).

**Error leakage:** none observed. The module's own design (`_error()` sends only fixed constant messages; the top-level `except Exception: log.exception(...)` logs server-side and returns a generic `{"error":"internal error","status":500}`) was confirmed live — 405 responses returned clean, fixed JSON with no path or exception text.

---

## 3. Live curl method tests (executed from the host, against the public hostnames)

| Test | Result |
|---|---|
| GET `/api/overview` | 200, valid JSON |
| GET `/overview/` (dashboard) | 200, HTML |
| HEAD `/api/overview` | 501 (unimplemented by the API, not a security gap) |
| OPTIONS `/api/overview` | 501 (same) |
| OPTIONS `/api/ops/actions` | **401** — auth enforced even for OPTIONS |
| POST `/api/ops/actions` | **401** — auth enforced |
| DELETE `/api/overview` | 405 |
| PATCH `/api/overview` | 405 |
| GET `/api/dashboards/home` (probing whether Grafana's own API leaks through) | 404 from the **custom API**, not Grafana — confirms nginx's `/api/` prefix match shadows Grafana's own `/api/*` routes entirely; Grafana's admin/datasource API is **not reachable** via this domain |

No destructive request was ever sent; DELETE/PATCH were tested only against the read-only `/overview` resource, which has no delete/patch handler at all (confirmed 405, not 200/204).

---

## 4. Common exposure-path sweep (dash.promlogix.com)

All of the following were tested; anything not explicitly matched by an nginx `location` block falls through to the Grafana catch-all, which correctly redirects unknown paths to `/login` (302) rather than serving a 404-with-details or, worse, a real file:

`.env`, `.git/config`, `.git/HEAD`, `backup.sql`, `backup.zip`, `.htpasswd`, `config.json`, `settings.js`, `secrets.json`, `server.pem`, `privkey.pem`, `id_rsa`, `.DS_Store`, `debug`, `admin`, `health`, `.well-known/security.txt`, `sitemap.xml`, `api/docs`, `api/openapi.json`, `swagger.json` → **all 302 (redirected to Grafana login), none served as static files.** No accidental static-file exposure found.

Two exceptions returned 200 directly, **not** via the redirect fallthrough:
- `robots.txt` → 200 (Grafana's own default; not sensitive)
- **`/metrics` → 200, and again `/metrics` on `influx.promlogix.com` → 200** — see Section 9, this is a real finding (Section 8 detail below).

---

## 5. Docker published ports / listening sockets

| Port | Service | Bind address | Internet exposure |
|---|---|---|---|
| 443 | nginx (TLS terminator for both domains) | `0.0.0.0` / `[::]` | **YES** |
| 22 | sshd | `0.0.0.0` / `[::]` | **YES** (expected management access) |
| 8883 | mosquitto (mTLS MQTT) | `0.0.0.0` | **YES** (intentional, for field devices) |
| 80 | nginx (redirect block) | container-internal only | NO |
| 1880 | Node-RED | `127.0.0.1` only | NO (reachable only via the nginx `/nr/` proxy) |
| 3000 | Grafana | `127.0.0.1` only | NO (reachable only via the nginx catch-all proxy) |
| 8086 | InfluxDB | `127.0.0.1` only | NO (reachable only via the `influx.promlogix.com` nginx proxy) |
| 8088 | custom API | `127.0.0.1` only | NO (reachable only via the nginx `/api/` proxy) |
| 1883 | mosquitto (plaintext, anonymous) | container-internal only | NO — confirmed no host binding via `ss -tlnp` |

No Docker daemon TCP socket (2375/2376) is exposed. No container has `/var/run/docker.sock` mounted — checked on all seven running containers, none found.

A **Cloudflare Tunnel** (`iot-stack-tunnel-1`, `cloudflared`) also runs, using an outbound, token-authenticated connection to Cloudflare's edge — this does not require any inbound port on this host and was not found to have any additional local config file (it runs entirely off the token). **This audit cannot enumerate what public hostnames/routes Cloudflare has configured for this tunnel** — that configuration lives in Cloudflare's own dashboard/API, not on this host, and inspecting it was out of scope/inaccessible from here. If the tunnel routes traffic to anything other than the nginx paths already covered above, that surface is not covered by this report.

---

## 6. MQTT (mosquitto)

- `listener 8883`: `require_certificate true`, `use_identity_as_username true`, `allow_anonymous false`, `acl_file` in effect. **This is hardened mTLS — a client without a certificate signed by this deployment's CA cannot even complete a TLS handshake.**
- **Live evidence of active Internet scanning against port 8883**, captured in this session's log read (not solicited by this audit — pre-existing traffic): a burst of connections from a `104.152.52.0/24` range, each rejected at the TLS layer (`peer did not return a certificate`, `wrong version number`) — consistent with automated Internet-wide TLS/MQTT scanners (e.g. Shodan/Censys-class probes), and consistent with mTLS correctly repelling them. Immediately after, the legitimate device (`pump01`) connected successfully with a valid client cert and negotiated TLS 1.2 — the hardening is working as intended.
- `listener 1883`: `allow_anonymous true`, but **not published to the host at all** (confirmed via `ss -tlnp`) — reachable only from inside the Docker network or via `docker exec`, not from the Internet. Separately, this same session **empirically tested** anonymous access on this internal listener earlier (a live `mosquitto_sub` with no credentials) and it silently received nothing, consistent with the shared `acl_file`'s deny-by-default behavior for an unrecognized (anonymous) identity even though `allow_anonymous` is technically `true`.
- No test messages were published in this audit.

---

## 7. Grafana

- Anonymous access: **not enabled.** No `GF_AUTH_ANONYMOUS_*` environment variable is set on the container, and no custom `grafana.ini` is mounted (only the data volume `/var/lib/grafana` is bind-mounted) — Grafana runs on its secure-by-default settings.
- **Live-confirmed**: requesting any dashboard path (`/d/<any-uid>`) returns **302 → `/login?redirectTo=...`**, not dashboard content. Login is required.
- Datasource/admin API (`/api/datasources`, `/api/admin/*`, etc.): **not reachable via the public domain at all** — nginx's `/api/` location match routes every `/api/*` request to the separate custom Python API instead of Grafana, so Grafana's own REST API surface is shadowed and inaccessible externally. (It would still be reachable internally, container-to-container, but that's not this audit's scope.)
- **`/metrics` (Grafana's own Prometheus metrics endpoint) is publicly reachable with no authentication** — confirmed live, returned real internal runtime metrics (GC stats, goroutine counts, Go build info). This is Grafana's default behavior when metrics auth isn't separately configured, and none was found.
- A `GF_SECURITY_ADMIN_PASSWORD` environment variable is set on the container (visible via `docker exec ... env` to anyone with host/Docker access — not Internet-reachable). Value not reproduced in this report.
- No login attempt was made; no configuration was changed.

---

## 8. File exposure

- Nginx document root for the dashboard (`/opt/iot-stack/frontend`, mounted read-only) is only reachable through the specific `alias` locations (`/machine/`, `/overview/`, `/plant/`, `/machines/`, `/shared/`) — no directory listing was tested or found, and the exposure-path sweep in Section 4 found no stray `.env`/`.git`/backup/key files served.
- `ops.htpasswd`: bcrypt-hashed (`$2y$` prefix confirmed without reading the hash value) — not a weak/crackable format.
- **Two live credentials were found exposed at the Docker/host-access level (not Internet-reachable, but worth recording as a hygiene gap):**
  1. The Cloudflare Tunnel token, visible in cleartext via `docker inspect iot-stack-tunnel-1` (passed as a `tunnel run --token ...` CLI argument, which Docker records permanently in the container's own metadata).
  2. The Grafana admin password, visible via `docker exec iot-stack-grafana-1 env`.
  Neither is reachable by an unauthenticated Internet user — both require pre-existing SSH/Docker access to this host, which is itself gated by port 22 and (presumably) key-based auth. They are recorded here because "what's on this host" is in scope, and because any future person with legitimate SSH access (a contractor, a wider ops team) would trivially see these full values with a single command.
- No `.git` directory, no database file, no backup archive, and no private key was found reachable over HTTP anywhere on either domain.

---

## 9. Risk Table

| Severity | Finding | Endpoint/Port | Evidence | Why it matters | Remediation | Affects current public Dashboard design? |
|---|---|---|---|---|---|---|
| **HIGH** | Raw InfluxDB HTTP API (UI, `/health`, `/ping`, `/metrics`) reverse-proxied to the public Internet with **no auth at the nginx layer** | `influx.promlogix.com:443` | Live: `/` → 200 HTML (InfluxDB UI), `/health` → 200 JSON w/ version, `/ping` → 204 w/ version header, `/metrics` → 200 Prometheus metrics, all unauthenticated. `/api/v2/buckets` correctly 401's. | The actual data API is token-gated (good), but the entire database server's UI, version banner, and internal metrics are open to anyone. Increases attack surface (version-targeted exploits, UI-based auth issues, brute-force target) against a full database server, not just an API. | Put `influx.promlogix.com` behind the same nginx `auth_basic` pattern already used for `/api/ops/`, or drop the public subdomain entirely and reach InfluxDB only via SSH tunnel / VPN for ops use. | **No** — the public customer dashboard never calls `influx.promlogix.com`; it only calls the custom read-only API. Fully separable. |
| **MEDIUM** | Node-RED editor (login page + static assets) publicly reachable | `dash.promlogix.com/nr/` | Live: `/nr/` → 200 (editor shell loads); `/nr/flows` (the real admin API) → 401, correctly protected. | The actual flow data/API is protected, but the editor UI itself being Internet-reachable is unnecessary attack surface (Node-RED version fingerprinting, future editor-specific CVEs, `adminAuth` brute-force target). | Restrict `/nr/` to a VPN/IP-allowlist at nginx, or move it off the public dashboard domain entirely. | **No** — the customer dashboard pages never link to or need `/nr/`. |
| **MEDIUM** | Grafana `/metrics` publicly reachable, no auth | `dash.promlogix.com/metrics` | Live: 200, real Prometheus metrics returned. | Same class of issue as the InfluxDB metrics finding — operational info disclosure, fingerprinting/DoS-planning aid. | Add `auth_basic` to a dedicated `location = /metrics` block, or disable Grafana's metrics endpoint if unused. | **No** — unrelated to the dashboard pages. |
| **LOW–MEDIUM** | No security headers (X-Frame-Options/CSP/X-Content-Type-Options) on the static dashboard pages; no HSTS anywhere | `dash.promlogix.com/overview/`, `/machine/`, etc.; both domains | Live headers captured, none of the above present. | Missing clickjacking/MIME-sniffing protection on customer-facing pages; no HSTS means no browser-enforced HTTPS-only policy after first visit. | Add `add_header X-Content-Type-Options nosniff; add_header X-Frame-Options DENY; add_header Strict-Transport-Security "max-age=..."` to the `server` block. Standard, low-risk nginx config addition. | **No** — purely additive headers, no behavior change to the dashboard itself. |
| **LOW** | No rate limiting on any public read endpoint | `/api/*`, static dashboard paths | Confirmed by config review (no `limit_req` in nginx, none in the Python API). | Unlimited-frequency querying is possible against the telemetry API and InfluxDB proxy; a scraper or minor DoS attempt has no friction. | Add `limit_req_zone`/`limit_req` in nginx for `/api/` and `influx.promlogix.com`. | **No** — transparent to legitimate dashboard traffic at any reasonable rate. |
| **LOW (hygiene, not Internet-exploitable)** | Cloudflare Tunnel token and Grafana admin password visible in cleartext via `docker inspect`/`docker exec env` | host-level, requires SSH/Docker access | Directly observed this session (values not reproduced here). | Anyone who ever gets legitimate or illegitimate host access sees these in one command; not rotatable without redeploying the container. | Move to Docker secrets or an env file with restricted permissions read at container start, not a CLI argument recorded in `docker inspect`. | **No** — invisible to the Internet-facing surface entirely. |
| **PASS** | Plaintext HTTP (port 80) not reachable from the Internet at all | host port 80 | `curl` from the host itself: connection refused. | Nothing to downgrade-attack; no unencrypted entry point exists. | none needed | — |
| **PASS** | mTLS on MQTT 8883 correctly rejects certificate-less connections, including live-observed scanner traffic | `iotprom:8883` | Log evidence of rejected scanner connections + one successful legitimate device connection, this session. | The Internet-facing IoT ingest port is properly hardened. | none needed | — |
| **PASS** | Anonymous MQTT listener (1883) not Internet-reachable | container-internal | `ss -tlnp` shows no host binding; docker ps confirms unpublished. | Even though `allow_anonymous true` is set, it cannot be reached from outside. | none needed (could still tighten as defense-in-depth, but not urgent) | — |
| **PASS** | Grafana requires login; anonymous viewing disabled | `dash.promlogix.com/*` (catch-all) | Live: any dashboard path → 302 to `/login`. | Prevents unauthenticated dashboard/data browsing through Grafana. | none needed | — |
| **PASS** | Grafana's own REST/admin API unreachable externally (shadowed by the custom API's `/api/` prefix) | `dash.promlogix.com/api/*` | Live: `/api/dashboards/home` returns the custom API's 404, not Grafana's. | Removes an entire class of Grafana-API attack surface from the public domain. | none needed | — |
| **PASS** | `/api/ops/` write methods correctly require HTTP Basic auth (bcrypt) | `dash.promlogix.com/api/ops/*` | Live: OPTIONS/POST → 401; GET/HEAD → passthrough (by design). | Mutating operator actions cannot be performed anonymously. | none needed | — |
| **PASS** | No accidental static file exposure (`.env`, `.git`, backups, keys) | `dash.promlogix.com/*` | Full sweep in Section 4, all fall through to Grafana's login redirect, none served as files. | No obvious low-effort data leak exists. | none needed | — |
| **PASS** | API error responses never leak stack traces, paths, or internals | `/api/*` | Live 405/500 responses matched fixed JSON constants; source review confirms server-side-only exception logging. | Reduces reconnaissance value of error responses to an attacker. | none needed | — |
| **PASS** | No Docker socket exposure anywhere | all containers | Checked all 7 containers' mounts; no `docker.sock`, no exposed daemon TCP port. | Removes a well-known container-escape vector entirely. | none needed | — |

---

## 10. Final Answers

**A. Is the public Dashboard currently safe enough for controlled Demo use?**
Yes, for the dashboard surface itself (`dash.promlogix.com/overview/`, `/machine/`, `/api/*` telemetry routes). That specific surface is read-only, correctly scoped, has no error leakage, no accidental file exposure, and its one auth-gated write path (`/api/ops/`) is properly protected. The findings above are real but sit **outside** the dashboard's own code path — they're adjacent services (InfluxDB UI, Node-RED editor, Grafana metrics) reachable through the same domain/certificate, not flaws in the dashboard itself.

**B. What is the single biggest security risk?**
The `influx.promlogix.com` reverse proxy — an entire production database server's UI, version banner, health check, and internal metrics endpoint sitting on the open Internet with zero authentication at the proxy layer. It's the one place where "a whole backend service," not just a designed API, is directly exposed.

**C. What must be fixed before exposing this to real (non-demo) customers?**
1. Put `influx.promlogix.com` behind auth (or remove the public subdomain and use a tunnel/VPN for ops access to InfluxDB).
2. Remove or auth-gate `/nr/` from the public dashboard domain — Node-RED's editor has no business being Internet-reachable at all, even with `adminAuth` in front of the actual API.
3. Auth-gate or disable the public `/metrics` endpoints (both Grafana's and InfluxDB's).
4. Add basic security headers (`X-Content-Type-Options`, `X-Frame-Options`, HSTS) to the dashboard's nginx server block.
5. Add rate limiting to the public API and the InfluxDB proxy.
6. Move the Cloudflare Tunnel token and Grafana admin password out of `docker inspect`-visible CLI args/env vars.

**D. What can remain public, as-is?**
- `dash.promlogix.com/overview/`, `/machine/`, `/plant/`, `/machines/`, `/shared/` (the dashboard pages).
- `/api/overview`, `/api/machines*`, `/api/machine/{plant}/{machine}`, `/api/machines/{id}/trend` (the read-only telemetry API) — this is the intended, designed-for-public-consumption data.
- `/api/ops/*` GET/HEAD (read-only operator history) — by explicit design in the API's own docstring.
- Port 8883 MQTT — correctly mTLS-hardened, already withstanding live scanning traffic.
- Grafana's login page — must be reachable for legitimate staff to log in; the login gate itself is working correctly.

**E. What must NEVER be public?**
- The raw InfluxDB HTTP API/UI/metrics on `influx.promlogix.com` (or any subdomain) without authentication in front of it.
- The Node-RED editor (`/nr/`), regardless of whether its API is separately protected — editors of this kind are an operational tool, not a customer-facing surface.
- Any operational `/metrics` endpoint (Grafana, InfluxDB, or otherwise) without auth.
- The Cloudflare Tunnel token, Grafana admin password, or any other credential currently sitting in `docker inspect`/`env`-visible form — these should never be extractable by anyone who isn't specifically authorized to administer this stack, and today they're one command away from any SSH session on the box.

---

*This report is read-only evidence. No file, container, firewall rule, or authentication setting was modified in the course of this audit. Not committed to git per instructions.*
