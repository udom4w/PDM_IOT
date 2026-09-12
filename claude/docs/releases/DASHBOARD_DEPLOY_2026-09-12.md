# DASHBOARD DEPLOY — 2026-09-12

**System:** PROMLOGIX Condition Monitoring — Machine Detail Dashboard
(`https://dash.promlogix.com/machine/`), plant `plant01` / machine `pump01`
(shared frontend code, applies to any machine served by it).
**Record type:** Production deployment record.
**Scope:** Frontend only. No backend, API, Node-RED, InfluxDB, or firmware
change is part of this release.

---

## 1. Deployment date/time

**2026-09-12, 13:31–13:38 ICT** (backup created 13:36:22; files overwritten
immediately after; post-deploy verification completed same session).

---

## 2. Exact deployed files

| File | Path on VPS |
|---|---|
| `app.js` | `/opt/iot-stack/frontend/app.js` |
| `index.html` | `/opt/iot-stack/frontend/index.html` |
| `style.css` | `/opt/iot-stack/frontend/style.css` |

No other file was deployed. `shared/api.js`, `shared/shell.js`,
`machines/*`, `overview/*`, `plant/*` were verified unchanged before and
after this deploy.

---

## 3. SHA256 AFTER (live production, verified post-deploy)

```
app.js      cdca3a807293af267e687c96919b1a5d956ff3151f09b340ebab2e16d7511d5f
index.html  fdcc3c79c5ab096d986c6e3b17671888f9e695b156a644da00d9481f3df564ae
style.css   4a332e9711b15e3fa64a525f46bfd2502020e4c92b12769a0ad2f5394af328b7
```

---

## 4. SHA256 BEFORE / rollback reference

```
app.js      5422030e3580452cf90316cb3c0c31830fe02949cf4cd2719f5dd30aadb4bb62
index.html  f9c2731a6a9e4dcb5ce08a4f18cc1861997b1812c5dd9041de180a56ae259d3a
style.css   2a368f2eea5e4150424887b31e944f09131eda35359abfdf29c83c6c356eb938
```

Rollback procedure: copy the three files from the backup path below back
over the live files in `/opt/iot-stack/frontend/`. No nginx/service restart
is required (static bind-mount, no build step — consistent with prior
deployment precedent recorded in `PHASE1_FREEZE_RECORD_2026-09-03.md` §10).

---

## 5. Backup path

```
/opt/iot-stack/backups/dashboard_deploy_20260912_133622/
```

Contains: `app.js`, `index.html`, `style.css` (pre-deploy copies) and
`SHA256_BEFORE.txt`. SHA256 values in that file were verified to match §4
above at backup-creation time, before any overwrite occurred.

---

## 6. Functional changes

1. **Offline motor state → `UNKNOWN`.** When `status.online` is `false`, the
   topbar device-motor chip and the Machine Status mini-grid motor field no
   longer display the (possibly stale) `motor_state` value as if current;
   they display `UNKNOWN`, with the last-known value surfaced only as a
   hover tooltip. The Attention card's offline line separately and
   explicitly states the last-known state by name (`"สถานะล่าสุดที่ทราบ (Last
   known): <state>"`) when one exists.
2. **Last-known state shown only when `last_seen` exists.** The Operating
   Condition card's `LAST KNOWN · HH:MM:SS` badge (Temperature/Current/RPM/
   Crest Factor's shared freshness indicator) is shown only when
   `!status.online && status.last_seen` is truthy. A machine that has never
   published any telemetry (`last_seen === null`) no longer shows the
   nonsensical `"LAST KNOWN · —"` — the badge stays hidden in that case.
3. **Trend stale → `UNAVAILABLE`.** The Trend headline pill no longer shows
   `VALID` off a trend row that is no longer fresh
   (`data_quality.trend.stale === true`). It shows `UNAVAILABLE` when the
   row was previously valid, or `NO DATA` when no trend data has ever
   existed for that machine.
4. **Trend direction suppressed when stale.** The trend direction
   arrow/label (`trendDirection()`'s output) is only computed and shown when
   the trend pill would show `VALID`; otherwise it renders `ไม่ทราบ` / `—`,
   so a stale slope can never be presented as a current directional claim.
5. **Trend Summary UI intentionally hidden.** The 6-stat grid
   (`.trend-summary-grid`: Mean/Min/Max/Std Dev/Samples/Slope) remains
   `display: none` in `style.css`, now explicitly documented as an
   intentional product decision (see §9) rather than an undocumented
   "Temporary" comment.
6. **Trend statistics/slope/rising-trend logic retained, unaffected.** The
   underlying data (`contract.py`'s `trend` block), `trendDirection()`,
   `risingPolls`/`RISING_POLLS_TO_NOTICE`, and the Phase-1 Attention
   priority-ladder rule "NORMAL + sustained rising trend" all continue to
   run every poll exactly as before — only the 6-stat grid's visibility and
   the one staleness gate in item 4 above changed.

---

## 7. Explicit non-changes

- **API contract unchanged** — `contract.py` verified byte-identical
  before and after this deploy.
- **InfluxDB unchanged** — no schema, measurement, retention, or data
  write of any kind.
- **Node-RED unchanged** — `flows.json` mtime confirmed unchanged
  (2026-09-07, predates this deploy).
- **Firmware unchanged** — not touched at any point.
- **`alarm_level` logic unchanged** — remains read-only in the frontend;
  no assignment path exists anywhere in the deployed diff.
- **Trend calculation unchanged** — `trend.py`'s aggregation windows,
  periods, and gap contract are untouched; only the frontend's *display*
  of trend freshness changed (§6.3–6.4).

---

## 8. Verification

| Check | Result |
|---|---|
| Public HTTP status | `index.html`/`app.js`/`style.css` all **HTTP 200** via `https://dash.promlogix.com/machine/` |
| Served bytes = deployed files | SHA256 of `curl`-fetched bytes matches §3 exactly, all 3 files |
| `node --check` | **PASS** on the live-served `app.js` |
| Docker/service restart | **None** — container uptimes identical before/after (`nginx` "Up 2 weeks", `iot-stack-api-1` "Up 5 days", `iot-stack-nodered-1` "Up 10 days", etc., unchanged) |
| Rollback backup verified | `SHA256_BEFORE.txt` in the backup path matches §4 exactly, confirmed at backup-creation time before the overwrite |

---

## 9. Reference

`claude/docs/adr/ADR-0008-trend-summary-ui-hidden.md` — the decision record
for item 6.5/6.6 above (Trend Summary UI hidden by design; data/logic
retained).

---

## 10. Status

> # DASHBOARD RELEASE = DEPLOYED / VERIFIED

---

*This record was produced after live deployment and post-deploy
verification. No firmware, COM5, or ATDIAG capture activity was touched at
any point in the deployment or in producing this document. This file itself
is local-repo documentation only — it has not been staged or committed.*
