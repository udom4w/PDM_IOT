# BUILD_FINGERPRINT.md — Firmware Build Fingerprint

Status: Implemented on branch `feature/build-fingerprint`, based directly on
clean HEAD (`8208b95`). Uncommitted, awaiting review before staging.

---

## What this adds

Every firmware build now identifies itself once, at boot, with zero runtime
cost afterward:

```
================================================
PROMLOGIX PDM IIOT
Firmware      : v16.5
Git Commit    : 8208b95
Build Date    : Jul 23 2026
Build Time    : 16:42:07
Motor Source  : MOTOR_SRC_RPM
================================================
BUILD_ID: 16.5-8208b95-20260723-1642
```

(Values shown are illustrative of the format — the actual `Build Date`/`Build
Time`/`BUILD_ID` reflect whenever the `.ino` is actually compiled, and `Git
Commit` reflects whatever `generate_build_info.ps1` last resolved.)

This is purely additive: it does not replace, alter, or remove the existing
`ESP32-S3 VIBRATION MONITOR ...` banner already printed in `setup()` — both
now print, back to back. The only change to that pre-existing banner is
removing its own independent version-number substring (see below).

## BUILD_ID

Single source of truth: `g_buildId[56]`, computed once by `buildBuildId()`
(called from `setup()`), read via `getBuildId()`. Any future feature — an MQTT
device-status field, a diagnostic serial command, a fault-latch record — should
call `getBuildId()` rather than re-deriving the version/hash/date/time itself.

**Format:** `<FW_VERSION>-<GIT_COMMIT_HASH>-<BUILD_DATE>-<BUILD_TIME>`
**Example:** `16.5-8208b95-20260723-1342`
**Example (dirty tree):** `16.5-8208b95-dirty-20260723-1342`
**If Git info is unavailable:** `16.5-UNKNOWN-20260723-1342`

`BUILD_ID` is a pure **composition** of four independently-accessible,
boot-time-immutable fields — it is built by concatenating the four getters
below, never the other way around, so it can never disagree with them:

| Field | Accessor | Source |
|---|---|---|
| Firmware version | `getFwVersion()` | `#define FW_VERSION "16.5"` |
| Git commit hash | `getGitCommitHash()` | `#define GIT_COMMIT_HASH ...` (from `build_info.h`) |
| Build date | `getBuildDate()` | `g_buildDate[9]`, normalized `YYYYMMDD`, parsed from `__DATE__` once at boot |
| Build time | `getBuildTime()` | `g_buildTime[5]`, normalized `HHMM`, parsed from `__TIME__` once at boot |

Future MQTT/REST diagnostics should call whichever single getter they need —
none of them require parsing `BUILD_ID` apart to recover one piece.

## Never fabricates a hash

If `build_info.h` is missing, wasn't regenerated, or `generate_build_info.ps1`
couldn't resolve a hash (git not installed, not a repository, command failed
for any reason) — `GIT_COMMIT_HASH` is the literal string `"UNKNOWN"`, both in
the banner and inside `BUILD_ID`. There is no code path that invents,
guesses, or falls back to a plausible-looking hash.

## Dirty-tree detection

`generate_build_info.ps1` also runs `git status --porcelain` against the
whole repository (including untracked files, since Arduino compiles whatever
sits in the sketch folder regardless of git tracking) and appends `-dirty` to
the **real** resolved hash if the tree isn't clean. The hash itself is never
altered or invented — `-dirty` is only ever an appended flag on a genuine hash.

---

## Build system changes (explained separately)

Arduino IDE / `arduino-cli` has **no native git-integration hook** — there is
no built-in mechanism to inject a commit hash into a `.ino` at compile time.

**What was added to bridge this**, without touching the actual build
configuration (no changes to `boards.txt`, no `boards.local.txt`, no
compiler flags, no `arduino-cli` invocation changes):

1. **`build_info.h`** — a small header, checked into the sketch folder,
   `#define`-ing `GIT_COMMIT_HASH`. Its checked-in default is `"UNKNOWN"`.
2. **`generate_build_info.ps1`** — a standalone PowerShell script (matching
   this project's existing convention of `capture_*.ps1` helper scripts
   already present in this sketch folder) that resolves the hash and dirty
   state and overwrites `build_info.h`.

**This is a manual step, not an automatic hook.** The developer (or a CI
pipeline, if one is ever added) must run `generate_build_info.ps1` **before**
compiling for `GIT_COMMIT_HASH` to reflect the commit actually being built.
If it is never run, the checked-in `"UNKNOWN"` default remains — the build
still succeeds and reports honestly.

**Not implemented, offered only as a future option, pending approval:**
wiring `generate_build_info.ps1` into an automatic pre-compile hook. Left out
because it would touch the build invocation itself, which is exactly the
kind of "build-system change" this document flags for explicit, separate
review rather than doing silently.

---

## Constraints honored

- **Zero runtime impact after boot**: `buildBuildId()` and both `Serial`
  print blocks run exactly once, inside `setup()`. No new FreeRTOS task, no
  periodic timer, nothing added to any hot path (Modbus, state machine,
  Analytics, MQTT publish loops are all untouched).
- **No application logic changed**: MQTT topics/schema, Motor State Machine,
  Analytics, FreeRTOS task set, and sensor processing are all byte-identical
  to HEAD — this branch contains *only* the Build Fingerprint feature, based
  directly on clean HEAD (`8208b95`), with none of the separately-tracked
  v16.5.4 architectural work (RPM EMA invalidation, Atomic Telemetry
  Snapshot, RPM Evidence freshness) or any other in-progress changes.
- **No commit made.** Branch and worktree only, awaiting review.
