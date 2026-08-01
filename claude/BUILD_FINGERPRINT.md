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

**What was added to bridge this:**

1. **`build_info.h`** — a small header, checked into the sketch folder,
   `#define`-ing `GIT_COMMIT_HASH`. Its checked-in default is `"UNKNOWN"`.
2. **`generate_build_info.ps1`** — a standalone PowerShell script (matching
   this project's existing convention of `capture_*.ps1` helper scripts
   already present in this sketch folder) that resolves the hash and dirty
   state and overwrites `build_info.h`. Fails fast (non-zero exit) on any
   real error — an unresolvable git repo is not an error (that's the
   legitimate `"UNKNOWN"` path); a header that can't be written is.
3. **`platform.local.txt`** (added `2026-08-01`, in the installed esp32 core,
   `Arduino15/packages/esp32/hardware/esp32/3.3.11/platform.local.txt` —
   *outside* this git repo) — wires step 2 into an automatic pre-build hook.

### [v16.5-autohook] Now implemented: automatic pre-build hook

Previously this was a manual step (see history below). It is now wired in as
`recipe.hooks.prebuild.9` in `platform.local.txt`:

```
recipe.hooks.prebuild.9.pattern.windows=cmd /c if exist "{build.source.path}\generate_build_info.ps1" powershell -NoProfile -ExecutionPolicy Bypass -File "{build.source.path}\generate_build_info.ps1"
recipe.hooks.prebuild.9.pattern=/usr/bin/env bash -c "[ ! -f '{build.source.path}/generate_build_info.ps1' ] || pwsh -NoProfile -ExecutionPolicy Bypass -File '{build.source.path}/generate_build_info.ps1'"
```

**How it works:**
- `{build.source.path}` is an Arduino build property resolving to the sketch
  folder actually being compiled (the same property already used by this
  core's own `recipe.hooks.prebuild.3/4/5` for `partitions.csv`/
  `bootloader.bin`/`build_opt.h`) — so the hook always targets the right
  sketch, whichever one is open.
- Pre-build hooks run **before** sources are compiled/preprocessed, so
  `build_info.h` is always fresh by the time `#include "build_info.h"` is
  resolved.
- The `if exist` guard makes this a no-op for any other esp32 sketch on this
  machine that doesn't have a `generate_build_info.ps1` of its own — this
  file is scoped to the *core install*, not to this one project, so it
  must not affect unrelated sketches.
- If `generate_build_info.ps1` exits non-zero, the `cmd /c` wrapper's exit
  code is non-zero too, and arduino-builder/arduino-cli **aborts the whole
  compile** with `Error during build: exit status 1` — verified by
  temporarily making `build_info.h` read-only and confirming the build
  failed, then confirming it succeeded again after clearing it.
- Applies identically to Arduino IDE and `arduino-cli`: both resolve boards
  from the same `Arduino15` data directory and therefore load the same
  `platform.local.txt` — no per-tool configuration needed.

**Requirements satisfied:**
1. `BUILD_ID`/`Git Commit`/`Build Date`/`Build Time` always reflect the
   source actually being compiled — `Build Date`/`Build Time` always did
   (from `__DATE__`/`__TIME__`, compiler-supplied); `Git Commit`/`BUILD_ID`
   now do too, since `build_info.h` is regenerated on every compile.
2. Dirty working trees are still detected automatically (`git status
   --porcelain` against the whole repo, unchanged from before).
3. The build fails if `build_info.h` cannot be regenerated (fail-fast
   contract in `generate_build_info.ps1`, verified above).
4. No manual step before compiling, from either Arduino IDE or
   `arduino-cli`.
5. This section documents the pipeline.
6. Compatible with both Arduino IDE and `arduino-cli` (same underlying core
   install, same hook).

**Caveat — not committed to this repo:** `platform.local.txt` lives inside
the Boards Manager package directory, not this git repo. If the esp32 core
is ever updated via Boards Manager (e.g. `3.3.11` → `3.3.12`), a new version
directory is installed without this file, and the hook must be re-added at:
`Arduino15/packages/esp32/hardware/esp32/<new-version>/platform.local.txt`
using the same two `recipe.hooks.prebuild.9.*` lines above (confirm the slot
number is still free in that version's `platform.txt` first).

### History: original manual-step design (superseded 2026-08-01)

Originally this was a manual step: the developer (or a CI pipeline) had to
run `generate_build_info.ps1` **before** compiling for `GIT_COMMIT_HASH` to
reflect the commit actually being built; if skipped, whatever was last
generated silently remained. This is exactly what caused a real incident:
the script was run once against commit `6160c05` on `2026-07-27` and never
again, so every build afterward — across 40 further commits — kept
reporting `Git Commit: 6160c05-dirty` on the serial banner even though the
actual compiled source was current. The automatic hook above closes that
gap.

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
