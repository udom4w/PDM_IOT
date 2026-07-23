# BUILD_FINGERPRINT.md — Firmware Build Fingerprint

Status: Implemented, uncommitted. Awaiting review before any further changes.

---

## What this adds

Every firmware build now identifies itself once, at boot, with zero runtime
cost afterward:

```
================================================
PROMLOGIX PDM IIOT
Firmware      : v16.5.4
Git Commit    : 8208b95
Build Date    : Jul 23 2026
Build Time    : 16:42:07
Motor Source  : MOTOR_SRC_RPM
================================================
BUILD_ID: 16.5.4-8208b95-20260723-1642
```

(Values shown are illustrative of the format — the actual `Build Date`/`Build
Time`/`BUILD_ID` reflect whenever the `.ino` is actually compiled, and `Git
Commit` reflects whatever `generate_build_info.ps1` last resolved.)

This is purely additive: it does not replace, alter, or remove the existing
`ESP32-S3 VIBRATION MONITOR ...` banner already printed in `setup()` — both
now print, back to back.

## BUILD_ID

Single source of truth: `g_buildId[48]`, computed once by `buildBuildId()`
(called from `setup()`), read via `getBuildId()`. Any future feature — an MQTT
device-status field, a diagnostic serial command, a fault-latch record — should
call `getBuildId()` rather than re-deriving the version/hash/date/time itself.

**Format:** `<FW_VERSION>-<GIT_COMMIT_HASH>-<YYYYMMDD>-<HHMM>`
**Example:** `16.5.4-8208b95-20260723-1342`
**If Git info is unavailable:** `16.5.4-UNKNOWN-20260723-1342` — falls out
automatically from `GIT_COMMIT_HASH` being the literal string `"UNKNOWN"`, no
special-casing needed.

## Design

| Piece | Mechanism | Why |
|---|---|---|
| `FW_VERSION` | `#define FW_VERSION "16.5.4"` | Single named constant; one place to bump the version string going forward, instead of a literal buried in a `Serial.println`. |
| `GIT_COMMIT_HASH` | `#include "build_info.h"`, which `#define`s it | See "Build system changes" below — this is the one piece that needs a build-system-adjacent mechanism, since Arduino has no native git integration. |
| `Build Date` / `Build Time` | Compiler-native `__DATE__` / `__TIME__` | Zero build-system changes needed — every C/C++ compiler provides these; always genuine, cannot be faked, and requires no extra tooling. |
| `Motor Source` | Reads the existing `g_motorStateSource` global at boot | No new logic — just a display of a value the firmware already computes at startup (`#ifdef TEST_CURRENT_SOURCE` selects it). |
| `BUILD_ID` | `buildBuildId()`, called once in `setup()` | Parses `__DATE__`/`__TIME__` into `YYYYMMDD`/`HHMM` once, concatenates with `FW_VERSION`/`GIT_COMMIT_HASH` via `snprintf`, stores in a static buffer. Runs exactly once; never touched again. |

## Never fabricates a hash

If `build_info.h` is missing, wasn't regenerated, or `generate_build_info.ps1`
couldn't resolve a hash (git not installed, not a repository, command failed
for any reason) — `GIT_COMMIT_HASH` is the literal string `"UNKNOWN"`, both in
the banner and inside `BUILD_ID`. There is no code path that invents,
guesses, or falls back to a plausible-looking hash.

---

## Build system changes (explained separately, per requirement 7)

Arduino IDE / `arduino-cli` has **no native git-integration hook** — there is
no built-in mechanism to inject a commit hash into a `.ino` at compile time
the way, say, a PlatformIO `pre:script` or a Makefile rule could.

**What was added to bridge this**, without touching the actual build
configuration (no changes to `boards.txt`, no `boards.local.txt`, no
compiler flags, no `arduino-cli` invocation changes):

1. **`build_info.h`** — a small header, checked into the sketch folder,
   `#define`-ing `GIT_COMMIT_HASH`. Its checked-in default is `"UNKNOWN"`.
2. **`generate_build_info.ps1`** — a standalone PowerShell script (matching
   this project's existing convention of `capture_*.ps1` helper scripts
   already present in this sketch folder) that runs `git rev-parse --short=7
   HEAD` and overwrites `build_info.h` with the result, or `"UNKNOWN"` if
   that command fails for any reason.

**This is a manual step, not an automatic hook.** The developer (or a CI
pipeline, if one is ever added) must run `generate_build_info.ps1` **before**
compiling for `GIT_COMMIT_HASH` to reflect the commit actually being built.
If it is never run, the checked-in `"UNKNOWN"` default remains — the build
still succeeds and reports honestly, it just doesn't carry a hash.

**Known limitation, stated plainly:** `git rev-parse --short HEAD` reports the
last **commit**, not whether the working tree has uncommitted changes on top
of it. In this repository specifically — given everything surfaced in the
prior provenance audit — a hash of `8208b95` describes "built from a tree
whose last commit was `8208b95`," which today also carries a substantial
uncommitted diff on top (this session's v16.5.4 work, the build-fingerprint
feature itself, and the pre-existing `v16.5.3-rpmdiag1` changes). The hash is
real and not fabricated, but it does not by itself certify "no local
modifications." No dirty-tree suffix (e.g. `-dirty`) was added, since it
wasn't requested — flagging it here as a natural follow-up if precise
build-to-tree correspondence ever matters again (as it did in the last
audit).

**Not implemented, offered only as a future option, pending approval:**
wiring `generate_build_info.ps1` into an automatic pre-compile hook (e.g., a
wrapper script around `arduino-cli compile`, or a `boards.local.txt`
`recipe.hooks.prebuild` entry). Left out because it would touch the build
invocation itself, which is exactly the kind of "build-system change" this
document is flagging for explicit, separate review rather than doing
silently.

---

## Constraints honored

- **Zero runtime impact after boot**: `buildBuildId()` and both `Serial`
  print blocks run exactly once, inside `setup()`. No new FreeRTOS task, no
  periodic timer, nothing added to any hot path (Modbus, state machine,
  Analytics, MQTT publish loops are all untouched).
- **No application logic changed**: MQTT topics/schema, Motor State Machine,
  Analytics, FreeRTOS task set, and sensor processing are all byte-identical
  to before this change — verified by clean build with only additive hunks
  (see unified diff).
- **No commit made.** Working tree only, awaiting review.
