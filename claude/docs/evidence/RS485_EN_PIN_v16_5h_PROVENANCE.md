# `[v16.5h]` RS485 EN-Pin Fix — Provenance Record

**Classification: `[v16.5h]` = PRODUCTION / CLOSED**
**Date recorded:** 2026-08-27
**Related:** `docs/releases/COMBINED_PRODUCTION_v16_5_S21_CURRENT_NTP_RELEASE_REPORT.md`, `docs/evidence/SRVERIFY_v16_5g_DISPOSITION.md`

---

## 1. Exact source location and code scope

Production `.ino`, `taskModbusRead()`, quick-reconfig branch (`sensorWasRestarted == false` path):

```
Lines 6504–6511 (production HEAD, commit c55853a) — explanatory comment
Lines 6512–6515 — the fix itself:
    rs485Enable("QUICK-RECONFIG");
    vTaskDelay(pdMS_TO_TICKS(5));
    bool reOk = reconfigSensorAfterRestart(false);
    rs485Disable("QUICK-RECONFIG");
```

Total scope: 12 lines (comment + 4-statement bracket).

## 2. Root cause

`rs485Disable("NORMAL-POLL")` had already executed earlier in the same branch by the time the quick-reconfig path is reached, so every Modbus transaction inside `reconfigSensorAfterRestart()` was executing with `RS485_EN_PIN` de-asserted — the actual cause of the `err=226` / `ku8MBResponseTimedOut` failures that an earlier comment (`[v16.3w]`) had misattributed to a noise-stuck sensor. The fix brackets the call with `rs485Enable("QUICK-RECONFIG")` / `rs485Disable("QUICK-RECONFIG")`, matching the pattern already used correctly elsewhere in the same file for the STUCK-RESTART path (around `restartSensorViaModbus()`), including its 5ms settle delay.

## 3. Original validation evidence

Source: `RELEASE_RECORD_pump01_b603c2f_P1_P2_P3.md` (untracked, found in `PDM_IOT_worktree_patch1_isolated`, dated 2026-08-26; validation window 2026-08-26T08:47:05Z–09:07:36Z, predating this session's own work).

There, `[v16.5h]` is "Patch 2 — QUICK-RECONFIG RS485 EN-pin fix," recorded as:

| Check | Result |
|---|---|
| Build/source integrity | VALIDATED — isolated, hash-proven, 1 hunk / 11 lines |
| Normal-operation stability | VALIDATED — 0 interference with normal polling across all observation windows |
| Quick-reconfig recovery (the actual fault path) | **NOT TESTED** — no Modbus error burst was intentionally induced during that session |

That record also validated Patch 1 (`[v16.5g]`, since archived per `SRVERIFY_v16_5g_DISPOSITION.md`) and Patch 3 (`[v16.5i]`, since discarded, per the same disposition record) alongside Patch 2, as a combined `b603c2f + P1 + P2 + P3` build.

## 4. Original Patch-2 source/binary hashes

The release record documents cryptographic proof of the **combined P1+P2+P3** state, not Patch 2 in isolation:

```
Combined (b603c2f + P1 + P2 + P3) source SHA256:  e2a9e4350b96d2ed98686b57ceb45a0a50d05b1522b24fe51a3a5cc35de700e1
Combined (b603c2f + P1 + P2 + P3) binary SHA256:   c81f0e4b5341d64b686d815ee7227add2d9b3dfa445b0749a43c7885a2b274fd
Binary size: 681,248 bytes
```
The record states each patch's incremental hash was verified by mechanical removal (`b603c2f+P1 → +P2 → +P3`, each step reproducing the prior recorded SHA256 byte-for-byte), but **does not print the intermediate `b603c2f+P1` or `b603c2f+P1+P2` hash values themselves** — only the final combined figures above are recorded in that document. No Patch-2-alone hash is documented anywhere. This binary (`c81f0e4b...`) is no longer present on disk — `build_out/` in that worktree was later overwritten by this session's own NTP-only build.

## 5. Presence confirmed

| Location | Contains `[v16.5h]`? |
|---|---|
| `PDM_IOT_worktree_patch1_isolated` (today's NTP source, SHA `cff2b61c...a1765`) | ✅ (line 6504 in that file's own numbering) |
| Combined Candidate (`WTVB02_v16_5_COMBINED_S21_CF_NTP_20260826.ino`, SHA `827137c2...e4f338`) | ✅ — line 6504, byte-identical block to production |
| Canonical production source at `c55853a` (SHA `827137c2...e4f338`) | ✅ — same line 6504, same content |

## 6. Absence confirmed

| Location | Contains `[v16.5h]`? |
|---|---|
| `b603c2f` (base) | ❌ — 0 occurrences |
| Aug-25 verified checkpoint (`WTVB02_v16_5.ino_PRODUCTION_CHECKPOINT_20260825_162729Z`) | ❌ — 0 occurrences |

## 7. First appearance in canonical git history

`c55853a2237b8d2b9e68c59829ef2bd2d9bc0922` — `fix(firmware): promote combined S21 current-freshness NTP production baseline`. This is the immediate child of `b603c2f` for this file (no other commit touched the `.ino` in between), so `c55853a` is unambiguously the first commit in git history containing `[v16.5h]`.

## 8. How it got there — precise, not assumed

Three distinct events, not to be conflated:

1. **Original Patch-2 validation** (earlier, 2026-08-26, before this session): `[v16.5h]` was written, built, flashed, and partially validated (normal-operation stability only — the actual fault path was never induced) in the isolated worktree, as one of three bundled patches, per §3 above. This work was never committed to git.
2. **Accidental/automatic merge inheritance** (this session, during the S21+CF+NTP reconciliation): the Combined Candidate was built via a 3-way `git merge-file` (base = `b603c2f`, ours = Aug-25 checkpoint, theirs = today's NTP worktree source). `[v16.5h]`'s hunk is a `base→worktree` diff that does not overlap any checkpoint hunk, so it merged in automatically alongside the intended NTP `[v16.5j]` hunks. **At no point during that reconciliation was `[v16.5h]` specifically identified, requested, or excluded** — unlike `[v16.5g]`/`[v16.5i]`, which were explicitly found (via a real merge conflict) and handled. `[v16.5h]` simply rode through cleanly, undetected, because nothing about its hunk location conflicted with anything else.
3. **Later production promotion** (this session, commit `c55853a`): the Combined Candidate — already containing `[v16.5h]` per event 2 — was copied byte-for-byte into the canonical production path and committed. **The commit message names only "S21 current-freshness NTP"; it does not mention `[v16.5h]` or the EN-pin fix, confirming its inclusion was not a deliberate decision made at commit time** — it was inherited unchanged from the Combined Candidate.

**Do not read `c55853a` as an intentional introduction of `[v16.5h]`.** It is real, beneficial, already-tested-for-normal-operation code that happened to travel along with the NTP fix without anyone flagging it — the first explicit identification of it as a distinct, named feature is this document.

## 9. Physically present in the currently flashed binary

**Yes.** The Combined Candidate source (containing `[v16.5h]` at the same line, byte-identical) was compiled with the production FQBN into binary SHA256 `62c812d948fc18f217dd2d97a13a452963b4a50cb7846d4800a5aec4cf05dfd0`, which was flashed to COM5 (app-partition-only, `0x10000`), esptool hash-verified, and runtime-verified for 130.1s with no panics/watchdog/reboots (`COMBINED_PRODUCTION_v16_5_S21_CURRENT_NTP_RELEASE_REPORT.md`). The full source→binary→flash chain fully supports this conclusion — it is not inferred, it is the same chain already documented for S21/current-freshness/NTP.

---

## 10. Disposition

**`[v16.5h]` = PRODUCTION / CLOSED**

**Why:**
- It is real, root-cause-diagnosed, beneficial code (fixes a genuine EN-pin/timeout defect on the sensor quick-reconfig path).
- It is already committed to git (`c55853a`) and already physically flashed and running on the device — there is no live decision left to make about whether to include it; it's already in.
- Its normal-operation-stability claim from the original validation record is consistent with this session's own 130.1s runtime capture (0 panics, 0 watchdog resets, continuous FIFO/MQTT/Trend health).

**Residual gap, not blocking "CLOSED":** the fault path itself — an actual quick-reconfig error burst — was never intentionally induced in either the original Patch-2 validation or this session's runtime tests. The fix's *presence and non-interference* are proven; its *effectiveness at the moment of the real fault* remains unproven by direct fault injection. This mirrors the same category of residual gap already accepted for `[v16.5g]` (dormant-but-present) and the NTP unsynced-retry cadence (documented limitation) elsewhere in this release's evidence — noted for completeness, not treated as a reason to withhold "CLOSED."
