# `[v16.5g]` SR-Verify — Disposition Record

**Classification:** ARCHIVED EXPERIMENTAL EVIDENCE — NOT FOR PRODUCTION
**Date:** 2026-08-27
**Related:** `docs/releases/COMBINED_PRODUCTION_v16_5_S21_CURRENT_NTP_RELEASE_REPORT.md`

---

## What this is

A read-only SR-provenance re-verification block (backoff-based retry of `AT` register `REG_SAMPLE_RATE`, firing only while `g_sensorSrHzVerified == 0`). It was discovered during reconciliation of the Combined Production Candidate — present in the dirty main production working copy and in today's NTP verification source, but never part of either the Aug-25 verified production checkpoint or any committed git history.

## Evidence — preserved, unmodified, hash-verified

| File | SHA256 |
|---|---|
| Full preserved source (`claude/backups/WTVB02_v16_5.ino_PRESERVED_SRVERIFY_EVIDENCE_20260827_022228Z`) | `06e4d9e91034459ab0ed0b2b1e342109a7f437ba4965354cd58e18746d877b4d` |
| Extracted SR-verify-only excerpt (`claude/backups/SRVERIFY_v16_5g_EXTRACTED_EVIDENCE_20260827_022228Z.txt`) | `d502b7352778566913f710e9f0ce5d451de5d8abc9330a983530e07521ba00bc` |

Both re-verified unchanged immediately before writing this record.

**Exact location within the preserved source:**
- Declaration/state block: lines 490–501 (12 lines)
- Re-verification logic block (`taskModbusRead()`): lines 6421–6476 (56 lines)
- **Total SR-verify content: 68 lines**

## Why it was excluded from the Combined Production Candidate

The Combined Production Candidate was built to reconstruct exactly the last verified production baseline (Aug-25 checkpoint: S21 thresholds + current-freshness) plus the separately-verified NTP `[v16.5j]` retry-scheduler fix — nothing else. `[v16.5g]` was never part of either verified line of work: it wasn't in the Aug-25 checkpoint, and it wasn't the subject of today's NTP hardware verification. It rode along undisclosed in today's NTP worktree source purely as an artifact of that worktree's starting point, not as a requested or tested feature. Per explicit instruction during reconciliation, no feature beyond the two named fixes was to be carried into the combined source, so this block was identified and removed before build.

## Status

- **NOT part of the current Production firmware.** Confirmed absent from: Git HEAD, the Aug-25 verified checkpoint, and the Combined Production Candidate (source SHA256 `827137c2bac8d1455e698f20f3c3f6e9c9684772342b82061d9096ade8e4f338`, binary SHA256 `62c812d948fc18f217dd2d97a13a452963b4a50cb7846d4800a5aec4cf05dfd0`, currently flashed).
- **Must NOT be used as a build or flash source.** The preserved file (`WTVB02_v16_5.ino_PRESERVED_SRVERIFY_EVIDENCE_20260827_022228Z`) is a point-in-time evidence snapshot of a dirty working copy, not a designated production or candidate source.
- **The preserved copy is evidence only** — retained so the block's provenance and content are not lost, in case a future controlled feature branch wants to evaluate it. It has not been hardware-tested in isolation (its gate `srHz==0` never fired during this session's runtime captures, since the sensor's sample rate was already verified at boot in every test run).
- Future disposition (permanent discard, promotion to a controlled feature branch, or continued archival) remains an open decision, not resolved by this record.

## Related — `[v16.5i]` variant

`[v16.5i]` (`currentFilteredA`/`currentEvidenceValid` publish-site variant, discovered as an undisclosed fourth divergence in today's NTP worktree source during reconciliation) is classified as:

**DISCARDED / NON-PRODUCTION VARIANT**

It was never present in the production working copy, the Aug-25 checkpoint, or Git HEAD — only in today's now-superseded NTP verification worktree, where it was explicitly discarded (not merged) when constructing the Combined Production Candidate, in favor of the checkpoint's `current_age_s`-based freshness implementation. No preserved evidence copy exists for `[v16.5i]` specifically; its content remains recoverable only from the isolated NTP worktree (`PDM_IOT_worktree_patch1_isolated`) should it ever be needed.
