# LINE Verdict-Consistency Patch — Final Pre-Deployment Review
Date: 2026-09-15

Read-only review of the isolated staged artifact only. No production file touched, no deploy, no restart, no message sent.

Reviewed artifact:
```
docs/engineering/evidence/line_audit_20260915/staging/staged_flows_ISOLATED_COPY.json
SHA256: d10be3a0eb983bf84407b9d53e6c162249d0f58b407eaad4906cf46a2fd3a3d5
```
Re-confirmed against a **fresh** pull of the current live `flows.json` at review time:
```
Live SHA256:   f3fba6aaac72e03d2a675e0417ac29d4c4ebc35146890ae35d9472f1996d2665
(identical to every prior audit this engagement — production has not drifted since the patch was staged)
```

---

## 1. Before/after behavior of the two patched nodes

**`adf3dc5f003a516c` Health Logic:** all pre-existing behavior (drop conditions, `vibration_measure_status`/`vibration_measure_mms` computation, `alarm_code` derivation, status badge) verified unchanged. One purely additive block confirmed present exactly once:
```js
var motorCode = Number(p.motor_state);
p.alarm_verdict_live = (p.vibration_measure_status === 'OK') && (motorCode === 2);
```

**`e982d76b3ebe0b06` LINE Message Builder:** token/groupId lookup, snooze/backoff gating, Thai timestamp formatting, the `[Notify-Fix-1]` Velocity RMS guard, the `escalation` branch, and the final `msg.headers`/`msg.payload` construction all verified unchanged byte-for-byte in structure. The `alert` and `recovery` branches are the only text-generating logic modified, both now gated on the new `verdictLive` flag.

## 2. Final LINE wording, verified directly from the staged code (not from memory)

| Verdict | Header produced | Includes Motor State? |
|---|---|---|
| NORMAL (vibration OK, RUNNING) | `✅ กลับสู่ NORMAL` | Yes |
| WARNING (vibration OK, RUNNING) | `⚠️ แจ้งเตือน WARNING!` | Yes |
| CRITICAL (vibration OK, RUNNING) | `🚨 แจ้งเตือน CRITICAL!` | Yes |
| VIBRATION UNAVAILABLE (any alarm_level, `verdictLive=false`) | `[emoji] [alarm_level] (ยืนยันสภาพจริงไม่ได้ — ข้อมูล vibration ไม่พร้อมใช้งาน / unconfirmed — vibration data unavailable)` | Yes |
| STOPPED (`verdictLive=false` because `motor_code≠2`) | Same qualified wording as VIBRATION UNAVAILABLE (see the disclosed scope note below) | Yes, shows `STOPPED` |
| CRITICAL → automatic recovery → NORMAL | If vibration is confirmed OK at the recovery instant: `✅ กลับสู่ NORMAL`. If not: `⚠️ NORMAL (...unconfirmed...)` | Yes |

## 3. "Health" → "Motor State"

Confirmed directly: the string `"Health:"` and the `❤️` emoji no longer appear anywhere in the LINE Message Builder's code. `"Motor State"` appears exactly twice as literal output text — once in the `alert` branch, once in the `recovery` branch — both sourced from `motorStateText`, itself derived from `p.motor_state` via a 4-entry name table (`STOPPED/STARTING/RUNNING/STOPPING`). The `escalation` branch (unchanged) never referenced Health and still doesn't reference Motor State — consistent, no regression.

## 4. Can any wording path say "กลับสู่ NORMAL" when vibration is unavailable?

**No — verified structurally, not just by testing.** The literal string `'✅ กลับสู่ NORMAL'` occurs **exactly once** in the entire function, as the `true`-branch of a ternary gated on `verdictLive`:
```js
var recoveryHeader = verdictLive
    ? '✅ กลับสู่ NORMAL'
    : '⚠️ NORMAL (...unconfirmed — vibration data unavailable)';
```
`verdictLive` is `false` whenever `vibration_measure_status !== 'OK'` (i.e., unavailable). There is no other code path, string concatenation, or fallback anywhere in the function that can produce this string outside that one gated ternary — this is a structural guarantee, not merely a tested outcome.

## 5. Does RPM invalidity affect the vibration verdict?

**No — confirmed by absence.** The only occurrence of `rpm_valid` anywhere in either patched node's source is inside a code **comment** in Health Logic (`// rpm_valid plays no part in this formula...`); it does not appear in either node's *executable* code at all, in the `alarm_verdict_live` formula, in `velRms`, or anywhere else. `rpm_valid` cannot influence the LINE verdict or wording because nothing reads it.

## 6. Is the automatic recovery mechanism untouched?

**Confirmed, directly, at the byte level.** Re-fetched the current live flow and diffed these nodes against the staged copy independently for this review:
```
ef04bfbbd91995e0  Unified State Engine        — byte-identical
f9378e1435e2c70e  Escalation Timer            — byte-identical
c7fcf5f3a2ec18c2  Auto Resume Check           — byte-identical
f63614552d4d191e  429 Handler                 — byte-identical
```
The state-transition rules (Rule 1 dedup, Rule 4 recovery-once), the `machines` flow-context object shape, and every timer (`stateChangedAt`, 10-minute escalation gate, snooze, backoff) are unmodified. Only the *wording* the LINE Message Builder produces for a given verdict has changed — the decision of *when* CRITICAL→NORMAL fires is entirely unaffected.

## 7. Did any other node change?

**No — confirmed by a fresh, independent full-file structural comparison performed for this review** (not reused from the prior validation pass):
```
live node count: 59   staged node count: 59
ids only in live: []   ids only in staged: []
changed node ids: ['adf3dc5f003a516c', 'e982d76b3ebe0b06']
```

## 8. Are any newly introduced credentials or secrets present?

**No.** The credential-holding node (`f96b3d962be3abba`, "⚙️ Identity Extractor / Credentials") was diffed independently for this review and is **byte-identical** between live and staged — untouched. Both patched nodes' source was additionally scanned for any base64/token-shaped string (30+ contiguous alphanumeric/`+/=` characters): **zero matches in either.** The patch introduces no new secret material of any kind.

## 9. Pre-existing credential exposure — flagged, not a blocker, value not reproduced

The staged artifact is a **full copy of the entire production flow**, and therefore necessarily also contains the same hardcoded LINE channel access token, group ID, and channel secret already present, unmodified, in the live `flows.json`'s credentials-init node. This is a pre-existing condition (already documented in `LINE_NOTIFICATION_AUDIT.md`), **not introduced or worsened by this patch**. Operational handling note: this staged file must be handled with the same care as the production flow itself — not committed, not shared outside this controlled evidence directory. (It has not been committed, per instructions throughout this engagement.) No credential value is reproduced anywhere in this report.

---

## Outstanding, non-blocking observations (carried forward from the validation report)

- **STOPPED wording:** currently shares the same "unconfirmed" phrasing as RUNNING+UNAVAILABLE rather than a distinct third wording. This still satisfies the hard requirement (never says clean "NORMAL"), but is a known simplification, disclosed in both this review and the prior validation report.
- **Manual test-inject buttons** (`7ec3e726495f761b`, `05f64b8eeea910fa`) remain wired into the production ingestion path — unrelated to this patch, previously flagged, not addressed by this change (isolation was explicitly out of scope for implementation, per the approved proposal).
- **Auto Resume Check's stale `LINE_TOKEN`/`LINE_GROUP_ID`** flow-context keys (identified in the original notification audit) are untouched by this patch and remain a separate, pre-existing defect.

None of these bear on the correctness or safety of the two nodes actually patched here.

---

## Final Recommendation

```
READY FOR PRODUCTION
```

Basis: every one of the 9 required checks was independently re-verified against a fresh pull of the current live flow and the staged artifact at review time (not merely restated from the prior validation pass). The patch is minimal, additive, structurally provably correct for the two hard requirements (never "กลับสู่ NORMAL" without live vibration backing; RPM validity cannot affect the vibration verdict), leaves the automatic recovery mechanism and every other node byte-for-byte untouched, and introduces no new secrets. The disclosed non-blocking observations are pre-existing or explicitly out-of-scope items, not defects in the reviewed patch.

Deployment itself (activating this staged content in production) is a separate, not-yet-authorized action and was not performed as part of this review.

---
*Read-only final review. No production file modified, no deploy performed, no message sent, nothing restarted. Not committed to git per instructions.*
