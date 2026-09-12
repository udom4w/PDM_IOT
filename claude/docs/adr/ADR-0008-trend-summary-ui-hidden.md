# ADR-0008: Trend Summary UI — Intentionally Hidden, Data/Logic Retained

**Status:** Accepted
**Date:** 2026-09-12
**Supersedes:** none
**Related:** `claude/docs/releases/PHASE1_FREEZE_RECORD_2026-09-03.md` §2/§3/§5,
`claude/docs/adr/ADR-0007-legacy-trend-deprecated-unused.md`,
`claude/docs/engineering/PHASE1_DASHBOARD_UX_AUDIT.md` §8,
Machine Detail dashboard reconciliation (this session, read-only)

---

## Context

The Machine Detail dashboard (`/opt/iot-stack/frontend/`, VPS `iotprom`, not in
git) renders a "Trend Summary" 6-stat grid (`.trend-summary-grid`:
Mean/Min/Max/Std Dev/Samples/Slope, plus a direction arrow) beneath the Trend
chart, backed by `contract.py`'s `trend` block
(`mean_mms/min_mms/max_mms/stddev_mms/slope_mms_per_s/samples`, a fixed 60-second
window per `PHASE1_FREEZE_RECORD_2026-09-03.md` §2 — explicitly frozen there as
"available for display and context").

A read-only production reconciliation (this session) found that
`style.css` has carried `.trend-summary-grid { display: none; }` in production
since **2026-09-06 08:27:31 ICT** (`/opt/iot-stack/backups/trend_summary_hide_20260906_082731/`
is the pre-change snapshot). The rule was authored with the comment tag
`[trend-summary-hide]` and described itself as:

> "Temporary UI removal, presentation-only: app.js keeps writing
> #tMean/#tMax/#tSamples/#tSlope/#trendArrow/#trendDirection every poll
> (untouched), and the DOM nodes stay in the document -- only visually
> suppressed. No id, field, calculation or the Trend algorithm is removed."

No decision record ever formalized this change. It is not one of the five
scoped items in `TREND_V2_PRODUCTION_RELEASE_20260906.md` (a same-day release
record for an unrelated set of Trend chart changes), and it predates that
release. It is not mentioned in `ADR-0007`, `PHASE1_FREEZE_RECORD`, or
`PHASE1_DASHBOARD_UX_AUDIT`. It had stood, unreverted and still labeled
"Temporary," for approximately 6 days at the time of this reconciliation.

The reconciliation also confirmed, by direct inspection of the live
production `app.js`, that despite the CSS hide:

- `slope_mms_per_s` is still read every poll by `trendDirection()`.
- `risingPolls` / `RISING_POLLS_TO_NOTICE` are still incrementing every poll,
  unconditionally.
- The Phase-1 Attention priority-ladder rule "NORMAL + sustained rising
  trend" (`PHASE1_FREEZE_RECORD_2026-09-03.md` §5, priority-ladder item 6)
  can still fire in production — an operator can see this Attention line with
  no visible Trend Summary numbers on screen to check it against, since the
  box that would show them is hidden.

## Decision

**TREND SUMMARY 6-STAT UI = INTENTIONALLY HIDDEN on the product Dashboard.**
This is now a documented product decision, replacing the undocumented
"Temporary" label the CSS comment carried since 2026-09-06.

**Explicitly retained, unaffected by this decision:**

- The Trend Summary **data** (`contract.py`'s `trend` block: `mean_mms`,
  `min_mms`, `max_mms`, `stddev_mms`, `slope_mms_per_s`, `samples`,
  `slope_valid`) — unchanged, still served by the API on every request.
- `trendDirection()` — unchanged, still computed every poll in `app.js`.
- `risingPolls` / `RISING_POLLS_TO_NOTICE` — unchanged, still incrementing
  every poll.
- The Phase-1 Attention rule "NORMAL + sustained rising trend" — unchanged,
  still able to fire; `slope_mms_per_s` remains its live input.
- The markup (`#tMean`/`#tMin`/`#tMax`/`#tStddev`/`#tSlope`/`#tSamples`/
  `#trendArrow`/`#trendDirection`, `.trend-summary-grid`) — unchanged, stays
  in the DOM, still written every poll.
- `contract.py`, `trend.py`, the Influx `trend` measurement, and the
  firmware's new trend engine (M1B-1..M1B-5) — **no change of any kind**.
  Per `ADR-0007`, this engine is explicitly required infrastructure
  regardless of this UI-only decision.

**Changed by this decision:** only the wording of two CSS comments
(`style.css`, the `.trend-summary-grid { display: none; }` rule and its
paired `.chart-wrap` height-compensation comment) and one new explanatory
HTML comment (`index.html`, immediately before `.trend-summary-grid`), all
retagged `[trend-summary-hidden-by-design]`. The functional CSS rule itself
(`display: none`) is unchanged — the visual outcome in production is
identical to before this ADR.

## Consequences

- Future maintainers reading `style.css` or `index.html` will see this ADR
  referenced directly in the comment, instead of a dead-end "Temporary" label
  with no record behind it.
- `display: none` remains the correct technique: it fully removes the section
  from layout and the accessibility tree (appropriate for content meant to be
  genuinely absent, not merely de-emphasized), the 66px it frees is already
  deliberately reclaimed by `.chart-wrap`'s height (no dead gap), and no
  interactive elements live inside `.trend-summary-grid` (plain
  `<div>`/`<span>` text only), so `display:none`'s usual side effects (no
  focus, no pointer events) have nothing to break. `app.js`'s `.textContent`
  writes to these elements succeed identically whether or not the container
  is hidden, and nothing in `app.js` reads their layout/geometry back — the
  CSS hide and the JS logic are fully decoupled.
- **Re-enabling the Trend Summary UI in the future is a UI-only decision.**
  It requires reverting the two `display: none`/`margin-top` CSS lines in
  `style.css` (see the tagged comments) and nothing else — no backend, API,
  Influx, trend-engine, or firmware change is needed, because none of those
  layers were ever touched to hide it in the first place.
- **No change was made to `contract.py`, `trend.py`, any Influx measurement,
  or firmware** as part of this decision. This ADR is a frontend
  presentation-only record.

---

*Read-only reconciliation preceded this decision; the sign-off above applies
only to the two `style.css` comment edits and the one `index.html` comment
addition described here. No production file was modified to produce this
document. No firmware, COM5, or ATDIAG capture activity was touched at any
point in the investigation or this write-up.*
