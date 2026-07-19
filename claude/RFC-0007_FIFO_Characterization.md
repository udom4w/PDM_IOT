# RFC-0007: Characterization of FIFO Time-Domain Acceleration Data

**Status:** 🟡 Evidence-backed conclusions on protocol/content identity; DSP-stage
question remains open (see §4).
**Relationship to other documents:** RFC-0006 asked "what is in the FIFO?"
and designed experiments to distinguish raw acceleration from several
alternatives (filtered, velocity, internal DSP buffer, compressed,
calibration, metadata). This document reports the results of running those
experiments on real hardware, across two independent tools (ESP32 firmware,
WitMotion's own PC software), and narrows the remaining question to a single
precise one: **which DSP stage does this data represent** — not *whether*
it's real acceleration data (that part is now well-supported).

---

## 1. Protocol Summary (reference: RFC-0006)

Confirmed, CRC-validated facts carried forward from RFC-0006 and the
earlier protocol reverse-engineering work:

- Modbus RTU, CRC16 (poly 0xA001), device ID 0x50.
- Normal polling reads 107 contiguous registers (0x30–0x9A) in one
  transaction.
- The FIFO (register 0x2C) uses a separate, non-standard transaction shape:
  small 7-byte "progress" frames (Len byte = 0x01) and a 6149-byte "full
  dump" frame (Len byte = 0x00, containing 6144 bytes = 1024 samples × 3
  axes × 2 bytes).
- **New in this document:** the FIFO progress-frames were confirmed to
  arrive **autonomously** (approx. every 250ms) without the host re-sending
  a request — demonstrated with a single-request-then-listen-only test
  (4 consecutive CRC-valid progress frames received after exactly one
  request, timestamps ~250ms apart). The full dump itself did not always
  arrive autonomously within a short window in every test, so a hybrid
  capture strategy (passive listen, with a follow-up request only if the
  autonomous stream stalls) was implemented and used for the captures below.

```
Host                          Sensor
 │                               │
 ├── Read FIFO request ────────►│
 │                               │
 │◄──────── Progress (24.7%) ────┤
 │◄──────── Progress (49.3%) ────┤   (autonomous, ~250ms apart,
 │◄──────── Progress (74.0%) ────┤    no further requests needed)
 │◄──────── Progress (98.6%) ────┤
 │                               │
 │◄──────── Full Dump (6149B) ───┤
 │                               │
 ├── CRC check ── OK ✅          │
```

---

## 2. Evidence

### 2.1 CRC

Every frame type (poll, progress, full-dump) referenced in this document
passed CRC16 verification, computed independently in this project's own
tooling (not reused from any vendor library). This applies to:
- All ESP32-captured frames (`FIFO_idle4.txt`, `Hybrid_0.txt`, and others).
- The WitMotion PC software's own capture (`data_0.bin`, this session):
  12 poll frames, 4 progress frames, 1 full-dump frame, **all CRC-verified**.

### 2.2 1024 samples

Every successful full-dump capture decoded to exactly 1024 × 3 × 16-bit
samples, matching the datasheet's documented FIFO size and this project's
byte-shape arithmetic (6144 = 1024×3×2), with no exceptions across multiple
independent captures.

### 2.3 Gravity (static DC) test

| Capture | Tool | \|DC vector\| |
|---|---|---|
| `FIFO_idle4.txt` | ESP32 | 0.9963 g |
| `HYBRID_FIFO_90.txt` (rotated 90°) | ESP32 | 0.9952 g |
| `data_0.bin` (this session) | WitMotion PC software | 0.9959 g |

All three land within **0.11% of each other**, and within ~0.4% of the
physically-expected 1.000g. This is now supported by **three independent
capture sessions across two independent tools (custom ESP32 firmware and
WitMotion's own PC software), taken roughly an hour apart on the same test
day** — not a single measurement, and not a single tool. (Note: these
captures were NOT taken on different calendar days — an earlier draft of
this claim considered that framing, but upload timestamps confirm all
three fall within about an hour of each other on 2026-07-09. Repeating this
test on a different day remains a useful future check, not yet done.)

### 2.4 Rotation test

Rotating the sensor 90° about its Z axis:

| | Before | After | 
|---|---|---|
| \|Total\| | 0.9963 g | 0.9952 g (Δ 0.11%) |
| \|XY-plane\| | 0.9960 g | 0.9951 g (Δ 0.09%) |
| XY-plane angle | 72.2° | 157.4° (Δ 85.2°, physical rotation was 90°) |
| Z component | -0.0242 g | -0.0157 g (both near zero, consistent with Z being the rotation axis) |

Magnitude conservation under rotation is the single most physically
specific test available for "this is gravity" — a coincidental unrelated
DC offset would have no reason to preserve magnitude across an arbitrary
physical rotation. The ~5° angle discrepancy is attributed to imprecise
manual rotation (no fixture used), not a strike against the hypothesis.

### 2.5 25 Hz motor test — cross-tool reproduction

| Capture | Tool | FIFO-derived dominant freq (Y) | FIFO-derived dominant freq (Z) |
|---|---|---|---|
| `Hybrid_0.txt` | ESP32 | 24.4 Hz | 24.4 Hz |
| `data_0.bin` (this session) | WitMotion PC software | 24.4 Hz | 24.4 Hz |

**Identical result (24.4 Hz) from two independent tools, two independent
sessions.** This is the strongest repeatability evidence gathered in this
project to date for any FIFO-related claim.

---

## 3. FFT Correlation

| Source | Z-axis frequency |
|---|---|
| FFT computed independently from ESP32 FIFO capture | 24.4 Hz |
| FFT computed independently from WitMotion-app FIFO capture | 24.4 Hz |
| Register `HZZ` (0x46), same test session | 23.7 Hz |

FFT resolution at SR=1kHz, N=1024 samples is 1000/1024 ≈ 0.977 Hz/bin. The
0.7 Hz difference between the independently-computed FFT and the
register's own reported frequency is **under 1 full bin** — effectively a
match at the limit of what this buffer size can resolve.

**What this establishes:** an FFT computed with no vendor logic at all,
run against the raw FIFO samples, lands on the same answer as the sensor's
internal frequency-detection register, and reproduces identically across
two independent capture tools. This is strong support for "FIFO contains
real, acceleration-domain vibration data reflecting the same physical
signal that feeds the register pipeline" — the central claim RFC-0006 set
out to test.

**What this does NOT establish:** it does not by itself prove the FIFO
samples are *unprocessed*. A lightly-filtered or calibrated signal would
show the same dominant frequency (filtering does not, in general, shift
where the energy peak sits, though it may reshape the spectrum around it).
Frequency-domain agreement is evidence about signal *content* (real vs.
unrelated), not about *processing stage* — hence §4 remains open.

### 3.1 New evidence gathered for this document: windowing check

A real analysis window (Hann, Hamming, etc.) applied before storing this
buffer would taper the signal toward zero near both edges. RMS of the AC
component was compared across the first 100, middle 200, and last 100
samples of three captures:

| Capture | RMS (first 100) | RMS (middle 200) | RMS (last 100) |
|---|---|---|---|
| Idle (ESP32) | 0.0011 g | 0.0012 g | 0.0012 g |
| Running motor (ESP32) | 0.0255 g | 0.0235 g | 0.0236 g |
| Running motor (WitMotion app) | 0.0252 g | 0.0248 g | 0.0208 g |

No segment shows the dramatic (multi-fold) drop toward the edges that a
real window function would produce — all three segments stay within
roughly ±20% of each other in every capture. **This is evidence against
significant windowing**, though a mild/tapered window with a long flat
plateau (common in some designs) would not necessarily be ruled out by
this check alone. **Small or proprietary window functions cannot yet be
excluded** — this check only tests for the coarse, large-taper style of
windowing (e.g. Hann/Hamming applied across the full buffer); a narrow
taper confined to a few samples at each edge, or a non-standard/proprietary
shaping function, would not necessarily produce a large enough RMS
deviation for this test to detect.

---

## 4. Open Questions (the one remaining technical question, broken into parts)

The central open question, as posed: **is this data (a) raw ADC counts,
(b) calibrated accelerometer counts, (c) post-anti-alias-filter, (d)
post-digital-low-pass-filter, or (e) partially windowed?**

| Sub-question | Status | Basis |
|---|---|---|
| Scaling factor (raw/32768×16g) is correct | ✅ Confirmed | Produces gravity ≈1.00g consistently across 3 independent captures; wrong scaling would not coincidentally hit 1g three times |
| Signal reflects real acceleration content (not an unrelated buffer) | ✅ Confirmed | FFT match to register HZZ, reproduced across 2 tools |
| Significant windowing (Hann-like taper) applied | 🟠 Evidence against | Edge-vs-middle RMS check (§3.1); not a large enough sample of tests to fully exclude a subtler window |
| Anti-alias filtering present | ⚪ Unknown / likely but unconfirmable | Virtually universal in ADC design generally, but this project's black-box tests cannot distinguish "some anti-alias filtering" from "none" — both are consistent with everything observed so far |
| Digital low-pass filtering beyond anti-alias present | ⚪ Unknown | Not tested; would require comparing FIFO frequency content against a known reference at multiple frequencies to see if high-frequency content is suppressed relative to expectation |
| Calibration (bias/scale correction) applied before this buffer | ⚪ Unknown | Cannot be distinguished from "no calibration" using DC-magnitude or frequency tests alone; would require a controlled reference acceleration input |
| Exact DSP stage this buffer represents | ⚪ Unknown | This is the resolvable-only-by-vendor-answer question flagged in RFC-0005 §8 Recommendation 1 |

### Recommended terminology (per review's own framing)

Given the above, **"raw acceleration" overstates what has been shown**.
Recommended going forward: call this **"time-domain acceleration samples"**
— accurate to everything confirmed (real, acceleration-domain, unwindowed
to first approximation, correctly scaled) without asserting a processing
stage that has not been established (raw ADC vs. calibrated vs. filtered).

---

## 5. What would close the remaining question

1. **Ask Witmotion directly** (already recommended in RFC-0005 §8,
   unchanged priority): does this register output raw ADC counts,
   calibrated counts, or filtered data? This is the only source that can
   answer the calibration/filtering sub-questions with certainty — no
   black-box experiment fully substitutes for it.
2. **Controlled reference input**: if a calibrated shaker table or known
   reference accelerometer becomes available, comparing its output against
   the FIFO at multiple known frequencies/amplitudes could reveal filtering
   (frequency-dependent attenuation) or calibration offsets directly.
3. **Compare FIFO content across different `MODE` settings** (still
   pending from RFC-0006 §6.3) — if switching MODE changes the FIFO's
   apparent DC level, noise floor, or spectral shape, that would suggest
   at least some of the observed characteristics are mode-dependent
   processing rather than a fixed raw tap.
