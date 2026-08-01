# FIFO Analysis Toolkit v1.0

## Overview

The FIFO Analysis Toolkit (`analyze_fifo_capture.py`) is an offline analysis
tool for vibration data captured via the WTVB05/WTVB02 sensor's FIFO buffer.
It turns a raw serial-log or CSV capture into a full set of time-domain and
frequency-domain plots (waveform, FFT, spectrogram, envelope, envelope FFT),
a machine-readable summary CSV, and — optionally — a comparison against a
bearing's theoretical characteristic fault frequencies.

The tool is deliberately conservative about what it claims: it computes
exactly what the numbers say from the captured samples, never estimates
missing inputs (sample rate, shaft RPM), and never performs fault diagnosis.
Anything requiring an assumption (e.g. "this peak means a bearing fault") is
left to the operator — the toolkit's job stops at *presenting the evidence*.

## Supported Input

The input file format is auto-detected from the first non-blank line, so
the same command works for either format without a flag.

### Legacy FIFO CSV (9-column)

```
FIFOIndex,Tag,SR,X_raw,Y_raw,Z_raw,X_g,Y_g,Z_g
```

- One or more capture blocks per file, each terminated by a
  `FIFO_CAPTURE_END` sentinel line.
- May be embedded in a raw Serial Monitor log with boot/debug noise around
  it — the parser only looks for the header line, data rows, and the END
  sentinel, and ignores everything else.
- `X_g`/`Y_g`/`Z_g` are already g-scaled by the firmware; the toolkit
  analyzes these values directly. `X_raw`/`Y_raw`/`Z_raw` are the
  corresponding raw ADC counts, parsed but not analyzed.
- `Tag` and `SR` (sample-rate register index) come from the capture itself.

### Phase 7A CSV (4-column)

```
Index,X,Y,Z
```

- `X`/`Y`/`Z` are **raw ADC counts**, not g — there is no g-conversion
  available for this format, and the toolkit never invents one. All plots
  and console output for this format are labeled "Raw Counts".
- No `Tag` or `SR` fields exist in this format; a tag is synthesized as
  `phase7a_1`, `phase7a_2`, ... per block, and the sample-rate register
  index is left blank (`SR_index=None`) in the console output.
- A block runs from its `Index,X,Y,Z` header line to the next header line
  or end of file — there is no END sentinel in this format.
- Because there's no gravity-scaled DC value, the gravity-vector sanity
  check (see below) is skipped for this format.

## Analysis Pipeline

```
FIFO Capture
     |
     v
    CSV                          (legacy or Phase 7A, auto-detected)
     |
     v
Time Waveform                    (*_timeseries.png)
     |
     v
    FFT                          (*_fft.png, RMS/Peak/dominant frequency)
     |
     v
Spectrogram                      (*_spectrogram.png, STFT)
     |
     v
Envelope                         (*_envelope.png, Hilbert transform)
     |
     v
Envelope FFT                     (*_envelope_fft.png)
     |
     v
Bearing Database                 (bearing_database/bearing_db.json)
     |
     v
Bearing Frequency Calculator     (shaft_hz, FTF, BPFO, BPFI, BSF)
     |
     v
Bearing Frequency Overlay        (dashed lines on *_envelope_fft.png)
```

Every stage after "CSV" is independent and additive — later stages read
from earlier ones (e.g. the envelope FFT reuses the exact envelope array,
the overlay reuses the exact computed frequencies) but never modify them.
Disabling a later stage (e.g. omitting `--manufacturer`/`--bearing`) never
changes the output of an earlier one.

## Command Line Options

```
python analyze_fifo_capture.py <logfile> [options]
```

| Option | Required | Description |
|---|---|---|
| `logfile` | yes | Serial monitor log or plain CSV containing one or more FIFO captures. |
| `--sr-hz FLOAT` | no | True sample rate in Hz, for correct FFT/spectrogram/envelope-FFT frequency-axis labeling and bearing-frequency overlay. If omitted, frequency axes are labeled in "cycles per block" instead of Hz, and the bearing-frequency overlay is not drawn (it needs a real Hz axis). Never guessed from the SR register index embedded in legacy captures — that index only maps to Hz via the firmware's `SAMPLE_RATE_HZ` table, which the tool doesn't have. |
| `--out-prefix STR` | no | Output filename prefix for all generated files. Default: `fifo_analysis`. |
| `--manufacturer STR` | only with `--bearing` | Bearing manufacturer, e.g. `SKF`. Must be supplied together with `--bearing` — either both or neither. Looked up in `bearing_database/bearing_db.json`. |
| `--bearing STR` | only with `--manufacturer` | Bearing model, e.g. `6205`. Must be supplied together with `--manufacturer`. |
| `--shaft-rpm FLOAT` | no | Shaft speed in RPM. Only used if a bearing was also selected. Must be `> 0` to produce a result — `0`, a negative value, or omitting it entirely leaves `shaft_hz`/`FTF`/`BPFO`/`BPFI`/`BSF` blank in the CSV and undrawn in the overlay. RPM is never estimated. |

### Examples

Basic run, legacy format, Hz-labeled frequency axes:

```
python analyze_fifo_capture.py serial_log.txt --sr-hz 1000
```

Phase 7A input, custom output prefix:

```
python analyze_fifo_capture.py phase7a_sample_captureId1_20260801.csv --sr-hz 1000 --out-prefix run042
```

Full pipeline — bearing lookup, frequency calculation, and overlay:

```
python analyze_fifo_capture.py serial_log.txt --sr-hz 1000 \
    --manufacturer SKF --bearing 6205 --shaft-rpm 1800
```

Bearing geometry only, no shaft speed known yet (frequency/overlay stages
stay blank/undrawn, everything else still runs):

```
python analyze_fifo_capture.py serial_log.txt --sr-hz 1000 --manufacturer SKF --bearing 6205
```

## Output Files

All output files are written next to wherever `--out-prefix` points, named
`<out-prefix>_<tag>_<kind>.png` (one set per capture) plus one shared
`<out-prefix>_summary.csv`.

| File | Description |
|---|---|
| `*_timeseries.png` | Time waveform, all samples, X/Y/Z stacked vertically. Units are g (legacy) or Raw Counts (Phase 7A). |
| `*_fft.png` | Whole-capture FFT magnitude spectrum per axis (mean-removed before transforming), with DC/RMS/Peak/dominant-frequency annotated per subplot title. |
| `*_spectrogram.png` | Short-Time Fourier Transform spectrogram per axis (explicit Hann window, 50% overlap, window length auto-sized to the capture length), one shared colorbar for the whole figure. |
| `*_envelope.png` | Envelope of the mean-removed waveform per axis, via `abs(hilbert(signal))`. No band-pass filter is applied (unfiltered envelope only). |
| `*_envelope_fft.png` | FFT of the envelope signal per axis (DC-removed, no window function). When a bearing and valid `--shaft-rpm` are both supplied, this plot additionally overlays dashed vertical lines at the bearing's characteristic frequencies (see "Bearing Frequency Overlay" below). |
| `*_summary.csv` | One row per capture, all numeric results in one machine-readable table (columns documented below). |

### `*_summary.csv` columns

| # | Column | Description |
|---|---|---|
| 1 | `tag` | Capture tag (from the CSV for legacy input, synthesized `phase7a_N` for Phase 7A). |
| 2 | `input_format` | `legacy` or `phase7a`. |
| 3 | `sample_count` | Number of samples in this capture (1024 for a full FIFO block). |
| 4 | `sample_rate_hz` | Value of `--sr-hz` if supplied, else blank. |
| 5–7 | `rms_x`, `rms_y`, `rms_z` | Mean-removed RMS per axis, from the raw samples. |
| 8–10 | `peak_x`, `peak_y`, `peak_z` | Mean-removed peak (max absolute value) per axis. |
| 11–13 | `dominant_freq_x_hz`, `dominant_freq_y_hz`, `dominant_freq_z_hz` | Dominant frequency from the whole-capture FFT per axis. Blank if `--sr-hz` wasn't supplied (the value would be in cycles/block, not Hz). |
| 14 | `waveform_png` | Path to the time waveform PNG. |
| 15 | `fft_png` | Path to the whole-capture FFT PNG. |
| 16 | `spectrogram_png` | Path to the spectrogram PNG. |
| 17 | `spectrogram_window` | STFT window length (samples) used for the spectrogram. |
| 18 | `spectrogram_overlap` | STFT overlap fraction used for the spectrogram (fixed at 0.5 = 50%). |
| 19 | `envelope_png` | Path to the envelope PNG. |
| 20 | `envelope_method` | Always `hilbert_raw` — Hilbert-transform envelope of the unfiltered, mean-removed waveform. |
| 21–23 | `dominant_envelope_freq_x_hz`, `dominant_envelope_freq_y_hz`, `dominant_envelope_freq_z_hz` | Dominant frequency from the envelope FFT per axis. Blank if `--sr-hz` wasn't supplied. |
| 24 | `envelope_fft_png` | Path to the envelope FFT PNG. |
| 25 | `bearing_model` | `<manufacturer><bearing>` (e.g. `SKF6205`) if a bearing was selected, else blank. |
| 26 | `rolling_elements` | Bearing's number of rolling elements (Z), from the bearing database. Blank if no bearing selected. |
| 27 | `ball_diameter_mm` | Bearing's ball/roller diameter (Bd) in mm, from the bearing database. Blank if no bearing selected. |
| 28 | `pitch_diameter_mm` | Bearing's pitch diameter (Pd) in mm, from the bearing database. Blank if no bearing selected. |
| 29 | `contact_angle_deg` | Bearing's contact angle in degrees, from the bearing database. Blank if no bearing selected. |
| 30 | `shaft_hz` | Shaft rotational speed in Hz (`--shaft-rpm` / 60). Blank unless a bearing was selected AND `--shaft-rpm` was a valid positive number. |
| 31 | `FTF` | Fundamental Train Frequency (cage frequency), Hz. Same blank rule as `shaft_hz`. |
| 32 | `BPFO` | Ball Pass Frequency, Outer race, Hz. Same blank rule as `shaft_hz`. |
| 33 | `BPFI` | Ball Pass Frequency, Inner race, Hz. Same blank rule as `shaft_hz`. |
| 34 | `BSF` | Ball Spin Frequency, Hz. Same blank rule as `shaft_hz`. |

## Bearing Database

Bearing geometry is stored externally in
`bearing_database/bearing_db.json`, next to the script, rather than
hard-coded — adding a new bearing model means editing this file, not the
Python source.

### Format

```
{
  "<manufacturer>": {
    "<model>": {
      "rolling_elements": <int>,
      "ball_diameter_mm": <float>,
      "pitch_diameter_mm": <float>,
      "contact_angle_deg": <float>
    },
    ...
  },
  ...
}
```

### Required fields (per bearing model)

| Field | Meaning |
|---|---|
| `rolling_elements` | Number of balls/rollers in the bearing (Z). |
| `ball_diameter_mm` | Ball/roller diameter in mm (Bd). |
| `pitch_diameter_mm` | Pitch diameter in mm (Pd) — the diameter of the circle through the centers of the rolling elements. |
| `contact_angle_deg` | Contact angle in degrees (0 for a pure radial deep-groove ball bearing). |

The database is read once per run, and its shape is validated before any
lookup: the file must exist and parse as JSON, the top level must be an
object mapping manufacturer names to objects of bearing models, and every
bearing model must have all four fields above. Any violation — missing
file, invalid JSON, wrong shape, or a bearing missing a required field —
prints a specific error message and exits (no traceback, no partial run).

### Example (shipped defaults)

```json
{
  "SKF": {
    "6205": {
      "rolling_elements": 9,
      "ball_diameter_mm": 7.94,
      "pitch_diameter_mm": 39.04,
      "contact_angle_deg": 0.0
    },
    "6206": {
      "rolling_elements": 9,
      "ball_diameter_mm": 9.53,
      "pitch_diameter_mm": 46.00,
      "contact_angle_deg": 0.0
    }
  }
}
```

### Bearing Frequency Calculator

Given a bearing's geometry and a shaft speed, the standard rolling-element
bearing equations are used:

```
shaft_hz = shaft_rpm / 60
FTF  = (shaft_hz / 2) * (1 - (Bd/Pd) * cos(contact_angle))
BPFO = (Z / 2) * shaft_hz * (1 - (Bd/Pd) * cos(contact_angle))
BPFI = (Z / 2) * shaft_hz * (1 + (Bd/Pd) * cos(contact_angle))
BSF  = (Pd / (2*Bd)) * shaft_hz * (1 - ((Bd/Pd) * cos(contact_angle))^2)
```

Shaft RPM is **never estimated or guessed** — if `--shaft-rpm` is omitted,
zero, or negative, every one of these frequencies is left blank rather than
computed from an assumed value.

### Bearing Frequency Overlay

When a bearing and a valid `--shaft-rpm` are both supplied (and `--sr-hz`
is given, so the FFT axis is genuinely in Hz), the five characteristic
frequencies above are drawn as dashed vertical lines on `*_envelope_fft.png`
— one distinct color per frequency, each with a legend entry labeled in Hz
(e.g. `BPFO 107.5 Hz`). Only frequencies below Nyquist (`sample_rate_hz /
2`) are drawn; a computed frequency at or above Nyquist is still present in
the CSV but simply isn't plotted (it would fall outside, or alias onto,
this FFT's frequency axis). If shaft RPM wasn't supplied, nothing is
drawn — no lines, no legend — and the plot is pixel-identical to a run
with no bearing selected at all.

The overlay is plot-only: it does not alter the envelope-FFT calculation,
any other FFT/spectrogram/envelope calculation, or any CSV column.

## Known Limitations

The following are intentionally **not** implemented in v1.0 — the toolkit
computes and presents numbers; it does not interpret them:

- **No Peak Search** — the overlay draws bearing frequencies at their
  theoretical positions; it does not search the spectrum for nearby peaks
  or report how close/far an actual peak is.
- **No Fault Diagnosis** — no automated "this looks like an inner-race
  fault" type conclusion is ever produced.
- **No PDF Report** — output is PNGs + CSV only; no formatted report
  document is generated.
- **No Order Tracking** — frequencies are absolute Hz, computed from a
  fixed `--shaft-rpm`; there is no RPM-synchronous resampling or
  order-domain analysis for varying shaft speed.
- **No Automatic Bearing Identification** — the manufacturer and model
  must be supplied explicitly via `--manufacturer`/`--bearing`; the tool
  never guesses which bearing is installed.
- **No band-pass filter before the envelope** — the envelope is computed
  from the raw (mean-removed) waveform, not a band-pass-filtered signal
  around a resonance band.

## Validation

A full Phase 1 Final Acceptance Test (FAT) was executed against this
release, covering legacy-format regression, Phase 7A regression, the
bearing database (valid/invalid manufacturer, valid/invalid bearing,
malformed JSON, missing required field), the bearing frequency calculator
(valid RPM, zero RPM, no RPM), the bearing overlay (with and without RPM),
the Nyquist cutoff, a full regression check isolating the overlay's effect
to only `*_envelope_fft.png`, summary CSV structure/values, `py_compile`,
and CLI `--help` coverage.

**Result: 10 / 10 PASS**

**No Regression Detected** — waveform, FFT, spectrogram, and envelope
outputs are confirmed byte-for-byte identical regardless of whether
bearing/RPM/overlay features are used; only `*_envelope_fft.png` differs,
and only when overlay data is actually available.

## Future Roadmap

Phase 2 candidate features (not implemented in this release):

- **Peak Marker** — locate and annotate the actual spectral peak nearest
  each bearing frequency, with amplitude and frequency offset.
- **Automatic Fault Detection** — threshold- or trend-based flagging of
  bearing frequencies that show sustained/growing energy.
- **Order Tracking** — RPM-synchronous resampling for analysis under
  varying shaft speed, replacing fixed-RPM absolute-Hz frequencies.
- **Bearing Envelope Filter** — band-pass filter the waveform around a
  resonance band before computing the envelope, instead of using the raw
  signal.
- **PDF Report** — a formatted, shareable report document combining the
  plots, summary table, and overlay results per capture.
