# V2 Architecture — Common Analysis Engine, Dual Input Format

**Status:** implemented in `experimental/analyze_fifo_dewesoft.py`. This is an **architectural
refactor only** — no DSP change. Every claim of "unchanged" below was verified by direct
execution, not asserted; see "Verification" at the end.

**Naming note:** V1 was frozen (as a git commit / feature baseline) with the Peak Spectrum
default, the Energy Spectrum addition, and the V2.1 stability fixes already included. This
document describes the *next* project phase after that freeze — a different, later use of
"V2" than the in-tool "V2: PEAK SPECTRUM vs ENERGY SPECTRUM" docstring section, which was a
pre-freeze, within-V1 feature addition. The module docstring itself calls this out to avoid
confusing the two.

---

## Goal

Support a second input format — WTVB05 FIFO capture CSVs (raw ADC counts) — **without**
writing a second FFT/RMS/statistics implementation, and without duplicating any DSP code
that already exists in `analyze_fifo_capture.py`. The single existing analysis engine now
serves both input formats.

## Architecture

```
                    +----------------------+
                    |     Input Layer       |
                    +----------+-----------+
                               |
          +--------------------+--------------------+
          |                                         |
   read_dewesoft_csv()                      read_fifo_csv()
   (Dewesoft acceleration CSV,               (WTVB05 FIFO CSV, raw ADC
    already in m/s^2)                         counts -> m/s^2 conversion
                                               happens HERE, inside the
                                               Input Layer, once)
          |                                         |
          +--------------------+--------------------+
                               |
                               v
              Common Analysis Engine (V1 DSP, unchanged)
        compute_time_domain_stats / compute_fft_spectrum /
        compute_energy_spectrum / compute_overall_rms /
        compute_band_rms / find_dominant_and_noise_floor /
        analyze_axis
                               |
                               |
         +----------+----------+-----------+----------+
         |          |          |           |          |
      Time       FFT       Statistics   JSON      Plots
   (waveform)  (Peak/     (DC/RMS/    (summary   (waveform.png,
               Energy      Peak/P2P/   .json)     fft.png)
               Spectrum)   StdDev/CF)
```

### Input Layer

Both loaders return the **identical** structure — this is the contract that lets the engine
stay input-agnostic:

```python
{
    "sample_rate": <float, Hz, echoed from --sr-hz>,
    "unit": "m/s²",
    "X": <ndarray>,
    "Y": <ndarray>,
    "Z": <ndarray>,
}
```

- **`read_dewesoft_csv(csv_path, sr_hz)`** — renamed/reshaped from V1's `load_axes()`. Column
  detection (`find_acceleration_columns()`) is byte-for-byte unchanged; acceleration is
  already in m/s² in a Dewesoft export, so no unit conversion happens here.
- **`read_fifo_csv(csv_path, sr_hz)`** — new. Supports the same two WTVB05 FIFO capture
  formats `analyze_fifo_capture.py` documents:
  - Phase 7A: `Index,X,Y,Z` header, raw ADC counts, single block.
  - Legacy: `FIFOIndex,Tag,SR,X_raw,Y_raw,Z_raw,X_g,Y_g,Z_g` header,
    `FIFO_CAPTURE_END`-delimited blocks. **Only the first capture block is used** if a file
    contains more than one — this engine analyzes one contiguous X/Y/Z record per run, the
    same one-array-per-axis shape the Dewesoft path has always produced. The precomputed
    `X_g`/`Y_g`/`Z_g` columns are intentionally ignored; raw counts are read and converted
    via the formula below, so both FIFO formats go through the identical conversion path.

- **`detect_input_format(csv_path)`** — sniffs the first non-blank line of the input file
  (a Dewesoft `m/s2` column header vs. a WTVB05 `Index,X,Y,Z` or legacy header) and returns
  `'dewesoft'` or `'fifo'`. This is what makes the *same* CLI invocation
  (`--sr-hz` and nothing else new) work for either input format — see requirement 5. The
  detection logic mirrors `analyze_fifo_capture.py`'s own first-line auto-detection *idiom*,
  not its code (that module is not imported — see "Why not import
  `analyze_fifo_capture.py`" below).

### FIFO unit conversion (requirement 3)

```
m/s² = (raw_counts / 2048) * 9.8
```

Same formula and constants as `analyze_fifo_capture.py`'s `counts_to_g()` / `g_to_ms2()`
(WTVB05 manual Sec 6.1.4.8/6.1.4.16: `g = raw_counts / 32768 * 16 = raw_counts / 2048`;
`m/s² = g * 9.8`). This conversion happens **entirely inside `read_fifo_csv()`**, before any
array reaches the Common Analysis Engine — the engine only ever sees already-converted m/s²
values, identically to the Dewesoft path. `FIFO_COUNTS_PER_G = 2048` and
`FIFO_STANDARD_GRAVITY_MS2 = 9.8` are named module-level constants, not a magic-number
one-liner, so the formula's provenance is traceable.

### Common Analysis Engine (unchanged)

Every function below is **byte-for-byte identical** to the pre-refactor (V2.1) file — none of
these were touched:

- `compute_time_domain_stats()` — Mean/AC RMS/Peak/Peak-to-Peak/Std Dev/Crest Factor.
- `hann_window()`, `coherent_gain()`, `noise_power_gain()`, `split_into_blocks()`.
- `compute_fft_spectrum()` — Peak Spectrum.
- `compute_energy_spectrum()`, `compute_overall_rms()`, `compute_band_rms()` — Energy
  Spectrum.
- `find_dominant_and_noise_floor()` — Dominant Frequency/Amplitude, Noise Floor, SNR.
- `analyze_axis()` — orchestrates the above; takes a plain ndarray, has no knowledge of
  where that array came from.

This layer receives `raw_axes['X']` / `raw_axes['Y']` / `raw_axes['Z']` — plain numpy arrays —
regardless of which loader produced them. It cannot ask, and does not need to ask
(requirement 4).

### Output Layer (unchanged)

`print_axis_report()`, `build_summary_dict()`, `plot_waveform()`, `plot_fft()` — none of these
were touched. `summary.json`'s schema is identical for both input formats (it has no
"source format" field — the engine's output has never distinguished input provenance, before
or after this refactor).

## Why not import `analyze_fifo_capture.py`

`analyze_fifo_capture.py` has its own, separate CLI and scope (multi-capture bearing-frequency
lookup, envelope analysis, spectrogram plotting) and is explicitly frozen/off-limits for
modification per project instructions. Rather than depend on it as a library (which would
couple this tool's stability to a much larger module's unrelated features, and pull in its
`scipy` dependency for a single arithmetic line), `read_fifo_csv()` reproduces only the one
conversion formula it actually needs. This is a one-line unit conversion (input
normalization), not a second FFT/RMS/statistics engine — the thing requirement 1/2 actually
prohibits duplicating.

## CLI compatibility (requirement 5)

```
python experimental/analyze_fifo_dewesoft.py dewesoft.csv --sr-hz 1000   # auto-detected as Dewesoft
python experimental/analyze_fifo_dewesoft.py fifo.csv --sr-hz 1000       # auto-detected as FIFO
```

No new required flag was added to select the format — `detect_input_format()` handles it.
`--sr-hz` remains required for both, since neither loader guesses sample rate from the file
(same "never guess the sample rate" rule `analyze_fifo_capture.py` already documents for its
own SR-index field).

## Explicitly not implemented (requirement 6)

Deferred to later V2 phases, per instruction — no scaffolding for any of these was added:

- Cross-format / cross-run comparison.
- ISO 20816 severity evaluation.
- Velocity integration.
- Envelope analysis.

## Verification

1. **Dewesoft path, byte-for-byte regression:** ran the validation file
   (`experimental/dewesoft_data/3Axis_acc_0009.csv`, `--sr-hz 1000`) through the refactored
   loader and compared the resulting `summary.json` against the pre-refactor V2.1 baseline as
   parsed JSON — identical on every field, every axis.
2. **FIFO path, independent cross-check:** ran a real WTVB05 FIFO capture
   (`experimental/WTVB05_ValidationTool_FIFO/SR1K/SR1K_20Hz_N/captureId5.csv`, Phase 7A
   format, 1024 samples) through `read_fifo_csv()` and independently recomputed
   `raw_counts / 2048 * 9.8` directly from the CSV in a separate script — X-axis Mean (DC),
   AC RMS, and Peak matched the tool's own reported values exactly
   (`9.652730` / `0.161762` / `0.515727`).
3. **Multi-block legacy-format handling:** a synthetic legacy-format file with two
   `FIFO_CAPTURE_END`-delimited blocks (first block: 3 samples averaging ~2015 counts;
   second block: 1 sample at 9999 counts) was loaded, and the tool reported exactly 3 samples
   with a DC matching the *first* block only — confirming the "first block only" behavior is
   real, not just documented.
4. **Format-detection error path:** an unrecognized CSV (`foo,bar,baz` header) was rejected
   with a specific, actionable error message and exit code 1, not a traceback.
5. **Plots inspected visually** for the FIFO-input run — dark theme, per-axis colors, and
   title/annotation rendering all carried over correctly (the Output Layer is shared, so this
   was expected, but was checked rather than assumed).
