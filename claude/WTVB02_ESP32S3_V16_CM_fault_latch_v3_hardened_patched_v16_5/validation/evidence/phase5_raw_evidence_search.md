# PHASE 5 — Search for real RAW FIFO capture evidence

**Result: NONE FOUND. No fabricated data was substituted.**

## What was searched

1. **`DEBUG_FIFO_DUMP` build flag** — the firmware's only code path that ever
   writes raw per-sample X/Y/Z data to Serial (`.ino`, `DumpFifoCaptureCsv()`,
   gated `#ifdef DEBUG_FIFO_DUMP`, "undefined by default"). Searched:
   - Every `.ino`/`.cpp`/`.h` in the sketch directory for the define itself:
     only the flag's own conditional-compilation guard exists; it is not
     defined anywhere.
   - Every build log produced during this entire engagement
     (`find .../Temp/claude -iname "*build*log*" | xargs grep -l DEBUG_FIFO_DUMP`):
     zero matches — no build in this engagement's history ever compiled with
     this flag enabled.
2. **Every captured serial log this engagement has produced**, both under
   `%TEMP%\claude\*.log` and `%TEMP%\*.log` (≈200 files, full listing
   enumerated during this search) — grepped for:
   - The `[FIFO-DUMP]`/`FIFO_DUMP` tag the debug path would emit.
   - A generic heuristic: any file containing more than 50 consecutive
     lines matching `^-?\d+,-?\d+,-?\d+` (a raw X,Y,Z CSV block would look
     like this). **Zero files matched.**
3. **`live_observe.log`** — present but empty (0 lines).

## Why this matters

Every capture examined across this entire engagement (post-flash smoke
tests, S3 validation, repeatability soak, sniffer soaks, etc.) only ever
contains **post-DSP metadata** — `capture_id`, `error`, `sample_count`,
`velocity_rms_x/y/z/overall`, `sample_rate_hz`, `dominant_frequency_*` — on
the `accel_rms` MQTT event, or the human-readable `SYSTEM STATUS REPORT`
block. **None contain the 1024 raw per-axis int16 samples a capture actually
consisted of.** Without those raw samples, there is no way to run the same
input through both the firmware's algorithm (already executed, producing
the logged `velocity_rms_*` numbers) and this review's independent Python
reference implementation, and therefore no way to perform PHASE 6/7/9's
numerical cross-check.

## Conclusion

**PHASE 5 is BLOCKED.** No real 1024-sample RAW FIFO capture with a
paired firmware VRMS result exists anywhere in this engagement's evidence.
No synthetic data was substituted for it. PHASE 6 (numerical cross-check),
PHASE 7 (intermediate-stage comparison), and PHASE 9 (spike-origin
investigation from RAW→DSP→MQTT) cannot be performed and are reported as
such in the final report, per the task's own explicit instruction for this
exact scenario.
