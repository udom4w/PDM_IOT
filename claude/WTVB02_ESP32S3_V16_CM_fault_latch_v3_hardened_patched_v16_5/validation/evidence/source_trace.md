# VRMS Calculation — Source Trace (Phases 1–3)

Read-only trace. Source SHA256 of the reviewed `.ino`:
`f90ab62a0c4fd8de0ef7d3140e6131cae838dc4a15e4f99ba3f8b445fe2bf8c8` (unchanged
throughout this review — confirmed via `git status`, no edits made).

## PHASE 1 — Implementation trace, RAW FIFO → final VRMS

| Stage | File : Function / Lines | What happens |
|---|---|---|
| WTVB05 FIFO register 0x002C read | `.ino` `taskModbusRead()` broker block (~L6851-7040), request built via `FifoDriver_Request()` | Register 0x002C, FC03, qty 1, per ADR-0006 SCHEDULED-only admission |
| FIFO transport read | `fifo_driver.cpp` `FifoDriver_Service()` / `ReadDump()` (~L421-449) | Reads 6144-byte payload + 2 CRC bytes off UART2 |
| Sample decoding / byte order / sign | `fifo_codec.cpp:75-91` `SampleDecoder_DecodeStride()` | `rawX=(b0<<8)\|b1` etc. (**big-endian**), stride=6 bytes=X,Y,Z; unsigned→signed int16 via narrowing reinterpret (documented GCC two's-complement behavior) |
| Storage | `fifo_arena.cpp` — `FifoArena_WriteHandleX/Y/Z()` | 1024×int16 per axis, static arrays, never `memset` between captures (gated by consumer, see below) |
| Consumer gate / zero-copy handoff | `.ino:6617-6621` (unchanged across all prior audits) | `error==NONE && sampleCount==1024 && srHz!=0` before `memcpy` into `g_accelSnap` |
| Acceleration scaling | `vib_accel.h:34-35`, used in `vib_accel.cpp:21,45,64` and `vib_velocity.cpp:200` | `a = raw * (VIB_ACCEL_G_MS2/VIB_ACCEL_LSB_PER_G)` = `raw * (9.8/2048.0)` → m/s² |
| DC removal | `vib_accel.cpp:24-28` (accel path) / `vib_velocity.cpp:205-209` (velocity path) | Two-pass: `mean = Σa/N`, `a' = a-mean`, per axis, per capture only |
| Hann window | `vib_velocity.cpp:49-51` `BuildTables()` | Symmetric Hann, **denominator N−1**: `w[i]=0.5-0.5·cos(2π·i/(N−1))` |
| NPG / window normalization | `vib_velocity.cpp:54` | `NPG = mean(w²)` (power gain, explicitly **not** `mean(w)`) |
| FFT | `vib_velocity.cpp:67-105` `Fft()` | In-place iterative radix-2 DIT, N=1024, real input (`s_im[i]=0`) |
| One-sided spectrum scaling | `vib_velocity.cpp:230-238` | `ms_a[k]=(re²+im²)/(NPG·N²)`, **×2 for interior bins only** (`k≠0, k≠N/2`) |
| Frequency calculation | `vib_velocity.cpp:223,240` | `dF=srHz/N`; `f[k]=k·dF` |
| Acceleration → velocity PSD | `vib_velocity.cpp:241-242` | `ms_v[k] = ms_a[k] / (2π·f[k])²` |
| Frequency-bin selection (integration range) | `vib_velocity.cpp:227` | `k = VIB_VEL_HP_BIN(8) .. NHALF(512)` inclusive |
| RMS integration | `vib_velocity.cpp:225,242-243,325` | `msvSum=Σms_v[k]`; `RMS_axis=sqrt(msvSum)` (m/s) |
| mm/s conversion | `vib_velocity.cpp:393-397` (`×VIB_VEL_MS_TO_MMS=1000.0`) | Applied once per axis after `AxisVelocityRms()` returns |
| X/Y/Z → Overall RMS | `vib_velocity.cpp:398` | `vo = sqrt(vx²+vy²+vz²)` — **vector sum, not mean-of-3** |
| MQTT publish (fresh, per capture) | `.ino:10970-10987` | `event:"accel_rms"`, `velocity_rms_x/y/z/overall`, `vibration_source:"fifo_dsp"` |
| MQTT publish (snapshot, aged ≤10s) | `.ino:9695-9708` `displayVelocity()` → `.ino:10616` `doc["velocity_rms_overall"]` on `/vibration` | Separate read path, see `RCA` note below |

## PHASE 2 — Sampling verification

| Item | Value | Evidence |
|---|---|---|
| `sample_count` | **1024**, hardcoded | `fifo_driver.cpp:1175` `s_result.sampleCount = 1024;` (assignment, not a measured count) |
| `Fs` | **2000 Hz** (SR4) | Confirmed live twice in this engagement's captured boot logs: `[SR] Readback = SR4 (2 kHz)` |
| Window duration | 1024/2000 = **0.512 s** | Derived |
| FFT size | **1024** | `VIB_VEL_FFT_N` = `VIB_ACCEL_REQUIRED_SAMPLES` = 1024 (single definition, cannot drift) |
| Frequency resolution | **1.953125 Hz** | `2000/1024` — confirmed both by source formula and independently recomputed in `synthetic_tests.py` |
| Bin 8 | **15.625 Hz** | `8 × 1.953125` — confirmed in `synthetic_tests.py` |
| Bin range used for RMS | **k = 8 .. 512 inclusive** | `vib_velocity.cpp:227` `for (k = VIB_VEL_HP_BIN; k <= NHALF; k++)` |
| Partial capture rejected? | **Yes** | `VibVelocity_ComputeRms()` (`vib_velocity.cpp:368-370`) and `VibAccel_ComputeRms()` (`vib_accel.cpp:107-109`) both hard-gate `sampleCount != 1024 → return false`; the FIFO driver itself never produces any count other than 1024 on a successful capture |
| X/Y/Z alignment | Correct by construction | Single `SampleDecoder_DecodeStride()` call per 6-byte stride writes all three axes from the same stride simultaneously — no possibility of axis misalignment across strides |
| Endianness | **Big-endian**, confirmed | `fifo_codec.cpp:76-81` (`b[0]<<8 \| b[1]`) |
| Signed int16 | Confirmed correct | Two's-complement narrowing reinterpret, documented GCC-defined behavior (not UB on this toolchain) |
| Sample duplication/drop | Structurally excluded on the accepted path | Arena is never cleared between captures (no `memset` in `fifo_arena.cpp`), but a failed/partial capture is discarded wholesale by the consumer gate (`.ino:6617-6621`) before any DSP call — no code path blends old and new samples within one accepted 1024-sample buffer |

## PHASE 3 — Mathematical reconstruction (exact, from source; see `reference_vrms.py` for the runnable form)

```
RAW → acceleration:
  a[i] = raw[i] * (9.8 / 2048.0)                              [m/s^2]

DC removal:
  mean = (1/N) * sum(a[i])
  a_ac[i] = a[i] - mean                                       [m/s^2]

Hann (symmetric, N-1 denominator):
  w[i] = 0.5 - 0.5*cos(2*pi*i/(N-1))                           [dimensionless]

Windowed signal:
  x[i] = a_ac[i] * w[i]                                       [m/s^2]

NPG:
  NPG = mean(w[i]^2)                                          [dimensionless, power gain]

FFT:
  X[k] = FFT(x)  (N=1024, real input, complex output)

One-sided spectrum (acceleration power):
  ms_a[k] = |X[k]|^2 / (NPG * N^2)          for k = 8..512
  ms_a[k] *= 2                              for k != 0 and k != N/2 (interior bins)
                                                                [ (m/s^2)^2 ]

Velocity conversion:
  f[k]     = k * Fs / N                                       [Hz]
  ms_v[k]  = ms_a[k] / (2*pi*f[k])^2                           [ (m/s)^2 ]

RMS integration:
  msvSum    = sum_{k=8}^{512} ms_v[k]                          [ (m/s)^2 ]
  RMS_axis  = sqrt(msvSum)                                    [m/s]
  RMS_axis_mms = RMS_axis * 1000                                [mm/s]

Overall:
  RMS_overall = sqrt(RMS_x_mms^2 + RMS_y_mms^2 + RMS_z_mms^2)   [mm/s]
```

**Unit chain, confirmed at every step:** `int16 RAW counts` → `g` (implicit, folded
into the /2048 step) → `m/s²` → `(m/s²)² spectral power` → `(m/s)² spectral
power` (division by `(2πf)²`) → `m/s` (sqrt) → `mm/s` (×1000).

### Note on the two publish paths (relevant to the earlier RCA, restated here for completeness)
`event:"accel_rms"` publishes a **freshly computed** `velocity_rms_overall`
every ~2 s (`FIFO_PERIODIC_INTERVAL_MS=2000`). `/vibration`'s
`doc["velocity_rms_overall"]` instead reads `g_velCarrier`, a cross-core
snapshot that can be up to `VIB_VELOCITY_MAX_AGE_MS_TBD=10000` ms old,
published on its own 5/10/30 s cadence (`.ino:8791-8793`). These are two
different reads of conceptually the same quantity, not two different
formulas — this document does not re-litigate that finding, it only confirms
the formula itself (§Phase 3) is identical regardless of which publish path
is read.

### Note on Phase 8 correction
The first draft of `synthetic_tests.py`'s frequency-scaling sanity check
asserted an incorrect expected ratio (~4.0, confusing the power-domain
`(2πf)²` weighting with its amplitude-domain consequence). This was caught
by the test itself failing, independently re-derived by hand
(`RMS_v ∝ A/ω`, linear in `1/ω`, not `1/ω²`), corrected to the mathematically
correct expectation (ratio ≈ 2.0 for a 2× frequency step), and re-run — see
`synthetic_test_results.txt`. This is disclosed here for auditability, not
omitted.
