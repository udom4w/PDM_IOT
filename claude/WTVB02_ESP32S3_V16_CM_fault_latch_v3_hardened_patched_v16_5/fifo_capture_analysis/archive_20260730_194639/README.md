# FIFO Investigation Archive — 2026-07-30 19:46:39 (+07:00)

## Archive creation time

2026-07-30 19:46:39 local time (folder name `archive_20260730_194639`), created as a snapshot before any further hardware testing.

## Investigation purpose

This archive captures the artifacts produced while investigating why the WTVB02/WTVB05 RAWFIFO capture on `WTVB02_ESP32S3_V16_CM_fault_latch_v3_hardened_patched_v16_5` repeatedly fails to complete: the driver consistently stalls at `partialBytesReceived≈5501/6146` (attempts 1 & 3, `ERR_INTER_BYTE_TIMEOUT`) or declares a CRC mismatch after reaching a full byte count (attempt 2, `ERR_DESYNC_LIMIT`), across independent hardware runs. The investigation proceeded in stages:

1. Confirmed whether any code triggers the FIFO read (`50 03 00 2C 00 01`) at all, and instrumented the one-shot trigger (`[FIFO-TRIGGER]` logging) to verify it fires and to capture its accept/reject reasoning.
2. Flashed the instrumented firmware, ran a verification capture, and reconstructed the full 3-attempt FIFO session timeline from the target's own Serial log.
3. Correlated that timeline against an independent, standalone ESP32 RS485 sniffer (COM4) tapped onto the bus, to determine whether the byte loss/CRC mismatch originates on the wire (sensor-side) or inside the ESP32 (driver-side).
4. Verified the confirmed Sampling Rate (SR5/1kHz, register `0x29`) was written and read back before the session in question.
5. Traced the complete firmware receive path (UART → parser → CRC) to identify the smallest set of code locations that could explain the observed byte loss, using UART `FIFO_OVF`/`BUFFER_FULL` error counters as the primary evidence.

No firmware source was modified, rebuilt, or reflashed as part of producing this archive; only diagnostic Serial logging was added and flashed in earlier steps of this investigation (already covered by the archived build/upload logs).

## Files in this archive

All files below were **copied**, not moved — originals remain in `fifo_capture_analysis/` (log files) and the sketch root (`.ps1` scripts) unchanged.

### Raw evidence — target ESP32 Serial captures (`serial_raw_*.log`)

| File | Description |
|---|---|
| `serial_raw_20260730.log` | Early baseline Serial capture, pre-instrumentation. |
| `serial_raw_0bfad96_baseline.log` | Baseline capture at commit `0bfad96` (FIFO architecture modularization checkpoint). |
| `serial_raw_recover-experimental_rerun.log` | Capture on the `recover-experimental` branch, re-run. |
| `serial_raw_TEMP_DIAG.log` | Capture with the transport-layer `TEMP_DIAG` instrumentation (`HandleReceivingImpl`/`Uart485Transport_OnReceiveError` diagnostic points A/B). |
| `serial_raw_TEMP_DIAG_VERIFY_A.log` | Follow-up capture verifying the `TEMP_DIAG` instrumentation actually produced visible output. |
| `serial_raw_FIFO_TRIGGER_VERIFY_20260730.log` | **Primary evidence file.** First full capture with the `[FIFO-TRIGGER]`/`[FIFO-DIAG]` accept/reject logging; the FIFO session analyzed in the "reconstruct the complete FIFO transaction" and "firmware receive-path trace" steps (trigger at `t≈74198ms`; all 3 attempts, `FIFO_OVF`/`BUFFER_FULL` counters, `WHY_NOT_FULL_DUMP` diagnostics). |
| `serial_raw_SNIFFER_VALIDATION_20260730_183449.log` | Paired target-side capture for the first sniffer-correlation attempt — no FIFO session occurred (board did not actually reset; see companion sniffer log). |
| `serial_raw_SNIFFER_VALIDATION_20260730_184253.log` | Paired target-side capture for the successful sniffer-correlation run (reset performed via `esptool` RTS handshake). Contains the full 3-attempt session used for the RS485 sniffer cross-check. |

### Raw evidence — standalone RS485 sniffer captures (`sniffer_raw_*.log`)

| File | Description |
|---|---|
| `sniffer_raw_SNIFFER_VALIDATION_20260730_183449.log` | Sniffer capture (COM4) for the first attempt — board never reset, so this shows only background Modbus polling, no FIFO session. |
| `sniffer_raw_SNIFFER_VALIDATION_20260730_184253.log` | **Primary sniffer evidence file.** Sniffer capture paired with the successful target reset — independently confirms all 3 FIFO attempts as `FULL_DUMP (CRC OK)`, 6149 bytes each, `Bytes Lost Because Buffer Full: 0`, used to establish that the byte loss/CRC mismatch is firmware-side, not on the wire. |

### Raw evidence — build/upload tool output

| File | Description |
|---|---|
| `build_log.txt`, `build_log_0bfad96.txt`, `build_log_recover-experimental_rerun.txt`, `build_log_TEMP_DIAG.txt`, `build_log_TEMP_DIAG_VERIFY.txt`, `build_log_TEMP_DIAG_VERIFY_A.txt`, `build_log_FIFO_TRIGGER_DIAG.txt` | `arduino-cli compile` output for each successive instrumentation stage, confirming clean builds against the project's mandated production FQBN. |
| `upload_log.txt`, `upload_log_0bfad96.txt`, `upload_log_recover-experimental_rerun.txt`, `upload_log_TEMP_DIAG.txt`, `upload_log_TEMP_DIAG_VERIFY_A.txt`, `upload_log_FIFO_TRIGGER_DIAG.txt` | `arduino-cli upload` output for each flash of the instrumented firmware. |

### Capture tooling (`*.ps1`) — not evidence, the scripts that produced it

| File | Description |
|---|---|
| `capture_fifo_dump_20260730.ps1` | Early ad-hoc Serial capture script, stops on `###FIFO_CSV_END`. |
| `capture_fifo_dump_TEMP_DIAG.ps1` | Variant adding an early-stop condition on `[FIFO-DUMP] result not usable, error=`. |
| `capture_fifo_trigger_verify_20260730.ps1` | Fixed-duration (90s) Serial capture used for the `[FIFO-TRIGGER]` verification run. |
| `capture_sniffer_COM4_20260730_183449.ps1`, `capture_sniffer_COM4_20260730_184253.ps1` | Passive (no DTR/RTS) capture scripts for the standalone RS485 sniffer on COM4. |
| `capture_target_COM5_20260730_183449.ps1`, `capture_target_COM5_20260730_184253.ps1` | Paired target-side (COM5) capture scripts for the two sniffer-correlation sessions. |

## Raw evidence vs. derived analysis

- **Raw evidence:** every `serial_raw_*.log`, `sniffer_raw_*.log`, `build_log_*.txt`, and `upload_log_*.txt` file above — direct, unedited tool/hardware output.
- **Capture tooling (neither evidence nor analysis):** every `capture_*.ps1` script — these produced the raw evidence but contain no findings themselves.
- **Derived analyses:** **none exist as separate files.** Every analysis step in this investigation (FIFO session reconstruction, sniffer-vs-target comparison table, SR/register-0x29 verification, and the firmware receive-path trace with its risk table) was delivered as in-conversation responses and was never written to disk as a standalone report, CSV, or Markdown file. If a persisted analysis document is needed, it must be generated separately — this archive intentionally does not fabricate one.

## Integrity

- `inventory.txt` — filename, size in bytes, and last-modified timestamp for every file in this archive (generated after copying, before this README).
- `SHA256SUMS.txt` — SHA-256 of every file in this archive except itself (verify with `sha256sum -c SHA256SUMS.txt` from inside this folder).
