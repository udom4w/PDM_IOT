# PHASE 5 — Flash / Capture / Restore Plan (NOT EXECUTED in this task)

**No step below has been run. This is a plan only, for explicit future
authorization.**

## Pre-conditions confirmed already
- Validation-variant binary compiled: `fifo_dump_variant_build/WTVB02_ESP32S3_V16_CM_fault_latch_v3_hardened_patched_v16_5.ino.bin`, SHA256 `c42069785dc617d945d278de3f87e2a364f2c9a85631228ac1ad6184915165f0`.
- Current production binary on Pump01 (COM5), from the prior "FINAL FLASH" task: SHA256 `608bd830c1b07013d97195f376da0fc35aea53b974f3f30633a7d82a4f641b6e`, still present at `label_fix_build/...ino.bin` (verified intact, unchanged, this task).

## ⚠️ Timing risk carried forward from Phase 1 audit (not introduced by this patch)
`DumpFifoCaptureCsv()` (pre-existing, unmodified) runs **1024 `Serial.printf()`
calls synchronously inside `taskModbusRead()` on Core 0** every time a FIFO
capture succeeds (~every 2 s while the motor runs and the sensor is
healthy) — the source's own comment states this costs "order-of-1-2s at
115200 baud" and will "block the Modbus/CTR4A01 poll loop for most of every
period." **While this validation variant is flashed, expect visible Modbus
timing disruption** (T1a-T7 jitter, possible transient timeouts) as a known,
disclosed, pre-existing characteristic of enabling `DEBUG_FIFO_DUMP` at all
— not a defect in the two-line patch this task adds. This is the reason the
plan below calls for the shortest practical exposure window.

## Planned steps

1. **Confirm COM port identity** — `arduino-cli board list`; verify COM5 is
   the sole enumerated `ESP32 Family Device` (established convention this
   entire engagement); do not open/write any other port.
2. **Backup the currently-flashed production binary** — already satisfied:
   `label_fix_build/...ino.bin` (SHA256 `608bd830...`) is the exact binary
   presently on the device (flashed and hardware-validated in the "FINAL
   FLASH" task) and remains on disk, untouched, verified this task.
3. **Flash the validation-variant binary to COM5 only** — same `esptool`
   procedure already used twice this engagement (bootloader/partitions/
   boot_app0/app.bin, all four segments hash-verified by esptool itself).
4. **Capture** — open COM5 once, `dtr=False, rts=False` (the established
   safe procedure from the reboot-forensics finding earlier this
   engagement), for the **shortest window that yields the required
   captures** (see Phase 6), given the Core-0 blocking risk above. Do not
   reconnect/reopen mid-capture.
5. **Restore production** — re-flash `label_fix_build/...ino.bin` (SHA256
   `608bd830...`) to COM5.
6. **Verify restoration** — recompute the on-device SHA256 evidence the same
   way the "FINAL FLASH" task did (esptool's own post-write hash
   verification, plus a short post-flash boot capture confirming normal
   `T1a-T7` transaction sequence and MQTT reconnection, no FREQ/CF
   transactions, exactly as validated then).

Every step above requires a separate, explicit authorization before
execution, per this task's own instruction ("ห้าม flash และห้าม commit ใน
task นี้").
