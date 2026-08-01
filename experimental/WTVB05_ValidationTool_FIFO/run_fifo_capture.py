#!/usr/bin/env python3
"""
run_fifo_capture.py

Opens COM5 @ 115200, sends "SETPOINT 30" (metadata log only -- no Modbus,
does not drive the motor), then "SR <SR_INDEX>" (see SR_TABLE/SR_INDEX
below -- the Validation Tool firmware's own SAMPLE_RATE_HZ table, mirrored
here), waits 1s, then sends "FIFO test1" and "FIFO test2" in turn,
listening after each until the "FIFO_CAPTURE_END" sentinel line (or ~25s
timeout). Everything received is appended to serial_log.txt.

If a "CRC MISMATCH" or "FIFO capture FAILED" line is seen before the next
CSV header ("FIFOIndex,Tag,SR,...") in the capture window, automatically
retries the same "FIFO test<N>" command (up to 5 attempts per tag).
Failure markers left over from a stale/overlapping prior attempt are
discarded once a fresh header line appears, so they don't poison the
verdict for the capture that actually follows it.

On completion (success or exhausted retries for both tags), runs:
    analyze_fifo_capture.py serial_log.txt --sr-hz <Hz for SR_INDEX>

SR_INDEX below is the single owner of the sample rate for this whole
pipeline (Phase 1.1 refactor). The "SR <n>" command sent to the firmware
and the "--sr-hz" value passed to analyze_fifo_capture.py are both derived
from it via SR_TABLE -- there is no second, independently-hardcoded rate
value anywhere in this file. To run at a different rate, change SR_INDEX
only.
"""

import subprocess
import sys
import time

import serial

PORT = "COM5"
BAUD = 115200
LOG_FILE = "serial_log.txt"
CAPTURE_END_MARKER = "FIFO_CAPTURE_END"
CSV_HEADER_MARKER = "FIFOIndex,Tag,SR,X_raw,Y_raw,Z_raw,X_g,Y_g,Z_g"
FAILURE_MARKERS = ("CRC MISMATCH", "FIFO capture FAILED")
CAPTURE_TIMEOUT_S = 25
MAX_ATTEMPTS = 5

# Single source of truth for sample rate (Phase 1.1). Mirrors the firmware's
# own SR index -> (label, Hz) table (WTVB05_ValidationTool_v3_11_TRUEPOLL.ino
# SAMPLE_RATE_NAMES/SAMPLE_RATE_HZ, SR_TABLE_SIZE=10). Index is what actually
# gets sent to the sensor over serial; Hz is derived from it below, never
# hardcoded separately.
SR_TABLE = {
    0: ("32 kHz", 32000),
    1: ("16 kHz", 16000),
    2: ("8 kHz", 8000),
    3: ("4 kHz", 4000),
    4: ("2 kHz", 2000),
    5: ("1 kHz", 1000),
    6: ("512 Hz", 512),
    7: ("256 Hz", 256),
    8: ("128 Hz", 128),
    9: ("64 Hz", 64),
}

# THE single variable to change to run this pipeline at a different sample
# rate. Everything else (the "SR <n>" command and the --sr-hz passed to
# analyze_fifo_capture.py) is derived from this via SR_TABLE -- do not
# hardcode a rate anywhere else in this file.
SR_INDEX = 5

SR_LABEL, SR_HZ = SR_TABLE[SR_INDEX]


def send_command(ser, log, cmd):
    log.write(f">>> {cmd}\n")
    log.flush()
    print(f">>> {cmd}")
    ser.write((cmd + "\n").encode("utf-8"))
    ser.flush()


def listen_until_end(ser, log, timeout_s):
    """Read lines until CAPTURE_END_MARKER or timeout. Returns (completed, failed)."""
    deadline = time.monotonic() + timeout_s
    completed = False
    failed = False

    while time.monotonic() < deadline:
        line = ser.readline()
        if not line:
            continue
        try:
            text = line.decode("utf-8", errors="replace").rstrip("\r\n")
        except Exception:
            text = repr(line)

        log.write(text + "\n")
        log.flush()
        print(text)

        if CSV_HEADER_MARKER in text:
            # A fresh capture's data is starting -- any failure marker seen
            # so far belonged to a stale/overlapping earlier attempt and no
            # longer applies to the capture that's about to complete.
            failed = False
        elif any(marker in text for marker in FAILURE_MARKERS):
            failed = True
        if CAPTURE_END_MARKER in text:
            completed = True
            break

    return completed, failed


def capture_tag(ser, log, tag):
    """Send 'FIFO <tag>' and retry up to MAX_ATTEMPTS times until a clean
    capture completes. Returns True on success."""
    for attempt in range(1, MAX_ATTEMPTS + 1):
        print(f"[INFO] FIFO {tag} capture attempt {attempt}/{MAX_ATTEMPTS}")
        log.write(f"[INFO] FIFO {tag} capture attempt {attempt}/{MAX_ATTEMPTS}\n")
        log.flush()

        ser.reset_input_buffer()
        send_command(ser, log, f"FIFO {tag}")
        completed, failed = listen_until_end(ser, log, CAPTURE_TIMEOUT_S)

        if completed and not failed:
            return True

        print(f"[WARN] FIFO {tag} attempt {attempt} did not complete cleanly "
              f"(completed={completed}, failed={failed}). Retrying...")
        log.write(f"[WARN] FIFO {tag} attempt {attempt} did not complete cleanly "
                  f"(completed={completed}, failed={failed}). Retrying...\n")
        log.flush()

    print(f"[ERROR] FIFO {tag} capture did not succeed after {MAX_ATTEMPTS} attempts.")
    log.write(f"[ERROR] FIFO {tag} capture did not succeed after {MAX_ATTEMPTS} attempts.\n")
    log.flush()
    return False


def main():
    try:
        ser = serial.Serial(PORT, BAUD, timeout=1)
    except serial.SerialException as e:
        print(f"[ERROR] Could not open {PORT}: {e}")
        sys.exit(1)

    with ser, open(LOG_FILE, "a", encoding="utf-8") as log:
        # Give the port a moment to settle after opening (some boards reset on connect).
        time.sleep(2)
        ser.reset_input_buffer()

        send_command(ser, log, "SETPOINT 30")
        time.sleep(0.5)
        send_command(ser, log, f"SR {SR_INDEX}")
        time.sleep(1)

        success_test1 = capture_tag(ser, log, "test1")
        success_test2 = capture_tag(ser, log, "test2")

    if not (success_test1 and success_test2):
        print(f"[ERROR] Not all captures succeeded (test1={success_test1}, test2={success_test2}).")

    print(f"[INFO] Running analyze_fifo_capture.py (SR_INDEX={SR_INDEX} -> {SR_LABEL} / --sr-hz {SR_HZ})...")
    result = subprocess.run(
        [sys.executable, "analyze_fifo_capture.py", LOG_FILE, "--sr-hz", str(SR_HZ)]
    )
    sys.exit(result.returncode)


if __name__ == "__main__":
    main()
