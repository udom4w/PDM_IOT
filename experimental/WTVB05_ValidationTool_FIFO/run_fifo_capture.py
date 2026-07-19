#!/usr/bin/env python3
"""
run_fifo_capture.py

Opens COM5 @ 115200, sends "SETPOINT 25" (metadata log only -- no Modbus,
does not drive the motor), then "SR 6", waits 1s, then sends "FIFO test2"
and listens until the "FIFO_CAPTURE_END" sentinel line (or ~25s timeout).
Everything received is appended to serial_log.txt.

If a "CRC MISMATCH" or "FIFO capture FAILED" line is seen before the next
CSV header ("FIFOIndex,Tag,SR,...") in the capture window, automatically
retries "FIFO test2" (up to 5 attempts total). Failure markers left over
from a stale/overlapping prior attempt are discarded once a fresh header
line appears, so they don't poison the verdict for the capture that
actually follows it.

On completion (success or exhausted retries), runs:
    analyze_fifo_capture.py serial_log.txt --sr-hz 512
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

        send_command(ser, log, "SETPOINT 25")
        time.sleep(0.5)
        send_command(ser, log, "SR 6")
        time.sleep(1)

        success = False
        for attempt in range(1, MAX_ATTEMPTS + 1):
            print(f"[INFO] FIFO capture attempt {attempt}/{MAX_ATTEMPTS}")
            log.write(f"[INFO] FIFO capture attempt {attempt}/{MAX_ATTEMPTS}\n")
            log.flush()

            ser.reset_input_buffer()
            send_command(ser, log, "FIFO test2")
            completed, failed = listen_until_end(ser, log, CAPTURE_TIMEOUT_S)

            if completed and not failed:
                success = True
                break

            print(f"[WARN] Attempt {attempt} did not complete cleanly "
                  f"(completed={completed}, failed={failed}). Retrying...")
            log.write(f"[WARN] Attempt {attempt} did not complete cleanly "
                      f"(completed={completed}, failed={failed}). Retrying...\n")
            log.flush()

        if not success:
            print(f"[ERROR] FIFO capture did not succeed after {MAX_ATTEMPTS} attempts.")
            log.write(f"[ERROR] FIFO capture did not succeed after {MAX_ATTEMPTS} attempts.\n")
            log.flush()

    print("[INFO] Running analyze_fifo_capture.py...")
    result = subprocess.run(
        [sys.executable, "analyze_fifo_capture.py", LOG_FILE, "--sr-hz", "512"]
    )
    sys.exit(result.returncode)


if __name__ == "__main__":
    main()
