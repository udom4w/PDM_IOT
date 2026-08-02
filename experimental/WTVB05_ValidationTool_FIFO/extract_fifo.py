#!/usr/bin/env python3
"""
extract_fifo.py -- Production Firmware FIFO CSV extractor (engineering
validation tool)

Extracts every FIFO waveform dump emitted by the Production Firmware's
debug-only DumpFifoCaptureCsv() (WTVB02_ESP32S3_V16_CM_fault_latch_v3_
hardened_patched_v16_5.ino, guarded by #define DEBUG_FIFO_DUMP) out of a raw
serial log, into standalone CSV files consumable directly by
analyze_fifo_capture.py.

This tool is READ-ONLY with respect to firmware: it does not modify, build,
or flash WTVB02_ESP32S3_V16_CM_fault_latch_v3_hardened_patched_v16_5.ino,
FifoDriver, the Trigger Broker, or DumpFifoCaptureCsv() itself. It only
parses whatever serial log text is handed to it on the command line.

Detection is pattern-based, not filename- or offset-based, so it works
against any serial log regardless of capture tool (raw TeraTerm log, a
capture_*.ps1 log with "HH:MM:SS.mmm " timestamps prefixed to every line,
or an unprefixed log) -- it looks for the literal substrings/shapes the
firmware itself prints:

    Serial.printf("[FIFO-RESULT] captureId=%lu trigger=%s error=%s "
                   "enqueued=%d szEvt=%u\n", ...);
    ...
    Serial.println("Index,X,Y,Z");                       // DumpFifoCaptureCsv()
    Serial.printf("%u,%d,%d,%d\n", i, x[i], y[i], z[i]);  // x1024

Usage:
    python extract_fifo.py teraterm.log
    python extract_fifo.py serial_raw_PHASE7A.log
    python extract_fifo.py my_capture.log
"""

import re
import sys
from pathlib import Path

HEADER_TEXT = "Index,X,Y,Z"
EXPECTED_SAMPLES = 1024

# Matches the [FIFO-RESULT] marker anywhere in a line, capturing captureId
# if present. Tolerant of any prefix (timestamp, etc.) before the marker.
RESULT_RE = re.compile(r"\[FIFO-RESULT\](?:.*?\bcaptureId=(\d+))?")

# Matches a bare "Index,X,Y,Z" header, allowing an arbitrary prefix (e.g. a
# capture-tool timestamp) before it, but nothing after it on the line.
HEADER_RE = re.compile(r"(?:^|\s)Index,X,Y,Z\s*$")

# Matches one DumpFifoCaptureCsv() data row ("%u,%d,%d,%d"), allowing an
# arbitrary prefix before it, but nothing after it on the line. This is
# also what tells the tool a dump has ended: the moment a line does NOT
# match this pattern, "another firmware log line" has begun.
ROW_RE = re.compile(r"(?:^|\s)(\d+),(-?\d+),(-?\d+),(-?\d+)\s*$")

# How many lines are allowed to intervene between a [FIFO-RESULT] line and
# its "Index,X,Y,Z" header before this is treated as a [FIFO-RESULT] with
# no dump attached (e.g. DEBUG_FIFO_DUMP was not compiled in for that
# firmware build). In the real call site the header printf is the very
# next line, but a small window is allowed for robustness.
HEADER_SEARCH_WINDOW = 5


def find_captures(lines):
    """Scan lines for FIFO captures.

    Returns a list of (capture_id_or_None, rows) tuples, in log order,
    where rows is a list of "idx,x,y,z" strings (already reconstructed
    from the matched groups, so any log-line prefix is stripped).
    """
    captures = []
    i = 0
    n = len(lines)
    while i < n:
        result_match = RESULT_RE.search(lines[i])
        if not result_match:
            i += 1
            continue

        capture_id = int(result_match.group(1)) if result_match.group(1) else None

        header_idx = None
        window_end = min(i + 1 + HEADER_SEARCH_WINDOW, n)
        for j in range(i + 1, window_end):
            if HEADER_RE.search(lines[j]):
                header_idx = j
                break
            if RESULT_RE.search(lines[j]):
                # A new [FIFO-RESULT] arrived before any header did --
                # this one had no dump attached (e.g. DEBUG_FIFO_DUMP was
                # off, or error != NONE so DumpFifoCaptureCsv() was never
                # called). Not a capture; resume scanning from there.
                break

        if header_idx is None:
            i += 1
            continue

        rows = []
        j = header_idx + 1
        while j < n and len(rows) < EXPECTED_SAMPLES:
            if lines[j].strip() == "":
                # Blank line -- a serial-capture artifact (observed once,
                # immediately after the header, in real production logs),
                # not a firmware log line. Skip without stopping.
                j += 1
                continue
            row_match = ROW_RE.search(lines[j])
            if not row_match:
                break  # another firmware log line begins -- stop
            rows.append(",".join(row_match.groups()))
            j += 1

        captures.append((capture_id, rows))
        i = j

    return captures


def write_capture_csv(out_path, rows):
    with out_path.open("w", newline="\n", encoding="ascii") as f:
        f.write(HEADER_TEXT + "\n")
        for row in rows:
            f.write(row + "\n")


def main():
    if len(sys.argv) != 2:
        print("Usage: python extract_fifo.py <serial_log_file>")
        sys.exit(1)

    log_path = Path(sys.argv[1])
    if not log_path.is_file():
        print(f"ERROR: file not found: {log_path}")
        sys.exit(1)

    text = log_path.read_text(errors="replace")
    lines = text.splitlines()

    captures = find_captures(lines)

    print(f"Found {len(captures)} FIFO captures")
    print()

    fallback_counter = 0
    summary = []
    for capture_id, rows in captures:
        if capture_id is not None:
            out_name = f"captureId{capture_id}.csv"
        else:
            fallback_counter += 1
            out_name = f"capture{fallback_counter}.csv"

        out_path = Path.cwd() / out_name
        write_capture_csv(out_path, rows)

        truncated = len(rows) != EXPECTED_SAMPLES
        if truncated:
            print("WARNING: truncated capture")
        summary.append((out_name, len(rows), truncated))

    for out_name, count, truncated in summary:
        suffix = " (WARNING)" if truncated else ""
        print(f"{out_name:<20} {count} samples{suffix}")


if __name__ == "__main__":
    main()
