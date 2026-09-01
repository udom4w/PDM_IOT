#!/usr/bin/env python3
# =============================================================================
# printf_audit.py -- build-gate static check for printf-family format strings
# =============================================================================
# WHY THIS EXISTS
#
# Commit 2c351f0 removed three trailing arguments from the /vibration debug
# log but left their conversion specifiers in the format string:
#
#     Serial.printf("... cf=%s kurt_max=%.3f(%s) bear=%s\n",
#                   ..., crestFactorOk ? ... : "--");   <-- 13 specs, 10 args
#
# printf then read three varargs that were never pushed. The %s dereferenced a
# garbage pointer and the device panicked (LoadProhibited, A2 = 0x00000000) on
# every publish -- a crash-reboot loop on live hardware. The build was clean
# with zero warnings: the ESP32 Arduino core does not surface -Wformat for
# Serial.printf, so nothing caught it before flash.
#
# This script closes that gap. It runs at BUILD TIME only and emits no code,
# so it adds exactly zero flash, RAM and CPU cost to the firmware.
#
# WHAT IT CHECKS
#   1. conversion-specifier count == argument count
#   2. obvious type mismatches (%s given a numeric literal, %d/%f given a
#      string literal or a .c_str() expression)
#
# WHAT IT DELIBERATELY DOES NOT DO
#   It is not a C++ parser. Where it cannot resolve an argument's type with
#   confidence it stays silent rather than guessing -- a checker that cries
#   wolf gets switched off, and then it protects nothing. Count mismatches are
#   what actually shipped the crash, and those it detects exactly.
#
# USAGE
#   python printf_audit.py [path ...]     # default: this script's directory
#   exit 0 = clean, exit 1 = findings (fail the build gate)
# =============================================================================

import os
import re
import sys

CALL_RE = re.compile(
    r"\b(?:(\w+)\s*\.\s*)?(printf|snprintf|sprintf|vsnprintf)\s*\(", re.MULTILINE
)

# leading non-format arguments before the format string, per function
LEADING_ARGS = {"printf": 0, "snprintf": 2, "sprintf": 1, "vsnprintf": 2}

SPEC_RE = re.compile(r"%[-+ #0-9.*]*(?:hh|h|ll|l|L|q|j|z|t)*[diouxXeEfFgGaAcspn%]")


def strip_comments(src):
    """Blank out comments and keep byte offsets stable."""
    out = []
    i, n = 0, len(src)
    while i < n:
        c = src[i]
        if c == '"' or c == "'":
            q = c
            out.append(c)
            i += 1
            while i < n:
                if src[i] == "\\":
                    out.append(src[i : i + 2])
                    i += 2
                    continue
                out.append(src[i])
                if src[i] == q:
                    i += 1
                    break
                i += 1
            continue
        if src.startswith("//", i):
            j = src.find("\n", i)
            j = n if j < 0 else j
            out.append(" " * (j - i))
            i = j
            continue
        if src.startswith("/*", i):
            j = src.find("*/", i + 2)
            j = n if j < 0 else j + 2
            out.append("".join(ch if ch == "\n" else " " for ch in src[i:j]))
            i = j
            continue
        out.append(c)
        i += 1
    return "".join(out)


def split_args(argtext):
    """Split a call's argument list on top-level commas only."""
    args, depth, cur, i, n = [], 0, [], 0, len(argtext)
    while i < n:
        c = argtext[i]
        if c in '"\'':
            q = c
            cur.append(c)
            i += 1
            while i < n:
                if argtext[i] == "\\":
                    cur.append(argtext[i : i + 2])
                    i += 2
                    continue
                cur.append(argtext[i])
                if argtext[i] == q:
                    i += 1
                    break
                i += 1
            continue
        if c in "([{":
            depth += 1
        elif c in ")]}":
            depth -= 1
        if c == "," and depth == 0:
            args.append("".join(cur).strip())
            cur = []
            i += 1
            continue
        cur.append(c)
        i += 1
    tail = "".join(cur).strip()
    if tail:
        args.append(tail)
    return args


def literal_text(expr):
    """Concatenate adjacent string literals; None if not a pure literal."""
    parts = re.findall(r'"((?:[^"\\]|\\.)*)"', expr)
    if not parts:
        return None
    if re.sub(r'"(?:[^"\\]|\\.)*"', "", expr).strip():
        return None  # something other than literals in there
    return "".join(parts)


def count_specs(fmt):
    return [s for s in SPEC_RE.findall(fmt) if s != "%%"]


def type_conflict(spec, arg):
    """Report only unambiguous mismatches."""
    conv = spec[-1]
    a = arg.strip()
    is_str_lit = bool(re.fullmatch(r'"(?:[^"\\]|\\.)*"', a))
    looks_str = is_str_lit or ".c_str()" in a or a.endswith("Str") or "String(" in a
    is_num_lit = bool(re.fullmatch(r"[-+]?[0-9][0-9.eExXa-fA-F+-]*[uUlLfF]*", a))
    if conv == "s" and is_num_lit:
        return "%s given a numeric literal"
    if conv in "diouxX" and looks_str:
        return "%%%s given a string expression" % conv
    if conv in "eEfFgGaA" and looks_str:
        return "%%%s given a string expression" % conv
    return None


def audit_file(path):
    raw = open(path, encoding="utf-8", errors="replace").read()
    src = strip_comments(raw)
    findings = []
    for m in CALL_RE.finditer(src):
        recv, func = m.group(1), m.group(2)
        if func == "printf" and recv not in (None, "Serial", "Serial1", "Serial2"):
            pass  # still audited; a printf is a printf
        start = m.end()
        depth, i, n = 1, start, len(src)
        while i < n and depth:
            ch = src[i]
            if ch in '"\'':
                q = ch
                i += 1
                while i < n:
                    if src[i] == "\\":
                        i += 2
                        continue
                    if src[i] == q:
                        break
                    i += 1
            elif ch == "(":
                depth += 1
            elif ch == ")":
                depth -= 1
                if depth == 0:
                    break
            i += 1
        if depth:
            continue
        args = split_args(src[start:i])
        lead = LEADING_ARGS[func]
        if len(args) <= lead:
            continue
        fmt = literal_text(args[lead])
        if fmt is None:
            continue  # non-literal format: cannot audit statically
        specs = count_specs(fmt)
        actual = args[lead + 1 :]
        line = src.count("\n", 0, m.start()) + 1
        if len(specs) != len(actual):
            findings.append(
                (line, "COUNT", "%d specifier(s) [%s] vs %d argument(s)"
                 % (len(specs), " ".join(specs), len(actual)))
            )
            continue
        for k, (sp, ag) in enumerate(zip(specs, actual), 1):
            bad = type_conflict(sp, ag)
            if bad:
                findings.append((line, "TYPE", "arg %d: %s -- %s" % (k, bad, ag[:60])))
    return findings


def main(argv):
    roots = argv[1:] or [os.path.dirname(os.path.abspath(__file__))]
    files = []
    for r in roots:
        if os.path.isfile(r):
            files.append(r)
            continue
        for dirpath, dirnames, filenames in os.walk(r):
            dirnames[:] = [
                d for d in dirnames
                if not d.startswith(".")
                and not d.startswith("build")
                and not d.startswith("pre_")
                and d not in ("node_modules", "fifo_capture_analysis")
            ]
            for fn in filenames:
                if fn.endswith((".ino", ".cpp", ".h", ".hpp", ".c")):
                    files.append(os.path.join(dirpath, fn))

    total = 0
    for path in sorted(set(files)):
        found = audit_file(path)
        if found:
            print("%s:" % path)
            for line, kind, msg in found:
                print("  %s:%d  [%s] %s" % (os.path.basename(path), line, kind, msg))
            total += len(found)

    print("[printf_audit] scanned %d file(s), %d finding(s)" % (len(set(files)), total))
    return 1 if total else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
