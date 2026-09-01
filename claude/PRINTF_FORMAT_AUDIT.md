# printf Format Audit — build gate step

## Why this step exists

Commit `2c351f0` removed three trailing arguments from the `/vibration` debug
log and left their conversion specifiers behind:

```c
Serial.printf("[MQTT] /vibration %d B | %s vel=%s mm/s rpm=%.1f "
              "state=%d | health=%d%% | fx=%s fy=%s fz=%s | "
              "cf=%s kurt_max=%.3f(%s) bear=%s\n",   // 13 specifiers
              jsonSize, alarmLevel, ... ,
              crestFactorOk ? String(crestFactor, 2).c_str() : "--");  // 10 args
```

`printf` read three varargs that were never pushed. The trailing `%s`
dereferenced a garbage pointer and the device panicked on **every** `/vibration`
publish:

```
Guru Meditation Error: Core 1 panic'ed (LoadProhibited)
A2 : 0x00000000
rst:0xc (RTC_SW_CPU_RST)
```

That is a crash-reboot loop on live hardware, and it reached the device because
**every existing gate passed**: clean build, **zero compiler warnings**, static
scans for the removed identifiers (all correctly gone), and binary string checks
(the removed keys were genuinely absent). None of them compares a format string
against its argument list — the ESP32 Arduino core does not surface `-Wformat`
diagnostics for `Serial.printf`.

Fixed forward in `fix(firmware): correct vibration publish debug format string`.

## The check

```
python claude/WTVB02_ESP32S3_V16_CM_fault_latch_v3_hardened_patched_v16_5/printf_audit.py \
       claude/WTVB02_ESP32S3_V16_CM_fault_latch_v3_hardened_patched_v16_5
```

* exit `0` — clean, proceed
* exit `1` — findings printed as `file:line [KIND] detail`; **fail the gate**

Covers `printf`, `Serial.printf`, `snprintf`, `sprintf`, `vsnprintf` across
`.ino/.cpp/.h/.hpp/.c`, handling multi-line calls and adjacent string-literal
concatenation. `build*/`, `pre_*/` and dot-directories are skipped.

Two finding kinds:

| Kind | Meaning |
|---|---|
| `COUNT` | specifier count != argument count — this is the class that shipped the crash |
| `TYPE` | unambiguous mismatch, e.g. `%d` handed a `.c_str()` expression |

## Cost

**Zero runtime cost.** The script runs at build time and emits no code — no
flash, no RAM, no CPU impact on the firmware.

## Deliberate limits

It is not a C++ parser. A format string that is not a plain literal (a variable,
a macro, a concatenation with a non-literal) is skipped rather than guessed at,
and `TYPE` only fires where the mismatch is unambiguous. A checker that produces
false positives gets switched off, and then it protects nothing. `COUNT`
mismatches — the failure that actually reached hardware — are detected exactly.

## Where it belongs in the gate

Run it **before** the clean build, alongside the existing static scans:

1. verify HEAD and source SHA256 against the git blob
2. **printf format audit — must exit 0**
3. clean build (must be 0 errors, 0 new warnings)
4. binary provenance: BUILD_ID references the commit
5. binary string checks for removed/retained keys

Step 2 is the one that would have caught `2c351f0` before it ever reached COM5.

## Regression test

The auditor is validated against the real defect. Run it against the
`pre_printf_fix_*` backup of the broken source and it must report:

```
...ino:9217  [COUNT] 13 specifier(s) [%d %s %s %.1f %d %d %s %s %s %s %.3f %s %s] vs 10 argument(s)
[printf_audit] scanned 1 file(s), 1 finding(s)   -> exit 1
```

and against the current sketch:

```
[printf_audit] scanned 34 file(s), 0 finding(s)  -> exit 0
```
