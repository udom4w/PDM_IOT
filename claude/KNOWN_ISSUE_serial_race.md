# KNOWN_ISSUE_serial_race.md

## Title

Serial logging race condition during RTOS task startup

## Status

Open
Priority: Low
Category: Debug Logging
Affects Production Data: No
Target Release: Future

---

## Summary

Boot log messages from multiple FreeRTOS tasks may interleave because
Serial is accessed concurrently from multiple cores without synchronization.

This affects log readability only.

Machine operation, sensor acquisition, MQTT telemetry,
analytics and alarm logic are NOT affected.

---

## Root Cause

ESP32 FreeRTOS tasks created by

xTaskCreatePinnedToCore()

may begin execution immediately.

Some tasks print startup messages while setup() is still printing.

Serial has no synchronization mechanism.

Multiple cores therefore write to UART simultaneously.

---

## Current Impact

Observed:

- Mixed boot log lines
- Startup messages appearing later than expected
- Difficult-to-read debug output

Not observed:

- Sensor corruption
- MQTT corruption
- State machine errors
- Analytics errors
- Memory corruption

---

## Current Decision

No fix in v16.5.

Reason:

v16.5 is dedicated to Crest Factor export policy.
Keeping one functional change per release reduces regression risk.

---

## Candidate Solutions

Option A
Introduce Serial mutex wrapper.
Preferred long-term solution.

Option B
Remove duplicate "task started" messages from worker tasks.
Lowest risk.

Option C
Replace Serial calls with SerialSafePrint()
Future refactoring.

---

## Acceptance Criteria

Future implementation shall ensure

- No interleaved boot log
- No deadlock
- No blocking impact on real-time tasks
- No performance regression

---

## Verification

Repeated cold boot
100+ power cycles

Verify startup log ordering.
Verify no task starvation.
Verify watchdog remains clean.
