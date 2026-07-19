# WTVB02 ESP32S3 Firmware v16.5
## Engineering Handoff

## Background

This firmware exports vibration features from the WTVB02 sensor.
The sensor continues to output statistical features while the motor is stopped.
Those values are dominated by sensor noise floor and are not considered valid
for PdM.

The firmware already gates RMS, Peak, Frequency and other exported values using
motor_state == MOTOR_RUNNING.

A bug was found where Crest Factor related fields were not following this policy.

---

## Root Cause

CF-related values bypassed the RUNNING gate.

Affected paths:

1. publishTelemetry()
    crestFactor

2. MQTT JSON topic #1
    cf_x
    cf_y
    cf_z

3. MQTT JSON topic #2
    cf_x
    cf_y
    cf_z

4. Telemetry Ring Buffer
    cf_max

Because cf_max entered the ring buffer while STOPPED,
MQTT replay after reconnect could resend stale CF values.

---

## Fix (v16.5)

Added RUNNING gate to

- crestFactor
- cf_x
- cf_y
- cf_z
- cf_max (ring buffer)

Behavior is now consistent with

- RMS
- Peak
- Kurtosis
- Frequency

Policy:

motor_state != MOTOR_RUNNING
↓
Exported value = 0

---

## Engineering Rule (IMPORTANT)

Any feature originating from the sensor that is intended for machine-condition
analysis MUST follow the same export policy.

If

motor_state != MOTOR_RUNNING

then exported value shall be zero.

Examples include

- RMS
- Peak
- Crest Factor
- Kurtosis
- Frequency
- Future statistical features

Do not export raw statistical values while STOPPED.

---

## Before Declaring Any Future Patch Complete

Search every occurrence of the field.

Example

grep -Rn "cf_max"

or

grep -Rn "crest_factor"

Verify ALL locations are updated.

Do not assume fixing one publish path is sufficient.

Current firmware has

- MQTT Topic #1
- MQTT Topic #2
- Ring Buffer
- Replay Path

All must remain consistent.

---

## Build Requirement

This patch has only been text-edited.

Before merge:

- Compile with Arduino CLI or PlatformIO
- Verify no warnings/errors
- Use production library versions

ESP32-S3
TinyGSM
ModbusMaster
PubSubClient
ArduinoJson

---

## Regression Test

1.
Motor RUNNING
Verify
CF
CF_X
CF_Y
CF_Z
are reported normally.

2.
Motor STOPPED
Verify
CF = 0
CF_X = 0
CF_Y = 0
CF_Z = 0

3.
Disconnect MQTT
Reconnect
Verify replay also reports
CF = 0

4.
Grafana
CF trend shall immediately drop to zero after motor stops.

---

## Design Intent

The firmware intentionally exports machine-condition features only while
the machine is considered RUNNING.

This is an architectural policy, not merely a workaround for Crest Factor.

Future features shall follow the same policy.

---

## Follow-up Fix (v16.5.1)

Found during field-log review after the v16.5 patch.

Issue:

The `[MQTT] /vibration` Serial debug print statement used the raw,
ungated `data->rms_overall` value instead of the gated `reportedRms`
variable that is actually sent in `doc["rms"]`.

Effect:

Console log showed a nonzero RMS (e.g. `rms=0.61`) while `state=0`
(STOPPED), even though the real published MQTT payload correctly
reported 0. This was a debug-log-only discrepancy — not a data
integrity issue in the exported telemetry itself.

Fix:

Serial.printf now uses `reportedRms` so the console log matches
exactly what is published over MQTT.

Lesson: the same "raw vs. gated variable" mistake can hide in debug
prints, not just export paths. When verifying a gate policy fix,
check Serial/console prints too, not only the JSON payload builders.

