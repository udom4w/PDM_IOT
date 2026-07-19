# PDM IoT Platform Engineering Log

This document records engineering validation, technical investigations,
experimental results, and evidence-based conclusions.

Unlike CHANGELOG.md, this log does not record source code changes.
Its purpose is to preserve engineering knowledge and validation history.

---

## 2026-07-12

### EV-0001 — Peak Telemetry Pipeline Validation

Status: PASS

Objective

Verify that the firmware field `peak` (Overall Peak Velocity / Max VRMS Peak)
is successfully delivered from the ESP32 firmware to InfluxDB.

Firmware

Firmware:
WTVB02_ESP32S3_V16_CM_fault_latch_v3_hardened_patched_v16_5.ino

Production Baseline Commit:

4274e524ea3568e693c3fa1e5068b7ce118599fb

Evidence

1. Firmware MQTT publish

Confirmed that `/vibration` publishes:

```json
"peak": 4.58
```

2. MQTT Verification

Mosquitto subscriber confirmed the `peak` field is present
in every vibration payload.

Result:

PASS

3. Node-RED Pipeline

Confirmed Node-RED receives the `/vibration` topic and writes
the `peak` field into InfluxDB.

Result:

PASS

4. InfluxDB Verification

Field inventory:

```
peak
```

Flux query confirmed continuous stored samples:

```
4.58
4.53
4.76
...
4.80
...
```

Result:

PASS

Conclusion

The telemetry pipeline has been validated.

ESP32
→ MQTT
→ Node-RED
→ InfluxDB

The `peak` field is successfully preserved throughout the entire pipeline.

No firmware defect was found.

No MQTT pipeline defect was found.

Issue closed.

---