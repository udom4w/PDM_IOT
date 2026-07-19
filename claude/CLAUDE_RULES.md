# CLAUDE_RULES.md

# Firmware Development Rules
Industrial IoT Predictive Maintenance
ESP32-S3 + FreeRTOS
Version: Phase 1

---

# 1. Primary Objective

Implement the approved design documents.

Do NOT redesign the architecture.

Architecture decisions are considered frozen unless explicitly changed by the project owner.

---

# 2. Architecture Status

Current status:

✅ Architecture Frozen

Architecture changes are NOT allowed.

If implementation cannot be completed without changing the architecture:

STOP.

Explain the issue.

Do not invent a new design.

---

# 3. Scope Control

Implement ONE design document at a time.

Example

DESIGN-0004

ONLY.

Never implement multiple DESIGN documents in one patch.

---

# 4. Minimal Patch Principle

Always create the smallest possible patch.

Avoid touching unrelated code.

Avoid formatting-only edits.

Avoid moving code.

Avoid renaming functions.

Avoid file restructuring.

---

# 5. Forbidden Actions

Unless explicitly requested, NEVER

- Refactor unrelated code
- Change task scheduling
- Change FreeRTOS priorities
- Change queue ownership
- Change MQTT architecture
- Change networking logic
- Change TLS logic
- Change Modbus protocol
- Change telemetry contract
- Change public interfaces
- Change configuration defaults
- Remove existing functionality

---

# 6. Allowed Actions

You MAY

- Add small helper functions
- Add new telemetry fields
- Add logging
- Add comments
- Add unit-test hooks
- Add validation
- Fix implementation bugs inside the requested scope

Only if required by the DESIGN document.

---

# 7. Preserve Existing Behaviour

The following must remain unchanged unless explicitly required.

- Alarm logic
- Fault latch
- MQTT topic structure
- Existing telemetry
- Sensor acquisition
- State machine
- Network behaviour
- OTA behaviour

Regression is unacceptable.

---

# 8. Build Requirements

Every implementation must

✔ Compile successfully

If compilation cannot be verified

State that clearly.

Never assume successful compilation.

---

# 9. Deliverables

Every implementation must provide

1. Summary

2. Files modified

3. Functions modified

4. Unified diff

5. Why each change was required

6. Regression risks

7. Manual test checklist

---

# 10. Risk Classification

Every change must include

🔴 Critical

🟠 High

🟡 Medium

🟢 Low

Explain why.

---

# 11. Regression Checklist

Check whether implementation changes

□ MQTT

□ Queue

□ Scheduler

□ Heap

□ Stack usage

□ CPU load

□ Sensor timing

□ Watchdog

□ Existing alarms

□ Existing telemetry

---

# 12. Performance

Avoid

- unnecessary RAM allocation
- dynamic allocation inside loops
- unnecessary String creation
- unnecessary task wakeups
- unnecessary copies

Prefer deterministic execution.

---

# 13. Error Handling

Never silently ignore errors.

Return status whenever practical.

Preserve existing error handling.

---

# 14. Design Authority

DESIGN documents are authoritative.

RFC documents explain design rationale.

Firmware must follow DESIGN.

If DESIGN and source code disagree

Report the conflict.

Do not choose automatically.

---

# 15. Implementation Philosophy

Implementation first.

Architecture second.

Do not improve code simply because it "looks better."

Only change code necessary for the requested DESIGN.

---

# 16. Stop Conditions

STOP immediately if implementation requires

- architecture redesign
- scheduler redesign
- queue redesign
- MQTT redesign
- breaking telemetry compatibility
- changing existing APIs

Explain the reason.

Do not continue.

---

# 17. Engineering Principles

Prefer

Correctness
over

Convenience

Prefer

Reliability
over

Optimization

Prefer

Small verified changes
over

Large intelligent changes

---

# 18. Definition of Done

A DESIGN implementation is complete only if

✓ Requested functionality implemented

✓ Clean compile

✓ Unified diff provided

✓ Regression checklist completed

✓ No architecture changes

✓ Existing behaviour preserved

Otherwise

The DESIGN is NOT complete.