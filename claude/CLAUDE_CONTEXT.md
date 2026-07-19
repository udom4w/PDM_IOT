# CLAUDE_CONTEXT.md

# Industrial IoT Predictive Maintenance Firmware
## Project Context

---

# Project Overview

This project develops a production-grade Industrial IoT Edge Device for
Predictive Maintenance (PdM).

Target platform

- ESP32-S3
- FreeRTOS
- RS485 Modbus
- MQTT over TLS
- 4G LTE
- InfluxDB
- Grafana

The device continuously monitors industrial rotating machinery and publishes
validated telemetry to the cloud.

---

# Current Project Status

Current Phase

Phase 1

Current Firmware

v16.4 Production Baseline

Architecture Status

Architecture Frozen

Current development focuses on controlled implementation of approved DESIGN
documents.

---

# High-Level Architecture

Sensor

↓

Modbus Acquisition

↓

Validation

↓

Analytics

↓

Decision Engine

↓

Telemetry

↓

MQTT

↓

Cloud

Every subsystem has a clearly defined responsibility.

Cross-layer shortcuts are discouraged.

---

# System Principles

The firmware follows these principles.

Single Responsibility

One task should have one primary responsibility.

Deterministic Execution

Avoid unpredictable execution time.

Minimal Coupling

Subsystems communicate through defined interfaces.

Fail Safe

Failures should not silently propagate.

Backward Compatibility

Existing production behaviour must be preserved.

---

# Task Overview

Core 0

- Sensor acquisition
- Modbus communication
- State machine

Core 1

- Analytics
- MQTT
- Display
- UI
- Network

Task ownership must remain clear.

---

# Data Flow

WTVB02 Sensor

↓

Modbus

↓

Validation

↓

Analytics

↓

Decision

↓

Telemetry Buffer

↓

MQTT Publish

↓

Cloud

Every processing stage has exactly one responsibility.

---

# MQTT Philosophy

MQTT is the transport layer.

Analytics does NOT directly manage MQTT.

MQTT ownership is centralized.

No hidden publishers.

No duplicated ownership.

---

# Telemetry Philosophy

Telemetry should represent measured facts.

Avoid derived values unless explicitly required.

Existing MQTT contracts must remain backward compatible.

---

# Sensor Philosophy

The sensor is treated as the source of truth.

Firmware validates sensor outputs but should not invent measurements.

Validation occurs before analytics.

---

# Analytics Philosophy

Analytics never modifies raw measurements.

Analytics derives additional information.

Raw data must remain recoverable.

---

# Alarm Philosophy

Alarm generation must remain deterministic.

Alarm logic should not depend on timing coincidence.

Fault latch behaviour must remain unchanged unless explicitly redesigned.

---

# Queue Philosophy

Queues define ownership boundaries.

One producer.

One consumer.

Ownership must be obvious.

Queue redesign requires explicit approval.

---

# Scheduler Philosophy

Task priorities are intentional.

Avoid changing

- priorities
- affinity
- wake-up intervals

unless required by a DESIGN document.

---

# Memory Philosophy

Prefer static allocation.

Avoid dynamic allocation inside periodic execution.

Avoid unnecessary copies.

Heap stability is more important than micro-optimizations.

---

# Performance Philosophy

Prefer

Predictable execution

over

Maximum throughput.

Low jitter is preferred.

---

# Existing Design Documents

DESIGN documents define implementation.

RFC documents explain rationale.

Firmware follows DESIGN.

RFC is informative.

---

# Current DESIGN Roadmap

DESIGN-0001

Trigger Manager

DESIGN-0002

FIFO Task

DESIGN-0003

PhysicalInvariant()

DESIGN-0004

Peak Telemetry

Only one DESIGN should be implemented per patch.

---

# Current Engineering Workflow

1. Review DESIGN

↓

2. Approve implementation

↓

3. Small implementation patch

↓

4. Compile

↓

5. Regression testing

↓

6. Code review

↓

7. Merge

No step should be skipped.

---

# Definition of Production Ready

A feature is considered production ready only if

- Clean compile
- Manual testing completed
- Regression tests passed
- Existing behaviour preserved
- Architecture unchanged
- Documentation updated

---

# Project Goal

Build a reliable, maintainable, production-grade Edge Predictive Maintenance
platform suitable for continuous industrial operation.

Reliability is more important than feature count.

Engineering discipline is more important than implementation speed.