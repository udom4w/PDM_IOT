# Release Notes — rc-v16.5.2-clean-fifo

## Changes
- Startup FAULT_LATCH holdoff — `MOTOR_RUN_FAULT_LATCH_HOLDOFF_READS` (default 4, ~1s @ 250ms) suppresses FAULT_LATCH creation for a short window immediately after STARTING->RUNNING, so a mechanical/vibration settling transient can't create a persistent fault record. STATE_WARNING/CRITICAL detection, buzzer, live telemetry, DEGLITCH, and the FIFO Broker are unaffected.
- Automatic BUILD_ID generation — `generate_build_info.ps1` now runs automatically via an arduino-cli/Arduino IDE pre-build hook (`platform.local.txt`), so `Git Commit`/`BUILD_ID` on the serial banner always reflect the source actually being compiled, with no manual step. Fails the build if `build_info.h` cannot be regenerated.
- Removed temporary COMMISSIONING auto-trigger — the Task 4.4 one-shot producer (self-armed ~45s after boot, no operator action) and the now-unused `FifoTriggerSource::COMMISSIONING` enum value are both fully removed, along with all associated logging.
- REMOTE_ON_DEMAND live-fire validation passed — a real MQTT command (`{"action":"capture","request_id":"..."}` on `factory/plant01/machine/pump01/vibration/command`) was published against the live broker and the complete path (MQTT receive -> trigger creation -> queue enqueue -> Broker -> `FifoDriver_Request()` -> real capture -> `FIFO-RESULT` -> `/event` publish) was confirmed end-to-end on hardware.

## Validation
- Build: PASS
- Flash: PASS
- Hash verify: PASS
- 100-second runtime: PASS
- REMOTE_ON_DEMAND MQTT capture: PASS
- No COMMISSIONING trigger remains
- No Task4.4 remains

## Known Technical Debt
- Sensor queue full log spam during FIFO ownership — while `FifoDriver` owns the RS485 bus for the duration of a capture, normal Modbus polling correctly backs off, but each blocked attempt logs `[CORE 0] Sensor queue full!` — functionally harmless (confirmed clean recovery to normal polling once every observed capture completed) but noisy (tens of thousands of lines during a single capture in the REMOTE_ON_DEMAND live-fire test).
- SCHEDULED trigger currently has no production producer — `FifoTriggerSource::SCHEDULED` still exists in the enum and in `fifoTriggerSourceStr()`, but no code path in the firmware ever sets it; the only reference is in `test/test_fifo_driver.cpp`. Left untouched as out of scope for this release — flagged here for future cleanup or wiring to a real producer.

## Git
- Branch: `recover-experimental`
- HEAD commit: `c2fe04d4e93bcae54b898b03b37a8f1ca24a860b` (`c2fe04d`) — refactor(fifo): remove temporary commissioning auto-trigger
- Previous tag: `rc-v16.5-current-evidence-fix`
- New tag: `rc-v16.5.2-clean-fifo`
