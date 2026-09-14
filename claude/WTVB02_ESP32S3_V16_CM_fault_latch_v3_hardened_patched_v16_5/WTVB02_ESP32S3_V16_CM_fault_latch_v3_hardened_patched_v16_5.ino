/*
 * ============================================================================
 * Industrial Condition Monitoring System - Dual Core FreeRTOS
 * ============================================================================
 * Hardware: LilyGO T-Vending S3 (ESP32-S3, 16MB Flash)
 * Modem: SIMCom A7670 (SIM7600 Compatible) - 4G LTE
 * RTC: DS3231
 *
 * Version: 16.5.4 (condition_monitoring_v1) [patched v16.5 -- CF motor_state gate]
 *
 * ============================================================================
 * v16.5.4 -- Architectural hardening (3 surgical improvements, no new features)
 * ============================================================================
 * 1. RPM EMA invalidation after long idle (MAX_EMA_INTERVAL_US): g_rpmFiltered
 *    is no longer a never-reset EMA -- a pulse gap (or no-pulse idle) longer
 *    than MAX_EMA_INTERVAL_US invalidates the EMA and zeroes g_rpmFiltered;
 *    the first valid pulse afterwards reseeds the EMA from its raw RPM
 *    (clean restart, no blending with the stale value).
 * 2. Atomic telemetry snapshot (TelemetrySnapshot / g_telemSnapshot): captured
 *    exactly once per state-machine cycle on Core 0, after
 *    updateMotorStateMachine() and the alarm/health evaluation have completely
 *    finished. All downstream consumers (MQTT publish, telemetry ring buffer,
 *    fault-latch replay, OLED, Analytics, 30s status log) read this one
 *    mutex-guarded struct instead of assembling their own view from
 *    g_vibData + g_systemState in separate mutex takes.
 * 3. Explicit RPM evidence freshness (RPMEvidence): rpm/valid/ageMs tracked at
 *    the EMA update site; buildMotorStateEvidence() derives signalPresent from
 *    it, so a stale EMA can never report signalPresent=true. FSM thresholds
 *    and STOPPING/STOPPED timing (evidence.ageMs = timeSincePulseMs) unchanged.
 * ============================================================================
 *
 * ============================================================================
 * v16.5.3-rpmdiag1 -- DIAGNOSTIC BUILD ONLY
 * ============================================================================
 * Adds read-only Serial logging ([MOTOR-DIAG]/[MOTOR-TRANSITION]/[SIGNAL]/
 * [PULSE]) around the RPM-sourced motor state machine, to investigate why
 * the state machine reaches STOPPING/STOPPED while current stays ~1.6A and
 * the motor is physically running. No thresholds, timing, control flow, or
 * any existing computation were changed -- logging only. Safe to flash for
 * diagnosis and revert afterward.
 * ============================================================================
 *
 * v16.5 Changes (CF garbage-while-stopped fix):
 * +--------------------------------------------------------------------+
 * | ROOT CAUSE: crest_factor / cf_x / cf_y / cf_z were the only        |
 * | vibration fields NOT gated by motor_state==RUNNING, unlike         |
 * | rms/peak/kurtosis which all zero out when not RUNNING. Sensor CF   |
 * | registers report noise-floor/garbage while STOPPED (unfiltered,    |
 * | same deglitch-only-runs-when-RUNNING issue as v16.4).              |
 * |                                                                    |
 * | FIX (4 locations):                                                 |
 * |  1. publishTelemetry(): crestFactor now requires motor_state==2    |
 * |  2. s["cf_x"/"cf_y"/"cf_z"] (PUB-1 /sensor) gated                  |
 * |  3. doc["cf_x"/"cf_y"/"cf_z"] (PUB-3 /vibration) gated             |
 * |  4. pushTelemBuf(): s->cf_max gated -- prevents stale CF garbage   |
 * |     from being replayed over MQTT after reconnect                 |
 * +--------------------------------------------------------------------+
 *
 * v16.5.1 (follow-up): [MQTT] /vibration Serial debug print used raw
 * data->rms_overall instead of the gated reportedRms -- console log
 * showed nonzero RMS while STOPPED even though the actual published
 * doc["rms"] was correctly 0. Fixed to print reportedRms so debug log
 * matches what is actually sent over MQTT.
 *
 * v16.4 Changes (Priority 1 root cause fix):
 * +--------------------------------------------------------------------+
 * | ROOT CAUSE CONFIRMED: v16.3x deglitch only caught RMS DROP         |
 * | glitches, not RMS SPIKE glitches. Spikes flowed straight through   |
 * | to the WARNING_RMS/CRITICAL_RMS comparison (unfiltered), causing   |
 * | false STATE_WARNING/CRITICAL + buzzer activation.                  |
 * |                                                                    |
 * | Field-test evidence (WTVB02_VelocityComparison.ino log, 2026-07-03)|
 * | captured 2 live spike events: VRMS jumped to abnormal values while |
 * | freq_x=freq_y=freq_z=0.0 Hz on every occurrence, then recovered to |
 * | baseline on the very next sample (single-sample transient).        |
 * |                                                                    |
 * | FIX: extended the existing single-sample deglitch block to also    |
 * | hold-and-suppress spike glitches, keyed on the physical invariant  |
 * | that real running-motor vibration always has nonzero dominant      |
 * | frequency -- freq==0 on all 3 axes combined with an RMS deviation  |
 * | (either direction) is now treated as a glitch, same 1-sample hold  |
 * | policy as the existing drop-glitch guard.                          |
 * +--------------------------------------------------------------------+
 *
 * v16.0 Changes (condition_monitoring_v1):
 * +--------------------------------------------------------------------+
 * | Simplified from v15.8 for Condition Monitoring Product V1          |
 * | REMOVED: Fault Detection & Prediction engine                       |
 * |   - classifyFaultProbabilistic() (Gaussian Bayesian classifier)    |
 * |   - runDecisionEngine() (AEDF + OSG + FVRI + TTW pipeline)         |
 * |   - FaultType_t / AlarmClass_t enums and DecisionResult_t          |
 * |   - PUB-B..G (/fusion/*, /ttw/model, /output)                      |
 * |                                                                    |
 * | KEPT: Full Condition Monitoring stack                               |
 * |   - Modbus RTU: RMS/Temp/Freq/CF/Kurtosis (3 axes) from WTVB02    |
 * |   - RPM / Motor state machine                                      |
 * |   - Multi-resolution trend buffers (buf1s/10s/60s) + EMA           |
 * |   - bearing_alert from Kurtosis, freq_alert, health_score          |
 * |   - PUB-1 /sensor, PUB-2 /status, PUB-3 /vibration, PUB-A /trend  |
 * |   - mTLS MQTT, 4G, RTC NTP, OLED, Button, Buzzer                  |
 * +--------------------------------------------------------------------+
 *
 * Based on v15.8 by Senior Real-Time Embedded Engineer
 * ============================================================================
 */

// ============================================================================
// MODEM CONFIGURATION (Must be before TinyGSM include)
// ============================================================================
#define TINY_GSM_MODEM_SIM7600
#define TINY_GSM_RX_BUFFER 4096

// ============================================================================
// [Phase 13] MQTT PUBLISH TIMING INSTRUMENTATION -- diagnostic build only.
//
// Purpose: measure where the ~1000ms lwmqtt command-timer budget is consumed
// during a QoS-1 publish, to confirm or refute the err=-4
// (LWMQTT_NETWORK_TIMEOUT) hypothesis. Adds ONLY timestamps + Serial output.
// No firmware logic, timeout value, reconnect behavior, TinyGSM behavior, or
// MQTT behavior is altered by anything under this macro.
//
// MUST be defined before <TinyGsmClient.h> below: the TinyGSM headers are
// textually #include-d into this translation unit, so modemSend()'s timing
// block in TinyGsmClientSIM7600.h is compiled in only if this is already set.
//
// COMMENT OUT BOTH THIS AND LWMQTT_DEBUG_TIMING (in
// .arduino/libraries/MQTT/src/lwmqtt/client.c) TO RETURN TO A PRODUCTION BUILD.
// ============================================================================
// #define DEBUG_MQTT_TIMING   // [production-clean] disabled -- see V16_5C_VERIFIED.md

// ============================================================================
// [Phase 15A] RECEIVE-PATH TIME ATTRIBUTION -- OFF by default.
//
// Purpose: locate where the ~1000 ms between "broker response reaches the
// modem" and "ESP32 returns the byte to lwmqtt" is actually spent. Wire
// evidence (mqtt_phase14.pcap) proved the broker answers in 198us-8ms with no
// retransmission, no loss and no zero-window, so the missing time is entirely
// inside the ESP32<->modem retrieval path.
//
// Three probes only (Phase 15A scope; 15B NOT implemented):
//   P1  modemGetAvailable()      TinyGsmClientSIM7600.h -- AT+CIPRXGET=4 cadence
//   P3  _bio_recv()              this file              -- blind-poll burst cost
//   P5  handleURCs()             TinyGsmClientSIM7600.h -- URC push timing
//
// MUST be defined before <TinyGsmClient.h> below: the TinyGSM headers are
// textually #include-d into this translation unit, so P1/P5 compile in only
// if this is already set.
//
// Observational only: no AT command added, no control flow, return path,
// timeout or reconnect behaviour altered. With this macro undefined every
// probe compiles to nothing and the binary is byte-identical to production.
// TO ENABLE: uncomment the single line below.
// ============================================================================
// #define DEBUG_RXPATH   // [production-clean] disabled -- see V16_5C_VERIFIED.md

// ============================================================================
// [v16.5b] ISSUE #2 PHASE 1 -- READ-ONLY A7670E MODEM DIAGNOSTICS
// ============================================================================
// Issue #1 proved the stall lies downstream of the modem's AT-level CIPSEND
// acceptance: the modem confirms the bytes, then emits no IP packet for up to
// 1.68 s (IP-ID advanced by exactly +1 across the stall). Nothing on the host
// side can see further. This probe samples the modem's own view of the radio
// once per minute so that uplink delay can be correlated against serving cell,
// signal quality, registration state and any power-saving configuration.
//
// STRICTLY READ-ONLY: every command below is a query form (+CPSI?, +CSQ,
// +CEREG?, +CPSMS?, +CEDRXS?). No "=" set form appears anywhere, so modem
// configuration cannot be altered. No MQTT, TinyGSM, reconnect or timeout
// behaviour is touched. With DEBUG_MODEM_DIAG undefined the whole feature
// compiles to nothing.
//
// SCOPE LIMIT (deliberate, per Issue #2 decision): the quiet-window gate can
// only fire when publishInterval >= 2 * MODEM_DIAG_QUIET_MS, i.e. in the
// NORMAL 30 s cadence only. WARNING (10 s) and CRITICAL (5 s) never sample.
// The investigation targets the NORMAL interval; no fallback is provided.
// ============================================================================
// #define DEBUG_MODEM_DIAG   // [production-clean] disabled -- see V16_5C_VERIFIED.md
#define MODEM_DIAG_PERIOD_MS  60000UL   // sample cadence
#define MODEM_DIAG_QUIET_MS   10000UL   // clearance from both publish edges
#define MODEM_DIAG_AT_TMO_MS   1000UL   // per-command ceiling (worst case 5 x)

#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <TinyGsmClient.h>
#include <MQTT.h>  // joel-gaehwiler/MQTT (arduino-mqtt) -- supports QoS 0/1/2
#include <ModbusMaster.h>
#include <ArduinoJson.h>
#include <RTClib.h>
#include <Preferences.h>   // NVS Flash -- runtime_hour persistence

// [Task 4.1] FIFO waveform driver -- included near the top, alongside the
// other library includes, matching CM-100_FIFO_Implementation_Plan_v1.0.md's
// own prescribed Task 4.1 placement ("near the top... avoids the .ino
// auto-prototype forward-declaration problem"). No new type is used as a
// return value or parameter of any function defined directly in this .ino
// file, so no typedef-before-first-use step is needed here.
#include "fifo_types.h"
#include "fifo_transport_uart485.h"
#include "fifo_driver.h"
#include "vib_accel.h"   // [Phase 3B] FIFO RAW -> acceleration RMS (Arduino-free)
#include "vib_velocity.h"  // [Phase 3C] FIFO RAW -> velocity RMS, frequency domain
#include "vib_history.h"   // [M1B-1] timestamped velocity trend history ring
#include "vib_ema.h"       // [M1B-2] timestamp-aware EMA over the history ring
#include "vib_window.h"    // [M1B-3] time-based trend windows over the history ring
#include "vib_slope.h"     // [M1B-4] timestamp-aware velocity slope (mm/s per s)
#include "vib_ttw.h"       // [M1B-5] Time-To-Warning from velocity + slope

// [v16.6 logging refactor] Centralized logging framework -- see log.h.
// Included near the top for the same .ino auto-prototype reason as the
// FIFO headers above: LOGE/LOGW/LOGI/LOGD/LOGT are plain macros (no
// return-value/parameter type), so this placement is not load-bearing for
// that specific hazard, but keeping all local module headers grouped
// together matches this file's existing convention.
#include "log.h"

///////////////////////////////////////////////////////////////////////////////
// COMMISSIONING CONFIGURATION
// Site-specific parameters.
// These are expected to change during installation and commissioning.
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
// Site Identity
///////////////////////////////////////////////////////////////////////////////
#define PLANT_ID "plant01"   // Plant / Site identity
#define MACHINE_ID "pump01"  // Machine identity (tag-level)
#define SENSOR_ID "vb01"     // Sensor identity

///////////////////////////////////////////////////////////////////////////////
// Motor Configuration
///////////////////////////////////////////////////////////////////////////////
#define NAMEPLATE_RPM 1800   // Motor nameplate RPM (used as RATED_RPM reference)

// Consumed ENTIRELY inside buildMotorStateEvidence()'s MOTOR_SRC_CURRENT
// branch; updateMotorStateMachine() never sees any of these values.
// No real site values known yet -- placeholders until commissioning data
// (motor nameplate FLA, installed CT ratio/turns) is available.
//
// [P2] MOTOR_NAMEPLATE_CURRENT_A is a COMMISSIONING PARAMETER, not a
// validated engineering value. 2.0f is the pre-existing placeholder carried
// forward unchanged -- it must be replaced with this specific motor's real
// nameplate Full-Load Amps before MOTOR_SRC_CURRENT is trusted beyond bench
// testing. CURRENT_ON_THRESHOLD_A / CURRENT_OFF_THRESHOLD_A are derived from
// it, so commissioning this one constant recalibrates both automatically.
constexpr float MOTOR_NAMEPLATE_CURRENT_A = 1.2f;   // [A] nameplate FLA -- commissioned per P3-02 (ABB M2BAX 71MA2, 220V/50Hz/Delta)

// [P2] Hysteresis pair on the EMA-filtered engineering current (s_currentFiltered
// in buildMotorStateEvidence()'s MOTOR_SRC_CURRENT branch): signalPresent
// latches true at/above ON and stays true until current drops below OFF --
// the 8-point-of-FLA gap is the dead band that stops a single threshold from
// chattering the state machine near one static value.
constexpr float CURRENT_ON_THRESHOLD_A  = MOTOR_NAMEPLATE_CURRENT_A * 0.20f;   // [A] 20% FLA -- signal "on"
constexpr float CURRENT_OFF_THRESHOLD_A = MOTOR_NAMEPLATE_CURRENT_A * 0.12f;   // [A] 12% FLA -- signal "off"

// [P2] Source-specific STOPPING/STOPPED absence timing for MOTOR_SRC_CURRENT,
// carried into updateMotorStateMachine() via MotorStateEvidence.absentStoppingMs/
// absentStoppedMs (see buildMotorStateEvidence()). Deliberately separate from
// ABSENT_STOPPING_MS/ABSENT_STOPPED_MS below, which remain RPM's values.
constexpr uint32_t NO_CURRENT_STOPPING_MS   = 1500;   // [ms] continuous below-OFF before RUNNING->STOPPING
constexpr uint32_t FORCE_CURRENT_STOPPED_MS = 5000;   // [ms] continuous below-OFF before STOPPING->STOPPED

///////////////////////////////////////////////////////////////////////////////
// Current Sensor Configuration
///////////////////////////////////////////////////////////////////////////////
#define CT_RATIO_PRIMARY_A          1.0f    // [A] external CT ratio primary -- 1:1 if no external CT
#define CT_RATIO_SECONDARY_A        1.0f    // [A] external CT ratio secondary
#define CT_TURNS                    1       // [turns] times the conductor loops through the CT clamp

///////////////////////////////////////////////////////////////////////////////
// Debug / Test Configuration
///////////////////////////////////////////////////////////////////////////////
// [Commit 4B] TEST_CURRENT_SOURCE -- bench-test-only build flag. Undefined
// by default: production builds are unaffected, this branch does not exist
// in the translation unit at all. Define via -DTEST_CURRENT_SOURCE to select
// MOTOR_SRC_CURRENT for bench testing. No runtime branch either way -- the
// preprocessor resolves this before compilation.
#define TEST_CURRENT_SOURCE   // โหมดกระแส

// [v16.6f] Diagnostic-only, bench-test build flag -- proves empirically
// whether CTR4A01's raw reading is already True Line Current, before any
// Phase-2 compensation is implemented. Prints once/second, gated entirely by
// this #ifdef -- zero cost and no behavior change when undefined. Undefine
// for production builds.
#define DEBUG_CURRENT_PATH

// [Phase 7A] Debug-only FIFO raw waveform CSV dump over Serial. Undefined by
// default: production builds are unaffected, DumpFifoCaptureCsv() and its
// one call site (handleFifoCaptureCompletion(), post-capture only) do not
// exist in the translation unit at all. Define via -DDEBUG_FIFO_DUMP (or
// uncomment below) to enable. Egress mechanism only -- does not touch MQTT
// payload construction or FIFO protocol/state-machine timing (the dump
// fires strictly after S11_RESULT_READY has already been reached and
// acquired; the capture itself is over by the time this runs).
// [Phase 2B TEST-ONLY] DISABLED for the periodic-capture hardware validation
// build. The dump is 1024 Serial.printf() calls (~1-2 s at this line rate) and
// runs inside handleFifoCaptureCompletion(), i.e. ON CORE 0 inside
// taskModbusRead() -- so with the 2000 ms SCHEDULED cadence it would block the
// Modbus/CTR4A01 poll loop for most of every period and make the measured
// capture interval / jitter a measurement of the DUMP rather than of the FIFO
// capture. Re-enable (uncomment) whenever the raw waveform is needed again;
// DumpFifoCaptureCsv() and the FIFO RAW capture path itself are UNCHANGED and
// still present -- only this egress switch is off.
// #define DEBUG_FIFO_DUMP

// ============================================================================
// [BUILD FINGERPRINT] Firmware identity -- printed once at boot in setup(),
// zero runtime cost afterward. See BUILD_FINGERPRINT.md for the full design
// and the companion generate_build_info.ps1 script that refreshes
// GIT_COMMIT_HASH before each compile.
// ============================================================================
#include "build_info.h"   // provides GIT_COMMIT_HASH; safe default "UNKNOWN" if never regenerated

#ifndef GIT_COMMIT_HASH
#define GIT_COMMIT_HASH "UNKNOWN"   // build_info.h missing/didn't define it -- never fabricate a hash
#endif

#define FW_VERSION "16.5"   // single source of truth for the firmware version string

// ============================================================================
// VERIFICATION INSTRUMENTATION (Checkpoint 1 -- disabled by default)
// ============================================================================
// [VERIFY_TEST] RAM-only causal-proof capture for poll_seq / deglitch forensics.
// Off in production builds; no runtime behavior change when undefined.


// ============================================================================
// HARDWARE CONFIGURATION
// ============================================================================

// --- Pin Definitions (LilyGO T-Vending S3) ---
#define PIN_BUTTON 5        // SELECT button (page navigation)
#define PIN_BUTTON_ENTER 6  // ENTER button (alarm acknowledge)
#define PIN_BUZZER 7        // Buzzer output

// [Production Trigger, Commit 6] ENTER double-click window for the
// OPERATOR_BUTTON FIFO trigger. Recommended 350-400ms range; 400ms chosen
// for slack on physical hardware. Measured click-release-to-click-release,
// not press-to-press (see taskButtonHandler()'s own comment for why).
#define ENTER_DOUBLECLICK_WINDOW_MS 400

// --- RS485 Pins ---
#define RS485_RX_PIN 38
#define RS485_TX_PIN 39
#define RS485_EN_PIN 42

// [Task 4.5 -- TEMPORARY DIAGNOSTIC ONLY, EN-ownership-hypothesis
// verification, not a permanent production feature] Last logged RS485_EN_PIN
// level -- 0xFF means "never logged yet" (distinct from any real digitalWrite
// value) so the very first rs485Enable()/rs485Disable() call of the run is
// always logged as a transition. Read/written only from rs485Enable()/
// rs485Disable() themselves (Core 0, taskModbusRead's own call chain plus
// setup(), both single-threaded with respect to this pin). Intended to be
// removed once this investigation concludes.
static uint8_t s_lastEnPinLoggedState = 0xFF;

// --- I2C Pins ---
#define I2C_SDA_PIN 44
#define I2C_SCL_PIN 43

// --- 4G Modem Pins (SIMCom A7670) ---
#define MODEM_TX 3
#define MODEM_RX 46
#define MODEM_POWER_ON 4
#define MODEM_RESET_PIN 9
#define MODEM_RESET_LEVEL HIGH
#define BUILTIN_LED 10
#define PIN_LED_CLOUD 15    // [Enclosure v1] Cloud LED - reflects modemState + MQTT connection

// --- Display Configuration ---
#define OLED_WIDTH 128
#define OLED_HEIGHT 64
#define OLED_ADDRESS 0x3C

// --- Modbus Configuration ---
// [Phase 0 / rs485-115200-validation] 9600 -> 115200. Single source of truth
// for the shared RS485 bus: consumed only by SerialRS485.begin() in setup()
// and by the boot banner. BOTH bus devices must already be reconfigured to
// 115200 (WTVB02 BAUD reg 0x04 = 0x06; CTR4A01 reg 0x00FE = 0x07) BEFORE
// flashing this, or all Modbus traffic fails -- see the Phase 0 procedure.
#define MODBUS_BAUDRATE 115200
#define MODBUS_SLAVE_ID 0x50

// --- CTR4A01 Current Sensor (shared RS485 bus, multi-drop Modbus) [v16.6a] ---
// Register map reused verbatim from experimental/CTR4A01_SENSOR/CTR4A01_SENSOR.ino
#define CURRENT_SENSOR_ID   0x01     // CTR4A01 slave address
#define CT_REG_AC_CURRENT   0x0000   // function 04 (input register), unit mA (0-5000 = 0-5A)
// §6.4.14-16: Velocity RMS (True RMS, ÷1000 → mm/s)
// เปลี่ยนจาก VX/VY/VZ (0x3A Peak ÷100) → VRMSX/Y/Z (True RMS ÷1000)
// ต้องตั้ง DRM=0x02 (Frequency domain) เพื่อให้ค่าถูกต้อง
// [M1A DEPRECATED] ==========================================================
// These three registers are NO LONGER the Product Phase-1 vibration alarm
// source. As of M1A the alarm state machine, health score and fault latch all
// read velocity_rms_overall produced by FIFO RAW -> DSP (see g_velCarrier).
//
// They are retained ONLY for:
//   - backward-compatible telemetry (/sensor rms,vx,vy,vz,peak,peak_velocity_*
//     and /vibration rms) -- deprecated, removal no earlier than Phase 5
//   - the trend/EMA/TTW engine, which still consumes them until M1B (their
//     cadence changes 4 Hz -> ~0.5 Hz, so that migration is deliberately a
//     separate change -- see M1B dependency list)
//   - VERIFY_TEST diagnostics
//
// Do NOT add a new Product decision that reads these values or anything
// derived from them (vel_peak_*, rms_x/y/z, rms_overall).
// Removal plan: Phase 5 cleanup -- see M1A review section 10.
// ===========================================================================
#define REG_VRMS_X 0x50  // VRMSX: X-axis velocity RMS (mm/s) §6.4.14  [DEPRECATED]
#define REG_VRMS_Y 0x5C  // VRMSY: Y-axis velocity RMS (mm/s) §6.4.15  [DEPRECATED]
#define REG_VRMS_Z 0x68  // VRMSZ: Z-axis velocity RMS (mm/s) §6.4.16  [DEPRECATED]
#define REG_TEMPERATURE 0x40
#define REG_FREQ_X 0x44  // Frequency X,Y,Z (0x44~0x46) per WTVB02 manual
#define REG_CFX    0x47  // CFX=Accel Crest Factor X, KX=Kurtosis X (0x47~0x48) §6.4.14
                         // CFY=0x53, CFZ=0x5F (ไม่ต่อเนื่อง -- อ่านแยก transaction ถ้าต้องการ)
#define REG_CFY    0x53  // CFY=Accel Crest Factor Y, KY=Kurtosis Y (0x53~0x54) §6.4.15
#define REG_CFZ    0x5F  // CFZ=Accel Crest Factor Z, KZ=Kurtosis Z (0x5F~0x60) §6.4.16
// [Phase2] REG_PEAK_X (0x3A) REMOVED -- no reader left after peak_velocity_*.

// --- Sensor Re-config Registers (v15.7) ---
// ใช้หลัง restartSensorViaModbus() เพื่อ restore config ที่อาจกลับเป็น default
#define REG_UNLOCK        0x0069  // Password/Unlock register
#define REG_CMD           0x0000  // Command register (Restart / Save)
#define REG_MODE          0x0007  // Algorithm mode register
#define REG_SAMPLE_RATE   0x0029  // Sample rate register
#define SENSOR_UNLOCK_KEY 0xB588  // Unlock password
#define SENSOR_SR_16K     0x0001  // Sample Rate = 16 kHz
// [PD-0001] SR6=512Hz was the Phase 1 experimental baseline (WTVB05_FIFO_Investigation_Report.md).
// [PD-0003] SR5=1kHz superseded PD-0001 above.
// [PD-0004] Switched production to SR4=2kHz (superseded by PD-0005 below).
// [PD-0005] Reverted production to SR5=1kHz for Phase 1 validation.
// [PD-0006] Production SR is now SR4=2kHz, and -- unlike PD-0001..PD-0005 --
// it is selected in exactly ONE place: SENSOR_SR_PRODUCTION below. Previously
// the rate appeared three times independently (the REG_SAMPLE_RATE write, the
// pre-write Serial label, and the read-back verify constant), so changing it
// meant editing three sites in lockstep; a partial edit silently produced a
// fail-closed verify against a rate the firmware no longer wrote. The
// SENSOR_SR_* table below stays as the raw register-value dictionary
// (WTVB02-485 manual §6.4.12); SENSOR_SR_PRODUCTION is the policy.
#define SENSOR_SR_512     0x0006  // Sample Rate = 512 Hz (SR6)
#define SENSOR_SR_1K      0x0005  // Sample Rate = 1 kHz (SR5)
#define SENSOR_SR_2K      0x0004  // Sample Rate = 2 kHz (SR4)

// [PD-0006] SINGLE SOURCE OF TRUTH for the production sample rate. Written to
// REG_SAMPLE_RATE and verified against the read-back -- write and verify can
// no longer disagree. Change this one line to change the production SR.
// SR4 = 2 kHz -> measurable frequency 8~1000 Hz (manual §6.4.12); FIFO Nyquist
// 1000 Hz, dF = 2000/1024 = 1.953125 Hz.
#define SENSOR_SR_PRODUCTION  SENSOR_SR_2K

// ----------------------------------------------------------------------------
// [Phase 3A] VERIFIED SAMPLE-RATE PROVENANCE
//
// Before this, the ONLY evidence of the sensor's actual rate was a local
// variable inside reconfigSensorAfterRestart() that was compared once and then
// discarded, so FifoCaptureResult::srHz/srIndexAtCapture -- declared, and
// published on /event -- were never written and always read 0. Any DSP built
// on that would silently assume a rate it had never observed; a sensor running
// SR5 while firmware assumed SR4 would make every velocity figure wrong by 2x
// with no error anywhere.
//
// These two globals hold the rate PROVEN BY READ-BACK, not the rate we asked
// for. They are set ONLY when the read-back both decodes to a known SR index
// AND matches SENSOR_SR_PRODUCTION, and are cleared to UNKNOWN on every entry
// to the config sequence and on any failure -- fail-closed by construction.
//
// SCOPE LIMITATION (documented, not worked around): reconfigSensorAfterRestart()
// is the firmware's only SR read-back, and it runs at boot and after a sensor
// restart -- NOT per capture. So provenance is "verified at last successful
// sensor configuration", not "verified at this capture". A rate change made
// externally mid-session would not be detected. Closing that would need a
// periodic SR re-read, which is a Modbus/behaviour change and deliberately out
// of Phase 3A scope. The staleness is bounded by the sensor-restart path,
// which itself re-runs this verification.
//
// Written on Core 0 (taskModbusRead) and read on Core 0 (drain block) only;
// single-word volatile scalars, no mutex needed (CLAUDE.md cross-core rule).
// ----------------------------------------------------------------------------
// NOTE: only VARIABLES are declared here. sensorSrIndexToHz() is deliberately
// defined much further down, immediately above reconfigSensorAfterRestart().
// Per CLAUDE.md, the .ino auto-prototype generator inserts every generated
// prototype directly before the FIRST function definition in the file -- so a
// function defined here, above the VibrationData_t / MachineState_t / ...
// typedefs, would push all of those prototypes ahead of the types they name
// and break the whole build.
static volatile uint16_t g_sensorSrIndexVerified = FIFO_SR_INDEX_UNKNOWN;
static volatile uint32_t g_sensorSrHzVerified    = 0;  // 0 == NOT established

#define REG_DRM           0x002B  // Displacement range mode register §6.4.11
#define SENSOR_DRM_FREQ   0x0002  // 0x02 = Frequency domain algorithm
                                  // จำเป็นสำหรับ VRMS (0x50/0x5C/0x68) ให้คำนวณถูกต้อง
#define SENSOR_MODE_FREQ  0x0002  // Frequency domain algorithm (MODE=0x02)
                                  // ยืนยันจาก CF test log: MODE=0x02 เท่านั้นที่ให้ CF/VRMS มีค่า
                                  // MODE=0x00 (LowFreq) และ 0x01 (HighFreq) → CF/VRMS = 0x0000 ทั้งหมด
                                  // [PATCHED v16.1] เปลี่ยนจาก SENSOR_MODE_TDLF=0x0000 → SENSOR_MODE_FREQ=0x0002
#define SENSOR_CMD_SAVE   0x0000  // Save config to NVM
// ????? error ??????????????????? OFFLINE (3 x 250ms = 750ms)
#define MODBUS_OFFLINE_THRESHOLD 3

// --- 4G Network Configuration ---
static constexpr const char* APN = "internet";

static constexpr const char* GPRS_USER = "";
static constexpr const char* GPRS_PASS = "";

// --- Proximity / RPM Sensor Configuration ---
#define PIN_RPM               17      // Proximity sensor pulse input (PC817 or NPN)
#define PULSE_PER_REV         1       // Pulses per revolution
// [v16.6-rpmsep] MAX_RPM is the MAXIMUM MEASURABLE SPEED, and nothing else.
// It feeds exactly two gates: RPM_MIN_INTERVAL_US (below) and the spike-reject
// comparison in processRPM(). It NO LONGER sets the ISR debounce -- see
// RPM_DEBOUNCE_US, which is now derived from measured bounce physics instead.
// 3000 -> 4000 so the ceilings clear 3600 RPM:
//   RPM_MIN_INTERVAL_US = 60e6/(4000*1) = 15,000 us  <  15,583 us (3600 RPM
//                                                       worst case, -6.5% jitter)
//   spike reject        = 4000 * 1.1    =  4,400 RPM  >  3,600 RPM
#define MAX_RPM               4000   // Maximum measurable speed (spike-reject + interval ceiling)
#define MIN_RPM_VALID         300      // Below this -> treat as zero
#define RATED_RPM             NAMEPLATE_RPM   // Rated speed (centre of RUNNING band)
#define RATED_RPM_TOL         75       // +/-75 RPM around RATED_RPM -> RUNNING band (5% of 1500)
#define RPM_SMOOTH_ALPHA      0.25f   // EMA filter coefficient (0=heavy,1=none)
#define SPIKE_REJECT_FACTOR   1.1f    // Reject pulses > MAX_RPM x factor
#define NO_PULSE_STOPPING_MS  400     // No pulse > 400 ms -> STOPPING
#define FORCE_STOP_TIMEOUT_MS 2000    // No pulse > 2 s   -> STOPPED
// [v16.5.4] Maximum credible pulse gap for RPM EMA continuity. A measured
// pulse interval above this (or an equally long no-pulse idle) invalidates
// the EMA: g_rpmFiltered is reset to 0 and the next valid pulse reseeds it
// from raw RPM instead of blending with the stale value. Normal running
// (interval << 2 s) is unaffected.
// Intentionally an INDEPENDENT literal, not derived from FORCE_STOP_TIMEOUT_MS
// -- mirrors this file's existing convention (see the historical
// NO_PROXIMITY_STOPPING_MS/FORCE_PROXIMITY_STOPPED_MS constants, which
// "mirror" NO_PULSE_STOPPING_MS/FORCE_STOP_TIMEOUT_MS in value but are kept
// as separate named constants "so [it] can be tuned independently ... without
// perturbing RPM"). Coupling EMA signal-processing timing to an FSM-tuning
// constant would mean a future change to FORCE_STOP_TIMEOUT_MS silently
// changes EMA invalidation too, and vice versa -- two different concerns
// (signal freshness vs. business-state timeout) sharing one knob. Value
// matches FORCE_STOP_TIMEOUT_MS's 2 s horizon today by deliberate choice, not
// by algebraic coupling.
#define MAX_EMA_INTERVAL_US   2000000UL   // 2.0 s, in microseconds
#define FAULT_WINDOW_MS       3000    // RUNNING but no pulse > 3 s -> prox=0 (Fault)
#define RUNNING_WARMUP_MS     2500    // [v16.3z] ต้อง in-band ต่อเนื่อง 2.5s ก่อนเป็น RUNNING (กัน bounce/spurious)
// [vNext] TEMPORARY startup-settling suppression for FAULT_LATCH ONLY -- does not
// affect newState (STATE_WARNING/CRITICAL, buzzer, live telemetry all still see the
// real transient), DEGLITCH, or the FIFO Broker. See g_motorRunFaultLatchHoldoff.
#define MOTOR_RUN_FAULT_LATCH_HOLDOFF_READS  4   // suppress FAULT_LATCH for N reads (~1s @ 250ms) immediately after STARTING->RUNNING -- mechanical/vibration settling transient, not a real fault
// [Commit 3A] Confirmed-absence timeouts -- deliberately separate from
// NO_PULSE_STOPPING_MS/FORCE_STOP_TIMEOUT_MS above, which are calibrated for
// "no signal at all" (sub-second/2s). These instead bound "evidence is fresh
// but signalPresent has been continuously false" -- e.g. Current reads
// succeeding but reporting below threshold. Must be well above
// RUNNING_WARMUP_MS so a normal startup ramp (signalPresent=false while
// climbing toward the band) is never mistaken for a stopped motor.
// PLACEHOLDER VALUES -- not validated against real startup ramp durations;
// review before relying on this in production, especially for RPM.
#define ABSENT_STOPPING_MS    15000   // signalPresent false (but fresh) > 15s -> STOPPING
#define ABSENT_STOPPED_MS     30000   // ... > 30s -> STOPPED
#define STOPPED_CLEAR_MS      (30UL*60UL*1000UL)  // [v16.3aa] หยุด > 30 นาที = clear trend (bearing state เทียบไม่ได้แล้ว)
#define COLD_START_TEMP_DROP_C 5.0f   // [v16.3ad] temp ลดจากตอนหยุด >= 5°C = bearing เย็นลง = cold start (เทียบ trend ไม่ได้)
                                      // gate หลัก: motor_state == MOTOR_RUNNING (3 จุด)
                                      // gate รอง: rpm >= 400 เป็น safety floor เพิ่มเติม
                                      // ป้องกัน false drift alert ระหว่าง STARTING/STOPPING

// [Phase 1] Bearing-alert kurtosis thresholds REMOVED together with the whole
// legacy bearing pipeline: KURTOSIS_EARLY_WARNING, KURTOSIS_CONFIRMED and
// BEARING_STABLE_CYCLES. They fed bearing_alert only -- never alarm_level,
// whose bearing escalation was already disabled in v16.3l. Machine condition
// is decided solely by FIFO/DSP velocity against VIB_WARNING_MMS /
// VIB_CRITICAL_MMS, which this change does not touch.
//
// [v16.3m] RMS sanity cap: ค่าสูงสุดที่เป็นไปได้จริง
// ถ้า rms > นี้ = garbage จาก sensor reconfig fail → ไม่ update peak hold และไม่ latch
// ตั้งไว้ที่ 3× CRITICAL threshold = 21.3 mm/s
                                      // ก่อนประเมิน bearing -- หลีกเลี่ยง startup transient
#define NVS_SAVE_INTERVAL_MS  30000UL // Save runtime_hour to NVS every 30 s

// ============================================================================
// FAULT LATCH v3 — CONSTANTS  [inserted: fault_latch_v3_code.ino SECTION 1]
// ============================================================================
#define FL_NS           "fault_latch"
#define FL_KEY_CODE     "ev_code"
#define FL_KEY_TS       "ev_ts"
#define FL_KEY_RMS      "ev_rms"
#define FL_KEY_KURT     "ev_kurt"
#define FL_KEY_PENDING  "ev_pending"
#define FL_KEY_MAGIC    "ev_magic"
#define FL_MAGIC_VALUE  0xF401A7CDUL
#define FL_EVT_NONE      0u
#define FL_EVT_WARNING   1u
#define FL_EVT_CRITICAL  2u
#define FL_EVT_BEARING   3u
#define FL_EVT_HEALTH    4u
#define FL_EVT_MAX_VALID 4u
#define FL_SEV_NONE     0u
#define FL_SEV_WARNING  1u
#define FL_SEV_HEALTH   2u
#define FL_SEV_CRITICAL 3u
#define FL_SEV_BEARING  4u
#define FL_HEALTH_LOW_THOLD  30
#define FL_TS_MIN_VALID  1577836800UL
#define FL_TS_UNKNOWN    0UL

// ============================================================================
// FAULT LATCH v3 — STRUCT  [inserted: fault_latch_v3_code.ino SECTION 2]
// ============================================================================
struct FaultLatch_t {
  uint32_t      ts;
  float         rms;
  float         kurtosis;
  uint8_t       code;
  volatile bool pending;
};
// NOTE (v3 hardened): globals for this struct (g_fl, g_flCount, g_flPrev*)
// are declared later, in the SHARED VARIABLES block, because g_flPrevState
// requires MachineState_t, which is not declared until the ENUMERATIONS
// section below. See that block for the SECTION 3 globals.

// --- MQTT Configuration (mTLS / Mosquitto) ---
#define MQTT_SERVER "iot.promlogix.com"  // Broker Public IP
#define MQTT_PORT 8883                           // TLS port
#define MQTT_CLIENT_ID "pump01"           // -> "PLANT01-ESP01"
#define MQTT_QOS 1                               // QoS 1 -- at-least-once delivery
// [P1-S3] /device-health cadence. Fixed and independent of publishInterval
// (30/10/5 s by alarm state) on purpose: engineering diagnostics must not
// accelerate with alarm state, and a 4th CIPSEND every 5 s is the A7670 link
// pressure that historically produced LWMQTT_NETWORK_FAILED_WRITE (-6).
#define DEVICE_HEALTH_INTERVAL_MS 60000UL
// [v16.5d] Bounds GsmTLSClient's _tcp.stop() teardown wait (was unbounded
// 15000ms via TinyGSM default) -- see session-takeover/GPRS-reconnect review.
#define BOUNDED_STOP_MS 1000UL
// Mosquitto config:  require_certificate = true
//                    use_identity_as_username = true
//                    allow_anonymous = false
// -> Username is extracted from client certificate CN by the broker.
//   No explicit username/password required in CONNECT packet.
// Topic built at runtime: factory/{PLANT_ID}/machine/{MACHINE_ID}/vibration

// ============================================================================
// mTLS CERTIFICATES (PEM format)
// ============================================================================
// Replace the placeholder blocks below with your actual certificate contents.
// Use OpenSSL to generate:
//   openssl genrsa -out PLANT01-ESP01.key 2048
//   openssl req -new -key PLANT01-ESP01.key -out PLANT01-ESP01.csr \
//           -subj "/CN=PLANT01-ESP01"
//   openssl x509 -req -in PLANT01-ESP01.csr -CA industrial-root-ca.crt \
//           -CAkey industrial-root-ca.key -CAcreateserial \
//           -out PLANT01-ESP01.crt -days 3650
// ============================================================================

// Root CA -- industrial-root-ca.crt
static const char* root_ca = R"EOF(
-----BEGIN CERTIFICATE-----
MIIDdzCCAl+gAwIBAgIUaeT/6iBLHWm30hAItq+9q9lv3ZMwDQYJKoZIhvcNAQEL
BQAwSzELMAkGA1UEBhMCVEgxEjAQBgNVBAoMCVByb21sb2dpeDEMMAoGA1UECwwD
SW9UMRowGAYDVQQDDBFQcm9tbG9naXgtUm9vdC1DQTAeFw0yNjA1MDIxMTQ1MTJa
Fw0zNjA0MjkxMTQ1MTJaMEsxCzAJBgNVBAYTAlRIMRIwEAYDVQQKDAlQcm9tbG9n
aXgxDDAKBgNVBAsMA0lvVDEaMBgGA1UEAwwRUHJvbWxvZ2l4LVJvb3QtQ0EwggEi
MA0GCSqGSIb3DQEBAQUAA4IBDwAwggEKAoIBAQDAupZ3ySVJ3qok6/ZmezLSDJsu
zp9eYGryO3w8ezbgyAvjE6nyaImmRUsTIwWR4V3J2XZz5G4Bzm32avDYqrMm4pnr
45UfVvldwLQFhMlkDUYFpWwxNDPxKvIkoJIzPKg/U5XT9glHsM5nvhpTKLpPrR7q
o1yfANtncwxRFQOwVrBqXjdmaveVqwOv/9n+J6+bELh3wKoOb5LTU1ZfISfE5PxE
YulvbVcPOcNTrIkmdyQP+arjiX4cNO3eQZo6OxdMYnenHlDaNbahv1qZBzwMHFUn
U9IGYsmmw/UIz+B1DUEUvuFdSbynqq6awfY76XPqDQidnXRHYo3Q+JPeY9uHAgMB
AAGjUzBRMB0GA1UdDgQWBBR/RPU+w/yXvpIX9CWZ0e8FUsEGxjAfBgNVHSMEGDAW
gBR/RPU+w/yXvpIX9CWZ0e8FUsEGxjAPBgNVHRMBAf8EBTADAQH/MA0GCSqGSIb3
DQEBCwUAA4IBAQC2k9C9TVMehB8N2UoW91+IacoJDBRSpHhH+U55EtJLSg4sU9RI
sfaA6VDNdttxdRgG6O8Vp1KNCw5NrUeICHei6boJyI0a3Fz3SOKC8y0hkclx6bcc
mv9jhc4OyCUTMat8ojjq3ozYrYSLlHAmpvc2rhhCn9Z39Fp+8mdPd42eLCy7x0sE
vGmxO01oyezvwJAZineNUn6e26DvuDYX+FrV4BxZBa++28Ab4iZSqBaE/XgQVMf7
gvuvFI/Aqftgjk0yv8wA6WcANh9stUDkI0gMBEmAzwl4y0v9G8ktgHrl1nk2WApy
GqxIBNFsqLdGN189BwBlBPnWscWJg+oGqLtT
-----END CERTIFICATE-----
)EOF";

// Client Certificate -- PLANT01-ESP01.crt  (CN must match MQTT_CLIENT_ID)
static const char* client_crt = R"EOF(
-----BEGIN CERTIFICATE-----
MIIC4zCCAcsCFHrIJfxa9kIp8xl2OiGamsQiUjWrMA0GCSqGSIb3DQEBCwUAMEsx
CzAJBgNVBAYTAlRIMRIwEAYDVQQKDAlQcm9tbG9naXgxDDAKBgNVBAsMA0lvVDEa
MBgGA1UEAwwRUHJvbWxvZ2l4LVJvb3QtQ0EwHhcNMjYwNTAyMTE0NTM0WhcNMjcw
NTAyMTE0NTM0WjARMQ8wDQYDVQQDDAZwdW1wMDEwggEiMA0GCSqGSIb3DQEBAQUA
A4IBDwAwggEKAoIBAQCcFCUyXJIAHj1m5LJNFSzrR9trx4ELBYq4UpAZZtiY618T
jYCecCqNRDIeaGNLadXjAqZRADE1cV1NZzIDnMw4DUA8fqEJ+8GURiCkvlypwDcf
Zv4Gdto20uQFSKfFvvoBRu7ze89VVNY/0uc4Qpip04hzK4fspmI/zDz8JUPxDELt
Rp769mKgmoNsUO14DSMWJbAolbXLDpBaGuD61Svlm5o/YKNfcLDbm35+6aDG7+/J
FzM9DYYPQxkvw+Nu+t2gkHfs+qcgLo+7oget/P2JXOy1Ysc2jMqQNDco8M1rob8T
NZzb4mKcW4LtgNGudsyel5p0utmlxKeTevXvIx5HAgMBAAEwDQYJKoZIhvcNAQEL
BQADggEBAF+pIJv97lLnSRRAwTo6UTxWEPhY32BCWsN09me6bDbq87dogqtZTL1H
c2wZClOea+jE0U/hBIo/Vh/f5YNCDH3nMEuIBgs6fNM8Y2DaIM/Wz6hh1iQap6FE
dwDbDMckfpGM4Sn5valLzYwkqe4aflaDY5Uas9FzolPOCItGsc4g2gafOaDoOrFo
XDjfTO3o2eznKC/Ou/ft/tVcdEX/BdFKu1Sqw8UntlkjaDtM4T9bz+RfoYTL0UPT
zKcQVPxhvKKbyiCe62gBYl5tLD1vriUBZwioCJLHfV2XMJx47msJiOLtyz6SdDc0
dzc7CkmVKekWfJ+jVC0bgIONpwLNEWo=
-----END CERTIFICATE-----
)EOF";

// Client Private Key -- PLANT01-ESP01.key
static const char* client_key = R"EOF(
-----BEGIN PRIVATE KEY-----
MIIEvQIBADANBgkqhkiG9w0BAQEFAASCBKcwggSjAgEAAoIBAQCcFCUyXJIAHj1m
5LJNFSzrR9trx4ELBYq4UpAZZtiY618TjYCecCqNRDIeaGNLadXjAqZRADE1cV1N
ZzIDnMw4DUA8fqEJ+8GURiCkvlypwDcfZv4Gdto20uQFSKfFvvoBRu7ze89VVNY/
0uc4Qpip04hzK4fspmI/zDz8JUPxDELtRp769mKgmoNsUO14DSMWJbAolbXLDpBa
GuD61Svlm5o/YKNfcLDbm35+6aDG7+/JFzM9DYYPQxkvw+Nu+t2gkHfs+qcgLo+7
oget/P2JXOy1Ysc2jMqQNDco8M1rob8TNZzb4mKcW4LtgNGudsyel5p0utmlxKeT
evXvIx5HAgMBAAECggEAAuJQaaTSQdRNOCiDru70PIjAYjZ2iPiaPpuv8/g1imXX
BOp5dPQHpUKcVnmBVDRpcl9rKVYCksU8fyCoCO8Nyv9br4J7gU64nf/JvKGT3sMh
gaAKk54AnEC7W+miyAGmZv2jjrY794ywxM8l3KFGZuT0wYQNZ+8PI7Snb9VUcxDK
jHGUQrXM+fbKGLXRkhfeFzfUD7aNqYZWMkt2KwzZsSh8M+ctp+G5IdcNEQOS2GVo
wX6d2KGkNudd9aCKi50XQPFem2TVOllFQYijld2SABYK1pQiPXR+NrLSnb6RUm15
3w/g3eax0qa52XdD+KTixxi1oPe8yxe4qDAz6lLEqQKBgQDK5U/Bt4Jo4geCV0Za
izoNnLcqa6+nHlsphYXw5pUqLSLq4RCGJMcbojkYXxXxJYsmlm0+OQ9S9WW6rJ8d
z2jPUWS3MKVW3KZ338oS9AgBr1aNhggkCQpg2SEbsmM1khs+t8HoN8LIFLGVb7PB
dZKLw8VctsSlxwPgsxsont4qHQKBgQDE7e+HKhKWplAwwIBDCUUy1xjZH+nIeYsV
hWF4iE1hDuUN/dYFVClDS4aRPDrgeB4lzzqw5NCSHR/Gu0TeHMspJj1k1CDBiI12
LruakLOw4P3ul13Yv23EJuUq/In8Q3otYF3yaks1nwM9/hgEMRmlsbUgbSGW1tKW
RkvorwqcswKBgHGEaOIuRPVfeOoQ4FjqSpmxE73VMBqlXkXV4cGNkOlfBYk6UN9s
lkW8tosPMBySb88wHIDSteMpTzhpOkEYeUB8/oeL3QXDQBQTjmCaThx7OEbINafL
sxXKhb6USPOBAmNNtlyxTfZZtZ2xOHZFzK8L4lFkJJPHzECclNZeRFh1AoGBAJML
t9eNquOivC4rD5r+yRT1WDCIi+COITSoq+d8n4rhvFd+Otkvxr/hHVJFTxFdn+VL
n9+Ge9ceuCOEoh/YEDthumYXn33joP2mV59KfWKOHg6SKBk4l5XoFSbL+5zKJejM
FFp21EHtwlX/7Z7zqtr2nvDfjD09m3FqfDP6wEnRAoGAK90MergErflRfvNqGmSf
BUkR4hIjiNHCnNizxJUtuwG+HDf7bg0Lvo5KYALYnwNuSEzE1EsLJEFl22a7zCJ8
EZ5Dxy12SrP6E+hbMhuV1KeMax45WU4beFu88wTlbz8A8is2KKjKbS9+8e79R1I/
l0PCpmCF8SZ8OXd/UfRIbLk=
-----END PRIVATE KEY-----
)EOF";

// --- Machine Configuration ---
#define MACHINE_NAME MACHINE_ID  // Display uses MACHINE_ID for consistency
// [M1A LEGACY/TBD] These two thresholds were baselined against the legacy
// VRMS-register metric, which is max(x,y,z). They are NOT valid for
// velocity_rms_overall, which is the vector magnitude sqrt(x^2+y^2+z^2) and
// therefore reads up to sqrt(3) ~ 1.73x higher for comparable axes. Reusing
// them would silently tighten the alarm by that factor.
//
// They remain ONLY because the un-migrated legacy consumers still reference
// them: computeHealthScore()'s normalization span, SANITY_RMS_MAX, the OLED
// WARN/CRIT legends, and the trend engine (M1B). They no longer gate any
// vibration alarm decision.

// [M1A] Product Phase-1 vibration alarm thresholds, in mm/s, applied to
// velocity_rms_overall (FIFO RAW -> DSP).
//
// *** TBD -- DELIBERATELY UNSET. NO VALUE HAS BEEN CHOSEN. ***
// Pending field/reference re-baselining against a calibrated instrument
// (deferred R&D; blocked on the same reference instrument as Validation D).
//
// VIB_THRESHOLD_UNSET is a sentinel, not a threshold: it is negative, so it
// can never be crossed by a non-negative RMS even if a future edit
// accidentally removed the vibThresholdsConfigured() gate. Fail-closed by
// construction rather than by convention.
#define VIB_THRESHOLD_UNSET (-1.0f)

// ============================================================================
// [v16.6 S21] PROMLOGIX PROVISIONAL V1 THRESHOLD
//             ISO-INFORMED / NOT ISO-CERTIFIED
// ============================================================================
// These are PROVISIONAL PRODUCT values chosen to enable live evaluation. They
// are ISO-INFORMED -- the magnitudes are drawn from the velocity-RMS band
// structure that ISO 20816-class guidance popularised -- but they are NOT an
// ISO limit, NOT ISO certified, and NOT ISO compliant. No ISO standard
// document exists in this repository and no ISO machine class has been
// established for this asset. Do not describe them as an ISO threshold in
// firmware, telemetry, API, UI or customer documentation.
//
// PROVENANCE OF THE MEASURED BASELINE (S13/S20, this bench, 1485 rpm):
//   healthy, bolts removed : velocity_rms_overall ~= 0.322 mm/s (n=805,
//                            8 independent starts, across-start CV 4.24%)
//   known unbalance, bolts installed : ~= 1.025 mm/s (n=550, 3 starts)
// Both sit BELOW the WARNING value below, so on the present bench this
// firmware is expected to report NORMAL in both configurations. These
// thresholds are therefore NOT yet calibrated severity anchors -- the S20
// multi-level unbalance ladder remains the work that would make them so.
// ============================================================================
#define VIB_WARNING_MMS       2.1f   // WARNING_ON   -- provisional V1
#define VIB_CRITICAL_MMS      4.5f   // CRITICAL_ON  -- provisional V1

// Hysteresis OFF (de-escalation) points. Provisional engineering values.
// REASON: within-condition CV measured 8.7% (sigma ~= 0.028 mm/s at baseline).
// A bare threshold makes a machine sitting near a boundary flip state -- and
// the buzzer -- on measurement spread alone, at the 250 ms decision cadence.
// The deadband must exceed that spread by a wide margin; 0.2 / 0.3 mm/s do.
#define VIB_WARNING_OFF_MMS   1.9f   // WARNING_OFF  -- provisional V1
#define VIB_CRITICAL_OFF_MMS  4.2f   // CRITICAL_OFF -- provisional V1

// Documented reference value only. Deliberately NOT a fourth alarm state:
// the state machine has exactly three vibration levels (NORMAL/WARNING/
// CRITICAL) and adding a state would change the alarm contract, the telemetry
// enum and the API. Recorded here so the intended high-severity reference is
// preserved with the thresholds it belongs to.
#define VIB_REFERENCE_HIGH_MMS 7.1f  // reference / high-severity -- NOT a state

// [S21] Escalation persistence, counted in DISTINCT FIFO CAPTURES.
// The alarm decision runs every ~250 ms but the velocity carrier only
// refreshes on a new FIFO capture (~2.1 s), so the SAME measurement is
// re-evaluated ~8 times. Counting decision ticks would count one measurement
// eight times and prevent nothing. Counting captureId transitions makes "two
// consecutive observations" mean two genuinely independent measurements
// (~4.2 s). De-escalation is deliberately NOT persistence-gated -- clearing an
// alarm promptly is the safe direction, and test 12 requires it.
#define VIB_ALARM_PERSIST_CAPTURES 2u

// [M1A] Freshness deadline for the velocity carrier.
// *** TBD PLACEHOLDER -- this is NOT a product SLA. ***
// Chosen only to be a few capture intervals (capture cadence ~2 s) so the
// plumbing is exercisable. The real value is a product decision tied to M3
// (freshness/age semantics) and must be set with the customer's detection
// latency requirement in hand, not here.
#define VIB_VELOCITY_MAX_AGE_MS_TBD 10000u

// --- FreeRTOS Configuration ---
#define STACK_SIZE_MODBUS    4096   // Modbus task stack
#define STACK_SIZE_DISPLAY   6144   // Display task stack (larger for U8g2)
#define STACK_SIZE_NETWORK  24576   // Phase2: 24KB -- RSA-2048 + JSON 2200B + TinyGSM peak
#define STACK_SIZE_BUTTON    4096   // V14.7: 2048→4096 (watermark was 172B=92% used; rtc+Wire+Serial.printf depth)
#define STACK_SIZE_STATE     8192   // [v16.3n] 6144→8192: NVS write (Preferences) ใน checkAndLatchFault
                                    // ใช้ IPC call ไป Core 1 → ipc1 stack overflow ถ้า stack ไม่พอ
// [M1B-3] 6144 -> 8192. The /trend JSON document and its serialization buffer
// both live on this task's stack and each grow 1024 -> 1536 B for the M1B-3
// window fields (+1024 B total). Measured Analytics high-water was 2940 B
// remaining, so the raise keeps the same margin rather than spending it.
#define STACK_SIZE_ANALYTICS 8192   // Phase 3: +decision engine +classifyFault on stack

#define PRIORITY_MODBUS    5  // Highest priority (time-critical)
#define PRIORITY_STATE     4  // State machine
#define PRIORITY_DISPLAY   3  // Display updates
#define PRIORITY_ANALYTICS 3  // Analytics (same as display -- non-critical, 1s cadence)
#define PRIORITY_NETWORK   2  // Network (can tolerate delays)
#define PRIORITY_BUTTON    2  // Button handling
#define PRIORITY_BUZZER    1  // Lowest priority

// --- Queue Sizes ---
#define QUEUE_SIZE_SENSOR 5
#define QUEUE_SIZE_BUTTON 3
#define QUEUE_SIZE_DISPLAY 3
#define QUEUE_SIZE_MAINT 2   // V14.4: maintenance reset events (Button -> Network)
#define QUEUE_SIZE_MQTT_OUTBOUND 6  // [v16.5] Section 7 Item 3: dormant outbound MQTT queue (Analytics -> Network4G, not wired yet)
#define QUEUE_SIZE_FIFO_TRIGGER 1  // [Broker, Commit 1] SDS SS14.1 depth-1 request queue (any task -> taskModbusRead)

// [M1B-3] 1024 -> 1536. REQUIRED, not cosmetic.
//
// enqueueMqttOutbound() REJECTS (does not truncate) any payload >= this limit:
//   if (... || len >= MQTT_OUTBOUND_PAYLOAD_MAX) return false;
// /trend measured 857 B on hardware after M1B-2. The twelve M1B-3 window keys
// add ~354 B -> ~1211 B, which under the old 1024 limit would have caused
// EVERY /trend message to be silently DROPPED -- taking the M1B-1 history_*
// observability down with it. Dropping is worse than truncating because
// nothing in the log would say so.
//
// Cost: MqttOutboundMsg_t.payload grows 512 B x QUEUE_SIZE_MQTT_OUTBOUND (6)
// = +3072 B RAM. Accepted deliberately; see the M1B-3 report.
#define MQTT_OUTBOUND_PAYLOAD_MAX 1536

// --- Modem Timeouts ---
#define MODEM_INIT_TIMEOUT 30000    // 30 seconds for modem init
#define GPRS_CONNECT_TIMEOUT 60000  // 60 seconds for GPRS connect
#define MODEM_RETRY_DELAY 10000     // 10 seconds between retries

// --- Time Sync Configuration ---
// [v16.5j] NTP retry-loop fix: previously a single pair of constants
// (NTP_SYNC_INTERVAL=24h, NTP_SYNC_RETRY_INTERVAL=30min) governed the
// post-sync and pre-sync cadence respectively, but the scheduling gate in
// checkAndSyncTime() never actually respected either value while sync kept
// failing (see lastCheckMillis below) -- renamed here to the cadence they
// actually now drive, with values re-baselined: fast retry while the modem
// has not yet produced a first valid time (was 30 min, now 30 s -- 30 min
// to acquire a first RTC sync after boot was too slow), normal interval
// once synced (was 24h, now 30 min, matching this device's expected drift
// characteristics).
#define NTP_CHECK_INTERVAL_MS 1800000UL    // 30 minutes between re-checks once synced
#define NTP_RETRY_INTERVAL_MS 30000UL      // 30 seconds retry while not yet synced
#define NTP_DRIFT_WARN_SEC 5               // Warn if drift exceeds 5 seconds
#define NTP_DRIFT_MAX_SEC 30               // Force-correct if drift > 30 seconds

// MAX_VALID_YEAR: upper bound for GSM date-time sanity check (v15.8).
// Rejects epoch-0 placeholder times sent by SIMCom modems before network
// sync, which 2-digit-year conversion ("YY"+2000) turns into far-future
// years -- e.g. "70/01/01" -> 2070, "80/01/06" -> 2080. Keep this value
// comfortably above the current year but below 2070 so both placeholders
// are rejected. Update if the device is still in service near 2069.
#define MAX_VALID_YEAR 2060
#define TIMEZONE_OFFSET_SEC (7 * 3600)     // UTC+7 (Bangkok/Thailand) -- adjust per site

// ============================================================================
// GLOBAL OBJECTS
// ============================================================================

// Hardware Serial for Modem (Serial1)
HardwareSerial SerialAT(1);

// Hardware Serial for Modbus RS485 (Serial2 / UART2)
// ESP32 Arduino core 3.x does not pre-declare Serial2 — must instantiate explicitly,
// same pattern as SerialAT above. UART number must match: 2 = UART2 (RX=38, TX=39).
HardwareSerial SerialRS485(2);

// TinyGSM objects
TinyGsm modem(SerialAT);
TinyGsmClient rawClient(modem, 1);  // Plain TCP on mux 1 (used for TCP reachability test)

// -----------------------------------------------------------------------------
// GsmTLSClient -- mirrors WiFiClientSecure but over 4G modem TCP transport
// -----------------------------------------------------------------------------
// Key design decisions vs previous versions:
//
// 1. CERTS PARSED ONCE -- entropy/drbg/x509/pk are initialized once at startup
//    via loadCerts() and never freed/re-parsed. This eliminates:
//    - "Private key parse failed: -0x3E80" (heap fragmentation after reconnects)
//    - ~42KB heap drop per attempt from repeated cert parsing
//
// 2. SSL CONTEXT RESET PER CONNECTION -- only _ssl and _conf are freed/re-init
//    on each stop()/connect() cycle, keeping the heavy cert data intact.
//
// 3. NON-BLOCKING bio_recv -- wait loop is in the handshake loop (not bio),
//    so mbedTLS drives timing correctly and large TLS records (server cert ~3KB)
//    have enough time to arrive over the slow 4G modem without hitting timeout.
// -----------------------------------------------------------------------------
#include "mbedtls/ssl.h"
#include "mbedtls/entropy.h"
#include "mbedtls/ctr_drbg.h"
#include "mbedtls/x509_crt.h"
#include "mbedtls/pk.h"
#include "mbedtls/error.h"
#include "mbedtls/debug.h"  // TLS debug trace via mbedtls_debug_set_threshold()
#include "esp_task_wdt.h"   // WDT: reconfigure timeout, subscribe/reset Network task
#include "esp_system.h"     // v15.3: esp_reset_reason() -- detect reboot cause

class GsmTLSClient : public Client {
public:
  explicit GsmTLSClient(TinyGsm& modem)
    : _tcp(modem, 0), _modem(modem) {
    // Persistent contexts -- live for the lifetime of the object
    mbedtls_entropy_init(&_entropy);
    mbedtls_ctr_drbg_init(&_drbg);
    mbedtls_x509_crt_init(&_ca_crt);
    mbedtls_x509_crt_init(&_cli_crt);
    mbedtls_pk_init(&_pk);
    // Per-connection contexts
    mbedtls_ssl_init(&_ssl);
    mbedtls_ssl_config_init(&_conf);

    mbedtls_ctr_drbg_seed(&_drbg, mbedtls_entropy_func, &_entropy,
                          (const uint8_t*)"gsmtls1", 7);
  }

  ~GsmTLSClient() {
    _freeSession();
    mbedtls_pk_free(&_pk);
    mbedtls_x509_crt_free(&_cli_crt);
    mbedtls_x509_crt_free(&_ca_crt);
    mbedtls_ctr_drbg_free(&_drbg);
    mbedtls_entropy_free(&_entropy);
  }

  // -- Call once after construction -- parses certs into mbedTLS structures --
  bool loadCerts(const char* ca, const char* crt, const char* key) {
    int ret;
    if (ca) {
      ret = mbedtls_x509_crt_parse(&_ca_crt,
                                   (const uint8_t*)ca, strlen(ca) + 1);
      if (ret != 0) {
        Serial.printf("[TLS] CA parse failed: -0x%04X\n", -ret);
        return false;
      }
    }
    if (crt) {
      ret = mbedtls_x509_crt_parse(&_cli_crt,
                                   (const uint8_t*)crt, strlen(crt) + 1);
      if (ret != 0) {
        Serial.printf("[TLS] Cert parse failed: -0x%04X\n", -ret);
        return false;
      }
    }
    if (key) {
      ret = mbedtls_pk_parse_key(&_pk,
                                 (const uint8_t*)key, strlen(key) + 1,
                                 NULL, 0, mbedtls_ctr_drbg_random, &_drbg);
      if (ret != 0) {
        Serial.printf("[TLS] Key parse failed: -0x%04X\n", -ret);
        return false;
      }
    }
    _certsLoaded = true;
    Serial.println("[TLS] Certificates parsed and loaded +");

    // -- Diagnostic: print cert subjects to confirm CN = MQTT client ID --
    // If CN does not match MQTT_CLIENT_ID, broker will reject with rc=5.
    // Requires mbedtls/x509.h which is included transitively via x509_crt.h.
    {
      char buf[128];
      mbedtls_x509_dn_gets(buf, sizeof(buf), &_cli_crt.subject);
      Serial.printf("[TLS] Client cert subject: %s\n", buf);
      mbedtls_x509_dn_gets(buf, sizeof(buf), &_cli_crt.issuer);
      Serial.printf("[TLS] Client cert issuer:  %s\n", buf);
      mbedtls_x509_dn_gets(buf, sizeof(buf), &_ca_crt.subject);
      Serial.printf("[TLS] CA cert subject:     %s\n", buf);
      // Confirm key matches cert by checking key type
      Serial.printf("[TLS] Key type: %s, bitlen: %u\n",
                    mbedtls_pk_get_name(&_pk),
                    (unsigned)mbedtls_pk_get_bitlen(&_pk));
    }
    return true;
  }

  // Compatibility shims for existing setupTLS() call pattern
  void setCACert(const char* ca) {
    _ca_pem = ca;
  }
  void setCertificate(const char* c) {
    _crt_pem = c;
  }
  void setPrivateKey(const char* k) {
    _key_pem = k;
  }

  // Called by setupTLS() -- triggers actual cert loading
  bool applyCredentials() {
    return loadCerts(_ca_pem, _crt_pem, _key_pem);
  }

  int connect(IPAddress ip, uint16_t port) override {
    return connect(ip.toString().c_str(), port);
  }

  int connect(const char* host, uint16_t port) override {
    if (!_certsLoaded) {
      Serial.println("[TLS] ERROR: loadCerts() not called before connect()");
      return 0;
    }

    // -- Always reset SSL session before connecting --
    // arduino-mqtt does NOT call stop() before retry, so _ssl/_conf may be
    // dirty from a previous failed handshake. Reset them unconditionally.
    //
    // [v16.5e] `if (_tcp.connected()) _tcp.stop();` REMOVED from this block.
    //
    // INVARIANT: taskNetwork() calls gsmClient.resetTLS() immediately before the
    // file's ONLY mqttClient.connect() call site, and resetTLS() unconditionally
    // performs _tcp.stop(BOUNDED_STOP_MS). The TCP socket is therefore already
    // closed on entry here, so the removed check could never do useful work.
    //
    // It was not merely redundant. _tcp is a TinyGsmClient, and its connected()
    // dispatches GsmClient::connected() -> available() -> maintain() ->
    // modemGetAvailable(), which sends AT+CIPRXGET=4,<mux>, and on an empty RX
    // buffer additionally modemGetConnected() -> AT+CIPCLOSE?. Right after
    // resetTLS()'s stop() that buffer is guaranteed empty, so BOTH AT round-trips
    // fired -- into a modem still settling its own CIPCLOSE, and against the
    // FIX-4 rule this class documents on connected() below.
    _freeSession();
    mbedtls_ssl_init(&_ssl);
    mbedtls_ssl_config_init(&_conf);
    _connected = false;

    // -- Step 1: TCP via modem --
    Serial.printf("[TLS] TCP connecting to %s:%d ...\n", host, port);
    if (!_tcp.connect(host, port)) {
      Serial.println("[TLS] TCP connect failed");
      return 0;
    }
    Serial.println("[TLS] TCP OK, starting TLS handshake...");

    // -- Step 2: configure SSL (TLS 1.2 to match broker mosquitto.conf) --
    mbedtls_ssl_config_defaults(&_conf,
                                MBEDTLS_SSL_IS_CLIENT,
                                MBEDTLS_SSL_TRANSPORT_STREAM,
                                MBEDTLS_SSL_PRESET_DEFAULT);
    mbedtls_ssl_conf_rng(&_conf, mbedtls_ctr_drbg_random, &_drbg);

    // Force TLS 1.2 -- broker mosquitto.conf: tls_version tlsv1.2
    mbedtls_ssl_conf_max_tls_version(&_conf, MBEDTLS_SSL_VERSION_TLS1_2);
    mbedtls_ssl_conf_min_tls_version(&_conf, MBEDTLS_SSL_VERSION_TLS1_2);

    // Verify server cert, and present our client cert (mTLS)
    mbedtls_ssl_conf_authmode(&_conf, MBEDTLS_SSL_VERIFY_REQUIRED);
    mbedtls_ssl_conf_ca_chain(&_conf, &_ca_crt, NULL);

    int ret = mbedtls_ssl_conf_own_cert(&_conf, &_cli_crt, &_pk);
    if (ret != 0) {
      Serial.printf("[TLS] own_cert failed: -0x%04X\n", -ret);
      return 0;
    }

    // -- FIX-1 (REVISED): DO NOT configure sig_algs or sig_hashes here --
    //
    // Root cause analysis of original code:
    //   mbedtls_ssl_conf_sig_algs()  -> only exists with CONFIG_MBEDTLS_SSL_PROTO_TLS1_3=y
    //   MBEDTLS_TLS1_3_SIG_*        -> only defined with TLS 1.3 enabled
    //   mbedtls_ssl_conf_sig_hashes()-> REMOVED in mbedTLS 3.x (Arduino ESP32 core 3.x)
    //
    // For TLS 1.2 with RSA keys, mbedTLS ALWAYS uses PKCS#1 v1.5 by default.
    // RSA-PSS is strictly TLS 1.3 only. Explicitly setting sig algorithms is
    // unnecessary and causes compile failures across mbedTLS versions.
    // Remove the block entirely -- defaults are correct for this use case.

    ret = mbedtls_ssl_setup(&_ssl, &_conf);
    if (ret != 0) {
      Serial.printf("[TLS] ssl_setup failed: -0x%04X\n", -ret);
      return 0;
    }

    // -- TLS DEBUG: disabled in production --
    // Set threshold: 0=off 1=error 2=state_change 3=info 4=verbose
    // To enable: change 0 -> 1 (errors only) or 3 (full trace) and recompile.
    // At threshold=3 generates ~10KB of serial output per handshake.
#if defined(MBEDTLS_DEBUG_C)
    mbedtls_ssl_conf_dbg(&_conf, _tls_debug_cb, NULL);
    mbedtls_debug_set_threshold(0);  // PRODUCTION: 0=silent (change to 1/3 to debug)
#endif

    mbedtls_ssl_set_hostname(&_ssl, host);
    // BioCtx carries self so static _bio_recv can access _rxBuf
    _bioCtx = { &_tcp, &_modem, this };
    mbedtls_ssl_set_bio(&_ssl, &_bioCtx, _bio_send, _bio_recv, NULL);

    // -- Step 3: TLS handshake --
    // NOTE: Network task (taskNetwork) is intentionally NOT subscribed to the
    // ESP-IDF Task Watchdog (TWDT). The task performs blocking operations
    // (4G modem TCP + TLS handshake ~1-3s, MQTT publish with modem I/O) that
    // cannot be interrupted. Subscribing would require resetting the WDT at
    // every blocking point, and any missed reset causes a reboot.
    // Network connectivity loss is handled at the application level via
    // reconnect logic (30s retry interval), not via TWDT.
    uint32_t t0 = millis();

    while ((ret = mbedtls_ssl_handshake(&_ssl)) != 0) {
      if (ret == MBEDTLS_ERR_SSL_WANT_READ) {
        uint32_t tw = millis();
        while (!_tcp.available()) {
          _modem.maintain();
          vTaskDelay(pdMS_TO_TICKS(5));
          // [v16.5a] Do NOT abort here on !_tcp.connected(). That check was a
          // false-trigger source: _tcp.connected() resolves to TinyGSM's
          // modemGetConnected(), which sends AT+CIPCLOSE? and -- when its
          // waitResponse() does not see the expected reply -- falls through
          // an intentionally-disabled early return (upstream TinyGSM
          // "TODO: Why does this not read correctly?", still unfixed in
          // v0.12.0) and stream.parseInt()s whatever is next on the wire,
          // silently clearing sock_connected. During a handshake this UART
          // is shared with _ciprxget()'s own raw reads, so a mid-handshake
          // false negative aborts a healthy connection. A genuinely dead
          // link is still caught by the 15s no-data timeout below (and the
          // 90s overall cap), both wall-clock and independent of modem
          // status. Do not re-add without resolving the upstream defect.
          if (millis() - tw > 15000UL) {
            Serial.println("[TLS] No data from broker (15s) -- handshake stalled");
            return 0;
          }
        }
      } else if (ret == MBEDTLS_ERR_SSL_WANT_WRITE) {
        _modem.maintain();
        vTaskDelay(pdMS_TO_TICKS(5));
      } else {
        char errbuf[80];
        mbedtls_strerror(ret, errbuf, sizeof(errbuf));
        Serial.printf("[TLS] Handshake FAILED (-0x%04X): %s\n", -ret, errbuf);
        if (ret == MBEDTLS_ERR_SSL_CONN_EOF)
          Serial.println("[TLS] -> TCP layer: connection closed by remote");
        else if (ret == MBEDTLS_ERR_X509_CERT_VERIFY_FAILED)
          Serial.println("[TLS] -> AUTH: server cert verify failed (check CA)");
        else if (ret == MBEDTLS_ERR_SSL_FATAL_ALERT_MESSAGE)
          Serial.printf("[TLS] -> ALERT: broker sent fatal alert "
                        "(check CN=%s matches MQTT_CLIENT_ID)\n",
                        host);
        else if (ret == MBEDTLS_ERR_SSL_HANDSHAKE_FAILURE)
          Serial.println("[TLS] -> CIPHER: no common cipher suite");
        return 0;
      }

      if (millis() - t0 > 90000UL) {
        Serial.println("[TLS] Handshake timeout (90s)");
        return 0;
      }
    }

    // Log negotiated cipher suite for diagnostics
    Serial.printf("[TLS] mTLS handshake OK + (%lus) cipher: %s\n",
                  (unsigned long)(millis() - t0) / 1000,
                  mbedtls_ssl_get_ciphersuite(&_ssl));
    _connected = true;
    return 1;
  }

  size_t write(uint8_t b) override {
    return write(&b, 1);
  }

  size_t write(const uint8_t* buf, size_t sz) override {
    if (!_connected) return 0;
#ifdef DEBUG_MQTT_TIMING
    uint32_t t0_tlsw = millis();
#endif
    int ret = mbedtls_ssl_write(&_ssl, buf, sz);
#ifdef DEBUG_MQTT_TIMING
    Serial.printf("[TLS-WRITE] requested=%u written=%d elapsed=%lums\n",
                  (unsigned)sz, ret, (unsigned long)(millis() - t0_tlsw));
#endif
    return ret > 0 ? (size_t)ret : 0;
  }

  int available() override {
    if (!_connected) return 0;
    // FIX-3: mbedtls_ssl_get_bytes_avail() only returns bytes already decrypted
    // from the LAST ssl_read() call. A7670E additionally has the issue where
    // TinyGSM's tcp.available() reports modem buffer count but local rx is empty.
    // Report 1 if EITHER ssl layer OR TCP has pending data -- read() will handle it.
    int pending = mbedtls_ssl_get_bytes_avail(&_ssl);
    if (pending > 0) return pending;
    return (_tcp.available() > 0) ? 1 : 0;
  }

  int read() override {
    uint8_t b;
    return (read(&b, 1) == 1) ? b : -1;
  }

  int read(uint8_t* buf, size_t sz) override {
    if (!_connected) return -1;
    int ret = mbedtls_ssl_read(&_ssl, buf, sz);
    if (ret == MBEDTLS_ERR_SSL_WANT_READ || ret == MBEDTLS_ERR_SSL_WANT_WRITE) return 0;
    if (ret <= 0) {
      _connected = false;
      return -1;
    }
    return ret;
  }

  int peek() override {
    return -1;
  }

  void flush() override {
    _tcp.flush();
  }

  void stop() override {
    if (_connected) {
      mbedtls_ssl_close_notify(&_ssl);
      _connected = false;
    }
    _tcp.stop(BOUNDED_STOP_MS);  // [v16.5d] bounded, was unbounded 15000ms
    _freeSession();
    // Re-init only the per-connection contexts (certs stay parsed)
    mbedtls_ssl_init(&_ssl);
    mbedtls_ssl_config_init(&_conf);
    _rxReset();  // clear local rx buffer for next connection
  }

  // FIX-4: Do NOT call _tcp.connected() here -- that issues an AT command.
  // The MQTT library calls connected() every 100ms (10 AT commands/sec starves data RX).
  // _connected is set false by read()/write() errors, which is sufficient.
  uint8_t connected() override {
    return _connected ? 1 : 0;
  }
  operator bool() {
    return connected();
  }

  // Use this ONLY in the slow 10-second network status check, not in the MQTT poll path.
  bool tcpConnected() {
    return _tcp.connected();
  }

  // v15.4 Fix B: Full mbedTLS context reset ก่อน reconnect ทุกครั้ง
  // เรียกก่อน mqttClient.connect() เพื่อล้าง stale cipher state จาก session เดิม
  // Root cause: 4G IP เปลี่ยน → TCP drop → old sequence numbers → decryption failed
  // stop() reinit เฉพาะตอน TCP close ซึ่งช้าเกินไป -- ต้อง force reset ก่อน attempt
  void resetTLS() {
    // Stop TCP ก่อน (ถ้ายังเปิดอยู่)
    if (_connected) {
      mbedtls_ssl_close_notify(&_ssl);
      _connected = false;
    }
    _tcp.stop(BOUNDED_STOP_MS);  // [v16.5d] bounded, was unbounded 15000ms

    // Free และ reinit per-connection contexts ทั้งหมด
    // (CA cert, client cert, private key ยังคงอยู่ -- parse ครั้งเดียวตอน setupTLS)
    _freeSession();
    mbedtls_ssl_free(&_ssl);
    mbedtls_ssl_config_free(&_conf);
    mbedtls_ssl_init(&_ssl);
    mbedtls_ssl_config_init(&_conf);
    _rxReset();

    Serial.println("[TLS] Context reset complete -- ready for fresh handshake");
  }

private:
  TinyGsm& _modem;
  TinyGsmClient _tcp;
  bool _connected = false;
  bool _certsLoaded = false;

  const char* _ca_pem = nullptr;
  const char* _crt_pem = nullptr;
  const char* _key_pem = nullptr;

  // Persistent mbedTLS contexts -- parsed ONCE, reused on every reconnect
  mbedtls_entropy_context _entropy;
  mbedtls_ctr_drbg_context _drbg;
  mbedtls_x509_crt _ca_crt;
  mbedtls_x509_crt _cli_crt;
  mbedtls_pk_context _pk;

  // Per-connection mbedTLS contexts -- reset on each stop()/connect()
  mbedtls_ssl_context _ssl;
  mbedtls_ssl_config _conf;

  // -- A7670E local RX buffer ------------------------------------------------
  // Root cause (confirmed): A7670E sends "+CADATAIND:0" URC but TinyGSM
  // SIM7600 driver expects "+CIPRXGET:1,0". Data sits in modem hardware buffer
  // (reported by AT+CIPRXGET=4 = available()) but TinyGSM local buffer is empty
  // (read() always returns 0).
  //
  // Previous fix (v1): call AT+CIPRXGET=2,0,N on each bio_recv call.
  // Problem: multiple small calls -> each response has "\r\nOK\r\n" trailer ->
  //          drain(delay=20ms) not sufficient -> "OK\r\n" bytes leak into the
  //          NEXT call's binary stream -> TLS record corrupted ->
  //          MBEDTLS_ERR_PK_INVALID_PUBKEY when parsing server cert public key.
  //
  // This fix (v2): ONE large CIPRXGET fills _rxBuf, bio_recv serves from it.
  //   No AT commands between bio_recv calls = zero corruption risk.
  // -------------------------------------------------------------------------
  static constexpr int RX_BUF_SIZE = 4096;
  uint8_t _rxBuf[RX_BUF_SIZE];
  int _rxHead = 0;  // next byte to serve to mbedTLS
  int _rxTail = 0;  // first free byte (buffered = _rxTail - _rxHead)

  void _rxReset() {
    _rxHead = _rxTail = 0;
  }

  // -- Compact buffer when head advances past halfway --
  void _rxCompact() {
    if (_rxHead == 0) return;
    int rem = _rxTail - _rxHead;
    if (rem > 0) memmove(_rxBuf, _rxBuf + _rxHead, rem);
    _rxHead = 0;
    _rxTail = (rem > 0) ? rem : 0;
  }

  // -- Fill _rxBuf from modem: 2-step CIPRXGET protocol (v3) -------------
  // Bug history:
  //   v1: single CIPRXGET=2 per bio_recv -> "OK\r\n" leaks into next binary read
  //   v2: wrong comma count (3 commas -> "remaining" not "actual")
  //       -> read loop waited 8s for 1460 bytes when only 107 arrived
  //       -> URC +CADATAIND consumed as binary -> data corruption
  //   v3: Step 1 AT+CIPRXGET=4,0 -> exact count; Step 2 fetch exactly that many
  //       -> no timeout, no partial reads, no URC pollution during binary read
  // ---------------------------------------------------------------------
  int _ciprxget(int /*hint*/) {
    _rxCompact();
    int freeSpace = RX_BUF_SIZE - _rxTail;
    if (freeSpace <= 0) return 0;

    // -- Step 1: Query exact bytes in modem hardware buffer --
    SerialAT.printf("AT+CIPRXGET=4,0\r\n");
    int modemCount = _atReadInt("+CIPRXGET:", 2, 3000);
    _drainOK();
    if (modemCount <= 0) return 0;

    // A7670E max per CIPRXGET=2 call: 1460 bytes (one TCP MSS)
    int fetch = min({ modemCount, freeSpace, 1460 });

    // -- Step 2: Fetch exactly 'fetch' bytes --
    SerialAT.printf("AT+CIPRXGET=2,0,%d\r\n", fetch);
    // Response: "+CIPRXGET: 2,0,<actual>,<remaining>\r\n<binary>\r\nOK\r\n"
    // actual = bytes modem actually gives us (may be < fetch if chunk boundary)
    // Comma count for _atReadInt: "+CIPRXGET: 2,0,actual,remaining"
    //   after prefix "+CIPRXGET:": skip 2 commas -> lands on "actual"
    int actual = _atReadInt("+CIPRXGET:", 2, 3000);
    if (actual <= 0) {
      _drainOK();
      return 0;
    }

    // -- Read exactly 'actual' binary bytes into _rxBuf --
    // We know the exact count -- no timeout risk, no URC pollution.
    // 1460 bytes @ 115200 baud = ~127ms. Budget 2s for safety.
    int got = 0;
    uint32_t t = millis();
    while (got < actual && millis() - t < 2000) {
      if (SerialAT.available()) {
        _rxBuf[_rxTail + got] = (uint8_t)SerialAT.read();
        got++;
      } else {
        delayMicroseconds(100);
      }
    }
    _rxTail += got;

    // -- Drain trailing "\r\nOK\r\n" --
    _drainOK();

    // -- HEX DUMP: disabled in production (set to 1 to re-enable during TLS debug) --
#if 0
    {
      static uint32_t s_fetchNo = 0;
      s_fetchNo++;
      int dumpLen = min(got, 32);
      Serial.printf("[CIPRXGET #%lu] +%d/%d bytes (modem had %d)\n",
                    s_fetchNo, got, actual, modemCount);
      Serial.printf("[HEX #%lu] first %d bytes:\n  ", s_fetchNo, dumpLen);
      for (int i = 0; i < dumpLen; i++) {
        Serial.printf("%02X ", _rxBuf[_rxTail - got + i]);
        if ((i + 1) % 16 == 0 && i + 1 < dumpLen) Serial.print("\n  ");
      }
      Serial.println();

      uint8_t* p = _rxBuf + (_rxTail - got);
      if (got >= 5 && p[0] >= 0x14 && p[0] <= 0x17) {
        const char* rec = (p[0] == 0x16) ? "Handshake" : (p[0] == 0x15) ? "Alert"
                                                       : (p[0] == 0x14) ? "ChangeCipherSpec"
                                                                        : "AppData";
        uint16_t rlen = ((uint16_t)p[3] << 8) | p[4];
        Serial.printf("[HEX #%lu] TLS: %s(0x%02X) ver=%02X%02X reclen=%u\n",
                      s_fetchNo, rec, p[0], p[1], p[2], rlen);
        if (p[0] == 0x16 && got >= 6) {
          const char* hs = "?";
          switch (p[5]) {
            case 0x02: hs = "ServerHello"; break;
            case 0x0B: hs = "Certificate"; break;
            case 0x0C: hs = "ServerKeyExchange"; break;
            case 0x0D: hs = "CertificateRequest"; break;
            case 0x0E: hs = "ServerHelloDone"; break;
            case 0x14: hs = "Finished"; break;
          }
          Serial.printf("[HEX #%lu] Handshake: %s(0x%02X)\n",
                        s_fetchNo, hs, p[5]);
        }
        if (p[0] == 0x15 && got >= 7)
          Serial.printf("[HEX #%lu] Alert: lvl=%u desc=%u\n",
                        s_fetchNo, p[5], p[6]);
      } else if (got >= 1) {
        Serial.printf("[HEX #%lu] ! first byte=0x%02X '%c' -- mid-record chunk\n",
                      s_fetchNo, p[0], isprint(p[0]) ? p[0] : '.');
      }
    }
#endif
    return got;
  }

  // -- Helper: read integer from modem AT response after N commas --
  // Waits for a line containing 'prefix', then counts commasBefore commas,
  // returns atoi() of the field that follows.
  int _atReadInt(const char* prefix, int commasBefore, uint32_t timeoutMs) {
    char line[128];
    int llen = 0;
    uint32_t t = millis();
    while (millis() - t < timeoutMs) {
      while (SerialAT.available() && llen < 126) {
        char ch = (char)SerialAT.read();
        if (ch == '\n') {
          line[llen] = '\0';
          const char* p = strstr(line, prefix);
          if (p) {
            p += strlen(prefix);
            int c = 0;
            while (*p && c < commasBefore) {
              if (*p++ == ',') c++;
            }
            return atoi(p);
          }
          llen = 0;
        } else if (ch != '\r') {
          line[llen++] = ch;
        }
      }
      delayMicroseconds(500);
    }
    return -1;
  }

  // -- Helper: drain UART until 80ms of silence --
  void _drainOK() {
    uint32_t quiet = millis();
    while (millis() - quiet < 80) {
      if (SerialAT.available()) {
        SerialAT.read();
        quiet = millis();
      }
    }
  }

  void _freeSession() {
    mbedtls_ssl_free(&_ssl);
    mbedtls_ssl_config_free(&_conf);
  }

  // -- BioCtx carries self pointer so static callback can access buffer --
  struct BioCtx {
    TinyGsmClient* tcp;
    TinyGsm* modem;
    GsmTLSClient* self;
  };
  BioCtx _bioCtx;

  // ==========================================================================
  // _bio_recv -- Buffered A7670E implementation (v2)
  //
  // Flow:
  //  1. Serve bytes from _rxBuf if available       <- no AT command, fast
  //  2. Pump modem + check tcp.available()
  //  3. Fill _rxBuf via ONE CIPRXGET call          <- single AT round-trip
  //  4. Serve from freshly filled buffer
  //
  // This eliminates the "OK\r\n leaks into next call's binary data" corruption
  // that caused MBEDTLS_ERR_PK_INVALID_PUBKEY in v1.
  // ==========================================================================
  static int _bio_recv(void* ctx, unsigned char* buf, size_t len) {
    BioCtx* c = (BioCtx*)ctx;
    GsmTLSClient* self = c->self;
#ifdef DEBUG_RXPATH
    // [P3] Blind-poll burst accumulators. Static locals are safe here:
    // _bio_recv() is only ever entered from mbedtls_ssl_read()/write() running
    // on taskNetwork -- never from an ISR, never from Core 0.
    static uint32_t s_dbgWaitStartMs = 0;
    static uint32_t s_dbgWaitCount   = 0;
#endif

    // -- Step 1: Serve from local buffer --
    int buffered = self->_rxTail - self->_rxHead;
    if (buffered > 0) {
      int n = (int)min((size_t)buffered, len);
      memcpy(buf, self->_rxBuf + self->_rxHead, n);
      self->_rxHead += n;
      if (self->_rxHead >= self->_rxTail) self->_rxReset();
      return n;
    }

    // -- Step 2: Pump modem, check for data --
    for (int i = 0; i < 5; i++) c->modem->maintain();
    int avail = c->tcp->available();
    if (avail <= 0) {
#ifdef DEBUG_RXPATH
      // [P3] Accumulate the no-data poll but emit NOTHING. This branch was
      // taken 871 times in a single Phase 14 wait; a printf per call would
      // dominate the very interval being measured (observer effect).
      if (s_dbgWaitCount == 0) s_dbgWaitStartMs = millis();
      s_dbgWaitCount++;
#endif
      return MBEDTLS_ERR_SSL_WANT_READ;
    }

#ifdef DEBUG_RXPATH
    // [P3] Data has appeared -- emit ONE line summarising the preceding blind
    // spin. spin_ms is the interval Phase 14 could not attribute: time during
    // which c->tcp->available() kept reporting 0 while (per pcap) the bytes
    // were already in the modem.
    uint32_t dbgSpins   = s_dbgWaitCount;
    uint32_t dbgSpinMs  = s_dbgWaitCount ? (millis() - s_dbgWaitStartMs) : 0;
    uint32_t dbgFetchT0 = millis();
    s_dbgWaitCount = 0;
#endif
    // -- Step 3: Fill local buffer via one CIPRXGET call --
    int fetched = self->_ciprxget(avail);
#ifdef DEBUG_RXPATH
    Serial.printf("[P3 bio_recv] t=%lu avail=%d fetched=%d spins=%lu spin_ms=%lu fetch_ms=%lu\n",
                  (unsigned long)dbgFetchT0, avail, fetched,
                  (unsigned long)dbgSpins, (unsigned long)dbgSpinMs,
                  (unsigned long)(millis() - dbgFetchT0));
#endif
    if (fetched <= 0) return MBEDTLS_ERR_SSL_WANT_READ;

    // -- Step 4: Serve from freshly filled buffer --
    int n = (int)min((size_t)fetched, len);
    memcpy(buf, self->_rxBuf + self->_rxHead, n);
    self->_rxHead += n;
    if (self->_rxHead >= self->_rxTail) self->_rxReset();
    return n;
  }

  // -- TLS debug callback -- routes mbedTLS trace to Arduino Serial --
  static void _tls_debug_cb(void* /*ctx*/, int level,
                            const char* file, int line, const char* str) {
    const char* f = strrchr(file, '/');
    if (!f) f = strrchr(file, '\\');
    f = f ? f + 1 : file;
    Serial.printf("[mbedTLS L%d] %s:%d: %s", level, f, line, str);
  }

  static int _bio_send(void* ctx, const unsigned char* buf, size_t len) {
    BioCtx* c = (BioCtx*)ctx;
    size_t written = c->tcp->write(buf, len);
    return (written > 0) ? (int)written : MBEDTLS_ERR_SSL_WANT_WRITE;
  }
};

// Global TLS client -- ESP32 mbedTLS over 4G modem TCP (same engine as WiFiClientSecure)
GsmTLSClient gsmClient(modem);

// MQTT TX/RX buffer must be larger than the biggest PUBLISH packet:
// Phase 3 JSON payload <= 2600 bytes + topic (~55) + MQTT header (4) + QoS1 msgid (2) = ~2661 bytes
// Set to 2800 for comfortable headroom.
MQTTClient mqttClient(2800);  // Phase 5: 2800->3400 (fault_score+uncertainty+final_state+alarm_class+weights)

// Display and Modbus
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE, I2C_SCL_PIN, I2C_SDA_PIN);
ModbusMaster modbus;

// [Task 4.1] FIFO transport binding -- global/static storage, matching
// Uart485Transport_Init()'s own documented lifetime precondition ("must
// back ctx with static/global storage, never a stack-local"). Bound once
// in setup() via Uart485Transport_Init(&g_fifoTransport, &SerialRS485).
// [Broker, Commit 1] The commissioning trigger in taskModbusRead() now
// enqueues onto queueFifoTrigger, and taskModbusRead()'s drain block calls
// FifoDriver_Request() -- so this DOES touch the RS485 bus once the
// commissioning gate's runtime conditions are met, not never.
FifoTransport g_fifoTransport;

// [Task 4.2A -- TEMPORARY DIAGNOSTIC ONLY] Type-only declaration, placed
// before the file's first function definition. The .ino auto-prototype
// generator collects forward declarations for every function in the file
// (including FifoDiag_SetBusOwner(), defined later near taskModbusRead())
// and inserts them all together near the top of the file -- so any
// enum/struct used as a parameter/return type must already be visible at
// that insertion point, not merely before its own function body (CLAUDE.md's
// documented auto-prototype hazard). Intended to be removed once Task 4.2A
// concludes.
enum class BusOwnerDiag : uint8_t { NONE, MODBUS, FIFO };

// RTC
RTC_DS3231 rtc;

// MQTT Topic (built from identity defines at compile time)
static char g_mqttTopic[128];  // populated in setup()

// ============================================================================
// ENUMERATIONS
// ============================================================================

typedef enum {
  STATE_NORMAL = 0,
  STATE_WARNING,
  STATE_CRITICAL,
  STATE_MAINTENANCE,
  STATE_WARMUP
} MachineState_t;

// [Phase 3J] vibration_status as a compact code, so a buffered slot can carry
// the CAPTURE-TIME status instead of a replay having to re-derive it from
// globals that have since moved on. The three values are exactly the three the
// live serializer has always published -- no meaning is added or changed here.
typedef enum {
  VIB_STATUS_OK               = 0,
  VIB_STATUS_UNAVAILABLE      = 1,
  VIB_STATUS_THRESHOLDS_UNSET = 2
} VibrationStatus_t;

typedef enum {
  PAGE_MACHINE = 0,
  PAGE_AXIS,
  PAGE_NETWORK,
  PAGE_MAX
} DisplayPage_t;

typedef enum {
  BTN_NONE = 0,
  BTN_SHORT_PRESS,
  BTN_LONG_PRESS,
  BTN_VLONG_PRESS
} ButtonEvent_t;

typedef enum {
  MODEM_STATE_OFF = 0,
  MODEM_STATE_INITIALIZING,
  MODEM_STATE_SEARCHING,
  MODEM_STATE_REGISTERED,
  MODEM_STATE_GPRS_CONNECTING,
  MODEM_STATE_GPRS_CONNECTED,
  MODEM_STATE_ERROR
} ModemState_t;

// Motor run-state -- the motor's BUSINESS state, same meaning regardless of source
typedef enum {
  MOTOR_STOPPED  = 0,   // Confirmed not running
  MOTOR_STARTING = 1,   // Evidence active, not yet past warm-up debounce
  MOTOR_RUNNING  = 2,   // Evidence active continuously past warm-up debounce
  MOTOR_STOPPING = 3    // Evidence inactive/stale, not yet past stop debounce
} MotorRunState_t;

// [vNext] Evidence source selector for the Motor State Machine.
// MOTOR_SRC_RPM and MOTOR_SRC_CURRENT are both fully implemented,
// independent evidence paths (see buildMotorStateEvidence()); PROXIMITY
// remains a placeholder that currently falls back to the RPM computation.
enum MotorStateSource {
  MOTOR_SRC_RPM,
  MOTOR_SRC_CURRENT,
  MOTOR_SRC_PROXIMITY
};

// [ADR Option E, Phase 1 -- minimal] Evidence-availability abstraction.
// Lets an acquisition-layer caller (taskModbusRead) tell
// buildMotorStateEvidence() that a source's data is expectedly unavailable
// this cycle (e.g. RS485 bus owned by FifoDriver) without the evidence
// builder needing to know why -- keeps buildMotorStateEvidence() unaware of
// FifoDriver specifically. Phase 1 supports only these two values;
// UNAVAILABLE_FAULT is deliberately not introduced yet.
enum class EvidenceAvailability {
  VALID,
  UNAVAILABLE_EXPECTED
};

// [Commit 3] Semantic evidence the state machine acts on -- decouples
// updateMotorStateMachine() from any specific sensor. Each source translates
// its own raw measurement into these two facts:
//   signalPresent -- does this source currently observe "motor active"
//                    conditions (RPM: in-band; Current: above threshold)?
//   ageMs         -- time since this source last had a fresh/valid reading.
// Deliberately minimal: no raw values, no thresholds, no source-specific
// config -- those stay entirely inside buildMotorStateEvidence().
struct MotorStateEvidence {
  bool     signalPresent;
  uint32_t ageMs;
  // [P2] Source-specific STOPPING/STOPPED absence thresholds. Each branch of
  // buildMotorStateEvidence() fills these with its own values -- RPM/PROXIMITY
  // set them to ABSENT_STOPPING_MS/ABSENT_STOPPED_MS (unchanged), CURRENT sets
  // them to NO_CURRENT_STOPPING_MS/FORCE_CURRENT_STOPPED_MS. Keeps
  // updateMotorStateMachine() unaware of which source produced the evidence.
  uint32_t absentStoppingMs;
  uint32_t absentStoppedMs;
  // [v16.5.6] Source-specific STOPPING/STOPPED staleness (ageMs) thresholds --
  // same purpose as absentStoppingMs/absentStoppedMs above, but for the other
  // axis: "evidence itself is stale" instead of "evidence is fresh but
  // absent". RPM/PROXIMITY set these to the existing NO_PULSE_STOPPING_MS/
  // FORCE_STOP_TIMEOUT_MS (unchanged). CURRENT derives them from
  // CURRENT_SAMPLE_INTERVAL_MS instead of a hardcoded literal, because
  // MOTOR_SRC_CURRENT's ageMs is time-since-last-CT-poll -- its normal range
  // is set by the CT sensor's own sampling cadence, not by RPM pulse timing.
  // Before this fix, ageMs was compared unconditionally against the
  // RPM-tuned constants regardless of active source, so CURRENT's normal
  // ~500ms sampling cadence alone (with current continuously present) was
  // enough to trip STOPPING every cycle -- see the STARTING<->STOPPING
  // oscillation investigation this fixes.
  uint32_t ageStoppingMs;
  uint32_t ageStoppedMs;
  // [ADR Option E, Phase 1] true when this cycle's evidence source was
  // expectedly unavailable (e.g. FifoDriver owned the bus) -- ageMs above
  // remains the true, honestly-computed elapsed time; this flag is the
  // sole signal updateMotorStateMachine() uses to suppress ageMs/absentMs-
  // based transitions while it is set. Always false for RPM/PROXIMITY.
  bool evidenceFrozen;
};

// [v16.5.4] Improvement 3: explicit freshness for the RPM EMA evidence.
// Written only by processRPM() (Core 0) at the EMA update site; read by
// buildMotorStateEvidence()'s RPM/PROXIMITY branches. Guarantees a stale
// EMA can never produce signalPresent=true:
//   rpm   -- current g_rpmFiltered value (0.0 while invalid)
//   valid -- false until the EMA has been seeded by a valid pulse, and false
//            again once no pulse has been seen for MAX_EMA_INTERVAL_US
//            (long idle / invalidation, Improvement 1)
//   ageMs -- same value as timeSincePulseMs (ms since the last detected
//            pulse, accepted or rejected) -- deliberately the same clock the
//            FSM itself uses, not a second independent one.
// NOTE: evidence.ageMs fed to updateMotorStateMachine() remains
// timeSincePulseMs exactly as before -- RPMEvidence.ageMs is freshness
// bookkeeping only, so STOPPING/STOPPED timing is untouched.
typedef struct {
  float    rpm;
  bool     valid;
  uint32_t ageMs;
} RPMEvidence;

// [v16.3ab] Analysis freeze reason (derived state) — วางไว้ต้นไฟล์เพราะ .ino auto-prototype
// ต้องเห็น type ก่อน function ที่ return มัน (analysisReason)
typedef enum {
  ANA_READY = 0,
  ANA_FRZ_SENSOR_OFFLINE,
  ANA_FRZ_RECONFIG,      // sensor warmup หลัง reconfig
  ANA_FRZ_STOPPED,
  ANA_FRZ_STARTING,      // รวม warmup + rpm ยังไม่ stable in-band
  ANA_FRZ_STOPPING,
} AnalysisReason_t;

// [Phase2] REMOVED: AnalyticsCommand_t / ANALYTICS_NONE / ANALYTICS_CLEAR.
// The only command was CLEAR, whose job was to wipe g_buf* and the legacy EMA.
// Both are gone; Core 0 now clears the temperature ring directly (it owns it).

// ============================================================================
// DATA STRUCTURES
// ============================================================================

// Sensor data (shared between cores)
typedef struct {
  // [Phase2] rms_x/y/z, rms_overall, vel_peak_x/y/z and vel_peak_overall REMOVED.
  // They were the legacy VRMS-register metric. Vibration now comes exclusively
  // from FIFO RAW -> DSP (velocity_rms_* via g_velCarrier).

  // [Phase2] peak_velocity_x/y/z REMOVED. Their only publisher was /sensor,
  // dropped in Phase 1, leaving them write-only. The Modbus read of registers
  // 0x3A-0x3C that produced them is removed with them.

  // --- Sensor-computed features [v15.0/15.1] ---
  // คำนวณภายใน chip จาก 16KHz FIFO ถูกต้องกว่าคำนวณบน ESP32
  float cf_x;            // Acceleration Crest Factor X (reg 0x47 / 1000) -- Peak/RMS acc
  float cf_y;            // Acceleration Crest Factor Y (reg 0x53 / 1000) [v15.1]
  float cf_z;            // Acceleration Crest Factor Z (reg 0x5F / 1000) [v15.1]
  float cf_max;          // max(cf_x, cf_y, cf_z)                         [v15.1]

  // [Phase 1] kurtosis_x/y/z/max + kurtosis_dominant_axis REMOVED -- the sensor's
  // own kurtosis registers fed bearing_alert only, and that pipeline is gone.

  float peak;            // velocity peak hold [mm/s] -- true peak (reset ทุก publish)
  float freq_x;
  float freq_y;
  float freq_z;
  float temperature;
  uint32_t timestamp;
  bool valid;
  // --- RPM / Motor fields (from proximity sensor) ---
  float    rpm;           // Actual filtered RPM (0.0 when stopped)
  uint8_t  motor_state;   // MotorRunState_t: 0=STOPPED,1=STARTING,2=RUNNING,3=STOPPING
  float    runtime_hour;  // Accumulated running hours (NVS persistent)
  uint8_t  prox;          // 1=pulse normal, 0=Fault/?????? pulse

  // --- CTR4A01 current sensor [v16.6a] ---
  float    current_a;     // AC current [A], valid only if current_valid
  bool     current_valid; // true only on cycles where a fresh 500ms CT sample was taken
  // [ADR Option E, Phase 1] VALID when this cycle's poll window actually
  // ran (regardless of whether the 500ms cadence gate fired a real read);
  // UNAVAILABLE_EXPECTED when the whole poll window was skipped because
  // FifoDriver owned the bus. Set in taskModbusRead(), consumed only by
  // buildMotorStateEvidence()'s MOTOR_SRC_CURRENT branch.
  EvidenceAvailability current_availability;

  // [Phase2] raw_x/y/z REMOVED from this struct. The raw VRMS registers are
  // still read in taskModbusRead() as LOCALS and consumed there by the
  // all-zero / Vx / Vy / Vz stuck detection and sensor auto-restart -- they
  // never leave that task and are never used as a vibration measurement.
  uint16_t read_time_ms;  // เวลาที่ใช้ทำ Modbus transaction ทั้งชุด (ms)
  uint16_t poll_interval_ms; // ระยะห่างจริงระหว่าง poll รอบนี้กับรอบก่อน (ms, nominal 250)
  uint8_t  retry_count;   // จำนวน sub-read ที่ fail ในรอบนี้ (0 = ผ่านหมด)
  bool     crc_ok;        // true = ทุก read ผ่าน CRC (มาถึง de-glitch = true เสมอ)

} VibrationData_t;


// System state (shared between cores)
typedef struct {
  MachineState_t state;
  DisplayPage_t currentPage;
  bool alarmAcknowledged;
  bool buzzerActive;
  bool blinkState;
  uint32_t stateEntryTime;
  bool mqttConnected;  // [v16.5] Section 7 Item 4 (design v16.5 §4.2) — Network4G-only writer, dormant until Items 6-8 wire readers
} SystemState_t;

// [v16.5.4] Improvement 2: atomic telemetry snapshot.
// Captured EXACTLY ONCE per state-machine cycle by captureTelemetrySnapshot()
// (Core 0, taskStateMachine) after updateMotorStateMachine() and the
// alarm/health evaluation have completely finished. Guarded by the existing
// mutexVibData. Downstream consumers (MQTT publish, telemetry ring buffer,
// fault-latch replay, OLED, Analytics, 30s status log) copy this ONE struct
// under ONE mutex take instead of assembling their own view from g_vibData +
// g_systemState in separate takes -- eliminating the window where vibration
// data and system state could come from different cycles.
// Top-level named fields are limited to values that (a) have no equivalent
// inside VibrationData_t (health_score, alarm_level -- genuine Business
// Decision outputs, not raw measurements) or (b) have real direct consumers
// that read the scalar rather than vib.<field> (rpm, motor_state -- read by
// taskAnalytics). Every other measurement (vrms x/y/z, crest factor,
// kurtosis, temperature, current, timestamp, valid, ...) is reachable via
// vib.<field> and is deliberately NOT duplicated at the top level -- doing so
// would just be a second copy of the same data with no reader, the same
// write-only-dead-weight problem this patch removes from g_vibData itself.
// Values are byte-for-byte the same data previously read from g_vibData /
// g_systemState -- no field changed meaning, no MQTT schema change.
typedef struct {
  float           rpm;           // == vib.rpm (presentation RPM) -- read directly by taskAnalytics
  uint8_t         motor_state;   // == vib.motor_state (MotorRunState_t) -- read directly by taskAnalytics
  int             health_score;  // Business Decision -- computed by computeHealthScore(), not present in VibrationData_t
  MachineState_t  alarm_level;   // effective g_systemState.state at capture (incl. MAINTENANCE) -- not present in VibrationData_t
  VibrationData_t vib;           // full measurement record -- all other fields (cf_max, temperature, current_a, timestamp, valid, ...) read from here
  bool            currentEvidenceValid;  // [P4-02] telemetry mirror of g_currentEvidenceValid at capture time -- pure copy, no computation
  float           currentFilteredA;      // [v16.5i] telemetry mirror of g_currentFilteredA at capture time -- pure copy, no computation; paired 1:1 with currentEvidenceValid above
} TelemetrySnapshot;

// Network status (Core 1 only) - Modified for 4G
typedef struct {
  bool modemReady;
  bool gprsConnected;
  ModemState_t modemState;
  int16_t signalQuality;  // CSQ value (0-31, 99=unknown)
  int8_t signalPercent;   // Signal strength in %
  uint32_t lastPublishTime;
  uint16_t publishCount;
  uint16_t publishFailures;
  char operatorName[20];
  char imei[20];
} NetworkStatus_t;

// Time synchronization status
typedef struct {
  bool synced;              // Has time been synced at least once?
  bool ntpReachable;        // Was last NTP/network-time fetch successful?
  uint32_t lastSyncMillis;  // millis() of last successful sync
  // [v16.5j] millis() of last check ATTEMPT (success or failure) -- this is
  // what checkAndSyncTime() schedules against. Deliberately separate from
  // lastSyncMillis (success-only): a failed attempt must still advance the
  // retry clock, or the interval gate never engages while sync keeps
  // failing (root cause of the pre-fix busy-retry-every-loop-tick bug).
  uint32_t lastCheckMillis;
  uint32_t syncCount;       // Total successful syncs
  uint32_t syncFailures;    // Total failed sync attempts
  int32_t lastDriftSec;     // Drift detected at last sync (seconds)
  char lastSyncTime[25];    // Human-readable last sync time (ISO 8601)
} TimeSyncStatus_t;

// Display update command
typedef struct {
  DisplayPage_t page;
  MachineState_t state;
  bool forceUpdate;
} DisplayCommand_t;

// Maintenance reset event (Button task -> Network task via queue)
// Carries all data needed for MQTT audit publish, built on Button stack-free.
typedef struct {
  uint32_t triggerMillis;   // millis() when reset was triggered
  bool     rtcValid;        // snapshot of g_rtcValid at trigger time
  uint16_t year;            // RTC fields (copied to avoid RTC access on Network task)
  uint8_t  month;
  uint8_t  day;
  uint8_t  hour;
  uint8_t  minute;
  uint8_t  second;
} MaintenanceEvent_t;

// [v16.5] Section 7 Item 3 — outbound MQTT queue message (design v16.5 §4.1).
// Independent of g_telemBuf/mutexTelemBuf (Q1 decision). Producer (Analytics)
// and consumer (Network4G) are wired in later checklist items; dormant here.
typedef enum {
  MQTT_OUTBOUND_TOPIC_TREND = 0,   // only topic routed through this queue (design v16.5 §3.2)
  MQTT_OUTBOUND_TOPIC_EVENT = 1,   // [Result Consumer, Commit 2] SDS SS18.3 /event -- FIFO capture metadata
} MqttOutboundTopic_t;

typedef struct {
  MqttOutboundTopic_t topic_id;
  char                payload[MQTT_OUTBOUND_PAYLOAD_MAX];
  size_t              len;
  uint8_t             qos;
} MqttOutboundMsg_t;

// [Broker, Commit 1] Cross-core/cross-task FIFO trigger intent -- the SDS
// SS14.1-specified "xQueueSend/xQueueReceive, depth 1" request-submission
// mechanism, hosted here in the .ino rather than in fifo_driver.cpp because
// fifo_driver.cpp has zero #include dependencies by design (host-test
// compatibility, fifo_driver.h's own documented contract) and structurally
// cannot hold a FreeRTOS queue. Depth 1 is deliberate, not a starting size:
// xQueueSend(...,0) is non-blocking and drops a new intent if one is
// already queued, so a second producer while one is in flight is discarded
// rather than accumulated (SDS SS14.2's own stated rationale for depth 1).
// Producer: any task, any core (SDS A-2). Consumer: taskModbusRead() only --
// the ONLY function in this firmware permitted to call FifoDriver_Request().
// [ARCH-INVARIANT] docs/FIFO_TRIGGER_BROKER_INVARIANTS.md -- every trigger
// producer enqueues one of these; none constructs a FifoCaptureRequest or
// calls the driver directly.
typedef struct {
  FifoTriggerSource source;
  char              tag[FIFO_TAG_MAXLEN];
  bool              requirePermissive;
} FifoTriggerIntent_t;

// ----------------------------------------------------------------------------
// [Phase 3B] Core 0 -> Core 1 waveform hand-off.
//
// WHY A COPY AT ALL: FifoCaptureResult.x/y/z are non-owning views into
// FifoArena, valid ONLY between TryAcquireResult() and ReleaseResult(). The
// Phase 3B architecture mandates "copy raw samples, release the FIFO result
// immediately, NEVER run DSP while holding the result" -- so Core 0 memcpy's
// the 6144 B waveform out of the arena and releases, and Core 1 does every
// floating-point operation against this copy. The driver is therefore never
// blocked by analytics, and the arena is never held across a DSP pass.
//
// The buffer is file-scope static, NOT stack: 6144 B would blow taskAnalytics'
// 6144 B stack outright and leaves taskModbusRead (708 B high-water headroom
// observed in the field) no margin whatsoever.
//
// Concurrency contract:
//   - g_accelSnapMutex guards g_accelSnap's contents.
//   - Core 0 (taskModbusRead) takes it with timeout 0 and SKIPS the capture if
//     it cannot get it instantly. The Modbus task is the time-critical one and
//     must never wait on analytics; a dropped waveform is acceptable (FIFO is
//     best-effort by Phase 3 decision #3), a stalled Modbus poll is not.
//   - queueAccelSnapshot (depth 1) carries only the captureId as the "ready"
//     notification -- the bulk samples travel via the mutex-guarded buffer, per
//     CLAUDE.md's cross-core rule (struct => mutex/queue, never a shared bool).
// ----------------------------------------------------------------------------
typedef struct {
  int16_t  x[VIB_ACCEL_REQUIRED_SAMPLES];
  int16_t  y[VIB_ACCEL_REQUIRED_SAMPLES];
  int16_t  z[VIB_ACCEL_REQUIRED_SAMPLES];
  uint32_t captureId;
  uint16_t sampleCount;
  uint32_t srHz;
  // [Phase 3E] RPM measured at the instant this waveform was captured, carried
  // WITH the waveform for the same reason srHz is: the harmonic bands are
  // defined by the shaft speed, and Core 1 analyses this buffer ~1-2 s after
  // Core 0 filled it. Reading a live RPM at analysis time would pair a
  // spectrum with a speed it was never measured at. 0.0f means "not known",
  // which the DSP treats as "1x/2x not observable", never as "0 RPM".
  float    rpmAtCapture;
} AccelSnapshot_t;

typedef struct {
  uint32_t captureId;   // notification only; payload lives in g_accelSnap
} AccelSnapshotReady_t;

// ----------------------------------------------------------------------------
// [M1A] Core 1 -> Core 0 velocity carrier.
//
// Direction is the REVERSE of the Phase 3B/3C waveform hand-off: velocity RMS
// is produced on Core 1 (taskAnalytics) but the alarm state machine, health
// score and fault latch all live on Core 0 (taskModbusRead). This struct is
// the only channel between them.
//
// Per CLAUDE.md a multi-word struct crossing cores must go through a mutex --
// a shared bool or a bare float would not be safe here because `overall`,
// `valid` and `timestampMs` must be read as ONE consistent set. Reading a
// fresh timestamp against a stale value (or vice versa) is exactly the
// tearing that would let a stale reading masquerade as current.
//
// timestampMs is millis() AT PUBLISH TIME on Core 1, so freshness is measured
// against when the value was computed, not when it was read.
//
// Initial state is deliberately valid=false: before the first successful
// capture the product is in VIBRATION_UNAVAILABLE, not "0.0 mm/s, healthy".
// ----------------------------------------------------------------------------
typedef struct {
  float    overall;      // velocity_rms_overall [mm/s]
  bool     valid;        // false => VIBRATION_UNAVAILABLE, NOT "zero vibration"
  uint32_t captureId;
  uint32_t timestampMs;  // millis() when produced on Core 1

  // [M1B-6] Per-axis values and capture provenance, added so taskNetwork's
  // pushTelemBuf() can populate an outage slot WITHOUT reading the M1B-1 ring.
  // The ring has no mutex and is safe only inside taskAnalytics; this carrier
  // already crosses exactly that boundary and already owns mutexVelCarrier, so
  // extending it adds no new lock and no new cross-task path.
  float    x;            // velocity_rms_x [mm/s]
  float    y;
  float    z;
  uint32_t sampleRateHz; // provenance of the capture these values came from
  uint16_t sampleCount;

  // [Phase 3D] Dominant frequency per axis, from the SAME capture and the SAME
  // spectrum as the velocity figures above -- same captureId, same timestamp,
  // no second FIFO read. Appended, so no existing reader is disturbed.
  // Each *Valid flag is the sole authority for its paired _hz field: when it is
  // false the value is 0.0f meaning "not computed", never "0 Hz measured".
  float    domFreqX;     // [Hz]
  float    domFreqY;
  float    domFreqZ;
  bool     domFreqXValid;
  bool     domFreqYValid;
  bool     domFreqZValid;
  // [Phase 3G] Crest factor from the SAME capture as every field above. It is
  // an ACCELERATION quantity, so its validity is independent of `valid` (which
  // is velocity validity) -- crestFactorValid is the only flag that governs it.
  float    crestFactor;
  bool     crestFactorValid;
} VelocityCarrier_t;

// ----------------------------------------------------------------------------
// [ADR-0006 D-1/D-9, Phase 2] PERIODIC FIFO PRODUCER -- the SOLE initiator of
// FIFO capture. Runs inside taskModbusRead() (same task as the drain block, so
// no new task, no new core, no new concurrency), enqueues only, never inspects
// driver state, never blocks.
//
// Monotonic deadline, NOT delay()/vTaskDelay(): s_fifoPeriodicNextDueMs is an
// absolute millis() deadline compared with signed wraparound-safe arithmetic.
// On a due tick the deadline advances by exactly one period so the cadence
// stays phase-locked to boot; if the tick was serviced so late that the new
// deadline is already in the past, it is re-based to now + period instead of
// accumulating. That is what makes a missed tick a SKIP and never a backlog:
// two captures can never become due at once (ADR-0006 D-9).
//
// s_fifoPeriodicSuspended satisfies "do not repeatedly enqueue once the
// circuit breaker has opened" WITHOUT the producer reading driver state
// (Trigger Broker invariant #4): the drain block -- which legitimately sees
// every admission verdict -- latches it on ERR_CIRCUIT_OPEN.
//
// [Phase 2D Option A] BOUNDED SELF-RECOVERY. Previously the latch had no
// clearing path at all and FifoDriver_ResetCircuitBreaker() had no caller, so a
// single burst of 5 failed sessions ended ALL vibration acquisition until the
// next reboot -- silently, while every other subsystem reported healthy
// (observed on hardware: 2C-ST 25.6 s from first failure to permanent death,
// then >30 min dead with Modbus/MQTT/heap all nominal). That is not acceptable
// for a monitoring product.
//
// Recovery is deliberately BOUNDED, not free: after a backoff the producer
// makes ONE attempt to reset the breaker and resume. If the fault persists the
// driver simply re-trips (5 more failed sessions) and the backoff DOUBLES, up
// to a 15-minute ceiling. A permanently dead sensor therefore costs a bounded,
// decaying share of the RS485 bus instead of either (a) dying forever or
// (b) retrying every 60 s indefinitely. The backoff resets to base only when a
// capture actually succeeds, so a recovered sensor returns to normal cadence
// immediately.
//
// What is NOT changed: T_COOLDOWN_MS, FIFO_MAX_RETRIES, FIFO_BREAKER_THRESHOLD,
// the S12->S14 trip itself, the 2 s cadence, and the rule that only the drain
// block may set the latch. The breaker still protects the bus exactly as
// before -- this only makes its verdict recoverable instead of terminal.
// ----------------------------------------------------------------------------
#define FIFO_PERIODIC_INTERVAL_MS  2000UL   // ADR-0006: 2 s capture cadence
#define FIFO_SUSPEND_RETRY_BASE_MS   60000UL   // first recovery attempt: 60 s
#define FIFO_SUSPEND_RETRY_MAX_MS   900000UL   // ceiling: 15 min
static uint32_t s_fifoPeriodicNextDueMs = 0;      // absolute millis() deadline
static bool     s_fifoPeriodicSuspended = false;  // latched by the drain block
static uint32_t s_fifoPeriodicEnqueued  = 0;      // diagnostic counters only --
static uint32_t s_fifoPeriodicSkipped   = 0;      // never read by any decision
static uint32_t s_fifoSuspendRetryAtMs  = 0;      // absolute recovery deadline
static uint32_t s_fifoSuspendBackoffMs  = FIFO_SUSPEND_RETRY_BASE_MS;
static uint32_t s_fifoSuspendRecoveries = 0;      // diagnostic counter only

// ============================================================================
// FREERTOS HANDLES
// ============================================================================

// Task Handles
TaskHandle_t taskHandleModbus    = NULL;
TaskHandle_t taskHandleState     = NULL;
TaskHandle_t taskHandleDisplay   = NULL;
TaskHandle_t taskHandleNetwork   = NULL;
TaskHandle_t taskHandleButton    = NULL;
TaskHandle_t taskHandleBuzzer    = NULL;
TaskHandle_t taskHandleAnalytics = NULL;  // Phase 2: analytics task

// Queue Handles
QueueHandle_t queueSensorData = NULL;
QueueHandle_t queueButtonEvent = NULL;
QueueHandle_t queueDisplayUpdate = NULL;
QueueHandle_t queueMaintEvent = NULL;   // V14.4: maintenance reset (Button -> Network)
QueueHandle_t queueMqttOutboundTrend = NULL;  // [v16.5] Section 7 Item 3: dormant, no producer/consumer wired yet
QueueHandle_t queueFifoTrigger = NULL;  // [Broker, Commit 1] FIFO trigger intent (any task -> taskModbusRead), depth 1
// [ARCH-INVARIANT] The Trigger Broker's single admission point. Every
// producer sends here; taskModbusRead()'s drain block is the only reader.
// See docs/FIFO_TRIGGER_BROKER_INVARIANTS.md.

// Mutex Handles
SemaphoreHandle_t mutexVibData    = NULL;
SemaphoreHandle_t mutexSystemState = NULL;
SemaphoreHandle_t mutexI2C        = NULL;

// [v16.3g] RTC_NOW_SAFE: thread-safe wrapper สำหรับ rtc.now()
// DS3231 ใช้ I2C bus เดียวกับ OLED → ต้อง take mutexI2C ก่อนเสมอ
// [v16.3h] year sanity check: ปี 2024-2035 เท่านั้น
// [v16.3s] retry logic: ถ้าอ่านได้ corrupt → retry 2 ครั้ง รอ 2ms ต่อครั้ง
//          ลด log noise จาก I2C collision กับ OLED
//          ถ้า retry ทั้ง 3 ครั้งล้มเหลว → return DateTime(0) และ log ครั้งเดียว
#define RTC_NOW_SAFE(dt) do { \
  bool _rtcOk = false; \
  for (int _retry = 0; _retry < 3 && !_rtcOk; _retry++) { \
    if (_retry > 0) vTaskDelay(pdMS_TO_TICKS(2)); \
    if (xSemaphoreTake(mutexI2C, pdMS_TO_TICKS(10)) == pdTRUE) { \
      (dt) = rtc.now(); \
      xSemaphoreGive(mutexI2C); \
      if ((dt).year() >= 2024 && (dt).year() <= 2035) { \
        _rtcOk = true; \
      } \
    } \
  } \
  if (!_rtcOk) { \
    Serial.println("[RTC] ! Corrupt read after 3 retries -- ignored"); \
    (dt) = DateTime((uint32_t)0); \
  } \
} while(0)
SemaphoreHandle_t mutexModem      = NULL;  // Mutex for modem access
SemaphoreHandle_t mutexFaultLatch = NULL;  // v3 hardened: guards g_fl + g_flCount + the "fault_latch" NVS namespace
SemaphoreHandle_t mutexTelemBuf   = NULL;  // guards g_telemBuf + g_telemBuf* counters
// [Phase 3B] guards g_accelSnap (Core 0 writes / Core 1 reads) -- see AccelSnapshot_t
SemaphoreHandle_t mutexAccelSnap  = NULL;
QueueHandle_t     queueAccelSnapshot = NULL;  // [Phase 3B] depth-1 "waveform ready" notification
static AccelSnapshot_t g_accelSnap;           // [Phase 3B] 6144 B waveform copy + provenance
// [Phase 3C] Core 1's private working copy. Phase 3C forbids holding the FIFO
// snapshot mutex while DSP executes -- with an FFT in the pipeline the hold
// would span milliseconds instead of microseconds, and Core 0 takes that mutex
// with timeout 0, so every capture landing during a DSP pass would be dropped.
// Core 1 therefore memcpy's g_accelSnap -> g_accelWork, releases immediately,
// and computes against g_accelWork with no lock held at all.
// Touched ONLY by taskAnalytics (Core 1), so it needs no lock of its own.
static AccelSnapshot_t g_accelWork;
// [Phase 3B] Waveforms dropped because Core 1 still held the mutex, or the
// depth-1 queue was still full. Counted so a silent loss can never be mistaken
// for "the sensor produced nothing"; internal diagnostic only -- deliberately
// NOT published, since Phase 3B's approved payload field list is closed.
static volatile uint32_t g_accelSnapDropped = 0;

// [M1A] Velocity carrier: written by Core 1, read by Core 0. See VelocityCarrier_t.
SemaphoreHandle_t mutexVelCarrier = NULL;
static VelocityCarrier_t g_velCarrier = { 0.0f, false, 0, 0 };
// [M1A] Latest vibration-availability verdict, for telemetry/display only.
// Written by Core 0 immediately after each alarm evaluation; single writer.
// Starts true: until the first fresh valid velocity arrives the product must
// report UNAVAILABLE rather than imply a healthy reading.
static volatile bool g_vibUnavailable = true;

// ----------------------------------------------------------------------------
// [M1B-5] TTW result, taskAnalytics (writer) -> taskNetwork (reader).
//
// WHY A CARRIER AND NOT A DIRECT READ: TTW needs VibHistory_LatestValid(), but
// the M1B-1 ring has NO mutex -- it is safe today only because every reader
// runs inside taskAnalytics, the same task that writes it. /decision is built
// in taskNetwork, so computing TTW there would create the first cross-task ring
// read and a real tearing risk. Instead taskAnalytics computes and publishes
// these two scalars, exactly mirroring the existing g_trendResult pattern.
// No new mutex is introduced, as required.
//
// ORDERED PUBLICATION (no lock needed, single core, volatile):
//   to VALIDATE   : write hours, THEN status  -- a reader seeing VALID is
//                   guaranteed the hours it then reads were already stored
//   to INVALIDATE : write status, THEN hours  -- a reader never sees VALID
//                   paired with a cleared hours
// Both directions are ordered so no interleaving can pair a VALID status with
// a stale or zeroed number.
// ----------------------------------------------------------------------------
static volatile float   g_ttwHours  = 0.0f;
static volatile uint8_t g_ttwStatus = (uint8_t)VIB_TTW_THRESHOLDS_UNSET;

// ============================================================================
// SHARED VARIABLES (protected by mutex)
// ============================================================================

// [v16.5.4] g_vibData removed: every consumer that used to read it now reads
// g_telemSnapshot (verified via full-file search -- no remaining readers),
// so keeping a second, separately-mutex-guarded copy of the same data would
// be dead-weight duplication (an extra memcpy + mutex acquisition per Core 0
// cycle for a value nothing consumes).
// [v16.5.4] Improvement 2: the one atomic snapshot all consumers read.
// Written only by captureTelemetrySnapshot() (Core 0); guarded by mutexVibData.
static TelemetrySnapshot g_telemSnapshot = { 0 };
static SystemState_t g_systemState = {
  .state = STATE_NORMAL,
  .currentPage = PAGE_MACHINE,
  .alarmAcknowledged = false,
  .buzzerActive = false,
  .blinkState = false,
  .stateEntryTime = 0,
  .mqttConnected = false
};
static NetworkStatus_t g_network = { 0 };
static TimeSyncStatus_t g_timeSync = { 0 };

// ============================================================================
// FAULT LATCH v3 — GLOBALS  [inserted: fault_latch_v3_code.ino SECTION 3]
// Relocated here (vs. the patch's literal "insert after NVS_SAVE_INTERVAL_MS"
// instruction) because g_flPrevState's type, MachineState_t, is not declared
// until the ENUMERATIONS section above this point in the file — the literal
// insertion point in the original patch would not compile.
// Protected by mutexFaultLatch (declared above) for all runtime accesses;
// volatile on `pending` retained for defense-in-depth.
// ============================================================================
static FaultLatch_t g_fl = { FL_TS_UNKNOWN, 0.0f, 0.0f, FL_EVT_NONE, false };
static volatile uint32_t g_flCount = 0u;

// ============================================================================
// TELEMETRY RING BUFFER (Phase 1 — RAM-only, no NVS persistence)
// ============================================================================
// ใช้เก็บ telemetry snapshot ขณะ MQTT/4G offline แล้ว burst-replay เมื่อ reconnect
//
// Sizing:
//   [M1B-7] ตัวเลขเดิม (72 B / 8 640 B) ค้างจากก่อน M1B-6 เพิ่ม 32 B velocity block
//   ทำให้เอกสารรายงาน RAM ต่ำกว่าจริง 3 360 B — แก้ให้ตรงกับ TELEM_SLOT_EXPECTED_SIZE
//   และ static_assert ด้านล่าง ซึ่งเป็นค่าที่ compiler ยืนยันแล้ว
//   Slot size : sizeof(TelemetrySlot_t) = 100 B
//   Slots     : TELEM_BUF_SIZE = 120      (60 min @ 30s/cycle NORMAL state)
//   Total RAM : 120 × 100 B = 12 000 B ≈ 11.7 KB (static .bss — NOT heap-allocated)
//
// Behaviour:
//   • Push  : overwrite oldest slot when full (drop-oldest FIFO)
//   • Replay: burst oldest-first, one slot per taskNetwork() iteration,
//             50 ms delay between slots to avoid flooding broker
//   • Loss  : data is lost if device reboots while disconnected (acceptable)
//
// Thread-safety:
//   mutexTelemBuf guards g_telemBuf[], g_telemBufHead, g_telemBufCount
//   and all g_telemBuf* stat counters.
// ============================================================================

#define TELEM_BUF_SIZE 120  // 60 min × 1 slot/30s = 120 slots max

typedef struct {
  uint32_t buffered_ts;       // Unix epoch (seconds) when snapshot was captured
                              // (from rtc.now().unixtime(), or 0 if RTC unavailable)
  // [Phase2] rms_overall / rms_x/y/z / vel_peak_* REMOVED -- the replay payload
  // no longer carries legacy VRMS/VPEAK (see the replay serializer).
  float    temperature;       // °C
  float    cf_max;            // crest factor max
  float    freq_x;            // Hz dominant frequency per axis
  float    freq_y;
  float    freq_z;
  float    rpm;               // filtered RPM
  uint8_t  motor_state;       // MotorRunState_t (0=STOPPED … 3=STOPPING)
  uint8_t  machine_state;     // MachineState_t cast to uint8_t
  uint8_t  prox;              // rotation signal ok flag
  // [Phase 3J] VibrationStatus_t at capture time. Deliberately placed here: the
  // three uint8_t above left one byte of alignment padding before the velocity
  // floats, and this field occupies exactly that pad -- sizeof(TelemetrySlot_t)
  // stays 64 B, so the 120-slot ring costs not one additional byte of RAM.
  // NOTE: because the size is unchanged, TELEM_SLOT_EXPECTED_SIZE below cannot
  // flag this edit. That is intended, not an oversight.
  uint8_t  vibration_status;  // VibrationStatus_t cast to uint8_t

  // ── [M1B-6] FIFO-DSP velocity, ADDITIVE ────────────────────────────────
  // Every legacy field above keeps its exact prior meaning and its existing
  // isRunningBuf gate until M1B-8. These are new fields, not a redefinition.
  //
  // INVALID IS NaN, NEVER 0.0f. The legacy rms_* fields above store literal
  // 0.0 when the motor is stopped; that convention is deliberately NOT copied
  // here. A stored 0.0 is a plausible-looking vibration reading that silently
  // biases anything that aggregates it, whereas NaN cannot be mistaken for a
  // measurement. velocity_data_valid is the sole authority, and the replay
  // serializer omits the velocity_rms_* keys entirely when it is false.
  float    velocity_rms_x;       // [mm/s] NaN when velocity_data_valid == false
  float    velocity_rms_y;
  float    velocity_rms_z;
  float    velocity_rms_overall;

  uint32_t capture_id;           // FIFO capture that produced the values
  uint32_t capture_ts_ms;        // producer millis() at capture completion.
                                 // Carried IN ADDITION to buffered_ts because
                                 // buffered_ts is the RTC second at PUSH time
                                 // and is 0 whenever the RTC is invalid -- the
                                 // very condition (power loss) that tends to
                                 // accompany an MQTT outage. capture_ts_ms is
                                 // monotonic and RTC-independent, so replayed
                                 // samples stay orderable either way.
  uint32_t sample_rate_hz;
  uint16_t sample_count;

  bool     velocity_data_valid;  // sole authority for the four floats above
  uint8_t  schema_version;       // 1 == this migration
} TelemetrySlot_t;

// [M1B-6] Layout guard. The size is the COMPILER-REPORTED value, not an
// assumed packing. If a future edit reorders or adds a field, this fails the
// build instead of silently changing the RAM footprint of the 120-slot buffer
// or the bytes memcpy'd on replay.
// [Phase2] 100 B -> 68 B: the eight legacy VRMS/VPEAK floats (rms_overall,
// rms_x/y/z, vel_peak_x/y/z, vel_peak_overall) were removed from the slot.
// [Phase 1] 68 B -> 64 B: kurtosis_max (float) and kurtosis_axis (uint8_t) were
// removed with the legacy bearing pipeline. Only the float shrinks the record --
// the uint8_t sat in the motor_state/machine_state/prox group and its byte is
// absorbed by the padding that already aligned the velocity floats behind it.
// Saves 4 B x TELEM_BUF_SIZE (120) = 480 B of static RAM.
#define TELEM_SLOT_EXPECTED_SIZE 64u
static_assert(sizeof(TelemetrySlot_t) == TELEM_SLOT_EXPECTED_SIZE,
              "TelemetrySlot_t size changed unexpectedly");

// [M1B-6] Schema version carried on every replayed record.
#define TELEM_SLOT_SCHEMA_VERSION 2u   // [Phase 3J] 1 -> 2: replay payload contract changed

// Ring buffer storage (static — data segment, not heap)
static TelemetrySlot_t  g_telemBuf[TELEM_BUF_SIZE];
static volatile uint8_t g_telemBufHead  = 0;   // index of next slot to write
static volatile uint8_t g_telemBufCount = 0;   // slots currently occupied (0..TELEM_BUF_SIZE)

// Diagnostic counters (cumulative, never reset)
static volatile uint32_t g_telemBufOverflowCount = 0;  // times oldest slot was overwritten
static volatile uint32_t g_telemBufReplayedCount = 0;  // cumulative successfully replayed
static volatile uint32_t g_trendEnqueueDropCount = 0;  // [v16.5] Section 7 Item 3: dormant — incremented only once callers exist
static bool           g_flPrevBearing   = false;
static bool           g_flPrevHealthLow = false;
static MachineState_t g_flPrevState     = STATE_NORMAL;

// Statistics (atomic operations, no mutex needed)
static volatile uint32_t g_sensorReads = 0;
static volatile uint32_t g_sensorErrors = 0;
static volatile uint32_t g_displayUpdates = 0;

// v15.3: Reset reason (อ่านจาก hardware ตอน boot, persistent via NVS)
static char     g_resetReasonStr[24] = "UNKNOWN";  // human-readable string
static uint8_t  g_resetReasonCode    = 0;           // esp_reset_reason_t value
static uint32_t g_rebootCount        = 0;           // สะสมข้ามรอบ (NVS)

// Modbus offline detection
static volatile uint8_t  g_modbusConsecErrors = 0;  // ??? error ?????????
static volatile bool     g_sensorOffline      = false; // true = sensor ?????/??????????
static volatile uint8_t  g_sensorWarmupReads  = 0;   // [v16.3b] suppress spike N reads หลัง sensor กลับ online
// [vNext] TEMPORARY: reads remaining to suppress FAULT_LATCH after RUNNING transition
// (settling transient, distinct from the sensor-reconfig warmup above) -- FAULT_LATCH
// qualification only, nothing else reads this.
static volatile uint8_t  g_motorRunFaultLatchHoldoff = 0;

// -- Velocity Peak Holding (Core 0 only -- taskModbusRead writes, taskNetwork reads+resets) --
// v15.0: เก็บ vel_peak_overall (true peak mm/s = raw/100) แทน rms_overall (ที่แปลงแล้ว)
// Access pattern: เขียน Core 0 (250ms) / อ่าน+reset Core 1 (publish interval)
// ใช้ mutex mutexVibData ป้องกัน read+reset จาก Core 1

// ============================================================================
// TREND BUFFER -- Phase 1: Raw Circular Buffer (Core 0 writes / Core 1 reads)
// ============================================================================
//
// Layer 1 (Raw Circular Buffer):
//   ???? sample ??? 250ms ??? taskStateMachine (Core 0)
//   ???? TREND_BUF_SIZE = 240 samples = 60 ?????? @ 4 Hz
//   RAM: 240 x (6 float) x 4 bytes = 5,760 bytes (~5.6 KB)
//
// Layer 2 (Trend Engine -- ???????? publish, Core 1):
//   ??? window ???????? TREND_WINDOW_SAMPLES samples ??? buffer
//   ?????: RMS slope (Linear Regression), Peak spike count,
//           Freq ratio drift (X/Y/Z), Temp slope, TTW estimate
//
// Thread safety:
//   g_trendBuf ???????? Core 0 (taskStateMachine) ????????
//   ??????? Core 1 (taskAnalytics, publishTelemetry) ?????? snapshot ??? head+count
//   ???????? -- float write ???? atomic ?? ESP32 (Xtensa LX7)
// --- Current Trend Buffer (CTR4A01, 500ms cadence) [v16.6a] ---
#define CURRENT_BUF_SIZE           120     // samples (60s @ 2Hz)
#define CURRENT_WINDOW_SAMPLES      60     // samples used for regression (30s window)
#define CURRENT_MIN_SAMPLES         10     // minimum samples before slope reported (5s)
#define CURRENT_SAMPLE_INTERVAL_MS 500     // acquisition cadence (matches CTR4A01_SENSOR.ino SAMPLE_RATE_HZ=2)
#define CURRENT_SAMPLE_INTERVAL_S  0.5f    // same, in seconds (for linRegSlope())

// [v16.6j] Hoisted to file scope -- was a function-local static const inside
// buildMotorStateEvidence() only. Same name, same value, unchanged meaning;
// moved here so publishTelemetry()'s telemetry-freshness check (see
// [v16.6j] CURRENT TELEMETRY FRESHNESS below) can share this single
// definition instead of duplicating the "500ms x 10 = 5s" constant.
constexpr uint32_t CURRENT_EVIDENCE_MAX_AGE_MS = CURRENT_SAMPLE_INTERVAL_MS * 10;

// [Commit 4A] EMA smoothing for the Current evidence path -- CTR4A01's raw
// Modbus reading has zero existing filtering (single instantaneous sample
// every 500ms). Alpha matches RPM_SMOOTH_ALPHA (0.25) deliberately, for two
// reasons: (1) consistency -- both evidence paths use the same smoothing
// idiom rather than inventing a second, arbitrarily-different one; (2) time
// constant -- ~1/alpha = 4 samples to reach ~63% of a step change, settling
// in roughly 1-2s, fast enough not to meaningfully compound the existing
// 15s/30s confirmed-absence timeouts, while still absorbing single-sample
// noise and inrush transients before the threshold comparison.
#define CURRENT_EMA_ALPHA           0.25f

// ============================================================================
// TEMPERATURE HISTORY RING  [Phase2]
// ============================================================================
// Replaces g_trendBuf, which mixed four unrelated concerns in one struct
// (legacy VRMS rms, legacy VPEAK peak, temperature, freq_ratio_x/y/z). The
// first, second and fourth are gone; temperature is the only survivor and it
// gets a plain float ring -- 240 samples @ 4 Hz = 60 s of history, 960 B
// (against 5.7 KB for the struct it replaces).
// Written by Core 0 (taskStateMachine), read by Core 1 (calcTemperatureTrend).
// Plain float array + volatile head/count -- same cross-core convention as the
// current ring below; float and uint16 reads are atomic on Xtensa.
// ============================================================================
#define TEMP_BUF_SIZE        240    // samples (60 s @ 4 Hz)
#define TEMP_WINDOW_SAMPLES  120    // samples in the slope window (30 s)
#define TEMP_MIN_SAMPLES      20    // minimum before a slope is reported (5 s)

static float             g_tempBuf[TEMP_BUF_SIZE];
static volatile uint16_t g_tempHead  = 0;
static volatile uint16_t g_tempCount = 0;

// ── [v16.3v] NVS Provisioning (relocated after struct definitions) ──
// ── [v16.3v] NVS Provisioning System ─────────────────────────────────────
// เก็บ config ที่เปลี่ยนได้โดยไม่ต้อง recompile
// Namespace: "prom_cfg"
// Keys: cfg_plant, cfg_machine, cfg_sensor, cfg_rpm, cfg_apn
// ตั้งค่าผ่าน Serial ใน Config Mode (กด Enter ภายใน 5 วินาทีหลัง boot)
// ──────────────────────────────────────────────────────────────────────────
#define CFG_NS          "prom_cfg"
#define CFG_KEY_PLANT   "cfg_plant"
#define CFG_KEY_MACHINE "cfg_machine"
#define CFG_KEY_SENSOR  "cfg_sensor"
#define CFG_KEY_RPM     "cfg_rpm"
#define CFG_KEY_APN     "cfg_apn"
#define CFG_MAGIC_KEY   "cfg_magic"
#define CFG_MAGIC_VAL   0xCF9A01UL   // ถ้า magic ตรง = config ถูก set แล้ว
#define CFG_KEY_TREND   "cfg_trend"  // [v16.3ab] trend persistence policy (0-4)

// [v16.3ab] Point 3: Trend persistence policy — ปรับได้ต่อชนิดเครื่องจักร (pump/conveyor/compressor/mixer)
typedef enum {
  TP_ALWAYS_RESUME = 0,  // ไม่เคย clear (เก็บ trend ตลอด)
  TP_CLEAR_10M,          // หยุด > 10 นาที = clear
  TP_CLEAR_30M,          // หยุด > 30 นาที = clear (default)
  TP_CLEAR_1H,           // หยุด > 1 ชม = clear
  TP_ALWAYS_CLEAR        // clear ทุกครั้งที่หยุด
} TrendPersistence_t;
static TrendPersistence_t g_trendPersistence = TP_CLEAR_30M;

static uint32_t trendClearThresholdMs() {
  switch (g_trendPersistence) {
    case TP_ALWAYS_RESUME: return UINT32_MAX;      // ไม่มีวันเกิน → ไม่ clear
    case TP_CLEAR_10M:     return 10UL*60UL*1000UL;
    case TP_CLEAR_30M:     return 30UL*60UL*1000UL;
    case TP_CLEAR_1H:      return 60UL*60UL*1000UL;
    case TP_ALWAYS_CLEAR:  return 0;               // ทุก stop → clear
  }
  return 30UL*60UL*1000UL;
}
static const char* trendPersistenceStr() {
  switch (g_trendPersistence) {
    case TP_ALWAYS_RESUME: return "always_resume";
    case TP_CLEAR_10M:     return "clear_10m";
    case TP_CLEAR_30M:     return "clear_30m";
    case TP_CLEAR_1H:      return "clear_1h";
    case TP_ALWAYS_CLEAR:  return "always_clear";
  }
  return "?";
}
// แปลง string → policy (สำหรับ config menu). คืน true ถ้า parse ได้
static bool parseTrendPersistence(const String& v, TrendPersistence_t* out) {
  if      (v == "always_resume" || v == "0") *out = TP_ALWAYS_RESUME;
  else if (v == "clear_10m"     || v == "1") *out = TP_CLEAR_10M;
  else if (v == "clear_30m"     || v == "2") *out = TP_CLEAR_30M;
  else if (v == "clear_1h"      || v == "3") *out = TP_CLEAR_1H;
  else if (v == "always_clear"  || v == "4") *out = TP_ALWAYS_CLEAR;
  else return false;
  return true;
}

// Runtime config — โหลดจาก NVS ถ้ามี ไม่มีใช้ default จาก #define
// [v16.3v] ค่า default จะถูกเซตจาก #define ใน loadNvsConfig()
// ไม่ใช้ PLANT_ID/MACHINE_ID โดยตรงเพราะ #define ยังไม่ถูก process ณ จุดนี้
static char g_cfgPlant[32]   = "plant01";
static char g_cfgMachine[32] = "pump01";
static char g_cfgSensor[16]  = "vb01";
static int  g_cfgRpm         = 1500;
static char g_cfgApn[32]     = "internet";

// โหลด config จาก NVS ถ้ามี (เรียกใน setup() ก่อน task create)
static void loadNvsConfig() {
  Preferences p;
  if (!p.begin(CFG_NS, true)) return;   // read-only
  uint32_t magic = p.getUInt(CFG_MAGIC_KEY, 0);
  if (magic != CFG_MAGIC_VAL) {
    p.end();
    Serial.println("[CFG] No NVS config found — using firmware defaults");
    return;
  }
  strncpy(g_cfgPlant,   p.getString(CFG_KEY_PLANT,   g_cfgPlant).c_str(),   sizeof(g_cfgPlant)-1);
  strncpy(g_cfgMachine, p.getString(CFG_KEY_MACHINE, g_cfgMachine).c_str(), sizeof(g_cfgMachine)-1);
  strncpy(g_cfgSensor,  p.getString(CFG_KEY_SENSOR,  g_cfgSensor).c_str(),  sizeof(g_cfgSensor)-1);
  g_cfgRpm = p.getInt(CFG_KEY_RPM, g_cfgRpm);
  strncpy(g_cfgApn,     p.getString(CFG_KEY_APN,     g_cfgApn).c_str(), sizeof(g_cfgApn)-1);
  g_trendPersistence = (TrendPersistence_t)p.getInt(CFG_KEY_TREND, (int)g_trendPersistence);  // [v16.3ab]
  p.end();
  Serial.printf("[CFG] NVS config loaded: plant=%s machine=%s rpm=%d apn=%s\n",
                g_cfgPlant, g_cfgMachine, g_cfgRpm, g_cfgApn);
}

// บันทึก config ลง NVS
static bool saveNvsConfig(const char* plant, const char* machine,
                          const char* sensor, int rpm, const char* apn) {
  Preferences p;
  if (!p.begin(CFG_NS, false)) return false;
  // [Task 7.3 -- TEMPORARY DIAGNOSTIC ONLY] NVS write entry/exit markers.
  // NOTE: only reachable via runConfigMode(), an interactive boot-time-only
  // path (setup(), gated on a 5s Serial Enter-key window) -- not reachable
  // during normal runtime/FIFO operation, instrumented for completeness.
  Serial.printf("[NVS_BEGIN] %lu\n", (unsigned long)millis());
  p.putUInt  (CFG_MAGIC_KEY,   0u);               // disarm magic ก่อน
  p.putString(CFG_KEY_PLANT,   plant);
  p.putString(CFG_KEY_MACHINE, machine);
  p.putString(CFG_KEY_SENSOR,  sensor);
  p.putInt   (CFG_KEY_RPM,     rpm);
  p.putString(CFG_KEY_APN,     apn);
  p.putInt   (CFG_KEY_TREND,   (int)g_trendPersistence);  // [v16.3ab] อ่าน global ตรง ไม่ต้องแก้ signature
  p.putUInt  (CFG_MAGIC_KEY,   CFG_MAGIC_VAL);    // arm magic หลังเขียนครบ
  p.end();
  Serial.printf("[NVS_END]   %lu\n", (unsigned long)millis());
  return true;
}

// Config Mode — รับคำสั่งผ่าน Serial
// เรียกจาก setup() ถ้าผู้ใช้กด Enter ภายใน 5 วินาที
static void runConfigMode() {
  // copy ค่าปัจจุบันมาแก้ไข
  char tmpPlant[32], tmpMachine[32], tmpSensor[16], tmpApn[32];
  int  tmpRpm;
  strncpy(tmpPlant,   g_cfgPlant,   sizeof(tmpPlant));
  strncpy(tmpMachine, g_cfgMachine, sizeof(tmpMachine));
  strncpy(tmpSensor,  g_cfgSensor,  sizeof(tmpSensor));
  strncpy(tmpApn,     g_cfgApn,     sizeof(tmpApn));
  tmpRpm = g_cfgRpm;

  Serial.println("\n+========================================================+");
  Serial.println("|           PROMLOGIX CONFIG MODE  (v16.3v)              |");
  Serial.println("+========================================================+");
  Serial.println("| Commands:                                              |");
  Serial.println("|   set plant_id    <value>                              |");
  Serial.println("|   set machine_id  <value>                              |");
  Serial.println("|   set sensor_id   <value>                              |");
  Serial.println("|   set rated_rpm   <value>                              |");
  Serial.println("|   set apn         <value>                              |");
  Serial.println("|   show     — แสดงค่าปัจจุบัน                           |");
  Serial.println("|   save     — บันทึกและ reboot                           |");
  Serial.println("|   cancel   — ออกโดยไม่บันทึก (reboot ปกติ)              |");
  Serial.println("+========================================================+");
  Serial.printf("Current: plant=%s machine=%s sensor=%s rpm=%d apn=%s\n\n",
                tmpPlant, tmpMachine, tmpSensor, tmpRpm, tmpApn);

  String inputBuf = "";
  while (true) {
    Serial.print("[CONFIG] > ");
    inputBuf = "";

    // รอรับ input จนกว่าจะกด Enter
    uint32_t lastChar = millis();
    while (true) {
      if (Serial.available()) {
        char c = Serial.read();
        if (c == '\n' || c == '\r') {
          Serial.println();
          break;
        }
        if (c == 127 || c == 8) {   // backspace
          if (inputBuf.length() > 0) { inputBuf.remove(inputBuf.length()-1); Serial.print("\b \b"); }
        } else {
          inputBuf += c;
          Serial.print(c);
        }
        lastChar = millis();
      }
      // timeout 5 นาที ถ้าไม่พิมพ์อะไร
      if (millis() - lastChar > 300000UL) {
        Serial.println("\n[CONFIG] Timeout — rebooting without saving");
        ESP.restart();
      }
    }

    inputBuf.trim();
    if (inputBuf.length() == 0) continue;

    // parse คำสั่ง
    if (inputBuf == "show") {
      Serial.printf("  plant_id   = %s\n", tmpPlant);
      Serial.printf("  machine_id = %s\n", tmpMachine);
      Serial.printf("  sensor_id  = %s\n", tmpSensor);
      Serial.printf("  rated_rpm  = %d\n", tmpRpm);
      Serial.printf("  apn        = %s\n", tmpApn);
      Serial.printf("  trend_persistence = %s\n", trendPersistenceStr());
    } else if (inputBuf.startsWith("set ")) {
      String rest = inputBuf.substring(4);
      rest.trim();
      int sp = rest.indexOf(' ');
      if (sp < 0) { Serial.println("  Error: format = set <key> <value>"); continue; }
      String key = rest.substring(0, sp);
      String val = rest.substring(sp+1);
      val.trim();
      key.trim();

      if      (key == "plant_id")   { val.toCharArray(tmpPlant,   sizeof(tmpPlant));   Serial.printf("  plant_id   = %s\n", tmpPlant); }
      else if (key == "machine_id") { val.toCharArray(tmpMachine, sizeof(tmpMachine)); Serial.printf("  machine_id = %s\n", tmpMachine); }
      else if (key == "sensor_id")  { val.toCharArray(tmpSensor,  sizeof(tmpSensor));  Serial.printf("  sensor_id  = %s\n", tmpSensor); }
      else if (key == "rated_rpm")  { tmpRpm = val.toInt(); Serial.printf("  rated_rpm  = %d\n", tmpRpm); }
      else if (key == "apn")        { val.toCharArray(tmpApn, sizeof(tmpApn)); Serial.printf("  apn        = %s\n", tmpApn); }
      else if (key == "trend_persistence") {
        TrendPersistence_t tp;
        if (parseTrendPersistence(val, &tp)) { g_trendPersistence = tp; Serial.printf("  trend_persistence = %s\n", trendPersistenceStr()); }
        else Serial.println("  Error: trend_persistence = always_resume|clear_10m|clear_30m|clear_1h|always_clear");
      }
      else { Serial.printf("  Unknown key: %s\n", key.c_str()); }

    } else if (inputBuf == "save") {
      if (saveNvsConfig(tmpPlant, tmpMachine, tmpSensor, tmpRpm, tmpApn)) {
        Serial.println("\n[CONFIG] + Config saved to NVS Flash");
        Serial.printf("  plant_id   = %s\n", tmpPlant);
        Serial.printf("  machine_id = %s\n", tmpMachine);
        Serial.printf("  sensor_id  = %s\n", tmpSensor);
        Serial.printf("  rated_rpm  = %d\n", tmpRpm);
        Serial.printf("  apn        = %s\n\n", tmpApn);
        Serial.println("[CONFIG] Rebooting in 2 seconds...");
        delay(2000);
        ESP.restart();
      } else {
        Serial.println("[CONFIG] ! ERROR: Failed to write NVS");
      }

    } else if (inputBuf == "cancel") {
      Serial.println("[CONFIG] Cancelled — rebooting with existing config");
      delay(500);
      ESP.restart();

    } else {
      Serial.printf("  Unknown command: %s\n", inputBuf.c_str());
    }
  }
}

// ── END NVS Provisioning ───────────────────────────────────────────────────

// --- Current Trend Buffer Globals (Core 0 writes / Core 1 reads) [v16.6a] ---
// Same cross-core convention as g_trendBuf above: plain float array + volatile
// head/count (atomic on Xtensa), no mutex -- single writer (taskStateMachine).
static float              g_currentBuf[CURRENT_BUF_SIZE];
static volatile uint16_t  g_currentHead  = 0;
static volatile uint16_t  g_currentCount = 0;
static volatile uint32_t  g_ctReadErrors = 0;  // [v16.6b] cumulative CTR4A01 Modbus failures since boot
static volatile uint32_t  g_lastCurrentSampleMs = 0;  // [Commit 3] millis() of last SUCCESSFUL CTR4A01 read (0=never); drives MotorStateEvidence.ageMs for MOTOR_SRC_CURRENT

// [P4-02] Telemetry mirror of s_currentEvidenceValid (buildMotorStateEvidence(),
// MOTOR_SRC_CURRENT case). NOT a second source of truth -- written only there,
// read only by captureTelemetrySnapshot(). Plain bool, not volatile: producer
// and consumer both run on Core 0 in taskStateMachine(), same cycle, sequential
// -- no cross-core read of this variable exists (see P4_02_DESIGN_CONTRACT.md §5).
static bool               g_currentEvidenceValid = false;
// [v16.5i] Telemetry mirror of s_currentFiltered (buildMotorStateEvidence(),
// MOTOR_SRC_CURRENT case) -- the held/EMA current magnitude paired with
// g_currentEvidenceValid above. Written at the exact same two update sites,
// in the same statement group, so the two mirrors can never describe
// different cycles. Same no-mutex contract as g_currentEvidenceValid: Core 0,
// same task, same cycle, sequential producer/consumer.
static float              g_currentFilteredA     = 0.0f;

// ============================================================================
// [Phase2] REMOVED: AggSample_t and the g_buf1s / g_buf10s / g_buf60s cascade
// ============================================================================
// 3 x 60 slots x 44 B ~ 7.9 KB of RAM whose only purpose was to feed the legacy
// VRMS trend keys (slope_1s/10s/60s, max_rms_10min/60min, stddev_1min,
// slope_var_*, spike_count). Multi-resolution trend now comes from VibWindow_ /
// VibSlope_ over the M1B-1 velocity history ring.


// --- EMA state (taskAnalytics writes, Core 1 only -- no cross-core issue) ---

// --- Analytics MQTT topic (built at setup) ---
static char g_mqttAnalyticsTopic[128];

// ── MQTT Pipeline Topics (mqtt_pipeline_patch §A) ────────────────────────────
static char g_mqttTopicSensor    [128];  // factory/.../sensor
static char g_mqttTopicDecision  [128];  // factory/.../decision
static char g_mqttTopicTrend     [128];  // factory/.../trend
static char g_mqttTopicEvent     [128];  // factory/.../vibration/event (V14.4 maintenance audit)
static char g_mqttTopicCommand   [128];  // [Commit 7A] factory/.../vibration/command -- inbound, subscribed only
static char g_mqttTopicDeviceHealth[128];  // [P1-S3] factory/.../device-health -- engineering diagnostics, NOT the Product-1 contract
// ─────────────────────────────────────────────────────────────────────────────

// [Commit 7A] MQTT inbound command callback. Fires from mqttClient.loop()
// (taskNetwork(), Core 1, already called unconditionally at ~100ms cadence
// -- no new polling loop introduced) whenever a message arrives on any
// subscribed topic; only g_mqttTopicCommand is ever subscribed (see the
// post-connect subscribe() call site in taskNetwork()), so in practice this
// fires only for command messages. Payload content is logged at LOGD
// (debug) only -- topic and length are logged unconditionally, matching
// this file's existing [FIFO-BROKER]/[LATCH] diagnostic convention.
//
// [Commit 7B] Trigger Broker producer. Reuses the exact enqueue-only
// pattern already established by the commissioning/FAULT_LATCH/
// OPERATOR_BUTTON producers: builds a FifoTriggerIntent_t and calls
// xQueueSend(queueFifoTrigger, ...) -- nothing else. Never calls
// FifoDriver_Request() (taskModbusRead()'s drain block remains the sole
// caller), never reads FifoDriver_GetPhase()/OwnsBus() to pre-judge
// ACTIVE/COOLDOWN (that decision belongs entirely to the driver, exactly
// as for every other producer), and never bypasses the queue. MQTT command
// response/event correlation is explicitly out of scope here (Commit 7C);
// request_id is carried forward only via the intent's existing `tag`
// field, the same mechanism every other producer already uses --
// no new plumbing added.
static void mqttCommandCallback(String &topic, String &payload) {
  Serial.printf("[MQTT-CMD] topic=%s len=%u\n", topic.c_str(), (unsigned)payload.length());
  LOGD("[MQTT-CMD] payload=%s\n", payload.c_str());

  // Validate: size bound before parsing (never hand an unbounded payload to
  // the JSON parser), valid JSON, action=="capture", request_id present.
  if (payload.length() == 0 || payload.length() >= 256) {
    Serial.printf("[MQTT-CMD] REJECTED invalid payload (len=%u, expected 1-255 bytes)\n",
                  (unsigned)payload.length());
    return;
  }

  StaticJsonDocument<256> cmdDoc;
  DeserializationError jsonErr = deserializeJson(cmdDoc, payload);
  if (jsonErr) {
    Serial.printf("[MQTT-CMD] REJECTED invalid JSON (%s)\n", jsonErr.c_str());
    return;
  }

  const char* action = cmdDoc["action"] | "";
  if (strcmp(action, "capture") != 0) {
    Serial.printf("[MQTT-CMD] REJECTED unknown action=\"%s\"\n", action);
    return;
  }

  const char* requestId = cmdDoc["request_id"] | "";
  if (requestId[0] == '\0') {
    Serial.println("[MQTT-CMD] REJECTED missing/empty request_id");
    return;
  }

  // [ADR-0006 D-1, Phase 2A] The REMOTE_ON_DEMAND FIFO producer that stood
  // here has been REMOVED -- periodic SCHEDULED capture is the sole FIFO
  // initiator, so a remote command can no longer start a capture.
  //
  // All command HANDLING above is unchanged: payload-size bound, JSON parse,
  // action=="capture", and request_id validation all still run and still
  // reject with their original messages.
  //
  // The MQTT response contract is preserved, not dropped. Previously a
  // rejected remote request was answered by the drain block via
  // publishMqttRejectionEvent() (Commit 7C); with no enqueue, that path would
  // never be reached and the command would fail SILENTLY. The same helper is
  // therefore called directly here -- same topic, same schema, same
  // request_id echo -- reporting ERR_NOT_PERMITTED, which is precisely what
  // an architecturally-disallowed trigger source is. Non-blocking:
  // publishMqttRejectionEvent() routes through enqueueMqttOutbound().
  Serial.printf("[MQTT-CMD] REJECTED request_id=%s -- remote capture disabled; "
                "periodic capture is the sole FIFO initiator (ADR-0006)\n", requestId);

  char rejTag[FIFO_TAG_MAXLEN];
  memset(rejTag, 0, sizeof(rejTag));
  for (size_t ti = 0; ti < FIFO_TAG_MAXLEN && requestId[ti] != '\0'; ti++) {
    rejTag[ti] = requestId[ti];
  }
  publishMqttRejectionEvent(rejTag, FifoError::ERR_NOT_PERMITTED);
}

// ============================================================================
// TREND ENGINE OUTPUT -- populated by calcTrend(), read by publishTelemetry()
// ============================================================================

// -- TrendResult_t -- complete output struct -------------------------------
typedef struct {
  // [Phase2] Only the two slopes that still have a live source and a live
  // consumer survive. Removed: rms_slope, trend_dir, spike_count, ttw_hours
  // (legacy VRMS/VPEAK), freq_drift_x/y/z + freq_alert (freq_ratio pipeline),
  // slope_1s/10s/60s + slope_ready_* + stddev_1min + max_rms_10min (slot
  // cascade) and ema_dir/ema_rms/ema_delta (legacy EMA).
  float    temp_slope;      // degC per sample -- TEMPERATURE register 0x40
  float    current_slope;   // [v16.6a] A per second (CTR4A01, 500ms samples)
  uint16_t window_samples;  // samples in the temperature window (debug)
} TrendResult_t;

static TrendResult_t g_trendResult = { 0 };  // ?? trend ??????

// ── Slope-readiness thresholds (minimum slots required before a tier's ────
//    slope_*s value is considered trustworthy enough to publish) ──────────

// ── Rolling variance of mean_rms per tier (OSG + FVRI inputs) ─────────────
// Updated by computeRmsVariance() inside taskAnalytics, exposed on /trend.

// [Phase2] REMOVED: PATENT CLAIM 2 -- RPM-adaptive slot duration.
// SLOT_REVS_TARGET / SLOT_DUR_MIN_MS / SLOT_DUR_MAX_MS / computeSlotDurMs()
// and the g_buf1s/10s/60s slot cascade they drove are all gone; they existed
// only to produce the legacy VRMS trend outputs.

static uint8_t  g_anaPublishCnt = 0;

// Per-slot running accumulators (build up the next AggSample_t for buf1s)

// millis() timestamp when the system last entered STATE_WARMUP (used for the
// maintenance-reset MQTT audit event timestamp when RTC is not valid).
static uint32_t g_warmupStartTs = 0;

// RTC valid flag
static bool g_rtcValid = false;

// ============================================================================
// PROXIMITY / RPM -- ISR Variables (volatile, written in IRAM ISR)
// ============================================================================

// (B) MAXIMUM MEASURABLE RPM -- the shortest interval that can still be a real
// revolution. Derived from MAX_RPM, never hardcoded. Consumed by processRPM()'s
// interval gate; a shorter interval than this cannot be a genuine revolution at
// or below MAX_RPM.
static const uint32_t RPM_MIN_INTERVAL_US =
    60000000UL / (MAX_RPM * PULSE_PER_REV);

// (A) MINIMUM PULSE SEPARATION / BOUNCE REJECTION -- a completely separate
// concern from (B), and no longer derived from it.
//
// [v16.5.5] previously set this to RPM_MIN_INTERVAL_US (20,000 us at
// MAX_RPM 3000). That tied bounce rejection to the speed ceiling, so at ~2990
// RPM the real pulse interval (20,033 us nominal) fell inside the debounce
// window and REAL edges were discarded as bounce: 19.3% of revolutions lost,
// each loss producing a ~40 ms interval -> raw RPM ~1500 -> the EMA propagated
// the error down to 1,241 RPM while the shaft was turning at ~2990.
//
// [v16.6-rpmsep] The value comes from the measured bounce distribution, not
// from any speed. Two temporary pulse-capture sessions (since removed) over
// 800 - 3600 RPM, 35,000+ bounce samples, showed the sensor emits a SECOND
// edge at a fixed ANGULAR offset of ~4.5% of one revolution (~16 deg of shaft
// rotation) -- a geometric property of the target, not electrical ringing:
//        885 RPM (period 67,836 us) -> bounce 2,833 us  (4.18%)
//      1,693 RPM (period 45,000 us) -> bounce 1,560 us  (4.40%)
//      3,020 RPM (period 20,000 us) -> bounce   952 us  (4.79%)
//      3,571 RPM (period 16,803 us) -> bounce   833 us  (4.96%)
// Largest bounce ever recorded: 6,581 us, at 390 RPM.
// 8,000 us clears every observed bounce while leaving 1.95x headroom below the
// real pulse at 3600 RPM (16,667 us). Verified on hardware at 2995/3000/3005
// and 3600 RPM: zero real edges rejected, every bounce still rejected.
//
// LOW-SPEED LIMIT -- because the bounce is angular, its delay grows as the
// shaft slows. Extrapolating the measured 4.5% ratio, the second edge reaches
// 8,000 us at roughly 350 RPM and would be accepted as a real pulse below
// that, reporting about twice the true speed. Fine for this machine
// (MIN_RPM_VALID is 300 and the tested range is 588-3600 RPM), but a
// period-proportional debounce would be required to run slower than ~400 RPM.
static const uint32_t RPM_DEBOUNCE_US = 8000UL;

volatile uint32_t g_rpmLastPulseTime  = 0;
volatile uint32_t g_rpmPulseInterval  = 0;
volatile uint32_t g_rpmTotalPulses    = 0;

// RPM processing state (Core 0 only -- no mutex needed)
static float           g_rpmFiltered       = 0.0f;  // EMA evidence signal -- state-machine input only; [v16.5.4] reset (with g_rpmEvidence.valid=false) after a pulse gap/idle > MAX_EMA_INTERVAL_US, otherwise never reset by state
static float           g_rpmReported       = 0.0f;  // presentation value derived from g_rpmFiltered + g_motorRunState -- telemetry/data->rpm source only
// [v16.5.4] Improvements 1+3: RPM EMA freshness state (Core 0 only -- no
// mutex needed). Refreshed every processRPM() cycle from timeSincePulseMs
// (the SAME clock the FSM itself uses for FORCE_STOP_TIMEOUT_MS) rather than
// a second, independently-tracked "time since last accepted EMA update"
// clock -- so EMA freshness and FSM staleness can never diverge. Consumed by
// buildMotorStateEvidence().
static RPMEvidence     g_rpmEvidence        = { 0.0f, false, 0 };
static uint32_t        g_rpmLastPulseCount  = 0;
static uint32_t        g_rpmLastPulseMillis = 0;
static MotorRunState_t g_motorRunState      = MOTOR_STOPPED;
// [vNext] Motor State evidence source -- runtime-mutable (not #define) so a
// future Preferences/NVS-backed config can change it without touching this
// API again. Default RPM preserves current behavior exactly.
// TEST_CURRENT_SOURCE is now defined in the COMMISSIONING CONFIGURATION
// section near the top of the file (Debug / Test Configuration).
#ifdef TEST_CURRENT_SOURCE
static MotorStateSource g_motorStateSource  = MOTOR_SRC_CURRENT;
#else
static MotorStateSource g_motorStateSource  = MOTOR_SRC_RPM;
#endif
static uint32_t        g_runInBandSince      = 0;   // [v16.3z] millis() ที่ rpm เริ่ม in-band ต่อเนื่อง (0=ยังไม่เข้า)
static uint32_t        g_absentSince         = 0;   // [Commit 3A] millis() when signalPresent first became continuously false (0=currently present)
static uint32_t        g_motorStoppedSince   = 0;   // [v16.3aa] millis() ที่เข้า STOPPED (0=ไม่ได้หยุด) — วัดระยะเวลาหยุด
static float           g_tempAtStop          = 0.0f; // [v16.3ad] อุณหภูมิตอนเข้า STOPPED — ใช้ตรวจ cold start ตอน resume
static volatile uint32_t g_lastResumeGapS    = 0;    // [v16.3ab] ระยะเวลา gap ครั้งล่าสุด (วินาที) — ส่งขึ้น telemetry

// ============================================================================
// [v16.5.3-rpmdiag1] DIAGNOSTIC-ONLY mirrors -- NEVER read by any control-flow
// or decision logic anywhere in the firmware. Written by processRPM() purely
// so updateMotorStateMachine()'s diagnostic prints (added in this build only)
// can report pulseCount/rpmRaw/timeSincePulseMs without changing that
// function's signature. Removing this block removes logging only.
// ============================================================================
static uint32_t g_diagPulseCount       = 0;
static float    g_diagRpmRaw           = 0.0f;
static uint32_t g_diagTimeSincePulseMs = 0;
static bool     g_diagPrevSignalPresent = false;  // for [SIGNAL] edge detection
static bool     g_diagSignalPresentInit = false;  // suppress the very first (boot) edge print

// [v16.3ab/ac] Point 1: readiness เป็น derived state ที่ประกอบจาก predicate แยกโดเมน
//   แต่ละโดเมนไม่รู้เรื่องกัน (RPM ไม่รู้เรื่อง sensor, sensor ไม่รู้เรื่อง vrms) — เพิ่มโดเมนใหม่
//   (current/temp/power) = เพิ่ม predicate 1 ตัว + 1 บรรทัดใน analysisReason() ไม่ต้องแก้ที่อื่น
static inline bool anaSensorHealthy() { return !g_sensorOffline && (g_sensorWarmupReads == 0); }
static inline bool anaMotorRunning()  { return g_motorRunState == MOTOR_RUNNING; }  // warmup baked-in (v16.3z)
// future: static inline bool anaCurrentHealthy() {...}  anaTempHealthy() {...}

static AnalysisReason_t analysisReason() {
  if (g_sensorOffline)            return ANA_FRZ_SENSOR_OFFLINE;
  if (g_sensorWarmupReads > 0)    return ANA_FRZ_RECONFIG;
  switch (g_motorRunState) {
    case MOTOR_STOPPED:  return ANA_FRZ_STOPPED;
    case MOTOR_STARTING: return ANA_FRZ_STARTING;
    case MOTOR_STOPPING: return ANA_FRZ_STOPPING;
    default: break;  // RUNNING
  }
  // future: if (!anaCurrentHealthy()) return ANA_FRZ_CURRENT; ...
  return ANA_READY;
}
static inline bool isAnalysisReady() { return analysisReason() == ANA_READY; }
static const char* analysisReasonStr(AnalysisReason_t r) {
  switch (r) {
    case ANA_READY:              return "READY";
    case ANA_FRZ_SENSOR_OFFLINE: return "SENSOR_OFFLINE";
    case ANA_FRZ_RECONFIG:       return "RECONFIG";
    case ANA_FRZ_STOPPED:        return "STOPPED";
    case ANA_FRZ_STARTING:       return "STARTING";
    case ANA_FRZ_STOPPING:       return "STOPPING";
  }
  return "?";
}
static MotorRunState_t g_prevMotorRunState  = MOTOR_STOPPED;  // v15.2: track transition
// [Phase 1] g_bearingStableCnt REMOVED -- its only reader was the bearing_alert
// warm-up gate, deleted with the rest of the legacy bearing pipeline.
static bool            g_trendFreqFlushed   = false;          // v16.0: freq_ratio flushed on RUNNING entry

// Runtime hour accumulation (NVS persistent)
static Preferences g_motorPrefs;
static float       g_runtimeHour    = 0.0f;
static uint32_t    g_runningStartMs = 0;
static bool        g_prevWasRunning = false;
static uint32_t    g_lastNvsSaveMs  = 0;

// ISR -- IRAM_ATTR: runs from IRAM, safe from Cache miss
void IRAM_ATTR rpmISR() {
  uint32_t now      = micros();
  uint32_t interval = now - g_rpmLastPulseTime;
  if (interval > RPM_DEBOUNCE_US) {
    g_rpmPulseInterval = interval;
    g_rpmLastPulseTime = now;
    g_rpmTotalPulses++;
  }
}

// NVS: ???? runtime_hour ??? Flash
static void loadRuntimeHour() {
  g_motorPrefs.begin("motor_nvs", false);
  g_runtimeHour = g_motorPrefs.getFloat("runtime_h", 0.0f);
  g_motorPrefs.end();
  Serial.printf("[RPM] Loaded runtime_hour = %.4f h\n", g_runtimeHour);
}

// NVS: ?????? runtime_hour ?? Flash
static void saveRuntimeHour(float value) {
  g_motorPrefs.begin("motor_nvs", false);
  // [Task 7.3 -- TEMPORARY DIAGNOSTIC ONLY] NVS write entry/exit markers,
  // bracketing exactly the commit call (putFloat + end(), where the actual
  // flash write/cross-core cache-disable happens per SDK behavior) for
  // correlation against [UART_ERR] timestamps.
  Serial.printf("[NVS_BEGIN] %lu\n", (unsigned long)millis());
  g_motorPrefs.putFloat("runtime_h", value);
  g_motorPrefs.end();
  Serial.printf("[NVS_END]   %lu\n", (unsigned long)millis());
}

// ============================================================================
// FAULT LATCH v3 — FUNCTIONS  [inserted: fault_latch_v3_code.ino SECTIONS 4-9]
// v3 hardened: g_fl/g_flCount access is serialized via mutexFaultLatch.
//   - faultSeverity()/faultEventStr(): pure functions of `code`, no shared
//     state touched, no lock needed.
//   - saveFaultLatchNVS(): assumes the CALLER already holds mutexFaultLatch
//     (it is only ever called from inside checkAndLatchFault()'s locked
//     region). It does not take/give the mutex itself, to avoid recursive
//     deadlock on the non-recursive FreeRTOS mutex.
//   - loadFaultLatchNVS(): called once from setup(), before any FreeRTOS
//     task is created (single-threaded context) -- no lock is required or
//     taken here; this is a deliberate, documented exception.
//   - clearFaultLatchNVS(): an independent entry point (called from
//     taskNetwork's replay block after its MQTT publish completes), so it
//     takes/gives mutexFaultLatch itself, protecting both g_fl.pending and
//     the NVS clear transaction.
//   - checkAndLatchFault(): takes mutexFaultLatch around the read-check-
//     write-save sequence on g_fl (the only piece that touches shared
//     state); the RTC read for epochNow happens BEFORE the lock is taken,
//     to keep the critical section -- which includes a flash/NVS write --
//     as short as possible.
// ============================================================================

static uint8_t faultSeverity(uint8_t code) {
  switch (code) {
    case FL_EVT_BEARING:  return FL_SEV_BEARING;
    case FL_EVT_CRITICAL: return FL_SEV_CRITICAL;
    case FL_EVT_HEALTH:   return FL_SEV_HEALTH;
    case FL_EVT_WARNING:  return FL_SEV_WARNING;
    default:              return FL_SEV_NONE;
  }
}

static const char* faultEventStr(uint8_t code) {
  switch (code) {
    case FL_EVT_BEARING:  return "BEARING_CONFIRMED";
    case FL_EVT_CRITICAL: return "ALARM_CRITICAL";
    case FL_EVT_HEALTH:   return "HEALTH_LOW";
    case FL_EVT_WARNING:  return "ALARM_WARNING";
    default:              return "UNKNOWN";
  }
}

// NOTE: caller must hold mutexFaultLatch before calling this function.
static void saveFaultLatchNVS() {
  Preferences p;
  p.begin(FL_NS, false);
  // [Task 7.3 -- TEMPORARY DIAGNOSTIC ONLY] NVS write entry/exit markers.
  // NOTE: this function currently has no call sites anywhere in this file
  // (checkAndLatchFault() uses its own inline write block instead, below) --
  // instrumented anyway per this task's explicit "at minimum" list, and in
  // case it becomes reachable in the future.
  Serial.printf("[NVS_BEGIN] %lu\n", (unsigned long)millis());
  p.putUChar(FL_KEY_PENDING, 0u);              // Step 1: disarm
  p.putUChar(FL_KEY_CODE,    g_fl.code);       // Step 2: data
  p.putUInt (FL_KEY_TS,      g_fl.ts);
  p.putFloat(FL_KEY_RMS,     g_fl.rms);
  p.putFloat(FL_KEY_KURT,    g_fl.kurtosis);
  p.putUInt (FL_KEY_MAGIC,   FL_MAGIC_VALUE);  // Step 3: commit marker
  p.putUChar(FL_KEY_PENDING, 1u);              // Step 4: arm
  p.end();
  Serial.printf("[NVS_END]   %lu\n", (unsigned long)millis());
  Serial.printf("[LATCH] SAVE code=%u(%s) sev=%u ts=%lu rms=%.2f kurt=%.3f n=%lu\n",
                g_fl.code, faultEventStr(g_fl.code), faultSeverity(g_fl.code),
                (unsigned long)g_fl.ts, g_fl.rms, g_fl.kurtosis,
                (unsigned long)g_flCount);
}

// NOTE: runs once from setup(), before any FreeRTOS task is created --
// no concurrent access to g_fl is possible yet, so no lock is taken here.
static void loadFaultLatchNVS() {
  Preferences p;
  p.begin(FL_NS, true);
  uint8_t  pending = p.getUChar(FL_KEY_PENDING, 0u);
  uint32_t magic   = p.getUInt (FL_KEY_MAGIC,   0u);
  uint8_t  code    = p.getUChar(FL_KEY_CODE,    FL_EVT_NONE);
  uint32_t ts      = p.getUInt (FL_KEY_TS,      0xFFFFFFFFUL);
  float    rms     = p.getFloat(FL_KEY_RMS,     -1.0f);
  float    kurt    = p.getFloat(FL_KEY_KURT,    -1.0f);
  p.end();

  if (!pending) {
    Serial.println("[LATCH] RECOVER — no pending latch");
    return;
  }
  if (magic != FL_MAGIC_VALUE) {
    Serial.printf("[LATCH] CORRUPT — magic=0x%08lX expected=0x%08lX\n",
                  (unsigned long)magic, (unsigned long)FL_MAGIC_VALUE);
    goto fl_discard;
  }
  if (code == FL_EVT_NONE || code > FL_EVT_MAX_VALID) {
    Serial.printf("[LATCH] CORRUPT — code=%u out of [1,%u]\n",
                  code, FL_EVT_MAX_VALID);
    goto fl_discard;
  }
  if (ts == 0xFFFFFFFFUL) {
    Serial.println("[LATCH] CORRUPT — ts=0xFFFFFFFF (erased cell)");
    goto fl_discard;
  }
  if (ts != FL_TS_UNKNOWN && ts < FL_TS_MIN_VALID) {
    Serial.printf("[LATCH] CORRUPT — ts=%lu below min valid\n", (unsigned long)ts);
    goto fl_discard;
  }
  if (rms != rms || rms < 0.0f || rms > 100.0f) {
    Serial.printf("[LATCH] CORRUPT — rms=%.3f implausible\n", rms);
    goto fl_discard;
  }
  if (kurt != kurt || kurt < 0.0f || kurt > 1000.0f) {
    Serial.printf("[LATCH] CORRUPT — kurt=%.3f implausible\n", kurt);
    goto fl_discard;
  }

  g_fl.code     = code;
  g_fl.ts       = ts;
  g_fl.rms      = rms;
  g_fl.kurtosis = kurt;
  g_fl.pending  = true;
  Serial.printf("[LATCH] RECOVER OK — code=%u(%s) sev=%u ts=%lu rms=%.2f kurt=%.3f\n",
                code, faultEventStr(code), faultSeverity(code),
                (unsigned long)ts, rms, kurt);
  return;

fl_discard:
  {
    Preferences pe;
    pe.begin(FL_NS, false);
    pe.clear();
    pe.end();
  }
  Serial.println("[LATCH] CORRUPT — namespace erased, system continues normally");
}

// NOTE (v3 hardened): independent entry point -- takes mutexFaultLatch
// itself, protecting g_fl.pending and the NVS clear transaction together.
static void clearFaultLatchNVS() {
  if (xSemaphoreTake(mutexFaultLatch, pdMS_TO_TICKS(20)) != pdTRUE) {
    Serial.println("[LATCH] CLEAR mutex timeout — will retry next replay cycle");
    return;
  }
  Preferences p;
  p.begin(FL_NS, false);
  // [Task 7.3 -- TEMPORARY DIAGNOSTIC ONLY] NVS write entry/exit markers.
  Serial.printf("[NVS_BEGIN] %lu\n", (unsigned long)millis());
  p.putUChar(FL_KEY_PENDING, 0u);
  p.putUInt (FL_KEY_MAGIC,   0u);
  p.end();
  Serial.printf("[NVS_END]   %lu\n", (unsigned long)millis());
  g_fl.pending = false;
  xSemaphoreGive(mutexFaultLatch);
  Serial.println("[LATCH] CLEAR — delivered, NVS latch released");
}

// ============================================================================
// pushTelemBuf() — snapshot VibrationData_t + MachineState_t into ring buffer
// ============================================================================
// เรียกจาก taskNetwork() เมื่อ MQTT offline แต่ถึงเวลา publish แล้ว
// Non-blocking: mutex timeout 10 ms (สั้นพอไม่บล็อก taskNetwork loop)
// Drop-oldest เมื่อ buffer เต็ม (overwrite head)
// ============================================================================
static void pushTelemBuf(const VibrationData_t* data, MachineState_t state) {
  if (xSemaphoreTake(mutexTelemBuf, pdMS_TO_TICKS(10)) != pdTRUE) {
    Serial.println("[TelemBuf] WARN: mutex timeout on push — slot dropped");
    return;
  }

  if (g_telemBufCount == TELEM_BUF_SIZE) {
    // Buffer เต็ม: overwrite oldest (advance head past the oldest entry)
    g_telemBufHead = (g_telemBufHead + 1) % TELEM_BUF_SIZE;
    g_telemBufOverflowCount++;
    // g_telemBufCount stays at TELEM_BUF_SIZE (slot count unchanged)
  } else {
    g_telemBufCount++;
  }

  // Write slot at (head + count - 1) % SIZE  i.e. the newest slot position
  // Since head advances only on overflow, we write at:
  //   write_idx = (g_telemBufHead + g_telemBufCount - 1) % TELEM_BUF_SIZE
  uint8_t writeIdx = (g_telemBufHead + g_telemBufCount - 1) % TELEM_BUF_SIZE;

  TelemetrySlot_t* s = &g_telemBuf[writeIdx];
  // [v16.3g] ใช้ RTC_NOW_SAFE แทน rtc.now() โดยตรง
  if (g_rtcValid) {
    DateTime _ts; RTC_NOW_SAFE(_ts);
    s->buffered_ts = (uint32_t)_ts.unixtime();
  } else {
    s->buffered_ts = 0u;
  }
  // [Phase2] legacy VRMS/VPEAK capture removed from the telemetry-buffer slot.
  s->temperature       = data->temperature;
  // [v16.5] gate เหมือน rms_overall/x/y/z ด้านบน -- ป้องกัน CF garbage
  // ตอน STOPPED เข้าไปนอน buffer แล้วถูก replay ออก MQTT ซ้ำทีหลัง
  // [Phase 3G] cf_max now carries the FIFO/DSP crest factor, filled from the
  // carrier in the velocity block below. NaN until then, exactly like
  // velocity_rms_* and freq_* in this same struct: NaN is what the serializer
  // tests to decide whether the key is publishable at all. The legacy register
  // value is deliberately NOT used as a fallback.
  s->cf_max            = NAN;
  // [Phase 3F] freq_x/y/z are the FIFO/DSP dominant frequency, filled from the
  // carrier in the velocity block below. NaN until then, exactly like
  // velocity_rms_* in this same struct: NaN is what the serializer tests to
  // decide whether the key is publishable at all.
  s->freq_x            = NAN;
  s->freq_y            = NAN;
  s->freq_z            = NAN;
  s->rpm               = data->rpm;
  s->motor_state       = data->motor_state;
  s->machine_state     = (uint8_t)state;
  // [Phase 3J] Capture-time vibration_status. Recorded here, never re-derived on
  // replay: by the time a backlog drains the globals it comes from describe a
  // different moment entirely.
  s->vibration_status  = (uint8_t)currentVibStatus();
  s->prox              = data->prox;

  // ── [M1B-6] FIFO-DSP velocity block ───────────────────────────────────
  // Sourced from g_velCarrier, NOT from the M1B-1 ring: this function runs in
  // taskNetwork, and the ring is only safe inside taskAnalytics. The carrier
  // already owns mutexVelCarrier for exactly this boundary.
  //
  // Freshness uses the same VIB_VELOCITY_MAX_AGE_MS_TBD deadline as the alarm
  // and trend paths, so "stale" means one thing everywhere.
  //
  // On ANY failure -- carrier missing, mutex busy, invalid, or stale -- the
  // four floats are NaN and velocity_data_valid is false. There is no path
  // that writes 0.0f to represent absent velocity.
  s->schema_version       = (uint8_t)TELEM_SLOT_SCHEMA_VERSION;
  s->velocity_rms_x       = NAN;
  s->velocity_rms_y       = NAN;
  s->velocity_rms_z       = NAN;
  s->velocity_rms_overall = NAN;
  s->velocity_data_valid  = false;
  s->capture_id           = 0u;
  s->capture_ts_ms        = 0u;
  s->sample_rate_hz       = 0u;
  s->sample_count         = 0u;

  if (mutexVelCarrier != NULL) {
    VelocityCarrier_t vc;
    bool got = false;
    if (xSemaphoreTake(mutexVelCarrier, pdMS_TO_TICKS(5)) == pdTRUE) {
      vc  = g_velCarrier;          // whole-struct copy: no torn read
      got = true;
      xSemaphoreGive(mutexVelCarrier);
    }
    if (got && vc.valid && vc.timestampMs != 0u) {
      const uint32_t vAge = millis() - vc.timestampMs;   // wrap-safe
      if (vAge <= VIB_VELOCITY_MAX_AGE_MS_TBD) {
        s->velocity_rms_x       = vc.x;
        s->velocity_rms_y       = vc.y;
        s->velocity_rms_z       = vc.z;
        s->velocity_rms_overall = vc.overall;
        s->capture_id           = vc.captureId;
        s->capture_ts_ms        = vc.timestampMs;
        s->sample_rate_hz       = vc.sampleRateHz;
        s->sample_count         = vc.sampleCount;
        // [Phase 3F] Per-axis: an axis whose flag is clear stays NaN and its
        // key is omitted on replay, independently of the other two.
        if (vc.domFreqXValid) s->freq_x = vc.domFreqX;
        if (vc.domFreqYValid) s->freq_y = vc.domFreqY;
        if (vc.domFreqZValid) s->freq_z = vc.domFreqZ;
        // [Phase 3G] Same capture, same mutex, same freshness gate as the
        // velocity and frequency fields beside it.
        if (vc.crestFactorValid) s->cf_max = vc.crestFactor;
        s->velocity_data_valid  = true;
      }
    }
  }

  xSemaphoreGive(mutexTelemBuf);

  Serial.printf("[TelemBuf] push: count=%u/%u  overflow_total=%lu\n",
                (unsigned)g_telemBufCount, TELEM_BUF_SIZE,
                (unsigned long)g_telemBufOverflowCount);
}

#ifdef DEBUG_MQTT_TIMING
// ============================================================================
// [Phase 13] Timing-instrumentation support. Observational only.
//
// PLACEMENT IS DELIBERATE -- do not move these earlier in the file. The Arduino
// .ino preprocessor emits its auto-generated forward prototypes immediately
// before the FIRST function definition in the sketch (currently
// trendClearThresholdMs()). Defining a function above that point drags the
// whole prototype block above the typedefs it depends on (VibrationData_t,
// AnalysisReason_t, MqttOutboundTopic_t, ...) and the build fails with a
// cascade of "does not name a type". These helpers therefore live here: after
// the prototype anchor, and before their first consumer (replayTelemBuf()).
// See CLAUDE.md "Coding Style & Conventions" -- .ino auto-prototype rule.
// ============================================================================
#include <stdarg.h>

// Shared formatter for every mqttClient.publish() call site, so the same block
// is emitted at all 8 sites without duplicating the format string 8 times.
// [v16.5c] wasConnected defaults to true so the 5 call sites that don't pass
// it keep byte-identical output. MQTTClient::publish()'s
// `if (!connected()) return false;` guard returns without touching
// _lastError -- when a prior publish() in the same cycle already dropped the
// connection, the next topic's call hits that guard at elapsed=0ms and this
// would otherwise re-report the FIRST failure's stale err code as if it were
// a fresh one. Diagnostic-only: no control flow, retry, timeout, or
// reconnect behaviour changes -- this only changes what gets printed.
static void dbgLogMqttPublish(const char* topic, size_t payloadLen, int qos,
                               bool result, int err, uint32_t elapsedMs,
                               bool wasConnected = true) {
  const char* label = result ? "OK" : (wasConnected ? "FAIL" : "NOT_CONNECTED");
  Serial.printf("[MQTT-TIMING] topic=%s payload=%u qos=%d result=%s err=%d elapsed=%lums\n",
                topic ? topic : "(null)", (unsigned)payloadLen, qos,
                label, err, (unsigned long)elapsedMs);
}

// C-linkage sink for lwmqtt/client.c's timing output.
//
// WHY THIS EXISTS instead of client.c calling printf() directly: client.c is a
// separately-compiled C translation unit. Its printf() writes to the ESP-IDF
// stdout console, which on this board is NOT guaranteed to be the same stream
// as Serial -- the FQBN sets CDCOnBoot=cdc / USBMode=hwcdc, so Serial is the
// USB CDC while the IDF console may still be UART0. Timing lines split across
// two physical interfaces would be unusable (and easy to mistake for "no
// output"). Routing them through this hook guarantees the [LWMQTT] lines land
// in the same capture, correctly interleaved with [MQTT-TIMING]/[MODEMSEND].
extern "C" void lwmqtt_dbg_log(const char* fmt, ...) {
  char buf[192];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  Serial.print(buf);
}
#endif  // DEBUG_MQTT_TIMING

#ifdef DEBUG_MODEM_DIAG
// ============================================================================
// [v16.5b] Read-only modem diagnostics -- Issue #2 Phase 1
// ============================================================================
// Called ONLY from taskNetwork() (Core 1), which is the sole owner of
// SerialAT. taskNetwork is single-threaded, so this can never interleave with
// a publish, a PUBACK wait or a reconnect -- those all run to completion in
// the same task before control reaches the call site.
//
// modem.sendAT()/modem.waitResponse() are public members of TinyGsmModem
// (TinyGsmModem.tpp:91 and :142, inside the public: block opened at :59) and
// are CALLED here, not modified -- the TinyGSM library is untouched.
// waitResponse() internally dispatches handleURCs(), the same path
// modemGetAvailable() already uses, so +CIPRXGET URCs continue to be consumed
// normally and no socket data can be lost.
//
// Return code: TinyGsmModem.tpp:605 initialises index=0 and returns it after
// the timeout loop expires (:657, :672), so rc==0 means TIMEOUT, rc==1 means
// the OK terminator matched, rc==2 means ERROR. A failed or unsupported
// command is therefore recorded and execution continues, never aborted.
// ============================================================================
static void modemDiagOne(const char* label, const char* cmd) {
  String   raw;
  uint32_t t0 = millis();
  modem.sendAT(cmd);
  int8_t   rc = modem.waitResponse(MODEM_DIAG_AT_TMO_MS, raw);
  uint32_t el = millis() - t0;

  raw.trim();
  raw.replace("\r\n", " | ");   // flatten to one log line; content kept verbatim

  Serial.printf("[MODEM-DIAG] t=%lu %-6s rc=%d timeout=%s elapsed=%lums raw=\"%s\"\n",
                (unsigned long)t0, label, (int)rc,
                (rc == 0) ? "YES" : "NO",
                (unsigned long)el, raw.c_str());
}

// Context is read from ALREADY-CACHED state only -- no extra AT traffic, so
// this cannot perturb the very uplink timing under measurement. In particular
// mqttClient.connected() is deliberately NOT called here: that path reaches
// TinyGsmClient::connected() -> modemGetConnected(), which issues an AT
// command (visible today as "[P1] ... incl modemGetConnected").
static void modemDiagPoll(bool mqttUp, bool gprsUp,
                          uint32_t sincePubMs, uint32_t pubIntervalMs) {
  uint32_t t0 = millis();
  Serial.printf("[MODEM-DIAG] t=%lu ==== BEGIN ====\n", (unsigned long)t0);
  Serial.printf("[MODEM-DIAG] t=%lu CONTEXT mqtt=%s gprs=%s since_pub=%lums "
                "next_pub_in=%lums idle=YES(gated)\n",
                (unsigned long)millis(),
                mqttUp ? "UP" : "DOWN",
                gprsUp ? "UP" : "DOWN",
                (unsigned long)sincePubMs,
                (unsigned long)(pubIntervalMs - sincePubMs));

  modemDiagOne("CPSI",   "+CPSI?");    // system mode, band, cell, RSRP/RSRQ/SINR
  modemDiagOne("CSQ",    "+CSQ");      // RSSI + BER
  modemDiagOne("CEREG",  "+CEREG?");   // EPS registration state
  modemDiagOne("CPSMS",  "+CPSMS?");   // Power Saving Mode configuration
  modemDiagOne("CEDRXS", "+CEDRXS?");  // extended DRX configuration

  Serial.printf("[MODEM-DIAG] t=%lu ==== END total=%lums ====\n",
                (unsigned long)millis(), (unsigned long)(millis() - t0));
}
#endif  // DEBUG_MODEM_DIAG

// ============================================================================
// replayTelemBuf() — ส่ง ONE slot ที่เก่าที่สุดออก MQTT แล้ว pop ออก
// ============================================================================
// เรียก 1 ครั้งต่อ taskNetwork() loop iteration เมื่อ MQTT connected + backlog > 0
// Returns: true = slot published + popped, false = publish failed หรือ buffer ว่าง
// ============================================================================
static bool replayTelemBuf() {
  // --- ล็อค mutex สั้นๆ เพื่อ copy oldest slot ออกมา ---
  TelemetrySlot_t snap;
  {
    if (xSemaphoreTake(mutexTelemBuf, pdMS_TO_TICKS(10)) != pdTRUE) {
      Serial.println("[TelemBuf] WARN: mutex timeout on replay peek");
      return false;
    }
    if (g_telemBufCount == 0) {
      xSemaphoreGive(mutexTelemBuf);
      return false;  // buffer ว่าง (เรียกจาก caller ไม่ควรเกิดขึ้น)
    }
    memcpy(&snap, &g_telemBuf[g_telemBufHead], sizeof(TelemetrySlot_t));
    xSemaphoreGive(mutexTelemBuf);
  }

  // --- Build replay JSON (ใช้ /sensor topic เดียวกัน + replay fields พิเศษ) ---
  // ขยาย StaticJsonDocument จาก 960 เป็น 1024 เพื่อรองรับ field เพิ่ม 3 ตัว
  // ขนาดยังคงอยู่บน stack ของ taskNetwork (stack size ตรวจสอบ watermark แล้ว)
  // [M1B-6] 1024 -> 1536: live /sensor measures 873-891 B and the additive
  // velocity block adds ~190 B, which would overflow the old 1024 B buffer
  // and hit the "[TelemBuf] WARN: replay JSON truncated" path. Replay uses
  // mqttClient.publish() directly, so MQTT_OUTBOUND_PAYLOAD_MAX does not
  // apply; taskNetwork has ~17 KB stack free.
  StaticJsonDocument<1536> r;

  r["plant"]              = PLANT_ID;
  r["machine_id"]         = MACHINE_ID;
  r["sensor_id"]          = SENSOR_ID;
  r["stage"]              = "sensor";
  r["execution_location"] = "edge";
  r["sensor_status"]      = "ONLINE";

  // Replay metadata — ช่วยให้ backend แยกแยะจาก live data
  r["replayed"]    = true;
  r["buffered_at"] = snap.buffered_ts;  // epoch seconds ขณะ capture

  // [v16.3q] Replay Age Check — drop slot ที่เก่าเกิน 24 ชั่วโมง
  // ข้อมูลสั่นสะเทือนเมื่อ >24h ที่แล้วไม่มีประโยชน์เชิง real-time แล้ว
  // ใช้ NTP lastSyncTime เป็น reference เพราะ buffered_ts อาจผิดถ้า RTC เคยเพี้ยน
  const uint32_t MAX_REPLAY_AGE_SEC = 24UL * 3600UL;  // 24 ชั่วโมง
  if (g_timeSync.synced && snap.buffered_ts > 1000000UL) {
    // ตรวจว่า buffered_ts ดูสมเหตุสมผลก่อน (> 2001 = 1000000000)
    uint32_t nowEpoch = 0;
    if (g_rtcValid) {
      DateTime nowDt; RTC_NOW_SAFE(nowDt);
      if (nowDt.year() >= 2024 && nowDt.year() <= 2035) {
        nowEpoch = nowDt.unixtime();
      }
    }
    if (nowEpoch > 0 && snap.buffered_ts < (nowEpoch - MAX_REPLAY_AGE_SEC)) {
      // slot เก่าเกิน 24h → drop ไม่ส่ง
      if (xSemaphoreTake(mutexTelemBuf, pdMS_TO_TICKS(10)) == pdTRUE) {
        g_telemBufHead  = (g_telemBufHead + 1) % TELEM_BUF_SIZE;
        g_telemBufCount--;
        xSemaphoreGive(mutexTelemBuf);
      }
      Serial.printf("[TelemBuf] DROP stale slot (buffered_at=%lu age>24h)\n",
                    (unsigned long)snap.buffered_ts);
      return true;  // pop สำเร็จ (drop) ให้ caller loop ต่อ
    }
  }

  // Timestamp เมื่อ replay (now)
  uint32_t sentAt = 0;
  char tsBufReplayed[26] = "not_available";
  if (g_rtcValid) {
    DateTime now; RTC_NOW_SAFE(now);  // [v16.3g]
    // [v16.3q] year sanity check ก่อนใช้ timestamp
    if (now.year() >= 2024 && now.year() <= 2035) {
      snprintf(tsBufReplayed, sizeof(tsBufReplayed),
               "%04d-%02d-%02dT%02d:%02d:%02dZ",
               now.year(), now.month(), now.day(),
               now.hour(), now.minute(), now.second());
    } else if (g_timeSync.synced && strlen(g_timeSync.lastSyncTime) > 10) {
      // RTC corrupt → fallback NTP time
      strncpy(tsBufReplayed, g_timeSync.lastSyncTime, sizeof(tsBufReplayed) - 1);
    }
  }

  // [v16.3q] buffered_at sanity check
  // ถ้า buffered_ts เป็นปี 2106 (0xFFFFFFFF region) → แสดง INVALID แทน
  // ยังส่งข้อมูลออกไปได้ เพื่อให้เห็นว่าเคยมีข้อมูลอยู่ แต่ timestamp ใช้ไม่ได้
  bool bufTsValid = (snap.buffered_ts > 1000000000UL &&  // > 2001
                     snap.buffered_ts < 2000000000UL);   // < 2033

  // [Replay-Contract] Capture-time string for the /vibration replay payload
  // ONLY. tsBufReplayed above is the clock AT REPLAY and keeps feeding
  // r["timestamp"] on /sensor unchanged. rv["timestamp"] (below) needs the
  // ORIGINAL capture moment instead, so historical points land at the time
  // the sample was taken, not the time the backlog happened to drain.
  // DateTime(uint32_t) is the same epoch constructor already used at the
  // NTP-sync call site (syncRTCFromModem()/parseGSMDateTime()).
  char tsBufCapture[26] = "not_available";
  if (bufTsValid) {
    DateTime capDt((uint32_t)snap.buffered_ts);
    snprintf(tsBufCapture, sizeof(tsBufCapture),
             "%04d-%02d-%02dT%02d:%02d:%02dZ",
             capDt.year(), capDt.month(), capDt.day(),
             capDt.hour(), capDt.minute(), capDt.second());
  }

  if (!bufTsValid) {
    r["timestamp"]   = "INVALID";
    r["time_synced"] = false;
    Serial.printf("[TelemBuf] ! buffered_at=%lu invalid timestamp — marking INVALID\n",
                  (unsigned long)snap.buffered_ts);
  }

  r["sent_at"]   = sentAt;
  if (bufTsValid) {
    r["timestamp"] = tsBufReplayed;  // ISO-8601 ขณะ replay (for Grafana time axis)
  }

  // Sensor payload (mirror ของ publishTelemetry /sensor fields)
  // [Phase1] LEGACY VRMS/VPEAK REMOVED from the replay serializer:
  //   rms, vx, vy, vz  (snap.rms_overall / rms_x / rms_y / rms_z)
  //   peak             (snap.vel_peak_overall)
  // Kept in lock-step with live /sensor, which no longer carries them either.
  // The FIFO-DSP velocity block further down is the replayed vibration source.

  r["temp"]         = round(snap.temperature * 10)  /  10.0f;
  r["rpm"]          = snap.rpm;
  r["motor_state"]  = snap.motor_state;
  r["rotation_signal_ok"] = snap.prox;

  // [Phase 3I] freq_x/y/z and crest_factor are NO LONGER in this /sensor
  // payload. Phase 3H made /vibration the one canonical carrier of machine
  // vibration truth and moved them there on the LIVE path, but this replay
  // serializer was written back when /sensor was canonical and kept emitting
  // them -- so after any MQTT outage the buffered slots put the very same
  // canonical keys back on /sensor, giving one value two topics. They are
  // emitted once, on the replay /vibration payload built further down, which
  // is the exact mirror of what the live path does.
  // [Phase 1] kurtosis_max REMOVED from the replay /sensor payload.

  // [Phase 1] kurtosis_axis REMOVED from the replay /sensor payload.

  // ── [M1B-6] FIFO-DSP velocity ─────────────────────────────────────────
  // [Phase1] vibration_source_legacy REMOVED -- rms/vx/vy/vz/peak are gone
  // from this payload, so there is no longer a legacy vibration source to
  // declare provenance for. crest_factor / kurtosis / freq / rpm / temp above
  // come from their own sensor registers (0x47.., 0x44.., 0x40), not from
  // VRMS, and are untouched.
  r["schema_version"]          = snap.schema_version;
  // [Phase 3J] velocity_data_valid, velocity_rms_x/y/z/overall and the capture
  // provenance (capture_id / capture_ts_ms / sample_rate_hz / sample_count) are
  // REMOVED from the replay /sensor payload. Live /sensor has carried no
  // vibration data since Phase 1 -- it describes the sensor and the device only
  // -- and this payload was still shaped like the pre-Phase-1 topic. The values
  // are not dropped: they move to the replay /vibration payload below, beside
  // the frequency and crest factor they were captured with. One canonical
  // value, one topic, live and replayed alike.

  // time_synced — ถูก set ใน block ด้านบนแล้วถ้า bufTsValid=false
  // ถ้า bufTsValid=true ใช้ค่าจาก NTP sync state
  if (bufTsValid) {
    r["time_synced"] = g_timeSync.synced;
  }

  // ══════════════════════════════════════════════════════════════════════
  // [Phase 3I] REPLAY /vibration -- the canonical vibration fields
  // ══════════════════════════════════════════════════════════════════════
  // Mirrors the live /vibration contract for a buffered slot: the canonical
  // set lives on /vibration and nowhere else, replayed or live.
  //
  // Published BEFORE the /sensor payload below on purpose. The slot is popped
  // only after the /sensor publish succeeds, so ordering decides what a failed
  // publish costs: send /vibration first and a failure here returns with the
  // slot still queued and nothing yet emitted, which retries cleanly. The
  // reverse order would have to choose between dropping the vibration data or
  // re-sending /sensor. The remaining split case -- /vibration accepted, then
  // /sensor refused -- replays /vibration once more on the next attempt; a
  // repeat of a historical sample already tagged replayed=true with its own
  // buffered_at is idempotent for a timestamp-keyed consumer, whereas losing
  // it is not recoverable.
  //
  // Scope: this block moves the five canonical keys only. velocity_rms_* stay
  // on the replay /sensor payload where they already were -- they are outside
  // this fix and moving them is a separate data-contract change.
  {
    StaticJsonDocument<512> rv;

    rv["plant"]      = PLANT_ID;
    rv["machine_id"] = MACHINE_ID;
    rv["sensor_id"]  = SENSOR_ID;
    rv["stage"]      = "vibration";

    // Same replay metadata as the /sensor payload, so a consumer can tell a
    // replayed vibration sample from a live one and pair it with its slot.
    rv["replayed"]    = true;
    rv["buffered_at"] = snap.buffered_ts;
    rv["sent_at"]     = sentAt;
    if (bufTsValid) {
      // [Replay-Contract] Capture time, not replay time -- see tsBufCapture
      // above. r["timestamp"] on /sensor (unchanged, above) still uses
      // tsBufReplayed; only the /vibration payload's historical accuracy
      // is corrected here.
      rv["timestamp"]    = tsBufCapture;
      rv["time_synced"]  = g_timeSync.synced;
    } else {
      rv["time_synced"]  = false;
    }

    // [Phase 3I] Same omit-when-invalid contract as the live path: a value the
    // capture never produced is ABSENT, never 0. NaN is what the slot stores
    // for "not measured" and JSON cannot carry NaN, so testing isfinite() here
    // is what stops it becoming a 0 or a null a consumer reads as a real 0 Hz
    // or a real crest factor of zero.
    if (isfinite(snap.freq_x)) rv["freq_x"] = roundf(snap.freq_x * 10) / 10.0f;
    if (isfinite(snap.freq_y)) rv["freq_y"] = roundf(snap.freq_y * 10) / 10.0f;
    if (isfinite(snap.freq_z)) rv["freq_z"] = roundf(snap.freq_z * 10) / 10.0f;
    if (isfinite(snap.cf_max)) rv["crest_factor"] = round(snap.cf_max * 100) / 100.0f;

    // dominant_vibration_axis: the axis carrying the highest velocity RMS,
    // read straight off the values already stored in the slot -- the same
    // comparison the live path makes on the velocity carrier. Nothing is
    // recomputed and no DSP runs here; this only labels which of three stored
    // numbers is largest.
    // [Phase 3J] Emitted UNCONDITIONALLY, with "-" for unknown, matching the
    // live serializer exactly. Previously the key was omitted when velocity was
    // invalid, which made a replayed record a different SHAPE from the live one
    // it is supposed to reproduce. "-" is the live representation for "no axis
    // measured"; no axis is ever guessed.
    const bool axisKnown = snap.velocity_data_valid &&
                           isfinite(snap.velocity_rms_x) &&
                           isfinite(snap.velocity_rms_y) &&
                           isfinite(snap.velocity_rms_z);
    rv["dominant_vibration_axis"] = !axisKnown ? "-"
        : (snap.velocity_rms_x >= snap.velocity_rms_y &&
           snap.velocity_rms_x >= snap.velocity_rms_z) ? "X"
        : (snap.velocity_rms_y >= snap.velocity_rms_z) ? "Y"
                                                       : "Z";

    // ── [Phase 3J] The rest of the canonical set, from stored capture-time
    // values only. Every field below is either read directly out of the slot or
    // rendered by a pure formatter; nothing here evaluates a threshold, reads a
    // live global, or forms a verdict. That is what keeps the RMS state machine
    // the single place a machine condition is ever decided.
    rv["vibration_source"]    = "fifo_dsp";
    rv["vibration_status"]    = vibStatusStr((VibrationStatus_t)snap.vibration_status);
    rv["velocity_data_valid"] = snap.velocity_data_valid;
    if (snap.velocity_data_valid) {
      // Same guard as live: published ONLY when the capture produced them. When
      // invalid the four are NaN, which JSON cannot carry -- omitting the keys
      // is what stops a NaN becoming a 0 a consumer reads as zero vibration.
      rv["velocity_rms_overall"] = roundf(snap.velocity_rms_overall * 1000.0f) / 1000.0f;
      rv["velocity_rms_x"]       = roundf(snap.velocity_rms_x       * 1000.0f) / 1000.0f;
      rv["velocity_rms_y"]       = roundf(snap.velocity_rms_y       * 1000.0f) / 1000.0f;
      rv["velocity_rms_z"]       = roundf(snap.velocity_rms_z       * 1000.0f) / 1000.0f;
    }

    // Operating context carried by the live payload and stored in the slot.
    // current_a / current_valid / current_age_s and operating_hours_total are
    // NOT stored, so they are absent rather than invented.
    rv["temp"]        = round(snap.temperature * 10) / 10.0f;
    rv["rpm"]         = snap.rpm;
    rv["motor_state"] = snap.motor_state;

    // [Phase 3J] alarm_level: the verdict THIS record carried when it was
    // captured, rendered by the same pure formatter the live path uses. It is
    // transported, never recomputed -- re-deriving it from the stored velocity
    // would bypass the hysteresis and capture-persistence the state machine
    // applied, and could publish a verdict the device never actually reported.
    rv["alarm_level"] = alarmLevelStr((MachineState_t)snap.machine_state,
                                      snap.motor_state);

    // [Phase 3J] 512 -> 1024 B: the canonical set above roughly doubles the
    // payload (worst case ~577 B). measureJson() below still refuses anything
    // that would not fit, so the guard remains fail-closed, never truncating.
    char   vbuf[1024];
    // [Phase 3I] measureJson() BEFORE serializing, matching the live
    // /vibration guard: the exact length is known while the buffer is still
    // untouched, so an oversized payload is refused outright instead of being
    // written truncated and detected afterwards.
    const size_t rvNeed = measureJson(rv);
    if (rvNeed + 1u > sizeof(vbuf)) {
      Serial.printf("[TelemBuf] replay /vibration payload %u B exceeds buffer %u B -- keeping slot\n",
                    (unsigned)rvNeed, (unsigned)sizeof(vbuf));
      return false;  // ไม่ pop — จะ retry รอบหน้า
    }

    size_t vsz = serializeJson(rv, vbuf, sizeof(vbuf));
    if (vsz == 0 || vsz >= sizeof(vbuf) - 1) {
      Serial.printf("[TelemBuf] WARN: replay /vibration JSON truncated vsz=%u\n", (unsigned)vsz);
      return false;  // ไม่ pop — จะ retry รอบหน้า
    }

#ifdef DEBUG_MQTT_TIMING
    uint32_t t0_pubV = millis();
#endif
    bool pubOkV = mqttClient.publish(g_mqttTopic, vbuf, (int)vsz, false, MQTT_QOS);
#ifdef DEBUG_MQTT_TIMING
    dbgLogMqttPublish(g_mqttTopic, vsz, MQTT_QOS, pubOkV,
                      mqttClient.lastError(), millis() - t0_pubV);
#endif
    if (!pubOkV) {
      Serial.printf("[TelemBuf] replay /vibration publish FAILED (err=%d) — keeping slot\n",
                    mqttClient.lastError());
      return false;  // ไม่ pop ออก — จะ retry เมื่อ MQTT reconnect อีกครั้ง
    }
  }

  char buf[1536];  // [M1B-6] 1024 -> 1536, matches the enlarged replay doc
  size_t sz = serializeJson(r, buf, sizeof(buf));
  if (sz == 0 || sz >= sizeof(buf) - 1) {
    Serial.printf("[TelemBuf] WARN: replay JSON truncated sz=%u\n", (unsigned)sz);
    return false;  // ไม่ pop — จะ retry รอบหน้า
  }

#ifdef DEBUG_MQTT_TIMING
  uint32_t t0_pub1 = millis();
#endif
  bool pubOk1 = mqttClient.publish(g_mqttTopicSensor, buf, (int)sz, false, MQTT_QOS);
#ifdef DEBUG_MQTT_TIMING
  dbgLogMqttPublish(g_mqttTopicSensor, sz, MQTT_QOS, pubOk1,
                    mqttClient.lastError(), millis() - t0_pub1);
#endif
  if (!pubOk1) {
    Serial.printf("[TelemBuf] replay publish FAILED (err=%d) — keeping slot\n",
                  mqttClient.lastError());
    return false;  // ไม่ pop ออก — จะ retry เมื่อ MQTT reconnect อีกครั้ง
  }

  // Publish สำเร็จ — pop oldest slot ออก
  {
    if (xSemaphoreTake(mutexTelemBuf, pdMS_TO_TICKS(10)) == pdTRUE) {
      g_telemBufHead  = (g_telemBufHead + 1) % TELEM_BUF_SIZE;
      g_telemBufCount--;
      g_telemBufReplayedCount++;
      xSemaphoreGive(mutexTelemBuf);
    }
  }

  Serial.printf("[TelemBuf] replayed 1 slot (buffered_at=%lu)  remaining=%u  total_replayed=%lu\n",
                (unsigned long)snap.buffered_ts,
                (unsigned)g_telemBufCount,
                (unsigned long)g_telemBufReplayedCount);
  return true;
}

// ============================================================================
// MQTT OUTBOUND QUEUE — producer helper (Section 7 Item 3, design v16.5 §4.1)
// Dormant: no caller wired yet (Analytics is wired in a later checklist item;
// this commit implements Item 3 only). Overflow policy: drop-newest with
// counter — non-blocking xQueueSend; on a full queue the new message is
// dropped and nothing already queued is evicted (design v16.5 §4.1).
// ============================================================================
static bool enqueueMqttOutbound(MqttOutboundTopic_t topicId, const char* payload, size_t len, uint8_t qos) {
  if (queueMqttOutboundTrend == NULL || payload == NULL ||
      len == 0 || len >= MQTT_OUTBOUND_PAYLOAD_MAX) {
    return false;
  }

  MqttOutboundMsg_t msg;
  msg.topic_id = topicId;
  msg.len      = len;
  msg.qos      = qos;
  memcpy(msg.payload, payload, len);
  msg.payload[len] = '\0';

  if (xQueueSend(queueMqttOutboundTrend, &msg, 0) != pdTRUE) {
    g_trendEnqueueDropCount++;
    return false;
  }
  return true;
}

// ============================================================================
// CACHED MQTT CONNECTION-STATE READER (Section 7 Item 5, design v16.5 §4.2)
// Dormant: no caller wired yet (DisplayUpdate/Analytics/loopTask are wired in
// later checklist items). Uses the same xSemaphoreTake(mutexSystemState, ...)
// pattern already used elsewhere in the file for g_systemState reads.
// ============================================================================
static bool getMqttConnectedCached() {
  bool cached = false;
  if (xSemaphoreTake(mutexSystemState, pdMS_TO_TICKS(5)) == pdTRUE) {
    cached = g_systemState.mqttConnected;
    xSemaphoreGive(mutexSystemState);
  }
  return cached;
}

static void checkAndLatchFault(const VibrationData_t* data,
                                MachineState_t         newState,
                                int                    healthScore,
                                bool                   bearingConfirmed,
                                bool                   suppressLatch)  // [v16.3m]
{
  // [v16.3m] suppress latch ถ้า rms garbage หรือ warmup suppress
  if (suppressLatch) return;

  const bool healthLowNow = (healthScore <= FL_HEALTH_LOW_THOLD);

  uint8_t evCode = FL_EVT_NONE;

  if (bearingConfirmed && !g_flPrevBearing) {
    evCode = FL_EVT_BEARING;
  } else if (newState == STATE_CRITICAL &&
             g_flPrevState != STATE_CRITICAL) {
    evCode = FL_EVT_CRITICAL;
  } else if (healthLowNow && !g_flPrevHealthLow) {
    evCode = FL_EVT_HEALTH;
  } else if (newState == STATE_WARNING &&
             g_flPrevState == STATE_NORMAL) {
    evCode = FL_EVT_WARNING;
  }

  // Update trackers UNCONDITIONALLY — must run even when no event fires.
  // (taskStateMachine-local trackers, single writer/reader task -- no lock needed.)
  g_flPrevBearing   = bearingConfirmed;
  g_flPrevHealthLow = healthLowNow;
  g_flPrevState     = newState;

  if (evCode == FL_EVT_NONE) return;

  uint32_t epochNow = FL_TS_UNKNOWN;
  if (g_rtcValid && g_timeSync.synced) {
    DateTime _rtcNow; RTC_NOW_SAFE(_rtcNow);  // [v16.3g]
    const uint32_t cand = (uint32_t)_rtcNow.unixtime();
    if (cand >= FL_TS_MIN_VALID) {
      epochNow = cand;
    } else {
      Serial.printf("[LATCH] ts candidate=%lu < min — storing UNKNOWN\n",
                    (unsigned long)cand);
    }
  } else {
    Serial.printf("[LATCH] ts=UNKNOWN (rtcValid=%d synced=%d)\n",
                  (int)g_rtcValid, (int)g_timeSync.synced);
  }

  // ── mutexFaultLatch: guards g_fl + g_flCount + the fault_latch NVS namespace ──
  if (xSemaphoreTake(mutexFaultLatch, pdMS_TO_TICKS(20)) != pdTRUE) {
    Serial.println("[LATCH] mutex timeout — event dropped this cycle");
    return;
  }

  if (g_fl.pending) {
    const uint8_t inSev = faultSeverity(evCode);
    const uint8_t exSev = faultSeverity(g_fl.code);
    if (inSev <= exSev) {
      Serial.printf("[LATCH] SKIP ev=%u(%s) sev=%u — pending ev=%u(%s) sev=%u\n",
                    evCode, faultEventStr(evCode), inSev,
                    g_fl.code, faultEventStr(g_fl.code), exSev);
      xSemaphoreGive(mutexFaultLatch);
      return;
    }
    Serial.printf("[LATCH] OVERWRITE pending ev=%u sev=%u -> ev=%u sev=%u\n",
                  g_fl.code, exSev, evCode, inSev);
  }

  g_fl.code     = evCode;
  g_fl.ts       = epochNow;
  // [M1A] Latch the value that actually drove the decision: velocity_rms_overall
  // [mm/s] from FIFO RAW -> DSP, not the deprecated VRMS register figure.
  // Control only reaches here when suppressLatch was false, which already
  // required a valid, fresh carrier -- so this read cannot record a stale or
  // unavailable value. The fallback below exists solely so the field is never
  // indeterminate if that invariant is ever broken by a future edit.
  {
    float latchedVel = 0.0f;
    if (!readVelocityForAlarm(&latchedVel, NULL, NULL)) {
      latchedVel = 0.0f;
    }
    g_fl.rms = latchedVel;   // [mm/s] velocity_rms_overall (unit unchanged)
  }
  // [Phase 1] The kurtosis measurement is gone, but FaultLatch_t.kurtosis and
  // its NVS key FL_KEY_KURT are RETAINED so the persisted record layout is
  // unchanged -- an existing latch saved by earlier firmware still loads. The
  // slot is written 0.0f as an explicit placeholder and is no longer published
  // on any topic; it is not a measurement and must never be read as one.
  g_fl.kurtosis = 0.0f;
  g_flCount++;

  // [v16.3n] FIX: ipc1 stack overflow
  // saveFaultLatchNVS() เรียก Preferences (Flash NVS write) ซึ่งต้องการ IPC call ไป Core 1
  // ถ้าเรียกขณะถือ mutexFaultLatch จาก Core 0 → ipc1 task stack overflow → PANIC
  // แก้: snapshot ข้อมูลที่ต้องการก่อน แล้ว release mutex ก่อน NVS write
  // ป้องกัน mutex hold time ยาวขึ้นด้วย
  const uint8_t  snapCode   = g_fl.code;
  const uint32_t snapTs     = g_fl.ts;
  const float    snapRms    = g_fl.rms;
  const float    snapKurt   = g_fl.kurtosis;
  const uint32_t snapCount  = g_flCount;

  g_fl.pending = true;  // set ก่อน release mutex

  // [Phase2] prints snapRms (== g_fl.rms, the FIFO/DSP velocity recorded above)
  // instead of the removed legacy data->rms_overall.
  Serial.printf("[LATCH] LATCHED ev=%u(%s) sev=%u ts=%lu vel=%.2f kurt=%.3f n=%lu\n",
                evCode, faultEventStr(evCode), faultSeverity(evCode),
                (unsigned long)epochNow,
                snapRms, g_fl.kurtosis,
                (unsigned long)g_flCount);

  xSemaphoreGive(mutexFaultLatch);  // ← release mutex ก่อน NVS write

  // NVS write หลัง release mutex — ปลอดภัยจาก ipc1 stack overflow
  // g_fl ถูก set pending=true แล้ว ถ้า crash ตรงนี้จะ recover ได้จาก pending flag
  {
    Preferences p;
    p.begin(FL_NS, false);
    // [Task 7.3 -- TEMPORARY DIAGNOSTIC ONLY] NVS write entry/exit markers
    // -- this is the ACTUAL, reachable fault-latch write path (unlike the
    // unused standalone saveFaultLatchNVS() above), called from
    // checkAndLatchFault() (taskStateMachine, Core 0) whenever a new fault
    // is latched.
    Serial.printf("[NVS_BEGIN] %lu\n", (unsigned long)millis());
    p.putUChar(FL_KEY_PENDING, 0u);
    p.putUChar(FL_KEY_CODE,    snapCode);
    p.putUInt (FL_KEY_TS,      snapTs);
    p.putFloat(FL_KEY_RMS,     snapRms);
    p.putFloat(FL_KEY_KURT,    snapKurt);
    p.putUInt (FL_KEY_MAGIC,   FL_MAGIC_VALUE);
    p.putUChar(FL_KEY_PENDING, 1u);
    p.end();
    Serial.printf("[NVS_END]   %lu\n", (unsigned long)millis());
    Serial.printf("[LATCH] SAVE code=%u(%s) sev=%u ts=%lu rms=%.2f kurt=%.3f n=%lu\n",
                  snapCode, faultEventStr(snapCode), faultSeverity(snapCode),
                  (unsigned long)snapTs, snapRms, snapKurt,
                  (unsigned long)snapCount);
  }

  // [ADR-0006 D-1, Phase 2A] The FAULT_LATCH FIFO producer that stood here
  // (Commit 5, later narrowed to BEARING/CRITICAL by v16.6j) has been REMOVED.
  // A fault no longer starts a FIFO capture; it consumes the most recent
  // completed periodic capture instead, so the waveform attached to a fault
  // event is at most one capture period old. Periodic SCHEDULED capture is now
  // the sole FIFO initiator.
  //
  // Everything this function does ABOVE is unchanged and still runs for every
  // event type: evCode derivation, severity arbitration/SKIP/OVERWRITE, the
  // g_fl latch, NVS persistence, and the [LATCH]/[NVS_BEGIN]/[NVS_END] logs.
  // Only the capture request is gone -- fault DETECTION and fault EVENT
  // generation are untouched.

  return;  // mutex ถูก release ไปแล้วข้างบน ไม่ต้องทำอีก
}

// ?????? runtime_hour ??? state transition + periodic save (30 s)
static void updateRuntimeHour() {
  bool isRunning = (g_motorRunState == MOTOR_RUNNING);
  if (isRunning && !g_prevWasRunning) {
    g_runningStartMs = millis();
    g_lastNvsSaveMs  = millis();
  }
  if (!isRunning && g_prevWasRunning) {
    uint32_t elapsed = millis() - g_runningStartMs;
    g_runtimeHour += elapsed / 3600000.0f;
    saveRuntimeHour(g_runtimeHour);
  }
  if (isRunning && (millis() - g_lastNvsSaveMs >= NVS_SAVE_INTERVAL_MS)) {
    uint32_t elapsed     = millis() - g_runningStartMs;
    float    totalToSave = g_runtimeHour + (elapsed / 3600000.0f);
    saveRuntimeHour(totalToSave);
    g_lastNvsSaveMs = millis();
  }
  g_prevWasRunning = isRunning;
}

// ?????? runtime ??????? session ????????
static float getCurrentRuntimeHour() {
  if (g_motorRunState == MOTOR_RUNNING) {
    return g_runtimeHour + (millis() - g_runningStartMs) / 3600000.0f;
  }
  return g_runtimeHour;
}

// [v16.6g] Phase-2: Measurement Layer's sole calibration function. Bench
// diagnostics (CURRENT_DIAG, previous phase) confirmed CTR4A01's raw reading
// scales with CT_TURNS -- i.e. rawCurrentA is NOT engineering current -- so
// this is no longer an identity passthrough. Called exactly once, inside
// readCTR4A01Current() immediately after the raw mA->A conversion; every
// other consumer in the firmware (EMA, motor-state threshold, MQTT current
// field, trend buffer, analytics slope) reads the result of this call
// (current_a) and never references CT_TURNS/CT_RATIO_PRIMARY_A/
// CT_RATIO_SECONDARY_A directly.
static float compensateCurrent(float rawCurrentA)
{
    return rawCurrentA *
           (CT_RATIO_PRIMARY_A / CT_RATIO_SECONDARY_A)
           / CT_TURNS;
}

// [Commit 3/4A] Builds the semantic evidence updateMotorStateMachine() will
// act on, based on g_motorStateSource. Each branch translates its own raw
// measurement into {signalPresent, ageMs} only -- no raw values, thresholds,
// filter state, or source-specific config (RATED_RPM, nameplate/CT/percent)
// ever leave this function; updateMotorStateMachine() remains completely
// unaware of any of it. RPM/Proximity branches remain pure functions of
// their current inputs with no memory of previous calls. The Current branch
// (Commit 4A) now holds one piece of function-local filter state (the EMA
// accumulator) -- confined entirely to this function, never exposed
// elsewhere, still never touching the frozen MotorStateEvidence shape.
// PROXIMITY is not implemented yet -- placeholder mirrors RPM for now.
static MotorStateEvidence buildMotorStateEvidence(uint32_t timeSincePulseMs, float currentA,
                                                    EvidenceAvailability currentAvailability) {
  MotorStateEvidence ev{};   // [P2] value-initialize -- new fields can never be left
                             // uninitialized if a future source forgets to set them
  switch (g_motorStateSource) {
    case MOTOR_SRC_CURRENT: {
      // [v16.6g] Phase-2: currentA arrives already compensated -- readCTR4A01Current()
      // is now the single call site for compensateCurrent(), so Business
      // Logic here never touches CT_TURNS/CT_RATIO_PRIMARY_A/CT_RATIO_SECONDARY_A.
      // This is just a clarity alias, not a second compensation step.
      float engineeringCurrentA = currentA;

      // [P4-01] CURRENT_EVIDENCE_MAX_AGE_MS is a separate, independently-named
      // constant -- not a reuse of the ageStoppedMs expression below -- same
      // horizon by deliberate choice, same rationale as MAX_EMA_INTERVAL_US
      // vs. FORCE_STOP_TIMEOUT_MS for RPM: two different concerns (evidence
      // freshness vs. FSM business-timeout) sharing one knob would let a
      // future change to one silently change the other.
      // [v16.6j] Now defined at file scope (see near CURRENT_SAMPLE_INTERVAL_MS)
      // so publishTelemetry() can share this exact same definition for the
      // telemetry-freshness check -- was a local static const here only.

      // [P4-01] Function-local statics, confined entirely to this branch --
      // not globals, not visible outside buildMotorStateEvidence(). Detects
      // whether this cycle carries a genuinely new sample (g_lastCurrentSampleMs
      // changed since last seen) vs. a repeat of the same stale sample --
      // needed so the EMA can hold, rather than re-converge toward a repeated
      // stale value, on cycles where no new poll landed (see s_currentFiltered
      // "Valid, no new poll" comment below).
      static uint32_t s_lastSeenSampleMs     = 0;
      static float    s_currentFiltered      = 0.0f;
      static bool     s_currentLatched       = false;
      // [P4-01] Evidence validity -- a concept distinct from s_currentFiltered's
      // numeric value. "No data" and "0 A" are not the same thing; this flag
      // carries the "no data" meaning so the 0.0f reset below is never
      // mistaken for a claim that current is genuinely zero.
      static bool     s_currentEvidenceValid = false;

      uint32_t ageMsNow      = millis() - g_lastCurrentSampleMs;
      bool     isFreshSample = (g_lastCurrentSampleMs != s_lastSeenSampleMs);
      if (isFreshSample) {
        s_lastSeenSampleMs = g_lastCurrentSampleMs;
      }

      // [ADR Option E, Phase 1] When this cycle's current evidence is
      // expectedly unavailable (FifoDriver owns the bus), none of the
      // expiry/EMA/validity/hysteresis logic below runs -- s_currentFiltered,
      // s_currentLatched, and s_currentEvidenceValid all hold exactly the
      // values they had on the last cycle real data was processed. ageMsNow
      // itself (above) is still the true, honestly-computed elapsed time;
      // only the reaction to it is suppressed here, and separately via
      // ev.evidenceFrozen below for updateMotorStateMachine()'s own checks.
      if (currentAvailability != EvidenceAvailability::UNAVAILABLE_EXPECTED) {
        if (ageMsNow > CURRENT_EVIDENCE_MAX_AGE_MS) {
          // [P4-01] Invalid/Expired: reset to an inert baseline. This is an
          // IMPLEMENTATION DETAIL, not a semantic claim -- s_currentEvidenceValid
          // alone carries the "no data" meaning.
          s_currentFiltered      = 0.0f;
          s_currentLatched       = false;
          s_currentEvidenceValid = false;
          g_currentEvidenceValid = s_currentEvidenceValid;  // [P4-02] telemetry mirror only
          g_currentFilteredA     = s_currentFiltered;       // [v16.5i] paired mirror, same statement group
        } else if (isFreshSample) {
          if (!s_currentEvidenceValid) {
            // [P4-01] Recovery from Expired/Uninitialized: reseed directly from
            // the fresh sample -- do not blend with the stale/reset baseline,
            // mirroring g_rpmFiltered's own reseed-on-recovery behavior.
            s_currentFiltered = engineeringCurrentA;
          } else {
            // [Commit 4A] EMA filter -- see CURRENT_EMA_ALPHA for full justification.
            s_currentFiltered =
                CURRENT_EMA_ALPHA * engineeringCurrentA +
                (1.0f - CURRENT_EMA_ALPHA) * s_currentFiltered;
          }
          s_currentEvidenceValid = true;
          g_currentEvidenceValid = s_currentEvidenceValid;  // [P4-02] telemetry mirror only
          g_currentFilteredA     = s_currentFiltered;       // [v16.5i] paired mirror, same statement group
        }
        // [P4-01] else: Valid, no new poll this cycle -- hold s_currentFiltered
        // unchanged rather than re-feeding the same stale engineeringCurrentA
        // into the EMA again (previously this ran unconditionally every call).

        // [P2] Hysteresis latch (Schmitt trigger) on the filtered value --
        // replaces the old single stateless threshold entirely. Latches true
        // at/above CURRENT_ON_THRESHOLD_A, stays true until dropping below
        // CURRENT_OFF_THRESHOLD_A. Unchanged -- still evaluated every cycle
        // against whatever s_currentFiltered currently holds.
        if (!s_currentLatched && s_currentFiltered >= CURRENT_ON_THRESHOLD_A) {
          s_currentLatched = true;
        } else if (s_currentLatched && s_currentFiltered < CURRENT_OFF_THRESHOLD_A) {
          s_currentLatched = false;
        }
      }
      ev.signalPresent    = s_currentLatched;
      ev.ageMs            = ageMsNow;
      ev.evidenceFrozen   = (currentAvailability == EvidenceAvailability::UNAVAILABLE_EXPECTED);
      ev.absentStoppingMs = NO_CURRENT_STOPPING_MS;
      ev.absentStoppedMs  = FORCE_CURRENT_STOPPED_MS;
      // [v16.5.6] Deliberately derived, not a hardcoded literal: ageMs above
      // is time-since-last-CT-poll, so its normal range is intrinsically set
      // by CURRENT_SAMPLE_INTERVAL_MS (the CT sensor's own 500ms sampling
      // cadence) -- not by RPM pulse timing. Deriving from it keeps this
      // threshold automatically in sync if the sampling cadence ever
      // changes, and prevents false STOPPING/STOPPED caused by normal sensor
      // sampling cadence (rather than a real current loss). x3/x10 preserve
      // the same ~1500ms/~5000ms horizons this source already used on the
      // absentStoppingMs/absentStoppedMs axis above.
      ev.ageStoppingMs    = CURRENT_SAMPLE_INTERVAL_MS * 3;
      ev.ageStoppedMs     = CURRENT_SAMPLE_INTERVAL_MS * 10;

#ifdef DEBUG_CURRENT_PATH
      // [P3-01] Diagnostic-only, 1 Hz -- decision-layer diagnostics, renamed
      // from [CURRENT_DIAG] to [CURRENT_DECISION]. This block owns only
      // decision-derived values; it no longer re-prints measurement-owned
      // data (rawA/engineeringA/CT constants/scale remain the sole
      // responsibility of readCTR4A01Current()'s [CURRENT_DIAG] block).
      // ageMs (ev.ageMs) is a decision-derived value computed from the
      // measurement-owned g_lastCurrentSampleMs timestamp -- it gives this
      // block a staleness signal without duplicating the measurement value.
      {
        static uint32_t s_lastDiagMs = 0;
        uint32_t nowMs = millis();
        if (nowMs - s_lastDiagMs >= 1000) {
          s_lastDiagMs = nowMs;
          LOGD("[CURRENT_DECISION]\r\n");
          // [P2] "threshold=" is kept, unchanged name, as the single source of
          // truth for the ON value -- it is an ALIAS of CURRENT_ON_THRESHOLD_A,
          // not a distinct measurement. The ON value is intentionally NOT
          // repeated below under a second key, to avoid two fields claiming
          // the same value with different names.
          LOGD("threshold=%.3f\n",    CURRENT_ON_THRESHOLD_A);
          LOGD("signalPresent=%d\n",  (int)ev.signalPresent);
          // [P2] Structured fields -- the OFF threshold is new information (no
          // prior field carried it); the ON value is deliberately not repeated
          // here since "threshold=" above already is that value.
          LOGD("ema_current=%.3f\n",           s_currentFiltered);
          LOGD("signal_present=%d\n",          (int)ev.signalPresent);
          LOGD("current_threshold_off=%.3f\n", CURRENT_OFF_THRESHOLD_A);
          LOGD("motor_state=%d\n",              (int)g_motorRunState);
          LOGD("ageMs=%lu\n",                   (unsigned long)ev.ageMs);
          LOGD("source=current\r\n");
        }
      }
#endif
      break;
    }
    case MOTOR_SRC_PROXIMITY:
      // [placeholder] not implemented yet -- falls back to the RPM computation
      // [v16.5.4] freshness-gated like MOTOR_SRC_RPM below
      ev.signalPresent = g_rpmEvidence.valid &&
                         (g_rpmEvidence.rpm >= (RATED_RPM - RATED_RPM_TOL)) &&
                         (g_rpmEvidence.rpm <= (RATED_RPM + RATED_RPM_TOL));
      ev.ageMs = timeSincePulseMs;
      ev.absentStoppingMs = ABSENT_STOPPING_MS;
      ev.absentStoppedMs  = ABSENT_STOPPED_MS;
      ev.ageStoppingMs    = NO_PULSE_STOPPING_MS;   // [v16.5.6] unchanged RPM value
      ev.ageStoppedMs     = FORCE_STOP_TIMEOUT_MS;  // [v16.5.6] unchanged RPM value
      break;
    case MOTOR_SRC_RPM:
    default:
      // [v16.5.4] Improvement 3: signalPresent now requires FRESH evidence --
      // g_rpmEvidence.valid is false after long idle (MAX_EMA_INTERVAL_US), so
      // a stale EMA that happens to still sit in the rated band can never
      // report signalPresent=true. In-band thresholds unchanged; ev.ageMs
      // stays timeSincePulseMs exactly as before, so the FSM's
      // STOPPING/STOPPED timing is untouched. (g_rpmEvidence.rpm ==
      // g_rpmFiltered -- same value, read through the freshness struct.)
      ev.signalPresent = g_rpmEvidence.valid &&
                         (g_rpmEvidence.rpm >= (RATED_RPM - RATED_RPM_TOL)) &&
                         (g_rpmEvidence.rpm <= (RATED_RPM + RATED_RPM_TOL));
      ev.ageMs = timeSincePulseMs;
      ev.absentStoppingMs = ABSENT_STOPPING_MS;   // [P2] unchanged RPM value
      ev.absentStoppedMs  = ABSENT_STOPPED_MS;    // [P2] unchanged RPM value
      ev.ageStoppingMs    = NO_PULSE_STOPPING_MS;   // [v16.5.6] unchanged RPM value
      ev.ageStoppedMs     = FORCE_STOP_TIMEOUT_MS;  // [v16.5.6] unchanged RPM value
      break;
  }
  return ev;
}

// [Commit 3A] Motor State Machine -- consumes only semantic evidence
// (signalPresent, ageMs). Knows nothing about RPM, Current, CT ratios,
// nameplate values, or threshold percentages -- those all stay inside
// buildMotorStateEvidence(). Responsibilities: STARTING/RUNNING debounce
// (warmup), STOPPING/STOPPED (timeout, now via TWO independent triggers --
// see below), and state transition. No RPM-value mutation happens here
// (evidence carries no numeric value to mutate); the existing g_rpmFiltered
// decay/zero on STOPPED/STOPPING lives in processRPM(), gated on the
// resulting state -- see that function.
//
// Two independent, source-agnostic triggers can each drive STOPPING/STOPPED:
//   (a) evidence itself is stale (ageMs) -- "we don't currently know"
//   (b) evidence is fresh but has been continuously absent (signalPresent
//       false) for a while -- "we know, and it says not-running"
// For RPM these two conditions normally coincide (no pulses = both stale
// and absent at once), so behavior is effectively unchanged for the typical
// no-pulse-at-all case. They diverge only if RPM pulses keep arriving
// (ageMs stays low) while never entering the rated band for longer than
// ABSENT_STOPPING_MS/ABSENT_STOPPED_MS -- previously this sat in STARTING
// indefinitely; it now eventually reaches STOPPING/STOPPED. This is an
// intentional, accepted behavior change (see Commit 3A review) needed to
// fix the equivalent, much more likely Current-source gap: CTR4A01 comms
// staying healthy (ageMs low) while current genuinely reads "not running".
static void updateMotorStateMachine(const MotorStateEvidence& evidence) {
  if (evidence.signalPresent) {
    g_absentSince = 0;                    // [Commit 3A] reset absence timer
  } else if (g_absentSince == 0) {
    g_absentSince = millis();             // [Commit 3A] start absence timer
  }
  uint32_t absentMs = evidence.signalPresent ? 0 : (millis() - g_absentSince);

  // [v16.5.3-rpmdiag1] DIAGNOSTIC ONLY -- read-only. Reports every time
  // signalPresent flips (evidence-only fields; no source-specific data);
  // never consulted by any branch below.
  if (g_diagSignalPresentInit && evidence.signalPresent != g_diagPrevSignalPresent) {
    Serial.printf("[SIGNAL]\n%d->%d\n",
                  (int)g_diagPrevSignalPresent, (int)evidence.signalPresent);
  }
  g_diagPrevSignalPresent  = evidence.signalPresent;
  g_diagSignalPresentInit  = true;

  // [v16.5.3-rpmdiag1] DIAGNOSTIC ONLY -- captures the state as it stands
  // before this call's branches may overwrite it, purely so the
  // [MOTOR-TRANSITION] prints below can report old/new. Read-only.
  MotorRunState_t diagOldState = g_motorRunState;

  // [v16.5.6] FORCE_STOP_TIMEOUT_MS/NO_PULSE_STOPPING_MS replaced with
  // evidence.ageStoppedMs/evidence.ageStoppingMs -- source-specific
  // thresholds populated per-branch in buildMotorStateEvidence() (RPM/
  // PROXIMITY keep the old global values numerically unchanged; CURRENT now
  // derives them from CURRENT_SAMPLE_INTERVAL_MS instead of being checked
  // against RPM-tuned constants). No other transition logic changed.
  // [ADR Option E, Phase 1] evidence.evidenceFrozen (always false for
  // RPM/PROXIMITY) suppresses only the ageMs half of each condition below --
  // the absentMs/signalPresent half is untouched, since holding
  // signalPresent constant during a frozen cycle (buildMotorStateEvidence())
  // already keeps absentMs from accumulating on its own.
  bool ageStoppedTripped  = !evidence.evidenceFrozen && (evidence.ageMs > evidence.ageStoppedMs);
  bool ageStoppingTripped = !evidence.evidenceFrozen && (evidence.ageMs > evidence.ageStoppingMs);
  if (ageStoppedTripped || absentMs > evidence.absentStoppedMs) {
    if (diagOldState != MOTOR_STOPPED) {
      Serial.printf("[MOTOR-TRANSITION]\nold=%d new=%d reason=%s\n"
                    "signalPresent=%d age=%lu ageThreshold=%lu absentMs=%lu absentThreshold=%lu\n",
                    (int)diagOldState, (int)MOTOR_STOPPED,
                    ageStoppedTripped ? "ageMs>ageStoppedMs" : "absentMs>absentStoppedMs",
                    (int)evidence.signalPresent,
                    (unsigned long)evidence.ageMs, (unsigned long)evidence.ageStoppedMs,
                    (unsigned long)absentMs, (unsigned long)evidence.absentStoppedMs);
    }
    g_motorRunState  = MOTOR_STOPPED;
    g_runInBandSince = 0;                 // [v16.3z] reset warm-up
  } else if (ageStoppingTripped || absentMs > evidence.absentStoppingMs) {
    if (diagOldState != MOTOR_STOPPING) {
      Serial.printf("[MOTOR-TRANSITION]\nold=%d new=%d reason=%s\n"
                    "signalPresent=%d age=%lu ageThreshold=%lu absentMs=%lu absentThreshold=%lu\n",
                    (int)diagOldState, (int)MOTOR_STOPPING,
                    ageStoppingTripped ? "ageMs>ageStoppingMs" : "absentMs>absentStoppingMs",
                    (int)evidence.signalPresent,
                    (unsigned long)evidence.ageMs, (unsigned long)evidence.ageStoppingMs,
                    (unsigned long)absentMs, (unsigned long)evidence.absentStoppingMs);
    }
    g_motorRunState  = MOTOR_STOPPING;
    g_runInBandSince = 0;                 // [v16.3z] reset warm-up
  } else {
    if (evidence.signalPresent) {
      // [v16.3z] Warm-up debounce: ต้อง in-band ต่อเนื่อง RUNNING_WARMUP_MS ก่อนเป็น RUNNING
      // กัน spurious STOPPED→RUNNING จาก pulse ที่หายชั่วขณะ (ซึ่งจะ flush freq_ratio 240 slots)
      if (g_runInBandSince == 0) g_runInBandSince = millis();
      if ((millis() - g_runInBandSince) >= RUNNING_WARMUP_MS) {
        if (g_motorRunState != MOTOR_RUNNING) {
          Serial.printf("[MOTOR] Warm-up complete (in-band %.1fs) -> RUNNING\n",
                        RUNNING_WARMUP_MS / 1000.0f);
          Serial.printf("[MOTOR-TRANSITION]\nold=%d new=%d reason=%s\n"
                        "signalPresent=%d ageMs=%lu absentMs=%lu\n",
                        (int)diagOldState, (int)MOTOR_RUNNING, "warmup_complete",
                        (int)evidence.signalPresent,
                        (unsigned long)evidence.ageMs, (unsigned long)absentMs);
          // [vNext] arm once, only on the actual STARTING->RUNNING transition --
          // TEMPORARY startup-settling suppression for FAULT_LATCH only.
          g_motorRunFaultLatchHoldoff = MOTOR_RUN_FAULT_LATCH_HOLDOFF_READS;
        }
        g_motorRunState = MOTOR_RUNNING;
      } else {
        if (diagOldState != MOTOR_STARTING) {
          Serial.printf("[MOTOR-TRANSITION]\nold=%d new=%d reason=%s\n"
                        "signalPresent=%d ageMs=%lu absentMs=%lu\n",
                        (int)diagOldState, (int)MOTOR_STARTING, "warmup_in_progress",
                        (int)evidence.signalPresent,
                        (unsigned long)evidence.ageMs, (unsigned long)absentMs);
        }
        g_motorRunState = MOTOR_STARTING; // ยังนับ warm-up อยู่
      }
    } else {
      if (diagOldState != MOTOR_STARTING) {
        Serial.printf("[MOTOR-TRANSITION]\nold=%d new=%d reason=%s\n"
                      "signalPresent=%d ageMs=%lu absentMs=%lu\n",
                      (int)diagOldState, (int)MOTOR_STARTING, "signal_absent_fresh",
                      (int)evidence.signalPresent,
                      (unsigned long)evidence.ageMs, (unsigned long)absentMs);
      }
      g_runInBandSince = 0;               // [v16.3z] หลุด band -> reset warm-up
      g_motorRunState  = MOTOR_STARTING;
    }
  }
}

// Process RPM -- ???? ISR vars -> ????? rpm / motor_state / prox / runtime_hour
// ???????? taskStateMachine ??? 250 ms (Core 0, no mutex needed)
static void processRPM(VibrationData_t* data) {
  uint32_t interval, pulseCopy;
  noInterrupts();
  interval  = g_rpmPulseInterval;
  pulseCopy = g_rpmTotalPulses;
  interrupts();

  bool newPulse = (pulseCopy != g_rpmLastPulseCount);
  if (newPulse) g_rpmLastPulseMillis = millis();
  g_rpmLastPulseCount = pulseCopy;

  uint32_t timeSincePulseMs = millis() - g_rpmLastPulseMillis;

  // ---------- RPM Calculation (EMA filtered) ----------
  // [v16.5.4] Improvement 1: EMA invalidation after long idle.
  // A pulse whose measured interval spans a gap > MAX_EMA_INTERVAL_US is a
  // restart-after-idle artifact (its rpmRaw is meaningless -- it averages the
  // whole idle period), so instead of feeding it into the EMA (which
  // previously dragged the stale value around), it invalidates the EMA
  // outright. The FIRST valid pulse after that reseeds g_rpmFiltered from its
  // raw RPM -- tracking restarts from a clean state, no blending with stale
  // history. Normal running (interval << MAX_EMA_INTERVAL_US) takes the
  // original EMA path unchanged.
  if (newPulse && interval > MAX_EMA_INTERVAL_US) {
    if (g_rpmEvidence.valid) {
      Serial.printf("[RPM-EMA] Invalidated -- pulse gap %.1fs > %.1fs (stale EMA %.1f discarded)\n",
                    interval / 1000000.0f, MAX_EMA_INTERVAL_US / 1000000.0f, g_rpmFiltered);
    }
    g_rpmEvidence.valid = false;
    g_rpmFiltered       = 0.0f;
  } else if (newPulse && interval >= RPM_MIN_INTERVAL_US) {
    float rpmRaw = (60000000.0f / interval) / PULSE_PER_REV;
    if (rpmRaw <= MAX_RPM * SPIKE_REJECT_FACTOR) {
      if (!g_rpmEvidence.valid) {
        // [v16.5.4] clean restart: seed EMA from the first valid pulse
        g_rpmFiltered       = rpmRaw;
        g_rpmEvidence.valid = true;
        Serial.printf("[RPM-EMA] Reseeded from first valid pulse -- rpm=%.1f\n", rpmRaw);
      } else {
        g_rpmFiltered = RPM_SMOOTH_ALPHA * rpmRaw
                      + (1.0f - RPM_SMOOTH_ALPHA) * g_rpmFiltered;
      }
    }
  }

  // [v16.5.4] Improvements 1+3: idle invalidation without pulses + freshness
  // bookkeeping. Reuses timeSincePulseMs -- the SAME clock the FSM itself
  // uses for FORCE_STOP_TIMEOUT_MS -- instead of a second, independent
  // "time since last accepted EMA update" clock, so EMA freshness and FSM
  // staleness can never diverge (e.g. under a stream of spike-rejected
  // pulses that keep timeSincePulseMs low without ever updating the EMA).
  // If no pulse at all has been seen for longer than MAX_EMA_INTERVAL_US,
  // invalidate the EMA here as well -- evidence must not survive
  // indefinitely across a motor stop. This fires at the same horizon as
  // FORCE_STOP_TIMEOUT_MS today (see MAX_EMA_INTERVAL_US), i.e. at/after the
  // moment the FSM already force-stops on evidence age, so no
  // RUNNING/STOPPING/STOPPED timing changes.
  {
    if (g_rpmEvidence.valid && timeSincePulseMs > (uint32_t)(MAX_EMA_INTERVAL_US / 1000UL)) {
      Serial.printf("[RPM-EMA] Invalidated -- no pulse for %lums (stale EMA %.1f discarded)\n",
                    (unsigned long)timeSincePulseMs, g_rpmFiltered);
      g_rpmEvidence.valid = false;
      g_rpmFiltered       = 0.0f;
    }
    g_rpmEvidence.rpm   = g_rpmFiltered;
    g_rpmEvidence.ageMs = timeSincePulseMs;
  }

  // [v16.5.3-rpmdiag1] DIAGNOSTIC ONLY -- read-only recomputation of the same
  // rpmRaw expression above (or 0 when the existing gate condition is false),
  // purely so it can be logged here and inside updateMotorStateMachine()
  // without changing that function's signature or touching the real
  // g_rpmFiltered EMA update above. Also mirrors pulseCount/timeSincePulseMs
  // for the same reason. None of this feeds back into g_rpmFiltered or any
  // decision.
  g_diagRpmRaw = (newPulse && interval >= RPM_MIN_INTERVAL_US)
                   ? ((60000000.0f / interval) / PULSE_PER_REV)
                   : 0.0f;
  g_diagPulseCount       = pulseCopy;
  g_diagTimeSincePulseMs = timeSincePulseMs;

  // [v16.5.3-rpmdiag1] DIAGNOSTIC ONLY -- [PULSE] fires once per detected new
  // pulse (naturally bounded to processRPM()'s own ~250ms/4Hz call rate, so
  // never exceeds 5 prints/sec). Printed here, not inside rpmISR() itself --
  // Serial I/O from an IRAM ISR is unsafe and would itself alter timing,
  // which this diagnostic build must not do.
  if (newPulse) {
    // [v16.5e] Serial output silenced for log readability -- calculation
    // above (g_diagRpmRaw/g_diagPulseCount/g_diagTimeSincePulseMs) unchanged.
    // Serial.printf("[PULSE]\nintervalUs=%lu rpmRaw=%.1f pulseCount=%lu\n",
    //               (unsigned long)interval, g_diagRpmRaw, (unsigned long)pulseCopy);
  }

  // ---------- Motor State Machine ----------
  MotorStateEvidence evidence = buildMotorStateEvidence(timeSincePulseMs, data->current_a,
                                                          data->current_availability);
  updateMotorStateMachine(evidence);

  // [v16.5.3-rpmdiag1] DIAGNOSTIC ONLY -- unconditional per-cycle snapshot.
  // processRPM() is invoked once per dequeued sample at the existing ~250ms
  // sensor-read cadence, so this naturally prints every ~250ms without adding
  // a separate timer. Read-only; state/evidence were already fully decided
  // above by the untouched logic.
  LOGD("[MOTOR-DIAG]\nstate=%d signalPresent=%d pulseCount=%lu rpmRaw=%.1f "
                "rpmFiltered=%.1f timeSincePulseMs=%lu ageMs=%lu absentMs=%lu\n",
                (int)g_motorRunState, (int)evidence.signalPresent,
                (unsigned long)g_diagPulseCount, g_diagRpmRaw, g_rpmFiltered,
                (unsigned long)g_diagTimeSincePulseMs, (unsigned long)evidence.ageMs,
                (unsigned long)(evidence.signalPresent ? 0 : (millis() - g_absentSince)));

  // [Commit 3] RPM-specific signal conditioning on STOPPED/STOPPING -- lives
  // here because MotorStateEvidence is now purely semantic (no mutable
  // numeric value to carry this). Gated on the resulting state rather than
  // re-checking timeSincePulseMs directly, since MOTOR_STOPPED/MOTOR_STOPPING
  // are only ever set by updateMotorStateMachine()'s ageMs timeout checks --
  // same net effect on g_rpmReported, same timing, as the original inline code.
  // [Recommendation B] Presentation-only: applies to g_rpmReported, never to
  // g_rpmFiltered. g_rpmFiltered stays a pure, never-reset EMA of pulse
  // timing (buildMotorStateEvidence()'s only input); g_rpmReported is the
  // telemetry/data->rpm source only.
  if (g_motorRunState == MOTOR_STOPPED) {
    g_rpmReported = 0.0f;
  } else if (g_motorRunState == MOTOR_STOPPING) {
    g_rpmReported *= 0.80f;
    if (g_rpmReported < MIN_RPM_VALID) g_rpmReported = 0.0f;
  } else {
    g_rpmReported = g_rpmFiltered;
  }

  // v15.2 Fix 18: Reset g_velPeakHold เมื่อ motor transition → STOPPED
  // ป้องกัน peak hold สะสมค่า impulse จาก deceleration ค้างถึง publish ถัดไป
  // Core 0 writes g_velPeakHold / Core 1 reads+resets -- atomic float (4-byte aligned)
  if (g_motorRunState == MOTOR_STOPPED &&
      g_prevMotorRunState != MOTOR_STOPPED) {
    g_trendFreqFlushed   = false;  // ต้องตัดสินใจ resume/clear อีกครั้งเมื่อ start ใหม่
    g_motorStoppedSince  = millis();  // [v16.3aa] เริ่มจับเวลาหยุด เพื่อตัดสิน resume vs clear
    g_tempAtStop         = data->temperature;  // [v16.3ad] จำ temp ตอนหยุด (ตรวจ cold start ตอน resume)
    Serial.println("[MOTOR] STOPPED transition -- freeze analytics");
  }
  g_prevMotorRunState = g_motorRunState;

  // [v16.3ad] Point 3: ตัดสิน resume/clear จาก "machine cycle" (thermal) ไม่ใช่แค่ magic number เวลา
  //   - หยุดนาน 40 นาที แต่ bearing ยังร้อน → trend ยังต่อได้ (RESUME)
  //   - เย็นลงจริง (cold start) → CLEAR แม้เวลาหยุดสั้น
  //   policy เวลา (configurable) ยังใช้เป็น fallback ร่วมกับ thermal
  // [Phase2] On entry to RUNNING, decide once whether the previous run history
  // may be resumed or must be discarded. The freq_ratio flush,
  // g_freqDriftSuppress, the legacy-EMA reseed (g_resumeReinit) and the
  // ANALYTICS_CLEAR command all went away with the pipelines they served;
  // g_lastResumeGapS survives because /trend still publishes trend_gap_s.
  if (g_motorRunState == MOTOR_RUNNING && !g_trendFreqFlushed) {
    uint32_t stoppedMs = (g_motorStoppedSince == 0) ? 0 : (millis() - g_motorStoppedSince);
    float    tempDrop  = (g_tempAtStop > 0.0f) ? (g_tempAtStop - data->temperature) : 0.0f;
    bool     coldStart = (tempDrop >= COLD_START_TEMP_DROP_C);

    bool clearTrend;
    if (g_trendPersistence == TP_ALWAYS_RESUME)      clearTrend = false;
    else if (g_trendPersistence == TP_ALWAYS_CLEAR)  clearTrend = true;
    else clearTrend = (stoppedMs > trendClearThresholdMs()) && coldStart;

    if (clearTrend) {
      memset(g_tempBuf, 0, sizeof(g_tempBuf));
      g_tempHead  = 0;
      g_tempCount = 0;
      Serial.printf("[MOTOR] RUNNING -- CLEAR trend (stop=%lus, tempDrop=%.1fC cold=%d, policy=%s)\n",
                    (unsigned long)(stoppedMs/1000), tempDrop, (int)coldStart, trendPersistenceStr());
    } else {
      g_lastResumeGapS = stoppedMs / 1000;
      Serial.printf("[MOTOR] RUNNING -- RESUME, trend preserved (stop=%lus, tempDrop=%.1fC, bearing warm)\n",
                    (unsigned long)(stoppedMs/1000), tempDrop);
    }
    g_trendFreqFlushed = true;
  }

  // ---------- Prox Signal Quality ----------
  // 1 = pulse ???? | 0 = Fault / ?????? pulse
  uint8_t prox;
  if (g_motorRunState == MOTOR_STOPPED) {
    prox = 0;
  } else if (g_motorRunState == MOTOR_RUNNING &&
             timeSincePulseMs > FAULT_WINDOW_MS) {
    prox = 0;
  } else {
    prox = 1;
  }

  // ---------- Runtime Hour Update ----------
  updateRuntimeHour();

  // ---------- Write to shared VibrationData_t ----------
  data->rpm          = roundf(g_rpmReported * 10.0f) / 10.0f;
  data->motor_state  = (uint8_t)g_motorRunState;
  data->runtime_hour = roundf(getCurrentRuntimeHour() * 10000.0f) / 10000.0f;
  data->prox         = prox;
}

// [v16.5.4] Business Decision computation -- single owner of this formula,
// called only from captureTelemetrySnapshot() below. Previously this exact
// formula was duplicated inline in TWO places (publishTelemetry() and the
// fault-latch replay block in taskNetwork()); both now read the precomputed
// snap->health_score instead of recomputing it, which is what eliminates the
// duplication. Formula and MOTOR_RUNNING gate are unchanged from v16.0 --
// only given a name and a single call site. Kept as its own function (not
// inlined into captureTelemetrySnapshot()) so the capture function stays a
// pure data-movement layer: it invokes an already-named decision, it does
// not itself contain the decision's arithmetic.
// [M1A] Are BOTH vibration thresholds explicitly configured and coherent?
// Until this returns true no vibration WARNING/CRITICAL decision may be made
// and no health score may be normalized. Both operands are currently the
// negative VIB_THRESHOLD_UNSET sentinel, so this returns false by construction
// -- it is not a runtime flag someone forgot to set, it is the documented
// state of the product pending re-baselining.
static inline bool vibThresholdsConfigured() {
  return (VIB_WARNING_MMS  > 0.0f) &&
         (VIB_CRITICAL_MMS > 0.0f) &&
         (VIB_CRITICAL_MMS > VIB_WARNING_MMS);
}

// [Phase 3J] The one place vibration_status is DETERMINED. Lifted verbatim from
// the ternary the live serializers already used, so the resulting value is
// identical to what /vibration and /decision published before. It is read once
// live at publish time and once at capture time by pushTelemBuf(), which is
// what lets a replayed slot report the status it actually had rather than the
// status the device happens to be in when the backlog finally drains.
// [Phase 3J] PURE FORMATTERS -- no evaluation, no thresholds, no state read.
// They exist so the live serializer and the buffered-replay serializer render
// the same already-decided values through the same single expression. Adding a
// second place that DECIDES a machine condition is exactly what these prevent.
//
// Placed HERE, after every typedef in the file, on purpose: Arduino inserts all
// auto-generated prototypes immediately before the FIRST function definition,
// so a helper defined earlier would push that insertion point ahead of
// VibrationData_t / AnalysisReason_t / MqttOutboundTopic_t and break their
// prototypes. Both callers sit above this line and reach it via the
// auto-prototype, which is exactly what that mechanism is for.

// Renders the capture-time alarm state. It does not evaluate anything: the
// verdict was formed by the RMS state machine (hysteresis + capture
// persistence) long before this is called, and is passed in. A non-RUNNING
// motor reports NORMAL, matching the live rule that no alarm is asserted while
// the machine is not turning.
static const char* alarmLevelStr(MachineState_t state, uint8_t motorState) {
  if (motorState != 2) return "NORMAL";          // not RUNNING -> no alarm asserted
  return (state == STATE_CRITICAL) ? "CRITICAL"
       : (state == STATE_WARNING)  ? "WARNING"
                                   : "NORMAL";
}

// Renders a VibrationStatus_t. The three strings are the exact ones the live
// serializer has always emitted.
static const char* vibStatusStr(VibrationStatus_t st) {
  return (st == VIB_STATUS_UNAVAILABLE)      ? "UNAVAILABLE"
       : (st == VIB_STATUS_THRESHOLDS_UNSET) ? "THRESHOLDS_UNSET"
                                             : "OK";
}

static VibrationStatus_t currentVibStatus() {
  if (g_vibUnavailable)          return VIB_STATUS_UNAVAILABLE;
  if (!vibThresholdsConfigured()) return VIB_STATUS_THRESHOLDS_UNSET;
  return VIB_STATUS_OK;
}

// [M1A] Core 0's read of the Core 1 velocity carrier, with freshness applied.
//
// Returns true ONLY when a genuinely usable velocity figure exists:
//   - carrier readable
//   - carrier.valid  (the DSP produced a real result for that capture)
//   - age <= VIB_VELOCITY_MAX_AGE_MS_TBD
//
// A false return means VIBRATION_UNAVAILABLE. It explicitly does NOT mean
// "zero vibration": *outMmS is left at 0.0f purely so the caller never reads
// an indeterminate float, and every caller is required to branch on the
// return value, never on the magnitude.
//
// Short mutex timeout: this runs in the 250 ms Core 0 poll loop, so it must
// never stall the Modbus cadence. A timeout is treated as unavailable, which
// is the fail-closed direction.
// [S21] outCaptureId is additive: the alarm-persistence counter must advance
// once per genuinely new measurement, not once per 250 ms poll of the same
// one. Nothing else about this function's contract changes. Deliberately NOT
// a defaulted parameter -- the .ino auto-prototype generator does not carry
// default arguments into the forward declaration, which breaks every caller
// that appears above this definition. All call sites pass it explicitly.
static bool readVelocityForAlarm(float* outMmS, uint32_t* outAgeMs,
                                 uint32_t* outCaptureId) {
  if (outMmS)      *outMmS      = 0.0f;
  if (outAgeMs)    *outAgeMs    = UINT32_MAX;
  if (outCaptureId) *outCaptureId = 0u;

  if (mutexVelCarrier == NULL) {
    return false;
  }

  VelocityCarrier_t snap;
  if (xSemaphoreTake(mutexVelCarrier, pdMS_TO_TICKS(5)) != pdTRUE) {
    return false;  // fail closed
  }
  snap = g_velCarrier;               // whole-struct copy: no torn read
  xSemaphoreGive(mutexVelCarrier);

  if (!snap.valid || snap.timestampMs == 0u) {
    return false;
  }

  // millis() wraps at ~49.7 days; unsigned subtraction stays correct across
  // the wrap, so this needs no special-casing.
  const uint32_t age = millis() - snap.timestampMs;
  if (outAgeMs) *outAgeMs = age;
  if (age > VIB_VELOCITY_MAX_AGE_MS_TBD) {
    return false;  // stale -> VIBRATION_UNAVAILABLE
  }

  if (outMmS) *outMmS = snap.overall;
  if (outCaptureId) *outCaptureId = snap.captureId;   // [S21]
  return true;
}

// [M1A] Sentinel for "health score could not be determined". Distinct from any
// real 0..100 score. Consumers MUST treat this as UNKNOWN and must not render
// or trend it as a low score.
#define HEALTH_SCORE_UNKNOWN (-1)

// [M1A] Health score now derives from velocity_rms_overall, not the deprecated
// VRMS-register rms_overall.
//
// Returns HEALTH_SCORE_UNKNOWN whenever a score cannot be honestly computed:
//   - motor not RUNNING                    (pre-existing semantics: not assessed)
//   - velocity unavailable or stale        (M1A requirement 7)
//   - vibration thresholds not configured  (M1A requirement 6)
//
// The threshold case matters and is easy to miss: the legacy normalization
// span BASELINE_RMS..CRITICAL_RMS was calibrated against the VRMS-register
// metric. Re-using that span for velocity would be inventing a threshold
// mapping by the back door -- exactly what requirement 6 forbids -- so the
// score is reported UNKNOWN rather than computed from an invalid scale.
static int computeHealthScore(const VibrationData_t* data) {
  // [M1A-CLEANUP] STOPPED/STARTING/STOPPING now reports UNKNOWN, not 100.
  //
  // The previous `return 100` was the pre-M1A convention "not assessed => assume
  // healthy". Once M1A began publishing vibration_status, that produced a
  // self-contradicting message: health_score=100 (healthy) alongside
  // vibration_status=UNAVAILABLE (no vibration evidence exists) -- observed on
  // 13 consecutive /decision samples during the 2026-08-19 validation run.
  //
  // UNKNOWN is the honest answer: when the machine is not running there is no
  // vibration evidence, so no health claim can be made in either direction.
  // This asserts nothing bad about the machine -- see the callers, which keep
  // alarm_code=0/alarm_level=NORMAL for non-RUNNING states, so a stopped motor
  // still raises no fault. It only stops asserting something GOOD that was
  // never measured.
  if (data->motor_state != 2) {
    return HEALTH_SCORE_UNKNOWN;
  }

  float vibMmS = 0.0f;
  if (!readVelocityForAlarm(&vibMmS, NULL, NULL)) {
    return HEALTH_SCORE_UNKNOWN;   // VIBRATION_UNAVAILABLE
  }
  if (!vibThresholdsConfigured()) {
    return HEALTH_SCORE_UNKNOWN;   // no valid scale exists yet
  }

  float normalized = (vibMmS - VIB_WARNING_MMS) /
                     (VIB_CRITICAL_MMS - VIB_WARNING_MMS) * 100.0f;
  return (int)max(0.0f, min(100.0f, roundf(100.0f - normalized)));
}

// ============================================================================
// [v16.5.4] Improvement 2: captureTelemetrySnapshot() -- Core 0 only
// ============================================================================
// The ONE capture point for g_telemSnapshot. Called exactly once per
// state-machine cycle from taskStateMachine(), after updateMotorStateMachine()
// (inside processRPM()) and the alarm evaluation have completely finished, so
// every field in the snapshot belongs to the same sensor sample and the same
// decision cycle. Pure data movement: every field here is either a straight
// copy of an already-known value or a call to a separately-named decision
// function (computeHealthScore()) -- no business arithmetic is inlined here.
// effectiveState is the post-update g_systemState.state (so MAINTENANCE is
// preserved), i.e. the same value consumers previously read from
// g_systemState in their own mutex take.
static void captureTelemetrySnapshot(const VibrationData_t* data, MachineState_t effectiveState) {
  TelemetrySnapshot snap;
  snap.rpm          = data->rpm;
  snap.motor_state  = data->motor_state;
  snap.alarm_level  = effectiveState;
  snap.health_score = computeHealthScore(data);
  memcpy(&snap.vib, data, sizeof(VibrationData_t));
  snap.currentEvidenceValid = g_currentEvidenceValid;  // [P4-02] pure copy, no computation
  snap.currentFilteredA     = g_currentFilteredA;       // [v16.5i] pure copy, no computation; same cycle as the line above

  if (xSemaphoreTake(mutexVibData, pdMS_TO_TICKS(10)) == pdTRUE) {
    memcpy(&g_telemSnapshot, &snap, sizeof(TelemetrySnapshot));
    xSemaphoreGive(mutexVibData);
  }
}


// ============================================================================
// RS485 CONTROL (Core 0)
// ============================================================================

// [Task 4.5 -- TEMPORARY DIAGNOSTIC ONLY, EN-ownership-hypothesis
// verification, not a permanent production feature] Logs RS485_EN_PIN
// (GPIO42) ONLY on an actual level change (requirement 1: "every
// transition"), alongside timestamp (2), current internal FifoState (3),
// current FifoPhase (4), and a directly-derived bus owner (5) -- FIFO
// whenever FifoDriver_GetPhase() == ACTIVE, otherwise MODBUS/OTHER (the only
// other caller of rs485Enable()/rs485Disable() in this codebase). Called
// from inside rs485Enable()/rs485Disable() themselves, AFTER the real
// digitalWrite() -- this is the single point every existing call site
// (taskModbusRead's gated poll, sensor-reconfig helpers, setup()) already
// funnels through, so no other code changes anywhere. Purely observational:
// reads state, writes nothing but its own tracking variable. Intended to be
// removed, along with rs485Enable()/rs485Disable()'s one added call each,
// FifoDriver_GetInternalStateNameForDiag(), and s_lastEnPinLoggedState, once
// this investigation concludes.
static void LogEnPinTransition(uint8_t newState) {
  if (newState == s_lastEnPinLoggedState) {
    return;  // no change -- not a transition, nothing to log
  }
  s_lastEnPinLoggedState = newState;
  FifoPhase phase = FifoDriver_GetPhase();
  LOGT("[EN-DIAG] GPIO42 -> %s FifoState=%s FifoPhase=%d busOwner=%s t=%lums\n",
                (newState == HIGH) ? "HIGH" : "LOW",
                FifoDriver_GetInternalStateNameForDiag(),
                static_cast<int>(phase),
                (phase == FifoPhase::ACTIVE) ? "FIFO" : "MODBUS/OTHER",
                (unsigned long)millis());
}

// [Task 5.3 -- TEMPORARY DIAGNOSTIC ONLY, auto-restart runtime-verification
// investigation, not a permanent production feature] `caller` is an
// optional, defaulted diagnostic tag identifying WHICH of this codebase's
// several call sites invoked rs485Enable()/rs485Disable() this time (Task
// 5.1/5.2 identified more than one: the Task 4.6 EN-ownership tracker, the
// normal-polling gate, the stuck-detection auto-restart path, the NaN/Inf
// guard, and setup()). Purely a string label passed through to logging --
// digitalWrite(RS485_EN_PIN, ...) itself, and every existing call site that
// does not pass this new optional argument, are byte-for-byte unchanged.
static inline void rs485Enable(const char* caller = "?") {
  digitalWrite(RS485_EN_PIN, LOW);
  LogEnPinTransition(LOW);  // [Task 4.5 -- TEMPORARY DIAGNOSTIC ONLY]
  // [Task 5.3 -- TEMPORARY DIAGNOSTIC ONLY] fires on EVERY call (not just
  // actual level transitions, unlike LogEnPinTransition() above) so the
  // caller identity is never lost even on a same-level repeat call.
  LOGT("[EN-CALLER] rs485Enable() caller=%s fifoOwnsBus=%d attempt=%lu t=%lums\n",
                caller, (int)FifoDriver_OwnsBus(),
                (unsigned long)FifoDriver_GetAttemptNumberForDiag(), (unsigned long)millis());
}

static inline void rs485Disable(const char* caller = "?") {
  digitalWrite(RS485_EN_PIN, HIGH);
  LogEnPinTransition(HIGH);  // [Task 4.5 -- TEMPORARY DIAGNOSTIC ONLY]
  // [Task 5.3 -- TEMPORARY DIAGNOSTIC ONLY] see rs485Enable()'s own comment.
  LOGT("[EN-CALLER] rs485Disable() caller=%s fifoOwnsBus=%d attempt=%lu t=%lums\n",
                caller, (int)FifoDriver_OwnsBus(),
                (unsigned long)FifoDriver_GetAttemptNumberForDiag(), (unsigned long)millis());
}

// ============================================================================
// VY STUCK DETECTION & AUTO-RECOVERY (Core 0)
// ============================================================================
// ?????: ???? hard reset ????? ??????????????? register VY (0x3B) = 0 ????
//         ????????? power cycle ???????? -> ??? Modbus restart ???
// ??????: ??????? Vx/Vy/Vz = 0 ????????? 5 ????? (1.25s) ????????????? > 0
//         -> ???? restart ???????????? Modbus (Unlock 0x69 -> Restart 0x00)
// ============================================================================

// --- Vy Stuck ---
static volatile uint16_t g_vyStuckCount   = 0;  // ??? Vy=0 ?????????
static volatile uint16_t g_vyRestartCount = 0;  // ???????????????? restart ????? Vy

// --- Vz Stuck (NEW) ---
static volatile uint16_t g_vzStuckCount   = 0;  // ??? Vz=0 ?????????
static volatile uint16_t g_vzRestartCount = 0;  // ???????????????? restart ????? Vz

// --- Vx Stuck (NEW) ---
static volatile uint16_t g_vxStuckCount   = 0;  // ??? Vx=0 ?????????
static volatile uint16_t g_vxRestartCount = 0;  // ???????????????? restart ????? Vx

// --- All-Zero Stuck [v16.3f] ---
// กรณี raw_x = raw_y = raw_z = 0 พร้อมกัน → othersAlive=false ทุกแกน
// per-axis check ไม่ trigger เพราะต้องการ othersAlive → ไม่มี auto-recovery
// ตรวจแยกเพื่อ handle กรณีนี้โดยเฉพาะ
static volatile uint16_t g_allZeroStuckCount   = 0;
static volatile uint16_t g_allZeroRestartCount = 0;

static uint32_t g_lastSensorRestart = 0;        // millis() ??? restart ?????? (shared cooldown)

static const uint16_t STUCK_THRESHOLD        = 5;      // 5 ????? x 250ms = 1.25 ?????? (????? 10)
static const int16_t  STUCK_MIN_RAW          = 5;      // |raw| > 5 (0.05 mm/s) ?????? "?????"
// [v16.3v] 30000 -> 15000: cooldown 30s บล็อกการกู้คืนนานเกินไปเมื่อ noise เกิดถี่
// (log: fault ครั้งที่ 2 เกิด ~8s หลัง restart แรก → โดน block → RMS ค้าง 0.00 > 90s)
// full restart ใช้เวลา ~3s reboot อยู่แล้ว การรีสตาร์ททุก 15s ไม่ทำให้เกิด storm
static const uint32_t SENSOR_RESTART_COOLDOWN = 15000; // shared cooldown ระหว่าง restart แต่ละครั้ง (15s)

/**
 * Re-configure WTVB02-485 หลัง reboot ผ่าน Modbus (v15.7)
 *
 * [PD-0005] ลำดับ config จริงที่ทำงานอยู่ (DRM ยังไม่เขียน -- ดู [PATCHED v16.3] ด้านล่าง):
 *   1. Unlock#1 (0x69=0xB588) → MODE=FreqDomain    (0x07=0x0002)
 *   2. Unlock#2 (0x69=0xB588) → SR=SENSOR_SR_PRODUCTION (0x29)
 *      → 500ms settle → one-shot read-back of REG_SAMPLE_RATE → decode via SR0-SR9 lookup
 *      + verify against SENSOR_SR_PRODUCTION (Serial only)
 *   3. Unlock#3 (0x69=0xB588) → Save               (0x00=0x0000)
 * ไม่มีการเขียน REG_DRM (0x2B) ในฟังก์ชันนี้ -- sensor ใช้ค่าที่ persist อยู่ใน NVM ของตัวมันเอง
 *
 * [PD-0006] The write value and the verify value are both SENSOR_SR_PRODUCTION
 * (currently SR4/2kHz) -- see its definition for the rate and the rationale.
 *
 * @return true  ทุก step สำเร็จ
 *         false มี step ใดล้มเหลว (log warning แต่ caller ยังนับ restart ว่า OK)
 */
// [Phase 3A] Maps a raw REG_SAMPLE_RATE value to its documented rate in Hz.
// Table is WTVB02-485 Data Sheet & User Manual V260406 Sec 6.4.12 verbatim
// (0x00 32K .. 0x09 64Hz). Returns 0 for any undocumented value, so an
// unexpected register read can never be mistaken for a valid rate.
//
// Defined HERE, not beside g_sensorSrHzVerified, because the .ino
// auto-prototype generator emits all prototypes before the file's first
// function definition -- placing this helper up with the globals would put
// those prototypes ahead of the typedefs they reference (CLAUDE.md).
static uint32_t sensorSrIndexToHz(uint16_t srIndex) {
  switch (srIndex) {
    case 0x00: return 32000u;
    case 0x01: return 16000u;
    case 0x02: return  8000u;
    case 0x03: return  4000u;
    case 0x04: return  2000u;   // SR4 -- current production
    case 0x05: return  1000u;
    case 0x06: return   512u;
    case 0x07: return   256u;
    case 0x08: return   128u;
    case 0x09: return    64u;
    default:   return     0u;   // undocumented -> provenance NOT established
  }
}

static bool reconfigSensorAfterRestart(bool sensorWasRestarted = true) {
  // [Phase 3A] INVALIDATE sample-rate provenance for the whole duration of
  // reconfiguration. Entering here means the sensor is being (re)configured
  // -- possibly after a restart that reverted its NVM -- so any previously
  // verified rate is no longer trustworthy until this run's own read-back
  // re-establishes it. Fail-closed: if this function is interrupted, fails,
  // or the read-back mismatches, provenance simply stays UNKNOWN and captures
  // taken meanwhile report srHz == 0 rather than a stale or assumed rate.
  //
  // [v16.5f] The unconditional form of this invalidation destroyed a VERIFIED
  // rate on a path that could never re-establish it. Observed in production
  // 2026-08-24T12:50-12:51Z: five consecutive ERR_NO_RESPONSE captures ->
  // sensor restart/reconfig -> the very next capture reported error=NONE and
  // sample_count=1024 but srHz=0, and stayed at srHz=0 for the remaining
  // 38 minutes. Downstream, the producer gate ("result.srHz != 0u") then
  // rejected every snapshot: 732 captures at srHz=0 produced 0 accel_rms
  // hand-offs, versus 420 of 425 at srHz=2000. The velocity carrier was
  // never refreshed again, so velocity_data_valid stayed false.
  //
  // The rate itself was never wrong -- SENSOR_SR_PRODUCTION (SR4 / 2 kHz) is
  // correct and unchanged. What was wrong is that a reconfiguration which does
  // NOT restart the sensor still forgot the rate it had already verified.
  //
  // sensorWasRestarted == true  (default, boot + full-restart callers):
  //     behaviour is byte-for-byte the previous behaviour -- fail-closed,
  //     because a restarted sensor may genuinely have reverted its NVM.
  // sensorWasRestarted == false (quick-reconfig, sensor NOT restarted):
  //     keep the previously verified rate until this run produces POSITIVE
  //     evidence that it is wrong or unknowable. See the three invalidation
  //     sites in the SR read-back block below. Provenance is still never
  //     fabricated: g_sensorSrHzVerified is only ever assigned from a real
  //     decoded read-back, exactly as before.
  if (sensorWasRestarted) {
    g_sensorSrIndexVerified = FIFO_SR_INDEX_UNKNOWN;
    g_sensorSrHzVerified    = 0;
  }

  // [PATCHED v16.1] reconfigSensorAfterRestart()
  //
  // การเปลี่ยนแปลงจาก v16.0:
  //   1. MODE เปลี่ยนจาก SENSOR_MODE_TDLF (0x0000) → SENSOR_MODE_FREQ (0x0002)
  //      ยืนยันจาก CF test log: MODE=0x02 เท่านั้นที่ให้ CF/VRMS มีค่า
  //      MODE=0x00 และ 0x01 → CF/VRMS = 0x0000 ทั้งหมด (ไม่มีประโยชน์สำหรับ CM)
  //
  //   2. เพิ่ม abort-on-unlock-fail guard ทุก step
  //      เดิม: unlock FAIL แต่ยัง write ต่อ → register อาจถูกเขียนโดยไม่ผ่าน unlock จริง
  //      ใหม่: unlock FAIL → skip write step นั้น + set allOk=false + log ชัดเจน
  //
  //   3. ลำดับ steps ตาม WTVB02 manual §6.2 และ §6.4.1 ที่ทำงานอยู่จริง (หลัง [PD-0006] ใช้ SENSOR_SR_PRODUCTION):
  //      Step 1: Unlock → MODE=0x02(0x07=0x0002)  [FreqDomain: ให้ CF/VRMS/Kurtosis]
  //      Step 2: Unlock → SR=SENSOR_SR_PRODUCTION (0x29) → 500ms → Read-back + decode + verify (Serial only)
  //      Step 3: Unlock → Save     (0x00=0x0000)
  //      (DRM=0x02 (0x2B) ไม่ได้เขียนในฟังก์ชันนี้)

  uint8_t result;
  bool allOk = true;

  Serial.println("[SENSOR-CFG] ========================================");
  Serial.println("[SENSOR-CFG] Re-configuring sensor after restart...");
  // [PATCHED v16.3, superseded by PD-0002] Unlock แยกทุก step
  // -----------------------------------------------------------------------
  // จากการทดสอบ (v16.3):
  //   - Single unlock: SR=OK, MODE=FAIL, Save=OK → MODE ต้องการ unlock ใหม่
  //   - DRM ถูกลบออกเพราะ FAIL ทุกครั้งและไม่เกี่ยวกับ CF/VRMS
  //
  // [PD-0006] เขียน SENSOR_SR_PRODUCTION (ปัจจุบัน = SENSOR_SR_2K, 0x0004)
  // -- ค่าเดียวที่ทั้ง write และ verify ใช้ร่วมกัน (ดูนิยามของ SENSOR_SR_PRODUCTION)
  // Read-back placed AFTER the 500ms settle delay (ไม่ใช่ทันทีหลัง write) -- หลักฐานเดียวที่มี (v16.3b)
  // คือ settle delay มีไว้เพื่อความน่าเชื่อถือของ transaction ถัดไป ซึ่ง read ก็นับเป็น transaction
  //
  // Sequence: Unlock → MODE=0x02 → Unlock → SR=0x0006 → 500ms → Read SR → Unlock → Save
  // -----------------------------------------------------------------------
  Serial.println("[SENSOR-CFG] [v16.3] Unlock-per-step: MODE then Save");

  // ------------------------------------------------------------------
  // Step 1: Unlock + MODE = 0x02 (FreqDomain)
  // ------------------------------------------------------------------
  Serial.printf("[SENSOR-CFG] [Unlock for MODE]...");
  result = modbus.writeSingleRegister(REG_UNLOCK, SENSOR_UNLOCK_KEY);
  if (result != modbus.ku8MBSuccess) {
    Serial.printf(" x FAILED (err=%d) -- ABORT\n", result);
    Serial.println("[SENSOR-CFG] ========================================");
    return false;
  }
  Serial.println(" + OK");
  vTaskDelay(pdMS_TO_TICKS(50));

  Serial.printf("[SENSOR-CFG] MODE=FreqDomain (0x%02X=0x%04X)...", REG_MODE, SENSOR_MODE_FREQ);
  result = modbus.writeSingleRegister(REG_MODE, SENSOR_MODE_FREQ);
  if (result != modbus.ku8MBSuccess) {
    Serial.printf(" x FAILED (err=%d)\n", result);
    allOk = false;
  } else {
    Serial.println(" + OK");
  }
  vTaskDelay(pdMS_TO_TICKS(500));  // [v16.3b] 100→500ms: sensor ต้องการเวลา settle หลัง MODE write

  // ------------------------------------------------------------------
  // Step 2: Unlock + SR = SENSOR_SR_PRODUCTION
  // [PD-0006] The rate itself is chosen at SENSOR_SR_PRODUCTION's definition,
  // not here -- this block writes and verifies whatever that resolves to.
  // Includes one-shot read-back + SR0-SR9 lookup decode + verify against
  // SENSOR_SR_PRODUCTION, Serial only (no retry, no MQTT, no struct, no analytics).
  // ------------------------------------------------------------------
  Serial.printf("[SENSOR-CFG] [Unlock for SR]...");
  result = modbus.writeSingleRegister(REG_UNLOCK, SENSOR_UNLOCK_KEY);
  if (result != modbus.ku8MBSuccess) {
    Serial.printf(" x FAILED (err=%d) -- SKIP SR\n", result);
    allOk = false;
  } else {
    Serial.println(" + OK");
    vTaskDelay(pdMS_TO_TICKS(50));

    // [PD-0006] Label no longer hard-codes a rate name -- the raw value below
    // is the authority, and the decoded name is printed by the read-back.
    Serial.printf("[SENSOR-CFG] SR write (0x%02X=0x%04X)...", REG_SAMPLE_RATE, SENSOR_SR_PRODUCTION);
    result = modbus.writeSingleRegister(REG_SAMPLE_RATE, SENSOR_SR_PRODUCTION);
    if (result != modbus.ku8MBSuccess) {
      Serial.printf(" x FAILED (err=%d)\n", result);
      allOk = false;
    } else {
      Serial.println(" + OK");
    }
    vTaskDelay(pdMS_TO_TICKS(500));

    result = modbus.readHoldingRegisters(REG_SAMPLE_RATE, 1);
    if (result == modbus.ku8MBSuccess) {
      uint16_t srReadback = modbus.getResponseBuffer(0);
      const char* srLabel;
      switch (srReadback) {
        case 0x00: srLabel = "SR0 (32 kHz)"; break;
        case 0x01: srLabel = "SR1 (16 kHz)"; break;
        case 0x02: srLabel = "SR2 (8 kHz)";  break;
        case 0x03: srLabel = "SR3 (4 kHz)";  break;
        case 0x04: srLabel = "SR4 (2 kHz)";  break;
        case 0x05: srLabel = "SR5 (1 kHz)";  break;
        case 0x06: srLabel = "SR6 (512 Hz)"; break;
        case 0x07: srLabel = "SR7 (256 Hz)"; break;
        case 0x08: srLabel = "SR8 (128 Hz)"; break;
        case 0x09: srLabel = "SR9 (64 Hz)";  break;
        default:   srLabel = NULL;           break;
      }
      if (srLabel != NULL) {
        Serial.printf("[SR] Readback = %s\n", srLabel);
      } else {
        Serial.printf("[SR] Unexpected value = 0x%04X\n", srReadback);
        // [v16.5f] POSITIVE EVIDENCE #1: the sensor answered with an
        // undocumented SR index, so the running rate is genuinely unknown.
        // Invalidate regardless of sensorWasRestarted. Idempotent on the
        // restarted path, where entry already cleared these.
        g_sensorSrIndexVerified = FIFO_SR_INDEX_UNKNOWN;
        g_sensorSrHzVerified    = 0;
      }
      // [PD-0005] Transaction success alone does not prove the sensor is
      // actually running at the intended rate -- compare the decoded value
      // against the intended rate explicitly. A mismatch here (e.g. sensor
      // silently rejected the write, or reverted to a stale NVM value) must
      // fail configuration the same way a transaction error already does,
      // not just log a label and continue.
      // [PD-0006] Verifies against SENSOR_SR_PRODUCTION -- the same constant
      // the write above used, so this check can never fail-closed against a
      // rate the firmware did not actually request.
      // [Phase 3A] Latch VERIFIED provenance. Deliberately placed so it runs
      // only when the read-back decoded to a documented SR index (srLabel !=
      // NULL) AND equals the intended rate -- i.e. exactly the same condition
      // that lets configuration be declared OK below. A mismatch or an
      // undocumented value leaves the globals at UNKNOWN/0, which downstream
      // must treat as "no provenance" rather than assuming 2 kHz.
      if (srLabel != NULL && srReadback == SENSOR_SR_PRODUCTION) {
        uint32_t srHz = sensorSrIndexToHz(srReadback);
        if (srHz != 0u) {
          g_sensorSrIndexVerified = srReadback;
          g_sensorSrHzVerified    = srHz;
        }
      }

      if (srReadback != SENSOR_SR_PRODUCTION) {
        Serial.printf("[SR] MISMATCH: readback=0x%04X expected=0x%04X (SENSOR_SR_PRODUCTION) -- SR config FAILED\n",
                      srReadback, SENSOR_SR_PRODUCTION);
        allOk = false;
        // [v16.5f] POSITIVE EVIDENCE #2: the sensor is demonstrably NOT at the
        // intended rate, so any previously verified rate is now false.
        g_sensorSrIndexVerified = FIFO_SR_INDEX_UNKNOWN;
        g_sensorSrHzVerified    = 0;
      }
    } else {
      Serial.println("[SR] Readback FAILED");
      allOk = false;
      // [v16.5f] POSITIVE EVIDENCE #3: the SR write was attempted above but the
      // read-back did not complete, so the running rate cannot be confirmed.
      // Fail closed here even on the quick-reconfig path.
      g_sensorSrIndexVerified = FIFO_SR_INDEX_UNKNOWN;
      g_sensorSrHzVerified    = 0;
    }
  }

  // ------------------------------------------------------------------
  // Step 3: Unlock + Save config to NVM
  // SENSOR_CMD_SAVE = 0x0000 ตาม WTVB02 manual §6.4.1
  // ------------------------------------------------------------------
  Serial.printf("[SENSOR-CFG] [Unlock for Save]...");
  result = modbus.writeSingleRegister(REG_UNLOCK, SENSOR_UNLOCK_KEY);
  if (result != modbus.ku8MBSuccess) {
    Serial.printf(" x FAILED (err=%d) -- SKIP Save\n", result);
    allOk = false;
  } else {
    Serial.println(" + OK");
    vTaskDelay(pdMS_TO_TICKS(50));

    Serial.printf("[SENSOR-CFG] Save (0x%02X=0x%04X)...", REG_CMD, SENSOR_CMD_SAVE);
    result = modbus.writeSingleRegister(REG_CMD, SENSOR_CMD_SAVE);
    if (result != modbus.ku8MBSuccess) {
      Serial.printf(" x FAILED (err=%d) -- config NOT persisted to NVM!\n", result);
      allOk = false;
    } else {
      Serial.println(" + OK");
    }
  }
  vTaskDelay(pdMS_TO_TICKS(300));

  if (allOk) {
    Serial.println("[SENSOR-CFG] All config steps OK -- MODE=FreqDomain(0x02) saved (SR/DRM not written by this function)");
  } else {
    Serial.println("[SENSOR-CFG] WARNING: Some config steps FAILED -- sensor may not output CF/VRMS");
  }
  Serial.println("[SENSOR-CFG] ========================================");

  return allOk;
}

/**
 * Restart เซนเซอร์ WTVB02-485 ผ่าน Modbus
 * คู่มือ: Unlock (reg 0x69 = 0xB588) -> Restart (reg 0x00 = 0x00FF)
 * เรียกจาก taskModbusRead เท่านั้น (Core 0) -- ไม่ต้องการ mutex เพิ่ม
 * rs485Enable() จัดการโดย ModbusMaster library อยู่แล้ว
 *
 * @param axisLabel  แกนที่ trigger stuck ("Vx", "Vy", หรือ "Vz") สำหรับ log
 */
static bool restartSensorViaModbus(const char* axisLabel) {
  uint32_t now = millis();

  // Cooldown check -- ป้องกัน restart loop วนซ้ำเร็วเกินไป (shared across all axes)
  if ((now - g_lastSensorRestart) < SENSOR_RESTART_COOLDOWN && g_lastSensorRestart != 0) {
    Serial.printf("[SENSOR] %s stuck -- Cooldown active (%lu s remaining)\n",
                  axisLabel,
                  (SENSOR_RESTART_COOLDOWN - (now - g_lastSensorRestart)) / 1000);
    return false;
  }

  Serial.println("[SENSOR] ========================================");
  Serial.printf("[SENSOR] ! %s STUCK -> Auto-restarting sensor...\n", axisLabel);
  Serial.println("[SENSOR] ========================================");

  // Step 1: Unlock (จำเป็นก่อน write คำสั่ง restart)
  uint8_t result = modbus.writeSingleRegister(REG_UNLOCK, SENSOR_UNLOCK_KEY);
  if (result != modbus.ku8MBSuccess) {
    Serial.printf("[SENSOR] x Unlock FAILED (err=%d)\n", result);
    return false;
  }
  Serial.printf("[SENSOR] + Unlock OK (reg 0x%02X = 0x%04X)\n", REG_UNLOCK, SENSOR_UNLOCK_KEY);
  vTaskDelay(pdMS_TO_TICKS(100));  // รอให้ unlock มีผลก่อนส่ง restart

  // Step 2: Restart
  // [PATCHED v16.2] WTVB02 reboot ตัวเองทันทีหลังได้รับ restart command
  // โดยไม่ส่ง Modbus response กลับ → ModbusMaster จะ return error (timeout)
  // พฤติกรรมนี้ถูกต้อง ไม่ใช่ failure -- ตรวจสอบว่า sensor alive หลัง 3s แทน
  result = modbus.writeSingleRegister(REG_CMD, 0x00FF);
  if (result != modbus.ku8MBSuccess) {
    // error ที่นี่ปกติมาก -- sensor reboot ก่อน ACK response
    // ถือเป็น OK และรอ sensor boot ขึ้นมาใหม่
    Serial.printf("[SENSOR] Restart sent (no ACK expected -- sensor rebooting) err=%d\n", result);
  } else {
    Serial.printf("[SENSOR] + Restart command sent (reg 0x%02X = 0x00FF)\n", REG_CMD);
  }
  Serial.println("[SENSOR] Waiting 3 seconds for sensor reboot...");

  // รอ sensor reboot -- คู่มือระบุ ~3 วินาที
  vTaskDelay(pdMS_TO_TICKS(3000));

  // Step 3: Re-config SR + MODE + Save (v15.7)
  // ไม่ abort ถ้า config บางขั้นล้มเหลว -- sensor อาจยังทำงานได้ด้วย default
  reconfigSensorAfterRestart();

  g_lastSensorRestart = millis();
  // reset stuck counters หลังจาก restart และ config เสร็จสมบูรณ์
  g_vxStuckCount = 0;
  g_vyStuckCount = 0;
  g_vzStuckCount = 0;
  g_allZeroStuckCount = 0;

  Serial.printf("[SENSOR] + Sensor restart complete (triggered by %s), resuming reads\n", axisLabel);
  Serial.println("[SENSOR] ========================================");

  return true;
}

// ============================================================================
// TLS SETUP -- parse certs ONCE into mbedTLS structures (not on every connect)
// ============================================================================
void setupTLS() {
  Serial.println("[TLS] Parsing mTLS certificates into ESP32 mbedTLS...");
  gsmClient.setCACert(root_ca);
  gsmClient.setCertificate(client_crt);
  gsmClient.setPrivateKey(client_key);
  if (!gsmClient.applyCredentials()) {
    Serial.println("[TLS] WARNING: Certificate load failed!");
  }
}

// ============================================================================
// MODEM CONTROL FUNCTIONS
// ============================================================================

void modemPowerOn() {
  Serial.println("[Modem] Starting power on sequence...");

  digitalWrite(MODEM_POWER_ON, LOW);
  digitalWrite(MODEM_RESET_PIN, LOW);
  delay(2000);

  digitalWrite(MODEM_POWER_ON, HIGH);
  delay(100);
  digitalWrite(MODEM_RESET_PIN, HIGH);
  delay(1200);
  digitalWrite(MODEM_RESET_PIN, LOW);
  delay(5000);

  Serial.println("[Modem] Power sequence completed");
}

bool checkModemResponse(int timeoutMs = 10000) {
  unsigned long startTime = millis();
  SerialAT.flush();

  while (millis() - startTime < timeoutMs) {
    if (modem.testAT()) {
      Serial.println("[Modem] Modem responded");
      return true;
    }
    delay(500);
    Serial.print(".");
  }
  Serial.println("\n[Modem] No response from modem");
  return false;
}

bool modemInit() {
  Serial.println("[Modem] Starting initialization...");
  g_network.modemState = MODEM_STATE_INITIALIZING;

  // Initialize serial for modem
  SerialAT.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX);
  delay(1000);
  SerialAT.flush();

  // Setup GPIO for modem control
  pinMode(MODEM_RESET_PIN, OUTPUT);
  pinMode(MODEM_POWER_ON, OUTPUT);

  bool modemInitialized = false;
  int modemRetryCount = 0;
  const int MAX_MODEM_RETRIES = 3;

  while (!modemInitialized && modemRetryCount < MAX_MODEM_RETRIES) {
    modemRetryCount++;
    Serial.printf("[Modem] Initialization attempt %d/%d\n", modemRetryCount, MAX_MODEM_RETRIES);
    // [v16.6d] FIX-WDT: modemPowerOn()+checkModemResponse() may take ~28s/attempt;
    // reset once per retry iteration so no single attempt runs unfed.
    esp_task_wdt_reset();

    // Power on modem
    modemPowerOn();
    delay(3000);

    // Check modem response
    if (checkModemResponse(15000)) {
      if (modem.init()) {
        modemInitialized = true;
        Serial.println("[Modem] Initialized successfully");

        // Get modem info
        String modemInfo = modem.getModemInfo();
        Serial.printf("[Modem] Info: %s\n", modemInfo.c_str());

        // Get IMEI
        String imei = modem.getIMEI();
        strncpy(g_network.imei, imei.c_str(), sizeof(g_network.imei) - 1);
        Serial.printf("[Modem] IMEI: %s\n", g_network.imei);

        g_network.modemReady = true;
        // TLS certs are loaded into ESP32 mbedTLS via setupTLS() in taskNetwork init.
        // No modem-side SSL configuration needed.
      } else {
        Serial.println("[Modem] Library initialization failed");
      }
    }

    if (!modemInitialized) {
      Serial.println("[Modem] Retrying...");
      delay(2000);
    }
  }

  if (!modemInitialized) {
    Serial.println("[Modem] Init failed after all retries!");
    g_network.modemState = MODEM_STATE_ERROR;
    return false;
  }

  return true;
}

bool modemConnectGPRS() {
  if (!g_network.modemReady) {
    return false;
  }

  Serial.println("[Modem] Waiting for network registration...");
  g_network.modemState = MODEM_STATE_SEARCHING;

  // [R-1] FIX-WDT AT THE BLOCKING CALLEE.
  //
  // waitForNetwork(30000L) below blocks for the FULL 30 s whenever there is no
  // usable RF -- and the Network4G task watchdog timeout is also 30 s
  // (esp_task_wdt_config_t{.timeout_ms = 30000}). A blocking call exactly equal
  // to the watchdog budget, with no feed inside it, fires the watchdog every
  // time rather than occasionally: this is deterministic, not a race.
  //
  // Reproduced on hardware at ~4 m 20 s of RF loss on BOTH the M1B-5 and M1B-6
  // binaries (same Network4G/CPU1 signature, same rst:0xc), which is what
  // established it as pre-existing and unrelated to the M1B chain. The 4.3 min
  // figure is just when the 30/60/120 s MQTT backoff ladder reaches its third
  // consecutive failure and calls modemConnectGPRS() from the reinit branch.
  //
  // v16.6d already identified this exact hazard and bracketed the BOOT call
  // site (see the "[v16.6d] FIX-WDT: modemConnectGPRS() -> waitForNetwork(30000L)
  // may block up to 30s" comment beside the boot call). The reinit call site
  // reached from taskNetwork's "3 consecutive MQTT failures -- reinit modem"
  // branch was never given the same bracket. Feeding here, inside the callee,
  // covers EVERY call site including any added later -- which is why this is
  // fixed at the callee rather than by bracketing a second caller.
  //
  // A watchdog feed has no side effects: it does not touch the modem, the
  // network state machine, the backoff ladder, or any timing.
  // [R-1B] CHUNKED, WATCHDOG-FED REGISTRATION WAIT.
  //
  // R-1 fed immediately before and after waitForNetwork(30000L) and still
  // WDT'd -- three times on hardware, at 30.007 s after the "Waiting for
  // network registration..." print. That is the whole point: the TWDT timeout
  // is ALSO 30 s, so a single call whose blocking duration EQUALS the entire
  // watchdog budget cannot be rescued by feeding outside it. Feeding around
  // the call was never going to work; the call itself has to be split.
  //
  // Splitting is safe because the library implementation is stateless --
  // TinyGSM TinyGsmModem.tpp:774-782 waitForNetworkImpl():
  //     for (uint32_t start = millis(); millis() - start < timeout_ms;) {
  //       if (thisModem().isNetworkConnected()) { return true; }
  //       delay(250);
  //     }
  //     return false;
  // `start` is a local initialised on every entry and each iteration issues an
  // independent AT+CGREG? (SIM7600 isNetworkConnectedImpl, TinyGsmClientSIM7600.h
  // :243). There is no session, handle or accumulated timer, so six 5 s calls
  // are semantically identical to one 30 s call -- they merely cost ~5 extra
  // CGREG polls at the chunk seams, on a link already polling every 250 ms.
  // SIM7600 does not override waitForNetwork (only TinyGsmClientXBee.h:802 does),
  // so the template above is what actually runs here.
  //
  // Result: max unfed block 5 s against a 30 s watchdog (6x margin instead of
  // R-1's 1.0x equality), total budget still 30 s, and early exit via the bool
  // return value that the previous code discarded.
  const uint32_t R1B_TOTAL_MS = 30000UL;
  const uint32_t R1B_CHUNK_MS = 5000UL;
  bool r1bRegistered = false;

  for (uint32_t waited = 0;
       waited < R1B_TOTAL_MS;
       waited += R1B_CHUNK_MS) {

    esp_task_wdt_reset();

    if (modem.waitForNetwork(R1B_CHUNK_MS)) {
      r1bRegistered = true;
      break;
    }

    esp_task_wdt_reset();
  }

  // Deliberately unused: the pre-existing isNetworkConnected() check below
  // remains the single authority on registration, so downstream behaviour is
  // byte-for-byte the same decision it always made. r1bRegistered exists only
  // to terminate the loop early.
  (void)r1bRegistered;

  if (modem.isNetworkConnected()) {
    Serial.println("[Modem] Network registered!");
    g_network.modemState = MODEM_STATE_REGISTERED;

    // Get operator name
    String op = modem.getOperator();
    strncpy(g_network.operatorName, op.c_str(), sizeof(g_network.operatorName) - 1);
    Serial.printf("[Modem] Operator: %s\n", g_network.operatorName);

    // Get signal quality
    g_network.signalQuality = modem.getSignalQuality();
    g_network.signalPercent = map(g_network.signalQuality, 0, 31, 0, 100);
    Serial.printf("[Modem] Signal: %d (%d%%)\n", g_network.signalQuality, g_network.signalPercent);

    // Connect GPRS
    Serial.printf("[Modem] Connecting to GPRS APN: %s\n", APN);
    g_network.modemState = MODEM_STATE_GPRS_CONNECTING;

    // [R-1] gprsConnect() performs PDP activation over AT and can block for
    // several seconds on a marginal link. Feeding immediately after it keeps
    // the remainder of this function (operator/IP queries, and the caller's
    // subsequent work) inside a fresh 30 s window rather than whatever was
    // left after registration.
    const bool r1GprsOk = modem.gprsConnect(g_cfgApn, GPRS_USER, GPRS_PASS);
    esp_task_wdt_reset();
    if (r1GprsOk) {
      Serial.println("[Modem] GPRS connected!");
      g_network.gprsConnected = true;
      g_network.modemState = MODEM_STATE_GPRS_CONNECTED;

      // Print IP address
      String ip = modem.localIP().toString();
      Serial.printf("[Modem] IP: %s\n", ip.c_str());

      return true;
    } else {
      Serial.println("[Modem] GPRS connection failed!");
      g_network.modemState = MODEM_STATE_ERROR;
      return false;
    }
  } else {
    Serial.println("[Modem] Network registration failed!");
    g_network.modemState = MODEM_STATE_ERROR;
    return false;
  }
}

void updateSignalQuality() {
  if (g_network.modemReady) {
    g_network.signalQuality = modem.getSignalQuality();
    if (g_network.signalQuality != 99) {
      g_network.signalPercent = map(g_network.signalQuality, 0, 31, 0, 100);
    } else {
      g_network.signalPercent = 0;
    }
  }
}

// ============================================================================
// NTP / NETWORK TIME SYNCHRONIZATION
// ============================================================================
// The SIMCom A7670 provides GSM network time via AT+CCLK?.
// TinyGSM wraps this as modem.getGSMDateTime().
// We parse it, compare with DS3231, and correct if needed.
// ============================================================================

/**
 * Enable automatic network time update on the modem.
 * AT+CTZU=1 enables auto-update of RTC from network.
 * AT+CLTS=1 enables getting local timestamp.
 * Should be called once after modem.init().
 */
void modemEnableNetworkTime() {
  // Enable network time auto-update (CTZU) and local timestamp (CLTS)
  SerialAT.println("AT+CTZU=1");
  delay(300);
  // Drain response
  while (SerialAT.available()) SerialAT.read();

  SerialAT.println("AT+CLTS=1");
  delay(300);
  while (SerialAT.available()) SerialAT.read();

  // Some modems need AT+COPS? or network re-registration to take effect
  // Give the modem a moment to receive network time
  Serial.println("[NTP] Network time auto-update enabled (CTZU=1, CLTS=1)");
}

/**
 * Parse GSM date-time string from modem.
 * Formats observed from SIMCom A7670:
 *   "24/12/25,14:30:00+28"   (YY/MM/DD,HH:MM:SS+/-TZ_quarters)
 *   "2024/12/25,14:30:00+28"
 * TZ is in quarter-hours from UTC (e.g. +28 = +7h = UTC+7).
 * Returns true if parsing succeeded and fills 'dt' with UTC time.
 */
bool parseGSMDateTime(const String& gsmTime, DateTime& dt) {
  // Expect at least "YY/MM/DD,HH:MM:SS"
  if (gsmTime.length() < 17) {
    Serial.printf("[NTP] GSM time too short: '%s'\n", gsmTime.c_str());
    return false;
  }

  // Find the date/time delimiters to handle 2-digit or 4-digit year
  int slash1 = gsmTime.indexOf('/');
  int slash2 = gsmTime.indexOf('/', slash1 + 1);
  int comma = gsmTime.indexOf(',');
  int colon1 = gsmTime.indexOf(':');
  int colon2 = gsmTime.indexOf(':', colon1 + 1);

  if (slash1 < 0 || slash2 < 0 || comma < 0 || colon1 < 0 || colon2 < 0) {
    Serial.printf("[NTP] Cannot parse GSM time: '%s'\n", gsmTime.c_str());
    return false;
  }

  int year = gsmTime.substring(0, slash1).toInt();
  int month = gsmTime.substring(slash1 + 1, slash2).toInt();
  int day = gsmTime.substring(slash2 + 1, comma).toInt();
  int hour = gsmTime.substring(comma + 1, colon1).toInt();
  int minute = gsmTime.substring(colon1 + 1, colon2).toInt();

  // Seconds may be followed by +/-TZ
  String secPart = gsmTime.substring(colon2 + 1);
  int second = 0;
  int tzQuarters = 0;

  int plusIdx = secPart.indexOf('+');
  int minusIdx = secPart.indexOf('-');
  int tzIdx = (plusIdx >= 0) ? plusIdx : minusIdx;

  if (tzIdx >= 0) {
    second = secPart.substring(0, tzIdx).toInt();
    tzQuarters = secPart.substring(tzIdx).toInt();  // includes sign
  } else {
    second = secPart.toInt();
  }

  // Handle 2-digit year. SIMCom modems always send 2-digit years (e.g.
  // "24" for 2024), so "+2000" is correct for real dates. The problem is
  // epoch-0 placeholder times like "70/01/01" (meaning 1970, not 2070)
  // and "80/01/06" (meaning 1980, not 2080) -- after "+2000" these become
  // 2070/2080. These are caught by the explicit string-prefix check in
  // syncRTCFromModem() AND by the tightened upper-bound sanity check below
  // (MAX_VALID_YEAR), which rejects any far-future placeholder year even
  // if the string-prefix filter is bypassed.
  if (year < 100) year += 2000;

  // Sanity check. MAX_VALID_YEAR is deliberately close to "now" so that
  // epoch-0 placeholders translated as 2070/2080 are rejected.
  if (year < 2024 || year > MAX_VALID_YEAR || month < 1 || month > 12 || day < 1 || day > 31 || hour > 23 || minute > 59 || second > 59) {
    Serial.printf("[NTP] Invalid GSM time values: %04d-%02d-%02d %02d:%02d:%02d\n",
                  year, month, day, hour, minute, second);
    return false;
  }

  // Convert local time to UTC by subtracting timezone offset
  // tzQuarters is in 15-minute increments (e.g. +28 = +7 hours)
  DateTime localTime(year, month, day, hour, minute, second);
  uint32_t unixLocal = localTime.unixtime();
  int32_t tzOffsetSec = tzQuarters * 15 * 60;
  uint32_t unixUTC = unixLocal - tzOffsetSec;

  dt = DateTime(unixUTC);

  Serial.printf("[NTP] Parsed: local=%04d-%02d-%02dT%02d:%02d:%02d (TZ=%+d quarters = %+dh)\n",
                year, month, day, hour, minute, second, tzQuarters, tzQuarters / 4);
  Serial.printf("[NTP] UTC  : %04d-%02d-%02dT%02d:%02d:%02dZ\n",
                dt.year(), dt.month(), dt.day(),
                dt.hour(), dt.minute(), dt.second());

  return true;
}

/**
 * Sync RTC from 4G modem network time.
 * Returns true if RTC was synced (or already accurate).
 */
bool syncRTCFromModem() {
  if (!g_network.modemReady) {
    Serial.println("[NTP] Modem not ready, skip sync");
    return false;
  }

  // Get GSM date-time from modem (TinyGSM wrapper for AT+CCLK?)
  String gsmDateTime = modem.getGSMDateTime(DATE_FULL);
  Serial.printf("[NTP] Raw GSM time: '%s'\n", gsmDateTime.c_str());

  if (gsmDateTime.length() < 10 ||
      gsmDateTime.startsWith("80/01/06") ||
      gsmDateTime.startsWith("70/01/01")) {
    // "80/01/06" and "70/01/01" are uninitialized/epoch-0 default times
    // reported by SIMCom modems before network time has been acquired.
    Serial.println("[NTP] Modem has no valid network time yet");
    g_timeSync.syncFailures++;
    g_timeSync.ntpReachable = false;
    return false;
  }
  Serial.println("[NTP] GSM time valid");

  // Parse the GSM time string
  DateTime networkUTC;
  if (!parseGSMDateTime(gsmDateTime, networkUTC)) {
    g_timeSync.syncFailures++;
    g_timeSync.ntpReachable = false;
    return false;
  }

  g_timeSync.ntpReachable = true;

  // Compare with current RTC
  if (g_rtcValid) {
    DateTime rtcNow; RTC_NOW_SAFE(rtcNow);  // [v16.3g]
    int32_t drift = (int32_t)networkUTC.unixtime() - (int32_t)rtcNow.unixtime();
    g_timeSync.lastDriftSec = drift;

    Serial.printf("[NTP] RTC drift: %+d seconds\n", drift);

    // [v16.3h] Implausible drift guard:
    // ถ้า drift > 1 ปี และ sync ไปแล้วภายใน 60 วินาที → RTC read corruption
    // (เกิดจาก I2C collision กับ OLED) → skip adjustment เพื่อไม่ให้ RTC เสีย
    const int32_t ONE_YEAR_SEC = 365L * 24 * 3600;
    uint32_t msSinceLastSync = millis() - g_timeSync.lastSyncMillis;
    if (g_timeSync.synced &&
        msSinceLastSync < 60000UL &&
        (drift > ONE_YEAR_SEC || drift < -ONE_YEAR_SEC)) {
      Serial.printf("[NTP] ! Drift implausible (%+d s) after recent sync (%lu ms ago) -- skip\n",
                    drift, (unsigned long)msSinceLastSync);
      // ไม่แก้ RTC แต่ mark synced ด้วย network time ที่ถูกต้อง
      g_timeSync.synced = true;
      g_timeSync.lastSyncMillis = millis();
      g_timeSync.syncCount++;
      g_timeSync.lastDriftSec = 0;  // reset drift ไม่ให้แสดงค่าผิด
      snprintf(g_timeSync.lastSyncTime, sizeof(g_timeSync.lastSyncTime),
               "%04d-%02d-%02dT%02d:%02d:%02dZ",
               networkUTC.year(), networkUTC.month(), networkUTC.day(),
               networkUTC.hour(), networkUTC.minute(), networkUTC.second());
      Serial.println("[NTP] Time synchronized");
      return true;
    }

    if (abs(drift) <= 1) {
      // RTC is accurate, no adjustment needed
      Serial.println("[NTP] RTC is accurate (drift <= 1s), no adjustment");
      g_timeSync.synced = true;
      g_timeSync.lastSyncMillis = millis();
      g_timeSync.syncCount++;
      snprintf(g_timeSync.lastSyncTime, sizeof(g_timeSync.lastSyncTime),
               "%04d-%02d-%02dT%02d:%02d:%02dZ",
               networkUTC.year(), networkUTC.month(), networkUTC.day(),
               networkUTC.hour(), networkUTC.minute(), networkUTC.second());
      Serial.println("[NTP] Time synchronized");
      return true;
    }

    if (abs(drift) > NTP_DRIFT_WARN_SEC) {
      Serial.printf("[NTP] ! WARNING: RTC drift = %+d seconds!\n", drift);
    }

    if (abs(drift) > NTP_DRIFT_MAX_SEC) {
      Serial.printf("[NTP] ! CRITICAL: RTC drift = %+d seconds -- FORCE CORRECTION\n", drift);
    }

    // Correct RTC
    Serial.printf("[NTP] Adjusting RTC: %04d-%02d-%02dT%02d:%02d:%02dZ (was off by %+ds)\n",
                  networkUTC.year(), networkUTC.month(), networkUTC.day(),
                  networkUTC.hour(), networkUTC.minute(), networkUTC.second(), drift);

    rtc.adjust(networkUTC);

  } else {
    // RTC was not valid -- set it from network time
    Serial.println("[NTP] RTC was invalid -- setting from network time");
    rtc.adjust(networkUTC);
    g_rtcValid = true;
    g_timeSync.lastDriftSec = 0;
  }

  g_timeSync.synced = true;
  g_timeSync.lastSyncMillis = millis();
  g_timeSync.syncCount++;
  snprintf(g_timeSync.lastSyncTime, sizeof(g_timeSync.lastSyncTime),
           "%04d-%02d-%02dT%02d:%02d:%02dZ",
           networkUTC.year(), networkUTC.month(), networkUTC.day(),
           networkUTC.hour(), networkUTC.minute(), networkUTC.second());

  Serial.printf("[NTP] + RTC synced successfully (total syncs: %lu)\n", g_timeSync.syncCount);
  Serial.println("[NTP] Time synchronized");
  return true;
}

/**
 * Check if NTP sync is due and perform it if needed.
 * Call this periodically from the network task.
 */
void checkAndSyncTime() {
  uint32_t now = millis();
  uint32_t interval = g_timeSync.synced ? NTP_CHECK_INTERVAL_MS : NTP_RETRY_INTERVAL_MS;

  // [v16.5j] Gate on lastCheckMillis, which advances on every attempt
  // (success or failure) -- see its declaration comment. The prior version
  // gated on lastSyncMillis == 0, which stays true forever while sync keeps
  // failing (lastSyncMillis is success-only), so the interval was never
  // actually enforced once the modem reported an invalid/epoch time: this
  // function was being called every taskNetwork() loop tick (~100ms) and
  // firing a sync attempt on every single call instead of waiting for
  // NTP_RETRY_INTERVAL_MS between attempts.
  if (g_timeSync.lastCheckMillis == 0 || (now - g_timeSync.lastCheckMillis >= interval)) {
    g_timeSync.lastCheckMillis = now;
    Serial.printf("[NTP] Time sync check (interval=%lus, synced=%s)\n",
                  interval / 1000, g_timeSync.synced ? "yes" : "no");
    if (!syncRTCFromModem()) {
      // synced is still false at this point on every failure path, so the
      // next scheduled attempt will use NTP_RETRY_INTERVAL_MS regardless of
      // which check inside syncRTCFromModem() rejected this attempt.
      Serial.printf("[NTP] Next retry in %lus\n", NTP_RETRY_INTERVAL_MS / 1000);
    }
  }
}

// ============================================================================
// CORE 0 TASKS - TIME CRITICAL OPERATIONS
// ============================================================================

// [v16.6c] Diagnostic-only: symbolic name for a ModbusMaster return code, for
// Serial logging. Does not affect control flow -- string lookup only.
static const char* modbusRcName(uint8_t rc) {
  switch (rc) {
    case ModbusMaster::ku8MBSuccess:          return "ku8MBSuccess";
    case ModbusMaster::ku8MBInvalidSlaveID:   return "ku8MBInvalidSlaveID";
    case ModbusMaster::ku8MBInvalidFunction:  return "ku8MBInvalidFunction";
    case ModbusMaster::ku8MBResponseTimedOut: return "ku8MBResponseTimedOut";
    case ModbusMaster::ku8MBInvalidCRC:       return "ku8MBInvalidCRC";
    case ModbusMaster::ku8MBIllegalFunction:      return "ku8MBIllegalFunction";
    case ModbusMaster::ku8MBIllegalDataAddress:   return "ku8MBIllegalDataAddress";
    case ModbusMaster::ku8MBIllegalDataValue:     return "ku8MBIllegalDataValue";
    case ModbusMaster::ku8MBSlaveDeviceFailure:   return "ku8MBSlaveDeviceFailure";
    default:                                  return "?";
  }
}

// [v16.6a] CTR4A01 current sensor read -- reused verbatim (same signature/body)
// from experimental/CTR4A01_SENSOR/CTR4A01_SENSOR.ino readCurrentSensor().
// Caller must already have the bus addressed to CURRENT_SENSOR_ID -- see
// readCTR4A01Current() below, which owns that addressing.
static bool readCurrentSensor(uint16_t &milliAmps) {
  // [v16.6c] diagnostic-only timing around the Modbus call -- millis() reads
  // add no delay and do not alter the call itself or its return value.
  uint32_t t0 = millis();
  uint8_t r = modbus.readInputRegisters(CT_REG_AC_CURRENT, 1);
  uint32_t elapsedMs = millis() - t0;
  bool success = (r == modbus.ku8MBSuccess);
  if (success) {
    milliAmps = modbus.getResponseBuffer(0);
    // [v16.5e] Serial output silenced for log readability -- milliAmps/
    // success return value unchanged.
    // Serial.printf("[CURRENT] raw=%u mA\n", milliAmps);  // [v16.6c] diagnostic only
  } else {
    // [v16.6c] diagnostic only -- current_read_errors/current_valid handling
    // is unchanged, still owned entirely by readCTR4A01Current() below.
    // [v16.5e] Serial output silenced for log readability -- r/elapsedMs/
    // success return value unchanged.
    // Serial.printf("[CURRENT] FAIL rc=0x%02X (%s) elapsed=%lu ms\n",
    //               r, modbusRcName(r), (unsigned long)elapsedMs);
  }
  return success;
}

// [v16.6a] Owns the CTR4A01 slave-ID switch + read + restore sequence, so
// taskModbusRead() only has to handle cadence gating and where to store the
// result -- not Modbus addressing details. Assumes RS485 is already enabled
// by the caller (shares the WTVB02 bus/rs485Enable() window, single
// acquisition source, no new UART/task/timer). Always restores
// MODBUS_SLAVE_ID before returning, whether the read succeeded or not.
static bool readCTR4A01Current(float &amps) {
  uint16_t currentMa = 0;
  modbus.begin(CURRENT_SENSOR_ID, SerialRS485);
  bool ok = readCurrentSensor(currentMa);
  modbus.begin(MODBUS_SLAVE_ID, SerialRS485);  // restore WTVB02 addressing
  vTaskDelay(pdMS_TO_TICKS(5));
  if (ok) {
    // [v16.6g] Phase-2: Measurement Layer owns calibration. rawCurrentA is
    // the CTR4A01's raw reading, converted to amps only (no CT compensation
    // yet) -- it exists solely in this scope and in the diagnostic block
    // below; nothing past this function ever sees it again.
    // compensateCurrent() applies the real CT_TURNS/CT_RATIO_* conversion --
    // its result, engineeringCurrentA, is what gets stored into current_a,
    // so every downstream consumer (EMA, motor-state threshold, MQTT current
    // field, trend buffer, analytics slope) receives engineering current
    // without ever referencing CT_TURNS/CT_RATIO_* itself.
    float rawCurrentA        = currentMa / 1000.0f;
    float engineeringCurrentA = compensateCurrent(rawCurrentA);
    amps = engineeringCurrentA;
    g_lastCurrentSampleMs = millis();  // [Commit 3] only updated on success -- drives evidence ageMs

#ifdef DEBUG_CURRENT_PATH
    // [v16.6g] Diagnostic-only, 1 Hz -- Measurement Layer half of
    // [CURRENT_DIAG]. rawCurrentA/CT_TURNS/CT_RATIO_*/scale are meaningful
    // only here, at the acquisition boundary; the decision-layer half
    // (engineeringA/threshold/signalPresent) is printed separately from
    // buildMotorStateEvidence(), which never references CT_TURNS/CT_RATIO_*.
    static uint32_t s_lastMeasDiagMs = 0;
    uint32_t nowMeasMs = millis();
    if (nowMeasMs - s_lastMeasDiagMs >= 10000) {
      s_lastMeasDiagMs = nowMeasMs;
      float scale = (rawCurrentA != 0.0f) ? (engineeringCurrentA / rawCurrentA) : 0.0f;
      LOGI("[CURRENT_DIAG]\r\n");
      LOGI("rawA=%.3f\n",              rawCurrentA);
      LOGI("engineeringA=%.3f\n",       engineeringCurrentA);
      LOGI("ctTurns=%d\n",              (int)CT_TURNS);
      LOGI("ctRatioPrimaryA=%.3f\n",    (float)CT_RATIO_PRIMARY_A);
      LOGI("ctRatioSecondaryA=%.3f\n",  (float)CT_RATIO_SECONDARY_A);
      LOGI("scale=%.3f\n",              scale);
    }
#endif
  }
  else    g_ctReadErrors++;  // [v16.6b] remote-visible failure counter -- see /vibration current_read_errors
  return ok;
}

/**
 * Task 1: Modbus RTU Communication (CORE 0, Priority 5)
 * Runs every 250ms
 * Reads sensor data via RS485 and sends to queue
 * v15.0: 3 Modbus transactions -- VEL(0x3A) + TEMP(0x40) + FREQ(0x44) + CF/K(0x47)
 */
// [Task 4.2A -- TEMPORARY DIAGNOSTIC ONLY, not a permanent production
// feature] Bus-ownership instrumentation to prove/falsify the RS485
// concurrent-access hypothesis (K-14 / err=226 cascade). Purely observational:
// derives ownership from FifoDriver_GetPhase() (already-public API) and the
// existing rs485Enable()/rs485Disable() poll window in taskModbusRead() --
// does not touch fifo_driver.cpp/fifo_session.cpp/fifo_codec.cpp, does not
// change FIFO retry/CRC/timing logic. Function body placed here, immediately
// before its only call sites in taskModbusRead(), rather than near the top
// of the file -- an earlier placement made this the file's first
// function-with-a-body, which shifted the .ino auto-prototype insertion
// point ahead of later typedefs (AnalysisReason_t, MqttOutboundTopic_t,
// TelemetrySnapshot, ...) and broke unrelated forward declarations
// (CLAUDE.md's documented auto-prototype hazard). The BusOwnerDiag enum
// itself is declared near g_fifoTransport, above, since it must be visible
// at the auto-prototype insertion point too. Intended to be removed once
// Task 4.2A concludes.
static BusOwnerDiag s_busOwnerDiag = BusOwnerDiag::NONE;
static void FifoDiag_SetBusOwner(BusOwnerDiag newOwner) {
  if (newOwner == s_busOwnerDiag) return;
  static const char* const kBusOwnerNames[] = { "NONE", "MODBUS", "FIFO" };
  LOGD("[BUS] BUS_OWNER -> %s\n", kBusOwnerNames[static_cast<int>(newOwner)]);
  s_busOwnerDiag = newOwner;
}

// [Result Consumer, Commit 2] Human-readable stringification for the FIFO
// /event payload -- same pattern as faultEventStr()/faultSeverity() above.
// `default` falls back to "UNKNOWN" rather than failing to compile, so a
// future addition to fifo_types.h's frozen enums (out of this file's
// control, per Task 6.1's own scope) degrades gracefully instead of
// breaking this build.
static const char* fifoTriggerSourceStr(FifoTriggerSource src) {
  switch (src) {
    case FifoTriggerSource::FAULT_LATCH:     return "FAULT_LATCH";
    case FifoTriggerSource::OPERATOR_BUTTON: return "OPERATOR_BUTTON";
    case FifoTriggerSource::SCHEDULED:       return "SCHEDULED";
    case FifoTriggerSource::REMOTE_ON_DEMAND: return "REMOTE_ON_DEMAND";  // [Commit 7B]
    default:                                 return "UNKNOWN";
  }
}

static const char* fifoPhaseStr(FifoPhase phase) {
  switch (phase) {
    case FifoPhase::IDLE:             return "IDLE";
    case FifoPhase::ACTIVE:           return "ACTIVE";
    case FifoPhase::RESULT_READY:     return "RESULT_READY";
    case FifoPhase::COOLDOWN:         return "COOLDOWN";
    case FifoPhase::BREAKER_DISABLED: return "BREAKER_DISABLED";
    default:                          return "UNKNOWN";
  }
}

static const char* fifoErrorStr(FifoError err) {
  switch (err) {
    case FifoError::NONE:                    return "NONE";
    case FifoError::ERR_NO_RESPONSE:         return "ERR_NO_RESPONSE";
    case FifoError::ERR_INTER_BYTE_TIMEOUT:  return "ERR_INTER_BYTE_TIMEOUT";
    case FifoError::ERR_DESYNC_LIMIT:        return "ERR_DESYNC_LIMIT";
    case FifoError::ERR_RX_OVERFLOW:         return "ERR_RX_OVERFLOW";
    case FifoError::ERR_CRC_MISMATCH:        return "ERR_CRC_MISMATCH";
    case FifoError::ERR_BAD_TYPE_BYTE:       return "ERR_BAD_TYPE_BYTE";
    case FifoError::ERR_PROGRESS_REGRESSION: return "ERR_PROGRESS_REGRESSION";
    case FifoError::ERR_PROGRESS_OVERRUN:    return "ERR_PROGRESS_OVERRUN";
    case FifoError::ERR_BUSY:                return "ERR_BUSY";
    case FifoError::ERR_NOT_PERMITTED:       return "ERR_NOT_PERMITTED";
    case FifoError::ERR_RESULT_NOT_RELEASED: return "ERR_RESULT_NOT_RELEASED";
    case FifoError::ERR_ABORTED:             return "ERR_ABORTED";
    case FifoError::ERR_RETRY_EXHAUSTED:     return "ERR_RETRY_EXHAUSTED";
    case FifoError::ERR_CIRCUIT_OPEN:        return "ERR_CIRCUIT_OPEN";
    default:                                 return "UNKNOWN";
  }
}

// [Commit 7C] MQTT-only rejection-event publisher. NOT part of the SDS
// SS9.1 result-consumer contract (invariant #3, below) -- it never calls
// FifoDriver_TryAcquireResult()/FifoDriver_ReleaseResult(), because a
// rejected request never reaches S2_ARMED and therefore never produces a
// FifoCaptureResult to acquire. There is nothing to release. Called only
// from taskModbusRead()'s broker drain block, strictly gated on
// fifoIntent.source == REMOTE_ON_DEMAND -- every other trigger source's
// rejection path (Serial log only) never reaches this function at all.
static void publishMqttRejectionEvent(const char rawTag[FIFO_TAG_MAXLEN], FifoError verdict) {
  // Same defensive null-terminate-before-treating-as-C-string pattern as
  // handleFifoCaptureCompletion()'s tagSafe, below -- rawTag has no
  // struct-level null-termination guarantee.
  char requestIdSafe[FIFO_TAG_MAXLEN + 1];
  memcpy(requestIdSafe, rawTag, FIFO_TAG_MAXLEN);
  requestIdSafe[FIFO_TAG_MAXLEN] = '\0';

  StaticJsonDocument<256> rejDoc;
  rejDoc["plant_id"]   = PLANT_ID;
  rejDoc["machine_id"] = MACHINE_ID;
  rejDoc["event"]      = "fifo_capture_rejected";
  rejDoc["request_id"] = requestIdSafe;
  rejDoc["trigger"]    = "REMOTE_ON_DEMAND";
  rejDoc["error"]      = fifoErrorStr(verdict);

  char rejBuf[300];
  size_t szRej = serializeJson(rejDoc, rejBuf, sizeof(rejBuf));

  bool enqueued = enqueueMqttOutbound(MQTT_OUTBOUND_TOPIC_EVENT, rejBuf, szRej, MQTT_QOS);
  Serial.printf("[MQTT-CMD] REJECTED (post-admission) request_id=%s error=%s enqueued=%d\n",
                requestIdSafe, fifoErrorStr(verdict), (int)enqueued);
}

#ifdef DEBUG_FIFO_DUMP
// [Phase 7A] Debug-only FIFO raw waveform egress -- the first (and, while
// this flag is undefined, only) reader of FifoCaptureResult::x/y/z anywhere
// in this firmware. Compiled out entirely when DEBUG_FIFO_DUMP is
// undefined (see its own #define site for the full contract). Caller
// (handleFifoCaptureCompletion(), below) is responsible for calling this
// only while `result` is still held (between TryAcquireResult() and
// ReleaseResult()) and only for a genuinely successful capture -- this
// function itself does not re-check result.error, matching every other
// helper in this file that trusts its caller's already-established
// precondition rather than re-deriving it.
static void DumpFifoCaptureCsv(const FifoCaptureResult& result) {
  Serial.println("Index,X,Y,Z");
  for (uint16_t i = 0; i < result.sampleCount; i++) {
    Serial.printf("%u,%d,%d,%d\n", i, result.x[i], result.y[i], result.z[i]);
  }
}
#endif  // DEBUG_FIFO_DUMP

// [ARCH-INVARIANT] docs/FIFO_TRIGGER_BROKER_INVARIANTS.md invariant #3:
// the only acquire/release site in the firmware. A second consumer would
// either be rejected (result already held) or race this one's release.
// [Result Consumer, Commit 2] SDS SS9.1's "handleFifoCaptureCompletion()" --
// the ONE function in this firmware permitted to call
// FifoDriver_TryAcquireResult() and FifoDriver_ReleaseResult() (Freeze
// Review Finding 2.1; fifo_driver.h's own documented "Intended call site:
// exactly one"). Called unconditionally, once per tick, immediately after
// FifoDriver_Service() in taskModbusRead() -- same task, same core as every
// other driver interaction, so no cross-core synchronization is needed
// around the acquire/release pair itself.
//
// Zero-copy (SDS D-10, SS9.1) in production: result.x/y/z are never read or
// dereferenced here -- the 6144-byte waveform is never copied, never
// serialized, and never published. Only the ~500 B metadata/diagnostics
// block (SS8.2) is built and enqueued. [Phase 7A] The one exception is the
// DEBUG_FIFO_DUMP build flag (undefined by default): when defined, a
// successful capture's samples are additionally dumped to Serial as CSV,
// strictly after this metadata block's own construction and strictly
// before FifoDriver_ReleaseResult() -- see the call site below. This does
// not change what gets enqueued to MQTT and does not exist in the binary
// at all when the flag is undefined.
//
// Every successful acquire is followed by a release on every path out of
// this function: one early `return` before acquisition (nothing ready --
// nothing was acquired, nothing to release) and one fall-through path after
// acquisition that always reaches FifoDriver_ReleaseResult() at the end, so
// no path can leave a result held. Holds no lock and calls nothing blocking
// between acquire and release -- enqueueMqttOutbound() is a non-blocking
// xQueueSend(...,0) -- so hold time is bounded by local JSON serialization
// only (SDS SS17.5's 30 s watchdog budget is never approached). [Phase 7A]
// When DEBUG_FIFO_DUMP is defined, hold time also includes ~1024 lines of
// Serial output (order-of-1-2s at 115200 baud) -- still nowhere near the
// 30 s watchdog budget, but disclosed here since it's a real, non-zero
// addition to hold time that only exists in that debug build.
static void handleFifoCaptureCompletion() {
  FifoCaptureResult result;
  if (!FifoDriver_TryAcquireResult(&result)) {
    return;  // nothing ready, or already held -- nothing to do this tick
  }

  // result.tag has no null-termination guarantee at the struct level (fixed
  // FIFO_TAG_MAXLEN=16 bytes) -- copy defensively before treating it as a
  // C string for JSON serialization.
  char tagSafe[FIFO_TAG_MAXLEN + 1];
  memcpy(tagSafe, result.tag, FIFO_TAG_MAXLEN);
  tagSafe[FIFO_TAG_MAXLEN] = '\0';

  StaticJsonDocument<640> evDoc;
  evDoc["plant_id"]               = PLANT_ID;
  evDoc["machine_id"]             = MACHINE_ID;
  evDoc["event"]                  = "fifo_capture";
  evDoc["capture_id"]             = result.captureId;
  evDoc["tag"]                    = tagSafe;
  // [Commit 7C] request_id -- additive, REMOTE_ON_DEMAND only. Every other
  // trigger source omits this key entirely; their /event output is
  // byte-for-byte unchanged from Commit 2 (requirement: "MQTT-triggered
  // captures only; other trigger sources must continue to behave exactly
  // as before"). Mirrors tagSafe, since the MQTT producer (Commit 7B)
  // carries request_id via the intent's tag field -- no new plumbing.
  if (result.triggerSource == FifoTriggerSource::REMOTE_ON_DEMAND) {
    evDoc["request_id"] = tagSafe;
  }
  evDoc["trigger"]                = fifoTriggerSourceStr(result.triggerSource);
  evDoc["status"]                 = fifoPhaseStr(result.status);
  evDoc["error"]                  = fifoErrorStr(result.error);
  evDoc["sample_count"]           = result.sampleCount;
  evDoc["sr_index"]               = result.srIndexAtCapture;
  evDoc["sr_hz"]                  = result.srHz;
  evDoc["t_request_ms"]           = result.tRequestMs;
  evDoc["t_complete_ms"]          = result.tCompleteMs;
  evDoc["duration_ms"]            = (result.tCompleteMs >= result.tRequestMs)
                                       ? (result.tCompleteMs - result.tRequestMs) : 0;
  evDoc["temp_c"]                 = result.tempCAtCapture;
  evDoc["motor_state"]            = result.motorStateAtCapture;
  evDoc["rpm"]                    = result.rpmAtCapture;
  evDoc["poll_count"]             = result.pollCount;
  evDoc["progress_frame_count"]   = result.progressFrameCount;
  evDoc["crc_error_count"]        = result.crcErrorCount;
  evDoc["retry_count"]            = result.retryCount;
  evDoc["last_progress_fill"]     = result.lastProgressFill;
  evDoc["desync_bytes_discarded"] = result.desyncBytesDiscarded;

  char evBuf[700];
  size_t szEvt = serializeJson(evDoc, evBuf, sizeof(evBuf));

  bool enqueued = enqueueMqttOutbound(MQTT_OUTBOUND_TOPIC_EVENT, evBuf, szEvt, MQTT_QOS);
  Serial.printf("[FIFO-RESULT] captureId=%lu trigger=%s error=%s enqueued=%d szEvt=%u\n",
                (unsigned long)result.captureId, fifoTriggerSourceStr(result.triggerSource),
                fifoErrorStr(result.error), (int)enqueued, (unsigned)szEvt);

  // [Phase 2D Option A] A capture that actually succeeded proves the fault
  // cleared, so collapse the escalated recovery backoff back to base. Without
  // this, one bad episode would leave a healthy sensor permanently on a
  // 15-minute recovery ceiling. Read-only w.r.t. the driver; touches no
  // capture, retry, breaker, cooldown or cadence state.
  if (result.error == FifoError::NONE) {
    s_fifoSuspendBackoffMs = FIFO_SUSPEND_RETRY_BASE_MS;
  }

#ifdef DEBUG_FIFO_DUMP
  // [Phase 7A] Debug-only, additive to everything above -- MQTT payload
  // (evDoc/evBuf/enqueueMqttOutbound) already fully built and enqueued by
  // this point, completely unmodified by this block. Gated on a genuinely
  // successful capture only ("dump only after a successful FIFO capture
  // has completed"); result.x/y/z are only valid while held, so this must
  // run before FifoDriver_ReleaseResult() below -- the capture itself
  // (S2..S10) is already long finished by the time control reaches here,
  // so this can never run "during acquisition".
  if (result.error == FifoError::NONE) {
    DumpFifoCaptureCsv(result);
  }
#endif  // DEBUG_FIFO_DUMP

  // [Phase 3B] Waveform hand-off to Core 1 -- the ONLY place result.x/y/z are
  // read in a production build. This is a memcpy and nothing else: no floating
  // point, no RMS, no serialization. The Phase 3B architecture forbids running
  // DSP while the FIFO result is held, and this is what makes that possible --
  // by the time Core 1 touches a single sample, the arena has been released
  // back to the driver below.
  //
  // The validity gate is applied HERE, before the copy, so an invalid capture
  // costs nothing: spec's "sample_count == 1024 && error == NONE && srHz != 0,
  // otherwise data_valid = false, do not calculate". VibAccel_ComputeRms()
  // re-checks the same conditions independently on Core 1 -- the duplication is
  // deliberate, since the module is separately host-tested and must not depend
  // on a caller having filtered its input.
  if (result.error == FifoError::NONE &&
      result.sampleCount == VIB_ACCEL_REQUIRED_SAMPLES &&
      result.srHz != 0u &&
      result.x != NULL && result.y != NULL && result.z != NULL &&
      mutexAccelSnap != NULL && queueAccelSnapshot != NULL) {

    // timeout 0: taskModbusRead is the time-critical task and must never block
    // on analytics. If Core 1 still holds the buffer, drop this waveform and
    // count it -- FIFO is best-effort (Phase 3 decision #3); the Modbus poll
    // cadence is not negotiable.
    if (xSemaphoreTake(mutexAccelSnap, 0) == pdTRUE) {
      memcpy(g_accelSnap.x, result.x, sizeof(g_accelSnap.x));
      memcpy(g_accelSnap.y, result.y, sizeof(g_accelSnap.y));
      memcpy(g_accelSnap.z, result.z, sizeof(g_accelSnap.z));
      g_accelSnap.captureId   = result.captureId;
      g_accelSnap.sampleCount = result.sampleCount;
      g_accelSnap.srHz        = result.srHz;
      // [Phase 3E] Same source, same struct, same instant as every field above
      // -- result.rpmAtCapture is the value the FIFO driver latched when this
      // capture was requested, and it is already published as `rpm` on the
      // fifo_capture event, so the two can never describe different speeds.
      g_accelSnap.rpmAtCapture = result.rpmAtCapture;
      xSemaphoreGive(mutexAccelSnap);

      // Non-blocking send on a depth-1 queue. A full queue means Core 1 has not
      // yet consumed the previous notification; the buffer above already holds
      // the newer waveform, so the stale notification still resolves to fresh
      // data and only the older waveform is lost.
      AccelSnapshotReady_t ready = { result.captureId };
      if (xQueueSend(queueAccelSnapshot, &ready, 0) != pdTRUE) {
        g_accelSnapDropped++;
      }
    } else {
      g_accelSnapDropped++;
    }
  }

  // Mandatory (SDS A-6/A-7 ownership contract): release on this, the only
  // path that reaches here, regardless of whether the enqueue above
  // succeeded -- a dropped /event is acceptable (enqueueMqttOutbound()'s own
  // "drop-newest with counter" overflow policy); a stuck driver is not.
  FifoDriver_ReleaseResult();
}

void taskModbusRead(void* parameter) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(250);  // 250ms = 4Hz
  // [v16.6h] Dual service cadence -- 250ms for normal sensor polling, 10ms
  // while FifoDriver owns the RS485 bus. The faster FIFO cadence is a
  // correctness requirement of the RAWFIFO capture, not a performance tuning
  // choice, for the reason below.
  //
  // FrameCodec_Step() performs exactly ONE bounded sub-action per call
  // (fifo_codec.cpp: ScanAnchor() stops the instant the `50 03` anchor is
  // matched and never consumes past it; ReadType() reads at most one byte and
  // never loops; the progress tail is 4 bytes), and FifoDriver_Service()
  // invokes it exactly once per iteration of this loop. Through the
  // anchor/type/progress phases the driver therefore consumes only 0-3 bytes
  // per cycle.
  //
  // The WTVB05 meanwhile streams its 6149-byte dump autonomously at 9600 baud
  // = ~240 bytes per 250ms. At a 250ms cadence that is a ~100:1
  // producer/consumer mismatch: the 2048-byte IDF RX ring buffer saturates in
  // ~8 cycles (~2s), after which the 128-byte UART hardware FIFO overflows and
  // bytes are discarded by the peripheral before the parser can ever read them
  // -- yielding either a truncated frame (ERR_INTER_BYTE_TIMEOUT) or a
  // byte-shifted frame that fails CRC. At 10ms the driver drains the ring
  // buffer every cycle and its occupancy stays under ~40 of 2048 bytes.
  //
  // [DESIGN-0005] INVARIANT: normal Modbus polling remains at 250ms. Two
  // independent, load-bearing reasons:
  //   1. STUCK_THRESHOLD is a READ COUNT, not a duration -- the stuck-axis
  //      detection it drives is calibrated to this cadence ("5 reads x 250ms
  //      = 1.25s"). A faster poll rate would trip it on the sensor's own
  //      not-yet-updated registers, causing spurious restartSensorViaModbus()
  //      calls, which set g_modbusConsecErrors != 0 and in turn block FIFO
  //      admission (see the request gate's sensorHealthy term).
  //   2. The normal-poll body cannot fit a shorter period: ~9 Modbus
  //      transactions at 9600 baud plus 10x vTaskDelay(5ms) is ~230ms.
  // Scoping the faster cadence to the FifoDriver_OwnsBus() window -- where
  // normal polling is already skipped entirely -- keeps both reasons intact.
  const TickType_t xFrequencyFifo = pdMS_TO_TICKS(10);

  VibrationData_t localData;
  int16_t raw_x, raw_y, raw_z, raw_temp;
  // [v16.3p] FIX: raw_fx/fy/fz ต้องเป็น uint16_t ไม่ใช่ int16_t
  // Frequency register เป็น unsigned เหมือน CF และ Kurtosis (ดู datasheet §6.4.13)
  // int16_t ทำให้ค่าสูง เช่น 0xFFB2 = 65458 กลายเป็น -78 → freq_z = -7.8 Hz
  uint16_t raw_fx = 0, raw_fy = 0, raw_fz = 0;
  // [Phase 1] raw_kx/ky/kz REMOVED. The CF registers are still read -- KX/KY/KZ
  // simply sit in word 2 of the same 2-register transaction and are no longer
  // taken off the wire. The reads themselves are unchanged (see below).
  uint16_t raw_cfx = 0;  // v15.0: CFX (0x47) -- unsigned per datasheet §6.4.14
  uint16_t raw_cfy = 0;  // v15.1: CFY (0x53) -- unsigned per datasheet §6.4.15
  uint16_t raw_cfz = 0;  // v15.1: CFZ (0x5F) -- unsigned per datasheet §6.4.16

  Serial.println("[CORE 0] Modbus task started");

  static uint32_t s_lastPollStart = 0;  // [v16.3y] วัด poll interval จริง
  static uint32_t s_lastCurrentSampleMs = 0;  // [v16.6a] CTR4A01 500ms cadence gate

  while (1) {
    g_sensorReads++;

    // [ADR-0006 D-1, Phase 2] PERIODIC FIFO PRODUCER -- the sole initiator.
    // Placed immediately before the drain block so an intent enqueued now is
    // admitted on THIS tick rather than waiting for the next one.
    //
    // Non-blocking by construction: one millis() comparison plus a
    // zero-timeout xQueueSend(). No delay(), no vTaskDelay(), no wait on FIFO
    // completion, no driver-state inspection (Trigger Broker invariant #4).
    //
    // Overlap is impossible without this producer needing to know: a queue
    // already holding an intent rejects the send (depth 1), and a driver that
    // is ACTIVE/COOLDOWN/DISABLED rejects the request inside
    // FifoDriver_Request(). Both outcomes are a SKIP -- this tick is dropped
    // and the next opportunity is one full period later. Nothing is deferred.
    {
      uint32_t nowPeriodicMs = millis();

      // [Phase 2D Option A] BOUNDED SELF-RECOVERY -- the only path that clears
      // s_fifoPeriodicSuspended. Runs before the cadence check so a recovered
      // producer can fire on this same tick. Exactly ONE reset attempt per
      // backoff window; if the underlying fault persists the driver re-trips
      // and the drain block re-arms the (now doubled) backoff.
      if (s_fifoPeriodicSuspended &&
          (int32_t)(nowPeriodicMs - s_fifoSuspendRetryAtMs) >= 0) {
        // FifoDriver_ResetCircuitBreaker() returns false if the breaker was
        // not actually tripped. That would mean suspend and driver state had
        // diverged; clear the latch anyway rather than deadlock the producer
        // against a breaker that is already closed.
        bool didReset = FifoDriver_ResetCircuitBreaker();
        s_fifoPeriodicSuspended = false;
        s_fifoSuspendRecoveries++;
        Serial.printf("[FIFO-PERIODIC] RESUME reason=backoff_elapsed didReset=%d "
                      "backoff_ms=%lu recoveries=%lu t=%lums\r\n",
                      (int)didReset, (unsigned long)s_fifoSuspendBackoffMs,
                      (unsigned long)s_fifoSuspendRecoveries,
                      (unsigned long)nowPeriodicMs);
      }

      if (s_fifoPeriodicNextDueMs == 0) {
        // First pass after boot: arm the schedule one full period out so the
        // very first capture cannot race sensor configuration in setup().
        s_fifoPeriodicNextDueMs = nowPeriodicMs + FIFO_PERIODIC_INTERVAL_MS;
      } else if (!s_fifoPeriodicSuspended &&
                 (int32_t)(nowPeriodicMs - s_fifoPeriodicNextDueMs) >= 0) {
        // Advance by exactly one period (phase-locked); re-base only if we are
        // already past the new deadline, so a missed tick never accumulates.
        s_fifoPeriodicNextDueMs += FIFO_PERIODIC_INTERVAL_MS;
        if ((int32_t)(nowPeriodicMs - s_fifoPeriodicNextDueMs) >= 0) {
          s_fifoPeriodicNextDueMs = nowPeriodicMs + FIFO_PERIODIC_INTERVAL_MS;
        }

        FifoTriggerIntent_t schedIntent{};
        schedIntent.source = FifoTriggerSource::SCHEDULED;
        const char* schedTag = "periodic";
        for (size_t ti = 0; ti < FIFO_TAG_MAXLEN && schedTag[ti] != '\0'; ti++) {
          schedIntent.tag[ti] = schedTag[ti];
        }
        // requirePermissive=true: only measure while the motor is RUNNING and
        // the sensor is healthy. A stopped machine has no vibration to sample.
        schedIntent.requirePermissive = true;
        // [ARCH-INVARIANT] producer -- enqueue only, never FifoDriver_Request().
        if (xQueueSend(queueFifoTrigger, &schedIntent, 0) != pdTRUE) {
          s_fifoPeriodicSkipped++;   // queue still holds an undrained intent
        } else {
          s_fifoPeriodicEnqueued++;
        }
      }
    }

    // [Broker, Commit 1] Drain at most one pending FIFO trigger intent,
    // before FifoDriver_Service() advances the driver this tick, so an
    // intent enqueued on a prior tick (or by a different task/core) is
    // turned into an admission attempt using THIS tick's freshest
    // g_motorRunState/g_modbusConsecErrors/mqttClient state -- never a
    // producer's possibly-stale snapshot (the data-freshness risk Task 6.2
    // of the Implementation Plan calls out). fifo_driver.cpp reads only
    // FifoAdmissionContext, never a .ino global directly, so this is the
    // one place that must populate it. taskModbusRead() is the ONLY
    // function in this firmware permitted to call FifoDriver_Request() --
    // this drain block is the sole call site.
    // [ARCH-INVARIANT] docs/FIFO_TRIGGER_BROKER_INVARIANTS.md invariants
    // #2/#5: the admission verdict below is decided entirely inside
    // FifoDriver_Request() (fifo_driver.cpp, unmodified) -- this block
    // never second-guesses it, never retries on rejection.
    {
      FifoTriggerIntent_t fifoIntent;
      if (xQueueReceive(queueFifoTrigger, &fifoIntent, 0) == pdPASS) {
        // [ADR-0006 D-1, Phase 2/2A] SCHEDULED-ONLY ADMISSION BOUNDARY.
        // Phase 2A removed the FAULT_LATCH / REMOTE_ON_DEMAND /
        // OPERATOR_BUTTON producers, so in a correct build nothing but
        // SCHEDULED can reach this queue and this branch is unreachable.
        // It is retained deliberately as a defence-in-depth invariant: if a
        // future producer is ever added, it cannot silently start a capture.
        // FifoTriggerSource enumerators are kept (FifoCaptureResult, the
        // /event payload and test/test_fifo_driver.cpp all still reference
        // them). The intent is consumed and dropped -- the queue slot is
        // freed this tick, and nothing is retried.
        if (fifoIntent.source != FifoTriggerSource::SCHEDULED) {
          Serial.printf("[FIFO-BROKER] REJECTED non-SCHEDULED source=%d tag=%s "
                        "-- periodic capture is the sole FIFO initiator (ADR-0006)\n",
                        static_cast<int>(fifoIntent.source), fifoIntent.tag);
        } else {
        FifoCaptureRequest fifoReq{};
        fifoReq.triggerSource = fifoIntent.source;
        for (size_t ti = 0; ti < FIFO_TAG_MAXLEN && fifoIntent.tag[ti] != '\0'; ti++) {
          fifoReq.tag[ti] = fifoIntent.tag[ti];
        }
        fifoReq.requirePermissive = fifoIntent.requirePermissive;
        fifoReq.maxRetries = 2;
        fifoReq.admissionContext.motorStable      = (g_motorRunState == MOTOR_RUNNING);
        fifoReq.admissionContext.sensorHealthy    = (g_modbusConsecErrors == 0);
        fifoReq.admissionContext.mqttReconnecting = !mqttClient.connected();
        // [Phase 3A] Bind sample-rate provenance to THIS capture, read here --
        // the same place, and for the same reason, as the admission context:
        // this is the one point that sees the freshest verified state, and the
        // driver stores what it is given without interpreting it. If the
        // read-back never verified, these carry UNKNOWN/0 and the capture is
        // reported as having no provenance -- never a silent nominal 2000 Hz.
        fifoReq.srIndexAtCapture = g_sensorSrIndexVerified;
        fifoReq.srHz             = g_sensorSrHzVerified;
        // [R-3] Machine-state provenance, bound to THIS capture at the same
        // point and for the same reason as the sample-rate pair above. Source
        // is g_telemSnapshot (the v16.5.4 atomic snapshot) so the /event
        // fifo_capture payload agrees with /vibration by construction --
        // previously these published zero-init defaults (motor_state=0 rpm=0
        // temp_c=0) because FifoCaptureResult had no producer for them.
        //
        // Single-word scalar members read lock-free, the sanctioned v16.5.4
        // pattern already used at latestRpm / snapMotorStateAnalytics below;
        // whole-struct readers use memcpy under mutexVibData instead. This
        // keeps the broker block non-blocking by construction.
        //
        // motor_state is whatever the production motor-state source already
        // produced (MOTOR_SRC_CURRENT) -- a pure copy of an existing value.
        // Nothing here selects, derives or alters motor-state detection.
        fifoReq.motorStateAtCapture = g_telemSnapshot.motor_state;
        fifoReq.rpmAtCapture        = g_telemSnapshot.rpm;
        fifoReq.tempCAtCapture      = g_telemSnapshot.vib.temperature;
        uint32_t fifoHandle = 0;
        FifoError fifoVerdict = FifoDriver_Request(&fifoReq, &fifoHandle);
        Serial.printf("[FIFO-BROKER] FifoDriver_Request() source=%d verdict=%d handle=%lu "
                      "motorStable=%d sensorHealthy=%d mqttReconnecting=%d\n",
                      static_cast<int>(fifoIntent.source),
                      static_cast<int>(fifoVerdict), (unsigned long)fifoHandle,
                      (int)fifoReq.admissionContext.motorStable,
                      (int)fifoReq.admissionContext.sensorHealthy,
                      (int)fifoReq.admissionContext.mqttReconnecting);

        // [Commit 7C] MQTT-only rejection feedback. Strictly gated on
        // source==REMOTE_ON_DEMAND -- FAULT_LATCH/OPERATOR_BUTTON/
        // COMMISSIONING/SCHEDULED rejections are entirely unaffected and
        // take exactly the path they always have (the Serial log above,
        // nothing else). Reads fifoVerdict only -- does not alter it, does
        // not retry, does not touch the queue again; admission itself was
        // already final the instant FifoDriver_Request() returned above.
        if (fifoIntent.source == FifoTriggerSource::REMOTE_ON_DEMAND &&
            fifoVerdict != FifoError::NONE) {
          publishMqttRejectionEvent(fifoIntent.tag, fifoVerdict);
        }

        // [ADR-0006 D-5, Phase 2] Latch the periodic producer off once the
        // circuit breaker has opened, satisfying "do not repeatedly enqueue
        // while disabled" WITHOUT the producer itself reading driver state.
        // Set here because the drain block is the one place that legitimately
        // sees admission verdicts.
        //
        // [Phase 2D Option A] The latch is no longer terminal: arm a bounded
        // recovery deadline, then DOUBLE the backoff (capped) so that a
        // persistent fault decays toward the ceiling instead of retrying
        // forever at a fixed rate. The producer performs the single reset
        // attempt when that deadline expires. Edge-gated: the deadline and the
        // escalation are applied once per false->true transition, never per
        // tick, so a stream of ERR_CIRCUIT_OPEN verdicts cannot inflate the
        // backoff beyond one step per actual trip.
        if (fifoVerdict == FifoError::ERR_CIRCUIT_OPEN) {
          if (!s_fifoPeriodicSuspended) {
            uint32_t nowSusMs = millis();
            s_fifoSuspendRetryAtMs = nowSusMs + s_fifoSuspendBackoffMs;
            Serial.printf("[FIFO-PERIODIC] SUSPEND reason=ERR_CIRCUIT_OPEN "
                          "breaker_state=%s driver_phase=%d retry_in_ms=%lu t=%lums\r\n",
                          "S14_DISABLED", (int)FifoDriver_GetPhase(),
                          (unsigned long)s_fifoSuspendBackoffMs,
                          (unsigned long)nowSusMs);
            // Escalate for the NEXT trip (saturating at the ceiling).
            if (s_fifoSuspendBackoffMs < FIFO_SUSPEND_RETRY_MAX_MS) {
              uint32_t nextBackoff = s_fifoSuspendBackoffMs * 2UL;
              s_fifoSuspendBackoffMs = (nextBackoff > FIFO_SUSPEND_RETRY_MAX_MS)
                                         ? FIFO_SUSPEND_RETRY_MAX_MS : nextBackoff;
            }
          }
          s_fifoPeriodicSuspended = true;
        }
        }  // [ADR-0006 D-1] end SCHEDULED-only admission branch
      }
    }

    // [Task 4.1] Advance the FIFO driver by one bounded step, unconditionally,
    // once per tick (SDS D-1/A-3) -- placed here, before any of this loop's
    // three exit paths (restart continue/NaN-guard continue/normal end),
    // specifically so it is never skipped regardless of which path a given
    // iteration takes. Not a no-op: the broker drain block immediately
    // above may have just admitted a request, so Service() genuinely
    // advances the driver -- existing bus usage below is unaffected while
    // no request is pending, exactly as before.
    FifoDriver_Service();

    // [Result Consumer, Commit 2] Immediately after Service(), every tick --
    // so a capture that just reached S11_RESULT_READY this tick is acquired,
    // published, and released before this loop iteration ends, minimizing
    // hold time. Internally a no-op (early return) whenever no result is
    // ready, matching FifoDriver_Service() call above's own unconditional
    // per-tick placement (SDS D-1/A-3 style).
    handleFifoCaptureCompletion();

    // [Task 4.6] RS485 EN ownership for FIFO transactions -- the missing
    // half of ADR-1 ("bus/EN ownership belongs entirely to
    // taskModbusRead()'s OwnsBus() boundary") that Task 4.2 left unfilled
    // (Task 4.2 only gated normal Modbus/CTR4A01 polling; nothing ever
    // asserted RS485_EN_PIN for FifoDriver's OWN transactions -- confirmed
    // by Task 4.5's direct GPIO capture: GPIO42 stayed HIGH/disabled for the
    // entire S2_ARMED..S10_DRAIN span of every attempt). Edge-triggered on
    // FifoDriver_OwnsBus() (already-public API, unchanged) so this is a
    // single rs485Enable()/rs485Disable() call each time ownership starts or
    // ends -- both functions, and RS485_EN_PIN itself, are unchanged; this
    // adds a second, unconditional call site (previously the only call
    // sites were inside the normal-polling gated block below and the
    // sensor-reconfig helpers). No effect whenever FifoDriver never owns the
    // bus (fifoOwnsBusNow stays false every tick) -- requirement 4,
    // "preserve existing normal Modbus behavior."
    {
      static bool s_fifoOwnedBusLastTick = false;
      bool fifoOwnsBusNow = FifoDriver_OwnsBus();
      if (fifoOwnsBusNow && !s_fifoOwnedBusLastTick) {
        rs485Enable("EN-TRACKER");   // FIFO acquiring the bus -- assert EN before any FIFO transmit
      } else if (!fifoOwnsBusNow && s_fifoOwnedBusLastTick) {
        rs485Disable("EN-TRACKER");  // FIFO ownership just ended -- release EN immediately
      }
      s_fifoOwnedBusLastTick = fifoOwnsBusNow;
    }

    // [Commissioning Removal] The Task 4.4 one-shot auto-trigger that used to
    // sit here (self-armed ~45s after boot, no operator action) and its
    // FifoTriggerSource::COMMISSIONING source have both been removed --
    // neither had a legitimate production caller. FAULT_LATCH,
    // OPERATOR_BUTTON, and REMOTE_ON_DEMAND remain the active producers.


    // [v16.3y] diagnostic timing
    uint32_t t_pollNow      = millis();
    uint16_t pollIntervalMs = (s_lastPollStart == 0) ? 0 : (uint16_t)(t_pollNow - s_lastPollStart);
    s_lastPollStart         = t_pollNow;
    uint32_t t_readStart    = t_pollNow;
    uint8_t  retryCount     = 0;

    bool success = true;

    // [Task 4.2] RS485 bus arbitration (A-5) -- the normal Modbus/CTR4A01
    // polling below is skipped entirely while FifoDriver owns the bus
    // (internal S2_ARMED..S10_DRAIN). `success` stays true (its
    // initialized value) and raw_x/y/z/... retain their previous
    // iteration's values on a skipped cycle -- existing polling code
    // itself, and all downstream success/stuck-detection/telemetry logic,
    // are unchanged.
    if (!FifoDriver_OwnsBus()) {
      // [Task 4.2A -- TEMPORARY DIAGNOSTIC ONLY] Requirement 3: whenever a
      // normal Modbus poll begins, log whether FifoDriver currently owns the
      // bus, BEFORE this task issues any modbus.* call. Now nested inside
      // the Task 4.2 gate -- fifoOwnsBusNow will always be false here post-
      // fix (a "poll start (fifoOwnsBus=1)" line would mean the gate failed
      // to prevent entry).
      {
        bool fifoOwnsBusNow = (FifoDriver_GetPhase() == FifoPhase::ACTIVE);
        LOGT("[MODBUS] poll start (fifoOwnsBus=%d)\n", (int)fifoOwnsBusNow);
        FifoDiag_SetBusOwner(fifoOwnsBusNow ? BusOwnerDiag::FIFO : BusOwnerDiag::MODBUS);
      }

      // Read all registers (blocking I/O, but isolated to this task)
      // v15.0: 3 Modbus transactions -- VEL + TEMP + FREQ + CF/K
      rs485Enable("NORMAL-POLL");
      vTaskDelay(pdMS_TO_TICKS(5));  // 5ms stabilization

      // Transaction 1: Velocity RMS X, Y, Z (§6.4.14-16)
      // VRMSX=0x50, VRMSY=0x5C, VRMSZ=0x68 (ไม่ consecutive -- อ่านแยก 3 ครั้ง)
      // Scaling: raw / 1000.0f → mm/s (True RMS, ไม่ต้อง × 0.7071)
      if (modbus.readHoldingRegisters(REG_VRMS_X, 1) == modbus.ku8MBSuccess) {
        raw_x = (int16_t)modbus.getResponseBuffer(0);
      } else {
        success = false; retryCount++;
      }
      vTaskDelay(pdMS_TO_TICKS(5));

      if (modbus.readHoldingRegisters(REG_VRMS_Y, 1) == modbus.ku8MBSuccess) {
        raw_y = (int16_t)modbus.getResponseBuffer(0);
      } else {
        success = false; retryCount++;
      }
      vTaskDelay(pdMS_TO_TICKS(5));

      if (modbus.readHoldingRegisters(REG_VRMS_Z, 1) == modbus.ku8MBSuccess) {
        raw_z = (int16_t)modbus.getResponseBuffer(0);
      } else {
        success = false; retryCount++;
      }
      vTaskDelay(pdMS_TO_TICKS(5));

      // Transaction 2a: Temperature (0x40)
      if (modbus.readHoldingRegisters(REG_TEMPERATURE, 1) == modbus.ku8MBSuccess) {
        raw_temp = (int16_t)modbus.getResponseBuffer(0);
      } else {
        success = false;
      }
      vTaskDelay(pdMS_TO_TICKS(5));

      // Transaction 2b: Frequency X, Y, Z (3 consecutive registers 0x44~0x46)
      if (modbus.readHoldingRegisters(REG_FREQ_X, 3) == modbus.ku8MBSuccess) {
        raw_fx = (uint16_t)modbus.getResponseBuffer(0);
        raw_fy = (uint16_t)modbus.getResponseBuffer(1);
        raw_fz = (uint16_t)modbus.getResponseBuffer(2);
      } else {
        raw_fx = 0;
        raw_fy = 0;
        raw_fz = 0;
      }
      vTaskDelay(pdMS_TO_TICKS(5));

      // Transaction 3: CFX (0x47) + KX (0x48) -- Accel Crest Factor [v15.0]
      // Optional -- ถ้า fail ปล่อยค่าเดิม (0) ไม่กระทบ success หลัก
      // [Phase 1] Still a 2-register read: the register map puts CFX and KX
      // adjacent, so the transaction width is fixed by the device, not by us.
      // Word 2 (KX) is simply no longer read out -- no bus traffic changes.
      if (modbus.readHoldingRegisters(REG_CFX, 2) == modbus.ku8MBSuccess) {
        raw_cfx = (uint16_t)modbus.getResponseBuffer(0); // v15.6 fix: unsigned per datasheet §6.4.14
      }
      vTaskDelay(pdMS_TO_TICKS(5));

      // Transaction 4: CFY (0x53) + KY (0x54) -- Y-axis [v15.1]
      if (modbus.readHoldingRegisters(REG_CFY, 2) == modbus.ku8MBSuccess) {
        raw_cfy = (uint16_t)modbus.getResponseBuffer(0); // v15.6 fix: unsigned per datasheet §6.4.15
      }
      vTaskDelay(pdMS_TO_TICKS(5));

      // Transaction 5: CFZ (0x5F) + KZ (0x60) -- Z-axis [v15.1]
      if (modbus.readHoldingRegisters(REG_CFZ, 2) == modbus.ku8MBSuccess) {
        raw_cfz = (uint16_t)modbus.getResponseBuffer(0); // v15.6 fix: unsigned per datasheet §6.4.16
      }
      vTaskDelay(pdMS_TO_TICKS(5));

      // [Phase2] Transaction 6 (Peak Velocity 0x3A~0x3C) REMOVED. Its only
      // consumer was peak_velocity_x/y/z on /sensor, dropped in Phase 1, so the
      // read had become pure bus traffic. This is the ONLY Modbus register read
      // removed by Phase 2: VRMS 0x50/0x5C/0x68 stay (stuck detection) and FREQ
      // 0x44-0x46 stay (raw Hz still shown on the OLED and /sensor).
      // ทั้ง T3/T4/T5 เป็น optional -- ไม่ set success = false ถ้า fail

      // Transaction 7: CTR4A01 current sensor -- optional, ~2Hz/500ms cadence [v16.6a]
      // Cadence gating + result storage only -- readCTR4A01Current() owns the
      // slave-ID switch/restore (shares this RS485-enabled window, runs before
      // rs485Disable() below).
      localData.current_valid = false;
      // [ADR Option E, Phase 1] reaching this line means the bus was not
      // owned by FifoDriver this cycle -- VALID regardless of whether the
      // 500ms cadence gate below actually fires a read this specific tick.
      localData.current_availability = EvidenceAvailability::VALID;
      if (millis() - s_lastCurrentSampleMs >= CURRENT_SAMPLE_INTERVAL_MS) {
        s_lastCurrentSampleMs = millis();
        // [PHASE2-EXPERIMENT] single controlled inter-frame delay before the only
        // readCTR4A01Current() call site -- validates the T6->T7 turnaround hypothesis.
        vTaskDelay(pdMS_TO_TICKS(5));
        localData.current_valid = readCTR4A01Current(localData.current_a);
      }

      rs485Disable("NORMAL-POLL");

      // [Task 4.2A -- TEMPORARY DIAGNOSTIC ONLY] Modbus poll window ended.
      // Only clear ownership to NONE if FIFO isn't concurrently ACTIVE --
      // otherwise this would incorrectly mask a still-in-progress FIFO
      // transaction (its own end transition is logged separately, above,
      // at the ACTIVE-boundary phase-change site).
      if (FifoDriver_GetPhase() != FifoPhase::ACTIVE) {
        FifoDiag_SetBusOwner(BusOwnerDiag::NONE);
      }
    } else {
      // [ADR Option E, Phase 1] FifoDriver owns the bus this cycle -- the
      // whole poll window above (including CTR4A01 current) was skipped.
      // localData.current_a/current_valid retain their previous iteration's
      // values, same as raw_x/y/z above; current_availability is the one
      // explicit signal that this is an expected, not a faulted, gap.
      localData.current_availability = EvidenceAvailability::UNAVAILABLE_EXPECTED;
    }

    if (success) {
      // -- + ??????????: Reset consecutive error counter --
      if (g_modbusConsecErrors > 0) {
        bool wasOffline = g_sensorOffline;
        Serial.printf("[MODBUS] + Sensor back ONLINE (was offline for %u consecutive reads, "
                      "total errors: %lu)\n",
                      g_modbusConsecErrors, g_sensorErrors);
        g_modbusConsecErrors = 0;
        g_sensorOffline      = false;
        // [PATCHED v16.3c] gate warmup ทุกครั้งที่ sensor กลับมา (consec >= 1)
        // เดิม: gate เฉพาะ wasOffline (consec >= 3) → spike ผ่านตอน consec=1
        // ใหม่: gate ทุกครั้ง เพราะ spike เกิดหลัง consec=1 เสมอ
        g_sensorWarmupReads  = 8;  // 8 reads x 250ms = 2s suppress window

        // [PATCHED v16.3d] Re-configure sensor ทุกครั้งที่กลับมา online
        // sensor WTVB02 รีบูตตัวเองจาก RS485 noise แม้ consec=1 read fail
        // ทำให้ MODE กลับ default (0x00) และ CF/VRMS=0
        // แก้: reconfig ทุกครั้งไม่ว่า wasOffline จะเป็น true หรือ false
        Serial.println("[MODBUS] Sensor back -- re-configuring MODE=FreqDomain...");
        // [v16.3w] แก้ diagnosis จาก v16.3v: การรอนานขึ้น (3000ms) ไม่ช่วย —
        // log ยืนยันว่า quick-reconfig ยัง err=226 ครบทุกครั้ง เพราะ sensor ที่ค้างจาก noise
        // ไม่ยอมรับ unlock จนกว่าจะโดน restart command (reg 0x00=0x00FF) จริง ซึ่งมีแค่ full-restart path
        // → fail-fast: รอสั้น + ลองครั้งเดียว แล้วปล่อยให้ ALL-ZERO detector (5 reads=1.25s)
        //   escalate ไป full restart path ที่ฟื้นได้จริง (ไม่เสียเวลา ~5s เปล่า ๆ ในเส้นทางที่ล้มเหลวแน่)
        vTaskDelay(pdMS_TO_TICKS(500));
        // [v16.5f] sensorWasRestarted=false: this is the QUICK reconfig -- no
        // restart command (reg 0x00=0x00FF) is issued on this path, as the
        // comment above states, so the sensor cannot have reverted its NVM and
        // a previously VERIFIED sample rate is still true. Passing false stops
        // this path from destroying that provenance on its way to failing,
        // which is what left srHz=0 permanently in the 2026-08-24T12:51Z event.
        // The full-restart path below still calls with the default (true).
        // [v16.5h] EN-pin fix: rs485Disable("NORMAL-POLL") has already run
        // (above) by the time this branch is reached, so every Modbus
        // transaction inside reconfigSensorAfterRestart() below was executing
        // with RS485_EN_PIN de-asserted -- the exact cause of the "err=226 /
        // ku8MBResponseTimedOut every time" behaviour the v16.3w comment
        // above misattributed to a noise-stuck sensor. Bracket matches the
        // already-correct STUCK-RESTART pattern (rs485Enable/rs485Disable
        // around restartSensorViaModbus()) exactly, including its settle delay.
        rs485Enable("QUICK-RECONFIG");
        vTaskDelay(pdMS_TO_TICKS(5));
        bool reOk = reconfigSensorAfterRestart(false);
        rs485Disable("QUICK-RECONFIG");
        if (!reOk) {
          Serial.println("[MODBUS] WARNING: Sensor reconfig failed -- CF/VRMS may be 0 until next restart");
          // [v16.3m] reconfig fail → extend warmup suppress window
          // sensor อาจส่งค่า garbage สูงผิดปกติ เช่น rms=19.5 mm/s
          // เพิ่ม warmup reads เป็น 16 (4 วินาที) เพื่อ suppress garbage values
          g_sensorWarmupReads = 16;
          Serial.println("[MODBUS] ! Extending warmup suppress to 16 reads (4s) after reconfig fail");
        }
      }

      // ============================================================
      // STUCK DETECTION -- Vx / Vy / Vz
      // ????????: ??????? == 0 ?????????? >= 1 ??? > STUCK_MIN_RAW
      // ???????????????????? ????????????????? 0
      // threshold = 5 ????? x 250ms = 1.25 ??????
      // Cooldown 30 ?????? shared ?????? (restart ??????????????????????)
      // ============================================================

      bool needRestart  = false;   // flag ??????? restart ????????
      const char* stuckAxis = "";  // ?????????? trigger ????? log

      // --- [v16.3f] All-Zero STUCK: raw_x = raw_y = raw_z = 0 พร้อมกัน ---
      // Bug เดิม: per-axis check ต้องการ othersAlive → ถ้าทุกแกน=0 ไม่มีแกนไหน trigger
      // Fix: ตรวจ all-zero แยกก่อน per-axis check เพื่อให้ auto-recovery ทำงานได้
      {
        bool allZero = (abs(raw_x) <= STUCK_MIN_RAW) &&
                       (abs(raw_y) <= STUCK_MIN_RAW) &&
                       (abs(raw_z) <= STUCK_MIN_RAW);

        if (allZero) {
          g_allZeroStuckCount++;
          if (g_allZeroStuckCount == STUCK_THRESHOLD) {
            Serial.printf("[SENSOR] ! ALL-ZERO for %u reads (Vx=%d Vy=%d Vz=%d) -> restart\n",
                          g_allZeroStuckCount, raw_x, raw_y, raw_z);
            needRestart = true;
            stuckAxis   = "ALL";
            g_allZeroRestartCount++;
          }
        } else {
          if (g_allZeroStuckCount > 0) {
            Serial.printf("[SENSOR] + All-zero recovered! (stuck %u reads)\n",
                          g_allZeroStuckCount);
          }
          g_allZeroStuckCount = 0;
        }
      }

      // --- Vy STUCK ---
      {
        bool vyIsZero    = (raw_y == 0);
        bool othersAlive = (abs(raw_x) > STUCK_MIN_RAW) || (abs(raw_z) > STUCK_MIN_RAW);

        if (vyIsZero && othersAlive) {
          g_vyStuckCount++;
          if (g_vyStuckCount == STUCK_THRESHOLD) {
            Serial.printf("[SENSOR] ! Vy=0 for %u reads (Vx=%d Vz=%d) -> restart\n",
                          g_vyStuckCount, raw_x, raw_z);
            needRestart = true;
            stuckAxis   = "Vy";
            g_vyRestartCount++;
          }
        } else {
          if (g_vyStuckCount > 0 && !vyIsZero) {
            Serial.printf("[SENSOR] + Vy recovered! (stuck %u reads, now raw_y=%d)\n",
                          g_vyStuckCount, raw_y);
          }
          g_vyStuckCount = 0;
        }
      }

      // --- Vz STUCK ---
      if (!needRestart) {
        bool vzIsZero    = (raw_z == 0);
        bool othersAlive = (abs(raw_x) > STUCK_MIN_RAW) || (abs(raw_y) > STUCK_MIN_RAW);

        if (vzIsZero && othersAlive) {
          g_vzStuckCount++;
          if (g_vzStuckCount == STUCK_THRESHOLD) {
            Serial.printf("[SENSOR] ! Vz=0 for %u reads (Vx=%d Vy=%d) -> restart\n",
                          g_vzStuckCount, raw_x, raw_y);
            needRestart = true;
            stuckAxis   = "Vz";
            g_vzRestartCount++;
          }
        } else {
          if (g_vzStuckCount > 0 && !vzIsZero) {
            Serial.printf("[SENSOR] + Vz recovered! (stuck %u reads, now raw_z=%d)\n",
                          g_vzStuckCount, raw_z);
          }
          g_vzStuckCount = 0;
        }
      }

      // --- Vx STUCK ---
      if (!needRestart) {
        bool vxIsZero    = (raw_x == 0);
        bool othersAlive = (abs(raw_y) > STUCK_MIN_RAW) || (abs(raw_z) > STUCK_MIN_RAW);

        if (vxIsZero && othersAlive) {
          g_vxStuckCount++;
          if (g_vxStuckCount == STUCK_THRESHOLD) {
            Serial.printf("[SENSOR] ! Vx=0 for %u reads (Vy=%d Vz=%d) -> restart\n",
                          g_vxStuckCount, raw_y, raw_z);
            needRestart = true;
            stuckAxis   = "Vx";
            g_vxRestartCount++;
          }
        } else {
          if (g_vxStuckCount > 0 && !vxIsZero) {
            Serial.printf("[SENSOR] + Vx recovered! (stuck %u reads, now raw_x=%d)\n",
                          g_vxStuckCount, raw_x);
          }
          g_vxStuckCount = 0;
        }
      }

      // --- ??? Restart ????? axis ??? stuck ??? threshold ---
      if (needRestart) {
        // [Task 5.3 -- TEMPORARY DIAGNOSTIC ONLY, auto-restart runtime-
        // verification investigation, not a permanent production feature]
        // needRestart-just-became-true + restartSensorViaModbus() entry,
        // with the exact FIFO-ownership/phase/attempt context Task 5.1/5.2
        // identified as unguarded. This is the single point that answers
        // "was a FIFO session ACTIVE when auto-restart fired?".
        uint32_t t5_3EntryMs = millis();
        Serial.printf("[RESTART-DIAG] needRestart=true axis=%s fifoOwnsBus=%d "
                      "fifoPhase=%d attempt=%lu t=%lums\n",
                      stuckAxis, (int)FifoDriver_OwnsBus(),
                      static_cast<int>(FifoDriver_GetPhase()),
                      (unsigned long)FifoDriver_GetAttemptNumberForDiag(),
                      (unsigned long)t5_3EntryMs);
        rs485Enable("STUCK-RESTART");
        vTaskDelay(pdMS_TO_TICKS(5));

        Serial.printf("[RESTART-DIAG] restartSensorViaModbus() ENTRY axis=%s "
                      "fifoOwnsBus=%d fifoPhase=%d attempt=%lu t=%lums\n",
                      stuckAxis, (int)FifoDriver_OwnsBus(),
                      static_cast<int>(FifoDriver_GetPhase()),
                      (unsigned long)FifoDriver_GetAttemptNumberForDiag(),
                      (unsigned long)millis());
        bool restartOk = restartSensorViaModbus(stuckAxis);
        Serial.printf("[RESTART-DIAG] restartSensorViaModbus() EXIT ok=%d axis=%s "
                      "fifoOwnsBus=%d fifoPhase=%d attempt=%lu elapsedMs=%lu t=%lums\n",
                      (int)restartOk, stuckAxis, (int)FifoDriver_OwnsBus(),
                      static_cast<int>(FifoDriver_GetPhase()),
                      (unsigned long)FifoDriver_GetAttemptNumberForDiag(),
                      (unsigned long)(millis() - t5_3EntryMs), (unsigned long)millis());

        if (restartOk) {
          Serial.printf("[SENSOR] + Auto-restart OK (axis=%s), monitoring recovery...\n", stuckAxis);
        } else {
          Serial.printf("[SENSOR] x Auto-restart FAILED (axis=%s), retry after cooldown\n", stuckAxis);
          // reset counters ????????????????????????? cooldown
          g_vxStuckCount = 0;
          g_vyStuckCount = 0;
          g_vzStuckCount = 0;
        }

        rs485Disable("STUCK-RESTART");

        // Reset timing ????? restart ??????? ~3 ??????
        xLastWakeTime = xTaskGetTickCount();
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        continue;  // ?????????? ?????????????
      }

      // ============================================================
      // DATA PROCESSING [v15.0]
      // ============================================================

      // [Phase2] Step 1/2 REMOVED. raw_x/y/z (VRMS registers 0x50/0x5C/0x68) are
      // NO LONGER converted into a vibration measurement. They survive solely as
      // the input to the all-zero / Vx / Vy / Vz stuck detection and the sensor
      // auto-restart above -- a hardware watchdog, not a metric. The single
      // authoritative vibration measurement is FIFO RAW -> DSP velocity
      // (VibVelocity_ComputeRms -> g_velCarrier -> velocity_rms_*).

      // Step 3: Sensor-computed CF & Kurtosis -- ครบ 3 แกน [v15.1]
      // คำนวณจาก 16KHz raw FIFO ภายใน chip
      // raw = 0 ถ้า transaction fail (ปลอดภัย -- guard > 0)
      localData.cf_x       = (raw_cfx > 0) ? raw_cfx / 1000.0f : 0.0f;
      localData.cf_y       = (raw_cfy > 0) ? raw_cfy / 1000.0f : 0.0f;
      localData.cf_z       = (raw_cfz > 0) ? raw_cfz / 1000.0f : 0.0f;

      // [Phase 1] kurtosis_x/y/z scaling REMOVED with the bearing pipeline.

      // Derived: max CF [v15.1]. cf_x/y/z remain a validation REFERENCE for the
      // canonical FIFO/DSP crest factor -- see the [CF-SRC] log. No product
      // consumer reads them.
      // [Phase 1] kurtosis_max / kurtosis_dominant_axis derivation REMOVED.
      localData.cf_max = max(localData.cf_x, max(localData.cf_y, localData.cf_z));

      // Step 4: Misc
      localData.temperature = raw_temp / 100.0f;
      localData.freq_x = raw_fx / 10.0f;
      localData.freq_y = raw_fy / 10.0f;
      localData.freq_z = raw_fz / 10.0f;

      // Step 4c: Peak Velocity X/Y/Z (signed, raw/100) [DESIGN-0004]
      // เก็บค่า signed ตรงจาก register -- ไม่ทำ abs() (Decision 4, ยืนยันจาก datasheet §6.4.6)
      // raw = 0 ถ้า T6 fail (optional transaction, ไม่กระทบ success หลัก) -> ค่าเป็น 0.0f เอง

      // [v16.3u] Step 4b: NaN / Inf guard — Defensive float check
      // ป้องกัน PANIC จาก Modbus corrupt value ที่ผ่าน sanity check แต่ทำให้ float exception
      // เงื่อนไขที่ trigger: raw_x อยู่ใน valid range แต่ pattern แปลก
      // → isnan / isinf จะ true → CPU exception ถ้าเอาไปคำนวณต่อ
      // Fix: ตรวจทุก derived float ก่อนใช้งาน ถ้าผิดปกติ → ใช้ค่า safe แทน
      auto isFloatSafe = [](float v) -> bool {
        return !isnan(v) && !isinf(v) && v >= 0.0f;
      };

      // ถ้าค่าใดค่าหนึ่งเป็น NaN/Inf → skip cycle นี้ทั้งหมด (ไม่เข้า State Machine)
      // [Phase2] legacy rms_overall / vel_peak_* dropped from this guard along
      // with the fields themselves; temperature / CF still checked.
      // [Phase 1] the kurtosis_max term is gone with the field. It was already
      // unreachable -- kurtosis_max derived from uint16/1000.0f, which can be
      // neither NaN nor Inf -- so removing it cannot change which cycles pass.
      if (!isFloatSafe(localData.temperature)   ||
          !isFloatSafe(localData.cf_max)) {
        Serial.printf("[SENSOR] ! NaN/Inf detected in derived values -- skipping cycle "
                      "(temp=%.2f cf=%.2f) [v16.3u]\n",
                      localData.temperature,
                      localData.cf_max);
        // ไม่ set localData.valid = true → State Machine ไม่รับค่านี้
        rs485Disable("NAN-GUARD");
        xLastWakeTime = xTaskGetTickCount();
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        continue;
      }

      // [Phase2] Step 5 Velocity Peak Hold REMOVED together with g_velPeakHold.
      // It was a max-hold over the legacy VRMS registers that bypassed the
      // de-glitch entirely. FIFO/DSP exposes no peak metric and none is invented
      // here -- a consumer that wanted "peak" now gets nothing, not a fake value.
      // ---------------------------------------------------------------

      localData.timestamp = millis();
      localData.valid = true;

      localData.read_time_ms     = (uint16_t)(millis() - t_readStart);
      localData.poll_interval_ms = pollIntervalMs;
      localData.retry_count      = retryCount;
      localData.crc_ok           = success;   // มาถึงจุดนี้ = ทุก read ผ่าน CRC


      // Send to queue (non-blocking)
      if (xQueueSend(queueSensorData, &localData, 0) != pdPASS) {
        Serial.println("[CORE 0] Sensor queue full!");
      }
    } else {
      // -- x Modbus ?????????? (timeout / no response) --
      g_sensorErrors++;
      g_modbusConsecErrors++;

      if (g_modbusConsecErrors >= MODBUS_OFFLINE_THRESHOLD) {
        // ?????? OFFLINE ?????????????? threshold
        if (!g_sensorOffline) {
          g_sensorOffline = true;
          Serial.printf("[MODBUS] ! SENSOR OFFLINE -- %u consecutive errors "
                        "(total: %lu). Check sensor power / RS485 wiring.\n",
                        g_modbusConsecErrors, g_sensorErrors);
        }

        // ??? invalid packet ????? queue ??? cycle ??? offline
        // ???????? taskStateMachine ??????? snapshot ???????????? ERROR ?? display [v16.5.4: was g_vibData]
        memset(&localData, 0, sizeof(VibrationData_t));
        localData.valid     = false;
        localData.timestamp = millis();
        if (xQueueSend(queueSensorData, &localData, 0) != pdPASS) {
          // queue ???? -- ??? critical, ??????????
        }
      } else {
        // ????????? threshold -- log warning ??????????????? offline
        Serial.printf("[MODBUS] x Read failed (consec=%u/%d, total=%lu)\n",
                      g_modbusConsecErrors, MODBUS_OFFLINE_THRESHOLD, g_sensorErrors);
      }
    }

    // Wait until next cycle (precise timing)
    // [v16.6h] Cadence selection -- see xFrequencyFifo's declaration at the
    // top of this function for the full rationale. This is the only delay
    // site that needs gating: the loop's other two vTaskDelayUntil() call
    // sites (the stuck-restart and NaN-guard early-continues) are nested
    // inside the `if (!FifoDriver_OwnsBus())` block above and so are
    // unreachable while FIFO owns the bus -- they always resolve to 250ms.
    vTaskDelayUntil(&xLastWakeTime,
                    FifoDriver_OwnsBus() ? xFrequencyFifo : xFrequency);
  }
}

/**
 * Task 2: State Machine & Safety Logic (CORE 0, Priority 4)
 * Processes sensor data and updates machine state
 * Triggered by queue from Modbus task
 */
void taskStateMachine(void* parameter) {
  VibrationData_t sensorData;

  Serial.println("[CORE 0] State machine task started");

  while (1) {
    // Wait for new sensor data (blocking on queue)
    if (xQueueReceive(queueSensorData, &sensorData, portMAX_DELAY) == pdPASS) {

      // -- ???????: sensor offline (valid = false) --
      if (!sensorData.valid) {
        // [v16.5.4] g_vibData removed (see declaration comment) -- the
        // zeroed/invalid offline view is now built directly into the
        // snapshot capture below instead of a separate g_vibData write.

        // ?????? state ???? NORMAL -- ???? trigger alarm ??? sensor ???????
        // [v16.5.4] offlineState captured in this SAME critical section
        // (rather than a second mutexSystemState take below) -- one lock
        // acquisition instead of two, and it is exactly the post-update
        // value (MAINTENANCE preserved if that's what it was).
        MachineState_t offlineState = STATE_NORMAL;
        if (xSemaphoreTake(mutexSystemState, pdMS_TO_TICKS(10)) == pdTRUE) {
          if (g_systemState.state != STATE_MAINTENANCE) {
            if (g_systemState.state != STATE_NORMAL) {
              Serial.println("[STATE] Sensor OFFLINE -> forced STATE_NORMAL, buzzer OFF");
            }
            g_systemState.state       = STATE_NORMAL;
            g_systemState.buzzerActive = false;
          }
          offlineState = g_systemState.state;
          xSemaphoreGive(mutexSystemState);
        }

        // [v16.5.4] Improvement 2: keep the atomic snapshot coherent with the
        // offline view consumers previously saw in g_vibData (zeroed struct,
        // valid=false, timestamp preserved). alarm_level mirrors the actual
        // post-update g_systemState.state so MAINTENANCE is not misreported.
        {
          VibrationData_t offlineVib;
          memset(&offlineVib, 0, sizeof(VibrationData_t));
          offlineVib.valid     = false;
          offlineVib.timestamp = sensorData.timestamp;
          captureTelemetrySnapshot(&offlineVib, offlineState);
        }
        continue;  // ??????????????? RPM / state machine ?????????
      }

      // -- + ?????????????: ???????????? --
      processRPM(&sensorData);

      // [Phase2] LEGACY VRMS DE-GLITCH REMOVED IN FULL.
      // Deleted: s_lastGoodRms / s_lastGoodX/Y/Z / s_glitchHold, isDropGlitch,
      // isSpikeGlitch, the [DEGLITCH] forensic log, g_deglitchCount and the
      // VERIFY_TEST PollDiagRecord instrumentation built around them.
      // It existed only to patch a defect of the VRMS register, which no longer
      // feeds any measurement. g_lastRmsOverall (and with it anaVrmsHealthy() /
      // ANA_FRZ_VRMS_INVALID) is removed for the same reason.

      // [v16.5.4] g_vibData removed (see declaration comment) -- sensorData
      // reaches consumers via captureTelemetrySnapshot() at the end of this
      // block instead of a separate g_vibData memcpy here.

      // [Phase2] Temperature history ring (Core 0 only, no mutex needed).
      // Replaces the former g_trendBuf, which mixed four unrelated concerns
      // (legacy VRMS rms, legacy VPEAK peak, temperature, freq_ratio) in one
      // struct. rms/peak/freq_ratio are gone; temperature is the only survivor
      // and it gets a plain float ring of its own.
      g_tempBuf[g_tempHead] = sensorData.temperature;
      g_tempHead  = (g_tempHead + 1) % TEMP_BUF_SIZE;
      if (g_tempCount < TEMP_BUF_SIZE) g_tempCount++;

      // -- Push current sample into circular buffer (Core 0 only, no mutex) [v16.6a] --
      // sensorData.current_valid is only true on cycles where taskModbusRead actually
      // sampled CTR4A01 (~500ms cadence); other cycles are skipped so calcTrend()'s
      // regression sees one entry per real sample, not per 250ms task tick.
      if (sensorData.current_valid) {
        g_currentBuf[g_currentHead] = sensorData.current_a;
        g_currentHead  = (g_currentHead + 1) % CURRENT_BUF_SIZE;
        if (g_currentCount < CURRENT_BUF_SIZE) g_currentCount++;
      }

      // Determine new state based on RMS
      // v16.0: gate ด้วย MOTOR_RUNNING -- STARTING/STOPPING มี transient RMS สูง
      // ไม่ควร trigger STATE_WARNING/CRITICAL ขณะ ramp-up/down
      MachineState_t newState;
      // [M1A] The Product Phase-1 vibration alarm source.
      float    vibMmS  = 0.0f;
      uint32_t vibAge  = 0;
      uint32_t vibCaptureId = 0;                             // [S21]
      const bool vibOk = readVelocityForAlarm(&vibMmS, &vibAge, &vibCaptureId);
      g_vibUnavailable = !vibOk;   // exposed on telemetry/display

      // [S21] Escalation-persistence state. Touched ONLY here, inside
      // taskStateMachine (Core 0), so it needs no lock. s_vibLastCaptureId
      // makes the counter advance once per new FIFO capture rather than once
      // per 250 ms poll of the same measurement.
      static uint32_t       s_vibLastCaptureId = 0;
      static MachineState_t s_vibPendState     = STATE_NORMAL;
      static uint8_t        s_vibPendCount     = 0;

      // [M1A] Gate on whether a vibration alarm DECISION may be made at all.
      // This is separate from newState: when false, newState is not applied,
      // so the previous state (including any active WARNING/CRITICAL) is held
      // rather than being overwritten with a fabricated NORMAL.
      bool vibDecisionValid = true;

      // [PATCHED v16.3b] Suppress transient spike หลัง sensor กลับ online
      // sensor ให้ค่า spike สูงใน 1-2 reads แรกหลัง power cycle (เห็น RMS=22mm/s)
      if (g_sensorWarmupReads > 0) {
        g_sensorWarmupReads--;
      }
      // [vNext] TEMPORARY: tick down the FAULT_LATCH startup-settling holdoff, same
      // cadence as g_sensorWarmupReads above. FAULT_LATCH qualification only.
      if (g_motorRunFaultLatchHoldoff > 0) {
        g_motorRunFaultLatchHoldoff--;
      }

      // [M1A] Vibration alarm decision tree. Reads vibMmS (FIFO RAW -> DSP
      // velocity_rms_overall). Does NOT read rms/sensorData.rms_overall.
      // [S21] Any branch that does not perform a real threshold comparison
      // must also drop a part-accumulated escalation. Requirement: an
      // observation with velocity_data_valid == 0 is NOT an alarm
      // observation, so it can neither count toward nor bridge a run of two.
      if (g_motorRunState != MOTOR_RUNNING || g_sensorWarmupReads > 0 ||
          !vibOk || !vibThresholdsConfigured()) {
        s_vibPendCount     = 0;
        s_vibPendState     = STATE_NORMAL;
        s_vibLastCaptureId = 0;
      }

      if (g_motorRunState != MOTOR_RUNNING) {
        newState = STATE_NORMAL;  // STOPPED/STARTING/STOPPING → ไม่ประเมิน alarm
        // Legitimate NORMAL: the machine genuinely is not running. This is a
        // real decision, not an absence of one, so it may clear an alarm.
      } else if (g_sensorWarmupReads > 0) {
        newState = STATE_NORMAL;  // warmup reads หลัง sensor online → suppress spike
      } else if (!vibOk) {
        // ── VIBRATION_UNAVAILABLE ────────────────────────────────────────────
        // Velocity invalid or stale (FIFO suspended, DSP gate failed, or no
        // capture within the freshness deadline).
        //   - no new vibration alarm is raised
        //   - 0.0f is NOT interpreted as low vibration
        //   - any existing alarm is HELD, never cleared
        newState = STATE_NORMAL;      // placeholder only -- NOT applied below
        vibDecisionValid = false;
      } else if (!vibThresholdsConfigured()) {
        // ── THRESHOLDS TBD ───────────────────────────────────────────────────
        // Velocity is valid and fresh, but VIB_WARNING_MMS / VIB_CRITICAL_MMS
        // have not been re-baselined. Per M1A requirement 6 no vibration
        // WARNING/CRITICAL may be raised, and equally no NORMAL may be
        // asserted -- asserting NORMAL would clear a latched alarm on the
        // strength of a comparison that was never performed.
        newState = STATE_NORMAL;      // placeholder only -- NOT applied below
        vibDecisionValid = false;
      } else {
        // ── [S21] HYSTERESIS + ESCALATION PERSISTENCE ───────────────────────
        // Same single evaluation engine as before -- this replaces the bare
        // three-way comparison in place; no second engine is introduced.
        //
        // HYSTERESIS: the band a sample is compared against depends on the
        // state already held, so a machine sitting on a boundary cannot flip
        // state (and the buzzer) on measurement spread at the 250 ms cadence.
        // PERSISTENCE: escalation additionally requires
        // VIB_ALARM_PERSIST_CAPTURES distinct FIFO captures agreeing, so one
        // transient sample can never raise an alarm. De-escalation applies
        // immediately -- clearing is the safe direction.
        const MachineState_t prevVib =
            (g_systemState.state == STATE_WARNING ||
             g_systemState.state == STATE_CRITICAL) ? g_systemState.state
                                                    : STATE_NORMAL;

        MachineState_t cand;
        if (prevVib == STATE_CRITICAL) {
          if      (vibMmS <  VIB_WARNING_OFF_MMS)  cand = STATE_NORMAL;
          else if (vibMmS <  VIB_CRITICAL_OFF_MMS) cand = STATE_WARNING;
          else                                     cand = STATE_CRITICAL;
        } else if (prevVib == STATE_WARNING) {
          if      (vibMmS >= VIB_CRITICAL_MMS)     cand = STATE_CRITICAL;
          else if (vibMmS <  VIB_WARNING_OFF_MMS)  cand = STATE_NORMAL;
          else                                     cand = STATE_WARNING;
        } else {  // held NORMAL -- use the ON edges
          if      (vibMmS >= VIB_CRITICAL_MMS)     cand = STATE_CRITICAL;
          else if (vibMmS >= VIB_WARNING_MMS)      cand = STATE_WARNING;
          else                                     cand = STATE_NORMAL;
        }

        if (cand <= prevVib) {
          // Same level or de-escalation: apply now, drop any pending count.
          newState = cand;
          s_vibPendCount = 0;
          s_vibPendState = cand;
        } else if (vibCaptureId != s_vibLastCaptureId) {
          // Escalation, and this is genuinely NEW evidence -- count it once.
          s_vibLastCaptureId = vibCaptureId;
          if (cand == s_vibPendState) {
            if (s_vibPendCount < 0xFFu) s_vibPendCount++;
          } else {
            s_vibPendState = cand;
            s_vibPendCount = 1;
          }
          newState = (s_vibPendCount >= VIB_ALARM_PERSIST_CAPTURES)
                       ? cand : prevVib;
        } else {
          // Escalation candidate, but this capture was already counted.
          // Hold the current state; never double-count one measurement.
          newState = prevVib;
        }
      }

      // Update system state (with mutex)
      // [v16.5.4] effectiveState = post-update g_systemState.state for the
      // snapshot capture below (preserves MAINTENANCE, which newState never
      // carries). Falls back to newState if the mutex times out.
      MachineState_t effectiveState = newState;
      if (xSemaphoreTake(mutexSystemState, pdMS_TO_TICKS(10)) == pdTRUE) {
        MachineState_t oldState = g_systemState.state;

        // Skip if in maintenance mode
        // [M1A] vibDecisionValid gates the whole transition: when no vibration
        // decision could be made (unavailable/stale, or thresholds unset) the
        // state is left exactly as it was. This is the single point that
        // guarantees an unavailable reading can neither raise nor clear an
        // alarm -- the buzzer, LED and effectiveState all follow from here.
        if (g_systemState.state != STATE_MAINTENANCE && vibDecisionValid) {
          if (newState != oldState) {
            g_systemState.state = newState;
            g_systemState.stateEntryTime = millis();
            g_systemState.alarmAcknowledged = false;

            // Activate buzzer on WARNING/CRITICAL
            if (newState >= STATE_WARNING) {
              g_systemState.buzzerActive = true;
            } else {
              g_systemState.buzzerActive = false;
            }

            // [M1A] Report the value that actually drove the decision.
            // [Phase2] the legacy VRMS comparison figure is gone with the metric.
            // Transition-only -- not per-tick.
            Serial.printf("[CORE 0] State: %d -> %d (vel: %.3f mm/s)\n",
                          oldState, newState, vibMmS);
          }
        }

        effectiveState = g_systemState.state;  // [v16.5.4] actual state after update
        xSemaphoreGive(mutexSystemState);
      }

      // ── Fault Latch v3: evaluate fault transitions ────────────────────────
      {
        int latchHealth = 100;
        // [M1A] Fault-latch health now derives from the velocity carrier, not
        // the deprecated VRMS register value. The legacy `rmsValid` sanity
        // gate is replaced by the carrier's own validity + freshness check --
        // the DSP path has no "garbage from reconfig fail" failure mode to
        // sanity-bound, it simply reports valid=false.
        //
        // healthUsable is what decides whether a HEALTH_LOW latch may fire at
        // all. Without it, HEALTH_SCORE_UNKNOWN (-1) would satisfy
        // checkAndLatchFault()'s `healthScore <= FL_HEALTH_LOW_THOLD` test and
        // manufacture a health fault out of missing data -- the exact
        // inversion M1A requirement 7 forbids.
        float      latchVibMmS = 0.0f;
        const bool latchVibOk  = readVelocityForAlarm(&latchVibMmS, NULL, NULL);
        const bool healthUsable = (latchVibOk && vibThresholdsConfigured());

        if (healthUsable && sensorData.motor_state == 2 &&
            latchVibMmS > VIB_WARNING_MMS) {
          float norm = (latchVibMmS - VIB_WARNING_MMS) /
                       (VIB_CRITICAL_MMS - VIB_WARNING_MMS) * 100.0f;
          latchHealth = (int)max(0.0f, min(100.0f, roundf(100.0f - norm)));
        }
        // [v16.3m] suppress latch ถ้า rms garbage หรืออยู่ใน warmup suppress
        // [vNext] TEMPORARY: added g_motorRunFaultLatchHoldoff term -- suppresses
        // FAULT_LATCH only, for a short window right after STARTING->RUNNING, so the
        // mechanical/vibration settling transient at motor start-up cannot create a
        // latch. Does not alter newState (STATE_WARNING/CRITICAL, buzzer, live
        // /vibration telemetry still reflect the real spike exactly as before),
        // DEGLITCH, or the FIFO Broker -- this function already returns before evCode/
        // the FIFO-enqueue block are evaluated whenever suppressed.
        // [M1A] Added !healthUsable and !vibDecisionValid to the suppression
        // set. checkAndLatchFault() returns immediately when suppressed and
        // ONLY ever sets latches (g_fl.code/g_flCount) -- it contains no clear
        // path -- so suppressing here can never clear an existing latch. That
        // is the structural guarantee behind M1A requirement 7's "do not clear
        // an existing vibration alarm latch".
        const bool suppressLatch = (!healthUsable || !vibDecisionValid ||
                                    g_sensorWarmupReads > 0 || g_motorRunFaultLatchHoldoff > 0);
        // [M1A] EDGE-TRIGGERED, not per-tick. With thresholds deliberately
        // unset, suppressLatch is true on EVERY 4 Hz cycle -- the original
        // unconditional printf would have become a 4 lines/second flood, which
        // M1A requirement 14 forbids. Logging only on transition keeps the
        // diagnostic value at zero steady-state cost.
        static bool s_prevSuppressLatch = false;
        if (suppressLatch != s_prevSuppressLatch) {
          Serial.printf("[LATCH] Suppress %s -- vibOk=%d thrCfg=%d decisionValid=%d warmup=%u runHoldoff=%u\n",
                        suppressLatch ? "ON" : "OFF",
                        (int)latchVibOk, (int)vibThresholdsConfigured(),
                        (int)vibDecisionValid, (unsigned)g_sensorWarmupReads,
                        (unsigned)g_motorRunFaultLatchHoldoff);
          s_prevSuppressLatch = suppressLatch;
        }
        const bool latchBearing = false;  // [v16.3l] ปิดถาวร

        checkAndLatchFault(&sensorData, newState, latchHealth,
                           latchBearing, suppressLatch);
      }
      // ── End Fault Latch ───────────────────────────────────────────────────

      // [v16.5.4] Improvement 2: THE single snapshot capture for this cycle --
      // after updateMotorStateMachine() (inside processRPM() above) and the
      // alarm/health evaluation have completely finished. sensorData is not
      // modified after the de-glitch block, so every field captured here
      // belongs to this exact sensor sample and this exact decision cycle.
      captureTelemetrySnapshot(&sensorData, effectiveState);
    }
  }
}

// ============================================================================
// CORE 1 TASKS - USER INTERFACE & NETWORK
// ============================================================================

/**
 * Task 3: OLED Display Rendering (CORE 1, Priority 3)
 * Updates display at 10 Hz (100ms)
 * Uses I2C mutex to prevent conflicts
 */
void taskDisplayUpdate(void* parameter) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(100);  // 100ms = 10Hz

  VibrationData_t localVibData;
  MachineState_t localState;
  DisplayPage_t localPage;
  bool blinkState = false;
  uint32_t lastBlink = 0;

  Serial.println("[CORE 1] Display task started");

  while (1) {
    uint32_t now = millis();

    // Update blink state
    uint32_t blinkInterval = (localState == STATE_CRITICAL) ? 300 : 1000;
    if (now - lastBlink >= blinkInterval) {
      blinkState = !blinkState;
      lastBlink = now;
    }

    // Get current data (with mutex)
    // [v16.5.4] Improvement 2: read the atomic snapshot's measurement record
    // instead of g_vibData (identical content, single capture point).
    // localState/localPage intentionally stay on g_systemState below: they are
    // UI state (page selection, button-driven MAINTENANCE entry/exit) that
    // must react immediately, not telemetry.
    if (xSemaphoreTake(mutexVibData, pdMS_TO_TICKS(5)) == pdTRUE) {
      memcpy(&localVibData, &g_telemSnapshot.vib, sizeof(VibrationData_t));
      xSemaphoreGive(mutexVibData);
    }

    if (xSemaphoreTake(mutexSystemState, pdMS_TO_TICKS(5)) == pdTRUE) {
      localState = g_systemState.state;
      localPage = g_systemState.currentPage;
      xSemaphoreGive(mutexSystemState);
    }

    // Render display (with I2C mutex)
    if (xSemaphoreTake(mutexI2C, pdMS_TO_TICKS(50)) == pdTRUE) {
      u8g2.clearBuffer();

      // Route to appropriate page renderer
      if (!localVibData.valid) {
        // -- Sensor offline -- ???????? ERROR ?????????? --
        drawSensorOfflineScreen();
      } else if (localPage == PAGE_MACHINE) {
        if (localState == STATE_CRITICAL) {
          drawCriticalScreen(&localVibData, blinkState);
        } else if (localState == STATE_WARNING) {
          drawWarningScreen(&localVibData, blinkState);
        } else {
          drawMachineScreen(&localVibData);
        }
      } else if (localPage == PAGE_AXIS) {
        drawAxisScreen(&localVibData);
      } else if (localPage == PAGE_NETWORK) {
        drawNetworkScreen();
      }

      u8g2.sendBuffer();
      xSemaphoreGive(mutexI2C);

      g_displayUpdates++;
    }

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

/**
 * Task 4: 4G Modem & MQTT Network (CORE 1, Priority 2)
 * Handles 4G connectivity and telemetry publishing
 * Based on working code from ESPSS3_in_ro_R11_final.ino
 */
void taskNetwork(void* parameter) {
  Serial.println("[CORE 1] Network task started (4G Modem)");

  // FIX-WDT (v14.9): Subscribe Network task to TWDT with extended timeout.
  // Arduino ESP32 core auto-subscribes IDLE tasks to the 5s TWDT. When
  // Network4G blocks CPU1 (modem I/O, TLS handshake, MQTT publish over 4G)
  // the IDLE task on CPU1 starves → "Network4G did not reset watchdog" crash.
  // Solution: subscribe this task explicitly and reset at every loop iteration.
  // Timeout set to 30s to cover worst-case modem re-registration + handshake.
  // The 100ms vTaskDelay at end of main loop resets the WDT each tick.
  esp_task_wdt_add(NULL);

  vTaskDelay(pdMS_TO_TICKS(5000));  // Wait for system to stabilize

  // Initialize modem
  // [v16.6d] FIX-WDT: modemInit() may block up to ~85s across its internal retries.
  esp_task_wdt_reset();
  if (!modemInit()) {
    Serial.println("[CORE 1] Modem init failed!");
    // Continue running but in error state
  } else {
    // Enable automatic network time update on the modem
    modemEnableNetworkTime();
  }
  esp_task_wdt_reset();   // [v16.6d] FIX-WDT: modemInit() returned

  // Connect to GPRS if modem is ready
  if (g_network.modemReady) {
    // [v16.6d] FIX-WDT: modemConnectGPRS() -> waitForNetwork(30000L) may block up to 30s.
    esp_task_wdt_reset();
    modemConnectGPRS();
    esp_task_wdt_reset();   // [v16.6d] FIX-WDT: modemConnectGPRS() returned

    // Perform initial time sync after GPRS connects
    if (g_network.gprsConnected) {
      Serial.println("[CORE 1] Performing initial NTP time sync...");
      // Wait a moment for modem to receive network time
      vTaskDelay(pdMS_TO_TICKS(3000));
      // [v16.5j] Routed through checkAndSyncTime() instead of calling
      // syncRTCFromModem() directly -- this was the second, independent
      // runtime call site, and it never touched lastCheckMillis, so it sat
      // outside the fixed scheduler entirely. g_timeSync.lastCheckMillis is
      // still 0 at this point (nothing has run yet), so checkAndSyncTime()'s
      // own "lastCheckMillis == 0" first-run gate performs the check
      // immediately here -- identical timing to the direct call it replaces
      // -- while now going through the single scheduling/logging path.
      checkAndSyncTime();
    }
  }

  Serial.printf("[CORE 1] Client ID: %s\n", MQTT_CLIENT_ID);

  // FIX-2: Removed raw TCP test against port 8883.
  // Port 8883 is TLS-only. A plaintext TCP connect causes Mosquitto's OpenSSL to start
  // a TLS handshake; the broker then receives garbage (no ClientHello), logs an SSL
  // error, and may rate-limit or flag subsequent connections from the same IP.
  // If port reachability testing is required, test a non-TLS port (e.g. port 80 HTTP).


  // -- Load mTLS certs into ESP32 mbedTLS (same as aws_test.ino setupTLS) --
  setupTLS();

  // -- MQTT over mTLS (arduino-mqtt + GsmTLSClient + ESP32 mbedTLS) --
  mqttClient.begin(MQTT_SERVER, MQTT_PORT, gsmClient);
  mqttClient.setKeepAlive(60);

  // Connection state tracking
  bool lastGprsState = false;
  bool lastMqttState = false;
  uint32_t lastConnectionAttempt = 0;
  uint32_t lastPublish = 0;
  uint32_t lastSignalCheck = 0;
  uint32_t lastStatusCheck = 0;
  const uint32_t CONNECTION_RETRY_INTERVAL = 30000;

  // v15.4 Fix C: Exponential backoff สำหรับ MQTT reconnect
  // 30s → 60s → 120s → 300s (max) -- reset เมื่อ connect สำเร็จ
  uint32_t mqttBackoffMs   = 30000;   // เริ่มต้นที่ 30s
  const uint32_t BACKOFF_MIN =  30000;
  const uint32_t BACKOFF_MAX = 300000; // max 5 นาที
  uint8_t  mqttFailCount   = 0;       // นับ fail ต่อเนื่อง

  // [v16.5.4] Improvement 2: one atomic snapshot replaces the separate
  // localVibData (g_vibData) + localState (g_systemState) copies -- vibration
  // data and system state can no longer come from different cycles.
  TelemetrySnapshot localSnap = {};

  while (1) {
    uint32_t now = millis();

    // v15.4 Fix A: WDT reset ทุก iteration -- ป้องกัน WDT ตอน network check ค้าง
    // (modem AT commands, GPRS check, NTP sync อาจใช้เวลา > 1s ต่อ call)
    esp_task_wdt_reset();

    // Check network status every 10 seconds
    if (now - lastStatusCheck > 10000) {
      lastStatusCheck = now;

      if (g_network.modemReady) {
        bool gprs = modem.isGprsConnected();
        bool network = modem.isNetworkConnected();

        // Try to reconnect network if lost
        if (!network) {
          Serial.println("[CORE 1] Network not connected, waiting...");
          esp_task_wdt_reset();   // v15.4: waitForNetwork อาจค้าง 10s
          modem.waitForNetwork(10000L);
          esp_task_wdt_reset();
          network = modem.isNetworkConnected();
          // [minimal-fix, keepalive-starvation] service MQTT keepalive
          // immediately after this <=10s blocking call, independent of
          // whatever the rest of this status-check block still has to do.
          // Safe unconditionally -- MQTTClient::loop() no-ops when not
          // connected (MQTTClient.cpp:507-511). Does not touch mqttConnSnap
          // or any existing cache-write; purely additive.
          mqttClient.loop();
        }

        // Try to reconnect GPRS if network is up but GPRS is down
        if (network && !gprs) {
          Serial.println("[CORE 1] Reconnecting GPRS...");
          // [v16.5d] Graceful MQTT close before the PDP context churns --
          // gprsConnectImpl() internally issues AT+NETCLOSE (closes all
          // sockets) before AT+NETOPEN, orphaning any still-"connected"
          // local MQTT/TLS state without the broker ever seeing a clean
          // disconnect (session-takeover root cause). Bounded via
          // BOUNDED_STOP_MS above -- see GsmTLSClient::stop()/resetTLS().
          if (mqttClient.connected()) {
            Serial.println("[MQTT] Graceful disconnect before GPRS reconnect");
            mqttClient.disconnect();
          }
          esp_task_wdt_reset();   // v15.4: gprsConnect อาจใช้เวลา
          modem.gprsConnect(g_cfgApn, GPRS_USER, GPRS_PASS);
          vTaskDelay(pdMS_TO_TICKS(5000));
          esp_task_wdt_reset();
          gprs = modem.isGprsConnected();
          // [minimal-fix, keepalive-starvation] same rationale as above --
          // services keepalive after this 5s blocking call too.
          mqttClient.loop();
        }

        g_network.gprsConnected = gprs;

        // Update signal quality
        if (gprs) {
          g_network.signalQuality = modem.getSignalQuality();
          if (g_network.signalQuality != 99) {
            g_network.signalPercent = map(g_network.signalQuality, 0, 31, 0, 100);
          } else {
            g_network.signalPercent = 0;
          }
        }

        // Try to connect MQTT if GPRS is up but MQTT is down
        bool mqttConnSnap9 = mqttClient.connected();
        // [v16.5] Section 7 Item 4 — cache write (design v16.5 §4.2, row #9)
        if (xSemaphoreTake(mutexSystemState, pdMS_TO_TICKS(5)) == pdTRUE) {
          g_systemState.mqttConnected = mqttConnSnap9;
          xSemaphoreGive(mutexSystemState);
        }
        if (gprs && !mqttConnSnap9 &&
            (now - lastConnectionAttempt > mqttBackoffMs)) {

          lastConnectionAttempt = now;

          // v15.4 Fix B: Full TLS reset ก่อน attempt ทุกครั้ง
          // ล้าง stale cipher state จาก session เดิม (IP เปลี่ยน → old context)
          esp_task_wdt_reset();   // Fix A: reset WDT ก่อน TLS operation
          gsmClient.resetTLS();
          esp_task_wdt_reset();   // Fix A: reset WDT หลัง TLS reset

          Serial.printf("[CORE 1] Connecting MQTT (mTLS, ID=%s) backoff=%lus attempt#%u...\n",
                        MQTT_CLIENT_ID,
                        (unsigned long)(mqttBackoffMs / 1000),
                        (unsigned)mqttFailCount + 1);

          esp_task_wdt_reset();   // Fix A: reset WDT ก่อน TLS handshake (อาจใช้เวลา ~2s)
          bool connected = mqttClient.connect(MQTT_CLIENT_ID);
          esp_task_wdt_reset();   // Fix A: reset WDT หลัง TLS handshake

          if (connected) {
            Serial.println("[CORE 1] MQTT Connected (mTLS) +");
            // Fix C: reset backoff เมื่อ connect สำเร็จ
            mqttBackoffMs = BACKOFF_MIN;
            mqttFailCount = 0;

            // [Commit 7A] Re-subscribe every successful (re)connect --
            // subscriptions do not survive a reconnect in this library.
            // Infrastructure only: mqttCommandCallback() above still only
            // logs; nothing is enqueued or acted on as a result of this
            // subscription in this commit.
            if (mqttClient.subscribe(g_mqttTopicCommand)) {
              Serial.printf("[CORE 1] MQTT subscribed -> %s\n", g_mqttTopicCommand);
            } else {
              Serial.printf("[CORE 1] MQTT subscribe FAILED -> %s\n", g_mqttTopicCommand);
            }
          } else {
            int mqttErr = mqttClient.lastError();
            int mqttRc  = mqttClient.returnCode();
            mqttFailCount++;

            // Fix C: exponential backoff -- 30s → 60s → 120s → 300s (max)
            mqttBackoffMs = min(mqttBackoffMs * 2, (uint32_t)BACKOFF_MAX);

            Serial.printf("[CORE 1] MQTT connect failed | err=%d rc=%d fail#%u next_retry=%lus\n",
                          mqttErr, mqttRc,
                          (unsigned)mqttFailCount,
                          (unsigned long)(mqttBackoffMs / 1000));

            if (mqttErr == LWMQTT_NETWORK_FAILED_CONNECT)
              Serial.println("[CORE 1]   -> Layer: TCP connect failed (modem/network issue)");
            else if (mqttErr == LWMQTT_NETWORK_TIMEOUT)
              Serial.println("[CORE 1]   -> Layer: TLS handshake timed out");
            else if (mqttRc == 5)
              Serial.println("[CORE 1]   -> Layer: MQTT AUTH refused (check CN=client ID)");
            else if (mqttRc == 4)
              Serial.println("[CORE 1]   -> Layer: MQTT bad credentials");
            else
              Serial.println("[CORE 1]   -> Layer: TLS handshake failed (see [TLS] lines above)");

            // Fix A: ถ้า fail มากกว่า 3 ครั้งต่อเนื่อง ให้ reset modem ด้วย
            if (mqttFailCount >= 3) {
              Serial.printf("[CORE 1] %u consecutive MQTT failures -- reinit modem\n",
                            (unsigned)mqttFailCount);
              esp_task_wdt_reset();
              modem.restart();
              esp_task_wdt_reset();
              modemConnectGPRS();
              esp_task_wdt_reset();
              mqttFailCount = 0;
              mqttBackoffMs = BACKOFF_MIN;
            }
          }
        }


        // Update modem state
        bool mqttConnSnap14 = mqttClient.connected();
        // [v16.5] Section 7 Item 4 — cache write (design v16.5 §4.2, row #14)
        if (xSemaphoreTake(mutexSystemState, pdMS_TO_TICKS(5)) == pdTRUE) {
          g_systemState.mqttConnected = mqttConnSnap14;
          xSemaphoreGive(mutexSystemState);
        }
        if (gprs && mqttConnSnap14) {
          g_network.modemState = MODEM_STATE_GPRS_CONNECTED;
        } else if (gprs) {
          g_network.modemState = MODEM_STATE_GPRS_CONNECTED;
        } else if (network) {
          g_network.modemState = MODEM_STATE_REGISTERED;
        } else {
          g_network.modemState = MODEM_STATE_SEARCHING;
        }

        // Log state changes
        if (gprs != lastGprsState) {
          Serial.printf("[CORE 1] GPRS: %s\n", gprs ? "CONNECTED" : "DISCONNECTED");
          // Trigger time sync when GPRS comes back up
          if (gprs && !lastGprsState) {
            Serial.println("[CORE 1] GPRS reconnected -- scheduling NTP sync");
            // [v16.5j] Forces checkAndSyncTime()'s NEXT call to fire
            // immediately, bypassing whatever interval wait was in
            // progress. Was "lastSyncMillis = 0" -- that field no longer
            // drives scheduling (see lastCheckMillis), so this line had
            // silently stopped forcing anything once the gate moved off
            // lastSyncMillis. Resetting lastCheckMillis instead is also
            // more correct than the original: lastSyncMillis (last
            // successful sync) is left untouched, so a GPRS blip no longer
            // makes the device look like it has "never synced" -- only the
            // next check's timing is affected, exactly as intended.
            g_timeSync.lastCheckMillis = 0;
          }
          lastGprsState = gprs;
        }

        if (mqttClient.connected() != lastMqttState) {
          bool nowConnected = mqttClient.connected();
          // [v16.5] Section 7 Item 4 — cache write (design v16.5 §4.2, rows #15/16)
          if (xSemaphoreTake(mutexSystemState, pdMS_TO_TICKS(5)) == pdTRUE) {
            g_systemState.mqttConnected = nowConnected;
            xSemaphoreGive(mutexSystemState);
          }
          Serial.printf("[CORE 1] MQTT: %s\n", nowConnected ? "CONNECTED" : "DISCONNECTED");

          // เมื่อ reconnect สำเร็จ: แจ้ง Serial ว่ามี backlog รอ replay เท่าไหร่
          if (nowConnected && !lastMqttState) {
            // อ่าน count โดยไม่ล็อก mutex ยาว (volatile read, ค่าประมาณ)
            uint8_t pendingSnap = g_telemBufCount;
            if (pendingSnap > 0) {
              Serial.printf("[TelemBuf] MQTT reconnected -- %u slot(s) pending replay\n",
                            (unsigned)pendingSnap);
            }
          }
          lastMqttState = nowConnected;
        }
      }
    }

    // -- mqttClient.loop() ??? iteration = ??? 100ms --
    // ????????????? publish ????? process ACK/PINGREQ ??????
    bool mqttConnSnap17 = mqttClient.connected();
    // [v16.5] Section 7 Item 4 — cache write (design v16.5 §4.2, row #17)
    if (xSemaphoreTake(mutexSystemState, pdMS_TO_TICKS(5)) == pdTRUE) {
      g_systemState.mqttConnected = mqttConnSnap17;
      xSemaphoreGive(mutexSystemState);
    }
    if (mqttConnSnap17) {
      mqttClient.loop();
    }

    // Periodic NTP time sync check
    if (g_network.gprsConnected) {
      checkAndSyncTime();
    }

    // Get current sensor data
    // [v16.5.4] Improvement 2: single atomic snapshot copy (ONE mutex take)
    // replaces the previous g_vibData + g_systemState pair of copies.
    if (xSemaphoreTake(mutexVibData, pdMS_TO_TICKS(5)) == pdTRUE) {
      memcpy(&localSnap, &g_telemSnapshot, sizeof(TelemetrySnapshot));
      xSemaphoreGive(mutexVibData);
    }

    // state-change boost, and minimum floor (3 s).
    uint32_t publishInterval;
    switch (localSnap.alarm_level) {
      case STATE_WARNING:  publishInterval = 10000; break;
      case STATE_CRITICAL: publishInterval =  5000; break;
      default:             publishInterval = 30000; break;
    }

    // ── Telemetry Ring Buffer Replay ────────────────────────────────────────
    // Rate-limited: ส่ง 1 slot ต่อ taskNetwork() loop iteration (≈100ms/loop)
    // หยุดทันทีถ้า MQTT หลุด กลาง burst (replayTelemBuf() returns false)
    // ล็อก mutex เฉพาะ peek + pop (ดู replayTelemBuf()) — ไม่บล็อก loop นาน
    // ────────────────────────────────────────────────────────────────────────
    bool mqttConnSnap19 = mqttClient.connected();
    // [v16.5] Section 7 Item 4 — cache write (design v16.5 §4.2, row #19)
    if (xSemaphoreTake(mutexSystemState, pdMS_TO_TICKS(5)) == pdTRUE) {
      g_systemState.mqttConnected = mqttConnSnap19;
      xSemaphoreGive(mutexSystemState);
    }
    if (mqttConnSnap19 && g_telemBufCount > 0) {
      replayTelemBuf();
      vTaskDelay(pdMS_TO_TICKS(75));  // 75ms delay between replayed messages
                                      // 120 slots × 75ms ≈ 9s burst สูงสุด (ไม่ flood broker)
    }

    // ── MQTT Outbound Queue Drain (Section 7 Item 9, design v16.5 §4.1) ──────
    // Consumer side of the dormant queue added in Item 3. No producer is wired
    // yet (Analytics still calls mqttClient.publish() directly, per Item 6 not
    // being implemented in this commit) -- queueMqttOutboundTrend is therefore
    // always empty here, xQueueReceive always returns pdFALSE immediately, and
    // this block's body cannot execute. Rate-limited to 1 message per
    // taskNetwork() iteration, analogous to the g_telemBuf replay above.
    // Publishes exactly like the existing /sensor, /status, /trend logic.
    // ──────────────────────────────────────────────────────────────────────────
    if (mqttConnSnap19 && queueMqttOutboundTrend != NULL) {
      MqttOutboundMsg_t outMsg;
      if (xQueueReceive(queueMqttOutboundTrend, &outMsg, 0) == pdPASS) {
        // [Result Consumer, Commit 2] MQTT_OUTBOUND_TOPIC_EVENT added -- routes
        // handleFifoCaptureCompletion()'s /event payload the same way TREND
        // already routes to /trend; no other change to this drain's logic.
        const char* outTopic = (outMsg.topic_id == MQTT_OUTBOUND_TOPIC_TREND)  ? g_mqttTopicTrend
                              : (outMsg.topic_id == MQTT_OUTBOUND_TOPIC_EVENT) ? g_mqttTopicEvent
                              : NULL;
        if (outTopic != NULL) {
#ifdef DEBUG_MQTT_TIMING
          uint32_t t0_pub2 = millis();
#endif
          bool pubOk2 = mqttClient.publish(outTopic, outMsg.payload, (int)outMsg.len, false, outMsg.qos);
#ifdef DEBUG_MQTT_TIMING
          dbgLogMqttPublish(outTopic, outMsg.len, outMsg.qos, pubOk2,
                            mqttClient.lastError(), millis() - t0_pub2);
#endif
          if (pubOk2) {
            g_network.publishCount++;
            Serial.printf("[MQTT] Outbound queue published -> %s (%u B)\n", outTopic, (unsigned)outMsg.len);
          } else {
            g_network.publishFailures++;
            Serial.printf("[MQTT] Outbound queue publish FAILED -> %s (err=%d)\n", outTopic, mqttClient.lastError());
          }
        }
      }
    }

    // -- Publish telemetry (?? FreeRTOS task ???????? ???????? ISR) --
    bool mqttConnSnap20 = mqttClient.connected();
    // [v16.5] Section 7 Item 4 — cache write (design v16.5 §4.2, rows #20/#22 — same
    // if/else-if evaluation, no intervening mqttClient call, so one write covers both)
    if (xSemaphoreTake(mutexSystemState, pdMS_TO_TICKS(5)) == pdTRUE) {
      g_systemState.mqttConnected = mqttConnSnap20;
      xSemaphoreGive(mutexSystemState);
    }
    if (mqttConnSnap20 && (now - lastPublish >= publishInterval)) {
      if (localSnap.vib.valid) {
        // -- Normal telemetry --
        // [v16.5.4] publish from the atomic snapshot
        if (publishTelemetry(&localSnap)) {
          lastPublish = now;
          g_network.publishCount++;
          g_network.lastPublishTime = now;
        } else {
          g_network.publishFailures++;
        }
      } else {
        // -- Sensor offline -- publish status alert ??? (??? 30s) --
        static uint32_t lastOfflinePublish = 0;
        if (now - lastOfflinePublish >= 30000) {
          StaticJsonDocument<296> offlineDoc;
          offlineDoc["plant"]              = PLANT_ID;
          offlineDoc["machine_id"]         = MACHINE_ID;
          offlineDoc["sensor_id"]          = SENSOR_ID;
          // Patch C: pipeline stage fields for /sensor topic consistency
          offlineDoc["stage"]              = "sensor";
          offlineDoc["execution_location"] = "edge";
          offlineDoc["sensor_status"]      = "OFFLINE";
          offlineDoc["error_count"]        = g_sensorErrors;
          offlineDoc["uptime_s"]           = millis() / 1000;
          // Timestamp
          if (g_rtcValid) {
            DateTime rtcNow; RTC_NOW_SAFE(rtcNow);  // [v16.3g]
            char tsBuf[25];
            snprintf(tsBuf, sizeof(tsBuf), "%04d-%02d-%02dT%02d:%02d:%02dZ",
                     rtcNow.year(), rtcNow.month(),  rtcNow.day(),
                     rtcNow.hour(), rtcNow.minute(), rtcNow.second());
            offlineDoc["ts"] = tsBuf;
          }
          char offlineJson[296];
          size_t szOffline = serializeJson(offlineDoc, offlineJson, sizeof(offlineJson));
          // Patch C: publish to /sensor topic (not /vibration) so consumers
          // receive offline alerts on the same topic as normal sensor data.
          // Explicit length avoids relying on strlen()/null-termination --
          // safe even if the document were ever truncated to fit the buffer.
#ifdef DEBUG_MQTT_TIMING
          uint32_t t0_pub3 = millis();
#endif
          bool pubOk3 = mqttClient.publish(g_mqttTopicSensor, offlineJson, (int)szOffline, false, MQTT_QOS);
#ifdef DEBUG_MQTT_TIMING
          dbgLogMqttPublish(g_mqttTopicSensor, szOffline, MQTT_QOS, pubOk3,
                            mqttClient.lastError(), millis() - t0_pub3);
#endif
          if (pubOk3) {
            lastOfflinePublish = now;
            g_network.publishCount++;
            Serial.printf("[MQTT] ! Offline alert published: %s\n", offlineJson);
          } else {
            g_network.publishFailures++;
          }
        }
      }
    } else if (!mqttConnSnap20 && (now - lastPublish >= publishInterval)) {
      // MQTT offline แต่ถึงเวลา publish -- บันทึกลง ring buffer แทน
      // [v16.5.4] buffer from the atomic snapshot (same fields as before)
      if (localSnap.vib.valid) {
        pushTelemBuf(&localSnap.vib, localSnap.alarm_level);
        lastPublish = now;  // advance timer เพื่อ push ทุก publishInterval (ไม่ push ซ้ำ)
      }
    }

    // ── V14.4: Maintenance reset MQTT audit (queued from taskButtonHandler) ──
    // Drains up to QUEUE_SIZE_MAINT events per loop tick (should be ≤1 normally).
    {
      MaintenanceEvent_t mEvt;
      while (xQueueReceive(queueMaintEvent, &mEvt, 0) == pdPASS) {
        bool mqttConnSnap23 = mqttClient.connected();
        // [v16.5] Section 7 Item 4 — cache write (design v16.5 §4.2, row #23)
        if (xSemaphoreTake(mutexSystemState, pdMS_TO_TICKS(5)) == pdTRUE) {
          g_systemState.mqttConnected = mqttConnSnap23;
          xSemaphoreGive(mutexSystemState);
        }
        if (mqttConnSnap23) {
          char tsBuf[32];
          if (mEvt.rtcValid) {
            snprintf(tsBuf, sizeof(tsBuf),
                     "%04d-%02d-%02dT%02d:%02d:%02d",
                     mEvt.year, mEvt.month, mEvt.day,
                     mEvt.hour, mEvt.minute, mEvt.second);
          } else {
            snprintf(tsBuf, sizeof(tsBuf), "millis:%lu", mEvt.triggerMillis);
          }

          StaticJsonDocument<256> evDoc;
          char evBuf[280];

          evDoc["plant_id"]   = PLANT_ID;
          evDoc["machine_id"] = MACHINE_ID;
          evDoc["event"]      = "maintenance_reset";
          evDoc["timestamp"]  = tsBuf;
          evDoc["state"]      = "WARMUP";

          size_t szEvt = serializeJson(evDoc, evBuf, sizeof(evBuf));

#ifdef DEBUG_MQTT_TIMING
          uint32_t t0_pub4 = millis();
#endif
          bool pubOk4 = mqttClient.publish(g_mqttTopicEvent, evBuf, (int)szEvt, false, MQTT_QOS);
#ifdef DEBUG_MQTT_TIMING
          dbgLogMqttPublish(g_mqttTopicEvent, szEvt, MQTT_QOS, pubOk4,
                            mqttClient.lastError(), millis() - t0_pub4);
#endif
          if (pubOk4) {
            g_network.publishCount++;
            Serial.println("[MAINT] MQTT audit event published +");
          } else {
            g_network.publishFailures++;
            Serial.println("[MAINT] MQTT audit event FAILED");
          }
        } else {
          // MQTT not connected — event is dropped (already dequeued).
          // Acceptable: maintenance reset is a manual operator action.
          Serial.println("[MAINT] MQTT audit event dropped (not connected)");
        }
      }
    }

    // ── Fault Latch v3: replay pending event on /status ──────────────────────
    // v3 hardened: snapshot-copy pattern -- take mutexFaultLatch only long
    // enough to copy g_fl + g_flCount into locals, release it immediately,
    // then build/publish the JSON from the local snapshot. The mutex is
    // NEVER held across mqttClient.publish() or any other blocking I/O.
    {
      bool     snapPending;
      uint32_t snapTs;
      uint8_t  snapCode;
      float    snapRms;
      float    snapKurt;
      uint32_t snapCount;

      if (xSemaphoreTake(mutexFaultLatch, pdMS_TO_TICKS(20)) == pdTRUE) {
        snapPending = g_fl.pending;
        snapTs      = g_fl.ts;
        snapCode    = g_fl.code;
        snapRms     = g_fl.rms;
        snapKurt    = g_fl.kurtosis;
        snapCount   = g_flCount;
        xSemaphoreGive(mutexFaultLatch);
      } else {
        // Mutex busy -- skip replay this cycle, retry next loop. Never block
        // here: this task still needs to service modem/MQTT/WDT below.
        snapPending = false;
        snapTs      = 0;
        snapCode    = FL_EVT_NONE;
        snapRms     = 0.0f;
        snapKurt    = 0.0f;
        snapCount   = g_flCount;
        Serial.println("[LATCH] replay snapshot mutex timeout — retry next loop");
      }

      bool mqttConnSnap25 = mqttClient.connected();
      // [v16.5] Section 7 Item 4 — cache write (design v16.5 §4.2, row #25)
      if (xSemaphoreTake(mutexSystemState, pdMS_TO_TICKS(5)) == pdTRUE) {
        g_systemState.mqttConnected = mqttConnSnap25;
        xSemaphoreGive(mutexSystemState);
      }
      if (mqttConnSnap25 && snapPending) {

        char flTs[26] = {};
        const bool flTsKnown = (snapTs >= FL_TS_MIN_VALID);
        if (flTsKnown) {
          DateTime flEv(snapTs);
          snprintf(flTs, sizeof(flTs), "%04d-%02d-%02dT%02d:%02d:%02dZ",
                   flEv.year(), flEv.month(), flEv.day(),
                   flEv.hour(), flEv.minute(), flEv.second());
        }

        int         snapAlarmCode  = 0;
        const char* snapAlarmLevel = "NORMAL";
        // [M1A-CLEANUP] Was 100. This default is what the fault-latch-pending
        // replay payload publishes whenever the motor is NOT RUNNING, since
        // the assignment below is gated on motor_state == 2. Leaving it at 100
        // reproduced the same contradiction just fixed in computeHealthScore():
        // a "healthy" score asserted on a message carrying no vibration
        // evidence. UNKNOWN is the honest default; it is published only, never
        // compared against FL_HEALTH_LOW_THOLD (see checkAndLatchFault(),
        // which takes latchHealth, not this value).
        int         snapHealth     = HEALTH_SCORE_UNKNOWN;
        {
          // [v16.5.4] Improvement 2: read the atomic snapshot (ONE mutex take)
          // instead of separate g_vibData + g_systemState copies, and use its
          // health_score instead of re-deriving it here (formula now has a
          // single owner: captureTelemetrySnapshot()). Same values, same gates.
          TelemetrySnapshot flSnap = {};
          if (xSemaphoreTake(mutexVibData, pdMS_TO_TICKS(5)) == pdTRUE) {
            memcpy(&flSnap, &g_telemSnapshot, sizeof(TelemetrySnapshot));
            xSemaphoreGive(mutexVibData);
          }
          if (flSnap.motor_state == 2) {
            snapAlarmCode  = (flSnap.alarm_level == STATE_CRITICAL) ? 2 :
                             (flSnap.alarm_level == STATE_WARNING)  ? 1 : 0;
            // [Phase 3J] Shared pure formatter. Equivalent in both branches:
            // inside this if the motor is RUNNING so the state maps exactly as
            // before, and outside it snapAlarmLevel keeps its "NORMAL" default,
            // which is what the helper returns for a non-RUNNING motor.
            snapAlarmLevel = alarmLevelStr(flSnap.alarm_level, flSnap.motor_state);
            snapHealth     = flSnap.health_score;
          }
        }

        char flNowTs[26] = "not_available";
        if (g_rtcValid) {
          DateTime rtcNow; RTC_NOW_SAFE(rtcNow);  // [v16.3g]
          snprintf(flNowTs, sizeof(flNowTs), "%04d-%02d-%02dT%02d:%02d:%02dZ",
                   rtcNow.year(), rtcNow.month(), rtcNow.day(),
                   rtcNow.hour(), rtcNow.minute(), rtcNow.second());
        }

        StaticJsonDocument<1024> flDoc;
        flDoc["plant"]              = PLANT_ID;
        flDoc["machine_id"]         = MACHINE_ID;
        flDoc["sensor_id"]          = SENSOR_ID;
        flDoc["stage"]              = "status";
        flDoc["execution_location"] = "edge";
        flDoc["alarm_code"]         = snapAlarmCode;
        flDoc["alarm_level"]        = snapAlarmLevel;
        flDoc["health_score"]       = snapHealth;
        flDoc["timestamp"]          = flNowTs;
        flDoc["fault_latch_pending"]= true;
        flDoc["fault_event"]        = faultEventStr(snapCode);
        flDoc["fault_severity"]     = faultSeverity(snapCode);
        if (flTsKnown) {
          flDoc["fault_ts"]         = snapTs;
          flDoc["fault_ts_iso"]     = flTs;
          flDoc["fault_ts_unknown"] = false;
        } else {
          flDoc["fault_ts"]         = (uint32_t)0;
          flDoc["fault_ts_iso"]     = (char*)nullptr;
          flDoc["fault_ts_unknown"] = true;
        }
        flDoc["fault_rms"]          = roundf(snapRms  * 100.0f)  / 100.0f;
        // [Phase 1] fault_kurtosis REMOVED from /decision -- no topic carries a
        // kurtosis field any more. snapKurt is still read from the latch record
        // so the NVS layout and its recovery path stay untouched.
        flDoc["fault_latch_count"]  = snapCount;

        char   flBuf[1024];
        size_t flSz = serializeJson(flDoc, flBuf, sizeof(flBuf));
        if (flSz == 0 || flSz >= sizeof(flBuf) - 1) {
          Serial.printf("[LATCH] JSON overflow sz=%u — skipping\n", (unsigned)flSz);
        } else {
#ifdef DEBUG_MQTT_TIMING
          uint32_t t0_pub5 = millis();
#endif
          bool pubOk5 = mqttClient.publish(g_mqttTopicDecision, flBuf, (int)flSz,
                                 false, MQTT_QOS);
#ifdef DEBUG_MQTT_TIMING
          dbgLogMqttPublish(g_mqttTopicDecision, flSz, MQTT_QOS, pubOk5,
                            mqttClient.lastError(), millis() - t0_pub5);
#endif
          if (pubOk5) {
            g_network.publishCount++;
            clearFaultLatchNVS();
            Serial.printf("[LATCH] PUBLISH+CLEAR %s\n", flBuf);
          } else {
            g_network.publishFailures++;
            Serial.println("[LATCH] PUBLISH failed — retry next loop");
          }
        }
      }
    }
    // ── End Fault Latch replay ────────────────────────────────────────────────

#ifdef DEBUG_MODEM_DIAG
    // [v16.5b] Issue #2 Phase 1 -- read-only modem diagnostics.
    // Runs at the very END of a loop iteration, after every publish/replay
    // path above has completed, and only inside a quiet window: at least
    // MODEM_DIAG_QUIET_MS since the last publish AND at least that much
    // before the next one is due. This keeps the probe out of the
    // publish/PUBACK window whose timing Issue #2 is measuring.
    // Consequence (accepted, per Issue #2 decision): the second condition
    // requires publishInterval >= 2 * MODEM_DIAG_QUIET_MS, so this samples in
    // the NORMAL 30 s cadence only -- never in WARNING (10 s) or CRITICAL
    // (5 s). No fallback is provided by design.
    {
      static uint32_t lastModemDiag = 0;
      uint32_t dnow     = millis();
      uint32_t sincePub = dnow - lastPublish;
      if ((dnow - lastModemDiag >= MODEM_DIAG_PERIOD_MS) &&
          (sincePub >= MODEM_DIAG_QUIET_MS) &&
          (publishInterval > sincePub) &&
          ((publishInterval - sincePub) >= MODEM_DIAG_QUIET_MS)) {
        modemDiagPoll(mqttConnSnap20, g_network.gprsConnected,
                      sincePub, publishInterval);
        lastModemDiag = millis();
      }
    }
#endif

    // -- 100ms sleep -- ให้ FreeRTOS scheduler ทำงาน tasks อื่น --
    // v15.4: WDT reset ย้ายไปอยู่ที่ TOP ของ loop + ทุก blocking operation
    // บรรทัดนี้เป็น safety net สำหรับ normal operation path
    esp_task_wdt_reset();
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

/**
 * Task 5: Button Input Handling (CORE 1, Priority 2)
 * Non-blocking button debounce and event detection
 * Supports 2 buttons: SELECT (PIN_BUTTON) and ENTER (PIN_BUTTON_ENTER)
 */
void taskButtonHandler(void* parameter) {
  // SELECT button state tracking
  bool lastStateSelect = HIGH;
  uint32_t pressStartSelect = 0;
  bool processedSelect = false;

  // ENTER button state tracking
  bool lastStateEnter = HIGH;
  uint32_t pressStartEnter = 0;
  bool processedEnter = false;
  // [Production Trigger, Commit 6] ENTER double-click detector state --
  // deliberately NOT reset on press-edge (unlike pressStartEnter/
  // processedEnter above): a double-click must be recognized ACROSS the
  // gap between the first click's release and the second click's press, so
  // this state has to survive that gap. Persists across multiple press/
  // release cycles. Armed (awaitingSecondEnterClick=true,
  // firstEnterReleaseMs=now) on every qualifying release, in the release
  // branch below; consumed (checked and cleared) on the NEXT press-edge,
  // in the press branch below -- window is release-to-press, per the
  // revised design, so the second click's own hold duration never affects
  // recognition.
  bool     awaitingSecondEnterClick = false;
  uint32_t firstEnterReleaseMs      = 0;

  Serial.println("[CORE 1] Button task started (SELECT + ENTER)");

  while (1) {
    bool currentStateSelect = digitalRead(PIN_BUTTON);
    bool currentStateEnter = digitalRead(PIN_BUTTON_ENTER);
    uint32_t now = millis();

    // ===== SELECT BUTTON HANDLING =====
    // Detect press
    if (currentStateSelect == LOW && lastStateSelect == HIGH) {
      pressStartSelect = now;
      processedSelect = false;
    }

    // Detect hold duration
    if (currentStateSelect == LOW && !processedSelect) {
      uint32_t duration = now - pressStartSelect;

      if (duration >= 8000) {
        if (xSemaphoreTake(mutexSystemState, pdMS_TO_TICKS(10)) == pdTRUE) {
          if (g_systemState.state != STATE_CRITICAL) {
            g_systemState.state = STATE_MAINTENANCE;
            g_systemState.buzzerActive = false;
            xSemaphoreGive(mutexSystemState);  // release before suspend

            // ── Freeze processing to avoid race condition ──
            if (taskHandleAnalytics != NULL) {
              vTaskSuspend(taskHandleAnalytics);
            }

            // --- Reset Evidence ---

            // --- Reset Trend Buffers [Phase2] ---
            // g_trendBuf / g_buf1s / g_buf10s / g_buf60s / g_slopeVar_* /
            // g_velPeakHold / legacy EMA / slot accumulators are all gone; the
            // temperature ring is the only raw trend storage left here.
            memset(g_tempBuf, 0, sizeof(g_tempBuf));
            g_tempHead  = 0;
            g_tempCount = 0;

            memset(&g_trendResult, 0, sizeof(g_trendResult));

            g_anaPublishCnt = 0;

            // Resume processing
            if (taskHandleAnalytics != NULL) {
              vTaskResume(taskHandleAnalytics);
            }

            // ── Enter Warm-up phase ──
            g_systemState.state = STATE_WARMUP;
            g_warmupStartTs    = millis();
            g_trendFreqFlushed  = false;

            // ── Queue maintenance event for Network task to publish (MQTT audit) ──
            // All JSON/MQTT work is done in taskNetwork to keep Button stack lean.
            {
              MaintenanceEvent_t mEvt;
              mEvt.triggerMillis = g_warmupStartTs;
              mEvt.rtcValid      = g_rtcValid;
              if (g_rtcValid) {
                // V14.7: take mutexI2C — taskDisplayUpdate holds it during OLED
                // writes (10 Hz); without the mutex, Wire transactions can interleave
                // causing bus corruption or a wrong timestamp in the audit event.
                // Timeout 100ms is safe: OLED render is < 20ms per frame.
                if (xSemaphoreTake(mutexI2C, pdMS_TO_TICKS(100)) == pdTRUE) {
                  DateTime rtcNow   = rtc.now();
                  xSemaphoreGive(mutexI2C);
                  mEvt.year   = rtcNow.year();
                  mEvt.month  = rtcNow.month();
                  mEvt.day    = rtcNow.day();
                  mEvt.hour   = rtcNow.hour();
                  mEvt.minute = rtcNow.minute();
                  mEvt.second = rtcNow.second();
                } else {
                  // I2C mutex timeout — fill with zeros; audit event is still
                  // queued (timestamp will show 0000-00-00 00:00:00, acceptable).
                  mEvt.year = mEvt.month = mEvt.day = 0;
                  mEvt.hour = mEvt.minute = mEvt.second = 0;
                  Serial.println("[MAINT] mutexI2C timeout — timestamp zeroed");
                }
              } else {
                mEvt.year = mEvt.month = mEvt.day = 0;
                mEvt.hour = mEvt.minute = mEvt.second = 0;
              }
              xQueueSend(queueMaintEvent, &mEvt, 0);  // non-blocking, drop if full
            }

            Serial.println("[MAINT] Reset complete → WARMUP (MQTT event queued)");

          } else {
            xSemaphoreGive(mutexSystemState);
          }
        }
        processedSelect = true;
      }
    }

    // Detect release (short press)
    if (currentStateSelect == HIGH && lastStateSelect == LOW) {
      uint32_t duration = now - pressStartSelect;

      if (duration >= 50 && duration < 800 && !processedSelect) {
        // Short press - Change page
        if (xSemaphoreTake(mutexSystemState, pdMS_TO_TICKS(10)) == pdTRUE) {
          g_systemState.currentPage = (DisplayPage_t)((g_systemState.currentPage + 1) % PAGE_MAX);
          Serial.printf("[CORE 1] SELECT: Page %d\n", g_systemState.currentPage);
          xSemaphoreGive(mutexSystemState);
        }
      }
    }

    lastStateSelect = currentStateSelect;

    // ===== ENTER BUTTON HANDLING =====
    // Detect press
    if (currentStateEnter == LOW && lastStateEnter == HIGH) {
      pressStartEnter = now;
      processedEnter = false;

      // [Production Trigger, Commit 6] OPERATOR_BUTTON double-click fire
      // check -- evaluated HERE, at this press-edge, not at this press's
      // eventual release: the window is first-click-release ->
      // second-click-press, so recognition happens the instant the second
      // press begins and is never affected by how long that second press
      // is then held (matches common double-click UX -- e.g. a mouse
      // double-click does not require a fast second release, only a fast
      // second press). Purely additive -- does not alter
      // pressStartEnter/processedEnter's reset above in any way.
      if (awaitingSecondEnterClick &&
          (now - firstEnterReleaseMs) <= ENTER_DOUBLECLICK_WINDOW_MS) {
        // Second click's press arrived within the window -- valid
        // double-click. Consume the gesture (reset immediately) BEFORE
        // touching the queue, so this pair can never be reused by a
        // subsequent third press.
        awaitingSecondEnterClick = false;

        // [ADR-0006 D-1, Phase 2A] The OPERATOR_BUTTON FIFO producer that
        // stood here has been REMOVED -- periodic SCHEDULED capture is the
        // sole FIFO initiator, so the double-click can no longer start a
        // capture. The GESTURE itself is unchanged: the same debounce, the
        // same ENTER_DOUBLECLICK_WINDOW_MS window, and the same
        // consume-before-acting reset above all still run, and the operator
        // still gets Serial acknowledgement that the gesture was recognised.
        Serial.println("[CORE 1] ENTER: double-click recognised -- operator-initiated FIFO "
                       "capture disabled; periodic capture is the sole initiator (ADR-0006)");
      }
    }

    // Detect hold duration (long press for alarm ACK)
    if (currentStateEnter == LOW && !processedEnter) {
      uint32_t duration = now - pressStartEnter;

      if (duration >= 2000) {
        // Long press (2s) - Acknowledge alarm
        if (xSemaphoreTake(mutexSystemState, pdMS_TO_TICKS(10)) == pdTRUE) {
          g_systemState.alarmAcknowledged = true;
          g_systemState.buzzerActive = false;
          Serial.println("[CORE 1] ENTER: Alarm ACK (2s)");
          xSemaphoreGive(mutexSystemState);
        }
        processedEnter = true;
      }
    }

    // Detect release (short press - can be used for other functions)
    if (currentStateEnter == HIGH && lastStateEnter == LOW) {
      uint32_t duration = now - pressStartEnter;

      if (duration >= 50 && duration < 800 && !processedEnter) {
        // Short press ENTER - Currently unused, can add functionality
        Serial.println("[CORE 1] ENTER: Short press");
        // Future: Toggle logging, reset stats, etc.

        // [Production Trigger, Commit 6] Arms the double-click window for
        // a POTENTIAL next press (recognition/firing itself happens at
        // the next press-edge, above -- see that block's own comment).
        // Reuses this branch's own existing condition (50-800ms,
        // !processedEnter) as the double-click's debounce/qualifying-click
        // floor -- deliberately not a separate mechanism, since a
        // double-click's constituent clicks must themselves be genuine
        // short presses, and this is already this button's established
        // debounce window. Unconditional re-arm on every qualifying
        // release (no "already awaiting" branch needed here anymore: the
        // press-edge check above already consumed+cleared the flag if
        // THIS click was itself a successful second click, so by the time
        // execution reaches here that case is already handled). Does not
        // alter the Serial.println() or "Future:" comment above in any way
        // -- purely additive.
        awaitingSecondEnterClick = true;
        firstEnterReleaseMs      = now;
      }
    }

    lastStateEnter = currentStateEnter;

    vTaskDelay(pdMS_TO_TICKS(10));  // 10ms polling
  }
}

/**
 * Task 6: Buzzer Control (CORE 1, Priority 1)
 * Lowest priority, non-critical
 */
void taskBuzzerControl(void* parameter) {
  bool beepState = false;
  uint32_t lastBeep = 0;

  Serial.println("[CORE 1] Buzzer task started");

  while (1) {
    bool buzzerActive = false;
    bool acknowledged = false;
    MachineState_t state;

    if (xSemaphoreTake(mutexSystemState, pdMS_TO_TICKS(5)) == pdTRUE) {
      buzzerActive = g_systemState.buzzerActive;
      acknowledged = g_systemState.alarmAcknowledged;
      state = g_systemState.state;
      xSemaphoreGive(mutexSystemState);
    }

    if (buzzerActive && !acknowledged) {
      uint32_t now = millis();
      uint32_t interval = (state == STATE_CRITICAL) ? 250 : 500;

      if (now - lastBeep >= interval) {
        beepState = !beepState;
        digitalWrite(PIN_BUZZER, beepState ? HIGH : LOW);
        lastBeep = now;
      }
    } else {
      digitalWrite(PIN_BUZZER, LOW);
    }

    vTaskDelay(pdMS_TO_TICKS(50));  // 50ms check
  }
}

// ============================================================================
// DISPLAY RENDERING FUNCTIONS (CORE 1)
// ============================================================================

/**
 * drawSensorOfflineScreen -- ????????? sensor ????? / ?????????? Modbus
 * ???????????????????????????? sensor ???????? online
 */
void drawSensorOfflineScreen() {
  char buf[32];

  // Header -- ????????????????????????????
  u8g2.drawBox(0, 0, 128, 14);          // ?????????
  u8g2.setDrawColor(0);                 // ????? (invert)
  u8g2.setFont(u8g2_font_6x10_tr);
  u8g2.drawStr(12, 10, "! SENSOR OFFLINE !");
  u8g2.setDrawColor(1);                 // ??????????

  // Divider
  u8g2.drawHLine(0, 16, 128);

  // Line 2: ??????????
  u8g2.setFont(u8g2_font_6x10_tr);
  u8g2.drawStr(0, 28, "Modbus: No Response");

  // Line 3: error count
  snprintf(buf, sizeof(buf), "RS485 Err: %lu", g_sensorErrors);
  u8g2.drawStr(0, 40, buf);

  // Line 4: uptime ??????????????? board ??? alive
  snprintf(buf, sizeof(buf), "Uptime: %lu s", millis() / 1000);
  u8g2.drawStr(0, 52, buf);

  // Line 5: hint
  u8g2.setFont(u8g2_font_5x7_tr);
  u8g2.drawStr(0, 63, "Check power / RS485 wiring");
}

void drawMachineScreen(VibrationData_t* data) {
  char buf[32];

  // Line 1: Header (y=10)
  u8g2.setFont(u8g2_font_6x10_tr);
  u8g2.drawStr(0, 10, MACHINE_NAME);
  u8g2.drawStr(100, 10, "[1/3]");

  // Line 2: STATUS (y=22)
  u8g2.setFont(u8g2_font_ncenB08_tr);
  u8g2.drawStr(15, 24, "STATUS: NORMAL");

  // Line 3: velocity RMS (y=38) - LARGE
  // [Phase2] FIFO/DSP velocity_rms_overall. "--" when no valid, fresh capture --
  // never a zero standing in for an unavailable measurement.
  u8g2.setFont(u8g2_font_ncenB10_tr);
  {
    float vAll = 0.0f;
    if (displayVelocity(&vAll, NULL, NULL, NULL)) snprintf(buf, sizeof(buf), "%.2f", vAll);
    else                                          snprintf(buf, sizeof(buf), "--");
  }
  uint8_t w = u8g2.getStrWidth(buf);
  u8g2.drawStr((128 - w) / 2, 40, buf);

  // Unit
  u8g2.setFont(u8g2_font_5x7_tr);
  u8g2.drawStr((128 - w) / 2 + w + 2, 40, "mm/s");

  // Line 4-5: Thresholds (y=50)
  u8g2.setFont(u8g2_font_6x10_tr);
  // [Phase2] show the thresholds the alarm actually uses (FIFO/DSP velocity),
  // not the retired legacy VRMS numbers.
  snprintf(buf, sizeof(buf), "WARN %.1f", VIB_WARNING_MMS);
  u8g2.drawStr(0, 52, buf);

  snprintf(buf, sizeof(buf), "CRIT %.1f", VIB_CRITICAL_MMS);
  u8g2.drawStr(80, 52, buf);

  // Line 6: Footer (y=64)
  u8g2.setFont(u8g2_font_5x7_tr);
  snprintf(buf, sizeof(buf), "T:%.1fC", data->temperature);
  u8g2.drawStr(0, 64, buf);

  // Show 4G status instead of MQTT
  if (g_network.gprsConnected) {
    snprintf(buf, sizeof(buf), "4G:%d%%", g_network.signalPercent);
  } else {
    snprintf(buf, sizeof(buf), "4G:--");
  }
  u8g2.drawStr(80, 64, buf);
}

// [Phase2] The one display-side read of the FIFO/DSP velocity carrier.
// Returns false when no valid, fresh capture exists -- callers then print "--"
// rather than a zero that would read as a real measurement of no vibration.
// Same carrier, same mutex, same freshness deadline as readVelocityForAlarm().
static bool displayVelocity(float* outOverall, float* outX, float* outY, float* outZ) {
  if (mutexVelCarrier == NULL) return false;
  VelocityCarrier_t vc;
  if (xSemaphoreTake(mutexVelCarrier, pdMS_TO_TICKS(5)) != pdTRUE) return false;
  vc = g_velCarrier;                    // whole-struct copy: no torn read
  xSemaphoreGive(mutexVelCarrier);
  if (!vc.valid || vc.timestampMs == 0u ||
      (uint32_t)(millis() - vc.timestampMs) > VIB_VELOCITY_MAX_AGE_MS_TBD) return false;
  if (outOverall) *outOverall = vc.overall;
  if (outX) *outX = vc.x;
  if (outY) *outY = vc.y;
  if (outZ) *outZ = vc.z;
  return true;
}

// [Phase 3F] The one read of the FIFO/DSP dominant frequency, and from here on
// the CANONICAL source of Fx/Fy/Fz. Same carrier, same mutex and the same
// VIB_VELOCITY_MAX_AGE_MS_TBD deadline as displayVelocity(), so "stale" keeps
// meaning one thing across the whole firmware.
//
// Fx/Fy/Fz are now a PRESENTATION NAME for dominant_frequency_x/y/z_hz. They
// are no longer the sensor's own frequency registers (0x44-0x46); those are
// still read and still available on the SensorData path, but only as a
// comparison reference -- see the [Phase 3F] note at the /sensor builder.
//
// Validity is PER AXIS because the DSP reports it per axis: an axis whose
// spectrum yielded no reportable peak comes back false and its caller must
// print "--" / omit the key rather than publish a number. The return value is
// the capture-level gate (carrier present, valid and fresh); a false return
// leaves every ok* false.
//
// Deliberately NOT gated on motor_state or RPM. The dominant frequency is a
// property of the captured waveform alone -- it is computed before, and
// independently of, the 1x/2x harmonic block that needs rpmAtCapture. Gating it
// on motor state would make Fx unavailable exactly where it is most useful:
// a machine whose speed signal is missing.
static bool dspDominantFreq(float* outX, bool* okX,
                            float* outY, bool* okY,
                            float* outZ, bool* okZ) {
  if (okX) *okX = false;
  if (okY) *okY = false;
  if (okZ) *okZ = false;
  if (mutexVelCarrier == NULL) return false;
  VelocityCarrier_t vc;
  if (xSemaphoreTake(mutexVelCarrier, pdMS_TO_TICKS(5)) != pdTRUE) return false;
  vc = g_velCarrier;                    // whole-struct copy: no torn read
  xSemaphoreGive(mutexVelCarrier);
  if (!vc.valid || vc.timestampMs == 0u ||
      (uint32_t)(millis() - vc.timestampMs) > VIB_VELOCITY_MAX_AGE_MS_TBD) return false;
  if (outX) *outX = vc.domFreqX;
  if (outY) *outY = vc.domFreqY;
  if (outZ) *outZ = vc.domFreqZ;
  if (okX) *okX = vc.domFreqXValid;
  if (okY) *okY = vc.domFreqYValid;
  if (okZ) *okZ = vc.domFreqZValid;
  return true;
}

// [Phase 3G] The one read of the FIFO/DSP crest factor, and from here on the
// CANONICAL source of the crest_factor product field. Same carrier, same mutex
// and the same VIB_VELOCITY_MAX_AGE_MS_TBD deadline as displayVelocity() and
// dspDominantFreq(), so "stale" keeps meaning one thing firmware-wide.
//
// Deliberately does NOT test vc.valid: that flag is velocity validity, and the
// crest factor is an acceleration quantity carrying its own flag. Freshness is
// still enforced -- a stale acceleration figure misleads exactly as much as a
// stale velocity one.
//
// The sensor's own crest-factor registers (0x47/0x53/0x5F) are still read and
// still available on the SensorData path, but only as a validation reference;
// nothing downstream treats them as the product crest_factor any more.
static bool dspCrestFactor(float* outCf) {
  if (mutexVelCarrier == NULL) return false;
  VelocityCarrier_t vc;
  if (xSemaphoreTake(mutexVelCarrier, pdMS_TO_TICKS(5)) != pdTRUE) return false;
  vc = g_velCarrier;                    // whole-struct copy: no torn read
  xSemaphoreGive(mutexVelCarrier);
  if (!vc.crestFactorValid || vc.timestampMs == 0u ||
      (uint32_t)(millis() - vc.timestampMs) > VIB_VELOCITY_MAX_AGE_MS_TBD) return false;
  if (outCf) *outCf = vc.crestFactor;
  return true;
}

void drawWarningScreen(VibrationData_t* data, bool blink) {
  char buf[32];

  u8g2.setFont(u8g2_font_6x10_tr);
  u8g2.drawStr(0, 10, MACHINE_NAME);
  u8g2.drawStr(100, 10, "[1/3]");

  if (blink) {
    u8g2.setFont(u8g2_font_ncenB08_tr);
    u8g2.drawStr(30, 26, "! WARNING !");
  }

  float wAll = 0.0f, wX = 0.0f, wY = 0.0f, wZ = 0.0f;
  const bool wOk = displayVelocity(&wAll, &wX, &wY, &wZ);

  u8g2.setFont(u8g2_font_ncenB10_tr);
  if (wOk) snprintf(buf, sizeof(buf), "%.2f", wAll);
  else     snprintf(buf, sizeof(buf), "--");
  u8g2.drawStr(40, 40, buf);
  u8g2.setFont(u8g2_font_5x7_tr);
  u8g2.drawStr(90, 40, "mm/s");

  u8g2.setFont(u8g2_font_6x10_tr);
  snprintf(buf, sizeof(buf), "CRIT: %.1f", VIB_CRITICAL_MMS);
  u8g2.drawStr(20, 54, buf);

  u8g2.setFont(u8g2_font_5x7_tr);
  if (wOk) snprintf(buf, sizeof(buf), "X:%.1f Y:%.1f Z:%.1f", wX, wY, wZ);
  else     snprintf(buf, sizeof(buf), "X:-- Y:-- Z:--");
  u8g2.drawStr(0, 64, buf);
}

void drawCriticalScreen(VibrationData_t* data, bool blink) {
  char buf[32];

  if (blink) {
    u8g2.setFont(u8g2_font_ncenB08_tr);
    u8g2.drawStr(30, 16, "! CRITICAL !");
  }

  u8g2.setFont(u8g2_font_ncenB10_tr);
  {
    float cAll = 0.0f;
    if (displayVelocity(&cAll, NULL, NULL, NULL)) snprintf(buf, sizeof(buf), "%.2f", cAll);
    else                                          snprintf(buf, sizeof(buf), "--");
  }
  u8g2.drawStr(40, 30, buf);

  u8g2.setFont(u8g2_font_5x7_tr);
  snprintf(buf, sizeof(buf), "LIMIT: %.1f mm/s", VIB_CRITICAL_MMS);
  u8g2.drawStr(20, 40, buf);

  u8g2.setFont(u8g2_font_ncenB08_tr);
  u8g2.drawStr(20, 52, "ACTION: STOP");

  u8g2.setFont(u8g2_font_5x7_tr);
  snprintf(buf, sizeof(buf), "T:%.1fC", data->temperature);
  u8g2.drawStr(0, 64, buf);
  u8g2.drawStr(100, 64, "[1/3]");
}

void drawAxisScreen(VibrationData_t* data) {
  char buf[20];

  // === Column headers (bold font) ===
  u8g2.setFont(u8g2_font_6x12_tf);  // bold-ish for headers
  u8g2.drawStr(3, 10, "VEL.(mm/s)");
  u8g2.drawStr(75, 10, "FRE.(Hz)");

  // Underline below each header
  u8g2.drawHLine(3, 13, 58);   // left column underline
  u8g2.drawHLine(70, 13, 55);  // right column underline

  // Vertical divider (full height below header)
  u8g2.drawVLine(64, 0, 64);

  // === Data rows (normal font) ===
  u8g2.setFont(u8g2_font_6x10_tr);

  // [v16.3af] gate เหมือน publishTelemetry -- ไม่ใช่ RUNNING = ค่า sensor เป็น
  // noise-floor/garbage (de-glitch filter v16.3x ทำงานเฉพาะตอน RUNNING) -> แสดง 0
  bool isRunningDisp = (data->motor_state == 2);

  // Row 1: VX / FX   (y=26)
  float dAll = 0.0f, dX = 0.0f, dY = 0.0f, dZ = 0.0f;
  const bool dOk = isRunningDisp && displayVelocity(&dAll, &dX, &dY, &dZ);

  // [Phase 3F] FX/FY/FZ now read the FIFO/DSP dominant frequency instead of the
  // sensor's frequency registers. The labels are unchanged on purpose -- the
  // operator sees the same screen, only the number behind it got a defensible
  // source. An axis with no reportable peak prints "--", the same convention
  // VX/VY/VZ already use, so a missing measurement can never be misread as a
  // measured 0 Hz.
  float fX = 0.0f, fY = 0.0f, fZ = 0.0f;
  bool  fXok = false, fYok = false, fZok = false;
  dspDominantFreq(&fX, &fXok, &fY, &fYok, &fZ, &fZok);

  if (dOk) snprintf(buf, sizeof(buf), "VX = %03.2f", dX);
  else     snprintf(buf, sizeof(buf), "VX = --");
  u8g2.drawStr(5, 26, buf);
  if (fXok) snprintf(buf, sizeof(buf), "FX = %02.0f", fX);
  else      snprintf(buf, sizeof(buf), "FX = --");
  u8g2.drawStr(70, 26, buf);

  // Row 2: VY / FY   (y=37)
  if (dOk) snprintf(buf, sizeof(buf), "VY = %03.2f", dY);
  else     snprintf(buf, sizeof(buf), "VY = --");
  u8g2.drawStr(5, 37, buf);
  if (fYok) snprintf(buf, sizeof(buf), "FY = %02.0f", fY);
  else      snprintf(buf, sizeof(buf), "FY = --");
  u8g2.drawStr(70, 37, buf);

  // Row 3: VZ / FZ   (y=48)
  if (dOk) snprintf(buf, sizeof(buf), "VZ = %03.2f", dZ);
  else     snprintf(buf, sizeof(buf), "VZ = --");
  u8g2.drawStr(5, 48, buf);
  if (fZok) snprintf(buf, sizeof(buf), "FZ = %02.0f", fZ);
  else      snprintf(buf, sizeof(buf), "FZ = --");
  u8g2.drawStr(70, 48, buf);

  // Row 4: MAX / [2/3]  (y=60)
  if (dOk) snprintf(buf, sizeof(buf), "MAX= %03.2f", dAll);
  else     snprintf(buf, sizeof(buf), "MAX= --");
  u8g2.drawStr(5, 60, buf);
  u8g2.drawStr(98, 60, "[2/3]");
}

void drawNetworkScreen() {
  char buf[32];

  u8g2.setFont(u8g2_font_6x10_tr);
  u8g2.drawStr(0, 10, "4G NETWORK");
  u8g2.drawStr(100, 10, "[3/3]");

  // Modem status
  const char* modemStatus = "UNKNOWN";
  switch (g_network.modemState) {
    case MODEM_STATE_OFF: modemStatus = "OFF"; break;
    case MODEM_STATE_INITIALIZING: modemStatus = "INIT..."; break;
    case MODEM_STATE_SEARCHING: modemStatus = "SEARCH"; break;
    case MODEM_STATE_REGISTERED: modemStatus = "REG OK"; break;
    case MODEM_STATE_GPRS_CONNECTING: modemStatus = "GPRS..."; break;
    case MODEM_STATE_GPRS_CONNECTED: modemStatus = "ONLINE"; break;
    case MODEM_STATE_ERROR: modemStatus = "ERROR"; break;
  }
  snprintf(buf, sizeof(buf), "Status: %s", modemStatus);
  u8g2.drawStr(5, 22, buf);

  // Operator and Signal
  if (g_network.gprsConnected) {
    snprintf(buf, sizeof(buf), "Op: %.10s", g_network.operatorName);
    u8g2.drawStr(5, 32, buf);

    snprintf(buf, sizeof(buf), "Signal: %d%% (CSQ:%d)",
             g_network.signalPercent, g_network.signalQuality);
    u8g2.drawStr(5, 42, buf);
  } else {
    u8g2.drawStr(5, 32, "Op: ---");
    u8g2.drawStr(5, 42, "Signal: ---");
  }

  // MQTT status
  // [v16.5] Section 7 Item 7 (design v16.5 §3.3, §4.2) — read via cache instead
  // of touching mqttClient directly; DisplayUpdate is not the owner task.
  snprintf(buf, sizeof(buf), "MQTT: %s",
           getMqttConnectedCached() ? "CONN" : "DISC");
  u8g2.drawStr(5, 52, buf);

  // Publish stats + NTP status
  snprintf(buf, sizeof(buf), "Tx:%d %s",
           g_network.publishCount,
           g_timeSync.synced ? "NTP:OK" : "NTP:--");
  u8g2.drawStr(5, 62, buf);
}

// ============================================================================
// [Phase2] REMOVED: aggLinRegSlope(), pushAggBuf(), computeRmsVariance()
// ============================================================================
// These three helpers served only the g_buf1s/g_buf10s/g_buf60s cascade and the
// RPM-adaptive slot machinery, all of which fed legacy VRMS trend outputs
// (slope_1s/10s/60s, max_rms_10min/60min, stddev_1min, slope_var_*). The trend
// pipeline is now VibHistory -> VibEma / VibWindow / VibSlope / VibTtw.

// ============================================================================
// GENERIC LINEAR REGRESSION -- linRegSlope()  [v16.6a]
// ============================================================================
// Least-squares slope over the most recent `windowSamples` entries of a
// circular buffer, addressed via an accessor callback (X = sample index
// 0..n-1, Y = getValue(idx)) instead of a flat float array -- lets callers
// read directly out of whatever storage they already have (a struct array's
// field, a plain float array, ...) with no intermediate copy.
// Generalizes the regression math that was previously inlined in calcTrend()
// for temp_slope. Reused for:
//   - temp_slope:    intervalSec=1.0f -- no time-scaling, preserves the
//                    original "degC per sample" output exactly.
//   - current_slope: intervalSec=CURRENT_SAMPLE_INTERVAL_S -- normalizes to
//                    "per second" since current is sampled at a different,
//                    fixed 500ms cadence.
// Returns 0.0f if fewer than 2 samples are available.
// ============================================================================
static float linRegSlope(uint16_t bufHead, uint16_t bufCount, uint16_t bufSize,
                          uint16_t windowSamples, float intervalSec,
                          float (*getValue)(uint16_t idx)) {
  uint16_t n = (bufCount < windowSamples) ? bufCount : windowSamples;
  if (n < 2 || bufSize == 0) return 0.0f;
  uint16_t startIdx = (bufHead + bufSize - n) % bufSize;
  double sumX=0.0, sumX2=0.0, sumY=0.0, sumXY=0.0;
  for (uint16_t i = 0; i < n; i++) {
    uint16_t idx = (startIdx + i) % bufSize;
    double x = (double)i;
    double y = (double)getValue(idx);
    sumX  += x;
    sumX2 += x * x;
    sumY  += y;
    sumXY += x * y;
  }
  double denom = (double)n * sumX2 - sumX * sumX;
  if (denom == 0.0) return 0.0f;
  float slopePerSample = (float)(((double)n * sumXY - sumX * sumY) / denom);
  return (intervalSec > 0.0f) ? (slopePerSample / intervalSec) : slopePerSample;
}

// Accessors for linRegSlope() -- trivial index->value lookups into the two
// buffers it's used against. [v16.6a]
static float tempBufAccessor(uint16_t idx)      { return g_tempBuf[idx]; }
static float currentBufAccessor(uint16_t idx)   { return g_currentBuf[idx]; }

// ============================================================================
// TEMPERATURE TREND -- calcTemperatureTrend()   [Phase2]
// ============================================================================
// Least-squares slope of the temperature history ring. Split out of the former
// monolithic calcTrend(), which also produced the legacy VRMS rms_slope /
// trend_dir / spike_count / ttw_hours and the freq_ratio drift/alert -- all
// removed. What is left has one input (TEMPERATURE register 0x40) and one
// consumer (/device-health temp_slope), so it gets its own function.
// Reads g_tempBuf, written by Core 0; float reads are atomic on Xtensa.
// ============================================================================
static void calcTemperatureTrend() {
  const uint16_t snapHead  = g_tempHead;
  const uint16_t snapCount = g_tempCount;

  if (snapCount >= TEMP_MIN_SAMPLES) {
    const uint16_t n = (snapCount < TEMP_WINDOW_SAMPLES) ? snapCount : TEMP_WINDOW_SAMPLES;
    const float tempSlope = linRegSlope(snapHead, snapCount, TEMP_BUF_SIZE,
                                        TEMP_WINDOW_SAMPLES, 1.0f, tempBufAccessor);
    g_trendResult.temp_slope     = roundf(tempSlope * 100000.0f) / 100000.0f;
    g_trendResult.window_samples = n;
  } else {
    g_trendResult.temp_slope     = 0.0f;
    g_trendResult.window_samples = snapCount;
  }
}

// ============================================================================
// CURRENT TREND -- calcCurrentTrend()   [Phase2, was inline in calcTrend()]
// ============================================================================
// CTR4A01 current slope, 500 ms cadence. Independent buffer and readiness from
// the temperature ring above (different source, different sample rate); shares
// only the generic linRegSlope() utility. No thresholds, no direction
// classification -- Motor State logic is untouched.
// ============================================================================
static void calcCurrentTrend() {
  if (g_currentCount >= CURRENT_MIN_SAMPLES) {
    g_trendResult.current_slope = linRegSlope(g_currentHead, g_currentCount, CURRENT_BUF_SIZE,
                                               CURRENT_WINDOW_SAMPLES, CURRENT_SAMPLE_INTERVAL_S,
                                               currentBufAccessor);
  } else {
    g_trendResult.current_slope = 0.0f;
  }
}

// ============================================================================
// MQTT PUBLISHING (CORE 1)
// ============================================================================

// [v16.5.4] Improvement 2: consumes the atomic TelemetrySnapshot instead of a
// separately-copied (VibrationData_t, MachineState_t) pair -- all published
// fields now belong to one capture. data/state below alias the snapshot's
// members so the rest of this function is textually unchanged.
bool publishTelemetry(const TelemetrySnapshot* snap) {
  if (!mqttClient.connected()) return false;

  const VibrationData_t* data  = &snap->vib;
  const MachineState_t   state = snap->alarm_level;

  // [v16.6j] CURRENT TELEMETRY FRESHNESS -- data->current_valid (VibrationData_t)
  // is a raw per-250ms-tick flag: true only on the exact tick CTR4A01 was
  // sampled (~500ms cadence), false every other tick even though current_a is
  // still a perfectly fresh measurement. That flag is deliberately kept as-is
  // for its OTHER consumer (the trend-buffer append gate in taskStateMachine,
  // which needs exactly this "new sample this tick" meaning to avoid
  // double-counting one 500ms sample across multiple 250ms ticks in
  // calcTrend()'s regression) -- so it is not repurposed here.
  // The EXPORTED current_valid instead reflects genuine age-based freshness,
  // reusing the same g_lastCurrentSampleMs timestamp and the same
  // CURRENT_EVIDENCE_MAX_AGE_MS (5 s) window buildMotorStateEvidence() already
  // uses -- no new state, just read at the export boundary. current_a itself
  // is untouched: it already always holds the last successfully measured
  // value and is never fabricated to 0.
  const uint32_t currentAgeMs   = millis() - g_lastCurrentSampleMs;
  const bool     currentFresh   = (g_lastCurrentSampleMs != 0) &&
                                   (currentAgeMs < CURRENT_EVIDENCE_MAX_AGE_MS);
  const float    currentAgeS    = currentAgeMs / 1000.0f;

  // ── Shared pre-computes ───────────────────────────────────────────────────
  // v16.0: gate alarm + health ด้วย MOTOR_RUNNING
  // STARTING/STOPPING: RMS transient สูง → ไม่ประเมิน alarm/health
  // ส่ง alarmCode=0 / alarmLevel="NORMAL" / healthScore=100 แทน
  // [v16.5.4] healthScore is now read from the snapshot (computed once, at
  // capture time, on Core 0, by computeHealthScore() -- same formula/gate as
  // before). alarmCode/alarmLevel logic below is untouched.
  int alarmCode;
  const char* alarmLevel;
  int healthScore = snap->health_score;

  if (data->motor_state == 2) {  // MOTOR_RUNNING เท่านั้น
    alarmCode   = (state == STATE_CRITICAL) ? 2 :
                  (state == STATE_WARNING)  ? 1 : 0;
    // [v16.3l] ปิด bearing escalation — kurtosis ไม่เสถียรพอสำหรับ V1
    // alarmCode ใช้ RMS state machine อย่างเดียว
    // [v16.3l] ปิด bearing health penalty — ใช้ RMS-based health อย่างเดียว
  } else {
    alarmCode   = 0;
  }
  // [Phase 3J] Same mapping as before, now expressed once and shared with the
  // buffered-replay serializer. Behaviour is unchanged: RUNNING maps the state
  // machine's verdict, anything else reports NORMAL.
  alarmLevel = alarmLevelStr(state, data->motor_state);

  // [Phase2] currentPeak / reportedRms / reportedVx / reportedVy / reportedVz
  // REMOVED together with g_velPeakHold and VibrationData_t.rms_* -- there is
  // no legacy vibration number left to gate or report.

  // v15.1: ใช้ cf_max (max ของทั้ง 3 แกน) แทน cf_x เพียงแกนเดียว
  // sensor คำนวณจาก raw 16KHz FIFO ภายใน chip:  CF = Peak_acc / RMS_acc
  // [v16.5] gate ด้วย MOTOR_RUNNING เหมือน RMS/peak/kurtosis ด้านบน --
  // ขณะ STOPPED/STARTING/STOPPING ค่า CF จาก sensor เป็น noise-floor/garbage
  // ที่ไม่ผ่าน deglitch (deglitch ทำงานเฉพาะ motor_state==2) -> ต้อง gate เป็น 0
  // [Phase 3G] crest_factor is now vector_peak / acceleration_rms_overall,
  // computed from the RAW FIFO waveform -- not the sensor's own CF registers.
  // Registers 0x47/0x53/0x5F are still read and still live in data->cf_x/y/z,
  // kept deliberately as a REFERENCE ONLY for the comparison log below.
  //
  // No motor_state gate is needed or wanted here: FIFO capture is admitted only
  // while the motor is RUNNING, so a fresh carrier entry already implies it.
  float crestFactor   = 0.0f;
  bool  crestFactorOk = dspCrestFactor(&crestFactor);
  if (crestFactorOk) {
    crestFactor = roundf(crestFactor * 100.0f) / 100.0f;
  }

  // [Phase 3G] Migration comparison, reference vs canonical. The two are NOT
  // expected to agree and are never forced toward each other: the sensor
  // derives its CF from an internal 16 kHz stream while this one sees 2 kHz, so
  // impulse energy above 1 kHz is invisible here and the canonical figure reads
  // systematically lower.
  Serial.printf("[CF-SRC] canonical(dsp) %s | reference(reg 0x47/53/5F) "
                "cf_max=%.2f x=%.2f y=%.2f z=%.2f\n",
                crestFactorOk ? String(crestFactor, 2).c_str() : "--",
                data->cf_max, data->cf_x, data->cf_y, data->cf_z);

  // [Phase 1] BEARING ALERT REMOVED AT ORIGIN.
  //
  // bearing_alert was a second machine-condition opinion derived from the
  // sensor's own kurtosis registers, published beside alarm_level while being
  // computed by a different engine from different physics. Measured over 24 h
  // the two disagreed in 69% of samples -- 1045 of them showing a bearing
  // fault while alarm_level read NORMAL.
  //
  // It decided nothing: it never fed alarmCode/alarmLevel (bearing escalation
  // was already disabled in v16.3l), never reached the API or the dashboard,
  // and never triggered a notification. Phase 1 makes alarm_level the single
  // machine-condition verdict, so the competing opinion stops being published.
  //
  // The bearing DIAGNOSIS is not replaced -- it is withdrawn. Nothing here
  // asserts a healthy bearing either; the claim is simply no longer made.

  // v16.0: kurtosis valid เฉพาะ MOTOR_RUNNING
  // ขณะ STOPPED/STARTING/STOPPING: noise floor → kurtosis สูงผิดปกติ (ไม่มีความหมาย)
  // [Phase 1] kurtosisValid / kx / ky / kz / kmax / kaxis REMOVED with the
  // kurtosis pipeline. No topic carries a kurtosis field any more.

  // dominant_vibration_axis: axis carrying the highest velocity RMS.
  // [Phase2] Re-sourced from the FIFO/DSP velocity carrier -- it previously read
  // the legacy VRMS per-axis rms_x/y/z, which no longer exist. When the carrier
  // holds no valid capture the axis is reported as "-" (unknown); no axis is
  // guessed and no zero is passed off as a measurement.
  const char* domVibAxis = "-";
  {
    VelocityCarrier_t vcAxis;
    bool vcAxisOk = false;
    if (mutexVelCarrier != NULL &&
        xSemaphoreTake(mutexVelCarrier, pdMS_TO_TICKS(5)) == pdTRUE) {
      vcAxis   = g_velCarrier;      // whole-struct copy: no torn read
      vcAxisOk = true;
      xSemaphoreGive(mutexVelCarrier);
    }
    if (vcAxisOk && vcAxis.valid && vcAxis.timestampMs != 0u &&
        (uint32_t)(millis() - vcAxis.timestampMs) <= VIB_VELOCITY_MAX_AGE_MS_TBD) {
      domVibAxis = (vcAxis.x >= vcAxis.y && vcAxis.x >= vcAxis.z) ? "X"
                 : (vcAxis.y >= vcAxis.z)                         ? "Y"
                                                                  : "Z";
    }
  }

  // [Phase 3F] freq_x/y/z are now the FIFO/DSP dominant frequency -- derived
  // from the RAW waveform through the same FFT that produces velocity RMS, not
  // from the sensor's frequency registers.
  //
  // The registers 0x44-0x46 are still read every poll and still live in
  // data->freq_x/y/z. They are kept deliberately, as a REFERENCE ONLY, so the
  // two sources can be compared on real hardware; nothing downstream treats
  // them as Fx/Fy/Fz any more.
  float freqX = 0.0f, freqY = 0.0f, freqZ = 0.0f;
  bool  freqXok = false, freqYok = false, freqZok = false;
  {
    float dfx = 0.0f, dfy = 0.0f, dfz = 0.0f;
    if (dspDominantFreq(&dfx, &freqXok, &dfy, &freqYok, &dfz, &freqZok)) {
      freqX = roundf(dfx * 10.0f) / 10.0f;
      freqY = roundf(dfy * 10.0f) / 10.0f;
      freqZ = roundf(dfz * 10.0f) / 10.0f;
    }
  }

  // [Phase 3F] Migration comparison, reference vs canonical. Logged at the
  // /sensor cadence rather than per FIFO capture because the register values
  // only exist on the Core 0 sensor path -- reaching them from the capture
  // consumer would mean new cross-core plumbing for a diagnostic, which is not
  // worth the coupling. The values are never forced toward each other.
  Serial.printf("[FREQ-SRC] canonical(dsp) x=%s y=%s z=%s | "
                "reference(reg 0x44-46) x=%.1f y=%.1f z=%.1f\n",
                freqXok ? String(freqX, 1).c_str() : "--",
                freqYok ? String(freqY, 1).c_str() : "--",
                freqZok ? String(freqZ, 1).c_str() : "--",
                data->freq_x, data->freq_y, data->freq_z);

  // [Phase2] calcTrend() split into two dedicated functions; trendDirStr removed
  // with the legacy trend_dir it formatted.
  calcTemperatureTrend();
  calcCurrentTrend();

  // Shared timestamp (built once, used in all three payloads)
  char tsBuf[26] = "not_available";
  if (g_rtcValid) {
    DateTime now; RTC_NOW_SAFE(now);  // [v16.3g]
    snprintf(tsBuf, sizeof(tsBuf), "%04d-%02d-%02dT%02d:%02d:%02dZ",
             now.year(), now.month(), now.day(),
             now.hour(), now.minute(), now.second());
  }

  bool success = false;

  // ──────────────────────────────────────────────────────────────────────────
  // PUBLISH 1 of 3 — /sensor
  // Stage  : raw acquisition
  // Fields : sensor readings + motor context + harmonic feature extraction
  // v15.0 additions: vel_peak_x/y/z, vel_peak, kurtosis_x
  // v15.3 additions: reset_reason, reboot_count
  // Size   : doc 960 B (stack), char buf[1000], JSON ~530 B estimated
  // ──────────────────────────────────────────────────────────────────────────
  {
    StaticJsonDocument<960> s;
    s["plant"]               = PLANT_ID;
    s["machine_id"]          = MACHINE_ID;
    s["sensor_id"]           = SENSOR_ID;
    s["stage"]               = "sensor";
    s["execution_location"]  = "edge";
    s["sensor_status"]       = "ONLINE";
    // [Phase1] LEGACY VRMS/VPEAK REMOVED from /sensor. The following fields
    // no longer exist on this topic:
    //   rms, vx, vy, vz            -- legacy VRMS-register metric
    //   peak                       -- legacy VPEAK max-hold (g_velPeakHold)
    //   peak_velocity_x/y/z        -- reg 0x3A-0x3C peak velocity
    //   vibration_source_legacy    -- provenance tag for the above
    //   deglitch_count             -- VRMS de-glitch counter (still printed
    //                                 in the serial status report; the field
    //                                 dies with the de-glitch itself)
    // The FIFO-DSP velocity figures (velocity_rms_* on the accel_rms event
    // and on /vibration) are now the only vibration source a consumer sees.
    // Verified before removal: nothing on the VPS subscribes to /sensor --
    // Node-RED subscribes only to /vibration, /trend and /device-health, and
    // the API reads InfluxDB, never MQTT.

    s["temp"]  = round(data->temperature *  10) /  10.0f;
    s["rpm"]   = data->rpm;

    // [v16.6i] CT-compensated engineering current (see compensateCurrent()).
    // [v16.6j] current_valid is now AGE-BASED freshness (currentFresh, computed
    // above from g_lastCurrentSampleMs / CURRENT_EVIDENCE_MAX_AGE_MS) rather
    // than "was this the exact tick CTR4A01 was sampled" -- current_valid=true
    // means the last successful CTR4A01 read is within the 5 s freshness
    // window; current_valid=false means either no successful read has ever
    // happened, or the last one is older than that window. current_a is
    // unaffected by this change -- it always holds the last successfully
    // measured value and is never fabricated to 0. current_age_s gives the
    // exact elapsed time so downstream systems can distinguish fresh /
    // temporarily stale / never-received without a new API field per state.
    s["current_a"]     = round(data->current_a * 100) / 100.0f;
    s["current_valid"] = currentFresh;
    s["current_age_s"] = round(currentAgeS * 10) / 10.0f;

    // Harmonic feature extraction
    // [Phase 3F] Published ONLY when that axis produced a reportable peak.
    // An omitted key means "not measured"; it must never be filled with a 0
    // that a consumer could read as a measured 0 Hz. Same contract the
    // dominant_frequency_* and velocity_rms_* keys already follow.
    // [Phase 3H] freq_x/y/z MOVED to /vibration. They are canonical machine
    // vibration, and /vibration is the topic that carries machine vibration
    // truth; /sensor now describes the sensor and the device only. Moved, not
    // copied -- one canonical value, one destination.
    // [Phase2] freq_ratio_x/y/z REMOVED with the frequency-ratio pipeline.

    // v15.1: CF ครบ 3 แกน + max
    // [Phase 3G] Published ONLY when the capture produced one. An omitted key
    // means "not measured"; it is never filled with a 0 a consumer could read
    // as a measured crest factor of zero, and never falls back to the legacy
    // register value.
    // [Phase 3H] crest_factor MOVED to /vibration, for the same reason as
    // freq_x/y/z above.
    // [Phase 3G] cf_x/cf_y/cf_z are NO LONGER PUBLISHED. They were the
    // sensor's own per-axis crest factors (registers 0x47/0x53/0x5F), and
    // Phase 1 exports one crest factor only -- the FIFO/DSP
    // vector_peak / acceleration_rms_overall published as crest_factor
    // above. The registers are still read and still reach data->cf_x/y/z,
    // but now solely as a validation reference in the [CF-SRC] log; no
    // product consumer reads them any more.

    // [Phase 1] kurtosis_x/y/z/max/axis/valid REMOVED from /sensor.
    // [Phase 3H] dominant_vibration_axis MOVED to /vibration -- it is derived
    // from the canonical velocity RMS, so it belongs beside the values it is
    // derived from.
    // [Phase 1] bearing_alert REMOVED from /sensor.

    s["motor_state"]           = data->motor_state;
    s["rotation_signal_ok"]    = data->prox;
    s["operating_hours_total"] = data->runtime_hour;

    // v15.3: Reset reason — ช่วยวินิจฉัย unexpected reboot
    s["reset_reason"]  = g_resetReasonStr;   // "POWER_ON" / "BROWNOUT" / "PANIC" etc.
    s["reboot_count"]  = g_rebootCount;      // สะสมทุก boot (NVS persistent)

    s["timestamp"]   = tsBuf;
    s["time_synced"] = g_timeSync.synced;

    char buf[1000];

    // [DESIGN-0004] Requirement 6: measure actual size via ArduinoJson measureJson(),
    // not an estimate -- reports the true serialized size even if serializeJson() below
    // truncates against the fixed char buf[] below.
    // NOTE: library is ArduinoJson v7.4.3 -- StaticJsonDocument<960> here is a deprecated
    // compatibility shim; its .capacity() would only echo the literal "960", not a real
    // pool limit (v7's JsonDocument allocates dynamically), so it is not logged here.
    // memoryUsage() reports the JsonDocument's actual current allocation instead.
    // [Code review fix] Compiled out entirely in production -- zero runtime cost
    // when undefined, since neither the measureJson() call nor the printf exist
    // in the compiled binary at all.
    // Enable for diagnostic builds using: -DDEBUG_JSON_SIZE
    // No source modification is required.
#ifdef DEBUG_JSON_SIZE
    size_t measuredSensorSize = measureJson(s);
    Serial.printf("[DESIGN-0004] /sensor measureJson()=%u B (doc memoryUsage=%u B, buf cap=%u B)\n",
                  (unsigned)measuredSensorSize, (unsigned)s.memoryUsage(),
                  (unsigned)sizeof(buf));
#endif

    size_t szSensor = serializeJson(s, buf, sizeof(buf));
    // [M1B-7 E5/F1] FAIL CLOSED. เดิม guard นี้ warn แล้ว "ส่งต่อ" buffer ที่ถูกตัด
    // ออกไปจริง -- consumer ได้ JSON พังและ JSON.parse() throw ทันที ซึ่งแย่กว่า
    // ไม่ได้รับ message เสียอีก (ไม่มีทางแยกจาก outage ได้) เทียบกับ
    // enqueueMqttOutbound() ที่ REJECT payload เกินขนาดมาตลอด -- direct-publish
    // path จึงปลอดภัยน้อยกว่า queued path มาโดยตลอด บรรทัดนี้ปิดช่องว่างนั้น
    // short-circuit && ทำให้ mqttClient.publish() ไม่ถูกเรียกเลยเมื่อ JSON ไม่ครบ
    const bool sensorJsonOk = (szSensor > 0 && szSensor < sizeof(buf) - 1);
    if (!sensorJsonOk) {
      Serial.printf("[WARN] /sensor JSON truncated -- NOT PUBLISHED! sz=%u buf=%u\n",
                    (unsigned)szSensor, (unsigned)sizeof(buf));
    }
    bool connBefore6 = mqttClient.connected();  // [v16.5c] sampled before the call -- see dbgLogMqttPublish
#ifdef DEBUG_MQTT_TIMING
    uint32_t t0_pub6 = millis();
#endif
    bool pubOk6 = sensorJsonOk &&
                  mqttClient.publish(g_mqttTopicSensor, buf, (int)szSensor, false, MQTT_QOS);
#ifdef DEBUG_MQTT_TIMING
    dbgLogMqttPublish(g_mqttTopicSensor, szSensor, MQTT_QOS, pubOk6,
                      mqttClient.lastError(), millis() - t0_pub6, connBefore6);
#endif
    if (!sensorJsonOk)
      { /* [M1B-7] warning printed above -- ไม่ report เป็น publish failure */ }
    else if (pubOk6)
      Serial.printf("[MQTT] /sensor %u B\n", (unsigned)szSensor);
    else if (!connBefore6)
      Serial.printf("[MQTT] /sensor NOT_CONNECTED (skipped, no send attempt)\n");
    else
      Serial.printf("[MQTT] /sensor FAILED (err=%d)\n", mqttClient.lastError());
  }

  // ──────────────────────────────────────────────────────────────────────────
  // ──────────────────────────────────────────────────────────────────────────
  // PUBLISH 2 of 3 — /status  (alarm + health, CM V1)
  // v3: StaticJsonDocument 512->768, char buf 512->768, added latch diagnostics
  // v3 hardened: g_fl.pending/g_flCount read via a brief mutexFaultLatch
  // snapshot, released before serializeJson()/mqttClient.publish() run.
  // ──────────────────────────────────────────────────────────────────────────
  {
    bool     snapFlPending;
    uint32_t snapFlCount;
    if (xSemaphoreTake(mutexFaultLatch, pdMS_TO_TICKS(20)) == pdTRUE) {
      snapFlPending = g_fl.pending;
      snapFlCount   = g_flCount;
      xSemaphoreGive(mutexFaultLatch);
    } else {
      snapFlPending = (bool)g_fl.pending;  // mutex busy: fall back to volatile read (stale-but-safe)
      snapFlCount   = g_flCount;
    }

    // [M1B-2] 800 -> 1024, same reasoning as /trend: measured max 618 B plus
    // the four additive velocity_ema_* keys (~110 B) leaves too little headroom
    // to risk truncating the existing decision payload.
    StaticJsonDocument<1024> d;
    d["plant"]               = PLANT_ID;
    d["machine_id"]          = MACHINE_ID;
    d["sensor_id"]           = SENSOR_ID;
    d["stage"]               = "status";
    d["execution_location"]  = "edge";

    d["alarm_code"]          = alarmCode;
    d["alarm_level"]         = alarmLevel;
    d["health_score"]        = healthScore;   // [M1A] -1 == UNKNOWN, see HEALTH_SCORE_UNKNOWN

    // [M1A] Which engine is the alarm source of record, and whether it is
    // currently able to make a decision at all. A consumer must read
    // vibration_status before interpreting alarm_level: with status
    // UNAVAILABLE or THRESHOLDS_UNSET, alarm_level reflects a HELD previous
    // state, not a fresh evaluation of current vibration.
    // [M1B-5] TTW from FIFO-DSP velocity + M1B-4 slope. ADDITIVE: the legacy
    // ttw_estimate_h below keeps its VRMS-derived meaning until M1B-8.
    // Read status FIRST, then hours -- see the ordered-publication note at
    // g_ttwHours. The numeric field is published ONLY when status is VALID;
    // every other state omits it entirely, which is the existing "unknown"
    // idiom on this topic and costs no payload.
    {
      const uint8_t twStatus = g_ttwStatus;
      const float   twHours  = g_ttwHours;
      d["velocity_ttw_status"] = VibTtw_StatusStr(twStatus);
      if (twStatus == (uint8_t)VIB_TTW_VALID) {
        d["velocity_ttw_hours"] = roundf(twHours * 10.0f) / 10.0f;
      }
    }
    d["vibration_source"]    = "fifo_dsp";
    // [Phase 3J] Last inline copy of the status rule, now routed through the
    // same evaluator + formatter as /vibration. Identical values, one expression.
    d["vibration_status"]    = vibStatusStr(currentVibStatus());

    // [M1B-2] Timestamp-aware EMA over the M1B-1 history ring. ADDITIVE:
    // `ema_rms` on /trend keeps its legacy VRMS-derived meaning untouched --
    // these are new keys, not a redefinition. velocity_ema_valid is the sole
    // authority; when false, velocity_ema_mms is 0.0f meaning "not computed",
    // never "no vibration".
    {
      // [R-2] Same freshness deadline as readVelocityForAlarm()/pushTelemBuf(), so
      // velocity_ema_valid can no longer report true while vibration_status is
      // UNAVAILABLE. Observed pre-fix: ema_mms held 0.998 with valid=true for >=565 s.
      const VibEmaState em = VibEma_Get(millis(), VIB_VELOCITY_MAX_AGE_MS_TBD);
      d["velocity_ema_mms"]          = roundf(em.ema_mms * 1000.0f) / 1000.0f;
      d["velocity_ema_valid"]        = em.valid;
      d["velocity_ema_reseeded"]     = em.reseeded;
      d["velocity_ema_timestamp_ms"] = em.timestampMs;
    }

    // [Phase 1] bearing_alert + kurtosis_max/axis/valid REMOVED from /decision.
    d["dominant_vibration_axis"]    = domVibAxis;

    // [Phase2] REMOVED from /decision: freq_alert, freq_drift_x/y/z (freq_ratio
    // pipeline), rms_slope + trend_dir (legacy VRMS slope), spike_count (legacy
    // VPEAK) and ttw_estimate_h (legacy-RMS TTW). Current equivalents already on
    // this topic: velocity_ema_*, velocity_ttw_status / velocity_ttw_hours.

    d["timestamp"]           = tsBuf;

    d["fault_latch_pending"] = snapFlPending;
    d["fault_latch_count"]   = snapFlCount;
    // Telemetry buffer backlog (volatile read — mutex not needed for a display counter)
    d["telemetry_buffer_pending"] = (uint8_t)g_telemBufCount;

    char buf[1024];  // [M1B-2] 800 -> 1024, matches the enlarged /decision doc
    size_t szStatus = serializeJson(d, buf, sizeof(buf));
    // [M1B-7 E5/F1] FAIL CLOSED -- เหตุผลเดียวกับ /sensor ด้านบน
    // topic นี้ถือ alarm_level/health_score/vibration_status ซึ่งเป็น safety
    // semantics โดยตรง ส่ง JSON ที่ถูกตัดออกไปอันตรายกว่าทุก topic
    const bool statusJsonOk = (szStatus > 0 && szStatus < sizeof(buf) - 1);
    if (!statusJsonOk)
      Serial.printf("[WARN] /status JSON truncated -- NOT PUBLISHED! sz=%u buf=%u\n",
                    (unsigned)szStatus, (unsigned)sizeof(buf));
    bool connBefore7 = mqttClient.connected();  // [v16.5c] sampled before the call -- see dbgLogMqttPublish
#ifdef DEBUG_MQTT_TIMING
    uint32_t t0_pub7 = millis();
#endif
    bool pubOk7 = statusJsonOk &&
                  mqttClient.publish(g_mqttTopicDecision, buf, (int)szStatus, false, MQTT_QOS);
#ifdef DEBUG_MQTT_TIMING
    dbgLogMqttPublish(g_mqttTopicDecision, szStatus, MQTT_QOS, pubOk7,
                      mqttClient.lastError(), millis() - t0_pub7, connBefore7);
#endif
    if (!statusJsonOk)
      { /* [M1B-7] warning printed above -- ไม่ report เป็น publish failure */ }
    else if (pubOk7)
      Serial.printf("[MQTT] /status %u B\n", (unsigned)szStatus);
    else if (!connBefore7)
      Serial.printf("[MQTT] /status NOT_CONNECTED (skipped, no send attempt)\n");
    else
      Serial.printf("[MQTT] /status FAILED (err=%d)\n", mqttClient.lastError());
  }

  // PUBLISH 3 of 3 — /vibration  (BACKWARD-COMPATIBLE Grafana payload)
  // ALL original field names kept verbatim.
  // StaticJsonDocument reduced 3200 -> 2048 B; all fields still fit.
  // ──────────────────────────────────────────────────────────────────────────
  {
    StaticJsonDocument<2048> doc;

    doc["plant"]      = PLANT_ID;
    doc["machine_id"] = MACHINE_ID;
    doc["sensor_id"]  = SENSOR_ID;
    // [M1B-7 E1] topic เดียวที่ยังไม่มี stage -- /sensor, /decision, /trend มีครบแล้ว
    // ทำให้ consumer routing ด้วย stage ใช้กับ /vibration ได้เหมือนกัน
    doc["stage"]      = "vibration";

    // [M1B-7 E2/E3] PROVENANCE -- ประกาศ "ใครเป็นเจ้าของตัวเลขไหน" ใน payload เอง
    //
    // ปัญหาเดิม: /vibration ส่ง rms/vx/vy/vz จาก legacy VRMS register อยู่ข้าง ๆ
    // alarm_code/alarm_level/health_score ที่มาจาก FIFO-DSP (M1A ย้ายไปแล้ว)
    // โดยไม่มี label อะไรเลย -- dashboard ที่อ่าน topic นี้ topic เดียวแยกไม่ออกว่า
    // ค่าสั่นสะเทือนกับ alarm มาจากคนละ engine กัน /sensor ได้ label นี้ตอน M1A
    // แต่ /vibration ตกหล่นไป
    //
    // vibration_source = engine ที่เป็น alarm source of record (ดู g_velCarrier)
    // vibration_status ต้องอ่านก่อน alarm_level เสมอ: ถ้าเป็น UNAVAILABLE หรือ
    // THRESHOLDS_UNSET แปลว่า alarm_level คือ state ที่ HELD ไว้ ไม่ใช่ผลประเมินสด
    // -- semantics เดียวกับ /decision ทุกประการ (คัด ternary มาตรง ๆ)
    doc["vibration_source"]        = "fifo_dsp";
    doc["vibration_status"]        = vibStatusStr(currentVibStatus());
    // [Phase2] vibration_source_legacy REMOVED -- rms/vx/vy/vz/peak are gone from
    // this topic, so there is no legacy source left to declare provenance for.

    // ── [P1-S1] PRODUCT-1 SOURCE OF RECORD: FIFO-DSP velocity, ADDITIVE ─────
    // The canonical vibration metric finally lands on the canonical customer
    // topic. Until now velocity_rms_* existed ONLY on the /event accel_rms
    // message and the outage-replay path, so /vibration carried alarm_level
    // and vibration_status derived from FIFO-DSP velocity without ever
    // carrying the velocity itself.
    //
    // NO NEW CALCULATION. This is a pure read of g_velCarrier, the same value
    // taskAnalytics already computed once and published on /event -- read
    // idiom copied from pushTelemBuf() (.ino:3307-3329): same task
    // (taskNetwork/Core 1), same mutex, same 5 ms timeout, same
    // VIB_VELOCITY_MAX_AGE_MS_TBD freshness deadline, so "stale" means one
    // thing everywhere and the value matches the /event message for the same
    // capture bit-for-bit before rounding.
    //
    // VALIDITY: velocity_data_valid is ALWAYS present and is the sole
    // authority. The four floats are published ONLY when it is true --
    // omitted entirely otherwise, matching the replay serializer at
    // .ino:3594-3607. No zero, no null, no NaN is ever fabricated: a consumer
    // cannot mistake "not measured" for "measured zero".
    // [Phase2] velOk / vOverall are hoisted out of this block so the debug line
    // at the end of the /vibration publish can report the same velocity figure
    // that was published, instead of the legacy rms/peak it used to print.
    bool  velOk = false;
    float vOverall = 0.0f;
    {
      float vX = 0.0f, vY = 0.0f, vZ = 0.0f;
      if (mutexVelCarrier != NULL) {
        VelocityCarrier_t vc;
        bool got = false;
        if (xSemaphoreTake(mutexVelCarrier, pdMS_TO_TICKS(5)) == pdTRUE) {
          vc  = g_velCarrier;          // whole-struct copy: no torn read
          got = true;
          xSemaphoreGive(mutexVelCarrier);
        }
        if (got && vc.valid && vc.timestampMs != 0u &&
            (uint32_t)(millis() - vc.timestampMs) <= VIB_VELOCITY_MAX_AGE_MS_TBD) {
          velOk    = true;
          vOverall = vc.overall; vX = vc.x; vY = vc.y; vZ = vc.z;
        }
      }
      doc["velocity_data_valid"] = velOk;
      if (velOk) {
        doc["velocity_rms_overall"] = roundf(vOverall * 1000.0f) / 1000.0f;
        doc["velocity_rms_x"]       = roundf(vX * 1000.0f) / 1000.0f;
        doc["velocity_rms_y"]       = roundf(vY * 1000.0f) / 1000.0f;
        doc["velocity_rms_z"]       = roundf(vZ * 1000.0f) / 1000.0f;
      }

      // [Phase 3H] The rest of the canonical vibration set, moved here from
      // /sensor. Every value below comes from the same FIFO capture as the
      // velocity figures above, computed in the same publishTelemetry() call
      // from the same TelemetrySnapshot -- one timestamp, one cadence, no way
      // for a frequency to be stale relative to the velocity beside it.
      //
      // Each key keeps the omit-when-invalid contract it already had: an axis
      // or a figure the DSP could not produce is ABSENT, never 0. That is what
      // lets Node-RED's undefined filter and the API's _num() both report
      // "not measured" instead of inventing a zero.
      //
      // Gated independently of velOk on purpose: frequency and crest factor
      // carry their own validity, and the crest factor is an acceleration
      // figure that can be sound when the velocity integration was rejected.
      if (freqXok) doc["freq_x"] = freqX;
      if (freqYok) doc["freq_y"] = freqY;
      if (freqZok) doc["freq_z"] = freqZ;
      if (crestFactorOk) doc["crest_factor"] = crestFactor;
      doc["dominant_vibration_axis"] = domVibAxis;
    }

    // [Phase2] LEGACY VRMS/VPEAK REMOVED from /vibration: rms, vx, vy, vz, peak.
    // velocity_rms_overall / _x / _y / _z + velocity_data_valid above are the
    // only vibration numbers on this topic now. Node-RED was migrated off the
    // legacy names in Phase 0 (Health Logic, Maintenance Analytics v2, Prepare
    // InfluxDB) and the API contract already banned them.
    doc["temp"]  = round(data->temperature *  10) /  10.0f;
    doc["rpm"]   = data->rpm;

    // [v16.6i] CT-compensated engineering current (see compensateCurrent()).
    // [v16.6j] current_valid is now AGE-BASED freshness -- see the matching
    // comment on the /sensor export above for the full rationale. current_a
    // is unaffected -- always the last successfully measured value, never
    // fabricated to 0.
    doc["current_a"]     = round(data->current_a * 100) / 100.0f;
    doc["current_valid"] = currentFresh;
    doc["current_age_s"] = round(currentAgeS * 10) / 10.0f;
    // [P1-S2] freq_x/y/z, freq_ratio_x/y/z REMOVED from /vibration -- still
    // published unchanged on /sensor. Raw Hz is not a customer-actionable
    // number; the order-ratio form is Phase-2 diagnostic. Calculations
    // (freqX/Y/Z, freqRatioX/Y/Z) are UNTOUCHED and still feed /sensor.
    doc["motor_state"]         = data->motor_state;
    // [P1-S2] "state" (DEPRECATED alias of motor_state) REMOVED -- the source
    // comment already said to delete it once dashboards moved to motor_state.
    doc["operating_hours_total"] = data->runtime_hour;
    // [P1-S2] rotation_signal_ok REMOVED -- still on /sensor.
    // [P1-S2] alarm_code REMOVED -- numeric duplicate of alarm_level, and both
    // are on /decision. health_score REMOVED -- on /decision, and it returns
    // HEALTH_SCORE_UNKNOWN(-1) on 100% of publishes while thresholds are unset.
    doc["alarm_level"]         = alarmLevel;
    // [P1-S2] crest_factor, cf_x/y/z, kurtosis_x/y/z, kurtosis_max/axis/valid
    // and dominant_vibration_axis REMOVED -- all still on /sensor (kurtosis_*
    // and dominant_vibration_axis also on /decision). Their customer-actionable
    // distillate is bearing_alert, which is kept below. No calculation deleted:
    // crestFactor/kx/ky/kz/kmax/kaxis/kurtosisValid/domVibAxis are all still
    // computed and still consumed by /sensor and /decision.
    //
    // NOTE dominant_vibration_axis is derived from LEGACY rms_x/y/z, not from
    // FIFO-DSP velocity -- a further reason not to surface it as Product-1.
    // [Phase 1] bearing_alert REMOVED from /vibration -- this was the ONLY
    // legacy bearing field that still reached a live consumer.
    // ── [P1-S2] TREND / DIAGNOSTIC / INTERNAL-STATE BLOCK REMOVED ────────────
    // sensor_status (a hardcoded "ONLINE" constant), deglitch_count,
    // rms_slope, trend_dir, spike_count, freq_drift_x/y/z, freq_alert,
    // slope_60s, ema_dir and ema_rms are all still published unchanged on
    // /sensor, /decision and/or /trend. ttw_hours is still published on
    // /decision as ttw_estimate_h from the same g_trendResult.ttw_hours.
    //
    // NOT deleted anywhere: every calculation behind them still runs and is
    // still consumed by the other topics. This is a customer-contract change,
    // not a telemetry deletion.
    //
    // [P1-S3] OBSERVABILITY GAP CLOSED -- these nine had /vibration as their
    // ONLY publication path and were unpublished between P1-S2 and P1-S3:
    //   analysis_ready, freeze_reason, temp_slope, current_slope,
    //   current_buf_count, current_read_errors, current_evidence_valid,
    //   trend_window_s, sync_age_s
    // They are engineering diagnostics, not customer data, so they do not
    // belong on the Product-1 contract. They now ship on /device-health
    // (PUBLISH 4 of 4, below) at a fixed 60 s cadence -- same values, same
    // expressions, moved off the customer topic rather than recomputed.

    // ── [P1-S2] TRANSPORT BUDGET: /vibration JSON <= 1320 B ──────────────────
    // Unchanged ceiling, same reason as M1B-7.1: the A7670 accepts one
    // CIPSEND per publish (TinyGSM does not chunk); exceeding it returns
    // LWMQTT_NETWORK_FAILED_WRITE (-6), MQTTClient::publish() calls close(),
    // and the link reconnect-loops -- historically 3 of 29 publishes landed.
    //
    // M1B-7.1 met the ceiling by trimming 11 duplicated trend fields from a
    // 65-field union. P1-S2 replaces that union with an actual product
    // contract: ~27 fields, worst case ~618 B, leaving ~700 B of headroom.
    // That headroom is what makes the five FIFO-DSP velocity fields above
    // affordable -- added alone they would have pushed the old payload to
    // ~1390-1421 B, i.e. over the ceiling.
    doc["timestamp"]   = tsBuf;
    doc["time_synced"] = g_timeSync.synced;
    // [P1-S2] sync_age_s REMOVED -- see the observability-gap note above.

    char   jsonBuffer[2048];
    // [Phase 3H] Measure before serializing, not after. The post-serialize
    // check below still stands as a second line of defence, but measureJson()
    // gives the exact length while the buffer is still untouched, so an
    // oversized payload is refused rather than written truncated and then
    // detected. Same discipline the /event payload already uses.
    const size_t vibNeed = measureJson(doc);
    size_t jsonSize = 0;
    if (vibNeed + 1u > sizeof(jsonBuffer)) {
      Serial.printf("[MQTT] /vibration payload %u B exceeds buffer %u B -- not published\n",
                    (unsigned)vibNeed, (unsigned)sizeof(jsonBuffer));
    } else {
      jsonSize = serializeJson(doc, jsonBuffer, sizeof(jsonBuffer));
    }
    // [M1B-7 E5/F1] FAIL CLOSED. topic นี้เดิม "ไม่มี guard เลย" -- serializeJson()
    // ตัด payload แล้ว publish ออกไปเงียบ ๆ ไม่มีแม้แต่ warning ต่างจาก /sensor และ
    // /decision ที่อย่างน้อยยัง warn M1B-7 เพิ่ม field provenance เข้ามาใน doc นี้
    // (~120 B) จึงต้องมี guard ก่อน ไม่ใช่ตามหลัง
    // วัดจริงก่อนแก้: max 1378 B จาก 12 message -- เหลือ headroom ~670 B
    const bool vibJsonOk = (jsonSize > 0 && jsonSize < sizeof(jsonBuffer) - 1);
    if (!vibJsonOk)
      Serial.printf("[WARN] /vibration JSON truncated -- NOT PUBLISHED! sz=%u buf=%u\n",
                    (unsigned)jsonSize, (unsigned)sizeof(jsonBuffer));
    bool connBefore8 = mqttClient.connected();  // [v16.5c] sampled before the call -- see dbgLogMqttPublish
#ifdef DEBUG_MQTT_TIMING
    uint32_t t0_pub8 = millis();
#endif
    success = vibJsonOk &&
              mqttClient.publish(g_mqttTopic, jsonBuffer, (int)jsonSize, false, MQTT_QOS);
#ifdef DEBUG_MQTT_TIMING
    dbgLogMqttPublish(g_mqttTopic, jsonSize, MQTT_QOS, success,
                      mqttClient.lastError(), millis() - t0_pub8, connBefore8);
#endif

    if (!vibJsonOk) {
      /* [M1B-7] warning printed above -- ไม่ report เป็น publish failure */
    } else if (success) {
      // [v16.5] ใช้ reportedRms (ค่าที่ gate แล้ว) แทน data->rms_overall (raw)
      // เพื่อให้ debug log ตรงกับค่าที่ publish จริงใน doc["rms"]
      // [Phase2] rms/peak and frx/fry/frz (freq_ratio) dropped from this line --
      // the values no longer exist. Velocity is reported by velOk/vOverall.
      // [Phase 3F] fx/fy/fz are the FIFO/DSP dominant frequency and print "--"
      // per axis when that axis had no reportable peak -- same convention as
      // vel above, so the log never shows a 0 Hz that was never measured.
      Serial.printf("[MQTT] /vibration %d B | %s vel=%s mm/s rpm=%.1f "
                    "state=%d | health=%d%% | fx=%s fy=%s fz=%s | "
                    "cf=%s\n",
                    jsonSize, alarmLevel,
                    velOk ? String(vOverall, 2).c_str() : "--", data->rpm,
                    data->motor_state,
                    healthScore,
                    freqXok ? String(freqX, 1).c_str() : "--",
                    freqYok ? String(freqY, 1).c_str() : "--",
                    freqZok ? String(freqZ, 1).c_str() : "--",
                    crestFactorOk ? String(crestFactor, 2).c_str() : "--");
    } else if (!connBefore8) {
      Serial.printf("[MQTT] /vibration NOT_CONNECTED (skipped, no send attempt)\n");
    } else {
      Serial.printf("[MQTT] Publish FAILED (err=%d)\n", mqttClient.lastError());
    }
  }

  // --------------------------------------------------------------------------
  // PUBLISH 4 of 4 - /device-health  (ENGINEERING DIAGNOSTICS, NOT Product-1)
  //
  // [P1-S3] Closes the observability gap P1-S2 documented and left open: nine
  // fields whose only publication path was /vibration, dropped when that topic
  // was frozen to the ~614 B Product-1 contract. Every value below is a pure
  // read or copy of state this function already holds -- NO new calculation,
  // and the same expressions P1-S2 removed, verbatim.
  //
  // CADENCE: its own 60 s throttle, deliberately NOT tied to publishInterval.
  // The telemetry cadence is alarm-driven (30/10/5 s for NORMAL/WARNING/
  // CRITICAL); diagnostics have no reason to accelerate with alarm state, and
  // a 4th CIPSEND every 5 s is exactly the A7670 link pressure that produced
  // LWMQTT_NETWORK_FAILED_WRITE historically. This block therefore rides the
  // telemetry cycle that first crosses 60 s and skips every cycle in between.
  // First call after boot publishes immediately (lastDeviceHealthPub == 0), so
  // a device that never reaches 60 s of MQTT uptime still reports once.
  //
  // FAILURE ISOLATION: the publish result stays in a LOCAL and is never folded
  // into `success`. A /device-health failure must not stall lastPublish or
  // inflate g_network.publishFailures -- diagnostics must never be able to make
  // the main telemetry path look broken.
  {
    static uint32_t lastDeviceHealthPub = 0;
    const uint32_t  nowDh = millis();
    if (lastDeviceHealthPub == 0 ||
        (uint32_t)(nowDh - lastDeviceHealthPub) >= DEVICE_HEALTH_INTERVAL_MS) {
      StaticJsonDocument<512> h;
      h["plant"]              = PLANT_ID;
      h["machine_id"]         = MACHINE_ID;
      h["sensor_id"]          = SENSOR_ID;
      h["stage"]              = "device_health";
      h["execution_location"] = "edge";

      // [P1-S3] analysisReason() is called ONCE, not twice as the pre-P1-S2
      // /vibration block did: analysis_ready and freeze_reason are two views of
      // one state and must not be able to disagree inside a single payload.
      const AnalysisReason_t anaR = analysisReason();
      h["analysis_ready"]     = (anaR == ANA_READY);
      h["freeze_reason"]      = analysisReasonStr(anaR);

      h["temp_slope"]         = g_trendResult.temp_slope;
      h["current_slope"]      = g_trendResult.current_slope;   // [v16.6a] CTR4A01, A/s
      // [v16.6b] CTR4A01 remote-diagnosis pair for the current_slope=0 ambiguity:
      // buf_count stuck at 0 while read_errors climbs => Modbus reads are failing
      // (slave address/wiring/baud); both stuck at 0 => the 500 ms cadence never fired.
      h["current_buf_count"]      = g_currentCount;
      h["current_read_errors"]    = g_ctReadErrors;
      // [P4-02] pure copy, no computation -- see P4_02_DESIGN_CONTRACT.md
      h["current_evidence_valid"] = snap->currentEvidenceValid;

      h["trend_window_s"]     = (g_trendResult.window_samples * 250) / 1000;

      h["timestamp"]          = tsBuf;
      h["time_synced"]        = g_timeSync.synced;
      if (g_timeSync.synced)
        h["sync_age_s"]       = (millis() - g_timeSync.lastSyncMillis) / 1000;

      char   hBuf[512];
      size_t szHealth = serializeJson(h, hBuf, sizeof(hBuf));
      // FAIL CLOSED -- same idiom as /sensor, /decision and /vibration.
      const bool healthJsonOk = (szHealth > 0 && szHealth < sizeof(hBuf) - 1);
      if (!healthJsonOk) {
        Serial.printf("[WARN] /device-health JSON truncated -- NOT PUBLISHED! sz=%u buf=%u\n",
                      (unsigned)szHealth, (unsigned)sizeof(hBuf));
      } else {
        bool connBefore9 = mqttClient.connected();  // sampled before the call, per dbgLogMqttPublish
#ifdef DEBUG_MQTT_TIMING
        uint32_t t0_pub9 = millis();
#endif
        bool pubOk9 = mqttClient.publish(g_mqttTopicDeviceHealth, hBuf, (int)szHealth,
                                         false, MQTT_QOS);
#ifdef DEBUG_MQTT_TIMING
        dbgLogMqttPublish(g_mqttTopicDeviceHealth, szHealth, MQTT_QOS, pubOk9,
                          mqttClient.lastError(), millis() - t0_pub9, connBefore9);
#endif
        if (pubOk9) {
          // The throttle advances ONLY on a successful publish, so a failed
          // cycle retries at the next telemetry tick instead of going dark for
          // another 60 s. g_network counters are deliberately not touched.
          lastDeviceHealthPub = nowDh;
          Serial.printf("[MQTT] /device-health %u B\n", (unsigned)szHealth);
        } else if (!connBefore9) {
          Serial.printf("[MQTT] /device-health NOT_CONNECTED (skipped, no send attempt)\n");
        } else {
          Serial.printf("[MQTT] /device-health FAILED (err=%d)\n", mqttClient.lastError());
        }
      }
    }
  }

  return success;
}

// ============================================================================
// TASK: ANALYTICS (CORE 1, Priority 3) -- Phase 2
// ============================================================================
//
// ???????:
//   1. ??????? 1,000 ms (1 Hz) — FreeRTOS tick unchanged
//   2. ???? g_trendBuf snapshot (Core 0 ????? / Core 1 ???? -- atomic float)
//   3. ????? AggSample_t ??? 4 raw samples ?????? -> push g_buf1s
//      (Claim 2: flush triggered by acc1sMs >= g_slotDur1sMs, not fixed counter)
//   4. Cascade -> g_buf10s ??? ~10s real time
//   5. Cascade -> g_buf60s ??? ~60s real time
//   6. ?????? EMA + g_emaDir
//   7. Publish /analytics MQTT topic ??? 60 ?????? (??? MQTT connected)
//
// Stack: 5120 bytes (?????? linreg loop + float arrays ?? stack)
// Priority: 3 (??????? display -- ??? block network/modbus)
// ============================================================================

// [Phase2] REMOVED: computeSlotDurMs() -- RPM-adaptive slot duration helper.


// ----------------------------------------------------------------------------
// [Phase 3B] Core 1 half of the waveform hand-off: consume one snapshot, run
// the acceleration RMS, publish. Called once per taskAnalytics tick (1 Hz)
// against a ~0.5 Hz capture cadence, so it keeps up with a 2x margin.
//
// Runs entirely on Core 1 and never touches FifoDriver -- by the time this
// sees a sample, Core 0 has already released the arena. This is where every
// floating-point operation in the Phase 3B pipeline happens.
//
// Publishes a SEPARATE /event message rather than extending the existing
// "fifo_capture" event: that event is built and enqueued on Core 0 before the
// DSP has run, so the acceleration figures cannot exist yet at that point.
// The two are correlated by capture_id. Nothing in the legacy payload is
// touched, and `rms`/`vx`/`vy`/`vz` on the other topics are not written here.
// ----------------------------------------------------------------------------
static void processPendingAccelSnapshot() {
  if (queueAccelSnapshot == NULL || mutexAccelSnap == NULL) {
    return;  // best-effort path unavailable (Phase 3 decision #3)
  }

  AccelSnapshotReady_t ready;
  if (xQueueReceive(queueAccelSnapshot, &ready, 0) != pdTRUE) {
    return;  // nothing new this tick -- normal for ~half of all ticks
  }

  // [Phase 3C] Copy out, release, THEN compute -- the mutex is held for one
  // ~6 KB memcpy only. No DSP of any kind runs under this lock (Phase 3C
  // architecture requirement), so Core 0's timeout-0 take can never collide
  // with an FFT.
  if (xSemaphoreTake(mutexAccelSnap, pdMS_TO_TICKS(50)) == pdTRUE) {
    memcpy(&g_accelWork, &g_accelSnap, sizeof(g_accelWork));
    xSemaphoreGive(mutexAccelSnap);
  } else {
    return;  // Core 0 mid-copy; the next capture's notification will follow
  }

  const uint32_t captureId   = g_accelWork.captureId;
  const uint16_t sampleCount = g_accelWork.sampleCount;
  const uint32_t srHz        = g_accelWork.srHz;   // per-capture provenance, never a literal
  const float    rpmAtCap    = g_accelWork.rpmAtCapture;  // [Phase 3E] same provenance rule as srHz

  // Both computations run lock-free against Core 1's private copy.
  // [Phase 3B] acceleration -- arithmetic untouched by Phase 3C.
  VibAccelRms rms;
  const bool aOk = VibAccel_ComputeRms(g_accelWork.x, g_accelWork.y, g_accelWork.z,
                                       sampleCount, srHz, &rms);
  // [Phase 3C] velocity -- frequency-domain integration.
  VibVelocityRms vel;
  const bool vOk = VibVelocity_ComputeRms(g_accelWork.x, g_accelWork.y, g_accelWork.z,
                                          sampleCount, srHz, rpmAtCap, &vel);

  // `valid` is the sole authority -- a false here publishes zeros WITH the flag
  // clear, so a consumer that honors vibration_data_valid can never read a
  // "not computed" 0.0 as a measured zero.
  //
  // [Phase 3C] vibration_data_valid keeps its EXACT Phase 3B meaning
  // (acceleration validity). Velocity gets its own flag rather than being
  // folded into the existing one: widening vibration_data_valid would silently
  // change what an already-deployed consumer sees for the acceleration fields,
  // which the "keep acceleration RMS fields unchanged" requirement forbids.
  const bool dataValid    = (aOk && rms.valid);
  const bool velDataValid = (vOk && vel.valid);

  StaticJsonDocument<512> aDoc;
  aDoc["plant_id"]                 = PLANT_ID;
  aDoc["machine_id"]               = MACHINE_ID;
  aDoc["event"]                    = "accel_rms";
  aDoc["capture_id"]               = captureId;
  aDoc["vibration_data_valid"]     = dataValid;
  aDoc["acceleration_rms_x"]       = rms.rms_x;
  aDoc["acceleration_rms_y"]       = rms.rms_y;
  aDoc["acceleration_rms_z"]       = rms.rms_z;
  aDoc["acceleration_rms_overall"] = rms.rms_overall;
  // [Phase 3C] additive velocity block -- new keys only. Legacy `rms`/`vx`/
  // `vy`/`vz` live on other topics and are not written anywhere in this
  // function, then or now.
  aDoc["velocity_data_valid"]      = velDataValid;
  aDoc["velocity_rms_x"]           = vel.rms_x;
  aDoc["velocity_rms_y"]           = vel.rms_y;
  aDoc["velocity_rms_z"]           = vel.rms_z;
  aDoc["velocity_rms_overall"]     = vel.rms_overall;
  aDoc["sample_rate_hz"]           = srHz;
  aDoc["sample_count"]             = sampleCount;
  // [Phase 3D] Dominant frequency, additive and per-axis. Published ONLY when
  // that axis's flag is true -- an invalid axis has its key OMITTED entirely
  // rather than sent as 0, null or NaN, matching how velocity_rms_* is handled
  // on the replay path: a consumer must never be able to read "not measured"
  // as a measurement. Derived from the same spectrum as velocity_rms_* above,
  // so capture_id and sample_rate_hz describe these figures too.
  if (velDataValid && vel.dominant_frequency_x_valid) {
    aDoc["dominant_frequency_x_hz"] = roundf(vel.dominant_frequency_x_hz * 1000.0f) / 1000.0f;
  }
  if (velDataValid && vel.dominant_frequency_y_valid) {
    aDoc["dominant_frequency_y_hz"] = roundf(vel.dominant_frequency_y_hz * 1000.0f) / 1000.0f;
  }
  if (velDataValid && vel.dominant_frequency_z_valid) {
    aDoc["dominant_frequency_z_hz"] = roundf(vel.dominant_frequency_z_hz * 1000.0f) / 1000.0f;
  }
  // [Phase 3E] Band-integrated 1x / 2x velocity amplitude, per axis, in mm/s.
  // Same omit-when-invalid contract as the dominant frequency above: an axis
  // whose harmonic band is not observable has its key absent, never 0.
  //
  // These are AMPLITUDES ONLY. No ratio, no imbalance/misalignment indicator
  // and no threshold is computed here or anywhere else in the firmware -- a
  // 2x/1x ratio is derived downstream from these two raw numbers, where the
  // definition can still change without a reflash.
  if (velDataValid && vel.velocity_1x_x_valid) {
    aDoc["velocity_1x_x_mm_s"] = roundf(vel.velocity_1x_x_mm_s * 1000.0f) / 1000.0f;
  }
  if (velDataValid && vel.velocity_1x_y_valid) {
    aDoc["velocity_1x_y_mm_s"] = roundf(vel.velocity_1x_y_mm_s * 1000.0f) / 1000.0f;
  }
  if (velDataValid && vel.velocity_1x_z_valid) {
    aDoc["velocity_1x_z_mm_s"] = roundf(vel.velocity_1x_z_mm_s * 1000.0f) / 1000.0f;
  }
  if (velDataValid && vel.velocity_2x_x_valid) {
    aDoc["velocity_2x_x_mm_s"] = roundf(vel.velocity_2x_x_mm_s * 1000.0f) / 1000.0f;
  }
  if (velDataValid && vel.velocity_2x_y_valid) {
    aDoc["velocity_2x_y_mm_s"] = roundf(vel.velocity_2x_y_mm_s * 1000.0f) / 1000.0f;
  }
  if (velDataValid && vel.velocity_2x_z_valid) {
    aDoc["velocity_2x_z_mm_s"] = roundf(vel.velocity_2x_z_mm_s * 1000.0f) / 1000.0f;
  }
  // [M1A] Declares which engine produced these vibration figures, so a
  // consumer can distinguish them from the deprecated VRMS-register values
  // still present on /sensor and /vibration.
  aDoc["vibration_source"]         = "fifo_dsp";

  // [Phase 3E] 600 -> 1024. Worst case observed on the running device was 543 B
  // for the Phase 3D payload; the six new keys add at most
  //   6 * ( 18 name + 2 quotes + 1 colon + 14 value + 1 comma ) = 216 B
  // giving ~759 B worst case and ~265 B of margin. taskAnalytics has an 8 KB
  // stack (STACK_SIZE_ANALYTICS) so the extra 424 B of frame is affordable.
  char aBuf[1024];

  // Truncation is NOT relied on as a safety net: measureJson gives the exact
  // length before anything is written, so an oversized payload is dropped with
  // a log rather than published as malformed JSON that a consumer would fail
  // to parse (or, worse, parse partially).
  const size_t needA = measureJson(aDoc);
  size_t szA = 0;
  if (needA + 1u > sizeof(aBuf)) {
    Serial.printf("[ACCEL-RMS] payload %u B exceeds buffer %u B -- event dropped\n",
                  (unsigned)needA, (unsigned)sizeof(aBuf));
  } else {
    szA = serializeJson(aDoc, aBuf, sizeof(aBuf));
    enqueueMqttOutbound(MQTT_OUTBOUND_TOPIC_EVENT, aBuf, szA, MQTT_QOS);
  }

  // [M1A] Hand the velocity figure to Core 0's alarm/health/latch path.
  // Published AFTER the MQTT enqueue so telemetry is never delayed by mutex
  // contention, and published on EVERY capture -- including invalid ones,
  // because Core 0 must learn that this capture produced no usable velocity
  // (that is what drives VIBRATION_UNAVAILABLE) rather than silently
  // continuing to age the previous value.
  //
  // timeout 0: taskAnalytics must not block. If Core 0 holds the mutex, skip
  // -- the carrier keeps its previous contents and simply ages, which the
  // freshness check on the reader side already handles correctly.
  if (mutexVelCarrier != NULL) {
    if (xSemaphoreTake(mutexVelCarrier, 0) == pdTRUE) {
      g_velCarrier.overall     = velDataValid ? vel.rms_overall : 0.0f;
      g_velCarrier.valid       = velDataValid;
      g_velCarrier.captureId   = captureId;
      g_velCarrier.timestampMs = millis();
      // [M1B-6] Per-axis + provenance for the outage buffer. Written under the
      // same mutex and in the same store as `overall`, so a reader taking the
      // mutex always sees a self-consistent set.
      g_velCarrier.x            = velDataValid ? vel.rms_x : 0.0f;
      g_velCarrier.y            = velDataValid ? vel.rms_y : 0.0f;
      g_velCarrier.z            = velDataValid ? vel.rms_z : 0.0f;
      g_velCarrier.sampleRateHz = srHz;
      g_velCarrier.sampleCount  = sampleCount;
      // [Phase 3D] Written under the SAME mutex, in the SAME store, from the
      // SAME capture as every field above -- a reader that takes the mutex can
      // never see a dominant frequency belonging to a different capture than
      // the velocity beside it.
      g_velCarrier.domFreqX      = velDataValid ? vel.dominant_frequency_x_hz : 0.0f;
      g_velCarrier.domFreqY      = velDataValid ? vel.dominant_frequency_y_hz : 0.0f;
      g_velCarrier.domFreqZ      = velDataValid ? vel.dominant_frequency_z_hz : 0.0f;
      g_velCarrier.domFreqXValid = velDataValid && vel.dominant_frequency_x_valid;
      g_velCarrier.domFreqYValid = velDataValid && vel.dominant_frequency_y_valid;
      g_velCarrier.domFreqZValid = velDataValid && vel.dominant_frequency_z_valid;
      // [Phase 3G] Gated on dataValid (ACCELERATION validity), not velDataValid:
      // a capture can yield a usable acceleration record while the velocity
      // integration is rejected, and the crest factor belongs to the former.
      g_velCarrier.crestFactor      = (dataValid && rms.crest_factor_valid)
                                        ? rms.crest_factor : 0.0f;
      g_velCarrier.crestFactorValid = dataValid && rms.crest_factor_valid;
      xSemaphoreGive(mutexVelCarrier);
    }
  }
}

// ----------------------------------------------------------------------------
// [M1B-1] THE ONE AND ONLY append point for the new trend pipeline.
//
// Runs on Core 1 inside taskAnalytics (1 Hz). Nothing else in this firmware
// calls VibHistory_Append()/VibHistory_AppendGap() -- that single-writer
// property is what makes the ring's ordering and gap invariants provable, and
// it mirrors handleFifoCaptureCompletion()'s "exactly one acquire/release
// site" contract.
//
// Idempotent by captureId: this ticks at 1 Hz while captures arrive every
// ~2.1-3.9 s, so most ticks see the same capture twice or three times. Rather
// than re-appending (and being refused as a duplicate, inflating the reject
// counter), it simply returns when the carrier's captureId has not advanced.
//
// Ordering of the three branches is deliberate:
//   1. motor not RUNNING  -> gap, and NO vibration sample. Checked FIRST
//      because a stale carrier from before the motor stopped must not be
//      mistaken for a live reading.
//   2. velocity invalid/stale -> gap.
//   3. new capture -> append.
// None of these paths can ever append a zero: branches 1 and 2 record a NaN
// gap marker (see vib_history.h), and branch 3 only stores a value the DSP
// declared valid.
//
// M1B-1 SCOPE: this fills the ring and nothing else. No EMA, no slope, no TTW,
// no windowing -- those read the ring in M1B-2..M1B-5.
// ----------------------------------------------------------------------------
static uint32_t s_lastIngestedCaptureId = 0;

static void vibHistoryIngest() {
  const uint32_t nowMs = millis();

  // 1. Motor gate (rule C).
  if (g_motorRunState != MOTOR_RUNNING) {
    VibHistory_AppendGap(nowMs, VIB_GAP_MOTOR_NOT_RUNNING);
    return;
  }

  if (mutexVelCarrier == NULL) {
    return;
  }
  VelocityCarrier_t snap;
  if (xSemaphoreTake(mutexVelCarrier, pdMS_TO_TICKS(5)) != pdTRUE) {
    return;  // try again next tick; not a gap -- we learned nothing
  }
  snap = g_velCarrier;   // whole-struct copy, no torn read
  xSemaphoreGive(mutexVelCarrier);

  // 2. Validity + freshness (rule B). Same deadline constant the M1A alarm
  // path uses, so trend and alarm can never disagree about what "stale" means.
  // Unsigned subtraction is millis()-rollover correct.
  const uint32_t age = nowMs - snap.timestampMs;
  if (!snap.valid || snap.timestampMs == 0u || age > VIB_VELOCITY_MAX_AGE_MS_TBD) {
    VibHistory_AppendGap(nowMs, snap.valid ? VIB_GAP_STALE : VIB_GAP_VELOCITY_INVALID);
    return;
  }

  // 3. Only advance on a genuinely new capture.
  if (snap.captureId == s_lastIngestedCaptureId) {
    return;
  }

  // dt is carried entirely by snap.timestampMs (rules D/E): the producer's own
  // completion time, NOT this tick's time and NOT a nominal 2 s period. That is
  // what preserves the measured bimodal 2.1 s / 3.9 s spacing instead of
  // quantising every sample onto the 1 Hz analytics grid.
  if (VibHistory_Append(snap.overall, snap.timestampMs, snap.captureId) == VIB_HIST_OK) {
    s_lastIngestedCaptureId = snap.captureId;
  }
}

// ----------------------------------------------------------------------------
// [M1B-2] Advance the timestamp-aware EMA.
//
// Reads the M1B-1 ring ONLY -- VibEma_Update() takes no arguments and pulls
// every new entry from VibHistory_*. It never sees g_velCarrier, and this
// function passes it nothing, so there is no path by which the EMA could
// bypass the ring. That is deliberate: the ring is the recorded truth, and an
// EMA computed from anything else could disagree with the history it claims to
// summarise.
//
// Idempotent, so calling it once per 1 Hz analytics tick against a ~0.32 Hz
// sample rate costs nothing on the majority of ticks that bring no new data.
//
// Touches neither g_emaRms nor any legacy trend state -- the legacy EMA
// continues to run untouched on the legacy VRMS path until M1B-8.
// ----------------------------------------------------------------------------
static void vibEmaTick() {
  VibEma_Update();
}

void taskAnalytics(void* parameter) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xPeriod = pdMS_TO_TICKS(1000);  // 1 Hz — unchanged


  // Publish /analytics counter
  uint8_t&  analyticsPublishCnt = (uint8_t&)g_anaPublishCnt;

  Serial.println("[CORE 1] Analytics task started (Phase 2+Claim2)");

  while (1) {
    vTaskDelayUntil(&xLastWakeTime, xPeriod);

    // [Phase 3B] Non-blocking; returns immediately when no waveform is pending.
    // Placed first so acceleration RMS is never delayed by the rest of the
    // tick, and deliberately outside every analytics FREEZE/RUNNING gate --
    // a FIFO capture's acceleration RMS is a property of the capture, not of
    // the trend engine's state.
    processPendingAccelSnapshot();

    // [M1B-1] Ingest into the trend history ring, immediately after the
    // velocity carrier may have been refreshed above. Non-blocking; no slope
    // or TTW is computed here -- M1B-1 only records what happened.
    vibHistoryIngest();

    // [M1B-2] Then advance the EMA over whatever the ring just gained.
    // Strictly after the ingest so a sample recorded this tick is folded in on
    // this tick rather than waiting a second.
    vibEmaTick();




    // [v16.3ab] FREEZE gate — derived state (คำนวณสด), พร้อมเหตุผล (Point 1+2)
    AnalysisReason_t anaReason = analysisReason();
    bool anaReady = (anaReason == ANA_READY);

    // log เหตุผลเมื่อ "เปลี่ยน" เท่านั้น (ไม่ spam ทุกวินาที)
    static AnalysisReason_t s_lastAnaReason = ANA_READY;
    if (anaReason != s_lastAnaReason) {
      if (anaReady) Serial.println("[ANALYTICS] Resumed -- analysis READY");
      else          Serial.printf("[ANALYTICS] Frozen -- reason=%s\n", analysisReasonStr(anaReason));
      s_lastAnaReason = anaReason;
    }


    // [Phase2] REMOVED: the RPM-adaptive slot accumulators, the
    // g_buf1s/10s/60s cascade and the legacy VRMS EMA that lived here.
    // Trend is now produced entirely by the FIFO/DSP modules invoked above
    // (vibHistoryIngest -> VibEma_Update), and published from VibWindow_/
    // VibSlope_/VibTtw_ in the /trend block below.

analytics_publish:
    analyticsPublishCnt++;
    if (analyticsPublishCnt < 60) continue;
    analyticsPublishCnt = 0;

    // [v16.5] Section 7 Item 6 (design v16.5 §3.3, §4.2) — read via cache
    // instead of touching mqttClient directly; Analytics is not the owner task.
    if (!getMqttConnectedCached()) continue;
    // [Phase2] the g_buf1sCount >= 4 readiness gate went away with the slot
    // cascade. The FIFO/DSP window/slope/EMA blocks below each carry their own
    // validity flag (velocity_status_*, velocity_slope_valid, ema.valid), so an
    // early /trend publish reports "not ready" rather than being suppressed.

    // Shared timestamp for this publish round (all 7 topics use same value)
    char tsA[26] = "not_available";
    if (g_rtcValid) {
      DateTime nowTs; RTC_NOW_SAFE(nowTs);  // [v16.3g] FIX: was rtc.now() without mutex → crash
      // [v16.3o] year sanity check — ป้องกัน 2106-02-06 จาก I2C collision กับ OLED
      // ถ้าปีผิด → ใช้ lastSyncTime จาก NTP แทน (ซึ่งถูกต้องเสมอหลัง sync)
      if (nowTs.year() >= 2024 && nowTs.year() <= 2035) {
        snprintf(tsA, sizeof(tsA), "%04d-%02d-%02dT%02d:%02d:%02dZ",
                 nowTs.year(), nowTs.month(), nowTs.day(),
                 nowTs.hour(), nowTs.minute(), nowTs.second());
      } else if (g_timeSync.synced && strlen(g_timeSync.lastSyncTime) > 10) {
        // fallback: ใช้ NTP sync time ล่าสุด — อาจช้าไปบ้าง แต่ดีกว่า 2106
        strncpy(tsA, g_timeSync.lastSyncTime, sizeof(tsA) - 1);
        tsA[sizeof(tsA) - 1] = '\0';
        Serial.printf("[ANALYTICS] ! RTC corrupt year=%u -- using NTP fallback ts\n",
                      nowTs.year());
      }
    } else if (g_timeSync.synced && strlen(g_timeSync.lastSyncTime) > 10) {
      // RTC ไม่พร้อม แต่มี NTP sync time
      strncpy(tsA, g_timeSync.lastSyncTime, sizeof(tsA) - 1);
      tsA[sizeof(tsA) - 1] = '\0';
    }

    // ──────────────────────────────────────────────────────────────────────
    // PUB-A  /trend   (replaces /analytics, same 60-s cadence)
    // Stage : multi-resolution buffer + slope + EMA + spike + variance
    // Doc   : 640 B stack
    // ──────────────────────────────────────────────────────────────────────
    {
      // [M1B-3] 1024 -> 1536: /trend measured 857 B after M1B-2 and the twelve
      // window keys add ~354 B. Sized together with MQTT_OUTBOUND_PAYLOAD_MAX
      // (also 1536) and buf[] below, so document capacity, buffer size and the
      // enqueue limit can never disagree.
      // [M1B-2] 640 -> 1024. NOT cosmetic: /trend was measured at up to 630 B
      // on hardware against a 640 B document, and the eight additive
      // history_* keys add ~227 B. Leaving 640 would have silently truncated
      // the EXISTING trend payload -- the "[WARN] /trend JSON truncated" path
      // below -- breaking current consumers rather than just omitting the new
      // fields. buf[1024] already downstream, and the enqueue limit
      // (MQTT_OUTBOUND_PAYLOAD_MAX 1024) still bounds the worst case ~857 B.
      StaticJsonDocument<1536> t;
      t["plant"]               = PLANT_ID;
      t["machine_id"]          = MACHINE_ID;
      t["sensor_id"]           = SENSOR_ID;
      t["stage"]               = "trend";
      t["execution_location"]  = "edge";

      // [Phase2] LEGACY TREND KEYS REMOVED (no consumer, no alias):
      //   buf_1s/10s/60s, ready_1s/10s/60s, slope_1s/10s/60s,
      //   max_rms_10min, max_rms_60min, stddev_1min,
      //   slope_var_1s/10s/60s, ema_rms, ema_dir, ema_delta, spike_count
      // Replaced by the FIFO/DSP block further down: velocity_slope_mms_per_s,
      // velocity_stddev_60s/300s_mms, velocity_max_60s/300s_mms,
      // velocity_samples_*, velocity_status_*, history_*.

      // [M1B-2 / part B] Low-rate history-ring observability.
      //
      // Hosted on /trend precisely because /trend already publishes on a 60 s
      // cadence -- that satisfies "at most once every 60 seconds" structurally,
      // with no rate-limiter to get wrong and no new topic. Nothing here is
      // printed per capture and no Serial output is added.
      //
      // Purely additive and read-only: these keys are new, and none of the
      // existing /trend fields change meaning. Their sole purpose is to let the
      // combined M1B-1/M1B-2 hardware run verify the ring's invariants
      // (append/gap/reject counts, captureId and timestamp advance, latest
      // value and validity) which are otherwise unobservable on hardware.
      //
      // Summary only -- deliberately NOT a ring dump.
      {
        const VibHistoryStats hs = VibHistory_GetStats();
        const VibHistorySample* hl = VibHistory_Latest();
        t["history_count"]         = VibHistory_Count();
        t["history_valid_count"]   = hs.appended;
        t["history_gap_count"]     = hs.gaps;
        t["history_reject_count"]  = hs.rejected;
        t["history_latest_capture_id"]  = hl ? hl->captureId   : 0;
        t["history_latest_timestamp_ms"] = hl ? hl->timestampMs : 0;
        // A gap's stored value is NaN, which is not representable in JSON.
        // Publish 0.0 ONLY when the entry is explicitly flagged invalid, so
        // history_latest_valid=false is what a consumer must branch on -- the
        // number is meaningless in that case and must never be read as a
        // measurement of zero vibration.
        t["history_latest_velocity_mms"] =
            (hl && hl->valid) ? roundf(hl->velocity_rms_overall * 1000.0f) / 1000.0f : 0.0f;
        t["history_latest_valid"]  = hl ? hl->valid : false;
      }

      // [M1B-3] Time-based trend windows over the M1B-1 ring.
      //
      // Computed here, at publish time, from the ring's own timestamps -- there
      // is no accumulator to keep in sync and no cadence assumption anywhere.
      // Both windows are evaluated against the SAME `nowW` so 60 s and 300 s
      // describe the same instant.
      //
      // Purely additive: the legacy slope_1s/10s/60s, ema_rms and rms fields
      // above keep their existing VRMS-derived meanings untouched until M1B-8.
      // The legacy 1 s window is NOT migrated -- at ~0.32 Hz a 1-second window
      // cannot contain a sample, so publishing one would be meaningless.
      //
      // velocity_status_* is the sole authority. On INSUFFICIENT_DATA every
      // statistic is 0 meaning "not computed", never "measured zero"; the
      // sample count is still reported so a consumer can tell "sensor down"
      // from "still filling".
      {
        const uint32_t nowW = millis();
        VibWindowStats w60, w300;
        VibWindow_Compute(VIB_WIN_60S_MS,  nowW, &w60);
        VibWindow_Compute(VIB_WIN_300S_MS, nowW, &w300);

        t["velocity_mean_60s_mms"]    = roundf(w60.mean_mms   * 1000.0f) / 1000.0f;
        t["velocity_min_60s_mms"]     = roundf(w60.min_mms    * 1000.0f) / 1000.0f;
        t["velocity_max_60s_mms"]     = roundf(w60.max_mms    * 1000.0f) / 1000.0f;
        t["velocity_stddev_60s_mms"]  = roundf(w60.stddev_mms * 100000.0f) / 100000.0f;
        t["velocity_samples_60s"]     = w60.sample_count;
        t["velocity_status_60s"]      = VibWindow_StatusStr(w60.status);

        t["velocity_mean_300s_mms"]   = roundf(w300.mean_mms   * 1000.0f) / 1000.0f;
        t["velocity_min_300s_mms"]    = roundf(w300.min_mms    * 1000.0f) / 1000.0f;
        t["velocity_max_300s_mms"]    = roundf(w300.max_mms    * 1000.0f) / 1000.0f;
        t["velocity_stddev_300s_mms"] = roundf(w300.stddev_mms * 100000.0f) / 100000.0f;
        t["velocity_samples_300s"]    = w300.sample_count;
        t["velocity_status_300s"]     = VibWindow_StatusStr(w300.status);

      // [M1B-4] Timestamp-aware velocity slope over the M1B-1 ring.
      //
      // Regressed on ELAPSED SECONDS, so the unit is mm/s per second and is
      // independent of the bimodal 2.1 s / 3.9 s spacing. The legacy
      // rms_slope / slope_1s / slope_10s / slope_60s fields above are NOT
      // touched: they remain sample-index based over the legacy VRMS buffer
      // until M1B-8. These are new keys with a different unit, deliberately
      // named so the two can never be confused.
      //
      // Window is VIB_SLOPE_WINDOW_MS (300 s), stated explicitly in
      // vib_slope.h rather than inherited from TREND_WINDOW_SAMPLES -- those
      // 120 samples meant 30 s at 4 Hz but would span ~6.2 min at this cadence.
      //
      // velocity_slope_valid is the sole authority. On INSUFFICIENT_DATA the
      // slope is 0.0 meaning "not computed", never "flat trend".
      // velocity_slope_reseeded reports that a >90 s outage inside the window
      // truncated the regression to the newest contiguous segment.
      {
        VibSlopeResult sl;
        VibSlope_Compute(nowW, &sl);
        t["velocity_slope_mms_per_s"]  = roundf(sl.slope_mms_per_s * 1000000.0f) / 1000000.0f;
        t["velocity_slope_valid"]      = sl.valid;
        t["velocity_slope_reseeded"]   = sl.reseeded;
        t["velocity_slope_timestamp_ms"] = sl.timestamp_ms;

      // [M1B-5] Time-To-Warning. Computed HERE (taskAnalytics) because it reads
      // the M1B-1 ring via VibHistory_LatestValid(); the result travels to
      // /decision through g_ttwHours/g_ttwStatus. Shares `nowW` and `sl` with
      // the windows and slope above, so all four describe the same instant.
      //
      // In the current build VIB_WARNING_MMS and VIB_TTW_MIN_SLOPE are both
      // UNSET sentinels, so this can only produce THRESHOLDS_UNSET -- numeric
      // TTW is disabled by construction, with no fallback path.
      {
        VibTtwResult tw;
        VibTtw_Compute(g_motorRunState == MOTOR_RUNNING, &sl, VIB_WARNING_MMS,
                       nowW, VIB_VELOCITY_MAX_AGE_MS_TBD, &tw);
        if (tw.status == VIB_TTW_VALID) {
          g_ttwHours  = tw.hours;                 // value first ...
          g_ttwStatus = (uint8_t)tw.status;       // ... then VALID
        } else {
          g_ttwStatus = (uint8_t)tw.status;       // non-VALID first ...
          g_ttwHours  = 0.0f;                     // ... then clear
        }
      }
      }
      }
      t["trend_gap_s"]  = (uint32_t)g_lastResumeGapS;  // [v16.3ab] gap ครั้งล่าสุด (วินาที) — consumer รู้ว่า time-series ไม่ต่อเนื่องช่วงไหน

      // v16.1 FIX: snapshot ผ่าน mutex ก่อน access [v16.5.4: g_telemSnapshot, was g_vibData]
      // เพื่อป้องกัน race condition กับ taskStateMachine (Core 0)
      // ที่เป็นสาเหตุของ PANIC LoadProhibited EXCVADDR:0x00000009
      uint8_t snapMotorState = 0;
      float   snapRpm        = 0.0f;
      if (xSemaphoreTake(mutexVibData, pdMS_TO_TICKS(5)) == pdTRUE) {
          snapMotorState = g_telemSnapshot.motor_state;  // [v16.5.4] read the atomic snapshot --
          snapRpm        = g_telemSnapshot.rpm;          // motor_state + rpm now from the same cycle
          xSemaphoreGive(mutexVibData);
      }
      // [Phase2] freq_alert / freq_drift_x/y/z REMOVED with the freq_ratio
      // pipeline. slot_dur_ms / slot_revs_target REMOVED with the RPM-adaptive
      // slot machinery. The raw dominant frequencies (registers 0x44-0x46) are
      // unaffected and still published on /sensor as freq_x/y/z.
      t["timestamp"] = tsA;
 
      // buf[1024] >> StaticJsonDocument<640>  (v14.3: was buf[700])
      char buf[1536];  // [M1B-3] 1024 -> 1536, matches the enlarged /trend doc
      size_t sz = serializeJson(t, buf, sizeof(buf));
      if (sz == 0 || sz >= sizeof(buf) - 1)
        Serial.printf("[WARN] /trend JSON truncated! sz=%u buf=%u\n",
                      (unsigned)sz, (unsigned)sizeof(buf));
      // [v16.5] Section 7 Item 6 (design v16.5 §3.2) — enqueue for Network4G to
      // publish instead of calling mqttClient.publish() directly; Analytics is
      // not the owner task. Non-blocking; drop-newest + g_trendEnqueueDropCount
      // on a full queue are handled inside enqueueMqttOutbound() (Item 3), unchanged here.
      if (enqueueMqttOutbound(MQTT_OUTBOUND_TOPIC_TREND, buf, sz, MQTT_QOS))
        Serial.printf("[TREND] /trend %u B queued\n", (unsigned)sz);
      else
        Serial.printf("[TREND] FAILED to queue -> %s\n", g_mqttTopicTrend);
    }

    // ──────────────────────────────────────────────────────────────────────
  }  // end while(1)
}  // end taskAnalytics

// ============================================================================
// SETUP
// ============================================================================

// ── v15.3: Reset Reason Logger ──────────────────────────────────────────────
// เรียกแรกสุดใน setup() หลัง Serial.begin
// อ่าน esp_reset_reason() จาก hardware register (ไม่ขึ้นกับ UART/Serial)
// บันทึก reboot count ลง NVS (ไม่ reset เมื่อ power cycle)
// ──────────────────────────────────────────────────────────────────────────

void logResetReason() {
  esp_reset_reason_t reason = esp_reset_reason();
  g_resetReasonCode = (uint8_t)reason;

  // Map reason code → human-readable string + severity
  const char* emoji = "";
  switch (reason) {
    case ESP_RST_POWERON:
      strncpy(g_resetReasonStr, "POWER_ON",    sizeof(g_resetReasonStr));
      emoji = "✓";
      break;
    case ESP_RST_EXT:
      strncpy(g_resetReasonStr, "EXT_PIN",     sizeof(g_resetReasonStr));
      emoji = "✓";  // กดปุ่ม RESET
      break;
    case ESP_RST_SW:
      strncpy(g_resetReasonStr, "SW_RESET",    sizeof(g_resetReasonStr));
      emoji = "✓";  // esp_restart() / OTA
      break;
    case ESP_RST_BROWNOUT:
      strncpy(g_resetReasonStr, "BROWNOUT",    sizeof(g_resetReasonStr));
      emoji = "⚠";   // ไฟตก -- ตรวจ PSU / 4G current spike
      break;
    case ESP_RST_TASK_WDT:
      strncpy(g_resetReasonStr, "TASK_WDT",    sizeof(g_resetReasonStr));
      emoji = "!!";  // Task ค้าง > 30s
      break;
    case ESP_RST_INT_WDT:
      strncpy(g_resetReasonStr, "INT_WDT",     sizeof(g_resetReasonStr));
      emoji = "!!";  // Interrupt ค้าง
      break;
    case ESP_RST_PANIC:
      strncpy(g_resetReasonStr, "PANIC",       sizeof(g_resetReasonStr));
      emoji = "!!";  // Exception / Stack overflow
      break;
    case ESP_RST_DEEPSLEEP:
      strncpy(g_resetReasonStr, "DEEP_SLEEP",  sizeof(g_resetReasonStr));
      emoji = "✓";
      break;
    default:
      snprintf(g_resetReasonStr, sizeof(g_resetReasonStr), "UNKNOWN_%d", (int)reason);
      emoji = "?";
      break;
  }

  // อ่าน + อัปเดต reboot count จาก NVS
  Preferences resetPrefs;
  if (resetPrefs.begin("boot", false)) {
    g_rebootCount = resetPrefs.getUInt("count", 0) + 1;
    // [Task 7.3 -- TEMPORARY DIAGNOSTIC ONLY] NVS write entry/exit markers.
    // NOTE: setup()-time only, before any task/FIFO activity exists --
    // instrumented for completeness, not expected to correlate with
    // anything in a 240s runtime capture.
    Serial.printf("[NVS_BEGIN] %lu\n", (unsigned long)millis());
    resetPrefs.putUInt("count", g_rebootCount);
    resetPrefs.end();
    Serial.printf("[NVS_END]   %lu\n", (unsigned long)millis());
  }

  // Print banner
  Serial.println();
  Serial.println("+========================================================+");
  Serial.printf( "|  [BOOT] Reset Reason : %-6s %s\n", g_resetReasonStr, emoji);
  Serial.printf( "|  [BOOT] Reboot Count : #%lu\n", (unsigned long)g_rebootCount);
  Serial.println("+========================================================+");

  // Extra warning สำหรับสาเหตุที่ต้องสอบสวน
  if (reason == ESP_RST_BROWNOUT) {
    Serial.println("[BOOT] !! BROWNOUT detected -- ตรวจสอบ PSU / 4G current spike");
    Serial.println("[BOOT] !! ควรเพิ่ม capacitor หรือ separate power rail สำหรับ modem");
  } else if (reason == ESP_RST_TASK_WDT) {
    Serial.println("[BOOT] !! TASK WATCHDOG -- task ค้างเกิน 30s");
    Serial.println("[BOOT] !! ดู task ที่ไม่ได้ reset WDT ก่อน timeout");
  } else if (reason == ESP_RST_INT_WDT) {
    Serial.println("[BOOT] !! INTERRUPT WATCHDOG -- ISR ค้างนานเกินไป");
  } else if (reason == ESP_RST_PANIC) {
    Serial.println("[BOOT] !! PANIC/EXCEPTION -- ดู backtrace ใน log ก่อนหน้า");
    Serial.println("[BOOT] !! สาเหตุที่พบบ่อย: stack overflow, null ptr, heap corruption");
  }
}

// ============================================================================
// [BUILD FINGERPRINT] Four separate, immutable-after-boot fields, each with
// its own accessor, so future MQTT/REST diagnostics can reuse any individual
// value directly -- without ever parsing BUILD_ID apart to get one piece
// back out of it. BUILD_ID itself is a pure composition of these four (see
// buildBuildId() below): it is built BY concatenating getFwVersion() /
// getGitCommitHash() / getBuildDate() / getBuildTime(), never the other way
// around, so BUILD_ID and the individual fields can never disagree with
// each other.
//
// FW_VERSION / GIT_COMMIT_HASH are already single-literal #define constants
// (see the top-of-file include block); g_buildDate/g_buildTime are the
// runtime-normalized counterparts of __DATE__/__TIME__ (YYYYMMDD / HHMM),
// computed ONCE by buildBuildId() (called from setup()) and never written
// again afterward.
//
// Format:  <FW_VERSION>-<GIT_COMMIT_HASH>-<BUILD_DATE>-<BUILD_TIME>
// Example: 16.5-8208b95-20260723-1342
// Example (dirty tree):   16.5-8208b95-dirty-20260723-1342
// Example (no git info):  16.5-UNKNOWN-20260723-1342
//
// GIT_COMMIT_HASH is never fabricated -- see build_info.h / generate_build_info.ps1.
// ============================================================================
static char g_buildDate[9]  = {0};  // "YYYYMMDD" + NUL
static char g_buildTime[5]  = {0};  // "HHMM" + NUL
static char g_buildId[56]   = {0};  // composition of the four fields below

static const char* getFwVersion()     { return FW_VERSION; }
static const char* getGitCommitHash() { return GIT_COMMIT_HASH; }
static const char* getBuildDate()     { return g_buildDate; }
static const char* getBuildTime()     { return g_buildTime; }

// [BUILD FINGERPRINT] Accessor for future MQTT/diagnostic use -- read-only,
// zero cost (returns the buffer computed once at boot).
static const char* getBuildId() {
  return g_buildId;
}

static void buildBuildId() {
  // __DATE__ format: "Mmm dd yyyy" (day may be space-padded, e.g. "Jul  5 2026")
  // __TIME__ format: "hh:mm:ss"
  static const char* months[] = { "Jan","Feb","Mar","Apr","May","Jun",
                                   "Jul","Aug","Sep","Oct","Nov","Dec" };
  char monStr[4] = {0};
  int  day = 0, year = 0;
  sscanf(__DATE__, "%3s %d %d", monStr, &day, &year);
  int mon = 1;
  for (int i = 0; i < 12; i++) {
    if (strncmp(monStr, months[i], 3) == 0) { mon = i + 1; break; }
  }
  int hh = 0, mm = 0, ss = 0;
  sscanf(__TIME__, "%d:%d:%d", &hh, &mm, &ss);

  // Populate the two normalized fields FIRST -- these are the immutable
  // source data BUILD_ID is composed from below, not a byproduct of it.
  snprintf(g_buildDate, sizeof(g_buildDate), "%04d%02d%02d", year, mon, day);
  snprintf(g_buildTime, sizeof(g_buildTime), "%02d%02d", hh, mm);

  // BUILD_ID: pure composition of the four fields via their own getters --
  // cannot diverge from them, since it is built directly from the same
  // values a future caller would get by calling those getters itself.
  snprintf(g_buildId, sizeof(g_buildId), "%s-%s-%s-%s",
           getFwVersion(), getGitCommitHash(), getBuildDate(), getBuildTime());
}

void setup() {
  Serial.begin(115200);
  Serial.setTxTimeoutMs(0);  // [R-1C] non-blocking USB CDC TX -- see HWCDC.cpp write(): with a host attached but not draining, isPlugged() stays true so write() takes the blocking branch and stalls up to 20*100ms, starving Core 0's 10ms FIFO service and overflowing the 2048B RS485 RX ring (178ms budget @115200) -> ERR_RX_OVERFLOW. Timeout 0 drops output instead of blocking.
  delay(1000);

  // [v16.3v] โหลด NVS Config ก่อนสิ่งอื่น
  loadNvsConfig();

  // [v16.3v] Config Mode — กด Enter ภายใน 5 วินาทีเพื่อตั้งค่า
  Serial.println("\n[CONFIG] Press ENTER within 5 seconds to enter Config Mode...");
  uint32_t cfgStart = millis();
  bool enterConfig = false;
  while (millis() - cfgStart < 5000) {
    if (Serial.available()) {
      char c = Serial.read();
      if (c == '\n' || c == '\r') { enterConfig = true; break; }
    }
    // แสดง countdown ทุก 1 วินาที
    uint32_t remaining = 5 - (millis() - cfgStart) / 1000;
    static uint32_t lastPrint = 0;
    if (millis() - lastPrint >= 1000) {
      Serial.printf("[CONFIG] %lu...\n", remaining);
      lastPrint = millis();
    }
  }
  if (enterConfig) {
    runConfigMode();  // ไม่ return — จบด้วย ESP.restart()
  }
  Serial.println("[CONFIG] Boot normal\n");

  // FIX-WDT (v14.9): Extend Task Watchdog timeout to 30s.
  // Arduino ESP32 core default is 5s -- too short for 4G modem operations
  // (network registration ~10s, TLS handshake ~2s, MQTT publish over 4G).
  // esp_task_wdt_reconfigure() replaces the IDF default config at runtime.
  // trigger=true: panic+reboot on timeout (same as default behavior).
  {
    esp_task_wdt_config_t wdt_cfg = {
      .timeout_ms    = 30000,  // 30 seconds
      .idle_core_mask = (1 << 0) | (1 << 1),  // Watch both CPU0 and CPU1 IDLE
      .trigger_panic  = true,
    };
    esp_task_wdt_reconfigure(&wdt_cfg);
  }

  Serial.println("\n\n");
  Serial.println("+========================================================+");
  // [BUILD FINGERPRINT] Version number removed from this line -- FW_VERSION
  // is now the ONLY firmware-version string literal in the file (see the
  // banner printed a few lines below, and BUILD_FINGERPRINT.md).
  Serial.println("|  ESP32-S3 VIBRATION MONITOR (Phase 5 Fusion AI)        |");
  Serial.println("|        LilyGO T-Vending S3 + SIMCom A7670             |");
  Serial.println("+========================================================+\n");

  // ==========================================================================
  // [BUILD FINGERPRINT] Printed once, here, at boot -- zero runtime cost
  // afterward. buildBuildId() runs exactly once; g_buildId is then held for
  // the rest of runtime as the single source of truth (see getBuildId()).
  // Does not replace or alter the banner above -- purely additive.
  // ==========================================================================
  buildBuildId();
  {
    const char* motorSrcStr =
      (g_motorStateSource == MOTOR_SRC_CURRENT)   ? "MOTOR_SRC_CURRENT"   :
      (g_motorStateSource == MOTOR_SRC_PROXIMITY) ? "MOTOR_SRC_PROXIMITY" :
                                                      "MOTOR_SRC_RPM";
    Serial.println("================================================");
    Serial.println("PROMLOGIX PDM IIOT");
    Serial.printf ("Firmware      : v%s\n", getFwVersion());
    Serial.printf ("Git Commit    : %s\n", getGitCommitHash());
    Serial.printf ("Build Date    : %s\n", __DATE__);   // human-readable form; getBuildDate() holds the normalized YYYYMMDD form BUILD_ID is composed from
    Serial.printf ("Build Time    : %s\n", __TIME__);   // human-readable form; getBuildTime() holds the normalized HHMM form BUILD_ID is composed from
    Serial.printf ("Motor Source  : %s\n", motorSrcStr);
    Serial.println("================================================");
    Serial.printf ("BUILD_ID: %s\n\n", getBuildId());
#ifdef DEBUG_MODEM_DIAG
    // [v16.5b] Issue #2 Phase 1 -- announce the diagnostic build in the boot
    // banner so a capture can never be mistaken for a plain production log.
    Serial.printf("[MODEM-DIAG] ENABLED -- read-only A7670E probe: period=%lums "
                  "quiet=%lums at_timeout=%lums (NORMAL 30s cadence only)\n\n",
                  (unsigned long)MODEM_DIAG_PERIOD_MS,
                  (unsigned long)MODEM_DIAG_QUIET_MS,
                  (unsigned long)MODEM_DIAG_AT_TMO_MS);
#endif
  }

  // Print CPU info
  Serial.printf("CPU Frequency: %d MHz\n", ESP.getCpuFreqMHz());
  Serial.printf("Flash Size: %d MB\n", ESP.getFlashChipSize() / (1024 * 1024));
  Serial.printf("Free Heap: %d bytes\n\n", ESP.getFreeHeap());

  // Build MQTT topic from identity (Phase 1)
  // ── MQTT Pipeline Topic Init (mqtt_pipeline_patch §D) ───────────────────────
  // Legacy backward-compat topic (/vibration) — field names UNCHANGED
  snprintf(g_mqttTopic, sizeof(g_mqttTopic),
           "factory/%s/machine/%s/vibration", PLANT_ID, MACHINE_ID);

  // Repurpose existing char arrays (no memory increase)
  //   g_mqttAnalyticsTopic  -> /trend   (60-s cadence, was /analytics)
  snprintf(g_mqttAnalyticsTopic, sizeof(g_mqttAnalyticsTopic),
           "factory/%s/machine/%s/trend",  PLANT_ID, MACHINE_ID);

  // New pipeline-stage topics (declared in §A)
  snprintf(g_mqttTopicSensor,     sizeof(g_mqttTopicSensor),
           "factory/%s/machine/%s/sensor",      PLANT_ID, MACHINE_ID);
  snprintf(g_mqttTopicDecision,   sizeof(g_mqttTopicDecision),
           "factory/%s/machine/%s/decision",    PLANT_ID, MACHINE_ID);
  snprintf(g_mqttTopicTrend,      sizeof(g_mqttTopicTrend),
           "factory/%s/machine/%s/trend",       PLANT_ID, MACHINE_ID);
  snprintf(g_mqttTopicEvent,      sizeof(g_mqttTopicEvent),
           "factory/%s/machine/%s/vibration/event",  PLANT_ID, MACHINE_ID);  // V14.4
  // [Commit 7A] Inbound-only topic. Subscribed after every successful
  // connect (taskNetwork, see mqttClient.subscribe() call site) -- never
  // published to. Registration of the message callback itself
  // (mqttClient.onMessage()) happens once, below, independent of topic
  // string construction.
  snprintf(g_mqttTopicCommand,    sizeof(g_mqttTopicCommand),
           "factory/%s/machine/%s/vibration/command", PLANT_ID, MACHINE_ID);
  // [P1-S3] Diagnostics topic. Exists because P1-S2 froze /vibration to the
  // Product-1 contract and nine engineering fields lost their only publication
  // path with it -- see the PUBLISH 4 of 4 block in publishTelemetry().
  snprintf(g_mqttTopicDeviceHealth, sizeof(g_mqttTopicDeviceHealth),
           "factory/%s/machine/%s/device-health", PLANT_ID, MACHINE_ID);

  Serial.println("[Init] MQTT Pipeline Topics:");
  Serial.printf("  /vibration (compat): %s\n", g_mqttTopic);
  Serial.printf("  /sensor:             %s\n", g_mqttTopicSensor);
  Serial.printf("  /decision:           %s\n", g_mqttTopicDecision);
  Serial.printf("  /trend:              %s\n", g_mqttTopicTrend);
  Serial.printf("  /event:              %s\n", g_mqttTopicEvent);            // V14.4
  Serial.printf("  /command (inbound):  %s\n", g_mqttTopicCommand);          // [Commit 7A]
  Serial.printf("  /device-health:      %s\n", g_mqttTopicDeviceHealth);       // [P1-S3]

  // [Commit 7A] Register the inbound message callback once, at boot --
  // independent of connection state (the library dispatches to this
  // callback from mqttClient.loop(), already called unconditionally in
  // taskNetwork() at ~100ms cadence; no new polling loop is introduced).
  // Infrastructure only: the callback logs and returns. It does not parse,
  // validate, or enqueue anything -- that is Commit 7B's job, not this
  // one's. It never touches queueFifoTrigger and never calls
  // FifoDriver_Request().
  mqttClient.onMessage(mqttCommandCallback);
  Serial.printf("[Init] Identity: plant=%s  machine=%s  sensor=%s  rated_rpm=%d\n",
                PLANT_ID, MACHINE_ID, SENSOR_ID, RATED_RPM);
  // ─────────────────────────────────────────────────────────────────────────────

  // Initialize GPIO
  pinMode(PIN_BUTTON, INPUT_PULLUP);
  pinMode(PIN_BUTTON_ENTER, INPUT_PULLUP);
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(RS485_EN_PIN, OUTPUT);
  pinMode(BUILTIN_LED, OUTPUT);
  pinMode(PIN_LED_CLOUD, OUTPUT);
  digitalWrite(PIN_BUZZER, LOW);
  digitalWrite(BUILTIN_LED, LOW);
  digitalWrite(PIN_LED_CLOUD, LOW);
  rs485Enable("SETUP");

  Serial.println("[Init] GPIO configured");

  // Initialize I2C and OLED
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(400000);  // 400 kHz

  u8g2.begin();
  u8g2.setContrast(255);

  // Splash screen
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_ncenB14_tr);
  u8g2.drawStr(25, 20, "4G LTE");
  u8g2.setFont(u8g2_font_ncenB10_tr);
  u8g2.drawStr(10, 40, "Vib Monitor");
  u8g2.setFont(u8g2_font_6x10_tr);
  u8g2.drawStr(30, 55, MACHINE_NAME);
  u8g2.drawStr(20, 64, "SIMCom A7670");
  u8g2.sendBuffer();

  Serial.println("[Init] OLED initialized");

  // Initialize RTC
  if (rtc.begin()) {
    g_rtcValid = true;
    DateTime now = rtc.now();
    DateTime compiled(F(__DATE__), F(__TIME__));

    if (rtc.lostPower()) {
      // RTC battery died -- set compile time as temporary fallback
      // This will be corrected by NTP sync once 4G connects
      Serial.println("[Init] RTC lost power! Setting compile time as fallback...");
      Serial.println("[Init] ! Time will be corrected via NTP after 4G connects");
      rtc.adjust(compiled);
      g_timeSync.synced = false;  // Mark as not synced
    } else if (now.year() < 2024 || now.year() > 2035) {
      // [v16.3t] FIX: ลบเงื่อนไข unixtime < compiled ออก
      // เหตุผล: compiled เป็น local time +7 แต่ RTC เก็บ UTC
      //         → now.unixtime() < compiled.unixtime() เป็น true เสมอ (ต่างกัน 7 ชั่วโมง)
      //         → ทำให้ reset เวลา RTC ทุก boot แม้ battery ดี
      // แก้: ตรวจแค่ปี 2024-2035 ก็เพียงพอ ถ้านอกช่วงค่อย fallback compile time
      Serial.println("[Init] RTC time appears invalid, setting compile time as fallback");
      rtc.adjust(compiled);
      g_timeSync.synced = false;
    } else {
      Serial.println("[Init] RTC time looks reasonable (will verify via NTP)");
    }

    now = rtc.now();
    Serial.printf("[Init] RTC: %04d-%02d-%02d %02d:%02d:%02d (sync pending)\n",
                  now.year(), now.month(), now.day(),
                  now.hour(), now.minute(), now.second());
  } else {
    Serial.println("[Init] RTC not found! Timestamps will be unavailable until NTP sync.");
    g_rtcValid = false;
  }

  // Initialize Modbus
  // [v16.6.1-fifo] RX buffer sized for FIFO dump transfers (SDS P-1): the
  // default 256B buffer gives only ~267ms of headroom against the 250ms
  // taskModbusRead() cadence (6% margin) -- insufficient once a 6146B FIFO
  // dump exists to receive. 2048B gives 8.5x margin. Normal Modbus responses
  // are <=37B, so this has no effect on existing polling behavior.
  SerialRS485.setRxBufferSize(2048);
  SerialRS485.begin(MODBUS_BAUDRATE, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);
  modbus.begin(MODBUS_SLAVE_ID, SerialRS485);

  // [Task 4.1] Bind the FIFO transport and initialize the driver, exactly
  // once, right after SerialRS485.begin()/modbus.begin() -- satisfies
  // Uart485Transport_Init()'s documented precondition (serial->begin()
  // already called) and FifoDriver_Init()'s "once per boot" contract.
  // [Broker, Commit 1] The driver is no longer permanently dormant: the
  // commissioning trigger (taskModbusRead()) enqueues onto queueFifoTrigger,
  // and taskModbusRead()'s drain block calls FifoDriver_Request() once its
  // runtime gate is satisfied. This Init() call itself still changes no
  // behavior on its own -- it only binds the transport.
  Uart485Transport_Init(&g_fifoTransport, &SerialRS485);
  FifoDriver_Init(&g_fifoTransport);
  // [Task 4.4 -- TEMPORARY DIAGNOSTIC ONLY, retry-root-cause investigation,
  // not a permanent production feature] Register a Serial-backed sink for
  // fifo_driver.cpp's optional diagnostic hook (default nullptr/no-op) --
  // a non-capturing lambda converts to the plain C function pointer
  // FifoDriver_SetDiagLogger() expects, so no separate named function or
  // forward declaration is needed. Intended to be removed, along with
  // FifoDriver_SetDiagLogger() itself and every DiagLog() call site it
  // guards in fifo_driver.cpp, once this investigation concludes.
  // [v16.6 logging refactor] Gated at LOG_TRACE: below that level the
  // registration call itself is skipped, so s_diagLog stays at its default
  // nullptr and fifo_driver.cpp's DiagLog() no-ops on its existing
  // `if (!s_diagLog) return;` check -- no vsnprintf formatting cost is paid
  // per call, not just the Serial write.
#if LOG_LEVEL >= LOG_TRACE
  FifoDriver_SetDiagLogger([](const char* msg) { LOGT("%s\r\n", msg); });
#endif
  // [Task 7.1 -- TEMPORARY DIAGNOSTIC ONLY, UART receive-error
  // instrumentation & validation, not a permanent production feature]
  // Registers fifo_driver.cpp's pull-based UART-stats hooks with the
  // concrete Uart485Transport_* accessors -- this is the ONLY place in the
  // whole project that lets fifo_driver.cpp (transport-agnostic) obtain
  // UART-485-specific data, without ever #including fifo_transport_uart485.h
  // itself. Intended to be removed, along with FifoDriver_SetUartDiagHooks()
  // and its call sites in fifo_driver.cpp/.h, once this investigation
  // concludes.
  {
    FifoUartDiagHooks uartHooks{};
    uartHooks.getStats = [](uint32_t* outFifoOvf, uint32_t* outBufferFull,
                             uint32_t* outBreak, uint32_t* outFrameErr,
                             uint32_t* outParityErr) {
      Uart485Transport_GetErrorCounts(outFifoOvf, outBufferFull, outBreak,
                                       outFrameErr, outParityErr);
    };
    uartHooks.resetStats = []() { Uart485Transport_ResetErrorCounts(); };
    FifoDriver_SetUartDiagHooks(uartHooks);
  }
  // [Task 7.4 -- TEMPORARY DIAGNOSTIC ONLY, ReadDump timing audit, not a
  // permanent production feature] Registers Arduino's micros() as
  // fifo_driver.cpp's microsecond-clock pull hook. Intended to be removed,
  // along with FifoDriver_SetMicrosProvider() and its call sites, once this
  // investigation concludes.
  FifoDriver_SetMicrosProvider([]() -> uint32_t { return micros(); });
  Serial.printf("[Init] FifoDriver initialized, phase=%d\n",
                static_cast<int>(FifoDriver_GetPhase()));

  Serial.printf("[Init] Modbus @ %d baud, ID: 0x%02X\n",
                MODBUS_BAUDRATE, MODBUS_SLAVE_ID);

  // [PATCHED v16.2] Boot-time sensor config with retry
  // -----------------------------------------------------------------------
  // reconfigSensorAfterRestart() เดิมถูกเรียกแค่ตอน stuck-auto-restart
  // ทำให้ทุก power cycle sensor กลับ default (MODE=0x00 → CF/VRMS = 0)
  // แก้โดยเรียก config ทุกครั้งที่ boot ก่อนสร้าง FreeRTOS tasks
  // ลำดับที่ทำงานอยู่จริง: Unlock → MODE=0x02(FreqDomain) → Unlock → Save
  // (ไม่มีการเขียน SR หรือ DRM -- ถูกถอดออกใน [PATCHED v16.3], ดู reconfigSensorAfterRestart())
  //
  // Retry 3 รอบ: sensor บางตัวใช้เวลา settle หลัง power-on นานกว่า 200ms
  // รอ 500ms ก่อน attempt แรก และ 300ms ระหว่าง retry
  // -----------------------------------------------------------------------
  Serial.println("[Init] Configuring WTVB02 sensor (MODE=FreqDomain)...");
  delay(500);  // [v16.2] เพิ่มจาก 200ms → 500ms ให้ sensor fully ready ก่อน config

  bool cfgOk = false;
  for (int attempt = 1; attempt <= 3 && !cfgOk; attempt++) {
    if (attempt > 1) {
      Serial.printf("[Init] Sensor config retry %d/3...\n", attempt);
      delay(300);
    }
    cfgOk = reconfigSensorAfterRestart();
  }

  if (cfgOk) {
    Serial.println("[Init] Sensor config OK -- CF/VRMS will be active");
  } else {
    Serial.println("[Init] WARNING: Sensor config partial after 3 attempts");
    Serial.println("[Init] MODE may still be set -- CF/VRMS active until next power cycle");
  }

  // -- Initialize Proximity / RPM Sensor (ISR-based, GPIO17) --
  loadRuntimeHour();                                       // ???? runtime ??????? NVS Flash
  loadFaultLatchNVS();   // v3: restore pending fault event from previous session (pre-task-creation, no lock required)
  pinMode(PIN_RPM, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_RPM), rpmISR, FALLING);
  g_rpmLastPulseMillis = millis();                         // ??????? false STOPPING ??? boot
  Serial.printf("[Init] RPM sensor GPIO%d | RATED=%d RPM +/-%d | PPR=%d\n",
                PIN_RPM, RATED_RPM, RATED_RPM_TOL, PULSE_PER_REV);

  // Create Mutexes
  mutexVibData    = xSemaphoreCreateMutex();
  mutexSystemState = xSemaphoreCreateMutex();
  mutexI2C        = xSemaphoreCreateMutex();
  mutexModem      = xSemaphoreCreateMutex();
  mutexFaultLatch = xSemaphoreCreateMutex();  // v3 hardened: guard g_fl/g_flCount + fault_latch NVS namespace
  mutexTelemBuf   = xSemaphoreCreateMutex();  // guards telemetry ring buffer
  mutexAccelSnap  = xSemaphoreCreateMutex();  // [Phase 3B] guards g_accelSnap
  mutexVelCarrier = xSemaphoreCreateMutex();  // [M1A] guards g_velCarrier (Core1->Core0)
  VibHistory_Init();                          // [M1B-1] trend history ring (Core 1 only)
  VibEma_Init();                              // [M1B-2] timestamp-aware EMA over that ring

  if (mutexVibData == NULL || mutexSystemState == NULL ||
      mutexI2C == NULL || mutexModem == NULL ||
      mutexFaultLatch == NULL) {
    Serial.println("[FATAL] Failed to create mutexes!");
    while (1) delay(1000);
  }

  Serial.println("[Init] Mutexes created");

  // Create Queues
  queueSensorData = xQueueCreate(QUEUE_SIZE_SENSOR, sizeof(VibrationData_t));
  queueButtonEvent = xQueueCreate(QUEUE_SIZE_BUTTON, sizeof(ButtonEvent_t));
  queueDisplayUpdate = xQueueCreate(QUEUE_SIZE_DISPLAY, sizeof(DisplayCommand_t));
  queueMaintEvent = xQueueCreate(QUEUE_SIZE_MAINT, sizeof(MaintenanceEvent_t));  // V14.4
  queueFifoTrigger = xQueueCreate(QUEUE_SIZE_FIFO_TRIGGER, sizeof(FifoTriggerIntent_t));  // [Broker, Commit 1]
  // [Phase 3B] Depth 1: analytics consumes at 1 Hz, captures arrive at ~0.5 Hz,
  // so a backlog means Core 1 is already behind and the freshest waveform is
  // the only one worth keeping. Deliberately NOT in the FATAL check below --
  // Phase 3 decision #3 makes the whole FIFO/acceleration path best-effort, so
  // failing to create it must degrade acceleration RMS only, never halt boot.
  queueAccelSnapshot = xQueueCreate(1, sizeof(AccelSnapshotReady_t));

  if (queueSensorData == NULL || queueButtonEvent == NULL || queueDisplayUpdate == NULL
      || queueMaintEvent == NULL || queueFifoTrigger == NULL) {
    Serial.println("[FATAL] Failed to create queues!");
    while (1) delay(1000);
  }

  Serial.println("[Init] Queues created");

  // [v16.5] Section 7 Item 3 — dormant outbound MQTT queue.
  // No producer/consumer wired yet (later checklist items); creation failure
  // here does not affect current runtime behavior, since enqueueMqttOutbound()
  // null-checks the handle and nothing calls it yet.
  queueMqttOutboundTrend = xQueueCreate(QUEUE_SIZE_MQTT_OUTBOUND, sizeof(MqttOutboundMsg_t));
  if (queueMqttOutboundTrend == NULL) {
    Serial.println("[WARN] Failed to create queueMqttOutboundTrend (dormant, no current consumer)");
  } else {
    Serial.println("[Init] MQTT outbound queue created (dormant)");
  }


  delay(2000);

  // ========================================================================
  // CREATE TASKS - CORE ASSIGNMENT
  // ========================================================================

  Serial.println("\n[Init] Creating FreeRTOS tasks...\n");

  // CORE 0 - Time-critical operations
  Serial.println("+--- CORE 0 (PRO_CPU) - Time Critical ---------------+");

  xTaskCreatePinnedToCore(
    taskModbusRead,
    "ModbusRead",
    STACK_SIZE_MODBUS,
    NULL,
    PRIORITY_MODBUS,
    &taskHandleModbus,
    0);
  Serial.printf("| [+] Modbus RTU       (Priority %d, Stack %d)      |\n",
                PRIORITY_MODBUS, STACK_SIZE_MODBUS);

  xTaskCreatePinnedToCore(
    taskStateMachine,
    "StateMachine",
    STACK_SIZE_STATE,
    NULL,
    PRIORITY_STATE,
    &taskHandleState,
    0);
  Serial.printf("| [+] State Machine    (Priority %d, Stack %d)      |\n",
                PRIORITY_STATE, STACK_SIZE_STATE);

  Serial.println("+----------------------------------------------------+\n");

  // CORE 1 - UI and network
  Serial.println("+--- CORE 1 (APP_CPU) - User Interface --------------+");

  xTaskCreatePinnedToCore(
    taskDisplayUpdate,
    "DisplayUpdate",
    STACK_SIZE_DISPLAY,
    NULL,
    PRIORITY_DISPLAY,
    &taskHandleDisplay,
    1);
  Serial.printf("| [+] OLED Display     (Priority %d, Stack %d)      |\n",
                PRIORITY_DISPLAY, STACK_SIZE_DISPLAY);

  xTaskCreatePinnedToCore(
    taskNetwork,
    "Network4G",
    STACK_SIZE_NETWORK,
    NULL,
    PRIORITY_NETWORK,
    &taskHandleNetwork,
    1);
  Serial.printf("| [+] 4G Modem/MQTT    (Priority %d, Stack %d)     |\n",
                PRIORITY_NETWORK, STACK_SIZE_NETWORK);

  xTaskCreatePinnedToCore(
    taskAnalytics,
    "Analytics",
    STACK_SIZE_ANALYTICS,
    NULL,
    PRIORITY_ANALYTICS,
    &taskHandleAnalytics,
    1);
  Serial.printf("| [+] Analytics        (Priority %d, Stack %d)      |\n",
                PRIORITY_ANALYTICS, STACK_SIZE_ANALYTICS);

  xTaskCreatePinnedToCore(
    taskButtonHandler,
    "ButtonHandler",
    STACK_SIZE_BUTTON,
    NULL,
    PRIORITY_BUTTON,
    &taskHandleButton,
    1);
  Serial.printf("| [+] Button Input     (Priority %d, Stack %d)      |\n",
                PRIORITY_BUTTON, STACK_SIZE_BUTTON);

  xTaskCreatePinnedToCore(
    taskBuzzerControl,
    "BuzzerControl",
    2048,
    NULL,
    PRIORITY_BUZZER,
    &taskHandleBuzzer,
    1);
  Serial.printf("| [+] Buzzer Control   (Priority %d, Stack %d)      |\n",
                PRIORITY_BUZZER, 2048);

  Serial.println("+----------------------------------------------------+\n");

  Serial.println("+========================================================+");
  Serial.println("|  SYSTEM READY -- 4G LTE + mTLS + Phase 5 Fusion AI     |");
  Serial.println("+========================================================+\n");

  Serial.println("Network Configuration:");
  Serial.printf("  APN:       %s\n", g_cfgApn);
  Serial.printf("  MQTT:      %s:%d (mTLS)\n", MQTT_SERVER, MQTT_PORT);
  Serial.printf("  Client ID: %s\n", MQTT_CLIENT_ID);
  Serial.printf("  QoS:       %d\n", MQTT_QOS);
  Serial.printf("  Topic:     %s\n\n", g_mqttTopic);

  // [S21b] Banner text only -- no threshold, logic or evaluation change.
  // Previously this block printed BASELINE_RMS/WARNING_RMS/CRITICAL_RMS under
  // the heading "Vibration Thresholds:", which reads as the active Product-1
  // alarm limits. They are not: they are legacy VRMS-register constants and
  // have never driven the vibration alarm. Every value below is printed from
  // its own #define so the banner can never drift from the constants.
  Serial.println("Product-1 Vibration Thresholds (PROMLOGIX PROVISIONAL V1):");
  Serial.println("  ISO-INFORMED / NOT ISO-CERTIFIED -- not an ISO limit");
  Serial.printf ("  WARNING        : %.1f mm/s RMS  (clears below %.1f)\n",
                 VIB_WARNING_MMS,  VIB_WARNING_OFF_MMS);
  Serial.printf ("  CRITICAL       : %.1f mm/s RMS  (clears below %.1f)\n",
                 VIB_CRITICAL_MMS, VIB_CRITICAL_OFF_MMS);
  Serial.printf ("  REFERENCE HIGH : %.1f mm/s RMS  (reference only -- not an alarm state)\n",
                 VIB_REFERENCE_HIGH_MMS);
  Serial.printf ("  Escalation     : %u consecutive FIFO captures\n",
                 (unsigned)VIB_ALARM_PERSIST_CAPTURES);
  Serial.printf ("  Source         : FIFO-DSP velocity_rms_overall\n\n");

  Serial.println("Adaptive Sending:");
  Serial.println("  NORMAL:   30 seconds");
  Serial.println("  WARNING:  10 seconds");
  Serial.println("  CRITICAL: 5 seconds\n");

  Serial.println("Phase 2 Analytics:");
  Serial.println("  taskAnalytics: Core 1, 1 Hz, Priority 3");
  Serial.println("  Trend source : VibHistory ring -> VibEma / VibWindow / VibSlope / VibTtw");
  Serial.printf("  Temp history : %d bytes (%u samples)\n",
                (int)sizeof(g_tempBuf), (unsigned)TEMP_BUF_SIZE);
  Serial.printf("  Analytics topic: %s\n\n", g_mqttAnalyticsTopic);


  Serial.println("NTP Time Sync:");
  Serial.printf("  Check Interval: %lu minutes (once synced)\n", NTP_CHECK_INTERVAL_MS / 60000UL);
  Serial.printf("  Retry Interval: %lu seconds (while unsynced)\n", NTP_RETRY_INTERVAL_MS / 1000UL);
  Serial.printf("  Drift Warn:     %d seconds\n", NTP_DRIFT_WARN_SEC);
  Serial.printf("  Drift Max:      %d seconds\n\n", NTP_DRIFT_MAX_SEC);

  Serial.println("FreeRTOS scheduler will now take over...\n");
}

// ============================================================================
// MAIN LOOP (Runs on Core 1 by default)
// ============================================================================

void loop() {
  // Statistics and monitoring (low priority background task)
  static uint32_t lastStats = 0;
  uint32_t now = millis();

  // Blink LED to show system is alive
  static uint32_t lastBlink = 0;
  if (now - lastBlink >= 1000) {
    digitalWrite(BUILTIN_LED, !digitalRead(BUILTIN_LED));
    lastBlink = now;
  }

  // [Enclosure v1] Cloud LED (GPIO15) - reflects real 4G/MQTT status
  // OFF = no signal, slow blink = connecting, fast blink = GPRS up but MQTT down, solid = MQTT connected
  static uint32_t lastCloudBlink = 0;
  {
    // [v16.5] Section 7 Item 8 (design v16.5 §3.3, §4.2) — read via cache
    // instead of touching mqttClient directly; loopTask is not the owner task.
    bool mqttUp = getMqttConnectedCached();
    bool gprsUp = g_network.gprsConnected;
    uint32_t cloudBlinkInterval = 0;

    if (mqttUp) {
      digitalWrite(PIN_LED_CLOUD, HIGH);
    } else if (gprsUp) {
      cloudBlinkInterval = 250;
    } else if (g_network.modemState == MODEM_STATE_OFF ||
               g_network.modemState == MODEM_STATE_ERROR) {
      digitalWrite(PIN_LED_CLOUD, LOW);
    } else {
      cloudBlinkInterval = 500;
    }

    if (cloudBlinkInterval > 0 && now - lastCloudBlink >= cloudBlinkInterval) {
      digitalWrite(PIN_LED_CLOUD, !digitalRead(PIN_LED_CLOUD));
      lastCloudBlink = now;
    }
  }

  if (now - lastStats >= 30000) {  // Every 30 seconds
    lastStats = now;

    Serial.println("\n+========================================================+");
    Serial.println("|              SYSTEM STATUS REPORT (4G mTLS)            |");
    Serial.println("+========================================================+");

    // Get current data
    // [v16.5.4] Improvement 2: one atomic snapshot copy (ONE mutex take)
    // replaces the previous g_vibData + g_systemState pair -- the reported
    // state and measurements now belong to the same cycle. Log format below
    // is unchanged.
    TelemetrySnapshot localSnap = {};
    if (xSemaphoreTake(mutexVibData, pdMS_TO_TICKS(10)) == pdTRUE) {
      memcpy(&localSnap, &g_telemSnapshot, sizeof(TelemetrySnapshot));
      xSemaphoreGive(mutexVibData);
    }
    const VibrationData_t& localVib   = localSnap.vib;
    const MachineState_t   localState = localSnap.alarm_level;

    Serial.printf("| Machine: %-45s |\n", MACHINE_NAME);

    const char* stateStr = "UNKNOWN";
    if (localState == STATE_NORMAL) stateStr = "NORMAL";
    else if (localState == STATE_WARNING) stateStr = "WARNING";
    else if (localState == STATE_CRITICAL) stateStr = "CRITICAL";
    else if (localState == STATE_MAINTENANCE) stateStr = "MAINTENANCE";
    Serial.printf("| State:   %-45s |\n", stateStr);

    {
      float sAll = 0.0f;
      if (displayVelocity(&sAll, NULL, NULL, NULL))
        Serial.printf("| Velocity:%-42.2f mm/s |\n", sAll);
      else
        Serial.printf("| Velocity:%-42s      |\n", "-- (unavailable)");
    }
    Serial.printf("| Temp:    %-43.1f  degC |\n", localVib.temperature);

    Serial.println("+========================================================+");
    Serial.printf("| Modem:   %-45s |\n", g_network.modemReady ? "READY" : "NOT READY");
    Serial.printf("| GPRS:    %-45s |\n", g_network.gprsConnected ? "CONNECTED" : "DISCONNECTED");
    // [v16.5] Section 7 Item 8 (design v16.5 §3.3, §4.2) — read via cache
    // instead of touching mqttClient directly; loopTask is not the owner task.
    Serial.printf("| MQTT:    %-45s |\n", getMqttConnectedCached() ? "CONNECTED (mTLS)" : "DISCONNECTED");
    Serial.printf("| Signal:  %d%% (CSQ: %d)                                |\n",
                  g_network.signalPercent, g_network.signalQuality);
    Serial.printf("| Operator: %-44s |\n", g_network.operatorName);

    Serial.println("+========================================================+");
    Serial.printf("| Sensor Reads:    %8lu (Errors: %8lu)       |\n",
                  g_sensorReads, g_sensorErrors);
    Serial.printf("| Display Updates: %8lu                          |\n",
                  g_displayUpdates);
    Serial.printf("| MQTT Publishes:  %8u (Failures: %8u)     |\n",
                  g_network.publishCount, g_network.publishFailures);

    // Vx / Vy / Vz Stuck Recovery Status
    Serial.println("+========================================================+");
    Serial.printf("| Stuck Threshold: %2u reads x 250ms = %.2f s           |\n",
                  STUCK_THRESHOLD, STUCK_THRESHOLD * 0.25f);
    Serial.printf("| Vx Stuck Count:  %3u / %u   (Restarts: %3u)           |\n",
                  g_vxStuckCount, STUCK_THRESHOLD, g_vxRestartCount);
    Serial.printf("| Vy Stuck Count:  %3u / %u   (Restarts: %3u)           |\n",
                  g_vyStuckCount, STUCK_THRESHOLD, g_vyRestartCount);
    Serial.printf("| Vz Stuck Count:  %3u / %u   (Restarts: %3u)           |\n",
                  g_vzStuckCount, STUCK_THRESHOLD, g_vzRestartCount);
    if (g_lastSensorRestart > 0) {
      uint32_t ageSec = (millis() - g_lastSensorRestart) / 1000;
      Serial.printf("| Last Restart:    %lu sec ago                       |\n", ageSec);
    } else {
      Serial.printf("| Last Restart:    never                              |\n");
    }

    Serial.println("+========================================================+");
    Serial.printf("| Free Heap:       %8d bytes                     |\n",
                  ESP.getFreeHeap());
    Serial.printf("| Min Free Heap:   %8d bytes                     |\n",
                  ESP.getMinFreeHeap());
    Serial.printf("| Uptime:          %8lu seconds                  |\n",
                  millis() / 1000);

    // RTC time if available
    if (g_rtcValid) {
      DateTime rtcNow; RTC_NOW_SAFE(rtcNow);  // [v16.3g]
      // [v16.3j] year sanity check ก่อน print
      // ถ้าได้ปี 2106 = 0xFF corruption จาก I2C collision กับ OLED
      if (rtcNow.year() >= 2024 && rtcNow.year() <= 2035) {
        Serial.printf("| RTC Time: %04d-%02d-%02d %02d:%02d:%02d                    |\n",
                      rtcNow.year(), rtcNow.month(), rtcNow.day(),
                      rtcNow.hour(), rtcNow.minute(), rtcNow.second());
      } else {
        Serial.println("|  RTC Time: INVALID (I2C read error)            |");
      }
    }

    // NTP Time Sync status
    Serial.println("+========================================================+");
    Serial.printf("| NTP Synced:  %-41s |\n", g_timeSync.synced ? "YES +" : "NO x");
    if (g_timeSync.synced) {
      uint32_t ageSec = (millis() - g_timeSync.lastSyncMillis) / 1000;
      Serial.printf("| Last Sync:   %-41s |\n", g_timeSync.lastSyncTime);
      Serial.printf("| Sync Age:    %lu seconds ago                        |\n", ageSec);
      // [v16.3j] ถ้า drift > 1 ปี = ค่าขยะจาก I2C collision → แสดง OK แทน
      const int32_t ONE_YEAR_SEC = 365L * 24 * 3600;
      if (g_timeSync.lastDriftSec > ONE_YEAR_SEC ||
          g_timeSync.lastDriftSec < -ONE_YEAR_SEC) {
        Serial.println("|  Last Drift: OK (I2C glitch suppressed)         |");
      } else {
        Serial.printf("| Last Drift:  %+d seconds                            |\n",
                      g_timeSync.lastDriftSec);
      }
    }
    Serial.printf("| Sync Count:  %lu  (Failures: %lu)                    |\n",
                  g_timeSync.syncCount, g_timeSync.syncFailures);

    Serial.println("+========================================================+\n");

    // Task stack watermarks (debug info)
    Serial.println("Task Stack High Water Marks (bytes remaining):");
    Serial.printf("  Modbus:      %u\n", uxTaskGetStackHighWaterMark(taskHandleModbus));
    Serial.printf("  State:       %u\n", uxTaskGetStackHighWaterMark(taskHandleState));
    Serial.printf("  Display:     %u\n", uxTaskGetStackHighWaterMark(taskHandleDisplay));
    Serial.printf("  Network:     %u\n", uxTaskGetStackHighWaterMark(taskHandleNetwork));
    Serial.printf("  Analytics:   %u\n", uxTaskGetStackHighWaterMark(taskHandleAnalytics));
    Serial.printf("  Button:      %u\n", uxTaskGetStackHighWaterMark(taskHandleButton));
    Serial.printf("  Buzzer:      %u\n\n", uxTaskGetStackHighWaterMark(taskHandleBuzzer));

    // [Phase2] FIFO/DSP trend status (was: legacy slot buffers + legacy EMA)
    {
      const VibEmaState emaSnap = VibEma_Get(millis(), VIB_VELOCITY_MAX_AGE_MS_TBD);
      Serial.printf("Trend history: %u samples  |  temp ring: %u/%u\n",
                    (unsigned)VibHistory_Count(),
                    (unsigned)g_tempCount, (unsigned)TEMP_BUF_SIZE);
      if (emaSnap.valid) Serial.printf("Velocity EMA: %.3f mm/s\n", emaSnap.ema_mms);
      else               Serial.printf("Velocity EMA: -- (unavailable)\n");
    }

    // Telemetry ring buffer stats
    Serial.println("+========================================================+");
    Serial.printf("| TelemBuf Pending:  %3u / %3u slots                     |\n",
                  (unsigned)g_telemBufCount, TELEM_BUF_SIZE);
    Serial.printf("| TelemBuf Overflow: %8lu  (oldest overwritten)      |\n",
                  (unsigned long)g_telemBufOverflowCount);
    Serial.printf("| TelemBuf Replayed: %8lu  (cumulative sent OK)      |\n",
                  (unsigned long)g_telemBufReplayedCount);
    Serial.println("+========================================================+\n");
  }

  // This loop runs at low priority on Core 1
  // Main work is done by FreeRTOS tasks
  vTaskDelay(pdMS_TO_TICKS(1000));  // Sleep for 1 second
}
