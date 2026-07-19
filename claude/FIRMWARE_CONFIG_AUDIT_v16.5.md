# Firmware Configuration Audit — WTVB02_ESP32S3_V16_CM_fault_latch_v3_hardened_patched_v16_5

**Source file:** `WTVB02_ESP32S3_V16_CM_fault_latch_v3_hardened_patched_v16_5.ino` (7,666 lines, single-file sketch — no other `.ino`/`.h`/`.cpp` in the sketch directory)
**Audit date:** 2026-07-18
**Scope:** every `#define`, `constexpr`, global `const`, and conditional-compilation configuration macro in the file. **185 items total** (183 extracted by exact regex match against the source + 2 manually added: `TEST_CURRENT_SOURCE`, a build-flag macro with no in-file `#define`, and `VERIFY_TEST`, a commented-out `#define` that still gates real code).

## Methodology (verification, not inference)

Every row below was produced mechanically from the source file, not recalled or inferred:
1. All `#define NAME` lines extracted via regex, with line numbers.
2. All top-level (column-0, non-function-local) `const`/`constexpr` globals extracted the same way — this deliberately **excludes** function-local `static const` values (e.g. `kExpectedFullLoadA`, `kThresholdA` inside `buildMotorStateEvidence()`), since those are derived computations, not independently configurable constants.
3. For every name, every other occurrence in the file was located via whole-word grep, excluding the definition line itself. That count is the "Referenced at" column and directly drives "Unused?" — **0 other occurrences = UNUSED**, verified per-item, not assumed.
4. Every zero-reference candidate was individually re-checked by direct grep before being labeled `UNUSED` (see the Unused Constants appendix for specifics and, where found, the reason).
5. "Motor State?" = Yes only for constants that feed the `g_motorRunState` decision chain (`buildMotorStateEvidence()` → `updateMotorStateMachine()`), traced by hand against the actual call graph — not by name pattern-matching.

**Known source artifact (not caused by this audit):** several Thai-language inline comments in the original file (`TREND_MIN_SAMPLES`, `STUCK_THRESHOLD`, `STUCK_MIN_RAW`, others) contain literal `?????` sequences — this is genuine mojibake already present in the source (confirmed via direct file read, not a shell-encoding artifact of this audit's tooling) and is reproduced verbatim below rather than guessed at.

## Summary

| Metric | Count |
|---|---|
| Total configuration items audited | 185 |
| Confirmed **UNUSED** (zero other references) | 13 |
| Directly affect the Motor State decision (`RUNNING`/`STARTING`/`STOPPING`/`STOPPED`) | 18 |
| Sections | 10 |

---

## Motor State (18 items)

| Name | Default | Def Line | Purpose | Referenced at (lines) | Runtime? | Motor State? | Type | Unused? |
|---|---|---|---|---|---|---|---|---|
| `TEST_CURRENT_SOURCE` | *(build flag; no in-file `#define`)* | N/A — passed via `-DTEST_CURRENT_SOURCE` compiler flag (`build.options.json`), not defined in this `.ino` | Selects `g_motorStateSource = MOTOR_SRC_CURRENT` at compile time; undefined → falls back to `MOTOR_SRC_RPM` | 2062-2071 (`#ifdef` selecting `g_motorStateSource`), 7129-7132 (boot-identity banner, diagnostic print only) | Yes | **Yes** | debug | used |
| `ABSENT_STOPPED_MS` | `30000` | 215 | ... > 30s -> STOPPED | 2785,2798 | Yes | **Yes** | timing | used |
| `ABSENT_STOPPING_MS` | `15000` | 214 | signalPresent false (but fresh) > 15s -> STOPPING | 2785,2801 | Yes | **Yes** | timing | used |
| `COLD_START_TEMP_DROP_C` | `5.0f` | 217 | [v16.3ad] temp ลดจากตอนหยุด >= 5°C = bearing เย็นลง = cold start (เทียบ trend ไม่ได้) | 2896 | Yes | No | calibration | used |
| `CT_RATIO_PRIMARY_A` | `1.0f` | 1610 | external CT ratio primary (A) -- 1:1 if no external CT | 2728 | Yes | **Yes** | calibration | used |
| `CT_RATIO_SECONDARY_A` | `1.0f` | 1611 | external CT ratio secondary (A) | 2728 | Yes | **Yes** | calibration | used |
| `CT_TURNS` | `1` | 1612 | times the conductor loops through the CT clamp | 2727 | Yes | **Yes** | calibration | used |
| `CURRENT_EMA_ALPHA` | `0.25f` | 1623 | EMA smoothing coefficient for the CURRENT evidence path (see [Commit 4A] comment at def site) -- 0.25 matches RPM_SMOOTH_ALPHA by design | 2731,2735,2735 | Yes | **Yes** | calibration | used |
| `FAULT_WINDOW_MS` | `3000` | 203 | RUNNING but no pulse > 3 s -> prox=0 (Fault) | 2930 | Yes | No | timing | used |
| `FORCE_STOP_TIMEOUT_MS` | `2000` | 202 | No pulse > 2 s   -> STOPPED | 206,2798,7141,7141 | Yes | **Yes** | timing | used |
| `MOTOR_NAMEPLATE_CURRENT_A` | `1.0f` | 1609 | motor Full-Load Amps (nameplate) -- placeholder | 2727 | Yes | **Yes** | calibration | used |
| `MOTOR_RUNNING_PERCENT` | `20.0f` | 1613 | % of expected full-load reading -- single stateless threshold | 2729,7139,7139 | Yes | **Yes** | calibration | used |
| `NAMEPLATE_RPM` | `1800` | 190 | Motor nameplate RPM (used as RATED_RPM reference) | 197 | Yes | **Yes** | calibration | used |
| `NO_PULSE_STOPPING_MS` | `400` | 201 | No pulse > 400 ms -> STOPPING | 206,2744,2801,7140,7140 | Yes | **Yes** | timing | used |
| `RATED_RPM` | `NAMEPLATE_RPM` | 197 | Rated speed (centre of RUNNING band) | 190,198,1136,2713,2753,2754,2759,2760,7172,7288 | Yes | **Yes** | calibration | used |
| `RATED_RPM_TOL` | `75` | 198 | +/-75 RPM around RATED_RPM -> RUNNING band (5% of 1500) | 1136,2753,2754,2759,2760,7288 | Yes | **Yes** | calibration | used |
| `RUNNING_WARMUP_MS` | `2500` | 204 | [v16.3z] ต้อง in-band ต่อเนื่อง 2.5s ก่อนเป็น RUNNING (กัน bounce/spurious) | 210,2806,2809,2812 | Yes | **Yes** | timing | used |
| `STOPPED_CLEAR_MS` | `(30UL*60UL*1000UL)` | 216 | [v16.3aa] หยุด > 30 นาที = clear trend (bearing state เทียบไม่ได้แล้ว) | (none) | No | No | timing | **UNUSED** |

## Current Measurement (7 items)

| Name | Default | Def Line | Purpose | Referenced at (lines) | Runtime? | Motor State? | Type | Unused? |
|---|---|---|---|---|---|---|---|---|
| `CT_REG_AC_CURRENT` | `0x0000` | 138 | function 04 (input register), unit mA (0-5000 = 0-5A) | 3678 | Yes | No | hardware | used |
| `CURRENT_BUF_SIZE` | `120` | 1597 | samples (60s @ 2Hz) | 1643,4438,4439,5920,6384 | Yes | No | telemetry | used |
| `CURRENT_MIN_SAMPLES` | `10` | 1599 | minimum samples before slope reported (5s) | 5919 | Yes | No | telemetry | used |
| `CURRENT_SAMPLE_INTERVAL_MS` | `500` | 1600 | acquisition cadence (matches CTR4A01_SENSOR.ino SAMPLE_RATE_HZ=2) | 3842,7138,7138 | Yes | **Yes** | timing | used |
| `CURRENT_SAMPLE_INTERVAL_S` | `0.5f` | 1601 | same, in seconds (for linRegSlope()) | 5746,5921 | Yes | No | timing | used |
| `CURRENT_SENSOR_ID` | `0x01` | 137 | CTR4A01 slave address | 3672,3701 | Yes | No | hardware | used |
| `CURRENT_WINDOW_SAMPLES` | `60` | 1598 | samples used for regression (30s window) | 5921 | Yes | No | telemetry | used |

## RPM (12 items)

| Name | Default | Def Line | Purpose | Referenced at (lines) | Runtime? | Motor State? | Type | Unused? |
|---|---|---|---|---|---|---|---|---|
| `MAX_RPM` | `3000` | 195 | Spike reject ceiling | 200,2046,2846 | Yes | **Yes** | calibration | used |
| `MIN_RPM_VALID` | `300` | 196 | Below this -> treat as zero | 2871,6462,6465 | Yes | No | calibration | used |
| `PIN_RPM` | `17` | 193 | Proximity sensor pulse input (PC817 or NPN) | 7284,7285,7288 | Yes | No | hardware | used |
| `PULSE_PER_REV` | `1` | 194 | Pulses per revolution | 2046,2845,7288 | Yes | **Yes** | calibration | used |
| `RPM_DEBOUNCE_US` | `RPM_MIN_INTERVAL_US/2` | 2047 | Derived: RPM_MIN_INTERVAL_US / 2 -- minimum pulse-to-pulse debounce window | 2132 | Yes | No | calibration | used |
| `RPM_FREQ_GATE` | `400` | 218 | v16.0: RPM floor สำหรับ freq_ratio / freq_alert | 6121,6128,6388,6952 | Yes | No | calibration | used |
| `RPM_MIN_INTERVAL_US` | `60000000UL/(MAX_RPM*PULSE_PER_REV)` | 2045 | Derived: 60000000 / (MAX_RPM * PULSE_PER_REV) -- shortest valid pulse interval at MAX_RPM | 2048,2844 | Yes | **Yes** | calibration | used |
| `RPM_SMOOTH_ALPHA` | `0.25f` | 199 | EMA filter coefficient (0=heavy,1=none) | 1616,2847,2848 | Yes | No | calibration | used |
| `SLOT_DUR_MAX_MS` | `5000UL` | 2002 | never go above this slot width (ms) | 1998,6461,6470,6470 | Yes | No | timing | used |
| `SLOT_DUR_MIN_MS` | `200UL` | 2001 | never go below this slot width (ms) | 1998,6461,6469,6469 | Yes | No | timing | used |
| `SLOT_REVS_TARGET` | `20` | 2000 | target shaft revolutions per buf1s slot | 1997,6466,6467,6630,6725,6960 | Yes | No | calibration | used |
| `SPIKE_REJECT_FACTOR` | `1.1f` | 200 | Reject pulses > MAX_RPM x factor | 2846 | Yes | No | calibration | used |

## Modbus (27 items)

| Name | Default | Def Line | Purpose | Referenced at (lines) | Runtime? | Motor State? | Type | Unused? |
|---|---|---|---|---|---|---|---|---|
| `MODBUS_BAUDRATE` | `9600` | 132 | RS485/Modbus RTU bus baud rate (WTVB02 + CTR4A01 both @ 9600) | 7245,7249 | Yes | No | hardware | used |
| `MODBUS_OFFLINE_THRESHOLD` | `3` | 177 | Consecutive Modbus read failures before sensor is declared OFFLINE | 4154,4177 | Yes | No | timing | used |
| `MODBUS_SLAVE_ID` | `0x50` | 133 | WTVB02 vibration sensor's Modbus slave address | 3698,3703,7246,7249 | Yes | No | hardware | used |
| `REG_CFX` | `0x47` | 147 | CFX=Accel Crest Factor X, KX=Kurtosis X (0x47~0x48) §6.4.14 | 3808 | Yes | No | hardware | used |
| `REG_CFY` | `0x53` | 149 | CFY=Accel Crest Factor Y, KY=Kurtosis Y (0x53~0x54) §6.4.15 | 3815 | Yes | No | hardware | used |
| `REG_CFZ` | `0x5F` | 150 | CFZ=Accel Crest Factor Z, KZ=Kurtosis Z (0x5F~0x60) §6.4.16 | 3822 | Yes | No | hardware | used |
| `REG_CMD` | `0x0000` | 157 | Command register (Restart / Save) | 3140,3141,3197,3203 | Yes | No | hardware | used |
| `REG_DRM` | `0x002B` | 168 | Displacement range mode register §6.4.11 | 3004 | Yes | No | hardware | used |
| `REG_FREQ_X` | `0x44` | 146 | Frequency X,Y,Z (0x44~0x46) per WTVB02 manual | 3795 | Yes | No | hardware | used |
| `REG_MODE` | `0x0007` | 158 | Algorithm mode register | 3063,3064 | Yes | No | hardware | used |
| `REG_PEAK_X` | `0x3A` | 151 | [DESIGN-0004] VX~VZ (vibration speed), 3 consecutive registers | 3830 | Yes | No | hardware | used |
| `REG_SAMPLE_RATE` | `0x0029` | 159 | Sample rate register | 164,3002,3089,3090,3099 | Yes | No | hardware | used |
| `REG_TEMPERATURE` | `0x40` | 145 | WTVB02 Modbus register address for temperature reading | 3787 | Yes | No | hardware | used |
| `REG_UNLOCK` | `0x0069` | 156 | Password/Unlock register | 3054,3081,3132,3185,3190 | Yes | No | hardware | used |
| `REG_VRMS_X` | `0x50` | 142 | VRMSX: X-axis velocity RMS (mm/s) §6.4.14 | 3765 | Yes | No | hardware | used |
| `REG_VRMS_Y` | `0x5C` | 143 | VRMSY: Y-axis velocity RMS (mm/s) §6.4.15 | 3772 | Yes | No | hardware | used |
| `REG_VRMS_Z` | `0x68` | 144 | VRMSZ: Z-axis velocity RMS (mm/s) §6.4.16 | 3779 | Yes | No | hardware | used |
| `SENSOR_CMD_SAVE` | `0x0000` | 175 | Save config to NVM | 3129,3140,3141 | Yes | No | hardware | used |
| `SENSOR_DRM_FREQ` | `0x0002` | 169 | 0x02 = Frequency domain algorithm | (none) | No | No | hardware | **UNUSED** |
| `SENSOR_MODE_FREQ` | `0x0002` | 171 | Frequency domain algorithm (MODE=0x02) | 174,3016,3063,3064 | Yes | No | hardware | used |
| `SENSOR_RESTART_COOLDOWN` | `15000;` | 2994 | shared cooldown ระหว่าง restart แต่ละครั้ง (15s) | 3173,3176 | Yes | No | timing | used |
| `SENSOR_SR_16K` | `0x0001` | 161 | Sample Rate = 16 kHz | 3041 | Yes | No | hardware | used |
| `SENSOR_SR_1K` | `0x0005` | 167 | Sample Rate = 1 kHz (SR5) | 3001,3026,3041,3074,3089,3090 | Yes | No | hardware | used |
| `SENSOR_SR_512` | `0x0006` | 166 | Sample Rate = 512 Hz (SR6) | 3041 | Yes | No | hardware | used |
| `SENSOR_UNLOCK_KEY` | `0xB588` | 160 | Unlock password | 3054,3081,3132,3185,3190 | Yes | No | hardware | used |
| `STUCK_MIN_RAW` | `5;` | 2990 | |raw| > 5 (0.05 mm/s) ?????? "?????" | 3890,3903,3904,3905,3928,3928,3951,3951,3974,3974 | Yes | No | calibration | used |
| `STUCK_THRESHOLD` | `5;` | 2989 | 5 ????? x 250ms = 1.25 ?????? (????? 10) | 3909,3932,3955,3978,7578,7578,7580,7582,7584 | Yes | No | timing | used |

## Timing (11 items)

| Name | Default | Def Line | Purpose | Referenced at (lines) | Runtime? | Motor State? | Type | Unused? |
|---|---|---|---|---|---|---|---|---|
| `GPRS_CONNECT_TIMEOUT` | `60000` | 419 | 60 seconds for GPRS connect | (none) | No | No | timing | **UNUSED** |
| `MAX_VALID_YEAR` | `2060` | 434 | Upper bound for RTC/NTP year sanity check (rejects garbage clock values) | 428,3490,3494,3496 | Yes | No | timing | used |
| `MODEM_INIT_TIMEOUT` | `30000` | 418 | 30 seconds for modem init | (none) | No | No | timing | **UNUSED** |
| `MODEM_RETRY_DELAY` | `10000` | 420 | 10 seconds between retries | (none) | No | No | timing | **UNUSED** |
| `NTP_DRIFT_MAX_SEC` | `30` | 426 | Force-correct if drift > 30 seconds | 3602,7473 | Yes | No | timing | used |
| `NTP_DRIFT_WARN_SEC` | `5` | 425 | Warn if drift exceeds 5 seconds | 3598,7472 | Yes | No | timing | used |
| `NTP_SYNC_INTERVAL` | `86400000UL` | 423 | 24 hours between NTP syncs (ms) | 3639,7470 | Yes | No | timing | used |
| `NTP_SYNC_RETRY_INTERVAL` | `1800000UL` | 424 | 30 minutes retry on failure (ms) | 3639,7471 | Yes | No | timing | used |
| `NVS_SAVE_INTERVAL_MS` | `30000UL` | 235 | Save runtime_hour to NVS every 30 s | 1473,2693 | Yes | No | timing | used |
| `RTC_NOW_SAFE` | `(dt) do { \` | 1432 | Macro: mutex-guarded wrapper around rtc.now() for safe cross-task RTC reads | 1426,2324,2326,2409,2431,2600,3557,4959,5114,6143,6854,7602 | Yes | No | timing | used |
| `TIMEZONE_OFFSET_SEC` | `(7 * 3600)` | 435 | UTC+7 (Bangkok/Thailand) -- adjust per site | (none) | No | No | timing | **UNUSED** |

## MQTT (12 items)

| Name | Default | Def Line | Purpose | Referenced at (lines) | Runtime? | Motor State? | Type | Unused? |
|---|---|---|---|---|---|---|---|---|
| `APN` | `"internet";` | 180 | Cellular APN string passed to modem GPRS connect (gprsConnect call) | 3369,3369,7444 | Yes | No | networking | used |
| `GPRS_PASS` | `"";` | 183 | GPRS auth password (empty = not required by this APN) | 3372,4705 | Yes | No | networking | used |
| `GPRS_USER` | `"";` | 182 | GPRS auth username (empty = not required by this APN) | 3372,4705 | Yes | No | networking | used |
| `MQTT_CLIENT_ID` | `"pump01"` | 281 | -> "PLANT01-ESP01" | 328,539,691,4642,4742,4747,7446 | Yes | No | networking | used |
| `MQTT_OUTBOUND_PAYLOAD_MAX` | `1024` | 415 | [v16.5] matches existing /trend serialization buffer size | 1393,2532 | Yes | No | networking | used |
| `MQTT_PORT` | `8883` | 280 | TLS port | 4655,7445 | Yes | No | networking | used |
| `MQTT_QOS` | `1` | 282 | QoS 1 -- at-least-once delivery | 2500,4972,5022,5152,6252,6316,6420,6973,7447 | Yes | No | networking | used |
| `MQTT_SERVER` | `"iot.promlogix.com"` | 279 | Broker Public IP | 4655,7445 | Yes | No | networking | used |
| `QUEUE_SIZE_MQTT_OUTBOUND` | `6` | 410 | [v16.5] Section 7 Item 3: dormant outbound MQTT queue (Analytics -> Network4G, not wired yet) | 7326 | Yes | No | networking | used |
| `client_crt` | `(PEM blob, see source)` | 329 | Client certificate (PEM) for mTLS identity to MQTT broker | 3233 | Yes | No | networking | used |
| `client_key` | `(PEM blob, see source)` | 351 | Client private key (PEM) for mTLS identity to MQTT broker | 3234 | Yes | No | networking | used |
| `root_ca` | `(PEM blob, see source)` | 304 | CA certificate (PEM) used to validate the MQTT broker's TLS cert | 3232 | Yes | No | networking | used |

## Telemetry (50 items)

| Name | Default | Def Line | Purpose | Referenced at (lines) | Runtime? | Motor State? | Type | Unused? |
|---|---|---|---|---|---|---|---|---|
| `AGG_BUF_10S_SIZE` | `60` | 1688 | 60 slots x 10 s = 600 s  history (10 min) | 1696,5955,5974,5974,5976,5976,6759,6776,6776,6812,6900,6906,6906,6909,6909 | Yes | No | telemetry | used |
| `AGG_BUF_1S_SIZE` | `60` | 1687 | 60 slots x 1 s  =  60 s  history | 1692,5950,5966,5966,5969,5969,6719,6736,6736,6810,6897,6920,6920,6922,6922 | Yes | No | telemetry | used |
| `AGG_BUF_60S_SIZE` | `60` | 1689 | 60 slots x 60 s = 3600 s history (60 min) | 1700,5960,6799,6814,6903,6913,6913,6916,6916 | Yes | No | telemetry | used |
| `BASELINE_RMS` | `2.8f` | 384 | Vibration RMS threshold (mm/s) marking baseline/healthy operation | 4499,4500,4501,5106,5107,6026,6027,7451 | Yes | No | calibration | used |
| `BEARING_STABLE_CYCLES` | `2` | 231 | [v16.3k] ลดจาก 8 → 2 cycles (~1 min @ 30s) หลัง RUNNING | 6077,6078,6081 | Yes | No | calibration | used |
| `CRITICAL_RMS` | `11.2f` | 386 | Vibration RMS threshold (mm/s) marking CRITICAL alarm level | 37,229,4246,4460,4501,5107,5515,5551,5573,6027,7453 | Yes | No | calibration | used |
| `EMA_ALPHA` | `0.20f` | 1928 | ? = 0.20 -> ? ? 4 samples (4s @ 1Hz) | 6833,6833 | Yes | No | calibration | used |
| `EMA_DIR_THRESHOLD` | `0.003f` | 1929 | |delta| > 3 ?m/s per 1s update -> direction | 6837,6838 | Yes | No | calibration | used |
| `FL_EVT_BEARING` | `3u` | 251 | Fault-latch event code: bearing-fault alarm | 2179,2189,2579 | Yes | No | telemetry | used |
| `FL_EVT_CRITICAL` | `2u` | 250 | Fault-latch event code: critical vibration alarm | 2180,2190,2582 | Yes | No | telemetry | used |
| `FL_EVT_HEALTH` | `4u` | 252 | Fault-latch event code: low health-score alarm | 2181,2191,2584 | Yes | No | telemetry | used |
| `FL_EVT_MAX_VALID` | `4u` | 253 | Highest valid fault-latch event code (range-check bound) | 2237,2239 | Yes | No | telemetry | used |
| `FL_EVT_NONE` | `0u` | 248 | Fault-latch event code: no fault / cleared | 1480,2222,2237,2576,2596,5063 | Yes | No | telemetry | used |
| `FL_EVT_WARNING` | `1u` | 249 | Fault-latch event code: warning-level vibration alarm | 2182,2192,2587 | Yes | No | telemetry | used |
| `FL_HEALTH_LOW_THOLD` | `30` | 259 | Health-score threshold below which a HEALTH fault event is latched | 2574 | Yes | No | calibration | used |
| `FL_KEY_CODE` | `"ev_code"` | 241 | NVS key name for the latched fault's event code | 2202,2222,2666 | Yes | No | telemetry | used |
| `FL_KEY_KURT` | `"ev_kurt"` | 244 | NVS key name for the latched fault's kurtosis value | 2205,2225,2669 | Yes | No | telemetry | used |
| `FL_KEY_MAGIC` | `"ev_magic"` | 246 | NVS key name for the fault-latch magic sentinel | 2206,2221,2289,2670 | Yes | No | telemetry | used |
| `FL_KEY_PENDING` | `"ev_pending"` | 245 | NVS key name for the fault-latch pending-delivery flag | 2201,2207,2220,2288,2665,2671 | Yes | No | telemetry | used |
| `FL_KEY_RMS` | `"ev_rms"` | 243 | NVS key name for the latched fault's RMS value | 2204,2224,2668 | Yes | No | telemetry | used |
| `FL_KEY_TS` | `"ev_ts"` | 242 | NVS key name for the latched fault's timestamp | 2203,2223,2667 | Yes | No | telemetry | used |
| `FL_MAGIC_VALUE` | `0xF401A7CDUL` | 247 | Magic sentinel value confirming NVS fault-latch record validity | 2206,2232,2234,2670 | Yes | No | telemetry | used |
| `FL_NS` | `"fault_latch"` | 240 | NVS namespace used for fault-latch persistence | 2200,2219,2272,2287,2664 | Yes | No | telemetry | used |
| `FL_SEV_BEARING` | `4u` | 258 | Fault-latch severity ranking for a bearing event | 2179 | Yes | No | telemetry | used |
| `FL_SEV_CRITICAL` | `3u` | 257 | Fault-latch severity ranking for a critical event | 2180 | Yes | No | telemetry | used |
| `FL_SEV_HEALTH` | `2u` | 256 | Fault-latch severity ranking for a health event | 2181 | Yes | No | telemetry | used |
| `FL_SEV_NONE` | `0u` | 254 | Fault-latch severity ranking for no event | 2183 | Yes | No | telemetry | used |
| `FL_SEV_WARNING` | `1u` | 255 | Fault-latch severity ranking for a warning event | 2182 | Yes | No | telemetry | used |
| `FL_TS_MIN_VALID` | `1577836800UL` | 260 | Minimum plausible Unix timestamp (2020-01-01) for latch validity check | 2246,2602,5079 | Yes | No | telemetry | used |
| `FL_TS_UNKNOWN` | `0UL` | 261 | Sentinel value meaning "no valid timestamp available yet" | 1480,2246,2598 | Yes | No | telemetry | used |
| `FREQ_DRIFT_THRESH` | `0.15f` | 1594 | freq_ratio drift > 0.15x -> drift detected | 1960,5865,5866,5867 | Yes | No | calibration | used |
| `KURTOSIS_CONFIRMED` | `6.0f` | 230 | Kurtosis > 6.0 → CONFIRMED bearing fault | 6084 | Yes | No | calibration | used |
| `KURTOSIS_EARLY_WARNING` | `4.0f` | 224 | Kurtosis > 4.0 → EARLY_WARNING (ISO 13373-2) | 6086 | Yes | No | calibration | used |
| `MACHINE_ID` | `"pump01"` | 188 | Machine identity (tag-level) | 288,383,383,1763,2391,4949,5015,5122,6162,6279,6330,6882,7148,7153,7157,7159,7161,7163,7172 | Yes | No | telemetry | used |
| `MACHINE_NAME` | `MACHINE_ID` | 383 | Display uses MACHINE_ID for consistency | 5491,5536,7203,7542 | Yes | No | telemetry | used |
| `PLANT_ID` | `"plant01"` | 187 | Plant / Site identity | 288,1763,2390,4948,5014,5121,6161,6278,6329,6881,7148,7153,7157,7159,7161,7163,7172 | Yes | No | telemetry | used |
| `SANITY_RMS_MAX` | `(CRITICAL_RMS * 3.0f)` | 229 | Upper sanity bound on RMS/peak values (3x CRITICAL_RMS) -- rejects garbage sensor reads | 2087,4117,4120,4125,4261,4495,4497 | Yes | No | calibration | used |
| `SENSOR_ID` | `"vb01"` | 189 | Sensor identity | 2392,4950,5123,6163,6280,6331,6883,7172 | Yes | No | telemetry | used |
| `SLOPE_10S_MIN_SLOTS` | `10` | 1984 | buf10s >= 10 slots (~100 s minimum history) | 1969,5933 | Yes | No | calibration | used |
| `SLOPE_1S_MIN_SLOTS` | `10` | 1983 | buf1s  >= 10 slots (~10 s minimum history) | 1968,5932 | Yes | No | calibration | used |
| `SLOPE_60S_MIN_SLOTS` | `20` | 1985 | buf60s >= 20 slots (~20 min minimum history) | 1970,5934 | Yes | No | calibration | used |
| `SPIKE_RMS_FACTOR` | `1.5f` | 1593 | peak > WARNING_RMS x 1.5 -> ??? spike | 1681,5789,5823,6676 | Yes | No | calibration | used |
| `TELEM_BUF_SIZE` | `120` | 1504 | 60 min × 1 slot/30s = 120 slots max | 1490,1531,1533,2309,2311,2313,2320,2321,2359,2417,2509,7655 | Yes | No | telemetry | used |
| `TEMP_SLOPE_WARN` | `0.001f` | 1602 | degC per sample -> temp rising (0.004 degC/s) | (none) | No | No | calibration | **UNUSED** |
| `TREND_BUF_SIZE` | `240` | 1586 | samples (60s @ 4Hz) -- Raw circular buffer | 1571,1636,2904,4428,4429,5806,5806,5814,5830,5832,5834,5842,5848,5873,5873,6611,6611,6615,6615,6656,6829,6829 | Yes | No | telemetry | used |
| `TREND_MIN_SAMPLES` | `20` | 1588 | ??????????????? 20 samples (5s) ????????? | 5804 | Yes | No | telemetry | used |
| `TREND_SLOPE_DOWN` | `-0.002f` | 1592 | mm/s per sample -> "DOWN" | 5870 | Yes | No | calibration | used |
| `TREND_SLOPE_UP` | `0.002f` | 1591 | mm/s per sample -> "UP"   (0.008 mm/s/s) | 5869 | Yes | No | calibration | used |
| `TREND_WINDOW_SAMPLES` | `120` | 1587 | samples ??????????? trend (30s window) | 1575,5805,5805,5830,5835 | Yes | No | telemetry | used |
| `WARNING_RMS` | `4.5f` | 385 | Vibration RMS threshold (mm/s) marking WARNING alarm level | 37,1593,1681,4246,4458,5512,5789,5823,5874,5876,6676,7452 | Yes | No | calibration | used |

## OLED (7 items)

| Name | Default | Def Line | Purpose | Referenced at (lines) | Runtime? | Motor State? | Type | Unused? |
|---|---|---|---|---|---|---|---|---|
| `BUILTIN_LED` | `10` | 123 | Onboard status LED GPIO pin | 7180,7183,7490,7490 | Yes | No | hardware | used |
| `I2C_SCL_PIN` | `43` | 115 | I2C clock pin, shared by OLED (U8G2 HW_I2C) | 1087,7190 | Yes | No | hardware | used |
| `I2C_SDA_PIN` | `44` | 114 | I2C data pin, shared by OLED (U8G2 HW_I2C) | 1087,7190 | Yes | No | hardware | used |
| `OLED_ADDRESS` | `0x3C` | 129 | Intended I2C address for the SSD1306 OLED -- see Unused Constants note | (none) | No | No | hardware | **UNUSED** |
| `OLED_HEIGHT` | `64` | 128 | Intended OLED panel height in px -- see Unused Constants note | (none) | No | No | hardware | **UNUSED** |
| `OLED_WIDTH` | `128` | 127 | Intended OLED panel width in px -- see Unused Constants note | (none) | No | No | hardware | **UNUSED** |
| `PIN_LED_CLOUD` | `15` | 124 | [Enclosure v1] Cloud LED - reflects modemState + MQTT connection | 7181,7184,7505,7510,7516,7516 | Yes | No | hardware | used |

## Debug (2 items)

| Name | Default | Def Line | Purpose | Referenced at (lines) | Runtime? | Motor State? | Type | Unused? |
|---|---|---|---|---|---|---|---|---|
| `VERIFY_TEST` | *(commented out: `// #define VERIFY_TEST`)* | 96 | Master switch for the "Checkpoint 1-5E" causal-proof diagnostic instrumentation (RAM-only poll_seq / deglitch forensics). Disabled by default — the enabling `#define` is itself commented out. | Gates 13 `#ifdef VERIFY_TEST` blocks: 411,1244,1250,1417,3742,4141,4168,4263,4295,4305,4327,6498,7333 | No (disabled by default; would be Yes if uncommented) | No | debug | used (but disabled) |
| `QUEUE_SIZE_DIAG_SNAPSHOT` | `1` | 412 | [VERIFY_TEST] Checkpoint 5D: one-shot frozen diagnostic snapshot (State -> Analytics); only compiled in when `VERIFY_TEST` is defined | 7338 | No (nested inside `#ifdef VERIFY_TEST`, disabled by default) | No | debug | used (but disabled) |

## Miscellaneous (39 items)

| Name | Default | Def Line | Purpose | Referenced at (lines) | Runtime? | Motor State? | Type | Unused? |
|---|---|---|---|---|---|---|---|---|
| `CFG_KEY_APN` | `"cfg_apn"` | 1715 | NVS key name for Config-Mode-overridden APN string | 1784,1801 | Yes | No | legacy | used |
| `CFG_KEY_MACHINE` | `"cfg_machine"` | 1712 | NVS key name for Config-Mode-overridden MACHINE_ID | 1781,1798 | Yes | No | legacy | used |
| `CFG_KEY_PLANT` | `"cfg_plant"` | 1711 | NVS key name for Config-Mode-overridden PLANT_ID | 1780,1797 | Yes | No | legacy | used |
| `CFG_KEY_RPM` | `"cfg_rpm"` | 1714 | NVS key name for Config-Mode-overridden rated RPM | 1783,1800 | Yes | No | legacy | used |
| `CFG_KEY_SENSOR` | `"cfg_sensor"` | 1713 | NVS key name for Config-Mode-overridden SENSOR_ID | 1782,1799 | Yes | No | legacy | used |
| `CFG_KEY_TREND` | `"cfg_trend"` | 1718 | [v16.3ab] trend persistence policy (0-4) | 1785,1802 | Yes | No | legacy | used |
| `CFG_MAGIC_KEY` | `"cfg_magic"` | 1716 | NVS key name for the magic-value sentinel marking valid saved config | 1774,1796,1803 | Yes | No | legacy | used |
| `CFG_MAGIC_VAL` | `0xCF9A01UL` | 1717 | ถ้า magic ตรง = config ถูก set แล้ว | 1775,1803 | Yes | No | legacy | used |
| `CFG_NS` | `"prom_cfg"` | 1710 | NVS namespace used for Config-Mode-saved identity overrides | 1773,1795 | Yes | No | legacy | used |
| `MODEM_POWER_ON` | `4` | 120 | GPIO pin driving the SIMCom A7670 PWRKEY line | 3247,3251,3288 | Yes | No | hardware | used |
| `MODEM_RESET_LEVEL` | `HIGH` | 122 | Logic level (HIGH) written to MODEM_RESET_PIN during modem hardware reset | (none) | No | No | hardware | **UNUSED** |
| `MODEM_RESET_PIN` | `9` | 121 | GPIO pin driving the SIMCom A7670 hardware reset line | 3248,3253,3255,3287 | Yes | No | hardware | used |
| `MODEM_RX` | `46` | 119 | ESP32 UART RX pin wired to modem TX | 3282 | Yes | No | hardware | used |
| `MODEM_TX` | `3` | 118 | ESP32 UART TX pin wired to modem RX | 3282 | Yes | No | hardware | used |
| `PIN_BUTTON` | `5` | 104 | SELECT button (page navigation) | 5176,5192,7176 | Yes | No | hardware | used |
| `PIN_BUTTON_ENTER` | `6` | 105 | ENTER button (alarm acknowledge) | 5176,5193,7177 | Yes | No | hardware | used |
| `PIN_BUZZER` | `7` | 106 | Buzzer output | 5437,5441,7178,7182 | Yes | No | hardware | used |
| `PRIORITY_ANALYTICS` | `3` | 400 | Analytics (same as display -- non-critical, 1s cadence) | 7409,7413 | Yes | No | hardware | used |
| `PRIORITY_BUTTON` | `2` | 402 | Button handling | 7420,7424 | Yes | No | hardware | used |
| `PRIORITY_BUZZER` | `1` | 403 | Lowest priority | 7431,7435 | Yes | No | hardware | used |
| `PRIORITY_DISPLAY` | `3` | 399 | Display updates | 7387,7391 | Yes | No | hardware | used |
| `PRIORITY_MODBUS` | `5` | 397 | Highest priority (time-critical) | 7360,7364 | Yes | No | hardware | used |
| `PRIORITY_NETWORK` | `2` | 401 | Network (can tolerate delays) | 7398,7402 | Yes | No | hardware | used |
| `PRIORITY_STATE` | `4` | 398 | State machine | 7371,7375 | Yes | No | hardware | used |
| `QUEUE_SIZE_BUTTON` | `3` | 407 | FreeRTOS queue depth for taskButtonHandler's button-event queue | 7310 | Yes | No | hardware | used |
| `QUEUE_SIZE_DISPLAY` | `3` | 408 | FreeRTOS queue depth for the OLED display-update queue | 7311 | Yes | No | hardware | used |
| `QUEUE_SIZE_MAINT` | `2` | 409 | V14.4: maintenance reset events (Button -> Network) | 4990,7312 | Yes | No | hardware | used |
| `QUEUE_SIZE_SENSOR` | `5` | 406 | FreeRTOS queue depth for queueSensorData (Modbus task -> State machine task) | 7309 | Yes | No | hardware | used |
| `RS485_EN_PIN` | `42` | 111 | GPIO pin controlling RS485 transceiver driver-enable (rs485Enable/Disable) | 2952,2956,7179 | Yes | No | hardware | used |
| `RS485_RX_PIN` | `38` | 109 | ESP32 UART RX pin for the RS485 bus (WTVB02 + CTR4A01) | 7245 | Yes | No | hardware | used |
| `RS485_TX_PIN` | `39` | 110 | ESP32 UART TX pin for the RS485 bus (WTVB02 + CTR4A01) | 7245 | Yes | No | hardware | used |
| `STACK_SIZE_ANALYTICS` | `6144` | 395 | Phase 3: +decision engine +classifyFault on stack | 7407,7413 | Yes | No | hardware | used |
| `STACK_SIZE_BUTTON` | `4096` | 392 | V14.7: 2048→4096 (watermark was 172B=92% used; rtc+Wire+Serial.printf depth) | 7418,7424 | Yes | No | hardware | used |
| `STACK_SIZE_DISPLAY` | `6144` | 390 | Display task stack (larger for U8g2) | 7385,7391 | Yes | No | hardware | used |
| `STACK_SIZE_MODBUS` | `4096` | 389 | Modbus task stack | 7358,7364 | Yes | No | hardware | used |
| `STACK_SIZE_NETWORK` | `24576` | 391 | Phase2: 24KB -- RSA-2048 + JSON 2200B + TinyGSM peak | 7396,7402 | Yes | No | hardware | used |
| `STACK_SIZE_STATE` | `8192` | 393 | [v16.3n] 6144→8192: NVS write (Preferences) ใน checkAndLatchFault | 7369,7375 | Yes | No | hardware | used |
| `TINY_GSM_MODEM_SIM7600` | `(flag, no value)` | 78 | Selects SIM7600/A7670 modem profile inside the TinyGSM library (consumed by the included library headers, not referenced elsewhere in this file) | (none) | No | No | networking | **UNUSED** |
| `TINY_GSM_RX_BUFFER` | `4096` | 79 | TinyGSM internal RX ring-buffer size in bytes (consumed by the included library headers, not referenced elsewhere in this file) | (none) | No | No | networking | **UNUSED** |


---

## Appendix: Unused Constants (13 confirmed, zero other references)

Each was independently re-verified by direct grep against the whole file before being labeled unused.

| Constant | Section | Notable finding |
|---|---|---|
| `OLED_WIDTH`, `OLED_HEIGHT`, `OLED_ADDRESS` | OLED | **Genuinely dead by substitution, not oversight.** The display object is constructed at line 1087 as `U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(...)` — the `128X64` dimensions are baked directly into the U8G2 class-name template argument, and the constructor used here doesn't take an address parameter at all. These three `#define`s were never wired to the actual display init. |
| `SENSOR_DRM_FREQ` | Modbus | **Deliberately unused, documented in-source.** A block comment directly above `reconfigSensorAfterRestart()` (~line 2999) states explicitly: *"ไม่มีการเขียน REG_DRM (0x2B) ในฟังก์ชันนี้ -- sensor ใช้ค่าที่ persist อยู่ใน NVM ของตัวมันเอง"* ("REG_DRM is not written in this function — the sensor uses the value already persisted in its own NVM"). Matches the boot log line seen in this project: `MODE=FreqDomain(0x02) saved (SR/DRM not written by this function)`. `REG_DRM` itself (the register address) *is* used (read-only, line 3004); only the write-value constant is dead. |
| `MODEM_RESET_LEVEL` | Miscellaneous | The reset sequence at lines 3248/3253/3255 calls `digitalWrite(MODEM_RESET_PIN, LOW/HIGH/LOW)` with **literal** `LOW`/`HIGH`, not this macro. The macro exists but the polarity is hardcoded inline instead. |
| `TINY_GSM_MODEM_SIM7600` | Miscellaneous | TinyGSM library selector macro — must be defined before `#include <TinyGsmClient.h>` (line ~78). It has zero references *within this file* by design: it's consumed by the library's own preprocessor conditionals, not by this sketch's code. Not dead in the "never mattered" sense — just invisible to a single-file text search. |
| `TINY_GSM_RX_BUFFER` | Miscellaneous | Same category as above — TinyGSM library-internal buffer-size macro, consumed by the included library headers. |
| `STOPPED_CLEAR_MS` | Motor State | Defined (`30 min`) with a detailed Thai comment about clearing trend data after a long stop, but never referenced anywhere else. The actual stop/resume-vs-clear decision logic (`g_motorStoppedSince`, `STOPPED_CLEAR_MS`'s apparent purpose) appears to be implemented via a different, thermal-based mechanism (`COLD_START_TEMP_DROP_C`, which **is** used) per the `[v16.3ad]` comment elsewhere in the file — `STOPPED_CLEAR_MS` looks like a superseded time-based policy left in place after the thermal-based policy replaced it. |
| `TEMP_SLOPE_WARN` | Telemetry | Defined with a specific numeric threshold and comment, never referenced. |
| `GPRS_CONNECT_TIMEOUT`, `MODEM_INIT_TIMEOUT`, `MODEM_RETRY_DELAY` | Timing | All three modem-timeout constants are defined together in a `-- Modem Timeouts --` block (line ~417) but none are referenced anywhere in the modem init/connect code path. The actual modem connect logic appears to use different, inline timeout values or library defaults instead. |
| `TIMEZONE_OFFSET_SEC` | Timing | Defined as UTC+7 with a "adjust per site" comment, but the actual timezone handling in the NTP sync code (`[NTP] Parsed: ... TZ=+28 quarters = +7h` per the live boot logs seen in this session) evidently derives the offset from the modem's own `AT+CCLK?` response quarter-hour field rather than from this constant. |

---

*End of audit. No source code was modified to produce this document.*
