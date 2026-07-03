/*
 * ============================================================================
 * Industrial Condition Monitoring System - Dual Core FreeRTOS
 * ============================================================================
 * Hardware: LilyGO T-Vending S3 (ESP32-S3, 16MB Flash)
 * Modem: SIMCom A7670 (SIM7600 Compatible) - 4G LTE
 * RTC: DS3231
 *
 * Version: 16.0 (condition_monitoring_v1)
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

#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <TinyGsmClient.h>
#include <MQTT.h>  // joel-gaehwiler/MQTT (arduino-mqtt) -- supports QoS 0/1/2
#include <ModbusMaster.h>
#include <ArduinoJson.h>
#include <RTClib.h>
#include <Preferences.h>   // NVS Flash -- runtime_hour persistence


// ============================================================================
// HARDWARE CONFIGURATION
// ============================================================================

// --- Pin Definitions (LilyGO T-Vending S3) ---
#define PIN_BUTTON 5        // SELECT button (page navigation)
#define PIN_BUTTON_ENTER 6  // ENTER button (alarm acknowledge)
#define PIN_BUZZER 7        // Buzzer output

// --- RS485 Pins ---
#define RS485_RX_PIN 38
#define RS485_TX_PIN 39
#define RS485_EN_PIN 42

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

// --- Display Configuration ---
#define OLED_WIDTH 128
#define OLED_HEIGHT 64
#define OLED_ADDRESS 0x3C

// --- Modbus Configuration ---
#define MODBUS_BAUDRATE 9600
#define MODBUS_SLAVE_ID 0x50
// §6.4.14-16: Velocity RMS (True RMS, ÷1000 → mm/s)
// เปลี่ยนจาก VX/VY/VZ (0x3A Peak ÷100) → VRMSX/Y/Z (True RMS ÷1000)
// ต้องตั้ง DRM=0x02 (Frequency domain) เพื่อให้ค่าถูกต้อง
#define REG_VRMS_X 0x50  // VRMSX: X-axis velocity RMS (mm/s) §6.4.14
#define REG_VRMS_Y 0x5C  // VRMSY: Y-axis velocity RMS (mm/s) §6.4.15
#define REG_VRMS_Z 0x68  // VRMSZ: Z-axis velocity RMS (mm/s) §6.4.16
#define REG_TEMPERATURE 0x40
#define REG_FREQ_X 0x44  // Frequency X,Y,Z (0x44~0x46) per WTVB02 manual
#define REG_CFX    0x47  // CFX=Accel Crest Factor X, KX=Kurtosis X (0x47~0x48) §6.4.14
                         // CFY=0x53, CFZ=0x5F (ไม่ต่อเนื่อง -- อ่านแยก transaction ถ้าต้องการ)
#define REG_CFY    0x53  // CFY=Accel Crest Factor Y, KY=Kurtosis Y (0x53~0x54) §6.4.15
#define REG_CFZ    0x5F  // CFZ=Accel Crest Factor Z, KZ=Kurtosis Z (0x5F~0x60) §6.4.16

// --- Sensor Re-config Registers (v15.7) ---
// ใช้หลัง restartSensorViaModbus() เพื่อ restore config ที่อาจกลับเป็น default
#define REG_UNLOCK        0x0069  // Password/Unlock register
#define REG_CMD           0x0000  // Command register (Restart / Save)
#define REG_MODE          0x0007  // Algorithm mode register
#define REG_SAMPLE_RATE   0x0029  // Sample rate register
#define SENSOR_UNLOCK_KEY 0xB588  // Unlock password
#define SENSOR_SR_16K     0x0001  // Sample Rate = 16 kHz
#define REG_DRM           0x002B  // Displacement range mode register §6.4.11
#define SENSOR_DRM_FREQ   0x0002  // 0x02 = Frequency domain algorithm
                                  // จำเป็นสำหรับ VRMS (0x50/0x5C/0x68) ให้คำนวณถูกต้อง
#define SENSOR_MODE_TDLF  0x0000  // Time domain low frequency algorithm
#define SENSOR_CMD_SAVE   0x0000  // Save config to NVM
// ????? error ??????????????????? OFFLINE (3 x 250ms = 750ms)
#define MODBUS_OFFLINE_THRESHOLD 3

// --- 4G Network Configuration ---
static constexpr const char* APN = "internet";
static constexpr const char* GPRS_USER = "";
static constexpr const char* GPRS_PASS = "";

// --- Identity Configuration (Phase 1) ---
// *** ??? 4 ???????????????? -- ?????????????????? ***
#define PLANT_ID "plant01"   // Plant / Site identity
#define MACHINE_ID "pump01"  // Machine identity (tag-level)
#define SENSOR_ID "vb01"     // Sensor identity
#define NAMEPLATE_RPM 1500   // Motor nameplate RPM (used as RATED_RPM reference)

// --- Proximity / RPM Sensor Configuration ---
#define PIN_RPM               17      // Proximity sensor pulse input (PC817 or NPN)
#define PULSE_PER_REV         1       // Pulses per revolution
#define MAX_RPM               3000   // Spike reject ceiling
#define MIN_RPM_VALID         300      // Below this -> treat as zero
#define RATED_RPM             NAMEPLATE_RPM   // Rated speed (centre of RUNNING band)
#define RATED_RPM_TOL         75       // +/-75 RPM around RATED_RPM -> RUNNING band (5% of 1500)
#define RPM_SMOOTH_ALPHA      0.25f   // EMA filter coefficient (0=heavy,1=none)
#define SPIKE_REJECT_FACTOR   1.1f    // Reject pulses > MAX_RPM x factor
#define NO_PULSE_STOPPING_MS  400     // No pulse > 400 ms -> STOPPING
#define FORCE_STOP_TIMEOUT_MS 2000    // No pulse > 2 s   -> STOPPED
#define FAULT_WINDOW_MS       3000    // RUNNING but no pulse > 3 s -> prox=0 (Fault)
#define RPM_FREQ_GATE         400     // v16.0: RPM floor สำหรับ freq_ratio / freq_alert
                                      // gate หลัก: motor_state == MOTOR_RUNNING (3 จุด)
                                      // gate รอง: rpm >= 400 เป็น safety floor เพิ่มเติม
                                      // ป้องกัน false drift alert ระหว่าง STARTING/STOPPING

// --- Bearing Alert (Kurtosis) Thresholds ---
#define KURTOSIS_EARLY_WARNING  4.0f  // Kurtosis > 4.0 → EARLY_WARNING (ISO 13373-2)
#define KURTOSIS_CONFIRMED      6.0f  // Kurtosis > 6.0 → CONFIRMED bearing fault
#define BEARING_STABLE_CYCLES   8     // รอ 8 publish cycles (~4 min @ 30s) หลัง RUNNING
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
#define BASELINE_RMS 2.1f
#define WARNING_RMS 4.5f
#define CRITICAL_RMS 7.1f

// --- FreeRTOS Configuration ---
#define STACK_SIZE_MODBUS    4096   // Modbus task stack
#define STACK_SIZE_DISPLAY   6144   // Display task stack (larger for U8g2)
#define STACK_SIZE_NETWORK  24576   // Phase2: 24KB -- RSA-2048 + JSON 2200B + TinyGSM peak
#define STACK_SIZE_BUTTON    4096   // V14.7: 2048→4096 (watermark was 172B=92% used; rtc+Wire+Serial.printf depth)
#define STACK_SIZE_STATE     6144   // Phase 4: 4096->6144 (watermark was 2236 = 54% used)
#define STACK_SIZE_ANALYTICS 6144   // Phase 3: +decision engine +classifyFault on stack

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

// --- Modem Timeouts ---
#define MODEM_INIT_TIMEOUT 30000    // 30 seconds for modem init
#define GPRS_CONNECT_TIMEOUT 60000  // 60 seconds for GPRS connect
#define MODEM_RETRY_DELAY 10000     // 10 seconds between retries

// --- Time Sync Configuration ---
#define NTP_SYNC_INTERVAL 86400000UL       // 24 hours between NTP syncs (ms)
#define NTP_SYNC_RETRY_INTERVAL 1800000UL  // 30 minutes retry on failure (ms)
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
    if (_tcp.connected()) _tcp.stop();
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
          if (!_tcp.connected()) {
            Serial.println("[TLS] TCP lost during handshake");
            return 0;
          }
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
    int ret = mbedtls_ssl_write(&_ssl, buf, sz);
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
    _tcp.stop();
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
    _tcp.stop();

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
    if (avail <= 0) return MBEDTLS_ERR_SSL_WANT_READ;

    // -- Step 3: Fill local buffer via one CIPRXGET call --
    int fetched = self->_ciprxget(avail);
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

// Motor run-state (derived from proximity sensor pulse timing)
typedef enum {
  MOTOR_STOPPED  = 0,   // ???? / ????? pulse
  MOTOR_STARTING = 1,   // ????? start (rpm ????????? rated band)
  MOTOR_RUNNING  = 2,   // RUNNING -- rpm ?????? RATED_RPM +/- RATED_RPM_TOL
  MOTOR_STOPPING = 3    // ????? stop (pulse ??????????????)
} MotorRunState_t;

// ============================================================================
// DATA STRUCTURES
// ============================================================================

// Sensor data (shared between cores)
typedef struct {
  float rms_x;           // estimated RMS velocity X [mm/s] = vel_peak_x / √2
  float rms_y;           // estimated RMS velocity Y [mm/s] = vel_peak_y / √2
  float rms_z;           // estimated RMS velocity Z [mm/s] = vel_peak_z / √2
  float rms_overall;     // max(rms_x, rms_y, rms_z) -- ใช้สำหรับ threshold / state machine

  // --- True Peak Velocity (raw/100, ก่อน ÷√2) [v15.0] ---
  float vel_peak_x;      // [mm/s] peak velocity X จาก register โดยตรง (0x3A / 100)
  float vel_peak_y;      // [mm/s] peak velocity Y (0x3B / 100)
  float vel_peak_z;      // [mm/s] peak velocity Z (0x3C / 100)
  float vel_peak_overall;// max(vel_peak_x, y, z) -- ใช้ drive g_velPeakHold

  // --- Sensor-computed features [v15.0/15.1] ---
  // คำนวณภายใน chip จาก 16KHz FIFO ถูกต้องกว่าคำนวณบน ESP32
  float cf_x;            // Acceleration Crest Factor X (reg 0x47 / 1000) -- Peak/RMS acc
  float cf_y;            // Acceleration Crest Factor Y (reg 0x53 / 1000) [v15.1]
  float cf_z;            // Acceleration Crest Factor Z (reg 0x5F / 1000) [v15.1]
  float cf_max;          // max(cf_x, cf_y, cf_z)                         [v15.1]

  float kurtosis_x;      // Acceleration Kurtosis X (reg 0x48 / 1000) -- bearing impact
  float kurtosis_y;      // Acceleration Kurtosis Y (reg 0x54 / 1000) [v15.1]
  float kurtosis_z;      // Acceleration Kurtosis Z (reg 0x60 / 1000) [v15.1]
  float kurtosis_max;    // max(kurtosis_x, y, z) -- ใช้ใน bearing alert  [v15.1]
  uint8_t kurtosis_dominant_axis; // 0=X, 1=Y, 2=Z (แกนที่ kurtosis_max มาจาก) [v15.1]

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
} VibrationData_t;

// System state (shared between cores)
typedef struct {
  MachineState_t state;
  DisplayPage_t currentPage;
  bool alarmAcknowledged;
  bool buzzerActive;
  bool blinkState;
  uint32_t stateEntryTime;
} SystemState_t;

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

// Mutex Handles
SemaphoreHandle_t mutexVibData    = NULL;
SemaphoreHandle_t mutexSystemState = NULL;
SemaphoreHandle_t mutexI2C        = NULL;
SemaphoreHandle_t mutexModem      = NULL;  // Mutex for modem access
SemaphoreHandle_t mutexAggBufs    = NULL;  // Phase 2: protects g_buf1s/10s/60s (taskAnalytics ? taskNetwork)
SemaphoreHandle_t mutexFaultLatch = NULL;  // v3 hardened: guards g_fl + g_flCount + the "fault_latch" NVS namespace

// ============================================================================
// SHARED VARIABLES (protected by mutex)
// ============================================================================

static VibrationData_t g_vibData = { 0 };
static SystemState_t g_systemState = {
  .state = STATE_NORMAL,
  .currentPage = PAGE_MACHINE,
  .alarmAcknowledged = false,
  .buzzerActive = false,
  .blinkState = false,
  .stateEntryTime = 0
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

// -- Velocity Peak Holding (Core 0 only -- taskModbusRead writes, taskNetwork reads+resets) --
// v15.0: เก็บ vel_peak_overall (true peak mm/s = raw/100) แทน rms_overall (ที่แปลงแล้ว)
// Access pattern: เขียน Core 0 (250ms) / อ่าน+reset Core 1 (publish interval)
// ใช้ mutex mutexVibData ป้องกัน read+reset จาก Core 1
static volatile float g_velPeakHold = 0.0f;  // [mm/s] true peak velocity hold (reset ทุก publish)

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
// ============================================================================

// --- Trend Buffer Configuration ---
#define TREND_BUF_SIZE        240    // samples (60s @ 4Hz) -- Raw circular buffer
#define TREND_WINDOW_SAMPLES  120    // samples ??????????? trend (30s window)
#define TREND_MIN_SAMPLES      20    // ??????????????? 20 samples (5s) ?????????

// Thresholds ?????? Trend Engine
#define TREND_SLOPE_UP      0.002f   // mm/s per sample -> "UP"   (0.008 mm/s/s)
#define TREND_SLOPE_DOWN   -0.002f   // mm/s per sample -> "DOWN"
#define SPIKE_RMS_FACTOR    1.5f     // peak > WARNING_RMS x 1.5 -> ??? spike
#define FREQ_DRIFT_THRESH   0.15f    // freq_ratio drift > 0.15x -> drift detected
#define TEMP_SLOPE_WARN     0.001f   //  degC per sample -> temp rising (0.004 degC/s)

// --- Trend Sample Struct (Layer 1) ---
typedef struct {
  float rms;          // rms_overall [mm/s]
  float peak;         // velocity peak hold [mm/s]
  float temp;         // temperature [ degC]
  float freq_ratio_x; // freq_x / (rpm/60) -- harmonic order
  float freq_ratio_y;
  float freq_ratio_z;
} TrendSample_t;

// --- Trend Buffer Globals (Core 0 writes / Core 1 reads) ---
static TrendSample_t     g_trendBuf[TREND_BUF_SIZE];
static volatile uint16_t g_trendHead  = 0;
static volatile uint16_t g_trendCount = 0;

// ============================================================================
// MULTI-RESOLUTION AGGREGATION BUFFERS -- Phase 2
// ============================================================================
//
// taskAnalytics (Core 1) reads g_trendBuf every 1 s -> aggregates -> pushes to
// 3 circular buffers:
//
//   g_buf1s [60]  -- 1 slot = 1 s   -> 60 s of 1-second averages
//   g_buf10s[60]  -- 1 slot = 10 s  -> 10 min of 10-second averages
//   g_buf60s[60]  -- 1 slot = 60 s  -> 60 min of 60-second averages
//
// RAM: sizeof(AggSample_t) x 60 x 3 ? 8.6 KB
//
// Thread safety:
//   taskAnalytics WRITES g_buf* (Core 1)
//   taskNetwork (calcTrend) READS g_buf* (Core 1)
//   Both on same core -> FreeRTOS preemption CAN interleave struct writes
//   -> protected by mutexAggBufs (lightweight, held <1 ms each direction)
// ============================================================================

// --- Aggregated Sample Struct ---
typedef struct {
  float   mean_rms;     // ????????? rms_overall ?? slot [mm/s]
  float   max_rms;      // ????????? rms_overall ?? slot
  float   stddev_rms;   // standard deviation ??? rms (?????? volatility)
  float   mean_temp;    // ????????? temperature [ degC]
  float   max_temp;     // ????????? temperature
  float   mean_peak;    // ????????? velocity peak hold
  float   max_peak;     // ????????? peak
  float   mean_frx;     // ????????? freq_ratio_x
  float   mean_fry;
  float   mean_frz;
  uint8_t spike_count;  // peak > WARNING_RMS x SPIKE_RMS_FACTOR ????? slot
  uint8_t n_samples;    // raw samples ?????? aggregate (?????? debug)
  // pad to 4-byte aligned: 10xfloat(40) + 2xuint8(2) + 2 pad = 44 bytes
} AggSample_t;

// --- Buffer sizes ---
#define AGG_BUF_1S_SIZE    60   // 60 slots x 1 s  =  60 s  history
#define AGG_BUF_10S_SIZE   60   // 60 slots x 10 s = 600 s  history (10 min)
#define AGG_BUF_60S_SIZE   60   // 60 slots x 60 s = 3600 s history (60 min)

// --- Aggregated circular buffers (taskAnalytics writes / calcTrend reads, both Core 1) ---
static AggSample_t       g_buf1s [AGG_BUF_1S_SIZE];
static volatile uint16_t g_buf1sHead  = 0;
static volatile uint16_t g_buf1sCount = 0;

static AggSample_t       g_buf10s[AGG_BUF_10S_SIZE];
static volatile uint16_t g_buf10sHead  = 0;
static volatile uint16_t g_buf10sCount = 0;

static AggSample_t       g_buf60s[AGG_BUF_60S_SIZE];
static volatile uint16_t g_buf60sHead  = 0;
static volatile uint16_t g_buf60sCount = 0;

// --- EMA state (taskAnalytics writes, Core 1 only -- no cross-core issue) ---
#define EMA_ALPHA          0.20f   // ? = 0.20 -> ? ? 4 samples (4s @ 1Hz)
#define EMA_DIR_THRESHOLD  0.003f  // |delta| > 3 ?m/s per 1s update -> direction
static float   g_emaRms     = 0.0f;  // EMA ??? rms_overall [mm/s]
static float   g_emaPrevRms = 0.0f;  // EMA ??????? (???????? direction)
static float   g_emaDelta   = 0.0f;  // g_emaRms ? g_emaPrevRms [mm/s per 1s]
static int8_t  g_emaDir     = 0;     // +1=UP  0=STABLE  -1=DOWN

// --- Analytics MQTT topic (built at setup) ---
static char g_mqttAnalyticsTopic[128];

// ── MQTT Pipeline Topics (mqtt_pipeline_patch §A) ────────────────────────────
static char g_mqttTopicSensor    [128];  // factory/.../sensor
static char g_mqttTopicDecision  [128];  // factory/.../decision
static char g_mqttTopicTrend     [128];  // factory/.../trend
static char g_mqttTopicEvent     [128];  // factory/.../vibration/event (V14.4 maintenance audit)
// ─────────────────────────────────────────────────────────────────────────────

// ============================================================================
// TREND ENGINE OUTPUT -- populated by calcTrend(), read by publishTelemetry()
// ============================================================================

// -- TrendResult_t -- complete output struct -------------------------------
typedef struct {
  // -- Phase 1 fields (30s single-resolution) ------------------------------
  float    rms_slope;       // mm/s per sample (+= rising, -= falling)
  float    temp_slope;      //  degC per sample
  int8_t   trend_dir;       // +1=UP  0=STABLE  -1=DOWN  (from linreg slope)
  uint16_t spike_count;     // peak > WARNINGx1.5 ?? 30s window
  float    freq_drift_x;    // harmonic drift X
  float    freq_drift_y;
  float    freq_drift_z;
  bool     freq_alert;      // drift > FREQ_DRIFT_THRESH >=1 axis
  float    ttw_hours;       // Phase1 TTW from raw slope [h]
  uint16_t window_samples;  // ????? samples ?????????? (debug)

  // -- Phase 2 fields (multi-resolution) -----------------------------------
  float  slope_1s;          // linreg ?? g_buf1s 30 slots  [mm/s per 1s slot]
  float  slope_10s;         // linreg ?? g_buf10s 30 slots [mm/s per 10s slot]
  float  slope_60s;         // linreg ?? g_buf60s 30 slots [mm/s per 60s slot]
  bool   slope_ready_1s;    // true = buf1s >= SLOPE_1S_MIN_SLOTS
  bool   slope_ready_10s;   // true = buf10s >= SLOPE_10S_MIN_SLOTS
  bool   slope_ready_60s;   // true = buf60s >= SLOPE_60S_MIN_SLOTS
  int8_t ema_dir;           // snapshot g_emaDir  (+1/0/-1)
  float  ema_rms;           // snapshot g_emaRms  [mm/s]
  float  ema_delta;         // snapshot g_emaDelta [mm/s per 1s]
  float  stddev_1min;       // mean stddev_rms ??? g_buf1s 60 slots
  float  max_rms_10min;     // peak max_rms ??? g_buf10s 60 slots

} TrendResult_t;

static TrendResult_t g_trendResult = { 0 };  // ?? trend ??????

// ── Slope-readiness thresholds (minimum slots required before a tier's ────
//    slope_*s value is considered trustworthy enough to publish) ──────────
#define SLOPE_1S_MIN_SLOTS   10   // buf1s  >= 10 slots (~10 s minimum history)
#define SLOPE_10S_MIN_SLOTS  10   // buf10s >= 10 slots (~100 s minimum history)
#define SLOPE_60S_MIN_SLOTS  20   // buf60s >= 20 slots (~20 min minimum history)

// ── Rolling variance of mean_rms per tier (OSG + FVRI inputs) ─────────────
// Updated by computeRmsVariance() inside taskAnalytics, exposed on /trend.
static float g_slopeVar_1s  = 1e-6f;
static float g_slopeVar_10s = 1e-6f;
static float g_slopeVar_60s = 1e-6f;

// ============================================================================
// PATENT CLAIM 2 -- RPM-Adaptive Slot Duration
// ============================================================================
// Instead of a fixed 1-second slot, buf1s slots are sized so each one covers
// a roughly constant number of shaft revolutions (SLOT_REVS_TARGET),
// regardless of motor speed. Clamped to [SLOT_DUR_MIN_MS, SLOT_DUR_MAX_MS]
// so slot width stays sane at very low/high RPM. See computeSlotDurMs().
#define SLOT_REVS_TARGET   20      // target shaft revolutions per buf1s slot
#define SLOT_DUR_MIN_MS    200UL   // never go below this slot width (ms)
#define SLOT_DUR_MAX_MS    5000UL  // never go above this slot width (ms)

// taskAnalytics millis-based slot accumulators -- promoted to globals so
// taskButtonHandler can atomically reset them during a maintenance event
// while taskAnalytics is suspended (see V14.4 maintenance reset block).
static uint32_t g_accMs_1s  = 0;
static uint32_t g_accMs_10s = 0;
static uint32_t g_accMs_60s = 0;

// Current RPM-adaptive duration (ms) of one buf1s slot; recomputed every
// taskAnalytics tick from live RPM via computeSlotDurMs().
static uint32_t g_slotDur1sMs = 1000UL;

// Raw-trend-buffer bookkeeping (taskAnalytics)
static uint16_t g_anaLastHead   = 0;
static bool     g_anaFirstRun   = true;
static uint8_t  g_anaPublishCnt = 0;

// Per-slot running accumulators (build up the next AggSample_t for buf1s)
static float    g_sl_sumRms   = 0.0f;
static float    g_sl_sumSqRms = 0.0f;
static float    g_sl_maxRms   = 0.0f;
static float    g_sl_sumTemp  = 0.0f;
static float    g_sl_maxTemp  = 0.0f;
static float    g_sl_sumPeak  = 0.0f;
static float    g_sl_maxPeak  = 0.0f;
static float    g_sl_sumFrx   = 0.0f;
static float    g_sl_sumFry   = 0.0f;
static float    g_sl_sumFrz   = 0.0f;
static uint8_t  g_sl_spikes   = 0;
static uint8_t  g_sl_n        = 0;

// millis() timestamp when the system last entered STATE_WARMUP (used for the
// maintenance-reset MQTT audit event timestamp when RTC is not valid).
static uint32_t g_warmupStartTs = 0;

// RTC valid flag
static bool g_rtcValid = false;

// ============================================================================
// PROXIMITY / RPM -- ISR Variables (volatile, written in IRAM ISR)
// ============================================================================

static const uint32_t RPM_MIN_INTERVAL_US =
    60000000UL / (MAX_RPM * PULSE_PER_REV);
static const uint32_t RPM_DEBOUNCE_US =
    RPM_MIN_INTERVAL_US / 2;

volatile uint32_t g_rpmLastPulseTime  = 0;
volatile uint32_t g_rpmPulseInterval  = 0;
volatile uint32_t g_rpmTotalPulses    = 0;

// RPM processing state (Core 0 only -- no mutex needed)
static float           g_rpmFiltered       = 0.0f;
static uint32_t        g_rpmLastPulseCount  = 0;
static uint32_t        g_rpmLastPulseMillis = 0;
static MotorRunState_t g_motorRunState      = MOTOR_STOPPED;
static MotorRunState_t g_prevMotorRunState  = MOTOR_STOPPED;  // v15.2: track transition
static volatile uint8_t g_bearingStableCnt  = 0;              // v16.0: cycles since RUNNING stable; volatile: written by Core1 (publishTelemetry/taskButtonHandler), read by Core0 (processRPM/taskStateMachine) [fault_latch v3 mandatory fix]
static bool            g_trendFreqFlushed   = false;          // v16.0: freq_ratio flushed on RUNNING entry
static uint8_t         g_freqDriftSuppress  = 0;              // v16.0: suppress drift for N calcTrend cycles after flush

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
  g_motorPrefs.putFloat("runtime_h", value);
  g_motorPrefs.end();
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
  p.putUChar(FL_KEY_PENDING, 0u);              // Step 1: disarm
  p.putUChar(FL_KEY_CODE,    g_fl.code);       // Step 2: data
  p.putUInt (FL_KEY_TS,      g_fl.ts);
  p.putFloat(FL_KEY_RMS,     g_fl.rms);
  p.putFloat(FL_KEY_KURT,    g_fl.kurtosis);
  p.putUInt (FL_KEY_MAGIC,   FL_MAGIC_VALUE);  // Step 3: commit marker
  p.putUChar(FL_KEY_PENDING, 1u);              // Step 4: arm
  p.end();
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
  p.putUChar(FL_KEY_PENDING, 0u);
  p.putUInt (FL_KEY_MAGIC,   0u);
  p.end();
  g_fl.pending = false;
  xSemaphoreGive(mutexFaultLatch);
  Serial.println("[LATCH] CLEAR — delivered, NVS latch released");
}

static void checkAndLatchFault(const VibrationData_t* data,
                                MachineState_t         newState,
                                int                    healthScore,
                                bool                   bearingConfirmed)
{
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
    const uint32_t cand = rtc.now().unixtime();
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
  g_fl.rms      = data->rms_overall;
  g_fl.kurtosis = data->kurtosis_max;
  g_flCount++;

  saveFaultLatchNVS();  // caller (here) holds mutexFaultLatch -- protects the NVS transaction too

  g_fl.pending = true;  // set LAST — after NVS write and barrier

  Serial.printf("[LATCH] LATCHED ev=%u(%s) sev=%u ts=%lu rms=%.2f kurt=%.3f n=%lu\n",
                evCode, faultEventStr(evCode), faultSeverity(evCode),
                (unsigned long)epochNow,
                data->rms_overall, data->kurtosis_max,
                (unsigned long)g_flCount);

  xSemaphoreGive(mutexFaultLatch);
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
  if (newPulse && interval >= RPM_MIN_INTERVAL_US) {
    float rpmRaw = (60000000.0f / interval) / PULSE_PER_REV;
    if (rpmRaw <= MAX_RPM * SPIKE_REJECT_FACTOR) {
      g_rpmFiltered = RPM_SMOOTH_ALPHA * rpmRaw
                    + (1.0f - RPM_SMOOTH_ALPHA) * g_rpmFiltered;
    }
  }

  // ---------- Motor State Machine ----------
  if (timeSincePulseMs > FORCE_STOP_TIMEOUT_MS) {
    g_motorRunState = MOTOR_STOPPED;
    g_rpmFiltered   = 0.0f;
  } else if (timeSincePulseMs > NO_PULSE_STOPPING_MS) {
    g_motorRunState  = MOTOR_STOPPING;
    g_rpmFiltered   *= 0.80f;
    if (g_rpmFiltered < MIN_RPM_VALID) g_rpmFiltered = 0.0f;
  } else {
    bool inBand = (g_rpmFiltered >= (RATED_RPM - RATED_RPM_TOL)) &&
                  (g_rpmFiltered <= (RATED_RPM + RATED_RPM_TOL));
    g_motorRunState = inBand ? MOTOR_RUNNING : MOTOR_STARTING;
    if (g_prevMotorRunState == MOTOR_STOPPED) {
      g_bearingStableCnt = 0;
    }
  }

  // v15.2 Fix 18: Reset g_velPeakHold เมื่อ motor transition → STOPPED
  // ป้องกัน peak hold สะสมค่า impulse จาก deceleration ค้างถึง publish ถัดไป
  // Core 0 writes g_velPeakHold / Core 1 reads+resets -- atomic float (4-byte aligned)
  if (g_motorRunState == MOTOR_STOPPED &&
      g_prevMotorRunState != MOTOR_STOPPED) {
    g_bearingStableCnt   = 0;
    g_trendFreqFlushed   = false;  // ต้อง flush freq อีกครั้งเมื่อ start ใหม่
    g_freqDriftSuppress  = 0;
    g_velPeakHold        = 0.0f;
    Serial.println("[MOTOR] STOPPED transition -- peak hold reset");
  }
  g_prevMotorRunState = g_motorRunState;

  // v16.0: Flush freq_ratio ใน trendBuf เมื่อ transition เข้า RUNNING
  // ป้องกัน drift calculation ใช้ข้อมูลจาก session ก่อน (STARTING noise / boot data)
  // zero-out freq_ratio_x/y/z ทุก entry -- ใช้ครั้งเดียวต่อ RUNNING session
  if (g_motorRunState == MOTOR_RUNNING && !g_trendFreqFlushed) {
    // Flush ทั้ง array (ไม่ใช่แค่ n entries) เพราะ g_trendBuf เป็น circular buffer
    // entries จริงอาจอยู่ที่ index ใดก็ได้ใน 0..TREND_BUF_SIZE-1
    for (uint16_t i = 0; i < TREND_BUF_SIZE; i++) {
      g_trendBuf[i].freq_ratio_x = 0.0f;
      g_trendBuf[i].freq_ratio_y = 0.0f;
      g_trendBuf[i].freq_ratio_z = 0.0f;
    }
    g_trendFreqFlushed  = true;
    g_freqDriftSuppress = 3;  // suppress drift 3 calcTrend cycles หลัง flush
    Serial.printf("[MOTOR] RUNNING -- freq_ratio flushed (all %u slots)\n", TREND_BUF_SIZE);
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
  data->rpm          = roundf(g_rpmFiltered * 10.0f) / 10.0f;
  data->motor_state  = (uint8_t)g_motorRunState;
  data->runtime_hour = roundf(getCurrentRuntimeHour() * 10000.0f) / 10000.0f;
  data->prox         = prox;
}


// ============================================================================
// RS485 CONTROL (Core 0)
// ============================================================================

static inline void rs485Enable() {
  digitalWrite(RS485_EN_PIN, LOW);
}

static inline void rs485Disable() {
  digitalWrite(RS485_EN_PIN, HIGH);
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

static uint32_t g_lastSensorRestart = 0;        // millis() ??? restart ?????? (shared cooldown)

static const uint16_t STUCK_THRESHOLD        = 5;      // 5 ????? x 250ms = 1.25 ?????? (????? 10)
static const int16_t  STUCK_MIN_RAW          = 5;      // |raw| > 5 (0.05 mm/s) ?????? "?????"
static const uint32_t SENSOR_RESTART_COOLDOWN = 30000; // ???? restart ???????? 30 ??????

/**
 * Re-configure WTVB02-485 หลัง reboot ผ่าน Modbus (v15.7)
 *
 * ลำดับ config ตาม log ที่กำหนด:
 *   1. Unlock#1 (0x69=0xB588) → SR=16K              (0x29=0x0001)  [1500 RPM motor]
 *   2. Unlock#2 (0x69=0xB588) → DRM=Freq domain     (0x2B=0x0002)
 *   3. Unlock#3 (0x69=0xB588) → MODE=TDLF           (0x07=0x0000)
 *   4. Unlock#4 (0x69=0xB588) → Save                (0x00=0x0000)
 *
 * @return true  ทุก step สำเร็จ
 *         false มี step ใดล้มเหลว (log warning แต่ caller ยังนับ restart ว่า OK)
 */
static bool reconfigSensorAfterRestart() {
  uint8_t result;
  bool allOk = true;

  Serial.println("[SENSOR-CFG] ========================================");
  Serial.println("[SENSOR-CFG] Re-configuring sensor after restart...");

  // ------------------------------------------------------------------
  // Step 1: Unlock#1 → SR = 16K
  // ------------------------------------------------------------------
  Serial.printf("[SENSOR-CFG] [Unlock#1 for SR]...");
  result = modbus.writeSingleRegister(REG_UNLOCK, SENSOR_UNLOCK_KEY);
  if (result != modbus.ku8MBSuccess) {
    Serial.printf(" x FAILED (err=%d)\n", result);
    allOk = false;
  } else {
    Serial.println(" + OK");
    vTaskDelay(pdMS_TO_TICKS(50));

    Serial.printf("[SENSOR-CFG] SR=16K/1500RPM (0x%02X=0x%04X)...", REG_SAMPLE_RATE, SENSOR_SR_16K);
    result = modbus.writeSingleRegister(REG_SAMPLE_RATE, SENSOR_SR_16K);
    if (result != modbus.ku8MBSuccess) {
      Serial.printf(" x FAILED (err=%d)\n", result);
      allOk = false;
    } else {
      Serial.println(" + OK");
    }
  }
  vTaskDelay(pdMS_TO_TICKS(100));

  // ------------------------------------------------------------------
  // Step 2: Unlock#2 → DRM = Frequency domain algorithm (0x002B = 0x0002)
  // จำเป็นสำหรับ VRMS registers (0x50/0x5C/0x68) ให้คำนวณค่าถูกต้อง
  // ------------------------------------------------------------------
  Serial.printf("[SENSOR-CFG] [Unlock#2 for DRM]...");
  result = modbus.writeSingleRegister(REG_UNLOCK, SENSOR_UNLOCK_KEY);
  if (result != modbus.ku8MBSuccess) {
    Serial.printf(" x FAILED (err=%d)\n", result);
    allOk = false;
  } else {
    Serial.println(" + OK");
    vTaskDelay(pdMS_TO_TICKS(50));

    Serial.printf("[SENSOR-CFG] DRM=Freq domain (0x%02X=0x%04X)...", REG_DRM, SENSOR_DRM_FREQ);
    result = modbus.writeSingleRegister(REG_DRM, SENSOR_DRM_FREQ);
    if (result != modbus.ku8MBSuccess) {
      Serial.printf(" x FAILED (err=%d)\n", result);
      allOk = false;
    } else {
      Serial.println(" + OK");
    }
  }
  vTaskDelay(pdMS_TO_TICKS(100));

  // ------------------------------------------------------------------
  // Step 3: Unlock#3 → MODE = Time domain low frequency algorithm
  // ------------------------------------------------------------------
  Serial.printf("[SENSOR-CFG] [Unlock#3 for MODE]...");
  result = modbus.writeSingleRegister(REG_UNLOCK, SENSOR_UNLOCK_KEY);
  if (result != modbus.ku8MBSuccess) {
    Serial.printf(" x FAILED (err=%d)\n", result);
    allOk = false;
  } else {
    Serial.println(" + OK");
    vTaskDelay(pdMS_TO_TICKS(50));

    Serial.printf("[SENSOR-CFG] MODE=Time domain low frequency algorithm (0x%02X=0x%04X)...",
                  REG_MODE, SENSOR_MODE_TDLF);
    result = modbus.writeSingleRegister(REG_MODE, SENSOR_MODE_TDLF);
    if (result != modbus.ku8MBSuccess) {
      Serial.printf(" x FAILED (err=%d)\n", result);
      allOk = false;
    } else {
      Serial.println(" + OK");
    }
  }
  vTaskDelay(pdMS_TO_TICKS(100));

  // ------------------------------------------------------------------
  // Step 4: Unlock#4 → Save config to NVM
  // ------------------------------------------------------------------
  Serial.printf("[SENSOR-CFG] [Unlock#4 before Save]...");
  result = modbus.writeSingleRegister(REG_UNLOCK, SENSOR_UNLOCK_KEY);
  if (result != modbus.ku8MBSuccess) {
    Serial.printf(" x FAILED (err=%d)\n", result);
    allOk = false;
  } else {
    Serial.println(" + OK");
    vTaskDelay(pdMS_TO_TICKS(50));

    Serial.printf("[SENSOR-CFG] Save (0x%02X=0x%04X)...", REG_CMD, SENSOR_CMD_SAVE);
    result = modbus.writeSingleRegister(REG_CMD, SENSOR_CMD_SAVE);
    if (result != modbus.ku8MBSuccess) {
      Serial.printf(" x FAILED (err=%d)\n", result);
      allOk = false;
    } else {
      Serial.println(" + OK");
    }
  }
  vTaskDelay(pdMS_TO_TICKS(200));  // รอ NVM write เสร็จก่อน resume

  if (allOk) {
    Serial.println("[SENSOR-CFG] All config steps OK");
  } else {
    Serial.println("[SENSOR-CFG] WARNING: Some config steps FAILED -- sensor may use default config");
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
  result = modbus.writeSingleRegister(REG_CMD, 0x00FF);
  if (result != modbus.ku8MBSuccess) {
    Serial.printf("[SENSOR] x Restart FAILED (err=%d)\n", result);
    return false;
  }

  Serial.printf("[SENSOR] + Restart command sent (reg 0x%02X = 0x00FF)\n", REG_CMD);
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

  // Wait for network registration (30 seconds)
  modem.waitForNetwork(30000L);

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

    if (modem.gprsConnect(APN, GPRS_USER, GPRS_PASS)) {
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
    DateTime rtcNow = rtc.now();
    int32_t drift = (int32_t)networkUTC.unixtime() - (int32_t)rtcNow.unixtime();
    g_timeSync.lastDriftSec = drift;

    Serial.printf("[NTP] RTC drift: %+d seconds\n", drift);

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
  return true;
}

/**
 * Check if NTP sync is due and perform it if needed.
 * Call this periodically from the network task.
 */
void checkAndSyncTime() {
  uint32_t now = millis();
  uint32_t interval = g_timeSync.synced ? NTP_SYNC_INTERVAL : NTP_SYNC_RETRY_INTERVAL;

  // On first run or after interval
  if (g_timeSync.lastSyncMillis == 0 || (now - g_timeSync.lastSyncMillis >= interval)) {
    Serial.printf("[NTP] Time sync check (interval=%lus, synced=%s)\n",
                  interval / 1000, g_timeSync.synced ? "yes" : "no");
    syncRTCFromModem();
  }
}

// ============================================================================
// CORE 0 TASKS - TIME CRITICAL OPERATIONS
// ============================================================================

/**
 * Task 1: Modbus RTU Communication (CORE 0, Priority 5)
 * Runs every 250ms
 * Reads sensor data via RS485 and sends to queue
 * v15.0: 3 Modbus transactions -- VEL(0x3A) + TEMP(0x40) + FREQ(0x44) + CF/K(0x47)
 */
void taskModbusRead(void* parameter) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(250);  // 250ms = 4Hz

  VibrationData_t localData;
  int16_t raw_x, raw_y, raw_z, raw_temp;
  int16_t raw_fx, raw_fy, raw_fz;
  uint16_t raw_cfx = 0, raw_kx = 0;  // v15.0: CFX (0x47), KX (0x48) -- unsigned per datasheet §6.4.14
  uint16_t raw_cfy = 0, raw_ky = 0;  // v15.1: CFY (0x53), KY (0x54) -- unsigned per datasheet §6.4.15
  uint16_t raw_cfz = 0, raw_kz = 0;  // v15.1: CFZ (0x5F), KZ (0x60) -- unsigned per datasheet §6.4.16

  Serial.println("[CORE 0] Modbus task started");

  while (1) {
    g_sensorReads++;

    bool success = true;

    // Read all registers (blocking I/O, but isolated to this task)
    // v15.0: 3 Modbus transactions -- VEL + TEMP + FREQ + CF/K
    rs485Enable();
    vTaskDelay(pdMS_TO_TICKS(5));  // 5ms stabilization

    // Transaction 1: Velocity RMS X, Y, Z (§6.4.14-16)
    // VRMSX=0x50, VRMSY=0x5C, VRMSZ=0x68 (ไม่ consecutive -- อ่านแยก 3 ครั้ง)
    // Scaling: raw / 1000.0f → mm/s (True RMS, ไม่ต้อง × 0.7071)
    if (modbus.readHoldingRegisters(REG_VRMS_X, 1) == modbus.ku8MBSuccess) {
      raw_x = (int16_t)modbus.getResponseBuffer(0);
    } else {
      success = false;
    }
    vTaskDelay(pdMS_TO_TICKS(5));

    if (modbus.readHoldingRegisters(REG_VRMS_Y, 1) == modbus.ku8MBSuccess) {
      raw_y = (int16_t)modbus.getResponseBuffer(0);
    } else {
      success = false;
    }
    vTaskDelay(pdMS_TO_TICKS(5));

    if (modbus.readHoldingRegisters(REG_VRMS_Z, 1) == modbus.ku8MBSuccess) {
      raw_z = (int16_t)modbus.getResponseBuffer(0);
    } else {
      success = false;
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
      raw_fx = (int16_t)modbus.getResponseBuffer(0);
      raw_fy = (int16_t)modbus.getResponseBuffer(1);
      raw_fz = (int16_t)modbus.getResponseBuffer(2);
    } else {
      raw_fx = 0;
      raw_fy = 0;
      raw_fz = 0;
    }
    vTaskDelay(pdMS_TO_TICKS(5));

    // Transaction 3: CFX (0x47) + KX (0x48) -- Accel Crest Factor & Kurtosis [v15.0]
    // Optional -- ถ้า fail ปล่อยค่าเดิม (0) ไม่กระทบ success หลัก
    if (modbus.readHoldingRegisters(REG_CFX, 2) == modbus.ku8MBSuccess) {
      raw_cfx = (uint16_t)modbus.getResponseBuffer(0); // v15.6 fix: unsigned per datasheet §6.4.14
      raw_kx  = (uint16_t)modbus.getResponseBuffer(1);
    }
    vTaskDelay(pdMS_TO_TICKS(5));

    // Transaction 4: CFY (0x53) + KY (0x54) -- Y-axis [v15.1]
    if (modbus.readHoldingRegisters(REG_CFY, 2) == modbus.ku8MBSuccess) {
      raw_cfy = (uint16_t)modbus.getResponseBuffer(0); // v15.6 fix: unsigned per datasheet §6.4.15
      raw_ky  = (uint16_t)modbus.getResponseBuffer(1);
    }
    vTaskDelay(pdMS_TO_TICKS(5));

    // Transaction 5: CFZ (0x5F) + KZ (0x60) -- Z-axis [v15.1]
    if (modbus.readHoldingRegisters(REG_CFZ, 2) == modbus.ku8MBSuccess) {
      raw_cfz = (uint16_t)modbus.getResponseBuffer(0); // v15.6 fix: unsigned per datasheet §6.4.16
      raw_kz  = (uint16_t)modbus.getResponseBuffer(1);
    }
    // ทั้ง T3/T4/T5 เป็น optional -- ไม่ set success = false ถ้า fail

    rs485Disable();

    if (success) {
      // -- + ??????????: Reset consecutive error counter --
      if (g_modbusConsecErrors > 0) {
        Serial.printf("[MODBUS] + Sensor back ONLINE (was offline for %u consecutive reads, "
                      "total errors: %lu)\n",
                      g_modbusConsecErrors, g_sensorErrors);
        g_modbusConsecErrors = 0;
        g_sensorOffline      = false;
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
        rs485Enable();
        vTaskDelay(pdMS_TO_TICKS(5));

        if (restartSensorViaModbus(stuckAxis)) {
          Serial.printf("[SENSOR] + Auto-restart OK (axis=%s), monitoring recovery...\n", stuckAxis);
        } else {
          Serial.printf("[SENSOR] x Auto-restart FAILED (axis=%s), retry after cooldown\n", stuckAxis);
          // reset counters ????????????????????????? cooldown
          g_vxStuckCount = 0;
          g_vyStuckCount = 0;
          g_vzStuckCount = 0;
        }

        rs485Disable();

        // Reset timing ????? restart ??????? ~3 ??????
        xLastWakeTime = xTaskGetTickCount();
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        continue;  // ?????????? ?????????????
      }

      // ============================================================
      // DATA PROCESSING [v15.0]
      // ============================================================

      // Step 1: True Peak Velocity (raw/100) -- ค่าดิบจาก register ก่อนแปลง
      // VRMS (0x50/0x5C/0x68) ส่งค่า True RMS velocity [mm/s] (§6.4.14-16)
      // Scaling: raw / 1000.0f → mm/s (ไม่ต้อง × 0.7071 เพราะเป็น True RMS แล้ว)
      localData.vel_peak_x       = abs(raw_x) / 1000.0f;  // [mm/s] VRMS X
      localData.vel_peak_y       = abs(raw_y) / 1000.0f;  // [mm/s] VRMS Y
      localData.vel_peak_z       = abs(raw_z) / 1000.0f;  // [mm/s] VRMS Z
      localData.vel_peak_overall = max(localData.vel_peak_x,
                                       max(localData.vel_peak_y, localData.vel_peak_z));

      // Step 2: Estimated RMS (Peak / √2) -- ใช้สำหรับ threshold / state machine
      // สมมติ sinusoidal vibration (standard approximation, rotating machinery)
      // VRMS = True RMS แล้ว -- ใช้ค่าตรงโดยไม่ต้องแปลง
      localData.rms_x       = localData.vel_peak_x;
      localData.rms_y       = localData.vel_peak_y;
      localData.rms_z       = localData.vel_peak_z;
      localData.rms_overall = max(localData.rms_x,
                                  max(localData.rms_y, localData.rms_z));

      // Step 3: Sensor-computed CF & Kurtosis -- ครบ 3 แกน [v15.1]
      // คำนวณจาก 16KHz raw FIFO ภายใน chip
      // raw = 0 ถ้า transaction fail (ปลอดภัย -- guard > 0)
      localData.cf_x       = (raw_cfx > 0) ? raw_cfx / 1000.0f : 0.0f;
      localData.cf_y       = (raw_cfy > 0) ? raw_cfy / 1000.0f : 0.0f;
      localData.cf_z       = (raw_cfz > 0) ? raw_cfz / 1000.0f : 0.0f;

      localData.kurtosis_x = (raw_kx  > 0) ? raw_kx  / 1000.0f : 0.0f;
      localData.kurtosis_y = (raw_ky  > 0) ? raw_ky  / 1000.0f : 0.0f;
      localData.kurtosis_z = (raw_kz  > 0) ? raw_kz  / 1000.0f : 0.0f;

      // Derived: max CF และ max Kurtosis พร้อม dominant axis [v15.1]
      // kurtosis_max ใช้ใน bearing alert: > 4.0 = early warning, > 6.0 = confirmed
      // kurtosis_dominant_axis ใช้ localize fault: 0=X(radial), 1=Y(radial), 2=Z(axial)
      localData.cf_max = max(localData.cf_x, max(localData.cf_y, localData.cf_z));

      if (localData.kurtosis_x >= localData.kurtosis_y &&
          localData.kurtosis_x >= localData.kurtosis_z) {
        localData.kurtosis_max            = localData.kurtosis_x;
        localData.kurtosis_dominant_axis  = 0;  // X
      } else if (localData.kurtosis_y >= localData.kurtosis_z) {
        localData.kurtosis_max            = localData.kurtosis_y;
        localData.kurtosis_dominant_axis  = 1;  // Y
      } else {
        localData.kurtosis_max            = localData.kurtosis_z;
        localData.kurtosis_dominant_axis  = 2;  // Z
      }
      // ถ้าทุกแกน = 0 (transactions ทั้งหมด fail) kurtosis_max = 0 -- ไม่ trigger alert

      // Step 4: Misc
      localData.temperature = raw_temp / 100.0f;
      localData.freq_x = raw_fx / 10.0f;
      localData.freq_y = raw_fy / 10.0f;
      localData.freq_z = raw_fz / 10.0f;

      // Step 5: Velocity Peak Hold -- v16.0: gate ด้วย MOTOR_RUNNING
      // STARTING/STOPPING: transient spike ไม่มีความหมาย mechanical → ไม่ update hold
      // RUNNING เท่านั้น: สะสมค่าสูงสุดตลอด publish interval
      // Core 0 เขียน / Core 1 อ่าน+reset -- atomic float write (ESP32 4-byte aligned)
      if (g_motorRunState == MOTOR_RUNNING &&
          localData.vel_peak_overall > g_velPeakHold) {
        g_velPeakHold = localData.vel_peak_overall;  // [mm/s] true peak hold
      }
      localData.peak = g_velPeakHold;
      // ---------------------------------------------------------------

      localData.timestamp = millis();
      localData.valid = true;

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
        // ???????? taskStateMachine ??????? g_vibData ???????????? ERROR ?? display
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
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
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
        // ???? g_vibData ??????? 0 ??? mark invalid
        // ????????????????????????????? display / MQTT
        if (xSemaphoreTake(mutexVibData, pdMS_TO_TICKS(10)) == pdTRUE) {
          memset(&g_vibData, 0, sizeof(VibrationData_t));
          g_vibData.valid     = false;
          g_vibData.timestamp = sensorData.timestamp;
          xSemaphoreGive(mutexVibData);
        }

        // Reset peak hold เมื่อ sensor offline
        // ป้องกัน peak ค้างข้ามช่วง offline -> online [v15.0: hold = true peak]
        g_velPeakHold = 0.0f;

        // ?????? state ???? NORMAL -- ???? trigger alarm ??? sensor ???????
        if (xSemaphoreTake(mutexSystemState, pdMS_TO_TICKS(10)) == pdTRUE) {
          if (g_systemState.state != STATE_MAINTENANCE) {
            if (g_systemState.state != STATE_NORMAL) {
              Serial.println("[STATE] Sensor OFFLINE -> forced STATE_NORMAL, buzzer OFF");
            }
            g_systemState.state       = STATE_NORMAL;
            g_systemState.buzzerActive = false;
          }
          xSemaphoreGive(mutexSystemState);
        }
        continue;  // ??????????????? RPM / state machine ?????????
      }

      // -- + ?????????????: ???????????? --
      processRPM(&sensorData);

      // Update shared vibration data (with mutex)
      if (xSemaphoreTake(mutexVibData, pdMS_TO_TICKS(10)) == pdTRUE) {
        memcpy(&g_vibData, &sensorData, sizeof(VibrationData_t));
        xSemaphoreGive(mutexVibData);
      }

      // -- Push sample ???? Trend Buffer (Core 0 only, no mutex needed) --
      // ????? freq_ratio ? ???????????????? drift detection
      {
        // v16.0: gate freq_ratio ด้วย MOTOR_RUNNING
        // STARTING/STOPPING: ratio ไม่ stable → เขียน 0 ลง trendBuf
        float ratX = 0.0f, ratY = 0.0f, ratZ = 0.0f;
        if (g_motorRunState == MOTOR_RUNNING && sensorData.rpm >= 100.0f) {
          float rf = sensorData.rpm / 60.0f;
          ratX = sensorData.freq_x / rf;
          ratY = sensorData.freq_y / rf;
          ratZ = sensorData.freq_z / rf;
        }
        g_trendBuf[g_trendHead] = {
          sensorData.rms_overall,
          g_velPeakHold,           // snapshot peak ? ??????? push
          sensorData.temperature,
          ratX, ratY, ratZ
        };
        g_trendHead  = (g_trendHead + 1) % TREND_BUF_SIZE;
        if (g_trendCount < TREND_BUF_SIZE) g_trendCount++;
      }

      // Determine new state based on RMS
      // v16.0: gate ด้วย MOTOR_RUNNING -- STARTING/STOPPING มี transient RMS สูง
      // ไม่ควร trigger STATE_WARNING/CRITICAL ขณะ ramp-up/down
      MachineState_t newState;
      float rms = sensorData.rms_overall;

      if (g_motorRunState != MOTOR_RUNNING) {
        newState = STATE_NORMAL;  // STOPPED/STARTING/STOPPING → ไม่ประเมิน alarm
      } else if (rms < WARNING_RMS) {
        newState = STATE_NORMAL;
      } else if (rms < CRITICAL_RMS) {
        newState = STATE_WARNING;
      } else {
        newState = STATE_CRITICAL;
      }

      // Update system state (with mutex)
      if (xSemaphoreTake(mutexSystemState, pdMS_TO_TICKS(10)) == pdTRUE) {
        MachineState_t oldState = g_systemState.state;

        // Skip if in maintenance mode
        if (g_systemState.state != STATE_MAINTENANCE) {
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

            Serial.printf("[CORE 0] State: %d -> %d (RMS: %.2f)\n",
                          oldState, newState, rms);
          }
        }

        xSemaphoreGive(mutexSystemState);
      }

      // ── Fault Latch v3: evaluate fault transitions ────────────────────────
      {
        int latchHealth = 100;
        if (sensorData.motor_state == 2 && sensorData.rms_overall > BASELINE_RMS) {
          float norm = (sensorData.rms_overall - BASELINE_RMS) /
                       (CRITICAL_RMS - BASELINE_RMS) * 100.0f;
          latchHealth = (int)max(0.0f, min(100.0f, roundf(100.0f - norm)));
        }
        const bool latchBearing =
          (sensorData.motor_state == 2) &&
          (g_bearingStableCnt >= BEARING_STABLE_CYCLES) &&
          (sensorData.kurtosis_max >= KURTOSIS_CONFIRMED);

        checkAndLatchFault(&sensorData, newState, latchHealth, latchBearing);
      }
      // ── End Fault Latch ───────────────────────────────────────────────────
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
    if (xSemaphoreTake(mutexVibData, pdMS_TO_TICKS(5)) == pdTRUE) {
      memcpy(&localVibData, &g_vibData, sizeof(VibrationData_t));
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
  if (!modemInit()) {
    Serial.println("[CORE 1] Modem init failed!");
    // Continue running but in error state
  } else {
    // Enable automatic network time update on the modem
    modemEnableNetworkTime();
  }

  // Connect to GPRS if modem is ready
  if (g_network.modemReady) {
    modemConnectGPRS();

    // Perform initial time sync after GPRS connects
    if (g_network.gprsConnected) {
      Serial.println("[CORE 1] Performing initial NTP time sync...");
      // Wait a moment for modem to receive network time
      vTaskDelay(pdMS_TO_TICKS(3000));
      syncRTCFromModem();
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

  VibrationData_t localVibData;
  MachineState_t localState;

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
        }

        // Try to reconnect GPRS if network is up but GPRS is down
        if (network && !gprs) {
          Serial.println("[CORE 1] Reconnecting GPRS...");
          esp_task_wdt_reset();   // v15.4: gprsConnect อาจใช้เวลา
          modem.gprsConnect(APN, GPRS_USER, GPRS_PASS);
          vTaskDelay(pdMS_TO_TICKS(5000));
          esp_task_wdt_reset();
          gprs = modem.isGprsConnected();
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
        if (gprs && !mqttClient.connected() &&
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
        if (gprs && mqttClient.connected()) {
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
            g_timeSync.lastSyncMillis = 0;  // Force immediate sync check
          }
          lastGprsState = gprs;
        }

        if (mqttClient.connected() != lastMqttState) {
          Serial.printf("[CORE 1] MQTT: %s\n", mqttClient.connected() ? "CONNECTED" : "DISCONNECTED");
          lastMqttState = mqttClient.connected();
        }
      }
    }

    // -- mqttClient.loop() ??? iteration = ??? 100ms --
    // ????????????? publish ????? process ACK/PINGREQ ??????
    if (mqttClient.connected()) {
      mqttClient.loop();
    }

    // Periodic NTP time sync check
    if (g_network.gprsConnected) {
      checkAndSyncTime();
    }

    // Get current sensor data
    if (xSemaphoreTake(mutexVibData, pdMS_TO_TICKS(5)) == pdTRUE) {
      memcpy(&localVibData, &g_vibData, sizeof(VibrationData_t));
      xSemaphoreGive(mutexVibData);
    }

    if (xSemaphoreTake(mutexSystemState, pdMS_TO_TICKS(5)) == pdTRUE) {
      localState = g_systemState.state;
      xSemaphoreGive(mutexSystemState);
    }

    // state-change boost, and minimum floor (3 s).
    uint32_t publishInterval;
    switch (localState) {
      case STATE_WARNING:  publishInterval = 10000; break;
      case STATE_CRITICAL: publishInterval =  5000; break;
      default:             publishInterval = 30000; break;
    }

    // -- Publish telemetry (?? FreeRTOS task ???????? ???????? ISR) --
    if (mqttClient.connected() && (now - lastPublish >= publishInterval)) {
      if (localVibData.valid) {
        // -- Normal telemetry --
        if (publishTelemetry(&localVibData, localState)) {
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
            DateTime rtcNow = rtc.now();
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
          if (mqttClient.publish(g_mqttTopicSensor, offlineJson, (int)szOffline, false, MQTT_QOS)) {
            lastOfflinePublish = now;
            g_network.publishCount++;
            Serial.printf("[MQTT] ! Offline alert published: %s\n", offlineJson);
          } else {
            g_network.publishFailures++;
          }
        }
      }
    }

    // ── V14.4: Maintenance reset MQTT audit (queued from taskButtonHandler) ──
    // Drains up to QUEUE_SIZE_MAINT events per loop tick (should be ≤1 normally).
    {
      MaintenanceEvent_t mEvt;
      while (xQueueReceive(queueMaintEvent, &mEvt, 0) == pdPASS) {
        if (mqttClient.connected()) {
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

          if (mqttClient.publish(g_mqttTopicEvent, evBuf, (int)szEvt, false, MQTT_QOS)) {
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

      if (mqttClient.connected() && snapPending) {

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
        int         snapHealth     = 100;
        {
          VibrationData_t snapVib = {};
          MachineState_t  snapState = STATE_NORMAL;
          if (xSemaphoreTake(mutexVibData, pdMS_TO_TICKS(5)) == pdTRUE) {
            memcpy(&snapVib, &g_vibData, sizeof(VibrationData_t));
            xSemaphoreGive(mutexVibData);
          }
          if (xSemaphoreTake(mutexSystemState, pdMS_TO_TICKS(5)) == pdTRUE) {
            snapState = g_systemState.state;
            xSemaphoreGive(mutexSystemState);
          }
          if (snapVib.motor_state == 2) {
            snapAlarmCode  = (snapState == STATE_CRITICAL) ? 2 :
                             (snapState == STATE_WARNING)  ? 1 : 0;
            snapAlarmLevel = (snapAlarmCode == 2) ? "CRITICAL" :
                             (snapAlarmCode == 1) ? "WARNING"  : "NORMAL";
            float flNorm = (snapVib.rms_overall - BASELINE_RMS) /
                           (CRITICAL_RMS - BASELINE_RMS) * 100.0f;
            snapHealth = (int)max(0.0f, min(100.0f, roundf(100.0f - flNorm)));
          }
        }

        char flNowTs[26] = "not_available";
        if (g_rtcValid) {
          DateTime rtcNow = rtc.now();
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
        flDoc["fault_kurtosis"]     = roundf(snapKurt * 1000.0f) / 1000.0f;
        flDoc["fault_latch_count"]  = snapCount;

        char   flBuf[1024];
        size_t flSz = serializeJson(flDoc, flBuf, sizeof(flBuf));
        if (flSz == 0 || flSz >= sizeof(flBuf) - 1) {
          Serial.printf("[LATCH] JSON overflow sz=%u — skipping\n", (unsigned)flSz);
        } else {
          if (mqttClient.publish(g_mqttTopicDecision, flBuf, (int)flSz,
                                 false, MQTT_QOS)) {
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

            // --- Reset Trend Buffers ---
            memset(g_trendBuf, 0, sizeof(g_trendBuf));
            g_trendHead  = 0;
            g_trendCount = 0;

            memset(g_buf1s,  0, sizeof(g_buf1s));
            g_buf1sHead  = 0; g_buf1sCount  = 0;

            memset(g_buf10s, 0, sizeof(g_buf10s));
            g_buf10sHead = 0; g_buf10sCount = 0;

            memset(g_buf60s, 0, sizeof(g_buf60s));
            g_buf60sHead = 0; g_buf60sCount = 0;

            // --- Reset Variance (must NOT be zero) ---
            g_slopeVar_1s  = 1e-6f;
            g_slopeVar_10s = 1e-6f;
            g_slopeVar_60s = 1e-6f;

            // --- Reset Peak Hold (V14.8) ---
            // g_velPeakHold persists across the reset without this line: the
            // first post-maintenance /vibration publish (taskNetwork PUB-1)
            // snapshots g_velPeakHold before overwriting it, so the broker
            // receives the pre-maintenance peak (e.g. 3.2 mm/s from a fault
            // run) as the peak of the very first post-reset cycle — corrupt data.
            // Setting to 0 here mirrors the sensor-offline branch (line ~2547)
            // and the normal per-publish reset in taskNetwork (line ~4185).
            g_velPeakHold = 0.0f;

            // --- Reset EMA ---
            g_emaRms     = 0.0f;
            g_emaPrevRms = 0.0f;
            g_emaDelta   = 0.0f;
            g_emaDir     = 0;

            memset(&g_trendResult, 0, sizeof(g_trendResult));

            // stale fault/score (e.g. RESONANCE sc=1.00) to remain published
            // until runDecisionEngine() accumulates enough new data.

            // OSG suppression; without reset they carry stale TTW weights into
            // the first post-warmup decision cycle.

            //   stateChanged=true → g_stateChangeCyc=3 → publish drops to 5s
            //   for 3 unnecessary cycles post-warmup.
            //   on first cycle, skewing adaptive publish interval.
            //   zeroed so ttwRoC is suppressed (condition: prevTtwRoC>0)
            //   on the first post-warmup call.
            //   not from before the maintenance event (harmless when
            //   evidence[]=0 but avoids a spuriously large dt on first update).

            // --- Reset Analytics Task Internal State (V14.4) ---
            // millis accumulators: reset so flush cadence restarts from zero,
            // preventing the immediate re-flush storm after maintenance reset.
            g_accMs_1s  = 0;
            g_accMs_10s = 0;
            g_accMs_60s = 0;
            // g_slotDur1sMs: reset to default 1000ms (V14.8).
            // taskAnalytics recomputes this from live RPM on its very first
            // tick after resume (line ~4499). Without this reset, if the pre-
            // maintenance RPM produced a wide slot (e.g. slow-spin = 2800ms),
            // the first flush threshold after reset is 2800ms instead of the
            // expected 1000ms — analytics appears "stuck" for up to 2.8s.
            g_slotDur1sMs = 1000UL;
            // Raw-buffer bookkeeping: force firstRun so lastHead re-syncs
            g_anaLastHead   = 0;
            g_anaFirstRun   = true;
            g_anaPublishCnt = 0;
            // Per-slot running accumulators
            g_sl_sumRms   = 0.0f;  g_sl_sumSqRms = 0.0f;  g_sl_maxRms  = 0.0f;
            g_sl_sumTemp  = 0.0f;  g_sl_maxTemp  = 0.0f;
            g_sl_sumPeak  = 0.0f;  g_sl_maxPeak  = 0.0f;
            g_sl_sumFrx   = 0.0f;  g_sl_sumFry   = 0.0f;  g_sl_sumFrz  = 0.0f;
            g_sl_spikes   = 0;
            g_sl_n        = 0;

            // Resume processing
            if (taskHandleAnalytics != NULL) {
              vTaskResume(taskHandleAnalytics);
            }

            // ── Enter Warm-up phase ──
            g_systemState.state = STATE_WARMUP;
            g_warmupStartTs    = millis();
            g_bearingStableCnt = 0;
            g_trendFreqFlushed  = false;
            g_freqDriftSuppress = 0;

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

  // Line 3: RMS value (y=38) - LARGE
  u8g2.setFont(u8g2_font_ncenB10_tr);
  snprintf(buf, sizeof(buf), "%.2f", data->rms_overall);
  uint8_t w = u8g2.getStrWidth(buf);
  u8g2.drawStr((128 - w) / 2, 40, buf);

  // Unit
  u8g2.setFont(u8g2_font_5x7_tr);
  u8g2.drawStr((128 - w) / 2 + w + 2, 40, "mm/s");

  // Line 4-5: Thresholds (y=50)
  u8g2.setFont(u8g2_font_6x10_tr);
  snprintf(buf, sizeof(buf), "WARN %.1f", WARNING_RMS);
  u8g2.drawStr(0, 52, buf);

  snprintf(buf, sizeof(buf), "CRIT %.1f", CRITICAL_RMS);
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

void drawWarningScreen(VibrationData_t* data, bool blink) {
  char buf[32];

  u8g2.setFont(u8g2_font_6x10_tr);
  u8g2.drawStr(0, 10, MACHINE_NAME);
  u8g2.drawStr(100, 10, "[1/3]");

  if (blink) {
    u8g2.setFont(u8g2_font_ncenB08_tr);
    u8g2.drawStr(30, 26, "! WARNING !");
  }

  u8g2.setFont(u8g2_font_ncenB10_tr);
  snprintf(buf, sizeof(buf), "%.2f", data->rms_overall);
  u8g2.drawStr(40, 40, buf);
  u8g2.setFont(u8g2_font_5x7_tr);
  u8g2.drawStr(90, 40, "mm/s");

  u8g2.setFont(u8g2_font_6x10_tr);
  snprintf(buf, sizeof(buf), "CRIT: %.1f", CRITICAL_RMS);
  u8g2.drawStr(20, 54, buf);

  u8g2.setFont(u8g2_font_5x7_tr);
  snprintf(buf, sizeof(buf), "X:%.1f Y:%.1f Z:%.1f",
           data->rms_x, data->rms_y, data->rms_z);
  u8g2.drawStr(0, 64, buf);
}

void drawCriticalScreen(VibrationData_t* data, bool blink) {
  char buf[32];

  if (blink) {
    u8g2.setFont(u8g2_font_ncenB08_tr);
    u8g2.drawStr(30, 16, "! CRITICAL !");
  }

  u8g2.setFont(u8g2_font_ncenB10_tr);
  snprintf(buf, sizeof(buf), "%.2f", data->rms_overall);
  u8g2.drawStr(40, 30, buf);

  u8g2.setFont(u8g2_font_5x7_tr);
  snprintf(buf, sizeof(buf), "LIMIT: %.1f mm/s", CRITICAL_RMS);
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

  // Row 1: VX / FX   (y=26)
  snprintf(buf, sizeof(buf), "VX = %03.2f", data->rms_x);
  u8g2.drawStr(5, 26, buf);
  snprintf(buf, sizeof(buf), "FX = %02.0f", data->freq_x);
  u8g2.drawStr(70, 26, buf);

  // Row 2: VY / FY   (y=37)
  snprintf(buf, sizeof(buf), "VY = %03.2f", data->rms_y);
  u8g2.drawStr(5, 37, buf);
  snprintf(buf, sizeof(buf), "FY = %02.0f", data->freq_y);
  u8g2.drawStr(70, 37, buf);

  // Row 3: VZ / FZ   (y=48)
  snprintf(buf, sizeof(buf), "VZ = %03.2f", data->rms_z);
  u8g2.drawStr(5, 48, buf);
  snprintf(buf, sizeof(buf), "FZ = %02.0f", data->freq_z);
  u8g2.drawStr(70, 48, buf);

  // Row 4: MAX / [2/3]  (y=60)
  snprintf(buf, sizeof(buf), "MAX= %03.2f", data->rms_overall);
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
  snprintf(buf, sizeof(buf), "MQTT: %s",
           mqttClient.connected() ? "CONN" : "DISC");
  u8g2.drawStr(5, 52, buf);

  // Publish stats + NTP status
  snprintf(buf, sizeof(buf), "Tx:%d %s",
           g_network.publishCount,
           g_timeSync.synced ? "NTP:OK" : "NTP:--");
  u8g2.drawStr(5, 62, buf);
}

static float aggLinRegSlope(const AggSample_t* buf, uint16_t bufHead,
                             uint16_t bufCount, uint16_t bufSize,
                             uint16_t windowSlots) {
  uint16_t n = (bufCount < windowSlots) ? bufCount : windowSlots;
  if (n < 4) return 0.0f;
  uint16_t startIdx = (bufHead + bufSize - n) % bufSize;
  double sumX=0.0, sumX2=0.0, sumY=0.0, sumXY=0.0;
  for (uint16_t i = 0; i < n; i++) {
    uint16_t idx = (startIdx + i) % bufSize;
    double x = (double)i;
    double y = (double)buf[idx].mean_rms;
    sumX  += x;
    sumX2 += x * x;
    sumY  += y;
    sumXY += x * y;
  }
  double denom = (double)n * sumX2 - sumX * sumX;
  if (denom == 0.0) return 0.0f;
  return (float)(((double)n * sumXY - sumX * sumY) / denom);
}

// Push one aggregated sample into a circular buffer of AggSample_t.
// Advances *head and grows *count up to bufSize (caller must hold
// mutexAggBufs before calling -- this function does no locking itself).
static void pushAggBuf(AggSample_t* buf, volatile uint16_t* head,
                        volatile uint16_t* count, uint16_t bufSize,
                        const AggSample_t* sample) {
  buf[*head] = *sample;
  *head = (uint16_t)((*head + 1) % bufSize);
  if (*count < bufSize) (*count)++;
}

// Population variance of mean_rms over the most recent `windowSlots` entries
// of a circular AggSample_t buffer (or bufCount entries if fewer are
// available). Used as an OSG/FVRI confidence input alongside the linreg
// slope -- low variance = stable trend, high variance = noisy/unstable.
static float computeRmsVariance(const AggSample_t* buf, uint16_t bufHead,
                                 uint16_t bufCount, uint16_t bufSize,
                                 uint16_t windowSlots) {
  uint16_t n = (bufCount < windowSlots) ? bufCount : windowSlots;
  if (n < 2) return 1e-6f;
  uint16_t startIdx = (bufHead + bufSize - n) % bufSize;
  double sum = 0.0, sumSq = 0.0;
  for (uint16_t i = 0; i < n; i++) {
    uint16_t idx = (startIdx + i) % bufSize;
    double v = (double)buf[idx].mean_rms;
    sum   += v;
    sumSq += v * v;
  }
  double mean = sum / n;
  double var  = (sumSq / n) - (mean * mean);
  if (var < 1e-6) var = 1e-6;  // never zero -- avoids div/0 in downstream consumers
  return (float)var;
}

// ============================================================================
// TREND ENGINE -- calcTrend()
// ============================================================================
// ???????? publishTelemetry() (Core 1) ???? build JSON
// ???? snapshot ??? g_trendBuf ? ???????????? -- thread-safe ???????????
// ????? float write ?? ESP32 (Xtensa LX7) ???? atomic 4-byte aligned
//
// ????? Phase 1:
//   1. rms_slope    -- Linear Regression (Least Squares) ??? rms ?? 30s window
//   2. temp_slope   -- Linear Regression ??? temperature
//   3. trend_dir    -- +1/0/-1 ??? rms_slope vs threshold
//   4. spike_count  -- ??? peak > WARNING_RMS x SPIKE_RMS_FACTOR ?? window
//   5. freq_drift   -- ???????????? freq_ratio ??????????????? window
//   6. ttw_hours    -- Time-to-Warning estimate ??? rms_slope + gap
//
// ????? Phase 2 (new):
//   7. slope_1s/10s/60s -- linreg ?? g_buf* multi-resolution
//   8. ema_dir/rms/delta -- snapshot ??? g_ema*
//   9. stddev_1min / max_rms_10min -- volatility indicators
// ============================================================================
static void calcTrend() {
  // -- Snapshot head + count ???????? --
  uint16_t snapHead  = g_trendHead;
  uint16_t snapCount = g_trendCount;

  // -- Phase 1: Single-resolution (30s window) -----------------------------
  if (snapCount >= TREND_MIN_SAMPLES) {
    uint16_t n = (snapCount < TREND_WINDOW_SAMPLES) ? snapCount : TREND_WINDOW_SAMPLES;
    uint16_t startIdx = (snapHead + TREND_BUF_SIZE - n) % TREND_BUF_SIZE;

    double sumX=0, sumX2=0;
    double sumRms=0, sumXRms=0;
    double sumTemp=0, sumXTemp=0;
    double sumFrX=0, sumFrY=0, sumFrZ=0;
    uint16_t spike_count = 0;

    for (uint16_t i = 0; i < n; i++) {
      uint16_t idx = (startIdx + i) % TREND_BUF_SIZE;
      TrendSample_t* s = &g_trendBuf[idx];
      sumX     += i;
      sumX2    += (double)i * i;
      sumRms   += s->rms;
      sumXRms  += (double)i * s->rms;
      sumTemp  += s->temp;
      sumXTemp += (double)i * s->temp;
      sumFrX   += s->freq_ratio_x;
      sumFrY   += s->freq_ratio_y;
      sumFrZ   += s->freq_ratio_z;
      if (s->peak > WARNING_RMS * SPIKE_RMS_FACTOR) spike_count++;
    }

    double denom = (double)n * sumX2 - sumX * sumX;
    float rmsSlope  = (denom != 0.0) ? (float)((n * sumXRms  - sumX * sumRms)  / denom) : 0.0f;
    float tempSlope = (denom != 0.0) ? (float)((n * sumXTemp - sumX * sumTemp) / denom) : 0.0f;

    float driftX = 0.0f, driftY = 0.0f, driftZ = 0.0f;
    if (n >= 20) {
      uint16_t half = n / 2;
      double s1X=0, s1Y=0, s1Z=0, s2X=0, s2Y=0, s2Z=0;
      for (uint16_t i = 0; i < half; i++) {
        uint16_t idx = (startIdx + i) % TREND_BUF_SIZE;
        s1X += g_trendBuf[idx].freq_ratio_x;
        s1Y += g_trendBuf[idx].freq_ratio_y;
        s1Z += g_trendBuf[idx].freq_ratio_z;
      }
      for (uint16_t i = half; i < n; i++) {
        uint16_t idx = (startIdx + i) % TREND_BUF_SIZE;
        s2X += g_trendBuf[idx].freq_ratio_x;
        s2Y += g_trendBuf[idx].freq_ratio_y;
        s2Z += g_trendBuf[idx].freq_ratio_z;
      }
      driftX = (float)((s2X - s1X) / half);
      driftY = (float)((s2Y - s1Y) / half);
      driftZ = (float)((s2Z - s1Z) / half);
    }
    // v16.0: suppress drift ใน N cycles แรกหลัง freq_ratio flush
    // ป้องกัน race condition ระหว่าง Core 0 flush กับ Core 1 calcTrend
    if (g_freqDriftSuppress > 0) {
      driftX = 0.0f;
      driftY = 0.0f;
      driftZ = 0.0f;
      g_freqDriftSuppress--;
    }
    bool freqAlert = (fabsf(driftX) > FREQ_DRIFT_THRESH ||
                      fabsf(driftY) > FREQ_DRIFT_THRESH ||
                      fabsf(driftZ) > FREQ_DRIFT_THRESH);

    int8_t trendDir = (rmsSlope >  TREND_SLOPE_UP)  ?  1 :
                      (rmsSlope <  TREND_SLOPE_DOWN) ? -1 : 0;

    float ttwHours  = 0.0f;
    float currentRms = g_trendBuf[(snapHead + TREND_BUF_SIZE - 1) % TREND_BUF_SIZE].rms;
    if (trendDir == 1 && currentRms < WARNING_RMS) {
      float ratePerHour = rmsSlope * 4.0f * 3600.0f;
      float gap = WARNING_RMS - currentRms;
      if (ratePerHour > 0.001f) {
        ttwHours = gap / ratePerHour;
        if (ttwHours > 9999.0f) ttwHours = 9999.0f;
      }
    }

    g_trendResult.rms_slope      = roundf(rmsSlope  * 100000.0f) / 100000.0f;
    g_trendResult.temp_slope     = roundf(tempSlope * 100000.0f) / 100000.0f;
    g_trendResult.trend_dir      = trendDir;
    g_trendResult.spike_count    = spike_count;
    g_trendResult.freq_drift_x   = roundf(driftX * 1000.0f) / 1000.0f;
    g_trendResult.freq_drift_y   = roundf(driftY * 1000.0f) / 1000.0f;
    g_trendResult.freq_drift_z   = roundf(driftZ * 1000.0f) / 1000.0f;
    g_trendResult.freq_alert     = freqAlert;
    g_trendResult.ttw_hours      = roundf(ttwHours * 10.0f) / 10.0f;
    g_trendResult.window_samples = n;

    Serial.printf("[TREND] n=%u slope=%.5f dir=%+d spikes=%u | "
                  "driftX=%.3f driftY=%.3f driftZ=%.3f alert=%d | "
                  "ttw=%.1fh tempSlope=%.5f\n",
                  n, rmsSlope, trendDir, spike_count,
                  driftX, driftY, driftZ, (int)freqAlert,
                  ttwHours, tempSlope);
  } else {
    // ????????????? -- zero Phase 1 fields ??????????? Phase 2 ???
    g_trendResult.rms_slope      = 0.0f;
    g_trendResult.temp_slope     = 0.0f;
    g_trendResult.trend_dir      = 0;
    g_trendResult.spike_count    = 0;
    g_trendResult.freq_drift_x   = 0.0f;
    g_trendResult.freq_drift_y   = 0.0f;
    g_trendResult.freq_drift_z   = 0.0f;
    g_trendResult.freq_alert     = false;
    g_trendResult.ttw_hours      = 0.0f;
    g_trendResult.window_samples = snapCount;
  }

  // -- Phase 2: Multi-Resolution Slopes ------------------------------------
  // hold mutexAggBufs ??????? g_buf* ???????????? taskAnalytics ?????????????
  if (xSemaphoreTake(mutexAggBufs, pdMS_TO_TICKS(5)) == pdTRUE) {

    // slope ready flags -- consumer knows when data is trustworthy
    g_trendResult.slope_ready_1s  = (g_buf1sCount  >= SLOPE_1S_MIN_SLOTS);
    g_trendResult.slope_ready_10s = (g_buf10sCount >= SLOPE_10S_MIN_SLOTS);
    g_trendResult.slope_ready_60s = (g_buf60sCount >= SLOPE_60S_MIN_SLOTS);

    // slope_1s: 30s window -- compute once buf1s >= 10 slots
    g_trendResult.slope_1s  = g_trendResult.slope_ready_1s
        ? aggLinRegSlope(g_buf1s,  g_buf1sHead,  g_buf1sCount, AGG_BUF_1S_SIZE,  30)
        : 0.0f;

    // slope_10s: 5-min window -- compute once buf10s >= 10 slots (~100 s)
    g_trendResult.slope_10s = g_trendResult.slope_ready_10s
        ? aggLinRegSlope(g_buf10s, g_buf10sHead, g_buf10sCount, AGG_BUF_10S_SIZE, 30)
        : 0.0f;

    // slope_60s: 30-min window -- compute once buf60s >= 20 slots (~20 min)
    g_trendResult.slope_60s = g_trendResult.slope_ready_60s
        ? aggLinRegSlope(g_buf60s, g_buf60sHead, g_buf60sCount, AGG_BUF_60S_SIZE, 30)
        : 0.0f;

    // stddev_1min and max_rms_10min
    {
      uint16_t n60 = (g_buf1sCount < AGG_BUF_1S_SIZE) ? g_buf1sCount : AGG_BUF_1S_SIZE;
      float sumSd = 0.0f;
      for (uint16_t i = 0; i < n60; i++) {
        sumSd += g_buf1s[(g_buf1sHead + AGG_BUF_1S_SIZE - n60 + i) % AGG_BUF_1S_SIZE].stddev_rms;
      }
      g_trendResult.stddev_1min = (n60 > 0) ? (sumSd / n60) : 0.0f;

      float maxRms10m = 0.0f;
      uint16_t n10m = (g_buf10sCount < AGG_BUF_10S_SIZE) ? g_buf10sCount : AGG_BUF_10S_SIZE;
      for (uint16_t i = 0; i < n10m; i++) {
        float mr = g_buf10s[(g_buf10sHead + AGG_BUF_10S_SIZE - n10m + i) % AGG_BUF_10S_SIZE].max_rms;
        if (mr > maxRms10m) maxRms10m = mr;
      }
      g_trendResult.max_rms_10min = maxRms10m;
    }

    xSemaphoreGive(mutexAggBufs);
  }

  // -- EMA snapshot ------------------------------------------------------
  g_trendResult.ema_dir   = g_emaDir;
  g_trendResult.ema_rms   = roundf(g_emaRms   * 1000.0f) / 1000.0f;
  g_trendResult.ema_delta = roundf(g_emaDelta * 100000.0f) / 100000.0f;

  Serial.printf("[TREND-P2] slope_1s=%s%.5f slope_10s=%s%.5f slope_60s=%s%.5f | "
                "ema=%.3f dir=%+d | stddev1m=%.3f maxRms10m=%.2f\n",
                g_trendResult.slope_ready_1s  ? "" : "~",  g_trendResult.slope_1s,
                g_trendResult.slope_ready_10s ? "" : "~",  g_trendResult.slope_10s,
                g_trendResult.slope_ready_60s ? "" : "~",  g_trendResult.slope_60s,
                g_trendResult.ema_rms, g_trendResult.ema_dir,
                g_trendResult.stddev_1min, g_trendResult.max_rms_10min);

  // -- EMA snapshot --
  g_trendResult.ema_dir   = g_emaDir;
  g_trendResult.ema_rms   = roundf(g_emaRms   * 1000.0f)  / 1000.0f;
  g_trendResult.ema_delta = roundf(g_emaDelta * 100000.0f) / 100000.0f;
}

// ============================================================================
// MQTT PUBLISHING (CORE 1)
// ============================================================================

bool publishTelemetry(VibrationData_t* data, MachineState_t state) {
  if (!mqttClient.connected()) return false;

  // ── Shared pre-computes ───────────────────────────────────────────────────
  // v16.0: gate alarm + health ด้วย MOTOR_RUNNING
  // STARTING/STOPPING: RMS transient สูง → ไม่ประเมิน alarm/health
  // ส่ง alarmCode=0 / alarmLevel="NORMAL" / healthScore=100 แทน
  int alarmCode;
  const char* alarmLevel;
  int healthScore;

  if (data->motor_state == 2) {  // MOTOR_RUNNING เท่านั้น
    alarmCode   = (state == STATE_CRITICAL) ? 2 :
                  (state == STATE_WARNING)  ? 1 : 0;
    alarmLevel  = (alarmCode == 2) ? "CRITICAL" :
                  (alarmCode == 1) ? "WARNING"  : "NORMAL";
    float normalized = (data->rms_overall - BASELINE_RMS) /
                       (CRITICAL_RMS - BASELINE_RMS) * 100.0f;
    healthScore = (int)max(0.0f, min(100.0f, roundf(100.0f - normalized)));
  } else {
    alarmCode   = 0;
    alarmLevel  = "NORMAL";
    healthScore = 100;  // ไม่ประเมิน health ขณะ STOPPED/STARTING/STOPPING
  }

  // v16.0: Peak gated by MOTOR_RUNNING
  // ถ้าไม่ใช่ RUNNING → peak = 0 (transient ไม่นับ)
  // reset hold เฉพาะตอน RUNNING เพื่อไม่ให้ค่าค้างข้าม state
  float currentPeak;
  if (data->motor_state == 2) {         // MOTOR_RUNNING
    currentPeak   = g_velPeakHold;
    g_velPeakHold = 0.0f;               // reset สำหรับ window ถัดไป
  } else {
    currentPeak   = 0.0f;               // STOPPED/STARTING/STOPPING → ไม่รายงาน peak
    g_velPeakHold = 0.0f;               // reset ทิ้งเพื่อไม่ค้างเข้า RUNNING ถัดไป
  }

  // v15.1: ใช้ cf_max (max ของทั้ง 3 แกน) แทน cf_x เพียงแกนเดียว
  // sensor คำนวณจาก raw 16KHz FIFO ภายใน chip:  CF = Peak_acc / RMS_acc
  float crestFactor = (data->cf_max > 0.0f)
                      ? roundf(data->cf_max * 100.0f) / 100.0f
                      : 0.0f;

  // v16.0: Bearing alert -- state-aware + stabilization gate
  // STOPPED/STARTING/STOPPING → ชื่อ state จริง (ไม่ใช่ INVALID_STATE)
  // RUNNING < BEARING_STABLE_CYCLES → WARMING_UP (หลีกเลี่ยง startup transient)
  // RUNNING ≥ BEARING_STABLE_CYCLES → ประเมิน kurtosis จริง
  const char* bearingAlert;
  if (data->motor_state == 2) {
    if (g_bearingStableCnt < BEARING_STABLE_CYCLES) {
      g_bearingStableCnt++;
      bearingAlert = "WARMING_UP";
    } else if (data->kurtosis_max >= KURTOSIS_CONFIRMED) {
      bearingAlert = "CONFIRMED";
    } else if (data->kurtosis_max >= KURTOSIS_EARLY_WARNING) {
      bearingAlert = "EARLY_WARNING";
    } else {
      bearingAlert = "NORMAL";
    }
  } else if (data->motor_state == 1) {
    bearingAlert = "STARTING";
  } else if (data->motor_state == 3) {
    bearingAlert = "STOPPING";
  } else {
    bearingAlert = "STOPPED";
  }
  const char* dominantAxis = (data->kurtosis_dominant_axis == 0) ? "X" :
                             (data->kurtosis_dominant_axis == 1) ? "Y" : "Z";

  // v16.0: kurtosis valid เฉพาะ MOTOR_RUNNING
  // ขณะ STOPPED/STARTING/STOPPING: noise floor → kurtosis สูงผิดปกติ (ไม่มีความหมาย)
  bool kurtosisValid = (data->motor_state == 2);  // MOTOR_RUNNING เท่านั้น
  float kx  = kurtosisValid ? round(data->kurtosis_x   * 1000) / 1000.0f : 0.0f;
  float ky  = kurtosisValid ? round(data->kurtosis_y   * 1000) / 1000.0f : 0.0f;
  float kz  = kurtosisValid ? round(data->kurtosis_z   * 1000) / 1000.0f : 0.0f;
  float kmax = kurtosisValid ? round(data->kurtosis_max * 1000) / 1000.0f : 0.0f;
  const char* kaxis = kurtosisValid ? dominantAxis : "-";

  // dominant_vibration_axis: แกนที่มี velocity RMS สูงสุด (ไม่เกี่ยวกับ kurtosis)
  // ใช้ rms_x/y/z (True RMS velocity จาก VRMS register)
  const char* domVibAxis;
  if (data->rms_x >= data->rms_y && data->rms_x >= data->rms_z) {
    domVibAxis = "X";
  } else if (data->rms_y >= data->rms_z) {
    domVibAxis = "Y";
  } else {
    domVibAxis = "Z";
  }

  // v16.0: Gate freq_ratio/freq_alert ด้วย MOTOR_RUNNING + RPM_FREQ_GATE
  // STARTING/STOPPING: RPM ไม่ stable → ratio ไม่มีความหมาย → suppressed
  float freqX = roundf(data->freq_x * 10.0f) / 10.0f;
  float freqY = roundf(data->freq_y * 10.0f) / 10.0f;
  float freqZ = roundf(data->freq_z * 10.0f) / 10.0f;
  float freqRatioX = 0.0f, freqRatioY = 0.0f, freqRatioZ = 0.0f;
  bool  freqGateOpen = (data->motor_state == 2 &&
                        data->rpm >= (float)RPM_FREQ_GATE);
  if (freqGateOpen) {
    float rotFreq  = data->rpm / 60.0f;
    freqRatioX = roundf((freqX / rotFreq) * 100.0f) / 100.0f;
    freqRatioY = roundf((freqY / rotFreq) * 100.0f) / 100.0f;
    freqRatioZ = roundf((freqZ / rotFreq) * 100.0f) / 100.0f;
  }

  calcTrend();   // same call as original
  const char* trendDirStr = (g_trendResult.trend_dir ==  1) ? "UP"   :
                            (g_trendResult.trend_dir == -1) ? "DOWN" : "STABLE";

  // Shared timestamp (built once, used in all three payloads)
  char tsBuf[26] = "not_available";
  if (g_rtcValid) {
    DateTime now = rtc.now();
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

    // Estimated RMS velocity (Peak / √2)
    s["rms"]   = round(data->rms_overall * 100) / 100.0f;
    s["vx"]    = round(data->rms_x       * 100) / 100.0f;
    s["vy"]    = round(data->rms_y       * 100) / 100.0f;
    s["vz"]    = round(data->rms_z       * 100) / 100.0f;

    // True peak velocity hold [mm/s] -- v15.0
    s["peak"]       = round(currentPeak            * 100) / 100.0f;
    s["vel_peak_x"] = round(data->vel_peak_x       * 100) / 100.0f;
    s["vel_peak_y"] = round(data->vel_peak_y       * 100) / 100.0f;
    s["vel_peak_z"] = round(data->vel_peak_z       * 100) / 100.0f;

    s["temp"]  = round(data->temperature *  10) /  10.0f;
    s["rpm"]   = data->rpm;

    // Harmonic feature extraction
    s["freq_x"]       = freqX;
    s["freq_y"]       = freqY;
    s["freq_z"]       = freqZ;
    s["freq_ratio_x"] = freqRatioX;
    s["freq_ratio_y"] = freqRatioY;
    s["freq_ratio_z"] = freqRatioZ;

    // v15.1: CF ครบ 3 แกน + max
    s["crest_factor"]   = crestFactor;                           // = cf_max
    s["cf_x"]           = round(data->cf_x * 100) / 100.0f;
    s["cf_y"]           = round(data->cf_y * 100) / 100.0f;
    s["cf_z"]           = round(data->cf_z * 100) / 100.0f;

    // v16.0: Kurtosis valid เฉพาะ MOTOR_RUNNING (ส่ง 0 เมื่อไม่ใช่ RUNNING)
    s["kurtosis_x"]     = kx;
    s["kurtosis_y"]     = ky;
    s["kurtosis_z"]     = kz;
    s["kurtosis_max"]   = kmax;
    s["kurtosis_axis"]           = kaxis;
    s["kurtosis_valid"]          = kurtosisValid;
    s["dominant_vibration_axis"] = domVibAxis;  // แกนที่ velocity RMS สูงสุด
    s["bearing_alert"]           = bearingAlert;

    s["motor_state"]           = data->motor_state;
    s["rotation_signal_ok"]    = data->prox;
    s["operating_hours_total"] = data->runtime_hour;

    // v15.3: Reset reason — ช่วยวินิจฉัย unexpected reboot
    s["reset_reason"]  = g_resetReasonStr;   // "POWER_ON" / "BROWNOUT" / "PANIC" etc.
    s["reboot_count"]  = g_rebootCount;      // สะสมทุก boot (NVS persistent)

    s["timestamp"]   = tsBuf;
    s["time_synced"] = g_timeSync.synced;

    char buf[1000];
    size_t szSensor = serializeJson(s, buf, sizeof(buf));
    if (szSensor == 0 || szSensor >= sizeof(buf) - 1) {
      Serial.printf("[WARN] /sensor JSON truncated! sz=%u buf=%u\n",
                    (unsigned)szSensor, (unsigned)sizeof(buf));
    }
    if (mqttClient.publish(g_mqttTopicSensor, buf, (int)szSensor, false, MQTT_QOS))
      Serial.printf("[MQTT] /sensor %u B\n", (unsigned)szSensor);
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

    StaticJsonDocument<768> d;
    d["plant"]               = PLANT_ID;
    d["machine_id"]          = MACHINE_ID;
    d["sensor_id"]           = SENSOR_ID;
    d["stage"]               = "status";
    d["execution_location"]  = "edge";

    d["alarm_code"]          = alarmCode;
    d["alarm_level"]         = alarmLevel;
    d["health_score"]        = healthScore;

    d["bearing_alert"]              = bearingAlert;
    d["kurtosis_max"]               = kmax;
    d["kurtosis_axis"]              = kaxis;
    d["kurtosis_valid"]             = kurtosisValid;
    d["dominant_vibration_axis"]    = domVibAxis;

    d["freq_alert"]          = freqGateOpen && g_trendResult.freq_alert;
    d["freq_drift_x"]        = freqGateOpen ? g_trendResult.freq_drift_x : 0.0f;
    d["freq_drift_y"]        = freqGateOpen ? g_trendResult.freq_drift_y : 0.0f;
    d["freq_drift_z"]        = freqGateOpen ? g_trendResult.freq_drift_z : 0.0f;

    d["trend_dir"]           = trendDirStr;
    d["rms_slope"]           = g_trendResult.rms_slope;
    d["spike_count"]         = g_trendResult.spike_count;
    if (g_trendResult.ttw_hours > 0.0f)
      d["ttw_estimate_h"]    = g_trendResult.ttw_hours;

    d["timestamp"]           = tsBuf;

    d["fault_latch_pending"] = snapFlPending;
    d["fault_latch_count"]   = snapFlCount;

    char buf[768];
    size_t szStatus = serializeJson(d, buf, sizeof(buf));
    if (szStatus == 0 || szStatus >= sizeof(buf) - 1)
      Serial.printf("[WARN] /status JSON truncated! sz=%u\n", (unsigned)szStatus);
    if (mqttClient.publish(g_mqttTopicDecision, buf, (int)szStatus, false, MQTT_QOS))
      Serial.printf("[MQTT] /status %u B\n", (unsigned)szStatus);
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

    doc["rms"]   = round(data->rms_overall * 100) / 100.0f;
    doc["vx"]    = round(data->rms_x       * 100) / 100.0f;
    doc["vy"]    = round(data->rms_y       * 100) / 100.0f;
    doc["vz"]    = round(data->rms_z       * 100) / 100.0f;
    doc["peak"]  = round(currentPeak        * 100) / 100.0f;
    doc["vel_peak_x"] = round(data->vel_peak_x * 100) / 100.0f;
    doc["vel_peak_y"] = round(data->vel_peak_y * 100) / 100.0f;
    doc["vel_peak_z"] = round(data->vel_peak_z * 100) / 100.0f;
    doc["temp"]  = round(data->temperature *  10) /  10.0f;
    doc["rpm"]   = data->rpm;
    doc["freq_x"] = freqX; doc["freq_y"] = freqY; doc["freq_z"] = freqZ;
    doc["freq_ratio_x"] = freqRatioX;
    doc["freq_ratio_y"] = freqRatioY;
    doc["freq_ratio_z"] = freqRatioZ;
    doc["state"]               = data->motor_state;
    doc["operating_hours_total"] = data->runtime_hour;
    doc["rotation_signal_ok"]  = data->prox;
    doc["alarm_code"]          = alarmCode;
    doc["alarm_level"]         = alarmLevel;
    doc["health_score"]        = healthScore;
    // v15.1: CF และ Kurtosis ครบ 3 แกน + derived
    doc["crest_factor"]     = crestFactor;                           // = cf_max
    doc["cf_x"]             = round(data->cf_x * 100) / 100.0f;
    doc["cf_y"]             = round(data->cf_y * 100) / 100.0f;
    doc["cf_z"]             = round(data->cf_z * 100) / 100.0f;
    // v16.0: Kurtosis valid เฉพาะ MOTOR_RUNNING
    doc["kurtosis_x"]       = kx;
    doc["kurtosis_y"]       = ky;
    doc["kurtosis_z"]       = kz;
    doc["kurtosis_max"]     = kmax;
    doc["kurtosis_axis"]           = kaxis;
    doc["kurtosis_valid"]          = kurtosisValid;
    doc["dominant_vibration_axis"] = domVibAxis;
    doc["bearing_alert"]           = bearingAlert;
    doc["sensor_status"]    = "ONLINE";

    doc["rms_slope"]      = g_trendResult.rms_slope;
    doc["temp_slope"]     = g_trendResult.temp_slope;
    doc["trend_dir"]      = trendDirStr;
    doc["spike_count"]    = g_trendResult.spike_count;
    // v15.2 Fix 17: suppress freq fields เมื่อ RPM < RPM_FREQ_GATE
    doc["freq_drift_x"]   = freqGateOpen ? g_trendResult.freq_drift_x : 0.0f;
    doc["freq_drift_y"]   = freqGateOpen ? g_trendResult.freq_drift_y : 0.0f;
    doc["freq_drift_z"]   = freqGateOpen ? g_trendResult.freq_drift_z : 0.0f;
    doc["freq_alert"]     = freqGateOpen && g_trendResult.freq_alert;
    doc["freq_gate_open"] = freqGateOpen;  // debug: บอกว่า gate เปิด/ปิด
    if (g_trendResult.ttw_hours > 0.0f)
      doc["ttw_hours"]    = g_trendResult.ttw_hours;
    doc["trend_window_s"] = (g_trendResult.window_samples * 250) / 1000;

    doc["slope_1s"]          = roundf(g_trendResult.slope_1s  * 100000.0f) / 100000.0f;
    doc["slope_10s"]         = roundf(g_trendResult.slope_10s * 100000.0f) / 100000.0f;
    doc["slope_60s"]         = roundf(g_trendResult.slope_60s * 100000.0f) / 100000.0f;
    doc["slope_ready_1s"]    = g_trendResult.slope_ready_1s;
    doc["slope_ready_10s"]   = g_trendResult.slope_ready_10s;
    doc["slope_ready_60s"]   = g_trendResult.slope_ready_60s;
    doc["ema_dir"]           = g_trendResult.ema_dir;
    doc["ema_rms"]           = g_trendResult.ema_rms;
    doc["stddev_1min"]       = roundf(g_trendResult.stddev_1min   * 1000.0f) / 1000.0f;
    doc["max_rms_10min"]     = roundf(g_trendResult.max_rms_10min * 100.0f)  / 100.0f;
    doc["agg_buf_1s"]        = g_buf1sCount;
    doc["agg_buf_10s"]       = g_buf10sCount;
    doc["agg_buf_60s"]       = g_buf60sCount;


    doc["timestamp"]   = tsBuf;
    doc["time_synced"] = g_timeSync.synced;
    if (g_timeSync.synced)
      doc["sync_age_s"] = (millis() - g_timeSync.lastSyncMillis) / 1000;

    char   jsonBuffer[2048];
    size_t jsonSize = serializeJson(doc, jsonBuffer, sizeof(jsonBuffer));
    success = mqttClient.publish(g_mqttTopic, jsonBuffer, (int)jsonSize, false, MQTT_QOS);

    if (success) {
      Serial.printf("[MQTT] /vibration %d B | %s rms=%.2f peak=%.2f(vpk=%.2f) rpm=%.1f "
                    "state=%d | health=%d%% | frx=%.2f fry=%.2f frz=%.2f | "
                    "cf=%.2f kurt_max=%.3f(%s) bear=%s\n",
                    jsonSize, alarmLevel,
                    data->rms_overall, currentPeak, data->vel_peak_overall, data->rpm,
                    data->motor_state,
                    healthScore, freqRatioX, freqRatioY, freqRatioZ,
                    crestFactor, kmax, kaxis, bearingAlert);
    } else {
      Serial.printf("[MQTT] Publish FAILED (err=%d)\n", mqttClient.lastError());
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

// ── Patent Claim 2 helper ────────────────────────────────────────────────────
// Compute RPM-adaptive slot duration for buf1s.
// Returns ms per slot clamped to [SLOT_DUR_MIN_MS, SLOT_DUR_MAX_MS].
// When rpm < MIN_RPM_VALID (motor stopped/starting) returns fixed 1000ms
// to avoid division near zero and spurious wide slots during transients.
static uint32_t computeSlotDurMs(float rpm) {
  if (rpm < (float)MIN_RPM_VALID) return 1000UL;  // motor not running -- keep default
  // target: SLOT_REVS_TARGET full revolutions per slot
  float ms = ((float)SLOT_REVS_TARGET * 60000.0f) / rpm;
  uint32_t dur = (uint32_t)ms;
  if (dur < SLOT_DUR_MIN_MS) dur = SLOT_DUR_MIN_MS;
  if (dur > SLOT_DUR_MAX_MS) dur = SLOT_DUR_MAX_MS;
  return dur;
}

void taskAnalytics(void* parameter) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xPeriod = pdMS_TO_TICKS(1000);  // 1 Hz — unchanged

  // ── Patent Claim 2: millis-based accumulators replace fixed counters ─────
  // acc*Ms and lastHead/firstRun are now globals (g_accMs_*, g_anaLastHead,
  // g_anaFirstRun) so taskButtonHandler can atomically reset them during
  // maintenance while taskAnalytics is suspended. Local aliases for readability.
  uint32_t& acc1sMs  = (uint32_t&)g_accMs_1s;
  uint32_t& acc10sMs = (uint32_t&)g_accMs_10s;
  uint32_t& acc60sMs = (uint32_t&)g_accMs_60s;

  // lastHead: ?? head ?????????????? -> ?? samples ???????????????? aggregate
  uint16_t& lastHead = (uint16_t&)g_anaLastHead;
  bool&     firstRun = (bool&)g_anaFirstRun;

  // Publish /analytics counter
  uint8_t&  analyticsPublishCnt = (uint8_t&)g_anaPublishCnt;

  Serial.println("[CORE 1] Analytics task started (Phase 2+Claim2)");

  while (1) {
    vTaskDelayUntil(&xLastWakeTime, xPeriod);

    // ── Patent Claim 2: update slot duration from current RPM ────────────
    // Read RPM (written atomically by Core 0 processRPM).
    // Update g_slotDur1sMs every tick so slot width adapts to operating speed.
    {
      float latestRpm = 0.0f;
      if (xSemaphoreTake(mutexVibData, pdMS_TO_TICKS(3)) == pdTRUE) {
        latestRpm = g_vibData.rpm;
        xSemaphoreGive(mutexVibData);
      }
      g_slotDur1sMs = computeSlotDurMs(latestRpm);
    }

    // Accumulate elapsed ms this tick (task period = 1000ms fixed)
    const uint32_t tickMs = 1000UL;
    acc1sMs  += tickMs;
    acc10sMs += tickMs;
    acc60sMs += tickMs;

    // -- Snapshot raw buffer state (Core 0 writes, float-atomic) --
    uint16_t snapHead  = g_trendHead;   // volatile read (atomic on Xtensa)
    uint16_t snapCount = g_trendCount;

    if (snapCount == 0) continue;

    // -- ?????????? new samples ??????? last read --
    // newSamples = ????? samples ??? Core 0 push ?????? 1 ?????? (??????? 4 +/-1)
    uint16_t newSamples;
    if (firstRun) {
      // ??????: ??? min(snapCount, 4) -- ???????????? 1 ??????
      newSamples = (snapCount < 4) ? snapCount : 4;
      lastHead   = (snapHead + TREND_BUF_SIZE - newSamples) % TREND_BUF_SIZE;
      firstRun   = false;
    } else {
      // ????: head ??????????? step ??? lastHead
      newSamples = (snapHead - lastHead + TREND_BUF_SIZE) % TREND_BUF_SIZE;
      if (newSamples == 0) {
        // Core 0 ?????? push ????? 1 ?????? (sensor offline?) -- ????
        analyticsPublishCnt++;
        goto analytics_publish;
      }
      // ???????????? 4 (buffer ??? overrun ??? wakeup ???)
      if (newSamples > 8) newSamples = 8;
    }

    // ── Patent Claim 2: rolling accumulator for variable-width slot ──────
    // Instead of building one a1s per tick and flushing immediately,
    // we accumulate raw samples into a_slot across multiple ticks.
    // When acc1sMs >= g_slotDur1sMs (the RPM-derived threshold), we
    // finalise and push the accumulated slot, then reset for the next one.
    // This ensures each buf1s slot always covers SLOT_REVS_TARGET revolutions
    // regardless of how many ticks fit within that time window.
    {
      // Per-slot running accumulators (persist across ticks between flushes)
      // V14.4: promoted to globals (g_sl_*) for maintenance reset support.
      float&   sl_sumRms   = (float&)g_sl_sumRms;
      float&   sl_sumSqRms = (float&)g_sl_sumSqRms;
      float&   sl_maxRms   = (float&)g_sl_maxRms;
      float&   sl_sumTemp  = (float&)g_sl_sumTemp;
      float&   sl_maxTemp  = (float&)g_sl_maxTemp;
      float&   sl_sumPeak  = (float&)g_sl_sumPeak;
      float&   sl_maxPeak  = (float&)g_sl_maxPeak;
      float&   sl_sumFrx   = (float&)g_sl_sumFrx;
      float&   sl_sumFry   = (float&)g_sl_sumFry;
      float&   sl_sumFrz   = (float&)g_sl_sumFrz;
      uint8_t& sl_spikes   = (uint8_t&)g_sl_spikes;
      uint8_t& sl_n        = (uint8_t&)g_sl_n;

      // Append this tick's samples into the running slot accumulator
      for (uint16_t i = 0; i < newSamples; i++) {
        uint16_t idx = (lastHead + i) % TREND_BUF_SIZE;
        const TrendSample_t* s = &g_trendBuf[idx];

        float rms  = s->rms;
        float peak = s->peak;
        float temp = s->temp;

        sl_sumRms   += rms;
        sl_sumSqRms += rms * rms;
        if (rms  > sl_maxRms)  sl_maxRms  = rms;
        sl_sumTemp  += temp;
        if (temp > sl_maxTemp) sl_maxTemp = temp;
        sl_sumPeak  += peak;
        if (peak > sl_maxPeak) sl_maxPeak = peak;
        sl_sumFrx   += s->freq_ratio_x;
        sl_sumFry   += s->freq_ratio_y;
        sl_sumFrz   += s->freq_ratio_z;
        if (peak > WARNING_RMS * SPIKE_RMS_FACTOR) sl_spikes++;
        if (sl_n < 255) sl_n++;  // guard uint8_t overflow (max 255 raw samples/slot)
      }

      // Advance lastHead — consumed up to snapHead
      lastHead = snapHead;

      if (sl_n == 0) goto analytics_ema;

      // ── Flush when slot duration threshold reached ──────────────────────
      if (acc1sMs >= g_slotDur1sMs) {
        acc1sMs = 0;

        float inv      = 1.0f / sl_n;
        float meanRms  = sl_sumRms * inv;
        float variance = (sl_sumSqRms * inv) - (meanRms * meanRms);
        float stddev   = (variance > 0.0f) ? sqrtf(variance) : 0.0f;

        AggSample_t a1s = {
          .mean_rms    = meanRms,
          .max_rms     = sl_maxRms,
          .stddev_rms  = stddev,
          .mean_temp   = sl_sumTemp * inv,
          .max_temp    = sl_maxTemp,
          .mean_peak   = sl_sumPeak * inv,
          .max_peak    = sl_maxPeak,
          .mean_frx    = sl_sumFrx * inv,
          .mean_fry    = sl_sumFry * inv,
          .mean_frz    = sl_sumFrz * inv,
          .spike_count = sl_spikes,
          .n_samples   = sl_n
        };

        // Reset slot accumulators for the next slot
        sl_sumRms = sl_sumSqRms = sl_maxRms  = 0.0f;
        sl_sumTemp = sl_maxTemp = 0.0f;
        sl_sumPeak = sl_maxPeak = 0.0f;
        sl_sumFrx  = sl_sumFry  = sl_sumFrz  = 0.0f;
        sl_spikes  = 0;
        sl_n       = 0;

        // -- Push 1s aggregate (hold mutex) --
        if (xSemaphoreTake(mutexAggBufs, pdMS_TO_TICKS(5)) == pdTRUE) {
          pushAggBuf(g_buf1s, &g_buf1sHead, &g_buf1sCount, AGG_BUF_1S_SIZE, &a1s);
          xSemaphoreGive(mutexAggBufs);
        }

        // -- Cascade -> 10s (target: 10 × slotDur1s real ms) ─────────────
        // acc10sMs threshold = 10 × g_slotDur1sMs, so the 10s slot always
        // covers exactly 10 × SLOT_REVS_TARGET revolutions regardless of RPM.
        if (acc10sMs >= 10UL * g_slotDur1sMs) {
          acc10sMs = 0;

          // Average ??? g_buf1s 10 slots ??????
          if (xSemaphoreTake(mutexAggBufs, pdMS_TO_TICKS(5)) == pdTRUE) {
            uint16_t take = (g_buf1sCount < 10) ? g_buf1sCount : 10;

            if (take > 0) {
              AggSample_t a10s = { 0 };
              for (uint16_t i = 0; i < take; i++) {
                uint16_t idx = (g_buf1sHead + AGG_BUF_1S_SIZE - take + i) % AGG_BUF_1S_SIZE;
                const AggSample_t* q = &g_buf1s[idx];
                a10s.mean_rms    += q->mean_rms;
                if (q->max_rms  > a10s.max_rms)  a10s.max_rms  = q->max_rms;
                a10s.stddev_rms  += q->stddev_rms;
                a10s.mean_temp   += q->mean_temp;
                if (q->max_temp > a10s.max_temp) a10s.max_temp = q->max_temp;
                a10s.mean_peak   += q->mean_peak;
                if (q->max_peak > a10s.max_peak) a10s.max_peak = q->max_peak;
                a10s.mean_frx    += q->mean_frx;
                a10s.mean_fry    += q->mean_fry;
                a10s.mean_frz    += q->mean_frz;
                a10s.spike_count += q->spike_count;
                a10s.n_samples   += q->n_samples;
              }
              float inv10 = 1.0f / take;
              a10s.mean_rms   *= inv10;
              a10s.stddev_rms *= inv10;
              a10s.mean_temp  *= inv10;
              a10s.mean_peak  *= inv10;
              a10s.mean_frx   *= inv10;
              a10s.mean_fry   *= inv10;
              a10s.mean_frz   *= inv10;
              pushAggBuf(g_buf10s, &g_buf10sHead, &g_buf10sCount, AGG_BUF_10S_SIZE, &a10s);
            }
            xSemaphoreGive(mutexAggBufs);
          }

          // -- Cascade -> 60s (target: 60 × slotDur1s real ms) ─────────
          // acc60sMs threshold = 60 × g_slotDur1sMs ≈ 60 s wall-clock
          // (adapts proportionally: same harmonic coverage at every speed)
          if (acc60sMs >= 60UL * g_slotDur1sMs) {
            acc60sMs = 0;

            if (xSemaphoreTake(mutexAggBufs, pdMS_TO_TICKS(5)) == pdTRUE) {
              uint16_t take6 = (g_buf10sCount < 6) ? g_buf10sCount : 6;

              if (take6 > 0) {
                AggSample_t a60s = { 0 };
                for (uint16_t i = 0; i < take6; i++) {
                  uint16_t idx = (g_buf10sHead + AGG_BUF_10S_SIZE - take6 + i) % AGG_BUF_10S_SIZE;
                  const AggSample_t* q = &g_buf10s[idx];
                  a60s.mean_rms    += q->mean_rms;
                  if (q->max_rms  > a60s.max_rms)  a60s.max_rms  = q->max_rms;
                  a60s.stddev_rms  += q->stddev_rms;
                  a60s.mean_temp   += q->mean_temp;
                  if (q->max_temp > a60s.max_temp) a60s.max_temp = q->max_temp;
                  a60s.mean_peak   += q->mean_peak;
                  if (q->max_peak > a60s.max_peak) a60s.max_peak = q->max_peak;
                  a60s.mean_frx    += q->mean_frx;
                  a60s.mean_fry    += q->mean_fry;
                  a60s.mean_frz    += q->mean_frz;
                  a60s.spike_count += q->spike_count;
                  a60s.n_samples   += q->n_samples;
                }
                float inv6 = 1.0f / take6;
                a60s.mean_rms   *= inv6;
                a60s.stddev_rms *= inv6;
                a60s.mean_temp  *= inv6;
                a60s.mean_peak  *= inv6;
                a60s.mean_frx   *= inv6;
                a60s.mean_fry   *= inv6;
                a60s.mean_frz   *= inv6;
                pushAggBuf(g_buf60s, &g_buf60sHead, &g_buf60sCount, AGG_BUF_60S_SIZE, &a60s);

                Serial.printf("[ANALYTICS] 60s flush -> buf1s=%u buf10s=%u buf60s=%u slot_ms=%lu\n",
                              g_buf1sCount, g_buf10sCount, g_buf60sCount, g_slotDur1sMs);
              }

              // -- Phase 5: Update slope variance trackers (OSG + FVRI inputs) --
              // Compute rolling variance of mean_rms for each buffer tier.
              // Called inside mutexAggBufs critical section -- safe to access buffers.
              // Use last 20 slots (coverage: 20×slotDur1s, 200×slotDur1s, 1200×slotDur1s).
              if (g_buf1sCount  >= 4)
                g_slopeVar_1s  = computeRmsVariance(g_buf1s,  g_buf1sHead,  g_buf1sCount,  AGG_BUF_1S_SIZE,  20);
              if (g_buf10sCount >= 4)
                g_slopeVar_10s = computeRmsVariance(g_buf10s, g_buf10sHead, g_buf10sCount, AGG_BUF_10S_SIZE, 20);
              if (g_buf60sCount >= 4)
                g_slopeVar_60s = computeRmsVariance(g_buf60s, g_buf60sHead, g_buf60sCount, AGG_BUF_60S_SIZE, 20);

              xSemaphoreGive(mutexAggBufs);
            }
          }
        }
      }
      // If acc1sMs < g_slotDur1sMs: samples already appended to sl_* accumulators above;
      // nothing else to do this tick — wait for the slot threshold to be reached.
    }

analytics_ema:
    // -- EMA update (??? 1 ??????) ------------------------------------------
    // ?????? rms ????????? g_trendBuf (1 sample ??????????? EMA ???????)
    {
      uint16_t lastIdx = (snapHead + TREND_BUF_SIZE - 1) % TREND_BUF_SIZE;
      float latestRms  = g_trendBuf[lastIdx].rms;

      float prevEma = g_emaRms;
      g_emaRms      = EMA_ALPHA * latestRms + (1.0f - EMA_ALPHA) * prevEma;
      g_emaDelta    = g_emaRms - g_emaPrevRms;
      g_emaPrevRms  = g_emaRms;

      g_emaDir = (g_emaDelta >  EMA_DIR_THRESHOLD) ?  1 :
                 (g_emaDelta < -EMA_DIR_THRESHOLD) ? -1 : 0;
    }

analytics_publish:
    analyticsPublishCnt++;
    if (analyticsPublishCnt < 60) continue;
    analyticsPublishCnt = 0;

    if (!mqttClient.connected()) continue;
    if (g_buf1sCount < 4)        continue;

    // Shared timestamp for this publish round (all 7 topics use same value)
    char tsA[26] = "not_available";
    if (g_rtcValid) {
      DateTime nowTs = rtc.now();
      snprintf(tsA, sizeof(tsA), "%04d-%02d-%02dT%02d:%02d:%02dZ",
               nowTs.year(), nowTs.month(), nowTs.day(),
               nowTs.hour(), nowTs.minute(), nowTs.second());
    }

    // ──────────────────────────────────────────────────────────────────────
    // PUB-A  /trend   (replaces /analytics, same 60-s cadence)
    // Stage : multi-resolution buffer + slope + EMA + spike + variance
    // Doc   : 640 B stack
    // ──────────────────────────────────────────────────────────────────────
    {
      StaticJsonDocument<640> t;
      t["plant"]               = PLANT_ID;
      t["machine_id"]          = MACHINE_ID;
      t["sensor_id"]           = SENSOR_ID;
      t["stage"]               = "trend";
      t["execution_location"]  = "edge";

      t["buf_1s"]    = g_buf1sCount;
      t["buf_10s"]   = g_buf10sCount;
      t["buf_60s"]   = g_buf60sCount;
      t["ready_1s"]  = g_trendResult.slope_ready_1s;
      t["ready_10s"] = g_trendResult.slope_ready_10s;
      t["ready_60s"] = g_trendResult.slope_ready_60s;

      if (xSemaphoreTake(mutexAggBufs, pdMS_TO_TICKS(5)) == pdTRUE) {
        if (g_trendResult.slope_ready_1s)
          t["slope_1s"]  = roundf(aggLinRegSlope(g_buf1s,  g_buf1sHead,  g_buf1sCount,
                                                 AGG_BUF_1S_SIZE,  30) * 100000.0f) / 100000.0f;
        if (g_trendResult.slope_ready_10s)
          t["slope_10s"] = roundf(aggLinRegSlope(g_buf10s, g_buf10sHead, g_buf10sCount,
                                                 AGG_BUF_10S_SIZE, 30) * 100000.0f) / 100000.0f;
        if (g_trendResult.slope_ready_60s)
          t["slope_60s"] = roundf(aggLinRegSlope(g_buf60s, g_buf60sHead, g_buf60sCount,
                                                 AGG_BUF_60S_SIZE, 30) * 100000.0f) / 100000.0f;

        float maxRms10m = 0.0f;
        uint16_t n10m = (g_buf10sCount < AGG_BUF_10S_SIZE) ? g_buf10sCount : AGG_BUF_10S_SIZE;
        for (uint16_t i = 0; i < n10m; i++)
          maxRms10m = max(maxRms10m,
            g_buf10s[(g_buf10sHead + AGG_BUF_10S_SIZE - n10m + i) % AGG_BUF_10S_SIZE].max_rms);
        t["max_rms_10min"] = roundf(maxRms10m * 100.0f) / 100.0f;

        float maxRms60m = 0.0f;
        uint16_t n60m = (g_buf60sCount < AGG_BUF_60S_SIZE) ? g_buf60sCount : AGG_BUF_60S_SIZE;
        for (uint16_t i = 0; i < n60m; i++)
          maxRms60m = max(maxRms60m,
            g_buf60s[(g_buf60sHead + AGG_BUF_60S_SIZE - n60m + i) % AGG_BUF_60S_SIZE].max_rms);
        if (n60m > 0) t["max_rms_60min"] = roundf(maxRms60m * 100.0f) / 100.0f;

        float sumSd = 0.0f;
        uint16_t n60s = (g_buf1sCount < AGG_BUF_1S_SIZE) ? g_buf1sCount : AGG_BUF_1S_SIZE;
        for (uint16_t i = 0; i < n60s; i++)
          sumSd += g_buf1s[(g_buf1sHead + AGG_BUF_1S_SIZE - n60s + i) % AGG_BUF_1S_SIZE].stddev_rms;
        t["stddev_1min"] = (n60s > 0) ? roundf((sumSd / n60s) * 1000.0f) / 1000.0f : 0.0f;

        // OSG + FVRI inputs — patent-relevant
        t["slope_var_1s"]  = (float)g_slopeVar_1s;
        t["slope_var_10s"] = (float)g_slopeVar_10s;
        t["slope_var_60s"] = (float)g_slopeVar_60s;

        xSemaphoreGive(mutexAggBufs);
      }

      t["ema_rms"]   = roundf(g_emaRms   * 1000.0f)  / 1000.0f;
      t["ema_dir"]   = g_emaDir;
      t["ema_delta"] = roundf(g_emaDelta * 100000.0f) / 100000.0f;
      t["spike_count"]  = g_trendResult.spike_count;
      // v16.0: suppress freq fields เมื่อ motor ไม่ใช่ RUNNING หรือ RPM < gate
      {
        bool trendFreqGate = (g_vibData.motor_state == (uint8_t)MOTOR_RUNNING &&
                              g_vibData.rpm >= (float)RPM_FREQ_GATE);
        t["freq_alert"]   = trendFreqGate && g_trendResult.freq_alert;
        t["freq_drift_x"] = trendFreqGate ? g_trendResult.freq_drift_x : 0.0f;
        t["freq_drift_y"] = trendFreqGate ? g_trendResult.freq_drift_y : 0.0f;
        t["freq_drift_z"] = trendFreqGate ? g_trendResult.freq_drift_z : 0.0f;
      }
      // Patent Claim 2: expose adaptive slot duration for external verification
      t["slot_dur_ms"]       = (uint32_t)g_slotDur1sMs;
      t["slot_revs_target"]  = SLOT_REVS_TARGET;
      t["timestamp"] = tsA;

      // buf[1024] >> StaticJsonDocument<640>  (v14.3: was buf[700])
      char buf[1024];
      size_t sz = serializeJson(t, buf, sizeof(buf));
      if (sz == 0 || sz >= sizeof(buf) - 1)
        Serial.printf("[WARN] /trend JSON truncated! sz=%u buf=%u\n",
                      (unsigned)sz, (unsigned)sizeof(buf));
      if (mqttClient.publish(g_mqttTopicTrend, buf, (int)sz, false, MQTT_QOS))
        Serial.printf("[TREND] /trend %u B\n", (unsigned)sz);
      else
        Serial.printf("[TREND] FAILED -> %s\n", g_mqttTopicTrend);
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
    resetPrefs.putUInt("count", g_rebootCount);
    resetPrefs.end();
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

void setup() {
  Serial.begin(115200);
  delay(1000);

  // v15.3: อ่าน reset reason แรกสุด ก่อน peripheral init ใดๆ
  // ต้องอยู่หลัง Serial.begin เพื่อให้ print ได้
  logResetReason();

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
  Serial.println("|  ESP32-S3 VIBRATION MONITOR v12.0 (Phase 5 Fusion AI)  |");
  Serial.println("|        LilyGO T-Vending S3 + SIMCom A7670             |");
  Serial.println("+========================================================+\n");

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

  Serial.println("[Init] MQTT Pipeline Topics:");
  Serial.printf("  /vibration (compat): %s\n", g_mqttTopic);
  Serial.printf("  /sensor:             %s\n", g_mqttTopicSensor);
  Serial.printf("  /decision:           %s\n", g_mqttTopicDecision);
  Serial.printf("  /trend:              %s\n", g_mqttTopicTrend);
  Serial.printf("  /event:              %s\n", g_mqttTopicEvent);            // V14.4
  Serial.printf("[Init] Identity: plant=%s  machine=%s  sensor=%s  rated_rpm=%d\n",
                PLANT_ID, MACHINE_ID, SENSOR_ID, RATED_RPM);
  // ─────────────────────────────────────────────────────────────────────────────

  // Initialize GPIO
  pinMode(PIN_BUTTON, INPUT_PULLUP);
  pinMode(PIN_BUTTON_ENTER, INPUT_PULLUP);
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(RS485_EN_PIN, OUTPUT);
  pinMode(BUILTIN_LED, OUTPUT);
  digitalWrite(PIN_BUZZER, LOW);
  digitalWrite(BUILTIN_LED, LOW);
  rs485Enable();

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
    } else if (now.year() < 2024 || now.unixtime() < compiled.unixtime()) {
      // RTC has invalid/old time -- likely battery issue
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
  SerialRS485.begin(MODBUS_BAUDRATE, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);
  modbus.begin(MODBUS_SLAVE_ID, SerialRS485);

  Serial.printf("[Init] Modbus @ %d baud, ID: 0x%02X\n",
                MODBUS_BAUDRATE, MODBUS_SLAVE_ID);

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
  mutexAggBufs    = xSemaphoreCreateMutex();  // Phase 2: guard g_buf1s/10s/60s
  mutexFaultLatch = xSemaphoreCreateMutex();  // v3 hardened: guard g_fl/g_flCount + fault_latch NVS namespace

  if (mutexVibData == NULL || mutexSystemState == NULL ||
      mutexI2C == NULL || mutexModem == NULL || mutexAggBufs == NULL ||
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

  if (queueSensorData == NULL || queueButtonEvent == NULL || queueDisplayUpdate == NULL
      || queueMaintEvent == NULL) {
    Serial.println("[FATAL] Failed to create queues!");
    while (1) delay(1000);
  }

  Serial.println("[Init] Queues created");

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
  Serial.printf("  APN:       %s\n", APN);
  Serial.printf("  MQTT:      %s:%d (mTLS)\n", MQTT_SERVER, MQTT_PORT);
  Serial.printf("  Client ID: %s\n", MQTT_CLIENT_ID);
  Serial.printf("  QoS:       %d\n", MQTT_QOS);
  Serial.printf("  Topic:     %s\n\n", g_mqttTopic);

  Serial.println("Vibration Thresholds:");
  Serial.printf("  Baseline: %.1f mm/s\n", BASELINE_RMS);
  Serial.printf("  Warning:  %.1f mm/s\n", WARNING_RMS);
  Serial.printf("  Critical: %.1f mm/s\n\n", CRITICAL_RMS);

  Serial.println("Adaptive Sending:");
  Serial.println("  NORMAL:   30 seconds");
  Serial.println("  WARNING:  10 seconds");
  Serial.println("  CRITICAL: 5 seconds\n");

  Serial.println("Phase 2 Analytics:");
  Serial.println("  taskAnalytics: Core 1, 1 Hz, Priority 3");
  Serial.println("  Buffers: g_buf1s[60]=60s  g_buf10s[60]=10min  g_buf60s[60]=60min");
  Serial.printf("  RAM overhead: ~%d bytes  (~%.1f KB)\n",
                (int)(sizeof(g_buf1s) + sizeof(g_buf10s) + sizeof(g_buf60s)),
                (sizeof(g_buf1s) + sizeof(g_buf10s) + sizeof(g_buf60s)) / 1024.0f);
  Serial.printf("  Analytics topic: %s\n\n", g_mqttAnalyticsTopic);


  Serial.println("NTP Time Sync:");
  Serial.printf("  Sync Interval:  %lu hours\n", NTP_SYNC_INTERVAL / 3600000UL);
  Serial.printf("  Retry Interval: %lu minutes\n", NTP_SYNC_RETRY_INTERVAL / 60000UL);
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

  if (now - lastStats >= 30000) {  // Every 30 seconds
    lastStats = now;

    Serial.println("\n+========================================================+");
    Serial.println("|              SYSTEM STATUS REPORT (4G mTLS)            |");
    Serial.println("+========================================================+");

    // Get current data
    VibrationData_t localVib;
    MachineState_t localState;

    if (xSemaphoreTake(mutexVibData, pdMS_TO_TICKS(10)) == pdTRUE) {
      memcpy(&localVib, &g_vibData, sizeof(VibrationData_t));
      xSemaphoreGive(mutexVibData);
    }

    if (xSemaphoreTake(mutexSystemState, pdMS_TO_TICKS(10)) == pdTRUE) {
      localState = g_systemState.state;
      xSemaphoreGive(mutexSystemState);
    }

    Serial.printf("| Machine: %-45s |\n", MACHINE_NAME);

    const char* stateStr = "UNKNOWN";
    if (localState == STATE_NORMAL) stateStr = "NORMAL";
    else if (localState == STATE_WARNING) stateStr = "WARNING";
    else if (localState == STATE_CRITICAL) stateStr = "CRITICAL";
    else if (localState == STATE_MAINTENANCE) stateStr = "MAINTENANCE";
    Serial.printf("| State:   %-45s |\n", stateStr);

    Serial.printf("| RMS:     %-42.2f mm/s |\n", localVib.rms_overall);
    Serial.printf("| Temp:    %-43.1f  degC |\n", localVib.temperature);

    Serial.println("+========================================================+");
    Serial.printf("| Modem:   %-45s |\n", g_network.modemReady ? "READY" : "NOT READY");
    Serial.printf("| GPRS:    %-45s |\n", g_network.gprsConnected ? "CONNECTED" : "DISCONNECTED");
    Serial.printf("| MQTT:    %-45s |\n", mqttClient.connected() ? "CONNECTED (mTLS)" : "DISCONNECTED");
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
      DateTime rtcNow = rtc.now();
      Serial.printf("| RTC Time: %04d-%02d-%02d %02d:%02d:%02d                    |\n",
                    rtcNow.year(), rtcNow.month(), rtcNow.day(),
                    rtcNow.hour(), rtcNow.minute(), rtcNow.second());
    }

    // NTP Time Sync status
    Serial.println("+========================================================+");
    Serial.printf("| NTP Synced:  %-41s |\n", g_timeSync.synced ? "YES +" : "NO x");
    if (g_timeSync.synced) {
      uint32_t ageSec = (millis() - g_timeSync.lastSyncMillis) / 1000;
      Serial.printf("| Last Sync:   %-41s |\n", g_timeSync.lastSyncTime);
      Serial.printf("| Sync Age:    %lu seconds ago                        |\n", ageSec);
      Serial.printf("| Last Drift:  %+d seconds                            |\n", g_timeSync.lastDriftSec);
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

    // Phase 2: Analytics buffer fill status
    Serial.printf("Analytics Buffer Fill: buf1s=%u/60  buf10s=%u/60  buf60s=%u/60\n",
                  g_buf1sCount, g_buf10sCount, g_buf60sCount);
    Serial.printf("EMA: rms=%.3f delta=%.5f dir=%+d\n",
                  g_emaRms, g_emaDelta, (int)g_emaDir);
  }

  // This loop runs at low priority on Core 1
  // Main work is done by FreeRTOS tasks
  vTaskDelay(pdMS_TO_TICKS(1000));  // Sleep for 1 second
}
