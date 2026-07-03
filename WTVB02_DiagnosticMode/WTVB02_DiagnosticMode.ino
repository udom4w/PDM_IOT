// ============================================================================
// WTVB02_DiagnosticMode.ino
// ----------------------------------------------------------------------------
// PURPOSE
//   Prove or disprove whether physically impossible values (RMS > Peak,
//   negative Peak, wildly wrong Peak/RMS ratios) are caused by the WTVB02
//   sensor updating its internal registers asynchronously between the
//   sequential Modbus RTU reads that are required because the Amplitude
//   and RMS registers are non-contiguous.
//
// STRATEGY
//   Every sample cycle executes exactly 4 Modbus transactions in order:
//     T1 → read 0x3A-0x3C  (VX / VY / VZ  – Velocity Amplitude / Peak)
//     T2 → read 0x0050      (VRMSX)
//     T3 → read 0x005C      (VRMSY)
//     T4 → read 0x0068      (VRMSZ)
//
//   For each transaction the sketch records:
//     • millis() before the call  (timestamp)
//     • millis() after the call   (duration = after − before)
//     • the raw uint16 register value(s) BEFORE any scaling
//
//   After all four transactions, consistency checks are applied per-axis.
//   A running anomaly counter accumulates across all samples.
//
// CONSISTENCY RULES (per axis)
//   Flag INCONSISTENT if ANY of the following are true:
//     1. raw Peak register ≥ 0x8000  → negative when cast to int16  (sign wrap)
//     2. Peak (scaled) <= 0
//     3. RMS  (scaled) >  Peak (scaled)
//     4. Peak/RMS < RATIO_MIN  (0.5)
//     5. Peak/RMS > RATIO_MAX  (5.0)
//
// Hardware  : LilyGO T-Vending S3 (ESP32-S3)
// Library   : ModbusMaster
// ============================================================================

#include <ModbusMaster.h>

// ── Pin definitions (LilyGO T-Vending S3) ────────────────────────────────────
#define RS485_TX_PIN    39
#define RS485_RX_PIN    38
#define RS485_EN_PIN    42

// ── Modbus settings ───────────────────────────────────────────────────────────
#define MODBUS_SLAVE_ID   0x50
#define MODBUS_BAUDRATE   9600

// ── Register addresses ───────────────────────────────────────────────────────
// Velocity Amplitude (Peak)  – 3 contiguous registers
//   Scaling: (int16_t)raw / 100.0 → mm/s
#define REG_VX      0x003A
// REG_VY = 0x003B  (read as offset 1 in the same transaction)
// REG_VZ = 0x003C  (read as offset 2 in the same transaction)

// Velocity RMS – non-contiguous, each read separately
//   Scaling: raw / 1000.0 → mm/s
#define REG_VRMSX   0x0050
#define REG_VRMSY   0x005C
#define REG_VRMSZ   0x0068

// ── Diagnostic thresholds ─────────────────────────────────────────────────────
#define RATIO_MIN   0.5f    // Peak/RMS below this → anomaly
#define RATIO_MAX   5.0f    // Peak/RMS above this → anomaly

// ── Sample interval ───────────────────────────────────────────────────────────
#define SAMPLE_INTERVAL_MS  2000UL

// ── Peripheral objects ────────────────────────────────────────────────────────
HardwareSerial SerialRS485(2);
ModbusMaster   modbus;

// ─────────────────────────────────────────────────────────────────────────────
// Data structures
// ─────────────────────────────────────────────────────────────────────────────

// One Modbus transaction's timing metadata
struct TxTiming {
  unsigned long tsStart;   // millis() just before readHoldingRegisters()
  unsigned long tsEnd;     // millis() just after  readHoldingRegisters()
  uint32_t      duration;  // tsEnd - tsStart  (ms)
};

// Complete data for one diagnostic sample
struct DiagSample {
  uint32_t sampleNum;

  // ── Raw register values (uint16, straight from response buffer) ────────────
  uint16_t rawVX;
  uint16_t rawVY;
  uint16_t rawVZ;
  uint16_t rawVrmsX;
  uint16_t rawVrmsY;
  uint16_t rawVrmsZ;

  // ── Scaled physical values ─────────────────────────────────────────────────
  float vx;      // mm/s  (int16 cast / 100.0)
  float vy;
  float vz;
  float vrmsx;   // mm/s  (raw / 1000.0)
  float vrmsy;
  float vrmsz;

  // ── Per-axis Peak/RMS ratios ───────────────────────────────────────────────
  float ratioX;   // vx   / vrmsx
  float ratioY;   // vy   / vrmsy
  float ratioZ;   // vz   / vrmsz

  // ── Timing for each of the 4 transactions ─────────────────────────────────
  TxTiming t1;   // VX/VY/VZ
  TxTiming t2;   // VRMSX
  TxTiming t3;   // VRMSY
  TxTiming t4;   // VRMSZ

  // ── Per-axis consistency flags ─────────────────────────────────────────────
  bool inconsistentX;
  bool inconsistentY;
  bool inconsistentZ;
  bool anySampleBad;   // true if any axis is flagged
};

// ── Global anomaly counters ───────────────────────────────────────────────────
static uint32_t g_totalSamples = 0;
static uint32_t g_badSamples   = 0;

// ─────────────────────────────────────────────────────────────────────────────
// rs485Enable()
// Hold DE/RE LOW permanently – no pre/post-transmission callbacks needed.
// ─────────────────────────────────────────────────────────────────────────────
void rs485Enable() {
  digitalWrite(RS485_EN_PIN, LOW);
}

// ─────────────────────────────────────────────────────────────────────────────
// safeRatio()
// Returns peak/rms, or 0 if either value would produce a nonsensical result.
// ─────────────────────────────────────────────────────────────────────────────
static float safeRatio(float peak, float rms) {
  if (rms  <= 0.0f) return 0.0f;
  if (peak <= 0.0f) return 0.0f;
  return peak / rms;
}

// ─────────────────────────────────────────────────────────────────────────────
// isAxisInconsistent()
// Applies all five consistency rules to one axis.
// Returns true when ANY rule fires (= anomaly detected on this axis).
// ─────────────────────────────────────────────────────────────────────────────
static bool isAxisInconsistent(float peak, float rms, float ratio) {
  // Rule 1: sign-wrap in the raw 16-bit register → negative amplitude
  if (peak <= 0.0f)  return true;
  // Rule 2: RMS cannot exceed Peak for any real-world sinusoidal signal
  if (rms  >  peak)  return true;
  // Rules 3 & 4: ratio band check
  if (ratio < RATIO_MIN) return true;
  if (ratio > RATIO_MAX) return true;
  return false;
}

// ─────────────────────────────────────────────────────────────────────────────
// runDiagnosticSample()
// Executes 4 Modbus transactions, timestamps each one, validates consistency,
// and fills a DiagSample struct.
//
// Returns true  when all 4 transactions succeed (data may still be flagged
//               as inconsistent – that is a sensor-side phenomenon, not a
//               communication error).
// Returns false when any Modbus transaction fails (CRC error, timeout, etc.).
// ─────────────────────────────────────────────────────────────────────────────
bool runDiagnosticSample(DiagSample &s) {
  uint8_t result;

  s = DiagSample{};          // zero-initialise

  // ── Transaction T1 : Velocity Amplitude VX / VY / VZ (0x3A, count=3) ──────
  s.t1.tsStart = millis();
  result        = modbus.readHoldingRegisters(REG_VX, 3);
  s.t1.tsEnd    = millis();
  s.t1.duration = s.t1.tsEnd - s.t1.tsStart;

  if (result != modbus.ku8MBSuccess) {
    Serial.printf("[ERROR] T1 VX/VY/VZ failed (Modbus code 0x%02X)\n", result);
    return false;
  }
  s.rawVX = modbus.getResponseBuffer(0);
  s.rawVY = modbus.getResponseBuffer(1);
  s.rawVZ = modbus.getResponseBuffer(2);
  // Cast to signed int16 before dividing to preserve sign for negative values
  s.vx = (int16_t)s.rawVX / 100.0f;
  s.vy = (int16_t)s.rawVY / 100.0f;
  s.vz = (int16_t)s.rawVZ / 100.0f;

  delay(20);  // inter-frame silent gap (> 3.5 char times at 9600 baud ≈ 4 ms)

  // ── Transaction T2 : VRMSX (0x50, count=1) ───────────────────────────────
  s.t2.tsStart = millis();
  result        = modbus.readHoldingRegisters(REG_VRMSX, 1);
  s.t2.tsEnd    = millis();
  s.t2.duration = s.t2.tsEnd - s.t2.tsStart;

  if (result != modbus.ku8MBSuccess) {
    Serial.printf("[ERROR] T2 VRMSX failed (Modbus code 0x%02X)\n", result);
    return false;
  }
  s.rawVrmsX = modbus.getResponseBuffer(0);
  s.vrmsx    = s.rawVrmsX / 1000.0f;

  delay(20);

  // ── Transaction T3 : VRMSY (0x5C, count=1) ───────────────────────────────
  s.t3.tsStart = millis();
  result        = modbus.readHoldingRegisters(REG_VRMSY, 1);
  s.t3.tsEnd    = millis();
  s.t3.duration = s.t3.tsEnd - s.t3.tsStart;

  if (result != modbus.ku8MBSuccess) {
    Serial.printf("[ERROR] T3 VRMSY failed (Modbus code 0x%02X)\n", result);
    return false;
  }
  s.rawVrmsY = modbus.getResponseBuffer(0);
  s.vrmsy    = s.rawVrmsY / 1000.0f;

  delay(20);

  // ── Transaction T4 : VRMSZ (0x68, count=1) ───────────────────────────────
  s.t4.tsStart = millis();
  result        = modbus.readHoldingRegisters(REG_VRMSZ, 1);
  s.t4.tsEnd    = millis();
  s.t4.duration = s.t4.tsEnd - s.t4.tsStart;

  if (result != modbus.ku8MBSuccess) {
    Serial.printf("[ERROR] T4 VRMSZ failed (Modbus code 0x%02X)\n", result);
    return false;
  }
  s.rawVrmsZ = modbus.getResponseBuffer(0);
  s.vrmsz    = s.rawVrmsZ / 1000.0f;

  // ── Compute ratios ────────────────────────────────────────────────────────
  s.ratioX = safeRatio(s.vx,  s.vrmsx);
  s.ratioY = safeRatio(s.vy,  s.vrmsy);
  s.ratioZ = safeRatio(s.vz,  s.vrmsz);

  // ── Consistency check per axis ────────────────────────────────────────────
  s.inconsistentX = isAxisInconsistent(s.vx, s.vrmsx, s.ratioX);
  s.inconsistentY = isAxisInconsistent(s.vy, s.vrmsy, s.ratioY);
  s.inconsistentZ = isAxisInconsistent(s.vz, s.vrmsz, s.ratioZ);
  s.anySampleBad  = s.inconsistentX || s.inconsistentY || s.inconsistentZ;

  return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// printDiagnosticReport()
// Formats the full diagnostic output for one sample.
// ─────────────────────────────────────────────────────────────────────────────
void printDiagnosticReport(const DiagSample &s) {
  Serial.println();
  Serial.println("=================================================================");
  Serial.printf ("  Sample #%u\n", s.sampleNum);
  Serial.println("-----------------------------------------------------------------");

  // ── Transaction timing & raw values ──────────────────────────────────────
  Serial.printf("[%lu ms] Read VX/VY/VZ      Duration = %lu ms\n",
                s.t1.tsStart, s.t1.duration);
  Serial.printf("         VX RAW = %-6u (0x%04X)  →  VX = %.3f mm/s\n",
                s.rawVX, s.rawVX, s.vx);
  Serial.printf("         VY RAW = %-6u (0x%04X)  →  VY = %.3f mm/s\n",
                s.rawVY, s.rawVY, s.vy);
  Serial.printf("         VZ RAW = %-6u (0x%04X)  →  VZ = %.3f mm/s\n",
                s.rawVZ, s.rawVZ, s.vz);

  Serial.printf("[%lu ms] Read VRMSX          Duration = %lu ms\n",
                s.t2.tsStart, s.t2.duration);
  Serial.printf("         VRMSX RAW = %-6u (0x%04X)  →  VRMSX = %.3f mm/s\n",
                s.rawVrmsX, s.rawVrmsX, s.vrmsx);

  Serial.printf("[%lu ms] Read VRMSY          Duration = %lu ms\n",
                s.t3.tsStart, s.t3.duration);
  Serial.printf("         VRMSY RAW = %-6u (0x%04X)  →  VRMSY = %.3f mm/s\n",
                s.rawVrmsY, s.rawVrmsY, s.vrmsy);

  Serial.printf("[%lu ms] Read VRMSZ          Duration = %lu ms\n",
                s.t4.tsStart, s.t4.duration);
  Serial.printf("         VRMSZ RAW = %-6u (0x%04X)  →  VRMSZ = %.3f mm/s\n",
                s.rawVrmsZ, s.rawVrmsZ, s.vrmsz);

  // ── Time span across the whole sample ────────────────────────────────────
  uint32_t sampleSpan = s.t4.tsEnd - s.t1.tsStart;
  Serial.printf("         Total sample window = %u ms  "
                "(T1 start → T4 end)\n", sampleSpan);

  Serial.println("-----------------------------------------------------------------");

  // ── Ratio table ───────────────────────────────────────────────────────────
  Serial.printf("  Peak/RMS X = %.3f  %s\n",
                s.ratioX, s.inconsistentX ? "<-- AXIS FLAG" : "");
  Serial.printf("  Peak/RMS Y = %.3f  %s\n",
                s.ratioY, s.inconsistentY ? "<-- AXIS FLAG" : "");
  Serial.printf("  Peak/RMS Z = %.3f  %s\n",
                s.ratioZ, s.inconsistentZ ? "<-- AXIS FLAG" : "");
  Serial.println("  Reference  Peak/RMS ~= 1.414  (pure sine wave)");

  Serial.println("-----------------------------------------------------------------");

  // ── Consistency verdict ───────────────────────────────────────────────────
  if (s.anySampleBad) {
    Serial.println("  *** INCONSISTENT SAMPLE DETECTED ***");
    Serial.println("  Possible cause: asynchronous sensor register update");
    Serial.println("  between sequential Modbus reads.");

    // Axis-level detail for the anomaly report
    if (s.inconsistentX) {
      Serial.printf("  [X] Peak=%.3f  VRMSX=%.3f  Ratio=%.3f\n",
                    s.vx, s.vrmsx, s.ratioX);
    }
    if (s.inconsistentY) {
      Serial.printf("  [Y] Peak=%.3f  VRMSY=%.3f  Ratio=%.3f\n",
                    s.vy, s.vrmsy, s.ratioY);
    }
    if (s.inconsistentZ) {
      Serial.printf("  [Z] Peak=%.3f  VRMSZ=%.3f  Ratio=%.3f\n",
                    s.vz, s.vrmsz, s.ratioZ);
    }
  } else {
    Serial.println("  Status = CONSISTENT");
  }

  Serial.println("-----------------------------------------------------------------");

  // ── Running anomaly statistics ────────────────────────────────────────────
  float badPct = (g_totalSamples > 0)
               ? (100.0f * (float)g_badSamples / (float)g_totalSamples)
               : 0.0f;
  Serial.printf("  Anomaly stats: Total=%u  Bad=%u  Bad%%=%.1f%%\n",
                g_totalSamples, g_badSamples, badPct);

  Serial.println("=================================================================");
}

// ─────────────────────────────────────────────────────────────────────────────
// waitForSensor()
// Blocks until at least one successful read cycle completes.
// Called once from setup() and can be called again after a run of failures.
// ─────────────────────────────────────────────────────────────────────────────
void waitForSensor() {
  Serial.println("[INIT] Waiting for sensor...");
  DiagSample probe;
  probe.sampleNum = 0;
  while (!runDiagnosticSample(probe)) {
    Serial.println("[INIT] No response – retrying in 3 s");
    delay(3000);
  }
  Serial.println("[INIT] Sensor OK.\n");
}

// ─────────────────────────────────────────────────────────────────────────────
// setup()
// ─────────────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(2000);

  Serial.println("=================================================================");
  Serial.println(" WTVB02 Diagnostic Mode  –  Reading Inconsistency Test");
  Serial.println(" Hardware : LilyGO T-Vending S3 (ESP32-S3)");
  Serial.println(" RS485    : EN=LOW (always enabled, no callbacks)");
  Serial.println("=================================================================");
  Serial.println(" OBJECTIVE");
  Serial.println("   Detect whether the sensor updates its registers mid-cycle,");
  Serial.println("   causing physically impossible Peak / RMS combinations.");
  Serial.println(" CONSISTENCY RULES  (any triggers = INCONSISTENT flag)");
  Serial.printf ("   1. Peak <= 0 mm/s  (sign-wrap or sensor glitch)\n");
  Serial.printf ("   2. RMS  >  Peak    (physically impossible)\n");
  Serial.printf ("   3. Peak/RMS < %.1f  (too low for any real signal)\n", RATIO_MIN);
  Serial.printf ("   4. Peak/RMS > %.1f  (too high for any real signal)\n", RATIO_MAX);
  Serial.println("=================================================================\n");

  // RS485 transceiver: DE/RE held LOW permanently
  pinMode(RS485_EN_PIN, OUTPUT);
  rs485Enable();

  // UART for RS485
  SerialRS485.begin(MODBUS_BAUDRATE, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);

  // Modbus without pre/post-transmission callbacks
  modbus.begin(MODBUS_SLAVE_ID, SerialRS485);

  delay(1000);

  waitForSensor();
}

// ─────────────────────────────────────────────────────────────────────────────
// loop()
// ─────────────────────────────────────────────────────────────────────────────
void loop() {
  static unsigned long lastSample = 0;

  if (millis() - lastSample < SAMPLE_INTERVAL_MS) {
    return;
  }
  lastSample = millis();

  g_totalSamples++;

  DiagSample sample;
  sample.sampleNum = g_totalSamples;

  if (!runDiagnosticSample(sample)) {
    // Modbus communication failure – not an inconsistency, but still notable
    Serial.printf("\n[WARN] Sample #%u – Modbus error. "
                  "Check wiring / power. Retrying...\n", g_totalSamples);
    // Don't count as a bad sample (it's a comm failure, not a data anomaly)
    g_totalSamples--;   // undo increment so stats stay meaningful
    delay(1000);
    return;
  }

  if (sample.anySampleBad) {
    g_badSamples++;
  }

  printDiagnosticReport(sample);
}
