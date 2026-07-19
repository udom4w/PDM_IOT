// ============================================================================
// WTVB02_VelocityComparison.ino
// ----------------------------------------------------------------------------
// Reads Velocity Amplitude (Peak) registers, Velocity RMS registers, and
// Vibration Frequency registers from the vibration sensor via RS485
// Modbus RTU, then prints a side-by-side comparison every READ_INTERVAL_MS.
//
// [v1.3] HARDWARE IDENTITY NOTE — confirmed via Witmotion PC software screen
// (Sensor Configuration panel, Device ID 57e5cc9e-cf4b), NOT via Modbus:
// the physical production sensor ("pump01") is a WTVB05, not a WTVB02-485,
// despite this file's original name/comments. VX/VY/VZ, VRMSX/Y/Z, DX/DY/DZ,
// HZX/Y/Z, and ERRX/Y/Z registers are byte-identical between the WTVB02-485
// and WTVB05 datasheets, so those addresses below remain valid either way.
// REG_MODE (0x07) and REG_STATICDETECTION (0x08) are documented ONLY in the
// WTVB05 datasheet (not present in the WTVB02-485 register table at all) —
// which is consistent with the confirmed WTVB05 identity.
//
// Hardware : LilyGO T-Vending S3  (ESP32-S3)
// Library  : ModbusMaster
// ============================================================================

#include <ModbusMaster.h>
#include <string.h>   // memcpy (used in computeP95)

// Forward declaration — REQUIRED.
// The Arduino IDE auto-generates function prototypes and inserts them
// right after this include block, BEFORE the full "struct VelocityData"
// definition below. Without this forward declaration, the auto-generated
// prototype for readVelocityComparison()/printVelocityReport() would
// reference an unknown type and fail to compile.
struct VelocityData;

// ── Pin definitions (LilyGO T-Vending S3) ────────────────────────────────────
#define RS485_TX_PIN    39
#define RS485_RX_PIN    38
#define RS485_EN_PIN    42

// ── Modbus settings ───────────────────────────────────────────────────────────
#define MODBUS_SLAVE_ID  0x50
#define MODBUS_BAUDRATE  9600

// ── Register map ─────────────────────────────────────────────────────────────
// Velocity Amplitude (Peak)   scale: raw / 100.0  → mm/s
#define REG_VX      0x003A
#define REG_VY      0x003B
#define REG_VZ      0x003C

// Velocity RMS                scale: raw / 1000.0 → mm/s
#define REG_VRMSX   0x0050
#define REG_VRMSY   0x005C
#define REG_VRMSZ   0x0068

// Vibration Frequency         scale: raw / 10.0 → Hz  (per datasheet: reg 0x44–0x46)
#define REG_HZX     0x0044
#define REG_HZY     0x0045
#define REG_HZZ     0x0046

// Config / mode-switch registers
#define REG_SR         0x0029   // Sample rate
#define REG_MODE       0x0007   // Output mode (0x00 LowFreq / 0x01 HighFreq-CF / 0x02 FreqDomain)
#define REG_STATICDETECTION 0x0008 // [v1.3] Static-detection coefficient (WTVB05 only, range 80~120, default 100 = 1.00x)
                                    // "After the device is stationary, there is still vibration data.
                                    //  The speed at which the data is cleared has no unit meaning and
                                    //  is a coefficient." — WTVB05 datasheet 6.1.4.5. Left at implicit
                                    // default (100) by prior firmware versions; now written explicitly
                                    // at boot so behavior is deterministic and tunable from one place.
#define REG_TEMP       0x0040   // Temperature, scale: raw / 100.0 → °C
#define REG_UNLOCK     0x0069   // Write-protect unlock register
#define REG_SAVE_REBOOT 0x0000  // Save (0x0000) / Reboot (0x00FF) command register

// [v1.3] REG_VERSION — REMOVED. The previous candidate address (0x002E) was
// tested and confirmed WRONG (returned implausible/garbage values). A search
// of BOTH the WTVB02-485 datasheet (register table, section 6.3) and the
// WTVB05 datasheet (register table, section 6.1.3) found NO documented
// firmware-version register anywhere in either device's standard Modbus
// holding-register map (0x00–0x9A / 0x00–0x9A respectively). Firmware
// version is only obtainable via the Witmotion PC software's Sensor
// Configuration panel ("Version: 10211.1.24" field) — that field is not
// sourced from a plain Modbus read at a fixed address per the vendor docs.
// Do NOT re-introduce a guessed REG_VERSION address; see readFirmwareVersion()
// below for what this firmware does instead.

#define UNLOCK_KEY       0xB588
#define SAVE_CMD         0x0000
#define REBOOT_CMD       0x00FF
#define MODE_FREQDOMAIN  0x0002 // Target mode: FreqDomain (needed for valid CF values)
#define STATICDETECTION_DEFAULT 100 // 100/100 = 1.00x, vendor default. Range 80~120.
                                     // TODO: tune against pump01 field data once baseline
                                     // false-WARNING correlation with this coefficient is confirmed.

// ── Read interval ─────────────────────────────────────────────────────────────
// Estimated total transaction time: ~150–250 ms @9600 baud
//   (5 Modbus transactions: Peak, VRMSX, VRMSY, VRMSZ, Freq
//    + delay(20 ms) between each → delay alone accounts for ~80–100 ms)
// NOT YET MEASURED — verify with [TIMING] log below before trusting this range
// or reducing READ_INTERVAL_MS further.
#define READ_INTERVAL_MS  500

// ── Peripheral objects ────────────────────────────────────────────────────────
HardwareSerial SerialRS485(2);
ModbusMaster   modbus;

// ── Read-cycle timing statistics (for validating READ_INTERVAL_MS assumption) ──
struct TimingStats {
  uint32_t count;
  uint32_t minMs;
  uint32_t maxMs;
  uint64_t sumMs;      // for average
  uint32_t samples[200]; // ring buffer for P95 (~ enough for several minutes at 500ms)
  uint16_t sampleIdx;
  uint16_t sampleCount;
};
TimingStats timingStats = {0, UINT32_MAX, 0, 0, {0}, 0, 0};

void recordTiming(uint32_t elapsedMs) {
  timingStats.count++;
  timingStats.sumMs += elapsedMs;
  if (elapsedMs < timingStats.minMs) timingStats.minMs = elapsedMs;
  if (elapsedMs > timingStats.maxMs) timingStats.maxMs = elapsedMs;

  timingStats.samples[timingStats.sampleIdx] = elapsedMs;
  timingStats.sampleIdx = (timingStats.sampleIdx + 1) % 200;
  if (timingStats.sampleCount < 200) timingStats.sampleCount++;
}

// Simple insertion-sort P95 (sampleCount <= 200, runs infrequently so O(n^2) is fine)
uint32_t computeP95() {
  if (timingStats.sampleCount == 0) return 0;
  uint32_t sorted[200];
  memcpy(sorted, timingStats.samples, timingStats.sampleCount * sizeof(uint32_t));
  for (uint16_t i = 1; i < timingStats.sampleCount; i++) {
    uint32_t key = sorted[i];
    int16_t j = i - 1;
    while (j >= 0 && sorted[j] > key) {
      sorted[j + 1] = sorted[j];
      j--;
    }
    sorted[j + 1] = key;
  }
  uint16_t idx = (uint16_t)(0.95f * (timingStats.sampleCount - 1));
  return sorted[idx];
}

void printTimingStats() {
  if (timingStats.count == 0) return;
  float avg = (float)timingStats.sumMs / timingStats.count;
  Serial.printf(
    "[TIMING] n=%lu  Min=%lu ms  Avg=%.1f ms  P95=%lu ms  Max=%lu ms\n",
    timingStats.count, timingStats.minMs, avg, computeP95(), timingStats.maxMs
  );
}

// ── Data structure ────────────────────────────────────────────────────────────
struct VelocityData {
  // Velocity Amplitude (Peak), mm/s
  float    vx;
  float    vy;
  float    vz;

  // Velocity RMS, mm/s
  float    vrmsx;
  float    vrmsy;
  float    vrmsz;

  // Raw RMS register values (before scaling)
  uint16_t rawVrmsX;
  uint16_t rawVrmsY;
  uint16_t rawVrmsZ;

  // Vibration Frequency, Hz
  float    hzx;
  float    hzy;
  float    hzz;
};

// ─────────────────────────────────────────────────────────────────────────────
// rs485Enable()
// Keep DE/RE line asserted LOW → transmit/receive always active (no callback).
// ─────────────────────────────────────────────────────────────────────────────
void rs485Enable() {
  digitalWrite(RS485_EN_PIN, LOW);
}

// ─────────────────────────────────────────────────────────────────────────────
// readFirmwareVersion()
// [v1.3] NO LONGER attempts a Modbus register read. The previous candidate
// (REG_VERSION = 0x002E) was tested on-device and returned implausible
// values — confirmed WRONG, not just unverified. A subsequent check of both
// the WTVB02-485 and WTVB05 datasheets' full register address tables found
// no documented VERSION register in either device's Modbus map. The firmware
// version actually in use ("10211.1.24") was obtained from the Witmotion PC
// software's Sensor Configuration panel, not from a register read — that is
// currently the only confirmed way to get this value for this hardware.
//
// This function is kept (rather than deleted) as a single, deliberate place
// to log that fact at boot, so nobody re-adds a guessed register address
// later without seeing this note first.
// ─────────────────────────────────────────────────────────────────────────────
void readFirmwareVersion() {
  Serial.println("\n[VERSION] Firmware version is NOT read via Modbus in this build.");
  Serial.println("  [VERSION] No VERSION register is documented in the WTVB02-485 or WTVB05");
  Serial.println("  [VERSION] register address tables. Confirmed value for this unit (\"pump01\",");
  Serial.println("  [VERSION] Device ID 57e5cc9e-cf4b) is 10211.1.24, read from the Witmotion PC");
  Serial.println("  [VERSION] software Sensor Configuration panel — re-check there if it matters,");
  Serial.println("  [VERSION] do not guess a new register address here.");
}

// ─────────────────────────────────────────────────────────────────────────────
// readConfig()
// Reads and prints the sensor's current SR (sample rate), MODE, and TEMP.
// Read-only — does not change any settings.
// ─────────────────────────────────────────────────────────────────────────────
void readConfig() {
  Serial.println("\n[CONFIG] Current sensor config:");

  // SR
  if (modbus.readHoldingRegisters(REG_SR, 1) == modbus.ku8MBSuccess) {
    uint16_t v = modbus.getResponseBuffer(0);
    const char* name = (v == 0 ? "32K" : v == 1 ? "16K" : v == 2 ? "8K" : v == 7 ? "256Hz(def)" : "unknown");
    Serial.printf("  SR  (0x%02X) = 0x%04X = %s\n", REG_SR, v, name);
  } else {
    Serial.println("  SR  : READ FAILED");
  }
  delay(50);

  // MODE
  if (modbus.readHoldingRegisters(REG_MODE, 1) == modbus.ku8MBSuccess) {
    uint16_t v = modbus.getResponseBuffer(0);
    const char* name = (v == 0 ? "LowFreq(def)" : v == 1 ? "HighFreq(CF)" : v == 2 ? "FreqDomain" : "unknown");
    Serial.printf("  MODE(0x%02X) = 0x%04X = %s\n", REG_MODE, v, name);
  } else {
    Serial.println("  MODE: READ FAILED");
  }
  delay(50);

  // STATICDETECTION [v1.3]
  if (modbus.readHoldingRegisters(REG_STATICDETECTION, 1) == modbus.ku8MBSuccess) {
    uint16_t v = modbus.getResponseBuffer(0);
    Serial.printf("  STATICDETECTION(0x%02X) = %u  (%.2fx)%s\n",
                  REG_STATICDETECTION, v, v / 100.0f,
                  v == STATICDETECTION_DEFAULT ? " [vendor default]" : "");
  } else {
    Serial.println("  STATICDETECTION: READ FAILED");
  }
  delay(50);

  // TEMP
  if (modbus.readHoldingRegisters(REG_TEMP, 1) == modbus.ku8MBSuccess) {
    Serial.printf("  TEMP(0x%02X) = %.1f C\n", REG_TEMP, (int16_t)modbus.getResponseBuffer(0) / 100.0f);
  } else {
    Serial.println("  TEMP: READ FAILED");
  }
  delay(50);
}

// ─────────────────────────────────────────────────────────────────────────────
// configureSensorInit()
// [v1.3] Renamed/extended from setModeFreqDomain(). Forces the sensor into:
//   - MODE = 0x02 (FreqDomain) — only mode observed to produce non-zero
//     Crest Factor / K-factor values on this unit.
//   - STATICDETECTION = STATICDETECTION_DEFAULT — written EXPLICITLY at boot
//     instead of being left implicit, so the coefficient is deterministic
//     and can be tuned in one place (see comment at STATICDETECTION_DEFAULT)
//     while investigating whether stale/residual vibration data at
//     start/stop transitions contributes to false WARNING events.
// Both writes share a single unlock → save → reboot cycle (the unlock is
// valid for 10 s per the datasheet, so both writes fit inside one window).
// Each write is still preceded by its own unlock as a defensive measure,
// since the sensor appears to re-lock write-protect after every command
// (per WTVB02_SensorReset_Test findings).
// Returns true only if the sensor comes back alive AND both MODE and
// STATICDETECTION read back as the values that were written.
// ─────────────────────────────────────────────────────────────────────────────
bool configureSensorInit() {
  uint8_t r;
  Serial.println("\n[SETUP-INIT] Forcing MODE=0x02 (FreqDomain) and STATICDETECTION="
                  + String(STATICDETECTION_DEFAULT) + "...");

  Serial.print("  [Unlock#1]... ");
  r = modbus.writeSingleRegister(REG_UNLOCK, UNLOCK_KEY);
  Serial.println(r == modbus.ku8MBSuccess ? "+ OK" : "x FAIL");
  delay(300);

  Serial.printf("  MODE=0x%02X (FreqDomain)... ", MODE_FREQDOMAIN);
  r = modbus.writeSingleRegister(REG_MODE, MODE_FREQDOMAIN);
  Serial.println(r == modbus.ku8MBSuccess ? "+ OK" : "x FAIL");
  delay(100);

  Serial.print("  [Unlock#2]... ");
  r = modbus.writeSingleRegister(REG_UNLOCK, UNLOCK_KEY);
  Serial.println(r == modbus.ku8MBSuccess ? "+ OK" : "x FAIL");
  delay(300);

  Serial.printf("  STATICDETECTION=%u... ", STATICDETECTION_DEFAULT);
  r = modbus.writeSingleRegister(REG_STATICDETECTION, STATICDETECTION_DEFAULT);
  Serial.println(r == modbus.ku8MBSuccess ? "+ OK" : "x FAIL");
  delay(100);

  Serial.print("  [Unlock#3]... ");
  r = modbus.writeSingleRegister(REG_UNLOCK, UNLOCK_KEY);
  Serial.println(r == modbus.ku8MBSuccess ? "+ OK" : "x FAIL");
  delay(300);

  Serial.print("  Save... ");
  r = modbus.writeSingleRegister(REG_SAVE_REBOOT, SAVE_CMD);
  Serial.println(r == modbus.ku8MBSuccess ? "+ OK" : "x FAIL");
  delay(300);

  Serial.print("  [Unlock#4]... ");
  r = modbus.writeSingleRegister(REG_UNLOCK, UNLOCK_KEY);
  Serial.println(r == modbus.ku8MBSuccess ? "+ OK" : "x FAIL");
  delay(300);

  Serial.print("  Reboot... ");
  r = modbus.writeSingleRegister(REG_SAVE_REBOOT, REBOOT_CMD);
  Serial.printf("%s  waiting 5s...\n", r == modbus.ku8MBSuccess ? "+ OK" : "x FAIL");
  delay(5000);

  // Wait for sensor to come back alive
  bool alive = false;
  for (int i = 0; i < 5 && !alive; i++) {
    if (modbus.readHoldingRegisters(REG_VX, 3) == modbus.ku8MBSuccess) {
      alive = true;
    } else {
      Serial.println("  x not yet, wait 1s...");
      delay(1000);
    }
  }
  if (!alive) {
    Serial.println("  x Sensor not responding after reboot!");
    return false;
  }
  Serial.println("  + Sensor alive after reboot.");

  // Verify MODE actually saved
  bool modeOk = false;
  if (modbus.readHoldingRegisters(REG_MODE, 1) == modbus.ku8MBSuccess) {
    uint16_t v = modbus.getResponseBuffer(0);
    Serial.printf("  MODE verify = 0x%04X (%s)\n", v,
                  v == MODE_FREQDOMAIN ? "OK - FreqDomain confirmed" : "MISMATCH!");
    modeOk = (v == MODE_FREQDOMAIN);
  } else {
    Serial.println("  x MODE verify READ FAILED");
  }
  delay(50);

  // Verify STATICDETECTION actually saved
  bool staticOk = false;
  if (modbus.readHoldingRegisters(REG_STATICDETECTION, 1) == modbus.ku8MBSuccess) {
    uint16_t v = modbus.getResponseBuffer(0);
    Serial.printf("  STATICDETECTION verify = %u (%s)\n", v,
                  v == STATICDETECTION_DEFAULT ? "OK - confirmed" : "MISMATCH!");
    staticOk = (v == STATICDETECTION_DEFAULT);
  } else {
    Serial.println("  x STATICDETECTION verify READ FAILED");
  }

  return modeOk && staticOk;
}

// ─────────────────────────────────────────────────────────────────────────────
// readVelocityComparison()
//
// Performs three Modbus read operations:
//   1. Holding registers 0x3A–0x3C  → Velocity Amplitude X/Y/Z
//   2. Three individual reads for VRMSX (0x50), VRMSY (0x5C), VRMSZ (0x68)
//      (registers are non-contiguous, so each is read separately)
//   3. Holding registers 0x44–0x46  → Vibration Frequency X/Y/Z
//
// Returns true  if all reads succeed and data is valid.
// Returns false if any Modbus transaction fails; out contains zeros.
// ─────────────────────────────────────────────────────────────────────────────
bool readVelocityComparison(VelocityData &out) {
  uint8_t result;

  // ── Zero-initialise the struct so partial failures leave known values ──────
  out = {0};

  // ── 1. Velocity Amplitude: read 3 contiguous registers starting at 0x3A ───
  result = modbus.readHoldingRegisters(REG_VX, 3);
  if (result != modbus.ku8MBSuccess) {
    Serial.printf("[ERROR] Velocity Amplitude read failed (code 0x%02X)\n", result);
    return false;
  }
  out.vx = (int16_t)modbus.getResponseBuffer(0) / 100.0f;
  out.vy = (int16_t)modbus.getResponseBuffer(1) / 100.0f;
  out.vz = (int16_t)modbus.getResponseBuffer(2) / 100.0f;
  delay(20);   // inter-frame guard

  // ── 2a. VRMSX (0x50) ──────────────────────────────────────────────────────
  result = modbus.readHoldingRegisters(REG_VRMSX, 1);
  if (result != modbus.ku8MBSuccess) {
    Serial.printf("[ERROR] VRMSX read failed (code 0x%02X)\n", result);
    return false;
  }
  out.rawVrmsX = modbus.getResponseBuffer(0);
  out.vrmsx    = out.rawVrmsX / 1000.0f;
  delay(20);

  // ── 2b. VRMSY (0x5C) ──────────────────────────────────────────────────────
  result = modbus.readHoldingRegisters(REG_VRMSY, 1);
  if (result != modbus.ku8MBSuccess) {
    Serial.printf("[ERROR] VRMSY read failed (code 0x%02X)\n", result);
    return false;
  }
  out.rawVrmsY = modbus.getResponseBuffer(0);
  out.vrmsy    = out.rawVrmsY / 1000.0f;
  delay(20);

  // ── 2c. VRMSZ (0x68) ──────────────────────────────────────────────────────
  result = modbus.readHoldingRegisters(REG_VRMSZ, 1);
  if (result != modbus.ku8MBSuccess) {
    Serial.printf("[ERROR] VRMSZ read failed (code 0x%02X)\n", result);
    return false;
  }
  out.rawVrmsZ = modbus.getResponseBuffer(0);
  out.vrmsz    = out.rawVrmsZ / 1000.0f;
  delay(20);

  // ── 3. Vibration Frequency: read 3 contiguous registers starting at 0x44 ──
  result = modbus.readHoldingRegisters(REG_HZX, 3);
  if (result != modbus.ku8MBSuccess) {
    Serial.printf("[ERROR] Vibration Frequency read failed (code 0x%02X)\n", result);
    return false;
  }
  out.hzx = (int16_t)modbus.getResponseBuffer(0) / 10.0f;
  out.hzy = (int16_t)modbus.getResponseBuffer(1) / 10.0f;
  out.hzz = (int16_t)modbus.getResponseBuffer(2) / 10.0f;

  return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// printVelocityReport()
// Formats and prints the comparison table.
// ─────────────────────────────────────────────────────────────────────────────
void printVelocityReport(const VelocityData &d) {
  // Helper lambdas to avoid divide-by-zero
  auto peakOverRms = [](float peak, float rms) -> float {
    return (rms > 0.0f) ? peak / rms : 0.0f;
  };
  auto rmsOverPeak = [](float rms, float peak) -> float {
    return (peak > 0.0f) ? rms / peak : 0.0f;
  };

  Serial.println("=================================================================");
  Serial.println(" WTVB02 Velocity Amplitude vs Velocity RMS");
  Serial.println("-----------------------------------------------------------------");

  Serial.printf(
    " X Peak=%6.3f mm/s  X RMS =%6.3f mm/s  X RAW =%u\n"
    "   X Peak/RMS=%6.3f            X RMS/Peak=%6.3f\n",
    d.vx, d.vrmsx, d.rawVrmsX,
    peakOverRms(d.vx, d.vrmsx), rmsOverPeak(d.vrmsx, d.vx)
  );

  Serial.printf(
    " Y Peak=%6.3f mm/s  Y RMS =%6.3f mm/s  Y RAW =%u\n"
    "   Y Peak/RMS=%6.3f            Y RMS/Peak=%6.3f\n",
    d.vy, d.vrmsy, d.rawVrmsY,
    peakOverRms(d.vy, d.vrmsy), rmsOverPeak(d.vrmsy, d.vy)
  );

  Serial.printf(
    " Z Peak=%6.3f mm/s  Z RMS =%6.3f mm/s  Z RAW =%u\n"
    "   Z Peak/RMS=%6.3f            Z RMS/Peak=%6.3f\n",
    d.vz, d.vrmsz, d.rawVrmsZ,
    peakOverRms(d.vz, d.vrmsz), rmsOverPeak(d.vrmsz, d.vz)
  );

  Serial.println("-----------------------------------------------------------------");
  Serial.println(" Reference: Peak/RMS ~= 1.414   RMS/Peak ~= 0.707");
  Serial.println("-----------------------------------------------------------------");

  Serial.printf(
    " X Freq=%6.1f Hz    Y Freq=%6.1f Hz    Z Freq=%6.1f Hz\n",
    d.hzx, d.hzy, d.hzz
  );

  Serial.println("=================================================================");
  Serial.println();
}

// ─────────────────────────────────────────────────────────────────────────────
// setup()
// ─────────────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(2000);

  Serial.println("=================================================================");
  Serial.println(" WTVB02 Velocity Amplitude vs Velocity RMS  v1.3");
  Serial.println(" RS485 EN=LOW (always enabled)");
  Serial.println("=================================================================");

  // RS485 enable pin – hold LOW permanently (no pre/post-transmission callbacks)
  pinMode(RS485_EN_PIN, OUTPUT);
  rs485Enable();

  // Initialise hardware UART for RS485
  SerialRS485.begin(MODBUS_BAUDRATE, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);

  // Initialise Modbus (no pre/post callbacks needed with always-enabled transceiver)
  modbus.begin(MODBUS_SLAVE_ID, SerialRS485);

  delay(1000);

  // ── Verify sensor is reachable before entering loop ───────────────────────
  Serial.println("[INIT] Checking sensor connection...");
  VelocityData probe = {0};
  while (!readVelocityComparison(probe)) {
    Serial.println("[INIT] Sensor not responding – retrying in 3 s...");
    delay(3000);
  }
  Serial.println("[INIT] Sensor OK.\n");

  // ── [v1.3] Version is not available via Modbus on this hardware — logs why ─
  readFirmwareVersion();

  // ── Read config as-found, then force MODE=0x02 (FreqDomain) + STATICDETECTION ──
  readConfig();

  if (!configureSensorInit()) {
    Serial.println("[INIT] WARNING: could not confirm MODE and/or STATICDETECTION.");
    Serial.println("[INIT] Continuing anyway — CF/velocity readings and static-decay");
    Serial.println("[INIT] behavior may not match the intended configuration.");
  }

  // ── Re-read config to show the confirmed post-switch state ────────────────
  readConfig();

  Serial.println("[INIT] Starting continuous monitoring.\n");
}

// ─────────────────────────────────────────────────────────────────────────────
// loop()
// ─────────────────────────────────────────────────────────────────────────────
void loop() {
  static unsigned long lastRead = 0;
  static unsigned long lastStatsPrint = 0;

  if (millis() - lastRead >= READ_INTERVAL_MS) {
    lastRead = millis();

    uint32_t t0 = millis();
    VelocityData data;
    bool ok = readVelocityComparison(data);
    uint32_t elapsed = millis() - t0;

    Serial.printf("[TIMING] ReadCycle=%lu ms\n", elapsed);
    recordTiming(elapsed);

    if (ok) {
      printVelocityReport(data);
    } else {
      Serial.println("[WARN] Read cycle failed – will retry next interval.");
    }
  }

  // Print rolling Min/Avg/P95/Max every 30 s to build a statistical picture
  // of real transaction time (per DESIGN_PRINCIPLES.md: facts before assumptions)
  if (millis() - lastStatsPrint >= 30000) {
    lastStatsPrint = millis();
    printTimingStats();
  }
}
