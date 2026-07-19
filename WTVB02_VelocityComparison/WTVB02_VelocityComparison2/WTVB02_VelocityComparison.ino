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
struct SpectrumEnergyData;

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

// ── [v1.4] Spectrum Energy registers (FreqDomain mode) ───────────────────────
// Each axis block is 15 contiguous holding registers:
//   Point1..Point8 (8 regs), Area1..Area6 (6 regs), Total Energy (1 reg) = 15
// Base address of each axis block (Point1 register):
#define REG_SPECTRUM_X_BASE   0x006E   // X axis: 0x6E–0x7C
#define REG_SPECTRUM_Y_BASE   0x007D   // Y axis: 0x7D–0x8B
#define REG_SPECTRUM_Z_BASE   0x008C   // Z axis: 0x8C–0x9A
#define SPECTRUM_REG_COUNT    15       // registers per axis block
#define SPECTRUM_POINT_COUNT  8        // Point1..Point8
#define SPECTRUM_AREA_COUNT   6        // Area1..Area6
// Total Energy is the last register of each axis block (Point1_base + 14)
#define SPECTRUM_TOTAL_ENERGY_OFFSET (SPECTRUM_POINT_COUNT + SPECTRUM_AREA_COUNT) // = 14, last reg in block
// Total Energy register addresses — derived, not hardcoded, so they can never
// drift out of sync with the base addresses above.
#define AX_TOTAL_ENERGY  (REG_SPECTRUM_X_BASE + SPECTRUM_TOTAL_ENERGY_OFFSET)  // = 0x7C
#define AY_TOTAL_ENERGY  (REG_SPECTRUM_Y_BASE + SPECTRUM_TOTAL_ENERGY_OFFSET)  // = 0x8B
#define AZ_TOTAL_ENERGY  (REG_SPECTRUM_Z_BASE + SPECTRUM_TOTAL_ENERGY_OFFSET)  // = 0x9A
// NOTE: datasheet does not document a scale factor for these registers, so
// they are treated as raw uint16_t counts (no /100 or /1000 division), same
// as other not-yet-scale-confirmed fields in this file. Revisit if a scale
// factor is later confirmed.

// [v1.4] Sampling Rate (SR, register 0x29) write support — see setSamplingRate()
#define SR_TABLE_SIZE 10
// Index -> label, per vendor SR table (0=32kHz fastest ... 9=64Hz slowest)
static const char* const SAMPLE_RATE_NAMES[SR_TABLE_SIZE] = {
  "32kHz", "16kHz", "8kHz", "4kHz", "2kHz",
  "1kHz", "512Hz", "256Hz", "128Hz", "64Hz"
};

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

// [v1.4] Spectrum Energy data — raw register values, one block per axis.
// Point1..Point8 and Area1..Area6 are kept for completeness (ENERGY command);
// *_totalEnergy is the field most consumers need (Serial + MQTT summary).
struct SpectrumEnergyData {
  uint16_t pointX[SPECTRUM_POINT_COUNT];
  uint16_t areaX[SPECTRUM_AREA_COUNT];
  uint16_t axTotalEnergy;

  uint16_t pointY[SPECTRUM_POINT_COUNT];
  uint16_t areaY[SPECTRUM_AREA_COUNT];
  uint16_t ayTotalEnergy;

  uint16_t pointZ[SPECTRUM_POINT_COUNT];
  uint16_t areaZ[SPECTRUM_AREA_COUNT];
  uint16_t azTotalEnergy;
};

// [v1.5] Cache of the most recently *measured* values, for the SUMMARY
// command (lab-testing convenience). These are only ever written where the
// underlying Modbus read/write already happens elsewhere in this file
// (loop()'s periodic velocity read, ENERGY's spectrum read, SR's write) —
// SUMMARY itself reads only these cached copies and issues no Modbus
// transactions of its own, per requirement.
VelocityData      g_lastVelocity = {0};
SpectrumEnergyData g_lastSpectrum = {0};
uint16_t          g_lastSR = 0xFFFF;   // 0xFFFF = "not yet known" (no SR read/write done yet)

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
// setSamplingRate()
// [v1.4] Part 4/5: writes the Sample Rate register (0x29) via Modbus Function
// Code 0x06 (ModbusMaster::writeSingleRegister), SAVES it, then immediately
// reads it back to verify the write actually took effect.
//
// sr must be 0..9 per the vendor SR table (see SAMPLE_RATE_NAMES). Sequence
// mirrors configureSensorInit()'s existing pattern for protected config
// registers: Unlock -> Write -> Save -> (read back to verify). Without the
// explicit Save (REG_SAVE_REBOOT/SAVE_CMD), some sensor firmware revisions
// do not persist the new value past the next power-cycle/reboot — only a
// full Reboot (REBOOT_CMD) is intentionally NOT issued here, since Part 5
// requires reading 0x29 back "immediately" and a reboot would introduce a
// multi-second gap and a temporary loss of comms before that read.
//
// Returns true only if the read-back value equals the written value.
// On mismatch, prints "Sampling Rate Write Failed" and returns false
// (per PART 5 — no silent continuation).
// ─────────────────────────────────────────────────────────────────────────────
bool setSamplingRate(uint16_t sr) {
  if (sr >= SR_TABLE_SIZE) {
    Serial.printf("[SR] Invalid sampling rate index %u (valid range 0-%d)\n", sr, SR_TABLE_SIZE - 1);
    return false;
  }

  // Unlock write-protect (same key/register used elsewhere in this file)
  modbus.writeSingleRegister(REG_UNLOCK, UNLOCK_KEY);
  delay(100);

  // Write SR via Function Code 0x06 (writeSingleRegister)
  uint8_t writeResult = modbus.writeSingleRegister(REG_SR, sr);
  if (writeResult != modbus.ku8MBSuccess) {
    Serial.printf("[SR] Write failed (code 0x%02X)\n", writeResult);
    Serial.println("Sampling Rate Write Failed");
    return false;
  }
  delay(50);

  // Save so the new SR persists past reboot (same SAVE_CMD used in
  // configureSensorInit(); no REBOOT_CMD here — see function comment above)
  uint8_t saveResult = modbus.writeSingleRegister(REG_SAVE_REBOOT, SAVE_CMD);
  if (saveResult != modbus.ku8MBSuccess) {
    Serial.printf("[SR] Save failed (code 0x%02X)\n", saveResult);
    Serial.println("Sampling Rate Write Failed");
    return false;
  }
  delay(100);

  // Immediately read back Register 0x29 to verify
  uint8_t readResult = modbus.readHoldingRegisters(REG_SR, 1);
  if (readResult != modbus.ku8MBSuccess) {
    Serial.printf("[SR] Verify read failed (code 0x%02X)\n", readResult);
    Serial.println("Sampling Rate Write Failed");
    return false;
  }

  uint16_t readBack = modbus.getResponseBuffer(0);
  if (readBack != sr) {
    Serial.printf("[SR] Verify mismatch: wrote %u, read back %u\n", sr, readBack);
    Serial.println("Sampling Rate Write Failed");
    return false;
  }

  g_lastSR = readBack;   // [v1.5] cache for SUMMARY — value already just verified above
  Serial.printf("Sampling Rate = %s\n", SAMPLE_RATE_NAMES[sr]);
  return true;
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
// readSpectrumAxisBlock()
// Reads one 15-register Spectrum Energy axis block (Point1..8, Area1..6,
// Total Energy) starting at 'baseReg' and unpacks it into the three output
// arrays/values. Shared by all three axes to avoid duplicated code.
// Returns false (Modbus error) without modifying outputs on failure.
// ─────────────────────────────────────────────────────────────────────────────
bool readSpectrumAxisBlock(uint16_t baseReg, uint16_t* points, uint16_t* areas, uint16_t& totalEnergy) {
  uint8_t result = modbus.readHoldingRegisters(baseReg, SPECTRUM_REG_COUNT);
  if (result != modbus.ku8MBSuccess) {
    Serial.printf("[ERROR] Spectrum Energy read failed at 0x%04X (code 0x%02X)\n", baseReg, result);
    return false;
  }
  for (uint8_t i = 0; i < SPECTRUM_POINT_COUNT; i++) {
    points[i] = modbus.getResponseBuffer(i);
  }
  for (uint8_t i = 0; i < SPECTRUM_AREA_COUNT; i++) {
    areas[i] = modbus.getResponseBuffer(SPECTRUM_POINT_COUNT + i);
  }
  totalEnergy = modbus.getResponseBuffer(SPECTRUM_TOTAL_ENERGY_OFFSET); // last of the 15
  return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// readSpectrumEnergy()
// Reads all three Spectrum Energy axis blocks (X, Y, Z). Called on-demand
// (ENERGY serial command / MQTT spectrum publish), NOT from the main
// READ_INTERVAL_MS velocity loop, so existing loop timing is unaffected.
// Returns false if any axis block read fails; out contains zeros in that case.
// ─────────────────────────────────────────────────────────────────────────────
bool readSpectrumEnergy(SpectrumEnergyData &out) {
  out = {0};

  if (!readSpectrumAxisBlock(REG_SPECTRUM_X_BASE, out.pointX, out.areaX, out.axTotalEnergy)) return false;
  delay(20);   // inter-frame guard, matches existing readVelocityComparison() style

  if (!readSpectrumAxisBlock(REG_SPECTRUM_Y_BASE, out.pointY, out.areaY, out.ayTotalEnergy)) return false;
  delay(20);

  if (!readSpectrumAxisBlock(REG_SPECTRUM_Z_BASE, out.pointZ, out.areaZ, out.azTotalEnergy)) return false;

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
// printSpectrumEnergyReport()
// Part 2 required output (short form: AX/AY/AZ totals) plus full Point/Area
// breakdown used by the ENERGY serial command (Part 6).
// ─────────────────────────────────────────────────────────────────────────────
void printSpectrumEnergyReport(const SpectrumEnergyData &d) {
  Serial.println("------------------------");
  Serial.println("Spectrum Energy");
  Serial.println("------------------------");
  Serial.printf("AX Total : %u\n", d.axTotalEnergy);
  Serial.printf("AY Total : %u\n", d.ayTotalEnergy);
  Serial.printf("AZ Total : %u\n", d.azTotalEnergy);

  // Full per-point / per-area breakdown (Part 6, ENERGY command)
  const uint16_t* pointSets[3] = { d.pointX, d.pointY, d.pointZ };
  const uint16_t* areaSets[3]  = { d.areaX,  d.areaY,  d.areaZ  };
  const char axisLetters[3] = { 'X', 'Y', 'Z' };

  for (uint8_t axis = 0; axis < 3; axis++) {
    Serial.printf("-- %c axis --\n", axisLetters[axis]);
    for (uint8_t i = 0; i < SPECTRUM_POINT_COUNT; i++) {
      Serial.printf("  Point%u : %u\n", i + 1, pointSets[axis][i]);
    }
    for (uint8_t i = 0; i < SPECTRUM_AREA_COUNT; i++) {
      Serial.printf("  Area%u  : %u\n", i + 1, areaSets[axis][i]);
    }
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// buildSpectrumJson()
// Part 3 requirement: expose Spectrum Energy as new JSON fields, e.g.
//   "spectrum":{"ax_total":853,"ay_total":790,"az_total":605}
// NOTE: this file has no existing WiFi/MQTT client or JSON publisher to
// attach to (none is present in the original firmware), so this helper only
// BUILDS the fragment as a String and prints it to Serial for now. If/when
// an MQTT publish function exists elsewhere in the project, splice this
// fragment into that existing payload — do not duplicate the concatenation
// logic. See PART 3 note in the accompanying explanation.
// ─────────────────────────────────────────────────────────────────────────────
String buildSpectrumJson(const SpectrumEnergyData &d) {
  String json = "\"spectrum\":{";
  json += "\"ax_total\":" + String(d.axTotalEnergy) + ",";
  json += "\"ay_total\":" + String(d.ayTotalEnergy) + ",";
  json += "\"az_total\":" + String(d.azTotalEnergy);
  json += "}";
  return json;
}

// ─────────────────────────────────────────────────────────────────────────────
// getModeName()
// Shared MODE (0x07) value -> label mapping, so the new post-SR status report
// doesn't duplicate the ternary already inline inside readConfig(). readConfig()
// itself is left untouched (out of scope) to avoid refactoring unrelated code.
// ─────────────────────────────────────────────────────────────────────────────
const char* getModeName(uint16_t modeValue) {
  switch (modeValue) {
    case 0:  return "LowFreq";
    case 1:  return "HighFreq(CF)";
    case 2:  return "FreqDomain";
    default: return "unknown";
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// printPostSRStatus()
// [v1.4] Convenience report requested for the "SR n" workflow: after a
// successful sampling-rate change, print current MODE and the current
// Spectrum Energy totals. (Current SR itself is NOT re-printed here —
// setSamplingRate() already prints "Sampling Rate = <name>" once on
// success; re-reading/re-printing it here would just duplicate that line.)
// Only invoked from handleSerialCommands() after setSamplingRate() succeeds,
// so it does not run on the timed READ_INTERVAL_MS loop.
// ─────────────────────────────────────────────────────────────────────────────
void printPostSRStatus() {
  // Current Mode
  if (modbus.readHoldingRegisters(REG_MODE, 1) == modbus.ku8MBSuccess) {
    Serial.printf("Mode = %s\n", getModeName(modbus.getResponseBuffer(0)));
  } else {
    Serial.println("Mode = READ FAILED");
  }
  delay(20);

  // Current Spectrum Energy totals
  SpectrumEnergyData spectrum;
  if (readSpectrumEnergy(spectrum)) {
    g_lastSpectrum = spectrum;   // [v1.5] cache for SUMMARY — no extra Modbus read
    Serial.printf("AX Total = %u\n", spectrum.axTotalEnergy);
    Serial.printf("AY Total = %u\n", spectrum.ayTotalEnergy);
    Serial.printf("AZ Total = %u\n", spectrum.azTotalEnergy);
  } else {
    Serial.println("Spectrum Energy = READ FAILED");
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// printInfoReport()
// Part 6 "INFO" command: firmware version, current SR, Modbus address, baud.
// ─────────────────────────────────────────────────────────────────────────────
void printInfoReport() {
  Serial.println("------------------------");
  Serial.println("Info");
  Serial.println("------------------------");

  // Firmware version: no Modbus register exists for this (see
  // readFirmwareVersion() note above) — report the confirmed known value.
  Serial.println("Firmware Version   : 10211.1.24 (per Witmotion PC tool, not Modbus-readable)");

  // Current Sampling Rate — read live from the sensor
  if (modbus.readHoldingRegisters(REG_SR, 1) == modbus.ku8MBSuccess) {
    uint16_t sr = modbus.getResponseBuffer(0);
    g_lastSR = sr;   // [v1.5] cache for SUMMARY — no extra Modbus read
    const char* name = (sr < SR_TABLE_SIZE) ? SAMPLE_RATE_NAMES[sr] : "unknown";
    Serial.printf("Current Sampling Rate : %s (index %u)\n", name, sr);
  } else {
    Serial.println("Current Sampling Rate : READ FAILED");
  }

  Serial.printf("Current Modbus Address : 0x%02X\n", MODBUS_SLAVE_ID);
  Serial.printf("Current Baudrate       : %u\n", MODBUS_BAUDRATE);
}

// ─────────────────────────────────────────────────────────────────────────────
// printSummaryReport()
// [v1.5] Compact one-screen lab-testing summary: Peak / VRMS / Freq per axis
// (from the last periodic velocity read, cached in g_lastVelocity by loop())
// plus each axis's Total Energy (from the last ENERGY/SR read, cached in
// g_lastSpectrum). Deliberately reads NO Modbus registers itself — it only
// formats values that were already measured elsewhere in this file, per
// requirement. If g_lastSpectrum/g_lastSR haven't been populated yet (i.e.
// ENERGY/SR/INFO has never been run this session), Total Energy shows 0 and
// Sampling Rate shows "unknown" rather than silently guessing.
// ─────────────────────────────────────────────────────────────────────────────
void printSummaryReport() {
  const char* srName = (g_lastSR < SR_TABLE_SIZE) ? SAMPLE_RATE_NAMES[g_lastSR] : "unknown";

  Serial.println("==============================================================");
  Serial.printf("Sampling Rate : %s\n", srName);
  Serial.println("Axis     Peak(mm/s)   VRMS(mm/s)   Freq(Hz)   TotalEnergy");
  Serial.println("----------------------------------------------------------");
  Serial.printf("X        %8.2f     %8.2f     %8.1f      %u\n",
                g_lastVelocity.vx, g_lastVelocity.vrmsx, g_lastVelocity.hzx, g_lastSpectrum.axTotalEnergy);
  Serial.printf("Y        %8.2f     %8.2f     %8.1f      %u\n",
                g_lastVelocity.vy, g_lastVelocity.vrmsy, g_lastVelocity.hzy, g_lastSpectrum.ayTotalEnergy);
  Serial.printf("Z        %8.2f     %8.2f     %8.1f      %u\n",
                g_lastVelocity.vz, g_lastVelocity.vrmsz, g_lastVelocity.hzz, g_lastSpectrum.azTotalEnergy);
  Serial.println("==============================================================");
}

// ─────────────────────────────────────────────────────────────────────────────
// handleSerialCommands()
// [v1.4] Part 6: non-blocking line-based command parser for USB Serial.
// Builds up a line in a static buffer as bytes arrive (no blocking reads),
// and only acts once a newline is seen — so it adds negligible overhead to
// loop() and does not disturb READ_INTERVAL_MS timing when idle/no input.
//
// Supported commands:
//   SR 0..SR 9  -> setSamplingRate(n), prints "Sampling Rate = <name>"
//   ENERGY      -> reads + prints full Spectrum Energy report
//   INFO        -> prints firmware version / SR / Modbus address / baud
//   SUMMARY     -> [v1.5] compact Peak/VRMS/Freq/TotalEnergy table (cached values only)
// ─────────────────────────────────────────────────────────────────────────────
void handleSerialCommands() {
  static char lineBuf[32];
  static uint8_t lineLen = 0;

  while (Serial.available() > 0) {
    char c = (char)Serial.read();

    if (c == '\n' || c == '\r') {
      if (lineLen == 0) continue;   // ignore blank lines / \r\n pairs
      lineBuf[lineLen] = '\0';

      String cmd = String(lineBuf);
      cmd.trim();
      cmd.toUpperCase();

      if (cmd.startsWith("SR ")) {
        int sr = cmd.substring(3).toInt();
        if (setSamplingRate((uint16_t)sr)) {   // prints "Sampling Rate = <name>" itself on success
          printPostSRStatus();                  // then: SR / Mode / AX-AY-AZ Total, for quick testing
        }
      } else if (cmd == "ENERGY") {
        SpectrumEnergyData spectrum;
        if (readSpectrumEnergy(spectrum)) {
          g_lastSpectrum = spectrum;   // [v1.5] cache for SUMMARY — no extra Modbus read
          printSpectrumEnergyReport(spectrum);
        } else {
          Serial.println("[ERROR] Spectrum Energy read failed.");
        }
      } else if (cmd == "INFO") {
        printInfoReport();
      } else if (cmd == "SUMMARY") {
        printSummaryReport();
      } else {
        Serial.printf("[CMD] Unknown command: %s\n", cmd.c_str());
      }

      lineLen = 0;   // reset for next line
    } else if (lineLen < sizeof(lineBuf) - 1) {
      lineBuf[lineLen++] = c;
    }
    // else: silently drop overflow chars, buffer stays valid
  }
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

  // [v1.4] Part 6: process any pending "SR n" / "ENERGY" / "INFO" serial
  // commands. Non-blocking (only drains what's already buffered), so it
  // does not alter the READ_INTERVAL_MS cadence below.
  handleSerialCommands();

  if (millis() - lastRead >= READ_INTERVAL_MS) {
    lastRead = millis();

    uint32_t t0 = millis();
    VelocityData data;
    bool ok = readVelocityComparison(data);
    uint32_t elapsed = millis() - t0;

    Serial.printf("[TIMING] ReadCycle=%lu ms\n", elapsed);
    recordTiming(elapsed);

    if (ok) {
      g_lastVelocity = data;   // [v1.5] cache for SUMMARY — no extra Modbus read
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
