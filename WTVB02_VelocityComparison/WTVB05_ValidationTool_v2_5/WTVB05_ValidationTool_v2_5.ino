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
struct AxisPrevSample;

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
#define REG_DRM 0x002B // [v2.2] Displacement Range Mode: 1=60000um-1um resolution, 2=600um-0.01um resolution (datasheet §6.1.4.14)
                                    // "After the device is stationary, there is still vibration data.
                                    //  The speed at which the data is cleared has no unit meaning and
                                    //  is a coefficient." — WTVB05 datasheet 6.1.4.5. Left at implicit
                                    // default (100) by prior firmware versions; now written explicitly
                                    // at boot so behavior is deterministic and tunable from one place.
#define REG_TEMP       0x0040   // Temperature, scale: raw / 100.0 → °C
#define REG_UNLOCK     0x0069   // Write-protect unlock register
#define REG_SAVE_REBOOT 0x0000  // Save (0x0000) / Reboot (0x00FF) command register

// ── [v2.0] Acceleration / Crest Factor / Kurtosis registers ──────────────────
// Instantaneous acceleration   scale: raw / 32768.0 * 16.0 → g   (§6.1.4.8)
#define REG_AX      0x0034
#define REG_AY      0x0035
#define REG_AZ      0x0036

// Crest Factor + Kurtosis      scale: raw / 1000.0 (both, unsigned)
// Each axis block is CFx (first reg) immediately followed by Kx (second reg),
// per Register Dictionary v1.2 §7 (CFX=0x47, CFY=0x53, CFZ=0x5F — same
// offsets confirmed independently in WTVB02_ESP32S3_V16_CM_fault_latch's
// REG_CFX/CFY/CFZ, cross-validated against this project's own reverse
// engineering of data_0.bin).
#define REG_CFX     0x0047   // CFX (0x47) + KX (0x48)
#define REG_CFY     0x0053   // CFY (0x53) + KY (0x54)
#define REG_CFZ     0x005F   // CFZ (0x5F) + KZ (0x60)

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
// [v1.6] Same table as plain Hz numbers (for the CSV "SamplingRate" column,
// which needs a bare number, not the "512Hz" text form used elsewhere).
static const uint32_t SAMPLE_RATE_HZ[SR_TABLE_SIZE] = {
  32000, 16000, 8000, 4000, 2000, 1000, 512, 256, 128, 64
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
// [v2.2] Estimated total transaction time: now 11 Modbus transactions per
//   cycle (was 9): Peak, VRMSX, VRMSY, VRMSZ, Freq, Accel, CFX/KX, CFY/KY,
//   CFZ/KZ, Temp, StaticDetection, DRM + delay(20ms) between each -> delay
//   alone accounts for ~220ms, plus 11x actual transaction time. Verify with
//   [TIMING] log below before trusting this range -- if P95/Max approaches
//   READ_INTERVAL_MS, increase READ_INTERVAL_MS rather than removing the
//   inter-frame delays.
#define READ_INTERVAL_MS  500

// ── [v1.6] Serial logging mode ────────────────────────────────────────────────
// 0 (default): unchanged verbose human-readable output, plus the new
//              "Experiment Summary" block after each cycle's existing report.
// 1: every periodic read cycle prints ONLY a single CSV data line (no banner,
//    no [TIMING] line, no velocity report, no periodic stats) — intended for
//    piping Serial output straight into a spreadsheet/logging tool. Serial
//    commands (ENERGY/SR/INFO/SUMMARY) still work normally either way, since
//    they are user-typed, not part of the automatic per-cycle output.
#define SERIAL_CSV_MODE 1

// ── [v1.7] Populate Spectrum Energy cache at boot? ────────────────────────────
// Default 0: TotalEnergy in Experiment Summary/CSV/SUMMARY reads 0 until you
//   run ENERGY or SR n at least once — this is the original, strictly
//   "zero extra Modbus reads" behavior.
// Set to 1: setup() performs ONE extra readSpectrumEnergy() call (3 Modbus
//   reads, same as the existing ENERGY command) right after boot config, so
//   TotalEnergy is already populated for the very first Experiment Summary —
//   at the cost of one boot-time round trip. This is a ONE-TIME transaction,
//   not added to the per-cycle polling loop, so READ_INTERVAL_MS/timing is
//   still unaffected — but it IS new Modbus traffic, so it defaults OFF.
#define SPECTRUM_READ_AT_BOOT 0

// ── [v1.9] Periodic Spectrum Energy auto-refresh ─────────────────────────────
// Root cause of "Spectrum Snapshot ... STALE" appearing indefinitely in
// Experiment Summary/CSV: g_lastSpectrum was previously written ONLY by the
// user-typed ENERGY / SR n commands (see readSpectrumEnergy()'s own comment
// above) — if neither is run during a long unattended session, the same
// snapshot (and its TotalEnergy) is reported forever, well past
// SPECTRUM_STALE_THRESHOLD_MS.
//
// Fix: re-run readSpectrumEnergy() automatically every
// SPECTRUM_REFRESH_INTERVAL_MS, on its own timer in loop() — same pattern as
// the existing 30 s printTimingStats() timer just below it. This is 3 extra
// Modbus transactions every 30 s (not every READ_INTERVAL_MS cycle), so the
// per-cycle polling timing this file already measures ([TIMING] ReadCycle)
// is unaffected. Set to 0 to disable and fall back to the old
// manual-only (ENERGY/SR n) behavior.
#define SPECTRUM_AUTO_REFRESH        1
#define SPECTRUM_REFRESH_INTERVAL_MS 30000

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

  // [v2.0] Instantaneous acceleration, g (registers 0x34-0x36)
  float    ax;
  float    ay;
  float    az;

  // [v2.0] Crest Factor (unitless ratio) + Kurtosis (unitless)
  // Read alongside Peak/RMS/Freq every cycle -- NOT gated on motor_state or
  // any decision logic (this tool has none); purely for correlating with
  // Peak/RMS deviations, per the VRMS-glitch-vs-real-DSP-output investigation.
  float    cfx, cfy, cfz;
  float    kx,  ky,  kz;

  // [v2.0] Temperature, °C (register 0x40) -- read EVERY cycle here, unlike
  // g_lastTempC elsewhere in this file which is only refreshed by readConfig()
  // (boot-time / manual). Having it inside VelocityData means it rides along
  // with every CSV row and Experiment Summary automatically.
  float    tempC;
  bool     tempValid;

  // [v2.2] Configuration registers, read EVERY cycle (not just at boot like
  // readConfig()/configureSensorInit()) -- so every data row and every
  // glitch/anomaly event has its own config ground-truth attached, instead
  // of relying on a boot-time snapshot that may be stale if the sensor
  // silently reverts config after a self-reboot (a known failure mode
  // documented in the main firmware, WTVB02_ESP32S3_V16_CM...).
  uint16_t staticDetectionRaw;  // register 0x08, raw value (100 = 1.00x)
  bool     staticDetectionValid;
  uint16_t drmRaw;              // register 0x2B, raw value (1=60000um range, 2=600um range)
  bool     drmValid;
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

// ── [v2.1] RFC-0004 instantaneous anomaly score (data-collection aid only) ──
// This computes the 4-condition score from RFC-0004_DSP_Anomaly_Handling.md
// §3 EVERY cycle, per axis, and logs it to the CSV. It does NOT suppress,
// hold, or alter any value in VelocityData -- this tool's job is to collect
// ground-truth evidence, not to act on it. Post-processing the CSV (filter
// rows where any score >= 3) replaces manually scanning Serial logs by eye.
//
// Tunable bounds -- these are guesses from a single observed event
// (Register Dictionary v1.3 §8.5) and are EXPECTED to need adjustment once
// more data comes in. Do not treat these constants as validated.
#define SCORE_PEAKRMS_RATIO_MIN   0.3f   // ratio below this = outside expected range
#define SCORE_PEAKRMS_RATIO_MAX   3.0f   // ratio above this = outside expected range
#define SCORE_FREQ_IMPLAUSIBLE_HZ 250.0f // dominant freq above this = implausible for this machine
#define SCORE_RMS_DELTA_PCT       0.80f  // RMS change vs previous sample, fraction
#define SCORE_PEAK_STABLE_PCT     0.15f  // Peak considered "roughly unchanged" within this fraction
#define SCORE_RMS_MOVED_PCT       0.50f  // RMS considered "moved a lot" beyond this fraction

struct AxisPrevSample { float peak = 0; float rms = 0; bool valid = false; };
AxisPrevSample g_prevX, g_prevY, g_prevZ;

// Returns 0-4. Pure function of current + previous sample -- no duration,
// no state, per RFC-0004 §2/§3 (detection must be instantaneous).
int computeAxisScore(float peak, float rms, float freq, AxisPrevSample &prev) {
  int score = 0;

  float ratio = (rms > 0.0f) ? peak / rms : 0.0f;
  if (ratio < SCORE_PEAKRMS_RATIO_MIN || ratio > SCORE_PEAKRMS_RATIO_MAX) score++;

  if (freq > SCORE_FREQ_IMPLAUSIBLE_HZ) score++;

  if (prev.valid && prev.rms > 0.0f) {
    float rmsDeltaFrac = fabsf(rms - prev.rms) / prev.rms;
    if (rmsDeltaFrac > SCORE_RMS_DELTA_PCT) score++;

    float peakDeltaFrac = (prev.peak > 0.0f) ? fabsf(peak - prev.peak) / prev.peak : 0.0f;
    if (peakDeltaFrac < SCORE_PEAK_STABLE_PCT && rmsDeltaFrac > SCORE_RMS_MOVED_PCT) score++;
  }

  prev.peak = peak;
  prev.rms = rms;
  prev.valid = true;
  return score;
}
SpectrumEnergyData g_lastSpectrum = {0};
uint16_t          g_lastSR = 0xFFFF;   // 0xFFFF = "not yet known" (no SR read/write done yet)

// [v1.6] Additional cached values for the Experiment Summary / CSV logging
// feature. Same rule as above: only ever written where the underlying
// Modbus read already happens elsewhere (readConfig() at boot,
// configureSensorInit()'s MODE verify, printPostSRStatus()'s MODE read,
// loop()'s existing elapsed-time calculation) — no new Modbus reads.
uint16_t g_lastMode      = 0xFFFF;  // 0xFFFF = "not yet known"
float    g_lastTempC     = 0.0f;
bool     g_lastTempValid = false;   // true once TEMP has been read at least once
uint32_t g_lastReadCycleMs = 0;

// [v1.7] Lab-test metadata — set via the new "ID <value>" / "SETPOINT <value>"
// serial commands. These are NOT read from the sensor at all (no Modbus
// involved); they exist purely so the Experiment Summary/CSV output can be
// correlated against external test conditions (which physical run, what the
// motor drive was set to) when reviewing logs later.
String g_experimentId    = "001";
float  g_motorSetpointHz = 0.0f;

// [v1.8] Spectrum snapshot provenance — records WHEN g_lastSpectrum was
// captured (millis() at that moment), so Experiment Summary/CSV can show
// the Realtime Timestamp and Spectrum Timestamp side by side instead of
// implying they're the same instant. Set at every existing point that
// writes g_lastSpectrum (ENERGY command, printPostSRStatus(), optional
// boot read) — no new Modbus reads, just millis() calls alongside the
// existing cache assignment.
uint32_t g_lastSpectrumTimestampMs = 0;
bool     g_spectrumValid           = false;  // true once a spectrum snapshot exists at all
#define SPECTRUM_STALE_THRESHOLD_MS 10000     // age beyond which the snapshot is flagged STALE

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
    g_lastSR = v;   // [v1.6] cache for Experiment Summary/CSV — no extra Modbus read
    // [v1.7] FIX: this used to be a local 4-entry mapping (0,1,2,7 only) that
    // reported "unknown" for any other valid index (e.g. 6 = 512Hz). Reuse
    // the same SAMPLE_RATE_NAMES table already used everywhere else in this
    // file, so this line can never disagree with SR n / INFO / SUMMARY again.
    const char* name = (v < SR_TABLE_SIZE) ? SAMPLE_RATE_NAMES[v] : "unknown";
    Serial.printf("  SR  (0x%02X) = 0x%04X = SR %u (%s)\n", REG_SR, v, v, name);
  } else {
    Serial.println("  SR  : READ FAILED");
  }
  delay(50);

  // MODE
  if (modbus.readHoldingRegisters(REG_MODE, 1) == modbus.ku8MBSuccess) {
    uint16_t v = modbus.getResponseBuffer(0);
    g_lastMode = v;   // [v1.6] cache for Experiment Summary/CSV — no extra Modbus read
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
    float tempC = (int16_t)modbus.getResponseBuffer(0) / 100.0f;
    g_lastTempC = tempC;        // [v1.6] cache for Experiment Summary/CSV — no extra Modbus read
    g_lastTempValid = true;
    Serial.printf("  TEMP(0x%02X) = %.1f C\n", REG_TEMP, tempC);
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
    g_lastMode = v;   // [v1.6] cache for Experiment Summary/CSV — no extra Modbus read
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
  delay(20);

  // ── [v2.0] 4. Instantaneous Acceleration: 3 contiguous registers at 0x34 ──
  // Scale per datasheet §6.1.4.8: raw / 32768.0 * 16.0 → g
  result = modbus.readHoldingRegisters(REG_AX, 3);
  if (result != modbus.ku8MBSuccess) {
    Serial.printf("[ERROR] Acceleration read failed (code 0x%02X)\n", result);
    return false;
  }
  out.ax = (int16_t)modbus.getResponseBuffer(0) / 32768.0f * 16.0f;
  out.ay = (int16_t)modbus.getResponseBuffer(1) / 32768.0f * 16.0f;
  out.az = (int16_t)modbus.getResponseBuffer(2) / 32768.0f * 16.0f;
  delay(20);

  // ── [v2.0] 5a. Crest Factor X + Kurtosis X (0x47-0x48) ────────────────────
  result = modbus.readHoldingRegisters(REG_CFX, 2);
  if (result != modbus.ku8MBSuccess) {
    Serial.printf("[ERROR] CFX/KX read failed (code 0x%02X)\n", result);
    return false;
  }
  out.cfx = (uint16_t)modbus.getResponseBuffer(0) / 1000.0f;   // unsigned per datasheet
  out.kx  = (uint16_t)modbus.getResponseBuffer(1) / 1000.0f;
  delay(20);

  // ── [v2.0] 5b. Crest Factor Y + Kurtosis Y (0x53-0x54) ────────────────────
  result = modbus.readHoldingRegisters(REG_CFY, 2);
  if (result != modbus.ku8MBSuccess) {
    Serial.printf("[ERROR] CFY/KY read failed (code 0x%02X)\n", result);
    return false;
  }
  out.cfy = (uint16_t)modbus.getResponseBuffer(0) / 1000.0f;
  out.ky  = (uint16_t)modbus.getResponseBuffer(1) / 1000.0f;
  delay(20);

  // ── [v2.0] 5c. Crest Factor Z + Kurtosis Z (0x5F-0x60) ────────────────────
  result = modbus.readHoldingRegisters(REG_CFZ, 2);
  if (result != modbus.ku8MBSuccess) {
    Serial.printf("[ERROR] CFZ/KZ read failed (code 0x%02X)\n", result);
    return false;
  }
  out.cfz = (uint16_t)modbus.getResponseBuffer(0) / 1000.0f;
  out.kz  = (uint16_t)modbus.getResponseBuffer(1) / 1000.0f;
  delay(20);

  // ── [v2.0] 6. Temperature (0x40) -- read every cycle, not just at boot ────
  result = modbus.readHoldingRegisters(REG_TEMP, 1);
  if (result == modbus.ku8MBSuccess) {
    out.tempC     = (int16_t)modbus.getResponseBuffer(0) / 100.0f;
    out.tempValid = true;
  } else {
    // Temperature is diagnostic, not core to the Peak/RMS/Freq comparison --
    // do NOT fail the whole cycle over it. Leave tempValid=false (already
    // zero-initialised at the top of this function) and continue.
    Serial.printf("[WARN] Temperature read failed (code 0x%02X) -- continuing\n", result);
  }
  delay(20);

  // ── [v2.2] 7. STATICDETECTION (0x08) -- read every cycle, not just at boot ─
  // Previously only read once at boot (readConfig()/configureSensorInit()).
  // Reading it here means every CSV row -- including any future glitch/
  // anomaly event -- has its own config ground-truth, so we can tell whether
  // an anomaly coincides with this coefficient silently changing (e.g. after
  // an undetected sensor self-reboot reverting it to a different value).
  // Diagnostic only -- a failed read here must NOT fail the whole cycle.
  result = modbus.readHoldingRegisters(REG_STATICDETECTION, 1);
  if (result == modbus.ku8MBSuccess) {
    out.staticDetectionRaw   = modbus.getResponseBuffer(0);
    out.staticDetectionValid = true;
  } else {
    Serial.printf("[WARN] STATICDETECTION read failed (code 0x%02X) -- continuing\n", result);
  }
  delay(20);

  // ── [v2.2] 8. DRM / Displacement Range Mode (0x2B) -- read every cycle ────
  // Same rationale as STATICDETECTION above: this affects displacement
  // resolution (60000um-1um vs 600um-0.01um range) and was previously only
  // known at boot time. Diagnostic only.
  result = modbus.readHoldingRegisters(REG_DRM, 1);
  if (result == modbus.ku8MBSuccess) {
    out.drmRaw   = modbus.getResponseBuffer(0);
    out.drmValid = true;
  } else {
    Serial.printf("[WARN] DRM read failed (code 0x%02X) -- continuing\n", result);
  }

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
// refreshSpectrumEnergy()
// [v1.9] Thin wrapper around readSpectrumEnergy() that also updates the
// caching/provenance globals (g_lastSpectrum, g_lastSpectrumTimestampMs,
// g_spectrumValid) — the exact same three lines already duplicated at every
// existing call site (ENERGY command, printPostSRStatus(), optional boot
// read). Added so the new periodic auto-refresh timer in loop() (and any
// future call site) can reuse one place instead of copying those three
// lines again. Existing call sites are left as-is (out of scope) to avoid
// touching unrelated, already-working code.
// Returns true/false exactly like readSpectrumEnergy() — caller decides
// whether/how to report a failure.
// ─────────────────────────────────────────────────────────────────────────────
bool refreshSpectrumEnergy() {
  SpectrumEnergyData spectrum;
  if (!readSpectrumEnergy(spectrum)) return false;
  g_lastSpectrum = spectrum;
  g_lastSpectrumTimestampMs = millis();
  g_spectrumValid = true;
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
  // [v2.3] Reference ratio (Peak/RMS ~= 1.414) removed from here -- it's a
  // constant, not measured data, so repeating it every cycle just wastes
  // Serial bandwidth. Printed once at boot instead (see setup()).

  Serial.printf(
    " X Freq=%6.1f Hz    Y Freq=%6.1f Hz    Z Freq=%6.1f Hz\n",
    d.hzx, d.hzy, d.hzz
  );

  Serial.println("-----------------------------------------------------------------");
  Serial.printf(
    " X Acc =%6.3f g     Y Acc =%6.3f g     Z Acc =%6.3f g\n",
    d.ax, d.ay, d.az
  );
  Serial.printf(
    " X CF  =%6.2f       Y CF  =%6.2f       Z CF  =%6.2f\n",
    d.cfx, d.cfy, d.cfz
  );
  Serial.printf(
    " X Kurt=%6.2f       Y Kurt=%6.2f       Z Kurt=%6.2f\n",
    d.kx, d.ky, d.kz
  );
  if (d.tempValid) {
    Serial.printf(" Temp  =%6.1f C\n", d.tempC);
  } else {
    Serial.println(" Temp  = READ FAILED this cycle");
  }
  // [v2.2] Config ground-truth, read every cycle -- see readVelocityComparison()
  if (d.staticDetectionValid) {
    Serial.printf(" StaticDetection = %u (%.2fx)", d.staticDetectionRaw, d.staticDetectionRaw / 100.0f);
  } else {
    Serial.print(" StaticDetection = READ FAILED this cycle");
  }
  if (d.drmValid) {
    Serial.printf("   DRM = %u\n", d.drmRaw);
  } else {
    Serial.println("   DRM = READ FAILED this cycle");
  }

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
    uint16_t modeVal = modbus.getResponseBuffer(0);
    g_lastMode = modeVal;   // [v1.6] cache for Experiment Summary/CSV — no extra Modbus read
    Serial.printf("Mode = %s\n", getModeName(modeVal));
  } else {
    Serial.println("Mode = READ FAILED");
  }
  delay(20);

  // Current Spectrum Energy totals
  SpectrumEnergyData spectrum;
  if (readSpectrumEnergy(spectrum)) {
    g_lastSpectrum = spectrum;   // [v1.5] cache for SUMMARY — no extra Modbus read
    g_lastSpectrumTimestampMs = millis();   // [v1.8] record when this snapshot was taken
    g_spectrumValid = true;
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
// printExperimentSummaryRow()
// One axis line of the Experiment Summary table. Peak/RMS ratio uses the
// same divide-by-zero guard as printVelocityReport()'s local lambda
// (duplicated here as a one-liner only because that lambda is scoped
// inside printVelocityReport() and not reusable from here without touching
// that existing, untouched function).
// [v2.0] Extended with Accel/CF/Kurtosis -- note the existing "Crest" column
// here was always the Peak/RMS ratio (NOT the sensor's own Crest Factor
// register), kept as-is and renamed "PeakRatio" in the header below to avoid
// clashing with the new, distinct "CF" column (sensor-computed Crest Factor).
// ─────────────────────────────────────────────────────────────────────────────
void printExperimentSummaryRow(char axis, float peak, float rms, float freq, uint16_t totalEnergy,
                                float acc, float cf, float kurt) {
  float ratio = (rms > 0.0f) ? peak / rms : 0.0f;
  Serial.printf("%c       %6.2f   %6.2f     %6.2f      %8.1f      %8u    %7.3f  %6.2f   %6.2f\n",
                axis, peak, rms, ratio, freq, totalEnergy, acc, cf, kurt);
}

// ─────────────────────────────────────────────────────────────────────────────
// printExperimentSummary()
// [v1.6] Verbose "Experiment Summary" block, printed after the existing
// detailed report on every read cycle (when SERIAL_CSV_MODE == 0). Reads
// only cached values (g_lastVelocity/g_lastSpectrum/g_lastSR/g_lastMode/
// g_lastTempC/g_lastReadCycleMs) — no Modbus transactions of its own.
// Spectrum Energy totals reflect whatever ENERGY/SR last measured (they are
// not re-read every cycle, since Spectrum Energy is intentionally NOT part
// of the automatic polling loop — see readSpectrumEnergy()'s own comment).
// ─────────────────────────────────────────────────────────────────────────────
void printExperimentSummary() {
  uint32_t nowMs = millis();

  Serial.println("======================================================================");
  Serial.println("Experiment Summary");
  Serial.println("======================================================================");
  Serial.printf("Experiment ID       : %s\n", g_experimentId.c_str());
  Serial.printf("Realtime Timestamp  : %lu ms\n", nowMs);
  if (g_lastSR < SR_TABLE_SIZE) {
    Serial.printf("Sampling Rate       : SR = %u (%s)\n", g_lastSR, SAMPLE_RATE_NAMES[g_lastSR]);
  } else {
    Serial.println("Sampling Rate       : unknown (run INFO or SR n at least once this session)");
  }
  Serial.printf("Sensor Mode         : %s\n", getModeName(g_lastMode));
  Serial.printf("Motor Setpoint      : %.1f Hz\n", g_motorSetpointHz);
  if (g_lastVelocity.tempValid) {
    Serial.printf("Sensor Temp         : %.1f C  (this cycle)\n", g_lastVelocity.tempC);
  } else if (g_lastTempValid) {
    Serial.printf("Sensor Temp         : %.1f C  (boot/readConfig, stale)\n", g_lastTempC);
  } else {
    Serial.println("Sensor Temp         : unknown (available after boot)");
  }
  // [v2.2] Config ground-truth, per-cycle -- so this always reflects THIS
  // sample's config, not a boot-time snapshot that may have silently changed.
  if (g_lastVelocity.staticDetectionValid) {
    Serial.printf("StaticDetection     : %u (%.2fx)  (this cycle)\n",
                  g_lastVelocity.staticDetectionRaw, g_lastVelocity.staticDetectionRaw / 100.0f);
  } else {
    Serial.println("StaticDetection     : READ FAILED this cycle");
  }
  if (g_lastVelocity.drmValid) {
    Serial.printf("DRM (Disp. Range)   : %u  (this cycle)\n", g_lastVelocity.drmRaw);
  } else {
    Serial.println("DRM (Disp. Range)   : READ FAILED this cycle");
  }
  Serial.printf("Read Cycle          : %lu ms\n", g_lastReadCycleMs);
  Serial.println();

  // [v1.8] Spectrum snapshot provenance — shown separately from the
  // Realtime block above, per the "don't conflate snapshot vs real-time"
  // conclusion: Peak/VRMS/Freq are live every cycle, Energy is a cached
  // snapshot from whenever ENERGY/SR n was last run. Source is always
  // "CACHE" in this firmware (Spectrum Energy is never read on the
  // automatic polling loop) — the field exists so this stays true and
  // visible rather than assumed.
  Serial.println("Spectrum Snapshot");
  Serial.println("  Source            : CACHE");
  if (g_spectrumValid) {
    uint32_t ageMs = nowMs - g_lastSpectrumTimestampMs;
    Serial.printf("  Timestamp         : %lu ms\n", g_lastSpectrumTimestampMs);
    if (ageMs > SPECTRUM_STALE_THRESHOLD_MS) {
      Serial.printf("  Age               : %lu ms (STALE)\n", ageMs);
    } else {
      Serial.printf("  Age               : %lu ms\n", ageMs);
    }
  } else {
    Serial.println("  Timestamp         : none yet (run ENERGY or SR n at least once this session)");
    Serial.println("  Age               : n/a");
  }
  Serial.println();

  Serial.println("------------------------------------------------------------------------------");
  Serial.println("Axis    Peak     VRMS     PeakRatio  Freq(Hz)   TotalEnergy      Acc(g)   CF     Kurt");
  Serial.println("------------------------------------------------------------------------------");
  printExperimentSummaryRow('X', g_lastVelocity.vx, g_lastVelocity.vrmsx, g_lastVelocity.hzx, g_lastSpectrum.axTotalEnergy,
                             g_lastVelocity.ax, g_lastVelocity.cfx, g_lastVelocity.kx);
  printExperimentSummaryRow('Y', g_lastVelocity.vy, g_lastVelocity.vrmsy, g_lastVelocity.hzy, g_lastSpectrum.ayTotalEnergy,
                             g_lastVelocity.ay, g_lastVelocity.cfy, g_lastVelocity.ky);
  printExperimentSummaryRow('Z', g_lastVelocity.vz, g_lastVelocity.vrmsz, g_lastVelocity.hzz, g_lastSpectrum.azTotalEnergy,
                             g_lastVelocity.az, g_lastVelocity.cfz, g_lastVelocity.kz);
  Serial.println("------------------------------------------------------------------------------");
  Serial.println();
  Serial.println("======================================================================");
}

// ─────────────────────────────────────────────────────────────────────────────
// printExperimentSummaryCsvHeader() / printExperimentSummaryCsvRow()
// [v1.6] CSV mode: one header line (printed once at boot) plus one data
// line per read cycle. Same cached values as printExperimentSummary() — no
// Modbus transactions, no extra text besides the CSV line itself.
// ─────────────────────────────────────────────────────────────────────────────
// [v2.5] g_sampleSeq: monotonic counter incremented once per loop() attempt,
// regardless of success/failure. Unlike Timestamp (which just keeps advancing
// smoothly even across a dropped cycle), a GAP in this sequence in the CSV
// is the only way to notice that a whole cycle was silently skipped (see
// loop(): readVelocityComparison() failing currently means NO row is written
// at all in CSV mode, with no indication anything happened).
uint32_t g_sampleSeq = 0;

void printExperimentSummaryCsvHeader() {
  Serial.println("SampleSeq,Timestamp,ExperimentID,MotorSetpointHz,SR,SamplingRate,SensorMode,StaticDetection,DRM,SoftFailFlag,Temperature,ReadCycle,"
                 "XPeak,XRMS,XPeakRmsRatio,XFreq,XEnergy,XAcc,XCF,XKurt,XScore,"
                 "YPeak,YRMS,YPeakRmsRatio,YFreq,YEnergy,YAcc,YCF,YKurt,YScore,"
                 "ZPeak,ZRMS,ZPeakRmsRatio,ZFreq,ZEnergy,ZAcc,ZCF,ZKurt,ZScore,"
                 "AnyAnomaly");
}

void printExperimentSummaryCsvRow() {
  uint32_t srHz = (g_lastSR < SR_TABLE_SIZE) ? SAMPLE_RATE_HZ[g_lastSR] : 0;
  // [v2.0] Temperature column now uses the per-cycle reading (g_lastVelocity.tempC)
  // when available, falling back to the boot/readConfig cache (g_lastTempC) only
  // if this cycle's Temp transaction failed -- so the CSV always has the freshest
  // value it can, and a single failed Temp read doesn't blank the whole column.
  float tempOut = g_lastVelocity.tempValid ? g_lastVelocity.tempC
                  : (g_lastTempValid ? g_lastTempC : 0.0f);

  // [v2.2] StaticDetection and DRM: per-cycle reading when available, else
  // fall back to -1 (not a valid register value, so it's unambiguous in the
  // CSV that this cycle's read failed) rather than silently reusing a stale
  // boot-time value that could mask a real config change mid-session.
  int staticDetectionOut = g_lastVelocity.staticDetectionValid
                            ? (int)g_lastVelocity.staticDetectionRaw : -1;
  int drmOut = g_lastVelocity.drmValid ? (int)g_lastVelocity.drmRaw : -1;

  // [v2.5] SoftFailFlag: 1 if any of the soft-fail diagnostic reads
  // (Temp / StaticDetection / DRM -- the ones that do NOT abort the whole
  // cycle on failure, see readVelocityComparison()) fell back to a stale or
  // sentinel value THIS cycle. Core registers (Peak/RMS/Freq/Accel/CF/Kurt)
  // are not part of this flag because a core failure already means this row
  // would never have been written at all -- see g_sampleSeq above for that.
  int softFailFlag = (!g_lastVelocity.tempValid
                       || !g_lastVelocity.staticDetectionValid
                       || !g_lastVelocity.drmValid) ? 1 : 0;

  // [v2.4] ExperimentID / MotorSetpoint now written into EVERY CSV row, not
  // just the verbose Experiment Summary block. These are user-typed (ID/
  // SETPOINT commands), not read from the sensor -- but without them here,
  // a CSV file collected in SERIAL_CSV_MODE has no way to tell which test
  // run or which motor speed any given row belongs to, which is exactly
  // the kind of confusion this column exists to prevent.

  // [v2.1] RFC-0004 instantaneous score, per axis -- logged only, not acted on.
  int scoreX = computeAxisScore(g_lastVelocity.vx, g_lastVelocity.vrmsx, g_lastVelocity.hzx, g_prevX);
  int scoreY = computeAxisScore(g_lastVelocity.vy, g_lastVelocity.vrmsy, g_lastVelocity.hzy, g_prevY);
  int scoreZ = computeAxisScore(g_lastVelocity.vz, g_lastVelocity.vrmsz, g_lastVelocity.hzz, g_prevZ);
  bool anyAnomaly = (scoreX >= 3) || (scoreY >= 3) || (scoreZ >= 3);

  // [v2.5] Peak/RMS ratio, per axis -- this is the SAME quantity already
  // computed inside computeAxisScore() as one of its 4 conditions, but was
  // never exposed as its own value -- only the final 0-4 score was logged.
  // Given how central this ratio has been to every anomaly investigated so
  // far (Register Dictionary §8.5, RFC-0004 G1/G2/G3), logging the raw
  // number costs nothing extra (no new Modbus transaction, pure arithmetic
  // on values already read) and saves recomputing it during analysis.
  float ratioX = (g_lastVelocity.vrmsx > 0.0f) ? g_lastVelocity.vx / g_lastVelocity.vrmsx : 0.0f;
  float ratioY = (g_lastVelocity.vrmsy > 0.0f) ? g_lastVelocity.vy / g_lastVelocity.vrmsy : 0.0f;
  float ratioZ = (g_lastVelocity.vrmsz > 0.0f) ? g_lastVelocity.vz / g_lastVelocity.vrmsz : 0.0f;

  Serial.printf(
    "%lu,%lu,%s,%.1f,%u,%lu,%s,%d,%d,%d,%.1f,%lu,"
    "%.2f,%.2f,%.3f,%.1f,%u,%.3f,%.2f,%.2f,%d,"
    "%.2f,%.2f,%.3f,%.1f,%u,%.3f,%.2f,%.2f,%d,"
    "%.2f,%.2f,%.3f,%.1f,%u,%.3f,%.2f,%.2f,%d,"
    "%d\n",
    g_sampleSeq, millis(), g_experimentId.c_str(), g_motorSetpointHz,
    g_lastSR, srHz, getModeName(g_lastMode), staticDetectionOut, drmOut, softFailFlag, tempOut, g_lastReadCycleMs,

    g_lastVelocity.vx, g_lastVelocity.vrmsx, ratioX, g_lastVelocity.hzx, g_lastSpectrum.axTotalEnergy,
    g_lastVelocity.ax, g_lastVelocity.cfx, g_lastVelocity.kx, scoreX,
    g_lastVelocity.vy, g_lastVelocity.vrmsy, ratioY, g_lastVelocity.hzy, g_lastSpectrum.ayTotalEnergy,
    g_lastVelocity.ay, g_lastVelocity.cfy, g_lastVelocity.ky, scoreY,
    g_lastVelocity.vz, g_lastVelocity.vrmsz, ratioZ, g_lastVelocity.hzz, g_lastSpectrum.azTotalEnergy,
    g_lastVelocity.az, g_lastVelocity.cfz, g_lastVelocity.kz, scoreZ,
    anyAnomaly ? 1 : 0
  );
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
//   ID <text>       -> [v1.7] set Experiment ID shown in Experiment Summary (no Modbus)
//   SETPOINT <num>  -> [v1.7] set Motor Setpoint (Hz) shown in Experiment Summary (no Modbus)
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
          g_lastSpectrumTimestampMs = millis();   // [v1.8] record when this snapshot was taken
          g_spectrumValid = true;
          printSpectrumEnergyReport(spectrum);
        } else {
          Serial.println("[ERROR] Spectrum Energy read failed.");
        }
      } else if (cmd == "INFO") {
        printInfoReport();
      } else if (cmd == "SUMMARY") {
        printSummaryReport();
      } else if (cmd.startsWith("ID ")) {
        g_experimentId = cmd.substring(3);
        g_experimentId.trim();
        Serial.printf("[CMD] Experiment ID set to: %s\n", g_experimentId.c_str());
      } else if (cmd.startsWith("SETPOINT ")) {
        g_motorSetpointHz = cmd.substring(9).toFloat();
        Serial.printf("[CMD] Motor Setpoint set to: %.1f Hz\n", g_motorSetpointHz);
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
  // [v2.3] Printed once here instead of every cycle in printVelocityReport() --
  // it's a fixed reference constant (sqrt(2)), not measured data.
  Serial.println(" Reference: Peak/RMS ~= 1.414   RMS/Peak ~= 0.707");
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

#if SPECTRUM_READ_AT_BOOT
  // [v1.7] Opt-in: one extra Modbus round trip at boot only, so
  // TotalEnergy is already populated for the very first Experiment
  // Summary/CSV row instead of reading 0 until ENERGY/SR n is run.
  Serial.println("[INIT] Reading Spectrum Energy once at boot (SPECTRUM_READ_AT_BOOT=1)...");
  if (readSpectrumEnergy(g_lastSpectrum)) {
    g_lastSpectrumTimestampMs = millis();   // [v1.8] record when this snapshot was taken
    g_spectrumValid = true;
    Serial.println("[INIT] Spectrum Energy cache populated.");
  } else {
    Serial.println("[INIT] Spectrum Energy read failed — cache stays at 0 until ENERGY/SR n.");
  }
#endif

  Serial.println("[INIT] Starting continuous monitoring.\n");

#if SERIAL_CSV_MODE
  printExperimentSummaryCsvHeader();   // [v1.6] one-time header for CSV capture tools
#endif
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
    g_sampleSeq++;  // [v2.5] increment on every ATTEMPT, success or fail --
                     // a gap in this number in the CSV is now the signal
                     // that a whole cycle was silently dropped.

    uint32_t t0 = millis();
    VelocityData data;
    bool ok = readVelocityComparison(data);
    uint32_t elapsed = millis() - t0;
    g_lastReadCycleMs = elapsed;   // [v1.6] cache for Experiment Summary/CSV — no extra work

#if !SERIAL_CSV_MODE
    Serial.printf("[TIMING] ReadCycle=%lu ms\n", elapsed);
#endif
    recordTiming(elapsed);

    if (ok) {
      g_lastVelocity = data;   // [v1.5] cache for SUMMARY — no extra Modbus read
#if SERIAL_CSV_MODE
      printExperimentSummaryCsvRow();   // ONE CSV line only, per requirement
#else
      printVelocityReport(data);        // unchanged existing report
      printExperimentSummary();         // [v1.6] new compact block, after existing output
#endif
    } else {
#if !SERIAL_CSV_MODE
      Serial.println("[WARN] Read cycle failed – will retry next interval.");
#endif
    }
  }

  // Print rolling Min/Avg/P95/Max every 30 s to build a statistical picture
  // of real transaction time (per DESIGN_PRINCIPLES.md: facts before assumptions)
  if (millis() - lastStatsPrint >= 30000) {
    lastStatsPrint = millis();
#if !SERIAL_CSV_MODE
    printTimingStats();
#endif
  }

#if SPECTRUM_AUTO_REFRESH
  // [v1.9] Independent timer, decoupled from READ_INTERVAL_MS/lastRead above —
  // keeps Spectrum Snapshot / TotalEnergy from silently going STALE forever
  // during long unattended runs. Same "own timer, no main-loop impact"
  // pattern as the timing-stats block just above.
  static unsigned long lastSpectrumRefresh = 0;
  if (millis() - lastSpectrumRefresh >= SPECTRUM_REFRESH_INTERVAL_MS) {
    lastSpectrumRefresh = millis();
    if (!refreshSpectrumEnergy()) {
#if !SERIAL_CSV_MODE
      Serial.println("[WARN] Auto Spectrum Energy refresh failed – will retry next interval.");
#endif
    }
  }
#endif
}
