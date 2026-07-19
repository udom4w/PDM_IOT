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
enum class FIFOFrameResult : int;
struct FIFOStressRunResult;   // [v3.9] same reason as the others above --
                               // Arduino auto-generates a prototype for
                               // runFIFOStressOnce() (which returns this
                               // struct) and inserts it near the top of
                               // the file, before the struct's real
                               // definition further down. Without this
                               // forward declaration that prototype fails
                               // to compile with "does not name a type."
struct TruePollRunResult;     // [v3.11] same reason, for runFIFOTruePollOnce().

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

// ═════════════════════════════════════════════════════════════════════════
// [v3.0] FIFO raw-acceleration capture — implements RFC-0006 §5.
//
// This does NOT go through ModbusMaster's readHoldingRegisters(). Two
// independent reasons make that impossible for this specific register:
//   1. ModbusMaster's internal response buffer (ku8MaxBufferSize, commonly
//      64 registers = 128 bytes in the stock library) is far smaller than
//      the 6144-byte FIFO payload -- it would silently overflow/truncate.
//   2. The FIFO response does not use standard Modbus semantics: the
//      "length" byte reads literally 0x00 or 0x01 regardless of the true
//      6144-byte payload size (see the datasheet's own documented examples,
//      cross-validated against real captured frames in this project's
//      earlier data_0.bin analysis). A standard Modbus client would
//      misinterpret this field.
// This module talks to SerialRS485 directly, with its own CRC16 and its own
// framing logic matching exactly what was reverse-engineered from real
// device captures earlier in this project.
// ═════════════════════════════════════════════════════════════════════════

#define REG_RAWFIFO         0x002C
#define FIFO_SAMPLE_COUNT   1024
#define FIFO_DATA_BYTES     6144   // 1024 samples * 3 axes * 2 bytes

// Standard Modbus CRC16 (poly 0xA001, init 0xFFFF) -- same algorithm used
// throughout this project's protocol reverse-engineering (data_0.bin).
uint16_t crc16_modbus_calc(const uint8_t* buf, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= buf[i];
    for (uint8_t b = 0; b < 8; b++) {
      if (crc & 0x0001) crc = (crc >> 1) ^ 0xA001;
      else               crc = (crc >> 1);
    }
  }
  return crc;
}

// Raw tri-axial sample buffer -- module-level so it doesn't need 6KB of
// stack space in whatever function calls the reader.
static int16_t g_fifoX[FIFO_SAMPLE_COUNT];
static int16_t g_fifoY[FIFO_SAMPLE_COUNT];
static int16_t g_fifoZ[FIFO_SAMPLE_COUNT];

// ─────────────────────────────────────────────────────────────────────────
// [v3.5] fifoPassiveListen() -- a FALSIFIABLE experiment, not a confirmation.
//
// Per review: CRC-matching the "unsolicited" bytes proved they are a real,
// non-corrupt frame -- it did NOT prove the sensor streams autonomously.
// An equally consistent explanation is that OUR OWN resync loop re-sends a
// new request before the sensor's normal response to the PREVIOUS request
// has finished arriving, truncating it -- producing the exact same
// "6 bytes + 1 stray byte" pattern.
//
// This function sends the FIFO read command exactly ONCE and then only
// listens -- it never re-sends. Frame boundaries are inferred from timing
// gaps (a new "chunk" starts whenever more than chunkGapMs elapses with no
// bytes), not from assuming any particular frame length, so this makes no
// assumption about what shape the data will arrive in.
//
//   - If multiple complete frames keep arriving on their own, one after
//     another, without us ever asking again -> supports autonomous
//     streaming.
//   - If exactly one frame arrives and then the line goes silent for the
//     rest of the listen window -> supports "the sensor only responds
//     when asked," and the earlier "6+1" pattern is better explained by
//     our own re-poll timing cutting off a real response.
// ─────────────────────────────────────────────────────────────────────────
void fifoPassiveListen(uint32_t totalListenMs = 15000, uint32_t chunkGapMs = 50) {
  uint8_t cmd[8];
  cmd[0] = MODBUS_SLAVE_ID;
  cmd[1] = 0x03;
  cmd[2] = (uint8_t)(REG_RAWFIFO >> 8);
  cmd[3] = (uint8_t)(REG_RAWFIFO & 0xFF);
  cmd[4] = 0x00;
  cmd[5] = 0x01;
  uint16_t cmdCrc = crc16_modbus_calc(cmd, 6);
  cmd[6] = (uint8_t)(cmdCrc & 0xFF);
  cmd[7] = (uint8_t)((cmdCrc >> 8) & 0xFF);

  while (SerialRS485.available()) SerialRS485.read();  // start clean
  Serial.println("[FIFO-LISTEN] Sending ONE request only. No further requests will be sent for the rest of this capture.");
  uint32_t tSend = millis();
  SerialRS485.write(cmd, 8);
  SerialRS485.flush();
  Serial.printf("[FIFO-LISTEN][T=%lu ms] Request sent.\n", (unsigned long)tSend);

  const int CHUNK_BUF_SIZE = 512;
  static uint8_t chunkBuf[CHUNK_BUF_SIZE];
  int chunkLen = 0;
  uint32_t chunkStartTime = 0;
  bool chunkTruncated = false;

  uint32_t listenStart = millis();
  uint32_t lastByteTime = 0;
  bool haveLastByte = false;
  uint32_t totalBytes = 0;
  int chunkNum = 0;

  while (millis() - listenStart < totalListenMs) {
    if (SerialRS485.available()) {
      uint32_t now = millis();
      if (haveLastByte && (now - lastByteTime) > chunkGapMs && chunkLen > 0) {
        // Gap detected -> the previous chunk has ended; print it.
        chunkNum++;
        Serial.printf("[FIFO-LISTEN][T=%lu ms] Chunk #%d (%d bytes%s): ",
                      (unsigned long)chunkStartTime, chunkNum, chunkLen,
                      chunkTruncated ? ", TRUNCATED at buffer limit" : "");
        for (int i = 0; i < chunkLen; i++) Serial.printf("%02X ", chunkBuf[i]);
        Serial.println();
        chunkLen = 0;
        chunkTruncated = false;
      }
      if (chunkLen == 0) chunkStartTime = now;
      uint8_t b = SerialRS485.read();
      if (chunkLen < CHUNK_BUF_SIZE) {
        chunkBuf[chunkLen++] = b;
      } else {
        chunkTruncated = true;   // still consume the byte, just stop storing it
      }
      lastByteTime = now;
      haveLastByte = true;
      totalBytes++;
    }
  }
  // Flush any trailing chunk still in progress when the listen window ends.
  if (chunkLen > 0) {
    chunkNum++;
    Serial.printf("[FIFO-LISTEN][T=%lu ms] Chunk #%d (%d bytes%s, window ended mid-chunk): ",
                  (unsigned long)chunkStartTime, chunkNum, chunkLen,
                  chunkTruncated ? ", TRUNCATED at buffer limit" : "");
    for (int i = 0; i < chunkLen; i++) Serial.printf("%02X ", chunkBuf[i]);
    Serial.println();
  }

  Serial.printf("[FIFO-LISTEN] Done. Listened for %lu ms, received %lu total byte(s) across %d chunk(s). "
                "Only ONE request was ever sent (at T=%lu ms).\n",
                (unsigned long)totalListenMs, (unsigned long)totalBytes, chunkNum, (unsigned long)tSend);
  if (chunkNum <= 1) {
    Serial.println("[FIFO-LISTEN] Interpretation: at most one chunk seen -> consistent with "
                    "\"sensor only responds when asked\" (supports re-poll-timing explanation, "
                    "not autonomous streaming).");
  } else {
    Serial.println("[FIFO-LISTEN] Interpretation: multiple chunks arrived without any further "
                    "request from us -> consistent with autonomous streaming.");
  }
}

// Sends the documented FIFO read command (50 03 00 2C 00 01 CRCL CRCH) and
// handles BOTH possible response shapes:
//   - 7-byte "still filling" progress frame  (3rd byte == 0x01)
//   - 6149-byte full dump                     (3rd byte == 0x00)
// Retries the progress-frame case until the full dump arrives or timeoutMs
// elapses. Returns true and fills g_fifoX/Y/Z on success.

// ─────────────────────────────────────────────────────────────────────────
// [v3.6] Shared frame reader, factored out so the hybrid strategy below
// doesn't duplicate the anchor-scan/decode logic three times. Waits for
// the "50 03" anchor (discarding any leading stray bytes, same approach
// validated in v3.2), then branches on the Len byte and decodes whichever
// frame type actually arrived. Does NOT send anything -- purely receive-side.
// ─────────────────────────────────────────────────────────────────────────
enum class FIFOFrameResult : int { PROGRESS, FULL_DUMP, TIMEOUT, CRC_ERROR };

FIFOFrameResult readOneFIFOFrame(uint32_t silenceTimeoutMs, uint16_t &outProgress, uint32_t *outAnchorMs) {
  uint8_t prevByte = 0x00;
  bool havePrev = false;
  uint32_t lastActivity = millis();

  while (true) {
    if (millis() - lastActivity > silenceTimeoutMs) return FIFOFrameResult::TIMEOUT;
    if (!SerialRS485.available()) continue;
    uint8_t curByte = SerialRS485.read();
    lastActivity = millis();
    if (havePrev && prevByte == MODBUS_SLAVE_ID && curByte == 0x03) break;  // anchored
    prevByte = curByte;
    havePrev = true;
  }

  // [v3.7] TIMELINE instrumentation -- Tanchor. This is the missing
  // timestamp identified in review: it marks the instant the '50 03'
  // anchor bytes were found, i.e. the moment readOneFIFOFrame() stops
  // "waiting for the next frame to start" and starts "receiving a frame
  // we already know exists." Printed for EVERY frame (progress or full
  // dump), so the delta between one frame's report and the NEXT anchor
  // line below it is exactly the "Progress -> waiting for first byte of
  // next frame" gap the review wants measured -- no guessing required.
  uint32_t tAnchor = millis();
  Serial.printf("[FIFO-TIMELINE][T=%lu ms] Anchor '50 03' found.\n", (unsigned long)tAnchor);
  if (outAnchorMs) *outAnchorMs = tAnchor;

  // Read Len byte.
  uint32_t waitStart = millis();
  while (!SerialRS485.available()) {
    if (millis() - waitStart > silenceTimeoutMs) return FIFOFrameResult::TIMEOUT;
  }
  uint8_t lenByte = SerialRS485.read();

  if (lenByte == 0x01) {
    uint8_t rest[4];
    uint32_t got = 0;
    waitStart = millis();
    while (got < 4) {
      if (SerialRS485.available()) { rest[got++] = SerialRS485.read(); waitStart = millis(); }
      else if (millis() - waitStart > silenceTimeoutMs) return FIFOFrameResult::TIMEOUT;
    }
    uint8_t frame[5] = { MODBUS_SLAVE_ID, 0x03, 0x01, rest[0], rest[1] };
    uint16_t calc = crc16_modbus_calc(frame, 5);
    uint16_t sent = ((uint16_t)rest[3] << 8) | rest[2];
    if (calc != sent) return FIFOFrameResult::CRC_ERROR;
    outProgress = ((uint16_t)rest[0] << 8) | rest[1];
    return FIFOFrameResult::PROGRESS;

  } else if (lenByte == 0x00) {
    static uint8_t dataBuf[FIFO_DATA_BYTES];
    uint8_t crcBuf[2];
    uint32_t need = FIFO_DATA_BYTES + 2;
    uint32_t got = 0;
    waitStart = millis();
    while (got < need) {
      if (SerialRS485.available()) {
        uint8_t b = SerialRS485.read();
        if (got < FIFO_DATA_BYTES) dataBuf[got] = b; else crcBuf[got - FIFO_DATA_BYTES] = b;
        got++;
        waitStart = millis();
      } else if (millis() - waitStart > 4000) {
        return FIFOFrameResult::TIMEOUT;
      }
    }
    uint8_t hdr3[3] = { MODBUS_SLAVE_ID, 0x03, 0x00 };
    uint8_t* fullFrame = (uint8_t*)malloc(3 + FIFO_DATA_BYTES);
    if (!fullFrame) return FIFOFrameResult::CRC_ERROR;
    memcpy(fullFrame, hdr3, 3);
    memcpy(fullFrame + 3, dataBuf, FIFO_DATA_BYTES);
    uint16_t calc = crc16_modbus_calc(fullFrame, 3 + FIFO_DATA_BYTES);
    free(fullFrame);
    uint16_t sent = ((uint16_t)crcBuf[1] << 8) | crcBuf[0];
    if (calc != sent) return FIFOFrameResult::CRC_ERROR;

    for (int i = 0; i < FIFO_SAMPLE_COUNT; i++) {
      int off = i * 6;
      g_fifoX[i] = (int16_t)((dataBuf[off + 0] << 8) | dataBuf[off + 1]);
      g_fifoY[i] = (int16_t)((dataBuf[off + 2] << 8) | dataBuf[off + 3]);
      g_fifoZ[i] = (int16_t)((dataBuf[off + 4] << 8) | dataBuf[off + 5]);
    }
    return FIFOFrameResult::FULL_DUMP;
  }
  return FIFOFrameResult::CRC_ERROR;   // unrecognized Len byte
}

void sendFIFORequestOnce() {
  uint8_t cmd[8];
  cmd[0] = MODBUS_SLAVE_ID; cmd[1] = 0x03;
  cmd[2] = (uint8_t)(REG_RAWFIFO >> 8); cmd[3] = (uint8_t)(REG_RAWFIFO & 0xFF);
  cmd[4] = 0x00; cmd[5] = 0x01;
  uint16_t crc = crc16_modbus_calc(cmd, 6);
  cmd[6] = (uint8_t)(crc & 0xFF); cmd[7] = (uint8_t)((crc >> 8) & 0xFF);
  while (SerialRS485.available()) SerialRS485.read();
  SerialRS485.write(cmd, 8);
  SerialRS485.flush();
}

// ─────────────────────────────────────────────────────────────────────────
// [v3.6] readFIFOHybrid() -- built directly from the v3.5 LISTEN finding:
// progress-frames arrive autonomously (~every 250ms, CRC-verified, 4-for-4
// in that experiment) but the full dump did NOT arrive on its own even
// after progress reached 6117/6144 (99.6%) and 14+ seconds of silence
// followed. This function reflects that two-part reality instead of
// re-testing either extreme in isolation:
//
//   Phase 1: send ONE request, then only listen (like v3.5) and decode
//            whatever arrives -- if the full dump shows up on its own,
//            great, done, no Phase 2 needed.
//   Phase 2: only entered if Phase 1's autonomous stream goes quiet for
//            `stallMs` after progress was already seen close to full --
//            sends exactly ONE follow-up request specifically to ask for
//            the dump, then waits for it.
//
// This is still a hypothesis about the RIGHT way to get the dump -- not a
// confirmed protocol. Log everything so a failure here is diagnosable too.
// ─────────────────────────────────────────────────────────────────────────
// ─────────────────────────────────────────────────────────────────────────
// [v3.7] printFIFOTimeline() -- prints the reconstructed timing model for
// one HYBRID capture attempt: Treq / TlastProgress / Ttimeout / Tphase2Req /
// Tcomplete, plus the deltas between them. This was requested in review
// specifically to distinguish "Host waited too long before Sensor even
// started the dump" (a stallMs problem) from "Sensor was mid-transmission
// when Phase 2 fired" (a state-machine collision, not a timeout problem) --
// see the per-frame [FIFO-TIMELINE] Anchor lines above this summary for the
// finer-grained Progress->Anchor / Anchor->Complete deltas within each
// individual frame read.
// Purely additive -- does not change any timeout, retry, or control-flow
// decision anywhere in readFIFOHybrid()/readOneFIFOFrame().
// ─────────────────────────────────────────────────────────────────────────
void printFIFOTimeline(uint32_t tReq, uint32_t tLastProg, bool haveProg,
                        uint32_t tTimeout, bool hadTimeout,
                        uint32_t tPhase2, bool sentPhase2,
                        uint32_t tComplete, bool completed) {
  Serial.println("[FIFO-TIMELINE] ---- Run summary ----");
  Serial.printf("[FIFO-TIMELINE]   Treq          = %lu ms\n", (unsigned long)tReq);
  if (haveProg) {
    Serial.printf("[FIFO-TIMELINE]   TlastProgress = %lu ms  (+%lu ms since Treq)\n",
                  (unsigned long)tLastProg, (unsigned long)(tLastProg - tReq));
  } else {
    Serial.println("[FIFO-TIMELINE]   TlastProgress = none (no progress frame seen -- e.g. very high SR)");
  }
  if (hadTimeout) {
    uint32_t base = haveProg ? tLastProg : tReq;
    Serial.printf("[FIFO-TIMELINE]   Ttimeout      = %lu ms  (+%lu ms since %s)\n",
                  (unsigned long)tTimeout, (unsigned long)(tTimeout - base),
                  haveProg ? "TlastProgress" : "Treq");
  }
  if (sentPhase2) {
    Serial.printf("[FIFO-TIMELINE]   Tphase2Req    = %lu ms  (+%lu ms since Ttimeout)\n",
                  (unsigned long)tPhase2, (unsigned long)(tPhase2 - tTimeout));
  }
  if (completed) {
    uint32_t base = sentPhase2 ? tPhase2 : (haveProg ? tLastProg : tReq);
    const char* baseName = sentPhase2 ? "Tphase2Req" : (haveProg ? "TlastProgress" : "Treq");
    Serial.printf("[FIFO-TIMELINE]   Tcomplete     = %lu ms  (+%lu ms since %s)\n",
                  (unsigned long)tComplete, (unsigned long)(tComplete - base), baseName);
  } else {
    Serial.println("[FIFO-TIMELINE]   Tcomplete     = NEVER (attempt failed)");
  }
  Serial.println("[FIFO-TIMELINE] ---------------------");
}

// ─────────────────────────────────────────────────────────────────────────
// [v3.8] readFIFOPureListen() -- Test A/B, per review of the v3.7 timeline
// data. The concern raised: readFIFOHybrid()'s Phase 2 retry and the
// autonomous-completion question were entangled -- a failed HYBRID run only
// proves "no bytes arrived for stallMs", NOT "the sensor would never have
// finished on its own." This function isolates that single question:
//
//   Send the request EXACTLY ONCE. Then listen, with a much longer
//   per-frame silence window (stallMs, default 10000 ms instead of
//   HYBRID's 2000 ms). If that window elapses with nothing arriving,
//   this is treated as a genuine, final failure -- NO follow-up request
//   is EVER sent, under any circumstance, regardless of how much
//   progress was last seen. That is the entire point of this function:
//   it cannot itself cause a state-machine collision, because it never
//   sends a second command.
//
//   - If the full dump arrives -> proves the sensor DOES complete the
//     transfer autonomously, and the earlier HYBRID failures were purely
//     a stallMs-too-short problem (Test A confirms; safe to raise stallMs
//     in HYBRID, or better, make it adaptive per the review's H1/H2/H3
//     split -- see doc comment above).
//   - If it does NOT arrive even after stallMs -> the autonomous-only
//     hypothesis is falsified for this run; the earlier "solved by more
//     patience" story is wrong, and the sensor's internal state machine
//     (not just link timing) needs investigating instead.
//
// This is Test A. Test B (does a Phase-2-style follow-up help AFTER a
// genuine PURELISTEN failure) is just running HYBRID afterward as a
// separate, explicit step -- deliberately NOT automated here, so the two
// questions stay cleanly separated in the log instead of one function
// silently answering both.
// ─────────────────────────────────────────────────────────────────────────
bool readFIFOPureListen(uint32_t stallMs = 10000, uint32_t maxTotalMs = 30000) {
  Serial.println("[FIFO-PURELISTEN] Sending ONE request, then listening ONLY. "
                  "No follow-up request will be sent under any circumstance in this function.");
  sendFIFORequestOnce();
  uint32_t tSend = millis();
  Serial.printf("[FIFO-PURELISTEN][T=%lu ms] Request sent.\n", (unsigned long)tSend);

  uint32_t tLastProgress = 0;
  bool haveLastProgress = false;

  uint32_t phaseStart = millis();
  while (millis() - phaseStart < maxTotalMs) {
    uint16_t progress = 0;
    FIFOFrameResult r = readOneFIFOFrame(stallMs, progress, nullptr);

    if (r == FIFOFrameResult::FULL_DUMP) {
      uint32_t tComplete = millis();
      Serial.printf("[FIFO-PURELISTEN][T=%lu ms] Full dump arrived with ZERO follow-up requests sent. CRC verified.\n",
                    (unsigned long)tComplete);
      if (haveLastProgress) {
        Serial.printf("[FIFO-PURELISTEN]   (+%lu ms since last progress frame -- no request sent in that gap)\n",
                      (unsigned long)(tComplete - tLastProgress));
      }
      Serial.println("[FIFO-PURELISTEN] RESULT: Test A CONFIRMS autonomous completion -- sensor finishes on its own given enough time.");
      return true;

    } else if (r == FIFOFrameResult::PROGRESS) {
      tLastProgress = millis();
      haveLastProgress = true;
      Serial.printf("[FIFO-PURELISTEN][T=%lu ms] Progress frame: %u/%u bytes (%.1f%%)\n",
                    (unsigned long)tLastProgress, progress, (unsigned)FIFO_DATA_BYTES,
                    100.0f * progress / FIFO_DATA_BYTES);

    } else if (r == FIFOFrameResult::CRC_ERROR) {
      Serial.println("[FIFO-PURELISTEN] CRC error on a frame -- ignoring, continuing to listen (still no request sent).");

    } else {  // TIMEOUT -- per this function's whole design, this is FINAL, not a retry cue
      Serial.printf("[FIFO-PURELISTEN][T=%lu ms] Silent for %lu ms with NOTHING sent from us since the original request.\n",
                    (unsigned long)millis(), (unsigned long)stallMs);
      Serial.println("[FIFO-PURELISTEN] RESULT: Test A FALSIFIES autonomous completion (within this window) -- "
                      "the earlier HYBRID failures are NOT purely a stallMs problem. Investigate sensor-side state instead.");
      return false;
    }
  }

  Serial.printf("[FIFO-PURELISTEN][T=%lu ms] Overall %lu ms window elapsed, no full dump, no request ever sent.\n",
                (unsigned long)millis(), (unsigned long)maxTotalMs);
  Serial.println("[FIFO-PURELISTEN] RESULT: inconclusive within maxTotalMs -- consider raising maxTotalMs, not stallMs, to re-test.");
  return false;
}

// ─────────────────────────────────────────────────────────────────────────
// [v3.9] Stress-test / state-machine-mapping harness -- per review, the
// goal has shifted from "find a timeout value that works" to "map the
// sensor's own state machine statistically." Terminology note, also per
// review: what earlier code called a "timeout" is a HOST-side observation
// (no bytes arrived). Whether the SENSOR itself is stuck, recovering, or
// just slow is the open question -- so failures here are labeled
// STALLED_DUMP_STATE, not "timeout," to keep host-observation and
// sensor-behavior questions visibly separate in the data.
//
// FIFOStressRunResult captures one PURELISTEN-style run's full timeline:
//   Treq        -- request sent (host action, the only one this harness
//                  ever takes -- exactly like PURELISTEN, no retries ever)
//   T49 / T99   -- first progress frame seen, and the LAST progress frame
//                  seen (empirically ~49% and ~99% in this project's runs
//                  so far, but recorded as whatever actually occurs, in
//                  case the sensor's progress steps ever differ)
//   TanchorNext -- anchor time of the frame immediately following the
//                  last progress frame -- this is the direct Gap1
//                  measurement (T99 -> TanchorNext) the review asked for,
//                  captured in-band instead of by hand-reading logs
//   Tcomplete   -- full dump arrival time (0 / haveTcomplete=false if the
//                  run never completed within maxTotalMs)
//   success     -- true only on full dump + CRC verified
//   failureReason -- "STALLED_DUMP_STATE" (no bytes at all for stallMs) or
//                  "MAXTOTAL_ELAPSED" (kept receiving *something* -- e.g.
//                  repeated progress/CRC-error frames -- but never the
//                  full dump before the overall budget ran out)
// ─────────────────────────────────────────────────────────────────────────
struct FIFOStressRunResult {
  uint32_t treq;
  uint32_t t49;
  bool     haveT49;
  uint32_t t99;
  bool     haveT99;
  uint32_t tAnchorNext;
  bool     haveAnchorNext;
  uint32_t tComplete;
  bool     success;
  char     failureReason[32];
};

FIFOStressRunResult runFIFOStressOnce(uint32_t stallMs, uint32_t maxTotalMs) {
  FIFOStressRunResult r;
  memset(&r, 0, sizeof(r));

  sendFIFORequestOnce();
  r.treq = millis();

  uint32_t phaseStart = millis();

  while (millis() - phaseStart < maxTotalMs) {
    uint16_t progress = 0;
    uint32_t anchorMs = 0;
    FIFOFrameResult fr = readOneFIFOFrame(stallMs, progress, &anchorMs);

    if (fr == FIFOFrameResult::FULL_DUMP) {
      // [v3.10 fix] Capture Gap1 here, at the frame that actually follows
      // the last progress frame -- NOT via a flag that could be
      // (re-)armed and consumed within the same PROGRESS iteration that
      // set T99 itself. The earlier version of this function did that,
      // which made TanchorNext echo back the 99% frame's OWN anchor on
      // every STALLED_DUMP_STATE run (visible as TanchorNext < T99 in
      // the v3.9 stress-test data -- impossible for a genuine "next"
      // frame, and the tell that gave the bug away).
      if (r.haveT99 && !r.haveAnchorNext) {
        r.tAnchorNext = anchorMs;
        r.haveAnchorNext = true;
      }
      r.tComplete = millis();
      r.success = true;
      return r;

    } else if (fr == FIFOFrameResult::PROGRESS) {
      // A new progress frame supersedes any previously captured
      // "next anchor" candidate -- Gap1 must be measured relative to the
      // MOST RECENT progress event, not a stale one from earlier in this
      // same run.
      r.haveAnchorNext = false;
      uint32_t now = millis();
      if (!r.haveT49) { r.t49 = now; r.haveT49 = true; }
      r.t99 = now;
      r.haveT99 = true;

    } else if (fr == FIFOFrameResult::CRC_ERROR) {
      // Keep listening -- same policy as PURELISTEN/HYBRID. A CRC-error
      // frame still had a real anchor, so it counts as "something arrived
      // after the last progress" for Gap1 purposes (first one only).
      if (r.haveT99 && !r.haveAnchorNext) {
        r.tAnchorNext = anchorMs;
        r.haveAnchorNext = true;
      }

    } else {  // TIMEOUT -- i.e. STALLED_DUMP_STATE (sensor-side question, not a host artifact)
      // No anchor arrived at all -- haveAnchorNext correctly stays false
      // (or reflects whatever CRC-error anchor, if any, preceded this
      // timeout), which is the honest answer: "nothing came."
      snprintf(r.failureReason, sizeof(r.failureReason), "STALLED_DUMP_STATE");
      return r;
    }
  }

  snprintf(r.failureReason, sizeof(r.failureReason), "MAXTOTAL_ELAPSED");
  return r;
}

// ─────────────────────────────────────────────────────────────────────────
// [v3.9] runFIFOStressLoop() -- runs runFIFOStressOnce() numRuns times back
// to back, printing one CSV row per run (RunID/SR/TempC/all five timestamps/
// Success/FailureReason) plus a summary at the end. This is deliberately a
// pure measurement tool: it never adapts stallMs, never retries beyond what
// runFIFOStressOnce() already does (nothing), and never interprets a single
// run's result -- per review, the goal right now is a statistical picture
// across many runs, not a fix.
// ─────────────────────────────────────────────────────────────────────────
void runFIFOStressLoop(int numRuns, uint32_t stallMs, uint32_t maxTotalMs) {
  Serial.printf("[STRESS] Starting %d run(s), stallMs=%lu, maxTotalMs=%lu...\n",
                numRuns, (unsigned long)stallMs, (unsigned long)maxTotalMs);
  Serial.println("[STRESS-CSV] RunID,SR,SRHz,TempC,Treq,T49,T99,TanchorNext,Tcomplete,Success,FailureReason");

  int successCount = 0;
  int failCount = 0;

  for (int i = 1; i <= numRuns; i++) {
    FIFOStressRunResult r = runFIFOStressOnce(stallMs, maxTotalMs);

    uint32_t srHz = (g_lastSR < SR_TABLE_SIZE) ? SAMPLE_RATE_HZ[g_lastSR] : 0;
    float tempOut = g_lastVelocity.tempValid ? g_lastVelocity.tempC
                    : (g_lastTempValid ? g_lastTempC : 0.0f);

    Serial.printf("[STRESS-CSV] %d,%u,%lu,%.1f,%lu,%lu,%lu,%lu,%lu,%d,%s\n",
                  i, g_lastSR, (unsigned long)srHz, tempOut,
                  (unsigned long)r.treq,
                  (unsigned long)(r.haveT49 ? r.t49 : 0),
                  (unsigned long)(r.haveT99 ? r.t99 : 0),
                  (unsigned long)(r.haveAnchorNext ? r.tAnchorNext : 0),
                  (unsigned long)(r.success ? r.tComplete : 0),
                  r.success ? 1 : 0,
                  r.success ? "" : r.failureReason);

    if (r.success) successCount++; else failCount++;

    // Bus-settle drain between runs -- same discipline as every other FIFO
    // command in this file, so one run's leftover bytes can't bleed into
    // the next run's Treq.
    {
      uint32_t quietStart = millis();
      while (millis() - quietStart < 300) {
        if (SerialRS485.available()) { SerialRS485.read(); quietStart = millis(); }
      }
    }
  }

  Serial.println("[STRESS] ---- Summary ----");
  Serial.printf("[STRESS]   Runs        = %d\n", numRuns);
  Serial.printf("[STRESS]   Success     = %d (%.1f%%)\n", successCount, 100.0f * successCount / numRuns);
  Serial.printf("[STRESS]   Fail        = %d (%.1f%%)\n", failCount, 100.0f * failCount / numRuns);
  Serial.println("[STRESS] ------------------");
}

// ─────────────────────────────────────────────────────────────────────────
// [v3.11] TRUEPOLL -- tests the datasheet-literal interpretation of the
// RAWFIFO protocol (WTVB05 Data Sheet & User Manual, section 6.1.4.16):
// the host re-sends the SAME read command ("50 03 00 2C 00 01 <CRC>")
// repeatedly; each response is documented as EITHER "50 03 01 <len_so_far>"
// (still filling) OR "50 03 00 <6144 bytes>" (ready) -- i.e. every response
// is described as an answer to an explicit request. Nothing in that
// section describes the sensor pushing frames on its own between
// requests. This is a DIFFERENT protocol model than HYBRID/PURELISTEN/
// STRESSTEST, which all send ONE request and then only listen.
//
// This is deliberately a PARALLEL, INDEPENDENT experiment -- it does not
// replace HYBRID or STRESSTEST. The comparison between their success
// rates is the actual point: if TRUEPOLL is reliably better, the single-
// request/passive-listen mental model used everywhere else in this file
// was wrong for this operation. If TRUEPOLL fails at a similar rate, the
// problem is not our protocol interpretation -- it is the sensor's
// FIFO/dump state machine itself, even when driven exactly as documented.
//
// Per-poll logging (Poll#/response-type/progress) is recorded so the
// state-transition sequence is visible directly, not just a final
// success/fail -- e.g. does progress ever go backwards across polls, does
// it plateau at 99% for many polls before completing, etc.
// ─────────────────────────────────────────────────────────────────────────
struct TruePollRunResult {
  bool     success;
  int      pollCount;
  uint32_t treq;        // first poll's request-sent time
  uint32_t tComplete;   // 0 if never completed
  char     failureReason[32];
};

TruePollRunResult runFIFOTruePollOnce(uint32_t perPollTimeoutMs, uint32_t pollIntervalMs, int maxPolls) {
  TruePollRunResult r;
  memset(&r, 0, sizeof(r));

  for (int pollNum = 1; pollNum <= maxPolls; pollNum++) {
    sendFIFORequestOnce();
    uint32_t tSend = millis();
    if (pollNum == 1) r.treq = tSend;
    Serial.printf("[TRUEPOLL][Poll#%d][T=%lu ms] Request sent.\n", pollNum, (unsigned long)tSend);

    uint16_t progress = 0;
    uint32_t anchorMs = 0;
    FIFOFrameResult fr = readOneFIFOFrame(perPollTimeoutMs, progress, &anchorMs);

    if (fr == FIFOFrameResult::FULL_DUMP) {
      uint32_t tComplete = millis();
      Serial.printf("[TRUEPOLL][Poll#%d][T=%lu ms] FULL_DUMP received. CRC verified.\n",
                    pollNum, (unsigned long)tComplete);
      r.success = true;
      r.pollCount = pollNum;
      r.tComplete = tComplete;
      return r;

    } else if (fr == FIFOFrameResult::PROGRESS) {
      Serial.printf("[TRUEPOLL][Poll#%d][T=%lu ms] PROGRESS response: %u/%u bytes (%.1f%%)\n",
                    pollNum, (unsigned long)millis(), progress, (unsigned)FIFO_DATA_BYTES,
                    100.0f * progress / FIFO_DATA_BYTES);

    } else if (fr == FIFOFrameResult::CRC_ERROR) {
      Serial.printf("[TRUEPOLL][Poll#%d][T=%lu ms] CRC_ERROR on response -- ignoring, will re-poll.\n",
                    pollNum, (unsigned long)millis());

    } else {  // TIMEOUT -- THIS SPECIFIC poll's anchor-scan got no response
      Serial.printf("[TRUEPOLL][Poll#%d][T=%lu ms] No response within %lu ms of this poll -- re-polling.\n",
                    pollNum, (unsigned long)millis(), (unsigned long)perPollTimeoutMs);
    }

    delay(pollIntervalMs);
  }

  r.pollCount = maxPolls;
  snprintf(r.failureReason, sizeof(r.failureReason), "MAXPOLLS_EXCEEDED");
  Serial.printf("[TRUEPOLL] Exceeded %d polls without a full dump -- giving up this attempt.\n", maxPolls);
  return r;
}

void runFIFOTruePollLoop(int numRuns, uint32_t perPollTimeoutMs, uint32_t pollIntervalMs, int maxPolls) {
  Serial.printf("[TRUEPOLL] Starting %d run(s): perPollTimeoutMs=%lu, pollIntervalMs=%lu, maxPolls=%d\n",
                numRuns, (unsigned long)perPollTimeoutMs, (unsigned long)pollIntervalMs, maxPolls);
  Serial.println("[TRUEPOLL-CSV] RunID,SR,SRHz,TempC,Treq,PollCount,Tcomplete,Success,FailureReason");

  int successCount = 0, failCount = 0;
  long pollCountSum = 0;

  for (int i = 1; i <= numRuns; i++) {
    TruePollRunResult r = runFIFOTruePollOnce(perPollTimeoutMs, pollIntervalMs, maxPolls);

    uint32_t srHz = (g_lastSR < SR_TABLE_SIZE) ? SAMPLE_RATE_HZ[g_lastSR] : 0;
    float tempOut = g_lastVelocity.tempValid ? g_lastVelocity.tempC
                    : (g_lastTempValid ? g_lastTempC : 0.0f);

    Serial.printf("[TRUEPOLL-CSV] %d,%u,%lu,%.1f,%lu,%d,%lu,%d,%s\n",
                  i, g_lastSR, (unsigned long)srHz, tempOut,
                  (unsigned long)r.treq, r.pollCount,
                  (unsigned long)(r.success ? r.tComplete : 0),
                  r.success ? 1 : 0,
                  r.success ? "" : r.failureReason);

    if (r.success) { successCount++; pollCountSum += r.pollCount; }
    else failCount++;

    // Bus-settle drain between runs -- same discipline as every other
    // FIFO command in this file.
    uint32_t quietStart = millis();
    while (millis() - quietStart < 300) {
      if (SerialRS485.available()) { SerialRS485.read(); quietStart = millis(); }
    }
  }

  Serial.println("[TRUEPOLL] ---- Summary ----");
  Serial.printf("[TRUEPOLL]   Runs        = %d\n", numRuns);
  Serial.printf("[TRUEPOLL]   Success     = %d (%.1f%%)\n", successCount, 100.0f * successCount / numRuns);
  Serial.printf("[TRUEPOLL]   Fail        = %d (%.1f%%)\n", failCount, 100.0f * failCount / numRuns);
  if (successCount > 0) {
    Serial.printf("[TRUEPOLL]   Avg polls to success = %.2f\n", (float)pollCountSum / successCount);
  }
  Serial.println("[TRUEPOLL] ------------------");
}

bool readFIFOHybrid(uint32_t maxTotalMs = 20000, uint32_t stallMs = 2000) {
  Serial.println("[FIFO-HYBRID] Phase 1: single request, passively decoding autonomous frames...");
  sendFIFORequestOnce();
  uint32_t tSend = millis();
  Serial.printf("[FIFO-HYBRID][T=%lu ms] Request sent.\n", (unsigned long)tSend);

  // [v3.7] TIMELINE instrumentation -- see printFIFOTimeline() doc comment.
  // Purely bookkeeping: records when each milestone happens so the summary
  // can be printed at every exit point below. Does not alter timeouts or
  // control flow.
  uint32_t tRequestSent     = tSend;
  uint32_t tLastProgress    = 0;
  bool     haveLastProgress = false;
  uint32_t tPhase1Timeout   = 0;
  bool     hadPhase1Timeout = false;
  uint32_t tPhase2Sent      = 0;
  bool     sentPhase2       = false;

  uint32_t phaseStart = millis();
  uint16_t lastProgress = 0;
  bool sawAnyProgress = false;

  while (millis() - phaseStart < maxTotalMs) {
    uint16_t progress = 0;
    FIFOFrameResult r = readOneFIFOFrame(stallMs, progress, nullptr);

    if (r == FIFOFrameResult::FULL_DUMP) {
      uint32_t tComplete = millis();
      Serial.printf("[FIFO-HYBRID][T=%lu ms] Full dump arrived AUTONOMOUSLY during Phase 1 -- no Phase 2 needed. CRC verified.\n",
                    (unsigned long)tComplete);
      printFIFOTimeline(tRequestSent, tLastProgress, haveLastProgress, tPhase1Timeout, hadPhase1Timeout,
                         tPhase2Sent, sentPhase2, tComplete, true);
      return true;

    } else if (r == FIFOFrameResult::PROGRESS) {
      lastProgress = progress;
      sawAnyProgress = true;
      tLastProgress = millis();
      haveLastProgress = true;
      Serial.printf("[FIFO-HYBRID][T=%lu ms] Autonomous progress frame: %u/%u bytes (%.1f%%)\n",
                    (unsigned long)tLastProgress, progress, (unsigned)FIFO_DATA_BYTES,
                    100.0f * progress / FIFO_DATA_BYTES);

    } else if (r == FIFOFrameResult::CRC_ERROR) {
      Serial.println("[FIFO-HYBRID] CRC error on an autonomous frame -- ignoring, continuing to listen.");

    } else {  // TIMEOUT -- the autonomous stream has gone quiet
      tPhase1Timeout = millis();
      hadPhase1Timeout = true;
      Serial.printf("[FIFO-HYBRID][T=%lu ms] Autonomous stream quiet for %lu ms (last known progress: %u/%u bytes).\n",
                    (unsigned long)tPhase1Timeout, (unsigned long)stallMs, lastProgress, (unsigned)FIFO_DATA_BYTES);
      break;  // move to Phase 2
    }
  }

  // Phase 2: explicit follow-up request, sent exactly once.
  tPhase2Sent = millis();
  sentPhase2 = true;
  Serial.printf("[FIFO-HYBRID][T=%lu ms] Phase 2: sending ONE follow-up request for the full dump...\n",
                (unsigned long)tPhase2Sent);
  sendFIFORequestOnce();
  uint16_t progress2 = 0;
  FIFOFrameResult r2 = readOneFIFOFrame(4000, progress2, nullptr);

  if (r2 == FIFOFrameResult::FULL_DUMP) {
    uint32_t tComplete = millis();
    Serial.printf("[FIFO-HYBRID][T=%lu ms] Full dump arrived after explicit Phase 2 request. CRC verified.\n",
                  (unsigned long)tComplete);
    printFIFOTimeline(tRequestSent, tLastProgress, haveLastProgress, tPhase1Timeout, hadPhase1Timeout,
                       tPhase2Sent, sentPhase2, tComplete, true);
    return true;
  } else if (r2 == FIFOFrameResult::PROGRESS) {
    Serial.printf("[FIFO-HYBRID] Phase 2 got another progress frame (%u/%u), not the full dump -- giving up this attempt.\n",
                  progress2, (unsigned)FIFO_DATA_BYTES);
    printFIFOTimeline(tRequestSent, tLastProgress, haveLastProgress, tPhase1Timeout, hadPhase1Timeout,
                       tPhase2Sent, sentPhase2, 0, false);
    return false;
  } else {
    Serial.println("[FIFO-HYBRID] Phase 2 failed (timeout or CRC error) -- giving up this attempt.");
    printFIFOTimeline(tRequestSent, tLastProgress, haveLastProgress, tPhase1Timeout, hadPhase1Timeout,
                       tPhase2Sent, sentPhase2, 0, false);
    return false;
  }
}

bool readFIFORaw(uint32_t timeoutMs = 8000) {
  uint32_t overallStart = millis();
  int attemptNum = 0;
  uint16_t prevProgress = 0;
  bool havePrevProgress = false;

  while (millis() - overallStart < timeoutMs) {
    attemptNum++;
    // Build command frame: ID(0x50) Func(0x03) AddrH(0x00) AddrL(0x2C) LenH(0x00) LenL(0x01) CRCL CRCH
    uint8_t cmd[8];
    cmd[0] = MODBUS_SLAVE_ID;
    cmd[1] = 0x03;
    cmd[2] = (uint8_t)(REG_RAWFIFO >> 8);
    cmd[3] = (uint8_t)(REG_RAWFIFO & 0xFF);
    cmd[4] = 0x00;
    cmd[5] = 0x01;
    uint16_t cmdCrc = crc16_modbus_calc(cmd, 6);
    cmd[6] = (uint8_t)(cmdCrc & 0xFF);        // CRC low byte first (Modbus RTU convention)
    cmd[7] = (uint8_t)((cmdCrc >> 8) & 0xFF);

    while (SerialRS485.available()) SerialRS485.read();  // flush stale bytes
    uint32_t tSend = millis();
    Serial.printf("[FIFO][T=%lu ms] Read attempt #%d: sending request...\n", (unsigned long)tSend, attemptNum);
    SerialRS485.write(cmd, 8);
    SerialRS485.flush();

    // [v3.2] Byte-scanning resync -- replaces the earlier rigid "read exactly
    // 3 bytes and check" logic. Real-hardware testing (see project log,
    // stray-byte reconstruction) CONFIRMED via exact CRC match that a single
    // extra byte can arrive immediately before an otherwise perfectly valid
    // frame, shifting naive fixed-position reads out of alignment. Instead
    // of assuming byte 0 of whatever arrives is the real ID byte, scan a
    // simple 2-byte sliding window for the ID+Func anchor (0x50 0x03) and
    // discard anything before it -- this handles any number of leading
    // stray bytes, not just exactly one.
    uint8_t hdr[3];
    uint32_t waitStart = millis();
    int strayBytesSkipped = 0;
    const int MAX_STRAY_SKIP = 32;   // give up resyncing after this many discarded bytes

    uint8_t prevByte = 0x00;
    bool havePrev = false;
    bool anchored = false;

    // [v3.4] Per user request: timestamp of first byte seen this attempt,
    // and the largest gap observed between consecutive bytes -- a large
    // inter-byte gap here would point toward the ESP32 side (this code)
    // falling behind and risking UART RX overflow, as distinct from the
    // sensor itself being slow to respond.
    uint32_t tFirstByte = 0;
    bool haveFirstByte = false;
    uint32_t lastByteTime = 0;
    uint32_t maxGapMs = 0;

    while (!anchored) {
      if (millis() - waitStart > 1500) {
        Serial.printf("[FIFO][T=%lu ms] No response to read command (timeout while resyncing).\n", (unsigned long)millis());
        return false;
      }
      if (!SerialRS485.available()) continue;
      uint32_t now = millis();
      if (!haveFirstByte) {
        tFirstByte = now;
        haveFirstByte = true;
        Serial.printf("[FIFO][T=%lu ms] First byte of response arrived (request-to-first-byte latency = %lu ms).\n",
                      (unsigned long)tFirstByte, (unsigned long)(tFirstByte - tSend));
      } else {
        uint32_t gap = now - lastByteTime;
        if (gap > maxGapMs) maxGapMs = gap;
      }
      lastByteTime = now;
      uint8_t curByte = SerialRS485.read();

      if (havePrev && prevByte == MODBUS_SLAVE_ID && curByte == 0x03) {
        anchored = true;   // found "50 03" -- this is our real frame start
        break;
      }

      if (havePrev) {
        strayBytesSkipped++;
        if (strayBytesSkipped > MAX_STRAY_SKIP) {
          Serial.printf("[FIFO] Gave up resyncing after skipping %d stray byte(s) -- bus may be truly noisy.\n", strayBytesSkipped);
          return false;
        }
      }
      prevByte = curByte;
      havePrev = true;
    }

    if (strayBytesSkipped > 0) {
      Serial.printf("[FIFO] Resynced after skipping %d stray byte(s). Max inter-byte gap so far: %lu ms.\n",
                    strayBytesSkipped, (unsigned long)maxGapMs);
    }
    hdr[0] = MODBUS_SLAVE_ID;
    hdr[1] = 0x03;
    // Now read the 3rd header byte (Len).
    uint32_t lenWaitStart = millis();
    while (!SerialRS485.available()) {
      if (millis() - lenWaitStart > 1000) {
        Serial.println("[FIFO] Timeout waiting for Len byte after resync.");
        return false;
      }
    }
    hdr[2] = SerialRS485.read();

    if (hdr[2] == 0x01) {
      // Progress frame: 2 data bytes (fill size) + 2 CRC bytes = 4 more bytes
      uint32_t need = 4;
      uint8_t rest[4];
      waitStart = millis();
      uint32_t got = 0;
      while (got < need) {
        if (SerialRS485.available()) {
          uint32_t now = millis();
          uint32_t gap = now - lastByteTime;
          if (gap > maxGapMs) maxGapMs = gap;
          lastByteTime = now;
          rest[got++] = SerialRS485.read();
          waitStart = now;
        }
        else if (millis() - waitStart > 1000) { Serial.println("[FIFO] Timeout mid progress-frame."); return false; }
      }
      uint16_t fillBytes = ((uint16_t)rest[0] << 8) | rest[1];

      // [v3.4] Per user request: report raw bytes for THIS specific read
      // (always 7 for a progress frame: 3 header + 4 rest -- logged
      // explicitly rather than assumed) alongside the delta from the
      // previous attempt's decoded fill value. A consistently-repeating
      // delta (e.g. always exactly 3030 across multiple runs) would
      // support "sensor reads its accelerometer in fixed-size internal
      // chunks"; a delta that varies run-to-run points more toward
      // timing/noise instead.
      int rawBytesThisRead = 3 /*header*/ + 4 /*rest*/;
      Serial.printf("[FIFO][T=%lu ms] Read attempt #%d: RAW bytes received=%d, decoded fill=%u/%u bytes",
                    (unsigned long)millis(), attemptNum, rawBytesThisRead, fillBytes, (unsigned)FIFO_DATA_BYTES);
      if (havePrevProgress) {
        int32_t delta = (int32_t)fillBytes - (int32_t)prevProgress;
        Serial.printf(" (delta from previous attempt: %ld bytes)", (long)delta);
      }
      Serial.printf(", max inter-byte gap=%lu ms\n", (unsigned long)maxGapMs);
      prevProgress = fillBytes;
      havePrevProgress = true;
      // [v3.1] Was a blind delay(250). Now actively drains any bytes that
      // arrive during this wait instead of ignoring them -- if the sensor
      // ever sends data unprompted between our polls (one of the
      // candidate explanations for the real-hardware desync seen in
      // testing), this will surface it as a logged, non-fatal event
      // instead of letting it silently accumulate toward the next
      // command's response.
      // [v3.3] Now hex-dumps the actual bytes (not just a count) -- needed
      // to test a new hypothesis raised by real-hardware testing: that
      // these "unsolicited" bytes might actually be the START of the
      // sensor proactively sending the full 6149-byte dump on its own,
      // once ready, without waiting for another explicit request. If so,
      // the first bytes here should look like "50 03 00 ..." (the full-dump
      // header), not random noise.
      // [v3.4] Per user request: also measure how long the flush-and-drain
      // window's OWN Serial.print() calls take -- if printing hex bytes
      // over USB (115200 baud) itself eats meaningful wall-clock time
      // while we are NOT draining SerialRS485, our own diagnostics could
      // be contributing to the very overflow risk they're meant to catch.
      {
        uint32_t drainStart = millis();
        uint32_t waitEnd = drainStart + 250;
        int unsolicited = 0;
        uint32_t printTimeTotal = 0;
        Serial.printf("[FIFO][T=%lu ms] Unsolicited bytes this wait (hex): ", (unsigned long)drainStart);
        while (millis() < waitEnd) {
          if (SerialRS485.available()) {
            uint8_t b = SerialRS485.read();
            uint32_t pStart = millis();
            Serial.printf("%02X ", b);
            printTimeTotal += (millis() - pStart);
            unsolicited++;
          }
        }
        if (unsolicited > 0) {
          Serial.printf("\n[FIFO] Note: %d unsolicited byte(s) arrived between polls (discarded above). "
                        "Our own Serial.print() calls consumed %lu ms of this 250ms window.\n",
                        unsolicited, (unsigned long)printTimeTotal);
        } else {
          Serial.println("(none)");
        }
      }
      continue;     // retry the read

    } else if (hdr[2] == 0x00) {
      // Full dump: 6144 data bytes + 2 CRC bytes = 6146 more bytes
      static uint8_t dataBuf[FIFO_DATA_BYTES];
      uint8_t crcBuf[2];
      uint32_t need = FIFO_DATA_BYTES + 2;
      uint32_t got = 0;
      waitStart = millis();
      // [v3.4] Full-dump-specific timing: this is the "final phase" the
      // user identified as the remaining unknown -- track it with the same
      // rigor as the progress-frame path above.
      uint32_t dumpStartTime = waitStart;
      uint32_t dumpLastByteTime = waitStart;
      uint32_t dumpMaxGap = 0;
      while (got < need) {
        if (SerialRS485.available()) {
          uint32_t now = millis();
          uint32_t gap = now - dumpLastByteTime;
          if (gap > dumpMaxGap) dumpMaxGap = gap;
          dumpLastByteTime = now;
          uint8_t b = SerialRS485.read();
          if (got < FIFO_DATA_BYTES) dataBuf[got] = b;
          else crcBuf[got - FIFO_DATA_BYTES] = b;
          got++;
          waitStart = now;
        } else if (millis() - waitStart > 4000) {
          Serial.printf("[FIFO][T=%lu ms] Timeout mid full-dump (%lu/%lu bytes received, max gap so far=%lu ms).\n",
                        (unsigned long)millis(), (unsigned long)got, (unsigned long)need, (unsigned long)dumpMaxGap);
          return false;
        }
      }
      uint32_t dumpElapsed = millis() - dumpStartTime;
      Serial.printf("[FIFO][T=%lu ms] Full dump received: %lu bytes in %lu ms (%.1f bytes/ms), max inter-byte gap=%lu ms.\n",
                    (unsigned long)millis(), (unsigned long)need, (unsigned long)dumpElapsed,
                    dumpElapsed > 0 ? (float)need / dumpElapsed : 0.0f, (unsigned long)dumpMaxGap);

      // Verify CRC over header(3) + data(6144)
      uint8_t* fullFrame = (uint8_t*)malloc(3 + FIFO_DATA_BYTES);
      if (!fullFrame) { Serial.println("[FIFO] malloc failed for CRC check."); return false; }
      memcpy(fullFrame, hdr, 3);
      memcpy(fullFrame + 3, dataBuf, FIFO_DATA_BYTES);
      uint16_t calcCrc = crc16_modbus_calc(fullFrame, 3 + FIFO_DATA_BYTES);
      free(fullFrame);
      uint16_t sentCrc = ((uint16_t)crcBuf[1] << 8) | crcBuf[0];  // low byte first on the wire
      if (calcCrc != sentCrc) {
        Serial.printf("[FIFO] CRC MISMATCH: calc=0x%04X sent=0x%04X -- discarding this capture.\n", calcCrc, sentCrc);
        return false;
      }

      // Decode 1024 tri-axial big-endian signed 16-bit samples.
      for (int i = 0; i < FIFO_SAMPLE_COUNT; i++) {
        int off = i * 6;
        g_fifoX[i] = (int16_t)((dataBuf[off + 0] << 8) | dataBuf[off + 1]);
        g_fifoY[i] = (int16_t)((dataBuf[off + 2] << 8) | dataBuf[off + 3]);
        g_fifoZ[i] = (int16_t)((dataBuf[off + 4] << 8) | dataBuf[off + 5]);
      }
      Serial.println("[FIFO] Full 1024-sample capture OK, CRC verified.");
      return true;

    } else {
      Serial.printf("[FIFO] Unrecognized length byte 0x%02X -- aborting.\n", hdr[2]);
      return false;
    }
  }

  Serial.println("[FIFO] Overall timeout waiting for buffer to fill.");
  return false;
}

// Dumps all 1024 samples as CSV over Serial: one row per sample, both raw
// int16 counts and g-scaled values (per datasheet formula, raw/32768*16g --
// this scaling itself is one of the things RFC-0006 wants cross-checked,
// not assumed correct just because it's applied here).
//
// `tag` is a free-text marker (e.g. "idle", "tap", "orientA") written into
// every row so a capture can be correlated with a physical action taken
// during that capture -- addresses RFC-0006 §5 item 5 (orientation/event
// marker). `srAtCapture` records the SR register value active during this
// capture -- addresses §5 item 4, needed for Experiment 5 (SR variation).
void dumpFIFOToSerialCSV(const char* tag, uint16_t srAtCapture) {
  Serial.println("FIFOIndex,Tag,SR,X_raw,Y_raw,Z_raw,X_g,Y_g,Z_g");
  const float scale = 16.0f / 32768.0f;
  for (int i = 0; i < FIFO_SAMPLE_COUNT; i++) {
    Serial.printf("%d,%s,%u,%d,%d,%d,%.5f,%.5f,%.5f\n",
                  i, tag, srAtCapture,
                  g_fifoX[i], g_fifoY[i], g_fifoZ[i],
                  g_fifoX[i] * scale, g_fifoY[i] * scale, g_fifoZ[i] * scale);
  }
  Serial.println("FIFO_CAPTURE_END");   // sentinel line -- makes it trivial for a
                                          // post-processing script to detect end-of-capture
                                          // when parsing a Serial log that may contain
                                          // other output before/after this block.
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
      } else if (cmd == "FIFO" || cmd.startsWith("FIFO ")) {
        // [v3.0] RFC-0006 §5: raw FIFO capture + full 1024-sample CSV dump.
        // Optional tag: "FIFO tap1" marks this specific capture in its CSV
        // output (see dumpFIFOToSerialCSV) -- use this right before/after a
        // physical action (tap, orientation change, etc.) so the capture
        // can be correlated with what was actually done to the sensor.
        String tag = (cmd.length() > 5) ? cmd.substring(5) : String("untagged");
        tag.trim();
        Serial.printf("[CMD] Starting FIFO capture (tag=\"%s\")...\n", tag.c_str());
        bool fifoOk = readFIFORaw();
        if (fifoOk) {
          dumpFIFOToSerialCSV(tag.c_str(), g_lastSR);
        } else {
          Serial.println("[CMD] FIFO capture FAILED -- see [FIFO] messages above.");
        }
        // [v3.1] Bus-settle drain, run regardless of success/failure.
        // Real-hardware testing showed a FIFO failure could cascade into a
        // ku8MBResponseTimedOut (0xE2) on the VERY NEXT normal register
        // read -- consistent with leftover/misaligned bytes still sitting
        // in the RS485 stream after a FIFO transaction goes wrong. This
        // actively drains and discards anything arriving for up to 300ms
        // of true silence before handing control back to normal polling,
        // rather than trusting the bus is clean just because our function
        // returned.
        {
          uint32_t quietStart = millis();
          int drained = 0;
          while (millis() - quietStart < 300) {
            if (SerialRS485.available()) {
              SerialRS485.read();
              drained++;
              quietStart = millis();  // any byte seen resets the quiet timer
            }
          }
          if (drained > 0) {
            Serial.printf("[FIFO] Bus-settle: drained %d leftover byte(s) before resuming normal polling.\n", drained);
          }
        }
      } else if (cmd == "LISTEN" || cmd.startsWith("LISTEN ")) {
        // [v3.5] Falsification experiment (per review): send ONE request,
        // then only listen. Does NOT prove or assume autonomous streaming --
        // see fifoPassiveListen()'s own doc comment for what each outcome
        // would and would not establish. Optional arg: listen duration in ms
        // (default 15000).
        uint32_t listenMs = 15000;
        if (cmd.length() > 7) {
          long parsed = cmd.substring(7).toInt();
          if (parsed > 0) listenMs = (uint32_t)parsed;
        }
        Serial.printf("[CMD] Starting passive FIFO listen (%lu ms, single request)...\n", (unsigned long)listenMs);
        fifoPassiveListen(listenMs);
      } else if (cmd == "HYBRID" || cmd.startsWith("HYBRID ")) {
        // [v3.6] Built from the v3.5 LISTEN finding: progress-frames stream
        // autonomously, but the full dump did not arrive on its own even
        // after the buffer reached 99.6% full. Phase 1 listens passively;
        // Phase 2 sends exactly one follow-up request only if Phase 1's
        // stream goes quiet. See readFIFOHybrid()'s doc comment for details.
        String tag = (cmd.length() > 7) ? cmd.substring(7) : String("untagged");
        tag.trim();
        Serial.printf("[CMD] Starting HYBRID FIFO capture (tag=\"%s\")...\n", tag.c_str());
        bool hybridOk = readFIFOHybrid();
        if (hybridOk) {
          dumpFIFOToSerialCSV(tag.c_str(), g_lastSR);
        } else {
          Serial.println("[CMD] HYBRID FIFO capture FAILED -- see [FIFO-HYBRID] messages above.");
        }
        // Same bus-settle discipline as the FIFO command (v3.1) -- a failed
        // or even successful HYBRID capture could leave timing residue
        // that affects the next normal register read otherwise.
        {
          uint32_t quietStart = millis();
          int drained = 0;
          while (millis() - quietStart < 300) {
            if (SerialRS485.available()) { SerialRS485.read(); drained++; quietStart = millis(); }
          }
          if (drained > 0) {
            Serial.printf("[FIFO-HYBRID] Bus-settle: drained %d leftover byte(s) before resuming normal polling.\n", drained);
          }
        }
      } else if (cmd == "PURELISTEN" || cmd.startsWith("PURELISTEN ")) {
        // [v3.8] Test A per review: sends the request exactly ONCE, then
        // listens with a long (10s default) per-frame silence window and
        // NEVER sends a follow-up request under any circumstance -- unlike
        // HYBRID, a timeout here is treated as final, not a retry cue. This
        // isolates "does the sensor finish the dump autonomously given
        // enough time" from "does re-requesting help," which were tangled
        // together in the HYBRID failures analyzed so far. Run this BEFORE
        // deciding to raise HYBRID's stallMs -- if PURELISTEN also fails,
        // the problem is not (just) timeout length.
        String tag = (cmd.length() > 11) ? cmd.substring(11) : String("untagged");
        tag.trim();
        Serial.printf("[CMD] Starting PURELISTEN FIFO capture (tag=\"%s\")...\n", tag.c_str());
        bool pureOk = readFIFOPureListen();
        if (pureOk) {
          dumpFIFOToSerialCSV(tag.c_str(), g_lastSR);
        } else {
          Serial.println("[CMD] PURELISTEN FIFO capture FAILED (or inconclusive) -- see [FIFO-PURELISTEN] messages above.");
        }
        {
          uint32_t quietStart = millis();
          int drained = 0;
          while (millis() - quietStart < 300) {
            if (SerialRS485.available()) { SerialRS485.read(); drained++; quietStart = millis(); }
          }
          if (drained > 0) {
            Serial.printf("[FIFO-PURELISTEN] Bus-settle: drained %d leftover byte(s) before resuming normal polling.\n", drained);
          }
        }
      } else if (cmd == "STRESSTEST" || cmd.startsWith("STRESSTEST ")) {
        // [v3.9] Per review: stop guessing fixes, start mapping the sensor's
        // state machine statistically. Usage:
        //   STRESSTEST <numRuns>
        //   STRESSTEST <numRuns> <stallMs>
        //   STRESSTEST <numRuns> <stallMs> <maxTotalMs>
        // Defaults match PURELISTEN's (stallMs=10000, maxTotalMs=30000) if
        // omitted. For "Experiment B" (does a stalled run ever recover
        // given much longer than 10s, with NO request sent in between),
        // pass a much larger stallMs, e.g.: STRESSTEST 20 60000 70000
        int numRuns = 10;
        uint32_t stallMs = 10000;
        uint32_t maxTotalMs = 30000;
        if (cmd.length() > 11) {
          String args = cmd.substring(11);
          args.trim();
          int sp1 = args.indexOf(' ');
          if (sp1 < 0) {
            numRuns = args.toInt();
          } else {
            numRuns = args.substring(0, sp1).toInt();
            String rest = args.substring(sp1 + 1);
            rest.trim();
            int sp2 = rest.indexOf(' ');
            if (sp2 < 0) {
              long v = rest.toInt();
              if (v > 0) stallMs = (uint32_t)v;
            } else {
              long v1 = rest.substring(0, sp2).toInt();
              long v2 = rest.substring(sp2 + 1).toInt();
              if (v1 > 0) stallMs = (uint32_t)v1;
              if (v2 > 0) maxTotalMs = (uint32_t)v2;
            }
          }
        }
        if (numRuns <= 0) numRuns = 1;
        if (maxTotalMs <= stallMs) maxTotalMs = stallMs + 5000;  // guard against a nonsensical config
        runFIFOStressLoop(numRuns, stallMs, maxTotalMs);
      } else if (cmd == "TRUEPOLL" || cmd.startsWith("TRUEPOLL ")) {
        // [v3.11] Per review: independent experiment testing the
        // datasheet-literal repeated-polling protocol for RAWFIFO, run
        // ALONGSIDE (not replacing) STRESSTEST. Usage:
        //   TRUEPOLL <numRuns> [perPollTimeoutMs] [pollIntervalMs] [maxPolls]
        // Defaults: perPollTimeoutMs=1000, pollIntervalMs=200, maxPolls=60
        // (60 polls * ~200ms spacing = 12s+ of active polling budget,
        // comparable in total wall-clock to STRESSTEST's default window).
        int numRuns = 10;
        uint32_t perPollTimeoutMs = 1000;
        uint32_t pollIntervalMs = 200;
        int maxPolls = 60;
        if (cmd.length() > 9) {
          String args = cmd.substring(9);
          args.trim();
          long vals[4] = {0, 0, 0, 0};
          int nVals = 0;
          int start = 0;
          while (nVals < 4) {
            int sp = args.indexOf(' ', start);
            String tok = (sp < 0) ? args.substring(start) : args.substring(start, sp);
            tok.trim();
            if (tok.length() == 0) break;
            vals[nVals++] = tok.toInt();
            if (sp < 0) break;
            start = sp + 1;
          }
          if (nVals >= 1 && vals[0] > 0) numRuns = (int)vals[0];
          if (nVals >= 2 && vals[1] > 0) perPollTimeoutMs = (uint32_t)vals[1];
          if (nVals >= 3 && vals[2] > 0) pollIntervalMs = (uint32_t)vals[2];
          if (nVals >= 4 && vals[3] > 0) maxPolls = (int)vals[3];
        }
        runFIFOTruePollLoop(numRuns, perPollTimeoutMs, pollIntervalMs, maxPolls);
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
  // [v3.0] RFC-0006 §5: raw FIFO capture tool
  Serial.println(" Commands: SR n | ID <text> | SETPOINT <hz> | ENERGY | INFO | SUMMARY");
  Serial.println("           FIFO [tag]   -- capture+dump 1024 raw tri-axial samples as CSV");
  Serial.println("           LISTEN [ms]  -- v3.5 falsification test: ONE request, then passive listen only");
  Serial.println("           HYBRID [tag] -- v3.7: passive listen for progress, one follow-up request for the dump");
  Serial.println("                          + [FIFO-TIMELINE] Anchor/Treq/Tprog/Ttimeout/Tphase2/Tcomplete instrumentation");
  Serial.println("           PURELISTEN [tag] -- v3.8 Test A: ONE request, then listen only (10s window), NEVER retries.");
  Serial.println("                          Proves/falsifies autonomous completion independent of any retry behavior.");
  Serial.println("           STRESSTEST <n> [stallMs] [maxTotalMs] -- v3.10 (fixed TanchorNext off-by-one bug");
  Serial.println("                          from v3.9): runs PURELISTEN-style n times, logs");
  Serial.println("                          RunID/SR/TempC/Treq/T49/T99/TanchorNext/Tcomplete/Success/FailureReason");
  Serial.println("                          as CSV ([STRESS-CSV] lines) for statistical state-machine mapping.");
  Serial.println("           TRUEPOLL <n> [perPollTimeoutMs] [pollIntervalMs] [maxPolls] -- v3.11:");
  Serial.println("                          INDEPENDENT experiment (does not replace HYBRID/STRESSTEST) that");
  Serial.println("                          repeatedly re-sends the RAWFIFO read command per the datasheet's");
  Serial.println("                          literal protocol (sec 6.1.4.16), logging every poll's response.");

  // RS485 enable pin – hold LOW permanently (no pre/post-transmission callbacks)
  pinMode(RS485_EN_PIN, OUTPUT);
  rs485Enable();

  // Initialise hardware UART for RS485
  // [v3.1] Defensive: default ESP32 HardwareSerial RX buffer is 256 bytes.
  // A 6144-byte FIFO dump arrives far faster than our code drains it during
  // any blocking section (e.g. delay() calls elsewhere in the sketch) --
  // widening this reduces (does not by itself prove it eliminates) the risk
  // of hardware-level RX overflow contributing to the desync observed in
  // real-hardware testing (see readFIFORaw's bus-settle drain, v3.1).
  SerialRS485.setRxBufferSize(2048);
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
