// ============================================================================
// WTVB02_CorruptionDiagnostic.ino  –  v3.0
// ----------------------------------------------------------------------------
// PURPOSE
//   Determine the ROOT CAUSE of physically impossible register values observed
//   in the WTVB02 vibration sensor.
//
//   Known symptoms:
//     VX RAW = 35349 (0x8A15)  →  int16 = -30187  →  Peak = -301.87 mm/s
//     VRMSY RAW = 38684 (0x971C)  →  RMS = 38.684 mm/s  (>> Peak)
//     Peak/RMS ratios exceeding 200
//
//   Timing-based asynchronous update has been RULED OUT (130 ms window tested).
//   This sketch investigates deeper causes:
//     1. Modbus frame corruption  (CRC errors, status codes per transaction)
//     2. Register mapping errors  (wrong address, page switching)
//     3. Endianness/byte-order errors
//     4. Single-register instability vs cross-register inconsistency
//     5. Sensor firmware anomalies during algorithm update cycles
//
// DIAGNOSTIC PHASES  (run once from setup(), then continuous statistics)
//   Phase 1 – STABILITY TEST   : Read each of 6 registers 10× in rapid burst
//   Phase 2 – BURST BLOCK READ : Single read covering 0x003A–0x0068 (47 regs)
//   Phase 3 – ENDIANNESS PROBE : Compare normal u16 vs byte-swapped u16
//   Phase 4 – 100-SAMPLE STATS : Collect statistics, flag suspicious values
//
// Hardware : LilyGO T-Vending S3  (ESP32-S3)
// Library  : ModbusMaster
// ============================================================================

#include <ModbusMaster.h>
#include <math.h>       // sqrtf

// ── Pin definitions (LilyGO T-Vending S3) ────────────────────────────────────
#define RS485_TX_PIN     39
#define RS485_RX_PIN     38
#define RS485_EN_PIN     42

// ── Modbus settings ───────────────────────────────────────────────────────────
#define MODBUS_SLAVE_ID   0x50
#define MODBUS_BAUDRATE   9600

// ── Target registers ─────────────────────────────────────────────────────────
//   Velocity Amplitude (Peak)  –  scale (int16_t)raw / 100.0  → mm/s
#define REG_VX       0x003A
#define REG_VY       0x003B
#define REG_VZ       0x003C
//   Velocity RMS  –  scale raw / 1000.0  → mm/s
#define REG_VRMSX    0x0050
#define REG_VRMSY    0x005C
#define REG_VRMSZ    0x0068

// ── Burst block boundaries ────────────────────────────────────────────────────
//   One contiguous read: 0x003A → 0x0068  =  47 registers
#define BURST_START   REG_VX
#define BURST_COUNT   47     // (0x0068 - 0x003A) + 1  = 0x2F = 47
// Offsets within burst array (from BURST_START)
#define BURST_OFF_VX      0   // 0x3A - 0x3A
#define BURST_OFF_VY      1
#define BURST_OFF_VZ      2
#define BURST_OFF_VRMSX  22   // 0x50 - 0x3A
#define BURST_OFF_VRMSY  34   // 0x5C - 0x3A
#define BURST_OFF_VRMSZ  46   // 0x68 - 0x3A

// ── Diagnostic parameters ─────────────────────────────────────────────────────
#define STABILITY_REPS         10     // consecutive reads per register in Phase 1
#define STABILITY_DELAY_MS     50     // gap between stability reads
#define STATS_SAMPLES         100     // samples to collect in Phase 4
#define STATS_INTERVAL_MS    2000UL   // interval between normal samples

// ── Suspicious-value thresholds ───────────────────────────────────────────────
#define THRESH_PEAK_MAX_MMPS   50.0f  // |Peak| > 50 mm/s → flag
#define THRESH_RAW_HIGH       60000U  // any raw uint16 > 60000 → flag
#define THRESH_RATIO_MIN        0.5f  // Peak/RMS below this → flag
#define THRESH_RATIO_MAX        5.0f  // Peak/RMS above this → flag

// ── Peripheral objects ────────────────────────────────────────────────────────
HardwareSerial SerialRS485(2);
ModbusMaster   modbus;

// ─────────────────────────────────────────────────────────────────────────────
// Utility: Modbus status code → human-readable string
// Covers all codes defined in ModbusMaster.h
// ─────────────────────────────────────────────────────────────────────────────
const char* modbusStatusStr(uint8_t code) {
  switch (code) {
    case 0x00: return "Success";
    case 0x01: return "IllegalFunction";
    case 0x02: return "IllegalDataAddress";
    case 0x03: return "IllegalDataValue";
    case 0x04: return "SlaveDeviceFailure";
    case 0x05: return "Acknowledge";
    case 0x06: return "SlaveDeviceBusy";
    case 0x08: return "MemoryParityError";
    case 0x0A: return "GatewayPathUnavailable";
    case 0x0B: return "GatewayTargetDeviceFailedToRespond";
    case 0xE0: return "InvalidSlaveID";
    case 0xE1: return "InvalidFunction";
    case 0xE2: return "ResponseTimedOut";           // ← timeout / no response
    case 0xE3: return "InvalidCRC";                 // ← frame corruption
    default:   return "UnknownError";
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// Utility: classify what is suspicious about a raw uint16 value
// Returns a bitmask; 0 = clean
//   bit 0 → raw > 60000 (near-max, possibly garbage)
//   bit 1 → raw >= 0x8000 (MSB set → negative when cast to int16)
// ─────────────────────────────────────────────────────────────────────────────
#define FLAG_RAW_HIGH   0x01
#define FLAG_SIGN_BIT   0x02

uint8_t rawSuspicion(uint16_t raw) {
  uint8_t f = 0;
  if (raw > THRESH_RAW_HIGH) f |= FLAG_RAW_HIGH;
  if (raw >= 0x8000)          f |= FLAG_SIGN_BIT;
  return f;
}

// ─────────────────────────────────────────────────────────────────────────────
// Utility: print a single register's dual interpretation + flags
// ─────────────────────────────────────────────────────────────────────────────
void printRegRow(const char* label, uint16_t raw, bool isSigned) {
  int16_t  s16 = (int16_t)raw;
  uint8_t  sus = rawSuspicion(raw);
  char     flags[32] = "";
  if (sus & FLAG_SIGN_BIT) strcat(flags, " [MSB_SET]");
  if (sus & FLAG_RAW_HIGH) strcat(flags, " [RAW>60000]");

  if (isSigned) {
    Serial.printf("  %-10s  uint16=%6u (0x%04X)  int16=%7d  scaled=%8.3f mm/s%s\n",
                  label, raw, raw, s16, s16 / 100.0f, flags);
  } else {
    Serial.printf("  %-10s  uint16=%6u (0x%04X)  int16=%7d  scaled=%8.3f mm/s%s\n",
                  label, raw, raw, s16, raw / 1000.0f, flags);
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// rs485Enable()
// DE/RE held LOW permanently – no pre/post-transmission callbacks.
// ─────────────────────────────────────────────────────────────────────────────
void rs485Enable() {
  digitalWrite(RS485_EN_PIN, LOW);
}

// ─────────────────────────────────────────────────────────────────────────────
// timedRead()
// Wrapper around readHoldingRegisters() that timestamps the call,
// prints the Modbus status code + description, and returns the code.
// tsEnd is written so the caller can compute duration.
// ─────────────────────────────────────────────────────────────────────────────
uint8_t timedRead(uint16_t reg, uint16_t count,
                  unsigned long &tsStart, unsigned long &tsEnd) {
  tsStart       = millis();
  uint8_t code  = modbus.readHoldingRegisters(reg, count);
  tsEnd         = millis();
  uint32_t dur  = (uint32_t)(tsEnd - tsStart);

  Serial.printf("[%lu ms] readHR(0x%04X, %u)  dur=%u ms  status=0x%02X (%s)\n",
                tsStart, reg, count, dur, code, modbusStatusStr(code));
  return code;
}

// ─────────────────────────────────────────────────────────────────────────────
// waitForSensor()
// Blocks until a basic 3-register read at 0x003A succeeds.
// ─────────────────────────────────────────────────────────────────────────────
void waitForSensor() {
  Serial.println("[INIT] Verifying sensor communication...");
  uint8_t code;
  do {
    unsigned long ts, te;
    code = timedRead(REG_VX, 3, ts, te);
    if (code != modbus.ku8MBSuccess) {
      Serial.printf("[INIT] Sensor not responding (%s) – retry in 3 s\n",
                    modbusStatusStr(code));
      delay(3000);
    }
  } while (code != modbus.ku8MBSuccess);
  Serial.println("[INIT] Sensor OK.\n");
}

// =============================================================================
// PHASE 1 – REGISTER STABILITY TEST
// =============================================================================
// Read each of the 6 target registers STABILITY_REPS times in rapid succession.
// Compute mean, stddev, min, max across all reps.
// Flag if any individual read deviates from the first read OR if any value
// is in the suspicious range.
// Purpose: determines whether a single register is inherently unstable,
//          which would indicate sensor-side firmware issues rather than
//          cross-register timing problems.
// =============================================================================
struct StabilityProbe {
  uint16_t addr;
  const char* name;
  bool isSigned;   // true = Peak register (int16 / 100), false = RMS (uint16 / 1000)
};

void runPhase1_StabilityTest() {
  Serial.println("\n");
  Serial.println("╔═══════════════════════════════════════════════════════════════╗");
  Serial.println("║  PHASE 1 – REGISTER STABILITY TEST                           ║");
  Serial.println("║  Each register read 10× consecutively (50 ms apart)          ║");
  Serial.println("║  Goal: detect if a single register flips between reads        ║");
  Serial.println("╚═══════════════════════════════════════════════════════════════╝");

  const StabilityProbe probes[] = {
    { REG_VX,    "VX(0x3A)",    true  },
    { REG_VY,    "VY(0x3B)",    true  },
    { REG_VZ,    "VZ(0x3C)",    true  },
    { REG_VRMSX, "VRMSX(0x50)", false },
    { REG_VRMSY, "VRMSY(0x5C)", false },
    { REG_VRMSZ, "VRMSZ(0x68)", false },
  };
  const uint8_t nProbes = sizeof(probes) / sizeof(probes[0]);

  for (uint8_t p = 0; p < nProbes; p++) {
    const StabilityProbe &pr = probes[p];
    Serial.printf("\n─── Register %s ─────────────────────────────────────\n",
                  pr.name);

    uint16_t vals[STABILITY_REPS];
    uint8_t  statCodes[STABILITY_REPS];
    bool     anyFail    = false;
    bool     anySuspect = false;

    for (uint8_t r = 0; r < STABILITY_REPS; r++) {
      unsigned long ts, te;
      uint8_t code = timedRead(pr.addr, 1, ts, te);
      statCodes[r] = code;

      if (code != modbus.ku8MBSuccess) {
        vals[r] = 0xFFFF;
        anyFail = true;
        Serial.printf("    Rep %2d: FAILED (%s)\n", r + 1, modbusStatusStr(code));
      } else {
        vals[r] = modbus.getResponseBuffer(0);
        uint8_t sus = rawSuspicion(vals[r]);
        if (sus) anySuspect = true;

        int16_t s16    = (int16_t)vals[r];
        float   scaled = pr.isSigned ? s16 / 100.0f : vals[r] / 1000.0f;

        Serial.printf("    Rep %2d: uint16=%6u (0x%04X)  int16=%7d  scaled=%8.3f mm/s%s%s\n",
                      r + 1,
                      vals[r], vals[r], s16, scaled,
                      (sus & FLAG_SIGN_BIT) ? " [MSB_SET]"   : "",
                      (sus & FLAG_RAW_HIGH) ? " [RAW>60000]" : "");
      }
      delay(STABILITY_DELAY_MS);
    }

    // ── Statistics across successful reads ──────────────────────────────────
    float sum = 0, sumSq = 0;
    uint16_t vmin = 0xFFFF, vmax = 0;
    uint8_t  nGood = 0;

    for (uint8_t r = 0; r < STABILITY_REPS; r++) {
      if (statCodes[r] == modbus.ku8MBSuccess && vals[r] != 0xFFFF) {
        float v = (float)vals[r];
        sum   += v;
        sumSq += v * v;
        if (vals[r] < vmin) vmin = vals[r];
        if (vals[r] > vmax) vmax = vals[r];
        nGood++;
      }
    }

    if (nGood > 1) {
      float mean   = sum / nGood;
      float var    = (sumSq / nGood) - (mean * mean);
      float stddev = (var > 0.0f) ? sqrtf(var) : 0.0f;
      float spread = (float)(vmax - vmin);

      Serial.printf("  ── Stats (n=%u): min=%u  max=%u  mean=%.1f  stddev=%.1f  spread=%u\n",
                    nGood, vmin, vmax, mean, stddev, (uint16_t)spread);

      // Verdict
      if (anyFail) {
        Serial.println("  *** VERDICT: MODBUS FAILURES DETECTED ***");
      } else if (anySuspect) {
        Serial.println("  *** VERDICT: SUSPICIOUS RAW VALUES (MSB set or > 60000) ***");
        Serial.println("  Interpretation: register contents are physically impossible.");
        Serial.println("  Possible cause: sensor outputs this register in a different");
        Serial.println("  unit/format than expected, OR firmware state machine writes");
        Serial.println("  partially during the read window.");
      } else if (spread > 1000) {
        Serial.println("  *** VERDICT: HIGH SPREAD – register is volatile ***");
        Serial.println("  Interpretation: sensor is actively updating this register.");
        Serial.println("  Not a Modbus transport error (reads succeed); the sensor");
        Serial.println("  firmware is writing new algorithm results between reads.");
      } else {
        Serial.println("  VERDICT: STABLE – no anomalies in this register burst.");
      }
    } else {
      Serial.println("  VERDICT: Insufficient successful reads to compute statistics.");
    }
  }
}

// =============================================================================
// PHASE 2 – BURST BLOCK READ TEST
// =============================================================================
// Read registers 0x003A through 0x0068 as a single 47-register Modbus request.
// Print the full register map, then compare the 6 known registers against
// a subsequent set of 4 individual reads.
// Purpose: if burst values match individual reads → register addresses are
//          correct and no page switching occurs between read operations.
//          If they differ → the sensor remaps registers between transactions.
// =============================================================================
void runPhase2_BurstBlockRead() {
  Serial.println("\n");
  Serial.println("╔═══════════════════════════════════════════════════════════════╗");
  Serial.println("║  PHASE 2 – BURST BLOCK READ TEST                             ║");
  Serial.printf ("║  Single read: 0x%04X → 0x%04X  (%u registers)              ║\n",
                 BURST_START, (uint16_t)(BURST_START + BURST_COUNT - 1), BURST_COUNT);
  Serial.println("║  Goal: detect register page-switching between transactions    ║");
  Serial.println("╚═══════════════════════════════════════════════════════════════╝");

  // ── Burst read ──────────────────────────────────────────────────────────────
  unsigned long ts, te;
  uint8_t code = timedRead(BURST_START, BURST_COUNT, ts, te);

  if (code != modbus.ku8MBSuccess) {
    Serial.printf("[PHASE2] Burst read FAILED: %s\n", modbusStatusStr(code));
    Serial.println("  Note: sensor may not support reading this many registers");
    Serial.println("  in a single transaction. Phase 2 skipped.");
    return;
  }

  // Copy burst data before any further Modbus call overwrites the buffer
  uint16_t burst[BURST_COUNT];
  for (uint8_t i = 0; i < BURST_COUNT; i++) {
    burst[i] = modbus.getResponseBuffer(i);
  }

  // ── Print full register table ───────────────────────────────────────────────
  Serial.println("\n  Full burst register map:");
  Serial.println("  Offset  Address  uint16   Hex     int16    Flags");
  Serial.println("  ──────────────────────────────────────────────────────");
  for (uint8_t i = 0; i < BURST_COUNT; i++) {
    uint16_t addr = BURST_START + i;
    uint16_t raw  = burst[i];
    int16_t  s16  = (int16_t)raw;
    uint8_t  sus  = rawSuspicion(raw);
    char     flag[32] = "";
    if (sus & FLAG_SIGN_BIT) strcat(flag, " MSB");
    if (sus & FLAG_RAW_HIGH) strcat(flag, " HIGH");

    // Highlight the 6 known registers
    const char* knownLabel = "";
    if (addr == REG_VX)    knownLabel = " ← VX";
    if (addr == REG_VY)    knownLabel = " ← VY";
    if (addr == REG_VZ)    knownLabel = " ← VZ";
    if (addr == REG_VRMSX) knownLabel = " ← VRMSX";
    if (addr == REG_VRMSY) knownLabel = " ← VRMSY";
    if (addr == REG_VRMSZ) knownLabel = " ← VRMSZ";

    Serial.printf("  [%2u]    0x%04X   %6u   0x%04X  %7d  %s%s\n",
                  i, addr, raw, raw, s16, flag, knownLabel);
  }

  // ── Extract burst values for the 6 known registers ──────────────────────────
  uint16_t b_vx    = burst[BURST_OFF_VX];
  uint16_t b_vy    = burst[BURST_OFF_VY];
  uint16_t b_vz    = burst[BURST_OFF_VZ];
  uint16_t b_vrmsx = burst[BURST_OFF_VRMSX];
  uint16_t b_vrmsy = burst[BURST_OFF_VRMSY];
  uint16_t b_vrmsz = burst[BURST_OFF_VRMSZ];

  // ── Now do 4 individual reads for comparison ────────────────────────────────
  delay(100);
  uint16_t i_vx = 0, i_vy = 0, i_vz = 0;
  uint16_t i_vrmsx = 0, i_vrmsy = 0, i_vrmsz = 0;
  bool     indivOk = true;

  unsigned long ts2, te2;

  code = timedRead(REG_VX, 3, ts2, te2);
  if (code == modbus.ku8MBSuccess) {
    i_vx = modbus.getResponseBuffer(0);
    i_vy = modbus.getResponseBuffer(1);
    i_vz = modbus.getResponseBuffer(2);
  } else { indivOk = false; }
  delay(20);

  code = timedRead(REG_VRMSX, 1, ts2, te2);
  if (code == modbus.ku8MBSuccess) i_vrmsx = modbus.getResponseBuffer(0);
  else indivOk = false;
  delay(20);

  code = timedRead(REG_VRMSY, 1, ts2, te2);
  if (code == modbus.ku8MBSuccess) i_vrmsy = modbus.getResponseBuffer(0);
  else indivOk = false;
  delay(20);

  code = timedRead(REG_VRMSZ, 1, ts2, te2);
  if (code == modbus.ku8MBSuccess) i_vrmsz = modbus.getResponseBuffer(0);
  else indivOk = false;
  delay(20);

  // ── Comparison table ────────────────────────────────────────────────────────
  Serial.println("\n  Burst vs Individual read comparison:");
  Serial.println("  Register    Burst     Individual   Match?");
  Serial.println("  ──────────────────────────────────────────────");

  auto cmpRow = [](const char* name, uint16_t bv, uint16_t iv) {
    bool match = (bv == iv);
    Serial.printf("  %-10s  %6u    %6u       %s\n",
                  name, bv, iv, match ? "YES" : "*** NO – MISMATCH ***");
    return match;
  };

  bool allMatch = true;
  if (indivOk) {
    allMatch &= cmpRow("VX",    b_vx,    i_vx);
    allMatch &= cmpRow("VY",    b_vy,    i_vy);
    allMatch &= cmpRow("VZ",    b_vz,    i_vz);
    allMatch &= cmpRow("VRMSX", b_vrmsx, i_vrmsx);
    allMatch &= cmpRow("VRMSY", b_vrmsy, i_vrmsy);
    allMatch &= cmpRow("VRMSZ", b_vrmsz, i_vrmsz);
  } else {
    Serial.println("  Individual reads partially failed – comparison skipped.");
    allMatch = false;
  }

  Serial.println("  ──────────────────────────────────────────────");
  if (allMatch) {
    Serial.println("  VERDICT: All registers match burst vs individual.");
    Serial.println("  → Register addresses are CORRECT.");
    Serial.println("  → No page-switching detected between transactions.");
    Serial.println("  → Corruption does NOT originate from address mapping.");
  } else {
    Serial.println("  *** VERDICT: MISMATCH detected between burst and individual reads ***");
    Serial.println("  → Sensor MAY be switching register pages between transactions.");
    Serial.println("  → OR sensor updated its values in the ~100 ms between burst and");
    Serial.println("    individual reads (normal sensor update cycle).");
    Serial.println("  → Repeat this phase multiple times to distinguish the two cases.");
  }
}

// =============================================================================
// PHASE 3 – ENDIANNESS PROBE
// =============================================================================
// Read register 0x003A once normally (1 register = 2 bytes from sensor).
// Read registers 0x003A and 0x003B together (2 registers).
// Interpret the first register in the 2-register read both ways:
//   Normal:    value_as_returned_by_ModbusMaster
//   Swapped:   (low_byte << 8) | high_byte
// Purpose: Modbus specifies big-endian (high byte first), but some sensor
//          firmware implementations use little-endian. If the swapped
//          interpretation produces more physically plausible values, the
//          sensor is using a non-standard byte order.
// =============================================================================
void runPhase3_EndianessProbe() {
  Serial.println("\n");
  Serial.println("╔═══════════════════════════════════════════════════════════════╗");
  Serial.println("║  PHASE 3 – ENDIANNESS / BYTE-ORDER PROBE                     ║");
  Serial.println("║  Goal: confirm whether Modbus big-endian assumption is valid  ║");
  Serial.println("╚═══════════════════════════════════════════════════════════════╝");

  Serial.println("\n  Step A: Single register read (0x003A = VX)");
  unsigned long ts, te;
  uint8_t code = timedRead(REG_VX, 1, ts, te);
  if (code != modbus.ku8MBSuccess) {
    Serial.println("[PHASE3] VX single read failed – probe skipped.");
    return;
  }
  uint16_t raw_single = modbus.getResponseBuffer(0);
  uint8_t  hi_a = (uint8_t)(raw_single >> 8);
  uint8_t  lo_a = (uint8_t)(raw_single & 0xFF);

  Serial.printf("    raw_single = 0x%04X  HI=0x%02X  LO=0x%02X\n",
                raw_single, hi_a, lo_a);
  Serial.printf("    Normal  (big-endian, HI<<8|LO): uint16=%u  int16=%d  → %.3f mm/s\n",
                raw_single, (int16_t)raw_single, (int16_t)raw_single / 100.0f);
  uint16_t swapped_a = (uint16_t)((lo_a << 8) | hi_a);
  Serial.printf("    Swapped (little-endian, LO<<8|HI): uint16=%u  int16=%d  → %.3f mm/s\n",
                swapped_a, (int16_t)swapped_a, (int16_t)swapped_a / 100.0f);

  delay(30);

  Serial.println("\n  Step B: Two-register block read starting at 0x003A");
  code = timedRead(REG_VX, 2, ts, te);
  if (code != modbus.ku8MBSuccess) {
    Serial.println("[PHASE3] Two-register read failed.");
    return;
  }
  uint16_t reg0 = modbus.getResponseBuffer(0);  // word at 0x3A
  uint16_t reg1 = modbus.getResponseBuffer(1);  // word at 0x3B
  uint8_t  hi0  = (uint8_t)(reg0 >> 8);
  uint8_t  lo0  = (uint8_t)(reg0 & 0xFF);
  uint8_t  hi1  = (uint8_t)(reg1 >> 8);
  uint8_t  lo1  = (uint8_t)(reg1 & 0xFF);

  Serial.printf("    reg[0x3A] = 0x%04X  (HI=0x%02X LO=0x%02X)\n", reg0, hi0, lo0);
  Serial.printf("    reg[0x3B] = 0x%04X  (HI=0x%02X LO=0x%02X)\n", reg1, hi1, lo1);

  // Interpretations
  uint16_t interp_normal  = reg0;
  uint16_t interp_swapped = (uint16_t)((lo0 << 8) | hi0);
  uint16_t interp_cross1  = (uint16_t)((hi0 << 8) | hi1);   // high bytes of both words
  uint16_t interp_cross2  = (uint16_t)((lo0 << 8) | lo1);   // low  bytes of both words

  Serial.println("\n    All byte-order interpretations of register 0x3A:");
  Serial.printf("    [A] Normal  big-endian  (hi0<<8|lo0) = %6u  int16=%d  → %.3f mm/s\n",
                interp_normal,  (int16_t)interp_normal,  (int16_t)interp_normal  / 100.0f);
  Serial.printf("    [B] Swapped little-endian (lo0<<8|hi0) = %6u  int16=%d  → %.3f mm/s\n",
                interp_swapped, (int16_t)interp_swapped, (int16_t)interp_swapped / 100.0f);
  Serial.printf("    [C] Cross hi bytes (hi0<<8|hi1)       = %6u  int16=%d  → %.3f mm/s\n",
                interp_cross1,  (int16_t)interp_cross1,  (int16_t)interp_cross1  / 100.0f);
  Serial.printf("    [D] Cross lo bytes (lo0<<8|lo1)       = %6u  int16=%d  → %.3f mm/s\n",
                interp_cross2,  (int16_t)interp_cross2,  (int16_t)interp_cross2  / 100.0f);

  // Plausibility check: which interpretation produces |value| < 50 mm/s?
  Serial.println("\n    Plausibility filter (|scaled| < 50 mm/s = physically possible):");
  const char* interps[] = { "A(normal)", "B(swapped)", "C(cross-hi)", "D(cross-lo)" };
  uint16_t    raws[]    = { interp_normal, interp_swapped, interp_cross1, interp_cross2 };
  for (uint8_t i = 0; i < 4; i++) {
    float v = (int16_t)raws[i] / 100.0f;
    bool  ok = (v > -50.0f && v < 50.0f && v > 0.0f);
    Serial.printf("    %s → %.3f mm/s : %s\n",
                  interps[i], v, ok ? "PLAUSIBLE" : "suspicious");
  }

  Serial.println("\n  VERDICT:");
  float v_normal  = (int16_t)interp_normal  / 100.0f;
  float v_swapped = (int16_t)interp_swapped / 100.0f;
  bool  normalOk  = (v_normal  > 0.0f && v_normal  < 50.0f);
  bool  swappedOk = (v_swapped > 0.0f && v_swapped < 50.0f);

  if (normalOk && !swappedOk) {
    Serial.println("  → Normal big-endian interpretation is CORRECT (standard Modbus).");
  } else if (swappedOk && !normalOk) {
    Serial.println("  *** → Swapped (little-endian) interpretation is MORE PLAUSIBLE! ***");
    Serial.println("  *** Sensor firmware may be using non-standard byte order.        ***");
  } else if (normalOk && swappedOk) {
    Serial.println("  → Both interpretations plausible at this vibration level.");
    Serial.println("    Endianness cannot be conclusively determined from this sample.");
    Serial.println("    Repeat at higher vibration to widen the values.");
  } else {
    Serial.println("  *** → NEITHER interpretation is physically plausible!           ***");
    Serial.println("  *** This strongly suggests the register content is garbage.     ***");
    Serial.println("  *** Look for: firmware update cycle, wrong slave address,       ***");
    Serial.println("  *** register page switching, or partial write during read.      ***");
  }
}

// =============================================================================
// PHASE 4 – 100-SAMPLE STATISTICS
// =============================================================================
// Per-register accumulators for min/max/mean + suspicious-value counters.
// =============================================================================
struct RegStats {
  uint32_t count;
  uint32_t countSuspect;   // raw >= 0x8000 OR raw > 60000
  uint32_t countNegPeak;   // int16 < 0  (only meaningful for Peak regs)
  uint32_t countHighPeak;  // |scaled| > THRESH_PEAK_MAX_MMPS
  uint16_t valMin;
  uint16_t valMax;
  float    sum;            // sum of uint16 values
};

// 6 registers: [0]=VX [1]=VY [2]=VZ [3]=VRMSX [4]=VRMSY [5]=VRMSZ
static RegStats g_stats[6];
static uint32_t g_statsCount     = 0;
static uint32_t g_statsBadSample = 0;
static uint32_t g_statsCommFail  = 0;

void resetStats() {
  memset(g_stats, 0, sizeof(g_stats));
  for (uint8_t i = 0; i < 6; i++) {
    g_stats[i].valMin = 0xFFFF;
    g_stats[i].valMax = 0;
  }
  g_statsCount     = 0;
  g_statsBadSample = 0;
  g_statsCommFail  = 0;
}

void updateStats(uint8_t idx, uint16_t raw, bool isSigned) {
  RegStats &s = g_stats[idx];
  s.count++;
  s.sum += (float)raw;
  if (raw < s.valMin) s.valMin = raw;
  if (raw > s.valMax) s.valMax = raw;
  if (rawSuspicion(raw))              s.countSuspect++;
  if (isSigned && (int16_t)raw < 0)   s.countNegPeak++;
  if (isSigned) {
    float v = fabsf((int16_t)raw / 100.0f);
    if (v > THRESH_PEAK_MAX_MMPS)     s.countHighPeak++;
  }
}

void printStatsTable() {
  Serial.println("\n");
  Serial.println("╔═══════════════════════════════════════════════════════════════════╗");
  Serial.printf ("║  PHASE 4 – STATISTICS REPORT  (n=%u samples)%*s║\n",
                 g_statsCount, (int)(23 - (g_statsCount > 99 ? 3 : g_statsCount > 9 ? 2 : 1)), "");
  Serial.println("╚═══════════════════════════════════════════════════════════════════╝");
  Serial.printf ("  Communication failures: %u\n", g_statsCommFail);
  Serial.printf ("  Bad samples (any axis inconsistent): %u / %u  (%.1f%%)\n",
                 g_statsBadSample, g_statsCount,
                 g_statsCount ? 100.0f * g_statsBadSample / g_statsCount : 0.0f);
  Serial.println();

  const char* names[]    = { "VX",    "VY",    "VZ",    "VRMSX", "VRMSY", "VRMSZ" };
  bool        isSigned[] = { true,    true,    true,    false,   false,   false   };

  Serial.println("  Reg       n     Min    Max   Mean    Suspect  NegPeak  HighPeak");
  Serial.println("  ─────────────────────────────────────────────────────────────────");

  for (uint8_t i = 0; i < 6; i++) {
    RegStats &s = g_stats[i];
    if (s.count == 0) continue;
    float mean = s.sum / s.count;

    Serial.printf("  %-6s  %4u  %6u %6u %7.1f  %4u(%3.0f%%)  %4u(%3.0f%%)  %4u(%3.0f%%)\n",
                  names[i],
                  s.count,
                  s.valMin, s.valMax, mean,
                  s.countSuspect,  s.count ? 100.0f * s.countSuspect  / s.count : 0.0f,
                  s.countNegPeak,  s.count ? 100.0f * s.countNegPeak  / s.count : 0.0f,
                  s.countHighPeak, s.count ? 100.0f * s.countHighPeak / s.count : 0.0f);
  }

  Serial.println("  ─────────────────────────────────────────────────────────────────");
  Serial.println("  Columns: Suspect = raw>=0x8000 OR raw>60000");
  Serial.println("           NegPeak = int16 < 0 (Peak registers only)");
  Serial.println("           HighPeak = |scaled| > 50 mm/s (Peak registers only)");
  Serial.println();

  // ── Diagnosis summary ───────────────────────────────────────────────────────
  Serial.println("  ── DIAGNOSIS ───────────────────────────────────────────────────");

  bool highSuspectRate = false;
  for (uint8_t i = 0; i < 6; i++) {
    if (g_stats[i].count > 0) {
      float suspectRate = 100.0f * g_stats[i].countSuspect / g_stats[i].count;
      if (suspectRate > 5.0f) {
        highSuspectRate = true;
        Serial.printf("  [!] %s has %.1f%% suspicious values (raw>=0x8000 or >60000)\n",
                      names[i], suspectRate);
      }
    }
  }

  if (g_statsCommFail > 0) {
    float commFailRate = 100.0f * g_statsCommFail / (g_statsCount + g_statsCommFail);
    Serial.printf("  [!] %.1f%% communication failures → check wiring, termination,\n"
                  "      baud rate, slave address, and cable length.\n", commFailRate);
    Serial.println("      InvalidCRC (0xE3) = frame corruption on RS485 bus.");
    Serial.println("      ResponseTimedOut (0xE2) = sensor not responding.");
  }

  if (!highSuspectRate && g_statsCommFail == 0) {
    Serial.println("  All registers appear nominally clean over this sample window.");
    Serial.println("  If anomalies are intermittent, increase STATS_SAMPLES and");
    Serial.println("  introduce mechanical vibration to stress the sensor.");
  }

  Serial.println("═══════════════════════════════════════════════════════════════════");
}

// ── One statistics sample cycle ───────────────────────────────────────────────
bool runStatsSample() {
  uint8_t code;
  unsigned long ts, te;
  bool badSample = false;

  // T1: VX / VY / VZ
  code = timedRead(REG_VX, 3, ts, te);
  if (code != modbus.ku8MBSuccess) { g_statsCommFail++; return false; }
  uint16_t raw_vx = modbus.getResponseBuffer(0);
  uint16_t raw_vy = modbus.getResponseBuffer(1);
  uint16_t raw_vz = modbus.getResponseBuffer(2);
  delay(20);

  // T2: VRMSX
  code = timedRead(REG_VRMSX, 1, ts, te);
  if (code != modbus.ku8MBSuccess) { g_statsCommFail++; return false; }
  uint16_t raw_vrmsx = modbus.getResponseBuffer(0);
  delay(20);

  // T3: VRMSY
  code = timedRead(REG_VRMSY, 1, ts, te);
  if (code != modbus.ku8MBSuccess) { g_statsCommFail++; return false; }
  uint16_t raw_vrmsy = modbus.getResponseBuffer(0);
  delay(20);

  // T4: VRMSZ
  code = timedRead(REG_VRMSZ, 1, ts, te);
  if (code != modbus.ku8MBSuccess) { g_statsCommFail++; return false; }
  uint16_t raw_vrmsz = modbus.getResponseBuffer(0);

  // ── Update per-register statistics ──────────────────────────────────────────
  updateStats(0, raw_vx,    true);
  updateStats(1, raw_vy,    true);
  updateStats(2, raw_vz,    true);
  updateStats(3, raw_vrmsx, false);
  updateStats(4, raw_vrmsy, false);
  updateStats(5, raw_vrmsz, false);

  // ── Suspicious-value flags ───────────────────────────────────────────────────
  float vx    = (int16_t)raw_vx    / 100.0f;
  float vy    = (int16_t)raw_vy    / 100.0f;
  float vz    = (int16_t)raw_vz    / 100.0f;
  float vrmsx = raw_vrmsx / 1000.0f;
  float vrmsy = raw_vrmsy / 1000.0f;
  float vrmsz = raw_vrmsz / 1000.0f;

  // Print per-sample row (compact)
  Serial.printf(
    "  [%4u] VX=%6.3f VY=%6.3f VZ=%6.3f | VRMSX=%6.3f VRMSY=%6.3f VRMSZ=%6.3f",
    g_statsCount + 1, vx, vy, vz, vrmsx, vrmsy, vrmsz);

  // Inline flags
  bool anyFlag = false;
  if (raw_vx    >= 0x8000 || raw_vx    > THRESH_RAW_HIGH) { Serial.print(" [VX!]");    anyFlag = true; badSample = true; }
  if (raw_vy    >= 0x8000 || raw_vy    > THRESH_RAW_HIGH) { Serial.print(" [VY!]");    anyFlag = true; badSample = true; }
  if (raw_vz    >= 0x8000 || raw_vz    > THRESH_RAW_HIGH) { Serial.print(" [VZ!]");    anyFlag = true; badSample = true; }
  if (raw_vrmsx >= 0x8000 || raw_vrmsx > THRESH_RAW_HIGH) { Serial.print(" [VRMSX!]"); anyFlag = true; badSample = true; }
  if (raw_vrmsy >= 0x8000 || raw_vrmsy > THRESH_RAW_HIGH) { Serial.print(" [VRMSY!]"); anyFlag = true; badSample = true; }
  if (raw_vrmsz >= 0x8000 || raw_vrmsz > THRESH_RAW_HIGH) { Serial.print(" [VRMSZ!]"); anyFlag = true; badSample = true; }
  if (vrmsx > vx && vx > 0) { Serial.print(" [RMSX>PEAKX]"); anyFlag = true; badSample = true; }
  if (vrmsy > vy && vy > 0) { Serial.print(" [RMSY>PEAKY]"); anyFlag = true; badSample = true; }
  if (vrmsz > vz && vz > 0) { Serial.print(" [RMSZ>PEAKZ]"); anyFlag = true; badSample = true; }
  if (!anyFlag) Serial.print(" OK");
  Serial.println();

  g_statsCount++;
  if (badSample) g_statsBadSample++;
  return true;
}

// =============================================================================
// setup()
// =============================================================================
void setup() {
  Serial.begin(115200);
  delay(2000);

  // ── Banner ──────────────────────────────────────────────────────────────────
  Serial.println();
  Serial.println("╔═══════════════════════════════════════════════════════════════════╗");
  Serial.println("║  WTVB02 CORRUPTION DIAGNOSTIC  v3.0                              ║");
  Serial.println("║  Goal: identify root cause of impossible register values          ║");
  Serial.println("╠═══════════════════════════════════════════════════════════════════╣");
  Serial.println("║  Known symptoms:                                                  ║");
  Serial.println("║    VX RAW = 35349 (0x8A15) → int16 = -30187 → -301.87 mm/s       ║");
  Serial.println("║    VRMSY RAW = 38684 (0x971C) → 38.684 mm/s >> Peak              ║");
  Serial.println("║  Ruled out: asynchronous update within 130 ms read window         ║");
  Serial.println("╠═══════════════════════════════════════════════════════════════════╣");
  Serial.println("║  Investigation:                                                   ║");
  Serial.println("║    Phase 1 – Single-register stability (10× burst per register)   ║");
  Serial.println("║    Phase 2 – Burst block read (0x003A–0x0068, 47 registers)       ║");
  Serial.println("║    Phase 3 – Endianness / byte-order probe                        ║");
  Serial.println("║    Phase 4 – 100-sample statistics with anomaly counters          ║");
  Serial.println("╚═══════════════════════════════════════════════════════════════════╝");
  Serial.println();
  Serial.println("  Suspicious value thresholds:");
  Serial.printf ("    Peak > %.0f mm/s → THRESH_PEAK_MAX_MMPS\n",  THRESH_PEAK_MAX_MMPS);
  Serial.printf ("    Raw uint16 > %u  → THRESH_RAW_HIGH\n",       THRESH_RAW_HIGH);
  Serial.printf ("    Peak/RMS < %.1f or > %.1f → THRESH_RATIO\n", THRESH_RATIO_MIN, THRESH_RATIO_MAX);
  Serial.println("  Modbus status codes printed for every transaction.");
  Serial.println("  0xE3 = InvalidCRC → frame corruption.");
  Serial.println("  0xE2 = ResponseTimedOut → no response.");
  Serial.println();

  // ── Hardware init ────────────────────────────────────────────────────────────
  pinMode(RS485_EN_PIN, OUTPUT);
  rs485Enable();    // DE/RE LOW always

  SerialRS485.begin(MODBUS_BAUDRATE, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);
  modbus.begin(MODBUS_SLAVE_ID, SerialRS485);

  delay(1000);

  waitForSensor();

  // ── Run one-time diagnostic phases ──────────────────────────────────────────
  runPhase1_StabilityTest();
  delay(500);

  runPhase2_BurstBlockRead();
  delay(500);

  runPhase3_EndianessProbe();
  delay(500);

  // ── Begin Phase 4 statistics collection ─────────────────────────────────────
  Serial.println("\n");
  Serial.println("╔═══════════════════════════════════════════════════════════════╗");
  Serial.println("║  PHASE 4 – CONTINUOUS STATISTICS  (100 samples then report)  ║");
  Serial.println("║  Format: [n] VX VY VZ | VRMSX VRMSY VRMSZ  [flags]          ║");
  Serial.println("╚═══════════════════════════════════════════════════════════════╝");

  resetStats();
}

// =============================================================================
// loop()
// =============================================================================
void loop() {
  static unsigned long lastSample = 0;

  if (millis() - lastSample < STATS_INTERVAL_MS) return;
  lastSample = millis();

  runStatsSample();

  // ── Print statistics table every STATS_SAMPLES samples, then reset ──────────
  if (g_statsCount >= STATS_SAMPLES) {
    printStatsTable();
    delay(3000);
    resetStats();
    Serial.println("  [Restarting statistics window...]\n");
  }
}
