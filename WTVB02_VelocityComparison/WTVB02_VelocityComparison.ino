// ============================================================================
// WTVB02_VelocityComparison.ino
// ----------------------------------------------------------------------------
// Reads Velocity Amplitude (Peak) registers and Velocity RMS registers from
// the WTVB02 vibration sensor via RS485 Modbus RTU, then prints a side-by-side
// comparison every 2 seconds.
//
// Hardware : LilyGO T-Vending S3  (ESP32-S3)
// Library  : ModbusMaster
// ============================================================================

#include <ModbusMaster.h>

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

// ── Read interval ─────────────────────────────────────────────────────────────
#define READ_INTERVAL_MS  2000

// ── Peripheral objects ────────────────────────────────────────────────────────
HardwareSerial SerialRS485(2);
ModbusMaster   modbus;

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
};

// ─────────────────────────────────────────────────────────────────────────────
// rs485Enable()
// Keep DE/RE line asserted LOW → transmit/receive always active (no callback).
// ─────────────────────────────────────────────────────────────────────────────
void rs485Enable() {
  digitalWrite(RS485_EN_PIN, LOW);
}

// ─────────────────────────────────────────────────────────────────────────────
// readVelocityComparison()
//
// Performs two Modbus read operations:
//   1. Holding registers 0x3A–0x3C  → Velocity Amplitude X/Y/Z
//   2. Three individual reads for VRMSX (0x50), VRMSY (0x5C), VRMSZ (0x68)
//      (registers are non-contiguous, so each is read separately)
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
  Serial.println(" WTVB02 Velocity Amplitude vs Velocity RMS  v1.0");
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
  Serial.println("[INIT] Sensor OK – starting continuous monitoring.\n");
}

// ─────────────────────────────────────────────────────────────────────────────
// loop()
// ─────────────────────────────────────────────────────────────────────────────
void loop() {
  static unsigned long lastRead = 0;

  if (millis() - lastRead >= READ_INTERVAL_MS) {
    lastRead = millis();

    VelocityData data;
    if (readVelocityComparison(data)) {
      printVelocityReport(data);
    } else {
      Serial.println("[WARN] Read cycle failed – will retry next interval.");
    }
  }
}
