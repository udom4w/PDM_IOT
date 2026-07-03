// ============================================================================
// WTVB02 Sensor Reset & CF Test  v2
// ============================================================================
// แก้: RS485 enable ค้างตลอด (ไม่ใช้ callback toggle)
//       เหมือน firmware หลักที่ใช้งานได้จริง
// ============================================================================

#include <ModbusMaster.h>

// ── Pins (LilyGO T-Vending S3) ───────────────────────────────────────────────
#define RS485_TX_PIN   39
#define RS485_RX_PIN   38
#define RS485_EN_PIN   42

// ── Modbus ───────────────────────────────────────────────────────────────────
#define MODBUS_SLAVE_ID   0x50
#define MODBUS_BAUDRATE   9600

HardwareSerial SerialRS485(2);
ModbusMaster   modbus;

void rs485Enable()  { digitalWrite(RS485_EN_PIN, LOW);  }
void rs485Disable() { digitalWrite(RS485_EN_PIN, HIGH); }

// ── Helper: print result ──────────────────────────────────────────────────────
const char* ok(uint8_t r) { return r == modbus.ku8MBSuccess ? "+ OK" : "x FAIL"; }

// =============================================================================
// อ่าน VX/VY/VZ -- ตรวจว่า Modbus ทำงานได้ก่อน
// =============================================================================
bool checkModbusAlive() {
  Serial.println("\n[CHECK] อ่าน VX/VY/VZ (reg 0x3A) -- ตรวจ Modbus alive...");
  if (modbus.readHoldingRegisters(0x003A, 3) == modbus.ku8MBSuccess) {
    float vx = (int16_t)modbus.getResponseBuffer(0) / 100.0f;
    float vy = (int16_t)modbus.getResponseBuffer(1) / 100.0f;
    float vz = (int16_t)modbus.getResponseBuffer(2) / 100.0f;
    Serial.printf("  + Modbus OK! VX=%.2f VY=%.2f VZ=%.2f mm/s\n", vx, vy, vz);
    return true;
  } else {
    Serial.println("  x Modbus FAILED -- ตรวจ wiring/power");
    return false;
  }
}

// =============================================================================
// STEP 1: อ่าน config ปัจจุบัน
// =============================================================================
void readConfig() {
  Serial.println("\n[CONFIG] Current sensor config:");

  // SR
  if (modbus.readHoldingRegisters(0x0029, 1) == modbus.ku8MBSuccess) {
    uint16_t v = modbus.getResponseBuffer(0);
    const char* name = (v==0?"32K": v==1?"16K": v==2?"8K": v==7?"256Hz(def)":"unknown");
    Serial.printf("  SR  (0x29) = 0x%04X = %s\n", v, name);
  } else {
    Serial.println("  SR  (0x29): READ FAILED");
  }
  delay(50);

  // MODE
  if (modbus.readHoldingRegisters(0x0007, 1) == modbus.ku8MBSuccess) {
    uint16_t v = modbus.getResponseBuffer(0);
    const char* name = (v==0?"LowFreq(def)": v==1?"HighFreq(CF)": v==2?"FreqDomain":"unknown");
    Serial.printf("  MODE(0x07) = 0x%04X = %s\n", v, name);
  } else {
    Serial.println("  MODE(0x07): READ FAILED (sensor อาจไม่ support)");
  }
  delay(50);

  // TEMP -- อ่าน verify
  if (modbus.readHoldingRegisters(0x0040, 1) == modbus.ku8MBSuccess) {
    Serial.printf("  TEMP(0x40) = %.1f C\n", (int16_t)modbus.getResponseBuffer(0)/100.0f);
  }
  delay(50);
}

// =============================================================================
// STEP 2: อ่าน CF raw
// =============================================================================
bool readCF(const char* label) {
  Serial.printf("\n[CF] %s\n", label);
  bool any = false;

  if (modbus.readHoldingRegisters(0x0047, 6) == modbus.ku8MBSuccess) {
    uint16_t cfx=modbus.getResponseBuffer(0), kx=modbus.getResponseBuffer(1);
    uint16_t aavgx=modbus.getResponseBuffer(2), varx=modbus.getResponseBuffer(3);
    uint16_t rrax=modbus.getResponseBuffer(4), wix=modbus.getResponseBuffer(5);
    Serial.printf("  X raw: CFX=0x%04X KX=0x%04X AAVGX=0x%04X VARX=0x%04X RRAX=0x%04X WIX=0x%04X\n",
                  cfx, kx, aavgx, varx, rrax, wix);
    Serial.printf("  X val: cfx=%.3f kx=%.3f\n", cfx/1000.0f, kx/1000.0f);
    if (cfx || kx) any = true;
  } else {
    Serial.println("  X (0x47~4C): FAILED");
  }
  delay(50);

  if (modbus.readHoldingRegisters(0x0053, 2) == modbus.ku8MBSuccess) {
    uint16_t cfy=modbus.getResponseBuffer(0), ky=modbus.getResponseBuffer(1);
    Serial.printf("  Y raw: CFY=0x%04X KY=0x%04X\n", cfy, ky);
    if (cfy || ky) any = true;
  } else {
    Serial.println("  Y (0x53~54): FAILED");
  }
  delay(50);

  if (modbus.readHoldingRegisters(0x005F, 2) == modbus.ku8MBSuccess) {
    uint16_t cfz=modbus.getResponseBuffer(0), kz=modbus.getResponseBuffer(1);
    Serial.printf("  Z raw: CFZ=0x%04X KZ=0x%04X\n", cfz, kz);
    if (cfz || kz) any = true;
  } else {
    Serial.println("  Z (0x5F~60): FAILED");
  }
  delay(50);

  if (modbus.readHoldingRegisters(0x0050, 1) == modbus.ku8MBSuccess) {
    uint16_t v = modbus.getResponseBuffer(0);
    Serial.printf("  VRMSX(0x50) = 0x%04X (%.3f mm/s)\n", v, v/1000.0f);
    if (v) any = true;
  }
  delay(50);

  if (any)
    Serial.println("  *** CF HAS NON-ZERO VALUES! ***");
  else
    Serial.println("  (all zero)");

  return any;
}

// =============================================================================
// STEP 3: ทดสอบทุก MODE (0x00, 0x01, 0x02) + Reboot แต่ละครั้ง
// =============================================================================
void testOneMode(uint8_t modeVal, const char* modeName) {
  Serial.printf("\n╔══════════════════════════════════════════╗\n");
  Serial.printf("║ TEST MODE=0x%02X (%s)\n", modeVal, modeName);
  Serial.printf("╚══════════════════════════════════════════╝\n");

  uint8_t r;

  // ── Unlock → SR=16K (unlock ใหม่ก่อนทุก write) ──
  Serial.print("  [Unlock#1]... ");
  r = modbus.writeSingleRegister(0x0069, 0xB588);
  Serial.println(ok(r));
  if (r != modbus.ku8MBSuccess) { Serial.println("  ABORT"); return; }
  delay(300);

  Serial.print("  SR=16K... ");
  r = modbus.writeSingleRegister(0x0029, 0x0001);
  Serial.println(ok(r));
  delay(100);

  // ── Unlock ใหม่ก่อน write MODE ──
  Serial.print("  [Unlock#2]... ");
  r = modbus.writeSingleRegister(0x0069, 0xB588);
  Serial.println(ok(r));
  delay(300);

  Serial.printf("  MODE=0x%02X (%s)... ", modeVal, modeName);
  r = modbus.writeSingleRegister(0x0007, modeVal);
  Serial.println(ok(r));
  delay(100);

  // Verify MODE ก่อน save
  if (modbus.readHoldingRegisters(0x0007, 1) == modbus.ku8MBSuccess)
    Serial.printf("  MODE readback=0x%04X\n", modbus.getResponseBuffer(0));
  else
    Serial.println("  MODE readback: n/a");
  delay(100);

  // ── Unlock ใหม่ก่อน Save ──
  Serial.print("  [Unlock#3]... ");
  r = modbus.writeSingleRegister(0x0069, 0xB588);
  Serial.println(ok(r));
  delay(300);

  Serial.print("  Save... ");
  r = modbus.writeSingleRegister(0x0000, 0x0000);
  Serial.println(ok(r));
  delay(300);

  // ── Unlock ใหม่ก่อน Reboot ──
  Serial.print("  [Unlock#4]... ");
  r = modbus.writeSingleRegister(0x0069, 0xB588);
  Serial.println(ok(r));
  delay(300);

  Serial.print("  Reboot... ");
  r = modbus.writeSingleRegister(0x0000, 0x00FF);
  Serial.printf("%s  waiting 5s...\n", ok(r));
  delay(5000);

  // Check alive
  bool alive = false;
  for (int i = 0; i < 3 && !alive; i++) {
    if (modbus.readHoldingRegisters(0x003A, 3) == modbus.ku8MBSuccess) {
      float vx = (int16_t)modbus.getResponseBuffer(0)/100.0f;
      Serial.printf("  + Alive! VX=%.2f\n", vx);
      alive = true;
    } else {
      Serial.println("  x not yet, wait 1s...");
      delay(1000);
    }
  }

  if (!alive) { Serial.println("  x Sensor not responding"); return; }

  // Verify MODE saved
  if (modbus.readHoldingRegisters(0x0007, 1) == modbus.ku8MBSuccess)
    Serial.printf("  MODE verify = 0x%04X\n", modbus.getResponseBuffer(0));
  delay(50);

  // อ่าน CF 5 ครั้ง
  Serial.printf("  Reading CF 5 times (2s apart):\n");
  bool anyNonZero = false;
  for (int i = 0; i < 5; i++) {
    uint16_t cfx=0, kx=0, cfy=0, ky=0, cfz=0, kz=0, vrmsx=0;
    if (modbus.readHoldingRegisters(0x0047,2)==modbus.ku8MBSuccess) {
      cfx=modbus.getResponseBuffer(0); kx=modbus.getResponseBuffer(1);
    }
    delay(30);
    if (modbus.readHoldingRegisters(0x0053,2)==modbus.ku8MBSuccess) {
      cfy=modbus.getResponseBuffer(0); ky=modbus.getResponseBuffer(1);
    }
    delay(30);
    if (modbus.readHoldingRegisters(0x005F,2)==modbus.ku8MBSuccess) {
      cfz=modbus.getResponseBuffer(0); kz=modbus.getResponseBuffer(1);
    }
    delay(30);
    if (modbus.readHoldingRegisters(0x0050,1)==modbus.ku8MBSuccess)
      vrmsx=modbus.getResponseBuffer(0);

    bool has = (cfx||kx||cfy||ky||cfz||kz||vrmsx);
    if (has) anyNonZero = true;

    Serial.printf("    #%d  CFX=0x%04X KX=0x%04X CFY=0x%04X KY=0x%04X CFZ=0x%04X VRMS=0x%04X %s\n",
      i+1, cfx, kx, cfy, ky, cfz, vrmsx, has?"<<< HAS VALUE!":"");
    delay(2000);
  }

  Serial.printf("  RESULT MODE=0x%02X: %s\n",
    modeVal, anyNonZero ? "*** CF HAS VALUES! ***" : "all zero");
}

void doConfigAndReboot() {
  // ทดสอบ MODE=0x00 (Low freq default)
  testOneMode(0x00, "LowFreq-default");

  // ทดสอบ MODE=0x01 (High freq)
  testOneMode(0x01, "HighFreq-CF");

  // ทดสอบ MODE=0x02 (Frequency domain)
  testOneMode(0x02, "FreqDomain");

  Serial.println("\n╔══════════════════════════════════════╗");
  Serial.println("║  ALL MODE TESTS DONE                 ║");
  Serial.println("╚══════════════════════════════════════╝");
}

// =============================================================================
// CONTINUOUS MONITOR
// =============================================================================
void monitor() {
  Serial.println("\n[MONITOR] CF every 2s -- RST to restart");
  Serial.println("  #    CFX     KX    CFY     KY    CFZ     KZ   VRMSX");
  Serial.println("  ─────────────────────────────────────────────────────");

  int n = 0;
  while (true) {
    n++;
    float cfx=0,kx=0,cfy=0,ky=0,cfz=0,kz=0,vx=0;

    if (modbus.readHoldingRegisters(0x0047,2)==modbus.ku8MBSuccess) {
      cfx=modbus.getResponseBuffer(0)/1000.0f;
      kx =modbus.getResponseBuffer(1)/1000.0f;
    }
    delay(30);
    if (modbus.readHoldingRegisters(0x0053,2)==modbus.ku8MBSuccess) {
      cfy=modbus.getResponseBuffer(0)/1000.0f;
      ky =modbus.getResponseBuffer(1)/1000.0f;
    }
    delay(30);
    if (modbus.readHoldingRegisters(0x005F,2)==modbus.ku8MBSuccess) {
      cfz=modbus.getResponseBuffer(0)/1000.0f;
      kz =modbus.getResponseBuffer(1)/1000.0f;
    }
    delay(30);
    if (modbus.readHoldingRegisters(0x0050,1)==modbus.ku8MBSuccess)
      vx=modbus.getResponseBuffer(0)/1000.0f;

    bool any = (cfx||kx||cfy||ky||cfz||kz||vx);
    Serial.printf("  %03d  %5.3f  %5.3f  %5.3f  %5.3f  %5.3f  %5.3f  %5.3f  %s\n",
      n, cfx, kx, cfy, ky, cfz, kz, vx, any?"<<< HAS VALUE":"");

    delay(2000);
  }
}

// =============================================================================
void setup() {
  Serial.begin(115200);
  delay(2000);

  Serial.println("========================================");
  Serial.println(" WTVB02 Sensor Reset & CF Test  v2");
  Serial.println(" RS485: EN=LOW (always enabled)");
  Serial.println("========================================");

  pinMode(RS485_EN_PIN, OUTPUT);
  rs485Enable();  // enable ค้างตลอด -- เหมือน firmware หลัก

  SerialRS485.begin(MODBUS_BAUDRATE, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);
  modbus.begin(MODBUS_SLAVE_ID, SerialRS485);
  // ไม่ใช้ preTransmission/postTransmission callback

  delay(1000);

  // ตรวจ alive ก่อน
  if (!checkModbusAlive()) {
    Serial.println("\nWARNING: Sensor not responding -- ตรวจ power/wiring");
    Serial.println("จะ retry ทุก 3s...");
    while (!checkModbusAlive()) delay(3000);
  }

  readConfig();
  readCF("INITIAL -- before any change");
  doConfigAndReboot();
  monitor();
}

void loop() {}
