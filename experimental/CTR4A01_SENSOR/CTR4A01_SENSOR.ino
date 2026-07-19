// ============================================================================
// CTR4A01 Current Sensor Test  v2
// ============================================================================
// อ่านค่ากระแส AC จากเซนเซอร์ CTR4A01 ผ่าน RS485/Modbus RTU
// RS485 enable ค้างตลอด (ไม่ใช้ callback toggle)
//
// Model: CTR4A01 (5A) -- ย่านวัดกระแส 0.05-5A (ค่าดิบ 0-5000 mA)
// ============================================================================

#include <ModbusMaster.h>

// ── Pins (LilyGO T-Vending S3) ───────────────────────────────────────────────
#define RS485_TX_PIN   39
#define RS485_RX_PIN   38
#define RS485_EN_PIN   42

// ── Modbus (CTR4A01 current sensor) ─────────────────────────────────────────
#define CURRENT_SENSOR_ID     0x01     // slave address (แก้ให้ตรงกับตัวจริง)
#define MODBUS_BAUDRATE       9600     // default ของ CTR4A01
#define CT_REG_AC_CURRENT     0x0000   // function 04, หน่วย mA (30001)
#define CT_REG_CALIBRATION    0x00C0   // function 03/06, current correction value (mA)
#define CT_REG_FACTORY_RESET  0x00FB   // function 06 (broadcast 0xFF), factory reset

// ── Sensor range (5A model) ──────────────────────────────────────────────────
#define CT_RANGE_MIN_MA        50      // 0.05A
#define CT_RANGE_MAX_MA        5000    // 5A

// ── Sample rate ───────────────────────────────────────────────────────────────
#define SAMPLE_RATE_HZ         2       // ความถี่การอ่าน/แสดงผล (ครั้ง/วินาที)
#define SAMPLE_INTERVAL_MS     (1000 / SAMPLE_RATE_HZ)   // = 500ms ที่ 2Hz

HardwareSerial SerialRS485(2);
ModbusMaster   modbus;

// ── CT Ratio (สำหรับ external CT clamp ที่ต่อก่อนเข้า CTR4A01) ──────────────
// เช่น CT clamp ภายนอกอัตราส่วน 100/5 (primary 100A : secondary 5A)
// ค่าที่ CTR4A01 วัดได้คือกระแสด้าน secondary (0-5A) ต้องคูณ ratio เพื่อได้ค่ากระแสจริง
float ctRatioNum = 1;      // primary  (default 1 = ไม่มี external CT, วัดตรง)
float ctRatioDen = 1;      // secondary
float ctRatio    = 1.0;    // = ctRatioNum / ctRatioDen

void rs485Enable()  { digitalWrite(RS485_EN_PIN, LOW);  }
void rs485Disable() { digitalWrite(RS485_EN_PIN, HIGH); }

// ── Helper: print result ──────────────────────────────────────────────────────
const char* ok(uint8_t r) { return r == modbus.ku8MBSuccess ? "+ OK" : "x FAIL"; }

// =============================================================================
// ตรวจว่าเซนเซอร์กระแสตอบสนองหรือไม่
// =============================================================================
bool checkCurrentSensorAlive() {
  Serial.println("\n[CHECK] อ่าน AC current (reg 0x0000, FC=04) -- ตรวจ CTR4A01 alive...");
  if (modbus.readInputRegisters(CT_REG_AC_CURRENT, 1) == modbus.ku8MBSuccess) {
    uint16_t raw = modbus.getResponseBuffer(0);
    Serial.printf("  + CTR4A01 OK! Current = %u mA\n", raw);
    return true;
  } else {
    Serial.println("  x CTR4A01 FAILED -- ตรวจ wiring/address/baudrate");
    return false;
  }
}

// =============================================================================
// อ่านค่ากระแส AC (หน่วย mA) -- คืนค่า true ถ้าอ่านสำเร็จ
// ค่าที่ได้อยู่ในย่าน 0-5000 mA (รุ่น 5A) ถ้าเกินย่านวัดจริงค่าจะ clip ที่ 5000
// =============================================================================
bool readCurrentSensor(uint16_t &milliAmps) {
  uint8_t r = modbus.readInputRegisters(CT_REG_AC_CURRENT, 1);
  bool success = (r == modbus.ku8MBSuccess);
  if (success) milliAmps = modbus.getResponseBuffer(0);
  return success;
}

// =============================================================================
// ตั้งค่า Current Correction (reg 0x00C0) -- calibrate ค่ากระแสให้ตรงกับค่าจริง
// ใช้เมื่อค่าที่วัดได้คลาดเคลื่อนจากค่าจริง (วัดด้วยเครื่องมือมาตรฐาน) เกิน 1%
// ตามคู่มือ: ใส่ "ค่ากระแสจริง" (หน่วย mA) ลงในรีจิสเตอร์นี้ ไม่ใช่ค่า offset
// =============================================================================
bool setCalibration(uint16_t actualMA) {
  uint8_t r = modbus.writeSingleRegister(CT_REG_CALIBRATION, actualMA);
  bool success = (r == modbus.ku8MBSuccess);
  Serial.printf("  [SET] Calibration = %u mA ... %s\n", actualMA, ok(r));
  return success;
}

// =============================================================================
// อ่านค่า Current Correction ปัจจุบัน (reg 0x00C0)
// ค่า 0xFFFF = ยังไม่เคย calibrate (ค่า default จากโรงงาน)
// =============================================================================
void readCalibration() {
  if (modbus.readHoldingRegisters(CT_REG_CALIBRATION, 1) == modbus.ku8MBSuccess) {
    uint16_t v = modbus.getResponseBuffer(0);
    if (v == 0xFFFF)
      Serial.println("  [READ] Calibration = ยังไม่ได้ตั้งค่า (default)");
    else
      Serial.printf("  [READ] Calibration = %u mA\n", v);
  } else {
    Serial.println("  [READ] Calibration: FAILED");
  }
}

// =============================================================================
// Factory Reset ผ่าน Modbus (broadcast address 0xFF)
// ตามคู่มือ: FF 06 00 FB 00 00 ED E5
// ใช้ได้เมื่อมีเซนเซอร์ตัวเดียวบนบัสเท่านั้น! ต้อง power cycle เซนเซอร์หลังทำ
// =============================================================================
bool factoryReset() {
  Serial.println("  [RESET] ส่งคำสั่ง Factory Reset (broadcast FF, reg 0x00FB)...");
  Serial.println("          (ใช้ได้เมื่อมีเซนเซอร์ตัวเดียวบนบัสเท่านั้น)");

  modbus.begin(0xFF, SerialRS485);
  uint8_t r = modbus.writeSingleRegister(CT_REG_FACTORY_RESET, 0x0000);
  bool success = (r == modbus.ku8MBSuccess);
  Serial.printf("  ... %s\n", ok(r));
  modbus.begin(CURRENT_SENSOR_ID, SerialRS485);   // กลับมา address เดิม

  if (success)
    Serial.println("  >> สำคัญ: ต้อง POWER CYCLE เซนเซอร์ใหม่ (ปิด-เปิดไฟ) ค่าที่ล้างถึงจะมีผลจริง");

  return success;
}

// =============================================================================
// ตรวจคำสั่งจาก Serial Monitor แบบไม่บล็อก (non-blocking)
// พิมพ์ "CT 100/5" Enter = ตั้งอัตราส่วน external CT เป็น 100A/5A
// พิมพ์ "CT?" Enter      = อ่านค่า ratio ปัจจุบัน
// พิมพ์ "CAL 1000" Enter = calibrate โดยใส่ค่ากระแสจริง (mA) ที่วัดได้จากเครื่องมือมาตรฐาน
// พิมพ์ "CAL?" Enter     = อ่านค่า calibration ปัจจุบัน
// พิมพ์ "RESET" Enter    = สั่ง Factory Reset เซนเซอร์ผ่าน Modbus (ต้อง power cycle ต่อ)
// =============================================================================
void checkSerialCommands() {
  if (!Serial.available()) return;

  String cmd = Serial.readStringUntil('\n');
  cmd.trim();
  if (cmd.length() == 0) return;

  Serial.printf("\n[CMD] รับคำสั่ง: \"%s\"\n", cmd.c_str());

  if (cmd.equalsIgnoreCase("ct?")) {
    Serial.printf("  [READ] CT Ratio ปัจจุบัน = %.0f/%.0f (x%.3f)\n", ctRatioNum, ctRatioDen, ctRatio);

  } else if (cmd.startsWith("CT ") || cmd.startsWith("ct ")) {
    String param = cmd.substring(3);
    param.trim();
    int slashIdx = param.indexOf('/');

    if (slashIdx > 0) {
      float primary   = param.substring(0, slashIdx).toFloat();
      float secondary = param.substring(slashIdx + 1).toFloat();

      if (primary > 0 && secondary > 0) {
        ctRatioNum = primary;
        ctRatioDen = secondary;
        ctRatio    = primary / secondary;
        Serial.printf("  [SET] CT Ratio = %.0f/%.0f (x%.3f) -- ค่ากระแสที่แสดงจะถูกคูณด้วยตัวนี้\n",
                      ctRatioNum, ctRatioDen, ctRatio);
      } else {
        Serial.println("  x ค่าไม่ถูกต้อง -- ตัวอย่าง: CT 100/5");
      }
    } else {
      Serial.println("  x รูปแบบผิด -- ตัวอย่าง: CT 100/5");
    }

  } else if (cmd.equalsIgnoreCase("cal?")) {
    readCalibration();

  } else if (cmd.startsWith("CAL ") || cmd.startsWith("cal ")) {
    String param = cmd.substring(4);
    param.trim();
    long actualMA = param.toInt();

    if (actualMA > 0 && actualMA <= 65535) {
      setCalibration((uint16_t)actualMA);
      readCalibration();
    } else {
      Serial.println("  x ค่าไม่ถูกต้อง -- ตัวอย่าง: CAL 1000  (ใส่ค่ากระแสจริงหน่วย mA)");
    }

  } else if (cmd.equalsIgnoreCase("reset")) {
    Serial.println("  ⚠ คำสั่งนี้จะล้างค่า calibration/config ทั้งหมดกลับเป็นค่าโรงงาน");
    Serial.println("  พิมพ์ \"RESET YES\" เพื่อยืนยัน (ป้องกันกดพลาด)");

  } else if (cmd.equalsIgnoreCase("reset yes")) {
    factoryReset();

  } else {
    Serial.println("  คำสั่งไม่รู้จัก -- พิมพ์ \"CT <primary>/<secondary>\", \"CT?\", \"CAL <mA>\", \"CAL?\", หรือ \"RESET\"");
  }
}

// =============================================================================
// CONTINUOUS MONITOR -- non-blocking timer ด้วย millis() ป้องกัน drift สะสม
// =============================================================================
void monitor() {
  Serial.printf("\n[MONITOR] Current ทุก %dms (%dHz) -- RST to restart\n", SAMPLE_INTERVAL_MS, SAMPLE_RATE_HZ);
  Serial.println("  พิมพ์คำสั่งใน Serial Monitor:");
  Serial.println("    \"CT 100/5\" = ตั้งอัตราส่วน external CT | \"CT?\" = อ่านค่าปัจจุบัน");
  Serial.println("    \"CAL 1000\" = calibrate ใส่ค่ากระแสจริง(mA) | \"CAL?\" = อ่านค่า calibration ปัจจุบัน");
  Serial.println("    \"RESET\" = Factory Reset (ต้องพิมพ์ \"RESET YES\" ยืนยันอีกครั้ง)");
  Serial.println("  #    Current(mA)");
  Serial.println("  ─────────────────");

  int n = 0;
  unsigned long nextSample = millis();   // เวลาของรอบถัดไป -- อิงจากจุดเริ่มต้น ไม่สะสม drift

  while (true) {
    checkSerialCommands();   // ตรวจคำสั่งจาก Serial Monitor ทุกครั้งที่ผ่านลูป (ไม่บล็อก)

    if ((long)(millis() - nextSample) >= 0) {
      // ถึงกำหนดรอบอ่านแล้ว
      nextSample += SAMPLE_INTERVAL_MS;   // กำหนดรอบถัดไปจาก "เวลาที่ควรจะเป็น" ไม่ใช่ millis() ปัจจุบัน -- กัน drift สะสม

      n++;
      uint16_t currentMA = 0;
      bool curOK = readCurrentSensor(currentMA);

      if (curOK) {
        if (ctRatio != 1.0) {
          float actualA = (currentMA / 1000.0f) * ctRatio;
          Serial.printf("  %04d  %u mA (raw)  ->  %.2f A (actual, CT %.0f/%.0f)\n",
                        n, currentMA, actualA, ctRatioNum, ctRatioDen);
        } else {
          Serial.printf("  %04d  %u mA\n", n, currentMA);
        }
      } else {
        Serial.printf("  %04d  FAIL\n", n);
      }

      // ถ้าการอ่าน Modbus กินเวลานานจนเลยรอบถัดไปไปแล้ว ให้ข้ามรอบที่พลาดไปเลย
      // (กันไม่ให้ loop วิ่งรัวเพื่อ "ตามให้ทัน" ซึ่งจะทำให้ค่าที่ได้ไม่สม่ำเสมอ)
      while ((long)(millis() - nextSample) >= 0) {
        nextSample += SAMPLE_INTERVAL_MS;
      }
    }
  }
}

// =============================================================================
void setup() {
  Serial.begin(115200);
  delay(2000);

  Serial.println("========================================");
  Serial.println(" CTR4A01 Current Sensor Test  v2");
  Serial.println(" Model: CTR4A01 (5A) -- range 0.05-5A");
  Serial.println(" RS485: EN=LOW (always enabled)");
  Serial.printf (" Current Sensor ID=0x%02X\n", CURRENT_SENSOR_ID);
  Serial.println("========================================");

  pinMode(RS485_EN_PIN, OUTPUT);
  rs485Enable();  // enable ค้างตลอด

  SerialRS485.begin(MODBUS_BAUDRATE, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);
  modbus.begin(CURRENT_SENSOR_ID, SerialRS485);
  // ไม่ใช้ preTransmission/postTransmission callback

  delay(1000);

  // ตรวจ alive ก่อน
  if (!checkCurrentSensorAlive()) {
    Serial.println("\nWARNING: Sensor not responding -- ตรวจ power/wiring/address/baudrate");
    Serial.println("จะ retry ทุก 3s...");
    while (!checkCurrentSensorAlive()) delay(3000);
  }

  readCalibration();   // แสดงค่า calibration ปัจจุบัน (ถ้าเคยตั้งไว้)

  monitor();
}

void loop() {}
