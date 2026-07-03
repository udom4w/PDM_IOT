// ============================================================
//  RTC DS3231 Diagnostic Test
//  สำหรับ LilyGO T-Vending S3 (ESP32-S3)
//  SDA = GPIO44, SCL = GPIO43
//  ทดสอบ: I2C ping, lostPower, อุณหภูมิ, เขียน/อ่านเวลา
// ============================================================

#include <Wire.h>
#include <RTClib.h>

#define I2C_SDA_PIN  44
#define I2C_SCL_PIN  43
#define DS3231_ADDR  0x68   // I2C address มาตรฐาน

RTC_DS3231 rtc;

// ── ทดสอบ 1: I2C ping ──────────────────────────────────────
bool testI2CPing() {
  Serial.println("\n[TEST 1] I2C Ping DS3231 @ 0x68");
  Wire.beginTransmission(DS3231_ADDR);
  uint8_t err = Wire.endTransmission();
  if (err == 0) {
    Serial.println("  + PASS: DS3231 ตอบสนองบน I2C bus");
    return true;
  } else {
    Serial.printf("  x FAIL: ไม่พบ DS3231 (err=%u)\n", err);
    Serial.println("  → ตรวจสอบการต่อสาย SDA/SCL และไฟเลี้ยง 3.3V");
    return false;
  }
}

// ── ทดสอบ 2: lostPower ─────────────────────────────────────
void testLostPower() {
  Serial.println("\n[TEST 2] lostPower() — ตรวจ battery backup");
  if (rtc.lostPower()) {
    Serial.println("  x FAIL: lostPower = YES");
    Serial.println("  → Battery หมดหรือ IC เสีย เวลาจะ reset ทุกครั้งที่ไฟดับ");
  } else {
    Serial.println("  + PASS: lostPower = NO — battery ดี");
  }
}

// ── ทดสอบ 3: อุณหภูมิภายใน ──────────────────────────────
void testTemperature() {
  Serial.println("\n[TEST 3] DS3231 Internal Temperature");
  float temp = rtc.getTemperature();
  Serial.printf("  Temp: %.2f °C\n", temp);
  if (temp > 0.0f && temp < 85.0f) {
    Serial.println("  + PASS: อุณหภูมิอยู่ในช่วงปกติ");
  } else {
    Serial.println("  x FAIL: อุณหภูมิผิดปกติ — อาจเป็น I2C error หรือ IC เสีย");
  }
}

// ── ทดสอบ 4: เขียนเวลาและอ่านกลับ ──────────────────────
void testWriteRead() {
  Serial.println("\n[TEST 4] Write/Read Time");

  // เขียนเวลาทดสอบ: 2026-06-22 12:00:00
  DateTime writeTime(2026, 6, 22, 12, 0, 0);
  rtc.adjust(writeTime);
  delay(100);

  // อ่านกลับ
  DateTime readTime = rtc.now();

  Serial.printf("  เขียน: %04d-%02d-%02d %02d:%02d:%02d\n",
    writeTime.year(), writeTime.month(), writeTime.day(),
    writeTime.hour(), writeTime.minute(), writeTime.second());
  Serial.printf("  อ่าน:  %04d-%02d-%02d %02d:%02d:%02d\n",
    readTime.year(), readTime.month(), readTime.day(),
    readTime.hour(), readTime.minute(), readTime.second());

  // ตรวจ year sanity ก่อน
  if (readTime.year() < 2024 || readTime.year() > 2035) {
    Serial.printf("  x FAIL: ปีผิดปกติ year=%u — น่าจะเป็น I2C corruption\n",
                  readTime.year());
    return;
  }

  // เปรียบเทียบค่า
  if (readTime.year()   == writeTime.year()   &&
      readTime.month()  == writeTime.month()  &&
      readTime.day()    == writeTime.day()    &&
      readTime.hour()   == writeTime.hour()   &&
      readTime.minute() == writeTime.minute()) {
    Serial.println("  + PASS: เวลาตรงกัน");
  } else {
    Serial.println("  x FAIL: เวลาไม่ตรง — IC อาจเสีย");
  }
}

// ── ทดสอบ 5: เวลาเดิน ──────────────────────────────────
void testTimeTick() {
  Serial.println("\n[TEST 5] Time Tick — รอ 3 วินาที ดูว่าเดินไหม");
  DateTime t1 = rtc.now();
  delay(3000);
  DateTime t2 = rtc.now();

  uint32_t diff = t2.unixtime() - t1.unixtime();
  Serial.printf("  t1 = %02d:%02d:%02d\n", t1.hour(), t1.minute(), t1.second());
  Serial.printf("  t2 = %02d:%02d:%02d\n", t2.hour(), t2.minute(), t2.second());
  Serial.printf("  diff = %lu วินาที\n", (unsigned long)diff);

  if (diff >= 2 && diff <= 4) {
    Serial.println("  + PASS: นาฬิกาเดินปกติ");
  } else if (diff == 0) {
    Serial.println("  x FAIL: นาฬิกาไม่เดิน — IC เสียหรือ oscillator มีปัญหา");
  } else {
    Serial.printf("  ! WARN: เดินผิดปกติ diff=%lu s (ควรได้ ~3)\n",
                  (unsigned long)diff);
  }
}

// ── ทดสอบ 6: I2C stability (อ่าน 10 ครั้ง) ────────────────
void testI2CStability() {
  Serial.println("\n[TEST 6] I2C Stability — อ่าน 10 ครั้ง ดูว่ามี corruption ไหม");
  int pass = 0, fail = 0;
  for (int i = 0; i < 10; i++) {
    delay(200);
    DateTime t = rtc.now();
    if (t.year() >= 2024 && t.year() <= 2035) {
      pass++;
      Serial.printf("  [%2d] OK  %04d-%02d-%02d %02d:%02d:%02d\n",
        i+1, t.year(), t.month(), t.day(),
        t.hour(), t.minute(), t.second());
    } else {
      fail++;
      Serial.printf("  [%2d] ERR year=%u ← I2C collision / corruption\n",
        i+1, t.year());
    }
  }
  Serial.printf("  Result: %d/10 OK, %d/10 Error\n", pass, fail);
  if (fail == 0)     Serial.println("  + PASS: I2C stable");
  else if (fail <= 2) Serial.println("  ! WARN: มี collision บ้าง — ตรวจ I2C bus");
  else               Serial.println("  x FAIL: I2C ไม่เสถียร — ควรเปลี่ยน DS3231 module");
}

// ─────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("================================================");
  Serial.println("  DS3231 RTC Diagnostic Test");
  Serial.println("  SDA=GPIO44  SCL=GPIO43");
  Serial.println("================================================");

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

  // Test 1: I2C ping ก่อน ถ้าไม่ผ่านหยุดทุกอย่าง
  if (!testI2CPing()) {
    Serial.println("\n!! หยุด: ไม่พบ DS3231 บน I2C bus");
    return;
  }

  // Init RTClib
  if (!rtc.begin()) {
    Serial.println("!! rtc.begin() failed — ไม่สามารถใช้งาน DS3231");
    return;
  }

  testLostPower();
  testTemperature();
  testWriteRead();
  testTimeTick();
  testI2CStability();

  Serial.println("\n================================================");
  Serial.println("  Test Complete");
  Serial.println("================================================");
}

void loop() {
  // แสดงเวลาปัจจุบันทุก 1 วินาที
  DateTime now = rtc.now();
  if (now.year() >= 2024 && now.year() <= 2035) {
    Serial.printf("[RTC] %04d-%02d-%02d %02d:%02d:%02d | Temp=%.2f C\n",
      now.year(), now.month(), now.day(),
      now.hour(), now.minute(), now.second(),
      rtc.getTemperature());
  } else {
    Serial.printf("[RTC] INVALID year=%u — I2C error\n", now.year());
  }
  delay(1000);
}
